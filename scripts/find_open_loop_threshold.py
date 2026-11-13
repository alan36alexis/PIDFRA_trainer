#!/usr/bin/env python3
"""Find the minimum OPEN-loop duty cycle that makes the motor move.

This script talks to the ESP32 over UART using the same CLI commands used by
this project. It switches to OPEN mode, applies increasing duty cycles, and
measures whether the encoder angle reported by the board changes.

Example:
    python3 scripts/find_open_loop_threshold.py --port /dev/ttyUSB0
    python3 scripts/find_open_loop_threshold.py --port COM3 --step 1 --pause 0.4
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from typing import List, Optional

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial no está instalado. Ejecuta: python3 -m pip install pyserial"
    ) from exc

STATUS_RE = re.compile(r"POS=([-+]?\d+(?:\.\d+)?)")


def read_lines(ser: serial.Serial, timeout: float = 1.0) -> List[str]:
    """Read all available serial data for ``timeout`` seconds."""
    deadline = time.time() + timeout
    chunks: List[str] = []

    while time.time() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            if not chunk:
                break
            text = chunk.decode("utf-8", errors="ignore")
            chunks.append(text)
            if "\n" in text or "\r" in text:
                # Keep consuming until the buffer is drained or timeout hits.
                continue
        else:
            time.sleep(0.01)

    message = "".join(chunks)
    lines = [line.strip() for line in message.replace("\r", "\n").split("\n") if line.strip()]
    return lines


def send_cmd(ser: serial.Serial, cmd: str, timeout: float = 0.5) -> List[str]:
    ser.write((cmd + "\r\n").encode("utf-8"))
    time.sleep(0.05)
    lines = read_lines(ser, timeout=timeout)
    return lines


def query_status(ser: serial.Serial) -> Optional[float]:
    lines = send_cmd(ser, "STATUS", timeout=0.6)
    for line in reversed(lines):
        match = STATUS_RE.search(line)
        if match:
            return float(match.group(1))
    return None


def wait_for_ready(ser: serial.Serial, timeout: float = 5.0, fail_on_missing: bool = True) -> bool:
    deadline = time.time() + timeout
    last_lines: List[str] = []
    while time.time() < deadline:
        lines = read_lines(ser, timeout=0.25)
        if lines:
            last_lines.extend(lines)
            if any("READY" in line.upper() or "HELP" in line.upper() for line in lines):
                return True
    if not last_lines and fail_on_missing:
        print("No se recibió respuesta del motor en el puerto serie.", file=sys.stderr)
        raise SystemExit(1)
    return bool(last_lines)


def determine_threshold(port: str, step: int, pause: float, start_duty: int, max_duty: int, direction: str) -> dict:
    ser = serial.Serial(port, 115200, timeout=0.1)
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ready = wait_for_ready(ser, timeout=8.0, fail_on_missing=False)
        if not ready:
            print("Advertencia: no hubo banner inicial; continuaré con los comandos aunque el firmware no emita READY/HELP.", file=sys.stderr)

        # Ensure clean state before the test.
        send_cmd(ser, "STOP", timeout=0.3)
        send_cmd(ser, "MODE OPEN", timeout=0.3)
        time.sleep(0.2)

        initial_pos = query_status(ser)
        if initial_pos is None:
            raise RuntimeError(
                "No se pudo leer la posición inicial del encoder. Revisa que el firmware esté corriendo y que el puerto sea correcto."
            )

        # Sweep increasing duty values until the encoder begins to move.
        found = None
        previous = None
        for duty in range(start_duty, max_duty + 1, step):
            send_cmd(ser, f"DUTY {direction} {duty}", timeout=0.2)
            time.sleep(pause)
            current_pos = query_status(ser)
            if current_pos is None:
                print(f"Duty {duty}: no se pudo leer STATUS.", file=sys.stderr)
                continue

            delta = abs(current_pos - initial_pos)
            print(f"duty={duty:3d}  pos={current_pos:.2f}  delta={delta:.2f}")

            # Movement is considered valid when the encoder position changes.
            if delta >= 1.0:
                found = duty
                previous = duty - step if duty >= step else 0
                break

            initial_pos = current_pos

        if found is None:
            raise RuntimeError(f"No hubo movimiento con ningún duty hasta {max_duty}.")

        # Refine the threshold with smaller steps around the first move.
        low = previous if previous is not None else max(0, found - step)
        high = found
        best = low

        while low < high:
            mid = (low + high) // 2
            send_cmd(ser, f"DUTY {direction} {mid}", timeout=0.2)
            time.sleep(pause)
            pos_after = query_status(ser)
            if pos_after is None:
                raise RuntimeError(f"No se pudo leer STATUS para duty={mid}.")

            delta = abs(pos_after - initial_pos)
            if delta >= 1.0:
                best = mid
                high = mid
            else:
                low = mid + 1

        send_cmd(ser, "STOP", timeout=0.3)
        return {
            "threshold": best,
            "found_first_move_at": found,
            "step": step,
            "direction": direction,
        }
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Determina el duty cycle mínimo en modo OPEN que hace mover el motor."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Puerto serie del ESP32.")
    parser.add_argument("--step", type=int, default=5, help="Paso del barrido en duty (p. ej. 1 o 5).")
    parser.add_argument("--pause", type=float, default=0.35, help="Tiempo de espera entre cada prueba en segundos.")
    parser.add_argument("--start-duty", type=int, default=0, help="Duty mínimo desde el que empezar a probar (p. ej. 200).")
    parser.add_argument("--max-duty", type=int, default=255, help="Duty máximo a probar.")
    parser.add_argument("--direction", choices=["F", "R"], default="F", help="Dirección a probar.")
    args = parser.parse_args()

    if args.step <= 0:
        raise SystemExit("--step debe ser > 0")
    if args.start_duty < 0 or args.start_duty > 255:
        raise SystemExit("--start-duty debe estar entre 0 y 255")
    if args.max_duty < 0 or args.max_duty > 255:
        raise SystemExit("--max-duty debe estar entre 0 y 255")
    if args.start_duty > args.max_duty:
        raise SystemExit("--start-duty no puede ser mayor que --max-duty")

    print(f"Buscando umbral de movimiento en {args.port} ...")
    print(f"Dirección: {args.direction} | paso: {args.step} | pausa: {args.pause}s | inicio: {args.start_duty} | max-duty: {args.max_duty}")
    print("")

    try:
        result = determine_threshold(
            port=args.port,
            step=args.step,
            pause=args.pause,
            start_duty=args.start_duty,
            max_duty=args.max_duty,
            direction=args.direction,
        )
    except Exception as exc:  # pragma: no cover
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)

    print("")
    print("RESULTADO:")
    print(f"- El mínimo duty que hizo mover el motor fue: {result['threshold']} / 255")
    print(f"- El primer movimiento apareció en: {result['found_first_move_at']} / 255")
    print(f"- Dirección probada: {result['direction']}")
    print("- Recomendación: empieza con un valor ligeramente superior al umbral y aumenta gradualmente.")


if __name__ == "__main__":
    main()
