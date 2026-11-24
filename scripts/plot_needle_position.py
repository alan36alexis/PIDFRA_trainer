#!/usr/bin/env python3
"""Grafica en tiempo real la posicion angular de la aguja en un dial circular.

Se conecta por puerto serie al ESP32, desactiva los mensajes de diagnostico
``POS OUT=...`` (``LOG POSOUT OFF``), configura el periodo de muestreo
(``LOG RATE <ms>``) y activa el stream ``P,<counts>,<degrees>\\n``
(``LOG POS ON``).

Usa wxPython + Cairo por defecto (o GTK3 + Cairo / Matplotlib si se solicita
con ``--backend``).

Ejemplos:
    python3 scripts/plot_needle_position.py --port /dev/ttyUSB0
    python3 scripts/plot_needle_position.py --port /dev/ttyUSB0 --rate 20
    python3 scripts/plot_needle_position.py --demo
    python3 scripts/plot_needle_position.py --self-test
"""

from __future__ import annotations

import argparse
import collections
import math
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None  # type: ignore[assignment]


POS_LINE_RE = re.compile(r"^P,\s*([-+]?\d+),\s*([-+]?\d+(?:\.\d+)?)$")
TEL_POS_RE = re.compile(
    r"^TEL\s+([PC])\s+R=([-+]?\d+(?:\.\d+)?)\s+A=([-+]?\d+(?:\.\d+)?)"
)
PID_LINE_RE = re.compile(
    r"^PID\s+([VPC])\s+KP=([-+]?\d+(?:\.\d+)?)\s+KI=([-+]?\d+(?:\.\d+)?)\s+KD=([-+]?\d+(?:\.\d+)?)$"
)

PID_CODE_TO_MODE: Dict[str, str] = {"V": "SPEED", "P": "POS", "C": "CLOCK"}
PID_MODES: Tuple[str, ...] = ("SPEED", "POS", "CLOCK")
XY_WINDOWS_S: Tuple[float, ...] = (5.0, 15.0, 30.0, 60.0)
XY_MODE_COUNTS = "Counts (encoder)"
XY_MODE_DEG_SINE = "Grados Senoidal (180°·sin θ)"
XY_MODE_DEG_360 = "Grados (0°..360°)"
XY_MODE_DEG_180 = "Grados (-180°..+180°)"
XY_MODE_DEG_ACCUM = "Grados (acumulados)"
XY_Y_MODES: Tuple[str, ...] = (
    XY_MODE_COUNTS,
    XY_MODE_DEG_SINE,
    XY_MODE_DEG_360,
    XY_MODE_DEG_180,
    XY_MODE_DEG_ACCUM,
)
TERMINAL_QUICK_CMDS: Tuple[str, ...] = (
    "HELP",
    "STATUS",
    "PID GET",
    "POT GET",
    "NVS GET",
    "CAL READ",
    "LCD STATUS",
)

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
DEFAULT_RATE_MS = 20
DEFAULT_FPS = 50
DEFAULT_TRAIL = 40
DEFAULT_COUNTS_PER_REV = 3600
DEFAULT_XY_WINDOW_S = 15.0


@dataclass(frozen=True)
class ParsedMessage:
    """Mensaje parseado desde una linea del puerto serie."""

    kind: str
    counts: Optional[int] = None
    degrees: Optional[float] = None
    target_degrees: Optional[float] = None
    pid_mode: Optional[str] = None
    pid_gains: Optional[Tuple[float, float, float]] = None
    raw_text: str = ""


@dataclass
class TelemetrySnapshot:
    """Instantanea thread-safe del estado actual de la aguja."""

    counts: int = 0
    degrees: float = 0.0
    angle_mod_360: float = 0.0
    seconds_mod_60: float = 0.0
    counts_mod_rev: float = 0.0
    counts_per_rev: int = DEFAULT_COUNTS_PER_REV
    turns: float = 0.0
    deg_per_sec: float = 0.0
    rpm: float = 0.0
    sample_rate_hz: float = 0.0
    target_degrees: Optional[float] = None
    connected: bool = False
    has_data: bool = False
    status_text: str = "Esperando datos..."
    last_event: str = ""
    trail: List[float] = field(default_factory=list)
    xy_history: List[Tuple[float, int, float]] = field(default_factory=list)
    xy_elapsed_s: float = 0.0
    pid_gains: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    pid_revision: int = 0
    serial_log: List[str] = field(default_factory=list)
    serial_log_revision: int = 0
    log_raw_pos_stream: bool = False


def parse_serial_line(line: str) -> Optional[ParsedMessage]:
    """Parsea una linea de texto emitida por el firmware del ESP32."""
    text = line.strip()
    if not text:
        return None

    pos_match = POS_LINE_RE.match(text)
    if pos_match:
        return ParsedMessage(
            kind="P",
            counts=int(pos_match.group(1)),
            degrees=float(pos_match.group(2)),
            raw_text=text,
        )

    tel_match = TEL_POS_RE.match(text)
    if tel_match:
        return ParsedMessage(
            kind="TEL",
            degrees=float(tel_match.group(3)),
            target_degrees=float(tel_match.group(2)),
            raw_text=text,
        )

    pid_match = PID_LINE_RE.match(text)
    if pid_match:
        return ParsedMessage(
            kind="PID",
            pid_mode=PID_CODE_TO_MODE[pid_match.group(1)],
            pid_gains=(
                float(pid_match.group(2)),
                float(pid_match.group(3)),
                float(pid_match.group(4)),
            ),
            raw_text=text,
        )

    if text.startswith(("EVT ", "OK ", "ERR ")):
        return ParsedMessage(kind="EVENT", raw_text=text)

    return None


def to_canvas_angle_rad(
    degrees: float, clockwise: bool = True, zero_north: bool = True
) -> float:
    """Convierte grados del encoder a radianes en coordenadas de pantalla Cairo."""
    base = -math.pi / 2.0 if zero_north else 0.0
    sign = 1.0 if clockwise else -1.0
    return base + sign * math.radians(degrees)


def shortest_angle_delta_deg(target_deg: float, source_deg: float) -> float:
    """Devuelve la diferencia angular con signo en [-180, 180]."""
    return (target_deg - source_deg + 180.0) % 360.0 - 180.0


class TelemetryTracker:
    """Almacena y calcula metricas en tiempo real de forma thread-safe."""

    def __init__(
        self,
        trail_len: int = DEFAULT_TRAIL,
        counts_per_rev: int = DEFAULT_COUNTS_PER_REV,
        xy_maxlen: int = 3600,
    ) -> None:
        self._lock = threading.Lock()
        self._trail: Deque[float] = collections.deque(maxlen=max(2, trail_len))
        self._sample_times: Deque[float] = collections.deque(maxlen=32)
        self._xy_history: Deque[Tuple[float, int, float]] = collections.deque(
            maxlen=max(64, xy_maxlen)
        )
        self._xy_start_time = time.monotonic()
        self._counts_per_rev = max(1, counts_per_rev)
        self._counts = 0
        self._degrees = 0.0
        self._deg_per_sec = 0.0
        self._target_degrees: Optional[float] = None
        self._connected = False
        self._has_data = False
        self._status_text = "Desconectado"
        self._last_event = ""
        self._last_sample_time: Optional[float] = None
        self._last_sample_deg: Optional[float] = None
        self._pid_gains: Dict[str, Tuple[float, float, float]] = {
            "SPEED": (0.1200, 0.1000, 0.0020),
            "POS": (0.4000, 0.2000, 0.0150),
            "CLOCK": (1.5000, 1.0000, 0.0150),
        }
        self._pid_revision = 0
        self._serial_log: Deque[str] = collections.deque(maxlen=400)
        self._serial_log_revision = 0
        self._log_raw_pos_stream = False

    def set_connection_state(self, connected: bool, status_text: str) -> None:
        with self._lock:
            self._connected = connected
            self._status_text = status_text

    def set_target_degrees(self, target_deg: Optional[float]) -> None:
        with self._lock:
            self._target_degrees = target_deg

    def record_event(self, text: str) -> None:
        with self._lock:
            self._last_event = text

    def update_pid_gains(
        self, mode: str, kp: float, ki: float, kd: float
    ) -> None:
        key = mode.upper()
        if key not in self._pid_gains:
            return
        with self._lock:
            self._pid_gains[key] = (kp, ki, kd)
            self._pid_revision += 1

    def clear_xy_history(self) -> None:
        with self._lock:
            self._xy_history.clear()
            self._xy_start_time = time.monotonic()

    def set_log_raw_pos_stream(self, enabled: bool) -> None:
        with self._lock:
            self._log_raw_pos_stream = bool(enabled)

    def append_serial_log(self, line: str, is_pos_stream: bool = False) -> None:
        with self._lock:
            if is_pos_stream and not self._log_raw_pos_stream:
                return
            self._serial_log.append(line)
            self._serial_log_revision += 1

    def clear_serial_log(self) -> None:
        with self._lock:
            self._serial_log.clear()
            self._serial_log_revision += 1

    def sync_target_from_command(self, cmd: str) -> None:
        parts = cmd.strip().upper().split()
        if not parts:
            return
        op = parts[0]
        if op in ("ANG", "A") and len(parts) == 2:
            try:
                self.set_target_degrees(float(parts[1]))
            except ValueError:
                pass
        elif op in ("POS", "P") and len(parts) == 2:
            try:
                cnt = float(parts[1])
                self.set_target_degrees(cnt * 360.0 / self._counts_per_rev)
            except ValueError:
                pass
        elif op == "CAL" and len(parts) >= 2 and parts[1] in ("START", "ZERO", "RESET"):
            self.clear_motion_history()

    def clear_motion_history(self) -> None:
        with self._lock:
            self._trail.clear()
            self._xy_history.clear()
            self._xy_start_time = time.monotonic()
            self._deg_per_sec = 0.0
            self._target_degrees = None
            self._last_sample_time = None
            self._last_sample_deg = None

    def update_position(
        self,
        counts: Optional[int],
        degrees: float,
        timestamp: Optional[float] = None,
        target_degrees: Optional[float] = None,
    ) -> None:
        now = time.monotonic() if timestamp is None else timestamp
        with self._lock:
            if counts is not None:
                self._counts = counts
            self._degrees = degrees
            self._has_data = True
            self._trail.append(degrees)
            self._sample_times.append(now)
            if now < self._xy_start_time:
                self._xy_start_time = now
            self._xy_history.append(
                (now - self._xy_start_time, self._counts, self._degrees)
            )

            if target_degrees is not None:
                self._target_degrees = target_degrees

            if self._last_sample_time is not None and self._last_sample_deg is not None:
                dt = now - self._last_sample_time
                if dt > 1e-4:
                    inst_dps = (degrees - self._last_sample_deg) / dt
                    self._deg_per_sec = 0.7 * self._deg_per_sec + 0.3 * inst_dps

            self._last_sample_time = now
            self._last_sample_deg = degrees

    def snapshot(self) -> TelemetrySnapshot:
        now = time.monotonic()
        with self._lock:
            hz = 0.0
            if len(self._sample_times) >= 2:
                span = self._sample_times[-1] - self._sample_times[0]
                if span > 1e-4:
                    hz = (len(self._sample_times) - 1) / span
            if (
                self._last_sample_time is not None
                and (now - self._last_sample_time) > 0.35
            ):
                hz = 0.0
                deg_per_sec = 0.0
            else:
                deg_per_sec = self._deg_per_sec

            angle_mod = self._degrees % 360.0
            return TelemetrySnapshot(
                counts=self._counts,
                degrees=self._degrees,
                angle_mod_360=angle_mod,
                seconds_mod_60=angle_mod / 6.0,
                counts_mod_rev=(angle_mod / 360.0) * self._counts_per_rev,
                counts_per_rev=self._counts_per_rev,
                turns=self._degrees / 360.0,
                deg_per_sec=deg_per_sec,
                rpm=deg_per_sec / 6.0,
                sample_rate_hz=hz,
                target_degrees=self._target_degrees,
                connected=self._connected,
                has_data=self._has_data,
                status_text=self._status_text,
                last_event=self._last_event,
                trail=list(self._trail),
                xy_history=list(self._xy_history),
                xy_elapsed_s=max(0.0, now - self._xy_start_time),
                pid_gains=dict(self._pid_gains),
                pid_revision=self._pid_revision,
                serial_log=list(self._serial_log),
                serial_log_revision=self._serial_log_revision,
                log_raw_pos_stream=self._log_raw_pos_stream,
            )


class SerialReader:
    """Hilo dedicado para leer el stream serie y enviar comandos CLI."""

    def __init__(
        self,
        port: str,
        baud: int,
        rate_ms: int,
        tracker: TelemetryTracker,
        init_stream: bool = True,
    ) -> None:
        self.port = port
        self.baud = baud
        self.rate_ms = max(10, rate_ms)
        self.tracker = tracker
        self.init_stream = init_stream
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._serial: Optional["serial.Serial"] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if serial is None:
            raise RuntimeError(
                "pyserial no esta instalado. Ejecuta: python3 -m pip install pyserial"
            )
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send_command(self, cmd: str) -> bool:
        cmd_clean = cmd.strip()
        if not cmd_clean:
            return False
        with self._write_lock:
            if self._serial is None or not self._serial.is_open:
                self.tracker.append_serial_log(f"ERR TX (sin puerto): {cmd_clean}")
                return False
            try:
                self._serial.write((cmd_clean + "\r\n").encode("utf-8"))
                self._serial.flush()
                self.tracker.append_serial_log(f"TX > {cmd_clean}")
                self.tracker.sync_target_from_command(cmd_clean)
                return True
            except Exception as exc:
                self.tracker.record_event(f"ERR TX: {exc}")
                self.tracker.append_serial_log(f"ERR TX: {exc}")
                return False

    def configure_stream(self) -> None:
        if not self.init_stream:
            return
        self.send_command("LOG POSOUT OFF")
        time.sleep(0.03)
        self.send_command(f"LOG RATE {self.rate_ms}")
        time.sleep(0.03)
        self.send_command("LOG POS ON")

    def stop(self) -> None:
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        if self.init_stream:
            self.send_command("LOG POS OFF")
            time.sleep(0.05)
        with self._write_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _consume_line(self, line: str) -> None:
        text = line.strip()
        if not text:
            return
        parsed = parse_serial_line(text)
        is_pos = parsed is not None and parsed.kind == "P"
        self.tracker.append_serial_log(f"RX < {text}", is_pos_stream=is_pos)
        if parsed is None:
            return
        if parsed.kind == "P" and parsed.degrees is not None:
            self.tracker.update_position(parsed.counts, parsed.degrees)
        elif parsed.kind == "TEL" and parsed.degrees is not None:
            self.tracker.update_position(
                None, parsed.degrees, target_degrees=parsed.target_degrees
            )
        elif (
            parsed.kind == "PID"
            and parsed.pid_mode is not None
            and parsed.pid_gains is not None
        ):
            self.tracker.update_pid_gains(parsed.pid_mode, *parsed.pid_gains)
            self.tracker.record_event(parsed.raw_text)
        elif parsed.kind == "EVENT":
            self.tracker.record_event(parsed.raw_text)
            if "READY" in parsed.raw_text.upper() and self.init_stream:
                self.configure_stream()

    def _run(self) -> None:
        assert serial is not None
        self.tracker.set_connection_state(False, f"Conectando a {self.port}...")
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as exc:
            self.tracker.set_connection_state(
                False, f"Error abriendo {self.port}: {exc}"
            )
            return

        with self._write_lock:
            self._serial = ser

        time.sleep(0.2)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        self.tracker.set_connection_state(
            True, f"Conectado: {self.port} @ {self.baud} baud"
        )
        self.configure_stream()

        rx_buffer = bytearray()
        while not self._stop_event.is_set():
            try:
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting > 0 else 1)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.tracker.set_connection_state(False, f"Error UART: {exc}")
                break

            if not chunk:
                continue

            rx_buffer.extend(chunk)
            while True:
                nl_idx = rx_buffer.find(b"\n")
                cr_idx = rx_buffer.find(b"\r")
                if nl_idx == -1 and cr_idx == -1:
                    break
                if nl_idx == -1:
                    split_idx = cr_idx
                elif cr_idx == -1:
                    split_idx = nl_idx
                else:
                    split_idx = min(nl_idx, cr_idx)

                raw_line = bytes(rx_buffer[:split_idx])
                del rx_buffer[: split_idx + 1]
                if raw_line:
                    self._consume_line(raw_line.decode("utf-8", errors="replace"))


class DemoRunner:
    """Genera movimiento simulado en modo --demo sin hardware conectado."""

    def __init__(
        self, tracker: TelemetryTracker, rate_ms: int, counts_per_rev: int
    ) -> None:
        self.tracker = tracker
        self.rate_s = max(10, rate_ms) / 1000.0
        self.counts_per_rev = max(1, counts_per_rev)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._target_deg: Optional[float] = None
        self._manual_mode = "CLOCK"
        self._deg = 0.0

    def start(self) -> None:
        self.tracker.set_connection_state(True, "Modo DEMO (sin puerto serie)")
        self.tracker.record_event("DEMO ACTIVO (1 paso/s + barrido)")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def send_command(self, cmd: str) -> bool:
        cmd_clean = cmd.strip()
        if not cmd_clean:
            return False
        upper = cmd_clean.upper()
        self.tracker.append_serial_log(f"TX > {cmd_clean}")
        with self._lock:
            if upper == "STOP":
                self._manual_mode = "STOP"
                self.tracker.record_event("OK STOP (DEMO)")
                self.tracker.append_serial_log("RX < OK STOP")
            elif upper == "CAL START":
                self._deg = 0.0
                self._target_deg = None
                self.tracker.clear_motion_history()
                self.tracker.update_position(0, 0.0)
                self.tracker.record_event("OK CAL START (DEMO)")
                self.tracker.append_serial_log("RX < OK CAL START")
            elif upper in ("MODE CLOCK", "MODE POS", "MODE SPEED", "MODE OPEN"):
                self._manual_mode = upper.split()[1]
                if self._manual_mode != "POS":
                    self._target_deg = None
                self.tracker.record_event(f"OK {upper} (DEMO)")
                self.tracker.append_serial_log(f"RX < OK {upper}")
            elif upper.startswith("ANG "):
                try:
                    self._target_deg = float(upper.split()[1])
                    self._manual_mode = "POS"
                    self.tracker.set_target_degrees(self._target_deg)
                    self.tracker.record_event(f"OK ANG {self._target_deg:.1f} (DEMO)")
                    self.tracker.append_serial_log("RX < OK ANG")
                except (IndexError, ValueError):
                    self.tracker.append_serial_log("RX < ERR ANG")
                    return False
            elif upper.startswith("POS "):
                try:
                    target_cnt = float(upper.split()[1])
                    self._target_deg = target_cnt * 360.0 / self.counts_per_rev
                    self._manual_mode = "POS"
                    self.tracker.set_target_degrees(self._target_deg)
                    self.tracker.record_event(f"OK POS {int(target_cnt)} (DEMO)")
                    self.tracker.append_serial_log("RX < OK POS")
                except (IndexError, ValueError):
                    self.tracker.append_serial_log("RX < ERR POS")
                    return False
            elif upper == "PID GET":
                snap = self.tracker.snapshot()
                for code, mode in PID_CODE_TO_MODE.items():
                    kp, ki, kd = snap.pid_gains[mode]
                    self.tracker.update_pid_gains(mode, kp, ki, kd)
                    self.tracker.append_serial_log(
                        f"RX < PID {code} KP={kp:.4f} KI={ki:.4f} KD={kd:.4f}"
                    )
                self.tracker.record_event("OK PID GET (DEMO)")
            elif upper == "PID DEFAULT":
                self.tracker.update_pid_gains("SPEED", 0.1200, 0.1000, 0.0020)
                self.tracker.update_pid_gains("POS", 0.4000, 0.2000, 0.0150)
                self.tracker.update_pid_gains("CLOCK", 1.5000, 1.0000, 0.0150)
                self.tracker.record_event("OK PID DEFAULT (DEMO)")
                self.tracker.append_serial_log("RX < OK PID DEFAULT")
            elif upper.startswith("PID "):
                parts = upper.split()
                if len(parts) == 4 and parts[1] in PID_MODES:
                    try:
                        val = float(parts[3])
                    except ValueError:
                        self.tracker.append_serial_log("RX < ERR PID")
                        return False
                    snap = self.tracker.snapshot()
                    kp, ki, kd = snap.pid_gains[parts[1]]
                    if parts[2] == "KP":
                        kp = val
                    elif parts[2] == "KI":
                        ki = val
                    elif parts[2] == "KD":
                        kd = val
                    else:
                        self.tracker.append_serial_log("RX < ERR PID")
                        return False
                    self.tracker.update_pid_gains(parts[1], kp, ki, kd)
                    self.tracker.record_event(f"OK {upper} (DEMO)")
                    self.tracker.append_serial_log(f"RX < OK {upper}")
            elif upper == "STATUS":
                snap = self.tracker.snapshot()
                self.tracker.append_serial_log(
                    f"RX < STAT S=C RUN=1 FLT=0 POS={snap.degrees:.2f} REF=0.00"
                )
            elif upper in ("HELP", "?"):
                self.tracker.append_serial_log(
                    "RX < CMD HELP|MODE SPEED/POS/OPEN/CLOCK|ANG <deg>|POS <cnt>|PID GET|STOP|STATUS"
                )
            else:
                self.tracker.record_event(f"OK {upper} (DEMO)")
                self.tracker.append_serial_log(f"RX < OK {upper}")
        return True

    def _run(self) -> None:
        start = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            elapsed = now - start
            with self._lock:
                if self._manual_mode == "POS" and self._target_deg is not None:
                    err = self._target_deg - self._deg
                    self._deg += err * min(1.0, self.rate_s * 5.0)
                    target = self._target_deg
                elif self._manual_mode == "CLOCK":
                    step_angle = 6.0 * math.floor(elapsed)
                    self._deg += (step_angle - self._deg) * min(1.0, self.rate_s * 12.0)
                    target = step_angle
                else:
                    target = self._target_deg
                deg = self._deg

            counts = round(deg * self.counts_per_rev / 360.0)
            self.tracker.update_position(counts, deg, timestamp=now, target_degrees=target)
            self.tracker.append_serial_log(
                f"RX < P,{counts},{deg:.2f}", is_pos_stream=True
            )
            time.sleep(self.rate_s)


@dataclass(frozen=True)
class CircularScaleSpec:
    """Especificacion visual de una escala circular concentrica."""

    name: str
    ring_frac: float
    inner_frac: float
    outer_frac: float
    label_frac: float
    major_color: Tuple[float, float, float]
    zero_color: Tuple[float, float, float]
    font_scale: float
    label_fn: Callable[[int], str]


def _draw_scale_ring(
    ctx: "cairo.Context",  # type: ignore[name-defined]
    cx: float,
    cy: float,
    radius: float,
    spec: CircularScaleSpec,
    clockwise: bool,
    zero_north: bool,
) -> None:
    """Dibuja una escala circular de 60 divisiones con marcas y etiquetas."""
    ctx.arc(cx, cy, radius * spec.ring_frac, 0, 2.0 * math.pi)
    ctx.set_line_width(1.2)
    ctx.set_source_rgba(*spec.major_color, 0.35)
    ctx.stroke()

    ctx.select_font_face("Sans", 0, 1)
    ctx.set_font_size(max(8.5, radius * spec.font_scale))
    for div in range(60):
        deg = div * 6.0
        theta = to_canvas_angle_rad(deg, clockwise=clockwise, zero_north=zero_north)
        is_major = (div % 5) == 0
        inner_r = radius * (
            spec.inner_frac if is_major else (spec.inner_frac + spec.outer_frac) * 0.5
        )
        outer_r = radius * spec.outer_frac

        ctx.move_to(cx + math.cos(theta) * inner_r, cy + math.sin(theta) * inner_r)
        ctx.line_to(cx + math.cos(theta) * outer_r, cy + math.sin(theta) * outer_r)
        ctx.set_line_width(2.0 if is_major else 0.9)
        if div == 0:
            ctx.set_source_rgb(*spec.zero_color)
        elif is_major:
            ctx.set_source_rgba(*spec.major_color, 0.95)
        else:
            ctx.set_source_rgba(*spec.major_color, 0.45)
        ctx.stroke()

        if is_major:
            label = spec.label_fn(div)
            ext = ctx.text_extents(label)
            label_r = radius * spec.label_frac
            lx = cx + math.cos(theta) * label_r - (ext.width * 0.5 + ext.x_bearing)
            ly = cy + math.sin(theta) * label_r - (ext.height * 0.5 + ext.y_bearing)
            ctx.move_to(lx, ly)
            ctx.set_source_rgb(*(spec.zero_color if div == 0 else spec.major_color))
            ctx.show_text(label)


def draw_circular_dial(
    ctx: "cairo.Context",  # type: ignore[name-defined]
    width: float,
    height: float,
    snap: TelemetrySnapshot,
    clockwise: bool = True,
    zero_north: bool = True,
    x0: float = 0.0,
    y0: float = 0.0,
) -> None:
    """Renderiza el grafico circular con escalas de counts, grados y segundos."""
    ctx.set_source_rgb(0.071, 0.086, 0.122)
    ctx.rectangle(x0, y0, width, height)
    ctx.fill()

    hud_height = 88.0
    avail_h = max(140.0, height - hud_height - 14.0)
    cx = x0 + width * 0.5
    cy = y0 + avail_h * 0.5 + 16.0
    radius = max(48.0, min(width * 0.36, avail_h * 0.38))

    # Esfera principal
    ctx.arc(cx, cy, radius, 0, 2.0 * math.pi)
    ctx.set_source_rgb(0.106, 0.133, 0.188)
    ctx.fill_preserve()
    ctx.set_line_width(2.5)
    ctx.set_source_rgb(0.220, 0.286, 0.400)
    ctx.stroke()

    cpr = max(1, snap.counts_per_rev)
    scales = (
        CircularScaleSpec(
            name=f"Counts (0..{cpr})",
            ring_frac=1.00,
            inner_frac=0.93,
            outer_frac=1.00,
            label_frac=1.10,
            major_color=(0.98, 0.75, 0.14),
            zero_color=(1.00, 0.85, 0.35),
            font_scale=0.068,
            label_fn=lambda div, _cpr=cpr: f"{int(round(div * _cpr / 60.0))}",
        ),
        CircularScaleSpec(
            name="Grados (0..360°)",
            ring_frac=0.87,
            inner_frac=0.80,
            outer_frac=0.87,
            label_frac=0.71,
            major_color=(0.82, 0.88, 0.96),
            zero_color=(0.22, 0.74, 0.97),
            font_scale=0.074,
            label_fn=lambda div: f"{div * 6}°",
        ),
        CircularScaleSpec(
            name="Segundos (0..60s)",
            ring_frac=0.60,
            inner_frac=0.54,
            outer_frac=0.60,
            label_frac=0.44,
            major_color=(0.20, 0.83, 0.60),
            zero_color=(0.45, 0.95, 0.75),
            font_scale=0.068,
            label_fn=lambda div: f"{div}s",
        ),
    )

    for spec in scales:
        _draw_scale_ring(
            ctx, cx, cy, radius, spec, clockwise=clockwise, zero_north=zero_north
        )

    # Leyenda compacta de las 3 escalas en la esquina superior izquierda
    ctx.select_font_face("Sans", 0, 1)
    ctx.set_font_size(11.0)
    for idx, spec in enumerate(scales):
        ly = y0 + 18.0 + idx * 16.0
        ctx.arc(x0 + 18.0, ly - 4.0, 4.0, 0, 2.0 * math.pi)
        ctx.set_source_rgb(*spec.major_color)
        ctx.fill()
        ctx.move_to(x0 + 27.0, ly)
        ctx.show_text(spec.name)

    # Estela de posiciones recientes (sin saltos al cruzar 0/360)
    if len(snap.trail) >= 2:
        trail_r = radius * 0.78
        n_pts = len(snap.trail)
        for i in range(1, n_pts):
            if abs(shortest_angle_delta_deg(snap.trail[i], snap.trail[i - 1])) > 90.0:
                continue
            alpha = (i / n_pts) ** 1.6 * 0.75
            a0 = to_canvas_angle_rad(
                snap.trail[i - 1], clockwise=clockwise, zero_north=zero_north
            )
            a1 = to_canvas_angle_rad(
                snap.trail[i], clockwise=clockwise, zero_north=zero_north
            )
            ctx.move_to(cx + math.cos(a0) * trail_r, cy + math.sin(a0) * trail_r)
            ctx.line_to(cx + math.cos(a1) * trail_r, cy + math.sin(a1) * trail_r)
            ctx.set_line_width(3.0)
            ctx.set_source_rgba(0.22, 0.74, 0.97, alpha)
            ctx.stroke()

    # Marcador de referencia/objetivo (si existe)
    if snap.target_degrees is not None:
        t_theta = to_canvas_angle_rad(
            snap.target_degrees, clockwise=clockwise, zero_north=zero_north
        )
        ctx.save()
        ctx.set_dash([5.0, 5.0])
        ctx.move_to(cx, cy)
        ctx.line_to(
            cx + math.cos(t_theta) * radius * 0.93,
            cy + math.sin(t_theta) * radius * 0.93,
        )
        ctx.set_line_width(1.8)
        ctx.set_source_rgba(0.98, 0.75, 0.14, 0.85)
        ctx.stroke()
        ctx.restore()

        tip_r = radius * 0.94
        base_r = radius * 1.01
        perp = t_theta + math.pi * 0.5
        ctx.move_to(cx + math.cos(t_theta) * tip_r, cy + math.sin(t_theta) * tip_r)
        ctx.line_to(
            cx + math.cos(t_theta) * base_r + math.cos(perp) * 5.5,
            cy + math.sin(t_theta) * base_r + math.sin(perp) * 5.5,
        )
        ctx.line_to(
            cx + math.cos(t_theta) * base_r - math.cos(perp) * 5.5,
            cy + math.sin(t_theta) * base_r - math.sin(perp) * 5.5,
        )
        ctx.close_path()
        ctx.set_source_rgb(0.98, 0.75, 0.14)
        ctx.fill()

    # Aguja principal cruzando las 3 escalas
    needle_theta = to_canvas_angle_rad(
        snap.degrees, clockwise=clockwise, zero_north=zero_north
    )
    perp_theta = needle_theta + math.pi * 0.5
    tip_r = radius * 0.92
    tail_r = radius * 0.16
    half_w = max(3.5, radius * 0.028)

    ctx.move_to(cx + math.cos(needle_theta) * tip_r, cy + math.sin(needle_theta) * tip_r)
    ctx.line_to(
        cx + math.cos(perp_theta) * half_w, cy + math.sin(perp_theta) * half_w
    )
    ctx.line_to(
        cx - math.cos(needle_theta) * tail_r, cy - math.sin(needle_theta) * tail_r
    )
    ctx.line_to(
        cx - math.cos(perp_theta) * half_w, cy - math.sin(perp_theta) * half_w
    )
    ctx.close_path()
    ctx.set_source_rgb(0.98, 0.36, 0.24)
    ctx.fill_preserve()
    ctx.set_line_width(1.2)
    ctx.set_source_rgb(1.0, 0.85, 0.80)
    ctx.stroke()

    # Pivote central
    ctx.arc(cx, cy, max(5.5, radius * 0.048), 0, 2.0 * math.pi)
    ctx.set_source_rgb(0.88, 0.92, 0.97)
    ctx.fill_preserve()
    ctx.set_line_width(2.0)
    ctx.set_source_rgb(0.18, 0.22, 0.30)
    ctx.stroke()

    # Panel inferior de telemetria numerica
    panel_x = x0 + 10.0
    panel_w = max(100.0, width - 20.0)
    panel_y = y0 + height - hud_height + 4.0
    ctx.set_source_rgb(0.10, 0.125, 0.175)
    ctx.rectangle(panel_x, panel_y, panel_w, hud_height - 12.0)
    ctx.fill_preserve()
    ctx.set_line_width(1.0)
    ctx.set_source_rgb(0.24, 0.30, 0.42)
    ctx.stroke()

    base_font = min(13.0, max(9.5, width / 56.0))
    line1 = (
        f"Ang: {snap.angle_mod_360:6.2f}° ({snap.degrees:7.1f}°) | "
        f"Seg: {snap.seconds_mod_60:5.2f}s | Cnt: {snap.counts:d} | "
        f"Rev: {snap.turns:+.2f}"
    )
    target_str = (
        f"{snap.target_degrees:.2f}°" if snap.target_degrees is not None else "--"
    )
    line2 = (
        f"Vel: {snap.rpm:+6.2f} RPM ({snap.deg_per_sec:+6.1f} °/s) | "
        f"Ref: {target_str} | {snap.sample_rate_hz:4.1f} Hz"
    )
    event_str = snap.last_event if snap.last_event else snap.status_text

    ctx.set_font_size(base_font)
    ctx.set_source_rgb(0.92, 0.95, 0.98)
    ctx.move_to(panel_x + 12.0, panel_y + 23.0)
    ctx.show_text(line1)

    ctx.set_font_size(max(9.0, base_font - 1.0))
    ctx.set_source_rgb(0.68, 0.78, 0.90)
    ctx.move_to(panel_x + 12.0, panel_y + 45.0)
    ctx.show_text(line2)

    ctx.set_font_size(max(8.5, base_font - 1.5))
    ctx.set_source_rgb(0.45, 0.85, 0.68) if snap.connected else ctx.set_source_rgb(
        0.98, 0.65, 0.35
    )
    ctx.move_to(panel_x + 12.0, panel_y + 66.0)
    ctx.show_text(event_str)


def _transform_xy_value(counts: float, degrees: float, y_mode: str) -> float:
    """Convierte una muestra (counts, degrees) a la escala Y seleccionada."""
    if y_mode == XY_MODE_DEG_SINE:
        return 180.0 * math.sin(math.radians(degrees))
    if y_mode == XY_MODE_DEG_360:
        return degrees % 360.0
    if y_mode == XY_MODE_DEG_180:
        return ((degrees + 180.0) % 360.0) - 180.0
    if y_mode == XY_MODE_DEG_ACCUM:
        return degrees
    return float(counts)


def _xy_bounds(
    y_mode: str, values: List[float], fallback: float
) -> Tuple[float, float, float, float]:
    """Calcula (y_min, y_max, raw_min, raw_max) segun la escala Y elegida."""
    vals = values or [fallback]
    raw_min = float(min(vals))
    raw_max = float(max(vals))
    if y_mode in (XY_MODE_DEG_SINE, XY_MODE_DEG_180):
        return -180.0, 180.0, raw_min, raw_max
    if y_mode == XY_MODE_DEG_360:
        return 0.0, 360.0, raw_min, raw_max
    min_span = 100.0 if y_mode == XY_MODE_COUNTS else 90.0
    span = raw_max - raw_min
    if span < min_span:
        mid = 0.5 * (raw_min + raw_max)
        return mid - 0.5 * min_span, mid + 0.5 * min_span, raw_min, raw_max
    pad_y = span * 0.12
    return raw_min - pad_y, raw_max + pad_y, raw_min, raw_max


def draw_cartesian_xy(
    ctx: "cairo.Context",  # type: ignore[name-defined]
    x0: float,
    y0: float,
    width: float,
    height: float,
    snap: TelemetrySnapshot,
    time_window_s: float = DEFAULT_XY_WINDOW_S,
    y_mode: str = XY_MODE_COUNTS,
) -> None:
    """Dibuja la grafica cartesiana X-Y (Tiempo [s] vs Counts o Grados)."""
    ctx.set_source_rgb(0.071, 0.086, 0.122)
    ctx.rectangle(x0, y0, width, height)
    ctx.fill()

    card_x = x0 + 8.0
    card_y = y0 + 10.0
    card_w = max(160.0, width - 18.0)
    card_h = max(140.0, height - 20.0)
    ctx.set_source_rgb(0.095, 0.118, 0.168)
    ctx.rectangle(card_x, card_y, card_w, card_h)
    ctx.fill_preserve()
    ctx.set_line_width(1.2)
    ctx.set_source_rgb(0.24, 0.30, 0.42)
    ctx.stroke()

    pad_l = 66.0
    pad_r = 22.0
    pad_t = 54.0
    pad_b = 46.0
    px = card_x + pad_l
    py = card_y + pad_t
    pw = max(60.0, card_w - pad_l - pad_r)
    ph = max(60.0, card_h - pad_t - pad_b)

    window_s = max(1.0, float(time_window_s))
    t_max = max(window_s, snap.xy_elapsed_s)
    t_min = t_max - window_s

    visible: List[Tuple[float, float]] = []
    prior_pt: Optional[Tuple[float, float]] = None
    for t_s, cnt, deg in snap.xy_history:
        y_val = _transform_xy_value(float(cnt), deg, y_mode)
        if t_s < t_min:
            prior_pt = (t_s, y_val)
        elif t_s <= t_max:
            visible.append((t_s, y_val))
    if prior_pt is not None:
        visible.insert(0, prior_pt)

    cur_val = _transform_xy_value(float(snap.counts), snap.degrees, y_mode)
    y_min, y_max, raw_min, raw_max = _xy_bounds(
        y_mode, [val for _, val in visible], cur_val
    )
    is_counts = y_mode == XY_MODE_COUNTS

    # Fondo interior del area de grafica
    ctx.set_source_rgb(0.068, 0.084, 0.120)
    ctx.rectangle(px, py, pw, ph)
    ctx.fill()

    # Titulo y estadisticas
    ctx.select_font_face("Sans", 0, 1)
    ctx.set_font_size(13.0)
    ctx.set_source_rgb(0.90, 0.94, 0.98)
    ctx.move_to(card_x + 16.0, card_y + 24.0)
    ctx.show_text(f"Gráfica X-Y ({y_mode}) vs Tiempo (s)")

    def _fmt_val(val: float) -> str:
        return f"{int(round(val))} cnt" if is_counts else f"{val:.1f}°"

    sp_txt = ""
    if snap.target_degrees is not None:
        sp_cnt = snap.target_degrees * snap.counts_per_rev / 360.0
        sp_val = _transform_xy_value(sp_cnt, snap.target_degrees, y_mode)
        sp_txt = f" | SP: {_fmt_val(sp_val)}"
    stats_txt = (
        f"Actual: {_fmt_val(cur_val)} | Min: {_fmt_val(raw_min)} | "
        f"Max: {_fmt_val(raw_max)} | Ventana: {window_s:.0f}s{sp_txt}"
    )
    ctx.set_font_size(11.0)
    ctx.set_source_rgb(0.65, 0.77, 0.90)
    ctx.move_to(card_x + 16.0, card_y + 42.0)
    ctx.show_text(stats_txt)

    # Grilla y marcas en X (tiempo) e Y (counts o grados)
    ctx.set_font_size(10.5)
    for i in range(5):
        frac = i / 4.0
        gy = py + ph - frac * ph
        ctx.move_to(px, gy)
        ctx.line_to(px + pw, gy)
        ctx.set_line_width(1.0)
        ctx.set_source_rgba(0.32, 0.42, 0.58, 0.35)
        ctx.stroke()

        y_tick = y_min + frac * (y_max - y_min)
        y_lbl = f"{int(round(y_tick))}" if is_counts else f"{y_tick:.0f}°"
        ext = ctx.text_extents(y_lbl)
        ctx.set_source_rgb(0.75, 0.83, 0.92)
        ctx.move_to(px - 8.0 - ext.width - ext.x_bearing, gy + ext.height * 0.4)
        ctx.show_text(y_lbl)

        gx = px + frac * pw
        ctx.move_to(gx, py)
        ctx.line_to(gx, py + ph)
        ctx.set_line_width(1.0)
        ctx.set_source_rgba(0.32, 0.42, 0.58, 0.35)
        ctx.stroke()

        t_val = t_min + frac * window_s
        x_lbl = f"{t_val:.1f}s"
        ext_x = ctx.text_extents(x_lbl)
        ctx.set_source_rgb(0.75, 0.83, 0.92)
        ctx.move_to(gx - ext_x.width * 0.5 - ext_x.x_bearing, py + ph + 16.0)
        ctx.show_text(x_lbl)

    # Etiquetas de ejes
    ctx.set_font_size(11.0)
    ctx.set_source_rgb(0.98, 0.75, 0.14) if is_counts else ctx.set_source_rgb(
        0.20, 0.83, 0.60
    )
    ctx.move_to(card_x + 10.0, py + 12.0)
    ctx.show_text("cnt" if is_counts else "deg (°)")
    ctx.set_source_rgb(0.65, 0.77, 0.90)
    lbl_axis_x = "Tiempo (s)"
    ext_ax = ctx.text_extents(lbl_axis_x)
    ctx.move_to(px + pw * 0.5 - ext_ax.width * 0.5, card_y + card_h - 10.0)
    ctx.show_text(lbl_axis_x)

    # Curva recortada dentro del area del grafico
    ctx.save()
    ctx.rectangle(px, py, pw, ph)
    ctx.clip()

    if y_min < 0.0 < y_max:
        zero_y = py + ph - ((0.0 - y_min) / (y_max - y_min)) * ph
        ctx.move_to(px, zero_y)
        ctx.line_to(px + pw, zero_y)
        ctx.set_line_width(1.0)
        ctx.set_source_rgba(0.55, 0.66, 0.80, 0.45)
        ctx.stroke()
    else:
        zero_y = py + ph

    if snap.target_degrees is not None:
        sp_cnt = snap.target_degrees * snap.counts_per_rev / 360.0
        sp_val = _transform_xy_value(sp_cnt, snap.target_degrees, y_mode)
        if y_min <= sp_val <= y_max:
            sp_y = py + ph - ((sp_val - y_min) / (y_max - y_min)) * ph
            ctx.save()
            ctx.set_dash([6.0, 5.0])
            ctx.move_to(px, sp_y)
            ctx.line_to(px + pw, sp_y)
            ctx.set_line_width(1.6)
            ctx.set_source_rgba(0.98, 0.75, 0.14, 0.85)
            ctx.stroke()
            ctx.restore()

    is_cyclic_jump = y_mode in (XY_MODE_DEG_360, XY_MODE_DEG_180)
    stroke_rgb = (0.22, 0.74, 0.97) if is_counts else (0.20, 0.88, 0.66)

    if len(visible) >= 2:
        segments: List[List[Tuple[float, float]]] = []
        cur_seg: List[Tuple[float, float]] = []
        prev_val: Optional[float] = None
        for t_s, val in visible:
            sx = px + ((t_s - t_min) / window_s) * pw
            sy = py + ph - ((val - y_min) / (y_max - y_min)) * ph
            if (
                is_cyclic_jump
                and prev_val is not None
                and abs(val - prev_val) > 180.0
            ):
                if cur_seg:
                    segments.append(cur_seg)
                cur_seg = [(sx, sy)]
            else:
                cur_seg.append((sx, sy))
            prev_val = val
        if cur_seg:
            segments.append(cur_seg)

        for seg in segments:
            if len(seg) < 2:
                continue
            ctx.move_to(seg[0][0], zero_y)
            for sx, sy in seg:
                ctx.line_to(sx, sy)
            ctx.line_to(seg[-1][0], zero_y)
            ctx.close_path()
            ctx.set_source_rgba(*stroke_rgb, 0.14)
            ctx.fill()

            ctx.move_to(seg[0][0], seg[0][1])
            for sx, sy in seg[1:]:
                ctx.line_to(sx, sy)
            ctx.set_line_width(2.2)
            ctx.set_source_rgb(*stroke_rgb)
            ctx.stroke()

        last_sx, last_sy = segments[-1][-1]
        ctx.arc(last_sx, last_sy, 4.5, 0, 2.0 * math.pi)
        ctx.set_source_rgb(0.98, 0.36, 0.24)
        ctx.fill_preserve()
        ctx.set_line_width(1.2)
        ctx.set_source_rgb(1.0, 1.0, 1.0)
        ctx.stroke()
    elif len(visible) == 1:
        t_s, val = visible[0]
        sx = px + ((t_s - t_min) / window_s) * pw
        sy = py + ph - ((val - y_min) / (y_max - y_min)) * ph
        ctx.arc(sx, sy, 4.5, 0, 2.0 * math.pi)
        ctx.set_source_rgb(0.98, 0.36, 0.24)
        ctx.fill()

    ctx.restore()

    # Borde de ejes
    ctx.rectangle(px, py, pw, ph)
    ctx.set_line_width(1.2)
    ctx.set_source_rgb(0.34, 0.44, 0.60)
    ctx.stroke()


def draw_combined_scene(
    ctx: "cairo.Context",  # type: ignore[name-defined]
    width: float,
    height: float,
    snap: TelemetrySnapshot,
    clockwise: bool = True,
    zero_north: bool = True,
    show_xy: bool = False,
    xy_window_s: float = DEFAULT_XY_WINDOW_S,
    xy_y_mode: str = XY_MODE_COUNTS,
) -> None:
    """Dibuja el dial circular y, si esta desplegada, la grafica X-Y a la derecha."""
    if not show_xy:
        draw_circular_dial(
            ctx, width, height, snap, clockwise=clockwise, zero_north=zero_north
        )
        return

    dial_w = max(320.0, width * 0.48)
    xy_w = max(240.0, width - dial_w)
    draw_circular_dial(
        ctx,
        dial_w,
        height,
        snap,
        clockwise=clockwise,
        zero_north=zero_north,
        x0=0.0,
        y0=0.0,
    )
    ctx.move_to(dial_w, 8.0)
    ctx.line_to(dial_w, height - 8.0)
    ctx.set_line_width(1.2)
    ctx.set_source_rgba(0.32, 0.42, 0.58, 0.65)
    ctx.stroke()
    draw_cartesian_xy(
        ctx,
        dial_w,
        0.0,
        xy_w,
        height,
        snap,
        time_window_s=xy_window_s,
        y_mode=xy_y_mode,
    )


def render_dial_rgba(
    width: int,
    height: int,
    snap: TelemetrySnapshot,
    clockwise: bool = True,
    zero_north: bool = True,
    show_xy: bool = False,
    xy_window_s: float = DEFAULT_XY_WINDOW_S,
    xy_y_mode: str = XY_MODE_COUNTS,
) -> bytes:
    """Renderiza el dial (y grafica X-Y opcional) sobre Cairo y devuelve RGBA."""
    import cairo

    w = max(1, int(width))
    h = max(1, int(height))
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, w, h)
    ctx = cairo.Context(surface)
    draw_combined_scene(
        ctx,
        float(w),
        float(h),
        snap,
        clockwise=clockwise,
        zero_north=zero_north,
        show_xy=show_xy,
        xy_window_s=xy_window_s,
        xy_y_mode=xy_y_mode,
    )
    surface.flush()
    buf = bytearray(surface.get_data())
    buf[0::4], buf[2::4] = buf[2::4], buf[0::4]
    return bytes(buf)


SETPOINT_OPTIONS: Tuple[str, ...] = (
    "ANG (deg)",
    "POS (cnt)",
    "SPEED F (rpm)",
    "SPEED R (rpm)",
    "DUTY F (0..255)",
    "DUTY R (0..255)",
)


def build_setpoint_command(
    kind: str, raw_val: str, counts_per_rev: int
) -> Tuple[Optional[str], Optional[float], Optional[str]]:
    """Valida y construye un comando de consigna para el CLI del ESP32."""
    try:
        val = float(raw_val.strip())
    except ValueError:
        return None, None, "ERR: valor numerico invalido"
    if not math.isfinite(val):
        return None, None, "ERR: valor no finito"

    if kind == "ANG (deg)":
        return f"ANG {val:.2f}", val, None
    if kind == "POS (cnt)":
        cnt = int(round(val))
        deg = (cnt * 360.0 / max(1, counts_per_rev))
        return f"POS {cnt}", deg, None
    if kind == "SPEED F (rpm)":
        return f"SPEED F {max(0.0, val):.2f}", None, None
    if kind == "SPEED R (rpm)":
        return f"SPEED R {max(0.0, val):.2f}", None, None
    if kind == "DUTY F (0..255)":
        duty = max(0, min(255, int(round(val))))
        return f"DUTY F {duty}", None, None
    if kind == "DUTY R (0..255)":
        duty = max(0, min(255, int(round(val))))
        return f"DUTY R {duty}", None, None
    return None, None, "ERR: tipo de consigna desconocido"


def build_pid_commands(
    mode: str, kp_s: str, ki_s: str, kd_s: str
) -> Tuple[List[str], Optional[Tuple[float, float, float]], Optional[str]]:
    """Valida ganancias PID y devuelve los 3 comandos CLI correspondientes."""
    mode_u = mode.strip().upper()
    if mode_u not in PID_MODES:
        return [], None, "ERR: perfil PID invalido"
    try:
        kp = float(kp_s.strip())
        ki = float(ki_s.strip())
        kd = float(kd_s.strip())
    except ValueError:
        return [], None, "ERR: KP/KI/KD deben ser numericos"
    if not (math.isfinite(kp) and math.isfinite(ki) and math.isfinite(kd)):
        return [], None, "ERR: KP/KI/KD deben ser finitos"
    if kp < 0.0 or ki < 0.0 or kd < 0.0:
        return [], None, "ERR: KP/KI/KD deben ser >= 0"
    cmds = [
        f"PID {mode_u} KP {kp:.4f}",
        f"PID {mode_u} KI {ki:.4f}",
        f"PID {mode_u} KD {kd:.4f}",
    ]
    return cmds, (kp, ki, kd), None


def run_wx_backend(
    tracker: TelemetryTracker,
    send_cmd: Callable[[str], bool],
    on_close: Callable[[], None],
    fps: int,
    clockwise: bool,
    zero_north: bool,
    xy_window_s: float = DEFAULT_XY_WINDOW_S,
    xy_y_mode: str = XY_MODE_COUNTS,
    auto_close_ms: Optional[int] = None,
    initial_show_config: bool = False,
    initial_show_xy: bool = False,
    initial_show_terminal: bool = False,
) -> int:
    """Ejecuta la interfaz grafica en tiempo real usando wxPython + Cairo."""
    import wx

    compact_width = 860
    expanded_width = 1520
    base_height = 760

    app = wx.App(False)
    frame = wx.Frame(
        None,
        title="PIDFRA_trainer — Posicion de Aguja, PID, X-Y y Terminal Serie",
        size=(expanded_width if initial_show_xy else compact_width, base_height),
    )
    root_panel = wx.Panel(frame)
    root_sizer = wx.BoxSizer(wx.VERTICAL)
    root_panel.SetSizer(root_sizer)

    state: Dict[str, object] = {
        "show_xy": initial_show_xy,
        "xy_window_s": float(xy_window_s),
        "xy_y_mode": xy_y_mode if xy_y_mode in XY_Y_MODES else XY_MODE_COUNTS,
        "compact_w": compact_width,
        "pid_rev": -1,
        "term_rev": -1,
    }

    # --- Barra superior ---
    toolbar = wx.BoxSizer(wx.HORIZONTAL)
    status_label = wx.StaticText(root_panel, label="Iniciando...")
    toolbar.Add(status_label, 1, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)

    def add_btn(
        parent: wx.Window,
        sizer: wx.Sizer,
        title: str,
        cmd: str,
        on_sent: Optional[Callable[[], None]] = None,
        margin_flags: int = wx.RIGHT,
    ) -> wx.Button:
        btn = wx.Button(parent, label=title)

        def _on_click(_evt: wx.CommandEvent) -> None:
            send_cmd(cmd)
            if on_sent is not None:
                on_sent()

        btn.Bind(wx.EVT_BUTTON, _on_click)
        sizer.Add(btn, 0, wx.ALIGN_CENTER_VERTICAL | margin_flags, 4)
        return btn

    add_btn(root_panel, toolbar, "STOP", "STOP")
    add_btn(root_panel, toolbar, "Cero", "CAL START", tracker.clear_motion_history)

    ang_entry = wx.TextCtrl(root_panel, size=(68, -1), style=wx.TE_PROCESS_ENTER)
    ang_entry.SetHint("deg")
    toolbar.Add(ang_entry, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)

    ang_btn = wx.Button(root_panel, label="ANG")

    def _send_ang(_evt: wx.CommandEvent) -> None:
        cmd, target_deg, err = build_setpoint_command(
            "ANG (deg)", ang_entry.GetValue(), tracker.snapshot().counts_per_rev
        )
        if err or cmd is None:
            tracker.record_event(err or "ERR ANG")
            return
        if send_cmd(cmd) and target_deg is not None:
            tracker.set_target_degrees(target_deg)

    ang_btn.Bind(wx.EVT_BUTTON, _send_ang)
    ang_entry.Bind(wx.EVT_TEXT_ENTER, _send_ang)
    toolbar.Add(ang_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)

    term_btn = wx.Button(root_panel, label="Terminal Serie")
    toolbar.Add(term_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)

    cfg_toggle = wx.ToggleButton(root_panel, label="Config PID / Modos")
    cfg_toggle.SetValue(initial_show_config)
    toolbar.Add(cfg_toggle, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)

    xy_toggle = wx.ToggleButton(
        root_panel,
        label="<< Ocultar X-Y" if initial_show_xy else "Gráfica X-Y >>",
    )
    xy_toggle.SetValue(initial_show_xy)
    toolbar.Add(xy_toggle, 0, wx.ALIGN_CENTER_VERTICAL)

    root_sizer.Add(toolbar, 0, wx.EXPAND | wx.ALL, 6)

    # --- Controles rapidos de la grafica X-Y (visibles al desplegar X-Y) ---
    xy_bar = wx.Panel(root_panel)
    xy_sizer = wx.WrapSizer(wx.HORIZONTAL)
    xy_bar.SetSizer(xy_sizer)
    xy_sizer.Add(
        wx.StaticText(xy_bar, label="Ventana X-Y:"),
        0,
        wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM,
        6,
    )
    win_choice = wx.Choice(xy_bar, choices=[f"{int(w)} s" for w in XY_WINDOWS_S])
    default_win_idx = min(
        range(len(XY_WINDOWS_S)),
        key=lambda idx: abs(XY_WINDOWS_S[idx] - float(xy_window_s)),
    )
    win_choice.SetSelection(default_win_idx)
    state["xy_window_s"] = XY_WINDOWS_S[default_win_idx]
    xy_sizer.Add(win_choice, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 8)

    xy_sizer.Add(
        wx.StaticText(xy_bar, label="Escala Y:"),
        0,
        wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM,
        6,
    )
    y_mode_choice = wx.Choice(xy_bar, choices=list(XY_Y_MODES))
    y_mode_choice.SetSelection(XY_Y_MODES.index(str(state["xy_y_mode"])))
    xy_sizer.Add(y_mode_choice, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 6)

    quick_y_btn = wx.Button(xy_bar, label="Counts <-> Grados Sen")

    def _on_y_mode_change(_evt: wx.CommandEvent) -> None:
        idx = y_mode_choice.GetSelection()
        if 0 <= idx < len(XY_Y_MODES):
            state["xy_y_mode"] = XY_Y_MODES[idx]

    def _toggle_counts_sine(_evt: wx.CommandEvent) -> None:
        nxt = (
            XY_MODE_DEG_SINE
            if state["xy_y_mode"] == XY_MODE_COUNTS
            else XY_MODE_COUNTS
        )
        state["xy_y_mode"] = nxt
        y_mode_choice.SetSelection(XY_Y_MODES.index(nxt))

    y_mode_choice.Bind(wx.EVT_CHOICE, _on_y_mode_change)
    quick_y_btn.Bind(wx.EVT_BUTTON, _toggle_counts_sine)
    xy_sizer.Add(quick_y_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 8)

    clear_xy_btn = wx.Button(xy_bar, label="Limpiar X-Y")
    clear_xy_btn.Bind(wx.EVT_BUTTON, lambda _evt: tracker.clear_xy_history())
    xy_sizer.Add(clear_xy_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 8)

    def _on_win_change(_evt: wx.CommandEvent) -> None:
        idx = win_choice.GetSelection()
        if 0 <= idx < len(XY_WINDOWS_S):
            state["xy_window_s"] = XY_WINDOWS_S[idx]

    win_choice.Bind(wx.EVT_CHOICE, _on_win_change)
    root_sizer.Add(xy_bar, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)
    xy_bar.Show(initial_show_xy)

    # --- Panel desplegable de configuracion PID y modos ---
    cfg_panel = wx.Panel(root_panel, style=wx.BORDER_THEME)
    cfg_sizer = wx.BoxSizer(wx.VERTICAL)
    cfg_panel.SetSizer(cfg_sizer)

    mode_row = wx.WrapSizer(wx.HORIZONTAL)
    mode_row.Add(
        wx.StaticText(cfg_panel, label="Modo:"),
        0,
        wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM,
        6,
    )
    for m_label, m_cmd in (
        ("SPEED", "MODE SPEED"),
        ("POS", "MODE POS"),
        ("CLOCK", "MODE CLOCK"),
        ("OPEN", "MODE OPEN"),
        ("POT ON", "POT ON"),
        ("POT OFF", "POT OFF"),
        ("Re-sync LOG", "LOG POS ON"),
    ):
        add_btn(cfg_panel, mode_row, m_label, m_cmd, margin_flags=wx.RIGHT | wx.BOTTOM)

    sp_choice = wx.Choice(cfg_panel, choices=list(SETPOINT_OPTIONS))
    sp_choice.SetSelection(0)
    mode_row.Add(sp_choice, 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT | wx.RIGHT | wx.BOTTOM, 4)

    sp_entry = wx.TextCtrl(cfg_panel, size=(72, -1), style=wx.TE_PROCESS_ENTER)
    sp_entry.SetHint("valor")
    mode_row.Add(sp_entry, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 4)

    sp_btn = wx.Button(cfg_panel, label="Enviar SP")

    def _send_sp(_evt: wx.CommandEvent) -> None:
        sel = sp_choice.GetStringSelection()
        cmd, target_deg, err = build_setpoint_command(
            sel, sp_entry.GetValue(), tracker.snapshot().counts_per_rev
        )
        if err or cmd is None:
            tracker.record_event(err or "ERR SP")
            return
        if send_cmd(cmd):
            tracker.set_target_degrees(target_deg)

    sp_btn.Bind(wx.EVT_BUTTON, _send_sp)
    sp_entry.Bind(wx.EVT_TEXT_ENTER, _send_sp)
    mode_row.Add(sp_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.BOTTOM, 4)
    cfg_sizer.Add(mode_row, 0, wx.EXPAND | wx.ALL, 6)

    pid_row = wx.WrapSizer(wx.HORIZONTAL)
    pid_row.Add(
        wx.StaticText(cfg_panel, label="Perfil PID:"),
        0,
        wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM,
        6,
    )
    pid_choice = wx.Choice(cfg_panel, choices=list(PID_MODES))
    pid_choice.SetSelection(1)  # POS por defecto
    pid_row.Add(pid_choice, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 8)

    kp_entry = wx.TextCtrl(cfg_panel, size=(72, -1))
    ki_entry = wx.TextCtrl(cfg_panel, size=(72, -1))
    kd_entry = wx.TextCtrl(cfg_panel, size=(72, -1))
    for lbl, ctrl in (("KP", kp_entry), ("KI", ki_entry), ("KD", kd_entry)):
        pid_row.Add(
            wx.StaticText(cfg_panel, label=f"{lbl}:"),
            0,
            wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM,
            3,
        )
        pid_row.Add(ctrl, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 6)

    def _populate_pid_fields() -> None:
        mode = pid_choice.GetStringSelection()
        gains = tracker.snapshot().pid_gains.get(mode)
        if gains is not None:
            kp_entry.SetValue(f"{gains[0]:.4f}")
            ki_entry.SetValue(f"{gains[1]:.4f}")
            kd_entry.SetValue(f"{gains[2]:.4f}")

    _populate_pid_fields()
    pid_choice.Bind(wx.EVT_CHOICE, lambda _evt: _populate_pid_fields())

    read_pid_btn = wx.Button(cfg_panel, label="Leer (PID GET)")
    read_pid_btn.Bind(wx.EVT_BUTTON, lambda _evt: send_cmd("PID GET"))
    pid_row.Add(read_pid_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 4)

    apply_pid_btn = wx.Button(cfg_panel, label="Aplicar PID")

    def _apply_pid(_evt: wx.CommandEvent) -> None:
        mode = pid_choice.GetStringSelection()
        cmds, gains, err = build_pid_commands(
            mode, kp_entry.GetValue(), ki_entry.GetValue(), kd_entry.GetValue()
        )
        if err or gains is None:
            tracker.record_event(err or "ERR PID")
            return
        for cmd in cmds:
            send_cmd(cmd)
        tracker.update_pid_gains(mode, *gains)

    apply_pid_btn.Bind(wx.EVT_BUTTON, _apply_pid)
    pid_row.Add(apply_pid_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 4)

    def_pid_btn = wx.Button(cfg_panel, label="Default")

    def _default_pid(_evt: wx.CommandEvent) -> None:
        if send_cmd("PID DEFAULT"):
            wx.CallLater(120, lambda: send_cmd("PID GET"))

    def_pid_btn.Bind(wx.EVT_BUTTON, _default_pid)
    pid_row.Add(def_pid_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 4)

    add_btn(
        cfg_panel, pid_row, "Guardar NVS", "SAVE", margin_flags=wx.RIGHT | wx.BOTTOM
    )

    minmove_entry = wx.TextCtrl(cfg_panel, size=(52, -1), value="185")
    pid_row.Add(
        wx.StaticText(cfg_panel, label="MINMOVE:"),
        0,
        wx.ALIGN_CENTER_VERTICAL | wx.LEFT | wx.RIGHT | wx.BOTTOM,
        4,
    )
    pid_row.Add(minmove_entry, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT | wx.BOTTOM, 4)
    minmove_btn = wx.Button(cfg_panel, label="Set")

    def _send_minmove(_evt: wx.CommandEvent) -> None:
        try:
            mm = max(0, min(255, int(round(float(minmove_entry.GetValue().strip())))))
        except ValueError:
            tracker.record_event("ERR MINMOVE")
            return
        send_cmd(f"MINMOVE {mm}")

    minmove_btn.Bind(wx.EVT_BUTTON, _send_minmove)
    pid_row.Add(minmove_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.BOTTOM, 4)

    cfg_sizer.Add(pid_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)
    root_sizer.Add(cfg_panel, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)
    cfg_panel.Show(initial_show_config)

    # --- Ventana secundaria: Terminal Serie para comandos crudos ---
    term_frame = wx.Frame(
        frame,
        title="Terminal Serie — Comandos Crudos CLI (ESP32)",
        size=(740, 480),
    )
    term_panel = wx.Panel(term_frame)
    term_sizer = wx.BoxSizer(wx.VERTICAL)
    term_panel.SetSizer(term_sizer)

    term_top = wx.WrapSizer(wx.HORIZONTAL)
    for q_cmd in TERMINAL_QUICK_CMDS:
        add_btn(term_panel, term_top, q_cmd, q_cmd, margin_flags=wx.RIGHT | wx.BOTTOM)

    raw_stream_chk = wx.CheckBox(term_panel, label="Ver stream P,<counts>,<deg>")
    raw_stream_chk.Bind(
        wx.EVT_CHECKBOX,
        lambda _evt: tracker.set_log_raw_pos_stream(raw_stream_chk.GetValue()),
    )
    term_top.Add(raw_stream_chk, 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)

    clear_term_btn = wx.Button(term_panel, label="Limpiar")
    clear_term_btn.Bind(wx.EVT_BUTTON, lambda _evt: tracker.clear_serial_log())
    term_top.Add(clear_term_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.BOTTOM, 4)
    term_sizer.Add(term_top, 0, wx.EXPAND | wx.ALL, 6)

    term_output = wx.TextCtrl(
        term_panel,
        style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_DONTWRAP | wx.HSCROLL,
    )
    term_output.SetBackgroundColour(wx.Colour(14, 20, 32))
    term_output.SetForegroundColour(wx.Colour(220, 234, 250))
    term_output.SetFont(
        wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
    )
    term_sizer.Add(term_output, 1, wx.EXPAND | wx.LEFT | wx.RIGHT, 6)

    term_input_row = wx.BoxSizer(wx.HORIZONTAL)
    term_input = wx.TextCtrl(term_panel, style=wx.TE_PROCESS_ENTER)
    term_input.SetHint("Escribe un comando crudo (ej. HELP, STATUS, ANG 90, PID GET)...")
    term_input_row.Add(term_input, 1, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)
    term_send_btn = wx.Button(term_panel, label="Enviar")
    term_input_row.Add(term_send_btn, 0, wx.ALIGN_CENTER_VERTICAL)
    term_sizer.Add(term_input_row, 0, wx.EXPAND | wx.ALL, 6)

    cmd_history: List[str] = []
    hist_idx = [-1]

    def _refresh_terminal(snap: TelemetrySnapshot, force: bool = False) -> None:
        if not force and snap.serial_log_revision == state["term_rev"]:
            return
        state["term_rev"] = snap.serial_log_revision
        txt = "\n".join(snap.serial_log)
        if txt:
            txt += "\n"
        term_output.ChangeValue(txt)
        term_output.ShowPosition(term_output.GetLastPosition())

    def _send_raw_cmd(_evt: wx.CommandEvent) -> None:
        raw = term_input.GetValue().strip()
        if not raw:
            return
        if not cmd_history or cmd_history[-1] != raw:
            cmd_history.append(raw)
        hist_idx[0] = -1
        send_cmd(raw)
        term_input.SetValue("")
        _refresh_terminal(tracker.snapshot(), force=True)

    def _on_term_key(evt: wx.KeyEvent) -> None:
        code = evt.GetKeyCode()
        if code == wx.WXK_UP and cmd_history:
            if hist_idx[0] == -1:
                hist_idx[0] = len(cmd_history) - 1
            elif hist_idx[0] > 0:
                hist_idx[0] -= 1
            term_input.SetValue(cmd_history[hist_idx[0]])
            term_input.SetInsertionPointEnd()
            return
        if code == wx.WXK_DOWN and cmd_history:
            if hist_idx[0] == -1:
                return
            if hist_idx[0] < len(cmd_history) - 1:
                hist_idx[0] += 1
                term_input.SetValue(cmd_history[hist_idx[0]])
                term_input.SetInsertionPointEnd()
            else:
                hist_idx[0] = -1
                term_input.SetValue("")
            return
        evt.Skip()

    term_send_btn.Bind(wx.EVT_BUTTON, _send_raw_cmd)
    term_input.Bind(wx.EVT_TEXT_ENTER, _send_raw_cmd)
    term_input.Bind(wx.EVT_KEY_DOWN, _on_term_key)

    def _open_terminal(_evt: wx.CommandEvent) -> None:
        _refresh_terminal(tracker.snapshot(), force=True)
        term_frame.Show()
        term_frame.Raise()
        term_input.SetFocus()

    term_btn.Bind(wx.EVT_BUTTON, _open_terminal)

    # --- Lienzo Cairo (dial circular + grafica X-Y opcional) ---
    canvas = wx.Panel(root_panel)
    canvas.SetBackgroundStyle(wx.BG_STYLE_PAINT)
    root_sizer.Add(canvas, 1, wx.EXPAND)

    def _toggle_config(_evt: wx.CommandEvent) -> None:
        cfg_panel.Show(cfg_toggle.GetValue())
        root_panel.Layout()

    cfg_toggle.Bind(wx.EVT_TOGGLEBUTTON, _toggle_config)

    def _toggle_xy(_evt: wx.CommandEvent) -> None:
        show_xy = bool(xy_toggle.GetValue())
        cur_w, cur_h = frame.GetSize()
        if show_xy and not state["show_xy"]:
            state["compact_w"] = max(compact_width, cur_w)
            frame.SetSize(max(expanded_width, cur_w + 560), cur_h)
            xy_toggle.SetLabel("<< Ocultar X-Y")
        elif not show_xy and state["show_xy"]:
            frame.SetSize(int(state["compact_w"]), cur_h)
            xy_toggle.SetLabel("Gráfica X-Y >>")
        state["show_xy"] = show_xy
        xy_bar.Show(show_xy)
        root_panel.Layout()
        canvas.Refresh(False)

    xy_toggle.Bind(wx.EVT_TOGGLEBUTTON, _toggle_xy)

    last_status = [""]

    def _on_paint(_evt: wx.PaintEvent) -> None:
        dc = wx.AutoBufferedPaintDC(canvas)
        w, h = canvas.GetClientSize()
        if w <= 0 or h <= 0:
            return
        snap = tracker.snapshot()
        if snap.status_text != last_status[0]:
            last_status[0] = snap.status_text
            status_label.SetLabel(snap.status_text)
            root_panel.Layout()
        if snap.pid_revision != state["pid_rev"]:
            state["pid_rev"] = snap.pid_revision
            _populate_pid_fields()
        if term_frame.IsShown():
            _refresh_terminal(snap)
        rgba = render_dial_rgba(
            w,
            h,
            snap,
            clockwise=clockwise,
            zero_north=zero_north,
            show_xy=bool(state["show_xy"]),
            xy_window_s=float(state["xy_window_s"]),
            xy_y_mode=str(state["xy_y_mode"]),
        )
        bmp = wx.Bitmap.FromBufferRGBA(w, h, rgba)
        dc.DrawBitmap(bmp, 0, 0, False)

    canvas.Bind(wx.EVT_PAINT, _on_paint)
    canvas.Bind(wx.EVT_SIZE, lambda evt: (canvas.Refresh(False), evt.Skip()))

    timer = wx.Timer(frame)
    interval_ms = max(10, int(1000 / max(1, fps)))

    def _on_timer(_evt: wx.TimerEvent) -> None:
        canvas.Refresh(False)

    frame.Bind(wx.EVT_TIMER, _on_timer, timer)
    timer.Start(interval_ms)

    closing = [False]

    def _on_term_close(evt: wx.CloseEvent) -> None:
        if closing[0]:
            evt.Skip()
        else:
            evt.Veto()
            term_frame.Hide()

    term_frame.Bind(wx.EVT_CLOSE, _on_term_close)

    def _on_close(evt: Optional[wx.CloseEvent] = None) -> None:
        if closing[0]:
            return
        closing[0] = True
        timer.Stop()
        on_close()
        if term_frame:
            term_frame.Destroy()
        if evt is not None:
            evt.Skip()
        else:
            frame.Destroy()

    frame.Bind(wx.EVT_CLOSE, _on_close)
    signal.signal(signal.SIGINT, lambda *_a: wx.CallAfter(frame.Close))
    if auto_close_ms is not None:
        wx.CallLater(auto_close_ms, frame.Close)

    root_panel.Layout()
    frame.Show()
    if initial_show_terminal:
        _refresh_terminal(tracker.snapshot(), force=True)
        term_frame.Show()
    app.MainLoop()
    return 0


def run_gtk_backend(
    tracker: TelemetryTracker,
    send_cmd: Callable[[str], bool],
    on_close: Callable[[], None],
    fps: int,
    clockwise: bool,
    zero_north: bool,
    xy_window_s: float = DEFAULT_XY_WINDOW_S,
    xy_y_mode: str = XY_MODE_COUNTS,
) -> int:
    """Ejecuta la interfaz grafica en tiempo real usando GTK3 + Cairo."""
    import gi

    gi.require_version("Gtk", "3.0")
    gi.require_version("Gdk", "3.0")
    gi.require_version("GdkPixbuf", "2.0")
    from gi.repository import Gdk, GdkPixbuf, GLib, Gtk

    has_foreign_cairo = False
    try:
        gi.require_foreign("cairo")
        has_foreign_cairo = True
    except ImportError:
        has_foreign_cairo = False

    compact_width = 860
    expanded_width = 1520
    base_height = 760

    window = Gtk.Window(title="PIDFRA_trainer — Posicion de Aguja, PID, X-Y y Terminal Serie")
    window.set_default_size(compact_width, base_height)

    state: Dict[str, object] = {
        "show_xy": False,
        "xy_window_s": float(xy_window_s),
        "xy_y_mode": xy_y_mode if xy_y_mode in XY_Y_MODES else XY_MODE_COUNTS,
        "compact_w": compact_width,
        "pid_rev": -1,
        "term_rev": -1,
    }

    root_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
    window.add(root_box)

    toolbar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
    toolbar.set_margin_top(6)
    toolbar.set_margin_bottom(2)
    toolbar.set_margin_start(8)
    toolbar.set_margin_end(8)
    root_box.pack_start(toolbar, False, False, 0)

    status_label = Gtk.Label(label="Iniciando...")
    status_label.set_xalign(0.0)
    toolbar.pack_start(status_label, True, True, 0)

    def add_btn(
        box: Gtk.Box,
        title: str,
        cmd: str,
        on_sent: Optional[Callable[[], None]] = None,
    ) -> None:
        btn = Gtk.Button(label=title)

        def _clicked(_widget: Gtk.Widget) -> None:
            send_cmd(cmd)
            if on_sent is not None:
                on_sent()

        btn.connect("clicked", _clicked)
        box.pack_start(btn, False, False, 0)

    add_btn(toolbar, "STOP", "STOP")
    add_btn(toolbar, "Cero", "CAL START", tracker.clear_motion_history)

    ang_entry = Gtk.Entry()
    ang_entry.set_width_chars(6)
    ang_entry.set_placeholder_text("deg")
    toolbar.pack_start(ang_entry, False, False, 0)

    ang_btn = Gtk.Button(label="ANG")

    def _send_ang(_widget: Gtk.Widget) -> None:
        cmd, target_deg, err = build_setpoint_command(
            "ANG (deg)", ang_entry.get_text(), tracker.snapshot().counts_per_rev
        )
        if err or cmd is None:
            tracker.record_event(err or "ERR ANG")
            return
        if send_cmd(cmd) and target_deg is not None:
            tracker.set_target_degrees(target_deg)

    ang_btn.connect("clicked", _send_ang)
    ang_entry.connect("activate", _send_ang)
    toolbar.pack_start(ang_btn, False, False, 0)

    term_btn = Gtk.Button(label="Terminal Serie")
    toolbar.pack_start(term_btn, False, False, 0)

    cfg_toggle = Gtk.ToggleButton(label="Config PID / Modos")
    toolbar.pack_start(cfg_toggle, False, False, 0)

    xy_toggle = Gtk.ToggleButton(label="Gráfica X-Y >>")
    toolbar.pack_start(xy_toggle, False, False, 0)

    # Barra X-Y desplegable
    xy_bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
    xy_bar.set_margin_start(8)
    xy_bar.set_margin_end(8)
    xy_bar.pack_start(Gtk.Label(label="Ventana X-Y:"), False, False, 0)
    win_combo = Gtk.ComboBoxText()
    for w_s in XY_WINDOWS_S:
        win_combo.append_text(f"{int(w_s)} s")
    default_win_idx = min(
        range(len(XY_WINDOWS_S)),
        key=lambda idx: abs(XY_WINDOWS_S[idx] - float(xy_window_s)),
    )
    win_combo.set_active(default_win_idx)
    state["xy_window_s"] = XY_WINDOWS_S[default_win_idx]
    win_combo.connect(
        "changed",
        lambda _c: state.__setitem__(
            "xy_window_s",
            XY_WINDOWS_S[max(0, win_combo.get_active())],
        ),
    )
    xy_bar.pack_start(win_combo, False, False, 0)

    xy_bar.pack_start(Gtk.Label(label="Escala Y:"), False, False, 0)
    y_combo = Gtk.ComboBoxText()
    for ym in XY_Y_MODES:
        y_combo.append_text(ym)
    y_combo.set_active(XY_Y_MODES.index(str(state["xy_y_mode"])))
    y_combo.connect(
        "changed",
        lambda _c: state.__setitem__(
            "xy_y_mode",
            XY_Y_MODES[max(0, y_combo.get_active())],
        ),
    )
    xy_bar.pack_start(y_combo, False, False, 0)

    quick_y_btn = Gtk.Button(label="Counts <-> Grados Sen")

    def _toggle_gtk_counts_sine(_b: Gtk.Widget) -> None:
        nxt = (
            XY_MODE_DEG_SINE
            if state["xy_y_mode"] == XY_MODE_COUNTS
            else XY_MODE_COUNTS
        )
        state["xy_y_mode"] = nxt
        y_combo.set_active(XY_Y_MODES.index(nxt))

    quick_y_btn.connect("clicked", _toggle_gtk_counts_sine)
    xy_bar.pack_start(quick_y_btn, False, False, 0)

    clear_xy_btn = Gtk.Button(label="Limpiar X-Y")
    clear_xy_btn.connect("clicked", lambda _b: tracker.clear_xy_history())
    xy_bar.pack_start(clear_xy_btn, False, False, 0)
    root_box.pack_start(xy_bar, False, False, 0)

    # Panel de configuracion PID / Modos
    cfg_frame = Gtk.Frame(label="Configuración PID y Modos")
    cfg_frame.set_margin_start(8)
    cfg_frame.set_margin_end(8)
    cfg_vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
    cfg_vbox.set_margin_top(6)
    cfg_vbox.set_margin_bottom(6)
    cfg_vbox.set_margin_start(8)
    cfg_vbox.set_margin_end(8)
    cfg_frame.add(cfg_vbox)

    mode_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
    mode_row.pack_start(Gtk.Label(label="Modo:"), False, False, 0)
    for m_label, m_cmd in (
        ("SPEED", "MODE SPEED"),
        ("POS", "MODE POS"),
        ("CLOCK", "MODE CLOCK"),
        ("OPEN", "MODE OPEN"),
        ("POT ON", "POT ON"),
        ("POT OFF", "POT OFF"),
        ("Re-sync LOG", "LOG POS ON"),
    ):
        add_btn(mode_row, m_label, m_cmd)

    sp_combo = Gtk.ComboBoxText()
    for opt in SETPOINT_OPTIONS:
        sp_combo.append_text(opt)
    sp_combo.set_active(0)
    sp_entry = Gtk.Entry()
    sp_entry.set_width_chars(7)
    sp_entry.set_placeholder_text("valor")
    sp_btn = Gtk.Button(label="Enviar SP")

    def _send_sp(_widget: Gtk.Widget) -> None:
        sel = sp_combo.get_active_text() or SETPOINT_OPTIONS[0]
        cmd, target_deg, err = build_setpoint_command(
            sel, sp_entry.get_text(), tracker.snapshot().counts_per_rev
        )
        if err or cmd is None:
            tracker.record_event(err or "ERR SP")
            return
        if send_cmd(cmd):
            tracker.set_target_degrees(target_deg)

    sp_btn.connect("clicked", _send_sp)
    sp_entry.connect("activate", _send_sp)
    sp_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
    sp_row.pack_start(Gtk.Label(label="Consigna:"), False, False, 0)
    sp_row.pack_start(sp_combo, False, False, 0)
    sp_row.pack_start(sp_entry, False, False, 0)
    sp_row.pack_start(sp_btn, False, False, 0)
    cfg_vbox.pack_start(mode_row, False, False, 0)
    cfg_vbox.pack_start(sp_row, False, False, 0)

    pid_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
    pid_row.pack_start(Gtk.Label(label="Perfil PID:"), False, False, 0)
    pid_combo = Gtk.ComboBoxText()
    for mode in PID_MODES:
        pid_combo.append_text(mode)
    pid_combo.set_active(1)
    pid_row.pack_start(pid_combo, False, False, 0)

    kp_entry = Gtk.Entry()
    ki_entry = Gtk.Entry()
    kd_entry = Gtk.Entry()
    for lbl, ent in (("KP:", kp_entry), ("KI:", ki_entry), ("KD:", kd_entry)):
        ent.set_width_chars(7)
        pid_row.pack_start(Gtk.Label(label=lbl), False, False, 0)
        pid_row.pack_start(ent, False, False, 0)

    def _populate_pid_fields() -> None:
        mode = pid_combo.get_active_text() or "POS"
        gains = tracker.snapshot().pid_gains.get(mode)
        if gains is not None:
            kp_entry.set_text(f"{gains[0]:.4f}")
            ki_entry.set_text(f"{gains[1]:.4f}")
            kd_entry.set_text(f"{gains[2]:.4f}")

    _populate_pid_fields()
    pid_combo.connect("changed", lambda _c: _populate_pid_fields())

    read_btn = Gtk.Button(label="Leer (PID GET)")
    read_btn.connect("clicked", lambda _b: send_cmd("PID GET"))

    apply_btn = Gtk.Button(label="Aplicar PID")

    def _apply_pid(_widget: Gtk.Widget) -> None:
        mode = pid_combo.get_active_text() or "POS"
        cmds, gains, err = build_pid_commands(
            mode, kp_entry.get_text(), ki_entry.get_text(), kd_entry.get_text()
        )
        if err or gains is None:
            tracker.record_event(err or "ERR PID")
            return
        for cmd in cmds:
            send_cmd(cmd)
        tracker.update_pid_gains(mode, *gains)

    apply_btn.connect("clicked", _apply_pid)

    def_btn = Gtk.Button(label="Default")

    def _default_pid(_widget: Gtk.Widget) -> None:
        if send_cmd("PID DEFAULT"):
            GLib.timeout_add(120, lambda: (send_cmd("PID GET"), False)[1])

    def_btn.connect("clicked", _default_pid)
    pid_actions = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
    pid_actions.pack_start(read_btn, False, False, 0)
    pid_actions.pack_start(apply_btn, False, False, 0)
    pid_actions.pack_start(def_btn, False, False, 0)
    add_btn(pid_actions, "Guardar NVS", "SAVE")

    pid_actions.pack_start(Gtk.Label(label="MINMOVE:"), False, False, 0)
    mm_entry = Gtk.Entry()
    mm_entry.set_width_chars(4)
    mm_entry.set_text("185")
    pid_actions.pack_start(mm_entry, False, False, 0)
    mm_btn = Gtk.Button(label="Set")

    def _send_mm(_widget: Gtk.Widget) -> None:
        try:
            mm = max(0, min(255, int(round(float(mm_entry.get_text().strip())))))
        except ValueError:
            tracker.record_event("ERR MINMOVE")
            return
        send_cmd(f"MINMOVE {mm}")

    mm_btn.connect("clicked", _send_mm)
    pid_actions.pack_start(mm_btn, False, False, 0)
    cfg_vbox.pack_start(pid_row, False, False, 0)
    cfg_vbox.pack_start(pid_actions, False, False, 0)

    root_box.pack_start(cfg_frame, False, False, 0)

    # Ventana de Terminal Serie
    term_win = Gtk.Window(title="Terminal Serie — Comandos Crudos CLI (ESP32)")
    term_win.set_default_size(740, 480)
    term_win.set_transient_for(window)

    term_vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
    term_vbox.set_margin_top(6)
    term_vbox.set_margin_bottom(6)
    term_vbox.set_margin_start(8)
    term_vbox.set_margin_end(8)
    term_win.add(term_vbox)

    term_top = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
    for q_cmd in TERMINAL_QUICK_CMDS:
        add_btn(term_top, q_cmd, q_cmd)

    raw_chk = Gtk.CheckButton(label="Ver stream P,<counts>,<deg>")
    raw_chk.connect("toggled", lambda c: tracker.set_log_raw_pos_stream(c.get_active()))
    term_top.pack_start(raw_chk, False, False, 6)

    clr_term_btn = Gtk.Button(label="Limpiar")
    clr_term_btn.connect("clicked", lambda _b: tracker.clear_serial_log())
    term_top.pack_start(clr_term_btn, False, False, 0)
    term_vbox.pack_start(term_top, False, False, 0)

    scrolled = Gtk.ScrolledWindow()
    term_view = Gtk.TextView()
    term_view.set_editable(False)
    term_view.set_monospace(True)
    term_buf = term_view.get_buffer()
    scrolled.add(term_view)
    term_vbox.pack_start(scrolled, True, True, 0)

    term_in_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
    term_entry = Gtk.Entry()
    term_entry.set_placeholder_text("Escribe un comando crudo (ej. HELP, STATUS, ANG 90, PID GET)...")
    term_in_row.pack_start(term_entry, True, True, 0)
    term_send_btn = Gtk.Button(label="Enviar")
    term_in_row.pack_start(term_send_btn, False, False, 0)
    term_vbox.pack_start(term_in_row, False, False, 0)

    gtk_cmd_hist: List[str] = []
    gtk_hist_idx = [-1]

    def _refresh_gtk_terminal(snap: TelemetrySnapshot, force: bool = False) -> None:
        if not force and snap.serial_log_revision == state["term_rev"]:
            return
        state["term_rev"] = snap.serial_log_revision
        txt = "\n".join(snap.serial_log)
        if txt:
            txt += "\n"
        term_buf.set_text(txt)
        end_iter = term_buf.get_end_iter()
        term_view.scroll_to_iter(end_iter, 0.0, False, 0.0, 1.0)

    def _send_gtk_raw(_w: Gtk.Widget) -> None:
        raw = term_entry.get_text().strip()
        if not raw:
            return
        if not gtk_cmd_hist or gtk_cmd_hist[-1] != raw:
            gtk_cmd_hist.append(raw)
        gtk_hist_idx[0] = -1
        send_cmd(raw)
        term_entry.set_text("")
        _refresh_gtk_terminal(tracker.snapshot(), force=True)

    def _on_gtk_term_key(_w: Gtk.Widget, evt: Gdk.EventKey) -> bool:
        if evt.keyval == Gdk.KEY_Up and gtk_cmd_hist:
            if gtk_hist_idx[0] == -1:
                gtk_hist_idx[0] = len(gtk_cmd_hist) - 1
            elif gtk_hist_idx[0] > 0:
                gtk_hist_idx[0] -= 1
            term_entry.set_text(gtk_cmd_hist[gtk_hist_idx[0]])
            term_entry.set_position(-1)
            return True
        if evt.keyval == Gdk.KEY_Down and gtk_cmd_hist:
            if gtk_hist_idx[0] == -1:
                return True
            if gtk_hist_idx[0] < len(gtk_cmd_hist) - 1:
                gtk_hist_idx[0] += 1
                term_entry.set_text(gtk_cmd_hist[gtk_hist_idx[0]])
                term_entry.set_position(-1)
            else:
                gtk_hist_idx[0] = -1
                term_entry.set_text("")
            return True
        return False

    term_send_btn.connect("clicked", _send_gtk_raw)
    term_entry.connect("activate", _send_gtk_raw)
    term_entry.connect("key-press-event", _on_gtk_term_key)
    term_win.connect("delete-event", lambda w, _e: (w.hide(), True)[1])

    def _open_gtk_term(_w: Gtk.Widget) -> None:
        _refresh_gtk_terminal(tracker.snapshot(), force=True)
        term_win.show_all()
        term_win.present()
        term_entry.grab_focus()

    term_btn.connect("clicked", _open_gtk_term)

    cfg_toggle.connect(
        "toggled",
        lambda btn: cfg_frame.set_visible(btn.get_active()),
    )

    def _on_xy_toggled(btn: Gtk.ToggleButton) -> None:
        show_xy = btn.get_active()
        cur_w, cur_h = window.get_size()
        if show_xy and not state["show_xy"]:
            state["compact_w"] = max(compact_width, cur_w)
            window.resize(max(expanded_width, cur_w + 560), cur_h)
            btn.set_label("<< Ocultar X-Y")
        elif not show_xy and state["show_xy"]:
            window.resize(int(state["compact_w"]), cur_h)
            btn.set_label("Gráfica X-Y >>")
        state["show_xy"] = show_xy
        xy_bar.set_visible(show_xy)

    xy_toggle.connect("toggled", _on_xy_toggled)

    if has_foreign_cairo:
        drawing_area = Gtk.DrawingArea()
        root_box.pack_start(drawing_area, True, True, 0)

        def _on_draw(widget: Gtk.Widget, ctx: object) -> bool:
            alloc = widget.get_allocation()
            snap = tracker.snapshot()
            status_label.set_text(snap.status_text)
            if snap.pid_revision != state["pid_rev"]:
                state["pid_rev"] = snap.pid_revision
                _populate_pid_fields()
            if term_win.get_visible():
                _refresh_gtk_terminal(snap)
            draw_combined_scene(
                ctx,  # type: ignore[arg-type]
                float(alloc.width),
                float(alloc.height),
                snap,
                clockwise=clockwise,
                zero_north=zero_north,
                show_xy=bool(state["show_xy"]),
                xy_window_s=float(state["xy_window_s"]),
                xy_y_mode=str(state["xy_y_mode"]),
            )
            return False

        drawing_area.connect("draw", _on_draw)

        def _refresh_view() -> bool:
            drawing_area.queue_draw()
            return True
    else:
        image_widget = Gtk.Image()
        image_widget.set_size_request(400, 400)
        root_box.pack_start(image_widget, True, True, 0)

        def _refresh_view() -> bool:
            alloc = image_widget.get_allocation()
            w = max(400, alloc.width)
            h = max(400, alloc.height)
            snap = tracker.snapshot()
            status_label.set_text(snap.status_text)
            if snap.pid_revision != state["pid_rev"]:
                state["pid_rev"] = snap.pid_revision
                _populate_pid_fields()
            if term_win.get_visible():
                _refresh_gtk_terminal(snap)
            rgba = render_dial_rgba(
                w,
                h,
                snap,
                clockwise=clockwise,
                zero_north=zero_north,
                show_xy=bool(state["show_xy"]),
                xy_window_s=float(state["xy_window_s"]),
                xy_y_mode=str(state["xy_y_mode"]),
            )
            pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                rgba, GdkPixbuf.Colorspace.RGB, True, 8, w, h, w * 4
            )
            image_widget.set_from_pixbuf(pixbuf)
            return True

    interval_ms = max(10, int(1000 / max(1, fps)))
    timer_id = GLib.timeout_add(interval_ms, _refresh_view)

    def _shutdown(*_args: object) -> bool:
        GLib.source_remove(timer_id)
        on_close()
        term_win.destroy()
        Gtk.main_quit()
        return False

    window.connect("delete-event", _shutdown)
    GLib.unix_signal_add(GLib.PRIORITY_HIGH, signal.SIGINT, _shutdown)
    window.show_all()
    cfg_frame.hide()
    xy_bar.hide()
    Gtk.main()
    return 0


def run_matplotlib_backend(
    tracker: TelemetryTracker,
    on_close: Callable[[], None],
    fps: int,
    clockwise: bool,
    zero_north: bool,
) -> int:
    """Ejecuta el grafico polar en tiempo real usando Matplotlib."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure("PIDFRA_trainer — Posicion de Aguja en Tiempo Real", figsize=(7, 7))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N" if zero_north else "E")
    ax.set_theta_direction(-1 if clockwise else 1)
    ax.set_ylim(0, 1.0)
    ax.set_yticks([])
    ax.set_xticks([math.radians(d) for d in range(0, 360, 30)])
    ax.set_xticklabels([f"{d}°" for d in range(0, 360, 30)])
    ax.set_facecolor("#161d2b")
    fig.patch.set_facecolor("#12161f")
    ax.tick_params(colors="#b8c9e0")
    ax.grid(color="#364663", linestyle="--", linewidth=0.7)

    (trail_line,) = ax.plot([], [], color="#38bdf8", linewidth=2.2, alpha=0.55)
    (target_line,) = ax.plot([], [], color="#f59e0b", linestyle="--", linewidth=1.8)
    (needle_line,) = ax.plot([], [], color="#fa5c3d", linewidth=3.5, marker="o")
    info_text = fig.text(
        0.5,
        0.02,
        "",
        ha="center",
        va="bottom",
        color="#e2e8f0",
        fontsize=10,
        family="monospace",
    )

    def _update(_frame: int) -> Tuple[object, ...]:
        snap = tracker.snapshot()
        theta = math.radians(snap.degrees)
        needle_line.set_data([theta, theta], [0.0, 0.88])
        if len(snap.trail) >= 2:
            trail_line.set_data(
                [math.radians(a) for a in snap.trail], [0.82] * len(snap.trail)
            )
        else:
            trail_line.set_data([], [])
        if snap.target_degrees is not None:
            t_theta = math.radians(snap.target_degrees)
            target_line.set_data([t_theta, t_theta], [0.0, 0.94])
        else:
            target_line.set_data([], [])

        info_text.set_text(
            f"Ang: {snap.angle_mod_360:6.2f}° ({snap.degrees:8.2f}°) | "
            f"Cnt: {snap.counts} | Vueltas: {snap.turns:+.3f} | "
            f"{snap.rpm:+.1f} RPM | {snap.sample_rate_hz:.1f} Hz"
        )
        return trail_line, target_line, needle_line, info_text

    interval_ms = max(10, int(1000 / max(1, fps)))
    _anim = FuncAnimation(fig, _update, interval=interval_ms, blit=False)
    fig.canvas.mpl_connect("close_event", lambda _evt: on_close())

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        on_close()
    return 0


def run_self_test() -> int:
    """Valida parser, metricas, renderizado Cairo (3 escalas + X-Y) y wx."""
    import os

    samples = [
        ("P,908,90.80\n", "P", 908, 90.80, None),
        ("P,-1800,-180.00\r\n", "P", -1800, -180.00, None),
        ("TEL P R=90.00 A=88.74 D=64\r\n", "TEL", None, 88.74, None),
        ("PID P KP=0.4500 KI=0.2100 KD=0.0150\r\n", "PID", None, None, "POS"),
        ("EVT READY A=23 B=22 C=900 Q=4\r\n", "EVENT", None, None, None),
        ("POS OUT=-0.06 ERR=11.00\r\n", None, None, None, None),
    ]
    for raw, exp_kind, exp_cnt, exp_deg, exp_pid_mode in samples:
        parsed = parse_serial_line(raw)
        if exp_kind is None:
            assert parsed is None, f"Se esperaba None para {raw!r}, obtuvo {parsed}"
        else:
            assert parsed is not None and parsed.kind == exp_kind, f"Fallo en {raw!r}"
            if exp_cnt is not None:
                assert parsed.counts == exp_cnt
            if exp_deg is not None:
                assert parsed.degrees is not None and abs(parsed.degrees - exp_deg) < 1e-6
            if exp_pid_mode is not None:
                assert parsed.pid_mode == exp_pid_mode
                assert parsed.pid_gains == (0.45, 0.21, 0.015)

    assert abs(to_canvas_angle_rad(0.0, True, True) + math.pi / 2.0) < 1e-9
    assert abs(to_canvas_angle_rad(90.0, True, True) - 0.0) < 1e-9
    assert abs(shortest_angle_delta_deg(5.0, 355.0) - 10.0) < 1e-9

    cmds, gains, err = build_pid_commands("POS", "0.5", "0.25", "0.02")
    assert err is None and gains == (0.5, 0.25, 0.02) and len(cmds) == 3

    tracker = TelemetryTracker(trail_len=10, counts_per_rev=3600)
    tracker.set_connection_state(True, "SELF-TEST OK")
    tracker.update_pid_gains("POS", 0.45, 0.21, 0.015)
    tracker.append_serial_log("TX > PID GET")
    tracker.append_serial_log("RX < PID P KP=0.4500 KI=0.2100 KD=0.0150")
    tracker.update_position(0, 0.0, timestamp=1.0)
    tracker.update_position(908, 90.80, timestamp=1.2, target_degrees=90.0)
    tracker.update_position(1800, 180.0, timestamp=1.4, target_degrees=90.0)
    tracker.update_position(2700, 270.0, timestamp=1.6, target_degrees=90.0)
    snap = tracker.snapshot()
    assert snap.counts == 2700
    assert abs(snap.angle_mod_360 - 270.0) < 1e-6
    assert abs(snap.seconds_mod_60 - 45.0) < 1e-6
    assert abs(snap.turns - 0.75) < 1e-6
    assert len(snap.xy_history) == 4
    assert len(snap.serial_log) == 2

    assert abs(_transform_xy_value(900, 90.0, XY_MODE_DEG_SINE) - 180.0) < 1e-6
    assert abs(_transform_xy_value(2700, 270.0, XY_MODE_DEG_SINE) + 180.0) < 1e-6
    assert abs(_transform_xy_value(3700, 370.0, XY_MODE_DEG_360) - 10.0) < 1e-6

    for y_mode in (XY_MODE_COUNTS, XY_MODE_DEG_SINE, XY_MODE_DEG_360):
        rgba = render_dial_rgba(
            1280,
            680,
            snap,
            clockwise=True,
            zero_north=True,
            show_xy=True,
            xy_window_s=15.0,
            xy_y_mode=y_mode,
        )
        assert len(rgba) == 1280 * 680 * 4
        assert any(b != 0 for b in rgba[:4096]), f"Buffer vacio en modo {y_mode}"

    if (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")) and (
        select_backend("auto") == "wx"
    ):
        rc = run_wx_backend(
            tracker=tracker,
            send_cmd=lambda _cmd: True,
            on_close=lambda: None,
            fps=30,
            clockwise=True,
            zero_north=True,
            xy_window_s=15.0,
            xy_y_mode=XY_MODE_DEG_SINE,
            auto_close_ms=180,
            initial_show_config=True,
            initial_show_xy=True,
            initial_show_terminal=True,
        )
        assert rc == 0

    print(
        "SELF-TEST OK: parser P/PID, 3 escalas, X-Y en grados senoidales y Terminal Serie verificados."
    )
    return 0


def select_backend(requested: str) -> str:
    """Elige el backend grafico disponible."""
    if requested in ("wx", "gtk", "matplotlib"):
        return requested
    try:
        import wx  # noqa: F401
        import cairo  # noqa: F401

        return "wx"
    except Exception:
        pass
    try:
        import gi

        gi.require_version("Gtk", "3.0")
        from gi.repository import Gtk  # noqa: F401
        import cairo  # noqa: F401

        return "gtk"
    except Exception:
        pass
    try:
        import matplotlib.pyplot  # noqa: F401

        return "matplotlib"
    except Exception as exc:
        raise RuntimeError(
            "No se encontro ni wxPython+Cairo, ni GTK3+Cairo, ni Matplotlib para dibujar el grafico."
        ) from exc


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Grafica en tiempo real la posicion circular de la aguja del encoder."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Puerto serie del ESP32 (por defecto: {DEFAULT_PORT}).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Velocidad UART en baudios (por defecto: {DEFAULT_BAUD}).",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=DEFAULT_RATE_MS,
        help=f"Periodo de muestreo LOG RATE en ms (>=10, por defecto: {DEFAULT_RATE_MS}).",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=DEFAULT_FPS,
        help=f"Frecuencia de refresco grafico en Hz (por defecto: {DEFAULT_FPS}).",
    )
    parser.add_argument(
        "--trail",
        type=int,
        default=DEFAULT_TRAIL,
        help=f"Cantidad de muestras recientes para la estela (por defecto: {DEFAULT_TRAIL}).",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "wx", "gtk", "matplotlib"],
        default="auto",
        help="Backend grafico: auto (wxPython+Cairo por defecto), wx, gtk o matplotlib.",
    )
    parser.add_argument(
        "--no-init",
        action="store_true",
        help="No enviar LOG POSOUT OFF / LOG RATE / LOG POS ON al conectar.",
    )
    parser.add_argument(
        "--counter-clockwise",
        action="store_true",
        help="Rotar angulos positivos en sentido antihorario.",
    )
    parser.add_argument(
        "--zero-east",
        action="store_true",
        help="Ubicar 0 grados a la derecha (Este) en lugar de arriba (Norte / 12h).",
    )
    parser.add_argument(
        "--counts-per-rev",
        type=int,
        default=DEFAULT_COUNTS_PER_REV,
        help=f"Cuentas por vuelta del encoder para la escala circular y --demo (por defecto: {DEFAULT_COUNTS_PER_REV}).",
    )
    parser.add_argument(
        "--xy-window",
        type=float,
        default=DEFAULT_XY_WINDOW_S,
        help=f"Ventana inicial en segundos de la grafica X-Y (por defecto: {DEFAULT_XY_WINDOW_S}).",
    )
    parser.add_argument(
        "--xy-y-mode",
        choices=["counts", "sine", "deg360", "deg180", "accum"],
        default="counts",
        help="Escala Y inicial de la grafica X-Y: counts, sine (180*sin), deg360, deg180 o accum.",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Ejecuta una simulacion sin hardware serie conectado.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Ejecuta pruebas automaticas fuera de pantalla y termina.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return run_self_test()

    if args.rate < 10:
        raise SystemExit("--rate debe ser >= 10 ms (periodo de control del firmware).")

    tracker = TelemetryTracker(
        trail_len=args.trail, counts_per_rev=args.counts_per_rev
    )
    backend = select_backend(args.backend)

    if args.demo:
        source: object = DemoRunner(
            tracker=tracker, rate_ms=args.rate, counts_per_rev=args.counts_per_rev
        )
        source.start()  # type: ignore[attr-defined]
        send_cmd = source.send_command  # type: ignore[attr-defined]
        stop_source = source.stop  # type: ignore[attr-defined]
    else:
        reader = SerialReader(
            port=args.port,
            baud=args.baud,
            rate_ms=args.rate,
            tracker=tracker,
            init_stream=not args.no_init,
        )
        reader.start()
        send_cmd = reader.send_command
        stop_source = reader.stop

    clockwise = not args.counter_clockwise
    zero_north = not args.zero_east
    y_mode_map = {
        "counts": XY_MODE_COUNTS,
        "sine": XY_MODE_DEG_SINE,
        "deg360": XY_MODE_DEG_360,
        "deg180": XY_MODE_DEG_180,
        "accum": XY_MODE_DEG_ACCUM,
    }
    xy_y_mode = y_mode_map[args.xy_y_mode]

    try:
        if backend == "wx":
            return run_wx_backend(
                tracker=tracker,
                send_cmd=send_cmd,
                on_close=stop_source,
                fps=args.fps,
                clockwise=clockwise,
                zero_north=zero_north,
                xy_window_s=args.xy_window,
                xy_y_mode=xy_y_mode,
            )
        if backend == "gtk":
            return run_gtk_backend(
                tracker=tracker,
                send_cmd=send_cmd,
                on_close=stop_source,
                fps=args.fps,
                clockwise=clockwise,
                zero_north=zero_north,
                xy_window_s=args.xy_window,
                xy_y_mode=xy_y_mode,
            )
        return run_matplotlib_backend(
            tracker=tracker,
            on_close=stop_source,
            fps=args.fps,
            clockwise=clockwise,
            zero_north=zero_north,
        )
    finally:
        stop_source()


if __name__ == "__main__":
    raise SystemExit(main())
