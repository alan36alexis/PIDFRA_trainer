# Simulación en GNU Octave — `PIDFRA_trainer`

Este directorio implementa un **gemelo digital híbrido (tiempo continuo + control de datos muestreados)** del entrenador `PIDFRA_trainer` en **GNU Octave** nativo (sin requerir paquetes externos como `octave-control`), reproduciendo fielmente tanto el firmware en C del ESP32 ([`include/app_config.h`](../include/app_config.h), [`lib/motor_pid/motor_pid.c`](../lib/motor_pid/motor_pid.c), [`lib/motor_control/motor_control.c`](../lib/motor_control/motor_control.c), [`lib/pot_adc/pot_adc.c`](../lib/pot_adc/pot_adc.c)) como los fundamentos analíticos de [`manual_teoria_control_discreto.md`](../manual_teoria_control_discreto.md).

---

## Estructura de archivos

| Archivo | Función |
|---|---|
| `sim_pidfra.m` | Script principal: ejecuta el análisis en el plano $\mathcal{Z}$, corre todos los escenarios no lineales, imprime el reporte de métricas y exporta las 6 figuras en `octave/output/`. |
| `test_sim_pidfra.m` | Suite automatizada de pruebas unitarias y de integración con aserciones (`assert`) sobre PID, planta, discretización ZOH, modos de control y fallas de seguridad. |
| `pidfra_params.m` | Parámetros centralizados del firmware (`T_s = 10 ms`, `3600 cuentas/vuelta`, `MINMOVE = 185`, ganancias `SPEED`, `POS`, `CLOCK`, rangos ADC) y de la planta física continua. |
| `motor_pid_init.m` / `motor_pid_step.m` | Réplica exacta de `motor_pid.c`: PID discreto por Euler regresivo, supresión de *derivative kick* inicial y *anti-windup* por integración condicional (*clamping*). |
| `motor_plant_step.m` | Integración continua sub-muestreada ($\Delta t = 0{,}5\text{ ms}$) del motor DC + puente H L298N con fricción estática (*stiction*), fricción cinética de Coulomb (*stick-slip*), amortiguamiento viscoso ($\tau_m$) y cuantización espacial PCNT en cuadratura $\times 4$. |
| `pot_adc_step.m` | Réplica de `pot_adc.c`: ADC SAR de 12 bits (`0..4095`), sobremuestreo $\times 8$, filtro recursivo IIR EMA ($\alpha = 0{,}25$, $T_f = 50\text{ ms}$), *zero-snap* y *deadband* del $1\%$. |
| `pidfra_simulate.m` | Motor de simulación a $100\text{ Hz}$ de la máquina de estados (`FSM_IDLE`, `FSM_SPEED`, `FSM_POSITION`, `FSM_CLOCK`, `FSM_OPEN_LOOP`, `FSM_FAULT`) y sus dos supervisores de seguridad (`STALL` y límite de `1,5` vueltas sin establecer consigna). |
| `pidfra_discrete_analysis.m` | Discretización exacta con retenedor ZOH de $G_v(z)$ (Tipo 0) y $G_p(z)$ (Tipo 1), cálculo del cero de muestreo $z_0 = -b_0/b_1$, polos de lazo cerrado, Lugar de las Raíces discreto y respuesta en frecuencia (ZOH y filtro EMA). |
| `pidfra_step_metrics.m` | Cálculo automático de tiempo de retardo ($t_d$), tiempo de subida ($t_r$), sobrepico ($M_p\%$), tiempo de establecimiento con permanencia ($t_s$) y error estacionario ($e_{ss}$). |
| `pidfra_scenarios.m` | Definición y ejecución de los 6 bancos de ensayo del sistema. |
| `pidfra_plot_suite.m` | Generación y exportación PNG de los 6 paneles gráficos diagnósticos. |
| `pidfra_interactive_cli.m` | Consola interactiva en Octave que emula los comandos serie del ESP32 (`ANG`, `POS`, `SPEED`, `CLOCK`, `DUTY`, `PID`, `MINMOVE`). |

---

## Ejecución rápida

### 1. Simulación completa y generación de gráficas PNG
Desde la **raíz del repositorio** (`PIDFRA_trainer/`):

```bash
octave --no-gui octave/sim_pidfra.m
```

O si ya estás **dentro de la carpeta `octave/`**:

```bash
octave --no-gui sim_pidfra.m
```

*(Nota: en Ubuntu también puedes usar `octave-cli sim_pidfra.m` para evitar el aviso de Qt en consola).*

Para abrir además las ventanas interactivas de figuras en escritorio:

```bash
# Desde la raíz del repositorio:
octave octave/sim_pidfra.m --show

# Desde dentro de octave/:
octave sim_pidfra.m --show
```

O desde la ventana de comandos de la GUI de GNU Octave:

```matlab
run('octave/sim_pidfra.m')
```

### 2. Solo reporte numérico en consola (sin renderizar figuras)

```bash
octave --no-gui octave/sim_pidfra.m --no-plots
```

### 3. Emulador interactivo de comandos CLI (estilo puerto serie ESP32)

```bash
octave octave/sim_pidfra.m --no-plots --interactive
```

Dentro del prompt `PIDFRA>` puedes probar en vivo:

```text
PIDFRA> ANG 120
PIDFRA> CLOCK
PIDFRA> PID POS KI 1.8
PIDFRA> ANG 90
PIDFRA> MINMOVE 0
PIDFRA> ANG 90
PIDFRA> SPEED 8
PIDFRA> Q
```

### 4. Ejecutar el banco de pruebas automatizadas

```bash
octave --no-gui octave/test_sim_pidfra.m
```

---

## Figuras generadas (`octave/output/`)

1. **`01_posicion_y_nolinealidades.png`**: Seguimiento multi-escalón en `FSM_POSITION` (`90° → 180° → 45°`), esfuerzo ZOH con precompensación `MINMOVE = 185`, demostración de ciclo límite (*stick-slip / hunting*) al anular `MINMOVE = 0` y demostración de *integral windup* al desactivar el *clamping*.
2. **`02_modo_clock_segundero.png`**: Cuadrante circular de 60 divisiones estilo aguja segundera (`6°/s`), escalera temporal de 10 pasos y detalle de pulsos PWM con descarga del integrador al entrar en la banda de tolerancia (`±8 cuentas` / `±0,8°`).
3. **`03_control_velocidad_y_cuantizacion.png`**: Operación a baja velocidad (`8 RPM`) evidenciando el cuanto mínimo del Método $M$ ($\Delta\omega_{\min} = 1{,}67\text{ RPM/cuenta}$) y comparación a `35 RPM` con perturbación de par de carga entre la ley de piso del firmware y la precompensación aditiva (*feedforward*).
4. **`04_calibracion_umbral_y_lazo_abierto.png`**: Curva estática de zona muerta e histéresis estática/cinética en `FSM_OPEN_LOOP` (emulando `scripts/find_open_loop_threshold.py` por barrido + búsqueda binaria) e identificación experimental de $K_m$ y $\tau_m$ al $63{,}2\%$.
5. **`05_potenciometros_adc_y_seguridad_fsm.png`**: Cadena de filtrado de potenciómetros (`ADC 12-bit → media x8 → EMA α=0,25 → deadband 1%`) y disparo de las dos protecciones de `FSM_FAULT`: eje trabado (`STALL` a `300 ms + 500 ms = 0,80 s`) y exceso de recorrido sin establecer consigna (`1,5 vueltas = 5400 cuentas`).
6. **`06_plano_z_y_respuesta_frecuencia.png`**: Polos de lazo cerrado de `SPEED`, `POS` y `CLOCK` sobre el círculo unitario, cero intrínseco de muestreo $z_0 = -b_0/b_1$, Lugar de las Raíces discreto y erosión de fase del retenedor ZOH frente al filtro EMA.
