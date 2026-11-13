# Control de motor ESP32 con FSM, PID, encoder y CLI

<img width="2048" height="2110" alt="Gemini_Generated_Image_z089a2z089a2z089" src="https://github.com/user-attachments/assets/30b1ff98-4123-4c27-b872-b10540740067" />
<img width="2048" height="2110" alt="Gemini_Generated_Image_boxrbboxrbboxrbb" src="https://github.com/user-attachments/assets/f8f15f02-1505-432a-bac0-2c80745b917c" />

## Objetivo

Este proyecto controla un motor DC con puente H y encoder incremental sobre un ESP32 NodeMCU-32S. La estructura actual incluye:

- PWM por hardware LEDC para la salida de potencia.
- Control de sentido mediante GPIOs de dirección.
- Encoder incremental A/B en cuadratura con PCNT.
- FSM para estados de velocidad, posición, abierto y falla.
- Control PID independiente para velocidad y posición.
- Modo abierto (`OPEN`) con duty fijo manual.
- Umbral mínimo de arranque configurable (`MINMOVE`).
- Comando de pulso corto con desenergizado final para frenado libre.
- CLI por UART para ajuste en tiempo real.
- Telemetría compacta y script automatizado para buscar el umbral mínimo de movimiento.

El firmware principal está en [`src/main.c`](src/main.c). La librería PID portable está en [`lib/motor_pid/motor_pid.h`](lib/motor_pid/motor_pid.h) y [`lib/motor_pid/motor_pid.c`](lib/motor_pid/motor_pid.c). El script de calibración experimental se encuentra en [`scripts/find_open_loop_threshold.py`](scripts/find_open_loop_threshold.py).

## Plataforma y compilación

El proyecto usa PlatformIO + ESP-IDF.

- Board: `nodemcu-32s`
- Framework: `espidf`
- Puerto serie: `115200 8N1`

Comandos útiles:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

La advertencia de tamaño de flash puede aparecer si la placa reporta 2 MB y la configuración espera 4 MB; no afecta la lógica de control.

## Conexión eléctrica

| Función | GPIO ESP32 | Señal / uso |
|---|---:|---|
| Dirección 1 | GPIO22 | IN1 del puente H |
| Dirección 2 | GPIO23 | IN2 del puente H |
| PWM | GPIO21 | EN del puente H |
| Encoder A | GPIO18 | salida A del encoder |
| Encoder B | GPIO19 | salida B del encoder |
| Potenciómetro `KP` | GPIO36 (ADC1_CH0) | cursor potenciómetro lineal 0–3,3 V (`POT_KP_GPIO`) |
| Potenciómetro `KI` | GPIO39 (ADC1_CH3) | cursor potenciómetro lineal 0–3,3 V (`POT_KI_GPIO`) |
| Potenciómetro `KD` | GPIO34 (ADC1_CH6) | cursor potenciómetro lineal 0–3,3 V (`POT_KD_GPIO`) |
| Potenciómetro `SP` | GPIO35 (ADC1_CH7) | cursor potenciómetro lineal 0–3,3 V (`POT_SP_GPIO`) |

Recomendaciones:

- Mantener GND común entre ESP32, driver del motor y encoder.
- No conectar directamente señales de 5 V a GPIOs del ESP32.
- Asegurar alimentación suficiente para el motor y separación de la alimentación lógica si es necesario.

## Parámetros actuales del sistema

Los valores actuales del firmware están en [`src/main.c`](src/main.c):

```c
#define IN1_GPIO GPIO_NUM_22
#define IN2_GPIO GPIO_NUM_23
#define EN_GPIO GPIO_NUM_21
#define ENCODER_A_GPIO GPIO_NUM_18
#define ENCODER_B_GPIO GPIO_NUM_19
#define ENCODER_CYCLES_PER_CHANNEL 1237
#define QUADRATURE_FACTOR 4
#define CONTROL_PERIOD_MS 100
#define PWM_FREQUENCY_HZ 20000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define DEFAULT_MIN_MOVE_DUTY 160
```

Con la configuración actual:

```text
cuentas_por_vuelta = 1237 ciclos/canal * 4 = 4948 cuentas/vuelta
```

La relación de cuadratura x4 cuenta ambos flancos de A y B, por lo que la resolución efectiva es mayor que la resolución de un canal simple.

## PID por defecto y umbral mínimo de movimiento

Los PID por defecto están cargados al iniciar el firmware:

```c
static pid_config_t speed_pid_config = {0.12f, 0.10f, 0.002f};
static pid_config_t position_pid_config = {0.05f, 0.0f, 0.001f};
```

El umbral mínimo de arranque configurado por defecto es:

```c
static volatile uint8_t min_move_duty = DEFAULT_MIN_MOVE_DUTY;
#define DEFAULT_MIN_MOVE_DUTY 160
```

Esto representa el duty mínimo real que el sistema requiere para iniciar movimiento. Por debajo de ese valor, la salida del PID puede ser teóricamente válida pero no produce torque suficiente para mover el eje.

## FSM y estados

Los estados del sistema son:

| Estado | Código | Descripción |
|---|---:|---|
| `FSM_IDLE` | `I` | Motor detenido |
| `FSM_SPEED` | `V` | Control de velocidad |
| `FSM_POSITION` | `P` | Control de posición |
| `FSM_OPEN_LOOP` | `O` | Duty manual, sin lazo cerrado |
| `FSM_FAULT` | `F` | Falla por bloqueo |

Los eventos principales son:

- `EVENT_STOP`
- `EVENT_MODE_SPEED`
- `EVENT_MODE_POSITION`
- `EVENT_MODE_OPEN_LOOP`
- `EVENT_SPEED_SETPOINT`
- `EVENT_POSITION_SETPOINT`
- `EVENT_ANGLE_SETPOINT`
- `EVENT_DUTY_SETPOINT`
- `EVENT_ENCODER_STALL`
- `EVENT_CALIBRATE_START`
- `EVENT_POT_KP_UPDATE`
- `EVENT_POT_KI_UPDATE`
- `EVENT_POT_KD_UPDATE`
- `EVENT_POT_SP_UPDATE`

La lógica central es: `serial_task` valida los comandos y publica eventos; `potentiometer_task` muestrea los 4 canales ADC (`adc_oneshot`) con sobremuestreo y filtro EMA, aplica banda muerta (`POT_DEADBAND_NORM`) y publica eventos de actualización; `control_task` consume esos eventos, actualiza la FSM y aplica PWM o PID.

## Potenciómetros ADC (KP, KI, KD, SP)

La lectura de los 4 potenciómetros lineales (0–3,3 V) se ejecuta en una tarea dedicada de FreeRTOS (`potentiometer_task`, cada `POT_TASK_PERIOD_MS`) utilizando el driver `esp_adc/adc_oneshot.h`. Todas las equivalencias a constantes de PID y Setpoint están centralizadas como macros `#define` en [`src/main.c`](src/main.c) para facilitar su calibración empírica:

```c
/* Pines ADC1 */
#define POT_KP_GPIO GPIO_NUM_36
#define POT_KI_GPIO GPIO_NUM_39
#define POT_KD_GPIO GPIO_NUM_34
#define POT_SP_GPIO GPIO_NUM_35

/* Modo Velocidad (PID velocidad + SP en RPM) */
#define POT_SPEED_KP_MIN 0.00f
#define POT_SPEED_KP_MAX 1.00f
#define POT_SPEED_KI_MIN 0.00f
#define POT_SPEED_KI_MAX 0.50f
#define POT_SPEED_KD_MIN 0.000f
#define POT_SPEED_KD_MAX 0.050f
#define POT_SPEED_SP_MIN 0.0f
#define POT_SPEED_SP_MAX 3000.0f

/* Modo Posición (PID posición + SP en grados) */
#define POT_POS_KP_MIN 0.00f
#define POT_POS_KP_MAX 0.50f
#define POT_POS_KI_MIN 0.00f
#define POT_POS_KI_MAX 0.10f
#define POT_POS_KD_MIN 0.000f
#define POT_POS_KD_MAX 0.020f
#define POT_POS_SP_MIN 0.0f
#define POT_POS_SP_MAX 360.0f

/* Modo Lazo Abierto (SP en Duty PWM 0..255) */
#define POT_OPEN_SP_MIN 0.0f
#define POT_OPEN_SP_MAX ((float)PWM_MAX_DUTY)
```

## CLI y comandos serie

La UART opera a `115200 8N1`. Los comandos se envían como texto ASCII y pueden terminar en `\r`, `\n` o ambos.

### Ayuda

```text
HELP
?
```

### Estado del sistema

```text
STATUS
```

Ejemplo:

```text
STAT S=V RUN=1 FLT=0 POS=12.50 REF=1000.00
```

### Cambio de modo

```text
MODE SPEED
MODE POS
MODE OPEN
```

Alias compatibles:

```text
M SPEED
M POS
M OPEN
```

### Duty manual / modo abierto

```text
DUTY F 200
DUTY R 180
DUTY D 220
DUTY 0
```

- `F`: sentido adelante
- `R`: inverso
- `D`: mantiene el sentido anterior y cambia el duty
- `0`: detiene el motor

### Control de velocidad

```text
SPEED F 1000
SPEED R 500
SPEED D 750
```

- `F`: establece sentido adelante
- `R`: sentido reverse
- `D`: mantiene el sentido actual y cambia la referencia

### Control de posición y ángulo

```text
POS 4948
ANG 90
```

`POS` usa cuentas absolutas de encoder y `ANG` convierte grados a cuentas con:

```text
cuentas_objetivo = grados * cuentas_por_vuelta / 360
```

### Umbral mínimo de arranque

```text
MINMOVE 160
MIN_MOVE_DUTY 160
```

Esto ajusta el valor mínimo de duty que se considera necesario para mover el motor real. El valor se puede modificar en tiempo real sin recompilar.

### Pulso de movimiento por ciclos

```text
PULSE F 10 220
PULSE R 8 180
PULSE F 6 160 12000
```

Formato:

```text
PULSE <F|R> <n_cycles> <duty> [freq_hz]
```

Límites operativos recomendados para este proyecto:

- `duty`: `0..255`
- `n_cycles`: `1..4294967295` (valor entero sin signo)
- `freq_hz`: `1..40000` (el valor por defecto del sistema es `20000`)
- rango útil para un motor DC con LEDC: `1 kHz..25 kHz`, usando `20 kHz` como referencia estándar

Esto ejecuta una secuencia de `n_cycles` a un duty dado en la dirección elegida, con la frecuencia PWM indicada para ese pulso, y luego desenergiza el motor para dejarlo frenar libremente. El comando se implementa mediante la librería de pulso en [`lib/motor_pid/motor_pid.h`](lib/motor_pid/motor_pid.h).

### Ajuste del PID por serie

```text
PID GET
PID GET SPEED
PID GET POS

PID SPEED KP 0.12
PID SPEED KI 0.10
PID SPEED KD 0.002

PID POS KP 0.05
PID POS KI 0.0
PID POS KD 0.001

PID DEFAULT
```

- `PID GET` devuelve ambos modos.
- `PID SPEED ...` ajusta solo velocidad.
- `PID POS ...` ajusta solo posición.
- `PID DEFAULT` recupera la configuración inicial.

### Potenciómetros ADC por serie

```text
POT ON
POT OFF
POT GET
```

- `POT ON`: habilita la actualización automática de PID y SP desde los potenciómetros.
- `POT OFF`: deshabilita las actualizaciones desde potenciómetros (permite ajuste exclusivo por CLI sin sobrescritura).
- `POT GET`: muestra estado (`EN`), modo activo (`V`, `P`, `O`), tensión leída (V) y valor equivalente mapeado de `KP`, `KI`, `KD` y `SP`.

### Logging y telemetría

```text
LOG ON
LOG OFF
LOG RATE 500
```

Los mensajes de telemetría pueden verse como:

```text
TEL V R=1000.0 M=985.2 D=120
TEL P R=90.00 A=88.74 D=64
TEL O D=180
```

- `V`: velocidad
- `P`: posición
- `O`: open-loop
- `R`: referencia
- `M`: RPM medido
- `A`: ángulo medido
- `D`: duty PWM

### Detener y reiniciar fallas

```text
STOP
```

`STOP` desactiva el motor, reinicia PID y limpia la falla de encoder. Si se detecta bloqueo de encoder, el sistema entra en `FSM_FAULT` y requiere `STOP` para reactivar nuevos comandos.

## Script de calibración experimental

Se incluye un script para detectar experimentalmente el duty mínimo que hace mover el motor en modo abierto:

```bash
python3 scripts/find_open_loop_threshold.py --port /dev/ttyUSB0 --direction F --step 5 --start-duty 0 --max-duty 255
```

También permite:

```bash
python3 scripts/find_open_loop_threshold.py --help
```

El script:

1. Pone el sistema en `MODE OPEN`.
2. Prueba duties crecientes.
3. Monitorea el cambio de posición del encoder.
4. Detecta el primer movimiento.
5. Refina el valor con búsqueda binaria alrededor del umbral.

Esto es útil para encontrar el piso de torque real del sistema antes de ajustar PID en lazo cerrado.

## Técnica recomendada para sintonizar el sistema

La estrategia práctica para determinar las constantes es:

1. Medir el duty mínimo real con el script de umbral.
2. Ajustar `MINMOVE` a un valor ligeramente superior al umbral encontrado.
3. Usar `MODE OPEN` y `DUTY F/R ...` para comprobar respuesta del motor sin lazo cerrado.
4. Especificar ganancias conservadoras para velocidad:
   - empezar con `KP` moderado,
   - luego `KI` pequeña,
   - `KD` mínima o nula si el sistema es ruidoso.
5. Ajustar posición con un modelo más suave y evaluar el seguimiento en ángulos o cuentas.
6. Validar con `PULSE F/R <n_cycles> <duty> [freq_hz]` para asegurar que el arranque, la frecuencia de conmutación y el frenado libre sean estables.

Una regla útil: no se sintoniza PID sobre un sistema que todavía no supera su zona muerta real; primero se calibra el arranque y luego se afinan las ganancias del lazo cerrado.

## Librería PID portable

[`lib/motor_pid/motor_pid.h`](lib/motor_pid/motor_pid.h) no depende de ESP-IDF ni de FreeRTOS. La estructura es portable y expone funciones como:

```c
void motor_pid_init(motor_pid_t *pid, float kp, float ki, float kd,
                    float output_min, float output_max);
void motor_pid_reset(motor_pid_t *pid);
float motor_pid_update(motor_pid_t *pid, float setpoint, float measurement,
                       float sample_time_seconds);
void motor_pid_run_cycles(int direction, uint32_t n_cycles, uint8_t duty,
                          uint32_t cycle_ms, uint32_t pwm_frequency_hz,
                          motor_pid_drive_cb drive_cb,
                          motor_pid_set_pwm_cb set_pwm_cb,
                          motor_pid_release_cb release_cb,
                          motor_pid_wait_ms_cb wait_ms_cb);
```

El PID es discreto y usa tiempo de muestreo en segundos. La implementación mantiene también un control básico para evitar saturación prolongada del integrador.

## Consideraciones finales

- Mantener `serial_task` como único dueño de UART.
- Mantener la FSM y la cola de eventos como mecanismo central de control.
- Actualizar este README cada vez que cambie la estrategia de control, los comandos, los GPIO o las constantes de tuning.
- Validar siempre con `pio run` después de cambios relevantes.
