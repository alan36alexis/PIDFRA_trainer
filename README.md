# Control de motor ESP32 con FSM, PID, encoder y CLI

<img width="2048" height="2110" alt="Gemini_Generated_Image_z089a2z089a2z089" src="https://github.com/user-attachments/assets/30b1ff98-4123-4c27-b872-b10540740067" />
<img width="2048" height="2110" alt="Gemini_Generated_Image_boxrbboxrbboxrbb" src="https://github.com/user-attachments/assets/f8f15f02-1505-432a-bac0-2c80745b917c" />

## Objetivo

Este proyecto controla un motor DC con puente H y encoder incremental sobre un ESP32 NodeMCU-32S. La estructura actual incluye:

- PWM por hardware LEDC para la salida de potencia.
- Control de sentido mediante GPIOs de dirección.
- Encoder incremental A/B en cuadratura con PCNT.
- FSM para estados de velocidad, posición, clock, abierto y falla.
- Control PID independiente para velocidad, posición y modo clock.
- Modo demostración automática (`CLOCK`) tipo aguja segundera de reloj analógico (60 divisiones por vuelta, avanzando 1 división / 6° cada 1 segundo).
- Modo abierto (`OPEN`) con duty fijo manual.
- Umbral mínimo de arranque configurable (`MINMOVE`).
- Comando de pulso corto con desenergizado final para frenado libre.
- CLI por UART para ajuste en tiempo real.
- Telemetría compacta y script automatizado para buscar el umbral mínimo de movimiento.
- Driver modular e independiente para display LCD alfanumérico de 8×2 caracteres **Tinsharp TC802A-01(R)** (controlador HD44780 / SPLC780D en bus paralelo de 4 bits con `R/W` fijo a `GND`), con rutina de validación inicial (*bring-up*) por etapas, pantalla principal fija con el modo activo en la línea 0 (`M:SPEED`, `M:POSIC`, `M:CLOCK`, `M:OPEN`) y el Setpoint (`SP`) en la línea 1 (sin alternar), y menú interactivo con timeout de `5 s`.
- Encoder rotativo de interfaz **EC11** por hardware (**PCNT Unidad 1**) para navegar entre `->MODE`, `->SP`, `->KP`, `->KI` y `->KD`; al presionar `SW` permite editar el parámetro seleccionado mediante su trimpot en tiempo real (o navegar y aceptar/confirmar doblemente el modo con `SW`), volviendo automáticamente a la pantalla principal tras `5 s` sin actividad.

El firmware principal está en [`src/main.c`](src/main.c) y la asignación centralizada de pines en [`include/board_pins.h`](include/board_pins.h) (según [`pinout_esp32_pidfra_trainer.md`](pinout_esp32_pidfra_trainer.md)). La librería PID portable está en [`lib/motor_pid/motor_pid.h`](lib/motor_pid/motor_pid.h) y [`lib/motor_pid/motor_pid.c`](lib/motor_pid/motor_pid.c). El driver modular del display 8×2 está en [`lib/tc802a/tc802a_hal.h`](lib/tc802a/tc802a_hal.h), [`lib/tc802a/tc802a_hal_espidf.c`](lib/tc802a/tc802a_hal_espidf.c), [`lib/tc802a/tc802a_driver.h`](lib/tc802a/tc802a_driver.h) y [`lib/tc802a/tc802a_driver.c`](lib/tc802a/tc802a_driver.c). El script de calibración experimental se encuentra en [`scripts/find_open_loop_threshold.py`](scripts/find_open_loop_threshold.py).

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

La asignación definitiva de los 18 GPIOs está definida en [`pinout_esp32_pidfra_trainer.md`](pinout_esp32_pidfra_trainer.md) y centralizada en [`include/board_pins.h`](include/board_pins.h):

| Función | GPIO ESP32 | Señal / uso |
|---|---:|---|
| Potenciómetro `KP` | GPIO36 (`ADC1_CH0`) | cursor potenciómetro lineal 0–3,3 V (`PIN_POT_KP`) |
| Potenciómetro `KI` | GPIO39 (`ADC1_CH3`) | cursor potenciómetro lineal 0–3,3 V (`PIN_POT_KI`) |
| Potenciómetro `KD` | GPIO34 (`ADC1_CH6`) | cursor potenciómetro lineal 0–3,3 V (`PIN_POT_KD`) |
| Potenciómetro `SP` | GPIO35 (`ADC1_CH7`) | cursor potenciómetro lineal 0–3,3 V (`PIN_POT_SETPOINT`) |
| LCD `RS` (Pin 4) | GPIO32 | Selección registro LCD (`PIN_LCD_RS`) |
| LCD `E` (Pin 6) | GPIO33 | Habilitación LCD (`PIN_LCD_E`) |
| LCD `DB4` (Pin 11) | GPIO25 | Bit 4 bus paralelo (`PIN_LCD_D4`) |
| LCD `DB5` (Pin 12) | GPIO26 | Bit 5 bus paralelo (`PIN_LCD_D5`) |
| LCD `DB6` (Pin 13) | GPIO27 | Bit 6 bus paralelo (`PIN_LCD_D6`) |
| LCD `DB7` (Pin 14) | GPIO13 | Bit 7 bus paralelo (`PIN_LCD_D7`) |
| Encoder motor `A` | GPIO23 | Fase A 900 PPR (`PIN_ENC_MOTOR_A`, PCNT Unidad 0) |
| Encoder motor `B` | GPIO22 | Fase B 900 PPR (`PIN_ENC_MOTOR_B`, PCNT Unidad 0) |
| PWM motor `ENA` | GPIO21 | `ENA` del L298N (`PIN_MOTOR_ENA`, pull-down externo $10\text{ k}\Omega$) |
| Dirección `IN1` | GPIO19 | `IN1` del L298N (`PIN_MOTOR_IN1`) |
| Dirección `IN2` | GPIO18 | `IN2` del L298N (`PIN_MOTOR_IN2`) |
| EC11 `CLK` (A) | GPIO17 | Fase A encoder interfaz (`PIN_EC11_CLK`, PCNT Unidad 1, `INPUT_PULLUP`) |
| EC11 `DT` (B) | GPIO16 | Fase B encoder interfaz (`PIN_EC11_DT`, PCNT Unidad 1, `INPUT_PULLUP`) |
| EC11 `SW` | GPIO4 | Pulsador encoder interfaz (`PIN_EC11_SW`, `INPUT_PULLUP`) |

Recomendaciones:

- Mantener `GND` común entre ESP32, driver L298N, encoders y módulo LCD **TC802A-01(R)**.
- Fijar `R/W` (Pin 5 del LCD) directamente a `GND` (modo solo escritura) para impedir retornos de $5\text{ V}$ hacia los GPIOs del ESP32.
- No aplicar señales $> 3{,}3\text{ V}$ a ningún GPIO del ESP32 (adaptar el encoder 900 PPR si es *push-pull* de $5\text{ V}$ o colocar pull-up a $3{,}3\text{ V}$ si es *open-collector*).
- Instalar una resistencia pull-down externa de $10\text{ k}\Omega$ en `ENA` (`GPIO21`) para mantener el puente H deshabilitado durante reset y flasheo.
- Verificar resistencia limitadora en el PCB del LCD antes de energizar el *backlight* (`Pin 15/16`) y ajustar el contraste en `V0` (Pin 3) con trimpot de $10\text{ k}\Omega$.

## Parámetros actuales del sistema

Los valores actuales del firmware están en [`include/board_pins.h`](include/board_pins.h) y [`src/main.c`](src/main.c):

```c
#define IN1_GPIO ((gpio_num_t)PIN_MOTOR_IN1)   // GPIO19
#define IN2_GPIO ((gpio_num_t)PIN_MOTOR_IN2)   // GPIO18
#define EN_GPIO  ((gpio_num_t)PIN_MOTOR_ENA)   // GPIO21
#define ENCODER_A_GPIO ((gpio_num_t)PIN_ENC_MOTOR_A) // GPIO23
#define ENCODER_B_GPIO ((gpio_num_t)PIN_ENC_MOTOR_B) // GPIO22
#define ENCODER_CYCLES_PER_CHANNEL 900
#define QUADRATURE_FACTOR 4
#define CONTROL_PERIOD_MS 10
#define PWM_FREQUENCY_HZ 20000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define DEFAULT_MIN_MOVE_DUTY 170
```

Con la configuración actual:

```text
cuentas_por_vuelta = 900 ciclos/canal * 4 = 3600 cuentas/vuelta
```

La relación de cuadratura x4 cuenta ambos flancos de A y B, por lo que la resolución efectiva es mayor que la resolución de un canal simple.

## PID por defecto y umbral mínimo de movimiento

Los PID por defecto están definidos mediante macros en [`src/main.c`](src/main.c) y se cargan al iniciar el firmware (o desde NVS si ya existe una configuración guardada válida):

```c
#define DEFAULT_SPEED_KP 0.2500f
#define DEFAULT_SPEED_KI 0.3500f
#define DEFAULT_SPEED_KD 0.0050f

#define DEFAULT_POS_KP 0.4000f
#define DEFAULT_POS_KI 0.2000f
#define DEFAULT_POS_KD 0.0150f

#define DEFAULT_CLOCK_KP 1.5000f
#define DEFAULT_CLOCK_KI 1.0000f
#define DEFAULT_CLOCK_KD 0.0150f
```

Es decir, el PID de posición inicia por defecto con `PID P KP=0.4000 KI=0.2000 KD=0.0150` y el PID dedicado del modo `CLOCK` inicia con `PID C KP=1.5000 KI=1.0000 KD=0.0150`.

El umbral mínimo de arranque configurado por defecto es:

```c
static volatile uint8_t min_move_duty = DEFAULT_MIN_MOVE_DUTY;
#define DEFAULT_MIN_MOVE_DUTY 170
```

Esto representa el duty mínimo real que el sistema requiere para iniciar movimiento. Por debajo de ese valor, la salida del PID puede ser teóricamente válida pero no produce torque suficiente para mover el eje.

## FSM y estados

Los estados del sistema son:

| Estado | Código | Descripción |
|---|---:|---|
| `FSM_IDLE` | `I` | Motor detenido |
| `FSM_SPEED` | `V` | Control de velocidad |
| `FSM_POSITION` | `P` | Control de posición |
| `FSM_CLOCK` | `C` | Demostración automática tipo aguja segundera (60 divisiones, 6°/s) |
| `FSM_OPEN_LOOP` | `O` | Duty manual, sin lazo cerrado |
| `FSM_FAULT` | `F` | Falla por bloqueo o límite de 1,5 vueltas sin setpoint |

Los eventos principales son:

- `EVENT_STOP`
- `EVENT_MODE_SPEED`
- `EVENT_MODE_POSITION`
- `EVENT_MODE_CLOCK`
- `EVENT_MODE_OPEN_LOOP`
- `EVENT_SPEED_SETPOINT`
- `EVENT_POSITION_SETPOINT`
- `EVENT_ANGLE_SETPOINT`
- `EVENT_DUTY_SETPOINT`
- `EVENT_ENCODER_STALL`
- `EVENT_SETPOINT_TIMEOUT_REVS`
- `EVENT_CALIBRATE_START`
- `EVENT_POT_KP_UPDATE`
- `EVENT_POT_KI_UPDATE`
- `EVENT_POT_KD_UPDATE`
- `EVENT_POT_SP_UPDATE`

La lógica central es: `serial_task` valida los comandos y publica eventos; `potentiometer_task` muestrea los 4 canales ADC (`adc_oneshot` con calibración *eFuse* por *line fitting* cuando está disponible), aplica sobremuestreo, filtro digital EMA-IIR, banda muerta (`POT_DEADBAND_NORM`) y acotación estricta `[min, max]` para aplicar de inmediato los cambios del parámetro habilitado en edición (`SP`, `KP`, `KI` o `KD`) reiniciando su timeout de `5 s`, además de muestrear el encoder rotativo de interfaz **EC11** (PCNT Unidad 1 y pulsador `SW` con antirrebote); `control_task` consume los eventos a $100\text{ Hz}$ (`10 ms`), actualiza la FSM y aplica PWM o PID leyendo el encoder del motor en **PCNT Unidad 0**; `nvs_task` gestiona de forma asíncrona el almacenamiento no volátil en Flash (NVS); y `display_task` actúa como única propietaria de los 6 GPIOs del display LCD **TC802A-01(R) 8×2**, mostrando en la pantalla principal el modo activo en `line0` y el `SP` en `line1` (sin alternar) o el estado actual del menú EC11.

## Almacenamiento no volátil (NVS) en FreeRTOS

La persistencia de configuración se realiza mediante una tarea dedicada de FreeRTOS (`nvs_task`, prioridad `NVS_TASK_PRIORITY`, cola `nvs_request_queue`) usando la API `nvs_flash` de ESP-IDF. De este modo, las operaciones de borrado/escritura en memoria Flash quedan completamente aisladas de `control_task`.

Parámetros persistidos en la estructura `nvs_stored_config_t` (namespace `pidfra_cfg`, clave `sys_cfg`, con magic number `0x50494446`, versión de esquema y verificación CRC32):

- Constantes PID de velocidad (`speed_pid`: `KP`, `KI`, `KD`).
- Constantes PID de posición (`position_pid`: `KP`, `KI`, `KD`).
- Constantes PID de modo clock (`clock_pid`: `KP`, `KI`, `KD`).
- Umbral mínimo de arranque (`min_move_duty`).
- Estado de potenciómetros (`pot_enabled`).
- Estado y período de telemetría (`telemetry_enabled`, `telemetry_period_ms`).
- Parámetros de calibración del encoder (`encoder_cycles_per_channel`, `encoder_zero_offset_counts`).

Al iniciar (`app_main`), `nvs_storage_init_and_load()` lee y valida el bloque NVS; si no existe o está corrupto, restaura y guarda los valores por defecto (incluyendo `PID P KP=0.4000 KI=0.2000 KD=0.0150`). Además, los cambios realizados por CLI activan un auto-guardado con debounce (`NVS_AUTOSAVE_DEBOUNCE_MS = 2000 ms`) que solo escribe en Flash si la configuración cambió.

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
#define POT_SPEED_KI_MAX 1.00f
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

/* Modo Clock (PID Clock dedicadas; trimpots KP/KI/KD activos, SP ignorado en CLOCK) */
#define POT_CLOCK_KP_MIN 0.00f
#define POT_CLOCK_KP_MAX 0.50f
#define POT_CLOCK_KI_MIN 0.00f
#define POT_CLOCK_KI_MAX 0.10f
#define POT_CLOCK_KD_MIN 0.000f
#define POT_CLOCK_KD_MAX 0.020f

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
MODE CLOCK
```

Alias compatibles:

```text
M SPEED
M POS
M OPEN
M CLOCK
M C
CLOCK
CLK
```

Al intercambiar entre modos de operación (por CLI `MODE`, cambio implícito mediante setpoint o menú del EC11), el sistema detiene el motor, reinicia los controladores PID y **reinicia el contador acumulado del encoder** (`encoder_position_counts = 0` junto con `pcnt_unit_clear_count`) para evitar que los pulsos acumulados en velocidad, lazo abierto o `CLOCK` generen error al pasar a modo posición.

### Modo demostración automática (`CLOCK`)

Al activar `MODE CLOCK` (o `M CLOCK`, `M C`, `CLOCK`, `CLK`, o mediante el encoder EC11), el sistema reinicia el contador del encoder a cero como base y avanza automáticamente 1 división cada 1 segundo sobre una circunferencia de 60 divisiones (6° por segundo, 1 vuelta completa cada 60 segundos), usando el controlador PID dedicado `clock_pid`. En este modo el trimpot de `SP` se ignora para no interferir con la secuencia automática de 1 paso/s, mientras que `KP`, `KI` y `KD` continúan aceptando ajustes con sus límites dedicados.

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
POS 3600
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
PID GET CLOCK

PID SPEED KP 0.12
PID SPEED KI 0.10
PID SPEED KD 0.002

PID POS KP 0.4000
PID POS KI 0.2000
PID POS KD 0.0150

PID CLOCK KP 1.5000
PID CLOCK KI 1.0000
PID CLOCK KD 0.0150

PID DEFAULT
```

- `PID GET` devuelve los tres modos (`V`, `P` y `C`).
- `PID SPEED ...` ajusta solo velocidad.
- `PID POS ...` ajusta solo posición.
- `PID CLOCK ...` (alias `CLK`, `C`) ajusta solo las constantes PID dedicadas del modo `CLOCK`.
- `PID DEFAULT` recupera la configuración inicial.

### Potenciómetros ADC por serie

```text
POT ON
POT OFF
POT GET
```

- `POT ON`: habilita la actualización automática de PID (`KP`, `KI`, `KD`) y `SP` desde los potenciómetros con filtro digital EMA-IIR y límites `[min, max]` por modo (estado por defecto `POT_DEFAULT_ENABLED = 1`; en `CLOCK` solo `SP` es ignorado).
- `POT OFF`: deshabilita las actualizaciones desde potenciómetros para permitir ajuste exclusivo por CLI sin sobrescritura.
- `POT GET`: muestra estado (`EN`), modo activo (`V`, `P`, `C`, `O`), tensión leída (V) y valor equivalente mapeado de `KP`, `KI`, `KD` y `SP`.

### Memoria no volátil (NVS) por serie

```text
SAVE
LOAD
NVS GET
NVS SAVE
NVS LOAD
NVS RESET
NVS AUTOSAVE ON
NVS AUTOSAVE OFF
```

- `SAVE` / `NVS SAVE`: solicita a `nvs_task` guardar inmediatamente la configuración actual en NVS.
- `LOAD` / `NVS LOAD`: recarga desde NVS la última configuración válida guardada.
- `NVS RESET`: restaura todos los parámetros persistidos a los valores por defecto y los guarda en NVS.
- `NVS GET`: muestra el estado de NVS (`READY`, `DIRTY`, `AUTO`, `SAVES`) y los valores actuales de PID, `MINMOVE`, `POT` y `LOG`.
- `NVS AUTOSAVE ON/OFF`: habilita o deshabilita el auto-guardado con debounce tras cambios por CLI.

### Logging y telemetría

```text
LOG ON
LOG OFF
LOG POS ON
LOG POS OFF
LOG POS
LOG POSOUT ON
LOG POSOUT OFF
LOG RATE 500
```

- `LOG ON` / `LOG OFF`: habilita o deshabilita la telemetría periódica (`TEL ...`).
- `LOG POS ON` / `LOG POS OFF` (alias `LOG P ON/OFF`): habilita o deshabilita de forma independiente el streaming periódico de posición instantánea acumulada del encoder al período configurado por `LOG RATE <ms>` (mínimo `10 ms`). Al activar `LOG POS ON`, se silencia automáticamente `POS OUT=...` para mantener limpio el stream `P,<counts>,<degrees>\n`.
- `LOG POS` (alias `LOG P`): imprime una lectura puntual instantánea de la posición acumulada del encoder.
- `LOG POSOUT ON` / `LOG POSOUT OFF` (alias `LOG OUT`, `LOG POS_OUT`, `LOG DBG`): habilita o deshabilita explícitamente los mensajes de diagnóstico `POS OUT=... ERR=...` en modo posición.

Los mensajes de telemetría pueden verse como:

```text
TEL V R=1000.0 M=985.2 D=120
TEL P R=90.00 A=88.74 D=64
TEL C R=6.00 A=5.95 D=170
TEL O D=180
```

Los mensajes de log instantáneo de posición (`LOG POS`) usan el formato `P,<counts>,<degrees>\n`:

```text
P,1800,180.00
```

- `V`: velocidad
- `P`: posición
- `C`: modo clock
- `O`: open-loop
- `R`: referencia
- `M`: RPM medido
- `A`: ángulo medido
- `D`: duty PWM

### Control del display LCD 8×2 TC802A-01(R)

```text
LCD STATUS
LCD GET
LCD BRINGUP
LCD PATTERN
LCD DASH
LCD PID
LCD ALLON ON
LCD ALLON OFF
LCD CURSOR ON
LCD CURSOR OFF
LCD BLINK ON
LCD BLINK OFF
LCD DISPLAY ON
LCD DISPLAY OFF
LCD CLEAR
```

- `LCD STATUS` / `LCD GET`: reporta estado de inicialización, vista activa (`PATTERN`, `DASH` o `PID`), flags (`DISP`, `CUR`, `BLINK`, `ALLON`), cantidad de frames renderizados, GPIOs asignados (`RS=32, E=33, D4=25, D5=26, D6=27, D7=13`) y el contenido actual de las 2 líneas (`L0` y `L1`).
- `LCD BRINGUP` (alias `LCD TEST`): re-ejecuta la secuencia diagnóstica completa de 3 etapas con logs por puerto serie.
- `LCD PATTERN`: fija el patrón de prueba de 8×2 (`PIDFRA82` / `12345678`).
- `LCD DASH` (alias `LCD DASHBOARD`): vista principal por defecto; muestra de forma fija en `line0` el modo activo (`M:SPEED `, `M:POSIC `, `M:CLOCK `, `M:OPEN  `, o `!FAULT! ` / `!STOP!  `) y en `line1` el Setpoint (`SP:1500r`, `SP: 180°`, `SP:S00/m`, `SP:D 180`, o `STALL/ST` / `>1.5V SP`), sin alternar con ninguna otra indicación.
- `LCD PID`: vista diagnóstica por CLI de las ganancias (`P`, `I`, `D`) y el código del modo activo en formato 8×2.
- `LCD ALLON <ON|OFF>`: rellena las 16 celdas con bloques sólidos (`0xFF`) para validar contraste (`V0`) o vuelve a la vista activa.
- `LCD CURSOR <ON|OFF>` / `LCD BLINK <ON|OFF>` / `LCD DISPLAY <ON|OFF>`: controla los bits `D/C/B` del controlador HD44780/SPLC780D.
- `LCD CLEAR` (alias `LCD CLS`): limpia el display por hardware y mantiene la pantalla vacía hasta seleccionar otra vista.

### Encoder rotativo de interfaz EC11 (`GPIO17, 16, 4`) y menú con timeout de 5 s

```text
EC11 GET
EC11 STATUS
EC11 ON
EC11 OFF
EC11 ZERO
```

- **Navegación del menú principal (`UI_STATE_MAIN_MENU`):** al girar el EC11 desde la pantalla principal se navega entre `->MODE`, `->SP`, `->KP`, `->KI` y `->KD` (mostrando el ítem en `line0` y su valor actual en `line1`).
- **Selección y doble confirmación de `MODE` (`UI_STATE_MODE_SELECT` → `UI_STATE_MODE_CONFIRM`):**
  1. Con `->MODE` en pantalla, al presionar `SW` se entra a `SEL MODO` y el giro del EC11 navega entre `>SPEED`, `>POSIC.`, `>CLOCK` y `>OPEN`.
  2. Al presionar `SW` por primera vez se acepta el modo elegido y la pantalla pasa a `CONFIRM?` / `SW:<MODO>`.
  3. Al presionar `SW` por segunda vez se confirma el cambio de modo hacia la FSM y se vuelve a la pantalla principal (`line0` = modo activo, `line1` = `SP`).
- **Edición inmediata de `SP`, `KP`, `KI` y `KD` por trimpot (`UI_STATE_EDIT_PARAM`):**
  - Al seleccionar `->SP`, `->KP`, `->KI` o `->KD` con `SW`, la pantalla indica `EDIT <PARAM>` en `line0` y su valor en tiempo real en `line1`.
  - Cualquier variación detectada en el trimpot del parámetro seleccionado (filtrada con EMA-IIR y acotada a `[min, max]`) se aplica **inmediatamente** sin pedir confirmación con `SW` y reinicia el timeout de `5 s`.
- **Timeout de 5 segundos (`UI_MENU_TIMEOUT_MS = 5000 ms`):** todos los menús (`MAIN_MENU`, `MODE_SELECT`, `MODE_CONFIRM` y `EDIT_PARAM`) vuelven automáticamente a la pantalla principal si transcurren `5 s` sin actividad en el EC11 ni variaciones en el trimpot activo.
- **Pulsación larga ($\ge 800\text{ ms}$) en `SW`:** ejecuta `STOP` y resetea el menú a la pantalla principal.
- `EC11 ON` / `EC11 OFF`: habilita o deshabilita la navegación por giro del encoder.
- `EC11 ZERO`: reinicia el acumulador de cuentas del EC11.

### Detener y reiniciar fallas

```text
STOP
```

`STOP` desactiva el motor, reinicia PID y limpia la falla activa. El sistema entra en `FSM_FAULT` ante dos protecciones máximas de seguridad:

1. **Bloqueo de encoder (`EVENT_ENCODER_STALL`):** emite `EVT STALL ENC/MOT; LOCK STOP` por serie y muestra `!FAULT! ` / `STALL/ST` en el LCD.
2. **Límite de 1,5 vueltas sin encontrar el Setpoint (`EVENT_SETPOINT_TIMEOUT_REVS`):** si en cualquiera de sus modos el motor recorre `1,5` vueltas completas (`5400` cuentas con encoder 900 PPR en cuadratura x4) sin estabilizar su referencia, se detiene de inmediato, emite `EVT SAFETY STOP: NO SP EN 1.5 VUELTAS (MODO=<...> REV=<...>); LOCK STOP` por puerto serie y muestra `!STOP!  ` / `>1.5V SP` en el LCD 8×2.

En ambos casos se requiere `STOP` (o pulsación corta de `SW` del EC11 / `CAL START`) para liberar el bloqueo y reactivar comandos.

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

## Gráfico circular en tiempo real de la posición de la aguja

Se incluye un script visual en tiempo real que consume el stream `P,<counts>,<degrees>\n` del ESP32 y dibuja la aguja sobre un cuadrante con **tres escalas circulares concéntricas** (`Counts` de encoder `0..3600`, `Grados` `0..360°` y `Segundos` `0..60s`):

```bash
python3 scripts/plot_needle_position.py --port /dev/ttyUSB0 --rate 20
```

Funciones integradas en la ventana:

- **Botón `STOP` (en rojo)**: detiene inmediatamente el motor enviando `STOP`.
- **Botón `Save Plot`**: exporta el último minuto de datos de la gráfica X-Y a un archivo CSV cuyo nombre refleja el rango horario guardado (`YYYY-MM-DD_HH-MM-SS_a_HH-MM-SS.csv`), con columnas `timestamp_iso,elapsed_s,counts,degrees,degrees_mod_360,degrees_sine,seconds_scale`.
- **`Terminal Serie`**: abre una ventana de terminal serie para enviar comandos crudos al CLI del ESP32 (`HELP`, `STATUS`, `PID GET`, `POT GET`, `NVS GET`, `CAL READ`, `LCD STATUS` o texto libre con historial ↑/↓) y ver las respuestas en tiempo real (con opción para incluir o filtrar las tramas `P,<counts>,<degrees>`).
- **`Config PID / Modos`**: despliega un panel para cambiar de modo (`SPEED`, `POS`, `CLOCK`, `OPEN`), habilitar/deshabilitar potenciómetros (`POT ON/OFF`), enviar consignas (`ANG`, `POS`, `SPEED`, `DUTY`), leer (`PID GET`) y aplicar ganancias `KP`/`KI`/`KD` por perfil, restaurar `PID DEFAULT`, ajustar `MINMOVE` y guardar en NVS (`SAVE`).
- **`Gráfica X-Y >>`**: despliega hacia la derecha una gráfica cartesiana interactiva en tiempo real (`Tiempo [s]` en el eje X vs `Counts` de encoder o `Grados` en el eje Y) con ventana temporal configurable (`5s`, `15s`, `30s`, `60s`), selector de escala Y (`Counts (encoder)`, `Grados Senoidal (180°·sin θ)` para trazar ondas senoidales continuas al girar, `Grados (0°..360°)`, `Grados (-180°..+180°)` y `Grados (acumulados)`), controles `⏸ Pausar`/`▶ En Vivo`, **pan arrastrando con botón izquierdo** sobre el historial, **zoom** con la rueda del ratón (`Rueda`=eje X, `Shift+Rueda`=eje Y, `Ctrl+Rueda`=XY, o botones `Zoom +`/`Zoom -`/`Reset Vista`), **cursores de medición `C1`/`C2`** (clic derecho para colocar, arrastrar para mover, checkbox `Snap Curva` y lectura de $\Delta t$, frecuencia y $\Delta Y$), línea de referencia objetivo y botón `Limpiar X-Y`.

Opciones útiles:

- `--demo`: ejecuta una simulación local sin necesidad de conectar el ESP32.
- `--xy-window <seg>`: ventana inicial de la gráfica X-Y en segundos (por defecto `15`).
- `--xy-y-mode <counts|sine|deg360|deg180|accum>`: escala Y inicial de la gráfica X-Y (`sine` grafica $180^\circ \cdot \sin\theta$).
- `--export-dir <ruta>`: carpeta de salida donde `Save Plot` guarda los CSV (por defecto `.`).
- `--backend <auto|wx|gtk|matplotlib>`: usa `wxPython + Cairo` por defecto en `auto` (incluido en el entorno), o permite elegir `gtk` (GTK3 + Cairo/Pixbuf) o `matplotlib` en proyección polar.
- `--no-init`: evita enviar automáticamente `LOG POSOUT OFF`, `LOG RATE <ms>` y `LOG POS ON` al abrir el puerto.
- `--help`: muestra todas las opciones disponibles.

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

## Display LCD 8×2 Tinsharp TC802A-01(R) (`lib/tc802a`)

La librería independiente en [`lib/tc802a/`](lib/tc802a/) implementa un driver modular para el módulo **Tinsharp TC802A-01(R)** (8 columnas × 2 filas, controlador compatible con HD44780 / SPLC780D en bus paralelo de 4 bits con `R/W` fijo a `GND`), separando la capa HAL del protocolo de instrucciones:

1. **Capa HAL (`tc802a_hal.h` / `tc802a_hal_espidf.c`):**
   - Interfaz abstracta `tc802a_hal_t` (`write_nibble`, `delay_us`, `delay_ms`, `ctx`).
   - Implementación ESP-IDF sobre los 6 GPIOs definidos en [`include/board_pins.h`](include/board_pins.h) (`RS=32`, `E=33`, `DB4=25`, `DB5=26`, `DB6=27`, `DB7=13`), asegurando nivel bajo inicial antes de habilitar salidas y pulso determinista de `E` ($\ge 2\ \mu\text{s}$ de *setup*, ancho de pulso y *hold*).
2. **Driver del controlador (`tc802a_driver.h` / `tc802a_driver.c`):**
   - **Secuencia de inicialización 4 bits:** espera $\ge 50\text{ ms}$, tres nibbles `0x03`, conmutación `0x02`, `0x28` (4 bits, 2 líneas, matriz 5×8), `0x08` (Display OFF), `0x01` (Clear Display), `0x06` (Entry Mode incremento) y `0x0C` (Display ON, Cursor OFF, Blink OFF).
   - **Buffer local de 8×2 caracteres:** `tc802a_write_line()`, `tc802a_set_char()`, `tc802a_create_char()` y `tc802a_flush()` con direccionamiento explícito de DDRAM (`0x00..0x07` en línea 0 y `0x40..0x47` en línea 1).
3. **Rutina de *bring-up* y tarea FreeRTOS (`display_task`):**
   - Al arrancar (`display_task`, prioridad `3`, cola `lcd_request_queue`), ejecuta las 3 etapas diagnósticas reportadas por `log_queue`:
     1. Inicialización en modo 4 bits del controlador HD44780/SPLC780D.
     2. Llenado completo de las 16 celdas con bloques sólidos (`0xFF`) durante `2000 ms` para validar contraste (`V0`) y todos los segmentos.
     3. Escritura del patrón de prueba `PIDFRA82` / `12345678` durante `3000 ms`, pasando luego automáticamente a la pantalla principal fija (`line0` = modo activo, `line1` = `SP`) refrescada a `10 Hz` (`100 ms`).

## Consideraciones finales

- Mantener `serial_task` como único dueño de la recepción/CLI por UART y `print_task` como única tarea *gatekeeper* propietaria de `printf` (`stdout` / UART TX).
- Mantener `display_task` como única propietaria de los pines del display LCD TC802A-01(R).
- Mantener la FSM y la cola de eventos como mecanismo central de control.
- Actualizar este README cada vez que cambie la estrategia de control, los comandos, los GPIO o las constantes de tuning.
- Validar siempre con `pio run` después de cambios relevantes.
