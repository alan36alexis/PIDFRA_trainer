# TODO - Evolucion del sistema

## 1. Parametros PID dinamicos

- [ ] Definir una estructura central de parametros de control:
  - `Kp`
  - `Ki`
  - `Kd`
  - `setpoint`
  - limites de salida
  - periodo de muestreo
- [ ] Separar los parametros de velocidad y posicion, si el ajuste requiere
      ganancias independientes.
- [ ] Agregar eventos FSM para actualizar parametros sin ejecutar cambios
      directamente desde `serial_task`.
- [ ] Validar rangos, valores finitos y comportamiento ante valores invalidos.
- [ ] Reiniciar el estado integral del PID cuando un cambio de parametros lo
      requiera.
- [ ] Definir si los parametros deben persistir despues de reiniciar el ESP32.
- [ ] Evaluar almacenamiento no volatil (NVS) para parametros persistentes.

## 2. Ajuste de parametros mediante CLI

- [ ] Agregar comandos mnemotecnicos para leer y escribir parametros, por
      ejemplo:

      ```text
      PID GET
      PID KP <valor>
      PID KI <valor>
      PID KD <valor>
      SETPOINT <valor>
      ```

- [ ] Definir si los comandos aplican al PID de velocidad, al de posicion o a
      ambos.
- [ ] Agregar un comando para restaurar valores por defecto.
- [ ] Mostrar confirmaciones compactas (`OK`, `ERR`) y no imprimir desde la
      tarea de control.
- [ ] Actualizar `HELP`, `README.md` y la tabla de eventos de la FSM.

## 3. Ajuste mediante potenciometros ADC

Se incorporaran cuatro potenciometros:

| Potenciometro | Funcion |
|---|---|
| `KP` | Ganancia proporcional |
| `KI` | Ganancia integral |
| `KD` | Ganancia derivativa |
| `SP` | Setpoint |

- [x] Seleccionar cuatro GPIO ADC validos del ESP32.
- [x] Confirmar que los GPIO elegidos no entren en conflicto con UART, PWM,
      PCNT, I2C o el switch.
- [x] Implementar lectura ADC con el driver recomendado por la version de
      ESP-IDF usada por el proyecto.
- [x] Definir atenuacion, resolucion, filtrado y frecuencia de muestreo ADC.
- [x] Convertir cada lectura al rango de control correspondiente.
- [x] Aplicar filtrado para evitar cambios bruscos por ruido.
- [x] Definir prioridad entre parametros recibidos por CLI y potenciometros.
- [x] Agregar una politica de seleccion:
  - CLI fija parametros manualmente.
  - ADC ajusta parametros en tiempo real.
  - o un comando/estado selecciona la fuente activa.
- [x] Generar eventos de actualizacion de parametros hacia la FSM/controlador.
- [x] Evitar que el ruido ADC produzca reinicios continuos del PID.

## 4. Display LCD 8x2 Tinsharp TC802A-01(R) en modo paralelo de 4 bits

- [x] Seleccionar GPIO seguros segun `pinout_esp32_pidfra_trainer.md` (`RS=32, E=33, D4=25, D5=26, D6=27, D7=13`, `R/W` fijo a `GND`) y centralizarlos en `include/board_pins.h`.
- [x] Reemplazar `lib/st7565/` por libreria modular e independiente en `lib/tc802a/` separando HAL (`tc802a_hal.h` / `tc802a_hal_espidf.c`) y driver del controlador (`tc802a_driver.h` / `tc802a_driver.c`).
- [x] Implementar secuencia de inicializacion HD44780/SPLC780D en 4 bits, buffer local de 8x2 caracteres, `tc802a_flush()` con direccionamiento DDRAM explicito (`0x00`/`0x40`) y rutina de bring-up (bloques `0xFF` + patron `PIDFRA82` / `12345678`).
- [x] Definir una tarea exclusiva en FreeRTOS (`display_task`, cola `lcd_request_queue`) para ejecutar el bring-up y actualizar las vistas 8x2.
- [x] No escribir el display desde `serial_task` ni desde `control_task`.
- [x] Mostrar informacion compacta en 8x2: pantalla principal fija con modo activo en `line0` (`M:SPEED`, `M:POSIC`, `M:CLOCK`, `M:OPEN`) y `SP` en `line1` sin alternar, ademas del menu interactivo (`->MODE, SP, KP, KI, KD`) con timeout de `5 s`.
- [x] Limitar la frecuencia de refresco (`LCD_REFRESH_PERIOD_MS = 100 ms` / 10 Hz) y adaptar comandos CLI `LCD`.
- [ ] Validar en hardware real contraste (`V0`), resistencia del backlight y legibilidad del display 8x2.

## 5. Encoder rotativo EC11 de interfaz y menu de parametros/modo

- [x] Configurar `PIN_EC11_CLK` (`GPIO17`), `PIN_EC11_DT` (`GPIO16`) en **PCNT Unidad 1** con `INPUT_PULLUP` y `PIN_EC11_SW` (`GPIO4`) con `INPUT_PULLUP`.
- [x] Definir logica activa en bajo del pulsador `SW` con antirrebote por software (`EC11_SW_DEBOUNCE_TICKS`).
- [x] Navegar con el giro del EC11 entre `->MODE`, `->SP`, `->KP`, `->KI`, `->KD` y habilitar su modificacion al presionar `SW`.
- [x] Al elegir `MODE`, navegar entre los modos con el EC11, aceptar con `SW` (`CONFIRM?`) y confirmar presionando `SW` nuevamente.
- [x] Al elegir `SP`, `KP`, `KI` o `KD` con `SW`, aplicar inmediatamente las variaciones detectadas en su trimpot (con filtro EMA-IIR y limites `[min, max]`) sin pedir confirmacion con `SW`, reiniciando el timeout de `5 s` en cada cambio y volviendo a la pantalla principal tras `5 s` sin cambios.
- [x] Aplicar un timeout de `5 s` (`UI_MENU_TIMEOUT_MS = 5000 ms`) en todos los menus para volver automaticamente a la pantalla principal si no hay input.
- [x] Mantener compatibilidad con comando `MODE` por UART, detener el motor, reiniciar los PID y reiniciar el contador del encoder al cambiar de modo.
- [ ] Probar en hardware real deteccion de *detents* y respuesta del pulsador `SW`.

## 6. Cero de posicion al arranque

- [ ] Leer el contador del encoder despues de inicializar PCNT.
- [ ] Establecer esa lectura como referencia mecanica inicial:

      ```text
      posicion_inicial_encoder = posicion_actual_encoder
      angulo_actual = 0 grados
      ```

- [ ] Guardar un offset de posicion separado del contador acumulado.
- [ ] Calcular la posicion relativa como:

      ```text
      posicion_relativa = contador_actual - offset_inicial
      ```

- [ ] Hacer que `POS` y `ANG` trabajen sobre la posicion relativa.
- [ ] Definir si el cero se toma automaticamente en cada reset o si tambien
      existira un comando manual `ZERO`.
- [ ] Agregar un evento `EVENT_POSITION_ZERO` si se implementa el comando
      manual.
- [ ] Documentar que no existe referencia absoluta del rotor sin sensor de
      index o procedimiento mecanico de homing.

## 7. Integracion y validacion

- [x] Actualizar la tabla de GPIO del `README.md`.
- [ ] Actualizar estados/eventos y transiciones de la FSM.
- [x] Mantener una unica tarea propietaria de UART.
- [x] Mantener una unica tarea propietaria del display/SPI (`display_task`).
- [x] Definir la sincronizacion de parametros compartidos entre tareas usando
      eventos, colas o secciones criticas.
- [ ] Verificar que el control continue ejecutandose aunque el LCD, ADC o CLI
      tengan actividad.
- [ ] Probar encoder desconectado, motor trabado, ruido ADC y rebote del
      switch.
- [x] Ejecutar:

      ```text
      pio run
      ```

## 8. Guardado dinamico en memoria no volatil (NVS)

- [x] Implementar una funcion de guardado dinamico en memoria no volatil (usando la API NVS de ESP-IDF).
- [x] Definir los parametros de configuracion a persistir:
  - Ganancias PID de velocidad (`Kp`, `Ki`, `Kd`).
  - Ganancias PID de posicion (`Kp`, `Ki`, `Kd`).
  - `min_move_duty` (PWM minimo de arranque).
  - Cuentas por revolucion / calibracion del encoder.
  - Offset de referencia de posicion cero.
- [x] Agregar comandos CLI para la gestion de la memoria (por ejemplo `SAVE`, `LOAD`, `NVS RESET`).
- [x] Implementar lectura y aplicacion automatica de los parametros guardados durante el arranque (`app_main`).
- [x] Validar integridad (magic number, version de esquema o checksum) para restaurar valores por defecto si la memoria no esta inicializada o esta corrupta.
- [x] Aislar las operaciones de escritura NVS de la tarea de control en tiempo real (`control_task`) para evitar bloqueos por operaciones de Flash.

