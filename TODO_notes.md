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

## 4. Display LCD 20x2 por I2C

- [ ] Seleccionar GPIO SDA y SCL libres.
- [ ] Confirmar direccion I2C del modulo LCD, normalmente `0x27` o `0x3F`.
- [ ] Implementar inicializacion del bus I2C y del controlador LCD.
- [ ] Definir una tarea exclusiva para actualizar el display.
- [ ] No escribir el display desde `serial_task` ni desde `control_task`.
- [ ] Mostrar informacion compacta, por ejemplo:
  - linea 1: modo, estado FSM y setpoint.
  - linea 2: posicion/RPM, duty o falla.
- [ ] Limitar la frecuencia de refresco para evitar parpadeos y bloqueo del
      bus I2C.
- [ ] Agregar mensajes abreviados para `FAULT`, encoder ausente y motor
      detenido.

## 5. Switch de seleccion de modo

- [ ] Seleccionar un GPIO de entrada con pull-up o pull-down definido.
- [ ] Definir la logica electrica activa del switch.
- [ ] Implementar antirrebote por software o temporizador.
- [ ] Convertir el cambio estable del switch en un evento FSM:
  - velocidad
  - posicion
- [ ] Definir prioridad entre el switch y el comando `MODE`.
- [ ] Evitar cambios de modo repetidos mientras el switch permanezca estable.
- [ ] Actualizar LCD y CLI cuando el modo cambie.
- [ ] Definir si el cambio de modo detiene siempre el motor. Recomendacion:
      detenerlo y reiniciar los PID antes de aceptar una nueva referencia.

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

- [ ] Actualizar la tabla de GPIO del `README.md`.
- [ ] Actualizar estados/eventos y transiciones de la FSM.
- [ ] Mantener una unica tarea propietaria de UART.
- [ ] Mantener una unica tarea propietaria del display/I2C.
- [ ] Definir la sincronizacion de parametros compartidos entre tareas usando
      eventos, colas o secciones criticas.
- [ ] Verificar que el control continue ejecutandose aunque el LCD, ADC o CLI
      tengan actividad.
- [ ] Probar encoder desconectado, motor trabado, ruido ADC y rebote del
      switch.
- [ ] Ejecutar:

      ```text
      pio run
      ```

## 8. Guardado dinamico en memoria no volatil (NVS)

- [ ] Implementar una funcion de guardado dinamico en memoria no volatil (usando la API NVS de ESP-IDF).
- [ ] Definir los parametros de configuracion a persistir:
  - Ganancias PID de velocidad (`Kp`, `Ki`, `Kd`).
  - Ganancias PID de posicion (`Kp`, `Ki`, `Kd`).
  - `min_move_duty` (PWM minimo de arranque).
  - Cuentas por revolucion / calibracion del encoder.
  - Offset de referencia de posicion cero.
- [ ] Agregar comandos CLI para la gestion de la memoria (por ejemplo `SAVE`, `LOAD`, `NVS RESET`).
- [ ] Implementar lectura y aplicacion automatica de los parametros guardados durante el arranque (`app_main`).
- [ ] Validar integridad (magic number, version de esquema o checksum) para restaurar valores por defecto si la memoria no esta inicializada o esta corrupta.
- [ ] Aislar las operaciones de escritura NVS de la tarea de control en tiempo real (`control_task`) para evitar bloqueos por operaciones de Flash.

