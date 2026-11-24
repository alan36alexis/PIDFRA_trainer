# Especificación de Pinout ESP32 — PIDFRA_trainer

Documentación de referencia de asignación de pines, restricciones eléctricas y criterios de puesta en marcha para el entrenador de control PID a lazo cerrado basado en **NodeMCU-32S (ESP32-WROOM-32, 38 pines)** —totalmente compatible con placas **ESP32 DevKit de 30 pines**.

* **Repositorio de referencia:** `https://github.com/alan36alexis/PIDFRA_trainer.git`

* **GPIOs asignados:** 18 en total (4 entradas exclusivas en `ADC1` + 14 GPIOs bidireccionales seguros, libres de *strapping* y sin *glitches* de arranque).

* **Estrategia de ruteo físico:** Separación entre la etapa analógica/señales lentas (lado izquierdo de la placa) y las señales rápidas de cuadratura y conmutación PWM (lado derecho) para minimizar el acoplamiento de ruido.

## 1. Mapa de Pines Asignados

### 1.1 Lado Izquierdo (Entradas Analógicas + LCD)

| GPIO | Etiqueta típica | Periférico | Señal | Pin LCD | Recurso ESP32 / Notas | 
| ----- | ----- | ----- | ----- | ----- | ----- | 
| **GPIO36** | `VP` / `SVP` | Trimpot 1 (ej. $K_p$) | Cursor (*Wiper*) | — | `ADC1_CH0` (solo entrada, sin pull-up/down interno) | 
| **GPIO39** | `VN` / `SVN` | Trimpot 2 (ej. $K_i$) | Cursor (*Wiper*) | — | `ADC1_CH3` (solo entrada, sin pull-up/down interno) | 
| **GPIO34** | `IO34` | Trimpot 3 (ej. $K_d$) | Cursor (*Wiper*) | — | `ADC1_CH6` (solo entrada, sin pull-up/down interno) | 
| **GPIO35** | `IO35` | Trimpot 4 (ej. Setpoint) | Cursor (*Wiper*) | — | `ADC1_CH7` (solo entrada, sin pull-up/down interno) | 
| **GPIO32** | `IO32` | LCD TC802A-01(R) | `RS` | **4** | Salida digital | 
| **GPIO33** | `IO33` | LCD TC802A-01(R) | `E` | **6** | Salida digital | 
| **GPIO25** | `IO25` | LCD TC802A-01(R) | `DB4` | **11** | Salida digital | 
| **GPIO26** | `IO26` | LCD TC802A-01(R) | `DB5` | **12** | Salida digital | 
| **GPIO27** | `IO27` | LCD TC802A-01(R) | `DB6` | **13** | Salida digital | 
| **GPIO13** | `IO13` | LCD TC802A-01(R) | `DB7` | **14** | Salida digital (evita saltar sobre `GPIO12`/`GPIO14`) | 

### 1.2 Lado Derecho (Control de Motor + Encoders)

| GPIO | Etiqueta típica | Periférico | Señal | Recurso ESP32 / Notas | 
| ----- | ----- | ----- | ----- | ----- | 
| **GPIO23** | `IO23` | Encoder 900 PPR | Fase `A` | **PCNT Unidad 0** (cuadratura por hardware) | 
| **GPIO22** | `IO22` | Encoder 900 PPR | Fase `B` | **PCNT Unidad 0** (cuadratura por hardware) | 
| **GPIO21** | `IO21` | L298N | `ENA` | **LEDC PWM** (*tri-state* en reset; requiere pull-down externo) | 
| **GPIO19** | `IO19` | L298N | `IN1` | Salida digital (sin *glitch* PWM en arranque) | 
| **GPIO18** | `IO18` | L298N | `IN2` | Salida digital (sin *glitch* PWM en arranque) | 
| **GPIO17** | `IO17` | Encoder EC11 | Fase `A` (`CLK`) | **PCNT Unidad 1** o ISR (`INPUT_PULLUP`) | 
| **GPIO16** | `IO16` | Encoder EC11 | Fase `B` (`DT`) | **PCNT Unidad 1** o ISR (`INPUT_PULLUP`) | 
| **GPIO4** | `IO4` | Encoder EC11 | Pulsador `SW` | Entrada digital (`INPUT_PULLUP`) | 

### 1.3 Pines de Alimentación y Control Fijo del LCD Tinsharp TC802A-01(R)

Corresponde al conector estándar de 16 pines (disposición habitual en doble fila $2 \times 8$ para módulos de $8 \times 2$ caracteres con controlador HD44780 / SPLC780D):

| Pin LCD | Señal | Conexión física | Notas | 
| ----- | ----- | ----- | ----- | 
| **1** | `VSS` | `GND` | Masa común con ESP32 y L298N | 
| **2** | `VDD` | $+5\text{ V}$ | Alimentación lógica del módulo | 
| **3** | `V0` | Cursor de trimpot de $10\text{ k}\Omega$ | Ajuste de contraste (extremos a $+5\text{ V}$ y `GND`) | 
| **5** | `R/W` | `GND` | **Fijo a masa** (modo solo escritura; impide retorno de $5\text{ V}$ hacia los GPIOs) | 
| **7–10** | `DB0–DB3` | `NC` o `GND` | Sin uso en modo 4 bits; poner a `GND` mejora inmunidad a ruido si `R/W` está fijo a masa | 
| **15** | `A` (`LED+`) | $+5\text{ V}$ vía resistencia serie | Verificar si el PCB ya incluye resistencia limitadora ($V_f \approx 4.1\text{ V}$) | 
| **16** | `K` (`LED-`) | `GND` | Cátodo de *backlight* LED | 

> **Advertencia de conector** $2 \times 8$**:** Verificar en la serigrafía del PCB la ubicación del pad cuadrado (Pin 1) y medir continuidad entre Pin 1 y el plano de masa antes de energizar para evitar errores de espejado o variantes con `VSS`/`VDD` invertidos.

## 2. Pines Excluidos / Reservados Deliberadamente

| GPIOs | Motivo de exclusión | 
| ----- | ----- | 
| **GPIO6 – GPIO11** | Bus SPI conectado a la memoria Flash interna. No utilizables. | 
| **GPIO1 (`TX0`), GPIO3 (`RX0`)** | Reservados para programación USB y telemetría serial del lazo PID. | 
| **GPIO0, GPIO2, GPIO5, GPIO12, GPIO15** | Pines de *strapping* de arranque (riesgo de fallo de boot o selección errónea de tensión Flash a $1.8\text{ V}$ en `GPIO12`). | 
| **GPIO14** | Emite una ráfaga PWM breve durante el arranque; queda libre solo como reserva para cargas no críticas (ej. conmutación de *backlight* por transistor). | 

> **Nota sobre módulos ESP32-WROVER:** `GPIO16` y `GPIO17` están libres únicamente en módulos **ESP32-WROOM-32** (sin PSRAM). En módulos con PSRAM externa estos pines quedan ocupados internamente y se requiere migrar el LCD a un expansor I2C (`PCF8574`).

## 3. Límites Eléctricos y Tolerancia a $5\text{ V}$

**Ningún GPIO del ESP32 es oficialmente tolerante a** $5\text{ V}$**.** De acuerdo con el *ESP32 Series Datasheet* y el *ESP-FAQ*:

| Parámetro | Especificación oficial | Valor con $V_{DD} = 3.3\text{ V}$ | 
| ----- | ----- | ----- | 
| Tensión de alimentación ($V_{DD}$) | $3.0\text{ V} \text{ a } 3.6\text{ V}$ | $3.3\text{ V}$ nominal | 
| Tensión máxima en pin I/O ($V_{IH(\max)}$) | $V_{DD} + 0.3\text{ V}$ | $3.6\text{ V}$ **máximo absoluto** | 
| Umbral mínimo de nivel alto ($V_{IH(\min)}$) | $0.75 \times V_{DD}$ | $2.475\text{ V}$ | 
| Tensión mínima en pin I/O ($V_{IL(\min)}$) | $-0.3\text{ V}$ | $-0.3\text{ V}$ | 

### Política de diseño del proyecto

* **Cero señales externas por encima de** $3.3\text{ V}$ **sin adaptación o aislamiento.**

* **Prevención de *back-powering*:** Evitar aplicar tensión a pines del ESP32 cuando el módulo está apagado o con el USB desconectado, ya que la corriente se inyecta al riel de $3.3\text{ V}$ a través de las protecciones ESD y puede generar fallos de arranque (*brownout*).

## 4. Consideraciones Críticas por Periférico

### 4.1 Cuatro Trimpots (`GPIO36, 39, 34, 35`)

* Ubicados exclusivamente en **ADC1** para mantener compatibilidad total si se habilita Wi-Fi a futuro (`ADC2` queda inhabilitado con radio activa).

* Alimentar los extremos de los potenciómetros estrictamente con $3.3\text{ V}$ (nunca $5\text{ V}$).

* Colocar un capacitor cerámico de $100\text{ nF}$ desde cada cursor a `GND` para reducir impedancia de fuente del SAR ADC y filtrar ruido de conmutación del L298N.

* Configurar atenuación de $11\text{ dB}$ / $12\text{ dB}$ (`ADC_ATTEN_DB_12`) con calibración de fábrica (*eFuse*), aplicar filtrado digital (media móvil o EMA) y banda muerta (*deadband*) para evitar *derivative kick* en el término $D$ por fluctuaciones de $\pm 2\text{–}3\text{ LSB}$.

### 4.2 Encoder Incremental 900 PPR (`GPIO23, 22`)

* En decodificación por cuadratura X4 produce $3600\text{ cuentas/vuelta}$.

* Utilizar obligatoriamente el periférico **PCNT (Pulse Counter)** por hardware del ESP32 para evitar pérdida de pulsos y *jitter* en el período de muestreo $T_s$ del PID.

* Ajustar el filtro de *glitches* de PCNT según la frecuencia máxima de pulsos esperada para no atenuar pulsos válidos a altas RPM.

* **Adaptación de etapa de salida:**

  * **Si es *open-collector* (NPN):** Colocar resistencias pull-up externas de $2.2\text{ k}\Omega$ a $4.7\text{ k}\Omega$ conectadas a $3.3\text{ V}$ (no depender del pull-up interno de $\sim 45\text{ k}\Omega$).

  * **Si es *push-pull* / *line driver* de** $5\text{ V}$**:** Adaptar nivel antes de los GPIOs mediante:

    * *Opción recomendada:* Buffer alimentado a $3.3\text{ V}$ con entradas tolerantes a $5\text{ V}$, *Schmitt trigger* y protección $I_{\text{off}}$ (ej. `74LVC2G17` o `74LVC245`).

    * *Opción económica (cable corto):* Divisor resistivo con $R_1 = 1.8\text{ k}\Omega$ en serie y $R_2 = 3.3\text{ k}\Omega$ a `GND`:
      

      $$
      V_{\text{GPIO}} = 5.0\text{ V} \times \frac{3.3\text{ k}\Omega}{1.8\text{ k}\Omega + 3.3\text{ k}\Omega} \approx 3.24\text{ V}
      $$

    * **No utilizar** conversores bidireccionales lentos basados en MOSFET para I2C, ya que degradan los flancos rápidos.

### 4.3 Driver de Motor L298N (`GPIO21, 19, 18`)

* Retirar el *jumper* físico de `ENA` del módulo antes de conectar `GPIO21`.

* Instalar una resistencia **pull-down externa de** $10\text{ k}\Omega$ entre `ENA` (`GPIO21`) y `GND` para garantizar que el motor permanezca deshabilitado mientras el ESP32 está en reset o flasheo (*tri-state*).

* Los niveles lógicos de $3.3\text{ V}$ son directamente compatibles con el L298N ($V_{IH(\min)} = 2.3\text{ V}$).

* Configurar **LEDC PWM** con ciclo de trabajo inicial del $0\%$ antes de habilitar salidas y considerar que la etapa bipolar del L298N introduce una caída de tensión interna de $\sim 2\text{–}3\text{ V}$ y una frecuencia práctica recomendada por debajo de $20\text{ kHz}$.

### 4.4 LCD Tinsharp TC802A-01(R) (`GPIO32, 33, 25, 26, 27, 13`)

* Conectar `R/W` (Pin 5) directamente a `GND`.

* **Margen de** $V_{IH}$**:** Un controlador SPLC780D alimentado a $5.0\text{ V}$ puede especificar hasta $V_{IH(\min)} = 0.7 \times V_{DD} = 3.5\text{ V}$. Aunque suele funcionar con $3.3\text{ V}$ cuando el riel USB cae a $\sim 4.6\text{–}4.7\text{ V}$, para inmunidad formal frente a caracteres corruptos se recomienda intercalar un buffer TTL unidireccional (`74HCT245` / `74AHCT125` a $5\text{ V}$) o reducir $V_{DD}$ del LCD a $\sim 4.5\text{ V}$ con un diodo en serie.

* **Backlight:** No aplicar $5\text{ V}$ directos sin confirmar existencia de resistencia limitadora en el PCB ($V_f \approx 4.1\text{ V}$).

### 4.5 Encoder Rotativo EC11 (`GPIO17, 16, 4`)

* Configurar los tres pines con `INPUT_PULLUP` (o pull-ups externos de $10\text{ k}\Omega$ a $3.3\text{ V}$ con red RC de $10\text{ k}\Omega$ / $100\text{ nF}$ para antirrebote por hardware).

* Conectar el pin común del encoder y un extremo del pulsador `SW` directamente a `GND`.

## 5. Plantilla de Abstracción para Firmware (`board_pins.h`)

Centralizar las definiciones en un único archivo de cabecera evita inconsistencias entre módulos de firmware:

```
#pragma once

#include <stdint.h>

// --- Entradas Analógicas (ADC1) ---
static constexpr int PIN_POT_KP       = 36; // ADC1_CH0 (Input only)
static constexpr int PIN_POT_KI       = 39; // ADC1_CH3 (Input only)
static constexpr int PIN_POT_KD       = 34; // ADC1_CH6 (Input only)
static constexpr int PIN_POT_SETPOINT = 35; // ADC1_CH7 (Input only)

// --- LCD Tinsharp TC802A-01(R) (Modo 4 bits, R/W fijo a GND) ---
static constexpr int PIN_LCD_RS       = 32; // Pin 4 LCD
static constexpr int PIN_LCD_E        = 33; // Pin 6 LCD
static constexpr int PIN_LCD_D4       = 25; // Pin 11 LCD
static constexpr int PIN_LCD_D5       = 26; // Pin 12 LCD
static constexpr int PIN_LCD_D6       = 27; // Pin 13 LCD
static constexpr int PIN_LCD_D7       = 13; // Pin 14 LCD

// --- Encoder Incremental Motor (900 PPR -> 3600 cuentas/vuelta en X4) ---
static constexpr int PIN_ENC_MOTOR_A  = 23; // PCNT Unit 0
static constexpr int PIN_ENC_MOTOR_B  = 22; // PCNT Unit 0

// --- Driver de Motor L298N ---
static constexpr int PIN_MOTOR_ENA    = 21; // LEDC PWM (Pull-down externo 10 kOhm)
static constexpr int PIN_MOTOR_IN1    = 19;
static constexpr int PIN_MOTOR_IN2    = 18;

// --- Encoder de Interfaz EC11 ---
static constexpr int PIN_EC11_CLK     = 17; // PCNT Unit 1 / ISR (INPUT_PULLUP)
static constexpr int PIN_EC11_DT      = 16; // PCNT Unit 1 / ISR (INPUT_PULLUP)
static constexpr int PIN_EC11_SW      = 4;  // INPUT_PULLUP

```

## 6. Lista de Verificación de Puesta en Marcha (*Bring-up*)

Ejecutar estas comprobaciones antes de cerrar el lazo de control PID:

* \[ \] **LCD aislado:** Energizar únicamente los pines 1, 2, 3, 15 y 16 del LCD (sin conectar los GPIOs del ESP32) y ajustar el trimpot de `V0` hasta visualizar la primera fila de bloques oscuros de *power-on reset*.

* \[ \] **Verificación del encoder 900 PPR sin carga:** Medir con multímetro u osciloscopio las fases `A` y `B` desconectadas del ESP32 al girar el eje para confirmar si la salida es *open-collector* ($0\text{ V}$ fijo sin pull-up) o *push-pull* ($0\text{ V}\text{–}5\text{ V}$). Confirmar que en el pin del ESP32 nunca se superen los $3.6\text{ V}$ (incluyendo *ringing*).

* \[ \] **Arranque seguro del puente H:** Capturar `ENA` (`GPIO21`) con osciloscopio durante el encendido (*power-on*), reset y flasheo para verificar $0\text{ V}$ constantes sin pulsos espurios.

* \[ \] **Integridad de cuentas PCNT:** Girar el eje exactamente $N$ vueltas mecánicas completas a baja y máxima velocidad verificando que el contador reporte $3600 \times N \pm 1$ cuenta.

* \[ \] **Ruido en ADC1 bajo carga:** Medir la desviación estándar de las 4 entradas analógicas con el motor excitado al $50\%$ de ciclo de trabajo PWM y validar que el filtrado evita oscilaciones en $K_p, K_i, K_d$ y *Setpoint*.

## 7. Decisión de Arquitectura (ADR) — Margen de I/O

* **Estado:** Aceptada para la revisión actual de hardware.

* **Contexto:** El modo paralelo de 4 bits del LCD consume 6 GPIOs, agotando el $100\%$ de los pines bidireccionales seguros ($14/14$).

* **Disparador de cambio:** La incorporación de cualquier periférico adicional (sensado de corriente `INA219`/`ACS712`, fines de carrera, botón físico de parada de emergencia o bus de comunicaciones) **dispara obligatoriamente la migración del LCD a un expansor I2C (`PCF8574`)**, liberando 4 GPIOs limpios sin recurrir a pines de *strapping*. En ese escenario, desacoplar el refresco de pantalla en una tarea del Core 0 y mantener el lazo PID determinista en el Core 1.