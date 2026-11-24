#pragma once

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

/* --- Configuracion de pines y parametros del motor y encoder --- */
#define IN1_GPIO ((gpio_num_t)PIN_MOTOR_IN1)
#define IN2_GPIO ((gpio_num_t)PIN_MOTOR_IN2)
#define EN_GPIO ((gpio_num_t)PIN_MOTOR_ENA)
#define ENCODER_A_GPIO ((gpio_num_t)PIN_ENC_MOTOR_A)
#define ENCODER_B_GPIO ((gpio_num_t)PIN_ENC_MOTOR_B)
#define ENCODER_CYCLES_PER_CHANNEL 900
#define ENCODER_GLITCH_FILTER_NS 1000U
#define QUADRATURE_FACTOR 4
#define CONTROL_PERIOD_MS 10
#define MAX_RPM 10000.0f
#define MAX_POSITION_COUNTS 100000000.0f
#define MAX_ANGLE_DEGREES 10000000.0f
#define ENCODER_STALL_TIMEOUT_MS 500
#define ENCODER_STARTUP_GRACE_MS 300
#define STALL_MIN_DUTY 185
#define DEFAULT_MIN_MOVE_DUTY 185
#define POSITION_TOLERANCE_COUNTS 8
#define SAFETY_MAX_SEARCH_REVS 1.5f
#define SAFETY_SETTLE_TICKS 5U
#define SAFETY_SPEED_RPM_EMA_ALPHA 0.20f
#define SAFETY_SPEED_TOL_RATIO 0.15f

/* --- Colas, tarea gatekeeper de impresion (printf) y telemetria --- */
#define PRINT_QUEUE_LENGTH 32
#define PRINT_LINE_MAX_LEN 256
#define PRINT_TASK_STACK_SIZE 4096
#define PRINT_TASK_PRIORITY 4
#define PRINT_CLI_SEND_TIMEOUT_MS 100
#define MOTOR_EVENT_QUEUE_LENGTH 32
#define DEFAULT_LOG_PERIOD_MS 500

/* --- Configuracion UART, PWM y PCNT --- */
#define UART_PORT UART_NUM_0
#define UART_RX_TIMEOUT_MS 20
#define UART_BUFFER_SIZE 128
#define COMMAND_BUFFER_SIZE 32
#define PWM_FREQUENCY_HZ 20000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY 255
#define PULSE_PWM_MIN_FREQ_HZ 310U
#define PULSE_PWM_MAX_FREQ_HZ 40000U
#define MAX_PID_GAIN 1000.0f

/* --- Tareas FreeRTOS principales --- */
#define SERIAL_TASK_STACK_SIZE 4096
#define SERIAL_TASK_PRIORITY 5
#define CONTROL_TASK_STACK_SIZE 3072
#define CONTROL_TASK_PRIORITY 6

/* --- Configuracion de pines ADC para los 4 potenciometros (ADC1 input-only) --- */
#define POT_KP_GPIO ((gpio_num_t)PIN_POT_KP)
#define POT_KI_GPIO ((gpio_num_t)PIN_POT_KI)
#define POT_KD_GPIO ((gpio_num_t)PIN_POT_KD)
#define POT_SP_GPIO ((gpio_num_t)PIN_POT_SETPOINT)

/* --- Configuracion del encoder rotativo de interfaz EC11 --- */
#define EC11_CLK_GPIO ((gpio_num_t)PIN_EC11_CLK)
#define EC11_DT_GPIO ((gpio_num_t)PIN_EC11_DT)
#define EC11_SW_GPIO ((gpio_num_t)PIN_EC11_SW)
#define EC11_GLITCH_FILTER_NS 10000U
#define EC11_COUNTS_PER_DETENT 4
#define EC11_DEFAULT_ENABLED 1
#define EC11_SW_DEBOUNCE_TICKS 2U
#define EC11_SW_LONG_PRESS_MS 800U
#define UI_MENU_TIMEOUT_MS 5000U

/* --- Parametros electricos, ADC y filtrado de potenciometros --- */
#define POT_VOLTAGE_MIN_V 0.0f
#define POT_VOLTAGE_MAX_V 3.3f
#define POT_ADC_RAW_MIN 0.0f
#define POT_ADC_RAW_MAX 4095.0f
#define POT_ADC_ATTEN ADC_ATTEN_DB_12
#define POT_ADC_BITWIDTH ADC_BITWIDTH_12
#define POT_TASK_PERIOD_MS 50
#define POT_TASK_STACK_SIZE 3072
#define POT_TASK_PRIORITY 4
#define POT_MULTISAMPLE_COUNT 8
#define POT_EMA_ALPHA 0.25f
#define POT_DEADBAND_NORM 0.01f
#define POT_ZERO_SNAP_NORM 0.015f
#define POT_DEFAULT_ENABLED 1

/* --- Rangos de equivalencia empirica: Modo Velocidad (PID Velocidad + SP en RPM) --- */
#define POT_SPEED_KP_MIN 0.00f
#define POT_SPEED_KP_MAX 1.00f
#define POT_SPEED_KI_MIN 0.00f
#define POT_SPEED_KI_MAX 0.50f
#define POT_SPEED_KD_MIN 0.000f
#define POT_SPEED_KD_MAX 0.050f
#define POT_SPEED_SP_MIN 0.0f
#define POT_SPEED_SP_MAX 10.0f

/* --- Rangos de equivalencia empirica: Modo Posicion (PID Posicion + SP en grados) --- */
#define POT_POS_KP_MIN 0.00f
#define POT_POS_KP_MAX 5.00f
#define POT_POS_KI_MIN 0.00f
#define POT_POS_KI_MAX 5.00f
#define POT_POS_KD_MIN 0.000f
#define POT_POS_KD_MAX 0.500f
#define POT_POS_SP_MIN 0.0f
#define POT_POS_SP_MAX 360.0f

/* --- Rango de equivalencia empirica: Modo Lazo Abierto (SP en Duty PWM 0..255) --- */
#define POT_OPEN_SP_MIN 0.0f
#define POT_OPEN_SP_MAX ((float)PWM_MAX_DUTY)

/* --- Parametros Modo Clock (aguja segundera: 60 divisiones, 1 paso/s) --- */
#define CLOCK_DIVISIONS_PER_REV 60
#define CLOCK_STEP_PERIOD_MS 1000

/* --- Rangos de equivalencia empirica: Modo Clock (PID Clock dedicadas) --- */
#define POT_CLOCK_KP_MIN 0.00f
#define POT_CLOCK_KP_MAX 2.50f
#define POT_CLOCK_KI_MIN 0.00f
#define POT_CLOCK_KI_MAX 1.00f
#define POT_CLOCK_KD_MIN 0.000f
#define POT_CLOCK_KD_MAX 0.500f

/* --- Macro de mapeo lineal entre rango normalizado [0..1] (0..3.3V) y [min..max] --- */
#define POT_MAP_LINEAR(norm, min_val, max_val)                                 \
  ((min_val) + (norm) * ((max_val) - (min_val)))

/* --- Constantes PID por defecto --- */
#define DEFAULT_SPEED_KP 0.1200f
#define DEFAULT_SPEED_KI 0.1000f
#define DEFAULT_SPEED_KD 0.0020f

#define DEFAULT_POS_KP 0.4000f
#define DEFAULT_POS_KI 0.2000f
#define DEFAULT_POS_KD 0.0150f

#define DEFAULT_CLOCK_KP 1.5000f
#define DEFAULT_CLOCK_KI 1.0000f
#define DEFAULT_CLOCK_KD 0.0150f

/* --- Configuracion de almacenamiento no volatil (NVS) y tarea FreeRTOS --- */
#define NVS_NAMESPACE "pidfra_cfg"
#define NVS_BLOB_KEY "sys_cfg"
#define NVS_CONFIG_MAGIC 0x50494446UL /* "PIDF" */
#define NVS_CONFIG_VERSION 4U
#define NVS_TASK_STACK_SIZE 4096
#define NVS_TASK_PRIORITY 3
#define NVS_QUEUE_LENGTH 8
#define NVS_AUTOSAVE_DEBOUNCE_MS 2000
#define NVS_DEFAULT_AUTOSAVE 1

/* --- Configuracion del display LCD 8x2 TC802A-01(R) y tarea FreeRTOS --- */
#define APP_LCD_COLS 8U
#define LCD_TASK_STACK_SIZE 3072
#define LCD_TASK_PRIORITY 3
#define LCD_QUEUE_LENGTH 8
#define LCD_REFRESH_PERIOD_MS 100
#define LCD_BRINGUP_ALL_ON_MS 2000
#define LCD_BRINGUP_PATTERN_HOLD_MS 3000
