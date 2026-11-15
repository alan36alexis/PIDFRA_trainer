#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "motor_pid.h"
#include "nvs.h"
#include "nvs_flash.h"

#define IN1_GPIO GPIO_NUM_22
#define IN2_GPIO GPIO_NUM_23
#define EN_GPIO GPIO_NUM_21
#define ENCODER_A_GPIO GPIO_NUM_18
#define ENCODER_B_GPIO GPIO_NUM_19
#define ENCODER_CYCLES_PER_CHANNEL 900
#define QUADRATURE_FACTOR 4
#define CONTROL_PERIOD_MS 10
#define MAX_RPM 10000.0f
#define MAX_POSITION_COUNTS 100000000.0f
#define MAX_ANGLE_DEGREES 10000000.0f
#define ENCODER_STALL_TIMEOUT_MS 500
#define ENCODER_STARTUP_GRACE_MS 300
#define STALL_MIN_DUTY 170
#define DEFAULT_MIN_MOVE_DUTY 170
#define POSITION_TOLERANCE_COUNTS 8
#define LOG_QUEUE_LENGTH 16
#define MOTOR_EVENT_QUEUE_LENGTH 32
#define DEFAULT_LOG_PERIOD_MS 500

#define UART_PORT UART_NUM_0
#define UART_BUFFER_SIZE 128
#define COMMAND_BUFFER_SIZE 32
#define PWM_FREQUENCY_HZ 20000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY 255
#define PCNT_UNIT_USED PCNT_UNIT_0

/* --- Configuracion de pines ADC para los 4 potenciometros (ADC1 input-only)
 * --- */
#define POT_KP_GPIO GPIO_NUM_36
#define POT_KI_GPIO GPIO_NUM_39
#define POT_KD_GPIO GPIO_NUM_34
#define POT_SP_GPIO GPIO_NUM_35

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
#define POT_DEFAULT_ENABLED 0

/* --- Rangos de equivalencia empirica: Modo Velocidad (PID Velocidad + SP en
 * RPM) --- */
#define POT_SPEED_KP_MIN 0.00f
#define POT_SPEED_KP_MAX 1.00f
#define POT_SPEED_KI_MIN 0.00f
#define POT_SPEED_KI_MAX 0.50f
#define POT_SPEED_KD_MIN 0.000f
#define POT_SPEED_KD_MAX 0.050f
#define POT_SPEED_SP_MIN 0.0f
#define POT_SPEED_SP_MAX 3000.0f

/* --- Rangos de equivalencia empirica: Modo Posicion (PID Posicion + SP en
 * grados) --- */
#define POT_POS_KP_MIN 0.00f
#define POT_POS_KP_MAX 0.50f
#define POT_POS_KI_MIN 0.00f
#define POT_POS_KI_MAX 0.10f
#define POT_POS_KD_MIN 0.000f
#define POT_POS_KD_MAX 0.020f
#define POT_POS_SP_MIN 0.0f
#define POT_POS_SP_MAX 360.0f

/* --- Rango de equivalencia empirica: Modo Lazo Abierto (SP en Duty PWM 0..255)
 * --- */
#define POT_OPEN_SP_MIN 0.0f
#define POT_OPEN_SP_MAX ((float)PWM_MAX_DUTY)

/* --- Parametros Modo Clock (aguja segundera: 60 divisiones, 1 paso/s) --- */
#define CLOCK_DIVISIONS_PER_REV 60
#define CLOCK_STEP_PERIOD_MS 1000

/* --- Rangos de equivalencia empirica: Modo Clock (PID Clock dedicadas) --- */
#define POT_CLOCK_KP_MIN 0.00f
#define POT_CLOCK_KP_MAX 0.50f
#define POT_CLOCK_KI_MIN 0.00f
#define POT_CLOCK_KI_MAX 0.10f
#define POT_CLOCK_KD_MIN 0.000f
#define POT_CLOCK_KD_MAX 0.020f

/* --- Macro de mapeo lineal entre rango normalizado [0..1] (0..3.3V) y
 * [min..max] --- */
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
#define NVS_CONFIG_VERSION 3U
#define NVS_TASK_STACK_SIZE 4096
#define NVS_TASK_PRIORITY 3
#define NVS_QUEUE_LENGTH 8
#define NVS_AUTOSAVE_DEBOUNCE_MS 2000
#define NVS_DEFAULT_AUTOSAVE 1

typedef struct {
  float kp;
  float ki;
  float kd;
} pid_config_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  pid_config_t speed_pid;
  pid_config_t position_pid;
  pid_config_t clock_pid;
  uint8_t min_move_duty;
  uint8_t pot_enabled;
  uint8_t telemetry_enabled;
  uint8_t reserved_flags;
  uint32_t telemetry_period_ms;
  uint32_t encoder_cycles_per_channel;
  int64_t encoder_zero_offset_counts;
  uint32_t crc32;
} nvs_stored_config_t;

typedef enum {
  NVS_OP_SAVE_NOW = 0,
  NVS_OP_LOAD_NOW,
  NVS_OP_RESET_DEFAULTS,
  NVS_OP_SCHEDULE_AUTOSAVE,
} nvs_op_type_t;

typedef struct {
  nvs_op_type_t type;
} nvs_request_t;

static volatile float target_rpm;
static volatile float target_position_counts;
static volatile uint8_t manual_duty = 0;
static volatile uint8_t min_move_duty = DEFAULT_MIN_MOVE_DUTY;
static volatile bool forward = true;
static volatile bool running;
static volatile bool position_mode;
static volatile bool open_loop_mode;
static volatile bool clock_mode = false;
static volatile int64_t clock_base_counts = 0;
static volatile uint32_t clock_step_count = 0;
static volatile uint32_t clock_elapsed_ms = 0;
static volatile bool pulse_active = false;
static volatile bool pulse_direction = true;
static volatile uint8_t pulse_duty = 0;
static volatile int64_t pulse_start_counts_snapshot = 0;
static volatile int64_t encoder_position_counts;
static volatile uint32_t encoder_cycles_per_channel =
    ENCODER_CYCLES_PER_CHANNEL;
static volatile int64_t encoder_zero_offset_counts = 0;
static volatile bool encoder_fault;
static volatile bool telemetry_enabled;
static volatile bool pos_in_target = false;
static volatile uint32_t telemetry_period_ms = DEFAULT_LOG_PERIOD_MS;
static pid_config_t speed_pid_config = {DEFAULT_SPEED_KP, DEFAULT_SPEED_KI,
                                        DEFAULT_SPEED_KD};
static pid_config_t position_pid_config = {DEFAULT_POS_KP, DEFAULT_POS_KI,
                                           DEFAULT_POS_KD};
static pid_config_t clock_pid_config = {DEFAULT_CLOCK_KP, DEFAULT_CLOCK_KI,
                                        DEFAULT_CLOCK_KD};
typedef enum {
  FSM_IDLE,
  FSM_SPEED,
  FSM_POSITION,
  FSM_CLOCK,
  FSM_OPEN_LOOP,
  FSM_FAULT,
} motor_state_t;

static motor_state_t motor_state = FSM_IDLE;

typedef enum {
  EVENT_STOP,
  EVENT_MODE_SPEED,
  EVENT_MODE_POSITION,
  EVENT_MODE_CLOCK,
  EVENT_MODE_OPEN_LOOP,
  EVENT_SPEED_SETPOINT,
  EVENT_POSITION_SETPOINT,
  EVENT_ANGLE_SETPOINT,
  EVENT_DUTY_SETPOINT,
  EVENT_ENCODER_STALL,
  EVENT_CALIBRATE_START,
  EVENT_POT_KP_UPDATE,
  EVENT_POT_KI_UPDATE,
  EVENT_POT_KD_UPDATE,
  EVENT_POT_SP_UPDATE,
} motor_event_type_t;

typedef struct {
  motor_event_type_t type;
  float value;
  bool forward;
} motor_event_t;

typedef enum {
  POT_CH_KP = 0,
  POT_CH_KI,
  POT_CH_KD,
  POT_CH_SP,
  POT_CH_COUNT,
} pot_channel_t;

typedef struct {
  adc_unit_t unit_id;
  adc_channel_t channel;
  adc_oneshot_unit_handle_t unit_handle;
} pot_adc_channel_t;

typedef struct {
  float raw_filtered[POT_CH_COUNT];
  float norm[POT_CH_COUNT];
  float voltage_v[POT_CH_COUNT];
  float last_applied_norm[POT_CH_COUNT];
  float mapped_kp;
  float mapped_ki;
  float mapped_kd;
  float mapped_sp;
  bool initialized;
  bool last_mode_valid;
  bool last_position_mode;
  bool last_open_loop_mode;
  bool last_clock_mode;
} pot_state_t;

static QueueHandle_t motor_event_queue;
static QueueHandle_t nvs_request_queue;
static motor_pid_t speed_pid;
static motor_pid_t position_pid;
static motor_pid_t clock_pid;
static pcnt_unit_handle_t encoder_unit;
static QueueHandle_t log_queue;
static pot_adc_channel_t pot_adc_channels[POT_CH_COUNT];
static pot_state_t pot_state;
static volatile bool pot_enabled = (POT_DEFAULT_ENABLED != 0);
static volatile bool pot_force_sync = false;
static portMUX_TYPE pot_state_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE config_state_lock = portMUX_INITIALIZER_UNLOCKED;
static nvs_stored_config_t nvs_last_saved_config;
static volatile bool nvs_storage_ready = false;
static volatile bool nvs_config_dirty = false;
static volatile bool nvs_autosave_enabled = (NVS_DEFAULT_AUTOSAVE != 0);
static volatile uint32_t nvs_save_count = 0;

typedef enum {
  LOG_EVENT,
  LOG_TELEMETRY,
} log_kind_t;

typedef struct {
  log_kind_t kind;
  char text[128];
  float reference;
  float measured_rpm;
  float measured_angle;
  uint8_t duty;
} log_message_t;

static void set_motor(uint8_t duty, bool motor_forward) {
  gpio_set_level(IN1_GPIO, motor_forward ? 1 : 0);
  gpio_set_level(IN2_GPIO, motor_forward ? 0 : 1);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void stop_motor(void) {
  gpio_set_level(IN1_GPIO, 0);
  gpio_set_level(IN2_GPIO, 0);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void drive_motor_for_pulse(bool motor_forward, uint8_t duty) {
  pulse_active = true;
  pulse_direction = motor_forward;
  pulse_duty = duty;
  set_motor(duty, motor_forward);
}

static void release_motor_for_pulse(void) {
  pulse_active = false;
  pulse_duty = 0;
  stop_motor();
}

static void wait_ms_for_pulse(uint32_t milliseconds) {
  vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

static void set_pwm_frequency_for_pulse(uint32_t pwm_frequency_hz) {
  if (pwm_frequency_hz == 0U) {
    pwm_frequency_hz = PWM_FREQUENCY_HZ;
  }
  ESP_ERROR_CHECK(
      ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, pwm_frequency_hz));
}

static uint32_t pulse_time_ms_now(void) {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static int32_t pulse_encoder_delta_counts(void) {
  int current_counts = 0;
  if (encoder_unit != NULL) {
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_unit, &current_counts));
  }
  return (int32_t)(current_counts - (int)pulse_start_counts_snapshot);
}

static bool pulse_motion_started(void) {
  return pulse_encoder_delta_counts() != 0;
}

static void pulse_diag_cb(const char *tag, uint32_t elapsed_ms,
                          int32_t counts) {
  if (strcmp(tag, "DBG TON_TO_MOTION") == 0) {
    printf("DBG TON_TO_MOTION %" PRIu32 "ms\r\n", elapsed_ms);
  } else if (strcmp(tag, "DBG PULSE_COUNTS") == 0) {
    printf("DBG PULSE_COUNTS %" PRId32 "\r\n", counts);
  }
}

static void latch_encoder_fault(void) {
  encoder_fault = true;
  motor_state = FSM_FAULT;
  running = false;
  motor_pid_reset(&speed_pid);
  motor_pid_reset(&position_pid);
  motor_pid_reset(&clock_pid);
  stop_motor();
  log_message_t message = {.kind = LOG_EVENT};
  snprintf(message.text, sizeof(message.text), "STALL ENC/MOT; LOCK STOP");
  xQueueSend(log_queue, &message, 0);
}

static bool post_motor_event(motor_event_type_t type, float value,
                             bool direction) {
  motor_event_t event = {
      .type = type,
      .value = value,
      .forward = direction,
  };
  return xQueueSend(motor_event_queue, &event, 0) == pdTRUE;
}

static float encoder_counts_per_revolution(void);

static void update_active_pid_gain(motor_event_type_t type, float value) {
  taskENTER_CRITICAL(&config_state_lock);
  pid_config_t *cfg =
      clock_mode ? &clock_pid_config
                 : (position_mode ? &position_pid_config : &speed_pid_config);
  motor_pid_t *pid =
      clock_mode ? &clock_pid : (position_mode ? &position_pid : &speed_pid);
  if (type == EVENT_POT_KP_UPDATE) {
    cfg->kp = value;
    pid->kp = value;
  } else if (type == EVENT_POT_KI_UPDATE) {
    cfg->ki = value;
    pid->ki = value;
  } else if (type == EVENT_POT_KD_UPDATE) {
    cfg->kd = value;
    pid->kd = value;
  }
  taskEXIT_CRITICAL(&config_state_lock);
}

static void handle_motor_event(const motor_event_t *event) {
  bool reset_pids = true;

  switch (event->type) {
  case EVENT_STOP:
    target_rpm = 0.0f;
    manual_duty = 0;
    running = false;
    encoder_fault = false;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    pos_in_target = false;
    motor_state = FSM_IDLE;
    break;
  case EVENT_MODE_SPEED:
    position_mode = false;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    manual_duty = 0;
    running = false;
    motor_state = FSM_IDLE;
    break;
  case EVENT_MODE_POSITION:
    position_mode = true;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    manual_duty = 0;
    running = false;
    motor_state = FSM_IDLE;
    break;
  case EVENT_MODE_CLOCK:
    position_mode = false;
    open_loop_mode = false;
    clock_mode = true;
    manual_duty = 0;
    encoder_fault = false;
    pos_in_target = false;
    clock_base_counts = encoder_position_counts;
    clock_step_count = 0;
    clock_elapsed_ms = 0;
    target_position_counts = (float)clock_base_counts;
    motor_state = FSM_CLOCK;
    running = true;
    break;
  case EVENT_MODE_OPEN_LOOP:
    position_mode = false;
    open_loop_mode = true;
    clock_mode = false;
    clock_elapsed_ms = 0;
    running = false;
    manual_duty = 0;
    motor_state = FSM_IDLE;
    break;
  case EVENT_SPEED_SETPOINT:
    target_rpm = event->value;
    forward = event->forward;
    position_mode = false;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    motor_state = target_rpm > 0.0f ? FSM_SPEED : FSM_IDLE;
    running = target_rpm > 0.0f;
    break;
  case EVENT_POSITION_SETPOINT:
  case EVENT_ANGLE_SETPOINT:
    target_position_counts = event->value;
    pos_in_target = false;
    position_mode = true;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    motor_state = FSM_POSITION;
    running = true;
    break;
  case EVENT_DUTY_SETPOINT:
    target_rpm = 0.0f;
    forward = event->forward;
    position_mode = false;
    open_loop_mode = true;
    clock_mode = false;
    clock_elapsed_ms = 0;
    manual_duty = (uint8_t)fminf(fmaxf(event->value, 0.0f), PWM_MAX_DUTY);
    motor_state = manual_duty > 0 ? FSM_OPEN_LOOP : FSM_IDLE;
    running = manual_duty > 0;
    break;
  case EVENT_ENCODER_STALL:
    latch_encoder_fault();
    return;
  case EVENT_CALIBRATE_START:
    target_rpm = 0.0f;
    manual_duty = 0;
    running = false;
    encoder_fault = false;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    pos_in_target = false;
    motor_state = FSM_IDLE;
    encoder_position_counts = 0;
    if (encoder_unit != NULL) {
      pcnt_unit_clear_count(encoder_unit);
    }
    stop_motor();
    break;
  case EVENT_POT_KP_UPDATE:
  case EVENT_POT_KI_UPDATE:
  case EVENT_POT_KD_UPDATE:
    if (!clock_mode) {
      update_active_pid_gain(event->type, event->value);
    }
    reset_pids = false;
    break;
  case EVENT_POT_SP_UPDATE:
    if (encoder_fault) {
      return;
    }
    if (clock_mode) {
      reset_pids = false;
      break;
    }
    if (open_loop_mode) {
      manual_duty = (uint8_t)fminf(fmaxf(event->value, 0.0f), PWM_MAX_DUTY);
      motor_state = manual_duty > 0 ? FSM_OPEN_LOOP : FSM_IDLE;
      running = manual_duty > 0;
    } else if (position_mode) {
      target_position_counts =
          event->value * encoder_counts_per_revolution() / 360.0f;
      pos_in_target = false;
      motor_state = FSM_POSITION;
      running = true;
    } else {
      target_rpm = event->value;
      motor_state = target_rpm > 0.0f ? FSM_SPEED : FSM_IDLE;
      running = target_rpm > 0.0f;
    }
    reset_pids = (motor_state == FSM_IDLE);
    break;
  }

  if (reset_pids) {
    motor_pid_reset(&speed_pid);
    motor_pid_reset(&position_pid);
    motor_pid_reset(&clock_pid);
  }
  if (motor_state == FSM_IDLE) {
    stop_motor();
  }
}

static void print_help(void) {
  printf(
      "CMD HELP|MODE SPEED/POS/OPEN/CLOCK|CLOCK|SPEED F/R/D <rpm>|POS <cnt>|"
      "ANG <deg>|DUTY F/R/D <0..255>|PULSE F/R <n_cycles> <duty> [freq_hz]|"
      "MINMOVE <0..255>|CAL START/READ/STOP|PID GET|PID SPEED KP/KI/KD <val>|"
      "PID POS KP/KI/KD <val>|PID CLOCK KP/KI/KD <val>|PID DEFAULT|"
      "POT ON/OFF/GET|SAVE|LOAD|NVS SAVE/LOAD/RESET/GET/AUTOSAVE|STOP|STATUS|"
      "LOG ON/OFF|LOG RATE <ms>\r\n");
}

static void apply_speed_pid_config(void) {
  motor_pid_init(&speed_pid, speed_pid_config.kp, speed_pid_config.ki,
                 speed_pid_config.kd, 0.0f, PWM_MAX_DUTY);
}

static void apply_position_pid_config(void) {
  motor_pid_init(&position_pid, position_pid_config.kp, position_pid_config.ki,
                 position_pid_config.kd, -PWM_MAX_DUTY, PWM_MAX_DUTY);
}

static void apply_clock_pid_config(void) {
  motor_pid_init(&clock_pid, clock_pid_config.kp, clock_pid_config.ki,
                 clock_pid_config.kd, -PWM_MAX_DUTY, PWM_MAX_DUTY);
}

static void reset_pid_configs_to_defaults(void) {
  taskENTER_CRITICAL(&config_state_lock);
  speed_pid_config =
      (pid_config_t){DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_SPEED_KD};
  position_pid_config =
      (pid_config_t){DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD};
  clock_pid_config =
      (pid_config_t){DEFAULT_CLOCK_KP, DEFAULT_CLOCK_KI, DEFAULT_CLOCK_KD};
  apply_speed_pid_config();
  apply_position_pid_config();
  apply_clock_pid_config();
  taskEXIT_CRITICAL(&config_state_lock);
}

static float encoder_counts_per_revolution(void) {
  return (float)encoder_cycles_per_channel * QUADRATURE_FACTOR;
}

static void post_log_event(const char *text) {
  if (log_queue == NULL || text == NULL) {
    return;
  }
  log_message_t message = {.kind = LOG_EVENT};
  snprintf(message.text, sizeof(message.text), "%s", text);
  xQueueSend(log_queue, &message, 0);
}

static uint32_t crc32_compute(const void *data, size_t length) {
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; i++) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; bit++) {
      if ((crc & 1U) != 0U) {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

static bool pid_config_is_valid(const pid_config_t *cfg) {
  return cfg != NULL && isfinite(cfg->kp) && cfg->kp >= 0.0f &&
         isfinite(cfg->ki) && cfg->ki >= 0.0f && isfinite(cfg->kd) &&
         cfg->kd >= 0.0f &&
         (cfg->kp > 0.0f || cfg->ki > 0.0f || cfg->kd > 0.0f);
}

static bool pid_configs_equal(const pid_config_t *a, const pid_config_t *b) {
  return a->kp == b->kp && a->ki == b->ki && a->kd == b->kd;
}

static void build_default_nvs_config(nvs_stored_config_t *cfg) {
  memset(cfg, 0, sizeof(*cfg));
  cfg->magic = NVS_CONFIG_MAGIC;
  cfg->version = NVS_CONFIG_VERSION;
  cfg->speed_pid =
      (pid_config_t){DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_SPEED_KD};
  cfg->position_pid =
      (pid_config_t){DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD};
  cfg->clock_pid =
      (pid_config_t){DEFAULT_CLOCK_KP, DEFAULT_CLOCK_KI, DEFAULT_CLOCK_KD};
  cfg->min_move_duty = DEFAULT_MIN_MOVE_DUTY;
  cfg->pot_enabled = (POT_DEFAULT_ENABLED != 0) ? 1U : 0U;
  cfg->telemetry_enabled = 0U;
  cfg->telemetry_period_ms = DEFAULT_LOG_PERIOD_MS;
  cfg->encoder_cycles_per_channel = ENCODER_CYCLES_PER_CHANNEL;
  cfg->encoder_zero_offset_counts = 0;
  cfg->crc32 = crc32_compute(cfg, offsetof(nvs_stored_config_t, crc32));
}

static bool validate_nvs_config(const nvs_stored_config_t *cfg) {
  if (cfg == NULL) {
    return false;
  }
  if (cfg->magic != NVS_CONFIG_MAGIC || cfg->version != NVS_CONFIG_VERSION) {
    return false;
  }
  uint32_t expected_crc =
      crc32_compute(cfg, offsetof(nvs_stored_config_t, crc32));
  if (cfg->crc32 != expected_crc) {
    return false;
  }
  if (!pid_config_is_valid(&cfg->speed_pid) ||
      !pid_config_is_valid(&cfg->position_pid) ||
      !pid_config_is_valid(&cfg->clock_pid)) {
    return false;
  }
  if (cfg->telemetry_period_ms < CONTROL_PERIOD_MS ||
      cfg->encoder_cycles_per_channel == 0U) {
    return false;
  }
  return true;
}

static void capture_current_nvs_config(nvs_stored_config_t *cfg) {
  memset(cfg, 0, sizeof(*cfg));
  cfg->magic = NVS_CONFIG_MAGIC;
  cfg->version = NVS_CONFIG_VERSION;
  taskENTER_CRITICAL(&config_state_lock);
  cfg->speed_pid = speed_pid_config;
  cfg->position_pid = position_pid_config;
  cfg->clock_pid = clock_pid_config;
  cfg->min_move_duty = min_move_duty;
  cfg->pot_enabled = pot_enabled ? 1U : 0U;
  cfg->telemetry_enabled = telemetry_enabled ? 1U : 0U;
  cfg->telemetry_period_ms = telemetry_period_ms;
  cfg->encoder_cycles_per_channel = encoder_cycles_per_channel;
  cfg->encoder_zero_offset_counts = encoder_zero_offset_counts;
  taskEXIT_CRITICAL(&config_state_lock);
  cfg->crc32 = crc32_compute(cfg, offsetof(nvs_stored_config_t, crc32));
}

static void apply_nvs_config_to_runtime(const nvs_stored_config_t *cfg) {
  taskENTER_CRITICAL(&config_state_lock);
  speed_pid_config = cfg->speed_pid;
  position_pid_config = cfg->position_pid;
  clock_pid_config = cfg->clock_pid;
  min_move_duty = cfg->min_move_duty;
  pot_enabled = (cfg->pot_enabled != 0U);
  telemetry_enabled = (cfg->telemetry_enabled != 0U);
  telemetry_period_ms = cfg->telemetry_period_ms;
  encoder_cycles_per_channel = cfg->encoder_cycles_per_channel;
  encoder_zero_offset_counts = cfg->encoder_zero_offset_counts;
  apply_speed_pid_config();
  apply_position_pid_config();
  apply_clock_pid_config();
  taskEXIT_CRITICAL(&config_state_lock);
}

static bool nvs_configs_equal(const nvs_stored_config_t *a,
                              const nvs_stored_config_t *b) {
  if (a == NULL || b == NULL) {
    return false;
  }
  return a->magic == b->magic && a->version == b->version &&
         pid_configs_equal(&a->speed_pid, &b->speed_pid) &&
         pid_configs_equal(&a->position_pid, &b->position_pid) &&
         pid_configs_equal(&a->clock_pid, &b->clock_pid) &&
         a->min_move_duty == b->min_move_duty &&
         a->pot_enabled == b->pot_enabled &&
         a->telemetry_enabled == b->telemetry_enabled &&
         a->telemetry_period_ms == b->telemetry_period_ms &&
         a->encoder_cycles_per_channel == b->encoder_cycles_per_channel &&
         a->encoder_zero_offset_counts == b->encoder_zero_offset_counts;
}

static esp_err_t nvs_write_config(const nvs_stored_config_t *cfg) {
  if (!nvs_storage_ready || cfg == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    return err;
  }
  err = nvs_set_blob(handle, NVS_BLOB_KEY, cfg, sizeof(*cfg));
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  return err;
}

static esp_err_t nvs_read_config(nvs_stored_config_t *cfg) {
  if (!nvs_storage_ready || cfg == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return err;
  }
  size_t required_size = sizeof(*cfg);
  err = nvs_get_blob(handle, NVS_BLOB_KEY, cfg, &required_size);
  nvs_close(handle);
  if (err == ESP_OK && required_size != sizeof(*cfg)) {
    return ESP_ERR_INVALID_SIZE;
  }
  return err;
}

static void nvs_storage_init_and_load(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    nvs_storage_ready = false;
    nvs_stored_config_t defaults;
    build_default_nvs_config(&defaults);
    apply_nvs_config_to_runtime(&defaults);
    post_log_event("NVS INIT ERR; DEFAULTS");
    return;
  }
  nvs_storage_ready = true;

  nvs_stored_config_t loaded;
  if (nvs_read_config(&loaded) == ESP_OK && validate_nvs_config(&loaded)) {
    apply_nvs_config_to_runtime(&loaded);
    taskENTER_CRITICAL(&config_state_lock);
    nvs_last_saved_config = loaded;
    nvs_config_dirty = false;
    taskEXIT_CRITICAL(&config_state_lock);
    post_log_event("NVS LOADED");
  } else {
    nvs_stored_config_t defaults;
    build_default_nvs_config(&defaults);
    apply_nvs_config_to_runtime(&defaults);
    if (nvs_write_config(&defaults) == ESP_OK) {
      taskENTER_CRITICAL(&config_state_lock);
      nvs_last_saved_config = defaults;
      nvs_config_dirty = false;
      nvs_save_count++;
      taskEXIT_CRITICAL(&config_state_lock);
      post_log_event("NVS DEFAULTS SAVED");
    } else {
      post_log_event("NVS DEFAULTS RAM");
    }
  }
}

static bool post_nvs_request(nvs_op_type_t type) {
  if (nvs_request_queue == NULL) {
    return false;
  }
  nvs_request_t req = {.type = type};
  return xQueueSend(nvs_request_queue, &req, 0) == pdTRUE;
}

static void notify_config_modified(void) {
  nvs_config_dirty = true;
  if (nvs_autosave_enabled) {
    post_nvs_request(NVS_OP_SCHEDULE_AUTOSAVE);
  }
}

static char motor_state_code(motor_state_t state) {
  switch (state) {
  case FSM_SPEED:
    return 'V';
  case FSM_POSITION:
    return 'P';
  case FSM_CLOCK:
    return 'C';
  case FSM_OPEN_LOOP:
    return 'O';
  case FSM_FAULT:
    return 'F';
  case FSM_IDLE:
  default:
    return 'I';
  }
}

static void emit_event(const char *text) { printf("%s\r\n", text); }

static void emit_log_message(const log_message_t *message) {
  if (message->kind == LOG_EVENT) {
    printf("EVT %s\r\n", message->text);
  } else if (motor_state == FSM_OPEN_LOOP) {
    printf("TEL O D=%u\r\n", message->duty);
  } else if (motor_state == FSM_CLOCK || clock_mode) {
    printf("TEL C R=%.2f A=%.2f D=%u\r\n", message->reference,
           message->measured_angle, message->duty);
  } else if (position_mode) {
    printf("TEL P R=%.2f A=%.2f D=%u\r\n", message->reference,
           message->measured_angle, message->duty);
  } else {
    printf("TEL V R=%.1f M=%.1f D=%u\r\n", message->reference,
           message->measured_rpm, message->duty);
  }
}

static void process_command(char *command) {
  char *operation = strtok(command, " \t");
  char *value_text = strtok(NULL, "");
  char *end = NULL;

  if (operation == NULL) {
    return;
  }
  for (char *p = operation; *p != '\0'; p++) {
    *p = (char)toupper((unsigned char)*p);
  }
  if (value_text == NULL) {
    value_text = "";
  }
  while (isspace((unsigned char)*value_text)) {
    value_text++;
  }
  if (strcmp(operation, "STOP") == 0 || strcmp(operation, "S") == 0) {
    emit_event(post_motor_event(EVENT_STOP, 0.0f, true) ? "OK STOP"
                                                        : "ERR EVT");
    return;
  }
  if (encoder_fault && strcmp(operation, "HELP") != 0 &&
      strcmp(operation, "?") != 0 && strcmp(operation, "STATUS") != 0 &&
      strcmp(operation, "CAL") != 0 && strcmp(operation, "EUR") != 0 &&
      strcmp(operation, "CALIB") != 0 && strcmp(operation, "SAVE") != 0 &&
      strcmp(operation, "LOAD") != 0 && strcmp(operation, "NVS") != 0) {
    emit_event("ERR LOCK STOP");
    return;
  }
  if (strcmp(operation, "HELP") == 0 || strcmp(operation, "?") == 0) {
    print_help();
    return;
  }
  if (strcmp(operation, "MODE") == 0 || strcmp(operation, "M") == 0) {
    char mode[12] = {0};
    if (sscanf(value_text, "%11s", mode) != 1) {
      emit_event("ERR MODE");
      return;
    }
    for (char *p = mode; *p != '\0'; p++) {
      *p = (char)toupper((unsigned char)*p);
    }
    if (strcmp(mode, "SPEED") != 0 && strcmp(mode, "V") != 0 &&
        strcmp(mode, "POS") != 0 && strcmp(mode, "P") != 0 &&
        strcmp(mode, "OPEN") != 0 && strcmp(mode, "O") != 0 &&
        strcmp(mode, "CLOCK") != 0 && strcmp(mode, "CLK") != 0 &&
        strcmp(mode, "C") != 0) {
      emit_event("ERR MODE");
      return;
    }
    bool position = strcmp(mode, "POS") == 0 || strcmp(mode, "P") == 0;
    bool open_loop = strcmp(mode, "OPEN") == 0 || strcmp(mode, "O") == 0;
    bool clock = strcmp(mode, "CLOCK") == 0 || strcmp(mode, "CLK") == 0 ||
                 strcmp(mode, "C") == 0;
    motor_event_type_t mode_event =
        clock ? EVENT_MODE_CLOCK
              : (position
                     ? EVENT_MODE_POSITION
                     : (open_loop ? EVENT_MODE_OPEN_LOOP : EVENT_MODE_SPEED));
    const char *ok_msg =
        clock ? "OK MODE CLOCK"
              : (position ? "OK MODE POS"
                          : (open_loop ? "OK MODE OPEN" : "OK MODE SPEED"));
    emit_event(post_motor_event(mode_event, 0.0f, true) ? ok_msg : "ERR EVT");
    return;
  }
  if (strcmp(operation, "CLOCK") == 0 || strcmp(operation, "CLK") == 0) {
    if (value_text[0] != '\0') {
      emit_event("ERR MODE");
      return;
    }
    emit_event(post_motor_event(EVENT_MODE_CLOCK, 0.0f, true) ? "OK MODE CLOCK"
                                                              : "ERR EVT");
    return;
  }
  if (strcmp(operation, "POS") == 0 || strcmp(operation, "P") == 0) {
    float position = strtof(value_text, &end);
    while (isspace((unsigned char)*end)) {
      end++;
    }
    if (value_text[0] == '\0' || *end != '\0' || !isfinite(position) ||
        fabsf(position) > MAX_POSITION_COUNTS) {
      emit_event("ERR POS");
      return;
    }
    emit_event(post_motor_event(EVENT_POSITION_SETPOINT, position, true)
                   ? "OK POS"
                   : "ERR EVT");
    return;
  }
  if (strcmp(operation, "ANG") == 0 || strcmp(operation, "A") == 0) {
    float angle = strtof(value_text, &end);
    while (isspace((unsigned char)*end)) {
      end++;
    }
    if (value_text[0] == '\0' || *end != '\0' || !isfinite(angle) ||
        fabsf(angle) > MAX_ANGLE_DEGREES) {
      emit_event("ERR ANG");
      return;
    }
    float target_counts = angle * encoder_counts_per_revolution() / 360.0f;
    emit_event(post_motor_event(EVENT_ANGLE_SETPOINT, target_counts, true)
                   ? "OK ANG"
                   : "ERR EVT");
    return;
  }
  if (strcmp(operation, "STATUS") == 0) {
    printf("STAT S=%c RUN=%d FLT=%d POS=%.2f REF=%.2f\r\n",
           motor_state_code(motor_state), running, encoder_fault,
           (float)encoder_position_counts * 360.0f /
               encoder_counts_per_revolution(),
           motor_state == FSM_OPEN_LOOP
               ? (float)manual_duty
               : ((position_mode || clock_mode)
                      ? target_position_counts * 360.0f /
                            encoder_counts_per_revolution()
                      : target_rpm));
    return;
  }
  if (strcmp(operation, "CAL") == 0 || strcmp(operation, "EUR") == 0 ||
      strcmp(operation, "CALIB") == 0) {
    char subcmd[16] = {0};
    if (sscanf(value_text, "%15s", subcmd) == 1) {
      for (char *p = subcmd; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
    }
    if (strcmp(subcmd, "START") == 0 || strcmp(subcmd, "ZERO") == 0 ||
        strcmp(subcmd, "RESET") == 0) {
      post_motor_event(EVENT_CALIBRATE_START, 0.0f, true);
      emit_event("OK CAL START (MOTOR RELEASED, ENCODER ZEROED)");
      return;
    }
    if (strcmp(subcmd, "READ") == 0 || strcmp(subcmd, "GET") == 0 ||
        strcmp(subcmd, "VAL") == 0 || subcmd[0] == '\0') {
      float cpr = encoder_counts_per_revolution();
      float deg =
          (cpr > 0.0f) ? ((float)encoder_position_counts * 360.0f / cpr) : 0.0f;
      float rev = (cpr > 0.0f) ? ((float)encoder_position_counts / cpr) : 0.0f;
      printf("CAL COUNTS=%" PRId64 " DEG=%.2f REV=%.4f\r\n",
             encoder_position_counts, deg, rev);
      return;
    }
    if (strcmp(subcmd, "STOP") == 0 || strcmp(subcmd, "END") == 0) {
      float cpr = encoder_counts_per_revolution();
      float deg =
          (cpr > 0.0f) ? ((float)encoder_position_counts * 360.0f / cpr) : 0.0f;
      float rev = (cpr > 0.0f) ? ((float)encoder_position_counts / cpr) : 0.0f;
      printf("OK CAL STOP COUNTS=%" PRId64 " DEG=%.2f REV=%.4f\r\n",
             encoder_position_counts, deg, rev);
      return;
    }
    emit_event("ERR CAL (USE: CAL START | CAL READ | CAL STOP)");
    return;
  }
  if (strcmp(operation, "PID") == 0) {
    char selector[12] = {0};
    char param[12] = {0};
    float new_value = 0.0f;
    if (sscanf(value_text, "%11s %11s %f", selector, param, &new_value) == 3) {
      for (char *p = selector; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
      for (char *p = param; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
      bool target_speed =
          strcmp(selector, "SPEED") == 0 || strcmp(selector, "V") == 0;
      bool target_position =
          strcmp(selector, "POS") == 0 || strcmp(selector, "P") == 0;
      bool target_clock = strcmp(selector, "CLOCK") == 0 ||
                          strcmp(selector, "CLK") == 0 ||
                          strcmp(selector, "C") == 0;
      bool set_kp = strcmp(param, "KP") == 0;
      bool set_ki = strcmp(param, "KI") == 0;
      bool set_kd = strcmp(param, "KD") == 0;
      if ((!target_speed && !target_position && !target_clock) ||
          (!set_kp && !set_ki && !set_kd) || !isfinite(new_value) ||
          new_value < 0.0f) {
        emit_event("ERR PID");
        return;
      }
      taskENTER_CRITICAL(&config_state_lock);
      pid_config_t *target_cfg =
          target_speed
              ? &speed_pid_config
              : (target_position ? &position_pid_config : &clock_pid_config);
      if (set_kp) {
        target_cfg->kp = new_value;
      } else if (set_ki) {
        target_cfg->ki = new_value;
      } else {
        target_cfg->kd = new_value;
      }
      if (target_speed) {
        apply_speed_pid_config();
      } else if (target_position) {
        apply_position_pid_config();
      } else {
        apply_clock_pid_config();
      }
      taskEXIT_CRITICAL(&config_state_lock);
      notify_config_modified();
      printf("OK PID %s %s %.4f\r\n", selector, param, new_value);
      return;
    }
    if (sscanf(value_text, "%11s %11s", selector, param) == 2) {
      for (char *p = selector; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
      for (char *p = param; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
      if (strcmp(selector, "GET") == 0) {
        bool speed = strcmp(param, "SPEED") == 0 || strcmp(param, "V") == 0;
        bool position = strcmp(param, "POS") == 0 || strcmp(param, "P") == 0;
        bool clock = strcmp(param, "CLOCK") == 0 || strcmp(param, "CLK") == 0 ||
                     strcmp(param, "C") == 0;
        if (speed || position || clock) {
          pid_config_t speed_cfg;
          pid_config_t pos_cfg;
          pid_config_t clock_cfg;
          taskENTER_CRITICAL(&config_state_lock);
          speed_cfg = speed_pid_config;
          pos_cfg = position_pid_config;
          clock_cfg = clock_pid_config;
          taskEXIT_CRITICAL(&config_state_lock);
          if (speed) {
            printf("PID V KP=%.4f KI=%.4f KD=%.4f\r\n", speed_cfg.kp,
                   speed_cfg.ki, speed_cfg.kd);
          } else if (position) {
            printf("PID P KP=%.4f KI=%.4f KD=%.4f\r\n", pos_cfg.kp, pos_cfg.ki,
                   pos_cfg.kd);
          } else {
            printf("PID C KP=%.4f KI=%.4f KD=%.4f\r\n", clock_cfg.kp,
                   clock_cfg.ki, clock_cfg.kd);
          }
          return;
        }
        emit_event("ERR PID");
        return;
      }
    }
    if (sscanf(value_text, "%11s", selector) == 1) {
      for (char *p = selector; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
      if (strcmp(selector, "DEFAULT") == 0) {
        reset_pid_configs_to_defaults();
        notify_config_modified();
        printf("OK PID DEFAULT\r\n");
        return;
      }
    } else {
      selector[0] = '\0';
    }
    if (strcmp(selector, "GET") == 0 || value_text[0] == '\0') {
      pid_config_t speed_cfg;
      pid_config_t pos_cfg;
      pid_config_t clock_cfg;
      taskENTER_CRITICAL(&config_state_lock);
      speed_cfg = speed_pid_config;
      pos_cfg = position_pid_config;
      clock_cfg = clock_pid_config;
      taskEXIT_CRITICAL(&config_state_lock);
      printf("PID V KP=%.4f KI=%.4f KD=%.4f\r\n", speed_cfg.kp, speed_cfg.ki,
             speed_cfg.kd);
      printf("PID P KP=%.4f KI=%.4f KD=%.4f\r\n", pos_cfg.kp, pos_cfg.ki,
             pos_cfg.kd);
      printf("PID C KP=%.4f KI=%.4f KD=%.4f\r\n", clock_cfg.kp, clock_cfg.ki,
             clock_cfg.kd);
      return;
    }
    emit_event("ERR PID");
    return;
  }
  if (strcmp(operation, "MINMOVE") == 0 ||
      strcmp(operation, "MIN_MOVE_DUTY") == 0) {
    unsigned min_duty = 0;
    if (sscanf(value_text, "%u", &min_duty) != 1 || min_duty > PWM_MAX_DUTY) {
      emit_event("ERR MINMOVE");
      return;
    }
    taskENTER_CRITICAL(&config_state_lock);
    min_move_duty = (uint8_t)min_duty;
    taskEXIT_CRITICAL(&config_state_lock);
    notify_config_modified();
    printf("OK MINMOVE %u\r\n", min_duty);
    return;
  }
  if (strcmp(operation, "PULSE") == 0 || strcmp(operation, "PULSO") == 0) {
    char direction = '\0';
    uint32_t cycles = 0;
    unsigned duty = 0;
    uint32_t pwm_frequency_hz = PWM_FREQUENCY_HZ;
    char *semicolon = strchr(value_text, ';');
    if (semicolon != NULL) {
      *semicolon = '\0';
      char *second = semicolon + 1;
      char *third = strchr(second, ';');
      if (third != NULL) {
        *third = '\0';
        char *fourth = strchr(third + 1, ';');
        unsigned long cycles_tmp = 0UL;
        unsigned long duty_tmp = 0UL;
        unsigned long freq_tmp = 0UL;
        if (fourth != NULL) {
          *fourth = '\0';
          if (sscanf(value_text, " %c", &direction) == 1 &&
              sscanf(second, "%lu", &cycles_tmp) == 1 &&
              sscanf(third + 1, "%lu", &duty_tmp) == 1 &&
              sscanf(fourth + 1, "%lu", &freq_tmp) == 1) {
            cycles = (uint32_t)cycles_tmp;
            duty = (unsigned)duty_tmp;
            pwm_frequency_hz = (uint32_t)freq_tmp;
          } else {
            emit_event("ERR PULSE");
            return;
          }
        } else if (sscanf(value_text, " %c", &direction) == 1 &&
                   sscanf(second, "%lu", &cycles_tmp) == 1 &&
                   sscanf(third + 1, "%lu", &duty_tmp) == 1) {
          cycles = (uint32_t)cycles_tmp;
          duty = (unsigned)duty_tmp;
        } else {
          emit_event("ERR PULSE");
          return;
        }
      } else {
        emit_event("ERR PULSE");
        return;
      }
    } else {
      unsigned long cycles_tmp = 0UL;
      unsigned long duty_tmp = 0UL;
      unsigned long freq_tmp = 0UL;
      if (sscanf(value_text, " %c %lu %lu %lu", &direction, &cycles_tmp,
                 &duty_tmp, &freq_tmp) == 4) {
        cycles = (uint32_t)cycles_tmp;
        duty = (unsigned)duty_tmp;
        pwm_frequency_hz = (uint32_t)freq_tmp;
      } else if (sscanf(value_text, " %c %lu %lu", &direction, &cycles_tmp,
                        &duty_tmp) == 3) {
        cycles = (uint32_t)cycles_tmp;
        duty = (unsigned)duty_tmp;
      } else {
        emit_event("ERR PULSE");
        return;
      }
    }
    direction = (char)toupper((unsigned char)direction);
    if ((direction != 'F' && direction != 'R') || cycles == 0U ||
        duty > PWM_MAX_DUTY || pwm_frequency_hz == 0U ||
        pwm_frequency_hz > 40000U) {
      emit_event("ERR PULSE");
      return;
    }
    bool pulse_forward = direction == 'F';
    uint32_t estimated_pulse_ms = 0U;
    if (pwm_frequency_hz > 0U) {
      uint64_t total_us =
          ((uint64_t)cycles * 1000000ULL) / (uint64_t)pwm_frequency_hz;
      estimated_pulse_ms = (uint32_t)((total_us + 999ULL) / 1000ULL);
      if (estimated_pulse_ms == 0U && cycles > 0U) {
        estimated_pulse_ms = 1U;
      }
    }
    printf("DBG PULSE DIR=%c CYCLES=%" PRIu32 " DUTY=%u FREQ=%" PRIu32
           "Hz EST_MS=%" PRIu32 "\r\n",
           direction, cycles, duty, pwm_frequency_hz, estimated_pulse_ms);
    int raw_count_snapshot = 0;
    if (encoder_unit != NULL) {
      ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_unit, &raw_count_snapshot));
    }
    pulse_start_counts_snapshot = raw_count_snapshot;
    motor_pid_run_cycles(pulse_forward ? MOTOR_PID_DIRECTION_FORWARD
                                       : MOTOR_PID_DIRECTION_REVERSE,
                         cycles, (uint8_t)duty, 0U, pwm_frequency_hz,
                         &drive_motor_for_pulse, &set_pwm_frequency_for_pulse,
                         &release_motor_for_pulse, &wait_ms_for_pulse,
                         &pulse_time_ms_now, &pulse_encoder_delta_counts,
                         &pulse_motion_started, &pulse_diag_cb);
    set_pwm_frequency_for_pulse(PWM_FREQUENCY_HZ);
    forward = pulse_forward;
    emit_event("OK PULSE");
    return;
  }
  if (strcmp(operation, "DUTY") == 0 || strcmp(operation, "PWM") == 0 ||
      strcmp(operation, "OPEN") == 0 || strcmp(operation, "MANUAL") == 0) {
    char direction = '\0';
    if (sscanf(value_text, " %c", &direction) == 1) {
      direction = (char)toupper((unsigned char)direction);
      value_text = strchr(value_text, direction);
      if (value_text != NULL) {
        value_text++;
      }
    } else if (strcmp(operation, "OPEN") == 0 ||
               strcmp(operation, "MANUAL") == 0) {
      direction = forward ? 'F' : 'R';
    }
    if (value_text == NULL) {
      emit_event("ERR DUTY");
      return;
    }
    float duty = strtof(value_text, &end);
    while (isspace((unsigned char)*end)) {
      end++;
    }
    if (value_text[0] == '\0' || *end != '\0' || !isfinite(duty) ||
        duty < 0.0f || duty > (float)PWM_MAX_DUTY) {
      emit_event("ERR DUTY");
      return;
    }
    if (direction == 'F') {
      forward = true;
    } else if (direction == 'R') {
      forward = false;
    } else if (direction == '\0') {
      direction = forward ? 'F' : 'R';
    }
    emit_event(post_motor_event(EVENT_DUTY_SETPOINT, duty, forward)
                   ? "OK DUTY"
                   : "ERR EVT");
    return;
  }
  if (strcmp(operation, "LOG") == 0) {
    char option[12] = {0};
    unsigned period = 0;
    if (sscanf(value_text, "%11s %u", option, &period) < 1) {
      emit_event("ERR LOG");
      return;
    }
    for (char *p = option; *p != '\0'; p++) {
      *p = (char)toupper((unsigned char)*p);
    }
    if (strcmp(option, "ON") == 0) {
      taskENTER_CRITICAL(&config_state_lock);
      telemetry_enabled = true;
      taskEXIT_CRITICAL(&config_state_lock);
      notify_config_modified();
      emit_event("OK LOG ON");
    } else if (strcmp(option, "OFF") == 0) {
      taskENTER_CRITICAL(&config_state_lock);
      telemetry_enabled = false;
      taskEXIT_CRITICAL(&config_state_lock);
      notify_config_modified();
      emit_event("OK LOG OFF");
    } else if (strcmp(option, "RATE") == 0 && period >= CONTROL_PERIOD_MS) {
      taskENTER_CRITICAL(&config_state_lock);
      telemetry_period_ms = period;
      taskEXIT_CRITICAL(&config_state_lock);
      notify_config_modified();
      printf("OK LOG RATE %u\r\n", period);
    } else {
      emit_event("ERR LOG");
    }
    return;
  }
  if (strcmp(operation, "POT") == 0 || strcmp(operation, "ADC") == 0 ||
      strcmp(operation, "POTS") == 0) {
    char option[16] = {0};
    if (sscanf(value_text, "%15s", option) == 1) {
      for (char *p = option; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
    }
    if (strcmp(option, "ON") == 0) {
      taskENTER_CRITICAL(&config_state_lock);
      pot_enabled = true;
      taskEXIT_CRITICAL(&config_state_lock);
      pot_force_sync = true;
      notify_config_modified();
      emit_event("OK POT ON");
      return;
    }
    if (strcmp(option, "OFF") == 0) {
      taskENTER_CRITICAL(&config_state_lock);
      pot_enabled = false;
      taskEXIT_CRITICAL(&config_state_lock);
      notify_config_modified();
      emit_event("OK POT OFF");
      return;
    }
    if (option[0] == '\0' || strcmp(option, "GET") == 0 ||
        strcmp(option, "READ") == 0 || strcmp(option, "STATUS") == 0) {
      pot_state_t snapshot;
      taskENTER_CRITICAL(&pot_state_lock);
      snapshot = pot_state;
      taskEXIT_CRITICAL(&pot_state_lock);
      char mode_ch = open_loop_mode
                         ? 'O'
                         : (clock_mode ? 'C' : (position_mode ? 'P' : 'V'));
      printf("POT EN=%d MODE=%c KP=%.4f(%.2fV) KI=%.4f(%.2fV) "
             "KD=%.4f(%.2fV) SP=%.2f(%.2fV)\r\n",
             pot_enabled ? 1 : 0, mode_ch, snapshot.mapped_kp,
             snapshot.voltage_v[POT_CH_KP], snapshot.mapped_ki,
             snapshot.voltage_v[POT_CH_KI], snapshot.mapped_kd,
             snapshot.voltage_v[POT_CH_KD], snapshot.mapped_sp,
             snapshot.voltage_v[POT_CH_SP]);
      return;
    }
    emit_event("ERR POT");
    return;
  }
  if (strcmp(operation, "SAVE") == 0) {
    emit_event(post_nvs_request(NVS_OP_SAVE_NOW) ? "OK NVS SAVE" : "ERR NVS");
    return;
  }
  if (strcmp(operation, "LOAD") == 0) {
    emit_event(post_nvs_request(NVS_OP_LOAD_NOW) ? "OK NVS LOAD" : "ERR NVS");
    return;
  }
  if (strcmp(operation, "NVS") == 0) {
    char subcmd[16] = {0};
    char option[16] = {0};
    int parsed = sscanf(value_text, "%15s %15s", subcmd, option);
    if (parsed >= 1) {
      for (char *p = subcmd; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
    }
    if (parsed >= 2) {
      for (char *p = option; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
    }
    if (parsed <= 0 || strcmp(subcmd, "GET") == 0 ||
        strcmp(subcmd, "STATUS") == 0) {
      nvs_stored_config_t snapshot;
      capture_current_nvs_config(&snapshot);
      printf("NVS READY=%d DIRTY=%d AUTO=%d SAVES=%" PRIu32
             " V_KP=%.4f V_KI=%.4f V_KD=%.4f P_KP=%.4f P_KI=%.4f P_KD=%.4f"
             " C_KP=%.4f C_KI=%.4f C_KD=%.4f"
             " MINMOVE=%u POT=%d LOG=%d RATE=%" PRIu32 "\r\n",
             nvs_storage_ready ? 1 : 0, nvs_config_dirty ? 1 : 0,
             nvs_autosave_enabled ? 1 : 0, nvs_save_count,
             snapshot.speed_pid.kp, snapshot.speed_pid.ki,
             snapshot.speed_pid.kd, snapshot.position_pid.kp,
             snapshot.position_pid.ki, snapshot.position_pid.kd,
             snapshot.clock_pid.kp, snapshot.clock_pid.ki,
             snapshot.clock_pid.kd, (unsigned)snapshot.min_move_duty,
             (int)snapshot.pot_enabled, (int)snapshot.telemetry_enabled,
             snapshot.telemetry_period_ms);
      return;
    }
    if (strcmp(subcmd, "SAVE") == 0) {
      emit_event(post_nvs_request(NVS_OP_SAVE_NOW) ? "OK NVS SAVE" : "ERR NVS");
      return;
    }
    if (strcmp(subcmd, "LOAD") == 0) {
      emit_event(post_nvs_request(NVS_OP_LOAD_NOW) ? "OK NVS LOAD" : "ERR NVS");
      return;
    }
    if (strcmp(subcmd, "RESET") == 0 || strcmp(subcmd, "DEFAULT") == 0 ||
        strcmp(subcmd, "CLEAR") == 0) {
      emit_event(post_nvs_request(NVS_OP_RESET_DEFAULTS) ? "OK NVS RESET"
                                                         : "ERR NVS");
      return;
    }
    if (strcmp(subcmd, "AUTOSAVE") == 0) {
      if (parsed >= 2 && strcmp(option, "ON") == 0) {
        nvs_autosave_enabled = true;
        if (nvs_config_dirty) {
          post_nvs_request(NVS_OP_SCHEDULE_AUTOSAVE);
        }
        emit_event("OK NVS AUTOSAVE ON");
        return;
      }
      if (parsed >= 2 && strcmp(option, "OFF") == 0) {
        nvs_autosave_enabled = false;
        emit_event("OK NVS AUTOSAVE OFF");
        return;
      }
      emit_event("ERR NVS AUTOSAVE");
      return;
    }
    emit_event("ERR NVS");
    return;
  }
  bool speed_command = strcmp(operation, "SPEED") == 0;
  char direction = '\0';
  if (speed_command) {
    if (sscanf(value_text, " %c", &direction) != 1) {
      emit_event("ERR SPEED");
      return;
    }
    direction = (char)toupper((unsigned char)direction);
    value_text = strchr(value_text, direction);
    if (value_text != NULL) {
      value_text++;
    }
  } else if (strcmp(operation, "F") == 0 || strcmp(operation, "R") == 0 ||
             strcmp(operation, "D") == 0) {
    direction = operation[0];
  } else {
    emit_event("ERR CMD");
    return;
  }
  if (value_text == NULL) {
    emit_event("ERR RPM");
    return;
  }
  float rpm = strtof(value_text, &end);
  while (isspace((unsigned char)*end)) {
    end++;
  }
  if (value_text[0] == '\0' || *end != '\0' || !isfinite(rpm) || rpm < 0.0f ||
      rpm > MAX_RPM) {
    emit_event("ERR RPM");
    return;
  }
  if (direction == 'F') {
    forward = true;
  } else if (direction == 'R') {
    forward = false;
  }
  emit_event(post_motor_event(EVENT_SPEED_SETPOINT, rpm, forward) ? "OK SPEED"
                                                                  : "ERR EVT");
}

static void serial_task(void *arg) {
  uint8_t data[UART_BUFFER_SIZE];
  char command[COMMAND_BUFFER_SIZE];
  size_t command_length = 0;
  (void)arg;
  while (true) {
    log_message_t message;
    while (xQueueReceive(log_queue, &message, 0) == pdTRUE) {
      emit_log_message(&message);
    }
    int length =
        uart_read_bytes(UART_PORT, data, sizeof(data), pdMS_TO_TICKS(20));
    for (int i = 0; i < length; i++) {
      if (data[i] == '\r' || data[i] == '\n') {
        if (command_length > 0) {
          command[command_length] = '\0';
          process_command(command);
          command_length = 0;
        }
      } else if (command_length < sizeof(command) - 1) {
        command[command_length++] = (char)data[i];
      } else {
        command_length = 0;
        emit_event("ERR LONG");
      }
    }
  }
}

static void encoder_init(void) {
  const pcnt_unit_config_t unit_config = {
      .high_limit = 32767,
      .low_limit = -32768,
  };
  const pcnt_chan_config_t channel_a_config = {
      .edge_gpio_num = ENCODER_A_GPIO,
      .level_gpio_num = ENCODER_B_GPIO,
  };
  const pcnt_chan_config_t channel_b_config = {
      .edge_gpio_num = ENCODER_B_GPIO,
      .level_gpio_num = ENCODER_A_GPIO,
  };
  pcnt_channel_handle_t channel_a;
  pcnt_channel_handle_t channel_b;

  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &encoder_unit));
  ESP_ERROR_CHECK(
      pcnt_new_channel(encoder_unit, &channel_a_config, &channel_a));
  ESP_ERROR_CHECK(
      pcnt_new_channel(encoder_unit, &channel_b_config, &channel_b));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));
}

static uint8_t run_position_pid_step(motor_pid_t *pid, float target_counts,
                                     float sample_time, bool log_status,
                                     bool reset_on_target,
                                     bool *movement_expected) {
  float output = motor_pid_update(pid, target_counts,
                                  (float)encoder_position_counts, sample_time);
  float position_error = fabsf(target_counts - (float)encoder_position_counts);
  bool motor_forward = output >= 0.0f;
  uint8_t applied_duty = 0;

  if (position_error > POSITION_TOLERANCE_COUNTS) {
    pos_in_target = false;
    if (log_status) {
      printf("POS OUT=%.2f ERR=%.2f\r\n", output, position_error);
    }
    float abs_output = fabsf(output);
    if (abs_output > 0.0f) {
      abs_output += (float)min_move_duty;
      applied_duty = (uint8_t)fminf(abs_output, PWM_MAX_DUTY);
      *movement_expected = applied_duty >= STALL_MIN_DUTY;
    } else {
      applied_duty = 0;
      *movement_expected = false;
    }
  } else {
    applied_duty = 0;
    *movement_expected = false;
    if (!pos_in_target) {
      pos_in_target = true;
      if (log_status) {
        printf("POS OUT=%.2f ERR=%.2f (SP REACHED)\r\n", output,
               position_error);
      }
      if (reset_on_target) {
        motor_pid_reset(pid);
      }
    }
  }
  set_motor(applied_duty, motor_forward);
  return applied_duty;
}

static void control_task(void *arg) {
  const float sample_time = CONTROL_PERIOD_MS / 1000.0f;
  const TickType_t period_ticks = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t last_wake_time = xTaskGetTickCount();
  uint32_t movement_start_ms = 0;
  uint32_t no_motion_ms = 0;
  uint32_t telemetry_elapsed_ms = 0;
  (void)arg;
  while (true) {
    motor_event_t event;
    while (xQueueReceive(motor_event_queue, &event, 0) == pdTRUE) {
      handle_motor_event(&event);
    }

    // Bypass de proceso PID cuando se usa comando PULSE
    if (pulse_active) {
      set_motor(pulse_duty, pulse_direction);
      vTaskDelayUntil(&last_wake_time, period_ticks);
      // bypassea todo lo de abajo
      continue;
    }

    int counts = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_unit, &counts));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    encoder_position_counts += counts;

    float counts_per_revolution = encoder_counts_per_revolution();
    float measured_rpm =
        ((float)abs(counts) * 60.0f) / (counts_per_revolution * sample_time);
    uint8_t applied_duty = 0;
    bool movement_expected = false;
    if (motor_state == FSM_SPEED || motor_state == FSM_POSITION ||
        motor_state == FSM_CLOCK || motor_state == FSM_OPEN_LOOP) {
      if (motor_state == FSM_POSITION) {
        applied_duty =
            run_position_pid_step(&position_pid, target_position_counts,
                                  sample_time, true, false, &movement_expected);
      } else if (motor_state == FSM_CLOCK) {
        clock_elapsed_ms += CONTROL_PERIOD_MS;
        if (clock_elapsed_ms >= CLOCK_STEP_PERIOD_MS) {
          while (clock_elapsed_ms >= CLOCK_STEP_PERIOD_MS) {
            clock_elapsed_ms -= CLOCK_STEP_PERIOD_MS;
            clock_step_count++;
          }
          target_position_counts = (float)clock_base_counts +
                                   (float)(((double)clock_step_count *
                                            (double)counts_per_revolution) /
                                           (double)CLOCK_DIVISIONS_PER_REV);
          pos_in_target = false;
          clock_pid.previous_error =
              target_position_counts - (float)encoder_position_counts;
        }
        applied_duty =
            run_position_pid_step(&clock_pid, target_position_counts,
                                  sample_time, false, true, &movement_expected);
      } else if (motor_state == FSM_SPEED) {
        float duty =
            motor_pid_update(&speed_pid, target_rpm, measured_rpm, sample_time);
        float abs_duty = fabsf(duty);
        if (fabsf(target_rpm) > 0.0f) {
          if (abs_duty < (float)min_move_duty) {
            duty = copysignf((float)min_move_duty, duty == 0.0f ? 1.0f : duty);
            abs_duty = (float)min_move_duty;
          }
          applied_duty = (uint8_t)fminf(abs_duty, PWM_MAX_DUTY);
          movement_expected = applied_duty >= STALL_MIN_DUTY;
        } else {
          applied_duty = 0;
          movement_expected = false;
        }
        set_motor(applied_duty, forward);
      } else {
        applied_duty = manual_duty;
        set_motor(applied_duty, forward);
        movement_expected = false;
      }
    } else {
      stop_motor();
    }
    telemetry_elapsed_ms += CONTROL_PERIOD_MS;
    if (telemetry_enabled && telemetry_elapsed_ms >= telemetry_period_ms) {
      log_message_t message = {
          .kind = LOG_TELEMETRY,
          .reference =
              (position_mode || clock_mode)
                  ? target_position_counts * 360.0f / counts_per_revolution
                  : target_rpm,
          .measured_rpm = measured_rpm,
          .measured_angle =
              (float)encoder_position_counts * 360.0f / counts_per_revolution,
          .duty = applied_duty,
      };
      if (xQueueSend(log_queue, &message, 0) != pdTRUE) {
        telemetry_elapsed_ms = telemetry_period_ms;
      } else {
        telemetry_elapsed_ms = 0;
      }
    }
    if (movement_expected && motor_state != FSM_FAULT) {
      if (movement_start_ms == 0) {
        movement_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
      }
      uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
      if (abs(counts) > 0) {
        no_motion_ms = 0;
      } else if (now_ms - movement_start_ms >= ENCODER_STARTUP_GRACE_MS) {
        no_motion_ms += CONTROL_PERIOD_MS;
      }
      if (no_motion_ms >= ENCODER_STALL_TIMEOUT_MS) {
        motor_event_t stall_event = {
            .type = EVENT_ENCODER_STALL,
        };
        handle_motor_event(&stall_event);
        movement_start_ms = 0;
        no_motion_ms = 0;
      }
    } else {
      movement_start_ms = 0;
      no_motion_ms = 0;
    }
    vTaskDelayUntil(&last_wake_time, period_ticks);
  }
}

static void potentiometer_adc_init(void) {
  static const gpio_num_t pot_gpios[POT_CH_COUNT] = {
      POT_KP_GPIO,
      POT_KI_GPIO,
      POT_KD_GPIO,
      POT_SP_GPIO,
  };
  adc_oneshot_unit_handle_t unit_handles[2] = {NULL, NULL};
  const adc_oneshot_chan_cfg_t chan_cfg = {
      .atten = POT_ADC_ATTEN,
      .bitwidth = POT_ADC_BITWIDTH,
  };

  for (int i = 0; i < POT_CH_COUNT; i++) {
    adc_unit_t unit_id = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    ESP_ERROR_CHECK(
        adc_oneshot_io_to_channel(pot_gpios[i], &unit_id, &channel));
    if (unit_handles[unit_id] == NULL) {
      const adc_oneshot_unit_init_cfg_t init_cfg = {
          .unit_id = unit_id,
          .ulp_mode = ADC_ULP_MODE_DISABLE,
      };
      ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &unit_handles[unit_id]));
    }
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(unit_handles[unit_id], channel, &chan_cfg));
    pot_adc_channels[i].unit_id = unit_id;
    pot_adc_channels[i].channel = channel;
    pot_adc_channels[i].unit_handle = unit_handles[unit_id];
  }
}

static float normalize_pot_raw(float raw_value) {
  const float span = POT_ADC_RAW_MAX - POT_ADC_RAW_MIN;
  float norm = (span > 0.0f) ? ((raw_value - POT_ADC_RAW_MIN) / span) : 0.0f;
  if (norm < 0.0f) {
    norm = 0.0f;
  } else if (norm > 1.0f) {
    norm = 1.0f;
  }
  if (norm < POT_ZERO_SNAP_NORM) {
    norm = 0.0f;
  } else if (norm > (1.0f - POT_ZERO_SNAP_NORM)) {
    norm = 1.0f;
  }
  return norm;
}

static void potentiometer_task(void *arg) {
  static const motor_event_type_t pot_events[POT_CH_COUNT] = {
      EVENT_POT_KP_UPDATE,
      EVENT_POT_KI_UPDATE,
      EVENT_POT_KD_UPDATE,
      EVENT_POT_SP_UPDATE,
  };
  const TickType_t period_ticks = pdMS_TO_TICKS(POT_TASK_PERIOD_MS);
  (void)arg;

  while (true) {
    float raw_filtered[POT_CH_COUNT];
    float norms[POT_CH_COUNT];
    float voltages[POT_CH_COUNT];
    bool was_initialized = false;

    taskENTER_CRITICAL(&pot_state_lock);
    was_initialized = pot_state.initialized;
    for (int ch = 0; ch < POT_CH_COUNT; ch++) {
      raw_filtered[ch] = pot_state.raw_filtered[ch];
      norms[ch] = pot_state.norm[ch];
      voltages[ch] = pot_state.voltage_v[ch];
    }
    taskEXIT_CRITICAL(&pot_state_lock);

    for (int ch = 0; ch < POT_CH_COUNT; ch++) {
      int32_t sum = 0;
      int valid_samples = 0;
      for (int s = 0; s < POT_MULTISAMPLE_COUNT; s++) {
        int raw = 0;
        if (adc_oneshot_read(pot_adc_channels[ch].unit_handle,
                             pot_adc_channels[ch].channel, &raw) == ESP_OK) {
          sum += raw;
          valid_samples++;
        }
      }
      if (valid_samples > 0) {
        float raw_avg = (float)sum / (float)valid_samples;
        if (!was_initialized) {
          raw_filtered[ch] = raw_avg;
        } else {
          raw_filtered[ch] += POT_EMA_ALPHA * (raw_avg - raw_filtered[ch]);
        }
        norms[ch] = normalize_pot_raw(raw_filtered[ch]);
        voltages[ch] =
            POT_MAP_LINEAR(norms[ch], POT_VOLTAGE_MIN_V, POT_VOLTAGE_MAX_V);
      }
    }

    typedef struct {
      float min_val;
      float max_val;
    } pot_range_t;
    static const pot_range_t speed_pot_ranges[3] = {
        {POT_SPEED_KP_MIN, POT_SPEED_KP_MAX},
        {POT_SPEED_KI_MIN, POT_SPEED_KI_MAX},
        {POT_SPEED_KD_MIN, POT_SPEED_KD_MAX},
    };
    static const pot_range_t pos_pot_ranges[3] = {
        {POT_POS_KP_MIN, POT_POS_KP_MAX},
        {POT_POS_KI_MIN, POT_POS_KI_MAX},
        {POT_POS_KD_MIN, POT_POS_KD_MAX},
    };
    static const pot_range_t clock_pot_ranges[3] = {
        {POT_CLOCK_KP_MIN, POT_CLOCK_KP_MAX},
        {POT_CLOCK_KI_MIN, POT_CLOCK_KI_MAX},
        {POT_CLOCK_KD_MIN, POT_CLOCK_KD_MAX},
    };

    bool cur_pos_mode = position_mode;
    bool cur_open_mode = open_loop_mode;
    bool cur_clock_mode = clock_mode;
    const pot_range_t *pid_ranges =
        cur_clock_mode ? clock_pot_ranges
                       : (cur_pos_mode ? pos_pot_ranges : speed_pot_ranges);
    float mapped_kp =
        POT_MAP_LINEAR(norms[POT_CH_KP], pid_ranges[POT_CH_KP].min_val,
                       pid_ranges[POT_CH_KP].max_val);
    float mapped_ki =
        POT_MAP_LINEAR(norms[POT_CH_KI], pid_ranges[POT_CH_KI].min_val,
                       pid_ranges[POT_CH_KI].max_val);
    float mapped_kd =
        POT_MAP_LINEAR(norms[POT_CH_KD], pid_ranges[POT_CH_KD].min_val,
                       pid_ranges[POT_CH_KD].max_val);
    float mapped_sp =
        cur_clock_mode
            ? (360.0f / (float)CLOCK_DIVISIONS_PER_REV)
            : (cur_open_mode
                   ? POT_MAP_LINEAR(norms[POT_CH_SP], POT_OPEN_SP_MIN,
                                    POT_OPEN_SP_MAX)
                   : (cur_pos_mode
                          ? POT_MAP_LINEAR(norms[POT_CH_SP], POT_POS_SP_MIN,
                                           POT_POS_SP_MAX)
                          : POT_MAP_LINEAR(norms[POT_CH_SP], POT_SPEED_SP_MIN,
                                           POT_SPEED_SP_MAX)));

    taskENTER_CRITICAL(&pot_state_lock);
    for (int ch = 0; ch < POT_CH_COUNT; ch++) {
      pot_state.raw_filtered[ch] = raw_filtered[ch];
      pot_state.norm[ch] = norms[ch];
      pot_state.voltage_v[ch] = voltages[ch];
    }
    pot_state.mapped_kp = mapped_kp;
    pot_state.mapped_ki = mapped_ki;
    pot_state.mapped_kd = mapped_kd;
    pot_state.mapped_sp = mapped_sp;
    pot_state.initialized = true;
    taskEXIT_CRITICAL(&pot_state_lock);

    bool mode_changed = !pot_state.last_mode_valid ||
                        (cur_pos_mode != pot_state.last_position_mode) ||
                        (cur_open_mode != pot_state.last_open_loop_mode) ||
                        (cur_clock_mode != pot_state.last_clock_mode);

    if (!was_initialized || mode_changed) {
      for (int ch = 0; ch < POT_CH_COUNT; ch++) {
        pot_state.last_applied_norm[ch] = norms[ch];
      }
      pot_state.last_mode_valid = true;
      pot_state.last_position_mode = cur_pos_mode;
      pot_state.last_open_loop_mode = cur_open_mode;
      pot_state.last_clock_mode = cur_clock_mode;
      if (mode_changed) {
        pot_force_sync = false;
      }
    }

    if (cur_clock_mode) {
      pot_force_sync = false;
    } else if (pot_enabled && !encoder_fault && !pulse_active) {
      bool force_sync = pot_force_sync;
      const float mapped_vals[POT_CH_COUNT] = {
          mapped_kp,
          mapped_ki,
          mapped_kd,
          mapped_sp,
      };
      bool all_posted = true;
      for (int ch = 0; ch < POT_CH_COUNT; ch++) {
        if (force_sync || fabsf(norms[ch] - pot_state.last_applied_norm[ch]) >=
                              POT_DEADBAND_NORM) {
          if (post_motor_event(pot_events[ch], mapped_vals[ch], forward)) {
            pot_state.last_applied_norm[ch] = norms[ch];
          } else {
            all_posted = false;
          }
        }
      }
      if (all_posted && force_sync) {
        pot_force_sync = false;
      }
    }

    vTaskDelay(period_ticks);
  }
}

static void nvs_task(void *arg) {
  const TickType_t debounce_ticks = pdMS_TO_TICKS(NVS_AUTOSAVE_DEBOUNCE_MS);
  bool pending_autosave = false;
  TickType_t autosave_deadline = 0;
  (void)arg;

  while (true) {
    nvs_request_t req;
    TickType_t wait_ticks = portMAX_DELAY;
    if (pending_autosave) {
      TickType_t now = xTaskGetTickCount();
      int32_t remaining = (int32_t)(autosave_deadline - now);
      wait_ticks = (remaining > 0) ? (TickType_t)remaining : 0;
    }

    if (xQueueReceive(nvs_request_queue, &req, wait_ticks) == pdTRUE) {
      switch (req.type) {
      case NVS_OP_SCHEDULE_AUTOSAVE:
        if (nvs_autosave_enabled) {
          pending_autosave = true;
          autosave_deadline = xTaskGetTickCount() + debounce_ticks;
        }
        break;
      case NVS_OP_SAVE_NOW: {
        pending_autosave = false;
        nvs_stored_config_t cfg;
        capture_current_nvs_config(&cfg);
        if (nvs_write_config(&cfg) == ESP_OK) {
          taskENTER_CRITICAL(&config_state_lock);
          nvs_last_saved_config = cfg;
          nvs_config_dirty = false;
          nvs_save_count++;
          taskEXIT_CRITICAL(&config_state_lock);
          post_log_event("NVS SAVED");
        } else {
          post_log_event("NVS SAVE ERR");
        }
        break;
      }
      case NVS_OP_LOAD_NOW: {
        pending_autosave = false;
        nvs_stored_config_t cfg;
        if (nvs_read_config(&cfg) == ESP_OK && validate_nvs_config(&cfg)) {
          apply_nvs_config_to_runtime(&cfg);
          taskENTER_CRITICAL(&config_state_lock);
          nvs_last_saved_config = cfg;
          nvs_config_dirty = false;
          taskEXIT_CRITICAL(&config_state_lock);
          post_log_event("NVS LOADED");
        } else {
          build_default_nvs_config(&cfg);
          apply_nvs_config_to_runtime(&cfg);
          taskENTER_CRITICAL(&config_state_lock);
          nvs_config_dirty = false;
          taskEXIT_CRITICAL(&config_state_lock);
          post_log_event("NVS LOAD ERR; DEFAULTS");
        }
        break;
      }
      case NVS_OP_RESET_DEFAULTS: {
        pending_autosave = false;
        nvs_stored_config_t cfg;
        build_default_nvs_config(&cfg);
        apply_nvs_config_to_runtime(&cfg);
        if (nvs_write_config(&cfg) == ESP_OK) {
          taskENTER_CRITICAL(&config_state_lock);
          nvs_last_saved_config = cfg;
          nvs_config_dirty = false;
          nvs_save_count++;
          taskEXIT_CRITICAL(&config_state_lock);
          post_log_event("NVS RESET DEFAULT");
        } else {
          post_log_event("NVS RESET ERR");
        }
        break;
      }
      }
    } else if (pending_autosave) {
      pending_autosave = false;
      if (!nvs_autosave_enabled) {
        continue;
      }
      nvs_stored_config_t cfg;
      capture_current_nvs_config(&cfg);
      bool unchanged = false;
      taskENTER_CRITICAL(&config_state_lock);
      unchanged = nvs_configs_equal(&cfg, &nvs_last_saved_config);
      if (unchanged) {
        nvs_config_dirty = false;
      }
      taskEXIT_CRITICAL(&config_state_lock);

      if (!unchanged) {
        if (nvs_write_config(&cfg) == ESP_OK) {
          taskENTER_CRITICAL(&config_state_lock);
          nvs_last_saved_config = cfg;
          nvs_config_dirty = false;
          nvs_save_count++;
          taskEXIT_CRITICAL(&config_state_lock);
          post_log_event("NVS AUTOSAVED");
        } else {
          post_log_event("NVS AUTOSAVE ERR");
        }
      }
    }
  }
}

void app_main(void) {
  const ledc_timer_config_t timer_config = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = PWM_RESOLUTION,
      .freq_hz = PWM_FREQUENCY_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  const ledc_channel_config_t channel_config = {
      .gpio_num = EN_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0,
  };
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  const gpio_config_t motor_config = {
      .pin_bit_mask = (1ULL << IN1_GPIO) | (1ULL << IN2_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  ESP_ERROR_CHECK(gpio_config(&motor_config));
  ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
  ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  encoder_init();
  potentiometer_adc_init();
  stop_motor();
  apply_speed_pid_config();
  apply_position_pid_config();
  apply_clock_pid_config();
  ESP_ERROR_CHECK(
      uart_driver_install(UART_PORT, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
  log_queue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(log_message_t));
  ESP_ERROR_CHECK(log_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  motor_event_queue =
      xQueueCreate(MOTOR_EVENT_QUEUE_LENGTH, sizeof(motor_event_t));
  ESP_ERROR_CHECK(motor_event_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  nvs_request_queue = xQueueCreate(NVS_QUEUE_LENGTH, sizeof(nvs_request_t));
  ESP_ERROR_CHECK(nvs_request_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  nvs_storage_init_and_load();
  xTaskCreate(serial_task, "serial_task", 3072, NULL, 5, NULL);
  xTaskCreate(control_task, "control_task", 3072, NULL, 6, NULL);
  xTaskCreate(potentiometer_task, "pot_task", POT_TASK_STACK_SIZE, NULL,
              POT_TASK_PRIORITY, NULL);
  xTaskCreate(nvs_task, "nvs_task", NVS_TASK_STACK_SIZE, NULL,
              NVS_TASK_PRIORITY, NULL);
  log_message_t startup = {.kind = LOG_EVENT};
  snprintf(startup.text, sizeof(startup.text), "READY A=%d B=%d C=%d Q=%d",
           ENCODER_A_GPIO, ENCODER_B_GPIO, (int)encoder_cycles_per_channel,
           QUADRATURE_FACTOR);
  xQueueSend(log_queue, &startup, portMAX_DELAY);
  log_message_t help = {.kind = LOG_EVENT};
  snprintf(help.text, sizeof(help.text), "HELP");
  xQueueSend(log_queue, &help, portMAX_DELAY);
}
