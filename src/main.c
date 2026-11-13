#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
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

/* --- Configuracion de pines ADC para los 4 potenciometros (ADC1 input-only) --- */
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
#define POT_DEFAULT_ENABLED 1

/* --- Rangos de equivalencia empirica: Modo Velocidad (PID Velocidad + SP en RPM) --- */
#define POT_SPEED_KP_MIN 0.00f
#define POT_SPEED_KP_MAX 1.00f
#define POT_SPEED_KI_MIN 0.00f
#define POT_SPEED_KI_MAX 0.50f
#define POT_SPEED_KD_MIN 0.000f
#define POT_SPEED_KD_MAX 0.050f
#define POT_SPEED_SP_MIN 0.0f
#define POT_SPEED_SP_MAX 3000.0f

/* --- Rangos de equivalencia empirica: Modo Posicion (PID Posicion + SP en grados) --- */
#define POT_POS_KP_MIN 0.00f
#define POT_POS_KP_MAX 0.50f
#define POT_POS_KI_MIN 0.00f
#define POT_POS_KI_MAX 0.10f
#define POT_POS_KD_MIN 0.000f
#define POT_POS_KD_MAX 0.020f
#define POT_POS_SP_MIN 0.0f
#define POT_POS_SP_MAX 360.0f

/* --- Rango de equivalencia empirica: Modo Lazo Abierto (SP en Duty PWM 0..255) --- */
#define POT_OPEN_SP_MIN 0.0f
#define POT_OPEN_SP_MAX ((float)PWM_MAX_DUTY)

/* --- Macro de mapeo lineal entre rango normalizado [0..1] (0..3.3V) y [min..max] --- */
#define POT_MAP_LINEAR(norm, min_val, max_val) \
  ((min_val) + (norm) * ((max_val) - (min_val)))

typedef struct {
  float kp;
  float ki;
  float kd;
} pid_config_t;

static volatile float target_rpm;
static volatile float target_position_counts;
static volatile uint8_t manual_duty = 0;
static volatile uint8_t min_move_duty = DEFAULT_MIN_MOVE_DUTY;
static volatile bool forward = true;
static volatile bool running;
static volatile bool position_mode;
static volatile bool open_loop_mode;
static volatile bool pulse_active = false;
static volatile bool pulse_direction = true;
static volatile uint8_t pulse_duty = 0;
static volatile int64_t pulse_start_counts_snapshot = 0;
static volatile int64_t encoder_position_counts;
static volatile bool encoder_fault;
static volatile bool telemetry_enabled;
static volatile bool pos_in_target = false;
static volatile uint32_t telemetry_period_ms = DEFAULT_LOG_PERIOD_MS;
static pid_config_t speed_pid_config = {0.12f, 0.10f, 0.002f};
static pid_config_t position_pid_config = {0.05f, 0.01f, 0.00f};
typedef enum {
  FSM_IDLE,
  FSM_SPEED,
  FSM_POSITION,
  FSM_OPEN_LOOP,
  FSM_FAULT,
} motor_state_t;

static motor_state_t motor_state = FSM_IDLE;

typedef enum {
  EVENT_STOP,
  EVENT_MODE_SPEED,
  EVENT_MODE_POSITION,
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
} pot_state_t;

static QueueHandle_t motor_event_queue;
static motor_pid_t speed_pid;
static motor_pid_t position_pid;
static pcnt_unit_handle_t encoder_unit;
static QueueHandle_t log_queue;
static pot_adc_channel_t pot_adc_channels[POT_CH_COUNT];
static pot_state_t pot_state;
static volatile bool pot_enabled = (POT_DEFAULT_ENABLED != 0);
static volatile bool pot_force_sync = true;
static portMUX_TYPE pot_state_lock = portMUX_INITIALIZER_UNLOCKED;

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

static void handle_motor_event(const motor_event_t *event) {
  bool reset_pids = true;

  switch (event->type) {
  case EVENT_STOP:
    target_rpm = 0.0f;
    manual_duty = 0;
    running = false;
    encoder_fault = false;
    open_loop_mode = false;
    pos_in_target = false;
    motor_state = FSM_IDLE;
    break;
  case EVENT_MODE_SPEED:
    position_mode = false;
    open_loop_mode = false;
    manual_duty = 0;
    running = false;
    motor_state = FSM_IDLE;
    pot_force_sync = true;
    break;
  case EVENT_MODE_POSITION:
    position_mode = true;
    open_loop_mode = false;
    manual_duty = 0;
    running = false;
    motor_state = FSM_IDLE;
    pot_force_sync = true;
    break;
  case EVENT_MODE_OPEN_LOOP:
    position_mode = false;
    open_loop_mode = true;
    running = false;
    manual_duty = 0;
    motor_state = FSM_IDLE;
    pot_force_sync = true;
    break;
  case EVENT_SPEED_SETPOINT:
    target_rpm = event->value;
    forward = event->forward;
    position_mode = false;
    open_loop_mode = false;
    motor_state = target_rpm > 0.0f ? FSM_SPEED : FSM_IDLE;
    running = target_rpm > 0.0f;
    break;
  case EVENT_POSITION_SETPOINT:
  case EVENT_ANGLE_SETPOINT:
    target_position_counts = event->value;
    pos_in_target = false;
    position_mode = true;
    open_loop_mode = false;
    motor_state = FSM_POSITION;
    running = true;
    break;
  case EVENT_DUTY_SETPOINT:
    target_rpm = 0.0f;
    forward = event->forward;
    position_mode = false;
    open_loop_mode = true;
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
    pos_in_target = false;
    motor_state = FSM_IDLE;
    encoder_position_counts = 0;
    if (encoder_unit != NULL) {
      pcnt_unit_clear_count(encoder_unit);
    }
    stop_motor();
    break;
  case EVENT_POT_KP_UPDATE:
    if (position_mode) {
      position_pid_config.kp = event->value;
      position_pid.kp = event->value;
    } else {
      speed_pid_config.kp = event->value;
      speed_pid.kp = event->value;
    }
    reset_pids = false;
    break;
  case EVENT_POT_KI_UPDATE:
    if (position_mode) {
      position_pid_config.ki = event->value;
      position_pid.ki = event->value;
    } else {
      speed_pid_config.ki = event->value;
      speed_pid.ki = event->value;
    }
    reset_pids = false;
    break;
  case EVENT_POT_KD_UPDATE:
    if (position_mode) {
      position_pid_config.kd = event->value;
      position_pid.kd = event->value;
    } else {
      speed_pid_config.kd = event->value;
      speed_pid.kd = event->value;
    }
    reset_pids = false;
    break;
  case EVENT_POT_SP_UPDATE:
    if (encoder_fault) {
      return;
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
  }
  if (motor_state == FSM_IDLE) {
    stop_motor();
  }
}

static void print_help(void) {
  printf(
      "CMD HELP|MODE SPEED/POS/OPEN|SPEED F/R/D <rpm>|POS <cnt>|"
      "ANG <deg>|DUTY F/R/D <0..255>|PULSE F/R <n_cycles> <duty> [freq_hz]|"
      "MINMOVE <0..255>|CAL START/READ/STOP|PID GET|PID SPEED KP/KI/KD <val>|"
      "PID POS KP/KI/KD <val>|PID DEFAULT|POT ON/OFF/GET|STOP|STATUS|"
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

static void reset_pid_configs_to_defaults(void) {
  speed_pid_config = (pid_config_t){0.12f, 0.10f, 0.002f};
  position_pid_config = (pid_config_t){0.05f, 0.0f, 0.001f};
  apply_speed_pid_config();
  apply_position_pid_config();
}

static float encoder_counts_per_revolution(void) {
  return (float)ENCODER_CYCLES_PER_CHANNEL * QUADRATURE_FACTOR;
}

static char motor_state_code(motor_state_t state) {
  switch (state) {
  case FSM_SPEED:
    return 'V';
  case FSM_POSITION:
    return 'P';
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
      strcmp(operation, "CALIB") != 0) {
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
        strcmp(mode, "OPEN") != 0 && strcmp(mode, "O") != 0) {
      emit_event("ERR MODE");
      return;
    }
    bool position = strcmp(mode, "POS") == 0 || strcmp(mode, "P") == 0;
    bool open_loop = strcmp(mode, "OPEN") == 0 || strcmp(mode, "O") == 0;
    bool posted = post_motor_event(
        position ? EVENT_MODE_POSITION
                 : (open_loop ? EVENT_MODE_OPEN_LOOP : EVENT_MODE_SPEED),
        0.0f, true);
    emit_event(posted
                   ? (position ? "OK MODE POS"
                               : (open_loop ? "OK MODE OPEN" : "OK MODE SPEED"))
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
               : (position_mode ? target_position_counts * 360.0f /
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
      bool set_kp = strcmp(param, "KP") == 0;
      bool set_ki = strcmp(param, "KI") == 0;
      bool set_kd = strcmp(param, "KD") == 0;
      if ((!target_speed && !target_position) ||
          (!set_kp && !set_ki && !set_kd) || !isfinite(new_value)) {
        emit_event("ERR PID");
        return;
      }
      if (target_speed) {
        if (set_kp) {
          speed_pid_config.kp = new_value;
        } else if (set_ki) {
          speed_pid_config.ki = new_value;
        } else {
          speed_pid_config.kd = new_value;
        }
        apply_speed_pid_config();
      } else {
        if (set_kp) {
          position_pid_config.kp = new_value;
        } else if (set_ki) {
          position_pid_config.ki = new_value;
        } else {
          position_pid_config.kd = new_value;
        }
        apply_position_pid_config();
      }
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
        if (speed || position) {
          if (speed) {
            printf("PID V KP=%.4f KI=%.4f KD=%.4f\r\n", speed_pid_config.kp,
                   speed_pid_config.ki, speed_pid_config.kd);
          } else {
            printf("PID P KP=%.4f KI=%.4f KD=%.4f\r\n", position_pid_config.kp,
                   position_pid_config.ki, position_pid_config.kd);
          }
          return;
        }
      }
    }
    if (sscanf(value_text, "%11s", selector) == 1) {
      for (char *p = selector; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
      if (strcmp(selector, "DEFAULT") == 0) {
        reset_pid_configs_to_defaults();
        printf("OK PID DEFAULT\r\n");
        return;
      }
    }
    if (strcmp(value_text, "GET") == 0 || strcmp(value_text, "") == 0) {
      printf("PID V KP=%.4f KI=%.4f KD=%.4f\r\n", speed_pid_config.kp,
             speed_pid_config.ki, speed_pid_config.kd);
      printf("PID P KP=%.4f KI=%.4f KD=%.4f\r\n", position_pid_config.kp,
             position_pid_config.ki, position_pid_config.kd);
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
    min_move_duty = (uint8_t)min_duty;
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
      telemetry_enabled = true;
      emit_event("OK LOG ON");
    } else if (strcmp(option, "OFF") == 0) {
      telemetry_enabled = false;
      emit_event("OK LOG OFF");
    } else if (strcmp(option, "RATE") == 0 && period >= CONTROL_PERIOD_MS) {
      telemetry_period_ms = period;
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
      pot_enabled = true;
      pot_force_sync = true;
      emit_event("OK POT ON");
      return;
    }
    if (strcmp(option, "OFF") == 0) {
      pot_enabled = false;
      emit_event("OK POT OFF");
      return;
    }
    if (option[0] == '\0' || strcmp(option, "GET") == 0 ||
        strcmp(option, "READ") == 0 || strcmp(option, "STATUS") == 0) {
      pot_state_t snapshot;
      taskENTER_CRITICAL(&pot_state_lock);
      snapshot = pot_state;
      taskEXIT_CRITICAL(&pot_state_lock);
      char mode_ch = open_loop_mode ? 'O' : (position_mode ? 'P' : 'V');
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
        motor_state == FSM_OPEN_LOOP) {
      if (motor_state == FSM_POSITION) {
        float output =
            motor_pid_update(&position_pid, target_position_counts,
                             (float)encoder_position_counts, sample_time);
        float position_error =
            fabsf(target_position_counts - (float)encoder_position_counts);
        bool motor_forward = output >= 0.0f;
        if (position_error > POSITION_TOLERANCE_COUNTS) {
          pos_in_target = false;
          printf("POS OUT=%.2f ERR=%.2f\r\n", output, position_error);
          float abs_output = fabsf(output);
          if (abs_output < (float)min_move_duty) {
            abs_output = (float)min_move_duty;
          }
          applied_duty = (uint8_t)fminf(abs_output, PWM_MAX_DUTY);
          movement_expected = applied_duty >= STALL_MIN_DUTY;
        } else {
          applied_duty = 0;
          movement_expected = false;
          if (!pos_in_target) {
            pos_in_target = true;
            printf("POS OUT=%.2f ERR=%.2f (SP REACHED)\r\n", output,
                   position_error);
          }
        }
        set_motor(applied_duty, motor_forward);
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
          .reference = position_mode ? target_position_counts * 360.0f /
                                           counts_per_revolution
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
      ESP_ERROR_CHECK(
          adc_oneshot_new_unit(&init_cfg, &unit_handles[unit_id]));
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

    bool cur_pos_mode = position_mode;
    bool cur_open_mode = open_loop_mode;
    float mapped_kp = cur_pos_mode
                          ? POT_MAP_LINEAR(norms[POT_CH_KP], POT_POS_KP_MIN,
                                           POT_POS_KP_MAX)
                          : POT_MAP_LINEAR(norms[POT_CH_KP], POT_SPEED_KP_MIN,
                                           POT_SPEED_KP_MAX);
    float mapped_ki = cur_pos_mode
                          ? POT_MAP_LINEAR(norms[POT_CH_KI], POT_POS_KI_MIN,
                                           POT_POS_KI_MAX)
                          : POT_MAP_LINEAR(norms[POT_CH_KI], POT_SPEED_KI_MIN,
                                           POT_SPEED_KI_MAX);
    float mapped_kd = cur_pos_mode
                          ? POT_MAP_LINEAR(norms[POT_CH_KD], POT_POS_KD_MIN,
                                           POT_POS_KD_MAX)
                          : POT_MAP_LINEAR(norms[POT_CH_KD], POT_SPEED_KD_MIN,
                                           POT_SPEED_KD_MAX);
    float mapped_sp =
        cur_open_mode
            ? POT_MAP_LINEAR(norms[POT_CH_SP], POT_OPEN_SP_MIN, POT_OPEN_SP_MAX)
            : (cur_pos_mode
                   ? POT_MAP_LINEAR(norms[POT_CH_SP], POT_POS_SP_MIN,
                                    POT_POS_SP_MAX)
                   : POT_MAP_LINEAR(norms[POT_CH_SP], POT_SPEED_SP_MIN,
                                    POT_SPEED_SP_MAX));

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
                        (cur_open_mode != pot_state.last_open_loop_mode);
    bool force_sync = pot_force_sync || mode_changed;

    if (pot_enabled && !encoder_fault && !pulse_active) {
      const float mapped_vals[POT_CH_COUNT] = {
          mapped_kp,
          mapped_ki,
          mapped_kd,
          mapped_sp,
      };
      bool all_posted = true;
      for (int ch = 0; ch < POT_CH_COUNT; ch++) {
        if (force_sync ||
            fabsf(norms[ch] - pot_state.last_applied_norm[ch]) >=
                POT_DEADBAND_NORM) {
          if (post_motor_event(pot_events[ch], mapped_vals[ch], forward)) {
            pot_state.last_applied_norm[ch] = norms[ch];
          } else {
            all_posted = false;
          }
        }
      }
      if (all_posted) {
        pot_force_sync = false;
        pot_state.last_mode_valid = true;
        pot_state.last_position_mode = cur_pos_mode;
        pot_state.last_open_loop_mode = cur_open_mode;
      }
    }

    vTaskDelay(period_ticks);
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
  ESP_ERROR_CHECK(
      uart_driver_install(UART_PORT, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
  log_queue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(log_message_t));
  ESP_ERROR_CHECK(log_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  motor_event_queue =
      xQueueCreate(MOTOR_EVENT_QUEUE_LENGTH, sizeof(motor_event_t));
  ESP_ERROR_CHECK(motor_event_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  xTaskCreate(serial_task, "serial_task", 3072, NULL, 5, NULL);
  xTaskCreate(control_task, "control_task", 3072, NULL, 6, NULL);
  xTaskCreate(potentiometer_task, "pot_task", POT_TASK_STACK_SIZE, NULL,
              POT_TASK_PRIORITY, NULL);
  log_message_t startup = {.kind = LOG_EVENT};
  snprintf(startup.text, sizeof(startup.text), "READY A=%d B=%d C=%d Q=%d",
           ENCODER_A_GPIO, ENCODER_B_GPIO, ENCODER_CYCLES_PER_CHANNEL,
           QUADRATURE_FACTOR);
  xQueueSend(log_queue, &startup, portMAX_DELAY);
  log_message_t help = {.kind = LOG_EVENT};
  snprintf(help.text, sizeof(help.text), "HELP");
  xQueueSend(log_queue, &help, portMAX_DELAY);
}
