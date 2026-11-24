#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_pins.h"
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
#include "tc802a_driver.h"
#include "tc802a_hal.h"

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
#define LOG_QUEUE_LENGTH 24
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

/* --- Rangos de equivalencia empirica: Modo Velocidad (PID Velocidad + SP en
 * RPM) --- */
#define POT_SPEED_KP_MIN 0.00f
#define POT_SPEED_KP_MAX 1.00f
#define POT_SPEED_KI_MIN 0.00f
#define POT_SPEED_KI_MAX 0.50f
#define POT_SPEED_KD_MIN 0.000f
#define POT_SPEED_KD_MAX 0.050f
#define POT_SPEED_SP_MIN 0.0f
#define POT_SPEED_SP_MAX 10.0f

/* --- Rangos de equivalencia empirica: Modo Posicion (PID Posicion + SP en
 * grados) --- */
#define POT_POS_KP_MIN 0.00f
#define POT_POS_KP_MAX 5.00f
#define POT_POS_KI_MIN 0.00f
#define POT_POS_KI_MAX 5.00f
#define POT_POS_KD_MIN 0.000f
#define POT_POS_KD_MAX 0.500f
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
#define POT_CLOCK_KP_MAX 2.50f
#define POT_CLOCK_KI_MIN 0.00f
#define POT_CLOCK_KI_MAX 1.00f
#define POT_CLOCK_KD_MIN 0.000f
#define POT_CLOCK_KD_MAX 0.500f

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
#define NVS_CONFIG_VERSION 4U
#define NVS_TASK_STACK_SIZE 4096
#define NVS_TASK_PRIORITY 3
#define NVS_QUEUE_LENGTH 8
#define NVS_AUTOSAVE_DEBOUNCE_MS 2000
#define NVS_DEFAULT_AUTOSAVE 1

/* --- Configuracion del display LCD 8x2 TC802A-01(R) y tarea FreeRTOS --- */
#define LCD_TASK_STACK_SIZE 3072
#define LCD_TASK_PRIORITY 3
#define LCD_QUEUE_LENGTH 8
#define LCD_REFRESH_PERIOD_MS 100
#define LCD_BRINGUP_ALL_ON_MS 2000
#define LCD_BRINGUP_PATTERN_HOLD_MS 3000

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
static volatile bool pos_log_enabled = false;
static volatile bool pos_out_log_enabled = true;
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

typedef enum {
  FAULT_NONE = 0,
  FAULT_ENCODER_STALL,
  FAULT_SETPOINT_TIMEOUT_REVS,
} motor_fault_reason_t;

static motor_state_t motor_state = FSM_IDLE;
static volatile motor_fault_reason_t motor_fault_reason = FAULT_NONE;
static volatile int64_t safety_unsettled_counts = 0;
static volatile uint32_t safety_settle_ticks = 0;
static volatile float safety_filtered_rpm = 0.0f;

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
  EVENT_SETPOINT_TIMEOUT_REVS,
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
static QueueHandle_t lcd_request_queue;
static motor_pid_t speed_pid;
static motor_pid_t position_pid;
static motor_pid_t clock_pid;
static pcnt_unit_handle_t encoder_unit;
static pcnt_unit_handle_t ec11_unit;
static QueueHandle_t log_queue;
static pot_adc_channel_t pot_adc_channels[POT_CH_COUNT];
static adc_cali_handle_t pot_adc_cali_handle = NULL;
static pot_state_t pot_state;
static volatile bool pot_enabled = (POT_DEFAULT_ENABLED != 0);
static volatile bool pot_force_sync = false;
static volatile bool ec11_enabled = (EC11_DEFAULT_ENABLED != 0);
static volatile int64_t ec11_total_counts = 0;
static int ec11_detent_accum = 0;
static portMUX_TYPE pot_state_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE config_state_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE lcd_state_lock = portMUX_INITIALIZER_UNLOCKED;
static nvs_stored_config_t nvs_last_saved_config;
static volatile bool nvs_storage_ready = false;
static volatile bool nvs_config_dirty = false;
static volatile bool nvs_autosave_enabled = (NVS_DEFAULT_AUTOSAVE != 0);
static volatile uint32_t nvs_save_count = 0;
static volatile float live_measured_rpm = 0.0f;
static volatile uint8_t live_applied_duty = 0;

typedef enum {
  UI_MODE_SPEED = 0,
  UI_MODE_POSITION,
  UI_MODE_CLOCK,
  UI_MODE_OPEN_LOOP,
  UI_MODE_COUNT,
} ui_selectable_mode_t;

typedef enum {
  UI_ITEM_MODE = 0,
  UI_ITEM_SP,
  UI_ITEM_KP,
  UI_ITEM_KI,
  UI_ITEM_KD,
  UI_ITEM_COUNT,
} ui_menu_item_t;

typedef enum {
  UI_STATE_HOME = 0,
  UI_STATE_MAIN_MENU,
  UI_STATE_MODE_SELECT,
  UI_STATE_MODE_CONFIRM,
  UI_STATE_EDIT_PARAM,
} ui_menu_state_t;

typedef enum {
  LCD_VIEW_BRINGUP_PATTERN = 0,
  LCD_VIEW_DASHBOARD,
  LCD_VIEW_PID,
} lcd_view_mode_t;

typedef enum {
  LCD_REQ_BRINGUP = 0,
  LCD_REQ_SET_VIEW_PATTERN,
  LCD_REQ_SET_VIEW_DASH,
  LCD_REQ_SET_VIEW_PID,
  LCD_REQ_SET_ALL_ON,
  LCD_REQ_SET_CURSOR,
  LCD_REQ_SET_BLINK,
  LCD_REQ_SET_DISPLAY,
  LCD_REQ_CLEAR,
} lcd_req_type_t;

typedef struct {
  lcd_req_type_t type;
  bool flag;
} lcd_request_t;

typedef struct {
  bool initialized;
  lcd_view_mode_t view_mode;
  bool display_on;
  bool cursor_on;
  bool blink_on;
  bool all_blocks_on;
  uint32_t frames_rendered;
  char line0[TC802A_COLS + 1U];
  char line1[TC802A_COLS + 1U];
} lcd_status_snapshot_t;

typedef struct {
  motor_state_t state;
  motor_fault_reason_t fault_reason;
  bool forward;
  bool position_mode;
  bool open_loop_mode;
  bool clock_mode;
  float target_rpm;
  float target_position_counts;
  float measured_rpm;
  int64_t encoder_position_counts;
  uint32_t clock_step_count;
  uint8_t applied_duty;
  uint8_t manual_duty;
  pid_config_t active_pid;
  float counts_per_rev;
  bool pot_enabled;
  bool nvs_ready;
  bool nvs_dirty;
} display_telemetry_snapshot_t;

static volatile ui_menu_state_t ui_menu_state = UI_STATE_HOME;
static volatile ui_menu_item_t ui_selected_item = UI_ITEM_MODE;
static volatile ui_selectable_mode_t ui_candidate_mode = UI_MODE_SPEED;
static volatile uint32_t ui_last_activity_ms = 0U;

static lcd_status_snapshot_t lcd_status_snapshot = {
    .display_on = true,
    .line0 = "        ",
    .line1 = "        ",
};

typedef enum {
  LOG_EVENT,
  LOG_TELEMETRY,
  LOG_POSITION,
} log_kind_t;

typedef struct {
  log_kind_t kind;
  char text[128];
  int64_t position_counts;
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

static void ui_reset_to_home(void);

static void reset_safety_search_tracker(void) {
  safety_unsettled_counts = 0;
  safety_settle_ticks = 0;
  safety_filtered_rpm = 0.0f;
}

static void reset_encoder_position_count(void) {
  encoder_position_counts = 0;
  target_position_counts = 0.0f;
  clock_base_counts = 0;
  clock_step_count = 0;
  if (encoder_unit != NULL) {
    pcnt_unit_clear_count(encoder_unit);
  }
}

static void latch_encoder_fault(void) {
  encoder_fault = true;
  motor_fault_reason = FAULT_ENCODER_STALL;
  motor_state = FSM_FAULT;
  running = false;
  reset_safety_search_tracker();
  motor_pid_reset(&speed_pid);
  motor_pid_reset(&position_pid);
  motor_pid_reset(&clock_pid);
  stop_motor();
  ui_reset_to_home();
  log_message_t message = {.kind = LOG_EVENT};
  snprintf(message.text, sizeof(message.text), "STALL ENC/MOT; LOCK STOP");
  xQueueSend(log_queue, &message, 0);
}

static void latch_setpoint_revs_fault(float traveled_revs) {
  const char *mode_name =
      clock_mode ? "CLOCK"
                 : (position_mode ? "POS"
                                  : (open_loop_mode ? "OPEN" : "SPEED"));
  encoder_fault = true;
  motor_fault_reason = FAULT_SETPOINT_TIMEOUT_REVS;
  motor_state = FSM_FAULT;
  running = false;
  pos_in_target = false;
  reset_safety_search_tracker();
  motor_pid_reset(&speed_pid);
  motor_pid_reset(&position_pid);
  motor_pid_reset(&clock_pid);
  stop_motor();
  ui_reset_to_home();
  log_message_t message = {.kind = LOG_EVENT};
  snprintf(message.text, sizeof(message.text),
           "SAFETY STOP: NO SP EN 1.5 VUELTAS (MODO=%s REV=%.2f); LOCK STOP",
           mode_name, (double)traveled_revs);
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

typedef struct {
  float min_val;
  float max_val;
} pot_range_t;

static float pot_clamp_range(float value, float min_val, float max_val) {
  if (!isfinite(value)) {
    return min_val;
  }
  if (value < min_val) {
    return min_val;
  }
  if (value > max_val) {
    return max_val;
  }
  return value;
}

static float pot_map_clamped(float norm, float min_val, float max_val) {
  float clamped_norm = pot_clamp_range(norm, 0.0f, 1.0f);
  float mapped = POT_MAP_LINEAR(clamped_norm, min_val, max_val);
  return pot_clamp_range(mapped, min_val, max_val);
}

static const pot_range_t *get_pid_pot_ranges(bool is_clock, bool is_pos) {
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
  if (is_clock) {
    return clock_pot_ranges;
  }
  if (is_pos) {
    return pos_pot_ranges;
  }
  return speed_pot_ranges;
}

static ui_selectable_mode_t ui_current_mode(void) {
  if (open_loop_mode) {
    return UI_MODE_OPEN_LOOP;
  }
  if (clock_mode) {
    return UI_MODE_CLOCK;
  }
  if (position_mode) {
    return UI_MODE_POSITION;
  }
  return UI_MODE_SPEED;
}

static motor_event_type_t ui_mode_event(ui_selectable_mode_t mode) {
  switch (mode) {
  case UI_MODE_POSITION:
    return EVENT_MODE_POSITION;
  case UI_MODE_CLOCK:
    return EVENT_MODE_CLOCK;
  case UI_MODE_OPEN_LOOP:
    return EVENT_MODE_OPEN_LOOP;
  case UI_MODE_SPEED:
  default:
    return EVENT_MODE_SPEED;
  }
}

static const char *ui_mode_short_label(ui_selectable_mode_t mode) {
  switch (mode) {
  case UI_MODE_POSITION:
    return "POS";
  case UI_MODE_CLOCK:
    return "CLK";
  case UI_MODE_OPEN_LOOP:
    return "OPEN";
  case UI_MODE_SPEED:
  default:
    return "SPD";
  }
}

static int ui_item_to_pot_channel(ui_menu_item_t item) {
  switch (item) {
  case UI_ITEM_SP:
    return (int)POT_CH_SP;
  case UI_ITEM_KP:
    return (int)POT_CH_KP;
  case UI_ITEM_KI:
    return (int)POT_CH_KI;
  case UI_ITEM_KD:
    return (int)POT_CH_KD;
  case UI_ITEM_MODE:
  default:
    return -1;
  }
}

static void ui_reset_to_home(void) {
  taskENTER_CRITICAL(&lcd_state_lock);
  ui_menu_state = UI_STATE_HOME;
  ui_selected_item = UI_ITEM_MODE;
  taskEXIT_CRITICAL(&lcd_state_lock);
}

static void update_active_pid_gain(motor_event_type_t type, float value) {
  taskENTER_CRITICAL(&config_state_lock);
  const pot_range_t *ranges = get_pid_pot_ranges(clock_mode, position_mode);
  pid_config_t *cfg =
      clock_mode ? &clock_pid_config
                 : (position_mode ? &position_pid_config : &speed_pid_config);
  motor_pid_t *pid =
      clock_mode ? &clock_pid : (position_mode ? &position_pid : &speed_pid);
  if (type == EVENT_POT_KP_UPDATE) {
    float clamped =
        pot_clamp_range(value, ranges[0].min_val, ranges[0].max_val);
    cfg->kp = clamped;
    pid->kp = clamped;
  } else if (type == EVENT_POT_KI_UPDATE) {
    float clamped =
        pot_clamp_range(value, ranges[1].min_val, ranges[1].max_val);
    cfg->ki = clamped;
    pid->ki = clamped;
  } else if (type == EVENT_POT_KD_UPDATE) {
    float clamped =
        pot_clamp_range(value, ranges[2].min_val, ranges[2].max_val);
    cfg->kd = clamped;
    pid->kd = clamped;
  }
  taskEXIT_CRITICAL(&config_state_lock);
}

static void handle_motor_event(const motor_event_t *event) {
  bool reset_pids = true;
  motor_state_t prev_state = motor_state;
  bool was_running = running;
  bool was_speed_mode = !position_mode && !open_loop_mode && !clock_mode;
  bool was_position_mode = position_mode && !open_loop_mode && !clock_mode;
  bool was_open_loop_mode = open_loop_mode && !position_mode && !clock_mode;

  switch (event->type) {
  case EVENT_STOP:
    target_rpm = 0.0f;
    manual_duty = 0;
    running = false;
    encoder_fault = false;
    motor_fault_reason = FAULT_NONE;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    pos_in_target = false;
    motor_state = FSM_IDLE;
    reset_safety_search_tracker();
    break;
  case EVENT_MODE_SPEED:
    position_mode = false;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    manual_duty = 0;
    running = false;
    pos_in_target = false;
    if (!encoder_fault) {
      motor_fault_reason = FAULT_NONE;
    }
    motor_state = FSM_IDLE;
    reset_encoder_position_count();
    reset_safety_search_tracker();
    break;
  case EVENT_MODE_POSITION:
    position_mode = true;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    manual_duty = 0;
    running = false;
    pos_in_target = false;
    if (!encoder_fault) {
      motor_fault_reason = FAULT_NONE;
    }
    motor_state = FSM_IDLE;
    reset_encoder_position_count();
    reset_safety_search_tracker();
    break;
  case EVENT_MODE_CLOCK:
    position_mode = false;
    open_loop_mode = false;
    clock_mode = true;
    manual_duty = 0;
    encoder_fault = false;
    motor_fault_reason = FAULT_NONE;
    pos_in_target = false;
    clock_elapsed_ms = 0;
    reset_encoder_position_count();
    motor_state = FSM_CLOCK;
    running = true;
    reset_safety_search_tracker();
    break;
  case EVENT_MODE_OPEN_LOOP:
    position_mode = false;
    open_loop_mode = true;
    clock_mode = false;
    clock_elapsed_ms = 0;
    running = false;
    manual_duty = 0;
    pos_in_target = false;
    if (!encoder_fault) {
      motor_fault_reason = FAULT_NONE;
    }
    motor_state = FSM_IDLE;
    reset_encoder_position_count();
    reset_safety_search_tracker();
    break;
  case EVENT_SPEED_SETPOINT:
    if (!was_speed_mode) {
      reset_encoder_position_count();
    }
    target_rpm = event->value;
    forward = event->forward;
    position_mode = false;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    motor_state = target_rpm > 0.0f ? FSM_SPEED : FSM_IDLE;
    running = target_rpm > 0.0f;
    if (!was_speed_mode || prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker();
    } else {
      safety_settle_ticks = 0;
    }
    break;
  case EVENT_POSITION_SETPOINT:
  case EVENT_ANGLE_SETPOINT:
    if (!was_position_mode) {
      reset_encoder_position_count();
    }
    target_position_counts = event->value;
    pos_in_target = false;
    position_mode = true;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    motor_state = FSM_POSITION;
    running = true;
    if (!was_position_mode || prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker();
    } else {
      safety_settle_ticks = 0;
    }
    break;
  case EVENT_DUTY_SETPOINT:
    if (!was_open_loop_mode) {
      reset_encoder_position_count();
    }
    target_rpm = 0.0f;
    forward = event->forward;
    position_mode = false;
    open_loop_mode = true;
    clock_mode = false;
    clock_elapsed_ms = 0;
    manual_duty = (uint8_t)fminf(fmaxf(event->value, 0.0f), PWM_MAX_DUTY);
    motor_state = manual_duty > 0 ? FSM_OPEN_LOOP : FSM_IDLE;
    running = manual_duty > 0;
    if (!was_open_loop_mode || prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker();
    } else {
      safety_settle_ticks = 0;
    }
    break;
  case EVENT_ENCODER_STALL:
    latch_encoder_fault();
    return;
  case EVENT_SETPOINT_TIMEOUT_REVS:
    latch_setpoint_revs_fault(event->value);
    return;
  case EVENT_CALIBRATE_START:
    target_rpm = 0.0f;
    manual_duty = 0;
    running = false;
    encoder_fault = false;
    motor_fault_reason = FAULT_NONE;
    open_loop_mode = false;
    clock_mode = false;
    clock_elapsed_ms = 0;
    pos_in_target = false;
    motor_state = FSM_IDLE;
    reset_encoder_position_count();
    reset_safety_search_tracker();
    stop_motor();
    break;
  case EVENT_POT_KP_UPDATE:
  case EVENT_POT_KI_UPDATE:
  case EVENT_POT_KD_UPDATE:
    update_active_pid_gain(event->type, event->value);
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
    if (prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker();
    } else {
      safety_settle_ticks = 0;
    }
    if (open_loop_mode) {
      float sp =
          pot_clamp_range(event->value, POT_OPEN_SP_MIN, POT_OPEN_SP_MAX);
      manual_duty = (uint8_t)lroundf(sp);
      motor_state = manual_duty > 0 ? FSM_OPEN_LOOP : FSM_IDLE;
      running = manual_duty > 0;
    } else if (position_mode) {
      float sp_deg =
          pot_clamp_range(event->value, POT_POS_SP_MIN, POT_POS_SP_MAX);
      target_position_counts =
          sp_deg * encoder_counts_per_revolution() / 360.0f;
      pos_in_target = false;
      motor_state = FSM_POSITION;
      running = true;
    } else {
      target_rpm =
          pot_clamp_range(event->value, POT_SPEED_SP_MIN, POT_SPEED_SP_MAX);
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
      "POT ON/OFF/GET|EC11 GET/ON/OFF/ZERO|SAVE|LOAD|NVS SAVE/LOAD/RESET/GET/"
      "AUTOSAVE|LCD STATUS/BRINGUP/PATTERN/DASH/PID/ALLON/CURSOR/BLINK/"
      "DISPLAY/CLEAR|STOP|STATUS|LOG ON/OFF|LOG POS ON/OFF|LOG POSOUT ON/OFF|"
      "LOG RATE <ms>\r\n");
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

static float encoder_counts_to_degrees(int64_t counts, float counts_per_rev) {
  return (counts_per_rev > 0.0f) ? ((float)counts * 360.0f / counts_per_rev)
                                 : 0.0f;
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
  } else if (message->kind == LOG_POSITION) {
    printf("P,%" PRId64 ",%.2f\n", message->position_counts,
           message->measured_angle);
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

static bool post_lcd_request(lcd_req_type_t type, bool flag) {
  if (lcd_request_queue == NULL) {
    return false;
  }
  lcd_request_t req = {
      .type = type,
      .flag = flag,
  };
  return xQueueSend(lcd_request_queue, &req, 0) == pdTRUE;
}

static bool parse_on_off_arg(const char *arg, bool *out_val) {
  if (arg == NULL || out_val == NULL) {
    return false;
  }
  if (strcmp(arg, "ON") == 0 || strcmp(arg, "1") == 0) {
    *out_val = true;
    return true;
  }
  if (strcmp(arg, "OFF") == 0 || strcmp(arg, "0") == 0) {
    *out_val = false;
    return true;
  }
  return false;
}

static const char *lcd_view_mode_name(lcd_view_mode_t view) {
  switch (view) {
  case LCD_VIEW_BRINGUP_PATTERN:
    return "PATTERN";
  case LCD_VIEW_PID:
    return "PID";
  case LCD_VIEW_DASHBOARD:
  default:
    return "DASH";
  }
}

static void process_lcd_command(const char *value_text) {
  char subcmd[16] = {0};
  char arg[16] = {0};
  int parsed = sscanf(value_text, "%15s %15s", subcmd, arg);
  if (parsed >= 1) {
    for (char *p = subcmd; *p != '\0'; p++) {
      *p = (char)toupper((unsigned char)*p);
    }
  }
  if (parsed >= 2) {
    for (char *p = arg; *p != '\0'; p++) {
      *p = (char)toupper((unsigned char)*p);
    }
  }

  if (parsed <= 0 || strcmp(subcmd, "STATUS") == 0 ||
      strcmp(subcmd, "GET") == 0) {
    lcd_status_snapshot_t snap;
    taskENTER_CRITICAL(&lcd_state_lock);
    snap = lcd_status_snapshot;
    taskEXIT_CRITICAL(&lcd_state_lock);
    printf("LCD INIT=%d TYPE=8x2 VIEW=%s DISP=%d CUR=%d BLINK=%d ALLON=%d "
           "FRAMES=%" PRIu32
           " RS=%d E=%d D4=%d D5=%d D6=%d D7=%d L0=\"%s\" L1=\"%s\"\r\n",
           snap.initialized ? 1 : 0, lcd_view_mode_name(snap.view_mode),
           snap.display_on ? 1 : 0, snap.cursor_on ? 1 : 0,
           snap.blink_on ? 1 : 0, snap.all_blocks_on ? 1 : 0,
           snap.frames_rendered, PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5,
           PIN_LCD_D6, PIN_LCD_D7, snap.line0, snap.line1);
    return;
  }
  if (strcmp(subcmd, "BRINGUP") == 0 || strcmp(subcmd, "TEST") == 0) {
    emit_event(post_lcd_request(LCD_REQ_BRINGUP, false) ? "OK LCD BRINGUP"
                                                        : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "PATTERN") == 0) {
    emit_event(post_lcd_request(LCD_REQ_SET_VIEW_PATTERN, false)
                   ? "OK LCD PATTERN"
                   : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "DASH") == 0 || strcmp(subcmd, "DASHBOARD") == 0) {
    emit_event(post_lcd_request(LCD_REQ_SET_VIEW_DASH, false) ? "OK LCD DASH"
                                                              : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "PID") == 0) {
    emit_event(post_lcd_request(LCD_REQ_SET_VIEW_PID, false) ? "OK LCD PID"
                                                             : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "CLEAR") == 0 || strcmp(subcmd, "CLS") == 0) {
    emit_event(post_lcd_request(LCD_REQ_CLEAR, false) ? "OK LCD CLEAR"
                                                      : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "ALLON") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      emit_event("ERR LCD ALLON");
      return;
    }
    if (post_lcd_request(LCD_REQ_SET_ALL_ON, enabled)) {
      emit_event(enabled ? "OK LCD ALLON ON" : "OK LCD ALLON OFF");
    } else {
      emit_event("ERR LCD");
    }
    return;
  }
  if (strcmp(subcmd, "CURSOR") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      emit_event("ERR LCD CURSOR");
      return;
    }
    if (post_lcd_request(LCD_REQ_SET_CURSOR, enabled)) {
      emit_event(enabled ? "OK LCD CURSOR ON" : "OK LCD CURSOR OFF");
    } else {
      emit_event("ERR LCD");
    }
    return;
  }
  if (strcmp(subcmd, "BLINK") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      emit_event("ERR LCD BLINK");
      return;
    }
    if (post_lcd_request(LCD_REQ_SET_BLINK, enabled)) {
      emit_event(enabled ? "OK LCD BLINK ON" : "OK LCD BLINK OFF");
    } else {
      emit_event("ERR LCD");
    }
    return;
  }
  if (strcmp(subcmd, "DISPLAY") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      emit_event("ERR LCD DISPLAY");
      return;
    }
    if (post_lcd_request(LCD_REQ_SET_DISPLAY, enabled)) {
      emit_event(enabled ? "OK LCD DISPLAY ON" : "OK LCD DISPLAY OFF");
    } else {
      emit_event("ERR LCD");
    }
    return;
  }
  emit_event("ERR LCD");
}

static void process_ec11_command(const char *value_text) {
  char subcmd[16] = {0};
  if (sscanf(value_text, "%15s", subcmd) == 1) {
    for (char *p = subcmd; *p != '\0'; p++) {
      *p = (char)toupper((unsigned char)*p);
    }
  }
  if (subcmd[0] == '\0' || strcmp(subcmd, "GET") == 0 ||
      strcmp(subcmd, "STATUS") == 0 || strcmp(subcmd, "READ") == 0) {
    int64_t counts = 0;
    taskENTER_CRITICAL(&config_state_lock);
    counts = ec11_total_counts;
    taskEXIT_CRITICAL(&config_state_lock);
    int sw_pressed = (gpio_get_level(EC11_SW_GPIO) == 0) ? 1 : 0;
    printf("EC11 EN=%d CNT=%" PRId64 " DET=%" PRId64
           " SW=%d CLK=%d DT=%d SW_PIN=%d\r\n",
           ec11_enabled ? 1 : 0, counts,
           counts / (int64_t)EC11_COUNTS_PER_DETENT, sw_pressed, PIN_EC11_CLK,
           PIN_EC11_DT, PIN_EC11_SW);
    return;
  }
  if (strcmp(subcmd, "ON") == 0) {
    ec11_enabled = true;
    emit_event("OK EC11 ON");
    return;
  }
  if (strcmp(subcmd, "OFF") == 0) {
    ec11_enabled = false;
    emit_event("OK EC11 OFF");
    return;
  }
  if (strcmp(subcmd, "ZERO") == 0 || strcmp(subcmd, "RESET") == 0) {
    taskENTER_CRITICAL(&config_state_lock);
    ec11_total_counts = 0;
    ec11_detent_accum = 0;
    taskEXIT_CRITICAL(&config_state_lock);
    if (ec11_unit != NULL) {
      pcnt_unit_clear_count(ec11_unit);
    }
    emit_event("OK EC11 ZERO");
    return;
  }
  emit_event("ERR EC11");
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
    bool ok = post_motor_event(EVENT_STOP, 0.0f, true);
    if (ok) {
      ui_reset_to_home();
    }
    emit_event(ok ? "OK STOP" : "ERR EVT");
    return;
  }
  if (encoder_fault && strcmp(operation, "HELP") != 0 &&
      strcmp(operation, "?") != 0 && strcmp(operation, "STATUS") != 0 &&
      strcmp(operation, "CAL") != 0 && strcmp(operation, "EUR") != 0 &&
      strcmp(operation, "CALIB") != 0 && strcmp(operation, "SAVE") != 0 &&
      strcmp(operation, "LOAD") != 0 && strcmp(operation, "NVS") != 0 &&
      strcmp(operation, "LCD") != 0 && strcmp(operation, "EC11") != 0 &&
      strcmp(operation, "LOG") != 0) {
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
    bool ok = post_motor_event(mode_event, 0.0f, true);
    if (ok) {
      ui_reset_to_home();
    }
    emit_event(ok ? ok_msg : "ERR EVT");
    return;
  }
  if (strcmp(operation, "CLOCK") == 0 || strcmp(operation, "CLK") == 0) {
    if (value_text[0] != '\0') {
      emit_event("ERR MODE");
      return;
    }
    bool ok = post_motor_event(EVENT_MODE_CLOCK, 0.0f, true);
    if (ok) {
      ui_reset_to_home();
    }
    emit_event(ok ? "OK MODE CLOCK" : "ERR EVT");
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
    char arg[16] = {0};
    int parsed = sscanf(value_text, "%11s %15s", option, arg);
    if (parsed < 1) {
      emit_event("ERR LOG");
      return;
    }
    for (char *p = option; *p != '\0'; p++) {
      *p = (char)toupper((unsigned char)*p);
    }
    if (parsed >= 2) {
      for (char *p = arg; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
      }
    }
    if (strcmp(option, "ON") == 0 && parsed == 1) {
      taskENTER_CRITICAL(&config_state_lock);
      telemetry_enabled = true;
      taskEXIT_CRITICAL(&config_state_lock);
      notify_config_modified();
      emit_event("OK LOG ON");
    } else if (strcmp(option, "OFF") == 0 && parsed == 1) {
      taskENTER_CRITICAL(&config_state_lock);
      telemetry_enabled = false;
      taskEXIT_CRITICAL(&config_state_lock);
      notify_config_modified();
      emit_event("OK LOG OFF");
    } else if (strcmp(option, "POS") == 0 || strcmp(option, "P") == 0) {
      if (parsed == 1) {
        int64_t counts_now = 0;
        float cpr = 0.0f;
        taskENTER_CRITICAL(&config_state_lock);
        counts_now = encoder_position_counts;
        cpr = encoder_counts_per_revolution();
        taskEXIT_CRITICAL(&config_state_lock);
        float deg_now = encoder_counts_to_degrees(counts_now, cpr);
        printf("P,%" PRId64 ",%.2f\n", counts_now, deg_now);
      } else {
        bool enabled = false;
        if (!parse_on_off_arg(arg, &enabled)) {
          emit_event("ERR LOG");
          return;
        }
        taskENTER_CRITICAL(&config_state_lock);
        pos_log_enabled = enabled;
        if (enabled) {
          pos_out_log_enabled = false;
        }
        taskEXIT_CRITICAL(&config_state_lock);
        emit_event(enabled ? "OK LOG POS ON" : "OK LOG POS OFF");
      }
    } else if (strcmp(option, "POSOUT") == 0 || strcmp(option, "OUT") == 0 ||
               strcmp(option, "POS_OUT") == 0 || strcmp(option, "DBG") == 0) {
      bool enabled = false;
      if (parsed != 2 || !parse_on_off_arg(arg, &enabled)) {
        emit_event("ERR LOG");
        return;
      }
      taskENTER_CRITICAL(&config_state_lock);
      pos_out_log_enabled = enabled;
      taskEXIT_CRITICAL(&config_state_lock);
      emit_event(enabled ? "OK LOG POSOUT ON" : "OK LOG POSOUT OFF");
    } else if (strcmp(option, "RATE") == 0 && parsed == 2) {
      char *rate_end = NULL;
      unsigned long period = strtoul(arg, &rate_end, 10);
      if (isdigit((unsigned char)arg[0]) && *rate_end == '\0' &&
          period >= CONTROL_PERIOD_MS && period <= UINT32_MAX) {
        taskENTER_CRITICAL(&config_state_lock);
        telemetry_period_ms = (uint32_t)period;
        taskEXIT_CRITICAL(&config_state_lock);
        notify_config_modified();
        printf("OK LOG RATE %lu\r\n", period);
      } else {
        emit_event("ERR LOG");
      }
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
  if (strcmp(operation, "LCD") == 0 || strcmp(operation, "DISPLAY") == 0) {
    process_lcd_command(value_text);
    return;
  }
  if (strcmp(operation, "EC11") == 0) {
    process_ec11_command(value_text);
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
    int length = uart_read_bytes(UART_PORT, data, sizeof(data),
                                 pdMS_TO_TICKS(CONTROL_PERIOD_MS));
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

static void init_quadrature_pcnt_unit(gpio_num_t gpio_a, gpio_num_t gpio_b,
                                      uint32_t max_glitch_ns,
                                      bool enable_pullups,
                                      pcnt_unit_handle_t *out_unit) {
  const pcnt_unit_config_t unit_config = {
      .high_limit = 32767,
      .low_limit = -32768,
  };
  const pcnt_chan_config_t channel_a_config = {
      .edge_gpio_num = gpio_a,
      .level_gpio_num = gpio_b,
  };
  const pcnt_chan_config_t channel_b_config = {
      .edge_gpio_num = gpio_b,
      .level_gpio_num = gpio_a,
  };
  pcnt_channel_handle_t channel_a = NULL;
  pcnt_channel_handle_t channel_b = NULL;

  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, out_unit));
  ESP_ERROR_CHECK(pcnt_new_channel(*out_unit, &channel_a_config, &channel_a));
  ESP_ERROR_CHECK(pcnt_new_channel(*out_unit, &channel_b_config, &channel_b));
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
  if (enable_pullups) {
    ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_a, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_b, GPIO_PULLUP_ONLY));
  }
  if (max_glitch_ns > 0U) {
    const pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = max_glitch_ns,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*out_unit, &filter_cfg));
  }
  ESP_ERROR_CHECK(pcnt_unit_enable(*out_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(*out_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(*out_unit));
}

static void encoder_init(void) {
  /* Orden B/A invertido en PCNT para que el giro horario (forward = true)
   * cuente positivo y sea consistente con el lazo cerrado. */
  init_quadrature_pcnt_unit(ENCODER_B_GPIO, ENCODER_A_GPIO,
                            ENCODER_GLITCH_FILTER_NS, false, &encoder_unit);
}

static void ec11_init(void) {
  init_quadrature_pcnt_unit(EC11_CLK_GPIO, EC11_DT_GPIO, EC11_GLITCH_FILTER_NS,
                            true, &ec11_unit);
  const gpio_config_t sw_cfg = {
      .pin_bit_mask = (1ULL << EC11_SW_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&sw_cfg));
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
  uint32_t pos_log_elapsed_ms = 0;
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
        applied_duty = run_position_pid_step(
            &position_pid, target_position_counts, sample_time,
            pos_out_log_enabled, false, &movement_expected);
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
        if (fabsf(target_rpm) > 0.0f && abs_duty > 0.0f) {
          if (abs_duty < (float)min_move_duty) {
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

    if (motor_state == FSM_SPEED || motor_state == FSM_POSITION ||
        motor_state == FSM_CLOCK || motor_state == FSM_OPEN_LOOP) {
      float max_search_counts = SAFETY_MAX_SEARCH_REVS * counts_per_revolution;
      safety_filtered_rpm +=
          SAFETY_SPEED_RPM_EMA_ALPHA * (measured_rpm - safety_filtered_rpm);
      bool setpoint_reached = false;
      bool clock_lag_exceeded = false;

      if (motor_state == FSM_POSITION || motor_state == FSM_CLOCK) {
        float pos_error =
            fabsf(target_position_counts - (float)encoder_position_counts);
        setpoint_reached = (pos_error <= (float)POSITION_TOLERANCE_COUNTS);
        if (motor_state == FSM_CLOCK && max_search_counts > 0.0f &&
            pos_error >= max_search_counts) {
          clock_lag_exceeded = true;
        }
      } else if (motor_state == FSM_SPEED) {
        if (target_rpm <= 0.0f) {
          setpoint_reached = true;
        } else {
          float rpm_per_count = 60.0f / (counts_per_revolution * sample_time);
          float speed_tol =
              fmaxf(1.5f * rpm_per_count, SAFETY_SPEED_TOL_RATIO * target_rpm);
          setpoint_reached =
              (fabsf(safety_filtered_rpm - target_rpm) <= speed_tol);
        }
      } else {
        /* En FSM_OPEN_LOOP no existe realimentacion de setpoint: el recorrido
         * continuo queda limitado a 1.5 vueltas como restriccion maxima. */
        setpoint_reached = false;
      }

      if (setpoint_reached) {
        if (safety_settle_ticks < SAFETY_SETTLE_TICKS) {
          safety_settle_ticks++;
        }
        if (safety_settle_ticks >= SAFETY_SETTLE_TICKS) {
          safety_unsettled_counts = 0;
        }
      } else {
        safety_settle_ticks = 0;
        safety_unsettled_counts += (int64_t)abs(counts);
      }

      if (max_search_counts > 0.0f &&
          ((float)safety_unsettled_counts >= max_search_counts ||
           clock_lag_exceeded)) {
        float traveled_revs =
            (counts_per_revolution > 0.0f)
                ? ((float)safety_unsettled_counts / counts_per_revolution)
                : SAFETY_MAX_SEARCH_REVS;
        if (clock_lag_exceeded && traveled_revs < SAFETY_MAX_SEARCH_REVS) {
          traveled_revs =
              fabsf(target_position_counts - (float)encoder_position_counts) /
              counts_per_revolution;
        }
        motor_event_t revs_fault_event = {
            .type = EVENT_SETPOINT_TIMEOUT_REVS,
            .value = traveled_revs,
            .forward = forward,
        };
        handle_motor_event(&revs_fault_event);
        applied_duty = 0;
        movement_expected = false;
        movement_start_ms = 0;
        no_motion_ms = 0;
      }
    } else {
      reset_safety_search_tracker();
    }
    taskENTER_CRITICAL(&config_state_lock);
    live_measured_rpm = measured_rpm;
    live_applied_duty = applied_duty;
    taskEXIT_CRITICAL(&config_state_lock);
    telemetry_elapsed_ms += CONTROL_PERIOD_MS;
    if (telemetry_enabled && telemetry_elapsed_ms >= telemetry_period_ms) {
      log_message_t message = {
          .kind = LOG_TELEMETRY,
          .reference =
              (position_mode || clock_mode)
                  ? target_position_counts * 360.0f / counts_per_revolution
                  : target_rpm,
          .measured_rpm = measured_rpm,
          .measured_angle = encoder_counts_to_degrees(encoder_position_counts,
                                                      counts_per_revolution),
          .duty = applied_duty,
      };
      if (xQueueSend(log_queue, &message, 0) != pdTRUE) {
        telemetry_elapsed_ms = telemetry_period_ms;
      } else {
        telemetry_elapsed_ms = 0;
      }
    }
    pos_log_elapsed_ms += CONTROL_PERIOD_MS;
    if (pos_log_enabled && pos_log_elapsed_ms >= telemetry_period_ms) {
      log_message_t pos_message = {
          .kind = LOG_POSITION,
          .position_counts = encoder_position_counts,
          .measured_angle = encoder_counts_to_degrees(encoder_position_counts,
                                                      counts_per_revolution),
      };
      if (xQueueSend(log_queue, &pos_message, 0) != pdTRUE) {
        pos_log_elapsed_ms = telemetry_period_ms;
      } else {
        pos_log_elapsed_ms = 0;
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
  const gpio_num_t pot_gpios[POT_CH_COUNT] = {
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

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  const adc_cali_line_fitting_config_t cali_cfg = {
      .unit_id = ADC_UNIT_1,
      .atten = POT_ADC_ATTEN,
      .bitwidth = POT_ADC_BITWIDTH,
      .default_vref = 1100U,
  };
  if (adc_cali_create_scheme_line_fitting(&cali_cfg, &pot_adc_cali_handle) !=
      ESP_OK) {
    pot_adc_cali_handle = NULL;
  }
#endif
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

static void ec11_poll_and_dispatch(void) {
  static bool sw_debounced_pressed = false;
  static bool sw_last_raw_pressed = false;
  static uint8_t sw_debounce_ticks = 0U;
  static uint32_t sw_press_start_ms = 0U;
  static bool sw_long_press_handled = false;

  uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  if (ec11_unit != NULL) {
    int delta_counts = 0;
    if (pcnt_unit_get_count(ec11_unit, &delta_counts) == ESP_OK &&
        delta_counts != 0) {
      pcnt_unit_clear_count(ec11_unit);
      taskENTER_CRITICAL(&config_state_lock);
      ec11_total_counts += delta_counts;
      ec11_detent_accum += delta_counts;
      int detents = ec11_detent_accum / EC11_COUNTS_PER_DETENT;
      ec11_detent_accum %= EC11_COUNTS_PER_DETENT;
      taskEXIT_CRITICAL(&config_state_lock);

      if (detents != 0 && ec11_enabled) {
        taskENTER_CRITICAL(&lcd_state_lock);
        ui_last_activity_ms = now_ms;
        if (ui_menu_state == UI_STATE_HOME) {
          ui_menu_state = UI_STATE_MAIN_MENU;
          int rem = (detents > 0) ? (detents - 1) : (detents + 1);
          int step = rem % (int)UI_ITEM_COUNT;
          int next = ((int)UI_ITEM_MODE + step + (int)UI_ITEM_COUNT) %
                     (int)UI_ITEM_COUNT;
          ui_selected_item = (ui_menu_item_t)next;
        } else if (ui_menu_state == UI_STATE_MAIN_MENU ||
                   ui_menu_state == UI_STATE_EDIT_PARAM) {
          ui_menu_state = UI_STATE_MAIN_MENU;
          int step = detents % (int)UI_ITEM_COUNT;
          int next = ((int)ui_selected_item + step + (int)UI_ITEM_COUNT) %
                     (int)UI_ITEM_COUNT;
          ui_selected_item = (ui_menu_item_t)next;
        } else if (ui_menu_state == UI_STATE_MODE_SELECT ||
                   ui_menu_state == UI_STATE_MODE_CONFIRM) {
          ui_menu_state = UI_STATE_MODE_SELECT;
          int step = detents % (int)UI_MODE_COUNT;
          int next = ((int)ui_candidate_mode + step + (int)UI_MODE_COUNT) %
                     (int)UI_MODE_COUNT;
          ui_candidate_mode = (ui_selectable_mode_t)next;
        }
        taskEXIT_CRITICAL(&lcd_state_lock);
      }
    }
  }

  taskENTER_CRITICAL(&lcd_state_lock);
  if (ui_menu_state != UI_STATE_HOME &&
      (now_ms - ui_last_activity_ms) >= UI_MENU_TIMEOUT_MS) {
    ui_menu_state = UI_STATE_HOME;
    ui_selected_item = UI_ITEM_MODE;
  }
  taskEXIT_CRITICAL(&lcd_state_lock);

  bool raw_pressed = (gpio_get_level(EC11_SW_GPIO) == 0);
  if (raw_pressed == sw_last_raw_pressed) {
    if (sw_debounce_ticks < EC11_SW_DEBOUNCE_TICKS) {
      sw_debounce_ticks++;
    }
  } else {
    sw_last_raw_pressed = raw_pressed;
    sw_debounce_ticks = 1U;
  }

  if (sw_debounce_ticks >= EC11_SW_DEBOUNCE_TICKS &&
      raw_pressed != sw_debounced_pressed) {
    sw_debounced_pressed = raw_pressed;
    if (sw_debounced_pressed) {
      sw_press_start_ms = now_ms;
      sw_long_press_handled = false;
    } else if (!sw_long_press_handled) {
      if (encoder_fault) {
        ui_reset_to_home();
        if (post_motor_event(EVENT_STOP, 0.0f, true)) {
          post_log_event("EC11 SW -> CLR FLT");
        }
      } else {
        ui_selectable_mode_t cur_mode = ui_current_mode();
        bool apply_mode = false;
        ui_selectable_mode_t confirmed_mode = cur_mode;
        int sync_pot_ch = -1;

        taskENTER_CRITICAL(&lcd_state_lock);
        ui_last_activity_ms = now_ms;
        switch (ui_menu_state) {
        case UI_STATE_HOME:
          ui_menu_state = UI_STATE_MAIN_MENU;
          ui_selected_item = UI_ITEM_MODE;
          break;
        case UI_STATE_MAIN_MENU:
          if (ui_selected_item == UI_ITEM_MODE) {
            ui_candidate_mode = cur_mode;
            ui_menu_state = UI_STATE_MODE_SELECT;
          } else {
            sync_pot_ch = ui_item_to_pot_channel(ui_selected_item);
            ui_menu_state = UI_STATE_EDIT_PARAM;
          }
          break;
        case UI_STATE_MODE_SELECT:
          ui_menu_state = UI_STATE_MODE_CONFIRM;
          break;
        case UI_STATE_MODE_CONFIRM:
          confirmed_mode = ui_candidate_mode;
          ui_menu_state = UI_STATE_HOME;
          ui_selected_item = UI_ITEM_MODE;
          apply_mode = true;
          break;
        case UI_STATE_EDIT_PARAM:
          ui_menu_state = UI_STATE_HOME;
          ui_selected_item = UI_ITEM_MODE;
          break;
        }
        taskEXIT_CRITICAL(&lcd_state_lock);

        if (sync_pot_ch >= 0 && sync_pot_ch < POT_CH_COUNT) {
          taskENTER_CRITICAL(&pot_state_lock);
          pot_state.last_applied_norm[sync_pot_ch] =
              pot_state.norm[sync_pot_ch];
          taskEXIT_CRITICAL(&pot_state_lock);
        }

        if (apply_mode &&
            post_motor_event(ui_mode_event(confirmed_mode), 0.0f, true)) {
          char msg[48];
          snprintf(msg, sizeof(msg), "EC11 MODE CONFIRMED -> %s",
                   ui_mode_short_label(confirmed_mode));
          post_log_event(msg);
        }
      }
    }
  }

  if (sw_debounced_pressed && !sw_long_press_handled &&
      (now_ms - sw_press_start_ms) >= EC11_SW_LONG_PRESS_MS) {
    sw_long_press_handled = true;
    ui_reset_to_home();
    if (post_motor_event(EVENT_STOP, 0.0f, true)) {
      post_log_event("EC11 LONG -> STOP");
    }
  }
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
        int cal_mv = 0;
        if (pot_adc_cali_handle != NULL &&
            adc_cali_raw_to_voltage(pot_adc_cali_handle,
                                    (int)lroundf(raw_filtered[ch]),
                                    &cal_mv) == ESP_OK) {
          voltages[ch] = (float)cal_mv / 1000.0f;
        } else {
          voltages[ch] =
              POT_MAP_LINEAR(norms[ch], POT_VOLTAGE_MIN_V, POT_VOLTAGE_MAX_V);
        }
      }
    }

    bool cur_pos_mode = position_mode;
    bool cur_open_mode = open_loop_mode;
    bool cur_clock_mode = clock_mode;
    const pot_range_t *pid_ranges =
        get_pid_pot_ranges(cur_clock_mode, cur_pos_mode);
    float mapped_kp =
        pot_map_clamped(norms[POT_CH_KP], pid_ranges[POT_CH_KP].min_val,
                        pid_ranges[POT_CH_KP].max_val);
    float mapped_ki =
        pot_map_clamped(norms[POT_CH_KI], pid_ranges[POT_CH_KI].min_val,
                        pid_ranges[POT_CH_KI].max_val);
    float mapped_kd =
        pot_map_clamped(norms[POT_CH_KD], pid_ranges[POT_CH_KD].min_val,
                        pid_ranges[POT_CH_KD].max_val);
    float mapped_sp =
        cur_clock_mode
            ? (360.0f / (float)CLOCK_DIVISIONS_PER_REV)
            : (cur_open_mode
                   ? pot_map_clamped(norms[POT_CH_SP], POT_OPEN_SP_MIN,
                                     POT_OPEN_SP_MAX)
                   : (cur_pos_mode
                          ? pot_map_clamped(norms[POT_CH_SP], POT_POS_SP_MIN,
                                            POT_POS_SP_MAX)
                          : pot_map_clamped(norms[POT_CH_SP], POT_SPEED_SP_MIN,
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

    if (pot_enabled && !encoder_fault && !pulse_active) {
      bool force_sync = pot_force_sync;
      const float mapped_vals[POT_CH_COUNT] = {
          mapped_kp,
          mapped_ki,
          mapped_kd,
          mapped_sp,
      };
      uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
      int edit_ch = -1;
      taskENTER_CRITICAL(&lcd_state_lock);
      if (ui_menu_state == UI_STATE_EDIT_PARAM) {
        edit_ch = ui_item_to_pot_channel(ui_selected_item);
      }
      taskEXIT_CRITICAL(&lcd_state_lock);

      bool all_posted = true;
      for (int ch = 0; ch < POT_CH_COUNT; ch++) {
        if (cur_clock_mode && ch == POT_CH_SP) {
          pot_state.last_applied_norm[ch] = norms[ch];
          continue;
        }
        if (!force_sync && ch != edit_ch) {
          pot_state.last_applied_norm[ch] = norms[ch];
          continue;
        }
        if (force_sync || fabsf(norms[ch] - pot_state.last_applied_norm[ch]) >=
                              POT_DEADBAND_NORM) {
          if (post_motor_event(pot_events[ch], mapped_vals[ch], forward)) {
            pot_state.last_applied_norm[ch] = norms[ch];
            if (!force_sync && ch == edit_ch) {
              taskENTER_CRITICAL(&lcd_state_lock);
              ui_last_activity_ms = now_ms;
              taskEXIT_CRITICAL(&lcd_state_lock);
            }
          } else {
            all_posted = false;
          }
        }
      }
      if (all_posted && force_sync) {
        pot_force_sync = false;
      }
    } else if (cur_clock_mode) {
      pot_force_sync = false;
    }

    ec11_poll_and_dispatch();
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

static void capture_display_telemetry(display_telemetry_snapshot_t *snap) {
  taskENTER_CRITICAL(&config_state_lock);
  snap->state = motor_state;
  snap->fault_reason = motor_fault_reason;
  snap->forward = forward;
  snap->position_mode = position_mode;
  snap->open_loop_mode = open_loop_mode;
  snap->clock_mode = clock_mode;
  snap->target_rpm = target_rpm;
  snap->target_position_counts = target_position_counts;
  snap->measured_rpm = live_measured_rpm;
  snap->encoder_position_counts = encoder_position_counts;
  snap->clock_step_count = clock_step_count;
  snap->applied_duty = live_applied_duty;
  snap->manual_duty = manual_duty;
  snap->active_pid =
      snap->clock_mode
          ? clock_pid_config
          : (snap->position_mode ? position_pid_config : speed_pid_config);
  snap->counts_per_rev =
      (float)encoder_cycles_per_channel * (float)QUADRATURE_FACTOR;
  snap->pot_enabled = pot_enabled;
  snap->nvs_ready = nvs_storage_ready;
  snap->nvs_dirty = nvs_config_dirty;
  taskEXIT_CRITICAL(&config_state_lock);
}

static void lcd_bringup_log_cb(tc802a_bringup_stage_t stage, void *user_ctx) {
  (void)user_ctx;
  switch (stage) {
  case TC802A_BRINGUP_STAGE_INIT_DONE:
    post_log_event("LCD STAGE1 INIT TC802A 8x2 (4-BIT MODE)");
    break;
  case TC802A_BRINGUP_STAGE_ALL_BLOCKS_ON:
    post_log_event("LCD STAGE2 ALL_BLOCKS_ON (0xFF) 2000ms");
    break;
  case TC802A_BRINGUP_STAGE_PATTERN_DRAWN:
    post_log_event("LCD STAGE3 PATTERN 8x2 (PIDFRA82 / 12345678)");
    break;
  }
}

static char lcd_sanitize_ascii_char(char ch) {
  uint8_t uc = (uint8_t)ch;
  if (uc == 0xDFU) {
    return 'd';
  }
  if (uc == 0xFFU) {
    return '#';
  }
  if (uc >= 0x20U && uc <= 0x7EU) {
    return ch;
  }
  return '?';
}

static void format_lcd_sp_line(char *buf, size_t buf_size,
                               const display_telemetry_snapshot_t *snap) {
  if (snap->clock_mode) {
    snprintf(buf, buf_size, "SP:S%02u/m",
             (unsigned)(snap->clock_step_count % 60U));
  } else if (snap->open_loop_mode) {
    snprintf(buf, buf_size, "SP:D %3u", (unsigned)snap->manual_duty);
  } else if (snap->position_mode) {
    float ref_angle_deg =
        (snap->counts_per_rev > 0.0f)
            ? (snap->target_position_counts * 360.0f / snap->counts_per_rev)
            : 0.0f;
    if (fabsf(ref_angle_deg) >= 1000.0f || ref_angle_deg <= -100.0f) {
      snprintf(buf, buf_size, "SP:%5.0f", (double)ref_angle_deg);
    } else {
      snprintf(buf, buf_size, "SP:%4.0f\xDF", (double)ref_angle_deg);
    }
  } else {
    float rpm_clamped = fmaxf(snap->target_rpm, 0.0f);
    if (rpm_clamped >= 10000.0f) {
      snprintf(buf, buf_size, "SP:%5.0f", (double)rpm_clamped);
    } else {
      snprintf(buf, buf_size, "SP:%4.0fr", (double)rpm_clamped);
    }
  }
}

static void render_lcd_home(tc802a_dev_t *dev,
                            const display_telemetry_snapshot_t *snap) {
  char line0[32];
  char line1[32];

  if (snap->state == FSM_FAULT) {
    if (snap->fault_reason == FAULT_SETPOINT_TIMEOUT_REVS) {
      snprintf(line0, sizeof(line0), "!STOP!  ");
      snprintf(line1, sizeof(line1), ">1.5V SP");
    } else {
      snprintf(line0, sizeof(line0), "!FAULT! ");
      snprintf(line1, sizeof(line1), "STALL/ST");
    }
  } else {
    const char *mode_line =
        snap->clock_mode
            ? "M:CLOCK "
            : (snap->position_mode
                   ? "M:POSIC "
                   : (snap->open_loop_mode ? "M:OPEN  " : "M:SPEED "));
    snprintf(line0, sizeof(line0), "%s", mode_line);
    format_lcd_sp_line(line1, sizeof(line1), snap);
  }

  tc802a_write_line(dev, 0U, line0);
  tc802a_write_line(dev, 1U, line1);
}

static void render_lcd_main_menu(tc802a_dev_t *dev, ui_menu_item_t item,
                                 const display_telemetry_snapshot_t *snap) {
  static const char *const item_headers[UI_ITEM_COUNT] = {
      "->MODE  ", "->SP    ", "->KP    ", "->KI    ", "->KD    ",
  };
  char line1[32];

  switch (item) {
  case UI_ITEM_SP:
    format_lcd_sp_line(line1, sizeof(line1), snap);
    break;
  case UI_ITEM_KP:
    snprintf(line1, sizeof(line1), "KP:%5.3f", (double)snap->active_pid.kp);
    break;
  case UI_ITEM_KI:
    snprintf(line1, sizeof(line1), "KI:%5.3f", (double)snap->active_pid.ki);
    break;
  case UI_ITEM_KD:
    snprintf(line1, sizeof(line1), "KD:%5.3f", (double)snap->active_pid.kd);
    break;
  case UI_ITEM_MODE:
  default:
    snprintf(line1, sizeof(line1), "%s",
             snap->clock_mode
                 ? "  CLOCK "
                 : (snap->position_mode
                        ? "  POSIC "
                        : (snap->open_loop_mode ? "  OPEN  " : "  SPEED ")));
    break;
  }

  tc802a_write_line(dev, 0U, item_headers[item % UI_ITEM_COUNT]);
  tc802a_write_line(dev, 1U, line1);
}

static void render_lcd_mode_select(tc802a_dev_t *dev,
                                   ui_selectable_mode_t mode) {
  static const char *const nav_labels[UI_MODE_COUNT] = {
      ">SPEED  ",
      ">POSIC. ",
      ">CLOCK  ",
      ">OPEN   ",
  };
  tc802a_write_line(dev, 0U, "SEL MODO");
  tc802a_write_line(dev, 1U, nav_labels[mode % UI_MODE_COUNT]);
}

static void render_lcd_mode_confirm(tc802a_dev_t *dev,
                                    ui_selectable_mode_t mode) {
  static const char *const confirm_labels[UI_MODE_COUNT] = {
      "SW:SPEED",
      "SW:POSIC",
      "SW:CLOCK",
      "SW:OPEN ",
  };
  tc802a_write_line(dev, 0U, "CONFIRM?");
  tc802a_write_line(dev, 1U, confirm_labels[mode % UI_MODE_COUNT]);
}

static void render_lcd_edit_param(tc802a_dev_t *dev, ui_menu_item_t item,
                                  const display_telemetry_snapshot_t *snap) {
  char line0[32];
  char line1[32];

  switch (item) {
  case UI_ITEM_KP:
    snprintf(line0, sizeof(line0), "EDIT KP ");
    snprintf(line1, sizeof(line1), "KP:%5.3f", (double)snap->active_pid.kp);
    break;
  case UI_ITEM_KI:
    snprintf(line0, sizeof(line0), "EDIT KI ");
    snprintf(line1, sizeof(line1), "KI:%5.3f", (double)snap->active_pid.ki);
    break;
  case UI_ITEM_KD:
    snprintf(line0, sizeof(line0), "EDIT KD ");
    snprintf(line1, sizeof(line1), "KD:%5.3f", (double)snap->active_pid.kd);
    break;
  case UI_ITEM_SP:
  default:
    snprintf(line0, sizeof(line0), "EDIT SP ");
    format_lcd_sp_line(line1, sizeof(line1), snap);
    break;
  }

  tc802a_write_line(dev, 0U, line0);
  tc802a_write_line(dev, 1U, line1);
}

static void render_lcd_pid(tc802a_dev_t *dev,
                           const display_telemetry_snapshot_t *snap) {
  char line0[32];
  char line1[32];
  char mode_ch =
      (snap->state == FSM_FAULT)
          ? 'F'
          : (snap->clock_mode
                 ? 'C'
                 : (snap->position_mode ? 'P'
                                        : (snap->open_loop_mode ? 'O' : 'V')));
  snprintf(line0, sizeof(line0), "P%.1fI%.1f", (double)snap->active_pid.kp,
           (double)snap->active_pid.ki);
  snprintf(line1, sizeof(line1), "D%.3f %c", (double)snap->active_pid.kd,
           mode_ch);
  tc802a_write_line(dev, 0U, line0);
  tc802a_write_line(dev, 1U, line1);
}

static void sync_lcd_status_snapshot(const tc802a_dev_t *dev,
                                     lcd_view_mode_t view_mode,
                                     uint32_t frames_rendered) {
  taskENTER_CRITICAL(&lcd_state_lock);
  lcd_status_snapshot.initialized = dev->initialized;
  lcd_status_snapshot.view_mode = view_mode;
  lcd_status_snapshot.display_on = dev->display_on;
  lcd_status_snapshot.cursor_on = dev->cursor_on;
  lcd_status_snapshot.blink_on = dev->blink_on;
  lcd_status_snapshot.all_blocks_on = dev->all_blocks_on;
  lcd_status_snapshot.frames_rendered = frames_rendered;
  for (size_t i = 0U; i < TC802A_COLS; i++) {
    lcd_status_snapshot.line0[i] = lcd_sanitize_ascii_char(dev->lines[0][i]);
    lcd_status_snapshot.line1[i] = lcd_sanitize_ascii_char(dev->lines[1][i]);
  }
  lcd_status_snapshot.line0[TC802A_COLS] = '\0';
  lcd_status_snapshot.line1[TC802A_COLS] = '\0';
  taskEXIT_CRITICAL(&lcd_state_lock);
}

static void display_task(void *arg) {
  static tc802a_dev_t lcd_dev;
  tc802a_espidf_hal_ctx_t hal_ctx;
  tc802a_hal_t hal;
  (void)arg;

  tc802a_espidf_gpio_config_t lcd_gpio_cfg = tc802a_hal_espidf_default_config();
  lcd_gpio_cfg.rs_gpio = PIN_LCD_RS;
  lcd_gpio_cfg.e_gpio = PIN_LCD_E;
  lcd_gpio_cfg.d4_gpio = PIN_LCD_D4;
  lcd_gpio_cfg.d5_gpio = PIN_LCD_D5;
  lcd_gpio_cfg.d6_gpio = PIN_LCD_D6;
  lcd_gpio_cfg.d7_gpio = PIN_LCD_D7;

  if (tc802a_hal_espidf_init(&hal_ctx, &hal, &lcd_gpio_cfg) != TC802A_OK) {
    post_log_event("LCD HAL INIT ERR");
    vTaskDelete(NULL);
    return;
  }

  char bus_msg[112];
  snprintf(bus_msg, sizeof(bus_msg),
           "LCD 8x2 RS=%d E=%d D4=%d D5=%d D6=%d D7=%d", lcd_gpio_cfg.rs_gpio,
           lcd_gpio_cfg.e_gpio, lcd_gpio_cfg.d4_gpio, lcd_gpio_cfg.d5_gpio,
           lcd_gpio_cfg.d6_gpio, lcd_gpio_cfg.d7_gpio);
  post_log_event(bus_msg);

  if (tc802a_init(&lcd_dev, &hal) != TC802A_OK) {
    post_log_event("LCD TC802A INIT ERR");
    tc802a_hal_espidf_deinit(&hal_ctx);
    vTaskDelete(NULL);
    return;
  }

  lcd_view_mode_t view_mode = LCD_VIEW_BRINGUP_PATTERN;
  uint32_t frames_rendered = 0U;
  bool auto_switch_to_dash = true;
  bool all_blocks_override = false;
  bool cleared_hold = false;
  uint32_t pattern_hold_ms = 0U;

  if (tc802a_run_bringup(&lcd_dev, LCD_BRINGUP_ALL_ON_MS, lcd_bringup_log_cb,
                         &lcd_dev) == TC802A_OK) {
    frames_rendered++;
  } else {
    post_log_event("LCD BRINGUP ERR");
  }
  sync_lcd_status_snapshot(&lcd_dev, view_mode, frames_rendered);

  const TickType_t refresh_ticks = pdMS_TO_TICKS(LCD_REFRESH_PERIOD_MS);
  while (true) {
    lcd_request_t req;
    while (xQueueReceive(lcd_request_queue, &req, 0) == pdTRUE) {
      switch (req.type) {
      case LCD_REQ_BRINGUP:
        auto_switch_to_dash = true;
        all_blocks_override = false;
        cleared_hold = false;
        pattern_hold_ms = 0U;
        view_mode = LCD_VIEW_BRINGUP_PATTERN;
        if (tc802a_run_bringup(&lcd_dev, LCD_BRINGUP_ALL_ON_MS,
                               lcd_bringup_log_cb, &lcd_dev) == TC802A_OK) {
          frames_rendered++;
        } else {
          post_log_event("LCD BRINGUP ERR");
        }
        break;
      case LCD_REQ_SET_VIEW_PATTERN:
        auto_switch_to_dash = false;
        all_blocks_override = false;
        cleared_hold = false;
        lcd_dev.all_blocks_on = false;
        view_mode = LCD_VIEW_BRINGUP_PATTERN;
        tc802a_write_line(&lcd_dev, 0U, "PIDFRA82");
        tc802a_write_line(&lcd_dev, 1U, "12345678");
        if (tc802a_flush(&lcd_dev) == TC802A_OK) {
          frames_rendered++;
        }
        post_log_event("LCD VIEW PATTERN");
        break;
      case LCD_REQ_SET_VIEW_DASH:
        auto_switch_to_dash = false;
        all_blocks_override = false;
        cleared_hold = false;
        lcd_dev.all_blocks_on = false;
        view_mode = LCD_VIEW_DASHBOARD;
        post_log_event("LCD VIEW DASHBOARD");
        break;
      case LCD_REQ_SET_VIEW_PID:
        auto_switch_to_dash = false;
        all_blocks_override = false;
        cleared_hold = false;
        lcd_dev.all_blocks_on = false;
        view_mode = LCD_VIEW_PID;
        post_log_event("LCD VIEW PID");
        break;
      case LCD_REQ_SET_ALL_ON:
        all_blocks_override = req.flag;
        cleared_hold = false;
        lcd_dev.all_blocks_on = req.flag;
        if (req.flag) {
          tc802a_fill_buffer(&lcd_dev, (char)0xFF);
          if (tc802a_flush(&lcd_dev) == TC802A_OK) {
            frames_rendered++;
          }
        } else if (view_mode == LCD_VIEW_BRINGUP_PATTERN) {
          tc802a_write_line(&lcd_dev, 0U, "PIDFRA82");
          tc802a_write_line(&lcd_dev, 1U, "12345678");
          if (tc802a_flush(&lcd_dev) == TC802A_OK) {
            frames_rendered++;
          }
        }
        break;
      case LCD_REQ_SET_CURSOR:
        tc802a_set_display_control(&lcd_dev, lcd_dev.display_on, req.flag,
                                   lcd_dev.blink_on);
        break;
      case LCD_REQ_SET_BLINK:
        tc802a_set_display_control(&lcd_dev, lcd_dev.display_on,
                                   lcd_dev.cursor_on, req.flag);
        break;
      case LCD_REQ_SET_DISPLAY:
        tc802a_set_display_control(&lcd_dev, req.flag, lcd_dev.cursor_on,
                                   lcd_dev.blink_on);
        break;
      case LCD_REQ_CLEAR:
        auto_switch_to_dash = false;
        all_blocks_override = false;
        cleared_hold = true;
        lcd_dev.all_blocks_on = false;
        if (tc802a_clear_hw(&lcd_dev) == TC802A_OK) {
          frames_rendered++;
        }
        break;
      }
    }

    if (!all_blocks_override && !cleared_hold &&
        view_mode == LCD_VIEW_BRINGUP_PATTERN && auto_switch_to_dash) {
      pattern_hold_ms += LCD_REFRESH_PERIOD_MS;
      if (pattern_hold_ms >= LCD_BRINGUP_PATTERN_HOLD_MS) {
        auto_switch_to_dash = false;
        view_mode = LCD_VIEW_DASHBOARD;
        post_log_event("LCD HOME VIEW ACTIVE");
      }
    }

    ui_menu_state_t menu_state = UI_STATE_HOME;
    ui_menu_item_t menu_item = UI_ITEM_MODE;
    ui_selectable_mode_t candidate_mode = UI_MODE_SPEED;

    taskENTER_CRITICAL(&lcd_state_lock);
    menu_state = ui_menu_state;
    menu_item = ui_selected_item;
    candidate_mode = ui_candidate_mode;
    taskEXIT_CRITICAL(&lcd_state_lock);

    if (!all_blocks_override && !cleared_hold) {
      display_telemetry_snapshot_t snap;
      capture_display_telemetry(&snap);
      if (menu_state == UI_STATE_MAIN_MENU) {
        render_lcd_main_menu(&lcd_dev, menu_item, &snap);
        if (tc802a_flush(&lcd_dev) == TC802A_OK) {
          frames_rendered++;
        }
      } else if (menu_state == UI_STATE_MODE_SELECT) {
        render_lcd_mode_select(&lcd_dev, candidate_mode);
        if (tc802a_flush(&lcd_dev) == TC802A_OK) {
          frames_rendered++;
        }
      } else if (menu_state == UI_STATE_MODE_CONFIRM) {
        render_lcd_mode_confirm(&lcd_dev, candidate_mode);
        if (tc802a_flush(&lcd_dev) == TC802A_OK) {
          frames_rendered++;
        }
      } else if (menu_state == UI_STATE_EDIT_PARAM) {
        render_lcd_edit_param(&lcd_dev, menu_item, &snap);
        if (tc802a_flush(&lcd_dev) == TC802A_OK) {
          frames_rendered++;
        }
      } else if (snap.state == FSM_FAULT || view_mode == LCD_VIEW_DASHBOARD ||
                 view_mode == LCD_VIEW_PID) {
        if (view_mode == LCD_VIEW_PID && snap.state != FSM_FAULT) {
          render_lcd_pid(&lcd_dev, &snap);
        } else {
          render_lcd_home(&lcd_dev, &snap);
        }
        if (tc802a_flush(&lcd_dev) == TC802A_OK) {
          frames_rendered++;
        }
      }
    }

    sync_lcd_status_snapshot(&lcd_dev, view_mode, frames_rendered);
    vTaskDelay(refresh_ticks);
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

  gpio_set_level(IN1_GPIO, 0);
  gpio_set_level(IN2_GPIO, 0);
  ESP_ERROR_CHECK(gpio_config(&motor_config));
  ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
  ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  encoder_init();
  ec11_init();
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
  lcd_request_queue = xQueueCreate(LCD_QUEUE_LENGTH, sizeof(lcd_request_t));
  ESP_ERROR_CHECK(lcd_request_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  nvs_storage_init_and_load();
  xTaskCreate(serial_task, "serial_task", 3072, NULL, 5, NULL);
  xTaskCreate(control_task, "control_task", 3072, NULL, 6, NULL);
  xTaskCreate(potentiometer_task, "pot_task", POT_TASK_STACK_SIZE, NULL,
              POT_TASK_PRIORITY, NULL);
  xTaskCreate(nvs_task, "nvs_task", NVS_TASK_STACK_SIZE, NULL,
              NVS_TASK_PRIORITY, NULL);
  xTaskCreate(display_task, "display_task", LCD_TASK_STACK_SIZE, NULL,
              LCD_TASK_PRIORITY, NULL);
  log_message_t startup = {.kind = LOG_EVENT};
  snprintf(startup.text, sizeof(startup.text), "READY A=%d B=%d C=%d Q=%d",
           ENCODER_A_GPIO, ENCODER_B_GPIO, (int)encoder_cycles_per_channel,
           QUADRATURE_FACTOR);
  xQueueSend(log_queue, &startup, portMAX_DELAY);
  log_message_t help = {.kind = LOG_EVENT};
  snprintf(help.text, sizeof(help.text), "HELP");
  xQueueSend(log_queue, &help, portMAX_DELAY);
}
