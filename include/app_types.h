#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

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
  float min_val;
  float max_val;
} pot_range_t;

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
  char line0[APP_LCD_COLS + 1U];
  char line1[APP_LCD_COLS + 1U];
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

typedef enum {
  LOG_EVENT = 0,
  LOG_RAW_LINE,
  LOG_HELP,
  LOG_TELEMETRY,
  LOG_POSITION,
  LOG_POS_OUT,
  LOG_PULSE_DIAG,
} log_kind_t;

typedef struct {
  log_kind_t kind;
  union {
    char text[PRINT_LINE_MAX_LEN];
    struct {
      motor_state_t state;
      bool clock_mode;
      bool position_mode;
      uint8_t duty;
      float reference;
      float measured_rpm;
      float measured_angle;
    } telemetry;
    struct {
      int64_t counts;
      float angle_deg;
    } position;
    struct {
      float output;
      float error;
      bool sp_reached;
    } pos_out;
    struct {
      bool is_ton_to_motion;
      uint32_t elapsed_ms;
      int32_t counts;
    } pulse_diag;
  } data;
} log_message_t;
