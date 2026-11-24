#include "serial_cli.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "lcd_ui.h"
#include "motor_hw.h"
#include "serial_print.h"
#include "ui_ec11.h"

static TickType_t cli_print_timeout_ticks(void) {
  return pdMS_TO_TICKS(PRINT_CLI_SEND_TIMEOUT_MS);
}

static void str_to_upper(char *s) {
  for (char *p = s; *p != '\0'; p++) {
    *p = (char)toupper((unsigned char)*p);
  }
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

static void process_lcd_command(app_context_t *app, const char *value_text) {
  char subcmd[16] = {0};
  char arg[16] = {0};
  int parsed = sscanf(value_text, "%15s %15s", subcmd, arg);
  if (parsed >= 1) {
    str_to_upper(subcmd);
  }
  if (parsed >= 2) {
    str_to_upper(arg);
  }

  if (parsed <= 0 || strcmp(subcmd, "STATUS") == 0 ||
      strcmp(subcmd, "GET") == 0) {
    lcd_status_snapshot_t snap;
    taskENTER_CRITICAL(&app->lcd_state_lock);
    snap = app->lcd_status_snapshot;
    taskEXIT_CRITICAL(&app->lcd_state_lock);
    serial_print_fmt(
        app->print_queue, cli_print_timeout_ticks(),
        "LCD INIT=%d TYPE=8x2 VIEW=%s DISP=%d CUR=%d BLINK=%d ALLON=%d "
        "FRAMES=%" PRIu32
        " RS=%d E=%d D4=%d D5=%d D6=%d D7=%d L0=\"%s\" L1=\"%s\"\r\n",
        snap.initialized ? 1 : 0, lcd_ui_view_mode_name(snap.view_mode),
        snap.display_on ? 1 : 0, snap.cursor_on ? 1 : 0, snap.blink_on ? 1 : 0,
        snap.all_blocks_on ? 1 : 0, snap.frames_rendered, PIN_LCD_RS, PIN_LCD_E,
        PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7, snap.line0, snap.line1);
    return;
  }
  if (strcmp(subcmd, "BRINGUP") == 0 || strcmp(subcmd, "TEST") == 0) {
    serial_cli_emit_event(
        app, app_post_lcd_request(app, LCD_REQ_BRINGUP, false) ? "OK LCD BRINGUP"
                                                               : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "PATTERN") == 0) {
    serial_cli_emit_event(
        app, app_post_lcd_request(app, LCD_REQ_SET_VIEW_PATTERN, false)
                 ? "OK LCD PATTERN"
                 : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "DASH") == 0 || strcmp(subcmd, "DASHBOARD") == 0) {
    serial_cli_emit_event(
        app, app_post_lcd_request(app, LCD_REQ_SET_VIEW_DASH, false)
                 ? "OK LCD DASH"
                 : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "PID") == 0) {
    serial_cli_emit_event(
        app, app_post_lcd_request(app, LCD_REQ_SET_VIEW_PID, false)
                 ? "OK LCD PID"
                 : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "CLEAR") == 0 || strcmp(subcmd, "CLS") == 0) {
    serial_cli_emit_event(
        app, app_post_lcd_request(app, LCD_REQ_CLEAR, false) ? "OK LCD CLEAR"
                                                             : "ERR LCD");
    return;
  }
  if (strcmp(subcmd, "ALLON") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      serial_cli_emit_event(app, "ERR LCD ALLON");
      return;
    }
    if (app_post_lcd_request(app, LCD_REQ_SET_ALL_ON, enabled)) {
      serial_cli_emit_event(app,
                            enabled ? "OK LCD ALLON ON" : "OK LCD ALLON OFF");
    } else {
      serial_cli_emit_event(app, "ERR LCD");
    }
    return;
  }
  if (strcmp(subcmd, "CURSOR") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      serial_cli_emit_event(app, "ERR LCD CURSOR");
      return;
    }
    if (app_post_lcd_request(app, LCD_REQ_SET_CURSOR, enabled)) {
      serial_cli_emit_event(app,
                            enabled ? "OK LCD CURSOR ON" : "OK LCD CURSOR OFF");
    } else {
      serial_cli_emit_event(app, "ERR LCD");
    }
    return;
  }
  if (strcmp(subcmd, "BLINK") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      serial_cli_emit_event(app, "ERR LCD BLINK");
      return;
    }
    if (app_post_lcd_request(app, LCD_REQ_SET_BLINK, enabled)) {
      serial_cli_emit_event(app,
                            enabled ? "OK LCD BLINK ON" : "OK LCD BLINK OFF");
    } else {
      serial_cli_emit_event(app, "ERR LCD");
    }
    return;
  }
  if (strcmp(subcmd, "DISPLAY") == 0 && parsed >= 2) {
    bool enabled = false;
    if (!parse_on_off_arg(arg, &enabled)) {
      serial_cli_emit_event(app, "ERR LCD DISPLAY");
      return;
    }
    if (app_post_lcd_request(app, LCD_REQ_SET_DISPLAY, enabled)) {
      serial_cli_emit_event(
          app, enabled ? "OK LCD DISPLAY ON" : "OK LCD DISPLAY OFF");
    } else {
      serial_cli_emit_event(app, "ERR LCD");
    }
    return;
  }
  serial_cli_emit_event(app, "ERR LCD");
}

static void process_ec11_command(app_context_t *app, const char *value_text) {
  char subcmd[16] = {0};
  if (sscanf(value_text, "%15s", subcmd) == 1) {
    str_to_upper(subcmd);
  }
  if (subcmd[0] == '\0' || strcmp(subcmd, "GET") == 0 ||
      strcmp(subcmd, "STATUS") == 0 || strcmp(subcmd, "READ") == 0) {
    int64_t counts = 0;
    taskENTER_CRITICAL(&app->config_state_lock);
    counts = app->ec11.total_counts;
    taskEXIT_CRITICAL(&app->config_state_lock);
    int sw_pressed = (gpio_get_level(EC11_SW_GPIO) == 0) ? 1 : 0;
    serial_print_fmt(app->print_queue, cli_print_timeout_ticks(),
                     "EC11 EN=%d CNT=%" PRId64 " DET=%" PRId64
                     " SW=%d CLK=%d DT=%d SW_PIN=%d\r\n",
                     app->ec11.enabled ? 1 : 0, counts,
                     counts / (int64_t)EC11_COUNTS_PER_DETENT, sw_pressed,
                     PIN_EC11_CLK, PIN_EC11_DT, PIN_EC11_SW);
    return;
  }
  if (strcmp(subcmd, "ON") == 0) {
    app->ec11.enabled = true;
    serial_cli_emit_event(app, "OK EC11 ON");
    return;
  }
  if (strcmp(subcmd, "OFF") == 0) {
    app->ec11.enabled = false;
    serial_cli_emit_event(app, "OK EC11 OFF");
    return;
  }
  if (strcmp(subcmd, "ZERO") == 0 || strcmp(subcmd, "RESET") == 0) {
    ui_ec11_zero(&app->ec11, &app->config_state_lock);
    serial_cli_emit_event(app, "OK EC11 ZERO");
    return;
  }
  serial_cli_emit_event(app, "ERR EC11");
}

void serial_cli_uart_init(void) {
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE * 2,
                                      UART_BUFFER_SIZE * 4, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
}

void serial_cli_emit_event(app_context_t *app, const char *text) {
  if (app == NULL || text == NULL) {
    return;
  }
  serial_print_fmt(app->print_queue, cli_print_timeout_ticks(), "%s\r\n", text);
}

void serial_cli_print_help(app_context_t *app) {
  if (app == NULL) {
    return;
  }
  serial_print_help(app->print_queue, cli_print_timeout_ticks());
}

void serial_cli_process_command(app_context_t *app, char *command) {
  char *operation = strtok(command, " \t");
  char *value_text = strtok(NULL, "");
  char *end = NULL;
  const TickType_t print_ticks = cli_print_timeout_ticks();

  if (operation == NULL) {
    return;
  }
  str_to_upper(operation);
  if (value_text == NULL) {
    value_text = "";
  }
  while (isspace((unsigned char)*value_text)) {
    value_text++;
  }
  if (strcmp(operation, "STOP") == 0 || strcmp(operation, "S") == 0) {
    bool ok = app_post_motor_event(app, EVENT_STOP, 0.0f, true);
    if (ok) {
      ui_ec11_reset_to_home(&app->ec11, &app->lcd_state_lock);
    }
    serial_cli_emit_event(app, ok ? "OK STOP" : "ERR EVT");
    return;
  }
  if (app->encoder_fault && strcmp(operation, "HELP") != 0 &&
      strcmp(operation, "?") != 0 && strcmp(operation, "STATUS") != 0 &&
      strcmp(operation, "CAL") != 0 && strcmp(operation, "EUR") != 0 &&
      strcmp(operation, "CALIB") != 0 && strcmp(operation, "SAVE") != 0 &&
      strcmp(operation, "LOAD") != 0 && strcmp(operation, "NVS") != 0 &&
      strcmp(operation, "LCD") != 0 && strcmp(operation, "EC11") != 0 &&
      strcmp(operation, "LOG") != 0) {
    serial_cli_emit_event(app, "ERR LOCK STOP");
    return;
  }
  if (strcmp(operation, "HELP") == 0 || strcmp(operation, "?") == 0) {
    serial_cli_print_help(app);
    return;
  }
  if (strcmp(operation, "MODE") == 0 || strcmp(operation, "M") == 0) {
    char mode[12] = {0};
    if (sscanf(value_text, "%11s", mode) != 1) {
      serial_cli_emit_event(app, "ERR MODE");
      return;
    }
    str_to_upper(mode);
    if (strcmp(mode, "SPEED") != 0 && strcmp(mode, "V") != 0 &&
        strcmp(mode, "POS") != 0 && strcmp(mode, "P") != 0 &&
        strcmp(mode, "OPEN") != 0 && strcmp(mode, "O") != 0 &&
        strcmp(mode, "CLOCK") != 0 && strcmp(mode, "CLK") != 0 &&
        strcmp(mode, "C") != 0) {
      serial_cli_emit_event(app, "ERR MODE");
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
    bool ok = app_post_motor_event(app, mode_event, 0.0f, true);
    if (ok) {
      ui_ec11_reset_to_home(&app->ec11, &app->lcd_state_lock);
    }
    serial_cli_emit_event(app, ok ? ok_msg : "ERR EVT");
    return;
  }
  if (strcmp(operation, "CLOCK") == 0 || strcmp(operation, "CLK") == 0) {
    if (value_text[0] != '\0') {
      serial_cli_emit_event(app, "ERR MODE");
      return;
    }
    bool ok = app_post_motor_event(app, EVENT_MODE_CLOCK, 0.0f, true);
    if (ok) {
      ui_ec11_reset_to_home(&app->ec11, &app->lcd_state_lock);
    }
    serial_cli_emit_event(app, ok ? "OK MODE CLOCK" : "ERR EVT");
    return;
  }
  if (strcmp(operation, "POS") == 0 || strcmp(operation, "P") == 0) {
    float position = strtof(value_text, &end);
    while (isspace((unsigned char)*end)) {
      end++;
    }
    if (value_text[0] == '\0' || *end != '\0' || !isfinite(position) ||
        fabsf(position) > MAX_POSITION_COUNTS) {
      serial_cli_emit_event(app, "ERR POS");
      return;
    }
    serial_cli_emit_event(
        app, app_post_motor_event(app, EVENT_POSITION_SETPOINT, position, true)
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
      serial_cli_emit_event(app, "ERR ANG");
      return;
    }
    float target_counts =
        angle *
        motor_hw_counts_per_revolution(app->encoder_cycles_per_channel) /
        360.0f;
    serial_cli_emit_event(
        app,
        app_post_motor_event(app, EVENT_ANGLE_SETPOINT, target_counts, true)
            ? "OK ANG"
            : "ERR EVT");
    return;
  }
  if (strcmp(operation, "STATUS") == 0) {
    motor_state_t st;
    bool run_now = false;
    bool flt_now = false;
    int64_t counts_now = 0;
    uint32_t cycles_now = 0;
    uint8_t duty_now = 0;
    bool pos_mode = false;
    bool clk_mode = false;
    float target_pos = 0.0f;
    float target_spd = 0.0f;
    taskENTER_CRITICAL(&app->config_state_lock);
    st = app->motor_state;
    run_now = app->running;
    flt_now = app->encoder_fault;
    counts_now = app->encoder_position_counts;
    cycles_now = app->encoder_cycles_per_channel;
    duty_now = app->manual_duty;
    pos_mode = app->position_mode;
    clk_mode = app->clock_mode;
    target_pos = app->target_position_counts;
    target_spd = app->target_rpm;
    taskEXIT_CRITICAL(&app->config_state_lock);

    float cpr = motor_hw_counts_per_revolution(cycles_now);
    serial_print_fmt(app->print_queue, print_ticks,
                     "STAT S=%c RUN=%d FLT=%d POS=%.2f REF=%.2f\r\n",
                     motor_control_state_code(st), run_now ? 1 : 0,
                     flt_now ? 1 : 0, (float)counts_now * 360.0f / cpr,
                     st == FSM_OPEN_LOOP
                         ? (float)duty_now
                         : ((pos_mode || clk_mode) ? target_pos * 360.0f / cpr
                                                   : target_spd));
    return;
  }
  if (strcmp(operation, "CAL") == 0 || strcmp(operation, "EUR") == 0 ||
      strcmp(operation, "CALIB") == 0) {
    char subcmd[16] = {0};
    if (sscanf(value_text, "%15s", subcmd) == 1) {
      str_to_upper(subcmd);
    }
    if (strcmp(subcmd, "START") == 0 || strcmp(subcmd, "ZERO") == 0 ||
        strcmp(subcmd, "RESET") == 0) {
      app_post_motor_event(app, EVENT_CALIBRATE_START, 0.0f, true);
      serial_cli_emit_event(app,
                            "OK CAL START (MOTOR RELEASED, ENCODER ZEROED)");
      return;
    }
    if (strcmp(subcmd, "READ") == 0 || strcmp(subcmd, "GET") == 0 ||
        strcmp(subcmd, "VAL") == 0 || subcmd[0] == '\0') {
      int64_t counts_now = 0;
      uint32_t cycles_now = 0;
      taskENTER_CRITICAL(&app->config_state_lock);
      counts_now = app->encoder_position_counts;
      cycles_now = app->encoder_cycles_per_channel;
      taskEXIT_CRITICAL(&app->config_state_lock);
      float cpr = motor_hw_counts_per_revolution(cycles_now);
      float deg = (cpr > 0.0f) ? ((float)counts_now * 360.0f / cpr) : 0.0f;
      float rev = (cpr > 0.0f) ? ((float)counts_now / cpr) : 0.0f;
      serial_print_fmt(app->print_queue, print_ticks,
                       "CAL COUNTS=%" PRId64 " DEG=%.2f REV=%.4f\r\n",
                       counts_now, deg, rev);
      return;
    }
    if (strcmp(subcmd, "STOP") == 0 || strcmp(subcmd, "END") == 0) {
      int64_t counts_now = 0;
      uint32_t cycles_now = 0;
      taskENTER_CRITICAL(&app->config_state_lock);
      counts_now = app->encoder_position_counts;
      cycles_now = app->encoder_cycles_per_channel;
      taskEXIT_CRITICAL(&app->config_state_lock);
      float cpr = motor_hw_counts_per_revolution(cycles_now);
      float deg = (cpr > 0.0f) ? ((float)counts_now * 360.0f / cpr) : 0.0f;
      float rev = (cpr > 0.0f) ? ((float)counts_now / cpr) : 0.0f;
      serial_print_fmt(app->print_queue, print_ticks,
                       "OK CAL STOP COUNTS=%" PRId64 " DEG=%.2f REV=%.4f\r\n",
                       counts_now, deg, rev);
      return;
    }
    serial_cli_emit_event(app, "ERR CAL (USE: CAL START | CAL READ | CAL STOP)");
    return;
  }
  if (strcmp(operation, "PID") == 0) {
    char selector[12] = {0};
    char param[12] = {0};
    float new_value = 0.0f;
    if (sscanf(value_text, "%11s %11s %f", selector, param, &new_value) == 3) {
      str_to_upper(selector);
      str_to_upper(param);
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
          new_value < 0.0f || new_value > MAX_PID_GAIN) {
        serial_cli_emit_event(app, "ERR PID");
        return;
      }
      taskENTER_CRITICAL(&app->config_state_lock);
      pid_config_t *target_cfg =
          target_speed ? &app->speed_pid_config
                       : (target_position ? &app->position_pid_config
                                          : &app->clock_pid_config);
      if (set_kp) {
        target_cfg->kp = new_value;
      } else if (set_ki) {
        target_cfg->ki = new_value;
      } else {
        target_cfg->kd = new_value;
      }
      if (target_speed) {
        motor_control_apply_speed_pid(app);
      } else if (target_position) {
        motor_control_apply_position_pid(app);
      } else {
        motor_control_apply_clock_pid(app);
      }
      taskEXIT_CRITICAL(&app->config_state_lock);
      app_notify_config_modified(app);
      serial_print_fmt(app->print_queue, print_ticks,
                       "OK PID %s %s %.4f\r\n", selector, param, new_value);
      return;
    }
    if (sscanf(value_text, "%11s %11s", selector, param) == 2) {
      str_to_upper(selector);
      str_to_upper(param);
      if (strcmp(selector, "GET") == 0) {
        bool speed = strcmp(param, "SPEED") == 0 || strcmp(param, "V") == 0;
        bool position = strcmp(param, "POS") == 0 || strcmp(param, "P") == 0;
        bool clock = strcmp(param, "CLOCK") == 0 || strcmp(param, "CLK") == 0 ||
                     strcmp(param, "C") == 0;
        if (speed || position || clock) {
          pid_config_t speed_cfg;
          pid_config_t pos_cfg;
          pid_config_t clock_cfg;
          taskENTER_CRITICAL(&app->config_state_lock);
          speed_cfg = app->speed_pid_config;
          pos_cfg = app->position_pid_config;
          clock_cfg = app->clock_pid_config;
          taskEXIT_CRITICAL(&app->config_state_lock);
          if (speed) {
            serial_print_fmt(app->print_queue, print_ticks,
                             "PID V KP=%.4f KI=%.4f KD=%.4f\r\n", speed_cfg.kp,
                             speed_cfg.ki, speed_cfg.kd);
          } else if (position) {
            serial_print_fmt(app->print_queue, print_ticks,
                             "PID P KP=%.4f KI=%.4f KD=%.4f\r\n", pos_cfg.kp,
                             pos_cfg.ki, pos_cfg.kd);
          } else {
            serial_print_fmt(app->print_queue, print_ticks,
                             "PID C KP=%.4f KI=%.4f KD=%.4f\r\n", clock_cfg.kp,
                             clock_cfg.ki, clock_cfg.kd);
          }
          return;
        }
        serial_cli_emit_event(app, "ERR PID");
        return;
      }
    }
    if (sscanf(value_text, "%11s", selector) == 1) {
      str_to_upper(selector);
      if (strcmp(selector, "DEFAULT") == 0) {
        motor_control_reset_pids_to_defaults(app);
        app_notify_config_modified(app);
        serial_cli_emit_event(app, "OK PID DEFAULT");
        return;
      }
    } else {
      selector[0] = '\0';
    }
    if (strcmp(selector, "GET") == 0 || value_text[0] == '\0') {
      pid_config_t speed_cfg;
      pid_config_t pos_cfg;
      pid_config_t clock_cfg;
      taskENTER_CRITICAL(&app->config_state_lock);
      speed_cfg = app->speed_pid_config;
      pos_cfg = app->position_pid_config;
      clock_cfg = app->clock_pid_config;
      taskEXIT_CRITICAL(&app->config_state_lock);
      serial_print_fmt(app->print_queue, print_ticks,
                       "PID V KP=%.4f KI=%.4f KD=%.4f\r\n", speed_cfg.kp,
                       speed_cfg.ki, speed_cfg.kd);
      serial_print_fmt(app->print_queue, print_ticks,
                       "PID P KP=%.4f KI=%.4f KD=%.4f\r\n", pos_cfg.kp,
                       pos_cfg.ki, pos_cfg.kd);
      serial_print_fmt(app->print_queue, print_ticks,
                       "PID C KP=%.4f KI=%.4f KD=%.4f\r\n", clock_cfg.kp,
                       clock_cfg.ki, clock_cfg.kd);
      return;
    }
    serial_cli_emit_event(app, "ERR PID");
    return;
  }
  if (strcmp(operation, "MINMOVE") == 0 ||
      strcmp(operation, "MIN_MOVE_DUTY") == 0) {
    unsigned min_duty = 0;
    if (sscanf(value_text, "%u", &min_duty) != 1 || min_duty > PWM_MAX_DUTY) {
      serial_cli_emit_event(app, "ERR MINMOVE");
      return;
    }
    taskENTER_CRITICAL(&app->config_state_lock);
    app->min_move_duty = (uint8_t)min_duty;
    taskEXIT_CRITICAL(&app->config_state_lock);
    app_notify_config_modified(app);
    serial_print_fmt(app->print_queue, print_ticks, "OK MINMOVE %u\r\n",
                     min_duty);
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
            serial_cli_emit_event(app, "ERR PULSE");
            return;
          }
        } else if (sscanf(value_text, " %c", &direction) == 1 &&
                   sscanf(second, "%lu", &cycles_tmp) == 1 &&
                   sscanf(third + 1, "%lu", &duty_tmp) == 1) {
          cycles = (uint32_t)cycles_tmp;
          duty = (unsigned)duty_tmp;
        } else {
          serial_cli_emit_event(app, "ERR PULSE");
          return;
        }
      } else {
        serial_cli_emit_event(app, "ERR PULSE");
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
        serial_cli_emit_event(app, "ERR PULSE");
        return;
      }
    }
    direction = (char)toupper((unsigned char)direction);
    if ((direction != 'F' && direction != 'R') || cycles == 0U ||
        duty > PWM_MAX_DUTY || pwm_frequency_hz < PULSE_PWM_MIN_FREQ_HZ ||
        pwm_frequency_hz > PULSE_PWM_MAX_FREQ_HZ) {
      serial_cli_emit_event(app, "ERR PULSE");
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
    serial_print_fmt(app->print_queue, print_ticks,
                     "DBG PULSE DIR=%c CYCLES=%" PRIu32 " DUTY=%u FREQ=%" PRIu32
                     "Hz EST_MS=%" PRIu32 "\r\n",
                     direction, cycles, duty, pwm_frequency_hz,
                     estimated_pulse_ms);
    motor_control_run_pulse(app, pulse_forward, cycles, (uint8_t)duty,
                            pwm_frequency_hz);
    serial_cli_emit_event(app, "OK PULSE");
    return;
  }
  if (strcmp(operation, "DUTY") == 0 || strcmp(operation, "PWM") == 0 ||
      strcmp(operation, "OPEN") == 0 || strcmp(operation, "MANUAL") == 0) {
    char direction = 'D';
    if (isalpha((unsigned char)value_text[0])) {
      direction = (char)toupper((unsigned char)value_text[0]);
      if ((direction != 'F' && direction != 'R' && direction != 'D') ||
          (value_text[1] != '\0' && !isspace((unsigned char)value_text[1]))) {
        serial_cli_emit_event(app, "ERR DUTY");
        return;
      }
      value_text++;
      while (isspace((unsigned char)*value_text)) {
        value_text++;
      }
    }
    float duty = strtof(value_text, &end);
    while (isspace((unsigned char)*end)) {
      end++;
    }
    if (value_text[0] == '\0' || *end != '\0' || !isfinite(duty) ||
        duty < 0.0f || duty > (float)PWM_MAX_DUTY) {
      serial_cli_emit_event(app, "ERR DUTY");
      return;
    }
    bool next_forward = true;
    if (direction == 'F') {
      next_forward = true;
    } else if (direction == 'R') {
      next_forward = false;
    } else {
      taskENTER_CRITICAL(&app->config_state_lock);
      next_forward = app->forward;
      taskEXIT_CRITICAL(&app->config_state_lock);
    }
    serial_cli_emit_event(
        app,
        app_post_motor_event(app, EVENT_DUTY_SETPOINT, duty, next_forward)
            ? "OK DUTY"
            : "ERR EVT");
    return;
  }
  if (strcmp(operation, "LOG") == 0) {
    char option[12] = {0};
    char arg[16] = {0};
    int parsed = sscanf(value_text, "%11s %15s", option, arg);
    if (parsed < 1) {
      serial_cli_emit_event(app, "ERR LOG");
      return;
    }
    str_to_upper(option);
    if (parsed >= 2) {
      str_to_upper(arg);
    }
    if (strcmp(option, "ON") == 0 && parsed == 1) {
      taskENTER_CRITICAL(&app->config_state_lock);
      app->telemetry_enabled = true;
      taskEXIT_CRITICAL(&app->config_state_lock);
      app_notify_config_modified(app);
      serial_cli_emit_event(app, "OK LOG ON");
    } else if (strcmp(option, "OFF") == 0 && parsed == 1) {
      taskENTER_CRITICAL(&app->config_state_lock);
      app->telemetry_enabled = false;
      taskEXIT_CRITICAL(&app->config_state_lock);
      app_notify_config_modified(app);
      serial_cli_emit_event(app, "OK LOG OFF");
    } else if (strcmp(option, "POS") == 0 || strcmp(option, "P") == 0) {
      if (parsed == 1) {
        int64_t counts_now = 0;
        float cpr = 0.0f;
        taskENTER_CRITICAL(&app->config_state_lock);
        counts_now = app->encoder_position_counts;
        cpr = motor_hw_counts_per_revolution(app->encoder_cycles_per_channel);
        taskEXIT_CRITICAL(&app->config_state_lock);
        float deg_now = motor_hw_counts_to_degrees(counts_now, cpr);
        serial_print_fmt(app->print_queue, print_ticks, "P,%" PRId64 ",%.2f\n",
                         counts_now, deg_now);
      } else {
        bool enabled = false;
        if (!parse_on_off_arg(arg, &enabled)) {
          serial_cli_emit_event(app, "ERR LOG");
          return;
        }
        taskENTER_CRITICAL(&app->config_state_lock);
        app->pos_log_enabled = enabled;
        if (enabled) {
          app->pos_out_log_enabled = false;
        }
        taskEXIT_CRITICAL(&app->config_state_lock);
        serial_cli_emit_event(app, enabled ? "OK LOG POS ON" : "OK LOG POS OFF");
      }
    } else if (strcmp(option, "POSOUT") == 0 || strcmp(option, "OUT") == 0 ||
               strcmp(option, "POS_OUT") == 0 || strcmp(option, "DBG") == 0) {
      bool enabled = false;
      if (parsed != 2 || !parse_on_off_arg(arg, &enabled)) {
        serial_cli_emit_event(app, "ERR LOG");
        return;
      }
      taskENTER_CRITICAL(&app->config_state_lock);
      app->pos_out_log_enabled = enabled;
      taskEXIT_CRITICAL(&app->config_state_lock);
      serial_cli_emit_event(app,
                            enabled ? "OK LOG POSOUT ON" : "OK LOG POSOUT OFF");
    } else if (strcmp(option, "RATE") == 0 && parsed == 2) {
      char *rate_end = NULL;
      unsigned long period = strtoul(arg, &rate_end, 10);
      if (isdigit((unsigned char)arg[0]) && *rate_end == '\0' &&
          period >= CONTROL_PERIOD_MS && period <= UINT32_MAX) {
        taskENTER_CRITICAL(&app->config_state_lock);
        app->telemetry_period_ms = (uint32_t)period;
        taskEXIT_CRITICAL(&app->config_state_lock);
        app_notify_config_modified(app);
        serial_print_fmt(app->print_queue, print_ticks, "OK LOG RATE %lu\r\n",
                         period);
      } else {
        serial_cli_emit_event(app, "ERR LOG");
      }
    } else {
      serial_cli_emit_event(app, "ERR LOG");
    }
    return;
  }
  if (strcmp(operation, "POT") == 0 || strcmp(operation, "ADC") == 0 ||
      strcmp(operation, "POTS") == 0) {
    char option[16] = {0};
    if (sscanf(value_text, "%15s", option) == 1) {
      str_to_upper(option);
    }
    if (strcmp(option, "ON") == 0) {
      taskENTER_CRITICAL(&app->config_state_lock);
      app->pot_enabled = true;
      taskEXIT_CRITICAL(&app->config_state_lock);
      app->pot_force_sync = true;
      app_notify_config_modified(app);
      serial_cli_emit_event(app, "OK POT ON");
      return;
    }
    if (strcmp(option, "OFF") == 0) {
      taskENTER_CRITICAL(&app->config_state_lock);
      app->pot_enabled = false;
      taskEXIT_CRITICAL(&app->config_state_lock);
      app_notify_config_modified(app);
      serial_cli_emit_event(app, "OK POT OFF");
      return;
    }
    if (option[0] == '\0' || strcmp(option, "GET") == 0 ||
        strcmp(option, "READ") == 0 || strcmp(option, "STATUS") == 0) {
      pot_state_t snapshot;
      taskENTER_CRITICAL(&app->pot_state_lock);
      snapshot = app->pot_state;
      taskEXIT_CRITICAL(&app->pot_state_lock);
      char mode_ch =
          app->open_loop_mode
              ? 'O'
              : (app->clock_mode ? 'C' : (app->position_mode ? 'P' : 'V'));
      serial_print_fmt(app->print_queue, print_ticks,
                       "POT EN=%d MODE=%c KP=%.4f(%.2fV) KI=%.4f(%.2fV) "
                       "KD=%.4f(%.2fV) SP=%.2f(%.2fV)\r\n",
                       app->pot_enabled ? 1 : 0, mode_ch, snapshot.mapped_kp,
                       snapshot.voltage_v[POT_CH_KP], snapshot.mapped_ki,
                       snapshot.voltage_v[POT_CH_KI], snapshot.mapped_kd,
                       snapshot.voltage_v[POT_CH_KD], snapshot.mapped_sp,
                       snapshot.voltage_v[POT_CH_SP]);
      return;
    }
    serial_cli_emit_event(app, "ERR POT");
    return;
  }
  if (strcmp(operation, "SAVE") == 0) {
    serial_cli_emit_event(
        app,
        app_post_nvs_request(app, NVS_OP_SAVE_NOW) ? "OK NVS SAVE" : "ERR NVS");
    return;
  }
  if (strcmp(operation, "LOAD") == 0) {
    serial_cli_emit_event(
        app,
        app_post_nvs_request(app, NVS_OP_LOAD_NOW) ? "OK NVS LOAD" : "ERR NVS");
    return;
  }
  if (strcmp(operation, "NVS") == 0) {
    char subcmd[16] = {0};
    char option[16] = {0};
    int parsed = sscanf(value_text, "%15s %15s", subcmd, option);
    if (parsed >= 1) {
      str_to_upper(subcmd);
    }
    if (parsed >= 2) {
      str_to_upper(option);
    }
    if (parsed <= 0 || strcmp(subcmd, "GET") == 0 ||
        strcmp(subcmd, "STATUS") == 0) {
      nvs_stored_config_t snapshot;
      motor_control_capture_nvs_config(app, &snapshot);
      serial_print_fmt(
          app->print_queue, print_ticks,
          "NVS READY=%d DIRTY=%d AUTO=%d SAVES=%" PRIu32
          " V_KP=%.4f V_KI=%.4f V_KD=%.4f P_KP=%.4f P_KI=%.4f P_KD=%.4f"
          " C_KP=%.4f C_KI=%.4f C_KD=%.4f"
          " MINMOVE=%u POT=%d LOG=%d RATE=%" PRIu32 "\r\n",
          app->nvs_storage_ready ? 1 : 0, app->nvs_config_dirty ? 1 : 0,
          app->nvs_autosave_enabled ? 1 : 0, app->nvs_save_count,
          snapshot.speed_pid.kp, snapshot.speed_pid.ki, snapshot.speed_pid.kd,
          snapshot.position_pid.kp, snapshot.position_pid.ki,
          snapshot.position_pid.kd, snapshot.clock_pid.kp,
          snapshot.clock_pid.ki, snapshot.clock_pid.kd,
          (unsigned)snapshot.min_move_duty, (int)snapshot.pot_enabled,
          (int)snapshot.telemetry_enabled, snapshot.telemetry_period_ms);
      return;
    }
    if (strcmp(subcmd, "SAVE") == 0) {
      serial_cli_emit_event(app, app_post_nvs_request(app, NVS_OP_SAVE_NOW)
                                     ? "OK NVS SAVE"
                                     : "ERR NVS");
      return;
    }
    if (strcmp(subcmd, "LOAD") == 0) {
      serial_cli_emit_event(app, app_post_nvs_request(app, NVS_OP_LOAD_NOW)
                                     ? "OK NVS LOAD"
                                     : "ERR NVS");
      return;
    }
    if (strcmp(subcmd, "RESET") == 0 || strcmp(subcmd, "DEFAULT") == 0 ||
        strcmp(subcmd, "CLEAR") == 0) {
      serial_cli_emit_event(app,
                            app_post_nvs_request(app, NVS_OP_RESET_DEFAULTS)
                                ? "OK NVS RESET"
                                : "ERR NVS");
      return;
    }
    if (strcmp(subcmd, "AUTOSAVE") == 0) {
      if (parsed >= 2 && strcmp(option, "ON") == 0) {
        app->nvs_autosave_enabled = true;
        if (app->nvs_config_dirty) {
          app_post_nvs_request(app, NVS_OP_SCHEDULE_AUTOSAVE);
        }
        serial_cli_emit_event(app, "OK NVS AUTOSAVE ON");
        return;
      }
      if (parsed >= 2 && strcmp(option, "OFF") == 0) {
        app->nvs_autosave_enabled = false;
        serial_cli_emit_event(app, "OK NVS AUTOSAVE OFF");
        return;
      }
      serial_cli_emit_event(app, "ERR NVS AUTOSAVE");
      return;
    }
    serial_cli_emit_event(app, "ERR NVS");
    return;
  }
  if (strcmp(operation, "LCD") == 0 || strcmp(operation, "DISPLAY") == 0) {
    process_lcd_command(app, value_text);
    return;
  }
  if (strcmp(operation, "EC11") == 0) {
    process_ec11_command(app, value_text);
    return;
  }
  bool speed_command = strcmp(operation, "SPEED") == 0;
  char direction = '\0';
  if (speed_command) {
    if (!isalpha((unsigned char)value_text[0])) {
      serial_cli_emit_event(app, "ERR SPEED");
      return;
    }
    direction = (char)toupper((unsigned char)value_text[0]);
    if ((direction != 'F' && direction != 'R' && direction != 'D') ||
        (value_text[1] != '\0' && !isspace((unsigned char)value_text[1]))) {
      serial_cli_emit_event(app, "ERR SPEED");
      return;
    }
    value_text++;
    while (isspace((unsigned char)*value_text)) {
      value_text++;
    }
  } else if (strcmp(operation, "F") == 0 || strcmp(operation, "R") == 0 ||
             strcmp(operation, "D") == 0) {
    direction = operation[0];
  } else {
    serial_cli_emit_event(app, "ERR CMD");
    return;
  }
  float rpm = strtof(value_text, &end);
  while (isspace((unsigned char)*end)) {
    end++;
  }
  if (value_text[0] == '\0' || *end != '\0' || !isfinite(rpm) || rpm < 0.0f ||
      rpm > MAX_RPM) {
    serial_cli_emit_event(app, "ERR RPM");
    return;
  }
  bool next_forward = true;
  if (direction == 'F') {
    next_forward = true;
  } else if (direction == 'R') {
    next_forward = false;
  } else {
    taskENTER_CRITICAL(&app->config_state_lock);
    next_forward = app->forward;
    taskEXIT_CRITICAL(&app->config_state_lock);
  }
  serial_cli_emit_event(
      app,
      app_post_motor_event(app, EVENT_SPEED_SETPOINT, rpm, next_forward)
          ? "OK SPEED"
          : "ERR EVT");
}
