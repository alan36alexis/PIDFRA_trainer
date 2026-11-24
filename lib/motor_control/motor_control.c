#include "motor_control.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "esp_err.h"
#include "freertos/task.h"
#include "motor_hw.h"
#include "nvs_config.h"
#include "serial_print.h"

static app_context_t *s_pulse_app = NULL;

static void pulse_drive_cb(bool motor_forward, uint8_t duty) {
  if (s_pulse_app != NULL) {
    s_pulse_app->pulse_direction = motor_forward;
    s_pulse_app->pulse_duty = duty;
  }
  motor_hw_set(duty, motor_forward);
}

static void pulse_release_cb(void) {
  if (s_pulse_app != NULL) {
    s_pulse_app->pulse_duty = 0;
  }
  motor_hw_stop();
}

static void pulse_wait_ms_cb(uint32_t milliseconds) {
  vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

static uint32_t pulse_time_ms_now_cb(void) {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static int32_t pulse_encoder_delta_counts_cb(void) {
  if (s_pulse_app == NULL) {
    return 0;
  }
  int current_counts = motor_hw_encoder_get_count(s_pulse_app->encoder_unit);
  return (int32_t)(current_counts -
                   (int)s_pulse_app->pulse_start_counts_snapshot);
}

static bool pulse_motion_started_cb(void) {
  return pulse_encoder_delta_counts_cb() != 0;
}

static void pulse_diag_cb(const char *tag, uint32_t elapsed_ms,
                          int32_t counts) {
  if (s_pulse_app == NULL) {
    return;
  }
  const TickType_t ticks = pdMS_TO_TICKS(PRINT_CLI_SEND_TIMEOUT_MS);
  if (strcmp(tag, "DBG TON_TO_MOTION") == 0) {
    serial_print_pulse_diag(s_pulse_app->print_queue, true, elapsed_ms, 0,
                            ticks);
  } else if (strcmp(tag, "DBG PULSE_COUNTS") == 0) {
    serial_print_pulse_diag(s_pulse_app->print_queue, false, 0U, counts, ticks);
  }
}

static void reset_safety_search_tracker(app_context_t *app) {
  app->safety_unsettled_counts = 0;
  app->safety_settle_ticks = 0;
  app->safety_filtered_rpm = 0.0f;
}

static void reset_encoder_state_locked(app_context_t *app) {
  app->encoder_position_counts = 0;
  app->target_position_counts = 0.0f;
  app->clock_base_counts = 0;
  app->clock_step_count = 0;
}

static void latch_encoder_fault(app_context_t *app) {
  taskENTER_CRITICAL(&app->config_state_lock);
  app->encoder_fault = true;
  app->motor_fault_reason = FAULT_ENCODER_STALL;
  app->motor_state = FSM_FAULT;
  app->running = false;
  reset_safety_search_tracker(app);
  motor_pid_reset(&app->speed_pid);
  motor_pid_reset(&app->position_pid);
  motor_pid_reset(&app->clock_pid);
  taskEXIT_CRITICAL(&app->config_state_lock);
  motor_hw_stop();
  ui_ec11_reset_to_home(&app->ec11, &app->lcd_state_lock);
  app_post_log_event(app, "STALL ENC/MOT; LOCK STOP");
}

static void latch_setpoint_revs_fault(app_context_t *app, float traveled_revs) {
  taskENTER_CRITICAL(&app->config_state_lock);
  const char *mode_name =
      app->clock_mode
          ? "CLOCK"
          : (app->position_mode ? "POS"
                                : (app->open_loop_mode ? "OPEN" : "SPEED"));
  app->encoder_fault = true;
  app->motor_fault_reason = FAULT_SETPOINT_TIMEOUT_REVS;
  app->motor_state = FSM_FAULT;
  app->running = false;
  app->pos_in_target = false;
  reset_safety_search_tracker(app);
  motor_pid_reset(&app->speed_pid);
  motor_pid_reset(&app->position_pid);
  motor_pid_reset(&app->clock_pid);
  taskEXIT_CRITICAL(&app->config_state_lock);
  motor_hw_stop();
  ui_ec11_reset_to_home(&app->ec11, &app->lcd_state_lock);
  char msg[128];
  snprintf(msg, sizeof(msg),
           "SAFETY STOP: NO SP EN 1.5 VUELTAS (MODO=%s REV=%.2f); LOCK STOP",
           mode_name, (double)traveled_revs);
  app_post_log_event(app, msg);
}

static bool update_active_pid_gain(app_context_t *app,
                                   motor_event_type_t type, float value) {
  bool changed = false;
  taskENTER_CRITICAL(&app->config_state_lock);
  const pot_range_t *ranges =
      pot_get_pid_ranges(app->clock_mode, app->position_mode);
  pid_config_t *cfg =
      app->clock_mode
          ? &app->clock_pid_config
          : (app->position_mode ? &app->position_pid_config
                                : &app->speed_pid_config);
  motor_pid_t *pid =
      app->clock_mode
          ? &app->clock_pid
          : (app->position_mode ? &app->position_pid : &app->speed_pid);
  if (type == EVENT_POT_KP_UPDATE) {
    float clamped =
        pot_clamp_range(value, ranges[0].min_val, ranges[0].max_val);
    changed = fabsf(cfg->kp - clamped) > 1e-6f;
    cfg->kp = clamped;
    pid->kp = clamped;
  } else if (type == EVENT_POT_KI_UPDATE) {
    float clamped =
        pot_clamp_range(value, ranges[1].min_val, ranges[1].max_val);
    changed = fabsf(cfg->ki - clamped) > 1e-6f;
    cfg->ki = clamped;
    pid->ki = clamped;
  } else if (type == EVENT_POT_KD_UPDATE) {
    float clamped =
        pot_clamp_range(value, ranges[2].min_val, ranges[2].max_val);
    changed = fabsf(cfg->kd - clamped) > 1e-6f;
    cfg->kd = clamped;
    pid->kd = clamped;
  }
  taskEXIT_CRITICAL(&app->config_state_lock);
  return changed;
}

static uint8_t run_position_pid_step(app_context_t *app, motor_pid_t *pid,
                                     float target_counts, float sample_time,
                                     bool log_status, bool reset_on_target,
                                     bool *movement_expected) {
  float output = 0.0f;
  float position_error = 0.0f;
  uint8_t min_duty = 0;
  bool emit_pos_out = false;
  bool sp_reached_flag = false;

  taskENTER_CRITICAL(&app->config_state_lock);
  float pos_f = (float)app->encoder_position_counts;
  output = motor_pid_update(pid, target_counts, pos_f, sample_time);
  position_error = fabsf(target_counts - pos_f);
  min_duty = app->min_move_duty;
  if (position_error > POSITION_TOLERANCE_COUNTS) {
    app->pos_in_target = false;
    emit_pos_out = log_status;
    sp_reached_flag = false;
  } else if (!app->pos_in_target) {
    app->pos_in_target = true;
    emit_pos_out = log_status;
    sp_reached_flag = true;
    if (reset_on_target) {
      motor_pid_reset(pid);
    }
  }
  taskEXIT_CRITICAL(&app->config_state_lock);

  bool motor_forward = output >= 0.0f;
  uint8_t applied_duty = 0;

  if (position_error > POSITION_TOLERANCE_COUNTS) {
    if (emit_pos_out) {
      serial_print_pos_out(app->print_queue, output, position_error, false);
    }
    float abs_output = fabsf(output);
    if (abs_output > 0.0f) {
      abs_output += (float)min_duty;
      applied_duty = (uint8_t)fminf(abs_output, PWM_MAX_DUTY);
      *movement_expected = applied_duty >= STALL_MIN_DUTY;
    } else {
      applied_duty = 0;
      *movement_expected = false;
    }
  } else {
    applied_duty = 0;
    *movement_expected = false;
    if (emit_pos_out) {
      serial_print_pos_out(app->print_queue, output, position_error,
                           sp_reached_flag);
    }
  }
  motor_hw_set(applied_duty, motor_forward);
  return applied_duty;
}

void app_context_init_defaults(app_context_t *app) {
  memset(app, 0, sizeof(*app));
  app->pot_state_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  app->config_state_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  app->lcd_state_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

  app->min_move_duty = DEFAULT_MIN_MOVE_DUTY;
  app->forward = true;
  app->encoder_cycles_per_channel = ENCODER_CYCLES_PER_CHANNEL;
  app->pos_out_log_enabled = true;
  app->telemetry_period_ms = DEFAULT_LOG_PERIOD_MS;

  app->speed_pid_config =
      (pid_config_t){DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_SPEED_KD};
  app->position_pid_config =
      (pid_config_t){DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD};
  app->clock_pid_config =
      (pid_config_t){DEFAULT_CLOCK_KP, DEFAULT_CLOCK_KI, DEFAULT_CLOCK_KD};
  app->motor_state = FSM_IDLE;
  app->motor_fault_reason = FAULT_NONE;

  app->pot_enabled = (POT_DEFAULT_ENABLED != 0);
  app->nvs_autosave_enabled = (NVS_DEFAULT_AUTOSAVE != 0);
  app->lcd_status_snapshot.display_on = true;
  snprintf(app->lcd_status_snapshot.line0, sizeof(app->lcd_status_snapshot.line0),
           "        ");
  snprintf(app->lcd_status_snapshot.line1, sizeof(app->lcd_status_snapshot.line1),
           "        ");
}

bool app_post_motor_event(app_context_t *app, motor_event_type_t type,
                          float value, bool direction) {
  if (app == NULL || app->motor_event_queue == NULL) {
    return false;
  }
  motor_event_t event = {
      .type = type,
      .value = value,
      .forward = direction,
  };
  return xQueueSend(app->motor_event_queue, &event, 0) == pdTRUE;
}

void app_post_log_event(app_context_t *app, const char *text) {
  if (app == NULL) {
    return;
  }
  serial_print_event(app->print_queue, text, 0);
}

bool app_post_nvs_request(app_context_t *app, nvs_op_type_t type) {
  if (app == NULL || app->nvs_request_queue == NULL) {
    return false;
  }
  nvs_request_t req = {.type = type};
  return xQueueSend(app->nvs_request_queue, &req, 0) == pdTRUE;
}

bool app_post_lcd_request(app_context_t *app, lcd_req_type_t type, bool flag) {
  if (app == NULL || app->lcd_request_queue == NULL) {
    return false;
  }
  lcd_request_t req = {
      .type = type,
      .flag = flag,
  };
  return xQueueSend(app->lcd_request_queue, &req, 0) == pdTRUE;
}

void app_notify_config_modified(app_context_t *app) {
  app->nvs_config_dirty = true;
  if (app->nvs_autosave_enabled) {
    app_post_nvs_request(app, NVS_OP_SCHEDULE_AUTOSAVE);
  }
}

void motor_control_apply_speed_pid(app_context_t *app) {
  motor_pid_init(&app->speed_pid, app->speed_pid_config.kp,
                 app->speed_pid_config.ki, app->speed_pid_config.kd, 0.0f,
                 PWM_MAX_DUTY);
}

void motor_control_apply_position_pid(app_context_t *app) {
  motor_pid_init(&app->position_pid, app->position_pid_config.kp,
                 app->position_pid_config.ki, app->position_pid_config.kd,
                 -PWM_MAX_DUTY, PWM_MAX_DUTY);
}

void motor_control_apply_clock_pid(app_context_t *app) {
  motor_pid_init(&app->clock_pid, app->clock_pid_config.kp,
                 app->clock_pid_config.ki, app->clock_pid_config.kd,
                 -PWM_MAX_DUTY, PWM_MAX_DUTY);
}

void motor_control_reset_pids_to_defaults(app_context_t *app) {
  taskENTER_CRITICAL(&app->config_state_lock);
  app->speed_pid_config =
      (pid_config_t){DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_SPEED_KD};
  app->position_pid_config =
      (pid_config_t){DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD};
  app->clock_pid_config =
      (pid_config_t){DEFAULT_CLOCK_KP, DEFAULT_CLOCK_KI, DEFAULT_CLOCK_KD};
  motor_control_apply_speed_pid(app);
  motor_control_apply_position_pid(app);
  motor_control_apply_clock_pid(app);
  taskEXIT_CRITICAL(&app->config_state_lock);
}

void motor_control_handle_event(app_context_t *app,
                                const motor_event_t *event) {
  if (event->type == EVENT_ENCODER_STALL) {
    latch_encoder_fault(app);
    return;
  }
  if (event->type == EVENT_SETPOINT_TIMEOUT_REVS) {
    latch_setpoint_revs_fault(app, event->value);
    return;
  }
  if (event->type == EVENT_POT_KP_UPDATE ||
      event->type == EVENT_POT_KI_UPDATE ||
      event->type == EVENT_POT_KD_UPDATE) {
    if (update_active_pid_gain(app, event->type, event->value)) {
      app_notify_config_modified(app);
    }
    return;
  }

  bool reset_pids = true;
  bool need_clear_hw_encoder = false;
  bool need_stop_hw_motor = false;

  taskENTER_CRITICAL(&app->config_state_lock);
  motor_state_t prev_state = app->motor_state;
  bool was_running = app->running;
  bool was_speed_mode =
      !app->position_mode && !app->open_loop_mode && !app->clock_mode;
  bool was_position_mode =
      app->position_mode && !app->open_loop_mode && !app->clock_mode;
  bool was_open_loop_mode =
      app->open_loop_mode && !app->position_mode && !app->clock_mode;

  switch (event->type) {
  case EVENT_STOP:
    app->target_rpm = 0.0f;
    app->manual_duty = 0;
    app->running = false;
    app->encoder_fault = false;
    app->motor_fault_reason = FAULT_NONE;
    app->open_loop_mode = false;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->pos_in_target = false;
    app->motor_state = FSM_IDLE;
    reset_safety_search_tracker(app);
    break;
  case EVENT_MODE_SPEED:
    app->position_mode = false;
    app->open_loop_mode = false;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->manual_duty = 0;
    app->running = false;
    app->pos_in_target = false;
    if (!app->encoder_fault) {
      app->motor_fault_reason = FAULT_NONE;
    }
    app->motor_state = FSM_IDLE;
    reset_encoder_state_locked(app);
    need_clear_hw_encoder = true;
    reset_safety_search_tracker(app);
    break;
  case EVENT_MODE_POSITION:
    app->position_mode = true;
    app->open_loop_mode = false;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->manual_duty = 0;
    app->running = false;
    app->pos_in_target = false;
    if (!app->encoder_fault) {
      app->motor_fault_reason = FAULT_NONE;
    }
    app->motor_state = FSM_IDLE;
    reset_encoder_state_locked(app);
    need_clear_hw_encoder = true;
    reset_safety_search_tracker(app);
    break;
  case EVENT_MODE_CLOCK:
    app->position_mode = false;
    app->open_loop_mode = false;
    app->clock_mode = true;
    app->manual_duty = 0;
    app->encoder_fault = false;
    app->motor_fault_reason = FAULT_NONE;
    app->pos_in_target = false;
    app->clock_elapsed_ms = 0;
    reset_encoder_state_locked(app);
    need_clear_hw_encoder = true;
    app->motor_state = FSM_CLOCK;
    app->running = true;
    reset_safety_search_tracker(app);
    break;
  case EVENT_MODE_OPEN_LOOP:
    app->position_mode = false;
    app->open_loop_mode = true;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->running = false;
    app->manual_duty = 0;
    app->pos_in_target = false;
    if (!app->encoder_fault) {
      app->motor_fault_reason = FAULT_NONE;
    }
    app->motor_state = FSM_IDLE;
    reset_encoder_state_locked(app);
    need_clear_hw_encoder = true;
    reset_safety_search_tracker(app);
    break;
  case EVENT_SPEED_SETPOINT:
    if (!was_speed_mode) {
      reset_encoder_state_locked(app);
      need_clear_hw_encoder = true;
    }
    app->target_rpm = event->value;
    app->forward = event->forward;
    app->position_mode = false;
    app->open_loop_mode = false;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->motor_state = app->target_rpm > 0.0f ? FSM_SPEED : FSM_IDLE;
    app->running = app->target_rpm > 0.0f;
    if (!was_speed_mode || prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker(app);
    } else {
      app->safety_settle_ticks = 0;
    }
    break;
  case EVENT_POSITION_SETPOINT:
  case EVENT_ANGLE_SETPOINT:
    if (!was_position_mode) {
      reset_encoder_state_locked(app);
      need_clear_hw_encoder = true;
    }
    app->target_position_counts = event->value;
    app->pos_in_target = false;
    app->position_mode = true;
    app->open_loop_mode = false;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->motor_state = FSM_POSITION;
    app->running = true;
    if (!was_position_mode || prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker(app);
    } else {
      app->safety_settle_ticks = 0;
    }
    break;
  case EVENT_DUTY_SETPOINT:
    if (!was_open_loop_mode) {
      reset_encoder_state_locked(app);
      need_clear_hw_encoder = true;
    }
    app->target_rpm = 0.0f;
    app->forward = event->forward;
    app->position_mode = false;
    app->open_loop_mode = true;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->manual_duty = (uint8_t)fminf(fmaxf(event->value, 0.0f), PWM_MAX_DUTY);
    app->motor_state = app->manual_duty > 0 ? FSM_OPEN_LOOP : FSM_IDLE;
    app->running = app->manual_duty > 0;
    if (!was_open_loop_mode || prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker(app);
    } else {
      app->safety_settle_ticks = 0;
    }
    break;
  case EVENT_CALIBRATE_START:
    app->target_rpm = 0.0f;
    app->manual_duty = 0;
    app->running = false;
    app->encoder_fault = false;
    app->motor_fault_reason = FAULT_NONE;
    app->open_loop_mode = false;
    app->clock_mode = false;
    app->clock_elapsed_ms = 0;
    app->pos_in_target = false;
    app->motor_state = FSM_IDLE;
    reset_encoder_state_locked(app);
    need_clear_hw_encoder = true;
    reset_safety_search_tracker(app);
    need_stop_hw_motor = true;
    break;
  case EVENT_POT_SP_UPDATE:
    if (app->encoder_fault) {
      taskEXIT_CRITICAL(&app->config_state_lock);
      return;
    }
    if (app->clock_mode) {
      reset_pids = false;
      break;
    }
    if (prev_state == FSM_IDLE || !was_running) {
      reset_safety_search_tracker(app);
    } else {
      app->safety_settle_ticks = 0;
    }
    if (app->open_loop_mode) {
      float sp =
          pot_clamp_range(event->value, POT_OPEN_SP_MIN, POT_OPEN_SP_MAX);
      app->manual_duty = (uint8_t)lroundf(sp);
      app->motor_state = app->manual_duty > 0 ? FSM_OPEN_LOOP : FSM_IDLE;
      app->running = app->manual_duty > 0;
    } else if (app->position_mode) {
      float sp_deg =
          pot_clamp_range(event->value, POT_POS_SP_MIN, POT_POS_SP_MAX);
      app->target_position_counts =
          sp_deg *
          motor_hw_counts_per_revolution(app->encoder_cycles_per_channel) /
          360.0f;
      app->pos_in_target = false;
      app->motor_state = FSM_POSITION;
      app->running = true;
    } else {
      app->target_rpm =
          pot_clamp_range(event->value, POT_SPEED_SP_MIN, POT_SPEED_SP_MAX);
      app->motor_state = app->target_rpm > 0.0f ? FSM_SPEED : FSM_IDLE;
      app->running = app->target_rpm > 0.0f;
    }
    reset_pids = (app->motor_state == FSM_IDLE);
    break;
  case EVENT_ENCODER_STALL:
  case EVENT_SETPOINT_TIMEOUT_REVS:
  case EVENT_POT_KP_UPDATE:
  case EVENT_POT_KI_UPDATE:
  case EVENT_POT_KD_UPDATE:
    break;
  }

  if (reset_pids) {
    motor_pid_reset(&app->speed_pid);
    motor_pid_reset(&app->position_pid);
    motor_pid_reset(&app->clock_pid);
  }
  if (app->motor_state == FSM_IDLE) {
    need_stop_hw_motor = true;
  }
  taskEXIT_CRITICAL(&app->config_state_lock);

  if (need_clear_hw_encoder) {
    motor_hw_encoder_clear(app->encoder_unit);
  }
  if (need_stop_hw_motor) {
    motor_hw_stop();
  }
}

void motor_control_step(app_context_t *app, int counts, float sample_time,
                        uint32_t *movement_start_ms, uint32_t *no_motion_ms,
                        uint32_t *telemetry_elapsed_ms,
                        uint32_t *pos_log_elapsed_ms) {
  taskENTER_CRITICAL(&app->config_state_lock);
  app->encoder_position_counts += counts;
  int64_t cur_pos_counts = app->encoder_position_counts;
  uint32_t cycles_per_ch = app->encoder_cycles_per_channel;
  motor_state_t cur_state = app->motor_state;
  bool cur_forward = app->forward;
  uint8_t cur_manual_duty = app->manual_duty;
  float cur_target_rpm = app->target_rpm;
  float cur_target_pos = app->target_position_counts;
  bool cur_pos_out_log = app->pos_out_log_enabled;
  taskEXIT_CRITICAL(&app->config_state_lock);

  float counts_per_revolution = motor_hw_counts_per_revolution(cycles_per_ch);
  float measured_rpm =
      ((float)abs(counts) * 60.0f) / (counts_per_revolution * sample_time);
  uint8_t applied_duty = 0;
  bool movement_expected = false;

  if (cur_state == FSM_SPEED || cur_state == FSM_POSITION ||
      cur_state == FSM_CLOCK || cur_state == FSM_OPEN_LOOP) {
    if (cur_state == FSM_POSITION) {
      applied_duty = run_position_pid_step(
          app, &app->position_pid, cur_target_pos, sample_time,
          cur_pos_out_log, false, &movement_expected);
    } else if (cur_state == FSM_CLOCK) {
      taskENTER_CRITICAL(&app->config_state_lock);
      app->clock_elapsed_ms += CONTROL_PERIOD_MS;
      if (app->clock_elapsed_ms >= CLOCK_STEP_PERIOD_MS) {
        while (app->clock_elapsed_ms >= CLOCK_STEP_PERIOD_MS) {
          app->clock_elapsed_ms -= CLOCK_STEP_PERIOD_MS;
          app->clock_step_count++;
        }
        app->target_position_counts =
            (float)app->clock_base_counts +
            (float)(((double)app->clock_step_count *
                     (double)counts_per_revolution) /
                    (double)CLOCK_DIVISIONS_PER_REV);
        app->pos_in_target = false;
        app->clock_pid.previous_error =
            app->target_position_counts - (float)app->encoder_position_counts;
      }
      cur_target_pos = app->target_position_counts;
      taskEXIT_CRITICAL(&app->config_state_lock);

      applied_duty = run_position_pid_step(
          app, &app->clock_pid, cur_target_pos, sample_time, false, true,
          &movement_expected);
    } else if (cur_state == FSM_SPEED) {
      float duty = 0.0f;
      uint8_t min_duty = 0;
      taskENTER_CRITICAL(&app->config_state_lock);
      cur_target_rpm = app->target_rpm;
      cur_forward = app->forward;
      min_duty = app->min_move_duty;
      duty = motor_pid_update(&app->speed_pid, cur_target_rpm, measured_rpm,
                              sample_time);
      taskEXIT_CRITICAL(&app->config_state_lock);

      float abs_duty = fabsf(duty);
      if (fabsf(cur_target_rpm) > 0.0f && abs_duty > 0.0f) {
        if (abs_duty < (float)min_duty) {
          abs_duty = (float)min_duty;
        }
        applied_duty = (uint8_t)fminf(abs_duty, PWM_MAX_DUTY);
        movement_expected = applied_duty >= STALL_MIN_DUTY;
      } else {
        applied_duty = 0;
        movement_expected = false;
      }
      motor_hw_set(applied_duty, cur_forward);
    } else {
      applied_duty = cur_manual_duty;
      motor_hw_set(applied_duty, cur_forward);
      movement_expected = false;
    }
  } else {
    motor_hw_stop();
  }

  if (cur_state == FSM_SPEED || cur_state == FSM_POSITION ||
      cur_state == FSM_CLOCK || cur_state == FSM_OPEN_LOOP) {
    float max_search_counts = SAFETY_MAX_SEARCH_REVS * counts_per_revolution;
    app->safety_filtered_rpm +=
        SAFETY_SPEED_RPM_EMA_ALPHA * (measured_rpm - app->safety_filtered_rpm);
    bool setpoint_reached = false;
    bool clock_lag_exceeded = false;

    if (cur_state == FSM_POSITION || cur_state == FSM_CLOCK) {
      float pos_error = fabsf(cur_target_pos - (float)cur_pos_counts);
      setpoint_reached = (pos_error <= (float)POSITION_TOLERANCE_COUNTS);
      if (cur_state == FSM_CLOCK && max_search_counts > 0.0f &&
          pos_error >= max_search_counts) {
        clock_lag_exceeded = true;
      }
    } else if (cur_state == FSM_SPEED) {
      if (cur_target_rpm <= 0.0f) {
        setpoint_reached = true;
      } else {
        float rpm_per_count = 60.0f / (counts_per_revolution * sample_time);
        float speed_tol = fmaxf(1.5f * rpm_per_count,
                                SAFETY_SPEED_TOL_RATIO * cur_target_rpm);
        setpoint_reached =
            (fabsf(app->safety_filtered_rpm - cur_target_rpm) <= speed_tol);
      }
    } else {
      /* En FSM_OPEN_LOOP no existe realimentacion de setpoint: el recorrido
       * continuo queda limitado a 1.5 vueltas como restriccion maxima. */
      setpoint_reached = false;
    }

    if (setpoint_reached) {
      if (app->safety_settle_ticks < SAFETY_SETTLE_TICKS) {
        app->safety_settle_ticks++;
      }
      if (app->safety_settle_ticks >= SAFETY_SETTLE_TICKS) {
        app->safety_unsettled_counts = 0;
      }
    } else {
      app->safety_settle_ticks = 0;
      app->safety_unsettled_counts += (int64_t)abs(counts);
    }

    if (max_search_counts > 0.0f &&
        ((float)app->safety_unsettled_counts >= max_search_counts ||
         clock_lag_exceeded)) {
      float traveled_revs =
          (counts_per_revolution > 0.0f)
              ? ((float)app->safety_unsettled_counts / counts_per_revolution)
              : SAFETY_MAX_SEARCH_REVS;
      if (clock_lag_exceeded && traveled_revs < SAFETY_MAX_SEARCH_REVS) {
        traveled_revs =
            fabsf(cur_target_pos - (float)cur_pos_counts) / counts_per_revolution;
      }
      motor_event_t revs_fault_event = {
          .type = EVENT_SETPOINT_TIMEOUT_REVS,
          .value = traveled_revs,
          .forward = cur_forward,
      };
      motor_control_handle_event(app, &revs_fault_event);
      applied_duty = 0;
      movement_expected = false;
      *movement_start_ms = 0;
      *no_motion_ms = 0;
    }
  } else {
    reset_safety_search_tracker(app);
  }

  motor_state_t log_state = FSM_IDLE;
  bool log_clock_mode = false;
  bool log_position_mode = false;
  float log_target_pos = 0.0f;
  float log_target_rpm = 0.0f;
  int64_t log_pos_counts = 0;
  bool log_tel_en = false;
  bool log_pos_en = false;
  uint32_t log_period_ms = DEFAULT_LOG_PERIOD_MS;

  taskENTER_CRITICAL(&app->config_state_lock);
  app->live_measured_rpm = measured_rpm;
  app->live_applied_duty = applied_duty;
  log_state = app->motor_state;
  log_clock_mode = app->clock_mode;
  log_position_mode = app->position_mode;
  log_target_pos = app->target_position_counts;
  log_target_rpm = app->target_rpm;
  log_pos_counts = app->encoder_position_counts;
  log_tel_en = app->telemetry_enabled;
  log_pos_en = app->pos_log_enabled;
  log_period_ms = app->telemetry_period_ms;
  taskEXIT_CRITICAL(&app->config_state_lock);

  *telemetry_elapsed_ms += CONTROL_PERIOD_MS;
  if (log_tel_en && *telemetry_elapsed_ms >= log_period_ms) {
    log_message_t message = {
        .kind = LOG_TELEMETRY,
        .data.telemetry =
            {
                .state = log_state,
                .clock_mode = log_clock_mode,
                .position_mode = log_position_mode,
                .duty = applied_duty,
                .reference =
                    (log_position_mode || log_clock_mode)
                        ? log_target_pos * 360.0f / counts_per_revolution
                        : log_target_rpm,
                .measured_rpm = measured_rpm,
                .measured_angle = motor_hw_counts_to_degrees(
                    log_pos_counts, counts_per_revolution),
            },
    };
    if (!serial_print_enqueue(app->print_queue, &message, 0)) {
      *telemetry_elapsed_ms = log_period_ms;
    } else {
      *telemetry_elapsed_ms = 0;
    }
  }

  *pos_log_elapsed_ms += CONTROL_PERIOD_MS;
  if (log_pos_en && *pos_log_elapsed_ms >= log_period_ms) {
    log_message_t pos_message = {
        .kind = LOG_POSITION,
        .data.position =
            {
                .counts = log_pos_counts,
                .angle_deg = motor_hw_counts_to_degrees(
                    log_pos_counts, counts_per_revolution),
            },
    };
    if (!serial_print_enqueue(app->print_queue, &pos_message, 0)) {
      *pos_log_elapsed_ms = log_period_ms;
    } else {
      *pos_log_elapsed_ms = 0;
    }
  }

  if (movement_expected && log_state != FSM_FAULT) {
    if (*movement_start_ms == 0) {
      *movement_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (abs(counts) > 0) {
      *no_motion_ms = 0;
    } else if (now_ms - *movement_start_ms >= ENCODER_STARTUP_GRACE_MS) {
      *no_motion_ms += CONTROL_PERIOD_MS;
    }
    if (*no_motion_ms >= ENCODER_STALL_TIMEOUT_MS) {
      motor_event_t stall_event = {
          .type = EVENT_ENCODER_STALL,
      };
      motor_control_handle_event(app, &stall_event);
      *movement_start_ms = 0;
      *no_motion_ms = 0;
    }
  } else {
    *movement_start_ms = 0;
    *no_motion_ms = 0;
  }
}

void motor_control_run_pulse(app_context_t *app, bool pulse_forward,
                             uint32_t cycles, uint8_t duty,
                             uint32_t pwm_frequency_hz) {
  s_pulse_app = app;
  app->pulse_ack = false;
  app->pulse_active = true;

  uint32_t waited_ms = 0U;
  while (!app->pulse_ack && waited_ms < (uint32_t)(CONTROL_PERIOD_MS * 3U)) {
    vTaskDelay(pdMS_TO_TICKS(1));
    waited_ms++;
  }

  motor_hw_encoder_clear(app->encoder_unit);
  app->pulse_start_counts_snapshot =
      motor_hw_encoder_get_count(app->encoder_unit);

  motor_pid_run_cycles(
      pulse_forward ? MOTOR_PID_DIRECTION_FORWARD : MOTOR_PID_DIRECTION_REVERSE,
      cycles, duty, 0U, pwm_frequency_hz, &pulse_drive_cb,
      &motor_hw_set_pwm_frequency, &pulse_release_cb, &pulse_wait_ms_cb,
      &pulse_time_ms_now_cb, &pulse_encoder_delta_counts_cb,
      &pulse_motion_started_cb, &pulse_diag_cb);

  motor_hw_set_pwm_frequency(PWM_FREQUENCY_HZ);
  int pulse_counts = motor_hw_encoder_get_and_clear(app->encoder_unit);

  taskENTER_CRITICAL(&app->config_state_lock);
  app->encoder_position_counts += pulse_counts;
  app->forward = pulse_forward;
  reset_safety_search_tracker(app);
  taskEXIT_CRITICAL(&app->config_state_lock);

  app->pulse_active = false;
  app->pulse_ack = false;
  s_pulse_app = NULL;
}

void motor_control_capture_nvs_config(app_context_t *app,
                                      nvs_stored_config_t *cfg) {
  memset(cfg, 0, sizeof(*cfg));
  taskENTER_CRITICAL(&app->config_state_lock);
  cfg->speed_pid = app->speed_pid_config;
  cfg->position_pid = app->position_pid_config;
  cfg->clock_pid = app->clock_pid_config;
  cfg->min_move_duty = app->min_move_duty;
  cfg->pot_enabled = app->pot_enabled ? 1U : 0U;
  cfg->telemetry_enabled = app->telemetry_enabled ? 1U : 0U;
  cfg->telemetry_period_ms = app->telemetry_period_ms;
  cfg->encoder_cycles_per_channel = app->encoder_cycles_per_channel;
  cfg->encoder_zero_offset_counts = app->encoder_zero_offset_counts;
  taskEXIT_CRITICAL(&app->config_state_lock);
  nvs_config_update_crc(cfg);
}

void motor_control_apply_nvs_config(app_context_t *app,
                                    const nvs_stored_config_t *cfg) {
  taskENTER_CRITICAL(&app->config_state_lock);
  app->speed_pid_config = cfg->speed_pid;
  app->position_pid_config = cfg->position_pid;
  app->clock_pid_config = cfg->clock_pid;
  app->min_move_duty = cfg->min_move_duty;
  app->pot_enabled = (cfg->pot_enabled != 0U);
  app->telemetry_enabled = (cfg->telemetry_enabled != 0U);
  app->telemetry_period_ms = cfg->telemetry_period_ms;
  app->encoder_cycles_per_channel = cfg->encoder_cycles_per_channel;
  app->encoder_zero_offset_counts = cfg->encoder_zero_offset_counts;
  motor_control_apply_speed_pid(app);
  motor_control_apply_position_pid(app);
  motor_control_apply_clock_pid(app);
  taskEXIT_CRITICAL(&app->config_state_lock);
}

void motor_control_nvs_init_and_load(app_context_t *app) {
  esp_err_t err = nvs_config_init_flash();
  if (err != ESP_OK) {
    app->nvs_storage_ready = false;
    nvs_stored_config_t defaults;
    nvs_config_build_defaults(&defaults);
    motor_control_apply_nvs_config(app, &defaults);
    app_post_log_event(app, "NVS INIT ERR; DEFAULTS");
    return;
  }
  app->nvs_storage_ready = true;

  nvs_stored_config_t loaded;
  if (nvs_config_read(app->nvs_storage_ready, &loaded) == ESP_OK &&
      nvs_config_validate(&loaded)) {
    motor_control_apply_nvs_config(app, &loaded);
    taskENTER_CRITICAL(&app->config_state_lock);
    app->nvs_last_saved_config = loaded;
    app->nvs_config_dirty = false;
    taskEXIT_CRITICAL(&app->config_state_lock);
    app_post_log_event(app, "NVS LOADED");
  } else {
    nvs_stored_config_t defaults;
    nvs_config_build_defaults(&defaults);
    motor_control_apply_nvs_config(app, &defaults);
    if (nvs_config_write(app->nvs_storage_ready, &defaults) == ESP_OK) {
      taskENTER_CRITICAL(&app->config_state_lock);
      app->nvs_last_saved_config = defaults;
      app->nvs_config_dirty = false;
      app->nvs_save_count++;
      taskEXIT_CRITICAL(&app->config_state_lock);
      app_post_log_event(app, "NVS DEFAULTS SAVED");
    } else {
      app_post_log_event(app, "NVS DEFAULTS RAM");
    }
  }
}

void motor_control_nvs_save_now(app_context_t *app) {
  nvs_stored_config_t cfg;
  motor_control_capture_nvs_config(app, &cfg);
  if (nvs_config_write(app->nvs_storage_ready, &cfg) == ESP_OK) {
    taskENTER_CRITICAL(&app->config_state_lock);
    app->nvs_last_saved_config = cfg;
    app->nvs_config_dirty = false;
    app->nvs_save_count++;
    taskEXIT_CRITICAL(&app->config_state_lock);
    app_post_log_event(app, "NVS SAVED");
  } else {
    app_post_log_event(app, "NVS SAVE ERR");
  }
}

void motor_control_nvs_load_now(app_context_t *app) {
  nvs_stored_config_t cfg;
  if (nvs_config_read(app->nvs_storage_ready, &cfg) == ESP_OK &&
      nvs_config_validate(&cfg)) {
    motor_control_apply_nvs_config(app, &cfg);
    taskENTER_CRITICAL(&app->config_state_lock);
    app->nvs_last_saved_config = cfg;
    app->nvs_config_dirty = false;
    taskEXIT_CRITICAL(&app->config_state_lock);
    app_post_log_event(app, "NVS LOADED");
  } else {
    nvs_config_build_defaults(&cfg);
    motor_control_apply_nvs_config(app, &cfg);
    taskENTER_CRITICAL(&app->config_state_lock);
    app->nvs_last_saved_config = cfg;
    app->nvs_config_dirty = false;
    taskEXIT_CRITICAL(&app->config_state_lock);
    app_post_log_event(app, "NVS LOAD ERR; DEFAULTS");
  }
}

void motor_control_nvs_reset_defaults(app_context_t *app) {
  nvs_stored_config_t cfg;
  nvs_config_build_defaults(&cfg);
  motor_control_apply_nvs_config(app, &cfg);
  if (nvs_config_write(app->nvs_storage_ready, &cfg) == ESP_OK) {
    taskENTER_CRITICAL(&app->config_state_lock);
    app->nvs_last_saved_config = cfg;
    app->nvs_config_dirty = false;
    app->nvs_save_count++;
    taskEXIT_CRITICAL(&app->config_state_lock);
    app_post_log_event(app, "NVS RESET DEFAULT");
  } else {
    app_post_log_event(app, "NVS RESET ERR");
  }
}

void motor_control_nvs_autosave_if_dirty(app_context_t *app) {
  nvs_stored_config_t cfg;
  motor_control_capture_nvs_config(app, &cfg);
  bool unchanged = false;
  taskENTER_CRITICAL(&app->config_state_lock);
  unchanged = nvs_config_equal(&cfg, &app->nvs_last_saved_config);
  if (unchanged) {
    app->nvs_config_dirty = false;
  }
  taskEXIT_CRITICAL(&app->config_state_lock);

  if (!unchanged) {
    if (nvs_config_write(app->nvs_storage_ready, &cfg) == ESP_OK) {
      taskENTER_CRITICAL(&app->config_state_lock);
      app->nvs_last_saved_config = cfg;
      app->nvs_config_dirty = false;
      app->nvs_save_count++;
      taskEXIT_CRITICAL(&app->config_state_lock);
      app_post_log_event(app, "NVS AUTOSAVED");
    } else {
      app_post_log_event(app, "NVS AUTOSAVE ERR");
    }
  }
}

void motor_control_capture_display_telemetry(app_context_t *app,
                                             display_telemetry_snapshot_t *snap) {
  taskENTER_CRITICAL(&app->config_state_lock);
  snap->state = app->motor_state;
  snap->fault_reason = app->motor_fault_reason;
  snap->forward = app->forward;
  snap->position_mode = app->position_mode;
  snap->open_loop_mode = app->open_loop_mode;
  snap->clock_mode = app->clock_mode;
  snap->target_rpm = app->target_rpm;
  snap->target_position_counts = app->target_position_counts;
  snap->measured_rpm = app->live_measured_rpm;
  snap->encoder_position_counts = app->encoder_position_counts;
  snap->clock_step_count = app->clock_step_count;
  snap->applied_duty = app->live_applied_duty;
  snap->manual_duty = app->manual_duty;
  snap->active_pid =
      snap->clock_mode
          ? app->clock_pid_config
          : (snap->position_mode ? app->position_pid_config
                                 : app->speed_pid_config);
  snap->counts_per_rev =
      motor_hw_counts_per_revolution(app->encoder_cycles_per_channel);
  snap->pot_enabled = app->pot_enabled;
  snap->nvs_ready = app->nvs_storage_ready;
  snap->nvs_dirty = app->nvs_config_dirty;
  taskEXIT_CRITICAL(&app->config_state_lock);
}

char motor_control_state_code(motor_state_t state) {
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
