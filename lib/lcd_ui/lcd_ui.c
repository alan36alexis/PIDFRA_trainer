#include "lcd_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"

_Static_assert(APP_LCD_COLS == TC802A_COLS,
               "APP_LCD_COLS must match TC802A_COLS");

static void lcd_ui_emit_log(lcd_ui_log_fn log_cb, void *log_user_ctx,
                            const char *text) {
  if (log_cb != NULL) {
    log_cb(log_user_ctx, text);
  }
}

bool lcd_ui_init(lcd_ui_ctx_t *ctx, tc802a_bringup_log_cb bringup_cb,
                 void *bringup_user_ctx, lcd_ui_log_fn log_cb,
                 void *log_user_ctx) {
  if (ctx == NULL) {
    return false;
  }
  memset(ctx, 0, sizeof(*ctx));
  ctx->view_mode = LCD_VIEW_BRINGUP_PATTERN;
  ctx->auto_switch_to_dash = true;

  tc802a_espidf_gpio_config_t lcd_gpio_cfg = tc802a_hal_espidf_default_config();
  lcd_gpio_cfg.rs_gpio = PIN_LCD_RS;
  lcd_gpio_cfg.e_gpio = PIN_LCD_E;
  lcd_gpio_cfg.d4_gpio = PIN_LCD_D4;
  lcd_gpio_cfg.d5_gpio = PIN_LCD_D5;
  lcd_gpio_cfg.d6_gpio = PIN_LCD_D6;
  lcd_gpio_cfg.d7_gpio = PIN_LCD_D7;

  if (tc802a_hal_espidf_init(&ctx->hal_ctx, &ctx->hal, &lcd_gpio_cfg) !=
      TC802A_OK) {
    lcd_ui_emit_log(log_cb, log_user_ctx, "LCD HAL INIT ERR");
    return false;
  }

  char bus_msg[112];
  snprintf(bus_msg, sizeof(bus_msg),
           "LCD 8x2 RS=%d E=%d D4=%d D5=%d D6=%d D7=%d", lcd_gpio_cfg.rs_gpio,
           lcd_gpio_cfg.e_gpio, lcd_gpio_cfg.d4_gpio, lcd_gpio_cfg.d5_gpio,
           lcd_gpio_cfg.d6_gpio, lcd_gpio_cfg.d7_gpio);
  lcd_ui_emit_log(log_cb, log_user_ctx, bus_msg);

  if (tc802a_init(&ctx->lcd_dev, &ctx->hal) != TC802A_OK) {
    lcd_ui_emit_log(log_cb, log_user_ctx, "LCD TC802A INIT ERR");
    tc802a_hal_espidf_deinit(&ctx->hal_ctx);
    return false;
  }

  if (tc802a_run_bringup(&ctx->lcd_dev, LCD_BRINGUP_ALL_ON_MS, bringup_cb,
                         bringup_user_ctx) == TC802A_OK) {
    ctx->frames_rendered++;
  } else {
    lcd_ui_emit_log(log_cb, log_user_ctx, "LCD BRINGUP ERR");
  }
  return true;
}

const char *lcd_ui_view_mode_name(lcd_view_mode_t view) {
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

char lcd_ui_sanitize_ascii_char(char ch) {
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

void lcd_ui_format_sp_line(char *buf, size_t buf_size,
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

void lcd_ui_render_home(tc802a_dev_t *dev,
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
    lcd_ui_format_sp_line(line1, sizeof(line1), snap);
  }

  tc802a_write_line(dev, 0U, line0);
  tc802a_write_line(dev, 1U, line1);
}

void lcd_ui_render_main_menu(tc802a_dev_t *dev, ui_menu_item_t item,
                             const display_telemetry_snapshot_t *snap) {
  static const char *const item_headers[UI_ITEM_COUNT] = {
      "->MODE  ", "->SP    ", "->KP    ", "->KI    ", "->KD    ",
  };
  char line1[32];

  switch (item) {
  case UI_ITEM_SP:
    lcd_ui_format_sp_line(line1, sizeof(line1), snap);
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

void lcd_ui_render_mode_select(tc802a_dev_t *dev, ui_selectable_mode_t mode) {
  static const char *const nav_labels[UI_MODE_COUNT] = {
      ">SPEED  ",
      ">POSIC. ",
      ">CLOCK  ",
      ">OPEN   ",
  };
  tc802a_write_line(dev, 0U, "SEL MODO");
  tc802a_write_line(dev, 1U, nav_labels[mode % UI_MODE_COUNT]);
}

void lcd_ui_render_mode_confirm(tc802a_dev_t *dev, ui_selectable_mode_t mode) {
  static const char *const confirm_labels[UI_MODE_COUNT] = {
      "SW:SPEED",
      "SW:POSIC",
      "SW:CLOCK",
      "SW:OPEN ",
  };
  tc802a_write_line(dev, 0U, "CONFIRM?");
  tc802a_write_line(dev, 1U, confirm_labels[mode % UI_MODE_COUNT]);
}

void lcd_ui_render_edit_param(tc802a_dev_t *dev, ui_menu_item_t item,
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
    lcd_ui_format_sp_line(line1, sizeof(line1), snap);
    break;
  }

  tc802a_write_line(dev, 0U, line0);
  tc802a_write_line(dev, 1U, line1);
}

void lcd_ui_render_pid(tc802a_dev_t *dev,
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

void lcd_ui_handle_request(lcd_ui_ctx_t *ctx, const lcd_request_t *req,
                           tc802a_bringup_log_cb bringup_cb,
                           void *bringup_user_ctx, lcd_ui_log_fn log_cb,
                           void *log_user_ctx) {
  switch (req->type) {
  case LCD_REQ_BRINGUP:
    ctx->auto_switch_to_dash = true;
    ctx->all_blocks_override = false;
    ctx->cleared_hold = false;
    ctx->pattern_hold_ms = 0U;
    ctx->view_mode = LCD_VIEW_BRINGUP_PATTERN;
    if (tc802a_run_bringup(&ctx->lcd_dev, LCD_BRINGUP_ALL_ON_MS, bringup_cb,
                           bringup_user_ctx) == TC802A_OK) {
      ctx->frames_rendered++;
    } else {
      lcd_ui_emit_log(log_cb, log_user_ctx, "LCD BRINGUP ERR");
    }
    break;
  case LCD_REQ_SET_VIEW_PATTERN:
    ctx->auto_switch_to_dash = false;
    ctx->all_blocks_override = false;
    ctx->cleared_hold = false;
    ctx->lcd_dev.all_blocks_on = false;
    ctx->view_mode = LCD_VIEW_BRINGUP_PATTERN;
    tc802a_write_line(&ctx->lcd_dev, 0U, "PIDFRA82");
    tc802a_write_line(&ctx->lcd_dev, 1U, "12345678");
    if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
      ctx->frames_rendered++;
    }
    lcd_ui_emit_log(log_cb, log_user_ctx, "LCD VIEW PATTERN");
    break;
  case LCD_REQ_SET_VIEW_DASH:
    ctx->auto_switch_to_dash = false;
    ctx->all_blocks_override = false;
    ctx->cleared_hold = false;
    ctx->lcd_dev.all_blocks_on = false;
    ctx->view_mode = LCD_VIEW_DASHBOARD;
    lcd_ui_emit_log(log_cb, log_user_ctx, "LCD VIEW DASHBOARD");
    break;
  case LCD_REQ_SET_VIEW_PID:
    ctx->auto_switch_to_dash = false;
    ctx->all_blocks_override = false;
    ctx->cleared_hold = false;
    ctx->lcd_dev.all_blocks_on = false;
    ctx->view_mode = LCD_VIEW_PID;
    lcd_ui_emit_log(log_cb, log_user_ctx, "LCD VIEW PID");
    break;
  case LCD_REQ_SET_ALL_ON:
    ctx->all_blocks_override = req->flag;
    ctx->cleared_hold = false;
    ctx->lcd_dev.all_blocks_on = req->flag;
    if (req->flag) {
      tc802a_fill_buffer(&ctx->lcd_dev, (char)0xFF);
      if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
        ctx->frames_rendered++;
      }
    } else if (ctx->view_mode == LCD_VIEW_BRINGUP_PATTERN) {
      tc802a_write_line(&ctx->lcd_dev, 0U, "PIDFRA82");
      tc802a_write_line(&ctx->lcd_dev, 1U, "12345678");
      if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
        ctx->frames_rendered++;
      }
    }
    break;
  case LCD_REQ_SET_CURSOR:
    tc802a_set_display_control(&ctx->lcd_dev, ctx->lcd_dev.display_on,
                               req->flag, ctx->lcd_dev.blink_on);
    break;
  case LCD_REQ_SET_BLINK:
    tc802a_set_display_control(&ctx->lcd_dev, ctx->lcd_dev.display_on,
                               ctx->lcd_dev.cursor_on, req->flag);
    break;
  case LCD_REQ_SET_DISPLAY:
    tc802a_set_display_control(&ctx->lcd_dev, req->flag,
                               ctx->lcd_dev.cursor_on, ctx->lcd_dev.blink_on);
    break;
  case LCD_REQ_CLEAR:
    ctx->auto_switch_to_dash = false;
    ctx->all_blocks_override = false;
    ctx->cleared_hold = true;
    ctx->lcd_dev.all_blocks_on = false;
    if (tc802a_clear_hw(&ctx->lcd_dev) == TC802A_OK) {
      ctx->frames_rendered++;
    }
    break;
  }
}

void lcd_ui_step_render(lcd_ui_ctx_t *ctx, ui_menu_state_t menu_state,
                        ui_menu_item_t menu_item,
                        ui_selectable_mode_t candidate_mode,
                        const display_telemetry_snapshot_t *snap,
                        lcd_ui_log_fn log_cb, void *log_user_ctx) {
  if (!ctx->all_blocks_override && !ctx->cleared_hold &&
      ctx->view_mode == LCD_VIEW_BRINGUP_PATTERN && ctx->auto_switch_to_dash) {
    ctx->pattern_hold_ms += LCD_REFRESH_PERIOD_MS;
    if (ctx->pattern_hold_ms >= LCD_BRINGUP_PATTERN_HOLD_MS) {
      ctx->auto_switch_to_dash = false;
      ctx->view_mode = LCD_VIEW_DASHBOARD;
      lcd_ui_emit_log(log_cb, log_user_ctx, "LCD HOME VIEW ACTIVE");
    }
  }

  if (!ctx->all_blocks_override && !ctx->cleared_hold) {
    if (menu_state == UI_STATE_MAIN_MENU) {
      lcd_ui_render_main_menu(&ctx->lcd_dev, menu_item, snap);
      if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
        ctx->frames_rendered++;
      }
    } else if (menu_state == UI_STATE_MODE_SELECT) {
      lcd_ui_render_mode_select(&ctx->lcd_dev, candidate_mode);
      if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
        ctx->frames_rendered++;
      }
    } else if (menu_state == UI_STATE_MODE_CONFIRM) {
      lcd_ui_render_mode_confirm(&ctx->lcd_dev, candidate_mode);
      if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
        ctx->frames_rendered++;
      }
    } else if (menu_state == UI_STATE_EDIT_PARAM) {
      lcd_ui_render_edit_param(&ctx->lcd_dev, menu_item, snap);
      if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
        ctx->frames_rendered++;
      }
    } else if (snap->state == FSM_FAULT ||
               ctx->view_mode == LCD_VIEW_DASHBOARD ||
               ctx->view_mode == LCD_VIEW_PID) {
      if (ctx->view_mode == LCD_VIEW_PID && snap->state != FSM_FAULT) {
        lcd_ui_render_pid(&ctx->lcd_dev, snap);
      } else {
        lcd_ui_render_home(&ctx->lcd_dev, snap);
      }
      if (tc802a_flush(&ctx->lcd_dev) == TC802A_OK) {
        ctx->frames_rendered++;
      }
    }
  }
}

void lcd_ui_sync_status_snapshot(const lcd_ui_ctx_t *ctx,
                                 lcd_status_snapshot_t *snapshot,
                                 portMUX_TYPE *lcd_lock) {
  taskENTER_CRITICAL(lcd_lock);
  snapshot->initialized = ctx->lcd_dev.initialized;
  snapshot->view_mode = ctx->view_mode;
  snapshot->display_on = ctx->lcd_dev.display_on;
  snapshot->cursor_on = ctx->lcd_dev.cursor_on;
  snapshot->blink_on = ctx->lcd_dev.blink_on;
  snapshot->all_blocks_on = ctx->lcd_dev.all_blocks_on;
  snapshot->frames_rendered = ctx->frames_rendered;
  for (size_t i = 0U; i < TC802A_COLS; i++) {
    snapshot->line0[i] =
        lcd_ui_sanitize_ascii_char(ctx->lcd_dev.lines[0][i]);
    snapshot->line1[i] =
        lcd_ui_sanitize_ascii_char(ctx->lcd_dev.lines[1][i]);
  }
  snapshot->line0[TC802A_COLS] = '\0';
  snapshot->line1[TC802A_COLS] = '\0';
  taskEXIT_CRITICAL(lcd_lock);
}
