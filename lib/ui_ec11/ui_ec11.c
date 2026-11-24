#include "ui_ec11.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "motor_hw.h"

void ui_ec11_init(ui_ec11_ctx_t *ctx) {
  if (ctx == NULL) {
    return;
  }
  memset(ctx, 0, sizeof(*ctx));
  ctx->enabled = (EC11_DEFAULT_ENABLED != 0);
  ctx->menu_state = UI_STATE_HOME;
  ctx->selected_item = UI_ITEM_MODE;
  ctx->candidate_mode = UI_MODE_SPEED;

  motor_hw_init_quadrature_pcnt(EC11_CLK_GPIO, EC11_DT_GPIO,
                                EC11_GLITCH_FILTER_NS, true, &ctx->unit);
  const gpio_config_t sw_cfg = {
      .pin_bit_mask = (1ULL << EC11_SW_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&sw_cfg));
}

void ui_ec11_zero(ui_ec11_ctx_t *ctx, portMUX_TYPE *config_lock) {
  if (ctx == NULL) {
    return;
  }
  taskENTER_CRITICAL(config_lock);
  ctx->total_counts = 0;
  ctx->detent_accum = 0;
  taskEXIT_CRITICAL(config_lock);
  motor_hw_encoder_clear(ctx->unit);
}

void ui_ec11_reset_to_home(ui_ec11_ctx_t *ctx, portMUX_TYPE *lcd_lock) {
  if (ctx == NULL) {
    return;
  }
  taskENTER_CRITICAL(lcd_lock);
  ctx->menu_state = UI_STATE_HOME;
  ctx->selected_item = UI_ITEM_MODE;
  taskEXIT_CRITICAL(lcd_lock);
}

int ui_ec11_get_edit_pot_channel(const ui_ec11_ctx_t *ctx,
                                 portMUX_TYPE *lcd_lock) {
  int edit_ch = -1;
  taskENTER_CRITICAL(lcd_lock);
  if (ctx->menu_state == UI_STATE_EDIT_PARAM) {
    edit_ch = ui_ec11_item_to_pot_channel(ctx->selected_item);
  }
  taskEXIT_CRITICAL(lcd_lock);
  return edit_ch;
}

void ui_ec11_touch_activity(ui_ec11_ctx_t *ctx, uint32_t now_ms,
                            portMUX_TYPE *lcd_lock) {
  taskENTER_CRITICAL(lcd_lock);
  ctx->last_activity_ms = now_ms;
  taskEXIT_CRITICAL(lcd_lock);
}

void ui_ec11_get_menu_state(const ui_ec11_ctx_t *ctx, portMUX_TYPE *lcd_lock,
                            ui_menu_state_t *out_state,
                            ui_menu_item_t *out_item,
                            ui_selectable_mode_t *out_candidate) {
  taskENTER_CRITICAL(lcd_lock);
  *out_state = ctx->menu_state;
  *out_item = ctx->selected_item;
  *out_candidate = ctx->candidate_mode;
  taskEXIT_CRITICAL(lcd_lock);
}

ui_selectable_mode_t ui_ec11_current_mode(bool open_loop_mode, bool clock_mode,
                                          bool position_mode) {
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

motor_event_type_t ui_ec11_mode_event(ui_selectable_mode_t mode) {
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

const char *ui_ec11_mode_short_label(ui_selectable_mode_t mode) {
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

int ui_ec11_item_to_pot_channel(ui_menu_item_t item) {
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

void ui_ec11_poll(ui_ec11_ctx_t *ctx, uint32_t now_ms, bool encoder_fault,
                  ui_selectable_mode_t cur_mode, portMUX_TYPE *config_lock,
                  portMUX_TYPE *lcd_lock, ui_ec11_action_t *out_action) {
  memset(out_action, 0, sizeof(*out_action));
  out_action->sync_pot_channel = -1;

  if (ctx->unit != NULL) {
    int delta_counts = 0;
    if (pcnt_unit_get_count(ctx->unit, &delta_counts) == ESP_OK &&
        delta_counts != 0) {
      pcnt_unit_clear_count(ctx->unit);
      taskENTER_CRITICAL(config_lock);
      ctx->total_counts += delta_counts;
      ctx->detent_accum += delta_counts;
      int detents = ctx->detent_accum / EC11_COUNTS_PER_DETENT;
      ctx->detent_accum %= EC11_COUNTS_PER_DETENT;
      taskEXIT_CRITICAL(config_lock);

      if (detents != 0 && ctx->enabled) {
        taskENTER_CRITICAL(lcd_lock);
        ctx->last_activity_ms = now_ms;
        if (ctx->menu_state == UI_STATE_HOME) {
          ctx->menu_state = UI_STATE_MAIN_MENU;
          int rem = (detents > 0) ? (detents - 1) : (detents + 1);
          int step = rem % (int)UI_ITEM_COUNT;
          int next = ((int)UI_ITEM_MODE + step + (int)UI_ITEM_COUNT) %
                     (int)UI_ITEM_COUNT;
          ctx->selected_item = (ui_menu_item_t)next;
        } else if (ctx->menu_state == UI_STATE_MAIN_MENU ||
                   ctx->menu_state == UI_STATE_EDIT_PARAM) {
          ctx->menu_state = UI_STATE_MAIN_MENU;
          int step = detents % (int)UI_ITEM_COUNT;
          int next = ((int)ctx->selected_item + step + (int)UI_ITEM_COUNT) %
                     (int)UI_ITEM_COUNT;
          ctx->selected_item = (ui_menu_item_t)next;
        } else if (ctx->menu_state == UI_STATE_MODE_SELECT ||
                   ctx->menu_state == UI_STATE_MODE_CONFIRM) {
          ctx->menu_state = UI_STATE_MODE_SELECT;
          int step = detents % (int)UI_MODE_COUNT;
          int next = ((int)ctx->candidate_mode + step + (int)UI_MODE_COUNT) %
                     (int)UI_MODE_COUNT;
          ctx->candidate_mode = (ui_selectable_mode_t)next;
        }
        taskEXIT_CRITICAL(lcd_lock);
      }
    }
  }

  taskENTER_CRITICAL(lcd_lock);
  if (ctx->menu_state != UI_STATE_HOME &&
      (now_ms - ctx->last_activity_ms) >= UI_MENU_TIMEOUT_MS) {
    ctx->menu_state = UI_STATE_HOME;
    ctx->selected_item = UI_ITEM_MODE;
  }
  taskEXIT_CRITICAL(lcd_lock);

  if (!ctx->enabled) {
    ctx->sw_debounced_pressed = false;
    ctx->sw_last_raw_pressed = false;
    ctx->sw_debounce_ticks = 0U;
    ctx->sw_long_press_handled = false;
    return;
  }

  bool raw_pressed = (gpio_get_level(EC11_SW_GPIO) == 0);
  if (raw_pressed == ctx->sw_last_raw_pressed) {
    if (ctx->sw_debounce_ticks < EC11_SW_DEBOUNCE_TICKS) {
      ctx->sw_debounce_ticks++;
    }
  } else {
    ctx->sw_last_raw_pressed = raw_pressed;
    ctx->sw_debounce_ticks = 1U;
  }

  if (ctx->sw_debounce_ticks >= EC11_SW_DEBOUNCE_TICKS &&
      raw_pressed != ctx->sw_debounced_pressed) {
    ctx->sw_debounced_pressed = raw_pressed;
    if (ctx->sw_debounced_pressed) {
      ctx->sw_press_start_ms = now_ms;
      ctx->sw_long_press_handled = false;
    } else if (!ctx->sw_long_press_handled) {
      if (encoder_fault) {
        ui_ec11_reset_to_home(ctx, lcd_lock);
        out_action->has_motor_event = true;
        out_action->motor_event_type = EVENT_STOP;
        out_action->has_log_event = true;
        snprintf(out_action->log_event_text, sizeof(out_action->log_event_text),
                 "EC11 SW -> CLR FLT");
      } else {
        bool apply_mode = false;
        ui_selectable_mode_t confirmed_mode = cur_mode;
        int sync_pot_ch = -1;

        taskENTER_CRITICAL(lcd_lock);
        ctx->last_activity_ms = now_ms;
        switch (ctx->menu_state) {
        case UI_STATE_HOME:
          ctx->menu_state = UI_STATE_MAIN_MENU;
          ctx->selected_item = UI_ITEM_MODE;
          break;
        case UI_STATE_MAIN_MENU:
          if (ctx->selected_item == UI_ITEM_MODE) {
            ctx->candidate_mode = cur_mode;
            ctx->menu_state = UI_STATE_MODE_SELECT;
          } else {
            sync_pot_ch = ui_ec11_item_to_pot_channel(ctx->selected_item);
            ctx->menu_state = UI_STATE_EDIT_PARAM;
          }
          break;
        case UI_STATE_MODE_SELECT:
          ctx->menu_state = UI_STATE_MODE_CONFIRM;
          break;
        case UI_STATE_MODE_CONFIRM:
          confirmed_mode = ctx->candidate_mode;
          ctx->menu_state = UI_STATE_HOME;
          ctx->selected_item = UI_ITEM_MODE;
          apply_mode = true;
          break;
        case UI_STATE_EDIT_PARAM:
          ctx->menu_state = UI_STATE_HOME;
          ctx->selected_item = UI_ITEM_MODE;
          break;
        }
        taskEXIT_CRITICAL(lcd_lock);

        out_action->sync_pot_channel = sync_pot_ch;
        if (apply_mode) {
          out_action->has_motor_event = true;
          out_action->motor_event_type = ui_ec11_mode_event(confirmed_mode);
          out_action->has_log_event = true;
          snprintf(out_action->log_event_text,
                   sizeof(out_action->log_event_text),
                   "EC11 MODE CONFIRMED -> %s",
                   ui_ec11_mode_short_label(confirmed_mode));
        }
      }
    }
  }

  if (ctx->sw_debounced_pressed && !ctx->sw_long_press_handled &&
      (now_ms - ctx->sw_press_start_ms) >= EC11_SW_LONG_PRESS_MS) {
    ctx->sw_long_press_handled = true;
    ui_ec11_reset_to_home(ctx, lcd_lock);
    out_action->has_motor_event = true;
    out_action->motor_event_type = EVENT_STOP;
    out_action->has_log_event = true;
    snprintf(out_action->log_event_text, sizeof(out_action->log_event_text),
             "EC11 LONG -> STOP");
  }
}
