#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_config.h"
#include "app_types.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lcd_ui.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "pot_adc.h"
#include "serial_cli.h"
#include "serial_print.h"
#include "ui_ec11.h"

static app_context_t g_app;

static void lcd_log_adapter(void *user_ctx, const char *text) {
  app_post_log_event((app_context_t *)user_ctx, text);
}

static void lcd_bringup_log_adapter(tc802a_bringup_stage_t stage,
                                    void *user_ctx) {
  app_context_t *app = (app_context_t *)user_ctx;
  switch (stage) {
  case TC802A_BRINGUP_STAGE_INIT_DONE:
    app_post_log_event(app, "LCD STAGE1 INIT TC802A 8x2 (4-BIT MODE)");
    break;
  case TC802A_BRINGUP_STAGE_ALL_BLOCKS_ON:
    app_post_log_event(app, "LCD STAGE2 ALL_BLOCKS_ON (0xFF) 2000ms");
    break;
  case TC802A_BRINGUP_STAGE_PATTERN_DRAWN:
    app_post_log_event(app, "LCD STAGE3 PATTERN 8x2 (PIDFRA82 / 12345678)");
    break;
  }
}

/* ============================================================================
 * Tareas FreeRTOS del sistema (visibles en el archivo principal)
 * ============================================================================ */

/**
 * @brief Tarea gatekeeper exclusiva del recurso de impresion serie (printf).
 *        Ninguna otra tarea invoca printf, evitando bloqueos e inversion de
 *        prioridad sobre stdout / UART TX en tareas de tiempo real.
 */
static void print_task(void *arg) {
  log_message_t message;
  (void)arg;

  while (true) {
    if (xQueueReceive(g_app.print_queue, &message, portMAX_DELAY) == pdTRUE) {
      serial_print_write_message(&message);
    }
  }
}

static void serial_task(void *arg) {
  uint8_t data[UART_BUFFER_SIZE];
  char command[COMMAND_BUFFER_SIZE];
  size_t command_length = 0;
  (void)arg;

  while (true) {
    int length = uart_read_bytes(UART_PORT, data, sizeof(data),
                                 pdMS_TO_TICKS(UART_RX_TIMEOUT_MS));
    for (int i = 0; i < length; i++) {
      if (data[i] == '\r' || data[i] == '\n') {
        if (command_length > 0) {
          command[command_length] = '\0';
          serial_cli_process_command(&g_app, command);
          command_length = 0;
        }
      } else if (command_length < sizeof(command) - 1) {
        command[command_length++] = (char)data[i];
      } else {
        command_length = 0;
        serial_cli_emit_event(&g_app, "ERR LONG");
      }
    }
  }
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
    while (xQueueReceive(g_app.motor_event_queue, &event, 0) == pdTRUE) {
      motor_control_handle_event(&g_app, &event);
    }

    /* Bypass de proceso PID cuando se ejecuta comando PULSE */
    if (g_app.pulse_active) {
      g_app.pulse_ack = true;
      vTaskDelayUntil(&last_wake_time, period_ticks);
      continue;
    }
    g_app.pulse_ack = false;

    int counts = motor_hw_encoder_get_and_clear(g_app.encoder_unit);
    motor_control_step(&g_app, counts, sample_time, &movement_start_ms,
                       &no_motion_ms, &telemetry_elapsed_ms,
                       &pos_log_elapsed_ms);

    vTaskDelayUntil(&last_wake_time, period_ticks);
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
    float norms[POT_CH_COUNT];
    float mapped_vals[POT_CH_COUNT];
    bool was_initialized = pot_adc_sample(&g_app.pot_driver, &g_app.pot_state,
                                          &g_app.pot_state_lock, norms);

    bool cur_pos_mode = false;
    bool cur_open_mode = false;
    bool cur_clock_mode = false;
    bool cur_forward = true;
    bool cur_pot_enabled = false;
    bool cur_encoder_fault = false;
    bool cur_pulse_active = false;
    taskENTER_CRITICAL(&g_app.config_state_lock);
    cur_pos_mode = g_app.position_mode;
    cur_open_mode = g_app.open_loop_mode;
    cur_clock_mode = g_app.clock_mode;
    cur_forward = g_app.forward;
    cur_pot_enabled = g_app.pot_enabled;
    cur_encoder_fault = g_app.encoder_fault;
    cur_pulse_active = g_app.pulse_active;
    taskEXIT_CRITICAL(&g_app.config_state_lock);

    pot_adc_map_for_mode(&g_app.pot_state, &g_app.pot_state_lock,
                         was_initialized, cur_pos_mode, cur_open_mode,
                         cur_clock_mode, norms, &g_app.pot_force_sync,
                         mapped_vals);

    if (cur_pot_enabled && !cur_encoder_fault && !cur_pulse_active) {
      bool force_sync = g_app.pot_force_sync;
      uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
      int edit_ch =
          ui_ec11_get_edit_pot_channel(&g_app.ec11, &g_app.lcd_state_lock);

      float last_applied[POT_CH_COUNT];
      taskENTER_CRITICAL(&g_app.pot_state_lock);
      for (int ch = 0; ch < POT_CH_COUNT; ch++) {
        last_applied[ch] = g_app.pot_state.last_applied_norm[ch];
      }
      taskEXIT_CRITICAL(&g_app.pot_state_lock);

      bool all_posted = true;
      bool update_last[POT_CH_COUNT] = {false};
      for (int ch = 0; ch < POT_CH_COUNT; ch++) {
        if (cur_clock_mode && ch == POT_CH_SP) {
          last_applied[ch] = norms[ch];
          update_last[ch] = true;
          continue;
        }
        if (!force_sync && ch != edit_ch) {
          last_applied[ch] = norms[ch];
          update_last[ch] = true;
          continue;
        }
        if (force_sync ||
            fabsf(norms[ch] - last_applied[ch]) >= POT_DEADBAND_NORM) {
          if (app_post_motor_event(&g_app, pot_events[ch], mapped_vals[ch],
                                   cur_forward)) {
            last_applied[ch] = norms[ch];
            update_last[ch] = true;
            if (!force_sync && ch == edit_ch) {
              ui_ec11_touch_activity(&g_app.ec11, now_ms,
                                     &g_app.lcd_state_lock);
            }
          } else {
            all_posted = false;
          }
        }
      }

      taskENTER_CRITICAL(&g_app.pot_state_lock);
      for (int ch = 0; ch < POT_CH_COUNT; ch++) {
        if (update_last[ch]) {
          g_app.pot_state.last_applied_norm[ch] = last_applied[ch];
        }
      }
      taskEXIT_CRITICAL(&g_app.pot_state_lock);

      if (all_posted && force_sync) {
        g_app.pot_force_sync = false;
      }
    } else if (cur_clock_mode) {
      g_app.pot_force_sync = false;
    }

    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    ui_selectable_mode_t cur_mode =
        ui_ec11_current_mode(cur_open_mode, cur_clock_mode, cur_pos_mode);
    ui_ec11_action_t ec11_action;
    ui_ec11_poll(&g_app.ec11, now_ms, cur_encoder_fault, cur_mode,
                 &g_app.config_state_lock, &g_app.lcd_state_lock, &ec11_action);

    if (ec11_action.sync_pot_channel >= 0 &&
        ec11_action.sync_pot_channel < POT_CH_COUNT) {
      taskENTER_CRITICAL(&g_app.pot_state_lock);
      g_app.pot_state.last_applied_norm[ec11_action.sync_pot_channel] =
          g_app.pot_state.norm[ec11_action.sync_pot_channel];
      taskEXIT_CRITICAL(&g_app.pot_state_lock);
    }
    if (ec11_action.has_motor_event) {
      if (app_post_motor_event(&g_app, ec11_action.motor_event_type, 0.0f,
                               true) &&
          ec11_action.has_log_event) {
        app_post_log_event(&g_app, ec11_action.log_event_text);
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

    if (xQueueReceive(g_app.nvs_request_queue, &req, wait_ticks) == pdTRUE) {
      switch (req.type) {
      case NVS_OP_SCHEDULE_AUTOSAVE:
        if (g_app.nvs_autosave_enabled) {
          pending_autosave = true;
          autosave_deadline = xTaskGetTickCount() + debounce_ticks;
        }
        break;
      case NVS_OP_SAVE_NOW:
        pending_autosave = false;
        motor_control_nvs_save_now(&g_app);
        break;
      case NVS_OP_LOAD_NOW:
        pending_autosave = false;
        motor_control_nvs_load_now(&g_app);
        break;
      case NVS_OP_RESET_DEFAULTS:
        pending_autosave = false;
        motor_control_nvs_reset_defaults(&g_app);
        break;
      }
    } else if (pending_autosave) {
      pending_autosave = false;
      if (g_app.nvs_autosave_enabled) {
        motor_control_nvs_autosave_if_dirty(&g_app);
      }
    }
  }
}

static void display_task(void *arg) {
  static lcd_ui_ctx_t lcd_ctx;
  (void)arg;

  if (!lcd_ui_init(&lcd_ctx, lcd_bringup_log_adapter, &g_app, lcd_log_adapter,
                   &g_app)) {
    vTaskDelete(NULL);
    return;
  }
  lcd_ui_sync_status_snapshot(&lcd_ctx, &g_app.lcd_status_snapshot,
                              &g_app.lcd_state_lock);

  const TickType_t refresh_ticks = pdMS_TO_TICKS(LCD_REFRESH_PERIOD_MS);
  while (true) {
    lcd_request_t req;
    while (xQueueReceive(g_app.lcd_request_queue, &req, 0) == pdTRUE) {
      lcd_ui_handle_request(&lcd_ctx, &req, lcd_bringup_log_adapter, &g_app,
                            lcd_log_adapter, &g_app);
    }

    ui_menu_state_t menu_state = UI_STATE_HOME;
    ui_menu_item_t menu_item = UI_ITEM_MODE;
    ui_selectable_mode_t candidate_mode = UI_MODE_SPEED;
    ui_ec11_get_menu_state(&g_app.ec11, &g_app.lcd_state_lock, &menu_state,
                           &menu_item, &candidate_mode);

    display_telemetry_snapshot_t snap;
    motor_control_capture_display_telemetry(&g_app, &snap);

    lcd_ui_step_render(&lcd_ctx, menu_state, menu_item, candidate_mode, &snap,
                       lcd_log_adapter, &g_app);
    lcd_ui_sync_status_snapshot(&lcd_ctx, &g_app.lcd_status_snapshot,
                                &g_app.lcd_state_lock);
    vTaskDelay(refresh_ticks);
  }
}

/* ============================================================================
 * Punto de entrada de la aplicacion
 * ============================================================================ */

void app_main(void) {
  app_context_init_defaults(&g_app);

  motor_hw_init();
  motor_hw_encoder_init(&g_app.encoder_unit);
  ui_ec11_init(&g_app.ec11);
  pot_adc_init(&g_app.pot_driver);
  motor_control_apply_speed_pid(&g_app);
  motor_control_apply_position_pid(&g_app);
  motor_control_apply_clock_pid(&g_app);
  serial_cli_uart_init();

  g_app.print_queue = xQueueCreate(PRINT_QUEUE_LENGTH, sizeof(log_message_t));
  ESP_ERROR_CHECK(g_app.print_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  g_app.motor_event_queue =
      xQueueCreate(MOTOR_EVENT_QUEUE_LENGTH, sizeof(motor_event_t));
  ESP_ERROR_CHECK(g_app.motor_event_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  g_app.nvs_request_queue =
      xQueueCreate(NVS_QUEUE_LENGTH, sizeof(nvs_request_t));
  ESP_ERROR_CHECK(g_app.nvs_request_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  g_app.lcd_request_queue =
      xQueueCreate(LCD_QUEUE_LENGTH, sizeof(lcd_request_t));
  ESP_ERROR_CHECK(g_app.lcd_request_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);

  motor_control_nvs_init_and_load(&g_app);

  ESP_ERROR_CHECK(xTaskCreate(print_task, "print_task", PRINT_TASK_STACK_SIZE,
                              NULL, PRINT_TASK_PRIORITY, NULL) == pdPASS
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreate(serial_task, "serial_task", SERIAL_TASK_STACK_SIZE,
                              NULL, SERIAL_TASK_PRIORITY, NULL) == pdPASS
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreate(control_task, "control_task",
                              CONTROL_TASK_STACK_SIZE, NULL,
                              CONTROL_TASK_PRIORITY, NULL) == pdPASS
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreate(potentiometer_task, "pot_task",
                              POT_TASK_STACK_SIZE, NULL, POT_TASK_PRIORITY,
                              NULL) == pdPASS
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreate(nvs_task, "nvs_task", NVS_TASK_STACK_SIZE, NULL,
                              NVS_TASK_PRIORITY, NULL) == pdPASS
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreate(display_task, "display_task", LCD_TASK_STACK_SIZE,
                              NULL, LCD_TASK_PRIORITY, NULL) == pdPASS
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);

  char startup_text[96];
  snprintf(startup_text, sizeof(startup_text), "READY A=%d B=%d C=%d Q=%d",
           ENCODER_A_GPIO, ENCODER_B_GPIO,
           (int)g_app.encoder_cycles_per_channel, QUADRATURE_FACTOR);
  serial_print_event(g_app.print_queue, startup_text, portMAX_DELAY);
  serial_print_event(g_app.print_queue, "HELP", portMAX_DELAY);
}
