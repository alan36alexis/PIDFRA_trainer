#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "tc802a_driver.h"
#include "tc802a_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  tc802a_dev_t lcd_dev;
  tc802a_espidf_hal_ctx_t hal_ctx;
  tc802a_hal_t hal;
  lcd_view_mode_t view_mode;
  uint32_t frames_rendered;
  bool auto_switch_to_dash;
  bool all_blocks_override;
  bool cleared_hold;
  uint32_t pattern_hold_ms;
} lcd_ui_ctx_t;

typedef void (*lcd_ui_log_fn)(void *user_ctx, const char *text);

/**
 * @brief Inicializa la HAL ESP-IDF, el driver TC802A y ejecuta el bring-up inicial.
 */
bool lcd_ui_init(lcd_ui_ctx_t *ctx, tc802a_bringup_log_cb bringup_cb,
                 void *bringup_user_ctx, lcd_ui_log_fn log_cb,
                 void *log_user_ctx);

/**
 * @brief Devuelve el nombre textual del modo de vista del LCD.
 */
const char *lcd_ui_view_mode_name(lcd_view_mode_t view);

/**
 * @brief Sanitiza caracteres especiales del LCD para reportes ASCII por CLI.
 */
char lcd_ui_sanitize_ascii_char(char ch);

/**
 * @brief Formatea la linea de Setpoint para pantalla 8x2 segun el modo activo.
 */
void lcd_ui_format_sp_line(char *buf, size_t buf_size,
                           const display_telemetry_snapshot_t *snap);

void lcd_ui_render_home(tc802a_dev_t *dev,
                        const display_telemetry_snapshot_t *snap);
void lcd_ui_render_main_menu(tc802a_dev_t *dev, ui_menu_item_t item,
                             const display_telemetry_snapshot_t *snap);
void lcd_ui_render_mode_select(tc802a_dev_t *dev, ui_selectable_mode_t mode);
void lcd_ui_render_mode_confirm(tc802a_dev_t *dev, ui_selectable_mode_t mode);
void lcd_ui_render_edit_param(tc802a_dev_t *dev, ui_menu_item_t item,
                              const display_telemetry_snapshot_t *snap);
void lcd_ui_render_pid(tc802a_dev_t *dev,
                       const display_telemetry_snapshot_t *snap);

/**
 * @brief Procesa un comando recibido por cola hacia el display LCD.
 */
void lcd_ui_handle_request(lcd_ui_ctx_t *ctx, const lcd_request_t *req,
                           tc802a_bringup_log_cb bringup_cb,
                           void *bringup_user_ctx, lcd_ui_log_fn log_cb,
                           void *log_user_ctx);

/**
 * @brief Ejecuta un ciclo de refresco de pantalla 8x2 (auto-transicion, menu o vista).
 */
void lcd_ui_step_render(lcd_ui_ctx_t *ctx, ui_menu_state_t menu_state,
                        ui_menu_item_t menu_item,
                        ui_selectable_mode_t candidate_mode,
                        const display_telemetry_snapshot_t *snap,
                        lcd_ui_log_fn log_cb, void *log_user_ctx);

/**
 * @brief Copia el estado actual del display al snapshot compartido bajo spinlock.
 */
void lcd_ui_sync_status_snapshot(const lcd_ui_ctx_t *ctx,
                                 lcd_status_snapshot_t *snapshot,
                                 portMUX_TYPE *lcd_lock);

#ifdef __cplusplus
}
#endif

#endif /* LCD_UI_H */
