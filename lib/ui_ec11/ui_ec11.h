#ifndef UI_EC11_H
#define UI_EC11_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  pcnt_unit_handle_t unit;
  volatile bool enabled;
  volatile int64_t total_counts;
  int detent_accum;
  bool sw_debounced_pressed;
  bool sw_last_raw_pressed;
  uint8_t sw_debounce_ticks;
  uint32_t sw_press_start_ms;
  bool sw_long_press_handled;
  volatile ui_menu_state_t menu_state;
  volatile ui_menu_item_t selected_item;
  volatile ui_selectable_mode_t candidate_mode;
  volatile uint32_t last_activity_ms;
} ui_ec11_ctx_t;

typedef struct {
  bool has_motor_event;
  motor_event_type_t motor_event_type;
  int sync_pot_channel;
  bool has_log_event;
  char log_event_text[48];
} ui_ec11_action_t;

/**
 * @brief Inicializa el hardware del encoder rotativo EC11 (PCNT + GPIO SW).
 */
void ui_ec11_init(ui_ec11_ctx_t *ctx);

/**
 * @brief Reinicia a cero el contador de cuentas del EC11.
 */
void ui_ec11_zero(ui_ec11_ctx_t *ctx, portMUX_TYPE *config_lock);

/**
 * @brief Vuelve la maquina de estados del menu UI a la pantalla principal.
 */
void ui_ec11_reset_to_home(ui_ec11_ctx_t *ctx, portMUX_TYPE *lcd_lock);

/**
 * @brief Devuelve el canal ADC en edicion activa o -1 si no se esta editando.
 */
int ui_ec11_get_edit_pot_channel(const ui_ec11_ctx_t *ctx,
                                 portMUX_TYPE *lcd_lock);

/**
 * @brief Actualiza la marca de tiempo de ultima actividad del menu UI.
 */
void ui_ec11_touch_activity(ui_ec11_ctx_t *ctx, uint32_t now_ms,
                            portMUX_TYPE *lcd_lock);

/**
 * @brief Lee de forma atomica el estado actual del menu UI para renderizado.
 */
void ui_ec11_get_menu_state(const ui_ec11_ctx_t *ctx, portMUX_TYPE *lcd_lock,
                            ui_menu_state_t *out_state,
                            ui_menu_item_t *out_item,
                            ui_selectable_mode_t *out_candidate);

/**
 * @brief Convierte las banderas del motor al modo seleccionable de UI.
 */
ui_selectable_mode_t ui_ec11_current_mode(bool open_loop_mode, bool clock_mode,
                                          bool position_mode);

/**
 * @brief Mapea un modo seleccionable al evento de cambio de modo de la FSM.
 */
motor_event_type_t ui_ec11_mode_event(ui_selectable_mode_t mode);

/**
 * @brief Devuelve la etiqueta corta del modo para logs.
 */
const char *ui_ec11_mode_short_label(ui_selectable_mode_t mode);

/**
 * @brief Mapea un item del menu UI al indice de canal de potenciometro (-1 si es MODE).
 */
int ui_ec11_item_to_pot_channel(ui_menu_item_t item);

/**
 * @brief Muestrea el giro del EC11, antirrebote del pulsador SW y timeout de 5s.
 */
void ui_ec11_poll(ui_ec11_ctx_t *ctx, uint32_t now_ms, bool encoder_fault,
                  ui_selectable_mode_t cur_mode, portMUX_TYPE *config_lock,
                  portMUX_TYPE *lcd_lock, ui_ec11_action_t *out_action);

#ifdef __cplusplus
}
#endif

#endif /* UI_EC11_H */
