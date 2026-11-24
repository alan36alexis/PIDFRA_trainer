#ifndef POT_ADC_H
#define POT_ADC_H

#include <stdbool.h>

#include "app_types.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  adc_unit_t unit_id;
  adc_channel_t channel;
  adc_oneshot_unit_handle_t unit_handle;
} pot_adc_channel_t;

typedef struct {
  pot_adc_channel_t channels[POT_CH_COUNT];
  adc_cali_handle_t cali_handle;
} pot_adc_driver_t;

/**
 * @brief Inicializa los 4 canales ADC1 y la calibracion line-fitting si esta disponible.
 */
void pot_adc_init(pot_adc_driver_t *drv);

/**
 * @brief Acota un valor flotante al intervalo [min_val, max_val].
 */
float pot_clamp_range(float value, float min_val, float max_val);

/**
 * @brief Mapea un valor normalizado [0..1] linealmente al intervalo [min_val, max_val].
 */
float pot_map_clamped(float norm, float min_val, float max_val);

/**
 * @brief Devuelve la tabla de rangos [KP, KI, KD] segun el modo activo.
 */
const pot_range_t *pot_get_pid_ranges(bool is_clock, bool is_pos);

/**
 * @brief Normaliza una lectura cruda ADC a [0..1] aplicando snap en extremos.
 */
float pot_normalize_raw(float raw_value);

/**
 * @brief Sobremuestrea los 4 potenciometros, aplica filtro EMA y guarda lecturas normalizadas bajo pot_lock.
 * @return true si el estado ya estaba inicializado previamente, false en la primera muestra.
 */
bool pot_adc_sample(const pot_adc_driver_t *drv, pot_state_t *state,
                    portMUX_TYPE *pot_lock, float out_norms[POT_CH_COUNT]);

/**
 * @brief Mapea las lecturas normalizadas segun el modo activo y actualiza pot_state bajo pot_lock.
 */
void pot_adc_map_for_mode(pot_state_t *state, portMUX_TYPE *pot_lock,
                          bool was_initialized, bool cur_pos_mode,
                          bool cur_open_mode, bool cur_clock_mode,
                          const float norms[POT_CH_COUNT],
                          volatile bool *pot_force_sync,
                          float out_mapped[POT_CH_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* POT_ADC_H */
