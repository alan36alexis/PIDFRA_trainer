#include "pot_adc.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

void pot_adc_init(pot_adc_driver_t *drv) {
  if (drv == NULL) {
    return;
  }
  memset(drv, 0, sizeof(*drv));

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
    drv->channels[i].unit_id = unit_id;
    drv->channels[i].channel = channel;
    drv->channels[i].unit_handle = unit_handles[unit_id];
  }

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  const adc_cali_line_fitting_config_t cali_cfg = {
      .unit_id = ADC_UNIT_1,
      .atten = POT_ADC_ATTEN,
      .bitwidth = POT_ADC_BITWIDTH,
      .default_vref = 1100U,
  };
  if (adc_cali_create_scheme_line_fitting(&cali_cfg, &drv->cali_handle) !=
      ESP_OK) {
    drv->cali_handle = NULL;
  }
#endif
}

float pot_clamp_range(float value, float min_val, float max_val) {
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

float pot_map_clamped(float norm, float min_val, float max_val) {
  float clamped_norm = pot_clamp_range(norm, 0.0f, 1.0f);
  float mapped = POT_MAP_LINEAR(clamped_norm, min_val, max_val);
  return pot_clamp_range(mapped, min_val, max_val);
}

const pot_range_t *pot_get_pid_ranges(bool is_clock, bool is_pos) {
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

float pot_normalize_raw(float raw_value) {
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

bool pot_adc_sample(const pot_adc_driver_t *drv, pot_state_t *state,
                    portMUX_TYPE *pot_lock, float out_norms[POT_CH_COUNT]) {
  float raw_filtered[POT_CH_COUNT];
  float norms[POT_CH_COUNT];
  float voltages[POT_CH_COUNT];
  bool was_initialized = false;

  taskENTER_CRITICAL(pot_lock);
  was_initialized = state->initialized;
  for (int ch = 0; ch < POT_CH_COUNT; ch++) {
    raw_filtered[ch] = state->raw_filtered[ch];
    norms[ch] = state->norm[ch];
    voltages[ch] = state->voltage_v[ch];
  }
  taskEXIT_CRITICAL(pot_lock);

  for (int ch = 0; ch < POT_CH_COUNT; ch++) {
    int32_t sum = 0;
    int valid_samples = 0;
    for (int s = 0; s < POT_MULTISAMPLE_COUNT; s++) {
      int raw = 0;
      if (adc_oneshot_read(drv->channels[ch].unit_handle,
                           drv->channels[ch].channel, &raw) == ESP_OK) {
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
      norms[ch] = pot_normalize_raw(raw_filtered[ch]);
      int cal_mv = 0;
      if (drv->cali_handle != NULL &&
          adc_cali_raw_to_voltage(drv->cali_handle,
                                  (int)lroundf(raw_filtered[ch]),
                                  &cal_mv) == ESP_OK) {
        voltages[ch] = (float)cal_mv / 1000.0f;
      } else {
        voltages[ch] =
            POT_MAP_LINEAR(norms[ch], POT_VOLTAGE_MIN_V, POT_VOLTAGE_MAX_V);
      }
    }
  }

  taskENTER_CRITICAL(pot_lock);
  for (int ch = 0; ch < POT_CH_COUNT; ch++) {
    state->raw_filtered[ch] = raw_filtered[ch];
    state->norm[ch] = norms[ch];
    state->voltage_v[ch] = voltages[ch];
    out_norms[ch] = norms[ch];
  }
  taskEXIT_CRITICAL(pot_lock);

  return was_initialized;
}

void pot_adc_map_for_mode(pot_state_t *state, portMUX_TYPE *pot_lock,
                          bool was_initialized, bool cur_pos_mode,
                          bool cur_open_mode, bool cur_clock_mode,
                          const float norms[POT_CH_COUNT],
                          volatile bool *pot_force_sync,
                          float out_mapped[POT_CH_COUNT]) {
  const pot_range_t *pid_ranges =
      pot_get_pid_ranges(cur_clock_mode, cur_pos_mode);
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

  bool clear_force_sync = false;
  taskENTER_CRITICAL(pot_lock);
  state->mapped_kp = mapped_kp;
  state->mapped_ki = mapped_ki;
  state->mapped_kd = mapped_kd;
  state->mapped_sp = mapped_sp;
  state->initialized = true;

  bool mode_changed = !state->last_mode_valid ||
                      (cur_pos_mode != state->last_position_mode) ||
                      (cur_open_mode != state->last_open_loop_mode) ||
                      (cur_clock_mode != state->last_clock_mode);

  if (!was_initialized || mode_changed) {
    for (int ch = 0; ch < POT_CH_COUNT; ch++) {
      state->last_applied_norm[ch] = norms[ch];
    }
    state->last_mode_valid = true;
    state->last_position_mode = cur_pos_mode;
    state->last_open_loop_mode = cur_open_mode;
    state->last_clock_mode = cur_clock_mode;
    clear_force_sync = mode_changed;
  }
  taskEXIT_CRITICAL(pot_lock);

  if (clear_force_sync && pot_force_sync != NULL) {
    *pot_force_sync = false;
  }

  out_mapped[POT_CH_KP] = mapped_kp;
  out_mapped[POT_CH_KI] = mapped_ki;
  out_mapped[POT_CH_KD] = mapped_kd;
  out_mapped[POT_CH_SP] = mapped_sp;
}
