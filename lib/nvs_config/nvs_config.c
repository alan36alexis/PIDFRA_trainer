#include "nvs_config.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "nvs.h"
#include "nvs_flash.h"

uint32_t nvs_config_crc32(const void *data, size_t length) {
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; i++) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; bit++) {
      if ((crc & 1U) != 0U) {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

bool nvs_config_pid_is_valid(const pid_config_t *cfg) {
  return cfg != NULL && isfinite(cfg->kp) && cfg->kp >= 0.0f &&
         cfg->kp <= MAX_PID_GAIN && isfinite(cfg->ki) && cfg->ki >= 0.0f &&
         cfg->ki <= MAX_PID_GAIN && isfinite(cfg->kd) && cfg->kd >= 0.0f &&
         cfg->kd <= MAX_PID_GAIN;
}

bool nvs_config_pid_equal(const pid_config_t *a, const pid_config_t *b) {
  return a->kp == b->kp && a->ki == b->ki && a->kd == b->kd;
}

void nvs_config_update_crc(nvs_stored_config_t *cfg) {
  if (cfg == NULL) {
    return;
  }
  cfg->magic = NVS_CONFIG_MAGIC;
  cfg->version = NVS_CONFIG_VERSION;
  cfg->crc32 = nvs_config_crc32(cfg, offsetof(nvs_stored_config_t, crc32));
}

void nvs_config_build_defaults(nvs_stored_config_t *cfg) {
  memset(cfg, 0, sizeof(*cfg));
  cfg->speed_pid =
      (pid_config_t){DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_SPEED_KD};
  cfg->position_pid =
      (pid_config_t){DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD};
  cfg->clock_pid =
      (pid_config_t){DEFAULT_CLOCK_KP, DEFAULT_CLOCK_KI, DEFAULT_CLOCK_KD};
  cfg->min_move_duty = DEFAULT_MIN_MOVE_DUTY;
  cfg->pot_enabled = (POT_DEFAULT_ENABLED != 0) ? 1U : 0U;
  cfg->telemetry_enabled = 0U;
  cfg->telemetry_period_ms = DEFAULT_LOG_PERIOD_MS;
  cfg->encoder_cycles_per_channel = ENCODER_CYCLES_PER_CHANNEL;
  cfg->encoder_zero_offset_counts = 0;
  nvs_config_update_crc(cfg);
}

bool nvs_config_validate(const nvs_stored_config_t *cfg) {
  if (cfg == NULL) {
    return false;
  }
  if (cfg->magic != NVS_CONFIG_MAGIC || cfg->version != NVS_CONFIG_VERSION) {
    return false;
  }
  uint32_t expected_crc =
      nvs_config_crc32(cfg, offsetof(nvs_stored_config_t, crc32));
  if (cfg->crc32 != expected_crc) {
    return false;
  }
  if (!nvs_config_pid_is_valid(&cfg->speed_pid) ||
      !nvs_config_pid_is_valid(&cfg->position_pid) ||
      !nvs_config_pid_is_valid(&cfg->clock_pid)) {
    return false;
  }
  if (cfg->telemetry_period_ms < CONTROL_PERIOD_MS ||
      cfg->encoder_cycles_per_channel == 0U) {
    return false;
  }
  return true;
}

bool nvs_config_equal(const nvs_stored_config_t *a,
                      const nvs_stored_config_t *b) {
  if (a == NULL || b == NULL) {
    return false;
  }
  return a->magic == b->magic && a->version == b->version &&
         nvs_config_pid_equal(&a->speed_pid, &b->speed_pid) &&
         nvs_config_pid_equal(&a->position_pid, &b->position_pid) &&
         nvs_config_pid_equal(&a->clock_pid, &b->clock_pid) &&
         a->min_move_duty == b->min_move_duty &&
         a->pot_enabled == b->pot_enabled &&
         a->telemetry_enabled == b->telemetry_enabled &&
         a->telemetry_period_ms == b->telemetry_period_ms &&
         a->encoder_cycles_per_channel == b->encoder_cycles_per_channel &&
         a->encoder_zero_offset_counts == b->encoder_zero_offset_counts;
}

esp_err_t nvs_config_init_flash(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  return err;
}

esp_err_t nvs_config_write(bool storage_ready, const nvs_stored_config_t *cfg) {
  if (!storage_ready || cfg == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    return err;
  }
  err = nvs_set_blob(handle, NVS_BLOB_KEY, cfg, sizeof(*cfg));
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  return err;
}

esp_err_t nvs_config_read(bool storage_ready, nvs_stored_config_t *cfg) {
  if (!storage_ready || cfg == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return err;
  }
  size_t required_size = sizeof(*cfg);
  err = nvs_get_blob(handle, NVS_BLOB_KEY, cfg, &required_size);
  nvs_close(handle);
  if (err == ESP_OK && required_size != sizeof(*cfg)) {
    return ESP_ERR_INVALID_SIZE;
  }
  return err;
}
