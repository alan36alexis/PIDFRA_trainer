#include "motor_hw.h"

#include "app_config.h"
#include "driver/ledc.h"
#include "esp_err.h"

void motor_hw_init(void) {
  const ledc_timer_config_t timer_config = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = PWM_RESOLUTION,
      .freq_hz = PWM_FREQUENCY_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  const ledc_channel_config_t channel_config = {
      .gpio_num = EN_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0,
  };
  const gpio_config_t motor_config = {
      .pin_bit_mask = (1ULL << IN1_GPIO) | (1ULL << IN2_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  gpio_set_level(IN1_GPIO, 0);
  gpio_set_level(IN2_GPIO, 0);
  ESP_ERROR_CHECK(gpio_config(&motor_config));
  ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
  ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  motor_hw_stop();
}

void motor_hw_set(uint8_t duty, bool motor_forward) {
  gpio_set_level(IN1_GPIO, motor_forward ? 1 : 0);
  gpio_set_level(IN2_GPIO, motor_forward ? 0 : 1);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void motor_hw_stop(void) {
  gpio_set_level(IN1_GPIO, 0);
  gpio_set_level(IN2_GPIO, 0);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void motor_hw_set_pwm_frequency(uint32_t pwm_frequency_hz) {
  if (pwm_frequency_hz == 0U) {
    pwm_frequency_hz = PWM_FREQUENCY_HZ;
  } else if (pwm_frequency_hz < PULSE_PWM_MIN_FREQ_HZ) {
    pwm_frequency_hz = PULSE_PWM_MIN_FREQ_HZ;
  } else if (pwm_frequency_hz > PULSE_PWM_MAX_FREQ_HZ) {
    pwm_frequency_hz = PULSE_PWM_MAX_FREQ_HZ;
  }
  esp_err_t err =
      ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, pwm_frequency_hz);
  if (err != ESP_OK) {
    (void)ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, PWM_FREQUENCY_HZ);
  }
}

void motor_hw_init_quadrature_pcnt(gpio_num_t gpio_a, gpio_num_t gpio_b,
                                   uint32_t max_glitch_ns,
                                   bool enable_pullups,
                                   pcnt_unit_handle_t *out_unit) {
  const pcnt_unit_config_t unit_config = {
      .high_limit = 32767,
      .low_limit = -32768,
  };
  const pcnt_chan_config_t channel_a_config = {
      .edge_gpio_num = gpio_a,
      .level_gpio_num = gpio_b,
  };
  const pcnt_chan_config_t channel_b_config = {
      .edge_gpio_num = gpio_b,
      .level_gpio_num = gpio_a,
  };
  pcnt_channel_handle_t channel_a = NULL;
  pcnt_channel_handle_t channel_b = NULL;

  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, out_unit));
  ESP_ERROR_CHECK(pcnt_new_channel(*out_unit, &channel_a_config, &channel_a));
  ESP_ERROR_CHECK(pcnt_new_channel(*out_unit, &channel_b_config, &channel_b));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  if (enable_pullups) {
    ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_a, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_b, GPIO_PULLUP_ONLY));
  }
  if (max_glitch_ns > 0U) {
    const pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = max_glitch_ns,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*out_unit, &filter_cfg));
  }
  ESP_ERROR_CHECK(pcnt_unit_enable(*out_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(*out_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(*out_unit));
}

void motor_hw_encoder_init(pcnt_unit_handle_t *out_unit) {
  /* Orden B/A invertido en PCNT para que el giro horario (forward = true)
   * cuente positivo y sea consistente con el lazo cerrado. */
  motor_hw_init_quadrature_pcnt(ENCODER_B_GPIO, ENCODER_A_GPIO,
                                ENCODER_GLITCH_FILTER_NS, false, out_unit);
}

int motor_hw_encoder_get_and_clear(pcnt_unit_handle_t unit) {
  int counts = 0;
  if (unit != NULL) {
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &counts));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  }
  return counts;
}

int motor_hw_encoder_get_count(pcnt_unit_handle_t unit) {
  int counts = 0;
  if (unit != NULL) {
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &counts));
  }
  return counts;
}

void motor_hw_encoder_clear(pcnt_unit_handle_t unit) {
  if (unit != NULL) {
    pcnt_unit_clear_count(unit);
  }
}

float motor_hw_counts_per_revolution(uint32_t cycles_per_channel) {
  return (float)cycles_per_channel * (float)QUADRATURE_FACTOR;
}

float motor_hw_counts_to_degrees(int64_t counts, float counts_per_rev) {
  return (counts_per_rev > 0.0f) ? ((float)counts * 360.0f / counts_per_rev)
                                 : 0.0f;
}
