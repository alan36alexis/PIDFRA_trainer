#include "tc802a_hal.h"

#ifdef ESP_PLATFORM

#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool tc802a_gpio_is_valid_output(int gpio_num)
{
    return gpio_num >= 0 && GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)gpio_num);
}

static int tc802a_espidf_write_nibble(void *ctx, bool rs, uint8_t nibble)
{
    tc802a_espidf_hal_ctx_t *hal_ctx = (tc802a_espidf_hal_ctx_t *)ctx;
    if (hal_ctx == NULL || !hal_ctx->initialized) {
        return TC802A_ERR_NOT_INIT;
    }

    gpio_set_level((gpio_num_t)hal_ctx->cfg.rs_gpio, rs ? 1 : 0);
    gpio_set_level((gpio_num_t)hal_ctx->cfg.d4_gpio, (nibble & 0x01U) ? 1 : 0);
    gpio_set_level((gpio_num_t)hal_ctx->cfg.d5_gpio, (nibble & 0x02U) ? 1 : 0);
    gpio_set_level((gpio_num_t)hal_ctx->cfg.d6_gpio, (nibble & 0x04U) ? 1 : 0);
    gpio_set_level((gpio_num_t)hal_ctx->cfg.d7_gpio, (nibble & 0x08U) ? 1 : 0);

    esp_rom_delay_us(hal_ctx->cfg.setup_us);
    gpio_set_level((gpio_num_t)hal_ctx->cfg.e_gpio, 1);
    esp_rom_delay_us(hal_ctx->cfg.pulse_width_us);
    gpio_set_level((gpio_num_t)hal_ctx->cfg.e_gpio, 0);
    esp_rom_delay_us(hal_ctx->cfg.hold_us);

    return TC802A_OK;
}

static void tc802a_espidf_delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    if (us > 0U) {
        esp_rom_delay_us(us);
    }
}

static void tc802a_espidf_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    if (ms == 0U) {
        return;
    }
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0U ? ticks : 1U);
}

tc802a_espidf_gpio_config_t tc802a_hal_espidf_default_config(void)
{
    tc802a_espidf_gpio_config_t cfg = {
        .rs_gpio = TC802A_DEFAULT_RS_GPIO,
        .e_gpio = TC802A_DEFAULT_E_GPIO,
        .d4_gpio = TC802A_DEFAULT_D4_GPIO,
        .d5_gpio = TC802A_DEFAULT_D5_GPIO,
        .d6_gpio = TC802A_DEFAULT_D6_GPIO,
        .d7_gpio = TC802A_DEFAULT_D7_GPIO,
        .setup_us = TC802A_DEFAULT_SETUP_US,
        .pulse_width_us = TC802A_DEFAULT_PULSE_WIDTH_US,
        .hold_us = TC802A_DEFAULT_HOLD_US,
    };
    return cfg;
}

int tc802a_hal_espidf_init(tc802a_espidf_hal_ctx_t *ctx,
                           tc802a_hal_t *hal,
                           const tc802a_espidf_gpio_config_t *cfg)
{
    if (ctx == NULL || hal == NULL) {
        return TC802A_ERR_INVALID_ARG;
    }

    tc802a_espidf_gpio_config_t resolved =
        (cfg != NULL) ? *cfg : tc802a_hal_espidf_default_config();

    const int pins[6] = {
        resolved.rs_gpio,
        resolved.e_gpio,
        resolved.d4_gpio,
        resolved.d5_gpio,
        resolved.d6_gpio,
        resolved.d7_gpio,
    };
    for (int i = 0; i < 6; i++) {
        if (!tc802a_gpio_is_valid_output(pins[i])) {
            return TC802A_ERR_INVALID_ARG;
        }
        for (int j = i + 1; j < 6; j++) {
            if (pins[i] == pins[j]) {
                return TC802A_ERR_INVALID_ARG;
            }
        }
    }

    if (resolved.setup_us < TC802A_DEFAULT_SETUP_US) {
        resolved.setup_us = TC802A_DEFAULT_SETUP_US;
    }
    if (resolved.pulse_width_us < TC802A_DEFAULT_PULSE_WIDTH_US) {
        resolved.pulse_width_us = TC802A_DEFAULT_PULSE_WIDTH_US;
    }
    if (resolved.hold_us < TC802A_DEFAULT_HOLD_US) {
        resolved.hold_us = TC802A_DEFAULT_HOLD_US;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = resolved;

    /* Asegurar nivel bajo antes de habilitar el driver de salida GPIO. */
    for (int i = 0; i < 6; i++) {
        gpio_set_level((gpio_num_t)pins[i], 0);
    }

    const uint64_t mask = (1ULL << resolved.rs_gpio) |
                          (1ULL << resolved.e_gpio) |
                          (1ULL << resolved.d4_gpio) |
                          (1ULL << resolved.d5_gpio) |
                          (1ULL << resolved.d6_gpio) |
                          (1ULL << resolved.d7_gpio);
    const gpio_config_t gpio_cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&gpio_cfg) != ESP_OK) {
        return TC802A_ERR_IO;
    }

    for (int i = 0; i < 6; i++) {
        gpio_set_level((gpio_num_t)pins[i], 0);
    }

    ctx->initialized = true;

    hal->write_nibble = tc802a_espidf_write_nibble;
    hal->delay_us = tc802a_espidf_delay_us;
    hal->delay_ms = tc802a_espidf_delay_ms;
    hal->ctx = ctx;

    return TC802A_OK;
}

int tc802a_hal_espidf_deinit(tc802a_espidf_hal_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return TC802A_ERR_NOT_INIT;
    }

    gpio_set_level((gpio_num_t)ctx->cfg.e_gpio, 0);
    gpio_set_level((gpio_num_t)ctx->cfg.rs_gpio, 0);
    gpio_set_level((gpio_num_t)ctx->cfg.d4_gpio, 0);
    gpio_set_level((gpio_num_t)ctx->cfg.d5_gpio, 0);
    gpio_set_level((gpio_num_t)ctx->cfg.d6_gpio, 0);
    gpio_set_level((gpio_num_t)ctx->cfg.d7_gpio, 0);

    ctx->initialized = false;
    return TC802A_OK;
}

#endif /* ESP_PLATFORM */

