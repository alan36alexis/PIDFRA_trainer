#include "tc802a_driver.h"

#include <string.h>

#define TC802A_POWERON_WAIT_MS 50U
#define TC802A_INIT_RETRY1_MS  5U
#define TC802A_INIT_RETRY2_MS  1U
#define TC802A_CMD_SETTLE_US   50U
#define TC802A_CLEAR_WAIT_MS   2U

static int tc802a_send_nibble(const tc802a_dev_t *dev, bool rs, uint8_t nibble)
{
    if (dev == NULL || dev->hal.write_nibble == NULL) {
        return TC802A_ERR_NOT_INIT;
    }
    return dev->hal.write_nibble(dev->hal.ctx, rs, (uint8_t)(nibble & 0x0FU));
}

static int tc802a_send_byte(const tc802a_dev_t *dev, bool rs, uint8_t byte)
{
    int err = tc802a_send_nibble(dev, rs, (uint8_t)(byte >> 4));
    if (err != TC802A_OK) {
        return err;
    }
    err = tc802a_send_nibble(dev, rs, (uint8_t)(byte & 0x0FU));
    if (err != TC802A_OK) {
        return err;
    }
    if (dev->hal.delay_us != NULL) {
        dev->hal.delay_us(dev->hal.ctx, TC802A_CMD_SETTLE_US);
    }
    return TC802A_OK;
}

static int tc802a_send_cmd(const tc802a_dev_t *dev, uint8_t cmd)
{
    return tc802a_send_byte(dev, false, cmd);
}

static int tc802a_send_data(const tc802a_dev_t *dev, uint8_t data)
{
    return tc802a_send_byte(dev, true, data);
}

static int tc802a_apply_init_sequence(tc802a_dev_t *dev)
{
    if (dev == NULL || dev->hal.write_nibble == NULL ||
        dev->hal.delay_ms == NULL || dev->hal.delay_us == NULL) {
        return TC802A_ERR_INVALID_ARG;
    }

    /* Espera de estabilización de VDD tras encendido (> 40 ms según HD44780). */
    dev->hal.delay_ms(dev->hal.ctx, TC802A_POWERON_WAIT_MS);

    /* Secuencia obligatoria de sincronización en modo 4 bits con R/W fijo a GND. */
    int err = tc802a_send_nibble(dev, false, 0x03U);
    if (err != TC802A_OK) {
        return err;
    }
    dev->hal.delay_ms(dev->hal.ctx, TC802A_INIT_RETRY1_MS);

    err = tc802a_send_nibble(dev, false, 0x03U);
    if (err != TC802A_OK) {
        return err;
    }
    dev->hal.delay_ms(dev->hal.ctx, TC802A_INIT_RETRY2_MS);

    err = tc802a_send_nibble(dev, false, 0x03U);
    if (err != TC802A_OK) {
        return err;
    }
    dev->hal.delay_ms(dev->hal.ctx, TC802A_INIT_RETRY2_MS);

    /* Conmutación a interfaz de 4 bits. */
    err = tc802a_send_nibble(dev, false, 0x02U);
    if (err != TC802A_OK) {
        return err;
    }
    dev->hal.delay_ms(dev->hal.ctx, TC802A_INIT_RETRY2_MS);

    /* Function Set: 4 bits, 2 líneas, fuente 5x8. */
    err = tc802a_send_cmd(dev, TC802A_CMD_FUNCTION_4BIT_2L);
    if (err != TC802A_OK) {
        return err;
    }

    /* Display OFF durante limpieza inicial. */
    err = tc802a_send_cmd(dev, TC802A_CMD_DISPLAY_CTRL);
    if (err != TC802A_OK) {
        return err;
    }

    /* Clear Display y espera de ejecución (> 1.52 ms). */
    err = tc802a_send_cmd(dev, TC802A_CMD_CLEAR_DISPLAY);
    if (err != TC802A_OK) {
        return err;
    }
    dev->hal.delay_ms(dev->hal.ctx, TC802A_CLEAR_WAIT_MS);

    /* Entry Mode: incremento DDRAM hacia la derecha sin desplazamiento. */
    err = tc802a_send_cmd(dev, TC802A_CMD_ENTRY_MODE_INC);
    if (err != TC802A_OK) {
        return err;
    }

    dev->display_on = true;
    dev->cursor_on = false;
    dev->blink_on = false;
    dev->all_blocks_on = false;
    dev->initialized = true;

    return tc802a_set_display_control(dev, true, false, false);
}

int tc802a_init(tc802a_dev_t *dev, const tc802a_hal_t *hal)
{
    if (dev == NULL || hal == NULL || hal->write_nibble == NULL ||
        hal->delay_us == NULL || hal->delay_ms == NULL) {
        return TC802A_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->hal = *hal;
    tc802a_clear_buffer(dev);

    return tc802a_apply_init_sequence(dev);
}

int tc802a_reinit(tc802a_dev_t *dev)
{
    if (dev == NULL) {
        return TC802A_ERR_INVALID_ARG;
    }
    return tc802a_apply_init_sequence(dev);
}

int tc802a_clear_hw(tc802a_dev_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return TC802A_ERR_NOT_INIT;
    }
    tc802a_clear_buffer(dev);
    int err = tc802a_send_cmd(dev, TC802A_CMD_CLEAR_DISPLAY);
    if (err != TC802A_OK) {
        return err;
    }
    dev->hal.delay_ms(dev->hal.ctx, TC802A_CLEAR_WAIT_MS);
    return TC802A_OK;
}

int tc802a_home_hw(tc802a_dev_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return TC802A_ERR_NOT_INIT;
    }
    int err = tc802a_send_cmd(dev, TC802A_CMD_RETURN_HOME);
    if (err != TC802A_OK) {
        return err;
    }
    dev->hal.delay_ms(dev->hal.ctx, TC802A_CLEAR_WAIT_MS);
    return TC802A_OK;
}

int tc802a_set_display_control(tc802a_dev_t *dev,
                               bool display_on,
                               bool cursor_on,
                               bool blink_on)
{
    if (dev == NULL || !dev->initialized) {
        return TC802A_ERR_NOT_INIT;
    }
    uint8_t cmd = TC802A_CMD_DISPLAY_CTRL;
    if (display_on) {
        cmd |= TC802A_BIT_DISPLAY_ON;
    }
    if (cursor_on) {
        cmd |= TC802A_BIT_CURSOR_ON;
    }
    if (blink_on) {
        cmd |= TC802A_BIT_BLINK_ON;
    }

    int err = tc802a_send_cmd(dev, cmd);
    if (err == TC802A_OK) {
        dev->display_on = display_on;
        dev->cursor_on = cursor_on;
        dev->blink_on = blink_on;
    }
    return err;
}

int tc802a_create_char(tc802a_dev_t *dev, uint8_t slot, const uint8_t pattern[8])
{
    if (dev == NULL || pattern == NULL) {
        return TC802A_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return TC802A_ERR_NOT_INIT;
    }

    uint8_t cgram_addr = (uint8_t)((slot & 0x07U) << 3);
    int err = tc802a_send_cmd(dev, (uint8_t)(TC802A_CMD_SET_CGRAM_ADDR | cgram_addr));
    if (err != TC802A_OK) {
        return err;
    }
    for (uint8_t row = 0; row < 8U; row++) {
        err = tc802a_send_data(dev, (uint8_t)(pattern[row] & 0x1FU));
        if (err != TC802A_OK) {
            return err;
        }
    }
    return tc802a_send_cmd(dev, (uint8_t)(TC802A_CMD_SET_DDRAM_ADDR | TC802A_DDRAM_ROW0));
}

void tc802a_clear_buffer(tc802a_dev_t *dev)
{
    tc802a_fill_buffer(dev, ' ');
}

void tc802a_fill_buffer(tc802a_dev_t *dev, char ch)
{
    if (dev == NULL) {
        return;
    }
    for (uint8_t row = 0; row < TC802A_ROWS; row++) {
        for (uint8_t col = 0; col < TC802A_COLS; col++) {
            dev->lines[row][col] = ch;
        }
    }
}

void tc802a_set_char(tc802a_dev_t *dev, uint8_t row, uint8_t col, char ch)
{
    if (dev == NULL || row >= TC802A_ROWS || col >= TC802A_COLS) {
        return;
    }
    dev->lines[row][col] = ch;
}

void tc802a_write_line(tc802a_dev_t *dev, uint8_t row, const char *text)
{
    if (dev == NULL || row >= TC802A_ROWS) {
        return;
    }
    size_t col = 0U;
    if (text != NULL) {
        while (col < TC802A_COLS && text[col] != '\0') {
            dev->lines[row][col] = text[col];
            col++;
        }
    }
    while (col < TC802A_COLS) {
        dev->lines[row][col] = ' ';
        col++;
    }
}

int tc802a_flush(tc802a_dev_t *dev)
{
    static const uint8_t row_offsets[TC802A_ROWS] = {
        TC802A_DDRAM_ROW0,
        TC802A_DDRAM_ROW1,
    };

    if (dev == NULL || !dev->initialized) {
        return TC802A_ERR_NOT_INIT;
    }

    for (uint8_t row = 0; row < TC802A_ROWS; row++) {
        int err = tc802a_send_cmd(
            dev, (uint8_t)(TC802A_CMD_SET_DDRAM_ADDR | row_offsets[row]));
        if (err != TC802A_OK) {
            return err;
        }
        for (uint8_t col = 0; col < TC802A_COLS; col++) {
            err = tc802a_send_data(dev, (uint8_t)dev->lines[row][col]);
            if (err != TC802A_OK) {
                return err;
            }
        }
    }
    return TC802A_OK;
}

int tc802a_run_bringup(tc802a_dev_t *dev,
                       uint32_t all_on_ms,
                       tc802a_bringup_log_cb log_cb,
                       void *user_ctx)
{
    if (dev == NULL) {
        return TC802A_ERR_INVALID_ARG;
    }

    int err = tc802a_reinit(dev);
    if (err != TC802A_OK) {
        return err;
    }
    if (log_cb != NULL) {
        log_cb(TC802A_BRINGUP_STAGE_INIT_DONE, user_ctx);
    }

    dev->all_blocks_on = true;
    tc802a_fill_buffer(dev, (char)0xFF);
    err = tc802a_flush(dev);
    if (err != TC802A_OK) {
        return err;
    }
    if (log_cb != NULL) {
        log_cb(TC802A_BRINGUP_STAGE_ALL_BLOCKS_ON, user_ctx);
    }
    if (all_on_ms > 0U && dev->hal.delay_ms != NULL) {
        dev->hal.delay_ms(dev->hal.ctx, all_on_ms);
    }

    dev->all_blocks_on = false;
    tc802a_write_line(dev, 0U, "PIDFRA82");
    tc802a_write_line(dev, 1U, "12345678");
    err = tc802a_flush(dev);
    if (err != TC802A_OK) {
        return err;
    }
    if (log_cb != NULL) {
        log_cb(TC802A_BRINGUP_STAGE_PATTERN_DRAWN, user_ctx);
    }
    return TC802A_OK;
}

