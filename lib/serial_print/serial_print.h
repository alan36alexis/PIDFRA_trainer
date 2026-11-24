#ifndef SERIAL_PRINT_H
#define SERIAL_PRINT_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encola un mensaje estructurado hacia print_task.
 */
bool serial_print_enqueue(QueueHandle_t print_queue, const log_message_t *msg,
                          TickType_t timeout_ticks);

/**
 * @brief Encola un evento asincrono ("EVT <text>\r\n") hacia print_task.
 */
bool serial_print_event(QueueHandle_t print_queue, const char *text,
                        TickType_t timeout_ticks);

/**
 * @brief Encola una linea de texto cruda (que ya incluye salto de linea) hacia print_task.
 */
bool serial_print_line(QueueHandle_t print_queue, const char *line,
                       TickType_t timeout_ticks);

/**
 * @brief Formatea en memoria local (sin tocar stdout) y encola hacia print_task.
 */
bool serial_print_fmt(QueueHandle_t print_queue, TickType_t timeout_ticks,
                      const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/**
 * @brief Solicita a print_task la impresion del banner de ayuda CLI.
 */
bool serial_print_help(QueueHandle_t print_queue, TickType_t timeout_ticks);

/**
 * @brief Encola sin bloqueo (0 ticks) un diagnostico de lazo de posicion desde control_task.
 */
bool serial_print_pos_out(QueueHandle_t print_queue, float output, float error,
                          bool sp_reached);

/**
 * @brief Encola un diagnostico de pulso (TON_TO_MOTION o PULSE_COUNTS) hacia print_task.
 */
bool serial_print_pulse_diag(QueueHandle_t print_queue, bool is_ton_to_motion,
                             uint32_t elapsed_ms, int32_t counts,
                             TickType_t timeout_ticks);

/**
 * @brief Unico punto de uso de printf en todo el firmware (ejecutado exclusivamente por print_task).
 */
void serial_print_write_message(const log_message_t *message);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PRINT_H */
