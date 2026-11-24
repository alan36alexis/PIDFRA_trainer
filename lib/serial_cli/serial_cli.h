#ifndef SERIAL_CLI_H
#define SERIAL_CLI_H

#include "app_types.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Instala y configura el driver UART del CLI (115200 8N1).
 */
void serial_cli_uart_init(void);

/**
 * @brief Encola una linea de respuesta CLI hacia print_task (sin llamar a printf directamente).
 */
void serial_cli_emit_event(app_context_t *app, const char *text);

/**
 * @brief Encola la impresion del resumen de comandos CLI hacia print_task.
 */
void serial_cli_print_help(app_context_t *app);

/**
 * @brief Parsea y despacha un comando recibido por UART.
 */
void serial_cli_process_command(app_context_t *app, char *command);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_CLI_H */
