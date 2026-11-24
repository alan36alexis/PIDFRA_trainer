#ifndef TC802A_DRIVER_H
#define TC802A_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tc802a_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Geometría y mapa DDRAM del display LCD Tinsharp TC802A-01(R) 8x2 */
/**@{*/
#define TC802A_COLS        8U    /**< Columnas visibles por línea. */
#define TC802A_ROWS        2U    /**< Líneas visibles. */
#define TC802A_DDRAM_ROW0  0x00U /**< Dirección base DDRAM de la línea 0. */
#define TC802A_DDRAM_ROW1  0x40U /**< Dirección base DDRAM de la línea 1. */
/**@}*/

/** @name Instrucciones HD44780 / SPLC780D */
/**@{*/
#define TC802A_CMD_CLEAR_DISPLAY    0x01U /**< Borra DDRAM y retorna cursor a 0. */
#define TC802A_CMD_RETURN_HOME      0x02U /**< Retorna cursor y shift a origen. */
#define TC802A_CMD_ENTRY_MODE_INC   0x06U /**< Incremento DDRAM sin desplazamiento. */
#define TC802A_CMD_DISPLAY_CTRL     0x08U /**< Base para D/C/B. */
#define TC802A_BIT_DISPLAY_ON       0x04U /**< Bit D: Display ON. */
#define TC802A_BIT_CURSOR_ON        0x02U /**< Bit C: Cursor subrayado ON. */
#define TC802A_BIT_BLINK_ON         0x01U /**< Bit B: Parpadeo de cursor ON. */
#define TC802A_CMD_FUNCTION_4BIT_2L 0x28U /**< Interfaz 4 bits, 2 líneas, matriz 5x8. */
#define TC802A_CMD_SET_CGRAM_ADDR   0x40U /**< Prefijo dirección CGRAM (0..63). */
#define TC802A_CMD_SET_DDRAM_ADDR   0x80U /**< Prefijo dirección DDRAM (0..127). */
/**@}*/

/**
 * @brief Etapas reportadas durante la rutina diagnóstica de bring-up.
 */
typedef enum {
    TC802A_BRINGUP_STAGE_INIT_DONE = 0, /**< Secuencia de inicialización 4 bits completada. */
    TC802A_BRINGUP_STAGE_ALL_BLOCKS_ON, /**< Todas las celdas 8x2 llenas con bloque 0xFF. */
    TC802A_BRINGUP_STAGE_PATTERN_DRAWN, /**< Patrón geométrico/texto de prueba 8x2 volcado. */
} tc802a_bringup_stage_t;

/**
 * @brief Callback opcional para emitir logs por etapa durante tc802a_run_bringup().
 */
typedef void (*tc802a_bringup_log_cb)(tc802a_bringup_stage_t stage, void *user_ctx);

/**
 * @brief Instancia del controlador TC802A-01(R) con buffer local de 8x2 caracteres.
 */
typedef struct {
    char lines[TC802A_ROWS][TC802A_COLS]; /**< Buffer local de caracteres (2x8). */
    tc802a_hal_t hal;                     /**< Capa HAL de 4 bits desacoplada. */
    bool initialized;                     /**< true tras inicialización exitosa. */
    bool display_on;                      /**< Estado actual del bit D. */
    bool cursor_on;                       /**< Estado actual del bit C. */
    bool blink_on;                        /**< Estado actual del bit B. */
    bool all_blocks_on;                   /**< Bandera de prueba de bloques sólidos (0xFF). */
} tc802a_dev_t;

/**
 * @brief Ejecuta la secuencia estándar HD44780/SPLC780D de inicialización en 4 bits.
 */
int tc802a_init(tc802a_dev_t *dev, const tc802a_hal_t *hal);

/**
 * @brief Re-ejecuta la secuencia completa de inicialización en modo 4 bits.
 */
int tc802a_reinit(tc802a_dev_t *dev);

/**
 * @brief Envía Clear Display (0x01) por hardware y limpia el buffer local.
 */
int tc802a_clear_hw(tc802a_dev_t *dev);

/**
 * @brief Envía Return Home (0x02) por hardware.
 */
int tc802a_home_hw(tc802a_dev_t *dev);

/**
 * @brief Configura los bits Display ON/OFF, Cursor y Blink (comando 0x08 | D/C/B).
 */
int tc802a_set_display_control(tc802a_dev_t *dev,
                               bool display_on,
                               bool cursor_on,
                               bool blink_on);

/**
 * @brief Define un carácter personalizado (5x8) en una de las 8 ranuras CGRAM (0..7).
 */
int tc802a_create_char(tc802a_dev_t *dev, uint8_t slot, const uint8_t pattern[8]);

/**
 * @brief Limpia el buffer local rellenándolo con espacios (' ').
 */
void tc802a_clear_buffer(tc802a_dev_t *dev);

/**
 * @brief Rellena todas las celdas del buffer local con el carácter indicado.
 */
void tc802a_fill_buffer(tc802a_dev_t *dev, char ch);

/**
 * @brief Escribe un carácter en la posición (row, col) del buffer local.
 */
void tc802a_set_char(tc802a_dev_t *dev, uint8_t row, uint8_t col, char ch);

/**
 * @brief Escribe una cadena en la fila indicada del buffer local, completando con espacios hasta 8 cols.
 */
void tc802a_write_line(tc802a_dev_t *dev, uint8_t row, const char *text);

/**
 * @brief Vuelca las 2 filas de 8 caracteres del buffer local a la DDRAM del LCD.
 */
int tc802a_flush(tc802a_dev_t *dev);

/**
 * @brief Ejecuta la secuencia diagnóstica de bring-up en 3 etapas sobre el LCD 8x2.
 */
int tc802a_run_bringup(tc802a_dev_t *dev,
                       uint32_t all_on_ms,
                       tc802a_bringup_log_cb log_cb,
                       void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* TC802A_DRIVER_H */

