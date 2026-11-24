#ifndef TC802A_HAL_H
#define TC802A_HAL_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__has_include)
#if __has_include("board_pins.h")
#include "board_pins.h"
#define TC802A_HAS_BOARD_PINS 1
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Códigos de retorno comunes para la capa HAL y el driver TC802A-01(R).
 */
typedef enum {
    TC802A_OK = 0,               /**< Operación completada sin errores. */
    TC802A_ERR_INVALID_ARG = -1, /**< Parámetro inválido o puntero nulo. */
    TC802A_ERR_IO = -2,          /**< Error de configuración o escritura GPIO. */
    TC802A_ERR_NOT_INIT = -3,    /**< Dispositivo o HAL no inicializado. */
} tc802a_status_t;

/** @name Asignación de pines por defecto para ESP32 NodeMCU-32S (modo 4 bits, R/W=GND) */
/**@{*/
#ifdef TC802A_HAS_BOARD_PINS
#define TC802A_DEFAULT_RS_GPIO PIN_LCD_RS /**< Pin 4 LCD: RS (0=Cmd, 1=Dato). */
#define TC802A_DEFAULT_E_GPIO  PIN_LCD_E  /**< Pin 6 LCD: Enable. */
#define TC802A_DEFAULT_D4_GPIO PIN_LCD_D4 /**< Pin 11 LCD: DB4. */
#define TC802A_DEFAULT_D5_GPIO PIN_LCD_D5 /**< Pin 12 LCD: DB5. */
#define TC802A_DEFAULT_D6_GPIO PIN_LCD_D6 /**< Pin 13 LCD: DB6. */
#define TC802A_DEFAULT_D7_GPIO PIN_LCD_D7 /**< Pin 14 LCD: DB7. */
#else
#define TC802A_DEFAULT_RS_GPIO 32 /**< Pin 4 LCD: RS (0=Cmd, 1=Dato). */
#define TC802A_DEFAULT_E_GPIO  33 /**< Pin 6 LCD: Enable. */
#define TC802A_DEFAULT_D4_GPIO 25 /**< Pin 11 LCD: DB4. */
#define TC802A_DEFAULT_D5_GPIO 26 /**< Pin 12 LCD: DB5. */
#define TC802A_DEFAULT_D6_GPIO 27 /**< Pin 13 LCD: DB6. */
#define TC802A_DEFAULT_D7_GPIO 13 /**< Pin 14 LCD: DB7. */
#endif
/**@}*/

/** @name Tiempos mínimos de pulso E (us) para HD44780 / SPLC780D */
/**@{*/
#define TC802A_DEFAULT_SETUP_US       2U /**< Estabilización RS/DB4..7 antes de E=1 (>= 2 us). */
#define TC802A_DEFAULT_PULSE_WIDTH_US 2U /**< Ancho de pulso E en alto (>= 2 us). */
#define TC802A_DEFAULT_HOLD_US        2U /**< Retención tras flanco descendente de E (>= 2 us). */
/**@}*/

/**
 * @brief Interfaz abstracta de transporte (HAL) de 4 bits para TC802A-01(R).
 */
typedef struct {
    /**
     * @brief Envía un nibble (4 bits bajos en DB4..DB7) con RS seleccionado.
     * @param ctx Puntero de contexto del backend HAL.
     * @param rs false para registro de instrucción, true para registro de datos.
     * @param nibble Bits 3..0 a reflejar en DB7..DB4.
     * @return TC802A_OK en caso de éxito o código negativo de error.
     */
    int (*write_nibble)(void *ctx, bool rs, uint8_t nibble);

    /**
     * @brief Retardo fino en microsegundos para temporización HD44780/SPLC780D.
     */
    void (*delay_us)(void *ctx, uint32_t us);

    /**
     * @brief Retardo en milisegundos (cediendo al RTOS cuando aplique).
     */
    void (*delay_ms)(void *ctx, uint32_t ms);

    /**
     * @brief Contexto opaco entregado en cada invocación de callback.
     */
    void *ctx;
} tc802a_hal_t;

#ifdef ESP_PLATFORM

/**
 * @brief Parámetros de configuración GPIO del backend ESP-IDF para LCD 8x2.
 */
typedef struct {
    int rs_gpio;             /**< GPIO para RS. */
    int e_gpio;              /**< GPIO para E. */
    int d4_gpio;             /**< GPIO para DB4. */
    int d5_gpio;             /**< GPIO para DB5. */
    int d6_gpio;             /**< GPIO para DB6. */
    int d7_gpio;             /**< GPIO para DB7. */
    uint32_t setup_us;       /**< Tiempo de estabilización antes de E=1 (us). */
    uint32_t pulse_width_us; /**< Ancho del pulso E en alto (us). */
    uint32_t hold_us;        /**< Tiempo de retención después de E=0 (us). */
} tc802a_espidf_gpio_config_t;

/**
 * @brief Estado interno del backend HAL GPIO sobre ESP-IDF.
 */
typedef struct {
    tc802a_espidf_gpio_config_t cfg; /**< Configuración aplicada. */
    bool initialized;                /**< Bandera de inicialización válida. */
} tc802a_espidf_hal_ctx_t;

/**
 * @brief Devuelve la configuración por defecto según board_pins.h.
 */
tc802a_espidf_gpio_config_t tc802a_hal_espidf_default_config(void);

/**
 * @brief Inicializa los 6 GPIOs del bus paralelo de 4 bits y vincula la HAL.
 */
int tc802a_hal_espidf_init(tc802a_espidf_hal_ctx_t *ctx,
                           tc802a_hal_t *hal,
                           const tc802a_espidf_gpio_config_t *cfg);

/**
 * @brief Lleva las líneas de salida a nivel bajo y marca el contexto como no inicializado.
 */
int tc802a_hal_espidf_deinit(tc802a_espidf_hal_ctx_t *ctx);

#endif /* ESP_PLATFORM */

#ifdef __cplusplus
}
#endif

#endif /* TC802A_HAL_H */

