#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calcula CRC32 IEEE 802.3 sobre un bloque de memoria.
 */
uint32_t nvs_config_crc32(const void *data, size_t length);

/**
 * @brief Valida que las ganancias de un PID sean finitas y pertenezcan a [0, MAX_PID_GAIN].
 */
bool nvs_config_pid_is_valid(const pid_config_t *cfg);

/**
 * @brief Compara igualdad exacta de dos estructuras pid_config_t.
 */
bool nvs_config_pid_equal(const pid_config_t *a, const pid_config_t *b);

/**
 * @brief Construye una configuracion NVS con todos los valores por defecto y su CRC32.
 */
void nvs_config_build_defaults(nvs_stored_config_t *cfg);

/**
 * @brief Recalcula magic, version y crc32 de una configuracion preparada para persistencia.
 */
void nvs_config_update_crc(nvs_stored_config_t *cfg);

/**
 * @brief Valida cabecera, CRC32 y rangos de una configuracion leida desde NVS.
 */
bool nvs_config_validate(const nvs_stored_config_t *cfg);

/**
 * @brief Compara dos configuraciones almacenables ignorando campos de relleno.
 */
bool nvs_config_equal(const nvs_stored_config_t *a, const nvs_stored_config_t *b);

/**
 * @brief Inicializa el subsistema NVS Flash (borrando y reintentando si no hay paginas libres).
 */
esp_err_t nvs_config_init_flash(void);

/**
 * @brief Escribe y confirma la estructura de configuracion en NVS.
 */
esp_err_t nvs_config_write(bool storage_ready, const nvs_stored_config_t *cfg);

/**
 * @brief Lee la estructura de configuracion desde NVS.
 */
esp_err_t nvs_config_read(bool storage_ready, nvs_stored_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* NVS_CONFIG_H */
