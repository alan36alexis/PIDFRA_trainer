#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa los GPIOs de direccion (IN1/IN2) y el PWM LEDC (ENA) del puente H.
 */
void motor_hw_init(void);

/**
 * @brief Aplica duty PWM y sentido de giro al puente H L298N.
 */
void motor_hw_set(uint8_t duty, bool motor_forward);

/**
 * @brief Desenergiza el puente H (IN1=0, IN2=0, duty=0).
 */
void motor_hw_stop(void);

/**
 * @brief Actualiza la frecuencia PWM del timer LEDC usado por el motor.
 */
void motor_hw_set_pwm_frequency(uint32_t pwm_frequency_hz);

/**
 * @brief Configura e inicia una unidad PCNT en modo cuadratura x4.
 */
void motor_hw_init_quadrature_pcnt(gpio_num_t gpio_a, gpio_num_t gpio_b,
                                   uint32_t max_glitch_ns,
                                   bool enable_pullups,
                                   pcnt_unit_handle_t *out_unit);

/**
 * @brief Inicializa la unidad PCNT del encoder incremental del motor.
 */
void motor_hw_encoder_init(pcnt_unit_handle_t *out_unit);

/**
 * @brief Lee y reinicia el contador hardware de una unidad PCNT.
 */
int motor_hw_encoder_get_and_clear(pcnt_unit_handle_t unit);

/**
 * @brief Lee el contador actual de una unidad PCNT sin limpiarlo.
 */
int motor_hw_encoder_get_count(pcnt_unit_handle_t unit);

/**
 * @brief Limpia el contador hardware de una unidad PCNT.
 */
void motor_hw_encoder_clear(pcnt_unit_handle_t unit);

/**
 * @brief Calcula cuentas por vuelta en cuadratura x4.
 */
float motor_hw_counts_per_revolution(uint32_t cycles_per_channel);

/**
 * @brief Convierte cuentas de encoder a grados sexagesimales.
 */
float motor_hw_counts_to_degrees(int64_t counts, float counts_per_rev);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HW_H */
