#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Estructura de estado de un controlador PID discreto.
 */
typedef struct {
    float kp;              /**< Ganancia proporcional. */
    float ki;              /**< Ganancia integral. */
    float kd;              /**< Ganancia derivativa. */
    float output_min;      /**< Límite inferior de salida. */
    float output_max;      /**< Límite superior de salida. */
    float integral;        /**< Acumulador integral. */
    float previous_error;  /**< Error anterior para derivada. */
    int initialized;       /**< Bandera que indica si el PID ya fue inicializado. */
} motor_pid_t;

/**
 * @brief Dirección lógica del actuador para un pulso de comando.
 */
typedef enum {
    MOTOR_PID_DIRECTION_STOP = 0,      /**< Sin movimiento. */
    MOTOR_PID_DIRECTION_FORWARD = 1,   /**< Mover en sentido directo. */
    MOTOR_PID_DIRECTION_REVERSE = -1, /**< Mover en sentido inverso. */
} motor_pid_direction_t;

/**
 * @brief Callback para activar la salida del driver con una dirección y duty específico.
 * @param direction true para sentido directo, false para inverso.
 * @param duty Duty PWM aplicado, normalmente entre 0 y 255.
 */
typedef void (*motor_pid_drive_cb)(bool direction, uint8_t duty);

/**
 * @brief Callback para cambiar la frecuencia PWM usada por el driver.
 * @param pwm_frequency_hz Frecuencia PWM deseada en Hz.
 */
typedef void (*motor_pid_set_pwm_cb)(uint32_t pwm_frequency_hz);

/**
 * @brief Callback para desenergizar el motor al terminar la secuencia.
 */
typedef void (*motor_pid_release_cb)(void);

/**
 * @brief Callback para esperar un tiempo en milisegundos.
 * @param milliseconds Tiempo de espera solicitado.
 */
typedef void (*motor_pid_wait_ms_cb)(uint32_t milliseconds);

/**
 * @brief Callback para consultar el tiempo actual en milisegundos.
 * @return Tiempo actual medido por el sistema.
 */
typedef uint32_t (*motor_pid_get_time_ms_cb)(void);

/**
 * @brief Callback para consultar el acumulado de cuentas del encoder.
 * @return Cantidad actual de cuentas del encoder.
 */
typedef int32_t (*motor_pid_get_counts_cb)(void);

/**
 * @brief Callback para indicar si el encoder ya detectó movimiento.
 * @return true si hubo movimiento detectado, false en caso contrario.
 */
typedef bool (*motor_pid_motion_started_cb)(void);

/**
 * @brief Callback para reportar diagnósticos de tiempo y cuentas del pulso.
 * @param tag Etiqueta del diagnóstico.
 * @param elapsed_ms Tiempo medido en milisegundos.
 * @param counts Cantidad de cuentas acumuladas del encoder.
 */
typedef void (*motor_pid_diag_cb)(const char *tag, uint32_t elapsed_ms,
                                  int32_t counts);

/**
 * @brief Inicializa un controlador PID con sus ganancias y límites.
 * @param pid Puntero al objeto motor_pid_t a inicializar.
 * @param kp Ganancia proporcional.
 * @param ki Ganancia integral.
 * @param kd Ganancia derivativa.
 * @param output_min Límite inferior de la salida del PID.
 * @param output_max Límite superior de la salida del PID.
 * @note Reinicia también el acumulador integral y el estado previo del error.
 */
void motor_pid_init(motor_pid_t *pid, float kp, float ki, float kd,
                    float output_min, float output_max);

/**
 * @brief Reinicia el estado interno del controlador PID.
 * @param pid Puntero al controlador a reiniciar.
 * @note Limpia el error previo, el acumulador integral y la bandera initialized.
 */
void motor_pid_reset(motor_pid_t *pid);

/**
 * @brief Calcula la salida del PID para un punto de consigna y una medición.
 * @param pid Puntero al controlador PID.
 * @param setpoint Valor deseado.
 * @param measurement Valor medido real.
 * @param sample_time_seconds Período de muestreo en segundos.
 * @return Salida del controlador ya acotada por output_min y output_max.
 * @note Si sample_time_seconds <= 0, la función devuelve 0 sin actualizar el estado.
 */
float motor_pid_update(motor_pid_t *pid, float setpoint, float measurement,
                       float sample_time_seconds);

/**
 * @brief Ejecuta una secuencia de impulsos de potencia en una dirección específica.
 * @param direction Dirección del movimiento: MOTOR_PID_DIRECTION_FORWARD,
 * MOTOR_PID_DIRECTION_REVERSE o MOTOR_PID_DIRECTION_STOP.
 * @param n_cycles Cantidad de ciclos de activación a ejecutar.
 * @param duty Duty PWM a aplicar durante cada ciclo.
 * @param cycle_ms Duración de cada ciclo en milisegundos.
 * @param pwm_frequency_hz Frecuencia PWM del pulso en Hz.
 * @param drive_cb Callback para energizar el motor con la dirección y el duty.
 * @param set_pwm_cb Callback para ajustar la frecuencia PWM antes del pulso.
 * @param release_cb Callback para desenergizar el motor al final.
 * @param wait_ms_cb Callback para esperar el tiempo de cada ciclo.
 * @note Si n_cycles es cero, la función no hace nada.
 */
void motor_pid_run_cycles(int direction, uint32_t n_cycles, uint8_t duty,
                         uint32_t cycle_ms, uint32_t pwm_frequency_hz,
                         motor_pid_drive_cb drive_cb,
                         motor_pid_set_pwm_cb set_pwm_cb,
                         motor_pid_release_cb release_cb,
                         motor_pid_wait_ms_cb wait_ms_cb,
                         motor_pid_get_time_ms_cb time_ms_cb,
                         motor_pid_get_counts_cb counts_cb,
                         motor_pid_motion_started_cb motion_started_cb,
                         motor_pid_diag_cb diag_cb);

#ifdef __cplusplus
}
#endif

#endif
