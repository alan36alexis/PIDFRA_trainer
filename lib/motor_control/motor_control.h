#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "motor_pid.h"
#include "pot_adc.h"
#include "ui_ec11.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  /* Colas FreeRTOS */
  QueueHandle_t motor_event_queue;
  QueueHandle_t nvs_request_queue;
  QueueHandle_t lcd_request_queue;
  QueueHandle_t print_queue;

  /* Spinlocks de sincronizacion */
  portMUX_TYPE pot_state_lock;
  portMUX_TYPE config_state_lock;
  portMUX_TYPE lcd_state_lock;

  /* Estado del motor, FSM y seguridad */
  motor_state_t motor_state;
  volatile motor_fault_reason_t motor_fault_reason;
  volatile float target_rpm;
  volatile float target_position_counts;
  volatile uint8_t manual_duty;
  volatile uint8_t min_move_duty;
  volatile bool forward;
  volatile bool running;
  volatile bool position_mode;
  volatile bool open_loop_mode;
  volatile bool clock_mode;
  volatile int64_t clock_base_counts;
  volatile uint32_t clock_step_count;
  volatile uint32_t clock_elapsed_ms;
  volatile bool pulse_active;
  volatile bool pulse_ack;
  volatile bool pulse_direction;
  volatile uint8_t pulse_duty;
  volatile int64_t pulse_start_counts_snapshot;
  volatile int64_t encoder_position_counts;
  volatile uint32_t encoder_cycles_per_channel;
  volatile int64_t encoder_zero_offset_counts;
  volatile bool encoder_fault;
  volatile bool pos_in_target;
  volatile int64_t safety_unsettled_counts;
  volatile uint32_t safety_settle_ticks;
  volatile float safety_filtered_rpm;
  int32_t speed_window_counts[SPEED_MEAS_WINDOW_TICKS];
  size_t speed_window_idx;
  uint32_t speed_window_samples;
  int32_t speed_window_sum;
  float speed_sd_accum;
  volatile float live_measured_rpm;
  volatile uint8_t live_applied_duty;

  /* Configuracion de telemetria */
  volatile bool telemetry_enabled;
  volatile bool pos_log_enabled;
  volatile bool pos_out_log_enabled;
  volatile uint32_t telemetry_period_ms;

  /* Configuracion e instancias PID */
  pid_config_t speed_pid_config;
  pid_config_t position_pid_config;
  pid_config_t clock_pid_config;
  motor_pid_t speed_pid;
  motor_pid_t position_pid;
  motor_pid_t clock_pid;

  /* Perifericos y sub-contextos */
  pcnt_unit_handle_t encoder_unit;
  pot_adc_driver_t pot_driver;
  pot_state_t pot_state;
  volatile bool pot_enabled;
  volatile bool pot_force_sync;
  ui_ec11_ctx_t ec11;
  lcd_status_snapshot_t lcd_status_snapshot;

  /* Estado de persistencia NVS */
  nvs_stored_config_t nvs_last_saved_config;
  volatile bool nvs_storage_ready;
  volatile bool nvs_config_dirty;
  volatile bool nvs_autosave_enabled;
  volatile uint32_t nvs_save_count;
} app_context_t;

/**
 * @brief Inicializa los valores por defecto y spinlocks de app_context_t.
 */
void app_context_init_defaults(app_context_t *app);

/**
 * @brief Publica un evento hacia la FSM de control del motor.
 */
bool app_post_motor_event(app_context_t *app, motor_event_type_t type,
                          float value, bool direction);

/**
 * @brief Publica un mensaje de evento textual hacia serial_task.
 */
void app_post_log_event(app_context_t *app, const char *text);

/**
 * @brief Publica una solicitud hacia nvs_task.
 */
bool app_post_nvs_request(app_context_t *app, nvs_op_type_t type);

/**
 * @brief Publica una solicitud hacia display_task.
 */
bool app_post_lcd_request(app_context_t *app, lcd_req_type_t type, bool flag);

/**
 * @brief Marca configuracion modificada y programa autosave si esta habilitado.
 */
void app_notify_config_modified(app_context_t *app);

void motor_control_apply_speed_pid(app_context_t *app);
void motor_control_apply_position_pid(app_context_t *app);
void motor_control_apply_clock_pid(app_context_t *app);
void motor_control_reset_pids_to_defaults(app_context_t *app);

/**
 * @brief Procesa un evento de la FSM del motor.
 */
void motor_control_handle_event(app_context_t *app, const motor_event_t *event);

/**
 * @brief Ejecuta un paso del lazo de control, protecciones de seguridad y telemetria.
 */
void motor_control_step(app_context_t *app, int counts, float sample_time,
                        uint32_t *movement_start_ms, uint32_t *no_motion_ms,
                        uint32_t *telemetry_elapsed_ms,
                        uint32_t *pos_log_elapsed_ms);

/**
 * @brief Ejecuta una secuencia de pulso por ciclos deshabilitando el PID durante el pulso.
 */
void motor_control_run_pulse(app_context_t *app, bool pulse_forward,
                             uint32_t cycles, uint8_t duty,
                             uint32_t pwm_frequency_hz);

/**
 * @brief Captura la configuracion actual en una estructura serializable para NVS.
 */
void motor_control_capture_nvs_config(app_context_t *app,
                                      nvs_stored_config_t *cfg);

/**
 * @brief Aplica una configuracion NVS validada sobre los parametros de runtime.
 */
void motor_control_apply_nvs_config(app_context_t *app,
                                    const nvs_stored_config_t *cfg);

/**
 * @brief Inicializa Flash NVS y carga la configuracion inicial o defaults.
 */
void motor_control_nvs_init_and_load(app_context_t *app);

/**
 * @brief Ejecuta guardado inmediato, carga, reset o autoguardado diferido para nvs_task.
 */
void motor_control_nvs_save_now(app_context_t *app);
void motor_control_nvs_load_now(app_context_t *app);
void motor_control_nvs_reset_defaults(app_context_t *app);
void motor_control_nvs_autosave_if_dirty(app_context_t *app);

/**
 * @brief Captura un snapshot consistente para renderizado de LCD.
 */
void motor_control_capture_display_telemetry(app_context_t *app,
                                             display_telemetry_snapshot_t *snap);

/**
 * @brief Devuelve el codigo de 1 caracter asociado al estado de la FSM.
 */
char motor_control_state_code(motor_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
