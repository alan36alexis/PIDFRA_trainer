#include "motor_pid.h"

#include <stdbool.h>
#include <stdint.h>

static float clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void motor_pid_init(motor_pid_t *pid, float kp, float ki, float kd,
                    float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    motor_pid_reset(pid);
}

void motor_pid_reset(motor_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->initialized = 0;
}

float motor_pid_update(motor_pid_t *pid, float setpoint, float measurement,
                       float sample_time_seconds)
{
    if (sample_time_seconds <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    float derivative = 0.0f;
    if (pid->initialized) {
        derivative = (error - pid->previous_error) / sample_time_seconds;
    } else {
        pid->initialized = 1;
    }

    float integral_candidate = pid->integral + error * sample_time_seconds;
    float output = pid->kp * error + pid->ki * integral_candidate
                   + pid->kd * derivative;

    if (output >= pid->output_max && error > 0.0f) {
        output = pid->output_max;
    } else if (output <= pid->output_min && error < 0.0f) {
        output = pid->output_min;
    } else {
        pid->integral = integral_candidate;
        output = clamp(output, pid->output_min, pid->output_max);
    }

    pid->previous_error = error;
    return output;
}

void motor_pid_run_cycles(int direction, uint32_t n_cycles, uint8_t duty,
                         uint32_t cycle_ms, uint32_t pwm_frequency_hz,
                         motor_pid_drive_cb drive_cb,
                         motor_pid_set_pwm_cb set_pwm_cb,
                         motor_pid_release_cb release_cb,
                         motor_pid_wait_ms_cb wait_ms_cb,
                         motor_pid_get_time_ms_cb time_ms_cb,
                         motor_pid_get_counts_cb counts_cb,
                         motor_pid_motion_started_cb motion_started_cb,
                         motor_pid_diag_cb diag_cb)
{
    if (n_cycles == 0U || drive_cb == NULL || release_cb == NULL
        || wait_ms_cb == NULL) {
        return;
    }

    if (pwm_frequency_hz > 0U && set_pwm_cb != NULL) {
        set_pwm_cb(pwm_frequency_hz);
    }

    bool motor_forward = direction == MOTOR_PID_DIRECTION_FORWARD;
    uint32_t effective_cycle_ms = cycle_ms;
    int32_t start_counts = 0;
    uint32_t ton_start_ms = 0U;
    int32_t delta_counts = 0;

    if (pwm_frequency_hz > 0U) {
        uint64_t total_pulse_time_us =
            ((uint64_t)n_cycles * 1000000ULL) / (uint64_t)pwm_frequency_hz;
        effective_cycle_ms = (uint32_t)((total_pulse_time_us + 999ULL) / 1000ULL);
        if (effective_cycle_ms == 0U) {
            effective_cycle_ms = 1U;
        }
    }

    if (time_ms_cb != NULL) {
        ton_start_ms = time_ms_cb();
    }
    if (counts_cb != NULL) {
        start_counts = counts_cb();
    }

    drive_cb(motor_forward, duty);

    if (motion_started_cb != NULL && time_ms_cb != NULL) {
        uint32_t time_to_motion_ms = effective_cycle_ms;
        bool motion_detected = false;
        while (true) {
            uint32_t elapsed_ms = time_ms_cb() - ton_start_ms;
            if (!motion_detected && motion_started_cb()) {
                motion_detected = true;
                time_to_motion_ms = elapsed_ms;
            }
            if (elapsed_ms >= effective_cycle_ms) {
                break;
            }
            wait_ms_cb(1U);
        }
        if (diag_cb != NULL) {
            diag_cb("DBG TON_TO_MOTION", time_to_motion_ms, 0);
        }
    } else if (effective_cycle_ms > 0U) {
        wait_ms_cb(effective_cycle_ms);
    }

    if (counts_cb != NULL) {
        delta_counts = counts_cb() - start_counts;
        if (diag_cb != NULL) {
            diag_cb("DBG PULSE_COUNTS", 0U, delta_counts);
        }
    }
    release_cb();
}
