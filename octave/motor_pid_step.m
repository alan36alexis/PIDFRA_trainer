function [output, pid] = motor_pid_step(pid, setpoint, measurement, sample_time_s)
% MOTOR_PID_STEP Calcula un paso del controlador PID discreto (Backward Euler).
%
% Replica fielmente motor_pid_update() en lib/motor_pid/motor_pid.c:
%   - Evita derivative kick en la primera muestra tras un reset (initialized).
%   - Integracion rectangular regresiva: I_cand = I + e * Ts.
%   - Anti-windup por integracion condicional (clamping) cuando la salida
%     excede [output_min, output_max] y el error empuja hacia la saturacion.

  if sample_time_s <= 0.0
    output = 0.0;
    return;
  end

  err = double(setpoint) - double(measurement);
  if pid.initialized
    derivative = (err - pid.previous_error) / sample_time_s;
  else
    derivative = 0.0;
    pid.initialized = true;
  end

  integral_candidate = pid.integral + err * sample_time_s;
  raw_output = pid.kp * err + pid.ki * integral_candidate + pid.kd * derivative;

  if pid.enable_antiwindup
    if (raw_output >= pid.output_max) && (err > 0.0)
      output = pid.output_max;
    elseif (raw_output <= pid.output_min) && (err < 0.0)
      output = pid.output_min;
    else
      pid.integral = integral_candidate;
      output = min(max(raw_output, pid.output_min), pid.output_max);
    end
  else
    % Modo sin anti-windup (util para demostrar enrollamiento del integrador)
    pid.integral = integral_candidate;
    output = min(max(raw_output, pid.output_min), pid.output_max);
  end

  pid.previous_error = err;
end
