function pid = motor_pid_init(kp, ki, kd, out_min, out_max, enable_antiwindup)
% MOTOR_PID_INIT Inicializa o reinicia una estructura PID discreta.
%
% Replica exactamente motor_pid_init() y motor_pid_reset() de
% lib/motor_pid/motor_pid.c.

  if nargin < 6
    enable_antiwindup = true;
  end

  pid = struct();
  pid.kp                = double(kp);
  pid.ki                = double(ki);
  pid.kd                = double(kd);
  pid.output_min        = double(out_min);
  pid.output_max        = double(out_max);
  pid.integral          = 0.0;
  pid.previous_error    = 0.0;
  pid.initialized       = false;
  pid.enable_antiwindup = logical(enable_antiwindup);
end
