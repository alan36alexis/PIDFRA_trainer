function [pstate, delta_counts] = motor_plant_step(pstate, applied_duty, forward, released, cfg)
% MOTOR_PLANT_STEP Integra la fisica continua del motor DC + puente H + PCNT durante Ts.
%
% Entradas:
%   pstate       - Estado continuo del motor (.theta_deg, .rpm, .hw_counts)
%   applied_duty - Ciclo de trabajo entero PWM 8 bits [0..255] mantenido por ZOH
%   forward      - Sentido logico (true: IN1=1,IN2=0; false: IN1=0,IN2=1)
%   released     - Puente H desenergizado (IN1=0, IN2=0, ENA=0 -> frenado libre)
%   cfg          - Estructura de pidfra_params()
%
% Salidas:
%   pstate       - Estado fisico actualizado en t + Ts
%   delta_counts - Cuentas enteras acumuladas por PCNT en esta ventana Ts

  if isempty(pstate)
    pstate = struct( ...
      'theta_deg', 0.0, ...
      'rpm',       0.0, ...
      'hw_counts', 0 ...
    );
  end

  plant  = cfg.plant;
  n_sub  = max(1, round(cfg.Ts / plant.dt_phys));
  dt     = cfg.Ts / double(n_sub);

  duty_clamped = min(max(double(round(applied_duty)), 0.0), double(cfg.pwm_max_duty));
  if released || (duty_clamped <= 0.0)
    u_drive = 0.0;
  else
    if forward
      u_drive = duty_clamped;
    else
      u_drive = -duty_clamped;
    end
  end

  tau_m       = plant.tau_m;
  u_static    = plant.u_static;
  u_coulomb   = plant.u_coulomb;
  k_rpm       = plant.k_rpm_per_duty;
  omega_stick = plant.omega_stick_rpm;
  u_load      = plant.load_duty_equiv;

  theta_deg = pstate.theta_deg;
  rpm       = pstate.rpm;

  if plant.rotor_locked
    rpm = 0.0;
    pstate.rpm = 0.0;
    delta_counts = 0;
    return;
  end

  for s = 1:n_sub
    u_ext = u_drive - u_load;

    if abs(rpm) <= omega_stick
      % Regimen adherido (stick): debe superar friccion estatica u_static
      if abs(u_ext) > u_static
        u_net = u_ext - sign(u_ext) * u_coulomb;
        rpm = rpm + (dt / tau_m) * (k_rpm * u_net - rpm);
      else
        rpm = 0.0;
      end
    else
      % Regimen cinematica (slip): friccion seca de Coulomb + amortiguamiento viscoso
      u_net = u_ext - sign(rpm) * u_coulomb;
      % Solucion exacta de EDO lineal por tramo en paso dt para maxima estabilidad
      alpha_d = exp(-dt / tau_m);
      rpm_inf = k_rpm * u_net;
      rpm_next = alpha_d * rpm + (1.0 - alpha_d) * rpm_inf;

      % Captura en cero (stick) si cruza velocidad nula sin vencer u_static
      if (rpm * rpm_next <= 0.0) && (abs(u_ext) <= u_static)
        rpm = 0.0;
      else
        rpm = rpm_next;
      end
    end

    % 1 RPM = 360 deg / 60 s = 6 deg/s
    theta_deg = theta_deg + (rpm * 6.0) * dt;
  end

  pstate.theta_deg = theta_deg;
  pstate.rpm       = rpm;

  % Cuantizacion espacial en cuadratura x4 (PCNT Unidad 0)
  new_hw_counts = round(theta_deg * cfg.counts_per_rev / 360.0);
  if plant.encoder_open
    delta_counts = 0;
    pstate.hw_counts = new_hw_counts;
  else
    delta_counts = new_hw_counts - pstate.hw_counts;
    pstate.hw_counts = new_hw_counts;
  end
end
