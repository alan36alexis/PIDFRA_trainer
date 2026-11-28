function ana = pidfra_discrete_analysis(cfg)
% PIDFRA_DISCRETE_ANALYSIS Analisis lineal exacto en el dominio Z y respuesta en frecuencia.
%
% Implementa las ecuaciones de manual_teoria_control_discreto.md usando
% exclusivamente funciones nativas de GNU Octave (conv, roots, polyval, filter),
% sin depender del paquete externo `control`.
%
% Incluye:
%   - Discretizacion exacta con ZOH de G_v(s) (Tipo 0) y G_p(s) (Tipo 1).
%   - Cero intrinseco de muestreo z_0 = -b_0 / b_1 cercano a z = -1.
%   - Controlador PID discreto por Euler hacia atras C(z) = (q0*z^2 + q1*z + q2)/(z*(z-1)).
%   - Polos, ceros, radio espectral y amortiguamiento equivalente en lazo cerrado
%     para los perfiles SPEED, POS y CLOCK.
%   - Lugar de las Raices discreto mostrando entrada en region de ringing (Re(z)<0)
%     y perdida de estabilidad al cruzar z = -1.
%   - Erosion de fase del retenedor ZOH y diagrama de Bode del filtro IIR EMA.

  if nargin < 1 || isempty(cfg)
    cfg = pidfra_params();
  end

  Ts    = cfg.Ts;
  tau_m = cfg.plant.tau_m;
  Km_v  = cfg.plant.Km_rpm;
  Km_p  = cfg.plant.Km_counts; % position_pid y clock_pid operan en cuentas de encoder

  ana = struct();
  ana.Ts    = Ts;
  ana.fs_hz = 1.0 / Ts;
  ana.tau_m = tau_m;

  % --- 1. Modelos discretos exactos de planta con ZOH (§4.1 y §4.2) ---
  a = exp(-Ts / tau_m);
  ana.pole_open_motor = a;

  % Planta de Velocidad G_v(z) = Km_v*(1 - a) / (z - a)
  Gv_num = [Km_v * (1.0 - a)];
  Gv_den = [1.0, -a];
  ana.Gv = struct('num', Gv_num, 'den', Gv_den, ...
                  'poles', roots(Gv_den), 'zeros', [], ...
                  'dc_gain', polyval(Gv_num, 1.0) / polyval(Gv_den, 1.0));

  % Planta de Posicion G_p(z) = Km_p*(b1*z + b0) / ((z - 1)*(z - a))
  b1 = Ts - tau_m * (1.0 - a);
  b0 = tau_m * (1.0 - a) - a * Ts;
  Gp_num = Km_p * [b1, b0];
  Gp_den = conv([1.0, -1.0], [1.0, -a]);
  z0_sampling = -b0 / b1;
  ana.Gp = struct('num', Gp_num, 'den', Gp_den, ...
                  'b1', b1, 'b0', b0, ...
                  'sampling_zero', z0_sampling, ...
                  'poles', roots(Gp_den), 'zeros', roots(Gp_num));

  % --- 2. Lazos cerrados con los PID por defecto de firmware (§5.2) ---
  ana.loop_speed = build_closed_loop(cfg.speed_pid, Gv_num, Gv_den, Ts, 2.5);
  ana.loop_pos   = build_closed_loop(cfg.pos_pid,   Gp_num, Gp_den, Ts, 2.5);
  ana.loop_clock = build_closed_loop(cfg.clock_pid, Gp_num, Gp_den, Ts, 1.0);

  % --- 3. Lugar de las Raices discreto (§3.2, §3.3, §4.2) ---
  % 3a) Lazo proporcional de velocidad: 1 + Kp * G_v(z) = 0
  %     Estabilidad critica cuando z = -1 => 1 + (-a) + Kp_crit*Km_v*(1-a) = 0 (con z=-1: -1 - a + Kp*Km*(1-a) = 0)
  Kp_ringing = a / (Km_v * (1.0 - a));          % Polo cruza z = 0 hacia Re(z) < 0
  Kp_crit_v  = (1.0 + a) / (Km_v * (1.0 - a));  % Polo cruza z = -1 (inestable)
  kp_vec_v   = linspace(0.0, 1.15 * Kp_crit_v, 250);
  rl_poles_v = zeros(length(kp_vec_v), 1);
  for i = 1:length(kp_vec_v)
    den_i = poly_add_padded(Gv_den, kp_vec_v(i) * Gv_num);
    rl_poles_v(i) = roots(den_i);
  end
  ana.rlocus_speed_P = struct( ...
    'kp_vec',     kp_vec_v, ...
    'poles',      rl_poles_v, ...
    'Kp_ringing', Kp_ringing, ...
    'Kp_crit',    Kp_crit_v ...
  );

  % 3b) Lazo de posicion escalando ganancia global k_scale * C_pos(z) * G_p(z)
  Cpos_num = pid_num_coeffs(cfg.pos_pid.kp, cfg.pos_pid.ki, cfg.pos_pid.kd, Ts);
  Cpos_den = [1.0, -1.0, 0.0]; % z*(z - 1) = z^2 - z
  OL_pos_num = conv(Cpos_num, Gp_num);
  OL_pos_den = conv(Cpos_den, Gp_den);
  n_order_pos = length(OL_pos_den) - 1;
  k_scale_vec = logspace(-2, 1.65, 220);
  rl_poles_pos = zeros(length(k_scale_vec), n_order_pos);
  k_crit_pos = NaN;
  for i = 1:length(k_scale_vec)
    den_i = poly_add_padded(OL_pos_den, k_scale_vec(i) * OL_pos_num);
    r_i = sort_poles_consistent(roots(den_i));
    rl_poles_pos(i, :) = r_i(:).';
    if isnan(k_crit_pos) && any(abs(r_i) >= 1.0)
      k_crit_pos = k_scale_vec(i);
    end
  end
  ana.rlocus_pos = struct( ...
    'k_scale_vec', k_scale_vec, ...
    'poles',       rl_poles_pos, ...
    'ol_poles',    roots(OL_pos_den), ...
    'ol_zeros',    roots(OL_pos_num), ...
    'k_crit',      k_crit_pos ...
  );

  % --- 4. Respuesta en frecuencia: ZOH (§2.1) y filtro IIR EMA (§8.2) ---
  f_nyq = 0.5 / Ts; % 50 Hz
  f_hz  = logspace(log10(0.05), log10(f_nyq), 300)';
  w_rad = 2.0 * pi * f_hz;

  % ZOH normalizado por Ts: sinc(w*Ts/2) * exp(-j*w*Ts/2)
  half_wt = 0.5 * w_rad * Ts;
  zoh_mag_norm = abs(sin(half_wt) ./ half_wt);
  zoh_phase_deg = -180.0 * (f_hz / ana.fs_hz);

  % Filtro EMA de potenciometros (muestreado cada Tf = 50 ms, alpha = 0.25)
  Tf    = cfg.pot.Ts;
  alpha = cfg.pot.ema_alpha;
  tau_f = -Tf / log(1.0 - alpha);
  fc_ema_hz = 1.0 / (2.0 * pi * tau_f);
  f_pot_hz  = logspace(log10(0.02), log10(0.5 / Tf), 250)';
  z_pot     = exp(1i * 2.0 * pi * f_pot_hz * Tf);
  H_ema     = (alpha * z_pot) ./ (z_pot - (1.0 - alpha));

  ana.freq = struct( ...
    'f_hz',          f_hz, ...
    'f_nyquist_hz',  f_nyq, ...
    'zoh_mag_db',    20.0 * log10(max(zoh_mag_norm, 1e-12)), ...
    'zoh_phase_deg', zoh_phase_deg, ...
    'Tf_pot_s',      Tf, ...
    'ema_alpha',     alpha, ...
    'ema_pole_z',    1.0 - alpha, ...
    'ema_tau_s',     tau_f, ...
    'ema_fc_hz',     fc_ema_hz, ...
    'f_pot_hz',      f_pot_hz, ...
    'ema_mag_db',    20.0 * log10(abs(H_ema)), ...
    'ema_phase_deg', (180.0 / pi) * atan2(imag(H_ema), real(H_ema)) ...
  );

  % --- 5.Resolucion espacial y de velocidad (§6.1 y §6.2) ---
  ana.quantization = struct( ...
    'q_theta_deg',   360.0 / cfg.counts_per_rev, ...
    'delta_rpm_min', 60.0 / (cfg.counts_per_rev * Ts), ...
    'delta_pwm_pct', 100.0 / cfg.pwm_max_duty ...
  );
end

function q = pid_num_coeffs(kp, ki, kd, Ts)
  q0 = kp + ki * Ts + kd / Ts;
  q1 = -(kp + 2.0 * kd / Ts);
  q2 = kd / Ts;
  q  = [q0, q1, q2];
end

function p_sum = poly_add_padded(p1, p2)
  p1 = p1(:).';
  p2 = p2(:).';
  n = max(length(p1), length(p2));
  p1_pad = [zeros(1, n - length(p1)), p1];
  p2_pad = [zeros(1, n - length(p2)), p2];
  p_sum = p1_pad + p2_pad;
end

function loop_info = build_closed_loop(pid_cfg, G_num, G_den, Ts, sim_horizon_s)
  C_num = pid_num_coeffs(pid_cfg.kp, pid_cfg.ki, pid_cfg.kd, Ts);
  C_den = [1.0, -1.0, 0.0]; % z*(z - 1)

  OL_num = conv(C_num, G_num);
  OL_den = conv(C_den, G_den);

  CL_den = poly_add_padded(OL_den, OL_num);
  CL_num = [zeros(1, length(CL_den) - length(OL_num)), OL_num];

  % Normalizar por coeficiente principal
  lead = CL_den(1);
  CL_den = CL_den / lead;
  CL_num = CL_num / lead;

  cl_poles = roots(CL_den);
  % Ceros finitos no nulos del numerador
  idx_nz = find(abs(OL_num) > 1e-14, 1, 'first');
  if isempty(idx_nz) || (idx_nz >= length(OL_num))
    cl_zeros = [];
  else
    cl_zeros = roots(OL_num(idx_nz:end));
  end

  rho = max(abs(cl_poles));
  is_stable = (rho < 1.0 - 1e-12);
  has_ringing = any((real(cl_poles) < -1e-6) & (abs(cl_poles) > 0.05));

  % Equivalentes continuos s = ln(z)/Ts para el par dominante (mayor |z|)
  [~, idx_dom] = max(abs(cl_poles));
  z_dom = cl_poles(idx_dom);
  if abs(z_dom) > 1e-12
    s_dom = log(z_dom) / Ts;
    wn_dom = abs(s_dom);
    if wn_dom > 1e-12
      zeta_dom = -real(s_dom) / wn_dom;
    else
      zeta_dom = 1.0;
    end
  else
    s_dom = -Inf;
    wn_dom = Inf;
    zeta_dom = 1.0;
  end

  % Respuesta al escalon unitario lineal ideal (sin saturacion ni zona muerta)
  N_step = max(10, round(sim_horizon_s / Ts) + 1);
  t_step = (0:(N_step - 1))' * Ts;
  u_step = ones(N_step, 1);
  y_step = filter(CL_num, CL_den, u_step);
  metrics = pidfra_step_metrics(t_step, y_step, 1.0, 0.02, 5);

  loop_info = struct( ...
    'C_num',        C_num, ...
    'C_den',        C_den, ...
    'CL_num',       CL_num, ...
    'CL_den',       CL_den, ...
    'poles',        cl_poles, ...
    'zeros',        cl_zeros, ...
    'pid_zeros',    roots(C_num(find(abs(C_num) > 1e-14, 1, 'first'):end)), ...
    'spectral_rad', rho, ...
    'is_stable',    is_stable, ...
    'has_ringing',  has_ringing, ...
    'z_dominant',   z_dom, ...
    'wn_rad_s',     wn_dom, ...
    'zeta_dom',     zeta_dom, ...
    't_step',       t_step, ...
    'y_step',       y_step, ...
    'metrics_lin',  metrics ...
  );
end

function r_sorted = sort_poles_consistent(r)
  [~, idx] = sort(real(r) * 1000.0 + imag(r));
  r_sorted = r(idx);
end
