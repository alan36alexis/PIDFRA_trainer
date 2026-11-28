function suite = pidfra_scenarios(cfg)
% PIDFRA_SCENARIOS Ejecuta todos los escenarios de simulacion del sistema PIDFRA_trainer.
%
% Cubre los 4 modos operativos (POSITION, CLOCK, SPEED, OPEN_LOOP), el efecto
% de no linealidades reales (stick-slip sin MINMOVE, integral windup sin clamping),
% calibracion automatica de umbral, filtro ADC+EMA y fallas de seguridad FSM.

  if nargin < 1 || isempty(cfg)
    cfg = pidfra_params();
  end

  suite = struct();

  % =========================================================================
  % 1. MODO POSICION: Nominal vs Sin MINMOVE (Stick-Slip) vs Sin Anti-Windup
  % =========================================================================
  scen_pos_seq = struct();
  scen_pos_seq.duration_s = 4.5;
  scen_pos_seq.events = {
    0.10, 'ANG',  90.0, true;
    1.60, 'ANG', 180.0, true;
    3.10, 'ANG',  45.0, true
  };
  res_pos_seq = pidfra_simulate(scen_pos_seq, cfg);

  % Metricas del primer escalon (0 -> 90 deg, ventana 0.10 s .. 1.55 s)
  idx_p1 = (res_pos_seq.t >= 0.10) & (res_pos_seq.t <= 1.55);
  tol_deg = cfg.position_tolerance_counts * cfg.deg_per_count; % 0.8 deg
  metrics_pos_nom = pidfra_step_metrics( ...
    res_pos_seq.t(idx_p1), res_pos_seq.theta_enc_deg(idx_p1), ...
    90.0, tol_deg, cfg.safety_settle_ticks);

  % Comparativa de no linealidades ante escalon unico de 90 grados (duracion 3.5 s)
  scen_pos_90 = struct('duration_s', 3.5, 'events', {{0.05, 'ANG', 90.0, true}});
  res_pos_90_nom = pidfra_simulate(scen_pos_90, cfg);

  % Caso B: Sin precompensacion de zona muerta (MINMOVE = 0) -> Stick-Slip / Hunting
  cfg_no_dz = cfg;
  cfg_no_dz.default_min_move_duty = 0;
  cfg_no_dz.enable_safety = false; % permitir observar el ciclo limite completo
  res_pos_90_nodz = pidfra_simulate(scen_pos_90, cfg_no_dz);

  % Caso C: Ganancia integral elevada con y sin Anti-Windup (Clamping) -> Windup
  cfg_aw_on = cfg;
  cfg_aw_on.pos_pid.ki = 2.20;
  cfg_aw_on.enable_antiwindup = true;
  cfg_aw_on.enable_safety = false;
  res_pos_aw_on = pidfra_simulate(scen_pos_90, cfg_aw_on);

  cfg_aw_off = cfg_aw_on;
  cfg_aw_off.enable_antiwindup = false;
  res_pos_aw_off = pidfra_simulate(scen_pos_90, cfg_aw_off);

  suite.pos = struct( ...
    'seq_nominal',   res_pos_seq, ...
    'step90_nom',    res_pos_90_nom, ...
    'step90_no_dz',  res_pos_90_nodz, ...
    'step90_aw_on',  res_pos_aw_on, ...
    'step90_aw_off', res_pos_aw_off, ...
    'metrics_90',    metrics_pos_nom ...
  );

  % =========================================================================
  % 2. MODO CLOCK: Aguja segundera (60 divisiones, +6 deg cada 1.0 s)
  % =========================================================================
  scen_clk = struct('duration_s', 10.5, 'events', {{0.00, 'MODE_CLOCK', 0, true}});
  res_clk  = pidfra_simulate(scen_clk, cfg);

  % Evaluar tiempo medio de establecimiento de los 10 pasos de 6 grados
  clk_step_settle = zeros(10, 1);
  for s = 1:10
    t_s0 = double(s);
    idx_s = (res_clk.t >= t_s0) & (res_clk.t < t_s0 + 0.98);
    m_s = pidfra_step_metrics(res_clk.t(idx_s), res_clk.theta_enc_deg(idx_s), ...
                              6.0 * double(s), tol_deg, 5);
    clk_step_settle(s) = m_s.t_settle_s;
  end

  suite.clock = struct( ...
    'sim',               res_clk, ...
    'step_settle_s',     clk_step_settle, ...
    'mean_settle_ms',    1000.0 * mean(clk_step_settle(~isnan(clk_step_settle))), ...
    'final_error_counts', abs(res_clk.target_counts(end) - res_clk.enc_counts(end)) ...
  );

  % =========================================================================
  % 3. MODO VELOCIDAD: Cuantizacion Metodo M (bajas RPM) y Rechazo de Carga
  % =========================================================================
  % 3a) Velocidad en rango de potenciometro (8.0 RPM <= POT_SPEED_SP_MAX=10 RPM)
  scen_spd_low = struct('duration_s', 3.0, 'events', {{0.05, 'SPEED', 8.0, true}});
  res_spd_low  = pidfra_simulate(scen_spd_low, cfg);

  % 3b) Velocidad media (35 RPM) con escalon de par de carga en t = 1.8 s:
  %     Comparando ley de piso de firmware vs precompensacion aditiva + PI sintonizado
  scen_spd_med = struct();
  scen_spd_med.duration_s = 3.6;
  scen_spd_med.events = {
    0.05, 'SPEED',     35.0, true;
    1.80, 'LOAD_DUTY', 14.0, true
  };

  cfg_spd_fw = cfg;
  cfg_spd_fw.enable_safety = false;
  res_spd_med_default = pidfra_simulate(scen_spd_med, cfg_spd_fw);

  cfg_spd_ff = cfg;
  cfg_spd_ff.speed_deadzone_mode = 'additive_ff';
  cfg_spd_ff.speed_pid.kp = 2.00;
  cfg_spd_ff.speed_pid.ki = 12.00;
  cfg_spd_ff.speed_pid.kd = 0.002;
  cfg_spd_ff.enable_safety = false;
  res_spd_med_ff = pidfra_simulate(scen_spd_med, cfg_spd_ff);

  idx_v = (res_spd_med_ff.t >= 0.05) & (res_spd_med_ff.t <= 1.75);
  metrics_spd_ff = pidfra_step_metrics( ...
    res_spd_med_ff.t(idx_v), res_spd_med_ff.rpm_true(idx_v), ...
    35.0, 0.05 * 35.0, 5);

  suite.speed = struct( ...
    'low_8rpm',        res_spd_low, ...
    'med_35_fw_floor', res_spd_med_default, ...
    'med_35_ff_tuned', res_spd_med_ff, ...
    'metrics_35_ff',   metrics_spd_ff ...
  );

  % =========================================================================
  % 4. MODO OPEN_LOOP: Curva Zona Muerta / Histeresis + Identificacion Km, tau_m
  % =========================================================================
  duties_up   = 0:5:255;
  duties_down = 255:-5:0;
  rpm_curve_up   = zeros(size(duties_up));
  rpm_curve_down = zeros(size(duties_down));

  cfg_ol = cfg;
  cfg_ol.enable_safety = false;

  % Barrido ascendente desde reposo (revela umbral estatico u_static)
  for i = 1:length(duties_up)
    ps = struct('theta_deg', 0.0, 'rpm', 0.0, 'hw_counts', 0);
    for step_k = 1:60 % 0.60 s por escalon
      [ps, ~] = motor_plant_step(ps, duties_up(i), true, duties_up(i) == 0, cfg_ol);
    end
    rpm_curve_up(i) = ps.rpm;
  end

  % Barrido descendente continuo (revela histeresis por friccion cinetica u_coulomb < u_static)
  ps_cont = struct('theta_deg', 0.0, 'rpm', 0.0, 'hw_counts', 0);
  for i = 1:length(duties_down)
    for step_k = 1:60
      [ps_cont, ~] = motor_plant_step(ps_cont, duties_down(i), true, duties_down(i) == 0, cfg_ol);
    end
    rpm_curve_down(i) = ps_cont.rpm;
  end

  % Emulacion del algoritmo scripts/find_open_loop_threshold.py (barrido + biseccion)
  found_coarse = NaN;
  prev_coarse  = 0;
  for d_test = 0:5:255
    ps_t = struct('theta_deg', 0.0, 'rpm', 0.0, 'hw_counts', 0);
    for step_k = 1:40 % pausa de 0.40 s
      [ps_t, ~] = motor_plant_step(ps_t, d_test, true, d_test == 0, cfg_ol);
    end
    deg_moved = double(ps_t.hw_counts) * cfg.deg_per_count;
    if abs(deg_moved) >= 1.0
      found_coarse = d_test;
      prev_coarse  = max(0, d_test - 5);
      break;
    end
  end
  lo_d = prev_coarse;
  hi_d = found_coarse;
  best_thresh = hi_d;
  while lo_d < hi_d
    mid_d = floor((lo_d + hi_d) / 2);
    ps_t = struct('theta_deg', 0.0, 'rpm', 0.0, 'hw_counts', 0);
    for step_k = 1:40
      [ps_t, ~] = motor_plant_step(ps_t, mid_d, true, mid_d == 0, cfg_ol);
    end
    if abs(double(ps_t.hw_counts) * cfg.deg_per_count) >= 1.0
      best_thresh = mid_d;
      hi_d = mid_d;
    else
      lo_d = mid_d + 1;
    end
  end

  % Escalon en lazo abierto (DUTY F 225) para identificacion experimental de Km y tau_m
  scen_id = struct('duration_s', 1.2, 'events', {{0.10, 'DUTY', 225, true}});
  res_id  = pidfra_simulate(scen_id, cfg_ol);
  idx_id  = res_id.t >= 0.10;
  t_id    = res_id.t(idx_id) - 0.10;
  w_id    = res_id.rpm_true(idx_id);
  w_inf   = mean(w_id(end-15:end));
  idx_63  = find(w_id >= 0.6321 * w_inf, 1, 'first');
  tau_est = t_id(idx_63);
  km_est  = w_inf / (225.0 - cfg.plant.u_coulomb);

  suite.open_cal = struct( ...
    'duties_up',            duties_up, ...
    'rpm_curve_up',         rpm_curve_up, ...
    'duties_down',          duties_down, ...
    'rpm_curve_down',       rpm_curve_down, ...
    'detected_threshold',   best_thresh, ...
    'recommended_minmove',  best_thresh + 6, ...
    'step_id',              res_id, ...
    'w_inf_rpm',            w_inf, ...
    'tau_m_est_s',          tau_est, ...
    'Km_est_rpm_per_duty',  km_est ...
  );

  % =========================================================================
  % 5. POTENCIOMETROS ADC (Sobremuestreo x8 + EMA + Deadband) Y FALLAS FSM
  % =========================================================================
  % Semilla determinista para reproducibilidad exacta
  randn('state', 42);
  Tf_pot = cfg.pot.Ts;
  N_pot  = round(6.0 / Tf_pot) + 1;
  t_pot  = (0:(N_pot - 1))' * Tf_pot;

  v_sp_true     = zeros(N_pot, 1);
  raw_1samp_sp  = zeros(N_pot, 1);
  raw_8avg_sp   = zeros(N_pot, 1);
  ema_sp_deg    = zeros(N_pot, 1);
  applied_sp_deg = zeros(N_pot, 1);
  pot_st = [];
  cur_applied_sp = 0.0;

  for k = 1:N_pot
    tk = t_pot(k);
    if tk < 1.0
      v_sp = 0.02;   % dentro de zero-snap -> debe clavarse en 0.0 deg
    elseif tk < 3.5
      v_sp = 1.65;   % mitad de carrera (180 deg)
    else
      v_sp = 2.475;  % 75% de carrera (270 deg)
    end
    v_sp_true(k) = v_sp;

    noise_mat = cfg.pot.noise_std_lsb * randn(4, cfg.pot.multisample);
    ideal_lsb = (v_sp / cfg.pot.v_max) * cfg.pot.adc_raw_max;
    raw_1_lsb = min(max(round(ideal_lsb + noise_mat(4, 1)), 0.0), cfg.pot.adc_raw_max);
    raw_1samp_sp(k) = (raw_1_lsb / cfg.pot.adc_raw_max) * 360.0;

    [pot_st, mapped, upd_mask, raw_avg_vec] = pot_adc_step( ...
      pot_st, [1.32, 0.66, 0.33, v_sp], 'POS', cfg, noise_mat);

    raw_8avg_sp(k) = (raw_avg_vec(4) / cfg.pot.adc_raw_max) * 360.0;
    ema_sp_deg(k)  = mapped(4);
    if (k == 1) || upd_mask(4)
      cur_applied_sp = mapped(4);
    end
    applied_sp_deg(k) = cur_applied_sp;
  end

  % Falla 1: Rotor bloqueado -> FAULT_ENCODER_STALL tras 300ms gracia + 500ms timeout = 800ms
  scen_stall = struct();
  scen_stall.duration_s = 1.5;
  scen_stall.events = {
    0.00, 'ROTOR_LOCK', 1,     true;
    0.10, 'ANG',        180.0, true
  };
  res_stall = pidfra_simulate(scen_stall, cfg);
  idx_stall_trip = find(res_stall.fsm_state == 5, 1, 'first');
  t_stall_trip_s = res_stall.t(idx_stall_trip) - 0.10;

  % Falla 2: Limite de seguridad de 1.5 vueltas (5400 cuentas) sin establecer SP
  scen_revs = struct();
  scen_revs.duration_s = 3.2;
  scen_revs.events = {
    0.10, 'DUTY', 240, true
  };
  res_revs = pidfra_simulate(scen_revs, cfg);
  idx_revs_trip = find(res_revs.fsm_state == 5, 1, 'first');
  revs_at_trip  = double(res_revs.enc_counts(idx_revs_trip)) / cfg.counts_per_rev;

  suite.pot_and_faults = struct( ...
    't_pot',          t_pot, ...
    'v_sp_true',      v_sp_true, ...
    'raw_1samp_deg',  raw_1samp_sp, ...
    'raw_8avg_deg',   raw_8avg_sp, ...
    'ema_sp_deg',     ema_sp_deg, ...
    'applied_sp_deg', applied_sp_deg, ...
    'fault_stall',    res_stall, ...
    't_stall_trip_s', t_stall_trip_s, ...
    'fault_revs',     res_revs, ...
    'revs_at_trip',   revs_at_trip ...
  );
end
