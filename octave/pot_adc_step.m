function [pot_state, mapped, updated_mask, raw_avg_vec] = pot_adc_step(pot_state, v_true, mode_str, cfg, noise_matrix)
% POT_ADC_STEP Simula lectura ADC 12-bit, sobremuestreo x8, filtro EMA y deadband.
%
% Replica exactamente lib/pot_adc/pot_adc.c:
%   Canales: 1=KP, 2=KI, 3=KD, 4=SP (tension 0..3.3 V -> 0..4095 LSB).
%
% Entradas:
%   pot_state    - Estado interno (vacio [] en la primera llamada)
%   v_true       - Vector 1x4 de tensiones fisicas en cursores [KP, KI, KD, SP] (V)
%   mode_str     - 'SPEED', 'POS', 'CLOCK' u 'OPEN'
%   cfg          - Estructura de pidfra_params()
%   noise_matrix - (Opcional) Matriz 4xM de ruido ADC en LSB para reproducibilidad

  pcfg = cfg.pot;
  M = pcfg.multisample;

  if isempty(pot_state)
    pot_state = struct( ...
      'raw_filtered',      zeros(1, 4), ...
      'norm',              zeros(1, 4), ...
      'voltage_v',         zeros(1, 4), ...
      'last_applied_norm', zeros(1, 4), ...
      'initialized',       false, ...
      'last_mode',         '' ...
    );
  end

  if nargin < 5 || isempty(noise_matrix)
    noise_matrix = pcfg.noise_std_lsb * randn(4, M);
  end

  raw_avg_vec = zeros(1, 4);
  for ch = 1:4
    v_ch = min(max(double(v_true(ch)), pcfg.v_min), pcfg.v_max);
    ideal_raw = (v_ch / pcfg.v_max) * pcfg.adc_raw_max;
    samples = round(ideal_raw + noise_matrix(ch, 1:M));
    samples = min(max(samples, pcfg.adc_raw_min), pcfg.adc_raw_max);
    raw_avg = mean(samples);
    raw_avg_vec(ch) = raw_avg;

    if ~pot_state.initialized
      pot_state.raw_filtered(ch) = raw_avg;
    else
      pot_state.raw_filtered(ch) = pot_state.raw_filtered(ch) + ...
        pcfg.ema_alpha * (raw_avg - pot_state.raw_filtered(ch));
    end

    % Normalizacion y snap en extremos (pot_normalize_raw)
    span = pcfg.adc_raw_max - pcfg.adc_raw_min;
    n_val = (pot_state.raw_filtered(ch) - pcfg.adc_raw_min) / span;
    n_val = min(max(n_val, 0.0), 1.0);
    if n_val < pcfg.zero_snap_norm
      n_val = 0.0;
    elseif n_val > (1.0 - pcfg.zero_snap_norm)
      n_val = 1.0;
    end

    pot_state.norm(ch)      = n_val;
    pot_state.voltage_v(ch) = pcfg.v_min + n_val * (pcfg.v_max - pcfg.v_min);
  end

  % Rangos segun modo activo (pot_get_pid_ranges + pot_adc_map_for_mode)
  mode_u = upper(strtrim(mode_str));
  if strcmp(mode_u, 'CLOCK')
    kp_r = pcfg.clock_kp_range;
    ki_r = pcfg.clock_ki_range;
    kd_r = pcfg.clock_kd_range;
    sp_r = [360.0 / cfg.clock_divisions_per_rev, 360.0 / cfg.clock_divisions_per_rev];
  elseif strcmp(mode_u, 'POS') || strcmp(mode_u, 'POSITION')
    kp_r = pcfg.pos_kp_range;
    ki_r = pcfg.pos_ki_range;
    kd_r = pcfg.pos_kd_range;
    sp_r = pcfg.pos_sp_range;
  elseif strcmp(mode_u, 'OPEN') || strcmp(mode_u, 'OPEN_LOOP')
    kp_r = pcfg.speed_kp_range;
    ki_r = pcfg.speed_ki_range;
    kd_r = pcfg.speed_kd_range;
    sp_r = pcfg.open_sp_range;
  else
    kp_r = pcfg.speed_kp_range;
    ki_r = pcfg.speed_ki_range;
    kd_r = pcfg.speed_kd_range;
    sp_r = pcfg.speed_sp_range;
  end

  mapped = zeros(1, 4);
  mapped(1) = kp_r(1) + pot_state.norm(1) * (kp_r(2) - kp_r(1));
  mapped(2) = ki_r(1) + pot_state.norm(2) * (ki_r(2) - ki_r(1));
  mapped(3) = kd_r(1) + pot_state.norm(3) * (kd_r(2) - kd_r(1));
  if strcmp(mode_u, 'CLOCK')
    mapped(4) = 360.0 / cfg.clock_divisions_per_rev;
  else
    mapped(4) = sp_r(1) + pot_state.norm(4) * (sp_r(2) - sp_r(1));
  end

  updated_mask = false(1, 4);
  mode_changed = (~pot_state.initialized) || (~strcmp(pot_state.last_mode, mode_u));

  if mode_changed
    pot_state.last_applied_norm = pot_state.norm;
    pot_state.initialized = true;
    pot_state.last_mode = mode_u;
  else
    for ch = 1:4
      if strcmp(mode_u, 'CLOCK') && (ch == 4)
        pot_state.last_applied_norm(ch) = pot_state.norm(ch);
        continue;
      end
      if abs(pot_state.norm(ch) - pot_state.last_applied_norm(ch)) >= pcfg.deadband_norm
        pot_state.last_applied_norm(ch) = pot_state.norm(ch);
        updated_mask(ch) = true;
      end
    end
  end
end
