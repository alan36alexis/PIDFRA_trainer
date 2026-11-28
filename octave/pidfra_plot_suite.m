function saved_files = pidfra_plot_suite(suite, ana, cfg, out_dir, show_figs, save_png)
% PIDFRA_PLOT_SUITE Genera y exporta las 6 figuras de analisis del sistema PIDFRA_trainer.
%
% Compatible con todos los graphics_toolkit de GNU Octave (qt, fltk, gnuplot)
% tanto en modo interactivo con ventana como en modo consola headless (--no-gui).

  if nargin < 4 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), 'output');
  end
  if nargin < 5 || isempty(show_figs)
    show_figs = true;
  end
  if nargin < 6 || isempty(save_png)
    save_png = true;
  end

  saved_files = {};
  has_display = (~isempty(getenv('DISPLAY'))) || (~isempty(getenv('WAYLAND_DISPLAY'))) || ispc() || ismac();
  vis_mode = 'on';
  if (~show_figs) || (~has_display)
    vis_mode = 'off';
  end

  configure_octave_graphics(has_display && show_figs);

  if save_png && (~exist(out_dir, 'dir'))
    mkdir(out_dir);
  end

  % =========================================================================
  % FIGURA 1: Control de Posicion y No Linealidades (Stick-Slip y Windup)
  % =========================================================================
  f1 = figure(1, 'Name', 'PIDFRA - Posicion y No Linealidades', ...
              'Visible', vis_mode, 'Position', [50, 50, 1280, 820]);
  clf(f1);

  subplot(2, 2, 1);
  r_seq = suite.pos.seq_nominal;
  h1 = plot(r_seq.t, r_seq.target_deg, 'k--', 'LineWidth', 1.3); hold on;
  h2 = plot(r_seq.t, r_seq.theta_true_deg, 'b-', 'LineWidth', 1.8);
  h3 = plot(r_seq.t, r_seq.theta_enc_deg, 'r:', 'LineWidth', 1.2);
  ylim([-5, 205]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Angulo [grados]');
  style_title('FSM_POSITION: Secuencia 90 -> 180 -> 45 deg');
  style_legend([h1, h2, h3], {'Referencia', 'Eje continuo', 'Encoder PCNT'}, 'northeast');

  subplot(2, 2, 2);
  h1 = stairs(r_seq.t, r_seq.signed_duty, 'Color', [0.10, 0.45, 0.80], 'LineWidth', 1.3); hold on;
  h2 = plot([r_seq.t(1), r_seq.t(end)], [ cfg.default_min_move_duty,  cfg.default_min_move_duty], 'r--');
  plot([r_seq.t(1), r_seq.t(end)], [-cfg.default_min_move_duty, -cfg.default_min_move_duty], 'r--');
  ylim([-280, 280]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Duty con signo [-255..255]');
  style_title('Esfuerzo ZOH + Precompensacion MINMOVE=185');
  style_legend([h1, h2], {'PWM aplicado', 'Umbral +/-MINMOVE'}, 'southwest');

  subplot(2, 2, 3);
  rn = suite.pos.step90_nom;
  rz = suite.pos.step90_no_dz;
  h1 = plot(rn.t, rn.target_deg, 'k--', 'LineWidth', 1.2); hold on;
  h2 = plot(rn.t, rn.theta_enc_deg, 'Color', [0.0, 0.55, 0.25], 'LineWidth', 1.8);
  h3 = plot(rz.t, rz.theta_enc_deg, 'Color', [0.85, 0.20, 0.15], 'LineWidth', 1.6);
  ylim([0, 105]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Angulo [grados]');
  style_title('Zona Muerta: MINMOVE=185 vs MINMOVE=0');
  style_legend([h1, h2, h3], {'Consigna 90 deg', 'Con MINMOVE=185', 'Sin MINMOVE=0'}, 'southeast');

  subplot(2, 2, 4);
  ra_on  = suite.pos.step90_aw_on;
  ra_off = suite.pos.step90_aw_off;
  h1 = plot(ra_on.t, ra_on.target_deg, 'k--', 'LineWidth', 1.2); hold on;
  h2 = plot(ra_on.t, ra_on.theta_enc_deg, 'b-', 'LineWidth', 1.8);
  h3 = plot(ra_off.t, ra_off.theta_enc_deg, 'm-', 'LineWidth', 1.6);
  ylim([0, 170]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Angulo [grados]');
  style_title('Anti-Windup (Ki=2.2): Clamping ON vs OFF');
  style_legend([h1, h2, h3], {'Consigna 90 deg', 'Clamping ON', 'Clamping OFF (Windup)'}, 'northeast');

  saved_files = maybe_export_png(f1, out_dir, '01_posicion_y_nolinealidades.png', save_png, saved_files);

  % =========================================================================
  % FIGURA 2: Modo CLOCK (Aguja Segundero 60 divisiones, 6 deg/s)
  % =========================================================================
  f2 = figure(2, 'Name', 'PIDFRA - Modo CLOCK Segundero', ...
              'Visible', vis_mode, 'Position', [80, 80, 1280, 780]);
  clf(f2);
  rc = suite.clock.sim;

  subplot(2, 2, [1, 3]);
  draw_clock_dial(rc, cfg);

  subplot(2, 2, 2);
  h1 = stairs(rc.t, rc.target_deg, 'k--', 'LineWidth', 1.3); hold on;
  h2 = plot(rc.t, rc.theta_enc_deg, 'Color', [0.05, 0.45, 0.85], 'LineWidth', 1.7);
  ylim([0, 68]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Angulo acumulado [deg]');
  style_title(sprintf('Escalonado CLOCK 6 deg/s (ts medio = %.0f ms)', suite.clock.mean_settle_ms));
  style_legend([h1, h2], {'Escalera 6 deg/s', 'Aguja encoder'}, 'northwest');

  subplot(2, 2, 4);
  idx_zoom = rc.t <= 4.2;
  yy_err = rc.target_deg(idx_zoom) - rc.theta_enc_deg(idx_zoom);
  h1 = plot(rc.t(idx_zoom), yy_err, 'Color', [0.80, 0.25, 0.10], 'LineWidth', 1.5); hold on;
  h2 = stairs(rc.t(idx_zoom), rc.signed_duty(idx_zoom) / 25.0, 'Color', [0.20, 0.60, 0.30], 'LineWidth', 1.2);
  h3 = plot([0, 4.2], [ 0.8,  0.8], 'k:');
  plot([0, 4.2], [-0.8, -0.8], 'k:');
  ylim([-11, 14]);
  grid on;
  xlabel('Tiempo [s] (primeros 4 pasos)');
  ylabel('Error [deg] | PWM/25');
  style_title('Pulsos PWM y Reset Integral en Banda (+/-0.8 deg)');
  style_legend([h1, h2, h3], {'Error [deg]', 'PWM / 25', 'Banda +/-0.8 deg'}, 'northeast');

  saved_files = maybe_export_png(f2, out_dir, '02_modo_clock_segundero.png', save_png, saved_files);

  % =========================================================================
  % FIGURA 3: Control de Velocidad, Cuantizacion Metodo M y Perturbacion
  % =========================================================================
  f3 = figure(3, 'Name', 'PIDFRA - Control de Velocidad y Cuantizacion', ...
              'Visible', vis_mode, 'Position', [110, 90, 1280, 820]);
  clf(f3);

  subplot(2, 2, 1);
  rl = suite.speed.low_8rpm;
  h1 = plot(rl.t, rl.target_rpm, 'k--', 'LineWidth', 1.3); hold on;
  h2 = stairs(rl.t, rl.rpm_meas_abs, 'Color', [0.85, 0.55, 0.15], 'LineWidth', 0.9);
  h3 = plot(rl.t, rl.rpm_true, 'b-', 'LineWidth', 1.6);
  h4 = plot(rl.t, rl.rpm_safety_ema, 'Color', [0.10, 0.65, 0.25], 'LineWidth', 1.6);
  ylim([0, 14]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Velocidad [RPM]');
  style_title('Baja Velocidad (8 RPM): Metodo M (1.67 RPM/cnt)');
  style_legend([h1, h2, h3, h4], {'SP 8 RPM', 'PCNT cruda', 'RPM real', 'EMA seg.'}, 'southeast');

  subplot(2, 2, 2);
  stairs(rl.t, rl.delta_counts, 'Color', [0.35, 0.25, 0.70], 'LineWidth', 1.1);
  ylim([0, 8]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Cuentas / Ts [entero]');
  style_title('Pulsos Enteros Delta c[k] cada Ts=10ms a 8 RPM');

  subplot(2, 2, 3);
  rm_fw = suite.speed.med_35_fw_floor;
  rm_ff = suite.speed.med_35_ff_tuned;
  h1 = plot(rm_ff.t, rm_ff.target_rpm, 'k--', 'LineWidth', 1.3); hold on;
  h2 = plot(rm_fw.t, rm_fw.rpm_true, 'Color', [0.80, 0.25, 0.20], 'LineWidth', 1.6);
  h3 = plot(rm_ff.t, rm_ff.rpm_true, 'Color', [0.05, 0.50, 0.85], 'LineWidth', 1.8);
  h4 = plot([1.80, 1.80], [0, 44], 'k:');
  ylim([0, 44]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Velocidad real [RPM]');
  style_title('SP 35 RPM + Carga (t=1.8s): Piso vs Feedforward');
  style_legend([h1, h2, h3, h4], {'SP 35 RPM', 'Piso firmware', 'Feedforward + PI', 'Carga t=1.8s'}, 'southeast');

  subplot(2, 2, 4);
  h1 = stairs(rm_fw.t, rm_fw.applied_duty, 'Color', [0.80, 0.25, 0.20], 'LineWidth', 1.3); hold on;
  h2 = stairs(rm_ff.t, rm_ff.applied_duty, 'Color', [0.05, 0.50, 0.85], 'LineWidth', 1.4);
  ylim([0, 275]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Duty PWM [0..255]');
  style_title('Esfuerzo PWM u[k] ante Escalon y Carga');
  style_legend([h1, h2], {'Piso firmware', 'Feedforward + PI'}, 'southeast');

  saved_files = maybe_export_png(f3, out_dir, '03_control_velocidad_y_cuantizacion.png', save_png, saved_files);

  % =========================================================================
  % FIGURA 4: Calibracion de Zona Muerta e Identificacion en Lazo Abierto
  % =========================================================================
  f4 = figure(4, 'Name', 'PIDFRA - Lazo Abierto e Identificacion', ...
              'Visible', vis_mode, 'Position', [140, 110, 1280, 620]);
  clf(f4);
  oc = suite.open_cal;

  subplot(1, 2, 1);
  h1 = plot(oc.duties_up, oc.rpm_curve_up, 'b-o', 'LineWidth', 1.6, 'MarkerSize', 4); hold on;
  h2 = plot(oc.duties_down, oc.rpm_curve_down, 'm--s', 'LineWidth', 1.4, 'MarkerSize', 4);
  h3 = plot([oc.detected_threshold, oc.detected_threshold], [0, 70], 'r--', 'LineWidth', 1.5);
  h4 = plot([cfg.default_min_move_duty, cfg.default_min_move_duty], [0, 70], 'g-.', 'LineWidth', 1.5);
  xlim([0, 260]);
  ylim([0, 70]);
  grid on;
  xlabel('Duty PWM en Lazo Abierto [0..255]');
  ylabel('Velocidad estacionaria [RPM]');
  style_title(sprintf('Zona Muerta e Histeresis (Umbral=%d, MINMOVE=%d)', ...
              oc.detected_threshold, cfg.default_min_move_duty));
  style_legend([h1, h2, h3, h4], ...
               {'Ascendente (stiction)', 'Descendente (Coulomb)', 'Umbral detectado', 'MINMOVE=185'}, ...
               'northwest');

  subplot(1, 2, 2);
  rid = oc.step_id;
  h1 = plot(rid.t, rid.rpm_true, 'b-', 'LineWidth', 1.8); hold on;
  h2 = stairs(rid.t, rid.rpm_meas_abs, 'Color', [0.72, 0.72, 0.72], 'LineWidth', 0.9);
  y63 = 0.6321 * oc.w_inf_rpm;
  t63 = 0.10 + oc.tau_m_est_s;
  h3 = plot([0.10, rid.t(end)], [oc.w_inf_rpm, oc.w_inf_rpm], 'k--');
  plot([t63, t63], [0, y63], 'r--');
  h4 = plot(t63, y63, 'ro', 'MarkerSize', 7, 'LineWidth', 1.8);
  ylim([0, 50]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Velocidad [RPM]');
  style_title(sprintf('Identificacion DUTY 225: Km=%.2f, tau_m=%.0f ms', ...
              oc.Km_est_rpm_per_duty, 1000.0 * oc.tau_m_est_s));
  style_legend([h1, h2, h3, h4], ...
               {'RPM continua', 'RPM encoder', 'Regimen est.', '63.2% (tau_m)'}, ...
               'southeast');

  saved_files = maybe_export_png(f4, out_dir, '04_calibracion_umbral_y_lazo_abierto.png', save_png, saved_files);

  % =========================================================================
  % FIGURA 5: Filtrado ADC/EMA de Potenciometros y Supervisores FSM_FAULT
  % =========================================================================
  f5 = figure(5, 'Name', 'PIDFRA - Potenciometros ADC y Seguridad FSM', ...
              'Visible', vis_mode, 'Position', [170, 90, 1280, 820]);
  clf(f5);
  pf = suite.pot_and_faults;

  subplot(2, 2, [1, 2]);
  h1 = plot(pf.t_pot, pf.raw_1samp_deg, 'Color', [0.80, 0.80, 0.80], 'LineWidth', 0.9); hold on;
  h2 = plot(pf.t_pot, pf.raw_8avg_deg, 'Color', [0.35, 0.65, 0.90], 'LineWidth', 1.1);
  h3 = plot(pf.t_pot, pf.ema_sp_deg, 'b-', 'LineWidth', 1.7);
  h4 = stairs(pf.t_pot, pf.applied_sp_deg, 'r-', 'LineWidth', 1.8);
  ylim([-10, 310]);
  grid on;
  xlabel('Tiempo [s] (muestreo pot_task Tf = 50 ms)');
  ylabel('Setpoint mapeado [deg]');
  style_title('Acondicionamiento ADC1: 12-bit -> Media x8 -> EMA (alpha=0.25) -> Deadband 1%');
  style_legend([h1, h2, h3, h4], ...
               {'1 muestra cruda', 'Sobremuestreo x8', 'Salida EMA-IIR', 'SP aplicado'}, ...
               'northwest');

  subplot(2, 2, 3);
  rst = pf.fault_stall;
  h1 = stairs(rst.t, rst.applied_duty, 'b-', 'LineWidth', 1.6); hold on;
  h2 = stairs(rst.t, rst.fsm_state * 45.0, 'r--', 'LineWidth', 1.6);
  ylim([-10, 280]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Duty PWM | Estado x45');
  style_title(sprintf('Proteccion STALL (Corte en %.2f s)', pf.t_stall_trip_s));
  style_legend([h1, h2], {'Duty PWM', 'FSM x45 (FAULT=225)'}, 'northeast');

  subplot(2, 2, 4);
  rrv = pf.fault_revs;
  h1 = plot(rrv.t, double(rrv.enc_counts) / cfg.counts_per_rev, 'b-', 'LineWidth', 1.8); hold on;
  h2 = stairs(rrv.t, double(rrv.applied_duty) / 150.0, 'Color', [0.15, 0.65, 0.25], 'LineWidth', 1.4);
  h3 = plot([rrv.t(1), rrv.t(end)], [cfg.safety_max_search_revs, cfg.safety_max_search_revs], 'r--', 'LineWidth', 1.5);
  ylim([0, 2.0]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Vueltas | Duty/150');
  style_title(sprintf('Proteccion >1.5 Vueltas sin SP (Corte %.2f rev)', pf.revs_at_trip));
  style_legend([h1, h2, h3], {'Vueltas eje', 'Duty / 150', 'Limite 1.5 rev'}, 'northwest');

  saved_files = maybe_export_png(f5, out_dir, '05_potenciometros_adc_y_seguridad_fsm.png', save_png, saved_files);

  % =========================================================================
  % FIGURA 6: Teoria de Control Discreto — Plano Z, Root Locus y Fase ZOH/EMA
  % =========================================================================
  f6 = figure(6, 'Name', 'PIDFRA - Analisis en Plano Z y Frecuencia', ...
              'Visible', vis_mode, 'Position', [200, 90, 1280, 820]);
  clf(f6);

  subplot(2, 2, 1);
  h_uc  = draw_unit_circle_grid(); hold on;
  p_spd = ana.loop_speed.poles(:);
  p_pos = ana.loop_pos.poles(:);
  p_clk = ana.loop_clock.poles(:);
  h_spd = plot(real(p_spd), imag(p_spd), 'bs', 'MarkerSize', 7, 'LineWidth', 1.6);
  h_pos = plot(real(p_pos), imag(p_pos), 'rx', 'MarkerSize', 8, 'LineWidth', 1.8);
  h_clk = plot(real(p_clk), imag(p_clk), 'm^', 'MarkerSize', 7, 'LineWidth', 1.6);
  h_z0  = plot(real(ana.Gp.sampling_zero), 0.0, 'ko', 'MarkerSize', 7, 'LineWidth', 1.6);
  axis equal;
  xlim([-1.22, 1.22]);
  ylim([-1.22, 1.22]);
  grid on;
  xlabel('Re(z)');
  ylabel('Im(z)');
  style_title('Polos de Lazo Cerrado en el Circulo Unitario');
  style_legend([h_uc, h_spd, h_pos, h_clk, h_z0], ...
               {'|z|=1', 'Polos SPEED', 'Polos POS', 'Polos CLOCK', 'Cero ZOH'}, ...
               'southwest');

  subplot(2, 2, 2);
  draw_unit_circle_grid(); hold on;
  rl_p = ana.rlocus_pos.poles;
  for br = 1:size(rl_p, 2)
    plot(real(rl_p(:, br)), imag(rl_p(:, br)), '-', 'LineWidth', 1.4);
  end
  ol_p = ana.rlocus_pos.ol_poles(:);
  ol_z = ana.rlocus_pos.ol_zeros(:);
  h_olp = plot(real(ol_p), imag(ol_p), 'kx', 'MarkerSize', 8, 'LineWidth', 1.7);
  h_olz = plot(real(ol_z), imag(ol_z), 'ko', 'MarkerSize', 7, 'LineWidth', 1.5);
  axis equal;
  xlim([-1.35, 1.22]);
  ylim([-1.22, 1.22]);
  grid on;
  xlabel('Re(z)');
  ylabel('Im(z)');
  style_title('Lugar de las Raices Discreto (Lazo Posicion)');
  style_legend([h_olp, h_olz], {'Polos L.A.', 'Ceros L.A.'}, 'southwest');

  subplot(2, 2, 3);
  h1 = semilogx(ana.freq.f_hz, ana.freq.zoh_phase_deg, 'r-', 'LineWidth', 1.8); hold on;
  h2 = semilogx(ana.freq.f_pot_hz, ana.freq.ema_phase_deg, 'b--', 'LineWidth', 1.6);
  ylim([-95, 5]);
  grid on;
  xlabel('Frecuencia [Hz]');
  ylabel('Fase [grados]');
  style_title('Desfase Retenedor ZOH (Ts=10ms) y Filtro EMA');
  style_legend([h1, h2], {'ZOH (-90 deg en 50 Hz)', 'Filtro EMA pot.'}, 'southwest');

  subplot(2, 2, 4);
  h1 = semilogx(ana.freq.f_pot_hz, ana.freq.ema_mag_db, 'b-', 'LineWidth', 1.8); hold on;
  h2 = semilogx(ana.freq.f_hz, ana.freq.zoh_mag_db, 'r--', 'LineWidth', 1.5);
  h3 = plot([ana.freq.ema_fc_hz, ana.freq.ema_fc_hz], [-28, 3], 'k:');
  ylim([-28, 3]);
  grid on;
  xlabel('Frecuencia [Hz]');
  ylabel('Magnitud [dB]');
  style_title(sprintf('Magnitud EMA (fc=%.2f Hz, tau=%.0f ms) y ZOH', ...
              ana.freq.ema_fc_hz, 1000.0 * ana.freq.ema_tau_s));
  style_legend([h1, h2, h3], {'EMA (alpha=0.25)', 'ZOH sinc', 'Corte -3 dB'}, 'southwest');

  saved_files = maybe_export_png(f6, out_dir, '06_plano_z_y_respuesta_frecuencia.png', save_png, saved_files);
end

function configure_octave_graphics(interactive_display)
  warning('off', 'Octave:gnuplot-graphics');
  set(0, 'DefaultAxesFontSize', 9);
  set(0, 'DefaultTextFontSize', 9);
  if exist('available_graphics_toolkits', 'builtin') || exist('available_graphics_toolkits', 'file')
    tks = available_graphics_toolkits();
    if interactive_display
      if any(strcmp(tks, 'qt'))
        graphics_toolkit('qt');
      elseif any(strcmp(tks, 'fltk'))
        graphics_toolkit('fltk');
      end
    else
      if any(strcmp(tks, 'qt'))
        if isempty(getenv('QT_QPA_PLATFORM'))
          setenv('QT_QPA_PLATFORM', 'offscreen');
        end
        graphics_toolkit('qt');
      elseif any(strcmp(tks, 'gnuplot'))
        graphics_toolkit('gnuplot');
      end
    end
  end
end

function style_title(txt)
  title(txt, 'Interpreter', 'none', 'FontSize', 10);
end

function style_legend(handles, labels, loc)
  h_leg = legend(handles, labels, 'Location', loc);
  try
    set(h_leg, 'FontSize', 8, 'Interpreter', 'none');
  catch
  end
end

function h_circle = draw_unit_circle_grid()
  th = linspace(0.0, 2.0 * pi, 300);
  h_circle = plot(cos(th), sin(th), 'k-', 'LineWidth', 1.3); hold on;
  plot([-1.4, 1.4], [0.0, 0.0], 'k:');
  plot([0.0, 0.0], [-1.4, 1.4], 'k:');
  zeta_ref = 0.7;
  wn_ts = linspace(0.0, pi / sqrt(1.0 - zeta_ref^2), 150);
  s_ts_pos = -zeta_ref * wn_ts + 1i * wn_ts * sqrt(1.0 - zeta_ref^2);
  z_spi = exp(s_ts_pos);
  plot(real(z_spi),  imag(z_spi), ':', 'Color', [0.65, 0.65, 0.65]);
  plot(real(z_spi), -imag(z_spi), ':', 'Color', [0.65, 0.65, 0.65]);
end

function draw_clock_dial(rc, cfg)
  th = linspace(0.0, 2.0 * pi, 240);
  plot(cos(th), sin(th), 'k-', 'LineWidth', 1.6); hold on;
  plot(0.82 * cos(th), 0.82 * sin(th), ':', 'Color', [0.55, 0.55, 0.55]);

  for div = 0:(cfg.clock_divisions_per_rev - 1)
    ang_rad = (pi / 2.0) - (double(div) * 2.0 * pi / double(cfg.clock_divisions_per_rev));
    if mod(div, 5) == 0
      r_in = 0.88;
      lw   = 1.5;
    else
      r_in = 0.94;
      lw   = 0.8;
    end
    plot([r_in * cos(ang_rad), cos(ang_rad)], [r_in * sin(ang_rad), sin(ang_rad)], 'k-', 'LineWidth', lw);
    if mod(div, 5) == 0
      lbl_r = 1.13;
      text(lbl_r * cos(ang_rad), lbl_r * sin(ang_rad), sprintf('%ds', div), ...
           'HorizontalAlignment', 'center', 'FontSize', 8);
    end
  end

  for s = 0:10
    idx_s = find(rc.t >= (double(s) + 0.45), 1, 'first');
    if ~isempty(idx_s)
      deg_s = rc.theta_enc_deg(idx_s);
      a_rad = (pi / 2.0) - deg_s * (pi / 180.0);
      plot([0.0, 0.76 * cos(a_rad)], [0.0, 0.76 * sin(a_rad)], '-', ...
           'Color', [0.70, 0.82, 0.95], 'LineWidth', 1.1);
      plot(0.76 * cos(a_rad), 0.76 * sin(a_rad), 'bo', 'MarkerSize', 5);
    end
  end

  deg_end = rc.theta_enc_deg(end);
  a_end   = (pi / 2.0) - deg_end * (pi / 180.0);
  plot([0.0, 0.84 * cos(a_end)], [0.0, 0.84 * sin(a_end)], 'r-', 'LineWidth', 2.4);
  plot(0.0, 0.0, 'ko', 'MarkerSize', 6, 'LineWidth', 1.8);

  axis equal;
  axis off;
  xlim([-1.32, 1.32]);
  ylim([-1.32, 1.32]);
  style_title(sprintf('Cuadrante Segundero (Final: %.1f deg / %d cnt)', deg_end, rc.enc_counts(end)));
end

function saved_files = maybe_export_png(fig_handle, out_dir, filename, save_png, saved_files)
  if ~save_png
    return;
  end
  out_path = fullfile(out_dir, filename);
  set(fig_handle, 'PaperUnits', 'inches', 'PaperPosition', [0, 0, 14.0, 9.0]);
  try
    if strcmp(graphics_toolkit(fig_handle), 'gnuplot')
      print(fig_handle, out_path, '-dpngcairo', '-S1600,1020', '-FSans:7');
    else
      try
        print(fig_handle, out_path, '-dpng', '-r140');
      catch
        graphics_toolkit(fig_handle, 'gnuplot');
        print(fig_handle, out_path, '-dpngcairo', '-S1600,1020', '-FSans:7');
      end
    end
    saved_files{end + 1} = out_path;
  catch err
    fprintf('  [Aviso] No se pudo exportar PNG (%s): %s\n', filename, err.message);
  end
end
