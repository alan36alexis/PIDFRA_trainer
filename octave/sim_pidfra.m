% SIM_PIDFRA Punto de entrada principal de la simulacion en GNU Octave de PIDFRA_trainer.
%
% Modos de ejecucion:
%   1) Desde terminal (genera reporte completo y exporta graficas PNG en octave/output/):
%        octave --no-gui octave/sim_pidfra.m
%   2) Sin renderizar figuras (solo calculo numerico rapido):
%        octave --no-gui octave/sim_pidfra.m --no-plots
%   3) Menu interactivo por consola (para experimentar con parametros propios):
%        octave octave/sim_pidfra.m --interactive
%   4) Dentro de la GUI de GNU Octave:
%        run('octave/sim_pidfra.m')

sim_root_dir = fileparts(mfilename('fullpath'));
if isempty(sim_root_dir)
  sim_root_dir = pwd();
end
addpath(sim_root_dir);

% Parseo de argumentos de linea de comandos en GNU Octave
cli_args = {};
if exist('argv', 'builtin') || exist('argv', 'file')
  cli_args = argv();
end

opt_no_plots    = any(strcmp(cli_args, '--no-plots'));
opt_interactive = any(strcmp(cli_args, '--interactive'));
opt_save_png    = ~any(strcmp(cli_args, '--no-save'));
has_gui_running = false;
if exist('isguirunning', 'builtin') || exist('isguirunning', 'file')
  has_gui_running = isguirunning();
end
opt_show_figs = has_gui_running || any(strcmp(cli_args, '--show'));

cfg = pidfra_params();
ana = pidfra_discrete_analysis(cfg);
suite = pidfra_scenarios(cfg);

fprintf('\n==============================================================================\n');
fprintf('  SIMULACION DEL SISTEMA PIDFRA_TRAINER EN GNU OCTAVE\n');
fprintf('  ESP32 + Puente H L298N (PWM 8-bit) + Encoder 900 PPR x4 (%d cuentas/rev)\n', cfg.counts_per_rev);
fprintf('==============================================================================\n\n');

fprintf('--- 1. PARAMETROS DEL SISTEMA Y PLANTA DISCRETIZADA (Ts = %.1f ms, fs = %.0f Hz) ---\n', ...
        1000.0 * cfg.Ts, cfg.fs_hz);
fprintf('  Resolucion espacial encoder (q_theta) : %.3f deg/cuenta\n', ana.quantization.q_theta_deg);
fprintf('  Cuanto minimo velocidad (Metodo M)    : %.2f RPM/cuenta\n', ana.quantization.delta_rpm_min);
fprintf('  Polo mecanico lazo abierto z = exp(-Ts/tau_m) : %.4f (tau_m = %.0f ms)\n', ...
        ana.pole_open_motor, 1000.0 * ana.tau_m);
fprintf('  Cero intrinseco ZOH en posicion z_0 = -b0/b1  : %.4f\n', ana.Gp.sampling_zero);
fprintf('  Ganancia critica P velocidad (cruce z = -1)   : Kp_crit = %.2f (ringing desde Kp > %.2f)\n', ...
        ana.rlocus_speed_P.Kp_crit, ana.rlocus_speed_P.Kp_ringing);
fprintf('  Filtro EMA potenciometros (Tf=50ms, a=0.25)   : polo z=%.2f, tau_f=%.1f ms, fc=%.2f Hz\n\n', ...
        ana.freq.ema_pole_z, 1000.0 * ana.freq.ema_tau_s, ana.freq.ema_fc_hz);

fprintf('--- 2. ESTABILIDAD EN EL PLANO Z (LAZOS LINEALIZADOS CON PID DE FIRMWARE) ---\n');
fprintf('  Modo     | KP      KI      KD     | max|z_p| | Estable | Ringing | zeta_dom\n');
fprintf('  ---------+------------------------+----------+---------+---------+---------\n');
fprintf('  SPEED    | %6.4f  %6.4f  %6.4f |  %6.4f  |   %s    |   %s    |  %6.3f\n', ...
        cfg.speed_pid.kp, cfg.speed_pid.ki, cfg.speed_pid.kd, ...
        ana.loop_speed.spectral_rad, bool_str(ana.loop_speed.is_stable), ...
        bool_str(ana.loop_speed.has_ringing), ana.loop_speed.zeta_dom);
fprintf('  POSITION | %6.4f  %6.4f  %6.4f |  %6.4f  |   %s    |   %s    |  %6.3f\n', ...
        cfg.pos_pid.kp, cfg.pos_pid.ki, cfg.pos_pid.kd, ...
        ana.loop_pos.spectral_rad, bool_str(ana.loop_pos.is_stable), ...
        bool_str(ana.loop_pos.has_ringing), ana.loop_pos.zeta_dom);
fprintf('  CLOCK    | %6.4f  %6.4f  %6.4f |  %6.4f  |   %s    |   %s    |  %6.3f\n\n', ...
        cfg.clock_pid.kp, cfg.clock_pid.ki, cfg.clock_pid.kd, ...
        ana.loop_clock.spectral_rad, bool_str(ana.loop_clock.is_stable), ...
        bool_str(ana.loop_clock.has_ringing), ana.loop_clock.zeta_dom);

fprintf('--- 3. RESULTADOS DE SIMULACION HIBRIDA NO LINEAL (FIRMWARE EXACTO) ---\n');
mp = suite.pos.metrics_90;
fprintf('  [FSM_POSITION] Escalon ANG 90 deg (MINMOVE=%d, tol=+/-%d cuentas):\n', ...
        cfg.default_min_move_duty, cfg.position_tolerance_counts);
fprintf('    t_retardo = %.3f s | t_subida = %.3f s | Mp = %.2f%% | t_estab = %.3f s | e_ss = %+.2f deg\n', ...
        mp.t_delay_s, mp.t_rise_s, mp.overshoot_pct, mp.t_settle_s, mp.e_ss);
fprintf('    Comparativa sin MINMOVE=0   : angulo en t=3.5s = %.2f deg (error residual Stick-Slip)\n', ...
        suite.pos.step90_no_dz.theta_enc_deg(end));
fprintf('    Comparativa Windup (Ki=2.2) : pico Clamping ON = %.1f deg vs Clamping OFF = %.1f deg\n', ...
        max(suite.pos.step90_aw_on.theta_enc_deg), max(suite.pos.step90_aw_off.theta_enc_deg));

fprintf('  [FSM_CLOCK] Segundero 60 div (10 pasos de 6 deg = 600 cuentas en 10.5 s):\n');
fprintf('    Tiempo medio de establecimiento por paso : %.1f ms (ventana disponible: 1000 ms)\n', ...
        suite.clock.mean_settle_ms);
fprintf('    Error final en el paso 10 (60.0 deg)     : %.0f cuentas (%.1f deg)\n', ...
        suite.clock.final_error_counts, suite.clock.final_error_counts * cfg.deg_per_count);

fprintf('  [FSM_SPEED] Velocidad baja (8.0 RPM) y media (35.0 RPM + carga en t=1.8s):\n');
fprintf('    Regimen 8.0 RPM (EMA supervisora final)  : %.2f RPM\n', ...
        suite.speed.low_8rpm.rpm_safety_ema(end));
fprintf('    Regimen 35.0 RPM (Feedforward + PI)      : t_estab = %.3f s, RPM final tras carga = %.2f RPM\n', ...
        suite.speed.metrics_35_ff.t_settle_s, suite.speed.med_35_ff_tuned.rpm_true(end));

fprintf('  [FSM_OPEN_LOOP] Calibracion de umbral e identificacion experimental:\n');
fprintf('    Umbral estatico detectado (biseccion)    : duty = %d -> MINMOVE sugerido = %d (firmware = %d)\n', ...
        suite.open_cal.detected_threshold, suite.open_cal.recommended_minmove, cfg.default_min_move_duty);
fprintf('    Identificacion DUTY 225                  : Km_est = %.2f RPM/duty, tau_m_est = %.0f ms\n', ...
        suite.open_cal.Km_est_rpm_per_duty, 1000.0 * suite.open_cal.tau_m_est_s);

fprintf('  [FSM_FAULT] Protecciones automaticas de seguridad:\n');
fprintf('    1) Bloqueo de eje (STALL)                : corte en %.2f s (esperado: %.2f s = 300ms + 500ms)\n', ...
        suite.pot_and_faults.t_stall_trip_s, ...
        (cfg.encoder_startup_grace_ms + cfg.encoder_stall_timeout_ms) / 1000.0);
fprintf('    2) Exceso de recorrido sin SP (>1.5 rev) : corte exacto en %.2f vueltas\n\n', ...
        suite.pot_and_faults.revs_at_trip);

if ~opt_no_plots
  out_dir = fullfile(sim_root_dir, 'output');
  fprintf('Generando suite grafica de 6 paneles...\n');
  saved_files = pidfra_plot_suite(suite, ana, cfg, out_dir, opt_show_figs, opt_save_png);
  if ~isempty(saved_files)
    fprintf('Graficas exportadas en %s:\n', out_dir);
    for idx_f = 1:length(saved_files)
      fprintf('  - %s\n', saved_files{idx_f});
    end
  end
end

if opt_interactive
  pidfra_interactive_cli(cfg);
elseif opt_show_figs && (~has_gui_running)
  fprintf('\nPresione Enter para cerrar las ventanas graficas...\n');
  pause();
end
