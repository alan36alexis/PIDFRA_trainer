function pidfra_interactive_cli(cfg)
% PIDFRA_INTERACTIVE_CLI Consola interactiva en Octave que emula la CLI UART del ESP32.
%
% Permite enviar comandos como en el puerto serie real (ANG 120, SPEED F 8,
% MODE CLOCK, DUTY F 210, PID POS KP 0.6, MINMOVE 185, RUN 3.0) y graficar al instante.

  if nargin < 1 || isempty(cfg)
    cfg = pidfra_params();
  end

  fprintf('\n=== MODO INTERACTIVO CLI (Emulador UART ESP32 en Octave) ===\n');
  fprintf('Comandos: ANG <deg> | POS <cnt> | SPEED <rpm> | CLOCK | DUTY <0..255>\n');
  fprintf('          PID <SPEED|POS|CLOCK> <KP|KI|KD> <val> | MINMOVE <0..255> | Q (salir)\n\n');

  pending_events = {};
  sim_dur = 3.0;

  while true
    line_in = input('PIDFRA> ', 's');
    if isempty(line_in)
      continue;
    end
    txt = strtrim(line_in);
    toks = strsplit(txt);
    op = upper(toks{1});

    if strcmp(op, 'Q') || strcmp(op, 'QUIT') || strcmp(op, 'EXIT')
      fprintf('Saliendo del modo interactivo.\n');
      break;
    elseif strcmp(op, 'ANG') && length(toks) >= 2
      deg = str2double(toks{2});
      scen = struct('duration_s', sim_dur, 'events', {{0.05, 'ANG', deg, true}});
      r = pidfra_simulate(scen, cfg);
      plot_quick_result(r, 'POS', sprintf('Respuesta ANG %.1f deg', deg));
    elseif strcmp(op, 'POS') && length(toks) >= 2
      cnt = str2double(toks{2});
      scen = struct('duration_s', sim_dur, 'events', {{0.05, 'POS', cnt, true}});
      r = pidfra_simulate(scen, cfg);
      plot_quick_result(r, 'POS', sprintf('Respuesta POS %.0f cuentas', cnt));
    elseif strcmp(op, 'CLOCK') || (strcmp(op, 'MODE') && length(toks) >= 2 && strcmpi(toks{2}, 'CLOCK'))
      scen = struct('duration_s', 6.5, 'events', {{0.0, 'MODE_CLOCK', 0, true}});
      r = pidfra_simulate(scen, cfg);
      plot_quick_result(r, 'POS', 'Modo CLOCK (6 deg/s)');
    elseif strcmp(op, 'SPEED') && length(toks) >= 2
      rpm = str2double(toks{end});
      scen = struct('duration_s', sim_dur, 'events', {{0.05, 'SPEED', rpm, true}});
      r = pidfra_simulate(scen, cfg);
      plot_quick_result(r, 'SPEED', sprintf('Respuesta SPEED %.1f RPM', rpm));
    elseif strcmp(op, 'DUTY') && length(toks) >= 2
      d = str2double(toks{end});
      cfg_tmp = cfg; cfg_tmp.enable_safety = false;
      scen = struct('duration_s', 2.0, 'events', {{0.05, 'DUTY', d, true}});
      r = pidfra_simulate(scen, cfg_tmp);
      plot_quick_result(r, 'SPEED', sprintf('Lazo Abierto DUTY %.0f', d));
    elseif strcmp(op, 'MINMOVE') && length(toks) >= 2
      cfg.default_min_move_duty = round(str2double(toks{2}));
      fprintf('OK MINMOVE %d\n', cfg.default_min_move_duty);
    elseif strcmp(op, 'PID') && length(toks) >= 4
      m_sel = upper(toks{2});
      k_sel = lower(toks{3});
      val   = str2double(toks{4});
      if strcmp(m_sel, 'POS')
        cfg.pos_pid.(k_sel) = val;
      elseif strcmp(m_sel, 'CLOCK')
        cfg.clock_pid.(k_sel) = val;
      else
        cfg.speed_pid.(k_sel) = val;
      end
      fprintf('OK PID %s %s = %.4f\n', m_sel, upper(k_sel), val);
    else
      fprintf('Comando no reconocido. Use ANG, POS, SPEED, CLOCK, DUTY, PID, MINMOVE o Q.\n');
    end
  end
end

function plot_quick_result(r, kind, ttl)
  figure(10, 'Name', 'PIDFRA - Simulacion Interactiva');
  clf(10);
  subplot(2, 1, 1);
  if strcmp(kind, 'POS')
    plot(r.t, r.target_deg, 'k--', 'LineWidth', 1.3); hold on;
    plot(r.t, r.theta_enc_deg, 'b-', 'LineWidth', 1.8);
    ylabel('Angulo [grados]');
    legend({'Referencia', 'Encoder'}, 'Location', 'southeast');
  else
    plot(r.t, r.target_rpm, 'k--', 'LineWidth', 1.3); hold on;
    plot(r.t, r.rpm_true, 'b-', 'LineWidth', 1.8);
    stairs(r.t, r.rpm_meas_abs, 'r:', 'LineWidth', 1.0);
    ylabel('Velocidad [RPM]');
    legend({'Referencia', 'RPM Real', 'RPM Encoder'}, 'Location', 'southeast');
  end
  grid on;
  title(ttl);

  subplot(2, 1, 2);
  stairs(r.t, r.signed_duty, 'Color', [0.1, 0.5, 0.25], 'LineWidth', 1.4);
  ylim([-270, 270]);
  grid on;
  xlabel('Tiempo [s]');
  ylabel('Duty PWM [-255..+255]');
  drawnow();
end
