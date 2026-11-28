function res = pidfra_simulate(scenario, cfg)
% PIDFRA_SIMULATE Ejecuta la simulacion hibrida completa (FSM + PID + Planta + PCNT).
%
% Replica el comportamiento exacto de control_task (100 Hz) y motor_control.c
% sobre el modelo continuo del motor DC con puente H L298N y encoder 3600 cpr.
%
% Estructura `scenario`:
%   .duration_s  - Duracion total de la simulacion en segundos (ej. 5.0)
%   .events      - Celda Nx4 de eventos programados: {t_sec, 'CMD', val, forward}
%                  Comandos soportados:
%                    'STOP'           : detiene motor y limpia fallas
%                    'MODE_SPEED'     : cambia a modo velocidad (resetea encoder)
%                    'MODE_POS'       : cambia a modo posicion (resetea encoder)
%                    'MODE_CLOCK'     : inicia modo segundero 6 deg/s
%                    'MODE_OPEN'      : cambia a lazo abierto (resetea encoder)
%                    'SPEED'          : consigna RPM y sentido (val=rpm, fwd=true/false)
%                    'ANG'            : consigna angular en grados (val=grados)
%                    'POS'            : consigna en cuentas de encoder (val=cuentas)
%                    'DUTY'           : duty manual 0..255 en lazo abierto
%                    'MINMOVE'        : actualiza min_move_duty (0..255)
%                    'PID_SPEED'      : val = [kp, ki, kd]
%                    'PID_POS'        : val = [kp, ki, kd]
%                    'PID_CLOCK'      : val = [kp, ki, kd]
%                    'LOAD_DUTY'      : par perturbador equivalente en unidades duty
%                    'ROTOR_LOCK'     : bloquea/libera eje (val ~= 0 bloquea)
%                    'ENCODER_OPEN'   : desconecta/conecta fases A/B del encoder

  if nargin < 2 || isempty(cfg)
    cfg = pidfra_params();
  end

  % Codigos numericos de estado FSM (include/app_types.h)
  FSM_IDLE      = 0;
  FSM_SPEED     = 1;
  FSM_POSITION  = 2;
  FSM_CLOCK     = 3;
  FSM_OPEN_LOOP = 4;
  FSM_FAULT     = 5;

  FAULT_NONE        = 0;
  FAULT_STALL       = 1;
  FAULT_REVS_LIMIT  = 2;

  Ts = cfg.Ts;
  N  = max(2, round(scenario.duration_s / Ts) + 1);
  t_vec = (0:(N - 1))' * Ts;

  % Inicializacion de los 3 controladores PID (motor_control.c)
  aw = cfg.enable_antiwindup;
  speed_pid = motor_pid_init(cfg.speed_pid.kp, cfg.speed_pid.ki, cfg.speed_pid.kd, ...
                             cfg.speed_pid.out_min, cfg.speed_pid.out_max, aw);
  pos_pid   = motor_pid_init(cfg.pos_pid.kp, cfg.pos_pid.ki, cfg.pos_pid.kd, ...
                             cfg.pos_pid.out_min, cfg.pos_pid.out_max, aw);
  clock_pid = motor_pid_init(cfg.clock_pid.kp, cfg.clock_pid.ki, cfg.clock_pid.kd, ...
                             cfg.clock_pid.out_min, cfg.clock_pid.out_max, aw);

  % Estado interno del contexto de control (app_context_t)
  motor_state            = FSM_IDLE;
  fault_reason           = FAULT_NONE;
  encoder_fault          = false;
  position_mode          = false;
  open_loop_mode         = false;
  clock_mode             = false;
  running                = false;
  forward                = true;
  min_move_duty          = double(cfg.default_min_move_duty);
  manual_duty            = 0;
  target_rpm             = 0.0;
  target_pos_counts      = 0.0;
  encoder_pos_counts     = 0;
  clock_base_counts      = 0;
  clock_step_count       = 0;
  clock_elapsed_ms       = 0;
  pos_in_target          = false;

  % Supervisores de seguridad (motor_control_step)
  safety_unsettled_counts = 0;
  safety_settle_ticks     = 0;
  safety_filtered_rpm     = 0.0;
  movement_start_ms       = -1;
  no_motion_ms            = 0;

  % Estado continuo de la planta
  pstate = [];
  [~, ~] = motor_plant_step(pstate, 0, true, true, cfg);
  pstate = struct('theta_deg', 0.0, 'rpm', 0.0, 'hw_counts', 0);
  theta_zero_deg = 0.0; % Referencia fisica correspondiente al ultimo clear de PCNT

  % Vectores de registro
  theta_true_deg   = zeros(N, 1);
  theta_rel_deg    = zeros(N, 1);
  theta_enc_deg    = zeros(N, 1);
  enc_counts_vec   = zeros(N, 1);
  delta_counts_vec = zeros(N, 1);
  target_deg_vec   = zeros(N, 1);
  target_cnt_vec   = zeros(N, 1);
  rpm_true_vec     = zeros(N, 1);
  rpm_meas_vec     = zeros(N, 1);
  rpm_ema_vec      = zeros(N, 1);
  target_rpm_vec   = zeros(N, 1);
  pid_out_vec      = zeros(N, 1);
  applied_duty_vec = zeros(N, 1);
  signed_duty_vec  = zeros(N, 1);
  pid_int_vec      = zeros(N, 1);
  fsm_state_vec    = zeros(N, 1);
  fault_vec        = zeros(N, 1);
  events_log       = {};

  has_events = isfield(scenario, 'events') && ~isempty(scenario.events);
  if has_events
    n_ev = size(scenario.events, 1);
    ev_fired = false(n_ev, 1);
  else
    n_ev = 0;
    ev_fired = [];
  end

  delta_counts_in = 0;

  for k = 1:N
    now_s  = t_vec(k);
    now_ms = round(now_s * 1000.0);

    % 1. Procesar eventos programados hasta el instante actual (cola FSM)
    for e = 1:n_ev
      if (~ev_fired(e)) && (now_s + 1e-9 >= scenario.events{e, 1})
        ev_fired(e) = true;
        cmd = upper(strtrim(scenario.events{e, 2}));
        val = scenario.events{e, 3};
        if size(scenario.events, 2) >= 4 && ~isempty(scenario.events{e, 4})
          ev_fwd = logical(scenario.events{e, 4});
        else
          ev_fwd = true;
        end

        % Perturbaciones fisicas externas sobre la planta
        if strcmp(cmd, 'LOAD_DUTY')
          cfg.plant.load_duty_equiv = double(val);
          events_log{end + 1} = sprintf('t=%.2fs: LOAD_DUTY=%.1f', now_s, cfg.plant.load_duty_equiv);
          continue;
        elseif strcmp(cmd, 'ROTOR_LOCK')
          cfg.plant.rotor_locked = (double(val) ~= 0.0);
          events_log{end + 1} = sprintf('t=%.2fs: ROTOR_LOCK=%d', now_s, double(cfg.plant.rotor_locked));
          continue;
        elseif strcmp(cmd, 'ENCODER_OPEN')
          cfg.plant.encoder_open = (double(val) ~= 0.0);
          events_log{end + 1} = sprintf('t=%.2fs: ENCODER_OPEN=%d', now_s, double(cfg.plant.encoder_open));
          continue;
        elseif strcmp(cmd, 'MINMOVE')
          min_move_duty = min(max(round(double(val)), 0.0), double(cfg.pwm_max_duty));
          events_log{end + 1} = sprintf('t=%.2fs: OK MINMOVE %d', now_s, min_move_duty);
          continue;
        elseif strcmp(cmd, 'PID_SPEED')
          speed_pid = motor_pid_init(val(1), val(2), val(3), 0.0, double(cfg.pwm_max_duty), aw);
          events_log{end + 1} = sprintf('t=%.2fs: OK PID SPEED [%.4f, %.4f, %.4f]', now_s, val(1), val(2), val(3));
          continue;
        elseif strcmp(cmd, 'PID_POS')
          pos_pid = motor_pid_init(val(1), val(2), val(3), -double(cfg.pwm_max_duty), double(cfg.pwm_max_duty), aw);
          events_log{end + 1} = sprintf('t=%.2fs: OK PID POS [%.4f, %.4f, %.4f]', now_s, val(1), val(2), val(3));
          continue;
        elseif strcmp(cmd, 'PID_CLOCK')
          clock_pid = motor_pid_init(val(1), val(2), val(3), -double(cfg.pwm_max_duty), double(cfg.pwm_max_duty), aw);
          events_log{end + 1} = sprintf('t=%.2fs: OK PID CLOCK [%.4f, %.4f, %.4f]', now_s, val(1), val(2), val(3));
          continue;
        end

        % Bloqueo de comandos si hay falla activa (salvo STOP)
        if encoder_fault && (~strcmp(cmd, 'STOP'))
          events_log{end + 1} = sprintf('t=%.2fs: ERR LOCK STOP (%s ignorado)', now_s, cmd);
          continue;
        end

        prev_state = motor_state;
        was_running = running;
        was_speed_mode = (~position_mode) && (~open_loop_mode) && (~clock_mode);
        was_pos_mode   = position_mode && (~open_loop_mode) && (~clock_mode);
        was_open_mode  = open_loop_mode && (~position_mode) && (~clock_mode);
        reset_pids     = true;
        clear_encoder  = false;

        if strcmp(cmd, 'STOP')
          target_rpm = 0.0;
          manual_duty = 0;
          running = false;
          encoder_fault = false;
          fault_reason = FAULT_NONE;
          open_loop_mode = false;
          clock_mode = false;
          clock_elapsed_ms = 0;
          pos_in_target = false;
          motor_state = FSM_IDLE;
          safety_unsettled_counts = 0;
          safety_settle_ticks = 0;
          safety_filtered_rpm = 0.0;
          events_log{end + 1} = sprintf('t=%.2fs: OK STOP', now_s);

        elseif strcmp(cmd, 'MODE_SPEED')
          position_mode = false; open_loop_mode = false; clock_mode = false;
          clock_elapsed_ms = 0; manual_duty = 0; running = false; pos_in_target = false;
          motor_state = FSM_IDLE;
          clear_encoder = true;
          safety_unsettled_counts = 0; safety_settle_ticks = 0; safety_filtered_rpm = 0.0;
          events_log{end + 1} = sprintf('t=%.2fs: OK MODE SPEED', now_s);

        elseif strcmp(cmd, 'MODE_POS')
          position_mode = true; open_loop_mode = false; clock_mode = false;
          clock_elapsed_ms = 0; manual_duty = 0; running = false; pos_in_target = false;
          motor_state = FSM_IDLE;
          clear_encoder = true;
          safety_unsettled_counts = 0; safety_settle_ticks = 0; safety_filtered_rpm = 0.0;
          events_log{end + 1} = sprintf('t=%.2fs: OK MODE POS', now_s);

        elseif strcmp(cmd, 'MODE_CLOCK')
          position_mode = false; open_loop_mode = false; clock_mode = true;
          manual_duty = 0; encoder_fault = false; fault_reason = FAULT_NONE;
          pos_in_target = false; clock_elapsed_ms = 0;
          clear_encoder = true;
          motor_state = FSM_CLOCK;
          running = true;
          safety_unsettled_counts = 0; safety_settle_ticks = 0; safety_filtered_rpm = 0.0;
          events_log{end + 1} = sprintf('t=%.2fs: OK MODE CLOCK', now_s);

        elseif strcmp(cmd, 'MODE_OPEN')
          position_mode = false; open_loop_mode = true; clock_mode = false;
          clock_elapsed_ms = 0; running = false; manual_duty = 0; pos_in_target = false;
          motor_state = FSM_IDLE;
          clear_encoder = true;
          safety_unsettled_counts = 0; safety_settle_ticks = 0; safety_filtered_rpm = 0.0;
          events_log{end + 1} = sprintf('t=%.2fs: OK MODE OPEN', now_s);

        elseif strcmp(cmd, 'SPEED')
          if ~was_speed_mode
            clear_encoder = true;
          end
          target_rpm = max(0.0, double(val));
          forward = ev_fwd;
          position_mode = false; open_loop_mode = false; clock_mode = false;
          clock_elapsed_ms = 0;
          if target_rpm > 0.0
            motor_state = FSM_SPEED; running = true;
          else
            motor_state = FSM_IDLE; running = false;
          end
          if (~was_speed_mode) || (prev_state == FSM_IDLE) || (~was_running)
            safety_unsettled_counts = 0; safety_settle_ticks = 0; safety_filtered_rpm = 0.0;
          else
            safety_settle_ticks = 0;
          end
          events_log{end + 1} = sprintf('t=%.2fs: OK SPEED %.1f RPM (fwd=%d)', now_s, target_rpm, double(forward));

        elseif strcmp(cmd, 'ANG') || strcmp(cmd, 'POS')
          if ~was_pos_mode
            clear_encoder = true;
          end
          if strcmp(cmd, 'ANG')
            target_pos_counts = double(val) * cfg.counts_per_rev / 360.0;
          else
            target_pos_counts = double(val);
          end
          pos_in_target = false;
          position_mode = true; open_loop_mode = false; clock_mode = false;
          clock_elapsed_ms = 0;
          motor_state = FSM_POSITION;
          running = true;
          if (~was_pos_mode) || (prev_state == FSM_IDLE) || (~was_running)
            safety_unsettled_counts = 0; safety_settle_ticks = 0; safety_filtered_rpm = 0.0;
          else
            safety_settle_ticks = 0;
          end
          events_log{end + 1} = sprintf('t=%.2fs: OK %s -> %.1f cnt (%.2f deg)', ...
            now_s, cmd, target_pos_counts, target_pos_counts * 360.0 / cfg.counts_per_rev);

        elseif strcmp(cmd, 'DUTY')
          if ~was_open_mode
            clear_encoder = true;
          end
          target_rpm = 0.0;
          forward = ev_fwd;
          position_mode = false; open_loop_mode = true; clock_mode = false;
          clock_elapsed_ms = 0;
          manual_duty = min(max(round(double(val)), 0.0), double(cfg.pwm_max_duty));
          if manual_duty > 0
            motor_state = FSM_OPEN_LOOP; running = true;
          else
            motor_state = FSM_IDLE; running = false;
          end
          if (~was_open_mode) || (prev_state == FSM_IDLE) || (~was_running)
            safety_unsettled_counts = 0; safety_settle_ticks = 0; safety_filtered_rpm = 0.0;
          else
            safety_settle_ticks = 0;
          end
          events_log{end + 1} = sprintf('t=%.2fs: OK DUTY %d (fwd=%d)', now_s, manual_duty, double(forward));
        end

        if clear_encoder
          encoder_pos_counts = 0;
          clock_base_counts = 0;
          clock_step_count = 0;
          if ~(strcmp(cmd, 'ANG') || strcmp(cmd, 'POS'))
            target_pos_counts = 0.0;
          end
          delta_counts_in = 0;
          theta_zero_deg = pstate.theta_deg;
          pstate.hw_counts = round(pstate.theta_deg * cfg.counts_per_rev / 360.0);
        end

        if reset_pids
          speed_pid = motor_pid_init(speed_pid.kp, speed_pid.ki, speed_pid.kd, ...
                                     speed_pid.output_min, speed_pid.output_max, aw);
          pos_pid   = motor_pid_init(pos_pid.kp, pos_pid.ki, pos_pid.kd, ...
                                     pos_pid.output_min, pos_pid.output_max, aw);
          clock_pid = motor_pid_init(clock_pid.kp, clock_pid.ki, clock_pid.kd, ...
                                     clock_pid.output_min, clock_pid.output_max, aw);
        end
      end
    end

    % 2. Lectura acumulada de PCNT y estimacion Metodo M de velocidad (motor_control_step)
    counts = delta_counts_in;
    encoder_pos_counts = encoder_pos_counts + counts;
    measured_rpm = (abs(double(counts)) * 60.0) / (cfg.counts_per_rev * Ts);

    applied_duty = 0;
    pid_raw_out  = 0.0;
    pid_int_cur  = 0.0;
    drive_fwd    = forward;
    released     = true;
    movement_expected = false;

    % 3. Evaluacion de lazo cerrado / abierto segun estado de la FSM
    if motor_state == FSM_POSITION
      pos_f = double(encoder_pos_counts);
      [pid_raw_out, pos_pid] = motor_pid_step(pos_pid, target_pos_counts, pos_f, Ts);
      pid_int_cur = pos_pid.integral;
      pos_err = abs(target_pos_counts - pos_f);
      drive_fwd = (pid_raw_out >= 0.0);

      if pos_err > cfg.position_tolerance_counts
        pos_in_target = false;
        abs_out = abs(pid_raw_out);
        if abs_out > 0.0
          applied_duty = floor(min(abs_out + min_move_duty, double(cfg.pwm_max_duty)));
          movement_expected = (applied_duty >= cfg.stall_min_duty);
          released = false;
        else
          applied_duty = 0;
          released = false;
        end
      else
        pos_in_target = true;
        applied_duty = 0;
        released = false;
      end

    elseif motor_state == FSM_CLOCK
      clock_elapsed_ms = clock_elapsed_ms + cfg.control_period_ms;
      if clock_elapsed_ms >= cfg.clock_step_period_ms
        while clock_elapsed_ms >= cfg.clock_step_period_ms
          clock_elapsed_ms = clock_elapsed_ms - cfg.clock_step_period_ms;
          clock_step_count = clock_step_count + 1;
        end
        target_pos_counts = double(clock_base_counts) + ...
          (double(clock_step_count) * double(cfg.counts_per_rev)) / double(cfg.clock_divisions_per_rev);
        pos_in_target = false;
        % Pre-carga previous_error para eliminar derivative kick en cada paso de 1 s
        clock_pid.previous_error = target_pos_counts - double(encoder_pos_counts);
      end

      pos_f = double(encoder_pos_counts);
      [pid_raw_out, clock_pid] = motor_pid_step(clock_pid, target_pos_counts, pos_f, Ts);
      pos_err = abs(target_pos_counts - pos_f);
      drive_fwd = (pid_raw_out >= 0.0);

      if pos_err > cfg.position_tolerance_counts
        pos_in_target = false;
        abs_out = abs(pid_raw_out);
        if abs_out > 0.0
          applied_duty = floor(min(abs_out + min_move_duty, double(cfg.pwm_max_duty)));
          movement_expected = (applied_duty >= cfg.stall_min_duty);
          released = false;
        else
          applied_duty = 0;
          released = false;
        end
      else
        if ~pos_in_target
          pos_in_target = true;
          % Reset del integrador al arribar al paso (reset_on_target = true)
          clock_pid = motor_pid_init(clock_pid.kp, clock_pid.ki, clock_pid.kd, ...
                                     clock_pid.output_min, clock_pid.output_max, aw);
        end
        applied_duty = 0;
        released = false;
      end
      pid_int_cur = clock_pid.integral;

    elseif motor_state == FSM_SPEED
      [pid_raw_out, speed_pid] = motor_pid_step(speed_pid, target_rpm, measured_rpm, Ts);
      pid_int_cur = speed_pid.integral;
      drive_fwd = forward;
      abs_duty = abs(pid_raw_out);
      if (abs(target_rpm) > 0.0) && (abs_duty > 0.0)
        if strcmp(cfg.speed_deadzone_mode, 'additive_ff')
          abs_duty = abs_duty + min_move_duty;
        else
          if abs_duty < min_move_duty
            abs_duty = min_move_duty;
          end
        end
        applied_duty = floor(min(abs_duty, double(cfg.pwm_max_duty)));
        movement_expected = (applied_duty >= cfg.stall_min_duty);
        released = false;
      else
        applied_duty = 0;
        released = false;
      end

    elseif motor_state == FSM_OPEN_LOOP
      applied_duty = manual_duty;
      drive_fwd = forward;
      released = (applied_duty == 0);
      movement_expected = false;

    else
      % FSM_IDLE o FSM_FAULT -> motor_hw_stop() (IN1=0, IN2=0, ENA=0)
      applied_duty = 0;
      released = true;
      movement_expected = false;
    end

    % 4. Supervisores de seguridad: limite de 1.5 vueltas sin SP y bloqueo (STALL)
    if cfg.enable_safety && (motor_state == FSM_SPEED || motor_state == FSM_POSITION || ...
                             motor_state == FSM_CLOCK || motor_state == FSM_OPEN_LOOP)
      max_search_counts = cfg.safety_max_search_revs * cfg.counts_per_rev;
      safety_filtered_rpm = safety_filtered_rpm + ...
        cfg.safety_speed_ema_alpha * (measured_rpm - safety_filtered_rpm);

      setpoint_reached = false;
      clock_lag_exceeded = false;

      if (motor_state == FSM_POSITION) || (motor_state == FSM_CLOCK)
        p_err = abs(target_pos_counts - double(encoder_pos_counts));
        setpoint_reached = (p_err <= cfg.position_tolerance_counts);
        if (motor_state == FSM_CLOCK) && (p_err >= max_search_counts)
          clock_lag_exceeded = true;
        end
      elseif motor_state == FSM_SPEED
        if target_rpm <= 0.0
          setpoint_reached = true;
        else
          rpm_per_cnt = 60.0 / (cfg.counts_per_rev * Ts);
          speed_tol = max(1.5 * rpm_per_cnt, cfg.safety_speed_tol_ratio * target_rpm);
          setpoint_reached = (abs(safety_filtered_rpm - target_rpm) <= speed_tol);
        end
      else
        setpoint_reached = false;
      end

      if setpoint_reached
        if safety_settle_ticks < cfg.safety_settle_ticks
          safety_settle_ticks = safety_settle_ticks + 1;
        end
        if safety_settle_ticks >= cfg.safety_settle_ticks
          safety_unsettled_counts = 0;
        end
      else
        safety_settle_ticks = 0;
        safety_unsettled_counts = safety_unsettled_counts + abs(counts);
      end

      if (safety_unsettled_counts >= max_search_counts) || clock_lag_exceeded
        traveled_revs = double(safety_unsettled_counts) / cfg.counts_per_rev;
        encoder_fault = true;
        fault_reason = FAULT_REVS_LIMIT;
        motor_state = FSM_FAULT;
        running = false;
        applied_duty = 0;
        released = true;
        movement_expected = false;
        movement_start_ms = -1;
        no_motion_ms = 0;
        events_log{end + 1} = sprintf( ...
          't=%.2fs: EVT SAFETY STOP: NO SP EN 1.5 VUELTAS (REV=%.2f); LOCK STOP', ...
          now_s, traveled_revs);
      end
    end

    if cfg.enable_safety && movement_expected && (motor_state ~= FSM_FAULT)
      if movement_start_ms < 0
        movement_start_ms = now_ms;
      end
      if abs(counts) > 0
        no_motion_ms = 0;
      elseif (now_ms - movement_start_ms) >= cfg.encoder_startup_grace_ms
        no_motion_ms = no_motion_ms + cfg.control_period_ms;
      end
      if no_motion_ms >= cfg.encoder_stall_timeout_ms
        encoder_fault = true;
        fault_reason = FAULT_STALL;
        motor_state = FSM_FAULT;
        running = false;
        applied_duty = 0;
        released = true;
        movement_start_ms = -1;
        no_motion_ms = 0;
        events_log{end + 1} = sprintf('t=%.2fs: EVT STALL ENC/MOT; LOCK STOP', now_s);
      end
    else
      movement_start_ms = -1;
      no_motion_ms = 0;
    end

    % 5. Guardar instantanea del instante t[k]
    theta_true_deg(k)   = pstate.theta_deg;
    theta_rel_deg(k)    = pstate.theta_deg - theta_zero_deg;
    theta_enc_deg(k)    = double(encoder_pos_counts) * 360.0 / cfg.counts_per_rev;
    enc_counts_vec(k)   = encoder_pos_counts;
    delta_counts_vec(k) = counts;
    target_cnt_vec(k)   = target_pos_counts;
    target_deg_vec(k)   = target_pos_counts * 360.0 / cfg.counts_per_rev;
    rpm_true_vec(k)     = pstate.rpm;
    if counts >= 0
      rpm_meas_vec(k)   = measured_rpm;
    else
      rpm_meas_vec(k)   = -measured_rpm;
    end
    rpm_ema_vec(k)      = safety_filtered_rpm;
    target_rpm_vec(k)   = target_rpm;
    pid_out_vec(k)      = pid_raw_out;
    applied_duty_vec(k) = applied_duty;
    if drive_fwd
      signed_duty_vec(k) = double(applied_duty);
    else
      signed_duty_vec(k) = -double(applied_duty);
    end
    pid_int_vec(k)      = pid_int_cur;
    fsm_state_vec(k)    = motor_state;
    fault_vec(k)        = fault_reason;

    % 6. Integrar planta fisica continua durante [k*Ts, (k+1)*Ts) con ZOH
    if k < N
      [pstate, delta_counts_in] = motor_plant_step(pstate, applied_duty, drive_fwd, released, cfg);
    end
  end

  res = struct();
  res.t                = t_vec;
  res.theta_true_deg   = theta_true_deg;
  res.theta_rel_deg    = theta_rel_deg;
  res.theta_enc_deg    = theta_enc_deg;
  res.enc_counts       = enc_counts_vec;
  res.delta_counts     = delta_counts_vec;
  res.target_deg       = target_deg_vec;
  res.target_counts    = target_cnt_vec;
  res.rpm_true         = rpm_true_vec;
  res.rpm_meas         = rpm_meas_vec;
  res.rpm_meas_abs     = abs(rpm_meas_vec);
  res.rpm_safety_ema   = rpm_ema_vec;
  res.target_rpm       = target_rpm_vec;
  res.pid_out          = pid_out_vec;
  res.applied_duty     = applied_duty_vec;
  res.signed_duty      = signed_duty_vec;
  res.pid_integral     = pid_int_vec;
  res.fsm_state        = fsm_state_vec;
  res.fault_reason     = fault_vec;
  res.events_log       = events_log;
end
