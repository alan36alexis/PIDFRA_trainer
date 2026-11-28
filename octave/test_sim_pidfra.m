% TEST_SIM_PIDFRA Suite automatizada de verificacion unitaria e integracion en Octave.
%
% Uso:
%   octave --no-gui octave/test_sim_pidfra.m

test_root_dir = fileparts(mfilename('fullpath'));
if isempty(test_root_dir)
  test_root_dir = pwd();
end
addpath(test_root_dir);

fprintf('Ejecutando pruebas automatizadas de PIDFRA_trainer (Octave)...\n');

cfg = pidfra_params();
assert(cfg.counts_per_rev == 3600, '3600 cuentas/vuelta esperadas (900 PPR x 4)');
assert(abs(cfg.Ts - 0.01) < 1e-12, 'Periodo de muestreo Ts debe ser 0.01 s (100 Hz)');

% 1. Prueba unitaria de motor_pid_step vs lib/motor_pid/motor_pid.c
pid = motor_pid_init(0.40, 0.20, 0.015, -255.0, 255.0, true);
[u1, pid] = motor_pid_step(pid, 100.0, 0.0, 0.01);
% Primera muestra: derivative = 0 (initialized=false -> true), I_cand = 1.0
% raw = 0.40*100 + 0.20*1.0 + 0 = 40.20
assert(abs(u1 - 40.20) < 1e-9, 'PID muestra 1 sin derivative kick incorrecta');
assert(pid.initialized == true, 'PID debe marcarse initialized tras primera muestra');

% Prueba de clamping anti-windup en saturacion superior
pid_sat = motor_pid_init(1.0, 10.0, 0.0, -255.0, 255.0, true);
[u_sat, pid_sat] = motor_pid_step(pid_sat, 500.0, 0.0, 0.01);
assert(abs(u_sat - 255.0) < 1e-9, 'Salida debe saturar en 255');
assert(abs(pid_sat.integral) < 1e-12, 'Integrador debe congelarse cuando satura con error positivo');

% 2. Prueba unitaria de discretizacion ZOH y analisis en plano Z
ana = pidfra_discrete_analysis(cfg);
assert(ana.loop_speed.is_stable, 'Lazo linealizado SPEED debe ser estable');
assert(ana.loop_pos.is_stable,   'Lazo linealizado POS debe ser estable');
assert(ana.loop_clock.is_stable, 'Lazo linealizado CLOCK debe ser estable');
assert(ana.Gp.sampling_zero < -0.90 && ana.Gp.sampling_zero > -1.0, ...
       'Cero intrinseco ZOH de posicion debe ubicarse en (-1, -0.90)');

% 3. Prueba de zona muerta de planta y cuantizacion PCNT
ps0 = struct('theta_deg', 0.0, 'rpm', 0.0, 'hw_counts', 0);
[ps1, dc1] = motor_plant_step(ps0, 150, true, false, cfg); % 150 < u_static=178
assert(ps1.rpm == 0.0 && dc1 == 0, 'Motor debe permanecer inmovil dentro de la zona muerta');
[ps2, ~] = motor_plant_step(ps0, 220, true, false, cfg);   % 220 > u_static=178
assert(ps2.rpm > 0.0, 'Motor debe iniciar movimiento por encima de u_static');

% 4. Prueba de integracion de escenarios completos
suite = pidfra_scenarios(cfg);

% Posicion ANG 90 converge dentro de +/-8 cuentas (0.8 grados) sin entrar en FAULT
assert(suite.pos.metrics_90.settled, 'ANG 90 debe establecerse dentro de +/-8 cuentas');
assert(abs(suite.pos.metrics_90.e_ss) <= 0.8 + 1e-6, 'Error estacionario ANG 90 excede 8 cuentas');
assert(all(suite.pos.step90_nom.fsm_state ~= 5), 'ANG 90 nominal no debe disparar FSM_FAULT');

% Modo CLOCK completa 10 pasos (600 cuentas) con establecimiento < 400 ms
assert(suite.clock.mean_settle_ms < 400.0, 'Pasos de CLOCK deben establecerse en < 400 ms');
assert(suite.clock.final_error_counts <= cfg.position_tolerance_counts, ...
       'Error final en CLOCK debe quedar dentro de POSITION_TOLERANCE_COUNTS');

% Calibracion de umbral detecta el despegue justo por encima de u_static (178 -> 179..181)
assert(suite.open_cal.detected_threshold >= 179 && suite.open_cal.detected_threshold <= 182, ...
       'Umbral de lazo abierto detectado fuera de rango esperado');

% Protecciones de seguridad FSM_FAULT (STALL a 0.80 s y limite de 1.5 vueltas)
assert(abs(suite.pot_and_faults.t_stall_trip_s - 0.80) <= 0.02, ...
       'Proteccion STALL debe disparar a 300ms + 500ms = 0.80s');
assert(abs(suite.pot_and_faults.revs_at_trip - 1.50) <= 0.05, ...
       'Proteccion de recorrido sin SP debe disparar en 1.50 vueltas');

fprintf('OK: Todas las pruebas de la simulacion en Octave pasaron exitosamente.\n');
