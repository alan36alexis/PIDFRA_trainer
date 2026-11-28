function m = pidfra_step_metrics(t, y, y_ref, tol_abs, n_settle)
% PIDFRA_STEP_METRICS Calcula indicadores transitorios ante una entrada escalon.
%
% Implementa los 5 indicadores de manual_teoria_control_discreto.md (§9.1):
%   1. t_delay_s     : Tiempo muerto/retardo hasta superar el 5% del escalon
%   2. t_rise_s      : Tiempo de subida entre 10% y 90% del escalon
%   3. overshoot_pct : Sobrepico maximo porcentual Mp%
%   4. t_settle_s    : Tiempo de establecimiento permaneciendo >= n_settle muestras
%                      dentro de |y - y_ref| <= tol_abs hasta el final
%   5. e_ss          : Error estacionario en el 10% final de la ventana

  t = t(:);
  y = y(:);
  N = length(t);

  if nargin < 5 || isempty(n_settle)
    n_settle = 5;
  end

  y0 = y(1);
  dy = double(y_ref) - double(y0);
  t0 = t(1);

  if nargin < 4 || isempty(tol_abs) || (tol_abs <= 0.0)
    tol_abs = max(0.02 * abs(dy), 1e-6);
  end

  m = struct( ...
    'y0',            y0, ...
    'y_ref',         double(y_ref), ...
    't_delay_s',     NaN, ...
    't_rise_s',      NaN, ...
    'overshoot_pct', 0.0, ...
    'y_peak',        y0, ...
    't_settle_s',    NaN, ...
    'settled',       false, ...
    'e_ss',          NaN ...
  );

  if abs(dy) < 1e-9 || N < 3
    m.e_ss = double(y_ref) - y(end);
    return;
  end

  sgn = sign(dy);
  prog = sgn * (y - y0) / abs(dy);

  % 1. Tiempo de retardo (5% del cambio)
  idx_5 = find(prog >= 0.05, 1, 'first');
  if ~isempty(idx_5)
    m.t_delay_s = t(idx_5) - t0;
  end

  % 2. Tiempo de subida (10% -> 90%)
  idx_10 = find(prog >= 0.10, 1, 'first');
  idx_90 = find(prog >= 0.90, 1, 'first');
  if (~isempty(idx_10)) && (~isempty(idx_90)) && (idx_90 >= idx_10)
    m.t_rise_s = t(idx_90) - t(idx_10);
  end

  % 3. Sobrepico maximo porcentual Mp%
  [max_prog, idx_pk] = max(prog);
  m.y_peak = y(idx_pk);
  if max_prog > 1.0
    m.overshoot_pct = (max_prog - 1.0) * 100.0;
  else
    m.overshoot_pct = 0.0;
  end

  % 4. Tiempo de establecimiento con criterio de permanencia
  in_band = abs(y - double(y_ref)) <= tol_abs;
  % Buscamos el primer indice k tal que in_band(k:end) sea verdadero y tenga al menos n_settle muestras
  idx_out = find(~in_band, 1, 'last');
  if isempty(idx_out)
    first_in = 1;
  else
    first_in = idx_out + 1;
  end

  if (first_in <= N) && ((N - first_in + 1) >= n_settle)
    m.t_settle_s = t(first_in) - t0;
    m.settled    = true;
  end

  % 5. Error en estado estacionario (promedio del ultimo 10% de muestras)
  tail_start = max(1, floor(0.90 * N));
  m.e_ss = double(y_ref) - mean(y(tail_start:N));
end
