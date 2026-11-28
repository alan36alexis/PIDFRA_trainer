function s = bool_str(flag)
% BOOL_STR Formatea un valor logico como 'SI' o 'NO' para reportes tabulares.
  if flag
    s = 'SI';
  else
    s = 'NO';
  end
end
