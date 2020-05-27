function tick_subbus(s)
fprintf(s, 'T');
tline = fgetl(s);
if isempty(tline)
  fprintf(1,'ERROR: tick_subbus timeout with no reply\n');
elseif tline(1) ~= '0'
  fprintf(1,'ERROR: tick_subbus received return string "%s"\n', tline);
end
