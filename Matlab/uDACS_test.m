%%
cd C:\Users\nort\Documents\Documents\Exp\Boards\uDACS\uDACS_V1_1_1\Matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init('COM9');
set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%%
% First check that the board is a uDACS
[subfunc,desc] = get_subfunction(s);
if subfunc ~= 9 && subfunc ~= 14
  error('Expected BdID 9 or 14. Reported %d', BdID);
end
BoardID = read_subbus(s,2);
Build = read_subbus(s,3);
[SerialNo,SNack] = read_subbus(s,4);
[InstID,InstIDack] = read_subbus(s,5);

if subfunc == 9
  Rev = 'A';
else
  Rev == 'B';
end
if BoardID == 1
  BdCfg = 'uDACS A';
else
  BdCfg = 'uDACS B';
end

fprintf(1, 'Attached to uDACS S/N %d Build # %d\n', SerialNo, Build);
fprintf(1, 'Board is Rev %s configured as "%s"\n', Rev, BdCfg);
fprintf(1, 'The description is "%s"\n', desc);


rm_obj = read_multi_prep([8,40,9,0]);
[vals,ack] = read_multi(s,rm_obj);
even = mod(vals(2:end),256);
odd = floor(vals(2:end)/256);
il = [even odd]';
il = il(:)';
nc = find(il == 0,1);
il = il(1:(nc-1));
desc = char(il);
fprintf(1,'Description from FIFO is: %s\n', desc);
%fprintf(1, 'Now figure out how to interpret the result\n');
%%
rm_obj = read_multi_prep([20,1,37]);
%%
while true
  %%
  [vals,ack] = read_multi(s,rm_obj);
  fprintf(1,'---------\n');
  fprintf(1,'%04X %d\n', vals(1),vals(end));
  hdr = floor(vals(3:2:end-1)/256);
  adc = bitand(vals(3:2:end-1),255)*65536 + vals(2:2:end-2);
  V = adc >= 2^23;
  sadc = adc - (adc>=2^23)*2^24;
  vref = 2.5;
  vadc = vref * sadc / (2^23);
  for i=1:length(hdr)
    fprintf(1,'%02X %8X %8d %10f V\n',hdr(i), adc(i), sadc(i), vadc(i));
  end
  %%
  pause(1);
end
%%
[value,ack] = read_subbus(s,39); % General read register
fprintf(1,'ack=%d value=%04X\n', ack,value);
%%
write_subbus(s, 17, 0);
%%
rm_obj = read_multi_prep([64,1,67]);
%%

  [vals,ack] = read_multi(s,rm_obj);

