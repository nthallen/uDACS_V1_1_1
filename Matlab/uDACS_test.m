%%
cd C:\Users\nort\Documents\Documents\Exp\HCHO\uDACS_V1_1_1\Matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init('COM9');
set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%%
% First check that the board is an FCC
BdID = read_subbus(s, 2);
if BdID ~= 9
  error('Expected BdID 9. Reported %d', BdID);
end
Build = read_subbus(s,3);
[SerialNo,SNack] = read_subbus(s,4);
[InstID,InstIDack] = read_subbus(s,5);
fprintf(1, 'Attached to uDACS S/N %d Build # %d\n', SerialNo, Build);
%%
% The verbose output apparently doesn't come here on uDACS
%rm_obj = read_multi_prep([8,40,9,0]);
%[vals,ack] = read_multi(s,rm_obj);
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
