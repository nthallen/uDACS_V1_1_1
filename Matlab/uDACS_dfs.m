function uDACS_dfs
% uDACS DFS test
serial_port_clear();

% Set up the data screen
fig = figure;
set(fig,'color',[.8 .8 1]);
dfs = data_fields(fig,'h_leading', 5, 'txt_fontsize', 12);

dfs.start_col();
dfs.field('uDACS','VSet0','%5.3f');
dfs.field('uDACS','VSet1','%5.3f');
dfs.field('uDACS','VSet2','%5.3f');
dfs.field('uDACS','VSet3','%5.3f');
dfs.field('uDACS','ADCStat', '%04X');
dfs.field('uDACS','ADCNcvt', '%5d');
dfs.field('uDACS','MFCtr', '%5d');
dfs.end_col();
dfs.start_col();
dfs.field('uDACS','Err0', '%2X');
dfs.field('uDACS','Err1', '%2X');
dfs.field('uDACS','Err2', '%2X');
dfs.field('uDACS','Err3', '%2X');
dfs.field('uDACS','Err4', '%2X');
dfs.field('uDACS','Err5', '%2X');
dfs.field('uDACS','Err6', '%2X');
dfs.field('uDACS','Err7', '%2X');
dfs.end_col();
dfs.start_col();
dfs.field('uDACS','AIN0', '%6.4f', true);
dfs.field('uDACS','AIN1', '%6.4f', true);
dfs.field('uDACS','AIN2', '%6.4f', true);
dfs.field('uDACS','AIN3', '%6.4f', true);
dfs.field('uDACS','AIN4', '%6.4f', true);
dfs.field('uDACS','AIN5', '%6.4f', true);
dfs.field('uDACS','AIN6', '%6.4f', true);
dfs.field('uDACS','AIN7', '%6.4f', true);
dfs.end_col();

ud = sup_setup;
sup = data_super(dfs, ud, @sup_acq);
end

function ud = sup_setup
  [ud.s,port] = serial_port_init('COM9');
  set(ud.s,'BaudRate',57600);
  % First check that the board is a uDACS
  BdID = read_subbus(ud.s, 2);
  if BdID ~= 9
    error('Expected BdID 9 (uDACS). Reported %d', BdID);
  end
  Build = read_subbus(ud.s,3);
  [SerialNo,SNack] = read_subbus(ud.s,4);
  [InstID,InstIDack] = read_subbus(ud.s,5);
  fprintf(1, 'Attached to uDACS S/N %d Build # %d\n', SerialNo, Build);
  ud.rm_obj = read_multi_prep([16,1,37]);
  ud.MFCtr = 0;
end

function sup_acq(sup)
  str.MFCtr = sup.ud.MFCtr;
  str.TuDACS = str.MFCtr;
  sup.ud.MFCtr = sup.ud.MFCtr + 1;
  [vals,ack] = read_multi(sup.ud.s,sup.ud.rm_obj);
  % fprintf(1,'%04X %d\n', vals(1),vals(end));
  str.ADCStat = vals(5);
  str.ADCNcvt = vals(end);
  vset = vals(1:4)*5/(2^16);
  for i=0:3
    name = sprintf('VSet%d',i);
    str.(name) = vset(i+1);
  end
  hdr = floor(vals(7:2:end-1)/256);
  adc = bitand(vals(7:2:end-1),255)*65536 + vals(6:2:end-2);
  V = adc >= 2^23;
  sadc = adc - (adc>=2^23)*2^24;
  vref = 2.5;
  vadc = vref * sadc / (2^23);
  for i=1:length(hdr)
    % fprintf(1,'%02X %8X %8d %10f V\n',hdr(i), adc(i), sadc(i), vadc(i));
    name = sprintf('Err%d',i-1);
    str.(name) = hdr(i);
    name = sprintf('AIN%d',i-1);
    str.(name) = vadc(i);
  end
  
  sup.dfs.process_record('uDACS', str);
  % fprintf(1, 'sup_acq(%d)\n', sup.ud.MFCtr);
end

function sup_shutdown
  serial_port_clear();
end
