classdef data_super < handle
  properties(Access = public)
    dfs
    running
    ud
    acq % function handle to iterate on the data
  end
  methods
    function sup = data_super(dfs, ud, acq)
      sup.dfs = dfs;
      sup.ud = ud;
      sup.acq = acq;
      sup.running = false;
      m = sup.dfs.add_menu('uDACS');
      uimenu(m,'Text','Start','callback',{ @sup.set_running, true });
      uimenu(m,'Text','Stop','callback',{ @sup.set_running, false });
      sup.dfs.add_userdata(sup);
    end
    function set_running(sup,~,~,do_start)
      fprintf(1,'set_running()\n');
      sup.running = do_start;
      while sup.running
        sup.acq(sup);
        pause(1);
      end
    end
  end
end