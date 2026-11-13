# GTKWave Tcl: add key signals and zoom to the first frame
set sigs [list tb_can_top.can_bus tb_can_top.can_tx\[2:0\] tb_can_top.inject_dom tb_can_top.irq\[2:0\] \
  tb_can_top.dut0.u_btl.tx_pt tb_can_top.dut0.u_btl.sample_pt tb_can_top.dut0.u_btl.sampled_bit \
  tb_can_top.dut0.u_btl.resync_p tb_can_top.dut0.u_btl.hard_sync_p \
  tb_can_top.dut0.u_bsp.st\[4:0\] tb_can_top.dut0.u_bsp.tx_active tb_can_top.dut0.u_bsp.stf_cnt\[2:0\] tb_can_top.dut0.u_bsp.stf_pend \
  tb_can_top.dut0.u_bsp.crc\[14:0\] tb_can_top.dut0.u_bsp.tec\[8:0\] tb_can_top.dut1.u_bsp.st\[4:0\] tb_can_top.dut1.u_bsp.rec\[7:0\] \
  tb_can_top.dut1.u_bsp.rx_id\[28:0\] tb_can_top.dut1.u_bsp.rx_data\[63:0\] tb_can_top.dut1.u_bsp.rx_valid_p]
gtkwave::addSignalsFromList $sigs
gtkwave::/Edit/Data_Format/Hex
gtkwave::setZoomRangeTimes $::env(T0) $::env(T1)
gtkwave::/View/Show_Grid
