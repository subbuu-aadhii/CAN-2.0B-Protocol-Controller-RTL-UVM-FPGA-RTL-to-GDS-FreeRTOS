cd /mnt/d/CAN20B_Controller_Project/06_fpga_vivado/sim
R=../../01_rtl
iverilog -g2012 -Wno-timescale -o fpga.vvp tb_fpga.v ../src/can_fpga_top.v $R/can_crc15.v $R/can_btl.v $R/can_bsp.v $R/can_acf.v $R/can_fifo.v $R/can_regs.v $R/can_top.v && vvp -n fpga.vvp | tee sim.log
