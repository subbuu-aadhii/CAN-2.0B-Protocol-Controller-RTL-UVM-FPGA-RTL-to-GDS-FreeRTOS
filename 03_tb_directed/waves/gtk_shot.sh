#!/bin/bash
cd /mnt/d/CAN20B_Controller_Project/03_tb_directed/waves
export T0=$2 T1=$3
nohup gtkwave -S gtkw_view.tcl wave_$1.vcd > /tmp/gtk_$1.log 2>&1 &
echo started
