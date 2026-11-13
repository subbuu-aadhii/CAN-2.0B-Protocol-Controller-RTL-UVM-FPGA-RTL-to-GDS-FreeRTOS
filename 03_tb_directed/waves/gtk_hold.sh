#!/bin/bash
# usage: gtk_hold.sh <test> <t0_ps> <t1_ps> <seconds>
cd /mnt/d/CAN20B_Controller_Project/03_tb_directed/waves
export T0=$2 T1=$3
gtkwave --rcvar "initial_window_x 1900" --rcvar "initial_window_y 1000" --rcvar "initial_signal_window_width 260" -S gtkw_view.tcl wave_$1.vcd > /tmp/gtk_$1.log 2>&1 &
P=$!
sleep $4
kill $P
