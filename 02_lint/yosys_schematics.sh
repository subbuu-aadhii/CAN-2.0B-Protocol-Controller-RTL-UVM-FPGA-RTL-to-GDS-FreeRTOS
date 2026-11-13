#!/bin/bash
# Yosys: synthesis statistics per module + gate-level schematics (Graphviz)
cd /mnt/d/CAN20B_Controller_Project/02_lint
R=../01_rtl; S=schematics
yosys -q -p "read_verilog $R/can_crc15.v; synth -top can_crc15 -flatten; abc -g AND,OR,XOR,MUX; opt_clean; stat; show -format png -colors 2 -width -stretch -prefix $S/crc15_gates" > $S/crc15_stat.txt 2>&1
yosys -q -p "read_verilog $R/can_acf.v; synth -top can_acf -flatten; abc -g AND,OR,XOR; opt_clean; show -format png -width -stretch -prefix $S/acf_gates" > /dev/null 2>&1
yosys -q -p "read_verilog $R/can_crc15.v $R/can_btl.v $R/can_bsp.v $R/can_acf.v $R/can_fifo.v $R/can_regs.v $R/can_top.v; hierarchy -top can_top; proc; opt; show -format png -prefix $S/can_top_hierarchy can_top" > /dev/null 2>&1
yosys -p "read_verilog $R/can_crc15.v $R/can_btl.v $R/can_bsp.v $R/can_acf.v $R/can_fifo.v $R/can_regs.v $R/can_top.v; synth -top can_top; stat" 2>&1 | sed -n '/Printing statistics/,$p' > yosys_generic_synth_stat.txt
ls -la $S; tail -30 yosys_generic_synth_stat.txt | grep -E "===|cells|wires|DFF|Chip|can_"
