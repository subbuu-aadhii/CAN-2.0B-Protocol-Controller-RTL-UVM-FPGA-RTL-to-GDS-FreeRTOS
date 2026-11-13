#!/bin/bash
# Oscillator-tolerance sweep: node1 +x ppm, node2 -x ppm, 3-node random collision test
cd "$(dirname "$0")"
R=../01_rtl
iverilog -g2012 -Wno-timescale -o sim.vvp -s tb_can_top tb_can_top.sv $R/can_crc15.v $R/can_btl.v $R/can_bsp.v $R/can_acf.v $R/can_fifo.v $R/can_regs.v $R/can_top.v || exit 1
mkdir -p logs/sweep; echo "ppm,passed,failed,result" > logs/sweep/tolerance_sweep.csv
for ppm in 0 2500 5000 7800 10000 15000 20000 25000 30000 40000; do
  vvp -n sim.vvp +TEST=random_burst +SKEW_PPM=$ppm > logs/sweep/random_burst_$ppm.log 2>&1
  p=$(grep -o "[0-9]* passed" logs/sweep/random_burst_$ppm.log | grep -o "[0-9]*"); f=$(grep -o "[0-9]* failed" logs/sweep/random_burst_$ppm.log | grep -o "[0-9]*")
  r=$(grep -o "TEST PASSED\|TEST FAILED" logs/sweep/random_burst_$ppm.log | tail -1)
  echo "$ppm,${p:-0},${f:-0},${r:-TIMEOUT}" | tee -a logs/sweep/tolerance_sweep.csv
  rm -f bits_random_burst.log
done
