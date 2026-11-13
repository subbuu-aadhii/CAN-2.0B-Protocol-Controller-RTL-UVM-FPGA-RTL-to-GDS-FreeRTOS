#!/bin/bash
# Directed regression: compile once with Icarus, run every test, summarise.
cd "$(dirname "$0")"
RTL=../01_rtl
mkdir -p logs
iverilog -g2012 -Wall -Wno-timescale -o sim.vvp -s tb_can_top tb_can_top.sv \
  $RTL/can_crc15.v $RTL/can_btl.v $RTL/can_bsp.v $RTL/can_acf.v $RTL/can_fifo.v $RTL/can_regs.v $RTL/can_top.v \
  2>&1 | tee logs/compile.log || exit 1
TESTS=${@:-"regs std_frame ext_frame remote_dlc arbitration arb_std_ext filter ack_error single_shot error_inject form_error bus_off loopback listen_only overrun random_burst"}
echo "TEST                 RESULT" > logs/summary.txt
for t in $TESTS; do
  vvp -n sim.vvp +TEST=$t $VCDARG > logs/$t.log 2>&1
  r=$(grep -o "TEST PASSED\|TEST FAILED" logs/$t.log | tail -1); [ -z "$r" ] && r="NO RESULT"
  printf "%-20s %s   (%s)\n" $t "$r" "$(grep -o '[0-9]* passed, [0-9]* failed' logs/$t.log)" | tee -a logs/summary.txt
  mv -f bits_$t.log logs/ 2>/dev/null
done
