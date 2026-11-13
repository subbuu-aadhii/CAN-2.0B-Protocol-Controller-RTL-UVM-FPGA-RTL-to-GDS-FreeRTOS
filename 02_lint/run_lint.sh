#!/bin/bash
# Lint the RTL with Verilator (all warnings) and Icarus (strict Verilog-2005)
cd "$(dirname "$0")"
RTL=../01_rtl
FILES="$RTL/can_crc15.v $RTL/can_btl.v $RTL/can_bsp.v $RTL/can_acf.v $RTL/can_fifo.v $RTL/can_regs.v $RTL/can_top.v"
echo "===== Verilator $(verilator --version) --lint-only -Wall ====="
verilator --lint-only -Wall -Wno-DECLFILENAME --top-module can_top $FILES 2>&1 | tee verilator_lint.log
echo "verilator exit: ${PIPESTATUS[0]}"
echo "===== Icarus Verilog -g2005 -Wall ====="
iverilog -g2005 -Wall -o /tmp/lint.vvp -s can_top $FILES 2>&1 | tee iverilog_lint.log
echo "iverilog exit: ${PIPESTATUS[0]}"
