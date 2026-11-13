# build.tcl - Vivado non-project batch flow
#   vivado -mode batch -source build.tcl
# 1) Out-of-context synthesis of the bare IP (can_top) @ 200 MHz target -> area / Fmax
# 2) Full Basys-3 demo: synth -> opt -> place -> phys_opt -> route -> bitstream + reports
set here [file dirname [file normalize [info script]]]
cd $here
set rtl  [file normalize ../01_rtl]
set part xc7a35tcpg236-1
file mkdir reports out

set rtl_files [list $rtl/can_crc15.v $rtl/can_btl.v $rtl/can_bsp.v $rtl/can_acf.v \
                    $rtl/can_fifo.v $rtl/can_regs.v $rtl/can_top.v]

# ------------------------------------------------------------------ 1. IP OOC
read_verilog $rtl_files
synth_design -top can_top -part $part -mode out_of_context -flatten_hierarchy rebuilt
create_clock -period 5.000 -name clk [get_ports clk]
set_input_delay  1.0 -clock clk [get_ports -filter {DIRECTION == IN && NAME != clk}]
set_output_delay 1.0 -clock clk [all_outputs]
opt_design
place_design
route_design
report_utilization    -hierarchical -file reports/ip_ooc_utilization_hier.rpt
report_utilization                  -file reports/ip_ooc_utilization.rpt
report_timing_summary -max_paths 10 -file reports/ip_ooc_timing_200MHz.rpt
write_checkpoint -force out/can_top_ooc_routed.dcp
close_project

# ------------------------------------------------------------------ 2. demo top
read_verilog $rtl_files
read_verilog src/can_fpga_top.v
read_xdc     src/basys3.xdc
synth_design -top can_fpga_top -part $part -flatten_hierarchy rebuilt
write_checkpoint -force out/post_synth.dcp
report_utilization -hierarchical -file reports/synth_utilization_hier.rpt
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force out/post_route.dcp
report_utilization                    -file reports/impl_utilization.rpt
report_utilization -hierarchical      -file reports/impl_utilization_hier.rpt
report_timing_summary -max_paths 10   -file reports/impl_timing_summary.rpt
report_clock_utilization              -file reports/clock_utilization.rpt
report_power                          -file reports/power.rpt
report_drc                            -file reports/drc.rpt
report_methodology                    -file reports/methodology.rpt
report_io                             -file reports/io.rpt
write_bitstream -force out/can_fpga_top.bit
puts "BUILD DONE"
