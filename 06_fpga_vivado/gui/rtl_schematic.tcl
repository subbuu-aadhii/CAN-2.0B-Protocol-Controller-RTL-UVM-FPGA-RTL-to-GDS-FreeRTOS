# open the elaborated (RTL) design of the IP and show its top-level schematic
set rtl [file normalize [file dirname [info script]]/../../01_rtl]
read_verilog [glob $rtl/*.v]
synth_design -rtl -top can_top -part xc7a35tcpg236-1 -name rtl_1
show_schematic [concat [get_cells] [get_ports]] -name can_top_rtl
