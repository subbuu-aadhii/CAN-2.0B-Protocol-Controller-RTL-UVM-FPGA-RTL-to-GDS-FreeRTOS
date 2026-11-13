// -----------------------------------------------------------------------------
// tb_top.sv - UVM top: 20 MHz clock, DUT, interfaces, SVA bind, run_test()
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_top;
    import uvm_pkg::*;
    `include "uvm_macros.svh"

    logic clk = 0, rst_n = 0;
    always #25 clk = ~clk;                 // 20 MHz
    initial begin repeat (10) @(posedge clk); rst_n = 1; end

    apb_if apb (.clk(clk), .rst_n(rst_n));
    can_if can (.clk(clk), .rst_n(rst_n));

    can_top dut (
        .clk(clk), .rst_n(rst_n),
        .psel(apb.psel), .penable(apb.penable), .pwrite(apb.pwrite), .paddr(apb.paddr),
        .pwdata(apb.pwdata), .prdata(apb.prdata), .pready(apb.pready), .pslverr(apb.pslverr),
        .irq(apb.irq), .can_rx(can.bus), .can_tx(can.dut_tx));

    initial begin
        uvm_config_db#(virtual apb_if)::set(null, "*", "apb_vif", apb);
        uvm_config_db#(virtual can_if)::set(null, "*", "can_vif", can);
        run_test();
    end

    initial begin
        if ($test$plusargs("WAVES")) begin
            $dumpfile("uvm_wave.vcd");
            $dumpvars(0, tb_top.can.bus, tb_top.can.dut_tx, tb_top.can.bfm_tx, tb_top.can.inj, tb_top.dut.u_bsp.st,
                      tb_top.dut.u_bsp.tec, tb_top.dut.u_bsp.rec, tb_top.apb.irq);
        end
    end
    initial begin #400_000_000; `uvm_fatal("TIMEOUT", "global timeout") end

    // tests are compiled inside the top so the factory sees them
    `include "can_tests.sv"
endmodule
