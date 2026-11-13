`timescale 1ns/1ps
// quick functional check of the Basys-3 demo top (TX period shortened to 0.4 ms)
module tb_fpga;
    reg clk = 0; always #5 clk = ~clk;
    reg btnC = 1; reg [15:0] sw = 16'h0123;
    wire [15:0] led; wire tx;
    can_fpga_top #(.TX_PERIOD(23'd40_000)) dut (.clk(clk), .btnC(btnC), .sw(sw), .led(led),
                                                .can_tx_pin(tx), .can_rx_pin(tx));
    initial begin
        $dumpfile("tb_fpga.vcd"); $dumpvars(0, tb_fpga.led, tb_fpga.tx);
        #200 btnC = 0;
        repeat (4) begin @(led[15]); $display("[%0t ns] node B received ID[3:0]=%h byte0=%h", $time, led[11:8], led[7:0]); end
        sw[11] = 1; sw[10:0] = 11'h7A5;     // switch to extended IDs
        repeat (2) begin @(led[15]); $display("[%0t ns] node B received ID[3:0]=%h byte0=%h", $time, led[11:8], led[7:0]); end
        $display("FPGA TOP SIM %s", (led[11:8] == 4'h5) ? "PASSED" : "FAILED");
        $finish;
    end
    initial #20_000_000 begin $display("TIMEOUT"); $finish; end
endmodule
