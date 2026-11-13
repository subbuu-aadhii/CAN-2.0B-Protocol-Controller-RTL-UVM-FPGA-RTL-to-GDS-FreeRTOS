// -----------------------------------------------------------------------------
// can_ifs.sv - APB and CAN-bus interfaces for the UVM testbench
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

interface apb_if (input logic clk, input logic rst_n);
    logic        psel, penable, pwrite;
    logic [7:0]  paddr;
    logic [31:0] pwdata, prdata;
    logic        pready, pslverr, irq;

    // APB protocol checks
    property p_penable_after_psel;
        @(posedge clk) disable iff (!rst_n) (psel && !penable) |=> (psel && penable);
    endproperty
    property p_stable_in_access;
        @(posedge clk) disable iff (!rst_n) (psel && penable) |-> ($stable(paddr) && $stable(pwrite));
    endproperty
    a_apb_setup_access: assert property (p_penable_after_psel) else $error("APB: SETUP not followed by ACCESS");
    a_apb_stable:       assert property (p_stable_in_access)   else $error("APB: addr/dir changed in ACCESS");
    a_apb_no_err:       assert property (@(posedge clk) disable iff (!rst_n) psel |-> !pslverr);
endinterface

// CAN bus: wired-AND of the DUT, the UVM BFM node and an error injector.
// Contains a simple TB-side bit timer (re-synchronises on every falling edge)
// used by the BFM driver (bit_start) and the protocol monitor (sample_pt).
interface can_if (input logic clk, input logic rst_n);
    logic dut_tx;            // DUT can_tx
    logic bfm_tx = 1'b1;     // UVM BFM node
    logic inj    = 1'b0;     // error injector: force dominant
    wire  bus = dut_tx & bfm_tx & ~inj;

    int unsigned bit_clks = 20;   // clocks per bit
    int unsigned sp_clks  = 14;   // sample point position
    int unsigned cnt;
    logic        bus_d;
    int unsigned ones;            // consecutive recessive bits sampled
    logic        sof_seen;        // the last sampled bit was a SOF after bus idle
    logic        ack_req = 0;     // monitor -> driver : drive ACK in the next bit

    wire bit_start = (cnt == 0);
    wire sample_pt = (cnt == sp_clks);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 1; bus_d <= 1'b1; ones <= 0; sof_seen <= 1'b0;
        end else begin
            bus_d <= bus;
            if (bus_d && !bus)              cnt <= 2;      // resync to the edge
            else if (cnt >= bit_clks - 1)   cnt <= 0;
            else                            cnt <= cnt + 1;
            if (sample_pt) begin
                ones     <= bus ? ((ones < 1000) ? ones + 1 : ones) : 0;
                sof_seen <= (!bus && ones >= 11);
            end
        end
    end
endinterface
