// -----------------------------------------------------------------------------
// can_sva.sv - SystemVerilog assertions bound into can_top (white-box checks)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module can_sva (
    input logic       clk, rst_n,
    input logic       can_tx,
    input logic       tx_bit, tx_pt, sample_pt,
    input logic       reset_mode, listen_only, loopback,
    input logic       tx_active,
    input logic [4:0] st,
    input logic [8:0] tec,
    input logic [7:0] rec,
    input logic       bus_off, err_passive,
    input logic [2:0] fifo_cnt
);
    default clocking cb @(posedge clk); endclocking

    // the drive point and the sample point never coincide
    a_tx_sample_excl: assert property (disable iff (!rst_n) !(tx_pt && sample_pt));

    // the transmitted bit only changes at a tx point (or when entering reset mode)
    a_tx_change_at_txpt: assert property (disable iff (!rst_n) $changed(tx_bit) |-> $past(tx_pt) || $past(reset_mode) || reset_mode);

    // silent modes / bus-off never drive the bus dominant
    a_silent_modes: assert property (disable iff (!rst_n) (reset_mode || listen_only || loopback) |-> can_tx);
    a_busoff_recessive: assert property (disable iff (!rst_n) bus_off && tx_bit |=> (bus_off -> can_tx));

    // fault confinement consistency
    a_busoff_tec: assert property (disable iff (!rst_n) $rose(bus_off) |-> tec > 255);
    a_passive_def: assert property (disable iff (!rst_n) err_passive == (tec > 127 || rec > 127));
    a_tec_range: assert property (disable iff (!rst_n) tec <= 263);

    // RX FIFO never above its depth
    a_fifo_depth: assert property (disable iff (!rst_n) fifo_cnt <= 4);

    // the DUT never transmits 6 equal consecutive bits inside the stuffed region
    int run; logic last;
    always @(posedge clk) begin
        if (sample_pt) begin
            if (tx_active && st >= 5'd2 && st <= 5'd11) begin
                run  <= (tx_bit == last) ? run + 1 : 1;
                last <= tx_bit;
            end else begin
                run <= 0; last <= 1'bx;
            end
        end
    end
    a_stuff_rule: assert property (disable iff (!rst_n) run <= 5) else $error("DUT transmitted 6 equal bits in stuffed region");

    // coverage of interesting protocol situations
    c_arb_lost:   cover property (disable iff (!rst_n) tx_active && st inside {[2:6]} ##1 !tx_active && st inside {[2:7]});
    c_bus_off:    cover property (disable iff (!rst_n) $rose(bus_off));
    c_passive:    cover property (disable iff (!rst_n) $rose(err_passive));
    c_err_flag:   cover property (disable iff (!rst_n) st == 5'd18);
    c_overload:   cover property (disable iff (!rst_n) st == 5'd18 && tx_bit == 0 && !tx_active);
    c_suspend:    cover property (disable iff (!rst_n) st == 5'd17);
endmodule

bind can_top can_sva u_sva (
    .clk(clk), .rst_n(rst_n), .can_tx(can_tx), .tx_bit(tx_bit), .tx_pt(tx_pt), .sample_pt(sample_pt),
    .reset_mode(reset_mode), .listen_only(listen_only), .loopback(loopback),
    .tx_active(u_bsp.tx_active), .st(u_bsp.st), .tec(tec), .rec(rec), .bus_off(bus_off),
    .err_passive(err_passive), .fifo_cnt(u_regs.fifo_cnt3));
