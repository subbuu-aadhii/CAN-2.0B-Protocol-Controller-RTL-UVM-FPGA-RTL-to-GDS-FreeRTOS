// -----------------------------------------------------------------------------
// can_top.v  -  CAN 2.0B controller IP (APB3 slave)
//
//            +---------------------------------------------------------+
//  APB3 ---->| can_regs (CSR, TX buffer, ACF, RX FIFO, IRQ)            |--> irq
//            |      |  cfg / tx frame          ^ rx frame / status       |
//            |      v                          |                         |
//            |  can_bsp (framing, stuffing, CRC, arbitration, errors,    |
//            |           TEC/REC, bus-off)                               |
//            |      ^ sample_pt / tx_pt        | tx_bit                  |
//            |  can_btl (prescaler, segments, hard/re-sync)              |
//  can_rx -->| 2-FF sync -> (loopback mux) ----+------------------------>|--> can_tx
//            +---------------------------------------------------------+
// -----------------------------------------------------------------------------
`default_nettype none
module can_top #(
    parameter RX_FIFO_DEPTH = 4,
    parameter RX_FIFO_AW    = 2
) (
    input  wire        clk,
    input  wire        rst_n,
    // APB3
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [7:0]  paddr,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,
    output wire        irq,
    // CAN transceiver interface (1 = recessive)
    input  wire        can_rx,
    output wire        can_tx
);
    // configuration
    wire        reset_mode, listen_only, loopback, single_shot;
    wire [9:0]  brp;  wire [3:0] tseg1; wire [2:0] tseg2; wire [1:0] sjw;
    // tx buffer
    wire        tx_pending, tx_abort_req, tx_ide, tx_rtr;
    wire [28:0] tx_id;  wire [3:0] tx_dlc;  wire [63:0] tx_data;
    // bsp status
    wire        tx_done_p, tx_fail_p, tx_busy, arb_lost_p, rx_valid_p, rx_busy;
    wire [6:0]  arb_lost_pos;
    wire [28:0] rx_id;  wire rx_ide, rx_rtr;  wire [3:0] rx_dlc;  wire [63:0] rx_data;
    wire [8:0]  tec;    wire [7:0] rec;
    wire        err_warning, err_passive, bus_off, bus_err_p, err_dir_tx;
    wire [2:0]  err_type;  wire [4:0] err_state, fsm_state;
    wire        busoff_enter_p, busoff_recover_p;
    // timing
    wire        sample_pt, tx_pt, sampled_bit, hard_sync_en, tx_bit;
    /* verilator lint_off UNUSED */
    wire        hard_sync_p, resync_p, overload_dbg;   // debug-only observability
    /* verilator lint_on UNUSED */

    // ---------------------------------------------- RX synchroniser ----------
    reg rx_s1, rx_s2;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin rx_s1 <= 1'b1; rx_s2 <= 1'b1; end
        else        begin rx_s1 <= can_rx; rx_s2 <= rx_s1; end

    wire rx_int = loopback ? tx_bit : rx_s2;          // internal loopback
    assign can_tx = (loopback | listen_only | reset_mode) ? 1'b1 : tx_bit;

    can_regs #(.RX_FIFO_DEPTH(RX_FIFO_DEPTH), .RX_FIFO_AW(RX_FIFO_AW)) u_regs (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata),
        .prdata(prdata), .pready(pready), .pslverr(pslverr), .irq(irq),
        .reset_mode(reset_mode), .listen_only(listen_only), .loopback(loopback),
        .single_shot(single_shot), .brp(brp), .tseg1(tseg1), .tseg2(tseg2), .sjw(sjw),
        .tx_pending(tx_pending), .tx_abort_req(tx_abort_req), .tx_id(tx_id), .tx_ide(tx_ide),
        .tx_rtr(tx_rtr), .tx_dlc(tx_dlc), .tx_data(tx_data),
        .tx_done_p(tx_done_p), .tx_fail_p(tx_fail_p), .tx_busy(tx_busy),
        .arb_lost_p(arb_lost_p), .arb_lost_pos(arb_lost_pos),
        .rx_valid_p(rx_valid_p), .rx_id(rx_id), .rx_ide(rx_ide), .rx_rtr(rx_rtr),
        .rx_dlc(rx_dlc), .rx_data(rx_data), .rx_busy(rx_busy),
        .tec(tec), .rec(rec), .err_warning(err_warning), .err_passive(err_passive),
        .bus_off(bus_off), .bus_err_p(bus_err_p), .err_type(err_type), .err_dir_tx(err_dir_tx),
        .err_state(err_state), .busoff_enter_p(busoff_enter_p),
        .busoff_recover_p(busoff_recover_p), .fsm_state(fsm_state));

    can_btl u_btl (
        .clk(clk), .rst_n(rst_n), .enable(~reset_mode),
        .brp(brp), .tseg1(tseg1), .tseg2(tseg2), .sjw(sjw),
        .rx(rx_int), .tx_bit(tx_bit), .hard_sync_en(hard_sync_en),
        .sample_pt(sample_pt), .tx_pt(tx_pt), .sampled_bit(sampled_bit),
        .hard_sync_p(hard_sync_p), .resync_p(resync_p));

    can_bsp u_bsp (
        .clk(clk), .rst_n(rst_n),
        .sample_pt(sample_pt), .tx_pt(tx_pt), .rx_bit(sampled_bit), .tx_bit(tx_bit),
        .hard_sync_en(hard_sync_en),
        .reset_mode(reset_mode), .listen_only(listen_only), .loopback(loopback),
        .single_shot(single_shot),
        .tx_req(tx_pending), .tx_abort(tx_abort_req), .tx_id(tx_id), .tx_ide(tx_ide),
        .tx_rtr(tx_rtr), .tx_dlc(tx_dlc), .tx_data(tx_data),
        .tx_done_p(tx_done_p), .tx_fail_p(tx_fail_p), .arb_lost_p(arb_lost_p),
        .arb_lost_pos(arb_lost_pos), .tx_busy(tx_busy),
        .rx_valid_p(rx_valid_p), .rx_id(rx_id), .rx_ide(rx_ide), .rx_rtr(rx_rtr),
        .rx_dlc(rx_dlc), .rx_data(rx_data), .rx_busy(rx_busy),
        .tec(tec), .rec(rec), .err_warning(err_warning), .err_passive(err_passive),
        .bus_off(bus_off), .bus_err_p(bus_err_p), .err_type(err_type),
        .err_dir_tx(err_dir_tx), .err_state(err_state),
        .busoff_enter_p(busoff_enter_p), .busoff_recover_p(busoff_recover_p),
        .overload_p(overload_dbg), .fsm_state(fsm_state));
endmodule
`default_nettype wire
