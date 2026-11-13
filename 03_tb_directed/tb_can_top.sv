// -----------------------------------------------------------------------------
// tb_can_top.sv - Directed, self-checking testbench (Icarus Verilog -g2012)
//
//  Three CAN controllers on one wired-AND bus, each with its OWN clock:
//      node0 : 50.00 MHz      node1 : 50.25 MHz (+0.5 %)    node2 : 49.75 MHz (-0.5 %)
//  -> every frame exercises hard-sync + re-synchronisation of the BTL.
//  Bit rate 500 kbit/s : BRP=5 clk/tq, 20 tq/bit (SYNC 1 + TSEG1 15 + TSEG2 4), SJW 4.
//
//  Run one test:  vvp sim.vvp +TEST=std_frame [+VCD]
//  Tests: regs std_frame ext_frame remote_dlc arbitration arb_std_ext filter
//         ack_error single_shot error_inject form_error bus_off loopback
//         listen_only overrun random_burst  (ALL = run all)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_can_top;
    // ------------------------------------------------------------- clocks ---
    reg clk0 = 0, clk1 = 0, clk2 = 0;
    // clock mismatch: node1 = +SKEW_PPM, node2 = -SKEW_PPM (default 5000 ppm = 0.5 %)
    integer skew_ppm = 5000;
    real    hp1 = 9.950, hp2 = 10.050;
    initial if ($value$plusargs("SKEW_PPM=%d", skew_ppm)) begin
        hp1 = 10.0 / (1.0 + skew_ppm * 1.0e-6);
        hp2 = 10.0 / (1.0 - skew_ppm * 1.0e-6);
    end
    always #10.000 clk0 = ~clk0;     // 50.00 MHz
    always #(hp1)  clk1 = ~clk1;     // 50 MHz * (1 + skew)
    always #(hp2)  clk2 = ~clk2;     // 50 MHz * (1 - skew)
    reg rst_n = 0;

    localparam real BIT_NS = 2000.0;
    localparam [31:0] BTR_500K = (3 << 20) | (3 << 16) | (14 << 12) | 4; // SJW4 TSEG2=4 TSEG1=15 BRP=5

    // register offsets
    localparam CTRL=8'h00, BTR=8'h04, CMD=8'h08, STATUS=8'h0C, ISTAT=8'h10, IEN=8'h14,
               ECNT=8'h18, ECODE=8'h1C, ACR=8'h20, AMR=8'h24, TXID=8'h30, TXDLC=8'h34,
               TXD0=8'h38, TXD1=8'h3C, RXID=8'h40, RXDLC=8'h44, RXD0=8'h48, RXD1=8'h4C, VER=8'h50;

    // ------------------------------------------------------------- APB ------
    reg  [2:0]  psel = 0, penable = 0, pwrite = 0;
    reg  [23:0] paddr = 0;
    reg  [95:0] pwdata = 0;
    wire [95:0] prdata;
    wire [2:0]  irq, can_tx, pready, pslverr;

    // ------------------------------------------------------------- bus ------
    reg  inject_dom = 0;                         // error injection: force dominant
    wire can_bus_ideal = &can_tx & ~inject_dom;  // wired-AND (dominant = 0)
    wire #(50) can_bus = can_bus_ideal;          // 50 ns propagation (cable + transceiver)

    can_top dut0 (.clk(clk0), .rst_n(rst_n), .psel(psel[0]), .penable(penable[0]), .pwrite(pwrite[0]),
                  .paddr(paddr[7:0]), .pwdata(pwdata[31:0]), .prdata(prdata[31:0]), .pready(pready[0]),
                  .pslverr(pslverr[0]), .irq(irq[0]), .can_rx(can_bus), .can_tx(can_tx[0]));
    can_top dut1 (.clk(clk1), .rst_n(rst_n), .psel(psel[1]), .penable(penable[1]), .pwrite(pwrite[1]),
                  .paddr(paddr[15:8]), .pwdata(pwdata[63:32]), .prdata(prdata[63:32]), .pready(pready[1]),
                  .pslverr(pslverr[1]), .irq(irq[1]), .can_rx(can_bus), .can_tx(can_tx[1]));
    can_top dut2 (.clk(clk2), .rst_n(rst_n), .psel(psel[2]), .penable(penable[2]), .pwrite(pwrite[2]),
                  .paddr(paddr[23:16]), .pwdata(pwdata[95:64]), .prdata(prdata[95:64]), .pready(pready[2]),
                  .pslverr(pslverr[2]), .irq(irq[2]), .can_rx(can_bus), .can_tx(can_tx[2]));

    // --------------------------------------------------------- bookkeeping --
    integer n_pass = 0, n_fail = 0;
    string  testname;
    integer bitlog;

    task automatic check(input string what, input logic cond);
        if (cond) begin n_pass++; $display("[%0t] PASS  %s", $time, what); end
        else      begin n_fail++; $display("[%0t] FAIL  %s", $time, what); end
    endtask

    task automatic check_eq(input string what, input [63:0] got, input [63:0] exp);
        if (got === exp) begin n_pass++; $display("[%0t] PASS  %s = 0x%0h", $time, what, got); end
        else begin n_fail++; $display("[%0t] FAIL  %s : got 0x%0h expected 0x%0h", $time, what, got, exp); end
    endtask

    // --------------------------------------------------------- APB BFM ------
    task automatic wclk(input integer n);
        case (n) 0: @(posedge clk0); 1: @(posedge clk1); default: @(posedge clk2); endcase
        #1;
    endtask

    task automatic apb_wr(input integer n, input [7:0] a, input [31:0] d);
        wclk(n);
        psel[n] = 1; pwrite[n] = 1; penable[n] = 0; paddr[n*8 +: 8] = a; pwdata[n*32 +: 32] = d;
        wclk(n); penable[n] = 1;
        wclk(n); psel[n] = 0; penable[n] = 0; pwrite[n] = 0;
    endtask

    task automatic apb_rd(input integer n, input [7:0] a, output [31:0] d);
        wclk(n);
        psel[n] = 1; pwrite[n] = 0; penable[n] = 0; paddr[n*8 +: 8] = a;
        wclk(n); penable[n] = 1;
        #1 d = prdata[n*32 +: 32];
        wclk(n); psel[n] = 0; penable[n] = 0;
    endtask

    // ---------------------------------------------------- driver helpers ----
    task automatic node_cfg(input integer n, input [31:0] ctrl_mode, input [30:0] acr_v, input [30:0] amr_v);
        apb_wr(n, CTRL, 32'h1);                 // enter reset mode
        apb_wr(n, BTR,  BTR_500K);
        apb_wr(n, ACR,  {1'b0, acr_v});
        apb_wr(n, AMR,  {1'b0, amr_v});
        apb_wr(n, IEN,  32'h3FF);
        apb_wr(n, ISTAT, 32'h3FF);
        apb_wr(n, CTRL, ctrl_mode & ~32'h1);    // leave reset mode -> bus integration
    endtask

    task automatic node_reset(input integer n);
        apb_wr(n, CTRL, 32'h1);
    endtask

    task automatic load_tx(input integer n, input [28:0] id, input ide, input rtr, input [3:0] dlc,
                           input [63:0] data);  // data[7:0] = byte0
        apb_wr(n, TXID, {1'b0, ide, rtr, id});
        apb_wr(n, TXDLC, {28'd0, dlc});
        apb_wr(n, TXD0, data[31:0]);
        apb_wr(n, TXD1, data[63:32]);
    endtask

    task automatic send(input integer n, input [28:0] id, input ide, input rtr, input [3:0] dlc,
                        input [63:0] data);
        load_tx(n, id, ide, rtr, dlc, data);
        apb_wr(n, CMD, 32'h1);
    endtask

    // wait until IRQ_STAT bit 'b' is set (polling), then clear it. ok=0 on timeout
    task automatic wait_irq(input integer n, input integer b, input real timeout_us, output logic ok);
        reg [31:0] s; real t0;
        t0 = $realtime; ok = 0;
        while (!ok && (($realtime - t0) < timeout_us * 1000.0)) begin
            apb_rd(n, ISTAT, s);
            if (s[b]) begin ok = 1; apb_wr(n, ISTAT, 32'h1 << b); end
            else #(BIT_NS);
        end
    endtask

    task automatic read_rx(input integer n, output [28:0] id, output ide, output rtr,
                           output [3:0] dlc, output [63:0] data);
        reg [31:0] r;
        apb_rd(n, RXID, r);  id = r[28:0]; rtr = r[29]; ide = r[30];
        apb_rd(n, RXDLC, r); dlc = r[3:0];
        apb_rd(n, RXD0, r);  data[31:0]  = r;
        apb_rd(n, RXD1, r);  data[63:32] = r;
        apb_wr(n, CMD, 32'h4);                  // release RX buffer
    endtask

    function automatic [63:0] mask_data(input [63:0] d, input [3:0] dlc, input rtr);
        integer k; reg [63:0] m;
        m = 0;
        if (!rtr) for (k = 0; k < 8; k++) if (k < dlc) m[k*8 +: 8] = 8'hFF;
        return d & m;
    endfunction

    task automatic expect_rx(input integer n, input [28:0] id, input ide, input rtr, input [3:0] dlc,
                             input [63:0] data, input string tag);
        reg [28:0] g_id; reg g_ide, g_rtr; reg [3:0] g_dlc; reg [63:0] g_data; reg [31:0] st;
        apb_rd(n, STATUS, st);
        if (!st[0]) check($sformatf("%s node%0d RX FIFO not empty", tag, n), 0);
        else begin
        read_rx(n, g_id, g_ide, g_rtr, g_dlc, g_data);
        check_eq($sformatf("%s node%0d RX ID ", tag, n), g_id, id);
        check_eq($sformatf("%s node%0d RX IDE", tag, n), g_ide, ide);
        check_eq($sformatf("%s node%0d RX RTR", tag, n), g_rtr, rtr);
        check_eq($sformatf("%s node%0d RX DLC", tag, n), g_dlc, dlc);
        check_eq($sformatf("%s node%0d RX DATA", tag, n), g_data, mask_data(data, (dlc > 8) ? 4'd8 : dlc, rtr));
        end
    endtask

    task automatic expect_fifo_empty(input integer n, input string tag);
        reg [31:0] st;
        apb_rd(n, STATUS, st);
        check($sformatf("%s node%0d RX FIFO empty", tag, n), st[0] == 0);
    endtask

    task automatic all_cfg_normal;
        node_cfg(0, 0, 0, 31'h7FFFFFFF);
        node_cfg(1, 0, 0, 31'h7FFFFFFF);
        node_cfg(2, 0, 0, 31'h7FFFFFFF);
        #(BIT_NS * 12);                         // bus integration (11 recessive bits)
    endtask

    function automatic [31:0] ecnt_tec(input [31:0] r); return r[8:0]; endfunction
    function automatic [31:0] ecnt_rec(input [31:0] r); return r[23:16]; endfunction

    // ------------------------------------------------- bus bit logger -------
    // logs every bus bit sampled by node2 (+ its BSP state) for the Python
    // golden-model cross-check (05_golden_model/check_bitstream.py)
    always @(posedge clk2) if (dut2.sample_pt && bitlog != 0)
        $fdisplay(bitlog, "%0t %0d %0d", $time, dut2.sampled_bit, dut2.u_bsp.st);

    // listen-only monitor: node2 must never drive dominant in listen-only test
    reg node2_drove = 0;
    always @(negedge can_tx[2]) node2_drove = 1;

    // ===================================================================== tests
    task automatic t_regs;
        reg [31:0] r;
        apb_rd(0, VER, r);    check_eq("VERSION", r, 32'hCA20_B100);
        apb_rd(0, CTRL, r);   check_eq("CTRL reset value (RESET_MODE=1)", r, 32'h1);
        apb_rd(0, BTR, r);    check_eq("BTR reset value", r, (1<<20)|(3<<16)|(14<<12)|4);
        apb_rd(0, AMR, r);    check_eq("AMR reset value (accept all)", r, 32'h7FFF_FFFF);
        apb_rd(0, STATUS, r); check_eq("STATUS: TX buffer free, RX empty", r[2:0], 3'b100);
        apb_wr(0, BTR, 32'h0012_3005); apb_rd(0, BTR, r); check_eq("BTR writable in reset mode", r, 32'h0012_3005);
        apb_wr(0, CTRL, 32'h0);
        apb_wr(0, BTR, BTR_500K);      apb_rd(0, BTR, r); check_eq("BTR write-protected in operating mode", r, 32'h0012_3005);
        apb_wr(0, TXID, 32'h4ABC_DEF1); apb_rd(0, TXID, r); check_eq("TX_ID read-back", r, 32'h4ABC_DEF1);
        apb_wr(0, IEN, 32'hFFFF_FFFF); apb_rd(0, IEN, r); check_eq("IRQ_EN width (10 bits)", r, 32'h3FF);
        apb_rd(0, ECNT, r);   check_eq("TEC/REC after reset", r, 0);
        check("pready always 1 / pslverr 0", pready === 3'b111 && pslverr === 3'b000);
    endtask

    task automatic t_std_frame;
        logic ok; reg [31:0] r;
        all_cfg_normal();
        send(0, 29'h123, 0, 0, 4'd8, 64'h8877665544332211);
        wait_irq(0, 1, 600, ok); check("std frame: TX_DONE irq on node0", ok);
        check("irq pin asserted on receiver node1", irq[1] === 1'b1);
        expect_rx(1, 29'h123, 0, 0, 8, 64'h8877665544332211, "std");
        expect_rx(2, 29'h123, 0, 0, 8, 64'h8877665544332211, "std");
        expect_fifo_empty(0, "std (no self-reception)");
        apb_rd(0, ECNT, r); check_eq("node0 TEC/REC", r, 0);
        apb_rd(1, ECNT, r); check_eq("node1 TEC/REC", r, 0);
    endtask

    task automatic t_ext_frame;
        logic ok;
        all_cfg_normal();
        send(1, 29'h1ABC_DE12, 1, 0, 4'd5, 64'h00000000_A5C3E1F0 | (64'h7E << 32));
        wait_irq(1, 1, 600, ok); check("ext frame: TX_DONE", ok);
        expect_rx(0, 29'h1ABC_DE12, 1, 0, 5, 64'h0000007E_A5C3E1F0, "ext");
        expect_rx(2, 29'h1ABC_DE12, 1, 0, 5, 64'h0000007E_A5C3E1F0, "ext");
    endtask

    task automatic t_remote_dlc;
        logic ok;
        all_cfg_normal();
        send(2, 29'h7FF, 0, 1, 4'd4, 64'hDEADBEEF_CAFEF00D);      // F1 std remote, DLC 4
        wait_irq(2, 1, 600, ok); check("remote frame TX_DONE", ok);
        send(0, 29'h000, 0, 0, 4'd0, 64'hFFFF);                     // F2 DLC 0, ID 0 (stuffing heavy)
        wait_irq(0, 1, 600, ok); check("DLC0 TX_DONE", ok);
        send(0, 29'h0000_0000, 1, 1, 4'd15, 0);                     // F3 ext remote, DLC 15
        wait_irq(0, 1, 600, ok); check("ext remote DLC15 TX_DONE", ok);
        send(1, 29'h555, 0, 0, 4'd12, 64'h0123456789ABCDEF);        // F4 DLC 12 -> 8 data bytes
        wait_irq(1, 1, 600, ok); check("DLC12 TX_DONE", ok);
        expect_rx(0, 29'h7FF, 0, 1, 4,  0, "F1 remote");
        expect_rx(0, 29'h555, 0, 0, 12, 64'h0123456789ABCDEF, "F4 dlc12");
        expect_rx(1, 29'h7FF, 0, 1, 4,  0, "F1 remote");
        expect_rx(1, 29'h000, 0, 0, 0,  0, "F2 dlc0");
        expect_rx(1, 29'h0,   1, 1, 15, 0, "F3 ext remote");
        expect_rx(2, 29'h000, 0, 0, 0,  0, "F2 dlc0");
        expect_rx(2, 29'h0,   1, 1, 15, 0, "F3 ext remote");
        expect_rx(2, 29'h555, 0, 0, 12, 64'h0123456789ABCDEF, "F4 dlc12");
    endtask

    task automatic t_arbitration;
        logic ok; reg [31:0] r;
        all_cfg_normal();
        load_tx(0, 29'h123, 0, 0, 2, 64'hA0A0);
        load_tx(1, 29'h122, 0, 0, 2, 64'hB1B1);
        fork
            apb_wr(0, CMD, 1);
            apb_wr(1, CMD, 1);
        join
        wait_irq(0, 5, 600, ok); check("node0 (ID 0x123) lost arbitration", ok);
        apb_rd(0, ECODE, r);     check_eq("node0 arbitration-lost bit position (last ID bit)", r[22:16], 11);
        wait_irq(1, 1, 600, ok); check("node1 (ID 0x122) won, TX_DONE", ok);
        wait_irq(0, 1, 600, ok); check("node0 automatic retransmission done", ok);
        expect_rx(2, 29'h122, 0, 0, 2, 64'hB1B1, "arb first ");
        expect_rx(2, 29'h123, 0, 0, 2, 64'hA0A0, "arb second");
        apb_rd(0, ECNT, r); check_eq("arbitration loss is not an error (TEC=0)", r, 0);
    endtask

    task automatic t_arb_std_ext;
        logic ok; reg [31:0] r;
        all_cfg_normal();
        load_tx(0, {11'h123, 18'h00000}, 1, 0, 1, 64'hE1);   // extended, same base ID
        load_tx(1, 29'h123,              0, 0, 1, 64'h51);   // standard data frame
        load_tx(2, 29'h123,              0, 1, 1, 64'h0);    // standard remote frame
        fork apb_wr(0, CMD, 1); apb_wr(1, CMD, 1); apb_wr(2, CMD, 1); join
        wait_irq(1, 1, 900, ok); check("std data frame wins (RTR dominant)", ok);
        wait_irq(2, 1, 900, ok); check("std remote frame second (IDE dominant beats ext)", ok);
        wait_irq(0, 1, 900, ok); check("ext frame last", ok);
        expect_rx(0, 29'h123, 0, 0, 1, 64'h51, "stdext n0 #1");
        expect_rx(0, 29'h123, 0, 1, 1, 64'h0,  "stdext n0 #2");
        expect_rx(1, 29'h123, 0, 1, 1, 64'h0,  "stdext n1 #1");
        expect_rx(1, {11'h123, 18'h0}, 1, 0, 1, 64'hE1, "stdext n1 #2");
    endtask

    task automatic t_filter;
        logic ok;
        node_cfg(0, 0, 0, 31'h7FFFFFFF);
        // node1 accepts only standard data frames 0x100..0x10F
        node_cfg(1, 0, {1'b0, 1'b0, 18'd0, 11'h100}, 31'h0000000F);
        node_cfg(2, 0, 0, 31'h7FFFFFFF);
        #(BIT_NS * 12);
        send(0, 29'h105, 0, 0, 1, 64'h11); wait_irq(0, 1, 600, ok);
        send(0, 29'h205, 0, 0, 1, 64'h22); wait_irq(0, 1, 600, ok);
        send(0, 29'h105, 0, 1, 0, 64'h0);  wait_irq(0, 1, 600, ok);   // remote -> rejected (RTR)
        send(0, 29'h105, 1, 0, 1, 64'h33); wait_irq(0, 1, 600, ok);   // ext -> rejected (IDE)
        expect_rx(1, 29'h105, 0, 0, 1, 64'h11, "filter accept");
        expect_fifo_empty(1, "filter rejects 0x205/remote/ext");
        expect_rx(2, 29'h105, 0, 0, 1, 64'h11, "nofilter#1");
        expect_rx(2, 29'h205, 0, 0, 1, 64'h22, "nofilter#2");
        expect_rx(2, 29'h105, 0, 1, 0, 64'h0,  "nofilter#3");
        expect_rx(2, 29'h105, 1, 0, 1, 64'h33, "nofilter#4");
    endtask

    task automatic t_ack_error;
        logic ok; reg [31:0] r, s;
        node_cfg(0, 0, 0, 31'h7FFFFFFF);         // node1/node2 stay in reset mode -> nobody ACKs
        #(BIT_NS * 12);
        send(0, 29'h3A5, 0, 0, 2, 64'h1234);
        wait_irq(0, 3, 8000, ok);  check("ACK errors -> ERROR_PASSIVE irq", ok);
        apb_rd(0, ECODE, r);       check_eq("error code = ACK error during TX", r[3:0], 4'b1101);
        #(BIT_NS * 1500);          // many more retries
        apb_rd(0, ECNT, r);        check_eq("TEC frozen at 128 (passive ACK-error exception)", ecnt_tec(r), 128);
        apb_rd(0, STATUS, s);      check("error-passive, NOT bus-off", s[7] && !s[8]);
        apb_wr(0, CMD, 32'h2);     // abort
        wait_irq(0, 8, 600, ok);   check("abort -> TX_FAIL irq", ok);
        apb_rd(0, STATUS, s);      check("TX buffer free after abort", s[2]);
        // bring the others online, a successful frame decrements TEC
        node_cfg(1, 0, 0, 31'h7FFFFFFF); node_cfg(2, 0, 0, 31'h7FFFFFFF);
        #(BIT_NS * 30);
        send(0, 29'h3A5, 0, 0, 2, 64'h1234);
        wait_irq(0, 1, 900, ok);   check("TX succeeds once nodes ACK", ok);
        apb_rd(0, ECNT, r);        check_eq("TEC 128 -> 127 after success", ecnt_tec(r), 127);
        apb_rd(0, STATUS, s);      check("back to error-active (TEC <= 127)", !s[7]);
    endtask

    task automatic t_single_shot;
        logic ok; reg [31:0] r;
        node_cfg(0, 32'h8, 0, 31'h7FFFFFFF);     // single-shot, alone on the bus
        #(BIT_NS * 12);
        send(0, 29'h0F0, 0, 0, 1, 64'h5A);
        wait_irq(0, 8, 600, ok);   check("single-shot: TX_FAIL after first ACK error", ok);
        #(BIT_NS * 200);
        apb_rd(0, ECNT, r);        check_eq("single-shot: only one attempt (TEC=8)", ecnt_tec(r), 8);
    endtask

    // force the bus dominant for one bit while node0 transmits a recessive DATA bit
    task automatic inject_in_data;
        logic done;
        done = 0;
        while (!done) begin
            do @(posedge clk0); while (!(dut0.u_bsp.st == 5'd10 && dut0.tx_pt));
            #(BIT_NS * 0.25);
            if (can_tx[0] === 1'b1 && dut0.u_bsp.st == 5'd10) begin
                inject_dom = 1; #(BIT_NS * 0.9); inject_dom = 0;
                done = 1;
            end
        end
    endtask

    task automatic t_error_inject;
        logic ok; reg [31:0] r;
        all_cfg_normal();
        send(0, 29'h2AA, 0, 0, 8, 64'hFFFFFFFF_FFFFFFFF);
        inject_in_data();
        wait_irq(0, 6, 600, ok);   check("node0 BUS_ERR irq", ok);
        apb_rd(0, ECODE, r);       check_eq("node0 error = BIT error during TX", r[3:0], 4'b1001);
        wait_irq(1, 6, 100, ok);   check("node1 BUS_ERR irq", ok);
        apb_rd(1, ECODE, r);       check_eq("node1 error = STUFF error (sees error flag)", r[3:0], 4'b0010);
        wait_irq(0, 1, 900, ok);   check("retransmission succeeds", ok);
        apb_rd(0, ECNT, r);        check_eq("node0 TEC = 8 - 1 = 7", ecnt_tec(r), 7);
        apb_rd(1, ECNT, r);        check_eq("node1 REC = 1 - 1 = 0", ecnt_rec(r), 0);
        expect_rx(1, 29'h2AA, 0, 0, 8, 64'hFFFFFFFF_FFFFFFFF, "after error");
        expect_fifo_empty(1, "corrupted frame discarded (exactly one copy)");
    endtask

    task automatic t_form_error;
        logic ok; reg [31:0] r;
        all_cfg_normal();
        send(1, 29'h0C3, 0, 0, 1, 64'h3C);
        // dominant pulse in the 3rd EOF bit -> receivers: FORM error, transmitter: BIT error
        do @(posedge clk0); while (!(dut0.u_bsp.st == 5'd15 && dut0.u_bsp.cnt == 2 && dut0.tx_pt));
        #(BIT_NS * 0.2) inject_dom = 1;
        #(BIT_NS * 0.9) inject_dom = 0;
        wait_irq(0, 6, 600, ok);   check("node0 detects error in EOF", ok);
        apb_rd(0, ECODE, r);       check_eq("node0 error = FORM error (receiver, EOF state)", {r[8:4], r[3:0]}, {5'd15, 4'b0100});
        wait_irq(1, 1, 900, ok);   check("node1 retransmits successfully", ok);
        expect_rx(0, 29'h0C3, 0, 0, 1, 64'h3C, "form");
        expect_fifo_empty(0, "form-error frame not stored");
    endtask

    task automatic t_bus_off;
        logic ok; reg [31:0] r, s; integer k;
        all_cfg_normal();
        send(0, 29'h111, 0, 0, 8, 64'hFFFFFFFF_FFFFFFFF);
        for (k = 0; k < 32; k++) inject_in_data();      // 32 x TEC+8 = 256
        wait_irq(0, 4, 600, ok);   check("node0 BUS_OFF irq after 32 bit errors", ok);
        apb_rd(0, STATUS, s);      check("STATUS.BUS_OFF", s[8]);
        apb_rd(0, ECNT, r);        check("TEC > 255", ecnt_tec(r) > 255);
        check("can_tx recessive while bus-off", can_tx[0] === 1'b1);
        wait_irq(0, 9, 4000, ok);  check("bus-off recovery after 128 x 11 recessive bits", ok);
        apb_rd(0, ECNT, r);        check_eq("TEC/REC reset after recovery", r, 0);
        wait_irq(0, 1, 900, ok);   check("pending frame transmitted after recovery", ok);
        expect_rx(1, 29'h111, 0, 0, 8, 64'hFFFFFFFF_FFFFFFFF, "after bus-off");
    endtask

    task automatic t_loopback;
        logic ok;
        node_cfg(2, 32'h4, 0, 31'h7FFFFFFF);         // internal loopback, others off
        #(BIT_NS * 12);
        node2_drove = 0;
        send(2, 29'h1FFF_FFFF, 1, 0, 8, 64'h0102030405060708);
        wait_irq(2, 1, 600, ok);  check("loopback: TX_DONE without external ACK", ok);
        expect_rx(2, 29'h1FFF_FFFF, 1, 0, 8, 64'h0102030405060708, "loopback self-rx");
        check("loopback: can_tx pin stays recessive", node2_drove == 0);
    endtask

    task automatic t_listen_only;
        logic ok;
        node_cfg(0, 0, 0, 31'h7FFFFFFF);
        node_cfg(1, 0, 0, 31'h7FFFFFFF);
        node_cfg(2, 32'h2, 0, 31'h7FFFFFFF);         // listen-only (bus monitor)
        #(BIT_NS * 12);
        node2_drove = 0;
        send(0, 29'h321, 0, 0, 3, 64'hC0FFEE);
        wait_irq(0, 1, 600, ok);  check("frame ACKed by node1", ok);
        expect_rx(2, 29'h321, 0, 0, 3, 64'hC0FFEE, "listen-only rx");
        send(2, 29'h001, 0, 0, 0, 0);                // must be ignored
        #(BIT_NS * 200);
        check("listen-only node never drove the bus (no ACK, no TX)", node2_drove == 0);
    endtask

    task automatic t_overrun;
        logic ok; reg [31:0] s; integer k;
        all_cfg_normal();
        for (k = 0; k < 5; k++) begin
            send(0, 29'h400 + k, 0, 0, 1, k);
            wait_irq(0, 1, 600, ok);
        end
        wait_irq(1, 7, 10, ok);    check("OVERRUN irq on 5th frame (FIFO depth 4)", ok);
        apb_rd(1, STATUS, s);      check("STATUS.OVERRUN and count=4", s[1] && s[11:9] == 4);
        for (k = 0; k < 4; k++) expect_rx(1, 29'h400 + k, 0, 0, 1, k, "fifo order");
        apb_wr(1, CMD, 32'h8);     apb_rd(1, STATUS, s); check("overrun cleared by CMD", !s[1]);
    endtask

    task automatic t_random_burst;
        logic ok; integer round, n, m, k;
        reg [28:0] id [0:2]; reg ide [0:2]; reg [3:0] dlc [0:2]; reg [63:0] dat [0:2];
        integer order [0:2]; integer tmp;
        reg [30:0] key_a, key_b;
        all_cfg_normal();
        for (round = 0; round < 12; round++) begin
            for (n = 0; n < 3; n++) begin
                ide[n] = $random & 1;
                id[n]  = ide[n] ? ($random & 29'h1FFF_FFFF) : ($random & 29'h7FF);
                id[n]  = ide[n] ? {id[n][28:2], n[1:0]} : {id[n][28:2], n[1:0]};  // unique per node
                dlc[n] = $random & 4'hF;
                dat[n] = {$random, $random};
                load_tx(n, id[n], ide[n], 0, dlc[n], dat[n]);
                order[n] = n;
            end
            fork apb_wr(0, CMD, 1); apb_wr(1, CMD, 1); apb_wr(2, CMD, 1); join
            for (n = 0; n < 3; n++) begin wait_irq(n, 1, 2000, ok); check($sformatf("burst r%0d node%0d TX_DONE", round, n), ok); end
            // expected bus order = arbitration priority (bit-wise compare of the arbitration field)
            for (n = 0; n < 3; n++) for (m = 0; m < 2; m++) begin
                key_a = ide[order[m]]   ? {id[order[m]][28:18],   2'b11, id[order[m]][17:0]}   : {id[order[m]][10:0],   2'b00, 18'h3FFFF};
                key_b = ide[order[m+1]] ? {id[order[m+1]][28:18], 2'b11, id[order[m+1]][17:0]} : {id[order[m+1]][10:0], 2'b00, 18'h3FFFF};
                if (key_b < key_a) begin tmp = order[m]; order[m] = order[m+1]; order[m+1] = tmp; end
            end
            for (k = 0; k < 3; k++) for (n = 0; n < 3; n++)
                if (order[k] != n) expect_rx(n, id[order[k]], ide[order[k]], 0, dlc[order[k]], dat[order[k]],
                                             $sformatf("burst r%0d", round));
        end
    endtask

    // ====================================================================== main
    initial begin
        if (!$value$plusargs("TEST=%s", testname)) testname = "ALL";
        if ($test$plusargs("VCD")) begin
            $dumpfile({"wave_", testname, ".vcd"});
            $dumpvars(0, tb_can_top.can_bus, tb_can_top.can_tx, tb_can_top.irq, tb_can_top.inject_dom);
            $dumpvars(1, tb_can_top.dut0.u_btl, tb_can_top.dut0.u_bsp, tb_can_top.dut1.u_bsp, tb_can_top.dut2.u_bsp);
            $dumpvars(1, tb_can_top.dut0.u_regs);
        end
        bitlog = $fopen({"bits_", testname, ".log"}, "w");
        #100 rst_n = 1;
        #100;
        $display("==================== TEST: %s ====================", testname);
        if (0) ;
        else if (testname == "regs") t_regs;
        else if (testname == "std_frame") t_std_frame;
        else if (testname == "ext_frame") t_ext_frame;
        else if (testname == "remote_dlc") t_remote_dlc;
        else if (testname == "arbitration") t_arbitration;
        else if (testname == "arb_std_ext") t_arb_std_ext;
        else if (testname == "filter") t_filter;
        else if (testname == "ack_error") t_ack_error;
        else if (testname == "single_shot") t_single_shot;
        else if (testname == "error_inject") t_error_inject;
        else if (testname == "form_error") t_form_error;
        else if (testname == "bus_off") t_bus_off;
        else if (testname == "loopback") t_loopback;
        else if (testname == "listen_only") t_listen_only;
        else if (testname == "overrun") t_overrun;
        else if (testname == "random_burst") t_random_burst;
        else $display("unknown test %s", testname);
        #(BIT_NS * 20);
        $fclose(bitlog);
        $display("==================== RESULT %s : %0d passed, %0d failed => %s",
                 testname, n_pass, n_fail, (n_fail == 0 && n_pass > 0) ? "TEST PASSED" : "TEST FAILED");
        $finish;
    end

    initial begin #60_000_000; $display("GLOBAL TIMEOUT"); $display("RESULT => TEST FAILED"); $finish; end
endmodule
