// -----------------------------------------------------------------------------
// can_btl.v  -  CAN Bit Timing Logic
//
//  One nominal bit time = SYNC_SEG (1 tq) + TSEG1 (PROP+PHASE1) + TSEG2 (PHASE2)
//      tq (time quantum) = (brp+1) system clocks
//      TSEG1 = tseg1+1 tq  (1..16)    TSEG2 = tseg2+1 tq (1..8)    SJW = sjw+1 tq (1..4)
//
//      |SYNC|<------- TSEG1 ------->|<-- TSEG2 -->|
//      ^tx_pt                       ^sample_pt
//
//  Synchronisation on recessive->dominant edges of the (synchronised) RX line:
//   * Hard sync  (bus idle / start of frame): bit time restarts, edge = SYNC_SEG.
//   * Re-sync    (inside a frame): edge in TSEG1  -> phase error e>0, TSEG1 += min(e,SJW)
//                                   edge in TSEG2  -> phase error e<0, TSEG2 -= min(|e|,SJW)
//     Only one synchronisation between two sample points. A node that is itself
//     driving a dominant bit does not resynchronise on that (its own) edge.
// -----------------------------------------------------------------------------
`default_nettype none
module can_btl (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       enable,        // 0 while controller is in reset mode
    // configuration
    input  wire [9:0] brp,           // prescaler - 1
    input  wire [3:0] tseg1,         // TSEG1 - 1
    input  wire [2:0] tseg2,         // TSEG2 - 1
    input  wire [1:0] sjw,           // SJW - 1
    // bus
    input  wire       rx,            // synchronised RX (1 = recessive)
    input  wire       tx_bit,        // what this node currently drives
    input  wire       hard_sync_en,  // from BSP: bus idle / SOF expected
    // outputs
    output reg        sample_pt,     // 1-clk pulse at the sample point
    output reg        tx_pt,         // 1-clk pulse at the start of a bit (drive point)
    output reg        sampled_bit,   // bus value captured at sample_pt
    output reg        hard_sync_p,   // debug: a hard sync happened
    output reg        resync_p       // debug: a re-sync happened
);
    localparam SEG_SYNC = 2'd0, SEG_T1 = 2'd1, SEG_T2 = 2'd2;

    reg  [9:0] clk_cnt;
    reg  [1:0] seg;
    reg  [4:0] qcnt;           // quantum counter inside the current segment
    reg  [4:0] t1_len;         // effective TSEG1 length (tq), incl. lengthening
    reg  [3:0] t2_len;         // effective TSEG2 length (tq), after shortening
    reg        rx_d;
    reg        synced;         // a synchronisation already happened in this bit

    wire       tq_tick  = (clk_cnt == brp);
    wire       edge_det = rx_d & ~rx;                 // recessive -> dominant
    wire [4:0] t1_nom   = {1'b0, tseg1} + 5'd1;
    wire [3:0] t2_nom   = {1'b0, tseg2} + 4'd1;
    wire [2:0] sjw_q    = {1'b0, sjw} + 3'd1;

    // phase error magnitudes (in tq)
    wire [4:0] e_pos    = qcnt + 5'd1;                         // edge late (in TSEG1)
    wire [4:0] e_neg    = {1'b0, t2_len} - qcnt;               // edge early (in TSEG2)
    wire [4:0] ext      = (e_pos > {2'b0, sjw_q}) ? {2'b0, sjw_q} : e_pos;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 10'd0; seg <= SEG_SYNC; qcnt <= 5'd0;
            t1_len <= 5'd2; t2_len <= 4'd1;
            rx_d <= 1'b1; synced <= 1'b0;
            sample_pt <= 1'b0; tx_pt <= 1'b0; sampled_bit <= 1'b1;
            hard_sync_p <= 1'b0; resync_p <= 1'b0;
        end else begin
            rx_d        <= rx;
            sample_pt   <= 1'b0;
            tx_pt       <= 1'b0;
            hard_sync_p <= 1'b0;
            resync_p    <= 1'b0;

            if (!enable) begin
                clk_cnt <= 10'd0; seg <= SEG_SYNC; qcnt <= 5'd0;
                t1_len <= t1_nom; t2_len <= t2_nom; synced <= 1'b0;
            end
            // ---------------- hard synchronisation ----------------
            else if (edge_det && hard_sync_en && !synced) begin
                // the quantum in which the edge occurred becomes SYNC_SEG
                clk_cnt <= 10'd0; seg <= SEG_T1; qcnt <= 5'd0;
                t1_len  <= t1_nom; t2_len <= t2_nom;
                synced  <= 1'b1;
                tx_pt   <= 1'b1;            // a new bit starts now
                hard_sync_p <= 1'b1;
            end
            // ---------------- re-synchronisation -------------------
            else if (edge_det && !hard_sync_en && !synced && tx_bit) begin
                synced   <= 1'b1;
                resync_p <= 1'b1;
                if (seg == SEG_T1) begin
                    // late edge -> lengthen phase segment 1
                    t1_len <= t1_nom + ext;
                    clk_cnt <= tq_tick ? 10'd0 : clk_cnt + 10'd1;
                    if (tq_tick) qcnt <= qcnt + 5'd1;
                end else if (seg == SEG_T2) begin
                    if (e_neg <= {2'b0, sjw_q}) begin
                        // early edge within SJW: end bit now, edge = SYNC of next bit
                        clk_cnt <= 10'd0; seg <= SEG_T1; qcnt <= 5'd0;
                        t1_len  <= t1_nom; t2_len <= t2_nom;
                        tx_pt   <= 1'b1;
                    end else begin
                        // shorten phase segment 2 by SJW
                        t2_len  <= t2_len - {1'b0, sjw_q};
                        clk_cnt <= tq_tick ? 10'd0 : clk_cnt + 10'd1;
                        if (tq_tick) qcnt <= qcnt + 5'd1;
                    end
                end else begin
                    // edge inside SYNC_SEG: no phase error, just keep counting
                    clk_cnt <= tq_tick ? 10'd0 : clk_cnt + 10'd1;
                    if (tq_tick) begin seg <= SEG_T1; qcnt <= 5'd0; end
                end
            end
            // ---------------- normal quantum counting --------------
            else if (tq_tick) begin
                clk_cnt <= 10'd0;
                case (seg)
                    SEG_SYNC: begin seg <= SEG_T1; qcnt <= 5'd0; end
                    SEG_T1: begin
                        if (qcnt + 5'd1 >= t1_len) begin
                            seg <= SEG_T2; qcnt <= 5'd0;
                            sample_pt   <= 1'b1;
                            sampled_bit <= rx;
                            synced      <= 1'b0;   // sync allowed again after sample point
                        end else qcnt <= qcnt + 5'd1;
                    end
                    default: begin // SEG_T2
                        if (qcnt + 5'd1 >= {1'b0, t2_len}) begin
                            seg <= SEG_SYNC; qcnt <= 5'd0;
                            t1_len <= t1_nom; t2_len <= t2_nom;
                            tx_pt  <= 1'b1;
                        end else qcnt <= qcnt + 5'd1;
                    end
                endcase
            end else begin
                clk_cnt <= clk_cnt + 10'd1;
            end
        end
    end
endmodule
`default_nettype wire
