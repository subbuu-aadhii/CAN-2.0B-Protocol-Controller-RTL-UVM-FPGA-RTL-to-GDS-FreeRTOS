// -----------------------------------------------------------------------------
// can_bsp.v  -  CAN 2.0A/2.0B Bit Stream Processor + Fault Confinement
//
//  Architecture ("unified transceiver FSM"):
//   * The node ALWAYS receives what is on the bus, including its own bits.
//     One field FSM is advanced at every sample point by the destuffed bus bit.
//   * When the node is the transmitter (tx_active) the bit to drive in the NEXT
//     bit time is derived from the FSM's next position (header/data index 'pos',
//     CRC register, stuff state) and registered as tx_next; it is put on the bus
//     at the next tx point (start of bit) by the BTL.
//   * Transmit bit == received bit check gives: arbitration (recessive sent,
//     dominant seen inside the arbitration field) and bit errors.
//
//  Implements: SOF, std (11b) / ext (29b) IDs, SRR/IDE/RTR/r1/r0, DLC, 0..8 data
//  bytes, CRC-15, CRC/ACK delimiters, ACK slot, EOF, intermission, suspend
//  transmission, bit stuffing/destuffing (SOF..CRC), arbitration, the five error
//  types (bit, stuff, CRC, form, ACK), active/passive error flags, overload
//  frames, TEC/REC rules, error-warning, error-passive, bus-off and bus-off
//  recovery (128 x 11 recessive bits), bus integration, automatic
//  retransmission, single-shot mode, listen-only and internal loopback.
// -----------------------------------------------------------------------------
`default_nettype none
module can_bsp (
    input  wire        clk,
    input  wire        rst_n,
    // timing (from BTL)
    input  wire        sample_pt,
    input  wire        tx_pt,
    input  wire        rx_bit,          // bus bit sampled at sample_pt
    output reg         tx_bit,          // bit driven this bit time (1 = recessive)
    output wire        hard_sync_en,
    // mode
    input  wire        reset_mode,
    input  wire        listen_only,
    input  wire        loopback,
    input  wire        single_shot,
    // transmit buffer
    input  wire        tx_req,          // a frame is pending
    input  wire        tx_abort,        // abort requested by host
    input  wire [28:0] tx_id,
    input  wire        tx_ide,
    input  wire        tx_rtr,
    input  wire [3:0]  tx_dlc,
    input  wire [63:0] tx_data,         // [63:56] = data byte 0 (sent first)
    output reg         tx_done_p,       // frame transmitted successfully
    output reg         tx_fail_p,       // single-shot / aborted attempt failed
    output reg         arb_lost_p,
    output reg  [6:0]  arb_lost_pos,    // bit position (from SOF=0) where arb. was lost
    output wire        tx_busy,
    // receive
    output reg         rx_valid_p,      // a valid frame has been received
    output reg  [28:0] rx_id,
    output reg         rx_ide,
    output reg         rx_rtr,
    output reg  [3:0]  rx_dlc,
    output reg  [63:0] rx_data,
    output wire        rx_busy,
    // fault confinement
    output reg  [8:0]  tec,
    output reg  [7:0]  rec,
    output wire        err_warning,
    output wire        err_passive,
    output wire        bus_off,
    output reg         bus_err_p,       // any protocol error detected
    output reg  [2:0]  err_type,        // 1 bit 2 stuff 3 crc 4 form 5 ack
    output reg         err_dir_tx,      // error detected while transmitting
    output reg  [4:0]  err_state,       // FSM state when error was detected
    output reg         busoff_enter_p,
    output reg         busoff_recover_p,
    output reg         overload_p,
    output wire [4:0]  fsm_state
);
    // ------------------------------------------------------------- states ---
    localparam [4:0]
        S_INTEG    = 5'd0,  S_IDLE    = 5'd1,  S_ID_A     = 5'd2,  S_RTR_SRR = 5'd3,
        S_IDE      = 5'd4,  S_ID_B    = 5'd5,  S_RTR_EXT  = 5'd6,  S_R1      = 5'd7,
        S_R0       = 5'd8,  S_DLC     = 5'd9,  S_DATA     = 5'd10, S_CRC     = 5'd11,
        S_CRC_DLM  = 5'd12, S_ACK     = 5'd13, S_ACK_DLM  = 5'd14, S_EOF     = 5'd15,
        S_INTERM   = 5'd16, S_SUSPEND = 5'd17, S_ERR_FLAG = 5'd18, S_ERR_WAIT= 5'd19,
        S_ERR_DLM  = 5'd20, S_BUSOFF  = 5'd21;

    localparam [2:0] E_NONE = 3'd0, E_BIT = 3'd1, E_STUFF = 3'd2, E_CRC = 3'd3,
                     E_FORM = 3'd4, E_ACK = 3'd5;

    // ------------------------------------------------------------ registers --
    reg  [4:0]  st;
    reg  [6:0]  cnt;          // bit counter inside a field
    reg  [6:0]  pos;          // destuffed bit index from SOF (for tx vector)
    reg  [2:0]  stf_cnt;      // consecutive equal bits (stuff rule)
    reg         stf_last;     // value of the last bus bit in the stuffed region
    reg         stf_pend;     // next bit is a stuff bit
    reg  [14:0] crc;          // running CRC
    reg  [14:0] rx_crc;       // received CRC field
    reg         crc_ok;
    reg         tx_active;    // this node is the transmitter of the current frame
    reg         tx_next;      // bit to drive at next tx point
    reg         is_ovl;       // current flag is an overload flag
    reg         suspend_req;  // error-passive transmitter must suspend after frame
    reg  [6:0]  boff_seq;     // bus-off recovery: count of 11-recessive sequences
    reg  [6:0]  dbits;        // number of data bits of the current frame
    reg         rtr_tmp;
    reg  [28:0] id_sh;
    reg  [2:0]  rx_dlc_sh;    // DLC shift register (3 MSBs; 4th bit taken live)

    assign fsm_state    = st;
    assign err_passive  = (tec > 9'd127) || (rec > 8'd127);
    assign err_warning  = (tec >= 9'd96) || (rec >= 8'd96);
    assign bus_off      = (st == S_BUSOFF);
    assign hard_sync_en = (st == S_IDLE) || (st == S_SUSPEND) ||
                          (st == S_INTERM && cnt == 7'd2);
    assign rx_busy      = (st >= S_ID_A) && (st <= S_EOF);
    assign tx_busy      = tx_active || ((st == S_IDLE) && (!tx_next || !tx_bit));

    wire   tx_go = tx_req && !tx_abort && !listen_only && !reset_mode;

    // -------------------------------------------------- transmit bit vector --
    // txv[0] = SOF ... header ... data (max 103 bits). Index = destuffed position.
    reg [127:0] txv;
    integer i;
    always @* begin
        txv = {128{1'b1}};
        txv[0] = 1'b0;                                          // SOF
        if (!tx_ide) begin
            for (i = 0; i < 11; i = i + 1) txv[1+i]  = tx_id[10-i];
            txv[12] = tx_rtr; txv[13] = 1'b0; txv[14] = 1'b0;   // RTR IDE r0
            for (i = 0; i < 4;  i = i + 1) txv[15+i] = tx_dlc[3-i];
            for (i = 0; i < 64; i = i + 1) txv[19+i] = tx_data[63-i];
        end else begin
            for (i = 0; i < 11; i = i + 1) txv[1+i]  = tx_id[28-i];
            txv[12] = 1'b1; txv[13] = 1'b1;                      // SRR IDE
            for (i = 0; i < 18; i = i + 1) txv[14+i] = tx_id[17-i];
            txv[32] = tx_rtr; txv[33] = 1'b0; txv[34] = 1'b0;   // RTR r1 r0
            for (i = 0; i < 4;  i = i + 1) txv[35+i] = tx_dlc[3-i];
            for (i = 0; i < 64; i = i + 1) txv[39+i] = tx_data[63-i];
        end
    end

    // ---------------------------------------------------------------- CRC ----
    wire [14:0] crc_step;
    can_crc15 u_crc (.crc_in(crc), .bit_in(rx_bit), .crc_out(crc_step));

    // ------------------------------------------------- next-state variables --
    reg  [4:0]  n_st;   reg [6:0] n_cnt;  reg [6:0] n_pos;
    reg  [2:0]  n_stf_cnt; reg n_stf_last; reg n_stf_pend;
    reg  [14:0] n_crc;  reg [14:0] n_rx_crc; reg n_crc_ok;
    reg         n_tx_active; reg n_is_ovl; reg n_suspend;
    reg  [6:0]  n_dbits; reg n_rtr_tmp; reg [28:0] n_id_sh;
    reg  [8:0]  n_tec;  reg [7:0] n_rec; reg [6:0] n_boff;
    reg  [2:0]  err;    reg ovl; reg is_stuff; reg do_sof; reg sof_tx_ok;
    reg         p_txdone, p_txfail, p_arb, p_rxvalid, p_boff_in, p_boff_out;
    reg  [3:0]  dlc_new;
    reg         b;
    reg         in_stuffed, in_arb, monitored, tx_n;

    // Next-state values are computed with blocking assignments into the n_*
    // temporaries (used only inside this block) and committed with <=.
    /* verilator lint_off BLKSEQ */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_INTEG; cnt <= 7'd0; pos <= 7'd0;
            stf_cnt <= 3'd0; stf_last <= 1'b1; stf_pend <= 1'b0;
            crc <= 15'd0; rx_crc <= 15'd0; crc_ok <= 1'b0;
            tx_active <= 1'b0; tx_next <= 1'b1; tx_bit <= 1'b1;
            is_ovl <= 1'b0; suspend_req <= 1'b0; boff_seq <= 7'd0;
            dbits <= 7'd0; rtr_tmp <= 1'b0; id_sh <= 29'd0; rx_dlc_sh <= 3'd0;
            tec <= 9'd0; rec <= 8'd0;
            tx_done_p <= 1'b0; tx_fail_p <= 1'b0; arb_lost_p <= 1'b0; arb_lost_pos <= 7'd0;
            rx_valid_p <= 1'b0; rx_id <= 29'd0; rx_ide <= 1'b0; rx_rtr <= 1'b0;
            rx_dlc <= 4'd0; rx_data <= 64'd0;
            bus_err_p <= 1'b0; err_type <= E_NONE; err_dir_tx <= 1'b0; err_state <= 5'd0;
            busoff_enter_p <= 1'b0; busoff_recover_p <= 1'b0; overload_p <= 1'b0;
        end else begin
            tx_done_p <= 1'b0; tx_fail_p <= 1'b0; arb_lost_p <= 1'b0; rx_valid_p <= 1'b0;
            bus_err_p <= 1'b0; busoff_enter_p <= 1'b0; busoff_recover_p <= 1'b0;
            overload_p <= 1'b0;

            // ---------------- drive point -----------------------------------
            if (tx_pt) tx_bit <= (listen_only || reset_mode) ? 1'b1 : tx_next;

            if (reset_mode) begin
                // controller held in reset/configuration mode
                st <= S_INTEG; cnt <= 7'd0; tx_active <= 1'b0; tx_next <= 1'b1;
                tx_bit <= 1'b1; stf_pend <= 1'b0; is_ovl <= 1'b0; suspend_req <= 1'b0;
            end
            else if (sample_pt) begin
                // ------------ defaults: hold everything --------------------
                b = rx_bit;
                n_st = st; n_cnt = cnt; n_pos = pos;
                n_stf_cnt = stf_cnt; n_stf_last = stf_last; n_stf_pend = stf_pend;
                n_crc = crc; n_rx_crc = rx_crc; n_crc_ok = crc_ok;
                n_tx_active = tx_active; n_is_ovl = is_ovl; n_suspend = suspend_req;
                n_dbits = dbits; n_rtr_tmp = rtr_tmp; n_id_sh = id_sh;
                n_tec = tec; n_rec = rec; n_boff = boff_seq;
                err = E_NONE; ovl = 1'b0; is_stuff = 1'b0; do_sof = 1'b0;
                p_txdone = 1'b0; p_txfail = 1'b0; p_arb = 1'b0; p_rxvalid = 1'b0;
                p_boff_in = 1'b0; p_boff_out = 1'b0; dlc_new = 4'd0;

                in_stuffed = (st >= S_ID_A) && (st <= S_CRC);
                in_arb     = (st == S_ID_A) || (st == S_RTR_SRR) || (st == S_IDE) ||
                             (st == S_ID_B) || (st == S_RTR_EXT);
                monitored  = (st >= S_ID_A) && (st <= S_EOF) && (st != S_ACK);

                // ------------ bit monitoring (transmitter) ------------------
                if (tx_active && monitored && (b != tx_bit)) begin
                    if (tx_bit && !b && in_arb && !stf_pend) begin
                        n_tx_active = 1'b0;                 // lost arbitration
                        p_arb = 1'b1;
                        if (single_shot || tx_abort) p_txfail = 1'b1;
                    end else begin
                        err = E_BIT;
                    end
                end

                // ------------ destuffing -----------------------------------
                if (stf_pend) begin
                    is_stuff   = 1'b1;
                    n_stf_pend = 1'b0;
                    if (b == stf_last && err == E_NONE) err = E_STUFF;
                    n_stf_last = b;
                    n_stf_cnt  = 3'd1;
                end
                else if (err == E_NONE) begin
                    // stuff counter (only inside SOF..CRC)
                    if (in_stuffed) begin
                        if (b == stf_last) n_stf_cnt = stf_cnt + 3'd1;
                        else begin n_stf_cnt = 3'd1; n_stf_last = b; end
                        if (n_stf_cnt == 3'd5) n_stf_pend = 1'b1;
                        if (st != S_CRC) n_crc = crc_step;
                        if (st <= S_DATA) n_pos = pos + 7'd1;
                    end

                    // ------------ field FSM ---------------------------------
                    case (st)
                    S_INTEG: begin            // need 11 recessive bits
                        if (b) begin
                            n_cnt = cnt + 7'd1;
                            if (cnt == 7'd10) begin n_st = S_IDLE; n_cnt = 7'd0; end
                        end else n_cnt = 7'd0;
                    end
                    S_IDLE:    if (!b) do_sof = 1'b1;
                    S_ID_A: begin
                        n_id_sh = {id_sh[27:0], b};
                        n_cnt = cnt + 7'd1;
                        if (cnt == 7'd10) begin n_st = S_RTR_SRR; n_cnt = 7'd0; end
                    end
                    S_RTR_SRR: begin n_rtr_tmp = b; n_st = S_IDE; end
                    S_IDE: begin
                        if (b) begin n_st = S_ID_B; n_cnt = 7'd0; end
                        else   n_st = S_R0;
                    end
                    S_ID_B: begin
                        n_id_sh = {id_sh[27:0], b};
                        n_cnt = cnt + 7'd1;
                        if (cnt == 7'd17) begin n_st = S_RTR_EXT; n_cnt = 7'd0; end
                    end
                    S_RTR_EXT: begin n_rtr_tmp = b; n_st = S_R1; end
                    S_R1:      n_st = S_R0;
                    S_R0:      begin n_st = S_DLC; n_cnt = 7'd0; end
                    S_DLC: begin
                        n_cnt = cnt + 7'd1;
                        if (cnt == 7'd3) begin
                            dlc_new = {rx_dlc_sh[2:0], b};
                            n_dbits = rtr_tmp ? 7'd0 :
                                      (dlc_new > 4'd8) ? 7'd64 : {dlc_new, 3'b000};
                            n_cnt = 7'd0;
                            n_st  = (n_dbits == 7'd0) ? S_CRC : S_DATA;
                        end
                    end
                    S_DATA: begin
                        n_cnt = cnt + 7'd1;
                        if (cnt == dbits - 7'd1) begin n_st = S_CRC; n_cnt = 7'd0; end
                    end
                    S_CRC: begin
                        n_rx_crc = {rx_crc[13:0], b};
                        n_cnt = cnt + 7'd1;
                        if (cnt == 7'd14) begin
                            n_crc_ok = ({rx_crc[13:0], b} == crc);
                            n_st = S_CRC_DLM; n_cnt = 7'd0;
                        end
                    end
                    S_CRC_DLM: begin
                        if (!b) err = E_FORM; else n_st = S_ACK;
                    end
                    S_ACK: begin
                        if (tx_active && b && !loopback) err = E_ACK;
                        else n_st = S_ACK_DLM;
                    end
                    S_ACK_DLM: begin
                        if (!b) err = E_FORM;
                        else if (!crc_ok) err = E_CRC;    // CRC error flag starts after ACK delimiter
                        else begin n_st = S_EOF; n_cnt = 7'd0; end
                    end
                    S_EOF: begin
                        if (!b) begin
                            if (cnt == 7'd6 && !tx_active) ovl = 1'b1;  // last EOF bit: overload
                            else err = E_FORM;
                        end else begin
                            n_cnt = cnt + 7'd1;
                            if (cnt == 7'd5 && (!tx_active || loopback)) begin
                                p_rxvalid = 1'b1;                    // receiver: frame valid
                                if (!tx_active && !listen_only)
                                    n_rec = (rec > 8'd127) ? 8'd120 :
                                            (rec != 8'd0)  ? rec - 8'd1 : 8'd0;
                            end
                            if (cnt == 7'd6) begin
                                n_st = S_INTERM; n_cnt = 7'd0;
                                if (tx_active) begin
                                    p_txdone    = 1'b1;
                                    n_tec       = (tec != 9'd0) ? tec - 9'd1 : 9'd0;
                                    n_suspend   = err_passive;
                                    n_tx_active = 1'b0;
                                end
                            end
                        end
                    end
                    S_INTERM: begin
                        if (!b) begin
                            if (cnt == 7'd2) do_sof = 1'b1;   // 3rd bit dominant = SOF
                            else ovl = 1'b1;
                        end else begin
                            n_cnt = cnt + 7'd1;
                            if (cnt == 7'd2) begin
                                n_cnt = 7'd0;
                                n_st  = suspend_req ? S_SUSPEND : S_IDLE;
                                n_suspend = 1'b0;
                            end
                        end
                    end
                    S_SUSPEND: begin
                        if (!b) do_sof = 1'b1;                // another node started
                        else begin
                            n_cnt = cnt + 7'd1;
                            if (cnt == 7'd7) begin n_st = S_IDLE; n_cnt = 7'd0; end
                        end
                    end
                    S_ERR_FLAG: begin
                        n_cnt = cnt + 7'd1;
                        if (cnt == 7'd5) begin n_st = S_ERR_WAIT; n_cnt = 7'd0; end
                    end
                    S_ERR_WAIT: begin                       // other nodes' flags (superposition)
                        if (b) begin n_st = S_ERR_DLM; n_cnt = 7'd1; end
                        else begin
                            n_cnt = cnt + 7'd1;
                            if (cnt == 7'd13) err = E_FORM;   // > 14 dominant bits after flag
                        end
                    end
                    S_ERR_DLM: begin                        // 8 recessive bits
                        if (!b) begin
                            if (cnt == 7'd7) ovl = 1'b1; else err = E_FORM;
                        end else begin
                            n_cnt = cnt + 7'd1;
                            if (cnt == 7'd7) begin n_st = S_INTERM; n_cnt = 7'd0; n_is_ovl = 1'b0; end
                        end
                    end
                    S_BUSOFF: begin                         // 128 x 11 recessive bits
                        if (b) begin
                            n_cnt = cnt + 7'd1;
                            if (cnt == 7'd10) begin
                                n_cnt  = 7'd0;
                                n_boff = boff_seq + 7'd1;
                                if (boff_seq == 7'd127) begin
                                    n_st = S_IDLE; n_tec = 9'd0; n_rec = 8'd0;
                                    n_boff = 7'd0; p_boff_out = 1'b1;
                                end
                            end
                        end else n_cnt = 7'd0;
                    end
                    default: n_st = S_INTEG;
                    endcase
                end

                // ------------ start of frame -------------------------------
                if (do_sof) begin
                    sof_tx_ok   = tx_go && (st != S_SUSPEND) && !suspend_req;
                    n_st        = S_ID_A; n_cnt = 7'd0; n_pos = 7'd1;
                    n_stf_cnt   = 3'd1;   n_stf_last = 1'b0; n_stf_pend = 1'b0;
                    n_crc       = 15'd0;  // CRC of the SOF bit (0) is 0
                    n_rx_crc    = 15'd0;  n_crc_ok = 1'b0;
                    n_tx_active = sof_tx_ok;
                    n_id_sh     = 29'd0;
                    n_suspend   = 1'b0;
                end

                // ------------ overload -------------------------------------
                if (ovl && err == E_NONE) begin
                    n_st = S_ERR_FLAG; n_cnt = 7'd0; n_is_ovl = 1'b1; n_stf_pend = 1'b0;
                end

                // ------------ error handling / fault confinement -----------
                if (err != E_NONE) begin
                    n_st = S_ERR_FLAG; n_cnt = 7'd0; n_is_ovl = 1'b0; n_stf_pend = 1'b0;
                    if (!listen_only) begin
                        if (tx_active) begin
                            // exception: error-passive transmitter + ACK error -> no change
                            if (!(err == E_ACK && err_passive))
                                n_tec = tec + 9'd8;            // max 263, bus-off above 255
                            if (single_shot || tx_abort) p_txfail = 1'b1;
                        end else if (st != S_ERR_WAIT && st != S_ERR_DLM) begin
                            n_rec = (rec == 8'd255) ? 8'd255 : rec + 8'd1;
                        end
                    end
                    n_tx_active = 1'b0;              // pending frame is retried later
                    bus_err_p   <= 1'b1;
                    err_type    <= err;
                    err_dir_tx  <= tx_active;
                    err_state   <= st;
                end

                // ------------ bus-off entry --------------------------------
                if (n_tec > 9'd255 && st != S_BUSOFF) begin
                    n_st = S_BUSOFF; n_cnt = 7'd0; n_boff = 7'd0; n_tx_active = 1'b0;
                    p_boff_in = 1'b1;
                end

                // ------------ next transmit bit ----------------------------
                tx_n = 1'b1;
                if (n_stf_pend) tx_n = n_tx_active ? ~n_stf_last : 1'b1;
                else case (n_st)
                    S_IDLE:     tx_n = tx_go ? 1'b0 : 1'b1;        // SOF
                    S_ID_A, S_RTR_SRR, S_IDE, S_ID_B, S_RTR_EXT, S_R1, S_R0, S_DLC, S_DATA:
                                tx_n = n_tx_active ? txv[n_pos] : 1'b1;
                    S_CRC:      tx_n = n_tx_active ? n_crc[4'd14 - n_cnt[3:0]] : 1'b1;
                    S_ACK:      tx_n = (!n_tx_active && n_crc_ok && !listen_only) ? 1'b0 : 1'b1;
                    S_ERR_FLAG: tx_n = (n_is_ovl || !(n_tec > 9'd127 || n_rec > 8'd127)) ? 1'b0 : 1'b1;
                    default:    tx_n = 1'b1;
                endcase

                // ------------ commit ---------------------------------------
                st <= n_st; cnt <= n_cnt; pos <= n_pos;
                stf_cnt <= n_stf_cnt; stf_last <= n_stf_last; stf_pend <= n_stf_pend;
                crc <= n_crc; rx_crc <= n_rx_crc; crc_ok <= n_crc_ok;
                tx_active <= n_tx_active; is_ovl <= n_is_ovl; suspend_req <= n_suspend;
                dbits <= n_dbits; rtr_tmp <= n_rtr_tmp; id_sh <= n_id_sh;
                tec <= n_tec; rec <= n_rec; boff_seq <= n_boff;
                tx_next <= tx_n;
                tx_done_p <= p_txdone; tx_fail_p <= p_txfail;
                arb_lost_p <= p_arb; if (p_arb) arb_lost_pos <= pos;
                busoff_enter_p <= p_boff_in; busoff_recover_p <= p_boff_out;
                overload_p <= ovl && (err == E_NONE);

                // ------------ received frame fields ------------------------
                if (do_sof) begin
                    rx_data <= 64'd0;
                end
                if (!is_stuff && err == E_NONE) begin
                    if (st == S_IDE) begin
                        rx_ide <= b;
                        if (!b) begin rx_rtr <= rtr_tmp; rx_id <= {18'd0, id_sh[10:0]}; end
                    end
                    if (st == S_RTR_EXT) begin rx_rtr <= b; rx_id <= id_sh; end
                    if (st == S_DLC)  rx_dlc_sh <= {rx_dlc_sh[1:0], b};
                    if (st == S_DLC && cnt == 7'd3) rx_dlc <= {rx_dlc_sh[2:0], b};
                    if (st == S_DATA) rx_data[6'd63 - cnt[5:0]] <= b;
                end
                rx_valid_p <= p_rxvalid;
            end
        end
    end
    /* verilator lint_on BLKSEQ */
endmodule
`default_nettype wire
