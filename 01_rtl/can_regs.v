// -----------------------------------------------------------------------------
// can_regs.v  -  APB3 register block, TX buffer, RX FIFO, acceptance filter, IRQ
//
//  Offset  Name      Acc  Description
//  0x00    CTRL      RW   [0] RESET_MODE (1 after reset)  [1] LISTEN_ONLY
//                         [2] LOOPBACK (internal, self-ACK) [3] SINGLE_SHOT
//  0x04    BTR       RW*  [9:0] BRP  [15:12] TSEG1-1  [18:16] TSEG2-1  [21:20] SJW-1
//  0x08    CMD       W    [0] TX_REQ  [1] ABORT_TX  [2] RELEASE_RX  [3] CLR_OVERRUN
//  0x0C    STATUS    R    [0] RX_AVAIL [1] OVERRUN [2] TX_BUF_FREE [3] TX_COMPLETE
//                         [4] RX_BUSY [5] TX_BUSY [6] ERR_WARN [7] ERR_PASSIVE
//                         [8] BUS_OFF [11:9] RX_FIFO_COUNT [12] RESET_MODE
//                         [20:16] BSP FSM state
//  0x10    IRQ_STAT  R/W1C [0] RX [1] TX_DONE [2] ERR_WARN [3] ERR_PASSIVE
//                         [4] BUS_OFF [5] ARB_LOST [6] BUS_ERR [7] OVERRUN
//                         [8] TX_FAIL [9] BUS_OFF_RECOVERED
//  0x14    IRQ_EN    RW   same bit layout as IRQ_STAT
//  0x18    ERR_CNT   R    [8:0] TEC  [23:16] REC
//  0x1C    ERR_CODE  R    [2:0] type (1 bit,2 stuff,3 crc,4 form,5 ack) [3] during TX
//                         [8:4] FSM state  [22:16] arbitration-lost bit position
//  0x20    ACR       RW*  [30:0] acceptance code  {IDE,RTR,ID[28:0]}
//  0x24    AMR       RW*  [30:0] acceptance mask (1 = don't care)
//  0x30    TX_ID     RW   [28:0] ID (std: [10:0]) [29] RTR [30] IDE
//  0x34    TX_DLC    RW   [3:0] DLC
//  0x38    TX_DATA0  RW   bytes 0..3 (byte0 = [7:0])
//  0x3C    TX_DATA1  RW   bytes 4..7
//  0x40    RX_ID     R    head of RX FIFO: [28:0] ID [29] RTR [30] IDE
//  0x44    RX_DLC    R    [3:0] DLC
//  0x48    RX_DATA0  R    bytes 0..3
//  0x4C    RX_DATA1  R    bytes 4..7
//  0x50    VERSION   R    0xCA20_B100
//  (*) writable only while CTRL.RESET_MODE = 1.  TX_* writable only when TX buffer free.
// -----------------------------------------------------------------------------
`default_nettype none
module can_regs #(
    parameter RX_FIFO_DEPTH = 4,
    parameter RX_FIFO_AW    = 2
) (
    input  wire        clk,
    input  wire        rst_n,
    // APB3 slave
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [7:0]  paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,
    output wire        pslverr,
    output wire        irq,
    // configuration to core
    output reg         reset_mode,
    output reg         listen_only,
    output reg         loopback,
    output reg         single_shot,
    output reg  [9:0]  brp,
    output reg  [3:0]  tseg1,
    output reg  [2:0]  tseg2,
    output reg  [1:0]  sjw,
    // TX buffer to BSP
    output reg         tx_pending,
    output reg         tx_abort_req,
    output reg  [28:0] tx_id,
    output reg         tx_ide,
    output reg         tx_rtr,
    output reg  [3:0]  tx_dlc,
    output wire [63:0] tx_data,
    // status from BSP
    input  wire        tx_done_p,
    input  wire        tx_fail_p,
    input  wire        tx_busy,
    input  wire        arb_lost_p,
    input  wire [6:0]  arb_lost_pos,
    input  wire        rx_valid_p,
    input  wire [28:0] rx_id,
    input  wire        rx_ide,
    input  wire        rx_rtr,
    input  wire [3:0]  rx_dlc,
    input  wire [63:0] rx_data,
    input  wire        rx_busy,
    input  wire [8:0]  tec,
    input  wire [7:0]  rec,
    input  wire        err_warning,
    input  wire        err_passive,
    input  wire        bus_off,
    input  wire        bus_err_p,
    input  wire [2:0]  err_type,
    input  wire        err_dir_tx,
    input  wire [4:0]  err_state,
    input  wire        busoff_enter_p,
    input  wire        busoff_recover_p,
    input  wire [4:0]  fsm_state
);
    localparam A_CTRL = 8'h00, A_BTR = 8'h04, A_CMD = 8'h08, A_STATUS = 8'h0C,
               A_ISTAT = 8'h10, A_IEN = 8'h14, A_ECNT = 8'h18, A_ECODE = 8'h1C,
               A_ACR = 8'h20, A_AMR = 8'h24, A_TXID = 8'h30, A_TXDLC = 8'h34,
               A_TXD0 = 8'h38, A_TXD1 = 8'h3C, A_RXID = 8'h40, A_RXDLC = 8'h44,
               A_RXD0 = 8'h48, A_RXD1 = 8'h4C, A_VER = 8'h50;
    localparam [31:0] VERSION = 32'hCA20_B100;

    assign pready  = 1'b1;          // zero wait state
    assign pslverr = 1'b0;

    wire wr = psel & penable &  pwrite;
    wire rd = psel & penable & ~pwrite;

    reg [31:0] txd0, txd1;          // byte0 in [7:0]
    // bus order: byte0 first -> tx_data[63:56]
    assign tx_data = {txd0[7:0], txd0[15:8], txd0[23:16], txd0[31:24],
                      txd1[7:0], txd1[15:8], txd1[23:16], txd1[31:24]};

    reg [30:0] acr, amr;
    reg [9:0]  irq_stat, irq_en;
    reg        overrun, tx_complete;
    reg        warn_d, passive_d;

    // -------------------------------------------------- acceptance filter ---
    wire acf_pass;
    can_acf u_acf (.id(rx_id), .ide(rx_ide), .rtr(rx_rtr), .acr(acr), .amr(amr), .pass(acf_pass));

    // ---------------------------------------------------------- RX FIFO -----
    wire [98:0] fifo_wdata = {rx_ide, rx_rtr, rx_id, rx_dlc, rx_data};
    wire [98:0] fifo_head;
    wire        fifo_empty, fifo_full;
    wire [RX_FIFO_AW:0] fifo_cnt;
    wire [2:0]  fifo_cnt3 = fifo_cnt;          // STATUS[11:9] (depth <= 4 in this field)
    wire        fifo_wr  = rx_valid_p & acf_pass;
    wire        fifo_pop = wr && (paddr == A_CMD) && pwdata[2];

    can_fifo #(.WIDTH(99), .DEPTH(RX_FIFO_DEPTH), .AW(RX_FIFO_AW)) u_fifo (
        .clk(clk), .rst_n(rst_n), .clr(reset_mode), .wr_en(fifo_wr), .wr_data(fifo_wdata),
        .rd_en(fifo_pop), .rd_data(fifo_head), .empty(fifo_empty), .full(fifo_full),
        .count(fifo_cnt));

    wire        h_ide = fifo_head[98];
    wire        h_rtr = fifo_head[97];
    wire [28:0] h_id  = fifo_head[96:68];
    wire [3:0]  h_dlc = fifo_head[67:64];
    wire [63:0] h_dat = fifo_head[63:0];

    // ------------------------------------------------------ IRQ events ------
    wire [9:0] ev;
    assign ev[0] = fifo_wr & ~fifo_full;
    assign ev[1] = tx_done_p;
    assign ev[2] = (err_warning != warn_d);
    assign ev[3] = (err_passive != passive_d);
    assign ev[4] = busoff_enter_p;
    assign ev[5] = arb_lost_p;
    assign ev[6] = bus_err_p;
    assign ev[7] = fifo_wr & fifo_full;
    assign ev[8] = tx_fail_p | (tx_pending & tx_abort_req & ~tx_busy);
    assign ev[9] = busoff_recover_p;

    assign irq = |(irq_stat & irq_en);

    // ------------------------------------------------------ writes ----------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_mode <= 1'b1; listen_only <= 1'b0; loopback <= 1'b0; single_shot <= 1'b0;
            brp <= 10'd4; tseg1 <= 4'd14; tseg2 <= 3'd3; sjw <= 2'd1;   // 20 tq, 80% SP
            acr <= 31'd0; amr <= {31{1'b1}};
            irq_stat <= 10'd0; irq_en <= 10'd0;
            tx_pending <= 1'b0; tx_abort_req <= 1'b0;
            tx_id <= 29'd0; tx_ide <= 1'b0; tx_rtr <= 1'b0; tx_dlc <= 4'd0;
            txd0 <= 32'd0; txd1 <= 32'd0;
            overrun <= 1'b0; tx_complete <= 1'b0;
            warn_d <= 1'b0; passive_d <= 1'b0;
        end else begin
            warn_d    <= err_warning;
            passive_d <= err_passive;

            // IRQ status: set by events, cleared by writing 1
            irq_stat <= (irq_stat & ~((wr && paddr == A_ISTAT) ? pwdata[9:0] : 10'd0)) | ev;

            if (fifo_wr && fifo_full) overrun <= 1'b1;

            // TX handshake
            if (tx_done_p) begin tx_pending <= 1'b0; tx_abort_req <= 1'b0; tx_complete <= 1'b1; end
            else if (tx_fail_p) begin tx_pending <= 1'b0; tx_abort_req <= 1'b0; tx_complete <= 1'b0; end
            else if (tx_pending && tx_abort_req && !tx_busy) begin
                tx_pending <= 1'b0; tx_abort_req <= 1'b0; tx_complete <= 1'b0;
            end

            if (wr) begin
                case (paddr)
                A_CTRL: begin
                    reset_mode  <= pwdata[0];
                    listen_only <= pwdata[1];
                    loopback    <= pwdata[2];
                    single_shot <= pwdata[3];
                    if (pwdata[0]) begin tx_pending <= 1'b0; tx_abort_req <= 1'b0; end
                end
                A_BTR: if (reset_mode) begin
                    brp <= pwdata[9:0]; tseg1 <= pwdata[15:12];
                    tseg2 <= pwdata[18:16]; sjw <= pwdata[21:20];
                end
                A_CMD: begin
                    if (pwdata[0] && !tx_pending && !reset_mode) begin
                        tx_pending <= 1'b1; tx_complete <= 1'b0;
                    end
                    if (pwdata[1] && tx_pending) tx_abort_req <= 1'b1;
                    if (pwdata[3]) overrun <= 1'b0;
                end
                A_IEN:  irq_en <= pwdata[9:0];
                A_ACR:  if (reset_mode) acr <= pwdata[30:0];
                A_AMR:  if (reset_mode) amr <= pwdata[30:0];
                A_TXID: if (!tx_pending) begin
                    tx_id <= pwdata[28:0]; tx_rtr <= pwdata[29]; tx_ide <= pwdata[30];
                end
                A_TXDLC: if (!tx_pending) tx_dlc <= pwdata[3:0];
                A_TXD0:  if (!tx_pending) txd0 <= pwdata;
                A_TXD1:  if (!tx_pending) txd1 <= pwdata;
                default: ;
                endcase
            end
        end
    end

    // ------------------------------------------------------ reads -----------
    always @* begin
        prdata = 32'd0;
        if (rd) begin
            case (paddr)
            A_CTRL:   prdata = {28'd0, single_shot, loopback, listen_only, reset_mode};
            A_BTR:    prdata = {10'd0, sjw, 1'b0, tseg2, tseg1, 2'd0, brp};
            A_STATUS: prdata = {11'd0, fsm_state, 3'd0, reset_mode,
                                fifo_cnt3,
                                bus_off, err_passive, err_warning, tx_busy, rx_busy,
                                tx_complete, ~tx_pending, overrun, ~fifo_empty};
            A_ISTAT:  prdata = {22'd0, irq_stat};
            A_IEN:    prdata = {22'd0, irq_en};
            A_ECNT:   prdata = {8'd0, rec, 7'd0, tec};
            A_ECODE:  prdata = {9'd0, arb_lost_pos, 7'd0, err_state, err_dir_tx, err_type};
            A_ACR:    prdata = {1'b0, acr};
            A_AMR:    prdata = {1'b0, amr};
            A_TXID:   prdata = {1'b0, tx_ide, tx_rtr, tx_id};
            A_TXDLC:  prdata = {28'd0, tx_dlc};
            A_TXD0:   prdata = txd0;
            A_TXD1:   prdata = txd1;
            A_RXID:   prdata = {1'b0, h_ide, h_rtr, h_id};
            A_RXDLC:  prdata = {28'd0, h_dlc};
            A_RXD0:   prdata = {h_dat[39:32], h_dat[47:40], h_dat[55:48], h_dat[63:56]};
            A_RXD1:   prdata = {h_dat[7:0],   h_dat[15:8],  h_dat[23:16], h_dat[31:24]};
            A_VER:    prdata = VERSION;
            default:  prdata = 32'd0;
            endcase
        end
    end
endmodule
`default_nettype wire
