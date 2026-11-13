// -----------------------------------------------------------------------------
// can_fpga_top.v - Basys-3 (Artix-7 xc7a35t) demonstrator for the CAN 2.0B IP
//
//   node A : can_top + hardware host (APB master FSM) that sends a frame every
//            ~5 ms: ID = sw[10:0] (sw[11] = extended ID), DLC 4, data = counter
//   node B : can_top + hardware host that polls its RX FIFO and shows
//            data byte 0 on led[7:0] and ID[3:0] on led[11:8]
//   bus    : internal wired-AND of A, B and the external PMOD RX pin
//            (JA1 = CAN TX to transceiver, JA2 = CAN RX from transceiver)
//   sw[15] : 1 = node A internal loopback (self test, nothing on the bus)
//   led[15]: toggles on every frame received by node B
//   led[14]: node A error-passive    led[13]: node A bus-off
//   led[12]: node B RX FIFO overrun seen
// -----------------------------------------------------------------------------
`default_nettype none
module can_fpga_top #(parameter [22:0] TX_PERIOD = 23'd500_000) (
    input  wire        clk,          // 100 MHz
    input  wire        btnC,         // reset (active high)
    input  wire [15:0] sw,
    output wire [15:0] led,
    output wire        can_tx_pin,   // PMOD JA1
    input  wire        can_rx_pin    // PMOD JA2
);
    // reset synchroniser
    reg [2:0] rst_sr = 3'b000;
    always @(posedge clk) rst_sr <= {rst_sr[1:0], ~btnC};
    wire rst_n = rst_sr[2];

    // external RX synchroniser (the IP also synchronises; this keeps the pin path clean)
    reg rx_m = 1'b1, rx_s = 1'b1;
    always @(posedge clk) begin rx_m <= can_rx_pin; rx_s <= rx_m; end

    wire a_tx, b_tx;
    wire bus = a_tx & b_tx & rx_s;
    assign can_tx_pin = a_tx & b_tx;

    // ---------------------------------------------------------------- node A
    wire        a_psel, a_pen, a_pwr; wire [7:0] a_paddr; wire [31:0] a_pwdata, a_prdata;
    wire [31:0] a_status;
    can_host #(.ROLE(0), .PERIOD(TX_PERIOD)) u_host_a (
        .clk(clk), .rst_n(rst_n), .sw(sw), .psel(a_psel), .penable(a_pen), .pwrite(a_pwr),
        .paddr(a_paddr), .pwdata(a_pwdata), .prdata(a_prdata),
        .rx_byte(), .rx_id(), .rx_toggle(), .status(a_status));
    can_top u_node_a (
        .clk(clk), .rst_n(rst_n), .psel(a_psel), .penable(a_pen), .pwrite(a_pwr), .paddr(a_paddr),
        .pwdata(a_pwdata), .prdata(a_prdata), .pready(), .pslverr(), .irq(),
        .can_rx(bus), .can_tx(a_tx));

    // ---------------------------------------------------------------- node B
    wire        b_psel, b_pen, b_pwr; wire [7:0] b_paddr; wire [31:0] b_pwdata, b_prdata;
    wire [7:0]  rx_byte; wire [28:0] rx_id; wire rx_toggle; wire [31:0] b_status;
    can_host #(.ROLE(1), .PERIOD(TX_PERIOD)) u_host_b (
        .clk(clk), .rst_n(rst_n), .sw(sw), .psel(b_psel), .penable(b_pen), .pwrite(b_pwr),
        .paddr(b_paddr), .pwdata(b_pwdata), .prdata(b_prdata),
        .rx_byte(rx_byte), .rx_id(rx_id), .rx_toggle(rx_toggle), .status(b_status));
    can_top u_node_b (
        .clk(clk), .rst_n(rst_n), .psel(b_psel), .penable(b_pen), .pwrite(b_pwr), .paddr(b_paddr),
        .pwdata(b_pwdata), .prdata(b_prdata), .pready(), .pslverr(), .irq(),
        .can_rx(bus), .can_tx(b_tx));

    assign led[7:0]  = rx_byte;
    assign led[11:8] = rx_id[3:0];
    assign led[12]   = b_status[1];
    assign led[13]   = a_status[8];
    assign led[14]   = a_status[7];
    assign led[15]   = rx_toggle;
endmodule

// -----------------------------------------------------------------------------
// can_host - tiny hardware "CPU": an APB master sequencer that runs the same
// register-level driver flow the firmware uses (init -> TX loop / RX poll loop)
// -----------------------------------------------------------------------------
module can_host #(parameter ROLE = 0,              // 0 = transmitter, 1 = receiver
                  parameter [22:0] PERIOD = 23'd500_000) (
    input  wire        clk, rst_n,
    input  wire [15:0] sw,
    output reg         psel, penable, pwrite,
    output reg  [7:0]  paddr,
    output reg  [31:0] pwdata,
    input  wire [31:0] prdata,
    output reg  [7:0]  rx_byte,
    output reg  [28:0] rx_id,
    output reg         rx_toggle,
    output reg  [31:0] status
);
    // 100 MHz, 500 kbit/s: BRP=10 clk/tq, 20 tq (SYNC1 + TSEG1 15 + TSEG2 4), SJW 2
    localparam [31:0] BTR = (1 << 20) | (3 << 16) | (14 << 12) | 9;

    reg  [3:0]  step;
    reg  [1:0]  ph;           // 0 idle, 1 setup, 2 access
    reg  [22:0] timer;
    reg  [7:0]  counter;
    reg  [31:0] rdat;

    // APB transaction engine
    task start_wr(input [7:0] a, input [31:0] d);
        begin paddr <= a; pwdata <= d; pwrite <= 1'b1; psel <= 1'b1; penable <= 1'b0; ph <= 2'd1; end
    endtask
    task start_rd(input [7:0] a);
        begin paddr <= a; pwrite <= 1'b0; psel <= 1'b1; penable <= 1'b0; ph <= 2'd1; end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            psel <= 0; penable <= 0; pwrite <= 0; paddr <= 0; pwdata <= 0; ph <= 0;
            step <= 0; timer <= 0; counter <= 0; rdat <= 0;
            rx_byte <= 0; rx_id <= 0; rx_toggle <= 0; status <= 0;
        end else if (ph == 2'd1) begin
            penable <= 1'b1; ph <= 2'd2;
        end else if (ph == 2'd2) begin
            if (!pwrite) rdat <= prdata;
            psel <= 0; penable <= 0; ph <= 2'd0;
            step <= step + 4'd1;
        end else begin
            case (step)
            4'd0: start_wr(8'h00, 32'h1);                         // reset mode
            4'd1: start_wr(8'h04, BTR);                           // bit timing
            4'd2: start_wr(8'h14, 32'h0);                         // polling, no IRQs
            4'd3: start_wr(8'h00, (ROLE == 0) ? {28'd0, 1'b0, sw[15], 2'b00} : 32'h0);
            default:
            if (ROLE == 0) begin
                case (step)
                4'd4: if (timer == PERIOD) begin timer <= 0; step <= 4'd5; end
                      else timer <= timer + 23'd1;
                4'd5: start_wr(8'h30, sw[11] ? {1'b0, 1'b1, 1'b0, 18'h0ABCD, sw[10:0]}
                                             : {1'b0, 1'b0, 1'b0, 18'd0, sw[10:0]});
                4'd6: start_wr(8'h34, 32'd4);
                4'd7: start_wr(8'h38, {8'hC4, 8'hCA, ~counter, counter});
                4'd8: start_wr(8'h08, 32'h1);                     // TX request
                4'd9: start_rd(8'h0C);                            // STATUS
                4'd10: begin
                    status <= rdat;
                    if (rdat[2]) begin counter <= counter + 8'd1; step <= 4'd4; end   // TX buffer free
                    else step <= 4'd9;
                end
                default: step <= 4'd4;
                endcase
            end else begin
                case (step)
                4'd4: start_rd(8'h0C);                            // STATUS
                4'd5: begin status <= rdat; step <= rdat[0] ? 4'd6 : 4'd4; end
                4'd6: start_rd(8'h48);                            // RX_DATA0
                4'd7: begin rx_byte <= rdat[7:0]; start_rd(8'h40); end   // RX_ID
                4'd8: begin rx_id <= rdat[28:0]; rx_toggle <= ~rx_toggle; start_wr(8'h08, 32'h4); end
                default: step <= 4'd4;
                endcase
            end
            endcase
        end
    end
endmodule
`default_nettype wire
