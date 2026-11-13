// -----------------------------------------------------------------------------
// can_fifo.v  -  Synchronous FIFO for received frames (first-word fall-through)
// -----------------------------------------------------------------------------
`default_nettype none
module can_fifo #(
    parameter WIDTH = 98,
    parameter DEPTH = 4,            // power of two
    parameter AW    = 2
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             clr,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    input  wire             rd_en,       // pop head
    output wire [WIDTH-1:0] rd_data,     // head of FIFO
    output wire             empty,
    output wire             full,
    output reg  [AW:0]      count
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW-1:0]    wp, rp;

    assign empty   = (count == 0);
    assign full    = (count == DEPTH);
    assign rd_data = mem[rp];

    wire do_wr = wr_en && !full;
    wire do_rd = rd_en && !empty;

    always @(posedge clk) if (do_wr) mem[wp] <= wr_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= {AW{1'b0}}; rp <= {AW{1'b0}}; count <= {(AW+1){1'b0}};
        end else if (clr) begin
            wp <= {AW{1'b0}}; rp <= {AW{1'b0}}; count <= {(AW+1){1'b0}};
        end else begin
            if (do_wr) wp <= wp + 1'b1;
            if (do_rd) rp <= rp + 1'b1;
            case ({do_wr, do_rd})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule
`default_nettype wire
