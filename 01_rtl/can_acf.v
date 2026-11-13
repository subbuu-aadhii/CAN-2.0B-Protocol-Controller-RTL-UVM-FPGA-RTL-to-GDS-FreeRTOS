// -----------------------------------------------------------------------------
// can_acf.v  -  Acceptance filter (code / mask)
//   key  = { IDE, RTR, ID[28:0] }  (standard frames: ID[10:0] in the low bits)
//   pass = ((key ^ ACR) & ~AMR) == 0     AMR bit = 1 -> "don't care"
//   After reset AMR = all ones -> every frame is accepted.
// -----------------------------------------------------------------------------
`default_nettype none
module can_acf (
    input  wire [28:0] id,
    input  wire        ide,
    input  wire        rtr,
    input  wire [30:0] acr,
    input  wire [30:0] amr,
    output wire        pass
);
    wire [30:0] key = {ide, rtr, ide ? id : {18'd0, id[10:0]}};
    assign pass = ~|((key ^ acr) & ~amr);
endmodule
`default_nettype wire
