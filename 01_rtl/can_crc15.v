// -----------------------------------------------------------------------------
// can_crc15.v  -  One-bit step of the CAN CRC-15 (ISO 11898-1)
//   Generator polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
//   = 0x4599 (x^15 term implicit). Computed over the destuffed bit stream
//   from SOF up to the end of the data field, register initialised to 0.
//   Purely combinational: crc_out = CRC after shifting in 'bit_in'.
// -----------------------------------------------------------------------------
`default_nettype none
module can_crc15 (
    input  wire [14:0] crc_in,
    input  wire        bit_in,
    output wire [14:0] crc_out
);
    wire fb = bit_in ^ crc_in[14];                  // feedback = new bit XOR MSB
    assign crc_out = {crc_in[13:0], 1'b0} ^ (fb ? 15'h4599 : 15'h0000);
endmodule
`default_nettype wire
