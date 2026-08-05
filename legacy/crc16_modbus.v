`timescale 1ns / 1ps
//======================================================================
// crc16_modbus -- combinational one-byte CRC-16/MODBUS update.
//
//   polynomial : 0xA001 (reflected 0x8005)
//   init       : 0xFFFF  (applied by the caller, not here)
//   refin/refout: true    (this reflected form handles it)
//   xorout     : 0x0000
//
// Feed bytes one at a time: crc_out = update(crc_in, data_in). Start
// each frame with crc_in = 0xFFFF. A received frame is valid when the
// running CRC over ALL bytes (payload + the 2 appended CRC bytes) is 0.
//
// Verified against the standard vector: CRC of ASCII "123456789" = 0x4B37.
//======================================================================
module crc16_modbus (
    input  wire [15:0] crc_in,
    input  wire [7:0]  data_in,
    output wire [15:0] crc_out
);
    function [15:0] crc_next;
        input [15:0] c;
        input [7:0]  d;
        integer i;
        begin
            c = c ^ {8'h00, d};
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0]) c = (c >> 1) ^ 16'hA001;
                else      c = (c >> 1);
            end
            crc_next = c;
        end
    endfunction

    assign crc_out = crc_next(crc_in, data_in);
endmodule
