`timescale 1ns / 1ps
//======================================================================
// crc16_modbus_tb -- verifies CRC-16/MODBUS against the standard
// reference vector and a real Modbus frame's appended-CRC property.
//======================================================================
module crc16_modbus_tb;

    reg  [15:0] crc_in;
    reg  [7:0]  data_in;
    wire [15:0] crc_out;

    integer pass_count = 0;
    integer fail_count = 0;
    integer i;

    crc16_modbus uut (.crc_in(crc_in), .data_in(data_in), .crc_out(crc_out));

    // Combinationally chain the CRC over a byte array by feeding bytes
    // through the DUT and carrying crc_in forward (no clock needed).
    reg [15:0] crc;
    task crc_over;
        input integer n;
        input [8*16-1:0] bytes;  // up to 16 bytes, byte 0 in the MSBs
        integer k;
        reg [7:0] b;
        begin
            crc = 16'hFFFF;
            for (k = 0; k < n; k = k + 1) begin
                b = bytes[8*(16-1-k) +: 8];
                crc_in  = crc;
                data_in = b;
                #1;                 // let the combinational DUT settle
                crc = crc_out;
            end
        end
    endtask

    task check16;
        input [15:0] got;
        input [15:0] exp;
        input [8*24-1:0] label;
        begin
            if (got === exp) begin
                $display("PASS [%0s]: 0x%04h", label, got);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL [%0s]: expected 0x%04h, got 0x%04h", label, exp, got);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        // --- Test 1: the canonical CRC-16/MODBUS reference vector ---
        // "123456789" (0x31..0x39) must yield 0x4B37.
        crc_over(9, {8'h31,8'h32,8'h33,8'h34,8'h35,8'h36,8'h37,8'h38,8'h39,
                     56'h0});
        check16(crc, 16'h4B37, "ref 123456789");

        // --- Test 2: a real Modbus read request (independent value) ---
        // slave 0x01, func 0x04, addr 0x0009, count 0x0002.
        // CRC = 0xC9A1 (appended little-endian as lo 0xA1, hi 0xC9).
        crc_over(6, {8'h01,8'h04,8'h00,8'h09,8'h00,8'h02, 80'h0});
        check16(crc, 16'hC9A1, "read req 01 04 0009 0002");

        // --- Test 3: appended-CRC property ---
        // Feeding a full frame (payload + its little-endian CRC lo,hi)
        // through the CRC must return 0 -- this is how the master will
        // validate received frames.
        crc_over(8, {8'h01,8'h04,8'h00,8'h09,8'h00,8'h02,8'hA1,8'hC9, 64'h0});
        check16(crc, 16'h0000, "full-frame CRC == 0");

        $display("RESULT: %0d/%0d checks passed", pass_count, pass_count + fail_count);
        $finish;
    end

endmodule
