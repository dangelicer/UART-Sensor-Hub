`timescale 1ns / 1ps
//======================================================================
// main_tb.v  --  integration testbench for `main` (UART Sensor Hub top)
//
// End-to-end path exercised (UV channel):
//   TB drives a UART frame onto uv_dt
//     -> sync_2ff -> rx -> sync_fifo -> fifo_bt_streamer -> uart_tx
//     -> TB decodes the two bytes that appear on bt_tx.
//
// Oracle independence (Rule 7):
//   - Byte VALUES driven in are chosen by the TB, not read from the RTL.
//   - Expected parity is computed with an explicit XOR chain
//     (d0^d1^...^d7), a DIFFERENT expression than the DUT's `^{...}`
//     reduction, so a typo in one is not mirrored in the other.
//   - The expected BT framing {3'b0,id,parity,framing,overrun}+data is
//     built from the protocol/word spec documented in rx.v and main.v,
//     independent of how the streamer computes it.
//
// Timing: 9600 baud @ 100 MHz -> 10416 clk/bit. This equals both the
//   protocol value round(100e6/9600) and the DUT's own bit period
//   (16 oversample ticks x 651-clk baud_tick_gen period). Stimulus is
//   driven on negedge (Rule 5); bt_tx is sampled at mid-bit.
//
// NOTE: only declared DUT inputs are driven (Rule 1); no hierarchical
//   writes anywhere (Rule 2). All comparisons use === with X-guards
//   (Rule 4). The IR channel was removed from the design, so this bench
//   exercises the UV channel only.
//
// Run: iverilog -g2012 -o main_tb.out main_tb.v main.v rx.v fifo.v \
//               baud_tick_gen.v basys3_sensors.v && vvp main_tb.out
//======================================================================
module main_tb;

    localparam integer CLKS_PER_BIT = 10416;               // 9600 baud @ 100 MHz
    // The BT transmission only begins near the END of the input frame (the
    // FIFO write happens on the last received bit), and the capture thread
    // runs concurrently with the whole frame (Rule 6), so the window to see
    // a start bit must span an entire input frame plus margin.
    localparam integer START_TIMEOUT = 20 * CLKS_PER_BIT;

    localparam [1:0] UV_ID = 2'b00;

    // DUT inputs (driven) and outputs (observed)
    reg  clk;
    reg  rst;
    reg  sw_uv_off;
    reg  uv_dt;
    reg  bt_rx;
    wire uv_cr;
    wire bt_tx;
    wire [15:0] led;

    integer pass_count;
    integer fail_count;

    // capture scratch
    reg [7:0] cap_hi, cap_lo, cap_byte;
    reg       got_hi, got_lo, got_byte;

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    main dut (
        .clk       (clk),
        .rst       (rst),
        .sw_uv_off (sw_uv_off),
        .uv_dt     (uv_dt),
        .uv_cr     (uv_cr),
        .bt_tx     (bt_tx),
        .bt_rx     (bt_rx),
        .led       (led)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    //------------------------------------------------------------------
    // Independent even-parity reference: explicit XOR chain.
    // (DUT uses a reduction `^{data_byte,RX}` -- deliberately different.)
    //------------------------------------------------------------------
    function even_parity;
        input [7:0] d;
        begin
            even_parity = d[0]^d[1]^d[2]^d[3]^d[4]^d[5]^d[6]^d[7];
        end
    endfunction

    //------------------------------------------------------------------
    // Stimulus helpers (drive uv_dt only)
    //------------------------------------------------------------------
    task drive_bit;
        input b;
        begin
            @(negedge clk);
            uv_dt = b;
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    // 8N1-with-parity frame: start(0), 8 data LSB-first, parity, stop(1).
    // pbit is sent as-is so the caller can inject a good or bad parity bit.
    // Trailing idle bit gives rx time to finish STOP and write the FIFO.
    task send_uart_frame;
        input [7:0] data;
        input       pbit;
        integer     i;
        begin
            drive_bit(1'b0);
            for (i = 0; i < 8; i = i + 1) drive_bit(data[i]);
            drive_bit(pbit);
            drive_bit(1'b1);
            drive_bit(1'b1);
        end
    endtask

    // Send a frame while concurrently capturing the two BT bytes it
    // produces (Rule 6: the capture thread must be listening BEFORE the
    // streamer starts, which happens before the frame send returns).
    task send_capture2;
        input [7:0] data;
        input       pbit;
        begin
            cap_hi = 8'h00; cap_lo = 8'h00; got_hi = 1'b0; got_lo = 1'b0;
            fork
                send_uart_frame(data, pbit);
                begin
                    bt_get_byte(cap_hi, got_hi);
                    bt_get_byte(cap_lo, got_lo);
                end
            join
        end
    endtask

    // Send a frame and confirm NO BT byte comes out (disabled channel).
    task send_capture_none;
        input [7:0] data;
        input       pbit;
        begin
            cap_byte = 8'h00; got_byte = 1'b0;
            fork
                send_uart_frame(data, pbit);
                bt_get_byte(cap_byte, got_byte);
            join
        end
    endtask

    //------------------------------------------------------------------
    // Decode one UART byte off bt_tx (idle high). Returns got=0 if no
    // start bit appears within START_TIMEOUT clocks.
    //------------------------------------------------------------------
    task bt_get_byte;
        output [7:0] b;
        output       got;
        integer      i, t;
        begin
            b   = 8'h00;
            got = 1'b0;
            t   = 0;
            // wait for falling edge (start bit)
            while (bt_tx === 1'b1 && t < START_TIMEOUT) begin
                @(posedge clk);
                t = t + 1;
            end
            if (bt_tx === 1'b0) begin
                // move to the middle of the start bit, then sample each bit
                repeat (CLKS_PER_BIT/2) @(posedge clk);
                for (i = 0; i < 8; i = i + 1) begin
                    repeat (CLKS_PER_BIT) @(posedge clk);
                    b[i] = bt_tx;                 // LSB first
                end
                repeat (CLKS_PER_BIT) @(posedge clk); // ride out the stop bit
                got = 1'b1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // Checks
    //------------------------------------------------------------------
    task check_byte;
        input [7:0]  got_val;
        input        valid;
        input [7:0]  exp;
        input integer tnum;
        begin
            if (!valid) begin
                $display("FAIL [t%0d]: expected 0x%02h but no BT byte was captured (timeout)", tnum, exp);
                fail_count = fail_count + 1;
            end else if (got_val === 8'bx || (^got_val) === 1'bx) begin
                $display("FAIL [t%0d]: captured byte is X (indeterminate)", tnum);
                fail_count = fail_count + 1;
            end else if (got_val === exp) begin
                $display("PASS [t%0d]: BT byte 0x%02h", tnum, got_val);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL [t%0d]: expected 0x%02h, got 0x%02h", tnum, exp, got_val);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Build the expected BT high byte for a channel word.
    function [7:0] exp_hi;
        input [1:0] id;
        input [7:0] data;
        input       pbit;   // parity bit actually sent
        input       stopb;  // stop bit actually sent
        reg         perr, ferr;
        begin
            perr  = even_parity(data) ^ pbit;   // 0 if sent parity matches even parity
            ferr  = ~stopb;                      // framing error = stop sampled low
            exp_hi = {3'b000, id, perr, ferr, 1'b0}; // overrun = 0 (FIFO never full here)
        end
    endfunction

    //------------------------------------------------------------------
    // Reset
    //------------------------------------------------------------------
    task do_reset;
        begin
            rst = 1'b1;
            repeat (5) @(posedge clk);
            @(negedge clk);
            rst = 1'b0;
            repeat (5) @(posedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // Waveforms + global timeout
    //------------------------------------------------------------------
    initial begin
        $dumpfile("main_tb.vcd");
        $dumpvars(0, main_tb);
    end

    initial begin
        #60_000_000;
        $display("TIMEOUT: simulation ran too long");
        $display("RESULT: %0d/%0d checks passed", pass_count, pass_count + fail_count);
        $finish;
    end

    //------------------------------------------------------------------
    // Test sequence (UV channel, id 0)
    //------------------------------------------------------------------
    initial begin
        clk        = 1'b0;
        rst        = 1'b0;
        sw_uv_off  = 1'b0;   // UV enabled
        uv_dt      = 1'b1;   // UART idle high
        bt_rx      = 1'b1;   // host->hub line idle (unused)
        pass_count = 0;
        fail_count = 0;

        do_reset();
        $display("Reset complete, starting integration tests");

        // ---- Test 1: UV good frame, 0x3C ----
        // word {id=00, data=3C, parity=0, framing=0, overrun=0} -> hi=0x00, lo=0x3C.
        send_capture2(8'h3C, even_parity(8'h3C));
        check_byte(cap_hi, got_hi, exp_hi(UV_ID, 8'h3C, even_parity(8'h3C), 1'b1), 1);
        check_byte(cap_lo, got_lo, 8'h3C, 1);

        // ---- Test 2: UV good frame, 0xA5 (bit7=1 -> bug-sensitive MSB) ----
        // hi=0x00, lo=0xA5. Confirms the top data bit survives the pipeline.
        send_capture2(8'hA5, even_parity(8'hA5));
        check_byte(cap_hi, got_hi, exp_hi(UV_ID, 8'hA5, even_parity(8'hA5), 1'b1), 2);
        check_byte(cap_lo, got_lo, 8'hA5, 2);

        // ---- Test 3: UV frame with WRONG parity bit (bug-sensitive) ----
        // Send ~even_parity -> DUT must flag parity_err=1 -> hi=0x04, lo=0xA5.
        // A DUT that ignored parity would send 0x00.
        send_capture2(8'hA5, ~even_parity(8'hA5));
        check_byte(cap_hi, got_hi, exp_hi(UV_ID, 8'hA5, ~even_parity(8'hA5), 1'b1), 3);
        check_byte(cap_lo, got_lo, 8'hA5, 3);

        // ---- Test 4: UV channel DISABLED -> no BT output ----
        // sw_uv_off high gates the FIFO write; sending a full frame must
        // produce NO byte on bt_tx (got stays 0).
        sw_uv_off = 1'b1;
        @(negedge clk);
        send_capture_none(8'h5A, even_parity(8'h5A));
        if (!got_byte) begin
            $display("PASS [t4]: disabled UV channel produced no BT output");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL [t4]: disabled UV channel still emitted 0x%02h", cap_byte);
            fail_count = fail_count + 1;
        end
        sw_uv_off = 1'b0;

        // ---- Test 5: re-enable UV, confirm channel recovers ----
        send_capture2(8'h81, even_parity(8'h81)); // 0x81: bits 7 and 0 set
        check_byte(cap_hi, got_hi, exp_hi(UV_ID, 8'h81, even_parity(8'h81), 1'b1), 5);
        check_byte(cap_lo, got_lo, 8'h81, 5);

        $display("RESULT: %0d/%0d checks passed", pass_count, pass_count + fail_count);
        $finish;
    end

endmodule
