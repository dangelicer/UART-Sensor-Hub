`timescale 1ns / 1ps
//======================================================================
// basys3_sensors.v -- shared UART / timing building blocks used by the
// UART Sensor Hub top-level (main.v).
//
// Only the two modules main.v actually instantiates live here:
//   - uart_tx         : 8N1 UART transmitter (UV trigger line + Bluetooth out)
//   - echo_pulse_ctrl : periodic strobe generator (paces the UV trigger)
//
// The earlier all-in-one design (uart_rx, uart_sensor_ctrl,
// basys3_uv_ir_top) was removed: main.v receives with rx.v, so that
// separate receive/trigger stack is unused.
//======================================================================


//----------------------------------------------------------------------
// from uart_tx.v
//----------------------------------------------------------------------
//======================================================================
// uart_tx  --  generic parameterized 8N1 UART transmitter
//======================================================================
module uart_tx #(
    parameter integer CLK_HZ  = 100_000_000,
    parameter integer BAUD    = 9600
) (
    input  wire       clk,
    input  wire        rst,
    input  wire        tx_start,   // pulse to send tx_data
    input  wire [7:0]  tx_data,
    output reg          tx,        // idles high
    output reg          busy
);

    localparam integer BIT_TICKS = CLK_HZ / BAUD;
    localparam integer CNT_W     = $clog2(BIT_TICKS + 1);

    localparam S_IDLE  = 2'd0,
               S_START  = 2'd1,
               S_DATA   = 2'd2,
               S_STOP   = 2'd3;

    reg [1:0]        state;
    reg [CNT_W-1:0]  tick_cnt;
    reg [2:0]        bit_idx;
    reg [7:0]        shift;

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            tx       <= 1'b1;
            busy     <= 1'b0;
            tick_cnt <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx <= 1'b1;
                    if (tx_start) begin
                        shift    <= tx_data;
                        busy     <= 1'b1;
                        tick_cnt <= 0;
                        state    <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0; // start bit
                    if (tick_cnt == BIT_TICKS - 1) begin
                        tick_cnt <= 0;
                        bit_idx  <= 3'd0;
                        state    <= S_DATA;
                    end else tick_cnt <= tick_cnt + 1'b1;
                end

                S_DATA: begin
                    tx <= shift[0];
                    if (tick_cnt == BIT_TICKS - 1) begin
                        tick_cnt <= 0;
                        shift    <= {1'b0, shift[7:1]};
                        if (bit_idx == 3'd7) state <= S_STOP;
                        else bit_idx <= bit_idx + 1'b1;
                    end else tick_cnt <= tick_cnt + 1'b1;
                end

                S_STOP: begin
                    tx <= 1'b1; // stop bit
                    if (tick_cnt == BIT_TICKS - 1) begin
                        tick_cnt <= 0;
                        busy     <= 1'b0;
                        state    <= S_IDLE;
                    end else tick_cnt <= tick_cnt + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

//----------------------------------------------------------------------
// from uart_rx.v
//----------------------------------------------------------------------
//======================================================================
// uart_rx  --  generic parameterized 8N1 UART receiver
// Samples at mid-bit; `rx_valid` pulses for one clock when a byte in
// `rx_data` is ready. Has a built-in 2-flop synchronizer on the async
// `rx` pin.
//======================================================================
module uart_rx #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 9600
) (
    input  wire       clk,
    input  wire        rst,
    input  wire        rx,          // raw pin, async to clk
    output reg  [7:0]  rx_data,
    output reg          rx_valid,
    output reg          frame_err
);

    localparam integer BIT_TICKS  = CLK_HZ / BAUD;
    localparam integer HALF_TICKS = BIT_TICKS / 2;
    localparam integer CNT_W      = $clog2(BIT_TICKS + 1);

    // 2-flop synchronizer for the async input pin
    reg rx_meta, rx_sync;
    always @(posedge clk) begin
        rx_meta <= rx;
        rx_sync <= rx_meta;
    end

    localparam S_IDLE  = 2'd0,
               S_START  = 2'd1,
               S_DATA   = 2'd2,
               S_STOP   = 2'd3;

    reg [1:0]       state;
    reg [CNT_W-1:0] tick_cnt;
    reg [2:0]       bit_idx;
    reg [7:0]       shift;

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            tick_cnt <= 0;
            rx_valid <= 1'b0;
            frame_err<= 1'b0;
        end else begin
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (!rx_sync) begin // falling edge = start bit begins
                        tick_cnt <= 0;
                        state    <= S_START;
                    end
                end

                S_START: begin
                    if (tick_cnt == HALF_TICKS - 1) begin
                        // confirm still low at the mid-point of the start bit
                        if (!rx_sync) begin
                            tick_cnt <= 0;
                            bit_idx  <= 3'd0;
                            state    <= S_DATA;
                        end else begin
                            state <= S_IDLE; // glitch, abandon
                        end
                    end else tick_cnt <= tick_cnt + 1'b1;
                end

                S_DATA: begin
                    if (tick_cnt == BIT_TICKS - 1) begin
                        tick_cnt <= 0;
                        shift    <= {rx_sync, shift[7:1]};
                        if (bit_idx == 3'd7) state <= S_STOP;
                        else bit_idx <= bit_idx + 1'b1;
                    end else tick_cnt <= tick_cnt + 1'b1;
                end

                S_STOP: begin
                    if (tick_cnt == BIT_TICKS - 1) begin
                        rx_data   <= shift;
                        rx_valid  <= rx_sync;      // stop bit should be high
                        frame_err <= ~rx_sync;
                        state     <= S_IDLE;
                    end else tick_cnt <= tick_cnt + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

//----------------------------------------------------------------------
// from echo_pulse_ctrl.v
//----------------------------------------------------------------------
//======================================================================
// echo_pulse_ctrl
//
// Every CYCLE_MS milliseconds this module asserts `pulse_out` for one
// clock cycle, then just counts until the next one ("wait delay"). In
// main.v this paces the UV sensor trigger: each pulse kicks off a
// uart_tx send of the UV command byte.
//
// CLK_HZ    : system clock frequency driving this module (Basys3 = 100e6)
// CYCLE_MS  : time from one pulse to the next, in milliseconds
//======================================================================
module echo_pulse_ctrl #(
    parameter integer CLK_HZ   = 100_000_000,
    parameter integer CYCLE_MS = 250
) (
    input  wire clk,
    input  wire rst,        // synchronous, active-high
    output reg  pulse_out    // 1 clock cycle wide "send echo pulse" strobe
);

    localparam integer CYCLE_TICKS = (CLK_HZ / 1000) * CYCLE_MS;
    localparam integer CNT_WIDTH   = $clog2(CYCLE_TICKS);

    reg [CNT_WIDTH-1:0] cnt;   // "Wait Delay" counter

    always @(posedge clk) begin
        if (rst) begin
            cnt       <= 0;
            pulse_out <= 1'b0;
        end else if (cnt == CYCLE_TICKS - 1) begin
            cnt       <= 0;
            pulse_out <= 1'b1;   // "Send Echo Pulse", loops back per the diagram
        end else begin
            cnt       <= cnt + 1'b1;
            pulse_out <= 1'b0;
        end
    end

endmodule
