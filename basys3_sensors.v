`timescale 1ns / 1ps
//======================================================================
// basys3_uv_ir.v -- combined single-file RTL for the Basys3 UV/IR
// sensor front end (all-UART design).
//
// Module order (bottom-up): uart_tx, uart_rx, echo_pulse_ctrl,
// uart_sensor_ctrl, basys3_uv_ir_top (the one to set as your Vivado
// top-level module). Pin constraints are in the companion
// basys3_uv_ir_top.xdc file -- Verilog sources and XDC constraints
// cannot be combined into one file in Vivado, so that one stays
// separate.
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
// `rx_data` is ready.
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
// Implements the bottom loop of the block diagram:
//
//      +----------------------------+
//      |                            |
//      v                            |
//   Send Echo Pulse ----------> Wait Delay
//
// Every CYCLE_MS milliseconds this module asserts `pulse_out` for one
// clock cycle -- that's the "Send Echo Pulse" event. In between it
// just counts, which is the "Wait Delay" box. `pulse_out` fans out to
// the UV (I2C) and IR (UART) sensor controllers so both start a new
// reading on the same cadence.
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

//----------------------------------------------------------------------
// from uart_sensor_ctrl.v
//----------------------------------------------------------------------
//======================================================================
// uart_sensor_ctrl
//
// Drives ANY UART sensor in "triggered" mode: on each pulse of
// `start_read` (wire this to echo_pulse_ctrl's pulse_out -- the "Send
// Echo Pulse" event from the diagram), it sends a one-byte query
// command, then collects RESP_LEN response bytes back before
// returning to idle and waiting for the next trigger. Used for both
// the UV sensor and the IR sensor -- each gets its own instance, its
// own UART pins, and its own TRIGGER_CMD/RESP_LEN.
//
// *** EDIT TRIGGER_CMD / RESP_LEN / BAUD FOR YOUR EXACT SENSORS ***
// Every UART sensor module has its own command and frame format.
// These are placeholders until you have the datasheets in hand:
//   TRIGGER_CMD - the exact byte your sensor's datasheet calls for
//   RESP_LEN    - how many bytes its reply frame contains
//   BAUD        - your sensor's UART baud rate
// resp_data packs bytes LSB-first: byte0 in resp_data[7:0], byte1 in
// resp_data[15:8], etc. (the order they arrive in).
//======================================================================
module uart_sensor_ctrl #(
    parameter integer CLK_HZ     = 100_000_000,
    parameter integer BAUD       = 9600,
    parameter [7:0]   TRIGGER_CMD = 8'h01,   // <-- placeholder, set per datasheet
    parameter integer RESP_LEN   = 2,        // <-- placeholder, set per datasheet
    parameter integer TIMEOUT_MS = 50
) (
    input  wire        clk,
    input  wire         rst,

    input  wire         start_read,          // pulse from echo_pulse_ctrl

    output reg  [8*RESP_LEN-1:0] resp_data,
    output reg           resp_valid,          // 1-cycle pulse when resp_data updates
    output reg           resp_timeout,        // 1-cycle pulse if the sensor didn't answer

    output wire          uart_tx_pin,
    input  wire          uart_rx_pin
);

    // ---- UART instances ----
    reg        tx_start;
    wire       tx_busy;
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst(rst),
        .tx_start(tx_start), .tx_data(TRIGGER_CMD),
        .tx(uart_tx_pin), .busy(tx_busy)
    );

    wire [7:0] rx_data;
    wire       rx_valid;
    wire       rx_frame_err;
    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst(rst),
        .rx(uart_rx_pin),
        .rx_data(rx_data), .rx_valid(rx_valid), .frame_err(rx_frame_err)
    );

    // ---- Sequencer FSM ----
    localparam integer TIMEOUT_TICKS = (CLK_HZ / 1000) * TIMEOUT_MS;
    localparam integer TCNT_W        = $clog2(TIMEOUT_TICKS + 1);

    localparam S_IDLE     = 2'd0,
               S_SEND     = 2'd1,
               S_WAIT_RSP = 2'd2;

    reg [1:0]                     state;
    reg [$clog2(RESP_LEN+1)-1:0]  byte_cnt;
    reg [TCNT_W-1:0]              to_cnt;
    wire                          byte_in = (state == S_WAIT_RSP) && rx_valid && !rx_frame_err;

    // Each received byte lands in its own fixed byte lane (byte0 in
    // resp_data[7:0], byte1 in resp_data[15:8], ...). A generate loop
    // keeps every part-select in range no matter what RESP_LEN is --
    // a plain shift register breaks for RESP_LEN==1.
    genvar gi;
    generate
        for (gi = 0; gi < RESP_LEN; gi = gi + 1) begin : gen_byte_capture
            always @(posedge clk) begin
                if (rst) begin
                    resp_data[(gi+1)*8-1 -: 8] <= 8'h00;
                end else if (byte_in && byte_cnt == gi) begin
                    resp_data[(gi+1)*8-1 -: 8] <= rx_data;
                end
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            tx_start     <= 1'b0;
            byte_cnt     <= 0;
            to_cnt       <= 0;
            resp_valid   <= 1'b0;
            resp_timeout <= 1'b0;
        end else begin
            tx_start     <= 1'b0;
            resp_valid   <= 1'b0;
            resp_timeout <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start_read && !tx_busy) begin
                        tx_start <= 1'b1;
                        state    <= S_SEND;
                    end
                end

                S_SEND: begin
                    if (!tx_busy) begin // TX module deasserts busy once the byte is sent
                        byte_cnt <= 0;
                        to_cnt   <= 0;
                        state    <= S_WAIT_RSP;
                    end
                end

                S_WAIT_RSP: begin
                    if (byte_in) begin
                        if (byte_cnt + 1 == RESP_LEN) begin
                            resp_valid <= 1'b1;
                            state      <= S_IDLE;
                        end else begin
                            byte_cnt <= byte_cnt + 1'b1;
                            to_cnt   <= 0;
                        end
                    end else if (to_cnt == TIMEOUT_TICKS - 1) begin
                        resp_timeout <= 1'b1;
                        state        <= S_IDLE;
                    end else begin
                        to_cnt <= to_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

//----------------------------------------------------------------------
// from basys3_uv_ir_top.v
//----------------------------------------------------------------------
//======================================================================
// basys3_uv_ir_top
//
// Top-level glue for the Basys3, all-UART:
//   - echo_pulse_ctrl generates the periodic "Send Echo Pulse /
//     Wait Delay" cadence from the bottom of the diagram.
//   - uart_sensor_ctrl (instance u_uv) triggers + reads the UV sensor
//     over its own UART line on that cadence.
//   - uart_sensor_ctrl (instance u_ir) triggers + reads the IR sensor
//     over its own, separate UART line on the same cadence.
//
// uv_data16_o/uv_data_valid_o and ir_data16_o/ir_data_valid_o are
// exposed at the top so you can feed them straight into your existing
// sensor_formatter / round-robin arbiter / queue pipeline (the
// "Format Sensor Data", "Input data into Queue" blocks) alongside the
// SCD41 and pressure readings -- that's what carries both readings
// out over the HM-10 Bluetooth link to the terminal. LEDs give a
// quick sanity check without needing the rest of that pipeline hooked
// up yet.
//
// *** EDIT PER YOUR SENSORS *** -- TRIGGER_CMD/RESP_LEN/BAUD for each
// uart_sensor_ctrl instance below are placeholders until you have the
// exact datasheets for your UART UV and IR modules. See the comment
// block at the top of uart_sensor_ctrl.v.
//
// Pin assignments: see basys3_uv_ir_top.xdc.
//======================================================================
module basys3_uv_ir_top (
    input  wire        clk100mhz,      // W5 on Basys3
    input  wire        btn_rst,        // btnC, active-high

    // UV sensor -- its own UART line
    output wire         uv_uart_tx,     // FPGA -> UV sensor
    input  wire          uv_uart_rx,     // UV sensor -> FPGA

    // IR sensor -- its own, separate UART line
    output wire         ir_uart_tx,     // FPGA -> IR sensor
    input  wire          ir_uart_rx,     // IR sensor -> FPGA

    // Debug LEDs
    output wire [15:0]   led,

    // Data outputs to feed your existing sensor_formatter / arbiter
    output wire [15:0]   uv_data16_o,
    output wire           uv_data_valid_o,
    output wire [15:0]   ir_data16_o,
    output wire           ir_data_valid_o
);

    localparam integer CLK_HZ   = 100_000_000;
    localparam integer CYCLE_MS = 250;   // how often to poll both sensors

    wire rst = btn_rst;

    // ---- Echo pulse / wait delay loop (bottom of the diagram) ----
    wire pulse_out;
    echo_pulse_ctrl #(
        .CLK_HZ  (CLK_HZ),
        .CYCLE_MS(CYCLE_MS)
    ) u_echo (
        .clk      (clk100mhz),
        .rst      (rst),
        .pulse_out(pulse_out)
    );

    // ---- UV sensor over its own UART line ----
    wire [15:0] uv_data16;      // RESP_LEN=2 -> naturally 16 bits, no truncation needed
    wire        uv_data_valid;
    wire        uv_timeout;

    uart_sensor_ctrl #(
        .CLK_HZ     (CLK_HZ),
        .BAUD       (9600),      // <-- set to match your UV sensor
        .TRIGGER_CMD(8'h01),     // <-- set per your UV sensor's datasheet
        .RESP_LEN   (2),         // <-- set per your UV sensor's datasheet
        .TIMEOUT_MS (50)
    ) u_uv (
        .clk         (clk100mhz),
        .rst         (rst),
        .start_read  (pulse_out),
        .resp_data   (uv_data16),
        .resp_valid  (uv_data_valid),
        .resp_timeout(uv_timeout),
        .uart_tx_pin (uv_uart_tx),
        .uart_rx_pin (uv_uart_rx)
    );

    // ---- IR sensor over its own, separate UART line ----
    wire [7:0] ir_data_raw;     // RESP_LEN=1 -> a single byte (e.g. 0x00/0x01)
    wire       ir_data_valid;
    wire       ir_timeout;

    uart_sensor_ctrl #(
        .CLK_HZ     (CLK_HZ),
        .BAUD       (9600),      // <-- set to match your IR sensor
        .TRIGGER_CMD(8'h02),     // <-- set per your IR sensor's datasheet
        .RESP_LEN   (1),         // <-- set per your IR sensor's datasheet
        .TIMEOUT_MS (50)
    ) u_ir (
        .clk         (clk100mhz),
        .rst         (rst),
        .start_read  (pulse_out),
        .resp_data   (ir_data_raw),
        .resp_valid  (ir_data_valid),
        .resp_timeout(ir_timeout),
        .uart_tx_pin (ir_uart_tx),
        .uart_rx_pin (ir_uart_rx)
    );

    wire [15:0] ir_data16 = {8'd0, ir_data_raw}; // zero-extend to match the 16-bit formatter width

    // ---- Outputs to the rest of your pipeline ----
    assign uv_data16_o     = uv_data16;
    assign uv_data_valid_o = uv_data_valid;
    assign ir_data16_o     = ir_data16;
    assign ir_data_valid_o = ir_data_valid;

    // ---- Debug LEDs ----
    reg uv_led_latch, ir_led_latch;
    always @(posedge clk100mhz) begin
        if (rst) begin
            uv_led_latch <= 1'b0;
            ir_led_latch <= 1'b0;
        end else begin
            if (uv_data_valid) uv_led_latch <= ~uv_led_latch; // toggles each good UV reply
            if (ir_data_valid) ir_led_latch <= ~ir_led_latch; // toggles each good IR reply
        end
    end

    assign led[0]    = pulse_out;      // flickers on the echo-pulse cadence
    assign led[1]    = uv_led_latch;   // toggles each successful UV read
    assign led[2]    = uv_timeout;     // on = UV sensor didn't answer in time
    assign led[3]    = ir_led_latch;   // toggles each successful IR read
    assign led[4]    = ir_timeout;     // on = IR sensor didn't answer in time
    assign led[5]    = ir_data_raw[0]; // live last-read IR bit
    assign led[15:6] = 10'd0;

endmodule
