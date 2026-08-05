`timescale 1ns / 1ps
//======================================================================
// main.v  --  Basys3 UART Sensor Hub (UV ambient light), top-level
//
// The UV sensor is a DFRobot SEN0540 (LTR390-UV) in UART/Modbus-RTU mode
// at slave address 0x1C, 9600 8N1. It is NOT a raw-UART device, so the
// data path is a Modbus RTU master, not rx.v:
//
//   ltr390_seq  --drives-->  modbus_rtu_master  <--UART-->  UV sensor
//        |                         (JA4 tx, JA3 rx)
//        | als, als_valid
//        v
//   bt_als_framer --> uart_tx --> HM-10 Bluetooth (JC4)
//
// Boot sequence (Modbus Write Single Register, func 0x06):
//   1. reg 0x0D (MEAS_RATE) = 0x0022   (18-bit, 100 ms)
//   2. reg 0x06 (GAIN)      = 0x0001   (gain 3)
//   3. reg 0x0E (MAIN_CTRL) = 0x0002   (ALS / ambient-light mode)
// Then poll (Read Input Registers, func 0x04): regs 0x07..0x08, and
//   ALS raw = reg[0x07] | (reg[0x08] << 16).
//
// Bluetooth output frame (per reading): 0xAA, als[23:16], als[15:8],
//   als[7:0]  -- the 0xAA sync byte lets the host re-align.
//
// Switch: sw_uv_off (SW15, R2) high = disable the UV master (idle).
//
// Pins: only JA3 (uv_dt), JA4 (uv_cr), JC3 (bt_rx), JC4 (bt_tx) are real
// FPGA I/O; sensor/HM-10 power+ground are the Pmod VCC/GND rails.
// See main.xdc.
//
// NOTE: the raw-UART path (baud_tick_gen, rx.v, fifo.v, fifo_bt_streamer)
// and the IR channel are no longer instantiated -- kept in the repo for
// reference/reuse only.
//======================================================================
module main #(
    parameter integer CLK_HZ         = 100_000_000,
    parameter integer BAUD           = 9600,
    parameter integer UV_TIMEOUT_MS  = 50,          // Modbus response timeout
    parameter integer UV_BOOT_CYCLES = 20_000_000,  // ~200 ms power-up settle
    parameter integer UV_POLL_CYCLES = 20_000_000   // ~200 ms between reads
) (
    input  wire clk,          // W5  , 100 MHz onboard oscillator
    input  wire rst,          // U18 , btnC, active-high reset

    input  wire sw_uv_off,    // R2  , SW15  (high = UV OFF)

    // UV sensor (Pmod JA) -- Modbus RTU over UART
    input  wire uv_dt,        // JA3 (J2), UV -> FPGA  (sensor TXD)
    output wire uv_cr,        // JA4 (G2), FPGA -> UV  (sensor RXD)

    // HM-10 Bluetooth (Pmod JC)
    output wire bt_tx,        // JC4 (P18), FPGA -> HM-10 RXD
    input  wire bt_rx,        // JC3 (N17), HM-10 TXD -> FPGA (unused)

    // Debug status LEDs (LD0-LD15) -- pipeline map, see below
    output wire [15:0] led
);

    wire uv_enable = ~sw_uv_off;

    //==================================================================
    // Modbus RTU master (drives JA4/tx, listens on JA3/rx)
    //==================================================================
    wire        mb_start;
    wire [7:0]  mb_addr;
    wire [7:0]  mb_func;
    wire [15:0] mb_reg;
    wire [15:0] mb_data;
    wire        mb_busy;
    wire        mb_done;
    wire        mb_error;
    wire [8*9-1:0] mb_resp;   // RESP_LEN = 9 (read response length)

    modbus_rtu_master #(
        .CLK_HZ     (CLK_HZ),
        .BAUD       (BAUD),
        .RESP_LEN   (9),
        .TIMEOUT_MS (UV_TIMEOUT_MS)
    ) u_mb (
        .clk           (clk),
        .rst           (rst),
        .start         (mb_start),
        .slave_addr    (mb_addr),
        .func_code     (mb_func),
        .reg_addr      (mb_reg),
        .data_or_count (mb_data),
        .busy          (mb_busy),
        .done          (mb_done),
        .error         (mb_error),
        .resp_frame    (mb_resp),
        .uart_tx_pin   (uv_cr),
        .uart_rx_pin   (uv_dt)
    );

    //==================================================================
    // Sequencer: boot config writes, then poll ALS
    //==================================================================
    wire [23:0] als;
    wire        als_valid;

    ltr390_seq #(
        .BOOT_CYCLES(UV_BOOT_CYCLES),
        .POLL_CYCLES(UV_POLL_CYCLES)
    ) u_seq (
        .clk           (clk),
        .rst           (rst),
        .enable        (uv_enable),
        .start         (mb_start),
        .slave_addr    (mb_addr),
        .func_code     (mb_func),
        .reg_addr      (mb_reg),
        .data_or_count (mb_data),
        .busy          (mb_busy),
        .done          (mb_done),
        .error         (mb_error),
        .resp_frame    (mb_resp),
        .als           (als),
        .als_valid     (als_valid),
        .booting       ()            // status output unused (LEDs removed)
    );

    //==================================================================
    // Bluetooth output: frame each reading as 0xAA + 3 ALS bytes
    //==================================================================
    wire       bt_tx_busy;
    wire       bt_tx_start;
    wire [7:0] bt_tx_data;

    bt_als_framer u_framer (
        .clk      (clk),
        .rst      (rst),
        .als_valid(als_valid),
        .als      (als),
        .tx_busy  (bt_tx_busy),
        .tx_start (bt_tx_start),
        .tx_data  (bt_tx_data)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_bt_tx (
        .clk     (clk),
        .rst     (rst),
        .tx_start(bt_tx_start),
        .tx_data (bt_tx_data),
        .tx      (bt_tx),
        .busy    (bt_tx_busy)
    );

    //==================================================================
    // LD0 blinks (~0.1 s) each time a byte is transmitted over the
    // Bluetooth link. All other LEDs are unused.
    //==================================================================
    wire led_tx;
    pulse_stretch #(.CYCLES(10_000_000)) s_tx (   // ~0.1 s visible blink @ 100 MHz
        .clk (clk),
        .rst (rst),
        .trig(bt_tx_start),                       // one pulse per BT byte sent
        .led (led_tx)
    );

    assign led[0]    = led_tx;
    assign led[15:1] = 15'b0;

endmodule


//======================================================================
// ltr390_seq -- command sequencer for the LTR390UV over modbus_rtu_master.
//
// On enable: wait BOOT_CYCLES, then issue the three ambient-light config
// writes (func 0x06), then repeatedly issue the ALS read (func 0x04,
// regs 0x07..0x08). Each read `done` extracts the 20-bit ALS value and
// pulses als_valid.
//
// The master is instantiated with RESP_LEN=9 (the read-response length).
// A write echo is only 8 bytes, so config writes end in the master's
// timeout `error` rather than `done` -- that's expected and harmless
// (the write still reached the sensor); the sequencer advances on either.
//======================================================================
module ltr390_seq #(
    parameter integer BOOT_CYCLES = 20_000_000,
    parameter integer POLL_CYCLES = 20_000_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,

    // to modbus_rtu_master
    output reg         start,
    output reg  [7:0]  slave_addr,
    output reg  [7:0]  func_code,
    output reg  [15:0] reg_addr,
    output reg  [15:0] data_or_count,
    input  wire        busy,
    input  wire        done,
    input  wire        error,
    input  wire [8*9-1:0] resp_frame,

    // reading out
    output reg  [23:0] als,
    output reg         als_valid,
    output wire        booting
);

    localparam [7:0] SLAVE = 8'h1C;
    localparam [7:0] FUNC_WRITE = 8'h06;
    localparam [7:0] FUNC_READ  = 8'h04;

    // step: 0,1,2 = config writes ; 3 = ALS read (poll)
    localparam [2:0] S_DISABLED = 3'd0,
                     S_BOOT     = 3'd1,
                     S_ISSUE    = 3'd2,
                     S_WAIT     = 3'd3,
                     S_POLL     = 3'd4;

    reg [2:0]  state;
    reg [1:0]  step;
    reg [31:0] delay;

    assign booting = (state != S_DISABLED) && (step != 2'd3);

    always @(posedge clk) begin
        if (rst) begin
            state <= S_DISABLED; step <= 2'd0; delay <= 0;
            start <= 1'b0; slave_addr <= SLAVE; func_code <= FUNC_WRITE;
            reg_addr <= 16'd0; data_or_count <= 16'd0;
            als <= 24'd0; als_valid <= 1'b0;
        end else if (!enable) begin
            state <= S_DISABLED; start <= 1'b0; als_valid <= 1'b0;
        end else begin
            start     <= 1'b0;
            als_valid <= 1'b0;

            case (state)
                S_DISABLED: begin
                    step  <= 2'd0;
                    delay <= BOOT_CYCLES;
                    state <= S_BOOT;
                end

                S_BOOT: if (delay == 0) state <= S_ISSUE;
                        else delay <= delay - 1'b1;

                S_ISSUE: begin
                    slave_addr <= SLAVE;
                    case (step)
                        2'd0: begin func_code <= FUNC_WRITE; reg_addr <= 16'h000D; data_or_count <= 16'h0022; end
                        2'd1: begin func_code <= FUNC_WRITE; reg_addr <= 16'h0006; data_or_count <= 16'h0001; end
                        2'd2: begin func_code <= FUNC_WRITE; reg_addr <= 16'h000E; data_or_count <= 16'h0002; end
                        default: begin func_code <= FUNC_READ; reg_addr <= 16'h0007; data_or_count <= 16'h0002; end
                    endcase
                    start <= 1'b1;
                    state <= S_WAIT;
                end

                S_WAIT: begin
                    // transaction finishes with a done (success) or error
                    // (CRC/echo/timeout) pulse. For config writes we accept
                    // either; for the read, `done` carries valid data.
                    if (done && step == 2'd3) begin
                        // ALS raw = reg0 | (reg1<<16); read response layout:
                        // [0]1C [1]04 [2]04 [3]r0hi [4]r0lo [5]r1hi [6]r1lo [7]crc [8]crc
                        als <= {resp_frame[55:48],   // reg1 low  -> als[23:16]
                                resp_frame[31:24],   // reg0 high -> als[15:8]
                                resp_frame[39:32]};  // reg0 low  -> als[7:0]
                        als_valid <= 1'b1;
                    end
                    if (done || error) begin
                        if (step == 2'd3) begin
                            delay <= POLL_CYCLES;
                            state <= S_POLL;
                        end else begin
                            step  <= step + 1'b1;
                            state <= S_ISSUE;
                        end
                    end
                end

                S_POLL: if (delay == 0) state <= S_ISSUE;  // re-issue read (step stays 3)
                        else delay <= delay - 1'b1;

                default: state <= S_DISABLED;
            endcase
        end
    end

endmodule


//======================================================================
// bt_als_framer -- send one reading as: 0xAA sync, als[23:16],
// als[15:8], als[7:0], MSB first, via a shared uart_tx.
//======================================================================
module bt_als_framer (
    input  wire        clk,
    input  wire        rst,
    input  wire        als_valid,
    input  wire [23:0] als,
    input  wire        tx_busy,
    output reg         tx_start,
    output reg  [7:0]  tx_data
);
    localparam [7:0] SYNC = 8'hAA;
    localparam [1:0] S_IDLE = 2'd0, S_REQ = 2'd1, S_WAIT = 2'd2, S_FIN = 2'd3;

    reg [1:0]  state;
    reg [1:0]  bidx;
    reg [23:0] latched;

    reg [7:0] cur;
    always @(*) begin
        case (bidx)
            2'd0:    cur = SYNC;
            2'd1:    cur = latched[23:16];
            2'd2:    cur = latched[15:8];
            default: cur = latched[7:0];
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; tx_start <= 1'b0; bidx <= 2'd0; tx_data <= 8'h00;
        end else begin
            tx_start <= 1'b0;
            case (state)
                S_IDLE: if (als_valid) begin latched <= als; bidx <= 2'd0; state <= S_REQ; end
                S_REQ:  if (!tx_busy) begin tx_data <= cur; tx_start <= 1'b1; state <= S_WAIT; end
                S_WAIT: if (tx_busy) state <= S_FIN;
                S_FIN:  if (!tx_busy) begin
                            if (bidx == 2'd3) state <= S_IDLE;
                            else begin bidx <= bidx + 1'b1; state <= S_REQ; end
                        end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule


//======================================================================
// pulse_stretch -- holds `led` high for CYCLES clocks after each `trig`.
//======================================================================
module pulse_stretch #(parameter integer CYCLES = 10_000_000) (
    input  wire clk,
    input  wire rst,
    input  wire trig,
    output wire led
);
    reg [31:0] cnt;
    always @(posedge clk) begin
        if (rst)           cnt <= 32'd0;
        else if (trig)     cnt <= CYCLES;
        else if (cnt != 0) cnt <= cnt - 32'd1;
    end
    assign led = (cnt != 0);
endmodule
