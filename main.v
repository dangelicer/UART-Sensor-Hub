`timescale 1ns / 1ps
//======================================================================
// main.v  --  Basys3 UART Sensor Hub (UV channel), top-level integration
//
// Data path:
//
//   UV sensor (JA3 D-T) --> 2FF sync --> rx (id 0) --> sync_fifo --> BT
//                                                                 streamer
//   baud_tick_gen -> 16x baud tick -> rx                             |
//                                                          uart_tx -> HM-10 (JC4)
//
// Sensor trigger (uart_tx + echo_pulse_ctrl from basys3_sensors.v):
//   UV is triggered: echo_pulse_ctrl polls every UV_POLL_MS, uart_tx
//   sends UV_TRIGGER_CMD out on JA4 (C-R); the reply comes back on
//   JA3 (D-T) through the rx pipeline above. basys3_sensors.v's own
//   uart_rx / uart_sensor_ctrl receive path is NOT used -- rx.v is the
//   receiver.
//
// NOTE: the IR channel was removed -- that sensor's output is not UART,
// so it cannot be received by rx.v.
//
// Switch (up/high = sensor OFF):
//   sw_uv_off (SW15, pin R2)  -> disable UV channel = gate the FIFO
//   writes and stop sending trigger pulses. It does NOT cut power: on a
//   standard Basys3 Pmod, pins 5/11 are GND and 6/12 are VCC (fixed
//   rails), so JA5/JA6 (sensor GND/VCC) come straight from the board.
//
// Pin/power notes: only JA3, JA4, JC3, JC4 are real FPGA I/O. The
// power/ground pins in the wiring plan (JA5/6, JC5/6) are the Pmod
// connector's own VCC/GND rails -- no ports for them.
// See main.xdc for the pin constraints.
//======================================================================
module main #(
    parameter integer CLK_HZ         = 100_000_000,
    parameter integer BAUD           = 9600,
    parameter integer UV_POLL_MS     = 250,        // UV trigger cadence
    parameter [7:0]   UV_TRIGGER_CMD = 8'h09,      // UV sensor command byte (per datasheet)
    parameter [1:0]   UV_ID          = 2'b00,
    parameter integer FIFO_DEPTH     = 8,
    parameter integer FIFO_WIDTH     = 13
) (
    input  wire clk,          // W5  , 100 MHz onboard oscillator
    input  wire rst,          // U18 , btnC, active-high reset

    // Channel-disable switch (high/up = OFF)
    input  wire sw_uv_off,    // R2  , SW15

    // UV sensor (Pmod JA) -- two-wire UART, triggered
    input  wire uv_dt,        // JA3 (J2), UV -> FPGA data   (D-T)
    output wire uv_cr,        // JA4 (G2), FPGA -> UV trigger (C-R)

    // HM-10 Bluetooth (Pmod JC)
    output wire bt_tx,        // JC4 (P18), FPGA -> HM-10 RXD
    input  wire bt_rx,        // JC3 (N17), HM-10 TXD -> FPGA (unused for now)

    // Debug status LEDs (LD0-LD15) -- pipeline visibility map, see below
    output wire [15:0] led
);

    // Enable (switch high = OFF -> enable is the inverse)
    wire uv_enable = ~sw_uv_off;

    //------------------------------------------------------------------
    // 16x baud tick for the rx
    //------------------------------------------------------------------
    wire tick;
    baud_tick_gen u_baud (
        .clk (clk),
        .rst (rst),
        .tick(tick)
    );

    //==================================================================
    // UV channel : sync -> rx -> fifo
    //==================================================================
    wire uv_rx_sync;
    sync_2ff u_uv_sync (
        .clk      (clk),
        .rst      (rst),
        .async_in (uv_dt),
        .sync_out (uv_rx_sync)
    );

    wire [FIFO_WIDTH-1:0] uv_rx_data;
    wire                  uv_rx_wr_en;
    wire                  uv_full;

    rx u_uv_rx (
        .clk        (clk),
        .rst        (rst),
        .RX         (uv_rx_sync),
        .queue_full (uv_full),
        .id         (UV_ID),
        .tick       (tick),
        .data_out   (uv_rx_data),
        .wr_en      (uv_rx_wr_en)
    );

    // rx.wr_en stays high for a full baud-tick period; turn it into a
    // single-clock write strobe so the FIFO writes each word exactly once.
    reg  uv_wr_en_d;
    always @(posedge clk) begin
        if (rst) uv_wr_en_d <= 1'b0;
        else     uv_wr_en_d <= uv_rx_wr_en;
    end
    wire uv_wr_pulse = uv_rx_wr_en & ~uv_wr_en_d & uv_enable;

    wire [FIFO_WIDTH-1:0] uv_fifo_dout;
    wire                  uv_empty;
    wire                  uv_rd_en;

    sync_fifo #(.DEPTH(FIFO_DEPTH), .DWIDTH(FIFO_WIDTH)) u_uv_fifo (
        .rstn  (~rst),
        .clk   (clk),
        .wr_en (uv_wr_pulse),
        .rd_en (uv_rd_en),
        .din   (uv_rx_data),
        .dout  (uv_fifo_dout),
        .empty (uv_empty),
        .full  (uv_full)
    );

    //==================================================================
    // UV trigger : echo_pulse_ctrl -> uart_tx (out on JA4 / C-R)
    //==================================================================
    wire uv_poll_pulse;
    echo_pulse_ctrl #(
        .CLK_HZ  (CLK_HZ),
        .CYCLE_MS(UV_POLL_MS)
    ) u_uv_echo (
        .clk      (clk),
        .rst      (rst),
        .pulse_out(uv_poll_pulse)
    );

    wire uv_tx_busy;
    reg  uv_tx_start;
    always @(posedge clk) begin
        if (rst) uv_tx_start <= 1'b0;
        else     uv_tx_start <= uv_poll_pulse & uv_enable & ~uv_tx_busy;
    end

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uv_tx (
        .clk     (clk),
        .rst     (rst),
        .tx_start(uv_tx_start),
        .tx_data (UV_TRIGGER_CMD),
        .tx      (uv_cr),
        .busy    (uv_tx_busy)
    );

    //==================================================================
    // Bluetooth output : drain the UV FIFO -> uart_tx
    //==================================================================
    wire       bt_tx_busy;
    wire       bt_tx_start;
    wire [7:0] bt_tx_data;

    fifo_bt_streamer #(.DWIDTH(FIFO_WIDTH)) u_streamer (
        .clk      (clk),
        .rst      (rst),
        .f_empty  (uv_empty),
        .f_dout   (uv_fifo_dout),
        .f_rd_en  (uv_rd_en),
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

    // bt_rx is wired to a pin for future host->hub commands; unused today.

    //==================================================================
    // Debug status LEDs -- live map of the pipeline. Single-cycle events
    // are stretched to ~0.1 s so they are visible.
    //
    //   LD0  heartbeat (~1.5 Hz)   design alive & clocked
    //   LD1  UV input activity     UV sensor is sending serial on JA3
    //   LD3  UV word captured      rx decoded a UV frame into the FIFO
    //   LD5  UV FIFO non-empty     a UV word is queued
    //   LD7  BT TX activity        bytes leaving toward the HM-10 on JC4
    //   LD8  UV trigger firing     UV poll is transmitting on JA4
    //   LD15 reset                 lit while held in reset
    //   (LD2, LD4, LD6, LD9-LD14 unused -> 0)
    //==================================================================
    localparam integer BLINK_CYCLES = 10_000_000; // ~0.1 s @ 100 MHz

    // Heartbeat: free-running counter, MSB blinks ~1.5 Hz.
    reg [25:0] heartbeat;
    always @(posedge clk) begin
        if (rst) heartbeat <= 26'd0;
        else     heartbeat <= heartbeat + 1'b1;
    end

    // Falling-edge detectors (UART start bits on the synced line / bt_tx).
    reg uv_line_d, bt_tx_d;
    always @(posedge clk) begin
        uv_line_d <= uv_rx_sync;
        bt_tx_d   <= bt_tx;
    end
    wire uv_line_fell = uv_line_d & ~uv_rx_sync;
    wire bt_tx_fell   = bt_tx_d   & ~bt_tx;

    wire led_uv_line, led_uv_word, led_uv_fifo, led_bt_tx, led_uv_trig;

    pulse_stretch #(.CYCLES(BLINK_CYCLES)) s_uv_line (.clk(clk), .rst(rst), .trig(uv_line_fell), .led(led_uv_line));
    pulse_stretch #(.CYCLES(BLINK_CYCLES)) s_uv_word (.clk(clk), .rst(rst), .trig(uv_wr_pulse),  .led(led_uv_word));
    pulse_stretch #(.CYCLES(BLINK_CYCLES)) s_uv_fifo (.clk(clk), .rst(rst), .trig(~uv_empty),    .led(led_uv_fifo));
    pulse_stretch #(.CYCLES(BLINK_CYCLES)) s_bt_tx   (.clk(clk), .rst(rst), .trig(bt_tx_fell),   .led(led_bt_tx));
    pulse_stretch #(.CYCLES(BLINK_CYCLES)) s_uv_trig (.clk(clk), .rst(rst), .trig(uv_tx_start),  .led(led_uv_trig));

    assign led[0]    = heartbeat[25];
    assign led[1]    = led_uv_line;
    assign led[2]    = 1'b0;
    assign led[3]    = led_uv_word;
    assign led[4]    = 1'b0;
    assign led[5]    = led_uv_fifo;
    assign led[6]    = 1'b0;
    assign led[7]    = led_bt_tx;
    assign led[8]    = led_uv_trig;
    assign led[14:9] = 6'b0;
    assign led[15]   = rst;

endmodule


//======================================================================
// pulse_stretch -- holds `led` high for CYCLES clocks after each `trig`.
// Retriggerable: a level input keeps it lit while active plus the tail.
// Used only for making brief debug events visible on the LEDs.
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
        else if (trig)     cnt <= CYCLES;        // (re)load on event
        else if (cnt != 0) cnt <= cnt - 32'd1;   // count down the tail
    end
    assign led = (cnt != 0);
endmodule


//======================================================================
// sync_2ff -- 2-flop synchronizer for an async UART input line.
// Resets to 1 (UART idle level) so no false start bit is seen at reset.
//======================================================================
module sync_2ff (
    input  wire clk,
    input  wire rst,
    input  wire async_in,
    output wire sync_out
);
    reg meta, sync;
    always @(posedge clk) begin
        if (rst) begin
            meta <= 1'b1;
            sync <= 1'b1;
        end else begin
            meta <= async_in;
            sync <= meta;
        end
    end
    assign sync_out = sync;
endmodule


//======================================================================
// fifo_bt_streamer
//
// Drains one FIFO of DWIDTH-bit words and pushes each word out as two
// UART bytes via uart_tx:
//
//   high byte = {3'b000, id[1:0], parity_err, framing_err, overrun_err}
//   low  byte = data_byte[7:0]
//
// matching the rx.v word layout {id, data_byte, parity, framing, overrun}.
//
// FIFO read timing (sync_fifo): rd_en pulses one cycle; dout is valid the
// cycle after rd_en deasserts -- handled by the READ/CAP/LATCH steps.
//======================================================================
module fifo_bt_streamer #(
    parameter integer DWIDTH = 13
) (
    input  wire              clk,
    input  wire              rst,

    input  wire              f_empty,
    input  wire [DWIDTH-1:0] f_dout,
    output reg               f_rd_en,

    input  wire              tx_busy,
    output reg               tx_start,
    output reg  [7:0]        tx_data
);

    localparam [3:0] S_IDLE    = 4'd0,
                     S_READ    = 4'd1,
                     S_CAP     = 4'd2,
                     S_LATCH   = 4'd3,
                     S_HI_REQ  = 4'd4,
                     S_HI_WAIT = 4'd5,
                     S_HI_FIN  = 4'd6,
                     S_LO_REQ  = 4'd7,
                     S_LO_WAIT = 4'd8,
                     S_LO_FIN  = 4'd9;

    reg [3:0]        state;
    reg [DWIDTH-1:0] word;

    // Byte framing of the captured word.
    wire [7:0] hi_byte = {3'b000, word[12:11], word[2], word[1], word[0]};
    wire [7:0] lo_byte = word[10:3];

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            f_rd_en  <= 1'b0;
            tx_start <= 1'b0;
            tx_data  <= 8'h00;
            word     <= {DWIDTH{1'b0}};
        end else begin
            // one-cycle strobes default low
            f_rd_en  <= 1'b0;
            tx_start <= 1'b0;

            case (state)
                S_IDLE: if (!f_empty) state <= S_READ;

                S_READ: begin
                    f_rd_en <= 1'b1;
                    state   <= S_CAP;
                end

                S_CAP: state <= S_LATCH;   // rd_en high this cycle; dout latched at cycle end

                S_LATCH: begin
                    word  <= f_dout;
                    state <= S_HI_REQ;
                end

                S_HI_REQ: if (!tx_busy) begin
                    tx_data  <= hi_byte;
                    tx_start <= 1'b1;
                    state    <= S_HI_WAIT;
                end
                S_HI_WAIT: if (tx_busy) state <= S_HI_FIN;  // byte accepted
                S_HI_FIN:  if (!tx_busy) state <= S_LO_REQ; // byte finished

                S_LO_REQ: if (!tx_busy) begin
                    tx_data  <= lo_byte;
                    tx_start <= 1'b1;
                    state    <= S_LO_WAIT;
                end
                S_LO_WAIT: if (tx_busy) state <= S_LO_FIN;
                S_LO_FIN:  if (!tx_busy) state <= S_IDLE;

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
