`timescale 1ns / 1ps
//======================================================================
// modbus_rtu_master
//
// Minimal Modbus RTU master for a single point-to-point UART link (no
// RS-485 multi-drop, so the usual 3.5-character inter-frame silence
// requirement is not enforced here -- each transaction is a clean
// request-then-response with nothing else sharing the wire).
//
// Supports exactly the two function codes the DFRobot SEN0540 (Gravity
// LTR390-UV, UART/Modbus mode) needs:
//   0x04 - Read Input Register(s)          -- data_or_count = register count
//   0x06 - Write (single) Holding Register -- data_or_count = value to write
//
// Request frame (always 8 bytes for these two function codes):
//   [addr][func][reg_hi][reg_lo][data_or_count_hi][data_or_count_lo][crc_lo][crc_hi]
//
// Response frame length depends on what was requested -- set RESP_LEN
// per instantiation:
//   func 0x06 (write)            -> RESP_LEN = 8  (slave echoes the request)
//   func 0x04, N registers read  -> RESP_LEN = 3 + 2*N + 2  (addr+func+bytecount+data+crc)
//
// resp_frame holds the raw captured response bytes, byte0 in
// resp_frame[7:0], byte1 in resp_frame[15:8], etc. (arrival order).
// `error` covers CRC mismatch, a mismatched address/function-code
// echo, a Modbus exception response (func code with bit 7 set), or a
// timeout with no response at all.
//======================================================================
module modbus_rtu_master #(
    parameter integer CLK_HZ     = 100_000_000,
    parameter integer BAUD       = 9600,
    parameter integer RESP_LEN   = 9,
    parameter integer TIMEOUT_MS = 100
) (
    input  wire        clk,
    input  wire         rst,

    input  wire         start,
    input  wire [7:0]   slave_addr,
    input  wire [7:0]   func_code,      // 8'h04 or 8'h06
    input  wire [15:0]  reg_addr,
    input  wire [15:0]  data_or_count,  // write value (0x06) or register count (0x04)

    output reg          busy,
    output reg          done,           // 1-cycle pulse, success
    output reg          error,          // 1-cycle pulse, any failure (see above)

    output reg  [8*RESP_LEN-1:0] resp_frame,

    output wire         uart_tx_pin,
    input  wire          uart_rx_pin
);

    // ---- CRC16 (Modbus: poly 0xA001, init 0xFFFF, LSB-first bit shift) ----
    function [15:0] crc16_step(input [15:0] crc_in, input [7:0] data_byte);
        integer i;
        reg [15:0] crc;
        begin
            crc = crc_in ^ {8'h00, data_byte};
            for (i = 0; i < 8; i = i + 1) begin
                if (crc[0])
                    crc = (crc >> 1) ^ 16'hA001;
                else
                    crc = crc >> 1;
            end
            crc16_step = crc;
        end
    endfunction

    // Combinationally chain the CRC across the 6 header bytes, in
    // transmission order, so it's a plain 1-2-3-4-5-6 read top to
    // bottom instead of one deeply-nested expression.
    wire [15:0] crc_s0 = 16'hFFFF;
    wire [15:0] crc_s1 = crc16_step(crc_s0, slave_addr);
    wire [15:0] crc_s2 = crc16_step(crc_s1, func_code);
    wire [15:0] crc_s3 = crc16_step(crc_s2, reg_addr[15:8]);
    wire [15:0] crc_s4 = crc16_step(crc_s3, reg_addr[7:0]);
    wire [15:0] crc_s5 = crc16_step(crc_s4, data_or_count[15:8]);
    wire [15:0] crc_tx = crc16_step(crc_s5, data_or_count[7:0]);

    // ---- UART instances ----
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst(rst), .tx_start(tx_start), .tx_data(tx_data),
        .tx(uart_tx_pin), .busy(tx_busy)
    );

    wire [7:0] rx_data;
    wire       rx_valid;
    wire       rx_frame_err;
    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst(rst), .rx(uart_rx_pin),
        .rx_data(rx_data), .rx_valid(rx_valid), .frame_err(rx_frame_err)
    );

    // ---- Main FSM ----
    localparam integer TIMEOUT_TICKS = (CLK_HZ / 1000) * TIMEOUT_MS;
    localparam integer TCNT_W        = $clog2(TIMEOUT_TICKS + 1);

    localparam S_IDLE     = 2'd0,
               S_SEND     = 2'd1,
               S_WAIT_RSP = 2'd2,
               S_CHECK    = 2'd3;

    reg [1:0]                    state;
    reg [2:0]                    tx_idx;
    reg [15:0]                   crc_latched; // CRC latched at transaction start
    reg [$clog2(RESP_LEN+1)-1:0] rx_idx;
    reg [TCNT_W-1:0]             to_cnt;

    wire byte_in = (state == S_WAIT_RSP) && rx_valid && !rx_frame_err;

    // Each received byte lands in its own fixed lane -- a generate
    // loop keeps every part-select in range regardless of RESP_LEN
    // (see uart_sensor_ctrl.v for why a plain shift register breaks
    // for small RESP_LEN values).
    genvar gi;
    generate
        for (gi = 0; gi < RESP_LEN; gi = gi + 1) begin : gen_byte_capture
            always @(posedge clk) begin
                if (rst) begin
                    resp_frame[(gi+1)*8-1 -: 8] <= 8'h00;
                end else if (byte_in && rx_idx == gi) begin
                    resp_frame[(gi+1)*8-1 -: 8] <= rx_data;
                end
            end
        end
    endgenerate

    // CRC over the received frame (all bytes except the trailing 2
    // CRC bytes themselves), built the same way as the TX chain above
    // but as a generate-based reduction since RESP_LEN is parameterized.
    wire [15:0] rx_crc_chain [0:RESP_LEN-2];
    assign rx_crc_chain[0] = 16'hFFFF;
    generate
        for (gi = 0; gi < RESP_LEN - 2; gi = gi + 1) begin : gen_crc_chain
            assign rx_crc_chain[gi+1] =
                crc16_step(rx_crc_chain[gi], resp_frame[(gi+1)*8-1 -: 8]);
        end
    endgenerate
    wire [15:0] rx_crc_calc = rx_crc_chain[RESP_LEN-2];
    wire [15:0] rx_crc_recv = {resp_frame[8*RESP_LEN-1 -: 8],
                                resp_frame[8*(RESP_LEN-1)-1 -: 8]};

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            tx_start    <= 1'b0;
            tx_idx      <= 3'd0;
            rx_idx      <= 0;
            to_cnt      <= 0;
            crc_latched <= 16'h0000;
        end else begin
            tx_start <= 1'b0;
            done     <= 1'b0;
            error    <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy        <= 1'b1;
                        crc_latched <= crc_tx;
                        tx_idx      <= 3'd0;
                        state       <= S_SEND;
                    end
                end

                S_SEND: begin
                    if (!tx_busy && !tx_start) begin
                        case (tx_idx)
                            3'd0: tx_data <= slave_addr;
                            3'd1: tx_data <= func_code;
                            3'd2: tx_data <= reg_addr[15:8];
                            3'd3: tx_data <= reg_addr[7:0];
                            3'd4: tx_data <= data_or_count[15:8];
                            3'd5: tx_data <= data_or_count[7:0];
                            3'd6: tx_data <= crc_latched[7:0];   // CRC low byte first
                            3'd7: tx_data <= crc_latched[15:8];  // then CRC high byte
                            default: tx_data <= 8'h00;
                        endcase
                        tx_start <= 1'b1;
                        if (tx_idx == 3'd7) begin
                            rx_idx <= 0;
                            to_cnt <= 0;
                            state  <= S_WAIT_RSP;
                        end else begin
                            tx_idx <= tx_idx + 1'b1;
                        end
                    end
                end

                S_WAIT_RSP: begin
                    if (byte_in) begin
                        if (rx_idx + 1 == RESP_LEN) begin
                            state <= S_CHECK;
                        end else begin
                            rx_idx <= rx_idx + 1'b1;
                            to_cnt <= 0;
                        end
                    end else if (to_cnt == TIMEOUT_TICKS - 1) begin
                        busy  <= 1'b0;
                        error <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        to_cnt <= to_cnt + 1'b1;
                    end
                end

                S_CHECK: begin
                    // validate: address+function echoed correctly, no
                    // exception (func code bit 7 set), and the
                    // received frame's CRC matches what we calculate
                    // over the bytes we captured
                    busy <= 1'b0;
                    if (resp_frame[7:0] != slave_addr ||
                        resp_frame[15:8] != func_code ||
                        resp_frame[15] == 1'b1 ||
                        rx_crc_calc != rx_crc_recv) begin
                        error <= 1'b1;
                    end else begin
                        done <= 1'b1;
                    end
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
