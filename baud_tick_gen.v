// Baud tick generator for 9600 baud UART RX with 16x oversampling
// Target: 100 MHz clock → tick period = 651 cycles → 153,846 Hz → 9615 baud (0.16% error)
module baud_tick_gen (
    input  wire clk,
    input  wire rst,
    output reg  tick
);

    localparam COUNT_MAX = 10'd649;

    localparam COUNTING = 1'b0;
    localparam PULSE    = 1'b1;

    reg       state;
    reg [9:0] count;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= COUNTING;
            count <= 10'd0;
            tick  <= 1'b0;
        end else begin
            tick <= 1'b0;
            case (state)
                COUNTING: begin
                    if (count == COUNT_MAX) begin
                        state <= PULSE;
                        count <= 10'd0;
                    end else begin
                        count <= count + 10'd1;
                    end
                end
                PULSE: begin
                    tick  <= 1'b1;
                    state <= COUNTING;
                end
                default: begin
                    state <= COUNTING;
                    count <= 10'd0;
                end
            endcase
        end
    end

endmodule
