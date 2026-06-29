module rx (
	input wire			clk,
	input wire			rst,
	input wire			RX,				// data input line
	input wire			queue_full,
	input wire [1:0]	id,				// sensors ID, based off of the channel the sensor's connected to
	input wire			tick,			// baud tick

	output reg [10:0]	data_out,		// pushed to queue, id[1:0] | data[7:0] | parity flag | framing flag | overrun flag
	output reg			wr_en			// write enable for the queue
);

	localparam			BIT_COUNT_MAX	= 10'd8;
	localparam			TICK_MAX		= 10'd16;
	localparam			TICK_MIDDLE		= 10'd8;

	localparam			IDLE			= 2'b00;
	localparam			START			= 2'b01;
	localparam			SAMPLING		= 2'b10;
	localparam			STOP			= 2'b11;

	reg			[1:0]	state, next_state;
	reg			[8:0]	data_byte;
	reg			parity_err;
	reg			framing_err;
	reg			overrun_err;

	// current state
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			state <= IDLE;
		end else begin
			state <= next_state;
		end
	end

	// next state
	always @(*) begin
		next_state = state;		// default = stay in current state

		case (state)
			IDLE: begin
				if (~RX) begin
					next_state = START;
				end
			end
			START: begin
				if (tick == TICK_MIDDLE && ~RX) begin
					next_state = SAMPLING;
				end else if (RX) begin
					next_state = IDLE;
				end
			end
			SAMPLING: begin
				if (tick == TICK_MAX && bit_count == BIT_COUNT_MAX) begin
					next_state = STOP;
				end
			end
			STOP: begin
				if (tick == TICK_MAX) begin
					next_state = IDLE;
				end
			end
		endcase
	end

	// datapath assignments
	always @(*) begin
		framing_err		= 0;
		parity_err		= 0;
		overrun_err		= 0;
		wr_en			= 0;
		data_out		= 11'b0;
		tick = 0;
		bit_count = 0;

		case(state)
			SAMPLING: begin
				if (bit_count < BIT_COUNT_MAX) begin
					RX >> data_out;
					tick = 0;
					bit_count = bit_count + 1;
				end else begin
					parity_err = ^data_byte ^ RX;	// check parity (even parity assumed here, add ~ at beginning for odd parity)
					tick = 0;
					bit_count = 0;
				end
			end
			STOP: begin
				if (~RX) begin
					framing_err = 1'b1;
				end else if (queue_full) begin
					overrun_err = 1'b1;
				end

				wr_en = 1'b1;
				data_out = {id, data_byte, parity_err, framing_err, overrun_err};
			end
		endcase
	end