module rx (
	input wire			clk,
	input wire			rst,
	input wire			RX,				// data input line
	input wire			queue_full,
	input wire [1:0]	id,				// sensors ID, based off of the channel the sensor's connected to
	input wire			tick,			// baud tick

	output reg [12:0]	data_out,		// pushed to queue, id[1:0] | data[7:0] | parity flag | framing flag | overrun flag
	output reg			wr_en			// write enable for the queue
);

	localparam			BIT_COUNT_MAX	= 10'd8;
	localparam			TICK_MAX		= 10'd15;
	localparam			TICK_MIDDLE		= 10'd7;

	localparam			IDLE			= 2'b00;
	localparam			START			= 2'b01;
	localparam			SAMPLING		= 2'b10;
	localparam			STOP			= 2'b11;

	reg			[1:0]	state, next_state;
	reg			[7:0]	data_byte;
	reg			[9:0]	bit_count;
	reg					parity_err;
	reg			[9:0]	ticks_elapsed;


	// current state
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			state <= IDLE;
		end else if (tick) begin
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
				if (ticks_elapsed == TICK_MIDDLE && ~RX) begin
					next_state = SAMPLING;
				end else if (RX) begin
					next_state = IDLE;
				end
			end
			SAMPLING: begin
				if (ticks_elapsed == TICK_MAX && bit_count == BIT_COUNT_MAX) begin
					next_state = STOP;
				end
			end
			STOP: begin
				if (ticks_elapsed == TICK_MAX) begin
					next_state = IDLE;
				end
			end
		endcase
	end

	// datapath assignments
	always @(posedge clk) begin
		if (tick) begin
			parity_err		<= 0;
			wr_en			<= 0;
			data_out		<= 13'b0;
			data_byte       <= 8'b0;
			ticks_elapsed <= 0;
			bit_count <= 0;

			case(state)
				START: begin
					if (ticks_elapsed < TICK_MIDDLE) begin
						ticks_elapsed <= ticks_elapsed + 1;
					end
				end
				SAMPLING: begin
					ticks_elapsed <= ticks_elapsed + 1;
					data_byte <= data_byte; // Persist data across ticks until the next bit is sampled
					bit_count <= bit_count;

					if (ticks_elapsed == TICK_MAX) begin
						if (bit_count < BIT_COUNT_MAX) begin
							data_byte <= {RX, data_byte[7:1]};
							ticks_elapsed <= 0;
							bit_count <= bit_count + 1;
						end else begin
							parity_err <= ^({data_byte, RX});	// check parity (even parity assumed here, add ~ at beginning for odd parity)
							ticks_elapsed <= 0;
						end
					end
				end
				STOP: begin
					ticks_elapsed <= ticks_elapsed + 1;
					data_byte <= data_byte; // Keep the changed value until we store it, don't reset until we leave STOP
					parity_err <= parity_err;
					if (ticks_elapsed == TICK_MAX) begin
						wr_en <= 1'b1;
						data_out <= {id, data_byte, parity_err, ~RX, queue_full};
					end
				end
			endcase
		end
	end
endmodule