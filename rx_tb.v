`timescale 1ns / 10ps

module rx_tb();

	reg clk;
	reg rst;
	reg RX;
	reg queue_full;
	reg [1:0] id;

	wire tick;
	wire wr_en;
	wire [12:0] data_out;

	reg [10:0] frame;
	reg [12:0] expected_data_out;
	integer error_count;

	baud_tick_gen baud_tick_gen_inst (
		.clk(clk),
		.rst(rst),
		.tick(tick)
	);

	rx uut (
		.clk(clk),
		.rst(rst),
		.RX(RX),
		.queue_full(queue_full),
		.id(id),
		.tick(tick),
		.data_out(data_out),
		.wr_en(wr_en)
	);

	// Generate 100MHz clock
	always #5 clk = ~clk;
	

	task reset;
		begin
			rst = 1;
			@(posedge clk);
			@(posedge clk);
			@(posedge clk);
			rst = 0;
		end
	endtask

	task send_frame;
		input	[10:0] frame;

		integer i;
		begin
			for (i = 0; i < 10; i = i + 1) begin
				RX = frame[i];
				repeat (16) @(posedge tick);
			end
			RX = frame[10]; // stop bit
		end
	endtask

	task test_states;
		integer i;
		integer timeout;

		begin
			frame = 11'b10101001110;

			// test IDLE freewheele
			for (i = 0; i < 3; i = i + 1) begin
				RX = 1;
				repeat (16) @(posedge tick);

				if (uut.state != uut.IDLE) begin
					$error("Test failed: state does not match expected value. Expected IDLE, got %b", uut.state);
					error_count = error_count + 1;
				end
			end

			// test START fallback
			RX = 0;
			repeat(4) @(posedge tick);
			if (uut.state != uut.START) begin
				$error("Test failed: state does not match expected value. Expected START, got %b", uut.state);
				error_count = error_count + 1;
			end
			@(posedge tick);
			RX = 1;
			repeat(3) @(posedge tick); // if we go back to IDLE tick shouldn't incriment so we use clk instead
			if (uut.state != uut.IDLE) begin
				$error("Test failed: state does not match expected value. Expected IDLE, got %b", uut.state);
				error_count = error_count + 1;
			end

			// test SAMPLING
			RX = frame[0]; // start bit
			
			timeout = 0;
			while ((uut.state != uut.SAMPLING) && timeout < 250) begin
				@(posedge tick);
				timeout = timeout + 1;
			end

			if (timeout >= 250) begin
				$error("Test failed: never reached SAMPLING (state=%b, ticks_elapsed=%d) timed out", uut.state, uut.ticks_elapsed);
				error_count = error_count + 1;
			end else if (uut.ticks_elapsed != 0) begin // tick should be 0 at start of every state
				$error("Test failed: ticks_elapsed did not reset on state transition. Expected 0, got %d", uut.ticks_elapsed);
				error_count = error_count + 1;
			end

			RX = frame[1]; // first data bit
			repeat (10) @(posedge tick);
			if (uut.state != uut.SAMPLING) begin
				$error("Test failed: state does not match expected value. Expected SAMPLING, got %b", uut.state);
				error_count = error_count + 1;
			end
			repeat (6) @(posedge tick);
			if (uut.data_byte[7] != frame[1]) begin
				$error("Test failed: data_byte didn't receive expected value. Expected %b, got %b", frame[1], uut.data_byte[0]);
				error_count = error_count + 1;
			end
			repeat (6) @(posedge tick);
			if (uut.bit_count != 1) begin
				$error("Test failed: bit_count didn't increment. Expected 1, got %d", uut.bit_count);
				error_count = error_count + 1;
			end

			timeout = 0;
			while ((uut.state != uut.STOP || uut.ticks_elapsed != 10) && timeout < 250) begin
				@(posedge tick);
				timeout = timeout + 1;
			end

			if (timeout >= 250) begin
				$error("Test failed: never reached STOP (state=%b, ticks_elapsed=%d) timed out", uut.state, uut.ticks_elapsed);
				error_count = error_count + 1;
			end else begin
				@(posedge clk);
				if (uut.state != uut.STOP) begin
					$error("Test failed: state does not match expected value. Expected STOP, got %b", uut.state);
					error_count = error_count + 1;
				end
			end

			$display("State tests concluded");
		end
	endtask

	task check_data_out;
		input [12:0] expected;
		integer t;
		reg seen;
		begin
			seen = 0;
			for (t = 0; t < 200 && !seen; t = t + 1) begin
				@(posedge tick);
				if (wr_en) seen = 1;
			end
			if (!seen) begin
				$error("Test failed: wr_en never asserted within timeout");
				error_count = error_count + 1;
			end else if (data_out != expected) begin
				$error("Test failed: data_out mismatch. Expected %b, got %b", expected, data_out);
				error_count = error_count + 1;
			end
		end
	endtask

	function [12:0] construct_data_out;
		input [1:0] id;
		input [10:0] frame;
		input parity_err;
		input framing_err;
		input overrun_err;

		begin
			construct_data_out = {id, frame[8:1], parity_err, framing_err, overrun_err};
		end
	endfunction

	// Simulation control
	initial begin
		clk = 0;
		rst = 0;
		RX = 1;
		queue_full = 0;
		id = 2'b10;
		error_count = 0;
		$display("Values Initialized");

		$dumpfile("rx_tb.vcd");
		$dumpvars(0, rx_tb);

		reset();
		$display("Reset complete");

		// frame testing
		frame = 11'b10101001100; // good frame, no errors
		send_frame(frame);
		expected_data_out = construct_data_out(id, frame, 0, 0, 0);
		check_data_out(expected_data_out);
		reset();

		frame = 11'b01101001100; // bad frame, all flags should get set
		queue_full = 1'b1;
		send_frame(frame);
		expected_data_out = construct_data_out(id, frame, 1, 1, 1);
		check_data_out(expected_data_out);
		queue_full = 1'b0;
		reset();


		// state testing
		test_states();
		reset();


		// Test multiple frames back to back
		frame = 11'b10101001100; // good frame, no errors
		send_frame(frame);
		expected_data_out = construct_data_out(id, frame, 0, 0, 0);
		check_data_out(expected_data_out);

		frame = 11'b10101001100; // good frame, no errors
		send_frame(frame);
		expected_data_out = construct_data_out(id, frame, 0, 0, 0);
		check_data_out(expected_data_out);

		frame = 11'b01101001100; // bad frame, all flags should get set
		queue_full = 1'b1;
		send_frame(frame);
		expected_data_out = construct_data_out(id, frame, 1, 1, 1);
		check_data_out(expected_data_out);
		queue_full = 1'b0;


		$display("All tests concluded with %d errors", error_count);
		$finish;
	end

endmodule