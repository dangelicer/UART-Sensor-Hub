`timescale 1 ns / 10 ps

module baud_tick_gen_tb();

	wire tick;

	reg			clk;
	reg			rst;
	integer		cycles; 
	integer		checked_in;			// 0 until reset deasserts
	
	integer		error_count;
	localparam	DURATION = 700000;	// Simulation duration in ns (about 100 ticks)

	initial begin
		clk   = 0;
		cycles = 0;
		error_count = 0;
		checked_in = 0;
	end


	// Generate 100MHz clock
	always #5 clk = ~clk;

	always @(posedge clk) begin
		if (checked_in) begin
			cycles = (cycles == 650) ? 0 : cycles + 1;
		end	
	end


	// Monitor tick signal and check timing
	always @(posedge tick) begin
		if (cycles != 650 && checked_in) begin
			$error("Tick went high at cycle %d, expected 650", cycles);
			error_count = error_count + 1;
		end
	end

	// pulse width check
	always @(negedge tick) begin
		if (cycles != 0 && checked_in) begin
			$error("Tick went low at cycle %d, expected 0", cycles);
			error_count = error_count + 1;
		end
	end


	// Instantiate the baud tick generator
	baud_tick_gen uut (
		.clk(clk),
		.rst(rst),
		.tick(tick)
	);

	initial begin
		rst = 1'b1;


		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		rst = 1'b0;
		checked_in = 1'b1;
	end

	// Run sim
	initial begin
	    
		$dumpfile("baud_tick_gen_tb.vcd");
		$dumpvars(0, baud_tick_gen_tb);

		#(DURATION)

		if (error_count == 0)
			$display("PASS: all ticks correct");
		else
			$display("FAIL: %0d assertion failures", error_count);

		$display("Simulation finished");
		$finish;
	end
endmodule