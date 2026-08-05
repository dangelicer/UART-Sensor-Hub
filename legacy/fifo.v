module sync_fifo #(parameter DEPTH=8, DWIDTH=13)
(
        input               	rstn,               // Active low reset
                            	clk,                // Clock
                            	wr_en, 				// Write enable
                            	rd_en, 				// Read enable
        input      [DWIDTH-1:0] din, 				// Data written into FIFO
        output reg [DWIDTH-1:0] dout, 				// Data read from FIFO
        output              	empty, 				// FIFO is empty when high
                            	full 				// FIFO is full when high
);


  reg [$clog2(DEPTH)-1:0]   wptr;
  reg [$clog2(DEPTH)-1:0]   rptr;

  reg [DWIDTH-1 : 0]    fifo [0:DEPTH-1];

  always @ (posedge clk) begin
    if (!rstn) begin
      wptr <= 0;
    end else begin
      if (wr_en) begin // always write when wr_en is high, even if full, to avoid losing data
        fifo[wptr] <= din;
        wptr <= wptr + 1;
      end
    end
  end

  initial begin
    $monitor("[%0t] [FIFO] wr_en=%0b din=0x%0h rd_en=%0b dout=0x%0h empty=%0b full=%0b",
             $time, wr_en, din, rd_en, dout, empty, full);
  end

  always @ (posedge clk) begin
    if (!rstn) begin
      rptr <= 0;
    end else begin
      if (rd_en & !empty) begin
        dout <= fifo[rptr];
        rptr <= rptr + 1;
      end
    end
  end

  assign full  = (wptr + 1) == rptr;
  assign empty = wptr == rptr;
endmodule