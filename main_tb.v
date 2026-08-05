`timescale 1ns / 1ps
//======================================================================
// main_tb.v  --  integration testbench for `main` (LTR390UV Modbus hub)
//
// Models the DFRobot SEN0540 as a Modbus RTU slave on the UV UART:
//   - receives the FPGA's request bytes on uv_cr (FPGA TX),
//   - echoes Write-Single (0x06) requests,
//   - answers the Read (0x04) request with a canned ALS value.
// Then checks that:
//   (a) the FPGA issued the exact ambient-light boot sequence, and
//   (b) the FPGA emits the correct Bluetooth frame 0xAA + 3 ALS bytes.
//
// Oracle independence: the ALS value and expected boot registers are
// TB constants (from the datasheet/driver), not read from the RTL.
// The slave-side CRC uses an explicit loop, separate from the DUT.
//
// Run: iverilog -g2012 -o main_tb.out main_tb.v main.v \
//        modbus_rtu_master.v basys3_sensors.v && vvp main_tb.out
//======================================================================
module main_tb;

    // To keep simulation fast, the DUT is told CLK_HZ = 1 MHz (below) while
    // still clocked at 100 MHz here, so a bit is BIT_TICKS = 1e6/9600 = 104
    // clocks (~104 real clocks) instead of ~10416. BIT_NS matches: 104 * 10ns.
    localparam integer BIT_NS = 1040;            // one bit period (ns) for the slave/BT
    localparam [23:0]  ALS_TEST = 24'h012345;    // reg0=0x2345, reg1=0x0001

    reg  clk;
    reg  rst;
    reg  sw_uv_off;
    reg  uv_dt;     // sensor -> FPGA (TB drives)
    reg  bt_rx;
    wire uv_cr;     // FPGA -> sensor (TB samples)
    wire bt_tx;
    wire [15:0] led;

    integer pass_count;
    integer fail_count;
    integer i;

    // Recorded requests seen by the slave (first 4: 3 writes + 1 read)
    reg [7:0]  seen_func [0:3];
    reg [15:0] seen_reg  [0:3];
    reg [15:0] seen_data [0:3];
    integer    req_count;

    reg [7:0] req [0:7];
    reg [7:0] resp7 [0:6];
    reg [15:0] c;

    // BT frame capture
    reg [7:0] f0, f1, f2, f3;

    //------------------------------------------------------------------
    // DUT -- delays shrunk for fast simulation
    //------------------------------------------------------------------
    main #(
        .CLK_HZ         (1_000_000),   // fictional (sim speed): 104 clk/bit, not 10416
        .BAUD           (9600),
        .UV_TIMEOUT_MS  (5),           // -> 5000 clk timeout (> response latency)
        .UV_BOOT_CYCLES (2000),
        .UV_POLL_CYCLES (2000)
    ) dut (
        .clk       (clk),
        .rst       (rst),
        .sw_uv_off (sw_uv_off),
        .uv_dt     (uv_dt),
        .uv_cr     (uv_cr),
        .bt_tx     (bt_tx),
        .bt_rx     (bt_rx),
        .led       (led)
    );

    always #5 clk = ~clk;

    //------------------------------------------------------------------
    // Independent Modbus CRC-16 (explicit loop) over the 7 payload bytes
    //------------------------------------------------------------------
    function [15:0] crc16_7;
        input [7:0] b0,b1,b2,b3,b4,b5,b6;
        integer j,k;
        reg [15:0] cc;
        reg [7:0] bytes [0:6];
        begin
            bytes[0]=b0; bytes[1]=b1; bytes[2]=b2; bytes[3]=b3;
            bytes[4]=b4; bytes[5]=b5; bytes[6]=b6;
            cc = 16'hFFFF;
            for (j=0;j<7;j=j+1) begin
                cc = cc ^ {8'h00, bytes[j]};
                for (k=0;k<8;k=k+1)
                    cc = cc[0] ? (cc>>1)^16'hA001 : (cc>>1);
            end
            crc16_7 = cc;
        end
    endfunction

    //------------------------------------------------------------------
    // Slave UART helpers (time-based, decoupled from the FPGA clock)
    //------------------------------------------------------------------
    task uart_send_dt;                 // drive one byte on uv_dt, 8N1 LSB-first
        input [7:0] b;
        integer n;
        begin
            uv_dt = 1'b0; #BIT_NS;          // start
            for (n=0;n<8;n=n+1) begin uv_dt = b[n]; #BIT_NS; end
            uv_dt = 1'b1; #BIT_NS;          // stop
        end
    endtask

    task uart_recv_cr;                 // receive one byte from uv_cr, 8N1
        output [7:0] b;
        integer n;
        begin
            @(negedge uv_cr);              // start bit
            #(BIT_NS/2);                   // mid start
            for (n=0;n<8;n=n+1) begin #BIT_NS; b[n] = uv_cr; end
            #BIT_NS;                       // ride out stop
        end
    endtask

    //------------------------------------------------------------------
    // Slave model: receive request, respond per function code
    //------------------------------------------------------------------
    initial begin
        uv_dt = 1'b1;
        req_count = 0;
        @(negedge rst);                    // wait for reset to release
        forever begin
            for (i=0;i<8;i=i+1) uart_recv_cr(req[i]);
            if (req_count < 4) begin
                seen_func[req_count] <= req[1];
                seen_reg [req_count] <= {req[2], req[3]};
                seen_data[req_count] <= {req[4], req[5]};
                req_count = req_count + 1;
            end
            #(BIT_NS);                      // processing gap
            if (req[1] == 8'h06) begin
                // Write Single: echo the 8-byte request back
                for (i=0;i<8;i=i+1) uart_send_dt(req[i]);
            end else if (req[1] == 8'h04) begin
                // Read Input Regs: respond with the canned ALS value
                resp7[0]=8'h1C; resp7[1]=8'h04; resp7[2]=8'h04;
                resp7[3]=ALS_TEST[15:8];  // reg0 high
                resp7[4]=ALS_TEST[7:0];   // reg0 low
                resp7[5]=8'h00;           // reg1 high
                resp7[6]=ALS_TEST[23:16]; // reg1 low
                for (i=0;i<7;i=i+1) uart_send_dt(resp7[i]);
                c = crc16_7(resp7[0],resp7[1],resp7[2],resp7[3],resp7[4],resp7[5],resp7[6]);
                uart_send_dt(c[7:0]);
                uart_send_dt(c[15:8]);
            end
        end
    end

    //------------------------------------------------------------------
    // Capture one BT byte and one 4-byte frame
    //------------------------------------------------------------------
    task bt_get;
        output [7:0] b;
        integer n;
        begin
            @(negedge bt_tx);
            #(BIT_NS/2);
            for (n=0;n<8;n=n+1) begin #BIT_NS; b[n] = bt_tx; end
            #BIT_NS;
        end
    endtask

    task bt_recv_frame;
        begin
            bt_get(f0); bt_get(f1); bt_get(f2); bt_get(f3);
        end
    endtask

    //------------------------------------------------------------------
    // Checks
    //------------------------------------------------------------------
    task chk8;
        input [7:0] got, exp;
        input [127:0] name;
        begin
            if (got === 8'bx || (^got) === 1'bx) begin
                $display("FAIL [%0s]: value is X", name);
                fail_count = fail_count + 1;
            end else if (got === exp) begin
                $display("PASS [%0s]: 0x%02h", name, got);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL [%0s]: expected 0x%02h, got 0x%02h", name, exp, got);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task chk16;
        input [15:0] got, exp;
        input [127:0] name;
        begin
            if (got === exp) begin
                $display("PASS [%0s]: 0x%04h", name, got);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL [%0s]: expected 0x%04h, got 0x%04h", name, exp, got);
                fail_count = fail_count + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    initial begin
        $dumpfile("main_tb.vcd");
        $dumpvars(0, main_tb);
    end
    initial begin
        #3_000_000;
        $display("TIMEOUT (stalled)");
        $display("RESULT: %0d/%0d checks passed", pass_count, pass_count + fail_count);
        $finish;
    end

    //------------------------------------------------------------------
    initial begin
        clk = 0; rst = 0; sw_uv_off = 0; bt_rx = 1;
        pass_count = 0; fail_count = 0;

        rst = 1'b1;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
        $display("Reset released, waiting for boot + first ALS reading...");

        // First BT frame appears only after the 3 config writes and a read.
        bt_recv_frame();

        // (a) Boot sequence content
        chk8 (seen_func[0], 8'h06,   "boot1 func");
        chk16(seen_reg [0], 16'h000D,"boot1 reg (MEAS_RATE)");
        chk16(seen_data[0], 16'h0022,"boot1 val");
        chk8 (seen_func[1], 8'h06,   "boot2 func");
        chk16(seen_reg [1], 16'h0006,"boot2 reg (GAIN)");
        chk16(seen_data[1], 16'h0001,"boot2 val");
        chk8 (seen_func[2], 8'h06,   "boot3 func");
        chk16(seen_reg [2], 16'h000E,"boot3 reg (MAIN_CTRL)");
        chk16(seen_data[2], 16'h0002,"boot3 val");
        chk8 (seen_func[3], 8'h04,   "read func");
        chk16(seen_reg [3], 16'h0007,"read reg (ALS_DATA_LOW)");
        chk16(seen_data[3], 16'h0002,"read count");

        // (b) BT frame: 0xAA sync + ALS MSB..LSB
        chk8(f0, 8'hAA,            "BT sync");
        chk8(f1, ALS_TEST[23:16], "BT als[23:16]");
        chk8(f2, ALS_TEST[15:8],  "BT als[15:8]");
        chk8(f3, ALS_TEST[7:0],   "BT als[7:0]");

        // Confirm polling repeats: a second frame should arrive
        bt_recv_frame();
        chk8(f0, 8'hAA,            "BT sync (2nd)");
        chk8(f3, ALS_TEST[7:0],   "BT als[7:0] (2nd)");

        $display("RESULT: %0d/%0d checks passed", pass_count, pass_count + fail_count);
        $finish;
    end

endmodule
