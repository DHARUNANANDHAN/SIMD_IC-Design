`timescale 1ns/1ps
module tb_simd_ai_accelerator;

    parameter SIMD_LANES=4, DATA_WIDTH=8, WEIGHT_WIDTH=8, ACC_WIDTH=16;
    reg clk, rst_n, enable, simd_start, data_valid;
    reg [SIMD_LANES*DATA_WIDTH-1:0] vec_data_in;
    reg [SIMD_LANES*WEIGHT_WIDTH-1:0] vec_weight_in;
    wire [SIMD_LANES*ACC_WIDTH-1:0] vec_mac_out;
    wire [SIMD_LANES*8-1:0] vec_activation_out;
    wire result_valid, busy;
    wire [7:0] power_status;

    always #50 clk = ~clk;

    simd_ai_accelerator dut (
        .clk(clk), .rst_n(rst_n),
        .enable(enable), .simd_start(simd_start),
        .vec_data_in(vec_data_in), .vec_weight_in(vec_weight_in),
        .data_valid(data_valid),
        .vec_mac_out(vec_mac_out),
        .vec_activation_out(vec_activation_out),
        .result_valid(result_valid),
        .busy(busy), .power_status(power_status)
    );

    initial begin
        $dumpfile("SMID_Chip.vcd");
        $dumpvars(0, tb_simd_ai_accelerator);

        clk=0; rst_n=0; enable=0; simd_start=0; data_valid=0;
        vec_data_in=0; vec_weight_in=0;

        #200 rst_n=1; #100;
        $display("===== SIMD AI Accelerator Simulation =====");

        // Test 1
        enable=1; simd_start=1; data_valid=1;
        vec_data_in = {8'd25,8'd15,8'd20,8'd10};
        vec_weight_in = {8'd2,8'd7,8'd3,8'd5};
        #200 simd_start=0;
        wait(result_valid);
        $display("[Test1] MAC Results: %d %d %d %d",
                 vec_mac_out[15:0], vec_mac_out[31:16], vec_mac_out[47:32], vec_mac_out[63:48]);

        #500;
        $display("Simulation complete!");
        $finish;
    end
endmodule
