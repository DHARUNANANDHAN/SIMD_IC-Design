`timescale 1ns/1ps

//=============================================================================
// TOP-LEVEL SIMD AI ACCELERATOR MODULE
//=============================================================================
module simd_ai_accelerator #(
    parameter SIMD_LANES = 4,           // Number of parallel MAC units
    parameter DATA_WIDTH = 8,           // Input data width
    parameter WEIGHT_WIDTH = 8,         // Weight width
    parameter ACC_WIDTH = 16            // Accumulator width
)(
    // Clock and Reset
    input  wire clk,
    input  wire rst_n,

    // Control Signals
    input  wire enable,
    input  wire simd_start,

    // SIMD Vector Inputs
    input  wire [SIMD_LANES*DATA_WIDTH-1:0] vec_data_in,
    input  wire [SIMD_LANES*WEIGHT_WIDTH-1:0] vec_weight_in,
    input  wire data_valid,

    // SIMD Vector Outputs
    output reg  [SIMD_LANES*ACC_WIDTH-1:0] vec_mac_out,
    output reg  [SIMD_LANES*8-1:0] vec_activation_out,
    output reg  result_valid,

    // Status Outputs
    output wire busy,
    output wire [7:0] power_status
);

    // Internal wires
    wire gated_clk;
    wire power_gated;
    wire [SIMD_LANES-1:0] mac_done;
    wire [SIMD_LANES-1:0] lane_enable;
    wire all_lanes_done;
    wire [ACC_WIDTH-1:0] mac_results [0:SIMD_LANES-1];
    wire [7:0] activation_results [0:SIMD_LANES-1];

    // FSM states
    localparam IDLE=3'b000, LOAD_VECTORS=3'b001, SIMD_COMPUTE=3'b010,
               ACTIVATION=3'b011, OUTPUT=3'b100;

    reg [2:0] state, next_state;

    assign busy = (state != IDLE);
    assign all_lanes_done = &mac_done;

    // Clock Gating
    clock_gating_cell u_main_clock_gate (
        .clk_in(clk),
        .enable(enable & ~power_gated),
        .test_en(1'b0),
        .gated_clk(gated_clk)
    );

    // Power Gating
    power_gating_controller u_power_gate (
        .clk(gated_clk),
        .rst_n(rst_n),
        .module_active(enable),
        .power_gate_signal(power_gated)
    );

    // SIMD Enable
    assign lane_enable = (state == SIMD_COMPUTE) ? {SIMD_LANES{1'b1}} : {SIMD_LANES{1'b0}};

    // Generate SIMD Lanes
    genvar i;
    generate
        for (i=0; i<SIMD_LANES; i=i+1) begin: LANE
            wire [DATA_WIDTH-1:0] lane_data = vec_data_in[(i+1)*DATA_WIDTH-1 : i*DATA_WIDTH];
            wire [WEIGHT_WIDTH-1:0] lane_weight = vec_weight_in[(i+1)*WEIGHT_WIDTH-1 : i*WEIGHT_WIDTH];

            simd_mac_unit #(.DATA_WIDTH(DATA_WIDTH),.WEIGHT_WIDTH(WEIGHT_WIDTH),.ACC_WIDTH(ACC_WIDTH)) mac_lane (
                .clk(gated_clk),
                .rst_n(rst_n),
                .enable(lane_enable[i]),
                .data_in(lane_data),
                .weight_in(lane_weight),
                .mac_result(mac_results[i]),
                .mac_done(mac_done[i])
            );

            activation_function act_lane (
                .clk(gated_clk),
                .rst_n(rst_n),
                .data_in(mac_results[i]),
                .activation_out(activation_results[i])
            );
        end
    endgenerate

    // FSM
    always @(posedge gated_clk or negedge rst_n)
        if(!rst_n) state <= IDLE; else state <= next_state;

    always @(*) begin
        next_state = state;
        case(state)
            IDLE:        if(enable && simd_start && data_valid) next_state = LOAD_VECTORS;
            LOAD_VECTORS: next_state = SIMD_COMPUTE;
            SIMD_COMPUTE: if(all_lanes_done) next_state = ACTIVATION;
            ACTIVATION:   next_state = OUTPUT;
            OUTPUT:       next_state = IDLE;
            default:      next_state = IDLE;
        endcase
    end

    // Output logic (fixed: no variable part-select)
    always @(posedge gated_clk or negedge rst_n) begin
        if(!rst_n) begin
            vec_mac_out <= 0;
            vec_activation_out <= 0;
            result_valid <= 0;
        end else if(state == OUTPUT) begin
            vec_mac_out <= {mac_results[3], mac_results[2], mac_results[1], mac_results[0]};
            vec_activation_out <= {activation_results[3], activation_results[2], activation_results[1], activation_results[0]};
            result_valid <= 1;
        end else begin
            result_valid <= 0;
        end
    end

    assign power_status = {state[2:0], all_lanes_done, power_gated, result_valid, busy, 1'b0};

endmodule


//-----------------------------------------------------------------------------
// MAC UNIT
//-----------------------------------------------------------------------------
module simd_mac_unit #(parameter DATA_WIDTH=8,WEIGHT_WIDTH=8,ACC_WIDTH=16)(
    input wire clk, rst_n, enable,
    input wire [DATA_WIDTH-1:0] data_in,
    input wire [WEIGHT_WIDTH-1:0] weight_in,
    output reg [ACC_WIDTH-1:0] mac_result,
    output reg mac_done
);
    reg [ACC_WIDTH-1:0] acc;
    reg [1:0] s;
    localparam IDLE=0, MULT=1, ACCUM=2, DONE=3;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin s<=IDLE; acc<=0; mac_result<=0; mac_done<=0; end
        else case(s)
            IDLE: begin mac_done<=0; if(enable) s<=MULT; end
            MULT: begin acc<=data_in*weight_in; s<=ACCUM; end
            ACCUM: begin mac_result<=acc; mac_done<=1; s<=DONE; end
            DONE: s<=IDLE;
        endcase
    end
endmodule


//-----------------------------------------------------------------------------
// CLOCK GATE
//-----------------------------------------------------------------------------
module clock_gating_cell(input wire clk_in, enable, test_en, output wire gated_clk);
    reg en_latched;
    always @(*) if(!clk_in) en_latched = enable | test_en;
    assign gated_clk = clk_in & en_latched;
endmodule


//-----------------------------------------------------------------------------
// POWER GATE
//-----------------------------------------------------------------------------
module power_gating_controller(input wire clk, rst_n, module_active, output reg power_gate_signal);
    reg [3:0] idle_count;
    localparam IDLE_THRESHOLD = 10;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin idle_count<=0; power_gate_signal<=0; end
        else if(module_active) begin idle_count<=0; power_gate_signal<=0; end
        else if(idle_count < IDLE_THRESHOLD) idle_count<=idle_count+1;
        else power_gate_signal<=1;
endmodule


//-----------------------------------------------------------------------------
// ACTIVATION FUNCTION (ReLU-like)
//-----------------------------------------------------------------------------
module activation_function(input wire clk, rst_n, input wire [15:0] data_in, output reg [7:0] activation_out);
    always @(posedge clk or negedge rst_n)
        if(!rst_n) activation_out <= 0;
        else if(data_in[15]) activation_out <= 0;
        else activation_out <= (|data_in[15:8]) ? 8'hFF : data_in[7:0];
endmodule
