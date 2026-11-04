
`timescale 1ns / 1ps

module position_trajectory #(
    parameter integer AMPLITUDE_COUNTS = 100,
    parameter integer PERIOD_SAMPLES = 4096
)(
    input wire clk,
    input wire rst_n,
    input wire sample_tick,

    input wire start_request,
    input wire clear_request,
    input wire advance_enable,

    input wire signed [31:0] encoder_position,

    output wire signed [31:0] trajectory_ref
);

    //==================================================
    // 1. 相位和中心位置
    //==================================================

    reg [31:0] phase;

    reg signed [31:0] center_position;

    reg trajectory_valid;

    localparam [31:0] PHASE_STEP =
        (64'd4294967296 / PERIOD_SAMPLES);

    //==================================================
    // 2. 四分之一周期正弦查表，Q15
    //==================================================

    function signed [15:0] quarter_sine;
        input [4:0] idx;

        begin
            case (idx)
                5'd0:  quarter_sine = 16'sd0;
                5'd1:  quarter_sine = 16'sd3212;
                5'd2:  quarter_sine = 16'sd6393;
                5'd3:  quarter_sine = 16'sd9512;
                5'd4:  quarter_sine = 16'sd12539;
                5'd5:  quarter_sine = 16'sd15446;
                5'd6:  quarter_sine = 16'sd18204;
                5'd7:  quarter_sine = 16'sd20787;
                5'd8:  quarter_sine = 16'sd23170;
                5'd9:  quarter_sine = 16'sd25329;
                5'd10: quarter_sine = 16'sd27245;
                5'd11: quarter_sine = 16'sd28898;
                5'd12: quarter_sine = 16'sd30273;
                5'd13: quarter_sine = 16'sd31356;
                5'd14: quarter_sine = 16'sd32137;
                5'd15: quarter_sine = 16'sd32609;
                5'd16: quarter_sine = 16'sd32767;
                default: quarter_sine = 16'sd0;
            endcase
        end
    endfunction

    //==================================================
    // 3. 64 点完整正弦查表
    //==================================================

    function signed [15:0] sine_lut;
        input [5:0] idx;

        reg [4:0] qidx;

        begin
            case (idx[5:4])

                2'b00: begin
                    qidx = {1'b0, idx[3:0]};
                    sine_lut = quarter_sine(qidx);
                end

                2'b01: begin
                    qidx = 5'd16 - {1'b0, idx[3:0]};
                    sine_lut = quarter_sine(qidx);
                end

                2'b10: begin
                    qidx = {1'b0, idx[3:0]};
                    sine_lut = -quarter_sine(qidx);
                end

                2'b11: begin
                    qidx = 5'd16 - {1'b0, idx[3:0]};
                    sine_lut = -quarter_sine(qidx);
                end

                default: begin
                    qidx = 5'd0;
                    sine_lut = 16'sd0;
                end

            endcase
        end
    endfunction

    //==================================================
    // 4. 查表线性插值
    //==================================================

    wire [5:0] lut_index;
    wire [5:0] lut_next_index;

    wire signed [15:0] sin0;
    wire signed [15:0] sin1;

    wire signed [16:0] sin_diff;

    wire signed [27:0] sin_diff_ext;
    wire signed [27:0] fraction_ext;

    wire signed [27:0] interp_product;
    wire signed [27:0] interp_sum;

    wire signed [16:0] sin_value;

    assign lut_index = phase[31:26];

    assign lut_next_index = lut_index + 6'd1;

    assign sin0 = sine_lut(lut_index);

    assign sin1 = sine_lut(lut_next_index);

    assign sin_diff =
        $signed(sin1) - $signed(sin0);

    assign sin_diff_ext = {
        {11{sin_diff[16]}},
        sin_diff
    };

    assign fraction_ext = {
        17'd0,
        phase[25:16]
    };

    assign interp_product =
        sin_diff_ext * fraction_ext;

    assign interp_sum =
        {{12{sin0[15]}}, sin0} +
        (interp_product >>> 10);

    assign sin_value = interp_sum[16:0];

    //==================================================
    // 5. Q15 转换为编码器计数
    //==================================================

    wire signed [63:0] sin_ext;
    wire signed [63:0] amplitude_ext;

    wire signed [63:0] offset_product;
    wire signed [63:0] offset_counts;

    assign sin_ext = {
        {47{sin_value[16]}},
        sin_value
    };

    assign amplitude_ext = AMPLITUDE_COUNTS;

    assign offset_product =
        sin_ext * amplitude_ext;

    function signed [63:0] round_q15;
        input signed [63:0] value;

        begin
            if (value < 0)
                round_q15 =
                    -(((-value) + 64'sd16384) >>> 15);
            else
                round_q15 =
                    (value + 64'sd16384) >>> 15;
        end
    endfunction

    assign offset_counts = round_q15(offset_product);

    // 轨迹参考 = 启动时中心位置 + 正弦偏移
    assign trajectory_ref =
        center_position + $signed(offset_counts[31:0]);

    //==================================================
    // 6. 轨迹状态控制
    //==================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            phase <= 32'd0;

            center_position <= 32'sd0;

            trajectory_valid <= 1'b0;

        end else if (clear_request) begin

            phase <= 32'd0;

            center_position <= 32'sd0;

            trajectory_valid <= 1'b0;

        end else if (start_request && !trajectory_valid) begin

            // 首次启动时锁存当前位置
            center_position <= encoder_position;

            // 从正弦零点开始
            phase <= 32'd0;

            trajectory_valid <= 1'b1;

        end else if (
            sample_tick &&
            trajectory_valid &&
            advance_enable
        ) begin

            // 只有平衡状态允许推进相位
            phase <= phase + PHASE_STEP;

        end

    end

endmodule
