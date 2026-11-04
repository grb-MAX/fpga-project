// 保留此模块以匹配 Gowin 工程的源文件列表。
// 顶层 acg525_pendulum_top 当前没有实例化该模块，
// 因而本模块不会进入 FPGA 的控制逻辑。
module one_euro_filter_signed #(
    parameter integer MIN_CUTOFF_MHZ      = 5_000,
    parameter integer BETA_MHZ_PER_SAMPLE = 500,
    parameter integer DERIV_ALPHA_Q15     = 16_384
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_tick,
    input  wire               enable,
    input  wire signed [31:0] input_value,
    output reg  signed [31:0] output_value
);

    // 直通实现。将来若重新需要 One Euro 滤波，应另行验证后替换此实现。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            output_value <= 32'sd0;
        else if (sample_tick)
            output_value <= input_value;
    end

endmodule
