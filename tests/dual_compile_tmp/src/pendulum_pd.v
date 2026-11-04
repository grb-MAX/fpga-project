// 独立 PD 角度控制模块。当前顶层使用 pendulum_controller 中集成的 PD 环。
// 控制公式：Kp*角度误差 + Kd*误差差分 + Kv*编码器速度，并进行对称限幅。
// 本模块保留在工程清单中，但当前顶层未实例化，参数与集成控制器相互独立。
module pendulum_pd #(
    parameter integer KP = 4, // 角度误差比例增益；此处为独立 PD 模块的默认值。
    parameter integer KD = 2, // 相邻采样误差差分增益，未除以采样时间。
    parameter integer KV = 1, // 编码器速度反馈增益，符号沿用本模块原有公式。
    parameter integer OUTPUT_LIMIT = 900 // 输出命令的正负对称幅值上限，单位为 PWM 计数。
) (
    input  wire               clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire               rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire               sample_tick, // 采样更新使能，由调用方提供；当前顶层未实例化本模块。
    input  wire               enable, // 模块运行使能，具体输出更新时刻由下方时序逻辑决定。
    input  wire [11:0]        adc_value, // 完整 12 位 ADC 采样计数，范围 0~4095。
    input  wire [11:0]        angle_zero, // 直立角度的 ADC 参考计数，由调用方提供。
    input  wire signed [31:0] encoder_velocity, // 编码器采样间隔内的位置增量，带符号计数。
    output reg signed [15:0]  command, // 有符号输出命令，符号决定方向，幅值使用 PWM 计数单位。
    output reg signed [15:0]  angle_error // 最近采样的角度误差：ADC 当前值减直立零点。
);

    reg signed [15:0] error_last; // 上一个采样的角度误差；禁用时仍随采样更新。
    reg signed [31:0] control_value; // 本次计算得到的 32 位控制量，限幅前的中间结果。
    reg signed [15:0] error_now; // 本次 ADC 相对直立参考值的有符号偏差。

    // 仅在采样使能时计算并更新输出；复位为异步低有效。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error_last   <= 16'sd0;
            control_value <= 32'sd0;
            command      <= 16'sd0;
            angle_error  <= 16'sd0;
        end else if (sample_tick) begin
            // ADC 减去直立零点；error_now 的正负决定修正方向。
            error_now = $signed({1'b0, adc_value}) - $signed({1'b0, angle_zero});
            angle_error <= error_now;

            // 当前误差使用阻塞赋值供本拍计算；D 项是相邻误差差值，未除以时间。
            control_value = (KP * error_now)
                          + (KD * (error_now - error_last))
                          + (KV * encoder_velocity);
            // 即使 enable 为 0 也继续跟踪误差历史，供后续采样差分使用。
            error_last <= error_now;

            // 控制禁用时输出 0；启用时把控制量限制在 OUTPUT_LIMIT 内。
            if (!enable) begin
                command <= 16'sd0;
            end else if (control_value > OUTPUT_LIMIT) begin
                command <= OUTPUT_LIMIT;
            end else if (control_value < -OUTPUT_LIMIT) begin
                command <= -OUTPUT_LIMIT;
            end else begin
                command <= control_value[15:0];
            end
        end
    end

endmodule
