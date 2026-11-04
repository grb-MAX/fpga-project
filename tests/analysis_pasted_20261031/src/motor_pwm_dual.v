// AT8236 双输入 PWM 驱动接口。正命令驱动 IN1，负命令驱动 IN2。
// 活跃输入为低有效 PWM，另一输入保持高；零命令时两输入均为低。
// 驱动占空比 = duty/PWM_PERIOD；幅值决定有效拍数，方向仅由命令符号选择。
module motor_pwm_dual #(
    parameter integer PWM_PERIOD = 2_500, // PWM 周期的系统时钟数；50 MHz、2500 拍对应 20 kHz。
    parameter integer DEAD_BAND = 0 // 命令幅值死区阈值；幅值不超过该值时占空比置零。
) (
    input  wire               clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire               rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire               enable, // 电机驱动使能；未使能时两路输入均输出低电平。
    input  wire signed [15:0] command, // 带符号 PWM 命令：正值选 IN1，负值选 IN2，绝对值决定占空比。
    output reg                motor_in1, // AT8236 IN1，正命令时输出低有效 PWM，负命令时保持高。
    output reg                motor_in2 // AT8236 IN2，负命令时输出低有效 PWM，正命令时保持高。
);

    reg [15:0] pwm_counter; // 载波计数器，从 0 累加到 PWM_PERIOD-1 后回零。
    reg [15:0] duty; // 死区和限幅处理后的有效驱动拍数，范围为 0~PWM_PERIOD-1。
    reg        command_positive; // 命令方向标记；为 1 选择正向通道，为 0 选择反向通道。
    reg signed [16:0] command_abs; // 17 位有符号幅值中间量，用于容纳 16 位输入命令的绝对值。

    // 取命令绝对值作为 PWM 比较值，符号只决定驱动方向。
    always @* begin
        if (command < 0) begin
            command_abs = -command;
            command_positive = 1'b0;
        end else begin
            command_abs = command;
            command_positive = 1'b1;
        end

        // 先判断命令死区，再将幅值限制在 0~PWM_PERIOD-1。
        // DEAD_BAND 是命令幅值阈值，此处没有额外的换向延时计数。
        if (command_abs <= DEAD_BAND)
            duty = 16'd0;
        else if (command_abs > PWM_PERIOD - 1)
            duty = PWM_PERIOD - 1;
        else
            duty = command_abs[15:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 16'd0;
            motor_in1 <= 1'b0;
            motor_in2 <= 1'b0;
        end else begin
            // PWM 计数器形成固定频率载波，默认 20 kHz。
            if (pwm_counter == PWM_PERIOD - 1)
                pwm_counter <= 16'd0;
            else
                pwm_counter <= pwm_counter + 1'b1;

            // 每拍默认输出双低；只有使能且 duty 非零时才产生方向驱动波形。
            motor_in1 <= 1'b0;
            motor_in2 <= 1'b0;
            if (enable && (duty != 0)) begin
                // 与已验证的 MSPM0 驱动波形一致：有效通道输出低有效 PWM，
                // 另一通道保持高电平。
                // 计数值小于 duty 时有效通道为低，其余时间两输入为高。
                if (command_positive) begin
                    motor_in1 <= (pwm_counter >= duty);
                    motor_in2 <= 1'b1;
                end else begin
                    motor_in1 <= 1'b1;
                    motor_in2 <= (pwm_counter >= duty);
                end
            end
        end
    end

endmodule
