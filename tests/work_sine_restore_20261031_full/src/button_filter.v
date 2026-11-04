// 低有效按键消抖器：每次 sample_tick 采样一次，连续四次一致才确认状态。
// press_pulse 是按下沿事件，只持续一个 clk 周期。
module button_filter (
    input  wire clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire sample_tick, // 采样更新使能脉冲，顶层连接 ADC 完成事件，默认约每 1 ms 一次。
    input  wire button_n, // 低有效按键输入，采样时先取反，按下记为 1。
    output reg  pressed, // 消抖后的持续按下状态；连续四次释放采样后清零。
    output reg  press_pulse // 一次新按下的事件脉冲，仅持续一个系统时钟。
);

    reg [3:0] history; // 最近四次按键采样历史，最低位存本次新采样。
    // 按键采样依赖 sample_tick；事件脉冲的宽度则始终为一个系统时钟。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            history      <= 4'b0000;
            pressed      <= 1'b0;
            press_pulse  <= 1'b0;
        end else begin
            // 事件脉冲默认清零，只在确认一次新按下时置位。
            press_pulse <= 1'b0;
            if (sample_tick) begin
                history <= {history[2:0], ~button_n};
                // 按键引脚低有效，先取反后采样；四个 1 表示稳定按下，四个 0 表示稳定释放。
                // 直接使用拼接后的新历史判定，避免等待 history 寄存器再更新一拍。
                if ({history[2:0], ~button_n} == 4'b1111) begin
                    if (!pressed)
                        press_pulse <= 1'b1;
                    pressed <= 1'b1;
                // 必须稳定释放后才清除 pressed，长按不会重复生成新的按下事件。
                end else if ({history[2:0], ~button_n} == 4'b0000) begin
                    pressed <= 1'b0;
                    press_pulse <= 1'b0;
                end
            end
        end
    end

endmodule
