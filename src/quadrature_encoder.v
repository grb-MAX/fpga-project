// 正交编码器：双触发器同步输入，解码 AB 相的格雷码状态变化。
// position 是累计计数；每次 speed_tick 输出上个采样周期的计数变化量 velocity。
module quadrature_encoder (
    input  wire              clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire              rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire              enc_a_i, // 编码器 A 相异步输入，经两级触发器同步。
    input  wire              enc_b_i, // 编码器 B 相异步输入，经两级触发器同步。
    input  wire              speed_tick, // 速度更新使能，顶层接 ADC 有效脉冲，默认约 1 kHz。
    output reg signed [31:0] position, // 合法 AB 相邻状态变化累计的位置，每次变化加 1 或减 1。
    output reg signed [31:0] velocity // 本次测速位置减上次测速位置，默认单位约为计数/ms。
);

    reg a_meta, a_sync; // A 相的第一级采样和第二级同步寄存器 a_sync。
    reg b_meta, b_sync; // B 相的第一级采样和第二级同步寄存器 b_sync。
    reg [1:0] previous_state; // 上一个系统时钟保存的同步 AB 状态，用于识别相邻变化。
    reg signed [31:0] position_last; // 上次 speed_tick 时锁存的位置，用于本次差分测速。

    wire [1:0] current_state = {b_sync, a_sync}; // 当前同步后的 AB 状态，位序为 B 在高位、A 在低位。

    // 每个系统时钟都解码位置，只有 speed_tick 到来时才更新速度。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_meta         <= 1'b0;
            a_sync         <= 1'b0;
            b_meta         <= 1'b0;
            b_sync         <= 1'b0;
            previous_state <= 2'b00;
            position       <= 32'sd0;
            position_last  <= 32'sd0;
            velocity       <= 32'sd0;
        end else begin
            // 两级同步降低异步编码器信号进入 FPGA 时的亚稳态风险。
            a_meta <= enc_a_i;
            a_sync <= a_meta;
            b_meta <= enc_b_i;
            b_sync <= b_meta;
            previous_state <= current_state;

            // 合法的相邻状态转移各计 +1 或 -1；静止和跨状态跳变均不累计。
            // 拼接顺序为 {上次 B、上次 A、当前 B、当前 A}，方向沿用此映射。
            case ({previous_state, current_state})
                4'b0001, 4'b0111, 4'b1110, 4'b1000:
                    position <= position + 1'b1;
                4'b0010, 4'b1011, 4'b1101, 4'b0100:
                    position <= position - 1'b1;
                default: position <= position;
            endcase

            // 默认 speed_tick 与 1 kHz ADC 采样同步，因此 velocity 单位约为计数/毫秒。
            // 非阻塞赋值读取更新前的 position，同拍发生的边沿计数会在后续测速中体现。
            if (speed_tick) begin
                velocity      <= position - position_last;
                position_last <= position;
            end
        end
    end

endmodule
