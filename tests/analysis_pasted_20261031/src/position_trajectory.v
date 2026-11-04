
`timescale 1ns / 1ps

// RUN4 正弦位置目标发生器：不修改角度 PID、位置 PD 或电机 PWM。
// 50 MHz 时钟，sample_tick 约 1 kHz，起点为启动瞬间编码器位置。
// 轨迹：position_ref = origin_position + A*sin(2*pi*n/PERIOD_SAMPLES)。
// 采用四分之一周期查表、线性插值和逐位移位累加。
// 本模块不使用运行时乘法器，减少宽组合乘法逻辑。
// 振幅参数范围为 0~32767，周期参数必须为正整数。

module position_trajectory #(
    parameter integer AMPLITUDE_COUNTS = 155,
    parameter integer PERIOD_SAMPLES   = 30000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire sample_tick,
    input  wire start,
    input  wire enable,
    input  wire signed [31:0] encoder_position,
    output reg signed [31:0] position_ref
);

    // 相位累加步长：2^32 / 周期采样点数。
    // PERIOD_SAMPLES 是常量，综合时计算，不生成运行时除法器。
    localparam [31:0] PHASE_STEP =
        (64'd4294967296 + PERIOD_SAMPLES/2)
        / PERIOD_SAMPLES;

    // 运算状态机。
    localparam [3:0] ST_IDLE        = 4'd0;
    localparam [3:0] ST_LOOKUP0     = 4'd1;
    localparam [3:0] ST_LOOKUP1     = 4'd2;
    localparam [3:0] ST_INTERP_INIT = 4'd3;
    localparam [3:0] ST_INTERP_RUN  = 4'd4;
    localparam [3:0] ST_INTERP_DONE = 4'd5;
    localparam [3:0] ST_SCALE_INIT  = 4'd6;
    localparam [3:0] ST_SCALE_RUN   = 4'd7;
    localparam [3:0] ST_SCALE_DONE  = 4'd8;
    localparam [3:0] ST_OUTPUT      = 4'd9;

    reg running;
    reg [3:0] state;
    reg [4:0] bit_count;

    reg [31:0] phase;
    reg [31:0] pending_phase;

    // 每次启动时记录当前位置，作为正弦轨迹中心。
    reg signed [31:0] origin_position;

    // 正弦查表及插值中间变量。
    reg signed [15:0] sin0;
    reg signed [15:0] sin1;
    reg signed [15:0] interp_sin;
    reg [15:0] fraction;

    // 插值乘法拆分为逐位移位累加。
    // 相邻正弦表值的差不超过约 804。
    reg signed [31:0] interp_acc;
    reg signed [31:0] interp_term;
    reg [15:0] frac_bits;

    // 振幅缩放也改为移位累加，不使用组合乘法器。
    reg signed [31:0] amp_acc;
    reg signed [31:0] amp_term;
    reg [15:0] amp_bits;

    // 最终相对启动位置的编码器目标计数。
    reg signed [31:0] relative_count;

    // 四分之一周期正弦表，Q15 格式。
    // 0 度为 0，90 度为 32767。
    function signed [15:0] quarter_sine;
        input [6:0] index;
        begin
            case (index)
                7'd0:  quarter_sine = 16'sd0;
                7'd1:  quarter_sine = 16'sd804;
                7'd2:  quarter_sine = 16'sd1608;
                7'd3:  quarter_sine = 16'sd2410;
                7'd4:  quarter_sine = 16'sd3212;
                7'd5:  quarter_sine = 16'sd4011;
                7'd6:  quarter_sine = 16'sd4808;
                7'd7:  quarter_sine = 16'sd5602;
                7'd8:  quarter_sine = 16'sd6393;
                7'd9:  quarter_sine = 16'sd7179;
                7'd10: quarter_sine = 16'sd7962;
                7'd11: quarter_sine = 16'sd8739;
                7'd12: quarter_sine = 16'sd9512;
                7'd13: quarter_sine = 16'sd10278;
                7'd14: quarter_sine = 16'sd11039;
                7'd15: quarter_sine = 16'sd11793;
                7'd16: quarter_sine = 16'sd12539;
                7'd17: quarter_sine = 16'sd13279;
                7'd18: quarter_sine = 16'sd14010;
                7'd19: quarter_sine = 16'sd14732;
                7'd20: quarter_sine = 16'sd15446;
                7'd21: quarter_sine = 16'sd16151;
                7'd22: quarter_sine = 16'sd16846;
                7'd23: quarter_sine = 16'sd17530;
                7'd24: quarter_sine = 16'sd18204;
                7'd25: quarter_sine = 16'sd18868;
                7'd26: quarter_sine = 16'sd19519;
                7'd27: quarter_sine = 16'sd20159;
                7'd28: quarter_sine = 16'sd20787;
                7'd29: quarter_sine = 16'sd21403;
                7'd30: quarter_sine = 16'sd22005;
                7'd31: quarter_sine = 16'sd22594;
                7'd32: quarter_sine = 16'sd23170;
                7'd33: quarter_sine = 16'sd23731;
                7'd34: quarter_sine = 16'sd24279;
                7'd35: quarter_sine = 16'sd24811;
                7'd36: quarter_sine = 16'sd25329;
                7'd37: quarter_sine = 16'sd25832;
                7'd38: quarter_sine = 16'sd26319;
                7'd39: quarter_sine = 16'sd26790;
                7'd40: quarter_sine = 16'sd27245;
                7'd41: quarter_sine = 16'sd27683;
                7'd42: quarter_sine = 16'sd28105;
                7'd43: quarter_sine = 16'sd28510;
                7'd44: quarter_sine = 16'sd28898;
                7'd45: quarter_sine = 16'sd29268;
                7'd46: quarter_sine = 16'sd29621;
                7'd47: quarter_sine = 16'sd29956;
                7'd48: quarter_sine = 16'sd30273;
                7'd49: quarter_sine = 16'sd30571;
                7'd50: quarter_sine = 16'sd30852;
                7'd51: quarter_sine = 16'sd31113;
                7'd52: quarter_sine = 16'sd31356;
                7'd53: quarter_sine = 16'sd31580;
                7'd54: quarter_sine = 16'sd31785;
                7'd55: quarter_sine = 16'sd31971;
                7'd56: quarter_sine = 16'sd32137;
                7'd57: quarter_sine = 16'sd32285;
                7'd58: quarter_sine = 16'sd32412;
                7'd59: quarter_sine = 16'sd32521;
                7'd60: quarter_sine = 16'sd32609;
                7'd61: quarter_sine = 16'sd32678;
                7'd62: quarter_sine = 16'sd32728;
                7'd63: quarter_sine = 16'sd32757;
                7'd64: quarter_sine = 16'sd32767;
                default: quarter_sine = 16'sd0;
            endcase
        end
    endfunction

    // 根据四象限对称关系，计算完整周期正弦值。
    function signed [15:0] sine_bin;
        input [7:0] index;
        reg [5:0] sub_index;
        begin
            sub_index = index[5:0];

            case (index[7:6])
                2'd0:
                    sine_bin =
                        quarter_sine({1'b0, sub_index});

                2'd1:
                    sine_bin =
                        quarter_sine(
                            7'd64 - {1'b0, sub_index}
                        );

                2'd2:
                    sine_bin =
                        -quarter_sine({1'b0, sub_index});

                default:
                    sine_bin =
                        -quarter_sine(
                            7'd64 - {1'b0, sub_index}
                        );
            endcase
        end
    endfunction

    // 两个相邻正弦值分两个时钟周期读取。
    // 共用一个查表逻辑，避免并行复制两个正弦查表电路。
    wire [7:0] lut_index =
        pending_phase[31:24] +
        ((state == ST_LOOKUP1) ? 8'd1 : 8'd0);

    wire signed [15:0] lut_value =
        sine_bin(lut_index);

    // 一个正弦采样的计算约需 40 个 50 MHz 时钟周期。
    // 1 kHz 采样间隔有约 50000 个时钟周期，计算时间足够。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running         <= 1'b0;
            state           <= ST_IDLE;
            bit_count       <= 5'd0;

            phase           <= 32'd0;
            pending_phase   <= 32'd0;
            origin_position <= 32'sd0;
            position_ref    <= 32'sd0;

            sin0            <= 16'sd0;
            sin1            <= 16'sd0;
            interp_sin      <= 16'sd0;
            fraction        <= 16'd0;

            interp_acc      <= 32'sd0;
            interp_term     <= 32'sd0;
            frac_bits       <= 16'd0;

            amp_acc         <= 32'sd0;
            amp_term        <= 32'sd0;
            amp_bits        <= 16'd0;

            relative_count  <= 32'sd0;

        end else if (start) begin
            // 启动时目标立即等于当前位置。
            // 避免 RUN4 刚启动时出现较大的位置目标阶跃。
            running         <= 1'b1;
            state           <= ST_IDLE;

            phase           <= 32'd0;
            pending_phase   <= 32'd0;

            origin_position <= encoder_position;
            position_ref    <= encoder_position;

        end else if (!enable) begin
            // 停止 RUN4 时停止相位更新。
            // 下次启动重新设置正弦中心。
            running <= 1'b0;
            state   <= ST_IDLE;

        end else if (running) begin
            case (state)

                ST_IDLE: begin
                    // 每收到一次 ADC 采样脉冲，
                    // 正弦相位前进一步。
                    if (sample_tick) begin
                        pending_phase <=
                            phase + PHASE_STEP;

                        state <= ST_LOOKUP0;
                    end
                end

                ST_LOOKUP0: begin
                    // 读取相位区间左端点正弦值。
                    sin0 <= lut_value;
                    fraction <= pending_phase[23:8];

                    state <= ST_LOOKUP1;
                end

                ST_LOOKUP1: begin
                    // 读取相位区间右端点正弦值。
                    sin1 <= lut_value;

                    state <= ST_INTERP_INIT;
                end

                ST_INTERP_INIT: begin
                    // 准备进行线性插值：
                    // sin = sin0 + (sin1-sin0)*fraction/65536。
                    interp_acc <= 32'sd0;

                    interp_term <=
                        {{16{sin1[15]}}, sin1} -
                        {{16{sin0[15]}}, sin0};

                    frac_bits <= fraction;
                    bit_count <= 5'd0;

                    state <= ST_INTERP_RUN;
                end

                ST_INTERP_RUN: begin
                    // 每个时钟处理 fraction 的一位。
                    // 如果该位为 1，就将当前移位项加到累加器。
                    if (frac_bits[0])
                        interp_acc <=
                            interp_acc + interp_term;

                    frac_bits <= frac_bits >> 1;
                    interp_term <= interp_term <<< 1;

                    if (bit_count == 5'd15)
                        state <= ST_INTERP_DONE;
                    else
                        bit_count <= bit_count + 1'b1;
                end

                ST_INTERP_DONE: begin
                    // 将 Q15 线性插值结果恢复为正弦表数值。
                    interp_sin <=
                        $signed(sin0) +
                        (interp_acc >>> 16);

                    state <= ST_SCALE_INIT;
                end

                ST_SCALE_INIT: begin
                    // 开始振幅缩放：
                    // position = sin(Q15)*AMPLITUDE_COUNTS/32768。
                    amp_acc <= 32'sd0;

                    amp_term <=
                        {{16{interp_sin[15]}}, interp_sin};

                    amp_bits <= AMPLITUDE_COUNTS[15:0];
                    bit_count <= 5'd0;

                    state <= ST_SCALE_RUN;
                end

                ST_SCALE_RUN: begin
                    // 每个时钟处理振幅的一位。
                    // 通过移位加法代替原来的乘法器。
                    if (amp_bits[0])
                        amp_acc <=
                            amp_acc + amp_term;

                    amp_bits <= amp_bits >> 1;
                    amp_term <= amp_term <<< 1;

                    if (bit_count == 5'd15)
                        state <= ST_SCALE_DONE;
                    else
                        bit_count <= bit_count + 1'b1;
                end

                ST_SCALE_DONE: begin
                    // Q15 转换为整数编码器计数。
                    // 正数和负数采用对称四舍五入。
                    if (amp_acc >= 0)
                        relative_count <=
                            (amp_acc + 32'sd16384) >>> 15;
                    else
                        relative_count <=
                            -(((-amp_acc) + 32'sd16384) >>> 15);

                    state <= ST_OUTPUT;
                end

                ST_OUTPUT: begin
                    // 更新相位，输出新的位置目标。
                    phase <= pending_phase;

                    position_ref <=
                        origin_position + relative_count;

                    state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end

            endcase
        end
    end

endmodule
