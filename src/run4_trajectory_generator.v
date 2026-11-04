`timescale 1ns / 1ps

// RUN4 三轨迹实时位置目标生成器。只生成编码器位置目标，不直接生成 PWM。
// 轨迹 0=梯形速度；1=S 曲线（五次平滑插值）；2=正弦往返。
// 原点：每次启动 RUN4 时记录的编码器位置。
// 幅度：原点 +/-100 计数。梯形/S 曲线依次 0->+100->0->-100->0；
//       主轨迹每段 683 个有效平衡采样（1 kHz 时约 0.683 s）；四段周期约 2.732 s。
// 正弦：保持原点 +/-100；相位交替推进1、2，平均每个有效采样1.5相位。
//       4096相位点/1.5约为2730.67次有效采样；1 kHz连续平衡时周期约2.731秒。
// 梯形/S曲线主轨迹的完整周期调整为约2.732秒，与可正常运行的正弦接近。
// 按键中途切换轨迹时：目标保持原值，先以五次曲线回到启动原点，
// 然后开始新轨迹。只保证目标位置连续；切换瞬间速度不能严格连续。
// 仅在 advance && adc_valid 时推进，计算拆成多拍；暂停/丢失平衡时冻结。

module run4_trajectory_generator (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               adc_valid,
    input  wire               start,              // RUN4 成功启动的单拍脉冲
    input  wire               running,            // 当前正在 RUN4
    input  wire               advance,            // 仅允许平衡、无锁止时推进
    input  wire [1:0]         selected_profile,   // 0梯形 1 S曲线 2正弦
    input  wire signed [31:0] encoder_position,
    output reg signed [31:0]  target_position
);

    localparam [1:0] PROF_TRAP = 2'd0, PROF_SCURVE = 2'd1, PROF_SINE = 2'd2;
    localparam signed [11:0] AMPLITUDE = 12'sd100;

    reg signed [31:0] origin;           // 本次启动中心，不随切换改变
    reg signed [31:0] leg_start;        // 当前单程起点
    reg signed [11:0] leg_delta;        // 当前单程位移，绝对值不超过 100

    wire signed [31:0] home_delta_full = origin - target_position; // 切换时回原点的位移

    reg [1:0] leg;                     // 0:正向 1:回中心 2:反向 3:回中心
    reg [11:0] phase;                  // 查表归一化相位 0..4095；梯形/S主轨迹每采样加6

    reg sine_step_toggle;              // 正弦交替加1、加2，平均1.5；0加1，1加2

    reg [1:0] applied_profile;         // 当前已经采用的轨迹类型
    reg transition_home;              // 切换时先使用 S 曲线回中心
    reg [2:0] state;

    // 每次采样先锁定位置计算输入，防止后续分拍读到不同的样本。
    reg [11:0] calc_phase;
    reg [1:0] calc_profile;
    reg signed [31:0] calc_base;
    reg signed [11:0] calc_delta;
    reg signed [17:0] lookup0, lookup1; // Q15 正弦可负；位置进度非负
    reg signed [22:0] interpolate_product;
    reg signed [17:0] profile_q15;
    reg signed [35:0] position_product;

    // 计算单程前后对称的 Q15 查表索引；镜像时避免运行期除法。
    wire mirror = calc_phase[11];

    wire [12:0] half_phase = mirror ?
            (13'd4096 - {1'b0,calc_phase}) : {1'b0,calc_phase};

    wire [6:0] ease_index = half_phase[11:5];

    wire [6:0] lookup_ease_index =
        ((state == 3'd2) && (ease_index < 7'd64)) ?
        ease_index + 7'd1 : ease_index;

    wire [7:0] lookup_sine_index = calc_phase[11:4] +
        ((state == 3'd2) ? 8'd1 : 8'd0);

    wire signed [15:0] sine_value = sine_bin(lookup_sine_index);

    wire signed [17:0] table_value =
        (calc_profile == PROF_SINE) ?
            {{2{sine_value[15]}},sine_value} :
        (calc_profile == PROF_TRAP) ?
            $signed({2'b00,trapezoid_half(lookup_ease_index)}) :
            $signed({2'b00,scurve_half(lookup_ease_index)});

    wire signed [17:0] lookup_delta = lookup1 - lookup0;

    // 正弦细分 16 点，其余曲线细分 32 点。
    wire [4:0] fraction = (calc_profile == PROF_SINE) ?
        {1'b0,calc_phase[3:0]} : half_phase[4:0];

    wire signed [22:0] interpolate_value =
        $signed(lookup_delta) * $signed({1'b0,fraction});

    // 先以23位完成有符号累加，再显式截取18位Q15结果，消除隐式截断告警。
    // 轨迹查表的正常范围在18位有符号区间内；截取结果与原代码一致。
    // 关键修复：正弦左右半波必须使用相同的“向零截断”规则。
    // 旧版对负差值直接 >>>4，会向负无穷取整；同一正弦幅值在
    // 正负半周期可能相差1个编码器计数（4096点中曾有12个点）。
    // 以下只改变正弦插值的负数取整，梯形/S曲线仍使用原先 >>>5。
    wire signed [22:0] sine_interp_step = (interpolate_product < 0) ?
        -((-interpolate_product) >>> 4) :
          (interpolate_product  >>> 4);

    wire signed [22:0] interpolated_full =
        $signed({{5{lookup0[17]}}, lookup0}) +
        ((calc_profile == PROF_SINE) ?
            sine_interp_step : (interpolate_product >>> 5));

    wire signed [17:0] interpolated = interpolated_full[17:0];

    wire signed [35:0] rounded_offset = (position_product >= 0) ?
        ((position_product + 36'sd16384) >>> 15) :
        -(((-position_product) + 36'sd16384) >>> 15);

    // 原始正弦插值函数，取自用户上传的统一顶层，表值未调整。
    function signed [15:0] quarter_sine;
        input [6:0] index;
        begin
            case (index)
                7'd0: quarter_sine = 16'sd0;
                7'd1: quarter_sine = 16'sd804;
                7'd2: quarter_sine = 16'sd1608;
                7'd3: quarter_sine = 16'sd2410;
                7'd4: quarter_sine = 16'sd3212;
                7'd5: quarter_sine = 16'sd4011;
                7'd6: quarter_sine = 16'sd4808;
                7'd7: quarter_sine = 16'sd5602;
                7'd8: quarter_sine = 16'sd6393;
                7'd9: quarter_sine = 16'sd7179;
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

    // 利用四个象限的对称关系得到 256 分段的正弦值。
    function signed [15:0] sine_bin;
        input [7:0] index;
        reg [5:0] sub_index;
        begin
            sub_index = index[5:0];
            case (index[7:6])
                2'd0:
                    sine_bin = quarter_sine({1'b0, sub_index});
                2'd1:
                    sine_bin = quarter_sine(7'd64 - {1'b0, sub_index});
                2'd2:
                    sine_bin = -quarter_sine({1'b0, sub_index});
                default:
                    sine_bin = -quarter_sine(7'd64 - {1'b0, sub_index});
            endcase
        end
    endfunction

    // 半程查表：索引 0~64 对应单程的 0~50% 时间；后半程利用 f(1-t)=1-f(t)。
    function [15:0] trapezoid_half;
        input [6:0] index;
        begin
            case (index)
                7'd0: trapezoid_half = 16'd0;
                7'd1: trapezoid_half = 16'd5;
                7'd2: trapezoid_half = 16'd21;
                7'd3: trapezoid_half = 16'd48;
                7'd4: trapezoid_half = 16'd85;
                7'd5: trapezoid_half = 16'd133;
                7'd6: trapezoid_half = 16'd192;
                7'd7: trapezoid_half = 16'd261;
                7'd8: trapezoid_half = 16'd341;
                7'd9: trapezoid_half = 16'd432;
                7'd10: trapezoid_half = 16'd533;
                7'd11: trapezoid_half = 16'd645;
                7'd12: trapezoid_half = 16'd768;
                7'd13: trapezoid_half = 16'd901;
                7'd14: trapezoid_half = 16'd1045;
                7'd15: trapezoid_half = 16'd1200;
                7'd16: trapezoid_half = 16'd1365;
                7'd17: trapezoid_half = 16'd1541;
                7'd18: trapezoid_half = 16'd1728;
                7'd19: trapezoid_half = 16'd1925;
                7'd20: trapezoid_half = 16'd2133;
                7'd21: trapezoid_half = 16'd2352;
                7'd22: trapezoid_half = 16'd2581;
                7'd23: trapezoid_half = 16'd2821;
                7'd24: trapezoid_half = 16'd3072;
                7'd25: trapezoid_half = 16'd3333;
                7'd26: trapezoid_half = 16'd3605;
                7'd27: trapezoid_half = 16'd3888;
                7'd28: trapezoid_half = 16'd4181;
                7'd29: trapezoid_half = 16'd4485;
                7'd30: trapezoid_half = 16'd4800;
                7'd31: trapezoid_half = 16'd5125;
                7'd32: trapezoid_half = 16'd5461;
                7'd33: trapezoid_half = 16'd5803;
                7'd34: trapezoid_half = 16'd6144;
                7'd35: trapezoid_half = 16'd6485;
                7'd36: trapezoid_half = 16'd6827;
                7'd37: trapezoid_half = 16'd7168;
                7'd38: trapezoid_half = 16'd7509;
                7'd39: trapezoid_half = 16'd7851;
                7'd40: trapezoid_half = 16'd8192;
                7'd41: trapezoid_half = 16'd8533;
                7'd42: trapezoid_half = 16'd8875;
                7'd43: trapezoid_half = 16'd9216;
                7'd44: trapezoid_half = 16'd9557;
                7'd45: trapezoid_half = 16'd9899;
                7'd46: trapezoid_half = 16'd10240;
                7'd47: trapezoid_half = 16'd10581;
                7'd48: trapezoid_half = 16'd10923;
                7'd49: trapezoid_half = 16'd11264;
                7'd50: trapezoid_half = 16'd11605;
                7'd51: trapezoid_half = 16'd11947;
                7'd52: trapezoid_half = 16'd12288;
                7'd53: trapezoid_half = 16'd12629;
                7'd54: trapezoid_half = 16'd12971;
                7'd55: trapezoid_half = 16'd13312;
                7'd56: trapezoid_half = 16'd13653;
                7'd57: trapezoid_half = 16'd13995;
                7'd58: trapezoid_half = 16'd14336;
                7'd59: trapezoid_half = 16'd14677;
                7'd60: trapezoid_half = 16'd15019;
                7'd61: trapezoid_half = 16'd15360;
                7'd62: trapezoid_half = 16'd15701;
                7'd63: trapezoid_half = 16'd16043;
                7'd64: trapezoid_half = 16'd16384;
                default: trapezoid_half = 16'd16384;
            endcase
        end
    endfunction

    // 半程查表：索引 0~64 对应单程的 0~50% 时间；后半程利用 f(1-t)=1-f(t)。
    function [15:0] scurve_half;
        input [6:0] index;
        begin
            case (index)
                7'd0: scurve_half = 16'd0;
                7'd1: scurve_half = 16'd0;
                7'd2: scurve_half = 16'd1;
                7'd3: scurve_half = 16'd4;
                7'd4: scurve_half = 16'd10;
                7'd5: scurve_half = 16'd18;
                7'd6: scurve_half = 16'd31;
                7'd7: scurve_half = 16'd49;
                7'd8: scurve_half = 16'd73;
                7'd9: scurve_half = 16'd102;
                7'd10: scurve_half = 16'd139;
                7'd11: scurve_half = 16'd182;
                7'd12: scurve_half = 16'd233;
                7'd13: scurve_half = 16'd293;
                7'd14: scurve_half = 16'd361;
                7'd15: scurve_half = 16'd439;
                7'd16: scurve_half = 16'd526;
                7'd17: scurve_half = 16'd623;
                7'd18: scurve_half = 16'd730;
                7'd19: scurve_half = 16'd847;
                7'd20: scurve_half = 16'd975;
                7'd21: scurve_half = 16'd1114;
                7'd22: scurve_half = 16'd1264;
                7'd23: scurve_half = 16'd1426;
                7'd24: scurve_half = 16'd1598;
                7'd25: scurve_half = 16'd1782;
                7'd26: scurve_half = 16'd1977;
                7'd27: scurve_half = 16'd2184;
                7'd28: scurve_half = 16'd2403;
                7'd29: scurve_half = 16'd2633;
                7'd30: scurve_half = 16'd2875;
                7'd31: scurve_half = 16'd3128;
                7'd32: scurve_half = 16'd3392;
                7'd33: scurve_half = 16'd3668;
                7'd34: scurve_half = 16'd3954;
                7'd35: scurve_half = 16'd4252;
                7'd36: scurve_half = 16'd4561;
                7'd37: scurve_half = 16'd4880;
                7'd38: scurve_half = 16'd5209;
                7'd39: scurve_half = 16'd5549;
                7'd40: scurve_half = 16'd5898;
                7'd41: scurve_half = 16'd6258;
                7'd42: scurve_half = 16'd6626;
                7'd43: scurve_half = 16'd7004;
                7'd44: scurve_half = 16'd7391;
                7'd45: scurve_half = 16'd7786;
                7'd46: scurve_half = 16'd8189;
                7'd47: scurve_half = 16'd8600;
                7'd48: scurve_half = 16'd9018;
                7'd49: scurve_half = 16'd9443;
                7'd50: scurve_half = 16'd9875;
                7'd51: scurve_half = 16'd10314;
                7'd52: scurve_half = 16'd10758;
                7'd53: scurve_half = 16'd11207;
                7'd54: scurve_half = 16'd11662;
                7'd55: scurve_half = 16'd12121;
                7'd56: scurve_half = 16'd12584;
                7'd57: scurve_half = 16'd13051;
                7'd58: scurve_half = 16'd13521;
                7'd59: scurve_half = 16'd13994;
                7'd60: scurve_half = 16'd14469;
                7'd61: scurve_half = 16'd14946;
                7'd62: scurve_half = 16'd15425;
                7'd63: scurve_half = 16'd15904;
                7'd64: scurve_half = 16'd16384;
                default: scurve_half = 16'd16384;
            endcase
        end
    endfunction

    // 轨迹控制状态机。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_position  <= 32'sd0;
            origin           <= 32'sd0;
            leg_start        <= 32'sd0;
            leg_delta        <= AMPLITUDE;
            leg              <= 2'd0;
            phase            <= 12'd0;
            sine_step_toggle <= 1'b0;
            applied_profile  <= PROF_TRAP;
            transition_home  <= 1'b0;
            state            <= 3'd0;
            calc_phase       <= 12'd0;
            calc_profile     <= PROF_TRAP;
            calc_base        <= 32'sd0;
            calc_delta       <= 12'sd0;
            lookup0          <= 18'sd0;
            lookup1          <= 18'sd0;
            interpolate_product <= 23'sd0;
            profile_q15      <= 18'sd0;
            position_product <= 36'sd0;

        end else if (start) begin
            // 从当前位置开始，不会因启动轨迹改变 RUN4 位置目标。
            target_position <= encoder_position;
            origin          <= encoder_position;
            leg_start       <= encoder_position;
            leg_delta       <= AMPLITUDE;
            leg             <= 2'd0;
            phase           <= 12'd0;
            sine_step_toggle <= 1'b0;
            applied_profile <= selected_profile;
            transition_home <= 1'b0;
            state           <= 3'd0;

        end else if (!running) begin
            state           <= 3'd0;
            applied_profile <= selected_profile;

        end else if (selected_profile != applied_profile) begin
            // 改选轨迹时绝不改写本拍的位置参考：先从当前参考平滑回中心。
            // 假设正常运行轨迹始终保持原点 +/-100，回中距离最多100。
            applied_profile <= selected_profile;
            leg_start       <= target_position;
            leg_delta       <= home_delta_full[11:0]; // 正常轨迹位移在+/-100内，显式截取12位
            leg             <= 2'd0;
            phase           <= 12'd0;
            sine_step_toggle <= 1'b0; // 切换后从+1、+2的固定顺序重新计数
            transition_home <= (target_position != origin);
            state           <= 3'd0;

            if (target_position == origin)
                leg_delta <= AMPLITUDE;

        end else if (!advance) begin
            // 已启动但未捕获直立、故障、或退出平衡时停止推进参考。
            state <= 3'd0;

        end else begin
            case (state)

                3'd0: if (adc_valid) begin
                    calc_phase   <= phase;
                    calc_profile <= transition_home ? PROF_SCURVE : applied_profile;
                    calc_base    <=
                        (applied_profile == PROF_SINE && !transition_home) ?
                        origin : leg_start;
                    calc_delta   <=
                        (applied_profile == PROF_SINE && !transition_home) ?
                        AMPLITUDE : leg_delta;
                    state        <= 3'd1;
                end

                3'd1: begin
                    lookup0 <= table_value;
                    state <= 3'd2;
                end

                3'd2: begin
                    lookup1 <= table_value;
                    state <= 3'd3;
                end

                3'd3: begin
                    interpolate_product <= interpolate_value;
                    state <= 3'd4;
                end

                3'd4: begin
                    // 对单程曲线的后半段使用 f(1-s)=1-f(s) 镜像。
                    if ((calc_profile != PROF_SINE) && mirror)
                        profile_q15 <= 18'sd32768 - interpolated;
                    else
                        profile_q15 <= interpolated;

                    state <= 3'd5;
                end

                3'd5: begin
                    // 单拍仅做18x12位乘法，不能让ROM/插值/限幅串成一个周期。
                    position_product <= profile_q15 * calc_delta;
                    state <= 3'd6;
                end

                3'd6: begin
                    // 先计算准确的单程终点，避免固定点舍入造成的终点漂移。
                    if ((phase >= 12'd4092) &&
                        (transition_home || applied_profile != PROF_SINE)) begin

                        target_position <= leg_start + leg_delta;
                        phase <= 12'd0;

                        if (transition_home) begin
                            transition_home <= 1'b0;
                            leg_start <= origin;
                            leg_delta <= AMPLITUDE;
                            leg <= 2'd0;
                        end else begin
                            case (leg)

                                2'd0: begin
                                    leg_start <= origin + 32'sd100;
                                    leg_delta <= -AMPLITUDE;
                                    leg <= 2'd1;
                                end

                                2'd1: begin
                                    leg_start <= origin;
                                    leg_delta <= -AMPLITUDE;
                                    leg <= 2'd2;
                                end

                                2'd2: begin
                                    leg_start <= origin - 32'sd100;
                                    leg_delta <= AMPLITUDE;
                                    leg <= 2'd3;
                                end

                                default: begin
                                    leg_start <= origin;
                                    leg_delta <= AMPLITUDE;
                                    leg <= 2'd0;
                                end
                            endcase
                        end

                    end else begin
                        target_position <= calc_base + rounded_offset[31:0];

                        // 正弦每个有效平衡采样交替+1、+2，平均步进1.5；
                        // 梯形和S曲线主轨迹固定+6；切换时的S曲线回中仍+4。
                        // 只在真正提交一次新目标时更新交替标志，
                        // 暂停、丢失平衡不会打乱相位步进顺序。
                        if (calc_profile == PROF_SINE) begin
                            phase <= phase +
                                (sine_step_toggle ? 12'd2 : 12'd1);

                            sine_step_toggle <= ~sine_step_toggle;
                        end else begin
                            // 梯形/S曲线主轨迹与已正常运行的正弦轨迹匹配周期。
                            // 四段各683次有效采样，完整周期约2.732s（1kHz）。
                            // 切换波形时的S曲线回原点仍沿用旧速率，避免影响正弦切换。
                            phase <= phase + (transition_home ? 12'd4 : 12'd6);
                        end
                    end

                    state <= 3'd0;
                end

                default: state <= 3'd0;
            endcase
        end
    end

endmodule