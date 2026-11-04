
// RUN3 12拍分拍控制器：参考RUN4流水组织，不修改RUN3顶层实参。
// ADC 角度计数和起摆能量阈值需按实机标定；调试时先使用较小输出限幅。
// 下列为模块默认参数；当前工程的实际参数以顶层 controller0 实例覆盖值为准。
module pendulum_controller #(
    // 角度误差进入此范围且摆动较慢时，允许切入平衡模式（单位：ADC 计数）。
    parameter integer ANGLE_CAPTURE       = 150, // 进入直立捕获区的误差幅值阈值，单位为 ADC 计数。
    // 偏离超过此范围时，从平衡模式退回起摆/恢复控制。
    parameter integer ANGLE_RELEASE       = 260, // 退出平衡模式的误差幅值阈值，与捕获阈值形成滞回。
    // 捕获条件中的误差变化阈值，rate_now 是相邻采样的误差差值。
    parameter integer CAPTURE_RATE        = 80, // 允许捕获的相邻采样误差变化幅值上限，未换算为角速度。
    // 平衡内环参数：KP 增强角度修正，KD 提供阻尼。
    parameter integer KP_ANGLE            = 6, // 平衡角度比例增益，实际工程值由顶层实例覆盖。
    parameter integer KP_ANGLE_NUM        = 0, // 比例项定点分子；为 0 时使用整数 KP_ANGLE 兼容旧实例。
    parameter integer KP_ANGLE_SHIFT      = 0, // 比例项定点右移位数，例如 15/2 表示 7.5。
    parameter integer KD_ANGLE            = 8, // 平衡差分增益，实际工程值由顶层实例覆盖。
    parameter integer KI_ANGLE            = 0, // 平衡角度积分增益，默认关闭，顶层按量纲显式开启。
    parameter integer KI_ANGLE_SHIFT      = 12, // 积分输出右移位数，值越大作用越弱。
    parameter integer ANGLE_I_UPDATE_SAMPLES = 5, // 积分更新周期；5 个 1 ms 样本对应参考程序的 5 ms。
    parameter integer ANGLE_I_LIMIT       = 64, // 积分项单独限幅，单位为 PWM 命令计数。
    parameter integer ANGLE_I_ACCUM_LIMIT  = (KI_ANGLE > 0) ?
        ((ANGLE_I_LIMIT * (1 << KI_ANGLE_SHIFT) + KI_ANGLE - 1) / KI_ANGLE) :
        262144, // 按 Ki 和积分输出限幅换算累加上限，避免隐藏的积分累积。
    // 平衡微分的采样窗口；1 保留原算法，8 表示比较当前与 8 个采样前的 ADC。
    // 该窗口不归一化：采样周期 1 ms 时，匀速下窗口 8 的 D 是窗口 1 的 8 倍。
    parameter integer BALANCE_D_WINDOW    = 1, // 平衡差分的历史长度；大于 1 时使用 ADC 多采样差分。
    // 旧的直接 PWM 位置反馈参数，仅在 POSITION_CASCADE_ENABLE=0 时使用。
    parameter integer KP_POSITION         = 1, // 目标位置减实际位置的反馈增益，顶层当前设为 0。
    parameter integer KD_POSITION         = 2, // 编码器速度反馈增益，顶层当前设为 0。
    // 位置外环可作为角度目标偏移，或以旧方式直接叠加 PWM 命令。
    parameter integer WEAK_POSITION_ENABLE = 0, // 位置外环总开关。
    parameter integer POSITION_UPDATE_SAMPLES = 32, // 位置环更新周期对应的采样数。
    parameter integer POSITION_KP_NUM     = 1, // 位置误差固定点分子，使用当前位置减目标位置。
    parameter integer POSITION_KP_SHIFT   = 5, // 位置比例项右移位数。
    parameter integer POSITION_KD_NUM     = 1, // 窗口位移反馈固定点分子。
    parameter integer POSITION_KD_SHIFT   = 0, // 窗口位移反馈右移位数。
    parameter integer POSITION_FEEDBACK_SIGN = 1, // 位置输出符号，用于匹配编码器和电机方向。
    parameter integer POSITION_OUTPUT_LIMIT = 64, // 位置输出限幅；串级模式下单位为 ADC 角度计数。
    parameter integer POSITION_SLEW_LIMIT = 4, // 每个位置环窗口的输出变化上限。
    parameter integer POSITION_ANGLE_LIMIT = 80, // 超过此角度误差时清除位置环状态。
    parameter integer POSITION_CASCADE_ENABLE = 0, // 1：位置输出作为角度目标偏移；0：兼容旧 PWM 叠加。
    // 起摆输出幅值及速度项增益。
    parameter integer SWING_GAIN          = 420, // 起摆输出的固定幅值，正负由速度和能量判断决定。
    parameter integer SWING_SPEED_GAIN    = 4, // 起摆能量估算中，对相邻角度差分使用的缩放系数。
    // 起摆能量估算目标和角度计数分段阈值，均使用算法内部量纲。
    parameter integer SWING_TARGET_ENERGY = 2_500_000, // 起摆能量估算的目标值，使用内部量纲。
    parameter integer COS_ZERO_CROSS      = 1024, // 用于起摆符号分段判断的角度计数阈值。
    // 电机命令对称限幅；默认 900 在 PWM 周期 2500 时对应 36% 驱动占空比。
    parameter integer OUTPUT_LIMIT        = 900, // 两种控制分支共用的最终输出幅值限制。
    // 平衡输出的正反向摩擦补偿；设为 0 可关闭，单位为 PWM 计数。
    parameter integer FRICTION_FWD        = 0, // 平衡分支的正向最大摩擦补偿，设为 0 可关闭。
    parameter integer FRICTION_REV        = 0, // 反向平衡输出的最大摩擦补偿，设为 0 可关闭。
    // 原始命令在 +/-2^SHIFT 内线性增加补偿，避免零点处突跳。
    parameter integer FRICTION_RAMP_SHIFT = 7, // 补偿渐增范围为 2 的此参数次方，默认 128 计数。
    parameter integer ADC_GLITCH_THRESHOLD = 512 // 相邻 1 kHz ADC 差分超过此值计为毛刺。
) (
    input  wire               clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire               rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire               sample_tick, // 采样更新使能脉冲，顶层连接 ADC 完成事件，默认约每 1 ms 一次。
    input  wire               enable, // 模块运行使能，具体输出更新时刻由下方时序逻辑决定。
    input  wire               position_loop_enable, // 自动起摆进入BAL后使能；位置PD每50ms更新。
    input  wire               angle_history_track, // 自动SWING期间继续更新D历史；手扶直立保持原策略。
    input  wire [11:0]        adc_value, // 完整 12 位 ADC 采样计数，范围 0~4095。
    input  wire [11:0]        angle_zero, // 直立角度的 ADC 参考值，由顶层校零逻辑提供。
    input  wire signed [31:0] encoder_position, // 编码器累计位置，带符号计数，未换算为长度或角度。
    input  wire signed [31:0] position_target, // 目标编码器位置，由顶层的启动和目标按键逻辑维护。
    input  wire signed [31:0] encoder_velocity, // 编码器采样间隔内的位置增量，带符号计数。
    output reg signed [15:0]  command, // 有符号输出命令，符号决定方向，幅值使用 PWM 计数单位。
    output reg signed [15:0]  angle_error, // 最近采样的角度误差：ADC 当前值减直立零点。
    output reg [1:0]          control_mode, // 内部状态编号：0 停止、1 起摆、2 直立平衡。
    output reg signed [31:0]  debug_angle_pd, // 诊断：本采样角度内环 P-D，未含积分和摩擦。
    output reg signed [31:0]  debug_position_applied, // 诊断：位置环角度参考偏移，或兼容模式下的PWM修正。
    output reg signed [15:0]  debug_balance_rate, // 诊断：本采样使用的多采样 ADC 差分。
    output reg signed [31:0]  debug_raw_control, // 诊断：当前分支摩擦补偿前的合成命令。
    output reg [7:0]          debug_flags, // 位0使能、1位置环配置、2平衡分支、3位置输出有效、4总限幅、5前拍使能。
    output wire [7:0]         debug_window_count, // 诊断：更新后的外环窗口计数低八位，当前范围为0~31。
    output reg [31:0]         debug_adc_glitch_count, // 片内 ADC 突变计数，饱和保持。
    output reg [31:0]         debug_adc_max_delta, // 片内观测到的最大相邻 ADC 差分。
    output reg [31:0]         debug_limit_events // 原始控制量超过最终输出限幅的次数，饱和保持。
);

    // 控制器内部状态：SAFE=停止，SWING=起摆，BAL=直立平衡。
    localparam [1:0] MODE_SAFE  = 2'd0;
    localparam [1:0] MODE_SWING = 2'd1;
    localparam [1:0] MODE_BAL   = 2'd2;

    // ------------------------------------------------------------------
    // RUN3时序优化：原版在一个 50 MHz 时钟周期内串接多个 64 位乘加、
    // 限幅、摩擦补偿及状态判断。这里保留控制公式及参数，改为 12 拍流水状态机。
    // ADC 周期约 50000 个主时钟，内部计算占 12 个主时钟（约 240 ns）。
    // 注意：顶层 UART/输出校验的 diagnostic_tick 必须在控制器提交后到达。
    // 已有顶层的guard_sample_tick位于ADC后12拍，不需要改变。
    // 所有调试寄存器与 command 在 S_COMMIT 同拍更新，避免跨采样混读。
    // ------------------------------------------------------------------
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_POS_DIFF   = 4'd1,
        S_POS_MUL    = 4'd2,
        S_POS_REQ    = 4'd3,
        S_POS_APPLY  = 4'd4,
        S_ANGLE      = 4'd5,
        S_I_CAND     = 4'd6,
        S_I_SCALE    = 4'd7,
        S_RAW        = 4'd8,
        S_FRIC_PRE   = 4'd9,
        S_FRIC_APPLY = 4'd10,
        S_COMMIT     = 4'd11;

    reg [3:0] state;

    reg [11:0] adc_last;
    reg adc_sample_seen;
    reg enable_last;
    reg [11:0] angle_history [0:BALANCE_D_WINDOW-1];
    integer history_index;

    reg [31:0] position_sample_count;
    reg signed [31:0] position_window_start;
    reg signed [63:0] position_feedback;
    reg signed [63:0] angle_integral;
    reg [31:0] angle_i_sample_count;

    // 每个 ADC 有效沿锁存完整输入；后续流水计算不会读取变化中的 ADC。
    reg sampled_enable, sampled_prev_enable;
    reg sampled_position_loop_enable; // 与本次ADC同拍锁存，避免流水计算期间使能变化。
    reg [1:0] sampled_mode, sampled_next_mode;
    reg signed [31:0] sampled_error, sampled_rate, sampled_d_rate;
    reg signed [31:0] sampled_position, sampled_target, sampled_velocity;
    reg signed [63:0] sampled_position_feedback;
    reg sampled_position_valid;

    reg signed [63:0] position_error_64, position_window_delta_64;
    reg signed [31:0] legacy_position_error;
    reg signed [63:0] position_p_product, position_d_product, position_requested;
    reg signed [31:0] cascade_error;
    reg signed [31:0] angle_p_term, angle_d_term, legacy_p_term, legacy_d_term;
    reg signed [31:0] angle_pd_result, base_control;
    reg signed [63:0] integral_candidate;
    reg signed [31:0] i_candidate_value, i_previous_value;
    reg integral_enabled;
    reg signed [31:0] raw_control, corrected_control;
    reg signed [31:0] friction_magnitude;
    reg signed [31:0] swing_speed_term;
    reg signed [63:0] swing_error_square, swing_speed_square, swing_energy;

    // SV01A103 有效行程约 333.3 度，ADC 端点不是连续的环形角度。
    // 保留线性差分，避免错误地把 0/4095 当作相邻角度。
    function signed [31:0] adc_linear_delta;
        input [11:0] current_adc;
        input [11:0] previous_adc;
        begin
            adc_linear_delta = $signed({1'b0, current_adc}) -
                                $signed({1'b0, previous_adc});
        end
    endfunction

    // 与原版相同：正、负定点结果均向零截断，而不是负数向下取整。
    function signed [63:0] symmetric_scale;
        input signed [63:0] value;
        input integer shift;
        begin
            if (value < 0)
                symmetric_scale = -((-value) >>> shift);
            else
                symmetric_scale = value >>> shift;
        end
    endfunction

    // 32 位限幅只作用于整数积分输出，外层 ANGLE_I_LIMIT 仍由顶层指定。
    function signed [31:0] limit_angle_i;
        input signed [63:0] value;
        begin
            if (value > ANGLE_I_LIMIT)
                limit_angle_i = ANGLE_I_LIMIT;
            else if (value < -ANGLE_I_LIMIT)
                limit_angle_i = -ANGLE_I_LIMIT;
            else
                limit_angle_i = value[31:0];
        end
    endfunction

    wire signed [31:0] wire_error = adc_linear_delta(adc_value, angle_zero);
    wire signed [31:0] wire_rate  = adc_linear_delta(adc_value, adc_last);
    wire signed [31:0] abs_wire_error = (wire_error < 0) ? -wire_error : wire_error;
    wire signed [31:0] abs_wire_rate  = (wire_rate  < 0) ? -wire_rate  : wire_rate;

    wire signed [31:0] sampled_abs_error =
        (sampled_error < 0) ? -sampled_error : sampled_error;
    wire signed [31:0] sampled_abs_rate =
        (sampled_rate < 0) ? -sampled_rate : sampled_rate;

    wire capture_now = (abs_wire_error <= ANGLE_CAPTURE) &&
                        (abs_wire_rate <= CAPTURE_RATE);

    // 状态选择提前在采样端算好，启动第一拍不会继续读取旧 SAFE 状态起摆。
    wire [1:0] next_mode_wire =
        !enable ? MODE_SAFE :
        !enable_last ? (capture_now ? MODE_BAL : MODE_SWING) :
        ((control_mode == MODE_BAL) && (abs_wire_error > ANGLE_RELEASE)) ? MODE_SWING :
        ((control_mode == MODE_SWING) && capture_now) ? MODE_BAL : control_mode;

    wire sampled_continuous = sampled_enable && sampled_prev_enable &&
                               (sampled_mode == sampled_next_mode);
    wire sampled_balancing = sampled_continuous && (sampled_mode == MODE_BAL);
    wire sampled_swinging = sampled_continuous && (sampled_mode == MODE_SWING);

    wire signed [63:0] position_request_limited =
        (position_requested > POSITION_OUTPUT_LIMIT) ? POSITION_OUTPUT_LIMIT :
        (position_requested < -POSITION_OUTPUT_LIMIT) ? -POSITION_OUTPUT_LIMIT :
        position_requested;

    wire signed [63:0] position_slew_up = position_feedback + POSITION_SLEW_LIMIT;
    wire signed [63:0] position_slew_down = position_feedback - POSITION_SLEW_LIMIT;

    // 串级模式下原控制器使用旧位置反馈更新本次角度 PID。
    // 同一窗口刚更新的新位置值，只会从下一 ADC 采样起生效。
    wire signed [31:0] legacy_extra =
        (WEAK_POSITION_ENABLE != 0 && POSITION_CASCADE_ENABLE == 0 &&
         sampled_position_loop_enable && sampled_position_valid) ?
        sampled_position_feedback[31:0] : 32'sd0;

    wire signed [31:0] complete_base = base_control + legacy_extra;
    wire anti_windup =
        ((complete_base + i_candidate_value > OUTPUT_LIMIT) && (cascade_error > 0)) ||
        ((complete_base + i_candidate_value < -OUTPUT_LIMIT) && (cascade_error < 0));

    wire signed [31:0] chosen_i = anti_windup ? i_previous_value : i_candidate_value;
    wire signed [31:0] abs_raw = (raw_control < 0) ? -raw_control : raw_control;

    wire final_limited =
        (corrected_control > OUTPUT_LIMIT) || (corrected_control < -OUTPUT_LIMIT);

    assign debug_window_count = position_sample_count[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            adc_last <= 12'd0;
            adc_sample_seen <= 1'b0;
            enable_last <= 1'b0;
            command <= 16'sd0;
            angle_error <= 16'sd0;
            control_mode <= MODE_SAFE;
            debug_angle_pd <= 32'sd0;
            debug_position_applied <= 32'sd0;
            debug_balance_rate <= 16'sd0;
            debug_raw_control <= 32'sd0;
            debug_flags <= 8'd0;
            debug_adc_glitch_count <= 32'd0;
            debug_adc_max_delta <= 32'd0;
            debug_limit_events <= 32'd0;
            position_sample_count <= 32'd0;
            position_window_start <= 32'sd0;
            position_feedback <= 64'sd0;
            angle_integral <= 64'sd0;
            angle_i_sample_count <= 32'd0;
            sampled_enable <= 1'b0;
            sampled_position_loop_enable <= 1'b0;
            sampled_prev_enable <= 1'b0;
            sampled_mode <= MODE_SAFE;
            sampled_next_mode <= MODE_SAFE;
            sampled_error <= 32'sd0;
            sampled_rate <= 32'sd0;
            sampled_d_rate <= 32'sd0;
            sampled_position <= 32'sd0;
            sampled_target <= 32'sd0;
            sampled_velocity <= 32'sd0;
            sampled_position_feedback <= 64'sd0;
            sampled_position_valid <= 1'b0;
            position_error_64 <= 64'sd0;
            position_window_delta_64 <= 64'sd0;
            legacy_position_error <= 32'sd0;
            position_p_product <= 64'sd0;
            position_d_product <= 64'sd0;
            position_requested <= 64'sd0;
            cascade_error <= 32'sd0;
            angle_p_term <= 32'sd0;
            angle_d_term <= 32'sd0;
            legacy_p_term <= 32'sd0;
            legacy_d_term <= 32'sd0;
            angle_pd_result <= 32'sd0;
            base_control <= 32'sd0;
            integral_candidate <= 64'sd0;
            i_candidate_value <= 32'sd0;
            i_previous_value <= 32'sd0;
            integral_enabled <= 1'b0;
            raw_control <= 32'sd0;
            corrected_control <= 32'sd0;
            friction_magnitude <= 32'sd0;
            swing_speed_term <= 32'sd0;
            swing_error_square <= 64'sd0;
            swing_speed_square <= 64'sd0;
            swing_energy <= 64'sd0;
            for (history_index = 0; history_index < BALANCE_D_WINDOW;
                 history_index = history_index + 1)
                angle_history[history_index] <= 12'd0;
        end else begin
            case (state)
                S_IDLE: begin
                     if (sample_tick) begin
                         // 第 0 拍：统一采集本次控制样本。
                         sampled_enable <= enable;
                         sampled_position_loop_enable <= position_loop_enable;
                         sampled_prev_enable <= enable_last;
                         sampled_mode <= control_mode;
                         sampled_next_mode <= next_mode_wire;
                         sampled_error <= wire_error;
                         sampled_rate <= wire_rate;
                         sampled_position <= encoder_position;
                         sampled_target <= position_target;
                         sampled_velocity <= encoder_velocity;
                         sampled_position_feedback <= position_feedback;
                         sampled_position_valid <=
                             (WEAK_POSITION_ENABLE != 0) && position_loop_enable &&
                             enable && enable_last &&
                             (control_mode == MODE_BAL) &&
                             (abs_wire_error <= POSITION_ANGLE_LIMIT);

                         // RUN3: 自起摆全程追踪D历史，但只在BAL分支实际使用。
                         if (BALANCE_D_WINDOW > 1) begin
                             if (!enable || !enable_last || (control_mode != MODE_BAL))
                                 sampled_d_rate <= 32'sd0;
                             else
                                 sampled_d_rate <= adc_linear_delta(
                                     adc_value, angle_history[BALANCE_D_WINDOW-1]);
                         end else begin
                             sampled_d_rate <= wire_rate;
                         end

                         // AUTO-SWING期间无论enable是否为0，都继续推进D历史队列。
                         // 手扶直立RUN3则与旧控制器一致：未进入BAL时重填历史。
                         if (BALANCE_D_WINDOW > 1) begin
                             if (angle_history_track ||
                                 (enable && enable_last && (control_mode == MODE_BAL))) begin
                                 angle_history[0] <= adc_value;
                                 for (history_index = 1; history_index < BALANCE_D_WINDOW;
                                      history_index = history_index + 1)
                                     angle_history[history_index] <= angle_history[history_index-1];
                             end else begin
                                 for (history_index = 0; history_index < BALANCE_D_WINDOW;
                                      history_index = history_index + 1)
                                     angle_history[history_index] <= adc_value;
                             end
                         end
                         adc_last <= adc_value;
                         enable_last <= enable;

                         // ADC 毛刺统计与旧版一致：只统计，暂不擅自改变控制策略。
                         if (adc_sample_seen) begin
                             if ((abs_wire_rate > ADC_GLITCH_THRESHOLD) &&
                                 (debug_adc_glitch_count != 32'hffff_ffff))
                                 debug_adc_glitch_count <= debug_adc_glitch_count + 1'b1;
                             if (abs_wire_rate > debug_adc_max_delta)
                                 debug_adc_max_delta <= abs_wire_rate;
                         end
                         adc_sample_seen <= 1'b1;
                         state <= S_POS_DIFF;
                     end
                end

                S_POS_DIFF: begin
                     // 第 1 拍：位置误差先扩展为 64 位再相减，防止 32 位溢出。
                     position_error_64 <=
                         $signed({{32{sampled_position[31]}}, sampled_position}) -
                         $signed({{32{sampled_target[31]}}, sampled_target});
                     position_window_delta_64 <=
                         $signed({{32{sampled_position[31]}}, sampled_position}) -
                         $signed({{32{position_window_start[31]}}, position_window_start});
                     legacy_position_error <= sampled_target - sampled_position;
                     state <= S_POS_MUL;
                end

                S_POS_MUL: begin
                     // 第 2 拍：位置 P、D 分开乘，不串入同拍限幅/角度 PID。
                     position_p_product <= position_error_64 * POSITION_KP_NUM;
                     position_d_product <= position_window_delta_64 * POSITION_KD_NUM;
                     state <= S_POS_REQ;
                end

                S_POS_REQ: begin
                     // 第 3 拍：位置外环固定点缩放及目标请求。
                     position_requested <= POSITION_FEEDBACK_SIGN *
                         (symmetric_scale(position_p_product, POSITION_KP_SHIFT) +
                          symmetric_scale(position_d_product, POSITION_KD_SHIFT));
                     state <= S_POS_APPLY;
                end

                S_POS_APPLY: begin
                     // 第 4 拍：位置环更新周期、限幅、斜率由RUN3顶层实参决定。
                     if ((WEAK_POSITION_ENABLE == 0) || !sampled_position_loop_enable ||
                         !sampled_enable ||
                         !sampled_prev_enable || (sampled_mode != MODE_BAL) ||
                         (sampled_abs_error > POSITION_ANGLE_LIMIT)) begin
                         position_sample_count <= 32'd0;
                         position_window_start <= sampled_position;
                         position_feedback <= 64'sd0;
                     end else if (position_sample_count == POSITION_UPDATE_SAMPLES - 1) begin
                         position_sample_count <= 32'd0;
                         position_window_start <= sampled_position;
                         if (position_request_limited > position_slew_up)
                             position_feedback <= position_slew_up;
                         else if (position_request_limited < position_slew_down)
                             position_feedback <= position_slew_down;
                         else
                             position_feedback <= position_request_limited;
                     end else begin
                         position_sample_count <= position_sample_count + 1'b1;
                     end
                     cascade_error <= sampled_error +
                         ((POSITION_CASCADE_ENABLE != 0 &&
                           sampled_position_loop_enable) ?
                          sampled_position_feedback[31:0] : 32'sd0);
                     state <= S_ANGLE;
                end

                S_ANGLE: begin
                     // 第 5 拍：角度 P、D、旧位置项和起摆速度项分别寄存。
                     if (KP_ANGLE_NUM != 0)
                         angle_p_term <= symmetric_scale(
                             $signed(cascade_error) * KP_ANGLE_NUM, KP_ANGLE_SHIFT);
                     else
                         angle_p_term <= KP_ANGLE * cascade_error;
                     angle_d_term <= KD_ANGLE * sampled_d_rate;
                     legacy_p_term <= KP_POSITION * legacy_position_error;
                     legacy_d_term <= KD_POSITION * sampled_velocity;
                     swing_speed_term <= sampled_rate * SWING_SPEED_GAIN;
                     state <= S_I_CAND;
                end

                S_I_CAND: begin
                     // 第 6 拍：角度 PD 合成、积分累加限幅；起摆平方并行计算。
                     angle_pd_result <= angle_p_term + angle_d_term;
                     base_control <= angle_p_term + angle_d_term +
                                     (sampled_position_loop_enable ?
                                       (legacy_p_term + legacy_d_term) : 32'sd0);
                     swing_error_square <= $signed(sampled_error) * $signed(sampled_error);
                     swing_speed_square <= $signed(swing_speed_term) * $signed(swing_speed_term);

                     integral_enabled <=
                         (KI_ANGLE != 0) && sampled_enable && sampled_prev_enable &&
                         (sampled_mode == MODE_BAL) &&
                         (sampled_abs_error <= ANGLE_RELEASE);
                     if ((KI_ANGLE == 0) || !sampled_enable || !sampled_prev_enable ||
                         (sampled_mode != MODE_BAL) ||
                         (sampled_abs_error > ANGLE_RELEASE)) begin
                         integral_candidate <= 64'sd0;
                         angle_i_sample_count <= 32'd0;
                     end else if (angle_i_sample_count == ANGLE_I_UPDATE_SAMPLES - 1) begin
                         angle_i_sample_count <= 32'd0;
                         if (angle_integral + cascade_error > ANGLE_I_ACCUM_LIMIT)
                             integral_candidate <= ANGLE_I_ACCUM_LIMIT;
                         else if (angle_integral + cascade_error < -ANGLE_I_ACCUM_LIMIT)
                             integral_candidate <= -ANGLE_I_ACCUM_LIMIT;
                         else
                             integral_candidate <= angle_integral + cascade_error;
                     end else begin
                         angle_i_sample_count <= angle_i_sample_count + 1'b1;
                         integral_candidate <= angle_integral;
                     end
                     state <= S_I_SCALE;
                end

                S_I_SCALE: begin
                     // 第 7 拍：积分候选值和上一值分别缩放，起摆能量合成。
                     i_candidate_value <= limit_angle_i(symmetric_scale(
                         integral_candidate * KI_ANGLE, KI_ANGLE_SHIFT));
                     i_previous_value <= limit_angle_i(symmetric_scale(
                         angle_integral * KI_ANGLE, KI_ANGLE_SHIFT));
                     swing_energy <= swing_error_square + swing_speed_square;
                     state <= S_RAW;
                end

                S_RAW: begin
                     // 第 8 拍：抗积分饱和选择并生成摩擦补偿前的控制命令。
                     if (sampled_balancing) begin
                         raw_control <= complete_base +
                             (integral_enabled ? chosen_i : 32'sd0);
                         if (integral_enabled)
                             angle_integral <= anti_windup ? angle_integral : integral_candidate;
                         else
                             angle_integral <= 64'sd0;
                     end else if (sampled_swinging) begin
                         angle_integral <= 64'sd0;
                         // 与原版相同：静止时按角度误差符号起摆；
                         // 其它情况按速度、角度分段及能量误差决定方向。
                         if (sampled_abs_rate == 0)
                             raw_control <= (sampled_error >= 0) ? SWING_GAIN : -SWING_GAIN;
                         else if (((sampled_rate < 0) ^
                                   (sampled_abs_error >= COS_ZERO_CROSS) ^
                                   (swing_energy > SWING_TARGET_ENERGY)))
                             raw_control <= -SWING_GAIN;
                         else
                             raw_control <= SWING_GAIN;
                     end else begin
                         // 启动、停机或状态转换的这一采样，只初始化，输出零命令。
                         // 解决零角度误差启动时误走旧 SAFE 起摆分支输出 +700 的问题。
                         raw_control <= 32'sd0;
                         angle_integral <= 64'sd0;
                     end
                     state <= S_FRIC_PRE;
                end

                S_FRIC_PRE: begin
                     // 第 9 拍：单独计算摩擦补偿使用的限幅幅值。
                     if (abs_raw >= (32'sd1 <<< FRICTION_RAMP_SHIFT))
                         friction_magnitude <= (32'sd1 <<< FRICTION_RAMP_SHIFT);
                     else
                         friction_magnitude <= abs_raw;
                     state <= S_FRIC_APPLY;
                end

                S_FRIC_APPLY: begin
                     // 第 10 拍：摩擦补偿只作用于平衡分支，起摆输出不加补偿。
                     if (sampled_balancing && (raw_control > 0))
                         corrected_control <= raw_control +
                             ((friction_magnitude * FRICTION_FWD) >>> FRICTION_RAMP_SHIFT);
                     else if (sampled_balancing && (raw_control < 0))
                         corrected_control <= raw_control -
                             ((friction_magnitude * FRICTION_REV) >>> FRICTION_RAMP_SHIFT);
                     else
                         corrected_control <= raw_control;
                     state <= S_COMMIT;
                end

                S_COMMIT: begin
                     // 第 11 拍：命令、控制状态与诊断数据同一时钟提交。
                     // 所有大数运算均在之前各拍完成，本拍只进行最后限幅。
                     control_mode <= sampled_next_mode;
                     angle_error <= sampled_error[15:0];
                     debug_balance_rate <= sampled_d_rate[15:0];
                     debug_angle_pd <= sampled_balancing ? angle_pd_result : 32'sd0;
                     debug_position_applied <=
                         (sampled_balancing && sampled_position_valid) ?
                         sampled_position_feedback[31:0] : 32'sd0;
                     debug_raw_control <= raw_control;
                     debug_flags <= {2'b00, sampled_prev_enable,
                                     final_limited && sampled_continuous,
                                     sampled_balancing && sampled_position_valid,
                                     sampled_balancing,
                                     (WEAK_POSITION_ENABLE != 0), sampled_enable};

                     if (!sampled_continuous) begin
                         command <= 16'sd0;
                     end else if (corrected_control > OUTPUT_LIMIT) begin
                         command <= OUTPUT_LIMIT;
                         if (debug_limit_events != 32'hffff_ffff)
                             debug_limit_events <= debug_limit_events + 1'b1;
                     end else if (corrected_control < -OUTPUT_LIMIT) begin
                         command <= -OUTPUT_LIMIT;
                         if (debug_limit_events != 32'hffff_ffff)
                             debug_limit_events <= debug_limit_events + 1'b1;
                     end else begin
                         command <= corrected_control[15:0];
                     end
                     state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
