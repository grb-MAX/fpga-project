// ACG525 倒立摆分层控制器：先起摆，进入直立捕获区后切换到平衡控制。
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
    parameter integer SWING_TARGET_ENERGY = 2_500_000, // 起摆能量估算的目标值，使用内部计数量纲。
    parameter integer COS_ZERO_CROSS      = 1024, // 用于起摆符号分段判断的角度计数阈值。
    // 电机命令对称限幅；默认 900 在 PWM 周期 2500 时对应 36% 驱动占空比。
    parameter integer OUTPUT_LIMIT        = 900, // 两种控制分支共用的最终输出幅值限制。
    // 平衡输出的正反向摩擦补偿；设为 0 可关闭，单位为 PWM 计数。
    parameter integer FRICTION_FWD        = 0, // 平衡分支的正向最大摩擦补偿，设为 0 可关闭。
    parameter integer FRICTION_REV        = 0, // 平衡分支的反向最大摩擦补偿，设为 0 可关闭。
    // 原始命令在 +/-2^SHIFT 内线性增加补偿，避免零点处突跳。
    parameter integer FRICTION_RAMP_SHIFT = 7, // 补偿渐增范围为 2 的此参数次方，默认 128 计数。
    parameter integer ADC_GLITCH_THRESHOLD = 512 // 相邻 1 kHz ADC 差分超过此值计为毛刺。
) (
    input  wire               clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire               rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire               sample_tick, // 采样更新使能脉冲，顶层连接 ADC 完成事件，默认约每 1 ms 一次。
    input  wire               enable, // 模块运行使能，具体输出更新时刻由下方时序逻辑决定。
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
    localparam [1:0] MODE_SAFE  = 2'd0; // 控制器停止状态；与顶层运行模式的编号独立。
    localparam [1:0] MODE_SWING = 2'd1; // 起摆或偏离直立后的恢复控制状态。
    localparam [1:0] MODE_BAL   = 2'd2; // 直立平衡状态，使用角度 PD 及可选位置/速度反馈。

    reg        [11:0] adc_last; // 上次采样的原始 ADC 值，用于计算线性相邻差分。
    reg               enable_last; // 上个采样时刻的使能状态，用于识别本次启动。
    reg [11:0]        angle_history [0:BALANCE_D_WINDOW-1]; // 平衡差分的 ADC 历史队列，下标 0 存最近一次历史值。
    integer           history_index; // 历史队列复位、重填和移位循环使用的下标。

    reg [31:0] position_sample_count; // 弱外环窗口计数，只在平衡小角度区累计。
    reg signed [31:0] position_window_start; // 外环窗口起点，重启时重填以清除旧速度。
    reg signed [63:0] position_feedback; // 保持的位置环输出；串级模式下是角度目标偏移。
    reg signed [63:0] outer_position_error; // 当前位置减目标位置，先扩位再相减。
    reg signed [63:0] outer_window_delta; // 整个窗口的编码器位置增量，不使用量化较粗的单毫秒速度。
    reg signed [63:0] outer_requested; // 缩放、合成后的外环请求。

    assign debug_window_count = position_sample_count[7:0];

    reg signed [31:0] error_now; // 本次 ADC 相对参考零点的线性有符号角度误差。
    reg signed [31:0] rate_now; // 当前 ADC 相对上次 ADC 的线性差分，使用一个采样间隔。
    reg signed [31:0] balance_rate_now; // 平衡分支使用的差分值；窗口大于 1 时不除以窗口长度。
    reg signed [31:0] cascade_error_now; // 串级位置环启用时，实际角度误差加位置环角度目标偏移。
    reg signed [31:0] position_error; // 旧直接 PWM 路径使用的目标位置减当前编码器位置。
    reg signed [63:0] energy_now; // 角度误差平方与缩放差分平方之和，为起摆能量估算值。
    reg signed [63:0] energy_error; // 目标能量减估算能量，负值表示当前估算能量已超过目标。
    reg signed [63:0] control_value; // 当前分支计算的 64 位控制中间量，随后加补偿并限幅。
    reg signed [63:0] friction_level; // 原始命令幅值，截取至渐增范围上限后用于计算补偿。
    reg signed [63:0] swing_value; // 起摆分支计算得到的有符号固定幅值命令。
    reg signed [63:0] abs_error; // 当前角度误差绝对值，用于捕获、释放及分段判断。
    reg signed [63:0] abs_rate; // 相邻采样误差差分的绝对值，用于捕获和静止判断。
    reg signed [63:0] speed_term; // 相邻采样误差差分乘起摆速度系数，用于能量估算。
    reg signed [63:0] swing_sign; // 起摆方向符号，只取正 1 或负 1。
    reg signed [63:0] angle_integral; // 平衡角度误差累加值，离开平衡区或停机时清零。
    reg signed [63:0] angle_integral_candidate; // 本采样待检查的积分累加值。
    reg signed [63:0] angle_i_value; // 按定点缩放并限幅后的积分输出。
    reg signed [63:0] angle_p_value; // 定点或整数比例项输出。
    reg [31:0] angle_i_sample_count; // 角度积分分频计数，模拟参考程序每 5 ms 调用一次 PID。
    reg adc_sample_seen; // 忽略复位后的首个样本，避免 adc_last=0 造成伪毛刺。

    // SV01A103 有效行程约 333.3 度，不是 360 度，ADC 端点不构成环形角度。
    // 角度误差、相邻速率和多采样 D 项均使用普通线性有符号差分。
    function signed [31:0] adc_linear_delta;
        input [11:0] current_adc;
        input [11:0] previous_adc;
        begin
            adc_linear_delta = $signed({1'b0, current_adc}) -
                               $signed({1'b0, previous_adc});
        end
    endfunction

    // 正负数均向零截断，避免负数算术右移向下取整引入固定单向偏置。
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

    // 每次 sample_tick（默认约 1 ms）更新一次角度误差、控制量和状态机。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_last      <= 12'd0;
            enable_last   <= 1'b0;
            command       <= 16'sd0;
            angle_error   <= 16'sd0;
            control_mode  <= MODE_SAFE;
            debug_angle_pd <= 32'sd0;
            debug_position_applied <= 32'sd0;
            debug_balance_rate <= 16'sd0;
            debug_raw_control <= 32'sd0;
            debug_flags <= 8'd0;
            debug_adc_glitch_count <= 32'd0;
            debug_adc_max_delta <= 32'd0;
            debug_limit_events <= 32'd0;
            adc_sample_seen <= 1'b0;
            position_sample_count <= 32'd0;
            position_window_start <= 32'sd0;
            position_feedback <= 64'sd0;
            angle_integral <= 64'sd0;
            angle_i_sample_count <= 32'd0;
            for (history_index = 0; history_index < BALANCE_D_WINDOW;
                 history_index = history_index + 1)
                angle_history[history_index] <= 12'd0;
        end else if (sample_tick) begin
            // 正误差定义为 ADC 线性值高于直立零点，沿用已使用的控制符号。
            // 本拍中间量用阻塞赋值依次计算；对外输出和历史量用非阻塞赋值保存。
            error_now     = adc_linear_delta(adc_value, angle_zero);
            rate_now      = adc_linear_delta(adc_value, adc_last);
            position_error = position_target - encoder_position;
            abs_error     = (error_now < 0) ? -error_now : error_now;
            abs_rate      = (rate_now < 0) ? -rate_now : rate_now;
            cascade_error_now = error_now;
            if (POSITION_CASCADE_ENABLE)
                cascade_error_now = error_now + position_feedback[31:0];
            if (adc_sample_seen) begin
                if (abs_rate > ADC_GLITCH_THRESHOLD &&
                    debug_adc_glitch_count != 32'hffff_ffff)
                    debug_adc_glitch_count <= debug_adc_glitch_count + 1'b1;
                if (abs_rate > debug_adc_max_delta)
                    debug_adc_max_delta <= abs_rate[31:0];
            end
            adc_sample_seen <= 1'b1;
            speed_term    = rate_now * SWING_SPEED_GAIN;
            energy_now    = (error_now * error_now) + (speed_term * speed_term);
            energy_error  = SWING_TARGET_ENERGY - energy_now;

            // 参考程序按固定控制周期执行 ErrorInt += Error0；这里每 5 个 1 ms
            // 样本更新一次，Ki=0 时清零，保持与参考程序相同的积分时序。
            // 起摆、停机或误差超出平衡释放区时清零，重新捕获后从零开始积分。
            if (!enable || !enable_last || (control_mode != MODE_BAL) ||
                (abs_error > ANGLE_RELEASE) || (KI_ANGLE == 0)) begin
                angle_integral_candidate = 64'sd0;
                angle_integral <= 64'sd0;
                angle_i_sample_count <= 32'd0;
            end else begin
                if (angle_i_sample_count == ANGLE_I_UPDATE_SAMPLES - 1) begin
                    angle_i_sample_count <= 32'd0;
                    if (angle_integral + cascade_error_now > ANGLE_I_ACCUM_LIMIT)
                        angle_integral_candidate = ANGLE_I_ACCUM_LIMIT;
                    else if (angle_integral + cascade_error_now < -ANGLE_I_ACCUM_LIMIT)
                        angle_integral_candidate = -ANGLE_I_ACCUM_LIMIT;
                    else
                        angle_integral_candidate = angle_integral + cascade_error_now;
                end else begin
                    angle_i_sample_count <= angle_i_sample_count + 1'b1;
                    angle_integral_candidate = angle_integral;
                end
            end
            angle_i_value = 64'sd0;
            if (KP_ANGLE_NUM != 0)
                angle_p_value = symmetric_scale(KP_ANGLE_NUM * cascade_error_now,
                                                KP_ANGLE_SHIFT);
            else
                angle_p_value = KP_ANGLE * cascade_error_now;

            angle_error <= error_now[15:0];
            adc_last    <= adc_value;

            // 仅平衡 D 使用多采样差分；捕获的变化率条件和起摆估算仍用相邻采样差。
            // 释放条件只判断线性角度误差。多采样历史保存 ADC 原值，差分时仍使用线性差分。
            if (BALANCE_D_WINDOW > 1) begin
                if (!enable || !enable_last || (control_mode != MODE_BAL)) begin
                    balance_rate_now = 32'sd0;
                    // 暂停、启动和起摆期间用当前值填满，避免旧历史带入平衡。
                    for (history_index = 0; history_index < BALANCE_D_WINDOW;
                         history_index = history_index + 1)
                        angle_history[history_index] <= adc_value;
                end else begin
                    balance_rate_now = adc_linear_delta(
                        adc_value, angle_history[BALANCE_D_WINDOW-1]);
                    angle_history[0] <= adc_value;
                    for (history_index = 1; history_index < BALANCE_D_WINDOW;
                         history_index = history_index + 1)
                        angle_history[history_index] <= angle_history[history_index-1];
                end
            end else begin
                balance_rate_now = rate_now;
            end

            // 以下寄存器仅用于观测，不参与控制计算；与本采样输出同时更新。
            debug_angle_pd <= 32'sd0;
            debug_position_applied <= 32'sd0;
            debug_balance_rate <= balance_rate_now[15:0];
            debug_raw_control <= 32'sd0;
            debug_flags <= {2'd0, enable_last, 2'd0,
                            (enable && (control_mode == MODE_BAL)),
                            (WEAK_POSITION_ENABLE != 0), enable};

            // 位置环使用当前位置减目标位置及窗口位移；符号由 POSITION_FEEDBACK_SIGN 校准。
            if (!WEAK_POSITION_ENABLE || !enable || !enable_last ||
                (control_mode != MODE_BAL) || (abs_error > POSITION_ANGLE_LIMIT)) begin
                position_sample_count <= 32'd0;
                position_window_start <= encoder_position;
                position_feedback <= 64'sd0;
            end else if (position_sample_count == POSITION_UPDATE_SAMPLES - 1) begin
                position_sample_count <= 32'd0;
                outer_position_error = $signed({{32{encoder_position[31]}}, encoder_position})
                                     - $signed({{32{position_target[31]}}, position_target});
                outer_window_delta = $signed({{32{encoder_position[31]}}, encoder_position})
                                   - $signed({{32{position_window_start[31]}}, position_window_start});
                position_window_start <= encoder_position;
                outer_requested = POSITION_FEEDBACK_SIGN *
                    (symmetric_scale(outer_position_error * POSITION_KP_NUM, POSITION_KP_SHIFT)
                   + symmetric_scale(outer_window_delta * POSITION_KD_NUM, POSITION_KD_SHIFT));

                if (outer_requested > POSITION_OUTPUT_LIMIT)
                    outer_requested = POSITION_OUTPUT_LIMIT;
                else if (outer_requested < -POSITION_OUTPUT_LIMIT)
                    outer_requested = -POSITION_OUTPUT_LIMIT;

                if (outer_requested > position_feedback + POSITION_SLEW_LIMIT)
                    position_feedback <= position_feedback + POSITION_SLEW_LIMIT;
                else if (outer_requested < position_feedback - POSITION_SLEW_LIMIT)
                    position_feedback <= position_feedback - POSITION_SLEW_LIMIT;
                else
                    position_feedback <= outer_requested;
            end else begin
                position_sample_count <= position_sample_count + 1'b1;
            end

            if (!enable) begin
                enable_last  <= 1'b0;
                command      <= 16'sd0;
                control_mode <= MODE_SAFE;
            end else begin
                if (!enable_last) begin
                    // 启动时满足角度和速度捕获条件则平衡，否则进入起摆分支。
                    // 位置目标由顶层启动/目标按键逻辑传入，本模块不修改目标。
                    if ((abs_error <= ANGLE_CAPTURE) &&
                        (abs_rate <= CAPTURE_RATE))
                        control_mode <= MODE_BAL;
                    else
                        control_mode <= MODE_SWING;

                end else if ((control_mode == MODE_BAL) &&
                             (abs_error > ANGLE_RELEASE)) begin
                    control_mode <= MODE_SWING;
                end else if ((control_mode == MODE_SWING) &&
                             (abs_error <= ANGLE_CAPTURE) &&
                             (abs_rate <= CAPTURE_RATE)) begin
                    control_mode <= MODE_BAL;
                end

                enable_last <= 1'b1;

                // control_mode 本拍采用非阻塞更新；以下分支使用更新前状态，
                // 新状态对应的输出计算在下一个采样节拍生效。
                if (control_mode == MODE_BAL) begin
                    // 先保留既有角度 PD 和旧位置公式，再按开关叠加独立约束的新外环。
                    // 旧位置误差仍为目标减实际；顶层继续将这两个旧增益设为零。
                    control_value = angle_p_value
                                  + (KD_ANGLE * balance_rate_now)
                                  + (KP_POSITION * position_error)
                                  + (KD_POSITION * encoder_velocity);
                    debug_angle_pd <= angle_p_value + (KD_ANGLE * balance_rate_now);

                    // 外环仅在已连续进入小角度平衡区后作用；新窗口值下一采样拍生效。
                    // 保留原始角度、角度微分、旧位置项和最终电机方向。
                    if (POSITION_CASCADE_ENABLE && WEAK_POSITION_ENABLE && enable_last &&
                        (abs_error <= POSITION_ANGLE_LIMIT)) begin
                        debug_position_applied <= position_feedback[31:0];
                        debug_flags[3] <= 1'b1;
                    end else if (!POSITION_CASCADE_ENABLE && WEAK_POSITION_ENABLE &&
                                 enable_last && (abs_error <= POSITION_ANGLE_LIMIT)) begin
                        debug_position_applied <= position_feedback[31:0];
                        debug_flags[3] <= 1'b1;
                        control_value = control_value + position_feedback;
                    end

                    // 积分在完整角度环和弱位置项之后加入，抗饱和判断覆盖合成命令。
                    if (KI_ANGLE != 0 && enable_last &&
                        (abs_error <= ANGLE_RELEASE)) begin
                        angle_i_value = symmetric_scale(
                            angle_integral_candidate * KI_ANGLE, KI_ANGLE_SHIFT);
                        if (angle_i_value > ANGLE_I_LIMIT)
                            angle_i_value = ANGLE_I_LIMIT;
                        else if (angle_i_value < -ANGLE_I_LIMIT)
                            angle_i_value = -ANGLE_I_LIMIT;

                        // 输出已到限幅且误差仍同向时冻结积分，避免积分继续推动饱和。
                        if (((control_value + angle_i_value > OUTPUT_LIMIT) &&
                            (cascade_error_now > 0)) ||
                            ((control_value + angle_i_value < -OUTPUT_LIMIT) &&
                             (cascade_error_now < 0))) begin
                            angle_i_value = symmetric_scale(
                                angle_integral * KI_ANGLE, KI_ANGLE_SHIFT);
                            if (angle_i_value > ANGLE_I_LIMIT)
                                angle_i_value = ANGLE_I_LIMIT;
                            else if (angle_i_value < -ANGLE_I_LIMIT)
                                angle_i_value = -ANGLE_I_LIMIT;
                        end else begin
                            angle_integral <= angle_integral_candidate;
                        end
                        control_value = control_value + angle_i_value;
                    end else begin
                        angle_integral <= 64'sd0;
                    end

                    debug_raw_control <= control_value[31:0];

                    // 仅补偿平衡分支。零命令不加偏置，小命令逐渐增加补偿；
                    // 默认达到 128 计数后加满补偿，再执行下方的总输出限幅。
                    friction_level = (control_value < 0) ? -control_value : control_value;
                    if (friction_level > (64'sd1 <<< FRICTION_RAMP_SHIFT))
                        friction_level = (64'sd1 <<< FRICTION_RAMP_SHIFT);
                    if (control_value > 0)
                        control_value = control_value +
                            ((friction_level * FRICTION_FWD) >>> FRICTION_RAMP_SHIFT);
                    else if (control_value < 0)
                        control_value = control_value -
                            ((friction_level * FRICTION_REV) >>> FRICTION_RAMP_SHIFT);
                end else begin
                    // 未进入直立捕获区时按摆角、摆速和能量误差决定起摆方向。
                    // 以摆动方向为初始符号；超过角度分段阈值或能量过高时分别反转符号。
                    // 这是内部计数构造的能量估算，并非以焦耳为单位的物理能量。
                    if (rate_now >= 0)
                        swing_sign = 64'sd1;
                    else
                        swing_sign = -64'sd1;

                    if (abs_error >= COS_ZERO_CROSS)
                        swing_sign = -swing_sign;

                    if (energy_error < 0)
                        swing_sign = -swing_sign;

                    swing_value = swing_sign * SWING_GAIN;
                    // 相邻差分为零时直接按角度误差符号起动，覆盖上面的能量符号结果。
                    if (abs_rate == 0)
                        swing_value = (error_now >= 0) ? SWING_GAIN : -SWING_GAIN;
                    control_value = swing_value;
                    debug_raw_control <= control_value[31:0];
                end

                // 对正负控制量做对称限幅，再交给顶层 PWM 模块。
                if (control_value > OUTPUT_LIMIT) begin
                    debug_flags[4] <= 1'b1;
                    if (debug_limit_events != 32'hffff_ffff)
                        debug_limit_events <= debug_limit_events + 1'b1;
                    command <= OUTPUT_LIMIT;
                end else if (control_value < -OUTPUT_LIMIT) begin
                    debug_flags[4] <= 1'b1;
                    if (debug_limit_events != 32'hffff_ffff)
                        debug_limit_events <= debug_limit_events + 1'b1;
                    command <= -OUTPUT_LIMIT;
                end else
                    command <= control_value[15:0];
            end
        end
    end

endmodule