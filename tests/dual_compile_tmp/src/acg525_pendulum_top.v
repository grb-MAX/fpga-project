`timescale 1ns / 1ps

// ACG525 倒立摆工程顶层：连接角度 ADC、编码器、电机驱动、按键、OLED 和 UART。
// 主时钟为 50 MHz；ADC 采样完成脉冲同时作为控制、测速和按键采样节拍。
// 所有按键均为低电平按下；KEY2 直接作为全局低有效异步复位。
// 调试入口：KEY0 选模式，启动/暂停键控制运行，KEY1 将当前 ADC 值设为直立零点。
// 信号路径：ADC/编码器 -> 控制器 -> 模式命令选择 -> 有符号命令 -> AT8236 PWM。
module acg525_pendulum_top (
    input  wire        clk_50m, // 50 MHz 系统时钟，周期 20 ns。
    input  wire        key0_n, // KEY0：停止时循环选择待启动模式。
    input  wire        key1_n, // KEY1：角度校零，锁存最近一次 ADC 采样值。
    input  wire        key2_n, // KEY2：低有效全局复位，恢复停机和默认零点。
    input  wire        mode_start_n, // 独立启动/暂停键，低有效。
    input  wire        target_inc_n, // 目标位置加键，每次增加 100 个编码器计数。
    input  wire        target_dec_n, // 目标位置减键，每次减少 100 个编码器计数。
    input  wire        adc_dout, // MCP3201 串行数据输入。
    output wire        adc_cs_n, // MCP3201 片选，拉低后开始一次转换/读取。
    output wire        adc_sclk, // MCP3201 串行时钟，由采集模块分频产生。
    output wire        motor_in1, // AT8236 IN1，具体低有效 PWM 波形由 motor0 生成。
    output wire        motor_in2, // AT8236 IN2，与 IN1 配合确定驱动方向。
    input  wire        encoder_a, // 电机编码器 A 相，由 encoder0 同步后解码。
    input  wire        encoder_b, // 电机编码器 B 相，与 A 相共同判断计数方向。
    inout  wire        oled_scl, // OLED I2C 时钟，开漏输出，需外部上拉。
    inout  wire        oled_sda, // OLED I2C 数据，开漏输出并回读总线。
    output wire        uart_tx, // 遥测串口发送，默认 115200 波特率、8N1。
    output wire [3:0]  led // 状态输出：心跳、ADC 活动、电机使能、编码器活动。
);

    // KEY2 不经过消抖模块，直接送往各子模块的低有效异步复位端。
    wire rst_n = key2_n; // 松开 KEY2 后为高电平，逻辑开始正常运行。
    wire [11:0] adc_value; // 完整 12 位 ADC 结果，范围 0~4095。
    wire        adc_valid; // 新样本有效脉冲，持续一个系统时钟，默认约每 1 ms 一次。
    wire        adc_busy; // SPI 正在读取；顶层保留此状态，未用于运行模式判断。
    wire signed [31:0] encoder_position; // 编码器累计位置，带符号计数。
    wire signed [31:0] encoder_velocity; // 相邻采样的位置增量，默认单位约为计数/ms。
    // 两套控制器使用相同的 ADC/编码器输入，但不共用积分、位置历史和状态机。
    // RUN3：ZIP 中的原始 pendulum_controller；RUN4：分拍 pendulum_controller_run4。
    // 两种模式严格互斥地使能，未运行的控制器会在下一个 ADC 采样清理内部状态。
    wire signed [15:0] run3_command, run4_command;
    wire signed [15:0] run3_angle_error, run4_angle_error;
    wire [1:0] run3_control_mode, run4_control_mode;
    wire signed [31:0] run3_debug_angle_pd, run4_debug_angle_pd;
    wire signed [31:0] run3_debug_position_applied, run4_debug_position_applied;
    wire signed [15:0] run3_debug_balance_rate, run4_debug_balance_rate;
    wire signed [31:0] run3_debug_raw_control, run4_debug_raw_control;
    wire [7:0] run3_debug_flags, run4_debug_flags;
    wire [7:0] run3_debug_window_count, run4_debug_window_count;
    wire [31:0] run3_debug_adc_glitch_count, run4_debug_adc_glitch_count;
    wire [31:0] run3_debug_adc_max_delta, run4_debug_adc_max_delta;
    wire [31:0] run3_debug_limit_events, run4_debug_limit_events;
    reg signed [15:0] drive_command; // 物理 PWM 和 UART 共同看到的、按模式选择后的命令。

    // Temporary compile-only declaration placement fix for the ZIP source.
    localparam [2:0] RUN_TRAJECTORY = 3'd4;
    reg [2:0] run_mode;

    // UART、OLED、轨迹平衡门控均使用当前模式对应的控制器信号。
    wire is_run4_selected = (run_mode == RUN_TRAJECTORY);
    wire signed [15:0] control_command = is_run4_selected ? run4_command : run3_command;
    wire signed [15:0] angle_error = is_run4_selected ? run4_angle_error : run3_angle_error;
    wire [1:0] control_mode = is_run4_selected ? run4_control_mode : run3_control_mode;
    wire signed [31:0] debug_angle_pd = is_run4_selected ? run4_debug_angle_pd : run3_debug_angle_pd;
    wire signed [31:0] debug_position_applied = is_run4_selected ? run4_debug_position_applied : run3_debug_position_applied;
    wire signed [15:0] debug_balance_rate = is_run4_selected ? run4_debug_balance_rate : run3_debug_balance_rate;
    wire signed [31:0] debug_raw_control = is_run4_selected ? run4_debug_raw_control : run3_debug_raw_control;
    wire [7:0] debug_flags = is_run4_selected ? run4_debug_flags : run3_debug_flags;
    wire [7:0] debug_window_count = is_run4_selected ? run4_debug_window_count : run3_debug_window_count;
    wire [31:0] debug_adc_glitch_count = is_run4_selected ? run4_debug_adc_glitch_count : run3_debug_adc_glitch_count;
    wire [31:0] debug_adc_max_delta = is_run4_selected ? run4_debug_adc_max_delta : run3_debug_adc_max_delta;
    wire [31:0] debug_limit_events = is_run4_selected ? run4_debug_limit_events : run3_debug_limit_events;
    reg telemetry_tick; // UART在RUN4校验完成后取本采样控制数据，避免新旧命令混帧。
    reg [10:0] telemetry_delay; // 11拍延迟：分拍控制器提交后，RUN4保护开始读取。
    reg [9:0] telemetry_guard_delay; // 进一步延迟10拍供UART使用，确保verified_command已更新。
    wire guard_sample_tick = telemetry_delay[10]; // 仅RUN4核验模块读取的采样节拍。
    reg [31:0] telemetry_sample_id; // 每个 ADC 控制采样递增，复位后从零开始。
    reg [11:0] telemetry_adc;
    reg [11:0] telemetry_zero;
    reg signed [31:0] telemetry_position;
    reg signed [15:0] telemetry_velocity;
    reg signed [31:0] telemetry_target;
    wire        key0_pulse; // KEY0 消抖后的单时钟按下事件。
    wire        key1_pulse; // KEY1 消抖后的单时钟校零事件。
    wire        mode_start_pulse; // 消抖后的启动/暂停事件。
    wire        target_inc_pulse; // 消抖后的目标位置加事件。
    wire        target_dec_pulse; // 消抖后的目标位置减事件。
    wire        oled_scl_low; // 为 1 时拉低 SCL，为 0 时释放总线。
    wire        oled_sda_low; // 为 1 时拉低 SDA，为 0 时释放总线。

    // angle_zero：KEY1 校零时锁存的 ADC 值；角度误差 = 当前 ADC - 零点。
    reg [11:0] angle_zero; // 上电/复位默认 3153，KEY1 可重新锁存。
    // 模式选择、运行使能和运行模式分别保存，暂停不会清除已经选择的模式。
    // run_mode declared above for compile ordering.
    reg [2:0]  selected_mode; // 停止时通过 KEY0 选择，下次启动时装入 run_mode。
    reg        run_active; // 运行请求标志，暂停或复位时清零。
    reg signed [31:0] target_position; // 位置目标，单位为编码器计数；用于有界的弱位置反馈。
    reg [23:0] heartbeat; // 自由运行计数器，取 bit23 作为心跳输出。
    reg        adc_activity; // 每出现一个 ADC 有效脉冲翻转一次。
    reg        encoder_activity; // 采样时检测到非零速度则翻转一次。
    reg [24:0] display_count; // OLED 刷新分频计数，计满 25,000,000 个时钟。
    reg        display_tick; // OLED 刷新请求脉冲，持续一个系统时钟。

    // 顶层模式编号：0 安全、1 正向测试、2 反向测试、3 起摆/平衡控制、4 正弦轨迹。
    // 与 control_mode 的内部状态编号独立；RUN3 内部仍会在起摆和平衡之间切换。
    localparam [2:0] RUN_SAFE     = 3'd0; // 安全模式：保持停止，启动键不会使电机运行。
    localparam [2:0] RUN_TEST_FWD = 3'd1; // 正向低速测试模式，使用固定正命令。
    localparam [2:0] RUN_TEST_REV = 3'd2; // 反向低速测试模式，使用固定负命令。
    localparam [2:0] RUN_BALANCE  = 3'd3; // 控制器运行模式，内部按条件选择起摆或直立平衡。
    // RUN_TRAJECTORY declared above for compile ordering.
    localparam integer POSITION_STEP = 32'sd100; // 每次目标位置按键的步长。

    // 只有启动 RUN3/RUN4 才使能控制器；RUN1/RUN2 绕过控制器，使用固定测试命令。
    // motor_enable 表示驱动允许输出，并不代表命令一定非零或电机一定正在转动。
    wire trajectory_mode = run_active && (run_mode == RUN_TRAJECTORY);
    wire run3_balance_enable = run_active && (run_mode == RUN_BALANCE);
    wire run4_balance_enable = trajectory_mode;

    // RUN4 仍沿用已经减轻抖动的首次进入平衡态后锁存电机使能逻辑。
    // 锁存后直接传递原控制器命令，不增加轨迹 PWM 补偿或反复关断。
    reg trajectory_balance_sampled;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            trajectory_balance_sampled <= 1'b0;
        else if (!trajectory_mode)
            trajectory_balance_sampled <= 1'b0;
        else if (adc_valid && (control_mode == 2'd2))
            trajectory_balance_sampled <= 1'b1;
    end
    wire trajectory_drive_ready =
        trajectory_mode && trajectory_balance_sampled;

    // ================================================================
    // RUN4 独立输出前核验（修正版：拒绝异常样本，连续三次异常锁止）
    //
    // 重要：过去的保护从 controller0 的 control_command 直接驱动 PWM，
    //       再通过 guard_stage 检查异常；故障检查的数个时钟内，
    //       异常 PWM 可能已送到 AT8236。
    // 本版改为：controller0 -> 快照 -> 多拍独立校验 -> verified_command -> PWM。
    //           PWM 不再直接连接未经校验的 control_command。
    // 单次异常：verified_command=0，本次 PWM 输出清零，下一样本恢复后可继续运行。
    // 连续三次异常：锁存停机，暂停/KEY2 复位后才能重新运行。
    // LED3 常亮表示曾检测到异常样本；LED2 同时熄灭才说明驱动已锁止。
    //
    // 平衡分支原控制器的精确关系（当前位置环是串级，旧直接位置增益为0）：
    //   debug_raw_control = debug_angle_pd + angle_i_value
    //   -600 <= angle_i_value <= +600 （ANGLE_I_LIMIT=600）
    // 再按原控制器的 FRICTION_FWD=FRICTION_REV=100、SHIFT=7 计算最终命令，
    // 比对 16 位 command 是否正确，并限制在 ±1400。
    //
    // 注意：这是一道输出隔离措施。若 50MHz 时序违例造成整个电路运行错误，
    //       仍必须修复时序；此保护不构成可认证的安全停机功能。
    // ================================================================
    reg output_fault_latched;
    reg [2:0] guard_stage;
    reg signed [31:0] guard_raw_snapshot;
    reg signed [31:0] guard_pd_snapshot;
    reg signed [15:0] guard_command_snapshot;
    reg guard_balance_snapshot;
    reg guard_math_invalid;
    reg signed [31:0] guard_abs_raw;
    reg signed [31:0] guard_friction;
    reg signed [31:0] guard_corrected;
    reg signed [15:0] guard_expected;
    reg signed [15:0] verified_command; // 唯一送往 RUN4 PWM 的已核验命令

    // 诊断用拒绝计数：连续三次检测失败才锁止；单次失败只清零当前输出。
    // 这样既不会放行异常命令，也不会因为一次孤立异常而永久停机。
    reg [1:0] guard_bad_streak;
    reg guard_reject_seen; // 最近一次启动中是否曾拒绝过异常命令，LED3 常亮表示。
    reg guard_enable_last_snapshot; // 控制器前一拍使能标志，用来抑制首次误起摆。

    wire guard_control_running = trajectory_mode; // 独立校验只作用于 RUN4，绝不锁住 RUN3。

    wire command_out_of_range = guard_control_running &&
        ((control_command > 16'sd1400) ||
         (control_command < -16'sd1400));

    wire signed [31:0] guard_delta =
        $signed({{16{guard_command_snapshot[15]}}, guard_command_snapshot}) -
        $signed({{16{guard_expected[15]}}, guard_expected});

    // 对 angle_i_value 的独立约束：由源码给定的 ANGLE_I_LIMIT 确定。
    // 使用 32 位有符号比较，正常角度 PD 远小于该位宽，避免 64 位算术链。
    wire guard_integral_invalid = guard_balance_snapshot &&
        ((guard_raw_snapshot > guard_pd_snapshot + 32'sd600) ||
         (guard_raw_snapshot < guard_pd_snapshot - 32'sd600));

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            output_fault_latched <= 1'b0;
            guard_stage <= 3'd0;
            guard_raw_snapshot <= 32'sd0;
            guard_pd_snapshot <= 32'sd0;
            guard_command_snapshot <= 16'sd0;
            guard_balance_snapshot <= 1'b0;
            guard_math_invalid <= 1'b0;
            guard_abs_raw <= 32'sd0;
            guard_friction <= 32'sd0;
            guard_corrected <= 32'sd0;
            guard_expected <= 16'sd0;
            verified_command <= 16'sd0;
            guard_bad_streak <= 2'd0;
            guard_reject_seen <= 1'b0;
            guard_enable_last_snapshot <= 1'b0;
        end else if (!trajectory_mode) begin
            // RUN3 与 RUN4 完全隔离：退出 RUN4 就清除其校验状态和延迟命令。
            // 进入 RUN3 不会继承 RUN4 的锁止或拒绝记录。
            output_fault_latched <= 1'b0;
            guard_stage <= 3'd0;
            guard_math_invalid <= 1'b0;
            verified_command <= 16'sd0;
            guard_bad_streak <= 2'd0;
            guard_reject_seen <= 1'b0;
        end else if (output_fault_latched) begin
            // 连续异常达到锁止门槛后保持停止，只有暂停或 KEY2 才清除。
            verified_command <= 16'sd0;
            guard_stage <= 3'd0;
        end else begin
            case (guard_stage)
                3'd0: begin
                    // telemetry_tick 已延迟到控制器 S_COMMIT 提交后的下一拍。
                    // 这些诊断值与 control_command 因此来自同一个 ADC 样本。
                    if (guard_sample_tick && guard_control_running) begin
                        guard_raw_snapshot <= debug_raw_control;
                        guard_pd_snapshot <= debug_angle_pd;
                        guard_command_snapshot <= control_command;
                        guard_balance_snapshot <= debug_flags[2];
                        // bit5 是 controller0 前一控制样本的 enable_last。
                        // 首次使能样本即使内部误走起摆分支也不送给 PWM。
                        guard_enable_last_snapshot <= debug_flags[5];
                        guard_stage <= 3'd1;
                    end
                end
                3'd1: begin
                    guard_math_invalid <= guard_integral_invalid;
                    // 此处只对不超过合理范围的数据进一步计算，
                    // 即便异常数据过大，最终仍会在 3'd5 阶段拒绝输出。
                    guard_abs_raw <= (guard_raw_snapshot < 0) ?
                                     -guard_raw_snapshot : guard_raw_snapshot;
                    guard_stage <= 3'd2;
                end
                3'd2: begin
                    // 复现原控制器摩擦补偿：floor(min(abs(raw),128)*100/128)。
                    // 非 BAL 分支不进行摩擦补偿。
                    if (!guard_balance_snapshot)
                        guard_friction <= 32'sd0;
                    else if (guard_abs_raw >= 32'sd128)
                        guard_friction <= 32'sd100;
                    else
                        guard_friction <=
                            ((guard_abs_raw <<< 6) +
                             (guard_abs_raw <<< 5) +
                             (guard_abs_raw <<< 2)) >>> 7;
                    guard_stage <= 3'd3;
                end
                3'd3: begin
                    if (guard_balance_snapshot && (guard_raw_snapshot > 0))
                        guard_corrected <= guard_raw_snapshot + guard_friction;
                    else if (guard_balance_snapshot && (guard_raw_snapshot < 0))
                        guard_corrected <= guard_raw_snapshot - guard_friction;
                    else
                        guard_corrected <= guard_raw_snapshot;
                    guard_stage <= 3'd4;
                end
                3'd4: begin
                    if (guard_corrected > 32'sd1400)
                        guard_expected <= 16'sd1400;
                    else if (guard_corrected < -32'sd1400)
                        guard_expected <= -16'sd1400;
                    else
                        guard_expected <= guard_corrected[15:0];
                    guard_stage <= 3'd5;
                end
                3'd5: begin
                    // 首次有效采样：只允许控制器初始化状态、差分历史和积分。
                    // 旧控制器首次进入 BAL 时可能先算出一拍 SWING_GAIN=700。
                    // 这一拍主动屏蔽，不作为异常计数。
                    if (!guard_enable_last_snapshot) begin
                        verified_command <= 16'sd0;
                        guard_bad_streak <= 2'd0;

                    end else if (guard_math_invalid ||
                                 (guard_command_snapshot > 16'sd1400) ||
                                 (guard_command_snapshot < -16'sd1400) ||
                                 (guard_delta > 32'sd2) ||
                                 (guard_delta < -32'sd2)) begin
                        // 本次数据没有通过一致性验证，禁止本次命令进 PWM。
                        // 不立刻永久锁死：只有连续第三个错误样本才锁止。
                        verified_command <= 16'sd0;
                        guard_reject_seen <= 1'b1;
                        if (guard_bad_streak >= 2'd2) begin
                            output_fault_latched <= 1'b1;
                            guard_bad_streak <= 2'd3;
                        end else begin
                            guard_bad_streak <= guard_bad_streak + 2'd1;
                        end

                    end else begin
                        // 本次命令通过完整校验，清除连续异常计数。
                        verified_command <= guard_command_snapshot;
                        guard_bad_streak <= 2'd0;
                    end
                    guard_stage <= 3'd0;
                end
                default: guard_stage <= 3'd0;
            endcase
        end
    end

    wire motor_request = run_active &&
        ((run_mode == RUN_TEST_FWD) ||
         (run_mode == RUN_TEST_REV) ||
         (run_mode == RUN_BALANCE) ||
         trajectory_drive_ready);
    // RUN3 保留原始控制命令的正常响应，不经过 RUN4 的多拍校验及连续锁止。
    // 唯一额外的底线：原控制器若出现超过 +/-1400 的不合法16位命令，
    // 此次命令直接丢弃为0，而不是截断为1400（避免把溢出突跳放行给电机）。
    // 警告：如果原控制器内部异常被限幅为刚好 +/-1400，这里仍无法识别。
    wire run3_command_invalid =
        (run3_command > 16'sd1400) || (run3_command < -16'sd1400);
    wire signed [15:0] run3_safe_command =
        run3_command_invalid ? 16'sd0 : run3_command;

    wire motor_enable = motor_request &&
        ((run_mode != RUN_TRAJECTORY) ||
         (!output_fault_latched && !command_out_of_range));

    // 输出口再做一次独立的硬限幅门控，防止异常验证寄存器被直接送到 PWM。
    // 不会改变任意一个合法的、绝对值不超过 1400 的 PID 命令。
    wire signed [15:0] safe_verified_command =
        ((verified_command > 16'sd1400) ||
         (verified_command < -16'sd1400)) ? 16'sd0 : verified_command;

    // 运行时显示正在执行的模式，停止/暂停时显示待启动的选择。
    wire [2:0] display_mode = motor_enable ? run_mode : selected_mode; // 运行时显示 run_mode，停止时显示 selected_mode。


    // ================================================================
    // RUN4 正弦位置目标生成层：在顶层实现，不需要 position_trajectory.v。
    // 目标 = 本次启动时的编码器位置 + 100*sin(2*pi*phase/4096)。
    // 4096 个允许推进的 ADC 样本完成一周期；1 kHz 且持续平衡时约 4.096 秒。
    // 只有 RUN4 且 control_mode==2（直立平衡）时才推进相位。
    // 起摆、退出平衡、停止/暂停时冻结目标与相位；停止后再启动从新位置的零点开始。
    // 为避免组合宽乘法，将 Q15 插值拆为 4 位移位加法，振幅 100 拆为 64+32+4。
    // ================================================================
    localparam [1:0] MODE_BAL = 2'd2;

    // 以当次 RUN4 启动时的当前位置为正弦中心，避免启动位置目标阶跃。
    wire signed [12:0] adc_diff =
        $signed({1'b0, adc_value}) - $signed({1'b0, angle_zero});
    wire [12:0] abs_adc_diff = (adc_diff < 0) ? -adc_diff : adc_diff;
    wire trajectory_start = mode_start_pulse && !run_active && !key0_pulse &&
        (selected_mode == RUN_TRAJECTORY) && (abs_adc_diff <= 13'd120);

    reg signed [31:0] trajectory_origin; // 运行中心，启动时锁存编码器位置。
    reg signed [31:0] trajectory_ref;    // RUN4 实时位置参考，送原有位置环。
    reg [11:0] trajectory_phase;         // 12 位相位：0~4095，溢出后自动回到 0。
    reg [11:0] trajectory_pending_phase;
    reg [2:0] trajectory_stage;
    reg signed [15:0] traj_sin0;
    reg signed [15:0] traj_sin1;
    reg [3:0] traj_fraction;
    reg signed [20:0] traj_interp_product;
    reg signed [17:0] traj_interp_sin;
    reg signed [31:0] traj_scaled_q15;
    reg signed [31:0] traj_relative_count;

    // RUN3 始终使用手动位置目标；RUN4 只在此处换成正弦目标。
    wire signed [31:0] controller_position_target =
        (run_mode == RUN_TRAJECTORY) ? trajectory_ref : target_position;

    // 四分之一周期 Q15 正弦表：0 度为 0，90 度为 32767。
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
                2'd0: sine_bin = quarter_sine({1'b0, sub_index});
                2'd1: sine_bin = quarter_sine(7'd64 - {1'b0, sub_index});
                2'd2: sine_bin = -quarter_sine({1'b0, sub_index});
                default: sine_bin = -quarter_sine(7'd64 - {1'b0, sub_index});
            endcase
        end
    endfunction

    // 相位高 8 位查表，低 4 位插值；一个完整周期共 4096 点。
    // 两个正弦端点分两个系统时钟读取，共用同一套查表逻辑。
    wire [7:0] traj_lookup_index = trajectory_pending_phase[11:4] +
        ((trajectory_stage == 3'd2) ? 8'd1 : 8'd0);
    wire signed [15:0] traj_lookup_value = sine_bin(traj_lookup_index);
    // 两端点各 16 位，差值扩为 21 位以进行有符号移位加法。
    wire signed [16:0] traj_sin_diff =
        $signed({traj_sin1[15], traj_sin1}) -
        $signed({traj_sin0[15], traj_sin0});
    wire signed [20:0] traj_diff_ext = {{4{traj_sin_diff[16]}}, traj_sin_diff};
    wire signed [20:0] traj_interp_sum =
        (traj_fraction[0] ? traj_diff_ext         : 21'sd0) +
        (traj_fraction[1] ? (traj_diff_ext <<< 1) : 21'sd0) +
        (traj_fraction[2] ? (traj_diff_ext <<< 2) : 21'sd0) +
        (traj_fraction[3] ? (traj_diff_ext <<< 3) : 21'sd0);
    wire signed [31:0] traj_interp_ext =
        {{14{traj_interp_sin[17]}}, traj_interp_sin};

    // 逐级寄存，避免查表、插值、缩放合并成一条 50 MHz 长组合路径。
    // 轨迹发生器每个 ADC 样本最多使用 8 个系统时钟，远少于 1 ms。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            trajectory_origin        <= 32'sd0;
            trajectory_ref           <= 32'sd0;
            trajectory_phase         <= 12'd0;
            trajectory_pending_phase <= 12'd0;
            trajectory_stage         <= 3'd0;
            traj_sin0                <= 16'sd0;
            traj_sin1                <= 16'sd0;
            traj_fraction            <= 4'd0;
            traj_interp_product      <= 21'sd0;
            traj_interp_sin          <= 18'sd0;
            traj_scaled_q15          <= 32'sd0;
            traj_relative_count      <= 32'sd0;
        end else if (trajectory_start) begin
            // 每次启动 RUN4 均从当前位置、相位零点开始。
            trajectory_origin        <= encoder_position;
            trajectory_ref           <= encoder_position;
            trajectory_phase         <= 12'd0;
            trajectory_pending_phase <= 12'd0;
            trajectory_stage         <= 3'd0;
        end else if (!trajectory_mode || (control_mode != MODE_BAL) ||
                     output_fault_latched || command_out_of_range) begin
            // 停止、起摆、输出锁止或离开平衡区时均不推进相位，不改写目标。
            // 丢弃尚未完成的本采样计算，下一次平衡采样重新计算。
            trajectory_stage <= 3'd0;
        end else begin
            case (trajectory_stage)
                3'd0: if (adc_valid) begin
                    trajectory_pending_phase <= trajectory_phase + 12'd1;
                    trajectory_stage <= 3'd1;
                end
                3'd1: begin
                    // 第一个时钟读取左端点。
                    traj_sin0 <= traj_lookup_value;
                    traj_fraction <= trajectory_pending_phase[3:0];
                    trajectory_stage <= 3'd2;
                end
                3'd2: begin
                    // 第二个时钟读取右端点，复用同一个 ROM。
                    traj_sin1 <= traj_lookup_value;
                    trajectory_stage <= 3'd3;
                end
                3'd3: begin
                    traj_interp_product <= traj_interp_sum;
                    trajectory_stage <= 3'd4;
                end
                3'd4: begin
                    traj_interp_sin <=
                        $signed({{2{traj_sin0[15]}}, traj_sin0}) +
                        (traj_interp_product >>> 4);
                    trajectory_stage <= 3'd5;
                end
                3'd5: begin
                    // 100 = 64 + 32 + 4；均为移位/加法，不产生宽乘法器。
                    traj_scaled_q15 <=
                        (traj_interp_ext <<< 6) +
                        (traj_interp_ext <<< 5) +
                        (traj_interp_ext <<< 2);
                    trajectory_stage <= 3'd6;
                end
                3'd6: begin
                    // 正负对称四舍五入，Q15 转换为编码器整数计数。
                    if (traj_scaled_q15 >= 0)
                        traj_relative_count <=
                            (traj_scaled_q15 + 32'sd16384) >>> 15;
                    else
                        traj_relative_count <=
                            -(((-traj_scaled_q15) + 32'sd16384) >>> 15);
                    trajectory_stage <= 3'd7;
                end
                3'd7: begin
                    trajectory_phase <= trajectory_pending_phase;
                    trajectory_ref <= trajectory_origin + traj_relative_count;
                    trajectory_stage <= 3'd0;
                end
                default: trajectory_stage <= 3'd0;
            endcase
        end
    end

    // 将模式命令送到 PWM：RUN1/RUN2 用固定低速测试值，RUN3/RUN4 用反馈控制量。
    // 命令单位为 PWM 计数，正负号决定方向；RUN3用旧控制器命令，RUN4使用通过核验的verified_command。
    // 当前电机/传感器安装方向沿用此符号约定，此处不再额外取反。
    always @* begin
        if (!motor_enable) begin
            // 不仅停止标志清零命令，任何命令一致性故障也立即清零命令。
            drive_command = 16'sd0;
        end else case (run_mode)
            // 实测启动门槛：正向约 153/2500=6.12%，反向约 150/2500=6.00%。
            RUN_TEST_FWD: drive_command = 16'sd153;
            RUN_TEST_REV: drive_command = -16'sd150;
            // RUN3 原始控制器：不经过 RUN4 校验的强制清零，恢复旧版响应。
            // 仅屏蔽超出 1400 的非法命令，其他合法命令保持完全一致。
            RUN_BALANCE: drive_command = run3_safe_command;
            // RUN4 分拍控制器：保留原版输出核验和首次平衡使能逻辑。
            RUN_TRAJECTORY: drive_command = trajectory_drive_ready ? safe_verified_command : 16'sd0;
            default:      drive_command = 16'sd0;
        endcase
    end

    // MCP3201 以 SPI 读取角度传感器，默认采样率 1 kHz，SCLK 约 781.25 kHz。
    // 子模块在第 4~15 个 SCLK 上升沿采集 B11~B0，输出完整 12 位结果。
    // adc_valid 每完成一次采样时拉高一个系统时钟，下游用该脉冲更新数据。
    mcp3201_spi adc0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .adc_dout(adc_dout),
        .adc_cs_n(adc_cs_n),
        .adc_sclk(adc_sclk),
        .adc_data(adc_value),
        .adc_valid(adc_valid),
        .busy(adc_busy)
    );

    // 编码器先同步 AB 输入，再按合法的相邻状态变化累计有符号位置。
    // 速度按 ADC 采样节拍计算相邻位置差，并非已换算好的转速。
    // 测速和控制器在同一采样沿更新，控制器此时读取上一采样周期的速度寄存值。
    quadrature_encoder encoder0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .enc_a_i(encoder_a),
        .enc_b_i(encoder_b),
        .speed_tick(adc_valid),
        .position(encoder_position),
        .velocity(encoder_velocity)
    );

    // 各功能按键分别消抖：按 ADC 节拍连续采到 4 次低电平后确认按下。
    // 默认采样间隔 1 ms，press_pulse 仅保持一个系统时钟；长按不连续触发。
    // 顶层只使用按下事件，因此各实例的持续按下状态 pressed 留空。
    button_filter key0_filter (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .button_n(key0_n),
        .pressed(),
        .press_pulse(key0_pulse)
    );

    button_filter key1_filter (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .button_n(key1_n),
        .pressed(),
        .press_pulse(key1_pulse)
    );

    button_filter mode_start_filter (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .button_n(mode_start_n),
        .pressed(),
        .press_pulse(mode_start_pulse)
    );

    button_filter target_inc_filter (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .button_n(target_inc_n),
        .pressed(),
        .press_pulse(target_inc_pulse)
    );

    button_filter target_dec_filter (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .button_n(target_dec_n),
        .pressed(),
        .press_pulse(target_dec_pulse)
    );

    // 角度环当前使用 KP=9/2、KD=7、4 ms 差分，并加入 Ki=200/2^12 积分项。
    // 角度 PD 原始命令 = (9/2)*最短角度误差 + 7*当前与 4 个采样前 ADC 的最短差分。
    // 多采样差分用于减轻短周期量化噪声影响，未除以窗口长度或采样时间。
    // 历史保存 ADC 原值，避免校零偏置直接进入平衡 D 项；启动/起摆时会重填历史。
    // 正命令使编码器位置增加；串级位置环使用当前位置减目标位置及窗口位移。
    // 串级位置 PD 每 50 ms 更新，输出有界角度参考偏移，由角度内环跟踪。
    // 起摆与平衡的切换依据角度/变化率阈值，最终输出对称限幅至 +/-1400。
    // ================================================================
    // RUN3：用户上传 ZIP 的原始控制器，配合单独粘贴的顶层参数。
    // 实际使用 Kp(position)=13/32、Kd(position)=6。
    // 该模块的 414 行实现及内部控制状态不做任何改动。
    // ================================================================
    pendulum_controller #(
        .ANGLE_CAPTURE(600), // 误差绝对值不超过 600 且变化较慢时允许捕获。
        .ANGLE_RELEASE(1040), // 平衡时误差绝对值超过 1040 则退回起摆。
        .CAPTURE_RATE(320), // 捕获要求相邻采样的误差变化绝对值不超过 320。
        .KP_ANGLE(0), // 关闭整数比例参数，改用下面的 10/2 定点比例增益。
        .KP_ANGLE_NUM(9), // 比例项定点分子，9/2 = 4.5。
        .KP_ANGLE_SHIFT(1), // 比例项定点右移一位。
        .KD_ANGLE(7), // 平衡多采样差分增益。
        .KI_ANGLE(200), // 每 5 ms 更新一次时，对应参考工程 Ki=0.01 的 PWM 计数缩放。
        .KI_ANGLE_SHIFT(12), // 定点积分缩放；积分输出仍受独立限幅保护。
        .ANGLE_I_UPDATE_SAMPLES(5), // 与参考程序角度环每 5 ms 调用一次 PID 对齐。
        .ANGLE_I_LIMIT(600), // 积分项最多贡献 600 个 PWM 命令计数。
        .BALANCE_D_WINDOW(4), // 差分跨度为 4 个采样间隔，默认约 4 ms。
        .KP_POSITION(0), // 关闭旧的直接 PWM 位置比例项。
        .KD_POSITION(0), // 关闭旧的直接 PWM 速度项。
        .WEAK_POSITION_ENABLE(1), // 启用串级位置环。
        .POSITION_CASCADE_ENABLE(1), // 位置 PD 调整角度目标，不直接叠加 PWM 命令。
        .POSITION_UPDATE_SAMPLES(50), // 与参考程序相同，每 50 ms 更新位置环。
        .POSITION_KP_NUM(13),
        .POSITION_KP_SHIFT(5), // 13/32≈0.406，近似参考程序 Kp=0.4。
        .POSITION_KD_NUM(6),
        .POSITION_KD_SHIFT(0), // 窗口位置误差变化乘参考程序 Kd=4。
        .POSITION_FEEDBACK_SIGN(1), // 正位置偏差生成负角度目标偏移，匹配串级参考符号。
        .POSITION_OUTPUT_LIMIT(100), // 角度参考偏移限幅，与参考位置环输出范围一致。
        .POSITION_SLEW_LIMIT(8), // 每个 50 ms 窗口最多变化 8 个 ADC 角度计数。
        .POSITION_ANGLE_LIMIT(400), // 超过小角度区时清除外环及其测速历史。
        .SWING_GAIN(700), // 起摆分支输出幅值，单位为 PWM 计数。
        .SWING_SPEED_GAIN(4), // 起摆能量估算中的相邻采样差分系数。
        .SWING_TARGET_ENERGY(40_000_000), // 起摆目标能量，使用算法内部计数量纲。
        .COS_ZERO_CROSS(4096), // 起摆方向判断使用的角度计数阈值。
        .OUTPUT_LIMIT(1400), // 总命令限幅，包含平衡分支已加上的摩擦补偿。
        // 摩擦补偿只作用于平衡分支：零命令不加偏置，小命令按幅值逐渐加补偿。
        // 正负方向分别设置补偿上限，加补偿后再执行 OUTPUT_LIMIT 总限幅。
        .FRICTION_FWD(100), // 正向平衡输出的最大摩擦补偿。
        .FRICTION_REV(100), // 反向平衡输出的最大摩擦补偿。
        .FRICTION_RAMP_SHIFT(7) // 原始命令幅值达到 2^7=128 后加满补偿。
    ) controller_run3 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .enable(run3_balance_enable),
        .adc_value(adc_value),
        .angle_zero(angle_zero),
        .encoder_position(encoder_position),
        .position_target(target_position),
        .encoder_velocity(encoder_velocity),
        .command(run3_command),
        .angle_error(run3_angle_error),
        .control_mode(run3_control_mode),
        .debug_angle_pd(run3_debug_angle_pd),
        .debug_position_applied(run3_debug_position_applied),
        .debug_balance_rate(run3_debug_balance_rate),
        .debug_raw_control(run3_debug_raw_control),
        .debug_flags(run3_debug_flags),
        .debug_window_count(run3_debug_window_count),
        .debug_adc_glitch_count(run3_debug_adc_glitch_count),
        .debug_adc_max_delta(run3_debug_adc_max_delta),
        .debug_limit_events(run3_debug_limit_events)
    );

    // ================================================================
    // RUN4：前一版已经使用的分拍计算控制器，位置目标是轨迹发生器输出。
    // 实际使用 Kp(position)=12/32、Kd(position)=6。
    // 两个控制器彼此隔离，各自拥有完整的 PID/位置环/状态历史。
    // ================================================================
    pendulum_controller_run4 #(
        .ANGLE_CAPTURE(600), // 误差绝对值不超过 600 且变化较慢时允许捕获。
        .ANGLE_RELEASE(1040), // 平衡时误差绝对值超过 1040 则退回起摆。
        .CAPTURE_RATE(320), // 捕获要求相邻采样的误差变化绝对值不超过 320。
        .KP_ANGLE(0), // 关闭整数比例参数，改用下面的 9/2 定点比例增益。
        .KP_ANGLE_NUM(9), // 比例项定点分子，9/2 = 4.5。
        .KP_ANGLE_SHIFT(1), // 比例项定点右移一位。
        .KD_ANGLE(7), // 平衡多采样差分增益。
        .KI_ANGLE(200), // 每 5 ms 更新一次时，对应参考工程 Ki=0.01 的 PWM 计数缩放。
        .KI_ANGLE_SHIFT(12), // 定点积分缩放；积分输出仍受独立限幅保护。
        .ANGLE_I_UPDATE_SAMPLES(5), // 与参考程序角度环每 5 ms 调用一次 PID 对齐。
        .ANGLE_I_LIMIT(600), // 积分项最多贡献 600 个 PWM 命令计数。
        .BALANCE_D_WINDOW(4), // 差分跨度为 4 个采样间隔，默认约 4 ms。
        .KP_POSITION(0), // 关闭旧的直接 PWM 位置比例项。
        .KD_POSITION(0), // 关闭旧的直接 PWM 速度项。
        .WEAK_POSITION_ENABLE(1), // 启用串级位置环。
        .POSITION_CASCADE_ENABLE(1), // 位置 PD 调整角度目标，不直接叠加 PWM 命令。
        .POSITION_UPDATE_SAMPLES(50), // 与参考程序相同，每 50 ms 更新位置环。
        .POSITION_KP_NUM(12), // 保留最近实际使用顶层的参数：Kp = 12/32。
        .POSITION_KP_SHIFT(5), // 位置环比例项右移 5 位，参数不在轨迹层内调整。
        .POSITION_KD_NUM(6), // 保留最近实际使用顶层的参数：Kd = 6。
        .POSITION_KD_SHIFT(0), // 窗口位置误差变化的缩放参数不变。
        .POSITION_FEEDBACK_SIGN(1), // 正位置偏差生成负角度目标偏移，匹配串级参考符号。
        .POSITION_OUTPUT_LIMIT(100), // 角度参考偏移限幅，与参考位置环输出范围一致。
        .POSITION_SLEW_LIMIT(8), // 每个 50 ms 窗口最多变化 8 个 ADC 角度计数。
        .POSITION_ANGLE_LIMIT(400), // 超过小角度区时清除外环及其测速历史。
        .SWING_GAIN(700), // 起摆分支输出幅值，单位为 PWM 计数。
        .SWING_SPEED_GAIN(4), // 起摆能量估算中的相邻采样差分系数。
        .SWING_TARGET_ENERGY(40_000_000), // 起摆目标能量，使用算法内部计数量纲。
        .COS_ZERO_CROSS(4096), // 起摆方向判断使用的角度计数阈值。
        .OUTPUT_LIMIT(1400), // 总命令限幅，包含平衡分支已加上的摩擦补偿。
        // 摩擦补偿只作用于平衡分支：零命令不加偏置，小命令按幅值逐渐加补偿。
        // 正负方向分别设置补偿上限，加补偿后再执行 OUTPUT_LIMIT 总限幅。
        .FRICTION_FWD(100), // 正向平衡输出的最大摩擦补偿。
        .FRICTION_REV(100), // 反向平衡输出的最大摩擦补偿。
        .FRICTION_RAMP_SHIFT(7) // 原始命令幅值达到 2^7=128 后加满补偿。
    ) controller_run4 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .enable(run4_balance_enable),
        .adc_value(adc_value),
        .angle_zero(angle_zero),
        .encoder_position(encoder_position),
        .position_target(trajectory_ref),
        .encoder_velocity(encoder_velocity),
        .command(run4_command),
        .angle_error(run4_angle_error),
        .control_mode(run4_control_mode),
        .debug_angle_pd(run4_debug_angle_pd),
        .debug_position_applied(run4_debug_position_applied),
        .debug_balance_rate(run4_debug_balance_rate),
        .debug_raw_control(run4_debug_raw_control),
        .debug_flags(run4_debug_flags),
        .debug_window_count(run4_debug_window_count),
        .debug_adc_glitch_count(run4_debug_adc_glitch_count),
        .debug_adc_max_delta(run4_debug_adc_max_delta),
        .debug_limit_events(run4_debug_limit_events)
    );

    // PWM_PERIOD=2500：50 MHz / 2500 = 20 kHz，|命令|/2500 为驱动占空比。
    // 正命令：IN1 输出低有效 PWM、IN2 保持高；负命令时两通道对调。
    // 未使能或零命令时两输入均输出低；DEAD_BAND=0 表示不额外设置命令死区。
    motor_pwm_dual #(
        .PWM_PERIOD(2_500),
        .DEAD_BAND(0)
    ) motor0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .enable(motor_enable),
        .command(drive_command),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2)
    );

    // 串口仅发送遥测数据，不接收控制命令；默认 115200 波特率、8N1。
    // 本版使用55 AB诊断帧：输入快照于 adc_valid 锁存，控制输出分拍完成后触发发送。
    // 保持原空闲计时10 ms，发送44字节另需约3.82 ms；上位机同时兼容旧55 AA帧。
    // command 上报 drive_command，即模式选择后的 PWM 输入命令；暂停后 run_mode 仍保留。
    // 检测到异常样本时上报零命令；LED3 常亮表示发生过拒绝，LED2 灭表示已锁止。
    uart_telemetry_tx #(.DIAGNOSTIC_ENABLE(1)) uart0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .adc_value(telemetry_adc),
        .encoder_position(telemetry_position),
        .encoder_velocity({{16{telemetry_velocity[15]}}, telemetry_velocity}),
        .command(drive_command),
        .control_mode(control_mode),
        .run_mode(run_mode),
        .diagnostic_tick(telemetry_tick),
        .sample_id(telemetry_sample_id),
        .angle_zero(telemetry_zero),
        .angle_error(angle_error),
        .position_target(telemetry_target),
        .angle_pd(debug_angle_pd),
        .position_applied(debug_position_applied),
        .balance_rate(debug_balance_rate),
        .raw_control(debug_raw_control),
        .diagnostic_flags({motor_enable, run_active, debug_flags[5:0]}),
        .window_count(debug_window_count),
        .adc_glitch_count(debug_adc_glitch_count),
        .adc_max_delta(debug_adc_max_delta),
        .limit_events(debug_limit_events),
        .uart_tx(uart_tx)
    );

    // OLED 每 0.5 秒收到一次刷新请求，显示模式/启停、ADC、角度误差、速度和控制器命令。
    // 此处显示 control_command；RUN1/RUN2 的固定测试命令由 UART 上报 drive_command。
    // 编码器速度只取低 16 位送显示，未在顶层执行单位换算。
    oled_ssd1306 oled0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .refresh_tick(display_tick),
        .display_mode(display_mode),
        .run_active(run_active),
        .adc_value(adc_value),
        .angle_error(angle_error),
        .encoder_velocity(encoder_velocity[15:0]),
        .control_command(control_command),
        .scl_drive_low(oled_scl_low),
        .sda_drive_low(oled_sda_low),
        .sda_i(oled_sda)
    );

    // I2C 为开漏方式：逻辑 0 时 FPGA 拉低，逻辑 1 时释放为高阻，由上拉电阻拉高。
    assign oled_scl = oled_scl_low ? 1'b0 : 1'bz;
    assign oled_sda = oled_sda_low ? 1'b0 : 1'bz;

    // 顶层时序控制：模式选择/启动暂停、目标位置调整、角度校零及状态指示。
    // 以下寄存器在 50 MHz 上升沿更新，复位下降沿立即进入默认停机状态。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            // 3153 是当前实测直立参考值；放正摆杆后仍可按 KEY1 重新记录。
            angle_zero       <= 12'd3153;
            run_mode         <= RUN_SAFE;
            selected_mode    <= RUN_SAFE;
            run_active       <= 1'b0;
            target_position  <= 32'sd0;
            heartbeat        <= 24'd0;
            adc_activity     <= 1'b0;
            encoder_activity <= 1'b0;
            display_count    <= 25'd0;
            display_tick     <= 1'b0;
            telemetry_tick <= 1'b0;
            telemetry_delay <= 11'd0;
            telemetry_guard_delay <= 10'd0;
            telemetry_sample_id <= 32'd0;
            telemetry_adc <= 12'd0;
            telemetry_zero <= 12'd3153;
            telemetry_position <= 32'sd0;
            telemetry_velocity <= 16'sd0;
            telemetry_target <= 32'sd0;
        end else begin
            // 采样值在ADC有效沿锁存；旧RUN3在ADC沿更新命令，新RUN4约11拍后提交。
            // RUN4保护读取第11拍后的结果，再进行6拍计算后更新verified_command。
            // UART再延迟10拍读取完整结果，使最终PWM命令和内部诊断字段同属本采样。
            telemetry_delay <= {telemetry_delay[9:0], adc_valid};
            telemetry_guard_delay <= {telemetry_guard_delay[8:0], guard_sample_tick};
            telemetry_tick <= telemetry_guard_delay[9];
            if (adc_valid) begin
                telemetry_sample_id <= telemetry_sample_id + 1'b1;
                telemetry_adc <= adc_value;
                telemetry_zero <= angle_zero;
                telemetry_position <= encoder_position;
                telemetry_velocity <= encoder_velocity[15:0];
                telemetry_target <= controller_position_target;
            end
            // 心跳连续运行，与电机是否启动无关。
            heartbeat <= heartbeat + 1'b1;
            // 刷新请求默认清零；50 MHz 下计满 25,000,000 拍即为 0.5 秒。
            display_tick <= 1'b0;
            if (display_count == 25'd24_999_999) begin
                display_count <= 25'd0;
                display_tick  <= 1'b1;
            end else begin
                display_count <= display_count + 1'b1;
            end
            // ADC 状态每个样本翻转；编码器状态仅在采样时速度非零才翻转。
            if (adc_valid) begin
                adc_activity <= ~adc_activity;
                if (encoder_velocity != 0)
                    encoder_activity <= ~encoder_activity;
            end
            // KEY0 只在停止/暂停时选模式：SAFE -> 正向 -> 反向 -> RUN3 -> RUN4 -> SAFE。
            if (key0_pulse && !run_active) begin
                case (selected_mode)
                    RUN_SAFE:     selected_mode <= RUN_TEST_FWD;
                    RUN_TEST_FWD: selected_mode <= RUN_TEST_REV;
                    RUN_TEST_REV: selected_mode <= RUN_BALANCE;
                    RUN_BALANCE:  selected_mode <= RUN_TRAJECTORY;
                    default:      selected_mode <= RUN_SAFE;
                endcase
            end

            // 启动/暂停键：运行中按下即暂停；停止时按 selected_mode 启动。
            // 停止时若选模式和启动脉冲同拍到达，只执行选模式，电机继续保持停止。
            // SAFE 模式下按启动仍停机；暂停保留 run_mode、selected_mode 和目标位置。
            if (mode_start_pulse && (run_active || !key0_pulse)) begin
                if (run_active) begin
                    run_active <= 1'b0;
                end else if (selected_mode == RUN_SAFE) begin
                    run_mode   <= RUN_SAFE;
                    run_active <= 1'b0;
                // RUN4 保持之前的直立附近启动条件，避免倾斜过大时直接启动。
                end else if ((selected_mode != RUN_TRAJECTORY) || trajectory_start) begin
                    run_mode   <= selected_mode;
                    run_active <= 1'b1;
                    // 上次运行模式不是 RUN3 时，以当前位置初始化新一轮 RUN3 的目标。
                    // 从 RUN3 暂停后直接恢复时不满足此条件，保留已有目标。
                    if ((selected_mode == RUN_BALANCE) &&
                        (run_mode != RUN_BALANCE))
                        target_position <= encoder_position;
                end
            end

            // 选择模式或上次运行模式为 RUN3 时，允许每次按键调整 100 个编码器计数。
            // 加减事件同拍出现时不修改目标；目标经串级位置环换算为角度参考偏移。
            // 若与启动初始化同拍写目标，下方按键赋值在本 always 块中优先。
            if ((selected_mode == RUN_BALANCE) ||
                (run_mode == RUN_BALANCE)) begin
                if (target_inc_pulse && !target_dec_pulse)
                    target_position <= target_position + POSITION_STEP;
                else if (target_dec_pulse && !target_inc_pulse)
                    target_position <= target_position - POSITION_STEP;
            end
            // KEY1 只在停止/暂停时锁存最近的 ADC 值为直立零点，避免运行时改变平衡基准。
            // angle_error 在控制器后续采样节拍中按新零点更新，此处不清零编码器位置。
            // RUN3恢复用户原版行为：允许运行时KEY1校零；RUN4仍只允许停机时校零。
            // 注意：运行中校零本身可能改变平衡参考，实际测试建议先暂停。
            if (key1_pulse && (!run_active || (run_mode == RUN_BALANCE)))
                angle_zero <= adc_value;
        end
    end

    assign led[0] = heartbeat[23]; // 50 MHz 下每约 168 ms 翻转一次，观察主时钟运行。
    assign led[1] = adc_activity; // ADC 采样活动翻转信号，并非采样有效脉冲本身。
    assign led[2] = motor_enable; // 电机驱动使能状态，为 1 时允许 PWM 输出。
    // RUN3恢复原版LED3编码器活动；RUN4仍用LED3提示发生过拒绝或锁止。
    assign led[3] = ((run_mode == RUN_TRAJECTORY) &&
                     (output_fault_latched || guard_reject_seen)) ?
                    1'b1 : encoder_activity;

endmodule
