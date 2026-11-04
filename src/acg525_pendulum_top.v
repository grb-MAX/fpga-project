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
    // RUN3角度PID/增益不变：自动起摆只在模式3从下垂启动时接管电机。
    // 手扶直立仍直通原位置环；仅自动起摆把START处定义为-330，目标从START起保持为0。
    wire run3_auto_pending;
    wire run3_auto_started;
    wire run3_auto_fault_latched;
    wire run3_auto_pid_ready;
    wire signed [15:0] run3_swing_command;
    wire signed [31:0] run3_swing_raw;
    wire run3_swing_active, run3_swing_fault;
    wire run3_swing_result_valid; // 独立起摆计算完成脉冲，避免读取尚未提交的结果。
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

    // UART、OLED、轨迹平衡门控均使用当前模式对应的控制器信号。
    wire is_run4_selected = (run_mode == RUN_TRAJECTORY);
    // OLED和UART应反映电机真正采用的RUN3输出（起摆/原PID）。
    wire signed [15:0] control_command = is_run4_selected ? run4_command :
        ((run_mode == RUN_BALANCE) ? run3_effective_command : run3_command);
    wire signed [15:0] angle_error = is_run4_selected ? run4_angle_error : run3_angle_error;
    wire [1:0] control_mode = is_run4_selected ? run4_control_mode :
        ((run_mode == RUN_BALANCE && run3_auto_pending) ? 2'd1 :
         ((run_mode == RUN_BALANCE && run3_auto_fault_latched) ? 2'd0 : run3_control_mode));
    wire signed [31:0] debug_angle_pd = is_run4_selected ? run4_debug_angle_pd : run3_debug_angle_pd;
    wire signed [31:0] debug_position_applied = is_run4_selected ? run4_debug_position_applied : run3_debug_position_applied;
    wire signed [15:0] debug_balance_rate = is_run4_selected ? run4_debug_balance_rate : run3_debug_balance_rate;
    wire signed [31:0] debug_raw_control = is_run4_selected ? run4_debug_raw_control :
        ((run_mode == RUN_BALANCE && run3_auto_pending) ? run3_swing_raw : run3_debug_raw_control);
    wire [7:0] debug_flags = is_run4_selected ? run4_debug_flags : run3_debug_flags;
    wire [7:0] debug_window_count = is_run4_selected ? run4_debug_window_count : run3_debug_window_count;
    wire [31:0] debug_adc_glitch_count = is_run4_selected ? run4_debug_adc_glitch_count : run3_debug_adc_glitch_count;
    wire [31:0] debug_adc_max_delta = is_run4_selected ? run4_debug_adc_max_delta : run3_debug_adc_max_delta;
    wire [31:0] debug_limit_events = is_run4_selected ? run4_debug_limit_events : run3_debug_limit_events;
    reg telemetry_tick; // UART在RUN4校验完成后取本采样控制数据，避免新旧命令混帧。
    reg [11:0] telemetry_delay; // 12拍延迟：RUN4在S_COMMIT提交后再读取诊断和命令。
    reg [9:0] telemetry_guard_delay; // 进一步延迟10拍供UART使用，确保verified_command已更新。
    wire guard_sample_tick = telemetry_delay[11]; // RUN4分拍控制器提交后的读取节拍。
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
    reg [2:0]  run_mode; // 已启动的模式；暂停时保留该编号。
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
    localparam [2:0] RUN_TRAJECTORY = 3'd4; // 独立分拍RUN4：梯形/S曲线/正弦轨迹跟踪。
    localparam integer POSITION_STEP = 32'sd100; // 每次目标位置按键的步长。

    // 只有启动 RUN3/RUN4 才使能控制器；RUN1/RUN2 绕过控制器，使用固定测试命令。
    // motor_enable 表示驱动允许输出，并不代表命令一定非零或电机一定正在转动。
    wire trajectory_mode = run_active && (run_mode == RUN_TRAJECTORY);
    wire run3_balance_enable = run_active && (run_mode == RUN_BALANCE);
    wire run4_balance_enable = trajectory_mode;

    // ================================================================
    // RUN3：自动起摆只作为同一模式的启动阶段，不增加模式5。
    // 进入模式3时，如果摆杆已位于原控制器捕获区(±600)，直接运行原PID。
    // 只有从较大倾角启动，才暂时使用独立起摆；其余原RUN3逻辑不受影响。
    // 原控制器在起摆阶段禁用，防止旧位置目标、积分和D历史提前积累。
    // ================================================================
    wire run3_auto_start = mode_start_pulse && !run_active && !key0_pulse &&
                            (selected_mode == RUN_BALANCE);
    wire signed [12:0] run3_auto_angle =
        $signed({1'b0,adc_value}) - $signed({1'b0,angle_zero});
    wire [12:0] run3_auto_abs_angle = (run3_auto_angle < 0) ?
                                     -run3_auto_angle : run3_auto_angle;
    // 与本次基准controller_run3的ANGLE_CAPTURE(600)一致。
    wire run3_auto_start_from_down = (run3_auto_abs_angle > 13'd600);
    wire run3_controller_enable = run3_balance_enable &&
                                  !run3_auto_pending && !run3_auto_fault_latched;
    // 手扶直立启动仍用原始编码器坐标；自动起摆路径独立使用START-relative坐标。
    // 自动起摆：首次进入BAL就启用位置外环，不再要求START后正向位移300。
    // SWING期间关闭；原控制器内部仅在连续BAL时每50个ADC样本更新一次位置PD。
    // BAL退出或故障后关断；手扶直立启动仍由原控制器BAL条件控制。
    wire run3_position_loop_enable = !run3_auto_started_r ||
        (!run3_auto_pending_r && run3_auto_capture_seen_r &&
         (run3_control_mode == 2'd2) && !run3_auto_fault_r);
    // 自动起摆时持续收集角度ADC历史，供首次进入BAL的D项使用。
    // 与位置外环使能独立；手扶直立启动保持原角度历史复位方式。
    wire run3_angle_history_track = run3_balance_enable &&
        run3_auto_started_r && !run3_auto_fault_r;

    reg run3_auto_pending_r, run3_auto_started_r, run3_auto_fault_r;
    reg run3_auto_pid_seen;
    // 仅自动起摆路径使用：捕获后等待原控制器首次真正进入BAL。
    // 手扶直立直接启动不触发自动目标更新，原RUN3位置环始终使能。
    reg run3_auto_capture_seen_r;
    reg run3_auto_center_latched_r;
    reg [1:0] run3_auto_good_samples;
    reg [11:0] run3_auto_last_adc;
    reg run3_auto_sample_seen;
    reg signed [31:0] run3_auto_fixed_origin;

    // RUN3自动路径的坐标原点定义：按下START瞬间位置为 -330。
    // raw = START处编码器原始值时，relative = raw - origin - 330 = -330。
    // 此后沿编码器正方向移动330个计数，相对位置恰好为0。
    // 注意：这是坐标变换，绝不向起摆控制器下达“移动330”的命令。
    // 手扶直立启动RUN3及RUN4继续使用原始绝对编码器坐标，不受影响。
    localparam signed [31:0] RUN3_AUTO_START_COORD = -32'sd405;
    wire run3_auto_coordinate_active =
        (run_mode == RUN_BALANCE) && run3_auto_started_r;
    wire signed [31:0] run3_auto_coordinate_position =
        encoder_position - run3_auto_fixed_origin + RUN3_AUTO_START_COORD;
    wire signed [31:0] run3_controller_position =
        run3_auto_coordinate_active ? run3_auto_coordinate_position : encoder_position;

    assign run3_auto_pending = run3_auto_pending_r;
    assign run3_auto_started = run3_auto_started_r;
    assign run3_auto_fault_latched = run3_auto_fault_r;

    wire signed [13:0] run3_auto_rate =
        $signed({1'b0,adc_value}) - $signed({1'b0,run3_auto_last_adc});
    wire [13:0] run3_auto_abs_rate = (run3_auto_rate < 0) ?
                                    -run3_auto_rate : run3_auto_rate;

    // 两个连续的有效样本满足 ±80、相邻差分≤6 才进入原平衡控制。
    // 这是针对已经实测可以起摆的捕获条件，不修改原PID捕获/释放参数。
    wire run3_auto_good_sample = run3_auto_sample_seen &&
        (run3_auto_abs_angle <= 13'd80) && (run3_auto_abs_rate <= 14'd6);
    wire run3_auto_capture = run3_balance_enable && run3_auto_pending_r &&
        !run3_auto_fault_r && adc_valid && run3_auto_good_sample &&
        (run3_auto_good_samples >= 2'd1);

    // 自动起摆捕获后：原角度PID进入BAL时，位置外环同步获得使能。
    // START起位置目标始终为0；首次BAL只锁存状态并再次确认目标0。
    // 位置环在首次BAL开始累计50个ADC样本，之后每50ms更新一次位置PD。
    localparam signed [31:0] RUN3_AUTO_BALANCE_TARGET = 32'sd0;
    wire run3_auto_first_balance = run3_balance_enable && run3_auto_started_r &&
        !run3_auto_pending_r && !run3_auto_fault_r &&
        run3_auto_capture_seen_r && !run3_auto_center_latched_r &&
        (run3_control_mode == 2'd2) &&
        !target_inc_pulse && !target_dec_pulse;

    // 固定起摆起点，不在成功捕获后不断更新，避免多次起摆时坐标原点漂移。
    // 安全行程判断仍在原始编码器坐标内计算相对START的位移，不受坐标显示变换影响。
    wire signed [32:0] run3_auto_delta =
        $signed({encoder_position[31],encoder_position}) -
        $signed({run3_auto_fixed_origin[31],run3_auto_fixed_origin});

    // 编码器速度约为计数/ms，向前估计8ms；SWING与BAL共用此预测。
    // 软件限位只保护自动起摆路径，手扶直立RUN3和RUN4完全不受影响。
    wire signed [36:0] run3_auto_predicted =
        $signed({{4{run3_auto_delta[32]}},run3_auto_delta}) +
        ($signed({{5{encoder_velocity[31]}},encoder_velocity}) <<< 3);
    // RUN3取消原±350软限位、±500硬限位：编译期恒为0，两处限位均不参与PWM门控。
    // 不影响PWM幅值核验、故障锁止、BAL退出保护及起摆模块自身的ARM_TRAVEL_LIMIT。
    // 恢复这两级限位时才改为1；没有独立硬件限位/急停时不可带动力电测试。
    localparam RUN3_AUTO_TRAVEL_LIMITS_ENABLE = 1'b0;
    localparam signed [36:0] RUN3_AUTO_SOFT_LIMIT = 37'sd350;
    localparam signed [32:0] RUN3_AUTO_HARD_LIMIT = 33'sd500;
    wire run3_auto_hard_reached = RUN3_AUTO_TRAVEL_LIMITS_ENABLE &&
        ((run3_auto_delta >= RUN3_AUTO_HARD_LIMIT) ||
         (run3_auto_delta <= -RUN3_AUTO_HARD_LIMIT));
    wire run3_auto_soft_outward = RUN3_AUTO_TRAVEL_LIMITS_ENABLE &&
        (((run3_auto_predicted >= RUN3_AUTO_SOFT_LIMIT) && (run3_swing_command > 0)) ||
         ((run3_auto_predicted <= -RUN3_AUTO_SOFT_LIMIT) && (run3_swing_command < 0)));

    // 只有自动起摆完成后，才需要检查RUN3输出确实是已计算好的BAL采样。
    // 手动扶直启动时这个门控被完全旁路，保持原RUN3响应。
    assign run3_auto_pid_ready = !run3_auto_started_r ||
        (!run3_auto_pending_r && run3_auto_capture_seen_r &&
         (run3_control_mode == 2'd2) &&
         run3_debug_flags[2] && run3_debug_flags[5]);

    wire signed [15:0] run3_effective_command =
        (run3_auto_fault_r || (run3_auto_started_r && run3_auto_hard_reached)) ? 16'sd0 :
        (run3_auto_pending_r ?
           ((run3_swing_active && !run3_auto_soft_outward) ?
             run3_swing_command : 16'sd0) :
           (run3_auto_pid_ready ? run3_safe_command : 16'sd0));

    // 起摆与原控制器的单向交接：一旦捕获成功，不允许再次自动起摆。
    // 若接管后再次退出BAL，锁止本次自动运行，避免重复大角度冲出行程。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            run3_auto_pending_r <= 1'b0;
            run3_auto_started_r <= 1'b0;
            run3_auto_fault_r <= 1'b0;
            run3_auto_pid_seen <= 1'b0;
            run3_auto_capture_seen_r <= 1'b0;
            run3_auto_center_latched_r <= 1'b0;
            run3_auto_good_samples <= 2'd0;
            run3_auto_last_adc <= 12'd0;
            run3_auto_sample_seen <= 1'b0;
            run3_auto_fixed_origin <= 32'sd0;
        end else if (run3_auto_start) begin
            run3_auto_pending_r <= run3_auto_start_from_down;
            run3_auto_started_r <= run3_auto_start_from_down;
            run3_auto_fault_r <= 1'b0;
            run3_auto_pid_seen <= 1'b0;
            run3_auto_capture_seen_r <= 1'b0;
            run3_auto_center_latched_r <= 1'b0;
            run3_auto_good_samples <= 2'd0;
            run3_auto_sample_seen <= 1'b0;
            run3_auto_last_adc <= adc_value;
            run3_auto_fixed_origin <= encoder_position;
        end else if (!run3_balance_enable) begin
            run3_auto_pending_r <= 1'b0;
            run3_auto_started_r <= 1'b0;
            run3_auto_fault_r <= 1'b0;
            run3_auto_pid_seen <= 1'b0;
            run3_auto_capture_seen_r <= 1'b0;
            run3_auto_center_latched_r <= 1'b0;
            run3_auto_good_samples <= 2'd0;
            run3_auto_sample_seen <= 1'b0;
        end else begin
            // 捕获当拍只记录状态；位置目标在START已设为0，首次BAL时再次确认0。
            if (run3_auto_capture)
                run3_auto_capture_seen_r <= 1'b1;
            // 首次BAL锁存状态；位置环已由run3_control_mode==BAL直接使能。
            if (run3_auto_first_balance)
                run3_auto_center_latched_r <= 1'b1;
            if (adc_valid) begin
                run3_auto_last_adc <= adc_value;
                run3_auto_sample_seen <= 1'b1;
            end
            // 原±500硬限位已由RUN3_AUTO_TRAVEL_LIMITS_ENABLE=0在编译期禁用。
            // 起摆故障与BAL退出故障仍需保持锁止。
            if ((run3_auto_started_r && run3_auto_hard_reached) ||
                (run3_auto_pending_r && run3_swing_fault))
                run3_auto_fault_r <= 1'b1;
            if (run3_auto_pending_r && adc_valid) begin
                if (run3_auto_good_sample) begin
                    if (run3_auto_capture) begin
                        run3_auto_pending_r <= 1'b0;
                        run3_auto_good_samples <= 2'd0;
                    end else if (run3_auto_good_samples != 2'd3)
                        run3_auto_good_samples <= run3_auto_good_samples + 2'd1;
                end else
                    run3_auto_good_samples <= 2'd0;
            end
            if (run3_auto_started_r && !run3_auto_pending_r && adc_valid) begin
                if (run3_auto_pid_ready)
                    run3_auto_pid_seen <= 1'b1;
                else if (run3_auto_pid_seen && (run3_control_mode != 2'd2))
                    run3_auto_fault_r <= 1'b1;
            end
        end
    end


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
            // RUN4校验状态独立：退出RUN4清零；RUN3使用另一套校验状态。
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

    // ================================================================
    // RUN3 独立输出一致性校验：原控制器/独立起摆 -> 多拍校验 -> PWM。
    // 仅验证并隔离异常输出，不修改原角度PID、位置环、起摆及RUN4参数。
    // 与RUN4一样先等待计算结果提交：BAL使用原12拍延迟，
    // SWING使用起摆模块的result_valid再延后一拍，禁止过早快照。
    // ADC采样时清零旧的已校验命令，未核验前PWM保持0。
    // 不允许未经核验的RUN3命令直接进入motor0。
    // ================================================================
    reg run3_swing_result_valid_delay; // 起摆result_valid之后再等一拍读取输出。
    reg [2:0] run3_guard_stage;
    reg signed [31:0] run3_guard_raw_snapshot;
    reg signed [31:0] run3_guard_pd_snapshot;
    reg signed [15:0] run3_guard_command_snapshot;
    reg run3_guard_bal_snapshot;
    reg run3_guard_allowed_snapshot;
    reg run3_guard_pid_initialized_snapshot;
    reg run3_guard_math_invalid;
    reg signed [31:0] run3_guard_abs_raw;
    reg signed [31:0] run3_guard_friction;
    reg signed [31:0] run3_guard_corrected;
    reg signed [15:0] run3_guard_expected;
    reg signed [15:0] run3_verified_command;
    reg [1:0] run3_guard_bad_streak;
    reg run3_guard_reject_seen;
    reg run3_output_fault_latched;

    // 独立起摆的输出没有摩擦补偿；PID平衡分支使用 +/-109 补偿。
    // 被起摆行程保护或尚未完成PID接管而主动屏蔽的命令不参与一致性判错。
    wire run3_guard_swing_source = run3_auto_pending_r;
    // BAL控制器与RUN4共用“ADC后12拍”的快照节拍；
    // 独立SWING起摆模块只在result_valid之后允许快照。
    wire run3_guard_capture_tick = run3_guard_swing_source ?
        run3_swing_result_valid_delay : guard_sample_tick;
    wire run3_guard_output_allowed = run3_guard_swing_source ?
        (!run3_auto_fault_r && !run3_auto_hard_reached &&
         run3_swing_active && !run3_auto_soft_outward) :
        (!run3_auto_fault_r &&
         !(run3_auto_started_r && run3_auto_hard_reached) &&
         run3_auto_pid_ready);
    wire signed [31:0] run3_guard_source_raw = run3_guard_swing_source ?
        run3_swing_raw : run3_debug_raw_control;
    wire signed [15:0] run3_guard_source_command = run3_guard_swing_source ?
        run3_swing_command : run3_command;
    wire run3_guard_source_bal = !run3_guard_swing_source && run3_debug_flags[2];
    // 这里用原控制器的前拍使能位，不使用顶层BAL状态做推测。
    wire run3_guard_pid_initialized = run3_guard_swing_source || run3_debug_flags[5];

    // 独立重建控制器的输出算式；积分限幅600，摩擦补偿109，PWM限幅1400。
    // 该校验与RUN4相互独立，绝不复用其状态和verified_command。
    wire signed [31:0] run3_guard_command_delta =
        $signed({{16{run3_guard_command_snapshot[15]}},run3_guard_command_snapshot}) -
        $signed({{16{run3_guard_expected[15]}},run3_guard_expected});
    wire run3_guard_integral_invalid = run3_guard_bal_snapshot &&
        ((run3_guard_raw_snapshot > run3_guard_pd_snapshot + 32'sd600) ||
         (run3_guard_raw_snapshot < run3_guard_pd_snapshot - 32'sd600));
    wire run3_guard_raw_invalid =
        (run3_guard_raw_snapshot > 32'sd50000) ||
        (run3_guard_raw_snapshot < -32'sd50000) ||
        (run3_guard_bal_snapshot &&
         ((run3_guard_pd_snapshot > 32'sd50000) ||
          (run3_guard_pd_snapshot < -32'sd50000)));

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            run3_swing_result_valid_delay <= 1'b0;
            run3_guard_stage <= 3'd0;
            run3_guard_raw_snapshot <= 32'sd0;
            run3_guard_pd_snapshot <= 32'sd0;
            run3_guard_command_snapshot <= 16'sd0;
            run3_guard_bal_snapshot <= 1'b0;
            run3_guard_allowed_snapshot <= 1'b0;
            run3_guard_pid_initialized_snapshot <= 1'b0;
            run3_guard_math_invalid <= 1'b0;
            run3_guard_abs_raw <= 32'sd0;
            run3_guard_friction <= 32'sd0;
            run3_guard_corrected <= 32'sd0;
            run3_guard_expected <= 16'sd0;
            run3_verified_command <= 16'sd0;
            run3_guard_bad_streak <= 2'd0;
            run3_guard_reject_seen <= 1'b0;
            run3_output_fault_latched <= 1'b0;
        end else if (!run3_balance_enable) begin
            // 离开RUN3或暂停时清零，下一次START须重新逐采样核验。
            run3_swing_result_valid_delay <= 1'b0;
            run3_guard_stage <= 3'd0;
            run3_verified_command <= 16'sd0;
            run3_guard_bad_streak <= 2'd0;
            run3_guard_reject_seen <= 1'b0;
            run3_output_fault_latched <= 1'b0;
        end else if (run3_output_fault_latched) begin
            // 第三个连续不合格样本后锁存停机，直到暂停或KEY2复位。
            run3_swing_result_valid_delay <= 1'b0;
            run3_guard_stage <= 3'd0;
            run3_verified_command <= 16'sd0;
        end else begin
            // 结果有效信号只用于起摆；平衡使用与RUN4相同的延迟采样节拍。
            run3_swing_result_valid_delay <= run3_swing_result_valid;
            if (adc_valid)
                run3_verified_command <= 16'sd0; // 未完成本采样核验前保持PWM为0

            if ((run3_guard_stage == 3'd0) && run3_guard_capture_tick) begin
                // BAL：原控制器已经在ADC沿更新，延后12拍保证数据稳定。
                // SWING：result_valid已发生，延后一拍再读取active/raw/command。
                run3_guard_raw_snapshot <= run3_guard_source_raw;
                run3_guard_pd_snapshot <= run3_debug_angle_pd;
                run3_guard_command_snapshot <= run3_guard_source_command;
                run3_guard_bal_snapshot <= run3_guard_source_bal;
                run3_guard_allowed_snapshot <= run3_guard_output_allowed;
                run3_guard_pid_initialized_snapshot <= run3_guard_pid_initialized;
                run3_guard_stage <= 3'd1;
            end else begin
                case (run3_guard_stage)
                    3'd1: begin
                        run3_guard_math_invalid <= run3_guard_integral_invalid ||
                                                   run3_guard_raw_invalid;
                        // 即便原始数值溢出，也在最后的invalid条件下拒绝。
                        run3_guard_abs_raw <= (run3_guard_raw_snapshot < 0) ?
                                              -run3_guard_raw_snapshot : run3_guard_raw_snapshot;
                        run3_guard_stage <= 3'd2;
                    end
                    3'd2: begin
                        if (!run3_guard_bal_snapshot)
                            run3_guard_friction <= 32'sd0;
                        else if (run3_guard_abs_raw >= 32'sd128)
                            run3_guard_friction <= 32'sd109;
                        else
                            run3_guard_friction <= (run3_guard_abs_raw * 32'sd109) >>> 7;
                        run3_guard_stage <= 3'd3;
                    end
                    3'd3: begin
                        if (run3_guard_bal_snapshot && (run3_guard_raw_snapshot > 0))
                            run3_guard_corrected <= run3_guard_raw_snapshot + run3_guard_friction;
                        else if (run3_guard_bal_snapshot && (run3_guard_raw_snapshot < 0))
                            run3_guard_corrected <= run3_guard_raw_snapshot - run3_guard_friction;
                        else
                            run3_guard_corrected <= run3_guard_raw_snapshot;
                        run3_guard_stage <= 3'd4;
                    end
                    3'd4: begin
                        if (run3_guard_corrected > 32'sd1400)
                            run3_guard_expected <= 16'sd1400;
                        else if (run3_guard_corrected < -32'sd1400)
                            run3_guard_expected <= -16'sd1400;
                        else
                            run3_guard_expected <= run3_guard_corrected[15:0];
                        run3_guard_stage <= 3'd5;
                    end
                    3'd5: begin
                        if (!run3_guard_allowed_snapshot ||
                            !run3_guard_pid_initialized_snapshot) begin
                            // 主动禁驱或PID第一次初始化样本：继续输出0，不算故障。
                            run3_verified_command <= 16'sd0;
                            run3_guard_bad_streak <= 2'd0;
                        end else if (run3_guard_math_invalid ||
                                     (run3_guard_command_snapshot > 16'sd1400) ||
                                     (run3_guard_command_snapshot < -16'sd1400) ||
                                     (run3_guard_command_delta > 32'sd2) ||
                                     (run3_guard_command_delta < -32'sd2)) begin
                            // 本次核验失败只输出0；连续第三次失败锁止电机。
                            run3_verified_command <= 16'sd0;
                            run3_guard_reject_seen <= 1'b1;
                            if (run3_guard_bad_streak >= 2'd2) begin
                                run3_output_fault_latched <= 1'b1;
                                run3_guard_bad_streak <= 2'd3;
                            end else
                                run3_guard_bad_streak <= run3_guard_bad_streak + 2'd1;
                        end else begin
                            run3_verified_command <= run3_guard_command_snapshot;
                            run3_guard_bad_streak <= 2'd0;
                        end
                        run3_guard_stage <= 3'd0;
                    end
                    default: run3_guard_stage <= 3'd0;
                endcase
            end
        end
    end

    // 再次独立约束实际送到电机的已验证命令，防止异常大16位输出。
    wire signed [15:0] run3_safe_verified_command =
        ((run3_verified_command > 16'sd1400) ||
         (run3_verified_command < -16'sd1400)) ? 16'sd0 : run3_verified_command;

    // 最终PWM行程门控：受RUN3_AUTO_TRAVEL_LIMITS_ENABLE控制。
    // 设为0后，SWING/BAL不因原±350/±500阈值被阻断。
    // 正/负PWM对应编码器方向仍需在断动力条件下核实。
    wire run3_auto_output_soft_outward = RUN3_AUTO_TRAVEL_LIMITS_ENABLE &&
        run3_auto_started_r &&
        ((((run3_auto_delta >= RUN3_AUTO_SOFT_LIMIT) ||
           (run3_auto_predicted >= RUN3_AUTO_SOFT_LIMIT)) &&
          (run3_safe_verified_command > 16'sd0)) ||
         (((run3_auto_delta <= -RUN3_AUTO_SOFT_LIMIT) ||
           (run3_auto_predicted <= -RUN3_AUTO_SOFT_LIMIT)) &&
          (run3_safe_verified_command < 16'sd0)));
    // 若重新启用软件限位，硬限位仍在组合路径立即置零。
    wire signed [15:0] run3_travel_safe_verified_command =
        (run3_auto_output_soft_outward ||
         (run3_auto_started_r && run3_auto_hard_reached)) ?
        16'sd0 : run3_safe_verified_command;

    wire motor_request = run_active &&
        ((run_mode == RUN_TEST_FWD) ||
         (run_mode == RUN_TEST_REV) ||
         (run_mode == RUN_BALANCE) ||
         trajectory_drive_ready);
    // 原PID经RUN3独立校验后才能驱动；起摆和原PID都不能绕过该校验。
    // run3_safe_command用于保留原顶层的源命令界限防护；但PWM必须使用核验寄存值。
    wire run3_command_invalid =
        (run3_command > 16'sd1400) || (run3_command < -16'sd1400);
    wire signed [15:0] run3_safe_command =
        run3_command_invalid ? 16'sd0 : run3_command;

    wire motor_enable = motor_request &&
        !((run_mode == RUN_BALANCE) &&
          (run3_auto_fault_r || run3_output_fault_latched ||
           (run3_auto_started_r && run3_auto_hard_reached))) &&
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
    // RUN4 三种位置轨迹：梯形速度 / S曲线 / 正弦，使用 +/- 键循环切换。
    // 此处仅生成位置参考 trajectory_ref，RUN4分拍控制器和参数完全不改。
    // ================================================================
    localparam [1:0] MODE_BAL = 2'd2;
    localparam [1:0] TRAJ_TRAPEZOID = 2'd0;
    localparam [1:0] TRAJ_SCURVE    = 2'd1;
    localparam [1:0] TRAJ_SINE      = 2'd2;

    wire signed [12:0] adc_diff =
        $signed({1'b0, adc_value}) - $signed({1'b0, angle_zero});
    wire [12:0] abs_adc_diff = (adc_diff < 0) ? -adc_diff : adc_diff;
    wire trajectory_start = mode_start_pulse && !run_active && !key0_pulse &&
        (selected_mode == RUN_TRAJECTORY) && (abs_adc_diff <= 13'd120);

    reg [1:0] trajectory_kind; // 0梯形、1 S曲线、2正弦；默认梯形。
    wire signed [31:0] trajectory_ref;
    wire trajectory_key_context =
        (run_active && (run_mode == RUN_TRAJECTORY)) ||
        (!run_active && (selected_mode == RUN_TRAJECTORY));

    // 按键属于RUN4时，只切换轨迹种类；不再对RUN3目标执行加减。
    // 暂停且选中RUN4时也允许预选轨迹，下一次启动从该种类开始。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            trajectory_kind <= TRAJ_TRAPEZOID;
        else if (trajectory_key_context) begin
            if (target_inc_pulse && !target_dec_pulse)
                trajectory_kind <= (trajectory_kind == TRAJ_SINE) ?
                    TRAJ_TRAPEZOID : (trajectory_kind + 2'd1);
            else if (target_dec_pulse && !target_inc_pulse)
                trajectory_kind <= (trajectory_kind == TRAJ_TRAPEZOID) ?
                    TRAJ_SINE : (trajectory_kind - 2'd1);
        end
    end

    wire trajectory_advance = trajectory_mode &&
        (control_mode == MODE_BAL) &&
        !output_fault_latched && !command_out_of_range;

    run4_trajectory_generator trajectory_gen0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .adc_valid(adc_valid),
        .start(trajectory_start),
        .running(trajectory_mode),
        .advance(trajectory_advance),
        .selected_profile(trajectory_kind),
        .encoder_position(encoder_position),
        .target_position(trajectory_ref)
    );

    // RUN3保持手动设定的位置目标；RUN4只使用轨迹发生器的目标。
    wire signed [31:0] controller_position_target =
        (run_mode == RUN_TRAJECTORY) ? trajectory_ref : target_position;

    // 将模式命令送到 PWM：RUN1/RUN2 用固定低速测试值，RUN3/RUN4 用反馈控制量。
    // 命令单位为 PWM 计数，正负号决定方向；RUN3/RUN4都只使用通过独立核验的命令。
    // 当前电机/传感器安装方向沿用此符号约定，此处不再额外取反。
    always @* begin
        if (!motor_enable) begin
            // 不仅停止标志清零命令，任何命令一致性故障也立即清零命令。
            drive_command = 16'sd0;
        end else case (run_mode)
            // 实测启动门槛：正向约 153/2500=6.12%，反向约 150/2500=6.00%。
            RUN_TEST_FWD: drive_command = 16'sd153;
            RUN_TEST_REV: drive_command = -16'sd150;
            // RUN3源命令经独立多拍核验；禁止直接使用run3_effective_command。
            // 任何异常本采样输出0，连续三次失败锁止，正常值保持原方向和幅值。
            RUN_BALANCE: drive_command = run3_travel_safe_verified_command;
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
    // RUN3：沿用用户原始控制器，仅新增独立位置环使能；PID参数和限幅不变。
    // 实际使用 Kp(position)=13/32、Kd(position)=6。
    // 当position_loop_enable=0时关闭位置反馈，并清零外环窗口；不改变角度PID。
    // ================================================================
    pendulum_controller #(
        .ANGLE_CAPTURE(600), // 误差绝对值不超过 600 且变化较慢时允许捕获。
        .ANGLE_RELEASE(1040), // 平衡时误差绝对值超过 1040 则退回起摆。
        .CAPTURE_RATE(320), // 捕获要求相邻采样的误差变化绝对值不超过 320。
        .KP_ANGLE(0), // 关闭整数比例参数，改用下面的 10/2 定点比例增益。
        .KP_ANGLE_NUM(9), // 比例项定点分子，9/2 = 4.5。
        .KP_ANGLE_SHIFT(1), // 比例项定点右移一位。
        .KD_ANGLE(7), // 平衡多采样差分增益。
        .KI_ANGLE(180), // 每 5 ms 更新一次时，对应参考工程 Ki=0.01 的 PWM 计数缩放。
        .KI_ANGLE_SHIFT(12), // 定点积分缩放；积分输出仍受独立限幅保护。
        .ANGLE_I_UPDATE_SAMPLES(5), // 与参考程序角度环每 5 ms 调用一次 PID 对齐。
        .ANGLE_I_LIMIT(600), // 积分项最多贡献 600 个 PWM 命令计数。
        .BALANCE_D_WINDOW(4), // 差分跨度为 4 个采样间隔，默认约 4 ms。
        .KP_POSITION(0), // 关闭旧的直接 PWM 位置比例项。
        .KD_POSITION(0), // 关闭旧的直接 PWM 速度项。
        .WEAK_POSITION_ENABLE(1), // 启用串级位置环。
        .POSITION_CASCADE_ENABLE(1), // 位置 PD 调整角度目标，不直接叠加 PWM 命令。
        .POSITION_UPDATE_SAMPLES(50), // 与参考程序相同，每 50 ms 更新位置环。
        .POSITION_KP_NUM(10),
        .POSITION_KP_SHIFT(5), // 13/32≈0.406，近似参考程序 Kp=0.4。
        .POSITION_KD_NUM(8),
        .POSITION_KD_SHIFT(0), // 窗口位置误差变化乘参考程序 Kd=4。
        .POSITION_FEEDBACK_SIGN(1), // 正位置偏差生成负角度目标偏移，匹配串级参考符号。
        .POSITION_OUTPUT_LIMIT(80), // RUN3：保持用户上传的原始位置环输出限幅。
        .POSITION_SLEW_LIMIT(12), // 每个 50 ms 窗口最多变化 8 个 ADC 角度计数。
        .POSITION_ANGLE_LIMIT(400), // 超过小角度区时清除外环及其测速历史。
        .SWING_GAIN(700), // 起摆分支输出幅值，单位为 PWM 计数。
        .SWING_SPEED_GAIN(8), // 起摆能量估算中的相邻采样差分系数。
        .SWING_TARGET_ENERGY(40_000_000), // 起摆目标能量，使用算法内部计数量纲。
        .COS_ZERO_CROSS(4096), // 起摆方向判断使用的角度计数阈值。
        .OUTPUT_LIMIT(1400), // 总命令限幅，包含平衡分支已加上的摩擦补偿。
        // 摩擦补偿只作用于平衡分支：零命令不加偏置，小命令按幅值逐渐加补偿。
        // 正负方向分别设置补偿上限，加补偿后再执行 OUTPUT_LIMIT 总限幅。
        .FRICTION_FWD(109), // 正向平衡输出的最大摩擦补偿。
        .FRICTION_REV(109), // 反向平衡输出的最大摩擦补偿。
        .FRICTION_RAMP_SHIFT(7) // 原始命令幅值达到 2^7=128 后加满补偿。
    ) controller_run3 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        // 仅自动起摆阶段暂停原控制器，防止在起摆过程中积累位置/积分状态。
        // 正常直立启动时enable与原run3_balance_enable完全相同。
        .enable(run3_controller_enable),
        .position_loop_enable(run3_position_loop_enable),
        .angle_history_track(run3_angle_history_track),
        .adc_value(adc_value),
        .angle_zero(angle_zero),
        .encoder_position(run3_controller_position),
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

    // RUN3同一模式内的独立自动起摆。始终使用相同的摆动象限注能算法。
    // 起摆时固定传入SWING状态，使本模块不会因原控制器提前进入BAL而停止。
    // 直立捕获由顶层连续样本判定，完成后关闭起摆模块使能。
    pendulum_swingup #(
        .ANGLE_CAPTURE(80),
        .ANGLE_RELEASE(240),
        .CAPTURE_RATE(6),
        .SWING_GAIN(700),
        .COS_ZERO_CROSS(1112),
        .HALF_TURN_COUNTS(2224),
        .BRAKE_ANGLE(240),
        .ARM_TRAVEL_LIMIT(600),
        .SWING_TIMEOUT_SAMPLES(12000),
        .OUTPUT_LIMIT(1400)
    ) swingup_run3 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .enable(run3_balance_enable && run3_auto_pending_r && !run3_auto_fault_r),
        .adc_value(adc_value),
        .angle_zero(angle_zero),
        .encoder_position(encoder_position),
        .control_mode(2'd1),
        .active(run3_swing_active),
        .command(run3_swing_command),
        .raw_command(run3_swing_raw),
        .fault_latched(run3_swing_fault),
        .result_valid(run3_swing_result_valid)
    );

    // ================================================================
    // RUN4：前一版已经使用的分拍计算控制器，位置目标是轨迹发生器输出。
    // 实际使用 Kp(position)=13/32、Kd(position)=5。
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
        .POSITION_KP_NUM(13), // RUN4：保持用户上传参数：Kp = 13/32.
        .POSITION_KP_SHIFT(5), // 位置环比例项右移 5 位，参数不在轨迹层内调整。
        .POSITION_KD_NUM(5), // RUN4：保持用户上传参数：Kd = 5.
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
        .control_command((run_mode == RUN_BALANCE) ? drive_command : control_command),
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
            // 3169参考值；放正摆杆后仍可按 KEY1 重新记录。
            angle_zero <= 12'd3169;
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
            telemetry_delay <= 12'd0;
            telemetry_guard_delay <= 10'd0;
            telemetry_sample_id <= 32'd0;
            telemetry_adc <= 12'd0;
            telemetry_zero <= 12'd3169;
            telemetry_position <= 32'sd0;
            telemetry_velocity <= 16'sd0;
            telemetry_target <= 32'sd0;
        end else begin
            // 采样值在ADC有效沿锁存；旧RUN3在ADC沿更新命令，新RUN4约11拍后提交。
            // RUN4保护在S_COMMIT提交之后读取结果，再进行6拍计算后更新verified_command。
            // UART再延迟10拍读取完整结果，使最终PWM命令和内部诊断字段同属本采样。
            telemetry_delay <= {telemetry_delay[10:0], adc_valid};
            telemetry_guard_delay <= {telemetry_guard_delay[8:0], guard_sample_tick};
            telemetry_tick <= telemetry_guard_delay[9];
            if (adc_valid) begin
                telemetry_sample_id <= telemetry_sample_id + 1'b1;
                telemetry_adc <= adc_value;
                telemetry_zero <= angle_zero;
                // RUN3自动路径上报相对坐标，保证位置与目标属于同一坐标系。
                // 其余模式（含手扶直立RUN3、RUN4）上报原始编码器坐标。
                telemetry_position <= run3_controller_position;
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
                    // 从自动起摆RUN3暂停：将相对目标折算回原始坐标。
                    // 暂停后自动坐标开关会清除，下一次手扶直立启动仍能保持同一物理目标。
                    if ((run_mode == RUN_BALANCE) && run3_auto_started_r)
                        target_position <= target_position +
                            run3_auto_fixed_origin - RUN3_AUTO_START_COORD;
                end else if (selected_mode == RUN_SAFE) begin
                    run_mode   <= RUN_SAFE;
                    run_active <= 1'b0;
                // RUN4 保持之前的直立附近启动条件，避免倾斜过大时直接启动。
                end else if ((selected_mode != RUN_TRAJECTORY) || trajectory_start) begin
                    run_mode   <= selected_mode;
                    run_active <= 1'b1;
                    // 从下垂自动起摆：START位置定义为-330，但目标从START起始终为0。
                    // 起摆期间位置外环关闭；进入BAL后立即开始50ms位置环计数。
                    // 手扶直立启动仍沿用原始编码器坐标，不改变原有控制路径。
                    if ((selected_mode == RUN_BALANCE) && run3_auto_start_from_down)
                        target_position <= 32'sd0;
                    else if ((selected_mode == RUN_BALANCE) &&
                             (run_mode != RUN_BALANCE))
                        target_position <= encoder_position;
                end
            end

            // 选择模式或上次运行模式为RUN3时，每次按键调整100个编码器计数。
            // 自动起摆路径在首次BAL锁存相对目标0前忽略按键，避免覆盖首次定位。
            // 锁存之后以及手扶直立启动时，保留原来的±100按键操作。
            if (!trajectory_key_context && !(mode_start_pulse && run_active) &&
                ((selected_mode == RUN_BALANCE) ||
                 (run_mode == RUN_BALANCE)) &&
                (!run3_auto_started_r || run3_auto_center_latched_r)) begin
                if (target_inc_pulse && !target_dec_pulse)
                    target_position <= target_position + POSITION_STEP;
                else if (target_dec_pulse && !target_inc_pulse)
                    target_position <= target_position - POSITION_STEP;
            end
            // 自动起摆成功、原控制器首次进入BAL时再次确认目标为0。
            // 位置外环在BAL时立即使能，输出仍按50ms周期更新。
            // 手扶直立启动（run3_auto_started_r=0）完全旁路此自动定位。
            if (run3_auto_first_balance && !(mode_start_pulse && run_active))
                target_position <= RUN3_AUTO_BALANCE_TARGET;

            // KEY1 只在停止/暂停时锁存最近的 ADC 值为直立零点，避免运行时改变平衡基准。
            // angle_error 在控制器后续采样节拍中按新零点更新，此处不清零编码器位置。
            // RUN3恢复用户原版行为：允许运行时KEY1校零；RUN4仍只允许停机时校零。
            // 注意：运行中校零本身可能改变平衡参考，实际测试建议先暂停。
            if (key1_pulse && (!run_active || (run_mode == RUN_BALANCE)))
                angle_zero <= adc_value;
        end
    end

    // RUN4运行时 LED[1:0] 显示轨迹种类：00 梯形、01 S曲线、10 正弦；其他模式保留原行为。
    assign led[0] = trajectory_mode ? trajectory_kind[0] : heartbeat[23]; // 50 MHz 下每约 168 ms 翻转一次，观察主时钟运行。
    assign led[1] = trajectory_mode ? trajectory_kind[1] : adc_activity; // ADC 采样活动翻转信号，并非采样有效脉冲本身。
    assign led[2] = motor_enable; // 电机驱动使能状态，为 1 时允许 PWM 输出。
    // RUN3校验拒绝/锁止或自动起摆硬限位故障时LED3常亮；
    // 软限位仅抑制向外PWM，不置故障LED；其他时候保留编码器活动。
    // RUN4仍沿用原来的拒绝/锁止LED3行为。
    assign led[3] = ((run_mode == RUN_BALANCE) &&
                     (run3_auto_fault_r || run3_guard_reject_seen ||
                      run3_output_fault_latched)) ? 1'b1 :
                    ((run_mode == RUN_TRAJECTORY) &&
                     (output_fault_latched || guard_reject_seen)) ?
                    1'b1 : encoder_activity;

endmodule