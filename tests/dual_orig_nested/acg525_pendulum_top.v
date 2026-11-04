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
    wire signed [15:0] control_command; // 控制器输出，已包含其内部限幅和适用的摩擦补偿。
    reg signed [15:0]  drive_command; // 按运行模式选择后的 PWM 输入命令，同时送 UART。
    wire signed [15:0] angle_error; // 当前 ADC 减直立零点，单位为 ADC 计数。
    wire [1:0]         control_mode; // 控制器内部状态：0 停止、1 起摆、2 平衡。
    wire signed [31:0] debug_angle_pd;
    wire signed [31:0] debug_position_applied;
    wire signed [15:0] debug_balance_rate;
    wire signed [31:0] debug_raw_control;
    wire [7:0] debug_flags;
    wire [7:0] debug_window_count;
    wire [31:0] debug_adc_glitch_count;
    wire [31:0] debug_adc_max_delta;
    wire [31:0] debug_limit_events;
    reg telemetry_tick; // 将诊断采集延后一系统时钟，以读取控制器本拍更新后的输出。
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

    // 顶层模式编号：0 安全、1 正向测试、2 反向测试、3 起摆/平衡控制。
    // 与 control_mode 的内部状态编号独立；RUN3 内部仍会在起摆和平衡之间切换。
    localparam [2:0] RUN_SAFE     = 3'd0; // 安全模式：保持停止，启动键不会使电机运行。
    localparam [2:0] RUN_TEST_FWD = 3'd1; // 正向低速测试模式，使用固定正命令。
    localparam [2:0] RUN_TEST_REV = 3'd2; // 反向低速测试模式，使用固定负命令。
    localparam [2:0] RUN_BALANCE  = 3'd3; // 控制器运行模式，内部按条件选择起摆或直立平衡。
    localparam integer POSITION_STEP = 32'sd100; // 每次目标位置按键的步长。

    // 只有启动 RUN3 才使能控制器；RUN1/RUN2 绕过控制器，使用固定测试命令。
    // motor_enable 表示驱动允许输出，并不代表命令一定非零或电机一定正在转动。
    wire balance_enable = run_active && (run_mode == RUN_BALANCE); // 仅运行 RUN3 时为高，允许控制器执行起摆或平衡计算。
    wire motor_enable   = run_active && ((run_mode == RUN_TEST_FWD) || // 运行测试或 RUN3 时为高，允许 PWM 驱动；不等同于电机实际运动。
                           (run_mode == RUN_TEST_REV) ||
                           (run_mode == RUN_BALANCE));
    // 运行时显示正在执行的模式，停止/暂停时显示待启动的选择。
    wire [2:0] display_mode = motor_enable ? run_mode : selected_mode; // 运行时显示 run_mode，停止时显示 selected_mode。

    // 将模式命令送到 PWM：RUN1/RUN2 用固定低速测试值，RUN3 用反馈控制量。
    // 命令单位为 PWM 计数，正负号决定方向；RUN3 直接传递 control_command。
    // 当前电机/传感器安装方向沿用此符号约定，此处不再额外取反。
    always @* begin
        if (!run_active) begin
            drive_command = 16'sd0;
        end else case (run_mode)
            // 实测启动门槛：正向约 153/2500=6.12%，反向约 150/2500=6.00%。
            RUN_TEST_FWD: drive_command = 16'sd153;
            RUN_TEST_REV: drive_command = -16'sd150;
            // RUN3 命令可能来自起摆分支或平衡分支，由 controller0 内部状态决定。
            RUN_BALANCE:  drive_command = control_command;
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

    // 角度环当前使用 KP=6、KD=15、4 ms 差分，并加入极弱的 Ki=1/2^12 积分项。
    // 角度原始命令 = 6*最短角度误差 + 15*当前与 4 个采样前 ADC 的最短差分。
    // 多采样差分用于减轻短周期量化噪声影响，未除以窗口长度或采样时间。
    // 历史保存 ADC 原值，避免校零偏置直接进入平衡 D 项；启动/起摆时会重填历史。
    // 正命令使编码器位置增加；串级位置环使用当前位置减目标位置及窗口位移。
    // 串级位置 PD 每 50 ms 更新，输出有界角度参考偏移，由角度内环跟踪。
    // 起摆与平衡的切换依据角度/变化率阈值，最终输出对称限幅至 +/-1400。
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
        .POSITION_KD_NUM(5),
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
    ) controller0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .enable(balance_enable),
        .adc_value(adc_value),
        .angle_zero(angle_zero),
        .encoder_position(encoder_position),
        .position_target(target_position),
        .encoder_velocity(encoder_velocity),
        .command(control_command),
        .angle_error(angle_error),
        .control_mode(control_mode),
        .debug_angle_pd(debug_angle_pd),
        .debug_position_applied(debug_position_applied),
        .debug_balance_rate(debug_balance_rate),
        .debug_raw_control(debug_raw_control),
        .debug_flags(debug_flags),
        .debug_window_count(debug_window_count),
        .debug_adc_glitch_count(debug_adc_glitch_count),
        .debug_adc_max_delta(debug_adc_max_delta),
        .debug_limit_events(debug_limit_events)
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
    // 本版使用55 AB诊断帧；先按控制采样形成完整快照，再由串口定时发送。
    // 保持原空闲计时10 ms，发送44字节另需约3.82 ms；上位机同时兼容旧55 AA帧。
    // command 上报 drive_command，即模式选择后的 PWM 输入命令；暂停后 run_mode 仍保留。
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
            telemetry_sample_id <= 32'd0;
            telemetry_adc <= 12'd0;
            telemetry_zero <= 12'd3153;
            telemetry_position <= 32'sd0;
            telemetry_velocity <= 16'sd0;
            telemetry_target <= 32'sd0;
        end else begin
            // 输入快照与控制器使用同一ADC采样沿；下一系统时钟读取相应控制输出。
            telemetry_tick <= adc_valid;
            if (adc_valid) begin
                telemetry_sample_id <= telemetry_sample_id + 1'b1;
                telemetry_adc <= adc_value;
                telemetry_zero <= angle_zero;
                telemetry_position <= encoder_position;
                telemetry_velocity <= encoder_velocity[15:0];
                telemetry_target <= target_position;
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
            // KEY0 只在停止/暂停时选模式，不直接启动：SAFE -> 正向 -> 反向 -> RUN3 -> SAFE。
            if (key0_pulse && !motor_enable) begin
                case (selected_mode)
                    RUN_SAFE:     selected_mode <= RUN_TEST_FWD;
                    RUN_TEST_FWD: selected_mode <= RUN_TEST_REV;
                    RUN_TEST_REV: selected_mode <= RUN_BALANCE;
                    default:      selected_mode <= RUN_SAFE;
                endcase
            end

            // 启动/暂停键：运行中按下即暂停；停止时按 selected_mode 启动。
            // 停止时若选模式和启动脉冲同拍到达，只执行选模式，电机继续保持停止。
            // SAFE 模式下按启动仍停机；暂停保留 run_mode、selected_mode 和目标位置。
            if (mode_start_pulse && (motor_enable || !key0_pulse)) begin
                if (motor_enable) begin
                    run_active <= 1'b0;
                end else if (selected_mode == RUN_SAFE) begin
                    run_mode   <= RUN_SAFE;
                    run_active <= 1'b0;
                end else begin
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
            // KEY1 无运行模式限制，将最近的 ADC 值锁存为直立零点。
            // angle_error 在控制器后续采样节拍中按新零点更新，此处不清零编码器位置。
            if (key1_pulse)
                angle_zero <= adc_value;
        end
    end

    assign led[0] = heartbeat[23]; // 50 MHz 下每约 168 ms 翻转一次，观察主时钟运行。
    assign led[1] = adc_activity; // ADC 采样活动翻转信号，并非采样有效脉冲本身。
    assign led[2] = motor_enable; // 电机驱动使能状态，为 1 时允许 PWM 输出。
    assign led[3] = encoder_activity; // 采样时速度非零则翻转，反映编码器活动。

endmodule