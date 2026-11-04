
module acg525_pendulum_top (
    input  wire        clk_50m,
    input  wire        key0_n,
    input  wire        key1_n,
    input  wire        key2_n,
    input  wire        mode_start_n,
    input  wire        target_inc_n,
    input  wire        target_dec_n,
    input  wire        adc_dout,
    output wire        adc_cs_n,
    output wire        adc_sclk,
    output wire        motor_in1,
    output wire        motor_in2,
    input  wire        encoder_a,
    input  wire        encoder_b,
    inout  wire        oled_scl,
    inout  wire        oled_sda,
    output wire        uart_tx,
    output wire [3:0]  led
);

    wire rst_n = key2_n;

    wire [11:0] adc_value;
    wire adc_valid;
    wire adc_busy;

    wire signed [31:0] encoder_position;
    wire signed [31:0] encoder_velocity;
    wire signed [15:0] control_command;
    reg  signed [15:0] drive_command;
    wire signed [15:0] angle_error;
    wire [1:0] control_mode;

    wire signed [31:0] debug_angle_pd;
    wire signed [31:0] debug_position_applied;
    wire signed [15:0] debug_balance_rate;
    wire signed [31:0] debug_raw_control;
    wire [7:0] debug_flags;
    wire [7:0] debug_window_count;
    wire [31:0] debug_adc_glitch_count;
    wire [31:0] debug_adc_max_delta;
    wire [31:0] debug_limit_events;

    reg telemetry_tick;
    reg [31:0] telemetry_sample_id;
    reg [11:0] telemetry_adc;
    reg [11:0] telemetry_zero;
    reg signed [31:0] telemetry_position;
    reg signed [15:0] telemetry_velocity;
    reg signed [31:0] telemetry_target;

    wire key0_pulse;
    wire key1_pulse;
    wire mode_start_pulse;
    wire target_inc_pulse;
    wire target_dec_pulse;

    wire oled_scl_low;
    wire oled_sda_low;

    reg [11:0] angle_zero;
    reg [2:0] run_mode;
    reg [2:0] selected_mode;
    reg run_active;

    reg signed [31:0] target_position;
    reg [23:0] heartbeat;
    reg adc_activity;
    reg encoder_activity;
    reg [24:0] display_count;
    reg display_tick;

    localparam [2:0] RUN_SAFE       = 3'd0;
    localparam [2:0] RUN_TEST_FWD   = 3'd1;
    localparam [2:0] RUN_TEST_REV   = 3'd2;
    localparam [2:0] RUN_BALANCE    = 3'd3;
    localparam [2:0] RUN_TRAJECTORY = 3'd4;

    localparam integer POSITION_STEP = 32'sd100;

    // RUN3 / RUN4 共用原来的平衡控制器
    wire trajectory_mode =
        run_active && (run_mode == RUN_TRAJECTORY);

    wire balance_enable =
        run_active &&
        ((run_mode == RUN_BALANCE) ||
         (run_mode == RUN_TRAJECTORY));

    wire signed [12:0] adc_diff =
        $signed({1'b0, adc_value}) -
        $signed({1'b0, angle_zero});

    wire [12:0] abs_adc_diff =
        (adc_diff < 0) ? -adc_diff : adc_diff;

    wire signed [31:0] trajectory_ref;

    // RUN3: 手动目标
    // RUN4: 正弦目标
    wire signed [31:0] active_position_target =
        (run_mode == RUN_TRAJECTORY) ?
        trajectory_ref : target_position;

    // RUN4 从直立附近启动
    wire trajectory_start =
        mode_start_pulse &&
        !run_active &&
        !key0_pulse &&
        (selected_mode == RUN_TRAJECTORY) &&
        (abs_adc_diff <= 13'd120);

    // 修复重点：
    // 第一次进入平衡状态后锁存
    // 不再因为控制模式变化而反复清零
    reg trajectory_balance_sampled;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            trajectory_balance_sampled <= 1'b0;
        end else if (!trajectory_mode) begin
            trajectory_balance_sampled <= 1'b0;
        end else if (adc_valid &&
                     (control_mode == 2'd2)) begin
            trajectory_balance_sampled <= 1'b1;
        end
    end

    wire trajectory_drive_ready =
        trajectory_mode && trajectory_balance_sampled;

    wire motor_enable =
        run_active &&
        ((run_mode == RUN_TEST_FWD) ||
         (run_mode == RUN_TEST_REV) ||
         (run_mode == RUN_BALANCE) ||
         trajectory_drive_ready);

    // 只产生正弦位置目标
    position_trajectory #(
        .AMPLITUDE_COUNTS(150),
        .PERIOD_SAMPLES(30000)
    ) trajectory0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .start(trajectory_start),
        .enable(trajectory_mode),
        .encoder_position(encoder_position),
        .position_ref(trajectory_ref)
    );

    wire [2:0] display_mode =
        motor_enable ? run_mode : selected_mode;

    // RUN4 一旦完成首次平衡启动，
    // 后续与 RUN3 一样直接使用原控制器命令
    always @* begin
        if (!run_active) begin
            drive_command = 16'sd0;
        end else begin
            case (run_mode)
                RUN_TEST_FWD:
                    drive_command = 16'sd153;

                RUN_TEST_REV:
                    drive_command = -16'sd150;

                RUN_BALANCE:
                    drive_command = control_command;

                RUN_TRAJECTORY:
                    drive_command =
                        trajectory_drive_ready ?
                        control_command : 16'sd0;

                default:
                    drive_command = 16'sd0;
            endcase
        end
    end

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

    quadrature_encoder encoder0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .enc_a_i(encoder_a),
        .enc_b_i(encoder_b),
        .speed_tick(adc_valid),
        .position(encoder_position),
        .velocity(encoder_velocity)
    );

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

    // 原稳定版 PID 参数保持不变
    pendulum_controller #(
        .ANGLE_CAPTURE(600),
        .ANGLE_RELEASE(1040),
        .CAPTURE_RATE(320),

        .KP_ANGLE(0),
        .KP_ANGLE_NUM(9),
        .KP_ANGLE_SHIFT(1),
        .KD_ANGLE(7),
        .KI_ANGLE(200),
        .KI_ANGLE_SHIFT(12),

        .ANGLE_I_UPDATE_SAMPLES(5),
        .ANGLE_I_LIMIT(600),
        .BALANCE_D_WINDOW(4),

        .KP_POSITION(0),
        .KD_POSITION(0),

        .WEAK_POSITION_ENABLE(1),
        .POSITION_CASCADE_ENABLE(1),

        .POSITION_UPDATE_SAMPLES(50),
        .POSITION_KP_NUM(12),
        .POSITION_KP_SHIFT(5),
        .POSITION_KD_NUM(6),
        .POSITION_KD_SHIFT(0),

        .POSITION_FEEDBACK_SIGN(1),
        .POSITION_OUTPUT_LIMIT(100),
        .POSITION_SLEW_LIMIT(8),
        .POSITION_ANGLE_LIMIT(400),

        .SWING_GAIN(700),
        .SWING_SPEED_GAIN(4),
        .SWING_TARGET_ENERGY(40_000_000),
        .COS_ZERO_CROSS(4096),

        .OUTPUT_LIMIT(1400),
        .FRICTION_FWD(100),
        .FRICTION_REV(100),
        .FRICTION_RAMP_SHIFT(7)
    ) controller0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .sample_tick(adc_valid),
        .enable(balance_enable),

        .adc_value(adc_value),
        .angle_zero(angle_zero),

        .encoder_position(encoder_position),
        .position_target(active_position_target),
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

    motor_pwm_dual #(
        .PWM_PERIOD(2500),
        .DEAD_BAND(0)
    ) motor0 (
        .clk(clk_50m),
        .rst_n(rst_n),
        .enable(motor_enable),
        .command(drive_command),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2)
    );

    uart_telemetry_tx #(
        .DIAGNOSTIC_ENABLE(1)
    ) uart0 (
        .clk(clk_50m),
        .rst_n(rst_n),

        .adc_value(telemetry_adc),
        .encoder_position(telemetry_position),

        .encoder_velocity(
            {{16{telemetry_velocity[15]}},
             telemetry_velocity}
        ),

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

        .diagnostic_flags(
            {motor_enable, run_active, debug_flags[5:0]}
        ),

        .window_count(debug_window_count),
        .adc_glitch_count(debug_adc_glitch_count),
        .adc_max_delta(debug_adc_max_delta),
        .limit_events(debug_limit_events),

        .uart_tx(uart_tx)
    );

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

    assign oled_scl =
        oled_scl_low ? 1'b0 : 1'bz;

    assign oled_sda =
        oled_sda_low ? 1'b0 : 1'bz;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
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

            telemetry_tick      <= 1'b0;
            telemetry_sample_id <= 32'd0;
            telemetry_adc       <= 12'd0;
            telemetry_zero      <= 12'd3153;
            telemetry_position  <= 32'sd0;
            telemetry_velocity  <= 16'sd0;
            telemetry_target    <= 32'sd0;

        end else begin
            telemetry_tick <= adc_valid;

            if (adc_valid) begin
                telemetry_sample_id <=
                    telemetry_sample_id + 1'b1;

                telemetry_adc <= adc_value;
                telemetry_zero <= angle_zero;
                telemetry_position <= encoder_position;
                telemetry_velocity <= encoder_velocity[15:0];
                telemetry_target <= active_position_target;
            end

            heartbeat <= heartbeat + 1'b1;

            display_tick <= 1'b0;

            if (display_count == 25'd24_999_999) begin
                display_count <= 25'd0;
                display_tick <= 1'b1;
            end else begin
                display_count <= display_count + 1'b1;
            end

            if (adc_valid) begin
                adc_activity <= ~adc_activity;

                if (encoder_velocity != 0)
                    encoder_activity <= ~encoder_activity;
            end

            // KEY0 循环选择模式
            if (key0_pulse && !run_active) begin
                case (selected_mode)
                    RUN_SAFE:
                        selected_mode <= RUN_TEST_FWD;

                    RUN_TEST_FWD:
                        selected_mode <= RUN_TEST_REV;

                    RUN_TEST_REV:
                        selected_mode <= RUN_BALANCE;

                    RUN_BALANCE:
                        selected_mode <= RUN_TRAJECTORY;

                    default:
                        selected_mode <= RUN_SAFE;
                endcase
            end

            // 启动 / 停止
            if (mode_start_pulse &&
                (run_active || !key0_pulse)) begin

                if (run_active) begin
                    run_active <= 1'b0;

                end else if (selected_mode == RUN_SAFE) begin
                    run_mode <= RUN_SAFE;
                    run_active <= 1'b0;

                end else if (
                    (selected_mode != RUN_TRAJECTORY) ||
                    trajectory_start
                ) begin

                    run_mode <= selected_mode;
                    run_active <= 1'b1;

                    if (
                        (selected_mode == RUN_BALANCE) &&
                        (run_mode != RUN_BALANCE)
                    )
                        target_position <= encoder_position;
                end
            end

            // RUN3 手动位置目标
            if (
                (selected_mode == RUN_BALANCE) ||
                (run_mode == RUN_BALANCE)
            ) begin
                if (target_inc_pulse && !target_dec_pulse)
                    target_position <=
                        target_position + POSITION_STEP;

                else if (
                    target_dec_pulse && !target_inc_pulse
                )
                    target_position <=
                        target_position - POSITION_STEP;
            end

            if (key1_pulse && !run_active)
                angle_zero <= adc_value;
        end
    end

    assign led[0] = heartbeat[23];
    assign led[1] = adc_activity;
    assign led[2] = motor_enable;
    assign led[3] = encoder_activity;

endmodule
