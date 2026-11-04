`timescale 1ns/1ps

module tb_weak_position;
    reg clk = 0;
    reg rst_n = 0;
    reg sample_tick = 0;
    reg enable = 0;
    reg [11:0] adc_value = 2048;
    reg signed [31:0] encoder_position = 1000;
    reg signed [31:0] position_target = 1000;
    reg signed [31:0] encoder_velocity = 0;
    wire signed [15:0] command, baseline_command, position_command, velocity_command;
    wire [1:0] mode;
    integer failures = 0;
    integer check_sequence = 0;
    integer i;
    reg test_passed = 0;
    reg signed [63:0] previous_feedback;
    reg signed [15:0] previous_command;

    always #10 clk = ~clk;

    // 主实例使用本轮实机角度参数及原有摩擦补偿。
    pendulum_controller #(
        .KP_ANGLE(8), .KD_ANGLE(10), .BALANCE_D_WINDOW(4),
        .KP_POSITION(0), .KD_POSITION(0), .WEAK_POSITION_ENABLE(1),
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1040), .CAPTURE_RATE(320),
        .FRICTION_FWD(153), .FRICTION_REV(150)
    ) dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(encoder_position), .position_target(position_target),
        .encoder_velocity(encoder_velocity), .command(command),
        .angle_error(), .control_mode(mode)
    );

    // 关闭新外环的对照实例用于检查暂停、起摆、超角度区及总限幅。
    pendulum_controller #(
        .KP_ANGLE(8), .KD_ANGLE(10), .BALANCE_D_WINDOW(4),
        .KP_POSITION(0), .KD_POSITION(0), .WEAK_POSITION_ENABLE(0),
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1040), .CAPTURE_RATE(320),
        .FRICTION_FWD(153), .FRICTION_REV(150)
    ) baseline (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(encoder_position), .position_target(position_target),
        .encoder_velocity(encoder_velocity), .command(baseline_command),
        .angle_error(), .control_mode()
    );

    // 分别关闭速度项和位置项，独立核验量纲及符号。
    pendulum_controller #(
        .KP_ANGLE(0), .KD_ANGLE(0), .KP_POSITION(0), .KD_POSITION(0),
        .WEAK_POSITION_ENABLE(1), .POSITION_KD_NUM(0),
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1040), .CAPTURE_RATE(320)
    ) position_only (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(encoder_position), .position_target(position_target),
        .encoder_velocity(encoder_velocity), .command(position_command),
        .angle_error(), .control_mode()
    );

    pendulum_controller #(
        .KP_ANGLE(0), .KD_ANGLE(0), .KP_POSITION(0), .KD_POSITION(0),
        .WEAK_POSITION_ENABLE(1), .POSITION_KP_NUM(0),
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1040), .CAPTURE_RATE(320)
    ) velocity_only (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(encoder_position), .position_target(position_target),
        .encoder_velocity(encoder_velocity), .command(velocity_command),
        .angle_error(), .control_mode()
    );

    task check(input bit condition, input string message);
        check_sequence++;
        if (!condition) begin
            failures++;
            $display("FAIL check=%0d time=%0t mode=%0d feedback=%0d command=%0d baseline=%0d position=%0d velocity=%0d: %s",
                     check_sequence, $time, mode, dut.position_feedback, command,
                     baseline_command, position_command, velocity_command, message);
        end
    endtask

    task sample;
        reg signed [63:0] feedback_before;
        integer reconstructed, magnitude;
        feedback_before = dut.position_feedback;
        @(negedge clk);
        sample_tick = 1;
        @(negedge clk);
        sample_tick = 0;
        check($signed(command) <= 900 && $signed(command) >= -900, "总输出始终在正负九百以内");
        check(dut.position_feedback <= 64 && dut.position_feedback >= -64, "外环原始命令独立限幅");
        if (rst_n) begin
            if (dut.debug_flags[2]) begin
                check(dut.debug_angle_pd == 8 * $signed(dut.angle_error) +
                      10 * $signed(dut.debug_balance_rate), "诊断角度PD对应本采样误差及差分");
                check(dut.debug_raw_control == dut.debug_angle_pd +
                      dut.debug_position_applied, "诊断摩擦前命令可由实际分项还原");
            end else begin
                check(dut.debug_angle_pd == 0 && dut.debug_position_applied == 0,
                      "非平衡计算不报告旧PD或旧外环贡献");
            end
            if (dut.debug_flags[3])
                check(dut.debug_position_applied == feedback_before,
                      "诊断报告实际使用的前窗口值而非本拍新窗口值");
            else
                check(dut.debug_position_applied == 0, "未叠加时实际外环贡献为零");
            reconstructed = dut.debug_raw_control;
            if (dut.debug_flags[2]) begin
                magnitude = reconstructed < 0 ? -reconstructed : reconstructed;
                if (magnitude > 128) magnitude = 128;
                if (reconstructed > 0) reconstructed += (magnitude * 153) >> 7;
                else if (reconstructed < 0) reconstructed -= (magnitude * 150) >> 7;
            end
            check(dut.debug_flags[4] == (reconstructed > 900 || reconstructed < -900),
                  "诊断总限幅标志对应摩擦补偿后的实际输出");
            if (reconstructed > 900) reconstructed = 900;
            if (reconstructed < -900) reconstructed = -900;
            check(command == reconstructed, "诊断分项还原最终命令");
        end
    endtask

    task samples(input integer count);
        integer n;
        for (n = 0; n < count; n++) sample();
    endtask

    task restart_at(input integer position_now, input integer target_now);
        enable = 0;
        adc_value = 2048;
        encoder_position = position_now;
        position_target = target_now;
        encoder_velocity = 0;
        samples(4);
        enable = 1;
        samples(3);
        check(mode === 2'd2, "近直立启动进入平衡");
        check(dut.position_feedback === 64'sd0, "启动没有旧外环输出冲击");
    endtask

    task expect_static(input integer position_now, input integer target_now,
                       input integer expected_raw, input integer expected_command);
        encoder_position = position_now;
        position_target = target_now;
        // 从正限幅到负限幅需要至少三十二次更新，另留四个窗口清除速度过渡。
        samples(32 * 36 + 2);
        check(dut.position_feedback === expected_raw, $sformatf("STATIC_RAW expected=%0d", expected_raw));
        check($signed(command) == expected_command, $sformatf("STATIC_COMMAND expected=%0d", expected_command));
        check($signed(position_command) == expected_raw, $sformatf("POSITION_ONLY expected=%0d", expected_raw));
        check(velocity_command === 16'sd0, "停止后窗口速度项归零");
        check(baseline_command === 16'sd0, "关闭外环时仍保持原角度零输出");
    endtask

    initial begin
        samples(3);
        rst_n = 1;
        restart_at(1000, 1000);
        samples(64);
        check(command === 16'sd0, "零角度且到达目标时命令为零");

        // 位置跳变不允许一个采样拍直接叠加大命令。
        encoder_position = 1320;
        for (i = 0; i < 100; i++) begin
            previous_feedback = dut.position_feedback;
            sample();
            check(dut.position_feedback - previous_feedback <= 4 &&
                  dut.position_feedback - previous_feedback >= -4, "外环每次变化不超过四计数");
        end
        expect_static(1320, 1000, 10, 21);
        expect_static(680, 1000, -10, -21);
        expect_static(1031, 1000, 0, 0);
        expect_static(969, 1000, 0, 0);
        expect_static(1032, 1000, 1, 2);
        expect_static(968, 1000, -1, -2);
        expect_static(2000000000, -2000000000, 64, 140);
        expect_static(-2000000000, 2000000000, -64, -139);

        // 只改目标、保持当前位置不变时，同样限制外环命令的变化率。
        restart_at(1000, 1000);
        position_target = 680;
        for (i = 0; i < 100; i++) begin
            previous_feedback = dut.position_feedback;
            sample();
            check(dut.position_feedback - previous_feedback <= 4 &&
                  dut.position_feedback - previous_feedback >= -4, "目标阶跃经过变化率限制");
        end
        samples(32 * 4);
        check(dut.position_feedback === 64'sd10, "位置目标参与新外环");

        // 单毫秒速度输入故意给极端值，新外环仍只使用窗口位置差。
        encoder_velocity = 32'sd2000000000;
        samples(64);
        check(dut.position_feedback === 64'sd10, "未误用原始单毫秒速度");
        encoder_velocity = 0;

        restart_at(0, 0);
        for (i = 1; i <= 32 * 20; i++) begin
            encoder_position = i;
            sample();
        end
        check($signed(velocity_command) == 32, "正向一计数每采样对应窗口速度三十二");
        for (i = 1; i <= 32 * 20; i++) begin
            encoder_position = 32 * 20 - i;
            sample();
        end
        check($signed(velocity_command) == -32, "负向窗口速度具有对称符号");
        samples(32 * 20);
        check(velocity_command === 16'sd0, "静止后不保留旧速度");

        expect_static(1320, 1000, 10, 21);
        previous_feedback = dut.position_feedback;
        previous_command = command;
        encoder_position = 1500;
        repeat (100) @(negedge clk);
        check(dut.position_feedback === previous_feedback && command === previous_command,
              "没有采样使能时输出和窗口保持不变");

        adc_value = 2129;
        sample();
        check(mode === 2'd2 && dut.position_feedback === 64'sd0, "仍在平衡但超过小角度区时撤掉外环");
        check(command === baseline_command, "超出外环角度区时只保留原角度控制");
        encoder_position = 3000;
        samples(4);
        adc_value = 2048;
        position_target = 3000;
        samples(5);
        check(dut.position_feedback === 64'sd0, "重入小角度区先重新填测速历史");
        samples(64);
        check(command === 16'sd0 && velocity_command === 16'sd0, "重入后没有旧速度冲击");

        adc_value = 2118;
        sample();
        check(command === 16'sd900, "角度正向快速变化仍执行总限幅");
        samples(5);
        adc_value = 1978;
        sample();
        check(command === -16'sd900, "角度负向快速变化仍执行总限幅");

        adc_value = 3248;
        samples(3);
        check(mode === 2'd1 && dut.position_feedback === 64'sd0, "失衡进入起摆时外环清零");
        check(command === baseline_command && (command === 420 || command === -420),
              "外环不改变起摆幅值与分支");

        enable = 0;
        sample();
        check(command === 16'sd0 && dut.position_feedback === 64'sd0, "暂停清除总输出及外环");
        restart_at(5000, 5000);
        samples(64);
        check(command === 16'sd0 && velocity_command === 16'sd0, "暂停搬动后重新启动无速度冲击");

        rst_n = 0;
        #1;
        check(command === 16'sd0 && dut.position_feedback === 64'sd0, "异步复位立即清零");
        test_passed = (failures == 0);
        $display("Weak position regression: %0d failures", failures);
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "弱外环回归超时");
    end
endmodule
