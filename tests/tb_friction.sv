`timescale 1ns/1ps

module tb_friction;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg enable = 1'b0;
    reg [11:0] adc_value = 12'd2048;
    reg signed [31:0] position_target = 0;
    wire signed [15:0] command, legacy_command;
    wire [1:0] mode;
    wire motor_in1, motor_in2;
    integer failures = 0;
    reg test_passed = 1'b0;

    always #10 clk = ~clk;

    pendulum_controller #(
        .KP_ANGLE(1), .KD_ANGLE(0), .KP_POSITION(1), .KD_POSITION(0),
        .FRICTION_FWD(153), .FRICTION_REV(150), .FRICTION_RAMP_SHIFT(7)
    ) dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(1'b1), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(32'sd0), .position_target(position_target),
        .encoder_velocity(32'sd0), .command(command),
        .angle_error(), .control_mode(mode)
    );

    // Default zero compensation must preserve the original control output.
    pendulum_controller #(
        .KP_ANGLE(1), .KD_ANGLE(0), .KP_POSITION(1), .KD_POSITION(0)
    ) legacy (
        .clk(clk), .rst_n(rst_n), .sample_tick(1'b1), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(32'sd0), .position_target(position_target),
        .encoder_velocity(32'sd0), .command(legacy_command),
        .angle_error(), .control_mode()
    );

    motor_pwm_dual pwm (
        .clk(clk), .rst_n(rst_n), .enable(enable), .command(command),
        .motor_in1(motor_in1), .motor_in2(motor_in2)
    );

    task check(input bit condition, input string message);
        if (!condition) begin
            failures = failures + 1;
            $display("FAIL: %s", message);
        end
    endtask

    task expect_output(input integer raw, input integer expected);
        integer legacy_expected;
        @(negedge clk);
        position_target = raw;
        @(negedge clk);
        legacy_expected = raw > 900 ? 900 : (raw < -900 ? -900 : raw);
        check(mode === 2'd2, "balance branch selected");
        check(command === expected,
              $sformatf("raw %0d: got %0d, expected %0d", raw, command, expected));
        check(legacy_command === legacy_expected, "zero compensation preserves output");
    endtask

    task expect_pwm(input integer forward_count, input integer reverse_count);
        integer forward_seen, reverse_seen, i;
        forward_seen = 0;
        reverse_seen = 0;
        repeat (3) @(negedge clk);
        for (i = 0; i < 2500; i = i + 1) begin
            @(negedge clk);
            if ({motor_in1, motor_in2} === 2'b01) forward_seen++;
            if ({motor_in1, motor_in2} === 2'b10) reverse_seen++;
        end
        check(forward_seen == forward_count, "forward differential PWM duration");
        check(reverse_seen == reverse_count, "reverse differential PWM duration");
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        repeat (3) @(negedge clk);
        check(command === 16'sd0, "stopped command is zero");
        enable = 1'b1;
        repeat (3) @(negedge clk);

        expect_output(0, 0);
        expect_output(1, 2);
        expect_output(-1, -2);
        expect_output(8, 17);
        expect_output(-8, -17);
        expect_output(64, 140);
        expect_output(-64, -139);
        expect_output(80, 175);
        expect_output(-80, -173);
        expect_output(127, 278);
        expect_output(-127, -275);
        expect_output(128, 281);
        expect_pwm(281, 0);
        expect_output(-128, -278);
        expect_pwm(0, 278);
        expect_output(129, 282);
        expect_output(-129, -279);
        expect_output(500, 653);
        expect_output(-500, -650);
        expect_output(746, 899);
        expect_output(-749, -899);
        expect_output(747, 900);
        expect_output(-750, -900);
        expect_output(1000, 900);
        expect_pwm(900, 0);
        expect_output(-1000, -900);
        expect_pwm(0, 900);
        expect_output(0, 0);
        expect_pwm(0, 0);
        check({motor_in1, motor_in2} === 2'b00, "zero command drives both pins low");

        // Falling out of capture must retain the original swing-up amplitude.
        @(negedge clk);
        position_target = 0;
        adc_value = 12'd2448;
        repeat (3) @(negedge clk);
        check(mode === 2'd1, "large deviation enters swing-up");
        check(command === 16'sd420, "swing-up is not friction compensated");
        check(command === legacy_command, "swing-up matches legacy output");

        enable = 1'b0;
        repeat (3) @(negedge clk);
        check(command === 16'sd0, "pause clears output");
        check({motor_in1, motor_in2} === 2'b00, "pause disables PWM");
        adc_value = 12'd2048;
        enable = 1'b1;
        repeat (3) @(negedge clk);
        expect_output(0, 0);
        expect_output(80, 175);
        rst_n = 1'b0;
        #1;
        check(command === 16'sd0, "reset clears compensated output immediately");
        check({motor_in1, motor_in2} === 2'b00, "reset clears motor pins immediately");

        test_passed = (failures == 0);
        $display("Friction/PWM regression: %0d failures", failures);
        $finish;
    end

    initial begin
        #1000000;
        $fatal(1, "friction test timeout");
    end
endmodule
