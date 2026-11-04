`timescale 1ns/1ps

module tb_angle_integral;
    reg clk = 0;
    reg rst_n = 0;
    reg sample_tick = 0;
    reg enable = 0;
    reg [11:0] adc_value = 12'd2058;
    wire signed [15:0] command;
    wire [1:0] mode;
    wire signed [15:0] kp_command;
    wire [1:0] kp_mode;
    integer failures = 0;
    integer i;
    reg test_passed = 0;

    always #10 clk = ~clk;

    pendulum_controller #(
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1000), .CAPTURE_RATE(320),
        .KP_ANGLE(0), .KD_ANGLE(0),
        .KI_ANGLE(4096), .KI_ANGLE_SHIFT(12),
        .ANGLE_I_UPDATE_SAMPLES(5), .ANGLE_I_LIMIT(500),
        .KP_POSITION(0), .KD_POSITION(0), .WEAK_POSITION_ENABLE(0),
        .SWING_GAIN(0), .OUTPUT_LIMIT(1000)
    ) dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(32'sd0), .position_target(32'sd0),
        .encoder_velocity(32'sd0), .command(command), .angle_error(),
        .control_mode(mode), .debug_angle_pd(), .debug_position_applied(),
        .debug_balance_rate(), .debug_raw_control(), .debug_flags(),
        .debug_window_count()
    );

    pendulum_controller #(
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1000), .CAPTURE_RATE(320),
        .KP_ANGLE(0), .KP_ANGLE_NUM(15), .KP_ANGLE_SHIFT(1), .KD_ANGLE(0),
        .KI_ANGLE(0), .KP_POSITION(0), .KD_POSITION(0),
        .WEAK_POSITION_ENABLE(0), .SWING_GAIN(0), .OUTPUT_LIMIT(1000)
    ) kp_dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(32'sd0), .position_target(32'sd0),
        .encoder_velocity(32'sd0), .command(kp_command), .angle_error(),
        .control_mode(kp_mode), .debug_angle_pd(), .debug_position_applied(),
        .debug_balance_rate(), .debug_raw_control(), .debug_flags(),
        .debug_window_count()
    );

    task check(input bit condition, input string message);
        if (!condition) begin
            failures++;
            $display("FAIL: %s", message);
        end
    endtask

    task sample;
        @(negedge clk);
        sample_tick = 1;
        @(negedge clk);
        sample_tick = 0;
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (3) sample();
        enable = 1;
        sample();
        check(mode === 2'd2, "starts in balance near upright");

        repeat (4) sample();
        check(command === 16'sd0,
              "integral does not update during the first four 1 ms samples");
        check(kp_mode === 2'd2 && kp_command === 16'sd75,
              "15/2 fixed-point proportional gain produces 7.5 times positive error");
        sample();
        check(command === 16'sd10,
              "fifth sample adds the current error once, matching 5 ms PID calls");

        repeat (4) sample();
        check(command === 16'sd10,
              "integral output holds between 5 ms updates");
        sample();
        check(command === 16'sd20,
              "next 5 ms update accumulates one more current error");

        adc_value = 12'd2038;
        repeat (4) sample();
        check(command === 16'sd20,
              "opposite error waits for the next 5 ms integral update");
        check(kp_command === -16'sd75,
              "15/2 fixed-point proportional gain preserves negative sign");
        sample();
        check(command === 16'sd10,
              "negative error reduces the signed accumulated integral");

        enable = 0;
        sample();
        check(dut.angle_integral === 64'sd0,
              "pause clears the integral state");

        test_passed = (failures == 0);
        $display("Angle integral regression: %0d failures", failures);
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "angle integral test timeout");
    end
endmodule
