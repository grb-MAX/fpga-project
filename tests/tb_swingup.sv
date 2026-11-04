`timescale 1ns/1ps

module tb_swingup;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg sample_tick = 1'b0;
    reg enable = 1'b0;
    reg [11:0] adc_value = 12'd2048;
    reg [11:0] angle_zero = 12'd2048;
    reg signed [31:0] encoder_position = 32'sd0;
    reg signed [31:0] position_target = 32'sd0;
    reg signed [31:0] encoder_velocity = 32'sd0;
    wire signed [15:0] command;
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
    integer checks = 0;

    always #5 clk = ~clk;

    pendulum_controller #(
        .ANGLE_CAPTURE(20),
        .ANGLE_RELEASE(40),
        .CAPTURE_RATE(5),
        .KP_ANGLE(1),
        .KD_ANGLE(1),
        .SWING_GAIN(10),
        .SWING_OBSERVE_INTERVAL(2),
        .SWING_PULSE_TICKS(3),
        .SWING_PEAK_RANGE(50),
        .SWING_CAPTURE_SAMPLES(2),
        .SWING_TIMEOUT_TICKS(30),
        .CATCH_RAMP_TICKS(4),
        .MOTION_POSITION_LIMIT(250),
        .MOTION_VELOCITY_LIMIT(64),
        .OUTPUT_LIMIT(20),
        .FRICTION_FWD(0),
        .FRICTION_REV(0)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .sample_tick(sample_tick),
        .enable(enable),
        .adc_value(adc_value),
        .angle_zero(angle_zero),
        .encoder_position(encoder_position),
        .position_target(position_target),
        .encoder_velocity(encoder_velocity),
        .command(command),
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

    task sample_error(input integer error_value);
        begin
            @(negedge clk);
            adc_value = 12'd2048 + error_value;
            sample_tick = 1'b1;
            @(posedge clk);
            #1;
            @(negedge clk);
            sample_tick = 1'b0;
        end
    endtask

    task check(input logic condition, input string message);
        begin
            checks = checks + 1;
            if (!condition) begin
                $display("FAIL: %s", message);
                $fatal(1);
            end
        end
    endtask

    integer i;
    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        enable = 1'b1;

        sample_error(-100);
        check(control_mode == 2'd1 && command == 0, "start enters swing-up with output initially safe");
        check(dut.swing_error_now == 12'd1025, "swing angle uses the 923-count down reference independently");
        repeat (3) sample_error(-100);
        check(command == 10, "initial positive 3 ms impulse");
        sample_error(-100);
        check(command == -10, "initial reverse impulse starts after positive impulse");
        repeat (2) sample_error(-100);
        check(command == -10, "initial reverse impulse lasts 3 ms");
        sample_error(-100);
        check(command == 0, "controller observes between impulse pairs");

        repeat (2) sample_error(80);
        repeat (2) sample_error(100);
        repeat (2) sample_error(60);
        check(dut.swing_phase == 3'd3, "three-point right-side minimum triggers return impulse");
        sample_error(100);
        check(command == 10, "detected right-side peak triggers positive impulse");
        repeat (3) sample_error(100);
        sample_error(100);
        check(command == -10, "counter impulse follows peak impulse");
        repeat (3) sample_error(100);

        sample_error(0);
        sample_error(0);
        sample_error(0);
        sample_error(0);
        check(control_mode == 2'd3, "two calm center observations enter catch mode");
        repeat (4) sample_error(0);
        check(control_mode == 2'd2, "catch ramp hands control to balance mode");

        enable = 1'b0;
        sample_error(0);
        check(command == 0 && control_mode == 2'd0 && dut.swing_failed == 1'b0,
              "disable immediately clears output and timeout latch");

        enable = 1'b1;
        sample_error(-100);
        repeat (31) sample_error(-100);
        check(command == 0 && control_mode == 2'd0 && dut.swing_failed == 1'b1,
              "swing-up timeout locks the motor output off");
        repeat (3) sample_error(-100);
        check(command == 0, "timeout remains latched while enabled");

        enable = 1'b0;
        sample_error(-100);
        encoder_position = 32'sd1000;
        enable = 1'b1;
        sample_error(-100);
        check(control_mode == 2'd1 && dut.swing_position_origin == 32'sd1000,
              "position safety origin captures the encoder position on restart");
        encoder_position = 32'sd1251;
        sample_error(-100);
        check(command == 0 && control_mode == 2'd0 && dut.swing_failed,
              "relative position limit immediately locks motor output off");

        enable = 1'b0;
        sample_error(-100);
        encoder_position = 32'sd0;
        enable = 1'b1;
        sample_error(-100);
        encoder_velocity = 32'sd65;
        sample_error(-100);
        check(command == 0 && control_mode == 2'd0 && dut.swing_failed,
              "velocity limit immediately locks motor output off");

        $display("PASS: %0d swing-up checks", checks);
        $finish;
    end
endmodule
