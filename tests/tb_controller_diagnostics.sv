`timescale 1ns/1ps
module tb_controller_diagnostics;
    reg clk = 0, rst_n = 0, sample_tick = 0, enable = 0;
    reg [11:0] adc_value = 0, angle_zero = 0;
    wire signed [15:0] command, angle_error;
    wire [1:0] control_mode;
    wire signed [31:0] debug_angle_pd, debug_position_applied, debug_raw_control;
    wire signed [15:0] debug_balance_rate;
    wire [7:0] debug_flags, debug_window_count;
    wire [31:0] debug_adc_glitch_count, debug_adc_max_delta, debug_limit_events;
    integer failures = 0;
    reg test_passed = 0;

    always #5 clk = ~clk;

    pendulum_controller #(
        .ANGLE_CAPTURE(300), .ANGLE_RELEASE(300), .CAPTURE_RATE(1000),
        .KP_ANGLE(100), .KD_ANGLE(0), .OUTPUT_LIMIT(100), .SWING_GAIN(0)
    ) dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(angle_zero),
        .encoder_position(32'sd0), .position_target(32'sd0), .encoder_velocity(32'sd0),
        .command(command), .angle_error(angle_error), .control_mode(control_mode),
        .debug_angle_pd(debug_angle_pd), .debug_position_applied(debug_position_applied),
        .debug_balance_rate(debug_balance_rate), .debug_raw_control(debug_raw_control),
        .debug_flags(debug_flags), .debug_window_count(debug_window_count),
        .debug_adc_glitch_count(debug_adc_glitch_count),
        .debug_adc_max_delta(debug_adc_max_delta), .debug_limit_events(debug_limit_events)
    );

    task tick;
        begin
            @(negedge clk); sample_tick = 1;
            @(negedge clk); sample_tick = 0;
        end
    endtask

    task check(input bit condition, input string message);
        if (!condition) begin failures++; $display("FAIL: %s", message); end
    endtask

    initial begin
        repeat (2) @(negedge clk); rst_n = 1; enable = 1;
        adc_value = 1000; angle_zero = 1000; tick();
        check(debug_adc_glitch_count == 0, "first ADC sample is not counted as a glitch");
        check(debug_adc_max_delta == 0, "first ADC sample does not set the maximum delta");

        adc_value = 1700; angle_zero = 1700; tick();
        check(debug_adc_glitch_count == 1, "large adjacent ADC delta increments glitch count");
        check(debug_adc_max_delta == 700, "maximum ADC delta records the measured difference");

        adc_value = 1900; angle_zero = 1700; tick();
        check(debug_raw_control == 20000, "raw controller result is exposed before output clamp");
        check(command == 100, "command remains clamped to the configured output limit");
        check(debug_limit_events == 1, "output clamp event is counted");
        test_passed = failures == 0;
        $display("CONTROLLER_DIAGNOSTICS failures=%0d", failures);
        $finish;
    end

    initial begin #100000; $fatal(1, "CONTROLLER_DIAGNOSTICS_TIMEOUT"); end
endmodule
