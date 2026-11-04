`timescale 1ns/1ps
module tb_cascade_position;
    reg clk = 0, rst_n = 0, sample_tick = 0, enable = 0;
    reg [11:0] adc_value = 2048;
    reg signed [31:0] encoder_position = 100, position_target = 0;
    wire signed [15:0] command, angle_error;
    wire [1:0] mode;
    wire signed [31:0] angle_pd, position_offset, raw_control;
    wire [7:0] flags, window_count;
    wire [31:0] glitch_count, max_delta, limit_count;
    integer failures = 0;
    reg test_passed = 0;

    always #10 clk = ~clk;

    pendulum_controller #(
        .ANGLE_CAPTURE(300), .ANGLE_RELEASE(300), .CAPTURE_RATE(100),
        .KP_ANGLE(0), .KP_ANGLE_NUM(2), .KP_ANGLE_SHIFT(0), .KD_ANGLE(0),
        .KP_POSITION(0), .KD_POSITION(0),
        .KI_ANGLE(0), .WEAK_POSITION_ENABLE(1), .POSITION_CASCADE_ENABLE(1),
        .POSITION_UPDATE_SAMPLES(2), .POSITION_KP_NUM(1), .POSITION_KP_SHIFT(0),
        .POSITION_KD_NUM(0), .POSITION_FEEDBACK_SIGN(-1),
        .POSITION_OUTPUT_LIMIT(50), .POSITION_SLEW_LIMIT(8),
        .POSITION_ANGLE_LIMIT(100), .OUTPUT_LIMIT(500),
        .FRICTION_FWD(0), .FRICTION_REV(0), .SWING_GAIN(0)
    ) dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(12'd2048),
        .encoder_position(encoder_position), .position_target(position_target),
        .encoder_velocity(32'sd0), .command(command), .angle_error(angle_error),
        .control_mode(mode), .debug_angle_pd(angle_pd),
        .debug_position_applied(position_offset), .debug_balance_rate(),
        .debug_raw_control(raw_control), .debug_flags(flags),
        .debug_window_count(window_count), .debug_adc_glitch_count(glitch_count),
        .debug_adc_max_delta(max_delta), .debug_limit_events(limit_count)
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
        tick(); tick(); tick();
        check(mode == 2, "near-upright startup enters balance mode");
        check(dut.position_feedback == 0, "outer loop waits for its configured sample window");
        tick();
        check(dut.position_feedback == -8, "position output follows the configured slew limit");
        tick();
        check(angle_error == 0, "reported sensor angle error remains the measured error");
        check(position_offset == -8, "diagnostic reports the angle-reference offset");
        check(angle_pd == -16 && raw_control == -16 && command == -16,
              "position offset is tracked through the inner angle gain");
        check(flags[3], "cascade offset is active in the balance branch");

        // A persistent position error remains bounded and approaches the outer-loop clamp.
        repeat (20) begin tick(); tick(); end
        check(dut.position_feedback == -50, "position reference offset obeys its output clamp");
        check(command == -100, "inner loop tracks the saturated angle-reference offset");

        adc_value = 2300;
        tick();
        check(dut.position_feedback == 0, "leaving the small-angle region clears outer-loop state");
        check(position_offset == 0, "outer-loop angle offset is not applied outside its gate");

        test_passed = failures == 0;
        $display("CASCADE_POSITION failures=%0d", failures);
        $finish;
    end
    initial begin #100000; $fatal(1, "CASCADE_POSITION_TIMEOUT"); end
endmodule
