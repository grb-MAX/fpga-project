`timescale 1ns/1ps

module tb_balance_derivative;
    reg clk = 0;
    reg rst_n = 0;
    reg sample_tick = 0;
    reg enable = 0;
    reg [11:0] adc_value = 2048;
    reg [11:0] angle_zero = 2048;
    wire signed [15:0] command, legacy_command;
    wire [1:0] mode;
    integer failures = 0;
    integer i;
    reg test_passed = 0;

    always #10 clk = ~clk;

    pendulum_controller #(
        .KP_ANGLE(0), .KD_ANGLE(32), .BALANCE_D_WINDOW(8),
        .KP_POSITION(0), .KD_POSITION(0),
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1040), .CAPTURE_RATE(320),
        .AUTOSWING_ENABLE(1)
    ) dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(angle_zero),
        .encoder_position(32'sd0), .position_target(32'sd0),
        .encoder_velocity(32'sd0), .command(command), .angle_error(),
        .control_mode(mode), .debug_angle_pd(), .debug_position_applied(),
        .debug_balance_rate(), .debug_raw_control(), .debug_flags(),
        .debug_window_count(), .debug_adc_glitch_count(), .debug_adc_max_delta(),
        .debug_limit_events()
    );

    pendulum_controller #(
        .KP_ANGLE(0), .KD_ANGLE(32),
        .KP_POSITION(0), .KD_POSITION(0),
        .ANGLE_CAPTURE(600), .ANGLE_RELEASE(1040), .CAPTURE_RATE(320)
    ) legacy (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(angle_zero),
        .encoder_position(32'sd0), .position_target(32'sd0),
        .encoder_velocity(32'sd0), .command(legacy_command), .angle_error(),
        .control_mode(), .debug_angle_pd(), .debug_position_applied(),
        .debug_balance_rate(), .debug_raw_control(), .debug_flags(),
        .debug_window_count(), .debug_adc_glitch_count(), .debug_adc_max_delta(),
        .debug_limit_events()
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
        check($signed(dut.adc_linear_delta(12'd2, 12'd4094)) === -32'sd4092,
              "ADC low endpoint uses linear negative difference");
        check($signed(dut.adc_linear_delta(12'd4094, 12'd2)) === 32'sd4092,
              "ADC high endpoint uses linear positive difference");
        check($signed(dut.adc_linear_delta(12'd2048, 12'd0)) === 32'sd2048,
              "linear ADC difference preserves half-scale value");

        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (3) sample();
        enable = 1;
        repeat (3) sample();
        check(mode === 2'd2, "start in balance");
        check(command === 0, "primed history gives zero at rest");

        for (i = 1; i <= 16; i++) begin
            adc_value = 2048 + i;
            sample();
            if (i >= 8) begin
                check(command === 16'sd256, "constant positive ramp uses full eight-sample slope");
                check(legacy_command === 16'sd32, "default window retains one-sample derivative");
            end
        end
        repeat (8) sample();
        check(command === 0, "derivative clears after eight stationary samples");

        // A slow quantized ramp should not lose velocity between ADC steps.
        for (i = 1; i <= 24; i++) begin
            adc_value = 2064 + ((i + 3) / 4);
            sample();
            if (i >= 8)
                check(command === 16'sd64, "slow staircase preserves its two-count window slope");
        end
        repeat (8) sample();
        angle_zero = adc_value;
        sample();
        check(command === 0, "zero calibration does not create a measurement derivative kick");

        // Pausing and moving by hand must not retain an old velocity estimate.
        enable = 0;
        sample();
        check(command === 0 && mode === 0, "pause clears output");
        adc_value = 2300;
        sample();
        enable = 1;
        repeat (3) sample();
        check(mode === 2'd2 && command === 0, "resume primes current angle without a kick");
        for (i = 1; i <= 16; i++) begin
            adc_value = 2300 - i;
            sample();
            if (i >= 8)
                check(command === -16'sd256, "negative ramp has the correct derivative sign");
        end
        repeat (20) @(negedge clk);
        check(command === -16'sd256, "history and output hold without a sample tick");
        repeat (8) sample();
        check(command === 0, "negative ramp clears after stopping");

        // Enable the guarded swing path explicitly for this release-transition check.
        angle_zero = 12'd2064;
        adc_value = 4095;
        repeat (3) sample();
        check(mode === 2'd1, "release enters the original swing recovery state");
        adc_value = angle_zero;
        repeat (20) sample();
        check(mode === 2'd2 && command === 0, "calm center samples re-enter balance from safe mode");
        rst_n = 0;
        #1;
        check(command === 0 && mode === 0, "reset clears output immediately");
        rst_n = 1;
        repeat (20) sample();
        check(mode === 2'd2 && command === 0, "reset re-primes a stationary measurement");

        test_passed = (failures == 0);
        $display("Balance derivative regression: %0d failures", failures);
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "balance derivative test timeout");
    end
endmodule
