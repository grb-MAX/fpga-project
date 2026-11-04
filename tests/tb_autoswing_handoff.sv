`timescale 1ns/1ps

module tb_autoswing_handoff;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg sample_tick = 1'b0;
    reg enable = 1'b0;
    reg [11:0] adc_value = 12'd2048;
    reg [11:0] angle_zero = 12'd2048;
    wire signed [15:0] command;
    wire [1:0] control_mode;
    integer failures = 0;

    always #5 clk = ~clk;

    pendulum_controller #(
        .ANGLE_CAPTURE(20),
        .ANGLE_RELEASE(40),
        .CAPTURE_RATE(5),
        .KP_ANGLE(1),
        .KD_ANGLE(1),
        .BALANCE_D_WINDOW(4),
        .AUTOSWING_ENABLE(1),
        .CAPTURE_CONFIRM_SAMPLES(5),
        .HANDOFF_SLEW_LIMIT(4),
        .SWING_GAIN(10),
        .OUTPUT_LIMIT(20),
        .FRICTION_FWD(0),
        .FRICTION_REV(0)
    ) dut (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enable),
        .adc_value(adc_value), .angle_zero(angle_zero),
        .encoder_position(32'sd0), .position_target(32'sd0),
        .encoder_velocity(32'sd0), .command(command), .angle_error(),
        .control_mode(control_mode), .debug_angle_pd(),
        .debug_position_applied(), .debug_balance_rate(),
        .debug_raw_control(), .debug_flags(), .debug_window_count(),
        .debug_adc_glitch_count(), .debug_adc_max_delta(), .debug_limit_events()
    );

    task automatic sample(input integer error_value);
        begin
            @(negedge clk);
            adc_value = 12'd2048 + error_value;
            sample_tick = 1'b1;
            @(negedge clk);
            sample_tick = 1'b0;
        end
    endtask

    task automatic check(input bit condition, input string message);
        begin
            if (!condition) begin
                failures = failures + 1;
                $display("FAIL: %s", message);
            end
        end
    endtask

    integer i;
    integer previous_command;
    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        enable = 1'b1;

        sample(-100);
        check(control_mode == 2'd1 && command == 0,
              "enable starts in SWING with the first output held at zero");
        sample(-100);
        check(control_mode == 2'd1 && command != 0,
              "SWING output is active after the first initialized sample");

        // One qualifying sample must not hand control to BAL.
        sample(0);
        check(control_mode == 2'd1, "single calm sample does not capture");
        sample(0);
        sample(0);
        sample(0);
        check(control_mode == 2'd1, "capture remains pending before confirmation count");
        sample(0);
        check(control_mode == 2'd1, "four qualifying samples still do not capture");
        sample(0);
        check(control_mode == 2'd2, "five calm samples capture into BAL");
        previous_command = command;
        for (i = 0; i < 3; i = i + 1) begin
            sample(0);
            check(($signed(command) - previous_command) <= 4 &&
                  (previous_command - $signed(command)) <= 4,
                  "handoff limits each PWM change");
            previous_command = command;
        end
        check(command == 0, "BAL output is settled after handoff");

        // Leaving the release window returns to SWING and does not retain BAL history.
        sample(100);
        check(control_mode == 2'd1, "release returns to SWING");
        check($signed(command) <= 20 && $signed(command) >= -20,
              "release output remains within the final limit");

        enable = 1'b0;
        sample(100);
        check(control_mode == 2'd0 && command == 0,
              "disable immediately clears mode and command");

        if (failures == 0)
            $display("AUTOSWING_HANDOFF PASS");
        else
            $display("AUTOSWING_HANDOFF failures=%0d", failures);
        $finish;
    end
endmodule
