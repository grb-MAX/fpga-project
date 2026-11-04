`timescale 1ns/1ps

module tb_balance_only;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg sample_tick = 1'b0;
    reg enable = 1'b0;
    reg [11:0] adc_value = 12'd1200;
    reg [11:0] angle_zero = 12'd2000;
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
        .KP_ANGLE(2),
        .KD_ANGLE(0),
        .OUTPUT_LIMIT(100),
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

    task tick;
        begin
            @(negedge clk);
            sample_tick = 1'b1;
            @(negedge clk);
            sample_tick = 1'b0;
            #1;
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
        tick();
        check(control_mode == 2'd0 && command == 0,
              "start away from upright leaves controller safe and output zero");

        for (i = 0; i < 8; i = i + 1) begin
            adc_value = adc_value + 12'd100;
            tick();
            check(control_mode != 2'd1 && control_mode != 2'd3 && command == 0,
                  "waiting for upright never emits swing or catch commands");
        end
        tick();
        check(control_mode == 2'd2 && command == 0,
              "stable upright sample enters balance mode without a pulse");
        tick();
        adc_value = 12'd2010;
        tick();
        check(control_mode == 2'd2 && command == 16'sd20,
              "existing angle loop controls normally inside the balance window");

        adc_value = 12'd1950;
        tick();
        check(control_mode == 2'd0 && command == 0,
              "leaving the balance release window stops instead of starting swing-up");
        check(dut.position_feedback == 0,
              "position-loop state is cleared when leaving balance mode");

        $display("PASS: %0d balance-only checks", checks);
        $finish;
    end
endmodule
