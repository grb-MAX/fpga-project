`timescale 1ns/1ps

module tb_run_mode_isolation;
    reg clk = 1'b0;
    reg key0_n = 1'b1;
    reg key1_n = 1'b1;
    reg key2_n = 1'b0;
    reg mode_start_n = 1'b1;
    reg target_inc_n = 1'b1;
    reg target_dec_n = 1'b1;
    reg adc_dout = 1'b0;
    reg encoder_a = 1'b0;
    reg encoder_b = 1'b0;
    wire adc_cs_n, adc_sclk, motor_in1, motor_in2;
    tri1 oled_scl, oled_sda;
    wire uart_tx;
    wire [3:0] led;
    integer failures = 0;

    always #10 clk = ~clk;

    acg525_pendulum_top dut (
        .clk_50m(clk),
        .key0_n(key0_n),
        .key1_n(key1_n),
        .key2_n(key2_n),
        .mode_start_n(mode_start_n),
        .target_inc_n(target_inc_n),
        .target_dec_n(target_dec_n),
        .adc_dout(adc_dout),
        .adc_cs_n(adc_cs_n),
        .adc_sclk(adc_sclk),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2),
        .encoder_a(encoder_a),
        .encoder_b(encoder_b),
        .oled_scl(oled_scl),
        .oled_sda(oled_sda),
        .uart_tx(uart_tx),
        .led(led)
    );

    initial begin
        #100;
        key2_n = 1'b1;
        #100;

        dut.run_active = 1'b1;
        force dut.target_position = 32'sd1234;
        force dut.trajectory_ref = -32'sd77;

        dut.run_mode = 3'd0;
        #1;
        dut.run_mode = 3'd3;
        #1;
        if ($signed(dut.controller_position_target) !== 32'sd1234) begin
            $display("FAIL: RUN3 selected trajectory target");
            failures = failures + 1;
        end
        if (dut.motor_enable !== 1'b1) begin
            $display("FAIL: RUN3 motor enable is not active");
            failures = failures + 1;
        end

        dut.run_mode = 3'd0;
        #1;
        dut.run_mode = 3'd4;
        #1;
        if ($signed(dut.controller_position_target) !== -32'sd77) begin
            $display("FAIL: RUN4 did not select trajectory target");
            failures = failures + 1;
        end
        if (dut.motor_enable !== 1'b1) begin
            $display("FAIL: RUN4 motor enable is not active");
            failures = failures + 1;
        end

        release dut.target_position;
        release dut.trajectory_ref;

        $display("RUN_MODE_TARGET_ISOLATION failures=%0d", failures);
        if (failures != 0)
            $fatal(1, "run-mode target isolation regression failed");
        $finish;
    end
endmodule
