`timescale 1ns/1ps

module tb_sine_trajectory;
    reg clk = 1'b0;
    reg key0_n = 1'b1, key1_n = 1'b1, key2_n = 1'b0;
    reg mode_start_n = 1'b1, target_inc_n = 1'b1, target_dec_n = 1'b1;
    reg adc_dout = 1'b0, encoder_a = 1'b0, encoder_b = 1'b0;
    wire adc_cs_n, adc_sclk, motor_in1, motor_in2, oled_scl, oled_sda, uart_tx;
    wire [3:0] led;
    integer phase_i;
    integer previous_offset;
    integer current_offset;
    integer maximum_step;
    integer failures;

    always #10 clk = ~clk;

    acg525_pendulum_top dut (
        .clk_50m(clk), .key0_n(key0_n), .key1_n(key1_n), .key2_n(key2_n),
        .mode_start_n(mode_start_n), .target_inc_n(target_inc_n), .target_dec_n(target_dec_n),
        .adc_dout(adc_dout), .adc_cs_n(adc_cs_n), .adc_sclk(adc_sclk),
        .motor_in1(motor_in1), .motor_in2(motor_in2),
        .encoder_a(encoder_a), .encoder_b(encoder_b),
        .oled_scl(oled_scl), .oled_sda(oled_sda), .uart_tx(uart_tx), .led(led)
    );

    initial begin
        failures = 0;
        #100;
        key2_n = 1'b1;
        force dut.run_active = 1'b1;
        force dut.run_mode = 3'd4;
        force dut.control_mode = 2'd2;
        force dut.trajectory_center = 32'sd500;
        force dut.trajectory_center_goal = 32'sd500;

        maximum_step = 0;
        force dut.trajectory_phase = 12'd0;
        #1;
        previous_offset = $signed(dut.trajectory_offset);
        for (phase_i = 1; phase_i < 4096; phase_i = phase_i + 1) begin
            force dut.trajectory_phase = phase_i;
            #1;
            current_offset = $signed(dut.trajectory_offset);
            if ((current_offset - previous_offset > maximum_step) ||
                (previous_offset - current_offset > maximum_step))
                maximum_step = (current_offset > previous_offset) ?
                               (current_offset - previous_offset) :
                               (previous_offset - current_offset);
            previous_offset = current_offset;
        end
        force dut.trajectory_phase = 12'd0;
        #1;
        current_offset = $signed(dut.trajectory_offset);
        if (current_offset - previous_offset > maximum_step)
            maximum_step = current_offset - previous_offset;
        if (previous_offset - current_offset > maximum_step)
            maximum_step = previous_offset - current_offset;
        if (maximum_step > 1) begin
            $display("FAIL: sine target step=%0d", maximum_step);
            failures = failures + 1;
        end
        if ($signed(dut.trajectory_offset) !== 0) begin
            $display("FAIL: phase zero offset=%0d", $signed(dut.trajectory_offset));
            failures = failures + 1;
        end
        force dut.trajectory_phase = 12'd1024;
        #1;
        if ($signed(dut.trajectory_offset) < 99 || $signed(dut.trajectory_offset) > 100) begin
            $display("FAIL: quarter peak offset=%0d", $signed(dut.trajectory_offset));
            failures = failures + 1;
        end
        force dut.trajectory_phase = 12'd3072;
        #1;
        if ($signed(dut.trajectory_offset) > -99 || $signed(dut.trajectory_offset) < -100) begin
            $display("FAIL: three-quarter peak offset=%0d", $signed(dut.trajectory_offset));
            failures = failures + 1;
        end
        force dut.target_position = 32'sd1234;
        force dut.run_mode = 3'd3;
        #1;
        if (dut.controller_position_target !== 32'sd1234) begin
            $display("FAIL: RUN3 target changed to %0d", $signed(dut.controller_position_target));
            failures = failures + 1;
        end
        force dut.run_mode = 3'd4;
        #1;
        if (dut.controller_position_target !== dut.trajectory_reference_reg) begin
            $display("FAIL: RUN4 did not select the sine reference");
            failures = failures + 1;
        end

        release dut.trajectory_phase;
        dut.trajectory_phase = 12'd0;
        force dut.adc_valid = 1'b1;
        force dut.control_mode = 2'd2;
        force dut.run_active = 1'b1;
        @(posedge clk); #1;
        if (dut.trajectory_phase !== 12'd1) begin
            $display("FAIL: phase did not advance in balance");
            failures = failures + 1;
        end
        phase_i = dut.trajectory_phase;
        force dut.control_mode = 2'd1;
        @(posedge clk); #1;
        if (dut.trajectory_phase !== phase_i[11:0]) begin
            $display("FAIL: phase advanced outside balance");
            failures = failures + 1;
        end
        force dut.control_mode = 2'd2;
        force dut.run_active = 1'b0;
        @(posedge clk); #1;
        if (dut.trajectory_phase !== phase_i[11:0]) begin
            $display("FAIL: phase advanced while paused");
            failures = failures + 1;
        end

        release dut.run_active;
        release dut.control_mode;
        release dut.run_mode;
        release dut.trajectory_center;
        release dut.trajectory_center_goal;
        force dut.run_active = 1'b1;
        dut.run_mode = 3'd4;
        force dut.control_mode = 2'd2;
        dut.trajectory_center = 32'sd500;
        dut.trajectory_center_goal = 32'sd502;
        dut.trajectory_center_slew_divider = 4'd0;
        repeat (9) begin @(posedge clk); #1; end
        if (dut.trajectory_center !== 32'sd500) begin
            $display("FAIL: center moved before its 10-sample slew interval");
            failures = failures + 1;
        end
        @(posedge clk); #1;
        if (dut.trajectory_center !== 32'sd501) begin
            $display("FAIL: center did not slew by one count after 10 samples");
            failures = failures + 1;
        end
        $display("SINE_TRAJECTORY maximum_step=%0d failures=%0d", maximum_step, failures);
        $finish;
    end
endmodule
