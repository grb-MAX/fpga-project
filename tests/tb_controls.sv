`timescale 1ns/1ps

module tb_controls;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg [4:0] buttons_n = 5'b11111;
    wire motor_in1, motor_in2;
    wire oled_scl, oled_sda;
    integer failures = 0;
    reg test_passed = 1'b0;

    always #10 clk = ~clk;

    acg525_pendulum_top dut (
        .clk_50m(clk),
        .key0_n(buttons_n[0]),
        .key1_n(buttons_n[1]),
        .key2_n(rst_n),
        .mode_start_n(buttons_n[2]),
        .target_inc_n(buttons_n[3]),
        .target_dec_n(buttons_n[4]),
        .adc_dout(1'b0),
        .adc_cs_n(),
        .adc_sclk(),
        .motor_in1(motor_in1),
        .motor_in2(motor_in2),
        .encoder_a(1'b0),
        .encoder_b(1'b0),
        .oled_scl(oled_scl),
        .oled_sda(oled_sda),
        .uart_tx(),
        .led()
    );

    task check(input bit condition, input string message);
        if (!condition) begin
            failures = failures + 1;
            $display("FAIL: %s", message);
        end
    endtask

    task press_keys(input [4:0] mask);
        @(negedge clk);
        buttons_n = ~mask;
        repeat (8) @(negedge clk);
        buttons_n = 5'b11111;
        repeat (8) @(negedge clk);
    endtask

    task check_state(input [2:0] mode, input bit active, input string message);
        check(dut.display_mode === mode, {message, ": displayed mode"});
        check(dut.motor_enable === active, {message, ": motor enable"});
        check(dut.run_active === active, {message, ": running flag"});
        if (!active) begin
            check(dut.drive_command === 16'sd0, {message, ": zero command"});
            check({motor_in1, motor_in2} === 2'b00, {message, ": outputs low"});
        end
    endtask

    task check_telemetry_snapshot;
        reg [7:0] checksum;
        integer b;
        begin
            check(dut.uart0.diagnostic_snapshot[7:0] === 8'h55, "diagnostic frame sync 0");
            check(dut.uart0.diagnostic_snapshot[15:8] === 8'hab, "diagnostic frame sync 1");
            check(dut.uart0.diagnostic_snapshot[23:16] === 8'd1, "diagnostic frame version");
            check(dut.uart0.diagnostic_snapshot[31:24] === 8'd44, "diagnostic frame length");
            check(dut.uart0.diagnostic_snapshot[79:64] === {4'd0, dut.telemetry_adc},
                  "diagnostic ADC snapshot");
            check(dut.uart0.diagnostic_snapshot[111:80] === dut.telemetry_position,
                  "diagnostic encoder snapshot");
            check(dut.uart0.diagnostic_snapshot[127:112] === dut.telemetry_velocity,
                  "diagnostic speed snapshot");
            check(dut.uart0.diagnostic_snapshot[143:128] === dut.drive_command,
                  "diagnostic PWM command snapshot");
            check(dut.uart0.diagnostic_snapshot[175:160] === {4'd0, dut.telemetry_zero},
                  "diagnostic FPGA zero snapshot");
            check(dut.uart0.diagnostic_snapshot[191:176] === dut.angle_error,
                  "diagnostic controller error snapshot");
            check(dut.uart0.diagnostic_snapshot[223:192] === dut.telemetry_target,
                  "diagnostic actual target snapshot");
            check(dut.uart0.diagnostic_snapshot[255:224] === dut.debug_angle_pd,
                  "diagnostic angle P-D contribution snapshot");
            check(dut.uart0.diagnostic_snapshot[287:256] === dut.debug_position_applied,
                  "diagnostic applied outer contribution snapshot");
            check(dut.uart0.diagnostic_snapshot[335:304] === dut.debug_raw_control,
                  "diagnostic pre-friction command snapshot");
            checksum = 0;
            for (b = 2; b < 43; b = b + 1)
                checksum = checksum ^ dut.uart0.diagnostic_snapshot[b*8 +: 8];
            check(dut.uart0.diagnostic_snapshot[351:344] === checksum,
                  "diagnostic snapshot XOR");
        end
    endtask

    task check_header(input [119:0] expected);
        reg [119:0] actual;
        reg [7:0] character;
        reg [7:0] pixels;
        integer i, c;
        // Skip I2C initialization/transfer time but exercise frame snapshotting.
        @(negedge clk);
        force dut.oled0.state = 5'd1;
        force dut.oled0.init_done = 1'b1;
        force dut.display_tick = 1'b1;
        @(negedge clk);
        release dut.display_tick;
        release dut.oled0.state;
        release dut.oled0.init_done;
        actual = 120'd0;
        for (i = 0; i < 15; i = i + 1) begin
            character = dut.oled0.line_char(2'd0, i[4:0]);
            actual = {actual[111:0], character};
            pixels = 8'd0;
            for (c = 0; c < 5; c = c + 1)
                pixels = pixels | dut.oled0.glyph_column(character, c[2:0]);
            if (character != " ")
                check(pixels != 0, $sformatf("visible glyph for %c", character));
            else
                check(pixels == 0, "space glyph is blank");
        end
        check(actual === expected, $sformatf("OLED header: got '%s', expected '%s'", actual, expected));
    endtask

    initial begin
        // Accelerate debounce samples; sensor protocol is outside this regression.
        force dut.adc_valid = 1'b1;
        force dut.adc_value = 12'd2048;
        force dut.encoder_position = 32'sd1234;
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        repeat (8) @(negedge clk);
        check_telemetry_snapshot();
        check_state(0, 0, "reset");
        check_header("MODE:SAFE STOP ");

        press_keys(5'b00001);
        check_state(1, 0, "select RUN1");
        check_header("MODE:RUN1 STOP ");
        press_keys(5'b00100);
        check_state(1, 1, "start RUN1");
        check(dut.drive_command === 16'sd153, "RUN1 calibrated command");
        check_header("MODE:RUN1 START");
        press_keys(5'b00001);
        check_state(1, 1, "mode switch locked while running");
        check(dut.selected_mode === 3'd1, "running selection unchanged");
        press_keys(5'b00100);
        check_state(1, 0, "pause RUN1");
        check(dut.run_mode === 3'd1, "pause retains executed mode");
        check_header("MODE:RUN1 STOP ");

        force dut.adc_value = 12'd2100;
        press_keys(5'b00010);
        check(dut.angle_zero === 12'd2100, "angle zero capture");
        press_keys(5'b00001);
        check_state(2, 0, "switch to RUN2 while paused");
        check_header("MODE:RUN2 STOP ");
        press_keys(5'b00100);
        check_state(2, 1, "start new selection RUN2");
        check(dut.drive_command === -16'sd150, "RUN2 calibrated command");
        check(dut.angle_zero === 12'd2100, "switch does not reset angle zero");
        check_header("MODE:RUN2 START");
        press_keys(5'b00100);
        check_state(2, 0, "pause RUN2");
        press_keys(5'b00001);
        check_state(3, 0, "select RUN3");
        check_header("MODE:RUN3 STOP ");
        press_keys(5'b00100);
        check_state(3, 1, "start RUN3");
        check(dut.target_position === 32'sd1234, "RUN3 initial target");
        // 当前角度参数为六和十五；位置不变时，新外环应为零。
        force dut.adc_value = 12'd2140;
        repeat (12) @(negedge clk);
        check(dut.drive_command === 16'sd393, "RUN3 positive friction compensation");
        force dut.adc_value = 12'd2060;
        repeat (12) @(negedge clk);
        check(dut.drive_command === -16'sd390, "RUN3 negative friction compensation");
        force dut.adc_value = 12'd2120;
        repeat (12) @(negedge clk);
        check(dut.drive_command === 16'sd263, "20-count positive error exceeds measured start threshold");
        force dut.adc_value = 12'd2080;
        repeat (12) @(negedge clk);
        check(dut.drive_command === -16'sd260, "20-count negative error exceeds measured start threshold");
        force dut.adc_value = 12'd2100;
        repeat (12) @(negedge clk);
        check(dut.drive_command === 16'sd0, "RUN3 zero stays zero");
        force dut.encoder_position = 32'sd2234;
        force dut.encoder_velocity = 32'sd5;
        repeat (96) @(negedge clk);
        check(dut.controller0.position_feedback < 0 && dut.controller0.position_feedback >= -64,
              "新外环对正位置偏移产生有界的反向状态反馈");
        check(dut.drive_command < 0, "顶层弱位置反馈朝目标方向回拉");
        force dut.encoder_position = 32'sd1234;
        force dut.encoder_velocity = 32'sd0;
        repeat (256) @(negedge clk);
        check(dut.controller0.position_feedback === 64'sd0, "回到目标且静止后外环归零");
        force dut.adc_value = 12'd2101;
        @(negedge clk);
        check(dut.drive_command === 16'sd46, "positive angle rate increases correction");
        repeat (12) @(negedge clk);
        check(dut.drive_command === 16'sd13, "positive derivative contribution decays when angle stops changing");
        force dut.adc_value = 12'd2100;
        repeat (12) @(negedge clk);
        force dut.adc_value = 12'd2099;
        @(negedge clk);
        check(dut.drive_command === -16'sd45, "negative angle rate increases correction");
        repeat (12) @(negedge clk);
        check(dut.drive_command === -16'sd13, "negative derivative contribution decays when angle stops changing");
        force dut.adc_value = 12'd2100;
        repeat (12) @(negedge clk);
        force dut.control_command = 16'sd123;
        #1 check(dut.drive_command === 16'sd123, "RUN3 command direction");
        release dut.control_command;
        check_header("MODE:RUN3 START");
        press_keys(5'b01000);
        check(dut.target_position === 32'sd1334, "increase target");
        press_keys(5'b00100);
        check_state(3, 0, "pause RUN3");
        force dut.encoder_position = 32'sd2000;
        press_keys(5'b00100);
        check_state(3, 1, "resume same RUN3");
        check(dut.target_position === 32'sd1334, "resume preserves target");
        press_keys(5'b00100);
        press_keys(5'b00001);
        check_state(0, 0, "switch to SAFE while paused");
        check_header("MODE:SAFE STOP ");
        press_keys(5'b00100);
        check_state(0, 0, "SAFE cannot start motor");
        check(dut.target_position === 32'sd1334, "SAFE selection is not reset");

        press_keys(5'b00001);
        press_keys(5'b00101);
        check_state(2, 0, "simultaneous selection and start stays stopped");
        press_keys(5'b00100);
        check_state(2, 1, "start after simultaneous press");
        rst_n = 1'b0;
        repeat (3) @(negedge clk);
        check_state(0, 0, "KEY2 reset while running");
        check(dut.target_position === 32'sd0, "KEY2 resets target");
        check(dut.angle_zero === 12'd3153, "KEY2 resets angle zero");

        test_passed = (failures == 0);
        $display("Control/OLED regression: %0d failures", failures);
        $finish;
    end

    initial begin
        #100000;
        $display("FAIL: test timeout");
        $finish;
    end
endmodule
