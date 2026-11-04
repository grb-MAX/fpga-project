`timescale 1ns/1ps

// 当前正弦目标层回归：只验证位置参考，不驱动 PWM。
module tb_sine_trajectory_current;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg sample_tick = 1'b0;
    reg start_request = 1'b0;
    reg clear_request = 1'b0;
    reg advance_enable = 1'b0;
    reg signed [31:0] encoder_position = 32'sd500;
    wire signed [31:0] trajectory_ref;
    integer failures = 0;
    integer i;
    integer previous_offset;
    integer current_offset;
    integer maximum_step;

    always #10 clk = ~clk;

    position_trajectory #(
        .AMPLITUDE_COUNTS(100),
        .PERIOD_SAMPLES(4096)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .sample_tick(sample_tick),
        .start_request(start_request),
        .clear_request(clear_request),
        .advance_enable(advance_enable),
        .encoder_position(encoder_position),
        .trajectory_ref(trajectory_ref)
    );

    task tick;
        begin
            sample_tick = 1'b1;
            @(posedge clk);
            #1;
            sample_tick = 1'b0;
        end
    endtask

    initial begin
        #100;
        rst_n = 1'b1;

        start_request = 1'b1;
        tick;
        start_request = 1'b0;
        if ($signed(trajectory_ref) !== 32'sd500) begin
            $display("FAIL: start center reference=%0d", $signed(trajectory_ref));
            failures = failures + 1;
        end

        // 轨迹在有效推进时保持在中心 ±100，且相邻采样最多变化 1 count。
        advance_enable = 1'b1;
        maximum_step = 0;
        previous_offset = $signed(trajectory_ref) - 500;
        for (i = 0; i < 4096; i = i + 1) begin
            tick;
            current_offset = $signed(trajectory_ref) - 500;
            if ((current_offset > 100) || (current_offset < -100)) begin
                $display("FAIL: sine range offset=%0d", current_offset);
                failures = failures + 1;
            end
            if ((current_offset - previous_offset > maximum_step) ||
                (previous_offset - current_offset > maximum_step))
                maximum_step = (current_offset > previous_offset) ?
                               (current_offset - previous_offset) :
                               (previous_offset - current_offset);
            previous_offset = current_offset;
        end
        if (maximum_step > 1) begin
            $display("FAIL: sine target step=%0d", maximum_step);
            failures = failures + 1;
        end

        // 禁止推进时参考必须冻结。
        advance_enable = 1'b0;
        current_offset = $signed(trajectory_ref);
        repeat (20) tick;
        if ($signed(trajectory_ref) !== current_offset) begin
            $display("FAIL: trajectory advanced while disabled");
            failures = failures + 1;
        end

        clear_request = 1'b1;
        tick;
        clear_request = 1'b0;
        encoder_position = 32'sd1234;
        start_request = 1'b1;
        tick;
        start_request = 1'b0;
        if ($signed(trajectory_ref) !== 32'sd1234) begin
            $display("FAIL: restart center reference=%0d", $signed(trajectory_ref));
            failures = failures + 1;
        end

        $display("SINE_TRAJECTORY_CURRENT maximum_step=%0d failures=%0d", maximum_step, failures);
        if (failures != 0)
            $fatal(1, "current sine trajectory regression failed");
        $finish;
    end
endmodule
