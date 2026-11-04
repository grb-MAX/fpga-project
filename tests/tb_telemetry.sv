`timescale 1ns/1ps
module tb_telemetry;
    reg clk = 0, rst_n = 0, diagnostic_tick = 0;
    always #5 clk = ~clk;
    reg [11:0] adc = 4095, zero = 3162;
    reg signed [31:0] position = -123456789, velocity = -32760;
    reg signed [15:0] command = -900, error = -37, rate = -12;
    reg [1:0] mode = 2;
    reg [2:0] run_mode = 3;
    reg [31:0] sample_id = 32'h87654321;
    reg signed [31:0] target = 7654321, pd = -70000, outer = -64, raw = -70064;
    reg [7:0] flags = 8'haf, window_count = 31;
    reg [31:0] adc_glitch_count = 32'h12345678, adc_max_delta = 32'd1023, limit_events = 32'd77;
    wire diagnostic_tx, legacy_tx;
    integer failures = 0, i, j, f;
    reg test_passed = 0;
    reg [7:0] diagnostic_bytes [0:87];
    reg [7:0] stats_bytes [0:21];
    reg [7:0] legacy_bytes [0:27];
    reg [7:0] expected_d [0:87];
    reg [7:0] expected_l [0:27];

    uart_telemetry_tx #(.CLK_HZ(1000), .BAUD(100), .REPORT_PERIOD(100), .DIAGNOSTIC_ENABLE(1), .STATS_EVERY(3)) diagnostic (
        .clk(clk), .rst_n(rst_n), .adc_value(adc), .encoder_position(position),
        .encoder_velocity(velocity), .command(command), .control_mode(mode), .run_mode(run_mode),
        .diagnostic_tick(diagnostic_tick), .sample_id(sample_id), .angle_zero(zero),
        .angle_error(error), .position_target(target), .angle_pd(pd), .position_applied(outer),
        .balance_rate(rate), .raw_control(raw), .diagnostic_flags(flags),
        .window_count(window_count), .adc_glitch_count(adc_glitch_count),
        .adc_max_delta(adc_max_delta), .limit_events(limit_events), .uart_tx(diagnostic_tx)
    );
    uart_telemetry_tx #(.CLK_HZ(1000), .BAUD(100), .REPORT_PERIOD(100)) legacy (
        .clk(clk), .rst_n(rst_n), .adc_value(adc), .encoder_position(position),
        .encoder_velocity(velocity), .command(command), .control_mode(mode), .run_mode(run_mode),
        .diagnostic_tick(1'b0), .sample_id(32'd0), .angle_zero(12'd0),
        .angle_error(16'sd0), .position_target(32'sd0), .angle_pd(32'sd0),
        .position_applied(32'sd0), .balance_rate(16'sd0), .raw_control(32'sd0),
        .diagnostic_flags(8'd0), .window_count(8'd0),
        .adc_glitch_count(32'd0), .adc_max_delta(32'd0), .limit_events(32'd0),
        .uart_tx(legacy_tx)
    );
    task check(input bit condition, input string message);
        if (!condition) begin failures++; $display("FAIL: %s", message); end
    endtask
    // 从真实串口线在各数据位中心接收，不读取发送器内部字节寄存器。
    task automatic receive_byte(input bit diag, output reg [7:0] value);
        integer bit_no;
        if (diag) @(negedge diagnostic_tx); else @(negedge legacy_tx);
        repeat (15) @(posedge clk);
        #1;
        for (bit_no = 0; bit_no < 8; bit_no++) begin
            value[bit_no] = diag ? diagnostic_tx : legacy_tx;
            repeat (10) @(posedge clk);
            #1;
        end
        check((diag ? diagnostic_tx : legacy_tx) === 1'b1, "停止位必须为高");
    endtask
    task pulse_snapshot;
        @(negedge clk); diagnostic_tick = 1;
        @(negedge clk); diagnostic_tick = 0;
    endtask
    // 逐字段独立写入期望数据，核对协议字节偏移和符号。
    task make_expected(input integer n);
        reg [7:0] checksum;
        integer b, d, l;
        d = n * 44; l = n * 14;
        expected_d[d] = 8'h55; expected_d[d+1] = 8'hab;
        expected_d[d+2] = 1; expected_d[d+3] = 44;
        for (b=0; b<4; b++) expected_d[d+4+b] = sample_id >> (8*b);
        expected_d[d+8] = adc; expected_d[d+9] = adc >> 8;
        for (b=0; b<4; b++) expected_d[d+10+b] = position >> (8*b);
        expected_d[d+14] = velocity; expected_d[d+15] = velocity >> 8;
        expected_d[d+16] = command; expected_d[d+17] = command >> 8;
        expected_d[d+18] = (mode << 4) | run_mode; expected_d[d+19] = flags;
        expected_d[d+20] = zero; expected_d[d+21] = zero >> 8;
        expected_d[d+22] = error; expected_d[d+23] = error >> 8;
        for (b=0; b<4; b++) begin
            expected_d[d+24+b] = target >> (8*b);
            expected_d[d+28+b] = pd >> (8*b);
            expected_d[d+32+b] = outer >> (8*b);
            expected_d[d+38+b] = raw >> (8*b);
        end
        expected_d[d+36] = rate; expected_d[d+37] = rate >> 8;
        expected_d[d+42] = window_count;
        checksum = 0;
        for (b=2; b<43; b++) checksum ^= expected_d[d+b];
        expected_d[d+43] = checksum;
        expected_l[l] = 8'h55; expected_l[l+1] = 8'haa;
        for (b=0; b<11; b++) expected_l[l+2+b] = expected_d[d+8+b];
        checksum = 0;
        for (b=2; b<13; b++) checksum ^= expected_l[l+b];
        expected_l[l+13] = checksum;
    endtask
    initial begin
        make_expected(0);
        repeat (3) @(negedge clk); rst_n = 1;
        fork
            begin
                for (i=0; i<88; i++) receive_byte(1, diagnostic_bytes[i]);
            end
            begin
                for (j=0; j<28; j++) receive_byte(0, legacy_bytes[j]);
            end
            begin
                pulse_snapshot();
                wait(diagnostic.tx_busy);
                repeat (200) @(negedge clk);
                // 在第一帧传输中改变所有字段；第二快照不得混入第一帧。
                adc=7; zero=4094; position=2147483647; velocity=32767;
                command=389; error=9; rate=15; mode=1; run_mode=2;
                sample_id=32'hfffffff0; target=-2147483647;
                pd=2222; outer=64; raw=2286; flags=8'hc3; window_count=0;
                make_expected(1);
                pulse_snapshot();
                // 未触发采样的输入变化也不得混入第二诊断快照。
                @(negedge clk);
                zero=123; error=123; target=123; pd=123; outer=123;
                rate=123; raw=123; flags=123; window_count=123;
            end
        join
        fork
            begin
                for (i=0; i<22; i++) receive_byte(1, stats_bytes[i]);
            end
            begin
                wait(diagnostic.tx_busy);
            end
        join
        check(stats_bytes[0] === 8'h55 && stats_bytes[1] === 8'hac, "统计帧同步字节");
        check(stats_bytes[2] === 8'd1 && stats_bytes[3] === 8'd22, "统计帧版本和长度");
        check(stats_bytes[4] === sample_id[7:0] && stats_bytes[7] === sample_id[31:24], "统计帧采样序号");
        check(stats_bytes[8] === adc_glitch_count[7:0] && stats_bytes[11] === adc_glitch_count[31:24], "统计帧毛刺计数");
        check(stats_bytes[12] === adc_max_delta[7:0] && stats_bytes[13] === adc_max_delta[15:8], "统计帧最大差分");
        check(stats_bytes[14] === limit_events[7:0] && stats_bytes[17] === limit_events[31:24], "统计帧限幅计数");
        check(stats_bytes[18] === command[7:0] && stats_bytes[19] === command[15:8], "统计帧命令");
        begin : check_stats_xor
            reg [7:0] checksum;
            checksum = 0;
            for (i=2; i<21; i++) checksum = checksum ^ stats_bytes[i];
            check(stats_bytes[21] === checksum, "统计帧异或校验");
        end
        for (i=0; i<88; i++) check(diagnostic_bytes[i] === expected_d[i],
            $sformatf("DIAG_BYTE index=%0d actual=%02h expected=%02h", i, diagnostic_bytes[i], expected_d[i]));
        for (i=0; i<28; i++) check(legacy_bytes[i] === expected_l[i],
            $sformatf("LEGACY_BYTE index=%0d actual=%02h expected=%02h", i, legacy_bytes[i], expected_l[i]));
        f = $fopen("tests/telemetry_frames.hex", "w");
        for (i=0; i<44; i++) $fwrite(f, "%02h", diagnostic_bytes[i]); $fwrite(f, "\n");
        for (i=0; i<14; i++) $fwrite(f, "%02h", legacy_bytes[i]); $fwrite(f, "\n");
        for (i=44; i<88; i++) $fwrite(f, "%02h", diagnostic_bytes[i]); $fwrite(f, "\n");
        for (i=14; i<28; i++) $fwrite(f, "%02h", legacy_bytes[i]); $fwrite(f, "\n");
        for (i=0; i<22; i++) $fwrite(f, "%02h", stats_bytes[i]); $fwrite(f, "\n");
        $fclose(f);
        test_passed = failures == 0;
        $display("TELEMETRY failures=%0d", failures);
        $finish;
    end
    initial begin #2000000; $fatal(1, "UART_TIMEOUT"); end
endmodule
