`timescale 1ns/1ps

module tb_mcp3201;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    tri1 adc_dout;
    wire cs_n, sclk, valid, busy;
    wire [11:0] data;
    reg model_drive = 1'b0;
    reg model_bit = 1'b0;
    reg [11:0] next_code = 0;
    reg [11:0] held_code = 0;
    integer falling_edges = 0;
    integer rising_edges = 0;
    integer failures = 0;
    integer sample_count = 0;
    integer i;
    reg test_passed = 1'b0;
    time last_start = 0;
    time last_clock_edge = 0;

    always #10 clk = ~clk;
    assign adc_dout = model_drive ? model_bit : 1'bz;

    // 25 ksps leaves ample room for all 16 SPI clocks at the default divider.
    mcp3201_spi #(.SAMPLE_HZ(25_000)) dut (
        .clk(clk), .rst_n(rst_n), .adc_dout(adc_dout),
        .adc_cs_n(cs_n), .adc_sclk(sclk), .adc_data(data),
        .adc_valid(valid), .busy(busy)
    );

    task check(input bit condition, input string message);
        if (!condition) begin
            failures++;
            $display("FAIL: %s", message);
        end
    endtask

    always @(negedge cs_n) begin
        held_code = next_code;
        falling_edges = 0;
        rising_edges = 0;
        model_drive = 1'b0;
        last_clock_edge = 0;
        if (last_start != 0)
            check($time - last_start == 40_000, "fixed 25 ksps start-to-start cadence");
        last_start = $time;
    end

    always @(posedge sclk) begin
        if (!cs_n) begin
            rising_edges++;
            if (last_clock_edge != 0)
                check($time - last_clock_edge >= 625, "SPI half-period supports low-voltage ADC");
            last_clock_edge = $time;
        end
    end

    // DS21290F Figure 6-1: two Hi-Z samples, null, B11..B0, then repeated B1.
    // Device data changes on falling edges with up to 200 ns propagation delay.
    always @(negedge sclk) begin
        if (!cs_n) begin
            falling_edges++;
            if (last_clock_edge != 0)
                check($time - last_clock_edge >= 625, "SPI high time supports low-voltage ADC");
            last_clock_edge = $time;
            #200;
            if (!cs_n) begin
                if (falling_edges == 2) begin
                    model_drive = 1'b1;
                    model_bit = 1'b0;
                end else if (falling_edges >= 3 && falling_edges <= 14) begin
                    model_bit = held_code[14 - falling_edges];
                end else if (falling_edges == 15) begin
                    model_bit = held_code[1];
                end
            end
        end
    end

    always @(posedge cs_n) model_drive = 1'b0;

    task expect_sample(input [11:0] expected);
        @(negedge clk);
        next_code = expected;
        @(posedge valid);
        #1;
        sample_count++;
        check(data === expected,
              $sformatf("ADC code: got %03h, expected %03h", data, expected));
        check(cs_n === 1'b1 && busy === 1'b0 && sclk === 1'b0,
              "conversion completes with CS high, SCLK low, busy low");
        check(rising_edges == 16, "exactly 16 rising edges per conversion");
        @(posedge clk);
        #1;
        check(valid === 1'b0, "valid is a single system-clock pulse");
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        expect_sample(12'h000);
        expect_sample(12'hfff);
        expect_sample(12'h001);
        expect_sample(12'h002);
        expect_sample(12'h800);
        expect_sample(12'h7ff);
        expect_sample(12'ha55);
        expect_sample(12'h5aa);
        for (i = 0; i < 12; i++) begin
            expect_sample(12'h001 << i);
            expect_sample(12'hfff ^ (12'h001 << i));
        end
        @(negedge cs_n);
        repeat (50) @(negedge clk);
        rst_n = 1'b0;
        #1;
        check(cs_n === 1'b1 && sclk === 1'b0 && valid === 1'b0 && busy === 1'b0,
              "reset aborts an in-progress conversion");
        test_passed = (failures == 0);
        $display("MCP3201 regression: %0d samples, %0d failures", sample_count, failures);
        $finish;
    end

    initial begin
        #3000000;
        $fatal(1, "MCP3201 test timeout");
    end
endmodule
