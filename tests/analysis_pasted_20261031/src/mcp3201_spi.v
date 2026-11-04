// MCP3201 SPI 采集模块。默认 50 MHz 系统时钟、每秒 1000 次采样。
// adc_valid 在新 adc_data 更新时产生一个 clk 周期脉冲。
module mcp3201_spi #(
    parameter integer CLK_HZ = 50_000_000, // 输入主时钟频率，单位为 Hz，默认 50 MHz。
    parameter integer SAMPLE_HZ = 1_000, // 目标采样率，默认 1000 Hz，即相邻转换启动间隔 1 ms。
    // 50 MHz / (2*32) = 781.25 kHz，适用于当前 3.3 V 供电。
    parameter integer SCLK_DIV = 32 // 串行时钟每个半周期的系统时钟数，默认 32。
) (
    input  wire        clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire        rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire        adc_dout, // MCP3201 串行数据输入，在规定的 SCLK 上升沿采集。
    output reg         adc_cs_n, // 低有效片选；空闲保持高，读取期间拉低。
    output reg         adc_sclk, // SPI 时钟输出；空闲为低，传输期间按分频计数翻转。
    output reg [11:0]  adc_data, // 本次已完成读取的 12 位结果，传输过程中保持上次值。
    output reg         adc_valid, // 采样结果更新事件，仅持续一个系统时钟。
    output reg         busy // 一次 SPI 读取正在进行的状态，高电平表示忙。
);

    // 两次采样之间的系统时钟数；默认 50,000 个时钟，即 1 ms。
    localparam integer SAMPLE_PERIOD = CLK_HZ / SAMPLE_HZ; // 两次转换启动之间的系统时钟数，默认 50,000。

    reg [31:0] sample_timer; // 持续运行的采样周期计数器，SPI 忙时也继续计时。
    reg [31:0] sclk_timer; // SPI 半周期分频计数器，计满后翻转一次 SCLK。
    reg [4:0]  rise_count; // 本次传输已产生的上升沿数；判断时使用更新前的计数值。
    reg [11:0] capture_data; // 接收移位暂存区，按位写入 B11~B0，读完后一次性发布。

    // 所有状态在系统时钟域更新，SCLK 只是输出信号，不作为内部逻辑时钟。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_timer <= 32'd0;
            sclk_timer   <= 32'd0;
            rise_count   <= 5'd0;
            capture_data <= 12'd0;
            adc_data     <= 12'd0;
            adc_valid    <= 1'b0;
            adc_cs_n     <= 1'b1;
            adc_sclk     <= 1'b0;
            busy         <= 1'b0;
        end else begin
            // 默认清除结果有效标志，仅在完整传输结束时置位一个系统时钟。
            adc_valid <= 1'b0;

            // 采样周期包含 SPI 传输时间，避免忙状态延长相邻采样的间隔。
            if (sample_timer == SAMPLE_PERIOD - 1)
                sample_timer <= 32'd0;
            else
                sample_timer <= sample_timer + 1'b1;

            if (!busy) begin
                adc_cs_n   <= 1'b1;
                adc_sclk   <= 1'b0;
                sclk_timer <= 32'd0;

                // 到达采样周期后拉低 CS，启动一次 SPI 读取。
                if (sample_timer == SAMPLE_PERIOD - 1) begin
                    busy         <= 1'b1;
                    adc_cs_n     <= 1'b0;
                    rise_count   <= 5'd0;
                    capture_data <= 12'd0;
                end
            end else if (sclk_timer == SCLK_DIV - 1) begin
                sclk_timer <= 32'd0;

                if (!adc_sclk) begin
                    adc_sclk <= 1'b1;
                    // DS21290F 图 6-1：前两次上升沿为高阻，第三次为空位；
                    // 第 4 到第 15 次上升沿读取 B11..B0，rise_count 从 0 开始。
                    if ((rise_count >= 3) && (rise_count <= 14)) begin
                        capture_data[14 - rise_count] <= adc_dout;
                    end
                    rise_count <= rise_count + 1'b1;
                end else begin
                    adc_sclk <= 1'b0;
                    // 第 16 个串行时钟下降沿结束事务；此时 12 位暂存值已经完整。
                    if (rise_count == 16) begin
                        adc_cs_n  <= 1'b1;
                        busy      <= 1'b0;
                        adc_data  <= capture_data;
                        adc_valid <= 1'b1;
                    end
                end
            end else begin
                sclk_timer <= sclk_timer + 1'b1;
            end
        end
    end

endmodule
