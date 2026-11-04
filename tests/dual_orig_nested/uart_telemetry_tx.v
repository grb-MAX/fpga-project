// 只读UART遥测：115200、8N1，所有多字节字段均低字节先发。
// 默认保留55 AA旧14字节帧；顶层发送55 AB诊断44字节帧，并每64帧附发55 AC统计帧。
module uart_telemetry_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 115_200,
    parameter integer REPORT_PERIOD = 500_000, // 只累计帧间空闲时间，发送耗时另计。
    parameter integer DIAGNOSTIC_ENABLE = 0,
    parameter integer STATS_EVERY = 64
) (
    input wire clk,
    input wire rst_n,
    input wire [11:0] adc_value,
    input wire signed [31:0] encoder_position,
    input wire signed [31:0] encoder_velocity,
    input wire signed [15:0] command,
    input wire [1:0] control_mode,
    input wire [2:0] run_mode,
    input wire diagnostic_tick, // 顶层在控制采样后一系统时钟发出，以下字段此时对应同一控制采样。
    input wire [31:0] sample_id,
    input wire [11:0] angle_zero,
    input wire signed [15:0] angle_error,
    input wire signed [31:0] position_target,
    input wire signed [31:0] angle_pd,
    input wire signed [31:0] position_applied,
    input wire signed [15:0] balance_rate,
    input wire signed [31:0] raw_control,
    input wire [7:0] diagnostic_flags,
    input wire [7:0] window_count,
    input wire [31:0] adc_glitch_count,
    input wire [31:0] adc_max_delta,
    input wire [31:0] limit_events,
    output reg uart_tx
);
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam integer LEGACY_FRAME_SIZE = 14;
    localparam integer DIAGNOSTIC_FRAME_SIZE = 44;
    localparam integer STATS_FRAME_SIZE = 22;
    reg [31:0] report_timer;
    reg [7:0] report_count;
    reg [5:0] active_frame_size;
    reg [15:0] baud_timer;
    reg [5:0] frame_index;
    reg [3:0] bit_index;
    reg [7:0] tx_byte;
    reg tx_busy;
    reg diagnostic_valid;
    reg [351:0] diagnostic_snapshot;
    reg [167:0] stats_snapshot;
    reg [351:0] packet_latched;

    wire [7:0] packed_mode = {2'd0, control_mode, 1'd0, run_mode};
    wire [103:0] legacy_payload = {packed_mode, command, encoder_velocity[15:0],
                                  encoder_position, 4'd0, adc_value, 16'hAA55};
    // 字节0..3：55 AB、版本1、长度44；4..7：控制采样序号。
    // 8..18：ADC、位置、速度低16位、最终命令、模式；19：诊断标志。
    // 20..27：FPGA零点、实际角度误差、实际目标位置；28..35：角度PD、已叠加外环。
    // 36..42：角度D差分、摩擦前合成命令、外环窗口计数；43：字节2..42异或校验。
    wire [343:0] diagnostic_payload = {
        window_count, raw_control, balance_rate, position_applied, angle_pd,
        position_target, angle_error, 4'd0, angle_zero, diagnostic_flags, packed_mode,
        command, encoder_velocity[15:0], encoder_position, 4'd0, adc_value,
        sample_id, 8'd44, 8'd1, 16'hAB55
    };

    // 独立统计帧：55 AC、版本1、长度22；字段为采样序号、ADC毛刺计数、最大差分、限幅次数、命令和标志。
    wire [167:0] stats_payload = {
        diagnostic_flags, command, limit_events, adc_max_delta[15:0],
        adc_glitch_count, sample_id, 8'd22, 8'd1, 16'hAC55
    };

    function [7:0] legacy_xor;
        input [103:0] data;
        integer i;
        begin
            legacy_xor = 8'd0;
            for (i = 2; i < 13; i = i + 1)
                legacy_xor = legacy_xor ^ data[i*8 +: 8];
        end
    endfunction

    function [7:0] diagnostic_xor;
        input [343:0] data;
        integer i;
        begin
            diagnostic_xor = 8'd0;
            for (i = 2; i < 43; i = i + 1)
                diagnostic_xor = diagnostic_xor ^ data[i*8 +: 8];
        end
    endfunction

    function [7:0] stats_xor;
        input [167:0] data;
        integer i;
        begin
            stats_xor = 8'd0;
            for (i = 2; i < 21; i = i + 1)
                stats_xor = stats_xor ^ data[i*8 +: 8];
        end
    endfunction

    function [7:0] frame_byte;
        input [5:0] index;
        begin
            frame_byte = packet_latched[index*8 +: 8];
        end
    endfunction

    always @* begin
        if (!tx_busy) uart_tx = 1'b1;
        else if (bit_index == 0) uart_tx = 1'b0;
        else if (bit_index <= 8) uart_tx = tx_byte[bit_index - 1'b1];
        else uart_tx = 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            report_timer <= 0;
            report_count <= 0;
            active_frame_size <= DIAGNOSTIC_ENABLE ? DIAGNOSTIC_FRAME_SIZE : LEGACY_FRAME_SIZE;
            baud_timer <= 0;
            frame_index <= 0;
            bit_index <= 0;
            tx_byte <= 0;
            tx_busy <= 0;
            diagnostic_valid <= 0;
            diagnostic_snapshot <= 0;
            stats_snapshot <= 0;
            packet_latched <= 0;
        end else begin
            // 即使串口正在发送，仍更新独立的完整控制快照；绝不在一帧中途替换字段。
            if (DIAGNOSTIC_ENABLE && diagnostic_tick) begin
                diagnostic_snapshot <= {diagnostic_xor(diagnostic_payload), diagnostic_payload};
                stats_snapshot <= stats_payload;
                diagnostic_valid <= 1'b1;
            end
            if (!tx_busy) begin
                baud_timer <= 0;
                if ((report_timer == REPORT_PERIOD - 1) &&
                    (!DIAGNOSTIC_ENABLE || diagnostic_valid)) begin
                    report_timer <= 0;
                    if (!DIAGNOSTIC_ENABLE) begin
                        packet_latched <= {legacy_xor(legacy_payload), legacy_payload};
                        active_frame_size <= LEGACY_FRAME_SIZE;
                    end else if (report_count == STATS_EVERY - 1) begin
                        packet_latched <= {176'd0, stats_xor(stats_snapshot), stats_snapshot};
                        active_frame_size <= STATS_FRAME_SIZE;
                        report_count <= 0;
                    end else begin
                        packet_latched <= diagnostic_snapshot;
                        active_frame_size <= DIAGNOSTIC_FRAME_SIZE;
                        report_count <= report_count + 1'b1;
                    end
                    frame_index <= 0;
                    bit_index <= 0;
                    tx_byte <= 8'h55;
                    tx_busy <= 1'b1;
                end else if (report_timer != REPORT_PERIOD - 1)
                    report_timer <= report_timer + 1'b1;
            end else if (baud_timer == CLKS_PER_BIT - 1) begin
                baud_timer <= 0;
                if (bit_index == 9) begin
                    if (frame_index == active_frame_size - 1'b1)
                        tx_busy <= 1'b0;
                    else begin
                        frame_index <= frame_index + 1'b1;
                        bit_index <= 0;
                        tx_byte <= frame_byte(frame_index + 1'b1);
                    end
                end else
                    bit_index <= bit_index + 1'b1;
            end else
                baud_timer <= baud_timer + 1'b1;
        end
    end
endmodule
