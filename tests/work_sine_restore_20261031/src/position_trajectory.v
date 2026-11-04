
`timescale 1ns / 1ps

module position_trajectory #(
    parameter integer AMPLITUDE_COUNTS = 50,
    parameter integer PERIOD_SAMPLES = 30000
)(
    input wire clk,
    input wire rst_n,
    input wire sample_tick,
    input wire start,
    input wire enable,
    input wire signed [31:0] encoder_position,
    output reg signed [31:0] position_ref
);

    localparam [31:0] PHASE_STEP =
        (64'd4294967296 + PERIOD_SAMPLES/2)
        / PERIOD_SAMPLES;

    reg running;
    reg [31:0] phase;
    reg [31:0] pending_phase;
    reg [2:0] stage;

    reg signed [31:0] origin_position;
    reg signed [15:0] sin0;
    reg signed [15:0] sin1;
    reg signed [15:0] interp_sin;

    reg [15:0] fraction;

    reg signed [47:0] interp_product;
    reg signed [47:0] amplitude_product;
    reg signed [31:0] relative_count;

    // 四分之一周期正弦表，Q15
    function signed [15:0] quarter_sine;
        input [6:0] index;
        begin
            case (index)
                7'd0: quarter_sine = 16'sd0;
                7'd1: quarter_sine = 16'sd804;
                7'd2: quarter_sine = 16'sd1608;
                7'd3: quarter_sine = 16'sd2410;
                7'd4: quarter_sine = 16'sd3212;
                7'd5: quarter_sine = 16'sd4011;
                7'd6: quarter_sine = 16'sd4808;
                7'd7: quarter_sine = 16'sd5602;
                7'd8: quarter_sine = 16'sd6393;
                7'd9: quarter_sine = 16'sd7179;
                7'd10: quarter_sine = 16'sd7962;
                7'd11: quarter_sine = 16'sd8739;
                7'd12: quarter_sine = 16'sd9512;
                7'd13: quarter_sine = 16'sd10278;
                7'd14: quarter_sine = 16'sd11039;
                7'd15: quarter_sine = 16'sd11793;
                7'd16: quarter_sine = 16'sd12539;
                7'd17: quarter_sine = 16'sd13279;
                7'd18: quarter_sine = 16'sd14010;
                7'd19: quarter_sine = 16'sd14732;
                7'd20: quarter_sine = 16'sd15446;
                7'd21: quarter_sine = 16'sd16151;
                7'd22: quarter_sine = 16'sd16846;
                7'd23: quarter_sine = 16'sd17530;
                7'd24: quarter_sine = 16'sd18204;
                7'd25: quarter_sine = 16'sd18868;
                7'd26: quarter_sine = 16'sd19519;
                7'd27: quarter_sine = 16'sd20159;
                7'd28: quarter_sine = 16'sd20787;
                7'd29: quarter_sine = 16'sd21403;
                7'd30: quarter_sine = 16'sd22005;
                7'd31: quarter_sine = 16'sd22594;
                7'd32: quarter_sine = 16'sd23170;
                7'd33: quarter_sine = 16'sd23731;
                7'd34: quarter_sine = 16'sd24279;
                7'd35: quarter_sine = 16'sd24811;
                7'd36: quarter_sine = 16'sd25329;
                7'd37: quarter_sine = 16'sd25832;
                7'd38: quarter_sine = 16'sd26319;
                7'd39: quarter_sine = 16'sd26790;
                7'd40: quarter_sine = 16'sd27245;
                7'd41: quarter_sine = 16'sd27683;
                7'd42: quarter_sine = 16'sd28105;
                7'd43: quarter_sine = 16'sd28510;
                7'd44: quarter_sine = 16'sd28898;
                7'd45: quarter_sine = 16'sd29268;
                7'd46: quarter_sine = 16'sd29621;
                7'd47: quarter_sine = 16'sd29956;
                7'd48: quarter_sine = 16'sd30273;
                7'd49: quarter_sine = 16'sd30571;
                7'd50: quarter_sine = 16'sd30852;
                7'd51: quarter_sine = 16'sd31113;
                7'd52: quarter_sine = 16'sd31356;
                7'd53: quarter_sine = 16'sd31580;
                7'd54: quarter_sine = 16'sd31785;
                7'd55: quarter_sine = 16'sd31971;
                7'd56: quarter_sine = 16'sd32137;
                7'd57: quarter_sine = 16'sd32285;
                7'd58: quarter_sine = 16'sd32412;
                7'd59: quarter_sine = 16'sd32521;
                7'd60: quarter_sine = 16'sd32609;
                7'd61: quarter_sine = 16'sd32678;
                7'd62: quarter_sine = 16'sd32728;
                7'd63: quarter_sine = 16'sd32757;
                7'd64: quarter_sine = 16'sd32767;
                default: quarter_sine = 16'sd0;
            endcase
        end
    endfunction

    // 通过象限对称得到完整正弦波
    function signed [15:0] sine_bin;
        input [7:0] index;
        reg [5:0] sub_index;
        begin
            sub_index = index[5:0];

            case (index[7:6])
                2'd0:
                    sine_bin =
                        quarter_sine(
                            {1'b0, sub_index}
                        );

                2'd1:
                    sine_bin =
                        quarter_sine(
                            7'd64 - {1'b0, sub_index}
                        );

                2'd2:
                    sine_bin =
                        -quarter_sine(
                            {1'b0, sub_index}
                        );

                default:
                    sine_bin =
                        -quarter_sine(
                            7'd64 - {1'b0, sub_index}
                        );
            endcase
        end
    endfunction

    // 每个采样周期推进一次正弦相位
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running <= 1'b0;
            phase <= 32'd0;
            pending_phase <= 32'd0;
            stage <= 3'd0;

            origin_position <= 32'sd0;
            position_ref <= 32'sd0;

            sin0 <= 16'sd0;
            sin1 <= 16'sd0;
            interp_sin <= 16'sd0;

            fraction <= 16'd0;
            interp_product <= 48'sd0;
            amplitude_product <= 48'sd0;
            relative_count <= 32'sd0;

        end else if (start) begin
            // 启动时以当前编码器位置为中心
            running <= 1'b1;
            phase <= 32'd0;
            pending_phase <= 32'd0;
            stage <= 3'd0;

            origin_position <= encoder_position;
            position_ref <= encoder_position;

        end else if (!enable) begin
            running <= 1'b0;
            stage <= 3'd0;

        end else if (running) begin
            case (stage)
                3'd0: begin
                    if (sample_tick) begin
                        pending_phase <=
                            phase + PHASE_STEP;
                        stage <= 3'd1;
                    end
                end

                3'd1: begin
                    fraction <= pending_phase[23:8];

                    sin0 <= sine_bin(
                        pending_phase[31:24]
                    );

                    sin1 <= sine_bin(
                        pending_phase[31:24] + 8'd1
                    );

                    stage <= 3'd2;
                end

                3'd2: begin
                    interp_product <=
                        ($signed(sin1) - $signed(sin0)) *
                        $signed({1'b0, fraction});

                    stage <= 3'd3;
                end

                3'd3: begin
                    interp_sin <=
                        $signed(sin0) +
                        (interp_product >>> 16);

                    stage <= 3'd4;
                end

                3'd4: begin
                    amplitude_product <=
                        $signed(interp_sin) *
                        AMPLITUDE_COUNTS;

                    stage <= 3'd5;
                end

                3'd5: begin
                    if (amplitude_product >= 0)
                        relative_count <=
                            (amplitude_product +
                             48'sd16384) >>> 15;
                    else
                        relative_count <=
                            -(((-amplitude_product) +
                                48'sd16384) >>> 15);

                    stage <= 3'd6;
                end

                3'd6: begin
                    phase <= pending_phase;

                    position_ref <=
                        origin_position + relative_count;

                    stage <= 3'd0;
                end

                default: begin
                    stage <= 3'd0;
                end
            endcase
        end
    end

endmodule
