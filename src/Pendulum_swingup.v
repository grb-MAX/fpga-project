`timescale 1ns / 1ps
// RUN3独立起摆：以直立为角度零点，按角度象限与运动方向注能；直立附近制动。
// 原error^2+(rate*4)^2与40000000并不是已标定的机械能，本版不再把它称为物理能量控制。
// 2224半周计数来自本次日志中直立3153与静止下垂约929之差，属于待实机复核的标定估计。
// 未将ADC 4095/0直接当作相邻角度：使用估计整周4448计数归一化，并拒绝过大的差分。
// 起摆只输出有界相位命令，捕获/释放仍由原控制器决定；平衡PID、位置目标及RUN4不改。
// 行程/超时故障只锁止RUN3本次起摆，暂停后清除；不是认证安全功能。
module pendulum_swingup #(
    parameter integer ANGLE_CAPTURE=80, ANGLE_RELEASE=240, CAPTURE_RATE=6,
    parameter integer SWING_GAIN=700, COS_ZERO_CROSS=1112, HALF_TURN_COUNTS=2224,
    parameter integer BRAKE_ANGLE=240, RATE_NOISE_BAND=1, RATE_VALID_LIMIT=128,
    parameter integer KICK_SAMPLES=20, QUIET_PERIOD_SAMPLES=200,
    parameter integer ARM_TRAVEL_LIMIT=600, SWING_TIMEOUT_SAMPLES=12000,
    parameter integer OUTPUT_LIMIT=1400
) (
    input wire clk,rst_n,sample_tick,enable,
    input wire [11:0] adc_value,angle_zero,
    input wire signed [31:0] encoder_position,
    input wire [1:0] control_mode,
    output reg active,
    output wire signed [15:0] command,
    output wire signed [31:0] raw_command,
    output reg fault_latched,result_valid
);
    localparam signed [15:0] GAIN=(SWING_GAIN>OUTPUT_LIMIT)?OUTPUT_LIMIT:SWING_GAIN;
    localparam signed [13:0] HALF_SPAN=HALF_TURN_COUNTS, FULL_SPAN=2*HALF_TURN_COUNTS;
    reg [11:0] adc_last;
    reg sample_seen,enable_seen,decide_pending;
    reg signed [31:0] arm_origin;
    reg [31:0] swing_count,quiet_count;
    reg kick_negative;
    reg sampled_capture,sampled_invalid,sampled_quiet,sampled_brake,sampled_negative,sampled_rate_negative;
    reg signed [15:0] command_reg;
    wire signed [12:0] linear_error=$signed({1'b0,adc_value})-$signed({1'b0,angle_zero});
    wire signed [13:0] linear_rate=$signed({1'b0,adc_value})-$signed({1'b0,adc_last});
    wire signed [13:0] error_extended={linear_error[12],linear_error};
    wire signed [13:0] normalized_error=(error_extended < -HALF_SPAN)?
        error_extended+FULL_SPAN:((error_extended>HALF_SPAN)?error_extended-FULL_SPAN:error_extended);
    wire signed [13:0] normalized_rate=(linear_rate < -HALF_SPAN)?
        linear_rate+FULL_SPAN:((linear_rate>HALF_SPAN)?linear_rate-FULL_SPAN:linear_rate);
    wire [13:0] abs_phase=(normalized_error<0)?-normalized_error:normalized_error;
    wire [12:0] abs_linear=(linear_error<0)?-linear_error:linear_error;
    wire [13:0] abs_rate=(normalized_rate<0)?-normalized_rate:normalized_rate;
    wire signed [32:0] arm_delta=$signed({encoder_position[31],encoder_position})-$signed({arm_origin[31],arm_origin});
    wire outside_balance=(control_mode!=2'd2)||(abs_linear>ANGLE_RELEASE);
    assign command=(enable && active && !fault_latched && !decide_pending)?command_reg:16'sd0;
    assign raw_command={{16{command[15]}},command};

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            adc_last<=0;sample_seen<=0;enable_seen<=0;decide_pending<=0;
            arm_origin<=0;swing_count<=0;quiet_count<=0;kick_negative<=0;
            sampled_capture<=0;sampled_invalid<=0;sampled_quiet<=0;sampled_brake<=0;
            sampled_negative<=0;sampled_rate_negative<=0;
            command_reg<=0;active<=0;fault_latched<=0;result_valid<=0;
        end else begin
            result_valid<=0;
            if(sample_tick) begin adc_last<=adc_value;sample_seen<=1;end
            if(!enable) begin
                enable_seen<=0;decide_pending<=0;command_reg<=0;active<=0;
                fault_latched<=0;swing_count<=0;quiet_count<=0;kick_negative<=0;
            end else if(fault_latched) begin
                command_reg<=0;decide_pending<=0;active<=1;
            end else if(decide_pending) begin
                // 输入已锁存；决策在采样后一拍提交，避免长乘法及组合链。
                decide_pending<=0;result_valid<=1;
                if(sampled_invalid || sampled_capture)command_reg<=0;
                else if(sampled_quiet) begin
                    if(quiet_count<KICK_SAMPLES)command_reg<=kick_negative?-GAIN:GAIN;
                    else command_reg<=0;
                    if(quiet_count==QUIET_PERIOD_SAMPLES-1) begin
                        quiet_count<=0;kick_negative<=!kick_negative;
                    end else quiet_count<=quiet_count+1'b1;
                end else begin
                    quiet_count<=0;
                    // 近直立且摆速过快时，按运动方向施加制动，不继续推着翻过顶部。
                    if(sampled_brake)command_reg<=sampled_rate_negative?-GAIN:GAIN;
                    else command_reg<=sampled_negative?-GAIN:GAIN;
                end
            end else if(sample_tick) begin
                enable_seen<=1;
                if(!enable_seen)arm_origin<=encoder_position;
                active<=outside_balance;command_reg<=0;
                if(!outside_balance) begin
                    arm_origin<=encoder_position;swing_count<=0;quiet_count<=0;
                end else if(enable_seen &&
                    ((arm_delta>=ARM_TRAVEL_LIMIT)||(arm_delta<=-ARM_TRAVEL_LIMIT)||
                     (swing_count>=SWING_TIMEOUT_SAMPLES-1))) begin
                    fault_latched<=1;command_reg<=0;
                end else begin
                    swing_count<=swing_count+1'b1;
                    sampled_invalid<=!sample_seen||(abs_rate>RATE_VALID_LIMIT);
                    sampled_capture<=(abs_linear<=ANGLE_CAPTURE)&&(abs_rate<=CAPTURE_RATE);
                    sampled_quiet<=(abs_rate<=RATE_NOISE_BAND);
                    sampled_brake<=(abs_linear<=BRAKE_ANGLE)&&(abs_rate>CAPTURE_RATE);
                    sampled_rate_negative<=(normalized_rate<0);
                    // 与正角度误差对应正平衡命令的符号约定一致：注能取-rate*cos(angle)的符号。
                    sampled_negative<=(normalized_rate<0)^(abs_phase<COS_ZERO_CROSS);
                    decide_pending<=1;
                end
            end
        end
    end
endmodule

// RUN3输入保持桥。数据只在接收新ADC采样时改变；计算触发至少晚8个完整周期。
// 忙期间不覆盖输入，确保短间隔/异常重复采样也不会破坏保持关系。
module run3_sample_bridge (
    input wire clk, rst_n, sample_tick, enable,
    input wire [11:0] adc_value, angle_zero,
    input wire signed [31:0] encoder_position, position_target, encoder_velocity,
    output reg control_tick,
    output reg held_enable /* synthesis syn_preserve=1 */,
    output reg [11:0] held_adc /* synthesis syn_preserve=1 */,
    output reg [11:0] held_zero /* synthesis syn_preserve=1 */,
    output reg signed [31:0] held_position /* synthesis syn_preserve=1 */,
    output reg signed [31:0] held_target /* synthesis syn_preserve=1 */,
    output reg signed [31:0] held_velocity /* synthesis syn_preserve=1 */
);
    reg [3:0] wait_count;
    reg busy;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            control_tick<=0;held_enable<=0;held_adc<=0;held_zero<=0;
            held_position<=0;held_target<=0;held_velocity<=0;wait_count<=0;busy<=0;
        end else begin
            control_tick<=0;
            if(busy) begin
                if(wait_count==4'd7) begin control_tick<=1;busy<=0;end
                else wait_count<=wait_count+1'b1;
            end else if(sample_tick && !control_tick) begin
                held_enable<=enable;held_adc<=adc_value;held_zero<=angle_zero;
                held_position<=encoder_position;held_target<=position_target;
                held_velocity<=encoder_velocity;wait_count<=0;busy<=1;
            end
        end
    end
endmodule
