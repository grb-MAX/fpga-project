// SSD1306 兼容 OLED 的 I2C 显示驱动，接口按开漏方式拉低或释放总线。
// 画面四行依次显示模式/启停、ADC 值、角度误差、编码器速度和控制命令。
module oled_ssd1306 #(
    parameter integer CLK_HZ = 50_000_000, // 输入主时钟频率，单位为 Hz，默认 50 MHz。
    parameter integer I2C_HZ = 400_000 // I2C 状态机的分频参考频率，具体高低电平时长由状态步骤决定。
) (
    input  wire               clk, // 系统主时钟，当前工程为 50 MHz。
    input  wire               rst_n, // 低有效异步复位，拉低后恢复模块初始状态。
    input  wire               refresh_tick, // 刷新请求脉冲；仅在空闲且初始化成功时接收。
    input  wire [2:0]         display_mode, // 顶层选择的显示模式编号，停止时可显示待启动模式。
    input  wire               run_active, // 顶层启停状态，用于显示运行或停止文字。
    input  wire [11:0]        adc_value, // 完整 12 位 ADC 采样计数，范围 0~4095。
    input  wire signed [15:0] angle_error, // 最近采样的角度误差：ADC 当前值减直立零点。
    input  wire signed [15:0] encoder_velocity, // 供显示的 16 位编码器速度，顶层取原速度低 16 位。
    input  wire signed [15:0] control_command, // 供显示的控制器命令，顶层连接 control_command。
    output reg                scl_drive_low, // 开漏时钟控制：为 1 拉低 SCL，为 0 释放为高阻。
    output reg                sda_drive_low, // 开漏数据控制：为 1 拉低 SDA，为 0 释放为高阻。
    input  wire               sda_i // 实际 SDA 总线回读值，用于读取从机应答。
);

    // I2C 状态机的基础节拍分频值；单个数据位会经过设置、高电平、下降三个状态。
    localparam integer I2C_TICK_DIV = CLK_HZ / (I2C_HZ * 2); // 状态机基础节拍的系统时钟数，默认整数分频为 62 拍。
    localparam [2:0] TX_INIT = 3'd0; // 初始化命令事务，包含地址、控制字节和初始化命令。
    localparam [2:0] TX_PAGE = 3'd1; // 页地址和列地址设置事务。
    localparam [2:0] TX_DATA = 3'd2; // 单页图像数据事务，包含 128 列的显示字节。

    localparam [4:0] ST_WAIT       = 5'd0; // 上电等待状态，计时约 20 ms 后开始初始化。
    localparam [4:0] ST_IDLE       = 5'd1; // 空闲状态，等待一次有效刷新请求。
    localparam [4:0] ST_START_A    = 5'd2; // 释放 SCL 并拉低 SDA，产生起始条件。
    localparam [4:0] ST_START_B    = 5'd3; // 拉低 SCL，准备开始发送最高位。
    localparam [4:0] ST_SEND_SETUP = 5'd4; // SCL 为低时设置当前待发送的数据位。
    localparam [4:0] ST_SEND_HIGH  = 5'd5; // 释放 SCL，让从机在高电平期间读取数据。
    localparam [4:0] ST_SEND_FALL  = 5'd6; // 拉低 SCL，推进下一数据位或应答阶段。
    localparam [4:0] ST_ACK_SETUP  = 5'd7; // 释放 SDA，让从机能够拉低应答。
    localparam [4:0] ST_ACK_HIGH   = 5'd8; // 释放 SCL 并读取 SDA，记录是否缺少应答。
    localparam [4:0] ST_ACK_FALL   = 5'd9; // 拉低 SCL，选择下一个字节或进入停止阶段。
    localparam [4:0] ST_STOP_A     = 5'd10; // 先将 SCL、SDA 都拉低，准备停止条件。
    localparam [4:0] ST_STOP_B     = 5'd11; // 释放 SCL，同时继续拉低 SDA。
    localparam [4:0] ST_STOP_C     = 5'd12; // 释放 SDA 形成停止条件，由下方 default 分支执行后续调度。

    reg [7:0] div_count; // 基础节拍分频计数器，持续按系统时钟累加。
    reg [20:0] startup_count; // 上电等待计数器，默认等待约 1,000,000 个系统时钟。
    reg [4:0] state; // I2C 传输状态机的当前状态。
    reg [2:0] tx_kind; // 当前事务类型：初始化、页地址或页数据。
    reg [7:0] tx_len; // 当前事务需要发送的总字节数，包含地址和控制字节。
    reg [7:0] tx_index; // 当前事务的字节序号，从 0 开始。
    reg [7:0] tx_byte; // 当前待发送字节，I2C 数据按最高位优先发送。
    reg [2:0] tx_page; // 当前显示页号，依次覆盖 0~7 页。
    reg [2:0] bit_index; // 当前字节的位序号，从 7 递减到 0。
    reg [1:0] tx_phase; // 预留的事务相位寄存器，当前流程只置零，未参与状态判断。
    reg       init_done; // 初始化结束时依据应答结果生成的完成标志。
    reg       ack_error; // 缺少应答的累计标志，一旦置位便保持到复位。

    reg [2:0] snap_mode; // 刷新开始时锁存的显示模式，整帧期间保持稳定。
    reg       snap_active; // 刷新开始时锁存的运行状态。
    reg [11:0] snap_adc; // 刷新开始时锁存的 ADC 计数。
    reg signed [15:0] snap_error; // 刷新开始时锁存的角度误差，以 16 位补码十六进制显示。
    reg signed [15:0] snap_velocity; // 刷新开始时锁存的速度，以 16 位补码十六进制显示。
    reg signed [15:0] snap_command; // 刷新开始时锁存的控制命令，以 16 位补码十六进制显示。

    wire i2c_tick = (div_count == I2C_TICK_DIV - 1); // 基础节拍使能脉冲，驱动传输状态步骤推进。

    // 将 4 位数值转换为 OLED 可显示的十六进制 ASCII 字符。
    function [7:0] hex_ascii;
        input [3:0] value; // 待转换为十六进制字符的 4 位数值。
        begin
            if (value < 10)
                hex_ascii = 8'h30 + value;
            else
                hex_ascii = 8'h41 + value - 10;
        end
    endfunction

    // 5x7 点阵字模查找表；每字节对应一列像素，每个字符额外留一列空白。
    // 下方十六进制常量为固定字形数据，不是控制算法参数。
    function [7:0] glyph_column;
        input [7:0] code; // 要查找点阵字模的 ASCII 字符编码。
        input [2:0] column; // 字符内的列序号，前五列为字形，后一列为空白。
        begin
            glyph_column = 8'h00;
            if (column < 5) begin
                case (code)
                    "0": case (column) 0:glyph_column=8'h3e; 1:glyph_column=8'h51; 2:glyph_column=8'h49; 3:glyph_column=8'h45; 4:glyph_column=8'h3e; endcase
                    "1": case (column) 0:glyph_column=8'h00; 1:glyph_column=8'h42; 2:glyph_column=8'h7f; 3:glyph_column=8'h40; 4:glyph_column=8'h00; endcase
                    "2": case (column) 0:glyph_column=8'h62; 1:glyph_column=8'h51; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h46; endcase
                    "3": case (column) 0:glyph_column=8'h22; 1:glyph_column=8'h49; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h36; endcase
                    "4": case (column) 0:glyph_column=8'h18; 1:glyph_column=8'h14; 2:glyph_column=8'h12; 3:glyph_column=8'h7f; 4:glyph_column=8'h10; endcase
                    "5": case (column) 0:glyph_column=8'h2f; 1:glyph_column=8'h49; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h31; endcase
                    "6": case (column) 0:glyph_column=8'h3e; 1:glyph_column=8'h49; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h32; endcase
                    "7": case (column) 0:glyph_column=8'h01; 1:glyph_column=8'h71; 2:glyph_column=8'h09; 3:glyph_column=8'h05; 4:glyph_column=8'h03; endcase
                    "8": case (column) 0:glyph_column=8'h36; 1:glyph_column=8'h49; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h36; endcase
                    "9": case (column) 0:glyph_column=8'h26; 1:glyph_column=8'h49; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h3e; endcase
                    "A": case (column) 0:glyph_column=8'h7e; 1:glyph_column=8'h11; 2:glyph_column=8'h11; 3:glyph_column=8'h11; 4:glyph_column=8'h7e; endcase
                    "C": case (column) 0:glyph_column=8'h3e; 1:glyph_column=8'h41; 2:glyph_column=8'h41; 3:glyph_column=8'h41; 4:glyph_column=8'h22; endcase
                    "D": case (column) 0:glyph_column=8'h7f; 1:glyph_column=8'h41; 2:glyph_column=8'h41; 3:glyph_column=8'h22; 4:glyph_column=8'h1c; endcase
                    "E": case (column) 0:glyph_column=8'h7f; 1:glyph_column=8'h49; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h41; endcase
                    "F": case (column) 0:glyph_column=8'h7f; 1:glyph_column=8'h09; 2:glyph_column=8'h09; 3:glyph_column=8'h09; 4:glyph_column=8'h01; endcase
                    "M": case (column) 0:glyph_column=8'h7f; 1:glyph_column=8'h02; 2:glyph_column=8'h0c; 3:glyph_column=8'h02; 4:glyph_column=8'h7f; endcase
                    "N": case (column) 0:glyph_column=8'h7f; 1:glyph_column=8'h06; 2:glyph_column=8'h18; 3:glyph_column=8'h60; 4:glyph_column=8'h7f; endcase
                    "O": case (column) 0:glyph_column=8'h3e; 1:glyph_column=8'h41; 2:glyph_column=8'h41; 3:glyph_column=8'h41; 4:glyph_column=8'h3e; endcase
                    "P": case (column) 0:glyph_column=8'h7f; 1:glyph_column=8'h09; 2:glyph_column=8'h09; 3:glyph_column=8'h09; 4:glyph_column=8'h06; endcase
                    "R": case (column) 0:glyph_column=8'h7f; 1:glyph_column=8'h09; 2:glyph_column=8'h19; 3:glyph_column=8'h29; 4:glyph_column=8'h46; endcase
                    "S": case (column) 0:glyph_column=8'h46; 1:glyph_column=8'h49; 2:glyph_column=8'h49; 3:glyph_column=8'h49; 4:glyph_column=8'h31; endcase
                    "T": case (column) 0:glyph_column=8'h01; 1:glyph_column=8'h01; 2:glyph_column=8'h7f; 3:glyph_column=8'h01; 4:glyph_column=8'h01; endcase
                    "U": case (column) 0:glyph_column=8'h3f; 1:glyph_column=8'h40; 2:glyph_column=8'h40; 3:glyph_column=8'h40; 4:glyph_column=8'h3f; endcase
                    "V": case (column) 0:glyph_column=8'h1f; 1:glyph_column=8'h20; 2:glyph_column=8'h40; 3:glyph_column=8'h20; 4:glyph_column=8'h1f; endcase
                    "x": case (column) 0:glyph_column=8'h00; 1:glyph_column=8'h36; 2:glyph_column=8'h08; 3:glyph_column=8'h36; 4:glyph_column=8'h00; endcase
                    ":": case (column) 0:glyph_column=8'h00; 1:glyph_column=8'h36; 2:glyph_column=8'h36; 3:glyph_column=8'h00; 4:glyph_column=8'h00; endcase
                    default: glyph_column = 8'h00;
                endcase
            end
        end
    endfunction

    // 根据行号和字符位置生成显示文本中的单个字符，数值统一显示为十六进制。
    // 第 0 行为模式与启停，第 1 行为 ADC，第 2 行为误差，第 3 行为速度与命令。
    // 有符号量直接显示其 16 位补码位型，负数不单独输出负号。
    function [7:0] line_char;
        input [1:0] line; // 显示文本行号，0~3 分别为模式、ADC、误差、速度和命令行。
        input [4:0] char_index; // 一行内的字符序号，从 0 开始。
        begin
            line_char = 8'h20;
            case (line)
                2'd0: begin
                    case (char_index)
                        0:line_char="M"; 1:line_char="O"; 2:line_char="D"; 3:line_char="E"; 4:line_char=":";
                        5:line_char=(snap_mode == 0) ? "S" : "R";
                        6:line_char=(snap_mode == 0) ? "A" : "U";
                        7:line_char=(snap_mode == 0) ? "F" : "N";
                        8:line_char=(snap_mode == 0) ? "E" : hex_ascii(snap_mode);
                        9:line_char=" ";
                        10:line_char="S";
                        11:line_char="T";
                        12:line_char=snap_active ? "A" : "O";
                        13:line_char=snap_active ? "R" : "P";
                        14:line_char=snap_active ? "T" : " ";
                        default: line_char=" ";
                    endcase
                end
                2'd1: begin
                    case (char_index)
                        0:line_char="A"; 1:line_char="D"; 2:line_char="C"; 3:line_char=":"; 4:line_char="0"; 5:line_char="x";
                        6:line_char=hex_ascii(snap_adc[11:8]); 7:line_char=hex_ascii(snap_adc[7:4]); 8:line_char=hex_ascii(snap_adc[3:0]);
                        default: line_char=" ";
                    endcase
                end
                2'd2: begin
                    case (char_index)
                        0:line_char="E"; 1:line_char="R"; 2:line_char="R"; 3:line_char=":"; 4:line_char="0"; 5:line_char="x";
                        6:line_char=hex_ascii(snap_error[15:12]); 7:line_char=hex_ascii(snap_error[11:8]);
                        8:line_char=hex_ascii(snap_error[7:4]); 9:line_char=hex_ascii(snap_error[3:0]);
                        default: line_char=" ";
                    endcase
                end
                default: begin
                    case (char_index)
                        0:line_char="V"; 1:line_char=":"; 2:line_char="0"; 3:line_char="x";
                        4:line_char=hex_ascii(snap_velocity[15:12]); 5:line_char=hex_ascii(snap_velocity[11:8]);
                        6:line_char=hex_ascii(snap_velocity[7:4]); 7:line_char=hex_ascii(snap_velocity[3:0]);
                        9:line_char="C"; 10:line_char=":"; 11:line_char="0"; 12:line_char="x";
                        13:line_char=hex_ascii(snap_command[15:12]); 14:line_char=hex_ascii(snap_command[11:8]);
                        15:line_char=hex_ascii(snap_command[7:4]); 16:line_char=hex_ascii(snap_command[3:0]);
                        default: line_char=" ";
                    endcase
                end
            endcase
        end
    endfunction

    // 将文本字符映射成一页内的列字节：每 6 列为一个字符，前五列字模、最后一列留白。
    // 偶数页承载文字，奇数页输出空白，因此 8 个显示页组成 4 行文字。
    function [7:0] data_byte;
        input [2:0] page; // OLED 页号，0~7；偶数页绘制字形，奇数页留白。
        input [6:0] column_index; // OLED 当前页中的像素列号，范围为 0~127。
        reg [4:0] char_index; // 一行内的字符序号，从 0 开始。
        reg [2:0] glyph_index; // 当前字符内的点阵列号，取列号除以 6 的余数。
        reg [7:0] character; // 本列对应的 ASCII 字符，由行内容函数生成。
        reg [7:0] char_calc; // 列号除以 6 得到的字符位置中间结果。
        begin
            char_calc = column_index / 8'd6;
            char_index = char_calc[4:0];
            glyph_index = column_index % 6;
            if ((page[0] == 1'b0) && (char_index < 22)) begin
                character = line_char(page[2:1], char_index);
                data_byte = glyph_column(character, glyph_index);
            end else begin
                data_byte = 8'h00;
            end
        end
    endfunction

    // 按事务类型选择初始化命令、页/列地址或显示数据。
    // 0x78 是 7 位地址 0x3C 加写位后的地址字节；0x00 为命令控制字，0x40 为数据控制字。
    function [7:0] stream_byte;
        input [2:0] kind; // 本次要生成的事务类型，对应初始化、页地址或页数据。
        input [7:0] index; // 当前事务内的字节序号，包含地址和控制字节。
        input [2:0] page; // OLED 页号，0~7；偶数页绘制字形，奇数页留白。
        begin
            case (kind)
                TX_INIT: begin
                    case (index)
                        0:stream_byte=8'h78; 1:stream_byte=8'h00;
                        2:stream_byte=8'hAE; 3:stream_byte=8'hD5; 4:stream_byte=8'h80;
                        5:stream_byte=8'hA8; 6:stream_byte=8'h3F; 7:stream_byte=8'hD3;
                        8:stream_byte=8'h00; 9:stream_byte=8'h40; 10:stream_byte=8'h8D;
                        11:stream_byte=8'h14; 12:stream_byte=8'h20; 13:stream_byte=8'h00;
                        14:stream_byte=8'hA1; 15:stream_byte=8'hC8; 16:stream_byte=8'hDA;
                        17:stream_byte=8'h12; 18:stream_byte=8'h81; 19:stream_byte=8'h8F;
                        20:stream_byte=8'hD9; 21:stream_byte=8'hF1; 22:stream_byte=8'hDB;
                        23:stream_byte=8'h40; 24:stream_byte=8'hA4; 25:stream_byte=8'hA6;
                        default:stream_byte=8'hAF;
                    endcase
                end
                TX_PAGE: begin
                    case (index)
                        0:stream_byte=8'h78; 1:stream_byte=8'h00;
                        2:stream_byte=8'hB0 | page; 3:stream_byte=8'h00;
                        default:stream_byte=8'h10;
                    endcase
                end
                default: begin
                    if (index == 0)
                        stream_byte = 8'h78;
                    else if (index == 1)
                        stream_byte = 8'h40;
                    else
                        stream_byte = data_byte(page, index - 2);
                end
            endcase
        end
    endfunction

    // 启动一笔 I2C 事务，并初始化字节、位和页索引。
    task start_transaction;
        input [2:0] kind; // 本次要生成的事务类型，对应初始化、页地址或页数据。
        input [7:0] length; // 本次事务的总字节数，启动时写入 tx_len。
        input [2:0] page; // OLED 页号，0~7；偶数页绘制字形，奇数页留白。
        begin
            tx_kind <= kind;
            tx_len <= length;
            tx_page <= page;
            tx_index <= 8'd0;
            tx_byte <= stream_byte(kind, 8'd0, page);
            bit_index <= 3'd7;
            tx_phase <= 2'd0;
            state <= ST_START_A;
        end
    endtask

    // I2C 状态机：上电初始化后等待刷新请求，再逐页发送字符点阵数据。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_count       <= 8'd0;
            startup_count   <= 21'd0;
            state           <= ST_WAIT;
            tx_kind         <= TX_INIT;
            tx_len          <= 8'd0;
            tx_index        <= 8'd0;
            tx_byte         <= 8'd0;
            tx_page         <= 3'd0;
            bit_index       <= 3'd7;
            tx_phase        <= 2'd0;
            init_done       <= 1'b0;
            ack_error       <= 1'b0;
            scl_drive_low   <= 1'b0;
            sda_drive_low   <= 1'b0;
            snap_mode       <= 1'b0;
            snap_active     <= 1'b0;
            snap_adc        <= 12'd0;
            snap_error      <= 16'sd0;
            snap_velocity   <= 16'sd0;
            snap_command    <= 16'sd0;
        end else begin
            if (div_count == I2C_TICK_DIV - 1)
                div_count <= 8'd0;
            else
                div_count <= div_count + 1'b1;
            // 默认 50 MHz 下等待约 20 ms，让屏幕上电后再发送 27 字节初始化事务。
            if (state == ST_WAIT) begin
                if (startup_count == 21'd1_000_000) begin
                    startup_count <= startup_count;
                    start_transaction(TX_INIT, 8'd27, 3'd0);
                end else begin
                    startup_count <= startup_count + 1'b1;
                end
            end else if (state == ST_IDLE) begin
                // 只在空闲且初始化成功时接受刷新，忙期间的刷新脉冲不缓存。
                // 一次性保存画面快照，随后发送 8 页时保持内容一致。
                if (refresh_tick && init_done) begin
                    snap_mode     <= display_mode;
                    snap_active   <= run_active;
                    snap_adc      <= adc_value;
                    snap_error    <= angle_error;
                    snap_velocity <= encoder_velocity;
                    snap_command  <= control_command;
                    start_transaction(TX_PAGE, 8'd5, 3'd0);
                end
            end else if (i2c_tick) begin
                case (state)
                    ST_START_A: begin
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;
                        state <= ST_START_B;
                    end
                    ST_START_B: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b1;
                        state <= ST_SEND_SETUP;
                        bit_index <= 3'd7;
                    end
                    ST_SEND_SETUP: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= ~tx_byte[bit_index];
                        state <= ST_SEND_HIGH;
                    end
                    ST_SEND_HIGH: begin
                        scl_drive_low <= 1'b0;
                        state <= ST_SEND_FALL;
                    end
                    ST_SEND_FALL: begin
                        scl_drive_low <= 1'b1;
                        if (bit_index == 0) begin
                            state <= ST_ACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 1'b1;
                            state <= ST_SEND_SETUP;
                        end
                    end
                    ST_ACK_SETUP: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b0;
                        state <= ST_ACK_HIGH;
                    end
                    ST_ACK_HIGH: begin
                        scl_drive_low <= 1'b0;
                        // SDA 未被从机拉低时记录缺少应答，标志保持到复位。
                        // 此处不立即中断传输；初始化结束时才用该标志决定 init_done。
                        if (sda_i)
                            ack_error <= 1'b1;
                        state <= ST_ACK_FALL;
                    end
                    ST_ACK_FALL: begin
                        scl_drive_low <= 1'b1;
                        if (tx_index + 1'b1 >= tx_len) begin
                            state <= ST_STOP_A;
                        end else begin
                            tx_index <= tx_index + 1'b1;
                            tx_byte <= stream_byte(tx_kind, tx_index + 1'b1, tx_page);
                            bit_index <= 3'd7;
                            state <= ST_SEND_SETUP;
                        end
                    end
                    ST_STOP_A: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b1;
                        state <= ST_STOP_B;
                    end
                    ST_STOP_B: begin
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;
                        state <= ST_STOP_C;
                    end
                    // ST_STOP_C 在此释放 SDA，完成停止条件，并调度下一个事务。
                    default: begin
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;
                        if (tx_kind == TX_INIT) begin
                            init_done <= !ack_error;
                            state <= ST_IDLE;
                        // 每页先发送 5 字节页/列设置，再发送地址、控制字及 128 列数据。
                        end else if (tx_kind == TX_PAGE) begin
                            start_transaction(TX_DATA, 8'd130, tx_page);
                        end else if (tx_page == 3'd7) begin
                            state <= ST_IDLE;
                        end else begin
                            tx_page <= tx_page + 1'b1;
                            start_transaction(TX_PAGE, 8'd5, tx_page + 1'b1);
                        end
                    end
                endcase
            end
        end
    end

endmodule
