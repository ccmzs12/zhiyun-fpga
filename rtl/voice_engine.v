`timescale 1ns/1ps

//=============================================================================
// voice_engine —— 8 声部时分复用合成引擎（附录 E）
//
// 时序：frame_start（每采样帧一拍，48.8 kHz）
//   → 依次处理声部 0..7，每声部 SLOT_CYCLES = 128 个时钟
//   → 8 x 128 = 1024 时钟，正好等于 50MHz / 48.828kHz 的帧长
//
// 每声部槽内子相位（slot）：
//   0: 发起 KS 延迟线读 x（地址 {声部号, 指针}）
//   1: 取回 x，发起读 y
//   2: 取回 y
//   3: FM 第一级：调制波查表 + 乘调制指数
//   4: FM 第二级：载波查表 + 乘包络；KS 环路计算
//   5: 写回 KS 延迟线、更新全部状态、送出本声部样点
//   （已改为 9 拍流水：见 128 点正弦表处的说明）6..127 原为空闲
//
// KS 延迟线：单块 RAM，地址 = {声部号[2:0], 指针[8:0]}，共 4096 x 16。
//   读在 slot0/1，写在 slot5，读写分时；不同声部地址高 3 位不同，互不干扰。
//=============================================================================
module voice_engine #(
    parameter integer NVOICES     = 8,
    parameter integer SLOT_CYCLES = 128,
    parameter [7:0]   K_FILT      = 8'd160,     // 环路滤波系数（0..256，越大衰减越快）
    parameter [15:0]  ATT_STEP    = 16'd134,    // 包络：起音
    parameter [15:0]  DEC_STEP    = 16'd13,     // 包络：衰减
    parameter [15:0]  REL_STEP    = 16'd4,      // 包络：释音
    parameter [15:0]  SUS_LEVEL   = 16'd19660   // 包络：延音电平
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,      // 每采样帧一拍脉冲

    // ---- 声部分配接口 ----
    input  wire        note_on,          // 一拍脉冲
    input  wire        note_off,         // 一拍脉冲：释放 note_idx 对应的声部
    input  wire        all_off,          // 一拍脉冲：全部进入释音
    input  wire [5:0]  note_idx,         // {八度[1:0], 键号[2:0]}
    input  wire        timbre,           // 0 = FM(电钢琴)  1 = KS(拨弦)
    input  wire [15:0] velocity,         // 力度

    // ---- 分配结果（调试用）----
    input  wire signed [15:0] bend,
    input  wire [7:0]  vib_depth,

    output reg  [2:0]  alloc_voice,
    output reg  [7:0]  active_mask,

    // ---- 串行送出 8 路声部样点（供 mixer 累加）----
    output reg         v_valid,
    output reg  [2:0]  v_index,
    output reg  signed [15:0] v_sample
);

    localparam integer VW = 3;
    reg [3:0] mds [0:NVOICES-1];

    //-------------------------------------------------------------------------
    // 声部状态（附录 E 列出的全部字段）
    //-------------------------------------------------------------------------
    reg        act  [0:NVOICES-1];   // 活动标志
    reg        tim  [0:NVOICES-1];   // 音色
    reg [5:0]  nte  [0:NVOICES-1];   // 音符
    reg [31:0] phc  [0:NVOICES-1];   // 载波相位
    reg [31:0] phm  [0:NVOICES-1];   // 调制相位
    reg [31:0] incr [0:NVOICES-1];   // 相位增量
    reg [2:0]  est  [0:NVOICES-1];   // 包络状态 0=off 1=att 2=dec 3=sus 4=rel
    reg [15:0] env  [0:NVOICES-1];   // 包络值
    reg [15:0] midx [0:NVOICES-1];   // FM 调制指数
    reg [8:0]  ksp  [0:NVOICES-1];   // KS 指针
    reg [15:0] ksl  [0:NVOICES-1];   // KS 长度
    reg [16:0] excn [0:NVOICES-1];   // 激励计数
    reg [15:0] vel  [0:NVOICES-1];   // 力度
    reg [15:0] lfs  [0:NVOICES-1];   // 激励噪声源（LFSR）
    // 【必须是有符号！】原先写成 reg [15:0]（无符号），它在
    //   (knorm - nlp) >>> 3 里与有符号数混算时，整个表达式被判为无符号：
    //   负数被当成巨大正数、算术右移变逻辑右移 → 激励低通彻底失真成宽带噪声，
    //   听感就是 KS 声部一直"沙沙"、音高完全出不来（实测 nlp 达到 36981 > 32767）。
    reg signed [15:0] nlp  [0:NVOICES-1];   // 激励低通状态（**有符号**）
    reg [2:0]  steal_ptr;            // round-robin voice-steal pointer

    // KS 延迟线：单块 RAM
    (* ram_style = "block" *)
    reg [15:0] ksmem [0:(1<<(VW+9))-1];

    //-------------------------------------------------------------------------
    // 扫描指针
    //-------------------------------------------------------------------------
    reg [VW:0]   cur_v;
    reg [7:0]    slot;
    reg [7:0]    frm_cnt;
    reg [23:0]   lfo_phase;

    wire [2:0]  v3  = cur_v[2:0];
    wire [8:0]  ptr = ksp[v3];
    wire [8:0]  ptr_n = (ptr + 9'd1 >= ksl[v3]) ? 9'd0 : ptr + 9'd1;

    wire [VW+8:0] ks_addr_x = {v3, ptr};
    wire [VW+8:0] ks_addr_y = {v3, ptr_n};

    reg  [VW+8:0] ks_rd_addr;
    reg  [15:0]   ks_rd_data;
    reg           ks_we;
    reg  [VW+8:0] ks_wr_addr;
    reg  [15:0]   ks_wr_data;

    always @(posedge clk) begin
        if (ks_we)
            ksmem[ks_wr_addr] <= ks_wr_data;
        // 【读取保护 + 起音空线】
        //   1) 激励阶段（excn < ksl）内把延迟线当作**空的**：
        //      这是经典 Karplus-Strong 的做法 —— 第一个循环由激励写满一圈再开始
        //      循环。否则会读到未初始化内容（仿真里是 X，硬件上是上一个音的残留），
        //      表现为"KS 声部完全没有输出"。
        //   2) 指针超出该声部有效长度（ptr >= ksl）一律读 0，避免读到旧音的尾巴。
        ks_rd_data <= (excn[ks_rd_addr[11:9]] <  ksl[ks_rd_addr[11:9]]) ? 16'd0 :
                      (ks_rd_addr[8:0]        <  ksl[ks_rd_addr[11:9]]) ? ksmem[ks_rd_addr] :
                                                                         16'd0;
    end

    //-------------------------------------------------------------------------
    // 工作寄存器
    //-------------------------------------------------------------------------
    reg signed [15:0] x_w, y_w;
    reg signed [15:0] fm_out;
    reg signed [31:0] mod_scaled;
    reg signed [15:0] ks_new;

    wire signed [15:0] rd_val = $signed(ks_rd_data);

    //-------------------------------------------------------------------------
    // 组合查表（**不要用寄存器**，否则会差一拍，读到上一个声部的值）
    //-------------------------------------------------------------------------
    wire [9:0] madr = phm[v3][31:22];

    // 载波相位：本拍组合计算，不能用寄存器再读回
    wire [10:0] car_sum_w = {1'b0, phc[v3][31:22]} + mod_scaled[25:15];
    wire [9:0]  cadr      = car_sum_w[9:0];

    //-------------------------------------------------------------------------
    // 【基础音色：保留 32 点正弦量化，使用同步查表 + 流水线】
    //   基准 F89A 包使用 qx[7:3] 选择 32 点表。这里仍从 128 项同步 ROM
    //   中每隔 4 项取值，所选 32 个值与基准 sin_tab 逐项一致。
    //   查表结果先打一拍寄存器，再进乘法器。这样不管综合器把表做成 LUT
    //   还是做成块 ROM（同步读），行为都和仿真一致；也把查表和乘法拆到不同时钟。
    //   折叠/取负这些算术全部放在函数外的 wire 上。
    //   时序：slot3 查调制表 → slot4 乘调制指数 → slot5 载波地址打拍
    //         → slot6 查载波表 → slot7 乘包络 → slot8 写回/送样点
    //-------------------------------------------------------------------------
    reg  [9:0]  cadr_r;
    wire [7:0]  m_qx = madr[8]   ? (8'd255 - madr[7:0])   : madr[7:0];
    wire [7:0]  c_qx = cadr_r[8] ? (8'd255 - cadr_r[7:0]) : cadr_r[7:0];
    wire [6:0]  m_tab_sel = {m_qx[7:3], 2'b00};
    wire [6:0]  c_tab_sel = {c_qx[7:3], 2'b00};
    reg  [15:0] m_tab, c_tab;
    reg         m_neg, c_neg;
    always @(posedge clk) begin
        m_neg <= madr[9];
        c_neg <= cadr_r[9];
    end
    always @(posedge clk) begin
        case (m_tab_sel)
            7'd0: m_tab <= 16'd0;
            7'd1: m_tab <= 16'd402;
            7'd2: m_tab <= 16'd804;
            7'd3: m_tab <= 16'd1206;
            7'd4: m_tab <= 16'd1608;
            7'd5: m_tab <= 16'd2009;
            7'd6: m_tab <= 16'd2410;
            7'd7: m_tab <= 16'd2811;
            7'd8: m_tab <= 16'd3212;
            7'd9: m_tab <= 16'd3612;
            7'd10: m_tab <= 16'd4011;
            7'd11: m_tab <= 16'd4410;
            7'd12: m_tab <= 16'd4808;
            7'd13: m_tab <= 16'd5205;
            7'd14: m_tab <= 16'd5602;
            7'd15: m_tab <= 16'd5998;
            7'd16: m_tab <= 16'd6393;
            7'd17: m_tab <= 16'd6786;
            7'd18: m_tab <= 16'd7179;
            7'd19: m_tab <= 16'd7571;
            7'd20: m_tab <= 16'd7962;
            7'd21: m_tab <= 16'd8351;
            7'd22: m_tab <= 16'd8739;
            7'd23: m_tab <= 16'd9126;
            7'd24: m_tab <= 16'd9512;
            7'd25: m_tab <= 16'd9896;
            7'd26: m_tab <= 16'd10278;
            7'd27: m_tab <= 16'd10659;
            7'd28: m_tab <= 16'd11039;
            7'd29: m_tab <= 16'd11417;
            7'd30: m_tab <= 16'd11793;
            7'd31: m_tab <= 16'd12167;
            7'd32: m_tab <= 16'd12539;
            7'd33: m_tab <= 16'd12910;
            7'd34: m_tab <= 16'd13279;
            7'd35: m_tab <= 16'd13645;
            7'd36: m_tab <= 16'd14010;
            7'd37: m_tab <= 16'd14372;
            7'd38: m_tab <= 16'd14732;
            7'd39: m_tab <= 16'd15090;
            7'd40: m_tab <= 16'd15446;
            7'd41: m_tab <= 16'd15800;
            7'd42: m_tab <= 16'd16151;
            7'd43: m_tab <= 16'd16499;
            7'd44: m_tab <= 16'd16846;
            7'd45: m_tab <= 16'd17189;
            7'd46: m_tab <= 16'd17530;
            7'd47: m_tab <= 16'd17869;
            7'd48: m_tab <= 16'd18204;
            7'd49: m_tab <= 16'd18537;
            7'd50: m_tab <= 16'd18868;
            7'd51: m_tab <= 16'd19195;
            7'd52: m_tab <= 16'd19519;
            7'd53: m_tab <= 16'd19841;
            7'd54: m_tab <= 16'd20159;
            7'd55: m_tab <= 16'd20475;
            7'd56: m_tab <= 16'd20787;
            7'd57: m_tab <= 16'd21096;
            7'd58: m_tab <= 16'd21403;
            7'd59: m_tab <= 16'd21705;
            7'd60: m_tab <= 16'd22005;
            7'd61: m_tab <= 16'd22301;
            7'd62: m_tab <= 16'd22594;
            7'd63: m_tab <= 16'd22884;
            7'd64: m_tab <= 16'd23170;
            7'd65: m_tab <= 16'd23452;
            7'd66: m_tab <= 16'd23731;
            7'd67: m_tab <= 16'd24007;
            7'd68: m_tab <= 16'd24279;
            7'd69: m_tab <= 16'd24547;
            7'd70: m_tab <= 16'd24811;
            7'd71: m_tab <= 16'd25072;
            7'd72: m_tab <= 16'd25329;
            7'd73: m_tab <= 16'd25582;
            7'd74: m_tab <= 16'd25832;
            7'd75: m_tab <= 16'd26077;
            7'd76: m_tab <= 16'd26319;
            7'd77: m_tab <= 16'd26556;
            7'd78: m_tab <= 16'd26790;
            7'd79: m_tab <= 16'd27019;
            7'd80: m_tab <= 16'd27245;
            7'd81: m_tab <= 16'd27466;
            7'd82: m_tab <= 16'd27683;
            7'd83: m_tab <= 16'd27896;
            7'd84: m_tab <= 16'd28105;
            7'd85: m_tab <= 16'd28310;
            7'd86: m_tab <= 16'd28510;
            7'd87: m_tab <= 16'd28706;
            7'd88: m_tab <= 16'd28898;
            7'd89: m_tab <= 16'd29085;
            7'd90: m_tab <= 16'd29268;
            7'd91: m_tab <= 16'd29447;
            7'd92: m_tab <= 16'd29621;
            7'd93: m_tab <= 16'd29791;
            7'd94: m_tab <= 16'd29956;
            7'd95: m_tab <= 16'd30117;
            7'd96: m_tab <= 16'd30273;
            7'd97: m_tab <= 16'd30424;
            7'd98: m_tab <= 16'd30571;
            7'd99: m_tab <= 16'd30714;
            7'd100: m_tab <= 16'd30852;
            7'd101: m_tab <= 16'd30985;
            7'd102: m_tab <= 16'd31113;
            7'd103: m_tab <= 16'd31237;
            7'd104: m_tab <= 16'd31356;
            7'd105: m_tab <= 16'd31470;
            7'd106: m_tab <= 16'd31580;
            7'd107: m_tab <= 16'd31685;
            7'd108: m_tab <= 16'd31785;
            7'd109: m_tab <= 16'd31880;
            7'd110: m_tab <= 16'd31971;
            7'd111: m_tab <= 16'd32057;
            7'd112: m_tab <= 16'd32137;
            7'd113: m_tab <= 16'd32213;
            7'd114: m_tab <= 16'd32285;
            7'd115: m_tab <= 16'd32351;
            7'd116: m_tab <= 16'd32412;
            7'd117: m_tab <= 16'd32469;
            7'd118: m_tab <= 16'd32521;
            7'd119: m_tab <= 16'd32567;
            7'd120: m_tab <= 16'd32609;
            7'd121: m_tab <= 16'd32646;
            7'd122: m_tab <= 16'd32678;
            7'd123: m_tab <= 16'd32705;
            7'd124: m_tab <= 16'd32728;
            7'd125: m_tab <= 16'd32745;
            7'd126: m_tab <= 16'd32757;
            default: m_tab <= 16'd32765;
        endcase
    end
    always @(posedge clk) begin
        case (c_tab_sel)
            7'd0: c_tab <= 16'd0;
            7'd1: c_tab <= 16'd402;
            7'd2: c_tab <= 16'd804;
            7'd3: c_tab <= 16'd1206;
            7'd4: c_tab <= 16'd1608;
            7'd5: c_tab <= 16'd2009;
            7'd6: c_tab <= 16'd2410;
            7'd7: c_tab <= 16'd2811;
            7'd8: c_tab <= 16'd3212;
            7'd9: c_tab <= 16'd3612;
            7'd10: c_tab <= 16'd4011;
            7'd11: c_tab <= 16'd4410;
            7'd12: c_tab <= 16'd4808;
            7'd13: c_tab <= 16'd5205;
            7'd14: c_tab <= 16'd5602;
            7'd15: c_tab <= 16'd5998;
            7'd16: c_tab <= 16'd6393;
            7'd17: c_tab <= 16'd6786;
            7'd18: c_tab <= 16'd7179;
            7'd19: c_tab <= 16'd7571;
            7'd20: c_tab <= 16'd7962;
            7'd21: c_tab <= 16'd8351;
            7'd22: c_tab <= 16'd8739;
            7'd23: c_tab <= 16'd9126;
            7'd24: c_tab <= 16'd9512;
            7'd25: c_tab <= 16'd9896;
            7'd26: c_tab <= 16'd10278;
            7'd27: c_tab <= 16'd10659;
            7'd28: c_tab <= 16'd11039;
            7'd29: c_tab <= 16'd11417;
            7'd30: c_tab <= 16'd11793;
            7'd31: c_tab <= 16'd12167;
            7'd32: c_tab <= 16'd12539;
            7'd33: c_tab <= 16'd12910;
            7'd34: c_tab <= 16'd13279;
            7'd35: c_tab <= 16'd13645;
            7'd36: c_tab <= 16'd14010;
            7'd37: c_tab <= 16'd14372;
            7'd38: c_tab <= 16'd14732;
            7'd39: c_tab <= 16'd15090;
            7'd40: c_tab <= 16'd15446;
            7'd41: c_tab <= 16'd15800;
            7'd42: c_tab <= 16'd16151;
            7'd43: c_tab <= 16'd16499;
            7'd44: c_tab <= 16'd16846;
            7'd45: c_tab <= 16'd17189;
            7'd46: c_tab <= 16'd17530;
            7'd47: c_tab <= 16'd17869;
            7'd48: c_tab <= 16'd18204;
            7'd49: c_tab <= 16'd18537;
            7'd50: c_tab <= 16'd18868;
            7'd51: c_tab <= 16'd19195;
            7'd52: c_tab <= 16'd19519;
            7'd53: c_tab <= 16'd19841;
            7'd54: c_tab <= 16'd20159;
            7'd55: c_tab <= 16'd20475;
            7'd56: c_tab <= 16'd20787;
            7'd57: c_tab <= 16'd21096;
            7'd58: c_tab <= 16'd21403;
            7'd59: c_tab <= 16'd21705;
            7'd60: c_tab <= 16'd22005;
            7'd61: c_tab <= 16'd22301;
            7'd62: c_tab <= 16'd22594;
            7'd63: c_tab <= 16'd22884;
            7'd64: c_tab <= 16'd23170;
            7'd65: c_tab <= 16'd23452;
            7'd66: c_tab <= 16'd23731;
            7'd67: c_tab <= 16'd24007;
            7'd68: c_tab <= 16'd24279;
            7'd69: c_tab <= 16'd24547;
            7'd70: c_tab <= 16'd24811;
            7'd71: c_tab <= 16'd25072;
            7'd72: c_tab <= 16'd25329;
            7'd73: c_tab <= 16'd25582;
            7'd74: c_tab <= 16'd25832;
            7'd75: c_tab <= 16'd26077;
            7'd76: c_tab <= 16'd26319;
            7'd77: c_tab <= 16'd26556;
            7'd78: c_tab <= 16'd26790;
            7'd79: c_tab <= 16'd27019;
            7'd80: c_tab <= 16'd27245;
            7'd81: c_tab <= 16'd27466;
            7'd82: c_tab <= 16'd27683;
            7'd83: c_tab <= 16'd27896;
            7'd84: c_tab <= 16'd28105;
            7'd85: c_tab <= 16'd28310;
            7'd86: c_tab <= 16'd28510;
            7'd87: c_tab <= 16'd28706;
            7'd88: c_tab <= 16'd28898;
            7'd89: c_tab <= 16'd29085;
            7'd90: c_tab <= 16'd29268;
            7'd91: c_tab <= 16'd29447;
            7'd92: c_tab <= 16'd29621;
            7'd93: c_tab <= 16'd29791;
            7'd94: c_tab <= 16'd29956;
            7'd95: c_tab <= 16'd30117;
            7'd96: c_tab <= 16'd30273;
            7'd97: c_tab <= 16'd30424;
            7'd98: c_tab <= 16'd30571;
            7'd99: c_tab <= 16'd30714;
            7'd100: c_tab <= 16'd30852;
            7'd101: c_tab <= 16'd30985;
            7'd102: c_tab <= 16'd31113;
            7'd103: c_tab <= 16'd31237;
            7'd104: c_tab <= 16'd31356;
            7'd105: c_tab <= 16'd31470;
            7'd106: c_tab <= 16'd31580;
            7'd107: c_tab <= 16'd31685;
            7'd108: c_tab <= 16'd31785;
            7'd109: c_tab <= 16'd31880;
            7'd110: c_tab <= 16'd31971;
            7'd111: c_tab <= 16'd32057;
            7'd112: c_tab <= 16'd32137;
            7'd113: c_tab <= 16'd32213;
            7'd114: c_tab <= 16'd32285;
            7'd115: c_tab <= 16'd32351;
            7'd116: c_tab <= 16'd32412;
            7'd117: c_tab <= 16'd32469;
            7'd118: c_tab <= 16'd32521;
            7'd119: c_tab <= 16'd32567;
            7'd120: c_tab <= 16'd32609;
            7'd121: c_tab <= 16'd32646;
            7'd122: c_tab <= 16'd32678;
            7'd123: c_tab <= 16'd32705;
            7'd124: c_tab <= 16'd32728;
            7'd125: c_tab <= 16'd32745;
            7'd126: c_tab <= 16'd32757;
            default: c_tab <= 16'd32765;
        endcase
    end
    wire signed [15:0] mod_s_r = m_neg ? -$signed({1'b0, m_tab}) : $signed({1'b0, m_tab});
    wire signed [15:0] car_s_r = c_neg ? -$signed({1'b0, c_tab}) : $signed({1'b0, c_tab});

    //-------------------------------------------------------------------------
    // 查表函数（函数体内只放纯 case 常量）
    //-------------------------------------------------------------------------
    function [31:0] inc_of;
        input [2:0] idx;
        begin
            // original 8-note major scale C4 D4 E4 F4 G4 A4 B4 C5
            case (idx)
                3'd0 :    inc_of = 32'h015F2602;   // C4
                3'd1 :    inc_of = 32'h018A2687;   // D4
                3'd2 :    inc_of = 32'h01BA6B52;   // E4
                3'd3 :    inc_of = 32'h01D4B9D4;   // F4
                3'd4 :    inc_of = 32'h020E2075;   // G4
                3'd5 :    inc_of = 32'h024E8ED9;   // A4
                3'd6 :    inc_of = 32'h0296E0E8;   // B4
                default: inc_of = 32'h02BE4BAD;   // C5
            endcase
        end
    endfunction

    function [15:0] len_of;
        input [2:0] idx;
        begin
            case (idx)
                3'd0 :    len_of = 16'd186;
                3'd1 :    len_of = 16'd166;
                3'd2 :    len_of = 16'd148;
                3'd3 :    len_of = 16'd139;
                3'd4 :    len_of = 16'd124;
                3'd5 :    len_of = 16'd110;
                3'd6 :    len_of = 16'd98;
                default: len_of = 16'd93;   // C5
            endcase
        end
    endfunction


    function signed [15:0] sat16;
        input signed [31:0] v;
        begin
            if (v > 32'sd32767)       sat16 = 16'sh7FFF;
            else if (v < -32'sd32768) sat16 = 16'sh8000;
            else                      sat16 = v[15:0];
        end
    endfunction

    //-------------------------------------------------------------------------
    // 每声部参数
    //-------------------------------------------------------------------------
    wire [7:0]  ratio_now = tim[v3] ? 8'd4 : 8'd1;
    wire [31:0] inc0  = incr[v3];
    wire [7:0]  lfo_mag = lfo_phase[23] ? (8'd255 - lfo_phase[22:15])
                                         : lfo_phase[22:15];
    wire signed [15:0] lfo_tri = lfo_phase[23]
                                 ? -$signed({1'b0, lfo_mag})
                                 :  $signed({1'b0, lfo_mag});
    wire signed [15:0] vib_add =
        (lfo_tri * $signed({1'b0, vib_depth})) >>> 6;
    wire signed [16:0] bend_sum = $signed(bend) + vib_add;
    wire signed [15:0] bend_eff =
        (bend_sum > 17'sd4000)  ? 16'sd4000 :
        (bend_sum < -17'sd4000) ? -16'sd4000 :
                                  bend_sum[15:0];
    wire signed [47:0] inc0_signed = $signed({16'd0, inc0});
    reg signed [63:0] bend_prod_r;
    reg signed [31:0] inc_eff_r;
    reg [31:0] mstep_r;
    wire signed [31:0] bend_term_r = bend_prod_r[46:15];
    wire signed [32:0] inc_eff_sum = $signed({1'b0, inc0}) + bend_term_r;
    wire [15:0] rel_step_now = tim[v3] ? 16'd4 : 16'd8;

    // KS 环路：y-x 一阶滤波 + 极小泄漏 + 前 len 个采样注入低通噪声
    wire signed [16:0] kdelta = $signed(y_w) - $signed(x_w);
    wire signed [31:0] kfilt  = (kdelta * $signed({1'b0, K_FILT}) + 32'sd128) >>> 8;
    // 泄漏项：每圈衰减约 1/128（强于原先的 1/512）。
    //   延迟线越长、每秒循环越少，靠环路滤波磨掉 2~8 kHz 噪声越慢 —— 实测原先
    //   需要接近 1 秒才干净，听感就是持续的"沙沙"。加强泄漏 + 加强环路滤波后，
    //   噪声在几十毫秒内衰减掉，音高迅速清晰。
    wire signed [31:0] kleak  = ($signed(x_w) + 32'sd16) >>> 5;
    wire               kexc_en = (excn[v3] < {1'b0, ksl[v3]});
    // 激励源必须**零均值**。
    //   原先写成 knorm = lfs[15] ? -lfs[15:1] : lfs[15:1] —— 用第 15 位当符号、
    //   低 15 位当幅度。但 LFSR 相邻位是相关的，两者并不独立，结果 **带明显直流
    //   偏置**（实测 nlp 稳定在 -6578，延迟线被灌进 -6000 的直流，KS 输出全是负值、
    //   完全不穿过零）。改成"低 15 位减去中点"，天然零均值。
    wire signed [15:0] knorm  = {1'b0, lfs[v3][14:0]} - 16'sd16384;
    // 激励带宽必须**窄于延迟线的模式间距**（间距 = fs/ksl）：
    //   延迟线越长、模式越密，带宽太宽就会同时激发好几个模式 → 听感"沙沙"、
    //   测到的过零数远高于基频。实测 ksl=93（C5）时过零精确 = 44，
    //   而 ksl=186（C4）时高达 163 —— 差别正是模式密度。
    //   这里按 ksl 分三档，**每档用常量移位**（不用变量移位量，避免综合差异）。
    wire signed [15:0] kdelta_e = $signed(knorm) - $signed(nlp[v3]);
    wire signed [15:0] nlp_n =
        (ksl[v3] <= 16'd110) ? (nlp[v3] + (kdelta_e >>> 3)) :   // 高音：带宽 ~970 Hz
        (ksl[v3] <= 16'd200) ? (nlp[v3] + (kdelta_e >>> 4)) :   // 中音：~490 Hz
                               (nlp[v3] + (kdelta_e >>> 5));    // 低音：~245 Hz
    wire signed [31:0] knew   = $signed(x_w) + kfilt - kleak +
                                (kexc_en ? $signed(nlp_n) : 16'sd0);

    //-------------------------------------------------------------------------
    // 主状态机
    //-------------------------------------------------------------------------
    integer i;
    reg [2:0]  pick;
    reg        found;
    reg [31:0] t_inc;
    reg [15:0] t_len;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_v       <= {(VW+1){1'b0}};
            slot        <= 8'd0;
            steal_ptr   <= 3'd0;
            frm_cnt     <= 8'd0;
            lfo_phase   <= 24'd0;
            v_valid     <= 1'b0;
            v_index     <= 3'd0;
            v_sample    <= 16'sd0;
            alloc_voice <= 3'd0;
            active_mask <= 8'd0;
            ks_rd_addr  <= {(VW+9){1'b0}};
            ks_we       <= 1'b0;
            ks_wr_addr  <= {(VW+9){1'b0}};
            ks_wr_data  <= 16'd0;
            x_w <= 16'sd0; y_w <= 16'sd0; fm_out <= 16'sd0;
            mod_scaled <= 32'sd0; ks_new <= 16'sd0; cadr_r <= 10'd0;
            bend_prod_r <= 64'sd0;
            inc_eff_r <= 32'sd0;
            mstep_r <= 32'd0;
            for (i = 0; i < NVOICES; i = i + 1) begin
                act[i] <= 1'b0;  tim[i] <= 1'b0;  nte[i] <= 5'd0;
                phc[i] <= 32'd0; phm[i] <= 32'd0; incr[i] <= 32'd0;
                est[i] <= 3'd0;  env[i] <= 16'd0; midx[i] <= 16'd0;
                mds[i] <= 4'd9;
                ksp[i] <= 9'd0;  ksl[i] <= 16'd0; excn[i] <= 17'd0;
                vel[i] <= 16'd0; lfs[i] <= 16'hACE1; nlp[i] <= 16'd0;
            end
        end else begin
            v_valid <= 1'b0;
            ks_we   <= 1'b0;

            //--------------------------------------------------------------
            // 声部分配：note_on（优先空闲，全忙抢占最早开始）
            //--------------------------------------------------------------
            if (note_on) begin
                found = 1'b0;
                pick  = 3'd0;
                for (i = 0; i < NVOICES; i = i + 1) begin
                    if (!act[i] && !found) begin
                        pick  = i[2:0];
                        found = 1'b1;
                    end
                end
                if (!found) begin
                    pick      = steal_ptr;
                    steal_ptr <= steal_ptr + 3'd1;
                end

                t_inc = inc_of(note_idx[2:0]);
                t_len = len_of(note_idx[2:0]);
                case (note_idx[5:4])
                    2'd1: begin t_inc = t_inc << 1; t_len = t_len >> 1; end
                    2'd2: begin t_inc = t_inc >> 1; t_len = t_len << 1; end
                    default: ;
                endcase
                if (t_len == 16'd0) t_len = 16'd1;

                alloc_voice <= pick;
                act[pick]  <= 1'b1;
                tim[pick]  <= timbre;
                nte[pick]  <= note_idx;
                incr[pick] <= t_inc;
                ksl[pick]  <= t_len;
                phc[pick]  <= 32'd0;
                phm[pick]  <= 32'd0;
                est[pick]  <= timbre ? 3'd3 : 3'd1;
                env[pick]  <= timbre ? 16'd32767 : 16'd0;
                midx[pick] <= timbre ? 16'd88 : 16'd80;
                mds[pick]  <= timbre ? 4'd6 : 4'd9;
                ksp[pick]  <= 9'd0;
                excn[pick] <= 17'd0;
                vel[pick]  <= velocity;
                lfs[pick]  <= 16'hACE1 ^ {10'd0, pick, 3'd0};
                nlp[pick]  <= 16'd0;
            end

            //--------------------------------------------------------------
            // note_off / all_off → 进入释音
            //--------------------------------------------------------------
            if (all_off) begin
                for (i = 0; i < NVOICES; i = i + 1)
                    if (act[i] && est[i] != 3'd4) est[i] <= 3'd4;
            end else if (note_off) begin
                for (i = 0; i < NVOICES; i = i + 1)
                    if (act[i] && (nte[i] == note_idx) && est[i] != 3'd4)
                        est[i] <= 3'd4;
            end

            //--------------------------------------------------------------
            // 扫描槽计数
            //--------------------------------------------------------------
            if (frame_start) begin
                cur_v   <= {(VW+1){1'b0}};
                slot    <= 8'd0;
                frm_cnt <= frm_cnt + 8'd1;
                lfo_phase <= lfo_phase + 24'd1890;
            end else begin
                if (slot == SLOT_CYCLES - 1) begin
                    slot <= 8'd0;
                    if (cur_v == NVOICES - 1) cur_v <= {(VW+1){1'b0}};
                    else                      cur_v <= cur_v + 1'b1;
                end else begin
                    slot <= slot + 8'd1;
                end
            end

            //--------------------------------------------------------------
            // 每声部运算（注意读数据和取值必须错开一拍，
            //   ks_rd_data 在地址给出后的**下一拍**才有效）
            //--------------------------------------------------------------
            case (slot)
                8'd0: ks_rd_addr <= ks_addr_x;              // 发起读 x
                8'd1: ks_rd_addr <= ks_addr_y;              // 此时 ks_rd_data 已是 x
                8'd2: begin
                    x_w <= rd_val;
                    bend_prod_r <= inc0_signed * $signed(bend_eff);
                end
                8'd3: begin
                    y_w <= rd_val;
                    inc_eff_r <= inc_eff_sum[31:0];
                end
                8'd4: begin
                    // K9/K10 keep the original handoff timbres (MIDX 88/80).
                    mod_scaled <= mod_s_r * $signed({1'b0, midx[v3]});
                    mstep_r <= (ratio_now == 8'd1) ? inc_eff_r
                                                    : (inc_eff_r << 2);
                end
                8'd5: begin
                    ks_new <= sat16(knew);
                    cadr_r <= cadr;                         // 载波地址打拍
                end
                // 8'd6：c_tab 在这一拍查表（同步 ROM）
                8'd7: begin
                    fm_out <= sat16((car_s_r * $signed({1'b0, env[v3]})) >>> 15);
                end
                8'd8: begin
                    if (tim[v3]) begin
                        ks_we      <= 1'b1;
                        ks_wr_addr <= ks_addr_x;
                        ks_wr_data <= ks_new;
                    end

                    phc[v3] <= phc[v3] + inc_eff_r;
                    phm[v3] <= phm[v3] + mstep_r;

                    if (midx[v3] > 16'd1) begin
                        midx[v3] <= midx[v3] - (midx[v3] >>> mds[v3]);
                    end

                    case (est[v3])
                        3'd1: if (env[v3] >= 16'd32767 - ATT_STEP) begin
                                  env[v3] <= 16'd32767;
                                  est[v3] <= 3'd2;
                              end else env[v3] <= env[v3] + ATT_STEP;
                        3'd2: if (env[v3] <= SUS_LEVEL + DEC_STEP) begin
                                  env[v3] <= SUS_LEVEL;
                                  est[v3] <= 3'd3;
                              end else env[v3] <= env[v3] - DEC_STEP;
                        3'd4: if (env[v3] <= rel_step_now) begin
                                  env[v3] <= 16'd0;
                                  est[v3] <= 3'd0;
                                  act[v3] <= 1'b0;
                              end else env[v3] <= env[v3] - rel_step_now;
                        default: ;
                    endcase

                    v_valid  <= 1'b1;
                    v_index  <= v3;
                    v_sample <= sat16((fm_out * $signed({1'b0, vel[v3]})) >>> 15);
                    active_mask[v3] <= act[v3];
                end
                default: ;
            endcase
        end
    end

endmodule
