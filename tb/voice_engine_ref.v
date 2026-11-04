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
//   6..127: 空闲（运算只要 6 拍，余量充足）
//
// KS 延迟线：单块 RAM，地址 = {声部号[2:0], 指针[8:0]}，共 4096 x 16。
//   读在 slot0/1，写在 slot5，读写分时；不同声部地址高 3 位不同，互不干扰。
//=============================================================================
module voice_engine_ref #(
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
    input  wire [4:0]  note_idx,         // {八度[1:0], 键号[2:0]}
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
    reg [4:0]  nte  [0:NVOICES-1];   // 音符
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
    reg [31:0] born [0:NVOICES-1];   // 起始时间戳（抢占用）

    // KS 延迟线：单块 RAM
    (* ram_style = "block" *)
    reg [15:0] ksmem [0:(1<<(VW+9))-1];

    //-------------------------------------------------------------------------
    // 扫描指针
    //-------------------------------------------------------------------------
    reg [VW:0]   cur_v;
    reg [7:0]    slot;
    reg [31:0]   ts;
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

    wire signed [15:0] mod_s_w = sine10(madr);
    wire signed [15:0] car_s_w = sine10(cadr);

    //-------------------------------------------------------------------------
    // 查表函数（函数体内只放纯 case 常量）
    //-------------------------------------------------------------------------
    function [31:0] inc_of;
        input [2:0] idx;
        begin
            case (idx)
                3'd0:    inc_of = 32'h015F2602;   // C4
                3'd1:    inc_of = 32'h018A2687;   // D4
                3'd2:    inc_of = 32'h01BA6B52;   // E4
                3'd3:    inc_of = 32'h01D4B9D4;   // F4
                3'd4:    inc_of = 32'h020E2075;   // G4
                3'd5:    inc_of = 32'h024E8ED9;   // A4
                3'd6:    inc_of = 32'h0296E0E8;   // B4
                default: inc_of = 32'h02BE4BAD;   // C5
            endcase
        end
    endfunction

    function [15:0] len_of;
        input [2:0] idx;
        begin
            case (idx)
                3'd0:    len_of = 16'd186;
                3'd1:    len_of = 16'd166;
                3'd2:    len_of = 16'd148;
                3'd3:    len_of = 16'd139;
                3'd4:    len_of = 16'd124;
                3'd5:    len_of = 16'd110;
                3'd6:    len_of = 16'd98;
                default: len_of = 16'd93;
            endcase
        end
    endfunction

    function [15:0] sin_tab;
        input [4:0] k;
        begin
            case (k)
                5'd0:    sin_tab = 16'd0;
                5'd1:    sin_tab = 16'd1608;
                5'd2:    sin_tab = 16'd3212;
                5'd3:    sin_tab = 16'd4808;
                5'd4:    sin_tab = 16'd6393;
                5'd5:    sin_tab = 16'd7962;
                5'd6:    sin_tab = 16'd9512;
                5'd7:    sin_tab = 16'd11039;
                5'd8:    sin_tab = 16'd12539;
                5'd9:    sin_tab = 16'd14010;
                5'd10:   sin_tab = 16'd15446;
                5'd11:   sin_tab = 16'd16846;
                5'd12:   sin_tab = 16'd18204;
                5'd13:   sin_tab = 16'd19519;
                5'd14:   sin_tab = 16'd20787;
                5'd15:   sin_tab = 16'd22005;
                5'd16:   sin_tab = 16'd23170;
                5'd17:   sin_tab = 16'd24279;
                5'd18:   sin_tab = 16'd25329;
                5'd19:   sin_tab = 16'd26319;
                5'd20:   sin_tab = 16'd27245;
                5'd21:   sin_tab = 16'd28105;
                5'd22:   sin_tab = 16'd28898;
                5'd23:   sin_tab = 16'd29621;
                5'd24:   sin_tab = 16'd30273;
                5'd25:   sin_tab = 16'd30852;
                5'd26:   sin_tab = 16'd31356;
                5'd27:   sin_tab = 16'd31785;
                5'd28:   sin_tab = 16'd32137;
                5'd29:   sin_tab = 16'd32412;
                5'd30:   sin_tab = 16'd32609;
                default: sin_tab = 16'd32728;
            endcase
        end
    endfunction

    function signed [15:0] sine10;
        input [9:0] a;
        reg [7:0] qx;
        reg signed [15:0] av;
        begin
            qx = a[8] ? (8'd255 - a[7:0]) : a[7:0];
            av = $signed({1'b0, sin_tab(qx[7:3])});
            sine10 = a[9] ? -av : av;
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
    reg [31:0] best;
    reg [31:0] t_inc;
    reg [15:0] t_len;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_v       <= {(VW+1){1'b0}};
            slot        <= 8'd0;
            ts          <= 32'd0;
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
            mod_scaled <= 32'sd0; ks_new <= 16'sd0;
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
                born[i] <= 32'd0;
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
                    best = born[0];
                    pick = 3'd0;
                    for (i = 1; i < NVOICES; i = i + 1) begin
                        if (born[i] < best) begin
                            best = born[i];
                            pick = i[2:0];
                        end
                    end
                end

                t_inc = inc_of(note_idx[2:0]);
                t_len = len_of(note_idx[2:0]);
                case (note_idx[4:3])
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
                born[pick] <= ts;
                ts <= ts + 32'd1;
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
                    inc_eff_r <= $signed({1'b0, inc0}) + bend_term_r;
                end
                8'd4: begin
                    mod_scaled <= mod_s_w * $signed({1'b0, midx[v3]});
                    mstep_r <= (ratio_now == 8'd1) ? inc_eff_r
                                                    : (inc_eff_r << 2);
                end
                8'd5: begin
                    ks_new <= sat16(knew);
                    fm_out <= sat16((car_s_w * $signed({1'b0, env[v3]})) >>> 15);
                end
                8'd6: begin
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
