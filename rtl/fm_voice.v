`timescale 1ns/1ps

//=============================================================================
// fm_voice.v -- single-voice, two-operator FM (Appendix C)
//
// phase_c and phase_m are independent 32-bit phase accumulators.  The
// modulation index and output envelope use integer decay recurrences:
//   x <= x - (x >>> shift)
// This gives the bright attack / softer tail without a divider or an
// exponential ROM.
//=============================================================================

module fm_voice #(
    parameter [7:0]  RATIO          = 8'd2,
    parameter [15:0] I0             = 16'd300,
    parameter [3:0]  MOD_DECAY_SHIFT = 4'd14,
    parameter [3:0]  ENV_DECAY_SHIFT = 4'd14,
    parameter        ENV_HOLD        = 1'b0,   // 1 = 幅度包络保持满幅，交给外部 ADSR 控制
    parameter [31:0] MAX_SAMPLES    = 32'd65536,
    parameter [4:0]  DEFAULT_NOTE   = 5'd5,
    parameter        NOTE_MEM_FILE  = "rtl/note_inc.mem",
    parameter        SINE_MEM_FILE  = "rtl/sine.mem"
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_tick,
    input  wire               start,
    input  wire [4:0]         note_index,
    output reg  signed [15:0] sample,
    output reg                sample_valid,
    output reg                active
);

    //-------------------------------------------------------------------------
    // 正弦：1/4 周期查表（**纯 case 常量**）+ 象限折叠（纯 wire 运算）
    //   【踩过的三个坑】
    //   1) 1024 点 sine_rom 用 $readmemh 初始化 → 综合器推断成 RAM，而代码是组合读
    //      → 硬件读出来是无效数据（"每个键都是同一声沙沙"）。
    //   2) 三角波代替 → 谐波太强，FM 出来非常刺耳（过零数是基频 3 倍）。
    //   3) 抛物线近似 y = 2x − x²/256 写在**函数体内**（含乘法、取负、内部 reg 数组）
    //      → ModelSim 完全正常，但 **Gowin 综合后输出几乎为 0**：硬件实测 FM 的
    //      幅度峰值恒为 0（只剩 ±1 LSB 翻转，所以过零数还对）。
    //   ⇒ 结论：函数体里只放**纯 case 常量**（与 len_of / inc_of 一样，硬件已验证），
    //      所有算术与取负都放到函数外的 wire 上。
    //-------------------------------------------------------------------------
    function [15:0] sin_tab;              // 输入 0..31 → sin(k*90/32 度) * 32767
        input [4:0] k;
        begin
            case (k)
                5'd0: sin_tab = 16'd0;
                5'd1: sin_tab = 16'd1608;
                5'd2: sin_tab = 16'd3212;
                5'd3: sin_tab = 16'd4808;
                5'd4: sin_tab = 16'd6393;
                5'd5: sin_tab = 16'd7962;
                5'd6: sin_tab = 16'd9512;
                5'd7: sin_tab = 16'd11039;
                5'd8: sin_tab = 16'd12539;
                5'd9: sin_tab = 16'd14010;
                5'd10: sin_tab = 16'd15446;
                5'd11: sin_tab = 16'd16846;
                5'd12: sin_tab = 16'd18204;
                5'd13: sin_tab = 16'd19519;
                5'd14: sin_tab = 16'd20787;
                5'd15: sin_tab = 16'd22005;
                5'd16: sin_tab = 16'd23170;
                5'd17: sin_tab = 16'd24279;
                5'd18: sin_tab = 16'd25329;
                5'd19: sin_tab = 16'd26319;
                5'd20: sin_tab = 16'd27245;
                5'd21: sin_tab = 16'd28105;
                5'd22: sin_tab = 16'd28898;
                5'd23: sin_tab = 16'd29621;
                5'd24: sin_tab = 16'd30273;
                5'd25: sin_tab = 16'd30852;
                5'd26: sin_tab = 16'd31356;
                5'd27: sin_tab = 16'd31785;
                5'd28: sin_tab = 16'd32137;
                5'd29: sin_tab = 16'd32412;
                5'd30: sin_tab = 16'd32609;
                default: sin_tab = 16'd32728;   // 5'd31 → 约 87.2 度
            endcase
        end
    endfunction

    // 相位增量表：同样用 case 常量（见 ks_voice.v 里那段说明）
    function [31:0] inc_of;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: inc_of = 32'h015F2602;   // C4
                3'd1: inc_of = 32'h018A2687;   // D4
                3'd2: inc_of = 32'h01BA6B52;   // E4
                3'd3: inc_of = 32'h01D4B9D4;   // F4
                3'd4: inc_of = 32'h020E2075;   // G4
                3'd5: inc_of = 32'h024E8ED9;   // A4
                3'd6: inc_of = 32'h0296E0E8;   // B4
                default: inc_of = 32'h02BE4BAD; // C5
            endcase
        end
    endfunction

    reg [31:0] phase_c;
    reg [31:0] phase_m;
    reg signed [15:0] mod_index;
    reg signed [15:0] envelope;
    reg [31:0] sample_count;

    wire [31:0] base_inc;
    wire [31:0] carrier_inc;
    wire [31:0] modulator_step;
    wire [9:0]  mod_addr;
    wire signed [15:0] mod_sample;
    wire signed [31:0] mod_scaled;
    wire signed [31:0] mod_offset;
    wire [10:0] carrier_sum;
    wire [9:0]  carrier_addr;
    wire signed [15:0] carrier_sample;
    wire signed [31:0] output_scaled;
    wire signed [15:0] output_value;

    assign base_inc = inc_of(note_index[2:0]);
    // note_index[4:3]：0 = 基准  1 = 升高八度(inc<<1)  2 = 降低八度(inc>>1)  3 = 基准
    // 原来的写法是 base_inc >> sh（与 ks_voice 的方向相反），这里统一为「1 = 升八度」。
    assign carrier_inc = (note_index[4:3] == 2'd0) ? base_inc :
                         (note_index[4:3] == 2'd1) ? (base_inc << 1) :
                         (note_index[4:3] == 2'd2) ? (base_inc >> 1) :
                         base_inc;
    assign modulator_step = carrier_inc * RATIO;

    assign mod_addr = phase_m[31:22];
    // 相位(10 位) → 正弦：bit9 = 符号，bit8 = 1/4 周期镜像，bit7:3 = 表索引
    wire [7:0] mod_qx = mod_addr[8] ? (8'd255 - mod_addr[7:0]) : mod_addr[7:0];
    wire signed [15:0] mod_abs = $signed({1'b0, sin_tab(mod_qx[7:3])});
    assign mod_sample = mod_addr[9] ? -mod_abs : mod_abs;
    assign mod_scaled = mod_sample * mod_index;
    assign mod_offset = mod_scaled >>> 15;

    // Taking the low ten bits implements phase wrap modulo 1024 for both
    // positive and negative modulation offsets.
    assign carrier_sum = {1'b0, phase_c[31:22]} + mod_offset[9:0];
    assign carrier_addr = carrier_sum[9:0];
    wire [7:0] car_qx = carrier_addr[8] ? (8'd255 - carrier_addr[7:0]) : carrier_addr[7:0];
    wire signed [15:0] car_abs = $signed({1'b0, sin_tab(car_qx[7:3])});
    assign carrier_sample = carrier_addr[9] ? -car_abs : car_abs;
    assign output_scaled = carrier_sample * envelope;
    assign output_value = output_scaled >>> 15;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_c      <= 32'd0;
            phase_m      <= 32'd0;
            mod_index    <= 16'sd0;
            envelope     <= 16'sd0;
            sample_count <= 32'd0;
            sample       <= 16'sd0;
            sample_valid <= 1'b0;
            active       <= 1'b0;
        end else begin
            sample_valid <= 1'b0;

            if (start) begin
                phase_c      <= 32'd0;
                phase_m      <= 32'd0;
                mod_index    <= I0;
                envelope     <= 16'sd32767;
                sample_count <= 32'd0;
                sample       <= 16'sd0;
                active       <= 1'b1;
            end else if (active && sample_tick) begin
                sample       <= output_value;
                sample_valid <= 1'b1;

                phase_c      <= phase_c + carrier_inc;
                phase_m      <= phase_m + modulator_step;
                mod_index    <= mod_index - (mod_index >>> MOD_DECAY_SHIFT);
                if (!ENV_HOLD)
                    envelope <= envelope - (envelope >>> ENV_DECAY_SHIFT);
                sample_count <= sample_count + 32'd1;

                if (sample_count + 32'd1 >= MAX_SAMPLES)
                    active <= 1'b0;
            end
        end
    end

endmodule
