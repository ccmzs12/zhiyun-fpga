`timescale 1ns/1ps

//=============================================================================
// mixer —— 8 路声部混音器（第 9 天第 2 步）
//
//   引擎在每个采样帧内依次送来 8 个声部样点（v_valid / v_index / v_sample），
//   本模块累加到 **20 位有符号**累加器，帧末：
//       out = saturate((acc * volume) >>> 15)
//
//   8 路 x ±32767 = ±262136，需要 19 位，累加器取 20 位 → 8 路满幅也不溢出翻转。
//
//   测试口：
//     raw_sum    —— 未乘音量的 20 位累加值（验证"叠加等于各自之和"）
//     chan_count —— 本帧收到的声部数（应恒为 8）
//     clip       —— 是否触发饱和限幅
//=============================================================================
module mixer #(
    parameter integer NVOICES = 8
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,

    input  wire        v_valid,
    input  wire [2:0]  v_index,
    input  wire signed [15:0] v_sample,

    input  wire [15:0] volume,

    output reg  signed [15:0] out_l,
    output reg  signed [15:0] out_r,

    output reg  signed [19:0] raw_sum,
    output reg  [7:0]         chan_count,
    output reg                clip
);

    reg  signed [19:0] acc;
    reg         [7:0]  cnt;
    reg  signed [35:0] scaled;
    reg  signed [15:0] sat;

    wire signed [19:0] acc_s    = acc;
    wire signed [35:0] scaled_s = scaled >>> 15;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc        <= 20'sd0;
            cnt        <= 8'd0;
            out_l      <= 16'sd0;
            out_r      <= 16'sd0;
            raw_sum    <= 20'sd0;
            chan_count <= 8'd0;
            clip       <= 1'b0;
            scaled     <= 36'sd0;
            sat        <= 16'sd0;
        end else begin
            // 帧末：锁存上一帧结果并清零累加器
            if (frame_start) begin
                scaled     <= acc_s * $signed({16'd0, volume});
                raw_sum    <= acc_s;
                chan_count <= cnt;
                acc        <= 20'sd0;
                cnt        <= 8'd0;
            end

            if (v_valid) begin
                acc <= acc + {{4{v_sample[15]}}, v_sample};
                cnt <= cnt + 8'd1;
            end

            sat   <= (scaled_s > 36'sd32767)  ? 16'sh7FFF :
                     (scaled_s < -36'sd32768) ? 16'sh8000 :
                                                scaled_s[15:0];
            out_l <= sat;
            out_r <= sat;
            clip  <= (scaled_s > 36'sd32767) || (scaled_s < -36'sd32768);
        end
    end

endmodule
