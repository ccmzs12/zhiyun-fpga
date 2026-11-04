`timescale 1ns/1ps
//=============================================================================
// dist_filter —— 测距后处理（独立模块，便于仿真验证）
//
//   解决的问题：HY-SRF05 偶尔给出"无回波"（很大）或"多径反射"（很小）的单点
//   野值，直接映射到音量/亮度会造成剧烈跳变。
//
//   处理链：
//     1) echo_count >> 12  得到原始单位（附录 G，不做除法）
//     2) 3 点中值          去掉单点野值
//     3) 8 次滑动平均      进一步平滑
//     4) 速率限制          每次测量最多变化 SLEW 个单位，避免突变
//
//   注意：中值滤波的延迟线必须存"原始值"，不能存"滤波后值"，否则会死锁在 0。
//=============================================================================
module dist_filter #(
    parameter integer DIST_SHIFT = 12,     // 计数 -> 单位
    parameter integer SLEW       = 4       // 每次测量最多变化多少个单位
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,           // 单拍脉冲：echo_count 已就绪
    input  wire [20:0] echo_count,
    output reg  [15:0] dist_unit,          // 滤波后的距离单位
    output reg  [15:0] dist_avg,           // 8 次平均（整数）
    output reg  [7:0]  filter_state        // 调试：0=正常 1=被中值修正
);
    //------------------------------------------------------------------
    // 1) 原始单位
    //------------------------------------------------------------------
    wire [15:0] u_raw = {7'd0, echo_count[20:DIST_SHIFT]};   // echo_count >> 12 (21-bit input)

    //------------------------------------------------------------------
    // 2) 3 点中值：median(a,b,c) = max(min(a,b), min(max(a,b), c))
    //    延迟线存原始值（h1 = 上一次原始，h2 = 上上次原始）
    //------------------------------------------------------------------
    reg  [15:0] h1, h2;

    wire [15:0] mn_ab = (u_raw < h1) ? u_raw : h1;
    wire [15:0] mx_ab = (u_raw > h1) ? u_raw : h1;
    wire [15:0] mc_c  = (mx_ab < h2) ? mx_ab : h2;
    wire [15:0] med   = (mn_ab > mc_c) ? mn_ab : mc_c;

    //------------------------------------------------------------------
    // 3) 8 次滑动平均（对中值结果）
    //------------------------------------------------------------------
    reg  [15:0] hist [0:7];
    reg  [2:0]  wr;
    reg  [18:0] sum;
    integer     i;

    //------------------------------------------------------------------
    // 4) 速率限制
    //------------------------------------------------------------------
    reg  [15:0] slew_next;
    reg  [15:0] avg_new;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h1 <= 16'd0; h2 <= 16'd0;
            wr <= 3'd0;
            for (i = 0; i < 8; i = i + 1) hist[i] <= 16'd0;
            dist_unit   <= 16'd0;
            dist_avg    <= 16'd0;
            filter_state <= 8'd0;
            avg_new      = 16'd0;
            slew_next    = 16'd0;
            sum          = 19'd0;
        end else begin
            // 延迟线：只在有新测量时推进，存原始值
            if (valid_in) begin
                h2 <= h1;
                h1 <= u_raw;

                // 中值进入平均窗
                hist[wr] <= med;
                wr <= wr + 3'd1;

                // 8 项求和（含本次替换最旧的一项）
                sum = {3'd0, hist[0]} + {3'd0, hist[1]} + {3'd0, hist[2]} + {3'd0, hist[3]}
                    + {3'd0, hist[4]} + {3'd0, hist[5]} + {3'd0, hist[6]} + {3'd0, hist[7]}
                    - {3'd0, hist[wr]} + {3'd0, med};
                avg_new = sum >> 3;

                dist_avg <= avg_new;

                // 速率限制：每次最多动 SLEW 个单位
                if (avg_new > dist_unit + SLEW)
                    slew_next = dist_unit + SLEW;
                else if (avg_new + SLEW < dist_unit)
                    slew_next = dist_unit - SLEW;
                else
                    slew_next = avg_new;
                dist_unit <= slew_next;

                // 调试：中值是否修正了原始值
                filter_state <= (med != u_raw) ? 8'd1 : 8'd0;
            end
        end
    end
endmodule
