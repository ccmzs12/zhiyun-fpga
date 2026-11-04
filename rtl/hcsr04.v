`timescale 1ns/1ps
//=============================================================================
// hcsr04 —— 超声波测距（附录 G，第 15/16 天）
//
//   1. 每 60ms 产生一次 TRIG，高电平宽度 600 个时钟（@50MHz = 12us >= 10us）
//   2. ECHO 经两级触发器同步；上升沿开始计数，下降沿停止；
//      计数超过 1.5M 个时钟（30ms）判为无效
//   3. 距离关系：50MHz 下约 2900 个时钟 = 1cm
//   4. 不做除法：dist_unit = 计数值 >> 12（1 单位约 1.4cm）
//      手在 10~50cm 对应 dist_unit 约 7~35
//   5. bright = clamp((dist_unit - 7) * 9, 0, 255)
//      附录 G 指定：近处暗、远处亮（方向可调换）
//   6. 滑动平均取最近 8 次，再做映射
//=============================================================================
module hcsr04 #(
    // ---- 时钟数常量（@50MHz）----
    parameter integer TRIG_CLK    = 600,          // 12us
    parameter integer PERIOD_CLK  = 3_000_000,    // 60ms
    parameter integer TIMEOUT_CLK = 1_500_000,    // 30ms
    parameter integer DIST_SHIFT  = 12,           // 计数值 >> 12
    parameter integer D_LO        = 7,            // 对应约 10cm
    parameter integer D_HI        = 35,           // 对应约 50cm
    parameter integer MAP_GAIN    = 9,            // (d - D_LO) * 9
    parameter integer SMOOTH_SHIFT = 12,          // IIR time constant
    parameter integer SMOOTH_DIV   = 1024,        // smoothing update divider
    // ---- 方向：1 = 近暗远亮（附录 G 默认），0 = 近亮远暗 ----
    parameter         NEAR_DARK   = 1'b1
)(
    input  wire        clk,
    input  wire        rst_n,
    output reg         trig,
    input  wire        echo,

    output reg  [20:0] echo_count,    // 最近一次 ECHO 高电平时钟数
    output reg  [15:0] dist_unit,     // 最近一次有效距离单位（>>12）
    output reg  [15:0] dist_avg,      // 8 次滑动平均后的距离单位
    output reg  [7:0]  bright,        // 映射后的亮度 0..255
    output reg         valid          // 最近一次测量有效
);

    reg [7:0]  bright_tgt;   // raw mapping result (smoothed into bright)
    reg [23:0] dist_avg_f;  // 16.8 fixed point average (low 8 bits = fraction)

    //------------------------------------------------------------------
    // ECHO 两级同步
    //------------------------------------------------------------------
    reg echo_s1, echo_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin echo_s1 <= 1'b0; echo_s2 <= 1'b0; end
        else begin echo_s1 <= echo; echo_s2 <= echo_s1; end
    end

    //------------------------------------------------------------------
    // 状态机
    //------------------------------------------------------------------
    localparam [1:0] S_IDLE = 2'd0,
                     S_TRIG = 2'd1,
                     S_HIGH = 2'd2,
                     S_MEAS = 2'd3;

    reg [1:0]  st;
    reg [21:0] per_cnt;      // 60ms = 3,000,000 < 2^22
    reg [9:0]  trg_cnt;      // 600
    reg [20:0] msr_cnt;      // 1.5M < 2^21
    reg        meas_ok;      // 本拍完成一次测量

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; per_cnt <= 22'd0; trg_cnt <= 10'd0;
            msr_cnt <= 21'd0; trig <= 1'b0; meas_ok <= 1'b0;
            echo_count <= 16'd0; valid <= 1'b0;
        end else begin
            meas_ok <= 1'b0;
            case (st)
            S_IDLE: begin
                trig <= 1'b0;
                if (per_cnt >= PERIOD_CLK - 1) begin
                    per_cnt <= 22'd0;
                    st      <= S_TRIG;
                    trg_cnt <= 10'd0;
                end else begin
                    per_cnt <= per_cnt + 22'd1;
                end
            end
            S_TRIG: begin
                trig <= 1'b1;
                if (trg_cnt >= TRIG_CLK - 1) begin
                    st      <= S_HIGH;
                    msr_cnt <= 21'd0;
                end else begin
                    trg_cnt <= trg_cnt + 10'd1;
                end
            end
            S_HIGH: begin
                trig <= 1'b0;
                if (echo_s2) begin              // 上升沿（已同步）
                    st      <= S_MEAS;
                    msr_cnt <= 21'd0;
                end else if (msr_cnt >= TIMEOUT_CLK - 1) begin
                    st        <= S_IDLE;        // 等不到回波 -> 无效
                    per_cnt   <= 22'd0;
                    valid     <= 1'b0;
                end else begin
                    msr_cnt <= msr_cnt + 21'd1;
                end
            end
            S_MEAS: begin
                if (!echo_s2) begin             // 下降沿 -> 测量完成
                    echo_count <= msr_cnt;
                    st         <= S_IDLE;
                    per_cnt    <= 22'd0;
                    meas_ok    <= (msr_cnt != 21'd0);
                    valid      <= (msr_cnt != 21'd0);
                end else if (msr_cnt >= TIMEOUT_CLK - 1) begin
                    st      <= S_IDLE;          // 高电平过长 -> 无效
                    per_cnt <= 22'd0;
                    valid   <= 1'b0;
                end else begin
                    msr_cnt <= msr_cnt + 21'd1;
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------
    // dist_unit = 计数值 >> 12（附录 G，不做除法）
    //------------------------------------------------------------------
    wire [15:0] unit_new = echo_count >> DIST_SHIFT;

    //------------------------------------------------------------------
    // 最近 8 次滑动平均 + 映射 -> bright
    //------------------------------------------------------------------
    reg [15:0] hist [0:7];
    reg [2:0]  wr;
    integer    k;
    reg [18:0] hist_sum;
    wire [18:0] hist_sum_next =
        hist_sum - {3'd0, hist[wr]} + {3'd0, unit_new};
    wire [18:0] sum = hist_sum_next;
    wire [23:0] avg_next_f = {5'd0, hist_sum_next} << 5;
    wire [23:0] d_lo_fixed = D_LO << 8;
    wire [31:0] bright_span =
        (avg_next_f > d_lo_fixed) ?
        ((avg_next_f - d_lo_fixed) * MAP_GAIN) : 32'd0;
    wire [31:0] bright_scaled = bright_span >> 8;
    wire [7:0] bright_raw =
        (bright_scaled > 32'd255) ? 8'd255 : bright_scaled[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr <= 3'd0;
            for (k = 0; k < 8; k = k + 1) hist[k] <= 16'd0;
            hist_sum  <= 19'd0;
            dist_unit <= 16'd0;
            dist_avg  <= 16'd0;
            dist_avg_f <= (D_HI << 8);
            bright_tgt <= 8'd255;   /* power-on = engine x1.000, i.e. the original 88/80 timbre */
        end else if (meas_ok && valid) begin
            hist[wr]  <= unit_new;
            wr        <= wr + 3'd1;
            dist_unit <= unit_new;

            hist_sum <= hist_sum_next;
            dist_avg_f <= (sum << 5);            // sum/8 with 8 fractional bits
            dist_avg   <= (sum << 5) >> 8;        // integer part, for the serial readout   // 用本次替换最旧的一次

            // bright = clamp((d - D_LO) * MAP_GAIN, 0, 255)
            // continuous mapping from the 16.8 fixed-point average
            if (avg_next_f <= d_lo_fixed) begin
                bright_tgt <= NEAR_DARK ? 8'd0 : 8'd255;
            end else if (bright_scaled >= 32'd255) begin
                bright_tgt <= NEAR_DARK ? 8'd255 : 8'd0;
            end else if (NEAR_DARK) begin
                bright_tgt <= bright_raw;
            end else begin
                bright_tgt <= 8'd255 - bright_raw;
            end
        end
        // 无效测量：保持上一次有效值（不写 bright / dist_avg）
    end

    //------------------------------------------------------------------
    // Brightness smoothing (day 16: "silky" transitions)
    //   The mapping only updates every 60 ms and moves in steps of
    //   MAP_GAIN, so driving the modulation index straight from it gives
    //   an audible staircase.  A first-order IIR with tau ~= 80 ms turns
    //   those steps into a continuous glide.
    //     48.828 kHz tick (1024 clocks), 16-bit accumulator, >>12 -> tau ~ 84 ms
    //------------------------------------------------------------------
    reg [9:0]  sm_div;
    reg [15:0] bright_hi;      // smoothed value, 8 fractional bits
    wire signed [16:0] bright_err =
        $signed({1'b0, bright_tgt, 8'd0}) - $signed({1'b0, bright_hi});
    wire signed [16:0] bright_next_hi =
        $signed({1'b0, bright_hi}) + (bright_err >>> SMOOTH_SHIFT);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sm_div    <= 10'd0;
            bright_hi <= 16'd65280;          // 224 << 8 : power-on default
            bright    <= 8'd255;
        end else begin
            if (sm_div >= SMOOTH_DIV - 1) begin
                sm_div <= 10'd0;
                // target in 8.8 fixed point
                bright_hi <= bright_next_hi[15:0];
                bright    <= bright_hi[15:8];
            end else begin
                sm_div <= sm_div + 10'd1;
            end
        end
    end
endmodule
