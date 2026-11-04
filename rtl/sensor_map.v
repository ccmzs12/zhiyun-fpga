`timescale 1ns/1ps

// Day 11: MPU6050 acceleration/gyro to FM bend and vibrato depth.
module sensor_map (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    input  wire signed [15:0] ay_raw,
    input  wire signed [15:0] gx_raw,

    output reg  signed [15:0] bend,
    output reg  [7:0]         vib_depth
);
    reg signed [15:0] ay_f;
    reg signed [15:0] gx_f;
    reg signed [15:0] ay_zero;
    reg signed [15:0] gx_zero;
    reg signed [22:0] ay_sum;
    reg signed [22:0] gx_sum;
    reg [5:0] cal_cnt;
    reg       cal_done;

    wire signed [16:0] ay_rel = $signed(ay_raw) - $signed(ay_zero);
    wire signed [16:0] gx_rel = $signed(gx_raw) - $signed(gx_zero);
    wire signed [23:0] ay_cal_total = $signed(ay_sum) + $signed(ay_raw);
    wire signed [23:0] gx_cal_total = $signed(gx_sum) + $signed(gx_raw);
    wire signed [16:0] ay_diff = ay_rel - $signed(ay_f);
    wire signed [16:0] gx_diff = gx_rel - $signed(gx_f);
    wire signed [16:0] ay_next_ext = $signed(ay_f) + (ay_diff >>> 4);
    wire signed [16:0] gx_next_ext = $signed(gx_f) + (gx_diff >>> 4);
    wire signed [15:0] ay_next = ay_next_ext[15:0];
    wire signed [15:0] gx_next = gx_next_ext[15:0];

    wire signed [15:0] ay_d =
        (ay_f > 16'sd400)  ? ay_f - 16'sd400 :
        (ay_f < -16'sd400) ? ay_f + 16'sd400 :
                             16'sd0;
    // dead zone on the accelerometer so hand movement / electrical noise near the
    // ultrasonic module does not produce a fake pitch bend
    wire signed [16:0] ay_dz = ($signed(ay_d) >  17'sd1500) ? $signed(ay_d) - 17'sd1500 :
                               ($signed(ay_d) < -17'sd1500) ? $signed(ay_d) + 17'sd1500 :
                                                             17'sd0;
    wire signed [16:0] bend_scaled = ay_dz >>> 1;
    wire signed [15:0] bend_next =
        (bend_scaled > 17'sd4000)  ? 16'sd4000 :
        (bend_scaled < -17'sd4000) ? -16'sd4000 :
                                     bend_scaled[15:0];
    wire signed [16:0] bend_delta = $signed(bend_next) - $signed(bend);
    wire signed [16:0] bend_step =
        (bend_delta > 17'sd16)   ? 17'sd16 :
        (bend_delta < -17'sd16)  ? -17'sd16 :
                                    bend_delta;

    wire signed [16:0] gx_ext = {gx_f[15], gx_f};
    wire [16:0] gx_abs = gx_ext[16] ? -gx_ext : gx_ext;
    wire [16:0] gx_dead = (gx_abs > 17'd1200) ? (gx_abs - 17'd1200) : 17'd0;   // 64 -> 400: ignore noise
    wire [16:0] vib_scaled = gx_dead >>> 6;
    wire [7:0] vib_next = (vib_scaled > 17'd255) ? 8'd255 : vib_scaled[7:0];
    wire signed [9:0] vib_cur = {2'b00, vib_depth};
    wire signed [9:0] vib_hi  = vib_cur + 10'sd8;
    wire signed [9:0] vib_lo  = vib_cur - 10'sd8;
    wire signed [9:0] vib_nx  = {2'b00, vib_next};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ay_f      <= 16'sd0;
            gx_f      <= 16'sd0;
            ay_zero   <= 16'sd0;
            gx_zero   <= 16'sd0;
            ay_sum    <= 23'sd0;
            gx_sum    <= 23'sd0;
            cal_cnt   <= 6'd0;
            cal_done  <= 1'b0;
            bend      <= 16'sd0;
            vib_depth <= 8'd0;
        end else if (sample_valid) begin
            if (!cal_done) begin
                if (cal_cnt == 6'd63) begin
                    ay_zero  <= ay_cal_total[21:6];
                    gx_zero  <= gx_cal_total[21:6];
                    cal_done <= 1'b1;
                end else begin
                    ay_sum  <= ay_sum + $signed(ay_raw);
                    gx_sum  <= gx_sum + $signed(gx_raw);
                    cal_cnt <= cal_cnt + 6'd1;
                end
            end else begin
                ay_f      <= ay_next;
                gx_f      <= gx_next;
                bend      <= bend + bend_step[15:0];
                if (vib_nx > vib_hi)
                    vib_depth <= (vib_hi > 10'sd255) ? 8'd255 : vib_hi[7:0];
                else if (vib_nx < vib_lo)
                    vib_depth <= (vib_lo < 10'sd0) ? 8'd0 : vib_lo[7:0];
                else
                    vib_depth <= vib_next;
            end
        end
    end
endmodule
