`timescale 1ns/1ps

//=============================================================================
// pitch_trend.v
//
// Tracks the most recently pressed note and moves a vertical screen position
// smoothly toward that note.  Larger note numbers are drawn nearer the top of
// the display, so an upward pitch change produces an upward line.
//=============================================================================
module pitch_trend #(
    parameter integer CLK_HZ        = 50000000,
    parameter integer SAMPLE_CYCLES = 2000000, // 40 ms
    parameter integer SLOPE_STEP    = 2,       // pixels per sample
    parameter integer BEND_SHIFT    = 9,
    parameter integer BEND_LIMIT    = 6
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        note_on,
    input  wire [5:0]  note_idx,
    input  wire signed [15:0] bend,

    output reg  [5:0]  y_pos,
    output wire [5:0]  target_y,
    output reg         sample_valid
);
    localparam [5:0] Y_MIN = 6'd5;
    localparam [5:0] Y_MAX = 6'd62;
    localparam [5:0] Y_CENTER = 6'd32;

    // note_idx = {octave_code[1:0], key_index[3:0]}
    // octave_code: 2 = one octave down, 0 = centre, 1 = one octave up.
    wire [3:0] key_index = note_idx[3:0];
    wire [4:0] rank_down = {1'b0, key_index};
    wire [4:0] rank_mid  = {1'b0, key_index} + 5'd8;
    wire [4:0] rank_up   = {1'b0, key_index} + 5'd16;
    wire [4:0] note_rank =
        (note_idx[5:4] == 2'd2) ? rank_down :
        (note_idx[5:4] == 2'd1) ? rank_up   :
                                  rank_mid;

    // 24 possible key positions. This panel displays larger y nearer the top.
    wire [6:0] rank_scaled = {2'b00, note_rank} * 7'd5;
    wire [6:0] rank_offset = rank_scaled >> 1;
    wire [5:0] mapped_y = 6'd5 + rank_offset[5:0];

    reg [27:0] sample_div;
    reg [5:0]  base_y;

    // Positive bend raises pitch, so add its magnitude to move upward.
    localparam [4:0] BEND_LIMIT_5 = BEND_LIMIT;
    wire [16:0] bend_abs =
        bend[15] ? -$signed({1'b0, bend}) : $signed({1'b0, bend});
    wire [16:0] bend_scaled = bend_abs >> BEND_SHIFT;
    wire [4:0] bend_px_raw = bend_scaled[4:0];
    wire [4:0] bend_px =
        (bend_px_raw > BEND_LIMIT_5) ? BEND_LIMIT_5 : bend_px_raw;
    wire [6:0] target_neg = {1'b0, base_y} - {2'b00, bend_px};
    wire [6:0] target_pos = {1'b0, base_y} + {2'b00, bend_px};
    wire [6:0] target_mix = bend[15] ? target_neg : target_pos;
    assign target_y =
        (target_mix < Y_MIN) ? Y_MIN :
        (target_mix > Y_MAX) ? Y_MAX :
                               target_mix[5:0];

    localparam [6:0] SLOPE_STEP_7 = SLOPE_STEP;
    wire [6:0] y_up = {1'b0, y_pos} + SLOPE_STEP_7;
    wire [6:0] y_dn = {1'b0, y_pos} - SLOPE_STEP_7;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_div   <= 28'd0;
            sample_valid <= 1'b0;
            y_pos        <= Y_CENTER;
            base_y       <= Y_CENTER;
        end else begin
            sample_valid <= 1'b0;

            if (note_on)
                base_y <= mapped_y;

            if (sample_div == SAMPLE_CYCLES - 1) begin
                sample_div   <= 28'd0;
                sample_valid <= 1'b1;

                if (y_pos < target_y) begin
                    if (y_up >= {1'b0, target_y})
                        y_pos <= target_y;
                    else
                        y_pos <= y_up[5:0];
                end else if (y_pos > target_y) begin
                    if (y_dn <= {1'b0, target_y})
                        y_pos <= target_y;
                    else
                        y_pos <= y_dn[5:0];
                end
            end else begin
                sample_div <= sample_div + 28'd1;
            end
        end
    end
endmodule
