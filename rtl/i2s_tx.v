//=============================================================================
// i2s_tx.v -- Philips I2S transmitter (Appendix A)
//
// Single 50 MHz clock. A free-running 10-bit counter counts 0..1023:
//   BCK        = cnt[3]           -> 50 MHz / 16     = 3.125 MHz
//   LRCK       = cnt[9]           -> 50 MHz / 1024   = 48.828 kHz
//   frame_tick = (cnt == 1023)    -> one clk pulse per frame
//
// Philips I2S format:
//   - LRCK low  = left channel, LRCK high = right channel
//   - one BCK period of delay after the LRCK edge, then MSB first
//   - din is updated near the BCK falling edge and sampled by the DAC on
//     the BCK rising edge
//   - 32 BCK periods per channel: 16 data bits followed by 16 zeros
//
// sample_l / sample_r are latched at cnt == 0 so they stay stable for the
// whole frame. For mono output tie sample_l and sample_r together.
//=============================================================================

module i2s_tx (
    input  wire               clk,        // 50 MHz
    input  wire               rst_n,      // active low
    input  wire signed [15:0] sample_l,   // left channel sample
    input  wire signed [15:0] sample_r,   // right channel sample
    output reg                din,        // serial audio data
    output wire               bck,        // 3.125 MHz
    output wire               lrck,       // 48.828 kHz
    output wire               frame_tick  // one clk pulse per frame
);

    reg [9:0] cnt;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) cnt <= 10'd0;
        else        cnt <= cnt + 10'd1;

    assign bck        = cnt[3];
    assign lrck       = cnt[9];
    assign frame_tick = (cnt == 10'd1023);

    reg [15:0] l_lat, r_lat;        // latched at the start of each frame
    wire [4:0] s = cnt[8:4];        // BCK index inside the channel, 0..31

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l_lat <= 16'd0;
            r_lat <= 16'd0;
            din   <= 1'b0;
        end else begin
            if (cnt == 10'd0) begin
                l_lat <= sample_l;
                r_lat <= sample_r;
            end

            // Update din at the BCK falling edge (cnt[3:0] == 0).
            // s == 0 is the one-bit I2S delay; s == 1 is the MSB.
            if (cnt[3:0] == 4'd0) begin
                if (s >= 5'd1 && s <= 5'd16)
                    din <= lrck ? r_lat[5'd16 - s] : l_lat[5'd16 - s];
                else
                    din <= 1'b0;
            end
        end
    end

endmodule
