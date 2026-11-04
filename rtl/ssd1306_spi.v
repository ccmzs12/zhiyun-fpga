`timescale 1ns/1ps

//=============================================================================
// ssd1306_spi.v
//
// Minimal 4-wire SPI transmitter for an SSD1306 OLED.  CS is assumed tied low.
// Data is shifted MSB first and sampled by the display on the rising SCL edge.
//=============================================================================
module ssd1306_spi #(
    parameter integer HALF_CYCLES = 4 // 6.25 MHz SCL at 50 MHz
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       start,
    input  wire       dc_in,
    input  wire [7:0] data_in,

    output reg        done,
    output reg        cs,
    output reg        scl,
    output reg        mosi,
    output reg        dc
);
    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_LOW  = 2'd1;
    localparam [1:0] S_HIGH = 2'd2;

    reg [1:0] state;
    reg [2:0] bit_index;
    reg [7:0] half_count;
    reg [7:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            bit_index  <= 3'd7;
            half_count <= 8'd0;
            shift_reg  <= 8'd0;
            done       <= 1'b0;
            cs         <= 1'b0;
            scl        <= 1'b0;
            mosi       <= 1'b0;
            dc         <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
            S_IDLE: begin
                scl <= 1'b0;
                if (start) begin
                    shift_reg  <= data_in;
                    bit_index  <= 3'd7;
                    half_count <= 8'd0;
                    mosi       <= data_in[7];
                    dc         <= dc_in;
                    state      <= S_LOW;
                end
            end

            S_LOW: begin
                scl <= 1'b0;
                if (half_count == HALF_CYCLES - 1) begin
                    half_count <= 3'd0;
                    scl        <= 1'b1;
                    state      <= S_HIGH;
                end else begin
                    half_count <= half_count + 8'd1;
                end
            end

            S_HIGH: begin
                scl <= 1'b1;
                if (half_count == HALF_CYCLES - 1) begin
                    half_count <= 8'd0;
                    scl        <= 1'b0;

                    if (bit_index == 3'd0) begin
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        bit_index <= bit_index - 3'd1;
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        mosi      <= shift_reg[6];
                        state     <= S_LOW;
                    end
                end else begin
                    half_count <= half_count + 8'd1;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
