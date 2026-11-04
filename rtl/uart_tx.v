`timescale 1ns/1ps

// Minimal 8-N-1 UART transmitter used for the keypad debug stream.
module uart_tx #(
    parameter integer CLK_HZ  = 50000000,
    parameter integer BAUD    = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       load,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);

    localparam integer BAUD_DIV = CLK_HZ / BAUD;

    localparam [1:0] IDLE  = 2'd0;
    localparam [1:0] START = 2'd1;
    localparam [1:0] DATA  = 2'd2;
    localparam [1:0] STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] baud_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_latch;
    wire       baud_tick = (baud_cnt == BAUD_DIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            baud_cnt   <= 16'd0;
            bit_idx    <= 3'd0;
            data_latch <= 8'd0;
            tx         <= 1'b1;
            busy       <= 1'b0;
        end else begin
            if (state == IDLE) begin
                if (load) begin
                    state      <= START;
                    baud_cnt   <= 16'd0;
                    data_latch <= data;
                    tx         <= 1'b0;
                    busy       <= 1'b1;
                end else begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                end
            end else if (baud_tick) begin
                baud_cnt <= 16'd0;
                case (state)
                    START: begin
                        tx      <= data_latch[0];
                        bit_idx <= 3'd0;
                        state   <= DATA;
                    end
                    DATA: begin
                        if (bit_idx == 3'd7) begin
                            tx    <= 1'b1;
                            state <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                            tx      <= data_latch[bit_idx + 3'd1];
                        end
                    end
                    STOP: begin
                        tx    <= 1'b1;
                        busy  <= 1'b0;
                        state <= IDLE;
                    end
                    default: begin
                        state <= IDLE;
                        tx    <= 1'b1;
                        busy  <= 1'b0;
                    end
                endcase
            end else begin
                baud_cnt <= baud_cnt + 16'd1;
            end
        end
    end

endmodule
