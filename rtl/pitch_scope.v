`timescale 1ns/1ps

//=============================================================================
// pitch_scope.v
//
// Captures a 128-point pitch history and draws it as a scrolling line on a
// 128x64 SSD1306 OLED.  New samples enter at the right edge.  A rising note
// moves the line toward the top; a falling note moves it toward the bottom.
//=============================================================================
module pitch_scope #(
    parameter integer CLK_HZ          = 50000000,
    parameter integer SAMPLE_CYCLES   = 2000000, // 40 ms
    parameter integer SLOPE_STEP      = 2,
    parameter integer RESET_CYCLES    = 5000000, // 100 ms
    parameter integer POST_RESET_CYCLES = 2500000, // 50 ms
    parameter integer SPI_HALF_CYCLES = 4
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        note_on,
    input  wire [5:0]  note_idx,
    input  wire signed [15:0] bend,

    output wire        oled_scl,
    output wire        oled_mosi,
    output wire        oled_dc,
    output wire        oled_cs,
    output wire        oled_reset,

    output wire [5:0]  dbg_y_position,
    output wire [5:0]  dbg_target_y,
    output wire        dbg_sample_tick,
    output wire        dbg_frame_done
);
    localparam [4:0] C_RESET         = 5'd0;
    localparam [4:0] C_INIT_START    = 5'd1;
    localparam [4:0] C_INIT_WAIT     = 5'd2;
    localparam [4:0] C_READY         = 5'd3;
    localparam [4:0] C_PAGE_START    = 5'd4;
    localparam [4:0] C_PAGE_WAIT     = 5'd5;
    localparam [4:0] C_COL0_START    = 5'd6;
    localparam [4:0] C_COL0_WAIT     = 5'd7;
    localparam [4:0] C_COL1_START    = 5'd8;
    localparam [4:0] C_COL1_WAIT     = 5'd9;
    localparam [4:0] C_DATA_START    = 5'd10;
    localparam [4:0] C_DATA_WAIT     = 5'd11;
    localparam [4:0] C_POST_RESET    = 5'd12;

    localparam [4:0] INIT_LAST_INDEX = 5'd25;

    wire [5:0] trend_y;
    wire [5:0] trend_target_y;
    wire       trend_sample_valid;

    pitch_trend #(
        .CLK_HZ        (CLK_HZ),
        .SAMPLE_CYCLES (SAMPLE_CYCLES),
        .SLOPE_STEP    (SLOPE_STEP)
    ) u_trend (
        .clk          (clk),
        .rst_n        (rst_n),
        .note_on      (note_on),
        .note_idx     (note_idx),
        .bend         (bend),
        .y_pos        (trend_y),
        .target_y     (trend_target_y),
        .sample_valid (trend_sample_valid)
    );

    reg [23:0] reset_div;
    reg [4:0]  state;
    reg [4:0]  init_index;
    reg [2:0]  page;
    reg [6:0]  column;

    reg [5:0] history [0:127];
    reg       pending_sample;
    reg [5:0] pending_y;

    reg        spi_start;
    reg        spi_dc;
    reg [7:0]  spi_data;
    wire       spi_done;
    wire       spi_scl;
    wire       spi_mosi;
    wire       spi_dc_out;
    wire       spi_cs;

    reg        frame_done;
    reg        oled_reset_r;
    integer    i;

    ssd1306_spi #(
        .HALF_CYCLES (SPI_HALF_CYCLES)
    ) u_spi (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (spi_start),
        .dc_in   (spi_dc),
        .data_in (spi_data),
        .done    (spi_done),
        .cs      (spi_cs),
        .scl     (spi_scl),
        .mosi    (spi_mosi),
        .dc      (spi_dc_out)
    );

    assign oled_scl       = spi_scl;
    assign oled_mosi      = spi_mosi;
    assign oled_dc        = spi_dc_out;
    assign oled_cs        = spi_cs;
    assign oled_reset     = oled_reset_r;
    assign dbg_y_position = trend_y;
    assign dbg_target_y   = trend_target_y;
    assign dbg_sample_tick= trend_sample_valid;
    assign dbg_frame_done = frame_done;

    function [7:0] init_command;
        input [4:0] index;
        begin
            case (index)
            5'd0:  init_command = 8'hAE; // display off
            5'd1:  init_command = 8'hD5; // clock divide
            5'd2:  init_command = 8'h80;
            5'd3:  init_command = 8'hA8; // multiplex ratio
            5'd4:  init_command = 8'h3F;
            5'd5:  init_command = 8'hD3; // display offset
            5'd6:  init_command = 8'h00;
            5'd7:  init_command = 8'h40; // start line 0
            5'd8:  init_command = 8'h8D; // charge pump
            5'd9:  init_command = 8'h14;
            5'd10: init_command = 8'h20; // horizontal addressing
            5'd11: init_command = 8'h02; // page addressing for SSD1306/SH1106
            5'd12: init_command = 8'hA1; // segment remap
            5'd13: init_command = 8'hC0; // normal COM scan: page 0 at top
            5'd14: init_command = 8'hDA; // COM pins
            5'd15: init_command = 8'h12;
            5'd16: init_command = 8'h81; // contrast
            5'd17: init_command = 8'hCF;
            5'd18: init_command = 8'hD9; // pre-charge
            5'd19: init_command = 8'hF1;
            5'd20: init_command = 8'hDB; // VCOM detect
            5'd21: init_command = 8'h40;
            5'd22: init_command = 8'h2E; // deactivate scroll
            5'd23: init_command = 8'hA4; // resume RAM display
            5'd24: init_command = 8'hA6; // normal display
            5'd25: init_command = 8'hAF; // display on
            default: init_command = 8'h00;
            endcase
        end
    endfunction

    wire [5:0] current_y = history[column];
    wire [5:0] previous_y = (column == 7'd0) ? history[0] : history[column - 7'd1];
    wire [5:0] line_min = (current_y < previous_y) ? current_y : previous_y;
    wire [5:0] line_max = (current_y < previous_y) ? previous_y : current_y;
    wire [6:0] line_max_thick = {1'b0, line_max} + 7'd1;

    reg [7:0] line_byte;
    reg [5:0] pixel_y;
    integer bit_i;

    always @(*) begin
        line_byte = 8'd0;
        for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1) begin
            pixel_y = {page, bit_i[2:0]};
            if (({1'b0, pixel_y} >= {1'b0, line_min}) &&
                ({1'b0, pixel_y} <= line_max_thick))
                line_byte[bit_i] = 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_div      <= 24'd0;
            state          <= C_RESET;
            init_index     <= 5'd0;
            page           <= 3'd0;
            column         <= 7'd0;
            pending_sample <= 1'b0;
            pending_y      <= 6'd32;
            spi_start      <= 1'b0;
            spi_dc         <= 1'b0;
            spi_data       <= 8'd0;
            frame_done     <= 1'b0;
            oled_reset_r   <= 1'b0;
            for (i = 0; i < 128; i = i + 1)
                history[i] <= 6'd32;
        end else begin
            spi_start  <= 1'b0;
            frame_done <= 1'b0;

            // A pending audio sample is committed only between completed
            // OLED frames, so the displayed line can never tear mid-refresh.
            if ((state != C_READY) && trend_sample_valid) begin
                pending_sample <= 1'b1;
                pending_y      <= trend_y;
            end

            case (state)
            C_RESET: begin
                if (reset_div == RESET_CYCLES - 1) begin
                    reset_div  <= 24'd0;
                    oled_reset_r <= 1'b1;
                    state      <= C_POST_RESET;
                end else begin
                    reset_div <= reset_div + 24'd1;
                end
            end

            C_POST_RESET: begin
                oled_reset_r <= 1'b1;
                if (reset_div == POST_RESET_CYCLES - 1) begin
                    reset_div  <= 24'd0;
                    init_index <= 5'd0;
                    state      <= C_INIT_START;
                end else begin
                    reset_div <= reset_div + 24'd1;
                end
            end

            C_INIT_START: begin
                spi_dc    <= 1'b0;
                spi_data  <= init_command(init_index);
                spi_start <= 1'b1;
                state     <= C_INIT_WAIT;
            end

            C_INIT_WAIT: begin
                if (spi_done) begin
                    if (init_index == INIT_LAST_INDEX) begin
                        pending_sample <= 1'b1;
                        pending_y      <= 6'd32;
                        state          <= C_READY;
                    end else begin
                        init_index <= init_index + 5'd1;
                        state      <= C_INIT_START;
                    end
                end
            end

            C_READY: begin
                if (pending_sample) begin
                    for (i = 0; i < 127; i = i + 1)
                        history[i] <= history[i + 1];
                    history[127] <= pending_y;

                    if (trend_sample_valid) begin
                        pending_sample <= 1'b1;
                        pending_y      <= trend_y;
                    end else begin
                        pending_sample <= 1'b0;
                    end

                    page   <= 3'd0;
                    column <= 7'd0;
                    state  <= C_PAGE_START;
                end else if (trend_sample_valid) begin
                    pending_sample <= 1'b1;
                    pending_y      <= trend_y;
                end
            end

            C_PAGE_START: begin
                spi_dc    <= 1'b0;
                spi_data  <= 8'hB0 | {5'd0, page};
                spi_start <= 1'b1;
                state     <= C_PAGE_WAIT;
            end

            C_PAGE_WAIT: begin
                if (spi_done)
                    state <= C_COL0_START;
            end

            C_COL0_START: begin
                spi_dc    <= 1'b0;
                spi_data  <= 8'h00;
                spi_start <= 1'b1;
                state     <= C_COL0_WAIT;
            end

            C_COL0_WAIT: begin
                if (spi_done)
                    state <= C_COL1_START;
            end

            C_COL1_START: begin
                spi_dc    <= 1'b0;
                spi_data  <= 8'h10;
                spi_start <= 1'b1;
                state     <= C_COL1_WAIT;
            end

            C_COL1_WAIT: begin
                if (spi_done)
                    state <= C_DATA_START;
            end

            C_DATA_START: begin
                spi_dc    <= 1'b1;
                spi_data  <= line_byte;
                spi_start <= 1'b1;
                state     <= C_DATA_WAIT;
            end

            C_DATA_WAIT: begin
                if (spi_done) begin
                    if (column == 7'd127) begin
                        if (page == 3'd7) begin
                            frame_done <= 1'b1;
                            state      <= C_READY;
                        end else begin
                            page   <= page + 3'd1;
                            column <= 7'd0;
                            state  <= C_PAGE_START;
                        end
                    end else begin
                        column <= column + 7'd1;
                        state  <= C_DATA_START;
                    end
                end
            end

            default: state <= C_RESET;
            endcase
        end
    end
endmodule
