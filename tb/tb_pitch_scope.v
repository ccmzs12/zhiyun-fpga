`timescale 1ns/1ps

module tb_pitch_scope;
    reg        clk;
    reg        rst_n;
    reg        note_on;
    reg [5:0]  note_idx;
    reg signed [15:0] bend;
    wire       oled_scl;
    wire       oled_mosi;
    wire       oled_dc;
    wire       oled_cs;
    wire       oled_reset;
    wire [5:0] y_position;
    wire [5:0] target_y;
    wire       sample_tick;
    wire       frame_done;

    reg [7:0]  first_cmd_shift;
    reg        saw_data_dc;
    integer    scl_edges;
    integer    frame_count;
    integer    errors;

    pitch_scope #(
        .CLK_HZ          (100000000),
        .SAMPLE_CYCLES   (20),
        .SLOPE_STEP      (2),
        .RESET_CYCLES    (20),
        .POST_RESET_CYCLES (20),
        .SPI_HALF_CYCLES (16)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .note_on         (note_on),
        .note_idx        (note_idx),
        .bend            (bend),
        .oled_scl        (oled_scl),
        .oled_mosi       (oled_mosi),
        .oled_dc         (oled_dc),
        .oled_cs         (oled_cs),
        .oled_reset      (oled_reset),
        .dbg_y_position  (y_position),
        .dbg_target_y    (target_y),
        .dbg_sample_tick (sample_tick),
        .dbg_frame_done  (frame_done)
    );

    always #5 clk = ~clk;

    always @(posedge oled_scl) begin
        scl_edges <= scl_edges + 1;
        if (scl_edges < 8)
            first_cmd_shift <= {first_cmd_shift[6:0], oled_mosi};
        if (oled_dc)
            saw_data_dc <= 1'b1;
    end

    always @(posedge clk) begin
        if (frame_done)
            frame_count <= frame_count + 1;
    end

    initial begin
        clk          = 1'b0;
        rst_n        = 1'b0;
        note_on      = 1'b0;
        note_idx     = 6'd0;
        bend         = 16'sd0;
        first_cmd_shift = 8'd0;
        saw_data_dc = 1'b0;
        scl_edges   = 0;
        frame_count  = 0;
        errors       = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // Trigger a real pitch change after the OLED initialization phase.
        repeat (120) @(posedge clk);
        note_idx <= 6'd7;
        note_on  <= 1'b1;
        @(posedge clk);
        note_on <= 1'b0;

        wait (frame_count >= 1);
        repeat (20) @(posedge clk);

        if (first_cmd_shift !== 8'hAE) begin
            $display("FAIL: first OLED command=%02h expected AE", first_cmd_shift);
            errors = errors + 1;
        end
        if (!saw_data_dc) begin
            $display("FAIL: no OLED pixel data was sent");
            errors = errors + 1;
        end
        if (scl_edges < 8000) begin
            $display("FAIL: too few SPI clocks: %0d", scl_edges);
            errors = errors + 1;
        end
        if (target_y !== 6'd42) begin
            $display("FAIL: target_y=%0d expected 42", target_y);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS: tb_pitch_scope (OLED init and full frame completed)");
        else
            $display("FAIL: tb_pitch_scope (%0d errors)", errors);

        $finish;
    end

    initial begin
        #5000000;
        $display("FAIL: tb_pitch_scope timeout");
        $finish;
    end
endmodule
