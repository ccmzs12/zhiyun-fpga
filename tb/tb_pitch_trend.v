`timescale 1ns/1ps

module tb_pitch_trend;
    reg        clk;
    reg        rst_n;
    reg        note_on;
    reg [5:0]  note_idx;
    reg signed [15:0] bend;
    wire [5:0] y_pos;
    wire [5:0] target_y;
    wire       sample_valid;

    integer errors;
    integer samples;

    pitch_trend #(
        .CLK_HZ        (100000000),
        .SAMPLE_CYCLES (4),
        .SLOPE_STEP    (2)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .note_on      (note_on),
        .note_idx     (note_idx),
        .bend         (bend),
        .y_pos        (y_pos),
        .target_y     (target_y),
        .sample_valid (sample_valid)
    );

    always #5 clk = ~clk;

    task press_note;
        input [5:0] note;
        begin
            @(posedge clk);
            note_idx <= note;
            note_on  <= 1'b1;
            @(posedge clk);
            note_on <= 1'b0;
            @(posedge clk);
        end
    endtask

    task wait_samples;
        input integer count;
        begin
            for (samples = 0; samples < count; samples = samples + 1)
                @(posedge sample_valid);
            #1;
        end
    endtask

    initial begin
        clk      = 1'b0;
        rst_n    = 1'b0;
        note_on  = 1'b0;
        note_idx = 6'd0;
        bend     = 16'sd0;
        errors   = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        if (y_pos !== 6'd32 || target_y !== 6'd32) begin
            $display("FAIL: reset position y=%0d target=%0d", y_pos, target_y);
            errors = errors + 1;
        end

        // C4 (centre octave, key 0) starts at the lower edge of the middle band.
        press_note(6'd0);
        if (target_y !== 6'd25) begin
            $display("FAIL: C4 target=%0d expected 25", target_y);
            errors = errors + 1;
        end
        wait_samples(12);
        if (y_pos !== 6'd25) begin
            $display("FAIL: C4 y=%0d expected 25", y_pos);
            errors = errors + 1;
        end

        // B4 (centre octave, key 7) must move upward.
        press_note(6'd7);
        if (target_y !== 6'd42) begin
            $display("FAIL: B4 target=%0d expected 42", target_y);
            errors = errors + 1;
        end

        // Positive bend must move the target upward; negative bend downward.
        bend = 16'sd4000;
        #1;
        if (target_y !== 6'd48) begin
            $display("FAIL: positive bend target=%0d expected 48", target_y);
            errors = errors + 1;
        end
        bend = -16'sd4000;
        #1;
        if (target_y !== 6'd36) begin
            $display("FAIL: negative bend target=%0d expected 36", target_y);
            errors = errors + 1;
        end
        bend = 16'sd0;
        #1;

        wait_samples(12);
        if (y_pos !== 6'd42) begin
            $display("FAIL: rising note y=%0d expected 42", y_pos);
            errors = errors + 1;
        end

        // B5 (one octave up) must reach the top band.
        press_note(6'd23);
        if (target_y !== 6'd62) begin
            $display("FAIL: high-octave target=%0d expected 62", target_y);
            errors = errors + 1;
        end

        // C3 (one octave down) must move downward again.
        press_note(6'd32);
        if (target_y !== 6'd5) begin
            $display("FAIL: C3 target=%0d expected 5", target_y);
            errors = errors + 1;
        end
        wait_samples(24);
        if (y_pos !== 6'd5) begin
            $display("FAIL: falling note y=%0d expected 5", y_pos);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS: tb_pitch_trend");
        else
            $display("FAIL: tb_pitch_trend (%0d errors)", errors);

        $finish;
    end

    initial begin
        #200000;
        $display("FAIL: tb_pitch_trend timeout");
        $finish;
    end
endmodule
