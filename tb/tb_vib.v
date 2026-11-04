`timescale 1ns/1ps

module tb_vib;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;

    reg frame_start = 1'b0;
    reg note_on = 1'b0;
    reg note_off = 1'b0;
    reg all_off = 1'b0;
    reg [4:0] note_idx = 5'd0;
    reg timbre = 1'b0;
    reg [15:0] velocity = 16'h7FFF;
    reg signed [15:0] bend = 16'sd0;
    reg [7:0] vib_depth = 8'd255;

    wire v_valid;
    wire [2:0] v_index;
    wire signed [15:0] v_sample;
    wire [2:0] alloc_voice;
    wire [7:0] active_mask;

    voice_engine #(.NVOICES(8), .SLOT_CYCLES(16)) dut (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .note_on(note_on), .note_off(note_off), .all_off(all_off),
        .note_idx(note_idx), .timbre(timbre), .velocity(velocity),
        .bend(bend), .vib_depth(vib_depth),
        .alloc_voice(alloc_voice), .active_mask(active_mask),
        .v_valid(v_valid), .v_index(v_index), .v_sample(v_sample)
    );

    integer fclk = 0;
    integer i;
    integer errs = 0;
    integer signed min_delta = 0;
    integer signed max_delta = 0;
    integer signed delta;

    always @(posedge clk) begin
        if (fclk == 127) begin
            fclk <= 0;
            frame_start <= 1'b1;
        end else begin
            fclk <= fclk + 1;
            frame_start <= 1'b0;
        end
    end

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        note_on = 1'b1;
        @(negedge clk);
        note_on = 1'b0;

        for (i = 0; i < 9000; i = i + 1) begin
            @(posedge clk);
            while (!v_valid || v_index !== 3'd0) @(posedge clk);
            #1;
            delta = dut.inc_eff_r - dut.incr[0];
            if (delta < min_delta) min_delta = delta;
            if (delta > max_delta) max_delta = delta;
        end

        $display("  vib delta min=%0d max=%0d", min_delta, max_delta);
        if (min_delta < -100000 && max_delta > 100000) begin
            $display("PASS: tb_vib");
        end else begin
            $display("FAIL: tb_vib");
            errs = errs + 1;
        end
        $finish;
    end

    initial begin
        #50000000;
        $display("FAIL: tb_vib timeout");
        $finish;
    end
endmodule
