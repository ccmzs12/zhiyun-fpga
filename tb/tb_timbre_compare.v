`timescale 1ns/1ps
// Compare the current voice_engine waveform against the F89A baseline.
// Control functionality may differ, but active-voice samples must be identical.
module tb_timbre_compare;
    localparam integer CLK_PERIOD = 20;
    localparam integer FRAME_CYC  = 1024;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    integer fclk = 0;
    reg frame_start = 1'b0;
    always @(posedge clk) begin
        if (fclk == FRAME_CYC - 1) begin
            fclk <= 0;
            frame_start <= 1'b1;
        end else begin
            fclk <= fclk + 1;
            frame_start <= 1'b0;
        end
    end

    reg note_on = 1'b0;
    reg note_off = 1'b0;
    reg all_off = 1'b0;
    reg [4:0] note_idx = 5'd0;
    reg timbre = 1'b0;
    reg [15:0] velocity = 16'h7FFF;

    wire cur_valid, ref_valid;
    wire [2:0] cur_index, ref_index;
    wire signed [15:0] cur_sample, ref_sample;
    wire [7:0] cur_mask, ref_mask;
    wire [2:0] cur_alloc, ref_alloc;

    voice_engine #(.NVOICES(8), .SLOT_CYCLES(128)) dut_cur (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .note_on(note_on), .note_off(note_off), .all_off(all_off),
        .note_idx(note_idx), .timbre(timbre), .velocity(velocity),
        .bend(16'sd0), .vib_depth(8'd0),
        .alloc_voice(cur_alloc), .active_mask(cur_mask),
        .v_valid(cur_valid), .v_index(cur_index), .v_sample(cur_sample)
    );

    voice_engine_ref #(.NVOICES(8), .SLOT_CYCLES(128)) dut_ref (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .note_on(note_on), .note_off(note_off), .all_off(all_off),
        .note_idx(note_idx), .timbre(timbre), .velocity(velocity),
        .bend(16'sd0), .vib_depth(8'd0),
        .alloc_voice(ref_alloc), .active_mask(ref_mask),
        .v_valid(ref_valid), .v_index(ref_index), .v_sample(ref_sample)
    );

    reg signed [15:0] ref_latest [0:7];
    reg               ref_seen [0:7];
    integer ref_count = 0;
    integer cur_count = 0;
    integer compare_count = 0;
    integer mismatch_count = 0;
    integer errors = 0;
    integer i;

    always @(posedge clk) begin
        if (rst_n) begin
            if (ref_valid) begin
                ref_latest[ref_index] <= ref_sample;
                ref_seen[ref_index] <= 1'b1;
                ref_count = ref_count + 1;
            end
            if (cur_valid) begin
                cur_count = cur_count + 1;
                if (!ref_seen[cur_index] ||
                    ref_latest[cur_index] !== cur_sample) begin
                    mismatch_count = mismatch_count + 1;
                    if (mismatch_count <= 8)
                        $display("  mismatch voice=%0d ref=%0d cur=%0d",
                                 cur_index, ref_latest[cur_index], cur_sample);
                end else begin
                    compare_count = compare_count + 1;
                end
            end
        end
    end

    task clear_stats;
        begin
            ref_count = 0;
            cur_count = 0;
            compare_count = 0;
            mismatch_count = 0;
            for (i = 0; i < 8; i = i + 1) begin
                ref_latest[i] = 16'sd0;
                ref_seen[i] = 1'b0;
            end
        end
    endtask

    task do_reset;
        begin
            rst_n = 1'b0;
            repeat (8) @(posedge clk);
            rst_n = 1'b1;
            repeat (8) @(posedge clk);
        end
    endtask

    task press(input [4:0] n, input t);
        begin
            note_idx = n;
            timbre = t;
            note_on = 1'b1;
            @(posedge clk);
            note_on = 1'b0;
            @(posedge clk);
        end
    endtask

    task release_note(input [4:0] n);
        begin
            note_idx = n;
            note_off = 1'b1;
            @(posedge clk);
            note_off = 1'b0;
            @(posedge clk);
        end
    endtask

    task run_timbre(input t);
        integer f;
        begin
            do_reset;
            clear_stats;
            press(5'd5, t);
            for (f = 0; f < 1200; f = f + 1) @(posedge frame_start);
            release_note(5'd5);
            repeat (40) @(posedge frame_start);
            // The two implementations have different internal pipeline depth.
            // At note release one may emit its final frame one cycle later.
            if (ref_count > cur_count + 1 || cur_count > ref_count + 1) begin
                $display("  count mismatch ref=%0d cur=%0d", ref_count, cur_count);
                errors = errors + 1;
            end
            if (mismatch_count != 0 || compare_count == 0) begin
                $display("  timbre=%0d FAIL compared=%0d mismatch=%0d",
                         t, compare_count, mismatch_count);
                errors = errors + 1;
            end else begin
                $display("  timbre=%0d PASS compared=%0d", t, compare_count);
            end
        end
    endtask

    initial begin
        $display("=== baseline timbre comparison ===");
        run_timbre(1'b0);
        run_timbre(1'b1);
        if (errors == 0) $display("PASS: tb_timbre_compare");
        else             $display("FAIL: tb_timbre_compare (%0d)", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("FAIL: tb_timbre_compare timeout");
        $finish;
    end
endmodule
