`timescale 1ns/1ps

// Compare the polyphonic engine's per-voice FM state against the original
// single-voice fm_voice module for both B1F1 presets.
module tb_tone;
    localparam integer CLK_PERIOD = 20;
    localparam integer FRAME_CYC  = 1024;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg frame_start = 1'b0;
    integer fclk = 0;

    reg        note_on = 1'b0;
    reg        note_off = 1'b0;
    reg        all_off = 1'b0;
    reg [4:0]  note_idx = 5'd0;
    reg        timbre = 1'b0;
    reg [15:0] velocity = 16'h7FFF;

    wire        v_valid;
    wire [2:0]  v_index;
    wire signed [15:0] v_sample;
    wire [2:0]  alloc_voice;
    wire [7:0]  active_mask;

    wire signed [15:0] ref_a_sample;
    wire signed [15:0] ref_b_sample;
    wire ref_a_valid, ref_b_valid, ref_a_active, ref_b_active;

    integer errs = 0;
    integer checks = 0;
    integer i;

    voice_engine #(.NVOICES(8), .SLOT_CYCLES(128)) dut (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .note_on(note_on), .note_off(note_off), .all_off(all_off),
        .note_idx(note_idx), .timbre(timbre), .velocity(velocity),
        .bend(16'sd0), .vib_depth(8'd0),
        .alloc_voice(alloc_voice), .active_mask(active_mask),
        .v_valid(v_valid), .v_index(v_index), .v_sample(v_sample)
    );

    fm_voice #(
        .RATIO(8'd1), .I0(16'd80), .MOD_DECAY_SHIFT(4'd9),
        .ENV_DECAY_SHIFT(4'd14), .ENV_HOLD(1'b1),
        .MAX_SAMPLES(32'd4000000), .DEFAULT_NOTE(5'd5),
        .NOTE_MEM_FILE("rtl/note_inc.mem"),
        .SINE_MEM_FILE("rtl/sine.mem")
    ) ref_a (
        .clk(clk), .rst_n(rst_n), .sample_tick(frame_start),
        .start(note_on), .note_index(note_idx),
        .sample(ref_a_sample), .sample_valid(ref_a_valid),
        .active(ref_a_active)
    );

    fm_voice #(
        .RATIO(8'd4), .I0(16'd88), .MOD_DECAY_SHIFT(4'd6),
        .ENV_DECAY_SHIFT(4'd14), .ENV_HOLD(1'b1),
        .MAX_SAMPLES(32'd4000000), .DEFAULT_NOTE(5'd5),
        .NOTE_MEM_FILE("rtl/note_inc.mem"),
        .SINE_MEM_FILE("rtl/sine.mem")
    ) ref_b (
        .clk(clk), .rst_n(rst_n), .sample_tick(frame_start),
        .start(note_on), .note_index(note_idx),
        .sample(ref_b_sample), .sample_valid(ref_b_valid),
        .active(ref_b_active)
    );

    always @(posedge clk) begin
        if (fclk == FRAME_CYC-1) begin
            fclk <= 0;
            frame_start <= 1'b1;
        end else begin
            fclk <= fclk + 1;
            frame_start <= 1'b0;
        end
    end

    task check(input cond);
        begin
            checks = checks + 1;
            if (!cond) begin
                errs = errs + 1;
                $display("    FAIL #%0d at %0t", checks, $time);
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

    task compare_mode(input mode, input [4:0] note, input integer frames);
        begin
            note_idx = note;
            timbre   = mode;
            @(negedge clk);
            note_on = 1'b1;
            @(negedge clk);
            note_on = 1'b0;

            for (i = 0; i < frames; i = i + 1) begin
                @(posedge clk);
                while (!v_valid || v_index !== 3'd0) @(posedge clk);
                #1;
                if (mode == 1'b0) begin
                    check(dut.phc[0] === ref_a.phase_c);
                    check(dut.phm[0] === ref_a.phase_m);
                    check(dut.midx[0] === ref_a.mod_index);
                end else begin
                    check(dut.phc[0] === ref_b.phase_c);
                    check(dut.phm[0] === ref_b.phase_m);
                    check(dut.midx[0] === ref_b.mod_index);
                end
            end
        end
    endtask

    initial begin
        do_reset;
        $display("== FM A state regression (K9 preset) ==");
        compare_mode(1'b0, 5'd0, 200);
        do_reset;
        $display("== FM B state regression (K10 preset) ==");
        compare_mode(1'b1, 5'd7, 200);

        if (errs == 0)
            $display("PASS: tb_tone (%0d checks)", checks);
        else
            $display("FAIL: tb_tone (%0d/%0d checks failed)", errs, checks);
        $finish;
    end

    initial begin
        #(CLK_PERIOD * 2_000_000);
        $display("FAIL: tb_tone timeout");
        $finish;
    end
endmodule
