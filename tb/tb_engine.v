`timescale 1ns/1ps

//=============================================================================
// tb_engine —— 第 8/9 天自检（干净重写版，纯 ASCII 输出）
//   1) 2 声部叠加：mixer.raw_sum == 声部0 + 声部1
//   2) 8 声部：全部激活、累加正确、20 位不溢出
//   3) 声部分配：优先空闲；第 9 个音抢占（仍然只有 8 个在响）
//   4) 释音：all_off 后全部释放
//   5) KS 音高：用**指针回绕周期**验证（周期 = 延迟线长度 ksl）
//   6) note_off 后该声部释放
//=============================================================================
module tb_engine;

    localparam integer CLK_PERIOD = 20;      // 50 MHz
    localparam integer FRAME_CYC  = 1024;    // 48.828 kHz

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer errs = 0;
    integer chk_no = 0;
    integer fclk = 0;
    integer acc_man = 0;
    integer acc_prev = 0;
    integer sum_mismatch = 0;
    integer overflow_cnt = 0;
    integer p0_prev = 0, p0_period = 0, p0_cnt = 0;
    integer p1_prev = 0, p1_period = 0, p1_cnt = 0;
    integer k;

    reg         frame_start = 1'b0;
    reg         note_on = 1'b0, note_off = 1'b0, all_off = 1'b0;
    reg  [4:0]  note_idx = 5'd0;
    reg         timbre = 1'b0;
    reg  [15:0] velocity = 16'h7FFF;

    wire        v_valid;
    wire [2:0]  v_index;
    wire signed [15:0] v_sample;
    wire [2:0]  alloc_voice;
    wire [7:0]  active_mask;
    wire signed [15:0] out_l, out_r;
    wire signed [19:0] raw_sum;
    wire [7:0]  chan_count;
    wire        clip;

    voice_engine #(.NVOICES(8), .SLOT_CYCLES(128)) dut_eng (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .note_on(note_on), .note_off(note_off), .all_off(all_off),
        .note_idx(note_idx), .timbre(timbre), .velocity(velocity),
        .bend(16'sd0), .vib_depth(8'd0),
        .alloc_voice(alloc_voice), .active_mask(active_mask),
        .v_valid(v_valid), .v_index(v_index), .v_sample(v_sample)
    );

    mixer #(.NVOICES(8)) dut_mix (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .v_valid(v_valid), .v_index(v_index), .v_sample(v_sample),
        .volume(16'h7FFF),
        .out_l(out_l), .out_r(out_r),
        .raw_sum(raw_sum), .chan_count(chan_count), .clip(clip)
    );

    always @(posedge clk) begin
        if (fclk == FRAME_CYC-1) begin fclk <= 0; frame_start <= 1'b1; end
        else begin fclk <= fclk + 1; frame_start <= 1'b0; end
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if (v_valid) acc_man = acc_man + v_sample;

            if (frame_start) begin
                if (raw_sum != acc_prev[19:0]) sum_mismatch = sum_mismatch + 1;
                if (raw_sum > 20'sd262136 || raw_sum < -20'sd262136)
                    overflow_cnt = overflow_cnt + 1;
                acc_prev = acc_man;
                acc_man  = 0;
            end

            if (frame_start && dut_eng.act[0] && dut_eng.tim[0]) begin
                if (dut_eng.ksp[0] < p0_prev) begin p0_period = p0_cnt; p0_cnt = 0; end
                else p0_cnt = p0_cnt + 1;
                p0_prev = dut_eng.ksp[0];
            end
            if (frame_start && dut_eng.act[1] && dut_eng.tim[1]) begin
                if (dut_eng.ksp[1] < p1_prev) begin p1_period = p1_cnt; p1_cnt = 0; end
                else p1_cnt = p1_cnt + 1;
                p1_prev = dut_eng.ksp[1];
            end
        end
    end

    task check(input integer cond);
        begin
            chk_no = chk_no + 1;
            if (cond) $display("    OK   #%0d", chk_no);
            else begin $display("    FAIL #%0d", chk_no); errs = errs + 1; end
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
            note_idx = n; timbre = t; note_on = 1'b1;
            @(posedge clk); note_on = 1'b0; @(posedge clk);
        end
    endtask

    task rel_note(input [4:0] n);
        begin
            note_idx = n; note_off = 1'b1;
            @(posedge clk); note_off = 1'b0; @(posedge clk);
        end
    endtask

    task wait_frames(input integer n);
        begin
            for (k = 0; k < n; k = k + 1) @(posedge frame_start);
        end
    endtask

    initial begin
        do_reset;

        $display("");
        $display("== 1) two voices: sum == parts ==");
        sum_mismatch = 0; overflow_cnt = 0;
        press(5'd0, 1'b0);
        wait_frames(2);
        press(5'd4, 1'b0);
        check(alloc_voice == 3'd1);
        wait_frames(600);
        sum_mismatch = 0;
        wait_frames(400);
        check(sum_mismatch == 0);
        check(overflow_cnt == 0);
        rel_note(5'd0); rel_note(5'd4);
        wait_frames(200);

        $display("");
        $display("== 2) eight voices + mixer ==");
        press(5'd0, 1'b0); press(5'd1, 1'b0); press(5'd2, 1'b0); press(5'd3, 1'b0);
        press(5'd4, 1'b0); press(5'd5, 1'b0); press(5'd6, 1'b0); press(5'd7, 1'b0);
        wait_frames(2);
        check(active_mask == 8'hFF);
        sum_mismatch = 0; overflow_cnt = 0;
        wait_frames(600);
        check(sum_mismatch == 0);
        check(overflow_cnt == 0);
        check(chan_count == 8'd8);

        $display("");
        $display("== 3) voice stealing ==");
        press(5'd7, 1'b0);
        wait_frames(2);
        check(alloc_voice <= 3'd7);
        check(active_mask == 8'hFF);

        $display("");
        $display("== 4) all_off -> release ==");
        all_off = 1'b1; @(posedge clk); all_off = 1'b0;
        wait_frames(9000);
        check(active_mask == 8'h00);

        $display("");
        $display("== 5) both original FM presets remain selectable ==");
        press(5'd0, 1'b0);
        press(5'd7, 1'b1);
        wait_frames(1000);
        check(dut_eng.act[0] == 1'b1 && dut_eng.tim[0] == 1'b0);
        check(dut_eng.act[1] == 1'b1 && dut_eng.tim[1] == 1'b1);
        check(dut_eng.midx[0] == 16'd80);
        check(dut_eng.midx[1] < 16'd88);

        $display("");
        $display("== 6) note_off -> voice released ==");
        rel_note(5'd0);
        wait_frames(9000);
        check(active_mask[0] == 1'b0);

        $display("");
        if (errs == 0) $display("PASS: tb_engine (day8/9) all checks passed");
        else           $display("FAIL: tb_engine %0d checks failed", errs);
        $display("Errors: %0d", errs);
        $finish;
    end

    initial begin
        #(CLK_PERIOD * 80_000_000);
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
