`timescale 1ns/1ps

// End-to-end keypad front-end test for a 5 ms two-key target.
// The matrix model is ideal (no ghost paths) so a failure points at debounce
// or event serialization, not at the passive keypad's hardware limitation.
module tb_twokey_5ms;
    localparam integer CLK_PERIOD       = 20;
    localparam integer SCAN_CYCLES      = 2500;
    localparam integer DEBOUNCE_SCANS   = 16;
    localparam integer HOLD_CYCLES      = 250_000; // 5 ms at 50 MHz
    localparam integer LIMIT_CYCLES     = 250_000; // 5 ms at 50 MHz

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg [15:0] pressed = 16'd0;

    wire [3:0] row;
    reg  [3:0] col;
    wire [3:0] key_code;
    wire key_event;
    wire key_down;
    wire [15:0] key_state;

    wire note_on;
    wire note_off;
    wire [4:0] note_idx;
    wire v_valid;
    wire [2:0] v_index;
    wire signed [15:0] v_sample;
    wire [2:0] alloc_voice;
    wire [7:0] active_mask;

    reg frame_start = 1'b0;
    reg [9:0] frame_div = 10'd0;

    integer r;
    integer c;
    integer on_count = 0;
    integer off_count = 0;
    integer errors = 0;
    integer guard;
    integer first_on_time = 0;
    integer last_on_time = 0;
    integer first_off_time = 0;
    integer last_off_time = 0;
    integer press_time = 0;
    integer release_time = 0;
    reg [4:0] on_notes [0:15];
    reg [4:0] off_notes [0:15];

    always #(CLK_PERIOD/2) clk = ~clk;

    keypad_scan #(
        .CLK_HZ(50000000),
        .SCAN_CYCLES(SCAN_CYCLES),
        .DEBOUNCE_SCANS(DEBOUNCE_SCANS),
        .LOCK_CYCLES(750000)
    ) u_scan (
        .clk(clk),
        .rst_n(rst_n),
        .col_raw(col),
        .row(row),
        .key_code(key_code),
        .key_event(key_event),
        .key_down(key_down),
        .key_state(key_state)
    );

    poly_key u_poly (
        .clk(clk),
        .rst_n(rst_n),
        .key_state(key_state),
        .octave(2'd1),
        .swap_rowcol(1'b1),
        .note_on(note_on),
        .note_off(note_off),
        .note_idx(note_idx)
    );

    always @(posedge clk) begin
        if (frame_div == 10'd1023) begin
            frame_div   <= 10'd0;
            frame_start <= 1'b1;
        end else begin
            frame_div   <= frame_div + 10'd1;
            frame_start <= 1'b0;
        end
    end

    voice_engine #(.NVOICES(8), .SLOT_CYCLES(128)) u_eng (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start(frame_start),
        .note_on(note_on),
        .note_off(note_off),
        .all_off(1'b0),
        .note_idx(note_idx),
        .timbre(1'b0),
        .velocity(16'h7FFF),
        .bend(16'sd0),
        .vib_depth(8'd0),
        .alloc_voice(alloc_voice),
        .active_mask(active_mask),
        .v_valid(v_valid),
        .v_index(v_index),
        .v_sample(v_sample)
    );

    // Ideal switch matrix: a pressed key pulls its column low only while its
    // row is selected by the scanner.
    always @(*) begin
        col = 4'hF;
        for (r = 0; r < 4; r = r + 1) begin
            for (c = 0; c < 4; c = c + 1) begin
                if (pressed[r*4+c] && row[r] === 1'b0)
                    col[c] = 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if (note_on) begin
                if (on_count == 0)
                    first_on_time = $time;
                last_on_time = $time;
                if (on_count < 16)
                    on_notes[on_count] = note_idx;
                on_count = on_count + 1;
                $display("note_on  t=%0t note=%0d state=%04h",
                         $time, note_idx, key_state);
            end
            if (note_off) begin
                if (off_count == 0)
                    first_off_time = $time;
                last_off_time = $time;
                if (off_count < 16)
                    off_notes[off_count] = note_idx;
                off_count = off_count + 1;
                $display("note_off t=%0t note=%0d state=%04h",
                         $time, note_idx, key_state);
            end
        end
    end

    task clear_stats;
        begin
            on_count = 0;
            off_count = 0;
            first_on_time = 0;
            last_on_time = 0;
            first_off_time = 0;
            last_off_time = 0;
            press_time = 0;
            release_time = 0;
        end
    endtask

    task hold_and_check;
        input [15:0] keys;
        input [4:0]  note_a;
        input [4:0]  note_b;
        input [8*40-1:0] label;
        reg found_a;
        reg found_b;
        begin
            clear_stats;
            @(negedge clk);
            press_time = $time;
            pressed = keys;

            // Both keys must be accepted while still held at the 5 ms point.
            repeat (HOLD_CYCLES) @(posedge clk);
            found_a = 1'b0;
            found_b = 1'b0;
            for (guard = 0; guard < on_count && guard < 16; guard = guard + 1) begin
                if (on_notes[guard] == note_a) found_a = 1'b1;
                if (on_notes[guard] == note_b) found_b = 1'b1;
            end
            if (on_count != 2 || !found_a || !found_b) begin
                $display("FAIL: %0s expected notes %0d/%0d, got %0d events",
                         label, note_a, note_b, on_count);
                errors = errors + 1;
            end else if (active_mask[1:0] != 2'b11) begin
                $display("FAIL: %0s two events did not allocate two voices, mask=%02h",
                         label, active_mask);
                errors = errors + 1;
            end else if ((last_on_time - press_time) > (LIMIT_CYCLES * CLK_PERIOD)) begin
                $display("FAIL: %0s note-on latency exceeds 5 ms", label);
                errors = errors + 1;
            end else if ((last_on_time - first_on_time) > (LIMIT_CYCLES * CLK_PERIOD)) begin
                $display("FAIL: %0s note-on stagger exceeds 5 ms", label);
                errors = errors + 1;
            end else begin
                $display("PASS: %0s note-on latency=%0t, stagger=%0t",
                         label, last_on_time - press_time,
                         last_on_time - first_on_time);
            end

            @(negedge clk);
            release_time = $time;
            pressed = 16'd0;
            guard = 0;
            while (off_count < 2 && guard < LIMIT_CYCLES) begin
                @(posedge clk);
                guard = guard + 1;
            end
            if (off_count != 2) begin
                $display("FAIL: %0s expected 2 note_off events, got %0d",
                         label, off_count);
                errors = errors + 1;
            end else if ((last_off_time - release_time) > (LIMIT_CYCLES * CLK_PERIOD)) begin
                $display("FAIL: %0s note-off latency exceeds 5 ms", label);
                errors = errors + 1;
            end else if ((last_off_time - first_off_time) > (LIMIT_CYCLES * CLK_PERIOD)) begin
                $display("FAIL: %0s note-off stagger exceeds 5 ms", label);
                errors = errors + 1;
            end else begin
                $display("PASS: %0s note-off latency=%0t, stagger=%0t",
                         label, last_off_time - release_time,
                         last_off_time - first_off_time);
            end
            repeat (10) @(posedge clk);
        end
    endtask

    initial begin
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (200) @(posedge clk);

        // Raw bits 0/4 are physical K1/K2 and map to notes 0/1.
        hold_and_check(16'h0011, 5'd0, 5'd1, "same-row K1/K2");

        // Raw bits 0/13 are on different scan rows and map to the first/last
        // note key. Both must still arrive inside the 10 ms budget.
        hold_and_check(16'h2001, 5'd0, 5'd7, "cross-row K1/K8");

        if (errors == 0)
            $display("PASS: tb_twokey_5ms");
        else
            $display("FAIL: tb_twokey_5ms (%0d errors)", errors);
        $finish;
    end
endmodule
