`timescale 1ns/1ps

module tb_poly_key_diag;
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

    integer r;
    integer c;
    integer on_count = 0;
    integer off_count = 0;
    integer errors = 0;
    integer first_on_time = 0;
    integer last_on_time = 0;
    integer first_off_time = 0;
    integer last_off_time = 0;
    reg [4:0] on_notes [0:15];
    reg [4:0] off_notes [0:15];

    always #10 clk = ~clk;

    keypad_scan #(
        .SCAN_CYCLES(40),
        .LOCK_CYCLES(30)
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

    // Open-drain rows: selected row is low, all others are released high-Z.
    // Columns are pulled up and go low only through a pressed key on the
    // selected row.
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
                on_notes[on_count] = note_idx;
                on_count = on_count + 1;
                $display("note_on  t=%0t note=%02h key_state=%04h",
                         $time, note_idx, key_state);
            end
            if (note_off) begin
                if (off_count == 0)
                    first_off_time = $time;
                last_off_time = $time;
                off_notes[off_count] = note_idx;
                off_count = off_count + 1;
                $display("note_off t=%0t note=%02h key_state=%04h",
                         $time, note_idx, key_state);
            end
        end
    end

    initial begin
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (200) @(posedge clk);

        // Physical K1 and K8 are raw matrix bits 0 and 13 with the current
        // row/column swap.
        pressed[0]  = 1'b1;
        pressed[13] = 1'b1;

        repeat (4000) @(posedge clk);
        if (on_count != 2) begin
            $display("FAIL: expected 2 note_on events, got %0d", on_count);
            errors = errors + 1;
        end else begin
            $display("INFO: note_on stagger=%0d ns, notes=%02h %02h",
                     last_on_time - first_on_time,
                     on_notes[0], on_notes[1]);
        end

        pressed = 16'd0;
        repeat (4000) @(posedge clk);
        if (off_count != 2) begin
            $display("FAIL: expected 2 note_off events, got %0d", off_count);
            errors = errors + 1;
        end else begin
            $display("INFO: note_off stagger=%0d ns, notes=%02h %02h",
                     last_off_time - first_off_time,
                     off_notes[0], off_notes[1]);
        end

        if (errors == 0)
            $display("PASS: simultaneous K1/K8 reached poly_key without loss");
        else
            $display("FAIL: %0d errors", errors);
        $finish;
    end
endmodule
