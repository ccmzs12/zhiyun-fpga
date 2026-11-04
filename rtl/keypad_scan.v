`timescale 1ns/1ps

//=============================================================================
// keypad_scan.v -- 4x4 matrix keypad scanner (Appendix H)
//
// One row is driven low at a time.  After the row has settled for
// SCAN_CYCLES clocks, the synchronized column vector is inspected.  Every key
// has its own debounce counter, so simultaneous keys are not serialized by a
// global lockout.  All changes found in one row scan are queued and emitted on
// consecutive clocks.
//=============================================================================

module keypad_scan #(
    parameter integer CLK_HZ        = 50000000,
    parameter integer SCAN_CYCLES   = 2500,     // 50 us at 50 MHz
    parameter integer DEBOUNCE_SCANS = 16,       // 16 row visits = 3.2 ms
    parameter integer LOCK_CYCLES   = 750000    // retained for compatibility
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [3:0]  col_raw,
    output wire [3:0]  row,
    output reg  [3:0]  key_code,
    output reg         key_event,
    output reg         key_down,
    output reg  [15:0] key_state
);

    reg [1:0]  scan_index;
    reg [31:0] scan_counter;
    reg [3:0]  col_meta;
    reg [3:0]  col_sync;
    reg [3:0]  col_capture;
    reg [3:0]  key_index;
    reg [4:0]  stable_count [0:15];
    reg [15:0] pending_mask;
    reg [15:0] pending_down;
    reg [3:0]  pending_index;
    reg        pending_found;
    integer    column;
    integer    pending_i;

    assign row = ~(4'b0001 << scan_index);

    // Select the lowest pending key. The queue is normally drained in one or
    // two clocks, long before the next row-scan interval.
    always @(*) begin
        pending_found = 1'b0;
        pending_index = 4'd0;
        for (pending_i = 0; pending_i < 16; pending_i = pending_i + 1) begin
            if (!pending_found && pending_mask[pending_i]) begin
                pending_found = 1'b1;
                pending_index = pending_i[3:0];
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_index  <= 2'd0;
            scan_counter <= 32'd0;
            col_meta    <= 4'b1111;
            col_sync    <= 4'b1111;
            col_capture <= 4'b1111;
            key_code    <= 4'hF;
            key_event   <= 1'b0;
            key_down    <= 1'b0;
            key_state   <= 16'd0;
            pending_mask <= 16'd0;
            pending_down <= 16'd0;
            for (column = 0; column < 16; column = column + 1)
                stable_count[column] <= 5'd0;
        end else begin
            // Two-stage synchronizer for the asynchronous column pins.
            col_meta <= col_raw;
            col_sync <= col_meta;
            key_event <= 1'b0;

            if (scan_counter == SCAN_CYCLES - 1) begin
                scan_counter <= 32'd0;
                col_capture  <= col_sync;

                // A row is revisited every four scan intervals. Require the
                // same logical level on consecutive visits before committing.
                for (column = 0; column < 4; column = column + 1) begin
                    key_index = {scan_index, column[1:0]};
                    if (~col_sync[column] == key_state[key_index]) begin
                        stable_count[key_index] <= 5'd0;
                    end else if (stable_count[key_index] >= DEBOUNCE_SCANS - 1) begin
                        key_state[key_index] <= ~col_sync[column];
                        pending_mask[key_index] <= 1'b1;
                        pending_down[key_index] <= ~col_sync[column];
                        stable_count[key_index] <= 5'd0;
                    end else begin
                        stable_count[key_index] <= stable_count[key_index] + 5'd1;
                    end
                end

                scan_index <= scan_index + 2'd1;
            end else begin
                scan_counter <= scan_counter + 32'd1;
            end

            // Emit queued changes one per clock. Consecutive key changes are
            // therefore reported in the same order in which they were found.
            if (pending_found) begin
                key_code  <= pending_index;
                key_down  <= pending_down[pending_index];
                key_event <= 1'b1;
                pending_mask[pending_index] <= 1'b0;
            end
        end
    end

endmodule
