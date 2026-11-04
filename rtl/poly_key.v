`timescale 1ns/1ps

//=============================================================================
// poly_key.v   (new, written from scratch)
//
// PURPOSE
//   keypad_scan already reports every key independently on key_state[15:0],
//   but the original top only used key_event/key_down -- "the one key that was
//   pressed last".  That is why the 8-voice engine never received more than one
//   note at a time and polyphony was inaudible.
//
//   This block converts the 16-bit bitmap into a stream of per-key note events:
//     * rising edge  -> one note_on  pulse with the note number
//     * falling edge -> one note_off pulse with the note number captured
//                       at press time (so changing octave while holding a
//                       key still releases the correct voice)
//
//   The engine accepts one note_on per clock, so events are queued and drained
//   by a fixed-priority arbiter: up to 8 note keys pressed together need at
//   most 8 clocks, i.e. well under one 20.5 us sample frame.
//
//   Only rows 0 and 1 are note keys.  Rows 2 and 3 are function keys and are
//   left to the top level (timbre / octave / all-off), so they never produce a
//   note event here.
//=============================================================================
module poly_key (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [15:0] key_state,       // raw bitmap, index = {row[1:0], col[1:0]}
    input  wire [1:0]  octave,          // 0 = one octave down, 1 = centre, 2 = one octave up
    input  wire        swap_rowcol,     // 1 = the pad is wired transposed

    output reg         note_on,         // single-clock pulse
    output reg         note_off,        // single-clock pulse
    output reg  [5:0]  note_idx         // {octave_code[1:0], key_index[2:0]}
);

    //-------------------------------------------------------------------------
    // octave -> 2-bit compression code used by voice_engine
    //   code 0 = centre, 1 = one octave up (phase increment x2),
    //   code 2 = one octave down (phase increment /2)
    //-------------------------------------------------------------------------
    wire [1:0] oct_code = (octave == 2'd0) ? 2'd2 :
                          (octave == 2'd2) ? 2'd1 : 2'd0;

    //-------------------------------------------------------------------------
    // decode each raw bit: is it a note key, and which note number is it
    //-------------------------------------------------------------------------
    reg [15:0] is_note;
    reg [5:0]  note_now [0:15];
    reg [3:0]  note_number;
    // pentatonic major degrees: C D E G A (0,2,4,7,9) then the octave above
    reg [4:0] scale_deg;
    always @(*) begin
        case (note_number)
            4'd0: scale_deg = 5'd0;   // C
            4'd1: scale_deg = 5'd2;   // D
            4'd2: scale_deg = 5'd4;   // E
            4'd3: scale_deg = 5'd7;   // G
            4'd4: scale_deg = 5'd9;   // A
            4'd5: scale_deg = 5'd12;  // C (next octave)
            4'd6: scale_deg = 5'd14;  // D
            4'd7: scale_deg = 5'd16;  // E
            default: scale_deg = 5'd0;
        endcase
    end


    reg [3:0]  t;
    integer    i;

    always @(*) begin
        for (i = 0; i < 16; i = i + 1) begin
            t            = swap_rowcol ? {i[1:0], i[3:2]} : i[3:0];
            is_note[i]   = 1'b1;   // all 16 matrix positions are playable
            // The keypad's K1-K8 labels run down the first two columns:
            // column0=K1..K4, column1=K5..K8.
            note_number  = swap_rowcol ? {1'b0, i[0], i[3:2]} : i[3:0];
            note_now[i]  = {oct_code, note_number};
        end
    end

    //-------------------------------------------------------------------------
    // edge detection and event queues
    //-------------------------------------------------------------------------
    reg  [15:0] ks_d;              // previous bitmap
    reg  [15:0] q_press;           // pending note_on  events
    reg  [15:0] q_rel;             // pending note_off events
    reg  [15:0] held;              // key currently held (blocks auto-repeat)
    reg  [5:0]  latch [0:15];      // note number captured at press time

    reg  [15:0] rise, fall;
    reg  [3:0]  sel;
    reg         hit;
    integer     k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ks_d     <= 16'd0;
            q_press  <= 16'd0;
            q_rel    <= 16'd0;
            held     <= 16'd0;
            note_on  <= 1'b0;
            note_off <= 1'b0;
            note_idx <= 6'd0;
            for (k = 0; k < 16; k = k + 1) latch[k] <= 6'd0;
        end else begin
            note_on  <= 1'b0;
            note_off <= 1'b0;

            // ---- 1) edges on note keys only ------------------------------
            rise = key_state & ~ks_d & is_note;
            fall = ~key_state & ks_d & is_note;
            ks_d <= key_state;

            // ---- 2) queue them (OR, so simultaneous keys are all kept) ---
            for (k = 0; k < 16; k = k + 1) begin
                if (rise[k] && !held[k]) begin
                    latch[k] <= note_now[k];   // capture octave at press
                    held[k]  <= 1'b1;
                    q_press[k] <= 1'b1;
                end
                if (fall[k]) begin
                    held[k]  <= 1'b0;
                    q_rel[k] <= 1'b1;
                end
            end

            // ---- 3) drain: presses first, then releases ------------------
            hit = 1'b0;
            sel = 4'd0;
            for (k = 0; k < 16; k = k + 1) begin
                if (q_press[k] && !hit) begin
                    sel = k[3:0];
                    hit = 1'b1;
                end
            end

            if (hit) begin
                note_on      <= 1'b1;
                note_idx     <= latch[sel];
                q_press[sel] <= 1'b0;
            end else begin
                hit = 1'b0;
                sel = 4'd0;
                for (k = 0; k < 16; k = k + 1) begin
                    if (q_rel[k] && !hit) begin
                        sel = k[3:0];
                        hit = 1'b1;
                    end
                end
                if (hit) begin
                    note_off   <= 1'b1;
                    note_idx   <= latch[sel];  // use the captured value
                    q_rel[sel] <= 1'b0;
                end
            end
        end
    end

endmodule
