`timescale 1ns/1ps

// Day 10/11 top with a hardware pitch-history display:
//   - 8-voice polyphony with the original B1F1 FM presets on K9/K10
//   - MPU6050 14-byte burst read on AA20/AA19
//   - acceleration/gyro mapped to FM bend and vibrato
//   - latest played note rendered as a rolling line on a 128x64 SPI OLED
module top #(
    parameter integer SCAN_CYCLES = 2500,
    parameter integer DEBOUNCE_SCANS = 16,
    parameter integer LOCK_CYCLES = 750000,
    parameter         SWAP_ROWCOL = 1'b1
) (
    input  wire       sys_clk,
    input  wire       sys_rst_n,
    input  wire [1:0] Key_in,
    input  wire [3:0] kp_col,
    output wire [3:0] kp_row,
    output wire       HP_BCK,
    output wire       HP_WS,
    output wire       HP_DIN,
    output wire       PA_EN,
    output wire       txp,
    output wire       event_led,
    // ---- pitch-history OLED ----
    output wire       oled_scl,
    output wire       oled_mosi,
    output wire       oled_dc,
    output wire       oled_cs,
    output wire       oled_reset,
    inout  wire       i2c_scl,
    inout  wire       i2c_sda,

    output wire       us_trig,
    input  wire       us_echo
);
    wire i2c_scl_oe_i, i2c_sda_oe_i, i2c_sda_in_i, i2c_scl_in_i;
    IOBUF u_scl_iobuf (.I(1'b0), .OEN(~i2c_scl_oe_i), .IO(i2c_scl), .O(i2c_scl_in_i));
    IOBUF u_sda_iobuf (.I(1'b0), .OEN(~i2c_sda_oe_i), .IO(i2c_sda), .O(i2c_sda_in_i));

    wire        frame_tick;
    wire [3:0]  key_code;
    wire        key_event;
    wire        key_down;
    wire [15:0] key_state;

    keypad_scan #(
        .CLK_HZ(50000000),
        .SCAN_CYCLES(SCAN_CYCLES),
        .DEBOUNCE_SCANS(DEBOUNCE_SCANS),
        .LOCK_CYCLES(LOCK_CYCLES)
    ) u_keypad (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .col_raw(kp_col), .row(kp_row),
        .key_code(key_code), .key_event(key_event),
        .key_down(key_down), .key_state(key_state)
    );

    wire [3:0] key_code_t = SWAP_ROWCOL ? {key_code[1:0], key_code[3:2]} : key_code;
    wire [1:0] k_row = key_code_t[3:2];
    wire [1:0] k_col = key_code_t[1:0];
    wire [2:0] k_idx = key_code_t[2:0];

    reg [1:0] octave_r;
    reg       timbre_r;
    reg       all_off_r;

    //-------------------------------------------------------------------------
    // POLYPHONY FRONT-END
    //   keypad_scan already reports every key independently on key_state[15:0].
    //   poly_key turns each press/release edge into its own note event, so
    //   holding 2, 3, 4 or up to 8 keys really starts that many voices.
    //   The engine accepts one note_on per clock, so poly_key queues the
    //   events and drains them one per clock.
    //   Function keys (rows 2 and 3) produce no note event: they are handled
    //   by the key_event state machine below (timbre / octave / all-off).
    //-------------------------------------------------------------------------
    wire       note_on_r;
    wire       note_off_r;
    wire [5:0] note_r;
    wire [15:0] note_key_mask =
        SWAP_ROWCOL ? 16'h3333 : 16'h00FF;

    poly_key u_poly (
        .clk         (sys_clk),
        .rst_n       (sys_rst_n),
        .key_state   (key_state & note_key_mask),
        .octave      (octave_r),
        .swap_rowcol (SWAP_ROWCOL),
        .note_on     (note_on_r),
        .note_off    (note_off_r),
        .note_idx    (note_r)
    );

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            octave_r  <= 2'd1;
            timbre_r  <= 1'b0;
            all_off_r <= 1'b0;
        end else begin
            all_off_r <= 1'b0;
            if (key_event && key_down && (k_row == 2'd2 || k_row == 2'd3)) begin
                case (k_row)
                2'd2: begin
                    if (k_col == 2'd0)      timbre_r <= 1'b0; // K9 : FM A
                    else if (k_col == 2'd1) timbre_r <= 1'b1; // K10: FM B
                    else if (k_col == 2'd2) begin
                        if (octave_r != 2'd0) octave_r <= octave_r - 2'd1;
                    end else begin
                        if (octave_r != 2'd2) octave_r <= octave_r + 2'd1;
                    end
                end
                default: if (k_col == 2'd0) all_off_r <= 1'b1;
                endcase
            end
        end
    end

    wire signed [15:0] bend;
    wire [7:0]         vib_depth;
    wire        v_valid;
    wire [2:0]  v_index;
    wire signed [15:0] v_sample;
    wire [2:0]  alloc_voice;
    wire [7:0]  active_mask;

    wire [5:0] pitch_y_position;
    wire [5:0] pitch_target_y;
    wire       pitch_sample_tick;
    wire       pitch_frame_done;

    pitch_scope #(
        .CLK_HZ          (50000000),
        .SAMPLE_CYCLES   (2000000), // one history point every 40 ms
        .SLOPE_STEP      (2),
        .RESET_CYCLES    (10000000), // 200 ms power-on delay
        .POST_RESET_CYCLES (2500000), // 50 ms after releasing RESET
        .SPI_HALF_CYCLES (16)        // about 1.56 MHz for breadboard wiring
    ) u_pitch_scope (
        .clk             (sys_clk),
        .rst_n           (sys_rst_n),
        .note_on         (note_on_r),
        .note_idx        (note_r),
        .bend            (bend),
        .oled_scl        (oled_scl),
        .oled_mosi       (oled_mosi),
        .oled_dc         (oled_dc),
        .oled_cs         (oled_cs),
        .oled_reset      (oled_reset),
        .dbg_y_position  (pitch_y_position),
        .dbg_target_y    (pitch_target_y),
        .dbg_sample_tick (pitch_sample_tick),
        .dbg_frame_done  (pitch_frame_done)
    );

    voice_engine #(.NVOICES(8), .SLOT_CYCLES(128)) u_eng (
        .clk(sys_clk), .rst_n(sys_rst_n), .frame_start(frame_tick),
        .note_on(note_on_r), .note_off(note_off_r), .all_off(all_off_r),
        .note_idx(note_r), .timbre(timbre_r), .velocity(16'h7FFF),
        .bend(bend), .vib_depth(vib_depth),   // IMU: tilt = pitch bend, shake = vibrato
        .alloc_voice(alloc_voice), .active_mask(active_mask),
        .v_valid(v_valid), .v_index(v_index), .v_sample(v_sample)
    );

    wire signed [15:0] mix_l, mix_r;
    wire signed [19:0] mix_raw;
    wire [7:0]         mix_cnt;
    wire               mix_clip;
    // The optional ultrasonic volume path is intentionally removed.
    //-------------------------------------------------------------------------
    // Ultrasonic distance -> 8-level volume
    //   hcsr04 issues a TRIG every 60 ms and measures the ECHO high time.
    //   dist_filter applies a 3-point median, an 8-sample average and a slew
    //   limit so the level does not jump with the raw readings.
    //   A new measurement is detected as a change of echo_count.
    //-------------------------------------------------------------------------
    wire [20:0] us_echo_count;
    wire [15:0] us_dist_unit, us_dist_avg;
    wire [7:0]  us_bright;
    wire        us_valid;

    hcsr04 u_us (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .trig(us_trig), .echo(us_echo),
        .echo_count(us_echo_count),
        .dist_unit(us_dist_unit), .dist_avg(us_dist_avg),
        .bright(us_bright), .valid(us_valid)
    );

    reg  [20:0] us_ec_d;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) us_ec_d <= 21'd0;
        else            us_ec_d <= us_echo_count;
    end
    wire us_meas = (us_echo_count != us_ec_d) && us_valid;

    wire [15:0] df_dist_unit, df_dist_avg;
    wire [7:0]  df_state;

    dist_filter #(.DIST_SHIFT(12), .SLEW(4)) u_dist_filter (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .valid_in(us_meas), .echo_count(us_echo_count),
        .dist_unit(df_dist_unit), .dist_avg(df_dist_avg),
        .filter_state(df_state)
    );

    wire [2:0] lvl_raw = (df_dist_avg <= 16'd8)  ? 3'd0 :
                         (df_dist_avg <= 16'd11) ? 3'd1 :
                         (df_dist_avg <= 16'd14) ? 3'd2 :
                         (df_dist_avg <= 16'd17) ? 3'd3 :
                         (df_dist_avg <= 16'd20) ? 3'd4 :
                         (df_dist_avg <= 16'd23) ? 3'd5 :
                         (df_dist_avg <= 16'd26) ? 3'd6 : 3'd7;

    reg [2:0] vol_lvl;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            vol_lvl <= 3'd7;
        else if ((lvl_raw > vol_lvl + 3'd1) || (lvl_raw + 3'd1 < vol_lvl))
            vol_lvl <= lvl_raw;
    end

    wire [15:0] vol_target = (vol_lvl == 3'd0) ? 16'd5600 :
                             (vol_lvl == 3'd1) ? 16'd5300 :
                             (vol_lvl == 3'd2) ? 16'd5000 :
                             (vol_lvl == 3'd3) ? 16'd4700 :
                             (vol_lvl == 3'd4) ? 16'd4400 :
                             (vol_lvl == 3'd5) ? 16'd4100 :
                             (vol_lvl == 3'd6) ? 16'd3800 : 16'd3200;

    reg [15:0] vol_now;
    reg [9:0]  vol_div;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            vol_now <= 16'd3200;
            vol_div <= 10'd0;
        end else begin
            vol_div <= vol_div + 10'd1;
            if (vol_div == 10'd1023) begin
                if      (vol_now < vol_target) vol_now <= vol_now + 16'd1;
                else if (vol_now > vol_target) vol_now <= vol_now - 16'd1;
            end
        end
    end

    wire [15:0] master_vol = vol_now;

    mixer #(.NVOICES(8)) u_mix (
        .clk(sys_clk), .rst_n(sys_rst_n), .frame_start(frame_tick),
        .v_valid(v_valid), .v_index(v_index), .v_sample(v_sample),
        .volume(master_vol),
        .out_l(mix_l), .out_r(mix_r),
        .raw_sum(mix_raw), .chan_count(mix_cnt), .clip(mix_clip)
    );

    i2s_tx u_i2s (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .sample_l(mix_l), .sample_r(mix_r),
        .din(HP_DIN), .bck(HP_BCK), .lrck(HP_WS),
        .frame_tick(frame_tick)
    );

    assign PA_EN = 1'b1;

    wire        i2c_cmd_start, i2c_cmd_rw, i2c_cmd_rep;
    wire [6:0]  i2c_cmd_addr;
    wire [7:0]  i2c_cmd_wdata, i2c_cmd_wdata2;
    wire [1:0]  i2c_cmd_wlen;
    wire [3:0]  i2c_cmd_rlen;
    wire [7:0]  i2c_cmd_rdata, i2c_rd_data;
    wire        i2c_rd_valid, i2c_cmd_done, i2c_cmd_ack_err;
    wire [3:0]  i2c_rd_index;

    i2c_master_multi #(.CLK_HZ(50000000), .SCL_HZ(100000)) u_i2c (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .cmd_start(i2c_cmd_start), .cmd_rw(i2c_cmd_rw),
        .cmd_rep(i2c_cmd_rep), .cmd_addr(i2c_cmd_addr),
        .cmd_wdata(i2c_cmd_wdata), .cmd_wdata2(i2c_cmd_wdata2),
        .cmd_wlen(i2c_cmd_wlen), .cmd_rlen(i2c_cmd_rlen),
        .cmd_rdata(i2c_cmd_rdata),
        .rd_data(i2c_rd_data), .rd_valid(i2c_rd_valid),
        .rd_index(i2c_rd_index), .cmd_done(i2c_cmd_done),
        .cmd_ack_err(i2c_cmd_ack_err),
        .scl_oe(i2c_scl_oe_i), .sda_oe(i2c_sda_oe_i),
        .sda_in(i2c_sda_in_i), .dbg_rd_sample()
    );

    wire whoami_ok, imu_data_valid, imu_i2c_error;
    wire [7:0] whoami;
    wire signed [15:0] imu_ax, imu_ay, imu_az, imu_temp;
    wire signed [15:0] imu_gx, imu_gy, imu_gz;

    mpu6050_ctrl #(.CLK_HZ(50000000)) u_imu (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .cmd_start(i2c_cmd_start), .cmd_rw(i2c_cmd_rw), .cmd_rep(i2c_cmd_rep),
        .cmd_addr(i2c_cmd_addr), .cmd_wdata(i2c_cmd_wdata),
        .cmd_wdata2(i2c_cmd_wdata2), .cmd_wlen(i2c_cmd_wlen),
        .cmd_rlen(i2c_cmd_rlen),
        .rd_data(i2c_rd_data), .rd_valid(i2c_rd_valid),
        .rd_index(i2c_rd_index), .cmd_done(i2c_cmd_done),
        .cmd_ack_err(i2c_cmd_ack_err),
        .imu_ax(imu_ax), .imu_ay(imu_ay), .imu_az(imu_az),
        .imu_temp(imu_temp), .imu_gx(imu_gx), .imu_gy(imu_gy),
        .imu_gz(imu_gz),
        .whoami_ok(whoami_ok), .data_valid(imu_data_valid),
        .whoami(whoami), .i2c_error(imu_i2c_error)
    );

    sensor_map u_map (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .sample_valid(imu_data_valid),
        .ay_raw(imu_ay), .gx_raw(imu_gx),
        .bend(bend), .vib_depth(vib_depth)
    );

    reg [25:0] led_div;
    reg [22:0] note_led_cnt;
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            led_div <= 26'd0;
            note_led_cnt <= 23'd0;
        end else begin
            led_div <= led_div + 26'd1;
            if (note_on_r) note_led_cnt <= 23'd2500000;
            else if (note_led_cnt != 23'd0) note_led_cnt <= note_led_cnt - 23'd1;
        end
    end

    assign event_led = ~(
        (note_led_cnt != 23'd0) ? 1'b1 :
        whoami_ok              ? 1'b1 :
        imu_i2c_error          ? led_div[22] :
                                 led_div[24]
    );

    reg  [7:0]  uart_data;
    reg         uart_load, uart_go;
    reg  [4:0]  uart_idx;
    wire        uart_busy;
    reg  [24:0] uart_div;
    wire        uart_tick = (uart_div == 25'd24999999);

    function [7:0] hexc;
        input [3:0] v;
        begin
            hexc = (v < 4'd10) ? (8'h30 + {4'd0, v})
                               : (8'h41 + {4'd0, v} - 8'd10);
        end
    endfunction

    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            uart_div  <= 25'd0;
            uart_load <= 1'b0;
            uart_data <= 8'd0;
            uart_idx  <= 5'd0;
            uart_go   <= 1'b0;
        end else begin
            uart_load <= 1'b0;
            uart_div  <= uart_tick ? 25'd0 : uart_div + 25'd1;
            if (uart_tick) begin
                uart_go  <= 1'b1;
                uart_idx <= 5'd0;
            end
            if (uart_go && !uart_busy && !uart_load) begin
                case (uart_idx)
                5'd0:  uart_data <= 8'h49; // I
                5'd1:  uart_data <= 8'h4D; // M
                5'd2:  uart_data <= hexc(whoami[7:4]);
                5'd3:  uart_data <= hexc(whoami[3:0]);
                5'd4:  uart_data <= hexc({2'b00, pitch_y_position[5:4]});
                5'd5:  uart_data <= hexc(pitch_y_position[3:0]);
                5'd6:  uart_data <= hexc({2'b00, pitch_target_y[5:4]});
                5'd7:  uart_data <= hexc(pitch_target_y[3:0]);
                5'd8:  uart_data <= hexc({4'd0, pitch_frame_done});
                5'd9:  uart_data <= hexc({2'b00, note_r[5:2]});
                5'd10: uart_data <= hexc({2'b00, note_r[1:0]});
                5'd11: uart_data <= hexc(df_dist_avg[7:4]);      // filtered distance
                5'd12: uart_data <= hexc(df_dist_avg[3:0]);
                5'd13: uart_data <= hexc(us_echo_count[19:16]);  // raw echo count high
                5'd14: uart_data <= hexc(bend[7:4]);
                5'd15: uart_data <= hexc(bend[3:0]);
                5'd16: uart_data <= hexc(vib_depth[7:4]);
                5'd17: uart_data <= hexc(vib_depth[3:0]);
                5'd18: uart_data <= hexc({2'b00, whoami_ok, imu_i2c_error});
                5'd19: uart_data <= 8'h0A;
                default: uart_data <= 8'h00;
                endcase
                uart_load <= 1'b1;
                if (uart_idx == 5'd19) begin
                    uart_go  <= 1'b0;
                    uart_idx <= 5'd0;
                end else uart_idx <= uart_idx + 5'd1;
            end
        end
    end

    wire main_tx, cap_tx, cap_active;
    uart_tx u_uart (
        .clk(sys_clk), .rst_n(sys_rst_n),
        .data(uart_data), .load(uart_load), .busy(uart_busy), .tx(main_tx)
    );

    audio_cap u_cap (
        .clk(sys_clk), .rst_n(sys_rst_n), .frame_tick(frame_tick),
        .sample(mix_l), .trig_n(Key_in[1]),
        .tx(cap_tx), .active(cap_active)
    );
    assign txp = cap_active ? cap_tx : main_tx;

endmodule
