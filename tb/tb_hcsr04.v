`timescale 1ns/1ps
//=============================================================================
// hcsr04 regression: shortened timing, board mapping, 8-sample average,
// smoothing, and timeout hold behavior.
//=============================================================================
module tb_hcsr04;
    reg  clk = 1'b0;
    reg  rst_n = 1'b0;
    wire trig;
    reg  echo = 1'b0;
    wire [20:0] echo_count;
    wire [15:0] dist_unit, dist_avg;
    wire [7:0]  bright;
    wire        valid;

    always #5 clk = ~clk;

    hcsr04 #(
        .TRIG_CLK(6), .PERIOD_CLK(100), .TIMEOUT_CLK(1000),
        .DIST_SHIFT(4), .D_LO(7), .D_HI(35), .MAP_GAIN(20),
        .SMOOTH_SHIFT(1), .SMOOTH_DIV(1), .NEAR_DARK(1'b1)
    ) dut (
        .clk(clk), .rst_n(rst_n), .trig(trig), .echo(echo),
        .echo_count(echo_count), .dist_unit(dist_unit), .dist_avg(dist_avg),
        .bright(bright), .valid(valid)
    );

    integer errors = 0;
    integer trigs  = 0;
    reg trig_d = 1'b0;
    always @(posedge clk) begin
        trig_d <= trig;
        if (trig && !trig_d) trigs = trigs + 1;
    end

    task do_echo(input integer delay_c, input integer width);
        begin
            @(posedge trig);
            repeat (delay_c) @(posedge clk);
            echo = 1'b1;
            repeat (width) @(posedge clk);
            echo = 1'b0;
            repeat (6) @(posedge clk);
        end
    endtask

    task fill_window(input integer width);
        integer n;
        begin
            // One measurement only fills one average slot.  Eight matching
            // measurements make dist_avg exact, then allow the 84 ms IIR to
            // settle within one LSB.
            for (n = 0; n < 8; n = n + 1) do_echo(4, width);
            repeat (20) @(posedge clk);
        end
    endtask

    task expect(input integer width, input integer exp_unit, output integer got_ok);
        integer exp_bright;
        integer bright_diff;
        begin
            if (exp_unit <= 7) exp_bright = 0;
            else if ((exp_unit - 7) * 20 >= 255) exp_bright = 255;
            else exp_bright = (exp_unit - 7) * 20;

            if (echo_count == (width - 1))
                $display("    echo_count = %0d  OK", echo_count);
            else begin
                $display("    echo_count = %0d  FAIL (want %0d)",
                         echo_count, width - 1);
                errors = errors + 1;
            end
            if (dist_unit == exp_unit)
                $display("    dist_unit  = %0d  OK", dist_unit);
            else begin
                $display("    dist_unit  = %0d  FAIL (want %0d)",
                         dist_unit, exp_unit);
                errors = errors + 1;
            end
            if (dist_avg == exp_unit)
                $display("    dist_avg   = %0d  OK", dist_avg);
            else begin
                $display("    dist_avg   = %0d  FAIL (want %0d)",
                         dist_avg, exp_unit);
                errors = errors + 1;
            end
            bright_diff = bright - exp_bright;
            if (bright_diff >= -1 && bright_diff <= 1)
                $display("    bright     = %0d  OK", bright);
            else begin
                $display("    bright     = %0d  FAIL (want %0d)",
                         bright, exp_bright);
                errors = errors + 1;
            end
            got_ok = (errors == 0);
        end
    endtask

    integer ok;
    reg [7:0] bright_hold;

    initial begin
        $display("=== tb_hcsr04 (board mapping) start ===");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // 176 -> measured count 175 -> unit 10 -> bright 60
        $display("  T1  width=176 -> unit=10, bright=60");
        fill_window(176);
        expect(176, 10, ok);

        // 240 -> measured count 239 -> unit 14 -> bright 140
        $display("  T2  width=240 -> unit=14, bright=140");
        fill_window(240);
        expect(240, 14, ok);

        // 80 -> measured count 79 -> unit 4 -> bright 0
        $display("  T3  width=80 -> unit=4, bright=0 (near = dark)");
        fill_window(80);
        expect(80, 4, ok);

        // 576 -> measured count 575 -> unit 35 -> bright 255
        $display("  T4  width=576 -> unit=35, bright=255 (far = bright)");
        fill_window(576);
        expect(576, 35, ok);

        // No ECHO must invalidate the sample but preserve the last result.
        $display("  T5  no echo -> timeout, hold last valid value");
        bright_hold = bright;
        @(posedge trig);
        repeat (1200) @(posedge clk);
        if (valid === 1'b0)
            $display("    valid = 0 (timeout)  OK");
        else begin
            $display("    valid = %0d  FAIL", valid);
            errors = errors + 1;
        end
        if (dist_unit === 16'd35)
            $display("    dist_unit held = %0d  OK", dist_unit);
        else begin
            $display("    dist_unit = %0d  FAIL (should hold 35)", dist_unit);
            errors = errors + 1;
        end
        if (dist_avg === 16'd35)
            $display("    dist_avg held = %0d  OK", dist_avg);
        else begin
            $display("    dist_avg = %0d  FAIL (should hold 35)", dist_avg);
            errors = errors + 1;
        end
        if (bright === bright_hold)
            $display("    bright held = %0d  OK", bright);
        else begin
            $display("    bright = %0d  FAIL (should hold %0d)",
                     bright, bright_hold);
            errors = errors + 1;
        end

        $display("  T6  TRIG pulses = %0d", trigs);
        if (trigs >= 5)
            $display("    TRIG periodic  OK");
        else begin
            $display("    TRIG pulses too few (%0d)  FAIL", trigs);
            errors = errors + 1;
        end

        $display("=== %s (errors=%0d) ===",
                 (errors == 0) ? "ALL PASS" : "FAIL", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("=== TIMEOUT ===");
        $finish;
    end
endmodule
