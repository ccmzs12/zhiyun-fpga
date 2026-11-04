`timescale 1ns/1ps

module tb_sensor_map;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;

    reg sample_valid = 1'b0;
    reg signed [15:0] ay_raw = 16'sd0;
    reg signed [15:0] gx_raw = 16'sd0;
    wire signed [15:0] bend;
    wire [7:0] vib_depth;

    sensor_map dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .ay_raw(ay_raw), .gx_raw(gx_raw),
        .bend(bend), .vib_depth(vib_depth)
    );

    integer errs = 0;
    integer i;

    task one_sample;
        begin
            @(negedge clk);
            sample_valid = 1'b1;
            @(negedge clk);
            sample_valid = 1'b0;
            @(negedge clk);
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        ay_raw = 16'sd8000;
        gx_raw = -16'sd200;
        for (i = 0; i < 64; i = i + 1) one_sample;
        for (i = 0; i < 200; i = i + 1) one_sample;
        if (bend < 16'sd20 && bend > -16'sd20) begin
            $display("  [PASS] calibrated neutral bend=%0d", bend);
        end else begin
            $display("  [FAIL] calibrated neutral bend=%0d", bend);
            errs = errs + 1;
        end

        // A 1000-LSB tilt and 1000-LSB shake are inside the new dead zones
        // (AY 1500, GX 1200) and must not move the musical controls.
        ay_raw = 16'sd9000;
        gx_raw = 16'sd800;
        for (i = 0; i < 200; i = i + 1) one_sample;
        if (bend < 16'sd20 && bend > -16'sd20 && vib_depth == 8'd0) begin
            $display("  [PASS] dead zones reject small motion");
        end else begin
            $display("  [FAIL] dead zones leak: bend=%0d vib=%0d", bend, vib_depth);
            errs = errs + 1;
        end

        // Move beyond both dead zones.  Calibration centred AY at 8000 and
        // GX at -200, so these measurements produce a 4000/3200 LSB change.
        ay_raw = 16'sd12000;
        gx_raw = 16'sd3000;
        for (i = 0; i < 200; i = i + 1) one_sample;
        if (bend > 16'sd100) begin
            $display("  [PASS] tilt bend=%0d", bend);
        end else begin
            $display("  [FAIL] tilt bend=%0d", bend);
            errs = errs + 1;
        end
        if (vib_depth > 8'd0) begin
            $display("  [PASS] vibrato depth=%0d", vib_depth);
        end else begin
            $display("  [FAIL] vibrato depth=%0d", vib_depth);
            errs = errs + 1;
        end

        gx_raw = 16'sd30000;
        for (i = 0; i < 400; i = i + 1) one_sample;
        if (vib_depth > 8'd240) begin
            $display("  [PASS] saturated vibrato depth=%0d", vib_depth);
        end else begin
            $display("  [FAIL] saturated vibrato depth=%0d", vib_depth);
            errs = errs + 1;
        end
        for (i = 0; i < 100; i = i + 1) one_sample;
        if (vib_depth > 8'd240) begin
            $display("  [PASS] vibrato saturation holds=%0d", vib_depth);
        end else begin
            $display("  [FAIL] vibrato saturation wrapped to %0d", vib_depth);
            errs = errs + 1;
        end

        if (errs == 0) $display("PASS: tb_sensor_map");
        else           $display("FAIL: tb_sensor_map (%0d checks)", errs);
        $finish;
    end

    initial begin
        #2000000;
        $display("FAIL: tb_sensor_map timeout");
        $finish;
    end
endmodule
