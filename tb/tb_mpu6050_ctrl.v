`timescale 1ns/1ps

module tb_mpu6050_ctrl;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;

    wire        cmd_start;
    wire        cmd_rw;
    wire        cmd_rep;
    wire [6:0]  cmd_addr;
    wire [7:0]  cmd_wdata;
    wire [7:0]  cmd_wdata2;
    wire [1:0]  cmd_wlen;
    wire [3:0]  cmd_rlen;
    reg  [7:0]  rd_data = 8'd0;
    reg         rd_valid = 1'b0;
    reg  [3:0]  rd_index = 4'd0;
    reg         cmd_done = 1'b0;
    reg         cmd_ack_err = 1'b0;

    wire whoami_ok;
    wire data_valid;
    wire [7:0] whoami;
    wire i2c_error;
    wire signed [15:0] imu_ax, imu_ay, imu_az, imu_temp;
    wire signed [15:0] imu_gx, imu_gy, imu_gz;

    mpu6050_ctrl #(
        .CLK_HZ(50000000),
        .POWER_WAIT_CYCLES(20),
        .INIT_WAIT_CYCLES(20),
        .SAMPLE_CYCLES(5000),
        .RETRY_CYCLES(100)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_start(cmd_start), .cmd_rw(cmd_rw), .cmd_rep(cmd_rep),
        .cmd_addr(cmd_addr), .cmd_wdata(cmd_wdata), .cmd_wdata2(cmd_wdata2),
        .cmd_wlen(cmd_wlen), .cmd_rlen(cmd_rlen),
        .rd_data(rd_data), .rd_valid(rd_valid), .rd_index(rd_index),
        .cmd_done(cmd_done), .cmd_ack_err(cmd_ack_err),
        .imu_ax(imu_ax), .imu_ay(imu_ay), .imu_az(imu_az),
        .imu_temp(imu_temp), .imu_gx(imu_gx), .imu_gy(imu_gy),
        .imu_gz(imu_gz),
        .whoami_ok(whoami_ok), .data_valid(data_valid),
        .whoami(whoami), .i2c_error(i2c_error)
    );

    integer errs = 0;
    integer saw_who = 0;
    integer saw_init = 0;
    integer saw_burst = 0;
    integer i;

    task check(input cond);
        begin
            if (!cond) errs = errs + 1;
        end
    endtask

    task pulse_done;
        begin
            @(negedge clk); cmd_done = 1'b1;
            @(negedge clk); cmd_done = 1'b0;
        end
    endtask

    task respond_whoami;
        begin
            @(negedge clk);
            rd_index = 4'd0; rd_data = 8'h68; rd_valid = 1'b1;
            @(negedge clk); rd_valid = 1'b0;
            pulse_done;
        end
    endtask

    task respond_burst;
        begin
            for (i = 0; i < 14; i = i + 1) begin
                @(negedge clk);
                rd_index = i[3:0];
                rd_data  = i[7:0] + 8'h01;
                rd_valid = 1'b1;
                @(negedge clk);
                rd_valid = 1'b0;
            end
            pulse_done;
        end
    endtask

    always @(posedge clk) begin
        if (rst_n && cmd_start) begin
            check(cmd_addr == 7'h68);
            if (cmd_wdata == 8'h75 && cmd_rep && cmd_rlen == 4'd1) begin
                saw_who = saw_who + 1;
                respond_whoami;
            end else if (cmd_wdata == 8'h6B && cmd_wlen == 2'd2 &&
                         cmd_wdata2 == 8'h00 && !cmd_rep) begin
                saw_init = saw_init + 1;
                pulse_done;
            end else if (cmd_wdata == 8'h3B && cmd_rep &&
                         cmd_wlen == 2'd1 && cmd_rlen == 4'd14) begin
                saw_burst = saw_burst + 1;
                respond_burst;
            end else begin
                $display("FAIL: unexpected I2C request");
                errs = errs + 1;
                pulse_done;
            end
        end
    end

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        wait (data_valid === 1'b1);
        #1;
        check(saw_who == 1);
        check(saw_init == 1);
        check(saw_burst == 1);
        check(whoami_ok == 1'b1);
        check(whoami == 8'h68);
        check(!i2c_error);
        check(imu_ax == 16'h0102);
        check(imu_ay == 16'h0304);
        check(imu_az == 16'h0506);
        check(imu_temp == 16'h0708);
        check(imu_gx == 16'h090A);
        check(imu_gy == 16'h0B0C);
        check(imu_gz == 16'h0D0E);

        if (errs == 0) $display("PASS: tb_mpu6050_ctrl");
        else           $display("FAIL: tb_mpu6050_ctrl (%0d checks)", errs);
        $finish;
    end

    initial begin
        #1000000;
        $display("FAIL: tb_mpu6050_ctrl timeout");
        $finish;
    end
endmodule
