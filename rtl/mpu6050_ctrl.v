`timescale 1ns/1ps

// MPU6050 bring-up and 14-byte burst reader.
module mpu6050_ctrl #(
    parameter integer CLK_HZ = 50000000,
    parameter integer POWER_WAIT_CYCLES = 2500000, // 50 ms
    parameter integer INIT_WAIT_CYCLES  = 2500000, // 50 ms
    parameter integer SAMPLE_CYCLES     = 250000,  // 5 ms
    parameter integer RETRY_CYCLES      = 5000000  // 100 ms
) (
    input  wire        clk,
    input  wire        rst_n,

    output reg         cmd_start,
    output reg         cmd_rw,
    output reg         cmd_rep,
    output reg  [6:0]  cmd_addr,
    output reg  [7:0]  cmd_wdata,
    output reg  [7:0]  cmd_wdata2,
    output reg  [1:0]  cmd_wlen,
    output reg  [3:0]  cmd_rlen,

    input  wire [7:0]  rd_data,
    input  wire        rd_valid,
    input  wire [3:0]  rd_index,
    input  wire        cmd_done,
    input  wire        cmd_ack_err,

    output wire signed [15:0] imu_ax,
    output wire signed [15:0] imu_ay,
    output wire signed [15:0] imu_az,
    output wire signed [15:0] imu_temp,
    output wire signed [15:0] imu_gx,
    output wire signed [15:0] imu_gy,
    output wire signed [15:0] imu_gz,

    output reg         whoami_ok,
    output reg         data_valid,
    output reg  [7:0]  whoami,
    output reg         i2c_error
);
    localparam [3:0] S_POWER_WAIT = 4'd0;
    localparam [3:0] S_WHO_REQ    = 4'd1;
    localparam [3:0] S_WHO_SET    = 4'd2;
    localparam [3:0] S_WHO_WAIT   = 4'd3;
    localparam [3:0] S_INIT_REQ   = 4'd4;
    localparam [3:0] S_INIT_SET   = 4'd5;
    localparam [3:0] S_INIT_WAIT  = 4'd6;
    localparam [3:0] S_START_WAIT = 4'd7;
    localparam [3:0] S_BURST_REQ  = 4'd8;
    localparam [3:0] S_BURST_SET  = 4'd9;
    localparam [3:0] S_BURST_WAIT = 4'd10;
    localparam [3:0] S_SAMPLE_WAIT= 4'd11;
    localparam [3:0] S_RETRY_WAIT = 4'd12;

    reg [3:0]  st;
    reg [23:0] timer;
    reg [7:0]  raw [0:13];
    integer i;

    wire signed [15:0] ax_raw = {raw[0], raw[1]};
    wire signed [15:0] ay_raw = {raw[2], raw[3]};
    wire signed [15:0] az_raw = {raw[4], raw[5]};
    wire signed [15:0] temp_raw = {raw[6], raw[7]};
    wire signed [15:0] gx_raw = {raw[8], raw[9]};
    wire signed [15:0] gy_raw = {raw[10], raw[11]};
    wire signed [15:0] gz_raw = {raw[12], raw[13]};
    wire burst_all_ff = (raw[0]  == 8'hFF) && (raw[1]  == 8'hFF) &&
                        (raw[2]  == 8'hFF) && (raw[3]  == 8'hFF) &&
                        (raw[4]  == 8'hFF) && (raw[5]  == 8'hFF) &&
                        (raw[6]  == 8'hFF) && (raw[7]  == 8'hFF) &&
                        (raw[8]  == 8'hFF) && (raw[9]  == 8'hFF) &&
                        (raw[10] == 8'hFF) && (raw[11] == 8'hFF) &&
                        (raw[12] == 8'hFF) && (raw[13] == 8'hFF);

    assign imu_ax = ax_raw;
    assign imu_ay = ay_raw;
    assign imu_az = az_raw;
    assign imu_temp = temp_raw;
    assign imu_gx = gx_raw;
    assign imu_gy = gy_raw;
    assign imu_gz = gz_raw;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_POWER_WAIT;
            timer <= 24'd0;
            cmd_start <= 1'b0;
            cmd_rw <= 1'b0;
            cmd_rep <= 1'b0;
            cmd_addr <= 7'h68;
            cmd_wdata <= 8'd0;
            cmd_wdata2 <= 8'd0;
            cmd_wlen <= 2'd1;
            cmd_rlen <= 4'd1;
            whoami_ok <= 1'b0;
            data_valid <= 1'b0;
            whoami <= 8'd0;
            i2c_error <= 1'b0;
            for (i = 0; i < 14; i = i + 1)
                raw[i] <= 8'd0;
        end else begin
            cmd_start <= 1'b0;
            data_valid <= 1'b0;

            if (rd_valid) begin
                if (st == S_WHO_WAIT)
                    whoami <= rd_data;
                if (st == S_BURST_WAIT && rd_index < 4'd14)
                    raw[rd_index] <= rd_data;
            end

            case (st)
            S_POWER_WAIT: begin
                if (timer == POWER_WAIT_CYCLES - 1) begin
                    timer <= 24'd0;
                    st <= S_WHO_REQ;
                end else timer <= timer + 24'd1;
            end

            S_WHO_REQ: begin
                cmd_rw <= 1'b0;
                cmd_rep <= 1'b1;
                cmd_addr <= 7'h68;
                cmd_wdata <= 8'h75;
                cmd_wdata2 <= 8'd0;
                cmd_wlen <= 2'd1;
                cmd_rlen <= 4'd1;
                st <= S_WHO_SET;
            end
            S_WHO_SET: begin
                cmd_start <= 1'b1;
                st <= S_WHO_WAIT;
            end
            S_WHO_WAIT: begin
                if (cmd_done) begin
                    if (cmd_ack_err || whoami != 8'h68) begin
                        whoami_ok <= 1'b0;
                        i2c_error <= 1'b1;
                        timer <= 24'd0;
                        st <= S_RETRY_WAIT;
                    end else begin
                        whoami_ok <= 1'b1;
                        i2c_error <= 1'b0;
                        st <= S_INIT_REQ;
                    end
                end
            end

            S_INIT_REQ: begin
                cmd_rw <= 1'b0;
                cmd_rep <= 1'b0;
                cmd_addr <= 7'h68;
                cmd_wdata <= 8'h6B;
                cmd_wdata2 <= 8'h00;
                cmd_wlen <= 2'd2;
                cmd_rlen <= 4'd1;
                st <= S_INIT_SET;
            end
            S_INIT_SET: begin
                cmd_start <= 1'b1;
                st <= S_INIT_WAIT;
            end
            S_INIT_WAIT: begin
                if (cmd_done) begin
                    timer <= 24'd0;
                    if (cmd_ack_err) begin
                        i2c_error <= 1'b1;
                        st <= S_RETRY_WAIT;
                    end else begin
                        i2c_error <= 1'b0;
                        st <= S_START_WAIT;
                    end
                end
            end

            S_START_WAIT: begin
                if (timer == INIT_WAIT_CYCLES - 1) begin
                    timer <= 24'd0;
                    st <= S_BURST_REQ;
                end else timer <= timer + 24'd1;
            end

            S_BURST_REQ: begin
                cmd_rw <= 1'b0;
                cmd_rep <= 1'b1;
                cmd_addr <= 7'h68;
                cmd_wdata <= 8'h3B;
                cmd_wdata2 <= 8'd0;
                cmd_wlen <= 2'd1;
                cmd_rlen <= 4'd14;
                st <= S_BURST_SET;
            end
            S_BURST_SET: begin
                cmd_start <= 1'b1;
                st <= S_BURST_WAIT;
            end
            S_BURST_WAIT: begin
                if (cmd_done) begin
                    timer <= 24'd0;
                    if (cmd_ack_err || burst_all_ff) begin
                        whoami_ok <= 1'b0;
                        i2c_error <= 1'b1;
                        st <= S_RETRY_WAIT;
                    end else begin
                        data_valid <= 1'b1;
                        i2c_error <= 1'b0;
                        st <= S_SAMPLE_WAIT;
                    end
                end
            end

            S_SAMPLE_WAIT: begin
                if (timer == SAMPLE_CYCLES - 1) begin
                    timer <= 24'd0;
                    st <= S_BURST_REQ;
                end else timer <= timer + 24'd1;
            end

            S_RETRY_WAIT: begin
                if (timer == RETRY_CYCLES - 1) begin
                    timer <= 24'd0;
                    st <= S_WHO_REQ;
                end else timer <= timer + 24'd1;
            end

            default: st <= S_POWER_WAIT;
            endcase
        end
    end
endmodule
