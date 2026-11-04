`timescale 1ns/1ps

// Byte-level I2C master with 1/2-byte writes and repeated-start burst reads.
module i2c_master_multi #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SCL_HZ = 100000
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        cmd_start,
    input  wire        cmd_rw,
    input  wire        cmd_rep,
    input  wire [6:0]  cmd_addr,
    input  wire [7:0]  cmd_wdata,
    input  wire [7:0]  cmd_wdata2,
    input  wire [1:0]  cmd_wlen,
    input  wire [3:0]  cmd_rlen,

    output reg  [7:0]  cmd_rdata,
    output reg  [7:0]  rd_data,
    output reg         rd_valid,
    output reg  [3:0]  rd_index,
    output reg         cmd_done,
    output reg         cmd_ack_err,

    output wire        scl_oe,
    output wire        sda_oe,
    input  wire        sda_in,
    output wire        dbg_rd_sample
);
    localparam integer QUARTER_RAW = CLK_HZ / (SCL_HZ * 4);
    localparam integer QUARTER     = (QUARTER_RAW < 1) ? 1 : QUARTER_RAW;

    localparam [3:0] S_IDLE     = 4'd0;
    localparam [3:0] S_START    = 4'd1;
    localparam [3:0] S_ADDR     = 4'd2;
    localparam [3:0] S_ADDR_AK  = 4'd3;
    localparam [3:0] S_WDATA    = 4'd4;
    localparam [3:0] S_WDATA_AK = 4'd5;
    localparam [3:0] S_RDATA    = 4'd6;
    localparam [3:0] S_RDATA_AK = 4'd7;
    localparam [3:0] S_STOP     = 4'd8;
    localparam [3:0] S_RSTART   = 4'd9;

    reg [3:0]  st;
    reg [1:0]  ph;
    reg [15:0] qc;
    reg [2:0]  bidx;
    reg [7:0]  sr;
    reg        rw_r;
    reg        rep_r;
    reg [7:0]  wdata_r;
    reg [7:0]  wdata2_r;
    reg [1:0]  wlen_r;
    reg [1:0]  wleft;
    reg [3:0]  rlen_r;
    reg [3:0]  rleft;
    reg        sda_low;

    wire bit_end    = (st != S_IDLE) && (ph == 2'd3) && (qc == QUARTER - 1);
    wire sample_now = (st != S_IDLE) && (ph == 2'd2) && (qc == QUARTER - 1);

    assign dbg_rd_sample = sample_now && (st == S_RDATA);
    assign scl_oe = (st != S_IDLE) && (ph <= 2'd1);
    assign sda_oe = sda_low;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            ph          <= 2'd0;
            qc          <= 16'd0;
            bidx        <= 3'd0;
            sr          <= 8'd0;
            rw_r        <= 1'b0;
            rep_r       <= 1'b0;
            wdata_r     <= 8'd0;
            wdata2_r    <= 8'd0;
            wlen_r      <= 2'd1;
            wleft       <= 2'd0;
            rlen_r      <= 4'd1;
            rleft       <= 4'd0;
            sda_low     <= 1'b0;
            cmd_done    <= 1'b0;
            cmd_ack_err <= 1'b0;
            cmd_rdata   <= 8'd0;
            rd_data     <= 8'd0;
            rd_valid    <= 1'b0;
            rd_index    <= 4'd0;
        end else begin
            cmd_done  <= 1'b0;
            rd_valid  <= 1'b0;

            if (st == S_IDLE) begin
                ph <= 2'd0;
                qc <= 16'd0;
            end else if (qc == QUARTER - 1) begin
                qc <= 16'd0;
                ph <= ph + 2'd1;
            end else begin
                qc <= qc + 16'd1;
            end

            case (st)
            S_IDLE: begin
                sda_low <= 1'b0;
                if (cmd_start) begin
                    sr          <= {cmd_addr, cmd_rw};
                    rw_r        <= cmd_rw;
                    rep_r       <= cmd_rep;
                    wdata_r     <= cmd_wdata;
                    wdata2_r    <= cmd_wdata2;
                    wlen_r      <= (cmd_wlen == 2'd0) ? 2'd1 :
                                   (cmd_wlen > 2'd2) ? 2'd2 : cmd_wlen;
                    wleft       <= (cmd_wlen == 2'd0) ? 2'd1 :
                                   (cmd_wlen > 2'd2) ? 2'd2 : cmd_wlen;
                    rlen_r      <= (cmd_rlen == 4'd0) ? 4'd1 : cmd_rlen;
                    rleft       <= (cmd_rlen == 4'd0) ? 4'd1 : cmd_rlen;
                    rd_index    <= 4'd0;
                    bidx        <= 3'd0;
                    cmd_rdata   <= 8'd0;
                    cmd_ack_err <= 1'b0;
                    if (!sda_in)
                        cmd_ack_err <= 1'b1;
                    st <= S_START;
                end
            end

            S_START: begin
                if (ph == 2'd3 && qc == 16'd0)
                    sda_low <= 1'b1;
                if (bit_end) begin
                    bidx <= 3'd0;
                    st   <= S_ADDR;
                end
            end

            S_ADDR: begin
                if (ph == 2'd1 && qc == 16'd0)
                    sda_low <= ~sr[7];
                if (bit_end) begin
                    sr <= {sr[6:0], 1'b0};
                    if (bidx == 3'd7) begin
                        bidx <= 3'd0;
                        st   <= S_ADDR_AK;
                    end else begin
                        bidx <= bidx + 3'd1;
                    end
                end
            end

            S_ADDR_AK: begin
                if (ph == 2'd1 && qc == 16'd0)
                    sda_low <= 1'b0;
                if (sample_now && sda_in)
                    cmd_ack_err <= 1'b1;
                if (bit_end) begin
                    if (rw_r) begin
                        sr    <= 8'd0;
                        rleft <= rlen_r;
                        st    <= S_RDATA;
                    end else begin
                        sr    <= wdata_r;
                        wleft <= wlen_r;
                        st    <= S_WDATA;
                    end
                end
            end

            S_WDATA: begin
                if (ph == 2'd1 && qc == 16'd0)
                    sda_low <= ~sr[7];
                if (bit_end) begin
                    sr <= {sr[6:0], 1'b0};
                    if (bidx == 3'd7) begin
                        bidx <= 3'd0;
                        st   <= S_WDATA_AK;
                    end else begin
                        bidx <= bidx + 3'd1;
                    end
                end
            end

            S_WDATA_AK: begin
                if (ph == 2'd1 && qc == 16'd0)
                    sda_low <= 1'b0;
                if (sample_now && sda_in)
                    cmd_ack_err <= 1'b1;
                if (bit_end) begin
                    if (wleft > 2'd1) begin
                        wleft <= wleft - 2'd1;
                        sr    <= wdata2_r;
                        st    <= S_WDATA;
                    end else if (rep_r) begin
                        sr    <= {cmd_addr, 1'b1};
                        rw_r  <= 1'b1;
                        rleft <= rlen_r;
                        st    <= S_RSTART;
                    end else begin
                        st <= S_STOP;
                    end
                end
            end

            S_RDATA: begin
                if (ph == 2'd1 && qc == 16'd0)
                    sda_low <= 1'b0;
                if (sample_now)
                    sr <= {sr[6:0], sda_in};
                if (bit_end) begin
                    if (bidx == 3'd7) begin
                        rd_data   <= sr;
                        cmd_rdata <= sr;
                        rd_valid  <= 1'b1;
                        bidx      <= 3'd0;
                        st        <= S_RDATA_AK;
                    end else begin
                        bidx <= bidx + 3'd1;
                    end
                end
            end

            S_RDATA_AK: begin
                if (ph == 2'd1 && qc == 16'd0)
                    sda_low <= (rleft > 4'd1);
                if (bit_end) begin
                    if (rleft > 4'd1) begin
                        rleft    <= rleft - 4'd1;
                        rd_index <= rd_index + 4'd1;
                        st       <= S_RDATA;
                    end else begin
                        st <= S_STOP;
                    end
                end
            end

            S_RSTART: begin
                if (ph == 2'd3 && qc == 16'd0)
                    sda_low <= 1'b1;
                if (bit_end) begin
                    bidx <= 3'd0;
                    st   <= S_ADDR;
                end
            end

            S_STOP: begin
                if (ph == 2'd1 && qc == 16'd0)
                    sda_low <= 1'b1;
                if (ph == 2'd3 && qc == 16'd0)
                    sda_low <= 1'b0;
                if (bit_end) begin
                    st       <= S_IDLE;
                    cmd_done <= 1'b1;
                end
            end

            default: begin
                st      <= S_IDLE;
                sda_low <= 1'b0;
            end
            endcase
        end
    end
endmodule
