`timescale 1ns/1ps

module tb_i2c_multi;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;

    reg        cmd_start = 1'b0;
    reg        cmd_rw = 1'b0;
    reg        cmd_rep = 1'b0;
    reg [6:0]  cmd_addr = 7'h68;
    reg [7:0]  cmd_wdata = 8'd0;
    reg [7:0]  cmd_wdata2 = 8'd0;
    reg [1:0]  cmd_wlen = 2'd1;
    reg [3:0]  cmd_rlen = 4'd1;

    wire [7:0] cmd_rdata;
    wire [7:0] rd_data;
    wire rd_valid;
    wire [3:0] rd_index;
    wire cmd_done;
    wire cmd_ack_err;
    wire scl_oe, sda_oe;
    wire scl, sda;
    wire sda_in;
    reg  slave_oe = 1'b0;

    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign sda = slave_oe ? 1'b0 : 1'bz;
    assign (weak1, weak0) scl = 1'b1;
    assign (weak1, weak0) sda = 1'b1;
    assign sda_in = sda;

    i2c_master_multi #(.CLK_HZ(50000000), .SCL_HZ(100000)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_start(cmd_start), .cmd_rw(cmd_rw), .cmd_rep(cmd_rep),
        .cmd_addr(cmd_addr), .cmd_wdata(cmd_wdata), .cmd_wdata2(cmd_wdata2),
        .cmd_wlen(cmd_wlen), .cmd_rlen(cmd_rlen),
        .cmd_rdata(cmd_rdata), .rd_data(rd_data), .rd_valid(rd_valid),
        .rd_index(rd_index), .cmd_done(cmd_done), .cmd_ack_err(cmd_ack_err),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_in(sda_in), .dbg_rd_sample()
    );

    localparam [1:0] RX = 2'd0;
    localparam [1:0] TX = 2'd1;
    reg [1:0]  sl_st;
    reg [3:0]  bn;
    reg [7:0]  sr;
    reg        have_addr;
    reg        matched;
    reg [7:0]  txd;
    reg [7:0]  txbytes [0:3];
    reg [7:0]  wrbytes [0:7];
    integer    wrcnt;
    reg        scl_d, sda_d;
    integer    rxcnt;
    reg [7:0]  rxbytes [0:7];

    initial begin
        sl_st = RX; bn = 0; sr = 0; have_addr = 0; matched = 0;
        txd = 0; wrcnt = 0; rxcnt = 0; scl_d = 1; sda_d = 1;
        txbytes[0] = 8'h11; txbytes[1] = 8'h22;
        txbytes[2] = 8'h33; txbytes[3] = 8'h44;
    end

    always @(scl or sda) begin
        if ((scl === 1'b1) && (scl_d !== 1'b1)) begin
            if (sl_st == RX) begin
                if (bn < 4'd8) sr = {sr[6:0], sda};
                bn = bn + 4'd1;
            end else begin
                bn = bn + 4'd1;
            end
        end else if ((scl === 1'b0) && (scl_d !== 1'b0)) begin
            if (sl_st == RX) begin
                if (bn == 4'd8) slave_oe = 1'b1;
                else if (bn == 4'd9) begin
                    slave_oe = 1'b0;
                    bn = 0;
                    if (!have_addr) begin
                        have_addr = 1'b1;
                        matched = (sr[7:1] == 7'h68);
                        if (matched && sr[0]) begin
                            sl_st = TX;
                            txd = txbytes[0];
                            slave_oe = (txd[7] == 1'b0);
                        end
                    end else if (matched) begin
                        wrbytes[wrcnt] = sr;
                        rxcnt = wrcnt;
                        wrcnt = wrcnt + 1;
                    end
                    sr = 8'd0;
                end
            end else begin
                if (bn >= 4'd1 && bn <= 4'd7) slave_oe = (txd[7-bn] == 1'b0);
                else if (bn == 4'd8) slave_oe = 1'b0;
                else if (bn == 4'd9) begin
                    slave_oe = 1'b0;
                    bn = 0;
                    if (sda === 1'b0 && rxcnt < 3) begin
                        rxcnt = rxcnt + 1;
                        txd = txbytes[rxcnt];
                        slave_oe = (txd[7] == 1'b0);
                    end else begin
                        sl_st = RX;
                    end
                end
            end
        end else if ((sda === 1'b1) && (sda_d !== 1'b1) && (scl === 1'b1)) begin
            sl_st = RX; bn = 0; sr = 0; have_addr = 0; matched = 0;
            slave_oe = 0; rxcnt = 0;
        end else if ((sda === 1'b0) && (sda_d !== 1'b0) && (scl === 1'b1)) begin
            sl_st = RX; bn = 0; sr = 0; have_addr = 0; matched = 0;
            slave_oe = 0; rxcnt = 0;
        end
        scl_d = scl;
        sda_d = sda;
    end

    integer errs = 0;
    reg [7:0] got [0:3];
    integer got_cnt;
    integer i;

    task check(input cond);
        begin
            if (!cond) errs = errs + 1;
        end
    endtask

    task launch;
        begin
            @(negedge clk);
            cmd_start = 1'b1;
            @(negedge clk);
            cmd_start = 1'b0;
            @(posedge clk);
            while (!cmd_done) @(posedge clk);
            @(negedge clk);
        end
    endtask

    always @(posedge clk) begin
        if (rd_valid && got_cnt < 4) begin
            got[got_cnt] = rd_data;
            got_cnt = got_cnt + 1;
        end
    end

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // Register write: 0x6B <- 0x00 in one transaction.
        wrcnt = 0;
        cmd_rw = 1'b0; cmd_rep = 1'b0; cmd_addr = 7'h68;
        cmd_wdata = 8'h6B; cmd_wdata2 = 8'h00;
        cmd_wlen = 2'd2; cmd_rlen = 4'd1;
        launch;
        check(!cmd_ack_err);
        check(wrcnt == 2);
        check(wrbytes[0] == 8'h6B);
        check(wrbytes[1] == 8'h00);

        // Pointer write 0x3B, repeated START, then four read bytes.
        got_cnt = 0;
        cmd_rw = 1'b0; cmd_rep = 1'b1; cmd_addr = 7'h68;
        cmd_wdata = 8'h3B; cmd_wdata2 = 8'd0;
        cmd_wlen = 2'd1; cmd_rlen = 4'd4;
        launch;
        check(!cmd_ack_err);
        check(got_cnt == 4);
        check(got[0] == 8'h11);
        check(got[1] == 8'h22);
        check(got[2] == 8'h33);
        check(got[3] == 8'h44);

        if (errs == 0) $display("PASS: tb_i2c_multi");
        else           $display("FAIL: tb_i2c_multi (%0d checks)", errs);
        $finish;
    end

    initial begin
        #3000000;
        $display("FAIL: tb_i2c_multi timeout");
        $finish;
    end
endmodule
