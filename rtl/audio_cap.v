`timescale 1ns/1ps
//=============================================================================
// audio_cap —— 抓取混音器真实输出，用串口发回电脑（诊断用）
//   按下 trig_n（低有效，接板载按键）→ 连续记录 8192 个采样
//   （48.828 kHz 下约 168 ms）到 BRAM → 然后以 115200 8N1 文本发出：
//       CAP\n
//       8192 行，每行 4 位十六进制（16 位补码）\n
//       END\n
//   发送约 3.6 s。active=1 期间（抓取+发送）由本模块占用串口。
//=============================================================================
module audio_cap #(
    parameter integer AW = 13
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               frame_tick,
    input  wire signed [15:0] sample,
    input  wire               trig_n,
    output wire               tx,
    output wire               active
);
    localparam [2:0] S_IDLE = 3'd0, S_CAP = 3'd1, S_HDR = 3'd2,
                     S_DATA = 3'd3, S_END = 3'd4;

    reg [2:0]    st;
    reg [2:0]    sync;
    reg [AW-1:0] wa, ra;
    reg [2:0]    ci;
    reg [7:0]    u_data;
    reg          u_load;
    wire         u_busy;

    // ---- 采样缓存（简单双口 RAM，寄存器读）----
    reg [15:0] mem [0:(1<<AW)-1];
    reg [15:0] rd_q;
    always @(posedge clk) begin
        if (st == S_CAP && frame_tick)
            mem[wa] <= sample;
        rd_q <= mem[ra];
    end

    // ---- 十六进制字符 ----
    reg [3:0] nib;
    always @(*) begin
        case (ci)
            3'd0:    nib = rd_q[15:12];
            3'd1:    nib = rd_q[11:8];
            3'd2:    nib = rd_q[7:4];
            default: nib = rd_q[3:0];
        endcase
    end
    wire [7:0] hx = (nib < 4'd10) ? {4'h3, nib} : ({4'd0, nib} + 8'h37);

    wire press = sync[2] & ~sync[1];       // 按键下降沿
    assign active = (st != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; sync <= 3'b111;
            wa <= {AW{1'b0}}; ra <= {AW{1'b0}};
            ci <= 3'd0; u_load <= 1'b0; u_data <= 8'd0;
        end else begin
            sync   <= {sync[1:0], trig_n};
            u_load <= 1'b0;
            case (st)
                S_IDLE: if (press) begin
                    wa <= {AW{1'b0}};
                    st <= S_CAP;
                end
                S_CAP: if (frame_tick) begin
                    if (wa == {AW{1'b1}}) begin
                        st <= S_HDR; ci <= 3'd0; ra <= {AW{1'b0}};
                    end
                    wa <= wa + 1'b1;
                end
                S_HDR: if (!u_busy && !u_load) begin
                    u_load <= 1'b1;
                    u_data <= (ci == 3'd0) ? 8'h43 :      // C
                              (ci == 3'd1) ? 8'h41 :      // A
                              (ci == 3'd2) ? 8'h50 :      // P
                                             8'h0A;
                    if (ci == 3'd3) begin ci <= 3'd0; st <= S_DATA; end
                    else ci <= ci + 3'd1;
                end
                S_DATA: if (!u_busy && !u_load) begin
                    u_load <= 1'b1;
                    u_data <= (ci == 3'd4) ? 8'h0A : hx;
                    if (ci == 3'd4) begin
                        ci <= 3'd0;
                        if (ra == {AW{1'b1}}) st <= S_END;
                        ra <= ra + 1'b1;
                    end else ci <= ci + 3'd1;
                end
                S_END: if (!u_busy && !u_load) begin
                    u_load <= 1'b1;
                    u_data <= (ci == 3'd0) ? 8'h45 :      // E
                              (ci == 3'd1) ? 8'h4E :      // N
                              (ci == 3'd2) ? 8'h44 :      // D
                                             8'h0A;
                    if (ci == 3'd3) begin ci <= 3'd0; st <= S_IDLE; end
                    else ci <= ci + 3'd1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    uart_tx u_tx (
        .clk(clk), .rst_n(rst_n),
        .load(u_load), .data(u_data), .tx(tx), .busy(u_busy)
    );
endmodule
