//=============================================================================
// adsr.v —— ADSR 包络（单声部参考实现，A 负责）
//-----------------------------------------------------------------------------
// 状态 : IDLE → ATTACK → DECAY → SUSTAIN → RELEASE → IDLE
//
// | 状态    | 进入条件      | 包络变化        | 速率（每采样）                   |
// |---------|---------------|-----------------|----------------------------------|
// | ATTACK  | FM 的 note_on | 线性升到 32767  | +134（≈5 ms）                    |
// | DECAY   | 到达 32767    | 线性降到 19660  | -13（≈200 ms）                   |
// | SUSTAIN | 到达 19660    | 保持            | —                                |
// | RELEASE | note_off      | 线性降到 0      | FM -4（≈101ms）/ KS -8（≈84ms）  |
//
// 说明 : KS（拨弦）的 note_on 直接 env=32767 进 SUSTAIN —— 音头就是拨弦激励
//        本身，不需要淡入；松键后按 80 ms 释放。
//        每采样更新一次，包络用 16 位（0..32767），足够避免速率过小时被截断。
//        相减前判下溢，禁止回绕。
//
// 用法 : 输出音频 = (audio * env) >>> 15；若还要乘力度，见
//        python/rtl_model.py:apply_env_vel。8 声部正式版在 voice_engine 内
//        按时分复用内联同一套规则（每声部一份状态）。
// 负责人: A
//=============================================================================
`default_nettype none

module adsr (
  input  wire        clk,
  input  wire        rst_n,
  input  wire        tick,          // 1 clk 使能 = 前进一个采样
  input  wire        note_on,       // 1 clk 脉冲
  input  wire        note_off,      // 1 clk 脉冲
  input  wire        timbre,        // 0 = KS，1 = FM
  output reg  [15:0] env            // 0..32767
);

  localparam E_IDLE    = 3'd0,
             E_ATTACK  = 3'd1,
             E_DECAY   = 3'd2,
             E_SUSTAIN = 3'd3,
             E_RELEASE = 3'd4;

  localparam [15:0] ATK_RATE   = 16'd134;
  localparam [15:0] DEC_RATE   = 16'd13;
  localparam [15:0] SUSTAIN_LV = 16'd19660;
  // 【第 5 天调参】验收标准要求「松键后约 100 ms 内平顺消失」，所以把释音放快：
  //   FM 从延音电平 19660 以 4/采样 降到 0 → 4915 采样 ≈ 101 ms
  //   KS 从满幅    32767 以 8/采样 降到 0 → 4096 采样 ≈  84 ms
  reg [2:0] st;

  // Linear releases give a predictable slope.  FM closes in about 50 ms;
  // the 0x0000031F K10 voice uses a 168 ms linear release.
  localparam [15:0] REL_FM = 16'd8;
  localparam [15:0] REL_KS = 16'd4;
  wire [15:0] rel_step = timbre ? REL_FM : REL_KS;
  wire [16:0] atk_sum  = {1'b0, env} + 17'd134;              // 加宽 1 位防溢出

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      env <= 16'd0;
      st  <= E_IDLE;
    end else begin
      if (note_on) begin
        if (!timbre) begin           // KS：音头即满幅
          env <= 16'd32767;
          st  <= E_SUSTAIN;
        end else begin               // FM：完整 ADSR
          env <= 16'd0;
          st  <= E_ATTACK;
        end
      end else if (note_off) begin
        if (st != E_IDLE) st <= E_RELEASE;
      end else if (tick) begin
        case (st)
          E_ATTACK: begin
            if (atk_sum >= 17'd32767) begin
              env <= 16'd32767;
              st  <= E_DECAY;
            end else begin
              env <= atk_sum[15:0];
            end
          end

          E_DECAY: begin
            if (env <= SUSTAIN_LV + DEC_RATE) begin
              env <= SUSTAIN_LV;
              st  <= E_SUSTAIN;
            end else begin
              env <= env - DEC_RATE;
            end
          end

          E_SUSTAIN: begin
            env <= env;
          end

          E_RELEASE: begin
            if (env <= rel_step) begin
              env <= 16'd0;
              st  <= E_IDLE;
            end else begin
              env <= env - rel_step;
            end
          end

          default: begin
            env <= 16'd0;
            st  <= E_IDLE;
          end
        endcase
      end
    end
  end

endmodule

`default_nettype wire
