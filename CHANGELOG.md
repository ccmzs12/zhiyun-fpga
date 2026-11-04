# 变更记录

## 2026-11-03，User Code 0x3696

基线：`zhiyun_ultrasonic_oled_8note_7D3A_20261101(2).zip`

### 键盘

- `L1` 从 `T21` 改到 `W17`。
- 其余键盘引脚保持不变。

### OLED 音高轨迹

- `pitch_scope.note_idx` 从 5 位改为完整 6 位。
- `pitch_trend.note_idx` 从 5 位改为完整 6 位。
- 八度判断从错误的 `note_idx[4:3]` 改为 `note_idx[5:4]`。
- 键号从 `note_idx[2:0]` 改为 `note_idx[3:0]`。
- 修正高八度被误当成低八度的问题。
- 按实际屏幕方向调整纵向映射：高音在顶部，低音在底部。
- 弯音方向同步调整：音高升高，轨迹向上。

### 验证

- `tb_pitch_trend` 通过。
- `tb_pitch_scope` 通过。
- Gowin 构建通过。
- 外部 Flash 写入和校验通过。
- SRAM 回读 User Code 为 `0x00003696`。

### 未改动

- 板载 I2S 音频通路。
- MPU6050 弯音与颤音映射。
- 超声波距离到音量映射。
- K9/K10 音色和 K11/K12 八度功能。

### 未包含

- 未启用 PCM5102A 外接音频映射。

