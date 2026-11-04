# 指韵空灵 FPGA 最终交付包

## 版本

- 目标板：Tang Mega 60K，GW5AT-60B
- Top module：`top`
- User Code：`0x00003696`
- 最终位流：`bitstream/zhiyun_L1W17_pitch6_0x3696.fs`
- 构建日期：2026-11-03
- 键盘列 `L1`：`W17`

## 已实现功能

- 8 声部 FM 复音引擎，K1～K8 可同时发声。
- K1～K8 音高由低到高：C4、D4、E4、F4、G4、A4、B4、C5。
- K9、K10 选择两组 FM 音色。
- K11、K12 分别降低、升高八度。
- K13 全部释音。
- MPU6050 倾角映射弯音，晃动映射颤音深度。
- HY-SRF05 / HC-SR04 超声波测距映射 8 档音量。
- 128x64 SSD1306/SH1106 OLED 显示音高滚动轨迹。
- 音高轨迹每 40 ms 采样一次，128 列约显示 5.1 秒。

## OLED 音高显示方向

本版按实际屏幕方向修正为：

| 音区 | 屏幕位置 | 逻辑纵坐标 |
|---|---|---|
| 高声部 | 最上方 | 45～62 |
| 中声部 | 中间 | 25～42 |
| 低声部 | 最下方 | 5～22 |

- K1 到 K8 音调上升时，轨迹持续向上移动。
- 弯音升高时轨迹向上，弯音降低时轨迹向下。
- 三个音区由连续坐标映射形成，没有绘制分隔线。
- 音高数据采用完整 6 位通路：`{octave[1:0], key_index[3:0]}`。

## 键盘接线

| 键盘丝印 | FPGA 引脚 |
|---|---|
| R1 | U21 |
| R2 | P16 |
| R3 | R17 |
| R4 | R18 |
| L1 | W17 |
| L2 | Y22 |
| L3 | P19 |
| L4 | AB22 |

K1～K8 为音符键；K9～K13 为功能键。

## OLED 接线

OLED 使用 4 线 SPI 模式，模块背面需安装 `R3`、`R4` 选择 4SPI。

| OLED | FPGA/电源 |
|---|---|
| VCC | 3.3 V |
| GND | 公共 GND |
| D0 / SCL | C17 |
| D1 / MOSI | C18 |
| DC | C19 |
| RES | B18 |
| CS | A18 |

`C17/C18/C19/B18/A18` 属于板载 SDRAM 地址组。本工程不使用 SDRAM；
其他工程不要同时占用这些引脚。

## 超声波接线

| HY-SRF05 | 接线 |
|---|---|
| VCC | 5 V |
| GND | 公共 GND |
| Trig | N15 |
| Echo | P17，必须经 1.2k / 2.2k 分压 |
| OUT | 悬空 |

Echo 分压后的电压约为 3.24 V，不能把 5 V Echo 直接接 FPGA。

## 其他引脚

| 信号 | FPGA 引脚 |
|---|---|
| sys_clk | V22 |
| sys_rst_n | AA13 |
| Key_in[0] / Key_in[1] | AB13 / Y12 |
| HP_BCK / HP_WS / HP_DIN | Y17 / AB17 / AA16 |
| PA_EN | AB16 |
| i2c_scl / i2c_sda | AA20 / AA19 |
| UART TX | U15 |
| event_led | T18 |

本版的音频输出使用开发板板载 I2S 音频通路。
`R19 / Y21 / AA21` 的 PCM5102A 外接映射未启用。

## 构建

在本包根目录运行：

```powershell
gw_sh build.tcl
```

输出：

```text
prj/impl/pnr/zhiyun.fs
prj/impl/pnr/zhiyun.bin
```

重新构建会生成新的 User Code，因此以包内预构建位流和校验值为准。

## 烧录

外部 Flash，掉电保留：

```powershell
programmer_cli --device GW5AT-60B --operation_index 37 `
  --frequency 0.3MHz `
  --fsFile bitstream\zhiyun_L1W17_pitch6_0x3696.fs
```

SRAM，立即生效：

```powershell
programmer_cli --device GW5AT-60B --operation_index 2 `
  --fsFile bitstream\zhiyun_L1W17_pitch6_0x3696.fs
```

## 验证结果

- ModelSim `tb_pitch_trend`：通过。
- ModelSim `tb_pitch_scope`：通过。
- Gowin 综合、布局布线和位流生成：通过。
- 外部 Flash `Erase, Program, Verify`：通过。
- SRAM 回读 User Code：`0x00003696`。
- 逻辑资源：5626 / 59904，约 10%。
- 寄存器：3214 / 60780，约 6%。
- BSRAM：8 / 118。
- DSP：7 / 118。

详细记录见 `logs/` 和 `reports/`。

## 已知限制

- 无二极管 4x4 矩阵键盘同时按 3 个以上键时仍可能出现鬼影。
- 超声波模块必须使用 5 V 供电，Echo 必须分压。
- OLED 使用板载 SDRAM 地址组引脚，不能在同时使用 SDRAM 的工程中复用。

