# F28388Test DAB Control Project

本工程是基于 TI F28388D / C2000Ware 的 DAB 双有源桥控制工程。当前代码重点包括：

- ePWM 相移输出、TripZone 软件拉闸保护
- ADCC / ADCD 六路差分采样与实际值换算
- P2S / S2P 双向功率流控制
- VMode / IMode / VIMode 三种闭环模式
- SPS / EPS / BDPS 调制入口
- 软启动与运行中目标值更新
- SCIB + CH340 + VOFA JustFloat 调试通信
- Flash 编译运行，关键 RAM 函数自动拷贝到 RAM

This project is a TI F28388D / C2000Ware based DAB control project. It includes ePWM phase-shift output, ADC sampling, software TripZone protection, bidirectional P2S/S2P control, voltage/current loop modes, soft-start, BDPS modulation, and SCIB + VOFA debug communication.

---

## 1. 工程目录 / Project Structure

| 路径 / Path | 作用 / Purpose |
| --- | --- |
| `app/main.c` | 程序入口，初始化 Device、Board、ControlLoop、SCIB、VOFA，并运行主循环 |
| `board/` | 板级驱动：ePWM、ADC、GPIO、SCI、TripZone、安全使能 |
| `control/` | DAB 控制算法、PI 参数、软启动、保护判断、调制入口 |
| `comm/` | VOFA 通信协议、SCIB 收发、上位机参数命令解析 |
| `device/` | TI device 支持代码 |
| `DCL/` | TI Digital Control Library 相关 PI / DF 控制函数 |
| `linker/` | RAM / Flash linker cmd |
| `docs/` | 控制链路、状态机、调制算法示意图 |

If you only want to tune control targets, PI gains, soft-start time, power direction, or control mode, normally you only need to edit `control/` and use VOFA commands. Avoid touching `device/`, `driverlib/`, or low-level board drivers unless you are changing hardware behavior.

---

## 2. 控制链路总览 / Control Overview

![DAB control loop overview](docs/control_loop_overview.svg)

主控制链路如下：

```text
EPWM9 SOCA
    -> ADCC / ADCD sampling
    -> ADCC1 ISR
    -> Board_ADC_UpdateCacheFromResult()
    -> ControlLoop_adcISR()
    -> ADC actual value conversion
    -> overload / over-voltage check
    -> target and PI update
    -> DABSoftStar() or ControlLoop_runMainCtrl()
    -> SPS / EPS / BDPS modulation
    -> Board_EPWM_setAllPhaseDeg()
```

The high-frequency control code runs from the ADC ISR. Communication is intentionally placed in the low-priority main loop slow task, so SCI/VOFA traffic will not block the control ISR.

---

## 3. 编译方式 / Build Mode

当前工程已经切换为 **Flash 编译**。

Current build is configured for **Flash execution**.

关键点：

- `_FLASH` 已加入工程定义
- 普通 `.text` 放到 Flash
- `.TI.ramfunc` 从 Flash 加载到 RAM 运行
- DCL 的 `dclfuncs` 也放入 `.TI.ramfunc`，避免 DCL PI 函数直接跑 Flash
- `.bss` 扩展到 `RAMLS5 | RAMLS6`

命令行构建：

```powershell
& "D:\Ti\ccs2051\ccs\utils\bin\gmake.exe" -C CPU1_RAM all
```

虽然目录名仍叫 `CPU1_RAM`，但当前生成 makefile 已经使用：

```text
--define=_FLASH
../linker/2838x_FLASH_lnk_cpu1.cmd
```

在 CCS 里建议选择 `CPU1_FLASH` 配置，或者确认当前配置包含 `_FLASH` 并排除了 `2838x_RAM_lnk_cpu1.cmd`。

---

## 4. PWM 输出与安全链路 / PWM Output and Safety

PWM 安全逻辑分两层：

1. 控制层发现故障后写 `DABCSS.error`，状态切到 `DABERROR`
2. 板级安全层通过 `Board_EPWM_forceLowAll()` 触发软件 TripZone，把 PWM 强制拉低

主循环里运行：

```c
Board_SystemTask();
ControlLoop_slowTask();
```

`Board_SystemTask()` 只看两个状态：

- `gBoardSystem.fault_latched`
- `gBoardSystem.pwm_output_enabled`

无故障且 `pwm_output_enabled == true` 时，才会调用 `Board_EPWM_enableOutputs()` 解除 TripZone。

当前 main 初始化顺序里已经先设置 50% duty，再请求使能 PWM：

```c
Board_EPWM_setDutyAll(0.5f);
gBoardSystem.pwm_output_enabled = true;
```

This means the PWM outputs are still held low by TripZone during initialization. They are only released later in the main loop when the safety supervisor sees no latched fault.

---

## 5. ADC 六路实际值 / Six ADC Actual Channels

控制层使用 `gAdcCActual[]` 和 `gAdcDActual[]`，它们是经过 offset / scale 换算后的实际物理量。

```c
gAdcCActual[0] = Second Side Current
gAdcCActual[1] = Second Side Tank Current
gAdcCActual[2] = Second Side Voltage

gAdcDActual[0] = Primary Side Tank Current
gAdcDActual[1] = Primary Side Voltage
gAdcDActual[2] = Primary Side Current
```

新上板时先在 CCS Expressions 或 VOFA 中观察这六个值，确认零点、比例和方向正确，再闭环。

Before closing the loop, verify that these actual values match the real hardware voltage/current measurements.

---

## 6. 控制模式 / Control Modes

### 6.1 功率方向 / Power Direction

| 枚举 / Enum | 含义 / Meaning |
| --- | --- |
| `P2S` | Primary to Secondary，原边到副边 |
| `S2P` | Secondary to Primary，副边到原边 |

### 6.2 闭环模式 / Loop Mode

| 枚举 / Enum | 含义 / Meaning |
| --- | --- |
| `VMode` | 单电压环，voltage PI output goes directly to modulation |
| `IMode` | 单电流环，current PI output goes directly to modulation |
| `VIMode` | 电压外环 + 电流内环，voltage loop provides current reference |

当前电压环比电流环慢 10 倍：

```c
gControlCurrentLoopTsSec = 10.0e-6f;
gControlVoltageLoopDiv = 10U;
gControlVoltageLoopTsSec = 100.0e-6f;
```

The current loop runs every ADC ISR. The voltage loop runs once every 10 current-loop cycles.

### 6.3 调制方式 / Modulation Mode

| 枚举 / Enum | 调制 / Modulation | 当前入口 / Current Entry |
| --- | --- | --- |
| `SPSDAB` | Single Phase Shift | `SPSCTRL()` |
| `EPSDAB` | Extended Phase Shift | `EPSCTRL()` |
| `TPSDAB` | Triple Phase Shift | 当前回退到 `SPSCTRL(0.0f)` |
| `BDPSOPTDAB` | Optimized BDPS | `BDPSOPTCTRL()` |

非法模式或未实现模式不会空操作，而是回退到 `SPSCTRL(0.0f)`。

Invalid or unimplemented modulation modes fall back to `SPSCTRL(0.0f)` instead of leaving the previous PWM phase unchanged.

![Modulation algorithms](docs/modulation_algorithms.svg)

---

## 7. 软启动与目标值更新 / Soft-Start and Target Update

状态流如下：

![State flow](docs/state_flow.svg)

软启动函数：

```c
void DABSoftStar(CtrlLoop *C);
```

默认软启动时间：

```c
DABSoftStart.SoftTime = 2.0f;
```

运行中只改目标电压或目标电流时，不再重新软启动。正确做法是：

```c
TempTargetVoltage = 48.0f;
TempTargetCurrent = 5.0f;
DABCSS.sts = DABWAITCHANGE;
```

下一次 ADC ISR 会同步目标值。如果软启动已经完成，会直接回到 `DABRUNING` 正常运行；如果上电软启动尚未完成，会继续保持软启动链路。

Changing target voltage/current during runtime does not restart soft-start. It only updates the target in the next ADC ISR.

---

## 8. PI 参数更新 / PI Parameter Update

电压环参数：

```c
gControlVpiKp
gControlVpiKi
gControlVpiUmax
gControlVpiUmin
gControlVpiImax
gControlVpiImin
```

电流环参数：

```c
gControlIpiKp
gControlIpiKi
gControlIpiUmax
gControlIpiUmin
gControlIpiImax
gControlIpiImin
```

改完 PI 参数后，不要直接改 DCL 内部寄存器，应调用：

```c
ControlLoop_requestVoltagePIUpdate();
ControlLoop_requestCurrentPIUpdate();
```

VOFA 接收命令中已经自动调用对应更新请求。

---

## 9. 保护逻辑 / Protection Logic

保护阈值结构体：

```c
DAB_OVERLOAD DABOverload;
```

检测对象：

| 故障 / Fault | 采样值 / Sample |
| --- | --- |
| `PrimarySideTrankCurrentOverLoard` | `gAdcDActual[0]` |
| `SecondSideTrankCurrentOverLoard` | `gAdcCActual[1]` |
| `PrimarySideInputCurrentOverLoard` | `gAdcDActual[2]` |
| `SecondSideInputCurrentOverLoard` | `gAdcCActual[0]` |
| `PrimarySideInputVoltageOverLoard` | `gAdcDActual[1]` |
| `SecondSideInputVoltageOverLoard` | `gAdcCActual[2]` |

超限后：

```text
DABCSS.error = fault code
DABCSS.sts = DABERROR
Board_SystemSetFault()
Board_EPWM_forceLowAll()
```

After a fault, PWM output is forced low by software TripZone.

---

## 10. VOFA 调试通信 / VOFA Debug Communication

### 10.1 硬件连接 / Hardware Connection

当前使用 SCIB + CH340：

| DSP 引脚 / DSP Pin | 功能 / Function | 连接 / Connection |
| --- | --- | --- |
| GPIO9 | SCIB_TX | 接 CH340 RXD |
| GPIO11 | SCIB_RX | 接 CH340 TXD |
| GND | Ground | 接 CH340 GND |

默认波特率：

```c
BOARD_SCIB_DEFAULT_BAUD = 921600
```

### 10.2 VOFA+ 接收波形 / VOFA+ Waveform Receive

发送格式为 VOFA JustFloat：

```text
float ch0
float ch1
...
float chN
00 00 80 7F
```

当前慢任务发送 6 路 ADC 实际值：

| VOFA 通道 / Channel | 数据 / Data |
| --- | --- |
| ch0 | `gAdcCActual[0]` |
| ch1 | `gAdcCActual[1]` |
| ch2 | `gAdcCActual[2]` |
| ch3 | `gAdcDActual[0]` |
| ch4 | `gAdcDActual[1]` |
| ch5 | `gAdcDActual[2]` |

VOFA+ 配置：

```text
串口：CH340 对应 COM 口
波特率：921600
协议：JustFloat
通道数：6
帧尾：00 00 80 7F
```

单帧字节数：

```text
6 channels * 4 bytes + 4 bytes tail = 28 bytes
```

### 10.3 VOFA 下发参数 / VOFA Command Receive

接收格式：

```text
AA + two-character command + decimal value + BB
```

支持命令：

| 命令 / Command | 示例 / Example | 作用 / Purpose |
| --- | --- | --- |
| `VP` | `AAVP0.02BB` | 设置电压环 Kp |
| `VI` | `AAVI0.001BB` | 设置电压环 Ki |
| `IP` | `AAIP0.02BB` | 设置电流环 Kp |
| `II` | `AAII0.001BB` | 设置电流环 Ki |
| `VT` | `AAVT48BB` | 设置目标电压 |
| `IT` | `AAIT5BB` | 设置目标电流 |

PI 命令会自动触发对应 `ControlLoop_request...PIUpdate()`。目标值命令会把状态置为 `DABWAITCHANGE`，下一次 ADC ISR 同步目标值。

### 10.4 中断与数据一致性 / Interrupt and Data Consistency

VOFA 发送在 `ControlLoop_slowTask()`，不在高频 ADC ISR 中发送。`vofaAdcData[]` 是慢任务局部数组，调用 `VOFA_CommSendFloatFrame()` 时会立即拷贝到内部静态发送缓冲 `gVofaTxBuffer[]`，所以发送过程中被 ADC ISR 打断不会导致局部数组悬空。

注意：如果 ADC ISR 正好在主循环读取 6 路 ADC 值时打断，单帧内可能出现前半部分来自上一拍、后半部分来自下一拍。这不影响程序安全，但如果要严格同一采样周期快照，可以后续在 ADC ISR 末尾维护专用 VOFA 快照数组。

---

## 11. 常用调试变量 / Useful Watch Variables

```c
TempTargetVoltage
TempTargetCurrent
DABCSS.sts
DABCSS.error
DABCSS.CtrlMode
DABCSS.CtrlLoopTransmissionMode
DABCSS.LoopMode
DABSoftStart.SoftSTS
DABSoftStart.SoftTemp
DABSoftStart.SoftTarget
DABCtrl.VoltageLoopOut
DABCtrl.CurrentLoopOut
gAdcCActual[0]
gAdcCActual[1]
gAdcCActual[2]
gAdcDActual[0]
gAdcDActual[1]
gAdcDActual[2]
gBoardSystem.pwm_output_enabled
gBoardSystem.fault_latched
```

BDPS 调试变量：

```c
gBdpsLast.U1
gBdpsLast.U2
gBdpsLast.I2
gBdpsLast.D2_in
gBdpsLast.D2
gBdpsLast.D1
gBdpsLast.err_flags
```

---

## 12. 上板前检查 / Bring-Up Checklist

1. 确认 CH340 串口接线：GPIO9 -> RXD，GPIO11 <- TXD，GND 共地。
2. 确认 Flash 版本可正常下载并启动。
3. 先不要上高压，观察 PWM 是否被 TripZone 正确拉低/释放。
4. 用示波器确认四路 PWM 为 50% duty，且相位关系正确。
5. 用 CCS 或 VOFA 确认六路 ADC 实际值正确。
6. 设置较小目标电压和较长软启动时间。
7. 确认 `DABOverload` 六个阈值符合硬件。
8. 确认 `DABCSS.error == 0` 后再逐步提高目标。
9. 闭环前先确认 P2S / S2P 方向、采样符号和硬件接线一致。
10. 出现异常时先看 `DABCSS.error` 和 `gBoardSystem.fault_latched`。

---

## 13. 不建议随便修改 / Do Not Change Casually

除非明确知道原因，否则不要随便改：

- `device/` 和 `driverlib/`
- ADC SOC 触发源和 SOC 顺序
- EPWM9 SOCA 触发链路
- TripZone 强制低电平逻辑
- `Board_EPWM_setAllPhaseDeg()` 的相位映射
- DCL 内部结构体字段
- BDPS 数学公式本身

For most tuning work, edit control parameters or use VOFA commands instead of changing low-level hardware drivers.
