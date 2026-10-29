# 车型智能搬运机器人 · 整机固件（从零实现）

![Language](https://img.shields.io/badge/language-C-00599C)
![MCU](https://img.shields.io/badge/MCU-STM32F407%20%40%20168MHz-green)
![Control](https://img.shields.io/badge/control-200%20Hz%20ISR-blueviolet)
![States](https://img.shields.io/badge/route%20states-53-orange)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

> 2026 中国机器人大赛暨 RoboCup 中国赛 · 工程竞技类 · **车型智能搬运**赛项
> **专项赛三等奖**
>
> 1.2 万行业务代码（BSP + Core），53 状态路线机，从零写完。

---

## 这套程序是怎么来的

原本计划继承学长的工程继续改。但代码交接在中途断了——带队队长没能把代码发过来，中转站上的版本又已经过期，取不回来。

没有退路，只能从零写。

那段开发期还不太会用 git，所有版本靠"改一版、压一个 zip、换个名字"来保存。桌面上那串 `F407_三环PID_简单循迹.zip` → `_任务1基本完成.zip` → `_巡线模块化.zip` → `_扫码器好.zip` → `_无位置环.zip` → `_可顺利到第一个定位点.zip`，就是这条从零长起来的路径。直到 8 月中旬才转成正经的 git 流程，于是有了本仓库从 8 月 17 日开始的提交记录。

**这套代码每一行都是自己写的**，没有继承任何前人的工程。

## 规模

| 指标 | 数值 |
| --- | --- |
| BSP + Core 代码量 | **12 336 行** |
| 比赛路线状态 | **53** 个 |
| 底盘控制频率 | **200 Hz**（TIM7 硬实时中断，5 ms） |
| 开发周期 | 8.17 – 8.22 每日迭代，含赛前最后一天收尾 |
| 里程标定 | 1850 counts/cm（实测） |

作为对照：一队车那套是 7 788 行 / 24 状态。两套是**互相独立的实现**，不是同一份代码改参数。

---

## 架构

```mermaid
graph TD
    subgraph 硬实时 200Hz TIM7 中断
        ISR[ProjectService_OnTimerElapsed] --> SMP[编码器采样<br/>velocity counts/5ms]
        SMP --> INT[编码器积分<br/>encoder counts]
        INT --> SM[53 状态路线机]
        SM --> CP[ChassisPos<br/>位置环]
        SM --> LF[LineFollow<br/>灰度巡线]
        SM --> VA[VisionAlignment<br/>视觉对准]
        CP --> WT[wheel_target 原子发布]
        LF --> WT
        VA --> WT
        WT --> CTL[ChassisMotor_ControlTick<br/>四轮增量 PID]
        CTL --> PWM[TIM8 → TB6612]
    end

    subgraph 主循环 ~100Hz
        ML[main loop] --> CT[CompetitionTask_Process]
        CT --> QR[条形码/二维码处理]
        CT --> VAP[VisionAlignment_Process]
        CT --> PSR[ProjectService_Run<br/>屏显 / 非实时逻辑]
    end

    IMU[HWT101 陀螺仪<br/>UART5] --> HH[HeadingHold 航向环]
    HH --> SM
    CAM[MaixCAM<br/>UART4] --> VA
    SCAN[GM65 扫码<br/>USART1] --> QR
    VOFA[VOFA 上位机<br/>USART2] --> TUNE[在线调参]
    T9[TIM9 软件 PWM] --> SERVO[8 路舵机]
```

关键设计：**几乎所有运动状态机都跑在 TIM7 中断里**（200 Hz），主循环只做屏显和少量非实时逻辑。代价是中断里绝对不能阻塞——串口发送、LCD 刷新、状态切换都得推给主循环，这条纪律贯穿全工程。

---

## 几个自己踩出来的设计

### 转盘库存管理 · `BSP/TurntableInventory.c`

转盘能放多个物块，但"哪个槽里是什么"必须记住——否则放置时会拿错、或者对着空槽做出放料动作。

所以单独做了一个库存表：转盘有几个槽、每槽当前放的是哪一类物块、哪一槽已占用，全部显式记录。放置前先查库存（`TurntableInventory_GetSlot`），找不到就报错而不是盲放。这是本工程唯一一个纯粹为"状态一致性"而存在的模块。

### VOFA 在线调参 · USART2

调试时最痛的是改一个 PID 参数要重新编译、烧录、跑一圈。于是把 USART2 让给 VOFA 上位机，参数运行时直接改，改完立刻生效。

控制周期里只读一个结构体，不碰串口发送，所以不影响 200 Hz 的时序。

### 舵机梯形斜坡 · `Servo4_ArmRampTick` / `RotationRampTick`

舵机直接跳到目标脉宽会猛冲，机构容易抖、物块容易掉。所以每个舵机通道都走梯形加减速：每个 tick 只靠近目标一小步，到了才置完成标志。状态机等的是"到位"，而不是"发过指令"。

### 视觉对准的滞环超时

对准时有 15 s 超时保护。但实测发现：目标恰好在容差带边缘反复进出时，会一直卡在这个状态直到超时，然后带着偏差直接去抓。

补了一个滞环带停留计数器：在容差带内停留超过 500 ms 就视为对准完成，不再等满超时。这一条把"偶发对不准"变成了"稳定对不准但至少不超时"。

---

## 文档

本工程在调试期写了两份逐行核对的设计文档，比 README 详细得多：

| 文档 | 内容 |
| --- | --- |
| [`docs/路线状态机说明.md`](docs/路线状态机说明.md) | 全部状态、转移条件、子状态机，逐条标注 **活跃 / 不可达 / 死代码** |
| [`docs/标定参数清单.md`](docs/标定参数清单.md) | 全工程距离常量、PID 参数、对应源码行号，附复检清单 |

这两份文档是给自己看的——状态机长到 53 个之后，不写下来下一次改就会踩到上次踩过的坑。特别是"不可达"那几个状态，标出来才知道哪些是废代码。

---

## 目录结构

```
.
├── BSP/                          # 业务层（46 个文件）
│   ├── ProjectService.c          # 53 状态路线机（3572 行）
│   ├── ChassisPos.c              # 位置环：梯形加减速 + 到位判定
│   ├── ChassisMotor.c            # 速度环：4 路增量 PID
│   ├── HeadingHold.c             # 航向环：IMU yaw 闭环
│   ├── LineFollow.c/.h           # 巡线 + 全路线常量表
│   ├── VisionAlignment.c         # 视觉对准（滞环 + 搜索摆动）
│   ├── MaxiCam.c                 # 视觉串口帧解析
│   ├── TurntableInventory.c      # 转盘槽位库存
│   ├── Servo4.c                  # 8 路舵机 PWM + 梯形斜坡
│   ├── HWT101.c                  # 陀螺仪 + UART 回调分发
│   ├── Sensor.c                  # 前/后 8 路灰度
│   └── ...
├── Core/                         # CubeMX 生成 + 中断
├── Drivers/                      # ST HAL / CMSIS
├── MDK-ARM/                      # Keil 工程
├── docs/                         # 设计文档（见上）
├── ARCHITECTURE.md               # 模块职责与强制约束
└── BanyunCommonF407.ioc          # CubeMX 工程
```

## 构建

```
1. Keil MDK 5 打开  MDK-ARM/BanyunCommonF407.uvprojx
2. 编译（F7）→ ST-Link 下载（F8）
```

外设配置改动用 CubeMX 打开 `BanyunCommonF407.ioc` 重新生成。

## 标定

所有距离常量基于 **1850 counts/cm** 实测值，而理论值是 2315 counts/cm（77 mm 轮径、500 PPR、4 倍频、1:28 减速比）。差异来自麦轮打滑、轮子受压形变、地面摩擦和编码器安装误差。

**改任何一个常量前必须实车重标定，不要拿理论值直接换算**——这条在 `LineFollow.h` 和 `docs/标定参数清单.md` 里都写了，因为踩过。

---

## 复盘：从零重写值不值

**代价很实在**：前两个月基本在重复造别人已经造过的轮子。位置环、巡线、舵机时序这些，如果当初拿到学长代码，可能一周就能跑起来。53 个状态里有几个是"改不动了就加一个状态"堆出来的，后期维护成本明显上升。

**但也不是白写的**：因为每个模块都是自己搭的，出了问题知道去哪找。后来接手一队车、看 mtc 学长那套视觉代码时，能一天之内看懂串口协议和对准逻辑——这个能力是重写换来的。

如果再来一次，我会在第一天就搭好 git 和测试脚本，而不是靠 zip 存档。

## 姊妹项目

同一赛项的另一辆车（一队）是另一套独立实现，24 状态 / 7 788 行：
[`robocup-smart-carry-firmware`](https://github.com/vega-thrive/robocup-smart-carry-firmware)

## License

[MIT](LICENSE)
