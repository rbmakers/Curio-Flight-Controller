# Curio Flight Controller

> **Language / 語言**: [English](#english) | [繁體中文](#繁體中文)

<img width="348" height="395" alt="image" src="https://github.com/user-attachments/assets/2b9b0481-7a06-441a-aec4-a85278cf8e2b" />
<img width="348" height="395" alt="image" src="https://github.com/user-attachments/assets/9363fa47-875a-48b3-b787-a798d51f5852" />

---

## English

### Background & Motivation

The rapid proliferation of unmanned aerial vehicles — from agricultural inspection and disaster relief to the asymmetric battlefield tactics prominently demonstrated in recent conflicts — has made UAV software and hardware literacy an increasingly critical engineering competency. Taiwan possesses world-class semiconductor and electronics manufacturing capabilities, yet its university-level UAV education ecosystem remains underdeveloped: most courses stop at theory, and students rarely encounter a platform that takes them in one continuous journey from bare-metal circuit bring-up, through firmware development, to closed-loop flight validation.

**Curio** was designed to close that gap. It is an open-source, education-first quadcopter flight controller built around the Raspberry Pi **RP2354A** microcontroller (dual Cortex-M33, 150 MHz).

### Why Curio? — Lessons from Existing Platforms

Before committing to a design, existing platforms were evaluated against a single core criterion: *how much of the learner's cognitive budget goes directly toward flight-control fundamentals, and how much is consumed by problems unrelated to the control loop itself?*

**Crazyflie** (Bitcraze) is the gold standard of academic micro-drone research. Its dual-MCU architecture — STM32F405 for flight control, nRF51822 for communications — combined with the custom CRTP multiplexing protocol, enabled landmark achievements: the Crazyswarm system simultaneously controlling 49 vehicles from a single PC, SE(3) geometric controllers, 18-second sim-to-real reinforcement learning, and on-board MPC solvers. These results, however, were achieved *against* structural constraints: 192 KB SRAM and a single Cortex-M4 core set hard ceilings on on-board inference and deterministic loop timing. For a student whose goal is to understand the flight-control loop, the additional MCU and CRTP layer introduce an abstraction barrier before the first line of control code is even written.

**dRehmFlight** (Nicholas Rehm, Teensy 4.1) takes the opposite philosophy: a complete multi-rotor and VTOL flight-controller distilled into a single Arduino sketch, placing every PID gain, sensor fusion step, and actuator command in plain sight. Its educational transparency is unmatched. Its limitation is that Teensy is a general-purpose development board — students must hand-wire IMU, barometer, and receiver modules before any tuning can begin, spending time on cable routing, weight budgets, and mechanical integration rather than on the control algorithm itself.

**Scout** (MicroPython on RP2040) demonstrated that Raspberry Pi Silicon has the compute headroom and toolchain maturity for flight-control tasks, but MicroPython's real-time limitations make it unsuitable as a production flight-controller substrate.

**Curio inherits the algorithmic transparency of dRehmFlight** — the firmware is structured identically, so every equation in every loop is directly readable — **while delivering the hardware integration of a purpose-built flight controller**, so students can focus on making the vehicle fly from the moment power is applied.

### The RP2354A Advantage

Curio's choice of the RP2354A is architecturally deliberate, not incidental:

| Feature | STM32F405 (Crazyflie) | RP2354A (Curio) |
|---|---|---|
| CPU cores | 1× Cortex-M4 @ 168 MHz | 2× Cortex-M33 @ 150 MHz |
| Instruction set | ARMv7-M | ARMv8-M + TrustZone |
| Floating-point | Single-precision FPU | Single-precision FPU + dual-precision coprocessor |
| SRAM | 192 KB | 520 KB (10 independent banks) |
| External memory | None | QSPI, up to 16 MB PSRAM |
| Programmable I/O | None | 12× PIO state machines |
| Secure boot | No | TrustZone + OTP signature verification |

The dual-core architecture maps directly onto the flight-control problem: **Core 0** runs a deterministic 2 kHz control loop (IMU read → sensor fusion → PID/geometric control → PWM output) with no OS scheduling interference; **Core 1** handles all communication tasks (ELRS parsing, MAVLink, logging), communicating with Core 0 via the hardware inter-processor FIFO. This is the clean, hardware-enforced task isolation that Crazyflie achieved only by splitting work across two physically separate chips.

The 520 KB SRAM — nearly three times that of the STM32F405 — meaningfully expands the space available for on-board policy networks (residual RL) and MPC solvers. The PIO state machines handle DSHOT ESC signaling and CRSF decoding entirely in background hardware, freeing Core 0's cycles for control computation.


### Hardware Specifications

| Component | Details |
|---|---|
| Microcontroller | RP2354A — dual Cortex-M33, 150 MHz |
| IMU | Bosch BMI088 — separate accel + gyro dice, SPI, 2 kHz interrupt-driven |
| Barometer | Bosch BMP580 — I²C, 50 Hz, 24-bit pressure + temperature |
| RC Receiver | ExpressLRS via CRSF — UART0 (GPIO12/13), 420 kBaud, 16 channels |
| Brushed motors | 4 × MOSFET PWM (GPIO 29/11/18/25, 32 kHz, 10-bit) |
| BLDC ESC output | 4 × DShot600 PIO (GPIO 1/4/10/14) — pads reserved for BLDC expansion |
| Battery sense | 12-bit ADC on GPIO 27 (100 kΩ / 10 kΩ voltage divider) |
| Power tree | TPS630701 Buck-Boost + MIC5323 LDO — clean MCU/sensor rail regardless of cell voltage |
| Debug | Standard SWD interface for GDB-level debugging |
| Build environment | Arduino IDE 2.3.x + earlephilhower/arduino-pico ≥ 3.x |

The BMI088 is the same IMU used on the Crazyflie 2.1 and the NanoBench benchmark dataset platform, meaning control algorithms developed on Curio can be cross-compared against published academic baselines directly.

### Repository Structure

```
Curio-FlightController/
├── Curio_FlightController/             # Single-core version (baseline)
│   ├── Curio_FlightController.ino      # All tasks on Core 0
│   ├── config.h
│   └── src/
│       ├── bmi088_driver.h/.cpp
│       └── bmp580_driver.h/.cpp
├── Curio_FlightController_DualCore/    # Dual-core version (RP2354A feature demo)
│   ├── Curio_FlightController_DualCore.ino  # Core 0 = control, Core 1 = CRSF
│   ├── config.h
│   └── src/
│       ├── bmi088_driver.h/.cpp
│       └── bmp580_driver.h/.cpp
├── test_IMU/
│   └── test_IMU.ino
├── test_ELRS/
│   └── test_ELRS.ino
├── test_Barometer/
│   └── test_Barometer.ino
└── README.md
```

### Single-Core vs Dual-Core Architecture

One of the RP2354A's most important advantages over the STM32F405 used in Crazyflie is its **two independent Cortex-M33 cores**. This repository provides two complete firmware versions so this advantage can be both understood and **quantitatively benchmarked**:

| | `Curio_FlightController` | `Curio_FlightController_DualCore` |
|---|---|---|
| **Core 0** | IMU → Madgwick → PID → PWM **+ CRSF parsing** | IMU → Madgwick → PID → PWM only |
| **Core 1** | Idle (unused) | CRSF parser (tight loop, all 16 channels) |
| **Inter-core comms** | None | Volatile `CRSFShared` struct (32-bit atomic writes) |
| **CRSF jitter on Core 0** | Present — UART drain time varies with buffer depth | Zero — Core 1 owns UART entirely |

#### Why Does This Matter?

In the single-core version, `parseCRSF()` runs inside `loop()` on Core 0. Every iteration, Core 0 must drain whatever bytes have accumulated in the 420 kBaud UART FIFO since the last iteration. Because CRSF frames are 26 bytes at irregular intervals, the number of bytes waiting varies — sometimes zero, sometimes a full frame. This creates **non-deterministic execution time** in the control loop, manifesting as loop jitter.

In the dual-core version, Core 0 never touches the UART. It calls `snapshotChannels()` — a simple volatile struct read — instead of a UART parsing loop. Core 1 handles all byte-level CRSF work in a tight, unthrottled loop running independently. Core 0's execution time is determined entirely by the control computation, which is **deterministic**.

#### Dual-Core Inter-Core Communication

```
Core 1 (CRSF parser)              Core 0 (control loop)
─────────────────────             ──────────────────────────
loop1() {                         loop() {
  c1_parseCRSF();   ──writes──►    snapshotChannels();
  // updates:                       // reads:
  crsfShared.ch1                    channel_1_pwm = crsfShared.ch1
  crsfShared.ch2                    channel_2_pwm = crsfShared.ch2
  crsfShared.ch3                    ...
  crsfShared.lastFrameMs            failsafe check
}                                 }
```

Each field in `CRSFShared` is a 32-bit aligned `volatile uint32_t`. On Cortex-M33, 32-bit aligned word stores/loads are atomic — no mutex is needed. The worst-case outcome is Core 0 reading a value one CRSF frame (≈6.7 ms at 150 Hz) old, which is negligible for RC channels.

#### Benchmark Procedure

Both firmware versions include `BENCHMARK_MODE`. When enabled, the firmware prints Core 0 loop timing statistics every second over USB Serial. **The key metric is Jitter = Max_dt − Min_dt.**

**Step 1 — Single-core baseline**
1. Open `Curio_FlightController/Curio_FlightController.ino`.
2. Uncomment `#define BENCHMARK_MODE`.
3. Flash and open Serial Monitor at 500000 baud with ELRS receiver connected and transmitter on.
4. Record the printed Jitter value. Example output:
   ```
   ──── Core 0 Loop Benchmark [SINGLE-CORE] ────
     Iterations :  2000
     Mean dt    :  498 µs
     Min  dt    :  490 µs
     Max  dt    :  563 µs
     Jitter     :   73 µs  ← compare with DualCore version
     (CRSF parsing runs on Core 0 — inflates jitter)
   ```

**Step 2 — Dual-core comparison**
1. Open `Curio_FlightController_DualCore/Curio_FlightController_DualCore.ino`.
2. `#define BENCHMARK_MODE` is already uncommented by default.
3. Flash (same receiver and transmitter setup).
4. Compare Jitter. Example output:
   ```
   ──── Core 0 Loop Benchmark (1-second window) ────
     Iterations :  2000
     Mean dt    :  494 µs
     Min  dt    :  492 µs
     Max  dt    :  497 µs
     Jitter     :    5 µs  ← key metric
     Core1 frames:  150  CRC errors: 0
   ```

The jitter reduction demonstrates real, hardware-enforced task isolation — the defining architectural advantage of the RP2354A over single-core flight controllers.

### Software Architecture

The firmware follows the dRehmFlight control architecture directly, adapted for Curio hardware:

```
2 kHz main loop — Core 0 (deterministic, no OS)
│
├─ getIMUdata()          — fetch atomic BMI088 snapshot from 2 kHz DRDY ISR
│                          apply bias offsets + 1st-order LP filter
│
├─ Madgwick6DOF()        — 6-DOF quaternion sensor fusion
│                          → roll, pitch, yaw (degrees)
│
├─ getDesState()         — CRSF µs values → physical setpoints
│                          (throttle 0–1, roll/pitch ±30°, yaw ±160 °/s)
│
├─ updateAltHold()       — 50 Hz baro outer P loop (altitude → climb-rate setpoint)
│   (if USE_ALT_HOLD)      + climb-rate inner PID → throttle trim
│
├─ controlANGLE()        — single-loop angle PID               (recommended first)
│  controlANGLE2()       — cascaded angle → rate PID           (better dynamics, harder to tune)
│  controlRATE()         — direct rate / acro mode
│
├─ controlMixer()        — X-quad mixing
│                          M1(FL)  M2(FR)  M3(RR)  M4(RL)
│
├─ scaleCommands()       — normalized 0–1 → 10-bit PWM (0–1023)
│
├─ throttleCut()         — CH5 > 1500 µs → instant disarm, zero all motors
│
├─ commandMotors()       — analogWrite() to four MOSFET motor pins
│
└─ getCommands()         — parseCRSF() + CRSF timeout failsafe

Core 1 (future expansion)
└─ ELRS / MAVLink / logging — communications isolated from control loop
```

### Getting Started

#### Step 1 — Install Build Environment

1. Install [Arduino IDE 2.3.x](https://www.arduino.cc/en/software).
2. Open **File → Preferences → Additional Boards Manager URLs** and add:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
3. Go to **Tools → Board → Boards Manager**, search **Raspberry Pi Pico/RP2040**, and install.
4. Select **Board: "Raspberry Pi Pico 2"**, **CPU Speed: 150 MHz**.

#### Step 2 — Pre-Flight Sensor Verification

Always run the three component tests before flashing the main flight controller. This confirms hardware is wired correctly and provides the IMU calibration values required for stable flight.

**A — IMU Test (`test_IMU/test_IMU.ino`)**
1. Flash and open Serial Monitor at **500000 baud**.
2. Chip-ID check runs automatically. Expected output:
   ```
   [BMI088] ACC chip ID: 0x1E  (expect 0x1E)   GYR chip ID: 0x0F  (expect 0x0F)
   [OK]  BMI088 found and configured.
   ```
3. Enter `c` to run bias calibration — board must be **flat and completely still**.
4. Copy the six printed values into `Curio_FlightController.ino`:
   ```cpp
   float AccErrorX  = /* printed value */;
   float AccErrorY  = /* printed value */;
   float AccErrorZ  = /* printed value */;
   float GyroErrorX = /* printed value */;
   float GyroErrorY = /* printed value */;
   float GyroErrorZ = /* printed value */;
   ```
5. Enter `s` to stream live data. Verify: board level → `Az ≈ +1 g`, `Ax ≈ Ay ≈ 0`. Tilt the board and confirm Roll/Pitch angles track correctly.

**B — ELRS Receiver Test (`test_ELRS/test_ELRS.ino`)**
1. Flash, power on your transmitter, confirm ELRS link LED is solid.
2. Open Serial Monitor at **500000 baud**.
3. Expected: 16 channels as bar charts, updating at 10 Hz.
4. Move all sticks and switches; verify channel assignments match the mapping table below.

**C — Barometer Test (`test_Barometer/test_Barometer.ino`)**
1. Flash and open Serial Monitor at **500000 baud**.
2. Enter `n` to run the noise test. Expected 1-σ altitude std-dev **< 0.30 m** indoors.
3. Enter `s` to stream. Confirm pressure ≈ 101 000 Pa at sea level (adjust for your altitude).

#### Step 3 — Transmitter Channel Mapping

| RC Channel | Function | Range |
|---|---|---|
| CH1 | Throttle | 1000 (min) – 2000 (max) µs |
| CH2 | Roll (Aileron) | 1000 – 2000, center 1500 µs |
| CH3 | Pitch (Elevator) | 1000 – 2000, center 1500 µs |
| CH4 | Yaw (Rudder) | 1000 – 2000, center 1500 µs |
| CH5 | Arm switch | < 1500 = armed, > 1500 = disarmed |
| CH6 | Altitude hold | > 1500 = altitude hold ON |

#### Step 4 — Arming Procedure

1. Throttle stick **fully down** (CH1 < 1060 µs).
2. Arm switch to **armed position** (CH5 < 1500 µs).
3. Curio arms; raise throttle to spin motors.
4. To disarm: flip CH5 to disarmed (> 1500 µs) **at any time**.

#### Step 5 — PID Tuning

All gains live in `config.h`. Begin with **angle mode** (`USE_CONTROL_ANGLE`, the default):

| Parameter | Default | Effect |
|---|---|---|
| `KP_ROLL_ANGLE` | 0.2 | Increase for snappier roll response |
| `KI_ROLL_ANGLE` | 0.3 | Corrects steady-state lean |
| `KD_ROLL_ANGLE` | 0.05 | Damps oscillation; raise carefully |
| `B_MADGWICK` | 0.04 | Lower → gyro-dominant, slower but smoother attitude estimate |
| `B_ACCEL` | 0.14 | Accelerometer LP filter coefficient |
| `B_GYRO` | 0.10 | Gyroscope LP filter coefficient |

Once angle mode is stable, switch to `USE_CONTROL_ANGLE2` (cascaded outer angle + inner rate PID) for improved dynamic response. Rate mode (`USE_CONTROL_RATE`) is for acro / manual flying.

#### Learning Pathway

| Stage | Goal | Recommended starting point |
|---|---|---|
| Beginner | Understand PID loops, tune a stable hover | `controlANGLE()`, serial debug prints |
| Intermediate | Sensor fusion, Madgwick filter internals | Modify `B_madgwick`, observe quaternion drift |
| Advanced | Cascaded control, altitude hold | `controlANGLE2()`, `USE_ALT_HOLD` |
| Research | Geometric control, residual RL, on-board MPC | Port algorithms to Core 0; offload comms to Core 1 |

### Safety Notice

> **PROPELLERS OFF during all software development and PID tuning.**
> Brushed motors spin immediately upon arming if throttle is above minimum.
> Always disarm before picking up or handling the craft.

### License

MIT License. Firmware architecture based on [dRehmFlight](https://github.com/nickrehm/dRehmFlight) by Nicholas Rehm (MIT). Scout reference: [TimHanewich/scout](https://github.com/TimHanewich/scout).

---

## 繁體中文

### 研究背景與開發動機

無人載具（UAV）技術的快速演進，從農業巡檢、災害救援到近年衝突中低成本商用無人機所展現的不對稱戰力，已使無人機軟硬體開發能力成為關鍵工程素養。台灣擁有世界頂尖的半導體與電子製造基礎，然而大學層級的無人機教育生態卻相對薄弱：多數課程停留於理論，少有平台能讓學生從底層電路設計、韌體開發，一路貫通至閉迴路飛行驗證。

**Curio** 是一款以教育為核心、開源的四旋翼飛行控制板，核心採用 Raspberry Pi **RP2354A** 微控制器（雙核 Cortex-M33，150 MHz）。

### 為什麼是 Curio？——從現有平台汲取的設計教訓

在確定設計方向前，本專案系統性評估了市場上幾款主流無人機教育平台。

**Crazyflie**（Bitcraze）是學術界最受推崇的微型無人機研究平台。其雙 MCU 架構——STM32F405 負責飛控運算、nRF51822 負責通訊——搭配自主研發的 CRTP 多工通訊協定，催生了一系列里程碑成果：Crazyswarm 系統以單台電腦同步控制 49 架飛行器、SE(3) 幾何控制器的實踐、僅需 18 秒的 Sim-to-Real 強化學習遷移，以及板載 MPC 求解器。然而這些成就，是在 STM32F405 192 KB SRAM 與單核 Cortex-M4 的算力限制下取得的，其代價是板載推理記憶體始終捉襟見肘、確定性控制時序受 FreeRTOS 任務切換干擾。對以學習飛控邏輯為目標的學生而言，nRF51822 與 CRTP 所形成的抽象層，在理解控制程式碼之前，已形成了一道不必要的障礙。

**dRehmFlight**（Nicholas Rehm，Teensy 4.1）採取簡潔的哲學：將完整的飛控邏輯濃縮在單一 Arduino 程式檔案中，PID 增益、感測器融合、電機指令 都清楚流暢, 容易理解。其限制在於 Teensy 是通用開發板——學習者在開始任何調較參數之前，必須先自行搭接 IMU、氣壓計與接收機模組，大量時間耗費在配線、重量管控與機體整合，而非飛控演算法本身。

**Scout**（MicroPython on RP2040）印證了 Raspberry Pi Silicon 在飛控場景的算力潛力，但 MicroPython 的即時性先天劣勢，使其難以作為生產級飛控的開發基底。

**Curio 繼承了 dRehmFlight 的清楚易解, 同步提供一個突顯雙核能力的比對版本**——韌體架構完全對應，每個迴圈、每個方程式均可直接閱讀——**以專用整合飛控板的形式消除通用板外掛的機體工程門檻**，讓「開電源、實習演算法、調校及觀察飛行響應」成為學習的起點，而非終點。

### RP2354A 的架構優勢

| 規格項目 | STM32F405（Crazyflie） | RP2354A（Curio） |
|---|---|---|
| 處理器核心 | 單核 Cortex-M4 @ 168 MHz | 雙核 Cortex-M33 @ 150 MHz |
| 指令集架構 | ARMv7-M | ARMv8-M（含 TrustZone） |
| 浮點支援 | 單精度 FPU | 單精度 FPU + 雙精度協處理器 |
| 片上 SRAM | 192 KB | 520 KB（10 個獨立 Bank） |
| 外部記憶體 | 無 | QSPI，最高 16 MB PSRAM |
| 可程式化 I/O | 無 | 12 組 PIO 狀態機 |
| 安全啟動 | 無 | TrustZone + OTP 簽章驗證 |

雙核架構直接對應飛控問題的本質：**Core 0** 執行確定性 2 kHz 控制迴圈（IMU 讀取 → 感測器融合 → PID／幾何控制 → PWM 輸出），完全不受 OS 調度干擾；**Core 1** 承擔所有通訊任務（ELRS 解析、MAVLink、飛行日誌），透過硬體核間 FIFO 與 Core 0 交換數據。這正是 Crazyflie 用兩顆獨立晶片才能達成的任務隔離——Curio 在單顆 SoC 內以硬體層面實現。

520 KB SRAM 較 STM32F405 多出近三倍，為殘差 RL 策略網路與板載 MPC 求解器的部署提供了實質空間。PIO 狀態機則在背景獨立處理 DSHOT ESC 信號與 CRSF 解碼，完全解放 Core 0 的算力。

### 硬體規格

| 元件 | 說明 |
|---|---|
| 微控制器 | RP2354A — 雙核 Cortex-M33，150 MHz |
| IMU | Bosch BMI088 — 加速度計與陀螺儀分離封裝，SPI，2 kHz 中斷驅動 |
| 氣壓計 | Bosch BMP580 — I²C，50 Hz，24-bit 氣壓 + 溫度 |
| 遙控接收機 | ExpressLRS CRSF — UART0（GPIO12/13），420 kBaud，16 頻道 |
| 有刷馬達 | 4 × MOSFET PWM（GPIO 29/11/18/25，32 kHz，10-bit） |
| 無刷 ESC 輸出 | 4 × DShot600 PIO（GPIO 1/4/10/14）— 預留焊盤，可擴充無刷動力 |
| 電池電壓偵測 | 12-bit ADC on GPIO27（100 kΩ / 10 kΩ 分壓） |
| 電源樹 | TPS630701 Buck-Boost + MIC5323 LDO — 確保 MCU 與感測器供電純淨穩定 |
| 除錯介面 | 標準 SWD 接口，支援 GDB 除錯 |
| 開發環境 | Arduino IDE 2.3.x + earlephilhower/arduino-pico ≥ 3.x |

BMI088 與 Crazyflie 2.1 及 NanoBench 基準測試平台採用相同 IMU，意味著 Curio 上開發的控制演算法可直接與公開學術基準進行跨平台比較。

### 檔案結構

```
Curio-FlightController/
├── Curio_FlightController/             # 單核版本（基線）
│   ├── Curio_FlightController.ino      # 所有任務在 Core 0 執行
│   ├── config.h
│   └── src/
│       ├── bmi088_driver.h/.cpp
│       └── bmp580_driver.h/.cpp
├── Curio_FlightController_DualCore/    # 雙核版本（RP2354A 特性展示）
│   ├── Curio_FlightController_DualCore.ino  # Core 0 = 飛控，Core 1 = CRSF
│   ├── config.h
│   └── src/
│       ├── bmi088_driver.h/.cpp
│       └── bmp580_driver.h/.cpp
├── test_IMU/
│   └── test_IMU.ino
├── test_ELRS/
│   └── test_ELRS.ino
├── test_Barometer/
│   └── test_Barometer.ino
└── README.md
```

### 單核版本 vs. 雙核版本架構對比

RP2354A 相較於 Crazyflie 所採用 STM32F405 最重要的架構優勢之一，在於其**兩顆獨立的 Cortex-M33 核心**。提供兩套完整韌體，讓此優勢不僅可以被理解，更可以被**量化基準測試**：

| | `Curio_FlightController`（單核） | `Curio_FlightController_DualCore`（雙核） |
|---|---|---|
| **Core 0** | IMU → Madgwick → PID → PWM **+ CRSF 解析** | 純飛控：IMU → Madgwick → PID → PWM |
| **Core 1** | 閒置（未使用） | CRSF 解析器（緊湊迴圈，16 頻道全解碼） |
| **核間通訊** | 無 | 揮發性 `CRSFShared` 結構（32-bit 原子讀寫） |
| **CRSF 對 Core 0 的時序影響** | 存在——UART 緩衝位元組數隨時間變動，引入非確定性延遲 | 零——Core 1 完全擁有 UART，Core 0 不接觸 |

#### 為什麼這很重要？

在單核版本中，`parseCRSF()` 在 `loop()` 裡由 Core 0 執行。每次迭代，Core 0 必須清空自上次迭代以來在 420 kBaud UART FIFO 中積累的所有位元組。由於 CRSF 幀為 26 位元組且到達時間不規律，等待的位元組數每次不同——有時為零，有時為完整一幀。這造成控制迴圈的**執行時間不確定**，表現為迴圈抖動（Jitter）。

在雙核版本中，Core 0 從不觸碰 UART。它呼叫 `snapshotChannels()`——對揮發性結構的簡單讀取——取代了 UART 解析迴圈。Core 1 在獨立的緊湊迴圈中無節制地處理所有 CRSF 位元組工作。Core 0 的執行時間完全由控制運算決定，具有**確定性**。

#### 雙核核間通訊設計

```
Core 1（CRSF 解析器）              Core 0（飛控迴圈）
──────────────────────            ─────────────────────────
loop1() {                         loop() {
  c1_parseCRSF();  ──寫入──►       snapshotChannels();
  // 更新：                          // 讀取：
  crsfShared.ch1                    channel_1_pwm = crsfShared.ch1
  crsfShared.ch2                    channel_2_pwm = crsfShared.ch2
  crsfShared.ch3                    ...
  crsfShared.lastFrameMs            失效保護檢查
}                                 }
```

`CRSFShared` 結構中每個欄位均為 32-bit 對齊的 `volatile uint32_t`。在 Cortex-M33 上，32-bit 對齊字存取是原子操作，無需互斥鎖。最壞情況是 Core 0 讀取到比當前晚一幀（≈6.7 ms，ELRS 150 Hz）的數值，對遙控頻道而言可忽略不計。

#### 基準測試流程

兩個版本均內建 `BENCHMARK_MODE`。啟用後，韌體每秒透過 USB 序列埠印出 Core 0 迴圈時序統計。**關鍵指標為 Jitter = Max_dt − Min_dt（µs）。**

**第一步——單核版本基線量測**
1. 開啟 `Curio_FlightController/Curio_FlightController.ino`。
2. 取消 `#define BENCHMARK_MODE` 的註解。
3. 燒錄，連接 ELRS 接收機並開啟發射機，以 500000 baud 開啟序列監視器。
4. 記錄印出的 Jitter 數值。範例輸出：
   ```
   ──── Core 0 Loop Benchmark [SINGLE-CORE] ────
     Iterations :  2000
     Mean dt    :  498 µs
     Min  dt    :  490 µs
     Max  dt    :  563 µs
     Jitter     :   73 µs  ← 與雙核版本比較
     (CRSF parsing runs on Core 0 — inflates jitter)
   ```

**第二步——雙核版本比較量測**
1. 開啟 `Curio_FlightController_DualCore/Curio_FlightController_DualCore.ino`。
2. `#define BENCHMARK_MODE` 預設已啟用。
3. 燒錄（相同接收機與發射機配置），觀察 Jitter 數值。範例輸出：
   ```
   ──── Core 0 Loop Benchmark (1-second window) ────
     Iterations :  2000
     Mean dt    :  494 µs
     Min  dt    :  492 µs
     Max  dt    :  497 µs
     Jitter     :    5 µs  ← 關鍵指標
     Core1 frames:  150  CRC errors: 0
   ```

Jitter 的大幅降低，直接展示了 RP2354A 雙核硬體隔離帶來的確定性提升——這正是其相較於單核飛控板（如 STM32F405）在飛控教育與研究場景中最具說服力的架構優勢。

### 軟體架構

韌體完整對應 dRehmFlight 控制架構，針對 Curio 硬體重新實作：

```
2 kHz 主迴圈 — Core 0（確定性執行，無 OS 介入）
│
├─ getIMUdata()          — 從 2 kHz DRDY ISR 取 BMI088 原子快照
│                          套用偏差補償 + 一階低通濾波
│
├─ Madgwick6DOF()        — 6 自由度四元數感測器融合
│                          → Roll, Pitch, Yaw（度）
│
├─ getDesState()         — CRSF µs 值 → 物理設定點
│                          （油門 0–1，橫滾/俯仰 ±30°，偏航 ±160 °/s）
│
├─ updateAltHold()       — 50 Hz 氣壓外迴路（高度誤差 → 爬升率設定點）
│   (USE_ALT_HOLD)         + 爬升率內迴路 PID → 油門補償量
│
├─ controlANGLE()        — 單迴路角度 PID              （建議初學者使用）
│  controlANGLE2()       — 串聯角度 → 角速率 PID        （效能更佳，較難調整）
│  controlRATE()         — 直接角速率 / Acro 模式
│
├─ controlMixer()        — X 型四旋翼混控
│                          M1(前左)  M2(前右)  M3(後右)  M4(後左)
│
├─ scaleCommands()       — 正規化 0–1 → 10-bit PWM（0–1023）
│
├─ throttleCut()         — CH5 > 1500 µs → 立即解除武裝，所有馬達歸零
│
├─ commandMotors()       — analogWrite() 輸出至四個 MOSFET 馬達腳位
│
└─ getCommands()         — parseCRSF() + CRSF 逾時失效保護

Core 1（未來擴充）
└─ ELRS / MAVLink / 飛行日誌 — 通訊任務與飛控迴圈硬體隔離
```

### 使用流程

#### 第一步：安裝開發環境

1. 安裝 [Arduino IDE 2.3.x](https://www.arduino.cc/en/software)。
2. 開啟「偏好設定 → 額外的板子管理員網址」，加入：
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
3. 至「板子管理員」搜尋並安裝 **Raspberry Pi Pico/RP2040**。
4. 選擇開發板：**「Raspberry Pi Pico 2」**，CPU 頻率：**150 MHz**。

#### 第二步：感測器功能驗證（飛行前必做）

在上傳主飛控程式前，依序完成以下三項測試，確認硬體接線正確，並取得 IMU 校正值。

**A — IMU 測試（`test_IMU/test_IMU.ino`）**
1. 燒錄後以 **500000 baud** 開啟序列監視器。
2. 晶片 ID 確認自動執行。預期輸出：
   ```
   [BMI088] ACC chip ID: 0x1E  (expect 0x1E)   GYR chip ID: 0x0F  (expect 0x0F)
   [OK]  BMI088 found and configured.
   ```
3. 輸入 `c` 執行偏差校正（板子需**水平靜止**）。
4. 將印出的六個數值填入 `Curio_FlightController.ino`：
   ```cpp
   float AccErrorX  = /* 印出值 */;
   float AccErrorY  = /* 印出值 */;
   float AccErrorZ  = /* 印出值 */;
   float GyroErrorX = /* 印出值 */;
   float GyroErrorY = /* 印出值 */;
   float GyroErrorZ = /* 印出值 */;
   ```
5. 輸入 `s` 串流即時數據，確認板子水平時 `Az ≈ +1 g`、`Ax ≈ Ay ≈ 0`；傾斜板子時 Roll/Pitch 角度正確跟隨。

**B — ELRS 接收機測試（`test_ELRS/test_ELRS.ino`）**
1. 燒錄後開啟遙控發射機，確認 ELRS 鏈路 LED 常亮。
2. 以 **500000 baud** 開啟序列監視器。
3. 預期：16 個頻道以長條圖呈現，10 Hz 更新。
4. 撥動所有搖桿與開關，確認頻道對應符合下表。

**C — 氣壓計測試（`test_Barometer/test_Barometer.ino`）**
1. 燒錄後以 **500000 baud** 開啟序列監視器。
2. 輸入 `n` 執行雜訊測試，室內預期 1-σ 高度標準差 **< 0.30 m**。
3. 輸入 `s` 確認氣壓讀值合理（海平面附近約 101 000 Pa）。

#### 第三步：遙控器頻道對應

| 頻道 | 功能 | 範圍 |
|---|---|---|
| CH1 | 油門（Throttle） | 1000（最低）– 2000（最高）µs |
| CH2 | 橫滾（Roll／副翼） | 1000 – 2000，中立 1500 µs |
| CH3 | 俯仰（Pitch／升降） | 1000 – 2000，中立 1500 µs |
| CH4 | 偏航（Yaw／方向） | 1000 – 2000，中立 1500 µs |
| CH5 | 解鎖開關 | < 1500 = 解鎖，> 1500 = 鎖定 |
| CH6 | 定高開關 | > 1500 = 定高模式開啟 |

#### 第四步：解鎖程序

1. 油門搖桿**完全拉低**（CH1 < 1060 µs）。
2. 解鎖開關撥至**解鎖位置**（CH5 < 1500 µs）。
3. Curio 解鎖；推油門馬達即轉動。
4. 隨時將 CH5 撥至鎖定位置（> 1500 µs）可立即**緊急停機**。

#### 第五步：PID 調整

所有增益定義於 `config.h`。建議從**角度模式**（`USE_CONTROL_ANGLE`，預設）開始：

| 參數 | 預設值 | 說明 |
|---|---|---|
| `KP_ROLL_ANGLE` | 0.2 | 增大可提升橫滾響應靈敏度 |
| `KI_ROLL_ANGLE` | 0.3 | 消除穩態傾斜誤差 |
| `KD_ROLL_ANGLE` | 0.05 | 抑制振盪，謹慎增大 |
| `B_MADGWICK` | 0.04 | 降低 → 偏向陀螺儀主導，姿態估測較平滑但較慢 |
| `B_ACCEL` | 0.14 | 加速度計低通濾波係數 |
| `B_GYRO` | 0.10 | 陀螺儀低通濾波係數 |

角度模式穩定後，可切換至 `USE_CONTROL_ANGLE2`（串聯外迴路角度 + 內迴路角速率 PID）以獲得更佳動態響應。速率模式（`USE_CONTROL_RATE`）適用於 Acro／手動飛行。

#### 學習路徑建議

| 階段 | 學習目標 | 建議切入點 |
|---|---|---|
| 入門 | 理解 PID 迴路，調出穩定懸停 | `controlANGLE()`，搭配序列除錯輸出 |
| 進階 | 感測器融合、Madgwick 濾波器原理 | 調整 `B_madgwick`，觀察四元數漂移 |
| 深入 | 串聯控制、定高迴路 | `controlANGLE2()`、`USE_ALT_HOLD` |
| 研究 | 幾何非線性控制、殘差 RL、板載 MPC | 將演算法移植至 Core 0，通訊任務交給 Core 1 |

### 安全警告

> **在所有軟體開發與 PID 調整過程中，請務必移除螺旋槳（無槳測試）。**
> 有刷馬達在解鎖且油門高於最低值後會立即轉動。
> 拿取或維修機體前，請確認已完全鎖定。

### 授權

MIT License。韌體架構源自 [dRehmFlight](https://github.com/nickrehm/dRehmFlight)（作者：Nicholas Rehm，MIT License）。Scout 參考：[TimHanewich/scout](https://github.com/TimHanewich/scout)。

---

*Developed by RocketBird Makers' Depot（火箭鳥創客倉庫）.*
