#pragma once

// ================================================================
//  RB-RP2354A Flight Controller — Hardware & Tuning Configuration
//  Board: RB-RP2354A (RP2354A / Cortex-M33, 150 MHz)
//  IMU  : Bosch BMI088 (SPI, interrupt-driven @ 2 kHz)
//  Baro : Bosch BMP580 (I²C, 50 Hz)
//  RC   : ExpressLRS CRSF via UART0 (420 kBaud, 16 ch)
//  Motor: 4× brushed MOSFET, 32 kHz 10-bit PWM
// ================================================================

// ----------------------------------------------------------------
// SPI — BMI088
// ----------------------------------------------------------------
#define PIN_MOSI        19
#define PIN_MISO        20
#define PIN_SCK         22
#define CS_ACCEL        21   // BMI088 accelerometer chip-select
#define CS_GYRO         23   // BMI088 gyroscope chip-select
#define GYRO_INT_PIN    24   // Gyro DRDY → rising-edge interrupt
// NOTE [FIX-3]: The SPI clock is set to 4 MHz inside bmi088_driver.cpp
// (SPISettings 4000000UL) overriding this value. 4 MHz was chosen after
// the confirmed-working RocketBird BMI088_SPI library operated at 1 MHz
// on the XIAO RP2350. Custom-PCB trace parasitics make 10 MHz unreliable.
// Change the SPISettings value in bmi088_driver.cpp — not here.
#define SPI_CLOCK_HZ    4000000UL    // 4 MHz (see bmi088_driver.cpp [FIX-3])

// ----------------------------------------------------------------
// I²C — BMP580
// ----------------------------------------------------------------
#define PIN_SDA         16
#define PIN_SCL         17
#define BMP580_I2C_ADDR 0x47   // SDO pin tied to VDD

// ----------------------------------------------------------------
// ELRS / CRSF — UART0
// ----------------------------------------------------------------
#define ELRS_SERIAL     Serial1
#define ELRS_BAUD       420000
#define PIN_ELRS_TX     12   // UART0 TX
#define PIN_ELRS_RX     13   // UART0 RX

// ----------------------------------------------------------------
// Brushed motors — 32 kHz, 10-bit PWM
// Motor layout (top-down, props-off view, NED frame):
//   M1 = Front-Left   M2 = Front-Right
//   M3 = Rear-Right   M4 = Rear-Left
// ----------------------------------------------------------------
#define PIN_M1          29
#define PIN_M2          11
#define PIN_M3          18
#define PIN_M4          25
#define MOTOR_PWM_FREQ  32000
#define MOTOR_PWM_BITS  10
#define MOTOR_PWM_MAX   1023   // 2^10 - 1 = full duty
#define MOTOR_ARM_MIN   30     // Minimum armed duty (keeps FETs lightly on)

// ----------------------------------------------------------------
// Battery voltage sensing
// ----------------------------------------------------------------
#define PIN_VBAT        27   // ADC1; voltage divider 100 kΩ / 10 kΩ
#define VBAT_R_UPPER    100000.0f
#define VBAT_R_LOWER    10000.0f
#define VBAT_ADC_REF    3.3f
#define VBAT_ADC_FULL   4095.0f  // 12-bit ADC
#define VBAT_SCALE      (VBAT_ADC_REF / VBAT_ADC_FULL * \
                         (VBAT_R_UPPER + VBAT_R_LOWER) / VBAT_R_LOWER)
#define VBAT_LOW_WARN   3.5f     // Per-cell low-voltage warning threshold (V)

// ----------------------------------------------------------------
// Onboard LED
// ----------------------------------------------------------------
#ifndef PIN_LED
  #define PIN_LED       LED_BUILTIN
#endif

// ================================================================
// BMI088 Scale Factors
// ================================================================
// Gyro at ±2000 dps → sensitivity = 32768 / 2000 = 16.384 LSB/(°/s)
// Output unit expected by Madgwick: degrees/second
#define GYRO_SCALE      (1.0f / 16.384f)     // raw int16 → °/s

// Accel at ±24 g → sensitivity = 32768 / 24 ≈ 1365.33 LSB/g
// Output unit expected by Madgwick and PID: g
#define ACCEL_SCALE     (1.0f / 1365.33f)    // raw int16 → g

// ================================================================
// Control Loop Rates
// ================================================================
#define LOOP_RATE_HZ    2000   // Attitude + rate PID inner loop
#define BARO_RATE_HZ    50     // Barometer read, altitude hold outer loop

// ================================================================
// Madgwick / Low-Pass Filter Parameters
// (tuned for LOOP_RATE_HZ = 2000 Hz)
// ================================================================
#define B_MADGWICK      0.04f  // Madgwick β: higher → noisier but faster
#define B_ACCEL         0.14f  // Accel first-order LP coefficient
#define B_GYRO          0.10f  // Gyro  first-order LP coefficient

// ================================================================
// PID Gains — Angle Mode  (controlANGLE)
// ================================================================
#define KP_ROLL_ANGLE   0.2f
#define KI_ROLL_ANGLE   0.3f
#define KD_ROLL_ANGLE   0.05f

#define KP_PITCH_ANGLE  0.2f
#define KI_PITCH_ANGLE  0.3f
#define KD_PITCH_ANGLE  0.05f

// Damping for controlANGLE2 cascaded outer→inner handoff
#define B_LOOP_ROLL     0.9f   // 0 = max damping, 1 = no damping
#define B_LOOP_PITCH    0.9f

// ================================================================
// PID Gains — Rate Mode inner loop  (controlANGLE2 / controlRATE)
// ================================================================
#define KP_ROLL_RATE    0.15f
#define KI_ROLL_RATE    0.20f
#define KD_ROLL_RATE    0.0002f

#define KP_PITCH_RATE   0.15f
#define KI_PITCH_RATE   0.20f
#define KD_PITCH_RATE   0.0002f

// ================================================================
// PID Gains — Yaw Rate
// ================================================================
#define KP_YAW          0.30f
#define KI_YAW          0.05f
#define KD_YAW          0.00015f

// ================================================================
// PID Gains — Altitude Hold (50 Hz outer loop)
// ================================================================
// Outer: altitude error → desired climb rate (m/s)
#define KP_ALT          2.0f
// Inner: climb-rate error → throttle delta (normalized 0–1)
#define KP_CLIMB        0.30f
#define KI_CLIMB        0.05f
#define KD_CLIMB        0.01f

// ================================================================
// Control Limits
// ================================================================
#define I_LIMIT         25.0f   // Integrator saturation (deg or deg/s)
#define MAX_ROLL        30.0f   // Max commanded roll angle (°) in angle mode
#define MAX_PITCH       30.0f   // Max commanded pitch angle (°) in angle mode
#define MAX_YAW         160.0f  // Max commanded yaw rate (°/s)
#define MAX_CLIMB_RATE  1.5f    // Max altitude-hold climb rate (m/s)
#define ALT_HOLD_THR_CENTER 1500UL  // RC throttle µs that corresponds to "hold"

// ================================================================
// RC Failsafe Values (µs, CRSF mapped to 1000–2000 range)
// ================================================================
#define CH1_FS  1000   // Throttle — minimum (cut)
#define CH2_FS  1500   // Roll     — center
#define CH3_FS  1500   // Pitch    — center
#define CH4_FS  1500   // Yaw      — center
#define CH5_FS  2000   // Arm switch — disarmed (>1500)
#define CH6_FS  1000   // Aux1 (alt-hold off)

// Failsafe: treat as lost-link if no CRSF frame arrives within this many ms
#define CRSF_TIMEOUT_MS 500
