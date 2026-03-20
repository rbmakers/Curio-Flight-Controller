#pragma once
#include <Arduino.h>

// ================================================================
//  bmi088_driver.h  — BMI088 IMU Driver (SPI, interrupt-driven)
//  RB-RP2354A Flight Controller
//
//  Hardware connections (RB-RP2354A board pinout):
//    MOSI = GPIO19    MISO = GPIO20    SCK  = GPIO22
//    ACC_CS  = GPIO21 (accelerometer chip-select)
//    GYR_CS  = GPIO23 (gyroscope chip-select)
//    GYRO_INT3 → GPIO24 (rising-edge DRDY interrupt, 2 kHz)
//
//  BMI088 SPI protocol notes:
//    Accel reads: mandatory dummy byte after the register address.
//    Gyro  reads: NO dummy byte (standard SPI).
//
//  Modification notes (vs. original version):
//    [FIX-1] ISR placed in SRAM via __not_in_flash_func() instead of
//            the ESP32-only IRAM_ATTR macro.
//    [FIX-2] CS deselect guard time added in every SPI transaction.
//    [FIX-3] SPI clock reduced from 10 MHz to 4 MHz.
//    [FIX-4] Gyro soft reset added to init sequence.
//    [FIX-5] Accel enable delay increased to 50 ms.
//    [FIX-6] CS pins driven HIGH before SPI.begin().
// ================================================================

// ----------------------------------------------------------------
// Public data type
// ----------------------------------------------------------------
typedef struct
{
    volatile float    gx, gy, gz;   // degrees/second  (2000 dps full-scale)
    volatile float    ax, ay, az;   // g               (+/- 24 g full-scale)
    volatile uint32_t timestamp;    // micros() at last ISR update
} IMURaw;

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

// Initialize SPI bus, configure both BMI088 dies, attach DRDY ISR.
// Call once in setup() BEFORE bmi088CheckIDs().
void bmi088Init();

// Returns an atomic snapshot copy of the latest ISR data.
// Safe to call from the main loop; disables interrupts briefly.
IMURaw bmi088GetLatest();

// Read chip IDs over SPI and print result to Serial.
// Returns true if ACC=0x1E and GYR=0x0F.
// Call AFTER bmi088Init().
bool bmi088CheckIDs();
