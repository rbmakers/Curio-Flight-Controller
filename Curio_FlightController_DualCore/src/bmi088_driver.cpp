// ================================================================
//  bmi088_driver.cpp  — BMI088 SPI Interrupt-Driven Driver
//  RB-RP2354A Flight Controller , 開發 : 火箭鳥創客倉庫
//
//  MODIFICATION LOG (vs. original bmi088_driver.cpp)
//  ---------------------------------------------------
//  [FIX-1] IRAM_ATTR removed from ISR; replaced with
//          __not_in_flash_func().
//          IRAM_ATTR is an ESP32/ESP-IDF macro and is meaningless
//          (or undefined) on RP2350/arduino-pico. The correct
//          RP2040/RP2350 idiom to force an ISR into SRAM is the
//          __not_in_flash_func() wrapper. This ensures the ISR
//          runs from SRAM and is never stalled by XIP flash cache
//          misses during DMA or background flash operations.
//
//  [FIX-2] CS deselect guard time added (delayMicroseconds(2)).
//          The BMI088 datasheet requires a minimum CS-deassert hold
//          time (t_CSB_desel) of >= 2 us for the gyro and >= 500 ns
//          for the accel. At 10 MHz SPI with software GPIO, back-to-
//          back transactions violated this requirement, causing
//          occasional corrupt reads. The confirmed-working RocketBird
//          library uses 10 us between every transaction; we use 2 us
//          as the minimum safe value while keeping ISR latency low.
//
//  [FIX-3] SPI clock reduced from 10 MHz to 4 MHz.
//          The BMI088 specifies 10 MHz max, but a custom PCB with
//          longer SPI traces and pad capacitance typically degrades
//          signal integrity above ~4-6 MHz. The confirmed-working
//          code on the XIAO RP2350 used 1 MHz. We choose 4 MHz as a
//          balance between reliability and ISR execution time.
//
//  [FIX-4] Gyro soft-reset added at the start of gyro init.
//          The confirmed-working library resets the gyro (reg 0x14,
//          value 0xB6) before writing any configuration. Without
//          this, the gyro can retain a partially-written register
//          state from a previous boot (especially after a watchdog
//          reset or power glitch), leaving INT3_INT4_IO_MAP or
//          INT_CTRL in a bad state so that DRDY never asserts and
//          the ISR never fires. 100 ms delay follows per datasheet.
//
//  [FIX-5] Accel enable (PWR_CTRL) delay increased from 10 ms to
//          50 ms. The BMI088 accelerometer datasheet (section 4.4)
//          specifies a start-up time of up to 50 ms after the
//          accelerometer is enabled (ACC_PWR_CTRL = 0x04). At 10 ms
//          the data path may not be fully settled, causing the first
//          accel reads inside the ISR to return invalid data.
//
//  [FIX-6] CS pins driven HIGH BEFORE SPI.begin().
//          If CS pins are still in INPUT (high-Z) state when the SPI
//          peripheral is powered on, the CS line can float to an
//          indeterminate voltage. The BMI088 accel latches its
//          interface selection (SPI vs I2C) on the first falling edge
//          of CS. A spurious LOW during SPI.begin() permanently locks
//          the accel into I2C mode until the next power cycle.
//          CS pins are now driven HIGH before SPI.begin() is called.
// ================================================================

#include "bmi088_driver.h"
#include "../config.h"
#include <SPI.h>

// ================================================================
// BMI088 Register Map
// ================================================================

// --- Accelerometer (separate die — requires dummy byte on SPI read)
#define ACC_REG_CHIP_ID    0x00   // Expected: 0x1E
#define ACC_REG_DATA       0x12   // 6 bytes: X_LSB X_MSB Y_LSB Y_MSB Z_LSB Z_MSB
#define ACC_REG_CONF       0x40   // [6:4]=BWP  [3:0]=ODR
#define ACC_REG_RANGE      0x41   // 0x00=3g 0x01=6g 0x02=12g 0x03=24g
#define ACC_REG_PWR_CONF   0x7C   // 0x00=active  0x03=suspend
#define ACC_REG_PWR_CTRL   0x7D   // 0x04=on  0x00=off
#define ACC_REG_SOFTRESET  0x7E   // write 0xB6

// Accel ODR + BWP encoding
// BWP[6:4]: 0x0=OSR4  0x1=OSR2  0x2=normal
// ODR[3:0]: 0x05=12.5 Hz ... 0x0C=1600 Hz
#define ACC_CONF_1600HZ_NORMAL  0x2C   // BWP=normal(2<<4) | ODR=1600 Hz(0x0C)
#define ACC_RANGE_24G           0x03

// --- Gyroscope (separate die — standard SPI, no dummy byte needed)
#define GYR_REG_CHIP_ID    0x00   // Expected: 0x0F
#define GYR_REG_DATA       0x02   // 6 bytes: X_LSB X_MSB Y_LSB Y_MSB Z_LSB Z_MSB
#define GYR_REG_RANGE      0x0F   // 0x00=2000 dps ... 0x04=125 dps
#define GYR_REG_BANDWIDTH  0x10   // 0x00=2000 Hz ODR/532 Hz BW ... 0x07=100 Hz/32 Hz
#define GYR_REG_LPM1       0x11   // 0x00=normal power mode
#define GYR_REG_SOFTRESET  0x14   // write 0xB6 to soft-reset  [FIX-4]
#define GYR_REG_INT_CTRL   0x15   // bit7=1 enables DRDY interrupt output
#define GYR_REG_INT34_CONF 0x16   // INT3: bit1=LVL(1=active-high)  bit0=OD(0=push-pull)
#define GYR_REG_INT34_MAP  0x18   // bit0=1 routes DRDY to INT3 pin

// Gyro configuration values — 2000 Hz ODR, 532 Hz filter bandwidth
#define GYR_BW_2000HZ            0x00
#define GYR_RANGE_2000           0x00
#define GYR_INT3_ACTIVE_HIGH_PP  0x02   // push-pull, active high
#define GYR_INT3_MAP_DRDY        0x01

// [FIX-2] Minimum CS deselect hold time.
// Gyro datasheet requires t_CSB_desel >= 2 us.
// Accel requires >= 500 ns. Using 2 us for both (safe for ISR).
#define CS_DESELECT_US  2

// ================================================================
// Module state
// ================================================================

// [FIX-3] SPI clock 4 MHz (was 10 MHz in config.h SPI_CLOCK_HZ).
// 4 MHz is well within the BMI088 spec while accommodating real-world
// PCB trace parasitics on the RB-RP2354A custom board.
static SPISettings spiSettings(4000000UL, MSBFIRST, SPI_MODE0);

static volatile IMURaw imuLatest;

// ================================================================
// Internal SPI helpers
// Every helper adds a CS deselect guard after the transaction. [FIX-2]
// ================================================================

static void accelWriteReg(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_ACCEL, LOW);
    SPI.transfer(reg & 0x7F);   // write: MSB = 0
    SPI.transfer(val);
    digitalWrite(CS_ACCEL, HIGH);
    SPI.endTransaction();
    delayMicroseconds(CS_DESELECT_US);         // [FIX-2]
}

static void accelReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_ACCEL, LOW);
    SPI.transfer(reg | 0x80);   // read: MSB = 1
    SPI.transfer(0x00);          // BMI088 accel mandatory dummy byte
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_ACCEL, HIGH);
    SPI.endTransaction();
    delayMicroseconds(CS_DESELECT_US);         // [FIX-2]
}

static uint8_t accelReadReg(uint8_t reg)
{
    uint8_t val;
    accelReadBurst(reg, &val, 1);
    return val;
}

static void gyroWriteReg(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_GYRO, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(CS_GYRO, HIGH);
    SPI.endTransaction();
    delayMicroseconds(CS_DESELECT_US);         // [FIX-2]
}

static void gyroReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_GYRO, LOW);
    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_GYRO, HIGH);
    SPI.endTransaction();
    delayMicroseconds(CS_DESELECT_US);         // [FIX-2]
}

// ================================================================
// ISR — triggered on every Gyro DRDY rising edge (2 kHz)
//
// [FIX-1] __not_in_flash_func() replaces IRAM_ATTR.
//         This is the arduino-pico (RP2040/RP2350) mechanism to
//         place a function in SRAM. It prevents XIP cache stalls
//         that could otherwise cause jitter or missed interrupts.
//
// Timing budget @ 2 kHz: 500 us per call.
// SPI cost @ 4 MHz:
//   Gyro  6-byte burst + addr   : ~16 us + overhead ~ 20 us
//   Accel 6-byte burst + 2 bytes: ~18 us + overhead ~ 22 us (1/10 calls)
// Worst case (with accel): ~42 us — well within the 500 us budget.
// ================================================================

static void __not_in_flash_func(gyroISR)()    // [FIX-1]
{
    uint8_t g[6];
    static uint8_t accelDiv = 0;

    // --- Gyro read (every ISR call = 2 kHz) ---
    gyroReadBurst(GYR_REG_DATA, g, 6);

    int16_t gxRaw = (int16_t)((g[1] << 8) | g[0]);
    int16_t gyRaw = (int16_t)((g[3] << 8) | g[2]);
    int16_t gzRaw = (int16_t)((g[5] << 8) | g[4]);

    imuLatest.gx = gxRaw * GYRO_SCALE;
    imuLatest.gy = gyRaw * GYRO_SCALE;
    imuLatest.gz = gzRaw * GYRO_SCALE;

    // --- Accel read every 10th ISR call = 200 Hz ---
    if (++accelDiv >= 10)
    {
        accelDiv = 0;
        uint8_t a[6];
        accelReadBurst(ACC_REG_DATA, a, 6);

        int16_t axRaw = (int16_t)((a[1] << 8) | a[0]);
        int16_t ayRaw = (int16_t)((a[3] << 8) | a[2]);
        int16_t azRaw = (int16_t)((a[5] << 8) | a[4]);

        imuLatest.ax = axRaw * ACCEL_SCALE;
        imuLatest.ay = ayRaw * ACCEL_SCALE;
        imuLatest.az = azRaw * ACCEL_SCALE;
    }

    imuLatest.timestamp = micros();
}

// ================================================================
// Public API
// ================================================================

bool bmi088CheckIDs()
{
    // The first accel SPI read after power-on always returns 0x00;
    // perform a dummy read then wait before the real read.
    accelReadReg(ACC_REG_CHIP_ID);
    delayMicroseconds(500);
    uint8_t accID = accelReadReg(ACC_REG_CHIP_ID);

    uint8_t buf;
    gyroReadBurst(GYR_REG_CHIP_ID, &buf, 1);
    uint8_t gyrID = buf;

    Serial.print("[BMI088] ACC chip ID: 0x");
    Serial.print(accID, HEX);
    Serial.print("  (expect 0x1E)   GYR chip ID: 0x");
    Serial.print(gyrID, HEX);
    Serial.println("  (expect 0x0F)");

    return (accID == 0x1E) && (gyrID == 0x0F);
}

void bmi088Init()
{
    // ==============================================================
    // [FIX-6] Drive CS pins HIGH BEFORE starting the SPI peripheral.
    //
    // The BMI088 accel selects its communication interface (SPI vs
    // I2C) on the very first falling edge it sees on its CS pin.
    // Driving CS HIGH here — before SPI.begin() toggles internal
    // clocks — prevents any glitch from accidentally latching the
    // accel into I2C mode, which would make it invisible on SPI for
    // the remainder of the power cycle.
    // ==============================================================
    pinMode(CS_ACCEL, OUTPUT); digitalWrite(CS_ACCEL, HIGH);
    pinMode(CS_GYRO,  OUTPUT); digitalWrite(CS_GYRO,  HIGH);
    delay(5);   // Let GPIO drive levels settle before SPI clocks start

    // ---- SPI bus: assign custom pins, then begin ----------------
    SPI.setTX(PIN_MOSI);
    SPI.setRX(PIN_MISO);
    SPI.setSCK(PIN_SCK);
    SPI.begin();

    pinMode(GYRO_INT_PIN, INPUT);
    delay(10);

    // ==============================================================
    // ACCELEROMETER INIT
    // ==============================================================

    // 1. Soft reset — returns all registers to power-on defaults
    accelWriteReg(ACC_REG_SOFTRESET, 0xB6);
    delay(50);   // Datasheet: >= 1 ms; 50 ms ensures full NVM reload

    // 2. Dummy SPI read to switch the accel from I2C-detect mode
    //    to SPI mode permanently (required after every reset).
    accelReadReg(ACC_REG_CHIP_ID);
    delay(5);

    // 3. Exit suspend → active mode
    accelWriteReg(ACC_REG_PWR_CONF, 0x00);   // 0x00 = active
    delay(10);

    // 4. Enable accelerometer measurement engine
    accelWriteReg(ACC_REG_PWR_CTRL, 0x04);   // 0x04 = acc_enable
    // [FIX-5] 50 ms wait (was 10 ms). Datasheet section 4.4:
    // accelerometer data path start-up time is up to 50 ms.
    delay(50);                                // [FIX-5]

    // 5. ODR = 1600 Hz, BWP = normal (no oversampling)
    //    0x2C = (0x2 << 4) | 0x0C
    //    Feeds the ISR-based 200 Hz accel subsample with headroom.
    accelWriteReg(ACC_REG_CONF, ACC_CONF_1600HZ_NORMAL);
    delay(2);

    // 6. Measurement range = +/- 24 g
    //    Scale factor: ACCEL_SCALE = 1/1365.33 g/LSB (see config.h)
    accelWriteReg(ACC_REG_RANGE, ACC_RANGE_24G);
    delay(5);

    // ==============================================================
    // GYROSCOPE INIT
    // ==============================================================

    // [FIX-4] 1. Gyro soft reset BEFORE writing any configuration.
    //
    //    Without this reset, a previous partial boot can leave the
    //    interrupt mapping registers (INT3_INT4_IO_MAP, INT_CTRL) in
    //    an undefined state so that the DRDY signal on INT3 never
    //    asserts, meaning the ISR never fires and gyro data is never
    //    updated. The confirmed-working RocketBird library (BMI088_SPI)
    //    always resets the gyro as the first init step.
    //
    //    Register 0x14 (GYR_SOFTRESET), value 0xB6.
    gyroWriteReg(GYR_REG_SOFTRESET, 0xB6);   // [FIX-4]
    delay(100);   // Datasheet: >= 30 ms after gyro reset; 100 ms for safety

    // 2. Range = +/- 2000 dps
    //    GYRO_SCALE = 1/16.384 (deg/s)/LSB (see config.h)
    gyroWriteReg(GYR_REG_RANGE, GYR_RANGE_2000);
    delay(2);

    // 3. ODR = 2000 Hz, filter BW = 532 Hz
    //    0x00 required for 2 kHz DRDY interrupt operation.
    gyroWriteReg(GYR_REG_BANDWIDTH, GYR_BW_2000HZ);
    delay(2);

    // 4. Normal power mode (not deep-suspend)
    gyroWriteReg(GYR_REG_LPM1, 0x00);
    delay(5);

    // 5. INT3 pin: push-pull output, active-high level
    //    0x02 = bit1(LVL)=1  bit0(OD)=0
    gyroWriteReg(GYR_REG_INT34_CONF, GYR_INT3_ACTIVE_HIGH_PP);
    delay(2);

    // 6. Route DRDY event to INT3 pin
    //    0x01 = INT3_DRDY bit in INT3_INT4_IO_MAP
    gyroWriteReg(GYR_REG_INT34_MAP, GYR_INT3_MAP_DRDY);
    delay(2);

    // 7. Unmask DRDY interrupt in the interrupt controller
    //    0x80 = int_en_1 bit (data-ready interrupt enable)
    gyroWriteReg(GYR_REG_INT_CTRL, 0x80);
    delay(5);

    // ---- Attach RP2354A GPIO interrupt --------------------------
    // GPIO24 (GYRO_INT_PIN) is wired to BMI088 INT3 (gyro DRDY).
    // The ISR fires on every rising edge at 2000 Hz.
    attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), gyroISR, RISING);
}

IMURaw bmi088GetLatest()
{
    // Atomic copy: briefly disable interrupts to prevent a torn read
    // of the volatile struct on Cortex-M33 (non-atomic multi-word copy).
    noInterrupts();
    IMURaw copy = imuLatest;
    interrupts();
    return copy;
}
