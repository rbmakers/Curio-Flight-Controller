// ================================================================
//  Curio Flight Controller — DUAL-CORE VERSION
//  RB-RP2354A Board  |  RP2354A (dual Cortex-M33, 150 MHz)
//  開發 : 火箭鳥創客倉庫
//  ARCHITECTURE
//  ─────────────────────────────────────────────────────────────
//  SINGLE-CORE version (Curio_FlightController.ino):
//    Core 0 runs everything:
//      IMU → Madgwick → PID → Mixer → PWM
//      AND parseCRSF() — UART byte-draining inside the 2 kHz loop
//    Problem: CRSF parsing takes variable time (~5–80 µs depending
//    on bytes waiting in UART FIFO), injecting non-deterministic
//    jitter into every control-loop iteration.
//
//  DUAL-CORE version (this file):
//    Core 0 — pure, deterministic control loop @ 2 kHz
//      IMU → Madgwick → PID → Mixer → PWM
//      Reads channel data from a shared volatile struct.
//      NEVER touches Serial1 / CRSF parser.
//
//    Core 1 — dedicated CRSF communication task (tight loop)
//      Drains Serial1 RX buffer continuously at maximum speed.
//      Writes decoded channel µs values to shared volatile struct.
//      Never writes motor outputs.
//
//  INTER-CORE COMMUNICATION
//  ─────────────────────────────────────────────────────────────
//  Channel data is shared via a seqlock-protected struct (CRSFShared).
//
//  Three shared-memory hazards exist and are all addressed:
//
//  [Issue 1] MISSING MEMORY BARRIER — fixed by __DMB()
//    volatile prevents compiler reordering but NOT processor
//    reordering.  Cortex-M33 has a weakly-ordered memory model;
//    Core 1's write buffer can hold stores before they propagate.
//    __DMB() is required to flush and synchronise visibility.
//
//  [Issue 2] NON-ATOMIC MULTI-FIELD SNAPSHOT — fixed by seqlock
//    Each 32-bit store is individually atomic, but a snapshot of
//    all 6 channels is not.  The seqlock sequence counter detects
//    when Core 1 writes mid-read and forces Core 0 to retry.
//
//  [Issue 3] NON-ATOMIC RMW ON COUNTERS — benign, documented
//    frameCount++ and crcErrors++ are load-add-store sequences.
//    Core 1 is the sole writer; Core 0 reads for diagnostics only.
//    A torn diagnostic read has no safety impact.
//
//  See CRSFShared struct and c1_decodeChannels / snapshotChannels
//  for the full seqlock implementation with barrier placement.
//
//  BENCHMARK MODE
//  ─────────────────────────────────────────────────────────────
//  Uncomment #define BENCHMARK_MODE below.
//  Core 0 measures its own loop execution time every iteration
//  and reports, every second over USB Serial:
//    • Mean loop time (µs)
//    • Minimum loop time (µs)   — best-case determinism
//    • Maximum loop time (µs)   — worst-case jitter
//    • Jitter = max − min (µs)  — THE key metric
//
//  Compare identical jitter figures between this file and the
//  single-core version to quantify the dual-core benefit.
//
//  Build environment:
//    Arduino IDE 2.3.x + earlephilhower/arduino-pico ≥ 3.x
//    Board: "Raspberry Pi Pico 2"   CPU Speed: 150 MHz
// ================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include "src/bmi088_driver.h"
#include "src/bmp580_driver.h"

// ================================================================
//  USER SELECTIONS
// ================================================================

// Control law — uncomment exactly one
#define USE_CONTROL_ANGLE    // Single-loop angle PID (recommended)
//#define USE_CONTROL_ANGLE2 // Cascaded angle→rate PID
//#define USE_CONTROL_RATE   // Pure rate / acro mode

// Altitude hold (requires BMP580)
//#define USE_ALT_HOLD

// Benchmark — print Core 0 loop timing statistics every second
#define BENCHMARK_MODE

// ================================================================
//  INTER-CORE SHARED CHANNEL DATA  —  Seqlock-Protected
//
//  Three distinct shared-memory hazards exist in a dual-core design
//  of this type.  All three are addressed by the seqlock below.
//
//  ISSUE 1 — Missing memory barrier (most critical)
//  ─────────────────────────────────────────────────
//  `volatile` prevents the COMPILER from reordering or caching
//  accesses, but it does NOT insert a processor-level memory
//  barrier.  The Cortex-M33 (ARMv8-M) uses a weakly-ordered memory
//  model: Core 1's stores can sit in its write buffer before they
//  propagate to the shared bus matrix, and Core 0 can read SRAM
//  before those stores have committed.
//
//  Concrete failure without a barrier:
//    Core 1 writes ch1..ch6, lastFrameMs  (still in write buffer)
//    Core 0 reads  lastFrameMs = fresh    (already visible)
//    Core 0 reads  ch1..ch6              = STALE  (not yet visible)
//    → Failsafe does not trigger but control loop uses old commands.
//
//  Fix: __DMB() (Data Memory Barrier) in both the writer and reader.
//  __DMB() flushes the write buffer and ensures all preceding stores
//  are globally visible before any subsequent loads execute.
//  Reference: ARM DDI0553B (Cortex-M33 TRM), §B3.4 "Memory ordering"
//  Reference: RP2040/RP2350 datasheet §2.1.2 "Multi-core hazards"
//
//  ISSUE 2 — Non-atomic snapshot across 6 fields (torn read)
//  ─────────────────────────────────────────────────────────
//  Each individual 32-bit store IS atomic (no torn half-words).
//  But the snapshot of all 6 channels together is NOT atomic.
//  Core 1 can be mid-write while Core 0 is mid-read:
//    Core 0 reads ch1 = frame N  (throttle from old frame)
//    Core 1 writes ch2..ch6      (roll/pitch/yaw updated)
//    Core 0 reads ch2..ch6 = frame N+1
//  → Throttle from frame N mixed with attitude from frame N+1.
//  The LP filter absorbs single-frame anomalies, but architecturally
//  this is incorrect and unpredictable.
//
//  ISSUE 3 — Non-atomic read-modify-write on counters (benign)
//  ─────────────────────────────────────────────────────────────
//  frameCount++ and crcErrors++ compile to LDR → ADD → STR.
//  Since Core 1 is the sole writer of both counters and Core 0
//  only reads them for diagnostics, a torn read produces a value
//  that is off by one for one iteration — no safety impact.
//  Documented here; no structural fix required.
//
//  SEQLOCK PATTERN (fixes Issues 1 and 2)
//  ─────────────────────────────────────────────────────────────
//  Core 1 writer:
//    seq++ → odd  (signals "write in progress")
//    __DMB()      (barrier: seq is visible before channel writes)
//    write ch1..ch6, lastFrameMs
//    __DMB()      (barrier: all channel writes visible before seq++)
//    seq++ → even (signals "write complete")
//
//  Core 0 reader (snapshotChannels):
//    do {
//      seq1 = seq;          // sample sequence
//      if (seq1 & 1) continue;  // odd = writer active, spin
//      __DMB();             // acquire barrier: reads happen after seq1
//      read ch1..ch6, lastFrameMs
//      __DMB();             // release barrier: all reads done before seq2
//      seq2 = seq;          // re-sample
//    } while (seq1 != seq2);    // retry if writer changed seq during read
//
//  At 150 Hz CRSF write rate and 2000 Hz Core 0 read rate, the
//  probability of a collision (writer mid-write during Core 0 read)
//  is 150/2000 ≈ 7.5 %.  Each retry takes ~10 cycles.  Average
//  overhead on Core 0 is < 0.4 µs — negligible vs 500 µs budget.
// ================================================================

struct CRSFShared {
    // Seqlock sequence counter.
    // Even = data stable.  Odd = write in progress.
    // SOLE WRITER: Core 1.  Core 0 reads only.
    // seq++ on a single writer is safe even without LDREX/STREX
    // because there is never a write-write race.
    volatile uint32_t seq;

    // Channel data (written atomically in pairs, protected by seq)
    volatile uint32_t ch1;        // Throttle µs
    volatile uint32_t ch2;        // Roll µs
    volatile uint32_t ch3;        // Pitch µs
    volatile uint32_t ch4;        // Yaw µs
    volatile uint32_t ch5;        // Arm switch µs
    volatile uint32_t ch6;        // Aux1 µs
    volatile uint32_t lastFrameMs;

    // Diagnostic counters — benign RMW, single writer (Issue 3)
    volatile uint32_t frameCount;
    volatile uint32_t crcErrors;
};

// Place in SRAM bank 4 (.scratch_x) so the two cores access
// different SRAM banks for their private data, minimising bus
// contention on the shared bus matrix.
// If your linker script does not define .scratch_x, remove the
// attribute — the default .data placement is functionally correct.
static CRSFShared crsfShared __attribute__((section(".scratch_x"))) = {
    .seq = 0,
    .ch1 = CH1_FS, .ch2 = CH2_FS, .ch3 = CH3_FS,
    .ch4 = CH4_FS, .ch5 = CH5_FS, .ch6 = CH6_FS,
    .lastFrameMs = 0, .frameCount = 0, .crcErrors = 0
};

// ================================================================
//  CRSF PARSER  (runs entirely on Core 1)
// ================================================================

#define CRSF_SYNC_BYTE      0xC8
#define CRSF_TYPE_RC_CHAN   0x16
#define CRSF_MAX_FRAME_LEN  64

static uint8_t  c1_crsfBuf[CRSF_MAX_FRAME_LEN];
static uint8_t  c1_crsfIdx  = 0;
static uint8_t  c1_crsfLen  = 0;
static bool     c1_inFrame  = false;

static uint8_t crsfCrc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    return crc;
}

static inline uint32_t crsfToUs(uint16_t raw)
{
    return (uint32_t)constrain((long)raw * 1000L / 1639L + 895L, 1000L, 2000L);
}

// Decode 22-byte RC-channels payload and write to shared struct.
// Uses seqlock write protocol to fix Issues 1 and 2:
//   step 1: seq++ → odd  (signals write in progress to Core 0)
//   step 2: __DMB()      (ensures seq store is globally visible BEFORE
//                         channel stores — prevents Core 0 seeing
//                         channel updates before the odd seq)
//   step 3: write all channel fields
//   step 4: __DMB()      (ensures all channel stores are globally
//                         visible BEFORE the final seq++ — prevents
//                         Core 0 seeing even seq before channels are
//                         updated)
//   step 5: seq++ → even (signals write complete)
static void c1_decodeChannels(const uint8_t *p)
{
    uint16_t ch[16];
    ch[0]  = ((uint16_t)(p[0])       | (uint16_t)(p[1])  << 8) & 0x07FF;
    ch[1]  = ((uint16_t)(p[1])  >> 3 | (uint16_t)(p[2])  << 5) & 0x07FF;
    ch[2]  = ((uint16_t)(p[2])  >> 6 | (uint16_t)(p[3])  << 2 | (uint16_t)(p[4]) << 10) & 0x07FF;
    ch[3]  = ((uint16_t)(p[4])  >> 1 | (uint16_t)(p[5])  << 7) & 0x07FF;
    ch[4]  = ((uint16_t)(p[5])  >> 4 | (uint16_t)(p[6])  << 4) & 0x07FF;
    ch[5]  = ((uint16_t)(p[6])  >> 7 | (uint16_t)(p[7])  << 1 | (uint16_t)(p[8]) << 9) & 0x07FF;
    ch[6]  = ((uint16_t)(p[8])  >> 2 | (uint16_t)(p[9])  << 6) & 0x07FF;
    ch[7]  = ((uint16_t)(p[9])  >> 5 | (uint16_t)(p[10]) << 3) & 0x07FF;
    ch[8]  = ((uint16_t)(p[11])      | (uint16_t)(p[12]) << 8) & 0x07FF;
    ch[9]  = ((uint16_t)(p[12]) >> 3 | (uint16_t)(p[13]) << 5) & 0x07FF;
    ch[10] = ((uint16_t)(p[13]) >> 6 | (uint16_t)(p[14]) << 2 | (uint16_t)(p[15]) << 10) & 0x07FF;
    ch[11] = ((uint16_t)(p[15]) >> 1 | (uint16_t)(p[16]) << 7) & 0x07FF;
    ch[12] = ((uint16_t)(p[16]) >> 4 | (uint16_t)(p[17]) << 4) & 0x07FF;
    ch[13] = ((uint16_t)(p[17]) >> 7 | (uint16_t)(p[18]) << 1 | (uint16_t)(p[19]) << 9) & 0x07FF;
    ch[14] = ((uint16_t)(p[19]) >> 2 | (uint16_t)(p[20]) << 6) & 0x07FF;
    ch[15] = ((uint16_t)(p[20]) >> 5 | (uint16_t)(p[21]) << 3) & 0x07FF;

    // ---- Seqlock write begin ----
    crsfShared.seq++;          // → odd: write in progress
    __DMB();                   // barrier: seq odd is visible before channel stores

    crsfShared.ch1 = crsfToUs(ch[0]);
    crsfShared.ch2 = crsfToUs(ch[1]);
    crsfShared.ch3 = crsfToUs(ch[2]);
    crsfShared.ch4 = crsfToUs(ch[3]);
    crsfShared.ch5 = crsfToUs(ch[4]);
    crsfShared.ch6 = crsfToUs(ch[5]);
    crsfShared.lastFrameMs = (uint32_t)millis();

    __DMB();                   // barrier: all channel stores visible before seq even
    crsfShared.seq++;          // → even: write complete
    // ---- Seqlock write end ----

    // Diagnostic counters — benign non-atomic RMW (Issue 3, sole writer)
    crsfShared.frameCount++;
}

// Core 1's byte-level CRSF state machine.
// Called inside loop1() — drains Serial1 with no rate limiting.
static void c1_parseCRSF()
{
    while (ELRS_SERIAL.available()) {
        uint8_t byte = ELRS_SERIAL.read();

        if (!c1_inFrame) {
            if (byte == CRSF_SYNC_BYTE) {
                c1_inFrame = true;
                c1_crsfIdx = 0;
                c1_crsfBuf[c1_crsfIdx++] = byte;
            }
        } else {
            c1_crsfBuf[c1_crsfIdx++] = byte;

            if (c1_crsfIdx == 2) {
                c1_crsfLen = byte + 2;
                if (c1_crsfLen > CRSF_MAX_FRAME_LEN) { c1_inFrame = false; }
            }

            if (c1_inFrame && c1_crsfLen > 0 && c1_crsfIdx >= c1_crsfLen) {
                uint8_t rxCrc  = c1_crsfBuf[c1_crsfLen - 1];
                uint8_t calCrc = crsfCrc8(&c1_crsfBuf[2], c1_crsfLen - 3);
                if (rxCrc == calCrc) {
                    if (c1_crsfBuf[2] == CRSF_TYPE_RC_CHAN)
                        c1_decodeChannels(&c1_crsfBuf[3]);
                } else {
                    crsfShared.crcErrors++;
                }
                c1_inFrame = false;
            }
        }
    }
}

// ================================================================
//  CORE 0 — CHANNEL SNAPSHOT  (seqlock reader)
//
//  Seqlock read protocol:
//    1. Read seq — if odd, Core 1 is mid-write: spin and retry.
//    2. __DMB() — acquire barrier.  All subsequent loads happen
//       AFTER seq1 is read.  Prevents the processor from
//       speculatively loading channel values before the seq check.
//    3. Read ch1..ch6, lastFrameMs.
//    4. __DMB() — release barrier.  All loads above are complete
//       before seq2 is read.
//    5. Read seq again.  If seq1 != seq2, Core 1 updated the struct
//       during our read window — discard and retry.
//
//  Collision rate:  Core 1 writes at 150 Hz, Core 0 reads at 2000 Hz.
//  P(collision) ≈ 150/2000 = 7.5 %.
//  Each retry costs ≈ 10 cycles (≈ 0.07 µs @ 150 MHz).
//  Average overhead: < 0.1 µs — negligible vs 500 µs loop budget.
// ================================================================

static unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm;
static unsigned long channel_4_pwm, channel_5_pwm, channel_6_pwm;
static unsigned long channel_1_prev, channel_2_prev;
static unsigned long channel_3_prev, channel_4_prev;

static inline void snapshotChannels()
{
    uint32_t seq1, seq2;
    uint32_t c1, c2, c3, c4, c5, c6, ts;

    // ---- Seqlock read begin ----
    do {
        seq1 = crsfShared.seq;
        if (seq1 & 1u) continue;    // odd = write in progress, spin

        __DMB();                    // acquire: loads below happen after seq1 load

        c1 = crsfShared.ch1;
        c2 = crsfShared.ch2;
        c3 = crsfShared.ch3;
        c4 = crsfShared.ch4;
        c5 = crsfShared.ch5;
        c6 = crsfShared.ch6;
        ts = crsfShared.lastFrameMs;

        __DMB();                    // release: all loads above complete before seq2

        seq2 = crsfShared.seq;
    } while (seq1 != seq2);         // retry if Core 1 wrote during our read
    // ---- Seqlock read end ----

    // Commit to local variables
    channel_1_pwm = c1;
    channel_2_pwm = c2;
    channel_3_pwm = c3;
    channel_4_pwm = c4;
    channel_5_pwm = c5;
    channel_6_pwm = c6;

    // Sanity-clamp (guards against uninitialised or corrupt values)
    channel_1_pwm = constrain(channel_1_pwm, 900UL, 2100UL);
    channel_2_pwm = constrain(channel_2_pwm, 900UL, 2100UL);
    channel_3_pwm = constrain(channel_3_pwm, 900UL, 2100UL);
    channel_4_pwm = constrain(channel_4_pwm, 900UL, 2100UL);
    channel_5_pwm = constrain(channel_5_pwm, 900UL, 2100UL);
    channel_6_pwm = constrain(channel_6_pwm, 900UL, 2100UL);

    // Failsafe: ts was read inside the seqlock, so it is guaranteed
    // to be consistent with the channel values above.
    if ((millis() - ts) > CRSF_TIMEOUT_MS) {
        channel_1_pwm = CH1_FS;
        channel_2_pwm = CH2_FS;
        channel_3_pwm = CH3_FS;
        channel_4_pwm = CH4_FS;
        channel_5_pwm = CH5_FS;  // disarm
        channel_6_pwm = CH6_FS;
    }

    // First-order LP filter on stick channels
    const float B_rc = 0.7f;
    channel_1_pwm = (unsigned long)((1.0f - B_rc) * channel_1_prev + B_rc * channel_1_pwm);
    channel_2_pwm = (unsigned long)((1.0f - B_rc) * channel_2_prev + B_rc * channel_2_pwm);
    channel_3_pwm = (unsigned long)((1.0f - B_rc) * channel_3_prev + B_rc * channel_3_pwm);
    channel_4_pwm = (unsigned long)((1.0f - B_rc) * channel_4_prev + B_rc * channel_4_pwm);
    channel_1_prev = channel_1_pwm;
    channel_2_prev = channel_2_pwm;
    channel_3_prev = channel_3_pwm;
    channel_4_prev = channel_4_pwm;
}

// ================================================================
//  BENCHMARK TIMING STATE  (Core 0 only)
// ================================================================

#ifdef BENCHMARK_MODE
static uint32_t bm_iterCount  = 0;
static uint32_t bm_sumUs      = 0;
static uint32_t bm_minUs      = UINT32_MAX;
static uint32_t bm_maxUs      = 0;
static uint32_t bm_windowStart = 0;

// Call at the very end of loop(), AFTER loopRate() returns.
// exec_us = measured execution time of the current iteration.
static void benchmarkAccumulate(uint32_t exec_us)
{
    bm_sumUs += exec_us;
    if (exec_us < bm_minUs) bm_minUs = exec_us;
    if (exec_us > bm_maxUs) bm_maxUs = exec_us;
    bm_iterCount++;

    // Print stats once per second
    if (millis() - bm_windowStart >= 1000) {
        bm_windowStart = millis();
        uint32_t mean = (bm_iterCount > 0) ? (bm_sumUs / bm_iterCount) : 0;

        Serial.println(F("──── Core 0 Loop Benchmark (1-second window) ────"));
        Serial.print(F("  Iterations : ")); Serial.println(bm_iterCount);
        Serial.print(F("  Mean dt    : ")); Serial.print(mean);   Serial.println(F(" µs"));
        Serial.print(F("  Min  dt    : ")); Serial.print(bm_minUs); Serial.println(F(" µs"));
        Serial.print(F("  Max  dt    : ")); Serial.print(bm_maxUs); Serial.println(F(" µs"));
        Serial.print(F("  Jitter     : ")); Serial.print(bm_maxUs - bm_minUs); Serial.println(F(" µs  ← key metric"));
        Serial.print(F("  Core1 frames: ")); Serial.print(crsfShared.frameCount);
        Serial.print(F("  CRC errors: ")); Serial.println(crsfShared.crcErrors);
        Serial.println(F("  Compare this jitter to the single-core version."));
        Serial.println();

        // Reset for next window
        bm_iterCount = 0;
        bm_sumUs     = 0;
        bm_minUs     = UINT32_MAX;
        bm_maxUs     = 0;
    }
}
#endif // BENCHMARK_MODE

// ================================================================
//  GLOBAL CONTROL STATE  (Core 0 only)
// ================================================================

float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, blink_counter, blink_delay;
bool blinkAlternate;

// IMU (filtered)
float AccX, AccY, AccZ, AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ, GyroX_prev, GyroY_prev, GyroZ_prev;

// Madgwick quaternion
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float roll_IMU, pitch_IMU, yaw_IMU, roll_IMU_prev, pitch_IMU_prev;

// Desired state
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

// PID state — roll
float error_roll, error_roll_prev, roll_des_prev;
float integral_roll, integral_roll_il, integral_roll_ol;
float integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol;
float derivative_roll, roll_PID = 0.0f;

// PID state — pitch
float error_pitch, error_pitch_prev, pitch_des_prev;
float integral_pitch, integral_pitch_il, integral_pitch_ol;
float integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol;
float derivative_pitch, pitch_PID = 0.0f;

// PID state — yaw
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev;
float derivative_yaw, yaw_PID = 0.0f;

// Altitude hold
float alt_est = 0.0f, alt_des = 0.0f, alt_prev = 0.0f;
float climb_rate_est = 0.0f, climb_rate_des = 0.0f;
float thro_alt_trim = 0.0f;
float integral_climb = 0.0f, integral_climb_prev = 0.0f;
float error_climb_prev = 0.0f;
float baroAltHome = 0.0f;
unsigned long baro_prev_time = 0;

// Motor commands
float m1_command_scaled, m2_command_scaled;
float m3_command_scaled, m4_command_scaled;
int   m1_command_PWM, m2_command_PWM;
int   m3_command_PWM, m4_command_PWM;

// Tunable parameters (from config.h defaults)
float B_madgwick = B_MADGWICK;
float B_accel    = B_ACCEL;
float B_gyro     = B_GYRO;
float i_limit    = I_LIMIT;
float maxRoll    = MAX_ROLL;
float maxPitch   = MAX_PITCH;
float maxYaw     = MAX_YAW;

float Kp_roll_angle  = KP_ROLL_ANGLE,  Ki_roll_angle  = KI_ROLL_ANGLE,  Kd_roll_angle  = KD_ROLL_ANGLE;
float Kp_pitch_angle = KP_PITCH_ANGLE, Ki_pitch_angle = KI_PITCH_ANGLE, Kd_pitch_angle = KD_PITCH_ANGLE;
float B_loop_roll    = B_LOOP_ROLL,    B_loop_pitch   = B_LOOP_PITCH;
float Kp_roll_rate   = KP_ROLL_RATE,   Ki_roll_rate   = KI_ROLL_RATE,   Kd_roll_rate   = KD_ROLL_RATE;
float Kp_pitch_rate  = KP_PITCH_RATE,  Ki_pitch_rate  = KI_PITCH_RATE,  Kd_pitch_rate  = KD_PITCH_RATE;
float Kp_yaw         = KP_YAW,         Ki_yaw         = KI_YAW,         Kd_yaw         = KD_YAW;
float Kp_alt         = KP_ALT,         Kp_climb       = KP_CLIMB;
float Ki_climb       = KI_CLIMB,       Kd_climb       = KD_CLIMB;

float AccErrorX = 0.0f, AccErrorY = 0.0f, AccErrorZ = 0.0f;
float GyroErrorX = 0.0f, GyroErrorY = 0.0f, GyroErrorZ = 0.0f;

bool armedFly = false;

// ================================================================
//  CORE 0 — SETUP
// ================================================================

void setup()
{
    Serial.begin(500000);
    delay(500);

    Serial.println(F("[DUAL-CORE] Curio Flight Controller — Dual-Core Version"));
    Serial.println(F("[DUAL-CORE] Core 0: Control loop (2 kHz, no CRSF)"));
    Serial.println(F("[DUAL-CORE] Core 1: CRSF parser (tight loop, no control)"));
#ifdef BENCHMARK_MODE
    Serial.println(F("[DUAL-CORE] BENCHMARK MODE ENABLED — loop timing printed every 1 s"));
#endif

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Motors
    analogWriteFreq(MOTOR_PWM_FREQ);
    analogWriteResolution(MOTOR_PWM_BITS);
    pinMode(PIN_M1, OUTPUT); analogWrite(PIN_M1, 0);
    pinMode(PIN_M2, OUTPUT); analogWrite(PIN_M2, 0);
    pinMode(PIN_M3, OUTPUT); analogWrite(PIN_M3, 0);
    pinMode(PIN_M4, OUTPUT); analogWrite(PIN_M4, 0);

    // IMU
    bmi088Init();
    delay(10);
    if (!bmi088CheckIDs()) {
        Serial.println(F("[ERROR] BMI088 not found!"));
        while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(200); }
    }
    Serial.println(F("[INIT] BMI088 OK"));

    // Barometer
    if (!bmp580Init()) {
        Serial.println(F("[INIT] WARNING: BMP580 not found. Altitude hold disabled."));
    } else {
        Serial.println(F("[INIT] BMP580 OK"));
        delay(100);
        float sumAlt = 0.0f;
        for (int i = 0; i < 50; i++) {
            sumAlt += bmp580PressureToAltitude(bmp580ReadPressure());
            delay(20);
        }
        baroAltHome = sumAlt / 50.0f;
        Serial.print(F("[INIT] Baro home: ")); Serial.print(baroAltHome, 1); Serial.println(F(" m MSL"));
    }

    // Warm up AHRS
    Serial.println(F("[INIT] Warming up AHRS (Core 0)..."));
    calibrateAttitude();
    Serial.println(F("[INIT] AHRS ready."));

    setupBlink(3, 160, 70);
    Serial.println(F("[INIT] Core 0 ready. Waiting for Core 1 (CRSF)..."));

    // Give Core 1 time to initialise Serial1 before the loop starts
    delay(200);

#ifdef BENCHMARK_MODE
    bm_windowStart = millis();
#endif
}

// ================================================================
//  CORE 0 — MAIN LOOP  (2 kHz, no CRSF parsing here)
// ================================================================

void loop()
{
#ifdef BENCHMARK_MODE
    uint32_t loop_start_us = micros();
#endif

    prev_time    = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0f;

    loopBlink();

    // --- Snapshot channel values from Core 1's shared struct ---
    snapshotChannels();

    // --- Arming check --------------------------------------------
    armedStatus();

    // --- IMU + AHRS ---------------------------------------------
    getIMUdata();
    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);

    // --- Desired state ------------------------------------------
    getDesState();

#ifdef USE_ALT_HOLD
    updateAltHold();
#endif

    // --- PID control law ----------------------------------------
#if defined USE_CONTROL_ANGLE
    controlANGLE();
#elif defined USE_CONTROL_ANGLE2
    controlANGLE2();
#elif defined USE_CONTROL_RATE
    controlRATE();
#endif

    // --- Mixer + scale ------------------------------------------
    controlMixer();
    scaleCommands();

    // --- Safety cut + motor output ------------------------------
    throttleCut();
    commandMotors();

    // --- Regulate to 2 kHz  (NOTE: no getCommands() call here) --
    loopRate(LOOP_RATE_HZ);

    // --- Benchmark measurement ----------------------------------
#ifdef BENCHMARK_MODE
    uint32_t exec_us = micros() - loop_start_us;
    benchmarkAccumulate(exec_us);
#endif

    // Debug prints (uncomment one at a time)
    // printRadioData();
    // printRollPitchYaw();
    // printPIDoutput();
    // printMotorCommands();
    // printLoopRate();
}

// ================================================================
//  CORE 1 — SETUP  (runs concurrently with Core 0's setup())
//
//  arduino-pico calls setup1() on Core 1 once at startup.
//  Core 0's setup() includes a delay(200) so that Serial1 is
//  fully initialised before the first snapshotChannels() call.
// ================================================================

void setup1()
{
    // Initialise ELRS CRSF serial on Core 1
    ELRS_SERIAL.setTX(PIN_ELRS_TX);
    ELRS_SERIAL.setRX(PIN_ELRS_RX);
    ELRS_SERIAL.begin(ELRS_BAUD);
    // No Serial.println here — Serial (USB) is owned by Core 0
}

// ================================================================
//  CORE 1 — LOOP  (tight CRSF parsing, no rate limiting)
//
//  Core 1 runs as fast as the UART delivers bytes (~420 kBaud).
//  It never performs any motor output or PID computation.
//  All timing decisions remain on Core 0.
// ================================================================

void loop1()
{
    c1_parseCRSF();
    // No delay — tight loop saturates Core 1's UART polling,
    // ensuring minimum latency between ELRS packet reception
    // and crsfShared update.
}

// ================================================================
//  IMU DATA ACQUISITION  (Core 0)
// ================================================================

void getIMUdata()
{
    IMURaw raw = bmi088GetLatest();

    float ax = raw.ax - AccErrorX;
    float ay = raw.ay - AccErrorY;
    float az = raw.az - AccErrorZ;
    AccX = (1.0f - B_accel) * AccX_prev + B_accel * ax;
    AccY = (1.0f - B_accel) * AccY_prev + B_accel * ay;
    AccZ = (1.0f - B_accel) * AccZ_prev + B_accel * az;
    AccX_prev = AccX; AccY_prev = AccY; AccZ_prev = AccZ;

    float gx = raw.gx - GyroErrorX;
    float gy = raw.gy - GyroErrorY;
    float gz = raw.gz - GyroErrorZ;
    GyroX = (1.0f - B_gyro) * GyroX_prev + B_gyro * gx;
    GyroY = (1.0f - B_gyro) * GyroY_prev + B_gyro * gy;
    GyroZ = (1.0f - B_gyro) * GyroZ_prev + B_gyro * gz;
    GyroX_prev = GyroX; GyroY_prev = GyroY; GyroZ_prev = GyroZ;
}

// ================================================================
//  MADGWICK 6-DOF AHRS  (Core 0)
// ================================================================

float invSqrt(float x) { return 1.0f / sqrtf(x); }

void Madgwick6DOF(float gx, float gy, float gz,
                  float ax, float ay, float az, float invSampleFreq)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
    float _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

    qDot1 = 0.5f*(-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f*( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f*( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f*( q0*gz + q1*gy - q2*gx);

    if (!((ax==0.0f)&&(ay==0.0f)&&(az==0.0f))) {
        recipNorm = invSqrt(ax*ax+ay*ay+az*az);
        ax*=recipNorm; ay*=recipNorm; az*=recipNorm;
        _2q0=2*q0; _2q1=2*q1; _2q2=2*q2; _2q3=2*q3;
        _4q0=4*q0; _4q1=4*q1; _4q2=4*q2;
        _8q1=8*q1; _8q2=8*q2;
        q0q0=q0*q0; q1q1=q1*q1; q2q2=q2*q2; q3q3=q3*q3;
        s0=_4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay;
        s1=_4q1*q3q3-_2q3*ax+4*q0q0*q1-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az;
        s2=4*q0q0*q2+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az;
        s3=4*q1q1*q3-_2q1*ax+4*q2q2*q3-_2q2*ay;
        recipNorm=invSqrt(s0*s0+s1*s1+s2*s2+s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;
        qDot1-=B_madgwick*s0; qDot2-=B_madgwick*s1;
        qDot3-=B_madgwick*s2; qDot4-=B_madgwick*s3;
    }
    q0+=qDot1*invSampleFreq; q1+=qDot2*invSampleFreq;
    q2+=qDot3*invSampleFreq; q3+=qDot4*invSampleFreq;
    recipNorm=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;

    roll_IMU  =  atan2f(q0*q1+q2*q3, 0.5f-q1*q1-q2*q2)*57.29577951f;
    pitch_IMU = -asinf(constrain(-2.0f*(q1*q3-q0*q2),-0.999999f,0.999999f))*57.29577951f;
    yaw_IMU   = -atan2f(q1*q2+q0*q3, 0.5f-q2*q2-q3*q3)*57.29577951f;
}

// ================================================================
//  DESIRED STATE
// ================================================================

void getDesState()
{
    thro_des = (channel_1_pwm - 1000.0f) / 1000.0f;
    roll_des  = (channel_2_pwm - 1500.0f) / 500.0f;
    pitch_des = (channel_3_pwm - 1500.0f) / 500.0f;
    yaw_des   = (channel_4_pwm - 1500.0f) / 500.0f;
    roll_passthru  = roll_des  / 2.0f;
    pitch_passthru = pitch_des / 2.0f;
    yaw_passthru   = yaw_des   / 2.0f;
    thro_des  = constrain(thro_des,  0.0f, 1.0f);
    roll_des  = constrain(roll_des, -1.0f, 1.0f) * maxRoll;
    pitch_des = constrain(pitch_des,-1.0f, 1.0f) * maxPitch;
    yaw_des   = constrain(yaw_des,  -1.0f, 1.0f) * maxYaw;
    roll_passthru  = constrain(roll_passthru,  -0.5f, 0.5f);
    pitch_passthru = constrain(pitch_passthru, -0.5f, 0.5f);
    yaw_passthru   = constrain(yaw_passthru,   -0.5f, 0.5f);
}

// ================================================================
//  ALTITUDE HOLD  (50 Hz outer loop on Core 0)
// ================================================================

void updateAltHold()
{
    unsigned long now = micros();
    float dtBaro = (now - baro_prev_time) / 1000000.0f;
    if (dtBaro < (1.0f / BARO_RATE_HZ)) return;
    baro_prev_time = now;

    float pPa   = bmp580ReadPressure();
    float altMSL = bmp580PressureToAltitude(pPa);
    alt_est = altMSL - baroAltHome;
    climb_rate_est = (alt_est - alt_prev) / dtBaro;
    alt_prev = alt_est;

    if (channel_6_pwm > 1500 && armedFly) {
        float error_alt = alt_des - alt_est;
        climb_rate_des = constrain(Kp_alt * error_alt, -MAX_CLIMB_RATE, MAX_CLIMB_RATE);
        float error_climb = climb_rate_des - climb_rate_est;
        integral_climb = constrain(integral_climb_prev + error_climb * dtBaro, -i_limit, i_limit);
        float deriv_climb = (error_climb - error_climb_prev) / dtBaro;
        thro_alt_trim = constrain(Kp_climb*error_climb + Ki_climb*integral_climb + Kd_climb*deriv_climb, -0.3f, 0.3f);
        integral_climb_prev = integral_climb;
        error_climb_prev    = error_climb;
        thro_des = constrain(0.5f + thro_alt_trim, 0.05f, 0.95f);
    } else {
        alt_des = alt_est;
        thro_alt_trim = 0.0f;
        integral_climb = integral_climb_prev = 0.0f;
        error_climb_prev = 0.0f;
    }
}

// ================================================================
//  ANGLE MODE PID
// ================================================================

void controlANGLE()
{
    error_roll = roll_des - roll_IMU;
    integral_roll = integral_roll_prev + error_roll * dt;
    if (channel_1_pwm < 1060) integral_roll = 0.0f;
    integral_roll = constrain(integral_roll, -i_limit, i_limit);
    derivative_roll = GyroX;
    roll_PID = 0.01f*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll);

    error_pitch = pitch_des - pitch_IMU;
    integral_pitch = integral_pitch_prev + error_pitch * dt;
    if (channel_1_pwm < 1060) integral_pitch = 0.0f;
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
    derivative_pitch = GyroY;
    pitch_PID = 0.01f*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch);

    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw * dt;
    if (channel_1_pwm < 1060) integral_yaw = 0.0f;
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
    derivative_yaw = (error_yaw - error_yaw_prev) / dt;
    yaw_PID = 0.01f*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw);

    integral_roll_prev  = integral_roll;
    integral_pitch_prev = integral_pitch;
    integral_yaw_prev   = integral_yaw;
    error_yaw_prev      = error_yaw;
}

// ================================================================
//  CASCADED ANGLE → RATE PID
// ================================================================

void controlANGLE2()
{
    float roll_des_ol, pitch_des_ol;

    error_roll = roll_des - roll_IMU;
    integral_roll_ol = constrain(integral_roll_prev_ol + error_roll*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_roll_ol = 0.0f;
    roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;

    error_pitch = pitch_des - pitch_IMU;
    integral_pitch_ol = constrain(integral_pitch_prev_ol + error_pitch*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_pitch_ol = 0.0f;
    pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;

    const float Kl = 30.0f;
    roll_des_ol  = constrain(Kl*roll_des_ol,  -240.0f, 240.0f);
    pitch_des_ol = constrain(Kl*pitch_des_ol, -240.0f, 240.0f);
    roll_des_ol  = (1.0f-B_loop_roll) *roll_des_prev  + B_loop_roll *roll_des_ol;
    pitch_des_ol = (1.0f-B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

    error_roll = roll_des_ol - GyroX;
    integral_roll_il = constrain(integral_roll_prev_il + error_roll*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_roll_il = 0.0f;
    derivative_roll  = (error_roll - error_roll_prev) / dt;
    roll_PID = 0.01f*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll);

    error_pitch = pitch_des_ol - GyroY;
    integral_pitch_il = constrain(integral_pitch_prev_il + error_pitch*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_pitch_il = 0.0f;
    derivative_pitch  = (error_pitch - error_pitch_prev) / dt;
    pitch_PID = 0.01f*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch);

    error_yaw = yaw_des - GyroZ;
    integral_yaw = constrain(integral_yaw_prev + error_yaw*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_yaw = 0.0f;
    derivative_yaw = (error_yaw - error_yaw_prev) / dt;
    yaw_PID = 0.01f*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw);

    integral_roll_prev_ol = integral_roll_ol; integral_roll_prev_il = integral_roll_il;
    error_roll_prev = error_roll; roll_IMU_prev = roll_IMU; roll_des_prev = roll_des_ol;
    integral_pitch_prev_ol = integral_pitch_ol; integral_pitch_prev_il = integral_pitch_il;
    error_pitch_prev = error_pitch; pitch_IMU_prev = pitch_IMU; pitch_des_prev = pitch_des_ol;
    integral_yaw_prev = integral_yaw; error_yaw_prev = error_yaw;
}

// ================================================================
//  RATE / ACRO PID
// ================================================================

void controlRATE()
{
    error_roll = roll_des - GyroX;
    integral_roll = constrain(integral_roll_prev + error_roll*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_roll = 0.0f;
    derivative_roll = (error_roll - error_roll_prev) / dt;
    roll_PID = 0.01f*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll);

    error_pitch = pitch_des - GyroY;
    integral_pitch = constrain(integral_pitch_prev + error_pitch*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_pitch = 0.0f;
    derivative_pitch = (error_pitch - error_pitch_prev) / dt;
    pitch_PID = 0.01f*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch);

    error_yaw = yaw_des - GyroZ;
    integral_yaw = constrain(integral_yaw_prev + error_yaw*dt, -i_limit, i_limit);
    if (channel_1_pwm < 1060) integral_yaw = 0.0f;
    derivative_yaw = (error_yaw - error_yaw_prev) / dt;
    yaw_PID = 0.01f*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw);

    error_roll_prev = error_roll; integral_roll_prev = integral_roll;
    error_pitch_prev = error_pitch; integral_pitch_prev = integral_pitch;
    error_yaw_prev = error_yaw; integral_yaw_prev = integral_yaw;
}

// ================================================================
//  MOTOR MIXER  (X-quad)
// ================================================================

void controlMixer()
{
    m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;  // FL
    m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;  // FR
    m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;  // RR
    m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;  // RL
}

void scaleCommands()
{
    m1_command_PWM = constrain((int)(m1_command_scaled * MOTOR_PWM_MAX), 0, MOTOR_PWM_MAX);
    m2_command_PWM = constrain((int)(m2_command_scaled * MOTOR_PWM_MAX), 0, MOTOR_PWM_MAX);
    m3_command_PWM = constrain((int)(m3_command_scaled * MOTOR_PWM_MAX), 0, MOTOR_PWM_MAX);
    m4_command_PWM = constrain((int)(m4_command_scaled * MOTOR_PWM_MAX), 0, MOTOR_PWM_MAX);
}

void commandMotors()
{
    analogWrite(PIN_M1, m1_command_PWM);
    analogWrite(PIN_M2, m2_command_PWM);
    analogWrite(PIN_M3, m3_command_PWM);
    analogWrite(PIN_M4, m4_command_PWM);
}

// ================================================================
//  ARMING / SAFETY
// ================================================================

void armedStatus()
{
    if ((channel_5_pwm < 1500) && (channel_1_pwm < 1060)) armedFly = true;
}

void throttleCut()
{
    if (channel_5_pwm > 1500) armedFly = false;
    if (!armedFly) {
        m1_command_PWM = m2_command_PWM = m3_command_PWM = m4_command_PWM = 0;
    }
}

// ================================================================
//  LOOP RATE REGULATOR  (Core 0)
// ================================================================

void loopRate(int freq)
{
    float invFreq = 1.0f / freq * 1000000.0f;
    unsigned long checker = micros();
    while (invFreq > (float)(checker - current_time)) checker = micros();
}

// ================================================================
//  LED BLINKER  (Core 0)
// ================================================================

void loopBlink()
{
    if (current_time - blink_counter > blink_delay) {
        blink_counter = micros();
        digitalWrite(PIN_LED, blinkAlternate);
        blinkAlternate = !blinkAlternate;
        blink_delay = blinkAlternate ? 100000UL : 2000000UL;
    }
}

void setupBlink(int n, int up, int down)
{
    for (int j = 0; j < n; j++) {
        digitalWrite(PIN_LED, LOW); delay(down);
        digitalWrite(PIN_LED, HIGH); delay(up);
    }
}

// ================================================================
//  MADGWICK WARM-UP  (Core 0)
// ================================================================

void calibrateAttitude()
{
    for (int i = 0; i <= 10000; i++) {
        prev_time = current_time; current_time = micros();
        dt = (current_time - prev_time) / 1000000.0f;
        getIMUdata();
        Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);
        loopRate(LOOP_RATE_HZ);
    }
}

// ================================================================
//  DEBUG PRINTS  (Core 0, call one at a time)
// ================================================================

void printRadioData()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F(" CH1:")); Serial.print(channel_1_pwm);
        Serial.print(F(" CH2:")); Serial.print(channel_2_pwm);
        Serial.print(F(" CH3:")); Serial.print(channel_3_pwm);
        Serial.print(F(" CH4:")); Serial.print(channel_4_pwm);
        Serial.print(F(" CH5:")); Serial.print(channel_5_pwm);
        Serial.print(F(" CH6:")); Serial.println(channel_6_pwm);
    }
}

void printRollPitchYaw()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("Roll:")); Serial.print(roll_IMU);
        Serial.print(F(" Pitch:")); Serial.print(pitch_IMU);
        Serial.print(F(" Yaw:")); Serial.println(yaw_IMU);
    }
}

void printPIDoutput()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("rPID:")); Serial.print(roll_PID);
        Serial.print(F(" pPID:")); Serial.print(pitch_PID);
        Serial.print(F(" yPID:")); Serial.println(yaw_PID);
    }
}

void printMotorCommands()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("M1:")); Serial.print(m1_command_PWM);
        Serial.print(F(" M2:")); Serial.print(m2_command_PWM);
        Serial.print(F(" M3:")); Serial.print(m3_command_PWM);
        Serial.print(F(" M4:")); Serial.println(m4_command_PWM);
    }
}

void printLoopRate()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("dt(us):")); Serial.println(dt * 1000000.0f);
    }
}
