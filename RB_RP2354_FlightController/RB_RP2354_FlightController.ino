// ================================================================
//  RB-RP2354A Flight Controller
//  Inspired by dRehmFlight (Nicholas Rehm, Beta 1.3)
//  Adapted for RB-RP2354A by CS / CYCU EE Dept.
//
//  Microcontroller : Raspberry Pi RP2354A  (Cortex-M33 dual-core, 150 MHz)
//  IMU             : Bosch BMI088           (SPI, 2 kHz interrupt-driven)
//  Barometer       : Bosch BMP580           (I²C, 50 Hz)
//  RC Receiver     : ExpressLRS CRSF        (UART0, 420 kBaud, 16 ch)
//  Motor driver    : 4× brushed MOSFET      (32 kHz, 10-bit PWM)
//
//  Build environment: Arduino IDE 2.3.x + earlephilhower/arduino-pico ≥ 3.x
//    Board: "Raspberry Pi Pico 2" (RP2350/RP2354 family)
//    CPU speed: 150 MHz
//
//  Control architecture (identical to dRehmFlight):
//    • Madgwick 6-DOF AHRS  (2 kHz)
//    • Angle mode    : controlANGLE()   — single-loop angle PID
//    • Cascaded mode : controlANGLE2()  — outer angle + inner rate PID
//    • Rate mode     : controlRATE()    — direct rate PID
//    • Altitude hold : controlAltHold() — baro outer + climb-rate inner PID
//    • X-quad mixer  : Front-Left/Front-Right/Rear-Right/Rear-Left
// ================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include "src/bmi088_driver.h"
#include "src/bmp580_driver.h"

// ================================================================
//  USER SELECTION — uncomment exactly ONE controller
// ================================================================
#define USE_CONTROL_ANGLE    // Single-loop angle PID  (recommended for first flight)
//#define USE_CONTROL_ANGLE2   // Cascaded angle→rate PID (better performance, harder to tune)
//#define USE_CONTROL_RATE     // Pure rate / acro mode

// Uncomment to enable altitude hold (requires BMP580)
//#define USE_ALT_HOLD

// ----------------------------------------------------------------
// BENCHMARK MODE — Core 0 loop timing statistics (1-second window).
// Enable this AND BENCHMARK_MODE in Curio_FlightController_DualCore
// to compare jitter between the single-core and dual-core versions.
// Key metric: Jitter = Max_dt − Min_dt (µs).  Lower = better.
// ----------------------------------------------------------------
//#define BENCHMARK_MODE

// ================================================================
//  USER-SPECIFIED VARIABLES
// ================================================================

// Radio failsafe (µs, CRSF mapped to 1000–2000)
unsigned long channel_1_fs = CH1_FS;  // Throttle cut
unsigned long channel_2_fs = CH2_FS;  // Roll  center
unsigned long channel_3_fs = CH3_FS;  // Pitch center
unsigned long channel_4_fs = CH4_FS;  // Yaw   center
unsigned long channel_5_fs = CH5_FS;  // Arm switch — disarmed
unsigned long channel_6_fs = CH6_FS;  // Aux1

// Filter parameters
float B_madgwick = B_MADGWICK;
float B_accel    = B_ACCEL;
float B_gyro     = B_GYRO;

// IMU calibration offsets (run calculate_IMU_error() once, paste here, then comment it out)
float AccErrorX  = 0.0f;
float AccErrorY  = 0.0f;
float AccErrorZ  = 0.0f;
float GyroErrorX = 0.0f;
float GyroErrorY = 0.0f;
float GyroErrorZ = 0.0f;

// Barometer calibration (home altitude, set on ground at startup)
float baroAltHome = 0.0f;

// Controller limits & setpoints
float i_limit  = I_LIMIT;
float maxRoll  = MAX_ROLL;
float maxPitch = MAX_PITCH;
float maxYaw   = MAX_YAW;

// Angle-mode PID gains
float Kp_roll_angle  = KP_ROLL_ANGLE;
float Ki_roll_angle  = KI_ROLL_ANGLE;
float Kd_roll_angle  = KD_ROLL_ANGLE;
float B_loop_roll    = B_LOOP_ROLL;

float Kp_pitch_angle = KP_PITCH_ANGLE;
float Ki_pitch_angle = KI_PITCH_ANGLE;
float Kd_pitch_angle = KD_PITCH_ANGLE;
float B_loop_pitch   = B_LOOP_PITCH;

// Rate-mode PID gains
float Kp_roll_rate  = KP_ROLL_RATE;
float Ki_roll_rate  = KI_ROLL_RATE;
float Kd_roll_rate  = KD_ROLL_RATE;

float Kp_pitch_rate = KP_PITCH_RATE;
float Ki_pitch_rate = KI_PITCH_RATE;
float Kd_pitch_rate = KD_PITCH_RATE;

float Kp_yaw = KP_YAW;
float Ki_yaw = KI_YAW;
float Kd_yaw = KD_YAW;

// Altitude-hold PID gains
float Kp_alt    = KP_ALT;
float Kp_climb  = KP_CLIMB;
float Ki_climb  = KI_CLIMB;
float Kd_climb  = KD_CLIMB;

// ================================================================
//  GLOBAL STATE VARIABLES
// ================================================================

// Timing
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, blink_counter;
unsigned long blink_delay;
bool blinkAlternate;

// Benchmark state (only used when BENCHMARK_MODE is defined)
#ifdef BENCHMARK_MODE
static uint32_t bm_iterCount   = 0;
static uint32_t bm_sumUs       = 0;
static uint32_t bm_minUs       = UINT32_MAX;
static uint32_t bm_maxUs       = 0;
static uint32_t bm_windowStart = 0;
#endif

// IMU data (filtered, deg/s and g)
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;

// Madgwick quaternion
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Attitude estimate (degrees)
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

// RC commands (µs, 1000–2000)
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm;
unsigned long channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev;
unsigned long channel_3_pwm_prev, channel_4_pwm_prev;
unsigned long crsf_last_frame_ms = 0;

// Desired state (normalized or in physical units)
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
float error_yaw, error_yaw_prev;
float integral_yaw, integral_yaw_prev;
float derivative_yaw, yaw_PID = 0.0f;

// Altitude hold state
float alt_est = 0.0f;          // Current altitude AGL (m)
float alt_des = 0.0f;          // Desired altitude AGL (m)
float climb_rate_est = 0.0f;   // Estimated climb rate (m/s), simple finite difference
float climb_rate_des = 0.0f;   // Desired climb rate from outer PID
float alt_prev = 0.0f;
float thro_alt_trim = 0.0f;    // Throttle trim from altitude PID
float integral_climb = 0.0f, integral_climb_prev = 0.0f;
float error_climb_prev = 0.0f;
unsigned long baro_prev_time = 0;

// Motor commands
float m1_command_scaled, m2_command_scaled;
float m3_command_scaled, m4_command_scaled;
int   m1_command_PWM, m2_command_PWM;
int   m3_command_PWM, m4_command_PWM;

// Arming flag
bool armedFly = false;

// ================================================================
//  CRSF PARSER  — 420 kBaud UART on Serial1
// ================================================================
//  Frame format (type 0x16 = RC channels):
//    [0xC8][len][0x16][22 bytes of 11-bit packed channels][CRC8]
//  Channel range raw: 172 (≈1000 µs) … 992 (≈1500 µs) … 1811 (≈2000 µs)
// ================================================================

#define CRSF_SYNC_BYTE      0xC8
#define CRSF_TYPE_RC_CHAN   0x16
#define CRSF_MAX_FRAME_LEN  64

static uint8_t  crsfBuf[CRSF_MAX_FRAME_LEN];
static uint8_t  crsfBufIdx = 0;
static uint8_t  crsfExpLen = 0;
static bool     crsfInFrame = false;

// CRC-8/DVB-S2 used by CRSF
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

// Map CRSF raw value (172–1811) to µs (1000–2000)
static inline unsigned long crsfToUs(uint16_t raw)
{
    // Linear interpolation: 172→1000 µs, 1811→2000 µs
    return (unsigned long)constrain(
        (long)raw * 1000L / 1639L + 895L,
        1000L, 2000L);
}

// Decode a validated RC-channels frame (22-byte payload after type byte)
static void crsfDecodeChannels(const uint8_t *p)
{
    // 16 channels × 11 bits packed little-endian
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

    // Map to µs; CRSF convention: ch1=throttle, ch2=roll, ch3=pitch, ch4=yaw
    channel_1_pwm = crsfToUs(ch[0]);  // Throttle
    channel_2_pwm = crsfToUs(ch[1]);  // Roll (aileron)
    channel_3_pwm = crsfToUs(ch[2]);  // Pitch (elevator)
    channel_4_pwm = crsfToUs(ch[3]);  // Yaw (rudder)
    channel_5_pwm = crsfToUs(ch[4]);  // Arm switch
    channel_6_pwm = crsfToUs(ch[5]);  // Aux1 (alt-hold)

    crsf_last_frame_ms = millis();
}

// Call frequently from loop() to drain Serial1 RX buffer
void parseCRSF()
{
    while (ELRS_SERIAL.available()) {
        uint8_t byte = ELRS_SERIAL.read();

        if (!crsfInFrame) {
            if (byte == CRSF_SYNC_BYTE) {
                crsfInFrame  = true;
                crsfBufIdx   = 0;
                crsfBuf[crsfBufIdx++] = byte;  // store sync
            }
        } else {
            crsfBuf[crsfBufIdx++] = byte;

            if (crsfBufIdx == 2) {
                // Byte 1 is frame length (bytes following this field)
                crsfExpLen = byte + 2;  // total = sync + len + payload + crc
                if (crsfExpLen > CRSF_MAX_FRAME_LEN) {
                    // Sanity check failed — restart
                    crsfInFrame = false;
                }
            }

            if (crsfInFrame && crsfExpLen > 0 && crsfBufIdx >= crsfExpLen) {
                // Full frame received; check CRC (CRC covers type + payload)
                uint8_t rxCrc  = crsfBuf[crsfExpLen - 1];
                uint8_t calCrc = crsfCrc8(&crsfBuf[2], crsfExpLen - 3);

                if (rxCrc == calCrc && crsfBuf[2] == CRSF_TYPE_RC_CHAN) {
                    crsfDecodeChannels(&crsfBuf[3]);  // payload starts at byte 3
                }
                crsfInFrame = false;
            }
        }
    }
}

// ================================================================
//  SETUP
// ================================================================

void setup()
{
    Serial.begin(500000);  // USB serial for debug
    delay(500);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);  // LED on during init

    // ---- Motor PWM setup ----------------------------------------
    analogWriteFreq(MOTOR_PWM_FREQ);
    analogWriteResolution(MOTOR_PWM_BITS);
    pinMode(PIN_M1, OUTPUT); analogWrite(PIN_M1, 0);
    pinMode(PIN_M2, OUTPUT); analogWrite(PIN_M2, 0);
    pinMode(PIN_M3, OUTPUT); analogWrite(PIN_M3, 0);
    pinMode(PIN_M4, OUTPUT); analogWrite(PIN_M4, 0);

    // ---- ELRS CRSF receiver -------------------------------------
    ELRS_SERIAL.setTX(PIN_ELRS_TX);
    ELRS_SERIAL.setRX(PIN_ELRS_RX);
    ELRS_SERIAL.begin(ELRS_BAUD);
    Serial.println("[INIT] ELRS CRSF UART started");

    // ---- IMU (BMI088 SPI, interrupt-driven) ---------------------
    bmi088Init();
    delay(10);
    if (!bmi088CheckIDs()) {
        Serial.println("[INIT] ERROR: BMI088 not found! Check wiring.");
        while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(200); }
    }
    Serial.println("[INIT] BMI088 OK");

    // ---- Barometer (BMP580 I²C) ---------------------------------
    if (!bmp580Init()) {
        Serial.println("[INIT] WARNING: BMP580 not found. Altitude hold disabled.");
    } else {
        Serial.println("[INIT] BMP580 OK");
        // Warm up barometer and record home altitude
        delay(100);
        float sumAlt = 0.0f;
        for (int i = 0; i < 50; i++) {
            float p = bmp580ReadPressure();
            sumAlt += bmp580PressureToAltitude(p);
            delay(20);
        }
        baroAltHome = sumAlt / 50.0f;
        Serial.print("[INIT] Baro home altitude: ");
        Serial.print(baroAltHome, 1);
        Serial.println(" m MSL");
    }

    // ---- Set failsafe channel values ----------------------------
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;

    // ---- IMU bias calibration (uncomment to run, paste results) -
    // Ensure craft is stationary and level before running!
    // calculate_IMU_error();

    // ---- Warm up Madgwick filter --------------------------------
    Serial.println("[INIT] Warming up AHRS...");
    calibrateAttitude();
    Serial.println("[INIT] AHRS ready.");

    // ---- Startup blink sequence ---------------------------------
    setupBlink(3, 160, 70);
    Serial.println("[INIT] Flight controller ready. Arm: CH5 < 1500, throttle low.");
}

// ================================================================
//  MAIN LOOP  (target: 2 kHz)
// ================================================================

void loop()
{
#ifdef BENCHMARK_MODE
    uint32_t bm_loop_start = micros();
#endif

    prev_time    = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0f;

    loopBlink();

    // ---- Debug print (uncomment ONE at a time) ------------------
    // printRadioData();
    // printDesiredState();
    // printGyroData();
    // printAccelData();
    // printRollPitchYaw();
    // printPIDoutput();
    // printMotorCommands();
    // printLoopRate();
    // printBatteryVoltage();
    // printAltitude();

    // ---- Arming check -------------------------------------------
    armedStatus();

    // ---- Get IMU data + AHRS -----------------------------------
    getIMUdata();
    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);

    // ---- Desired state from RC ----------------------------------
    getDesState();

    // ---- Altitude hold (50 Hz) ----------------------------------
#ifdef USE_ALT_HOLD
    updateAltHold();
#endif

    // ---- Attitude / Rate PID ------------------------------------
#if defined USE_CONTROL_ANGLE
    controlANGLE();
#elif defined USE_CONTROL_ANGLE2
    controlANGLE2();
#elif defined USE_CONTROL_RATE
    controlRATE();
#endif

    // ---- Mixer + scale to PWM -----------------------------------
    controlMixer();
    scaleCommands();

    // ---- Throttle cut / safety ----------------------------------
    throttleCut();

    // ---- Send PWM to motors -------------------------------------
    commandMotors();

    // ---- Update radio commands (drain CRSF buffer) --------------
    // NOTE: This is the single-core version — CRSF parsing runs HERE
    // on Core 0, competing with the control loop for CPU time.
    // Enable BENCHMARK_MODE to measure the jitter this introduces,
    // then compare against Curio_FlightController_DualCore.
    getCommands();
    failSafe();

    // ---- Regulate loop rate to 2 kHz ---------------------------
    loopRate(LOOP_RATE_HZ);

    // ---- Benchmark accumulation --------------------------------
#ifdef BENCHMARK_MODE
    uint32_t bm_exec = micros() - bm_loop_start;
    bm_sumUs += bm_exec;
    if (bm_exec < bm_minUs) bm_minUs = bm_exec;
    if (bm_exec > bm_maxUs) bm_maxUs = bm_exec;
    bm_iterCount++;
    if (millis() - bm_windowStart >= 1000) {
        bm_windowStart = millis();
        uint32_t mean = (bm_iterCount > 0) ? (bm_sumUs / bm_iterCount) : 0;
        Serial.println(F("──── Core 0 Loop Benchmark [SINGLE-CORE] ────"));
        Serial.print(F("  Iterations : ")); Serial.println(bm_iterCount);
        Serial.print(F("  Mean dt    : ")); Serial.print(mean);         Serial.println(F(" µs"));
        Serial.print(F("  Min  dt    : ")); Serial.print(bm_minUs);     Serial.println(F(" µs"));
        Serial.print(F("  Max  dt    : ")); Serial.print(bm_maxUs);     Serial.println(F(" µs"));
        Serial.print(F("  Jitter     : ")); Serial.print(bm_maxUs - bm_minUs); Serial.println(F(" µs  ← compare with DualCore version"));
        Serial.println(F("  (CRSF parsing runs on Core 0 — inflates jitter)"));
        Serial.println();
        bm_iterCount = 0; bm_sumUs = 0;
        bm_minUs = UINT32_MAX; bm_maxUs = 0;
    }
#endif
}

// ================================================================
//  IMU DATA ACQUISITION
// ================================================================

void getIMUdata()
{
    // Fetch atomic snapshot from the 2 kHz ISR
    IMURaw raw = bmi088GetLatest();

    // Accel: apply bias correction and LP filter
    float ax = raw.ax - AccErrorX;
    float ay = raw.ay - AccErrorY;
    float az = raw.az - AccErrorZ;
    AccX = (1.0f - B_accel) * AccX_prev + B_accel * ax;
    AccY = (1.0f - B_accel) * AccY_prev + B_accel * ay;
    AccZ = (1.0f - B_accel) * AccZ_prev + B_accel * az;
    AccX_prev = AccX; AccY_prev = AccY; AccZ_prev = AccZ;

    // Gyro: apply bias correction and LP filter
    float gx = raw.gx - GyroErrorX;
    float gy = raw.gy - GyroErrorY;
    float gz = raw.gz - GyroErrorZ;
    GyroX = (1.0f - B_gyro) * GyroX_prev + B_gyro * gx;
    GyroY = (1.0f - B_gyro) * GyroY_prev + B_gyro * gy;
    GyroZ = (1.0f - B_gyro) * GyroZ_prev + B_gyro * gz;
    GyroX_prev = GyroX; GyroY_prev = GyroY; GyroZ_prev = GyroZ;
}

// ================================================================
//  MADGWICK 6-DOF AHRS
//  gx, gy, gz : degrees/second
//  ax, ay, az : g
//  invSampleFreq = dt
// ================================================================

void Madgwick6DOF(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float invSampleFreq)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
    float _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyro deg/s → rad/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Accel feedback only if valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay
             - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay
             - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    }

    // Integrate
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // Normalise
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;

    // Euler angles (NWU)
    roll_IMU  =  atan2f(q0*q1 + q2*q3,  0.5f - q1*q1 - q2*q2) * 57.29577951f;
    pitch_IMU = -asinf(constrain(-2.0f * (q1*q3 - q0*q2), -0.999999f, 0.999999f)) * 57.29577951f;
    yaw_IMU   = -atan2f(q1*q2 + q0*q3,  0.5f - q2*q2 - q3*q3) * 57.29577951f;
}

// ================================================================
//  DESIRED STATE (normalized RC → physical units)
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
//  ALTITUDE HOLD — runs at 50 Hz via baro
// ================================================================

void updateAltHold()
{
    unsigned long now = micros();
    float dtBaro = (now - baro_prev_time) / 1000000.0f;
    if (dtBaro < (1.0f / BARO_RATE_HZ)) return;  // Not yet time
    baro_prev_time = now;

    // Read barometer
    float pPa  = bmp580ReadPressure();
    float altMSL = bmp580PressureToAltitude(pPa);
    alt_est = altMSL - baroAltHome;  // AGL

    // Estimate climb rate
    climb_rate_est = (alt_est - alt_prev) / dtBaro;
    alt_prev = alt_est;

    // When CH6 > 1500: engage altitude hold; otherwise track pilot throttle
    if (channel_6_pwm > 1500 && armedFly) {
        // Outer P loop: altitude error → desired climb rate
        float error_alt = alt_des - alt_est;
        climb_rate_des = constrain(Kp_alt * error_alt,
                                   -MAX_CLIMB_RATE, MAX_CLIMB_RATE);

        // Inner PID: climb-rate error → throttle trim
        float error_climb = climb_rate_des - climb_rate_est;
        integral_climb = integral_climb_prev + error_climb * dtBaro;
        integral_climb = constrain(integral_climb, -i_limit, i_limit);
        float deriv_climb = (error_climb - error_climb_prev) / dtBaro;
        thro_alt_trim = Kp_climb * error_climb
                      + Ki_climb * integral_climb
                      + Kd_climb * deriv_climb;
        thro_alt_trim = constrain(thro_alt_trim, -0.3f, 0.3f);

        integral_climb_prev = integral_climb;
        error_climb_prev    = error_climb;

        // Override thro_des
        thro_des = constrain(0.5f + thro_alt_trim, 0.05f, 0.95f);
    } else {
        // Not in alt-hold: set desired altitude to current altitude
        alt_des = alt_est;
        thro_alt_trim = 0.0f;
        integral_climb = 0.0f;
        integral_climb_prev = 0.0f;
        error_climb_prev = 0.0f;
    }
}

// ================================================================
//  CONTROL — Angle mode (single PID loop)
// ================================================================

void controlANGLE()
{
    // Roll
    error_roll = roll_des - roll_IMU;
    integral_roll = integral_roll_prev + error_roll * dt;
    if (channel_1_pwm < 1060) integral_roll = 0.0f;
    integral_roll = constrain(integral_roll, -i_limit, i_limit);
    derivative_roll = GyroX;  // Use gyro directly as rate feedback
    roll_PID = 0.01f * (Kp_roll_angle * error_roll
                      + Ki_roll_angle * integral_roll
                      - Kd_roll_angle * derivative_roll);

    // Pitch
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch = integral_pitch_prev + error_pitch * dt;
    if (channel_1_pwm < 1060) integral_pitch = 0.0f;
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
    derivative_pitch = GyroY;
    pitch_PID = 0.01f * (Kp_pitch_angle * error_pitch
                       + Ki_pitch_angle * integral_pitch
                       - Kd_pitch_angle * derivative_pitch);

    // Yaw (rate control always)
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw * dt;
    if (channel_1_pwm < 1060) integral_yaw = 0.0f;
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
    derivative_yaw = (error_yaw - error_yaw_prev) / dt;
    yaw_PID = 0.01f * (Kp_yaw * error_yaw
                     + Ki_yaw * integral_yaw
                     + Kd_yaw * derivative_yaw);

    integral_roll_prev  = integral_roll;
    integral_pitch_prev = integral_pitch;
    integral_yaw_prev   = integral_yaw;
    error_yaw_prev      = error_yaw;
}

// ================================================================
//  CONTROL — Cascaded angle→rate mode (outer angle + inner rate PID)
// ================================================================

void controlANGLE2()
{
    float roll_des_ol, pitch_des_ol;

    // --- Outer loop (angle PID → rate setpoint) ---
    error_roll = roll_des - roll_IMU;
    integral_roll_ol = integral_roll_prev_ol + error_roll * dt;
    if (channel_1_pwm < 1060) integral_roll_ol = 0.0f;
    integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit);
    roll_des_ol = Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll_ol;

    error_pitch = pitch_des - pitch_IMU;
    integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
    if (channel_1_pwm < 1060) integral_pitch_ol = 0.0f;
    integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit);
    pitch_des_ol = Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch_ol;

    // Apply gain + damping LP filter
    const float Kl = 30.0f;
    roll_des_ol  = constrain(Kl * roll_des_ol,  -240.0f, 240.0f);
    pitch_des_ol = constrain(Kl * pitch_des_ol, -240.0f, 240.0f);
    roll_des_ol  = (1.0f - B_loop_roll)  * roll_des_prev  + B_loop_roll  * roll_des_ol;
    pitch_des_ol = (1.0f - B_loop_pitch) * pitch_des_prev + B_loop_pitch * pitch_des_ol;

    // --- Inner loop (rate PID) ---
    error_roll = roll_des_ol - GyroX;
    integral_roll_il = integral_roll_prev_il + error_roll * dt;
    if (channel_1_pwm < 1060) integral_roll_il = 0.0f;
    integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit);
    derivative_roll  = (error_roll - error_roll_prev) / dt;
    roll_PID = 0.01f * (Kp_roll_rate * error_roll
                      + Ki_roll_rate * integral_roll_il
                      + Kd_roll_rate * derivative_roll);

    error_pitch = pitch_des_ol - GyroY;
    integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
    if (channel_1_pwm < 1060) integral_pitch_il = 0.0f;
    integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit);
    derivative_pitch  = (error_pitch - error_pitch_prev) / dt;
    pitch_PID = 0.01f * (Kp_pitch_rate * error_pitch
                       + Ki_pitch_rate * integral_pitch_il
                       + Kd_pitch_rate * derivative_pitch);

    // Yaw rate
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw * dt;
    if (channel_1_pwm < 1060) integral_yaw = 0.0f;
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
    derivative_yaw = (error_yaw - error_yaw_prev) / dt;
    yaw_PID = 0.01f * (Kp_yaw * error_yaw
                     + Ki_yaw * integral_yaw
                     + Kd_yaw * derivative_yaw);

    // Save states
    integral_roll_prev_ol  = integral_roll_ol;
    integral_roll_prev_il  = integral_roll_il;
    error_roll_prev        = error_roll;
    roll_IMU_prev          = roll_IMU;
    roll_des_prev          = roll_des_ol;

    integral_pitch_prev_ol = integral_pitch_ol;
    integral_pitch_prev_il = integral_pitch_il;
    error_pitch_prev       = error_pitch;
    pitch_IMU_prev         = pitch_IMU;
    pitch_des_prev         = pitch_des_ol;

    integral_yaw_prev = integral_yaw;
    error_yaw_prev    = error_yaw;
}

// ================================================================
//  CONTROL — Rate mode (acro)
// ================================================================

void controlRATE()
{
    error_roll = roll_des - GyroX;
    integral_roll = integral_roll_prev + error_roll * dt;
    if (channel_1_pwm < 1060) integral_roll = 0.0f;
    integral_roll = constrain(integral_roll, -i_limit, i_limit);
    derivative_roll = (error_roll - error_roll_prev) / dt;
    roll_PID = 0.01f * (Kp_roll_rate * error_roll
                      + Ki_roll_rate * integral_roll
                      + Kd_roll_rate * derivative_roll);

    error_pitch = pitch_des - GyroY;
    integral_pitch = integral_pitch_prev + error_pitch * dt;
    if (channel_1_pwm < 1060) integral_pitch = 0.0f;
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
    derivative_pitch = (error_pitch - error_pitch_prev) / dt;
    pitch_PID = 0.01f * (Kp_pitch_rate * error_pitch
                       + Ki_pitch_rate * integral_pitch
                       + Kd_pitch_rate * derivative_pitch);

    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw * dt;
    if (channel_1_pwm < 1060) integral_yaw = 0.0f;
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
    derivative_yaw = (error_yaw - error_yaw_prev) / dt;
    yaw_PID = 0.01f * (Kp_yaw * error_yaw
                     + Ki_yaw * integral_yaw
                     + Kd_yaw * derivative_yaw);

    error_roll_prev  = error_roll;  integral_roll_prev  = integral_roll;
    error_pitch_prev = error_pitch; integral_pitch_prev = integral_pitch;
    error_yaw_prev   = error_yaw;   integral_yaw_prev   = integral_yaw;
}

// ================================================================
//  MOTOR MIXER  — X-quad configuration
//
//   M1(FL)  M2(FR)        + roll  = right side up  → M1↑ M4↑, M2↓ M3↓
//     \    /              + pitch = nose down       → M3↑ M4↑, M1↓ M2↓
//      \  /               + yaw   = CW nose         → M1↑ M3↑, M2↓ M4↓
//      /  \               (adjacent motors spin same direction in X-config)
//     /    \
//   M4(RL)  M3(RR)
// ================================================================

void controlMixer()
{
    m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;  // FL
    m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;  // FR
    m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;  // RR
    m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;  // RL
}

// ================================================================
//  SCALE COMMANDS  — normalized 0–1 → 10-bit PWM (0–1023)
// ================================================================

void scaleCommands()
{
    // Scale to 10-bit integer; minimum armed value keeps MOSFET gate-drive alive
    m1_command_PWM = (int)(m1_command_scaled * MOTOR_PWM_MAX);
    m2_command_PWM = (int)(m2_command_scaled * MOTOR_PWM_MAX);
    m3_command_PWM = (int)(m3_command_scaled * MOTOR_PWM_MAX);
    m4_command_PWM = (int)(m4_command_scaled * MOTOR_PWM_MAX);

    m1_command_PWM = constrain(m1_command_PWM, 0, MOTOR_PWM_MAX);
    m2_command_PWM = constrain(m2_command_PWM, 0, MOTOR_PWM_MAX);
    m3_command_PWM = constrain(m3_command_PWM, 0, MOTOR_PWM_MAX);
    m4_command_PWM = constrain(m4_command_PWM, 0, MOTOR_PWM_MAX);
}

// ================================================================
//  COMMAND MOTORS
// ================================================================

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
    // Arm: CH5 (arm switch) < 1500 AND throttle < 1060
    if ((channel_5_pwm < 1500) && (channel_1_pwm < 1060)) {
        armedFly = true;
    }
}

void throttleCut()
{
    // CH5 > 1500 → disarmed / throttle cut
    if (channel_5_pwm > 1500) {
        armedFly = false;
        m1_command_PWM = 0;
        m2_command_PWM = 0;
        m3_command_PWM = 0;
        m4_command_PWM = 0;
    }

    // Force zero if not armed
    if (!armedFly) {
        m1_command_PWM = 0;
        m2_command_PWM = 0;
        m3_command_PWM = 0;
        m4_command_PWM = 0;
    }
}

// ================================================================
//  GET COMMANDS — drain CRSF parser
// ================================================================

void getCommands()
{
    parseCRSF();

    // First-order LP filter to smooth RC signal
    float B_rc = 0.7f;
    channel_1_pwm = (unsigned long)((1.0f - B_rc) * channel_1_pwm_prev + B_rc * channel_1_pwm);
    channel_2_pwm = (unsigned long)((1.0f - B_rc) * channel_2_pwm_prev + B_rc * channel_2_pwm);
    channel_3_pwm = (unsigned long)((1.0f - B_rc) * channel_3_pwm_prev + B_rc * channel_3_pwm);
    channel_4_pwm = (unsigned long)((1.0f - B_rc) * channel_4_pwm_prev + B_rc * channel_4_pwm);

    channel_1_pwm_prev = channel_1_pwm;
    channel_2_pwm_prev = channel_2_pwm;
    channel_3_pwm_prev = channel_3_pwm;
    channel_4_pwm_prev = channel_4_pwm;
}

void failSafe()
{
    // If no CRSF frame received within timeout, apply failsafe values
    if ((millis() - crsf_last_frame_ms) > CRSF_TIMEOUT_MS) {
        channel_1_pwm = channel_1_fs;
        channel_2_pwm = channel_2_fs;
        channel_3_pwm = channel_3_fs;
        channel_4_pwm = channel_4_fs;
        channel_5_pwm = channel_5_fs;  // → disarm
        channel_6_pwm = channel_6_fs;
    }

    // Sanity range check
    unsigned long lims[6] = { channel_1_pwm, channel_2_pwm, channel_3_pwm,
                               channel_4_pwm, channel_5_pwm, channel_6_pwm };
    unsigned long fs[6]   = { channel_1_fs, channel_2_fs, channel_3_fs,
                               channel_4_fs, channel_5_fs, channel_6_fs };
    for (int i = 0; i < 6; i++) {
        if (lims[i] > 2100 || lims[i] < 900) {
            *((&channel_1_pwm) + i) = fs[i];
        }
    }
}

// ================================================================
//  IMU CALIBRATION UTILITY
//  Place craft LEVEL and STATIONARY, call once in setup().
//  Copy the printed values into the user-specified section above.
// ================================================================

void calculate_IMU_error()
{
    Serial.println("[CAL] IMU bias calibration — keep craft level and still...");
    delay(2000);

    float axi = 0, ayi = 0, azi = 0;
    float gxi = 0, gyi = 0, gzi = 0;
    const int N = 12000;

    for (int c = 0; c < N; c++) {
        IMURaw raw = bmi088GetLatest();
        axi += raw.ax; ayi += raw.ay; azi += raw.az;
        gxi += raw.gx; gyi += raw.gy; gzi += raw.gz;
        delayMicroseconds(500);
    }
    axi /= N; ayi /= N; azi /= N;
    gxi /= N; gyi /= N; gzi /= N;
    azi -= 1.0f;  // Subtract 1 g on Z-axis (gravity)

    Serial.println("[CAL] Paste these values into config section:");
    Serial.print("float AccErrorX  = "); Serial.print(axi, 6); Serial.println(";");
    Serial.print("float AccErrorY  = "); Serial.print(ayi, 6); Serial.println(";");
    Serial.print("float AccErrorZ  = "); Serial.print(azi, 6); Serial.println(";");
    Serial.print("float GyroErrorX = "); Serial.print(gxi, 6); Serial.println(";");
    Serial.print("float GyroErrorY = "); Serial.print(gyi, 6); Serial.println(";");
    Serial.print("float GyroErrorZ = "); Serial.print(gzi, 6); Serial.println(";");
    Serial.println("[CAL] Done. Comment out calculate_IMU_error() and re-flash.");
    while (1);
}

// ================================================================
//  MADGWICK WARM-UP  (10 000 iterations ≈ 5 s at 2 kHz)
// ================================================================

void calibrateAttitude()
{
    for (int i = 0; i <= 10000; i++) {
        prev_time    = current_time;
        current_time = micros();
        dt = (current_time - prev_time) / 1000000.0f;
        getIMUdata();
        Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);
        loopRate(LOOP_RATE_HZ);
    }
}

// ================================================================
//  LOOP RATE REGULATOR
// ================================================================

void loopRate(int freq)
{
    float invFreq = 1.0f / freq * 1000000.0f;
    unsigned long checker = micros();
    while (invFreq > (float)(checker - current_time)) {
        checker = micros();
    }
}

// ================================================================
//  LED BLINKER
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

void setupBlink(int numBlinks, int upTime, int downTime)
{
    for (int j = 0; j < numBlinks; j++) {
        digitalWrite(PIN_LED, LOW);  delay(downTime);
        digitalWrite(PIN_LED, HIGH); delay(upTime);
    }
}

// ================================================================
//  FAST INVERSE SQUARE ROOT
// ================================================================

float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

// ================================================================
//  DEBUG PRINT UTILITIES  (call one per loop iteration max)
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

void printDesiredState()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("thro:")); Serial.print(thro_des);
        Serial.print(F(" roll:")); Serial.print(roll_des);
        Serial.print(F(" pitch:")); Serial.print(pitch_des);
        Serial.print(F(" yaw:")); Serial.println(yaw_des);
    }
}

void printGyroData()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("Gx:")); Serial.print(GyroX);
        Serial.print(F(" Gy:")); Serial.print(GyroY);
        Serial.print(F(" Gz:")); Serial.println(GyroZ);
    }
}

void printAccelData()
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("Ax:")); Serial.print(AccX);
        Serial.print(F(" Ay:")); Serial.print(AccY);
        Serial.print(F(" Az:")); Serial.println(AccZ);
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
        Serial.print(F("dt(us):"));
        Serial.println(dt * 1000000.0f);
    }
}

void printBatteryVoltage()
{
    if (current_time - print_counter > 100000) {  // 10 Hz
        print_counter = micros();
        analogReadResolution(12);
        float vbat = analogRead(PIN_VBAT) * VBAT_SCALE;
        Serial.print(F("Vbat:"));
        Serial.print(vbat, 2);
        Serial.println(F(" V"));
        if (vbat < VBAT_LOW_WARN * 4.0f) {  // Assume 4S; adjust as needed
            Serial.println(F("*** LOW BATTERY WARNING ***"));
        }
    }
}

void printAltitude()
{
    if (current_time - print_counter > 20000) {  // 50 Hz
        print_counter = micros();
        Serial.print(F("Alt(AGL):"));
        Serial.print(alt_est, 2);
        Serial.print(F("m  climbRate:"));
        Serial.print(climb_rate_est, 2);
        Serial.println(F("m/s"));
    }
}
