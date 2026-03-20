// ================================================================
//  test_IMU.ino — BMI088 Sensor Verification & Calibration Tool
//  RB-RP2354A Flight Controller Board
//
//  Purpose:
//    1. Verify BMI088 chip IDs over SPI
//    2. Collect static bias (IMU calibration)
//    3. Stream live filtered sensor data for visual inspection
//
//  Build: Arduino IDE 2.3.x + earlephilhower/arduino-pico ≥ 3.x
//         Board: "Raspberry Pi Pico 2"
//
//  Usage:
//    Step 1  – Flash and open Serial Monitor at 500000 baud.
//    Step 2  – Place the board flat and motionless, enter 'c' → calibrate.
//    Step 3  – Copy the printed AccError / GyroError values into
//              config.h of the main flight controller.
//    Step 4  – Enter 's' to stream live data. Tilt the board and
//              confirm roll/pitch angles respond correctly.
//              With board level: Ax≈0, Ay≈0, Az≈+1 g; Gx/Gy/Gz≈0 deg/s.
// ================================================================

#include <Arduino.h>
#include <SPI.h>

// Adjust path if needed relative to this sketch folder
#define CONFIG_H_STANDALONE   // Tell config.h we are in a standalone test
#include "../RB_RP2354_FlightController/config.h"
#include "../RB_RP2354_FlightController/src/bmi088_driver.h"

// ---- Madgwick state for attitude display ----
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;

static float invSqrt(float x) { return 1.0f / sqrtf(x); }

void madgwick6DOF(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt)
{
    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;
    float qDot1 = 0.5f*(-q1*gx - q2*gy - q3*gz);
    float qDot2 = 0.5f*( q0*gx + q2*gz - q3*gy);
    float qDot3 = 0.5f*( q0*gy - q1*gz + q3*gx);
    float qDot4 = 0.5f*( q0*gz + q1*gy - q2*gx);

    if (!((ax==0.0f)&&(ay==0.0f)&&(az==0.0f))) {
        float rn = invSqrt(ax*ax+ay*ay+az*az);
        ax*=rn; ay*=rn; az*=rn;
        float _2q0=2*q0,_2q1=2*q1,_2q2=2*q2,_2q3=2*q3;
        float _4q0=4*q0,_4q1=4*q1,_4q2=4*q2;
        float _8q1=8*q1,_8q2=8*q2;
        float q0q0=q0*q0,q1q1=q1*q1,q2q2=q2*q2,q3q3=q3*q3;
        float s0=_4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay;
        float s1=_4q1*q3q3-_2q3*ax+4*q0q0*q1-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az;
        float s2=4*q0q0*q2+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az;
        float s3=4*q1q1*q3-_2q1*ax+4*q2q2*q3-_2q2*ay;
        rn=invSqrt(s0*s0+s1*s1+s2*s2+s3*s3);
        s0*=rn;s1*=rn;s2*=rn;s3*=rn;
        qDot1-=0.04f*s0; qDot2-=0.04f*s1;
        qDot3-=0.04f*s2; qDot4-=0.04f*s3;
    }
    q0+=qDot1*dt; q1+=qDot2*dt; q2+=qDot3*dt; q3+=qDot4*dt;
    float rn=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    q0*=rn;q1*=rn;q2*=rn;q3*=rn;
    roll_deg  =  atan2f(q0*q1+q2*q3, 0.5f-q1*q1-q2*q2)*57.29578f;
    pitch_deg = -asinf(constrain(-2.0f*(q1*q3-q0*q2),-0.999999f,0.999999f))*57.29578f;
    yaw_deg   = -atan2f(q1*q2+q0*q3, 0.5f-q2*q2-q3*q3)*57.29578f;
}

// ---- Calibration offsets (filled in during calibration) ----
float accErrX = 0, accErrY = 0, accErrZ = 0;
float gyrErrX = 0, gyrErrY = 0, gyrErrZ = 0;

void runCalibration()
{
    Serial.println();
    Serial.println("============================================");
    Serial.println(" IMU BIAS CALIBRATION");
    Serial.println(" Place board FLAT and STATIONARY.");
    Serial.println(" Collecting 12 000 samples @ 2 kHz...");
    Serial.println("============================================");
    delay(3000);

    double sumAx=0,sumAy=0,sumAz=0;
    double sumGx=0,sumGy=0,sumGz=0;
    const int N = 12000;

    for (int i = 0; i < N; i++) {
        IMURaw r = bmi088GetLatest();
        sumAx+=r.ax; sumAy+=r.ay; sumAz+=r.az;
        sumGx+=r.gx; sumGy+=r.gy; sumGz+=r.gz;
        delayMicroseconds(500);
    }

    accErrX = (float)(sumAx/N);
    accErrY = (float)(sumAy/N);
    accErrZ = (float)(sumAz/N) - 1.0f;  // Subtract 1 g gravity on Z
    gyrErrX = (float)(sumGx/N);
    gyrErrY = (float)(sumGy/N);
    gyrErrZ = (float)(sumGz/N);

    Serial.println("\n--- Results (copy into config.h) ---");
    Serial.print("float AccErrorX  = "); Serial.print(accErrX, 6); Serial.println(";");
    Serial.print("float AccErrorY  = "); Serial.print(accErrY, 6); Serial.println(";");
    Serial.print("float AccErrorZ  = "); Serial.print(accErrZ, 6); Serial.println(";");
    Serial.print("float GyroErrorX = "); Serial.print(gyrErrX, 6); Serial.println(";");
    Serial.print("float GyroErrorY = "); Serial.print(gyrErrY, 6); Serial.println(";");
    Serial.print("float GyroErrorZ = "); Serial.print(gyrErrZ, 6); Serial.println(";");

    // Basic sanity check
    bool ok = true;
    if (fabsf(accErrX) > 0.2f || fabsf(accErrY) > 0.2f) {
        Serial.println("[WARN] Accel X/Y bias > 0.2 g — board may not be level.");
        ok = false;
    }
    if (fabsf(gyrErrX) > 5.0f || fabsf(gyrErrY) > 5.0f || fabsf(gyrErrZ) > 5.0f) {
        Serial.println("[WARN] Gyro bias > 5 deg/s — check for vibrations.");
        ok = false;
    }
    if (ok) Serial.println("[OK]  Calibration looks good.");
    Serial.println("\nEnter 's' to stream live data, 'c' to recalibrate.");
}

void printMenu()
{
    Serial.println("\n========================================");
    Serial.println(" RB-RP2354A  IMU Test Utility");
    Serial.println("========================================");
    Serial.println(" c  — Run bias calibration (board flat)");
    Serial.println(" s  — Stream raw + attitude @ 100 Hz");
    Serial.println(" q  — Stop streaming");
    Serial.println("========================================");
}

void setup()
{
    Serial.begin(500000);
    delay(600);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    Serial.println("\n[INIT] Initialising BMI088...");
    bmi088Init();
    delay(20);

    if (!bmi088CheckIDs()) {
        Serial.println("[ERROR] BMI088 chip ID mismatch! Check SPI wiring:");
        Serial.println("  MOSI=GPIO19  MISO=GPIO20  SCK=GPIO22");
        Serial.println("  ACC_CS=GPIO21  GYR_CS=GPIO23");
        while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(300); }
    }

    Serial.println("[OK]  BMI088 found and configured.");
    printMenu();
}

void loop()
{
    static bool streaming = false;
    static unsigned long lastPrint = 0;
    static unsigned long lastImu   = 0;

    // --- Menu input ---
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'c') { streaming = false; runCalibration(); }
        else if (c == 's') { streaming = true; Serial.println("\n[STREAM] Ctrl+Q or 'q' to stop.\n"); }
        else if (c == 'q') { streaming = false; printMenu(); }
    }

    // --- Stream at 100 Hz ---
    if (streaming && (millis() - lastPrint >= 10)) {
        lastPrint = millis();

        IMURaw raw = bmi088GetLatest();

        // Apply calibration offsets
        float ax = raw.ax - accErrX;
        float ay = raw.ay - accErrY;
        float az = raw.az - accErrZ;
        float gx = raw.gx - gyrErrX;
        float gy = raw.gy - gyrErrY;
        float gz = raw.gz - gyrErrZ;

        // Update Madgwick filter
        float dtSec = (raw.timestamp - lastImu) * 1e-6f;
        if (dtSec > 0.0001f && dtSec < 0.1f)
            madgwick6DOF(gx, -gy, -gz, -ax, ay, az, dtSec);
        lastImu = raw.timestamp;

        // Print columns: Ax Ay Az Gx Gy Gz Roll Pitch Yaw
        Serial.print(ax, 4);  Serial.print('\t');
        Serial.print(ay, 4);  Serial.print('\t');
        Serial.print(az, 4);  Serial.print('\t');
        Serial.print(gx, 2);  Serial.print('\t');
        Serial.print(gy, 2);  Serial.print('\t');
        Serial.print(gz, 2);  Serial.print('\t');
        Serial.print(roll_deg,  2); Serial.print('\t');
        Serial.print(pitch_deg, 2); Serial.print('\t');
        Serial.println(yaw_deg, 2);
    }

    // Heartbeat LED
    static unsigned long ledT = 0;
    if (millis() - ledT > 500) { ledT = millis(); digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }
}
