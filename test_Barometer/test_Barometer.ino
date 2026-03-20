// ================================================================
//  test_Barometer.ino — BMP580 Barometer Verification Tool
//  RB-RP2354A Flight Controller Board
//
//  Purpose:
//    1. Verify BMP580 chip ID over I²C (address 0x47)
//    2. Stream pressure (Pa), temperature (°C), and altitude (m)
//    3. Measure output noise and compute 1-σ altitude standard deviation
//    4. Demonstrate low-pass filtering for altitude hold usage
//
//  Build: Arduino IDE 2.3.x + earlephilhower/arduino-pico ≥ 3.x
//         Board: "Raspberry Pi Pico 2"
//
//  Hardware connections:
//    SDA = GPIO16   SCL = GPIO17   (I²C0)
//    SDO pin on BMP580 must be tied to VDD → I²C address 0x47
//
//  Expected output (sea level, room temperature ≈ 25 °C):
//    Pressure    : ~101 000 – 103 000 Pa
//    Temperature : ~20 – 35 °C
//    Altitude    : depends on your location; std-dev < 0.3 m indoors
//
//  Commands (Serial Monitor, 500000 baud):
//    s  — stream live data at 10 Hz (default)
//    n  — run noise / std-dev test (200 samples)
//    q  — stop streaming
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "../RB_RP2354_FlightController/config.h"
#include "../RB_RP2354_FlightController/src/bmp580_driver.h"

// ---- Simple first-order IIR for altitude noise reduction ----
static float altFiltered = 0.0f;
static const float B_BARO_LP = 0.1f;   // Higher = less smoothing

// ---- Reference altitude set at startup ----
static float altHomeM = 0.0f;

// ----------------------------------------------------------------

void runNoiseTest()
{
    Serial.println("\n============================================");
    Serial.println(" NOISE / STANDARD DEVIATION TEST");
    Serial.println(" Collecting 200 samples @ 50 Hz...");
    Serial.println("============================================");

    const int N = 200;
    float samples[N];
    float sum = 0.0f;

    for (int i = 0; i < N; i++) {
        float p = bmp580ReadPressure();
        samples[i] = bmp580PressureToAltitude(p);
        sum += samples[i];
        delay(20);   // 50 Hz
        if (i % 20 == 0) { Serial.print('.'); }
    }
    Serial.println();

    float mean = sum / N;

    float variance = 0.0f;
    float minV =  1e9f, maxV = -1e9f;
    for (int i = 0; i < N; i++) {
        float d = samples[i] - mean;
        variance += d * d;
        if (samples[i] < minV) minV = samples[i];
        if (samples[i] > maxV) maxV = samples[i];
    }
    variance /= N;
    float stddev = sqrtf(variance);

    Serial.print("\n Mean altitude : "); Serial.print(mean, 3); Serial.println(" m MSL");
    Serial.print(" Std-dev (1σ)  : "); Serial.print(stddev, 4); Serial.println(" m");
    Serial.print(" Peak-to-peak  : "); Serial.print(maxV - minV, 4); Serial.println(" m");
    Serial.print(" Pressure now  : "); Serial.print(bmp580ReadPressure(), 1); Serial.println(" Pa");
    Serial.print(" Temperature   : "); Serial.print(bmp580ReadTemperature(), 2); Serial.println(" °C");

    if (stddev < 0.30f)
        Serial.println("\n [OK]  Noise level acceptable for altitude hold.");
    else
        Serial.println("\n [WARN] High noise — check for vibrations or heater nearby.");

    Serial.println("\nEnter 's' to stream, 'n' to repeat test.");
}

void printMenu()
{
    Serial.println("\n========================================");
    Serial.println(" RB-RP2354A  Barometer Test Utility");
    Serial.println("========================================");
    Serial.println(" s  — Stream data at 10 Hz");
    Serial.println(" n  — Run noise / std-dev test");
    Serial.println(" q  — Stop streaming");
    Serial.println("========================================");
}

void setup()
{
    Serial.begin(500000);
    delay(600);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    Serial.println("\n[INIT] Initialising BMP580...");
    Serial.print("  I²C SDA=GPIO"); Serial.print(PIN_SDA);
    Serial.print("  SCL=GPIO");     Serial.print(PIN_SCL);
    Serial.print("  Addr=0x");      Serial.println(BMP580_I2C_ADDR, HEX);

    if (!bmp580Init()) {
        Serial.println("[ERROR] BMP580 not found!");
        Serial.println("  Check wiring and that SDO is tied to VDD (addr 0x47).");
        while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(300); }
    }
    Serial.println("[OK]  BMP580 found and configured (50 Hz, 16× OSR pressure).");

    // Warm up — allow IIR filter in chip to settle
    Serial.println("[INIT] Warming up sensor (1 s)...");
    delay(1000);

    // Record home altitude
    float sumP = 0.0f;
    for (int i = 0; i < 50; i++) { sumP += bmp580ReadPressure(); delay(20); }
    altHomeM  = bmp580PressureToAltitude(sumP / 50.0f);
    altFiltered = altHomeM;

    Serial.print("[INIT] Home altitude: "); Serial.print(altHomeM, 1); Serial.println(" m MSL");
    printMenu();
}

void loop()
{
    static bool streaming = false;
    static unsigned long lastDisplay = 0;

    // --- Menu input ---
    if (Serial.available()) {
        char c = Serial.read();
        if      (c == 's') { streaming = true;  Serial.println("\n[STREAM] Ctrl+Q or 'q' to stop."); }
        else if (c == 'q') { streaming = false; printMenu(); }
        else if (c == 'n') { streaming = false; runNoiseTest(); printMenu(); }
    }

    // --- Stream at 10 Hz ---
    if (streaming && (millis() - lastDisplay >= 100)) {
        lastDisplay = millis();

        float pPa    = bmp580ReadPressure();
        float tempC  = bmp580ReadTemperature();
        float altMSL = bmp580PressureToAltitude(pPa);
        float altAGL = altMSL - altHomeM;

        // Apply LP filter
        altFiltered = (1.0f - B_BARO_LP) * altFiltered + B_BARO_LP * altMSL;
        float altFiltAGL = altFiltered - altHomeM;

        Serial.print("P:");     Serial.print(pPa, 1);      Serial.print(" Pa  ");
        Serial.print("T:");     Serial.print(tempC, 2);     Serial.print(" C  ");
        Serial.print("Alt:");   Serial.print(altMSL, 2);    Serial.print(" m MSL  ");
        Serial.print("AGL:");   Serial.print(altAGL, 2);    Serial.print(" m  ");
        Serial.print("Filt:");  Serial.print(altFiltAGL, 2); Serial.println(" m");

        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
}
