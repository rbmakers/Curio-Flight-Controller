// ================================================================
//  test_ELRS.ino — ExpressLRS / CRSF Receiver Verification Tool
//  RB-RP2354A Flight Controller Board
//  開發 : 火箭鳥創客倉庫
//  Purpose:
//    Decode CRSF frames from the ELRS receiver connected to UART0
//    (TX=GPIO12, RX=GPIO13, 420 kBaud) and display all 16 channel
//    values in µs (1000–2000) at 10 Hz in the Serial Monitor.
//
//  Build: Arduino IDE 2.3.x + earlephilhower/arduino-pico ≥ 3.x
//         Board: "Raspberry Pi Pico 2"
//
//  Hardware connections:
//    ELRS RX module TX  →  RB-RP2354A GPIO13 (UART0 RX)
//    ELRS RX module RX  →  RB-RP2354A GPIO12 (UART0 TX)  [optional, for telemetry]
//    GND                →  GND
//    5V / 3.3V          →  VCC of ELRS module
//
//  Expected values at rest (all sticks centered, switches in default):
//    CH1 (Throttle): ~1000 µs (full down)
//    CH2 (Roll)    : ~1500 µs (center)
//    CH3 (Pitch)   : ~1500 µs (center)
//    CH4 (Yaw)     : ~1500 µs (center)
//    CH5 (Arm)     : ~1000 or ~2000 µs depending on switch position
//
//  If no data appears:
//    • Check baud rate in your ELRS LUA script (must be 420k for CRSF)
//    • Check TX/RX are not swapped
//    • Check receiver is bound and link LED is solid
// ================================================================

#include <Arduino.h>
#include "../RB_RP2354_FlightController/config.h"

// ----------------------------------------------------------------
// CRSF parser (identical to flight controller version)
// ----------------------------------------------------------------
#define CRSF_SYNC_BYTE     0xC8
#define CRSF_TYPE_RC_CHAN  0x16
#define CRSF_MAX_FRAME_LEN 64

static uint8_t  crsfBuf[CRSF_MAX_FRAME_LEN];
static uint8_t  crsfBufIdx  = 0;
static uint8_t  crsfExpLen  = 0;
static bool     crsfInFrame = false;

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

static inline unsigned long crsfToUs(uint16_t raw)
{
    return (unsigned long)constrain(
        (long)raw * 1000L / 1639L + 895L, 1000L, 2000L);
}

// 16 channels decoded
static unsigned long channels[16];
static uint32_t frameCount     = 0;
static uint32_t crcErrorCount  = 0;
static uint32_t lastFrameMs    = 0;

static void crsfDecodeChannels(const uint8_t *p)
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

    for (int i = 0; i < 16; i++) channels[i] = crsfToUs(ch[i]);
    frameCount++;
    lastFrameMs = millis();
}

void parseCRSF()
{
    while (ELRS_SERIAL.available()) {
        uint8_t byte = ELRS_SERIAL.read();

        if (!crsfInFrame) {
            if (byte == CRSF_SYNC_BYTE) {
                crsfInFrame = true;
                crsfBufIdx  = 0;
                crsfBuf[crsfBufIdx++] = byte;
            }
        } else {
            crsfBuf[crsfBufIdx++] = byte;
            if (crsfBufIdx == 2) {
                crsfExpLen = byte + 2;
                if (crsfExpLen > CRSF_MAX_FRAME_LEN) { crsfInFrame = false; }
            }
            if (crsfInFrame && crsfExpLen > 0 && crsfBufIdx >= crsfExpLen) {
                uint8_t rxCrc  = crsfBuf[crsfExpLen - 1];
                uint8_t calCrc = crsfCrc8(&crsfBuf[2], crsfExpLen - 3);
                if (rxCrc == calCrc) {
                    if (crsfBuf[2] == CRSF_TYPE_RC_CHAN)
                        crsfDecodeChannels(&crsfBuf[3]);
                } else {
                    crcErrorCount++;
                }
                crsfInFrame = false;
            }
        }
    }
}

// ----------------------------------------------------------------
// Bar chart helper for visual display
// ----------------------------------------------------------------
void printBar(unsigned long us)
{
    // Map 1000–2000 µs to 0–20 chars
    int len = (int)((us - 1000UL) * 20UL / 1000UL);
    len = constrain(len, 0, 20);
    Serial.print('[');
    for (int i = 0; i < 20; i++) Serial.print(i < len ? '#' : ' ');
    Serial.print("] ");
    Serial.print(us);
    Serial.print(" us");
}

// ----------------------------------------------------------------
void setup()
{
    Serial.begin(500000);
    delay(600);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    ELRS_SERIAL.setTX(PIN_ELRS_TX);
    ELRS_SERIAL.setRX(PIN_ELRS_RX);
    ELRS_SERIAL.begin(ELRS_BAUD);

    Serial.println("\n========================================");
    Serial.println(" RB-RP2354A  ELRS / CRSF Receiver Test");
    Serial.println("========================================");
    Serial.print(" UART:  Serial1  TX=GPIO"); Serial.print(PIN_ELRS_TX);
    Serial.print("  RX=GPIO"); Serial.println(PIN_ELRS_RX);
    Serial.print(" Baud:  "); Serial.println(ELRS_BAUD);
    Serial.println(" Waiting for CRSF frames...\n");

    // Init channels to center
    for (int i = 0; i < 16; i++) channels[i] = 1500;
    channels[0] = 1000;  // Throttle default low
}

void loop()
{
    parseCRSF();

    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay >= 100) {   // Display at 10 Hz
        lastDisplay = millis();

        // Link quality indicator
        bool linked = (millis() - lastFrameMs < 500);
        unsigned long frameRate = (millis() > 1000) ? (frameCount * 1000UL / millis()) : 0;

        Serial.print("\033[2J\033[H");  // ANSI clear screen (works in Arduino Serial Monitor with ANSI mode, and in most terminals)
        Serial.println("========================================");
        Serial.println(" RB-RP2354A  ELRS / CRSF Live Data");
        Serial.println("========================================");
        Serial.print(" Link: ");
        Serial.print(linked ? "CONNECTED" : "NO SIGNAL ");
        Serial.print("   Frames: ");
        Serial.print(frameCount);
        Serial.print("   ~");
        Serial.print(frameRate);
        Serial.print(" fps   CRC errors: ");
        Serial.println(crcErrorCount);
        Serial.println();

        const char *chName[] = {
            "CH1 Throttle", "CH2 Roll    ", "CH3 Pitch   ", "CH4 Yaw     ",
            "CH5 Arm     ", "CH6 Aux1    ", "CH7 Aux2    ", "CH8 Aux3    ",
            "CH9         ", "CH10        ", "CH11        ", "CH12        ",
            "CH13        ", "CH14        ", "CH15        ", "CH16        "
        };

        for (int i = 0; i < 16; i++) {
            Serial.print(' ');
            Serial.print(chName[i]);
            Serial.print("  ");
            printBar(channels[i]);
            Serial.println();
        }

        Serial.println();
        Serial.println(" Move sticks to verify channel assignment.");
        Serial.println(" CH1 throttle full up  → ~2000 us");
        Serial.println(" CH2-CH4 sticks center → ~1500 us");
        Serial.println(" CH5 arm switch        → 1000 or 2000 us");

        // LED: fast blink if linked, slow if not
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
}
