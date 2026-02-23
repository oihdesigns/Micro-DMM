/*
 * PixelShieldController.ino
 *
 * Controls an Adafruit NeoPixel Shield (5x8 RGBW, 40 LEDs, data pin 6)
 * on an Arduino UNO R4 Minima via USB Serial commands from the
 * Pixel Shield Android app or PC GUI.
 *
 * Requires: Adafruit NeoPixel library (install via Arduino Library Manager)
 *
 * USB power note: 40 LEDs at full brightness can draw ~2.4A.
 * Brightness is capped at ~20% (50/255) so peak draw stays under 500mA.
 * If you have a separate 5V power supply for the shield you can raise it.
 *
 * Protocol (115200 baud, newline-terminated commands):
 *   C:r,g,b   — solid colour (r,g,b 0-255), stops rainbow
 *   R:hz       — rainbow cycle at given Hz (e.g. R:1.50)
 *   OFF        — all pixels off, stops rainbow
 *   N:count    — set number of active LEDs (1-255)
 *   B:bright   — set brightness (0-255; default 50 ≈ 20%)
 */

#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN    6
#define PIXEL_COUNT  40
#define BRIGHTNESS   50    // 0-255; 50 ≈ 20% — safe for USB power
#define BAUD_RATE    115200

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRBW + NEO_KHZ800);

// ---- State ----
enum Mode { SOLID, RAINBOW };

static Mode     currentMode  = SOLID;
static uint8_t  solidR = 0, solidG = 0, solidB = 0;
static float    rainbowHz    = 1.0f;
static float    rainbowPhase = 0.0f;   // 0 – 360 degrees
static uint32_t lastUs       = 0;

// ---- Serial line buffer ----
static char    lineBuf[64];
static uint8_t lineLen = 0;

// ---- HSV → RGB (full saturation & value) ----
static void hsvToRgb(float hue, uint8_t &r, uint8_t &g, uint8_t &b) {
    float c  = 1.0f;
    float x  = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
    float r1, g1, b1;
    if      (hue < 60)  { r1 = c; g1 = x; b1 = 0; }
    else if (hue < 120) { r1 = x; g1 = c; b1 = 0; }
    else if (hue < 180) { r1 = 0; g1 = c; b1 = x; }
    else if (hue < 240) { r1 = 0; g1 = x; b1 = c; }
    else if (hue < 300) { r1 = x; g1 = 0; b1 = c; }
    else                { r1 = c; g1 = 0; b1 = x; }
    r = (uint8_t)(r1 * 255);
    g = (uint8_t)(g1 * 255);
    b = (uint8_t)(b1 * 255);
}

static void applySolid(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < (int)strip.numPixels(); i++)
        strip.setPixelColor(i, strip.Color(r, g, b));
    strip.show();
}

// Each pixel offset around the wheel so the whole strip looks like a rainbow
static void applyRainbow(float phase) {
    const int   n    = (int)strip.numPixels();
    const float step = 360.0f / (n > 0 ? n : 1);
    for (int i = 0; i < n; i++) {
        float hue = fmodf(phase + i * step, 360.0f);
        uint8_t r, g, b;
        hsvToRgb(hue, r, g, b);
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

static void processLine(const char *line) {
    if (strncmp(line, "C:", 2) == 0) {
        int r, g, b;
        if (sscanf(line + 2, "%d,%d,%d", &r, &g, &b) == 3) {
            solidR = (uint8_t)constrain(r, 0, 255);
            solidG = (uint8_t)constrain(g, 0, 255);
            solidB = (uint8_t)constrain(b, 0, 255);
            currentMode = SOLID;
            applySolid(solidR, solidG, solidB);
        }
    } else if (strncmp(line, "R:", 2) == 0) {
        float hz = atof(line + 2);
        if (hz > 0.0f) {
            rainbowHz   = hz;
            currentMode = RAINBOW;
            lastUs      = micros();
        }
    } else if (strcmp(line, "OFF") == 0) {
        currentMode = SOLID;
        solidR = solidG = solidB = 0;
        applySolid(0, 0, 0);
    } else if (strncmp(line, "N:", 2) == 0) {
        int n = atoi(line + 2);
        if (n >= 1 && n <= 255) {
            strip.updateLength((uint16_t)n);
            // Re-apply current state to the resized strip
            if (currentMode == SOLID) applySolid(solidR, solidG, solidB);
            else strip.show();
        }
    } else if (strncmp(line, "B:", 2) == 0) {
        int b = atoi(line + 2);
        if (b >= 0 && b <= 255) {
            strip.setBrightness((uint8_t)b);
            // Re-apply so the brightness change is visible immediately
            if (currentMode == SOLID) applySolid(solidR, solidG, solidB);
            else strip.show();
        }
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
    delay(1000);
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show();   // all off
    lastUs = micros();
}

void loop() {
    // Read incoming serial bytes and process complete lines
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                processLine(lineBuf);
                lineLen = 0;
            }
        } else if (lineLen < (uint8_t)(sizeof(lineBuf) - 1)) {
            lineBuf[lineLen++] = c;
        }
    }

    // Advance rainbow animation
    if (currentMode == RAINBOW) {
        uint32_t now     = micros();
        uint32_t elapsed = now - lastUs;
        lastUs = now;
        rainbowPhase += rainbowHz * 360.0f * (elapsed / 1000000.0f);
        if (rainbowPhase >= 360.0f) rainbowPhase -= 360.0f;
        applyRainbow(rainbowPhase);
    }
}
