/*
  CharlieTouchScreen.ino
  Adafruit 2.4" TFT FeatherWing v2 (#3315) — "Spell CHARLIE" touch game.

  The start screen is a 2x2 menu of pictures — a "C" (Charlie), a bike, a
  unicorn, and a house — each picking a word to spell (CHARLIE, BIKE, UNICORN,
  HOUSE).  After choosing, each round shows a 2x2 grid of four letters; one is
  the correct next letter, the rest are random.  Tap the correct letter to
  advance; tap a wrong one and it's back to the picture menu.  The letters
  spelled so far show along the bottom.  Finish the word and the NeoPixel
  celebrates, then a tap returns to the menu.  Letters/positions re-randomize
  every round.

  Touch handling / orientation is copied from DataLogger_TFT so the screen
  reads correctly in landscape (setRotation(1)) and taps land where expected.

  ── Libraries required ──────────────────────────────────────────────────────────
  Adafruit ILI9341         (TFT driver)
  Adafruit TSC2007         (I2C resistive touchscreen — v2 FeatherWing)
  Adafruit GFX Library     (graphics primitives)

  ── Pin assignments (TFT FeatherWing v2 hardwired) ──────────────────────────────
  TFT CS    →  9
  TFT DC    → 10
  TS IRQ    →  6   (TSC2007 touch interrupt)
  TSC2007   →  I2C — see address note below

  ── TSC2007 I2C address ─────────────────────────────────────────────────────────
  Factory default is 0x48.  The board used for DataLogger_TFT had the A0+A1
  jumpers bridged to move it to 0x4B (to dodge an ADS1015 conflict).  This demo
  has no other I2C devices, so set TS_I2C_ADDR to whichever matches your board:
    unmodified FeatherWing → 0x48
    A0+A1 bridged          → 0x4B
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_TSC2007.h>
#include <Adafruit_NeoPixel.h>

// ─── Display / touch pins (FeatherWing v2 hardwired) ────────────────────────
#define TFT_CS    9
#define TFT_DC   10
#define TS_IRQ    6

// ─── NeoPixel (Feather RP2350 onboard) ──────────────────────────────────────
#define PIXEL_PIN  PIN_NEOPIXEL

// ─── TSC2007 I2C address (see note above) ───────────────────────────────────
#define TS_I2C_ADDR  0x4B

// ─── Display geometry (landscape 320×240) ───────────────────────────────────
#define DISP_W   320
#define DISP_H   240
#define FOOTER_H  24                  // bottom strip showing progress
#define GRID_H   (DISP_H - FOOTER_H)  // grid area height (above the footer)

// ─── Colours (RGB565) ───────────────────────────────────────────────────────
#define COL_BG      0x0000   // black
#define COL_LETTER  0x07E0   // green
#define COL_DIV     0x4208   // dark gray grid dividers
#define COL_BIKE    0x07FF   // cyan
#define COL_UNICORN 0xF81F   // magenta
#define COL_HOUSE   0xFD20   // orange
#define COL_CAP     0xC618   // light gray captions

// ─── Touch calibration (landscape rotation 1) — same as DataLogger_TFT ──────
// In landscape rotation=1 the sensor axes are SWAPPED vs portrait:
//   rawY → screenX (horizontal 0-319)
//   rawX → screenY (vertical   0-239, inverted: high rawX = top)
#define TS_RAW_Y_LEFT    578   // rawY at screen left  edge (screenX=0)
#define TS_RAW_Y_RIGHT  3320   // rawY at screen right edge (screenX=DISP_W)
#define TS_RAW_X_TOP    3636   // rawX at screen top   edge (screenY=0)
#define TS_RAW_X_BOT     492   // rawX at screen bot   edge (screenY=DISP_H)
#define TS_MIN_Z          10   // minimum pressure (Z) to register a touch

// ─── Peripheral objects ─────────────────────────────────────────────────────
Adafruit_ILI9341  tft(TFT_CS, TFT_DC);
Adafruit_TSC2007  ts;
Adafruit_NeoPixel pixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
bool tsOK = false;

// ─── Word choices (start-screen menu) ───────────────────────────────────────
enum IconType { ICON_C, ICON_BIKE, ICON_UNICORN, ICON_HOUSE };
struct Choice { const char* word; uint8_t icon; };
const Choice CHOICES[4] = {
  { "CHARLIE", ICON_C       },
  { "BIKE",    ICON_BIKE    },
  { "UNICORN", ICON_UNICORN },
  { "HOUSE",   ICON_HOUSE   },
};

// ─── Game state ─────────────────────────────────────────────────────────────
const char* target    = CHOICES[0].word;  // word currently being spelled
uint8_t     targetLen = 7;

enum GameState { STATE_MENU, STATE_PLAYING, STATE_WIN };
GameState gameState = STATE_MENU;
uint8_t   progress  = 0;        // letters correctly chosen so far (index into target)

char    cellLetter[4];          // letter shown in each of the 4 grid cells
uint8_t correctCell = 0;        // which cell holds the right next letter

// ─── Touch debounce (same scheme as DataLogger_TFT) ─────────────────────────
bool          touchActive   = false;
unsigned long touchStartMs  = 0;
unsigned long lastTouchCheck = 0;
#define TOUCH_DEBOUNCE_MS 180

// Draw a single line of text centred, as large as will fit horizontally.
void drawBigText(const char* text, uint16_t color) {
  uint8_t len = strlen(text);

  // Pick the biggest text size that fits the width (with margin), capped so a
  // single letter doesn't get absurdly large.
  uint8_t sz = (DISP_W - 20) / (len * 6);
  if (sz > 25) sz = 25;
  if (sz < 1)  sz = 1;

  int16_t textW = len * 6 * sz;
  int16_t textH = 8 * sz;

  tft.fillScreen(COL_BG);
  tft.setTextSize(sz);
  tft.setTextColor(color);
  tft.setCursor((DISP_W - textW) / 2, (DISP_H - textH) / 2);
  tft.print(text);
}

// Build a new round: correct letter goes in a random cell, the other three get
// random distinct letters (never matching the correct one or each other).
void newRound() {
  char correct = target[progress];
  correctCell  = random(4);

  bool used[26] = { false };
  used[correct - 'A'] = true;

  for (uint8_t i = 0; i < 4; i++) {
    if (i == correctCell) { cellLetter[i] = correct; continue; }
    char c;
    do { c = 'A' + random(26); } while (used[c - 'A']);
    used[c - 'A'] = true;
    cellLetter[i] = c;
  }
}

// Draw the footer strip showing the letters of CHARLIE spelled so far.
void drawFooter() {
  tft.fillRect(0, GRID_H, DISP_W, FOOTER_H, COL_BG);
  tft.drawFastHLine(0, GRID_H, DISP_W, COL_DIV);

  char buf[16];
  for (uint8_t i = 0; i < progress && i < 15; i++) buf[i] = target[i];
  buf[progress] = '\0';

  const uint8_t SZ = 2;
  tft.setTextSize(SZ);
  tft.setTextColor(COL_LETTER);
  int16_t tw = progress * 6 * SZ;
  tft.setCursor((DISP_W - tw) / 2, GRID_H + (FOOTER_H - 8 * SZ) / 2);
  tft.print(buf);
}

// Draw the 2x2 grid of letter choices, with the progress footer below it.
void drawGrid() {
  const int16_t cw = DISP_W / 2;   // cell width  (160)
  const int16_t ch = GRID_H / 2;   // cell height (108)
  const uint8_t SZ = 12;           // letter size: 72x96 px, fits a cell

  tft.fillScreen(COL_BG);
  tft.drawFastVLine(cw, 0, GRID_H, COL_DIV);
  tft.drawFastHLine(0, ch, DISP_W, COL_DIV);

  tft.setTextSize(SZ);
  tft.setTextColor(COL_LETTER);
  for (uint8_t i = 0; i < 4; i++) {
    int16_t cellX = (i % 2) * cw;
    int16_t cellY = (i / 2) * ch;
    int16_t gw = 6 * SZ, gh = 8 * SZ;
    tft.setCursor(cellX + (cw - gw) / 2, cellY + (ch - gh) / 2);
    tft.print(cellLetter[i]);
  }

  drawFooter();
}

// Celebrate the finished word: blink the onboard NeoPixel through a few
// different colours for a few seconds.
void celebrate() {
  static const uint32_t colors[] = {
    0xFF0000, // red
    0xFF8000, // orange
    0xFFFF00, // yellow
    0x00FF00, // green
    0x00FFFF, // cyan
    0x0000FF, // blue
    0xFF00FF, // magenta
  };
  const uint8_t nColors = sizeof(colors) / sizeof(colors[0]);

  // ~3 seconds: 20 flashes × (120 ms on + 30 ms off)
  for (uint8_t i = 0; i < 20; i++) {
    uint32_t c = colors[i % nColors];
    pixel.setPixelColor(0, pixel.Color((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF));
    pixel.show();
    delay(120);
    pixel.clear();
    pixel.show();
    delay(30);
  }
}

// ─── Start-menu icons — drawn centred on (cx, cy) ───────────────────────────

void drawIconC(int16_t cx, int16_t cy, uint16_t col) {
  const uint8_t SZ = 9;            // 54x72 px
  tft.setTextSize(SZ);
  tft.setTextColor(col);
  tft.setCursor(cx - 3 * SZ, cy - 4 * SZ);
  tft.print('C');
}

void drawIconBike(int16_t cx, int16_t cy, uint16_t col) {
  const int16_t r = 18;
  int16_t wy = cy + 14;
  int16_t lx = cx - 30, rx = cx + 30;   // wheel hubs
  tft.drawCircle(lx, wy, r, col);
  tft.drawCircle(rx, wy, r, col);

  int16_t bb = cx - 2;                   // bottom bracket (pedals) x
  int16_t sx = cx - 12, sy = cy - 18;    // seat
  int16_t hx = cx + 20, hy = cy - 18;    // handlebar
  tft.drawLine(lx, wy, bb, wy, col);     // rear-bottom
  tft.drawLine(bb, wy, sx, sy, col);     // seat tube
  tft.drawLine(sx, sy, lx, wy, col);     // seat stay
  tft.drawLine(bb, wy, hx, hy, col);     // down tube
  tft.drawLine(sx, sy, hx, hy, col);     // top tube
  tft.drawLine(hx, hy, rx, wy, col);     // fork
  tft.drawLine(sx - 6, sy, sx + 6, sy, col);   // seat
  tft.drawLine(hx - 2, hy, hx + 8, hy - 4, col); // handlebar
}

void drawIconUnicorn(int16_t cx, int16_t cy, uint16_t col) {
  // Stylised head facing left: muzzle, head, ear, horn, eye, mane.
  int16_t hx = cx + 6, hy = cy + 2;      // head centre
  tft.fillCircle(hx, hy, 18, col);                 // head
  tft.fillTriangle(hx - 26, hy + 10, hx - 4, hy - 6,
                   hx - 2, hy + 16, col);          // muzzle
  tft.fillTriangle(hx + 6, hy - 14, hx + 16, hy - 6,
                   hx + 4, hy - 2, col);           // ear
  // Horn (yellow), pointing up
  tft.fillTriangle(hx - 8, hy - 16, hx - 2, hy - 16,
                   hx - 10, hy - 40, COL_HOUSE);
  // Eye
  tft.fillCircle(hx - 4, hy - 2, 2, COL_BG);
  // Mane down the back of the neck
  for (int8_t k = 0; k < 4; k++)
    tft.drawCircle(hx + 16, hy - 8 + k * 9, 5, col);
}

void drawIconHouse(int16_t cx, int16_t cy, uint16_t col) {
  int16_t bw = 56, bh = 38;
  int16_t bx = cx - bw / 2, by = cy - 2;
  tft.drawRect(bx, by, bw, bh, col);                       // body
  tft.fillTriangle(cx - bw / 2 - 6, by, cx + bw / 2 + 6, by,
                   cx, by - 28, col);                      // roof
  tft.drawRect(cx - 8, by + bh - 20, 16, 20, col);         // door
}

void drawIcon(uint8_t icon, int16_t cx, int16_t cy) {
  switch (icon) {
    case ICON_C:       drawIconC(cx, cy, COL_LETTER);        break;
    case ICON_BIKE:    drawIconBike(cx, cy, COL_BIKE);       break;
    case ICON_UNICORN: drawIconUnicorn(cx, cy, COL_UNICORN); break;
    case ICON_HOUSE:   drawIconHouse(cx, cy, COL_HOUSE);     break;
  }
}

// Start menu: 2x2 grid of pictures, each picking a word to spell.
void drawMenu() {
  const int16_t cw = DISP_W / 2, ch = DISP_H / 2;
  tft.fillScreen(COL_BG);
  tft.drawFastVLine(cw, 0, DISP_H, COL_DIV);
  tft.drawFastHLine(0, ch, DISP_W, COL_DIV);

  for (uint8_t i = 0; i < 4; i++) {
    int16_t cellX = (i % 2) * cw;
    int16_t cellY = (i / 2) * ch;
    drawIcon(CHOICES[i].icon, cellX + cw / 2, cellY + ch / 2 - 8);

    // Small caption under each picture
    const char* cap = CHOICES[i].word;
    int16_t tw = strlen(cap) * 6;
    tft.setTextSize(1);
    tft.setTextColor(COL_CAP);
    tft.setCursor(cellX + (cw - tw) / 2, cellY + ch - 14);
    tft.print(cap);
  }
}

// TSC2007 raw → screen pixels (landscape rotation 1).
void mapTouch(uint16_t rawX, uint16_t rawY, int16_t* sx, int16_t* sy) {
  *sx = (int16_t)map((long)rawY, TS_RAW_Y_LEFT, TS_RAW_Y_RIGHT, 0, DISP_W);
  *sy = (int16_t)map((long)rawX, TS_RAW_X_TOP,  TS_RAW_X_BOT,  0, DISP_H);
  *sx = constrain(*sx, 0, DISP_W - 1);
  *sy = constrain(*sy, 0, DISP_H - 1);
}

void processTouches() {
  if (!tsOK) return;

  // IRQ HIGH = no touch; clear active flag so next press is treated as new
  if (digitalRead(TS_IRQ)) {
    touchActive = false;
    return;
  }

  // Rate-limit I2C reads
  unsigned long now = millis();
  if (now - lastTouchCheck < 20) return;
  lastTouchCheck = now;

  // Finger still held from a previously handled tap — wait for release
  if (touchActive) return;

  TS_Point p = ts.getPoint();
  if ((p.x == 0 && p.y == 0) || p.z < TS_MIN_Z) return;

  // Cooldown: ignore spurious re-triggers within debounce window
  if (now - touchStartMs < TOUCH_DEBOUNCE_MS) return;

  // Valid new tap — fire once and mark active until IRQ goes HIGH (release)
  touchActive  = true;
  touchStartMs = now;

  int16_t sx, sy;
  mapTouch((uint16_t)p.x, (uint16_t)p.y, &sx, &sy);
  Serial.print(F("Touch screen ")); Serial.print(sx); Serial.print(','); Serial.println(sy);

  // ── Win screen: any tap returns to the word-select menu ──
  if (gameState == STATE_WIN) {
    gameState = STATE_MENU;
    drawMenu();
    return;
  }

  // ── Menu: pick a picture → spell that word ──
  if (gameState == STATE_MENU) {
    uint8_t col  = (sx < DISP_W / 2) ? 0 : 1;
    uint8_t row  = (sy < DISP_H / 2) ? 0 : 1;
    uint8_t pick = row * 2 + col;
    target    = CHOICES[pick].word;
    targetLen = strlen(target);
    progress  = 0;
    gameState = STATE_PLAYING;
    newRound();
    drawGrid();
    return;
  }

  // ── Playing: figure out which grid cell was tapped ──
  uint8_t col  = (sx < DISP_W / 2) ? 0 : 1;
  uint8_t row  = (sy < GRID_H / 2) ? 0 : 1;
  uint8_t cell = row * 2 + col;

  if (cell == correctCell) {
    progress++;
    if (progress >= targetLen) {
      // Win — show the finished word with a "!" and celebrate
      char winText[18];
      snprintf(winText, sizeof(winText), "%s!", target);
      gameState = STATE_WIN;
      drawBigText(winText, COL_LETTER);
      celebrate();
    } else {
      newRound();
      drawGrid();
    }
  } else {
    // Wrong letter — flash red and return to the menu
    pixel.setPixelColor(0, pixel.Color(255, 0, 0)); pixel.show();
    delay(300);
    pixel.clear(); pixel.show();
    gameState = STATE_MENU;
    drawMenu();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("CharlieTouchScreen starting"));

  Wire.begin();
  Wire.setClock(400000);
  SPI.begin();

  pinMode(TS_IRQ, INPUT);   // TSC2007 IRQ: HIGH=idle, LOW=touched — no pullup

  // ── NeoPixel ──
  pixel.begin();
  pixel.setBrightness(80);   // keep it from being blinding
  pixel.clear();
  pixel.show();

  // ── TFT ──
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);

  // ── TSC2007 touch ──
  if (ts.begin(TS_I2C_ADDR, &Wire)) {
    tsOK = true;
    Serial.println(F("TSC2007 touch OK"));
  } else {
    Serial.println(F("TSC2007 not found — check TS_I2C_ADDR"));
    tft.setTextSize(2);
    tft.setTextColor(0xFFE0);  // yellow
    tft.setCursor(10, 100);
    tft.print(F("Touch not found"));
    return;
  }

  // Seed the RNG so letters/positions differ each power-up (A0 floats here).
  randomSeed((uint32_t)analogRead(A0) ^ micros());

  // Show the word-select menu
  gameState = STATE_MENU;
  drawMenu();
  Serial.println(F("Pick a picture, then spell the word!"));
}

void loop() {
  processTouches();
}
