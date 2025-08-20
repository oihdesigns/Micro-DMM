/*
  GIGA R1 WiFi + GIGA Display Shield — Generic Template

  What this sketch gives you (easy to reuse):
  - DIO 2–7 configured as inputs, DIO 8–15 as outputs
  - A0–A7 configured as analog inputs; analogReadResolution set to 16 bits (0–65535)
  - Left side of the display shows live status for DI 2–7, DO 8–15, and A0–A7
  - Right side shows 6 on‑screen buttons laid out 2×3:
      * Left column (3) are LATCHING buttons
      * Right column (3) are MOMENTARY buttons
    Default debounce is 500 ms, adjustable via setDebounceMs().
  - A tiny non‑blocking scheduler for periodic jobs (used here to refresh the UI at 1 Hz).

  Libraries you need (Library Manager):
    - Arduino_GigaDisplay_GFX
    - Arduino_GigaDisplayTouch

  Notes:
  * The GIGA’s ADC hardware is 12‑bit; analogReadResolution(16) scales readings to 0–65535.
  * All code is non‑blocking; no delay() in the main loop. The scheduler uses millis().
  * Hook functions are provided so you can react to button events without digging into the UI code.
*/

#include <Arduino.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>

// ---------- Pins ----------
static const uint8_t PIN_DI[] = {2, 3, 4, 5, 6, 7};        // Inputs
static const uint8_t PIN_DO[] = {8, 9, 10, 11, 12, 13, 14, 15}; // Outputs
static const uint8_t PIN_AI[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog inputs

// If you prefer input pullups, set to true (HIGH = not pressed, LOW = pressed)
static const bool USE_PULLUPS = false;

// ---------- Display ----------
Arduino_GigaDisplay_GFX gfx;           // 800×480 canvas
Arduino_GigaDisplayTouch touch;        // multi‑touch controller

static const int16_t SCREEN_W = 800;
static const int16_t SCREEN_H = 480;

// Layout
static const int16_t LEFT_W  = 540;              // Left pane width for IO table
static const int16_t RIGHT_W = SCREEN_W - LEFT_W; // Right pane for buttons
static const int16_t MARGIN  = 8;

// Colors (RGB565 via helper)
static inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) { return gfx.color565(r, g, b); }
static const uint16_t COL_BG       = RGB(16,16,20);
static const uint16_t COL_PANEL    = RGB(30,30,36);
static const uint16_t COL_FRAME    = RGB(70,70,80);
static const uint16_t COL_TEXT     = RGB(240,240,240);
static const uint16_t COL_SUBTEXT  = RGB(170,170,180);
static const uint16_t COL_OK       = RGB(30, 180, 90);
static const uint16_t COL_WARN     = RGB(250, 170, 40);
static const uint16_t COL_BAD      = RGB(220, 60, 60);
static const uint16_t COL_OFF      = RGB(70, 70, 70);

// Button palette
static const uint16_t BTN_RED      = RGB(220,60,60);
static const uint16_t BTN_GREEN    = RGB(50,200,90);
static const uint16_t BTN_BLUE     = RGB(70,140,255);
static const uint16_t BTN_ORANGE   = RGB(255,165,0);
static const uint16_t BTN_CYAN     = RGB(0,200,255);
static const uint16_t BTN_MAGENTA  = RGB(255,0,180);

// ---------- Debounce & timing ----------
static uint32_t g_debounceMs = 500; // default 500 ms

void setDebounceMs(uint32_t ms) { g_debounceMs = ms; }

// ---------- Mini scheduler ----------
typedef void (*TaskFn)();
struct Task { TaskFn fn; uint32_t interval; uint32_t last; bool enabled; };
static const uint8_t MAX_TASKS = 8;
static Task g_tasks[MAX_TASKS];
static uint8_t g_taskCount = 0;

void addTask(TaskFn fn, uint32_t intervalMs) {
  if (g_taskCount < MAX_TASKS) {
    g_tasks[g_taskCount++] = {fn, intervalMs, 0, true};
  }
}

void runTasks() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < g_taskCount; ++i) {
    Task &t = g_tasks[i];
    if (!t.enabled) continue;
    if (now - t.last >= t.interval) {
      // Catch-up safe for long delays
      t.last = now;  // or t.last += t.interval if you prefer fixed cadence
      t.fn();
    }
  }
}

// ---------- UI Buttons ----------
struct UIButton {
  int16_t x, y, w, h;
  const char* label;
  uint16_t color;
  bool latching;     // true = latching, false = momentary
  bool state;        // latching: ON/OFF; momentary: pressed/released
  bool pressed;      // internal: finger currently inside rect
  uint32_t lastChangeMs;
  bool drawnState;   // last drawn state for cheap invalidation
};

// 6 buttons in a 2×3 grid
static UIButton g_buttons[6];

// Forward decls
void drawStaticChrome();
void drawLeftTableStatic();
void refreshLeftTable();
void drawButtons(bool full);
void handleTouch();

// ---------- IO State ----------
static uint8_t diState[sizeof(PIN_DI)];
static uint8_t doState[sizeof(PIN_DO)];
static uint16_t aiValue[sizeof(PIN_AI)];

static uint8_t diPrev[sizeof(PIN_DI)];
static uint8_t doPrev[sizeof(PIN_DO)];
static uint16_t aiPrev[sizeof(PIN_AI)];

// ---------- Hooks you can customize ----------
void onLatchChanged(uint8_t index /*0..2*/, bool on) {
  // Example (commented): tie latched buttons to outputs 8..10
  // digitalWrite(PIN_DO[index], on ? HIGH : LOW);
}

void onMomentaryPress(uint8_t index /*0..2*/) {
  // Example: pulse an output when pressed
  // digitalWrite(PIN_DO[3 + index], HIGH);
}

void onMomentaryRelease(uint8_t index /*0..2*/) {
  // Example: release the output
  // digitalWrite(PIN_DO[3 + index], LOW);
}

// ---------- Init helpers ----------
void initPins() {
  for (uint8_t i = 0; i < sizeof(PIN_DI); ++i) {
    pinMode(PIN_DI[i], USE_PULLUPS ? INPUT_PULLUP : INPUT);
  }
  for (uint8_t i = 0; i < sizeof(PIN_DO); ++i) {
    pinMode(PIN_DO[i], OUTPUT);
    digitalWrite(PIN_DO[i], LOW);
  }
  analogReadResolution(16); // 0..65535 (scaled by core)
}

void initButtons() {
  const int cols = 2, rows = 3;
  const int16_t pad = MARGIN;
  const int16_t gridX = LEFT_W + pad;
  const int16_t gridY = pad + 40; // leave space for title
  const int16_t gridW = RIGHT_W - 2*pad;
  const int16_t gridH = SCREEN_H - gridY - pad;
  const int16_t cellW = (gridW - pad) / cols;
  const int16_t cellH = (gridH - pad*2) / rows;

  const char* labels[6] = {"L1","L2","L3","M1","M2","M3"};
  const uint16_t colors[6] = {BTN_RED, BTN_GREEN, BTN_BLUE, BTN_ORANGE, BTN_CYAN, BTN_MAGENTA};

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const int idx = r*cols + c;
      UIButton &b = g_buttons[idx];
      b.w = cellW;
      b.h = cellH;
      b.x = gridX + c*(cellW + pad);
      b.y = gridY + r*(cellH + pad);
      b.label = labels[idx];
      b.color = colors[idx];
      b.latching = (c == 0); // left column latching, right momentary
      b.state = false;
      b.pressed = false;
      b.lastChangeMs = 0;
      b.drawnState = !b.state; // force first draw
    }
  }
}

// ---------- Drawing ----------
void drawRoundedPanel(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t frame, uint16_t fill) {
  gfx.fillRoundRect(x, y, w, h, 8, fill);
  gfx.drawRoundRect(x, y, w, h, 8, frame);
}

void drawStaticChrome() {
  gfx.fillScreen(COL_BG);

  // Left info panel
  drawRoundedPanel(MARGIN, MARGIN, LEFT_W - 2*MARGIN, SCREEN_H - 2*MARGIN, COL_FRAME, COL_PANEL);

  // Right buttons panel
  drawRoundedPanel(LEFT_W + MARGIN, MARGIN, RIGHT_W - 2*MARGIN, SCREEN_H - 2*MARGIN, COL_FRAME, COL_PANEL);

  // Titles
  gfx.setTextColor(COL_TEXT);
  gfx.setTextSize(2);
  gfx.setCursor(MARGIN + 14, MARGIN + 10);
  gfx.print("I/O Monitor");

  gfx.setCursor(LEFT_W + MARGIN + 14, MARGIN + 10);
  gfx.print("Controls");
}

// Cached text cell geometry for the left table
struct Cell { int16_t x, y, w, h; };
static const uint8_t ROW_DI = sizeof(PIN_DI);
static const uint8_t ROW_DO = sizeof(PIN_DO);
static const uint8_t ROW_AI = sizeof(PIN_AI);

static Cell diCells[ROW_DI];
static Cell doCells[ROW_DO];
static Cell aiCells[ROW_AI];

void drawLeftTableStatic() {
  const int16_t x = MARGIN + 12;
  int16_t y = MARGIN + 44;
  const int16_t W = LEFT_W - 2*MARGIN - 24;
  const int16_t rowH = 18;

  gfx.setTextSize(2);
  gfx.setTextColor(COL_SUBTEXT);
  gfx.setCursor(x, y); gfx.print("Digital In (2-7)");
  y += 24;

  gfx.setTextSize(1);
  gfx.setTextColor(COL_TEXT);
  for (uint8_t i = 0; i < ROW_DI; ++i) {
    diCells[i] = {x, y, W, rowH};
    gfx.setCursor(x, y);
    gfx.printf("D%-2u  : --\n", PIN_DI[i]);
    y += rowH;
  }

  y += 10;
  gfx.setTextSize(2);
  gfx.setTextColor(COL_SUBTEXT);
  gfx.setCursor(x, y); gfx.print("Digital Out (8-15)");
  y += 24;

  gfx.setTextSize(1);
  gfx.setTextColor(COL_TEXT);
  for (uint8_t i = 0; i < ROW_DO; ++i) {
    doCells[i] = {x, y, W, rowH};
    gfx.setCursor(x, y);
    gfx.printf("D%-2u  : --\n", PIN_DO[i]);
    y += rowH;
  }

  y += 10;
  gfx.setTextSize(2);
  gfx.setTextColor(COL_SUBTEXT);
  gfx.setCursor(x, y); gfx.print("Analog In (A0-A7)");
  y += 24;

  gfx.setTextSize(1);
  gfx.setTextColor(COL_TEXT);
  for (uint8_t i = 0; i < ROW_AI; ++i) {
    aiCells[i] = {x, y, W, rowH};
    gfx.setCursor(x, y);
    gfx.printf("A%-2u  : -----\n", i);
    y += rowH;
  }
}

void drawValueIntoCell(const Cell& c, const String& label, const String& value, uint16_t color) {
  // Clear the cell’s value area then print fresh text
  // We preserve the left label width by overwriting from a fixed X
  const int labelW = 56; // pixels reserved for "Dx:" / "Ax:" label
  gfx.fillRect(c.x + labelW, c.y, c.w - labelW, c.h, COL_PANEL);

  gfx.setTextSize(1);
  gfx.setTextColor(color);
  gfx.setCursor(c.x, c.y);
  gfx.print(label);
  gfx.print("  : ");
  gfx.print(value);
}

// Redraw changed values only
void refreshLeftTable() {
  // Digital inputs
  for (uint8_t i = 0; i < ROW_DI; ++i) {
    if (diState[i] != diPrev[i]) {
      const bool high = diState[i];
      String label = String("D") + String(PIN_DI[i]);
      drawValueIntoCell(diCells[i], label, high ? "HIGH" : "LOW", high ? COL_OK : COL_BAD);
      diPrev[i] = diState[i];
    }
  }
  // Digital outputs (readback)
  for (uint8_t i = 0; i < ROW_DO; ++i) {
    if (doState[i] != doPrev[i]) {
      const bool high = doState[i];
      String label = String("D") + String(PIN_DO[i]);
      drawValueIntoCell(doCells[i], label, high ? "HIGH" : "LOW", high ? COL_OK : COL_OFF);
      doPrev[i] = doState[i];
    }
  }
  // Analog values
  for (uint8_t i = 0; i < ROW_AI; ++i) {
    if (aiValue[i] != aiPrev[i]) {
      String label = String("A") + String(i);
      // Render both raw and approx volts (3.3V ref assumed)
      char buf[48];
      const float v = (aiValue[i] * 3.3f) / 65535.0f;
      snprintf(buf, sizeof(buf), "%5u  (%.3f V)", aiValue[i], v);
      drawValueIntoCell(aiCells[i], label, String(buf), COL_TEXT);
      aiPrev[i] = aiValue[i];
    }
  }
}

void drawButton(const UIButton& b) {
  const uint16_t base = b.color;
  const bool on = b.latching ? b.state : b.pressed; // pressed visual for momentary
  const uint16_t fill = on ? base : COL_OFF;
  const uint16_t frame = on ? COL_TEXT : COL_FRAME;
  gfx.fillRoundRect(b.x, b.y, b.w, b.h, 12, fill);
  gfx.drawRoundRect(b.x, b.y, b.w, b.h, 12, frame);
  // Label
  gfx.setTextColor(COL_TEXT);
  gfx.setTextSize(2);
  int16_t tx = b.x + (b.w/2) - 12;
  int16_t ty = b.y + (b.h/2) - 8;
  gfx.setCursor(tx, ty);
  gfx.print(b.label);

  // Mode tag
  gfx.setTextSize(1);
  gfx.setCursor(b.x + 8, b.y + b.h - 16);
  gfx.print(b.latching ? "LATCH" : "MOMENT");
}

void drawButtons(bool full) {
  for (uint8_t i = 0; i < 6; ++i) {
    UIButton &b = g_buttons[i];
    const bool visState = b.latching ? b.state : b.pressed;
    if (full || visState != b.drawnState) {
      drawButton(b);
      b.drawnState = visState;
    }
  }
}

// ---------- Sampling ----------
void sampleIO() {
  // DI 2..7
  for (uint8_t i = 0; i < ROW_DI; ++i) {
    diState[i] = (uint8_t)digitalRead(PIN_DI[i]);
  }
  // DO 8..15 (read back actual pin state)
  for (uint8_t i = 0; i < ROW_DO; ++i) {
    doState[i] = (uint8_t)digitalRead(PIN_DO[i]);
  }
  // AI A0..A7
  for (uint8_t i = 0; i < ROW_AI; ++i) {
    aiValue[i] = (uint16_t)analogRead(PIN_AI[i]);
  }
}

// ---------- Touch handling ----------
static inline bool inRect(int16_t x, int16_t y, const UIButton& b) {
  return (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h);
}

void handleTouch() {
  uint8_t contacts = 0;
  GDTpoint_t pts[5];
  contacts = touch.getTouchPoints(pts);

  // Track which buttons are touched this frame (supports multi‑touch)
  bool touched[6] = {false,false,false,false,false,false};
  for (uint8_t k = 0; k < contacts; ++k) {
    const int16_t x = pts[k].x;
    const int16_t y = pts[k].y;
    for (uint8_t i = 0; i < 6; ++i) {
      if (inRect(x, y, g_buttons[i])) touched[i] = true;
    }
  }

  const uint32_t now = millis();

  for (uint8_t i = 0; i < 6; ++i) {
    UIButton &b = g_buttons[i];

    if (b.latching) {
      // Latching: toggle on touch down (edge), debounce guarded
      const bool entering = touched[i] && !b.pressed;
      if (entering && (now - b.lastChangeMs >= g_debounceMs)) {
        b.state = !b.state;
        b.lastChangeMs = now;
        onLatchChanged(i, b.state);
      }
      b.pressed = touched[i];

    } else {
      // Momentary: state mirrors touch with debounce on transitions
      if (touched[i] != b.state && (now - b.lastChangeMs >= g_debounceMs)) {
        b.state = touched[i];
        b.lastChangeMs = now;
        if (b.state) onMomentaryPress(i - 3); else onMomentaryRelease(i - 3);
      }
      b.pressed = touched[i];
    }
  }
}

// ---------- Tasks ----------
void task_refreshUI() {
  refreshLeftTable();
  drawButtons(false); // only redraw buttons whose state changed
}

void task_poll() {
  sampleIO();
  handleTouch();
}

// ---------- Setup & Loop ----------
void setup() {
  initPins();
  gfx.begin();
  gfx.setRotation(1); // landscape (800×480)
  touch.begin();      // start touch controller

  drawStaticChrome();
  drawLeftTableStatic();
  initButtons();
  drawButtons(true);

  // Prime previous values so first refresh draws all dynamic fields
  memcpy(diPrev, diState, sizeof(diPrev));
  memcpy(doPrev, doState, sizeof(doPrev));
  memcpy(aiPrev, aiValue, sizeof(aiPrev));

  // Tasks: poll often, refresh visible UI at 1 Hz (adjust as you like)
  addTask(task_poll, 25);          // 40 Hz sampling of IO & touch
  addTask(task_refreshUI, 1000);   // 1 Hz UI text refresh
}

void loop() {
  runTasks();
}

// ---------- Convenience: adjust debounce at runtime ----------
// Example: call from your own code to change debounce to 200 ms
// setDebounceMs(200);
