/*
  Oscilloscope_Giga.ino
  Touchscreen oscilloscope for Arduino GIGA R1 WiFi + Display Shield

  Input: AMC0330 isolation op-amp output on A0 (centered at Vdd/2 ~1.65V, unity gain)
  Display: 800x480 landscape with waveform plot, grid, info bar, and touch buttons
  ADC: DMA-based high-speed sampling via Arduino_AdvancedAnalog (AdvancedADC)

  Libraries (install via Library Manager):
    - Arduino_AdvancedAnalog
    - Arduino_GigaDisplay_GFX
    - Arduino_GigaDisplayTouch
*/

#include <Arduino.h>
#include <Arduino_AdvancedAnalog.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>

// ======================= Display & Touch =======================
GigaDisplay_GFX gfx;
Arduino_GigaDisplayTouch touchCtrl;

static const int16_t SCREEN_W = 800;
static const int16_t SCREEN_H = 480;

// RGB565 helper
static inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
  return gfx.color565(r, g, b);
}

// ======================= UI Buttons (declared early for Arduino auto-prototype) =======================
struct UIButton {
  int16_t x, y, w, h;
  const char* label;
  uint16_t color;
  bool latching;
  bool state;
  bool pressed;
  uint32_t lastChangeMs;
  bool drawnState;
};

// ======================= App Mode =======================
enum AppMode { MODE_SCOPE, MODE_DMM };
static AppMode appMode = MODE_SCOPE;

// ======================= DMM State =======================
static const uint32_t DMM_SAMPLE_RATE = 1000;  // 10 kHz for oversampling
static const float AMC0330_OFFSET = 1.65692f;     // AMC0330 output at 0V input
static float dmmAvg    = 0;   // offset-corrected DC average
static float dmmMin    = 0;
static float dmmMax    = 0;
static float dmmVrms   = 0;
static float dmmRefV   = 0;
static bool  dmmRefSet = false;
static bool  dmmShowVac = false;  // when true, VAC is the big reading
static bool  dmmCalMode = false; // true while cal voltage picker is shown

// ======================= Colors =======================
static uint16_t COL_BG;
static uint16_t COL_PANEL;
static uint16_t COL_FRAME;
static uint16_t COL_TEXT;
static uint16_t COL_SUBTEXT;
static uint16_t COL_GRID;
static uint16_t COL_TRACE;
static uint16_t COL_TRIGGER;
static uint16_t COL_BTN_TB;
static uint16_t COL_BTN_VD;
static uint16_t COL_BTN_OFS;
static uint16_t COL_BTN_RUN;
static uint16_t COL_BTN_AUTO;
static uint16_t COL_BTN_MODE;
static uint16_t COL_BTN_DMM;
static uint16_t COL_OFF;
static uint16_t COL_STOPPED;

void initColors() {
  COL_BG       = RGB(0, 0, 0);
  COL_PANEL    = RGB(20, 20, 28);
  COL_FRAME    = RGB(60, 60, 70);
  COL_TEXT     = RGB(220, 220, 220);
  COL_SUBTEXT  = RGB(140, 140, 150);
  COL_GRID     = RGB(50, 50, 60);
  COL_TRACE    = RGB(0, 255, 50);
  COL_TRIGGER  = RGB(255, 220, 0);
  COL_BTN_TB   = RGB(70, 140, 255);
  COL_BTN_VD   = RGB(50, 200, 90);
  COL_BTN_OFS  = RGB(255, 165, 0);
  COL_BTN_RUN  = RGB(220, 60, 60);
  COL_BTN_AUTO = RGB(0, 200, 255);
  COL_BTN_MODE = RGB(200, 100, 255);
  COL_BTN_DMM  = RGB(255, 200, 50);
  COL_OFF      = RGB(60, 60, 60);
  COL_STOPPED  = RGB(180, 40, 40);
}

// ======================= Layout =======================
static const int16_t INFO_H    = 36;
static const int16_t PLOT_X    = 10;
static const int16_t PLOT_Y    = INFO_H + 4;
static const int16_t PLOT_W    = 620;
static const int16_t PLOT_H    = 400;
static const int16_t HDIVS    = 10;
static const int16_t VDIVS    = 8;
static const int16_t PX_PER_HDIV = PLOT_W / HDIVS;  // 62
static const int16_t PX_PER_VDIV = PLOT_H / VDIVS;  // 50

static const int16_t BTN_PANEL_X = PLOT_X + PLOT_W + 10;
static const int16_t BTN_PANEL_W = SCREEN_W - BTN_PANEL_X - 6;

// ======================= Probe / Voltage Divider =======================
// Scale factor to convert ADC voltage back to probe-tip voltage.
// Set to 1.0 for direct connection (no divider).
// For a resistive divider: PROBE_SCALE = (R_top + R_bottom) / R_bottom
// Example: 10:1 divider (9k + 1k) → PROBE_SCALE = 10.0
static float PROBE_SCALE = 15.9574f;  // mutable: calibration adjusts this at runtime

// ======================= ADC =======================
AdvancedADC adc(A0);

static const int ADC_BUFSIZE   = 32;
static const int ADC_NBUFFERS  = 32;

// Full-scale voltage for 16-bit ADC reading
static const float VREF = 3.3f;
static const float ADC_MAX = 65535.0f;

// ======================= Timebase =======================
// 1-2-5 sequence in microseconds per division
static const uint32_t TB_OPTIONS[] = {
  50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000
};
static const int TB_COUNT = sizeof(TB_OPTIONS) / sizeof(TB_OPTIONS[0]);
static int tbIndex = 4;  // default 1ms/div

// ======================= V/div =======================
// Volts per division options
static const float VD_OPTIONS[] = {0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 3.3f};
static const int VD_COUNT = sizeof(VD_OPTIONS) / sizeof(VD_OPTIONS[0]);
static int vdIndex = 3;  // default 1.0 V/div

// ======================= Vertical Offset =======================
static int16_t vertOffset = 0;  // pixels, positive = trace moves up

// ======================= Run/Stop =======================
static bool running = true;

// ======================= Headless Mode =======================
// If pin 2 is pulled LOW at boot, skip all display/touch code
static bool headless = false;

// ======================= Trigger =======================
static float triggerLevel = 0.0f;  // V, offset-corrected probe-tip (0 = zero-crossing)

// ======================= Sampling Buffer =======================
// We acquire 2x the plot width to allow trigger search in the first half
static const int ACQUIRE_LEN = PLOT_W * 2;  // 1240 samples
static uint16_t acquireBuf[ACQUIRE_LEN];
static int acquireCount = 0;
static bool frameReady = false;

// Acquisition timing -- measure actual sample rate via micros()
static uint32_t acquireStartUs = 0;
static uint32_t acquireEndUs   = 0;
static float    actualSampleRate = 0;  // computed after each acquisition

// ======================= Trace Arrays =======================
static int16_t oldTraceY[PLOT_W];
static int16_t newTraceY[PLOT_W];
static bool firstTrace = true;

// ======================= Measurements =======================
static float measFreq = 0;
static float measVpp  = 0;
static float measVmin = 0;
static float measVmax = 0;
static float measVrms = 0;

// ======================= Trigger Indicator =======================
static int16_t prevTrigY = -1;  // track old triangle position for erase

// ======================= Mini Scheduler =======================
struct Task { void (*fn)(); uint32_t interval; uint32_t last; };
static const uint8_t MAX_TASKS = 8;
static Task g_tasks[MAX_TASKS];
static uint8_t g_taskCount = 0;

void addTask(void (*fn)(), uint32_t intervalMs) {
  if (g_taskCount < MAX_TASKS) {
    g_tasks[g_taskCount++] = {fn, intervalMs, 0};
  }
}

void runTasks() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < g_taskCount; ++i) {
    Task &t = g_tasks[i];
    if (now - t.last >= t.interval) {
      t.last = now;
      t.fn();
    }
  }
}

static const int MAX_BUTTONS = 9;
static int activeButtonCount = 9;
static UIButton g_buttons[MAX_BUTTONS];

static const uint32_t DEBOUNCE_MS = 300;

// Scope button indices
enum BtnIdx {
  BTN_TB_FAST = 0,
  BTN_TB_SLOW,
  BTN_VD_PLUS,
  BTN_VD_MINUS,
  BTN_OFS_UP,
  BTN_OFS_DN,
  BTN_RUNSTOP,
  BTN_AUTO,
  BTN_MODE_SCOPE  // MODE button in scope mode (index 8)
};

// DMM button indices
enum DmmBtnIdx {
  BTN_DMM_RSTMM = 0,
  BTN_DMM_SETREF,
  BTN_DMM_VAC,
  BTN_DMM_CAL,
  BTN_DMM_MODE  // MODE button in DMM mode
};

// ======================= Forward Declarations =======================
void drawStaticChrome();
void drawGrid();
void drawGridDot(int16_t x, int16_t y);
void drawInfoBar();
void drawButtons(bool full);
void drawButton(const UIButton& b);
void handleTouch();
void acquireSamples();
void processAndRender();
uint32_t calcSampleRate();
void restartADC();
void autoRange();
void drawTriggerIndicator();
void initScopeButtons();
void initDmmButtons();
void switchToScope();
void switchToDmm();
void drawDmmChrome();
void refreshDmmReading();
void dmmAcquireAndCompute();
void drawCalScreen();
void handleCalTouch(int16_t tx, int16_t ty);
void sendScopeFrame(int startIdx);
void sendDmmReading();
void sendConfig();
void sendMode(const char* mode);
void processSerialCommand();
void handleSerialCmd(const char* cmd);
void task_serialDmm();

// ======================= Helpers =======================
void drawRoundedPanel(int16_t x, int16_t y, int16_t w, int16_t h,
                      uint16_t frame, uint16_t fill) {
  gfx.fillRoundRect(x, y, w, h, 8, fill);
  gfx.drawRoundRect(x, y, w, h, 8, frame);
}

#define inRect(tx, ty, b) \
  ((tx) >= (b).x && (tx) < (b).x + (b).w && (ty) >= (b).y && (ty) < (b).y + (b).h)

static inline float adcToVolts(uint16_t raw) {
  float rawV = (raw / ADC_MAX) * VREF;
  return (rawV - AMC0330_OFFSET) * PROBE_SCALE;
}

// DMM conversion: remove AMC0330 DC offset at the ADC pin, then scale by probe ratio
static inline float dmmToVolts(uint16_t raw) {
  float rawV = (raw / ADC_MAX) * VREF;       // voltage at ADC pin
  return (rawV - AMC0330_OFFSET) * PROBE_SCALE;  // offset-correct, then scale
}

// Format a timebase value for display
void formatTimebase(char* buf, size_t len, uint32_t usPerDiv) {
  if (usPerDiv < 1000) {
    snprintf(buf, len, "%luus", (unsigned long)usPerDiv);
  } else if (usPerDiv < 1000000) {
    snprintf(buf, len, "%lums", (unsigned long)(usPerDiv / 1000));
  } else {
    snprintf(buf, len, "%lus", (unsigned long)(usPerDiv / 1000000));
  }
}

// ======================= ADC Rate Calculation =======================
uint32_t calcSampleRate() {
  // Samples needed across full plot width = PLOT_W
  // Time across full plot = HDIVS * usPerDiv (microseconds)
  // Sample rate = PLOT_W / (HDIVS * usPerDiv * 1e-6) = PLOT_W * 1e6 / (HDIVS * usPerDiv)
  uint32_t usPerDiv = TB_OPTIONS[tbIndex];
  uint32_t totalUs = (uint32_t)HDIVS * usPerDiv;
  // samples per second
  uint32_t rate = (uint32_t)((uint64_t)PLOT_W * 1000000ULL / totalUs);

  // Clamp to AdvancedADC limits
  if (rate < 1000)    rate = 1000;
  if (rate > 1000000) rate = 1000000;
  return rate;
}

void restartADC() {
  adc.stop();
  delay(5);
  acquireCount = 0;
  frameReady = false;
  if (!adc.begin(AN_RESOLUTION_16, calcSampleRate(), ADC_BUFSIZE, ADC_NBUFFERS)) {
    if (!headless) {
      gfx.setTextColor(COL_STOPPED);
      gfx.setTextSize(2);
      gfx.setCursor(PLOT_X + 20, PLOT_Y + PLOT_H / 2);
      gfx.print("ADC INIT FAILED");
    }
  }
}

// ======================= Init Buttons =======================
void initScopeButtons() {
  activeButtonCount = 9;
  const int16_t btnH = 42;
  const int16_t gap  = 4;
  int16_t y = PLOT_Y;

  struct BtnDef {
    const char* label;
    uint16_t color;
    bool latching;
  };

  BtnDef defs[9] = {
    {"T<< Fast", COL_BTN_TB,   false},
    {"T>> Slow", COL_BTN_TB,   false},
    {"V/d +",    COL_BTN_VD,   false},
    {"V/d -",    COL_BTN_VD,   false},
    {"Ofs UP",   COL_BTN_OFS,  false},
    {"Ofs DN",   COL_BTN_OFS,  false},
    {"RUN/STOP", COL_BTN_RUN,  true},
    {"AUTO",     COL_BTN_AUTO, false},
    {"MODE",     COL_BTN_MODE, false},
  };

  for (int i = 0; i < 9; ++i) {
    UIButton &b = g_buttons[i];
    b.x = BTN_PANEL_X;
    b.y = y;
    b.w = BTN_PANEL_W;
    b.h = btnH;
    b.label = defs[i].label;
    b.color = defs[i].color;
    b.latching = defs[i].latching;
    b.state = (i == BTN_RUNSTOP) ? true : false;  // RUN/STOP starts ON (running)
    b.pressed = false;
    b.lastChangeMs = 0;
    b.drawnState = !b.state;  // force first draw
    y += btnH + gap;
  }
}

void initDmmButtons() {
  activeButtonCount = 5;
  const int16_t btnH = 42;
  const int16_t gap  = 6;
  // Center 5 buttons vertically in the button panel area
  int16_t totalH = 5 * btnH + 4 * gap;
  int16_t y = PLOT_Y + (PLOT_H - totalH) / 2;

  struct BtnDef {
    const char* label;
    uint16_t color;
    bool latching;
  };

  BtnDef defs[5] = {
    {"RST M/M",  COL_BTN_DMM,  false},
    {"SET REF",  COL_BTN_DMM,  false},
    {"VAC",      COL_BTN_AUTO, false},
    {"CAL",      COL_BTN_OFS,  false},
    {"MODE",     COL_BTN_MODE, false},
  };

  for (int i = 0; i < 5; ++i) {
    UIButton &b = g_buttons[i];
    b.x = BTN_PANEL_X;
    b.y = y;
    b.w = BTN_PANEL_W;
    b.h = btnH;
    b.label = defs[i].label;
    b.color = defs[i].color;
    b.latching = defs[i].latching;
    b.state = false;
    b.pressed = false;
    b.lastChangeMs = 0;
    b.drawnState = !b.state;  // force first draw
    y += btnH + gap;
  }
}

// ======================= Drawing =======================
void drawStaticChrome() {
  gfx.fillScreen(COL_BG);

  // Info bar background
  gfx.fillRect(0, 0, SCREEN_W, INFO_H, COL_PANEL);
  gfx.drawFastHLine(0, INFO_H - 1, SCREEN_W, COL_FRAME);

  // Plot area border
  gfx.drawRect(PLOT_X - 1, PLOT_Y - 1, PLOT_W + 2, PLOT_H + 2, COL_FRAME);
}

void drawGrid() {
  // Draw dotted grid lines inside the plot area
  for (int gx = 0; gx <= HDIVS; ++gx) {
    int16_t x = PLOT_X + gx * PX_PER_HDIV;
    for (int16_t y = PLOT_Y; y < PLOT_Y + PLOT_H; y += 4) {
      gfx.drawPixel(x, y, COL_GRID);
    }
  }
  for (int gy = 0; gy <= VDIVS; ++gy) {
    int16_t y = PLOT_Y + gy * PX_PER_VDIV;
    for (int16_t x = PLOT_X; x < PLOT_X + PLOT_W; x += 4) {
      gfx.drawPixel(x, y, COL_GRID);
    }
  }
}

// Redraw a single grid dot at pixel coordinates (used after erasing trace)
void drawGridDot(int16_t px, int16_t py) {
  // Check if this pixel lies on a grid line intersection dot
  int16_t relX = px - PLOT_X;
  int16_t relY = py - PLOT_Y;

  // Horizontal grid lines (dotted every 4px)
  if (relY >= 0 && relY <= PLOT_H && (relY % PX_PER_VDIV) == 0 && (relX % 4) == 0) {
    gfx.drawPixel(px, py, COL_GRID);
  }
  // Vertical grid lines (dotted every 4px)
  if (relX >= 0 && relX <= PLOT_W && (relX % PX_PER_HDIV) == 0 && (relY % 4) == 0) {
    gfx.drawPixel(px, py, COL_GRID);
  }
}

void drawTriggerIndicator() {
  // Erase old triangle if it moved
  if (prevTrigY >= 0) {
    gfx.fillTriangle(PLOT_X - 8, prevTrigY - 5,
                      PLOT_X - 8, prevTrigY + 5,
                      PLOT_X - 1, prevTrigY, COL_BG);
  }

  // Compute new trigger Y position
  float vPerDiv = VD_OPTIONS[vdIndex];
  float centerV = 0;  // 0V at center (offset-corrected)
  float deltaV = triggerLevel - centerV;
  int16_t trigY = PLOT_Y + (PLOT_H / 2) - (int16_t)(deltaV / vPerDiv * PX_PER_VDIV) - vertOffset;

  if (trigY < PLOT_Y) trigY = PLOT_Y;
  if (trigY > PLOT_Y + PLOT_H - 1) trigY = PLOT_Y + PLOT_H - 1;

  // Draw new triangle
  gfx.fillTriangle(PLOT_X - 8, trigY - 5,
                    PLOT_X - 8, trigY + 5,
                    PLOT_X - 1, trigY, COL_TRIGGER);
  prevTrigY = trigY;
}

void drawInfoBar() {
  // Clear info bar area
  gfx.fillRect(0, 0, SCREEN_W, INFO_H - 1, COL_PANEL);

  gfx.setTextSize(2);
  gfx.setTextColor(COL_TEXT);

  char tbStr[16];
  formatTimebase(tbStr, sizeof(tbStr), TB_OPTIONS[tbIndex]);

  char line[140];
  snprintf(line, sizeof(line), "TB:%s/d V/d:%.1fV F:%.0fHz Vpp:%.2fV Vrms:%.2fV",
           tbStr, VD_OPTIONS[vdIndex], measFreq, measVpp, measVrms);

  gfx.setCursor(8, 10);
  gfx.print(line);

  // Show RUN/STOP status
  if (!running) {
    gfx.setTextColor(COL_STOPPED);
    gfx.setCursor(SCREEN_W - 90, 10);
    gfx.print("STOP");
  }
}

void drawButton(const UIButton& b) {
  const bool on = b.latching ? b.state : b.pressed;
  const uint16_t fill = on ? b.color : COL_OFF;
  const uint16_t frame = on ? COL_TEXT : COL_FRAME;

  gfx.fillRoundRect(b.x, b.y, b.w, b.h, 8, fill);
  gfx.drawRoundRect(b.x, b.y, b.w, b.h, 8, frame);

  // Center label
  gfx.setTextColor(COL_TEXT);
  gfx.setTextSize(2);

  // Approximate text width: ~12px per char at size 2
  int16_t textW = strlen(b.label) * 12;
  int16_t tx = b.x + (b.w - textW) / 2;
  int16_t ty = b.y + (b.h - 16) / 2;
  gfx.setCursor(tx, ty);
  gfx.print(b.label);
}

void drawButtons(bool full) {
  for (int i = 0; i < activeButtonCount; ++i) {
    UIButton &b = g_buttons[i];
    const bool visState = b.latching ? b.state : b.pressed;
    if (full || visState != b.drawnState) {
      drawButton(b);
      b.drawnState = visState;
    }
  }
}

// ======================= Touch Handling =======================
void handleTouch() {
  uint8_t contacts = 0;
  GDTpoint_t pts[5];
  contacts = touchCtrl.getTouchPoints(pts);

  // Intercept touches for calibration screen
  if (dmmCalMode) {
    static uint32_t lastCalTouchMs = 0;
    static bool calTouchDown = false;
    if (contacts > 0 && !calTouchDown) {
      uint32_t now = millis();
      if (now - lastCalTouchMs >= DEBOUNCE_MS) {
        int16_t tx = pts[0].y;
        int16_t ty = (SCREEN_H - 1) - pts[0].x;
        handleCalTouch(tx, ty);
        lastCalTouchMs = now;
      }
    }
    calTouchDown = (contacts > 0);
    return;  // don't process regular buttons while cal is up
  }

  bool touched[MAX_BUTTONS] = {};
  for (uint8_t k = 0; k < contacts; ++k) {
    // Touch controller returns native portrait coords (480x800).
    // Remap for GFX rotation(1) landscape (800x480):
    int16_t tx = pts[k].y;
    int16_t ty = (SCREEN_H - 1) - pts[k].x;
    for (int i = 0; i < activeButtonCount; ++i) {
      if (inRect(tx, ty, g_buttons[i])) touched[i] = true;
    }
  }

  const uint32_t now = millis();

  for (int i = 0; i < activeButtonCount; ++i) {
    UIButton &b = g_buttons[i];

    if (b.latching) {
      // Toggle on touch-down edge with debounce
      bool entering = touched[i] && !b.pressed;
      if (entering && (now - b.lastChangeMs >= DEBOUNCE_MS)) {
        b.state = !b.state;
        b.lastChangeMs = now;
        onButtonAction(i);
      }
      b.pressed = touched[i];
    } else {
      // Momentary: fire on touch-down edge
      bool entering = touched[i] && !b.pressed;
      if (entering && (now - b.lastChangeMs >= DEBOUNCE_MS)) {
        b.lastChangeMs = now;
        onButtonAction(i);
      }
      b.pressed = touched[i];
    }
  }
}

// ======================= Button Actions =======================
void onButtonAction(int idx) {
  if (appMode == MODE_SCOPE) {
    switch (idx) {
      case BTN_TB_FAST:
        if (tbIndex > 0) {
          tbIndex--;
          restartADC();
          if (!headless) drawInfoBar();
        }
        break;
      case BTN_TB_SLOW:
        if (tbIndex < TB_COUNT - 1) {
          tbIndex++;
          restartADC();
          if (!headless) drawInfoBar();
        }
        break;
      case BTN_VD_PLUS:
        if (vdIndex < VD_COUNT - 1) {
          vdIndex++;
          if (!headless) drawInfoBar();
        }
        break;
      case BTN_VD_MINUS:
        if (vdIndex > 0) {
          vdIndex--;
          if (!headless) drawInfoBar();
        }
        break;
      case BTN_OFS_UP:
        vertOffset += 10;
        break;
      case BTN_OFS_DN:
        vertOffset -= 10;
        break;
      case BTN_RUNSTOP:
        running = g_buttons[BTN_RUNSTOP].state;
        if (running) {
          acquireCount = 0;
          frameReady = false;
          firstTrace = true;
        }
        if (!headless) drawInfoBar();
        break;
      case BTN_AUTO:
        autoRange();
        if (!headless) drawInfoBar();
        break;
      case BTN_MODE_SCOPE:
        switchToDmm();
        break;
    }
  } else {
    // DMM mode buttons
    switch (idx) {
      case BTN_DMM_RSTMM:
        dmmMin = dmmAvg;
        dmmMax = dmmAvg;
        if (!headless) {
          refreshDmmReading();
        } else {
          sendDmmReading();
        }
        break;
      case BTN_DMM_SETREF:
        if (dmmRefSet) {
          dmmRefSet = false;
          dmmRefV = 0;
        } else {
          dmmRefSet = true;
          dmmRefV = dmmAvg;
        }
        if (!headless) {
          drawDmmChrome();
          drawButtons(true);
          refreshDmmReading();
        } else {
          sendDmmReading();
        }
        break;
      case BTN_DMM_VAC:
        dmmShowVac = !dmmShowVac;
        if (!headless) {
          drawDmmChrome();
          drawButtons(true);
          refreshDmmReading();
        } else {
          sendDmmReading();
        }
        break;
      case BTN_DMM_CAL:
        if (!headless) {
          dmmCalMode = true;
          drawCalScreen();
        }
        break;
      case BTN_DMM_MODE:
        switchToScope();
        break;
    }
  }
}

// ======================= Auto Range =======================
void autoRange() {
  // Find the best V/div to fit the signal currently in the acquire buffer
  if (acquireCount < PLOT_W) return;

  float vMin = 9999, vMax = -9999;
  for (int i = 0; i < acquireCount; ++i) {
    float v = adcToVolts(acquireBuf[i]);
    if (v < vMin) vMin = v;
    if (v > vMax) vMax = v;
  }

  float vpp = vMax - vMin;
  if (vpp < 0.01f) vpp = 0.01f;

  // Pick the smallest V/div that fits the signal in ~6 divisions (leaving margin)
  for (int i = 0; i < VD_COUNT; ++i) {
    if (VD_OPTIONS[i] * 6.0f >= vpp) {
      vdIndex = i;
      return;
    }
  }
  vdIndex = VD_COUNT - 1;
}

// ======================= Acquisition =======================
void acquireSamples() {
  if (!running || frameReady) return;

  while (adc.available()) {
    SampleBuffer buf = adc.read();

    if (acquireCount == 0) {
      acquireStartUs = micros();  // timestamp first buffer
    }

    for (size_t i = 0; i < buf.size(); ++i) {
      if (acquireCount < ACQUIRE_LEN) {
        acquireBuf[acquireCount++] = buf[i];
      }
    }

    buf.release();

    if (acquireCount >= ACQUIRE_LEN) {
      acquireEndUs = micros();  // timestamp last buffer
      uint32_t elapsedUs = acquireEndUs - acquireStartUs;
      if (elapsedUs > 0) {
        actualSampleRate = (float)acquireCount / ((float)elapsedUs * 1e-6f);
      }
      frameReady = true;
      break;
    }
  }
}

// ======================= Trigger Search =======================
// Find rising-edge trigger point in the first half of the acquire buffer
int findTrigger() {
  // Convert offset-corrected probe-tip voltage back to raw ADC value
  float trigRawV = (triggerLevel / PROBE_SCALE) + AMC0330_OFFSET;
  float trigRawF = (trigRawV / VREF) * ADC_MAX;
  if (trigRawF < 0) trigRawF = 0;
  if (trigRawF > ADC_MAX) trigRawF = ADC_MAX;
  uint16_t trigRaw = (uint16_t)trigRawF;
  int searchEnd = ACQUIRE_LEN - PLOT_W;  // ensure PLOT_W samples after trigger

  for (int i = 1; i < searchEnd; ++i) {
    if (acquireBuf[i - 1] < trigRaw && acquireBuf[i] >= trigRaw) {
      return i;
    }
  }
  return -1;  // no trigger found
}

// ======================= Measurements =======================
void computeMeasurements(int startIdx) {
  // --- Vmin / Vmax / Vpp (from displayed window) ---
  float vMin = 9999, vMax = -9999;
  float sum = 0;
  for (int i = 0; i < PLOT_W; ++i) {
    float v = adcToVolts(acquireBuf[startIdx + i]);
    if (v < vMin) vMin = v;
    if (v > vMax) vMax = v;
    sum += v;
  }
  measVmin = vMin;
  measVmax = vMax;
  measVpp  = vMax - vMin;

  // --- AC-coupled VRMS (subtract DC mean first) ---
  float dcMean = sum / PLOT_W;
  float sumAcSq = 0;
  for (int i = 0; i < PLOT_W; ++i) {
    float ac = adcToVolts(acquireBuf[startIdx + i]) - dcMean;
    sumAcSq += ac * ac;
  }
  measVrms = sqrtf(sumAcSq / PLOT_W);

  // --- Frequency (use full acquire buffer for more periods) ---
  // Convert DC mean (offset-corrected probe-tip V) back to raw ADC value
  float midRawV = (dcMean / PROBE_SCALE) + AMC0330_OFFSET;
  float midRawF = (midRawV / VREF) * ADC_MAX;
  if (midRawF < 0) midRawF = 0;
  if (midRawF > ADC_MAX) midRawF = ADC_MAX;
  uint16_t midRaw = (uint16_t)midRawF;
  float lastCrossF = -1;
  float totalPeriod = 0;
  int   periodCount = 0;

  for (int i = 1; i < acquireCount; ++i) {
    uint16_t prev = acquireBuf[i - 1];
    uint16_t cur  = acquireBuf[i];
    if (prev < midRaw && cur >= midRaw) {
      // Linear interpolation for sub-sample crossing accuracy
      float frac = (cur == prev) ? 0.0f
                   : (float)(midRaw - prev) / (float)(cur - prev);
      float crossF = (float)(i - 1) + frac;

      if (lastCrossF >= 0) {
        totalPeriod += (crossF - lastCrossF);
        periodCount++;
      }
      lastCrossF = crossF;
    }
  }

  if (periodCount > 0 && actualSampleRate > 0) {
    float avgPeriodSamples = totalPeriod / periodCount;
    measFreq = actualSampleRate / avgPeriodSamples;
  } else {
    measFreq = 0;
  }
}

// ======================= Waveform Rendering =======================
void processAndRender() {
  if (!frameReady) return;

  // Find trigger
  int trigIdx = findTrigger();
  int startIdx = (trigIdx >= 0) ? trigIdx : 0;  // free-run fallback

  // Compute measurements
  computeMeasurements(startIdx);

  if (!headless) {
    // Convert samples to pixel Y positions
    float vPerDiv = VD_OPTIONS[vdIndex];
    float centerV = 0;  // 0V at center of display (offset-corrected)

    for (int i = 0; i < PLOT_W; ++i) {
      float v = adcToVolts(acquireBuf[startIdx + i]);
      float deltaV = v - centerV;
      int16_t py = (PLOT_Y + PLOT_H / 2) - (int16_t)(deltaV / vPerDiv * PX_PER_VDIV) - vertOffset;

      if (py < PLOT_Y) py = PLOT_Y;
      if (py > PLOT_Y + PLOT_H - 1) py = PLOT_Y + PLOT_H - 1;

      newTraceY[i] = py;
    }

    // Erase old trace, draw new trace
    for (int i = 0; i < PLOT_W; ++i) {
      int16_t px = PLOT_X + i;

      if (!firstTrace) {
        int16_t eraseMin = oldTraceY[i];
        int16_t eraseMax = oldTraceY[i];
        if (i > 0) {
          if (oldTraceY[i - 1] < eraseMin) eraseMin = oldTraceY[i - 1];
          if (oldTraceY[i - 1] > eraseMax) eraseMax = oldTraceY[i - 1];
        }
        if (i < PLOT_W - 1) {
          if (oldTraceY[i + 1] < eraseMin) eraseMin = oldTraceY[i + 1];
          if (oldTraceY[i + 1] > eraseMax) eraseMax = oldTraceY[i + 1];
        }

        int16_t spanH = eraseMax - eraseMin + 1;
        gfx.drawFastVLine(px, eraseMin, spanH, COL_BG);

        for (int16_t gy = eraseMin; gy <= eraseMax; ++gy) {
          drawGridDot(px, gy);
        }
      }

      if (i < PLOT_W - 1) {
        gfx.drawLine(px, newTraceY[i], px + 1, newTraceY[i + 1], COL_TRACE);
      }

      oldTraceY[i] = newTraceY[i];
    }

    firstTrace = false;

    // Redraw trigger indicator
    drawTriggerIndicator();
  }

  // Stream scope frame to PC over serial
  sendScopeFrame(startIdx);

  // Reset for next acquisition
  acquireCount = 0;
  frameReady = false;
}

// ======================= DMM Display =======================
// DMM main display area dimensions (left of button panel)
static const int16_t DMM_MAIN_W = BTN_PANEL_X - 10;

void drawDmmChrome() {
  gfx.fillScreen(COL_BG);

  // Title bar
  gfx.fillRect(0, 0, SCREEN_W, INFO_H, COL_PANEL);
  gfx.drawFastHLine(0, INFO_H - 1, SCREEN_W, COL_FRAME);
  gfx.setTextSize(2);
  gfx.setTextColor(COL_BTN_DMM);
  gfx.setCursor(8, 10);
  // Title reflects which reading is primary
  if (dmmShowVac) {
    gfx.print("DMM - AC Voltage (RMS)");
  } else {
    gfx.print("DMM - DC Voltage");
  }

  // Divider line between main area and button panel
  gfx.drawFastVLine(DMM_MAIN_W, INFO_H, SCREEN_H - INFO_H, COL_FRAME);

  // Stats area divider (horizontal line above stats)
  int16_t statsY = SCREEN_H - 70;
  gfx.drawFastHLine(0, statsY, DMM_MAIN_W, COL_FRAME);

  // Static labels in stats area
  gfx.setTextSize(2);
  gfx.setTextColor(COL_SUBTEXT);
  gfx.setCursor(10, statsY + 8);
  gfx.print("Min:");
  gfx.setCursor(220, statsY + 8);
  gfx.print("Max:");
  // Secondary reading label (the one NOT shown big)
  gfx.setCursor(430, statsY + 8);
  if (dmmShowVac) {
    gfx.print("VDC:");
  } else {
    gfx.print("VRMS:");
  }

  gfx.setCursor(10, statsY + 36);
  if (dmmRefSet) {
    gfx.print("Actual:");
  } else {
    gfx.print("Delta:");
  }
}

void refreshDmmReading() {
  if (appMode != MODE_DMM) return;
  if (dmmCalMode) return;  // don't overwrite cal screen

  if (!headless) {
    int16_t statsY = SCREEN_H - 70;
    int16_t mainAreaH = statsY - INFO_H;

    // Clear the main text area
    gfx.fillRect(10, INFO_H + 10, DMM_MAIN_W - 20, mainAreaH - 20, COL_BG);

    // Determine what the big reading is
    float bigValue;
    uint16_t bigColor;

    if (dmmRefSet) {
      bigValue = dmmAvg - dmmRefV;
      bigColor = COL_TRIGGER;
    } else if (dmmShowVac) {
      bigValue = dmmVrms;
      bigColor = COL_BTN_AUTO;
    } else {
      bigValue = dmmAvg;
      bigColor = COL_TRACE;
    }

    // --- Large reading ---
    char bigBuf[20];
    snprintf(bigBuf, sizeof(bigBuf), "%+8.4f V", bigValue);

    int16_t bigTextY;
    if (dmmRefSet) {
      bigTextY = INFO_H + (mainAreaH / 2) - 80;
    } else {
      bigTextY = INFO_H + (mainAreaH / 2) - 48;
    }

    gfx.setTextColor(bigColor);
    gfx.setTextSize(6);
    int16_t textW = strlen(bigBuf) * 36;
    int16_t textX = (DMM_MAIN_W - textW) / 2;
    if (textX < 10) textX = 10;
    gfx.setCursor(textX, bigTextY);
    gfx.print(bigBuf);

    // --- Sub-label above big reading ---
    gfx.setTextSize(2);
    gfx.setTextColor(COL_SUBTEXT);
    int16_t labelY = bigTextY - 22;
    if (labelY < INFO_H + 4) labelY = INFO_H + 4;
    if (dmmRefSet) {
      gfx.setCursor(textX, labelY);
      gfx.print("DELTA");
    }

    // --- When REF active, show actual voltage below the big delta ---
    if (dmmRefSet) {
      char actBuf[20];
      snprintf(actBuf, sizeof(actBuf), "%+8.4f V", dmmAvg);
      gfx.setTextSize(3);
      gfx.setTextColor(COL_TRACE);
      int16_t actW = strlen(actBuf) * 18;
      int16_t actX = (DMM_MAIN_W - actW) / 2;
      if (actX < 10) actX = 10;
      int16_t actY = bigTextY + 100;
      gfx.setCursor(actX, actY);
      gfx.print(actBuf);
    }

    // --- Stats line 1: Min / Max / secondary ---
    gfx.fillRect(70, statsY + 6, 140, 20, COL_BG);
    gfx.fillRect(280, statsY + 6, 140, 20, COL_BG);
    gfx.fillRect(500, statsY + 6, 130, 20, COL_BG);

    gfx.setTextSize(2);
    gfx.setTextColor(COL_TEXT);

    char valBuf[16];

    snprintf(valBuf, sizeof(valBuf), "%.4f V", dmmMin);
    gfx.setCursor(70, statsY + 8);
    gfx.print(valBuf);

    snprintf(valBuf, sizeof(valBuf), "%.4f V", dmmMax);
    gfx.setCursor(280, statsY + 8);
    gfx.print(valBuf);

    if (dmmShowVac) {
      snprintf(valBuf, sizeof(valBuf), "%.4f V", dmmAvg);
    } else {
      snprintf(valBuf, sizeof(valBuf), "%.4f V", dmmVrms);
    }
    gfx.setCursor(500, statsY + 8);
    gfx.print(valBuf);

    // --- Stats line 2: Delta / Actual / REF status ---
    gfx.fillRect(100, statsY + 34, 240, 20, COL_BG);

    gfx.setTextSize(2);
    gfx.setCursor(100, statsY + 36);
    if (dmmRefSet) {
      char actSmall[20];
      snprintf(actSmall, sizeof(actSmall), "%+.4f V", dmmAvg);
      gfx.setTextColor(COL_TEXT);
      gfx.print(actSmall);
    } else {
      gfx.setTextColor(COL_SUBTEXT);
      gfx.print("REF: ---");
    }
  }

  // Stream DMM reading to PC over serial
  sendDmmReading();
}

// ======================= DMM Acquisition =======================
void dmmAcquireAndCompute() {
  while (adc.available()) {
    SampleBuffer buf = adc.read();

    float batchSum = 0;
    float batchMin = 9999;
    float batchMax = -9999;

    for (size_t i = 0; i < buf.size(); ++i) {
      float v = dmmToVolts(buf[i]);
      batchSum += v;
      if (v < batchMin) batchMin = v;
      if (v > batchMax) batchMax = v;
    }

    float batchMean = batchSum / buf.size();

    // AC-coupled RMS for this batch (ripple around DC mean)
    float sumAcSq = 0;
    for (size_t i = 0; i < buf.size(); ++i) {
      float ac = dmmToVolts(buf[i]) - batchMean;
      sumAcSq += ac * ac;
    }
    float batchRms = sqrtf(sumAcSq / buf.size());

    buf.release();

    // Exponential moving average (alpha ~0.1 for smoothing)
    const float alpha = 0.1f;
    dmmAvg  = dmmAvg * (1.0f - alpha) + batchMean * alpha;
    dmmVrms = dmmVrms * (1.0f - alpha) + batchRms * alpha;

    // Track running min/max
    if (batchMin < dmmMin) dmmMin = batchMin;
    if (batchMax > dmmMax) dmmMax = batchMax;
  }
}

// ======================= Calibration =======================
static const float CAL_VOLTAGES[] = {1.0f, 5.0f, 10.0f, 20.0f};
static const int   CAL_COUNT = 4;

// Layout constants for cal touch targets (shared by draw and touch)
static const int16_t CAL_GRID_X = 20;
static const int16_t CAL_GRID_Y = INFO_H + 80;
static const int16_t CAL_GAP    = 12;

static int16_t calCellW() { return (DMM_MAIN_W - 2 * CAL_GRID_X - CAL_GAP) / 2; }
static const int16_t CAL_CELL_H = 90;

void drawCalScreen() {
  // Clear only the main area (leave button panel)
  gfx.fillRect(0, INFO_H, DMM_MAIN_W, SCREEN_H - INFO_H, COL_BG);

  // Title bar update
  gfx.fillRect(0, 0, DMM_MAIN_W, INFO_H, COL_PANEL);
  gfx.drawFastHLine(0, INFO_H - 1, SCREEN_W, COL_FRAME);
  gfx.setTextSize(2);
  gfx.setTextColor(COL_BTN_OFS);
  gfx.setCursor(8, 10);
  gfx.print("CALIBRATE");

  // Show current reading so user can see what they're calibrating
  gfx.setTextSize(2);
  gfx.setTextColor(COL_SUBTEXT);
  gfx.setCursor(20, INFO_H + 15);
  gfx.print("Apply known voltage, then tap:");

  char curBuf[30];
  snprintf(curBuf, sizeof(curBuf), "Current: %+.4f V", dmmAvg);
  gfx.setTextColor(COL_TRACE);
  gfx.setCursor(20, INFO_H + 42);
  gfx.print(curBuf);

  int16_t cw = calCellW();

  // Draw 2x2 grid of voltage options
  for (int i = 0; i < CAL_COUNT; i++) {
    int col = i % 2;
    int row = i / 2;
    int16_t cx = CAL_GRID_X + col * (cw + CAL_GAP);
    int16_t cy = CAL_GRID_Y + row * (CAL_CELL_H + CAL_GAP);

    gfx.fillRoundRect(cx, cy, cw, CAL_CELL_H, 8, COL_BTN_VD);
    gfx.drawRoundRect(cx, cy, cw, CAL_CELL_H, 8, COL_TEXT);

    char label[10];
    snprintf(label, sizeof(label), "%dV", (int)CAL_VOLTAGES[i]);
    gfx.setTextSize(4);
    gfx.setTextColor(COL_TEXT);
    int16_t tw = strlen(label) * 24;  // size 4 ≈ 24px/char
    gfx.setCursor(cx + (cw - tw) / 2, cy + (CAL_CELL_H - 32) / 2);
    gfx.print(label);
  }

  // Cancel button below the grid
  int16_t cancelW = 2 * cw + CAL_GAP;
  int16_t cancelY = CAL_GRID_Y + 2 * (CAL_CELL_H + CAL_GAP) + 4;
  gfx.fillRoundRect(CAL_GRID_X, cancelY, cancelW, 56, 8, COL_BTN_RUN);
  gfx.drawRoundRect(CAL_GRID_X, cancelY, cancelW, 56, 8, COL_TEXT);
  gfx.setTextSize(3);
  gfx.setTextColor(COL_TEXT);
  int16_t tw = 6 * 18;  // "CANCEL" = 6 chars at size 3
  gfx.setCursor(CAL_GRID_X + (cancelW - tw) / 2, cancelY + (56 - 24) / 2);
  gfx.print("CANCEL");
}

void handleCalTouch(int16_t tx, int16_t ty) {
  int16_t cw = calCellW();

  // Check voltage option rectangles
  for (int i = 0; i < CAL_COUNT; i++) {
    int col = i % 2;
    int row = i / 2;
    int16_t cx = CAL_GRID_X + col * (cw + CAL_GAP);
    int16_t cy = CAL_GRID_Y + row * (CAL_CELL_H + CAL_GAP);

    if (tx >= cx && tx < cx + cw && ty >= cy && ty < cy + CAL_CELL_H) {
      // Apply calibration: adjust PROBE_SCALE so current reading becomes target
      float target = CAL_VOLTAGES[i];
      if (fabsf(dmmAvg) > 0.01f) {
        PROBE_SCALE = PROBE_SCALE * (target / dmmAvg);
      }
      // Reset min/max since scale just changed
      dmmMin = dmmAvg;  // will be recalculated from new scale next batch
      dmmMax = dmmAvg;
      dmmCalMode = false;
      drawDmmChrome();
      drawButtons(true);
      refreshDmmReading();
      return;
    }
  }

  // Check cancel button
  int16_t cancelW = 2 * cw + CAL_GAP;
  int16_t cancelY = CAL_GRID_Y + 2 * (CAL_CELL_H + CAL_GAP) + 4;
  if (tx >= CAL_GRID_X && tx < CAL_GRID_X + cancelW &&
      ty >= cancelY && ty < cancelY + 56) {
    dmmCalMode = false;
    drawDmmChrome();
    drawButtons(true);
    refreshDmmReading();
  }
}

// ======================= Serial Streaming =======================
void sendConfig() {
  Serial.print("$CFG,");
  Serial.print(PROBE_SCALE, 4);
  Serial.print(",");
  Serial.print(AMC0330_OFFSET, 5);
  Serial.print(",");
  Serial.print(VREF, 2);
  Serial.print(",");
  Serial.print((int)ADC_MAX);
  Serial.print(",");
  Serial.println(PLOT_W);
}

void sendMode(const char* mode) {
  Serial.print("$MODE,");
  Serial.println(mode);
}

void sendScopeFrame(int startIdx) {
  Serial.print("$SCOPE,");
  Serial.print(actualSampleRate, 1);
  Serial.print(",");
  Serial.print(TB_OPTIONS[tbIndex]);
  Serial.print(",");
  Serial.print(VD_OPTIONS[vdIndex], 2);
  Serial.print(",");
  Serial.print(triggerLevel, 4);
  Serial.print(",");
  Serial.print(measFreq, 2);
  Serial.print(",");
  Serial.print(measVpp, 4);
  Serial.print(",");
  Serial.print(measVmin, 4);
  Serial.print(",");
  Serial.print(measVmax, 4);
  Serial.print(",");
  Serial.print(measVrms, 4);
  Serial.print(",");
  Serial.print(PLOT_W);
  for (int i = 0; i < PLOT_W; ++i) {
    Serial.print(",");
    Serial.print(acquireBuf[startIdx + i]);
  }
  Serial.println();
}

void sendDmmReading() {
  Serial.print("$DMM,");
  Serial.print(dmmAvg, 5);
  Serial.print(",");
  Serial.print(dmmMin, 5);
  Serial.print(",");
  Serial.print(dmmMax, 5);
  Serial.print(",");
  Serial.print(dmmVrms, 5);
  Serial.print(",");
  Serial.print(dmmRefV, 5);
  Serial.print(",");
  Serial.print(dmmRefSet ? 1 : 0);
  Serial.print(",");
  Serial.println(dmmShowVac ? 1 : 0);
}

// ======================= Serial Command Handler =======================
static char serialCmdBuf[32];
static uint8_t serialCmdLen = 0;

void processSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        handleSerialCmd(serialCmdBuf);
        serialCmdLen = 0;
      }
    } else {
      if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
        serialCmdBuf[serialCmdLen++] = c;
      }
    }
  }
}

void handleSerialCmd(const char* cmd) {
  if (cmd[0] != '!') return;
  const char* c = cmd + 1;  // skip '!'

  // Mode-independent commands
  if (strcmp(c, "CFG") == 0) {
    sendConfig();
    sendMode(appMode == MODE_SCOPE ? "SCOPE" : "DMM");
    return;
  }

  if (appMode == MODE_SCOPE) {
    if (strcmp(c, "TB-") == 0) {
      onButtonAction(BTN_TB_FAST);
    } else if (strcmp(c, "TB+") == 0) {
      onButtonAction(BTN_TB_SLOW);
    } else if (strcmp(c, "VD+") == 0) {
      onButtonAction(BTN_VD_PLUS);
    } else if (strcmp(c, "VD-") == 0) {
      onButtonAction(BTN_VD_MINUS);
    } else if (strcmp(c, "OFS+") == 0) {
      onButtonAction(BTN_OFS_UP);
    } else if (strcmp(c, "OFS-") == 0) {
      onButtonAction(BTN_OFS_DN);
    } else if (strcmp(c, "RUN") == 0) {
      g_buttons[BTN_RUNSTOP].state = !g_buttons[BTN_RUNSTOP].state;
      onButtonAction(BTN_RUNSTOP);
    } else if (strcmp(c, "AUTO") == 0) {
      onButtonAction(BTN_AUTO);
    } else if (strncmp(c, "TRIG,", 5) == 0) {
      triggerLevel = atof(c + 5);
    } else if (strcmp(c, "MODE") == 0) {
      onButtonAction(BTN_MODE_SCOPE);
    }
  } else {
    // DMM mode
    if (strcmp(c, "RST") == 0) {
      onButtonAction(BTN_DMM_RSTMM);
    } else if (strcmp(c, "REF") == 0) {
      onButtonAction(BTN_DMM_SETREF);
    } else if (strcmp(c, "VAC") == 0) {
      onButtonAction(BTN_DMM_VAC);
    } else if (strcmp(c, "MODE") == 0) {
      onButtonAction(BTN_DMM_MODE);
    } else if (strncmp(c, "CAL,", 4) == 0) {
      float target = atof(c + 4);
      if (fabsf(dmmAvg) > 0.01f && target > 0) {
        PROBE_SCALE = PROBE_SCALE * (target / dmmAvg);
        dmmMin = dmmAvg;
        dmmMax = dmmAvg;
        sendConfig();
      }
    }
  }
}

// ======================= Mode Transitions =======================
void switchToDmm() {
  adc.stop();
  delay(5);
  appMode = MODE_DMM;
  sendMode("DMM");

  // Initialize DMM state (offset-corrected, so 0V is neutral)
  dmmAvg    = 0;
  dmmMin    = 9999;
  dmmMax    = -9999;
  dmmVrms   = 0;
  dmmRefV   = 0;
  dmmRefSet = false;
  dmmShowVac = false;
  dmmCalMode = false;

  if (!headless) {
    drawDmmChrome();
    initDmmButtons();
    drawButtons(true);
  }

  // Start ADC at DMM sample rate
  acquireCount = 0;
  frameReady = false;
  if (!adc.begin(AN_RESOLUTION_16, DMM_SAMPLE_RATE, ADC_BUFSIZE, ADC_NBUFFERS)) {
    if (!headless) {
      gfx.setTextColor(COL_STOPPED);
      gfx.setTextSize(2);
      gfx.setCursor(20, SCREEN_H / 2);
      gfx.print("ADC INIT FAILED");
    }
  }
}

void switchToScope() {
  adc.stop();
  delay(5);
  appMode = MODE_SCOPE;
  sendMode("SCOPE");

  if (!headless) {
    drawStaticChrome();
    drawGrid();
    drawTriggerIndicator();

    initScopeButtons();
    drawButtons(true);
    drawInfoBar();
  }

  // Reset trace state
  firstTrace = true;
  acquireCount = 0;
  frameReady = false;
  for (int i = 0; i < PLOT_W; ++i) {
    oldTraceY[i] = PLOT_Y + PLOT_H / 2;
    newTraceY[i] = PLOT_Y + PLOT_H / 2;
  }

  // Restart ADC with scope sample rate
  running = true;
  g_buttons[BTN_RUNSTOP].state = true;
  if (!adc.begin(AN_RESOLUTION_16, calcSampleRate(), ADC_BUFSIZE, ADC_NBUFFERS)) {
    if (!headless) {
      gfx.setTextColor(COL_STOPPED);
      gfx.setTextSize(2);
      gfx.setCursor(PLOT_X + 20, PLOT_Y + PLOT_H / 2);
      gfx.print("ADC INIT FAILED");
    }
  }
}

// ======================= Scheduled Tasks =======================
void task_touch() {
  handleTouch();
  drawButtons(false);
}

void task_infoBar() {
  if (appMode == MODE_SCOPE) {
    drawInfoBar();
  } else {
    refreshDmmReading();
  }
}

void task_serialDmm() {
  // In headless DMM mode, periodically send DMM readings (replaces task_infoBar drawing)
  if (appMode == MODE_DMM) {
    sendDmmReading();
  }
}

// ======================= Setup =======================
void setup() {
  Serial.begin(115200);

  // Check headless pin early
  pinMode(2, INPUT_PULLUP);
  delay(10);
  headless = (digitalRead(2) == LOW);

  if (!headless) {
    initColors();
    gfx.begin();
    gfx.setRotation(1);  // landscape 800x480
    touchCtrl.begin();

    drawStaticChrome();
    drawGrid();
    drawTriggerIndicator();

    initScopeButtons();
    drawButtons(true);
    drawInfoBar();
  } else {
    // In headless mode, still init RUN/STOP state for serial commands
    g_buttons[BTN_RUNSTOP].state = true;
  }

  // Initialize trace arrays to center of plot
  for (int i = 0; i < PLOT_W; ++i) {
    oldTraceY[i] = PLOT_Y + PLOT_H / 2;
    newTraceY[i] = PLOT_Y + PLOT_H / 2;
  }

  // Start ADC
  if (!adc.begin(AN_RESOLUTION_16, calcSampleRate(), ADC_BUFSIZE, ADC_NBUFFERS)) {
    if (!headless) {
      gfx.setTextColor(COL_STOPPED);
      gfx.setTextSize(3);
      gfx.setCursor(PLOT_X + 80, PLOT_Y + PLOT_H / 2 - 12);
      gfx.print("ADC INIT FAILED");
    }
    while (1) delay(1000);
  }

  // Send initial config and mode to PC
  sendConfig();
  sendMode("SCOPE");

  // Scheduler
  if (!headless) {
    addTask(task_touch, 30);     // touch at ~33 Hz
    addTask(task_infoBar, 250);  // info bar at 4 Hz
  } else {
    addTask(task_serialDmm, 250);  // headless DMM streaming at 4 Hz
  }
}

// ======================= Loop =======================
void loop() {
  processSerialCommand();
  runTasks();
  if (appMode == MODE_SCOPE) {
    acquireSamples();
    processAndRender();
  } else {
    dmmAcquireAndCompute();
  }
}
