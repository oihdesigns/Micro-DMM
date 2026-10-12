/*
  NeoPixelWiFi

  Arduino UNO R4 WiFi driving an Adafruit 60-pixel NeoPixel (WS2812B, GRB) strip, with a
  web page anyone on the WiFi network can open to run it.

  Hardware:
    D6  -> strip DIN (through a 300-500 ohm series resistor if you have one)
    5V  -> strip +5V, GND -> strip GND (common ground with the Uno is required)
    Put a 500-1000 uF cap across the strip's 5V/GND at the input end.

    POWER: 60 pixels at full white is ~3.6 A. Do NOT run that from the Uno's 5V pin.
    Feed the strip from its own 5 V supply and set MAX_MILLIAMPS to what that supply
    can deliver -- the sketch dims each frame so its estimated draw stays under it.
    (On USB power only, leave it at 400 or so.)

  WiFi:
    Put your network name/password in arduino_secrets.h (copy arduino_secrets.example.h).
    On boot the board prints its IP address on the Serial monitor (115200 baud) and
    scrolls it across the LED matrix. Browse to http://<that IP>/ from the same network.

  Web API (GET or POST; the page uses POST, curl/scripts can use either):
    /api/state                -> JSON status
    /api/on  /api/off  /api/toggle
    /api/color?hex=RRGGBB     -> main colour (used by solid/breathe/chase/comet/twinkle)
    /api/bright?v=0..255      -> brightness
    /api/speed?v=1..100       -> animation speed
    /api/effect?n=<name>      -> solid | breathe | rainbow | colorcycle | chase | comet | twinkle | fire

  Serial commands (115200 baud, newline terminated):
    ON | OFF | TOGGLE | COLOR RRGGBB | BRIGHT n | SPEED n | EFFECT name | STATUS | IP

  Settings are saved to EEPROM a few seconds after the last change and restored at boot.
*/

#include <WiFiS3.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include "ArduinoGraphics.h"   // must come before Arduino_LED_Matrix for text scrolling
#include "Arduino_LED_Matrix.h"
#include "arduino_secrets.h"

// ---------------- configuration ----------------
#define PIXEL_PIN        6
#define PIXEL_COUNT      240
#define PIXEL_TYPE       (NEO_GRB + NEO_KHZ800)
#define MAX_MILLIAMPS    10000     // supply budget for the strip; see POWER above

#define HOSTNAME         "neopixel-strip"
#define USE_STATIC_IP    0        // 1 = use the addresses below instead of DHCP
IPAddress staticIP(192, 168, 1, 51);
IPAddress staticDNS(192, 168, 1, 1);
IPAddress staticGW(192, 168, 1, 1);
IPAddress staticMask(255, 255, 255, 0);

const unsigned long FRAME_MS        = 20;     // ~50 fps
const unsigned long SAVE_DELAY_MS   = 5000;   // write EEPROM this long after the last change
const unsigned long WIFI_RETRY_MS   = 15000;
const unsigned long HTTP_TIMEOUT_MS = 1000;

// Per-pixel current estimate for the limiter (WS2812B: ~20 mA per channel at full, ~1 mA idle).
const uint16_t MA_PER_CHANNEL = 20;
const uint16_t MA_IDLE_PER_PX = 1;

// ---------------- effects ----------------
enum Effect : uint8_t { FX_SOLID, FX_BREATHE, FX_RAINBOW, FX_COLORCYCLE, FX_CHASE, FX_COMET, FX_TWINKLE, FX_FIRE, FX_COUNT };
const char* const EFFECT_NAMES[FX_COUNT] = { "solid", "breathe", "rainbow", "colorcycle", "chase", "comet", "twinkle", "fire" };

// ---------------- state ----------------
struct Settings {
  uint32_t magic;
  bool     on;
  uint8_t  r, g, b;
  uint8_t  bright;     // 0..255
  uint8_t  speed;      // 1..100
  uint8_t  effect;
};
const uint32_t SETTINGS_MAGIC = 0x4E505831;   // "NPX1" -- bump if Settings changes
const int      SETTINGS_ADDR  = 0;

Settings cfg = { SETTINGS_MAGIC, true, 255, 120, 20, 96, 40, FX_RAINBOW };

bool          dirty = false;          // settings changed since last EEPROM save
unsigned long lastChangeMs = 0;
bool          needRender = true;      // force a frame even for static effects

uint32_t      frame[PIXEL_COUNT];     // full-scale colours; brightness/limit applied at show()
uint8_t       heat[PIXEL_COUNT];      // fire effect
uint32_t      phase = 0;              // animation clock, advances at a rate set by speed
unsigned long lastFrameMs = 0;
uint8_t       shownBright = 0;        // brightness actually used last frame (after limiting)
uint16_t      estMilliamps = 0;

unsigned long lastWifiCheckMs = 0;
bool          wasConnected = false;

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
WiFiServer        server(80);
ArduinoLEDMatrix  matrix;

// ---------------- helpers ----------------
uint32_t mainColor() { return Adafruit_NeoPixel::Color(cfg.r, cfg.g, cfg.b); }

uint32_t scaleColor(uint32_t c, uint8_t s) {
  uint8_t r = (uint8_t)(c >> 16), g = (uint8_t)(c >> 8), b = (uint8_t)c;
  return Adafruit_NeoPixel::Color((r * s) >> 8, (g * s) >> 8, (b * s) >> 8);
}

// Triangle wave 0..255..0 over a 512-step period (cheaper and smoother-looking than sin for breathing).
uint8_t triwave(uint32_t x) {
  x &= 511;
  return x < 256 ? x : 511 - x;
}

void markChanged() {
  dirty = true;
  lastChangeMs = millis();
  needRender = true;
}

int effectFromName(String name) {
  name.trim();
  name.toLowerCase();
  for (int i = 0; i < FX_COUNT; i++) if (name == EFFECT_NAMES[i]) return i;
  return -1;
}

// ---------------- settings ----------------
void loadSettings() {
  Settings s;
  EEPROM.get(SETTINGS_ADDR, s);
  if (s.magic == SETTINGS_MAGIC && s.effect < FX_COUNT && s.speed >= 1 && s.speed <= 100) {
    cfg = s;
    Serial.println("Settings restored from EEPROM.");
  } else {
    Serial.println("No saved settings -- using defaults.");
  }
}

void saveSettingsIfIdle() {
  if (dirty && millis() - lastChangeMs >= SAVE_DELAY_MS) {
    EEPROM.put(SETTINGS_ADDR, cfg);
    dirty = false;
    Serial.println("Settings saved.");
  }
}

// ---------------- setters (shared by web + serial) ----------------
void setPower(bool on) {
  cfg.on = on;
  markChanged();
  Serial.print("Strip ");
  Serial.println(on ? "ON" : "OFF");
}

bool setColorHex(String hex) {
  hex.trim();
  if (hex.startsWith("#")) hex = hex.substring(1);
  if (hex.length() != 6) return false;
  for (unsigned i = 0; i < 6; i++) if (!isHexadecimalDigit(hex[i])) return false;
  uint32_t v = strtoul(hex.c_str(), nullptr, 16);
  cfg.r = v >> 16;
  cfg.g = v >> 8;
  cfg.b = v;
  markChanged();
  return true;
}

bool setBright(long v) {
  if (v < 0 || v > 255) return false;
  cfg.bright = v;
  markChanged();
  return true;
}

bool setSpeed(long v) {
  if (v < 1 || v > 100) return false;
  cfg.speed = v;
  markChanged();
  return true;
}

bool setEffect(const String& name) {
  int fx = effectFromName(name);
  if (fx < 0) return false;
  if (fx != cfg.effect) {
    cfg.effect = fx;
    memset(frame, 0, sizeof(frame));   // comet/twinkle fade from what's there; start clean
    memset(heat, 0, sizeof(heat));
  }
  markChanged();
  return true;
}

// ---------------- rendering ----------------
// Each effect fills frame[] at full scale. `phase` advances faster at higher speed.
void renderEffect() {
  switch (cfg.effect) {
    case FX_SOLID:
      for (int i = 0; i < PIXEL_COUNT; i++) frame[i] = mainColor();
      break;

    case FX_BREATHE: {
      // Floor at ~8% so it never goes fully dark mid-breath.
      uint8_t lvl = 20 + (triwave(phase >> 3) * 235) / 255;
      uint32_t c = scaleColor(mainColor(), lvl);
      for (int i = 0; i < PIXEL_COUNT; i++) frame[i] = c;
      break;
    }

    case FX_RAINBOW: {
      uint16_t base = phase * 16;
      for (int i = 0; i < PIXEL_COUNT; i++)
        frame[i] = strip.gamma32(strip.ColorHSV(base + (uint32_t)i * 65536 / PIXEL_COUNT));
      break;
    }

    case FX_COLORCYCLE: {
      uint32_t c = strip.gamma32(strip.ColorHSV(phase * 16));
      for (int i = 0; i < PIXEL_COUNT; i++) frame[i] = c;
      break;
    }

    case FX_CHASE: {
      // Theater chase: every 3rd pixel lit, stepping along.
      int offset = (phase >> 6) % 3;
      for (int i = 0; i < PIXEL_COUNT; i++) frame[i] = (i % 3 == offset) ? mainColor() : 0;
      break;
    }

    case FX_COMET: {
      for (int i = 0; i < PIXEL_COUNT; i++) frame[i] = scaleColor(frame[i], 200);   // fading tail
      // Bounce the head back and forth.
      uint32_t span = 2 * (PIXEL_COUNT - 1);
      uint32_t pos = (phase >> 4) % span;
      int head = pos < PIXEL_COUNT ? pos : span - pos;
      frame[head] = mainColor();
      break;
    }

    case FX_TWINKLE: {
      for (int i = 0; i < PIXEL_COUNT; i++) frame[i] = scaleColor(frame[i], 235);
      // Spawn rate scales with speed: ~0.3 to ~3 new sparkles per frame on average.
      int spawns = random(0, 2 + cfg.speed / 15);
      for (int k = 0; k < spawns; k++) frame[random(PIXEL_COUNT)] = mainColor();
      break;
    }

    case FX_FIRE: {
      // Fire2012 (Mark Kriegsman), with the strip's start as the base of the flame.
      const uint8_t COOLING = 55, SPARKING = 120;
      for (int i = 0; i < PIXEL_COUNT; i++) {
        int cool = random(0, (COOLING * 10) / PIXEL_COUNT + 2);
        heat[i] = heat[i] > cool ? heat[i] - cool : 0;
      }
      for (int i = PIXEL_COUNT - 1; i >= 2; i--) heat[i] = (heat[i - 1] + 2 * heat[i - 2]) / 3;
      if (random(255) < SPARKING) {
        int y = random(7);
        heat[y] = min(255, heat[y] + (int)random(160, 255));
      }
      for (int i = 0; i < PIXEL_COUNT; i++) {
        uint8_t t = (heat[i] * 191) / 255;           // 0..191
        uint8_t ramp = (t & 0x3F) << 2;              // 0..252 within each third
        if (t > 0x80)      frame[i] = Adafruit_NeoPixel::Color(255, 255, ramp);
        else if (t > 0x40) frame[i] = Adafruit_NeoPixel::Color(255, ramp, 0);
        else               frame[i] = Adafruit_NeoPixel::Color(ramp, 0, 0);
      }
      break;
    }
  }
}

// ---------------- strip output ----------------
// Replacement for strip.show(). The library's RA4M1 driver times each high pulse with a
// polling loop on the cycle counter. Each pass of that loop costs ~8-10 cycles out of flash,
// so a 0-bit (target 12 cycles = 250 ns) actually comes out anywhere up to ~20+ cycles --
// right on this strip's 0/1 threshold. With the library, only the first (cold-cache) bit
// crossed it, so pixel 0 was stuck green; any extra loop overhead made every bit a 1 (all white).
//
// Here the high time has no loop or branch in it: raise the pin, a fixed run of NOPs, drop
// it early (0-bit) or not (writing 0 to PORR is a no-op), more NOPs, drop it. The cycle
// counter only paces the 1.25 us bit period, where a few cycles of jitter don't matter.
// The loop is first run against a dummy RAM "port" so the real pass starts with a warm
// flash cache.
#define ARM_DEMCR        (*(volatile uint32_t *)0xE000EDFC)
#define ARM_DWT_CTRL     (*(volatile uint32_t *)0xE0001000)
#define ARM_DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004)
#define F_CPU_HZ         48000000UL     // UNO R4 core clock
const uint32_t CYC_BIT = F_CPU_HZ / 800000;    // 60 cycles = 1.25 us

// High times, in NOPs (~1 cycle = 20.8 ns each, plus ~1-2 cycles for the store).
// 0-bit ~ T0H_NOPS + 1;  1-bit ~ T0H_NOPS + T1H_EXTRA_NOPS + 2.
#define T0H_NOPS         13    // ~0.30 us  (WS2812B spec 0.22-0.38 us)
#define T1H_EXTRA_NOPS   24    // 1-bit ~0.81 us (spec 0.58-1.0 us)

#define NOPS(n) __asm__ volatile(".rept " #n "\n\tnop\n\t.endr")
#define NOPS_(n) NOPS(n)

volatile uint16_t *pixSet, *pixClr;
uint16_t          pixMask;
unsigned long     lastSendUs = 0;

static void __attribute__((noinline)) sendBytes(volatile uint16_t* set, volatile uint16_t* clr,
                                                uint16_t msk, const uint8_t* p, const uint8_t* end) {
  uint32_t cyc = ARM_DWT_CYCCNT - CYC_BIT;
  while (p < end) {
    uint32_t pix = *p++;
    for (int b = 0; b < 8; b++) {
      // Value for the early PORR write: msk for a 0-bit (drop now), 0 for a 1-bit (no-op).
      uint16_t early = (uint16_t)(msk & ((pix & 0x80) ? 0 : 0xFFFF));
      pix <<= 1;
      while (ARM_DWT_CYCCNT - cyc < CYC_BIT) ;
      cyc = ARM_DWT_CYCCNT;
      *set = msk;
      NOPS_(T0H_NOPS);
      *clr = early;
      NOPS_(T1H_EXTRA_NOPS);
      *clr = msk;
    }
  }
  while (ARM_DWT_CYCCNT - cyc < CYC_BIT) ;
}

void beginStripOutput() {
  bsp_io_port_pin_t io = g_pin_cfg[PIXEL_PIN].pin;
  R_PORT0_Type* port = (R_PORT0_Type*)((uint32_t)R_PORT0 + ((uint32_t)R_PORT1 - (uint32_t)R_PORT0) * (io >> 8));
  pixSet  = &port->POSR;
  pixClr  = &port->PORR;
  pixMask = 1U << (io & 0xFF);
  ARM_DEMCR    |= (1UL << 24);   // enable DWT
  ARM_DWT_CTRL |= 1UL;           // enable cycle counter
}

void sendStrip() {
  static volatile uint16_t dummy;
  static const uint8_t warm[2] = { 0x00, 0xFF };   // exercise both the 0 and 1 paths
  while (micros() - lastSendUs < 300) ;            // WS2812 latch/reset time
  const uint8_t* px = strip.getPixels();
  noInterrupts();
  sendBytes(&dummy, &dummy, pixMask, warm, warm + sizeof(warm));
  sendBytes(pixSet, pixClr, pixMask, px, px + PIXEL_COUNT * 3);
  interrupts();
  lastSendUs = micros();
}

bool effectIsAnimated() {
  return cfg.effect != FX_SOLID;
}

// Push frame[] to the strip at the user's brightness, dimmed further if the estimated
// current would exceed MAX_MILLIAMPS.
void showFrame() {
  uint32_t sum = 0;
  for (int i = 0; i < PIXEL_COUNT; i++) {
    uint32_t c = frame[i];
    sum += ((c >> 16) & 0xFF) + ((c >> 8) & 0xFF) + (c & 0xFF);
  }
  uint8_t bright = cfg.on ? cfg.bright : 0;
  const uint32_t idle   = (uint32_t)MA_IDLE_PER_PX * PIXEL_COUNT;
  const uint32_t budget = MAX_MILLIAMPS > idle ? MAX_MILLIAMPS - idle : 0;
  uint32_t fullMa = sum * MA_PER_CHANNEL / 255;     // draw at brightness 255
  if (fullMa > 0 && (uint32_t)bright * fullMa / 255 > budget) {
    bright = budget * 255 / fullMa;
  }
  shownBright  = bright;
  estMilliamps = idle + (uint32_t)bright * fullMa / 255;

  // setBrightness() then setPixelColor() scales on the way in; frame[] stays full-scale.
  strip.setBrightness(bright);
  for (int i = 0; i < PIXEL_COUNT; i++) strip.setPixelColor(i, frame[i]);
  sendStrip();
}

void updateStrip() {
  unsigned long now = millis();
  unsigned long dt = now - lastFrameMs;
  if (dt < FRAME_MS) return;
  lastFrameMs = now;
  if (dt > 200) dt = FRAME_MS;   // don't leap ahead after a blocking WiFi call

  if (!cfg.on) {
    // Only need to send the "off" frame once.
    if (needRender) { showFrame(); needRender = false; }
    return;
  }
  if (!effectIsAnimated() && !needRender) return;

  phase += dt * cfg.speed / 10;
  renderEffect();
  showFrame();
  needRender = false;
}

// ---------------- status ----------------
String stateJson() {
  char hex[8];
  snprintf(hex, sizeof(hex), "%02X%02X%02X", cfg.r, cfg.g, cfg.b);
  String j = "{\"on\":";
  j += cfg.on ? "true" : "false";
  j += ",\"color\":\"";  j += hex;  j += "\"";
  j += ",\"bright\":";   j += cfg.bright;
  j += ",\"shown\":";    j += shownBright;
  j += ",\"ma\":";       j += estMilliamps;
  j += ",\"maxma\":";    j += MAX_MILLIAMPS;
  j += ",\"speed\":";    j += cfg.speed;
  j += ",\"effect\":\""; j += EFFECT_NAMES[cfg.effect]; j += "\"";
  j += ",\"effects\":[";
  for (int i = 0; i < FX_COUNT; i++) {
    if (i) j += ",";
    j += "\""; j += EFFECT_NAMES[i]; j += "\"";
  }
  j += "],\"pixels\":";  j += PIXEL_COUNT;
  j += ",\"rssi\":";     j += WiFi.RSSI();
  j += ",\"uptime\":";   j += millis() / 1000;
  j += "}";
  return j;
}

void printStatus() {
  Serial.println(stateJson());
  Serial.print("Web page: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

// ---------------- serial ----------------
void handleSerial() {
  if (Serial.available() <= 0) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  if (command.length() == 0) return;
  String upper = command;
  upper.toUpperCase();
  String arg = command.indexOf(' ') > 0 ? command.substring(command.indexOf(' ') + 1) : "";
  arg.trim();

  bool ok = true;
  if (upper == "ON")                    setPower(true);
  else if (upper == "OFF")              setPower(false);
  else if (upper == "TOGGLE")           setPower(!cfg.on);
  else if (upper.startsWith("COLOR "))  ok = setColorHex(arg);
  else if (upper.startsWith("BRIGHT ")) ok = arg.length() > 0 && setBright(arg.toInt());
  else if (upper.startsWith("SPEED "))  ok = arg.length() > 0 && setSpeed(arg.toInt());
  else if (upper.startsWith("EFFECT ")) ok = setEffect(arg);
  else if (upper == "STATUS" || upper == "IP") printStatus();
  else {
    Serial.println("Commands: ON | OFF | TOGGLE | COLOR RRGGBB | BRIGHT 0-255 | SPEED 1-100 | EFFECT name | STATUS | IP");
    Serial.print("Effects:");
    for (int i = 0; i < FX_COUNT; i++) { Serial.print(' '); Serial.print(EFFECT_NAMES[i]); }
    Serial.println();
    return;
  }
  Serial.println(ok ? stateJson() : "Invalid value.");
}

// ---------------- WiFi ----------------
void showOnMatrix(const String& text) {
  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(60);
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println("  " + text + "  ");
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();
}

// One connection attempt. WiFi.begin() blocks for a few seconds, so the animation
// pauses while this runs -- it only happens at boot and after a dropout.
void connectWifi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not found -- serial control only.");
    return;
  }
  WiFi.setHostname(HOSTNAME);
#if USE_STATIC_IP
  WiFi.config(staticIP, staticDNS, staticGW, staticMask);
#endif
  Serial.print("Connecting to ");
  Serial.println(SECRET_SSID);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
}

void maintainWifi() {
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wasConnected) {
    // DHCP can lag the association by a moment; wait for a real address.
    if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) return;
    server.begin();
    Serial.print("Connected. Web page: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    showOnMatrix(WiFi.localIP().toString());
  }
  if (!connected && wasConnected) {
    Serial.println("WiFi lost -- will retry.");
    lastWifiCheckMs = millis();
  }
  wasConnected = connected;

  if (!connected && millis() - lastWifiCheckMs >= WIFI_RETRY_MS) {
    lastWifiCheckMs = millis();
    WiFi.disconnect();
    connectWifi();
  }
}

// ---------------- web ----------------
const char PAGE_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NeoPixel Strip</title>
<style>
:root{--bg:#f4f4f2;--card:#fff;--fg:#1d1d1b;--muted:#6b6b66;--line:#ddddd8;--accent:#6a5cff;--accent-fg:#fff;--off:#c9c9c4;--bad:#c0392b}
@media (prefers-color-scheme:dark){:root{--bg:#141414;--card:#1f1f1f;--fg:#f0f0ec;--muted:#9a9a94;--line:#333;--accent:#8b80ff;--off:#3a3a3a}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:440px;margin:0 auto;padding:24px 16px}
h1{font-size:1.4rem;margin:0 0 16px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:18px;margin-bottom:14px}
#power{width:100%;padding:22px;font-size:1.3rem;font-weight:600;border:0;border-radius:12px;cursor:pointer;background:var(--off);color:var(--fg);transition:background .15s}
#power.on{background:var(--swatch,var(--accent));color:#111;text-shadow:0 0 6px #fff8}
.label{font-size:.85rem;color:var(--muted);margin-bottom:10px;display:flex;justify-content:space-between}
.fx{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.fx button{padding:12px 0;font-size:.85rem;font-weight:600;border:1px solid var(--line);border-radius:10px;background:transparent;color:var(--fg);cursor:pointer;text-transform:capitalize}
.fx button.sel{background:var(--accent);color:var(--accent-fg);border-color:var(--accent)}
.colors{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.colors button{width:36px;height:36px;border-radius:50%;border:2px solid var(--line);cursor:pointer;padding:0}
input[type=color]{width:52px;height:40px;border:1px solid var(--line);border-radius:10px;background:transparent;padding:2px;cursor:pointer}
input[type=range]{width:100%;accent-color:var(--accent);height:32px}
#status{font-size:.85rem;color:var(--muted);text-align:center}
#status.bad{color:var(--bad)}
</style></head><body><main>
<h1>NeoPixel Strip</h1>
<div class="card"><button id="power">…</button></div>
<div class="card">
  <div class="label"><span>Effect</span></div>
  <div class="fx" id="fx"></div>
</div>
<div class="card">
  <div class="label"><span>Color</span><span id="hexText"></span></div>
  <div class="colors" id="colors"><input type="color" id="pick"></div>
</div>
<div class="card">
  <div class="label"><span>Brightness</span><span id="brText"></span></div>
  <input type="range" id="bright" min="0" max="255" step="1">
  <div class="label" style="margin-top:12px"><span>Speed</span><span id="spText"></span></div>
  <input type="range" id="speed" min="1" max="100" step="1">
</div>
<div id="status">Connecting…</div>
</main><script>
const $=id=>document.getElementById(id);
const presets=['FF0000','FF7800','FFD000','00FF40','00C8FF','0030FF','A000FF','FF00A0','FFFFFF','FFB060'];
const cols=$('colors');
presets.forEach(h=>{const b=document.createElement('button');b.style.background='#'+h;b.title='#'+h;b.onclick=()=>send('/api/color?hex='+h);cols.insertBefore(b,$('pick'));});
let builtFx=false,active=null;
function render(s){
  if(!builtFx){s.effects.forEach(n=>{const b=document.createElement('button');b.textContent=n;b.dataset.n=n;b.onclick=()=>send('/api/effect?n='+n);$('fx').appendChild(b);});builtFx=true;}
  const p=$('power');p.textContent=s.on?'ON':'OFF';p.classList.toggle('on',s.on);p.style.setProperty('--swatch','#'+s.color);
  [...$('fx').children].forEach(b=>b.classList.toggle('sel',b.dataset.n===s.effect));
  $('hexText').textContent='#'+s.color;
  if(active!=='pick')$('pick').value='#'+s.color.toLowerCase();
  if(active!=='bright')$('bright').value=s.bright;
  if(active!=='speed')$('speed').value=s.speed;
  $('brText').textContent=Math.round(s.bright*100/255)+'%'+(s.on&&s.shown<s.bright?' (power-limited to '+Math.round(s.shown*100/255)+'%)':'');
  $('spText').textContent=s.speed;
  const st=$('status');st.classList.remove('bad');
  st.textContent='Connected · ~'+s.ma+' mA of '+s.maxma+' · signal '+s.rssi+' dBm';
}
function fail(){const st=$('status');st.classList.add('bad');st.textContent='Lost contact with the strip controller';}
// One request at a time (the board serves one connection at once); while a request is
// in flight, keep only the newest pending one so sliders stay live without piling up.
let busy=false,pending=null;
async function send(url){
  if(busy){pending=url;return;}
  busy=true;
  try{const r=await fetch(url,{method:'POST'});render(await r.json());}catch(e){fail();}
  busy=false;
  if(pending){const u=pending;pending=null;send(u);}
}
async function poll(){if(busy||active)return;try{const r=await fetch('/api/state');render(await r.json());}catch(e){fail();}}
$('power').onclick=()=>send('/api/toggle');
function live(id,url,fmt){const el=$(id);
  el.addEventListener('input',()=>{active=id;if(fmt)fmt(el.value);send(url+encodeURIComponent(el.value.replace('#','')));});
  el.addEventListener('change',()=>{active=null;send(url+encodeURIComponent(el.value.replace('#','')));});}
live('bright','/api/bright?v=',v=>$('brText').textContent=Math.round(v*100/255)+'%');
live('speed','/api/speed?v=',v=>$('spText').textContent=v);
live('pick','/api/color?hex=',v=>$('hexText').textContent=v.toUpperCase());
poll();setInterval(poll,3000);
</script></body></html>
)HTML";

// Value of ?key=... in a path, or "" if absent.
String queryParam(const String& path, const char* key) {
  int q = path.indexOf('?');
  if (q < 0) return "";
  String k = String(key) + "=";
  int start = q + 1;
  while (start < (int)path.length()) {
    int amp = path.indexOf('&', start);
    if (amp < 0) amp = path.length();
    if (path.substring(start, start + k.length()) == k)
      return path.substring(start + k.length(), amp);
    start = amp + 1;
  }
  return "";
}

void sendResponse(WiFiClient& c, int code, const char* type, const char* body, size_t len) {
  c.print("HTTP/1.1 ");
  c.print(code);
  c.println(code == 200 ? " OK" : (code == 404 ? " Not Found" : " Bad Request"));
  c.print("Content-Type: ");
  c.println(type);
  c.print("Content-Length: ");
  c.println(len);
  c.println("Cache-Control: no-store");
  c.println("Connection: close");
  c.println();
  // The R4's WiFi bridge is happier with modest writes than one big one.
  const size_t CHUNK = 512;
  for (size_t i = 0; i < len; i += CHUNK) {
    c.write((const uint8_t*)body + i, min(CHUNK, len - i));
  }
}

void sendJson(WiFiClient& c, int code, const String& json) {
  sendResponse(c, code, "application/json", json.c_str(), json.length());
}

void handleWeb() {
  if (!wasConnected) return;   // server isn't started until we have an address
  WiFiClient c = server.available();
  if (!c) return;

  c.setTimeout(HTTP_TIMEOUT_MS);
  String requestLine = c.readStringUntil('\n');
  requestLine.trim();
  // Drain the headers; we don't need any of them.
  unsigned long t0 = millis();
  while (c.connected() && millis() - t0 < HTTP_TIMEOUT_MS) {
    String h = c.readStringUntil('\n');
    if (h.length() <= 1) break;   // blank line ("\r") ends the headers
  }

  // "GET /path?query HTTP/1.1"
  int sp1 = requestLine.indexOf(' ');
  int sp2 = requestLine.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) { c.stop(); return; }
  String path = requestLine.substring(sp1 + 1, sp2);
  String route = path;
  int q = route.indexOf('?');
  if (q >= 0) route = route.substring(0, q);

  bool ok = true;
  const char* err = "";
  if (route == "/" || route == "/index.html") {
    sendResponse(c, 200, "text/html; charset=utf-8", PAGE_HTML, strlen(PAGE_HTML));
    ok = false; err = nullptr;   // already answered
  } else if (route == "/api/state") {
    // just report
  } else if (route == "/api/on") {
    setPower(true);
  } else if (route == "/api/off") {
    setPower(false);
  } else if (route == "/api/toggle") {
    setPower(!cfg.on);
  } else if (route == "/api/color") {
    ok = setColorHex(queryParam(path, "hex"));
    err = "{\"error\":\"hex must be RRGGBB\"}";
  } else if (route == "/api/bright") {
    String v = queryParam(path, "v");
    ok = v.length() > 0 && setBright(v.toInt());
    err = "{\"error\":\"v must be 0-255\"}";
  } else if (route == "/api/speed") {
    String v = queryParam(path, "v");
    ok = v.length() > 0 && setSpeed(v.toInt());
    err = "{\"error\":\"v must be 1-100\"}";
  } else if (route == "/api/effect") {
    ok = setEffect(queryParam(path, "n"));
    err = "{\"error\":\"unknown effect\"}";
  } else {
    sendJson(c, 404, "{\"error\":\"not found\"}");
    ok = false; err = nullptr;
  }

  if (ok) {
    updateStrip();   // so the reply's shown/ma reflect the change
    sendJson(c, 200, stateJson());
  } else if (err) {
    sendJson(c, 400, err);
  }

  // Let the bytes leave before closing -- the R4 truncates if stop() comes too soon.
  c.flush();
  delay(2);
  c.stop();
}

// ---------------- main ----------------
void setup() {
  Serial.begin(115200);
  matrix.begin();
  strip.begin();
  strip.clear();
  beginStripOutput();
  sendStrip();
  randomSeed(analogRead(A0));

  loadSettings();
  needRender = true;

  connectWifi();
  lastWifiCheckMs = millis();
}

void loop() {
  updateStrip();
  handleSerial();
  maintainWifi();
  handleWeb();
  saveSettingsIfIdle();
}
