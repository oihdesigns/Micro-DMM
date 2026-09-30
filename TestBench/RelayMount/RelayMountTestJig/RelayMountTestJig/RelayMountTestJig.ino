/*
 * RelayMountTestJig.ino  --  relay load-selector jig
 *
 * Hardware: Arduino UNO R4 WiFi (Freenove V5) + two 4-channel relay boards
 *           on the RelayMount plate.  Relay channel n is driven by pin
 *           RELAY_PIN[n-1].  Channel 8 (K8) switches the BlinkyHawk's USB
 *           cable: it drives the coils of four external relays, one per USB
 *           wire (VBUS, D+, D-, GND).  K8 energised = USB connected.
 *
 * What the jig does (hand schematic 2026-09-26):
 *
 *   The DUT's +/- input leads go either straight to the function generator
 *   or into a relay-selected resistor network:
 *
 *     K1  DUT+   : NC -> FGen+             NO -> K3 common
 *     K2  DUT-   : NC -> FGen-             NO -> network return bus
 *     K3         : NC -> return (SHORT)    NO -> K4 common
 *     K4         : NC -> R1 1M -> K5       NO -> K6 common
 *     K5         : NC -> return (1M)       NO -> no connection (OPEN)
 *     K6         : NC -> R2 10k -> return  NO -> K7 common
 *     K7         : NC -> R3 10.6M -> return  NO -> R4 150k -> return
 *
 *   "NC" is the contact drawn as the rest position on the schematic, so with
 *   every relay released (power-up, reset, !ALL) the DUT sees the function
 *   generator.  If a relay was wired the other way round, flip its entry in
 *   RELAY_NO_SWAPPED below rather than editing the mode table.
 *
 *   Mode    K1 K2 K3 K4 K5 K6 K7   DUT sees
 *   FGEN     0  0  0  0  0  0  0   function generator
 *   SHORT    1  1  0  0  0  0  0   0 ohm
 *   OPEN     1  1  1  0  1  0  0   open circuit
 *   1M       1  1  1  0  0  0  0   R1
 *   10K      1  1  1  1  0  0  0   R2
 *   10.6M    1  1  1  1  0  1  0   R3
 *   150K     1  1  1  1  0  1  1   R4
 *
 * Break-before-make: every mode change walks through OPEN one relay at a
 * time (SETTLE ms apart), so the DUT never momentarily sees a *different*
 * resistance or a short while the relays are moving -- only open circuit.
 *
 * Serial protocol (115200, '\n' terminated):
 *   !MODE,<FGEN|SHORT|OPEN|1M|10K|10.6M|150K>   select a load
 *   !K,<1-7>,<0|1>      drive one relay directly (mode becomes RAW)
 *   !ALL                release K1-K7 (= FGEN, no sequencing).  USB untouched.
 *   !USB[,0|1]          connect (1) / disconnect (0) the BlinkyHawk's USB via
 *                       K8; bare = report.  (v1.2)
 *   !SETTLE,<ms>        relay settle time between steps (default 25)
 *   !STATE              report state
 *   !ID                 identify
 *   !HELP
 *
 * LED watcher (v1.1) -- an Adafruit AS7343 held over the BlinkyHawk's SK6812,
 * so the rig can see what the unit is ALERTING even when it runs on its own
 * battery with no serial link.  No electrical contact with the DUT at all,
 * which is the point: a scope probe ground would re-earth a floating unit.
 *   !LED[,0|1]          flash reporting on/off (on at boot if the sensor is found)
 *   !LEDRAW[,<ms>]      also stream raw samples every <ms> (0 = off) -- for
 *                       aiming the sensor and choosing the gain
 *   !LEDCFG[,<gain>,<atime>,<astep>,<thr>]   sensor gain (0-12 = 0.5x..2048x),
 *                       integration ATIME/ASTEP, and the flash threshold in
 *                       counts above the dark baseline; bare = report
 *   !LEDSTAT            report
 * Replies:
 *   $BOOT,RelayJig,<ver>
 *   $ID,RelayJig,<ver>
 *   $STATE,<mode>,<K1..K8 bits>,<what the DUT sees>,<settle ms>,<usb 0|1>
 *   $USB,<0|1>          reply to !USB
 *   $FLASH,<startMs>,<durMs>,<n>,<fz>,<fy>,<fxl>,<vis>,<peak>,<sat>
 *        one per LED flash, sent when it ENDS.  fz/fy/fxl/vis are the mean of
 *        the samples in the flash with the dark baseline subtracted (FZ 450 nm
 *        ~ blue, FY 555 nm ~ green, FXL 600 nm ~ red); peak is the brightest
 *        sample of fz+fy+fxl; sat=1 if any sample clipped (lower the gain)
 *   $LEDRAW,<ms>,<fz>,<fy>,<fxl>,<nir>,<vis>
 *   $LEDSTATE,sensor=..,on=..,gain=..,atime=..,astep=..,thr=..,hz=..,base=..,fullscale=..
 *   $ERR,<reason>
 *   anything not starting with '$' is human-readable debug.
 */

#define FW_VERSION "1.2"

// K8 = the BlinkyHawk's USB.  It is NOT part of the load selector: mode
// changes and !ALL never move it, only !USB does.  Boot state is CONNECTED, so
// the rig behaves like an ordinary cable until the test suite deliberately
// unplugs it.  (While this board is resetting its pins float and the relay
// drops out, so a jig reset still blips the USB -- connect the jig before the
// BlinkyHawk.)  If your USB relays connect with K8 RELEASED instead, set
// RELAY_NO_SWAPPED[K8] below rather than changing this.
#define USB_BOOT_CONNECTED 1

#include <Wire.h>
#include <Adafruit_AS7343.h>

// The AS7343 is on the header I2C pins (SDA/SCL, next to AREF) = Wire.  A
// STEMMA QT / Qwiic cable to a board that has that connector goes to Wire1 on
// the UNO R4 WiFi -- change this one line if that is how it is plugged in.
#define LED_WIRE Wire1

// Most cheap 4-channel relay boards (opto input, jumper JD-VCC) energise the
// relay when IN is pulled LOW.  Set to 0 for boards that are active-high.
#define RELAY_ACTIVE_LOW 1

const uint8_t NUM_RELAYS = 8;
const uint8_t RELAY_PIN[NUM_RELAYS] = {2, 3, 4, 5, 6, 7, 8, 9};  // K1..K8

// Set an entry to true if that relay's contacts are wired opposite to the
// schematic (the "NC" wire is on the NO terminal).  The coil drive is then
// inverted so the logical state -- and the mode table -- stay correct.
const bool RELAY_NO_SWAPPED[NUM_RELAYS] = {false, false, false, false,
                                           false, false, false, false};

enum Mode : uint8_t { M_FGEN, M_SHORT, M_OPEN, M_1M, M_10K, M_10M6, M_150K, M_RAW };
const char *const MODE_NAME[] = {"FGEN", "SHORT", "OPEN", "1M", "10K", "10.6M", "150K", "RAW"};
const uint8_t NUM_NAMED_MODES = 7;

// Logical relay indices (0-based) so the sequencing code reads like the schematic.
enum { K1, K2, K3, K4, K5, K6, K7, K8 };

bool relayOn[NUM_RELAYS];          // logical state: true = moved to the NO contact
bool usbOn = USB_BOOT_CONNECTED;   // K8: the BlinkyHawk's USB is connected
Mode currentMode = M_FGEN;
uint16_t settleMs = 25;

// ---------------------------------------------------------------------------
// Relay drive

void driveRelay(uint8_t k, bool on) {
  bool coil = on ^ RELAY_NO_SWAPPED[k];
  bool level = RELAY_ACTIVE_LOW ? !coil : coil;
  digitalWrite(RELAY_PIN[k], level ? HIGH : LOW);
  relayOn[k] = on;
}

// Move one relay and wait for its contacts to settle -- only if it changes.
void step(uint8_t k, bool on) {
  if (relayOn[k] == on) return;
  driveRelay(k, on);
  delay(settleMs);
}

// K1/K2 move together so both DUT leads change over in the same instant
// (never one lead on FGen and the other on the network).
void stepLeads(bool on) {
  if (relayOn[K1] == on && relayOn[K2] == on) return;
  driveRelay(K1, on);
  driveRelay(K2, on);
  delay(settleMs);
}

// ---------------------------------------------------------------------------
// What the DUT actually sees for the present relay state.  Used for the
// $STATE report, so RAW poking is always explained in load terms.

const char *resolveLoad() {
  if (!relayOn[K1] && !relayOn[K2]) return "FGEN";
  if (relayOn[K1] != relayOn[K2]) return "SPLIT";       // one lead on FGen, one on network
  if (!relayOn[K3]) return "SHORT";
  if (!relayOn[K4]) return relayOn[K5] ? "OPEN" : "1M";
  if (!relayOn[K6]) return "10K";
  return relayOn[K7] ? "150K" : "10.6M";
}

// ---------------------------------------------------------------------------
// Sequenced mode change

// Bring the network to OPEN.  Each step is safe from any starting state:
//   K5 on  : breaks the 1M path (harmless if K4 is routing elsewhere)
//   K4 off : routes through R1 -> K5(NO) = open, dropping 10k/10.6M/150k
//   K3 on  : lifts a short onto the now-open K4 path
void networkToOpen() {
  step(K5, true);
  step(K4, false);
  step(K3, true);
}

void setMode(Mode m) {
  // Open the network first (if the DUT is on FGen this happens off-line).
  networkToOpen();

  if (m == M_FGEN) {
    stepLeads(false);
    // Network is now disconnected; release its coils.
    for (uint8_t k = K3; k <= K7; k++) step(k, false);
    currentMode = m;
    return;
  }

  // Hand the DUT leads to the (open) network.
  stepLeads(true);

  // K6/K7 are out of the path while K4 is released -- preset them.
  step(K6, m == M_10M6 || m == M_150K);
  step(K7, m == M_150K);

  // Close the one relay that makes the target path, then release K5 if it
  // is no longer in the path (saves coil current, keeps the table canonical).
  switch (m) {
    case M_SHORT: step(K3, false); step(K5, false); break;
    case M_1M:    step(K5, false); break;
    case M_10K:
    case M_10M6:
    case M_150K:  step(K4, true);  step(K5, false); break;
    case M_OPEN:
    default:      break;
  }
  currentMode = m;
}

void releaseAll() {
  for (uint8_t k = K1; k <= K7; k++) driveRelay(k, false);
  driveRelay(K8, usbOn);             // the USB is not a load: leave it as it was
  currentMode = M_FGEN;
}

void setUsb(bool on) {
  usbOn = on;
  driveRelay(K8, on);
}

// ---------------------------------------------------------------------------
// LED watcher (AS7343)
//
// Runs continuously in 6-channel SMUX mode -- one integration cycle gives FZ
// (450 nm), FY (555 nm) and FXL (600 nm), which is all three SK6812 colours --
// polled without blocking, so relay commands are never held up by it.  A flash
// is "intensity (fz+fy+fxl above the dark baseline) over thr"; while one is in
// progress its samples are averaged, and the summary goes out when it ends.
// Colour decisions are the host's job: it calibrates against known alerts.
//
// Raw register access for the sample loop, as in AS7343_TFT_Controller: the
// library's readAllChannels() re-programs the chip every call and is far too
// slow for 20 ms flashes.  Reading ASTATUS latches the data registers, so
// ASTATUS + the six channels are one burst read.

Adafruit_AS7343 as7343;
TwoWire *asWire = &LED_WIRE;

#define AS_ADDR        0x39
#define AS_REG_STATUS2 0x90
#define AS_REG_ASTATUS 0x94
#define AS_REG_CFG0    0xBF
#define AS_BIT_AVALID  0x40

bool     ledSensor   = false;   // AS7343 found at boot
bool     ledOn       = false;   // flash reporting enabled
uint16_t ledRawMs    = 0;       // raw stream period (0 = off)
uint8_t  ledGain     = AS7343_GAIN_16X;
uint8_t  ledAtime    = 15;
uint16_t ledAstep    = 256;     // (ATIME+1)(ASTEP+1) x 2.78 us = 11.4 ms, full scale 4112
float    ledThr      = 20.0f;   // counts above baseline that make a flash

float    baseCh[3]   = {0, 0, 0};  // dark baseline per colour channel (EMA)
float    baseVis     = 0;
bool     baseSeeded  = false;
bool     inFlash     = false;
unsigned long flashStartMs = 0, lastSampleMs = 0, lastRawMs = 0;
uint16_t flashN      = 0;
float    flashSum[4] = {0, 0, 0, 0};
float    flashPeak   = 0;
bool     flashSat    = false;
uint32_t sampleCount = 0;
unsigned long rateT0 = 0;
float    sampleHz    = 0;

static uint32_t ledFullScale() {
  uint32_t fs = (uint32_t)(ledAtime + 1) * (ledAstep + 1);
  return fs > 65535 ? 65535 : fs;
}

static bool asRead(uint8_t reg, uint8_t *buf, size_t len) {
  asWire->beginTransmission(AS_ADDR);
  asWire->write(reg);
  if (asWire->endTransmission(false) != 0) return false;
  if (asWire->requestFrom((uint8_t)AS_ADDR, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = asWire->read();
  return true;
}

static uint8_t asRead8(uint8_t reg) {
  uint8_t v = 0;
  asRead(reg, &v, 1);
  return v;
}

bool ledApplyConfig() {
  if (!ledSensor) return false;
  bool ok = as7343.setGain((as7343_gain_t)ledGain) &&
            as7343.setATIME(ledAtime) &&
            as7343.setASTEP(ledAstep) &&
            as7343.setSMUXMode(AS7343_SMUX_6CH) &&
            as7343.startMeasurement();
  // Registers 0x80+ are only visible with REG_BANK=0 (CFG0 bit 4).
  uint8_t cfg0 = asRead8(AS_REG_CFG0);
  if (cfg0 & 0x10) {
    asWire->beginTransmission(AS_ADDR);
    asWire->write(AS_REG_CFG0);
    asWire->write(cfg0 & ~0x10);
    asWire->endTransmission();
  }
  baseSeeded = false;           // new gain/timing = new counts scale
  inFlash    = false;
  return ok;
}

void ledReport() {
  Serial.print(F("$LEDSTATE,sensor=")); Serial.print(ledSensor ? 1 : 0);
  Serial.print(F(",on="));      Serial.print(ledOn ? 1 : 0);
  Serial.print(F(",gain="));    Serial.print(ledGain);
  Serial.print(F(",atime="));   Serial.print(ledAtime);
  Serial.print(F(",astep="));   Serial.print(ledAstep);
  Serial.print(F(",thr="));     Serial.print(ledThr, 1);
  Serial.print(F(",hz="));      Serial.print(sampleHz, 1);
  Serial.print(F(",base="));    Serial.print(baseCh[0] + baseCh[1] + baseCh[2], 1);
  Serial.print(F(",fullscale=")); Serial.println(ledFullScale());
}

void ledEndFlash() {
  inFlash = false;
  if (!ledOn || flashN == 0) return;
  Serial.print(F("$FLASH,"));
  Serial.print(flashStartMs);                 Serial.print(',');
  Serial.print(lastSampleMs - flashStartMs);  Serial.print(',');
  Serial.print(flashN);                       Serial.print(',');
  for (uint8_t i = 0; i < 4; i++) { Serial.print(flashSum[i] / flashN, 1); Serial.print(','); }
  Serial.print(flashPeak, 1);                 Serial.print(',');
  Serial.println(flashSat ? 1 : 0);
}

void ledService() {
  if (!ledSensor) return;
  if (!(asRead8(AS_REG_STATUS2) & AS_BIT_AVALID)) return;
  uint8_t buf[13];
  if (!asRead(AS_REG_ASTATUS, buf, sizeof(buf))) return;
  unsigned long now = millis();
  uint16_t ch[6];
  for (uint8_t i = 0; i < 6; i++) ch[i] = buf[1 + 2 * i] | (buf[2 + 2 * i] << 8);
  // 6-channel order: FZ, FY, FXL, NIR, VIS, FD

  sampleCount++;
  if (now - rateT0 >= 1000) {
    sampleHz = sampleCount * 1000.0f / (now - rateT0);
    sampleCount = 0;
    rateT0 = now;
  }
  if (ledRawMs && now - lastRawMs >= ledRawMs) {
    lastRawMs = now;
    Serial.print(F("$LEDRAW,")); Serial.print(now);
    for (uint8_t i = 0; i < 5; i++) { Serial.print(','); Serial.print(ch[i]); }
    Serial.println();
  }

  float c[3] = {(float)ch[0], (float)ch[1], (float)ch[2]};
  if (!baseSeeded) {
    for (uint8_t i = 0; i < 3; i++) baseCh[i] = c[i];
    baseVis = ch[4];
    baseSeeded = true;
    return;
  }
  float d[3], inten = 0;
  for (uint8_t i = 0; i < 3; i++) { d[i] = c[i] - baseCh[i]; inten += d[i]; }
  bool sat = false;
  uint32_t clip = ledFullScale() * 98UL / 100UL;
  for (uint8_t i = 0; i < 5; i++) if (ch[i] >= clip) sat = true;

  if (!inFlash) {
    if (inten > ledThr) {
      inFlash = true;
      flashStartMs = now;
      flashN = 0;
      for (uint8_t i = 0; i < 4; i++) flashSum[i] = 0;
      flashPeak = 0;
      flashSat = false;
    } else {
      // Track the dark level only while dark, slowly, so ambient drift and the
      // sensor's own offset come out without a flash dragging it up.
      for (uint8_t i = 0; i < 3; i++) baseCh[i] += (c[i] - baseCh[i]) / 32.0f;
      baseVis += (ch[4] - baseVis) / 32.0f;
      return;
    }
  } else if (inten < ledThr * 0.5f || now - flashStartMs > 5000) {
    // Hysteresis on the way out; and a solid-on LED is cut into 5 s pieces so
    // the host still hears about it.
    ledEndFlash();
    return;
  }
  flashN++;
  for (uint8_t i = 0; i < 3; i++) flashSum[i] += d[i];
  flashSum[3] += ch[4] - baseVis;
  if (inten > flashPeak) flashPeak = inten;
  if (sat) flashSat = true;
  lastSampleMs = now;
}

// ---------------------------------------------------------------------------
// Reporting

void reportState() {
  Serial.print(F("$STATE,"));
  Serial.print(MODE_NAME[currentMode]);
  Serial.print(',');
  for (uint8_t k = 0; k < NUM_RELAYS; k++) Serial.print(relayOn[k] ? '1' : '0');
  Serial.print(',');
  Serial.print(resolveLoad());
  Serial.print(',');
  Serial.print(settleMs);
  Serial.print(',');
  Serial.println(usbOn ? 1 : 0);     // appended: older hosts stop at settle
}

void printHelp() {
  Serial.println(F("RelayJig commands:"));
  Serial.println(F("  !MODE,<FGEN|SHORT|OPEN|1M|10K|10.6M|150K>"));
  Serial.println(F("  !K,<1-8>,<0|1>   raw relay drive"));
  Serial.println(F("  !ALL             release K1-K7 (FGEN); USB untouched"));
  Serial.println(F("  !USB[,0|1]       BlinkyHawk USB via K8: 1 connect, 0 disconnect"));
  Serial.println(F("  !SETTLE,<ms>     settle time per relay step"));
  Serial.println(F("  !STATE  !ID  !HELP"));
  Serial.println(F("  !LED[,0|1]  !LEDRAW[,ms]  !LEDCFG[,gain,atime,astep,thr]  !LEDSTAT"));
}

// ---------------------------------------------------------------------------
// Command parsing

char lineBuf[64];
uint8_t lineLen = 0;

void handleCommand(char *cmd) {
  // Upper-case in place so "!mode,10k" works.
  for (char *p = cmd; *p; p++) *p = toupper(*p);

  char *arg = strchr(cmd, ',');
  if (arg) *arg++ = '\0';

  if (!strcmp(cmd, "!MODE")) {
    if (!arg) { Serial.println(F("$ERR,MODE needs an argument")); return; }
    for (uint8_t i = 0; i < NUM_NAMED_MODES; i++) {
      if (!strcmp(arg, MODE_NAME[i])) {
        Serial.print(F("Switching to ")); Serial.println(MODE_NAME[i]);
        setMode((Mode)i);
        reportState();
        return;
      }
    }
    Serial.print(F("$ERR,unknown mode ")); Serial.println(arg);
  } else if (!strcmp(cmd, "!K")) {
    char *val = arg ? strchr(arg, ',') : nullptr;
    if (!val) { Serial.println(F("$ERR,usage !K,<1-8>,<0|1>")); return; }
    *val++ = '\0';
    int n = atoi(arg);
    if (n < 1 || n > NUM_RELAYS || (*val != '0' && *val != '1')) {
      Serial.println(F("$ERR,usage !K,<1-8>,<0|1>"));
      return;
    }
    if (n - 1 == K8) {
      // K8 is the USB, not part of the load network: keep it out of RAW mode
      // and keep usbOn honest.
      setUsb(*val == '1');
      Serial.print(F("$USB,")); Serial.println(usbOn ? 1 : 0);
      reportState();
      return;
    }
    driveRelay(n - 1, *val == '1');
    currentMode = M_RAW;
    reportState();
  } else if (!strcmp(cmd, "!USB")) {
    if (arg) {
      if (*arg != '0' && *arg != '1') { Serial.println(F("$ERR,usage !USB[,0|1]")); return; }
      setUsb(*arg == '1');
    }
    Serial.print(F("$USB,")); Serial.println(usbOn ? 1 : 0);
  } else if (!strcmp(cmd, "!ALL")) {
    releaseAll();
    reportState();
  } else if (!strcmp(cmd, "!SETTLE")) {
    long ms = arg ? atol(arg) : -1;
    if (ms < 0 || ms > 2000) { Serial.println(F("$ERR,settle 0-2000 ms")); return; }
    settleMs = ms;
    reportState();
  } else if (!strcmp(cmd, "!STATE")) {
    reportState();
  } else if (!strcmp(cmd, "!LED")) {
    if (arg) ledOn = atoi(arg) != 0;
    if (ledOn && !ledSensor) Serial.println(F("$ERR,no AS7343 found"));
    inFlash = false;
    ledReport();
  } else if (!strcmp(cmd, "!LEDRAW")) {
    long ms = arg ? atol(arg) : 0;
    ledRawMs = (ms < 0) ? 0 : (ms > 60000 ? 60000 : ms);
    ledReport();
  } else if (!strcmp(cmd, "!LEDCFG")) {
    if (arg) {
      // gain,atime,astep,thr -- all four, so a typo cannot half-apply
      long g = -1, a = -1, st = -1;
      float t = -1;
      char *p1 = arg, *p2 = strchr(p1, ',');
      char *p3 = p2 ? strchr(p2 + 1, ',') : nullptr;
      char *p4 = p3 ? strchr(p3 + 1, ',') : nullptr;
      if (p2 && p3 && p4) {
        g = atol(p1); a = atol(p2 + 1); st = atol(p3 + 1); t = atof(p4 + 1);
      }
      if (g < 0 || g > 12 || a < 0 || a > 255 || st < 0 || st > 65534 || t <= 0) {
        Serial.println(F("$ERR,usage !LEDCFG,<gain 0-12>,<atime 0-255>,<astep 0-65534>,<thr>"));
        return;
      }
      ledGain = g; ledAtime = a; ledAstep = st; ledThr = t;
      if (ledSensor && !ledApplyConfig()) Serial.println(F("$ERR,AS7343 config failed"));
    }
    ledReport();
  } else if (!strcmp(cmd, "!LEDSTAT")) {
    ledReport();
  } else if (!strcmp(cmd, "!ID")) {
    Serial.println(F("$ID,RelayJig," FW_VERSION));
  } else if (!strcmp(cmd, "!HELP")) {
    printHelp();
  } else {
    Serial.print(F("$ERR,unknown command ")); Serial.println(cmd);
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen && lineBuf[0] == '!') handleCommand(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                       // overlong line: drop it
      Serial.println(F("$ERR,line too long"));
    }
  }
}

// ---------------------------------------------------------------------------

void setup() {
  // Write the released level before and after pinMode: on the RA4M1 core
  // pinMode(OUTPUT) can briefly drive LOW, which would click an active-low
  // board.  A few microseconds is far too short to pull a relay in.
  for (uint8_t k = 0; k < NUM_RELAYS; k++) {
    bool on = (k == K8) ? usbOn : false;   // K8 comes up as USB_BOOT_CONNECTED
    driveRelay(k, on);
    pinMode(RELAY_PIN[k], OUTPUT);
    driveRelay(k, on);
  }

  Serial.begin(115200);
  delay(200);
  Serial.println(F("$BOOT,RelayJig," FW_VERSION));
  reportState();

  LED_WIRE.begin();
  LED_WIRE.setClock(400000);
  ledSensor = as7343.begin(AS7343_I2CADDR_DEFAULT, &LED_WIRE);
  if (ledSensor) ledSensor = ledApplyConfig();
  ledOn = ledSensor;
  rateT0 = millis();
  ledReport();
}

void loop() {
  pollSerial();
  ledService();
}
