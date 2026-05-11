/*
  WirelessGigaLogger.ino  —  Arduino Giga R1 WiFi
  BLE-controlled multi-channel ADC datalogger.

  Channels   : A0–A7 (up to 8, individually configured)
  Bluetooth  : ArduinoBLE — custom GATT service (BLE, not classic BT)
  Storage    : Optional USB stick via Arduino_USBHostMbed5 (mbed::FATFileSystem)

  Required libraries (install via Arduino Library Manager):
    ArduinoBLE
    Arduino_USBHostMbed5

  ── Commands  (Python GUI → Giga, written to CMD characteristic) ─────────────
  !TIME,<epoch_ms>             Set software wall-clock (ms since Unix epoch)
  !STATUS                      Send full status reply
  !CH,<n>,<en>,<off>,<sc>,<u> Configure channel n (0-7):
                                 en=0/1, offset (float), scale (float), unit V or I
  !ACTIVE,<mask8>              Enable channels by 8-bit bitmask (bit0=A0…bit7=A7)
  !SMOOTH,<n>                  Smoothing window size 1–64 (default 7)
  !THRES,<n>,<val>             Trigger threshold for channel n (scaled units)
  !TRIG_MODE                   Switch to trigger-on-threshold mode
  !BURST_MODE                  Switch to burst-capture mode
  !CONT_MODE                   Switch to continuous 1 Hz USB log mode
  !BURST_MS,<ms>               Burst capture duration ms (default 1000)
  !COMPACT,<0/1>               0 = transmit burst as 16-bit counts (default)
                                1 = transmit burst as 12-bit counts (right-shift 4, smaller packets)
                               Note: capture is always 16-bit via hardware oversampling.
                               Python applies offset/scale after receipt in both modes.
  !TRIG_HOLD,<ms>              Hold time in ms: how long to wait after the last
                               qualifying sample before transmitting (default 5)
  !START                       Start logging (or fire a burst)
  !STOP                        Stop logging / abort burst TX
  !MARK                        Flag next logged sample with MARK
  !USB,<0/1>                   Enable / disable USB stick logging
  !DLFILE                      Download current session CSV over BLE

  ── Replies   (Giga → Python, notified on STREAM characteristic) ─────────────
  $STATUS,<mode>,<mask>,<logActive>,<smoothN>,<burstMs>,<usbReady>,<usbLog>,<logs>
  $CHCFG,<n>,<en>,<offset>,<scale>,<unit>,<thres>
  $LOG,<epoch_ms>,<ch0>,...          Live preview every 500 ms (active channels)
  $TRIG,<epoch_ms>,<ch0>,...[,MARK]  Triggered sample
  $BURST_STARTED                     Burst capture has begun
  $BURST_START,<n>,<mask>,<smoothN>,<bits>,<epoch_ms>  All samples captured; BLE TX beginning
                                                       bits = 12 or 16
  $BD,<idx>,<t_rel_us>,<ch0>,...     One burst sample (raw ADC counts, scale on Python side)
                                     t_rel_us is microseconds from capture start
  $BURST_END,<n>                     Burst TX complete
  $FILE_START,<name>,<bytes>         File download starting
  $FC,<seq>,<total>,<csv_text>       File chunk (raw CSV bytes, may contain \n)
  $FILE_END,<name>                   File download complete
  $ACK,<cmd>                         Command acknowledged
  $ERR,<msg>                         Error reply

  Mode values in $STATUS: 0=TRIG, 1=BURST, 2=CONT
*/

#include <ArduinoBLE.h>
#include <Arduino_USBHostMbed5.h>
// FATFileSystem is part of the mbed namespace included via Arduino_USBHostMbed5

// ─── BLE UUIDs ────────────────────────────────────────────────────────────────
#define SVC_UUID    "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CMD_UUID    "19b10001-e8f2-537e-4f6c-d104768a1214"
#define STREAM_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEService        loggerSvc(SVC_UUID);
BLECharacteristic cmdChar(CMD_UUID,    BLEWrite | BLEWriteWithoutResponse, 200);
BLECharacteristic streamChar(STREAM_UUID, BLERead | BLENotify, 200);

// ─── Channel config ───────────────────────────────────────────────────────────
struct ChCfg {
  bool  active;
  float offset;
  float scale;
  char  unit;       // 'V' or 'I'
  float threshold;
};

ChCfg ch[8] = {
  {true,  0.0f, 1.0f, 'V', 1.0f},  // A0
  {false, 0.0f, 1.0f, 'V', 1.0f},  // A1
  {false, 0.0f, 1.0f, 'V', 1.0f},  // A2
  {false, 0.0f, 1.0f, 'V', 1.0f},  // A3
  {false, 0.0f, 1.0f, 'V', 1.0f},  // A4
  {false, 0.0f, 1.0f, 'V', 1.0f},  // A5
  {false, 0.0f, 1.0f, 'V', 1.0f},  // A6
  {false, 0.0f, 1.0f, 'V', 1.0f},  // A7
};
const int adcPin[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };

// ─── Smoothing — rolling mean of smoothN raw reads ───────────────────────────
const int MAX_SMOOTH = 64;
int   smoothN = 7;
float sBuf[8][MAX_SMOOTH];
int   sIdx[8];
int   sFill[8];

void smoothAdd(int c, float v) {
  sBuf[c][sIdx[c]] = v;
  sIdx[c] = (sIdx[c] + 1) % smoothN;
  if (sFill[c] < smoothN) sFill[c]++;
}

float smoothGet(int c) {
  if (!sFill[c]) return 0.0f;
  float s = 0.0f;
  for (int i = 0; i < sFill[c]; i++) s += sBuf[c][i];
  return s / (float)sFill[c];
}

void smoothReset() {
  memset(sIdx,  0, sizeof(sIdx));
  memset(sFill, 0, sizeof(sFill));
}

// ─── ADC ──────────────────────────────────────────────────────────────────────
// analogReadResolution(16) enables hardware oversampling on STM32H747 for genuine 16-bit.
// rawADC[] holds the last 16-bit count per channel for the burst buffer.
float    chVal[8];
uint16_t rawADC[8];

void readChannels() {
  for (int c = 0; c < 8; c++) {
    if (!ch[c].active) { chVal[c] = 0.0f; rawADC[c] = 0; continue; }
    uint16_t r = (uint16_t)analogRead(adcPin[c]);
    rawADC[c] = r;
    float rawV = r * (3.3f / 65535.0f);
    smoothAdd(c, rawV);
    chVal[c] = (smoothGet(c) + ch[c].offset) * ch[c].scale;
  }
}

// ─── Burst buffer  (~70 KB static — 3500 × (4 + 8×2) bytes) ─────────────────
// Stores raw 16-bit ADC counts; offset/scale applied on Python side.
const int MAX_BURST = 3500;
struct BurstSample { uint32_t t_rel_us; uint16_t raw[8]; };
static BurstSample bBuf[MAX_BURST];
int      bCount          = 0;
uint32_t burstMs         = 1000;
uint32_t burstStart      = 0;
bool     bursting        = false;
int64_t  burstEpochStart = 0;   // epochMs() at capture start; sent in $BURST_START

// Burst BLE TX — non-blocking, rate-limited to ~100 packets/s
bool     burstTxing    = false;
int      burstTxIdx    = 0;
uint32_t lastBurstTxAt = 0;
const int BURST_TX_PER_TICK   = 3;    // packets per tick
const int BURST_TX_INTERVAL_MS = 10; // ms between ticks

// ─── Trigger capture state ────────────────────────────────────────────────────
// Delta-logger with hold timer.  On !START the first sample is written as the
// reference (bBuf[0]).  Any sample that deviates from the reference by ≥ threshold
// is added and arms a hold timer.  While armed, each further deviation from the
// last buffer value resets the timer.  When the timer expires the buffer is sent
// using the same $BURST_START/$BD/$BURST_END protocol as burst mode.
bool     trigFirstSample = true;   // true → set reference on next handleTrig() call
bool     trigArmed       = false;  // true → hold timer is running
uint32_t trigHoldMs      = 5;      // hold time in ms before transmitting
uint32_t trigTimerStart  = 0;
uint32_t trigT0          = 0;      // millis() when current bBuf started
float    trigRef[8]      = {};     // reference value (set at start / after TX)
float    trigLastBuf[8]  = {};     // last value written to bBuf (ARMED comparison)

// ─── Logging state ────────────────────────────────────────────────────────────
bool compactTx = false;   // true = burst TX uses 12-bit (right-shift 4), false = 16-bit
enum Mode { TRIG, BURST, CONT };
Mode logMode   = TRIG;
bool logActive = false;
bool pendMark  = false;
int  logCount  = 0;
float lastLogged[8] = {};

// ─── Software wall-clock ──────────────────────────────────────────────────────
int64_t clockOff = 0;
bool    clockSet = false;

int64_t epochMs() { return (int64_t)millis() + clockOff; }

// ─── USB mass storage ─────────────────────────────────────────────────────────
USBHostMSD           msd;
mbed::FATFileSystem  usbFS("usb");
bool usbReady   = false;
bool usbLogging = false;
bool usbHdrDone = false;
char usbFile[56] = "gigarec_0.csv";

bool tryMountUSB() {
  if (!msd.connect()) return false;
  return usbFS.mount(&msd) == 0;
}

void writeUSBLine(const char* line) {
  char path[72];
  snprintf(path, sizeof(path), "/usb/%s", usbFile);
  FILE* f = fopen(path, "a");
  if (!f) { usbReady = false; return; }
  if (!usbHdrDone) {
    fprintf(f, "epoch_ms");
    for (int c = 0; c < 8; c++) if (ch[c].active) fprintf(f, ",ch%d_%c", c, ch[c].unit);
    fprintf(f, "\n");
    usbHdrDone = true;
  }
  fprintf(f, "%s\n", line);
  fclose(f);
}

void writeBurstToUSB() {
  char path[72];
  snprintf(path, sizeof(path), "/usb/%s", usbFile);
  FILE* f = fopen(path, "a");
  if (!f) { usbReady = false; return; }
  if (!usbHdrDone) {
    fprintf(f, "t_rel_ms");
    for (int c = 0; c < 8; c++) if (ch[c].active) fprintf(f, ",ch%d_%c", c, ch[c].unit);
    fprintf(f, "\n");
    usbHdrDone = true;
  }
  // Write physical values (offset+scale applied) so USB CSV is immediately usable
  const float rawScale = 3.3f / 65535.0f;
  for (int i = 0; i < bCount; i++) {
    fprintf(f, "%lu", (unsigned long)bBuf[i].t_rel_us);
    for (int c = 0; c < 8; c++) {
      if (!ch[c].active) continue;
      float phys = (bBuf[i].raw[c] * rawScale + ch[c].offset) * ch[c].scale;
      fprintf(f, ",%.5f", phys);
    }
    fprintf(f, "\n");
  }
  fclose(f);
}

// ─── File download state ──────────────────────────────────────────────────────
bool  dlActive = false;
FILE* dlFP     = nullptr;
int   dlSeq    = 0;
int   dlTotal  = 0;
long  dlSize   = 0;
uint32_t lastDlAt = 0;
const int DL_CHUNK       = 150;   // CSV bytes per BLE packet
const int DL_INTERVAL_MS =  15;  // ms between chunks

// ─── BLE send helper ──────────────────────────────────────────────────────────
void bleSend(const char* s) {
  if (streamChar.subscribed())
    streamChar.writeValue((const uint8_t*)s, strlen(s));
}

// ─── Status reply ─────────────────────────────────────────────────────────────
void sendStatus() {
  uint8_t mask = 0;
  for (int c = 0; c < 8; c++) if (ch[c].active) mask |= (1 << c);

  char buf[200];
  snprintf(buf, sizeof(buf),
    "$STATUS,%d,%d,%d,%d,%lu,%d,%d,%d",
    (int)logMode, mask, logActive ? 1 : 0, smoothN,
    (unsigned long)burstMs,
    usbReady ? 1 : 0, usbLogging ? 1 : 0, logCount);
  bleSend(buf);

  for (int c = 0; c < 8; c++) {
    snprintf(buf, sizeof(buf), "$CHCFG,%d,%d,%.5f,%.5f,%c,%.5f",
      c, ch[c].active ? 1 : 0,
      ch[c].offset, ch[c].scale, ch[c].unit, ch[c].threshold);
    bleSend(buf);
    // No delay — delay() here blocks BLE.poll() and causes commands to be dropped.
    // The BLE stack queues outgoing notifications internally.
  }
}

// ─── Live preview ($LOG every 500 ms, regardless of mode) ────────────────────
uint32_t lastLiveAt = 0;

void sendLive() {
  char buf[200];
  int pos = snprintf(buf, sizeof(buf), "$LOG,%lld", (long long)epochMs());
  for (int c = 0; c < 8; c++) {
    if (!ch[c].active) continue;
    pos += snprintf(buf + pos, sizeof(buf) - pos, ",%.5f", chVal[c]);
  }
  bleSend(buf);
}

// ─── Trigger mode ─────────────────────────────────────────────────────────────
void handleTrig() {
  if (burstTxing) return;   // TX in progress; trigFirstSample=true re-arms on completion

  // ── INIT: set reference from current reading, write it as bBuf[0] ─────────
  if (trigFirstSample) {
    trigFirstSample = false;
    trigArmed       = false;
    trigT0          = micros();
    burstEpochStart = epochMs();
    bCount          = 0;
    for (int c = 0; c < 8; c++) {
      trigRef[c]     = chVal[c];
      trigLastBuf[c] = chVal[c];
    }
    bBuf[0].t_rel_us = 0;
    for (int c = 0; c < 8; c++) bBuf[0].raw[c] = rawADC[c];
    bCount = 1;
    return;
  }

  // ── IDLE: watch for first deviation from reference ────────────────────────
  if (!trigArmed) {
    bool fire = false;
    for (int c = 0; c < 8; c++) {
      if (!ch[c].active) continue;
      if (fabsf(chVal[c] - trigRef[c]) >= ch[c].threshold) { fire = true; break; }
    }
    if (!fire) return;

    // Deviation found — add sample, arm hold timer
    if (bCount < MAX_BURST) {
      bBuf[bCount].t_rel_us = micros() - trigT0;
      for (int c = 0; c < 8; c++) {
        bBuf[bCount].raw[c] = rawADC[c];
        trigLastBuf[c]      = chVal[c];
      }
      bCount++;
    }
    trigArmed      = true;
    trigTimerStart = millis();   // hold timer stays in ms
    return;
  }

  // ── ARMED: check for further deviation from last buffer value ─────────────
  bool fire = false;
  for (int c = 0; c < 8; c++) {
    if (!ch[c].active) continue;
    if (fabsf(chVal[c] - trigLastBuf[c]) >= ch[c].threshold) { fire = true; break; }
  }
  if (fire) {
    if (bCount < MAX_BURST) {
      bBuf[bCount].t_rel_us = micros() - trigT0;
      for (int c = 0; c < 8; c++) {
        bBuf[bCount].raw[c] = rawADC[c];
        trigLastBuf[c]      = chVal[c];
      }
      bCount++;
    }
    trigTimerStart = millis();   // restart hold timer on each new deviation
    return;
  }

  // No further deviation — wait for hold timer
  if (millis() - trigTimerStart < trigHoldMs) return;

  // ── TRANSMIT ─────────────────────────────────────────────────────────────
  trigArmed       = false;
  trigFirstSample = true;   // re-arm with new reference after TX completes

  uint8_t mask = 0;
  for (int c = 0; c < 8; c++) if (ch[c].active) mask |= (1 << c);
  char hdr[80];
  snprintf(hdr, sizeof(hdr), "$BURST_START,%d,%d,%d,%d,%lld",
           bCount, mask, smoothN, compactTx ? 12 : 16, (long long)burstEpochStart);
  bleSend(hdr);
  burstTxIdx    = 0;
  burstTxing    = true;
  lastBurstTxAt = millis();
  logCount++;
}

// ─── Burst capture ────────────────────────────────────────────────────────────
void startBurst() {
  bCount          = 0;
  burstStart      = micros();
  bursting        = true;
  burstTxing      = false;
  burstEpochStart = epochMs();
  bleSend("$BURST_STARTED");
}

void captureBurst() {
  uint32_t el = micros() - burstStart;
  if (el >= burstMs * 1000UL || bCount >= MAX_BURST) {
    // Capture complete — begin BLE TX
    bursting  = false;
    logActive = false;

    uint8_t mask = 0;
    for (int c = 0; c < 8; c++) if (ch[c].active) mask |= (1 << c);
    char hdr[80];
    snprintf(hdr, sizeof(hdr), "$BURST_START,%d,%d,%d,%d,%lld",
             bCount, mask, smoothN, compactTx ? 12 : 16, (long long)burstEpochStart);
    bleSend(hdr);

    burstTxIdx    = 0;
    burstTxing    = true;
    lastBurstTxAt = millis();
    return;
  }
  if (bCount < MAX_BURST) {
    bBuf[bCount].t_rel_us = el;
    for (int c = 0; c < 8; c++) bBuf[bCount].raw[c] = rawADC[c];
    bCount++;
  }
}

// ─── Burst BLE TX (non-blocking) ──────────────────────────────────────────────
void txBurst() {
  if (!burstTxing) return;
  uint32_t now = millis();
  if (now - lastBurstTxAt < (uint32_t)BURST_TX_INTERVAL_MS) return;
  lastBurstTxAt = now;

  int shift = compactTx ? 4 : 0;   // 12-bit: right-shift 4; 16-bit: no shift
  for (int i = 0; i < BURST_TX_PER_TICK && burstTxIdx < bCount; i++, burstTxIdx++) {
    char buf[200];
    int pos = snprintf(buf, sizeof(buf), "$BD,%d,%lu",
      burstTxIdx, (unsigned long)bBuf[burstTxIdx].t_rel_us);
    for (int c = 0; c < 8; c++) {
      if (!ch[c].active) continue;
      pos += snprintf(buf + pos, sizeof(buf) - pos, ",%u", bBuf[burstTxIdx].raw[c] >> shift);
    }
    bleSend(buf);
  }

  if (burstTxIdx >= bCount) {
    char foot[40];
    snprintf(foot, sizeof(foot), "$BURST_END,%d", bCount);
    bleSend(foot);
    burstTxing = false;
    if (usbLogging && usbReady) writeBurstToUSB();
  }
}

// ─── Continuous mode  (USB at 1 Hz, BLE via the 500 ms $LOG) ─────────────────
uint32_t lastContAt = 0;

void handleCont() {
  uint32_t now = millis();
  if (now - lastContAt < 1000) return;
  lastContAt = now;

  char buf[200];
  int pos = snprintf(buf, sizeof(buf), "%lld", (long long)epochMs());
  for (int c = 0; c < 8; c++) {
    if (!ch[c].active) continue;
    pos += snprintf(buf + pos, sizeof(buf) - pos, ",%.5f", chVal[c]);
  }
  if (pendMark) {
    strncat(buf, ",MARK", sizeof(buf) - strlen(buf) - 1);
    pendMark = false;
  }
  if (usbLogging && usbReady) writeUSBLine(buf);
  logCount++;
}

// ─── File download (non-blocking) ────────────────────────────────────────────
void startDL() {
  if (!usbReady) { bleSend("$ERR,NO_USB"); return; }
  char path[72];
  snprintf(path, sizeof(path), "/usb/%s", usbFile);
  dlFP = fopen(path, "r");
  if (!dlFP) { bleSend("$ERR,NO_FILE"); return; }
  fseek(dlFP, 0, SEEK_END);
  dlSize  = ftell(dlFP);
  fseek(dlFP, 0, SEEK_SET);
  dlTotal = (int)((dlSize + DL_CHUNK - 1) / DL_CHUNK);
  dlSeq   = 0;
  char hdr[80];
  snprintf(hdr, sizeof(hdr), "$FILE_START,%s,%ld", usbFile, dlSize);
  bleSend(hdr);
  dlActive  = true;
  lastDlAt  = millis();
}

void continueDL() {
  if (!dlActive || !dlFP) return;
  uint32_t now = millis();
  if (now - lastDlAt < (uint32_t)DL_INTERVAL_MS) return;
  lastDlAt = now;

  char data[DL_CHUNK + 1];
  int n = (int)fread(data, 1, DL_CHUNK, dlFP);
  if (n <= 0) {
    char foot[80];
    snprintf(foot, sizeof(foot), "$FILE_END,%s", usbFile);
    bleSend(foot);
    fclose(dlFP);
    dlFP     = nullptr;
    dlActive = false;
    return;
  }
  // Prefix + raw CSV bytes in one packet
  char pkt[200];
  int plen = snprintf(pkt, sizeof(pkt), "$FC,%d,%d,", dlSeq, dlTotal);
  int space = (int)sizeof(pkt) - plen - 1;
  if (n > space) n = space;
  memcpy(pkt + plen, data, n);
  pkt[plen + n] = '\0';
  bleSend(pkt);
  dlSeq++;
}

// ─── Command handler ──────────────────────────────────────────────────────────
void handleCmd(const char* raw) {
  String cmd = String(raw);
  cmd.trim();
  if (cmd.length() == 0 || cmd.charAt(0) != '!') return;

  // ── !STATUS ──
  if (cmd == F("!STATUS")) { sendStatus(); return; }

  // ── !TIME,<epoch_ms> ──
  if (cmd.startsWith(F("!TIME,"))) {
    double ep = cmd.substring(6).toDouble();
    clockOff = (int64_t)ep - (int64_t)millis();
    clockSet = true;
    // Name USB file using epoch seconds for easy identification
    long epSec = (long)(ep / 1000.0);
    snprintf(usbFile, sizeof(usbFile), "log_%ld.csv", epSec);
    usbHdrDone = false;
    bleSend("$ACK,TIME");
    return;
  }

  // ── !CH,<n>,<en>,<off>,<scale>,<unit> ──
  if (cmd.startsWith(F("!CH,"))) {
    int a = cmd.indexOf(',', 4);
    int b = cmd.indexOf(',', a + 1);
    int c2 = cmd.indexOf(',', b + 1);
    int d = cmd.indexOf(',', c2 + 1);
    if (d < 0 || d + 1 >= (int)cmd.length()) { bleSend("$ERR,CH_PARSE"); return; }
    int n = cmd.substring(4, a).toInt();
    if (n < 0 || n > 7) { bleSend("$ERR,CH_RANGE"); return; }
    ch[n].active    = (bool)cmd.substring(a + 1, b).toInt();
    ch[n].offset    = cmd.substring(b + 1, c2).toFloat();
    ch[n].scale     = cmd.substring(c2 + 1, d).toFloat();
    char u = cmd.charAt(d + 1);
    ch[n].unit      = (u == 'I') ? 'I' : 'V';
    bleSend("$ACK,CH");
    // No sendStatus() here — rapid multi-channel config sends would pile up
    // and cause the next command to be dropped. Python requests !STATUS when done.
    return;
  }

  // ── !ACTIVE,<mask8> ──
  if (cmd.startsWith(F("!ACTIVE,"))) {
    int mask = cmd.substring(8).toInt();
    for (int c = 0; c < 8; c++) ch[c].active = (mask >> c) & 1;
    smoothReset();
    bleSend("$ACK,ACTIVE");
    return;
  }

  // ── !SMOOTH,<n> ──
  if (cmd.startsWith(F("!SMOOTH,"))) {
    smoothN = constrain(cmd.substring(8).toInt(), 1, MAX_SMOOTH);
    smoothReset();
    bleSend("$ACK,SMOOTH");
    return;
  }

  // ── !THRES,<n>,<val> ──
  if (cmd.startsWith(F("!THRES,"))) {
    int comma = cmd.indexOf(',', 7);
    if (comma < 0) { bleSend("$ERR,THRES_PARSE"); return; }
    int n = cmd.substring(7, comma).toInt();
    if (n < 0 || n > 7) { bleSend("$ERR,THRES_RANGE"); return; }
    ch[n].threshold = cmd.substring(comma + 1).toFloat();
    bleSend("$ACK,THRES");
    return;
  }

  // ── Mode switches ──
  if (cmd == F("!TRIG_MODE"))  { logMode = TRIG;  bleSend("$ACK,TRIG_MODE");  return; }
  if (cmd == F("!BURST_MODE")) { logMode = BURST; bleSend("$ACK,BURST_MODE"); return; }
  if (cmd == F("!CONT_MODE"))  { logMode = CONT;  bleSend("$ACK,CONT_MODE");  return; }

  // ── !BURST_MS,<ms> ──
  if (cmd.startsWith(F("!BURST_MS,"))) {
    burstMs = (uint32_t)constrain(cmd.substring(10).toInt(), 10, 30000);
    bleSend("$ACK,BURST_MS");
    return;
  }

  // ── !COMPACT,<0/1> ──
  if (cmd.startsWith(F("!COMPACT,"))) {
    compactTx = (bool)cmd.substring(9).toInt();
    bleSend(compactTx ? "$ACK,COMPACT_12" : "$ACK,COMPACT_16");
    return;
  }

  // ── !TRIG_HOLD,<ms> ──
  if (cmd.startsWith(F("!TRIG_HOLD,"))) {
    trigHoldMs = (uint32_t)constrain(cmd.substring(11).toInt(), 1, 60000);
    bleSend("$ACK,TRIG_HOLD");
    return;
  }

  // ── !START ──
  if (cmd == F("!START")) {
    logActive = true;
    logCount  = 0;
    if (logMode == BURST) {
      startBurst();
    } else if (logMode == TRIG) {
      trigFirstSample = true;
      trigArmed       = false;
    }
    bleSend("$ACK,START");
    return;
  }

  // ── !STOP ──
  if (cmd == F("!STOP")) {
    logActive      = false;
    bursting       = false;
    burstTxing     = false;
    trigArmed       = false;
    trigFirstSample = true;
    if (dlActive && dlFP) { fclose(dlFP); dlFP = nullptr; }
    dlActive = false;
    bleSend("$ACK,STOP");
    return;
  }

  // ── !MARK ──
  if (cmd == F("!MARK")) { pendMark = true; bleSend("$ACK,MARK"); return; }

  // ── !USB,<0/1> ──
  if (cmd.startsWith(F("!USB,"))) {
    bool want = (bool)cmd.substring(5).toInt();
    if (want && !usbReady) usbReady = tryMountUSB();
    usbLogging = want && usbReady;
    bleSend(usbLogging ? "$ACK,USB_ON" : "$ACK,USB_OFF");
    return;
  }

  // ── !DLFILE ──
  if (cmd == F("!DLFILE")) { startDL(); return; }

  bleSend("$ERR,UNKNOWN");
}

// ─── BLE event handlers ───────────────────────────────────────────────────────
void onConnect(BLEDevice central) {
  Serial.print("BLE connected: ");
  Serial.println(central.address());
  delay(100);
  sendStatus();
}

void onDisconnect(BLEDevice central) {
  Serial.println("BLE disconnected — re-advertising");
  BLE.advertise();
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  analogReadResolution(16);   // 16-bit via hardware oversampling on STM32H747

  if (!BLE.begin()) {
    Serial.println("BLE init failed — halting");
    while (1);
  }
  BLE.setLocalName("GigaLogger");
  BLE.setAdvertisedService(loggerSvc);
  loggerSvc.addCharacteristic(cmdChar);
  loggerSvc.addCharacteristic(streamChar);
  BLE.addService(loggerSvc);
  BLE.setEventHandler(BLEConnected,    onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  BLE.advertise();
  Serial.println("GigaLogger advertising");

  // USB — non-blocking; logger works without it
  usbReady = tryMountUSB();
  Serial.println(usbReady ? "USB stick mounted" : "No USB stick at startup");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  BLE.poll();

  // Incoming BLE command
  if (cmdChar.written()) {
    char buf[201];
    int n = cmdChar.readValue(buf, 200);
    buf[n] = '\0';
    handleCmd(buf);
  }

  // ADC — read and smooth all active channels every iteration
  readChannels();

  // Mode-specific logging
  if (logActive) {
    switch (logMode) {
      case TRIG: handleTrig();              break;
      case BURST: if (bursting) captureBurst(); break;
      case CONT:  handleCont();             break;
    }
  } else if (bursting) {
    // captureBurst() can set logActive = false when burst completes
    captureBurst();
  }

  // Non-blocking burst BLE TX
  if (burstTxing) txBurst();

  // Non-blocking file download
  if (dlActive) continueDL();

  // Live preview every 500 ms
  uint32_t now = millis();
  if (now - lastLiveAt >= 500) {
    lastLiveAt = now;
    sendLive();
  }
}
