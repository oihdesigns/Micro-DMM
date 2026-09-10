// ─────────────────────────────────────────────────────────────────────────────
//  Giga_RemoteController_Dual — dual-core ADC capture / live-stream controller
//
//  Dual-core split of Giga_RemoteController_WiFi. The STM32H747's Cortex-M4
//  runs the ADC (the Giga_RemoteController_M4 sketch); this Cortex-M7 sketch
//  runs everything else — WiFi, USB serial, the touchscreen, USB-stick logging
//  — and talks to the M4 through a shared block in SRAM4. See
//  giga_dual_shared.h for the layout and the capture engine itself.
//
//  Why it matters: the single-core sketch did per-sample smoothing and
//  statistics inside a blocking while() loop, so WiFi and the display were dead
//  for the whole capture and the ADC queue could overrun whenever the socket
//  stack ran long. Here the M7 only memcpy's finished frames out of a ring, so
//  it can push a 10 000-point DATA dump over TCP without perturbing
//  acquisition, and a live stream can run indefinitely while commands keep
//  being served.
//
//  ── Flashing (both sketches, SAME flash split) ──────────────────────────────
//    1. Giga_RemoteController_M4   — Tools ▸ Target core ▸ "M4 Co-processor"
//    2. Giga_RemoteController_Dual — Tools ▸ Target core ▸ "Main Core"
//    Leave Tools ▸ Flash split at its default ("1.5MB M7 + 0.5MB M4"); both
//    sketches must agree or bootM4() jumps to the wrong address.
//
//  Set ADC_ON_M4 to 0 for a single-core build that runs the same capture engine
//  on the M7 — useful for bisecting a problem, and still non-blocking, but
//  acquisition then competes with WiFi for the core.
// ─────────────────────────────────────────────────────────────────────────────
#define ADC_ON_M4 1

#include <Arduino_AdvancedAnalog.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>
#include <WiFi.h>            // Giga R1 WiFi onboard Murata module

#include "giga_dual_shared.h"
static_assert(SHM_CORE_M7, "Compile with Tools > Target core = Main Core");

// ── WiFi configuration ─────────────────────────────────────────────────────────
// Fill in your network. The board joins your Wi-Fi and listens for the same
// line-based command protocol used over USB serial, but on a TCP socket.
#define WIFI_SSID   "MTBs & Science"
#define WIFI_PASS   "xyz"
static const uint16_t CMD_PORT = 8080;   // TCP port the GUI connects to

WiFiServer cmdServer(CMD_PORT);
static bool   wifiActive = false;
static char   wifiIPStr[20] = "no wifi";

// ── ADC ───────────────────────────────────────────────────────────────────────
static const int N_CHANNELS    = 8;
static const int MAX_LOG       = 10000;

static const char* CH_NAMES[N_CHANNELS] = {"A0","A1","A2","A3","A4","A5","A6","A7"};
// uint16_t, not int: the ADC is at most 16-bit unsigned, and halving this array
// gives back 160 KB of the M7's 512 KB AXI SRAM.
static uint16_t logData[N_CHANNELS][MAX_LOG];

// One live-stream reporting period, reduced: the headline number plus its
// spread. Declared up here rather than beside its drawing code because it
// appears in a function signature, and the .ino preprocessor emits its
// generated prototypes above the first function definition.
struct PeriodStat { uint32_t n; double mean, mn, mx, sd, last; };

// ─────────────────────────────────────────────────────────────────────────────
//  Host side of the inter-core link
// ─────────────────────────────────────────────────────────────────────────────
#if ADC_ON_M4
// The M4 sketch owns the ADC and publishes into SRAM4 at a fixed address.
static GigaShm* const shm = (GigaShm*)GIGA_SHM_BASE;
#else
// Single-core fallback: the same engine, against a block in ordinary RAM. The
// cache helpers detect that it is not in SRAM4 and become no-ops.
static GigaShm     localShm;
static GigaShm* const shm = &localShm;
AdvancedADC        adc(A0, A1, A2, A3, A4, A5, A6, A7);
static CapEngine   localEng;
#endif

static bool     engineUp    = false;   // engine answered on this boot
static uint32_t hostBootId  = 0;       // stamped once, echoed by the engine
static uint32_t hostCmdSeq  = 0;
static uint32_t ringTail    = 0;

// Drain-side bookkeeping, filled while a capture is running.
static bool hPinReq[N_CHANNELS] = {};
static int  hLogCnt             = 0;    // frames stored (uniform / trigger modes)
static int  hLogTarget          = 0;
static bool hMidMode            = false;
static int  hMidPreSize = 0, hMidPostSize = 0;
static int  hMidRingHead = 0, hMidRingFill = 0, hMidPostCount = 0;

// In the fallback build the engine only advances when we call it.
static inline void hostPumpEngine() {
#if !ADC_ON_M4
    engStep(localEng);
#endif
}

static inline void hostReadStatus() {
    shmInvalidate(&shm->e, sizeof(ShmFromEngine));
}

// A stable copy of one period's statistics. The engine writes the payload and
// then bumps statSeq, so re-reading the sequence after the copy catches the rare
// case where a new period landed mid-read and would otherwise blend two periods
// into one report.
static ShmFromEngine statSnap;

static bool hostSnapshotStats() {
    for (int tries = 0; tries < 4; tries++) {
        hostReadStatus();
        uint32_t seq = shm->e.statSeq;
        memcpy((void*)&statSnap, (const void*)&shm->e, sizeof(ShmFromEngine));
        hostReadStatus();
        if (shm->e.statSeq == seq) return true;
    }
    return false;
}

static bool engineAlive() {
    hostReadStatus();
    // The boot id must match too: a stale magic word survives in SRAM4 across a
    // reset of either core, and would otherwise read as a live engine.
    return shm->e.magic == GIGA_SHM_MAGIC && shm->e.ackBootId == hostBootId;
}

// Stamp this boot's id and hand the engine a clean set of host-side counters.
// Only the host's own region is written - never the engine's, and never the
// ring: with the BCM4 option bit set the M4 boots at reset and may already be
// running, and clearing its region would strand it.
static void hostClaimShared() {
    uint32_t id = shm->h.hostBootId + 0x9E3779B9u;   // differs from last boot
    if (id == 0) id = 1;

    shm->h.cmd          = GIGA_CMD_IDLE;
    shm->h.cmdSeq       = 0;
    shm->h.abortFlag    = 0;
    shm->h.ringTail     = 0;
    shm->h.statPeriodMs = 250;
    for (int i = 0; i < N_CHANNELS; i++) shm->h.trigThresh[i] = 0;
    for (int d = 0; d < SHM_N_DIFF; d++) shm->h.diffEn[d] = 0;
    shmClean(&shm->h, sizeof(ShmToEngine));   // counters visible first...

    hostBootId = id;
    shm->h.hostBootId = id;
    shmClean(&shm->h, sizeof(ShmToEngine));   // ...then the id that arms them
    hostCmdSeq = 0;
}

#if SHM_CORE_M7
// Everything needed to tell "M4 image never flashed" from "flashed at the wrong
// address" from "running but out of sync".
static void printEngineDiag(Stream& io) {
    // Rule out the whole "SRAM4 isn't really shared memory" class first: if the
    // M7 cannot even round-trip a word through 0x38000000, nothing else below
    // means anything.
    bool sramOk = true;
    for (uint32_t pat : {0xA5A5A5A5u, 0x5A5A5A5Au, 0x00000000u}) {
        shm->h.hostScratch = pat;
        shmClean(&shm->h, sizeof(ShmToEngine));
        shmInvalidate(&shm->h, sizeof(ShmToEngine));
        if (shm->h.hostScratch != pat) { sramOk = false; break; }
    }
    io.print(F("  SRAM4 round-trip: ")); io.println(sramOk ? F("ok") : F("FAILED"));
    io.print(F("  CM4_BINARY_START = 0x"));
    io.println((uint32_t)CM4_BINARY_START, HEX);
    io.print(F("  option bytes: BCM4="));
    io.print((FLASH->OPTSR_CUR & FLASH_OPTSR_BCM4_Msk) ? 1 : 0);
    io.print(F("  BOOT4_CUR=0x"));
    io.println((uint32_t)FLASH->BOOT4_CUR, HEX);
    hostReadStatus();
    io.print(F("  shared block @0x")); io.print((uint32_t)GIGA_SHM_BASE, HEX);
    io.print(F("  magic=0x"));      io.print((uint32_t)shm->e.magic, HEX);
    io.print(F(" (expected 0x"));   io.print((uint32_t)GIGA_SHM_MAGIC, HEX);
    io.print(F(")  bootId=0x"));    io.print(hostBootId, HEX);
    io.print(F(" ack=0x"));         io.print((uint32_t)shm->e.ackBootId, HEX);
    io.print(F("  heartbeat="));    io.println((uint32_t)shm->e.heartbeat);
    if (shm->e.heartbeat == 0 && shm->e.magic == 0) {
        io.println(F("  -> the M4 is not executing. Flash Giga_RemoteController_M4"));
        io.println(F("     with Target core = \"M4 Co-processor\" and Flash split ="));
        io.println(F("     \"1MB M7 + 1MB M4\", the same split as this sketch."));
    } else if (shm->e.magic != GIGA_SHM_MAGIC) {
        io.println(F("  -> the M4 is alive but speaks a different protocol version."));
        io.println(F("     Copy giga_dual_shared.h over the M4 sketch's copy and reflash both."));
    }
}
#endif

// Wait for the co-processor to come up after bootM4(). Returns false if it
// never answers — the caller then reports ERR:M4_NOT_RUNNING rather than
// capturing whatever happens to be in SRAM4.
static bool engineWaitBoot(uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        hostPumpEngine();
        if (engineAlive()) { engineUp = true; return true; }
        delay(5);
    }
    engineUp = false;
    return false;
}

// Issue a command and block until the engine acknowledges it (which it only
// does once the ADC is actually running, so a false return means the capture
// never started).
static bool hostIssue(uint32_t cmd,
                      uint32_t rate, uint32_t bits, uint32_t smooth,
                      uint32_t durMs, uint32_t logTarget,
                      uint32_t chMask, uint32_t statMask,
                      const int32_t* trig, bool midMode, uint32_t statPeriodMs) {
    if (!engineUp && !engineWaitBoot(500)) return false;

    shm->h.cmd          = cmd;
    shm->h.sampleRate   = rate;
    shm->h.bits         = bits;
    shm->h.smooth       = smooth;
    shm->h.durationMs   = durMs;
    shm->h.logTarget    = logTarget;
    shm->h.chMask       = chMask;
    shm->h.statMask     = statMask;
    shm->h.midMode      = midMode ? 1u : 0u;
    shm->h.statPeriodMs = statPeriodMs;
    for (int i = 0; i < N_CHANNELS; i++)
        shm->h.trigThresh[i] = trig ? trig[i] : 0;
    shm->h.ringTail = 0;
    ringTail = 0;
    hostCmdSeq++;
    shm->h.cmdSeq = hostCmdSeq;             // written last: it is the doorbell
    shmClean(&shm->h, sizeof(ShmToEngine));

    uint32_t t0 = millis();
    while (millis() - t0 < 3000) {
        hostPumpEngine();
        hostReadStatus();
        if (shm->e.ackSeq == hostCmdSeq)
            return shm->e.errCode == GIGA_ERR_NONE;
    }
    return false;
}

static void hostStop() {
    if (!engineUp) return;
    hostIssue(GIGA_CMD_STOP, 0, 0, 0, 0, 0, 0, 0, nullptr, false, 0);
}

// Move finished frames out of the shared ring into logData. Cheap enough that
// it can run in the middle of anything else the M7 is doing.
static void hostDrainRing() {
    hostReadStatus();
    uint32_t head = shm->e.ringHead;
    uint32_t trig = shm->e.midTrigAt;      // 0xFFFFFFFF until the trigger fires

    while (ringTail != head) {
        const volatile uint16_t* src = shm->ring[ringTail % SHM_RING_FRAMES];
        shmInvalidate(src, sizeof(uint16_t) * SHM_N_CH);

        if (hMidMode) {
            if (ringTail < trig) {
                // Pre-trigger: circular over the first half of logData.
                for (int ch = 0; ch < N_CHANNELS; ch++)
                    if (hPinReq[ch]) logData[ch][hMidRingHead] = src[ch];
                if (hMidRingFill < hMidPreSize) hMidRingFill++;
                hMidRingHead = (hMidRingHead + 1) % hMidPreSize;
            } else if (hMidPostCount < hMidPostSize) {
                for (int ch = 0; ch < N_CHANNELS; ch++)
                    if (hPinReq[ch]) logData[ch][hMidPreSize + hMidPostCount] = src[ch];
                hMidPostCount++;
            }
        } else if (hLogCnt < hLogTarget) {
            for (int ch = 0; ch < N_CHANNELS; ch++)
                if (hPinReq[ch]) logData[ch][hLogCnt] = src[ch];
            hLogCnt++;
        }
        ringTail++;
    }
    shm->h.ringTail = ringTail;
    shmClean(&shm->h, sizeof(ShmToEngine));
}

// Statistics come back as integer accumulators so the engine's hot loop never
// touches floating point; the maths is done here, once.
static void hostChanStats(int ch, double& mean, double& rms, double& stdDev) {
    uint32_t n = shm->e.cCount[ch];
    if (n == 0) { mean = rms = stdDev = 0.0; return; }
    double sum = (double)shm->e.cSum[ch];
    double sq  = (double)shm->e.cSumSq[ch];
    mean   = sum / n;
    rms    = sqrt(sq / n);
    stdDev = sqrt(fabs(sq / n - mean * mean));
}

// ── Display / touch / USB ─────────────────────────────────────────────────────
GigaDisplay_GFX          gfx;
Arduino_GigaDisplayTouch touch;
bool displayPresent = false;

USBHostMSD          msd;
mbed::FATFileSystem usbFs("usb");
bool usbMounted = false;

// ── Colors ────────────────────────────────────────────────────────────────────
#define C_BG      0x0000u
#define C_PANEL   0x2104u
#define C_BORDER  0x4208u
#define C_TEXT    0xFFFFu
#define C_CYAN    0x07FFu
#define C_GREEN   0x07E0u
#define C_RED     0xF800u
#define C_YELLOW  0xFFE0u
#define C_MAGENTA 0xF81Fu

static const uint16_t CH_COLORS[8] = {
    0x1BB6u, 0xFBE1u, 0x2CC5u, 0xD009u,
    0x93D5u, 0x8B0Bu, 0xE336u, 0x7BEFu,
};
static const uint16_t DIFF_COLORS[2] = { C_RED, C_MAGENTA };

// ── Preset data (hardcoded from giga_presets.json) ────────────────────────────
struct PChan { bool act; int typ; float off, scl; };  // typ: 1=V 2=I
struct DiffCfg { bool en; int pos, neg, typ; float off, scl; };
struct Preset {
    const char* name;
    int  rateIdx, bits, timeMs, smooth, points;
    PChan ch[8];
    DiffCfg d1, d2;
};

// Rate options — idx matches RATE_OPTIONS[] below
// 0=10k 1=44.1k 2=50k 3=100k 4=250k 5=500k 6=1M
static const uint32_t RATE_OPTIONS[] =
    {10000UL,44100UL,50000UL,100000UL,250000UL,500000UL,1000000UL};
static const int N_RATE_OPT = 7;

// Generated from giga_presets.json — 7 preset(s)
static const Preset PRESETS[] = {
  { "Board1",
    1,16,25,1,5000,
    { {true,1,1.612548f,44.462994f},{true,2,1.613425f,10.900730f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {false,4,5,1,0,1}, {false,2,3,1,0,1} },
  { "Board1 DiffA",
    1,16,25,1,5000,
    { {true,1,1.612548f,44.462994f},{true,2,1.613425f,10.900730f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {false,4,5,2,0.0f,11.8f}, {false,2,3,1,0,1} },
  { "Board1 DiffA 5mOhm",
    3,14,30,2,2000,
    { {true,1,1.612548f,44.462994f},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,4,5,2,0.0f,44.107620f}, {false,2,3,1,0.0f,1.0f} },
  { "120VAC Board",
    3,14,50,1,4000,
    { {true,2,0,1},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,4,5,2,0.0f,1.181836f}, {true,2,3,1,0.0f,102.779803f} },
  { "120VAC Board 25A",
    0,16,100,5,4000,
    { {false,2,0,1},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,3,2,2,0.0f,11.917674f}, {true,5,4,1,0.0f,102.508232f} },
  { "47V board",
    1,16,50,3,4000,
    { {false,2,0,1},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,3,2,2,0.0f,10.603871f}, {true,5,4,1,0.0f,22.550535f} },
};
static const int N_PRESETS = 6;



// ── Screen state ──────────────────────────────────────────────────────────────
enum Screen { SCR_SETTINGS, SCR_CAPTURING, SCR_PLOT, SCR_PRESETS, SCR_DIFF,
              SCR_LOGGER, SCR_LOGGING, SCR_STREAM };
static Screen currentScreen = SCR_SETTINGS;

// ── GUI parameters ────────────────────────────────────────────────────────────
// dChanType: 0=off 1=V 2=I
static int   dChanType[N_CHANNELS]   = {1,0,0,0,0,0,0,0};
static float dChanOffset[N_CHANNELS] = {0,0,0,0,0,0,0,0};
static float dChanScale[N_CHANNELS]  = {1,1,1,1,1,1,1,1};
static DiffCfg dDiff[2] = {{false,4,5,2,0,1},{false,2,3,1,0,1}};

static int dRateIdx = 4;   // 250 kHz default
static int dBits    = 12;
static int dTime    = 100;
static int dSmooth  = 1;
static int dPoints  = 1000;
static int dTrigThr = 0;

// Live-stream reporting rate. The headline number and the min/max/std summary
// are recomputed once per period over EVERY sample taken in that period.
static const int STREAM_HZ_OPTS[] = {1, 2, 4, 5, 10, 20};
static const int N_STREAM_HZ      = 6;
static int       dStreamHzIdx     = 2;   // 4 Hz

// ── Post-capture state ────────────────────────────────────────────────────────
static bool     captureReady  = false;
static bool     lastPinReq[N_CHANNELS]  = {};
static int      lastLogCnt[N_CHANNELS] = {};
static int      lastChanType[N_CHANNELS] = {};
static float    lastChanOffset[N_CHANNELS] = {};
static float    lastChanScale[N_CHANNELS]  = {};
static DiffCfg  lastDiff[2];
static int      lastBitRes    = 12;
static uint32_t lastElapsedMs = 0;
static bool     lastMidMode   = false;
static int      lastMidRingHead=0, lastMidRingFill=0, lastMidPostCount=0, lastMidPreSize=0;

// ── Continuous view ───────────────────────────────────────────────────────────
static bool     contMode   = false;
static uint32_t lastContMs = 0;

// ── Live stream ───────────────────────────────────────────────────────────────
// One reporting period yields, per channel, the headline number (the mean of
// every sample taken in that period) plus that period's min, max and standard
// deviation. Emitted to whichever Stream started it, and drawn on-screen.
static bool       streamActive   = false;
static bool       streamOnWifi   = false;
static bool       streamLocal    = false;  // started from the touchscreen
static Stream*    streamIo       = nullptr;
static WiFiClient streamClient;
static uint32_t   streamSeen     = 0;      // last statSeq consumed
static uint32_t   streamHz       = 4;
static int        streamBits     = 12;
static uint32_t   streamRate     = 250000;
static uint32_t   streamSmooth   = 1;
static bool       streamSel[SHM_N_STAT] = {};  // per stat slot: 0-7 = A0-A7, 8-9 = D1/D2
// Which pins D1/D2 mean for the stream in progress. A host names them in the
// STREAM command; a stream started from the touchscreen inherits dDiff[].
static int8_t     streamDPos[SHM_N_DIFF] = {4,2};
static int8_t     streamDNeg[SHM_N_DIFF] = {5,3};
static bool       holdClient     = false;  // keep this TCP socket after the reply
static bool       cmdFromStream  = false;  // this command arrived on the held socket

// ── Datalogger state ──────────────────────────────────────────────────────────
static float    logThresh[N_CHANNELS]    = {};
static float    logDiffThresh[2]         = {};
static int      logIntervalMs            = 1000;
static bool     logRunning               = false;
static FILE*    logFile                  = nullptr;
static int      logFileNum               = 0;
static int      logRowCount              = 0;
static uint32_t logStartMs               = 0;
static uint32_t logStartUs               = 0;
static uint32_t logLastWriteMs           = 0;
static float    logLastCh[N_CHANNELS]    = {};
static float    logLastDiff[2]           = {};
static float    logCurCh[N_CHANNELS]     = {};
static float    logCurDiff[2]            = {};
static uint32_t logLastDispMs            = 0;

// ── Numpad overlay state ──────────────────────────────────────────────────────
static bool  numpadActive = false;
static char  numpadBuf[12] = "";
static int   numpadTarget  = -1;   // 0-7=logThresh[ch], 8-9=logDiffThresh[d], 10=logIntervalMs
static char  numpadTitle[24] = "";

// ── Touch debounce ────────────────────────────────────────────────────────────
static uint32_t       lastTouchMs        = 0;
static const uint32_t TOUCH_DEBOUNCE_MS  = 250;

// ── Layout constants ──────────────────────────────────────────────────────────
static const int DISP_W    = 800;
static const int DISP_H    = 480;
static const int HDR_H     = 44;
static const int CH_ROW_Y  = HDR_H + 4;
static const int CH_ROW_H  = 52;
static const int PARAM_Y   = CH_ROW_Y + CH_ROW_H + 4;
static const int PARAM_ROW = 40;
static const int N_PARAMS  = 7;
static const int DIFF_BTN_Y = PARAM_Y + N_PARAMS * PARAM_ROW + 4;
static const int DIFF_BTN_H = 44;
static const int CAP_BTN_Y  = DIFF_BTN_Y + DIFF_BTN_H + 4;
static const int CAP_BTN_H  = DISP_H - CAP_BTN_Y - 4;
// CAPTURE + CONT + LIVE side-by-side split
static const int CAP_W  = 430;
static const int CONT_X = 4 + CAP_W + 4;
static const int CONT_W = 170;
static const int LIVE_X = CONT_X + CONT_W + 4;
static const int LIVE_W = DISP_W - 4 - LIVE_X;

static const int PL_X0      = 10;
static const int PL_LABEL_W = 185;
static const int PL_BTN_W   = 52;
static const int PL_VAL_W   = 160;

static const char* PARAM_LABELS[N_PARAMS] =
    {"Rate (Hz)","Bits","Time (ms)","Smooth","Log Points","Trig Thresh","Live Rate (Hz)"};

// Plot area — narrower to leave room for right axis labels
static const int PLT_X      = 62;
static const int PLT_Y      = HDR_H + 4;
static const int PLT_W      = DISP_W - PLT_X - 54;
static const int STAT_ROW_H = 11;  // px per stats row below the plot

// ── Calibration math ──────────────────────────────────────────────────────────
static float vStep(int bits) { return 3.3f / (float)((1 << bits) - 1); }
static float calChan(int raw, float offset, float scale, int bits) {
    return (raw * vStep(bits) - offset) * scale;
}
static float calDiff(int rp, int rn, float offset, float scale, int bits) {
    return ((rp - rn) * vStep(bits) - offset) * scale;
}

// ── Drawing helpers ───────────────────────────────────────────────────────────
static void drawBtn(int x, int y, int w, int h,
                    uint16_t fill, uint16_t tc, const char* lbl, int ts=2) {
    gfx.fillRoundRect(x,y,w,h,5,fill);
    gfx.drawRoundRect(x,y,w,h,5,C_BORDER);
    gfx.setTextColor(tc); gfx.setTextSize(ts);
    int tw = strlen(lbl)*6*ts, th = 8*ts;
    gfx.setCursor(x+(w-tw)/2, y+(h-th)/2);
    gfx.print(lbl);
}
static bool hit(int bx,int by,int bw,int bh,int tx,int ty) {
    return tx>=bx && tx<bx+bw && ty>=by && ty<by+bh;
}

// ── Preset application ────────────────────────────────────────────────────────
static void applyPreset(int idx) {
    const Preset& p = PRESETS[idx];
    dRateIdx = p.rateIdx;
    dBits    = p.bits;
    dTime    = p.timeMs;
    dSmooth  = max(1, p.smooth);
    dPoints  = p.points;
    for (int i=0;i<N_CHANNELS;i++) {
        dChanType[i]   = p.ch[i].act ? p.ch[i].typ : 0;
        dChanOffset[i] = p.ch[i].off;
        dChanScale[i]  = p.ch[i].scl;
    }
    dDiff[0] = p.d1;
    dDiff[1] = p.d2;
}

// ── Settings screen ───────────────────────────────────────────────────────────
static void getParamStr(int r, char* buf, int sz) {
    switch(r) {
        case 0: snprintf(buf,sz,"%lu",(unsigned long)RATE_OPTIONS[dRateIdx]); break;
        case 1: snprintf(buf,sz,"%d",dBits);   break;
        case 2: snprintf(buf,sz,"%d",dTime);   break;
        case 3: snprintf(buf,sz,"%d",dSmooth); break;
        case 4: snprintf(buf,sz,"%d",dPoints); break;
        case 5: if(dTrigThr==0) snprintf(buf,sz,"OFF");
                else            snprintf(buf,sz,"%d",dTrigThr); break;
        case 6: snprintf(buf,sz,"%d",STREAM_HZ_OPTS[dStreamHzIdx]); break;
    }
}
static void refreshParamValue(int r) {
    int ry = PARAM_Y + r*PARAM_ROW;
    int vx = PL_X0 + PL_LABEL_W + PL_BTN_W;
    gfx.fillRect(vx,ry,PL_VAL_W,PARAM_ROW,(r&1)?C_PANEL:C_BG);
    char vbuf[32]; getParamStr(r,vbuf,sizeof(vbuf));
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    int vw = strlen(vbuf)*12;
    gfx.setCursor(vx+(PL_VAL_W-vw)/2, ry+(PARAM_ROW-16)/2);
    gfx.print(vbuf);
}
static void drawCaptureButtons() {
    int n=0;
    for(int i=0;i<N_CHANNELS;i++) if(dChanType[i]) n++;
    if(dDiff[0].en||dDiff[1].en) n++;
    bool ok=(n>0);
    drawBtn(4,      CAP_BTN_Y, CAP_W, CAP_BTN_H, ok?C_GREEN:C_BORDER, C_TEXT, ok?"CAPTURE":"Select a channel", 3);
    drawBtn(CONT_X, CAP_BTN_Y, CONT_W,CAP_BTN_H, ok?C_CYAN  :C_BORDER, C_TEXT, "CONT", 2);
    drawBtn(LIVE_X, CAP_BTN_Y, LIVE_W,CAP_BTN_H, ok?C_YELLOW:C_BORDER, C_BG,   "LIVE", 2);
}
static void drawSettingsScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Giga ADC Capture");
    // WiFi endpoint + co-processor state (small, between title and buttons)
    gfx.setTextColor(wifiActive?C_GREEN:C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(320,12); gfx.print("WiFi: "); gfx.print(wifiIPStr);
    gfx.setTextColor(engineUp?C_GREEN:C_RED);
    gfx.setCursor(320,24);
#if ADC_ON_M4
    gfx.print(engineUp ? "ADC: M4 co-processor" : "ADC: M4 NOT RUNNING");
#else
    gfx.print(engineUp ? "ADC: M7 (single core)" : "ADC: engine down");
#endif
    drawBtn(DISP_W-164,2,78,HDR_H-4,C_BORDER,C_TEXT,"Presets",2);
    drawBtn(DISP_W-82,2,78,HDR_H-4,C_BORDER,C_TEXT,"Logger",2);

    // Channel toggles — label shows type
    int cbw = DISP_W/N_CHANNELS;
    for(int i=0;i<N_CHANNELS;i++) {
        char lbl[8];
        if(dChanType[i]==0)      snprintf(lbl,sizeof(lbl),"%s",CH_NAMES[i]);
        else if(dChanType[i]==1) snprintf(lbl,sizeof(lbl),"%s V",CH_NAMES[i]);
        else                     snprintf(lbl,sizeof(lbl),"%s I",CH_NAMES[i]);
        drawBtn(i*cbw+2,CH_ROW_Y,cbw-4,CH_ROW_H-4,
                dChanType[i]?CH_COLORS[i]:C_PANEL, C_TEXT, lbl, 2);
    }

    // Param rows
    for(int r=0;r<N_PARAMS;r++) {
        int ry=PARAM_Y+r*PARAM_ROW;
        if(r&1) gfx.fillRect(0,ry,DISP_W,PARAM_ROW,C_PANEL);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        gfx.setCursor(PL_X0+4, ry+(PARAM_ROW-16)/2);
        gfx.print(PARAM_LABELS[r]);
        int bx=PL_X0+PL_LABEL_W;
        drawBtn(bx,ry+2,PL_BTN_W,PARAM_ROW-4,C_BORDER,C_TEXT,"-",3);
        char vbuf[32]; getParamStr(r,vbuf,sizeof(vbuf));
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        int vw=strlen(vbuf)*12;
        gfx.setCursor(bx+PL_BTN_W+(PL_VAL_W-vw)/2, ry+(PARAM_ROW-16)/2);
        gfx.print(vbuf);
        drawBtn(bx+PL_BTN_W+PL_VAL_W,ry+2,PL_BTN_W,PARAM_ROW-4,C_BORDER,C_TEXT,"+",3);
    }

    // Diff config button — shows current diff summary
    char dbuf[48] = "Diff: off";
    if(dDiff[0].en && dDiff[1].en)
        snprintf(dbuf,sizeof(dbuf),"Diff: %s-%s & %s-%s",
            CH_NAMES[dDiff[0].pos],CH_NAMES[dDiff[0].neg],
            CH_NAMES[dDiff[1].pos],CH_NAMES[dDiff[1].neg]);
    else if(dDiff[0].en)
        snprintf(dbuf,sizeof(dbuf),"Diff: %s-%s (%s)",
            CH_NAMES[dDiff[0].pos],CH_NAMES[dDiff[0].neg],
            dDiff[0].typ==1?"V":"I");
    else if(dDiff[1].en)
        snprintf(dbuf,sizeof(dbuf),"Diff: %s-%s (%s)",
            CH_NAMES[dDiff[1].pos],CH_NAMES[dDiff[1].neg],
            dDiff[1].typ==1?"V":"I");
    drawBtn(4,DIFF_BTN_Y,DISP_W-8,DIFF_BTN_H,C_PANEL,C_CYAN,dbuf,2);

    drawCaptureButtons();
}

// ── Presets screen ────────────────────────────────────────────────────────────
static const int PRE_COLS  = 5;
static const int PRE_ROWS  = 5;
static const int PRE_CW    = DISP_W / PRE_COLS;          // 160 px per cell
static const int PRE_RH    = (DISP_H - HDR_H) / PRE_ROWS; // 87 px per cell
static const int PRE_BTN_W = PRE_CW - 4;                 // 156 px
static const int PRE_BTN_H = PRE_RH - 3;                 // 84 px

static void drawPresetsScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Select Preset");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_BORDER,C_TEXT,"< Back",2);
    int n = min(N_PRESETS, PRE_COLS * PRE_ROWS);
    for(int i=0;i<n;i++) {
        int bx = (i % PRE_COLS) * PRE_CW + 2;
        int by = HDR_H + (i / PRE_COLS) * PRE_RH + 1;
        int ts = (strlen(PRESETS[i].name) * 12 <= PRE_BTN_W) ? 2 : 1;
        drawBtn(bx, by, PRE_BTN_W, PRE_BTN_H, C_PANEL, C_TEXT, PRESETS[i].name, ts);
    }
}

// ── Diff config screen ────────────────────────────────────────────────────────
static void drawDiffRow(int d) {
    int ry = 60 + d*110;
    uint16_t bg = (d==0) ? C_BG : C_PANEL;
    gfx.fillRect(0,ry-2,DISP_W,56,bg);

    DiffCfg& dc = dDiff[d];
    drawBtn(10,ry,160,50,dc.en?C_GREEN:C_BORDER,C_TEXT,dc.en?"ON":"OFF",2);

    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(182,ry+17); gfx.print("Pos:");
    drawBtn(228,ry,44,50,C_BORDER,C_TEXT,"<",3);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    gfx.setCursor(278,ry+17); gfx.print(CH_NAMES[dc.pos]);
    drawBtn(322,ry,44,50,C_BORDER,C_TEXT,">",3);

    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(382,ry+17); gfx.print("Neg:");
    drawBtn(428,ry,44,50,C_BORDER,C_TEXT,"<",3);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    gfx.setCursor(478,ry+17); gfx.print(CH_NAMES[dc.neg]);
    drawBtn(522,ry,44,50,C_BORDER,C_TEXT,">",3);

    drawBtn(588,ry,88,50,dc.typ==1?C_CYAN:C_BORDER,C_TEXT,"V",3);
    drawBtn(684,ry,88,50,dc.typ==2?C_CYAN:C_BORDER,C_TEXT,"I",3);
}
static void drawDiffScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Differential Inputs");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_GREEN,C_TEXT,"Done",2);

    gfx.setTextColor(C_BORDER); gfx.setTextSize(2);
    gfx.setCursor(10,54); gfx.print("── Diff 1 ──────────────────────────────────────────");
    drawDiffRow(0);
    gfx.setTextColor(C_BORDER); gfx.setTextSize(2);
    gfx.setCursor(10,166); gfx.print("── Diff 2 ──────────────────────────────────────────");
    drawDiffRow(1);

    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(10,300); gfx.print("Offset & scale are loaded from presets.");
}

// ── Capture progress screen ───────────────────────────────────────────────────
static void drawCaptureScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Capturing...");
    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(20,100); gfx.print("Running ADC capture");
}
static void updateProgress(uint32_t elapsed, uint32_t total) {
    const int bx=40,by=200,bw=DISP_W-80,bh=30;
    int fill=(total>0)?(int)((uint64_t)elapsed*bw/total):0;
    if(fill>bw) fill=bw;
    gfx.fillRect(bx,by,bw,bh,C_BORDER);
    if(fill>0) gfx.fillRect(bx,by,fill,bh,C_CYAN);
    gfx.drawRect(bx,by,bw,bh,C_TEXT);
    gfx.fillRect(bx,by+bh+8,280,20,C_BG);
    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(bx,by+bh+8);
    char buf[32]; snprintf(buf,sizeof(buf),"%lu / %d ms",(unsigned long)elapsed,(int)total);
    gfx.print(buf);
}

// ── Live stream ───────────────────────────────────────────────────────────────
// A "stat slot" is one of the 8 real channels or one of the 2 differential
// pairs. The engine reduces all of them per sample, so a pair's spread is the
// spread of the actual difference rather than something guessed from its legs.
static const char* DIFF_NAMES[SHM_N_DIFF] = {"D1","D2"};

static const char* statName(int k) {
    return (k < N_CHANNELS) ? CH_NAMES[k] : DIFF_NAMES[k - N_CHANNELS];
}

// Screen label: a pair always names its legs, because the host can point D1/D2
// at pins that differ from the board's own Diff config.
static void statLabel(int k, char* buf, int sz) {
    if (k < N_CHANNELS) { snprintf(buf, sz, "%s", CH_NAMES[k]); return; }
    int d = k - N_CHANNELS;
    snprintf(buf, sz, "%s %s-%s", DIFF_NAMES[d],
             CH_NAMES[streamDPos[d]], CH_NAMES[streamDNeg[d]]);
}

// Engineering value for a stat slot's raw reading.
static float statCal(int k, float raw) {
    if (k < N_CHANNELS)
        return (raw * vStep(streamBits) - dChanOffset[k]) * dChanScale[k];
    const DiffCfg& dc = dDiff[k - N_CHANNELS];
    return (raw * vStep(streamBits) - dc.off) * dc.scl;
}
// NOTE: the on-screen numbers use the board's own Diff offset/scale even when a
// host has repointed D1/D2 at other pins - the board has no way to know the
// host's calibration. The label names the legs so the mismatch is visible; a
// host-driven stream's authoritative numbers are the ones in the GUI.

// A spread carries the gain but not the offset.
static float statCalSpread(int k, float raw) {
    float scl = (k < N_CHANNELS) ? dChanScale[k] : dDiff[k - N_CHANNELS].scl;
    return raw * vStep(streamBits) * fabsf(scl);
}

static bool statIsCurrent(int k) {
    return (k < N_CHANNELS) ? (dChanType[k] == 2) : (dDiff[k - N_CHANNELS].typ == 2);
}

static uint16_t statColor(int k) {
    return (k < N_CHANNELS) ? CH_COLORS[k] : DIFF_COLORS[k - N_CHANNELS];
}

static int streamActiveList(int* idx) {
    int n = 0;
    for (int k = 0; k < SHM_N_STAT; k++) if (streamSel[k]) idx[n++] = k;
    return n;
}

// Raw counts straight from the engine — no calibration applied.
static void streamRawStat(int k, PeriodStat& p) {
    p.n = statSnap.sCount[k];
    if (p.n == 0) { p.mean = p.mn = p.mx = p.sd = p.last = 0; return; }
    double sum = (double)statSnap.sSum[k];
    double sq  = (double)statSnap.sSumSq[k];
    p.mean = sum / p.n;
    p.mn   = statSnap.sMin[k];
    p.mx   = statSnap.sMax[k];
    p.last = statSnap.sLast[k];
    p.sd   = sqrt(fabs(sq / p.n - p.mean * p.mean));
}

static void drawStreamScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("LIVE");
    gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
    char hb[72];
    snprintf(hb,sizeof(hb),"%lu Hz  %lu S/s  %d-bit  smooth %lu",
             (unsigned long)streamHz,(unsigned long)streamRate,streamBits,
             (unsigned long)streamSmooth);
    gfx.setCursor(96,10);  gfx.print(hb);
    gfx.setCursor(96,24);
    gfx.print(streamLocal ? "started on-screen" : "streaming to host");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_RED,C_TEXT,"Stop",2);
}

// Repaint just the numbers — the frame around them never changes, so a period
// update touches only the rows.
static void drawStreamValues() {
    int idx[SHM_N_STAT];
    int n = streamActiveList(idx);
    if (n == 0) return;

    int top  = HDR_H + 4;
    int rowH = (DISP_H - top - 4) / n;
    // Headline text scales with the space each slot gets.
    int big = rowH >= 150 ? 7 : rowH >= 100 ? 5 : rowH >= 70 ? 4 : rowH >= 50 ? 3 : 2;

    for (int i = 0; i < n; i++) {
        int k  = idx[i];
        int ry = top + i * rowH;
        gfx.fillRect(0, ry, DISP_W, rowH - 2, (i & 1) ? C_PANEL : C_BG);

        PeriodStat p; streamRawStat(k, p);
        const char* unit = statIsCurrent(k) ? "A" : "V";
        float mean = statCal(k, (float)p.mean);
        float mn   = statCal(k, (float)p.mn);
        float mx   = statCal(k, (float)p.mx);
        float sd   = statCalSpread(k, (float)p.sd);

        char lbl[16]; statLabel(k, lbl, sizeof(lbl));
        gfx.setTextColor(statColor(k)); gfx.setTextSize(2);
        gfx.setCursor(8, ry + 4); gfx.print(lbl);

        char buf[24];
        snprintf(buf, sizeof(buf), "%.4g%s", mean, unit);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(big);
        gfx.setCursor(64, ry + (rowH - 8 * big) / 2);
        gfx.print(buf);

        char sbuf[80];
        snprintf(sbuf, sizeof(sbuf), "min %.4g   max %.4g   sd %.3g   n=%lu",
                 mn, mx, sd, (unsigned long)p.n);
        gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
        gfx.setCursor(64, ry + rowH - 14);
        gfx.print(sbuf);
    }
}

// One line per stat slot, then a terminator. Raw counts, like every other reply
// from this board — the GUI owns calibration.
static void streamEmitPeriod() {
    if (!streamIo) return;
    Stream& io = *streamIo;
    uint32_t seq = statSnap.statSeq;
    for (int k = 0; k < SHM_N_STAT; k++) {
        if (!streamSel[k]) continue;
        PeriodStat p; streamRawStat(k, p);
        if (p.n == 0) continue;
        io.print("S|SEQ:");  io.print(seq);
        io.print("|T:");     io.print(statSnap.statTimeMs);
        io.print("|SPAN:");  io.print(statSnap.statSpanMs);
        io.print("|CH:");    io.print(statName(k));
        io.print("|N:");     io.print(p.n);
        io.print("|MIN:");   io.print((long)p.mn);
        io.print("|MAX:");   io.print((long)p.mx);
        io.print("|MEAN:");  io.print(p.mean, 4);
        io.print("|STD:");   io.print(p.sd, 4);
        io.print("|LAST:");  io.println((long)p.last);
    }
    io.print("E|SEQ:");   io.print(seq);
    io.print("|BITS:");   io.print(streamBits);
    io.print("|DROP:");   io.println(statSnap.ringDropped);
}

static void stopStream(bool notify) {
    if (!streamActive) return;
    streamActive = false;
    hostStop();
    if (notify && streamIo) streamIo->println("STREAMOFF");
    if (streamOnWifi) {
        // Give the GUI a chance to read STREAMOFF and close first - stop() on a
        // socket with unread data can drop it.
        streamClient.flush();
        uint32_t t0 = millis();
        while (streamClient.connected() && millis() - t0 < 2000) delay(2);
        streamClient.stop();
        streamClient = WiFiClient();
    }
    streamIo     = nullptr;
    streamOnWifi = false;
    streamLocal  = false;
    if (displayPresent && currentScreen == SCR_STREAM) {
        currentScreen = SCR_SETTINGS;
        drawSettingsScreen();
    }
}

static void stopLoggerQuiet();   // defined with the datalogger, below

// io == nullptr starts a screen-only stream (the LIVE button). sel[] is indexed
// by stat slot: 0-7 are A0-A7, 8 and 9 are Diff 1 and Diff 2.
//
// dpos/dneg name the pins behind D1/D2, or are nullptr to inherit the board's
// own Diff config. A host MUST name them: the board's Diff 1 is not necessarily
// the host's Diff 1 (the built-in preset table and the GUI's giga_presets.json
// drift apart), and silently pairing one side's number with the other side's
// calibration reads as a plausible but wrong measurement.
static bool startStream(Stream* io, uint32_t rate, int bits, uint32_t smooth,
                        uint32_t hz, const bool* sel,
                        const int8_t* dpos, const int8_t* dneg) {
    stopStream(false);
    contMode = false;
    if (logRunning) stopLoggerQuiet();

    streamRate   = rate   ? rate : 250000u;
    streamBits   = bits   ? bits : 12;
    streamSmooth = smooth ? smooth : 1u;
    streamHz     = hz     ? hz : 4u;
    if (streamHz > 50) streamHz = 50;

    // A differential pair needs both its legs sampled, whether or not they are
    // displayed in their own right.
    for (int d = 0; d < SHM_N_DIFF; d++) {
        int8_t p = dpos ? dpos[d] : (int8_t)dDiff[d].pos;
        int8_t n = dneg ? dneg[d] : (int8_t)dDiff[d].neg;
        streamDPos[d] = (p >= 0 && p < N_CHANNELS) ? p : (int8_t)dDiff[d].pos;
        streamDNeg[d] = (n >= 0 && n < N_CHANNELS) ? n : (int8_t)dDiff[d].neg;
    }

    uint32_t mask = 0;
    for (int k = 0; k < SHM_N_STAT; k++) streamSel[k] = sel[k];
    for (int ch = 0; ch < N_CHANNELS; ch++) if (streamSel[ch]) mask |= (1u << ch);
    for (int d = 0; d < SHM_N_DIFF; d++) {
        if (!streamSel[N_CHANNELS + d]) continue;
        mask |= (1u << streamDPos[d]) | (1u << streamDNeg[d]);
    }
    if (mask == 0) { if (io) io->println("ERR:NO_CHANNELS"); return false; }

    for (int d = 0; d < SHM_N_DIFF; d++) {
        shm->h.diffEn[d]  = streamSel[N_CHANNELS + d] ? 1u : 0u;
        shm->h.diffPos[d] = streamDPos[d];
        shm->h.diffNeg[d] = streamDNeg[d];
    }

    uint32_t periodMs = 1000u / streamHz;
    if (!hostIssue(GIGA_CMD_STREAM, streamRate, streamBits, streamSmooth,
                   0, 0, mask, mask, nullptr, false, periodMs)) {
        if (io) io->println(engineUp ? "ERR:ADC_BEGIN_FAILED" : "ERR:M4_NOT_RUNNING");
        return false;
    }

    streamIo     = io;
    streamLocal  = (io == nullptr);
    streamSeen   = 0;
    streamActive = true;

    if (io) {
        io->print("STREAMON|HZ:");   io->print(streamHz);
        io->print("|BITS:");         io->print(streamBits);
        io->print("|RATE:");         io->print(streamRate);
        io->print("|SMOOTH:");       io->print(streamSmooth);
        io->print("|CH:");
        bool first = true;
        for (int k = 0; k < SHM_N_STAT; k++) {
            if (!streamSel[k]) continue;
            if (!first) io->print(',');
            io->print(statName(k)); first = false;
        }
        for (int d = 0; d < SHM_N_DIFF; d++) {
            if (!streamSel[N_CHANNELS + d]) continue;
            io->print('|'); io->print(DIFF_NAMES[d]); io->print(':');
            io->print(CH_NAMES[streamDPos[d]]); io->print('-');
            io->print(CH_NAMES[streamDNeg[d]]);
        }
        io->println();
    }
    if (displayPresent) { currentScreen = SCR_STREAM; drawStreamScreen(); }
    return true;
}

// Called every loop() while a stream is running.
static void streamStep() {
    if (!streamActive) return;
    hostPumpEngine();
    hostReadStatus();
    if (shm->e.statSeq == streamSeen) return;
    if (!hostSnapshotStats()) return;         // torn read; retry next loop
    streamSeen = statSnap.statSeq;
    streamEmitPeriod();
    if (displayPresent && currentScreen == SCR_STREAM) drawStreamValues();
}

static void handleStreamTouch(int tx,int ty) {
    if (hit(DISP_W-114,2,110,HDR_H-4,tx,ty)) stopStream(true);
}

// ── Plot screen ───────────────────────────────────────────────────────────────
struct ChanSt { float mn, mx, sum, ssq; int cnt; };

static int getRawVal(int ch, int j) {
    if(lastMidMode && j<lastMidRingFill) {
        int si=(lastMidRingFill<lastMidPreSize)?0:lastMidRingHead;
        return logData[ch][(si+j)%lastMidPreSize];
    } else if(lastMidMode) {
        return logData[ch][lastMidPreSize+(j-lastMidRingFill)];
    }
    return logData[ch][j];
}

static void drawPlotScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    gfx.setCursor(12,14);
    char hbuf[48];
    snprintf(hbuf,sizeof(hbuf),"Done  %lu ms  %d-bit",(unsigned long)lastElapsedMs,lastBitRes);
    gfx.print(hbuf);
    drawBtn(DISP_W-220,2,100,HDR_H-4,contMode?C_RED:C_BORDER,C_TEXT,contMode?"Stop":"< Back",2);
    drawBtn(DISP_W-114,2,110,HDR_H-4,usbMounted?C_GREEN:C_BORDER,C_TEXT,
            usbMounted?"Export":"No USB",2);

    // ── Count active channels → dynamic plot height ───────────────────────
    int nActive = 0;
    for(int ch=0;ch<N_CHANNELS;ch++)
        if(lastPinReq[ch]&&lastChanType[ch]!=0&&lastLogCnt[ch]>0) nActive++;
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(dc.en&&lastLogCnt[dc.pos]>0&&lastLogCnt[dc.neg]>0) nActive++;
    }
    int pltH = DISP_H - PLT_Y - 4 - nActive * STAT_ROW_H - 2;
    if(pltH < 180) pltH = 180;
    int statY = PLT_Y + pltH + 2;

    // ── Determine Y ranges AND accumulate per-channel stats ───────────────
    ChanSt chSt[N_CHANNELS], dSt[2];
    for(int i=0;i<N_CHANNELS;i++) chSt[i]={1e30f,-1e30f,0.0f,0.0f,0};
    for(int i=0;i<2;i++)           dSt[i]={1e30f,-1e30f,0.0f,0.0f,0};

    float vMin=1e30f,vMax=-1e30f,iMin=1e30f,iMax=-1e30f;
    bool hasV=false,hasI=false;

    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0||lastLogCnt[ch]==0) continue;
        bool isI=(lastChanType[ch]==2);
        ChanSt& st=chSt[ch];
        for(int j=0;j<lastLogCnt[ch];j++) {
            float v=calChan(getRawVal(ch,j),lastChanOffset[ch],lastChanScale[ch],lastBitRes);
            if(v<st.mn)st.mn=v; if(v>st.mx)st.mx=v; st.sum+=v; st.ssq+=v*v; st.cnt++;
            if(isI){if(v<iMin)iMin=v;if(v>iMax)iMax=v;hasI=true;}
            else   {if(v<vMin)vMin=v;if(v>vMax)vMax=v;hasV=true;}
        }
    }
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en||lastLogCnt[dc.pos]==0||lastLogCnt[dc.neg]==0) continue;
        bool isI=(dc.typ==2);
        int cnt=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]);
        ChanSt& st=dSt[d];
        for(int j=0;j<cnt;j++) {
            float v=calDiff(getRawVal(dc.pos,j),getRawVal(dc.neg,j),dc.off,dc.scl,lastBitRes);
            if(v<st.mn)st.mn=v; if(v>st.mx)st.mx=v; st.sum+=v; st.ssq+=v*v; st.cnt++;
            if(isI){if(v<iMin)iMin=v;if(v>iMax)iMax=v;hasI=true;}
            else   {if(v<vMin)vMin=v;if(v>vMax)vMax=v;hasV=true;}
        }
    }
    if(!hasV&&!hasI){ vMin=0;vMax=1;hasV=true; }
    if(hasV&&vMax-vMin<1e-6f){ float m=(vMin+vMax)/2; vMin=m-1;vMax=m+1; }
    if(hasI&&iMax-iMin<1e-6f){ float m=(iMin+iMax)/2; iMin=m-1;iMax=m+1; }
    if(!hasV){ vMin=iMin;vMax=iMax; }
    if(!hasI){ iMin=vMin;iMax=vMax; }

    // ── Draw plot area ────────────────────────────────────────────────────
    gfx.fillRect(PLT_X,PLT_Y,PLT_W,pltH,0x0821u);
    gfx.drawRect(PLT_X,PLT_Y,PLT_W,pltH,C_BORDER);
    for(int g=1;g<4;g++) {
        int gy=PLT_Y+g*pltH/4;
        for(int x=PLT_X;x<PLT_X+PLT_W;x+=6) gfx.drawPixel(x,gy,C_BORDER);
    }

    // Y axis labels — V left, I right
    gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
    char lbuf[12];
    snprintf(lbuf,sizeof(lbuf),"%.3g",vMax); gfx.setCursor(2,PLT_Y+2);          gfx.print(lbuf);
    snprintf(lbuf,sizeof(lbuf),"%.3g",vMin); gfx.setCursor(2,PLT_Y+pltH-10);   gfx.print(lbuf);
    if(hasI) {
        int rx=PLT_X+PLT_W+4;
        snprintf(lbuf,sizeof(lbuf),"%.3g",iMax); gfx.setCursor(rx,PLT_Y+2);         gfx.print(lbuf);
        snprintf(lbuf,sizeof(lbuf),"%.3g",iMin); gfx.setCursor(rx,PLT_Y+pltH-10);  gfx.print(lbuf);
    }

    // Find max log count for X scaling
    int maxCnt=1;
    for(int ch=0;ch<N_CHANNELS;ch++) if(lastPinReq[ch]&&lastLogCnt[ch]>maxCnt) maxCnt=lastLogCnt[ch];
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(dc.en) { int c=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]); if(c>maxCnt) maxCnt=c; }
    }

    auto mapY = [&](float v, float mn, float mx) -> int {
        int py = PLT_Y+pltH-1-(int)((v-mn)*(pltH-2)/(mx-mn));
        return constrain(py, PLT_Y, PLT_Y+pltH-1);
    };

    // Draw standalone channels
    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0||lastLogCnt[ch]<2) continue;
        bool isI=(lastChanType[ch]==2);
        float mn=isI?iMin:vMin, mx=isI?iMax:vMax;
        int ppx=-1,ppy=-1;
        for(int j=0;j<lastLogCnt[ch];j++) {
            float v=calChan(getRawVal(ch,j),lastChanOffset[ch],lastChanScale[ch],lastBitRes);
            int px=PLT_X+(int)((int64_t)j*PLT_W/maxCnt);
            int py=mapY(v,mn,mx);
            if(ppx>=0) gfx.drawLine(ppx,ppy,px,py,CH_COLORS[ch]);
            ppx=px; ppy=py;
        }
    }
    // Draw diff channels
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en) continue;
        int cnt=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]);
        if(cnt<2) continue;
        bool isI=(dc.typ==2);
        float mn=isI?iMin:vMin, mx=isI?iMax:vMax;
        int ppx=-1,ppy=-1;
        for(int j=0;j<cnt;j++) {
            float v=calDiff(getRawVal(dc.pos,j),getRawVal(dc.neg,j),dc.off,dc.scl,lastBitRes);
            int px=PLT_X+(int)((int64_t)j*PLT_W/maxCnt);
            int py=mapY(v,mn,mx);
            if(ppx>=0) gfx.drawLine(ppx,ppy,px,py,DIFF_COLORS[d]);
            ppx=px; ppy=py;
        }
    }

    // ── Stats rows ────────────────────────────────────────────────────────
    int ry = statY;
    auto drawStat = [&](uint16_t col, const char* lbl,
                        float mn, float mx, float sum, float ssq, int cnt) {
        if(cnt == 0) return;
        float avg = sum / cnt;
        float rms = sqrtf(ssq / cnt);
        gfx.fillRect(4, ry, 10, 8, col);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
        gfx.setCursor(17, ry); gfx.print(lbl);
        char buf[64];
        snprintf(buf,sizeof(buf),"Min:%.4g  Max:%.4g  Avg:%.4g  RMS:%.4g",mn,mx,avg,rms);
        gfx.setCursor(80, ry); gfx.print(buf);
        ry += STAT_ROW_H;
    };
    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0) continue;
        char lb[10]; snprintf(lb,sizeof(lb),"%s(%s)",CH_NAMES[ch],lastChanType[ch]==1?"V":"I");
        ChanSt& st=chSt[ch];
        drawStat(CH_COLORS[ch], lb, st.mn, st.mx, st.sum, st.ssq, st.cnt);
    }
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en) continue;
        char lb[10]; snprintf(lb,sizeof(lb),"D%d(%s)",d+1,dc.typ==1?"V":"I");
        ChanSt& st=dSt[d];
        drawStat(DIFF_COLORS[d], lb, st.mn, st.mx, st.sum, st.ssq, st.cnt);
    }
}

// ── Datalogger screens & logic ────────────────────────────────────────────────
static int nextLogFileNum() {
    for(int n=0;n<1000;n++){
        char fn[32]; snprintf(fn,sizeof(fn),"/usb/logger_%03d.csv",n);
        FILE* f=fopen(fn,"r"); if(!f) return n; fclose(f);
    }
    return 999;
}
static float adjThreshDn(float v) {
    if(v<=0) return 0;
    if(v>10)    return v-1.0f;
    if(v>1)     return v-0.1f;
    if(v>0.1f)  return v-0.01f;
    return (v<=0.001f)?0.0f:v-0.001f;
}
static float adjThreshUp(float v) {
    if(v<0.001f) return 0.001f;
    if(v<0.1f)   return v+0.001f;
    if(v<1)      return v+0.01f;
    if(v<10)     return v+0.1f;
    return v+1.0f;
}

static const int LG_ROW_H = 36;
static const int LG_ROW0  = HDR_H + 2;

// Fills rowMap[] with channel index (0-7) or diff index+8 (8=d0, 9=d1); returns count.
static int buildLogRowMap(int* rowMap) {
    int n=0;
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        if(rowMap) rowMap[n]=ch; n++;
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        if(rowMap) rowMap[n]=8+d; n++;
    }
    return n;
}

// ── Numpad popup ──────────────────────────────────────────────────────────────
static const int NP_X     = 160;
static const int NP_Y     = 44;
static const int NP_W     = 480;
static const int NP_BTN_W = 144;
static const int NP_BTN_H = 52;
static const int NP_COL0  = NP_X + 16;
static const int NP_COL1  = NP_COL0 + NP_BTN_W + 8;
static const int NP_COL2  = NP_COL1 + NP_BTN_W + 8;
static const int NP_ROW0  = NP_Y + 92;
static const int NP_ROW1  = NP_ROW0 + 58;
static const int NP_ROW2  = NP_ROW1 + 58;
static const int NP_ROW3  = NP_ROW2 + 58;
static const int NP_ROWOK = NP_ROW3 + 58;

static void drawNumpad() {
    int h = NP_ROWOK + NP_BTN_H + 8 - NP_Y;
    gfx.fillRoundRect(NP_X+4, NP_Y+4, NP_W, h, 8, C_BORDER);
    gfx.fillRoundRect(NP_X,   NP_Y,   NP_W, h, 8, C_PANEL);
    gfx.drawRoundRect(NP_X,   NP_Y,   NP_W, h, 8, C_CYAN);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    gfx.setCursor(NP_X+16, NP_Y+10); gfx.print(numpadTitle);
    gfx.fillRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_BG);
    gfx.drawRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_CYAN);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(3);
    const char* disp = numpadBuf[0] ? numpadBuf : "0";
    int vw = strlen(disp)*18;
    gfx.setCursor(NP_X+16+(NP_W-32-vw)/2, NP_Y+48); gfx.print(disp);
    static const char* NP_LABELS[12] = {"7","8","9","4","5","6","1","2","3",".","0","<-"};
    for(int r=0;r<4;r++){
        int ry = NP_ROW0 + r*58;
        for(int c=0;c<3;c++){
            bool isDec = (r==3 && c==0);
            bool disabled = isDec && (numpadTarget==10 || strchr(numpadBuf,'.')!=nullptr);
            drawBtn(NP_COL0+c*(NP_BTN_W+8), ry, NP_BTN_W, NP_BTN_H,
                    disabled ? C_BORDER : 0x3186u, C_TEXT, NP_LABELS[r*3+c], 3);
        }
    }
    drawBtn(NP_COL0, NP_ROWOK, NP_BTN_W,           NP_BTN_H, C_RED,   C_TEXT, "Cancel", 2);
    drawBtn(NP_COL1, NP_ROWOK, 2*NP_BTN_W+8,       NP_BTN_H, C_GREEN, C_TEXT, "OK",     3);
}

static void openNumpad(int target, const char* title, float curVal) {
    numpadTarget = target;
    strncpy(numpadTitle, title, sizeof(numpadTitle)-1);
    numpadTitle[sizeof(numpadTitle)-1] = '\0';
    if(target == 10)
        snprintf(numpadBuf, sizeof(numpadBuf), "%d", (int)(curVal+0.5f));
    else {
        if(curVal <= 0) numpadBuf[0] = '\0';
        else snprintf(numpadBuf, sizeof(numpadBuf), "%.5g", (double)curVal);
    }
    numpadActive = true;
    drawNumpad();
}

static void handleNumpadTouch(int tx, int ty) {
    if(hit(NP_COL0, NP_ROWOK, NP_BTN_W, NP_BTN_H, tx, ty)){
        numpadActive = false; drawLoggerScreen(); return;
    }
    if(hit(NP_COL1, NP_ROWOK, 2*NP_BTN_W+8, NP_BTN_H, tx, ty)){
        float v = numpadBuf[0] ? (float)atof(numpadBuf) : 0.0f;
        if(numpadTarget == 10)
            logIntervalMs = constrain((int)(v+0.5f), 100, 60000);
        else if(numpadTarget >= 8)
            logDiffThresh[numpadTarget-8] = max(0.0f, v);
        else
            logThresh[numpadTarget] = max(0.0f, v);
        numpadActive = false; drawLoggerScreen(); return;
    }
    static const char NP_KEYS[12] = {'7','8','9','4','5','6','1','2','3','.','0','\x08'};
    for(int r=0;r<4;r++){
        int ry = NP_ROW0 + r*58;
        for(int c=0;c<3;c++){
            if(!hit(NP_COL0+c*(NP_BTN_W+8), ry, NP_BTN_W, NP_BTN_H, tx, ty)) continue;
            char k = NP_KEYS[r*3+c];
            int n = strlen(numpadBuf);
            if(k == '\x08'){
                if(n > 0) numpadBuf[n-1] = '\0';
            } else if(k == '.'){
                if(numpadTarget==10 || strchr(numpadBuf,'.')!=nullptr) return;
                if(n < 11){ numpadBuf[n]='.'; numpadBuf[n+1]='\0'; }
            } else {
                if(n < 11){ numpadBuf[n]=k; numpadBuf[n+1]='\0'; }
            }
            // Refresh input field + decimal button (enabled state may change)
            gfx.fillRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_BG);
            gfx.drawRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_CYAN);
            gfx.setTextColor(C_YELLOW); gfx.setTextSize(3);
            const char* disp = numpadBuf[0] ? numpadBuf : "0";
            int vw = strlen(disp)*18;
            gfx.setCursor(NP_X+16+(NP_W-32-vw)/2, NP_Y+48); gfx.print(disp);
            bool disabled = (numpadTarget==10 || strchr(numpadBuf,'.')!=nullptr);
            drawBtn(NP_COL0, NP_ROW3, NP_BTN_W, NP_BTN_H,
                    disabled ? C_BORDER : 0x3186u, C_TEXT, ".", 3);
            return;
        }
    }
}

static void drawLoggerScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Datalogger Setup");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_BORDER,C_TEXT,"< Back",2);

    int rowMap[10]; int nRows=buildLogRowMap(rowMap);
    for(int row=0;row<nRows;row++){
        int ry=LG_ROW0+row*LG_ROW_H;
        gfx.fillRect(0,ry,DISP_W,LG_ROW_H,(row&1)?C_PANEL:C_BG);
        int idx=rowMap[row]; bool isDiff=(idx>=8); int didx=idx-8;
        uint16_t col = isDiff ? DIFF_COLORS[didx] : CH_COLORS[idx];
        const char* unit = isDiff ? (dDiff[didx].typ==1?"V":"A") : (dChanType[idx]==1?"V":"A");
        float thresh = isDiff ? logDiffThresh[didx] : logThresh[idx];
        char lb[20];
        if(isDiff)
            snprintf(lb,sizeof(lb),"D%d %s-%s(%s)",didx+1,CH_NAMES[dDiff[didx].pos],CH_NAMES[dDiff[didx].neg],dDiff[didx].typ==1?"V":"A");
        else
            snprintf(lb,sizeof(lb),"%s(%s)",CH_NAMES[idx],dChanType[idx]==1?"V":"A");
        gfx.fillRect(4,ry+8,12,20,col);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        gfx.setCursor(20,ry+(LG_ROW_H-16)/2); gfx.print(lb);
        drawBtn(244,ry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"-",3);
        char vb[16];
        if(thresh<=0) snprintf(vb,sizeof(vb),"always"); else snprintf(vb,sizeof(vb),"%.4g",thresh);
        gfx.fillRoundRect(292,ry+3,130,LG_ROW_H-6,3,C_BG);
        gfx.drawRoundRect(292,ry+3,130,LG_ROW_H-6,3,C_BORDER);
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        int vw=strlen(vb)*12; gfx.setCursor(292+(130-vw)/2,ry+(LG_ROW_H-16)/2); gfx.print(vb);
        drawBtn(426,ry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"+",3);
        gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
        gfx.setCursor(476,ry+(LG_ROW_H-8)/2); gfx.print(unit);
    }
    // Interval row
    int iry=LG_ROW0+nRows*LG_ROW_H+4;
    gfx.fillRect(0,iry,DISP_W,LG_ROW_H,(nRows&1)?C_PANEL:C_BG);
    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(20,iry+(LG_ROW_H-16)/2); gfx.print("Max interval (ms)");
    drawBtn(244,iry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"-",3);
    char ib[16]; snprintf(ib,sizeof(ib),"%d",logIntervalMs);
    gfx.fillRoundRect(292,iry+3,130,LG_ROW_H-6,3,C_BG);
    gfx.drawRoundRect(292,iry+3,130,LG_ROW_H-6,3,C_BORDER);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    int iw=strlen(ib)*12; gfx.setCursor(292+(130-iw)/2,iry+(LG_ROW_H-16)/2); gfx.print(ib);
    drawBtn(426,iry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"+",3);
    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(476,iry+(LG_ROW_H-8)/2); gfx.print("ms (fallback)");
    // Start button
    char sbuf[52];
    if(nRows==0)
        snprintf(sbuf,sizeof(sbuf),"Select channels on main screen");
    else if(usbMounted){
        int n=nextLogFileNum();
        snprintf(sbuf,sizeof(sbuf),"START -> logger_%03d.csv",n);
    } else
        snprintf(sbuf,sizeof(sbuf),"START (mount USB first)");
    drawBtn(4,DISP_H-52,DISP_W-8,48,(nRows>0&&usbMounted)?C_GREEN:C_BORDER,C_TEXT,sbuf,2);
}

static void updateLoggingDisplay() {
    int rx=DISP_W/2+8;
    char buf[48];
    gfx.fillRect(rx,HDR_H+20,360,24,C_BG);
    gfx.setTextColor(C_GREEN); gfx.setTextSize(2);
    snprintf(buf,sizeof(buf),"%d rows",logRowCount);
    gfx.setCursor(rx,HDR_H+20); gfx.print(buf);
    gfx.fillRect(rx,HDR_H+60,360,24,C_BG);
    uint32_t eSec=(millis()-logStartMs)/1000;
    snprintf(buf,sizeof(buf),"%02lu:%02lu:%02lu",(unsigned long)(eSec/3600),
             (unsigned long)((eSec%3600)/60),(unsigned long)(eSec%60));
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    gfx.setCursor(rx,HDR_H+60); gfx.print(buf);
    int ry=HDR_H+10;
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        gfx.fillRect(160,ry,180,20,C_BG);
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        snprintf(buf,sizeof(buf),"%.5g",(double)logCurCh[ch]);
        gfx.setCursor(160,ry); gfx.print(buf);
        ry+=28;
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        gfx.fillRect(160,ry,180,20,C_BG);
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        snprintf(buf,sizeof(buf),"%.5g",(double)logCurDiff[d]);
        gfx.setCursor(160,ry); gfx.print(buf);
        ry+=28;
    }
}

static void drawLoggingScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,0x0360u);  // dark green header
    gfx.setTextColor(C_TEXT); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Logging...");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_RED,C_TEXT,"STOP",3);
    gfx.drawFastVLine(DISP_W/2,HDR_H,DISP_H-HDR_H,C_BORDER);
    int rx=DISP_W/2+8;
    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(rx,HDR_H+8);  gfx.print("ROWS WRITTEN");
    gfx.setCursor(rx,HDR_H+48); gfx.print("ELAPSED");
    gfx.setCursor(rx,HDR_H+88); gfx.print("FILE");
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    char fnbuf[32]; snprintf(fnbuf,sizeof(fnbuf),"logger_%03d.csv",logFileNum);
    gfx.setCursor(rx,HDR_H+100); gfx.print(fnbuf);
    gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
    char sbuf[64];
    snprintf(sbuf,sizeof(sbuf),"%lu Hz  %d-bit  smooth:%d",
             (unsigned long)RATE_OPTIONS[dRateIdx],dBits,max(1,dSmooth));
    gfx.setCursor(rx,HDR_H+130); gfx.print(sbuf);
    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    snprintf(sbuf,sizeof(sbuf),"Interval fallback: %d ms",logIntervalMs);
    gfx.setCursor(rx,HDR_H+145); gfx.print(sbuf);
    // Channel labels (left column)
    int ry=HDR_H+10;
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        gfx.fillRect(4,ry,12,16,CH_COLORS[ch]);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        char lb[16]; snprintf(lb,sizeof(lb),"%s(%s):",CH_NAMES[ch],dChanType[ch]==1?"V":"A");
        gfx.setCursor(20,ry); gfx.print(lb);
        ry+=28;
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        gfx.fillRect(4,ry,12,16,DIFF_COLORS[d]);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        char lb[16]; snprintf(lb,sizeof(lb),"D%d(%s):",d+1,dDiff[d].typ==1?"V":"A");
        gfx.setCursor(20,ry); gfx.print(lb);
        ry+=28;
    }
    updateLoggingDisplay();
}

static void startLogger() {
    if(!usbMounted){
        showOverlay(C_YELLOW,"Mounting USB...");
        tryMountUSB();
    }
    if(!usbMounted){ showOverlay(C_RED,"USB not found!"); delay(2000); drawLoggerScreen(); return; }
    logFileNum=nextLogFileNum();
    char fname[32]; snprintf(fname,sizeof(fname),"/usb/logger_%03d.csv",logFileNum);
    logFile=fopen(fname,"w");
    if(!logFile){ showOverlay(C_RED,"File open failed!"); delay(1500); drawLoggerScreen(); return; }
    fprintf(logFile,"time_us");
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        fprintf(logFile,",%s(%s)",CH_NAMES[ch],dChanType[ch]==1?"V":"A");
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        fprintf(logFile,",D%d=%s-%s(%s)",d+1,CH_NAMES[dDiff[d].pos],CH_NAMES[dDiff[d].neg],dDiff[d].typ==1?"V":"A");
    }
    fprintf(logFile,"\n");
    logRowCount=0; logStartMs=millis(); logStartUs=micros(); logLastWriteMs=0;
    for(int ch=0;ch<N_CHANNELS;ch++){logLastCh[ch]=1e30f;logCurCh[ch]=0;}
    for(int d=0;d<2;d++){logLastDiff[d]=1e30f;logCurDiff[d]=0;}
    logLastDispMs=0;

    // The logger rides on the engine's stream mode: period statistics drive the
    // on-screen readout and the "log at least every N ms" rule, while armed
    // per-channel thresholds make the engine forward the frames that moved.
    //
    // The thresholds the user types are in engineering units, so they are
    // converted to raw counts for the engine — a cheap pre-filter that never
    // discards a frame the exact test below would have kept. The exact
    // engineering-unit comparison still happens here, on every forwarded frame.
    int32_t trig[N_CHANNELS];
    for(int ch=0;ch<N_CHANNELS;ch++) trig[ch]=0;
    uint32_t mask=0;
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        mask|=(1u<<ch);
        // A zero threshold means "log everything" — arm at 1 count, the finest
        // the ADC can express.
        float sc=fabsf(dChanScale[ch]); if(sc<1e-9f) sc=1.0f;
        int32_t raw=(logThresh[ch]>0)?(int32_t)floorf(logThresh[ch]/(vStep(dBits)*sc)):1;
        if(raw<1) raw=1;
        trig[ch]=raw;
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        mask|=(1u<<dDiff[d].pos)|(1u<<dDiff[d].neg);
        float sc=fabsf(dDiff[d].scl); if(sc<1e-9f) sc=1.0f;
        int32_t raw=(logDiffThresh[d]>0)?(int32_t)floorf(logDiffThresh[d]/(vStep(dBits)*sc)):1;
        if(raw<1) raw=1;
        // A differential threshold is armed on both legs: either one moving is
        // enough to forward the frame, which is conservative (it can forward a
        // common-mode shift the exact test then rejects).
        for(int leg=0;leg<2;leg++){
            int ch=leg?dDiff[d].neg:dDiff[d].pos;
            if(trig[ch]==0||raw<trig[ch]) trig[ch]=raw;
        }
    }

    // 250 ms periods keep the readout lively regardless of the max-interval
    // setting, which is enforced here rather than by the engine.
    if(!hostIssue(GIGA_CMD_STREAM, RATE_OPTIONS[dRateIdx], dBits, max(1,dSmooth),
                  0, 0, mask, mask, trig, false, 250)){
        fclose(logFile); logFile=nullptr;
        showOverlay(C_RED, engineUp?"ADC start failed!":"M4 not running!");
        delay(2000); drawLoggerScreen(); return;
    }
    logRunning=true;
    streamSeen=0;
    currentScreen=SCR_LOGGING; drawLoggingScreen();
}

static void stopLoggerQuiet() {
    logRunning=false;
    hostStop();
    if(logFile){ fflush(logFile); fclose(logFile); logFile=nullptr; }
    usbFs.unmount(); usbMounted=false;
}

static void stopLogger() {
    stopLoggerQuiet();
    currentScreen=SCR_SETTINGS; drawSettingsScreen();
}

// Write one CSV row from a set of smoothed raw counts.
static void loggerWriteRow(const uint16_t* sv, uint32_t now) {
    if(!logFile) return;
    float ev[N_CHANNELS];
    for(int ch=0;ch<N_CHANNELS;ch++)
        ev[ch]=dChanType[ch]?calChan(sv[ch],dChanOffset[ch],dChanScale[ch],dBits):0.0f;
    float ed[2];
    for(int d=0;d<2;d++)
        ed[d]=dDiff[d].en?calDiff(sv[dDiff[d].pos],sv[dDiff[d].neg],dDiff[d].off,dDiff[d].scl,dBits):0.0f;
    for(int ch=0;ch<N_CHANNELS;ch++) if(dChanType[ch]) logCurCh[ch]=ev[ch];
    for(int d=0;d<2;d++) if(dDiff[d].en) logCurDiff[d]=ed[d];

    fprintf(logFile,"%lu",(unsigned long)(micros()-logStartUs));
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        fprintf(logFile,",%.5g",(double)ev[ch]);
        logLastCh[ch]=ev[ch];
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        fprintf(logFile,",%.5g",(double)ed[d]);
        logLastDiff[d]=ed[d];
    }
    fprintf(logFile,"\n");
    logRowCount++;
    logLastWriteMs=now;
    if(logRowCount%50==0) fflush(logFile);
}

static void loggerStep() {
    if(!logRunning) return;
    hostPumpEngine();
    hostReadStatus();

    // Threshold-triggered frames arrive through the ring. Timestamps are taken
    // when the frame is drained rather than when it was sampled — the same
    // one-DMA-buffer lag the single-core version had.
    uint32_t head=shm->e.ringHead;
    while(ringTail!=head){
        const volatile uint16_t* src=shm->ring[ringTail % SHM_RING_FRAMES];
        shmInvalidate(src,sizeof(uint16_t)*SHM_N_CH);
        uint16_t sv[N_CHANNELS];
        for(int ch=0;ch<N_CHANNELS;ch++) sv[ch]=src[ch];
        ringTail++;

        // Exact test, in engineering units, on the frames the engine forwarded.
        bool doLog=false;
        for(int ch=0;ch<N_CHANNELS&&!doLog;ch++){
            if(!dChanType[ch]) continue;
            float v=calChan(sv[ch],dChanOffset[ch],dChanScale[ch],dBits);
            if(logThresh[ch]<=0||fabsf(v-logLastCh[ch])>=logThresh[ch]) doLog=true;
        }
        for(int d=0;d<2&&!doLog;d++){
            if(!dDiff[d].en) continue;
            float v=calDiff(sv[dDiff[d].pos],sv[dDiff[d].neg],dDiff[d].off,dDiff[d].scl,dBits);
            if(logDiffThresh[d]<=0||fabsf(v-logLastDiff[d])>=logDiffThresh[d]) doLog=true;
        }
        if(doLog) loggerWriteRow(sv,millis());
    }
    shm->h.ringTail=ringTail;
    shmClean(&shm->h,sizeof(ShmToEngine));

    // Period boundary: refresh the readout and honour the max-interval rule
    // even when nothing has moved.
    if(shm->e.statSeq!=streamSeen && hostSnapshotStats()){
        streamSeen=statSnap.statSeq;
        uint16_t sv[N_CHANNELS];
        for(int ch=0;ch<N_CHANNELS;ch++) sv[ch]=(uint16_t)statSnap.sLast[ch];
        uint32_t now=millis();
        if(now-logLastWriteMs>=(uint32_t)logIntervalMs) loggerWriteRow(sv,now);
        else {
            for(int ch=0;ch<N_CHANNELS;ch++)
                if(dChanType[ch]) logCurCh[ch]=calChan(sv[ch],dChanOffset[ch],dChanScale[ch],dBits);
            for(int d=0;d<2;d++)
                if(dDiff[d].en) logCurDiff[d]=calDiff(sv[dDiff[d].pos],sv[dDiff[d].neg],
                                                      dDiff[d].off,dDiff[d].scl,dBits);
        }
        if(now-logLastDispMs>=500){ logLastDispMs=now; updateLoggingDisplay(); }
    }
}

// ── USB export ────────────────────────────────────────────────────────────────
static void tryMountUSB() {
    pinMode(PA_15,OUTPUT); digitalWrite(PA_15,HIGH); delay(300);
    usbMounted=false;
    if(msd.connect() && usbFs.mount(&msd)==0) usbMounted=true;
}
static void showOverlay(uint16_t col, const char* msg) {
    gfx.fillRect(20,DISP_H/2-20,DISP_W-40,40,C_PANEL);
    gfx.setTextColor(col); gfx.setTextSize(2);
    gfx.setCursor(24,DISP_H/2-8); gfx.print(msg);
}
static void exportToUSB() {
    showOverlay(C_YELLOW,"Mounting USB...");
    tryMountUSB();
    if(!usbMounted){ showOverlay(C_RED,"USB not found!"); delay(1500); drawPlotScreen(); return; }
    FILE* f=fopen("/usb/capture.csv","w");
    if(!f){ showOverlay(C_RED,"File open failed!"); delay(1500); drawPlotScreen(); return; }

    // Header
    fprintf(f,"Time (ms)");
    for(int ch=0;ch<N_CHANNELS;ch++)
        if(lastPinReq[ch]&&lastChanType[ch]!=0)
            fprintf(f,",%s (%s)",CH_NAMES[ch],lastChanType[ch]==1?"V":"I");
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(dc.en) fprintf(f,",D%d=%s-%s (%s)",d+1,CH_NAMES[dc.pos],CH_NAMES[dc.neg],dc.typ==1?"V":"I");
    }
    fprintf(f,"\n");

    int maxCnt=1;
    for(int ch=0;ch<N_CHANNELS;ch++) if(lastPinReq[ch]&&lastLogCnt[ch]>maxCnt) maxCnt=lastLogCnt[ch];

    for(int j=0;j<maxCnt;j++) {
        double t_ms=(maxCnt>1)?(double)j*lastElapsedMs/(maxCnt-1):0.0;
        fprintf(f,"%.5g",t_ms);
        for(int ch=0;ch<N_CHANNELS;ch++) {
            if(!lastPinReq[ch]||lastChanType[ch]==0) continue;
            if(j<lastLogCnt[ch]) {
                float v=calChan(getRawVal(ch,j),lastChanOffset[ch],lastChanScale[ch],lastBitRes);
                fprintf(f,",%.5g",(double)v);
            } else fprintf(f,",");
        }
        for(int d=0;d<2;d++) {
            const DiffCfg& dc=lastDiff[d];
            if(!dc.en) continue;
            int cnt=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]);
            if(j<cnt) {
                float v=calDiff(getRawVal(dc.pos,j),getRawVal(dc.neg,j),dc.off,dc.scl,lastBitRes);
                fprintf(f,",%.5g",(double)v);
            } else fprintf(f,",");
        }
        fprintf(f,"\n");
    }
    fclose(f); usbFs.unmount(); usbMounted=false;
    showOverlay(C_GREEN,"Saved: /usb/capture.csv");
    delay(2000); drawPlotScreen();
}

// ── Display-mode capture ──────────────────────────────────────────────────────
// Run a capture on behalf of the touchscreen. The engine does the acquisition;
// this only pumps the ring and paints the progress bar, so the display keeps
// updating at full rate even at 1 MS/s.
static bool hostRunCapture(uint32_t rate, int bits, uint32_t smooth,
                           uint32_t durMs, uint32_t logTarget,
                           const bool* capMask, const bool* statMask,
                           const int32_t* trig, bool midMode,
                           bool showProgress) {
    uint32_t cm = 0, sm = 0;
    for (int i = 0; i < N_CHANNELS; i++) {
        hPinReq[i] = capMask[i];
        if (capMask[i])  cm |= (1u << i);
        if (statMask[i]) sm |= (1u << i);
    }
    hLogTarget    = (int)logTarget;
    hLogCnt       = 0;
    hMidMode      = midMode;
    hMidPreSize   = max(1, (int)logTarget / 2);
    hMidPostSize  = (int)logTarget - hMidPreSize;
    hMidRingHead  = 0;
    hMidRingFill  = 0;
    hMidPostCount = 0;

    if (!hostIssue(GIGA_CMD_CAPTURE, rate, bits, smooth, durMs, logTarget,
                   cm, sm, trig, midMode, 0))
        return false;

    // A midpoint capture waits for its trigger, so it gets no deadline — only
    // an explicit abort ends it.
    uint32_t deadline = millis() + durMs + 5000;
    uint32_t lastDisp = 0;
    uint32_t t0 = millis();
    while (true) {
        hostPumpEngine();
        hostDrainRing();
        if (shm->e.state == GIGA_ST_CAPTURE_DONE) break;
        if (shm->e.state == GIGA_ST_ERROR) return false;
        if (!midMode && (int32_t)(millis() - deadline) > 0) break;
        if (showProgress && displayPresent) {
            uint32_t now = millis();
            if (now - lastDisp > 80) {
                lastDisp = now;
                updateProgress(min(now - t0, durMs), durMs);
            }
        }
    }
    hostDrainRing();     // anything the engine published as it finished
    return true;
}

static void runDisplayCapture() {
    if(!contMode) drawCaptureScreen();

    uint32_t sampleRate = RATE_OPTIONS[dRateIdx];
    int bitRes=dBits, duration=dTime, smoothCount=max(1,dSmooth);
    int logTarget=min(dPoints,MAX_LOG), trigThr=dTrigThr;

    // Build capture mask — standalone active + diff source pins
    bool doCapt[N_CHANNELS]={};
    for(int i=0;i<N_CHANNELS;i++) if(dChanType[i]) doCapt[i]=true;
    for(int d=0;d<2;d++) {
        if(dDiff[d].en){ doCapt[dDiff[d].pos]=true; doCapt[dDiff[d].neg]=true; }
    }
    bool statMask[N_CHANNELS];
    for(int i=0;i<N_CHANNELS;i++) statMask[i]=(dChanType[i]!=0);

    // The on-screen trigger threshold is one value applied to every displayed
    // channel; the serial protocol allows a different one per channel.
    int32_t trig[N_CHANNELS];
    for(int i=0;i<N_CHANNELS;i++) trig[i]=(trigThr>0&&dChanType[i])?trigThr:0;

    if(!hostRunCapture(sampleRate,bitRes,smoothCount,duration,logTarget,
                       doCapt,statMask,trig,false,!contMode)) {
        showOverlay(C_RED, engineUp?"ADC start failed!":"M4 not running!");
        delay(2000);
        contMode=false;
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }

    // Save post-capture state
    for(int i=0;i<N_CHANNELS;i++) {
        lastPinReq[i]    = doCapt[i];
        lastLogCnt[i]    = doCapt[i] ? hLogCnt : 0;
        lastChanType[i]  = dChanType[i];
        lastChanOffset[i]= dChanOffset[i];
        lastChanScale[i] = dChanScale[i];
    }
    memcpy(lastDiff,dDiff,sizeof(dDiff));
    lastBitRes    = bitRes;
    lastElapsedMs = shm->e.elapsedMs;
    lastMidMode   = false;
    captureReady  = true;

    // Serial mirror (raw counts, Python GUI compatible)
    for(int i=0;i<N_CHANNELS;i++) {
        if(!dChanType[i]||shm->e.cCount[i]==0) continue;
        double mean,rms,stdDev; hostChanStats(i,mean,rms,stdDev);
        Serial.print("PIN:");    Serial.print(CH_NAMES[i]);
        Serial.print("|BITS:");  Serial.print(bitRes);
        Serial.print("|COUNT:"); Serial.print(shm->e.cCount[i]);
        Serial.print("|MIN:");   Serial.print(shm->e.cMin[i]);
        Serial.print("|MAX:");   Serial.print(shm->e.cMax[i]);
        Serial.print("|MEAN:");  Serial.print(mean,4);
        Serial.print("|RMS:");   Serial.print(rms,4);
        Serial.print("|STD:");   Serial.print(stdDev,4);
        Serial.print("|LOGGED:");Serial.println(hLogCnt);
    }
    for(int i=0;i<N_CHANNELS;i++) {
        if(!dChanType[i]||hLogCnt==0) continue;
        Serial.print("DATA:"); Serial.print(CH_NAMES[i]); Serial.print("|VALS:");
        for(int j=0;j<hLogCnt;j++) {
            Serial.print(logData[i][j]);
            if(j<hLogCnt-1) Serial.print(',');
        }
        Serial.println();
    }
    Serial.print("DONE|ELAPSED:"); Serial.println(lastElapsedMs);

    tryMountUSB();
    currentScreen=SCR_PLOT;
    drawPlotScreen();
}

// ── Touch handlers ────────────────────────────────────────────────────────────
static void handleSettingsTouch(int tx,int ty) {
    // Presets / Logger buttons
    if(hit(DISP_W-164,2,78,HDR_H-4,tx,ty)){
        currentScreen=SCR_PRESETS; drawPresetsScreen(); return;
    }
    if(hit(DISP_W-82,2,78,HDR_H-4,tx,ty)){
        contMode=false;
        currentScreen=SCR_LOGGER; drawLoggerScreen(); return;
    }
    // Channel type cycle (off→V→I→off)
    int cbw=DISP_W/N_CHANNELS;
    for(int i=0;i<N_CHANNELS;i++) {
        if(hit(i*cbw+2,CH_ROW_Y,cbw-4,CH_ROW_H-4,tx,ty)) {
            dChanType[i]=(dChanType[i]+1)%3;
            char lbl[8];
            if(dChanType[i]==0)      snprintf(lbl,sizeof(lbl),"%s",CH_NAMES[i]);
            else if(dChanType[i]==1) snprintf(lbl,sizeof(lbl),"%s V",CH_NAMES[i]);
            else                     snprintf(lbl,sizeof(lbl),"%s I",CH_NAMES[i]);
            drawBtn(i*cbw+2,CH_ROW_Y,cbw-4,CH_ROW_H-4,
                    dChanType[i]?CH_COLORS[i]:C_PANEL, C_TEXT, lbl, 2);
            drawCaptureButtons(); return;
        }
    }
    // Param +/-
    for(int r=0;r<N_PARAMS;r++) {
        int ry=PARAM_Y+r*PARAM_ROW, bx=PL_X0+PL_LABEL_W;
        bool changed=false;
        if(hit(bx,ry+2,PL_BTN_W,PARAM_ROW-4,tx,ty)) {
            switch(r){
                case 0:if(dRateIdx>0)dRateIdx--;break;
                case 1:if(dBits>8)dBits-=2;break;
                case 2:dTime=max(1,dTime-(dTime>1000?100:dTime>100?10:1));break;
                case 3:if(dSmooth>1)dSmooth--;break;
                case 4:dPoints=max(10,dPoints-(dPoints>1000?500:dPoints>100?100:10));break;
                case 5:dTrigThr=max(0,dTrigThr-(dTrigThr>100?100:dTrigThr>10?10:1));break;
                case 6:if(dStreamHzIdx>0)dStreamHzIdx--;break;
            }
            changed=true;
        } else if(hit(bx+PL_BTN_W+PL_VAL_W,ry+2,PL_BTN_W,PARAM_ROW-4,tx,ty)) {
            switch(r){
                case 0:if(dRateIdx<N_RATE_OPT-1)dRateIdx++;break;
                case 1:if(dBits<16)dBits+=2;break;
                case 2:dTime=min(60000,dTime+(dTime>=1000?100:dTime>=100?10:1));break;
                case 3:if(dSmooth<64)dSmooth++;break;
                case 4:dPoints=min(MAX_LOG,dPoints+(dPoints>=1000?500:dPoints>=100?100:10));break;
                case 5:dTrigThr=min(4095,dTrigThr+(dTrigThr>=100?100:dTrigThr>=10?10:1));break;
                case 6:if(dStreamHzIdx<N_STREAM_HZ-1)dStreamHzIdx++;break;
            }
            changed=true;
        }
        if(changed){refreshParamValue(r);return;}
    }
    // Diff button
    if(hit(4,DIFF_BTN_Y,DISP_W-8,DIFF_BTN_H,tx,ty)){
        currentScreen=SCR_DIFF; drawDiffScreen(); return;
    }
    // Capture / Continuous buttons
    bool anyActive=false;
    for(int i=0;i<N_CHANNELS;i++) if(dChanType[i]) anyActive=true;
    if(dDiff[0].en||dDiff[1].en) anyActive=true;
    if(anyActive && hit(4,CAP_BTN_Y,CAP_W,CAP_BTN_H,tx,ty)){
        contMode=false;
        currentScreen=SCR_CAPTURING; runDisplayCapture();
    }
    if(anyActive && hit(CONT_X,CAP_BTN_Y,CONT_W,CAP_BTN_H,tx,ty)){
        contMode=true; lastContMs=0;
        runDisplayCapture();
        return;
    }
    if(anyActive && hit(LIVE_X,CAP_BTN_Y,LIVE_W,CAP_BTN_H,tx,ty)){
        // Screen-only live view: every channel with a type set, plus every
        // enabled differential pair as its own readout.
        bool sel[SHM_N_STAT]={};
        for(int i=0;i<N_CHANNELS;i++) if(dChanType[i]) sel[i]=true;
        for(int d=0;d<SHM_N_DIFF;d++) if(dDiff[d].en) sel[N_CHANNELS+d]=true;
        if(!startStream(nullptr,RATE_OPTIONS[dRateIdx],dBits,max(1,dSmooth),
                        STREAM_HZ_OPTS[dStreamHzIdx],sel,nullptr,nullptr)) {
            showOverlay(C_RED, engineUp?"ADC start failed!":"M4 not running!");
            delay(2000); drawSettingsScreen();
        }
    }
}

static void handlePresetsTouch(int tx,int ty) {
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)){
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    int n = min(N_PRESETS, PRE_COLS * PRE_ROWS);
    for(int i=0;i<n;i++) {
        int bx = (i % PRE_COLS) * PRE_CW + 2;
        int by = HDR_H + (i / PRE_COLS) * PRE_RH + 1;
        if(hit(bx, by, PRE_BTN_W, PRE_BTN_H, tx, ty)){
            applyPreset(i);
            currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
        }
    }
}

static void handleDiffTouch(int tx,int ty) {
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)){
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    for(int d=0;d<2;d++) {
        int ry=60+d*110; DiffCfg& dc=dDiff[d];
        if(hit(10,ry,160,50,tx,ty)){ dc.en=!dc.en; drawDiffRow(d); return; }
        if(hit(228,ry,44,50,tx,ty)){ dc.pos=(dc.pos+N_CHANNELS-1)%N_CHANNELS; drawDiffRow(d); return; }
        if(hit(322,ry,44,50,tx,ty)){ dc.pos=(dc.pos+1)%N_CHANNELS;            drawDiffRow(d); return; }
        if(hit(428,ry,44,50,tx,ty)){ dc.neg=(dc.neg+N_CHANNELS-1)%N_CHANNELS; drawDiffRow(d); return; }
        if(hit(522,ry,44,50,tx,ty)){ dc.neg=(dc.neg+1)%N_CHANNELS;            drawDiffRow(d); return; }
        if(hit(588,ry,88,50,tx,ty)){ dc.typ=1; drawDiffRow(d); return; }
        if(hit(684,ry,88,50,tx,ty)){ dc.typ=2; drawDiffRow(d); return; }
    }
}

static void handlePlotTouch(int tx,int ty) {
    if(hit(DISP_W-220,2,100,HDR_H-4,tx,ty)){
        contMode=false; currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)) exportToUSB();
}

static void handleLoggerTouch(int tx,int ty) {
    if(numpadActive){ handleNumpadTouch(tx,ty); return; }
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)){
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    int rowMap[10]; int nRows=buildLogRowMap(rowMap);
    for(int row=0;row<nRows;row++){
        int ry=LG_ROW0+row*LG_ROW_H;
        int idx=rowMap[row]; bool isDiff=(idx>=8); int didx=idx-8;
        if(hit(244,ry+3,44,LG_ROW_H-6,tx,ty)){
            if(isDiff) logDiffThresh[didx]=adjThreshDn(logDiffThresh[didx]);
            else       logThresh[idx]=adjThreshDn(logThresh[idx]);
            drawLoggerScreen(); return;
        }
        if(hit(426,ry+3,44,LG_ROW_H-6,tx,ty)){
            if(isDiff) logDiffThresh[didx]=adjThreshUp(logDiffThresh[didx]);
            else       logThresh[idx]=adjThreshUp(logThresh[idx]);
            drawLoggerScreen(); return;
        }
        if(hit(292,ry,130,LG_ROW_H,tx,ty)){
            char title[24];
            if(isDiff) snprintf(title,sizeof(title),"D%d delta thresh",didx+1);
            else       snprintf(title,sizeof(title),"%s delta thresh",CH_NAMES[idx]);
            openNumpad(isDiff?8+didx:idx, title, isDiff?logDiffThresh[didx]:logThresh[idx]);
            return;
        }
    }
    int iry=LG_ROW0+nRows*LG_ROW_H+4;
    if(hit(244,iry+3,44,LG_ROW_H-6,tx,ty)){
        logIntervalMs=max(100,logIntervalMs-(logIntervalMs>5000?1000:logIntervalMs>1000?500:100));
        drawLoggerScreen(); return;
    }
    if(hit(426,iry+3,44,LG_ROW_H-6,tx,ty)){
        logIntervalMs=min(60000,logIntervalMs+(logIntervalMs>=5000?1000:logIntervalMs>=1000?500:100));
        drawLoggerScreen(); return;
    }
    if(hit(292,iry,130,LG_ROW_H,tx,ty)){
        openNumpad(10,"Max interval (ms)",(float)logIntervalMs); return;
    }
    if(hit(4,DISP_H-52,DISP_W-8,48,tx,ty)){
        if(nRows>0) startLogger();
    }
}

static void handleLoggingTouch(int tx,int ty) {
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)) stopLogger();
}

// ── WiFi bring-up ─────────────────────────────────────────────────────────────
static void processCommand(Stream& io);   // fwd decl (Stream& param)

static void connectWiFi() {
    strcpy(wifiIPStr, "connecting...");
    if(WiFi.status() == WL_NO_MODULE){ strcpy(wifiIPStr,"no module"); return; }
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while(WiFi.status() != WL_CONNECTED && millis()-t0 < 15000) delay(300);
    if(WiFi.status() == WL_CONNECTED){
        IPAddress ip = WiFi.localIP();
        snprintf(wifiIPStr, sizeof(wifiIPStr), "%d.%d.%d.%d:%u",
                 ip[0], ip[1], ip[2], ip[3], (unsigned)CMD_PORT);
        cmdServer.begin();
        wifiActive = true;
        Serial.print("WiFi ready at "); Serial.println(wifiIPStr);
    } else {
        strcpy(wifiIPStr, "wifi failed");
        Serial.println("WiFi connect failed — USB serial still works.");
    }
}

// ── setup / loop ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

#if ADC_ON_M4
    hostClaimShared();
    bootM4();
    engineWaitBoot(3000);
    if(engineUp) Serial.println(F("M4 ADC co-processor up"));
    else {
        Serial.println(F("M4 ADC co-processor DID NOT START"));
        printEngineDiag(Serial);
    }
#else
    hostClaimShared();
    engBegin(localEng, shm, &adc);
    engineWaitBoot(500);
    Serial.println(F("ADC engine running on M7 (single-core build)"));
#endif

    displayPresent=touch.begin();
    if(displayPresent){ gfx.begin(); gfx.setRotation(1); drawSettingsScreen(); }
    connectWiFi();
    if(displayPresent) drawSettingsScreen();   // redraw so IP shows
}

void loop() {
    hostPumpEngine();               // no-op unless this is the single-core build

#if ADC_ON_M4
    // Keep looking for the co-processor. Cheap, and it means a slow or late M4
    // start recovers on its own instead of needing a board reset. It also gives
    // the diagnosis somewhere to land: USB CDC is rarely enumerated in time to
    // catch what setup() printed.
    if(!engineUp){
        static uint32_t lastProbeMs = 0;
        static uint8_t  probeCount  = 0;
        if(millis()-lastProbeMs >= 500){
            lastProbeMs = millis();
            if(engineAlive()){
                Serial.println(F("M4 ADC co-processor came up late - now running"));
                if(displayPresent && currentScreen==SCR_SETTINGS) drawSettingsScreen();
            } else if(probeCount < 6){
                probeCount++;
                Serial.println(F("M4 ADC co-processor DID NOT START"));
                printEngineDiag(Serial);
            }
        }
    }
#endif
    if(streamActive) streamStep();
    if(logRunning) loggerStep();
    if(displayPresent) {
        // ── Continuous mode: fire next capture when interval has elapsed ──
        if(contMode && currentScreen==SCR_PLOT) {
            uint32_t waitMs = (uint32_t)dTime < 1000u ? 1000u-(uint32_t)dTime : 0u;
            if(millis()-lastContMs >= waitMs) {
                lastContMs=millis();
                runDisplayCapture();
                return;
            }
        }
        // ── Touch handling ────────────────────────────────────────────────
        GDTpoint_t pts[5];
        uint8_t n=touch.getTouchPoints(pts);
        if(n>0) {
            uint32_t now=millis();
            if(now-lastTouchMs>TOUCH_DEBOUNCE_MS) {
                lastTouchMs=now;
                int tx=pts[0].y, ty=DISP_H-1-pts[0].x;
                switch(currentScreen) {
                    case SCR_SETTINGS: handleSettingsTouch(tx,ty); break;
                    case SCR_PRESETS:  handlePresetsTouch(tx,ty);  break;
                    case SCR_DIFF:     handleDiffTouch(tx,ty);     break;
                    case SCR_PLOT:     handlePlotTouch(tx,ty);     break;
                    case SCR_LOGGER:   handleLoggerTouch(tx,ty);   break;
                    case SCR_LOGGING:  handleLoggingTouch(tx,ty);  break;
                    case SCR_STREAM:   handleStreamTouch(tx,ty);   break;
                    default: break;
                }
            }
        }
    }

    // ── Command channel: USB serial OR WiFi TCP (both are Streams) ───────────
    // USB serial takes priority when bytes are waiting — it is also the way to
    // stop a stream that is running over a wedged WiFi link.
    if(Serial.available()){ processCommand(Serial); return; }

    // A live stream needs its socket to stay open across many replies, so while
    // one is running that connection is held and no new one is accepted. Every
    // other command still gets the one-command-per-connection treatment: the
    // Giga's socket pool is tiny and leaks/wedges if a socket is left dangling.
    if(streamActive && streamOnWifi){
        if(!streamClient.connected()) stopStream(false);
        else if(streamClient.available()){
            cmdFromStream = true;
            processCommand(streamClient);
            cmdFromStream = false;
        }
        return;
    }

    if(wifiActive){
        WiFiClient c = cmdServer.available();
        if(c){
            // Wait briefly for the full command line to arrive.
            uint32_t t0 = millis();
            while(c.connected() && !c.available() && millis()-t0 < 1500) delay(1);
            holdClient = false;
            if(c.available()) processCommand(c);
            if(holdClient){
                // STREAM: keep this socket. WiFiClient is refcounted, so the
                // copy keeps the socket alive when `c` goes out of scope.
                streamClient = c;
                streamOnWifi = true;
                streamIo     = &streamClient;
                holdClient   = false;
                return;
            }
            // Let the GUI finish reading the whole reply (incl. large DATA dumps)
            // and close first, so stop() can't truncate the response. Capped so a
            // stalled client can never wedge the board.
            c.flush();
            uint32_t t1 = millis();
            while(c.connected() && millis()-t1 < 8000) delay(2);
            c.stop();
        }
    }
}

// ── Command processing: shared by USB serial and WiFi (both are Streams) ──────
static void processCommand(Stream& io) {
    // Read + validate the command line BEFORE any side effects. A partial or
    // stray read (common right after a socket opens/closes) must never trigger a
    // capture with the wrong parameters — that was the source of mislabeled
    // elapsed times over WiFi.
    String cmd=io.readStringUntil('\n'); cmd.trim();

    // ── Live stream control ──────────────────────────────────────────────────
    // While a stream holds this socket it serves STOP and nothing else: any
    // other reply would race the teardown, and the GUI opens a fresh connection
    // per capture anyway. USB serial can always force a stop.
    if(cmdFromStream && !cmd.startsWith("STOP")){
        if(cmd.length()) io.println("ERR:STREAM_ACTIVE");
        return;
    }

    if(cmd.startsWith("STOP")){
        if(streamActive) stopStream(true);
        else             io.println("STREAMOFF");
        return;
    }
    if(cmd.startsWith("STREAM:")){
        // STREAM:A0,A1,D1|D1:A4-A5|BITS:14|RATE:250000|SMOOTH:8|HZ:4
        // The reply's STREAMON line names the legs actually used for each pair.
        int bar=cmd.indexOf('|');
        String pinsStr=(bar<0)?cmd.substring(7):cmd.substring(7,bar);
        bool sel[SHM_N_STAT]={};
        int ci=0;
        while(ci>=0){
            int nc=pinsStr.indexOf(',',ci);
            String nm=(nc<0)?pinsStr.substring(ci):pinsStr.substring(ci,nc); nm.trim();
            for(int k=0;k<SHM_N_STAT;k++) if(nm.equals(statName(k))){sel[k]=true;break;}
            if(nc<0) break; ci=nc+1;
        }
        auto field=[&](const char* key,long dflt)->long{
            int k=cmd.indexOf(key);
            if(k<0) return dflt;
            long v=cmd.substring(k+strlen(key)).toInt();
            return v>0?v:dflt;
        };
        long bits   = field("|BITS:",   12);
        long rate   = field("|RATE:",   250000);
        long smooth = field("|SMOOTH:", 1);
        long hz     = field("|HZ:",     4);

        // |D1:A4-A5 / |D2:A2-A3 name the legs behind each pair. Without them a
        // pair falls back to the board's own Diff config, which is very likely
        // NOT what the host means by "Diff 1" - the two configs drift.
        int8_t dpos[SHM_N_DIFF], dneg[SHM_N_DIFF];
        bool   named = false;
        for(int d=0;d<SHM_N_DIFF;d++){
            dpos[d] = -1; dneg[d] = -1;
            char key[6]; snprintf(key,sizeof(key),"|%s:",DIFF_NAMES[d]);
            int k = cmd.indexOf(key);
            if(k < 0) continue;
            String spec = cmd.substring(k+strlen(key));
            int bar = spec.indexOf('|'); if(bar >= 0) spec = spec.substring(0,bar);
            int dash = spec.indexOf('-'); if(dash < 0) continue;
            String pn = spec.substring(0,dash);      pn.trim();
            String nn = spec.substring(dash+1);      nn.trim();
            for(int i=0;i<N_CHANNELS;i++){
                if(pn.equals(CH_NAMES[i])) dpos[d]=i;
                if(nn.equals(CH_NAMES[i])) dneg[d]=i;
            }
            if(dpos[d]>=0 && dneg[d]>=0) named = true;
        }

        if(startStream(&io,(uint32_t)rate,(int)bits,(uint32_t)smooth,(uint32_t)hz,sel,
                       named?dpos:nullptr, named?dneg:nullptr))
            holdClient=true;      // the WiFi branch keeps this socket open
        return;
    }

    if(!cmd.startsWith("PINS:")){
        if(cmd.length()) io.println("ERR:BAD_CMD");
        return;
    }

    // ── One-shot capture ─────────────────────────────────────────────────────
    contMode=false;  // any incoming command stops continuous view
    if(streamActive) stopStream(false);
    if(logRunning)   stopLoggerQuiet();
    if(displayPresent){ currentScreen=SCR_CAPTURING; drawCaptureScreen(); }

    int bitRes      =cmd.substring(cmd.indexOf("BITS:")+5,cmd.indexOf("|TIME:")).toInt();
    int duration    =cmd.substring(cmd.indexOf("TIME:")+5,cmd.indexOf("|SMOOTH:")).toInt();
    int smoothCount =cmd.substring(cmd.indexOf("SMOOTH:")+7,cmd.indexOf("|LOG:")).toInt();
    int logTarget   =cmd.substring(cmd.indexOf("LOG:")+4).toInt();
    int ri          =cmd.indexOf("|RATE:");
    uint32_t sampleRate=(ri>=0)?(uint32_t)cmd.substring(ri+6).toInt():500000;

    int trigIdx=cmd.indexOf("|TRIG:");
    int32_t trigThresh[N_CHANNELS]={};
    if(trigIdx>=0){
        String ts=cmd.substring(trigIdx+6);
        for(int i=0;i<N_CHANNELS;i++){
            trigThresh[i]=ts.toInt();
            int c=ts.indexOf(','); if(c<0) break;
            ts=ts.substring(c+1);
        }
    }
    int midIdx=cmd.indexOf("|MID:");
    bool midpointMode=(midIdx>=0&&cmd.substring(midIdx+5).toInt()==1);

    if(bitRes<8)         bitRes=12;
    if(duration<1)       duration=1000;
    if(smoothCount<1)    smoothCount=1;
    if(logTarget<1)      logTarget=1;
    if(logTarget>MAX_LOG)logTarget=MAX_LOG;
    if(sampleRate<1000)  sampleRate=500000;

    bool pinReq[N_CHANNELS]={};
    String pinsStr=cmd.substring(5,cmd.indexOf("|BITS:"));
    int ci=0;
    while(ci>=0){
        int nc=pinsStr.indexOf(',',ci);
        String nm=(nc<0)?pinsStr.substring(ci):pinsStr.substring(ci,nc); nm.trim();
        for(int i=0;i<N_CHANNELS;i++) if(nm.equals(CH_NAMES[i])){pinReq[i]=true;break;}
        if(nc<0) break; ci=nc+1;
    }
    for(int i=0;i<N_CHANNELS;i++) if(!pinReq[i]) trigThresh[i]=0;

    // Set the capture running on the engine, then pump the ring while it does
    // the work. On a midpoint capture that pump also has to watch for !ABORT.
    uint32_t cm=0;
    for(int i=0;i<N_CHANNELS;i++){
        hPinReq[i]=pinReq[i];
        if(pinReq[i]) cm|=(1u<<i);
    }
    hLogTarget    = logTarget;
    hLogCnt       = 0;
    hMidMode      = midpointMode;
    hMidPreSize   = max(1,logTarget/2);
    hMidPostSize  = logTarget-hMidPreSize;
    hMidRingHead  = 0;
    hMidRingFill  = 0;
    hMidPostCount = 0;

    if(!hostIssue(GIGA_CMD_CAPTURE,sampleRate,bitRes,smoothCount,
                  (uint32_t)duration,(uint32_t)logTarget,cm,cm,
                  trigThresh,midpointMode,0)){
        io.println(engineUp?"ERR:ADC_BEGIN_FAILED":"ERR:M4_NOT_RUNNING");
        if(displayPresent){ currentScreen=SCR_SETTINGS; drawSettingsScreen(); }
        return;
    }

    uint32_t deadline=millis()+(uint32_t)duration+5000;
    bool aborted=false;
    while(true){
        hostPumpEngine();
        hostDrainRing();
        if(shm->e.state==GIGA_ST_CAPTURE_DONE) break;
        if(shm->e.state==GIGA_ST_ERROR){ io.println("ERR:ADC_BEGIN_FAILED"); return; }
        if(midpointMode){
            if(io.available()){
                String inc=io.readStringUntil('\n'); inc.trim();
                if(inc==F("!ABORT")){
                    shm->h.abortFlag=shm->h.abortFlag+1;
                    shmClean(&shm->h,sizeof(ShmToEngine));
                    aborted=true;
                }
            }
            // Wait for the engine to acknowledge the abort by finishing.
            if(aborted && (int32_t)(millis()-(deadline+2000))>0) break;
        } else if((int32_t)(millis()-deadline)>0) break;
    }
    hostDrainRing();

    int nStored = midpointMode ? (hMidRingFill+hMidPostCount) : hLogCnt;

    for(int i=0;i<N_CHANNELS;i++){
        if(!pinReq[i]||shm->e.cCount[i]==0) continue;
        double mean,rms,stdDev; hostChanStats(i,mean,rms,stdDev);
        io.print("PIN:");io.print(CH_NAMES[i]);io.print("|BITS:");io.print(bitRes);
        io.print("|COUNT:");io.print(shm->e.cCount[i]);io.print("|MIN:");io.print(shm->e.cMin[i]);
        io.print("|MAX:");io.print(shm->e.cMax[i]);io.print("|MEAN:");io.print(mean,4);
        io.print("|RMS:");io.print(rms,4);io.print("|STD:");io.print(stdDev,4);
        io.print("|LOGGED:");io.println(nStored);
    }
    for(int i=0;i<N_CHANNELS;i++){
        if(!pinReq[i]||nStored==0) continue;
        io.print("DATA:");io.print(CH_NAMES[i]);io.print("|VALS:");
        if(midpointMode){
            int si=(hMidRingFill<hMidPreSize)?0:hMidRingHead;
            for(int j=0;j<hMidRingFill;j++){
                io.print(logData[i][(si+j)%hMidPreSize]);
                if(j<hMidRingFill-1||hMidPostCount>0) io.print(',');
            }
            for(int j=0;j<hMidPostCount;j++){
                io.print(logData[i][hMidPreSize+j]);
                if(j<hMidPostCount-1) io.print(',');
            }
        } else {
            for(int j=0;j<nStored;j++){io.print(logData[i][j]);if(j<nStored-1)io.print(',');}
        }
        io.println();
    }
    // Elapsed time comes from the engine, which derives it from the sample maths
    // rather than the wall clock — wall clock is inflated by ADC start-up
    // latency and WiFi loop jitter, which stretched the GUI's time axis and made
    // frequency read ~20% low over WiFi.
    if(shm->e.ringDropped){
        // Only reachable if the M7 was starved long enough for the shared ring
        // to fill; the reply is still valid, just short. Emitted before DONE
        // because the GUI stops reading at DONE, and a line left unread would
        // surface at the head of the next capture's reply.
        io.print("WARN|DROPPED:"); io.println(shm->e.ringDropped);
    }
    io.print("DONE|ELAPSED:"); io.println(shm->e.elapsedMs);

    // ── If display present, show the serial-triggered capture on screen ───────
    if(displayPresent) {
        for(int i=0;i<N_CHANNELS;i++) {
            lastPinReq[i]     = pinReq[i];
            lastLogCnt[i]     = pinReq[i] ? nStored : 0;
            lastChanType[i]   = pinReq[i] ? 1 : 0;  // raw/uncalibrated
            lastChanOffset[i] = 0.0f;
            lastChanScale[i]  = 1.0f;
        }
        memset(lastDiff, 0, sizeof(lastDiff));
        lastBitRes        = bitRes;
        lastElapsedMs     = shm->e.elapsedMs;
        lastMidMode       = midpointMode;
        lastMidRingHead   = hMidRingHead;
        lastMidRingFill   = hMidRingFill;
        lastMidPostCount  = hMidPostCount;
        lastMidPreSize    = hMidPreSize;
        captureReady      = true;
        currentScreen     = SCR_PLOT;
        drawPlotScreen();
    }
}
