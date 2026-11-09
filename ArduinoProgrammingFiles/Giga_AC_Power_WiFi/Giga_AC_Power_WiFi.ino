/*!
 * Giga AC Power Monitor (Wi-Fi)
 * A bench mains power monitor on an Arduino Giga R1 WiFi, controlled over Wi-Fi
 * (TCP port 8080) or USB serial with the same line protocol. It speaks the
 * 120VMonitor protocol (fw 1.4), so 120VMonitor's GUI, calibration and Android
 * app drive it. Copies of the GUI, adapted, are in this folder.
 *
 * ── What the Giga does differently ───────────────────────────────────────────
 *
 *  1. V and I pairs are sampled AT THE SAME TIME. ADC1 and ADC2 run in dual
 *     regular-simultaneous mode (AdvancedADCDual), paced by a timer, DMA'd:
 *         ADC1 rank 0 = V+ (A0)    ADC2 rank 0 = V- (A1)    <- same instant
 *         ADC1 rank 1 = I+ (A2)    ADC2 rank 1 = I- (A3)    <- same instant
 *     V and I are one conversion apart (1.28 us at the defaults). The
 *     metering maths removes that lag, and captures are interpolated onto
 *     the V instants, so PHASEUS is only the front ends' own difference.
 *
 *  2. Gap-free. The ADC never stops: 80 kHz scans go into an 80 kHz ring
 *     (6.5 s, captures come from it) and, averaged by DECIM, into an 8 kS/s
 *     ring (metering, 8 s). Every mains cycle is measured, so the energy count
 *     has no holes, and a capture's pretrigger is always full: it reaches
 *     back before the !CAP even arrived. A lost DMA buffer is detected,
 *     counted ($STATUS gaps) and flagged (+16), never spliced.
 *
 *  3. Captures at any rate up to the scan rate, from the ring: each stored
 *     sample is the mean of N scans spread across its period (the Average
 *     N of the GUI); scans are 12.5 us apart at 80 kHz ($STATUS scanus4).
 *     Triggers and safe start run on the live scan stream, so the relay
 *     reacts within one DMA buffer (~2 ms) of an event.
 *
 *  4. Non-blocking output (net_sink.h) and the Wi-Fi join on its own RTOS
 *     thread, so neither a slow link nor a missing network costs samples.
 *
 * ── Measurement ─────────────────────────────────────────────────────────────
 *   Cycles run between rising zero crossings of V, found to a fraction of a
 *   sample with hysteresis, integrated to those fractional ends with a
 *   piecewise-cubic rule. The power integrand pairs each current sample with
 *   V interpolated (cubic Lagrange) to the instant that current sample was
 *   taken; the interpolator's and the boxcar's gains are divided back out.
 *   Verified with !SYNTH: V, I and P within 4 ppm, PF to the 4th decimal.
 *   With no voltage crossings (DC, no mains) nominal-length cycles are cut,
 *   flagged +1. A window is CYCLES mains cycles: $PWR comes 5 times a second
 *   at 60 Hz with the default 12.
 *
 * ── Current ranges (RNGPIN, default D6) ─────────────────────────────────────
 *   HIGH = IGAIN/IOFF, LOW = IGAINLO/IOFFLO. RNGINV 0: pin HIGH = LOW range.
 *   RNGMODE 0 auto: any scan over RNGUPA (or clipped) on LOW goes HIGH at
 *     once; after RNGHOLD ms of window peaks under RNGDNA it drops to LOW.
 *     Closing the load relay goes HIGH first (the close waits for the range
 *     relay to settle), so inrush never lands on the LOW shunt.
 *   RNGMODE 1 HIGH only.  2 LOW (an overload latches HIGH until the range is
 *     selected again).  3 auto-delayed: starts LOW, goes HIGH only when more
 *     than RNGCYC mains cycles in a row overload; drops back like auto; a
 *     relay close stays LOW.
 *   Each scan is scaled by the range it was taken on: the DMA buffers carry
 *   the time they filled, so the switch lands on the right scan although the
 *   buffers in flight still hold the old range. For RNGSETMS after a switch
 *   (a relay: ~3-5 ms of the old path on the new scale, then up to 8 ms of
 *   bounce at ~60x, measured 2026-11-08) each current scan is replaced by
 *   the one a mains cycle earlier, flagged +128. RMS, energy, trips and auto
 *   range never see the spike.
 *
 * ── Commands (newline terminated) ───────────────────────────────────────────
 *   120VMonitor protocol (see its README):
 *   !ID                       $READY,GIGA,<fw>
 *   !START / !STOP            stream / stop $PWR ($OK,start / $OK,stop)
 *   !STATUS                   $STATUS,key=value,...
 *   !CFG                      $CFG,<key>,<value>,<min>,<max>,<help> ... $CFGEND
 *   !GET,<key>  !SET,<key>,<value>   one $CFG row back (RAM; !SAVE keeps it)
 *   !SAVE / !LOAD / !DEFAULTS the last internal flash sector ($OK,save ...).
 *                             !SAVE stops sampling for ~1 s (the erase runs
 *                             with interrupts off; it shows as a gap), so it
 *                             is refused while the load relay is closed.
 *   !DUMP[,SAVED]             $DUMPHDR,fw=,board=,uid=,cfgver=,src=,dirty=
 *                             $DUMP,<group>,<key>,<value> ... $DUMPEND
 *   !RELAY,<0|1>              $RLY (a close clears a latched trip)
 *   !RANGE,<0|1|2|3>          RNGMODE (RAM); $RNG,<0 HIGH|1 LOW>,CMD,<mode>
 *   !ZERO                     V, then each current range in turn, as offsets
 *                             (~0.5 s): $CFG rows for VOFF IOFF IOFFLO, $OK,zero
 *   !EZERO                    energy to 0 ($OK,ezero)
 *   !DC[,<ms>]                $DC,<v_adc>,<i_adc>,<v>,<i>,<v_sd_adc>,<i_sd_adc>,
 *                             <scans>,<flags>,<rng>,<rng_changed>
 *   !CAP,<rate>,<n>,<ch>,<trig>,<level>,<pre%>,<timeout_ms>,<relay_us>,
 *        <ipk_a>,<ov_a>,<ov_cyc>,<auto_ms>,<open_after>,<avg>,<rng>,<mon>
 *        as 120VMonitor. rate 0 = the scan rate. trig 0 now, 1 |I|>=level,
 *        2 V rising zero, 3 close the relay (relay_us after a rising zero, or
 *        at once with -1), 4 |V|>=level, 5 close then |I|>=level, 6 a cycle's
 *        Irms drops below level after one at or above it. mon is ignored:
 *        metering never stops here. Any command while a capture waits for
 *        its trigger aborts it ($CAPTMO,ABORT), as on 120VMonitor.
 *   !ABORT                    stop waiting for a trigger
 *   Giga extras:
 *   !SMASK,<bits>             1 $PWR, 2 $CYC, 4 $HARM, 8 $AVG, 16 $INFO
 *   !STAT                     one $INFO line
 *   !HGET / !AVGW             one $HARM / $AVG from the last window
 *   !SYNTH,<on>[,vrms,irms,deg,h3%,hz]  exact synthetic V/I instead of the
 *                             ADC (still paced by it); !SYNTH,0 to stop
 *   !WIFI / !SCAN             rejoin Wi-Fi / module firmware + networks
 *
 * ── Output ──────────────────────────────────────────────────────────────────
 *   $PWR,<vrms>,<irms>,<w>,<va>,<var>,<pf>,<hz>,<vpk>,<ipk>,<wh>,<t_s>,<n>,
 *        <samples_per_cycle>,<relay>,<flags>,<vdc>,<idc>,<rng>
 *        t_s is the sample clock. flags: +1 no V zero crossings  +2 V clipped
 *        +4 I clipped  +8 trip latched  +16 samples lost  +64 synthetic data
 *        +128 a current range switch inside the window
 *   $CAP,<n>,<ch>,<dt_us>,<v_per>,0,<i_per>,0,<trig_idx>,<flags>,<relay_idx>,
 *        <open_idx>,<open_why>,<avg>,<rng>,<rng_idx>,<i_per>,0,<settle_us>
 *        then $CD,<hex int16 x32> lines (V,I interleaved for ch 2), $CEND,<n>.
 *        The current is already scaled per range, so i_per_hi = i_per.
 *   $RNG,<0|1>,<BOOT|CMD|AUTO|OVER|CLOSE|CAP>,<mode>[,<amps>]
 *   $SAFE,<INST|OVERCURRENT|TIMER|END|TRIPPK|REMOTE|TRIPA>,<amps>
 *   $TRIP,<RMS|PK>,<value>,<limit>   $RLY,<0|1>   $CAPTMO,<TIMEOUT|ABORT>
 *   $OK,<what>   $ERR,<message>   $NET,ip,port,rssi,status,powersave,ssid
 *   $CYC $HARM $AVG $INFO (extras, !SMASK): see README.
 */

#include <Arduino_AdvancedAnalog.h>
#include <WiFi.h>
#include "USB/PluggableUSBSerial.h"   // _SerialUSB: the USBSerial behind Serial
#include "WhdSTAInterface.h"          // the Giga's Wi-Fi driver, for power-save
#include "whd_wifi_api.h"
#include "FlashIAP.h"
#include "SDRAM.h"
#include "ac_types.h"
#include "adc_access.h"
#include "net_sink.h"

#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#else
#error "Copy arduino_secrets.example.h to arduino_secrets.h and put your Wi-Fi name/password in it"
#endif

#define FW_VERSION   "2.0"
#define CFG_VERSION  1
#define CFG_MAGIC    0x50434147UL   // "GACP"
static const uint16_t CMD_PORT = 8080;

// A0..A6 can sit on ADC1 or ADC2 (A7 is ADC1 only). Keep each pair's + on
// adcA and - on adcB, so the pair converts at one instant.
AdvancedADC     adcA(A0, A2);       // ADC1: rank 0 = V+, rank 1 = I+
AdvancedADC     adcB(A1, A3);       // ADC2: rank 0 = V-, rank 1 = I-
AdvancedADCDual adcDual(adcA, adcB);
static const int ADC_RANKS = 2;

// ── result flags (per sample in the ring; mapped to the protocol's on output)
#define FLG_NOCROSS  0x01
#define FLG_VCLIP    0x02
#define FLG_ICLIP    0x04
#define FLG_TRIP     0x08
#define FLG_GAP      0x10
#define FLG_ADCERR   0x20
#define FLG_SYNTH    0x40
#define FLG_IRANGE   0x80

#define SM_PWR   0x01
#define SM_CYC   0x02
#define SM_HARM  0x04
#define SM_AVG   0x08
#define SM_INFO  0x10

// ── buffers, all in the 8 MB SDRAM ────────────────────────────────────────────
#define RING_BITS  16
#define RING_N     (1UL << RING_BITS)        // 65536 metering samples: 8.2 s at 8 kS/s
#define RING_MASK  (RING_N - 1)
#define RAW_BITS   19
#define RAW_N      (1UL << RAW_BITS)         // 524288 scans: 6.55 s at 80 kHz
#define RAW_MASK   (RAW_N - 1)
#define CAP_VALUES 262144UL                  // int16 values per capture ($STATUS capmax)
#define H_MAX      63
#define AVG_M      128
#define POOL_BYTES (64UL * 1024UL)           // DMA pool per ADC

static float*   g_rv = nullptr;   // metering V, raw differential counts
static float*   g_ri = nullptr;   // metering I, amps (each scan scaled by its own range)
static uint8_t* g_rf = nullptr;   // FLG_* per metering sample
static float*   g_xv = nullptr;   // every scan's V, volts
static float*   g_xi = nullptr;   // every scan's I, amps
static int16_t* g_cq = nullptr;   // the capture being sent

// ── ADC constants ─────────────────────────────────────────────────────────────
// AdvancedADC clocks the ADCs from per_ck (64 MHz) /1, and rev-V silicon
// halves it: 32 MHz. One conversion = sample time + (bits/2 + 0.5) cycles.
static const double ADC_CLK_HZ = 32.0e6;
static const float  SMP_CYC[8] = {1.5f, 2.5f, 8.5f, 16.5f, 32.5f, 64.5f, 387.5f, 810.5f};
static const adc_sample_time_t SMP_T[8] = {
  AN_ADC_SAMPLETIME_1_5,  AN_ADC_SAMPLETIME_2_5,  AN_ADC_SAMPLETIME_8_5,
  AN_ADC_SAMPLETIME_16_5, AN_ADC_SAMPLETIME_32_5, AN_ADC_SAMPLETIME_64_5,
  AN_ADC_SAMPLETIME_387_5, AN_ADC_SAMPLETIME_810_5
};

// ═════════════════════════════════════════════════════════════════════════════
//  configuration
// ═════════════════════════════════════════════════════════════════════════════

static Config cfg;
static bool   g_dirty = false;
static const char* g_cfgSrc = "DEFAULTS";

#define CF(m) ((uint16_t)offsetof(Config, m))
enum { CG_CAL, CG_FUSE, CG_RANGE, CG_METER, CG_WIRING, CG_ADC };
static const char* const CFG_GROUP_NAMES[] = { "CAL", "FUSE", "RANGE", "METER", "WIRING", "ADC" };
#define AP_DERIVED  0x01      // recompute scaling / alpha
#define AP_RESTART  0x02      // restart the ADC (rate, resolution ...)
#define AP_PINS     0x04      // relay / range pins
#define AP_ANALYZER 0x08      // restart the cycle detector
#define AP_RANGE    0x10      // re-apply the range mode
#define AP_WINDOW   0x20      // clear the window accumulators
#define AP_GAP      0x40      // a discontinuity in the samples

static const CfgKey KEYS[] = {
  {"VGAIN",   FT_FLOAT, CF(vGain),   -1e6f, 1e6f,  "volts per ADC volt", CG_CAL, AP_DERIVED},
  {"VOFF",    FT_FLOAT, CF(vOff),    -2.0f, 2.0f,  "ADC volts at 0 V", CG_CAL, AP_DERIVED},
  {"IGAIN",   FT_FLOAT, CF(iGain),   -1e5f, 1e5f,  "HIGH range amps per ADC volt", CG_CAL, AP_DERIVED},
  {"IOFF",    FT_FLOAT, CF(iOff),    -2.0f, 2.0f,  "HIGH range ADC volts at 0 A", CG_CAL, AP_DERIVED},
  {"IGAINLO", FT_FLOAT, CF(iGainLo), -1e5f, 1e5f,  "LOW range amps per ADC volt", CG_CAL, AP_DERIVED},
  {"IOFFLO",  FT_FLOAT, CF(iOffLo),  -2.0f, 2.0f,  "LOW range ADC volts at 0 A", CG_CAL, AP_DERIVED},
  {"PHASEUS", FT_FLOAT, CF(phaseUs), -100,  100,   "current front end's delay vs voltage, us (the ADC's own V->I lag is already removed)", CG_CAL, AP_DERIVED},
  {"ADCVREF", FT_FLOAT, CF(adcVref), 1.0f,  3.6f,  "ADC reference volts", CG_CAL, AP_DERIVED},
  {"TRIPA",   FT_FLOAT, CF(tripA),   0,     1000,  "relay trip Irms (checked every mains cycle), 0=off", CG_FUSE, 0},
  {"TRIPPK",  FT_FLOAT, CF(tripPk),  0,     1000,  "relay trip |i| on any single scan, 0=off", CG_FUSE, 0},
  {"RNGMODE", FT_U8,    CF(rngMode), 0,     3,     "0 auto, 1 HIGH, 2 LOW, 3 auto-delayed", CG_RANGE, AP_RANGE},
  {"RNGUPA",  FT_FLOAT, CF(rngUpA),  0.05f, 1000,  "LOW: any |i| scan above -> HIGH", CG_RANGE, 0},
  {"RNGDNA",  FT_FLOAT, CF(rngDnA),  0,     1000,  "auto: window |i| pk below -> LOW", CG_RANGE, 0},
  {"RNGHOLD", FT_U16,   CF(rngHoldMs),0,    60000, "auto: ms of quiet windows in HIGH before LOW", CG_RANGE, 0},
  {"RNGSETMS",FT_U8,    CF(rngSetMs),1,     200,   "range relay settle ms (current replaced by the cycle before)", CG_RANGE, 0},
  {"RNGCYC",  FT_U8,    CF(rngCyc),  0,     60,    "auto-delayed: HIGH after more than this many overloaded cycles in a row", CG_RANGE, 0},
  {"MAINS",   FT_FLOAT, CF(mainsHz), 10.0f, 400.0f,"nominal mains Hz", CG_METER, AP_DERIVED | AP_ANALYZER},
  {"CYCLES",  FT_U8,    CF(cycles),  1,     250,   "mains cycles per window", CG_METER, 0},
  {"HARMN",   FT_U8,    CF(harmN),   1,     63,    "harmonics computed ($HARM)", CG_METER, AP_DERIVED | AP_WINDOW},
  {"VMIN",    FT_FLOAT, CF(vMin),    0,     1000,  "smallest V peak that counts as mains for the crossings", CG_METER, 0},
  {"DCREM",   FT_U8,    CF(dcRem),   0,     1,     "remove DC from each cycle", CG_METER, 0},
  {"RATE",    FT_U32,   CF(rate),    1000,  400000,"ADC scan rate, S/s (restarts sampling)", CG_ADC, AP_RESTART},
  {"DECIM",   FT_U16,   CF(decim),   1,     1000,  "scans averaged per metering sample (restarts)", CG_ADC, AP_RESTART},
  {"BITS",    FT_U8,    CF(bits),    8,     16,    "ADC resolution 8/10/12/14/16 (restarts)", CG_ADC, AP_RESTART},
  {"SMP",     FT_U8,    CF(smp),     0,     7,     "ADC sample time 1.5/2.5/8.5/16.5/32.5/64.5/387.5/810.5 cycles (restarts)", CG_ADC, AP_RESTART},
  {"VSE",     FT_U8,    CF(vSe),     0,     1,     "voltage single-ended: ignore A1, subtract mid-scale", CG_ADC, AP_GAP},
  {"ISE",     FT_U8,    CF(iSe),     0,     1,     "current single-ended: ignore A3", CG_ADC, AP_GAP},
  {"RLYPIN",  FT_U8,    CF(relayPin),2,     53,    "load relay driver pin", CG_WIRING, AP_PINS},
  {"RLYINV",  FT_U8,    CF(relayInv),0,     1,     "load relay active-low", CG_WIRING, AP_PINS},
  {"RNGPIN",  FT_U8,    CF(rngPin),  0,     53,    "range relay pin, 0 = one range (HIGH)", CG_WIRING, AP_PINS | AP_RANGE},
  {"RNGINV",  FT_U8,    CF(rngInv),  0,     1,     "0: pin HIGH = LOW range, 1: pin HIGH = HIGH", CG_WIRING, AP_PINS},
};
static const int N_KEYS = sizeof(KEYS) / sizeof(KEYS[0]);

// Defaults are this board's bench calibration (2026-11-08, SPD4323X + DL3021A
// through the isolated front end). A different front end needs its own.
static void cfgDefaults(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC; c.version = CFG_VERSION; c.size = sizeof(Config);
  c.vGain = 102.744f;  c.vOff = 0.001267f;
  c.iGain = 11.7231f;  c.iOff = 0.000215f;
  c.iGainLo = 1.0727f; c.iOffLo = 0.000282f;
  c.phaseUs = 0.0f;    c.adcVref = 3.3f;
  c.tripA = 0.0f;      c.tripPk = 0.0f;
  // LOW reads +-1.77 A at the ADC's +-VREF/2 here, so it goes up at 1.6 A
  c.rngUpA = 1.6f;     c.rngDnA = 1.0f;  c.rngHoldMs = 3000;
  c.rngMode = 0;       c.rngSetMs = 20;  c.rngCyc = 2;
  c.cycles = 12;       c.harmN = 40;     c.dcRem = 1;
  c.mainsHz = 60.0f;   c.vMin = 8.5f;
  c.rate = 80000;      c.decim = 10;     c.bits = 16; c.smp = 4; c.vSe = 0; c.iSe = 0;
  c.relayPin = 5;      c.relayInv = 0;   c.rngPin = 6; c.rngInv = 0;
}

static float cfgGet(const CfgKey& k) {
  const uint8_t* p = (const uint8_t*)&cfg + k.off;
  switch (k.type) {
    case FT_U8:  return (float)*p;
    case FT_U16: { uint16_t v; memcpy(&v, p, 2); return (float)v; }
    case FT_U32: { uint32_t v; memcpy(&v, p, 4); return (float)v; }
    default:     { float v; memcpy(&v, p, 4); return v; }
  }
}
static void cfgPut(const CfgKey& k, float x) {
  uint8_t* p = (uint8_t*)&cfg + k.off;
  switch (k.type) {
    case FT_U8:  *p = (uint8_t)lroundf(x); break;
    case FT_U16: { uint16_t v = (uint16_t)lroundf(x); memcpy(p, &v, 2); } break;
    case FT_U32: { uint32_t v = (uint32_t)llroundf(x); memcpy(p, &v, 4); } break;
    default:     memcpy(p, &x, 4); break;
  }
}
static int cfgFind(const char* key) {
  for (int i = 0; i < N_KEYS; i++) if (strcasecmp(KEYS[i].key, key) == 0) return i;
  return -1;
}

static uint32_t crc32(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFUL;
  while (n--) { c ^= *p++; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1))); }
  return ~c;
}
static bool cfgValid(const Config& c) {
  return c.magic == CFG_MAGIC && c.version == CFG_VERSION && c.size == sizeof(Config) &&
         c.crc == crc32((const uint8_t*)&c, offsetof(Config, crc));
}

// ── the engine's working copies (applyConfig() fills them from cfg) ───────────
static uint32_t g_rate_req = 80000;   // scans per second requested
static double   g_rate     = 80000;   // what the timer actually runs at
static uint16_t g_decim    = 10;
static uint8_t  g_bits     = 16;
static uint8_t  g_smp      = 4;
static bool     g_v_se = false, g_i_se = false;
static float    g_vref     = 3.3f;
static bool     g_dcrem = true;
static uint8_t  g_cycles = 12;
static float    g_mains_hz = 60.0f;
static float    g_vmin = 8.5f;
static uint8_t  g_harmN = 40;
static float    g_trip_a = 0.0f, g_trippk = 0.0f;
static uint8_t  g_rlyPin = 5, g_rngPin = 6;
static bool     g_rly_inv = false, g_irInv = false;
static float    g_irSettleMs = 20.0f;

// derived, by applyDerived()
static double   g_fs = 8000, g_dt = 1.0 / 8000;   // metering sample rate
static double   g_scanUs = 12.5;                   // scan period
static double   g_lag_s  = 0;                      // I+ converts this long after V+
static int32_t  g_mid = 32768, g_clipLo = 0, g_clipHi = 65535;
static uint32_t g_back80 = 1333;                   // scans in a nominal mains cycle
// real = raw*K + B; alpha = I-sample lag in metering samples; Lagrange weights.
// Per current range (0 HIGH, 1 LOW): K, B, alpha, weights, full scale in A pk.
static double g_KV, g_BV, g_alpha, g_N0;
static double g_W[4];
static double g_KIr[2], g_BIr[2], g_alphaR[2], g_WR[2][4], g_irFsA[2];

// ── relay / protection ────────────────────────────────────────────────────────
static bool  g_relay = false, g_trip_latch = false;
static bool  g_closePending = false;     // !RELAY,1 waiting for the range relay (auto mode)

// ── current range: 0 HIGH, 1 LOW ──────────────────────────────────────────────
// g_irWant is what was asked for. serviceRange() moves the pin to it when no
// switch is in flight: g_irReq is the pin, g_irCur the range the incoming
// scans are scaled with; drainAdc() moves g_irCur over at the scan the switch
// happened on, from the DMA buffers' timestamps.
static uint8_t  g_irCur = 0, g_irReq = 0, g_irWant = 0;
static bool     g_irPending = false, g_irHistDone = false, g_irReport = false;
static uint32_t g_irSwUs = 0, g_irLastSwMs = 0, g_irSwitches = 0;
static IrHist   g_irHist[8];             // recent switch points, for the analysis's alpha
static uint8_t  g_irHistN = 0;
static uint8_t  g_rngWhy = 0;            // RNG_WHY index of the pending request
static float    g_rngWhyAmps = -1.0f;
static bool     g_rngLatchHigh = false;  // mode 2: an overload holds HIGH
static uint32_t g_rngQuietSince = 0;     // auto: first quiet window in HIGH (ms), 0 = none
static bool     g_cycOver = false;       // mode 3: this cycle overloaded
static float    g_cycOverPk = 0.0f;
static uint8_t  g_overCycles = 0;
static int8_t   g_capForce = -1;         // a capture pinned the range (0/1), -1 none
static uint8_t  g_zeroStep = 0;          // !ZERO in progress (auto range waits)
static const char* const RNG_WHY[] = { "BOOT", "CMD", "AUTO", "OVER", "CLOSE", "CAP" };
enum { RW_BOOT, RW_CMD, RW_AUTO, RW_OVER, RW_CLOSE, RW_CAP };

// ── run state ─────────────────────────────────────────────────────────────────
static uint8_t g_smask = 0;
static double  g_wh = 0.0;
static bool    g_adcOn = false;

// ── synthetic source ──────────────────────────────────────────────────────────
static bool  g_synth = false;
static float g_syn_vrms = 120.0f, g_syn_irms = 5.0f, g_syn_deg = 30.0f, g_syn_h3 = 0.0f, g_syn_hz = 60.0f;
static double g_synPh = 0.0;

// ── diagnostics ───────────────────────────────────────────────────────────────
static uint32_t g_gaps = 0, g_resyncs = 0, g_adcFixes = 0, g_dmaResets = 0, g_tcpBusy = 0;
static uint64_t g_lostUs = 0;          // sample time lost to gaps (not in the energy count)
static uint32_t g_lowDrops = 0;        // low-priority lines dropped to keep latency down
static uint32_t g_wifiReconnects = 0;
static uint32_t g_busyUs = 0, g_loopMaxUs = 0, g_lastLoopUs = 0;
static float    g_loadPct = 0.0f, g_loopMaxMs = 0.0f;
static uint32_t g_bufScans = 0, g_bufUs = 0, g_capBufs = 0, g_capFirstTs = 0, g_capLastTs = 0, g_prevTsA = 0;
static char     g_adcDiag[112] = "";

// ═════════════════════════════════════════════════════════════════════════════
//  output plumbing
// ═════════════════════════════════════════════════════════════════════════════

static TxRing     g_txSer, g_txTcp;
static LineBuf    g_ln;
static WiFiServer g_server(CMD_PORT);
static WiFiClient g_client;
static bool       g_clientOn = false, g_wifiOk = false;
static char       g_ipStr[20] = "none";
static int        g_wifiStatus = 0;     // last WiFi.status(): 3 = connected, 255 = no module
static int        g_wifiPs = -1;        // radio power-save mode after the join
static volatile bool g_wifiBusy = false, g_wifiDone = false, g_wifiDisc = false;   // join thread, see wifiTask()
static volatile int  g_wifiResult = 0;
// On this core Serial is a UART wrapper around _SerialUSB (a USBSerial), and
// `if (Serial)` runs delay(1) before it answers (operator bool in
// PluggableUSBSerial.h). Asked once per emitted line, that throttled the loop
// to a few hundred lines a second and the ADC overran (2026-11-08: 117 ms
// loops, lost buffers). So the USB state is read once per loop instead.
static bool       g_serOn = false;

// Hand the line in g_ln to every connected output, then clear it.
static void emit() {
  if (g_ln.size()) {
    if (g_serOn)    g_txSer.put(g_ln.data(), g_ln.size());
    if (g_clientOn) g_txTcp.put(g_ln.data(), g_ln.size());
  }
  g_ln.reset();
}
// Bulk lines ($CYC $HARM $AVG): queued only while that output is keeping up.
// 12 KB, half the ring: one window's $HARM + $AVG is a ~3.5 KB burst, and a
// 4 KB limit shed it on every slow Wi-Fi round trip (2026-11-08).
static const size_t LOW_BACKLOG = 12288;
// a capture's $CD lines keep the queue under this, so live lines still get
// through while it streams
static const size_t CAP_BACKLOG = 4096;
static void emitLow() {
  if (g_ln.size()) {
    if (g_serOn)    { if (g_txSer.used() < LOW_BACKLOG) g_txSer.put(g_ln.data(), g_ln.size()); else g_lowDrops++; }
    if (g_clientOn) { if (g_txTcp.used() < LOW_BACKLOG) g_txTcp.put(g_ln.data(), g_ln.size()); else g_lowDrops++; }
  }
  g_ln.reset();
}
static void emitStr(const char* s) { g_ln.reset(); g_ln.println(s); emit(); }
static void emitErr(const char* s) { g_ln.reset(); g_ln.print("$ERR,"); g_ln.println(s); emit(); }
static void emitOk(const char* s)  { g_ln.reset(); g_ln.print("$OK,");  g_ln.println(s); emit(); }

// Unsent bytes in the slowest connected output (paces the capture sender).
static size_t sinkBacklog() {
  size_t r = 0;
  if (g_serOn)    r = max(r, g_txSer.used());
  if (g_clientOn) r = max(r, g_txTcp.used());
  return r;
}
static bool anySink() { return g_serOn || g_clientOn; }

// Non-blocking drain of both rings. The socket timeout is 0, so write() takes
// what lwIP has room for and returns at once; the rest stays queued.
static void serviceTx() {
  const uint8_t* p;
  if (g_serOn) {
    size_t n = g_txSer.peek(&p);
    // send_nb, not write(): USBSerial::write() blocks until each packet is out
    if (n) { uint32_t got = 0; _SerialUSB.send_nb((uint8_t*)p, n, &got, true); g_txSer.consume(got); }
  } else {
    g_txSer.clear();
  }
  if (g_clientOn) {
    for (int k = 0; k < 6; k++) {
      size_t n = g_txTcp.peek(&p);
      if (!n) break;
      if (n > 1436) n = 1436;
      size_t w = g_client.write(p, n);
      if (w == 0) { g_tcpBusy++; break; }
      g_txTcp.consume(w);
      if (w < n) break;
    }
  }
}

// Numbers for $CFG / $DUMP: about 7 significant digits, without printf
// (float printf is not linked on this core).
static void printNum(float v) {
  float a = fabsf(v);
  int d = 6;
  if (a >= 1.0f) { d = 6 - (int)floorf(log10f(a)); if (d < 0) d = 0; }
  else if (a > 0.0f) { d = 6 - (int)floorf(log10f(a)); if (d > 9) d = 9; }
  g_ln.print(v, d);
}

// ═════════════════════════════════════════════════════════════════════════════
//  ADC: start/stop, the stuck-DMA repair, buffer pairing
// ═════════════════════════════════════════════════════════════════════════════

static uint32_t toResolution(int bits) {
  if (bits <= 8)  return AN_RESOLUTION_8;
  if (bits <= 10) return AN_RESOLUTION_10;
  if (bits <= 12) return AN_RESOLUTION_12;
  if (bits <= 14) return AN_RESOLUTION_14;
  return AN_RESOLUTION_16;
}
static double rankTime() { return (SMP_CYC[g_smp] + g_bits / 2.0 + 0.5) / ADC_CLK_HZ; }
static uint32_t maxRate() { return (uint32_t)(0.9 / (ADC_RANKS * rankTime())); }

// The timer clock as Arduino_AdvancedAnalog computes it (HALConfig.cpp
// hal_tim_freq). With PSC/ARR read back, this gives the rate the timer really
// runs at: rates that don't divide 120 MHz are rounded (76.8k -> 76.82k).
static uint32_t timClock(TIM_TypeDef* t) {
  if (((uint32_t)t & ~0xFFFFUL) == D2_APB1PERIPH_BASE)
    return HAL_RCC_GetPCLK1Freq() * ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) ? 2 : 1);
  return HAL_RCC_GetPCLK2Freq() * ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) ? 2 : 1);
}
static double actualRate() {
  AdcDescrMirror* d = adcDescr(adcA);
  if (!adcDescrValid(d) || !d->tim.Instance) return (double)g_rate_req;
  TIM_TypeDef* t = d->tim.Instance;
  return (double)timClock(t) / ((double)(t->PSC + 1) * (double)(t->ARR + 1));
}

static size_t largestFreeBlock() {
  size_t lo = 0, hi = 512 * 1024;
  while (hi - lo > 256) {
    size_t mid = (lo + hi) / 2; void* p = malloc(mid);
    if (p) { free(p); lo = mid; } else hi = mid;
  }
  return lo;
}

// Ported from Giga_RemoteController_WiFi (2026-11-05): a stop can leave
// ADC2's DMA stream stuck with EN=1 and its HAL handle in ERROR, and every
// later begin() fails until reset. adcSanitize() puts a stopped ADC and its
// stream back to the state begin() expects. It writes only after
// adcDescrValid() confirms the mirrored library struct.
static const uint32_t ADC_ERR_STATE = HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_CONFIG | HAL_ADC_STATE_ERROR_DMA;
static bool adcNeedsFix(AdvancedADC& a) {
  AdcDescrMirror* d = adcDescr(a);
  if (!adcDescrValid(d)) return false;
  DMA_Stream_TypeDef* st = (DMA_Stream_TypeDef*)d->dma.Instance;
  return (d->adc.State & ADC_ERR_STATE) || (d->adc.State & (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY)) ||
         d->adc.Lock != HAL_UNLOCKED || (d->adc.Instance->CR & (ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_JADSTART)) ||
         (st->CR & DMA_SxCR_EN) || d->dma.State == HAL_DMA_STATE_BUSY || d->dma.State == HAL_DMA_STATE_ERROR ||
         d->dma.State == HAL_DMA_STATE_ABORT || d->dma.Lock != HAL_UNLOCKED;
}
static void adcSanitize(AdvancedADC& a) {
  if (!adcNeedsFix(a)) return;
  AdcDescrMirror* d = adcDescr(a);
  snprintf(g_adcDiag, sizeof(g_adcDiag), "ADC%d st=%lX cr=%lX dma=%X",
           d->adc.Instance == ADC1 ? 1 : 2, (unsigned long)d->adc.State,
           (unsigned long)d->adc.Instance->CR, (unsigned)d->dma.State);
  ADC_TypeDef* r = d->adc.Instance;
  DMA_Stream_TypeDef* st = (DMA_Stream_TypeDef*)d->dma.Instance;
  const uint32_t RS = ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART |
                      ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN;   // write-1 bits: mask on RMW
  uint32_t t0 = micros();
  if (r->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) {
    r->CR = (r->CR & ~RS) | ADC_CR_ADSTP | ADC_CR_JADSTP;
    while ((r->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) && micros() - t0 < 2000) {}
  }
  if (r->CR & ADC_CR_ADEN) {
    r->CR = (r->CR & ~RS) | ADC_CR_ADDIS;
    while ((r->CR & ADC_CR_ADEN) && micros() - t0 < 4000) {}
  }
  st->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  st->FCR &= ~DMA_SxFCR_FEIE;
  t0 = micros();
  while ((st->CR & DMA_SxCR_EN) && micros() - t0 < 5000) {}
  if (st->CR & DMA_SxCR_EN) {
    DMA_Stream_TypeDef* const S[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
                                      DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
    bool others = false;
    for (DMA_Stream_TypeDef* o : S)
      if (o != DMA1_Stream1 && o != DMA1_Stream2 && (o->CR & DMA_SxCR_EN)) others = true;
    if (!others) {
      RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST; __DSB();
      RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA1RST; __DSB();
      NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn); NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
      g_dmaResets++;
      AdcDescrMirror* o1 = adcDescr(adcA); AdcDescrMirror* o2 = adcDescr(adcB);
      if (adcDescrValid(o1)) { o1->dma.State = HAL_DMA_STATE_RESET; o1->dma.ErrorCode = 0; o1->dma.Lock = HAL_UNLOCKED; }
      if (adcDescrValid(o2)) { o2->dma.State = HAL_DMA_STATE_RESET; o2->dma.ErrorCode = 0; o2->dma.Lock = HAL_UNLOCKED; }
    }
  }
  const uint32_t off = (st == DMA1_Stream1) ? 6 : (st == DMA1_Stream2) ? 16 : 22;
  DMA1->LIFCR = 0x3DUL << off;
  d->adc.State = HAL_ADC_STATE_RESET; d->adc.ErrorCode = HAL_ADC_ERROR_NONE; d->adc.Lock = HAL_UNLOCKED;
  d->dma.State = HAL_DMA_STATE_RESET; d->dma.ErrorCode = HAL_DMA_ERROR_NONE; d->dma.Lock = HAL_UNLOCKED;
  g_adcFixes++;
}

static void adcStop() {
  if (g_adcOn) adcDual.stop();
  g_adcOn = false;
}

static void markGap();
static void applyDerived();

// The H7 ADC has a linearity self-calibration (ADCALLIN), and AdvancedADC
// never runs it: hal_adc_config() asks for ADC_CALIB_OFFSET only. It runs
// here on a handle of our own, with the ADC disabled, just before AdvancedADC
// starts; its offset-only calibration leaves the linearity factors in place,
// and stop() only disables the ADC (no deep power-down), so they are kept.
// (2026-11-08: zero offsets ~2.3 mV -> 0.2-1.3 mV. It does not remove ADC2's
// code-transition error near 0x6000, which is the converter's own.)
static bool g_linCal = false;
static bool adcLinearityCal(ADC_TypeDef* inst) {
  ADC_HandleTypeDef h;
  memset(&h, 0, sizeof(h));
  h.Instance = inst;
  h.Init.Resolution = g_bits >= 16 ? ADC_RESOLUTION_16B : g_bits >= 14 ? ADC_RESOLUTION_14B
                    : g_bits >= 12 ? ADC_RESOLUTION_12B : g_bits >= 10 ? ADC_RESOLUTION_10B
                    : ADC_RESOLUTION_8B;
  h.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;          // as AdvancedADC
  h.Init.ScanConvMode = ADC_SCAN_DISABLE;
  h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  h.Init.LowPowerAutoWait = DISABLE;
  h.Init.ContinuousConvMode = DISABLE;
  h.Init.NbrOfConversion = 1;
  h.Init.DiscontinuousConvMode = DISABLE;
  h.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  h.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  h.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  h.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  h.Init.OversamplingMode = DISABLE;
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);
  __HAL_RCC_ADC12_CLK_ENABLE();
  if (inst->CR & ADC_CR_ADEN) return false;               // must be disabled to calibrate
  if (HAL_ADC_Init(&h) != HAL_OK) return false;
  return HAL_ADCEx_Calibration_Start(&h, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) == HAL_OK;
}

static bool adcStart() {
  adcStop();
  uint32_t lim = maxRate();
  if (g_rate_req > lim) g_rate_req = lim;
  if (g_rate_req < 1000) g_rate_req = 1000;

  // ~2 ms per buffer (a whole number of boxcars): safe start and the trips
  // see a buffer ~2 ms after it fills. The pool is as deep as POOL_BYTES
  // allows: 160 scans x 96 buffers = ~190 ms of slack at the defaults.
  uint32_t ns = g_rate_req / 500;
  if (ns < 32) ns = 32;
  ns = (ns / g_decim) * g_decim;
  if (ns < g_decim) ns = g_decim;
  size_t depth = POOL_BYTES / (ns * ADC_RANKS * sizeof(Sample));
  if (depth > 96) depth = 96;
  if (depth < 8)  depth = 8;

  for (int attempt = 0; attempt < 2 && !g_adcOn; attempt++) {
    adcDual.stop();
    adcSanitize(adcA); adcSanitize(adcB);
    g_linCal = adcLinearityCal(ADC1) && adcLinearityCal(ADC2);
    g_adcOn = adcDual.begin(toResolution(g_bits), g_rate_req, ns, depth, SMP_T[g_smp]) != 0;
  }
  if (!g_adcOn) {
    adcDual.stop();
    g_ln.reset(); g_ln.print("$ERR,ADC begin failed,heap "); g_ln.print(largestFreeBlock());
    g_ln.print(','); g_ln.println(g_adcDiag); emit();
    return false;
  }
  g_rate = actualRate();
  g_bufScans = ns;
  g_bufUs = (uint32_t)(ns * 1.0e6 / g_rate);
  g_capBufs = 0; g_prevTsA = 0;
  applyDerived();
  markGap();      // a restart is a discontinuity, by definition
  return true;
}

static inline bool adcAvailable() { return g_adcOn && adcA.available() && adcB.available(); }

// Each ADC fills its own buffers on the same triggers, so a matching pair
// completes microseconds apart. If the reader falls behind, the library reuses
// a full buffer instead of queuing a new one, and decides that per ADC. So the
// pair is checked by timestamp and re-paired if the sides drifted apart.
static ScanBuf adcRead() {
  DMABuffer<Sample>* a = &adcA.read();
  DMABuffer<Sample>* b = &adcB.read();
  bool gap = false;
  for (int k = 0; k < 4; k++) {
    int32_t d = (int32_t)(a->timestamp() - b->timestamp());
    if ((uint32_t)abs(d) <= g_bufUs / 2) break;
    g_resyncs++; gap = true;
    if (d < 0) { a->release(); a = &adcA.read(); }
    else       { b->release(); b = &adcB.read(); }
  }
  if (a->get_flags(DMA_BUFFER_DISCONT) || b->get_flags(DMA_BUFFER_DISCONT)) gap = true;
  uint32_t ts = a->timestamp();
  if (g_prevTsA) {
    uint32_t dts = (uint32_t)(ts - g_prevTsA);
    if (dts > (g_bufUs * 5) / 2) gap = true;
    if (gap && dts > g_bufUs) g_lostUs += dts - g_bufUs;
  }
  g_prevTsA = ts;
  if (g_capBufs++ == 0) g_capFirstTs = ts;
  g_capLastTs = ts;
  ScanBuf s = { a, b, (int)(min(a->size(), b->size()) / ADC_RANKS), gap, ts };
  return s;
}
static inline void adcRelease(ScanBuf& s) { s.a->release(); s.b->release(); }

// Scans per second over the buffers read since the last call.
static uint32_t measuredRate() {
  uint32_t r = 0;
  if (g_capBufs >= 3 && g_capLastTs != g_capFirstTs)
    r = (uint32_t)((uint64_t)(g_capBufs - 1) * g_bufScans * 1000000ULL / (uint32_t)(g_capLastTs - g_capFirstTs));
  g_capBufs = 0;
  return r;
}

// ═════════════════════════════════════════════════════════════════════════════
//  derived constants
// ═════════════════════════════════════════════════════════════════════════════

static inline float lsb() { return g_vref / (float)(1UL << g_bits); }

// Cubic Lagrange weights for nodes at -1, 0, +1, +2, evaluated at a. This is
// what places the voltage at the instant each current sample was taken.
static void lagrange4(double a, double* w) {
  w[0] = -a * (a - 1.0) * (a - 2.0) / 6.0;
  w[1] =  (a + 1.0) * (a - 1.0) * (a - 2.0) / 2.0;
  w[2] = -(a + 1.0) * a * (a - 2.0) / 2.0;
  w[3] =  (a + 1.0) * a * (a - 1.0) / 6.0;
}
// Magnitude response of that interpolator at theta radians per sample.
static double interpGain(const double* w, double theta) {
  double c1 = cos(theta), s1 = sin(theta);
  double c2 = cos(2.0 * theta), s2 = sin(2.0 * theta);
  double re = w[0] * c1 + w[1] + w[2] * c1 + w[3] * c2;
  double im = -w[0] * s1 + w[2] * s1 + w[3] * s2;
  return sqrt(re * re + im * im);
}

static void resetAnalyzer();

static void applyDerived() {
  g_fs = g_rate / (double)g_decim;
  g_dt = 1.0 / g_fs;
  g_scanUs = 1.0e6 / g_rate;
  g_lag_s = rankTime();
  uint32_t full = (1UL << g_bits) - 1UL;
  g_mid = (int32_t)(1UL << (g_bits - 1));
  g_clipLo = (int32_t)(full / 256);
  g_clipHi = (int32_t)(full - full / 256);
  g_back80 = (uint32_t)lround(g_rate / (double)g_mains_hz);

  g_KV = (double)lsb() * cfg.vGain;  g_BV = -(double)cfg.vOff * cfg.vGain;
  const float gain[2] = {cfg.iGain, cfg.iGainLo}, off[2] = {cfg.iOff, cfg.iOffLo};
  for (int r = 0; r < 2; r++) {
    g_KIr[r] = (double)lsb() * gain[r];  g_BIr[r] = -(double)off[r] * gain[r];
    g_irFsA[r] = 0.5 * (double)g_vref * fabs((double)gain[r]);
    double al = (g_lag_s - (double)cfg.phaseUs * 1.0e-6) / g_dt;
    // +-1 sample keeps every read inside processCycle's index budget (and is
    // +-125 us, +-2.7 deg at 60 Hz, at the defaults: far more than a CT needs)
    if (al < -1.0) al = -1.0;
    if (al >  1.0) al =  1.0;
    g_alphaR[r] = al;
    lagrange4(al, g_WR[r]);
  }
  g_alpha = g_alphaR[g_irCur];
  memcpy(g_W, g_WR[g_irCur], sizeof(g_W));

  g_N0 = g_fs / (double)g_mains_hz;
  int hmax = (int)(g_N0 / 2.0) - 1;
  if (hmax > H_MAX) hmax = H_MAX;
  if (g_harmN > hmax) g_harmN = (uint8_t)max(hmax, 1);
}

// ═════════════════════════════════════════════════════════════════════════════
//  acquisition: DMA buffers -> scan ring + boxcar -> metering ring
// ═════════════════════════════════════════════════════════════════════════════

static uint64_t g_wr = 0;          // metering samples written, ever
static uint64_t g_validFrom = 0;   // first metering sample after the latest gap
static uint64_t g_w80 = 0;         // scans written, ever
static uint64_t g_gapScan = 0;     // first scan after the latest gap ...
static uint64_t g_gapDec = 0;      // ... which starts metering sample g_gapDec
static uint64_t g_mapScan = 0;     // the last scan of the latest buffer ...
static uint32_t g_mapTs = 0;       // ... was taken at this micros()
static int64_t  g_accV = 0;
static double   g_accIA = 0.0;     // current, already in amps
static uint16_t g_accN = 0;
static uint8_t  g_accF = 0, g_pendF = 0;
static double   g_imean = 0.0;     // last cycle's mean current, for the peak trip

static void tripRelay(const char* why, float value, float limit);
static void rangeOverload(float amps);
static void capOnScan(float ia);

// A break in the sample stream. Deliberate ones (a restart, a source change)
// just restart the analyser. A loss is also counted in $STATUS gaps.
static void markGap() {
  g_accV = 0; g_accIA = 0.0; g_accN = 0; g_accF = 0;
  g_pendF |= FLG_GAP;
  g_validFrom = g_wr;
  g_gapScan = g_w80; g_gapDec = g_wr;
  resetAnalyzer();
}

// scan <-> metering sample, valid after the latest gap
static inline uint64_t scanOfDec(uint64_t j) {
  return j >= g_gapDec ? g_gapScan + (j - g_gapDec) * g_decim : g_gapScan;
}
static inline uint64_t decOfScan(uint64_t s) {
  return s >= g_gapScan ? g_gapDec + (s - g_gapScan) / g_decim : g_gapDec;
}
// scan <-> micros(), from the latest buffer's timestamp
static inline uint64_t scanAt(uint32_t t) {
  int64_t s = (int64_t)g_mapScan + (int64_t)llround((double)(int32_t)(t - g_mapTs) / g_scanUs);
  return s < 0 ? 0 : (uint64_t)s;
}
static inline uint32_t timeAt(double scan) {
  return g_mapTs + (uint32_t)(int32_t)llround((scan - (double)g_mapScan) * g_scanUs);
}

static inline void pushSample() {
  uint32_t idx = (uint32_t)g_wr & RING_MASK;
  g_rv[idx] = (float)g_accV / (float)g_decim;
  g_ri[idx] = (float)(g_accIA / (double)g_decim);
  g_rf[idx] = g_accF | g_pendF | (g_synth ? FLG_SYNTH : 0);
  g_pendF = 0;
  g_wr++;
  g_accV = 0; g_accIA = 0.0; g_accN = 0; g_accF = 0;
}

static void irHistPush(uint64_t idx, uint8_t r) {
  if (g_irHistN == 8) { memmove(g_irHist, g_irHist + 1, 7 * sizeof(IrHist)); g_irHistN = 7; }
  g_irHist[g_irHistN].idx = idx; g_irHist[g_irHistN].r = r; g_irHistN++;
}
// the range metering sample k was taken on (for its phase delay and $PWR rng)
static uint8_t rangeAt(uint64_t k) {
  for (int j = (int)g_irHistN - 1; j >= 0; j--) if (g_irHist[j].idx <= k) return g_irHist[j].r;
  return g_irHistN ? g_irHist[0].r : g_irCur;
}
// In an ns-scan buffer whose last scan was d us after an event, scan k was at
// d - (ns-1-k)*scanUs: the first scan at or after the event.
static int scanIndexAfter(int ns, double d, double scanUs) {
  double k = ceil((double)(ns - 1) - d / scanUs);
  if (k < 0.0) return 0;
  if (k > (double)ns) return ns;
  return (int)k;
}

static void drainAdc() {
  uint32_t t0 = micros();
  // the synthetic source: V in raw counts, counts = (real/gain + off)/lsb.
  // I straight in amps: an ideal front end, the same on either range.
  double syAv = 0, syAi = 0, syVo = 0, syStep = 0, syLag = 0, syPhi = 0;
  if (g_synth) {
    syAv = g_syn_vrms * M_SQRT2 / cfg.vGain / lsb();
    syAi = g_syn_irms * M_SQRT2 / sqrt(1.0 + (double)g_syn_h3 * g_syn_h3 * 1.0e-4);
    syVo = cfg.vOff / lsb();
    syStep = 2.0 * M_PI * g_syn_hz / g_rate;
    syLag  = 2.0 * M_PI * g_syn_hz * g_lag_s;     // I+ really is converted later
    syPhi  = g_syn_deg * M_PI / 180.0;
  }

  int guard = 0;
  while (adcAvailable() && guard++ < 96) {
    ScanBuf s = adcRead();
    if (s.gap && g_wr) { g_gaps++; markGap(); }
    const Sample* pa = s.a->data();
    const Sample* pb = s.b->data();
    // A range switch in flight: scans from kSw on were taken on the new range,
    // and up to kSet the front end was still settling. Buffers filled before
    // the switch are still coming out of the pool, so this goes by each
    // buffer's own timestamp, not by when it is read.
    int kSw = s.ns, kSet = s.ns;
    if (g_irPending) {
      double d = (double)(int32_t)(s.ts - g_irSwUs);   // switch -> this buffer's last scan, us
      kSw  = scanIndexAfter(s.ns, d, g_scanUs);
      kSet = scanIndexAfter(s.ns, d - (double)g_irSettleMs * 1000.0, g_scanUs);
    }
    const double Ka = g_KIr[g_irCur], Ba = g_BIr[g_irCur];
    const double Kb = g_KIr[g_irReq], Bb = g_BIr[g_irReq];
    for (int k = 0; k < s.ns; k++) {
      int32_t vp = pa[2 * k], ip = pa[2 * k + 1];
      int32_t vn = pb[2 * k], in = pb[2 * k + 1];
      uint8_t f = 0;
      if (vp <= g_clipLo || vp >= g_clipHi || (!g_v_se && (vn <= g_clipLo || vn >= g_clipHi))) f |= FLG_VCLIP;
      if (ip <= g_clipLo || ip >= g_clipHi || (!g_i_se && (in <= g_clipLo || in >= g_clipHi))) f |= FLG_ICLIP;
      int32_t v = vp - (g_v_se ? g_mid : vn);
      int32_t i = ip - (g_i_se ? g_mid : in);
      double ia = (k < kSw) ? (double)i * Ka + Ba : (double)i * Kb + Bb;
      if (g_synth) {
        float ph = (float)g_synPh;
        g_synPh += syStep;
        if (g_synPh >= 2.0 * M_PI) g_synPh -= 2.0 * M_PI;
        float pi_ = ph + (float)(syLag - syPhi);
        v = (int32_t)lrintf((float)syAv * sinf(ph) + (float)syVo);
        ia = syAi * (double)(sinf(pi_) + g_syn_h3 * 0.01f * sinf(3.0f * pi_));
        f = 0;
      }
      bool settling = false;
      if (k >= kSw) {
        if (!g_irHistDone) { irHistPush(g_wr, g_irReq); g_irHistDone = true; }
        if (k < kSet) {
          // Still settling after a range switch: the front end reads garbage
          // here. The scan is replaced by the one a nominal mains cycle
          // earlier: exact for DC, close for steady AC. It stays flagged.
          settling = true;
          f |= FLG_IRANGE;
          f &= (uint8_t)~FLG_ICLIP;
          if (g_w80 >= g_gapScan + g_back80) ia = g_xi[(uint32_t)(g_w80 - g_back80) & RAW_MASK];
        }
      }
      const uint32_t xi = (uint32_t)g_w80 & RAW_MASK;
      g_xv[xi] = (float)((double)v * g_KV + g_BV);
      g_xi[xi] = (float)ia;
      g_w80++;
      if (!settling) {
        const float fa = (float)ia;
        if (g_trippk > 0.0f && !g_trip_latch && fabsf(fa - (float)g_imean) > g_trippk)
          tripRelay("PK", fabsf(fa - (float)g_imean), g_trippk);
        capOnScan(fa);
        if (g_irCur == 1 && !g_irPending && ((f & FLG_ICLIP) || fabsf(fa) > cfg.rngUpA))
          rangeOverload(fabsf(fa));
      }
      g_accV += v; g_accIA += ia; g_accF |= f;
      if (++g_accN >= g_decim) pushSample();
    }
    g_mapScan = g_w80 - 1; g_mapTs = s.ts;
    // settled inside this buffer: the switch is done
    if (g_irPending && kSet < s.ns) { g_irCur = g_irReq; g_irPending = false; g_irReport = true; }
    adcRelease(s);
  }
  g_busyUs += micros() - t0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  analysis: crossings -> cycles -> windows
// ═════════════════════════════════════════════════════════════════════════════

static inline double vAt(uint64_t k) { return (double)g_rv[(uint32_t)k & RING_MASK] * g_KV + g_BV; }
static inline double iAt(uint64_t k) { return (double)g_ri[(uint32_t)k & RING_MASK]; }   // amps
// cubic (4-point Lagrange) interpolation at a fractional index, for the
// averaged cycle; reads floor(x)-1 .. floor(x)+2
static inline float vCub(double x) {
  uint64_t k = (uint64_t)x; double w[4]; lagrange4(x - (double)k, w);
  return (float)(w[0] * vAt(k - 1) + w[1] * vAt(k) + w[2] * vAt(k + 1) + w[3] * vAt(k + 2));
}
static inline float iCub(double x) {
  uint64_t k = (uint64_t)x; double w[4]; lagrange4(x - (double)k, w);
  return (float)(w[0] * iAt(k - 1) + w[1] * iAt(k) + w[2] * iAt(k + 1) + w[3] * iAt(k + 2));
}

// crossing detector
static uint64_t g_scan = 0;
static double   g_cycStart = -1.0;
static bool     g_cycReal = false, g_armed = false;
static double   g_cand = -1.0;
static double   g_base = 0.0;
static float    g_lastVpk = 0.0f;

// window accumulators
static SpanSums g_win;
static uint16_t g_winCycles = 0, g_winReal = 0;
static double   g_winRealL = 0.0;
static double   g_wVr[H_MAX + 1], g_wVi[H_MAX + 1], g_wIr[H_MAX + 1], g_wIi[H_MAX + 1];
static double   g_wVp[H_MAX + 1], g_wIp[H_MAX + 1];   // L-weighted |phasor|^2 (power average)
static float    g_avgAccV[AVG_M], g_avgAccI[AVG_M];
static uint16_t g_avgAccN = 0;
static double   g_winEnd = 0.0;
static uint32_t g_seq = 0, g_cycSeq = 0;

// last window, kept for !HGET / !AVGW
static float    g_hVr[H_MAX + 1], g_hVd[H_MAX + 1], g_hIr[H_MAX + 1], g_hId[H_MAX + 1];
static uint8_t  g_hN = 0;
static float    g_hHz = 0, g_hVthd = 0, g_hIthd = 0, g_hDpf = 0, g_hP1 = 0;
static float    g_avgV[AVG_M], g_avgI[AVG_M];
static uint16_t g_avgN = 0;
static float    g_avgHz = 0;

static void clearWindow() {
  memset(&g_win, 0, sizeof(g_win));
  g_win.vmax = g_win.imax = -1e30f;
  g_win.vmin = g_win.imin = 1e30f;
  g_winCycles = g_winReal = 0; g_winRealL = 0.0;
  memset(g_wVr, 0, sizeof(g_wVr)); memset(g_wVi, 0, sizeof(g_wVi));
  memset(g_wIr, 0, sizeof(g_wIr)); memset(g_wIi, 0, sizeof(g_wIi));
  memset(g_wVp, 0, sizeof(g_wVp)); memset(g_wIp, 0, sizeof(g_wIp));
  memset(g_avgAccV, 0, sizeof(g_avgAccV)); memset(g_avgAccI, 0, sizeof(g_avgAccI));
  g_avgAccN = 0;
}

static void resetAnalyzer() {
  g_cycStart = -1.0; g_cycReal = false; g_armed = false; g_cand = -1.0;
  if (g_scan < g_validFrom + 3) g_scan = g_validFrom + 3;
  g_win.flags |= FLG_GAP;
}

// Gain of the DECIM-scan boxcar at theta radians per metering sample:
// sin(theta/2) / (D sin(theta/2D)). 0.99991 at 60 Hz at the defaults; divided out.
static double boxcarGain(double theta) {
  if (g_decim <= 1 || theta <= 0.0) return 1.0;
  double d = sin(theta / (2.0 * g_decim));
  return d > 0.0 ? sin(theta / 2.0) / ((double)g_decim * d) : 1.0;
}

// RMS, power etc. from span sums. theta = radians per sample at the measured
// frequency; 0 (no crossings) corrects at the nominal mains frequency instead.
static void reduceSums(const SpanSums& s, double theta, PowerResult& r) {
  double L = s.L > 0 ? s.L : 1.0;
  double mv = s.sv / L, mi = s.si / L;
  double vv = s.svv / L, ii = s.sii / L, vi = s.svi / L;
  if (g_dcrem) { vv -= mv * mv; ii -= mi * mi; vi -= mv * mi; }
  if (theta <= 0.0) theta = 2.0 * M_PI * g_mains_hz * g_dt;
  double gain = interpGain(g_W, theta);
  if (gain < 0.5) gain = 1.0;
  double bg = boxcarGain(theta);
  if (bg < 0.5) bg = 1.0;
  r.vrms = vv > 0 ? (float)(sqrt(vv) / bg) : 0.0f;
  r.irms = ii > 0 ? (float)(sqrt(ii) / bg) : 0.0f;
  r.p    = (float)(vi / (gain * bg * bg));
  r.s    = r.vrms * r.irms;
  double qq = (double)r.s * r.s - (double)r.p * r.p;
  r.q  = qq > 0 ? (float)sqrt(qq) : 0.0f;
  r.pf = r.s > 1e-9f ? r.p / r.s : 0.0f;
  r.vdc = (float)mv; r.idc = (float)mi;
  float c = g_dcrem ? (float)mv : 0.0f, ci = g_dcrem ? (float)mi : 0.0f;
  r.vpk = max(s.vmax - c, c - s.vmin);
  r.ipk = max(s.imax - ci, ci - s.imin);
  r.hz = 0.0f;
}

static void finishWindow();
static void onCycle(const PowerResult& r, double a, double b, uint8_t flags);

// ∫ from 0 to u of the cubic Lagrange basis polynomial j, nodes -1, 0, 1, 2.
static inline double lagInt(int j, double u) {
  const double u2 = u * u, u3 = u2 * u, u4 = u3 * u;
  switch (j) {
    case 0:  return -(u4 / 4.0 - u3 + u2) / 6.0;
    case 1:  return (u4 / 4.0 - 2.0 * u3 / 3.0 - u2 / 2.0 + 2.0 * u) / 2.0;
    case 2:  return -(u4 / 4.0 - u3 / 3.0 - u2) / 2.0;
    default: return (u4 / 4.0 - u2 / 2.0) / 6.0;
  }
}

// Weight of node k in the integral over [a, b] (fractional ends) of the
// piecewise cubic through the samples. Interior nodes come out at exactly 1;
// only the three nodes nearest each end differ. (The trapezoid rule's end
// error grows with h^2 and read 0.1-0.2 % fake current THD, 2026-11-08.)
static double nodeWeight(int64_t k, int64_t ka, int64_t kb, double a, double b) {
  if (k >= ka + 3 && k <= kb - 2) return 1.0;
  double w = 0.0;
  for (int64_t s = k - 2; s <= k + 1; s++) {
    if (s < ka || s > kb) continue;
    double u0 = a - (double)s; if (u0 < 0.0) u0 = 0.0;
    double u1 = b - (double)s; if (u1 > 1.0) u1 = 1.0;
    if (u1 <= u0) continue;
    const int j = (int)(k - s + 1);
    w += lagInt(j, u1) - lagInt(j, u0);
  }
  return w;
}

// One span [a, b] of the metering stream, usually one mains cycle between two
// rising crossings. Reads samples floor(a)-2 .. floor(b)+4.
static void processCycle(double a, double b, uint8_t flags) {
  double L = b - a;
  if (L < 2.0) return;
  const int64_t ka = (int64_t)floor(a), kb = (int64_t)floor(b);
  const bool real = !(flags & FLG_NOCROSS);
  const uint8_t rr = rangeAt((uint64_t)ka);
  g_alpha = g_alphaR[rr];
  memcpy(g_W, g_WR[rr], sizeof(g_W));
  if (rangeAt((uint64_t)kb) != rr) flags |= FLG_IRANGE;

  SpanSums s;
  memset(&s, 0, sizeof(s));
  s.L = L;
  s.vmax = s.imax = -1e30f; s.vmin = s.imin = 1e30f;

  // harmonic phasors: theta(k) = 2 pi (k - a) / L, locked to this cycle's own
  // crossings. Current is analysed at theta + alpha*delta, when it was sampled.
  const int H = g_harmN;
  float cVr[H_MAX + 1], cVi[H_MAX + 1], cIr[H_MAX + 1], cIi[H_MAX + 1];
  memset(cVr, 0, sizeof(float) * (H + 1)); memset(cVi, 0, sizeof(float) * (H + 1));
  memset(cIr, 0, sizeof(float) * (H + 1)); memset(cIi, 0, sizeof(float) * (H + 1));
  const double delta = 2.0 * M_PI / L;
  const double th0 = ((double)(ka - 1) - a) * delta;
  const double dr = cos(delta), di = -sin(delta);
  const double ar = cos(g_alpha * delta), ai = -sin(g_alpha * delta);

  // Pass 1: power sums, peaks, and the fundamental phasors (h = 1).
  double zr = cos(th0), zi = -sin(th0);            // e^{-j theta(k)}
  double v1r = 0, v1i = 0, i1r = 0, i1i = 0;
  for (int64_t k = ka - 1; k <= kb + 2; k++) {
    const double w = nodeWeight(k, ka, kb, a, b);
    double v = vAt(k), i = iAt(k);
    double vt = g_W[0] * vAt(k - 1) + g_W[1] * v + g_W[2] * vAt(k + 1) + g_W[3] * vAt(k + 2);
    s.sv  += w * v;      s.si  += w * i;
    s.svv += w * v * v;  s.sii += w * i * i;
    s.svi += w * vt * i;
    s.flags |= g_rf[(uint32_t)k & RING_MASK];
    if (k > ka && k <= kb) {
      float fv = (float)v, fi = (float)i;
      if (fv > s.vmax) s.vmax = fv;
      if (fv < s.vmin) s.vmin = fv;
      if (fi > s.imax) s.imax = fi;
      if (fi < s.imin) s.imin = fi;
    }
    const double qr = zr * ar - zi * ai, qi = zr * ai + zi * ar;   // current's kernel
    v1r += w * v * zr; v1i += w * v * zi;
    i1r += w * i * qr; i1i += w * i * qi;
    double t = zr * dr - zi * di; zi = zr * di + zi * dr; zr = t;
  }
  s.flags |= flags;

  // Pass 2: harmonics 2..H on what is left after removing DC and the
  // fundamental (the quadrature's end error scales with what it integrates).
  const double k2 = 2.0 / L;
  const double A1vr = k2 * v1r, A1vi = k2 * v1i, A1ir = k2 * i1r, A1ii = k2 * i1i;
  const double dcv = s.sv / L, dci = s.si / L;
  cVr[1] = (float)v1r; cVi[1] = (float)v1i; cIr[1] = (float)i1r; cIi[1] = (float)i1i;
  if (H >= 2) {
    zr = cos(th0); zi = -sin(th0);
    for (int64_t k = ka - 1; k <= kb + 2; k++) {
      const double w = nodeWeight(k, ka, kb, a, b);
      const double qr = zr * ar - zi * ai, qi = zr * ai + zi * ar;
      const float wv = (float)(w * (vAt(k) - dcv - (A1vr * zr + A1vi * zi)));
      const float wi = (float)(w * (iAt(k) - dci - (A1ir * qr + A1ii * qi)));
      const float fzr = (float)zr, fzi = (float)zi, fqr = (float)qr, fqi = (float)qi;
      float pr = fzr, pi = fzi;
      float sr = fqr, si = fqi;
      for (int h = 2; h <= H; h++) {
        float t = pr * fzr - pi * fzi; pi = pr * fzi + pi * fzr; pr = t;
        cVr[h] += wv * pr; cVi[h] += wv * pi;
        t = sr * fqr - si * fqi; si = sr * fqi + si * fqr; sr = t;
        cIr[h] += wi * sr; cIi[h] += wi * si;
      }
      double t = zr * dr - zi * di; zi = zr * di + zi * dr; zr = t;
    }
  }

  PowerResult r;
  reduceSums(s, delta, r);
  r.hz = real ? (float)(g_fs / L) : 0.0f;

  g_win.L += s.L; g_win.sv += s.sv; g_win.si += s.si;
  g_win.svv += s.svv; g_win.sii += s.sii; g_win.svi += s.svi;
  g_win.vmax = max(g_win.vmax, s.vmax); g_win.vmin = min(g_win.vmin, s.vmin);
  g_win.imax = max(g_win.imax, s.imax); g_win.imin = min(g_win.imin, s.imin);
  g_win.flags |= s.flags;
  g_winCycles++;
  if (real) { g_winReal++; g_winRealL += L; }
  for (int h = 1; h <= H; h++) {
    g_wVr[h] += cVr[h]; g_wVi[h] += cVi[h];
    g_wIr[h] += cIr[h]; g_wIi[h] += cIi[h];
    g_wVp[h] += L * k2 * k2 * ((double)cVr[h] * cVr[h] + (double)cVi[h] * cVi[h]);
    g_wIp[h] += L * k2 * k2 * ((double)cIr[h] * cIr[h] + (double)cIi[h] * cIi[h]);
  }
  if (real) {
    for (int m = 0; m < AVG_M; m++) {
      double x = a + (double)m * L / AVG_M;
      g_avgAccV[m] += vCub(x);
      g_avgAccI[m] += iCub(x - g_alpha);
    }
    g_avgAccN++;
  }
  g_winEnd = b;

  // energy: every cycle, end to end, so nothing between windows is missed
  g_wh += (double)r.p * L * g_dt / 3600.0;
  g_base = r.vdc;
  g_imean = r.idc;
  g_lastVpk = r.vpk;

  onCycle(r, a, b, s.flags);
  if (g_winCycles >= g_cycles) finishWindow();
}

static float hysteresis() {
  float h = 0.2f * g_lastVpk;
  return h > g_vmin ? h : g_vmin;
}

static void onCrossing(double x) {
  double L = x - g_cycStart;
  if (g_cycReal) {
    if (L < 0.6 * g_N0) return;          // too soon: noise, keep the cycle going
    processCycle(g_cycStart, x, 0);
  } else {
    if (x <= g_cycStart) return;
    if (L >= 2.0) processCycle(g_cycStart, x, FLG_NOCROSS);
  }
  g_cycStart = x;
  g_cycReal = true;
}

static void serviceAnalysis() {
  uint32_t t0 = micros();
  if (g_wr > g_scan + RING_N / 2) {      // fell half a ring behind: skip ahead
    g_lostUs += (uint64_t)((double)(g_wr - 64 - g_scan) * g_dt * 1.0e6);
    g_validFrom = g_wr - 64; g_gaps++;
    resetAnalyzer();
  }
  if (g_scan < g_validFrom + 3) g_scan = g_validFrom + 3;

  uint32_t budget = 4096;
  while (g_scan + 6 <= g_wr && budget--) {   // processCycle reads up to kb+4
    const uint64_t k = g_scan;
    if (g_cycStart < 0.0) {
      g_cycStart = (double)k; g_cycReal = false; g_armed = false; g_cand = -1.0;
      g_scan++;
      continue;
    }
    double v = vAt(k) - g_base, vp = vAt(k - 1) - g_base;
    double h = hysteresis();
    if (v < -h) g_armed = true;
    if (g_armed) {
      if (vp <= 0.0 && v > 0.0)      g_cand = (double)(k - 1) + (-vp / (v - vp));
      else if (vp > 0.0 && v <= 0.0) g_cand = -1.0;
      if (g_cand >= 0.0 && v > h) {
        onCrossing(g_cand);
        g_armed = false; g_cand = -1.0;
      }
    }
    if (g_cycStart >= 0.0 && (double)k - g_cycStart > 1.6 * g_N0) {
      double b = g_cycStart + g_N0;
      processCycle(g_cycStart, b, FLG_NOCROSS);
      g_cycStart = b; g_cycReal = false;
    }
    g_scan++;
  }
  g_busyUs += micros() - t0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  relay and trips
// ═════════════════════════════════════════════════════════════════════════════

static void setLeds();
static void requestRange(uint8_t r, uint8_t why, float amps);

static inline void relayPinWrite(bool on) { digitalWrite(g_rlyPin, (on != g_rly_inv) ? HIGH : LOW); }

static void relayNow(bool on) {
  g_relay = on;
  relayPinWrite(on);
  if (on) g_trip_latch = false;
  g_ln.reset(); g_ln.print("$RLY,"); g_ln.println(on ? 1 : 0); emit();
  setLeds();
}

// what the capture engine needs from a relay opening it didn't ask for
static void capNoteOpen(uint8_t why, float amps);
static uint8_t g_autoOpenOn = 0;
static void cancelAutoOpen() { g_autoOpenOn = 0; }

// !RELAY. In auto range mode a close goes HIGH first and waits for the range
// relay to settle, so inrush never hits the LOW shunt.
static void setRelay(bool on) {
  cancelAutoOpen();
  if (!on) { g_closePending = false; relayNow(false); return; }
  if (cfg.rngMode == 0 && g_rngPin && (g_irWant != 0 || g_irCur != 0)) {
    requestRange(0, RW_CLOSE, -1.0f);
    g_closePending = true;
    return;
  }
  relayNow(true);
}

static void tripRelay(const char* why, float value, float limit) {
  g_trip_latch = true;
  g_closePending = false;
  relayPinWrite(false);
  g_relay = false;
  g_ln.reset(); g_ln.print("$TRIP,"); g_ln.print(why); g_ln.print(',');
  g_ln.print(value, 4); g_ln.print(','); g_ln.println(limit, 4); emit();
  capNoteOpen(why[0] == 'P' ? 5 : 7, value);
  relayNow(false);
}

static void capOnCycle(const PowerResult& r, double a, double b);

static void onCycle(const PowerResult& r, double a, double b, uint8_t flags) {
  g_cycSeq++;
  if (g_trip_a > 0.0f && !g_trip_latch && g_relay && r.irms > g_trip_a) tripRelay("RMS", r.irms, g_trip_a);
  // auto-delayed: HIGH only after more than RNGCYC overloaded cycles in a row
  if (cfg.rngMode == 3) {
    if (g_cycOver) {
      if (++g_overCycles > cfg.rngCyc) { requestRange(0, RW_OVER, g_cycOverPk); g_overCycles = 0; }
    } else {
      g_overCycles = 0;
    }
  }
  g_cycOver = false; g_cycOverPk = 0.0f;
  capOnCycle(r, a, b);

  if (g_smask & SM_CYC) {
    g_ln.reset(); g_ln.print("$CYC,"); g_ln.print(g_cycSeq); g_ln.print(',');
    g_ln.print(r.vrms, 3); g_ln.print(','); g_ln.print(r.irms, 4); g_ln.print(',');
    g_ln.print(r.p, 3);    g_ln.print(','); g_ln.print(r.hz, 3);   g_ln.print(',');
    g_ln.print(r.ipk, 4);  g_ln.print(','); g_ln.println(flags | (g_trip_latch ? FLG_TRIP : 0));
    emitLow();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  window results
// ═════════════════════════════════════════════════════════════════════════════

static float phaseDeg(double re, double im) { return (float)(atan2(im, re) * 180.0 / M_PI); }
static float wrapDeg(float d) { while (d > 180.0f) d -= 360.0f; while (d <= -180.0f) d += 360.0f; return d; }

static void emitHarm(bool bulk) {
  g_ln.reset();
  g_ln.print("$HARM,"); g_ln.print(g_seq); g_ln.print(','); g_ln.print(g_hN); g_ln.print(',');
  g_ln.print(g_hHz, 3); g_ln.print(','); g_ln.print(g_hVthd, 3); g_ln.print(',');
  g_ln.print(g_hIthd, 3); g_ln.print(','); g_ln.print(g_hDpf, 4); g_ln.print(',');
  g_ln.print(g_hP1, 3);
  for (int h = 1; h <= g_hN; h++) {
    g_ln.print(','); g_ln.print(g_hVr[h], 4); g_ln.print(','); g_ln.print(g_hVd[h], 1);
    g_ln.print(','); g_ln.print(g_hIr[h], 5); g_ln.print(','); g_ln.print(g_hId[h], 1);
  }
  g_ln.println(); if (bulk) emitLow(); else emit();
}

static void emitAvg(bool bulk) {
  g_ln.reset();
  g_ln.print("$AVG,"); g_ln.print(g_seq); g_ln.print(','); g_ln.print(AVG_M); g_ln.print(',');
  g_ln.print(g_avgN); g_ln.print(','); g_ln.print(g_avgHz, 3);
  for (int m = 0; m < AVG_M; m++) { g_ln.print(','); g_ln.print(g_avgV[m], 3); }
  for (int m = 0; m < AVG_M; m++) { g_ln.print(','); g_ln.print(g_avgI[m], 4); }
  g_ln.println(); if (bulk) emitLow(); else emit();
}

static bool rangeAutoActive();

static void finishWindow() {
  g_seq++;
  PowerResult r;
  double hz = (g_winReal > 0 && g_winRealL > 0) ? (double)g_winReal * g_fs / g_winRealL : 0.0;
  reduceSums(g_win, hz > 0 ? 2.0 * M_PI * hz * g_dt : 0.0, r);
  r.hz = (float)hz;

  // harmonics: phase from the coherent sum, magnitude from the power average
  const int H = g_harmN;
  const double WL = g_win.L > 0 ? g_win.L : 1.0;
  double v1d = 0.0;
  double vh2 = 0.0, ih2 = 0.0;
  const double th1 = (hz > 0 ? hz : (double)g_mains_hz) * 2.0 * M_PI * g_dt;
  for (int h = 1; h <= H; h++) {
    double bg = boxcarGain(h * th1);
    if (bg < 0.2) bg = 0.2;
    float vr = (float)(sqrt(g_wVp[h] / WL / 2.0) / bg);
    float ir = (float)(sqrt(g_wIp[h] / WL / 2.0) / bg);
    float vd = phaseDeg(g_wVr[h], g_wVi[h]);
    float id = phaseDeg(g_wIr[h], g_wIi[h]);
    if (h == 1) v1d = vd;
    g_hVr[h] = vr; g_hIr[h] = ir;
    g_hVd[h] = wrapDeg(vd - (float)(h * v1d));
    g_hId[h] = wrapDeg(id - (float)(h * v1d));
    if (h >= 2) { vh2 += (double)vr * vr; ih2 += (double)ir * ir; }
  }
  g_hN = (uint8_t)H; g_hHz = r.hz;
  g_hVthd = g_hVr[1] > 1e-9f ? (float)(100.0 * sqrt(vh2) / g_hVr[1]) : 0.0f;
  g_hIthd = g_hIr[1] > 1e-9f ? (float)(100.0 * sqrt(ih2) / g_hIr[1]) : 0.0f;
  float phi = (g_hId[1]) * (float)M_PI / 180.0f;
  g_hDpf = (g_hVr[1] > 1e-9f && g_hIr[1] > 1e-9f) ? cosf(phi) : 0.0f;
  g_hP1 = g_hVr[1] * g_hIr[1] * g_hDpf;

  if (g_avgAccN) {
    for (int m = 0; m < AVG_M; m++) { g_avgV[m] = g_avgAccV[m] / g_avgAccN; g_avgI[m] = g_avgAccI[m] / g_avgAccN; }
  } else {
    memset(g_avgV, 0, sizeof(g_avgV)); memset(g_avgI, 0, sizeof(g_avgI));
  }
  g_avgN = g_avgAccN; g_avgHz = r.hz;

  uint8_t fl = g_win.flags;
  if (g_winReal < g_winCycles) fl |= FLG_NOCROSS;
  const uint8_t wr = rangeAt((uint64_t)g_winEnd);

  // auto range down: RNGHOLD ms of window peaks under RNGDNA, in HIGH
  if (rangeAutoActive() && g_irReq == 0 && g_irWant == 0 && !g_irPending) {
    float pk = max(fabsf(g_win.imax), fabsf(g_win.imin));
    if (!(fl & (FLG_ICLIP | FLG_IRANGE)) && pk < cfg.rngDnA) {
      uint32_t now = millis();
      if (!g_rngQuietSince) g_rngQuietSince = now ? now : 1;
      else if (now - g_rngQuietSince >= cfg.rngHoldMs) { requestRange(1, RW_AUTO, pk); g_rngQuietSince = 0; }
    } else {
      g_rngQuietSince = 0;
    }
  } else {
    g_rngQuietSince = 0;
  }

  if (g_smask & SM_PWR) {
    // the protocol's flags: 16 = samples lost, the rest as stored
    uint8_t out = (fl & (FLG_NOCROSS | FLG_VCLIP | FLG_ICLIP | FLG_SYNTH | FLG_IRANGE)) |
                  (g_trip_latch ? FLG_TRIP : 0) | ((fl & (FLG_GAP | FLG_ADCERR)) ? 0x10 : 0);
    g_ln.reset();
    g_ln.print("$PWR,");
    g_ln.print(r.vrms, 4);  g_ln.print(','); g_ln.print(r.irms, 5);  g_ln.print(',');
    g_ln.print(r.p, 4);     g_ln.print(','); g_ln.print(r.s, 4);     g_ln.print(',');
    g_ln.print(r.q, 4);     g_ln.print(','); g_ln.print(r.pf, 4);    g_ln.print(',');
    g_ln.print(r.hz, 3);    g_ln.print(','); g_ln.print(r.vpk, 4);   g_ln.print(',');
    g_ln.print(r.ipk, 5);   g_ln.print(','); g_ln.print((float)g_wh, 6); g_ln.print(',');
    g_ln.print(g_winEnd * g_dt, 3); g_ln.print(',');
    g_ln.print((uint32_t)(g_win.L + 0.5)); g_ln.print(',');
    g_ln.print(hz > 0 ? (uint32_t)lround(g_fs / hz) : (uint32_t)lround(g_N0)); g_ln.print(',');
    g_ln.print(g_relay ? 1 : 0); g_ln.print(',');
    g_ln.print(out);        g_ln.print(',');
    g_ln.print(r.vdc, 4);   g_ln.print(','); g_ln.print(r.idc, 5);   g_ln.print(',');
    g_ln.println(wr);
    emit();
  }
  if (g_smask & SM_HARM) emitHarm(true);
  if (g_smask & SM_AVG)  emitAvg(true);
  setLeds();
  clearWindow();
}

// ═════════════════════════════════════════════════════════════════════════════
//  capture (!CAP), from the scan ring
// ═════════════════════════════════════════════════════════════════════════════
//
// States: 1 preparing (range relay moving), 2 armed (waiting for the
// trigger), 3 collecting the post-trigger part, 4 building (two passes:
// peak, then int16), 5 sending. A stored sample k is the mean of N scans at
// start + (k + (j+0.5)/N) * S, j = 0..N-1, S = scans per sample. The current
// scans are interpolated back by the V->I conversion lag first, so V and I
// in a capture are simultaneous.

static uint8_t  g_cs = 0;
static double   g_cS = 4.0;              // scans per stored sample
static uint16_t g_cN = 1;                // scans averaged per stored sample
static uint32_t g_cn = 0, g_cPre = 0;
static uint8_t  g_cCh = 2, g_cTrig = 0, g_cOvCyc = 3;
static bool     g_cOpenAfter = false;
static int8_t   g_cRng = -1;
static float    g_cLevel = 0, g_cIpk = 0, g_cOvA = 0;
static int32_t  g_cRelayUs = -1;
static uint32_t g_cTimeoutMs = 0, g_cAutoMs = 0, g_cArmMs = 0;
static double   g_cPos = 0;              // armed: next block to test (scan index)
static double   g_cPrev = 0;             // previous block's value (zero-cross)
static bool     g_cZArm = false, g_c6Arm = false, g_cWaitClose = false;
static double   g_cStart = 0;            // first scan of the capture (fractional grid origin)
static uint64_t g_cTrigScan = 0, g_cEndScan = 0, g_cOvRef = 0, g_cSafeFrom = 0;
static int64_t  g_cRelayScan = -1, g_cOpenScan = -1;
static uint8_t  g_cOpenWhy = 0, g_cOvCount = 0, g_cFlags = 0;
static uint32_t g_cK = 0, g_cSent = 0;
static float    g_cVmax = 0, g_cImax = 0, g_cVper = 1, g_cIper = 1;
static uint8_t  g_cPass = 0;
static uint32_t g_autoOpenUs = 0;        // safe-start auto-open deadline (micros)
static mbed::Timeout g_closeTimer;
static volatile bool     g_closeFired = false;
static volatile uint32_t g_closeFiredUs = 0;
static const char* const SAFE_NAMES[] = { "NONE", "INST", "OVERCURRENT", "TIMER", "END", "TRIPPK", "REMOTE", "TRIPA" };

static inline bool capSafeLive() { return g_cs >= 2 && g_cs <= 3; }

// the relay opened while a capture runs: remember where and why
static void capNoteOpen(uint8_t why, float amps) {
  if (g_cs >= 1 && g_cs <= 3 && g_cOpenScan < 0) { g_cOpenScan = (int64_t)scanAt(micros()); g_cOpenWhy = why; }
  if (g_cs >= 1 && g_cs <= 3 && why == 7) { g_ln.reset(); g_ln.print("$SAFE,TRIPA,"); g_ln.println(amps, 3); emit(); }
}

// safe start opening the relay (from drainAdc, onCycle or the loop)
static void safeOpen(uint8_t why, float amps) {
  if (!g_relay && !g_closePending) return;
  g_closePending = false;
  relayPinWrite(false);
  g_relay = false;
  if (why == 1 || why == 2) g_trip_latch = true;
  // where the relay really opened: now, not the (older) scan that set it off
  if (g_cs >= 1 && g_cs <= 4 && g_cOpenScan < 0) { g_cOpenScan = (int64_t)scanAt(micros()); g_cOpenWhy = why; }
  cancelAutoOpen();
  g_ln.reset(); g_ln.print("$SAFE,"); g_ln.print(SAFE_NAMES[why]); g_ln.print(',');
  g_ln.println(amps, 3); emit();
  relayNow(false);
}

// every scan, from drainAdc: the instant trip
static void capOnScan(float ia) {
  // scans from before the arm / the relay close are still coming out of the
  // DMA pool for a while: they can't be this load's current
  if (g_cIpk > 0.0f && capSafeLive() && g_relay && g_w80 >= g_cSafeFrom && fabsf(ia) > g_cIpk) safeOpen(1, fabsf(ia));
}

static uint64_t capRefScan() { return g_cTrig == 5 && g_cRelayScan >= 0 ? (uint64_t)g_cRelayScan : g_cTrigScan; }

static void capTriggered(uint64_t t);

// every mains cycle: the over-current check and the dropout trigger
static void capOnCycle(const PowerResult& r, double a, double b) {
  if (g_cs < 2 || g_cs > 3) return;
  const uint64_t sa = scanOfDec((uint64_t)a);
  if (g_cOvA > 0.0f && g_cOvRef && sa >= g_cOvRef) {
    if (r.irms > g_cOvA) { if (++g_cOvCount >= g_cOvCyc) safeOpen(2, r.irms); }
    else g_cOvCount = 0;
  }
  if (g_cs == 2 && g_cTrig == 6 && sa >= (uint64_t)g_cPos) {
    if (r.irms >= g_cLevel) g_c6Arm = true;
    else if (g_c6Arm) capTriggered(sa);
  }
}

static void capEnd() {
  g_cs = 0;
  g_closeTimer.detach();
  g_cWaitClose = false;
  if (g_capForce >= 0) {             // the capture pinned the range: back to the mode
    g_capForce = -1;
    if (cfg.rngMode == 1 || !g_rngPin) requestRange(0, RW_CMD, -1.0f);
    else if ((cfg.rngMode == 2 && !g_rngLatchHigh) || cfg.rngMode == 3) requestRange(1, RW_CMD, -1.0f);
  }
}

static void capAbort(const char* why) {
  g_ln.reset(); g_ln.print("$CAPTMO,"); g_ln.println(why); emit();
  if (g_cTrig == 5 && g_cOpenAfter && g_relay) safeOpen(4, 0.0f);
  capEnd();
}

// ISR: the scheduled relay close of an inrush capture
static void closeIsr() {
  relayPinWrite(true);
  g_relay = true;
  g_closeFiredUs = micros();
  g_closeFired = true;
}

static void closeForCapture() {
  relayNow(true);
  g_cRelayScan = (int64_t)scanAt(micros());
}

static void capTriggered(uint64_t t) {
  g_cTrigScan = t;
  g_cStart = (double)t - (double)g_cPre * g_cS;
  if (g_cStart < (double)g_gapScan) {      // can't reach across a gap / restart
    g_cStart = (double)g_gapScan;
    g_cFlags |= 0x10;
  }
  g_cEndScan = (uint64_t)ceil(g_cStart + (double)g_cn * g_cS) + 2;
  if (!g_cOvRef) g_cOvRef = t;
  if (g_cAutoMs) {
    g_autoOpenUs = timeAt((double)capRefScan()) + g_cAutoMs * 1000UL;
    g_autoOpenOn = 1;
  }
  g_cs = 3;
}

// mean of the N scans of the block starting at scan p (for the triggers)
static inline float blockV(double p) {
  float s = 0; for (uint16_t j = 0; j < g_cN; j++) s += g_xv[(uint32_t)(uint64_t)(p + (j + 0.5) * g_cS / g_cN) & RAW_MASK];
  return s / g_cN;
}
static inline float blockI(double p) {
  float s = 0; for (uint16_t j = 0; j < g_cN; j++) s += g_xi[(uint32_t)(uint64_t)(p + (j + 0.5) * g_cS / g_cN) & RAW_MASK];
  return s / g_cN;
}

// The rising zero near block p, to a fraction of a scan: a least-squares line
// through the scans around it (the sine is straight to 0.2 % within +-3 deg).
static double refineCrossing(double p) {
  const int64_t c = (int64_t)p, w = 24;
  double sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0;
  for (int64_t s = c - w; s <= c + w; s++) {
    if (s < (int64_t)g_gapScan || (uint64_t)s >= g_w80) continue;
    double x = (double)(s - c), y = (double)g_xv[(uint32_t)s & RAW_MASK] - g_base;
    sx += x; sy += y; sxx += x * x; sxy += x * y; n++;
  }
  double den = n * sxx - sx * sx;
  if (n < 8 || den <= 0) return p;
  double m = (n * sxy - sx * sy) / den, q = (sy - m * sx) / n;
  if (m <= 0) return p;
  double x0 = -q / m;
  if (x0 < -w || x0 > w) return p;
  return (double)c + x0;
}

static void serviceCapture() {
  if (g_autoOpenOn && (int32_t)(micros() - g_autoOpenUs) >= 0) { g_autoOpenOn = 0; safeOpen(3, 0.0f); }
  if (!g_cs) return;

  if (g_cs == 1) {                         // wait for the range relay
    if (g_irPending || g_irWant != g_irReq || millis() - g_irLastSwMs < (uint32_t)cfg.rngSetMs + 5) return;
    g_cs = 2;
    g_cArmMs = millis();
    g_cPos = (double)g_w80;
    g_cZArm = false; g_c6Arm = false; g_cWaitClose = false; g_closeFired = false;
    g_cSafeFrom = scanAt(micros());
    if (g_cTrig == 0) { capTriggered(g_w80); return; }
    if (g_cTrig == 5 || (g_cTrig == 3 && g_cRelayUs < 0)) {
      closeForCapture();
      g_cOvRef = g_cSafeFrom = (uint64_t)g_cRelayScan;
      g_cPos = (double)g_cRelayScan;
      if (g_cTrig == 3) { capTriggered((uint64_t)g_cRelayScan); return; }
    }
    return;
  }

  if (g_cs == 2) {
    if (g_cTimeoutMs && millis() - g_cArmMs > g_cTimeoutMs) { capAbort("TIMEOUT"); return; }
    if (g_cWaitClose) {
      if (!g_closeFired) return;
      g_cWaitClose = false;
      g_closeFired = false;
      g_trip_latch = false;
      g_ln.reset(); g_ln.println("$RLY,1"); emit();
      g_cRelayScan = (int64_t)scanAt(g_closeFiredUs);
      g_cOvRef = g_cSafeFrom = (uint64_t)g_cRelayScan;
      capTriggered((uint64_t)g_cRelayScan);
      return;
    }
    if (g_cTrig == 6) return;              // fired from capOnCycle
    int budget = max(1, 16000 / (int)g_cN);   // blocks, ~16k scan reads a pass
    const float h = hysteresis();
    while (budget-- > 0 && g_cPos + g_cS + 1.0 <= (double)g_w80) {
      const double p = g_cPos;
      bool hit = false;
      switch (g_cTrig) {
        case 1: case 5: hit = fabsf(blockI(p)) >= g_cLevel; break;
        case 4:         hit = fabsf(blockV(p)) >= g_cLevel; break;
        case 2: case 3: {
          float v = blockV(p) - (float)g_base;
          if (v < -h) g_cZArm = true;
          hit = g_cZArm && g_cPrev < 0.0f && v >= 0.0f;
          g_cPrev = v;
        } break;
      }
      g_cPos += g_cS;
      if (!hit) continue;
      if (g_cTrig != 3) { capTriggered((uint64_t)p); return; }
      // inrush at a phase: from this crossing, the next one far enough ahead,
      // then relay_us after it, on a hardware timer
      const double c = refineCrossing(p);
      const double hz = g_hHz > 1.0f ? (double)g_hHz : (double)g_mains_hz;
      const double perUs = 1.0e6 / hz;
      const uint32_t now = micros();
      double tc = (double)(int32_t)(timeAt(c) - now);          // us, relative to now (negative)
      double k = ceil((500.0 - (double)g_cRelayUs - tc) / perUs);
      if (k < 0) k = 0;
      double delay = tc + k * perUs + (double)g_cRelayUs;
      if (delay < 200.0) delay += perUs;
      g_closeFired = false;
      g_cWaitClose = true;
      g_closeTimer.attach(&closeIsr, std::chrono::microseconds((uint32_t)delay));
      return;
    }
    return;
  }

  if (g_cs == 3) {
    if (g_w80 < g_cEndScan) return;
    if (g_w80 - (uint64_t)g_cStart >= RAW_N - g_rate_req / 10) {   // fell behind the ring
      emitErr("capture overwritten before it was read; capture again");
      capEnd(); return;
    }
    if (g_cOpenAfter && g_relay) safeOpen(4, 0.0f);
    // flags over the span, from the metering ring
    for (uint64_t j = decOfScan((uint64_t)g_cStart); j <= decOfScan(g_cEndScan) && j < g_wr; j++) {
      uint8_t f = g_rf[(uint32_t)j & RING_MASK];
      g_cFlags |= f & (FLG_VCLIP | FLG_ICLIP);
      if (f & FLG_GAP && j > decOfScan((uint64_t)g_cStart)) g_cFlags |= 0x10;
    }
    g_cs = 4; g_cPass = 0; g_cK = 0; g_cVmax = 0; g_cImax = 0;
    return;
  }

  if (g_cs == 4) {
    // V at scan s; I at scan s moved back by the V->I lag (linear between scans)
    const float lagF = (float)(g_lag_s * 1.0e6 / g_scanUs);
    uint32_t picks = 0;
    // ~16k scan reads a pass keeps the loop near 2-3 ms (60k made it 15 ms)
    while (g_cK < g_cn && picks < 16000) {
      const double p = g_cStart + (double)g_cK * g_cS;
      float sv = 0, si = 0;
      for (uint16_t j = 0; j < g_cN; j++) {
        const uint64_t s = (uint64_t)(p + (j + 0.5) * g_cS / g_cN);
        const float i1 = g_xi[(uint32_t)s & RAW_MASK], i0 = g_xi[(uint32_t)(s - 1) & RAW_MASK];
        sv += g_xv[(uint32_t)s & RAW_MASK];
        si += i1 - lagF * (i1 - i0);
      }
      sv /= g_cN; si /= g_cN;
      picks += g_cN;
      if (g_cPass == 0) {
        if (fabsf(sv) > g_cVmax) g_cVmax = fabsf(sv);
        if (fabsf(si) > g_cImax) g_cImax = fabsf(si);
      } else {
        const int16_t qv = (int16_t)lrintf(sv / g_cVper), qi = (int16_t)lrintf(si / g_cIper);
        if (g_cCh == 2) { g_cq[2 * g_cK] = qv; g_cq[2 * g_cK + 1] = qi; }
        else g_cq[g_cK] = g_cCh == 0 ? qv : qi;
      }
      g_cK++;
    }
    if (g_cK < g_cn) return;
    if (g_cPass == 0) {
      g_cVper = g_cVmax > 0 ? g_cVmax / 32000.0f : 1e-6f;
      g_cIper = g_cImax > 0 ? g_cImax / 32000.0f : 1e-6f;
      g_cPass = 1; g_cK = 0;
      return;
    }
    // header
    const uint64_t d0 = decOfScan((uint64_t)g_cStart), d1 = decOfScan(g_cEndScan);
    const uint8_t rng0 = rangeAt(d0);
    int32_t rngIdx = -1;
    for (int j = 0; j < g_irHistN; j++)
      if (g_irHist[j].idx > d0 && g_irHist[j].idx <= d1) {
        rngIdx = (int32_t)(((double)scanOfDec(g_irHist[j].idx) - g_cStart) / g_cS);
        break;
      }
    auto idxOf = [](int64_t scan) -> int32_t {
      if (scan < 0) return -1;
      double k = ((double)scan - g_cStart) / g_cS;
      return (k < 0 || k >= (double)g_cn) ? -1 : (int32_t)k;
    };
    g_ln.reset();
    g_ln.print("$CAP,"); g_ln.print(g_cn); g_ln.print(','); g_ln.print(g_cCh); g_ln.print(',');
    g_ln.print(g_cS * g_scanUs, 4); g_ln.print(',');
    printNum(g_cVper); g_ln.print(",0,"); printNum(g_cIper); g_ln.print(",0,");
    g_ln.print((int32_t)lround(((double)g_cTrigScan - g_cStart) / g_cS)); g_ln.print(',');
    g_ln.print(g_cFlags); g_ln.print(',');
    g_ln.print(idxOf(g_cRelayScan)); g_ln.print(','); g_ln.print(idxOf(g_cOpenScan)); g_ln.print(',');
    g_ln.print(g_cOpenScan >= 0 ? g_cOpenWhy : 0); g_ln.print(',');
    g_ln.print(g_cN); g_ln.print(','); g_ln.print(rng0); g_ln.print(','); g_ln.print(rngIdx); g_ln.print(',');
    printNum(g_cIper); g_ln.print(",0,"); g_ln.println((uint32_t)cfg.rngSetMs * 1000UL);
    emit();
    g_cs = 5; g_cSent = 0;
    return;
  }

  if (g_cs == 5) {
    if (!anySink()) { capEnd(); return; }
    const uint32_t total = g_cn * (g_cCh == 2 ? 2 : 1);
    static const char HEX_[] = "0123456789ABCDEF";
    int lines = 48;
    while (g_cSent < total && lines-- > 0 && sinkBacklog() < CAP_BACKLOG) {
      char b[4 + 32 * 4 + 2];
      int n = 0;
      b[n++] = '$'; b[n++] = 'C'; b[n++] = 'D'; b[n++] = ',';
      for (int j = 0; j < 32 && g_cSent < total; j++, g_cSent++) {
        uint16_t u = (uint16_t)g_cq[g_cSent];
        b[n++] = HEX_[u >> 12]; b[n++] = HEX_[(u >> 8) & 15]; b[n++] = HEX_[(u >> 4) & 15]; b[n++] = HEX_[u & 15];
      }
      b[n++] = '\n';
      g_ln.reset(); g_ln.write((const uint8_t*)b, n); emit();
    }
    if (g_cSent >= total && sinkBacklog() < CAP_BACKLOG) {
      g_ln.reset(); g_ln.print("$CEND,"); g_ln.println(g_cn); emit();
      capEnd();
    }
  }
}

// !CAP: parse, check, and start preparing
static int parseFloats(const char* args, float* out, int n);
static void startCapture(const char* args) {
  if (g_cs >= 3) { emitErr("capture busy"); return; }
  float a[16];
  for (int k = 0; k < 16; k++) a[k] = 0;
  a[2] = 2; a[5] = 10; a[6] = 5000; a[7] = -1; a[10] = 3; a[13] = 1; a[14] = -1;
  int n = parseFloats(args, a, 16);
  if (n < 2) { emitErr("CAP needs <rate>,<n>[,...]"); return; }
  double rate = a[0] <= 0 ? g_rate : (double)a[0];
  if (rate > g_rate) rate = g_rate;
  if (rate < 1.0) rate = 1.0;
  g_cS = g_rate / rate;
  g_cCh = (uint8_t)constrain((int)a[2], 0, 2);
  g_cTrig = (uint8_t)constrain((int)a[3], 0, 6);
  g_cLevel = a[4];
  float pre = constrain(a[5], 0.0f, 100.0f);
  g_cTimeoutMs = a[6] > 0 ? (uint32_t)a[6] : 0;
  g_cRelayUs = (int32_t)a[7];
  g_cIpk = a[8]; g_cOvA = a[9]; g_cOvCyc = (uint8_t)max(1, (int)a[10]);
  g_cAutoMs = (uint32_t)constrain((long)a[11], 0L, 60000L);
  g_cOpenAfter = a[12] != 0;
  int avg = constrain((int)a[13], 1, 64);
  g_cN = (uint16_t)max(1, min(avg, (int)floor(g_cS)));
  g_cRng = (int8_t)constrain((int)a[14], -1, 1);
  if ((g_cTrig == 1 || g_cTrig == 5 || g_cTrig == 6) && g_cCh == 0) { emitErr("a current trigger needs the current channel"); return; }
  if ((g_cTrig == 2 || g_cTrig == 4 || (g_cTrig == 3 && g_cRelayUs >= 0)) && g_cCh == 1) {
    emitErr("a voltage trigger needs the voltage channel"); return;
  }
  uint32_t nmax = CAP_VALUES / (g_cCh == 2 ? 2 : 1);
  uint32_t nreq = (uint32_t)max(1.0f, a[1]);
  g_cFlags = 0;
  if (nreq > nmax) nreq = nmax;
  // the whole capture has to fit in the scan ring with room to read it out
  uint32_t nring = (uint32_t)((double)(RAW_N - g_rate_req) / g_cS);
  if (nreq > nring) { nreq = nring; g_cFlags |= 0x10; }
  g_cn = nreq;
  g_cPre = (uint32_t)lround(g_cn * pre / 100.0f);
  if (g_cPre >= g_cn) g_cPre = g_cn - 1;
  g_cRelayScan = -1; g_cOpenScan = -1; g_cOpenWhy = 0; g_cOvCount = 0; g_cOvRef = 0;
  g_cPrev = 0;
  // the range: pinned for this capture, or HIGH before an inrush close in auto
  if (g_cRng >= 0 && g_rngPin) {
    g_capForce = g_cRng;
    requestRange((uint8_t)g_cRng, RW_CAP, -1.0f);
  } else if ((g_cTrig == 3 || g_cTrig == 5) && cfg.rngMode == 0 && g_rngPin) {
    requestRange(0, RW_CLOSE, -1.0f);
  }
  g_cs = 1;
}

// ═════════════════════════════════════════════════════════════════════════════
//  !DC: averaged DC of both channels (calibration)
// ═════════════════════════════════════════════════════════════════════════════

static uint8_t  g_dcState = 0;           // 1 waiting for the range, 2 averaging
static uint32_t g_dcMs = 500;
static uint64_t g_dcFrom = 0;
static uint8_t  g_dcRng = 0;

static void serviceDc() {
  if (!g_dcState) return;
  if (g_dcState == 1) {
    if (g_irPending || g_irWant != g_irReq || millis() - g_irLastSwMs < (uint32_t)cfg.rngSetMs + 5) return;
    g_dcFrom = g_wr; g_dcRng = g_irCur; g_dcState = 2;
    return;
  }
  uint32_t n = (uint32_t)max(1.0, g_dcMs * 1e-3 * g_fs);
  if (n > RING_N / 2) n = RING_N / 2;
  if (g_wr < g_dcFrom + n) return;
  g_dcState = 0;
  if (g_dcFrom < g_validFrom) { emitErr("DC reading interrupted by a gap"); return; }
  double sv = 0, si = 0, svv = 0, sii = 0;
  uint8_t fl = 0;
  for (uint32_t j = 0; j < n; j++) {
    uint32_t idx = (uint32_t)(g_dcFrom + j) & RING_MASK;
    double v = g_rv[idx], i = g_ri[idx];
    sv += v; si += i; svv += v * v; sii += i * i; fl |= g_rf[idx];
  }
  double mv = sv / n, mi = si / n;
  double sdv = sqrt(max(0.0, svv / n - mv * mv)), sdi = sqrt(max(0.0, sii / n - mi * mi));
  const double gain = g_dcRng ? cfg.iGainLo : cfg.iGain, off = g_dcRng ? cfg.iOffLo : cfg.iOff;
  const double vAdc = mv * lsb();
  const double iAdc = (gain != 0) ? mi / gain + off : 0;
  const bool changed = rangeAt(g_dcFrom) != rangeAt(g_dcFrom + n - 1) || (fl & FLG_IRANGE);
  g_ln.reset(); g_ln.print("$DC,");
  g_ln.print(vAdc, 7); g_ln.print(','); g_ln.print(iAdc, 7); g_ln.print(',');
  g_ln.print((vAdc - cfg.vOff) * cfg.vGain, 5); g_ln.print(','); g_ln.print(mi, 6); g_ln.print(',');
  g_ln.print(sdv * lsb(), 7); g_ln.print(','); g_ln.print(gain != 0 ? sdi / fabs(gain) : 0.0, 7); g_ln.print(',');
  g_ln.print((uint32_t)(n * g_decim)); g_ln.print(',');
  g_ln.print((fl & (FLG_VCLIP | FLG_ICLIP)) | ((fl & FLG_GAP) ? 0x10 : 0)); g_ln.print(',');
  g_ln.print(g_dcRng); g_ln.print(','); g_ln.println(changed ? 1 : 0);
  emit();
}

// ═════════════════════════════════════════════════════════════════════════════
//  zeroing
// ═════════════════════════════════════════════════════════════════════════════

// !ZERO: V and the active current range, then the other range, then back.
// Each range is averaged only once its switch has settled.
static uint64_t g_zeroFrom = 0;
static bool     g_zeroArmed = false;
static uint8_t  g_zeroBack = 0;

static void printCfgRow(int i);

static void startZero() {
  g_zeroStep = 1; g_zeroArmed = false; g_zeroBack = g_irWant;
}

static void serviceZero() {
  if (!g_zeroStep) return;
  if (g_irPending || g_irWant != g_irReq || millis() - g_irLastSwMs < (uint32_t)cfg.rngSetMs + 5) {
    g_zeroArmed = false; return;
  }
  if (!g_zeroArmed) { g_zeroArmed = true; g_zeroFrom = g_wr; return; }
  uint32_t n = (uint32_t)(0.2 * g_fs);
  if (g_wr < g_zeroFrom + n) return;
  if (g_zeroFrom < g_validFrom) {
    g_zeroStep = 0; requestRange(g_zeroBack, RW_CMD, -1.0f);
    emitErr("Zero interrupted by a gap"); return;
  }
  double sv = 0, si = 0;
  for (uint32_t j = 0; j < n; j++) {
    uint32_t idx = (uint32_t)(g_zeroFrom + j) & RING_MASK;
    sv += g_rv[idx]; si += g_ri[idx];
  }
  // the ring's current is in amps with the old offset in it, so the new
  // offset is the old one plus that mean, back in ADC volts
  const uint8_t r = g_irCur;
  if (r == 0) cfg.iOff += (float)((si / n) / cfg.iGain);
  else        cfg.iOffLo += (float)((si / n) / cfg.iGainLo);
  if (g_zeroStep == 1) cfg.vOff = (float)(sv / n * lsb());
  g_dirty = true;
  applyDerived();
  if (g_zeroStep == 1 && g_rngPin) {
    g_zeroStep = 2; g_zeroArmed = false;
    requestRange(1 - r, RW_CMD, -1.0f);
    return;
  }
  g_zeroStep = 0;
  requestRange(g_zeroBack, RW_CMD, -1.0f);
  printCfgRow(cfgFind("VOFF")); printCfgRow(cfgFind("IOFF")); printCfgRow(cfgFind("IOFFLO"));
  emitOk("zero");
}

// ═════════════════════════════════════════════════════════════════════════════
//  current range switching
// ═════════════════════════════════════════════════════════════════════════════

static void setIrangePin(uint8_t r) {
  if (g_rngPin) digitalWrite(g_rngPin, ((r == 1) != g_irInv) ? HIGH : LOW);
}

static bool rangeAutoActive() {
  return (cfg.rngMode == 0 || cfg.rngMode == 3) && g_rngPin && g_capForce < 0 && !g_zeroStep;
}

// ask for a range; serviceRange() does it (and reports $RNG) when it can
static void requestRange(uint8_t r, uint8_t why, float amps) {
  if (!g_rngPin) r = 0;
  if (r == g_irWant) return;
  g_irWant = r; g_rngWhy = why; g_rngWhyAmps = amps;
}

// an overloaded scan on LOW (from drainAdc)
static void rangeOverload(float amps) {
  if (g_irWant != 1) return;               // already on the way up
  switch (cfg.rngMode) {
    case 3:
      if (g_capForce < 0) { g_cycOver = true; if (amps > g_cycOverPk) g_cycOverPk = amps; break; }
      // a capture pinned LOW: protect the shunt anyway
    default:
      if (cfg.rngMode == 2) g_rngLatchHigh = true;
      requestRange(0, RW_OVER, amps);
      break;
  }
}

static void startRangeSwitch(uint8_t r) {
  g_irReq = r;
  setIrangePin(r);
  g_irSwUs = micros();
  g_irPending = true; g_irHistDone = false;
  g_irLastSwMs = millis();
  g_irSwitches++;
  g_rngQuietSince = 0;
  g_ln.reset(); g_ln.print("$RNG,"); g_ln.print(r); g_ln.print(','); g_ln.print(RNG_WHY[g_rngWhy]);
  g_ln.print(','); g_ln.print(cfg.rngMode);
  if (g_rngWhyAmps >= 0) { g_ln.print(','); g_ln.print(g_rngWhyAmps, 3); }
  g_ln.println(); emit();
}

// RNGMODE (or the range pin) changed, or boot: go where the mode says
static void applyRangeMode(uint8_t why) {
  g_rngLatchHigh = false;
  uint8_t r = g_irWant;
  if (!g_rngPin || cfg.rngMode == 1) r = 0;
  else if (cfg.rngMode == 2 || cfg.rngMode == 3) r = 1;
  else if (why == RW_BOOT) r = 0;            // auto starts HIGH, drops after RNGHOLD
  if (r != g_irWant) requestRange(r, why, -1.0f);
}

static void serviceRange() {
  if (g_irPending && (!g_adcOn || millis() - g_irLastSwMs > 500)) {
    // no ADC data is coming to place the switch on: apply it from here on
    if (!g_irHistDone) irHistPush(g_wr, g_irReq);
    g_irCur = g_irReq; g_irPending = false; g_irReport = true;
  }
  if (g_irReport) {
    g_irReport = false;
    g_alpha = g_alphaR[g_irCur]; memcpy(g_W, g_WR[g_irCur], sizeof(g_W));
  }
  if (!g_irPending && g_irWant != g_irReq) startRangeSwitch(g_irWant);
  // a relay close that waited for HIGH
  if (g_closePending && !g_irPending && g_irCur == 0 && g_irWant == 0 &&
      millis() - g_irLastSwMs >= (uint32_t)cfg.rngSetMs + 5) {
    g_closePending = false;
    relayNow(true);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  config: apply, flash storage, $CFG / $DUMP
// ═════════════════════════════════════════════════════════════════════════════

static bool adcStart();
static void restartAcq() { if (adcStart()) clearWindow(); }

// copy cfg into the engine; `what` = AP_* bits that changed
static void applyConfig(uint8_t what) {
  const uint8_t oldRly = g_rlyPin, oldRng = g_rngPin;
  g_rate_req = cfg.rate; g_decim = cfg.decim; g_bits = cfg.bits; g_smp = cfg.smp;
  g_v_se = cfg.vSe; g_i_se = cfg.iSe;
  g_vref = cfg.adcVref; g_dcrem = cfg.dcRem; g_cycles = cfg.cycles;
  g_mains_hz = cfg.mainsHz; g_vmin = cfg.vMin; g_harmN = cfg.harmN;
  g_trip_a = cfg.tripA; g_trippk = cfg.tripPk;
  g_rlyPin = cfg.relayPin; g_rly_inv = cfg.relayInv;
  g_rngPin = cfg.rngPin; g_irInv = cfg.rngInv;
  g_irSettleMs = cfg.rngSetMs;
  if (what & AP_PINS) {
    if (oldRly != g_rlyPin && oldRly) { digitalWrite(oldRly, LOW); pinMode(oldRly, INPUT); }
    if (oldRng != g_rngPin && oldRng) { digitalWrite(oldRng, LOW); pinMode(oldRng, INPUT); }
    pinMode(g_rlyPin, OUTPUT); relayPinWrite(g_relay);
    if (g_rngPin) { pinMode(g_rngPin, OUTPUT); setIrangePin(g_irReq); }
  }
  if (what & AP_RESTART) restartAcq();
  else if (what & AP_DERIVED) applyDerived();
  if (what & AP_GAP) markGap();
  if (what & AP_ANALYZER) resetAnalyzer();
  if (what & AP_WINDOW) clearWindow();
  if (what & AP_RANGE) applyRangeMode(RW_CMD);
}

// The last 128 KB sector of the internal flash (bank 2). With the default
// "2MB M7 + M4 in SDRAM" split nothing else lives there. As a guard, a sector
// holding something other than an erased block or our own config is never
// erased: that would be another core's code.
static const uint32_t CFG_FLASH_ADDR = 0x081E0000UL;
static uint8_t  g_saveBuf[256] __attribute__((aligned(32)));
static volatile uint8_t g_saveState = 0;      // 1 running, 2 done ok, 3 failed
static char     g_saveErr[48] = "";
static rtos::Thread* g_saveThread = nullptr;

static bool flashReadCfg(Config& c) {
  mbed::FlashIAP f;
  if (f.init() != 0) return false;
  int r = f.read(&c, CFG_FLASH_ADDR, sizeof(Config));
  f.deinit();
  return r == 0 && cfgValid(c);
}

static bool flashSectorOurs() {
  mbed::FlashIAP f;
  if (f.init() != 0) return false;
  uint32_t head[4];
  f.read(head, CFG_FLASH_ADDR, sizeof(head));
  f.deinit();
  return head[0] == 0xFFFFFFFFUL || head[0] == CFG_MAGIC;
}

// mbed::FlashIAP erases inside a critical section, so the ~1 s erase stops
// everything, DMA interrupts included: measured 2026-11-08, a 957 ms gap per
// !SAVE (reported in $STATUS gaps / lostms like any other). Hence !SAVE is
// refused with the load relay closed, as on 120VMonitor's F405 build. Calling
// the HAL erase/program directly from this thread with interrupts on hung the
// board, so that route is not used. The thread just keeps the command handler
// from blocking; the stall itself is the erase.
static void saveTask() {
  mbed::FlashIAP f;
  if (f.init() != 0) { strcpy(g_saveErr, "flash init failed"); g_saveState = 3; return; }
  const uint32_t sec = f.get_sector_size(CFG_FLASH_ADDR);
  const uint32_t page = f.get_page_size();
  uint32_t len = (sizeof(Config) + page - 1) / page * page;
  if (len > sizeof(g_saveBuf)) { strcpy(g_saveErr, "config too big"); f.deinit(); g_saveState = 3; return; }
  if (f.erase(CFG_FLASH_ADDR, sec) != 0) { strcpy(g_saveErr, "flash erase failed"); f.deinit(); g_saveState = 3; return; }
  if (f.program(g_saveBuf, CFG_FLASH_ADDR, len) != 0) { strcpy(g_saveErr, "flash program failed"); f.deinit(); g_saveState = 3; return; }
  f.deinit();
  g_saveState = 2;
}

static void startSave() {
  if (g_saveState == 1) { emitErr("save already running"); return; }
  if (g_relay) { emitErr("open the load relay first: saving stalls the board for up to ~2 s"); return; }
  if (!flashSectorOurs()) { emitErr("the config sector holds other data; not erasing it"); return; }
  if (g_saveThread) { g_saveThread->join(); delete g_saveThread; g_saveThread = nullptr; }
  cfg.magic = CFG_MAGIC; cfg.version = CFG_VERSION; cfg.size = sizeof(Config);
  cfg.crc = crc32((const uint8_t*)&cfg, offsetof(Config, crc));
  memset(g_saveBuf, 0xFF, sizeof(g_saveBuf));
  memcpy(g_saveBuf, &cfg, sizeof(Config));
  g_saveState = 1;
  g_saveThread = new rtos::Thread(osPriorityNormal, 4096, nullptr, "cfgsave");
  g_saveThread->start(saveTask);
}

static void serviceSave() {
  if (g_saveState == 2) { g_saveState = 0; g_dirty = false; g_cfgSrc = "SAVED"; emitOk("save"); }
  else if (g_saveState == 3) { g_saveState = 0; emitErr(g_saveErr); }
}

static void printCfgRow(int i) {
  if (i < 0) return;
  const CfgKey& k = KEYS[i];
  g_ln.reset(); g_ln.print("$CFG,"); g_ln.print(k.key); g_ln.print(',');
  if (k.type == FT_FLOAT) printNum(cfgGet(k)); else g_ln.print((uint32_t)cfgGet(k));
  g_ln.print(',');
  if (k.type == FT_FLOAT) { printNum(k.lo); g_ln.print(','); printNum(k.hi); }
  else { g_ln.print((long)k.lo); g_ln.print(','); g_ln.print((long)k.hi); }
  g_ln.print(','); g_ln.println(k.help); emit();
}

static void uidStr(char* out) {
  snprintf(out, 25, "%08lX%08lX%08lX", (unsigned long)HAL_GetUIDw2(), (unsigned long)HAL_GetUIDw1(),
           (unsigned long)HAL_GetUIDw0());
}

static void printDump(const Config& c, const char* src) {
  char uid[26]; uidStr(uid);
  g_ln.reset(); g_ln.print("$DUMPHDR,fw="); g_ln.print(FW_VERSION); g_ln.print(",board=GIGA,uid=");
  g_ln.print(uid); g_ln.print(",cfgver="); g_ln.print(CFG_VERSION); g_ln.print(",src="); g_ln.print(src);
  g_ln.print(",dirty="); g_ln.println(g_dirty ? 1 : 0); emit();
  const Config keep = cfg;
  cfg = c;                                   // cfgGet reads cfg
  for (int i = 0; i < N_KEYS; i++) {
    const CfgKey& k = KEYS[i];
    g_ln.reset(); g_ln.print("$DUMP,"); g_ln.print(CFG_GROUP_NAMES[k.group]); g_ln.print(',');
    g_ln.print(k.key); g_ln.print(',');
    if (k.type == FT_FLOAT) printNum(cfgGet(k)); else g_ln.print((uint32_t)cfgGet(k));
    g_ln.println(); emit();
  }
  cfg = keep;
  emitStr("$DUMPEND");
}

// ═════════════════════════════════════════════════════════════════════════════
//  status lines
// ═════════════════════════════════════════════════════════════════════════════

static void printNet() {
  g_ln.reset();
  g_ln.print("$NET,"); g_ln.print(g_ipStr); g_ln.print(','); g_ln.print(CMD_PORT); g_ln.print(',');
  g_ln.print(g_wifiOk ? (int)WiFi.RSSI() : 0); g_ln.print(',');
  g_ln.print(g_wifiStatus); g_ln.print(','); g_ln.print(g_wifiPs); g_ln.print(',');
  g_ln.println(SECRET_SSID); emit();
}

static void printStatus() {
  char uid[26]; uidStr(uid);
  g_ln.reset();
  g_ln.print("$STATUS,fw="); g_ln.print(FW_VERSION); g_ln.print(",board=GIGA,uid="); g_ln.print(uid);
  g_ln.print(",relay="); g_ln.print(g_relay ? 1 : 0);
  g_ln.print(",rng="); g_ln.print(g_irCur); g_ln.print(",rngmode="); g_ln.print(cfg.rngMode);
  g_ln.print(",stream="); g_ln.print((g_smask & SM_PWR) ? 1 : 0);
  g_ln.print(",dirty="); g_ln.print(g_dirty ? 1 : 0); g_ln.print(",cfgsrc="); g_ln.print(g_cfgSrc);
  g_ln.print(",trip="); g_ln.print(g_trip_latch ? 1 : 0);
  g_ln.print(",capmax="); g_ln.print(CAP_VALUES);
  g_ln.print(",capspan_ms="); g_ln.print((uint32_t)((RAW_N - g_rate_req) * 1000.0 / g_rate));
  g_ln.print(",scanus4="); g_ln.print(g_scanUs, 3); g_ln.print(",scanus2="); g_ln.print(g_scanUs, 3);
  g_ln.print(",rate="); g_ln.print(g_rate, 1); g_ln.print(",fs="); g_ln.print(g_fs, 1);
  g_ln.print(",lag_us="); g_ln.print(g_lag_s * 1e6, 4); g_ln.print(",lincal="); g_ln.print(g_linCal ? 1 : 0);
  g_ln.print(",fsa0="); g_ln.print(g_irFsA[0], 4); g_ln.print(",fsa1="); g_ln.print(g_irFsA[1], 4);
  g_ln.print(",synth="); g_ln.print(g_synth ? 1 : 0);
  g_ln.print(",gaps="); g_ln.print(g_gaps); g_ln.print(",lostms="); g_ln.print((uint32_t)(g_lostUs / 1000ULL));
  g_ln.print(",loopms="); g_ln.print(g_loopMaxMs, 2);
  g_ln.print(",ip="); g_ln.print(g_ipStr); g_ln.print(",rssi="); g_ln.print(g_wifiOk ? (int)WiFi.RSSI() : 0);
  g_ln.print(",uptime="); g_ln.println(millis() / 1000UL);
  emit();
}

static void printInfo() {
  g_ln.reset();
  g_ln.print("$INFO,"); g_ln.print(measuredRate()); g_ln.print(','); g_ln.print(g_gaps); g_ln.print(',');
  g_ln.print(g_resyncs); g_ln.print(','); g_ln.print(g_loadPct, 1); g_ln.print(',');
  g_ln.print(g_loopMaxMs, 2); g_ln.print(','); g_ln.print((uint32_t)(g_wr - g_scan)); g_ln.print(',');
  g_ln.print(g_txTcp.drops()); g_ln.print(','); g_ln.print(g_txSer.drops()); g_ln.print(',');
  g_ln.print(g_tcpBusy); g_ln.print(','); g_ln.print(largestFreeBlock()); g_ln.print(',');
  g_ln.print(g_adcFixes); g_ln.print(','); g_ln.print(millis() / 1000.0f, 1); g_ln.print(',');
  g_ln.print(g_wifiReconnects); g_ln.print(','); g_ln.print((float)(g_lostUs / 1000ULL)); g_ln.print(',');
  g_ln.println(g_lowDrops);
  emit();
}

static void printBanner() {
  g_ln.reset(); g_ln.print("$READY,GIGA,"); g_ln.println(FW_VERSION); emit();
  printNet();
}

// ═════════════════════════════════════════════════════════════════════════════
//  commands
// ═════════════════════════════════════════════════════════════════════════════

// up to n comma-separated floats; returns how many were present
static int parseFloats(const char* args, float* out, int n) {
  int got = 0;
  const char* p = args;
  while (got < n && p && *p) {
    out[got++] = atof(p);
    p = strchr(p, ',');
    if (p) p++;
  }
  return got;
}

static void setKey(const char* args) {
  char key[16];
  const char* c = strchr(args, ',');
  size_t kl = c ? (size_t)(c - args) : strlen(args);
  if (!c || kl == 0 || kl >= sizeof(key)) { emitErr("SET needs <key>,<value>"); return; }
  memcpy(key, args, kl); key[kl] = 0;
  int i = cfgFind(key);
  if (i < 0) { g_ln.reset(); g_ln.print("$ERR,unknown key "); g_ln.println(key); emit(); return; }
  const CfgKey& k = KEYS[i];
  float v = atof(c + 1);
  if (isnan(v) || v < k.lo || v > k.hi) {
    g_ln.reset(); g_ln.print("$ERR,"); g_ln.print(k.key); g_ln.print(" must be ");
    if (k.type == FT_FLOAT) { printNum(k.lo); g_ln.print(".."); printNum(k.hi); }
    else { g_ln.print((long)k.lo); g_ln.print(".."); g_ln.print((long)k.hi); }
    g_ln.println(); emit(); return;
  }
  if (k.off == CF(bits)) {
    int b = (int)lroundf(v);
    if (b != 8 && b != 10 && b != 12 && b != 14 && b != 16) { emitErr("BITS must be 8, 10, 12, 14 or 16"); return; }
  }
  if ((k.apply & AP_PINS) && g_relay) { emitErr("open the load relay before changing pins"); return; }
  cfgPut(k, v);
  g_dirty = true;
  applyConfig(k.apply);
  if (k.off == CF(rate)) cfg.rate = g_rate_req;          // what it could do
  printCfgRow(i);
}

static void handleCommand(const char* cmd) {
  // as on 120VMonitor: any command stops a capture that is waiting for its trigger
  if ((g_cs == 1 || g_cs == 2) && strncmp(cmd, "!CAP,", 5) != 0) {
    capAbort("ABORT");
    if (strcmp(cmd, "!ABORT") == 0) return;
  }

  if (strcmp(cmd, "!ID") == 0) {
    g_ln.reset(); g_ln.print("$READY,GIGA,"); g_ln.println(FW_VERSION); emit();

  } else if (strcmp(cmd, "!START") == 0) {
    g_smask |= SM_PWR; emitOk("start");

  } else if (strcmp(cmd, "!STOP") == 0) {
    g_smask = 0; emitOk("stop");

  } else if (strcmp(cmd, "!STATUS") == 0) {
    printStatus();

  } else if (strcmp(cmd, "!CFG") == 0) {
    for (int i = 0; i < N_KEYS; i++) printCfgRow(i);
    emitStr("$CFGEND");

  } else if (strncmp(cmd, "!GET,", 5) == 0) {
    int i = cfgFind(cmd + 5);
    if (i < 0) emitErr("unknown key"); else printCfgRow(i);

  } else if (strncmp(cmd, "!SET,", 5) == 0) {
    setKey(cmd + 5);

  } else if (strcmp(cmd, "!SAVE") == 0) {
    startSave();

  } else if (strcmp(cmd, "!LOAD") == 0) {
    Config c;
    if (!flashReadCfg(c)) { emitErr("no saved config"); return; }
    if (g_relay && (c.relayPin != cfg.relayPin || c.rngPin != cfg.rngPin)) { emitErr("open the load relay first"); return; }
    cfg = c; g_dirty = false; g_cfgSrc = "SAVED";
    applyConfig(0xFF);
    emitOk("load");

  } else if (strcmp(cmd, "!DEFAULTS") == 0) {
    if (g_relay) { emitErr("open the load relay first"); return; }
    cfgDefaults(cfg); g_dirty = true; g_cfgSrc = "DEFAULTS";
    applyConfig(0xFF);
    emitOk("defaults");

  } else if (strcmp(cmd, "!DUMP") == 0) {
    printDump(cfg, "RAM");

  } else if (strcmp(cmd, "!DUMP,SAVED") == 0) {
    Config c;
    if (!flashReadCfg(c)) emitErr("no saved config"); else printDump(c, "SAVED");

  } else if (strncmp(cmd, "!RELAY,", 7) == 0) {
    setRelay(atoi(cmd + 7) != 0);

  } else if (strncmp(cmd, "!RANGE,", 7) == 0) {
    int m = atoi(cmd + 7);
    if (m < 0 || m > 3) { emitErr("RANGE must be 0-3"); return; }
    cfg.rngMode = (uint8_t)m; g_dirty = true;
    applyRangeMode(RW_CMD);
    // the reply: the range this mode is heading for
    g_ln.reset(); g_ln.print("$RNG,"); g_ln.print(g_irWant); g_ln.print(",CMD,"); g_ln.println(m); emit();

  } else if (strcmp(cmd, "!ZERO") == 0) {
    startZero();

  } else if (strcmp(cmd, "!EZERO") == 0) {
    g_wh = 0.0; emitOk("ezero");

  } else if (strcmp(cmd, "!DC") == 0 || strncmp(cmd, "!DC,", 4) == 0) {
    g_dcMs = cmd[3] == ',' ? (uint32_t)constrain(atol(cmd + 4), 10L, 4000L) : 500;
    g_dcState = 1;

  } else if (strncmp(cmd, "!CAP,", 5) == 0) {
    if (g_cs == 1 || g_cs == 2) capAbort("ABORT");
    startCapture(cmd + 5);

  } else if (strcmp(cmd, "!ABORT") == 0) {
    // nothing waiting

  } else if (strncmp(cmd, "!DACTEST", 8) == 0) {
    emitErr("no DAC test on this board");

  // ── Giga extras ──
  } else if (strncmp(cmd, "!SMASK,", 7) == 0) {
    g_smask = (uint8_t)atoi(cmd + 7); emitOk("smask");

  } else if (strcmp(cmd, "!STAT") == 0) {
    printInfo();

  } else if (strcmp(cmd, "!HGET") == 0) {
    emitHarm(false);

  } else if (strcmp(cmd, "!AVGW") == 0) {
    emitAvg(false);

  } else if (strcmp(cmd, "!WIFI") == 0) {
    if (g_wifiBusy) emitErr("Already joining");
    else { dropClient(); g_wifiOk = false; g_wifiReconnects++; startWifiConnect(true); printNet(); }

  } else if (strcmp(cmd, "!SCAN") == 0 && g_wifiBusy) {
    emitErr("Wi-Fi is joining; try again");

  } else if (strcmp(cmd, "!SCAN") == 0) {
    const char* fw = WiFi.firmwareVersion();
    int n = WiFi.scanNetworks();
    g_ln.reset(); g_ln.print("$SCAN,"); g_ln.print(fw); g_ln.print(','); g_ln.println(n); emit();
    for (int k = 0; k < n && k < 30; k++) {
      g_ln.reset(); g_ln.print("$AP,"); g_ln.print(WiFi.SSID(k)); g_ln.print(',');
      g_ln.print((int)WiFi.RSSI(k)); g_ln.print(','); g_ln.println((int)WiFi.encryptionType(k)); emit();
    }
    markGap();

  } else if (strncmp(cmd, "!SYNTH,", 7) == 0) {
    float a[6];
    int n = parseFloats(cmd + 7, a, 6);
    if (n >= 2) g_syn_vrms = a[1];
    if (n >= 3) g_syn_irms = a[2];
    if (n >= 4) g_syn_deg  = a[3];
    if (n >= 5) g_syn_h3   = a[4];
    if (n >= 6) g_syn_hz   = a[5]; else if (n >= 1 && a[0] != 0) g_syn_hz = g_mains_hz;
    g_synth = (n >= 1 && a[0] != 0.0f);
    markGap();
    g_ln.reset(); g_ln.print("$SYNTH,"); g_ln.print(g_synth ? 1 : 0); g_ln.print(',');
    g_ln.print(g_syn_vrms, 3); g_ln.print(','); g_ln.print(g_syn_irms, 4); g_ln.print(',');
    g_ln.print(g_syn_deg, 3); g_ln.print(','); g_ln.print(g_syn_h3, 3); g_ln.print(',');
    g_ln.println(g_syn_hz, 4); emit();

  } else {
    g_ln.reset(); g_ln.print("$ERR,unknown command: "); g_ln.println(cmd); emit();
  }
}

// line assembly, one per input (RxLine is in ac_types.h)
static RxLine g_rxSer = {{0}, 0}, g_rxTcp = {{0}, 0};

static void feed(RxLine& rx, int c) {
  if (c == '\n' || c == '\r') {
    if (rx.n) { rx.b[rx.n] = 0; handleCommand(rx.b); rx.n = 0; }
  } else if (rx.n < sizeof(rx.b) - 1) {
    rx.b[rx.n++] = (char)c;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Wi-Fi
// ═════════════════════════════════════════════════════════════════════════════

static uint32_t g_lastWifiTry = 0, g_lastWifiCheck = 0;

static void setLeds() {
  // the Giga's RGB LED is active-low. Red = tripped, green = a client is
  // connected (toggles each window), blue = no Wi-Fi.
  static bool tog = false;
  tog = !tog;
  digitalWrite(LEDR, g_trip_latch ? LOW : HIGH);
  digitalWrite(LEDG, (g_clientOn && tog) ? LOW : HIGH);
  digitalWrite(LEDB, g_wifiOk ? HIGH : LOW);
}

// Joining the network happens on its own RTOS thread. WiFi.begin() blocks for
// 5-20 s, and on the loop thread that was 20+ s of lost samples per attempt
// (measured 2026-11-08). The loop thread does not touch WiFi while
// g_wifiBusy is set; it picks up the result once the thread is done.
static rtos::Thread*  g_wifiThread = nullptr;

static void wifiTask() {
  int st = WiFi.status();
  if (st != WL_NO_MODULE) {
    if (g_wifiDisc) { WiFi.disconnect(); g_wifiDisc = false; }
    for (int k = 0; k < 2 && st != WL_CONNECTED; k++) {
      WiFi.begin(SECRET_SSID, SECRET_PASS);
      uint32_t t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
      st = WiFi.status();
    }
  }
  g_wifiResult = st;
  g_wifiDone = true;
  g_wifiBusy = false;
}

static void startWifiConnect(bool disconnectFirst) {
  if (g_wifiBusy) return;
  if (g_wifiThread) { g_wifiThread->join(); delete g_wifiThread; g_wifiThread = nullptr; }
  g_wifiDisc = disconnectFirst;
  g_wifiBusy = true;
  g_lastWifiTry = millis();
  strcpy(g_ipStr, "joining");
  // same priority as the loop: RTX round-robins equal priorities, and the loop
  // never sleeps, so a lower priority would starve the join
  g_wifiThread = new rtos::Thread(osPriorityNormal, 8192, nullptr, "wifijoin");
  g_wifiThread->start(wifiTask);
}

// The radio's power-save sleeps between beacons (pings 22-406 ms), and this
// core's lwIP allows 2 x 536 B in flight per connection (fixed in libmbed.a),
// so throughput is ~1 KB per round trip. Turning power-save off: 21 ms pings.
static void disableWifiPowerSave() {
  WhdSTAInterface* sta = static_cast<WhdSTAInterface*>(WiFiInterface::get_default_instance());
  whd_interface_t ifp = nullptr;
  g_wifiPs = -1;
  if (!sta || sta->wifi_get_ifp(&ifp) != 0 || !ifp) return;
  whd_wifi_disable_powersave(ifp);
  uint32_t mode = 0;
  if (whd_wifi_get_powersave_mode(ifp, &mode) == 0) g_wifiPs = (int)mode;
}

static void finishWifiConnect() {
  g_wifiDone = false;
  g_wifiStatus = g_wifiResult;
  if (g_wifiStatus == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(g_ipStr, sizeof(g_ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    g_server.begin();
    disableWifiPowerSave();
    g_wifiOk = true;
  } else {
    strcpy(g_ipStr, g_wifiStatus == WL_NO_MODULE ? "no-module" : "none");
    g_wifiOk = false;
  }
  setLeds();
  printNet();
}

static void dropClient() {
  if (g_clientOn) g_client.stop();
  g_clientOn = false;
  g_txTcp.clear();
  g_rxTcp.n = 0;
}

static void serviceNet() {
  uint32_t now = millis();
  if (g_wifiBusy) return;                 // the join thread owns WiFi right now
  if (g_wifiDone) finishWifiConnect();
  if (g_wifiOk && now - g_lastWifiCheck > 2000) {
    g_lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      g_wifiOk = false; dropClient(); strcpy(g_ipStr, "none"); setLeds(); printNet();
    }
  }
  if (!g_wifiOk) {
    if (g_wifiStatus != WL_NO_MODULE && now - g_lastWifiTry > 20000) {
      g_wifiReconnects++;
      startWifiConnect(true);
    }
    return;
  }
  WiFiClient c = g_server.available();
  if (c) {
    // newest connection wins: a half-open old socket must not hold the slot,
    // because the Giga's socket pool is tiny (the Giga_RemoteController lesson)
    dropClient();
    g_client = c;
    g_client.setSocketTimeout(0);   // non-blocking send; net_sink.h keeps the rest
    g_clientOn = true;
    printBanner();
    setLeds();
  }
  if (g_clientOn && !g_client.connected()) { dropClient(); setLeds(); }
  if (g_clientOn) {
    int guard = 512;
    while (g_client.available() && guard-- > 0) feed(g_rxTcp, g_client.read());
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  setup / loop
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  // the settings first, then the relay is driven open before anything else
  cfgDefaults(cfg);
  {
    Config c;
    if (flashReadCfg(c)) { cfg = c; g_cfgSrc = "SAVED"; }
  }
  g_rlyPin = cfg.relayPin; g_rly_inv = cfg.relayInv;
  pinMode(g_rlyPin, OUTPUT);
  relayPinWrite(false);
  g_rngPin = cfg.rngPin; g_irInv = cfg.rngInv;
  if (g_rngPin) { pinMode(g_rngPin, OUTPUT); setIrangePin(0); }   // HIGH at boot
  irHistPush(0, 0);
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, LOW);

  Serial.begin(115200);   // no wait: this board is meant to run headless

  SDRAM.begin();
  g_rv = (float*)SDRAM.malloc(RING_N * sizeof(float));
  g_ri = (float*)SDRAM.malloc(RING_N * sizeof(float));
  g_rf = (uint8_t*)SDRAM.malloc(RING_N);
  g_xv = (float*)SDRAM.malloc(RAW_N * sizeof(float));
  g_xi = (float*)SDRAM.malloc(RAW_N * sizeof(float));
  g_cq = (int16_t*)SDRAM.malloc(CAP_VALUES * sizeof(int16_t));
  if (!g_rv || !g_ri || !g_rf || !g_xv || !g_xi || !g_cq) {
    while (1) { Serial.println("$ERR,SDRAM allocation failed"); digitalWrite(LEDR, LOW); delay(1000); }
  }
  memset(g_rf, 0, RING_N);

  // measuring starts at once; the network joins in the background
  applyConfig(0);
  applyDerived();
  adcStart();
  clearWindow();
  g_gaps = 0;          // the start-up "gap" is not one
  applyRangeMode(RW_BOOT);
  startWifiConnect(false);
  setLeds();
  g_serOn = _SerialUSB.connected();
  printBanner();
}

void loop() {
  uint32_t now = micros();
  if (g_lastLoopUs) {
    uint32_t d = now - g_lastLoopUs;
    if (d > g_loopMaxUs) g_loopMaxUs = d;
  }
  g_lastLoopUs = now;
  g_serOn = _SerialUSB.connected();

  drainAdc();
  serviceAnalysis();
  drainAdc();
  serviceCapture();
  serviceDc();
  serviceZero();
  serviceRange();
  serviceSave();

  while (Serial.available()) feed(g_rxSer, Serial.read());
  serviceNet();
  serviceTx();

  // once a second: load and worst loop time, and the $INFO stream
  static uint32_t lastStat = 0, statInfo = 0;
  uint32_t ms = millis();
  if (ms - lastStat >= 1000) {
    float el = (float)(ms - lastStat) * 1000.0f;
    g_loadPct = lastStat ? 100.0f * (float)g_busyUs / el : 0.0f;
    g_loopMaxMs = g_loopMaxUs / 1000.0f;   // worst loop of the last second ($STATUS loopms, $INFO)
    g_loopMaxUs = 0;
    g_busyUs = 0;
    lastStat = ms;
    if ((g_smask & SM_INFO) && ++statInfo >= 2) { statInfo = 0; printInfo(); }
  }
}
