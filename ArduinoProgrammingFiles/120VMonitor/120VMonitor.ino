/*!
 * 120VMonitor — bench 120 VAC power monitor
 *
 * ── Hardware ────────────────────────────────────────────────────────────────
 *   Seeed XIAO RA4M1 (also builds for the UNO R4 — same ADC channel mapping),
 *   or Adafruit Feather STM32F405 (picked automatically; see "Feather" below)
 *   Isolation-amp front end, each output pair read pseudo-differentially:
 *     A0 (+) / A1 (−)   load current      (Feather: A4 / A5)
 *     A2 (+) / A3 (−)   mains voltage
 *   OLED on I2C (D4 SDA / D5 SCL; Feather SDA / SCL), 0x3C. OLED picks the
 *     screen: 1 SSD1306 128x64 (XIAO default), 2 Adafruit OLED FeatherWing
 *     128x32 (SSD1306, Feather default), 3 Adafruit FeatherWing 128x64
 *     (SH1107). Needs the Adafruit SH110X library. The FeatherWings' A/B/C
 *     buttons are on D9/D6/D5 — the IR, load relay and range relay pins — so
 *     they are not read, and must not be pressed: they short those lines to GND.
 *   Relay driver on RLYPIN (config key, default D6), driven OPEN at boot
 *   IR receiver (TSOP-style, active low) on IRPIN, default D9 — the pin needs
 *     an edge interrupt, which D10 (P109) does not have on the XIAO RA4M1
 *   Built-in NeoPixel: red at LIVEVAC (30) Vrms or above, green below
 *   Optional second current range: a relay on RNGPIN (0 = single range)
 *     selects the 0.1 ohm shunt (LOW range, IGAINLO/IOFFLO) when energized;
 *     de-energized is the 0.01 ohm HIGH range (IGAIN/IOFF), which is also
 *     what a reset or power loss leaves it in.
 *
 * ── Current ranges ──────────────────────────────────────────────────────────
 *   RNGMODE 0 auto: an overload goes HIGH, and after RNGHOLD ms of window
 *     peaks under RNGDNA it drops back to LOW. Closing the load relay goes
 *     HIGH first, so inrush never lands on the 0.1 ohm shunt.
 *   RNGMODE 1: HIGH only.   RNGMODE 2: LOW; an overload latches HIGH until
 *     the range is selected again.
 *   RNGMODE 3 auto-delayed: starts LOW and stays there through short
 *     overloads; only when |i| passes RNGUPA (or clips) in more than RNGCYC
 *     (2) mains cycles in a row does it go HIGH. It drops back to LOW like
 *     auto, and closing the load relay does NOT switch to HIGH first, so a
 *     short inrush is recorded clipped on the LOW range. While LOW the
 *     current reading tops out near 2.5 A, so TRIPPK and safe start cannot
 *     see a fault above that until the range goes HIGH.
 *   Overload = any single scan above RNGUPA (or clipped) in the LOW range, checked inside
 *   every hot loop (the same place as TRIPPK), so the relay is told within
 *   microseconds. For RNGSETMS after a switch the reading is the relay in
 *   motion, so the current trips (TRIPPK, safe start) ignore it.
 *
 * ── Remote (NEC) ────────────────────────────────────────────────────────────
 *   Relay open:   IROK primes a close, IRCONF within IRARMMS closes it;
 *                 IROK again cancels the prime.
 *   Relay closed: IROK opens it at once — checked inside the capture and
 *                 metering loops, so it acts within microseconds.
 *   Every decoded frame is reported as $IR,<addr>,<cmd> (hex) for learning
 *   a new remote; repeat frames (button held) are ignored.
 *
 * ── Why the ADC is driven by hand ───────────────────────────────────────────
 *   The core's analogRead() re-opens and re-configures the ADC on every call,
 *   and its scan mask only ever grows, so once four pins have been read each
 *   call converts all four of them. Here the pins are routed to the ADC once
 *   with analogRead() and then the ADC registers are driven directly: ONE
 *   single-scan converts all four channels back to back in a few microseconds.
 *
 *   That changes the measurement problem completely compared with
 *   ac_power_monitor: V and I land microseconds apart instead of half a
 *   sample apart. The hardware scans in channel-number order — AN000 (A1, I−),
 *   AN001 (A2, V+), AN002 (A3, V−), AN009 (A0, I+) — so both pairs are centred
 *   on the same instant and there is no inherent V/I skew to correct. PHASEUS
 *   is left for what the two front ends do differently.
 *
 * ── Feather STM32F405 ───────────────────────────────────────────────────────
 *   ADC1 and ADC2 run in dual injected-simultaneous mode: a scan converts V+
 *   (ADC1) with V− (ADC2) at one instant, then I+ with I− at the next, so each
 *   pair's two halves are sampled together and common-mode noise cancels at
 *   any frequency rather than only below a few kHz. The current pair lands one
 *   conversion (~1.9 us) after the voltage pair — 0.04 degrees at 60 Hz, well
 *   inside what PHASEUS calibrates out. Injected results each have their own
 *   data register, so an interrupt mid-scan cannot overrun anything, and
 *   adcScan() keeps exactly the same shape as on the RA4M1.
 *   Other differences: 12-bit ADC; current moves to A4/A5 so A0/A1 (the two
 *   DACs) stay free for !DACTEST; the arena is the F405's 64 KB core-coupled
 *   RAM, so a capture holds 32768 values instead of 7168; config lives in
 *   flash (no EEPROM on the chip), and a !SAVE stalls the board for ~1-2 s
 *   while the sector erases, so it is refused with the load relay closed.
 *
 * ── Metering (always running, also feeds the OLED) ──────────────────────────
 *   Each window is CYCLES+2 mains cycles cut into SPC equal time buckets per
 *   cycle; every scan landing in a bucket is averaged into one stored sample.
 *   The window is bracketed by interpolated rising zero crossings of V and
 *   trapezoid-integrated to those fractional endpoints, DC removed. PHASEUS
 *   pairs each current sample with a voltage cubic-interpolated to the
 *   compensating instant, and the interpolator's magnitude response is divided
 *   back out at the measured frequency (see ac_power_monitor's README for why
 *   both matter).
 *
 * ── Fast capture ────────────────────────────────────────────────────────────
 *   Paced samples (DWT cycle counter, so the grid is exact unless the
 *   requested rate outruns the converter — then flag LATE and the measured
 *   rate is reported), raw int16 differential counts into a circular buffer
 *   with pretrigger. Scanning one pair only roughly doubles the top rate.
 *   Each sample can be the mean of several scans (oversampling, avg arg).
 *
 * ── Commands (newline terminated) ───────────────────────────────────────────
 *   !START / !STOP            stream / stop $PWR lines
 *   !STATUS                   $STATUS line
 *   !CFG                      every key: $CFG,<key>,<value>,<min>,<max>,<help> … $CFGEND
 *   !DUMP[,SAVED]             snapshot for copying settings between boards:
 *                             $DUMPHDR,fw=,board=,uid=,cfgver=,src=<RAM|SAVED>,dirty=
 *                             then $DUMP,<group>,<key>,<value> per key, $DUMPEND.
 *                             SAVED reads the flash/EEPROM block (RAM untouched).
 *                             group: CAL analog calibration · FUSE TRIPA/TRIPPK ·
 *                             RANGE switching · METER window · WIRING pins ·
 *                             UI OLED/IR remote/pixel
 *   !GET,<key>                one $CFG row
 *   !SET,<key>,<value>        RAM only, immediate
 *   !SAVE / !LOAD / !DEFAULTS EEPROM
 *   !RELAY,<0|1>              open / close (a close clears a latched trip)
 *   !ZERO                     store present DC readings as VOFF / IOFF (RAM)
 *   !EZERO                    reset the energy accumulator
 *   !DC[,<ms>]                averaged DC reading of both channels (calibration)
 *   !CAP,<rate_hz>,<n>,<ch>,<trig>,<level>,<pre_pct>,<timeout_ms>,<relay_us>
 *        rate_hz   stored samples per second, 0 = as fast as the ADC goes
 *        n         samples per channel (clamped to the buffer)
 *        ch        0 = V, 1 = I, 2 = both
 *        trig      0 now · 1 |I| >= level A · 2 V rising zero crossing ·
 *                  3 close the relay (inrush) · 4 |V| >= level V ·
 *                  5 close the relay once the pretrigger is full, then
 *                    trigger on |I| >= level A (needs ch 1/2) ·
 *                  6 dropout: a mains cycle's Irms falls below level A after
 *                    a cycle at or above it (needs ch 1/2); trig_idx is the
 *                    start of the first low cycle, so a high pre_pct keeps
 *                    what led up to an over-current trip
 *        pre_pct   share of the buffer kept from before the trigger
 *        timeout   ms to wait for the trigger, 0 = until aborted (any serial
 *                  byte aborts too); trig 5 leaves the relay closed on a
 *                  timeout unless open_after
 *        relay_us  trig 3 only: -1 = close as soon as armed, otherwise close
 *                  this many us after a rising V zero crossing (needs ch 0/2)
 *      ,<ipk_a>,<ov_a>,<ov_cyc>,<auto_ms>,<open_after>   safe start, 0 = off:
 *        ipk_a     open the relay on any sample with |i| above this
 *        ov_a      open when the per-mains-cycle Irms stays above ov_a for
 *        ov_cyc    this many consecutive cycles (counted from the trigger,
 *                  or from the relay close for trig 5)
 *        auto_ms   open this long after the trigger (the relay close for
 *                  trig 3 and 5); keeps running after the capture ends, max
 *                  60 s; any !RELAY command cancels it
 *        open_after open as soon as the capture completes (trig 5: or times out)
 *        Recording continues after the relay opens.
 *      ,<avg>   oversampling, 1…64 (default 1): every stored sample is the mean
 *        of avg consecutive scans, spread evenly across the sample period
 *        (back to back at rate 0). Triggers and the over-current check see
 *        the averages; TRIPPK and the instant trip see every single scan.
 *      ,<rng>  -1 follow RNGMODE (V-only captures and auto-mode inrush
 *                captures go HIGH), 0 HIGH, 1 LOW
 *      ,<mon>  0 suspend: nothing but sampling until the capture ends (best
 *                timing). 1 readings while armed: the capture's own samples
 *                also make a $PWR line every CYCLES mains cycles (needs ch 2
 *                for power), so the meter and its log keep going. Each line
 *                goes out a field per sample; lower the rate or avg if the
 *                capture comes back LATE. Each reading also updates the
 *                screen, status light, Wh and auto-range like a metering
 *                window, and is checked against TRIPA (opens: $SAFE,TRIPA).
 *                With trig 0 the screen isn't told CAPTURE, so back-to-back
 *                captures (live persistence) leave the meter showing, and the
 *                maths and $PWR lines (up to 8 windows) wait until after the
 *                last sample, so sampling can't slip; only the TRIPA check
 *                runs at each window's end.
 *   !RANGE,<0|1|2|3>          RNGMODE (RAM): auto / HIGH / LOW / auto-delayed
 *   !ABORT                    stop waiting for a trigger ($CAPTMO,ABORT)
 *   !DACTEST,<hz>,<deg>[,<amp>]  Feather only: loopback test sines, A0 "voltage"
 *                             and A1 "current" lagging it by <deg>, <amp> V peak
 *                             about mid-rail (default 1.0); !DACTEST,0 stops
 *
 * ── Output ──────────────────────────────────────────────────────────────────
 *   $READY,120VMON,<fw>
 *   $PWR,<vrms>,<irms>,<w>,<va>,<var>,<pf>,<hz>,<vpk>,<ipk>,<wh>,<uptime_s>,
 *        <n>,<depth>,<relay>,<flags>,<vdc>,<idc>,<rng 0 HIGH / 1 LOW>
 *        flags: +1 no V zero crossings  +2 V clipped  +4 I clipped
 *               +8 trip latched  +16 window truncated  +32 capture ran late
 *   $DC,<v_adc>,<i_adc>,<v>,<i>,<v_sd_adc>,<i_sd_adc>,<scans>,<flags>,<rng>,<rng_changed>
 *   $CAP,<n>,<ch>,<dt_us>,<v_per_count>,<v_off_counts>,<i_per_count>,
 *        <i_off_counts>,<trig_idx>,<flags>,<relay_idx>,<open_idx>,<open_why>,<avg>,
 *        <rng>,<rng_idx>,<i_per_hi>,<i_off_hi>,<settle_us>
 *        i_per/i_off are for the range the data starts in (rng); from rng_idx
 *        on (-1 = never) the range was HIGH, scaled by i_per_hi/i_off_hi, and
 *        the first settle_us after it is the range relay moving
 *   $RNG,<0 HIGH|1 LOW>,<BOOT|CMD|AUTO|OVER|CLOSE|CAP>,<mode>[,<amps>]
 *        open_why: 0 none 1 instant 2 over-current 3 timer 4 end 5 TRIPPK 6 remote 7 TRIPA
 *   $SAFE,<INST|OVERCURRENT|TIMER|END|TRIPPK|REMOTE|TRIPA>,<amps>   relay opened by itself
 *   $IR,<addr>,<cmd>   $REMOTE,<PRIMED|CANCEL|TIMEOUT|CLOSE|OPEN>
 *   $DAC,<hz>,<deg>,<amp>   the test generator's actual settings (hz 0 = off)
 *   $CD,<hex>   4 hex digits per int16 sample, V,I interleaved when ch = 2
 *   $CEND,<n>   $CAPTMO,<TIMEOUT|ABORT>   $RLY,<0|1>   $TRIP,<RMS|PK>,<value>,<limit>
 *   $STATUS,…   $OK,<what>   $ERR,<message>
 */

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_NeoPixel.h>
#include "monitor_types.h"

#define FW_VERSION   "1.4"
#define CFG_VERSION  5
#define CFG_ADDR     0

#if defined(STM32F4xx)
  #define BOARD_F405 1
  #include <PeripheralPins.h>
#else
  #define BOARD_F405 0
#endif

#if BOARD_F405
// ── analog pins: A4/A5 current, A2/A3 voltage — A0/A1 are the DACs ────────────
static const uint8_t PIN_IP = A4, PIN_IN = A5, PIN_VP = A2, PIN_VN = A3;
#define ADC_MAX      4095
#define PIN_RGB      8                // the Feather's NeoPixel, PC0
#else
// ── analog pins: A0/A1 current, A2/A3 voltage ─────────────────────────────────
static const uint8_t PIN_IP = A0, PIN_IN = A1, PIN_VP = A2, PIN_VN = A3;
#define ADC_MAX      16383
#endif
#define CLIP_LO      4
#define CLIP_HI      (ADC_MAX - 4)

// ── one RAM arena, shared by metering (float pairs) and capture (int16) ───────
#if BOARD_F405
// The F405's 64 KB of core-coupled RAM. Only the CPU can reach it, which is
// all this needs, and the linker script never places anything there, so it is
// addressed directly and the main RAM is left alone. Not zeroed at boot; the
// arena is scratch.
#define ARENA_BYTES  65536
static uint32_t* const g_arena = (uint32_t*)CCMDATARAM_BASE;
#else
#define ARENA_BYTES  14336
static uint32_t g_arena[ARENA_BYTES / 4];
#endif
#define PAIR_MAX     ((uint16_t)(ARENA_BYTES / 8))      // 1792 float V/I pairs
#define CAP_SAMPLES  ((uint16_t)(ARENA_BYTES / 2))      // 7168 int16 samples
static float*   const g_vbuf = (float*)g_arena;
static float*   const g_ibuf = (float*)g_arena + PAIR_MAX;
static int16_t* const g_cap  = (int16_t*)g_arena;

// ── flags ─────────────────────────────────────────────────────────────────────
#define FLG_NOCROSS  0x01
#define FLG_VCLIP    0x02
#define FLG_ICLIP    0x04
#define FLG_TRIP     0x08
#define FLG_SHORT    0x10
#define FLG_LATE     0x20

// ── state ─────────────────────────────────────────────────────────────────────
static Config   cfg;
static bool     g_dirty      = false;
static bool     g_streaming  = false;
static bool     g_relay      = false;
static bool     g_tripLatch  = false;
static bool     g_abort      = false;      // a command arrived mid-window
static double   g_wh         = 0.0;
static uint32_t g_lastWinMs  = 0;
static float    g_scanUs4 = 0, g_scanUs2 = 0;   // measured scan times
static PowerResult g_last;
static bool     g_haveLast   = false;
// a relay opened inside a hot loop is reported from loop(): SafeWhy code, 0 = none
static volatile uint8_t g_pendingOpen = 0;

// ── IR remote: NEC decoded from falling edges in an interrupt ─────────────────
// A TSOP output idles high and goes low for each burst, so falling edge to
// falling edge gives leader 13.5 ms, repeat 11.25 ms, bit 0 1.125 ms, bit 1
// 2.25 ms. The ISR only runs while a remote transmits.
static volatile uint32_t ir_last = 0, ir_bits = 0, ir_frame = 0;
static volatile int8_t   ir_n = -1;           // -1 = waiting for a leader
static volatile bool     ir_ready = false;
static int8_t           g_irAttached = -1;   // pin currently attached
static bool             g_primed = false;
static uint32_t         g_primedAt = 0;

static uint32_t CYC_PER_US_ISR = 48;
static void irIsr() {
  uint32_t now = DWT->CYCCNT;
  uint32_t us = (now - ir_last) / CYC_PER_US_ISR;
  ir_last = now;
  if (us > 12500 && us < 14500) { ir_n = 0; ir_bits = 0; return; }
  if (ir_n < 0) return;
  if (us > 850 && us < 1450)       { ir_n++; }
  else if (us > 1850 && us < 2700) { ir_bits |= (1UL << ir_n); ir_n++; }
  else                             { ir_n = -1; return; }
  if (ir_n >= 32) { ir_frame = ir_bits; ir_ready = true; ir_n = -1; }
}

// Splits a frame; false when the command check byte is wrong.
static bool irDecode(uint32_t f, uint16_t& addr, uint8_t& cmd) {
  uint8_t a = f & 0xFF, na = (f >> 8) & 0xFF, c = (f >> 16) & 0xFF, nc = (f >> 24) & 0xFF;
  if ((uint8_t)(c ^ nc) != 0xFF) return false;
  addr = ((uint8_t)(a ^ na) == 0xFF) ? a : (uint16_t)(f & 0xFFFF);   // extended NEC
  cmd = c;
  return true;
}

// Hot-loop check: true (and the frame consumed) when the frame waiting is
// this remote's OK and the relay is closed. The caller opens the relay.
static inline bool irOpenRequested() {
  if (!ir_ready || !g_relay) return false;
  uint16_t a; uint8_t c;
  if (!irDecode(ir_frame, a, c) || a != cfg.irAddr || c != cfg.irOk) return false;
  ir_ready = false;
  return true;
}

// the capture auto-open timer outlives a capture that ends before it fires
static bool     g_autoOffArmed = false;
static uint32_t g_autoOffAt    = 0;      // millis()

// ADC channel numbers and scan masks, looked up from the core's pin table
static uint8_t  CH_IP, CH_IN, CH_VP, CH_VN;
static uint32_t MASK_ALL, MASK_V, MASK_I;

// peak trip, precomputed in counts for the hot loops
static bool     g_pkArmed = false;
static int32_t  g_pkLo, g_pkHi;
static uint8_t  g_pkPin = 0;               // set when the hot loop opened the relay
static float    g_pkVal = 0;

// current range: HIGH = 0.01 ohm (IGAIN/IOFF), LOW = 0.1 ohm (IGAINLO/IOFFLO)
#define RNG_HI 0
#define RNG_LO 1
enum RngWhy : uint8_t { RW_BOOT, RW_CMD, RW_AUTO, RW_OVER, RW_CLOSE, RW_CAP };
static const char* const RNG_WHY[] = { "BOOT", "CMD", "AUTO", "OVER", "CLOSE", "CAP" };
static uint8_t  g_rng = RNG_HI;
static float    g_ig = 1.0f, g_io = 0.0f;  // the active range's IGAIN / IOFF
static bool     g_upArmed = false;         // LOW-range overload check, raw counts
static bool     g_upDelay = false;         // ... counted per mains cycle (RNGMODE 3)
static bool     g_upHit = false;           // this cycle went over
static uint8_t  g_upRun = 0;               // overloaded cycles in a row
static int32_t  g_upVal = 0;               // the reading that went over, for the report
static uint32_t g_upCycStart = 0, g_cycLen = 1;
static int32_t  g_upLo, g_upHi;
static volatile bool g_rngEvt = false;     // the range changed: hot loops rescale / restart
static bool     g_rngLatched = false;      // RNGMODE 2 after an overload
static uint32_t g_rngAt = 0, g_overAt = 0; // millis of the last change / overload or close
static bool     g_blank = false;           // current trips ignored until g_blankEnd (cycles)
static uint32_t g_blankEnd = 0;
static int8_t   g_rngReport = -1;          // RngWhy to report from loop()
static float    g_overVal = 0;

// one object per screen type; the frame buffer is only allocated by begin()
static Adafruit_SSD1306 g_ssd64(128, 64, &Wire, -1), g_ssd32(128, 32, &Wire, -1);
static Adafruit_SH1107  g_sh64(64, 128, &Wire);
static Adafruit_GFX*    g_gfx = &g_ssd64;      // the screen in use
static uint8_t  g_oledType = 0;                // cfg.oled it was started as
static int16_t  g_oledH = 64;
static bool     g_oledOk = false;
static uint32_t g_lastDispMs = 0;

// ═══ config table ═════════════════════════════════════════════════════════════

#define CF(f) offsetof(Config, f)
static const CfgField CFG_FIELDS[] = {
  {"VGAIN",   FT_FLOAT, CF(vGain),   -1e6f, 1e6f,  "volts per ADC volt", CG_CAL},
  {"VOFF",    FT_FLOAT, CF(vOff),    -2.0f, 2.0f,  "ADC volts at 0 V", CG_CAL},
  {"IGAIN",   FT_FLOAT, CF(iGain),   -1e5f, 1e5f,  "HIGH range amps per ADC volt", CG_CAL},
  {"IOFF",    FT_FLOAT, CF(iOff),    -2.0f, 2.0f,  "HIGH range ADC volts at 0 A", CG_CAL},
  {"IGAINLO", FT_FLOAT, CF(iGainLo), -1e5f, 1e5f,  "LOW range amps per ADC volt", CG_CAL},
  {"IOFFLO",  FT_FLOAT, CF(iOffLo),  -2.0f, 2.0f,  "LOW range ADC volts at 0 A", CG_CAL},
  {"PHASEUS", FT_FLOAT, CF(phaseUs), -2000, 2000,  "current path delay vs voltage, us", CG_CAL},
  {"ADCVREF", FT_FLOAT, CF(adcVref), 1.0f,  5.5f,  "ADC reference volts", CG_CAL},
  {"MAINS",   FT_FLOAT, CF(mainsHz), 10.0f, 400.0f,"nominal mains Hz", CG_METER},
  {"SPC",     FT_U16,   CF(spc),     16,    256,   "stored samples per cycle", CG_METER},
  {"CYCLES",  FT_U8,    CF(cycles),  1,     30,    "mains cycles per window", CG_METER},
  {"TRIPA",   FT_FLOAT, CF(tripA),   0,     1000,  "relay trip Irms, 0=off", CG_FUSE},
  {"TRIPPK",  FT_FLOAT, CF(tripPk),  0,     1000,  "relay trip |i| peak, 0=off", CG_FUSE},
  {"RLYPIN",  FT_U8,    CF(relayPin),0,     21,    "relay driver pin", CG_WIRING},
  {"RLYINV",  FT_U8,    CF(relayInv),0,     1,     "relay active-low", CG_WIRING},
  {"OLED",    FT_U8,    CF(oled),    0,     3,     "0 off, 1 SSD1306 128x64, 2 FeatherWing 128x32, 3 FeatherWing 128x64 (SH1107)", CG_UI},
  {"OLEDADDR",FT_U8,    CF(oledAddr),0x3C,  0x3D,  "OLED I2C address", CG_UI},
  {"OLEDROT", FT_U8,    CF(oledRot), 0,     1,     "OLED 0 = right side up, 1 = upside down", CG_UI},
  {"IRPIN",   FT_U8,    CF(irPin),   0,     21,    "IR receiver pin, 0=off", CG_WIRING},
  {"IRADDR",  FT_U16,   CF(irAddr),  0,     65535, "remote NEC address", CG_UI},
  {"IROK",    FT_U8,    CF(irOk),    0,     255,   "NEC cmd: prime close / open", CG_UI},
  {"IRCONF",  FT_U8,    CF(irConfirm),0,    255,   "NEC cmd: confirm close", CG_UI},
  {"IRARMMS", FT_U16,   CF(irArmMs), 500,   30000, "prime waits this long, ms", CG_UI},
  {"PIXEL",   FT_U8,    CF(pixel),   0,     255,   "NeoPixel brightness, 0=off", CG_UI},
  {"LIVEVAC", FT_FLOAT, CF(liveVac), 1,     500,   "pixel red at/above this Vrms", CG_UI},
  {"RNGPIN",  FT_U8,    CF(rngPin),  0,     21,    "range relay pin, 0=one range", CG_WIRING},
  {"RNGINV",  FT_U8,    CF(rngInv),  0,     1,     "0: pin HIGH = LOW range, 1: pin HIGH = HIGH", CG_WIRING},
  {"RNGMODE", FT_U8,    CF(rngMode), 0,     3,     "0 auto, 1 HIGH, 2 LOW, 3 auto-delayed", CG_RANGE},
  {"RNGUPA",  FT_FLOAT, CF(rngUpA),  0.05f, 1000,  "LOW: any |i| above -> HIGH", CG_RANGE},
  {"RNGDNA",  FT_FLOAT, CF(rngDnA),  0,     1000,  "auto: window |i| pk below -> LOW", CG_RANGE},
  {"RNGHOLD", FT_U16,   CF(rngHoldMs),0,    60000, "auto: ms in HIGH before LOW", CG_RANGE},
  {"RNGSETMS",FT_U8,    CF(rngSetMs),1,     200,   "range relay settle ms", CG_RANGE},
  {"RNGCYC",  FT_U8,    CF(rngCyc),  0,     60,    "auto-delayed: HIGH after more than this many overloaded cycles in a row", CG_RANGE},
};
#undef CF
#define CFG_FIELD_COUNT (sizeof(CFG_FIELDS) / sizeof(CFG_FIELDS[0]))
static const char* const CFG_GROUP_NAMES[] = { "CAL", "FUSE", "RANGE", "METER", "WIRING", "UI" };

// explicit prototypes — the auto-prototype pass would otherwise hoist these
// above the types they mention
static float fieldGetFrom(const Config& c, const CfgField& f);
static float fieldGet(const CfgField& f);
static void  fieldSet(const CfgField& f, float v);
static const CfgField* findField(const char* key);

void configDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  memcpy(cfg.magic, "M120", 4);
  cfg.version  = CFG_VERSION;
  cfg.size     = sizeof(Config);
  // Placeholders until calibrated: assumes ~1 V differential at 170 V peak and
  // 2.5 A peak. The bench-supply calibration replaces all four numbers.
  cfg.vGain    = 170.0f;
  cfg.vOff     = 0.0f;
  cfg.iGain    = 2.5f;
  cfg.iOff     = 0.0f;
  cfg.phaseUs  = 0.0f;
  cfg.adcVref  = 3.3f;
  cfg.mainsHz  = 60.0f;
  cfg.spc      = 128;
  cfg.cycles   = 10;
  cfg.tripA    = 0.0f;
  cfg.tripPk   = 0.0f;
  cfg.relayPin = 6;
  cfg.relayInv = 0;
#if BOARD_F405
  cfg.oled     = 2;                // Adafruit OLED FeatherWing 128x32
#else
  cfg.oled     = 1;
#endif
  cfg.oledAddr = 0x3C;
  cfg.oledRot  = 0;
  cfg.irPin     = 9;
  cfg.irAddr    = 0x00;
  cfg.irOk      = 0x1C;
  cfg.irConfirm = 0x16;
  cfg.irArmMs   = 5000;
  cfg.pixel     = 40;
  cfg.liveVac   = 30.0f;
  cfg.iGainLo   = 0.25f;           // 10x the HIGH placeholder's sensitivity
  cfg.iOffLo    = 0.0f;
  cfg.rngUpA    = 2.2f;            // just under the 0.1 ohm shunt's 2.5 A full scale
  cfg.rngDnA    = 1.5f;
  cfg.rngHoldMs = 3000;
#if BOARD_F405
  cfg.rngPin    = 5;               // Feather: range relay on D5 (relay D6, IR D9 as above)
#else
  cfg.rngPin    = 0;               // single range until the relay is wired
#endif
  cfg.rngInv    = 0;
  cfg.rngMode   = 0;
  cfg.rngSetMs  = 20;
  cfg.rngCyc    = 2;
}

static uint16_t crc16_ccitt(const uint8_t* p, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*p++) << 8;
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
  }
  return c;
}
// offsetof, never sizeof-2: the struct has tail padding after crc
static uint16_t configCrc(const Config& c) {
  return crc16_ccitt((const uint8_t*)&c, offsetof(Config, crc));
}

#if BOARD_F405
// The F405 has no EEPROM; the core emulates one in the last flash sector
// (128 KB), and its EEPROM.put() flushes — erases that whole sector — once
// per byte. The block goes through the core's RAM copy instead and is
// flashed once.
static void eeRead(void* p, size_t n) {
  eeprom_buffer_fill();
  for (size_t k = 0; k < n; k++) ((uint8_t*)p)[k] = eeprom_buffered_read_byte(CFG_ADDR + k);
}
static void eeWrite(const void* p, size_t n) {
  eeprom_buffer_fill();
  for (size_t k = 0; k < n; k++) eeprom_buffered_write_byte(CFG_ADDR + k, ((const uint8_t*)p)[k]);
  eeprom_buffer_flush();
}
#define EE_GET(x) eeRead(&(x), sizeof(x))
#define EE_PUT(x) eeWrite(&(x), sizeof(x))
#else
#define EE_GET(x) EEPROM.get(CFG_ADDR, x)
#define EE_PUT(x) EEPROM.put(CFG_ADDR, x)
#endif

static bool g_migrated = false;
static bool g_migratedLo = false;          // came from v1/v2: IGAINLO was seeded

// A v1 block keeps its calibration; the new fields take their defaults.
static bool configMigrateV1() {
  ConfigV1 o;
  EE_GET(o);
  if (memcmp(o.magic, "M120", 4) != 0 || o.version != 1 || o.size != sizeof(ConfigV1) ||
      o.crc != crc16_ccitt((const uint8_t*)&o, offsetof(ConfigV1, crc))) return false;
  configDefaults();
  cfg.vGain = o.vGain; cfg.vOff = o.vOff; cfg.iGain = o.iGain; cfg.iOff = o.iOff;
  cfg.phaseUs = o.phaseUs; cfg.adcVref = o.adcVref; cfg.mainsHz = o.mainsHz;
  cfg.spc = o.spc; cfg.cycles = o.cycles; cfg.tripA = o.tripA; cfg.tripPk = o.tripPk;
  cfg.relayPin = o.relayPin; cfg.relayInv = o.relayInv; cfg.oled = o.oled; cfg.oledAddr = o.oledAddr;
  return true;
}

// A v4 block is the v5 layout minus rngCyc. Copy only up to rngCyc: the v4
// crc sits a padding byte later.
static_assert(offsetof(ConfigV4, oledRot) + 1 == offsetof(Config, rngCyc), "ConfigV4 must prefix Config");
static bool configMigrateV4() {
  ConfigV4 o;
  EE_GET(o);
  if (memcmp(o.magic, "M120", 4) != 0 || o.version != 4 || o.size != sizeof(ConfigV4) ||
      o.crc != crc16_ccitt((const uint8_t*)&o, offsetof(ConfigV4, crc))) return false;
  configDefaults();
  memcpy(&cfg, &o, offsetof(Config, rngCyc));
  cfg.version = CFG_VERSION;
  cfg.size = sizeof(Config);
  return true;
}

// A v3 block is the v4 layout minus oledRot at the end.
static_assert(offsetof(ConfigV3, crc) == offsetof(Config, oledRot), "ConfigV3 must prefix Config");
static bool configMigrateV3() {
  ConfigV3 o;
  EE_GET(o);
  if (memcmp(o.magic, "M120", 4) != 0 || o.version != 3 || o.size != sizeof(ConfigV3) ||
      o.crc != crc16_ccitt((const uint8_t*)&o, offsetof(ConfigV3, crc))) return false;
  configDefaults();
  memcpy(&cfg, &o, offsetof(ConfigV3, crc));
  cfg.version = CFG_VERSION;
  cfg.size = sizeof(Config);
  return true;
}

// A v2 block is the v3 layout minus the range fields at the end.
static bool configMigrateV2() {
  ConfigV2 o;
  EE_GET(o);
  if (memcmp(o.magic, "M120", 4) != 0 || o.version != 2 || o.size != sizeof(ConfigV2) ||
      o.crc != crc16_ccitt((const uint8_t*)&o, offsetof(ConfigV2, crc))) return false;
  configDefaults();
  memcpy(&cfg, &o, offsetof(ConfigV2, crc));
  cfg.version = CFG_VERSION;
  cfg.size = sizeof(Config);
  return true;
}

static bool configLoad() {
  Config t;
  EE_GET(t);
  if (memcmp(t.magic, "M120", 4) != 0 || t.version != CFG_VERSION ||
      t.size != sizeof(Config) || t.crc != configCrc(t)) {
    if (configMigrateV4() || configMigrateV3()) { g_migrated = true; return true; }
    if (!configMigrateV2() && !configMigrateV1()) return false;
    // the LOW range starts as the HIGH calibration scaled by the shunt ratio
    cfg.iGainLo = cfg.iGain * 0.1f;
    cfg.iOffLo  = cfg.iOff;
    g_migratedLo = true;
    g_migrated = true;
    return true;
  }
  cfg = t;
  return true;
}

static bool configSave();
static bool configSave() {
  cfg.crc = configCrc(cfg);
  EE_PUT(cfg);
  Config t;
  EE_GET(t);
  return memcmp(&t, &cfg, sizeof(Config)) == 0;
}

static float fieldGetFrom(const Config& c, const CfgField& f) {
  const uint8_t* b = (const uint8_t*)&c + f.offset;
  switch (f.type) {
    case FT_FLOAT: { float v; memcpy(&v, b, 4); return v; }
    case FT_U8:    return (float)*b;
    case FT_U16:   { uint16_t v; memcpy(&v, b, 2); return (float)v; }
  }
  return NAN;
}
static float fieldGet(const CfgField& f) { return fieldGetFrom(cfg, f); }

static void fieldSet(const CfgField& f, float v) {
  if (v < f.lo) v = f.lo;
  if (v > f.hi) v = f.hi;
  uint8_t* b = (uint8_t*)&cfg + f.offset;
  switch (f.type) {
    case FT_FLOAT: memcpy(b, &v, 4); break;
    case FT_U8:    *b = (uint8_t)lroundf(v); break;
    case FT_U16:   { uint16_t u = (uint16_t)lroundf(v); memcpy(b, &u, 2); } break;
  }
}

static const CfgField* findField(const char* key) {
  for (size_t k = 0; k < CFG_FIELD_COUNT; k++)
    if (strcasecmp(key, CFG_FIELDS[k].key) == 0) return &CFG_FIELDS[k];
  return nullptr;
}

static void printField(const CfgField& f) {
  Serial.print(F("$CFG,"));
  Serial.print(f.key); Serial.print(',');
  if (f.type == FT_FLOAT) Serial.print(fieldGet(f), 6);
  else                    Serial.print((long)fieldGet(f));
  Serial.print(','); Serial.print(f.lo, 4);
  Serial.print(','); Serial.print(f.hi, 4);
  Serial.print(','); Serial.println(f.help);
}

// ═══ timing ═══════════════════════════════════════════════════════════════════

static uint32_t CYC_PER_US = 48;
static inline uint32_t cyc() { return DWT->CYCCNT; }

static void cycInit() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CYC_PER_US = SystemCoreClock / 1000000UL;
}

// ═══ ADC, register level ══════════════════════════════════════════════════════

#if BOARD_F405

// Sample time code for every channel: 2 = 28 ADC clocks, plus 12 to convert,
// so each simultaneous pair takes ~1.9 us at the 21 MHz ADC clock.
#define F4_SMP 2

static int32_t g_rIP, g_rIN, g_rVP, g_rVN;   // the last scan's four conversions
static uint32_t g_mask = 0;

static uint8_t adcChannelOf(uint8_t pin) {
  return (uint8_t)STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_ADC));
}

// analogRead() is never called on this board, so the core never touches the
// ADCs and they are set up here from scratch.
static void adcInit() {
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
  (void)RCC->APB2ENR;
  pinMode(PIN_IP, INPUT_ANALOG); pinMode(PIN_IN, INPUT_ANALOG);
  pinMode(PIN_VP, INPUT_ANALOG); pinMode(PIN_VN, INPUT_ANALOG);

  CH_IP = adcChannelOf(PIN_IP); CH_IN = adcChannelOf(PIN_IN);
  CH_VP = adcChannelOf(PIN_VP); CH_VN = adcChannelOf(PIN_VN);
  MASK_I   = (1UL << CH_IP) | (1UL << CH_IN);
  MASK_V   = (1UL << CH_VP) | (1UL << CH_VN);
  MASK_ALL = MASK_I | MASK_V;

  // ADC clock: the smallest PCLK2 divider that stays within 36 MHz
  static const uint8_t DIVS[4] = { 2, 4, 6, 8 };
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint8_t d = 0;
  while (d < 3 && pclk2 / DIVS[d] > 36000000UL) d++;

  ADC1->CR2 = 0;
  ADC2->CR2 = 0;
  ADC123_COMMON->CCR = ((uint32_t)d << ADC_CCR_ADCPRE_Pos)
                     | (5U << ADC_CCR_MULTI_Pos);   // dual, injected simultaneous
  const uint8_t ch[4] = { CH_IP, CH_IN, CH_VP, CH_VN };
  uint32_t smpr1 = 0, smpr2 = 0;
  for (uint8_t k = 0; k < 4; k++) {
    if (ch[k] < 10) smpr2 |= (uint32_t)F4_SMP << (3 * ch[k]);
    else            smpr1 |= (uint32_t)F4_SMP << (3 * (ch[k] - 10));
  }
  ADC1->SMPR1 = smpr1;  ADC2->SMPR1 = smpr1;
  ADC1->SMPR2 = smpr2;  ADC2->SMPR2 = smpr2;
  ADC1->CR1 = ADC_CR1_SCAN;      // 12 bit; SCAN lets the injected group run two ranks
  ADC2->CR1 = ADC_CR1_SCAN;
  ADC2->CR2 = ADC_CR2_ADON;      // the slave starts with the master
  ADC1->CR2 = ADC_CR2_ADON;      // software start (JEXTEN = 0)
  delayMicroseconds(5);          // tSTAB
}

// ADC1 converts the + pins and ADC2 the − pins. With two ranks the injected
// sequence runs JSQ3 then JSQ4 (results in JDR1, JDR2); with one, JSQ4 alone.
static inline void adcMask(uint32_t m) {
  g_mask = m;
  uint32_t a, b;
  if (m == MASK_V)      { a = (uint32_t)CH_VP << 15; b = (uint32_t)CH_VN << 15; }
  else if (m == MASK_I) { a = (uint32_t)CH_IP << 15; b = (uint32_t)CH_IN << 15; }
  else {
    a = (1UL << 20) | ((uint32_t)CH_VP << 10) | ((uint32_t)CH_IP << 15);
    b = (1UL << 20) | ((uint32_t)CH_VN << 10) | ((uint32_t)CH_IN << 15);
  }
  ADC1->JSQR = a;
  ADC2->JSQR = b;
}

static inline void adcScan() {
  ADC1->SR = 0;
  ADC2->SR = 0;
  ADC1->CR2 |= ADC_CR2_JSWSTART;
  while (!(ADC1->SR & ADC_SR_JEOC)) {}
  while (!(ADC2->SR & ADC_SR_JEOC)) {}
  if (g_mask == MASK_V)      { g_rVP = ADC1->JDR1; g_rVN = ADC2->JDR1; }
  else if (g_mask == MASK_I) { g_rIP = ADC1->JDR1; g_rIN = ADC2->JDR1; }
  else { g_rVP = ADC1->JDR1; g_rIP = ADC1->JDR2; g_rVN = ADC2->JDR1; g_rIN = ADC2->JDR2; }
}

static inline int32_t rd(uint8_t ch) {
  if (ch == CH_VP) return g_rVP;
  if (ch == CH_VN) return g_rVN;
  if (ch == CH_IP) return g_rIP;
  return g_rIN;
}

#else   // RA4M1

static uint8_t adcChannelOf(uint8_t pin) {
  return (uint8_t)GET_CHANNEL(getPinCfgs(digitalPinToAnalogPin(pin), PIN_CFG_REQ_ADC)[0]);
}

static void adcInit() {
  analogReference(AR_DEFAULT);
  analogReadResolution(14);
  // routes each pin to the ADC and opens the unit at its native 14 bits
  analogRead(PIN_IP); analogRead(PIN_IN); analogRead(PIN_VP); analogRead(PIN_VN);

  CH_IP = adcChannelOf(PIN_IP); CH_IN = adcChannelOf(PIN_IN);
  CH_VP = adcChannelOf(PIN_VP); CH_VN = adcChannelOf(PIN_VN);
  MASK_I   = (1UL << CH_IP) | (1UL << CH_IN);
  MASK_V   = (1UL << CH_VP) | (1UL << CH_VN);
  MASK_ALL = MASK_I | MASK_V;

  R_ADC0->ADCSR_b.ADST = 0;
  R_ADC0->ADCSR_b.ADCS = 0;      // single scan
  R_ADC0->ADCSR_b.ADIE = 0;
  R_ADC0->ADCSR_b.TRGE = 0;      // software start only
  R_ADC0->ADADC = 0;             // no hardware addition
}

static inline void adcMask(uint32_t m) {
  R_ADC0->ADANSA[0] = (uint16_t)(m & 0xFFFF);
  R_ADC0->ADANSA[1] = (uint16_t)(m >> 16);
}

static inline void adcScan() {
  R_ADC0->ADCSR_b.ADST = 1;
  while (R_ADC0->ADCSR_b.ADST) {}
}

static inline int32_t rd(uint8_t ch) { return (int32_t)R_ADC0->ADDR[ch]; }

#endif

// Both pairs from the last scan, clip flags from the individual conversions
// (on a pseudo-differential pair a difference of zero could be two rails).
static inline int32_t pairV(uint8_t& flags) {
  int32_t a = rd(CH_VP), b = rd(CH_VN);
  if (a <= CLIP_LO || a >= CLIP_HI || b <= CLIP_LO || b >= CLIP_HI) flags |= FLG_VCLIP;
  return a - b;
}
static inline int32_t pairI(uint8_t& flags) {
  int32_t a = rd(CH_IP), b = rd(CH_IN);
  if (a <= CLIP_LO || a >= CLIP_HI || b <= CLIP_LO || b >= CLIP_HI) flags |= FLG_ICLIP;
  return a - b;
}

static float measureScanUs(uint32_t mask) {
  adcMask(mask);
  uint32_t t = cyc();
  for (uint8_t k = 0; k < 64; k++) adcScan();
  float us = (float)(cyc() - t) / (float)CYC_PER_US / 64.0f;
  adcMask(MASK_ALL);
  return us;
}

static inline float lsb() { return cfg.adcVref / (float)(ADC_MAX + 1); }

// ═══ relay / protection ═══════════════════════════════════════════════════════

static inline void relayDrive(bool on) {
  digitalWrite(cfg.relayPin, (on != (cfg.relayInv != 0)) ? HIGH : LOW);
}

void setRelay(bool on) {
  g_relay = on;
  relayDrive(on);
  Serial.print(F("$RLY,"));
  Serial.println(on ? 1 : 0);
}

static void relayPinInit(uint8_t oldPin) {
  if (oldPin != cfg.relayPin) pinMode(oldPin, INPUT);
  pinMode(cfg.relayPin, OUTPUT);
  relayDrive(g_relay);
}

// |i| > TRIPPK and the LOW-range overload, as raw differential counts in the
// active range, for the hot loops
static void armPeakTrip() {
  float l = lsb();
  g_pkArmed = (cfg.tripPk > 0.0f && g_ig != 0.0f);
  if (g_pkArmed) {
    float c0 = g_io / l;
    float dc = fabsf(cfg.tripPk / g_ig) / l;
    g_pkLo = (int32_t)floorf(c0 - dc);
    g_pkHi = (int32_t)ceilf(c0 + dc);
  }
  g_upArmed = (g_rng == RNG_LO && cfg.iGainLo != 0.0f);
  g_upDelay = g_upArmed && cfg.rngMode == 3;
  g_cycLen = (uint32_t)((float)SystemCoreClock / cfg.mainsHz);
  if (g_upArmed) {
    float c0 = cfg.iOffLo / l;
    float dc = fabsf(cfg.rngUpA / cfg.iGainLo) / l;
    g_upLo = (int32_t)floorf(c0 - dc);
    g_upHi = (int32_t)ceilf(c0 + dc);
  }
}

static void rangeOverload(int32_t icounts);

// auto-delayed: note an over-threshold (or clipped) LOW reading for this cycle
static inline void rangeUpHit(int32_t icounts) { g_upHit = true; g_upVal = icounts; }

// auto-delayed: at each mains-cycle boundary, count overloaded cycles in a
// row and go HIGH once there are more than RNGCYC
static inline void rangeUpCycle() {
  uint32_t now = cyc(), el = now - g_upCycStart;
  if (el < g_cycLen) return;
  if (el >= 2 * g_cycLen) {              // no samples for a while: count afresh
    g_upRun = 0; g_upHit = false; g_upCycStart = now;
    return;
  }
  g_upCycStart += g_cycLen;
  bool hit = g_upHit;
  g_upHit = false;
  if (!hit) { g_upRun = 0; return; }
  if (++g_upRun > cfg.rngCyc) { g_upRun = 0; rangeOverload(g_upVal); }
}

// a clipped LOW reading: an overload now, or one more hit when delayed
static inline void rangeClip(int32_t icounts) {
  if (g_upDelay) rangeUpHit(icounts); else rangeOverload(icounts);
}

// true while the range relay is still moving: the current reading is junk
static inline bool tripBlanked() {
  if (g_blank && (int32_t)(cyc() - g_blankEnd) >= 0) g_blank = false;
  return g_blank;
}

// Called from the hot loops with every raw current reading: a LOW-range
// overload goes HIGH at once (auto-delayed: after RNGCYC cycles of it);
// TRIPPK opens the load relay at once. Both are reported afterwards.
static inline void checkPeak(int32_t icounts) {
  if (g_upArmed && (icounts < g_upLo || icounts > g_upHi)) {
    if (!g_upDelay) { rangeOverload(icounts); return; }
    rangeUpHit(icounts);
  }
  if (g_upDelay) {
    rangeUpCycle();
    if (!g_upArmed) return;              // just went HIGH
  }
  if (g_pkArmed && g_relay && (icounts < g_pkLo || icounts > g_pkHi) && !tripBlanked()) {
    relayDrive(false);
    g_relay = false;
    g_tripLatch = true;
    g_pkPin = 1;
    g_pkVal = ((float)icounts * lsb() - g_io) * g_ig;
  }
}

static void reportPeakTrip() {
  if (!g_pkPin) return;
  g_pkPin = 0;
  Serial.println(F("$RLY,0"));
  Serial.print(F("$TRIP,PK,")); Serial.print(g_pkVal, 4);
  Serial.print(','); Serial.println(cfg.tripPk, 4);
}

// ═══ current range ════════════════════════════════════════════════════════════

static inline void rngDrive(uint8_t r) {
  if (cfg.rngPin) digitalWrite(cfg.rngPin, ((r == RNG_LO) != (cfg.rngInv != 0)) ? HIGH : LOW);
}

static void rangePinInit(uint8_t oldPin) {
  if (oldPin && oldPin != cfg.rngPin && oldPin != cfg.relayPin) pinMode(oldPin, INPUT);
  if (!cfg.rngPin) { g_rng = RNG_HI; return; }
  pinMode(cfg.rngPin, OUTPUT);
  rngDrive(g_rng);
}

// the active calibration and hot-loop thresholds for g_rng
static void rngCal() {
  g_ig = (g_rng == RNG_LO) ? cfg.iGainLo : cfg.iGain;
  g_io = (g_rng == RNG_LO) ? cfg.iOffLo  : cfg.iOff;
  armPeakTrip();
}

static void rngBlank() {
  g_blankEnd = cyc() + (uint32_t)cfg.rngSetMs * 1000UL * CYC_PER_US;
  g_blank = true;
}

// hot loop: LOW range overloaded -> HIGH now
static void rangeOverload(int32_t icounts) {
  g_overVal = ((float)icounts * lsb() - g_io) * g_ig;
  g_rng = RNG_HI;
  rngDrive(RNG_HI);
  rngCal();
  rngBlank();
  g_overAt = g_rngAt = millis();
  if (cfg.rngMode == 2) g_rngLatched = true;
  g_rngEvt = true;
  g_rngReport = RW_OVER;
}

// wait = block until the relay has settled
static void rangeSet(uint8_t r, uint8_t why, bool wait) {
  if (!cfg.rngPin) r = RNG_HI;
  if (r == g_rng) { rngCal(); return; }
  g_rng = r;
  g_upRun = 0; g_upHit = false; g_upCycStart = cyc();
  rngDrive(r);
  rngCal();
  rngBlank();
  g_rngAt = millis();
  g_rngEvt = true;
  g_rngReport = why;
  if (wait) delay(cfg.rngSetMs);
}

// whatever RNGMODE asks for right now; auto keeps the range it has
static void rangeRefresh(uint8_t why) {
  uint8_t r = g_rng;
  if (!cfg.rngPin || cfg.rngMode == 1) r = RNG_HI;
  else if (cfg.rngMode == 2) r = g_rngLatched ? RNG_HI : RNG_LO;
  else if (cfg.rngMode == 3 && why == RW_BOOT) r = RNG_LO;     // auto-delayed starts LOW
  rangeSet(r, why, false);
}

// auto mode: go HIGH before the load relay closes, so inrush never lands on
// the LOW shunt, then hold there for RNGHOLD
static void rangeBeforeClose() {
  if (cfg.rngPin && cfg.rngMode == 0 && g_rng == RNG_LO) rangeSet(RNG_HI, RW_CLOSE, true);
  g_overAt = millis();
}

// auto / auto-delayed, after a clean window: back to LOW once it has been quiet long enough
static void rangeAuto(const PowerResult& r) {
  if (!cfg.rngPin || (cfg.rngMode != 0 && cfg.rngMode != 3) || g_rng != RNG_HI) return;
  uint32_t now = millis();
  if ((r.flags & FLG_ICLIP) || r.ipk + fabsf(r.idc) >= cfg.rngDnA) { g_overAt = now; return; }
  if (now - g_overAt < cfg.rngHoldMs || now - g_rngAt < cfg.rngHoldMs) return;
  rangeSet(RNG_LO, RW_AUTO, false);
}

static void rangeReport() {
  if (g_rngReport < 0) return;
  uint8_t why = (uint8_t)g_rngReport;
  g_rngReport = -1;
  Serial.print(F("$RNG,")); Serial.print(g_rng);
  Serial.print(','); Serial.print(RNG_WHY[why]);
  Serial.print(','); Serial.print(cfg.rngMode);
  if (why == RW_OVER) { Serial.print(','); Serial.print(g_overVal, 3); }
  Serial.println();
  g_lastDispMs = 0;
}

// ═══ metering acquisition ═════════════════════════════════════════════════════

// maxn equal time buckets across dur_us; every scan in a bucket is averaged
// into one stored sample. dt comes from the bucket centres actually achieved.
uint16_t acquireWindow(uint16_t maxn, uint32_t dur_us, float& dt_s,
                       uint8_t& flags, uint16_t& depth) {
  if (maxn > PAIR_MAX) maxn = PAIR_MAX;
  adcMask(MASK_ALL);
  uint32_t bucket = (uint32_t)((uint64_t)dur_us * CYC_PER_US / maxn);
  if (bucket < 1) bucket = 1;
  uint32_t cap = dur_us * CYC_PER_US * 2UL + 50000UL * CYC_PER_US;

  uint16_t n = 0;
  uint32_t total = 0, first_mid = 0, last_mid = 0;
  uint32_t t0 = cyc();

  while (n < maxn) {
    if (Serial.available()) { g_abort = true; break; }
    if (g_autoOffArmed && g_relay && (int32_t)(millis() - g_autoOffAt) >= 0) {
      relayDrive(false); g_relay = false; g_autoOffArmed = false;
      g_pendingOpen = 3;                   // SAFE_TIMER, reported from loop()
    }
    if (irOpenRequested()) {
      relayDrive(false); g_relay = false; g_autoOffArmed = false;
      g_pendingOpen = 6;                   // SAFE_REMOTE
    }
    if ((uint32_t)(cyc() - t0) > cap) { flags |= FLG_SHORT; break; }
    if (g_rngEvt) break;                   // mixed ranges: the caller discards it

    uint32_t deadline = t0 + (uint32_t)(n + 1) * bucket;
    uint32_t a = cyc();
    int32_t sv = 0, si = 0;
    uint16_t c = 0;
    do {
      adcScan();
      int32_t i = pairI(flags);
      sv += pairV(flags);
      si += i;
      checkPeak(i);
      if (g_upArmed && (flags & FLG_ICLIP)) rangeClip(i);   // LOW clipped below RNGUPA
      c++;
    } while ((int32_t)(cyc() - deadline) < 0 && c < 4000);
    uint32_t b = cyc();

    g_vbuf[n] = (float)sv / (float)c;
    g_ibuf[n] = (float)si / (float)c;
    total += c;
    uint32_t mid = a + (b - a) / 2;
    if (n == 0) first_mid = mid;
    last_mid = mid;
    n++;
  }
  depth = n ? (uint16_t)(total / n) : 0;
  dt_s = (n > 1) ? (float)(last_mid - first_mid) / (float)CYC_PER_US * 1e-6f / (float)(n - 1) : 0.0f;
  return n;
}

// ═══ AC analysis ══════════════════════════════════════════════════════════════

static float  A_LSB;
static double A_VM, A_IM;
static int32_t A_M;          // integer part of the voltage shift, samples
static double  A_W[4];       // cubic weights for the fractional part

static inline double vAt(int32_t k) { return ((double)g_vbuf[k] * A_LSB - cfg.vOff) * cfg.vGain; }
static inline double iAt(int32_t k) { return ((double)g_ibuf[k] * A_LSB - g_io) * g_ig; }

static void integrandDC(int32_t k, double* o) { o[0] = vAt(k); o[1] = iAt(k); }

// Cubic Lagrange weights for nodes -1, 0, +1, +2 evaluated at a.
static void lagrange4(double a, double* w) {
  w[0] = -a * (a - 1.0) * (a - 2.0) / 6.0;
  w[1] =  (a + 1.0) * (a - 1.0) * (a - 2.0) / 2.0;
  w[2] = -(a + 1.0) * a * (a - 2.0) / 2.0;
  w[3] =  (a + 1.0) * a * (a - 1.0) / 6.0;
}

// Magnitude response of that interpolator at theta rad/sample.
static float interpGain(const double* w, double theta) {
  double c1 = cos(theta), s1 = sin(theta), c2 = cos(2 * theta), s2 = sin(2 * theta);
  double re = w[0] * c1 + w[1] + w[2] * c1 + w[3] * c2;
  double im = -w[0] * s1 + w[2] * s1 + w[3] * s2;
  return (float)sqrt(re * re + im * im);
}

// v^2, i^2 and v*i, with v taken PHASEUS earlier than i (cubic-interpolated)
// so a current front end that lags the voltage one is paired correctly.
// Reads k+A_M-1 … k+A_M+2.
static void integrandPower(int32_t k, double* o) {
  double v = vAt(k) - A_VM;
  double i = iAt(k) - A_IM;
  int32_t j = k + A_M;
  double vs = A_W[0] * (vAt(j - 1) - A_VM) + A_W[1] * (vAt(j) - A_VM)
            + A_W[2] * (vAt(j + 1) - A_VM) + A_W[3] * (vAt(j + 2) - A_VM);
  o[0] = v * v;
  o[1] = i * i;
  o[2] = vs * i;
}

// Mean over the continuous index range [a, b]: trapezoids on the grid with the
// two partial end segments removed. Evaluates fn at floor(a) … floor(b)+1.
static void integrateMean(IntegrandFn fn, uint8_t m, double a, double b, double* out) {
  int32_t ka = (int32_t)floor(a), kb = (int32_t)floor(b);
  double fa = a - ka, fb = b - kb;
  double g[3], gn[3];
  for (uint8_t j = 0; j < m; j++) out[j] = 0.0;
  fn(ka, g);
  for (int32_t k = ka; k <= kb; k++) {
    fn(k + 1, gn);
    for (uint8_t j = 0; j < m; j++) { out[j] += 0.5 * (g[j] + gn[j]); g[j] = gn[j]; }
  }
  double gka[3], gka1[3], gkb[3], gkb1[3];
  fn(ka, gka); fn(ka + 1, gka1); fn(kb, gkb); fn(kb + 1, gkb1);
  double span = b - a;
  for (uint8_t j = 0; j < m; j++) {
    double at_a = gka[j] + fa * (gka1[j] - gka[j]);
    double at_b = gkb[j] + fb * (gkb1[j] - gkb[j]);
    out[j] -= 0.5 * fa * (gka[j] + at_a);
    out[j] -= 0.5 * (1.0 - fb) * (at_b + gkb1[j]);
    out[j] /= span;
  }
}

bool analyzePower(uint16_t n, float dt_s, PowerResult& r) {
  r.flags = 0; r.n = n; r.dt_s = dt_s; r.span_s = 0; r.hz = 0;
  if (n < 16 || dt_s <= 0.0f) return false;

  A_LSB = lsb();
  A_VM = A_IM = 0.0;

  // voltage shift in samples: i[k] pairs with v at k - phase/dt
  double alpha = -(double)cfg.phaseUs * 1e-6 / (double)dt_s;
  A_M = (int32_t)floor(alpha);
  lagrange4(alpha - A_M, A_W);

  // index budget: integrandPower reads k+A_M-1 … k+A_M+2 for k up to floor(b)+1
  int32_t lo = 1 - A_M; if (lo < 0) lo = 0;
  int32_t hi = (int32_t)n - 4 - A_M; if (hi > (int32_t)n - 2) hi = n - 2;
  if (hi - lo < 8) return false;

  double rough = 0.0;
  for (int32_t k = lo; k <= hi; k++) rough += vAt(k);
  rough /= (double)(hi - lo + 1);

  double first_x = 0, last_x = 0;
  uint16_t ncross = 0;
  double prev = vAt(lo) - rough;
  for (int32_t k = lo + 1; k <= hi; k++) {
    double cur = vAt(k) - rough;
    if (prev <= 0.0 && cur > 0.0) {
      double x = (double)(k - 1) + ((cur != prev) ? (-prev / (cur - prev)) : 0.0);
      if (ncross == 0) first_x = x;
      last_x = x;
      ncross++;
    }
    prev = cur;
  }

  double a, b;
  uint16_t cycles_used = 0;
  // A real mains window has ~CYCLES+1 crossings; noise on a dead channel
  // crosses far more often, so treat that as "no crossings" too.
  float max_cross = (float)n * dt_s * cfg.mainsHz * 3.0f + 4.0f;
  if (ncross >= 2 && ncross <= max_cross && (last_x - first_x) >= 4.0) {
    a = first_x; b = last_x; cycles_used = ncross - 1;
  } else {
    a = lo; b = hi; r.flags |= FLG_NOCROSS;
  }

  double dc[3];
  integrateMean(integrandDC, 2, a, b, dc);
  A_VM = dc[0];
  A_IM = dc[1];
  r.vdc = (float)A_VM;
  r.idc = (float)A_IM;

  double acc[3];
  integrateMean(integrandPower, 3, a, b, acc);

  r.span_s = (float)(b - a) * dt_s;
  r.hz = (cycles_used && r.span_s > 0) ? (float)cycles_used / r.span_s : 0.0f;

  float gain = 1.0f;
  if (r.hz > 0.0f) {
    gain = interpGain(A_W, 2.0 * M_PI * r.hz * dt_s);
    if (gain < 0.5f) gain = 1.0f;
  }

  r.vrms = acc[0] > 0 ? (float)sqrt(acc[0]) : 0.0f;
  r.irms = acc[1] > 0 ? (float)sqrt(acc[1]) : 0.0f;
  r.p    = (float)acc[2] / gain;
  r.s    = r.vrms * r.irms;
  double qq = (double)r.s * r.s - (double)r.p * r.p;
  r.q  = qq > 0 ? (float)sqrt(qq) : 0.0f;
  r.pf = r.s > 1e-6f ? r.p / r.s : 0.0f;

  float vpk = 0, ipk = 0;
  for (int32_t k = (int32_t)a; k <= (int32_t)b; k++) {
    float dv = (float)fabs(vAt(k) - A_VM), di = (float)fabs(iAt(k) - A_IM);
    if (dv > vpk) vpk = dv;
    if (di > ipk) ipk = di;
  }
  r.vpk = vpk; r.ipk = ipk;
  return true;
}

// ═══ metering window ══════════════════════════════════════════════════════════

void serviceMeasurement() {
  float want_s = ((float)cfg.cycles + 2.0f) / cfg.mainsHz;
  uint32_t n_want = (uint32_t)cfg.spc * (cfg.cycles + 2UL);
  if (n_want > PAIR_MAX) n_want = PAIR_MAX;

  // a range relay that just moved: let it settle before the window starts
  uint32_t since = millis() - g_rngAt;
  if (since < cfg.rngSetMs) delay(cfg.rngSetMs - since);
  g_rngEvt = false;
  armPeakTrip();
  uint8_t acq_flags = 0;
  float dt_s = 0;
  uint16_t depth = 0;
  uint16_t n = acquireWindow((uint16_t)n_want, (uint32_t)(want_s * 1e6f), dt_s, acq_flags, depth);
  reportPeakTrip();
  if (g_abort) { g_abort = false; return; }    // a command is waiting
  if (g_rngEvt) { g_rngEvt = false; return; }  // the range changed mid-window

  PowerResult r;
  if (!analyzePower(n, dt_s, r)) return;
  r.flags |= acq_flags;
  r.depth = depth;

  uint32_t now = millis();
  if (g_lastWinMs) g_wh += (double)r.p * (double)(now - g_lastWinMs) / 3.6e6;
  g_lastWinMs = now;

  if (cfg.tripA > 0 && r.irms > cfg.tripA && g_relay) {
    g_tripLatch = true;
    setRelay(false);
    Serial.print(F("$TRIP,RMS,")); Serial.print(r.irms, 4);
    Serial.print(','); Serial.println(cfg.tripA, 4);
  }
  if (g_tripLatch) r.flags |= FLG_TRIP;
  rangeAuto(r);

  g_last = r;
  g_haveLast = true;
  pixelUpdate(r.vrms);

  if (g_streaming && Serial) {
    Serial.print(F("$PWR,"));
    Serial.print(r.vrms, 4); Serial.print(',');
    Serial.print(r.irms, 5); Serial.print(',');
    Serial.print(r.p, 4);    Serial.print(',');
    Serial.print(r.s, 4);    Serial.print(',');
    Serial.print(r.q, 4);    Serial.print(',');
    Serial.print(r.pf, 4);   Serial.print(',');
    Serial.print(r.hz, 3);   Serial.print(',');
    Serial.print(r.vpk, 3);  Serial.print(',');
    Serial.print(r.ipk, 5);  Serial.print(',');
    Serial.print((float)g_wh, 6); Serial.print(',');
    Serial.print(millis() / 1000.0f, 2); Serial.print(',');
    Serial.print(r.n);       Serial.print(',');
    Serial.print(r.depth);   Serial.print(',');
    Serial.print(g_relay ? 1 : 0); Serial.print(',');
    Serial.print(r.flags);   Serial.print(',');
    Serial.print(r.vdc, 4);  Serial.print(',');
    Serial.print(r.idc, 5);  Serial.print(',');
    Serial.println(g_rng);
  }
}

// ═══ DC measurement (calibration) ═════════════════════════════════════════════

void measureDC(uint32_t ms) {
  if (ms < 10) ms = 10;
  if (ms > 10000) ms = 10000;
  adcMask(MASK_ALL);
  uint32_t since = millis() - g_rngAt;
  if (since < cfg.rngSetMs) delay(cfg.rngSetMs - since);
  g_rngEvt = false;
  armPeakTrip();
  uint8_t flags = 0;
  double sv = 0, si = 0, sv2 = 0, si2 = 0;
  uint32_t n = 0;
  uint32_t t0 = cyc(), dur = ms * 1000UL * CYC_PER_US;
  while ((uint32_t)(cyc() - t0) < dur) {
    adcScan();
    int32_t v = pairV(flags), i = pairI(flags);
    checkPeak(i);
    if (g_upArmed && (flags & FLG_ICLIP)) rangeClip(i);
    sv += v; si += i; sv2 += (double)v * v; si2 += (double)i * i;
    n++;
  }
  reportPeakTrip();
  float l = lsb();
  double mv = sv / n, mi = si / n;
  double vsd = sqrt(fmax(0.0, sv2 / n - mv * mv)), isd = sqrt(fmax(0.0, si2 / n - mi * mi));
  float va = (float)(mv * l), ia = (float)(mi * l);
  Serial.print(F("$DC,"));
  Serial.print(va, 7); Serial.print(',');
  Serial.print(ia, 7); Serial.print(',');
  Serial.print((va - cfg.vOff) * cfg.vGain, 5); Serial.print(',');
  Serial.print((ia - g_io) * g_ig, 6); Serial.print(',');
  Serial.print((float)(vsd * l), 7); Serial.print(',');
  Serial.print((float)(isd * l), 7); Serial.print(',');
  Serial.print(n); Serial.print(',');
  Serial.print(flags); Serial.print(',');
  Serial.print(g_rng); Serial.print(',');
  Serial.println(g_rngEvt ? 1 : 0);          // an overload switched range mid-reading
  g_rngEvt = false;
}

// whole mains cycles, so any AC present averages out
static void zeroMeasure(double& v, double& i) {
  uint32_t ms = (uint32_t)(12000.0f / cfg.mainsHz);
  adcMask(MASK_ALL);
  uint8_t flags = 0;
  double sv = 0, si = 0;
  uint32_t n = 0, t0 = cyc(), dur = ms * 1000UL * CYC_PER_US;
  while ((uint32_t)(cyc() - t0) < dur) {
    adcScan();
    sv += pairV(flags); si += pairI(flags); n++;
  }
  v = sv / n * lsb();
  i = si / n * lsb();
}

// Zeroes both current ranges when there are two (needs no load current).
void zeroChannels() {
  uint32_t since = millis() - g_rngAt;
  if (since < cfg.rngSetMs) delay(cfg.rngSetMs - since);
  uint8_t start = g_rng;
  double v, i;
  zeroMeasure(v, i);
  cfg.vOff = (float)v;
  if (g_rng == RNG_LO) cfg.iOffLo = (float)i; else cfg.iOff = (float)i;
  if (cfg.rngPin) {
    rangeSet(start == RNG_LO ? RNG_HI : RNG_LO, RW_CMD, true);
    zeroMeasure(v, i);
    if (g_rng == RNG_LO) cfg.iOffLo = (float)i; else cfg.iOff = (float)i;
    rangeSet(start, RW_CMD, true);
  }
  rngCal();
  g_dirty = true;
  printField(*findField("VOFF"));
  printField(*findField("IOFF"));
  if (cfg.rngPin) printField(*findField("IOFFLO"));
  Serial.println(F("$OK,zero"));
}

// ═══ fast capture ═════════════════════════════════════════════════════════════

struct CapReq {
  uint32_t rate; uint16_t n; uint8_t ch, trig;
  float level; float pre; uint32_t timeout_ms; int32_t relay_us;
  // safe start (0 = off)
  float ipk_a;            // open on any |i| above this
  float ov_a;             // open when per-cycle Irms stays above this ...
  uint16_t ov_cyc;        // ... for this many consecutive mains cycles
  uint32_t auto_ms;       // open this long after the trigger
  uint8_t open_after;     // open when the capture completes
  uint8_t avg;            // scans averaged into each stored sample
  int8_t  rng;            // -1 follow RNGMODE, 0 HIGH, 1 LOW
  uint8_t mon;            // 1 = $PWR readings from the capture's samples while it runs
};
static CapReq g_cap_req;
static bool g_cap_pending = false;

enum SafeWhy : uint8_t { SAFE_NONE, SAFE_INST, SAFE_OVER, SAFE_TIMER, SAFE_END, SAFE_TRIPPK, SAFE_REMOTE, SAFE_TRIPA };
static const char* const SAFE_NAMES[] = { "NONE", "INST", "OVERCURRENT", "TIMER", "END", "TRIPPK", "REMOTE", "TRIPA" };

static void serviceAutoOff() {
  if (g_autoOffArmed && (int32_t)(millis() - g_autoOffAt) >= 0) {
    g_autoOffArmed = false;
    if (g_relay) {
      setRelay(false);
      Serial.println(F("$SAFE,TIMER,0"));
    }
  }
}

static void hex4(char* p, int16_t v) {
  static const char H[] = "0123456789ABCDEF";
  uint16_t u = (uint16_t)v;
  p[0] = H[(u >> 12) & 15]; p[1] = H[(u >> 8) & 15]; p[2] = H[(u >> 4) & 15]; p[3] = H[u & 15];
}

static void drawCaptureNotice(const char* what);

// readings while armed: a finished window's $PWR line goes out one field per
// sample, so formatting never holds the sampling up for long
static float   g_monF[18];
static int8_t  g_monPos = -1;
static bool    g_monFresh = false;     // the last capture made a reading (now in g_last)

// A finished window's raw sums. With no trigger to wait for, the maths and the
// $PWR line wait until after the last sample, so sampling never slips (LATE).
struct MonSnap {
  int64_t sv, si, svv, sii, svi;
  uint32_t n, zx, zxSpan, span;          // span, zxSpan in CPU cycles
  uint32_t endMs;
  int32_t vpk, ipk;
  uint8_t flags;
};
static const uint8_t MON_DEFER_MAX = 8;  // more windows than this: worked out as they end
static const uint8_t MON_DP[18] = { 4, 5, 4, 4, 4, 4, 3, 3, 5, 6, 2, 0, 0, 0, 0, 4, 5, 0 };

static void monPrintStep() {
  if (g_monPos < 0) return;
  if (g_monPos == 0) Serial.print(F("$PWR,"));
  Serial.print(g_monF[g_monPos], MON_DP[g_monPos]);
  if (++g_monPos >= 18) { Serial.println(); g_monPos = -1; }
  else Serial.print(',');
}

void runCapture() {
  g_cap_pending = false;
  CapReq q = g_cap_req;
  const uint8_t per_scan = (q.ch == 2) ? 2 : 1;
  uint16_t max_n = CAP_SAMPLES / per_scan;
  if (q.n < 16) q.n = 16;
  if (q.n > max_n) q.n = max_n;
  if (q.pre < 0) q.pre = 0;
  if (q.pre > 95) q.pre = 95;
  bool needV = (q.trig == 2 || q.trig == 4 || (q.trig == 3 && q.relay_us >= 0));
  if (needV && q.ch == 1) {
    Serial.println(F("$ERR,that trigger needs the voltage channel (ch 0 or 2)"));
    return;
  }
  if ((q.trig == 1 || q.trig == 5 || q.trig == 6) && q.ch == 0) {
    Serial.println(F("$ERR,current trigger needs the current channel (ch 1 or 2)"));
    return;
  }
  if ((q.ipk_a > 0 || q.ov_a > 0) && q.ch == 0) {
    Serial.println(F("$ERR,current limits need the current channel (ch 1 or 2)"));
    return;
  }
  if (q.auto_ms > 60000UL) q.auto_ms = 60000UL;
  if (q.ov_cyc < 1) q.ov_cyc = 1;
  if (q.avg < 1) q.avg = 1;
  if (q.avg > 64) q.avg = 64;
  g_autoOffArmed = false;        // a new capture owns the relay timing

  // range: V-only can't watch for an overload, so it always goes HIGH; an
  // auto-mode inrush capture goes HIGH like any other close
  int8_t want = q.rng;
  const bool closes = (q.trig == 3 || q.trig == 5);
  if (want < 0) want = (closes && cfg.rngMode == 0) ? RNG_HI : g_rng;
  if (q.ch == 0) want = RNG_HI;
  if (cfg.rngPin && want != g_rng) rangeSet((uint8_t)want, RW_CAP, true);
  { uint32_t since = millis() - g_rngAt; if (since < cfg.rngSetMs) delay(cfg.rngSetMs - since); }
  if (closes) g_overAt = millis();
  g_rngEvt = false;
  rangeReport();
  const uint8_t rng0 = g_rng;

  // live persistence sends these back to back: keep the meter on screen, and the I2C time
  if (!(q.mon && q.trig == 0)) drawCaptureNotice(q.trig ? "ARMED" : "CAPTURE");
  uint32_t mask = (q.ch == 0) ? MASK_V : (q.ch == 1) ? MASK_I : MASK_ALL;
  adcMask(mask);
  armPeakTrip();

  float l = lsb();
  float vOffC = cfg.vOff / l, iOffC = g_io / l;
  int32_t lvlI = (g_ig != 0) ? (int32_t)fabsf(q.level / g_ig / l) : 0x7FFFFFFF;
  int32_t lvlV = (cfg.vGain != 0) ? (int32_t)fabsf(q.level / cfg.vGain / l) : 0x7FFFFFFF;
  // hysteresis for the zero-crossing trigger: 5 V below zero must be seen first
  int32_t hystV = (cfg.vGain != 0) ? (int32_t)fabsf(5.0f / cfg.vGain / l) : 10;
  int32_t v0 = (int32_t)lroundf(vOffC), i0 = (int32_t)lroundf(iOffC);

  uint32_t per = q.rate ? (SystemCoreClock / q.rate) : 0;
  uint32_t sub = per / q.avg;            // spacing of the scans inside one sample
  uint32_t pre_n = (uint32_t)((float)q.n * q.pre / 100.0f);
  uint32_t post_n = q.n - pre_n;
  if (q.trig == 0) pre_n = 0, post_n = q.n;

  uint8_t flags = 0;
  uint16_t idx = 0;
  uint32_t count = 0;
  int32_t trigAt = -1, relayAt = -1;
  uint32_t remaining = 0;
  bool vLow = false;
  int32_t relayDeadline = 0; bool relayWait = false;   // trig 3 with sync
  uint32_t t_first = 0, t_last = 0;
  uint32_t ms0 = millis();
  bool timedOut = false, aborted = false;

  // safe start, in raw counts / CPU cycles
  float ipkC = (q.ipk_a > 0 && g_ig != 0) ? fabsf(q.ipk_a / g_ig / l) : 0;
  float ovC  = (q.ov_a  > 0 && g_ig != 0) ? fabsf(q.ov_a  / g_ig / l) : 0;
  float ovSq = ovC * ovC;
  uint32_t cycLen = (uint32_t)((float)SystemCoreClock / cfg.mainsHz);
  uint64_t ovSum = 0; uint32_t ovN = 0, ovStart = cyc(); uint16_t ovRun = 0;
  float ovWorst = 0;
  // auto-open: 64-bit, advanced a sample at a time — at 168 MHz (F405) the
  // 32-bit cycle counter wraps every 25 s, inside the 60 s this allows
  bool autoArmed = false;
  uint32_t autoPrev = 0;
  uint64_t autoEl = 0, autoCyc = (uint64_t)q.auto_ms * 1000ULL * CYC_PER_US;
  int32_t openAt = -1; uint8_t openWhy = SAFE_NONE; float openVal = 0;
  bool wasClosed = g_relay;
  int32_t rngAt = -1;                   // sample where an overload went HIGH

  // trig 6: per-cycle Irms, armed by a cycle at or above the level
  float lvlSq = (float)lvlI * (float)lvlI;
  uint64_t dqSum = 0; uint32_t dqN = 0, dqStart = cyc(); bool dqArmed = false;

  // readings while armed: one window = CYCLES mains cycles, sums relative to v0 / i0
  const bool hasV = q.ch != 1, hasI = q.ch != 0;
  const uint32_t monLen = cycLen * (cfg.cycles ? cfg.cycles : 1);
  uint32_t mwN = 0, mwStart = cyc(), mwZx = 0, mwZxFirst = 0, mwZxLast = 0;
  int64_t mwSv = 0, mwSi = 0, mwSvv = 0, mwSii = 0, mwSvi = 0;
  int32_t mwVpk = 0, mwIpk = 0;
  uint8_t mwFlags = 0;
  bool mwVLow = false;
  auto monReset = [&](uint32_t ts) {
    mwN = 0; mwStart = ts; mwZx = 0; mwSv = mwSi = mwSvv = mwSii = mwSvi = 0;
    mwVpk = mwIpk = 0; mwFlags = 0;
  };
  // the end of a window, inside the sample loop: copy the sums and check TRIPA
  // (which normal metering checks every window) in single-precision float
  auto monTake = [&](uint32_t ts) {
    MonSnap m;
    m.sv = mwSv; m.si = mwSi; m.svv = mwSvv; m.sii = mwSii; m.svi = mwSvi;
    m.n = mwN; m.zx = mwZx; m.zxSpan = mwZxLast - mwZxFirst; m.span = ts - mwStart;
    m.endMs = millis(); m.vpk = mwVpk; m.ipk = mwIpk; m.flags = mwFlags;
    if (hasI && cfg.tripA > 0 && g_relay && mwN) {
      float n = (float)mwN, mi = (float)mwSi / n;
      float irms = sqrtf(fmaxf((float)mwSii / n - mi * mi, 0.0f)) * fabsf(l * g_ig);
      if (irms > cfg.tripA) {
        relayDrive(false); g_relay = false; g_tripLatch = true;
        if (openAt < 0) { openAt = (int32_t)count - 1; openWhy = SAFE_TRIPA; openVal = irms; }
      }
    }
    return m;
  };
  // the maths and the $PWR fields, from a window's sums
  auto monCompute = [&](const MonSnap& m) {
    float kv = l * cfg.vGain, ki = l * g_ig;
    double n = (double)m.n;
    double mv = m.sv / n, mi = m.si / n;
    float vrms = hasV ? (float)sqrt(fmax(m.svv / n - mv * mv, 0.0)) * fabsf(kv) : 0.0f;
    float irms = hasI ? (float)sqrt(fmax(m.sii / n - mi * mi, 0.0)) * fabsf(ki) : 0.0f;
    float p = (hasV && hasI) ? (float)(m.svi / n - mv * mi) * kv * ki : 0.0f;
    float s = vrms * irms;
    float span = (float)m.span / (float)SystemCoreClock;
    // Wh over the time since the last reading, as normal metering does, so the gaps
    // between back-to-back captures still count; a stale mark falls back to the window
    float since = g_lastWinMs ? (float)(m.endMs - g_lastWinMs) / 1000.0f : -1.0f;
    g_wh += (double)p * (since >= 0 && since < span + 2.0f ? since : span) / 3600.0;
    g_lastWinMs = m.endMs;
    uint8_t fl = m.flags | (flags & FLG_LATE) | (g_tripLatch ? FLG_TRIP : 0);
    if (hasV && m.zx < 2) fl |= FLG_NOCROSS;
    float* f = g_monF;
    f[0] = vrms; f[1] = irms; f[2] = p; f[3] = s;
    f[4] = sqrtf(fmaxf(s * s - p * p, 0.0f)); f[5] = s > 0 ? p / s : 0.0f;
    f[6] = m.zx >= 2 ? (float)(m.zx - 1) * (float)SystemCoreClock / (float)m.zxSpan : 0.0f;
    f[7] = m.vpk * fabsf(kv); f[8] = m.ipk * fabsf(ki); f[9] = (float)g_wh;
    f[10] = m.endMs / 1000.0f; f[11] = (float)m.n; f[12] = q.avg; f[13] = g_relay ? 1 : 0;
    f[14] = fl;
    f[15] = hasV ? (float)(mv + v0 - vOffC) * kv : 0.0f;
    f[16] = hasI ? (float)(mi + i0 - iOffC) * ki : 0.0f;
    f[17] = g_rng;
    // the screen and the status light show this reading too
    PowerResult& r = g_last;
    r.vrms = vrms; r.irms = irms; r.p = p; r.s = s; r.q = f[4]; r.pf = f[5]; r.hz = f[6];
    r.vpk = f[7]; r.ipk = f[8]; r.vdc = f[15]; r.idc = f[16];
    r.dt_s = per ? (float)per / (float)SystemCoreClock : 0.0f; r.span_s = span;
    r.n = m.n > 65535 ? 65535 : (uint16_t)m.n; r.depth = q.avg; r.flags = fl;
    g_haveLast = true; g_monFresh = true;
    if (g_streaming && Serial) g_monPos = 0;
  };
  // no trigger to wait for (live persistence): everything slow waits for the end
  const bool monDefer = q.trig == 0;
  MonSnap monQ[MON_DEFER_MAX];
  uint8_t monQn = 0;
  g_monPos = -1;

  uint32_t next = cyc();
  for (;;) {
    if (per) {
      while ((int32_t)(cyc() - next) < 0) {}
      if ((int32_t)(cyc() - next) > (int32_t)per) { flags |= FLG_LATE; next = cyc(); }
      next += per;
    }
    uint32_t ts = cyc();
    // oversampling: average q.avg scans; the instant trip keeps the worst single one
    int32_t sv = 0, si = 0, iDev = 0;
    uint8_t sf = 0;                      // this sample's clip flags
    for (uint8_t a = 0; a < q.avg; a++) {
      if (a && sub) { uint32_t due = ts + a * sub; while ((int32_t)(cyc() - due) < 0) {} }
      adcScan();
      uint8_t f1 = 0;
      if (q.ch != 1) sv += pairV(f1);
      if (q.ch != 0) {
        int32_t r = pairI(f1);
        checkPeak(r);
        if (g_upArmed && (f1 & FLG_ICLIP)) rangeClip(r);
        si += r;
        if (labs(r - i0) > labs(iDev)) iDev = r - i0;
      }
      sf |= f1;
    }
    flags |= sf;
    int32_t v = sv, i = si;
    if (q.avg > 1) {
      int32_t h = q.avg / 2;
      v = (sv >= 0 ? sv + h : sv - h) / q.avg;
      i = (si >= 0 ? si + h : si - h) / q.avg;
    }

    if (q.ch == 2) { g_cap[2 * idx] = (int16_t)v; g_cap[2 * idx + 1] = (int16_t)i; }
    else           { g_cap[idx] = (int16_t)(q.ch == 0 ? v : i); }
    if (++idx >= q.n) idx = 0;
    if (count == 0) t_first = ts;
    t_last = ts;
    count++;

    if (g_rngEvt) {                      // overload: HIGH from here, rescale the limits
      g_rngEvt = false;
      if (rngAt < 0) rngAt = (int32_t)(count - 1);
      iOffC = g_io / l;
      i0 = (int32_t)lroundf(iOffC);
      lvlI = (g_ig != 0) ? (int32_t)fabsf(q.level / g_ig / l) : 0x7FFFFFFF;
      ipkC = (q.ipk_a > 0 && g_ig != 0) ? fabsf(q.ipk_a / g_ig / l) : 0;
      ovC  = (q.ov_a  > 0 && g_ig != 0) ? fabsf(q.ov_a  / g_ig / l) : 0;
      ovSq = ovC * ovC;
      ovSum = 0; ovN = 0; ovStart = ts;
      lvlSq = (float)lvlI * (float)lvlI;
      dqSum = 0; dqN = 0; dqStart = ts;
      monReset(ts);
    }

    if (trigAt < 0) {
      if (count > pre_n) {
        bool fire = false;
        if (q.trig == 5 && relayAt < 0) {  // close once the pretrigger is full, then wait for the level
          g_tripLatch = false;
          relayDrive(true);
          g_relay = true;
          relayAt = (int32_t)(count - 1);
        }
        switch (q.trig) {
          case 0: fire = true; break;
          case 1: case 5: fire = labs(i - i0) >= lvlI; break;
          case 6: {
            int32_t d = i - i0;
            uint32_t el = ts - dqStart;
            if (el >= 2 * cycLen) { dqSum = 0; dqN = 0; dqStart = ts; }   // not sampling yet: start afresh
            else if (el >= cycLen) {
              float ms = dqN ? (float)dqSum / (float)dqN : 0.0f;
              if (ms >= lvlSq) dqArmed = true;
              else if (dqArmed) fire = true;            // dqN samples ago it was still on
              if (!fire) { dqSum = 0; dqN = 0; dqStart += cycLen; }
            }
            if (!fire) { dqSum += (uint64_t)((int64_t)d * d); dqN++; }
            break;
          }
          case 4: fire = labs(v - v0) >= lvlV; break;
          case 2:
            if (v < v0 - hystV) vLow = true;
            else if (vLow && v >= v0) fire = true;
            break;
          case 3:
            if (q.relay_us < 0) fire = true;
            else if (!relayWait) {
              if (v < v0 - hystV) vLow = true;
              else if (vLow && v >= v0) {
                relayWait = true;
                relayDeadline = (int32_t)(ts + (uint32_t)q.relay_us * CYC_PER_US);
              }
            }
            if (relayWait && (int32_t)(ts - (uint32_t)relayDeadline) >= 0) fire = true;
            break;
        }
        if (fire) {
          trigAt = (int32_t)(count - 1);
          remaining = post_n;
          if (q.trig == 6) {                // mark the start of the first low cycle
            trigAt -= (int32_t)dqN;
            if (trigAt < 0) trigAt = 0;
            uint32_t since = count - 1 - (uint32_t)trigAt;
            remaining = post_n > since ? post_n - since : 1;
          }
          if (q.trig == 3) {
            g_tripLatch = false;
            relayDrive(true);
            g_relay = true;
            relayAt = trigAt;
          }
        }
      }
      if ((count & 255) == 0) {
        if (Serial.available()) { timedOut = aborted = true; break; }
        if (q.timeout_ms && (millis() - ms0) > q.timeout_ms) { timedOut = true; break; }
      }
    }
    // ── safe start ──
    // cycles count from the trigger, or from the close when the trigger follows it
    if ((q.trig == 5 ? relayAt : trigAt) == (int32_t)(count - 1)) {
      ovStart = ts; ovSum = 0; ovN = 0; ovRun = 0;
      if (q.auto_ms) { autoArmed = true; autoPrev = ts; autoEl = 0; }
    }
    if (autoArmed) { autoEl += (uint32_t)(ts - autoPrev); autoPrev = ts; }
    if (g_relay) {
      wasClosed = true;
      uint8_t why = SAFE_NONE;
      if (q.ch != 0 && !tripBlanked()) {
        int32_t d = i - i0;
        if (ipkC > 0 && fabsf((float)iDev) > ipkC) { why = SAFE_INST; openVal = (float)iDev * l * g_ig; }
        if (ovSq > 0) {
          ovSum += (uint64_t)((int64_t)d * d); ovN++;
          if ((uint32_t)(ts - ovStart) >= cycLen) {
            float r = sqrtf((float)ovSum / (float)ovN);
            if (r > ovC) {
              if (r > ovWorst) ovWorst = r;
              if (++ovRun >= q.ov_cyc && !why) { why = SAFE_OVER; openVal = ovWorst * l * fabsf(g_ig); }
            } else { ovRun = 0; ovWorst = 0; }
            ovSum = 0; ovN = 0; ovStart += cycLen;
          }
        }
      }
      if (!why && autoArmed && autoEl >= autoCyc) why = SAFE_TIMER;
      if (!why && ir_ready && irOpenRequested()) why = SAFE_REMOTE;
      if (why) {
        relayDrive(false); g_relay = false;
        if (why != SAFE_TIMER && why != SAFE_REMOTE) g_tripLatch = true;
        openAt = (int32_t)(count - 1); openWhy = why;
        autoArmed = false;
      }
    } else if (wasClosed && openAt < 0) {
      openAt = (int32_t)(count - 1); openWhy = SAFE_TRIPPK;   // the global TRIPPK got there first
    }

    if (q.mon) {
      int32_t dv = v - v0, di = i - i0;
      mwN++;
      if (hasV) {
        mwSv += dv; mwSvv += (int64_t)dv * dv;
        if (labs(dv) > mwVpk) mwVpk = labs(dv);
        if (dv < -hystV) mwVLow = true;
        else if (mwVLow && dv >= 0) { mwVLow = false; if (!mwZx++) mwZxFirst = ts; mwZxLast = ts; }
      }
      if (hasI) {
        mwSi += di; mwSii += (int64_t)di * di;
        if (labs(di) > mwIpk) mwIpk = labs(di);
        if (hasV) mwSvi += (int64_t)dv * di;
      }
      mwFlags |= sf;
      if ((uint32_t)(ts - mwStart) >= monLen) {
        MonSnap m = monTake(ts);
        if (monDefer && monQn < MON_DEFER_MAX) monQ[monQn++] = m;
        else if (g_monPos < 0) monCompute(m);  // the last line is still going out: skip this one
        monReset(ts);
      }
      monPrintStep();
    }

    if (trigAt >= 0) {
      if (remaining <= 1) break;
      remaining--;
    }
  }
  while (g_monPos >= 0) monPrintStep();  // finish a half-sent $PWR line
  for (uint8_t k = 0; k < monQn; k++) {  // the readings held back until now
    monCompute(monQ[k]);
    while (g_monPos >= 0) monPrintStep();
  }
  reportPeakTrip();
  if (relayAt >= 0) Serial.println(F("$RLY,1"));
  // trig 5 has closed the relay before its trigger, so open_after covers a timeout too
  if ((!timedOut || q.trig == 5) && q.open_after && g_relay) {
    relayDrive(false); g_relay = false;
    openAt = (int32_t)(count - 1); openWhy = SAFE_END; autoArmed = false;
  }
  if (autoArmed && g_relay) {            // capture ended first: keep the timer running
    uint64_t el = autoEl + (uint32_t)(cyc() - autoPrev);
    uint32_t left_us = el >= autoCyc ? 0 : (uint32_t)((autoCyc - el) / CYC_PER_US);
    g_autoOffAt = millis() + (left_us + 999) / 1000;
    g_autoOffArmed = true;
  }
  if (openAt >= 0) {
    Serial.println(F("$RLY,0"));
    Serial.print(F("$SAFE,")); Serial.print(SAFE_NAMES[openWhy]);
    Serial.print(','); Serial.println(openVal, 4);
  }

  adcMask(MASK_ALL);
  if (timedOut) {
    Serial.println(aborted ? F("$CAPTMO,ABORT") : F("$CAPTMO,TIMEOUT"));
    return;
  }

  uint32_t total = count < q.n ? count : q.n;
  uint32_t start = (count >= q.n) ? idx : 0;
  uint32_t first_abs = count - total;
  float dt_us = (per && !(flags & FLG_LATE))
              ? (float)per / (float)CYC_PER_US
              : (count > 1 ? (float)(t_last - t_first) / (float)CYC_PER_US / (float)(count - 1) : 0.0f);

  Serial.print(F("$CAP,"));
  Serial.print(total); Serial.print(',');
  Serial.print(q.ch); Serial.print(',');
  Serial.print(dt_us, 4); Serial.print(',');
  Serial.print(l * cfg.vGain, 9); Serial.print(',');
  Serial.print(vOffC, 3); Serial.print(',');
  // the data starts in rng0 unless the switch is older than the oldest sample kept
  int32_t rngRel = rngAt >= 0 ? rngAt - (int32_t)first_abs : -1;
  uint8_t rngStart = rng0;
  if (rngAt >= 0 && rngRel <= 0) { rngStart = RNG_HI; rngRel = -1; }
  float igS = rngStart == RNG_LO ? cfg.iGainLo : cfg.iGain;
  float ioS = rngStart == RNG_LO ? cfg.iOffLo : cfg.iOff;
  Serial.print(l * igS, 9); Serial.print(',');
  Serial.print(ioS / l, 3); Serial.print(',');
  Serial.print(trigAt >= 0 ? (long)(trigAt - (int32_t)first_abs) : -1L); Serial.print(',');
  Serial.print(flags); Serial.print(',');
  Serial.print(relayAt >= (int32_t)first_abs ? (long)(relayAt - (int32_t)first_abs) : -1L); Serial.print(',');
  Serial.print(openAt >= (int32_t)first_abs ? (long)(openAt - (int32_t)first_abs) : -1L); Serial.print(',');
  Serial.print(openWhy); Serial.print(',');
  Serial.print(q.avg); Serial.print(',');
  Serial.print(rngStart); Serial.print(',');
  Serial.print(rngRel); Serial.print(',');
  Serial.print(l * cfg.iGain, 9); Serial.print(',');
  Serial.print(cfg.iOff / l, 3); Serial.print(',');
  Serial.println((uint32_t)cfg.rngSetMs * 1000UL);

  // 32 values per line, V/I interleaved for ch 2
  char line[4 + 32 * 4 + 2];
  uint32_t values = total * per_scan;
  uint32_t vstart = start * per_scan, vmod = (uint32_t)q.n * per_scan;
  for (uint32_t k = 0; k < values;) {
    memcpy(line, "$CD,", 4);
    uint8_t m = 0;
    while (m < 32 && k < values) {
      hex4(line + 4 + 4 * m, g_cap[(vstart + k) % vmod]);
      m++; k++;
    }
    line[4 + 4 * m] = '\r'; line[5 + 4 * m] = '\n';
    Serial.write((const uint8_t*)line, 6 + 4 * m);
  }
  Serial.print(F("$CEND,")); Serial.println(total);
}

// ═══ NeoPixel ═════════════════════════════════════════════════════════════════

#ifdef PIN_RGB
static Adafruit_NeoPixel g_pix(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
#endif
static int8_t g_pixState = -1;          // -1 unknown, 0 green, 1 red

static void pixelInit() {
#ifdef PIN_RGB
#ifdef PIN_RGB_EN
  pinMode(PIN_RGB_EN, OUTPUT);
  digitalWrite(PIN_RGB_EN, HIGH);
#endif
  g_pix.begin();
  g_pix.clear();
  g_pix.show();
#endif
  g_pixState = -1;
}

// Red at or above LIVEVAC, green below; 2 V of hysteresis so it doesn't
// flicker at the threshold. Only writes on a change — show() masks interrupts.
static void pixelUpdate(float vrms) {
#ifdef PIN_RGB
  int8_t want = (g_pixState == 1) ? (vrms > cfg.liveVac - 2.0f) : (vrms >= cfg.liveVac);
  if (want == g_pixState) return;
  g_pixState = want;
  uint8_t b = cfg.pixel;
  g_pix.setPixelColor(0, want ? g_pix.Color(b, 0, 0) : g_pix.Color(0, b, 0));
  g_pix.show();
#else
  (void)vrms;
#endif
}

// ═══ remote ═══════════════════════════════════════════════════════════════════

static void irInit() {
  if (g_irAttached >= 0) { detachInterrupt(digitalPinToInterrupt(g_irAttached)); g_irAttached = -1; }
  ir_ready = false; ir_n = -1; g_primed = false;
  if (!cfg.irPin) return;
#if !BOARD_F405                  // every F405 pin can take an edge interrupt
  if (getPinCfgs(cfg.irPin, PIN_CFG_REQ_INTERRUPT)[0] == 0) {
    Serial.println(F("$ERR,IRPIN has no edge interrupt (D10 doesn't) - try D9, D8 or D7"));
    return;
  }
#endif
  pinMode(cfg.irPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(cfg.irPin), irIsr, FALLING);
  g_irAttached = cfg.irPin;
}

static void reportPendingOpen() {
  uint8_t why = g_pendingOpen;
  if (!why) return;
  g_pendingOpen = 0;
  Serial.println(F("$RLY,0"));
  Serial.print(F("$SAFE,")); Serial.print(SAFE_NAMES[why]); Serial.println(F(",0"));
  if (why == SAFE_REMOTE) Serial.println(F("$REMOTE,OPEN"));
  g_lastDispMs = 0;
}

static void serviceRemote() {
  reportPendingOpen();
  if (ir_ready) {
    uint32_t f = ir_frame;
    ir_ready = false;
    uint16_t a; uint8_t c;
    if (irDecode(f, a, c)) {
      Serial.print(F("$IR,")); Serial.print(a, HEX); Serial.print(','); Serial.println(c, HEX);
      if (a == cfg.irAddr && c == cfg.irOk) {
        if (g_relay) {
          g_autoOffArmed = false;
          setRelay(false);
          Serial.println(F("$REMOTE,OPEN"));
        } else if (g_primed) {
          g_primed = false;
          Serial.println(F("$REMOTE,CANCEL"));
        } else {
          g_primed = true; g_primedAt = millis();
          Serial.println(F("$REMOTE,PRIMED"));
        }
        g_lastDispMs = 0;
      } else if (a == cfg.irAddr && c == cfg.irConfirm && g_primed && !g_relay) {
        g_primed = false;
        g_tripLatch = false;               // same as a !RELAY,1
        g_autoOffArmed = false;
        rangeBeforeClose();
        setRelay(true);
        Serial.println(F("$REMOTE,CLOSE"));
        g_lastDispMs = 0;
      }
    }
  }
  if (g_primed && millis() - g_primedAt > cfg.irArmMs) {
    g_primed = false;
    Serial.println(F("$REMOTE,TIMEOUT"));
    g_lastDispMs = 0;
  }
}

// ═══ OLED ═════════════════════════════════════════════════════════════════════

// the drivers share Adafruit_GFX for drawing, but not clear / push
static void oledClear() {
  switch (g_oledType) {
    case 1: g_ssd64.clearDisplay(); break;
    case 2: g_ssd32.clearDisplay(); break;
    case 3: g_sh64.clearDisplay(); break;
  }
}

static void oledShow() {
  switch (g_oledType) {
    case 1: g_ssd64.display(); break;
    case 2: g_ssd32.display(); break;
    case 3: g_sh64.display(); break;
  }
}

// blank the screen in use (before switching it off or to another type)
static void oledBlank() {
  if (!g_oledOk) return;
  oledClear();
  oledShow();
}

static void oledInit() {
  g_oledOk = false;
  g_oledType = 0;
  bool ok = false;
  uint8_t rot = cfg.oledRot ? 2 : 0;
  switch (cfg.oled) {
    case 1: ok = g_ssd64.begin(SSD1306_SWITCHCAPVCC, cfg.oledAddr); g_gfx = &g_ssd64; g_oledH = 64; break;
    case 2: ok = g_ssd32.begin(SSD1306_SWITCHCAPVCC, cfg.oledAddr); g_gfx = &g_ssd32; g_oledH = 32; break;
    case 3: ok = g_sh64.begin(cfg.oledAddr, true); g_gfx = &g_sh64; g_oledH = 64;
            rot += 1; break;               // the panel is 64x128: turn it landscape
  }
  if (!ok) return;
  g_oledType = cfg.oled;
  g_oledOk = true;
  g_gfx->setRotation(rot);
  oledClear();
  g_gfx->setTextColor(1);                  // SSD1306_WHITE == SH110X_WHITE
  g_gfx->setTextSize(1);
  g_gfx->setCursor(0, 0);
  g_gfx->println(F("120V MONITOR"));
  g_gfx->print(F("fw ")); g_gfx->println(F(FW_VERSION));
  oledShow();
}

static void drawCaptureNotice(const char* what) {
  if (!g_oledOk) return;
  g_gfx->fillRect(0, g_oledH - 8, 128, 8, 0);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(0, g_oledH - 8);
  g_gfx->print(what);
  oledShow();
}

static void printFit(float v, uint8_t width, uint8_t maxdp) {
  // as many decimals as fit in `width` characters
  char buf[16];
  float a = fabsf(v);
  uint8_t dp = maxdp;
  while (dp > 0 && (a >= powf(10.0f, (float)(width - dp - 2 - (v < 0))))) dp--;
  dtostrf(v, width, dp, buf);
  g_gfx->print(buf);
}

// the bottom line: prompt / trip / clip, else `idle`
static bool drawAlert(const PowerResult& r) {
  Adafruit_GFX& d = *g_gfx;
  if (g_primed)                  d.print(F("PRESS * TO CLOSE"));
  else if (g_tripLatch)          d.print(F("TRIPPED"));
  else if (r.flags & (FLG_VCLIP | FLG_ICLIP)) d.print(F("CLIP"));
  else return false;
  return true;
}

static void drawMeter() {
  if (!g_oledOk) return;
  uint32_t now = millis();
  if (now - g_lastDispMs < 250) return;
  g_lastDispMs = now;
  const PowerResult& r = g_last;
  Adafruit_GFX& d = *g_gfx;

  oledClear();
  d.setTextSize(1);
  d.setCursor(0, 0);
  d.print(g_relay ? F("LOAD ON ") : F("LOAD OFF"));
  if (cfg.rngPin) { d.setCursor(54, 0); d.print(g_rng == RNG_LO ? F("LO") : F("HI")); }
  d.setCursor(74, 0);
  if (r.hz > 0) { printFit(r.hz, 5, 2); d.print(F("Hz")); }
  else d.print(F(" --.-Hz"));

  if (g_oledH < 64) {
    // 128x32: big V on the left, A and W stacked beside it, PF and Wh below
    d.setTextSize(2);
    d.setCursor(0, 8);
    printFit(r.vrms, 6, 1); d.print('V');
    d.setTextSize(1);
    d.setCursor(86, 8);
    printFit(r.irms, 6, 3); d.print('A');
    d.setCursor(86, 16);
    printFit(r.p, 6, 1); d.print('W');
    d.setCursor(0, 24);
    if (!drawAlert(r)) {
      d.print(F("PF")); printFit(r.pf, 5, 2);
      d.setCursor(48, 24);
      printFit((float)g_wh, 7, 3); d.print(F("Wh"));
    }
  } else {
    d.setTextSize(2);
    d.setCursor(0, 11);
    printFit(r.vrms, 7, 2); d.print('V');
    d.setCursor(0, 29);
    printFit(r.irms, 7, 3); d.print('A');

    d.setTextSize(1);
    d.setCursor(0, 47);
    printFit(r.p, 7, 1); d.print(F("W PF"));
    printFit(r.pf, 6, 3);

    d.setCursor(0, 56);
    if (!drawAlert(r)) { printFit((float)g_wh, 8, 3); d.print(F("Wh")); }
  }
  if (g_streaming && !g_primed) { d.setCursor(110, g_oledH - 8); d.print(F("PC")); }
  oledShow();
}

// ═══ DAC loopback test (Feather) ══════════════════════════════════════════════

#if BOARD_F405
#define DAC_PTS 128
static uint32_t g_dacTab[DAC_PTS];       // DHR12RD words: DAC2 in 27:16, DAC1 in 11:0
static float g_dacHz = 0, g_dacDeg = 0, g_dacAmp = 0;

// Timers on an APB bus run at twice the bus clock whenever its prescaler is
// not 1 (on the Feather APB1 is /4, so TIM4 ticks at 84 MHz).
static uint32_t apb1TimerClock() {
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk : 2UL * pclk;
}

void dacTestStop() {
  TIM4->CR1 = 0;
  DMA1_Stream5->CR = 0;
  DAC->CR = 0;
  g_dacHz = 0;
}

// Two sines for checking the whole chain without mains: A0 is the "voltage",
// A1 the "current" lagging it by deg, each amp volts peak about mid-rail, so
// the readings come back with PF = cos(deg). Jumper A0 to A2 (V+) and A1 to A4
// (I+), hold A3 and A5 at mid-rail with a divider, and disconnect the
// isolation amps. TIM4 updates both DACs on the same edge from one DMA stream
// feeding DHR12RD, so the phase between them is exact. No interrupts.
void dacTestStart(float hz, float deg, float amp) {
  dacTestStop();
  RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM4EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  (void)RCC->AHB1ENR;
  pinMode(A0, INPUT_ANALOG);
  pinMode(A1, INPUT_ANALOG);

  float a = amp / cfg.adcVref * 4096.0f, ph = deg * (float)M_PI / 180.0f;
  for (uint16_t k = 0; k < DAC_PTS; k++) {
    float w = 2.0f * (float)M_PI * (float)k / (float)DAC_PTS;
    long v = constrain(lroundf(2048.0f + a * sinf(w)),      0L, 4095L);
    long i = constrain(lroundf(2048.0f + a * sinf(w - ph)), 0L, 4095L);
    g_dacTab[k] = ((uint32_t)i << 16) | (uint32_t)v;
  }

  DMA1->HIFCR = 0xF40U;                              // every stream 5 flag
  DMA1_Stream5->PAR  = (uint32_t)&DAC->DHR12RD;
  DMA1_Stream5->M0AR = (uint32_t)g_dacTab;
  DMA1_Stream5->NDTR = DAC_PTS;
  DMA1_Stream5->FCR  = 0;
  DMA1_Stream5->CR   = (7U << DMA_SxCR_CHSEL_Pos)    // channel 7 = DAC1
                     | (2U << DMA_SxCR_MSIZE_Pos) | (2U << DMA_SxCR_PSIZE_Pos)
                     | DMA_SxCR_MINC | DMA_SxCR_CIRC
                     | (1U << DMA_SxCR_DIR_Pos);     // memory to peripheral
  DMA1_Stream5->CR  |= DMA_SxCR_EN;

  DAC->CR = DAC_CR_EN1 | DAC_CR_TEN1 | (5U << DAC_CR_TSEL1_Pos) | DAC_CR_DMAEN1
          | DAC_CR_EN2 | DAC_CR_TEN2 | (5U << DAC_CR_TSEL2_Pos);   // TSEL 5 = TIM4

  uint32_t clk = apb1TimerClock();
  uint32_t ticks = (uint32_t)((float)clk / (hz * DAC_PTS) + 0.5f);
  if (ticks < 2) ticks = 2;
  uint32_t psc = (ticks - 1) / 65536UL;
  uint32_t arr = (uint32_t)((float)ticks / (float)(psc + 1) + 0.5f) - 1;
  TIM4->CR1 = 0;
  TIM4->CR2 = 0;                       // keep TRGO quiet while the update loads PSC
  TIM4->PSC = psc;
  TIM4->ARR = arr;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->SR  = 0;
  TIM4->CR2 = 2U << TIM_CR2_MMS_Pos;   // TRGO = update
  TIM4->CR1 = TIM_CR1_CEN;

  g_dacHz  = (float)clk / ((float)(psc + 1) * (float)(arr + 1)) / (float)DAC_PTS;
  g_dacDeg = deg;
  g_dacAmp = amp;
}
#endif

// ═══ commands ═════════════════════════════════════════════════════════════════

static void printStatus() {
  Serial.print(F("$STATUS,fw=")); Serial.print(F(FW_VERSION));
  Serial.print(F(",stream=")); Serial.print(g_streaming ? 1 : 0);
  Serial.print(F(",relay=")); Serial.print(g_relay ? 1 : 0);
  Serial.print(F(",trip=")); Serial.print(g_tripLatch ? 1 : 0);
  Serial.print(F(",dirty=")); Serial.print(g_dirty ? 1 : 0);
  Serial.print(F(",scanus4=")); Serial.print(g_scanUs4, 3);
  Serial.print(F(",scanus2=")); Serial.print(g_scanUs2, 3);
  Serial.print(F(",capmax=")); Serial.print(CAP_SAMPLES);
  Serial.print(F(",oled=")); Serial.print(g_oledOk ? 1 : 0);
  Serial.print(F(",ir=")); Serial.print(g_irAttached >= 0 ? g_irAttached : 0);
  Serial.print(F(",primed=")); Serial.print(g_primed ? 1 : 0);
  Serial.print(F(",ch=")); Serial.print(CH_IP); Serial.print('/'); Serial.print(CH_IN);
  Serial.print('/'); Serial.print(CH_VP); Serial.print('/'); Serial.print(CH_VN);
  Serial.print(F(",rng=")); Serial.print(g_rng);
  Serial.print(F(",rngmode=")); Serial.print(cfg.rngMode);
  Serial.print(F(",rngpin=")); Serial.print(cfg.rngPin);
  Serial.print(F(",wh=")); Serial.print((float)g_wh, 6);
#if BOARD_F405
  Serial.print(F(",board=F405,dac=")); Serial.println(g_dacHz, 3);
#else
  Serial.println(F(",board=RA4M1"));
#endif
}

// The chip's factory unique ID as hex, so a snapshot names the board it came from.
static void printUid() {
#if BOARD_F405
  uint32_t w[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
  const uint8_t n = 3;
#else
  const uint32_t* w = R_BSP_UniqueIdGet()->unique_id_words;
  const uint8_t n = 4;
#endif
  for (uint8_t k = 0; k < n; k++)
    for (int8_t s = 28; s >= 0; s -= 4) Serial.print((w[k] >> s) & 0xF, HEX);
}

// The stored block as configLoad() sees it (migrations included); RAM is
// left as it was. Only loop() reads cfg, so the brief swap is invisible.
static bool configPeek(Config& out) {
  Config keep = cfg;
  bool m = g_migrated, ml = g_migratedLo;
  bool ok = configLoad();
  out = cfg;
  cfg = keep; g_migrated = m; g_migratedLo = ml;
  return ok;
}

// !DUMP: every key with its group, for a snapshot file that can be copied to
// another board group by group. src RAM = what is running, SAVED = flash/EEPROM.
static void printDump(const Config& c, bool saved) {
  Serial.print(F("$DUMPHDR,fw=")); Serial.print(F(FW_VERSION));
  Serial.print(F(",board=")); Serial.print(BOARD_F405 ? F("F405") : F("RA4M1"));
  Serial.print(F(",uid=")); printUid();
  Serial.print(F(",cfgver=")); Serial.print(c.version);
  Serial.print(F(",src=")); Serial.print(saved ? F("SAVED") : F("RAM"));
  Serial.print(F(",dirty=")); Serial.println(g_dirty ? 1 : 0);
  for (size_t k = 0; k < CFG_FIELD_COUNT; k++) {
    const CfgField& f = CFG_FIELDS[k];
    Serial.print(F("$DUMP,")); Serial.print(CFG_GROUP_NAMES[f.group]);
    Serial.print(','); Serial.print(f.key); Serial.print(',');
    if (f.type == FT_FLOAT) Serial.println(fieldGetFrom(c, f), 7);
    else                    Serial.println((long)fieldGetFrom(c, f));
  }
  Serial.println(F("$DUMPEND"));
}

// A digital pin free for the relay, range relay or IR receiver: not analog,
// I2C, or (Feather) the NeoPixel and the SPI flash.
#if BOARD_F405
static bool pinFree(uint8_t p) { return p == 1 || p == 5 || p == 6 || (p >= 9 && p <= 13); }
#define PIN_RULE "D1, D5, D6 or D9-D13"
#else
static bool pinFree(uint8_t p) { return p > 5; }
#define PIN_RULE "not analog or I2C"
#endif

// n-th comma-separated field after the command, or def
static bool argAt(const char* s, uint8_t n, char* out, size_t outn) {
  const char* p = strchr(s, ',');
  for (uint8_t k = 0; p && k < n; k++) p = strchr(p + 1, ',');
  if (!p) return false;
  p++;
  size_t len = strcspn(p, ",");
  if (len == 0 || len >= outn) return false;
  memcpy(out, p, len); out[len] = 0;
  return true;
}
static float argF(const char* s, uint8_t n, float def) {
  char b[24];
  return argAt(s, n, b, sizeof(b)) ? (float)atof(b) : def;
}

void handleCommand(char* cmd) {
  char key[16], val[24];
  if (!strcmp(cmd, "!START"))       { g_streaming = true;  Serial.println(F("$OK,start")); }
  else if (!strcmp(cmd, "!STOP"))   { g_streaming = false; Serial.println(F("$OK,stop")); }
  else if (!strcmp(cmd, "!STATUS")) printStatus();
  else if (!strcmp(cmd, "!CFG")) {
    for (size_t k = 0; k < CFG_FIELD_COUNT; k++) printField(CFG_FIELDS[k]);
    Serial.println(F("$CFGEND"));
  }
  else if (!strcmp(cmd, "!DUMP")) printDump(cfg, false);
  else if (!strcasecmp(cmd, "!DUMP,SAVED")) {
    Config s;
    if (configPeek(s)) printDump(s, true);
    else Serial.println(F("$ERR,no valid config stored"));
  }
  else if (!strncmp(cmd, "!GET,", 5)) {
    const CfgField* f = findField(cmd + 5);
    if (f) printField(*f); else Serial.println(F("$ERR,unknown key"));
  }
  else if (!strncmp(cmd, "!SET,", 5)) {
    if (!argAt(cmd, 0, key, sizeof(key)) || !argAt(cmd, 1, val, sizeof(val))) {
      Serial.println(F("$ERR,SET needs <key>,<value>")); return;
    }
    const CfgField* f = findField(key);
    if (!f) { Serial.println(F("$ERR,unknown key")); return; }
    uint8_t oldPin = cfg.relayPin, oldRng = cfg.rngPin;
    uint8_t oldOled = cfg.oled, oldAddr = cfg.oledAddr, oldIr = cfg.irPin;
    fieldSet(*f, (float)atof(val));
    if (!pinFree(cfg.relayPin) || (cfg.rngPin && cfg.relayPin == cfg.rngPin)) {
      cfg.relayPin = oldPin;
      Serial.println(F("$ERR,RLYPIN must be " PIN_RULE ", and not the range relay pin"));
    }
    if (cfg.rngPin != 0 && (!pinFree(cfg.rngPin) || cfg.rngPin == cfg.relayPin || cfg.rngPin == cfg.irPin)) {
      cfg.rngPin = oldRng;
      Serial.println(F("$ERR,RNGPIN must be " PIN_RULE ", and not the relay or IR pin"));
    }
    if (f->offset == offsetof(Config, rngMode)) {
      g_rngLatched = false;
      if (cfg.rngMode == 3) rangeSet(RNG_LO, RW_CMD, false);   // auto-delayed starts LOW
    }
    if (f->offset == offsetof(Config, rngPin) || f->offset == offsetof(Config, rngInv)) {
      if (oldRng != cfg.rngPin) g_rng = RNG_HI;
      rangePinInit(oldRng);
    }
    rangeRefresh(RW_CMD);                 // also rescales after a gain / offset change
    if (f->offset == offsetof(Config, relayPin) || f->offset == offsetof(Config, relayInv))
      relayPinInit(oldPin);
    if (f->offset == offsetof(Config, irPin)) {
      if (cfg.irPin != 0 && (!pinFree(cfg.irPin) || cfg.irPin == cfg.relayPin || cfg.irPin == cfg.rngPin)) {
        cfg.irPin = oldIr;
        Serial.println(F("$ERR,IRPIN must be " PIN_RULE ", and not a relay pin"));
      }
      irInit();
    }
    if (f->offset == offsetof(Config, pixel) || f->offset == offsetof(Config, liveVac)) {
      g_pixState = -1;
      if (!cfg.pixel) { pixelInit(); }
      else if (g_haveLast) pixelUpdate(g_last.vrms);
    }
    if (cfg.oled != oldOled || cfg.oledAddr != oldAddr || f->offset == offsetof(Config, oledRot)) {
      oledBlank();                        // off, or a different screen type
      oledInit();
    }
    g_dirty = true;
    printField(*f);
  }
  else if (!strcmp(cmd, "!SAVE")) {
#if BOARD_F405
    // the flash erase stalls the CPU, and with it every trip check, for 1-2 s
    if (g_relay) { Serial.println(F("$ERR,open the load relay before saving (flash erase stalls the board)")); return; }
#endif
    if (configSave()) { g_dirty = false; Serial.println(F("$OK,save")); }
    else Serial.println(F("$ERR,save verify failed"));
  }
  else if (!strcmp(cmd, "!LOAD")) {
    uint8_t oldPin = cfg.relayPin, oldRng = cfg.rngPin;
    if (configLoad()) {
      g_dirty = false; relayPinInit(oldPin); oledInit(); irInit(); pixelInit();
      g_rngLatched = false; rangePinInit(oldRng); rangeRefresh(RW_CMD);
      Serial.println(F("$OK,load"));
    }
    else Serial.println(F("$ERR,no valid config stored"));
  }
  else if (!strcmp(cmd, "!DEFAULTS")) {
    uint8_t oldPin = cfg.relayPin, oldRng = cfg.rngPin;
    configDefaults(); g_dirty = true; relayPinInit(oldPin); oledInit(); irInit(); pixelInit();
    g_rngLatched = false; rangePinInit(oldRng); rangeRefresh(RW_CMD);
    Serial.println(F("$OK,defaults"));
  }
  else if (!strncmp(cmd, "!RELAY,", 7)) {
    g_autoOffArmed = false;               // a manual command cancels the timer
    g_primed = false;
    bool on = atoi(cmd + 7) != 0;
    if (on) { g_tripLatch = false; rangeBeforeClose(); }
    setRelay(on);
  }
  else if (!strncmp(cmd, "!RANGE,", 7)) {
    cfg.rngMode = (uint8_t)constrain(atoi(cmd + 7), 0, 3);
    g_rngLatched = false;
    g_dirty = true;
    if (!cfg.rngPin) Serial.println(F("$ERR,no range relay (RNGPIN 0) - HIGH only"));
    if (cfg.rngMode == 3) rangeSet(RNG_LO, RW_CMD, false);    // auto-delayed starts LOW
    rangeRefresh(RW_CMD);
    g_rngReport = RW_CMD;                 // report even when nothing moved
    rangeReport();
  }
  else if (!strcmp(cmd, "!ZERO"))  zeroChannels();
  else if (!strcmp(cmd, "!EZERO")) { g_wh = 0; Serial.println(F("$OK,ezero")); }
  else if (!strcmp(cmd, "!DC"))    measureDC(200);
  else if (!strncmp(cmd, "!DC,", 4)) measureDC((uint32_t)atol(cmd + 4));
  else if (!strncmp(cmd, "!CAP", 4)) {
    g_cap_req.rate       = (uint32_t)argF(cmd, 0, 20000);
    g_cap_req.n          = (uint16_t)argF(cmd, 1, 4000);
    g_cap_req.ch         = (uint8_t)argF(cmd, 2, 2);
    g_cap_req.trig       = (uint8_t)argF(cmd, 3, 0);
    g_cap_req.level      = argF(cmd, 4, 0.5f);
    g_cap_req.pre        = argF(cmd, 5, 10);
    g_cap_req.timeout_ms = (uint32_t)argF(cmd, 6, 5000);
    g_cap_req.relay_us   = (int32_t)argF(cmd, 7, -1);
    g_cap_req.ipk_a      = argF(cmd, 8, 0);
    g_cap_req.ov_a       = argF(cmd, 9, 0);
    g_cap_req.ov_cyc     = (uint16_t)argF(cmd, 10, 3);
    g_cap_req.auto_ms    = (uint32_t)argF(cmd, 11, 0);
    g_cap_req.open_after = (uint8_t)argF(cmd, 12, 0);
    g_cap_req.avg        = (uint8_t)constrain(argF(cmd, 13, 1), 1, 64);
    g_cap_req.rng        = (int8_t)constrain(argF(cmd, 14, -1), -1, 1);
    g_cap_req.mon        = argF(cmd, 15, 0) != 0;
    if (g_cap_req.ch > 2 || g_cap_req.trig > 6) { Serial.println(F("$ERR,bad CAP args")); return; }
    if (g_cap_req.rate > 500000UL) g_cap_req.rate = 500000UL;
    g_cap_pending = true;
  }
  else if (!strcmp(cmd, "!ABORT")) { /* the byte itself aborts a waiting capture */ }
#if BOARD_F405
  else if (!strncmp(cmd, "!DACTEST,", 9)) {
    float hz = argF(cmd, 0, 0), deg = argF(cmd, 1, 0), amp = argF(cmd, 2, 1.0f);
    if (hz == 0.0f) dacTestStop();
    // the DAC output buffer cannot get within ~0.2 V of either rail
    else if (hz < 0.5f || hz > 1000.0f || amp <= 0.0f || amp > 1.4f) {
      Serial.println(F("$ERR,DACTEST needs <0.5-1000 Hz>,<deg>[,<0-1.4 V>]; 0 stops")); return;
    }
    else dacTestStart(hz, deg, amp);
    Serial.print(F("$DAC,")); Serial.print(g_dacHz, 3);
    Serial.print(','); Serial.print(g_dacDeg, 2);
    Serial.print(','); Serial.println(g_dacAmp, 3);
  }
#endif
  else if (!strcmp(cmd, "!ID"))    { Serial.print(F("$READY,120VMON,")); Serial.println(F(FW_VERSION)); }
  else { Serial.print(F("$ERR,unknown command: ")); Serial.println(cmd); }
}

static char    g_rx[96];
static uint8_t g_rxpos = 0;

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_rxpos) { g_rx[g_rxpos] = 0; handleCommand(g_rx); g_rxpos = 0; }
    } else if (g_rxpos < sizeof(g_rx) - 1) {
      g_rx[g_rxpos++] = c;
    }
  }
}

// ═══ setup / loop ═════════════════════════════════════════════════════════════

void setup() {
  configDefaults();
  bool loaded = configLoad();
  // relay open before anything else can go wrong
  pinMode(cfg.relayPin, OUTPUT);
  relayDrive(false);
  g_rng = RNG_HI;                       // and the high-current shunt
  rangePinInit(0);
  rngCal();

  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 1500) delay(10);   // standalone: don't wait for a host

  cycInit();
  CYC_PER_US_ISR = CYC_PER_US;
  adcInit();
  g_scanUs4 = measureScanUs(MASK_ALL);
  g_scanUs2 = measureScanUs(MASK_I);

  Wire.begin();
  oledInit();
  pixelInit();
  irInit();

  Serial.print(F("$READY,120VMON,")); Serial.println(F(FW_VERSION));
  if (!loaded) Serial.println(F("$ERR,no valid config in EEPROM - defaults loaded (uncalibrated)"));
  if (g_migrated) {
    bool ok = configSave();
    if (!ok) Serial.println(F("$ERR,config migrated to v5 but the save failed"));
    else if (g_migratedLo) Serial.println(F("$OK,config migrated to v5, calibration kept (IGAINLO = IGAIN/10 until calibrated)"));
    else Serial.println(F("$OK,config migrated to v5, calibration kept"));
  }
  rangeRefresh(RW_BOOT);                // RNGMODE 2 starts LOW
  rangeReport();
  printStatus();
}

void loop() {
  readSerial();
  rangeReport();
  serviceRemote();
  serviceAutoOff();
  if (g_cap_pending) {
    g_monFresh = false;
    runCapture();
    // a reading made during the capture stands in for a metering window
    if (g_monFresh) { rangeAuto(g_last); pixelUpdate(g_last.vrms); }
    else g_lastWinMs = 0;
    return;
  }
  serviceMeasurement();
  if (g_haveLast) drawMeter();
}
