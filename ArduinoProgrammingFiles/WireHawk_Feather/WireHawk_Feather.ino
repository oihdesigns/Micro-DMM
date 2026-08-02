/*
 * WireHawk_Feather.ino
 *
 * Motor drive / fault tester  +  open-lead detector, on ONE board.
 * Target: Adafruit Feather RP2350 (HSTX or Adalogger), earlephilhower core.
 * FQBN: rp2040:rp2040:adafruit_feather_rp2350_hstx
 *
 * This is WireHawk.ino (relay control, current sense, diagnosis, safety, USB
 * link black box, EEPROM config, GUI protocol) merged with
 * OpenLeadDetect_Feather_Headless.ino (ADS1x15 differential voltmeter with
 * open/closed lead classification).  The two subsystems share the same test
 * leads, so they are interlocked -- see LEAD SUBSYSTEM below.
 *
 * ── Hardware ──────────────────────────────────────────────────────
 *   A0   (GPIO26) current sense, analog.  Mid-rail = 0 A, above mid-rail =
 *        forward current.  ~1.590 V measured zero, 500 mV/A default (MVPERA).
 *        CALIBRATE BEFORE TRUSTING READINGS: !ZERO with the relay open, then
 *        !CAL,<mA> with a known current.
 *   D24  relay control, output.  HIGH = relay closed = motor energised.
 *   D10  load-side sense bit, LOW digit of the two-bit code   (was D3)
 *   D11  load-side sense bit, HIGH digit of the two-bit code  (was D4)
 *   D5   bridge MOSFET gate for the lead test (active LOW by default)
 *   D6   lead-status output, high-order bit
 *   D9   lead-status output, low-order bit
 *   I2C  (SDA GPIO2 / SCL GPIO3) ADS1015 or ADS1115 at 0x48, reading its own
 *        A0-A1 differential across the test leads.
 *   NeoPixel (GPIO21) status colour.
 *
 * The sense-bit pins moved off D3/D4 because on a Feather those are the I2C
 * bus the lead ADC needs.  Everything else keeps its WireHawk meaning.
 *
 * ── The D11/D10 sense code ("XY", X = D11, Y = D10) ───────────────
 *   10  motor present   -- winding continuity seen, no voltage on the load
 *   01  no motor        -- open circuit / nothing connected
 *   11  voltage present -- the load is energised
 *   00  unknown         -- sense circuit unpowered, shorted, or NOT WIRED YET
 * USEIO defaults to 0: the state machine runs on current alone and the raw
 * bits are still reported so the wiring can be checked.  Set USEIO=1 (and
 * !SAVE) once they are hooked up.
 *
 * ── Diagnosis ─────────────────────────────────────────────────────
 * Relay OPEN:
 *   10 -> READY          motor present, waiting
 *   01 -> NO_MOTOR       nothing connected
 *   11 -> BACKFEED       voltage on the load with the relay open: relay welded
 *                        shut, or the load is being driven from elsewhere
 *   00 -> UNKNOWN
 * Relay CLOSED:
 *   11 + current      -> RUNNING
 *   11 + no current   -> STARTING during the INRUSHMS grace window, then
 *                        STALL once STALLMS of voltage-without-current has
 *                        elapsed  ("motor fault stall")
 *   10                -> NO_VOLTAGE   relay closed but the load never got
 *                        voltage: relay contacts, fuse, or supply
 *   10 + current      -> SENSEFAULT   current flowing but the voltage bit is
 *                        low -- the sense input disagrees with reality
 *   01                -> NO_MOTOR     open winding / lead fell off
 *   00                -> UNKNOWN
 * With USEIO=0 the same logic runs without the voltage bit, so a closed relay
 * with no current reads NO_CURRENT rather than STALL.
 *
 * State changes are debounced by STABLEN consecutive passes.  OVERCURRENT is
 * a latched fault: the relay drops immediately and stays down until !CLEAR,
 * !OFF, or a fresh !ON.
 *
 * ── LEAD SUBSYSTEM (from OpenLeadDetect) ──────────────────────────
 * A resistor bridge is switched onto the test leads through the D5 MOSFET and
 * the resulting differential is read by the ADS1x15.  A small bridge voltage
 * means the leads are joined by a low resistance (CLOSED); a large one means
 * they are open (FLOAT).  The test is only valid on DEAD leads, so it runs
 * only when the measured lead voltage is below CORFTRIG.
 *
 * Lead code, mirrored on D6/D9 as <hi><lo> and reported as leadCode:
 *   00 NONE     detection suppressed (!CFSUP,1) or the lead ADC is absent
 *   01 FLOAT    open circuit between the leads
 *   10 CLOSED   continuity between the leads
 *   11 VOLTAGE  voltage present -- no lead test possible
 *
 * INTERLOCK: only the BRIDGE is interlocked, not the measurement.  With the
 * relay closed the bridge is held off and no open/closed test runs (the code
 * reads NONE if the leads are dead), but the voltage reading keeps updating --
 * so leadV shows the LIVE supply voltage during a run, which is what "voltage
 * present" is claiming.  LEADHOT=1 lifts the interlock (the voltage check still
 * gates the bridge on its own); leave it 0 unless the load is isolated.
 * LEADEN=0 disables the subsystem entirely and the board behaves exactly like
 * plain WireHawk.
 *
 * ── Safety ────────────────────────────────────────────────────────
 * The relay is driven LOW and the bridge OFF before their pinModes are set, so
 * a reset or a reflash cannot pulse the motor or energise the bridge.
 * MAXRUNMS force-opens the relay after a maximum run time (default 30 s, 0
 * disables) and OCTRIP opens it on over-current.  !STOP always opens the relay
 * and clears any timed run.  HOSTTO (default 2000 ms) opens the relay when a
 * host that WAS pinging goes quiet; it arms only after the first !PING of a
 * run, so a hand-typed serial terminal is never cut off mid-test.
 *
 * Nothing in the lead path may stall the control loop: the ADS is read at most
 * once per LEADMS, a missing ADC is detected once at boot and then skipped
 * forever, and the NeoPixel is never blinked with delay().
 *
 * ── Is it the board or the host?  ($WH seq / maxLoopUs / txSkips) ──
 * "The host stopped receiving" and "the board stopped sending" look identical
 * from the host, so every $WH row carries the evidence to tell them apart:
 *   seq        increments for every row the board DECIDED to send, even one it
 *              then had to drop.  A jump in seq at the host = rows that were
 *              not delivered; seq continuing 1-by-1 across a long wall-clock
 *              silence = the board itself was not running.
 *   txSkips    rows the BOARD dropped because the CDC buffer was full, i.e.
 *              the host was not reading.  seq gap WITH a txSkips rise = host
 *              side; seq gap WITHOUT one = the row left the board and was lost
 *              in transit (link, driver, or re-enumeration).
 *   maxLoopUs  worst loop() time since the previous row, cleared each row.
 * $STATUS repeats these plus txroom (Serial.availableForWrite()).
 * !LINK dumps the board's record of every USB dropout and !HIST backfills the
 * samples a dropout cost the host, so the data survives either way.
 *
 * ── Serial protocol (115200 baud, line based) ─────────────────────
 * Motor commands (newline terminated):
 *   !ON / !OFF            close / open the relay
 *   !RELAY,<0|1>          same, explicit
 *   !RUN,<ms>             close the relay for <ms> then open it
 *   !STOP                 open the relay, cancel a timed run
 *   !PING                 keepalive; arms the HOSTTO cutoff, replies $PONG
 *   !LINK                 dump the board's record of every USB dropout
 *   !HIST[,<sinceMs>]     replay recorded samples newer than <sinceMs>
 *   !CLEAR                clear a latched fault (OVERCURRENT)
 *   !RST                  reset the current statistics
 *   !ZERO                 capture A0 now as 0 A (relay must be open)
 *   !CAL,<mA>             set MVPERA from a known current flowing now
 *   !CAP[,<ms>]           inrush capture: sample A0 fast across relay close
 *   !STREAM[,0|1]         continuous $WH streaming on/off (bare = toggle)
 *   !RATE,<ms>            stream interval
 * Lead commands:
 *   !LEAD                 print one $LEAD row now
 *   !CFSUP[,0|1]          suppress the lead test, volt-only (bare = toggle)
 *   !ACMODE[,0|1]         AC/VRMS lead measurement (bare = toggle)
 *   !VMODE,<0|1|2>        voltage gate: 0=auto 1=force "voltage present"
 *                         2=force "dead" (runs the bridge test regardless)
 *   !MOSFET,<-1|0|1>      bridge: -1=auto 0=hold off 1=hold on
 *   !LSTREAM[,0|1]        raw ADS differential streaming ($LD rows)
 *   !LRATE,<ms>           $LD interval
 *   !LCAP[,<ms>]          capture the ADS across a bridge toggle, then dump
 * Config:
 *   !SET,<key>,<value>    set a config value in RAM (effective immediately)
 *   !GET,<key>            report one config value
 *   !CFG                  dump every config key ($CFG rows + $CFGEND)
 *   !SAVE / !LOAD / !DEFAULTS / !SN[,<value>] / !STATUS / !?
 * Data out:
 *   $WH,<ms>,<relay>,<mA>,<avgMA>,<minMA>,<maxMA>,<pkMA>,<rippleMA>,<mV>,
 *       <d10>,<d11>,<io>,<state>,<stateName>,<runMs>,<seq>,<maxLoopUs>,
 *       <txSkips>,<leadCode>,<leadName>,<leadV>,<bridgeAvg>
 *     Fields through <txSkips> are byte-for-byte the original WireHawk row, so
 *     an older GUI still parses it; the four lead fields are appended.
 *   $LEAD,<code>,<name>,<leadV>,<bridgeV>,<bridgeAvg>,<rEst>,<conf%>,<vzero>,
 *       <ac>,<suppressed>                    sent on every lead-code change
 *   $STATE,<code>,<name>,<detail>            motor state change
 *   $BOOT,<resetReason>,<cfgSource>,<board>  sent once on boot
 *   $STATUS,... / $CFG,<key>,<value> / $CFGEND / $SN,<value>
 *   $PONG,<ms>,<relay>
 *   $LINK,... / $LINKEV,... / $LINKEND
 *   $HIST,<held>,<sinceMs> / $H,<ms>,<mA>,<state>,<io>,<relay>,<lead> /
 *   $HISTEND,<sent>
 *   $OK,<what> / $ERR,<what>[,detail] / $MSG,<text>
 *   $CAPSTART,<n>,<trigUs>,<durMs>,<fullScale>,<vrefMv>,<zeroMv>,<mvPerA>
 *   $CAP,<t_us>,<raw>,<io>  /  $CAPEND                 current capture
 *   $LCAPSTART,<n>,<toggleUs>,<durMs>,<fullScale>,<mvPerBit>
 *   $LCAP,<t_us>,<rawDiff>  /  $LCAPEND                lead capture
 *   $LD,<ms>,<rawDiff>,<diffV>                         lead raw stream
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <strings.h>

// ══════════════════════════════════════════════════════════════════
//  PLATFORM  (Adafruit Feather RP2350)
// ══════════════════════════════════════════════════════════════════
// The original WireHawk built for the Nano ESP32 and the XIAO RA4M1; this
// variant is Feather-specific because the pin map (D24 relay, I2C lead ADC)
// is.  Keep WireHawk.ino for those boards.
#if !defined(ARDUINO_ARCH_RP2040)
  #error "WireHawk_Feather targets the Adafruit Feather RP2350 (rp2040 core)."
#endif

#define WH_BOARD  "feather_rp2350"
const int ADC_BITS  = 12;          // RP2350 SAR ADC
const int CAP_MAX   = 1600;        // plenty of SRAM on the RP2350
const int HIST_MAX  = 1200;        // ~2 min at RATEMS=100
const int ADC_FULL_SCALE = (1 << ADC_BITS) - 1;

// ── Lead ADC selection: comment/uncomment ONE ─────────────────────
//#define USE_ADS1115    // 16-bit
#define USE_ADS1015      // 12-bit

// ── Bridge MOSFET logic: comment/uncomment ONE ────────────────────
//#define VBRIDGE_ACTIVE_HIGH
#define VBRIDGE_ACTIVE_LOW

#ifdef VBRIDGE_ACTIVE_HIGH
  #define BRIDGE_ON   HIGH
  #define BRIDGE_OFF  LOW
#else
  #define BRIDGE_ON   LOW
  #define BRIDGE_OFF  HIGH
#endif

#ifdef USE_ADS1115
Adafruit_ADS1115 ads;
#define LEAD_ADC_NAME "ADS1115"
#else
Adafruit_ADS1015 ads;
#define LEAD_ADC_NAME "ADS1015"
#endif

#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 21
#endif
Adafruit_NeoPixel whPixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ══════════════════════════════════════════════════════════════════
//  PIN MAP
// ══════════════════════════════════════════════════════════════════
const int CURRENT_PIN = A0;   // bidirectional current sense, mid-rail = 0 A
const int RELAY_PIN   = D24;  // relay control, HIGH = closed = motor on
const int SENSE_LO    = D10;  // sense code low digit  ("Y")
const int SENSE_HI    = D11;  // sense code high digit ("X")
const int VBRIDGE_PIN = D5;   // bridge MOSFET gate for the lead test
const int LEADHI_PIN  = D6;   // lead-status output, high bit
const int LEADLO_PIN  = D9;   // lead-status output, low bit

// ══════════════════════════════════════════════════════════════════
//  CONFIGURATION  (EEPROM backed)
// ══════════════════════════════════════════════════════════════════
const uint32_t CFG_MAGIC   = 0x57484631UL;   // "WHF1"
const uint16_t CFG_VERSION = 1;
const int      EEPROM_ADDR = 0;

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  float    zeroMv;        // ZEROMV     A0 reading at 0 A (mV)
  float    mvPerAmp;      // MVPERA     sense sensitivity (mV per amp)
  float    curThreshMa;   // CURTHRESH  |mA| at or above this = current flowing
  float    ocTripMa;      // OCTRIP     over-current trip (mA), 0 = disabled

  uint32_t inrushMs;      // INRUSHMS   grace window after relay close
  uint32_t stallMs;       // STALLMS    voltage-without-current before STALL
  uint32_t maxRunMs;      // MAXRUNMS   force relay open after this, 0 = off
  uint32_t hostToMs;      // HOSTTO     open relay if a PINGING host goes silent

  uint16_t adcAvg;        // ADCAVG     A0 samples averaged per pass
  uint16_t rateMs;        // RATEMS     $WH stream interval
  uint16_t vrefMv;        // VREF       full-scale mV for the current sense
  uint16_t capPreMs;      // CAPPRE     capture pre-trigger window
  uint16_t capMs;         // CAPMS      default total capture duration

  uint8_t  useIo;         // USEIO      1 = use the sense bits in the diagnosis
  uint8_t  ioInvert;      // IOINVERT   1 = sense bits are active low
  uint8_t  ioMode;        // IOMODE     0 = pulldown, 1 = pullup, 2 = plain input
  uint8_t  absCur;        // ABSCUR     1 = compare |current| (either direction)
  uint8_t  autoStream;    // STREAM     1 = stream $WH from boot
  uint8_t  useCalMv;      // ADCMV      reserved (no calibrated read on RP2350)
  uint8_t  stableN;       // STABLEN    passes before a state change is accepted
  uint8_t  ledOn;         // LED        1 = drive the onboard NeoPixel
  uint8_t  capRestore;    // CAPREST    1 = restore the relay after a capture

  // ── lead detector ──
  uint8_t  leadEn;        // LEADEN     1 = run the open-lead subsystem
  uint8_t  leadHot;       // LEADHOT    1 = allow the lead test with the relay closed
  uint8_t  acMode;        // ACMODE     1 = AC/VRMS lead measurement
  uint16_t leadMs;        // LEADMS     lead measurement interval (ms)
  float    corFTrig;      // CORFTRIG   |V| below this = leads are dead
  float    clsdThres;     // CLSDTHRS   bridge V below this = CLOSED
  float    bridgeBaseV;   // BRBASEV    bridge V at the ~10K calibration point
  float    vScaleFull;    // VSCALE     counts-to-volts scale, bridge disconnected

  char     sn[16];        // unit serial number (preserved across !DEFAULTS)
  uint32_t crc;           // must stay last
};

Config cfg;

// ── Key table: one row per settable field ─────────────────────────
// NOTE: every type used in a function signature must be declared BEFORE the
// first function definition in the sketch -- the Arduino builder hoists its
// auto-generated prototypes to that point.
enum KType { K_F, K_U8, K_U16, K_U32, K_STR };

struct KeyDef {
  const char* name;
  KType       type;
  void*       ptr;
};

const KeyDef KEYS[] = {
  { "ZEROMV",    K_F,   &cfg.zeroMv      },
  { "MVPERA",    K_F,   &cfg.mvPerAmp    },
  { "CURTHRESH", K_F,   &cfg.curThreshMa },
  { "OCTRIP",    K_F,   &cfg.ocTripMa    },
  { "INRUSHMS",  K_U32, &cfg.inrushMs    },
  { "STALLMS",   K_U32, &cfg.stallMs     },
  { "MAXRUNMS",  K_U32, &cfg.maxRunMs    },
  { "HOSTTO",    K_U32, &cfg.hostToMs    },
  { "ADCAVG",    K_U16, &cfg.adcAvg      },
  { "RATEMS",    K_U16, &cfg.rateMs      },
  { "VREF",      K_U16, &cfg.vrefMv      },
  { "CAPPRE",    K_U16, &cfg.capPreMs    },
  { "CAPMS",     K_U16, &cfg.capMs       },
  { "USEIO",     K_U8,  &cfg.useIo       },
  { "IOINVERT",  K_U8,  &cfg.ioInvert    },
  { "IOMODE",    K_U8,  &cfg.ioMode      },
  { "ABSCUR",    K_U8,  &cfg.absCur      },
  { "STREAM",    K_U8,  &cfg.autoStream  },
  { "ADCMV",     K_U8,  &cfg.useCalMv    },
  { "STABLEN",   K_U8,  &cfg.stableN     },
  { "LED",       K_U8,  &cfg.ledOn       },
  { "CAPREST",   K_U8,  &cfg.capRestore  },
  { "LEADEN",    K_U8,  &cfg.leadEn      },
  { "LEADHOT",   K_U8,  &cfg.leadHot     },
  { "ACMODE",    K_U8,  &cfg.acMode      },
  { "LEADMS",    K_U16, &cfg.leadMs      },
  { "CORFTRIG",  K_F,   &cfg.corFTrig    },
  { "CLSDTHRS",  K_F,   &cfg.clsdThres   },
  { "BRBASEV",   K_F,   &cfg.bridgeBaseV },
  { "VSCALE",    K_F,   &cfg.vScaleFull  },
};
const int N_KEYS = sizeof(KEYS) / sizeof(KEYS[0]);

// ── Diagnosis states (declared early, same prototype-hoisting reason) ──
enum State {
  ST_IDLE = 0,      // relay open, nothing to say
  ST_READY,         // relay open, motor present
  ST_NO_MOTOR,      // open circuit
  ST_STARTING,      // relay closed, inside the inrush grace window
  ST_RUNNING,       // voltage + current
  ST_STALL,         // voltage, no current -> motor fault stall
  ST_NO_VOLTAGE,    // relay closed but the load is not energised
  ST_NO_CURRENT,    // relay closed, no current, voltage unconfirmed (USEIO=0)
  ST_BACKFEED,      // voltage / current with the relay OPEN
  ST_SENSEFAULT,    // current flowing but the voltage bit says otherwise
  ST_OVERCURRENT,   // latched over-current trip
  ST_UNKNOWN,       // sense code 00
  ST_COUNT
};

const char* const STATE_NAMES[ST_COUNT] = {
  "IDLE", "READY", "NO_MOTOR", "STARTING", "RUNNING", "STALL",
  "NO_VOLTAGE", "NO_CURRENT", "BACKFEED", "SENSEFAULT", "OVERCURRENT",
  "UNKNOWN"
};

// ── Lead codes: the value IS the two-bit output on LEADHI/LEADLO ──
#define LEAD_NONE     0x0   // suppressed / no lead ADC
#define LEAD_FLOAT    0x1   // open circuit
#define LEAD_CLOSED   0x2   // continuity
#define LEAD_VOLTAGE  0x3   // voltage present, no test possible

const char* const LEAD_NAMES[4] = { "NONE", "FLOAT", "CLOSED", "VOLTAGE" };

void loadDefaults(bool keepSerial) {
  char saved[16];
  strncpy(saved, cfg.sn, sizeof(saved));
  saved[sizeof(saved) - 1] = '\0';

  memset(&cfg, 0, sizeof(cfg));
  cfg.magic       = CFG_MAGIC;
  cfg.version     = CFG_VERSION;
  cfg.size        = sizeof(Config);

  cfg.zeroMv      = 1590.0f;   // measured prototype zero, NOT 3300/2
  cfg.mvPerAmp    = 500.0f;    // 40 mV / 80 mA
  cfg.curThreshMa = 20.0f;
  cfg.ocTripMa    = 0.0f;

  cfg.inrushMs    = 250;
  cfg.stallMs     = 300;
  cfg.maxRunMs    = 30000;
  cfg.hostToMs    = 2000;      // only armed once the host pings -- see safetyChecks

  cfg.adcAvg      = 16;
  cfg.rateMs      = 100;
  cfg.vrefMv      = 3300;
  cfg.capPreMs    = 25;
  cfg.capMs       = 400;

  cfg.useIo       = 0;         // sense bits not wired on the prototype
  cfg.ioInvert    = 0;
  cfg.ioMode      = 0;         // pulldown: unwired reads 00 = UNKNOWN, honestly
  cfg.absCur      = 1;
  cfg.autoStream  = 1;
  cfg.useCalMv    = 0;         // no analogReadMilliVolts on this core
  cfg.stableN     = 3;
  cfg.ledOn       = 1;
  cfg.capRestore  = 1;

  cfg.leadEn      = 1;
  cfg.leadHot     = 0;         // never bridge a live load unless asked
  cfg.acMode      = 0;
  cfg.leadMs      = 50;
  cfg.corFTrig    = 0.3f;      // 0.25 on the split-board unit
#ifdef USE_ADS1115
  cfg.clsdThres   = 0.07f;
#else
  cfg.clsdThres   = 1.42f;     // red board = 1.34
#endif
  cfg.bridgeBaseV = 1.262f;
  cfg.vScaleFull  = -69.95f;   // separate-boards version: 69.6023

  if (keepSerial) {
    strncpy(cfg.sn, saved, sizeof(cfg.sn));
    cfg.sn[sizeof(cfg.sn) - 1] = '\0';
  }
}

uint32_t configCrc(const Config& c) {
  const uint8_t* p = (const uint8_t*)&c;
  size_t n = offsetof(Config, crc);
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
  }
  return ~crc;
}

bool loadConfig() {
  Config tmp;
  EEPROM.get(EEPROM_ADDR, tmp);
  if (tmp.magic != CFG_MAGIC || tmp.version != CFG_VERSION ||
      tmp.size != sizeof(Config) || tmp.crc != configCrc(tmp))
    return false;
  cfg = tmp;
  return true;
}

bool saveConfig() {
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.size    = sizeof(Config);
  cfg.crc     = configCrc(cfg);
  EEPROM.put(EEPROM_ADDR, cfg);
  return EEPROM.commit();
}

// ══════════════════════════════════════════════════════════════════
//  STATE
// ══════════════════════════════════════════════════════════════════
const char* stateDetail(State s) {
  switch (s) {
    case ST_IDLE:        return "relay open";
    case ST_READY:       return "motor present, relay open";
    case ST_NO_MOTOR:    return "no motor / open circuit";
    case ST_STARTING:    return "energised, inrush window";
    case ST_RUNNING:     return "motor running";
    case ST_STALL:       return "voltage present, no current -- motor fault stall";
    case ST_NO_VOLTAGE:  return "relay closed but no voltage at the load";
    case ST_NO_CURRENT:  return "relay closed, no current (voltage unconfirmed)";
    case ST_BACKFEED:    return "voltage present with the relay OPEN";
    case ST_SENSEFAULT:  return "current flowing but voltage bit low";
    case ST_OVERCURRENT: return "over-current trip, relay latched open";
    default:             return "sense code 00 -- unknown";
  }
}

bool     relayOn      = false;
uint32_t relayOnMs    = 0;      // millis() when the relay last closed
uint32_t runUntilMs   = 0;      // 0 = no timed run pending
uint32_t lastCmdMs    = 0;

State    state        = ST_IDLE;
State    pendingState = ST_IDLE;
uint8_t  pendingCount = 0;
bool     faultLatched = false;

float    curMa        = 0.0f;   // this pass
float    curMv        = 0.0f;
uint8_t  senseLo = 0, senseHi = 0, ioCode = 0;
uint32_t lowCurSinceMs = 0;     // when |current| last dropped below threshold

// statistics, reset on relay close and on !RST
double   statSum = 0, statSumSq = 0;
uint32_t statN = 0;
float    statMin = 0, statMax = 0, statPeak = 0;

bool     streaming    = true;
uint32_t lastStreamMs = 0;
bool     pingSeen     = false;  // host has proven it sends !PING this run

// ── Link / loop health, so "is it the board or the host?" is measurable ──
uint32_t streamSeq  = 0;   // increments for EVERY row the board decided to send
uint32_t txSkips    = 0;   // rows the BOARD dropped because the CDC pipe was full
uint32_t maxLoopUs  = 0;   // worst loop() time since the last report

// ══════════════════════════════════════════════════════════════════
//  LEAD DETECTOR STATE
// ══════════════════════════════════════════════════════════════════
bool     leadAdcOk    = false;  // ADS answered at boot
uint32_t leadI2cHz    = 1000000;// bus clock the ADS actually answered on
uint8_t  leadCode     = LEAD_NONE;
uint8_t  leadPrev     = 0xFF;   // impossible, so the first pass announces
uint32_t lastLeadMs   = 0;

float    leadV        = 0.0f;   // reported lead voltage (median, or VRMS in AC)
float    leadRaw      = 0.0f;   // most recent instantaneous reading
float    leadMedian   = 0.0f;   // EMA of the instantaneous reading
float    leadActualV  = 0.0f;   // volts at the ADC input, before the divider scale
int16_t  leadCount    = 0;      // raw differential counts
bool     leadZero     = true;   // |V| below CORFTRIG -- the leads are dead
bool     leadFirstRun = true;
bool     cfSuppress   = false;  // !CFSUP -- volt-only, no bridge test

float    bridgeV      = 0.0f;   // last bridge sample
float    bridgeAvg    = 0.0f;   // EMA of |bridgeV|
float    closedConf   = 5.0f;   // 0..10, >5 = closed
bool     closedTrig = false, floatTrig = false;

// AC/VRMS accumulation
float    acSumSq      = 0.0f;
int      acSampleCount = 0;
const int AC_VRMS_SAMPLES = 32;      // ~2 cycles at 60 Hz
// Zero-crossing debounce: a genuine mains zero crossing at 120 Vrms passes
// through +/-0.5 V in ~8 us, so 4 ms of continuous "below threshold" can only
// mean the leads really are dead.
uint32_t acBelowStart = 0;
bool     acBelowActive = false;
const uint32_t AC_ZERO_DEBOUNCE_MS = 4;

// Lead diagnostics
bool     leadStream   = false;
uint32_t leadStreamMs = 200;
uint32_t lastLeadStreamMs = 0;
int      mosfetHold   = -1;     // -1 auto, 0 hold off, 1 hold on
enum VoltOverride { VOLT_AUTO, VOLT_FORCE_ON, VOLT_DISABLED };
VoltOverride voltOverride = VOLT_AUTO;

// ── ADC gain tables (values differ for ADS1115 vs ADS1015) ────────
#ifdef USE_ADS1115
const float GAIN_FACTOR_TWOTHIRDS = 0.1875;    // mV/bit, +/-6.144 V
const float GAIN_FACTOR_1         = 0.125;
const float GAIN_FACTOR_2         = 0.0625;
const float GAIN_FACTOR_4         = 0.03125;
const float GAIN_FACTOR_8         = 0.015625;
const float GAIN_FACTOR_16        = 0.0078125;
static const int ADC_COUNT_LOW_THRESH  = 10000;
static const int ADC_COUNT_HIGH_THRESH = 30000;
#define ADS_RATE_FAST  RATE_ADS1115_860SPS
#define ADS_RATE_MID   RATE_ADS1115_250SPS
#define ADS_RATE_SLOW  RATE_ADS1115_64SPS
#else
const float GAIN_FACTOR_TWOTHIRDS = 3.0;       // mV/bit, 16x the ADS1115 steps
const float GAIN_FACTOR_1         = 2.0;
const float GAIN_FACTOR_2         = 1.0;
const float GAIN_FACTOR_4         = 0.5;
const float GAIN_FACTOR_8         = 0.25;
const float GAIN_FACTOR_16        = 0.125;
static const int ADC_COUNT_LOW_THRESH  = 600;
static const int ADC_COUNT_HIGH_THRESH = 1800;
#define ADS_RATE_FAST  RATE_ADS1015_3300SPS
#define ADS_RATE_MID   RATE_ADS1015_490SPS
#define ADS_RATE_SLOW  RATE_ADS1015_250SPS
#endif

static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN
};
static const float kGainFactors[] = {
  GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2,
  GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16
};
static const int kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);
static size_t gainIndexVolt = 5;

// ══════════════════════════════════════════════════════════════════
//  CAPTURE BUFFERS
// ══════════════════════════════════════════════════════════════════
uint32_t  capUs[CAP_MAX];
uint16_t  capRaw[CAP_MAX];
uint8_t   capIo[CAP_MAX];

// Lead capture is far shorter -- the ADS converts in hundreds of us, so a few
// hundred samples already covers a multi-ms window.
const int LCAP_MAX = 600;
const uint32_t LCAP_PRE_US = 500;      // baseline sampled before the toggle
uint32_t lcapT[LCAP_MAX];
int16_t  lcapDiff[LCAP_MAX];

// ══════════════════════════════════════════════════════════════════
//  RESET CAUSE
// ══════════════════════════════════════════════════════════════════
// Switching an inductive load off arcs the relay contacts and dumps a wide EMI
// burst into everything nearby, so "the USB link died when I hit MOTOR OFF" has
// two very different causes.  A $BOOT line means the board really restarted
// (BROWNOUT = the coil/motor collapsed the rail, WDT = it hung); no $BOOT after
// a reconnect means only the USB link dropped.
const char* resetReasonName() {
  switch (rp2040.getResetReason()) {
    case RP2040::PWRON_RESET:    return "POWERON";
    case RP2040::RUN_PIN_RESET:  return "EXT";
    case RP2040::SOFT_RESET:     return "SW";
    case RP2040::WDT_RESET:      return "WDT";
    case RP2040::DEBUG_RESET:    return "DEBUG";
    case RP2040::GLITCH_RESET:   return "GLITCH";
    case RP2040::BROWNOUT_RESET: return "BROWNOUT";
    default:                     return "UNKNOWN";
  }
}

// ══════════════════════════════════════════════════════════════════
//  SERIAL OUTPUT  (bounded -- never let a stalled host hang the board)
// ══════════════════════════════════════════════════════════════════
// A native-USB CDC write can spin indefinitely when the host stops draining
// while the device still looks connected: ONE stalled write would hang the
// sketch, freezing the relay wherever it was.  So we never hand write() more
// bytes than the stack currently has room for -- chunk against
// availableForWrite() and give up on our own deadline instead of spinning.
uint32_t txDeadlineMisses = 0;   // host WAS connected but would not drain
uint32_t txNoHost         = 0;   // CDC reported no host at all -- a bus-level drop
uint32_t wedgeSinceMs     = 0;   // when output first started failing (0 = fine)

// These two counters are the difference between "the host is slow" and "the
// device fell off the USB bus", and they must never be conflated.
bool writeLine(const char* s, size_t len, uint32_t deadlineMs) {
  if (!Serial) { txNoHost++; return false; }   // no host: drop, never spin
  uint32_t t0 = millis();
  size_t off = 0;
  while (off < len) {
    // Deadline first, on EVERY pass: write() can legitimately return 0 even
    // with room reported, and a continue that skipped this check would spin
    // forever -- the exact failure being avoided.
    if ((millis() - t0) >= deadlineMs) {
      txDeadlineMisses++;
      if (!wedgeSinceMs) wedgeSinceMs = t0;
      return false;
    }
    int room = Serial.availableForWrite();
    size_t wrote = 0;
    if (room > 0) {
      size_t want = len - off;
      if (want > (size_t)room) want = (size_t)room;
      wrote = Serial.write((const uint8_t*)s + off, want);
      off += wrote;
    }
    if (wrote == 0) delay(1);      // no progress: yield, then re-check
  }
  if (wedgeSinceMs) {
    uint32_t stalledMs = millis() - wedgeSinceMs;
    wedgeSinceMs = 0;
    // Report recovery so a wedge that clears on its own is distinguishable
    // from one that needed the reset button.
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "$MSG,output recovered after %lu ms\n",
                     (unsigned long)stalledMs);
    if (n > 0) writeLine(buf, (size_t)n, 20);
  }
  return true;
}

// Drop-in replacement for Serial.printf that cannot hang.
// The buffer has to clear the longest line this firmware can produce, which is
// $STATUS with every counter saturated (~350 bytes).  If a line ever does
// overflow it is truncated ONTO a newline rather than losing it -- a dropped
// terminator would glue two records together and desync every parser
// downstream, which is far worse than a short line.
bool say(const char* fmt, ...) {
  char buf[420];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return false;
  if (n > (int)sizeof(buf) - 1) {
    n = (int)sizeof(buf) - 1;
    buf[n - 1] = '\n';
  }
  return writeLine(buf, (size_t)n, 20);
}

// ══════════════════════════════════════════════════════════════════
//  USB LINK BLACK BOX
// ══════════════════════════════════════════════════════════════════
// The MCU survives these dropouts (loop keeps running, seq keeps counting), so
// it records them and hands the history over once the host is back.  Each entry
// says WHEN the CDC link went down, for HOW LONG, and what the motor was doing
// at that instant -- which is what correlates dropouts with load current.  An
// empty log after a host-side "device disappeared" error is itself the answer:
// the device never left the bus, so the fault is on the host side.
struct LinkEvent {
  uint32_t atMs;      // millis() when the link went down
  uint32_t durMs;     // how long it stayed down
  float    mA;        // current at the moment it dropped
  uint32_t runMs;     // how long the motor had been running
  uint8_t  relay;
};
const int LINK_LOG_MAX = 8;
LinkEvent linkLog[LINK_LOG_MAX];
uint8_t   linkLogHead  = 0;    // next slot to write
uint16_t  linkDropCount = 0;   // total drops seen (may exceed LINK_LOG_MAX)
bool      cdcUp = false, cdcEverUp = false;
uint32_t  cdcDownAtMs = 0;
float     cdcDownMa = 0;
uint32_t  cdcDownRunMs = 0;
uint8_t   cdcDownRelay = 0;

void pollCdcLink() {
  bool up = (bool)Serial;
  if (up == cdcUp) return;
  cdcUp = up;
  if (up) {
    if (cdcEverUp) {                     // a completed dropout: record it
      LinkEvent& e = linkLog[linkLogHead];
      e.atMs  = cdcDownAtMs;
      e.durMs = millis() - cdcDownAtMs;
      e.mA    = cdcDownMa;
      e.runMs = cdcDownRunMs;
      e.relay = cdcDownRelay;
      linkLogHead = (linkLogHead + 1) % LINK_LOG_MAX;
      if (linkDropCount < 0xFFFF) linkDropCount++;
    }
    cdcEverUp = true;
  } else if (cdcEverUp) {
    cdcDownAtMs  = millis();
    cdcDownMa    = curMa;
    cdcDownRelay = relayOn ? 1 : 0;
    cdcDownRunMs = relayOn ? (millis() - relayOnMs) : 0;
  }
}

// ══════════════════════════════════════════════════════════════════
//  TELEMETRY HISTORY  (survives a USB dropout, so no data is lost)
// ══════════════════════════════════════════════════════════════════
// The MCU keeps running through dropouts, so it records every sample interval
// into RAM whether or not the row could be transmitted.  After the host
// reconnects it asks for the gap (!HIST,<sinceMs>) and stitches the missing
// samples back into the plot and the CSV.
struct HistRec {
  uint32_t ms;
  float    mA;
  uint8_t  state;
  uint8_t  io;
  uint8_t  relay;
  uint8_t  lead;      // fits in the struct's existing padding
};
HistRec  hist[HIST_MAX];
int      histHead  = 0;         // next slot to write
int      histCount = 0;         // valid entries (saturates at HIST_MAX)

void histRecord() {
  HistRec& h = hist[histHead];
  h.ms    = millis();
  h.mA    = curMa;
  h.state = (uint8_t)state;
  h.io    = ioCode;
  h.relay = relayOn ? 1 : 0;
  h.lead  = leadCode;
  histHead = (histHead + 1) % HIST_MAX;
  if (histCount < HIST_MAX) histCount++;
}

// ══════════════════════════════════════════════════════════════════
//  CURRENT MEASUREMENT
// ══════════════════════════════════════════════════════════════════
float readMilliVolts(uint16_t samples) {
  if (samples < 1) samples = 1;
  double acc = 0;
  for (uint16_t i = 0; i < samples; i++)
    acc += (double)analogRead(CURRENT_PIN) * cfg.vrefMv / ADC_FULL_SCALE;
  return (float)(acc / samples);
}

float mvToMa(float mv) {
  float span = cfg.mvPerAmp;
  if (fabsf(span) < 1e-6f) span = 1.0f;      // never divide by a zeroed key
  return (mv - cfg.zeroMv) * 1000.0f / span;
}

bool currentFlowing() {
  float v = cfg.absCur ? fabsf(curMa) : curMa;
  return v >= cfg.curThreshMa;
}

void readSenseBits() {
  int a = digitalRead(SENSE_LO);
  int b = digitalRead(SENSE_HI);
  if (cfg.ioInvert) { a = !a; b = !b; }
  senseLo = a ? 1 : 0;
  senseHi = b ? 1 : 0;
  ioCode = (uint8_t)((senseHi << 1) | senseLo);
}

void applyIoMode() {
  uint8_t m = (cfg.ioMode == 1) ? INPUT_PULLUP
            : (cfg.ioMode == 2) ? INPUT
                                : INPUT_PULLDOWN;
  pinMode(SENSE_LO, m);
  pinMode(SENSE_HI, m);
}

void resetStats() {
  statSum = statSumSq = 0;
  statN = 0;
  statMin = statMax = statPeak = 0;
}

void accumulateStats(float ma) {
  float mag = fabsf(ma);
  if (mag > statPeak) statPeak = mag;
  // min / max / average deliberately skip the inrush window so a startup spike
  // does not swamp the running numbers
  if (relayOn && (millis() - relayOnMs) < cfg.inrushMs) return;
  if (statN == 0) { statMin = statMax = ma; }
  else {
    if (ma < statMin) statMin = ma;
    if (ma > statMax) statMax = ma;
  }
  statSum   += ma;
  statSumSq += (double)ma * ma;
  statN++;
}

float statAvg() { return statN ? (float)(statSum / statN) : 0.0f; }

float statRipple() {
  if (statN < 2) return 0.0f;
  double mean = statSum / statN;
  double var  = statSumSq / statN - mean * mean;
  return var > 0 ? (float)sqrt(var) : 0.0f;
}

// ══════════════════════════════════════════════════════════════════
//  LEAD DETECTOR
// ══════════════════════════════════════════════════════════════════
// Ported from OpenLeadDetect_Feather_Headless.
//
// Two different permissions, and conflating them would either hide data or
// destroy hardware:
//   leadActive()        -- may we MEASURE?  Reading the differential leaves the
//                          bridge disconnected, so it is safe at any time and
//                          stays on through a motor run: that is what makes the
//                          reported lead voltage the LIVE supply voltage rather
//                          than a stale pre-run number.
//   leadBridgeAllowed() -- may we switch the BRIDGE across the leads?  Only
//                          with the relay open (or LEADHOT=1 for an isolated
//                          load).  This is the interlock.
bool leadActive() { return cfg.leadEn && leadAdcOk; }
bool leadBridgeAllowed() { return !relayOn || cfg.leadHot; }

void writeLeadOutputs(uint8_t code) {
  leadCode = code;
  digitalWrite(LEADHI_PIN, (code & 0x2) ? HIGH : LOW);
  digitalWrite(LEADLO_PIN, (code & 0x1) ? HIGH : LOW);
}

// Baseline-subtracted confidence: 0% at the ~10K bridge baseline, 100% at the
// open/closed threshold.  Readings above the threshold exceed 100%.
float leadConfidencePct() {
  float span = cfg.clsdThres - cfg.bridgeBaseV;
  float pct  = (span > 0.0f)
               ? ((fabsf(bridgeAvg) - cfg.bridgeBaseV) / span) * 100.0f
               : 0.0f;
  return (pct < 0.0f) ? 0.0f : pct;
}

// Maps a bridge voltage to an estimated resistance bin by nearest-neighbour
// against the measured calibration points.  Below the first point reads
// "<33K"; above the last reads ">10M".
const char* bridgeResistanceLabel(float v) {
  v = fabsf(v);
  if (v < 1.34f)  return "<33K";
  if (v > 1.706f) return ">10M";

/*Red board
  static const float kBinV[]    = { 1.273f, 1.288f, 1.339f, 1.431f, 1.578f, 1.633f };
  static const char* kBinName[] = { "~33K", "~100K", "~330K", "~1M", "~10M", "~50M" };
*/
  static const float kBinV[]    = { 1.35f, 1.37f, 1.425f, 1.521f, 1.679f };
  static const char* kBinName[] = { "~33K", "~100K", "~330K", "~1M", "~10M" };

  const int n = sizeof(kBinV) / sizeof(kBinV[0]);
  int   best     = 0;
  float bestDiff = fabsf(v - kBinV[0]);
  for (int i = 1; i < n; i++) {
    float d = fabsf(v - kBinV[i]);
    if (d < bestDiff) { bestDiff = d; best = i; }
  }
  return kBinName[best];
}

// Switch the bridge on, take one differential sample, switch it off, and fold
// the magnitude into the running average.  Only ever called with the leads
// confirmed dead.
void leadClosedOrFloat() {
  digitalWrite(VBRIDGE_PIN, BRIDGE_ON);
  ads.setDataRate(ADS_RATE_FAST);
  ads.setGain(GAIN_TWO);
  bridgeV = ads.readADC_Differential_0_1() * (GAIN_FACTOR_2 / 1000.0f);
  digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);
  ads.setDataRate(ADS_RATE_MID);

  bridgeAvg += (fabsf(bridgeV) - bridgeAvg) / 10.0f;

  if (fabsf(bridgeV) < cfg.clsdThres) {
    // Two consecutive samples agree before the confidence moves, so a single
    // noisy conversion cannot flip the classification.
    closedTrig = true;
    floatTrig  = false;
    closedConf = min(closedConf + 1.0f, 10.0f);
  } else if (fabsf(bridgeV) > cfg.clsdThres) {
    floatTrig  = true;
    closedTrig = false;
    closedConf = max(closedConf - 1.0f, 0.0f);
  }
  // exactly equal: hold the previous classification

  delay(1);   // let the bridge settle before the next voltage read
}

// One lead measurement pass: auto-range, convert, decide dead-or-live, and run
// the bridge test when the leads are dead.
void leadMeasure() {
  uint32_t now = millis();

  if (mosfetHold >= 0) {
    // Manual bridge hold for diagnostics: park the MOSFET, skip the test.
    // The interlock still wins -- a hold set while the relay was open must not
    // re-assert the bridge the moment the relay closes.
    digitalWrite(VBRIDGE_PIN,
                 (mosfetHold && leadBridgeAllowed()) ? BRIDGE_ON : BRIDGE_OFF);
    return;
  }

  digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);   // bridge disconnected while measuring

  if (leadFirstRun) {
    gainIndexVolt = kNumGainLevels - 1;    // start at the highest gain
    leadFirstRun = false;
    ads.setDataRate(cfg.acMode ? ADS_RATE_FAST : ADS_RATE_MID);
  }

  // Volt-only mode has no bridge test to keep up with, so trade conversion
  // rate for resolution.  AC always needs the fast rate to resolve a cycle.
  static bool prevCfSuppress = false;
  if (prevCfSuppress != cfSuppress && !cfg.acMode) {
    ads.setDataRate(cfSuppress ? ADS_RATE_SLOW : ADS_RATE_MID);
    prevCfSuppress = cfSuppress;
  }

  ads.setGain(kGainLevels[gainIndexVolt]);
  leadCount = ads.readADC_Differential_0_1();

  if (abs(leadCount) > ADC_COUNT_HIGH_THRESH && gainIndexVolt > 0) {
    --gainIndexVolt;
    ads.setGain(kGainLevels[gainIndexVolt]);
    leadCount = ads.readADC_Differential_0_1();
  } else if (abs(leadCount) < ADC_COUNT_LOW_THRESH &&
             gainIndexVolt < (size_t)(kNumGainLevels - 1)) {
    ++gainIndexVolt;
    ads.setGain(kGainLevels[gainIndexVolt]);
    leadCount = ads.readADC_Differential_0_1();
  }

  leadActualV = leadCount * kGainFactors[gainIndexVolt] / 1000.0f;
  leadRaw     = leadActualV * cfg.vScaleFull;
  leadMedian += (leadRaw - leadMedian) / 10.0f;

  if (cfg.acMode) {
    // ── AC: accumulate VRMS over a short window ──
    acSumSq += leadRaw * leadRaw;
    if (++acSampleCount >= AC_VRMS_SAMPLES) {
      leadV         = sqrtf(acSumSq / (float)acSampleCount);
      acSumSq       = 0.0f;
      acSampleCount = 0;
    }

    // The instantaneous reading must stay below CORFTRIG for a full quarter
    // cycle before the bridge test is allowed -- a real mains zero crossing
    // passes the threshold in microseconds and can never qualify.
    bool zeroCond = (fabsf(leadRaw) < cfg.corFTrig);
    if (voltOverride == VOLT_DISABLED) zeroCond = true;
    if (voltOverride == VOLT_FORCE_ON) zeroCond = false;
    if (!cfSuppress && zeroCond) {
      if (!acBelowActive) { acBelowActive = true; acBelowStart = now; }
      if (now - acBelowStart >= AC_ZERO_DEBOUNCE_MS) {
        leadZero = true;
        if (leadBridgeAllowed()) leadClosedOrFloat();
      } else {
        leadZero = false;
      }
    } else {
      acBelowActive = false;
      leadZero = false;
    }

  } else {
    // ── DC ──
    leadV = cfSuppress ? leadRaw : leadMedian;

    bool zeroCond = (fabsf(leadMedian) < cfg.corFTrig);
    if (voltOverride == VOLT_DISABLED) zeroCond = true;
    if (voltOverride == VOLT_FORCE_ON) zeroCond = false;
    if (zeroCond && !cfSuppress) {
      leadZero = true;
      if (leadBridgeAllowed()) leadClosedOrFloat();
    } else {
      leadZero = false;
    }
  }
}

// Classification -> two-bit code.  Suppression, a missing ADC and "dead leads
// but the bridge test is not allowed" all report NONE: there is no lead result,
// and saying FLOAT would be a lie.
//
// Note VOLTAGE is reported from a MEASUREMENT, never assumed from the relay
// being closed: a closed relay with no voltage at the leads is a real fault
// (blown fuse, welded-open contact) and must not be painted as normal.
uint8_t leadClassify() {
  if (!cfg.leadEn || !leadAdcOk) return LEAD_NONE;
  if (cfSuppress)                return LEAD_NONE;
  if (!leadZero)                 return LEAD_VOLTAGE;
  if (!leadBridgeAllowed())      return LEAD_NONE;
  return (closedConf > 5.0f) ? LEAD_CLOSED : LEAD_FLOAT;
}

void announceLead() {
  say("$LEAD,%d,%s,%.3f,%.4f,%.4f,%s,%d,%d,%d,%d\n",
      leadCode, LEAD_NAMES[leadCode & 0x3], leadV, bridgeV, bridgeAvg,
      bridgeResistanceLabel(bridgeAvg), (int)leadConfidencePct(),
      leadZero ? 1 : 0, cfg.acMode ? 1 : 0, cfSuppress ? 1 : 0);
}

void leadTask() {
  if (!leadActive()) {
    // Disabled or no ADC: park the bridge and keep the status pins truthful.
    digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);
    writeLeadOutputs(leadClassify());
    if (leadCode != leadPrev) { leadPrev = leadCode; announceLead(); }
    return;
  }

  uint32_t now = millis();
  if (now - lastLeadMs >= cfg.leadMs) {
    lastLeadMs = now;
    leadMeasure();
    writeLeadOutputs(leadClassify());
    if (leadCode != leadPrev) { leadPrev = leadCode; announceLead(); }
  }

  if (leadStream && (now - lastLeadStreamMs >= leadStreamMs)) {
    lastLeadStreamMs = now;
    ads.setGain(GAIN_TWO);                 // fixed gain -> constant host scaling
    int16_t rawDif = ads.readADC_Differential_0_1();
    say("$LD,%lu,%d,%.4f\n", (unsigned long)now, rawDif,
        rawDif * GAIN_FACTOR_2 / 1000.0f);
  }
}

// Capture the ADS differential across a bridge toggle: hold the bridge OFF,
// sample a baseline, engage it at LCAP_PRE_US, keep sampling, restore OFF.
// Fixed gain so the host's count -> volts conversion is constant.
void leadCapture(uint32_t durMs) {
  if (!cfg.leadEn || !leadAdcOk) { say("$ERR,lcap,lead ADC not available\n"); return; }
  if (relayOn && !cfg.leadHot) {
    say("$ERR,lcap,relay is closed -- !OFF first (or LEADHOT=1)\n");
    return;
  }
  if (durMs < 1)    durMs = 5;
  if (durMs > 1000) durMs = 1000;

  ads.setGain(GAIN_TWO);
  ads.setDataRate(ADS_RATE_FAST);
  digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);
  delay(2);

  // ONE continuous conversion stream: each loop pass is a single
  // conversion-register read instead of the single-shot start/poll/read
  // sequence, which is what makes the sample step short enough to be useful.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);

  int n = 0;
  uint32_t durUs = durMs * 1000UL;
  uint32_t toggleUs = 0;
  bool toggled = false;
  uint32_t t0 = micros();
  uint32_t nextSampleUs = 0;
  // ADS1015 @ 3300 SPS converts every ~303 us; pace the loop to that so every
  // stored sample is a fresh conversion rather than a re-read of the last one.
  const uint32_t sampleStepUs = 305;

  while (n < LCAP_MAX) {
    uint32_t t = micros() - t0;
    if (!toggled && t >= LCAP_PRE_US) {
      digitalWrite(VBRIDGE_PIN, BRIDGE_ON);
      toggleUs = micros() - t0;
      toggled = true;
    }
    if (t >= durUs) break;
    if (t < nextSampleUs) continue;
    nextSampleUs = t + sampleStepUs;
    lcapT[n]    = t;
    lcapDiff[n] = ads.getLastConversionResults();
    n++;
  }

  digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);
  ads.setDataRate(ADS_RATE_MID);

  say("$LCAPSTART,%d,%lu,%lu,%d,%.6f\n", n, (unsigned long)toggleUs,
      (unsigned long)durMs, 1000, GAIN_FACTOR_2);
  for (int i = 0; i < n; i++) {
    char row[48];
    int len = snprintf(row, sizeof(row), "$LCAP,%lu,%d\n",
                       (unsigned long)lcapT[i], lcapDiff[i]);
    if (len <= 0) continue;
    if (!writeLine(row, (size_t)len, 2000)) {
      say("$ERR,lcap,host stopped draining at row %d of %d\n", i, n);
      break;
    }
  }
  say("$LCAPEND\n");
  leadFirstRun = true;     // re-acquire the gain after the fixed-gain capture
}

// ══════════════════════════════════════════════════════════════════
//  RELAY
// ══════════════════════════════════════════════════════════════════
void setRelay(bool on, const char* why) {
  if (on && faultLatched) {
    say("$ERR,relay,fault latched -- !CLEAR first\n");
    return;
  }
  if (on) {
    // Never energise the leads with the bridge still connected across them.
    digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);
  }
  if (on == relayOn) {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
    return;
  }
  relayOn = on;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  if (on) {
    relayOnMs = millis();
    lowCurSinceMs = relayOnMs;
    resetStats();
  } else {
    runUntilMs = 0;
    // Each run must re-earn host-timeout protection: a plain serial terminal
    // that never pings then cannot be cut off unexpectedly.
    pingSeen = false;
    leadFirstRun = true;      // the lead ADC re-ranges from scratch
  }
  // a relay change invalidates the debounce in flight
  pendingState = state;
  pendingCount = 0;
  say("$MSG,relay %s (%s)\n", on ? "CLOSED" : "OPEN", why ? why : "");
}

// ══════════════════════════════════════════════════════════════════
//  LED
// ══════════════════════════════════════════════════════════════════
// The motor state owns the pixel; when the relay is open and the motor state
// has nothing to say (IDLE / READY), the lead result takes over.  Never blinks
// with delay() -- this runs inside the control loop.
void setLed(uint8_t r, uint8_t g, uint8_t b) {
  // Only touch the hardware on a change: updateLed() runs every loop and a
  // NeoPixel refresh masks interrupts for tens of microseconds each time.
  static int last = -1;
  int want = (r << 16) | (g << 8) | b;
  if (want == last) return;
  last = want;
  whPixel.setPixelColor(0, whPixel.Color(r, g, b));
  whPixel.show();
}

void updateLed() {
  if (!cfg.ledOn) { setLed(0, 0, 0); return; }
  switch (state) {
    case ST_RUNNING:                       setLed(0,  40, 0);   return;
    case ST_STARTING:                      setLed(0,  40, 40);  return;
    case ST_STALL:
    case ST_OVERCURRENT:
    case ST_BACKFEED:
    case ST_SENSEFAULT:
    case ST_NO_VOLTAGE:                    setLed(40, 0,  0);   return;
    case ST_NO_MOTOR:
    case ST_NO_CURRENT:                    setLed(40, 0,  40);  return;
    default: break;                        // IDLE / READY / UNKNOWN
  }
  // Relay open and the motor diagnosis is quiet: show the leads instead.
  switch (leadCode) {
    case LEAD_CLOSED:  setLed(0,  30, 0);  break;   // green: continuity
    case LEAD_FLOAT:   setLed(30, 12, 0);  break;   // amber: open
    case LEAD_VOLTAGE: setLed(30, 30, 0);  break;   // yellow: live leads
    default:
      if (state == ST_READY) setLed(0, 0, 30);      // blue: motor present
      else                   setLed(0, 0, 0);
      break;
  }
}

// ══════════════════════════════════════════════════════════════════
//  DIAGNOSIS
// ══════════════════════════════════════════════════════════════════
State classify() {
  if (faultLatched) return ST_OVERCURRENT;

  uint32_t now = millis();
  bool flowing = currentFlowing();   // lowCurSinceMs is advanced in updateState()

  if (!relayOn) {
    if (cfg.useIo) {
      switch (ioCode) {
        case 3:  return ST_BACKFEED;
        case 2:  return ST_READY;
        case 1:  return ST_NO_MOTOR;
        default: return ST_UNKNOWN;
      }
    }
    if (flowing) return ST_BACKFEED;
    // With no sense bits wired, the lead detector is the only thing that knows
    // whether a load is actually attached -- so use it rather than reporting a
    // flat IDLE.  Voltage on open leads is a backfeed, exactly as case 3 above.
    if (cfg.leadEn && leadAdcOk && !cfSuppress) {
      if (leadCode == LEAD_VOLTAGE) return ST_BACKFEED;
      if (leadCode == LEAD_CLOSED)  return ST_READY;
      if (leadCode == LEAD_FLOAT)   return ST_NO_MOTOR;
    }
    return ST_IDLE;
  }

  bool grace   = (now - relayOnMs) < cfg.inrushMs;
  bool stalled = (now - lowCurSinceMs) >= cfg.stallMs;

  if (cfg.useIo) {
    switch (ioCode) {
      case 1: return ST_NO_MOTOR;                       // open winding
      case 0: return ST_UNKNOWN;
      case 2: return flowing ? ST_SENSEFAULT : ST_NO_VOLTAGE;
      default: break;                                   // case 3: energised
    }
    if (flowing) return ST_RUNNING;
    if (grace)   return ST_STARTING;
    return stalled ? ST_STALL : ST_STARTING;
  }

  if (flowing) return ST_RUNNING;
  if (grace)   return ST_STARTING;
  return stalled ? ST_NO_CURRENT : ST_STARTING;
}

void announceState() {
  say("$STATE,%d,%s,%s\n", (int)state, STATE_NAMES[state], stateDetail(state));
}

void updateState() {
  if (currentFlowing()) lowCurSinceMs = millis();

  State want = classify();
  if (want == state) { pendingCount = 0; pendingState = state; return; }

  uint8_t need = cfg.stableN ? cfg.stableN : 1;
  // faults latch instantly; everything else has to hold for STABLEN passes
  if (want == ST_OVERCURRENT) need = 1;

  if (want != pendingState) { pendingState = want; pendingCount = 1; }
  else                      { pendingCount++; }

  if (pendingCount >= need) {
    state = want;
    pendingCount = 0;
    announceState();
  }
}

// ══════════════════════════════════════════════════════════════════
//  SAFETY
// ══════════════════════════════════════════════════════════════════
void safetyChecks() {
  if (!relayOn) return;
  uint32_t now = millis();

  if (cfg.ocTripMa > 0 && fabsf(curMa) >= cfg.ocTripMa) {
    faultLatched = true;
    setRelay(false, "over-current trip");
    state = ST_OVERCURRENT;
    pendingCount = 0;
    pendingState = state;
    say("$ERR,overcurrent,%.1f mA >= %.1f mA\n", curMa, cfg.ocTripMa);
    announceState();
    return;
  }

  if (runUntilMs && (int32_t)(now - runUntilMs) >= 0) {
    setRelay(false, "timed run complete");
    return;
  }

  if (cfg.maxRunMs && (now - relayOnMs) >= cfg.maxRunMs) {
    setRelay(false, "MAXRUNMS reached");
    say("$MSG,max run time %lu ms reached\n", (unsigned long)cfg.maxRunMs);
    return;
  }

  // Host-dead cutoff.  Only armed once the host has sent a !PING this run --
  // the GUI pings continuously while energised, a hand-typed terminal never
  // does and so is never cut off mid-test.  This is what catches "the USB link
  // dropped while the motor was running".
  if (cfg.hostToMs && pingSeen && (now - lastCmdMs) >= cfg.hostToMs) {
    setRelay(false, "host timeout");
    say("$ERR,hosttimeout,%lu ms -- host stopped pinging\n",
        (unsigned long)cfg.hostToMs);
  }
}

// ══════════════════════════════════════════════════════════════════
//  REPORTING
// ══════════════════════════════════════════════════════════════════
void streamRow() {
  // seq counts every row the board WANTED to send, whether or not it fitted.
  // A gap in seq at the host therefore means "not delivered", and txSkips says
  // which side dropped it.
  streamSeq++;

  char row[288];
  uint32_t runMs = relayOn ? (millis() - relayOnMs) : 0;
  int len = snprintf(row, sizeof(row),
                "$WH,%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%d,%s,%lu,"
                "%lu,%lu,%lu,%d,%s,%.3f,%.4f\n",
                (unsigned long)millis(), relayOn ? 1 : 0,
                curMa, statAvg(), statMin, statMax, statPeak, statRipple(),
                curMv, senseLo, senseHi, ioCode, (int)state, STATE_NAMES[state],
                (unsigned long)runMs,
                (unsigned long)streamSeq, (unsigned long)maxLoopUs,
                (unsigned long)txSkips,
                leadCode, LEAD_NAMES[leadCode & 0x3], leadV, bridgeAvg);
  if (len <= 0) return;
  if (len > (int)sizeof(row) - 1) len = (int)sizeof(row) - 1;

  // A row is bigger than the CDC FIFO, so it always takes several chunks.
  // Bound how long the control loop may spend feeding them: if the host is not
  // keeping up, abandon this row rather than delay the safety checks -- and
  // count it, so the host can tell who dropped what.
  if (!writeLine(row, (size_t)len, 20)) { txSkips++; return; }
  maxLoopUs = 0;
}

void printStatus() {
  say("$STATUS,relay=%d,state=%s,mA=%.2f,mV=%.1f,io=%d%d,useio=%d,"
      "stream=%d,rate=%d,zero=%.1f,mvpera=%.2f,thresh=%.1f,"
      "fault=%d,run=%lu,seq=%lu,txskips=%lu,maxloopus=%lu,txroom=%d,"
      "txmiss=%lu,txnohost=%lu,usbdrops=%u,"
      "lead=%s,leadadc=%d,leadv=%.3f,bridge=%.4f,cfsup=%d,ac=%d\n",
      relayOn ? 1 : 0, STATE_NAMES[state], curMa, curMv, senseHi, senseLo,
      cfg.useIo, streaming ? 1 : 0, cfg.rateMs, cfg.zeroMv,
      cfg.mvPerAmp, cfg.curThreshMa, faultLatched ? 1 : 0,
      (unsigned long)(relayOn ? millis() - relayOnMs : 0),
      (unsigned long)streamSeq, (unsigned long)txSkips,
      (unsigned long)maxLoopUs, Serial.availableForWrite(),
      (unsigned long)txDeadlineMisses, (unsigned long)txNoHost,
      linkDropCount,
      LEAD_NAMES[leadCode & 0x3], leadAdcOk ? 1 : 0, leadV, bridgeAvg,
      cfSuppress ? 1 : 0, cfg.acMode ? 1 : 0);
}

void printKey(const KeyDef& k) {
  switch (k.type) {
    case K_F:   say("$CFG,%s,%.6g\n", k.name, *(float*)k.ptr); break;
    case K_U8:  say("$CFG,%s,%u\n", k.name, (unsigned)*(uint8_t*)k.ptr); break;
    case K_U16: say("$CFG,%s,%u\n", k.name, (unsigned)*(uint16_t*)k.ptr); break;
    case K_U32: say("$CFG,%s,%lu\n", k.name, (unsigned long)*(uint32_t*)k.ptr); break;
    case K_STR: say("$CFG,%s,%s\n", k.name, (const char*)k.ptr); break;
  }
}

const KeyDef* findKey(const char* name) {
  for (int i = 0; i < N_KEYS; i++)
    if (strcasecmp(name, KEYS[i].name) == 0) return &KEYS[i];
  return nullptr;
}

bool setKey(const KeyDef& k, const char* value) {
  char* end = nullptr;
  switch (k.type) {
    case K_F: {
      float v = strtof(value, &end);
      if (end == value) return false;
      *(float*)k.ptr = v;
      break;
    }
    case K_U8: case K_U16: case K_U32: {
      long v = strtol(value, &end, 10);
      if (end == value || v < 0) return false;
      if (k.type == K_U8)       *(uint8_t*)k.ptr  = (uint8_t)(v > 255 ? 255 : v);
      else if (k.type == K_U16) *(uint16_t*)k.ptr = (uint16_t)(v > 65535 ? 65535 : v);
      else                      *(uint32_t*)k.ptr = (uint32_t)v;
      break;
    }
    case K_STR:
      strncpy((char*)k.ptr, value, 15);
      ((char*)k.ptr)[15] = '\0';
      break;
  }
  return true;
}

// ══════════════════════════════════════════════════════════════════
//  INRUSH CAPTURE  (current sense, across a relay close)
// ══════════════════════════════════════════════════════════════════
void runCapture(uint32_t durMs) {
  if (faultLatched) { say("$ERR,cap,fault latched\n"); return; }
  if (durMs < 5)    durMs = cfg.capMs ? cfg.capMs : 400;
  if (durMs > 5000) durMs = 5000;

  bool wasOn = relayOn;
  uint32_t preMs = cfg.capPreMs;
  if (preMs >= durMs) preMs = durMs / 4;

  // Start from a known-open relay so the capture always spans a real close.
  if (wasOn) {
    setRelay(false, "capture setup");
    delay(50);
  }
  digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);   // never bridge the leads we energise

  uint32_t t0 = micros();
  uint32_t preUs = preMs * 1000UL;
  uint32_t durUs = durMs * 1000UL;
  uint32_t trigUs = 0;
  int n = 0;
  bool fired = false;

  while (n < CAP_MAX) {
    uint32_t dt = micros() - t0;
    if (dt >= durUs) break;
    if (!fired && dt >= preUs) {
      digitalWrite(RELAY_PIN, HIGH);
      relayOn = true;
      relayOnMs = millis();
      lowCurSinceMs = relayOnMs;
      resetStats();
      trigUs = micros() - t0;
      fired = true;
    }
    capUs[n]  = dt;
    capRaw[n] = (uint16_t)analogRead(CURRENT_PIN);
    int a = digitalRead(SENSE_LO);
    int b = digitalRead(SENSE_HI);
    if (cfg.ioInvert) { a = !a; b = !b; }
    capIo[n]  = (uint8_t)(((b ? 1 : 0) << 1) | (a ? 1 : 0));
    n++;
  }
  if (!fired) {   // duration was shorter than the pre-trigger; close anyway
    digitalWrite(RELAY_PIN, HIGH);
    relayOn = true;
    relayOnMs = millis();
    trigUs = micros() - t0;
  }

  if (cfg.capRestore && !wasOn) setRelay(false, "capture complete");

  // The dump is far more than the CDC transmit buffer holds and writes are
  // deadline-bounded, so an unthrottled blast would be silently TRUNCATED.
  // Pace it against availableForWrite() instead, and say so plainly if the host
  // stops reading rather than emitting a short file.
  say("$CAPSTART,%d,%lu,%lu,%d,%u,%.1f,%.2f\n", n,
      (unsigned long)trigUs, (unsigned long)durMs, ADC_FULL_SCALE,
      (unsigned)cfg.vrefMv, cfg.zeroMv, cfg.mvPerAmp);
  for (int i = 0; i < n; i++) {
    char row[64];
    int len = snprintf(row, sizeof(row), "$CAP,%lu,%u,%u\n",
                       (unsigned long)capUs[i], (unsigned)capRaw[i],
                       (unsigned)capIo[i]);
    if (len <= 0) continue;
    if (!writeLine(row, (size_t)len, 2000)) {
      say("$ERR,cap,host stopped draining at row %d of %d\n", i, n);
      break;
    }
    // A dump of ~1600 rows takes a while; with the relay closed the safety
    // checks must keep running so it cannot outlive MAXRUNMS or hide a trip.
    if (relayOn && (i & 0x1F) == 0) {
      curMv = readMilliVolts(cfg.adcAvg);
      curMa = mvToMa(curMv);
      safetyChecks();
    }
  }
  say("$CAPEND\n");
}

// ══════════════════════════════════════════════════════════════════
//  COMMANDS
// ══════════════════════════════════════════════════════════════════
void handleCommand(char* line) {
  lastCmdMs = millis();

  // split on commas into at most 3 tokens
  char* tok[3] = { line, nullptr, nullptr };
  int nTok = 1;
  for (char* p = line; *p && nTok < 3; p++) {
    if (*p == ',') { *p = '\0'; tok[nTok++] = p + 1; }
  }
  char* cmd = tok[0];
  while (*cmd == '!') cmd++;
  for (char* p = cmd; *p; p++) *p = toupper(*p);

  if (!strcmp(cmd, "ON")) {
    faultLatched = false;
    setRelay(true, "!ON");
    say("$OK,on\n");

  } else if (!strcmp(cmd, "OFF") || !strcmp(cmd, "STOP")) {
    runUntilMs = 0;
    setRelay(false, cmd);
    say("$OK,off\n");

  } else if (!strcmp(cmd, "RELAY")) {
    if (!tok[1]) { say("$ERR,relay,need 0 or 1\n"); return; }
    bool on = atoi(tok[1]) != 0;
    if (on) faultLatched = false;
    setRelay(on, "!RELAY");
    say("$OK,relay,%d\n", on ? 1 : 0);

  } else if (!strcmp(cmd, "RUN")) {
    uint32_t ms = tok[1] ? (uint32_t)strtoul(tok[1], nullptr, 10) : 0;
    if (ms == 0) { say("$ERR,run,need ms\n"); return; }
    if (cfg.maxRunMs && ms > cfg.maxRunMs) ms = cfg.maxRunMs;
    faultLatched = false;
    setRelay(true, "!RUN");
    if (relayOn) {
      runUntilMs = millis() + ms;
      say("$OK,run,%lu\n", (unsigned long)ms);
    }

  } else if (!strcmp(cmd, "PING")) {
    // Keepalive. Arms the HOSTTO cutoff (see safetyChecks) and lets the host
    // measure a round trip, so a one-way-dead link is still detectable.
    pingSeen = true;
    say("$PONG,%lu,%d\n", (unsigned long)millis(), relayOn ? 1 : 0);

  } else if (!strcmp(cmd, "HIST")) {
    // Replay recorded samples newer than <sinceMs> so the host can fill the
    // hole a dropout left.  Oldest first, so the host can append in order.
    uint32_t since = tok[1] ? (uint32_t)strtoul(tok[1], nullptr, 10) : 0;
    int start = (histCount < HIST_MAX) ? 0 : histHead;
    int sent = 0;
    say("$HIST,%d,%lu\n", histCount, (unsigned long)since);
    for (int k = 0; k < histCount; k++) {
      const HistRec& h = hist[(start + k) % HIST_MAX];
      if (since && h.ms <= since) continue;
      char row[80];
      int len = snprintf(row, sizeof(row), "$H,%lu,%.2f,%d,%d,%d,%d\n",
                         (unsigned long)h.ms, h.mA, h.state, h.io, h.relay,
                         h.lead);
      if (len <= 0) continue;
      if (!writeLine(row, (size_t)len, 2000)) {
        say("$ERR,hist,host stopped draining after %d rows\n", sent);
        break;
      }
      sent++;
      if (relayOn && (sent & 0x1F) == 0) {
        curMv = readMilliVolts(cfg.adcAvg);
        curMa = mvToMa(curMv);
        safetyChecks();
      }
    }
    say("$HISTEND,%d\n", sent);

  } else if (!strcmp(cmd, "LINK")) {
    // Hand over the USB dropout history the board recorded while the host was
    // away.  Empty log + host-side "device disappeared" = the device never left
    // the bus, so look at the host/driver/cable, not the board.
    say("$LINK,%u,%lu,%lu,%d\n", linkDropCount, (unsigned long)txNoHost,
        (unsigned long)txDeadlineMisses, cdcUp ? 1 : 0);
    for (int k = 0; k < LINK_LOG_MAX; k++) {
      const LinkEvent& e = linkLog[(linkLogHead + k) % LINK_LOG_MAX];
      if (!e.atMs && !e.durMs) continue;
      say("$LINKEV,%d,%lu,%lu,%d,%.2f,%lu\n", k, (unsigned long)e.atMs,
          (unsigned long)e.durMs, e.relay, e.mA, (unsigned long)e.runMs);
    }
    say("$LINKEND\n");

  } else if (!strcmp(cmd, "CLEAR")) {
    faultLatched = false;
    state = ST_IDLE;
    pendingState = state;
    pendingCount = 0;
    say("$OK,clear\n");
    announceState();

  } else if (!strcmp(cmd, "RST")) {
    resetStats();
    say("$OK,rst\n");

  } else if (!strcmp(cmd, "ZERO")) {
    if (relayOn) { say("$ERR,zero,relay is closed -- !OFF first\n"); return; }
    float mv = readMilliVolts(cfg.adcAvg * 8);
    cfg.zeroMv = mv;
    resetStats();
    say("$OK,zero,%.1f\n", mv);
    printKey(*findKey("ZEROMV"));

  } else if (!strcmp(cmd, "CAL")) {
    if (!tok[1]) { say("$ERR,cal,need known mA\n"); return; }
    float knownMa = strtof(tok[1], nullptr);
    if (fabsf(knownMa) < 1.0f) { say("$ERR,cal,current too small\n"); return; }
    float mv = readMilliVolts(cfg.adcAvg * 8);
    float dMv = mv - cfg.zeroMv;
    if (fabsf(dMv) < 0.5f) { say("$ERR,cal,no deflection from zero\n"); return; }
    cfg.mvPerAmp = dMv * 1000.0f / knownMa;
    say("$OK,cal,%.2f\n", cfg.mvPerAmp);
    printKey(*findKey("MVPERA"));

  } else if (!strcmp(cmd, "CAP")) {
    uint32_t ms = tok[1] ? (uint32_t)strtoul(tok[1], nullptr, 10) : cfg.capMs;
    runCapture(ms);

  } else if (!strcmp(cmd, "STREAM")) {
    streaming = tok[1] ? (atoi(tok[1]) != 0) : !streaming;
    say("$OK,stream,%d\n", streaming ? 1 : 0);

  } else if (!strcmp(cmd, "RATE")) {
    if (!tok[1]) { say("$ERR,rate,need ms\n"); return; }
    long ms = strtol(tok[1], nullptr, 10);
    if (ms < 5) ms = 5;
    cfg.rateMs = (uint16_t)ms;
    say("$OK,rate,%u\n", cfg.rateMs);

  // ── lead detector ──
  } else if (!strcmp(cmd, "LEAD")) {
    announceLead();

  } else if (!strcmp(cmd, "CFSUP")) {
    cfSuppress = tok[1] ? (atoi(tok[1]) != 0) : !cfSuppress;
    leadFirstRun = true;
    say("$OK,cfsup,%d\n", cfSuppress ? 1 : 0);

  } else if (!strcmp(cmd, "ACMODE")) {
    cfg.acMode = tok[1] ? (atoi(tok[1]) != 0) : !cfg.acMode;
    acSumSq = 0.0f; acSampleCount = 0; acBelowActive = false;
    leadV = 0.0f;
    leadFirstRun = true;
    say("$OK,acmode,%d\n", cfg.acMode ? 1 : 0);

  } else if (!strcmp(cmd, "VMODE")) {
    int v = tok[1] ? atoi(tok[1]) : 0;
    voltOverride = (VoltOverride)constrain(v, 0, 2);
    say("$OK,vmode,%d\n", (int)voltOverride);

  } else if (!strcmp(cmd, "MOSFET")) {
    int m = tok[1] ? atoi(tok[1]) : -1;
    if (m > 0 && relayOn && !cfg.leadHot) {
      say("$ERR,mosfet,relay is closed -- !OFF first (or LEADHOT=1)\n");
      return;
    }
    mosfetHold = (m < 0) ? -1 : (m ? 1 : 0);
    say("$OK,mosfet,%d\n", mosfetHold);

  } else if (!strcmp(cmd, "LSTREAM")) {
    leadStream = tok[1] ? (atoi(tok[1]) != 0) : !leadStream;
    say("$OK,lstream,%d\n", leadStream ? 1 : 0);
    if (leadStream && !leadActive())
      say("$MSG,lead stream idle -- lead subsystem inactive "
          "(relay closed, LEADEN=0, or no lead ADC)\n");

  } else if (!strcmp(cmd, "LRATE")) {
    if (!tok[1]) { say("$ERR,lrate,need ms\n"); return; }
    long ms = strtol(tok[1], nullptr, 10);
    if (ms < 5) ms = 5;
    leadStreamMs = (uint32_t)ms;
    say("$OK,lrate,%lu\n", (unsigned long)leadStreamMs);

  } else if (!strcmp(cmd, "LCAP")) {
    uint32_t ms = tok[1] ? (uint32_t)strtoul(tok[1], nullptr, 10) : 5;
    leadCapture(ms);

  // ── config ──
  } else if (!strcmp(cmd, "SET")) {
    if (!tok[1] || !tok[2]) { say("$ERR,set,need key and value\n"); return; }
    const KeyDef* k = findKey(tok[1]);
    if (!k) { say("$ERR,set,unknown key %s\n", tok[1]); return; }
    if (!setKey(*k, tok[2])) { say("$ERR,set,bad value %s\n", tok[2]); return; }
    if (!strcasecmp(k->name, "IOMODE")) applyIoMode();
    if (!strcasecmp(k->name, "LED"))    updateLed();
    if (!strcasecmp(k->name, "ACMODE") || !strcasecmp(k->name, "VSCALE"))
      leadFirstRun = true;
    printKey(*k);
    say("$OK,set,%s\n", k->name);

  } else if (!strcmp(cmd, "GET")) {
    if (!tok[1]) { say("$ERR,get,need key\n"); return; }
    const KeyDef* k = findKey(tok[1]);
    if (!k) { say("$ERR,get,unknown key %s\n", tok[1]); return; }
    printKey(*k);

  } else if (!strcmp(cmd, "CFG")) {
    for (int i = 0; i < N_KEYS; i++) printKey(KEYS[i]);
    say("$CFGEND\n");

  } else if (!strcmp(cmd, "SAVE")) {
    if (saveConfig()) say("$OK,save\n");
    else              say("$ERR,save,commit failed\n");

  } else if (!strcmp(cmd, "LOAD")) {
    if (loadConfig()) { applyIoMode(); leadFirstRun = true; say("$OK,load\n"); }
    else              say("$ERR,load,no valid config in EEPROM\n");

  } else if (!strcmp(cmd, "DEFAULTS")) {
    loadDefaults(true);
    applyIoMode();
    leadFirstRun = true;
    say("$OK,defaults\n");

  } else if (!strcmp(cmd, "SN")) {
    if (tok[1] && *tok[1]) {
      strncpy(cfg.sn, tok[1], sizeof(cfg.sn));
      cfg.sn[sizeof(cfg.sn) - 1] = '\0';
      if (saveConfig()) say("$OK,sn\n");
      else              say("$ERR,sn,commit failed\n");
    }
    say("$SN,%s\n", cfg.sn);

  } else if (!strcmp(cmd, "STATUS") || !strcmp(cmd, "?")) {
    printStatus();
    announceState();
    announceLead();

  } else {
    say("$ERR,cmd,%s\n", cmd);
  }
}

void pollSerial() {
  static char buf[96];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      if (len) handleCommand(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;   // overlong line: drop it rather than half-execute
    }
  }
}

// ══════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ══════════════════════════════════════════════════════════════════
void setup() {
  // Relay LOW and bridge OFF before their pinModes: a reset or reflash must
  // never pulse the motor or drop the bridge across live leads.
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);
  pinMode(VBRIDGE_PIN, OUTPUT);
  digitalWrite(VBRIDGE_PIN, BRIDGE_OFF);

  pinMode(LEADHI_PIN, OUTPUT);
  pinMode(LEADLO_PIN, OUTPUT);
  writeLeadOutputs(LEAD_NONE);       // 00 until the first classification

  whPixel.begin();
  whPixel.clear();
  whPixel.show();

  Serial.begin(115200);

  EEPROM.begin(sizeof(Config) + 16);
  loadDefaults(false);
  bool loaded = loadConfig();
  if (!loaded) { loadDefaults(false); saveConfig(); }

  analogReadResolution(ADC_BITS);
  applyIoMode();

  // ── Lead ADC ──
  // Bounded retry, NOT the original while(!ads.begin()) spin: a missing or
  // unpowered lead board must not stop the motor tester from working.
  // Each ADS read is several register transactions, so the bus clock dominates
  // throughput -- try 1 MHz first, then fall back to 400 kHz if the wiring
  // will not take it, and report which one won.
  Wire.begin();
  // Wire inherits Stream's 1000 ms default timeout, and its I2C reads BLOCK for
  // it.  Lead measurement now runs while the motor does, so one glitched
  // transaction on a bus sitting next to a switching inductive load could stall
  // the loop that enforces OCTRIP / MAXRUNMS / HOSTTO for a full second.  Cap it
  // well under the loop budget and let the core reset the bus on a timeout.
  Wire.setTimeout(10, true);
  for (int i = 0; i < 4 && !leadAdcOk; i++) {
    if (i == 2) leadI2cHz = 400000;          // two tries fast, then back off
    Wire.setClock(leadI2cHz);
    leadAdcOk = ads.begin(0x48, &Wire);
    if (!leadAdcOk) delay(100);
  }
  if (leadAdcOk) {
    ads.setGain(GAIN_SIXTEEN);
    ads.setDataRate(ADS_RATE_MID);
  }

  streaming = cfg.autoStream != 0;
  lastCmdMs = millis();
  lowCurSinceMs = millis();
  lastLeadMs = millis();
  resetStats();

  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);

  // $BOOT is the host's proof that the board restarted -- and that the relay
  // is therefore open and the motor de-energised.
  say("$BOOT,%s,%s,%s\n", resetReasonName(),
      loaded ? "eeprom" : "defaults", WH_BOARD);
  say("$MSG,WireHawk Feather ready, %d-bit ADC, reset reason %s (config %s)\n",
      ADC_BITS, resetReasonName(), loaded ? "from EEPROM" : "defaults written");
  if (leadAdcOk)
    say("$MSG,lead ADC %s found at 0x48, I2C %lu Hz\n",
        LEAD_ADC_NAME, (unsigned long)leadI2cHz);
  else
    say("$MSG,lead ADC NOT found at 0x48 -- lead detection disabled, "
        "motor functions unaffected\n");
  say("$SN,%s\n", cfg.sn);
  printStatus();
  announceState();
  announceLead();
  leadPrev = leadCode;      // the boot report counts as the first announcement
}

void loop() {
  uint32_t loopT0 = micros();

  pollSerial();
  pollCdcLink();

  curMv = readMilliVolts(cfg.adcAvg);
  curMa = mvToMa(curMv);
  readSenseBits();
  accumulateStats(curMa);

  // Lead work happens BEFORE the diagnosis so classify() sees this pass's lead
  // code, and before safetyChecks() so a bridge sample can never delay a trip
  // by more than its own duration.
  leadTask();

  updateState();
  safetyChecks();

  if ((millis() - lastStreamMs) >= cfg.rateMs) {
    lastStreamMs = millis();
    histRecord();                 // always, even with the host gone
    if (streaming) streamRow();
  }
  updateLed();

  // Worst-case loop time is reported in every $WH row and cleared there.  If
  // the host sees a silence but maxLoopUs stays small, the board was healthy
  // and running -- the data was lost downstream, not never produced.
  uint32_t dt = micros() - loopT0;
  if (dt > maxLoopUs) maxLoopUs = dt;
}
