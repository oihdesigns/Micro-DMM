/*
 * WireHawk.ino
 *
 * Motor drive / fault tester.
 * Targets: Arduino Nano ESP32 (ESP32-S3) and Seeed XIAO RA4M1 (Renesas RA4M1).
 * One firmware builds for both so the USB-dropout question can be A/B tested
 * with identical logic on different silicon -- see the PLATFORM section for
 * every difference (ADC width, buffer sizes, EEPROM, LED, pull-downs).
 * FQBNs: arduino:esp32:nano_nora  /  Seeeduino:renesas_uno:XIAO_RA4M1
 *
 * The board closes a relay to energise a motor, watches the current draw on a
 * bidirectional current-sense amplifier, and combines that with a two-bit
 * load-side sense input to tell the difference between "no motor connected",
 * "motor running", and "voltage present but the motor is not turning"
 * (a fault stall).
 *
 * ── Hardware ──────────────────────────────────────────────────────
 *   A0  current sense, analog.  Mid-rail = 0 A, above mid-rail = forward
 *       current.  Measured zero on the prototype is ~1.590 V (nominal 3.3/2 =
 *       1.650 V, so the sense amp sits a little low -- the real value is a
 *       config key, not a constant), and ~1.630 V = 80 mA.  That is 40 mV per
 *       80 mA => 500 mV/A (MVPERA), which is the default sensitivity.
 *       CALIBRATE BEFORE TRUSTING READINGS: !ZERO with the relay open captures
 *       the true zero, and !CAL,<mA> with a known current sets MVPERA.
 *   D2  relay control, output.  HIGH = relay closed = motor energised.
 *   D3  load-side sense bit, LOW digit of the two-bit code.
 *   D4  load-side sense bit, HIGH digit of the two-bit code.
 *
 * ── The D4/D3 sense code ("XY", X = D4, Y = D3) ───────────────────
 *   10  motor present   -- winding continuity seen, no voltage on the load
 *   01  no motor        -- open circuit / nothing connected
 *   11  voltage present -- the load is energised
 *   00  unknown         -- sense circuit unpowered, shorted, or NOT WIRED YET
 *
 * D3/D4 are not connected on the current prototype, so USEIO defaults to 0:
 * the state machine runs on current alone and the raw bits are still reported
 * so the wiring can be checked.  Set USEIO=1 (and !SAVE) once they are hooked
 * up to enable the full diagnosis table below.
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
 * with no current reads NO_CURRENT rather than STALL (the firmware cannot
 * confirm that voltage was actually delivered).
 *
 * State changes are debounced by STABLEN consecutive passes.  OVERCURRENT is
 * a latched fault: the relay drops immediately and stays down until !CLEAR,
 * !OFF, or a fresh !ON.
 *
 * ── Safety ────────────────────────────────────────────────────────
 * The relay is driven LOW before its pinMode is set, so a reset or a reflash
 * cannot pulse the motor.  MAXRUNMS force-opens the relay after a maximum run
 * time (default 30 s, 0 disables) and OCTRIP opens it on over-current.
 * !STOP always opens the relay and clears any timed run.
 *
 * HOSTTO (default 2000 ms) opens the relay when a host that WAS pinging goes
 * quiet -- this is the guard against the observed failure mode below.  It arms
 * only after the first !PING of a run, so a hand-typed serial terminal that
 * never pings is never cut off mid-test.
 *
 * ── Observed failure: link dies while running, dead until reset ────
 * Reported symptom: after the motor has run for several seconds the board stops
 * talking and only the reset button brings it back.  The cause is in the core,
 * not in the wiring: USBCDC::write() spins forever when the host stops draining
 * (see the SERIAL OUTPUT section), so ONE stalled write hangs the whole sketch
 * -- no telemetry, no command handling, and the relay frozen wherever it was.
 * Everything printed here therefore goes through say()/writeLine(), which never
 * hands write() more bytes than TinyUSB has room for and gives up on a deadline
 * instead of spinning.  HOSTTO additionally opens the relay by itself if a
 * pinging host goes quiet, so a link failure cannot leave the motor turning.
 * If $BOOT ever reports BROWNOUT/PANIC then the relay and motor ARE also
 * crashing the board's supply, which is a wiring fix on top: coil flyback
 * diode, snubber across the contacts, separate relay/motor supply.
 *
 * ── Measured on hardware: the device LEAVES THE USB BUS ────────────
 * Evidence from a run that dropped several seconds after the motor started:
 *   txmiss=0, txskips=318  -> every dropped row took the "!Serial" path, i.e.
 *                             tud_cdc_n_connected() was FALSE.  Not backpressure,
 *                             not a stalled write, no slow host: no host at all.
 *   seq kept counting, maxloopus=2006 (2 ms), no $BOOT
 *                          -> the MCU never reset and never stalled.
 *   host side: ClearCommError -> PermissionError(13) -- Windows' signature for
 *                             a surprise-removed USB device.
 * So the MCU keeps running perfectly while its USB device disappears from the
 * bus and re-enumerates.  That is electrical, not a data-rate or firmware
 * problem (~1.8 kB/s, and writes never once stalled).
 *
 * The relay and motor run from a separate 24 V rail through a driver IC -- USB
 * never powers the load -- so VBUS loading is NOT the cause.  What remains is
 * the SHARED GROUND.  Motor return current (and brush commutation noise) flows
 * in the ground the board shares with the 24 V supply, while the USB cable's
 * ground ties the board to the PC: that is a ground loop.  Any IR drop or
 * L*di/dt along the shared return appears as a common-mode shift between board
 * ground and host ground, and USB signalling is only valid over a narrow
 * common-mode range.  Enough corrupted packets in a row and the host resets the
 * port -- a surprise-removal, exactly what is observed, with the MCU none the
 * wiser.  "Several seconds in" is simply how long it takes to accumulate that
 * many errors.  Ranked fixes:
 *   1. Star-ground: the board's ground must reach the 24 V supply's 0 V on its
 *      own conductor, NOT via a path that carries motor return current.
 *   2. Kill the noise at its source: 0.1 uF across the motor terminals (and
 *      terminal-to-case if it is a brushed motor), ferrite on the motor leads,
 *      leads twisted and routed away from the USB cable.
 *   3. Galvanic isolation -- an isolated USB hub or ADuM3160-class isolator
 *      breaks the loop outright and is the definitive fix.
 *   4. Confirm the coil freewheel path: on a ULN2003/2803-style driver the COM
 *      pin must be tied to +24 V or the internal clamp diodes do nothing.
 * !LINK dumps the board's own record of every dropout (see USB LINK BLACK BOX)
 * so dropouts can be correlated with load current, and !HIST backfills the
 * samples a dropout cost the host -- the data survives either way.
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
 *              Small values prove nothing in the firmware is blocking; a large
 *              value names the stall.
 * $STATUS repeats these plus txroom (Serial.availableForWrite()).
 *
 * ── Serial protocol (115200 baud, line based) ─────────────────────
 * Commands in (newline terminated):
 *   !ON / !OFF            close / open the relay
 *   !RELAY,<0|1>          same, explicit
 *   !RUN,<ms>             close the relay for <ms> then open it
 *   !STOP                 open the relay, cancel a timed run
 *   !PING                 keepalive; arms the HOSTTO cutoff, replies $PONG
 *   !LINK                 dump the board's record of every USB dropout
 *   !HIST[,<sinceMs>]     replay recorded samples newer than <sinceMs>, so the
 *                         host can backfill whatever a dropout cost it
 *   !CLEAR                clear a latched fault (OVERCURRENT)
 *   !RST                  reset the current statistics
 *   !ZERO                 capture A0 now as 0 A (relay must be open)
 *   !CAL,<mA>             set MVPERA from a known current flowing now
 *   !CAP[,<ms>]           inrush capture: sample A0 fast across relay close
 *   !STREAM[,0|1]         continuous $WH streaming on/off (bare = toggle)
 *   !RATE,<ms>            stream interval
 *   !SET,<key>,<value>    set a config value in RAM (effective immediately)
 *   !GET,<key>            report one config value
 *   !CFG                  dump every config key ($CFG rows + $CFGEND)
 *   !SAVE                 persist the RAM config to EEPROM
 *   !LOAD                 discard RAM changes, reload from EEPROM
 *   !DEFAULTS             factory defaults into RAM (then !SAVE to keep)
 *   !SN[,<value>]         read / write the unit serial number
 *   !STATUS / !?          print current status
 * Data out:
 *   $WH,<ms>,<relay>,<mA>,<avgMA>,<minMA>,<maxMA>,<pkMA>,<rippleMA>,<mV>,
 *       <d3>,<d4>,<io>,<state>,<stateName>,<runMs>,<seq>,<maxLoopUs>,<txSkips>
 *     seq/maxLoopUs/txSkips are link diagnostics -- see "Is it the board or
 *     the host?" below.
 *   $STATE,<code>,<name>,<detail>        state change (sent even when idle)
 *   $BOOT,<resetReason>,<cfgSource>,<board>   sent once on boot: the board
 *                                        restarted (so the relay is open), why,
 *                                        and which board this firmware is on
 *   $STATUS,...                          summary
 *   $CFG,<key>,<value> / $CFGEND         config dump
 *   $SN,<value>                          serial number ("" if unassigned)
 *   $PONG,<ms>,<relay>                   reply to !PING
 *   $LINK,<drops>,<txNoHost>,<txMiss>,<up>            !LINK header
 *   $LINKEV,<i>,<atMs>,<durMs>,<relay>,<mA>,<runMs>   one recorded dropout
 *   $LINKEND
 *   $HIST,<held>,<sinceMs>               !HIST header
 *   $H,<ms>,<mA>,<state>,<io>,<relay>    one recorded sample (oldest first)
 *   $HISTEND,<sent>
 *   $OK,<what> / $ERR,<what>[,detail]    acknowledge / failure
 *   $MSG,<text>                          human-readable note
 *   $CAPSTART,<n>,<trigUs>,<durMs>,<fullScale>,<vrefMv>,<zeroMv>,<mvPerA>
 *   $CAP,<t_us>,<raw>,<io>               capture rows
 *   $CAPEND
 */

#include <EEPROM.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <stdarg.h>
#include <stdio.h>
#include <strings.h>

// ══════════════════════════════════════════════════════════════════
//  PLATFORM  (Arduino Nano ESP32  or  Seeed XIAO RA4M1)
// ══════════════════════════════════════════════════════════════════
// Both boards are NATIVE USB -- the MCU is the USB device -- and, notably,
// both cores carry the SAME unbounded spin in their CDC write (see SERIAL
// OUTPUT).  Building one firmware for both makes the dropout question an A/B
// test: identical logic, identical protocol, different silicon and USB stack.
// If the dropouts follow the board it is silicon; if they follow the rig it is
// the wiring.  The pin names A0/D2/D3/D4 mean the same thing on both.
#if defined(ARDUINO_ARCH_ESP32)
  #include <esp_system.h>
  #define WH_BOARD        "nano_esp32"
  const int ADC_BITS  = 12;
  // ESP32-S3 has RAM to spare
  const int CAP_MAX   = 1600;
  const int HIST_MAX  = 1200;      // ~2 min at RATEMS=100
#elif defined(ARDUINO_ARCH_RENESAS)
  #define WH_BOARD        "xiao_ra4m1"
  const int ADC_BITS  = 14;        // RA4M1 ADC is 14-bit
  // RA4M1 has only 32 KB of SRAM, so the buffers must be far smaller: the
  // ESP32 sizes alone would be ~26 KB and would not link.
  const int CAP_MAX   = 400;
  const int HIST_MAX  = 300;       // ~30 s at RATEMS=100
#else
  #error "WireHawk targets the Arduino Nano ESP32 or the Seeed XIAO RA4M1."
#endif

const int ADC_FULL_SCALE = (1 << ADC_BITS) - 1;

// Onboard status LED: discrete RGB pins on the Nano ESP32, a single NeoPixel
// on the XIAO RA4M1.  Falls back to LED_BUILTIN if the NeoPixel library is not
// installed, so the sketch always builds.
// NOTE: the include is unconditional rather than wrapped in __has_include,
// because the Arduino builder resolves libraries by pre-scanning #include
// lines -- inside __has_include it never finds the library, so the guard
// silently disables itself.  Adafruit_NeoPixel is the same dependency
// BlinkyHawk_RA4M1 already uses on this board.
#if defined(ARDUINO_ARCH_RENESAS)
  #include <Adafruit_NeoPixel.h>
  #define WH_NEOPIXEL 1
  Adafruit_NeoPixel whPixel(1, RGB_BUILTIN, NEO_GRB + NEO_KHZ800);
#endif

// ══════════════════════════════════════════════════════════════════
//  PIN MAP
// ══════════════════════════════════════════════════════════════════
const int CURRENT_PIN = A0;   // bidirectional current sense, mid-rail = 0 A
const int RELAY_PIN   = D2;   // relay control, HIGH = closed = motor on
const int SENSE_LO    = D3;   // sense code low digit  ("Y")
const int SENSE_HI    = D4;   // sense code high digit ("X")

// ══════════════════════════════════════════════════════════════════
//  CONFIGURATION  (EEPROM backed)
// ══════════════════════════════════════════════════════════════════
const uint32_t CFG_MAGIC   = 0x57484B31UL;   // "WHK1"
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
  uint32_t hostToMs;      // HOSTTO     open relay if a PINGING host goes silent, 0 = off

  uint16_t adcAvg;        // ADCAVG     A0 samples averaged per pass
  uint16_t rateMs;        // RATEMS     $WH stream interval
  uint16_t vrefMv;        // VREF       full-scale mV when ADCMV = 0
  uint16_t capPreMs;      // CAPPRE     capture pre-trigger window
  uint16_t capMs;         // CAPMS      default total capture duration

  uint8_t  useIo;         // USEIO      1 = use D3/D4 in the diagnosis
  uint8_t  ioInvert;      // IOINVERT   1 = sense bits are active low
  uint8_t  ioMode;        // IOMODE     0 = pulldown, 1 = pullup, 2 = plain input
  uint8_t  absCur;        // ABSCUR     1 = compare |current| (either direction)
  uint8_t  autoStream;    // STREAM     1 = stream $WH from boot
  uint8_t  useCalMv;      // ADCMV      1 = analogReadMilliVolts (calibrated)
  uint8_t  stableN;       // STABLEN    passes before a state change is accepted
  uint8_t  ledOn;         // LED        1 = drive the onboard RGB LED
  uint8_t  capRestore;    // CAPREST    1 = restore the relay after a capture

  char     sn[16];        // unit serial number (preserved across !DEFAULTS)
  uint32_t crc;           // must stay last
};

Config cfg;

// ── Key table: one row per settable field ─────────────────────────
// NOTE: every type used in a function signature must be declared BEFORE the
// first function definition in the sketch -- the Arduino builder hoists its
// auto-generated prototypes to that point, so a type defined later gives
// "'KeyDef' does not name a type".
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

  cfg.useIo       = 0;         // D3/D4 not wired on the prototype
  cfg.ioInvert    = 0;
  cfg.ioMode      = 0;         // pulldown: unwired reads 00 = UNKNOWN, honestly
  cfg.absCur      = 1;
  cfg.autoStream  = 1;
  cfg.useCalMv    = 1;
  cfg.stableN     = 3;
  cfg.ledOn       = 1;
  cfg.capRestore  = 1;

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
#if defined(ARDUINO_ARCH_ESP32)
  return EEPROM.commit();
#else
  return true;      // Renesas EEPROM writes through to data flash
#endif
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
uint8_t  d3 = 0, d4 = 0, ioCode = 0;
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
//  CAPTURE BUFFER
// ══════════════════════════════════════════════════════════════════
// CAP_MAX is set per board in the PLATFORM section (RA4M1 has far less SRAM)
uint32_t  capUs[CAP_MAX];
uint16_t  capRaw[CAP_MAX];
uint8_t   capIo[CAP_MAX];

// ══════════════════════════════════════════════════════════════════
//  RESET CAUSE
// ══════════════════════════════════════════════════════════════════
// Switching an inductive load off arcs the relay contacts and dumps a wide
// EMI burst into everything nearby, so "the USB link died when I hit MOTOR
// OFF" has two very different causes.  Reporting the reset reason on boot
// tells them apart: if the host sees a $BOOT line the board really restarted
// (BROWNOUT = the coil/motor collapsed the rail, PANIC/WDT = it crashed,
// EXT/POWERON = supply glitch); if the host reconnects and NO $BOOT arrives,
// the board never reset and only the USB link dropped.
const char* resetReasonName() {
#if !defined(ARDUINO_ARCH_ESP32)
  // RA4M1 exposes its reset cause in RSTSR0/1/2, but the important signal is
  // simply that a $BOOT line appeared at all -- the board restarted.
  return "UNKNOWN";
#else
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
#endif
}

// ══════════════════════════════════════════════════════════════════
//  SERIAL OUTPUT  (bounded -- never let a stalled host hang the board)
// ══════════════════════════════════════════════════════════════════
// USBCDC::write() in the ESP32 core contains
//     while(to_send){ ... if(!space){ tud_cdc_n_write_flush(itf); continue; } ... }
// which is an UNBOUNDED SPIN: if the host stops draining while TinyUSB still
// reports the device connected, write() never returns and the sketch is hung
// until someone presses reset.  setTxTimeoutMs() does NOT help -- it only
// bounds acquiring the tx semaphore, not the write loop itself.
//
// So we never hand write() more bytes than TinyUSB currently has room for:
// chunk against availableForWrite(), which keeps write() out of the !space
// branch entirely, and give up on our own deadline instead of spinning.
// The CDC transmit FIFO is CONFIG_TINYUSB_CDC_TX_BUFSIZE = 64 bytes, which is
// SMALLER THAN ONE $WH ROW -- so gating a whole row on availableForWrite() can
// never succeed, and chunking is mandatory rather than an optimisation.
const int CDC_TX_FIFO = 64;

uint32_t txDeadlineMisses = 0;   // host WAS connected but would not drain
uint32_t txNoHost         = 0;   // CDC reported no host at all -- a bus-level drop
uint32_t wedgeSinceMs     = 0;   // when output first started failing (0 = fine)

// These two counters are the difference between "the host is slow" and "the
// device fell off the USB bus", and they must never be conflated: txNoHost
// rising with txDeadlineMisses at zero means tud_cdc_n_connected() went false,
// i.e. the device was gone from the host's point of view while this MCU kept
// running.  No amount of buffer tuning fixes that -- it is electrical.

bool writeLine(const char* s, size_t len, uint32_t deadlineMs) {
  if (!Serial) { txNoHost++; return false; }   // no host: drop, never spin
  uint32_t t0 = millis();
  size_t off = 0;
  while (off < len) {
    // Deadline first, on EVERY pass: write() can legitimately return 0 even
    // with room reported (a contended tx semaphore), and a continue that
    // skipped this check would spin forever -- the exact failure being fixed.
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
bool say(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return false;
  if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
  return writeLine(buf, (size_t)n, 20);
}

// ══════════════════════════════════════════════════════════════════
//  USB LINK BLACK BOX
// ══════════════════════════════════════════════════════════════════
// The MCU survives these dropouts (loop keeps running, seq keeps counting), so
// it can record them and hand the history over once the host is back.  Each
// entry says WHEN the CDC link went down, for HOW LONG, and what the motor was
// doing at that instant -- which is what correlates dropouts with load current.
// An empty log after a host-side "device disappeared" error is itself the
// answer: the device never left the bus and the fault is on the host side.
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
// The MCU keeps running through these dropouts, so it records every sample
// interval into RAM whether or not the row could be transmitted.  After the
// host reconnects it asks for the gap (!HIST,<sinceMs>) and stitches the
// missing samples back into the plot and the CSV.  A flaky link therefore
// costs nothing but latency -- the motor data itself is complete.
struct HistRec {
  uint32_t ms;
  float    mA;
  uint8_t  state;
  uint8_t  io;
  uint8_t  relay;
};
// HIST_MAX is set per board in the PLATFORM section
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
  histHead = (histHead + 1) % HIST_MAX;
  if (histCount < HIST_MAX) histCount++;
}

// ══════════════════════════════════════════════════════════════════
//  MEASUREMENT
// ══════════════════════════════════════════════════════════════════
float readMilliVolts(uint16_t samples) {
  if (samples < 1) samples = 1;
  double acc = 0;
  for (uint16_t i = 0; i < samples; i++) {
#if defined(ARDUINO_ARCH_ESP32)
    // ESP32 can convert with its factory ADC calibration; the RA4M1 core has
    // no equivalent, so there ADCMV is ignored and VREF does the scaling.
    if (cfg.useCalMv) acc += (double)analogReadMilliVolts(CURRENT_PIN);
    else              acc += (double)analogRead(CURRENT_PIN) * cfg.vrefMv / ADC_FULL_SCALE;
#else
    acc += (double)analogRead(CURRENT_PIN) * cfg.vrefMv / ADC_FULL_SCALE;
#endif
  }
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
  d3 = a ? 1 : 0;
  d4 = b ? 1 : 0;
  ioCode = (uint8_t)((d4 << 1) | d3);
}

void applyIoMode() {
#if defined(INPUT_PULLDOWN)
  uint8_t m = (cfg.ioMode == 1) ? INPUT_PULLUP
            : (cfg.ioMode == 2) ? INPUT
                                : INPUT_PULLDOWN;
#else
  // The RA4M1 has no internal pull-downs, so IOMODE 0 falls back to a plain
  // input.  With D3/D4 unwired the pins then float -- read them as unknown
  // rather than trusting 00, or fit external pull-downs.
  uint8_t m = (cfg.ioMode == 1) ? INPUT_PULLUP : INPUT;
#endif
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
  // min / max / average deliberately skip the inrush window so a startup
  // spike does not swamp the running numbers
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
//  RELAY
// ══════════════════════════════════════════════════════════════════
void setRelay(bool on, const char* why) {
  if (on && faultLatched) {
    say("$ERR,relay,fault latched -- !CLEAR first\n");
    return;
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
  }
  // a relay change invalidates the debounce in flight
  pendingState = state;
  pendingCount = 0;
  say("$MSG,relay %s (%s)\n", on ? "CLOSED" : "OPEN", why ? why : "");
}

// ══════════════════════════════════════════════════════════════════
//  LED
// ══════════════════════════════════════════════════════════════════
void setLed(bool r, bool g, bool b) {
  // Only touch the hardware on a change: updateLed() runs every loop, and a
  // NeoPixel refresh masks interrupts for tens of microseconds each time.
  static int last = -1;
  int want = (r ? 4 : 0) | (g ? 2 : 0) | (b ? 1 : 0);
  if (want == last) return;
  last = want;

#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
  // Nano ESP32 RGB LED is active LOW
  digitalWrite(LED_RED,   r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE,  b ? LOW : HIGH);
#elif defined(WH_NEOPIXEL)
  whPixel.setPixelColor(0, whPixel.Color(r ? 40 : 0, g ? 40 : 0, b ? 40 : 0));
  whPixel.show();
#elif defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, (r || g || b) ? HIGH : LOW);
#else
  (void)r; (void)g; (void)b;
#endif
}

void updateLed() {
  if (!cfg.ledOn) { setLed(false, false, false); return; }
  switch (state) {
    case ST_RUNNING:                       setLed(false, true,  false); break;
    case ST_STARTING:                      setLed(false, true,  true ); break;
    case ST_STALL:
    case ST_OVERCURRENT:
    case ST_BACKFEED:
    case ST_SENSEFAULT:
    case ST_NO_VOLTAGE:                    setLed(true,  false, false); break;
    case ST_NO_MOTOR:
    case ST_NO_CURRENT:                    setLed(true,  false, true ); break;
    case ST_READY:                         setLed(false, false, true ); break;
    default:                               setLed(false, false, false); break;
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
    return flowing ? ST_BACKFEED : ST_IDLE;
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
  say("$STATE,%d,%s,%s\n", (int)state, STATE_NAMES[state],
                stateDetail(state));
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
  // dropped while the motor was running": the board notices on its own instead
  // of waiting out MAXRUNMS.
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
  // which side dropped it: skips rising = the host was not draining the pipe;
  // a gap with no rise in skips = the row left the board and vanished in
  // transit (link or driver).  That is the whole point of these two fields.
  streamSeq++;

  char row[224];
  uint32_t runMs = relayOn ? (millis() - relayOnMs) : 0;
  int len = snprintf(row, sizeof(row),
                "$WH,%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%d,%s,%lu,"
                "%lu,%lu,%lu\n",
                (unsigned long)millis(), relayOn ? 1 : 0,
                curMa, statAvg(), statMin, statMax, statPeak, statRipple(),
                curMv, d3, d4, ioCode, (int)state, STATE_NAMES[state],
                (unsigned long)runMs,
                (unsigned long)streamSeq, (unsigned long)maxLoopUs,
                (unsigned long)txSkips);
  if (len <= 0) return;
  if (len > (int)sizeof(row) - 1) len = (int)sizeof(row) - 1;

  // A row is bigger than the 64-byte CDC FIFO, so it always takes several
  // chunks.  Bound how long the control loop may spend feeding them: if the
  // host is not keeping up, abandon this row rather than delay the safety
  // checks -- and count it, so the host can tell who dropped what.
  if (!writeLine(row, (size_t)len, 20)) { txSkips++; return; }
  maxLoopUs = 0;
}

void printStatus() {
  say("$STATUS,relay=%d,state=%s,mA=%.2f,mV=%.1f,io=%d%d,useio=%d,"
                "stream=%d,rate=%d,zero=%.1f,mvpera=%.2f,thresh=%.1f,"
                "fault=%d,run=%lu,seq=%lu,txskips=%lu,maxloopus=%lu,txroom=%d,"
                "txmiss=%lu,txnohost=%lu,usbdrops=%u\n",
                relayOn ? 1 : 0, STATE_NAMES[state], curMa, curMv, d4, d3,
                cfg.useIo, streaming ? 1 : 0, cfg.rateMs, cfg.zeroMv,
                cfg.mvPerAmp, cfg.curThreshMa, faultLatched ? 1 : 0,
                (unsigned long)(relayOn ? millis() - relayOnMs : 0),
                (unsigned long)streamSeq, (unsigned long)txSkips,
                (unsigned long)maxLoopUs, Serial.availableForWrite(),
                (unsigned long)txDeadlineMisses, (unsigned long)txNoHost,
                linkDropCount);
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
//  INRUSH CAPTURE
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

  // The dump is ~1600 lines / tens of kB, far more than the CDC transmit
  // buffer holds.  Serial.setTxTimeoutMs(0) makes writes non-blocking (so the
  // control loop can never stall), which means an unthrottled blast would be
  // silently TRUNCATED.  Pace it against availableForWrite() instead, and say
  // so plainly if the host stops reading rather than emitting a short file.
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
      int len = snprintf(row, sizeof(row), "$H,%lu,%.2f,%d,%d,%d\n",
                         (unsigned long)h.ms, h.mA, h.state, h.io, h.relay);
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

  } else if (!strcmp(cmd, "SET")) {
    if (!tok[1] || !tok[2]) { say("$ERR,set,need key and value\n"); return; }
    const KeyDef* k = findKey(tok[1]);
    if (!k) { say("$ERR,set,unknown key %s\n", tok[1]); return; }
    if (!setKey(*k, tok[2])) { say("$ERR,set,bad value %s\n", tok[2]); return; }
    if (!strcasecmp(k->name, "IOMODE")) applyIoMode();
    if (!strcasecmp(k->name, "LED"))    updateLed();
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
    if (loadConfig()) { applyIoMode(); say("$OK,load\n"); }
    else              say("$ERR,load,no valid config in EEPROM\n");

  } else if (!strcmp(cmd, "DEFAULTS")) {
    loadDefaults(true);
    applyIoMode();
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
  // relay LOW before pinMode: a reset or reflash must never pulse the motor
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
#elif defined(WH_NEOPIXEL)
  pinMode(PIN_RGB_EN, OUTPUT);
  digitalWrite(PIN_RGB_EN, HIGH);      // XIAO RA4M1 gates power to the pixel
  whPixel.begin();
  whPixel.clear();
  whPixel.show();
#elif defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  setLed(false, false, false);

  Serial.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
  // Small but NON-ZERO: this timeout guards the CDC tx semaphore, and with 0
  // every availableForWrite() that meets any contention returns 0 -- which
  // would make the chunked writer think there is never any room.  It does NOT
  // bound write() itself; writeLine() does that.  No equivalent on the RA4M1.
  Serial.setTxTimeoutMs(10);
#endif

#if defined(ARDUINO_ARCH_ESP32)
  EEPROM.begin(sizeof(Config) + 16);   // RA4M1 EEPROM is data-flash backed and
#endif                                 // needs no begin()/commit()
  loadDefaults(false);
  bool loaded = loadConfig();
  if (!loaded) { loadDefaults(false); saveConfig(); }

  analogReadResolution(ADC_BITS);
#if defined(ADC_11db)
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);
#endif
  applyIoMode();

  streaming = cfg.autoStream != 0;
  lastCmdMs = millis();
  lowCurSinceMs = millis();
  resetStats();

  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);

  // $BOOT is the host's proof that the board restarted -- and that the relay
  // is therefore open and the motor de-energised.
  say("$BOOT,%s,%s,%s\n", resetReasonName(),
                loaded ? "eeprom" : "defaults", WH_BOARD);
  say("$MSG,WireHawk ready on %s, %d-bit ADC, reset reason %s (config %s)\n",
                WH_BOARD, ADC_BITS, resetReasonName(),
                loaded ? "from EEPROM" : "defaults written");
  say("$SN,%s\n", cfg.sn);
  printStatus();
  announceState();
}

void loop() {
  uint32_t loopT0 = micros();

  pollSerial();
  pollCdcLink();

  curMv = readMilliVolts(cfg.adcAvg);
  curMa = mvToMa(curMv);
  readSenseBits();
  accumulateStats(curMa);

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
