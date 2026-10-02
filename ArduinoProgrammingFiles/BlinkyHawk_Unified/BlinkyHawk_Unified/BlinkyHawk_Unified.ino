/*
 * BlinkyHawk_Unified.ino
 *
 * The production Blinky Hawk firmware for EVERY board revision, merged from
 * the two sketches it replaces:
 *
 *   BlinkyHawk_RA4M1  production: detection, alerts, sleep, EEPROM config,
 *                     the v2..v8 migration chain, the host protocol.
 *   BlinkyHawk_Bench  the breadboard fork: power gates, the two-stage sleep,
 *                     DEEPPARK, CHGINHIBIT, !GATE / !EXPT / !DETLOG.
 *
 * and adds support for BlinkyHawk V3b (HWREV 4), the first PCB with the power
 * gates in hardware and a PAM8904 piezo driver with firmware volume control.
 *
 * What changed relative to each parent:
 *   - vs production: everything the bench fork learned is now here.  Config
 *     v9 appends the gate, deep-sleep, charge-inhibit and volume keys to the
 *     v8 layout, and a v8 unit migrates with its tuning intact.
 *   - vs bench: the pin map is NOT runtime config any more.  It is fixed per
 *     board revision (see applyBoardPins), because a shipped unit's wiring is
 *     a fact about its PCB, not a tunable.  The PIN* and *POL keys and the AUX
 *     gate are gone; !PINS still reports the map, read-only, in the same $PIN
 *     row format so the bench GUI and the test suite keep working.
 *   - the config lives in the PRODUCTION EEPROM block ("BHK1" at 0).  The
 *     bench block ("BHKX" at 1024) is never read or written, so flashing the
 *     bench firmware back onto a unit still finds its bench settings.
 *
 * ------------------------------------------------------------------
 * Target: Seeed XIAO RA4M1 ONLY (Renesas RA4M1, 14-bit ADC).
 * Descended from OpenLeadDetect_XIAO_Minimal with the multi-MCU support and
 * the A5 potentiometer / adaptive-threshold features removed.
 *
 * "Pseudo-differential": analogRead() has no native differential mode, so
 * SENSE_POS and SENSE_NEG are each sampled vs. GND and subtracted in
 * software.  On every shipping board SENSE_NEG is replaced by the fixed
 * pseudo-reference NEGV (NEGFIX=1), and on V3b it has no pin at all.
 *
 * Measurement logic (repeated continuously):
 *   1. MOSFET held HIGH (resting / bridge connected), read the differential.
 *   2. Voltage-present decision (de-noised):
 *        - any single read beyond VOLTFAST * REFBAND -> present;
 *        - otherwise average VOLTAVG reads and compare to the band.
 *      If voltage is present the open/closed test is bypassed, and the
 *      voltage is classified VDC+ / VDC- / VAC for the alert pattern.
 *   3. Test (only when no voltage):  MOSFET LOW, derive an open/closed metric,
 *      MOSFET back HIGH.  metric > active threshold -> OPEN (blue), else CLOSED
 *      (green).  How the metric is derived is selectable (cfg.detectMethod):
 *        0 SINGLE  : one differential read after settlePreUs; metric = |diff| (V)
 *        1 TIMERET : time (ms) for the differential to return within
 *                    detReturnBand of the resting centre.  LONGER = MORE OPEN.
 *        2 AREA    : tail-windowed integral (V*ms) of |diff-centre| from
 *                    detAreaStartUs to detWindowUs.  LARGER = MORE OPEN.
 *      All three keep "larger metric = more open", so the threshold table and
 *      the compare are shared -- but the threshold's UNITS change with the
 *      method (V / ms / V*ms), so THRESH00..11 must be re-tuned after a switch.
 *
 * ── Board revisions (HWREV) ───────────────────────────────────────
 * HWREV (EEPROM config) says which PCB is under the XIAO.  It is read before
 * any pin is configured because it decides what several pins physically ARE,
 * and getting it wrong is not cosmetic: anything other than 2 on a V2 board
 * drives D8 (and on V3b, D10) push-pull into a closed DIP switch's short to
 * ground.
 *
 *   HWREV 2  OpenLead_Headless V2
 *            D8/D10 = threshold-select DIP switches (inputs, PCB pull-ups)
 *            D9     = buzzer, other leg hard-wired to ground (single-ended)
 *            A1     = SENSE_NEG,  A3 = VBUS/2 charge sense
 *   HWREV 3  OpenLead_Headless V3
 *            D8/D9  = piezo BZ1 between them, driven anti-phase (SPKDIFF)
 *            D10    = no connection (parked as output low)
 *            A1     = no connection (NEGFIX must stay 1),  A3 = VBUS/2
 *            LED1 wired straight to BatteryRail; nothing can be gated.
 *   HWREV 4  BlinkyHawk V3b  (default for a board with blank EEPROM)
 *            A1     = VBUS/2 charge sense  (MOVED from A3)
 *            A2     = sense node (unchanged)
 *            A3     = Analog_Rail enable -- TPS22914 load switch, active HIGH.
 *                     Feeds the TL431 reference and so the whole front end.
 *            D6     = LED1 data,  D7 = bridge MOSFET (unchanged)
 *            D8/D9  = PAM8904 EN1/EN2 -- gain select, 00 = shutdown
 *            D10    = LEDRail enable -- TPS22914, active HIGH.  LED1 is now
 *                     powered from the gated rail.
 *            D15    = PAM8904 DIN (the tone).  D15 is a back pad on the XIAO
 *                     (P101), wired to the board's J7.
 *            A0, D4, D5 = no connection.  There is no SENSE_NEG pin at all,
 *                     so NEGFIX is forced on regardless of the key.
 *            Threshold select is THRESHSEL, as on V3.
 *
 * A unit carrying a config from a pre-v6 firmware is migrated with HWREV
 * pinned to 2 -- such a config can only exist on a V2.  v6..v8 knew about
 * HWREV and carry their stored value across.  Only a blank EEPROM defaults to
 * 4.  !DEFAULTS preserves HWREV.  NOTE the one case this cannot catch: a V3b
 * that was ever flashed with v8 production firmware stored HWREV 3 (v8's
 * blank default) and will keep it -- set !SET,HWREV,4 + !SAVE on such a unit.
 *
 * ── Power gates (HWREV 4 only; inert elsewhere) ───────────────────
 * Two load switches, each with a MODE and a settle time:
 *   ANA  the analog rail (A3).  ANAMODE / ANAUS.
 *   LED  LED1's rail (D10).     LEDGMODE / LEDGMS.
 *   MODE 0 ALWAYS  held on
 *        1 SLEEP   on while awake, dropped while asleep or in !FLOOR (the
 *                  sleeping probe still raises it around each probe)
 *        2 PULSED  off at rest.  ANA is raised only around a measurement;
 *                  LED is raised only while a colour is showing.
 * Modes 1 and 2 PERTURB THE MEASUREMENT: rail loading moves the resting
 * differential by more than REFBAND, so tune THRESH / SLEEPTHR under the
 * gating scheme the unit will ship with, and set ANAUS from a !CAP trace
 * (the capture shows the rail coming up inside the sampled window).
 * !GATE holds a gate on/off (RAM only) for current measurements.
 *
 * ── Buzzer and volume ─────────────────────────────────────────────
 *   V2   square wave on D9 via tone().
 *   V3   anti-phase on D8/D9 off a private FspTimer (SPKDIFF=1), ~+6 dB.
 *   V3b  PAM8904 charge-pump driver.  D15 carries the tone as a hardware PWM
 *        (GPT5A), D8/D9 set the gain:
 *            EN1 EN2   0 0 shutdown   0 1 1x   1 0 2x   1 1 3x
 *        CONTVOL / VOLTVOL (0-3) pick the gain per alert, 0 = that alert
 *        silent.  1x puts about what V3's anti-phase drive did across the
 *        element; 3x is ~+9.5 dB over that.  SPKDUTY (1-50 %) trims below a
 *        step: the tone's fundamental scales with sin(pi * duty), so 50 is
 *        full and 25 is ~-3 dB, 12 ~-8 dB, 5 ~-16 dB.  The amp is held in
 *        shutdown (EN1=EN2=0, < 1 uA) whenever no pulse is sounding, so
 *        volume costs nothing between beeps.  !TONE plays one test pulse
 *        regardless of charge lockout / mute, for setting the volume by ear.
 *        PASSIVE and SPKDIFF do not apply to V3b.
 *
 * ── Threshold select ──────────────────────────────────────────────
 * Four thresholds live in EEPROM (THRESH00/01/10/11); one of them is active.
 *   HWREV 2  the DIP switches pick it (raw pin readings "XY", X = D8, Y = D10,
 *            HIGH = switch open), re-read every detection pass.
 *   HWREV 3+ THRESHSEL (0-3) picks it, indexing the same table.
 * SLEEPTHR00..11 are the matching wake thresholds for the sleeping probe.
 *
 * ── EEPROM configuration ──────────────────────────────────────────
 * Nearly every tunable lives in a Config struct persisted to the RA4M1's
 * data-flash-backed EEPROM ("BHK1" at address 0, CFG_VERSION 9).  On boot the
 * stored config is validated (magic + version + CRC); an older layout is
 * migrated, anything unreadable is replaced with defaults.  Data flash is
 * not erased by a sketch upload, so a unit keeps its tuning across a reflash.
 * The unit serial number has its own block at 512 and survives everything.
 *
 * ── Serial protocol (115200 baud, line based) ─────────────────────
 * Commands in (each terminated with newline):
 *   !SET,<key>,<value>  set a config value in RAM (takes effect immediately)
 *   !GET,<key>          report one config value
 *   !CFG                dump every config key ($CFG rows + $CFGEND)
 *   !SAVE               persist the RAM config to EEPROM
 *   !LOAD               discard RAM changes, reload from EEPROM
 *   !DEFAULTS           factory defaults into RAM (then !SAVE to keep)
 *   !SN[,<value>]       read the unit serial number, or write it (no commas)
 *   !DIAG[,0|1]         enter/exit diagnostic mode (bare = toggle)
 *   !STREAM[,0|1]       continuous raw streaming on/off (diag only)
 *   !RATE,<ms>          stream interval in ms
 *   !VMODE,<0|1|2>      voltage mode: 0=auto  1=lock ON  2=disable
 *   !VTEST              classify the voltage on the leads now ($VTEST,...)
 *   !MOSFET,<-1|0|1>    MOSFET: -1=auto(run detection) 0=hold off 1=hold on
 *   !ALERTS[,0|1]       re-enable normal alerts while charging (1=on,0=blink)
 *   !CAP[,<ms>]         capture ADC across a MOSFET toggle, then dump
 *   !TONE[,<vol>[,<ms>[,<hz>]]]  play one test pulse now (defaults VOLTVOL,
 *                       200 ms, VOLTFREQ).  Ignores mute and charge lockout.
 *   !SLEEP[,0]          arm the low-power timeout to fire as soon as it is
 *                       allowed (arm over USB, then unplug); ,0 = disarm
 *   !SLEEPTEST          run one sleeping-mode probe now and report it
 *   !SLEEPLOG[,0]       dump the probes made while asleep (,0 = clear)
 *   !FLOOR,<0-3>        park the board for a current measurement (see below)
 *   !DEEP[,0|1]         report the two-stage sleep schedule, or (while
 *                       asleep) force stage 2 (1) / back to stage 1 (0)
 *   !PINS               report the pin map this HWREV resolves to (read-only)
 *   !GATE[,<ANA|LED>[,<0|1|-1>]]  report the gates, or hold one off/on
 *                       (-1 = follow its MODE).  RAM only; survives sleep.
 *   !EXPT[,<sec>]       run the ten-step power profile; !EXPT,0 aborts
 *   !DETLOG[,0|1]       one $DET line per detection pass (test suite)
 *   !STATUS  / !?       print current status
 * Data out:
 *   $STATUS,...                          current mode/state summary
 *   $CFG,<key>,<value> / $CFGEND         config values
 *   $SN,<value>                          unit serial number (empty if unassigned)
 *   $OK,<what> / $ERR,<what>[,detail]    command acknowledge / failure
 *   $DIP,<idx>,<thresh>                  threshold position changed (live)
 *   $VTEST,<VDC+|VDC-|VAC>,...           one voltage classification
 *   $DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>    (streaming)
 *   $CAPSTART,<n>,<toggleUs>,<durMs>,<fullScale>,<vref>,<gateUs>,<gateMask>,
 *             <gateSettleUs>  /  $CAP,<t_us>,<rawPos>,<rawNeg>  /  $CAPEND
 *   $DET,...                             one detection pass (!DETLOG)
 *   $PIN,<func>,<num|none>,<name>  $PINNAME,<name>,<num>  $PINEND   (!PINS)
 *   $GATE,<name>,pin=..,pol=..,mode=..,settleus=..,force=..,state=..
 *   $DEEP,...   $EXPTPLAN,...  $EXPT,...  $EXPTEND,<why>
 *
 * ── Alerts ────────────────────────────────────────────────────────
 * LED: dim-blue flash = floating, green flash = closed, voltage = a red-led
 * colour sequence per kind (VDC+ red-red, VDC- red-blue, VAC red-blue-green);
 * slow dim-red blink = charging (green when battery >= BATTFULLPCT); 1-4
 * green boot blinks = battery level.  Brightness / on-time / period per state
 * are keys; the hues are fixed on purpose -- the colour is the meaning.
 * Speaker mirrors the LED: continuity beep, and the voltage beep's RHYTHM
 * carries the kind (short-short / short-LONG / three shorts).  Shorting the
 * leads during boot mutes audio for the session (BOOTMUTE).
 *
 * ── Low-power timeout mode ────────────────────────────────────────
 * After SLEEPSEC seconds of open leads the board parks every load it can
 * (LED, speaker/amp, battery-sense divider, gates per MODE, optionally the
 * bridge) and enters Software Standby, woken by the RTC every SLEEPTICKMS to
 * run one cheap probe; closed leads or voltage bring it back.  After DEEPSEC
 * more seconds of open leads it drops to stage 2, re-programming the RTC to
 * DEEPHZ so it genuinely wakes less often (DEEPSEC=0 = single stage).  Sleep
 * is skipped while charging (unless CHGINHIBIT=0), in diagnostic mode, and
 * while a host holds the serial port open.
 *
 * NOTE: millis() does not advance during Standby, so the sleeping loop counts
 * ticks rather than timing, and the idle timer is re-based on wake.
 *
 * !FLOOR,<1-3> parks the board in one fixed state for a series ammeter:
 *   1 vs 3  = what Software Standby buys over a plain WFI idle
 *   1 vs 2  = the bridge leg's share (bridge resting vs disconnected)
 * !EXPT sweeps the gates awake / asleep / deep in one timed run for a PPK2.
 */

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <FspTimer.h>  // anti-phase buzzer drive (V3)
#include <pwm.h>       // PAM8904 tone on D15 (V3b)
#include <ctype.h>
#include <string.h>
#include <stddef.h>    // offsetof (configCrc)

#if !defined(ARDUINO_ARCH_RENESAS)
  #error "BlinkyHawk targets the Seeed XIAO RA4M1 -- select a Renesas RA4M1 board in Tools > Board."
#endif

// ══════════════════════════════════════════════════════════════════
//  USB SERIAL THAT CANNOT HANG THE LOOP
// ══════════════════════════════════════════════════════════════════
// The core's SerialUSB::write() checks "host connected" once, then spins until
// the CDC FIFO has room -- forever, if nobody is draining it.  Found on the
// test rig (Oct 2026): pull the cable (or cut USB on battery) while a host
// still has the port open, and the next print -- the 4 Hz debug line, or a
// reply still going out -- never returns.  Detection, alerts and sleep all
// stop until a host reopens the port and reads the backlog.
//
// Every Serial use in this sketch goes through GuardedSerial instead (the
// #define below): a write only ever hands the core as much as the FIFO has
// room for, so the core's spin is never reached.  If the FIFO stays full for
// SERIAL_TX_WAIT_MS nobody is reading -- the output is dropped and the port
// counts as STALLED, so later writes drop at once with no wait.  The first
// write that finds room again clears it.
//
// STALLED also makes `if (Serial)` false: a port that is open but not being
// drained is no host.  Without that, a cable pulled while the GUI was
// connected left DTR latched and lowPowerAllowed() held the board awake on
// battery indefinitely.
const uint32_t SERIAL_TX_WAIT_MS = 20;   // a live host drains in well under 1 ms

class GuardedSerial : public Stream {
public:
  void   begin(unsigned long baud)    { SerialUSB.begin(baud); }
  int    available() override         { return SerialUSB.available(); }
  int    read() override              { return SerialUSB.read(); }
  int    peek() override              { return SerialUSB.peek(); }
  void   flush() override             { SerialUSB.flush(); }   // non-blocking in this core
  int    availableForWrite() override { return SerialUSB.availableForWrite(); }
  size_t write(uint8_t c) override    { return write(&c, 1); }
  size_t write(const uint8_t *p, size_t n) override {
    size_t done = 0;
    while (done < n) {
      int room = SerialUSB.availableForWrite();
      if (room <= 0 && !stalled) {
        uint32_t t0 = millis();
        while ((room = SerialUSB.availableForWrite()) <= 0 &&
               millis() - t0 < SERIAL_TX_WAIT_MS) { }
      }
      if (room <= 0) { stalled = true; return n; }   // drop the rest, quietly
      stalled = false;
      size_t chunk = min((size_t)room, n - done);
      SerialUSB.write(p + done, chunk);              // fits: the core never spins
      done += chunk;
    }
    return n;
  }
  using Print::write;
  // Port open (DTR) AND being drained.
  operator bool() { return !stalled && (bool)SerialUSB; }
  bool stalled = false;
};
GuardedSerial bhSerial;
#undef  Serial
#define Serial bhSerial

// ══════════════════════════════════════════════════════════════════
//  PIN MAP / HARDWARE CONSTANTS
// ══════════════════════════════════════════════════════════════════
// These are VARIABLES because they depend on the board revision.
// applyBoardPins() is the only thing that assigns them, from cfg.hwRev, and it
// runs after the config load and after anything that can change HWREV.  The
// initialisers are the V3b wiring, but nothing drives a pin before
// applyBoardPins() has run.
#define PIN_NONE 255                     // "this function has no pin on this board"

int   SENSE_POS     = A2;                // sense node (all revisions)
int   SENSE_NEG     = PIN_NONE;          // V2: A1.  V3: A1 but NC.  V3b: none.
int   CHARGE_PIN    = A1;                // VBUS/2.  V2/V3: A3.  V3b: A1.
int   MOSFET_PIN    = D7;                // bridge MOSFET gate (HIGH = on/resting)
int   LED_PIN       = 6;                 // D6 -> LED1, the SK6812 data line
int   SPEAKER_PIN   = PIN_NONE;          // V2/V3: D9, the buzzer's driven leg
int   SPEAKER_PIN_B = PIN_NONE;          // V3: D8, the anti-phase leg
int   DIP_PIN_A     = PIN_NONE;          // V2: D8 threshold DIP ("X" in XY)
int   DIP_PIN_B     = PIN_NONE;          // V2: D10 threshold DIP ("Y" in XY)
int   PARK_PIN      = PIN_NONE;          // V3: D10, a no-connect parked low
int   AMP_DIN_PIN   = PIN_NONE;          // V3b: D15, PAM8904 DIN
int   AMP_EN1_PIN   = PIN_NONE;          // V3b: D8,  PAM8904 EN1
int   AMP_EN2_PIN   = PIN_NONE;          // V3b: D9,  PAM8904 EN2

// PIN_RGB_EN is pin 21 / P500, the power gate for the XIAO module's OWN
// onboard RGB LED -- it is NOT connected to LED1 on any board revision.  It is
// still worth switching: the onboard LED is a permanent load on the module's
// 3.3 V rail, and killing it is part of what the sleep-current numbers assume.
#define     RGB_POWER_PIN   PIN_RGB_EN
const int   BATT_PIN      = BAT_DET_PIN; // P105, onboard battery sense (Vbatt/2)
const int   BATT_EN_PIN   = BAT_READ_EN; // P400, HIGH = enable battery sense
const float BATT_DIV      = 2.0f;        // BAT_DET_PIN = Vbatt/2 -> multiply back up

const float ADC_REF_VOLTAGE = 3.3f;      // VREFH tied to the 3.3 V rail
const int   ADC_RESOLUTION  = 14;
const float ADC_FULL_SCALE  = 16383.0f;

#define MOSFET_ON   HIGH
#define MOSFET_OFF  LOW
#define SPEAKER_ON  HIGH
#define SPEAKER_OFF LOW

// Board revisions.  Every "what is this pin" decision goes through these
// predicates rather than comparing hwRev inline, so adding a revision means
// editing applyBoardPins() and this list, not hunting for ">= 3".
#define HWREV_V2   2
#define HWREV_V3   3
#define HWREV_V3B  4

// ══════════════════════════════════════════════════════════════════
//  EEPROM CONFIGURATION
// ══════════════════════════════════════════════════════════════════
// Every runtime-tunable setting lives in this struct.  It is held in RAM
// (edited by !SET, applied immediately) and persisted to EEPROM by !SAVE.
// Layout changes REQUIRE bumping CFG_VERSION and adding a migration.
#define CFG_MAGIC   0x42484B31UL   // "BHK1" -- the production block
#define CFG_VERSION 9              // v9: gates, two-stage sleep, CHGINHIBIT and
                                   //     the PAM8904 volume keys, APPENDED to v8
                                   // (v8 added the voltage-kind classifier;
                                   //  v7 the beep pulse-shape and alert LED keys;
                                   //  v6 hwRev + threshSel + spkDiff (V3);
                                   //  v5 sleepTickMs; v4 sleepThresh[];
                                   //  v3 the low-power timeout;
                                   //  v2 detectMethod + recovery params)
#define CFG_EEPROM_ADDR 0

struct Config {
  uint32_t magic;
  uint16_t version;

  // -- Board revision ----------------------------------------------
  // 2 = OpenLead_Headless V2, 3 = OpenLead_Headless V3, 4 = BlinkyHawk V3b.
  // Decides what several pins physically are -- see applyBoardPins().
  uint8_t  hwRev;

  // -- Detection ---------------------------------------------------
  float    refCenterV;     // resting differential centre (~0 V)
  float    refBandV;       // |diff - centre| within this -> no voltage, run test
  float    thresh[4];      // open/closed threshold per position [00,01,10,11]
  uint8_t  threshSel;      // hwRev 3+: which thresh[] entry is active (0-3).
                           // Ignored on hwRev 2, where the DIP pins decide.
  float    voltFastMult;   // single-read "voltage present" shortcut multiplier
  uint8_t  voltAvgSamples; // reads averaged for the voltage-present decision
  uint8_t  testAgree;      // consecutive matching MOSFET tests required
  uint8_t  stableCount;    // detection passes a new state must repeat (display debounce)
  uint16_t settlePreUs;    // us settle after MOSFET off, before the test read
  uint8_t  settlePostMs;   // ms settle after the test read, before MOSFET on
  uint8_t  negFix;         // 1 = replace the live SENSE_NEG read with negFixV
  float    negFixV;        // fixed pseudo-reference voltage when negFix

  // -- Detection method (how the open/closed metric is derived) -----
  // 0 = SINGLE  : one differential read at settlePreUs; metric = |diff|   (V)
  // 1 = TIMERET : time for |diff-refCentre| to fall back within detReturnBand (ms)
  // 2 = AREA    : tail-windowed integral of |diff-refCentre| over the recovery (V*ms)
  uint8_t  detectMethod;
  float    detReturnBand;  // method 1: |diff-refCentre| within this = "returned"
  uint16_t detWindowUs;    // methods 1&2: max sample window / timeout (us from toggle)
  uint16_t detAreaStartUs; // method 2: tail-area integration start (us from toggle)

  // -- Voltage kind (VDC+ / VDC- / VAC) ----------------------------
  // Once voltage is present, a second pass decides WHICH kind it is, so the
  // alert can carry polarity as well as presence.  SENSE_POS rests near mid-rail
  // behind nothing but two Schottky clamps, with no filter capacitor on the
  // node, so the ADC sees the instantaneous waveform and the sign of
  // (v - refCenterV) is real information.  (A future board that adds a filter
  // cap on A2 kills this feature silently -- it would average AC toward centre.)
  //   both peaks past acBandV -> the waveform crosses the centre -> VAC
  //   otherwise               -> the larger peak's side gives the polarity
  uint8_t  voltClassify;   // 1 = classify; 0 = every voltage alerts as VDC+ (legacy)
  uint16_t acWindowMs;     // classification window; must span a full mains cycle
  float    acBandV;        // per-side peak needed to call AC.  0 = voltFastMult*refBandV

  // -- Alerts ------------------------------------------------------
  uint8_t  ledEnable;      // 1 = normal detection LED alerts
  uint8_t  beepEnable;     // 1 = speaker alerts (master enable)
  uint8_t  bootMute;       // 1 = leads CLOSED at boot mutes audio for the session
  uint8_t  passiveBuzzer;  // 1 = passive buzzer via tone(), 0 = active (DC on/off).
                           // V2/V3 only -- the V3b amp always needs a tone.
  uint8_t  spkDiff;        // hwRev 3 ONLY: anti-phase D8/D9 drive (~+6 dB)
  uint16_t contFreqHz;     // continuity pitch
  uint16_t voltFreqHz;     // voltage pitch
  uint8_t  contPulses;     // pulses per continuity beep
  uint8_t  voltPulses;     // pulses per voltage beep (VDC+ and VDC-)
  uint8_t  contRepeat;     // 1 = re-beep while CLOSED holds
  uint8_t  voltRepeat;     // 1 = re-beep while VOLTAGE holds
  uint16_t contRepeatMs;   // repeat period while CLOSED
  uint16_t voltRepeatMs;   // repeat period while VOLTAGE
  uint16_t beepMinMs;      // min gap between beep sequences (rate cap)

  // -- Beep pulse shape --------------------------------------------
  uint16_t contOnMs;       // first-contact continuity pulse on-time
  uint16_t contHoldMs;     // ongoing "still there" continuity pulse on-time
  uint16_t contOffMs;      // gap between continuity pulses (both cases)
  uint16_t voltOnMs;       // voltage pulse on-time
  uint16_t voltOffMs;      // gap between voltage pulses
  // VDC- is short-LONG: every pulse but the last uses voltOnMs and the final one
  // stretches to voltNegLongMs.  VAC keeps the short pulse and adds a third.
  // The kinds differ by RHYTHM, not pitch -- the resonator is only loud near 4 kHz.
  uint16_t voltNegLongMs;  // VDC-: on-time of the final (long) pulse
  uint8_t  voltAcPulses;   // VAC: pulses per beep (each short, at voltOnMs)

  // -- Alert LED (per detection state) ------------------------------
  // One colour channel per state (FLOAT blue, CLOSED green, VOLTAGE red), so
  // "brightness" is that channel's value.  Hue is fixed: the colour is the meaning.
  uint8_t  ledFloatBright;
  uint8_t  ledClosedBright;
  uint8_t  ledVoltBright;
  // Flash on-time, and the minimum gap between flash STARTS (a rate cap).
  uint16_t ledFloatMs;
  uint16_t ledClosedMs;
  uint16_t ledVoltMs;
  uint16_t ledFloatPerMs;
  uint16_t ledClosedPerMs;
  uint16_t ledVoltPerMs;

  // -- Power / battery ---------------------------------------------
  float    chargeThreshV;  // charge-sense volts (VBUS/2) above this = charging
  float    battEmptyV;     // battery voltage mapped to 0%
  float    battFullV;      // battery voltage mapped to 100%
  uint8_t  battFullPct;    // >= this % while charging = green charge blink

  // -- Low-power timeout -------------------------------------------
  uint16_t idleTimeoutS;   // seconds of open-lead inactivity before sleeping (0 = never)
  uint16_t sleepTickMs;    // RTC wake period, snapped to the ladder and written back
  uint8_t  sleepPollTicks; // probe every N wake ticks
  uint8_t  sleepVoltAvg;   // reads for the voltage check while asleep
  uint8_t  sleepHbTicks;   // heartbeat flash every N ticks (0 = no heartbeat)
  uint8_t  sleepParkOff;   // 1 = park the bridge MOSFET OFF while asleep
  // Wake threshold per position, used ONLY by the sleeping probe (its metric
  // runs a few percent high).  0 = fall back to thresh[] for that position.
  float    sleepThresh[4];

  // -- Misc --------------------------------------------------------
  uint16_t loopDelayMs;    // main-loop pacing (WFI idle between passes)

  // ════ v9: everything below was APPENDED to the v8 layout ═════════
  // configMigrateV8() relies on that: a v8 image is a byte-identical prefix of
  // this struct.  Keep appending new fields here (and bump the version), never
  // insert above this line.

  // -- Power gates (HWREV 4) ---------------------------------------
  // The load switches on V3b.  Pins and polarity are fixed by the PCB (see
  // applyBoardPins); only the behaviour is config.
  //   mode    0 ALWAYS on / 1 on awake, off while parked / 2 pulsed
  //   settle  how long to wait after raising the gate before trusting what is
  //           downstream of it -- microseconds for ANA, milliseconds for LED
  //           (an SK6812 rail coming up is a much slower thing)
  uint8_t  gAnaMode;
  uint16_t gAnaSettleUs;
  uint8_t  gLedMode;
  uint16_t gLedSettleMs;

  // -- Deep sleep (stage 2) ----------------------------------------
  //   deepSec  seconds of stage-1 sleep, every probe reading open, before
  //            dropping to stage 2.  Counted in TICKS (millis() is frozen in
  //            Standby).  0 = never, i.e. single-stage.
  //   deepHz   stage-2 probe rate.  Re-programs the RTC wake period itself --
  //            skipping probes on a fast tick would save almost nothing, since
  //            the standby wake is most of the cost at these rates.  Snapped to
  //            the ladder (never faster than asked) and written back.
  //   deepParkOff  bridge MOSFET in stage 2: 0 resting, 1 parked OFF,
  //            2 = follow sleepParkOff.  CAUTION with 1: the node then floats a
  //            whole deep period, so raise SETTLEPOSTMS if closed leads stop
  //            waking the board, and check with !SLEEPTEST / !SLEEPLOG.
  uint16_t deepSec;
  float    deepHz;
  uint8_t  deepParkOff;

  // -- Charge-detect inhibit ---------------------------------------
  // 1 (normal): VBUS present = on the charger = no sleep, no normal alerts,
  //   immediate wake.  Right for a product whose charger is a USB cable that
  //   can earth-ground the meter.
  // 0: charge is still detected and REPORTED but inhibits nothing -- for a
  //   board powered through its 5 V input by a bench supply or a boost.
  uint8_t  chargeInhibit;

  // -- Buzzer amplifier (HWREV 4) ----------------------------------
  // PAM8904 gain per alert: 0 = silent, 1 = 1x, 2 = 2x, 3 = 3x (EN1/EN2).
  // Separate keys because the two alerts want different things: a voltage
  // alert is the safety signal and should be loud, continuity repeats every
  // second while the leads are closed and is the one people want quieter.
  uint8_t  contVol;
  uint8_t  voltVol;
  // Duty cycle of the tone on D15, 1-50 %.  A finer trim below one gain step:
  // the fundamental scales with sin(pi * duty), so 50 = full, 25 = -3 dB,
  // 12 = -8 dB, 5 = -16 dB.  Above 50 is the same waveform inverted, hence the cap.
  uint8_t  spkDuty;

  uint16_t crc;            // CRC16 over everything above (must stay LAST)
};

Config cfg;                // live (RAM) configuration
bool   cfgDirty = false;   // RAM differs from EEPROM (informational, in $STATUS)

// Factory defaults.  memset first so struct padding is deterministic and the
// CRC of a defaults-derived image is reproducible.
void configDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;

  // Defaults describe a NEW board, which is a V3b.  Units carrying a stored
  // config keep (v6+) or are pinned to (pre-v6: 2) their revision by the
  // migrations -- this default only ever reaches a blank or reset EEPROM.
  cfg.hwRev          = HWREV_V3B;

  cfg.refCenterV     = 0.018f;
  cfg.refBandV       = 0.025f;
  // thresh[] is in the units of cfg.detectMethod, which is 1 (time-to-return),
  // so slot 11 is 1.0 MILLISECONDS.  Slots 00-10 are old method-0 volt figures
  // and have NOT been re-tuned for method 1 -- placeholders.  All of these were
  // tuned on V3 with an always-on analog rail; re-characterise on V3b.
  cfg.thresh[0]      = 0.15f;    // 00: not re-tuned for method 1
  cfg.thresh[1]      = 0.54f;    // 01: not re-tuned for method 1
  cfg.thresh[2]      = 0.45f;    // 10: not re-tuned for method 1
  cfg.thresh[3]      = 1.0f;     // 11: ms, the factory selection
  cfg.threshSel      = 3;
  cfg.voltFastMult   = 5.0f;
  cfg.voltAvgSamples = 10;
  cfg.testAgree      = 1;
  cfg.stableCount    = 2;
  cfg.settlePreUs    = 300;
  cfg.settlePostMs   = 3;
  cfg.negFix         = 1;
  cfg.negFixV        = 1.250f;

  cfg.detectMethod   = 1;        // time-to-return
  cfg.detReturnBand  = 0.05f;
  cfg.detWindowUs    = 1500;
  cfg.detAreaStartUs = 400;

  cfg.voltClassify   = 1;
  cfg.acWindowMs     = 25;       // > one 50 Hz cycle; 1.5 cycles at 60 Hz
  cfg.acBandV        = 0.0f;     // 0 = voltFastMult * refBandV

  cfg.ledEnable      = 1;
  cfg.beepEnable     = 1;
  cfg.bootMute       = 1;
  cfg.passiveBuzzer  = 1;
  cfg.spkDiff        = 1;        // V3: anti-phase drive (loudest)
  // The fitted buzzer is a 4 kHz resonator on every revision, so both alerts
  // sit there and are told apart by pulse count, not pitch.
  cfg.contFreqHz     = 4000;
  cfg.voltFreqHz     = 4000;
  cfg.contPulses     = 1;
  cfg.voltPulses     = 2;
  cfg.contRepeat     = 1;
  cfg.voltRepeat     = 0;
  cfg.contRepeatMs   = 1000;
  cfg.voltRepeatMs   = 1000;
  cfg.beepMinMs      = 250;

  cfg.contOnMs       = 100;
  cfg.contHoldMs     = 100;
  cfg.contOffMs      = 10;
  cfg.voltOnMs       = 20;
  cfg.voltOffMs      = 10;
  cfg.voltNegLongMs  = 150;
  cfg.voltAcPulses   = 3;

  cfg.ledFloatBright  = 20;
  cfg.ledClosedBright = 64;
  cfg.ledVoltBright   = 200;
  cfg.ledFloatMs      = 50;
  cfg.ledClosedMs     = 200;
  cfg.ledVoltMs       = 200;
  cfg.ledFloatPerMs   = 1000;
  cfg.ledClosedPerMs  = 500;
  cfg.ledVoltPerMs    = 500;

  cfg.chargeThreshV  = 2.0f;
  // 3.70, not the cell's electrical floor: below ~3.6 V moving the leads trips
  // the voltage detector (measured Aug 2026), so "empty" means "stop believing
  // it" rather than "the cell is flat".
  cfg.battEmptyV     = 3.70f;
  cfg.battFullV      = 4.20f;
  cfg.battFullPct    = 90;

  // Sleep is the operating state: 1 s of open leads, then wake ~16x/s to probe.
  cfg.idleTimeoutS   = 1;
  cfg.sleepTickMs    = 63;
  cfg.sleepPollTicks = 1;
  cfg.sleepVoltAvg   = 3;
  cfg.sleepHbTicks   = 32;
  cfg.sleepParkOff   = 0;
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = 0.0f;   // 0 = use thresh[i]
  cfg.sleepThresh[3] = 1.2f;     // ms: a probe out of standby reads high

  cfg.loopDelayMs    = 50;

  // -- v9 ------------------------------------------------------------
  // Gates: the scheme the bench unit was characterised on (ANAMODE 2 /
  // ANAUS 4000, LEDGMODE 2 / LEDGMS 2).  Pulsed on both -- the analog rail is
  // up only for a measurement and LED1 only while it is lit.  ANAUS was tuned
  // on a breadboard switch, not the TPS22914: confirm it with !CAP on V3b.
  cfg.gAnaMode       = 2;
  cfg.gAnaSettleUs   = 4000;
  cfg.gLedMode       = 2;
  cfg.gLedSettleMs   = 2;

  cfg.deepSec        = 5;        // 5 s of open leads in stage 1 -> stage 2
  cfg.deepHz         = 1.0f;     // stage 2 probes once a second
  cfg.deepParkOff    = 2;        // follow SLEEPPARK
  cfg.chargeInhibit  = 1;

  cfg.contVol        = 2;        // 2x: continuity repeats, so a step down
  cfg.voltVol        = 3;        // 3x: the safety alert gets everything
  cfg.spkDuty        = 50;       // full
}

// CRC16-CCITT over an arbitrary byte range (shared by Config and SerialId).
uint16_t crc16_ccitt(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// CRC16-CCITT over the struct bytes, excluding the trailing crc field.
// offsetof, NOT sizeof-2: the struct is 4-byte aligned so there are padding
// bytes AFTER crc, and sizeof-2 would run the CRC over the crc field itself --
// making every reload/boot validation fail and re-seed defaults.
uint16_t configCrc(const Config &c) {
  return crc16_ccitt((const uint8_t *)&c, offsetof(Config, crc));
}

// Persist RAM config to EEPROM (data flash).  Only called on !SAVE / first
// boot -- data-flash writes are slow and endurance-limited, so the firmware
// never saves on its own during normal operation.
void configSave() {
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.crc     = configCrc(cfg);
  EEPROM.put(CFG_EEPROM_ADDR, cfg);
  cfgDirty = false;
}

// Load config from EEPROM into RAM.  Returns true if the stored image was
// valid; on failure the caller decides whether to fall back to defaults.
bool configLoad() {
  Config stored;
  EEPROM.get(CFG_EEPROM_ADDR, stored);
  if (stored.magic != CFG_MAGIC)          return false;
  if (stored.version != CFG_VERSION)      return false;
  if (stored.crc != configCrc(stored))    return false;
  cfg = stored;
  cfgDirty = false;
  return true;
}

// ── v8 -> v9 ──────────────────────────────────────────────────────
// v8 EXACTLY as it shipped in BlinkyHawk_RA4M1 (commit 4f4fecf).  Frozen --
// never edit.  v9 is this layout with fields appended before crc, so the
// whole v8 image up to its crc is a byte-identical prefix of Config, which the
// static_asserts below pin down.  That lets the migration be a prefix copy
// rather than sixty field-by-field assignments that could drift.
struct ConfigV8 {
  uint32_t magic;
  uint16_t version;
  uint8_t  hwRev;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  uint8_t  threshSel;
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  voltClassify;
  uint16_t acWindowMs;
  float    acBandV;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint8_t  spkDiff;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  uint16_t contOnMs;
  uint16_t contHoldMs;
  uint16_t contOffMs;
  uint16_t voltOnMs;
  uint16_t voltOffMs;
  uint16_t voltNegLongMs;
  uint8_t  voltAcPulses;
  uint8_t  ledFloatBright;
  uint8_t  ledClosedBright;
  uint8_t  ledVoltBright;
  uint16_t ledFloatMs;
  uint16_t ledClosedMs;
  uint16_t ledVoltMs;
  uint16_t ledFloatPerMs;
  uint16_t ledClosedPerMs;
  uint16_t ledVoltPerMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint16_t sleepTickMs;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  float    sleepThresh[4];
  uint16_t loopDelayMs;
  uint16_t crc;
};

// The prefix copy is only valid if v9 really is v8 plus an appended tail.
// These catch an insertion above the v9 marker at compile time.
static_assert(offsetof(Config, gAnaMode) == offsetof(ConfigV8, crc),
              "Config v9 must be ConfigV8 with fields appended before crc");
static_assert(offsetof(Config, loopDelayMs) == offsetof(ConfigV8, loopDelayMs) &&
              offsetof(Config, voltAcPulses) == offsetof(ConfigV8, voltAcPulses) &&
              offsetof(Config, acBandV) == offsetof(ConfigV8, acBandV) &&
              offsetof(Config, hwRev) == offsetof(ConfigV8, hwRev),
              "Config v9 must keep every v8 field at its v8 offset");

// Upgrade a stored v8 image.  hwRev is carried across (v8 knew about V3).
// The appended fields keep their defaults -- except that the caller turns the
// deep stage off for any migrated unit (see setup), so a field unit's sleep
// behaves exactly as it did.  The gate and volume keys do nothing on the V2/V3
// hardware a v8 image necessarily came from.
bool configMigrateV8() {
  ConfigV8 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 8)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV8, crc))) return false;

  memcpy(&cfg, &old, offsetof(ConfigV8, crc));   // cfg holds defaults beyond this
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  return true;
}

// ── Upgrade path from older stored layouts ────────────────────────
// EEPROM here is the RA4M1's data flash, which a sketch upload does NOT erase --
// so a unit reflashed with new firmware still has its old config sitting there.
// configLoad() rejects it (the version differs, and the bytes genuinely can't
// be reinterpreted as the new struct), which for a field-tuned unit means
// silently reverting it to factory thresholds.  Instead, keep each superseded
// layout frozen here and copy the values across.
//
// Every migration from v5 AND OLDER sets cfg.hwRev = 2, and that was the whole
// point of the v6 bump: a config stored in one of those layouts can only exist
// on a unit that predates V3, and every one of those is an OpenLead_Headless V2
// with DIP switches on D8/D10.  (v6 onwards knows about hwRev for real, so
// configMigrateV6 carries the stored value across instead -- see there.)
// Inheriting the current DEFAULT (hwRev 4) instead would make a reflashed V2
// drive D8 as a push-pull output straight into whatever the DIP switch is doing
// -- a dead short to ground whenever that switch is closed.  A V2 that is later
// rebuilt as a V3 is a deliberate !SET,HWREV,3 + !SAVE, never an accident.
//
// v7 EXACTLY as it shipped (v6 plus the beep pulse-shape and alert LED
// brightness/timing keys, before the VDC+/VDC-/VAC classifier).  Frozen --
// never edit.  Like v6 this can be a genuine V3 board, so hwRev is carried
// across rather than forced to 2.
struct ConfigV7 {
  uint32_t magic;
  uint16_t version;
  uint8_t  hwRev;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  uint8_t  threshSel;
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint8_t  spkDiff;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  uint16_t contOnMs;
  uint16_t contHoldMs;
  uint16_t contOffMs;
  uint16_t voltOnMs;
  uint16_t voltOffMs;
  uint8_t  ledFloatBright;
  uint8_t  ledClosedBright;
  uint8_t  ledVoltBright;
  uint16_t ledFloatMs;
  uint16_t ledClosedMs;
  uint16_t ledVoltMs;
  uint16_t ledFloatPerMs;
  uint16_t ledClosedPerMs;
  uint16_t ledVoltPerMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint16_t sleepTickMs;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  float    sleepThresh[4];
  uint16_t loopDelayMs;
  uint16_t crc;
};

// v6 EXACTLY as it shipped (v5 plus hwRev/threshSel/spkDiff, before the beep
// pulse-shape and LED brightness/timing keys).  Frozen -- never edit.
// NOTE: unlike v5 and older, a v6 image CAN be a genuine V3 board, so this
// migration is the one that must carry hwRev across rather than forcing 2.
struct ConfigV6 {
  uint32_t magic;
  uint16_t version;
  uint8_t  hwRev;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  uint8_t  threshSel;
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint8_t  spkDiff;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint16_t sleepTickMs;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  float    sleepThresh[4];
  uint16_t loopDelayMs;
  uint16_t crc;
};

// v5 EXACTLY as it shipped (v4 plus sleepTickMs, before hwRev/threshSel/spkDiff).
// Frozen -- never edit; see the note on ConfigV2.
struct ConfigV5 {
  uint32_t magic;
  uint16_t version;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint16_t sleepTickMs;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  float    sleepThresh[4];
  uint16_t loopDelayMs;
  uint16_t crc;
};

// v4 EXACTLY as it shipped (v3 plus sleepThresh[], before sleepTickMs existed).
// Frozen -- never edit; see the note on ConfigV2.
struct ConfigV4 {
  uint32_t magic;
  uint16_t version;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  float    sleepThresh[4];
  uint16_t loopDelayMs;
  uint16_t crc;
};

// v3 EXACTLY as it shipped (v2 plus the low-power timeout block, before
// sleepThresh[] existed).  Frozen -- never edit; see the note on ConfigV2.
struct ConfigV3 {
  uint32_t magic;
  uint16_t version;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  uint16_t loopDelayMs;
  uint16_t crc;
};

// This struct is v2 EXACTLY as it shipped.  It must never be edited again --
// it is not "the config minus the new fields", it is a historical record of
// what is actually in the flash of units already in the field.
struct ConfigV2 {
  uint32_t magic;
  uint16_t version;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t loopDelayMs;
  uint16_t crc;
};

// Upgrade a stored v7 image.  The classifier keys keep their defaults, so a
// migrated unit gains VDC-/VAC discrimination but its VDC+ alert -- the only
// one v7 could produce -- looks and sounds exactly as it did (VKIND_DCP is
// red-red at voltPulses x voltOnMs, which is the v7 flash and beep verbatim).
// hwRev is CARRIED ACROSS, not forced to 2: v7, like v6, knew about V3 boards.
bool configMigrateV7() {
  ConfigV7 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 7)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV7, crc))) return false;

  cfg.hwRev           = old.hwRev;
  cfg.refCenterV      = old.refCenterV;
  cfg.refBandV        = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.threshSel       = old.threshSel;
  cfg.voltFastMult    = old.voltFastMult;
  cfg.voltAvgSamples  = old.voltAvgSamples;
  cfg.testAgree       = old.testAgree;
  cfg.stableCount     = old.stableCount;
  cfg.settlePreUs     = old.settlePreUs;
  cfg.settlePostMs    = old.settlePostMs;
  cfg.negFix          = old.negFix;
  cfg.negFixV         = old.negFixV;
  cfg.detectMethod    = old.detectMethod;
  cfg.detReturnBand   = old.detReturnBand;
  cfg.detWindowUs     = old.detWindowUs;
  cfg.detAreaStartUs  = old.detAreaStartUs;
  cfg.ledEnable       = old.ledEnable;
  cfg.beepEnable      = old.beepEnable;
  cfg.bootMute        = old.bootMute;
  cfg.passiveBuzzer   = old.passiveBuzzer;
  cfg.spkDiff         = old.spkDiff;
  cfg.contFreqHz      = old.contFreqHz;
  cfg.voltFreqHz      = old.voltFreqHz;
  cfg.contPulses      = old.contPulses;
  cfg.voltPulses      = old.voltPulses;
  cfg.contRepeat      = old.contRepeat;
  cfg.voltRepeat      = old.voltRepeat;
  cfg.contRepeatMs    = old.contRepeatMs;
  cfg.voltRepeatMs    = old.voltRepeatMs;
  cfg.beepMinMs       = old.beepMinMs;
  cfg.contOnMs        = old.contOnMs;
  cfg.contHoldMs      = old.contHoldMs;
  cfg.contOffMs       = old.contOffMs;
  cfg.voltOnMs        = old.voltOnMs;
  cfg.voltOffMs       = old.voltOffMs;
  cfg.ledFloatBright  = old.ledFloatBright;
  cfg.ledClosedBright = old.ledClosedBright;
  cfg.ledVoltBright   = old.ledVoltBright;
  cfg.ledFloatMs      = old.ledFloatMs;
  cfg.ledClosedMs     = old.ledClosedMs;
  cfg.ledVoltMs       = old.ledVoltMs;
  cfg.ledFloatPerMs   = old.ledFloatPerMs;
  cfg.ledClosedPerMs  = old.ledClosedPerMs;
  cfg.ledVoltPerMs    = old.ledVoltPerMs;
  cfg.chargeThreshV   = old.chargeThreshV;
  cfg.battEmptyV      = old.battEmptyV;
  cfg.battFullV       = old.battFullV;
  cfg.battFullPct     = old.battFullPct;
  cfg.idleTimeoutS    = old.idleTimeoutS;
  cfg.sleepTickMs     = old.sleepTickMs;
  cfg.sleepPollTicks  = old.sleepPollTicks;
  cfg.sleepVoltAvg    = old.sleepVoltAvg;
  cfg.sleepHbTicks    = old.sleepHbTicks;
  cfg.sleepParkOff    = old.sleepParkOff;
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = old.sleepThresh[i];
  cfg.loopDelayMs     = old.loopDelayMs;
  return true;
}

// Upgrade a stored v6 image.  The new beep/LED keys keep their defaults, which
// are the exact constants v6 had compiled in, so nothing changes audibly or
// visibly.  hwRev is CARRIED ACROSS here, not forced to 2: v6 is the first
// layout that knew about V3 boards, so a stored 3 is real information.
bool configMigrateV6() {
  ConfigV6 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 6)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV6, crc))) return false;

  cfg.hwRev          = old.hwRev;
  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.threshSel      = old.threshSel;
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.spkDiff        = old.spkDiff;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.idleTimeoutS   = old.idleTimeoutS;
  cfg.sleepTickMs    = old.sleepTickMs;
  cfg.sleepPollTicks = old.sleepPollTicks;
  cfg.sleepVoltAvg   = old.sleepVoltAvg;
  cfg.sleepHbTicks   = old.sleepHbTicks;
  cfg.sleepParkOff   = old.sleepParkOff;
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = old.sleepThresh[i];
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// Upgrade a stored v5 image.  threshSel and spkDiff are irrelevant on the V2
// hardware this necessarily is (the DIP pins decide the threshold, and D8 must
// stay an input), so only hwRev actually matters here.
bool configMigrateV5() {
  ConfigV5 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 5)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV5, crc))) return false;

  cfg.hwRev          = 2;        // see the note above ConfigV5
  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.idleTimeoutS   = old.idleTimeoutS;
  cfg.sleepTickMs    = old.sleepTickMs;
  cfg.sleepPollTicks = old.sleepPollTicks;
  cfg.sleepVoltAvg   = old.sleepVoltAvg;
  cfg.sleepHbTicks   = old.sleepHbTicks;
  cfg.sleepParkOff   = old.sleepParkOff;
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = old.sleepThresh[i];
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// Upgrade a stored v4 image.  sleepTickMs keeps its default (2000), which is
// the rate v4 hard-coded, so a migrated unit wakes at exactly the same cadence.
bool configMigrateV4() {
  ConfigV4 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 4)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV4, crc))) return false;

  cfg.hwRev          = 2;        // see the note above ConfigV5
  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.idleTimeoutS   = old.idleTimeoutS;
  cfg.sleepPollTicks = old.sleepPollTicks;
  cfg.sleepVoltAvg   = old.sleepVoltAvg;
  cfg.sleepHbTicks   = old.sleepHbTicks;
  cfg.sleepParkOff   = old.sleepParkOff;
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = old.sleepThresh[i];
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// Upgrade a stored v3 image.  cfg must already hold defaults on entry, so the
// fields v3 never had (sleepThresh[]) keep their defaults -- which are 0, i.e.
// "fall back to thresh[]", so a migrated unit behaves exactly as it did before.
bool configMigrateV3() {
  ConfigV3 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 3)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV3, crc))) return false;

  cfg.hwRev          = 2;        // see the note above ConfigV5
  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.idleTimeoutS   = old.idleTimeoutS;
  cfg.sleepPollTicks = old.sleepPollTicks;
  cfg.sleepVoltAvg   = old.sleepVoltAvg;
  cfg.sleepHbTicks   = old.sleepHbTicks;
  cfg.sleepParkOff   = old.sleepParkOff;
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// Try to upgrade a stored v2 image into the live config.  cfg must already
// hold defaults on entry, so the fields v2 never had keep their default values.
// Returns true if a valid v2 image was found and migrated.
bool configMigrateV2() {
  ConfigV2 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 2)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV2, crc))) return false;

  cfg.hwRev          = 2;        // see the note above ConfigV5
  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// ══════════════════════════════════════════════════════════════════
//  UNIT SERIAL NUMBER (device identity)
// ══════════════════════════════════════════════════════════════════
// Stored SEPARATELY from Config, in its own EEPROM block, so it survives
// !DEFAULTS and any future CFG_VERSION bump -- the SN is the unit's permanent
// identity, not a tunable.  Written once (usually at first bring-up) via !SN,
// read back on every boot and reported in $STATUS so the host can key its
// per-unit config log on it.
#define SN_MAGIC     0x42485F53UL   // "BH_S"
#define SN_MAX_LEN   16             // buffer size incl. null terminator
#define SN_EEPROM_ADDR 512          // well clear of the Config block at addr 0
                                    // (Config must stay < 512 bytes; it is ~150)

struct SerialId {
  uint32_t magic;
  char     sn[SN_MAX_LEN];
  uint16_t crc;
};

char unitSN[SN_MAX_LEN] = "";       // live copy ("" = unassigned)

// Explicit prototype (see the ConfigField note above): keeps the Arduino
// preprocessor from hoisting an auto-generated prototype above SerialId.
uint16_t snCrc(const SerialId &s);

uint16_t snCrc(const SerialId &s) {
  return crc16_ccitt((const uint8_t *)&s, offsetof(SerialId, crc));
}

// Load the SN from EEPROM into unitSN, or leave it empty if unset/corrupt.
void snLoad() {
  SerialId s;
  EEPROM.get(SN_EEPROM_ADDR, s);
  if (s.magic == SN_MAGIC && s.crc == snCrc(s)) {
    s.sn[SN_MAX_LEN - 1] = '\0';    // paranoia: guarantee termination
    strncpy(unitSN, s.sn, SN_MAX_LEN);
    unitSN[SN_MAX_LEN - 1] = '\0';
  } else {
    unitSN[0] = '\0';
  }
}

// Persist a new SN to EEPROM and update the live copy.
void snSave(const char *sn) {
  SerialId s;
  memset(&s, 0, sizeof(s));         // deterministic padding for a stable CRC
  s.magic = SN_MAGIC;
  strncpy(s.sn, sn, SN_MAX_LEN - 1);
  s.sn[SN_MAX_LEN - 1] = '\0';
  s.crc = snCrc(s);
  EEPROM.put(SN_EEPROM_ADDR, s);
  strncpy(unitSN, s.sn, SN_MAX_LEN);
  unitSN[SN_MAX_LEN - 1] = '\0';
}

// ── Config field table ────────────────────────────────────────────
// Maps serial key names onto struct fields so !SET/!GET/!CFG are generic:
// adding a tunable = add the struct field, a default, and one table row.
// min/max bounds stop a bad !SET from bricking a unit (values are clamped
// and the clamped value is echoed back).
enum FieldType { FT_FLOAT, FT_U8, FT_U16, FT_BOOL };

struct ConfigField {
  const char *name;
  FieldType   type;
  void       *ptr;
  float       minV;
  float       maxV;
};

// Explicit prototypes: the Arduino preprocessor would otherwise hoist its
// auto-generated ones above this struct definition and fail to compile.
const ConfigField *findField(const char *name);
float fieldGet(const ConfigField *f);
void  fieldSet(const ConfigField *f, float v);
void  printField(const ConfigField *f);

const ConfigField CFG_FIELDS[] = {
  // Board revision (decides pin roles -- see the Config struct)
  { "HWREV",       FT_U8,    &cfg.hwRev,           2,      4      },
  // Detection
  { "REFCENTER",   FT_FLOAT, &cfg.refCenterV,     -1.0f,   1.0f   },
  { "REFBAND",     FT_FLOAT, &cfg.refBandV,        0.001f, 1.0f   },
  { "THRESH00",    FT_FLOAT, &cfg.thresh[0],       0.001f, 3.3f   },
  { "THRESH01",    FT_FLOAT, &cfg.thresh[1],       0.001f, 3.3f   },
  { "THRESH10",    FT_FLOAT, &cfg.thresh[2],       0.001f, 3.3f   },
  { "THRESH11",    FT_FLOAT, &cfg.thresh[3],       0.001f, 3.3f   },
  // Which of the four above is active.  hwRev 3+ only -- on hwRev 2 the DIP
  // switches decide and this value is ignored.
  { "THRESHSEL",   FT_U8,    &cfg.threshSel,       0,      3      },
  { "VOLTFAST",    FT_FLOAT, &cfg.voltFastMult,    1.0f,   50.0f  },
  { "VOLTAVG",     FT_U8,    &cfg.voltAvgSamples,  1,      50     },
  { "TESTAGREE",   FT_U8,    &cfg.testAgree,       1,      10     },
  { "STABLECOUNT", FT_U8,    &cfg.stableCount,     1,      10     },
  { "SETTLEPREUS", FT_U16,   &cfg.settlePreUs,     0,      5000   },
  { "SETTLEPOSTMS",FT_U8,    &cfg.settlePostMs,    0,      50     },
  { "NEGFIX",      FT_BOOL,  &cfg.negFix,          0,      1      },
  { "NEGV",        FT_FLOAT, &cfg.negFixV,         0.0f,   3.3f   },
  { "DETMETHOD",   FT_U8,    &cfg.detectMethod,    0,      2      },
  { "DETBAND",     FT_FLOAT, &cfg.detReturnBand,   0.005f, 1.0f   },
  { "DETWINUS",    FT_U16,   &cfg.detWindowUs,     200,    5000   },
  { "DETAREAUS",   FT_U16,   &cfg.detAreaStartUs,  0,      5000   },
  // Voltage kind (VDC+ / VDC- / VAC).  VCLASS=0 reverts to the single all-red
  // "voltage present" alert, which is the escape hatch if a unit's front end
  // turns out not to resolve polarity cleanly -- no reflash needed.
  { "VCLASS",      FT_BOOL,  &cfg.voltClassify,    0,      1      },
  // Lower bound 20 ms is not padding: the window has to cover a full 50 Hz
  // cycle or a half cycle can be all that is sampled, and AC reads as DC.
  { "ACWINMS",     FT_U16,   &cfg.acWindowMs,      20,     200    },
  { "ACBAND",      FT_FLOAT, &cfg.acBandV,         0.0f,   3.3f   },
  // Alerts
  { "LED",         FT_BOOL,  &cfg.ledEnable,       0,      1      },
  { "BEEP",        FT_BOOL,  &cfg.beepEnable,      0,      1      },
  { "BOOTMUTE",    FT_BOOL,  &cfg.bootMute,        0,      1      },
  { "PASSIVE",     FT_BOOL,  &cfg.passiveBuzzer,   0,      1      },
  { "SPKDIFF",     FT_BOOL,  &cfg.spkDiff,         0,      1      },
  { "CONTFREQ",    FT_U16,   &cfg.contFreqHz,      100,    10000  },
  { "VOLTFREQ",    FT_U16,   &cfg.voltFreqHz,      100,    10000  },
  { "CONTPULSES",  FT_U8,    &cfg.contPulses,      1,      5      },
  { "VOLTPULSES",  FT_U8,    &cfg.voltPulses,      1,      5      },
  { "CONTREP",     FT_BOOL,  &cfg.contRepeat,      0,      1      },
  { "VOLTREP",     FT_BOOL,  &cfg.voltRepeat,      0,      1      },
  { "CONTREPMS",   FT_U16,   &cfg.contRepeatMs,    100,    60000  },
  { "VOLTREPMS",   FT_U16,   &cfg.voltRepeatMs,    100,    60000  },
  { "BEEPMIN",     FT_U16,   &cfg.beepMinMs,       50,     10000  },
  // Beep pulse shape.  Lower bound 1 ms rather than 0: a zero-length pulse
  // would leave the beep state machine switching the speaker on and straight
  // back off every pass, which is a click, not silence -- use BEEP to mute.
  { "CONTONMS",    FT_U16,   &cfg.contOnMs,        1,      2000   },
  { "CONTHOLDMS",  FT_U16,   &cfg.contHoldMs,      1,      2000   },
  { "CONTOFFMS",   FT_U16,   &cfg.contOffMs,       1,      2000   },
  { "VOLTONMS",    FT_U16,   &cfg.voltOnMs,        1,      2000   },
  { "VOLTOFFMS",   FT_U16,   &cfg.voltOffMs,       1,      2000   },
  // VDC- final pulse, and the VAC pulse count -- the rhythms that separate the
  // three voltage kinds.  VOLTPULSES still sets the count for VDC+ and VDC-.
  { "VOLTLONGMS",  FT_U16,   &cfg.voltNegLongMs,   1,      2000   },
  { "VACPULSES",   FT_U8,    &cfg.voltAcPulses,    1,      5      },
  // Alert LED: brightness (0 = that state's LED off), flash on-time, and the
  // minimum gap between flash starts.  0 brightness is allowed -- it silences
  // one state's LED without disabling the other two, which LED cannot do.
  { "LEDFLOATBR",  FT_U8,    &cfg.ledFloatBright,  0,      255    },
  { "LEDCLOSEDBR", FT_U8,    &cfg.ledClosedBright, 0,      255    },
  { "LEDVOLTBR",   FT_U8,    &cfg.ledVoltBright,   0,      255    },
  { "LEDFLOATMS",  FT_U16,   &cfg.ledFloatMs,      1,      10000  },
  { "LEDCLOSEDMS", FT_U16,   &cfg.ledClosedMs,     1,      10000  },
  { "LEDVOLTMS",   FT_U16,   &cfg.ledVoltMs,       1,      10000  },
  { "LEDFLOATPER", FT_U16,   &cfg.ledFloatPerMs,   1,      60000  },
  { "LEDCLOSEDPER",FT_U16,   &cfg.ledClosedPerMs,  1,      60000  },
  { "LEDVOLTPER",  FT_U16,   &cfg.ledVoltPerMs,    1,      60000  },
  // Power / battery
  { "CHGTHRESH",   FT_FLOAT, &cfg.chargeThreshV,   0.5f,   3.3f   },
  { "BATTEMPTY",   FT_FLOAT, &cfg.battEmptyV,      2.5f,   4.0f   },
  { "BATTFULL",    FT_FLOAT, &cfg.battFullV,       3.0f,   4.5f   },
  { "BATTFULLPCT", FT_U8,    &cfg.battFullPct,     50,     100    },
  // Low-power timeout
  { "SLEEPSEC",    FT_U16,   &cfg.idleTimeoutS,    0,      65535  },
  // SLEEPTICKMS is snapped to the RTC ladder (SLEEP_TICK_OPTIONS), so the
  // bounds only have to admit the ends of it -- 4 ms = 1/256 s, 2000 = 2 s.
  { "SLEEPTICKMS", FT_U16,   &cfg.sleepTickMs,     4,      2000   },
  // 255 rather than 100 so the fast rungs can still be paired with a slow probe
  // cadence: at a 16 ms tick, 100 ticks is only 1.6 s between probes.
  { "SLEEPTICKS",  FT_U8,    &cfg.sleepPollTicks,  1,      255    },
  { "SLEEPAVG",    FT_U8,    &cfg.sleepVoltAvg,    1,      50     },
  { "SLEEPHB",     FT_U8,    &cfg.sleepHbTicks,    0,      255    },
  { "SLEEPPARK",   FT_BOOL,  &cfg.sleepParkOff,    0,      1      },
  // Wake thresholds: 0 = use the matching THRESH value for that DIP position.
  { "SLEEPTHR00",  FT_FLOAT, &cfg.sleepThresh[0],  0.0f,   3.3f   },
  { "SLEEPTHR01",  FT_FLOAT, &cfg.sleepThresh[1],  0.0f,   3.3f   },
  { "SLEEPTHR10",  FT_FLOAT, &cfg.sleepThresh[2],  0.0f,   3.3f   },
  { "SLEEPTHR11",  FT_FLOAT, &cfg.sleepThresh[3],  0.0f,   3.3f   },
  // Misc
  { "LOOPMS",      FT_U16,   &cfg.loopDelayMs,     1,      1000   },

  // Buzzer amplifier (HWREV 4).  0 = that alert silent, 1-3 = PAM8904 gain.
  { "CONTVOL",     FT_U8,    &cfg.contVol,         0,      3      },
  { "VOLTVOL",     FT_U8,    &cfg.voltVol,         0,      3      },
  { "SPKDUTY",     FT_U8,    &cfg.spkDuty,         1,      50     },
  // Power gates (HWREV 4; inert on V2/V3).  Mode 0 always / 1 off asleep /
  // 2 pulsed.  Pins and polarity are fixed by the PCB.
  { "ANAMODE",     FT_U8,    &cfg.gAnaMode,        0,      2      },
  { "ANAUS",       FT_U16,   &cfg.gAnaSettleUs,    0,      60000  },
  { "LEDGMODE",    FT_U8,    &cfg.gLedMode,        0,      2      },
  { "LEDGMS",      FT_U16,   &cfg.gLedSettleMs,    0,      1000   },
  // Deep sleep (stage 2).  DEEPHZ is a float so sub-1 Hz rates are expressible;
  // the floor is 0.05 Hz because the slowest RTC rung is 2 s and 255 of those
  // is the most the tick counter should be asked to carry between probes.
  { "DEEPSEC",     FT_U16,   &cfg.deepSec,         0,      65535  },
  { "DEEPHZ",      FT_FLOAT, &cfg.deepHz,          0.05f,  100.0f },
  // 0 = bridge resting, 1 = bridge parked off, 2 = follow SLEEPPARK (default)
  { "DEEPPARK",    FT_U8,    &cfg.deepParkOff,     0,      2      },
  // 0 = charge detection inhibits nothing (bench supply / boost on the 5 V in)
  { "CHGINHIBIT",  FT_BOOL,  &cfg.chargeInhibit,   0,      1      },
};
const int CFG_FIELD_COUNT = sizeof(CFG_FIELDS) / sizeof(CFG_FIELDS[0]);

const ConfigField *findField(const char *name) {
  for (int i = 0; i < CFG_FIELD_COUNT; i++)
    if (strcmp(name, CFG_FIELDS[i].name) == 0) return &CFG_FIELDS[i];
  return NULL;
}

float fieldGet(const ConfigField *f) {
  switch (f->type) {
    case FT_FLOAT: return *(float *)f->ptr;
    case FT_U16:   return *(uint16_t *)f->ptr;
    default:       return *(uint8_t *)f->ptr;   // FT_U8 / FT_BOOL
  }
}

void fieldSet(const ConfigField *f, float v) {
  if (v < f->minV) v = f->minV;
  if (v > f->maxV) v = f->maxV;
  switch (f->type) {
    case FT_FLOAT: *(float *)f->ptr    = v;                      break;
    case FT_U16:   *(uint16_t *)f->ptr = (uint16_t)(v + 0.5f);   break;
    case FT_BOOL:  *(uint8_t *)f->ptr  = (v != 0.0f) ? 1 : 0;    break;
    default:       *(uint8_t *)f->ptr  = (uint8_t)(v + 0.5f);    break;
  }
}

void printField(const ConfigField *f) {
  Serial.print("$CFG,");
  Serial.print(f->name);
  Serial.print(",");
  if (f->type == FT_FLOAT) Serial.println(*(float *)f->ptr, 4);
  else                     Serial.println((long)fieldGet(f));
}

// ══════════════════════════════════════════════════════════════════
//  RUNTIME STATE (not persisted)
// ══════════════════════════════════════════════════════════════════
const int TEST_MAX_ATTEMPTS = 30;   // safety cap on MOSFET test repeats

// Recovery-transient methods (1 & 2): only start looking for the "return to
// zero" after the differential has actually dipped past this magnitude, so the
// first read (taken microseconds after MOSFET-off, before the node discharges)
// can't be mistaken for an instant return.  The trough is always ~1.1 V.
const float DET_TROUGH_MIN_V = 0.30f;

// LED colours.  Only the HUE is compile-time -- one channel per state, so the
// meaning of the colour cannot be reconfigured away.  The magnitude of that
// channel is cfg.ledFloatBright / ledClosedBright / ledVoltBright, and the
// cadence is cfg.led*Ms / cfg.led*PerMs.  See the Config struct.
//
// Not covered by these keys (still compile-time, different features):
// CHARGE_BLINK_* for the charging cue and SLEEP_HB_* for the sleep heartbeat.

#define NUM_PIXELS 1
Adafruit_NeoPixel pixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ══════════════════════════════════════════════════════════════════
//  BOARD PIN MAP (fixed per HWREV)
// ══════════════════════════════════════════════════════════════════
// Defined later in the file; needed here.
void sleepMs(unsigned long ms);
void buzzerApplyPinModes();
void buzzerStop();

// The pins !PINS can name, with the XIAO silkscreen label.  Display only --
// it also doubles as "is this a real pin" for the guards below.  The XIAO
// aliases its analog and digital names (A0 and D0 are the same pad), so
// numbers repeat.  A4/A5 are NOT defined by this variant: do not add them.
// The analog names come first so pinNameOf() reports A1/A3 -- what the
// schematic calls them -- rather than their D1/D3 aliases.
struct PinName { const char *name; uint8_t num; };
const PinName PIN_NAMES[] = {
  { "A0",  (uint8_t)A0 },  { "A1",  (uint8_t)A1 },  { "A2",  (uint8_t)A2 },
  { "A3",  (uint8_t)A3 },
  { "D0",  (uint8_t)D0 },  { "D1",  (uint8_t)D1 },  { "D2",  (uint8_t)D2 },
  { "D3",  (uint8_t)D3 },  { "D4",  (uint8_t)D4 },  { "D5",  (uint8_t)D5 },
  { "D6",  (uint8_t)D6 },  { "D7",  (uint8_t)D7 },  { "D8",  (uint8_t)D8 },
  { "D9",  (uint8_t)D9 },  { "D10", (uint8_t)D10 }, { "D15", (uint8_t)D15 },
};
const int PIN_NAME_COUNT = sizeof(PIN_NAMES) / sizeof(PIN_NAMES[0]);

bool pinIsValid(int p) {
  for (int i = 0; i < PIN_NAME_COUNT; i++) if (PIN_NAMES[i].num == p) return true;
  return false;
}

// First silkscreen name for a pin number, or "?" -- display only.
const char *pinNameOf(int p) {
  for (int i = 0; i < PIN_NAME_COUNT; i++) if (PIN_NAMES[i].num == p) return PIN_NAMES[i].name;
  return "?";
}

static inline bool boardIsV2()   { return cfg.hwRev <= HWREV_V2; }
static inline bool boardIsV3()   { return cfg.hwRev == HWREV_V3; }
static inline bool boardHasAmp() { return cfg.hwRev >= HWREV_V3B; }

// No SENSE_NEG pin (V3b) means there is nothing to read, whatever NEGFIX says.
static inline bool negFixed() { return cfg.negFix || !pinIsValid(SENSE_NEG); }

// ══════════════════════════════════════════════════════════════════
//  POWER GATES (HWREV 4)
// ══════════════════════════════════════════════════════════════════
// Two TPS22914 load switches on V3b: ANA feeds the analog front end (the TL431
// reference and through it the whole sense node), LED feeds LED1.  On V2/V3
// neither exists and every function here is a no-op -- gatePin[] is PIN_NONE.
// See the Config struct for what mode/settle mean.
enum { GATE_ANA = 0, GATE_LED = 1, GATE_COUNT = 2 };
const char *const GATE_NAME[GATE_COUNT] = { "ANA", "LED" };
uint8_t *const GATE_MODE[GATE_COUNT] = { &cfg.gAnaMode, &cfg.gLedMode };
int     gatePin[GATE_COUNT] = { PIN_NONE, PIN_NONE };  // set by applyBoardPins()
uint8_t gatePol[GATE_COUNT] = { 1, 1 };                // 1 = HIGH enables (TPS22914 ON)

// -1 = follow the configured MODE; 0/1 = held by !GATE.  A hold is deliberately
// NOT config: it is the knob you turn while watching an ammeter, and it must
// not survive a power cycle into a unit someone later thinks is stock.
int8_t gateForce[GATE_COUNT] = { -1, -1 };
bool   gateOn[GATE_COUNT]    = { false, false };
bool   gatesParked           = false;   // context of the last gatesRest()
bool   ledGateByPixel        = false;   // the LED gate is up only because a colour is showing

static inline bool gatePresent(uint8_t i) { return pinIsValid(gatePin[i]); }

// ANA carries a microsecond settle, LED a millisecond one.  One accessor so
// the callers do not care which.
uint32_t gateSettleUs(uint8_t i) {
  if (i == GATE_LED) return (uint32_t)cfg.gLedSettleMs * 1000UL;
  return cfg.gAnaSettleUs;
}

// delayMicroseconds() is a busy loop, which is the wrong thing to burn a
// multi-millisecond settle on when the whole point is saving current.
// Anything over a millisecond goes through sleepMs() (WFI) instead.
void gateDelayUs(uint32_t us) {
  if (us == 0) return;
  if (us >= 1000UL) { sleepMs(us / 1000UL); us %= 1000UL; }
  if (us) delayMicroseconds((unsigned int)us);
}

void gateWrite(uint8_t i, bool on) {
  gateOn[i] = on;
  if (i == GATE_LED) ledGateByPixel = false;   // whoever writes it now owns it
  if (!gatePresent(i)) return;                 // no such rail: bookkeeping only
  digitalWrite(gatePin[i], (on == (gatePol[i] != 0)) ? HIGH : LOW);
}

// Put every gate into its resting state for the current run context.
// `parked` = the board is asleep or in !FLOOR, which is the only thing MODE 1
// distinguishes.  A !GATE hold overrides the mode entirely.
void gatesRest(bool parked) {
  gatesParked = parked;
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    if (gateForce[i] >= 0) { gateWrite(i, gateForce[i] != 0); continue; }
    uint8_t m = *GATE_MODE[i];
    gateWrite(i, (m == 0) || (m == 1 && !parked));
  }
}

// The measurement side.  Only ANA takes part: LED follows the pixel (see
// setPixel), not the ADC.  In mode 2 this is where the measurement
// perturbation lives -- rail loading moves the resting differential by more
// than refBandV -- so if a mode-2 metric disagrees with a mode-0 one, raise
// ANAUS before suspecting the detection method.
//
// gatesPendingMask() says what gatesMeasureRaise() WOULD raise without
// touching anything, so runCapture() can lay out its timeline first and then
// put the rail's rise INSIDE the captured window -- watching that edge is how
// ANAUS gets tuned.
uint8_t gatesPendingMask(uint32_t *settleUsOut) {
  uint8_t pending = 0;
  uint32_t settle = 0;
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    if (i == GATE_LED)        continue;
    if (gateForce[i] >= 0)    continue;       // held by !GATE
    if (gateOn[i])            continue;       // already up
    if (!gatePresent(i))      continue;       // not on this board
    pending |= (uint8_t)(1 << i);
    if (gateSettleUs(i) > settle) settle = gateSettleUs(i);
  }
  if (settleUsOut) *settleUsOut = settle;
  return pending;
}

uint8_t gatesMeasureRaise(uint32_t *settleUsOut) {
  uint8_t raised = gatesPendingMask(settleUsOut);
  for (uint8_t i = 0; i < GATE_COUNT; i++)
    if (raised & (1 << i)) gateWrite(i, true);
  return raised;
}

uint8_t gatesMeasureBegin() {
  uint32_t settle = 0;
  uint8_t raised = gatesMeasureRaise(&settle);
  if (raised) gateDelayUs(settle);
  return raised;
}

void gatesMeasureEnd(uint8_t raised) {
  for (uint8_t i = 0; i < GATE_COUNT; i++)
    if (raised & (1 << i)) gateWrite(i, false);
}

// ── Power-profile experiment (!EXPT) ──────────────────────────────
// Ten steps of equal length, each a different gating scheme, so one PPK2
// capture contains every case with the boundaries at known times.  The point
// is the DELTAS between steps, exactly as with !FLOOR.
//
// ana/led: 0 = force the gate off, 1 = force it on, -1 = leave it to its
// configured MODE.  The `run` steps sweep the two gates while the board runs
// normally; the `park` steps repeat the sweep asleep; the `deep` steps repeat
// the best and worst parked cases in stage 2.  On a board without gates
// (V2/V3) the sweeps are all the same step, which is still a valid run --
// it just measures awake / asleep / deep.
struct ExptStep {
  const char *name; uint8_t parked; uint8_t deep;
  int8_t ana; int8_t led;
};
const ExptStep EXPT_STEPS[] = {
  { "run-baseline",  0, 0, -1, -1 },
  { "run-led-off",   0, 0, -1,  0 },
  { "run-ana-off",   0, 0,  0, -1 },
  { "run-both-off",  0, 0,  0,  0 },
  { "park-baseline", 1, 0,  1,  1 },
  { "park-led-off",  1, 0,  1,  0 },
  { "park-ana-off",  1, 0,  0,  1 },
  { "park-both-off", 1, 0,  0,  0 },
  { "deep-baseline", 1, 1,  1,  1 },
  { "deep-both-off", 1, 1,  0,  0 },
};
const int EXPT_STEP_COUNT = sizeof(EXPT_STEPS) / sizeof(EXPT_STEPS[0]);

int           exptStep        = -1;    // -1 = not running
uint16_t      exptStepSec     = 10;
unsigned long exptStepStartMs = 0;
uint32_t      exptParkTicks   = 0;

// ── Applying the board's pin map ──────────────────────────────────
// Assigns every pin variable from cfg.hwRev and configures the hardware.
// Called at boot after the config load, and again after anything that can
// change HWREV (!SET,HWREV / !LOAD / !DEFAULTS).  Pins the previous map used
// and this one does not are returned to INPUT first, so a revision change at
// runtime never leaves a push-pull output driving what is now something else.
static bool pinsApplied = false;

void applyBoardPins() {
  buzzerStop();                          // nothing may be toggling a pin whose role changes

  const int PIN_SLOTS = 15;
  int prev[PIN_SLOTS] = { SENSE_POS, SENSE_NEG, CHARGE_PIN, MOSFET_PIN, LED_PIN,
                          SPEAKER_PIN, SPEAKER_PIN_B, DIP_PIN_A, DIP_PIN_B, PARK_PIN,
                          AMP_DIN_PIN, AMP_EN1_PIN, AMP_EN2_PIN,
                          gatePin[GATE_ANA], gatePin[GATE_LED] };

  // Common to every revision.
  SENSE_POS     = A2;
  MOSFET_PIN    = D7;
  LED_PIN       = 6;                     // D6
  SENSE_NEG     = CHARGE_PIN  = PIN_NONE;
  SPEAKER_PIN   = SPEAKER_PIN_B = PIN_NONE;
  DIP_PIN_A     = DIP_PIN_B   = PARK_PIN = PIN_NONE;
  AMP_DIN_PIN   = AMP_EN1_PIN = AMP_EN2_PIN = PIN_NONE;
  gatePin[GATE_ANA] = gatePin[GATE_LED] = PIN_NONE;

  if (boardIsV2()) {
    SENSE_NEG   = A1;
    CHARGE_PIN  = A3;
    SPEAKER_PIN = D9;                    // other leg of the piezo is ground
    DIP_PIN_A   = D8;                    // threshold DIP switches (PCB pull-ups)
    DIP_PIN_B   = D10;
  } else if (boardIsV3()) {
    SENSE_NEG     = A1;                  // not connected -- NEGFIX covers it
    CHARGE_PIN    = A3;
    SPEAKER_PIN   = D9;                  // BZ1- via R17
    SPEAKER_PIN_B = D8;                  // BZ1+ via R16
    PARK_PIN      = D10;                 // no-connect, parked low
  } else {                               // V3b
    CHARGE_PIN        = A1;              // VBUS/2 moved here from A3
    AMP_DIN_PIN       = D15;             // PAM8904 DIN (back pad P101 -> J7)
    AMP_EN1_PIN       = D8;              // PAM8904 EN1
    AMP_EN2_PIN       = D9;              // PAM8904 EN2
    gatePin[GATE_ANA] = A3;              // Analog_Rail, TPS22914 U1
    gatePin[GATE_LED] = D10;             // LEDRail,     TPS22914 U4
    gatePol[GATE_ANA] = gatePol[GATE_LED] = 1;   // ON pin: HIGH = rail up
  }

  int now[PIN_SLOTS] = { SENSE_POS, SENSE_NEG, CHARGE_PIN, MOSFET_PIN, LED_PIN,
                         SPEAKER_PIN, SPEAKER_PIN_B, DIP_PIN_A, DIP_PIN_B, PARK_PIN,
                         AMP_DIN_PIN, AMP_EN1_PIN, AMP_EN2_PIN,
                         gatePin[GATE_ANA], gatePin[GATE_LED] };
  if (pinsApplied) {
    for (int i = 0; i < PIN_SLOTS; i++) {
      if (!pinIsValid(prev[i])) continue;
      bool stillUsed = false;
      for (int j = 0; j < PIN_SLOTS; j++) if (now[j] == prev[i]) stillUsed = true;
      if (!stillUsed) pinMode(prev[i], INPUT);
    }
  }
  pinsApplied = true;

  pinMode(SENSE_POS, INPUT);
  if (pinIsValid(SENSE_NEG)) pinMode(SENSE_NEG, INPUT);
  pinMode(CHARGE_PIN, INPUT);
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state
  pixel.setPin(LED_PIN);

  // Gate pins go straight to their resting level: a load switch whose ON pin
  // floats between here and the first gatesRest() has an undefined rail.
  for (uint8_t i = 0; i < GATE_COUNT; i++)
    if (gatePresent(i)) pinMode(gatePin[i], OUTPUT);
  gatesRest(gatesParked);

  // Speaker / amp / DIP / park pins belong to buzzerApplyPinModes(), the single
  // place that decides what D8/D9/D10 are.  Call it last so it wins.
  buzzerApplyPinModes();
}

enum LeadState { STATE_FLOAT, STATE_CLOSED, STATE_VOLTAGE };
LeadState leadState = STATE_VOLTAGE;

// Explicit prototypes (see note at the ConfigField struct).
LeadState runMosfetTest();
LeadState runMosfetTestStable();
LeadState lowPowerProbe();
void      slogRecord(LeadState s, bool awake);

// Sub-classification of STATE_VOLTAGE (ported from production).  leadState
// still means only "voltage is present", and every detection, debounce, sleep
// and logging path keys off that alone -- this decides nothing except which
// alert pattern is played.  It defaults to VKIND_DCP, whose LED sequence and
// beep are byte-for-byte the plain voltage alert, so any path that reaches the
// voltage alert without a fresh classification behaves exactly as before.
enum VoltKind { VKIND_DCP, VKIND_DCN, VKIND_AC };
VoltKind voltKind = VKIND_DCP;
// False means voltKind does NOT describe whatever is on the leads right now,
// so the next classification is adopted outright instead of being debounced.
// Cleared whenever the voltage goes away, and on a wake from low power -- the
// sleeping probe decides only that voltage is PRESENT and never classifies it.
bool voltKindValid = false;
// Raw (per-pass, undebounced) kind of the last classification, for $DET.
VoltKind voltKindRaw = VKIND_DCP;
bool     voltKindRawFresh = false;   // a classification ran this pass

// LED colour sequence per kind.  One flash shows one step and the step advances
// per flash, at the existing ledVoltMs / ledVoltPerMs cadence.  Each value is a
// colour CHANNEL (0 = red, 1 = green, 2 = blue) lit at cfg.ledVoltBright.  Red
// leads every sequence, because "voltage" has to read at the first flash.
const uint8_t VKIND_SEQ[3][3] = {
  { 0, 0, 0 },        // VKIND_DCP: red, red
  { 0, 2, 0 },        // VKIND_DCN: red, blue
  { 0, 2, 1 },        // VKIND_AC : red, blue, green
};
const uint8_t VKIND_SEQ_LEN[3] = { 2, 2, 3 };
uint8_t voltSeqIdx = 0;         // next step of the active sequence

VoltKind    classifyVoltage();
const char *voltKindName(VoltKind k);

// Active open/closed threshold, refreshed from the DIP switches + cfg.thresh
// table every detection pass.
float   activeThreshV = 0.5f;
uint8_t dipIdx        = 3;      // last-read DIP position (0..3)

// Non-blocking blink state
bool floatFlashing = false, closedFlashing = false, voltFlashing = false;
unsigned long lastFloatFlash = 0, lastClosedFlash = 0, lastVoltFlash = 0;

// Charge / USB-power lockout
bool chargeActive  = false;   // VBUS sense above threshold (USB in)
bool alertOverride = false;   // user re-enabled normal alerts while charging

// "is charge detection allowed to stop us doing things right now".
// Every place that used to test chargeActive as a REASON TO INHIBIT tests this
// instead; the places that merely report or track the rail still use
// chargeActive directly, so CHGINHIBIT=0 loses the inhibits without losing the
// telemetry.  See cfg.chargeInhibit for why that split is the useful one.
static inline bool chargeInhibits() {
  return chargeActive && cfg.chargeInhibit;
}
bool chargeBlinkOn = false;
unsigned long lastChargeBlink = 0;
const unsigned long CHARGE_BLINK_PERIOD_MS = 2000;
const unsigned long CHARGE_BLINK_ON_MS     = 250;
const uint8_t       CHARGE_BLINK_BRIGHT    = 25;

// Power-on battery cue.  Everything here is dead time between flipping the
// switch and being able to measure, so it is kept as short as still reads as a
// countable blink.  The pre-gap doubles as the BAT_READ_EN divider settle
// (see setup(), which enables it first thing), so that settle costs nothing.
const unsigned long BOOT_CUE_PREGAP_MS = 120;
const unsigned long BOOT_CUE_ON_MS     = 90;
const unsigned long BOOT_CUE_OFF_MS    = 90;

// VBUS test used before the config is loaded, so it cannot use cfg.chargeThreshV.
const float BOOT_USB_THRESH_V = 2.0f;

// Battery monitor
float battV   = 0.0f;
int   battPct = 0;

// Beep-sequence state
bool          beepOn           = false;
int           beepPulsesLeft   = 0;
unsigned long beepPhaseStart   = 0;
unsigned long lastBeepSeqStart = 0;
unsigned long beepOnMs = 20, beepOffMs = 10;
// On-time of the FINAL pulse of a sequence; 0 = every pulse uses beepOnMs.
// This is the whole of what makes VDC- a short-LONG beep while VDC+ stays
// short-short -- the voltage alerts differ in rhythm, not pitch.
unsigned long beepLastOnMs = 0;
unsigned int  beepFreq = 0;
uint8_t       beepVol  = 3;      // volume key of the sequence in progress
bool speakerMuted = false;    // session mute (leads closed at boot)

// Debug values
float lastRestV = 0.0f, lastTestV = 0.0f;
float lastMetric = 0.0f;      // scalar actually compared to the threshold
float lastReturnMs = 0.0f;    // method 1 result (ms), or window on timeout
float lastAreaVms  = 0.0f;    // method 2 result (V*ms)
// Voltage classifier, for $STATUS / !VTEST / $DET / the debug line.  Peaks are
// signed excursions from cfg.refCenterV, each measured as a positive magnitude
// on its own side, so both being large is what "AC" means.
float lastVPosPeakV = 0.0f;   // largest excursion above the resting centre
float lastVNegPeakV = 0.0f;   // largest excursion below it
int   lastVSamples  = 0;      // reads taken in the last classification window

// !DETLOG -- one $DET line per detection pass, for the automated test
// suite (TestBench/BlinkyHawkTestSuite).  RAM only, off at boot.  While on,
// voltagePresent() takes all VOLTAVG reads instead of returning at the first
// fast-band trip, so every pass reports the full mean/min/max of its resting
// reads and the host can re-evaluate the voltage decision offline for any
// REFCENTER/REFBAND/VOLTFAST.  The DECISION is unchanged -- "any read beyond
// the fast band, else the average beyond the band" -- only a voltage-present
// pass runs up to VOLTAVG-1 reads longer, and on such a pass the MOSFET test
// is skipped anyway, so the open/closed metric is not disturbed.
bool    detLogOn    = false;
LeadState lastRawState = STATE_FLOAT;  // runDetection()'s undebounced result
uint8_t detRestN    = 0;       // resting reads taken this pass
float   detRestMean = 0.0f, detRestMin = 0.0f, detRestMax = 0.0f;
char    detVoltPath = '-';     // F fast trip, A averaged trip, - none, L locked on, D disabled
bool    detTestRan  = false;   // the open/closed test ran this pass (metric is fresh)
unsigned long lastSerialTime = 0;
const unsigned long serialInterval = 250;

// Diagnostic mode
bool diagMode = false;
bool streamOn = false;
unsigned long streamIntervalMs = 20;
unsigned long lastStreamMs = 0;
enum VoltOverride { VOLT_AUTO, VOLT_FORCE_ON, VOLT_DISABLED };
VoltOverride voltOverride = VOLT_AUTO;
int mosfetHold = -1;          // -1 auto, 0 hold off, 1 hold on

// Transient capture buffer (raw counts; volts computed by the host)
const int CAP_MAX_SAMPLES = 600;
const unsigned long CAP_PRE_US = 500;
// Rail-down baseline held at the start of a gated capture, so the step where
// the analog rail comes up has something to step away from.
const unsigned long CAP_GATE_US = 200;
unsigned long capDurationMs = 5;
uint32_t capT[CAP_MAX_SAMPLES];
uint16_t capPos[CAP_MAX_SAMPLES];
uint16_t capNeg[CAP_MAX_SAMPLES];
int capCount = 0;

char cmdBuf[64];
int  cmdLen = 0;

// ══════════════════════════════════════════════════════════════════
//  LOW-POWER IDLE
// ══════════════════════════════════════════════════════════════════
// The 1 kHz AGT tick that drives millis() interrupts the core every ms, so a
// plain WFI loop idles between ticks (CPU clock gated, peripherals running).
void sleepMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    __WFI();
  }
}

// ══════════════════════════════════════════════════════════════════
//  LOW-POWER TIMEOUT MODE
// ══════════════════════════════════════════════════════════════════
// After cfg.idleTimeoutS seconds in which the leads have only ever read OPEN,
// the board stops running the detection loop, parks every load it can switch
// off (LED rail, speaker, battery-sense divider, optionally the bridge) and
// enters Software Standby -- CPU and all peripheral clocks stopped, RAM
// retained.  It wakes on the RTC periodic interrupt, runs one cheap probe, and
// either sleeps again (still open) or returns to full-rate operation (closed
// leads or voltage present).
//
// The energy cost of the mode is (wake current x awake time) / tick period, so
// the knobs that matter are how often it probes (cfg.sleepPollTicks) and how
// much each probe does (cfg.sleepVoltAvg) -- both are EEPROM config so a unit
// can be characterised on the bench without a reflash.
#include "RTC.h"
#include "r_lpm.h"

// The RTC periodic interrupt is the wake source.  Its period is not free-form --
// the library exposes a fixed ladder of 2 s down to 1/256 s -- so cfg.sleepTickMs
// is snapped to the nearest rung.  2 s is the library's maximum and the cheapest.
// Worst-case detection latency = cfg.sleepTickMs * cfg.sleepPollTicks.
//
// The whole ladder is exposed because the useful operating point is not obvious
// from the datasheet: sleeping costs ~1.4 mA average against ~11.5 mA awake, so
// a fast tick paired with a short SLEEPSEC (sleep almost immediately, poll
// quickly) can be both more responsive AND cheaper than staying awake.  Where
// that trade stops paying is a bench question, hence the rungs.
//
// The sub-125 ms rungs are not whole milliseconds (1/16 s = 62.5 ms, 1/256 s =
// 3.90625 ms); the ms column is the rounded value, since cfg.sleepTickMs is a
// uint16_t of milliseconds.  Only the derived latency/pacing arithmetic uses it,
// and it is off by at most ~2.5% -- the interrupt itself runs at the exact rate.
// Below roughly 30 ms the probe and the standby wake overhead dominate the tick,
// so the board stops idling between wakes and average current climbs toward the
// awake figure: those rungs are for measuring that knee, not for shipping.
struct SleepTickOption { uint16_t ms; Period period; };
const SleepTickOption SLEEP_TICK_OPTIONS[] = {
  { 2000, Period::ONCE_EVERY_2_SEC     },
  { 1000, Period::ONCE_EVERY_1_SEC     },
  {  500, Period::N2_TIMES_EVERY_SEC   },
  {  250, Period::N4_TIMES_EVERY_SEC   },
  {  125, Period::N8_TIMES_EVERY_SEC   },
  {   63, Period::N16_TIMES_EVERY_SEC  },   // 62.5 ms
  {   31, Period::N32_TIMES_EVERY_SEC  },   // 31.25 ms
  {   16, Period::N64_TIMES_EVERY_SEC  },   // 15.625 ms
  {    8, Period::N128_TIMES_EVERY_SEC },   // 7.8125 ms
  {    4, Period::N256_TIMES_EVERY_SEC },   // 3.90625 ms
};
const int SLEEP_TICK_OPTION_COUNT =
    sizeof(SLEEP_TICK_OPTIONS) / sizeof(SLEEP_TICK_OPTIONS[0]);
const unsigned long SLEEP_HB_MS   = 6;    // heartbeat flash on-time
const uint8_t SLEEP_HB_BRIGHT     = 12;   // heartbeat flash brightness (dim blue)
const unsigned long SLEEP_BOOT_GRACE_MS = 8000;   // never sleep this soon after boot

bool          lowPowerActive = false;  // currently parked + sleeping
bool          sleepArmed     = false;  // !SLEEP: sleep as soon as it is allowed
unsigned long idleSinceMs    = 0;      // millis() of the last non-open activity
unsigned int  sleepTickCount = 0;      // ticks since the last probe
unsigned int  sleepHbCount   = 0;      // ticks since the last heartbeat

// ── Two-stage sleep ───────────────────────────────────────────────
// 0 = awake, 1 = the normal sleeping mode, 2 = the deep stage.  Stage 2 is
// reached only from stage 1 and only through deepSec ticks of open leads;
// anything that wakes the board drops it straight back to 0, so the next sleep
// always starts at stage 1.
uint8_t       sleepStage      = 0;
unsigned long stageTickCount  = 0;     // ticks spent in stage 1 (for deepSec)
uint16_t      deepTickMs      = 1000;  // stage-2 RTC period, derived from deepHz
uint16_t      deepPollTicks   = 1;     // stage-2 ticks per probe
bool          deepForced      = false; // !DEEP,1 / an experiment step put us here

// How long one wake period currently is.  Everything that reasons in ticks --
// the deepSec countdown, the experiment sequencer's parked steps, the WFI
// fallback -- has to ask, because the period changes with the stage.
static inline uint16_t currentTickMs() {
  return (sleepStage == 2) ? deepTickMs : cfg.sleepTickMs;
}

// Should the bridge MOSFET be parked OFF right now?  Stage 2 can answer this
// differently from stage 1 (cfg.deepParkOff), which is the point -- the bridge
// resting is a continuous draw through the 100K sense leg, and a stage that has
// already given up wake rate is the natural place to give up the bridge too.
// deepParkOff == 2 means "no opinion, do whatever stage 1 does".
static inline bool bridgeParkOff() {
  if (sleepStage == 2 && cfg.deepParkOff <= 1) return cfg.deepParkOff != 0;
  return cfg.sleepParkOff != 0;
}

// Put the bridge where the current stage wants it.  Called on every stage
// change, because the answer can differ between stages and nothing else in the
// sleeping loop touches that pin between probes.
static inline void applyBridgePark() {
  digitalWrite(MOSFET_PIN, bridgeParkOff() ? MOSFET_OFF : MOSFET_ON);
}

// Current-measurement parking (!FLOOR).  Holds the board in one fixed state so
// a series ammeter reads a stable number:
//   1 = parked, bridge resting (ON), Software Standby
//   2 = parked, bridge OFF, Software Standby  -> delta vs 1 = the bridge leg
//   3 = parked, bridge resting (ON), WFI only -> delta vs 1 = what Standby buys
int floorMode = 0;

// ── Offline probe log ─────────────────────────────────────────────
// The sleeping probe runs with USB unplugged and so cannot print, which makes
// "it won't wake when I'm off USB" impossible to diagnose from the bench: the
// board behaves differently precisely when you cannot see it.  Every probe
// therefore records its decision to RAM, and !SLEEPLOG dumps the history once
// the unit is plugged back in.  Ring buffer -- keeps the most recent probes,
// which are the ones next to the behaviour you just observed.
//
// Each entry carries everything a $DET line does, because the battery is the
// ONLY condition that matters for tuning and $DET cannot be printed there: USB
// earths the board through the host, which moves the resting differential by
// ~35 mV and slows the recovery tail (measured Sep 2026 -- thresholds tuned on
// USB read every closed lead as open on battery), and an earthed signal
// generator on the leads becomes a ground loop.  !SLEEPLOG,D dumps the detail
// ($SLOGD rows, same fields as $DET plus the source); !SLEEPLOG,2 clears the
// log and makes voltagePresent() take all VOLTAVG reads, as !DETLOG does, so
// the rest mean/min/max is complete and the voltage decision can be replayed
// offline.  192 entries x 52 bytes = 10 KB of RAM -- the most that fits beside
// the core's fixed 8 KB heap (240 overflowed into the stack at link time).
const int SLOG_MAX = 192;
struct SleepLogEntry {
  uint32_t ms;         // millis() when recorded (frozen while in Standby)
  float   metric;      // what was compared against the threshold
  float   retms;       // method 1/2 recovery time (ms)
  float   area;        // method 2 tail area (V*ms)
  float   rest;        // resting differential at the probe (V)
  float   thr;         // threshold in force (the selector stays live while asleep)
  float   rmean, rmin, rmax;   // resting reads this pass (valid when rn > 0)
  float   vpos, vneg;  // voltage-kind peaks (valid when rawkind != 255)
  uint8_t state;       // LeadState decided (debounced, for awake entries)
  uint8_t raw;         // this pass's raw LeadState (awake entries)
  uint8_t awake;       // 1 = sampled by the awake loop, 0 = by the sleeping probe
  uint8_t stage;       // sleepStage at the time: 0 awake, 1 light sleep, 2 deep
  uint8_t rn;          // resting reads taken this pass (0 = none recorded)
  uint8_t tested;      // 1 = the open/closed test ran (metric fields fresh)
  uint8_t rawkind;     // VoltKind classified this pass, 255 = none
  uint8_t kind;        // debounced VoltKind
};
bool slogFullReads = false;    // !SLEEPLOG,2: voltagePresent() takes every read
SleepLogEntry slog[SLOG_MAX];
int           slogValid = 0;   // entries currently held (<= SLOG_MAX)
int           slogHead  = 0;   // next write index
unsigned long slogTotal = 0;   // samples recorded since the last clear
unsigned long lastSlogAwakeMs = 0;   // pacing for awake-on-battery samples
float lastProbeThreshV = 0.0f;       // threshold the last sleeping probe used

// ⚠ SBYCR IS GLOBAL STATE.  R_LPM_Open() / R_LPM_LowPowerReconfigure() write the
// SBYCR register immediately from cfg.low_power_mode:
//     LPM_MODE_SLEEP   -> SBYCR = 0x4000 (SSBY=0)
//     LPM_MODE_STANDBY -> SBYCR = 0xC000 (SSBY=1)
// SSBY decides what EVERY __WFI() in the program does -- R_LPM_LowPowerModeEnter
// itself only executes "dsb; wfi", it does not set SSBY.  So opening the driver
// in STANDBY mode silently converts sleepMs()'s idle WFI into a full Software
// Standby: all peripheral clocks stop (USB dies mid-enumeration, "USB device not
// recognized") and millis() freezes, so sleepMs()'s millis() deadline never
// arrives and the board locks up until reset.
//
// Therefore: stay in SLEEP mode at all times, and switch to STANDBY only for the
// duration of a deliberate sleep in lowPowerTick(), switching straight back.
static lpm_instance_ctrl_t lpmCtrl;
static lpm_cfg_t           lpmSleepCfg;     // SSBY=0: plain WFI idle (the safe default)
static lpm_cfg_t           lpmStandbyCfg;   // SSBY=1: WFI enters Software Standby
static volatile bool       lpmRtcTick = false;
static bool                lpmReady   = false;

static void lpmRtcCallback() { lpmRtcTick = true; }

// Snap cfg.sleepTickMs to a rate the RTC can actually produce and program it.
// Writing the snapped value back means !CFG and the sleep log always describe
// the rate in force rather than what was asked for.  Re-programming is safe to
// repeat: IRQManager only allocates a vector slot the first time (it guards on
// periodic_irq == FSP_INVALID_VECTOR) and merely re-enables thereafter.
// Snap `ms` to the nearest rung and program the RTC for it.  Writes the snapped
// value through `snappedOut` so the caller can store what is actually in force.
// The programmed period is cached because re-programming the same rate is
// pointless work on every stage change.
static bool programTickPeriod(uint16_t ms, uint16_t *snappedOut) {
  const SleepTickOption *best = &SLEEP_TICK_OPTIONS[0];
  long bestDiff = 0x7FFFFFFF;
  for (int i = 0; i < SLEEP_TICK_OPTION_COUNT; i++) {
    long diff = (long)ms - (long)SLEEP_TICK_OPTIONS[i].ms;
    if (diff < 0) diff = -diff;
    if (diff < bestDiff) { bestDiff = diff; best = &SLEEP_TICK_OPTIONS[i]; }
  }
  if (snappedOut) *snappedOut = best->ms;

  static uint16_t programmedMs = 0;
  if (programmedMs == best->ms) return true;          // already in force
  if (!RTC.setPeriodicCallback(lpmRtcCallback, best->period)) return false;
  programmedMs = best->ms;
  return true;
}

bool applySleepTickPeriod() {
  return programTickPeriod(cfg.sleepTickMs, &cfg.sleepTickMs);
}

// Work out the stage-2 schedule from cfg.deepHz.
//
// The rule is "the LARGEST rung that is not longer than the target period,
// then however many of those fit" -- NOT the closest achievable rate.  Chasing
// the exact rate picks tiny rungs (a 3 Hz target lands on 4 ms x 83 ticks,
// i.e. 250 wakes a second to probe three times), which is the opposite of what
// a deep stage is for.  Waking as rarely as the ladder allows is the whole
// point; the achieved rate is written back to cfg.deepHz so !CFG reports what
// the board is really doing.
void computeDeepSchedule() {
  float hz = cfg.deepHz;
  if (hz < 0.001f) hz = 0.001f;
  unsigned long targetMs = (unsigned long)(1000.0f / hz + 0.5f);

  // Rungs are listed slowest first, so the first one that fits is the largest.
  const SleepTickOption *rung = &SLEEP_TICK_OPTIONS[SLEEP_TICK_OPTION_COUNT - 1];
  for (int i = 0; i < SLEEP_TICK_OPTION_COUNT; i++) {
    if (SLEEP_TICK_OPTIONS[i].ms <= targetMs) { rung = &SLEEP_TICK_OPTIONS[i]; break; }
  }
  // CEILING, not nearest: this is a power-saving stage, so landing slower than
  // asked for is a smaller sin than landing faster.  Nearest would turn a 3 Hz
  // request into 4 Hz (333 ms rounds down onto the 250 ms rung); ceiling makes
  // it 2 Hz.  Every rate the ladder can hit exactly still comes out exact.
  unsigned long ticks = (targetMs + rung->ms - 1) / rung->ms;
  if (ticks < 1)     ticks = 1;
  if (ticks > 65535) ticks = 65535;

  deepTickMs    = rung->ms;
  deepPollTicks = (uint16_t)ticks;
  cfg.deepHz    = 1000.0f / (float)((unsigned long)deepTickMs * ticks);
}

// Put the RTC on whichever period the current stage calls for.  Called after
// anything that can change either schedule (!SET, !LOAD, !DEFAULTS) so a config
// edit made while the board is asleep does not leave it on the wrong rate.
void refreshSleepSchedule() {
  computeDeepSchedule();
  if (sleepStage == 2) programTickPeriod(deepTickMs, NULL);
  else                 applySleepTickPeriod();
}

// Explicit prototypes: same reason as the ConfigField helpers above -- the
// Arduino preprocessor hoists its auto-generated ones too far up the file.
void enterLowPower();
void exitLowPower();
void serviceLowPower();
void serviceFloorMode();
void pollSerial();
void printDetLine(LeadState rawState);
static char stateChar(LeadState s);

// Bring up the RTC periodic interrupt and open the LPM driver.  Returns false
// if either fails, in which case lowPowerTick() degrades to a WFI idle -- the
// timeout mode still parks every load, it just leaves the core clocked.
bool lowPowerInit() {
  if (!RTC.begin()) return false;

  // The periodic interrupt only runs once the RTC counter is started, so seed a
  // nominal time if it isn't already running.  The value is irrelevant: nothing
  // reads the calendar, we only need the divider ticking.
  RTCTime seed(1, Month::JANUARY, 2025, 0, 0, 0,
               DayOfWeek::WEDNESDAY, SaveLight::SAVING_TIME_INACTIVE);
  RTC.setTimeIfNotRunning(seed);

  if (!applySleepTickPeriod()) return false;

  lpmStandbyCfg.low_power_mode       = LPM_MODE_STANDBY;
  lpmStandbyCfg.standby_wake_sources = LPM_STANDBY_WAKE_SOURCE_RTCPRD;
  lpmSleepCfg.low_power_mode         = LPM_MODE_SLEEP;
  lpmSleepCfg.standby_wake_sources   = 0;

  // Open in SLEEP mode: this leaves SSBY clear, so sleepMs()'s WFI keeps
  // behaving as an ordinary CPU idle.  Opening in STANDBY here would arm every
  // WFI in the program -- see the warning above.
  if (R_LPM_Open(&lpmCtrl, &lpmSleepCfg) != FSP_SUCCESS) return false;

  lpmReady = true;
  return true;
}

// Enter Software Standby until the RTC periodic interrupt fires.  Other enabled
// interrupts can also return from the WFI inside the driver, so this loops until
// the tick flag is actually set.
//
// NOTE: every peripheral clock stops in Standby, including the AGT behind
// millis() -- so millis() does NOT advance while asleep.  Nothing in this mode
// depends on it (ticks are counted, not timed) and exitLowPower() re-bases
// idleSinceMs on wake, but keep it in mind before adding millis() logic here.
void lowPowerTick() {
  if (!lpmReady) { sleepMs(currentTickMs()); return; }

  // Arm Software Standby (SSBY=1) only for this sleep, and disarm it again
  // immediately afterwards so no other WFI in the program can trip into it.
  R_LPM_LowPowerReconfigure(&lpmCtrl, &lpmStandbyCfg);
  lpmRtcTick = false;
  while (!lpmRtcTick) R_LPM_LowPowerModeEnter(&lpmCtrl);
  R_LPM_LowPowerReconfigure(&lpmCtrl, &lpmSleepCfg);
}

// ── Load parking ──────────────────────────────────────────────────
// The onboard RGB draws its own quiescent current whenever its rail is up, even
// showing black, so blanking the pixel is not enough -- RGB_POWER_PIN has to go.
void setPixel(uint8_t r, uint8_t g, uint8_t b);

static void ledPower(bool on) {
  digitalWrite(RGB_POWER_PIN, on ? HIGH : LOW);
  if (on) {
    sleepMs(1);                 // let the rail come up before clocking data out
    setPixel(0, 0, 0);          // not pixel.show(): on V3b LED1's rail may be down
  }
}

// The onboard Vbatt/2 sense divider is gated by BAT_READ_EN, which setup()
// drives HIGH and normal operation never lowers -- so it draws continuously.
// There is no reason to keep it enabled while asleep.
static void battSense(bool on) {
  digitalWrite(BATT_EN_PIN, on ? HIGH : LOW);
  // NOTE: the divider needs time to settle after re-enabling (see
  // startupBatteryIndicate), so the first battV reading after a wake reads low.
}

// Drop from stage 1 into stage 2: slower RTC period, fewer probes.  Everything
// else about the parked state is already correct -- stage 2 differs from stage 1
// only in how often the board wakes, not in what is switched off.
void enterDeepSleep() {
  sleepStage     = 2;
  sleepTickCount = 0;
  sleepHbCount   = 0;
  computeDeepSchedule();
  programTickPeriod(deepTickMs, NULL);
  applyBridgePark();             // DEEPPARK may differ from SLEEPPARK
}

// Back up to stage 1 without fully waking (used by !DEEP,0 and by the
// experiment sequencer between steps).
void exitDeepSleep() {
  sleepStage     = 1;
  sleepTickCount = 0;
  sleepHbCount   = 0;
  stageTickCount = 0;
  deepForced     = false;
  applySleepTickPeriod();
  applyBridgePark();             // back to whatever SLEEPPARK says
}

// Park every switchable load and mark the mode active.
void enterLowPower() {
  lowPowerActive = true;
  sleepStage     = 1;            // every sleep starts at stage 1
  sleepTickCount = 0;
  sleepHbCount   = 0;
  stageTickCount = 0;
  deepForced     = false;
  applySleepTickPeriod();        // pick up any change made via !SET / !LOAD

  silenceSpeaker();
  speakerOff();                 // park the pin idle, not merely stop the sequence
  setPixel(0, 0, 0);
  ledPower(false);
  battSense(false);
  gatesRest(true);                       // mode-1 gates drop here
  applyBridgePark();
}

// Restore everything and hand control back to the normal loop.
void exitLowPower() {
  lowPowerActive = false;
  // Leaving stage 2 has to put the RTC back on the stage-1 rung, or the next
  // sleep would silently inherit the deep period.
  bool wasDeep = (sleepStage == 2);
  sleepStage   = 0;
  deepForced   = false;
  if (wasDeep) applySleepTickPeriod();
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state
  battSense(true);
  gatesRest(false);                      // back to the awake resting state
  ledPower(true);

  // millis() froze while we were in Standby, so the idle timer has to be
  // re-based here rather than carried across the sleep.
  idleSinceMs = millis();

  // The probe that woke us can report STATE_VOLTAGE without having classified
  // it, so whatever voltKind holds is the PREVIOUS contact's.  Mark it stale
  // and let the first awake detection pass adopt the real one.
  voltKindValid = false;
}

// One probe pass: is anything still there?  Runs the real runMosfetTest() so
// every detection method (SINGLE / TIMERET / AREA) and the live DIP threshold
// behave exactly as they do awake -- only the averaging is cut, and the test's
// trailing settle is suppressed because the pin is about to be parked anyway.
// Returns STATE_FLOAT if the leads are still open.
LeadState lowPowerProbe() {
  // The selector stays live while asleep (hwRev 2: the DIP pins are still read;
  // hwRev 3+: cfg.threshSel is just a memory read), but refresh the threshold
  // WITHOUT updateThresholdFromDip(): that announces changes on serial, and a
  // CDC write with USB unplugged (which it always is here) can block.
  dipIdx        = readDipIndex();
  activeThreshV = cfg.thresh[dipIdx];

  // Wake threshold, if one is configured for this DIP position.  Kept separate
  // from thresh[] so the continuity threshold can be tuned to a target
  // resistance without that value having to double as "is anything connected".
  // Restored before returning -- runMosfetTest() reads activeThreshV, and
  // !SLEEPTEST runs this probe while awake, where the awake value must survive.
  float savedThresh = activeThreshV;
  if (cfg.sleepThresh[dipIdx] > 0.0f) activeThreshV = cfg.sleepThresh[dipIdx];

  // Measure under the SAME rail conditions the awake detector sees.  The LED
  // rail and the battery-sense divider are both loads on 3.3 V, and on a
  // floating (battery) supply their current shifts the analog baseline -- the
  // resting differential moves by ~26 mV between rails-up and rails-down, which
  // is more than refBandV.  Probing with them parked off makes the probe's
  // metric incomparable to the awake one, so no single threshold can serve both.
  // The LED is powered but written black, so nothing is visible; a few ms of
  // rail per probe costs well under 5 uA averaged.
  digitalWrite(RGB_POWER_PIN, HIGH);
  digitalWrite(BATT_EN_PIN, HIGH);
  // Same argument, extended to the V3b gates: whatever the analog front end
  // needs comes up here and is dropped again on the way out.  (LED1's own rail
  // follows the pixel and stays down, exactly as it is between awake flashes.)
  uint8_t gatesRaised = gatesMeasureBegin();
  sleepMs(1);                                 // let the rail come up
  setPixel(0, 0, 0);                          // hold it dark, not whatever it powered up as

  // Settle at the resting state before testing.  This is NOT optional padding:
  // detection methods 1 and 2 measure the recovery transient from the toggle,
  // and sampleRecovery() only accepts a "return" after the deviation has first
  // dipped past DET_TROUGH_MIN_V.  From an unsettled baseline that trough never
  // registers, returnMs falls back to the detWindowUs timeout, the metric lands
  // above the threshold and a CLOSED lead is misread as FLOAT -- i.e. continuity
  // silently fails to wake the board.  The awake path gets this settle for free
  // from voltagePresent()'s ten reads; the probe has to do it explicitly, and
  // needs it more, having just come out of standby with the ADC clock restarted.
  digitalWrite(MOSFET_PIN, MOSFET_ON);        // bridge to resting for the test
  sleepMs(cfg.settlePostMs);                  // rest at baseline (CPU idles)
  readVoltage();                              // throwaway: flush the ADC after the wake

  uint8_t savedPost = cfg.settlePostMs;
  cfg.settlePostMs  = 0;                      // no point settling before parking

  // voltagePresentSleep() (not voltagePresent()) -- the wide bypass band only,
  // so lead noise can't wake the board every tick.  It reads sleepVoltAvg
  // samples itself, so voltAvgSamples is left alone here.
  bool present = (voltOverride == VOLT_DISABLED) ? false : voltagePresentSleep();
  LeadState result = present ? STATE_VOLTAGE : runMosfetTest();

  cfg.settlePostMs  = savedPost;
  lastProbeThreshV  = activeThreshV;    // what the decision above was made against
  activeThreshV     = savedThresh;      // hand the awake value back

  applyBridgePark();                    // stage 2 may park differently

  gatesMeasureEnd(gatesRaised);         // drop whatever this probe raised

  // Park the rails again, but only if we are actually asleep -- !SLEEPTEST runs
  // this same probe while awake and must not switch the LED off underneath the
  // running alert logic.
  if (lowPowerActive) {
    digitalWrite(RGB_POWER_PIN, LOW);
    digitalWrite(BATT_EN_PIN, LOW);
  }
  return result;
}

// Record one measurement.  Called from the sleeping loop, so it must not print.
// `awake` distinguishes samples taken by the normal loop (rails up) from those
// taken by the sleeping probe -- comparing the two on battery is the whole
// point, since that is where the baseline shift shows up.
void slogRecord(LeadState s, bool awake) {
  // A !SLEEPLOG,2 capture is a scripted run with a known start: keep the start
  // and stop when full, rather than letting the time it takes to replug USB
  // overwrite the run with idle samples.
  if (slogFullReads && slogValid >= SLOG_MAX) return;
  SleepLogEntry &e = slog[slogHead];
  e.ms      = millis();
  e.metric  = lastMetric;
  e.retms   = lastReturnMs;
  e.area    = lastAreaVms;
  e.rest    = lastRestV;
  e.thr     = awake ? activeThreshV : lastProbeThreshV;
  e.state   = (uint8_t)s;
  e.raw     = awake ? (uint8_t)lastRawState : (uint8_t)s;
  e.awake   = awake ? 1 : 0;
  e.stage   = awake ? 0 : sleepStage;
  // The sleeping probe keeps no rest statistics (voltagePresentSleep), so only
  // awake entries carry them.
  e.rn      = awake ? detRestN : 0;
  e.rmean   = detRestMean;  e.rmin = detRestMin;  e.rmax = detRestMax;
  e.tested  = awake ? (detTestRan ? 1 : 0) : (s != STATE_VOLTAGE ? 1 : 0);
  e.rawkind = (awake && voltKindRawFresh) ? (uint8_t)voltKindRaw : 255;
  e.kind    = (uint8_t)voltKind;
  e.vpos    = lastVPosPeakV;  e.vneg = lastVNegPeakV;
  slogHead = (slogHead + 1) % SLOG_MAX;
  if (slogValid < SLOG_MAX) slogValid++;
  slogTotal++;
}

// Detailed dump, oldest first:
//   $SLOGD,<i>,<ms>,<src>,<raw>,<lead>,<n>,<mean>,<min>,<max>,<metric>,<retms>,
//          <area>,<thr>,<rawkind>,<kind>,<vpos>,<vneg>
// src = A awake / S light sleep / D deep.  Fields after <src> follow $DET, with
// blanks where the pass did not produce them.
void dumpSleepLogDetail() {
  Serial.print("$SLOGSTART,"); Serial.print(slogValid); Serial.print(",");
  Serial.print(slogTotal);     Serial.print(",method="); Serial.print(cfg.detectMethod);
  Serial.println(",detail=1");
  int start = (slogHead - slogValid + SLOG_MAX) % SLOG_MAX;
  for (int i = 0; i < slogValid; i++) {
    const SleepLogEntry &e = slog[(start + i) % SLOG_MAX];
    Serial.print("$SLOGD,"); Serial.print(i); Serial.print(',');
    Serial.print(e.ms);      Serial.print(',');
    Serial.print(e.awake ? 'A' : (e.stage == 2 ? 'D' : 'S')); Serial.print(',');
    Serial.print(stateChar((LeadState)e.raw));   Serial.print(',');
    Serial.print(stateChar((LeadState)e.state)); Serial.print(',');
    Serial.print(e.rn);      Serial.print(',');
    if (e.rn) {
      Serial.print(e.rmean, 5); Serial.print(',');
      Serial.print(e.rmin, 5);  Serial.print(',');
      Serial.print(e.rmax, 5);  Serial.print(',');
    } else {
      Serial.print(e.rest, 5);  Serial.print(",,,");   // probe: its rest only
    }
    if (e.tested) {
      Serial.print(e.metric, 5); Serial.print(',');
      Serial.print(e.retms, 4);  Serial.print(',');
      Serial.print(e.area, 5);   Serial.print(',');
    } else {
      Serial.print(",,,");
    }
    Serial.print(e.thr, 5);  Serial.print(',');
    if (e.rawkind != 255) Serial.print(voltKindName((VoltKind)e.rawkind));
    Serial.print(',');
    Serial.print(voltKindName((VoltKind)e.kind)); Serial.print(',');
    if (e.rawkind != 255) {
      Serial.print(e.vpos, 4); Serial.print(','); Serial.println(e.vneg, 4);
    } else {
      Serial.println(',');
    }
  }
  Serial.println("$SLOGEND");
}

// Dump the log oldest-first.  Called only from the command handler, i.e. on USB.
void dumpSleepLog() {
  Serial.print("$SLOGSTART,");
  Serial.print(slogValid);   Serial.print(",");
  Serial.print(slogTotal);   Serial.print(",method=");
  Serial.println(cfg.detectMethod);
  int start = (slogHead - slogValid + SLOG_MAX) % SLOG_MAX;
  for (int i = 0; i < slogValid; i++) {
    const SleepLogEntry &e = slog[(start + i) % SLOG_MAX];
    Serial.print("$SLOG,");
    Serial.print(i);         Serial.print(",");
    Serial.print(e.awake ? "AWAKE" : (e.stage == 2 ? "DEEP" : "SLEEP"));
    Serial.print(",");
    Serial.print(e.state == STATE_FLOAT ? "FLOAT" :
                 (e.state == STATE_CLOSED ? "CLOSED" : "VOLTAGE"));
    Serial.print(",");       Serial.print(e.metric, 4);
    Serial.print(",");       Serial.print(e.thr, 4);
    Serial.print(",");       Serial.print(e.retms, 3);
    Serial.print(",");       Serial.println(e.rest, 4);
  }
  Serial.println("$SLOGEND");
}

// Brief "asleep, not dead" flash.
//
// sleepHbTicks counts TICKS, so the heartbeat automatically slows down in stage
// 2 along with everything else -- 32 ticks is ~2 s at the 63 ms stage-1 rung and
// ~32 s at a 1 s deep rung.  That is the intended behaviour: a deeper sleep
// should also be a quieter one, and the flash is not free.
static void sleepHeartbeat() {
  if (cfg.sleepHbTicks == 0 || !cfg.ledEnable) return;
  if (++sleepHbCount < cfg.sleepHbTicks) return;
  sleepHbCount = 0;
  digitalWrite(RGB_POWER_PIN, HIGH);
  sleepMs(1);
  setPixel(0, 0, SLEEP_HB_BRIGHT);
  sleepMs(SLEEP_HB_MS);
  setPixel(0, 0, 0);
  ledPower(false);
}

// Is the board in a state where sleeping is allowed?  Diagnostics need the loop
// running, and there is no point sleeping while USB is supplying the power.
//
// The boot grace period is a safety net, not a feature: sleeping breaks the USB
// link, so if the charge detect ever misreads (bad A3 divider, wrong CHGTHRESH)
// a freshly-flashed unit could sleep before a host can reach it.  Holding it
// awake for the first SLEEP_BOOT_GRACE_MS guarantees a window to connect and
// send !SET,SLEEPSEC,0.  (Recovery does not depend on this -- the DFU
// bootloader runs before the sketch, so a double-tap of RESET always works.)
//
// an open host port (DTR asserted) also holds it awake, independently of
// CHGINHIBIT.  With CHGINHIBIT=0 the charge detect no longer stands in for "on
// USB", and Standby stops the USB peripheral -- so without this the board slept
// SLEEPSEC after the !SET and the COM port vanished, which looks like a crash.
// Arm with !SLEEP as before; it now fires when the host closes the port.
// "Open" means open AND reading (GuardedSerial): a pulled cable leaves DTR
// latched, but the stalled FIFO releases the hold within a second.
static bool lowPowerAllowed() {
  if (millis() < SLEEP_BOOT_GRACE_MS) return false;
  if (Serial) return false;
  return (cfg.idleTimeoutS > 0) && !diagMode && !chargeInhibits();
}

// One pass of the sleeping loop: sleep a tick, then decide whether to wake up
// properly.  Emits no serial -- USB is unplugged by definition here, and CDC
// writes can block when it is.
void serviceLowPower() {
  lowPowerTick();

  // A host may have replugged and sent something; that always ends the mode.
  if (Serial.available()) { exitLowPower(); pollSerial(); return; }

  // Deliberately not updateChargeState(): that also reads the battery, whose
  // divider is gated off right now and would return a meaningless value.
  //
  // skipped entirely when CHGINHIBIT=0, not merely ignored -- with a
  // boost or bench supply on the 5 V input this would otherwise fire on the
  // very first tick and wake the board forever, and skipping it also saves an
  // ADC conversion per wake, which is not nothing at these rates.  chargeActive
  // then goes stale until the next awake updateChargeState(), which is fine:
  // nothing consults it while asleep.
  if (cfg.chargeInhibit && readChargeV() > cfg.chargeThreshV) {
    chargeActive = true;
    exitLowPower();
    return;
  }

  // Stage 2 probes on its own, slower divisor.  Both stages share every other
  // part of this loop -- the probe, the wake conditions, the log.
  unsigned int pollTicks = (sleepStage == 2) ? deepPollTicks : cfg.sleepPollTicks;
  if (++sleepTickCount >= pollTicks) {
    sleepTickCount = 0;
    LeadState s = lowPowerProbe();
    slogRecord(s, false);            // for !SLEEPLOG -- no serial while asleep
    if (s != STATE_FLOAT) {          // something is connected -- wake up properly
      leadState = s;
      exitLowPower();
      return;
    }
  }

  // Stage 1 -> stage 2, once deepSec seconds of it have passed with every probe
  // reading open.  Counted in ticks: millis() does not advance in Standby, and
  // the tick length is exactly what the countdown is denominated in anyway.
  // Any probe that finds something calls exitLowPower() above, which resets the
  // stage -- so reaching here really does mean an uninterrupted open run.
  if (sleepStage == 1 && cfg.deepSec > 0 && !deepForced) {
    stageTickCount++;
    if (stageTickCount * cfg.sleepTickMs >= (unsigned long)cfg.deepSec * 1000UL)
      enterDeepSleep();
  }

  sleepHeartbeat();
}

// ── Current-measurement parking (!FLOOR) ──────────────────────────
// Holds the board in a fixed, fully-parked state so a series ammeter reads a
// stable number.  No detection, no LED, no probes: the only activity is a check
// for an exit command on each tick (a few hundred microseconds every 2 s, well
// under 1 uA averaged).  Exit with !FLOOR,0 or a reset -- note that exiting over
// USB means the meter is no longer reading the battery-only path.
void serviceFloorMode() {
  static int applied = 0;
  if (applied != floorMode) {
    applied = floorMode;
    silenceSpeaker();
    speakerOff();
    setPixel(0, 0, 0);
    ledPower(false);
    battSense(false);
    gatesRest(true);                     // !FLOOR counts as parked
    digitalWrite(MOSFET_PIN, (floorMode == 2) ? MOSFET_OFF : MOSFET_ON);
  }

  if (floorMode == 3) sleepMs(cfg.sleepTickMs);   // WFI only, for the Standby delta
  else                lowPowerTick();

  if (Serial.available()) {
    pollSerial();
    if (floorMode == 0) {            // !FLOOR,0 -- restore and resume
      applied = 0;
      digitalWrite(MOSFET_PIN, MOSFET_ON);
      battSense(true);
      gatesRest(false);
      ledPower(true);
      idleSinceMs = millis();
    }
  }
}

// ══════════════════════════════════════════════════════════════════
//  THRESHOLD SELECT -> ACTIVE THRESHOLD
// ══════════════════════════════════════════════════════════════════
// Which of the four cfg.thresh[] entries is in force.  Where that choice comes
// from depends on the board:
//   hwRev 2  physical DIP switches.  Raw pin readings, first digit D8, second
//            digit D10 (HIGH = 1 = switch open).  Index = 0b(D8)(D10), i.e.
//            "01" = D8 low + D10 high = 1.
//   hwRev 3+ the DIP switches are gone from the PCB and D8 is a buzzer leg (V3)
//            or an amp enable (V3b), so the selection moves into EEPROM as
//            cfg.threshSel.  The index space
//            and every downstream user (sleepThresh[], the $DIP message, the
//            GUI's config table) are deliberately unchanged.
uint8_t readDipIndex() {
  if (!boardIsV2()) return cfg.threshSel & 0x03;
  uint8_t a = digitalRead(DIP_PIN_A) ? 1 : 0;   // D8
  uint8_t b = digitalRead(DIP_PIN_B) ? 1 : 0;   // D10
  return (a << 1) | b;
}

// Refresh activeThreshV from the selector; announce live changes on serial.
// Still reported as $DIP on hwRev 3+ -- the host tooling keys on that name, and
// the meaning ("the threshold position changed") is the same.  On V3 it fires
// in response to !SET,THRESHSEL rather than someone moving a switch.
void updateThresholdFromDip() {
  uint8_t idx = readDipIndex();
  if (idx != dipIdx) {
    dipIdx = idx;
    Serial.print("$DIP,");
    Serial.print(idx);
    Serial.print(",");
    Serial.println(cfg.thresh[idx], 4);
  }
  activeThreshV = cfg.thresh[dipIdx];
}

// ══════════════════════════════════════════════════════════════════
//  MEASUREMENT
// ══════════════════════════════════════════════════════════════════
// Negative-channel read.  Normally a live SENSE_NEG sample, but when
// cfg.negFix is on -- or the board has no SENSE_NEG pin at all (V3b) -- it
// returns the count for cfg.negFixV instead, so the diff rides on a clean fixed
// pseudo-reference (the shipped default).
int readNegRaw() {
  if (negFixed()) {
    return (int)((cfg.negFixV / ADC_REF_VOLTAGE) * ADC_FULL_SCALE + 0.5f);
  }
  return analogRead(SENSE_NEG);
}

// Pseudo-differential read: sample both pins vs. GND and subtract.
// The RA4M1 shares one ADC + sample/hold behind an input mux, so the first
// conversion after a channel switch carries residual charge from the previous
// channel.  Discard one throwaway conversion per channel before the real read.
float readVoltage() {
  analogRead(SENSE_POS);                 // throwaway: settle S/H after prior channel
  int rawPos = analogRead(SENSE_POS);
  if (!negFixed()) analogRead(SENSE_NEG);
  int rawNeg = readNegRaw();
  return ((rawPos - rawNeg) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

// Decide whether a real voltage is present (MOSFET resting / on):
// any single read beyond voltFastMult * refBand -> present immediately;
// otherwise average voltAvgSamples reads and test against refBand.
bool voltagePresent() {
  float sum = 0.0f;
  // with !DETLOG off this is the production loop exactly.  With it on,
  // a fast trip is remembered rather than returned, so the remaining reads
  // still land in the mean/min/max the $DET line reports (see detLogOn).
  bool  fastTrip = false;
  float vmin = 1e9f, vmax = -1e9f;
  int   n = 0;
  for (int i = 0; i < cfg.voltAvgSamples; i++) {
    float v = readVoltage();
    n++;
    sum += v;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
    if (!fastTrip && fabs(v - cfg.refCenterV) > cfg.voltFastMult * cfg.refBandV) {
      lastRestV = v;
      fastTrip  = true;
      if (!detLogOn && !slogFullReads) break;
    }
  }
  detRestN    = n;
  detRestMean = sum / n;
  detRestMin  = vmin;
  detRestMax  = vmax;
  if (fastTrip) { detVoltPath = 'F'; return true; }
  lastRestV = detRestMean;               // n == voltAvgSamples here
  bool present = (fabs(lastRestV - cfg.refCenterV) > cfg.refBandV);
  detVoltPath = present ? 'A' : '-';
  return present;
}

// Voltage check used ONLY by the sleeping probe.  Same reads as
// voltagePresent(), but the decision uses only the wide "instant bypass" band
// (voltFastMult * refBandV) and never the tight averaged refBandV test -- on a
// floating lead, noise that drifts past refBandV is enough to wake the board
// every couple of seconds, which defeats the whole point of sleeping.
//
// NOTE: with a single band there is nothing left to average.  If no individual
// read exceeds the band then their mean cannot either (the mean deviation is at
// most the largest single deviation), so the averaged comparison would be
// mathematically dead code.  The check is therefore "did ANY of sleepVoltAvg
// reads exceed the band" -- which means MORE reads = MORE chances to trip, the
// opposite of the awake path.  Keep sleepVoltAvg low, and raise VOLTFAST (not
// SLEEPAVG) if genuine noise still wakes it.
bool voltagePresentSleep() {
  float band = cfg.voltFastMult * cfg.refBandV;
  float sum  = 0.0f;
  int   n    = cfg.sleepVoltAvg;
  for (int i = 0; i < n; i++) {
    float v = readVoltage();
    if (fabs(v - cfg.refCenterV) > band) {
      lastRestV = v;
      return true;
    }
    sum += v;
  }
  lastRestV = sum / n;        // for debug only; not a decision input
  return false;
}

// Name a kind for the serial output.
const char *voltKindName(VoltKind k) {
  return (k == VKIND_AC) ? "VAC" : ((k == VKIND_DCN) ? "VDC-" : "VDC+");
}

// Decide WHICH kind of voltage is on the leads, once voltagePresent() has said
// there is one (ported verbatim from production).  Presence itself is untouched
// by this -- it is the safety-critical decision, and this runs after it.
//
// A peak hunt, not an average: sampling for acWindowMs -- which must cover a
// full mains cycle -- catches both peaks of a 50/60 Hz waveform whatever its
// phase when the window opens:
//   both peaks past the band -> the waveform crosses the resting centre -> VAC
//   otherwise                -> the side with the larger peak gives the polarity
// A large input CLAMPS rather than rectifies, which is why this survives one:
// AC flat-tops symmetrically and still reads two-sided, while DC of either
// polarity clamps on one side only.
//
// The band defaults to voltFastMult * refBandV -- the wide "instant bypass"
// band, NOT the tighter refBandV the averaged presence test uses: a peak over
// hundreds of samples is a far noisier statistic than a mean over ten.  The
// cost is that AC whose peaks fall between the two bands classifies as DC (it
// still alerts as voltage, with the wrong sub-pattern).  ACBAND overrides.
//
// the caller must have the analog gate raised -- runDetection() does
// (the whole pass sits inside gatesMeasureBegin/End) and !VTEST does its own.
VoltKind classifyVoltage() {
  if (!cfg.voltClassify) return VKIND_DCP;      // legacy: every voltage is VDC+

  float band = (cfg.acBandV > 0.0f) ? cfg.acBandV
                                    : cfg.voltFastMult * cfg.refBandV;
  float vMax = -1.0e6f, vMin = 1.0e6f;
  int   n    = 0;

  // Bounded by TIME, not by a sample count: the mains-cycle argument above is
  // about wall clock.
  unsigned long windowUs = (unsigned long)cfg.acWindowMs * 1000UL;
  unsigned long t0 = micros();
  while (micros() - t0 < windowUs) {
    float v = readVoltage();
    if (v > vMax) vMax = v;
    if (v < vMin) vMin = v;
    n++;
  }

  lastVPosPeakV = vMax - cfg.refCenterV;
  lastVNegPeakV = cfg.refCenterV - vMin;
  lastVSamples  = n;

  if (lastVPosPeakV > band && lastVNegPeakV > band) return VKIND_AC;
  return (lastVPosPeakV >= lastVNegPeakV) ? VKIND_DCP : VKIND_DCN;
}

// Sample the recovery transient once (methods 1 & 2).  MOSFET is assumed to
// have just been switched OFF by the caller; timing starts here.  In a single
// pass this computes BOTH candidate metrics so either can be thresholded and
// the other reported for tuning:
//   lastReturnMs = time for |diff-refCentre| to fall back within detReturnBand
//                  (or detWindowUs, expressed in ms, if it never does -> OPEN)
//   lastAreaVms  = integral of |diff-refCentre| over [detAreaStartUs, window]
// Returns the metric selected by cfg.detectMethod (ms for 1, V*ms for 2).
float sampleRecovery() {
  unsigned long t0 = micros();
  float area       = 0.0f;
  float prevDev    = 0.0f;
  unsigned long prevT = 0;
  bool  havePrev   = false;
  bool  seenTrough = false;      // the dip has occurred (guards false returns)
  bool  returned   = false;
  float returnMs   = (float)cfg.detWindowUs / 1000.0f;   // timeout default
  unsigned long elapsed;

  while ((elapsed = micros() - t0) < cfg.detWindowUs) {
    float v   = readVoltage();
    float dev = fabs(v - cfg.refCenterV);
    lastTestV = v;

    if (dev > DET_TROUGH_MIN_V) seenTrough = true;

    // Tail-area: trapezoid between consecutive in-window samples.
    if (havePrev && elapsed >= cfg.detAreaStartUs && prevT >= cfg.detAreaStartUs) {
      float dt = (float)(elapsed - prevT) / 1000.0f;     // ms
      area += 0.5f * (prevDev + dev) * dt;
    }

    // Time-to-return: first crossing back within the band, but only after the
    // transient has actually dipped.
    if (!returned && seenTrough && dev <= cfg.detReturnBand) {
      returnMs = (float)elapsed / 1000.0f;
      returned = true;
      if (cfg.detectMethod == 1) break;   // time method needs nothing further
    }

    prevDev = dev; prevT = elapsed; havePrev = true;
  }

  lastReturnMs = returnMs;
  lastAreaVms  = area;
  return (cfg.detectMethod == 1) ? returnMs : area;
}

// One open/closed test: MOSFET off, derive the metric per cfg.detectMethod,
// MOSFET back on.  For every method a LARGER metric means MORE OPEN, so the
// threshold comparison and DIP table are shared across methods.
LeadState runMosfetTest() {
  digitalWrite(MOSFET_PIN, MOSFET_OFF);

  float metric;
  if (cfg.detectMethod == 0) {
    // Legacy single-sample: settle, one differential read, threshold |diff|.
    delayMicroseconds(cfg.settlePreUs);
    lastTestV = readVoltage();
    metric    = fabs(lastTestV);
  } else {
    // Methods 1 & 2 sample the recovery from the toggle instant (no pre-settle:
    // the early samples carry the transient the metrics are built from).
    metric = sampleRecovery();
  }
  lastMetric = metric;

  sleepMs(cfg.settlePostMs);
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  return (metric > activeThreshV) ? STATE_FLOAT : STATE_CLOSED;
}

// Repeat the test until the same result appears cfg.testAgree times in a row
// (capped at TEST_MAX_ATTEMPTS so a noisy boundary can never hang the loop).
LeadState runMosfetTestStable() {
  LeadState result = runMosfetTest();
  LeadState prev   = result;
  int agree = 1, attempts = 1;
  while (agree < cfg.testAgree && attempts < TEST_MAX_ATTEMPTS) {
    result = runMosfetTest();
    agree  = (result == prev) ? (agree + 1) : 1;
    prev   = result;
    attempts++;
  }
  return result;
}

// ══════════════════════════════════════════════════════════════════
//  LED
// ══════════════════════════════════════════════════════════════════
// On V3b LED1 is powered from the gated LEDRail, so a colour needs the rail up
// and dark does not.  Whenever the rail is down -- LEDGMODE 2 between flashes,
// LEDGMODE 1 while asleep -- a colour raises it for exactly as long as the
// colour is showing and the next dark write drops it again.  The heartbeat and
// the boot cue therefore work in every mode without knowing about the gate.
// Only a raise made HERE is undone here (ledGateByPixel): a rail that is up
// because of MODE 0/1 or a !GATE hold belongs to gatesRest().
void setPixel(uint8_t r, uint8_t g, uint8_t b) {
  bool lit = (r || g || b);
  pixel.setPixelColor(0, pixel.Color(r, g, b));

  if (!gatePresent(GATE_LED)) { pixel.show(); return; }   // V2/V3: LED always powered

  if (lit && !gateOn[GATE_LED] && gateForce[GATE_LED] < 0) {
    gateWrite(GATE_LED, true);
    ledGateByPixel = true;
    gateDelayUs(gateSettleUs(GATE_LED));    // rail up before any data goes out
  }
  // Never clock data into an unpowered SK6812 -- the data pin would back-feed
  // it through its input protection.  With the rail down the part is dark,
  // which is all a dark write asks for (and a lit one held off by !GATE,0
  // cannot be shown anyway).
  if (!gateOn[GATE_LED]) return;
  pixel.show();
  // Drop it only on the way to dark.  Cutting the rail with a colour still
  // latched would make the next flash's first bit land on an unpowered part.
  if (!lit && ledGateByPixel) gateWrite(GATE_LED, false);
}

// Rate-limited flash: on for onMs, then off, and no new flash until minGapMs
// after the last one began.  Flags/timestamps owned per-state by the caller.
// Returns true on the pass that STARTS a flash, and only then -- that is what
// lets the voltage alert step through a colour sequence one flash at a time.
bool flashState(unsigned long now,
                uint8_t r, uint8_t g, uint8_t b,
                unsigned long onMs, unsigned long minGapMs,
                bool &flashing, unsigned long &lastFlash) {
  if (!flashing && (now - lastFlash >= minGapMs)) {
    flashing  = true;
    lastFlash = now;
    setPixel(r, g, b);
    return true;
  } else if (flashing && (now - lastFlash >= onMs)) {
    flashing = false;
    setPixel(0, 0, 0);
  }
  return false;
}

void updateLed() {
  static LeadState prevState = STATE_VOLTAGE;
  static VoltKind  prevKind  = VKIND_DCP;
  unsigned long now = millis();

  // On a state change -- or a change of voltage KIND, which is a different
  // pattern and has to restart on red rather than resume mid-sequence -- end
  // any in-progress flash but keep the lastXFlash timestamps: each state's
  // rate limit persists across transitions so a bouncing state can't re-fire
  // immediately.
  if (leadState != prevState || voltKind != prevKind) {
    prevState      = leadState;
    prevKind       = voltKind;
    floatFlashing  = false;
    closedFlashing = false;
    voltFlashing   = false;
    voltSeqIdx     = 0;
    setPixel(0, 0, 0);
  }

  if (!cfg.ledEnable) { setPixel(0, 0, 0); return; }

  // One channel per state: blue = floating, green = closed, red = voltage --
  // except that the voltage alert spells its KIND out across successive
  // flashes (VKIND_SEQ), always starting on red.
  switch (leadState) {
    case STATE_FLOAT:
      flashState(now, 0, 0, cfg.ledFloatBright,
                 cfg.ledFloatMs, cfg.ledFloatPerMs, floatFlashing, lastFloatFlash);
      break;
    case STATE_CLOSED:
      flashState(now, 0, cfg.ledClosedBright, 0,
                 cfg.ledClosedMs, cfg.ledClosedPerMs, closedFlashing, lastClosedFlash);
      break;
    case STATE_VOLTAGE: {
      // One flash = one step of this kind's sequence.  Advance only when a
      // flash actually started, so the pattern tracks what was shown.
      uint8_t ch = VKIND_SEQ[voltKind][voltSeqIdx];
      if (flashState(now,
                     (ch == 0) ? cfg.ledVoltBright : 0,
                     (ch == 1) ? cfg.ledVoltBright : 0,
                     (ch == 2) ? cfg.ledVoltBright : 0,
                     cfg.ledVoltMs, cfg.ledVoltPerMs, voltFlashing, lastVoltFlash))
        voltSeqIdx = (voltSeqIdx + 1) % VKIND_SEQ_LEN[voltKind];
      break;
    }
  }
}

// ══════════════════════════════════════════════════════════════════
//  SPEAKER
// ══════════════════════════════════════════════════════════════════
// Three drives, one per board family:
//
// V2 wires one leg of the piezo to D9 and the other to ground, so a tone is
// just a square wave on D9 and the core's tone() does the whole job.
//
// V3 wires the element BETWEEN D8 and D9 (BZ1+ through R16, BZ1- through R17).
// Holding D8 low reproduces the V2 drive exactly; driving it INVERTED against
// D9 puts twice the voltage across the element -- about +6 dB for no extra
// parts.  That needs both pins toggled from one timer, which tone() cannot do
// (it owns a single pin), so cfg.spkDiff routes through a private FspTimer.
// The tidier hardware route -- a GPT channel's complementary GTIOCnA/GTIOCnB
// pair -- is not available: D8 is P111 = GTIOC3A and D9 is P110 = GTIOC1B,
// different channels.  Hence the software toggle below.
//
// V3b puts a PAM8904 between the MCU and the element.  The MCU only supplies
// the tone on DIN (D15) and the gain on EN1/EN2 (D8/D9); the chip's charge
// pump and H-bridge do the rest, so the element sees up to 3x VDD each way.
// D15 is P101 = GTIOC5A, a real GPT output, so the tone is HARDWARE PWM -- no
// ISR -- and that is what makes the duty-cycle trim (SPKDUTY) free.
//
// Volume, all boards: an alert whose volume key is 0 does not sound.  1-3
// only mean something on V3b, where they are the PAM8904 gain step.

static FspTimer     buzzTimer;
static bool         buzzTimerOpen = false;   // FspTimer channel claimed
static volatile bool buzzPhase    = false;

// True when the anti-phase drive should be used: V3 hardware, enabled in
// config, and a pitched (passive-buzzer) tone rather than a DC level.  On
// V2, D8 is a DIP input and must never be driven; on V3b it is EN1.
static inline bool buzzDifferential() {
  return boardIsV3() && cfg.spkDiff && cfg.passiveBuzzer;
}

// Toggle both legs.  Two digitalWrite calls, exactly as the core's own tone
// ISR does: the ~1 us of skew between them is well under 1% of a half-period
// at these frequencies, and the load is a capacitor, so it is neither audible
// nor a shoot-through concern.
void buzzTimerCallback(timer_callback_args_t *args) {
  (void)args;
  buzzPhase = !buzzPhase;
  digitalWrite(SPEAKER_PIN,   buzzPhase ? HIGH : LOW);
  digitalWrite(SPEAKER_PIN_B, buzzPhase ? LOW  : HIGH);
}

// Claim (once) and run the anti-phase timer at `freq`.  The ISR toggles on
// every fire, so it has to run at twice the tone frequency.  Returns false if
// no timer channel was free, letting the caller fall back to single-ended.
static bool buzzTimerRun(unsigned int freq) {
  float toggleHz = (float)freq * 2.0f;
  if (buzzTimerOpen) {
    // set_frequency() only rewrites the period register -- it does NOT start
    // the timer (the core's own Tone class calls start() separately).  Without
    // the explicit start below, every beep after the first one is silent.
    buzzTimer.stop();
    buzzTimer.set_frequency(toggleHz);
    buzzTimer.start();
    return true;
  }
  uint8_t type = 0;
  int8_t  ch   = FspTimer::get_available_timer(type);
  if (ch < 0) return false;
  if (!buzzTimer.begin(TIMER_MODE_PERIODIC, type, (uint8_t)ch, toggleHz, 50.0f,
                       buzzTimerCallback, nullptr)) return false;
  if (!buzzTimer.setup_overflow_irq()) return false;
  if (!buzzTimer.open())               return false;
  buzzTimerOpen = true;
  buzzTimer.start();
  return true;
}

// ── PAM8904 (V3b) ─────────────────────────────────────────────────
// Gain select, from the datasheet's mode table (DIN active):
//     EN1 EN2    0 0 shutdown    0 1 1x    1 0 2x    1 1 3x
// so volume 0-3 maps straight onto the two bits: EN1 = bit 1, EN2 = bit 0.
// VDD is the 3.3 V rail, well inside the 4.5 V limit the datasheet puts on
// 3x mode (above that VOUT would exceed its 13.5 V absolute maximum).
//
// The amp is held in SHUTDOWN (EN1 = EN2 = 0, < 1 uA) whenever no pulse is
// sounding, not merely left to its own auto-standby: that only arrives 42 ms
// after DIN stops, and at 3x the charge pump draws ~1.7 mA unloaded until it
// does.  Start-up from shutdown is ~0.3 ms (tON), negligible against even the
// 20 ms voltage pulse.
static PwmOut ampPwm(D15);            // P101 / GTIOC5A -- fixed on V3b
static bool   ampPwmOn  = false;
static bool   ampToneOn = false;      // fallback path, if the PWM would not start
static int    ampPwmCh  = -1;         // GPT channel the PWM ran on (5 on the XIAO)

static void ampGain(uint8_t vol) {
  digitalWrite(AMP_EN1_PIN, (vol & 0x02) ? HIGH : LOW);
  digitalWrite(AMP_EN2_PIN, (vol & 0x01) ? HIGH : LOW);
}

// Release the PWM.  FspTimer::end() marks the channel FREE, but the variant
// reserves GPT5 for PWM at boot (D11 shares it) precisely so that tone() and
// FspTimer::get_available_timer() never hand it out as a periodic timer.  Left
// FREE, the next such request could take it, and the following ampPwm.begin()
// would then try to share a non-PWM timer's config.  So put the reservation back.
static void ampPwmRelease() {
  ampPwm.end();
  if (ampPwmCh >= 0) FspTimer::set_initial_timer_channel_as_pwm(GPT_TIMER, ampPwmCh);
}

// Silence and shut down.  DIN goes back to a plain GPIO driven low: after the
// timer is closed the pin is still muxed to the GPT with whatever level the
// counter stopped on, and a DIN left high would hold the amp awake.
static void ampStop() {
  if (!pinIsValid(AMP_DIN_PIN)) return;
  if (ampPwmOn)  { ampPwmRelease();       ampPwmOn  = false; }
  if (ampToneOn) { noTone(AMP_DIN_PIN);   ampToneOn = false; }
  pinMode(AMP_DIN_PIN, OUTPUT);
  digitalWrite(AMP_DIN_PIN, LOW);
  ampGain(0);
}

// One pulse at `freq` and gain `vol`.  The duty cycle is the fine trim below a
// gain step: the element sees +/-VOUT for duty and (1-duty) of each period, so
// the fundamental scales with sin(pi * duty) -- 50 % is full, and the rest of
// the waveform is harmonics the 4 kHz resonator largely ignores.
static void ampOn(unsigned int freq, uint8_t vol) {
  ampStop();                         // a priority alert can preempt a pulse mid-tone
  if (vol == 0 || !pinIsValid(AMP_DIN_PIN)) return;
  ampGain(vol > 3 ? 3 : vol);
  float duty = (float)constrain(cfg.spkDuty, 1, 50);
  if (ampPwm.begin((float)freq, duty)) {
    ampPwmOn = true;
    ampPwmCh = (int)ampPwm.get_timer()->get_channel();
  } else {
    ampPwmRelease();                 // a half-made begin still claimed the channel
    pinMode(AMP_DIN_PIN, OUTPUT);
    tone(AMP_DIN_PIN, freq);         // 50 % only, but never silent
    ampToneOn = true;
  }
}

// Stop every drive on every pin, whatever the board.  applyBoardPins() calls
// this BEFORE re-assigning pin roles, so it has to use the old ones.
void buzzerStop() {
  if (buzzTimerOpen) buzzTimer.stop();
  if (pinIsValid(SPEAKER_PIN)) noTone(SPEAKER_PIN);
  ampStop();
  buzzPhase = false;
}

// Set the buzzer / amp / DIP / park pins to the roles the current cfg.hwRev
// calls for.  This is the ONLY place those pins' directions are decided --
// applyBoardPins() calls it last, after anything that can change hwRev.
void buzzerApplyPinModes() {
  // Silence first.  This can be called with a tone running (!SET,HWREV while
  // the board is beeping), and the pin roles are about to change under it.
  buzzerStop();

  if (pinIsValid(SPEAKER_PIN))   { pinMode(SPEAKER_PIN, OUTPUT);   digitalWrite(SPEAKER_PIN, SPEAKER_OFF); }
  if (pinIsValid(SPEAKER_PIN_B)) { pinMode(SPEAKER_PIN_B, OUTPUT); digitalWrite(SPEAKER_PIN_B, LOW); }
  // V3's D10 is a pure no-connect (broken out on J6 only): park it as a driven
  // low rather than a floating input, which burns crossbar current at the
  // sleeping-current levels this board is tuned to.
  if (pinIsValid(PARK_PIN))      { pinMode(PARK_PIN, OUTPUT);      digitalWrite(PARK_PIN, LOW); }
  // V2: both DIP pins are inputs (hardware pull-ups on the PCB).
  if (pinIsValid(DIP_PIN_A))     pinMode(DIP_PIN_A, INPUT);
  if (pinIsValid(DIP_PIN_B))     pinMode(DIP_PIN_B, INPUT);
  // V3b: EN pins as outputs, then ampStop() leaves DIN low and the amp shut down.
  if (pinIsValid(AMP_EN1_PIN))   pinMode(AMP_EN1_PIN, OUTPUT);
  if (pinIsValid(AMP_EN2_PIN))   pinMode(AMP_EN2_PIN, OUTPUT);
  ampStop();
}

// One pulse of `freq` at volume `vol` (0 = silent on every board; 1-3 = the
// PAM8904 gain on V3b, ignored elsewhere).  Passive buzzer (cfg.passiveBuzzer):
// square wave, so each alert can have its own pitch.  Active buzzer: DC level.
void speakerOn(unsigned int freq, uint8_t vol) {
  if (boardHasAmp()) { ampOn(freq, vol); return; }
  if (vol == 0) { speakerOff(); return; }

  if (buzzDifferential() && buzzTimerRun(freq)) return;

  // Not using the anti-phase timer for this pulse.  Stop it explicitly rather
  // than assuming speakerOff() already did: a priority alert preempts a beep in
  // progress by calling startBeep(force) -> speakerOn() with no speakerOff()
  // in between, so a PASSIVE or SPKDIFF change between pulses could otherwise
  // leave the ISR toggling both legs underneath the drive selected here.
  if (buzzTimerOpen) buzzTimer.stop();
  if (pinIsValid(SPEAKER_PIN_B)) digitalWrite(SPEAKER_PIN_B, LOW);   // park the second leg

  if (!cfg.passiveBuzzer) {
    // DC drive.  With the second leg low this still puts the full rail across
    // the element -- but note a bare piezo like the PKLCS1212E makes no sound
    // from a DC level at all.  PASSIVE=0 only does something useful with a
    // self-oscillating buzzer fitted in its place.
    digitalWrite(SPEAKER_PIN, SPEAKER_ON);
    return;
  }
  tone(SPEAKER_PIN, freq);                    // single-ended (V2, or no timer free)
}

void speakerOff() {
  if (boardHasAmp()) { ampStop(); return; }
  if (buzzTimerOpen) buzzTimer.stop();
  if (!pinIsValid(SPEAKER_PIN)) return;
  noTone(SPEAKER_PIN);                        // harmless when not toning
  digitalWrite(SPEAKER_PIN, SPEAKER_OFF);
  // Both legs low = no voltage across the element and no static current, which
  // is what the low-power park and !FLOOR measurements assume.
  if (pinIsValid(SPEAKER_PIN_B)) digitalWrite(SPEAKER_PIN_B, LOW);
  buzzPhase = false;
}

// Begin a rate-limited sequence of `pulses` beeps.  `force` preempts any
// in-progress sequence and ignores the rate cap (priority alerts).
// `lastOnMs` is the on-time of the final pulse; 0 means every pulse uses onMs.
// `vol` is the alert's volume key; the sequence keeps its timing at vol 0, so
// muting one alert never changes when the other one is allowed to sound.
void startBeep(unsigned long now, int pulses, unsigned long onMs, unsigned long offMs,
               bool force, unsigned int freq, unsigned long lastOnMs, uint8_t vol) {
  if (!force) {
    if (beepOn || beepPulsesLeft > 0)             return;
    if (now - lastBeepSeqStart < cfg.beepMinMs)   return;
  }
  lastBeepSeqStart = now;
  beepPulsesLeft   = pulses;
  beepOnMs         = onMs;
  beepOffMs        = offMs;
  beepLastOnMs     = lastOnMs;
  beepFreq         = freq;
  beepVol          = vol;
  beepPhaseStart   = now;
  beepOn           = true;
  speakerOn(beepFreq, beepVol);
  beepPulsesLeft--;
}

void updateBeep(unsigned long now) {
  if (!beepOn && beepPulsesLeft == 0) return;
  if (beepOn) {
    // The final pulse may be a different length.  beepPulsesLeft is decremented
    // when a pulse STARTS, so 0 here means "the pulse now sounding is the last".
    unsigned long onMs = (beepPulsesLeft == 0 && beepLastOnMs > 0) ? beepLastOnMs
                                                                   : beepOnMs;
    if (now - beepPhaseStart >= onMs) {
      speakerOff();
      beepOn         = false;
      beepPhaseStart = now;
    }
  } else if (beepPulsesLeft > 0 && now - beepPhaseStart >= beepOffMs) {
    speakerOn(beepFreq, beepVol);
    beepOn         = true;
    beepPhaseStart = now;
    beepPulsesLeft--;
  }
}

void silenceSpeaker() {
  if (beepOn) speakerOff();
  beepOn         = false;
  beepPulsesLeft = 0;
}

// Map the detection state to audio, mirroring updateLed().
//   CLOSED  -> continuity beep;  VOLTAGE -> voltage beep;  FLOAT -> silent.
// The voltage beep's RHYTHM carries the same information as the LED sequence:
//   VDC+  short-short        VDC-  short-long        VAC  three shorts
// Suppressed while charging (same lockout as the LED) and when muted/disabled.
void updateSpeaker() {
  static LeadState prevState = STATE_VOLTAGE;
  static VoltKind  prevKind  = VKIND_DCP;
  unsigned long now = millis();

  if (!cfg.beepEnable || speakerMuted) {
    silenceSpeaker();
    prevState = leadState;
    prevKind  = voltKind;
    return;
  }
  if (chargeInhibits() && !alertOverride) {
    silenceSpeaker();
    prevState = leadState;      // avoid a stale beep on unplug
    prevKind  = voltKind;
    return;
  }

  // A change of voltage KIND re-beeps too -- the rhythm is the message -- but
  // it is NOT a priority alert the way entering the state is: it goes through
  // the rate cap, which stops a signal on the classifier's boundary beeping
  // every couple of hundred milliseconds.
  bool entered     = (leadState != prevState);
  bool kindChanged = (leadState == STATE_VOLTAGE && !entered &&
                      voltKind != prevKind);
  prevState = leadState;
  prevKind  = voltKind;

  if (leadState == STATE_CLOSED) {
    if (entered)
      startBeep(now, cfg.contPulses, cfg.contOnMs, cfg.contOffMs, false, cfg.contFreqHz, 0, cfg.contVol);
    else if (cfg.contRepeat && now - lastBeepSeqStart >= cfg.contRepeatMs)
      startBeep(now, cfg.contPulses, cfg.contHoldMs, cfg.contOffMs, false, cfg.contFreqHz, 0, cfg.contVol);
  } else if (leadState == STATE_VOLTAGE) {
    // Pulse count and shape per kind.  VDC- stretches the LAST pulse rather
    // than pinning the count at two, so raising VOLTPULSES gives
    // short-short-long instead of breaking the pattern.
    int           pulses = (voltKind == VKIND_AC) ? cfg.voltAcPulses : cfg.voltPulses;
    unsigned long lastMs = (voltKind == VKIND_DCN) ? cfg.voltNegLongMs : 0;
    if (entered)                // priority alert: always sounds on entry
      startBeep(now, pulses, cfg.voltOnMs, cfg.voltOffMs, true, cfg.voltFreqHz, lastMs, cfg.voltVol);
    else if (kindChanged)       // informational: goes through the rate cap
      startBeep(now, pulses, cfg.voltOnMs, cfg.voltOffMs, false, cfg.voltFreqHz, lastMs, cfg.voltVol);
    else if (cfg.voltRepeat && now - lastBeepSeqStart >= cfg.voltRepeatMs)
      startBeep(now, pulses, cfg.voltOnMs, cfg.voltOffMs, false, cfg.voltFreqHz, lastMs, cfg.voltVol);
  }

  updateBeep(now);
}

// One clean measurement at boot to decide the session mute: leads shorted
// (CLOSED) at boot -> speaker muted until the next boot cycle.
void bootSpeakerMuteCheck() {
  if (!cfg.bootMute) return;
  updateThresholdFromDip();
  // This reads the front end, so it needs the analog rail like every other ADC
  // consumer -- with ANAMODE 2 the rail is down here, and an unpowered front end
  // would decide the mute on garbage.
  uint8_t gatesRaised = gatesMeasureBegin();
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  bool present = voltagePresent();       // voltage at boot -> leave audio enabled
  if (!present && runMosfetTestStable() == STATE_CLOSED) speakerMuted = true;
  gatesMeasureEnd(gatesRaised);
}

// ══════════════════════════════════════════════════════════════════
//  CHARGE / BATTERY
// ══════════════════════════════════════════════════════════════════
float readChargeV() {
  return (analogRead(CHARGE_PIN) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

float readBattV() {
  return (analogRead(BATT_PIN) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE * BATT_DIV;
}

int battPercentOf(float v) {
  float pct = (v - cfg.battEmptyV) / (cfg.battFullV - cfg.battEmptyV) * 100.0f;
  return (int)constrain(pct, 0.0f, 100.0f);
}

// VBUS high = USB plugged in -> charging.  Unplugging clears the per-session
// alert override so the next charge starts back in the charging-blink state.
void updateChargeState() {
  chargeActive = (readChargeV() > cfg.chargeThreshV);
  if (!chargeActive) alertOverride = false;
  battV   = readBattV();
  battPct = battPercentOf(battV);
}

// Slow "charging" blink: dim red at 25% duty; green once the battery reads
// full (>= cfg.battFullPct).
void chargeBlink() {
  unsigned long now   = millis();
  unsigned long phase = now - lastChargeBlink;
  if (!chargeBlinkOn && phase >= CHARGE_BLINK_PERIOD_MS) {
    chargeBlinkOn   = true;
    lastChargeBlink = now;
    uint8_t r = CHARGE_BLINK_BRIGHT, g = 0;
    if (battPct >= cfg.battFullPct) { r = 0; g = CHARGE_BLINK_BRIGHT; }
    setPixel(r, g, 0);
  } else if (chargeBlinkOn && phase >= CHARGE_BLINK_ON_MS) {
    chargeBlinkOn = false;
    setPixel(0, 0, 0);
  }
}

// LED owner: charging (and not overridden) -> charging blink; otherwise the
// normal detection alerts.  Blank + reset flags on mode transitions.
void updateAlerts() {
  static bool prevCharging = false;
  bool charging = (chargeInhibits() && !alertOverride);
  if (charging != prevCharging) {
    prevCharging   = charging;
    chargeBlinkOn  = false;
    floatFlashing  = false;
    closedFlashing = false;
    voltFlashing   = false;
    setPixel(0, 0, 0);
  }
  if (charging) chargeBlink();
  else          updateLed();
}

// Power-on charge-level cue: 1..4 slow green blinks (0-25% = 1 ... 75-100% = 4).
void startupBatteryIndicate() {
  // BAT_READ_EN is driven HIGH at the very top of setup(), so by now the
  // divider has had the whole of init to settle; this pre-gap finishes the job
  // and is the pause the cue wanted anyway, so the settle is effectively free.
  // (It replaces a dedicated 100 ms discard loop that used to sit here.)
  sleepMs(BOOT_CUE_PREGAP_MS);
  for (int i = 0; i < 4; i++) readBattV();      // discard: flush the ADC S/H
  float v = 0.0f;
  for (int i = 0; i < 8; i++) v += readBattV();
  v /= 8.0f;

  int blinks = battPercentOf(v) / 25 + 1;
  if (blinks > 4) blinks = 4;
  for (int i = 0; i < blinks; i++) {
    setPixel(0, CHARGE_BLINK_BRIGHT, 0);
    sleepMs(BOOT_CUE_ON_MS);
    setPixel(0, 0, 0);
    // No trailing gap after the last blink -- nothing follows it to separate
    // from, and it would just delay the first measurement.
    if (i < blinks - 1) sleepMs(BOOT_CUE_OFF_MS);
  }
}

// ══════════════════════════════════════════════════════════════════
//  DETECTION (one pass) -- sets leadState, honouring voltOverride
// ══════════════════════════════════════════════════════════════════
void runDetection() {
  // raise any pulsed (mode 2) gate for the measurement and settle.
  // No-op unless a gate is wired AND in mode 2, so the awake path is otherwise
  // exactly the production one.
  uint8_t gatesRaised = gatesMeasureBegin();

  updateThresholdFromDip();              // selector re-read every pass
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state

  bool present;
  detRestN = 0;                          // $DET reports no rest stats unless read
  if (voltOverride == VOLT_FORCE_ON) {
    lastRestV = readVoltage();           // keep a fresh reading for debug
    present = true;
    detVoltPath = 'L';
  } else if (voltOverride == VOLT_DISABLED) {
    present = false;
    detVoltPath = 'D';
  } else {
    present = voltagePresent();
  }

  detTestRan = !present;
  LeadState rawState = present ? STATE_VOLTAGE : runMosfetTestStable();
  lastRawState = rawState;               // for the sleep log's awake entries

  // Sub-classify while the voltage is actually on the leads -- the MOSFET is
  // still at rest from the presence test, which is the condition the classifier
  // wants, and the analog gate is still raised.  Nothing here can change
  // rawState: getting the kind wrong picks the wrong alert pattern, never the
  // wrong alert.
  //
  // The FIRST classification of a new contact is adopted outright; after that a
  // change has to repeat cfg.stableCount times, the same debounce leadState
  // uses.  The immediate adopt makes the very first beep the right rhythm
  // (leadState still needs its own stableCount passes before it commits to
  // STATE_VOLTAGE); the debounce afterwards stops a reading on the boundary
  // flickering the pattern.  While no voltage is present the kind is marked
  // stale, so the next contact starts clean.
  static VoltKind kindCandidate = VKIND_DCP;
  static int      kindStable    = 0;
  voltKindRawFresh = false;
  if (present) {
    VoltKind rawKind = classifyVoltage();
    voltKindRaw      = rawKind;
    voltKindRawFresh = cfg.voltClassify;
    if (!voltKindValid) {
      voltKind      = rawKind;
      voltKindValid = true;
      kindCandidate = rawKind;
      kindStable    = 0;
    } else if (rawKind == voltKind) {
      kindCandidate = rawKind;
      kindStable    = 0;
    } else {
      if (rawKind != kindCandidate) { kindCandidate = rawKind; kindStable = 0; }
      if (++kindStable >= cfg.stableCount) { voltKind = rawKind; kindStable = 0; }
    }
  } else {
    voltKindValid = false;
  }

  // Display debounce: commit to leadState only after the raw result repeats
  // cfg.stableCount passes in a row, so a single noisy test can't flip the
  // alert and cancel an in-progress LED flash.
  static LeadState candidate   = STATE_VOLTAGE;
  static int       stableCount = 0;
  if (rawState == leadState) {
    candidate   = rawState;
    stableCount = 0;
  } else {
    if (rawState != candidate) { candidate = rawState; stableCount = 0; }
    if (++stableCount >= cfg.stableCount) {
      leadState   = rawState;
      stableCount = 0;
    }
  }

  gatesMeasureEnd(gatesRaised);          // drop the pulsed gates again

  if (detLogOn) printDetLine(rawState);  // after the MOSFET is back on
}

// one detection pass for the test suite.
//   $DET,<ms>,<raw>,<lead>,<vpath>,<n>,<restMean>,<restMin>,<restMax>,
//        <metric>,<retms>,<areavms>,<thr>,<rawkind>,<kind>,<vpos>,<vneg>
// raw/lead: F float, C closed, V voltage (lead = after STABLECOUNT debounce).
// vpath: F fast single-read trip, A averaged trip, - no voltage, L VMODE 1
// (locked on), D VMODE 2 (disabled).  n = 0 means no resting reads were taken
// this pass, and the three rest fields are then blank.  The metric fields are
// blank when the open/closed test did not run (voltage present).  In method 1
// the recovery loop stops at the return, so areavms is only complete in
// method 2 -- which is why the suite characterises under method 2.
static char stateChar(LeadState s) {
  return s == STATE_FLOAT ? 'F' : (s == STATE_CLOSED ? 'C' : 'V');
}

void printDetLine(LeadState rawState) {
  Serial.print("$DET,");
  Serial.print(millis());             Serial.print(',');
  Serial.print(stateChar(rawState));  Serial.print(',');
  Serial.print(stateChar(leadState)); Serial.print(',');
  Serial.print(detVoltPath);          Serial.print(',');
  Serial.print(detRestN);             Serial.print(',');
  if (detRestN) {
    Serial.print(detRestMean, 5); Serial.print(',');
    Serial.print(detRestMin, 5);  Serial.print(',');
    Serial.print(detRestMax, 5);  Serial.print(',');
  } else {
    Serial.print(",,,");
  }
  if (detTestRan) {
    Serial.print(lastMetric, 5);   Serial.print(',');
    Serial.print(lastReturnMs, 4); Serial.print(',');
    Serial.print(lastAreaVms, 5);  Serial.print(',');
  } else {
    Serial.print(",,,");
  }
  Serial.print(activeThreshV, 5);
  // Voltage kind, appended (a host that stops at <thr> is unaffected).
  // rawkind = this pass's classification, blank if none ran (no voltage, or
  // VCLASS=0); kind = the debounced one the alert is playing (always shown,
  // meaningful only while lead is V); vpos/vneg = the peaks, blank with rawkind.
  Serial.print(',');
  if (voltKindRawFresh) Serial.print(voltKindName(voltKindRaw));
  Serial.print(',');
  Serial.print(voltKindName(voltKind));
  Serial.print(',');
  if (voltKindRawFresh) {
    Serial.print(lastVPosPeakV, 4); Serial.print(',');
    Serial.println(lastVNegPeakV, 4);
  } else {
    Serial.println(',');
  }
}

// ══════════════════════════════════════════════════════════════════
//  DIAGNOSTICS: status, streaming, capture
// ══════════════════════════════════════════════════════════════════
void printStatus() {
  Serial.print("$STATUS,diag=");  Serial.print(diagMode ? 1 : 0);
  Serial.print(",hwrev=");        Serial.print(cfg.hwRev);
  Serial.print(",vmode=");        Serial.print((int)voltOverride);
  Serial.print(",mosfet=");       Serial.print(mosfetHold);
  Serial.print(",stream=");       Serial.print(streamOn ? 1 : 0);
  Serial.print(",rate=");         Serial.print(streamIntervalMs);
  Serial.print(",capms=");        Serial.print(capDurationMs);
  Serial.print(",res=");          Serial.print(ADC_RESOLUTION);
  Serial.print(",vref=");         Serial.print(ADC_REF_VOLTAGE, 3);
  Serial.print(",dip=");          Serial.print(dipIdx);   // hwRev 3+: = THRESHSEL
  Serial.print(",spkdiff=");      Serial.print(buzzDifferential() ? 1 : 0);
  Serial.print(",amp=");          Serial.print(boardHasAmp() ? 1 : 0);
  Serial.print(",openthr=");      Serial.print(activeThreshV, 3);
  Serial.print(",detmethod=");    Serial.print(cfg.detectMethod);
  Serial.print(",metric=");       Serial.print(lastMetric, 4);
  Serial.print(",retms=");        Serial.print(lastReturnMs, 3);
  Serial.print(",areavms=");      Serial.print(lastAreaVms, 4);
  Serial.print(",vkind=");        Serial.print(voltKindName(voltKind));
  Serial.print(",vpos=");         Serial.print(lastVPosPeakV, 4);
  Serial.print(",vneg=");         Serial.print(lastVNegPeakV, 4);
  Serial.print(",negfix=");       Serial.print(cfg.negFix ? 1 : 0);
  Serial.print(",negv=");         Serial.print(cfg.negFixV, 3);
  Serial.print(",charge=");       Serial.print(chargeActive ? 1 : 0);
  Serial.print(",alertovr=");     Serial.print(alertOverride ? 1 : 0);
  Serial.print(",chginhibit=");   Serial.print(cfg.chargeInhibit ? 1 : 0);
  Serial.print(",muted=");        Serial.print(speakerMuted ? 1 : 0);
  Serial.print(",dirty=");        Serial.print(cfgDirty ? 1 : 0);
  Serial.print(",lp=");           Serial.print(lowPowerActive ? 1 : 0);
  // which sleep stage, and the stage-2 schedule actually in force.
  Serial.print(",lpstage=");      Serial.print(sleepStage);
  Serial.print(",deepsec=");      Serial.print(cfg.deepSec);
  Serial.print(",deephz=");       Serial.print(cfg.deepHz, 3);
  Serial.print(",deepms=");       Serial.print(deepTickMs);
  Serial.print(",deeppoll=");     Serial.print(deepPollTicks);
  Serial.print(",deeppark=");     Serial.print(cfg.deepParkOff);
  Serial.print(",armed=");        Serial.print(sleepArmed ? 1 : 0);
  Serial.print(",idle=");         Serial.print((millis() - idleSinceMs) / 1000);
  Serial.print(",floor=");        Serial.print(floorMode);
  Serial.print(",battpct=");      Serial.print(battPct);
  Serial.print(",battv=");        Serial.print(battV, 3);
  // Gate levels as two digits (ANA LED), the !GATE holds as two characters
  // (a = auto, 0/1 = held), and the running experiment step.  Always printed;
  // on V2/V3 there are no gates and the levels are bookkeeping only.
  Serial.print(",gates=");
  for (uint8_t i = 0; i < GATE_COUNT; i++) Serial.print(gateOn[i] ? 1 : 0);
  Serial.print(",gforce=");
  for (uint8_t i = 0; i < GATE_COUNT; i++)
    Serial.print(gateForce[i] < 0 ? "a" : (gateForce[i] ? "1" : "0"));
  Serial.print(",expt=");         Serial.print(exptStep);
  // debounced lead state (F/C/V) and whether !DETLOG is streaming.
  Serial.print(",lead=");         Serial.print(stateChar(leadState));
  Serial.print(",detlog=");       Serial.print(detLogOn ? 1 : 0);
  Serial.print(",sn=");           Serial.println(unitSN);  // last: may be empty
}

// One streamed sample: both pins independently + computed differential.
void streamSample() {
  // the live stream reads the analog front end, so it needs the same
  // rail the detector gets.  Without this, a pulsed (MODE 2) analog gate is
  // DOWN for every sample taken here -- the gate is only raised inside a
  // detection pass -- and the stream plots a flat, meaningless line.
  uint8_t gatesRaised = gatesMeasureBegin();

  analogRead(SENSE_POS);                 // throwaway: settle S/H after prior channel
  int rawPos = analogRead(SENSE_POS);
  if (!negFixed()) analogRead(SENSE_NEG);
  int rawNeg = readNegRaw();
  float pv = (rawPos / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
  float nv = (rawNeg / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
  Serial.print("$DIAG,");
  Serial.print(millis()); Serial.print(",");
  Serial.print(rawPos);   Serial.print(",");
  Serial.print(rawNeg);   Serial.print(",");
  Serial.print(pv, 4);    Serial.print(",");
  Serial.print(nv, 4);    Serial.print(",");
  Serial.println(pv - nv, 4);

  gatesMeasureEnd(gatesRaised);
}

// Capture both ADC pins as fast as possible (no settling delays) across a
// MOSFET toggle: baseline, toggle OFF at CAP_PRE_US, sample until durationMs
// or the buffer fills, restore ON, dump raw counts to the host.
//
// when a pulsed (MODE 2) gate is in play the capture also has to bring
// the analog rail up, and it does so INSIDE the sampled window rather than
// before it -- the first CAP_GATE_US of the trace is the rail down, then it
// rises, then the settle, and only then the MOSFET toggle.  That makes one
// capture show both things worth seeing: how long the front end actually takes
// to come good (which is what ANAUS should be set from), and the usual
// detection transient measured from a properly settled baseline.
//
// The whole rail-up phase is added to the requested duration rather than eaten
// out of it, so the post-toggle window is the length that was asked for.
void runCapture(unsigned long durationMs) {
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  delay(2);                              // settle to resting before baseline

  // Which gates this capture will have to raise, and how long they need.  Asked
  // BEFORE t0 so the timeline can be laid out, but not raised until inside the
  // loop -- gatesMeasureRaise() is called at CAP_GATE_US below.
  uint32_t gateSettle = 0;
  uint8_t  gateMask   = gatesPendingMask(&gateSettle);
  unsigned long gateAtUs   = gateMask ? CAP_GATE_US : 0;
  unsigned long toggleAtUs = gateMask ? (gateAtUs + gateSettle + CAP_PRE_US)
                                      : CAP_PRE_US;

  capCount = 0;
  // Keep the post-toggle window the length that was asked for: the rail-up
  // phase extends the capture rather than eating into the interesting part.
  unsigned long durUs = durationMs * 1000UL;
  if (gateMask) durUs += (toggleAtUs - CAP_PRE_US);

  unsigned long toggleUs = 0;
  unsigned long gateUs   = 0;
  bool toggled = false;
  bool gated   = (gateMask == 0);        // nothing to raise -> already "done"
  uint8_t raised = 0;
  unsigned long t0 = micros();

  while (capCount < CAP_MAX_SAMPLES) {
    unsigned long t = micros() - t0;
    if (!gated && t >= gateAtUs) {
      uint32_t ignored;
      raised = gatesMeasureRaise(&ignored);   // raise now, settle is sampled
      gateUs = micros() - t0;
      gated  = true;
    }
    if (!toggled && t >= toggleAtUs) {
      digitalWrite(MOSFET_PIN, MOSFET_OFF);
      toggleUs = micros() - t0;
      toggled = true;
    }
    if (t >= durUs) break;
    capT[capCount]   = t;
    capPos[capCount] = analogRead(SENSE_POS);
    capNeg[capCount] = readNegRaw();
    capCount++;
  }

  digitalWrite(MOSFET_PIN, MOSFET_ON);   // restore resting state
  gatesMeasureEnd(raised);               // put the rail back where it was

  Serial.print("$CAPSTART,");
  Serial.print(capCount);          Serial.print(",");
  Serial.print(toggleUs);          Serial.print(",");
  Serial.print(durUs / 1000UL);    Serial.print(",");
  Serial.print(ADC_FULL_SCALE, 0); Serial.print(",");
  Serial.print(ADC_REF_VOLTAGE, 3);
  // Appended, so an older host that stops reading at vref is unaffected.
  // gatemask 0 = nothing was gated and gateus is meaningless.
  Serial.print(",");               Serial.print(gateUs);
  Serial.print(",");               Serial.print(gateMask);
  Serial.print(",");               Serial.println(gateSettle);
  for (int i = 0; i < capCount; i++) {
    Serial.print("$CAP,");
    Serial.print(capT[i]);   Serial.print(",");
    Serial.print(capPos[i]); Serial.print(",");
    Serial.println(capNeg[i]);
  }
  Serial.println("$CAPEND");
}

// ═══════════════════════════════════════════════════════════════
//  PIN / GATE REPORTING AND THE POWER-PROFILE EXPERIMENT
// ═══════════════════════════════════════════════════════════════

// One $PIN row: what a function is wired to on this board.
static void printPinRow(const char *func, int pin) {
  Serial.print("$PIN,");
  Serial.print(func);
  Serial.print(",");
  if (!pinIsValid(pin)) { Serial.println("none,-"); return; }
  Serial.print(pin);
  Serial.print(",");
  Serial.println(pinNameOf(pin));
}

// !PINS -- the map this HWREV resolves to, then this core's name-to-number
// table.  Read-only: the map is a fact about the PCB, and HWREV is the only
// knob.  The row format is the bench fork's, so tools written against it
// (the bench GUI, the test suite) still parse it; functions a board does not
// have print "none".
void dumpPinMap() {
  Serial.print("$PINBOARD,");  Serial.println(cfg.hwRev);
  printPinRow("SENSEP",  SENSE_POS);
  printPinRow("SENSEN",  SENSE_NEG);
  printPinRow("CHARGE",  CHARGE_PIN);
  printPinRow("MOSFET",  MOSFET_PIN);
  printPinRow("SPKA",    SPEAKER_PIN);
  printPinRow("SPKB",    SPEAKER_PIN_B);
  printPinRow("LEDDATA", LED_PIN);
  printPinRow("DIPA",    DIP_PIN_A);
  printPinRow("DIPB",    DIP_PIN_B);
  printPinRow("PARK",    PARK_PIN);
  printPinRow("AMPDIN",  AMP_DIN_PIN);
  printPinRow("AMPEN1",  AMP_EN1_PIN);
  printPinRow("AMPEN2",  AMP_EN2_PIN);
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    char f[12];
    snprintf(f, sizeof(f), "GATE%s", GATE_NAME[i]);
    printPinRow(f, gatePin[i]);
  }
  // Fixed by the XIAO module itself -- listed so the map is a complete account
  // of what the firmware drives.
  Serial.print("$PIN,RGBEN,");  Serial.print(RGB_POWER_PIN);  Serial.println(",fixed");
  Serial.print("$PIN,BATTEN,"); Serial.print(BATT_EN_PIN);    Serial.println(",fixed");
  Serial.print("$PIN,BATTADC,");Serial.print(BATT_PIN);       Serial.println(",fixed");

  for (int i = 0; i < PIN_NAME_COUNT; i++) {
    Serial.print("$PINNAME,");
    Serial.print(PIN_NAMES[i].name);
    Serial.print(",");
    Serial.println(PIN_NAMES[i].num);
  }
  Serial.println("$PINEND");
}

// The two-stage sleep schedule, as derived rather than as requested.  Printed
// by !DEEP and at boot: DEEPHZ is snapped to the RTC ladder, so the only
// trustworthy statement of the rate is the one the board computes.
void dumpDeepSchedule() {
  Serial.print("$DEEP,stage=");     Serial.print(sleepStage);
  Serial.print(",deepsec=");        Serial.print(cfg.deepSec);
  Serial.print(",hz=");             Serial.print(cfg.deepHz, 3);
  Serial.print(",tickms=");         Serial.print(deepTickMs);
  Serial.print(",ticks=");          Serial.print(deepPollTicks);
  Serial.print(",probems=");        Serial.print((unsigned long)deepTickMs * deepPollTicks);
  // What stage 1 is doing, for comparison -- the point of the pair is the ratio.
  Serial.print(",lightms=");        Serial.print(cfg.sleepTickMs);
  Serial.print(",lightticks=");     Serial.print(cfg.sleepPollTicks);
  Serial.print(",lightprobems=");   Serial.print((unsigned long)cfg.sleepTickMs * cfg.sleepPollTicks);
  // The bridge: the raw key, and what it resolves to in each stage.  Printed
  // resolved because DEEPPARK=2 ("follow SLEEPPARK") is the default and reading
  // the key alone does not tell you what the pin is doing.
  Serial.print(",deeppark=");       Serial.print(cfg.deepParkOff);
  Serial.print(",lightbridge=");    Serial.print(cfg.sleepParkOff ? "off" : "resting");
  Serial.print(",deepbridge=");
  Serial.print((cfg.deepParkOff <= 1 ? cfg.deepParkOff != 0 : cfg.sleepParkOff != 0)
               ? "off" : "resting");
  Serial.print(",bridgenow=");      Serial.print(bridgeParkOff() ? "off" : "resting");
  Serial.print(",forced=");         Serial.println(deepForced ? 1 : 0);
}

// One $GATE row per gate.  `state` is what the pin is doing right now, which
// is not always what MODE says: a pulsed gate is down between measurements and
// a !GATE hold overrides the mode entirely.  pin=none on a board without it.
void dumpGates() {
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    Serial.print("$GATE,");
    Serial.print(GATE_NAME[i]);
    Serial.print(",pin=");
    if (gatePresent(i)) Serial.print(gatePin[i]); else Serial.print("none");
    Serial.print(",pol=");    Serial.print(gatePol[i]);
    Serial.print(",mode=");   Serial.print(*GATE_MODE[i]);
    Serial.print(",settleus=");Serial.print(gateSettleUs(i));
    Serial.print(",force=");  Serial.print(gateForce[i]);
    Serial.print(",state=");  Serial.println(gateOn[i] ? 1 : 0);
  }
}

// Apply the current step: set the holds, put the board in the right run mode,
// and announce the transition.  The announcement is flushed because the
// interesting runs happen with USB unplugged, where these lines are the only
// record of the plan -- see exptStart(), which prints the whole schedule up
// front for exactly that reason.
static void exptApply() {
  const ExptStep &s = EXPT_STEPS[exptStep];
  gateForce[GATE_ANA] = s.ana;
  gateForce[GATE_LED] = s.led;

  if (s.parked && !lowPowerActive)       enterLowPower();
  else if (!s.parked && lowPowerActive)  exitLowPower();
  // Put the parked steps on the stage they are meant to measure.  enterLowPower()
  // always lands in stage 1, so only the deep steps need moving -- but a deep
  // step followed by a light one does need moving back, hence both branches.
  if (s.parked) {
    if (s.deep && sleepStage != 2)       { enterDeepSleep(); deepForced = true; }
    else if (!s.deep && sleepStage == 2) exitDeepSleep();
    deepForced = s.deep ? true : false;  // the countdown must not re-decide
  }
  gatesRest(s.parked);

  exptStepStartMs = millis();
  exptParkTicks   = 0;
  Serial.print("$EXPT,");
  Serial.print(exptStep);      Serial.print(",");
  Serial.print(s.name);        Serial.print(",");
  Serial.println(exptStepSec);
  Serial.flush();
}

// Stop early or at the end: drop every hold, wake up, and go back to normal.
void exptEnd(const char *why) {
  if (exptStep < 0) return;
  exptStep = -1;
  for (uint8_t i = 0; i < GATE_COUNT; i++) gateForce[i] = -1;
  if (lowPowerActive) exitLowPower();
  gatesRest(false);
  idleSinceMs = millis();
  Serial.print("$EXPTEND,");
  Serial.println(why);
}

void exptStart(uint16_t sec) {
  exptStepSec = sec;
  // Print the schedule before anything moves.  A parked step is timed by
  // counting RTC ticks (millis() is frozen in Software Standby), so the real
  // boundaries land within one sleepTickMs of these offsets -- close enough to
  // slice a capture by, and the $EXPT markers confirm them whenever USB is
  // still attached.
  Serial.print("$EXPTPLAN,steps=");   Serial.print(EXPT_STEP_COUNT);
  Serial.print(",sec=");              Serial.print(exptStepSec);
  Serial.print(",totalsec=");         Serial.println((uint32_t)EXPT_STEP_COUNT * exptStepSec);
  for (int i = 0; i < EXPT_STEP_COUNT; i++) {
    Serial.print("$EXPTPLAN,");
    Serial.print(i);                                  Serial.print(",");
    Serial.print(EXPT_STEPS[i].name);                 Serial.print(",start=");
    Serial.print((uint32_t)i * exptStepSec);          Serial.print(",parked=");
    Serial.print(EXPT_STEPS[i].parked);               Serial.print(",deep=");
    Serial.println(EXPT_STEPS[i].deep);
  }
  Serial.flush();
  exptStep = 0;
  exptApply();
}

// Called from loop() ahead of everything else while a run is in progress.
void serviceExperiment() {
  const ExptStep &s = EXPT_STEPS[exptStep];
  bool done;

  if (s.parked) {
    lowPowerTick();                     // one wake period in Software Standby
    exptParkTicks++;
    // millis() does not advance in Standby, so a parked step is timed by
    // counting ticks of a known length.  currentTickMs(), not cfg.sleepTickMs:
    // a deep step's ticks are much longer, and timing it against the stage-1
    // period would make it run for a small fraction of the requested time.  The
    // rounded ms value (63 for the 1/16 s rung) costs under 1% and does not
    // accumulate across steps, since each step restarts the count.
    done = ((uint32_t)exptParkTicks * currentTickMs()
            >= (uint32_t)exptStepSec * 1000UL);
  } else {
    runDetection();
    updateChargeState();
    updateAlerts();
    updateSpeaker();
    sleepMs(cfg.loopDelayMs);
    done = (millis() - exptStepStartMs >= (unsigned long)exptStepSec * 1000UL);
  }

  // A host command aborts: this mode holds gates in states the board would
  // never choose for itself, so it must never be something you get stuck in.
  if (Serial.available()) { exptEnd("interrupted"); pollSerial(); return; }

  if (done) {
    if (++exptStep >= EXPT_STEP_COUNT) exptEnd("complete");
    else                               exptApply();
  }
}

// ══════════════════════════════════════════════════════════════════
//  SERIAL COMMAND HANDLING
// ══════════════════════════════════════════════════════════════════
void handleLine(char *line) {
  if (line[0] != '!') return;
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }
  for (char *p = cmd; *p; ++p) *p = toupper(*p);

  // ── Configuration ────────────────────────────────────────────
  if (strcmp(cmd, "SET") == 0) {
    // !SET,<key>,<value>  -- set a config field in RAM (clamped to its
    // min/max), applied immediately.  Persist with !SAVE.
    char *val = arg ? strchr(arg, ',') : NULL;
    if (!arg || !val) { Serial.println("$ERR,set,usage !SET,<key>,<value>"); return; }
    *val = '\0'; val++;
    for (char *p = arg; *p; ++p) *p = toupper(*p);
    const ConfigField *f = findField(arg);
    if (!f) { Serial.print("$ERR,set,unknown key "); Serial.println(arg); return; }
    fieldSet(f, atof(val));
    // The wake period can only take a value the RTC can produce, so snap it
    // before echoing -- otherwise the reply reports a rate that was never set.
    // The wake period can only take a value the RTC can produce, and the deep
    // rate is derived from the same ladder, so both are re-snapped here and the
    // snapped value echoed.  refreshSleepSchedule() also re-programs the RTC for
    // whichever stage is currently in force -- an edit made while the board is
    // asleep must not leave it running on the other stage's period.
    if (strcmp(f->name, "SLEEPTICKMS") == 0 || strcmp(f->name, "DEEPHZ") == 0)
      refreshSleepSchedule();
    // Either park key can be edited while the board is already parked, and
    // nothing else writes that pin until the next probe -- so apply it now.
    if (lowPowerActive &&
        (strcmp(f->name, "SLEEPPARK") == 0 || strcmp(f->name, "DEEPPARK") == 0))
      applyBridgePark();
    // HWREV re-assigns what several pins physically are, so the whole map has
    // to be re-applied before anything drives them again.
    if (strcmp(f->name, "HWREV") == 0) { silenceSpeaker(); applyBoardPins(); }
    // A gate's mode decides its resting level, so a change has to reach the pin
    // now rather than at the next sleep transition.
    if (strcmp(f->name, "ANAMODE") == 0 || strcmp(f->name, "LEDGMODE") == 0)
      gatesRest(gatesParked);
    // A1 is a no-connect on V3, so a live SENSE_NEG read there is just a
    // floating pin.  Allowed (it is occasionally worth looking at on the
    // bench) but never silent -- this is otherwise a baffling failure.  On V3b
    // there is no SENSE_NEG pin at all and NEGFIX=0 is simply ignored.
    if (strcmp(f->name, "NEGFIX") == 0 && !cfg.negFix && boardIsV3())
      Serial.println("$ERR,set,NEGFIX=0 with HWREV=3: A1 is not connected on V3");
    if (strcmp(f->name, "NEGFIX") == 0 && !cfg.negFix && boardHasAmp())
      Serial.println("$ERR,set,NEGFIX=0 has no effect on HWREV=4: A1 is the charge sense");
    cfgDirty = true;
    printField(f);                       // echo the (possibly clamped) value
  } else if (strcmp(cmd, "GET") == 0) {
    if (!arg) { Serial.println("$ERR,get,usage !GET,<key>"); return; }
    for (char *p = arg; *p; ++p) *p = toupper(*p);
    const ConfigField *f = findField(arg);
    if (!f) { Serial.print("$ERR,get,unknown key "); Serial.println(arg); return; }
    printField(f);
  } else if (strcmp(cmd, "CFG") == 0) {
    for (int i = 0; i < CFG_FIELD_COUNT; i++) printField(&CFG_FIELDS[i]);
    Serial.println("$CFGEND");
  } else if (strcmp(cmd, "SAVE") == 0) {
    configSave();
    // Verify: read the flash image back and compare byte-for-byte, so a
    // failed/incomplete data-flash write reports $ERR instead of a false $OK.
    Config check;
    EEPROM.get(CFG_EEPROM_ADDR, check);
    if (memcmp(&check, &cfg, sizeof(Config)) == 0) {
      Serial.println("$OK,save");
    } else {
      Serial.println("$ERR,save,verify failed (flash readback mismatch)");
    }
  } else if (strcmp(cmd, "LOAD") == 0) {
    if (configLoad()) {
      silenceSpeaker();
      applyBoardPins();                  // the reloaded image may change hwRev
      refreshSleepSchedule();            // ...or either sleep stage's rate
      Serial.println("$OK,load");
    } else {
      Serial.println("$ERR,load,stored config invalid");
    }
  } else if (strcmp(cmd, "DEFAULTS") == 0) {
    // HWREV describes the PCB this XIAO is plugged into, not a preference, so
    // it survives a factory reset the way the serial number does.  Letting it
    // revert to the default 4 would hand D8/D10 to outputs on a V2 board and
    // drive them push-pull into whatever its DIP switches are doing.  Change
    // it deliberately with !SET,HWREV if a board is genuinely rebuilt.
    uint8_t keepHwRev = cfg.hwRev;
    configDefaults();
    cfg.hwRev = keepHwRev;
    silenceSpeaker();
    applyBoardPins();                    // gate modes revert with the rest
    refreshSleepSchedule();
    cfgDirty = true;                     // RAM now differs from EEPROM
    Serial.println("$OK,defaults");
  } else if (strcmp(cmd, "SN") == 0) {
    // !SN            -> report the stored serial number ($SN,<value>)
    // !SN,<value>    -> write it to EEPROM (persists immediately; it is
    //                   device identity, not part of the tunable config).
    if (arg) {
      while (*arg == ' ') arg++;         // tolerate a leading space
      if (*arg == '\0') {
        Serial.println("$ERR,sn,empty");
      } else if (strchr(arg, ',')) {
        Serial.println("$ERR,sn,comma not allowed");   // keeps host CSV clean
      } else if (strlen(arg) > SN_MAX_LEN - 1) {
        Serial.print("$ERR,sn,too long (max ");
        Serial.print(SN_MAX_LEN - 1);
        Serial.println(")");
      } else {
        snSave(arg);
        Serial.print("$SN,");  Serial.println(unitSN);
        Serial.println("$OK,sn");
      }
    } else {
      Serial.print("$SN,");  Serial.println(unitSN);
    }

  // ── Diagnostics / overrides ──────────────────────────────────
  } else if (strcmp(cmd, "DIAG") == 0) {
    diagMode = arg ? (atoi(arg) != 0) : !diagMode;
    if (!diagMode) { streamOn = false; mosfetHold = -1; }
    printStatus();
  } else if (strcmp(cmd, "STREAM") == 0) {
    streamOn = arg ? (atoi(arg) != 0) : !streamOn;
    printStatus();
  } else if (strcmp(cmd, "RATE") == 0) {
    if (arg) { long r = atol(arg); streamIntervalMs = (r < 1) ? 1 : r; }
    printStatus();
  } else if (strcmp(cmd, "VMODE") == 0) {
    int v = arg ? atoi(arg) : 0;
    voltOverride = (VoltOverride)constrain(v, 0, 2);
    printStatus();
  } else if (strcmp(cmd, "MOSFET") == 0) {
    int m = arg ? atoi(arg) : -1;
    mosfetHold = (m < 0) ? -1 : (m ? 1 : 0);
    printStatus();
  } else if (strcmp(cmd, "ALERTS") == 0) {
    // Re-enable normal alerts while charging (override auto-clears on unplug).
    alertOverride = arg ? (atoi(arg) != 0) : !alertOverride;
    printStatus();
  } else if (strcmp(cmd, "SLEEP") == 0) {
    // !SLEEP  arm the low-power timeout to fire as soon as it is allowed --
    // i.e. expire the countdown now, so the board sleeps the moment it is idle
    // and off USB.  Arm it over USB, then unplug (same pattern as !OLOG).
    // The timeout length and probe cadence are config (SLEEPSEC / SLEEPTICKS
    // / SLEEPAVG / SLEEPHB / SLEEPPARK), set with !SET and kept with !SAVE.
    if (arg && atoi(arg) == 0) {
      sleepArmed = false;                // !SLEEP,0 -- cancel a pending arm
      idleSinceMs = millis();
      Serial.println("$OK,sleep,disarmed");
    } else if (cfg.idleTimeoutS == 0) {
      Serial.println("$ERR,sleep,disabled (set SLEEPSEC > 0)");
    } else {
      sleepArmed = true;
      Serial.println("$OK,sleep,armed");
    }
    printStatus();
  } else if (strcmp(cmd, "SLEEPLOG") == 0) {
    // !SLEEPLOG    dump every probe the board made while it was asleep
    // !SLEEPLOG,0  clear the log
    // Rows are $SLOG,<i>,<state>,<metric>,<thr>,<retms>,<rest> -- oldest first.
    // This is how you see what a CLOSED lead actually measures off USB, where
    // the ground reference (and therefore the metric) is not what it is on the
    // bench.  Tune the DIP threshold against these numbers, not the USB ones.
    // !SLEEPLOG,D  detailed dump ($SLOGD rows -- see dumpSleepLogDetail)
    // !SLEEPLOG,2  clear, and take every VOLTAVG read per pass (full rest
    //              stats for offline replay; the decision is unchanged)
    // The D test comes first: atoi("D") is 0, which would otherwise clear.
    if (arg && toupper(arg[0]) == 'D') {
      dumpSleepLogDetail();
    } else if (arg && (atoi(arg) == 0 || atoi(arg) == 2)) {
      slogValid = 0; slogHead = 0; slogTotal = 0;
      slogFullReads = (atoi(arg) == 2);
      Serial.print("$OK,sleeplog,cleared,fullreads=");
      Serial.println(slogFullReads ? 1 : 0);
    } else {
      dumpSleepLog();
    }
  } else if (strcmp(cmd, "SLEEPTEST") == 0) {
    // Run one sleeping-mode probe right now, awake and over USB, and report
    // exactly what it decided.  The probe normally runs unplugged and silent,
    // so this is the only way to see its numbers.  Short the leads and compare
    // with the periodic debug line: if this reports CLOSED but a sleeping unit
    // still won't wake, the difference is the standby wake itself rather than
    // the probe logic.
    LeadState s = lowPowerProbe();
    digitalWrite(MOSFET_PIN, MOSFET_ON);      // undo the probe's park
    Serial.print("$SLEEPTEST,");
    Serial.print(s == STATE_FLOAT ? "FLOAT" : (s == STATE_CLOSED ? "CLOSED" : "VOLTAGE"));
    Serial.print(",metric=");  Serial.print(lastMetric, 4);
    Serial.print(",thr=");     Serial.print(lastProbeThreshV, 4);   // wake threshold used
    Serial.print(",awakethr="); Serial.print(activeThreshV, 4);
    Serial.print(",rest=");    Serial.print(lastRestV, 4);
    Serial.print(",retms=");   Serial.print(lastReturnMs, 3);
    Serial.print(",areavms="); Serial.print(lastAreaVms, 4);
    Serial.print(",method=");  Serial.println(cfg.detectMethod);
  } else if (strcmp(cmd, "VTEST") == 0) {
    // Run one voltage classification right now and report the peaks it decided
    // on.  Verify on the bench that a known DC source reads one-sided and that
    // mains reads two-sided BEFORE trusting the pattern, and use vpos/vneg
    // against band to pick an ACBAND.  Runs regardless of VCLASS.
    // this reads the ADC, so it raises the pulsed analog gate like every
    // other ADC consumer -- missing that is what flat-lined !CAP and !STREAM.
    uint8_t gatesRaised = gatesMeasureBegin();
    uint8_t savedClassify = cfg.voltClassify;
    cfg.voltClassify = 1;
    digitalWrite(MOSFET_PIN, MOSFET_ON);      // resting, as the normal path is
    VoltKind k = classifyVoltage();
    cfg.voltClassify = savedClassify;
    gatesMeasureEnd(gatesRaised);
    float band = (cfg.acBandV > 0.0f) ? cfg.acBandV
                                      : cfg.voltFastMult * cfg.refBandV;
    Serial.print("$VTEST,");
    Serial.print(voltKindName(k));
    Serial.print(",vpos=");     Serial.print(lastVPosPeakV, 4);
    Serial.print(",vneg=");     Serial.print(lastVNegPeakV, 4);
    Serial.print(",band=");     Serial.print(band, 4);
    Serial.print(",winms=");    Serial.print(cfg.acWindowMs);
    Serial.print(",n=");        Serial.print(lastVSamples);
    Serial.print(",classify="); Serial.println(savedClassify ? 1 : 0);
  } else if (strcmp(cmd, "FLOOR") == 0) {
    // !FLOOR,<0-3>  park the board in a fixed state for a current measurement:
    //   0 = exit    1 = parked, bridge resting, standby
    //   2 = parked, bridge off, standby       3 = parked, bridge resting, WFI only
    // Take the reading on battery with the meter in series; the deltas between
    // levels are what say which loads are worth switching in hardware.
    floorMode = arg ? constrain(atoi(arg), 0, 3) : 0;
    printStatus();
    Serial.flush();              // last words before the board goes quiet
  } else if (strcmp(cmd, "DEEP") == 0) {
    // !DEEP     report the two-stage schedule
    // !DEEP,1   drop into stage 2 right now (only while already asleep)
    // !DEEP,0   go back up to stage 1
    // Forcing is for the bench: it removes the DEEPSEC wait so a meter reading
    // of stage 2 can be taken immediately.  A forced stage survives until
    // something wakes the board, which resets to stage 1 like any other wake.
    if (arg) {
      if (!lowPowerActive) {
        Serial.println("$ERR,deep,not asleep (arm the sleep first with !SLEEP)");
      } else if (atoi(arg) == 0) {
        exitDeepSleep();
        Serial.println("$OK,deep,stage 1");
      } else {
        enterDeepSleep();
        deepForced = true;             // don't let the countdown re-decide
        Serial.println("$OK,deep,stage 2");
      }
    }
    dumpDeepSchedule();
  } else if (strcmp(cmd, "PINS") == 0) {
    // !PINS -- the pin map this HWREV resolves to (read-only) plus this core's
    // name-to-number table.
    dumpPinMap();
  } else if (strcmp(cmd, "GATE") == 0) {
    // !GATE                     report both gates
    // !GATE,<ANA|LED>           report them (same output)
    // !GATE,<name>,<0|1|-1>     hold it off / on, or -1 = follow its MODE
    // A hold is RAM only and survives into sleep and !FLOOR, which is how the
    // sleeping current for a given gating scheme gets measured: hold the gates
    // where you want them, then !SLEEP or !FLOOR and unplug.
    if (!arg) { dumpGates(); return; }
    char *val = strchr(arg, ',');
    if (val) { *val = '\0'; val++; }
    for (char *p = arg; *p; ++p) *p = toupper(*p);
    int idx = -1;
    for (uint8_t i = 0; i < GATE_COUNT; i++)
      if (strcmp(arg, GATE_NAME[i]) == 0) idx = i;
    if (idx < 0) { Serial.print("$ERR,gate,unknown gate "); Serial.println(arg); return; }
    if (val) {
      int v = atoi(val);
      gateForce[idx] = (v < 0) ? -1 : (v ? 1 : 0);
      gatesRest(gatesParked);            // re-settle every gate under the new hold
      if (gateForce[idx] >= 0 && !gatePresent(idx))
        Serial.println("$ERR,gate,held but this board has no such rail (HWREV 4 only)");
    }
    dumpGates();
  } else if (strcmp(cmd, "EXPT") == 0) {
    // !EXPT[,<sec>]  run the ten-step power profile, <sec> per step (10 s
    //                default, 1-600).  !EXPT,0 aborts.
    // Arm it over USB, read the printed plan, then unplug and let it run on
    // battery through the meter -- the plan's start offsets are what slice the
    // capture.  Any serial byte aborts, so replugging ends the run.
    long sec = arg ? atol(arg) : 10;
    if (arg && sec == 0) {
      if (exptStep < 0) Serial.println("$ERR,expt,not running");
      else              exptEnd("aborted");
    } else {
      if (sec < 1)   sec = 1;
      if (sec > 600) sec = 600;
      // !FLOOR and !EXPT are two ways of holding the board still for a meter
      // and they would fight over it -- loop() gives the experiment priority,
      // so a floorMode left set would simply be ignored rather than obeyed.
      if (floorMode) Serial.println("$ERR,expt,exit !FLOOR first (!FLOOR,0)");
      else           exptStart((uint16_t)sec);
    }
  } else if (strcmp(cmd, "CAP") == 0) {
    unsigned long d = arg ? atol(arg) : capDurationMs;
    if (d < 1) d = 1;
    capDurationMs = d;
    runCapture(d);
  } else if (strcmp(cmd, "TONE") == 0) {
    // !TONE[,<vol>[,<ms>[,<hz>]]]  one test pulse, now.  For setting the
    // volume keys by ear: it deliberately ignores BEEP, the boot mute and the
    // charge lockout -- all three are normally in force exactly when you are
    // sitting at the bench on USB.  Defaults: VOLTVOL, 200 ms, VOLTFREQ.
    // Blocking, like !CAP: the loop stops for <ms>.  SPKDUTY applies (V3b).
    long v = cfg.voltVol, ms = 200, hz = cfg.voltFreqHz;
    if (arg) {
      char *a2 = strchr(arg, ',');
      if (a2) { *a2++ = '\0'; }
      if (*arg) v = atol(arg);
      if (a2) {
        char *a3 = strchr(a2, ',');
        if (a3) { *a3++ = '\0'; }
        if (*a2) ms = atol(a2);
        if (a3 && *a3) hz = atol(a3);
      }
    }
    v  = constrain(v, 0, 3);
    ms = constrain(ms, 1, 2000);
    hz = constrain(hz, 100, 10000);
    silenceSpeaker();
    speakerOn((unsigned int)hz, (uint8_t)v);
    sleepMs((unsigned long)ms);
    speakerOff();
    Serial.print("$OK,tone,vol=");  Serial.print(v);
    Serial.print(",ms=");           Serial.print(ms);
    Serial.print(",hz=");           Serial.print(hz);
    Serial.print(",duty=");         Serial.print(boardHasAmp() ? cfg.spkDuty : 50);
    Serial.print(",amp=");          Serial.println(boardHasAmp() ? 1 : 0);
  } else if (strcmp(cmd, "DETLOG") == 0) {
    // !DETLOG[,0|1] -- one $DET line per detection pass (see printDetLine).
    // Bare = toggle.  RAM only; off after a reset.
    detLogOn = arg ? (atoi(arg) != 0) : !detLogOn;
    Serial.print("$OK,detlog,"); Serial.println(detLogOn ? 1 : 0);
  } else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "?") == 0) {
    printStatus();
  } else {
    Serial.print("$ERR,unknown,"); Serial.println(cmd);
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; handleLine(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  // Battery sense first.  The BAT_READ_EN divider needs time to settle before
  // the power-on charge cue can read it; starting it here lets that settle
  // overlap the rest of init rather than being paid for as its own delay.
  pinMode(BATT_PIN, INPUT);              // BAT_DET_PIN (P105) = Vbatt/2 sense
  pinMode(BATT_EN_PIN, OUTPUT);          // BAT_READ_EN (P400)
  digitalWrite(BATT_EN_PIN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //Turn User LED Off, HIGH=Off

  analogReadResolution(ADC_RESOLUTION);

  // Config BEFORE any board pin, because cfg.hwRev decides what the pins are
  // -- including which one is the charge sense the USB check below reads (A3
  // on V2/V3, A1 on V3b).  Reading EEPROM needs neither Serial nor a settle.
  configDefaults();
  bool loaded   = configLoad();
  bool migrated = false;
  if (!loaded) {
    // Not a current-version image.  Before falling back to factory defaults,
    // see whether it is a valid older layout worth upgrading -- a unit tuned in
    // the field must not lose its thresholds just because the struct grew.
    // Newest layout first, so an image is never mis-read as something older.
    // v6-v8 carry their stored hwRev across; v5 and older pin it to 2,
    // because a config in one of those layouts means a pre-V3 unit.
    migrated = configMigrateV8() || configMigrateV7() || configMigrateV6() ||
               configMigrateV5() || configMigrateV4() || configMigrateV3() ||
               configMigrateV2();
    // A migrated unit is a field unit with a sleep behaviour its owner knows.
    // The deep stage is new in v9, so it starts OFF for them and has to be
    // turned on deliberately (!SET,DEEPSEC,5); only a blank board gets it.
    if (migrated) cfg.deepSec = 0;
    configSave();                        // persist the migration (or seed defaults)
  }
  snLoad();                              // unit serial number (separate block)

  // One call configures every board pin: sense, MOSFET, charge sense, LED data,
  // the gates at their resting level, and (via buzzerApplyPinModes) the
  // speaker / amp / DIP / park pins.
  applyBoardPins();

  Serial.begin(115200);
  // The USB settle only buys anything when a host is actually attached.  On
  // battery -- the case that decides time-to-first-measurement -- it is pure
  // dead time, so gate it on VBUS.  The literal threshold keeps this check
  // independent of a mistuned CHGTHRESH.
  readChargeV();                         // throwaway: first conversion after reset
  if (readChargeV() > BOOT_USB_THRESH_V) delay(300);

  // (BATT_PIN / BATT_EN_PIN are set up at the top of setup(), so the battery
  //  divider is already settling while the rest of this runs.)

  pinMode(RGB_POWER_PIN, OUTPUT);        // onboard NeoPixel power rail
  digitalWrite(RGB_POWER_PIN, HIGH);

  pixel.begin();
  setPixel(0, 0, 0);                     // not pixel.show(): LED1's rail may be down

  startupBatteryIndicate();              // power-on battery charge-level cue

  bootSpeakerMuteCheck();                // leads CLOSED at boot -> session mute
  Serial.print("SN: ");
  Serial.println(unitSN[0] ? unitSN : "(unassigned -- write with !SN,<value>)");
  Serial.print("Config: ");
  if (loaded)        Serial.println("loaded from EEPROM (v9)");
  else if (migrated) Serial.println("MIGRATED from an older layout (tuning preserved; "
                                    "new keys at defaults, DEEPSEC=0)");
  else               Serial.println("defaults (EEPROM seeded)");
  Serial.print("Board: HWREV ");
  Serial.print(cfg.hwRev);
  if (boardIsV2())      Serial.println(" (V2: DIP switches, single-ended buzzer)");
  else if (boardIsV3()) Serial.println(" (V3: THRESHSEL, anti-phase buzzer, no gates)");
  else                  Serial.println(" (V3b: THRESHSEL, PAM8904 amp, ANA/LED rail gates)");
  Serial.print("Speaker: ");
  Serial.print(speakerMuted ? "MUTED (leads closed at boot)" : "enabled");
  if (boardHasAmp()) {
    Serial.print(", PAM8904 cont vol ");  Serial.print(cfg.contVol);
    Serial.print(", volt vol ");          Serial.print(cfg.voltVol);
    Serial.print(", duty ");              Serial.print(cfg.spkDuty);
    Serial.println(" %");
  }
  else if (!cfg.passiveBuzzer)   Serial.println(", DC drive (PASSIVE=0)");
  else if (buzzDifferential())   Serial.println(", anti-phase D8/D9");
  else                           Serial.println(", single-ended D9");
  Serial.print(boardIsV2() ? "DIP: " : "Threshold: THRESHSEL ");
  Serial.print(readDipIndex());
  Serial.print(" -> threshold ");
  Serial.println(cfg.thresh[readDipIndex()], 3);

  if (!cfg.chargeInhibit)
    Serial.println("Charge inhibit: OFF (CHGINHIBIT=0) -- 5 V on the input is "
                   "reported but blocks nothing: sleep and alerts run as on battery");

  // Bring up the sleep timer / wake source.  A failure is not fatal: the
  // timeout mode falls back to a WFI idle, which still parks every load.
  bool lpOk = lowPowerInit();
  idleSinceMs = millis();
  Serial.print("Sleep: ");
  if (cfg.idleTimeoutS == 0) {
    Serial.println("disabled (SLEEPSEC=0)");
  } else {
    Serial.print(cfg.idleTimeoutS);
    Serial.print(" s timeout, probe every ");
    Serial.print((unsigned long)cfg.sleepTickMs * cfg.sleepPollTicks);
    Serial.println(lpOk ? " ms (standby)" : " ms (WFI fallback -- RTC/LPM init failed)");
  }

  // Derive the stage-2 schedule now so cfg.deepHz holds the achieved rate from
  // the first !CFG, not the requested one.
  computeDeepSchedule();
  Serial.print("Deep sleep: ");
  if (cfg.deepSec == 0) {
    Serial.println("disabled (DEEPSEC=0) -- single-stage");
  } else {
    Serial.print("stage 2 after ");
    Serial.print(cfg.deepSec);
    Serial.print(" s of stage 1, probing at ");
    Serial.print(cfg.deepHz, 3);
    Serial.print(" Hz (");
    Serial.print(deepTickMs);
    Serial.print(" ms tick x ");
    Serial.print(deepPollTicks);
    Serial.print("), bridge ");
    if (cfg.deepParkOff > 1) {
      Serial.print(cfg.sleepParkOff ? "off" : "resting");
      Serial.println(" (DEEPPARK=2, following SLEEPPARK)");
    } else {
      Serial.println(cfg.deepParkOff ? "parked OFF" : "resting");
    }
  }

  if (boardHasAmp()) {
    Serial.println("Power gates:");
    dumpGates();
  }

  Serial.println("BlinkyHawk_Unified ready.");
}

// ══════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════════════
void loop() {
  pollSerial();

  // Measurement parking and the sleeping loop each own the board completely --
  // they run before everything else and return without touching detection.
  // The experiment sequencer goes first of all: its parked steps set
  // lowPowerActive themselves and must not be handed to serviceLowPower(),
  // which would probe, wake, and end the step early.
  if (exptStep >= 0)  { serviceExperiment(); return; }
  if (floorMode)      { serviceFloorMode(); return; }
  if (lowPowerActive) { serviceLowPower();  return; }

  // ── Diagnostic mode ─────────────────────────────────────────
  if (diagMode) {
    if (mosfetHold >= 0) {
      // Manual MOSFET hold: detection paused, pin parked for observation.
      digitalWrite(MOSFET_PIN, mosfetHold ? MOSFET_ON : MOSFET_OFF);
    } else {
      runDetection();          // detection still runs (LED stays meaningful)
    }

    if (streamOn && (millis() - lastStreamMs >= streamIntervalMs)) {
      lastStreamMs = millis();
      streamSample();
    }

    updateChargeState();
    updateAlerts();
    updateSpeaker();
    delay(1);                  // light idle; streaming sets its own pace
    return;
  }

  // ── Normal mode ─────────────────────────────────────────────
  runDetection();
  updateChargeState();
  updateAlerts();
  updateSpeaker();

  // Periodic human-readable debug
  if (millis() - lastSerialTime >= serialInterval) {
    lastSerialTime = millis();
    Serial.print("Rest:");
    Serial.print(lastRestV, 3);
    Serial.print("V  ");
    if (leadState == STATE_VOLTAGE) {
      // Peaks alongside the kind: the two numbers are what the call was made
      // on, so a wrong pattern can be diagnosed from this line alone.
      Serial.print("-> VOLTAGE (bypass) ");
      Serial.print(voltKindName(voltKind));
      Serial.print("  +pk:"); Serial.print(lastVPosPeakV, 3);
      Serial.print("V -pk:"); Serial.print(lastVNegPeakV, 3);
      Serial.println("V");
    } else {
      // Metric + units depend on the active detection method; print the metric
      // that was actually thresholded plus the raw return/area for tuning.
      Serial.print("m");   Serial.print(cfg.detectMethod);
      Serial.print(" metric:");
      if      (cfg.detectMethod == 1) { Serial.print(lastMetric, 3); Serial.print("ms"); }
      else if (cfg.detectMethod == 2) { Serial.print(lastMetric, 4); Serial.print("Vms"); }
      else                            { Serial.print(lastMetric, 3); Serial.print("V"); }
      Serial.print(" (ret:"); Serial.print(lastReturnMs, 3);
      Serial.print("ms area:"); Serial.print(lastAreaVms, 4);
      Serial.print("Vms thr:"); Serial.print(activeThreshV, 3);
      Serial.print(")  -> ");
      Serial.println(leadState == STATE_FLOAT ? "FLOATING" : "CLOSED");
    }
  }

  // Log awake samples while on battery, at the same cadence the sleeping probe
  // uses.  This is what makes the baseline shift visible: unplug, let it run
  // awake for a while, let it sleep, replug and compare the AWAKE and SLEEP
  // rows in !SLEEPLOG.  Skipped on USB -- the host can already see those.
  // Paced by the PROBE interval (tick x ticks-per-probe), not the raw tick: on
  // the fast rungs a per-tick sample would overwrite the whole 64-entry ring
  // several times a second, leaving nothing of the run to compare against. The
  // 100 ms floor holds even if the probe interval itself is shorter than that.
  unsigned long slogEveryMs =
      (unsigned long)cfg.sleepTickMs * cfg.sleepPollTicks;
  if (slogEveryMs < 100UL) slogEveryMs = 100UL;
  if (!chargeInhibits() && millis() - lastSlogAwakeMs >= slogEveryMs) {
    lastSlogAwakeMs = millis();
    slogRecord(leadState, true);
  }

  // Inactivity timer.  Anything other than an open lead counts as activity, as
  // does any state in which sleeping is disallowed -- so the countdown starts
  // fresh once the board is idle AND allowed to sleep, rather than expiring
  // while it was busy or on USB.  An armed !SLEEP is exempt from the reset:
  // it has to survive being issued over USB until the unit is unplugged.
  if (leadState != STATE_FLOAT || !lowPowerAllowed()) {
    if (!sleepArmed) idleSinceMs = millis();
  } else if (sleepArmed ||
             millis() - idleSinceMs >= (unsigned long)cfg.idleTimeoutS * 1000UL) {
    sleepArmed = false;
    enterLowPower();
    return;
  }

  sleepMs(cfg.loopDelayMs);    // pace the loop with a real CPU idle (WFI)
}
