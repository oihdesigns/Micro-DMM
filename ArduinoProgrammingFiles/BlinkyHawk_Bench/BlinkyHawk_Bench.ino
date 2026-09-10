/*
 * BlinkyHawk_Bench.ino
 *
 * EXPERIMENTAL BENCH FORK of BlinkyHawk_RA4M1.ino.  Not production firmware:
 * this is the build for a XIAO RA4M1 and a Blinky Hawk PCB sitting on a
 * breadboard as two separate parts, wired together with jumpers, so that
 * individual parts of the circuit can be cut into and measured.
 *
 * Two things it adds, and the reasons for them:
 *
 *   1. A RUNTIME PIN MAP.  Every pin the firmware touches is an EEPROM key
 *      (PINMOSFET, PINSPKA, PINSPKB, PINLEDDAT, PINSENSEP, ...) instead of a
 *      compile-time constant, so a jumper can be moved and the firmware told
 *      about it with !SET + !SAVE rather than a recompile.  !PINS dumps the
 *      live map plus the D0-D10 / A0-A5 name-to-number table for this core.
 *
 *   2. THREE POWER GATES.  Spare GPIOs (PINANA / PINLEDG / PINAUX) drive
 *      external load switches so a section of the circuit can be powered down
 *      and the saving measured.  The two that motivated this:
 *        ANA  the analog front end -- the TL431 reference bias and the divider
 *             strings, which on the real PCB sit permanently across the rail
 *             and are the prime candidate for a hardware gate.
 *        LED  LED1's supply.  On OpenLead_Headless V3 LED1 hangs off
 *             BatteryRail with nothing in series, so its quiescent draw is a
 *             hardware floor firmware cannot reach.  On the breadboard it can.
 *      Each gate has a polarity (POL), a settle time (US/MS) and a MODE:
 *        0 ALWAYS  held on -- the "no gating" control case
 *        1 SLEEP   on while awake, dropped while asleep or in !FLOOR
 *        2 PULSED  off at rest, raised only for the settle+measure window
 *   3. A SECOND SLEEP STAGE.  The production firmware has one sleeping state,
 *      which is now the normal operating mode (~2 mA, waking ~16x/s to probe).
 *      This fork adds a deeper one underneath it: after DEEPSEC seconds during
 *      which stage 1 has only ever seen open leads, the probe rate drops to
 *      DEEPHZ (default 1 Hz) and the RTC wake period is re-programmed to match,
 *      so the board genuinely wakes less often rather than merely skipping
 *      probes.  Anything that ends stage 1 -- a connected lead, USB, a serial
 *      byte -- ends stage 2 the same way, and the next sleep starts at stage 1.
 *      DEEPSEC = 0 disables it, giving exactly the production behaviour.
 *
 *      Modes 1 and 2 perturb the measurement -- rail loading moves the resting
 *      differential, which is the whole reason the sleeping probe powers its
 *      rails back up (see lowPowerProbe).  That is what the experiment is for,
 *      so expect to re-tune THRESH/SLEEPTHR per gating scheme and compare the
 *      current saving against the detection cost, rather than reusing one
 *      threshold across all of them.
 *
 * Plus the bench-only commands !PINS, !GATE and !EXPT (a scripted eight-step
 * power profile for slicing a PPK2 capture).  Everything else -- detection,
 * alerts, sleep, the config system, the GUI protocol -- is the production
 * firmware unchanged, so numbers taken here transfer back.
 *
 * ** ITS EEPROM IS A SEPARATE BLOCK. **  Magic "BHKX" at address 1024, not
 * "BHK1" at 0, so flashing this onto a real unit does NOT touch that unit's
 * production config, and flashing production back restores it untouched.  The
 * unit serial-number block (512) is shared and read normally.  There are no
 * config migrations here: a bench config is not worth carrying forward, and an
 * unrecognised image is simply replaced with defaults.
 *
 * ------------------------------------------------------------------
 * Target: Seeed XIAO RA4M1 ONLY (Renesas RA4M1, 14-bit ADC).
 * Descended from OpenLeadDetect_XIAO_Minimal with the multi-MCU support and
 * the A5 potentiometer / adaptive-threshold features removed.
 *
 * "Pseudo-differential": analogRead() has no native differential mode, so
 * SENSE_POS and SENSE_NEG are each sampled vs. GND and subtracted in
 * software.  Resting differential is ~0 V, so detection works on the
 * magnitude of the deviation from zero.
 *
 * Measurement logic (repeated continuously):
 *   1. MOSFET held HIGH (resting / bridge connected), read the differential.
 *   2. Voltage-present decision (de-noised):
 *        - any single read beyond VOLTFAST * REFBAND -> present;
 *        - otherwise average VOLTAVG reads and compare to the band.
 *      If voltage is present the open/closed test is bypassed.
 *   3. Test (only when no voltage):  MOSFET LOW, derive an open/closed metric,
 *      MOSFET back HIGH.  metric > active threshold -> OPEN (blue), else CLOSED
 *      (green).  How the metric is derived is selectable (cfg.detectMethod):
 *        0 SINGLE  : one differential read after settlePreUs; metric = |diff| (V)
 *                    -- the original method (default; unchanged behaviour).
 *        1 TIMERET : sample the recovery; metric = time (ms) for the differential
 *                    to return within detReturnBand of the resting centre.  A
 *                    dead short recovers fastest, an open lead slowest, so a
 *                    LONGER time = MORE OPEN.
 *        2 AREA    : metric = tail-windowed integral (V*ms) of |diff-centre|
 *                    from detAreaStartUs to detWindowUs.  LARGER area = MORE OPEN.
 *      All three keep "larger metric = more open", so the DIP threshold table
 *      and the compare are shared -- but the threshold's UNITS change with the
 *      method (V / ms / V*ms), so THRESH00..11 must be re-tuned after a switch.
 *
 * ── Board revisions (HWREV) ───────────────────────────────────────
 * Two PCBs share this firmware; HWREV (EEPROM config) says which one is under
 * the XIAO, and it is read before any pin is configured because it decides
 * what D8 physically IS.  Getting it wrong is not cosmetic: HWREV 3 on a V2
 * board drives D8 push-pull into a closed DIP switch's short to ground.
 *
 *   HWREV 2  OpenLead_Headless V2
 *            D8/D10 = threshold-select DIP switches (inputs, PCB pull-ups)
 *            D9     = buzzer, other leg hard-wired to ground (single-ended)
 *   HWREV 3  OpenLead_Headless V3  (default for a board with blank EEPROM)
 *            D8     = buzzer BZ1+ through R16 100R
 *            D9     = buzzer BZ1- through R17 100R  -> drivable anti-phase
 *            D10    = no connection (broken out on J6; parked as output low)
 *            A1     = NO CONNECTION.  SENSE_NEG has nothing driving it, so
 *                     NEGFIX must stay 1 on this board (it is the default).
 *            Threshold select moves from the DIP pins into THRESHSEL.
 *
 * A unit carrying a config from an older firmware is migrated with HWREV
 * pinned to 2 -- a stored config can only exist on a board already in the
 * field, and all of those are V2s.  Only a blank EEPROM defaults to 3.
 *
 * ── Threshold select ──────────────────────────────────────────────
 * Four thresholds live in EEPROM (THRESH00/01/10/11); one of them is active.
 * Which one depends on the board:
 *   HWREV 2  the DIP switches pick it.  Both pins have hardware pull-ups; a
 *            switch ON connects its pin to ground.  The pins are read directly
 *            (HIGH = 1 = switch OFF/open, LOW = 0 = ON):
 *                config "XY":  X = D8 reading, Y = D10 reading
 *              11 (both switches OFF)  -> cfg.thresh11  (factory default 0.62 V)
 *              10 (D8 high, D10 low)   -> cfg.thresh10
 *              01 (D8 low,  D10 high)  -> cfg.thresh01
 *              00 (both switches ON)   -> cfg.thresh00
 *            Re-read every detection pass, so they can be changed live.
 *   HWREV 3  THRESHSEL (0-3) picks it, indexing the same table in the same
 *            order.  Everything downstream -- SLEEPTHR00..11, the $DIP
 *            message, the host config table -- is unchanged.
 * The four values themselves are EEPROM configuration, so what each position
 * *means* can be re-programmed over serial without reflashing.
 *
 * ── EEPROM configuration ──────────────────────────────────────────
 * Nearly every tunable lives in a Config struct persisted to the RA4M1's
 * data-flash-backed EEPROM.  On boot the stored config is validated
 * (magic + version + CRC); if invalid, factory defaults are loaded and saved.
 * A host (serial terminal or Python GUI) can permanently retune a shipped
 * unit -- e.g. disable the beeper, move REFCENTER, re-map the DIP table --
 * with !SET + !SAVE, no recompile needed.  See the CONFIG FIELD TABLE below
 * for every key, and BlinkyHawk firmware manual (HTML) for full docs.
 * EEPROM here is data flash, which a sketch upload does NOT erase, so a unit
 * keeps its tuning across a reflash.  When the struct layout changes,
 * CFG_VERSION is bumped and a migration (see configMigrateV2) carries the old
 * values forward rather than reverting the unit to factory defaults.
 *
 * ── Serial protocol (115200 baud, line based) ─────────────────────
 * Commands in (each terminated with newline):
 *   !SET,<key>,<value>  set a config value in RAM (takes effect immediately)
 *   !GET,<key>          report one config value
 *   !CFG                dump every config key ($CFG rows + $CFGEND)
 *   !SAVE               persist the RAM config to EEPROM
 *   !LOAD               discard RAM changes, reload from EEPROM
 *   !DEFAULTS           factory defaults into RAM (then !SAVE to keep)
 *   !SN[,<value>]       read the unit serial number, or (with value) write it
 *                       to its own EEPROM block (survives !DEFAULTS; no commas)
 *   !DIAG[,0|1]         enter/exit diagnostic mode (bare = toggle)
 *   !STREAM[,0|1]       continuous raw streaming on/off (diag only)
 *   !RATE,<ms>          stream interval in ms
 *   !VMODE,<0|1|2>      voltage mode: 0=auto  1=lock ON  2=disable
 *   !MOSFET,<-1|0|1>    MOSFET: -1=auto(run detection) 0=hold off 1=hold on
 *   !ALERTS[,0|1]       re-enable normal alerts while charging (1=on,0=blink)
 *   !CAP[,<ms>]         capture ADC across a MOSFET toggle, then dump
 *   !SLEEP[,0]          arm the low-power timeout to fire as soon as it is
 *                       allowed (arm over USB, then unplug); ,0 = disarm
 *   !SLEEPTEST          run one sleeping-mode probe now and report its decision
 *                       ($SLEEPTEST,<state>,metric=..,thr=..,retms=..,areavms=..)
 *   !SLEEPLOG[,0]       dump the probes made while asleep (,0 = clear).  The
 *                       only way to see what the board measured off USB, where
 *                       the ground reference -- and the metric -- differ.
 *   !FLOOR,<0-3>        park the board for a current measurement (see below)
 *   !DEEP[,0|1]         report the two-stage sleep schedule; with an argument,
 *                       force the board into stage 2 now (1) or back to
 *                       stage 1 (0) while it is already asleep
 *   !PINS               dump the live pin map + this core's pin-name table
 *   !GATE[,<ANA|LED|AUX>[,<0|1|-1>]]   hold a power gate off/on, or -1 to let
 *                       it follow its configured MODE again.  Bare !GATE
 *                       reports all three.  A hold survives into sleep and
 *                       !FLOOR, which is how the sleeping current for a given
 *                       gating scheme is measured.
 *   !EXPT[,<sec>]       run the eight-step power profile (default 10 s per
 *                       step); !EXPT,0 aborts.  Prints the whole plan up
 *                       front so a PPK2 capture can be sliced by time even
 *                       though USB is unplugged for the run itself.
 *   !STATUS  / !?       print current status
 * Data out:
 *   $STATUS,...                          current mode/state summary
 *   $CFG,<key>,<value>                   one config value (from !GET/!SET/!CFG)
 *   $CFGEND                              end of a !CFG dump
 *   $SN,<value>                          unit serial number (empty if unassigned)
 *   $OK,<what> / $ERR,<what>[,detail]    command acknowledge / failure
 *   $DIP,<idx>,<threshV>                 threshold position changed (live).
 *                                        HWREV 2: a DIP switch moved.
 *                                        HWREV 3: THRESHSEL was set.
 *   $DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>    (streaming)
 *   $CAPSTART,<n>,<toggleUs>,<durMs>,<fullScale>,<vref>   (capture header)
 *   $CAP,<t_us>,<rawPos>,<rawNeg>                         (capture rows)
 *   $CAPEND
 *
 * ── Alerts ────────────────────────────────────────────────────────
 * NeoPixel: dim-blue flash = floating, green flash = closed, red flash =
 * voltage present; slow dim-red 25% blink = charging (green when battery
 * >= BATTFULLPCT); 1-4 green boot blinks = battery level.
 * Each detection state's LED is tunable: LEDFLOATBR/LEDCLOSEDBR/LEDVOLTBR set
 * brightness (0-255, 0 = that state dark), LEDFLOATMS/LEDCLOSEDMS/LEDVOLTMS
 * the flash on-time, and LEDFLOATPER/LEDCLOSEDPER/LEDVOLTPER the minimum gap
 * between flash STARTS (so the period is a rate cap; a PER below the matching
 * MS just gives a solid-on LED).  The hue of each state is fixed in firmware
 * on purpose -- the colour is the meaning.  The charging cue and the sleep
 * heartbeat have their own compile-time constants and are not affected.
 * Speaker: mirrors the LED (continuity beep / voltage double-beep), passive
 * or active buzzer selectable at runtime (PASSIVE key).  Shorting the leads
 * during boot mutes audio for the session (BOOTMUTE key to disable).
 * Beep shape: CONTPULSES/VOLTPULSES set how many pulses a sequence has,
 * CONTONMS/VOLTONMS how long each pulse sounds and CONTOFFMS/VOLTOFFMS the
 * gap between them.  CONTHOLDMS is the (longer) pulse used for the ongoing
 * re-beep while continuity persists, so a held contact sounds different from
 * first contact.  BEEPMIN still rate-caps whole sequences.
 * On HWREV 3 the piezo sits between D8 and D9, so SPKDIFF=1 (the default)
 * drives the two legs anti-phase for twice the swing across the element,
 * ~+6 dB over the single-ended V2 drive.  SPKDIFF=0 parks D8 low and
 * reproduces the V2 drive exactly, for a quieter unit or a bench A/B.
 * The pair cannot use a GPT complementary output -- D8 is P111/GTIOC3A and
 * D9 is P110/GTIOC1B, different channels -- so the anti-phase toggle runs
 * off a private FspTimer instead of the core's single-pin tone().
 *
 * ── Low-power timeout mode ────────────────────────────────────────
 * After SLEEPSEC seconds in which the leads have only ever read OPEN, the
 * board parks every load it can switch (LED rail, speaker, battery-sense
 * divider, optionally the bridge) and enters Software Standby -- CPU and all
 * peripheral clocks stopped, RAM retained -- woken by the RTC periodic
 * interrupt every SLEEPTICKMS.  Each wake runs one cheap probe (SLEEPAVG instead
 * of VOLTAVG, no agreement count, no display debounce) and goes straight back
 * to sleep unless the leads are closed or voltage is present, in which case it
 * returns to full-rate operation.  The probe compares against SLEEPTHR00..11
 * rather than THRESH00..11 when those are non-zero, because the sleeping metric
 * runs a few percent high (the analog path has not fully settled after a
 * standby wake) -- so a THRESH tuned to a target resistance can sit below the
 * sleeping closed-lead reading and silently stop continuity from waking the
 * unit.  0 means "use THRESH for that position".  The sleeping voltage test is
 * deliberately
 * LESS sensitive than the awake one: it uses only the instant-bypass band
 * (VOLTFAST x REFBAND), never the tight averaged REFBAND test, so lead noise
 * cannot wake the unit every tick.  A 6 ms dim-blue flash every
 * SLEEPHB ticks shows it is asleep rather than dead.  Sleeping is skipped
 * entirely while charging or in diagnostic mode; SLEEPSEC = 0 disables it.
 * Worst-case wake latency is SLEEPTICKMS x SLEEPTICKS.  SLEEPTICKMS snaps to
 * the RTC ladder, 2 s down to ~4 ms; sleeping averages roughly an eighth of
 * the awake current, so a short SLEEPSEC with a fast tick is a legitimate
 * operating point and not only a standby mode.
 *
 * NOTE: millis() does not advance during Standby (its timer is clocked off),
 * so the sleeping loop counts ticks rather than timing, and the idle timer is
 * re-based on wake.
 *
 * !FLOOR,<1-3> parks the board in one fixed state so a series ammeter can read
 * a stable current.  The point is the deltas, which say what is worth switching
 * in hardware on a future board revision:
 *   1 vs 3  = what Software Standby buys over a plain WFI idle (the MCU's share)
 *   1 vs 2  = the bridge leg's share (bridge resting vs disconnected)
 *   1       = the firmware-reachable floor; whatever is left is hardware --
 *             the voltage reference's bias current, the LDO and charger
 *             quiescent, and any permanently-connected divider strings.
 * Send the command over USB with the battery attached through the meter, then
 * unplug USB -- the board keeps running on battery in the parked state.
 */

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <FspTimer.h>  // anti-phase buzzer drive (see the SPEAKER section)
#include <ctype.h>
#include <string.h>
#include <stddef.h>   // offsetof (configCrc)

#if !defined(ARDUINO_ARCH_RENESAS)
  #error "BlinkyHawk_RA4M1 targets the Seeed XIAO RA4M1 -- select a Renesas RA4M1 board in Tools > Board."
#endif

// ══════════════════════════════════════════════════════════════════
//  PIN MAP / HARDWARE CONSTANTS  (fixed by the Blinky Hawk PCB)
// ══════════════════════════════════════════════════════════════════
// D8 has two different jobs depending on the board revision (cfg.hwRev), so it
// appears twice below.  Only ONE of the two names may ever be used at a time --
// see buzzerApplyPinModes(), which is the single place that decides.  Driving
// D8 as an output on a V2 board would fight a closed DIP switch's hard short to
// ground; reading it as an input on a V3 board leaves BZ1+ floating and the
// piezo silent.
// BENCH FORK: these are VARIABLES, not constants.  applyPinMap() assigns them
// from the PIN* config keys after the config load and after any !SET,PIN*, so a
// jumper can move without a recompile.  The initialisers below are the
// as-designed OpenLead_Headless V3 wiring and are what a blank EEPROM defaults
// to, so an unmodified board behaves exactly as the production firmware does.
int   SENSE_POS     = A2;                // pseudo-differential positive input
int   SENSE_NEG     = A1;                // pseudo-differential negative input
                                         // (V3: NOT CONNECTED -- keep NEGFIX=1)
int   MOSFET_PIN    = D7;                // bridge MOSFET gate (HIGH = on/resting)
int   SPEAKER_PIN   = D9;                // buzzer, "hot" leg (V3: BZ1- via R17 100R)
int   SPEAKER_PIN_B = D8;                // buzzer, anti-phase leg (V3: BZ1+ via R16 100R)
int   DIP_PIN_A     = D8;                // V2 ONLY: threshold DIP, first digit  ("X" in XY)
int   DIP_PIN_B     = D10;               // V2 ONLY: threshold DIP, second digit ("Y" in XY)
                                         // (V3: no connection; parked as an output low)
int   LED_PIN       = 6;                 // D6 -> LED1, the SK6812 data line

// "this function is not wired to anything".  Only the pins that are genuinely
// optional accept it -- the buzzer's second leg, the DIP inputs, and the three
// power gates, all of which are already guarded at every use.  A required pin
// (sense, MOSFET, LED data) set to PIN_NONE or to a number this core does not
// have falls back to its default in applyPinMap() and says so, because a
// digitalWrite() to a nonexistent pin indexes off the end of the core's pin
// table rather than failing cleanly.
#define PIN_NONE 255
// PIN_RGB_EN is pin 21 / P500, the power gate for the XIAO module's OWN
// onboard RGB LED -- it is NOT connected to LED1 on either board revision.
// It is still worth switching: the onboard LED is a permanent load on the
// module's 3.3 V rail, and killing it is part of what the sleep-current and
// !FLOOR numbers assume.  (LED1 itself cannot be gated on V3 at all -- it is
// wired straight to BatteryRail, so its quiescent draw is now a hardware
// floor rather than something firmware can park.)
#define     RGB_POWER_PIN   PIN_RGB_EN
int         CHARGE_PIN    = A3;          // VBUS/2 divider (USB-power sense)
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

// ══════════════════════════════════════════════════════════════════
//  EEPROM CONFIGURATION
// ══════════════════════════════════════════════════════════════════
// Every runtime-tunable setting lives in this struct.  It is held in RAM
// (edited by !SET, applied immediately) and persisted to EEPROM by !SAVE.
// Layout changes REQUIRE bumping CFG_VERSION so stale stored data is
// rejected and replaced with defaults instead of being misread.
// BENCH FORK: a DIFFERENT magic at a DIFFERENT address from the production
// firmware's ("BHK1" at 0).  EEPROM here is data flash and a sketch upload does
// not erase it, so the two firmwares would otherwise fight over the same bytes:
// flashing the bench build onto a real unit would overwrite its field tuning
// with bench values, and flashing production back would find a foreign image
// and reseed defaults.  With separate blocks a board can be moved between the
// two builds freely and each keeps its own settings.  Config is ~200 bytes and
// the SN block lives at 512, so 1024 is clear of both.
#define CFG_MAGIC   0x42484B58UL   // "BHKX" -- bench, deliberately not "BHK1"
#define CFG_VERSION 4              // bench layout v4 (adds chargeInhibit; v3
                                   //  added deepParkOff, v2 the deep-sleep
                                   //  stage, v1 the pin map + power gates).
                                   //  No migration by design -- an older bench
                                   //  block is simply replaced with defaults.
#define CFG_EEPROM_ADDR 1024

struct Config {
  uint32_t magic;
  uint16_t version;

  // -- Board revision ----------------------------------------------
  // 2 = OpenLead_Headless V2: threshold DIP switches on D8/D10, buzzer driven
  //     single-ended from D9 with its other leg hard-wired to ground.
  // 3 = OpenLead_Headless V3: no DIP switches (threshSel picks the threshold),
  //     buzzer wired between D8 and D9 so it can be driven anti-phase.
  // Set from EEPROM before any pin is configured, because it decides whether
  // D8 is an input or an output.  A unit migrated from an older stored config
  // is forced to 2 -- only a board that has never been configured (or one
  // explicitly told otherwise) is assumed to be V3.
  uint8_t  hwRev;

  // -- Detection ---------------------------------------------------
  float    refCenterV;     // resting differential centre (~0 V)
  float    refBandV;       // |diff - centre| within this -> no voltage, run test
  float    thresh[4];      // open/closed threshold per position [00,01,10,11]
  uint8_t  threshSel;      // hwRev 3 ONLY: which thresh[] entry is active (0-3).
                           // Replaces the V2 DIP switches; ignored on hwRev 2,
                           // where the pins still decide.
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
  // In every method a LARGER metric = more OPEN, so the DIP threshold table
  // and the "metric > threshold -> FLOAT" test are unchanged -- but the units
  // of the threshold change with the method, so re-tune THRESH00..11 on switch.
  uint8_t  detectMethod;   // 0=single(legacy) 1=time-to-return 2=tail-area
  float    detReturnBand;  // method 1: |diff-refCentre| within this = "returned"
  uint16_t detWindowUs;    // methods 1&2: max sample window / timeout (us from toggle)
  uint16_t detAreaStartUs; // method 2: tail-area integration start (us from toggle)

  // -- Alerts ------------------------------------------------------
  uint8_t  ledEnable;      // 1 = normal detection LED alerts
  uint8_t  beepEnable;     // 1 = speaker alerts (master enable)
  uint8_t  bootMute;       // 1 = leads CLOSED at boot mutes audio for the session
  uint8_t  passiveBuzzer;  // 1 = passive buzzer via tone(), 0 = active (DC on/off)
  // hwRev 3 ONLY: drive the piezo anti-phase (D8 inverted against D9) instead of
  // parking D8 low.  Doubles the voltage across the element, ~+6 dB.  Kept
  // switchable so a unit can be quietened, and so the two drives can be A/B'd on
  // the bench.  Forced off on hwRev 2, where D8 is a DIP input.
  uint8_t  spkDiff;
  uint16_t contFreqHz;     // continuity pitch (passive buzzer only)
  uint16_t voltFreqHz;     // voltage pitch    (passive buzzer only)
  uint8_t  contPulses;     // pulses per continuity beep
  uint8_t  voltPulses;     // pulses per voltage beep
  uint8_t  contRepeat;     // 1 = re-beep while CLOSED holds
  uint8_t  voltRepeat;     // 1 = re-beep while VOLTAGE holds
  uint16_t contRepeatMs;   // repeat period while CLOSED
  uint16_t voltRepeatMs;   // repeat period while VOLTAGE
  uint16_t beepMinMs;      // min gap between beep sequences (rate cap)

  // -- Beep pulse shape --------------------------------------------
  // Length of the individual pulses inside one beep sequence.  contPulses /
  // voltPulses say how many, these say how long.  The "hold" pulse is the
  // longer one used for the ongoing re-beep while continuity persists, so a
  // held contact sounds different from first contact.
  uint16_t contOnMs;       // first-contact continuity pulse on-time
  uint16_t contHoldMs;     // ongoing "still there" continuity pulse on-time
  uint16_t contOffMs;      // gap between continuity pulses (both cases)
  uint16_t voltOnMs;       // voltage pulse on-time
  uint16_t voltOffMs;      // gap between voltage pulses

  // -- Alert LED (per detection state) ------------------------------
  // Each state owns one colour channel -- FLOAT is blue, CLOSED green,
  // VOLTAGE red -- so "brightness" is just that channel's value, 0-255.
  // The hue is deliberately fixed: these are safety signals and the colour
  // is the meaning.  (Full per-state RGB would be three more keys each if
  // that ever changes.)
  uint8_t  ledFloatBright;
  uint8_t  ledClosedBright;
  uint8_t  ledVoltBright;
  // Flash on-time, and the minimum gap between the START of one flash and
  // the next -- so the period is the rate cap, not on-time + gap.  Making
  // perMs shorter than msOn just gives a solid-on LED.
  uint16_t ledFloatMs;
  uint16_t ledClosedMs;
  uint16_t ledVoltMs;
  uint16_t ledFloatPerMs;
  uint16_t ledClosedPerMs;
  uint16_t ledVoltPerMs;

  // -- Power / battery ---------------------------------------------
  float    chargeThreshV;  // A3 volts (VBUS/2) above this = charging
  float    battEmptyV;     // battery voltage mapped to 0%
  float    battFullV;      // battery voltage mapped to 100%
  uint8_t  battFullPct;    // >= this % while charging = green charge blink

  // -- Low-power timeout -------------------------------------------
  // After idleTimeoutS seconds in which the leads have only read OPEN, the
  // board parks every switchable load and drops into Software Standby, waking
  // on a 2 s RTC tick to probe.  See the LOW-POWER TIMEOUT MODE section.
  uint16_t idleTimeoutS;   // seconds of open-lead inactivity before sleeping (0 = never)
  // Base wake period.  The RTC periodic interrupt is the wake source and it only
  // supports a fixed set of rates (2000/1000/500/250/125 ms here), so a value
  // set over serial is snapped to the nearest supported one and written back --
  // !CFG therefore always reports the rate actually programmed, not the request.
  uint16_t sleepTickMs;
  uint8_t  sleepPollTicks; // probe every N wake ticks (N * sleepTickMs between probes)
  uint8_t  sleepVoltAvg;   // reads taken for the voltage check while asleep (any
                           // one over the bypass band wakes -> fewer = quieter)
  uint8_t  sleepHbTicks;   // heartbeat flash every N ticks (0 = no heartbeat)
  uint8_t  sleepParkOff;   // 1 = park the bridge MOSFET OFF while asleep

  // Wake threshold per DIP position, used ONLY by the sleeping probe.  The
  // sleeping metric runs a few percent higher than the awake one (the analog
  // path has not fully settled after a standby wake), so a thresh[] value tuned
  // to a particular resistance can sit below the sleeping closed-lead reading
  // and silently stop continuity from waking the unit.  These decouple the two:
  // thresh[] answers "is this resistance low enough to call continuity", while
  // sleepThresh[] answers the coarser "is anything connected at all -- wake up".
  // 0 = fall back to thresh[] for that position (the default, so behaviour is
  // unchanged until a value is deliberately set).
  float    sleepThresh[4];

  // -- Misc --------------------------------------------------------
  uint16_t loopDelayMs;    // main-loop pacing (WFI idle between passes)

  // == BENCH FORK ONLY =============================================
  // -- Runtime pin map ---------------------------------------------
  // Arduino pin NUMBERS (what D7 / A2 evaluate to on this core), not names.
  // 255 = PIN_NONE = "not wired", which every consumer checks before driving
  // it.  !PINS prints the name-to-number table so a value can be looked up
  // without guessing.  A change takes effect immediately (applyPinMap()),
  // which includes returning the OLD pin to a floating input so a moved jumper
  // does not leave a driven output behind on the breadboard.
  uint8_t  pinSensePos;
  uint8_t  pinSenseNeg;
  uint8_t  pinCharge;
  uint8_t  pinMosfet;
  uint8_t  pinSpkA;        // buzzer hot leg (D9 on the PCB)
  uint8_t  pinSpkB;        // buzzer anti-phase leg (D8 on V3)
  uint8_t  pinLedData;     // SK6812 data
  uint8_t  pinDipA;        // hwRev 2 only
  uint8_t  pinDipB;        // hwRev 2 only

  // -- Power gates -------------------------------------------------
  // Three identical gates driving external load switches.  Gate 0 (ANA) is
  // meant for the analog front end -- the TL431 bias and the divider strings.
  // Gate 1 (LEDG) is meant for LED1's supply, which on V3 is hard-wired to
  // BatteryRail and so cannot be gated on the real board at all.  Gate 2 (AUX)
  // is spare, for whatever the next experiment is.
  //   pin   255 = not wired, and the gate is then inert
  //   pol   1 = HIGH enables the load, 0 = LOW enables it.  A P-FET high-side
  //         switch is usually active low; get this wrong and the experiment
  //         runs with the rail permanently on, which reads as "gating saves
  //         nothing" rather than as a wiring error.
  //   mode  0 ALWAYS on / 1 on while awake, off while parked / 2 pulsed:
  //         off at rest, raised only around a measurement
  //   settle  microseconds (ANA/AUX) or milliseconds (LEDG) to wait after
  //         raising the gate before trusting what is downstream of it
  uint8_t  gAnaPin,  gAnaPol,  gAnaMode;   uint16_t gAnaSettleUs;
  uint8_t  gLedPin,  gLedPol,  gLedMode;   uint16_t gLedSettleMs;
  uint8_t  gAuxPin,  gAuxPol,  gAuxMode;   uint16_t gAuxSettleUs;

  // -- Deep sleep (stage 2) ----------------------------------------
  // The production firmware has ONE sleeping state and it is the normal
  // operating mode: sleep after idleTimeoutS, wake every sleepTickMs, probe
  // every sleepPollTicks.  These two keys put a second, slower stage under it.
  //
  //   deepSec  seconds of stage-1 sleep, with every probe reading open, before
  //            dropping to stage 2.  Counted in TICKS, not by millis(), which
  //            is frozen in Software Standby.  0 = never (production behaviour).
  //   deepHz   stage-2 probe rate.  This re-programs the RTC wake period as
  //            well as the probe divisor, so the board really does wake less
  //            often -- skipping probes on a fast tick would save almost
  //            nothing, since the standby wake itself is most of the cost at
  //            these rates.  Snapped to what the RTC ladder can produce and
  //            written back, exactly as sleepTickMs is, so !CFG always reports
  //            the rate in force rather than the one requested.
  //
  // The cost is latency: at 1 Hz a closed lead can wait a second to be noticed,
  // against ~63 ms in stage 1.  That is the trade deepSec exists to place --
  // fast while the unit might still be in use, slow once it clearly is not.
  uint16_t deepSec;
  float    deepHz;

  //   deepParkOff  what the bridge MOSFET does in stage 2, independently of
  //            sleepParkOff.  The bridge resting (ON) leaves the 100K sense
  //            resistor connected to the 2.5 V reference, so it draws
  //            continuously -- it is the "bridge leg's share" that !FLOOR
  //            level 1 vs 2 was built to measure, and the obvious next thing
  //            to switch off once the wake rate is already down.
  //              0 = bridge resting (ON), as in normal operation
  //              1 = bridge parked OFF
  //              2 = follow sleepParkOff, i.e. stage 2 behaves like stage 1
  //            2 is the default so adding this key changed nothing, and so a
  //            unit configured with SLEEPPARK=1 cannot end up with a DEEP stage
  //            that is LESS parked than its light stage just because one of the
  //            two keys was forgotten.
  //
  // CAUTION when setting this to 1: the probe re-establishes the resting state
  // and waits settlePostMs before testing, but in stage 2 the node will have
  // been floating for a whole deep period (a second at the default) rather than
  // one 63 ms tick.  If closed leads stop waking the board after enabling this,
  // raise SETTLEPOSTMS before suspecting anything else -- and check the effect
  // on the metric with !SLEEPTEST / !SLEEPLOG, not by eye.
  uint8_t  deepParkOff;

  // -- Charge-detect inhibit ---------------------------------------
  // A3 reads VBUS/2, and on a real unit VBUS present means "on the charger",
  // which the firmware treats as a reason to stop doing things: no sleeping
  // (lowPowerAllowed), no normal alerts (the charging blink takes the LED and
  // the speaker is silenced), and an immediate wake if it appears while
  // asleep.  Those are the right defaults for a product whose charger is a USB
  // cable that can earth-ground the meter and inject supply noise.
  //
  // On the bench they are wrong whenever the 5 V input is a bench supply, a
  // boost converter, or primary cells behind a boost -- VBUS is then simply
  // "how this board is powered", permanently true, and the unit would never
  // sleep, never alert, and never be measurable.
  //
  //   1 = normal, the production behaviour (default)
  //   0 = detection still runs and is still REPORTED ($STATUS charge=, and the
  //       GUI's charge readout), but it inhibits nothing: the board sleeps,
  //       alerts, and stages exactly as it would on a battery.
  //
  // Deliberately not "stop detecting": keeping the telemetry means you can see
  // the 5 V rail is present while the board ignores it, which is the difference
  // between a working boost supply and a disconnected one.
  uint8_t  chargeInhibit;

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

  // Defaults describe a NEW board, which is a V3.  Units carrying an older
  // stored config are V2s and are pinned to hwRev 2 by the migrations below --
  // this default only ever reaches a board whose EEPROM was blank or reset.
  cfg.hwRev          = 3;

  cfg.refCenterV     = 0.018f;
  cfg.refBandV       = 0.025f;
  // NOTE: thresh[] is in the units of cfg.detectMethod, which is now 1
  // (time-to-return), so slot 11 is 1.0 MILLISECONDS, not volts.  Slots 00-10
  // are the old method-0 volt figures and have NOT been re-tuned for method 1 --
  // they are placeholders until someone characterises them.  Only slot 11 (the
  // factory selection) is a real, bench-tuned value.
  cfg.thresh[0]      = 0.15f;    // 00: not re-tuned for method 1
  cfg.thresh[1]      = 0.54f;    // 01: not re-tuned for method 1
  cfg.thresh[2]      = 0.45f;    // 10: not re-tuned for method 1
  cfg.thresh[3]      = 1.0f;     // 11: ms, the factory selection
  cfg.threshSel      = 3;        // V3: same entry the V2 factory DIP position used
  cfg.voltFastMult   = 5.0f;
  cfg.voltAvgSamples = 10;
  cfg.testAgree      = 1;
  cfg.stableCount    = 2;
  cfg.settlePreUs    = 300;
  cfg.settlePostMs   = 3;
  cfg.negFix         = 1;        // matches the proven XIAO_Minimal behaviour
  cfg.negFixV        = 1.250f;

  cfg.detectMethod   = 1;        // time-to-return: what the V3 units ship on
  cfg.detReturnBand  = 0.05f;    // best IsoGnd/Open separation in bench captures
  cfg.detWindowUs    = 1500;     // recovery completes ~0.7-0.95 ms; window past it
  cfg.detAreaStartUs = 400;      // skip the common initial dip; integrate the tail

  cfg.ledEnable      = 1;
  cfg.beepEnable     = 1;
  cfg.bootMute       = 1;
  cfg.passiveBuzzer  = 1;
  cfg.spkDiff        = 1;        // V3 default: anti-phase drive (loudest)
  // The fitted buzzer is a resonator: it is only usefully loud near 4 kHz, so
  // both alerts sit there and are told apart by pulse count, not pitch.
  cfg.contFreqHz     = 4000;
  cfg.voltFreqHz     = 4000;
  cfg.contPulses     = 1;
  cfg.voltPulses     = 2;        // double beep (still being tuned by ear)
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

  cfg.ledFloatBright  = 20;      // dim blue -- the common idle state
  cfg.ledClosedBright = 64;      // green
  cfg.ledVoltBright   = 200;     // red
  cfg.ledFloatMs      = 50;
  cfg.ledClosedMs     = 200;
  cfg.ledVoltMs       = 200;
  cfg.ledFloatPerMs   = 1000;    // 1 Hz cap
  cfg.ledClosedPerMs  = 500;     // 2 Hz cap
  cfg.ledVoltPerMs    = 500;     // 2 Hz cap

  cfg.chargeThreshV  = 2.0f;
  // 3.70, not the cell's electrical floor: below ~3.6 V the analog baseline has
  // drifted far enough that merely moving the leads trips the voltage detector
  // (measured Aug 2026 -- false alerts at 3.6 V in, clean at 3.7 V).  The gauge
  // therefore has to read empty while the unit is still trustworthy, so "empty"
  // means "stop believing it" rather than "the cell is flat".
  cfg.battEmptyV     = 3.70f;
  cfg.battFullV      = 4.20f;
  cfg.battFullPct    = 90;

  // Sleeping costs ~2 mA against ~11 mA awake, so the unit drops into the
  // low-power poll almost immediately and lives there: 1 s of open leads, then
  // wake ~16x/sec to probe.  This is the operating state, not a standby mode.
  cfg.idleTimeoutS   = 1;        // seconds of open leads before sleeping
  cfg.sleepTickMs    = 63;       // 1/16 s rung; ~2 mA average
  cfg.sleepPollTicks = 1;        // probe on every wake
  cfg.sleepVoltAvg   = 3;        // reads per probe; any one over the bypass band wakes
  cfg.sleepHbTicks   = 32;       // counts TICKS, so ~2 s at a 63 ms tick
  cfg.sleepParkOff   = 0;        // bridge parked resting (as in normal operation)
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = 0.0f;   // 0 = use thresh[i]
  // Slot 11 is the factory selection and the one that matters: a probe taken out
  // of standby reads high, so its wake threshold sits above the awake 1.0 ms.
  cfg.sleepThresh[3] = 1.2f;     // ms (units follow detectMethod)

  cfg.loopDelayMs    = 50;

  // == BENCH FORK ===================================================
  // The pin map defaults to the as-designed V3 wiring, so a bench build on an
  // unmodified board is the production firmware plus inert extras.
  cfg.pinSensePos    = (uint8_t)A2;
  cfg.pinSenseNeg    = (uint8_t)A1;
  cfg.pinCharge      = (uint8_t)A3;
  cfg.pinMosfet      = (uint8_t)D7;
  cfg.pinSpkA        = (uint8_t)D9;
  cfg.pinSpkB        = (uint8_t)D8;
  cfg.pinLedData     = 6;            // D6
  cfg.pinDipA        = (uint8_t)D8;
  cfg.pinDipB        = (uint8_t)D10;

  // Gates default to NOT WIRED, so nothing about the board's behaviour changes
  // until a load switch is actually jumpered in and its pin named.  Polarity
  // defaults to active-low because that is what a P-FET high-side switch -- the
  // obvious part to reach for here -- wants.
  cfg.gAnaPin  = PIN_NONE;  cfg.gAnaPol = 0;  cfg.gAnaMode = 0;
  cfg.gAnaSettleUs = 2000;           // TL431 + a 1 M divider is not instant
  cfg.gLedPin  = PIN_NONE;  cfg.gLedPol = 0;  cfg.gLedMode = 0;
  cfg.gLedSettleMs = 1;              // the SK6812 needs its rail up before data
  cfg.gAuxPin  = PIN_NONE;  cfg.gAuxPol = 0;  cfg.gAuxMode = 0;
  cfg.gAuxSettleUs = 1000;

  cfg.deepSec = 5;                   // 5 s of open leads in stage 1 -> stage 2
  cfg.deepHz  = 1.0f;                // stage 2 probes once a second
  cfg.deepParkOff = 2;               // 2 = follow SLEEPPARK (no change by default)
  cfg.chargeInhibit = 1;             // production behaviour until told otherwise
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

// ── Upgrading a stored bench config ───────────────────────────────
// This fork originally shipped with NO migrations, on the reasoning that a
// bench config is only a few minutes of !SET.  That was wrong.  Three
// CFG_VERSION bumps in three days wiped a unit that was mid-experiment, and a
// bench unit that has been characterised against a current trace is exactly as
// much of a record as a field unit -- losing it costs a re-characterisation,
// not a re-typing.  Hence this.
//
// It does NOT use the production fork's frozen-struct-per-version approach.
// Instead it relies on an invariant of how this struct has actually changed:
//
//   ** EVERY bench version so far is the previous one with fields APPENDED
//      IMMEDIATELY BEFORE crc. **
//
// That makes every older image a byte-identical PREFIX of the current struct,
// so a single function covers all of them and every future append as well:
// CRC-check the stored image over its own prefix, copy that prefix over a
// defaults-initialised cfg, and every field that version did not have simply
// keeps its new default.
//
// THE INVARIANT IS LOAD-BEARING.  If a field is ever reordered, resized, or
// removed rather than appended, this would silently reinterpret bytes.  In that
// case do not add a row below -- change CFG_MAGIC instead, which rejects every
// old image cleanly and reseeds.
//
// Adding a version: append the field(s) before crc, bump CFG_VERSION, and add a
// row naming THE FIRST FIELD THE OLD VERSION DID NOT HAVE.  Offsets are taken
// from the live struct, so there are no magic numbers to drift out of date.
//
// Failure is safe in the only direction that matters: if a row is wrong, the
// CRC will not match, the migration declines, and the board falls back to
// defaults -- i.e. to exactly the behaviour it had before this existed.
struct BenchLayout {
  uint16_t version;
  uint16_t prefixLen;    // bytes of Config this version also had, byte-identical
};
const BenchLayout BENCH_LAYOUTS[] = {
  { 1, (uint16_t)offsetof(Config, deepSec)       },  // v1: pin map + power gates
  { 2, (uint16_t)offsetof(Config, deepParkOff)   },  // v2: + the deep-sleep stage
  { 3, (uint16_t)offsetof(Config, chargeInhibit) },  // v3: + DEEPPARK
};
const int BENCH_LAYOUT_COUNT = sizeof(BENCH_LAYOUTS) / sizeof(BENCH_LAYOUTS[0]);

// The table is only meaningful if the fields really were appended in this
// order.  These catch a reordering at compile time rather than at the bench.
static_assert(offsetof(Config, deepSec) < offsetof(Config, deepParkOff),
              "bench layout table: deepSec must precede deepParkOff");
static_assert(offsetof(Config, deepParkOff) < offsetof(Config, chargeInhibit),
              "bench layout table: deepParkOff must precede chargeInhibit");
static_assert(offsetof(Config, chargeInhibit) < offsetof(Config, crc),
              "bench layout table: every listed field must precede crc");

// Where that version's crc sat in its own image: immediately after its last
// field, rounded up to the 2-byte alignment a uint16_t gets.  (v1 and v2 both
// ended on an even offset so this is a no-op for them; v3 ended on the odd byte
// after a uint8_t, and its crc was therefore one padding byte further on.)
static uint16_t benchCrcOffset(uint16_t prefixLen) {
  return (uint16_t)((prefixLen + 1) & ~1);
}

// Try to read a stored image written by an older bench firmware.  Returns true
// and leaves the upgraded config in cfg; returns false if there is nothing
// usable there, in which case the caller keeps defaults.
bool configMigrateBench() {
  uint8_t raw[sizeof(Config)];
  for (uint16_t i = 0; i < (uint16_t)sizeof(raw); i++)
    raw[i] = EEPROM.read(CFG_EEPROM_ADDR + i);

  // magic and version are the first two fields and have never moved, so they
  // can be read before we know which layout this is.
  uint32_t magic;
  uint16_t version;
  memcpy(&magic,   raw + offsetof(Config, magic),   sizeof(magic));
  memcpy(&version, raw + offsetof(Config, version), sizeof(version));
  if (magic != CFG_MAGIC) return false;        // not a bench image at all

  for (int i = 0; i < BENCH_LAYOUT_COUNT; i++) {
    if (BENCH_LAYOUTS[i].version != version) continue;
    uint16_t prefixLen = BENCH_LAYOUTS[i].prefixLen;
    uint16_t crcOff    = benchCrcOffset(prefixLen);

    uint16_t storedCrc;
    memcpy(&storedCrc, raw + crcOff, sizeof(storedCrc));
    if (crc16_ccitt(raw, crcOff) != storedCrc) return false;

    configDefaults();                  // fields this version lacked get defaults
    memcpy(&cfg, raw, prefixLen);      // everything it had is carried across
    cfg.magic   = CFG_MAGIC;           // memcpy brought the old ones over
    cfg.version = CFG_VERSION;
    cfgDirty    = false;
    return true;
  }
  return false;                        // a version we have no row for
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
  { "HWREV",       FT_U8,    &cfg.hwRev,           2,      3      },
  // Detection
  { "REFCENTER",   FT_FLOAT, &cfg.refCenterV,     -1.0f,   1.0f   },
  { "REFBAND",     FT_FLOAT, &cfg.refBandV,        0.001f, 1.0f   },
  { "THRESH00",    FT_FLOAT, &cfg.thresh[0],       0.001f, 3.3f   },
  { "THRESH01",    FT_FLOAT, &cfg.thresh[1],       0.001f, 3.3f   },
  { "THRESH10",    FT_FLOAT, &cfg.thresh[2],       0.001f, 3.3f   },
  { "THRESH11",    FT_FLOAT, &cfg.thresh[3],       0.001f, 3.3f   },
  // Which of the four above is active.  hwRev 3 only -- on hwRev 2 the DIP
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

  // == BENCH FORK ===================================================
  // Pin map.  Values are Arduino pin numbers; 255 = not wired.  !PINS prints
  // the name table.  Max is PIN_NONE itself so "unwire this" is expressible.
  { "PINSENSEP",   FT_U8,    &cfg.pinSensePos,     0,      255    },
  { "PINSENSEN",   FT_U8,    &cfg.pinSenseNeg,     0,      255    },
  { "PINCHARGE",   FT_U8,    &cfg.pinCharge,       0,      255    },
  { "PINMOSFET",   FT_U8,    &cfg.pinMosfet,       0,      255    },
  { "PINSPKA",     FT_U8,    &cfg.pinSpkA,         0,      255    },
  { "PINSPKB",     FT_U8,    &cfg.pinSpkB,         0,      255    },
  { "PINLEDDAT",   FT_U8,    &cfg.pinLedData,      0,      255    },
  { "PINDIPA",     FT_U8,    &cfg.pinDipA,         0,      255    },
  { "PINDIPB",     FT_U8,    &cfg.pinDipB,         0,      255    },
  // Power gates (see the Config struct for what mode/pol mean)
  { "PINANA",      FT_U8,    &cfg.gAnaPin,         0,      255    },
  { "ANAPOL",      FT_BOOL,  &cfg.gAnaPol,         0,      1      },
  { "ANAMODE",     FT_U8,    &cfg.gAnaMode,        0,      2      },
  { "ANAUS",       FT_U16,   &cfg.gAnaSettleUs,    0,      60000  },
  { "PINLEDG",     FT_U8,    &cfg.gLedPin,         0,      255    },
  { "LEDGPOL",     FT_BOOL,  &cfg.gLedPol,         0,      1      },
  { "LEDGMODE",    FT_U8,    &cfg.gLedMode,        0,      2      },
  { "LEDGMS",      FT_U16,   &cfg.gLedSettleMs,    0,      1000   },
  { "PINAUX",      FT_U8,    &cfg.gAuxPin,         0,      255    },
  { "AUXPOL",      FT_BOOL,  &cfg.gAuxPol,         0,      1      },
  { "AUXMODE",     FT_U8,    &cfg.gAuxMode,        0,      2      },
  { "AUXUS",       FT_U16,   &cfg.gAuxSettleUs,    0,      60000  },
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
//  BENCH FORK: RUNTIME PIN MAP
// ══════════════════════════════════════════════════════════════════
// Defined later in the file; needed here.
void sleepMs(unsigned long ms);
void buzzerApplyPinModes();

// Every pin a jumper can reasonably land on, with the name printed on the
// XIAO's silkscreen.  This doubles as the validity test: a config value that
// is not in this table is not a pin, so applyPinMap() refuses it.  NOTE the
// XIAO pinout aliases its analog and digital names (A0 and D0 are the same
// physical pad), so numbers repeat here -- !PINS prints the whole table so
// that aliasing is visible rather than surprising.
struct PinName { const char *name; uint8_t num; };
const PinName PIN_NAMES[] = {
  { "D0",  (uint8_t)D0 },  { "D1",  (uint8_t)D1 },  { "D2",  (uint8_t)D2 },
  { "D3",  (uint8_t)D3 },  { "D4",  (uint8_t)D4 },  { "D5",  (uint8_t)D5 },
  { "D6",  (uint8_t)D6 },  { "D7",  (uint8_t)D7 },  { "D8",  (uint8_t)D8 },
  { "D9",  (uint8_t)D9 },  { "D10", (uint8_t)D10 },
  { "A0",  (uint8_t)A0 },  { "A1",  (uint8_t)A1 },  { "A2",  (uint8_t)A2 },
  { "A3",  (uint8_t)A3 },
  // A4/A5 are NOT defined by this variant -- the XIAO RA4M1 breaks out four
  // analog inputs, not six.  Do not "restore" them; the build fails.
};
const int PIN_NAME_COUNT = sizeof(PIN_NAMES) / sizeof(PIN_NAMES[0]);

bool pinIsValid(uint8_t p) {
  for (int i = 0; i < PIN_NAME_COUNT; i++) if (PIN_NAMES[i].num == p) return true;
  return false;
}

// First silkscreen name for a pin number, or "?" -- display only.
const char *pinNameOf(uint8_t p) {
  for (int i = 0; i < PIN_NAME_COUNT; i++) if (PIN_NAMES[i].num == p) return PIN_NAMES[i].name;
  return "?";
}

// ══════════════════════════════════════════════════════════════════
//  BENCH FORK: POWER GATES
// ══════════════════════════════════════════════════════════════════
// Three identical outputs driving external load switches.  See the Config
// struct for what pin/pol/mode/settle mean.  Everything here is a no-op when
// a gate's pin is PIN_NONE, which is the default, so an unmodified board is
// unaffected by the existence of this section.
enum { GATE_ANA = 0, GATE_LED = 1, GATE_AUX = 2, GATE_COUNT = 3 };
const char *const GATE_NAME[GATE_COUNT] = { "ANA", "LED", "AUX" };
uint8_t *const GATE_PIN[GATE_COUNT]  = { &cfg.gAnaPin,  &cfg.gLedPin,  &cfg.gAuxPin  };
uint8_t *const GATE_POL[GATE_COUNT]  = { &cfg.gAnaPol,  &cfg.gLedPol,  &cfg.gAuxPol  };
uint8_t *const GATE_MODE[GATE_COUNT] = { &cfg.gAnaMode, &cfg.gLedMode, &cfg.gAuxMode };

// -1 = follow the configured MODE; 0/1 = held by !GATE.  A hold is deliberately
// NOT config: it is the knob you turn while watching an ammeter, and it must
// not survive a power cycle into a unit someone later thinks is stock.
int8_t gateForce[GATE_COUNT] = { -1, -1, -1 };
bool   gateOn[GATE_COUNT]    = { false, false, false };
bool   gatesParked           = false;   // context of the last gatesRest()

// ANA/AUX carry a microsecond settle, LEDG a millisecond one (an SK6812 rail
// coming up is a much slower thing than a reference settling).  One accessor
// so the callers do not care which.
uint32_t gateSettleUs(uint8_t i) {
  if (i == GATE_LED) return (uint32_t)cfg.gLedSettleMs * 1000UL;
  return (i == GATE_ANA) ? cfg.gAnaSettleUs : cfg.gAuxSettleUs;
}

// delayMicroseconds() is a busy loop, which is the wrong thing to burn a
// multi-millisecond settle on when the whole exercise is measuring current.
// Anything over a millisecond goes through sleepMs() (WFI) instead.
void gateDelayUs(uint32_t us) {
  if (us == 0) return;
  if (us >= 1000UL) { sleepMs(us / 1000UL); us %= 1000UL; }
  if (us) delayMicroseconds((unsigned int)us);
}

void gateWrite(uint8_t i, bool on) {
  gateOn[i] = on;
  uint8_t p = *GATE_PIN[i];
  if (!pinIsValid(p)) return;                 // not wired: state is bookkeeping only
  digitalWrite(p, (on == (*GATE_POL[i] != 0)) ? HIGH : LOW);
}

// Put every gate into its resting state for the current run context.
// `parked` = the board is asleep or in !FLOOR, which is the only thing MODE 1
// distinguishes.  A !GATE hold overrides the mode entirely -- that is the
// point of the hold.
void gatesRest(bool parked) {
  gatesParked = parked;
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    if (gateForce[i] >= 0) { gateWrite(i, gateForce[i] != 0); continue; }
    uint8_t m = *GATE_MODE[i];
    gateWrite(i, (m == 0) || (m == 1 && !parked));
  }
}

// Raise whatever the analog path needs for one measurement and wait the
// longest settle involved.  Returns a mask for gatesMeasureEnd() so only the
// gates this call actually raised are dropped again.
//
// The LED gate is deliberately NOT part of this: it follows the pixel (see
// setPixel), not the ADC.  Note that in mode 2 this is where the measurement
// perturbation lives -- the production firmware's sleeping probe powers its
// rails up and waits precisely because rail loading moves the resting
// differential by more than refBandV, and a pulsed reference is the same
// problem with a slower time constant.  If a mode-2 metric disagrees with a
// mode-0 one, raise ANAUS before suspecting the detection method.
// Raise the gates WITHOUT waiting, reporting the longest settle through
// `settleUsOut`.  Split out from gatesMeasureBegin() so runCapture() can put
// the rail's rise inside the captured window instead of before it -- watching
// that edge is how ANAUS gets tuned, and it is invisible if the settle has
// already been paid before sampling starts.
// What gatesMeasureRaise() WOULD raise, without touching anything.  runCapture()
// needs to lay out its timeline before it starts sampling.
uint8_t gatesPendingMask(uint32_t *settleUsOut) {
  uint8_t pending = 0;
  uint32_t settle = 0;
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    if (i == GATE_LED)        continue;
    if (gateForce[i] >= 0)    continue;
    if (gateOn[i])            continue;
    if (!pinIsValid(*GATE_PIN[i])) continue;
    pending |= (uint8_t)(1 << i);
    if (gateSettleUs(i) > settle) settle = gateSettleUs(i);
  }
  if (settleUsOut) *settleUsOut = settle;
  return pending;
}

uint8_t gatesMeasureRaise(uint32_t *settleUsOut) {
  uint8_t raised = 0;
  uint32_t settle = 0;
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    if (i == GATE_LED)        continue;
    if (gateForce[i] >= 0)    continue;       // held by !GATE
    if (gateOn[i])            continue;       // already up
    if (!pinIsValid(*GATE_PIN[i])) continue;  // not wired
    gateWrite(i, true);
    raised |= (uint8_t)(1 << i);
    if (gateSettleUs(i) > settle) settle = gateSettleUs(i);
  }
  if (settleUsOut) *settleUsOut = settle;
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
// Eight steps of equal length, each a different gating scheme, so one PPK2
// capture contains every case with the boundaries at known times.  The point
// is the DELTAS between steps, exactly as with !FLOOR -- the absolute number
// includes the breadboard, the jumpers and whatever the meter is doing.
//
// ana/led/aux: 0 = force the gate off, 1 = force it on, -1 = leave it to its
// configured MODE.  The four `run` steps sweep the two interesting gates while
// the board runs normally; the four `park` steps repeat the sweep asleep, with
// both gates forced explicitly so the parked state does not depend on MODE.
//
// `deep` selects the sleep stage for a parked step.  The last two steps repeat
// the first and last parked cases in stage 2, so one capture answers both "what
// does gating buy" and "what does the deeper stage buy" -- and, read together,
// whether the two are additive or whether the slower probe rate has already
// taken most of what gating the analog rail would have.
struct ExptStep {
  const char *name; uint8_t parked; uint8_t deep;
  int8_t ana; int8_t led; int8_t aux;
};
const ExptStep EXPT_STEPS[] = {
  { "run-baseline",  0, 0, -1, -1, -1 },
  { "run-led-off",   0, 0, -1,  0, -1 },
  { "run-ana-off",   0, 0,  0, -1, -1 },
  { "run-both-off",  0, 0,  0,  0, -1 },
  { "park-baseline", 1, 0,  1,  1, -1 },
  { "park-led-off",  1, 0,  1,  0, -1 },
  { "park-ana-off",  1, 0,  0,  1, -1 },
  { "park-both-off", 1, 0,  0,  0, -1 },
  { "deep-baseline", 1, 1,  1,  1, -1 },
  { "deep-both-off", 1, 1,  0,  0, -1 },
};
const int EXPT_STEP_COUNT = sizeof(EXPT_STEPS) / sizeof(EXPT_STEPS[0]);

int           exptStep        = -1;    // -1 = not running
uint16_t      exptStepSec     = 10;
unsigned long exptStepStartMs = 0;
uint32_t      exptParkTicks   = 0;

// ── Applying the pin map ──────────────────────────────────────────
// Safe to call at any time.  Pins that dropped out of the map are returned to
// INPUT first: on a breadboard a stale push-pull output left driving a node
// that now has a jumper to something else is a short, not just a bug.
static uint8_t pinMapPrev[12];
static bool    pinMapValid = false;
bool           pinMapFellBack = false;   // reported by !PINS and setup()

static int pinRequired(uint8_t v, int dflt) {
  if (pinIsValid(v)) return (int)v;
  pinMapFellBack = true;
  return dflt;
}

void applyPinMap() {
  pinMapFellBack = false;
  uint8_t now[12] = {
    cfg.pinSensePos, cfg.pinSenseNeg, cfg.pinCharge, cfg.pinMosfet,
    cfg.pinSpkA,     cfg.pinSpkB,     cfg.pinLedData,
    cfg.pinDipA,     cfg.pinDipB,
    cfg.gAnaPin,     cfg.gLedPin,     cfg.gAuxPin,
  };
  if (pinMapValid) {
    for (int i = 0; i < 12; i++) {
      uint8_t old = pinMapPrev[i];
      if (!pinIsValid(old)) continue;
      bool stillUsed = false;
      for (int j = 0; j < 12; j++) if (now[j] == old) stillUsed = true;
      if (!stillUsed) pinMode(old, INPUT);
    }
  }
  memcpy(pinMapPrev, now, sizeof(now));
  pinMapValid = true;

  // Required pins: fall back to the as-designed V3 wiring rather than driving
  // a pin number this core does not have.
  SENSE_POS     = pinRequired(cfg.pinSensePos, A2);
  SENSE_NEG     = pinRequired(cfg.pinSenseNeg, A1);
  CHARGE_PIN    = pinRequired(cfg.pinCharge,   A3);
  MOSFET_PIN    = pinRequired(cfg.pinMosfet,   D7);
  SPEAKER_PIN   = pinRequired(cfg.pinSpkA,     D9);
  LED_PIN       = pinRequired(cfg.pinLedData,  6);
  // Optional pins: PIN_NONE is legal and every use of these is already guarded
  // by cfg.hwRev, so pass them through untouched.
  SPEAKER_PIN_B = (int)cfg.pinSpkB;
  DIP_PIN_A     = (int)cfg.pinDipA;
  DIP_PIN_B     = (int)cfg.pinDipB;

  pinMode(SENSE_POS, INPUT);
  pinMode(SENSE_NEG, INPUT);
  pinMode(CHARGE_PIN, INPUT);
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, MOSFET_ON);     // resting state
  pixel.setPin(LED_PIN);

  // Gate pins.  Driven to their resting level immediately -- an output left
  // floating between here and the first gatesRest() is a load switch with an
  // undefined state.
  for (uint8_t i = 0; i < GATE_COUNT; i++)
    if (pinIsValid(*GATE_PIN[i])) pinMode(*GATE_PIN[i], OUTPUT);
  gatesRest(gatesParked);

  // Speaker legs and the DIP inputs belong to buzzerApplyPinModes(), which is
  // the single place hwRev decides what D8/D10 are.  Call it last so it wins.
  buzzerApplyPinModes();
}

enum LeadState { STATE_FLOAT, STATE_CLOSED, STATE_VOLTAGE };
LeadState leadState = STATE_VOLTAGE;

// Explicit prototypes (see note at the ConfigField struct).
LeadState runMosfetTest();
LeadState runMosfetTestStable();
LeadState lowPowerProbe();
void      slogRecord(LeadState s, bool awake);

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

// BENCH: "is charge detection allowed to stop us doing things right now".
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
unsigned int  beepFreq = 0;
bool speakerMuted = false;    // session mute (leads closed at boot)

// Debug values
float lastRestV = 0.0f, lastTestV = 0.0f;
float lastMetric = 0.0f;      // scalar actually compared to the threshold
float lastReturnMs = 0.0f;    // method 1 result (ms), or window on timeout
float lastAreaVms  = 0.0f;    // method 2 result (V*ms)
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

// ── Two-stage sleep (bench fork) ──────────────────────────────────
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
const int SLOG_MAX = 64;
struct SleepLogEntry {
  float   metric;      // what was compared against the threshold
  float   retms;       // method 1/2 recovery time (ms)
  float   rest;        // resting differential at the probe (V)
  float   thr;         // threshold in force (the selector stays live while asleep)
  uint8_t state;       // LeadState the probe decided
  uint8_t awake;       // 1 = sampled by the awake loop, 0 = by the sleeping probe
  uint8_t stage;       // sleepStage at the time: 0 awake, 1 light sleep, 2 deep
};
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
static void ledPower(bool on) {
  digitalWrite(RGB_POWER_PIN, on ? HIGH : LOW);
  if (on) {
    sleepMs(1);                 // let the rail come up before clocking data out
    pixel.clear();
    pixel.show();
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
  gatesRest(true);                       // BENCH: mode-1 gates drop here
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
  gatesRest(false);                      // BENCH: back to the awake resting state
  ledPower(true);

  // millis() froze while we were in Standby, so the idle timer has to be
  // re-based here rather than carried across the sleep.
  idleSinceMs = millis();
}

// One probe pass: is anything still there?  Runs the real runMosfetTest() so
// every detection method (SINGLE / TIMERET / AREA) and the live DIP threshold
// behave exactly as they do awake -- only the averaging is cut, and the test's
// trailing settle is suppressed because the pin is about to be parked anyway.
// Returns STATE_FLOAT if the leads are still open.
LeadState lowPowerProbe() {
  // The selector stays live while asleep (hwRev 2: the DIP pins are still read;
  // hwRev 3: cfg.threshSel is just a memory read), but refresh the threshold
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
  // BENCH: same argument, extended to the external gates -- whatever the
  // analog front end needs comes up here and is dropped again on the way out.
  uint8_t gatesRaised = gatesMeasureBegin();
  sleepMs(1);                                 // let the rail come up
  pixel.clear();
  pixel.show();                               // hold it dark, not whatever it powered up as

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

  applyBridgePark();                    // BENCH: stage 2 may park differently

  gatesMeasureEnd(gatesRaised);         // BENCH: drop whatever this probe raised

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
  slog[slogHead].metric = lastMetric;
  slog[slogHead].retms  = lastReturnMs;
  slog[slogHead].rest   = lastRestV;
  slog[slogHead].thr    = awake ? activeThreshV : lastProbeThreshV;
  slog[slogHead].state  = (uint8_t)s;
  slog[slogHead].awake  = awake ? 1 : 0;
  slog[slogHead].stage  = awake ? 0 : sleepStage;
  slogHead = (slogHead + 1) % SLOG_MAX;
  if (slogValid < SLOG_MAX) slogValid++;
  slogTotal++;
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
static bool lowPowerAllowed() {
  if (millis() < SLEEP_BOOT_GRACE_MS) return false;
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
  // BENCH: skipped entirely when CHGINHIBIT=0, not merely ignored -- with a
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
    gatesRest(true);                     // BENCH: !FLOOR counts as parked
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
//   hwRev 3  the DIP switches are gone from the PCB and D8 is a buzzer leg, so
//            the selection moves into EEPROM as cfg.threshSel.  The index space
//            and every downstream user (sleepThresh[], the $DIP message, the
//            GUI's config table) are deliberately unchanged.
uint8_t readDipIndex() {
  if (cfg.hwRev >= 3) return cfg.threshSel & 0x03;
  uint8_t a = digitalRead(DIP_PIN_A) ? 1 : 0;   // D8
  uint8_t b = digitalRead(DIP_PIN_B) ? 1 : 0;   // D10
  return (a << 1) | b;
}

// Refresh activeThreshV from the selector; announce live changes on serial.
// Still reported as $DIP on hwRev 3 -- the host tooling keys on that name, and
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
// cfg.negFix is on it returns the count for cfg.negFixV instead, so the diff
// rides on a clean fixed pseudo-reference (the shipped default).
int readNegRaw() {
  if (cfg.negFix) {
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
  if (!cfg.negFix) analogRead(SENSE_NEG);
  int rawNeg = readNegRaw();
  return ((rawPos - rawNeg) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

// Decide whether a real voltage is present (MOSFET resting / on):
// any single read beyond voltFastMult * refBand -> present immediately;
// otherwise average voltAvgSamples reads and test against refBand.
bool voltagePresent() {
  float sum = 0.0f;
  for (int i = 0; i < cfg.voltAvgSamples; i++) {
    float v = readVoltage();
    if (fabs(v - cfg.refCenterV) > cfg.voltFastMult * cfg.refBandV) {
      lastRestV = v;
      return true;
    }
    sum += v;
  }
  lastRestV = sum / cfg.voltAvgSamples;
  return (fabs(lastRestV - cfg.refCenterV) > cfg.refBandV);
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
void setPixel(uint8_t r, uint8_t g, uint8_t b) {
  // BENCH FORK: LEDG mode 2 keeps LED1's supply down except while something is
  // actually being displayed, which is the experiment that cannot be run on a
  // V3 board at all (LED1 is wired straight to BatteryRail there).  Modes 0
  // and 1 leave the gate to gatesRest() and this is a plain pixel write.
  bool lit    = (r || g || b);
  bool pulsed = (*GATE_MODE[GATE_LED] == 2) && (gateForce[GATE_LED] < 0)
                && pinIsValid(*GATE_PIN[GATE_LED]);
  if (pulsed && lit && !gateOn[GATE_LED]) {
    gateWrite(GATE_LED, true);
    gateDelayUs(gateSettleUs(GATE_LED));    // rail up before any data goes out
  }
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  // Drop it only on the way to dark.  Cutting the rail with a colour still
  // latched would make the next flash's first bit land on an unpowered part.
  if (pulsed && !lit) gateWrite(GATE_LED, false);
}

// Rate-limited flash: on for onMs, then off, and no new flash until minGapMs
// after the last one began.  Flags/timestamps owned per-state by the caller.
void flashState(unsigned long now,
                uint8_t r, uint8_t g, uint8_t b,
                unsigned long onMs, unsigned long minGapMs,
                bool &flashing, unsigned long &lastFlash) {
  if (!flashing && (now - lastFlash >= minGapMs)) {
    flashing  = true;
    lastFlash = now;
    setPixel(r, g, b);
  } else if (flashing && (now - lastFlash >= onMs)) {
    flashing = false;
    setPixel(0, 0, 0);
  }
}

void updateLed() {
  static LeadState prevState = STATE_VOLTAGE;
  unsigned long now = millis();

  // On a state change, end any in-progress flash but keep the lastXFlash
  // timestamps: each state's rate limit persists across transitions so a
  // bouncing state can't re-fire immediately.
  if (leadState != prevState) {
    prevState      = leadState;
    floatFlashing  = false;
    closedFlashing = false;
    voltFlashing   = false;
    setPixel(0, 0, 0);
  }

  if (!cfg.ledEnable) { setPixel(0, 0, 0); return; }

  // One channel per state: blue = floating, green = closed, red = voltage.
  switch (leadState) {
    case STATE_FLOAT:
      flashState(now, 0, 0, cfg.ledFloatBright,
                 cfg.ledFloatMs, cfg.ledFloatPerMs, floatFlashing, lastFloatFlash);
      break;
    case STATE_CLOSED:
      flashState(now, 0, cfg.ledClosedBright, 0,
                 cfg.ledClosedMs, cfg.ledClosedPerMs, closedFlashing, lastClosedFlash);
      break;
    case STATE_VOLTAGE:
      flashState(now, cfg.ledVoltBright, 0, 0,
                 cfg.ledVoltMs, cfg.ledVoltPerMs, voltFlashing, lastVoltFlash);
      break;
  }
}

// ══════════════════════════════════════════════════════════════════
//  SPEAKER
// ══════════════════════════════════════════════════════════════════
// V2 wires one leg of the piezo to D9 and the other to ground, so a tone is
// just a square wave on D9 and the core's tone() does the whole job.
//
// V3 wires the element BETWEEN D8 and D9 (BZ1+ through R16, BZ1- through R17).
// Holding D8 low reproduces the V2 drive exactly; driving it INVERTED against
// D9 puts twice the voltage across the element -- about +6 dB for no extra
// parts.  That needs both pins toggled from one timer, which tone() cannot do
// (it owns a single pin), so cfg.spkDiff routes through a private FspTimer.
//
// The tidier hardware route -- a GPT channel's complementary GTIOCnA/GTIOCnB
// pair -- is not available on these pins: on the XIAO RA4M1, D8 is P111 =
// GTIOC3A and D9 is P110 = GTIOC1B, which are different channels.  (D9 and D10
// *are* a complementary pair, GTIOC1B/GTIOC1A, but D10 is not wired to the
// buzzer on V3.)  Hence the software toggle below.

static FspTimer     buzzTimer;
static bool         buzzTimerOpen = false;   // FspTimer channel claimed
static volatile bool buzzPhase    = false;

// True when the anti-phase drive should be used: V3 hardware, enabled in
// config, and a pitched (passive-buzzer) tone rather than a DC level.  On
// hwRev 2, D8 is a DIP input and must never be driven.
static inline bool buzzDifferential() {
  return (cfg.hwRev >= 3) && cfg.spkDiff && cfg.passiveBuzzer;
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

// Set D8/D10 to the roles the current cfg.hwRev calls for.  This is the ONLY
// place either pin's direction is decided -- call it after anything that can
// change hwRev (boot, !LOAD, !DEFAULTS, !SET,HWREV).
void buzzerApplyPinModes() {
  // Silence first.  This can be called with a tone running (!SET,HWREV while
  // the board is beeping), and the pin roles are about to change under it.
  if (buzzTimerOpen) buzzTimer.stop();
  noTone(SPEAKER_PIN);
  buzzPhase = false;

  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, SPEAKER_OFF);

  if (cfg.hwRev >= 3) {
    // V3: D8 is the buzzer's second leg.  D10 is a pure no-connect (broken out
    // on J6 only), so park it as a driven low rather than leaving a floating
    // input -- a floating CMOS input burns crossbar current, which matters at
    // the sleeping-current numbers this board is tuned to.
    pinMode(SPEAKER_PIN_B, OUTPUT);
    digitalWrite(SPEAKER_PIN_B, LOW);
    pinMode(DIP_PIN_B, OUTPUT);
    digitalWrite(DIP_PIN_B, LOW);
  } else {
    // V2: both DIP pins are inputs (hardware pull-ups on the PCB).
    pinMode(DIP_PIN_A, INPUT);
    pinMode(DIP_PIN_B, INPUT);
  }
}

// Passive buzzer (cfg.passiveBuzzer): square wave at `freq`, so each alert can
// have its own pitch.  Active buzzer: DC level, fixed tone.
void speakerOn(unsigned int freq) {
  if (buzzDifferential() && buzzTimerRun(freq)) return;

  // Not using the anti-phase timer for this pulse.  Stop it explicitly rather
  // than assuming speakerOff() already did: a priority alert preempts a beep in
  // progress by calling startBeep(force) -> speakerOn() with no speakerOff()
  // in between, so a PASSIVE or SPKDIFF change between pulses could otherwise
  // leave the ISR toggling both legs underneath the drive selected here.
  if (buzzTimerOpen) buzzTimer.stop();
  if (cfg.hwRev >= 3) digitalWrite(SPEAKER_PIN_B, LOW);   // park the second leg

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
  if (buzzTimerOpen) buzzTimer.stop();
  noTone(SPEAKER_PIN);                        // harmless when not toning
  digitalWrite(SPEAKER_PIN, SPEAKER_OFF);
  // Both legs low = no voltage across the element and no static current, which
  // is what the low-power park and !FLOOR measurements assume.
  if (cfg.hwRev >= 3) digitalWrite(SPEAKER_PIN_B, LOW);
  buzzPhase = false;
}

// Begin a rate-limited sequence of `pulses` beeps.  `force` preempts any
// in-progress sequence and ignores the rate cap (priority alerts).
void startBeep(unsigned long now, int pulses, unsigned long onMs, unsigned long offMs,
               bool force, unsigned int freq) {
  if (!force) {
    if (beepOn || beepPulsesLeft > 0)             return;
    if (now - lastBeepSeqStart < cfg.beepMinMs)   return;
  }
  lastBeepSeqStart = now;
  beepPulsesLeft   = pulses;
  beepOnMs         = onMs;
  beepOffMs        = offMs;
  beepFreq         = freq;
  beepPhaseStart   = now;
  beepOn           = true;
  speakerOn(beepFreq);
  beepPulsesLeft--;
}

void updateBeep(unsigned long now) {
  if (!beepOn && beepPulsesLeft == 0) return;
  if (beepOn) {
    if (now - beepPhaseStart >= beepOnMs) {
      speakerOff();
      beepOn         = false;
      beepPhaseStart = now;
    }
  } else if (beepPulsesLeft > 0 && now - beepPhaseStart >= beepOffMs) {
    speakerOn(beepFreq);
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
// Suppressed while charging (same lockout as the LED) and when muted/disabled.
void updateSpeaker() {
  static LeadState prevState = STATE_VOLTAGE;
  unsigned long now = millis();

  if (!cfg.beepEnable || speakerMuted) {
    silenceSpeaker();
    prevState = leadState;
    return;
  }
  if (chargeInhibits() && !alertOverride) {
    silenceSpeaker();
    prevState = leadState;      // avoid a stale beep on unplug
    return;
  }

  bool entered = (leadState != prevState);
  prevState = leadState;

  if (leadState == STATE_CLOSED) {
    if (entered)
      startBeep(now, cfg.contPulses, cfg.contOnMs, cfg.contOffMs, false, cfg.contFreqHz);
    else if (cfg.contRepeat && now - lastBeepSeqStart >= cfg.contRepeatMs)
      startBeep(now, cfg.contPulses, cfg.contHoldMs, cfg.contOffMs, false, cfg.contFreqHz);
  } else if (leadState == STATE_VOLTAGE) {
    if (entered)                // priority alert: always sounds on entry
      startBeep(now, cfg.voltPulses, cfg.voltOnMs, cfg.voltOffMs, true, cfg.voltFreqHz);
    else if (cfg.voltRepeat && now - lastBeepSeqStart >= cfg.voltRepeatMs)
      startBeep(now, cfg.voltPulses, cfg.voltOnMs, cfg.voltOffMs, false, cfg.voltFreqHz);
  }

  updateBeep(now);
}

// One clean measurement at boot to decide the session mute: leads shorted
// (CLOSED) at boot -> speaker muted until the next boot cycle.
void bootSpeakerMuteCheck() {
  if (!cfg.bootMute) return;
  updateThresholdFromDip();
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  if (voltagePresent()) return;          // voltage at boot -> leave audio enabled
  if (runMosfetTestStable() == STATE_CLOSED) speakerMuted = true;
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
  // BENCH: raise any pulsed (mode 2) gate for the measurement and settle.
  // No-op unless a gate is wired AND in mode 2, so the awake path is otherwise
  // exactly the production one.
  uint8_t gatesRaised = gatesMeasureBegin();

  updateThresholdFromDip();              // selector re-read every pass
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state

  bool present;
  if (voltOverride == VOLT_FORCE_ON) {
    lastRestV = readVoltage();           // keep a fresh reading for debug
    present = true;
  } else if (voltOverride == VOLT_DISABLED) {
    present = false;
  } else {
    present = voltagePresent();
  }

  LeadState rawState = present ? STATE_VOLTAGE : runMosfetTestStable();

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

  gatesMeasureEnd(gatesRaised);          // BENCH: drop the pulsed gates again
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
  Serial.print(",dip=");          Serial.print(dipIdx);   // hwRev 3: = THRESHSEL
  Serial.print(",spkdiff=");      Serial.print(buzzDifferential() ? 1 : 0);
  Serial.print(",openthr=");      Serial.print(activeThreshV, 3);
  Serial.print(",detmethod=");    Serial.print(cfg.detectMethod);
  Serial.print(",metric=");       Serial.print(lastMetric, 4);
  Serial.print(",retms=");        Serial.print(lastReturnMs, 3);
  Serial.print(",areavms=");      Serial.print(lastAreaVms, 4);
  Serial.print(",negfix=");       Serial.print(cfg.negFix ? 1 : 0);
  Serial.print(",negv=");         Serial.print(cfg.negFixV, 3);
  Serial.print(",charge=");       Serial.print(chargeActive ? 1 : 0);
  Serial.print(",alertovr=");     Serial.print(alertOverride ? 1 : 0);
  Serial.print(",chginhibit=");   Serial.print(cfg.chargeInhibit ? 1 : 0);
  Serial.print(",muted=");        Serial.print(speakerMuted ? 1 : 0);
  Serial.print(",dirty=");        Serial.print(cfgDirty ? 1 : 0);
  Serial.print(",lp=");           Serial.print(lowPowerActive ? 1 : 0);
  // BENCH: which sleep stage, and the stage-2 schedule actually in force.
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
  // BENCH: gate levels as three digits (ANA LED AUX), the !GATE holds as three
  // characters (a = auto, 0/1 = held), and the running experiment step.
  Serial.print(",gates=");
  for (uint8_t i = 0; i < GATE_COUNT; i++) Serial.print(gateOn[i] ? 1 : 0);
  Serial.print(",gforce=");
  for (uint8_t i = 0; i < GATE_COUNT; i++)
    Serial.print(gateForce[i] < 0 ? "a" : (gateForce[i] ? "1" : "0"));
  Serial.print(",expt=");         Serial.print(exptStep);
  Serial.print(",sn=");           Serial.println(unitSN);  // last: may be empty
}

// One streamed sample: both pins independently + computed differential.
void streamSample() {
  // BENCH: the live stream reads the analog front end, so it needs the same
  // rail the detector gets.  Without this, a pulsed (MODE 2) analog gate is
  // DOWN for every sample taken here -- the gate is only raised inside a
  // detection pass -- and the stream plots a flat, meaningless line.
  uint8_t gatesRaised = gatesMeasureBegin();

  analogRead(SENSE_POS);                 // throwaway: settle S/H after prior channel
  int rawPos = analogRead(SENSE_POS);
  if (!cfg.negFix) analogRead(SENSE_NEG);
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
// BENCH: when a pulsed (MODE 2) gate is in play the capture also has to bring
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
//  BENCH FORK: PIN / GATE REPORTING AND THE POWER-PROFILE EXPERIMENT
// ═══════════════════════════════════════════════════════════════

// One $PIN row: what a function is currently wired to.
static void printPinRow(const char *func, int pin) {
  Serial.print("$PIN,");
  Serial.print(func);
  Serial.print(",");
  if (!pinIsValid((uint8_t)pin)) { Serial.println("none,-"); return; }
  Serial.print(pin);
  Serial.print(",");
  Serial.println(pinNameOf((uint8_t)pin));
}

// !PINS -- the live map, then this core's whole name-to-number table.  The
// table is the part you actually need at the breadboard: it is the only place
// the numbers a PIN* key wants are written down, and it shows which analog and
// digital names share a pad on this module.
void dumpPinMap() {
  printPinRow("SENSEP",  SENSE_POS);
  printPinRow("SENSEN",  SENSE_NEG);
  printPinRow("CHARGE",  CHARGE_PIN);
  printPinRow("MOSFET",  MOSFET_PIN);
  printPinRow("SPKA",    SPEAKER_PIN);
  printPinRow("SPKB",    SPEAKER_PIN_B);
  printPinRow("LEDDATA", LED_PIN);
  printPinRow("DIPA",    DIP_PIN_A);
  printPinRow("DIPB",    DIP_PIN_B);
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    char f[12];
    snprintf(f, sizeof(f), "GATE%s", GATE_NAME[i]);
    printPinRow(f, (int)*GATE_PIN[i]);
  }
  // Fixed by the XIAO module itself, not remappable -- listed so the map is a
  // complete account of what the firmware drives.
  Serial.print("$PIN,RGBEN,");  Serial.print(RGB_POWER_PIN);  Serial.println(",fixed");
  Serial.print("$PIN,BATTEN,"); Serial.print(BATT_EN_PIN);    Serial.println(",fixed");
  Serial.print("$PIN,BATTADC,");Serial.print(BATT_PIN);       Serial.println(",fixed");

  for (int i = 0; i < PIN_NAME_COUNT; i++) {
    Serial.print("$PINNAME,");
    Serial.print(PIN_NAMES[i].name);
    Serial.print(",");
    Serial.println(PIN_NAMES[i].num);
  }
  if (pinMapFellBack)
    Serial.println("$ERR,pins,one or more PIN* keys are not pins on this board -- defaults used");

  // Two functions on one pin is legal and sometimes deliberate -- A1 is a hard
  // no-connect on V3, so using it as a gate output while SENSE_NEG nominally
  // still names it is exactly the right move (NEGFIX=1 means SENSE_NEG is never
  // read).  It is also an easy way to break something by accident, so say so
  // rather than leaving it to be discovered.  applyPinMap() configures in a
  // fixed order and the LAST writer wins: sense/charge inputs, then MOSFET,
  // then the gates, then buzzerApplyPinModes().
  int mapped[12] = { SENSE_POS, SENSE_NEG, CHARGE_PIN, MOSFET_PIN,
                     SPEAKER_PIN, SPEAKER_PIN_B, LED_PIN, DIP_PIN_A, DIP_PIN_B,
                     (int)*GATE_PIN[0], (int)*GATE_PIN[1], (int)*GATE_PIN[2] };
  const char *names[12] = { "SENSEP", "SENSEN", "CHARGE", "MOSFET",
                            "SPKA", "SPKB", "LEDDATA", "DIPA", "DIPB",
                            "GATEANA", "GATELED", "GATEAUX" };
  for (int a = 0; a < 12; a++) {
    if (!pinIsValid((uint8_t)mapped[a])) continue;
    for (int b = a + 1; b < 12; b++) {
      if (mapped[b] != mapped[a]) continue;
      Serial.print("$PINSHARED,");
      Serial.print(pinNameOf((uint8_t)mapped[a]));  Serial.print(",");
      Serial.print(names[a]);                        Serial.print(",");
      Serial.println(names[b]);
    }
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
// a !GATE hold overrides the mode entirely.
void dumpGates() {
  for (uint8_t i = 0; i < GATE_COUNT; i++) {
    Serial.print("$GATE,");
    Serial.print(GATE_NAME[i]);
    Serial.print(",pin=");
    if (pinIsValid(*GATE_PIN[i])) Serial.print(*GATE_PIN[i]); else Serial.print("none");
    Serial.print(",pol=");    Serial.print(*GATE_POL[i]);
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
  gateForce[GATE_AUX] = s.aux;

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
    // HWREV re-assigns what D8/D10 physically are, so the pins have to be
    // re-configured before anything drives them again.
    if (strcmp(f->name, "HWREV") == 0) { silenceSpeaker(); buzzerApplyPinModes(); }
    // BENCH: a pin or gate key describes the wiring, so it has to reach the
    // hardware now -- including releasing whatever pin it stopped using.
    if (strncmp(f->name, "PIN", 3) == 0 || strcmp(f->name, "ANAPOL") == 0 ||
        strcmp(f->name, "LEDGPOL") == 0 || strcmp(f->name, "AUXPOL") == 0 ||
        strcmp(f->name, "ANAMODE") == 0 || strcmp(f->name, "LEDGMODE") == 0 ||
        strcmp(f->name, "AUXMODE") == 0) {
      silenceSpeaker();
      applyPinMap();
    }
    // A1 is a no-connect on V3, so a live SENSE_NEG read there is just a
    // floating pin.  Allowed (it is occasionally worth looking at on the
    // bench) but never silent -- this is otherwise a baffling failure.
    if (strcmp(f->name, "NEGFIX") == 0 && !cfg.negFix && cfg.hwRev >= 3)
      Serial.println("$ERR,set,NEGFIX=0 with HWREV=3: A1 is not connected on V3");
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
      applyPinMap();                     // BENCH: also re-does buzzerApplyPinModes()
      refreshSleepSchedule();            // BENCH: the reloaded image may change either stage
      Serial.println("$OK,load");
    } else {
      Serial.println("$ERR,load,stored config invalid");
    }
  } else if (strcmp(cmd, "DEFAULTS") == 0) {
    // HWREV describes the PCB this XIAO is plugged into, not a preference, so
    // it survives a factory reset the way the serial number does.  Letting it
    // revert to the default 3 would hand D8 to the buzzer on a V2 board and
    // drive a push-pull output into whatever its DIP switch is doing.  Change
    // it deliberately with !SET,HWREV if a board is genuinely rebuilt.
    uint8_t keepHwRev = cfg.hwRev;
    configDefaults();
    cfg.hwRev = keepHwRev;
    silenceSpeaker();
    applyPinMap();                       // BENCH: pin map reverts with the rest
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
    if (arg && atoi(arg) == 0) {
      slogValid = 0; slogHead = 0; slogTotal = 0;
      Serial.println("$OK,sleeplog,cleared");
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
    // !PINS -- the live pin map plus this core's name-to-number table.  Read
    // it before setting any PIN* key: the keys take NUMBERS, and on the XIAO
    // the analog and digital names alias each other.
    dumpPinMap();
  } else if (strcmp(cmd, "GATE") == 0) {
    // !GATE                     report all three gates
    // !GATE,<ANA|LED|AUX>       report one
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
      if (gateForce[idx] >= 0 && !pinIsValid(*GATE_PIN[idx]))
        Serial.println("$ERR,gate,held but no pin assigned (set PINANA/PINLEDG/PINAUX)");
    }
    dumpGates();
  } else if (strcmp(cmd, "EXPT") == 0) {
    // !EXPT[,<sec>]  run the eight-step power profile, <sec> per step (10 s
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
  // BENCH: this one read happens BEFORE the config load, so it uses the
  // compiled-in A3 rather than PINCHARGE.  That is fine for what it is -- a
  // "is USB attached" check to decide whether the CDC settle below is worth
  // paying for -- but if PINCHARGE is ever remapped, remember the boot USB
  // detection still looks at A3.  applyPinMap() takes over immediately after.
  pinMode(CHARGE_PIN, INPUT);            // A3 = VBUS/2 (USB-power sense)

  Serial.begin(115200);
  // The USB settle only buys anything when a host is actually attached.  On
  // battery -- the case that decides time-to-first-measurement -- it is pure
  // dead time, so gate it on VBUS.  cfg is not loaded yet, hence the literal
  // threshold instead of cfg.chargeThreshV.
  readChargeV();                         // throwaway: first conversion after reset
  if (readChargeV() > BOOT_USB_THRESH_V) delay(300);

  // Config first: everything below reads cfg.
  configDefaults();
  bool loaded   = configLoad();
  bool migrated = false;
  if (!loaded) {
    // Not a current-version image.  Before falling back to defaults, see whether
    // it is an older BENCH layout worth carrying forward -- see
    // configMigrateBench() for why this fork now has one.
    migrated = configMigrateBench();
    configSave();                        // persist the migration (or seed defaults)
  }
  snLoad();                              // unit serial number (shared block)

  // BENCH: one call does every pin the firmware drives -- sense, MOSFET,
  // charge sense, LED data, the gates, and (via buzzerApplyPinModes) the
  // speaker legs and DIP inputs.  It has to come after the config load, both
  // because cfg.hwRev decides what D8 is and because the map itself is config.
  applyPinMap();
  gatesRest(false);

  // (BATT_PIN / BATT_EN_PIN are set up at the top of setup(), so the battery
  //  divider is already settling while the rest of this runs.  They are fixed
  //  XIAO module pins and deliberately not part of the remappable map.)

  pinMode(RGB_POWER_PIN, OUTPUT);        // onboard NeoPixel power rail
  digitalWrite(RGB_POWER_PIN, HIGH);

  pixel.begin();
  pixel.clear();
  pixel.show();

  startupBatteryIndicate();              // power-on battery charge-level cue

  bootSpeakerMuteCheck();                // leads CLOSED at boot -> session mute
  Serial.print("SN: ");
  Serial.println(unitSN[0] ? unitSN : "(unassigned -- write with !SN,<value>)");
  Serial.print("Config: ");
  if (loaded)        Serial.println("loaded from EEPROM (bench block, addr 1024)");
  else if (migrated) Serial.println("MIGRATED from an older bench layout "
                                    "(tuning preserved; new keys are at defaults)");
  else               Serial.println("defaults (bench block seeded at addr 1024)");
  Serial.print("Board: HWREV ");
  Serial.println(cfg.hwRev >= 3 ? "3 (V3: no DIP, differential buzzer)"
                                : "2 (V2: DIP switches, single-ended buzzer)");
  Serial.print("Speaker: ");
  Serial.print(speakerMuted ? "MUTED (leads closed at boot)" : "enabled");
  if (!cfg.passiveBuzzer)        Serial.println(", DC drive (PASSIVE=0)");
  else if (buzzDifferential())   Serial.println(", anti-phase D8/D9");
  else                           Serial.println(", single-ended D9");
  Serial.print(cfg.hwRev >= 3 ? "Threshold: THRESHSEL " : "DIP: ");
  Serial.print(readDipIndex());
  Serial.print(" -> threshold ");
  Serial.print(cfg.thresh[readDipIndex()], 3);
  Serial.println(" V");

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

  // BENCH: derive the stage-2 schedule now so cfg.deepHz holds the achieved
  // rate from the first !CFG, not the requested one.
  computeDeepSchedule();
  Serial.print("Deep sleep: ");
  if (cfg.deepSec == 0) {
    Serial.println("disabled (DEEPSEC=0) -- single-stage, as production");
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

  // Pin map and gates, printed every boot: on a breadboard the wiring is the
  // thing most likely to be wrong, and this is cheaper than remembering which
  // jumper went where between sessions.
  Serial.println("Pin map:");
  dumpPinMap();
  Serial.println("Power gates:");
  dumpGates();
  if (pinMapFellBack)
    Serial.println("WARNING: a PIN* key is not a pin on this board -- the default was used.");

  Serial.println("BlinkyHawk_Bench ready (experimental fork -- see !PINS / !GATE / !EXPT).");
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
      Serial.println("-> VOLTAGE (bypass)");
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
