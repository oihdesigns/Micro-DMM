/*!
 * AC Power Monitor / Logger
 * Derived from ads122c04_tests — same serial-command style, AC-specific math.
 *
 * ── Pick a front end: change ADC_BACKEND below, nothing else ────────────────
 *   BACKEND_ADS122C04  external 24-bit delta-sigma over I2C, truly differential
 *   BACKEND_RA4M1      the RA4M1's own ADC, 14 bit, pseudo-differential
 *
 *   Everything downstream of acquisition — the RMS and power maths, the
 *   windowing, the relay, the energy accumulator, the waveform capture, the
 *   serial protocol — is shared. Only the ~150 lines that fetch a sample
 *   differ, so the two backends cannot drift apart in the part that is hard
 *   to get right.
 *
 * ── Channels ────────────────────────────────────────────────────────────────
 *   ADS122C04:  AIN0 (+) / AIN1 (−) is mains voltage
 *               AIN2 (+) / AIN3 (−) is load current
 *   RA4M1:      A0 (+) / A1 (−) is mains voltage      (pins are settable,
 *               A2 (+) / A3 (−) is load current        see !VPIN / !IPIN)
 *   Both:       D5 is the relay driver, driven open at boot
 *
 * On the ADS122C04 both pairs are truly differential, so bias the front end
 * near mid-supply and leave the PGA bypassed (the default), which lets the
 * inputs swing rail to rail.
 *
 * The RA4M1 ADC is single-ended only, so a "pair" here is two conversions
 * subtracted: the + pin carries the signal biased at mid-supply, and the − pin
 * watches the bias node itself. Subtracting cancels bias and supply drift,
 * which is the whole point — but the two conversions are not simultaneous, so
 * the − pin must be a quiet reference, not the other half of an anti-phase
 * drive. Set the − pin to -1 for a plain single-ended read; per-window DC
 * removal then does the same job, just without rejecting supply noise.
 *
 * ── Scaling ─────────────────────────────────────────────────────────────────
 *   Each channel is "this many ADC volts differential = this much real world",
 *   defaulting to 1.5 V = 170 V and 1.5 V = 25 A. Both halves are tunable
 *   (!VSCALE / !ISCALE), so a different divider or CT is only a number change.
 *   170 V is the peak of 120 Vrms, so 1.5 V of ADC swing is one full-scale
 *   mains cycle — the readings reported here are RMS, not peak.
 *
 * ── How the two channels are measured at once (they are not) ────────────────
 *   Either way there is one converter, so V and I are sampled alternately.
 *
 *   On the ADS122C04 every mux change is followed by a START so each
 *   conversion is fully settled, which costs one conversion time per sample.
 *   At 1000 SPS with turbo that lands near 550-750 V/I pairs per second, about
 *   9-12 pairs per 60 Hz cycle — the tight case, and what the maths below is
 *   built for.
 *
 *   The RA4M1 is far quicker, so instead of racing it, acquisition is divided
 *   into equal time buckets and every conversion taken inside a bucket is
 *   averaged into one stored sample (SPC buckets per mains cycle, default 32).
 *   That fixes the sample grid the maths wants regardless of how fast
 *   analogRead actually is, spends the surplus speed on noise rather than
 *   throwing it away, and the boxcar is a genuine anti-alias filter — at
 *   32 samples per cycle it costs 0.02 % of the fundamental.
 *
 *   Two consequences are worth knowing:
 *
 *     - Each current sample sits half a step after the voltage sample beside
 *       it, so real power uses a voltage interpolated to the current instant —
 *       a cubic through v[k-1..k+2] evaluated at PHASECAL, 0.5 nominal, with
 *       the interpolator's own amplitude loss divided back out at the measured
 *       frequency. Against synthetic 50/60 Hz that returns P and PF exact to
 *       better than 0.01 % at any phase angle. Trimming PHASECAL off 0.5 also
 *       cancels the phase error of a CT or of an input filter, which is what
 *       makes the power factor believable.
 *     - On the ADS122C04, anything above ~300 Hz aliases. Real power and RMS
 *       are right for linear loads and approximate for choppy ones. When the
 *       shape is what matters, take a single-channel waveform capture instead:
 *       one mux, continuous mode, the full 2000 SPS — 33 points per 60 Hz
 *       cycle, good to the 16th harmonic. The RA4M1 does not have this problem
 *       at its default 32 samples per cycle.
 *
 * ── Measurement window ──────────────────────────────────────────────────────
 *   Sums run between two rising zero crossings of the voltage, over a whole
 *   number of cycles, with the crossings located to a fraction of a sample and
 *   the sums trapezoid-integrated to those fractional endpoints. Without that,
 *   the partial cycle left at each end would put about a percent of ripple on
 *   every reading at this sample density. The DC content of each channel is
 *   measured over that same window and removed, so a drifting bias point
 *   costs nothing.
 *
 * ── Commands (newline terminated) ───────────────────────────────────────────
 *   !START / !STOP        begin / end streaming of $PWR lines
 *   !CFG                  re-print $CFG + $ACFG
 *   !RELAY,<0|1>          open / close the load relay (a close clears a trip)
 *   !RLYINV,<0|1>         relay driver is active-low
 *   !VSCALE,<adcV>,<V>    e.g. !VSCALE,1.5,170
 *   !ISCALE,<adcV>,<A>    e.g. !ISCALE,1.5,25
 *   !PHASECAL,<f>         voltage interpolation weight, nominal 0.5
 *   !CYCLES,<n>           mains cycles per measurement window (default 10)
 *   !MAINS,<hz>           nominal mains frequency, sizes the window (default 60)
 *   !TRIP,<amps>          open the relay above this Irms; 0 disables
 *   !DCREM,<0|1>          remove each window DC content (default 1)
 *   !ZERO                 store the present readings as the channel offsets
 *   !VOFF,<uV> !IOFF,<uV> set those offsets directly
 *   !EZERO                reset the energy accumulator
 *   !VREF,<volts>         full-scale reference voltage of the converter
 *   !WAVE                 capture a waveform now
 *   !WLEN,<sec>           waveform capture length
 *   !WCH,<0|1|2>          capture voltage / current / both interleaved
 *
 *   ADS122C04 only:
 *   !VGAIN,<0-7> !IGAIN   per-channel gain index (0=1x … 7=128x)
 *   !PGA,<0|1>            PGA in circuit (needed above 4x; costs common mode)
 *   !RATE,<0-6>           data rate index
 *   !TURBO,<0|1>          turbo mode
 *   !I2C,<hz>             I2C clock — 1000000 buys ~35 % more pairs per second
 *
 *   RA4M1 only:
 *   !BITS,<8|10|12|14>    analogReadResolution (default 14)
 *   !SPC,<n>              stored samples per mains cycle (default 32)
 *   !VPIN,<p>,<n>         voltage + and − analog channels, 0-3; −1 = none
 *   !IPIN,<p>,<n>         current + and − analog channels
 *   !AREF,<0-4>           0 default/AVCC, 1 internal, 2 1.5 V, 3 2.0 V, 4 2.5 V
 *
 * ── Output ──────────────────────────────────────────────────────────────────
 *   $PWR,<vrms>,<irms>,<w>,<va>,<var>,<pf>,<hz>,<vpk>,<ipk>,<wh>,<uptime_s>,
 *        <pairs>,<pairs_per_s>,<relay>,<flags>
 *        flags: +1 no zero crossings  +2 V clipped  +4 I clipped
 *               +8 over-current trip  +16 window truncated  +32 ADC read failed
 *   $CFG,<backend>,<relay>,<streaming>            backend: 0 = ADS, 1 = RA4M1
 *   $ADC,0,<rate>,<turbo>,<pga>,<vgain>,<igain>,<i2c_hz>
 *   $ADC,1,<bits>,<spc>,<vp>,<vn>,<ip>,<in>,<aref>,<avg_depth>
 *   $ACFG,<v_adc_fs>,<v_fs>,<i_adc_fs>,<i_fs>,<phasecal>,<cycles>,<mains_hz>,
 *         <trip_a>,<dcrem>,<v_off_uv>,<i_off_uv>,<rly_inv>,<wave_len_s>,
 *         <wave_ch>,<vref>
 *   $WAVE,<n>,<ch>,<dt_us>,<lsb_v>,<lsb_i>,<flags>   then n x $WD then $WEND
 *        $WD,<v_raw>,<i_raw>  when ch = 2, otherwise $WD,<raw>
 *   $RLY,<0|1>        $TRIP,<irms>,<limit>        $ERR,<message>
 */

// ═════════════════════════════════════════════════════════════════════════════
//  CHANGE THIS ONE LINE TO PICK A FRONT END
// ═════════════════════════════════════════════════════════════════════════════
#define BACKEND_ADS122C04  0
#define BACKEND_RA4M1      1

#define ADC_BACKEND  BACKEND_RA4M1

// ═════════════════════════════════════════════════════════════════════════════

#include "ac_types.h"

#if ADC_BACKEND == BACKEND_ADS122C04

  #include <Wire.h>
  #include <Adafruit_ADS122C04.h>
  Adafruit_ADS122C04 ads;

  // The ADS122C04 is 24-bit, so samples are stored as raw signed counts.
  typedef int32_t sample_t;
  #define BUF_MAX 2048          // ~2 s interleaved / ~1 s single channel

#else

  // Samples are the mean of however many conversions landed in a time bucket,
  // so they carry a fraction and are stored as floats. The RA4M1 has 32 KB of
  // RAM, so the buffers are sized for a window rather than for seconds of
  // capture: 512 pairs is 16 cycles at the default 32 samples per cycle.
  typedef float sample_t;
  #define BUF_MAX 512

  // Analog channels, listed rather than computed: the XIAO RA4M1 has no A4 and
  // numbers PIN_A0 from 0, so A0 + n is not portable. Edit to suit the board.
  static const uint8_t ANALOG_PINS[] = { A0, A1, A2, A3 };
  #define ANALOG_PIN_COUNT ((int8_t)(sizeof(ANALOG_PINS) / sizeof(ANALOG_PINS[0])))

#endif

// ── board wiring ──────────────────────────────────────────────────────────────
#define RELAY_PIN 5

#if ADC_BACKEND == BACKEND_ADS122C04
// ── register-level constants ──────────────────────────────────────────────────
// The hot loop writes CONFIG0 and reads DRDY/data with raw Wire calls. The
// library setMux() is a read-modify-write plus a four-register cache refresh —
// around 500 us of I2C — which would halve the achievable pair rate when it has
// to run twice per V/I pair.
#define REG_CFG0        0x00
#define CMD_WREG(r)     (0x40 | ((r) << 2))
#define CMD_RREG(r)     (0x20 | ((r) << 2))
#define CMD_STARTSYNC   0x08
#define CMD_RDATA       0x10

#define MUX_V   ADS122C04_MUX_AIN0_AIN1   // mains voltage
#define MUX_I   ADS122C04_MUX_AIN2_AIN3   // load current

#define ADC_FULL_SCALE  8388608.0f        // 2^23
#define CLIP_RAW        8000000L          // ~95 % of full scale
#endif

// ── measurement buffers ───────────────────────────────────────────────────────
static sample_t g_vbuf[BUF_MAX];
static sample_t g_ibuf[BUF_MAX];

// ── result flags ──────────────────────────────────────────────────────────────
#define FLG_NOCROSS  0x01
#define FLG_VCLIP    0x02
#define FLG_ICLIP    0x04
#define FLG_TRIP     0x08
#define FLG_SHORT    0x10
#define FLG_ADCERR   0x20

// ── ADC config state ──────────────────────────────────────────────────────────
#if ADC_BACKEND == BACKEND_ADS122C04

static uint8_t  g_addr     = ADS122C04_DEFAULT_ADDR;
static uint8_t  g_rate_idx = 6;        // 1000 SPS
static bool     g_turbo    = true;     // -> 2000 SPS
static bool     g_pga      = false;    // bypassed: rail-to-rail common mode
static uint8_t  g_v_gain   = 0;        // 1x
static uint8_t  g_i_gain   = 0;        // 1x
static float    g_vref     = 2.048f;   // internal reference
static uint32_t g_i2c_hz   = 400000;
static uint32_t g_conv_us  = 500;      // one conversion at the current rate

#else

static uint8_t  g_bits     = 14;       // analogReadResolution
static uint16_t g_spc      = 32;       // stored samples per mains cycle
static int8_t   g_vp = 0,  g_vn = 1;   // indices into ANALOG_PINS; -1 = none
static int8_t   g_ip = 2,  g_in = 3;
static uint8_t  g_aref     = 0;        // index into AREF_TABLE
// The board's actual analog reference. It only sets the scale that !VSCALE /
// !ISCALE are then calibrated against, so getting it wrong costs nothing once
// the channels have been calibrated against a known load — but 3.3 here and a
// 5 V board would make the pre-calibration numbers look 50 % low.
static float    g_vref     = 3.3f;
static uint16_t g_adc_max  = 16383;    // (1 << bits) - 1
static uint16_t g_avg_depth = 0;       // conversions averaged per stored sample

static const uint8_t AREF_TABLE[] = {
  AR_DEFAULT, AR_INTERNAL, AR_INTERNAL_1_5V, AR_INTERNAL_2_0V, AR_INTERNAL_2_5V
};
#define AREF_COUNT ((uint8_t)(sizeof(AREF_TABLE) / sizeof(AREF_TABLE[0])))

#endif

// ── scaling / calibration ─────────────────────────────────────────────────────
static float g_v_adc_fs = 1.5f,  g_v_fs = 170.0f;   // 1.5 V diff = 170 V
static float g_i_adc_fs = 1.5f,  g_i_fs = 25.0f;    // 1.5 V diff = 25 A
static float g_v_off    = 0.0f,  g_i_off = 0.0f;    // ADC volts
static float g_phasecal = 0.5f;
static bool  g_dcrem    = true;

// ── windowing ─────────────────────────────────────────────────────────────────
static uint8_t g_cycles   = 10;
static float   g_mains_hz = 60.0f;

// ── relay / protection ────────────────────────────────────────────────────────
static bool  g_relay      = false;
static bool  g_rly_inv    = false;
static float g_trip_a     = 0.0f;      // 0 = disabled
static bool  g_trip_latch = false;

// ── energy / run state ────────────────────────────────────────────────────────
static double g_wh        = 0.0;
static bool   g_streaming = false;
// A measurement window runs for ~200 ms with the serial port unattended, which
// is a long time to sit on a !RELAY command. The pair loop drops out as soon as
// a byte arrives and the window is discarded, so relay latency is one sample
// rather than one window.
static bool   g_abort     = false;

// ── waveform capture ──────────────────────────────────────────────────────────
static float   g_wave_len_s   = 0.2f;
static uint8_t g_wave_ch      = 2;     // 0 = V, 1 = I, 2 = both interleaved
static bool    g_wave_pending = false;

#if ADC_BACKEND == BACKEND_ADS122C04
// ── lookup tables ─────────────────────────────────────────────────────────────
const ads122c04_gain_t GAIN_TABLE[] = {
  ADS122C04_GAIN_1,  ADS122C04_GAIN_2,  ADS122C04_GAIN_4,  ADS122C04_GAIN_8,
  ADS122C04_GAIN_16, ADS122C04_GAIN_32, ADS122C04_GAIN_64, ADS122C04_GAIN_128
};
const uint8_t GAIN_VALUE[] = {1, 2, 4, 8, 16, 32, 64, 128};
const uint8_t GAIN_COUNT   = sizeof(GAIN_TABLE) / sizeof(GAIN_TABLE[0]);

const ads122c04_rate_t RATE_TABLE[] = {
  ADS122C04_RATE_20SPS,  ADS122C04_RATE_45SPS,  ADS122C04_RATE_90SPS,
  ADS122C04_RATE_175SPS, ADS122C04_RATE_330SPS, ADS122C04_RATE_600SPS,
  ADS122C04_RATE_1000SPS
};
// nominal SPS [normal, turbo]
const uint16_t RATE_NOMINAL[][2] = {
  {20, 40}, {45, 90}, {90, 180}, {175, 350}, {330, 660}, {600, 1200}, {1000, 2000}
};
const uint8_t RATE_COUNT = sizeof(RATE_TABLE) / sizeof(RATE_TABLE[0]);

// ═══ raw I2C fast path ════════════════════════════════════════════════════════

static inline void adcCommand(uint8_t c) {
  Wire.beginTransmission(g_addr);
  Wire.write(c);
  Wire.endTransmission();
}

// CONFIG0 = MUX[7:4] | GAIN[3:1] | PGA_BYPASS[0]
static inline void adcSetChannel(uint8_t mux, uint8_t gain_idx) {
  Wire.beginTransmission(g_addr);
  Wire.write(CMD_WREG(REG_CFG0));
  Wire.write((uint8_t)((mux << 4) | (gain_idx << 1) | (g_pga ? 0 : 1)));
  Wire.endTransmission();
}

static inline bool adcReady() {
  Wire.beginTransmission(g_addr);
  Wire.write(CMD_RREG(2));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)g_addr, (uint8_t)1) != 1) return false;
  return (Wire.read() & 0x80) != 0;
}

static inline int32_t adcReadRaw() {
  Wire.beginTransmission(g_addr);
  Wire.write(CMD_RDATA);
  if (Wire.endTransmission(false) != 0) return INT32_MIN;
  if (Wire.requestFrom((uint8_t)g_addr, (uint8_t)3) != 3) return INT32_MIN;
  // separate reads: the evaluation order of the operands of a shifted OR is
  // unspecified, and these three bytes are order-critical
  uint8_t b0 = Wire.read();
  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  uint32_t r = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;
  if (r & 0x800000UL) r |= 0xFF000000UL;
  return (int32_t)r;
}

// One fully settled conversion on the requested channel. The START after the
// mux write restarts the modulator and the digital filter, so the result that
// follows is settled rather than a blend of the two inputs.
static int32_t readChannel(uint8_t mux, uint8_t gain_idx) {
  adcSetChannel(mux, gain_idx);
  adcCommand(CMD_STARTSYNC);
  delayMicroseconds(g_conv_us - (g_conv_us >> 2));   // idle through ~75 %
  uint32_t t0 = micros();
  while (!adcReady()) {
    if ((uint32_t)(micros() - t0) > g_conv_us * 4 + 3000) return INT32_MIN;
  }
  return adcReadRaw();
}

// ═══ configuration ════════════════════════════════════════════════════════════

static uint16_t nominalSps() { return RATE_NOMINAL[g_rate_idx][g_turbo ? 1 : 0]; }

// ADC volts per LSB on each channel — the reference over gain and full scale.
static inline float lsbV() {
  return g_vref / ((float)GAIN_VALUE[g_v_gain] * ADC_FULL_SCALE);
}
static inline float lsbI() {
  return g_vref / ((float)GAIN_VALUE[g_i_gain] * ADC_FULL_SCALE);
}

// The ADC sets the pace here, so a window is bounded by time, not by count.
static inline uint16_t targetPairs() { return BUF_MAX; }

void applyConfig() {
  Wire.setClock(g_i2c_hz);
  ads.setContinuousMode(false);          // single shot: one settled conversion
  ads.setDataRate(RATE_TABLE[g_rate_idx]);
  ads.setTurboMode(g_turbo);
  ads.enableTempSensor(false);
  adcSetChannel(MUX_V, g_v_gain);
  g_conv_us = 1000000UL / nominalSps();
}

#else   // ── BACKEND_RA4M1 ─────────────────────────────────────────────────────

// Both channels share the converter and its reference, so one LSB serves both.
static inline float lsbV() { return g_vref / (float)(g_adc_max + 1); }
static inline float lsbI() { return lsbV(); }

// Here the converter is faster than the maths needs, so the window is bounded
// by a sample count and the surplus speed is averaged away inside each bucket.
static inline uint16_t targetPairs() {
  uint32_t t = (uint32_t)g_spc * ((uint32_t)g_cycles + 2UL);
  return (t > BUF_MAX) ? (uint16_t)BUF_MAX : (uint16_t)t;
}

void applyConfig() {
  analogReference(AREF_TABLE[g_aref]);
  analogReadResolution(g_bits);
  g_adc_max = (uint16_t)((1UL << g_bits) - 1UL);
  for (int8_t k = 0; k < ANALOG_PIN_COUNT; k++) pinMode(ANALOG_PINS[k], INPUT);
}

// One pseudo-differential reading in raw counts. The two conversions are not
// simultaneous, so the − pin wants to be a quiet bias node rather than the
// other half of an anti-phase drive; clipping is caught on the individual
// conversions, because a difference of zero could equally be two rails.
static inline int32_t readPair(int8_t pp, int8_t pn, uint8_t &flags, uint8_t clipbit) {
  int32_t a = analogRead(ANALOG_PINS[pp]);
  if (a <= 2 || a >= (int32_t)g_adc_max - 2) flags |= clipbit;
  if (pn < 0) return a - (int32_t)(g_adc_max >> 1);
  int32_t b = analogRead(ANALOG_PINS[pn]);
  if (b <= 2 || b >= (int32_t)g_adc_max - 2) flags |= clipbit;
  return a - b;
}

#endif

void setRelay(bool on) {
  g_relay = on;
  digitalWrite(RELAY_PIN, (on != g_rly_inv) ? HIGH : LOW);
  Serial.print(F("$RLY,"));
  Serial.println(g_relay ? 1 : 0);
}

// Backend-specific settings get their own line, so the shared $CFG stays the
// same shape whichever front end is fitted and the GUI can pick a panel from
// the backend id before it has to parse anything else.
void printAdcConfig() {
#if ADC_BACKEND == BACKEND_ADS122C04
  Serial.print(F("$ADC,0,"));
  Serial.print(g_rate_idx);            Serial.print(',');
  Serial.print(g_turbo ? 1 : 0);       Serial.print(',');
  Serial.print(g_pga ? 1 : 0);         Serial.print(',');
  Serial.print(g_v_gain);              Serial.print(',');
  Serial.print(g_i_gain);              Serial.print(',');
  Serial.println(g_i2c_hz);
#else
  Serial.print(F("$ADC,1,"));
  Serial.print(g_bits);                Serial.print(',');
  Serial.print(g_spc);                 Serial.print(',');
  Serial.print(g_vp);                  Serial.print(',');
  Serial.print(g_vn);                  Serial.print(',');
  Serial.print(g_ip);                  Serial.print(',');
  Serial.print(g_in);                  Serial.print(',');
  Serial.print(g_aref);                Serial.print(',');
  Serial.println(g_avg_depth);
#endif
}

void printConfig() {
  Serial.print(F("$CFG,"));
  Serial.print(ADC_BACKEND);           Serial.print(',');
  Serial.print(g_relay ? 1 : 0);       Serial.print(',');
  Serial.println(g_streaming ? 1 : 0);
  printAdcConfig();
}

void printAcConfig() {
  Serial.print(F("$ACFG,"));
  Serial.print(g_v_adc_fs, 6);         Serial.print(',');
  Serial.print(g_v_fs, 4);             Serial.print(',');
  Serial.print(g_i_adc_fs, 6);         Serial.print(',');
  Serial.print(g_i_fs, 4);             Serial.print(',');
  Serial.print(g_phasecal, 4);         Serial.print(',');
  Serial.print(g_cycles);              Serial.print(',');
  Serial.print(g_mains_hz, 2);         Serial.print(',');
  Serial.print(g_trip_a, 3);           Serial.print(',');
  Serial.print(g_dcrem ? 1 : 0);       Serial.print(',');
  Serial.print(g_v_off * 1.0e6f, 2);   Serial.print(',');
  Serial.print(g_i_off * 1.0e6f, 2);   Serial.print(',');
  Serial.print(g_rly_inv ? 1 : 0);     Serial.print(',');
  Serial.print(g_wave_len_s, 4);       Serial.print(',');
  Serial.print(g_wave_ch);             Serial.print(',');
  Serial.println(g_vref, 6);
}

// ═══ acquisition ══════════════════════════════════════════════════════════════
//
// Both backends present the same two calls. dt_s always comes back as the
// *measured* spacing between voltage samples, never a nominal one, because
// that is the grid every later calculation indexes against.

#if ADC_BACKEND == BACKEND_ADS122C04

uint16_t acquirePairs(uint16_t maxn, uint32_t dur_us, bool abortable,
                      float &dt_s, uint8_t &flags) {
  uint16_t n = 0;
  uint32_t t_first = 0, t_last = 0;
  uint32_t t_start = micros();

  while (n < maxn && (uint32_t)(micros() - t_start) < dur_us) {
    if (abortable && Serial.available()) { g_abort = true; break; }
    uint32_t tv = micros();
    int32_t v = readChannel(MUX_V, g_v_gain);
    int32_t i = readChannel(MUX_I, g_i_gain);
    if (v == INT32_MIN || i == INT32_MIN) { flags |= FLG_ADCERR; break; }
    if (labs((long)v) > CLIP_RAW) flags |= FLG_VCLIP;
    if (labs((long)i) > CLIP_RAW) flags |= FLG_ICLIP;
    if (n == 0) t_first = tv;
    t_last = tv;
    g_vbuf[n] = v;
    g_ibuf[n] = i;
    n++;
  }
  if (n >= maxn) flags |= FLG_SHORT;      // ran out of buffer, not out of time
  dt_s = (n > 1) ? (float)(t_last - t_first) * 1.0e-6f / (float)(n - 1) : 0.0f;
  return n;
}

// One channel at the full data rate, continuous mode, mux untouched — this is
// the mode that actually resolves waveform shape.
uint16_t acquireSingle(uint8_t ch, uint16_t maxn, uint32_t dur_us,
                       float &dt_s, uint8_t &flags) {
  uint8_t mux  = (ch == 1) ? MUX_I    : MUX_V;
  uint8_t gain = (ch == 1) ? g_i_gain : g_v_gain;

  adcSetChannel(mux, gain);
  ads.setContinuousMode(true);
  adcCommand(CMD_STARTSYNC);

  // discard the first conversion — it straddles the mode change
  uint32_t t0 = micros();
  while (!adcReady()) {
    if ((uint32_t)(micros() - t0) > g_conv_us * 6 + 5000) { flags |= FLG_ADCERR; break; }
  }
  adcReadRaw();

  uint16_t n = 0;
  uint32_t t_first = 0, t_last = 0;
  uint32_t t_start = micros();
  while (n < maxn && (uint32_t)(micros() - t_start) < dur_us) {
    if (!adcReady()) continue;
    uint32_t t = micros();
    int32_t raw = adcReadRaw();
    if (raw == INT32_MIN) { flags |= FLG_ADCERR; break; }
    if (labs((long)raw) > CLIP_RAW) flags |= (ch == 1) ? FLG_ICLIP : FLG_VCLIP;
    if (n == 0) t_first = t;
    t_last = t;
    g_vbuf[n++] = raw;
  }
  if (n >= maxn) flags |= FLG_SHORT;
  dt_s = (n > 1) ? (float)(t_last - t_first) * 1.0e-6f / (float)(n - 1) : 0.0f;

  ads.setContinuousMode(false);
  return n;
}

#else   // ── BACKEND_RA4M1 ─────────────────────────────────────────────────────

// Acquisition is divided into maxn equal time buckets across dur_us, and every
// conversion that lands inside a bucket is averaged into one stored sample.
// The RA4M1 is quicker than the maths needs, so this spends the surplus on
// noise instead of discarding it, and pins the sample grid to a known rate
// rather than to whatever analogRead happens to cost. Each sample is stamped
// at the centre of its bucket's conversions, so dt stays honest even if the
// converter turns out to be slower than the requested bucket.
uint16_t acquirePairs(uint16_t maxn, uint32_t dur_us, bool abortable,
                      float &dt_s, uint8_t &flags) {
  if (maxn > BUF_MAX) maxn = BUF_MAX;
  if (maxn == 0) { dt_s = 0.0f; return 0; }

  uint32_t bucket_us = dur_us / maxn;
  if (bucket_us < 1) bucket_us = 1;

  uint16_t n = 0;
  uint32_t total_conv = 0;
  uint32_t first_mid = 0, last_mid = 0;
  uint32_t t_start = micros();

  while (n < maxn) {
    if (abortable && Serial.available()) { g_abort = true; break; }
    // a hard cap, in case the converter cannot keep up with the bucket rate
    if ((uint32_t)(micros() - t_start) > dur_us * 4UL + 100000UL) {
      flags |= FLG_SHORT;
      break;
    }

    uint32_t deadline  = t_start + (uint32_t)(n + 1) * bucket_us;
    uint32_t acc_start = micros();
    int32_t sv = 0, si = 0;
    uint16_t cnt = 0;
    do {
      sv += readPair(g_vp, g_vn, flags, FLG_VCLIP);
      si += readPair(g_ip, g_in, flags, FLG_ICLIP);
      cnt++;
    } while ((int32_t)(micros() - deadline) < 0 && cnt < 2000);
    uint32_t acc_end = micros();

    g_vbuf[n] = (float)sv / (float)cnt;
    g_ibuf[n] = (float)si / (float)cnt;
    total_conv += cnt;

    uint32_t mid = acc_start + (acc_end - acc_start) / 2;
    if (n == 0) first_mid = mid;
    last_mid = mid;
    n++;
  }

  g_avg_depth = n ? (uint16_t)(total_conv / n) : 0;
  dt_s = (n > 1) ? (float)(last_mid - first_mid) * 1.0e-6f / (float)(n - 1) : 0.0f;
  return n;
}

// The same bucketing on one channel, which simply halves the conversions per
// bucket and so doubles the averaging depth at a given rate.
uint16_t acquireSingle(uint8_t ch, uint16_t maxn, uint32_t dur_us,
                       float &dt_s, uint8_t &flags) {
  if (maxn > BUF_MAX) maxn = BUF_MAX;
  if (maxn == 0) { dt_s = 0.0f; return 0; }

  int8_t  pp = (ch == 1) ? g_ip : g_vp;
  int8_t  pn = (ch == 1) ? g_in : g_vn;
  uint8_t cb = (ch == 1) ? FLG_ICLIP : FLG_VCLIP;

  uint32_t bucket_us = dur_us / maxn;
  if (bucket_us < 1) bucket_us = 1;

  uint16_t n = 0;
  uint32_t total_conv = 0;
  uint32_t first_mid = 0, last_mid = 0;
  uint32_t t_start = micros();

  while (n < maxn) {
    if ((uint32_t)(micros() - t_start) > dur_us * 4UL + 100000UL) {
      flags |= FLG_SHORT;
      break;
    }
    uint32_t deadline  = t_start + (uint32_t)(n + 1) * bucket_us;
    uint32_t acc_start = micros();
    int32_t s = 0;
    uint16_t cnt = 0;
    do {
      s += readPair(pp, pn, flags, cb);
      cnt++;
    } while ((int32_t)(micros() - deadline) < 0 && cnt < 2000);
    uint32_t acc_end = micros();

    g_vbuf[n] = (float)s / (float)cnt;
    total_conv += cnt;

    uint32_t mid = acc_start + (acc_end - acc_start) / 2;
    if (n == 0) first_mid = mid;
    last_mid = mid;
    n++;
  }

  g_avg_depth = n ? (uint16_t)(total_conv / n) : 0;
  dt_s = (n > 1) ? (float)(last_mid - first_mid) * 1.0e-6f / (float)(n - 1) : 0.0f;
  return n;
}

#endif

// ═══ AC analysis ══════════════════════════════════════════════════════════════

// PowerResult and IntegrandFn are defined in ac_types.h — see the note there.

// Analysis context at file scope, so the integrand helpers below stay cheap
// enough to call from inside the integration loop.
static float  A_LSB_V, A_LSB_I, A_SCL_V, A_SCL_I, A_OFF_V, A_OFF_I;
static double A_VM, A_IM;

static inline double vAt(uint16_t k) {
  return ((double)g_vbuf[k] * A_LSB_V - A_OFF_V) * A_SCL_V;
}
static inline double iAt(uint16_t k) {
  return ((double)g_ibuf[k] * A_LSB_I - A_OFF_I) * A_SCL_I;
}

// Raw channel values, for finding the DC content of the window.
static void integrandDC(uint16_t k, double* o) {
  o[0] = vAt(k);
  o[1] = iAt(k);
}

// Cubic Lagrange weights for nodes at -1, 0, +1, +2, evaluated at alpha. This
// is what places the voltage at the instant the current sample was taken.
// Straight linear interpolation gets the phase right but shaves the amplitude:
// the chord across a sine cuts inside the arc by cos(pi*f*dt), which at ~10
// samples per 60 Hz cycle is 4.9 % — the whole error budget of a power monitor,
// spent on an avoidable approximation. A cubic through four points cuts that to
// 0.35 %, and at alpha = 0.5 its symmetry makes the phase exact.
static void lagrange4(double a, double* w) {
  w[0] = -a * (a - 1.0) * (a - 2.0) / 6.0;
  w[1] =  (a + 1.0) * (a - 1.0) * (a - 2.0) / 2.0;
  w[2] = -(a + 1.0) * a * (a - 2.0) / 2.0;
  w[3] =  (a + 1.0) * a * (a - 1.0) / 6.0;
}

// Magnitude response of that interpolator at theta radians per sample. The
// residual 0.35 % lands entirely on real power, so a resistive load would read
// PF 0.9965 and invite someone to "fix" it by mistuning PHASECAL. The response
// is a known function of the measured frequency, so it gets divided back out.
// Correcting at the fundamental alone is the right answer rather than a
// shortcut: mains voltage is near-sinusoidal, so P = sum(Vh*Ih*cos(phi_h)) only
// has a term where V has content, and the interpolator only touches V.
static float interpGain(const double* w, double theta) {
  double c1 = cos(theta), s1 = sin(theta);
  double c2 = cos(2.0 * theta), s2 = sin(2.0 * theta);
  double re = w[0] * c1 + w[1] + w[2] * c1 + w[3] * c2;
  double im = -w[0] * s1 + w[2] * s1 + w[3] * s2;
  return (float)sqrt(re * re + im * im);
}

static double A_W[4];    // interpolation weights for the current phasecal

// v-squared, i-squared, and v*i with the voltage interpolated to the instant
// the current sample was actually taken. Reads k-1 through k+2, so callers must
// keep the integration bounds inside [1, N-3].
static void integrandPower(uint16_t k, double* o) {
  double v = vAt(k) - A_VM;
  double i = iAt(k) - A_IM;
  double vi = A_W[0] * (vAt(k - 1) - A_VM) + A_W[1] * v
            + A_W[2] * (vAt(k + 1) - A_VM) + A_W[3] * (vAt(k + 2) - A_VM);
  o[0] = v * v;
  o[1] = i * i;
  o[2] = vi * i;
}

// Mean of each integrand over the continuous index range [a, b]: trapezoid rule
// on the sample grid, with the two partial end segments taken back out. The
// endpoints are fractional because they are interpolated zero crossings, and at
// ~10 samples per cycle, rounding them to whole samples would put roughly a
// percent of ripple on every reading.
static void integrateMean(IntegrandFn fn, uint8_t m, double a, double b, double* out) {
  uint16_t ka = (uint16_t)floor(a), kb = (uint16_t)floor(b);
  double fa = a - (double)ka, fb = b - (double)kb;

  double g[3], gn[3];
  for (uint8_t j = 0; j < m; j++) out[j] = 0.0;

  fn(ka, g);
  for (uint16_t k = ka; k <= kb; k++) {
    fn(k + 1, gn);
    for (uint8_t j = 0; j < m; j++) {
      out[j] += 0.5 * (g[j] + gn[j]);
      g[j] = gn[j];
    }
  }

  double gka[3], gka1[3], gkb[3], gkb1[3];
  fn(ka, gka); fn(ka + 1, gka1); fn(kb, gkb); fn(kb + 1, gkb1);
  double span = b - a;
  for (uint8_t j = 0; j < m; j++) {
    double at_a = gka[j] + fa * (gka1[j] - gka[j]);
    double at_b = gkb[j] + fb * (gkb1[j] - gkb[j]);
    out[j] -= 0.5 * fa * (gka[j] + at_a);            // strip [ka, a]
    out[j] -= 0.5 * (1.0 - fb) * (at_b + gkb1[j]);   // strip [b, kb+1]
    out[j] /= span;
  }
}

bool analyzePower(uint16_t n, float dt_s, PowerResult &r) {
  r.flags  = 0;
  r.n      = n;
  r.dt_s   = dt_s;
  r.span_s = 0.0f;
  if (n < 12 || dt_s <= 0.0f) return false;

  A_LSB_V = lsbV();
  A_LSB_I = lsbI();
  A_SCL_V = g_v_fs / g_v_adc_fs;
  A_SCL_I = g_i_fs / g_i_adc_fs;
  A_OFF_V = g_v_off;
  A_OFF_I = g_i_off;
  A_VM = 0.0;
  A_IM = 0.0;
  // clipping is flagged during acquisition, where the individual conversions
  // are still visible — on a pseudo-differential pair a difference of zero
  // could equally be two rails

  lagrange4((double)g_phasecal, A_W);

  // Index budget: integrandPower at k reads k+2, and integrateMean evaluates
  // one sample past its upper bound, so the deepest read is b+3. It also reads
  // k-1, so the lower bound may not fall below 1.
  const uint16_t last = n - 4;

  // a rough mean first, only so the crossing detector has a baseline
  double rough = 0.0;
  for (uint16_t k = 1; k <= last; k++) rough += vAt(k);
  rough /= (double)last;

  // Rising zero crossings of the voltage, located between samples. The scan
  // starts at k = 2 so the earliest crossing it can report is index 1.
  double first_x = 0.0, last_x = 0.0;
  uint16_t ncross = 0;
  double prev = vAt(1) - rough;
  for (uint16_t k = 2; k <= last; k++) {
    double cur = vAt(k) - rough;
    if (prev <= 0.0 && cur > 0.0) {
      double frac = (cur != prev) ? (-prev / (cur - prev)) : 0.0;
      double x = (double)(k - 1) + frac;
      if (ncross == 0) first_x = x;
      last_x = x;
      ncross++;
    }
    prev = cur;
  }

  double a, b;
  uint16_t cycles_used = 0;
  if (ncross >= 2 && (last_x - first_x) >= 1.0) {
    a = first_x;
    b = last_x;
    cycles_used = ncross - 1;
  } else {
    // no usable crossings — DC, no mains present, or a window shorter than one
    // cycle. Fall back to the whole buffer and say so in the flags.
    a = 1.0;
    b = (double)last;
    r.flags |= FLG_NOCROSS;
  }
  if (b > (double)last) b = (double)last;
  if (b - a < 1.0) return false;

  if (g_dcrem) {
    double dc[3];
    integrateMean(integrandDC, 2, a, b, dc);
    A_VM = dc[0];
    A_IM = dc[1];
  }

  double acc[3];
  integrateMean(integrandPower, 3, a, b, acc);

  r.span_s = (float)(b - a) * dt_s;
  r.hz = (cycles_used > 0 && r.span_s > 0.0f)
       ? (float)cycles_used / r.span_s : 0.0f;

  // undo the interpolator's amplitude loss at the measured frequency
  float gain = 1.0f;
  if (r.hz > 0.0f) {
    gain = interpGain(A_W, 2.0 * M_PI * (double)r.hz * (double)dt_s);
    if (gain < 0.5f) gain = 1.0f;    // nonsense frequency: do not amplify noise
  }

  r.vrms = (acc[0] > 0.0) ? (float)sqrt(acc[0]) : 0.0f;
  r.irms = (acc[1] > 0.0) ? (float)sqrt(acc[1]) : 0.0f;
  r.p    = (float)(acc[2]) / gain;
  r.s    = r.vrms * r.irms;
  double qq = (double)r.s * r.s - (double)r.p * r.p;
  r.q  = (qq > 0.0) ? (float)sqrt(qq) : 0.0f;
  r.pf = (r.s > 1.0e-6f) ? (r.p / r.s) : 0.0f;

  float vpk = 0.0f, ipk = 0.0f;
  for (uint16_t k = (uint16_t)a; k <= (uint16_t)b; k++) {
    float dv = (float)fabs(vAt(k) - A_VM);
    float di = (float)fabs(iAt(k) - A_IM);
    if (dv > vpk) vpk = dv;
    if (di > ipk) ipk = di;
  }
  r.vpk = vpk;
  r.ipk = ipk;
  return true;
}

// ═══ measurement window ═══════════════════════════════════════════════════════

void serviceMeasurement() {
  // ask for two extra cycles so there is always a crossing pair to bracket
  float want_s = ((float)g_cycles + 2.0f) / g_mains_hz;
  uint32_t dur_us = (uint32_t)(want_s * 1.0e6f);

  uint8_t acq_flags = 0;
  float dt_s = 0.0f;
  uint16_t n = acquirePairs(targetPairs(), dur_us, true, dt_s, acq_flags);

  if (g_abort) {          // a command is waiting — drop this window and go read it
    g_abort = false;
    return;
  }

  PowerResult r;
  if (!analyzePower(n, dt_s, r)) {
    Serial.println(F("$ERR,Window too short to analyse"));
    return;
  }
  r.flags |= acq_flags;

  if (r.span_s > 0.0f) g_wh += (double)r.p * (double)r.span_s / 3600.0;

  if (g_trip_a > 0.0f && r.irms > g_trip_a && !g_trip_latch) {
    g_trip_latch = true;
    setRelay(false);
    Serial.print(F("$TRIP,"));
    Serial.print(r.irms, 4); Serial.print(',');
    Serial.println(g_trip_a, 4);
  }
  if (g_trip_latch) r.flags |= FLG_TRIP;

  Serial.print(F("$PWR,"));
  Serial.print(r.vrms, 4);                           Serial.print(',');
  Serial.print(r.irms, 5);                           Serial.print(',');
  Serial.print(r.p, 4);                              Serial.print(',');
  Serial.print(r.s, 4);                              Serial.print(',');
  Serial.print(r.q, 4);                              Serial.print(',');
  Serial.print(r.pf, 4);                             Serial.print(',');
  Serial.print(r.hz, 3);                             Serial.print(',');
  Serial.print(r.vpk, 4);                            Serial.print(',');
  Serial.print(r.ipk, 5);                            Serial.print(',');
  Serial.print((float)g_wh, 6);                      Serial.print(',');
  Serial.print(millis() / 1000.0f, 2);               Serial.print(',');
  Serial.print(r.n);                                 Serial.print(',');
  Serial.print(dt_s > 0.0f ? 1.0f / dt_s : 0.0f, 1); Serial.print(',');
  Serial.print(g_relay ? 1 : 0);                     Serial.print(',');
  Serial.println(r.flags);
}

// ═══ waveform capture ═════════════════════════════════════════════════════════

// A stored sample is whole counts on the ADS122C04 but the mean of a bucket of
// conversions on the RA4M1, so the fraction has to survive the wire.
static inline void printSample(sample_t v) {
#if ADC_BACKEND == BACKEND_ADS122C04
  Serial.print(v);
#else
  Serial.print(v, 3);
#endif
}

void captureWave() {
  g_wave_pending = false;

  float len = g_wave_len_s;
  if (len < 0.01f) len = 0.01f;
  if (len > 5.0f)  len = 5.0f;
  uint32_t dur_us = (uint32_t)(len * 1.0e6f);

  uint8_t flags = 0;
  float dt_s = 0.0f;
  uint16_t n;
  if (g_wave_ch == 2) n = acquirePairs(BUF_MAX, dur_us, false, dt_s, flags);
  else                n = acquireSingle(g_wave_ch, BUF_MAX, dur_us, dt_s, flags);

  if (n == 0) {
    Serial.println(F("$ERR,Waveform captured 0 samples"));
    applyConfig();
    printConfig();
    return;
  }

  Serial.print(F("$WAVE,"));
  Serial.print(n);                        Serial.print(',');
  Serial.print(g_wave_ch);                Serial.print(',');
  Serial.print(dt_s * 1.0e6f, 3);         Serial.print(',');
  Serial.print(lsbV(), 12);     Serial.print(',');
  Serial.print(lsbI(), 12);     Serial.print(',');
  Serial.println(flags);

  for (uint16_t k = 0; k < n; k++) {
    Serial.print(F("$WD,"));
    printSample(g_vbuf[k]);
    if (g_wave_ch == 2) { Serial.print(','); printSample(g_ibuf[k]); }
    Serial.println();
  }
  Serial.println(F("$WEND"));

  applyConfig();     // continuous mode / mux may have moved during the capture
  printConfig();
}

// ═══ zeroing ══════════════════════════════════════════════════════════════════

// Average both channels for ~100 ms and keep the result as the offsets. Run it
// with the front end powered but no mains and no load: whatever is present at
// that moment is what gets subtracted from every later reading.
void zeroChannels() {
  uint8_t flags = 0;
  float dt_s = 0.0f;
  uint16_t n = acquirePairs(BUF_MAX, 100000UL, false, dt_s, flags);
  if (n < 4) { Serial.println(F("$ERR,Zero failed")); return; }

  double sv = 0.0, si = 0.0;
  float lv = lsbV(), li = lsbI();
  for (uint16_t k = 0; k < n; k++) {
    sv += (double)g_vbuf[k] * lv;
    si += (double)g_ibuf[k] * li;
  }
  g_v_off = (float)(sv / n);
  g_i_off = (float)(si / n);
  printAcConfig();
}

// ═══ command parser ═══════════════════════════════════════════════════════════

char    g_rxbuf[64];
uint8_t g_rxpos = 0;

// Second comma-separated float of "!CMD,a,b", or NAN when there is not one.
static float secondArg(const char* args) {
  const char* c = strchr(args, ',');
  return c ? atof(c + 1) : NAN;
}

#if ADC_BACKEND == BACKEND_RA4M1
// An index into ANALOG_PINS; the negative side may also be -1 for "none".
static bool validPin(int v, bool allow_none) {
  if (allow_none && v == -1) return true;
  return v >= 0 && v < ANALOG_PIN_COUNT;
}
#endif

void handleCommand(const char* cmd) {
  if (strcmp(cmd, "!START") == 0) {
    g_streaming = true; applyConfig(); printConfig();

  } else if (strcmp(cmd, "!STOP") == 0) {
    g_streaming = false; printConfig();

  } else if (strcmp(cmd, "!CFG") == 0) {
    printConfig(); printAcConfig();

  } else if (strncmp(cmd, "!RELAY,", 7) == 0) {
    bool on = (atoi(cmd + 7) != 0);
    if (on) g_trip_latch = false;      // an explicit close clears the trip
    setRelay(on);

  } else if (strncmp(cmd, "!RLYINV,", 8) == 0) {
    g_rly_inv = (atoi(cmd + 8) != 0);
    setRelay(g_relay);                 // re-drive the pin in the new polarity
    printAcConfig();

  } else if (strncmp(cmd, "!VSCALE,", 8) == 0) {
    float adc = atof(cmd + 8), real = secondArg(cmd + 8);
    if (adc > 0.0f && !isnan(real) && real != 0.0f) {
      g_v_adc_fs = adc; g_v_fs = real; printAcConfig();
    } else Serial.println(F("$ERR,VSCALE needs <adcVolts>,<realVolts>"));

  } else if (strncmp(cmd, "!ISCALE,", 8) == 0) {
    float adc = atof(cmd + 8), real = secondArg(cmd + 8);
    if (adc > 0.0f && !isnan(real) && real != 0.0f) {
      g_i_adc_fs = adc; g_i_fs = real; printAcConfig();
    } else Serial.println(F("$ERR,ISCALE needs <adcVolts>,<realAmps>"));

  } else if (strncmp(cmd, "!PHASECAL,", 10) == 0) {
    float v = atof(cmd + 10);
    if (v >= -1.0f && v <= 2.0f) { g_phasecal = v; printAcConfig(); }
    else Serial.println(F("$ERR,PHASECAL must be -1..2"));

  } else if (strncmp(cmd, "!CYCLES,", 8) == 0) {
    int v = atoi(cmd + 8);
    if (v >= 1 && v <= 100) { g_cycles = (uint8_t)v; printAcConfig(); }
    else Serial.println(F("$ERR,CYCLES must be 1-100"));

  } else if (strncmp(cmd, "!MAINS,", 7) == 0) {
    float v = atof(cmd + 7);
    if (v >= 10.0f && v <= 400.0f) { g_mains_hz = v; printAcConfig(); }
    else Serial.println(F("$ERR,MAINS must be 10-400 Hz"));

  } else if (strncmp(cmd, "!TRIP,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v >= 0.0f) { g_trip_a = v; g_trip_latch = false; printAcConfig(); }
    else Serial.println(F("$ERR,TRIP must be >= 0"));

  } else if (strncmp(cmd, "!DCREM,", 7) == 0) {
    g_dcrem = (atoi(cmd + 7) != 0); printAcConfig();

  } else if (strcmp(cmd, "!ZERO") == 0) {
    zeroChannels();

  } else if (strncmp(cmd, "!VOFF,", 6) == 0) {
    g_v_off = atof(cmd + 6) * 1.0e-6f; printAcConfig();

  } else if (strncmp(cmd, "!IOFF,", 6) == 0) {
    g_i_off = atof(cmd + 6) * 1.0e-6f; printAcConfig();

  } else if (strcmp(cmd, "!EZERO") == 0) {
    g_wh = 0.0;

  } else if (strncmp(cmd, "!VREF,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v > 0.1f && v < 6.0f) { g_vref = v; printAcConfig(); }
    else Serial.println(F("$ERR,VREF must be 0.1-6 V"));

#if ADC_BACKEND == BACKEND_ADS122C04

  } else if (strncmp(cmd, "!VGAIN,", 7) == 0) {
    int v = atoi(cmd + 7);
    if (v >= 0 && v < GAIN_COUNT) { g_v_gain = (uint8_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,VGAIN out of range"));

  } else if (strncmp(cmd, "!IGAIN,", 7) == 0) {
    int v = atoi(cmd + 7);
    if (v >= 0 && v < GAIN_COUNT) { g_i_gain = (uint8_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,IGAIN out of range"));

  } else if (strncmp(cmd, "!PGA,", 5) == 0) {
    g_pga = (atoi(cmd + 5) != 0); applyConfig(); printConfig();

  } else if (strncmp(cmd, "!RATE,", 6) == 0) {
    int v = atoi(cmd + 6);
    if (v >= 0 && v < RATE_COUNT) { g_rate_idx = (uint8_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,RATE out of range"));

  } else if (strncmp(cmd, "!TURBO,", 7) == 0) {
    g_turbo = (atoi(cmd + 7) != 0); applyConfig(); printConfig();

  } else if (strncmp(cmd, "!I2C,", 5) == 0) {
    long v = atol(cmd + 5);
    if (v >= 100000L && v <= 1000000L) { g_i2c_hz = (uint32_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,I2C must be 100000-1000000"));

#else

  } else if (strncmp(cmd, "!BITS,", 6) == 0) {
    int v = atoi(cmd + 6);
    if (v == 8 || v == 10 || v == 12 || v == 14) {
      g_bits = (uint8_t)v; applyConfig(); printConfig();
    } else Serial.println(F("$ERR,BITS must be 8, 10, 12 or 14"));

  } else if (strncmp(cmd, "!SPC,", 5) == 0) {
    int v = atoi(cmd + 5);
    if (v >= 8 && v <= 256) { g_spc = (uint16_t)v; printConfig(); }
    else Serial.println(F("$ERR,SPC must be 8-256"));

  } else if (strncmp(cmd, "!VPIN,", 6) == 0) {
    int p = atoi(cmd + 6);
    float q = secondArg(cmd + 6);
    if (validPin(p, false) && !isnan(q) && validPin((int)q, true)) {
      g_vp = (int8_t)p; g_vn = (int8_t)q; printConfig();
    } else Serial.println(F("$ERR,VPIN needs <plus>,<minus>; minus may be -1"));

  } else if (strncmp(cmd, "!IPIN,", 6) == 0) {
    int p = atoi(cmd + 6);
    float q = secondArg(cmd + 6);
    if (validPin(p, false) && !isnan(q) && validPin((int)q, true)) {
      g_ip = (int8_t)p; g_in = (int8_t)q; printConfig();
    } else Serial.println(F("$ERR,IPIN needs <plus>,<minus>; minus may be -1"));

  } else if (strncmp(cmd, "!AREF,", 6) == 0) {
    int v = atoi(cmd + 6);
    if (v >= 0 && v < AREF_COUNT) { g_aref = (uint8_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,AREF out of range"));

#endif

  } else if (strcmp(cmd, "!WAVE") == 0) {
    g_wave_pending = true;

  } else if (strncmp(cmd, "!WLEN,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v > 0.0f) { g_wave_len_s = v; printAcConfig(); }
    else Serial.println(F("$ERR,WLEN must be > 0"));

  } else if (strncmp(cmd, "!WCH,", 5) == 0) {
    int v = atoi(cmd + 5);
    if (v >= 0 && v <= 2) { g_wave_ch = (uint8_t)v; printAcConfig(); }
    else Serial.println(F("$ERR,WCH must be 0, 1 or 2"));

  } else {
    Serial.print(F("$ERR,Unknown command: "));
    Serial.println(cmd);
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_rxpos > 0) {
        g_rxbuf[g_rxpos] = '\0';
        handleCommand(g_rxbuf);
        g_rxpos = 0;
      }
    } else if (g_rxpos < sizeof(g_rxbuf) - 1) {
      g_rxbuf[g_rxpos++] = c;
    }
  }
}

// ═══ setup / loop ═════════════════════════════════════════════════════════════

void setup() {
  // the relay is driven open before anything else has a chance to go wrong
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, g_rly_inv ? HIGH : LOW);

  Serial.begin(115200);
  while (!Serial) delay(10);

#if ADC_BACKEND == BACKEND_ADS122C04
  Wire.begin();
  Wire.setClock(g_i2c_hz);

  // A0/A1 strapping moves the address around inside 0x40-0x4F
  bool found = false;
  for (uint8_t a = 0x40; a <= 0x4F && !found; a++) {
    if (ads.begin(a)) { g_addr = a; found = true; }
  }
  if (!found) {
    Serial.println(F("$ERR,ADS122C04 not found"));
    while (1) delay(100);
  }

  applyConfig();
  Serial.print(F("$READY,ADS122C04,0x"));
  Serial.println(g_addr, HEX);
#else
  applyConfig();
  Serial.print(F("$READY,RA4M1,"));
  Serial.print(g_bits);
  Serial.println(F("bit"));
#endif

  printConfig();
  printAcConfig();
}

void loop() {
  readSerial();

  if (g_wave_pending) { captureWave(); return; }
  if (!g_streaming) return;

  serviceMeasurement();
}
