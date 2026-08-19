/*!
 * ADS122C04 Interactive Test Sketch
 * Serial command interface + structured data output for Python GUI
 *
 * Commands (send from PC, newline terminated):
 *   !MUX,<0-14>     set input mux
 *   !GAIN,<0-7>     set gain index (0=1x ... 7=128x)
 *   !RATE,<0-6>     set data rate index
 *   !PGA,<0|1>      disable/enable PGA
 *   !TURBO,<0|1>    disable/enable turbo mode
 *   !TEMP,<0|1>     disable/enable temperature sensor
 *   !START          begin streaming
 *   !STOP           stop streaming
 *   !CFG            print current config
 *   !BURST          trigger burst capture (uses current duration)
 *   !BURST,<sec>    set duration and trigger (float, e.g. 0.5 or 2.0)
 *   !BLEN,<sec>     set burst duration without triggering
 *
 * Type-K wiring notes:
 *   Thermocouple across AIN0 (+, chromel) and AIN1 (−, alumel); TC mode selects
 *   that differential pair at 32× with the PGA in circuit, giving ±64 mV of
 *   range off the 2.048 V reference — the entire type-K span (−6 … +55 mV).
 *   A floating thermocouple has no defined common-mode voltage and will read
 *   nonsense until it is biased into range: the usual fix is a high-value
 *   divider (e.g. two 1 MΩ resistors from AVDD and AVSS meeting at AIN1), plus
 *   a differential RC filter (≈1 kΩ in each leg, 0.1 µF across) at the inputs.
 *   The cold junction is the ADC's own die sensor, which sits warmer than the
 *   terminal block it is standing in for — !TCCJTRIM exists for exactly that,
 *   and !TCCJSRC/!TCCJVAL let an external reference take over entirely.
 *
 * Type-K thermocouple mode:
 *   !TC,<0|1>       leave/enter thermocouple mode (applies TC-friendly config)
 *   !TCCJSRC,<0|1>  cold junction from the internal temp sensor (0) or fixed (1)
 *   !TCCJVAL,<degC> fixed cold-junction temperature, used when source = 1
 *   !TCCJTRIM,<deg> offset added to the internal sensor (die runs warm)
 *   !TCCJMS,<ms>    how often to re-read the cold junction (default 2000)
 *   !TCZERO         use the present TC voltage as zero (short the inputs first)
 *   !TCOFF,<uV>     set that zero offset directly; 0 clears it
 *   !BURNOUT,<0|1>  burn-out current sources — an open TC then reads full scale
 *
 * Data output (CSV, one line per sample):
 *   $ADC,<raw_s32>,<voltage_f>,<mux_idx>,<gain_idx>,<pga>,<rate_idx>,<turbo>,<actual_sps>
 *   $TEMP,<deg_c>,<rate_idx>,<turbo>,<actual_sps>
 *   $TC,<raw_s32>,<tc_volts>,<cj_c>,<hot_c>,<gain_idx>,<rate_idx>,<turbo>,<actual_sps>,<flags>
 *        flags: +1 outside type-K range, +2 cold junction stale, +4 ADC saturated
 *   $CFG,<mux>,<gain>,<pga>,<rate>,<turbo>,<temp>
 *   $TCCFG,<tc_on>,<cj_src>,<cj_fixed_c>,<cj_trim_c>,<cj_ms>,<offset_uv>,<burnout>
 *   $ERR,<message>
 *   $BURST_START,<count>,<actual_sps>,<lsb_v>
 *   $BD,<raw_s32>                    (one per sample)
 *   $BURST_END,<mean_v>,<min_v>,<max_v>,<std_v>
 */

#include <Adafruit_ADS122C04.h>

Adafruit_ADS122C04 ads;

// ── burst buffer ──────────────────────────────────────────────────────────────
// 4096 × 4 B = 16 KB — fine for M4/RP2040/RP2350; reduce for Uno (max ~400)
#define BURST_MAX_SAMPLES 4096
static int32_t  g_burst_buf[BURST_MAX_SAMPLES];
static uint16_t g_burst_count    = 0;
static uint32_t g_burst_start_ms = 0;
static bool     g_burst_active   = false;
static float    g_burst_dur_s    = 1.0f;

// ── config state ──────────────────────────────────────────────────────────────
static uint8_t  g_mux_idx   = 8;    // AIN0 SE
static uint8_t  g_gain_idx  = 0;    // 1x
static uint8_t  g_rate_idx  = 0;    // 20 SPS
static bool     g_pga       = false;
static bool     g_turbo     = false;
static bool     g_temp      = false;
static bool     g_streaming = false;

// ── thermocouple state ────────────────────────────────────────────────────────
static bool     g_tc_mode     = false;
static bool     g_tc_cj_fixed = false;   // false = internal sensor, true = fixed
static float    g_tc_cj_val   = 25.0f;   // fixed cold-junction temperature, °C
static float    g_tc_cj_trim  = 0.0f;    // added to the internal sensor reading
static uint32_t g_tc_cj_ms    = 2000;    // cold-junction refresh interval
static float    g_tc_offset_v = 0.0f;    // subtracted from every TC reading
static bool     g_burnout     = false;

static float    g_tc_cj_c       = NAN;   // last cold-junction temperature
static uint32_t g_tc_cj_last_ms = 0;
static bool     g_tc_cj_valid   = false;

// settings to put back when thermocouple mode is switched off
static uint8_t  g_tc_saved_mux  = 8;
static uint8_t  g_tc_saved_gain = 0;
static bool     g_tc_saved_pga  = false;

// TC mode defaults: AIN0−AIN1 differential, 32× with the PGA in circuit
// (±64 mV FSR off the 2.048 V reference — the whole type-K span), 20 SPS.
#define TC_DEFAULT_MUX   0
#define TC_DEFAULT_GAIN  5
#define TC_DEFAULT_RATE  0

// ── SPS measurement (streaming mode) ─────────────────────────────────────────
static uint32_t g_sample_count = 0;
static uint32_t g_sps_last_ms  = 0;
static float    g_actual_sps   = 0.0f;

// ── lookup tables ─────────────────────────────────────────────────────────────
const ads122c04_mux_t MUX_TABLE[] = {
  ADS122C04_MUX_AIN0_AIN1, ADS122C04_MUX_AIN0_AIN2, ADS122C04_MUX_AIN0_AIN3,
  ADS122C04_MUX_AIN1_AIN0, ADS122C04_MUX_AIN1_AIN2, ADS122C04_MUX_AIN1_AIN3,
  ADS122C04_MUX_AIN2_AIN3, ADS122C04_MUX_AIN3_AIN2,
  ADS122C04_MUX_AIN0, ADS122C04_MUX_AIN1, ADS122C04_MUX_AIN2, ADS122C04_MUX_AIN3,
  ADS122C04_MUX_REFPN_4, ADS122C04_MUX_SUPPLY_4, ADS122C04_MUX_SHORTED
};
const uint8_t MUX_COUNT = sizeof(MUX_TABLE) / sizeof(MUX_TABLE[0]);

const ads122c04_gain_t GAIN_TABLE[] = {
  ADS122C04_GAIN_1,  ADS122C04_GAIN_2,  ADS122C04_GAIN_4,  ADS122C04_GAIN_8,
  ADS122C04_GAIN_16, ADS122C04_GAIN_32, ADS122C04_GAIN_64, ADS122C04_GAIN_128
};
const uint8_t GAIN_COUNT = sizeof(GAIN_TABLE) / sizeof(GAIN_TABLE[0]);

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

// ── type-K thermocouple math (NIST ITS-90) ────────────────────────────────────
// Forward polynomial (°C → mV) is needed for the cold junction; the inverse
// (mV → °C) turns the compensated voltage back into a temperature. Both are
// the published ITS-90 fits, so they agree with a type-K table to well under
// a tenth of a degree over the whole range.

static const double TK_NEG[] = {           // -270 … 0 °C
   0.0,
   0.394501280250E-01,  0.236223735980E-04, -0.328589067840E-06,
  -0.499048287770E-08, -0.675090591730E-10, -0.574103274280E-12,
  -0.310888728940E-14, -0.104516093650E-16, -0.198892668780E-19,
  -0.163226974860E-22
};
static const double TK_POS[] = {           // 0 … 1372 °C, plus the exp term
  -0.176004136860E-01,  0.389212049750E-01,  0.185587700320E-04,
  -0.994575928740E-07,  0.318409457190E-09, -0.560728448890E-12,
   0.560750590590E-15, -0.320207200030E-18,  0.971511471520E-22,
  -0.121047212750E-25
};
static const double TK_A0 = 0.118597600000E+00;
static const double TK_A1 = -0.118343200000E-03;
static const double TK_A2 = 0.126968600000E+03;

static const double TK_INV_NEG[] = {       // -5.891 … 0 mV  (-200 … 0 °C)
   0.0000000E+00,  2.5173462E+01, -1.1662878E+00, -1.0833638E+00,
  -8.9773540E-01, -3.7342377E-01, -8.6632643E-02, -1.0450598E-02,
  -5.1920577E-04
};
static const double TK_INV_MID[] = {       // 0 … 20.644 mV  (0 … 500 °C)
   0.000000E+00,  2.508355E+01,  7.860106E-02, -2.503131E-01,
   8.315270E-02, -1.228034E-02,  9.804036E-04, -4.413030E-05,
   1.057734E-06, -1.052755E-08
};
static const double TK_INV_HIGH[] = {      // 20.644 … 54.886 mV (500 … 1372 °C)
  -1.318058E+02,  4.830222E+01, -1.646031E+00,  5.464731E-02,
  -9.650715E-04,  8.802193E-06, -3.110810E-08
};

#define TK_MV_MIN  (-5.891)   // -200 °C — the bottom of the inverse fit
#define TK_MV_MAX  (54.886)   // 1372 °C

static double tkPoly(const double* c, uint8_t n, double x) {
  double out = 0.0, p = 1.0;
  for (uint8_t i = 0; i < n; i++) { out += c[i] * p; p *= x; }
  return out;
}

// °C → thermocouple EMF in mV, referenced to a 0 °C cold junction
double tkTempToMv(double t_c) {
  if (t_c < 0.0) return tkPoly(TK_NEG, sizeof(TK_NEG) / sizeof(double), t_c);
  return tkPoly(TK_POS, sizeof(TK_POS) / sizeof(double), t_c)
       + TK_A0 * exp(TK_A1 * (t_c - TK_A2) * (t_c - TK_A2));
}

// mV (referenced to 0 °C) → °C; NAN outside the type-K range
double tkMvToTemp(double mv) {
  if (isnan(mv) || mv < TK_MV_MIN || mv > TK_MV_MAX) return NAN;
  if (mv < 0.0)     return tkPoly(TK_INV_NEG,  sizeof(TK_INV_NEG)  / sizeof(double), mv);
  if (mv < 20.644)  return tkPoly(TK_INV_MID,  sizeof(TK_INV_MID)  / sizeof(double), mv);
  return tkPoly(TK_INV_HIGH, sizeof(TK_INV_HIGH) / sizeof(double), mv);
}

// ── config helpers ────────────────────────────────────────────────────────────
void applyConfig() {
  ads.setContinuousMode(false);
  ads.setMux(MUX_TABLE[g_mux_idx]);
  ads.setGain(GAIN_TABLE[g_gain_idx]);
  ads.enablePGA(g_pga);
  ads.setDataRate(RATE_TABLE[g_rate_idx]);
  ads.setTurboMode(g_turbo);
  ads.enableTempSensor(g_temp);
  ads.enableBurnOutCurrent(g_burnout);
  ads.setContinuousMode(true);
  ads.startSync();
  g_sample_count = 0;
  g_sps_last_ms  = millis();
  g_actual_sps   = 0.0f;
}

void printConfig() {
  Serial.print(F("$CFG,"));
  Serial.print(g_mux_idx);        Serial.print(',');
  Serial.print(g_gain_idx);       Serial.print(',');
  Serial.print(g_pga   ? 1 : 0);  Serial.print(',');
  Serial.print(g_rate_idx);       Serial.print(',');
  Serial.print(g_turbo ? 1 : 0);  Serial.print(',');
  Serial.println(g_temp ? 1 : 0);
  printTcConfig();
}

void printTcConfig() {
  Serial.print(F("$TCCFG,"));
  Serial.print(g_tc_mode     ? 1 : 0);   Serial.print(',');
  Serial.print(g_tc_cj_fixed ? 1 : 0);   Serial.print(',');
  Serial.print(g_tc_cj_val, 3);          Serial.print(',');
  Serial.print(g_tc_cj_trim, 3);         Serial.print(',');
  Serial.print(g_tc_cj_ms);              Serial.print(',');
  Serial.print(g_tc_offset_v * 1.0e6f, 3); Serial.print(',');
  Serial.println(g_burnout ? 1 : 0);
}

// ── thermocouple mode ─────────────────────────────────────────────────────────
uint16_t nominalSps() {
  return RATE_NOMINAL[g_rate_idx][g_turbo ? 1 : 0];
}

// Read the internal temperature sensor for cold-junction compensation.
// The sensor shares the conversion path, so this borrows the ADC for two
// conversions (the first after a mode change is discarded) and hands it back.
// At the 2 s default interval that costs well under a percent of throughput.
void readColdJunction() {
  g_tc_cj_last_ms = millis();
  if (g_tc_cj_fixed) {
    g_tc_cj_c     = g_tc_cj_val;
    g_tc_cj_valid = true;
    return;
  }

  uint32_t period_ms = 1000UL / nominalSps() + 1;
  uint32_t timeout   = 4 * period_ms + 25;

  ads.enableTempSensor(true);
  ads.startSync();

  float    temp_c  = NAN;
  uint8_t  wanted  = 2;            // discard one conversion, keep the second
  uint32_t started = millis();
  while (wanted > 0 && (millis() - started) < timeout) {
    if (ads.isDataReady()) {
      int32_t raw = ads.readData();
      if (raw != (int32_t)0xEE000000) {
        temp_c = ads.convertToTemperature(raw);
        wanted--;
        started = millis();
      }
    }
  }

  ads.enableTempSensor(false);
  ads.startSync();

  if (wanted == 0 && !isnan(temp_c)) {
    g_tc_cj_c     = temp_c + g_tc_cj_trim;
    g_tc_cj_valid = true;
  } else {
    g_tc_cj_valid = false;         // keep the last value, but flag it stale
  }
}

void serviceThermocouple() {
  if ((millis() - g_tc_cj_last_ms) >= g_tc_cj_ms) readColdJunction();
  if (!ads.isDataReady()) return;

  int32_t raw = ads.readData();
  if (raw == (int32_t)0xEE000000) { Serial.println(F("$ERR,Read failed")); return; }

  uint32_t now = millis();
  g_sample_count++;
  uint32_t elapsed = now - g_sps_last_ms;
  if (elapsed >= 1000) {
    g_actual_sps   = g_sample_count * 1000.0f / elapsed;
    g_sample_count = 0;
    g_sps_last_ms  = now;
  }

  float  volts   = ads.convertToVoltage(raw) - g_tc_offset_v;
  double cj_c    = g_tc_cj_valid ? (double)g_tc_cj_c : NAN;
  double mv_meas = (double)volts * 1000.0;
  // Cold-junction compensation: add back the EMF the cold junction is not
  // producing, so the sum is referenced to 0 °C the way the tables are.
  double mv_total = isnan(cj_c) ? mv_meas : mv_meas + tkTempToMv(cj_c);
  double hot_c    = tkMvToTemp(mv_total);

  uint8_t flags = 0;
  if (isnan(hot_c))                     flags |= 0x01;   // outside type-K range
  if (!g_tc_cj_valid)                   flags |= 0x02;   // cold junction stale
  if (abs(raw) > (int32_t)8200000)      flags |= 0x04;   // railed — open TC?

  Serial.print(F("$TC,"));
  Serial.print(raw);                       Serial.print(',');
  Serial.print(volts, 8);                  Serial.print(',');
  if (isnan(cj_c)) Serial.print(F("nan")); else Serial.print((float)cj_c, 4);
  Serial.print(',');
  if (isnan(hot_c)) Serial.print(F("nan")); else Serial.print((float)hot_c, 4);
  Serial.print(',');
  Serial.print(g_gain_idx);                Serial.print(',');
  Serial.print(g_rate_idx);                Serial.print(',');
  Serial.print(g_turbo ? 1 : 0);           Serial.print(',');
  Serial.print(g_actual_sps, 1);           Serial.print(',');
  Serial.println(flags);
}

void setThermocoupleMode(bool on) {
  if (on == g_tc_mode) return;
  if (on) {
    g_tc_saved_mux  = g_mux_idx;
    g_tc_saved_gain = g_gain_idx;
    g_tc_saved_pga  = g_pga;
    g_mux_idx  = TC_DEFAULT_MUX;
    g_gain_idx = TC_DEFAULT_GAIN;
    g_pga      = true;
    g_rate_idx = TC_DEFAULT_RATE;
    g_temp     = false;            // TC mode owns the temperature sensor
    g_tc_mode  = true;
    g_tc_cj_valid   = false;
    g_tc_cj_last_ms = millis() - g_tc_cj_ms;   // read the cold junction at once
  } else {
    g_tc_mode  = false;
    g_mux_idx  = g_tc_saved_mux;
    g_gain_idx = g_tc_saved_gain;
    g_pga      = g_tc_saved_pga;
  }
  applyConfig();
}

// ── burst capture (blocking — no UART during this phase) ──────────────────────
void captureBurst() {
  g_burst_count    = 0;
  g_burst_start_ms = millis();
  uint32_t end_ms  = g_burst_start_ms + (uint32_t)(g_burst_dur_s * 1000.0f);

  // ensure continuous mode is running
  ads.setContinuousMode(true);
  ads.startSync();

  while (millis() < end_ms && g_burst_count < BURST_MAX_SAMPLES) {
    if (ads.isDataReady()) {
      int32_t raw = ads.readData();
      if (raw != (int32_t)0xEE000000) {
        g_burst_buf[g_burst_count++] = raw;
      }
    }
  }

  g_burst_active = false;
  transmitBurst();
  applyConfig();   // always restore clean ADC state after burst
  printConfig();   // sync Python — $CFG not sent during burst
}

void transmitBurst() {
  if (g_burst_count == 0) {
    Serial.println(F("$ERR,Burst captured 0 samples"));
    return;
  }

  uint32_t elapsed_ms = millis() - g_burst_start_ms;
  float actual_sps = g_burst_count * 1000.0f / (float)elapsed_ms;
  float lsb_v = ads.convertToVoltage(1);   // volts per raw LSB

  // two-pass stats in voltage domain
  float sum = 0.0f, min_v = 3.4e38f, max_v = -3.4e38f;
  for (uint16_t i = 0; i < g_burst_count; i++) {
    float v = (float)g_burst_buf[i] * lsb_v;
    sum += v;
    if (v < min_v) min_v = v;
    if (v > max_v) max_v = v;
  }
  float mean_v = sum / g_burst_count;

  float sum_sq = 0.0f;
  for (uint16_t i = 0; i < g_burst_count; i++) {
    float d = (float)g_burst_buf[i] * lsb_v - mean_v;
    sum_sq += d * d;
  }
  float std_v = sqrt(sum_sq / g_burst_count);

  // header
  Serial.print(F("$BURST_START,"));
  Serial.print(g_burst_count);  Serial.print(',');
  Serial.print(actual_sps, 1);  Serial.print(',');
  Serial.println(lsb_v, 10);

  // samples
  for (uint16_t i = 0; i < g_burst_count; i++) {
    Serial.print(F("$BD,"));
    Serial.println(g_burst_buf[i]);
  }

  // footer stats
  Serial.print(F("$BURST_END,"));
  Serial.print(mean_v, 8); Serial.print(',');
  Serial.print(min_v,  8); Serial.print(',');
  Serial.print(max_v,  8); Serial.print(',');
  Serial.println(std_v, 8);
}

// ── command parser ────────────────────────────────────────────────────────────
char    g_rxbuf[32];
uint8_t g_rxpos = 0;

void handleCommand(const char* cmd) {
  if (strncmp(cmd, "!MUX,", 5) == 0) {
    int v = atoi(cmd + 5);
    if (v >= 0 && v < MUX_COUNT) { g_mux_idx  = (uint8_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,MUX out of range"));

  } else if (strncmp(cmd, "!GAIN,", 6) == 0) {
    int v = atoi(cmd + 6);
    if (v >= 0 && v < GAIN_COUNT) { g_gain_idx = (uint8_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,GAIN out of range"));

  } else if (strncmp(cmd, "!RATE,", 6) == 0) {
    int v = atoi(cmd + 6);
    if (v >= 0 && v < RATE_COUNT) { g_rate_idx = (uint8_t)v; applyConfig(); printConfig(); }
    else Serial.println(F("$ERR,RATE out of range"));

  } else if (strncmp(cmd, "!PGA,", 5) == 0) {
    g_pga = (atoi(cmd + 5) != 0); applyConfig(); printConfig();

  } else if (strncmp(cmd, "!TURBO,", 7) == 0) {
    g_turbo = (atoi(cmd + 7) != 0); applyConfig(); printConfig();

  } else if (strncmp(cmd, "!TEMP,", 6) == 0) {
    g_temp = (atoi(cmd + 6) != 0);
    if (g_temp && g_tc_mode) setThermocoupleMode(false);   // one owner at a time
    applyConfig(); printConfig();

  } else if (strncmp(cmd, "!TC,", 4) == 0) {
    setThermocoupleMode(atoi(cmd + 4) != 0); printConfig();

  } else if (strncmp(cmd, "!TCCJSRC,", 9) == 0) {
    g_tc_cj_fixed   = (atoi(cmd + 9) != 0);
    g_tc_cj_last_ms = millis() - g_tc_cj_ms;
    g_tc_cj_valid   = false;
    printTcConfig();

  } else if (strncmp(cmd, "!TCCJVAL,", 9) == 0) {
    g_tc_cj_val = atof(cmd + 9);
    if (g_tc_cj_fixed) { g_tc_cj_c = g_tc_cj_val; g_tc_cj_valid = true; }
    printTcConfig();

  } else if (strncmp(cmd, "!TCCJTRIM,", 10) == 0) {
    g_tc_cj_trim    = atof(cmd + 10);
    g_tc_cj_last_ms = millis() - g_tc_cj_ms;
    printTcConfig();

  } else if (strncmp(cmd, "!TCCJMS,", 8) == 0) {
    long v = atol(cmd + 8);
    if (v >= 50 && v <= 600000) { g_tc_cj_ms = (uint32_t)v; printTcConfig(); }
    else Serial.println(F("$ERR,TCCJMS must be 50-600000 ms"));

  } else if (strcmp(cmd, "!TCZERO") == 0) {
    // Whatever the inputs read right now becomes zero — short the TC input (or
    // hold the junction at the cold-junction temperature) before sending this.
    g_tc_offset_v += ads.convertToVoltage(ads.readData());
    printTcConfig();

  } else if (strncmp(cmd, "!TCOFF,", 7) == 0) {
    g_tc_offset_v = atof(cmd + 7) * 1.0e-6f;
    printTcConfig();

  } else if (strncmp(cmd, "!BURNOUT,", 9) == 0) {
    g_burnout = (atoi(cmd + 9) != 0); applyConfig(); printConfig();

  } else if (strcmp(cmd, "!START") == 0) {
    g_streaming = true; applyConfig(); printConfig();

  } else if (strcmp(cmd, "!STOP") == 0) {
    g_streaming = false;

  } else if (strcmp(cmd, "!CFG") == 0) {
    printConfig();

  } else if (strncmp(cmd, "!BURST", 6) == 0) {
    if (cmd[6] == ',') {
      float v = atof(cmd + 7);
      if (v > 0.0f) g_burst_dur_s = v;
    }
    g_burst_active = true;

  } else if (strncmp(cmd, "!BLEN,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v > 0.0f) g_burst_dur_s = v;

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

// ── setup / loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

   Wire.setClock(400000);

  if (!ads.begin()) {
    Serial.println(F("$ERR,ADS122C04 not found"));
    while (1) delay(100);
  }

  applyConfig();
  Serial.println(F("$READY"));
  printConfig();
}

void loop() {
  readSerial();

  // burst takes priority — blocks until complete, then returns
  if (g_burst_active) {
    captureBurst();
    return;
  }

  if (!g_streaming) return;

  if (g_tc_mode) { serviceThermocouple(); return; }

  if (!ads.isDataReady()) return;

  uint32_t now = millis();
  g_sample_count++;
  uint32_t elapsed = now - g_sps_last_ms;
  if (elapsed >= 1000) {
    g_actual_sps   = g_sample_count * 1000.0f / elapsed;
    g_sample_count = 0;
    g_sps_last_ms  = now;
  }

  if (g_temp) {
    float temp_c = ads.readTemperature();
    Serial.print(F("$TEMP,"));
    Serial.print(temp_c, 4);      Serial.print(',');
    Serial.print(g_rate_idx);     Serial.print(',');
    Serial.print(g_turbo ? 1:0);  Serial.print(',');
    Serial.println(g_actual_sps, 1);
  } else {
    int32_t raw = ads.readData();
    if (raw == (int32_t)0xEE000000) { Serial.println(F("$ERR,Read failed")); return; }
    float volts = ads.convertToVoltage(raw);

    Serial.print(F("$ADC,"));
    Serial.print(raw);            Serial.print(',');
    Serial.print(volts, 8);       Serial.print(',');
    Serial.print(g_mux_idx);      Serial.print(',');
    Serial.print(g_gain_idx);     Serial.print(',');
    Serial.print(g_pga ? 1 : 0);  Serial.print(',');
    Serial.print(g_rate_idx);     Serial.print(',');
    Serial.print(g_turbo ? 1:0);  Serial.print(',');
    Serial.println(g_actual_sps, 1);
  }
}
