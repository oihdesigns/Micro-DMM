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
 * Data output (CSV, one line per sample):
 *   $ADC,<raw_s32>,<voltage_f>,<mux_idx>,<gain_idx>,<pga>,<rate_idx>,<turbo>,<actual_sps>
 *   $TEMP,<deg_c>,<rate_idx>,<turbo>,<actual_sps>
 *   $CFG,<mux>,<gain>,<pga>,<rate>,<turbo>,<temp>
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

// ── config helpers ────────────────────────────────────────────────────────────
void applyConfig() {
  ads.setContinuousMode(false);
  ads.setMux(MUX_TABLE[g_mux_idx]);
  ads.setGain(GAIN_TABLE[g_gain_idx]);
  ads.enablePGA(g_pga);
  ads.setDataRate(RATE_TABLE[g_rate_idx]);
  ads.setTurboMode(g_turbo);
  ads.enableTempSensor(g_temp);
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
    g_temp = (atoi(cmd + 6) != 0); applyConfig(); printConfig();

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
