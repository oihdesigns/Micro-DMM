#include <Arduino_AdvancedAnalog.h>

// AdvancedADC requires the channel list at construction time, so all 8 pins are
// always sampled.  Only the pins requested in each command are reported.
// If your board can't scan all 8 simultaneously, reduce N_CHANNELS and the
// constructor arguments (e.g. A0,A1,A2,A3 with N_CHANNELS = 4).
static const int N_CHANNELS = 8;
AdvancedADC adc(A0, A1, A2, A3, A4, A5, A6, A7); // fixed at compile time — library requirement

static const char* CH_NAMES[N_CHANNELS] = {"A0","A1","A2","A3","A4","A5","A6","A7"};
static const int   MAX_LOG   = 1000;   // log points per channel (fits easily in Giga RAM)
static const int   N_SAMPLES   = 512;  // samples per DMA buffer per channel
static const int   QUEUE_DEPTH = 8;

static int logData[N_CHANNELS][MAX_LOG];
static bool adcRunning = false;

static uint32_t toResolution(int bits) {
    if (bits <= 8)  return AN_RESOLUTION_8;
    if (bits <= 10) return AN_RESOLUTION_10;
    if (bits <= 12) return AN_RESOLUTION_12;
    if (bits <= 14) return AN_RESOLUTION_14;
    return AN_RESOLUTION_16;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
}

void loop() {
    if (!Serial.available()) return;

    // Expected Format: PINS:A0,A1|BITS:12|TIME:1000|SMOOTH:7|LOG:1000|RATE:500000
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int bitRes      = cmd.substring(cmd.indexOf("BITS:")   + 5, cmd.indexOf("|TIME:")).toInt();
    int duration    = cmd.substring(cmd.indexOf("TIME:")   + 5, cmd.indexOf("|SMOOTH:")).toInt();
    int smoothCount = cmd.substring(cmd.indexOf("SMOOTH:") + 7, cmd.indexOf("|LOG:")).toInt();
    int logTarget   = cmd.substring(cmd.indexOf("LOG:")    + 4).toInt();
    int rateIdx     = cmd.indexOf("|RATE:");
    uint32_t sampleRate = (rateIdx >= 0) ? (uint32_t)cmd.substring(rateIdx + 6).toInt() : 500000;

    if (bitRes < 8)          bitRes      = 12;
    if (duration < 1)        duration    = 1000;
    if (smoothCount < 1)     smoothCount = 1;
    if (logTarget   < 1)     logTarget   = 1;
    if (logTarget > MAX_LOG) logTarget   = MAX_LOG;
    if (sampleRate  < 1000)  sampleRate  = 500000;

    // Parse which channels the host wants reported
    bool pinReq[N_CHANNELS] = {};
    String pinsStr = cmd.substring(5, cmd.indexOf("|BITS:"));
    int ci = 0;
    while (ci >= 0) {
        int nc = pinsStr.indexOf(',', ci);
        String nm = (nc < 0) ? pinsStr.substring(ci) : pinsStr.substring(ci, nc);
        nm.trim();
        for (int i = 0; i < N_CHANNELS; i++)
            if (nm.equals(CH_NAMES[i])) { pinReq[i] = true; break; }
        if (nc < 0) break;
        ci = nc + 1;
    }

    // Per-channel accumulators
    uint32_t cnt[N_CHANNELS]  = {};
    double   sum[N_CHANNELS]  = {};
    double   sq[N_CHANNELS]   = {};
    int      mn[N_CHANNELS], mx[N_CHANNELS];
    for (int i = 0; i < N_CHANNELS; i++) { mn[i] = 65535; mx[i] = 0; }
    int logCnt[N_CHANNELS] = {};

    // Smoothing accumulators (averaging smoothCount raw DMA samples → 1 filtered value)
    long smoothBuf[N_CHANNELS] = {};
    int  smoothAcc[N_CHANNELS] = {};

    // Spread logTarget points evenly across the run
    uint32_t totalSmoothed = (sampleRate * (uint32_t)duration / 1000) / (uint32_t)smoothCount;
    uint32_t logEvery = (totalSmoothed > (uint32_t)logTarget) ? totalSmoothed / logTarget : 1;

    if (adcRunning) adc.stop();
    if (!adc.begin(toResolution(bitRes), sampleRate, N_SAMPLES, QUEUE_DEPTH)) {
        Serial.println("ERR:ADC_BEGIN_FAILED");
        return;
    }
    adcRunning = true;

    uint32_t startMs = millis();
    while (millis() - startMs < (uint32_t)duration) {
        if (!adc.available()) continue;

        SampleBuffer buf = adc.read();
        // buf is interleaved: [ch0_t0, ch1_t0, ..., ch7_t0, ch0_t1, ch1_t1, ...]
        int ns = buf.size() / N_CHANNELS;

        for (int s = 0; s < ns; s++) {
            for (int ch = 0; ch < N_CHANNELS; ch++) {
                smoothBuf[ch] += buf[s * N_CHANNELS + ch];
                if (++smoothAcc[ch] >= smoothCount) {
                    int val = smoothBuf[ch] / smoothCount;
                    smoothBuf[ch] = 0;
                    smoothAcc[ch] = 0;

                    if (pinReq[ch]) {
                        if (val < mn[ch]) mn[ch] = val;
                        if (val > mx[ch]) mx[ch] = val;
                        sum[ch] += val;
                        sq[ch]  += (double)val * val;
                        uint32_t n = ++cnt[ch];
                        if (logCnt[ch] < logTarget && n % logEvery == 0)
                            logData[ch][logCnt[ch]++] = val;
                    }
                }
            }
        }
        buf.release();
    }

    adc.stop();
    adcRunning = false;

    // 1. Output stats — same format as before so Python parser is unchanged
    for (int i = 0; i < N_CHANNELS; i++) {
        if (!pinReq[i] || cnt[i] == 0) continue;
        double mean   = sum[i] / cnt[i];
        double rms    = sqrt(sq[i] / cnt[i]);
        double stdDev = sqrt(fabs(sq[i] / cnt[i] - mean * mean));
        Serial.print("PIN:");    Serial.print(CH_NAMES[i]);
        Serial.print("|BITS:");  Serial.print(bitRes);
        Serial.print("|COUNT:"); Serial.print(cnt[i]);
        Serial.print("|MIN:");   Serial.print(mn[i]);
        Serial.print("|MAX:");   Serial.print(mx[i]);
        Serial.print("|MEAN:");  Serial.print(mean, 4);
        Serial.print("|RMS:");   Serial.print(rms, 4);
        Serial.print("|STD:");   Serial.println(stdDev, 4);
    }

    // 2. Output log data for plotting
    for (int i = 0; i < N_CHANNELS; i++) {
        if (!pinReq[i] || logCnt[i] == 0) continue;
        Serial.print("DATA:"); Serial.print(CH_NAMES[i]); Serial.print("|VALS:");
        for (int j = 0; j < logCnt[i]; j++) {
            Serial.print(logData[i][j]);
            if (j < logCnt[i] - 1) Serial.print(',');
        }
        Serial.println();
    }

    Serial.println("DONE");
}
