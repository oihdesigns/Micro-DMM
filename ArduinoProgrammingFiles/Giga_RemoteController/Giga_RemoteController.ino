#include <Arduino_AdvancedAnalog.h>

// AdvancedADC requires the channel list at construction time, so all 8 pins are
// always sampled.  Only the pins requested in each command are reported.
// If your board can't scan all 8 simultaneously, reduce N_CHANNELS and the
// constructor arguments (e.g. A0,A1,A2,A3 with N_CHANNELS = 4).
static const int N_CHANNELS = 8;
AdvancedADC adc(A0, A1, A2, A3, A4, A5, A6, A7); // fixed at compile time — library requirement

static const char* CH_NAMES[N_CHANNELS] = {"A0","A1","A2","A3","A4","A5","A6","A7"};
static const int   MAX_LOG   = 10000;  // log points per channel (~320 KB static for 8 ch)
static const int   N_SAMPLES_MAX = 512; // upper bound for DMA buffer (per channel)
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
    int trigIdx = cmd.indexOf("|TRIG:");
    int trigThresh[N_CHANNELS] = {};
    if (trigIdx >= 0) {
        String ts = cmd.substring(trigIdx + 6);
        for (int i = 0; i < N_CHANNELS; i++) {
            trigThresh[i] = ts.toInt();
            int c = ts.indexOf(',');
            if (c < 0) break;
            ts = ts.substring(c + 1);
        }
    }
    int midIdx = cmd.indexOf("|MID:");
    bool midpointMode = (midIdx >= 0 && cmd.substring(midIdx + 5).toInt() == 1);

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
    int  smoothTick = 0;
    int  lastLoggedVal[N_CHANNELS];
    for (int i = 0; i < N_CHANNELS; i++) lastLoggedVal[i] = -32768;
    bool usingTrig = false;
    for (int i = 0; i < N_CHANNELS; i++)
        if (pinReq[i] && trigThresh[i] > 0) { usingTrig = true; break; }

    // Spread logTarget points evenly across the run (used when trigger disabled)
    uint32_t totalSmoothed = (sampleRate * (uint32_t)duration / 1000) / (uint32_t)smoothCount;
    uint32_t logEvery  = (totalSmoothed > (uint32_t)logTarget) ? totalSmoothed / logTarget : 1;
    uint32_t sampleTick = 0;
    uint32_t lastLogMs  = 0;  // ms from start when the most recent log point was stored

    // Midpoint-trigger ring-buffer state
    int  midPreSize   = logTarget / 2;
    int  midPostSize  = logTarget - midPreSize;
    int  midRingHead  = 0;   // next write slot in ring (0..midPreSize-1)
    int  midRingFill  = 0;   // samples currently in ring (0..midPreSize)
    int  midPostCount = 0;   // post-trigger samples captured
    bool midTrigFired = false;
    bool midDone      = false;
    int  midPrevVal[N_CHANNELS];
    for (int i = 0; i < N_CHANNELS; i++) midPrevVal[i] = -32768;

    // Size DMA buffer so it fills in ~1/4 of the capture window.
    // This ensures adc.available() returns true well within short captures.
    int nSamples = (int)((sampleRate / 1000UL) * (uint32_t)max(duration / 4, 1));
    if (nSamples < 16)           nSamples = 16;
    if (nSamples > N_SAMPLES_MAX) nSamples = N_SAMPLES_MAX;

    if (adcRunning) adc.stop();
    if (!adc.begin(toResolution(bitRes), sampleRate, nSamples, QUEUE_DEPTH)) {
        Serial.println("ERR:ADC_BEGIN_FAILED");
        return;
    }
    adcRunning = true;

    uint32_t startMs = millis();
    while (!midDone && (midpointMode || millis() - startMs < (uint32_t)duration)) {
        // In midpoint mode, check for an abort command from the host
        if (midpointMode && Serial.available()) {
            String incoming = Serial.readStringUntil('\n');
            incoming.trim();
            if (incoming == F("!ABORT")) { midDone = true; break; }
        }
        if (!adc.available()) continue;

        SampleBuffer buf = adc.read();
        // buf is interleaved: [ch0_t0, ch1_t0, ..., ch7_t0, ch0_t1, ch1_t1, ...]
        int ns = buf.size() / N_CHANNELS;

        for (int s = 0; s < ns; s++) {
            for (int ch = 0; ch < N_CHANNELS; ch++)
                smoothBuf[ch] += buf[s * N_CHANNELS + ch];

            if (++smoothTick >= smoothCount) {
                smoothTick = 0;
                int sv[N_CHANNELS];
                for (int ch = 0; ch < N_CHANNELS; ch++) {
                    sv[ch] = smoothBuf[ch] / smoothCount;
                    smoothBuf[ch] = 0;
                    if (!pinReq[ch]) continue;
                    if (sv[ch] < mn[ch]) mn[ch] = sv[ch];
                    if (sv[ch] > mx[ch]) mx[ch] = sv[ch];
                    sum[ch] += sv[ch];
                    sq[ch]  += (double)sv[ch] * sv[ch];
                    cnt[ch]++;
                }
                sampleTick++;

                if (midpointMode) {
                    // ── Midpoint / pre-post trigger ───────────────────────────
                    if (!midTrigFired) {
                        // Continuously fill ring buffer, overwriting oldest
                        for (int ch = 0; ch < N_CHANNELS; ch++) {
                            if (!pinReq[ch]) continue;
                            logData[ch][midRingHead] = sv[ch];
                        }
                        if (midRingFill < midPreSize) midRingFill++;
                        midRingHead = (midRingHead + 1) % midPreSize;

                        // OR trigger: compare to previous sample (consecutive change)
                        for (int ch = 0; ch < N_CHANNELS; ch++) {
                            if (pinReq[ch] && trigThresh[ch] > 0 &&
                                midPrevVal[ch] != -32768 &&
                                abs(sv[ch] - midPrevVal[ch]) >= trigThresh[ch]) {
                                midTrigFired = true; break;
                            }
                        }
                    } else if (midPostCount < midPostSize) {
                        // Post-trigger: append into the upper half of logData
                        for (int ch = 0; ch < N_CHANNELS; ch++) {
                            if (!pinReq[ch]) continue;
                            logData[ch][midPreSize + midPostCount] = sv[ch];
                        }
                        if (++midPostCount >= midPostSize) midDone = true;
                    }
                    for (int ch = 0; ch < N_CHANNELS; ch++)
                        if (pinReq[ch]) midPrevVal[ch] = sv[ch];

                } else {
                    // ── Normal / threshold trigger ────────────────────────────
                    bool doLog;
                    if (usingTrig) {
                        doLog = false;
                        for (int ch = 0; ch < N_CHANNELS; ch++) {
                            if (pinReq[ch] && trigThresh[ch] > 0 &&
                                abs(sv[ch] - lastLoggedVal[ch]) >= trigThresh[ch]) {
                                doLog = true; break;
                            }
                        }
                    } else {
                        doLog = (sampleTick % logEvery == 0);
                    }

                    if (doLog) {
                        bool anyStored = false;
                        for (int ch = 0; ch < N_CHANNELS; ch++) {
                            if (!pinReq[ch] || logCnt[ch] >= logTarget) continue;
                            logData[ch][logCnt[ch]++] = sv[ch];
                            lastLoggedVal[ch] = sv[ch];
                            anyStored = true;
                        }
                        if (anyStored) lastLogMs = millis() - startMs;
                    }
                }
            }
        }
        buf.release();
    }

    adc.stop();
    adcRunning = false;

    // In midpoint mode, report total captured (ring + post) as logCnt
    if (midpointMode) {
        for (int i = 0; i < N_CHANNELS; i++)
            if (pinReq[i]) logCnt[i] = midRingFill + midPostCount;
    }

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
        Serial.print("|STD:");    Serial.print(stdDev, 4);
        Serial.print("|LOGGED:"); Serial.println(logCnt[i]);
    }

    // 2. Output log data for plotting
    for (int i = 0; i < N_CHANNELS; i++) {
        if (!pinReq[i] || logCnt[i] == 0) continue;
        Serial.print("DATA:"); Serial.print(CH_NAMES[i]); Serial.print("|VALS:");
        if (midpointMode) {
            // Linearise ring (oldest first), then post-trigger block
            int startIdx = (midRingFill < midPreSize) ? 0 : midRingHead;
            for (int j = 0; j < midRingFill; j++) {
                Serial.print(logData[i][(startIdx + j) % midPreSize]);
                if (j < midRingFill - 1 || midPostCount > 0) Serial.print(',');
            }
            for (int j = 0; j < midPostCount; j++) {
                Serial.print(logData[i][midPreSize + j]);
                if (j < midPostCount - 1) Serial.print(',');
            }
        } else {
            for (int j = 0; j < logCnt[i]; j++) {
                Serial.print(logData[i][j]);
                if (j < logCnt[i] - 1) Serial.print(',');
            }
        }
        Serial.println();
    }

    Serial.print("DONE|ELAPSED:");
    if (midpointMode) {
        // Span = total captured samples × time per smoothed sample
        uint32_t spanMs = (uint32_t)((uint64_t)(midRingFill + midPostCount) * smoothCount * 1000 / sampleRate);
        Serial.println(spanMs > 0 ? spanMs : 1);
    } else {
        Serial.println(lastLogMs > 0 ? lastLogMs : (millis() - startMs));
    }
}
