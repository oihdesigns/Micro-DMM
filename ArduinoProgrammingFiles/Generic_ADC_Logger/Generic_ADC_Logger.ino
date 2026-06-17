/*
 * Generic_ADC_Logger.ino
 *
 * Drop-in replacement for Giga_RemoteController.ino that works on any board
 * supporting analogRead() — Uno, Mega, Due, Zero, Nano 33, Teensy, etc.
 *
 * Uses the identical serial protocol; the Giga_RemoteController Python GUI
 * connects without modification.
 *
 * ── Board configuration ───────────────────────────────────────────────────────
 *   N_CHANNELS    : number of analog channels (Uno/Zero: 6, Mega: up to 16,
 *                   Nano/MKR/Nano33: 8)
 *   MAX_LOG       : log points per channel.  On AVR (int = 2 bytes):
 *                     Uno/Nano  — keep ≤ 80  (leaves ~1 KB for stack/vars)
 *                     Mega      — up to 500
 *                   On ARM (int = 4 bytes):
 *                     Due/Zero/Nano33/MKR — up to 2000+
 *   ANALOG_RATE_HZ: raw analogRead() calls per second on your board.
 *                   Used only for even-spaced logging (ignored in trigger mode).
 *                     Uno/Nano (16 MHz AVR) ≈ 10 000
 *                     Mega     (16 MHz AVR) ≈ 10 000
 *                     Due      (84 MHz ARM) ≈ 1 000 000
 *                     Zero/MKR (48 MHz ARM) ≈ 350 000
 *                     Teensy 4.1            ≈ 5 000 000
 *   ANALOG_PINS   : pin list — extend or shorten to match N_CHANNELS.
 *                   If your board lacks A6/A7, reduce N_CHANNELS to 6.
 * ─────────────────────────────────────────────────────────────────────────────
 */

// ── Board configuration ───────────────────────────────────────────────────────
static const int      N_CHANNELS     = 8;      // ← adjust for your board
static const int      MAX_LOG        = 511;     // ← reduce for Uno/Nano, raise for ARM
static const uint32_t ANALOG_RATE_HZ = 50000;  // ← raw analogRead rate for your board

static const int   ANALOG_PINS[8]  = {A0, A1, A2, A3};//, A4, A5, A6, A7}; // last 2 unused if N_CHANNELS≤6
static const char* CH_NAMES[8]     = {"A0","A1","A2","A3","A4","A5","A6","A7"};

static int logData[N_CHANNELS][MAX_LOG];

void setup() {
    Serial.begin(115200);
    while (!Serial);
}

void loop() {
    if (!Serial.available()) return;

    // Command format: PINS:A0,A1|BITS:12|TIME:1000|SMOOTH:7|LOG:100|RATE:10000|TRIG:0,...|MID:0
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int bitRes      = cmd.substring(cmd.indexOf("BITS:")   + 5, cmd.indexOf("|TIME:")).toInt();
    int duration    = cmd.substring(cmd.indexOf("TIME:")   + 5, cmd.indexOf("|SMOOTH:")).toInt();
    int smoothCount = cmd.substring(cmd.indexOf("SMOOTH:") + 7, cmd.indexOf("|LOG:")).toInt();
    int logTarget   = cmd.substring(cmd.indexOf("LOG:")    + 4).toInt();
    int rateIdx     = cmd.indexOf("|RATE:");
    // RATE overrides ANALOG_RATE_HZ if provided; use as the raw analogRead rate hint
    uint32_t boardRate = (rateIdx >= 0) ? (uint32_t)cmd.substring(rateIdx + 6).toInt() : ANALOG_RATE_HZ;

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

    if (bitRes < 8)          bitRes      = 10;
    if (duration < 1)        duration    = 1000;
    if (smoothCount < 1)     smoothCount = 1;
    if (logTarget   < 1)     logTarget   = 1;
    if (logTarget > MAX_LOG) logTarget   = MAX_LOG;
    if (boardRate   < 1000)  boardRate   = ANALOG_RATE_HZ;

    // Set ADC resolution — only available on non-AVR boards
#if !defined(__AVR__)
    analogReadResolution(bitRes);
#else
    bitRes = 10;  // AVR is always 10-bit
#endif

    // Parse requested channels
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
    uint32_t cnt[N_CHANNELS] = {};
    double   sum[N_CHANNELS] = {};
    double   sq[N_CHANNELS]  = {};
    int      mn[N_CHANNELS], mx[N_CHANNELS];
    for (int i = 0; i < N_CHANNELS; i++) { mn[i] = 65535; mx[i] = 0; }
    int logCnt[N_CHANNELS] = {};

    int  lastLoggedVal[N_CHANNELS];
    for (int i = 0; i < N_CHANNELS; i++) lastLoggedVal[i] = -32768;
    bool usingTrig = false;
    for (int i = 0; i < N_CHANNELS; i++)
        if (pinReq[i] && trigThresh[i] > 0) { usingTrig = true; break; }

    // Effective smoothed rate: one outer-loop iteration reads all N_CHANNELS × smoothCount times
    uint32_t smoothedRate = boardRate / ((uint32_t)N_CHANNELS * (uint32_t)smoothCount);
    if (smoothedRate < 1) smoothedRate = 1;
    uint32_t totalSmoothed = smoothedRate * (uint32_t)duration / 1000;
    uint32_t logEvery  = (totalSmoothed > (uint32_t)logTarget) ? totalSmoothed / logTarget : 1;
    uint32_t sampleTick = 0;
    uint32_t lastLogMs  = 0;

    // Midpoint-trigger ring-buffer state
    int  midPreSize   = logTarget / 2;
    int  midPostSize  = logTarget - midPreSize;
    int  midRingHead  = 0;
    int  midRingFill  = 0;
    int  midPostCount = 0;
    bool midTrigFired = false;
    bool midDone      = false;
    int  midPrevVal[N_CHANNELS];
    for (int i = 0; i < N_CHANNELS; i++) midPrevVal[i] = -32768;

    uint32_t startMs = millis();
    while (!midDone && (midpointMode || millis() - startMs < (uint32_t)duration)) {

        // In midpoint mode, poll for abort command from host
        if (midpointMode && Serial.available()) {
            String incoming = Serial.readStringUntil('\n');
            incoming.trim();
            if (incoming == F("!ABORT")) { midDone = true; break; }
        }

        // Read all channels, averaging smoothCount samples each
        int sv[N_CHANNELS];
        for (int ch = 0; ch < N_CHANNELS; ch++) {
            long acc = 0;
            for (int s = 0; s < smoothCount; s++)
                acc += analogRead(ANALOG_PINS[ch]);
            sv[ch] = (int)(acc / smoothCount);
        }
        sampleTick++;

        // Stats for all requested channels
        for (int ch = 0; ch < N_CHANNELS; ch++) {
            if (!pinReq[ch]) continue;
            if (sv[ch] < mn[ch]) mn[ch] = sv[ch];
            if (sv[ch] > mx[ch]) mx[ch] = sv[ch];
            sum[ch] += sv[ch];
            sq[ch]  += (double)sv[ch] * sv[ch];
            cnt[ch]++;
        }

        if (midpointMode) {
            // ── Midpoint / pre-post trigger ───────────────────────────────
            if (!midTrigFired) {
                for (int ch = 0; ch < N_CHANNELS; ch++) {
                    if (!pinReq[ch]) continue;
                    logData[ch][midRingHead] = sv[ch];
                }
                if (midRingFill < midPreSize) midRingFill++;
                midRingHead = (midRingHead + 1) % midPreSize;

                for (int ch = 0; ch < N_CHANNELS; ch++) {
                    if (pinReq[ch] && trigThresh[ch] > 0 &&
                        midPrevVal[ch] != -32768 &&
                        abs(sv[ch] - midPrevVal[ch]) >= trigThresh[ch]) {
                        midTrigFired = true; break;
                    }
                }
            } else if (midPostCount < midPostSize) {
                for (int ch = 0; ch < N_CHANNELS; ch++) {
                    if (!pinReq[ch]) continue;
                    logData[ch][midPreSize + midPostCount] = sv[ch];
                }
                if (++midPostCount >= midPostSize) midDone = true;
            }
            for (int ch = 0; ch < N_CHANNELS; ch++)
                if (pinReq[ch]) midPrevVal[ch] = sv[ch];

        } else {
            // ── Normal / threshold trigger ────────────────────────────────
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

    // In midpoint mode, set logCnt from ring + post counters
    if (midpointMode) {
        for (int i = 0; i < N_CHANNELS; i++)
            if (pinReq[i]) logCnt[i] = midRingFill + midPostCount;
    }

    // 1. Output stats
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
        Serial.print("|STD:");   Serial.print(stdDev, 4);
        Serial.print("|LOGGED:"); Serial.println(logCnt[i]);
    }

    // 2. Output log data
    for (int i = 0; i < N_CHANNELS; i++) {
        if (!pinReq[i] || logCnt[i] == 0) continue;
        Serial.print("DATA:"); Serial.print(CH_NAMES[i]); Serial.print("|VALS:");
        if (midpointMode) {
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
        uint32_t spanMs = (smoothedRate > 0)
            ? (uint32_t)((uint64_t)(midRingFill + midPostCount) * 1000 / smoothedRate)
            : 1;
        Serial.println(spanMs > 0 ? spanMs : 1);
    } else {
        Serial.println(lastLogMs > 0 ? lastLogMs : (millis() - startMs));
    }
}
