// ─────────────────────────────────────────────────────────────────────────────
//  giga_dual_shared.h — shared inter-core contract + ADC capture engine
//
//  IMPORTANT: this file exists in TWO sketch folders and the copies MUST stay
//  byte-identical:
//      Giga_RemoteController_Dual/giga_dual_shared.h   (M7 sketch)
//      Giga_RemoteController_M4/giga_dual_shared.h     (M4 sketch)
//  Arduino cannot share a header across sibling sketches, so it is duplicated.
//  GIGA_SHM_PROTO_VER is baked into the magic word; if the copies drift out of
//  sync the M7 refuses to talk to the M4 and says so instead of reading
//  garbage. After editing one copy, run:  copy the file over the other.
//
//  ── What this is ────────────────────────────────────────────────────────────
//  The STM32H747 on the Giga R1 has a Cortex-M7 (480 MHz) and a Cortex-M4
//  (240 MHz). This header defines:
//    1. A fixed-address control block in SRAM4 (D3 domain, 0x38000000) that
//       both cores can see, and
//    2. A non-blocking ADC capture engine that reduces samples into that block.
//
//  The engine is compiled into whichever core owns the ADC. Normally that is
//  the M4 (ADC_ON_M4 = 1 in the M7 sketch), leaving the M7 free for WiFi,
//  USB serial, the touchscreen and USB-stick logging. Set ADC_ON_M4 to 0 and
//  the *same* engine runs on the M7 against a local (non-shared) copy of the
//  block — the single-core fallback, still non-blocking.
//
//  ── Memory map of SRAM4 ─────────────────────────────────────────────────────
//  SRAM4 is 64 KB at 0x38000000 and is not used by either core unless the RPC
//  / OpenAMP library is linked in (it claims 0x38000400..0x3800FBFF) or the PDM
//  library is used (it claims the top 1 KB at 0x3800FC00). This sketch pair
//  uses neither, so it owns the region. The block is capped at 60 KB, leaving
//  the PDM page untouched.
//
//  ── Cache coherency ─────────────────────────────────────────────────────────
//  The M7 has a write-back D-cache; the M4 has none. Every field is therefore
//  grouped by *writer* into its own 32-byte-aligned region so the M7 can clean
//  only what it writes and invalidate only what the M4 writes — cleaning a line
//  that the other core also writes would silently clobber it.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <Arduino.h>
#include <Arduino_AdvancedAnalog.h>
#include <string.h>

// ── Flash split guard ─────────────────────────────────────────────────────────
// Tools > Flash split must put the M4 image in FLASH, for BOTH sketches.
//
// The IDE defaults a custom board menu to its FIRST entry, which here is
// "2MB M7 + M4 in SDRAM". That entry links the M4 sketch to run from SDRAM at
// 0x60000000 AND leaves boards.txt's upload.address_m4 empty - so the M4 image
// has no flash destination, the upload puts it nowhere, and bootM4() points the
// M4 at uninitialised SDRAM. The board comes up looking exactly as if the M4
// sketch was never flashed. Catch it at build time instead.
#if !defined(CM4_BINARY_START)
  #error "CM4_BINARY_START undefined - build this for the Arduino Giga R1 board."
#elif (CM4_BINARY_START >= 0x60000000)
  #error "Set Tools > Flash split to \"1MB M7 + 1MB M4\" for BOTH sketches. The default \"2MB M7 + M4 in SDRAM\" cannot flash the M4 image and the co-processor will never start."
#endif

// ── Protocol version ──────────────────────────────────────────────────────────
#define GIGA_SHM_PROTO_VER   2u
#define GIGA_SHM_BASE        0x38000000UL
#define GIGA_SHM_MAGIC       (0x4741D000UL | GIGA_SHM_PROTO_VER)   // 'GA' + ver
#define GIGA_SHM_LIMIT       (60u * 1024u)      // keep clear of the PDM page

// ── Sizes ─────────────────────────────────────────────────────────────────────
#define SHM_N_CH             8
#define SHM_N_DIFF           2
// Live-stream statistics cover the 8 real channels plus the two differential
// pairs. A pair's min/max/sigma cannot be recovered from its legs' statistics,
// so the engine reduces (pos - neg) per sample alongside them. Captures do not
// need this: the host has every stored frame and computes the difference
// exactly from those.
#define SHM_N_STAT           (SHM_N_CH + SHM_N_DIFF)
#define SHM_RING_FRAMES      2048               // 2048 x 8 x 2 B = 32 KB
#define SHM_MAX_LOG          10000              // matches MAX_LOG on the M7

// ── Commands (host → engine) ──────────────────────────────────────────────────
#define GIGA_CMD_IDLE        0u
#define GIGA_CMD_CAPTURE     1u
#define GIGA_CMD_STREAM      2u
#define GIGA_CMD_STOP        3u

// ── Engine states (engine → host) ─────────────────────────────────────────────
#define GIGA_ST_BOOT         0u
#define GIGA_ST_IDLE         1u
#define GIGA_ST_CAPTURING    2u
#define GIGA_ST_CAPTURE_DONE 3u
#define GIGA_ST_STREAMING    4u
#define GIGA_ST_ERROR        5u

// ── Error codes ───────────────────────────────────────────────────────────────
#define GIGA_ERR_NONE        0u
#define GIGA_ERR_ADC_BEGIN   1u

// ── Which core are we compiling for? ──────────────────────────────────────────
// Only the Cortex-M7 defines __DCACHE_PRESENT (and only it has the SCB cache
// maintenance intrinsics), which makes this an exact test for the target core.
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  #define SHM_CORE_M7  1
#else
  #define SHM_CORE_M7  0
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  Shared block layout
// ─────────────────────────────────────────────────────────────────────────────

// Written by the host (M7), read by the engine.
struct ShmToEngine {
    volatile uint32_t cmdSeq;         // host bumps this to issue a command
    volatile uint32_t cmd;            // GIGA_CMD_*
    volatile uint32_t sampleRate;     // Hz
    volatile uint32_t bits;           // 8/10/12/14/16
    volatile uint32_t smooth;         // samples averaged per stored point
    volatile uint32_t durationMs;     // capture length (ignored when streaming)
    volatile uint32_t logTarget;      // points to store per channel
    volatile uint32_t chMask;         // bit i = record channel i
    volatile uint32_t statMask;       // bit i = accumulate stats for channel i
    volatile uint32_t statPeriodMs;   // stream reporting period
    volatile uint32_t midMode;        // 1 = midpoint (pre/post trigger) capture
    volatile uint32_t abortFlag;      // host bumps to abort a midpoint capture
    volatile uint32_t ringTail;       // frames the host has consumed
    volatile uint32_t hostBootId;     // changes every host boot; see ackBootId
    volatile uint32_t hostScratch;    // host-only, used by the SRAM4 self-test
    volatile int32_t  trigThresh[SHM_N_CH];   // raw counts, 0 = channel unarmed
    // Differential pairs reduced during a live stream (stat slots 8 and 9).
    volatile uint32_t diffEn[SHM_N_DIFF];
    volatile uint32_t diffPos[SHM_N_DIFF];
    volatile uint32_t diffNeg[SHM_N_DIFF];
};

// Written by the engine, read by the host.
struct ShmFromEngine {
    volatile uint32_t magic;          // GIGA_SHM_MAGIC once the engine is alive
    // Echoes hostBootId. The two cores can start in either order - with the
    // BCM4 option bit set the M4 boots at reset, before the M7 runs a line -
    // and either can be reset alone. Neither may clear the other's region to
    // announce itself, so instead the host stamps a fresh id each boot and the
    // engine echoes it; a match means both are live and agreed on this session.
    volatile uint32_t ackBootId;
    volatile uint32_t ackSeq;         // echoes cmdSeq when the command is taken
    volatile uint32_t heartbeat;      // free-running, proves the core is alive
    volatile uint32_t state;          // GIGA_ST_*
    volatile uint32_t errCode;        // GIGA_ERR_*

    // Frame ring bookkeeping
    volatile uint32_t ringHead;       // frames produced (monotonic)
    volatile uint32_t ringDropped;    // frames lost because the host fell behind

    // Capture results
    volatile uint32_t elapsedMs;      // signal span, from sample maths
    volatile uint32_t logEvery;       // decimation factor actually used
    volatile uint32_t nLogged;        // frames pushed for this capture
    volatile uint32_t cCount[SHM_N_CH];
    volatile uint32_t cMin[SHM_N_CH];
    volatile uint32_t cMax[SHM_N_CH];
    volatile uint64_t cSum[SHM_N_CH];
    volatile uint64_t cSumSq[SHM_N_CH];

    // Midpoint capture: monotonic frame index at which the trigger fired.
    // 0xFFFFFFFF while still pre-trigger. Frames before it belong in the
    // circular pre-trigger region, frames from it on are post-trigger.
    volatile uint32_t midTrigAt;
    volatile uint32_t midPreSize;

    // Live-stream period statistics (one completed reporting period)
    volatile uint32_t statSeq;        // bumped once per completed period
    volatile uint32_t statTimeMs;     // ms since the stream started
    volatile uint32_t statSpanMs;     // wall-clock length of this period
    // Signed, because slots 8 and 9 hold a difference.
    volatile uint32_t sCount[SHM_N_STAT];
    volatile int32_t  sMin[SHM_N_STAT];
    volatile int32_t  sMax[SHM_N_STAT];
    volatile int32_t  sLast[SHM_N_STAT];
    volatile int64_t  sSum[SHM_N_STAT];
    volatile uint64_t sSumSq[SHM_N_STAT];
};

// The three regions are padded to whole cache lines and never share one.
struct GigaShm {
    union { ShmToEngine   h; uint8_t _pad_h[256];  };
    union { ShmFromEngine e; uint8_t _pad_e[1024]; };
    uint16_t ring[SHM_RING_FRAMES][SHM_N_CH];
};

static_assert(sizeof(ShmToEngine)   <= 256,  "ShmToEngine outgrew its cache-line block");
static_assert(sizeof(ShmFromEngine) <= 1024, "ShmFromEngine outgrew its cache-line block");
static_assert(sizeof(GigaShm)       <= GIGA_SHM_LIMIT, "shared block would overrun SRAM4");
// Each writer's region must start on its own cache line, or the M7 cleaning
// its own block would write back stale copies of the M4's.
static_assert(offsetof(GigaShm, e)    % 32 == 0, "engine block is not cache-line aligned");
static_assert(offsetof(GigaShm, ring) % 32 == 0, "frame ring is not cache-line aligned");

// ─────────────────────────────────────────────────────────────────────────────
//  Cache maintenance
//
//  Both helpers are no-ops on the M4 (no D-cache) and no-ops on the M7 when the
//  block lives in ordinary RAM — which is the single-core fallback, where the
//  same core is both writer and reader and an invalidate would throw away its
//  own pending writes.
// ─────────────────────────────────────────────────────────────────────────────
static inline bool shmInSRAM4(const volatile void* p) {
    return ((uint32_t)p & 0xFFFF0000u) == (uint32_t)GIGA_SHM_BASE;
}

static inline void shmClean(const volatile void* p, size_t n) {
#if SHM_CORE_M7
    if (!shmInSRAM4(p)) return;
    uint32_t s = (uint32_t)p & ~31u;
    uint32_t e = ((uint32_t)p + n + 31u) & ~31u;
    SCB_CleanDCache_by_Addr((uint32_t*)s, (int32_t)(e - s));
#else
    (void)p; (void)n;
#endif
    __DMB();
}

static inline void shmInvalidate(const volatile void* p, size_t n) {
#if SHM_CORE_M7
    if (!shmInSRAM4(p)) { __DMB(); return; }
    uint32_t s = (uint32_t)p & ~31u;
    uint32_t e = ((uint32_t)p + n + 31u) & ~31u;
    SCB_InvalidateDCache_by_Addr((uint32_t*)s, (int32_t)(e - s));
#else
    (void)p; (void)n;
#endif
    __DMB();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Capture engine — runs on whichever core owns the ADC
// ─────────────────────────────────────────────────────────────────────────────
#define ENG_MODE_IDLE    0
#define ENG_MODE_CAPTURE 1
#define ENG_MODE_STREAM  2

struct CapEngine {
    GigaShm*     shm;
    AdvancedADC* adc;
    bool         adcRunning;
    uint32_t     lastCmdSeq;
    uint32_t     lastAbort;
    int          mode;

    // Config snapshot, taken when the command is accepted
    uint32_t rate, bits, smooth, durationMs, logTarget, statPeriodMs;
    bool     act[SHM_N_CH];        // record
    bool     sta[SHM_N_STAT];      // accumulate statistics (8 real + 2 diffs)
    int32_t  trig[SHM_N_CH];
    bool     dEn[SHM_N_DIFF];
    int      dPos[SHM_N_DIFF], dNeg[SHM_N_DIFF];
    bool     usingTrig, midMode;

    // Smoothing accumulator
    uint32_t sAcc[SHM_N_CH];
    uint32_t smoothTick;

    // Capture-wide statistics
    uint32_t cCnt[SHM_N_CH];
    uint16_t cMn[SHM_N_CH], cMx[SHM_N_CH];
    uint64_t cSum[SHM_N_CH], cSq[SHM_N_CH];

    // Live-stream period statistics
    uint32_t pCnt[SHM_N_STAT];
    int32_t  pMn[SHM_N_STAT], pMx[SHM_N_STAT], pLast[SHM_N_STAT];
    int64_t  pSum[SHM_N_STAT];
    uint64_t pSq[SHM_N_STAT];

    // Decimation / trigger logging
    uint32_t sampleTick, logEvery, nLogged;
    int32_t  lastLogged[SHM_N_CH];

    // Midpoint capture
    uint32_t midPreSize, midPostSize, midPostCount, midTrigAt;
    bool     midTrigFired;
    int32_t  midPrev[SHM_N_CH];

    // Ring producer state
    uint32_t ringHead, ringDropped;

    // Timing
    uint32_t startMs, periodStartMs;
};

// ── Publishing helpers ────────────────────────────────────────────────────────
static inline void engPublishStatus(CapEngine& e) {
    shmClean(&e.shm->e, sizeof(ShmFromEngine));
}

static inline void engPushFrame(CapEngine& e, const uint16_t* sv) {
    GigaShm* s = e.shm;
    // The host's tail is written by the other core; on the M7 fallback the read
    // is plain, on the M4 SRAM4 is uncached, so no invalidate is needed here.
    uint32_t tail = s->h.ringTail;
    if ((uint32_t)(e.ringHead - tail) >= SHM_RING_FRAMES) {
        e.ringDropped++;
        s->e.ringDropped = e.ringDropped;
        return;
    }
    uint16_t* dst = s->ring[e.ringHead % SHM_RING_FRAMES];
    for (int ch = 0; ch < SHM_N_CH; ch++) dst[ch] = sv[ch];
    shmClean(dst, sizeof(uint16_t) * SHM_N_CH);
    e.ringHead++;
    e.nLogged++;
    s->e.ringHead = e.ringHead;
    s->e.nLogged  = e.nLogged;
}

static inline uint32_t engToResolution(uint32_t bits) {
    if (bits <= 8)  return AN_RESOLUTION_8;
    if (bits <= 10) return AN_RESOLUTION_10;
    if (bits <= 12) return AN_RESOLUTION_12;
    if (bits <= 14) return AN_RESOLUTION_14;
    return AN_RESOLUTION_16;
}

// ── Statistics reset ──────────────────────────────────────────────────────────
static inline void engResetCapStats(CapEngine& e) {
    for (int i = 0; i < SHM_N_CH; i++) {
        e.cCnt[i] = 0; e.cMn[i] = 0xFFFF; e.cMx[i] = 0;
        e.cSum[i] = 0; e.cSq[i] = 0;
    }
}
static inline void engResetPeriod(CapEngine& e) {
    for (int i = 0; i < SHM_N_STAT; i++) {
        e.pCnt[i] = 0; e.pMn[i] = INT32_MAX; e.pMx[i] = INT32_MIN; e.pLast[i] = 0;
        e.pSum[i] = 0; e.pSq[i] = 0;
    }
    e.periodStartMs = millis();
}

// ── Engine lifecycle ──────────────────────────────────────────────────────────
static inline void engBegin(CapEngine& e, GigaShm* shm, AdvancedADC* adc) {
    memset(&e, 0, sizeof(e));
    e.shm = shm;
    e.adc = adc;
    e.mode = ENG_MODE_IDLE;
    e.midTrigAt = 0xFFFFFFFFu;

    // Only the engine's own region is initialised. Clearing the whole block
    // would wipe a host boot id that was written before this core started.
    memset((void*)&shm->e, 0, sizeof(ShmFromEngine));
    shm->e.state     = GIGA_ST_IDLE;
    shm->e.midTrigAt = 0xFFFFFFFFu;
    shm->e.ackBootId = ~shm->h.hostBootId;  // force the resync path in engStep
    shm->e.magic     = GIGA_SHM_MAGIC;
    engPublishStatus(e);
}

static inline void engStopAdc(CapEngine& e) {
    if (e.adcRunning) { e.adc->stop(); e.adcRunning = false; }
}

static inline void engFinishCapture(CapEngine& e) {
    engStopAdc(e);
    GigaShm* s = e.shm;

    for (int i = 0; i < SHM_N_CH; i++) {
        s->e.cCount[i] = e.cCnt[i];
        s->e.cMin[i]   = e.cMn[i];
        s->e.cMax[i]   = e.cMx[i];
        s->e.cSum[i]   = e.cSum[i];
        s->e.cSumSq[i] = e.cSq[i];
    }
    s->e.logEvery   = e.logEvery;
    s->e.nLogged    = e.nLogged;
    s->e.midTrigAt  = e.midTrigAt;
    s->e.midPreSize = e.midPreSize;

    // Signal span from the sample maths, not the wall clock: ADC start-up
    // latency and loop jitter inflate millis() and would stretch the GUI's time
    // axis. Trigger mode logs on change (non-uniform spacing) so wall clock is
    // the only meaningful span there.
    uint32_t span;
    if (e.midMode) {
        span = (uint32_t)((uint64_t)e.nLogged * e.smooth * 1000ull / e.rate);
    } else if (e.usingTrig) {
        span = millis() - e.startMs;
    } else {
        span = (uint32_t)((uint64_t)e.nLogged * e.logEvery * e.smooth * 1000ull / e.rate);
    }
    if (span == 0) span = millis() - e.startMs;
    s->e.elapsedMs = span;

    __DMB();                                // results land before the state flag
    s->e.state = GIGA_ST_CAPTURE_DONE;
    e.mode = ENG_MODE_IDLE;
    engPublishStatus(e);
}

static inline void engPublishPeriod(CapEngine& e) {
    GigaShm* s = e.shm;
    uint32_t now = millis();
    for (int i = 0; i < SHM_N_STAT; i++) {
        s->e.sCount[i] = e.pCnt[i];
        s->e.sMin[i]   = e.pCnt[i] ? e.pMn[i] : 0;
        s->e.sMax[i]   = e.pCnt[i] ? e.pMx[i] : 0;
        s->e.sLast[i]  = e.pLast[i];
        s->e.sSum[i]   = e.pSum[i];
        s->e.sSumSq[i] = e.pSq[i];
    }
    s->e.statTimeMs = now - e.startMs;
    s->e.statSpanMs = now - e.periodStartMs;
    __DMB();                                // payload lands before the flag
    s->e.statSeq    = s->e.statSeq + 1;     // bumped last — it is the "ready" flag
    engPublishStatus(e);
    engResetPeriod(e);
}

// ── Command intake ────────────────────────────────────────────────────────────
static inline void engStartCommand(CapEngine& e) {
    GigaShm* s = e.shm;
    uint32_t cmd = s->h.cmd;

    engStopAdc(e);
    e.mode = ENG_MODE_IDLE;
    s->e.errCode = GIGA_ERR_NONE;

    // Resync the abort counter. The abort check in engStep only runs while a
    // capture is in flight, so an abort raised at any other moment would
    // otherwise still be pending and cut the next capture short immediately.
    e.lastAbort = s->h.abortFlag;

    if (cmd == GIGA_CMD_STOP || cmd == GIGA_CMD_IDLE) {
        s->e.state = GIGA_ST_IDLE;
        s->e.ackSeq = e.lastCmdSeq;
        engPublishStatus(e);
        return;
    }

    // Snapshot the configuration so a later host write cannot change it mid-run.
    e.rate         = s->h.sampleRate ? s->h.sampleRate : 500000u;
    e.bits         = s->h.bits ? s->h.bits : 12u;
    e.smooth       = s->h.smooth ? s->h.smooth : 1u;
    e.durationMs   = s->h.durationMs ? s->h.durationMs : 1000u;
    e.logTarget    = s->h.logTarget ? s->h.logTarget : 1u;
    e.statPeriodMs = s->h.statPeriodMs ? s->h.statPeriodMs : 250u;
    e.midMode      = (cmd == GIGA_CMD_CAPTURE) && (s->h.midMode != 0);
    if (e.logTarget > SHM_MAX_LOG) e.logTarget = SHM_MAX_LOG;

    uint32_t chMask  = s->h.chMask;
    uint32_t staMask = s->h.statMask;
    for (int d = 0; d < SHM_N_DIFF; d++) {
        e.dEn[d]  = (cmd == GIGA_CMD_STREAM) && s->h.diffEn[d];
        e.dPos[d] = s->h.diffPos[d] % SHM_N_CH;
        e.dNeg[d] = s->h.diffNeg[d] % SHM_N_CH;
        e.sta[SHM_N_CH + d] = e.dEn[d];
    }
    e.usingTrig = false;
    for (int i = 0; i < SHM_N_CH; i++) {
        e.act[i]  = (chMask  >> i) & 1u;
        e.sta[i]  = (staMask >> i) & 1u;
        e.trig[i] = s->h.trigThresh[i];
        if (e.act[i] && e.trig[i] > 0) e.usingTrig = true;
        e.lastLogged[i] = INT32_MIN;
        e.midPrev[i]    = INT32_MIN;
        e.sAcc[i]       = 0;
    }
    // Streaming with thresholds armed is how the on-board USB datalogger runs:
    // period statistics for the display, plus a change-triggered frame whenever
    // a channel moves. With every threshold at 0 (the live-view case) no frames
    // are pushed at all and the ring stays idle.

    e.smoothTick = 0;
    e.sampleTick = 0;
    e.nLogged    = 0;
    e.ringHead   = 0;
    e.ringDropped = 0;
    e.midTrigFired = false;
    e.midTrigAt    = 0xFFFFFFFFu;
    e.midPostCount = 0;
    e.midPreSize   = e.logTarget / 2;
    e.midPostSize  = e.logTarget - e.midPreSize;
    if (e.midPreSize == 0) { e.midPreSize = 1; e.midPostSize = e.logTarget - 1; }

    engResetCapStats(e);
    engResetPeriod(e);

    // Uniform decimation factor for a normal capture.
    uint32_t totalSmoothed = (uint32_t)((uint64_t)e.rate * e.durationMs / 1000ull / e.smooth);
    e.logEvery = (totalSmoothed > e.logTarget && e.logTarget > 0)
                 ? totalSmoothed / e.logTarget : 1u;
    if (e.logEvery == 0) e.logEvery = 1;

    // Reset the ring. ringTail belongs to the host, which zeroes it before
    // ringing the doorbell — the engine must never write into the host's cache
    // lines or the host's next clean would write its stale copy back.
    s->e.ringHead    = 0;
    s->e.ringDropped = 0;
    s->e.nLogged     = 0;
    s->e.midTrigAt   = 0xFFFFFFFFu;
    s->e.midPreSize  = e.midPreSize;
    s->e.statSeq     = 0;

    // DMA buffer sizing: a quarter of the capture, clamped.
    uint32_t dur = (cmd == GIGA_CMD_STREAM) ? e.statPeriodMs : e.durationMs;
    int nSamples = (int)((e.rate / 1000u) * (dur / 4u ? dur / 4u : 1u));
    if (nSamples < 16)  nSamples = 16;
    if (nSamples > 512) nSamples = 512;

    if (!e.adc->begin(engToResolution(e.bits), e.rate, nSamples, 8)) {
        s->e.errCode = GIGA_ERR_ADC_BEGIN;
        s->e.state   = GIGA_ST_ERROR;
        s->e.ackSeq  = e.lastCmdSeq;
        engPublishStatus(e);
        return;
    }
    e.adcRunning = true;
    e.startMs    = millis();
    e.periodStartMs = e.startMs;

    if (cmd == GIGA_CMD_STREAM) {
        e.mode = ENG_MODE_STREAM;
        s->e.state = GIGA_ST_STREAMING;
    } else {
        e.mode = ENG_MODE_CAPTURE;
        s->e.state = GIGA_ST_CAPTURING;
    }
    s->e.elapsedMs = 0;
    s->e.ackSeq    = e.lastCmdSeq;
    engPublishStatus(e);
}

// ── The hot loop ──────────────────────────────────────────────────────────────
// Non-blocking: drains at most one DMA buffer per call, so the owning core can
// still poll for commands (and, in the single-core fallback, service WiFi).
static inline void engStep(CapEngine& e) {
    GigaShm* s = e.shm;

    s->e.heartbeat = s->e.heartbeat + 1;

    shmInvalidate(&s->h, sizeof(ShmToEngine));

    // A new host boot id means the host (re)started. Adopt its counters rather
    // than treating their values as commands, and re-announce.
    if (s->e.ackBootId != s->h.hostBootId) {
        engStopAdc(e);
        e.mode       = ENG_MODE_IDLE;
        e.lastCmdSeq = s->h.cmdSeq;
        e.lastAbort  = s->h.abortFlag;
        s->e.state   = GIGA_ST_IDLE;
        s->e.errCode = GIGA_ERR_NONE;
        s->e.magic   = GIGA_SHM_MAGIC;
        __DMB();
        s->e.ackBootId = s->h.hostBootId;   // last: it is the "in sync" flag
        engPublishStatus(e);
        return;
    }

    // Command intake — the host bumps cmdSeq after writing the config.
    if (s->h.cmdSeq != e.lastCmdSeq) {
        e.lastCmdSeq = s->h.cmdSeq;
        engStartCommand(e);
        return;
    }
    if (e.mode == ENG_MODE_IDLE) {
        // Heartbeat only needs publishing occasionally when idle.
        if ((s->e.heartbeat & 0xFFu) == 0) engPublishStatus(e);
        return;
    }

    // Abort (midpoint captures can run indefinitely waiting for a trigger).
    if (s->h.abortFlag != e.lastAbort) {
        e.lastAbort = s->h.abortFlag;
        engFinishCapture(e);
        return;
    }

    uint32_t now = millis();

    if (e.mode == ENG_MODE_CAPTURE && !e.midMode &&
        (now - e.startMs) >= e.durationMs) {
        engFinishCapture(e);
        return;
    }
    if (e.mode == ENG_MODE_STREAM && (now - e.periodStartMs) >= e.statPeriodMs) {
        engPublishPeriod(e);
    }

    if (!e.adc->available()) return;
    SampleBuffer buf = e.adc->read();
    int ns = buf.size() / SHM_N_CH;

    const uint32_t smooth = e.smooth;
    const bool     stream = (e.mode == ENG_MODE_STREAM);

    for (int i = 0; i < ns; i++) {
        const int base = i * SHM_N_CH;
        for (int ch = 0; ch < SHM_N_CH; ch++) e.sAcc[ch] += buf[base + ch];
        if (++e.smoothTick < smooth) continue;
        e.smoothTick = 0;

        uint16_t sv[SHM_N_CH];
        for (int ch = 0; ch < SHM_N_CH; ch++) {
            sv[ch] = (uint16_t)(e.sAcc[ch] / smooth);
            e.sAcc[ch] = 0;
        }

        if (stream) {
            for (int k = 0; k < SHM_N_STAT; k++) {
                if (!e.sta[k]) continue;
                int32_t v;
                if (k < SHM_N_CH) v = sv[k];
                else {
                    int d = k - SHM_N_CH;
                    v = (int32_t)sv[e.dPos[d]] - (int32_t)sv[e.dNeg[d]];
                }
                if (v < e.pMn[k]) e.pMn[k] = v;
                if (v > e.pMx[k]) e.pMx[k] = v;
                e.pSum[k] += v;
                e.pSq[k]  += (uint64_t)((int64_t)v * v);
                e.pCnt[k]++;
                e.pLast[k] = v;
            }
            // Unarmed (all thresholds 0) is the live-view case: no ring traffic.
            // Armed is the datalogger case: forward the frames that moved.
            if (e.usingTrig) {
                bool fwd = false;
                for (int ch = 0; ch < SHM_N_CH; ch++) {
                    if (!e.act[ch] || e.trig[ch] <= 0) continue;
                    if (e.lastLogged[ch] == INT32_MIN ||
                        abs((int32_t)sv[ch] - e.lastLogged[ch]) >= e.trig[ch]) { fwd = true; break; }
                }
                if (fwd) {
                    engPushFrame(e, sv);
                    for (int ch = 0; ch < SHM_N_CH; ch++)
                        if (e.act[ch]) e.lastLogged[ch] = sv[ch];
                }
            }
            continue;
        }

        for (int ch = 0; ch < SHM_N_CH; ch++) {
            if (!e.sta[ch]) continue;
            uint16_t v = sv[ch];
            if (v < e.cMn[ch]) e.cMn[ch] = v;
            if (v > e.cMx[ch]) e.cMx[ch] = v;
            e.cSum[ch] += v;
            e.cSq[ch]  += (uint32_t)v * (uint32_t)v;
            e.cCnt[ch]++;
        }
        e.sampleTick++;

        if (e.midMode) {
            // Every smoothed frame goes to the host, which keeps the circular
            // pre-trigger region itself. midTrigAt tells it where post-trigger
            // starts.
            engPushFrame(e, sv);
            if (!e.midTrigFired) {
                for (int ch = 0; ch < SHM_N_CH; ch++) {
                    if (!e.act[ch] || e.trig[ch] <= 0 || e.midPrev[ch] == INT32_MIN) continue;
                    if (abs((int32_t)sv[ch] - e.midPrev[ch]) >= e.trig[ch]) {
                        e.midTrigFired = true;
                        e.midTrigAt    = e.ringHead;   // first post-trigger frame
                        s->e.midTrigAt = e.midTrigAt;
                        break;
                    }
                }
            } else if (++e.midPostCount >= e.midPostSize) {
                buf.release();
                engFinishCapture(e);
                return;
            }
            for (int ch = 0; ch < SHM_N_CH; ch++)
                if (e.act[ch]) e.midPrev[ch] = sv[ch];
            continue;
        }

        bool doLog;
        if (e.usingTrig) {
            doLog = false;
            for (int ch = 0; ch < SHM_N_CH; ch++) {
                if (!e.act[ch] || e.trig[ch] <= 0) continue;
                if (e.lastLogged[ch] == INT32_MIN ||
                    abs((int32_t)sv[ch] - e.lastLogged[ch]) >= e.trig[ch]) { doLog = true; break; }
            }
        } else {
            doLog = (e.sampleTick % e.logEvery) == 0;
        }
        if (doLog && e.nLogged < e.logTarget) {
            engPushFrame(e, sv);
            for (int ch = 0; ch < SHM_N_CH; ch++)
                if (e.act[ch]) e.lastLogged[ch] = sv[ch];
        }
    }
    buf.release();
}
