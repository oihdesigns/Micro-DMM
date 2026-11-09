/*!
 * Giga AC Power Monitor (Wi-Fi)
 * The ac_power_monitor sketch moved to an Arduino Giga R1 WiFi, controlled over
 * Wi-Fi (TCP port 8080) or USB serial. The line protocol is the same on both.
 *
 * ── What changed from ac_power_monitor, and why ─────────────────────────────
 *
 *  1. V and I are sampled AT THE SAME TIME, not alternately.
 *     ADC1 and ADC2 run in dual regular-simultaneous mode (AdvancedADCDual),
 *     paced by a timer and drained by DMA:
 *         ADC1 rank 0 = V+ (A0)    ADC2 rank 0 = V- (A1)    <- same instant
 *         ADC1 rank 1 = I+ (A2)    ADC2 rank 1 = I- (A3)    <- same instant
 *     So each pseudo-differential pair is subtracted from two samples taken
 *     at one instant. Giga_RemoteController_WiFi measured what that is worth:
 *     common-mode ripple cancels instead of turning into differential noise.
 *     V and I are one conversion apart (~1.3 us at the defaults, 0.03 deg at
 *     60 Hz). That lag is known exactly, so the maths removes it; it is not a
 *     "half a sample" estimate as it was on the single-mux boards.
 *
 *  2. Gap-free. The old sketches acquired a window, stopped, did the maths and
 *     printed. Here the ADC never stops. Decimated samples go into a ring in
 *     SDRAM and are analysed cycle by cycle, so every mains cycle is measured
 *     and the energy count has no holes. If the CPU ever falls behind and a
 *     DMA buffer is lost, that is detected, counted ($INFO gaps) and flagged
 *     (+16 in the flags). It is never silently spliced.
 *
 *  3. Oversample and average (the RA4M1 backend's idea, at a bigger scale).
 *     The ADC scans at RATE (default 80 kHz) and every DECIM scans
 *     (default 10) are boxcar-averaged into one analysed sample: 8 kS/s,
 *     133 samples per 60 Hz cycle. That gives ~3x less noise and a real
 *     (if gentle) anti-alias filter. The Giga's own input does not filter
 *     anything below ~5 MHz.
 *
 *  4. Less data on the wire: the board does the reductions.
 *       $PWR  per window (CYCLES mains cycles)   ~150 B, 5 per second
 *       $CYC  per mains cycle                    ~60 B, 60 per second
 *       $HARM harmonics 1..H, magnitude + phase  ~1.5 KB per window
 *       $AVG  one cycle-synchronous AVERAGED cycle of V and I, 128 points each
 *             (the shape of the waveform, noise averaged out, ~2 KB per window)
 *       $WAVE raw capture from the ring, sent only when something happens:
 *             an I step, a V sag/swell, a relay close (inrush), a peak trip,
 *             or !WAVE. Pre-trigger data comes from the ring, so a capture
 *             can include the time before the event.
 *     Output is queued and sent without blocking (net_sink.h), so a big
 *     $WAVE goes out while the next cycles are still being measured.
 *
 *  Why one core, not "M4 samples while M7 transmits":
 *     The DMA already samples while the CPU does other work. The loop only has
 *     to empty the buffers, and the pool holds ~200 ms. The things that could
 *     block for longer were all taken off the loop thread:
 *       - output: non-blocking TCP and USB sends from per-output rings
 *         (net_sink.h). A slow link sheds bulk lines and never stalls the loop;
 *       - the Wi-Fi join (WiFi.begin blocks 5-20+ s): it runs on its own RTOS
 *         thread (wifiTask), so a missing network costs no samples. This is
 *         the same idea, one thread connecting while the other samples,
 *         without a second core.
 *     Measured 2026-11-08 on a -84 dBm link that managed 0.1 KB/s: 0 gaps,
 *     worst loop 2.8 ms. If $INFO ever shows gaps anyway, moving the engine
 *     to the M4 (the Giga_RemoteController_Dual pattern) is the next step. The
 *     engine only talks to the rest of the sketch through the ring and the
 *     emit*() calls, to keep that move easy.
 *
 * ── Scaling ─────────────────────────────────────────────────────────────────
 *   "This many ADC volts differential = this much real world": the default
 *   1.5 V = 170 V and 1.5 V = 25 A, set with !VSCALE / !ISCALE.
 *
 * ── Current range (D6) ──────────────────────────────────────────────────────
 *   D6 selects the current front end's range: LOW = range 0, HIGH = range 1
 *   (!IRINV,1 swaps them). Each range has its own ISCALE, IOFF and PHASECAL.
 *   Range 0 is selected at boot, and defaults to 1.5 V = 25 A; range 1
 *   defaults to 1.5 V = 2.5 A. If the switch changes a shunt, make range 0
 *   the high-current one, so a reboot can never leave a big current on the
 *   sensitive shunt.
 *   Every scan is scaled by the range it was taken on. DMA buffers carry the
 *   time they filled, so the switch lands on the right scan even though the
 *   buffers in flight still hold the old range. The ring holds amps, so a
 *   cycle, a window or a capture that spans a switch stays in one unit.
 *   Within IRSETTLE of the switch (the front end settling; a relay needs
 *   ~15 ms) each current sample is replaced by the one a mains cycle earlier
 *   and flagged +128, as is any cycle or window that contains a switch.
 *   Auto mode (!IRANGE,A) goes up at once on any sample over IRLIM up % of
 *   the sensitive range's full scale, or on clipping. It comes down after two
 *   windows whose peak is under IRLIM down %, at most once a second.
 *   Full scale is taken as VREF/2 of differential input.
 *
 * ── Measurement (shared maths, ported) ──────────────────────────────────────
 *   Cycles run between rising zero crossings of V, found to a fraction of a
 *   sample (with hysteresis), and integrated to those fractional endpoints
 *   with a piecewise-cubic rule (4th order; the trapezoid rule leaked into
 *   the harmonics). The power integrand pairs each current sample with V
 *   interpolated (cubic Lagrange) to the instant that current sample was
 *   taken. The interpolator's gain and the boxcar's gain at the measured
 *   frequency are divided back out. Harmonics 2..N are taken from what is
 *   left after removing the fundamental. Verified with !SYNTH: V, I and P
 *   within 4 ppm, PF to the 4th decimal at any angle, THD of a pure sine
 *   0.001 %, and a 20 % 3rd harmonic reads 20.000 %. With no voltage
 *   crossings (DC, no mains, nothing connected) the board falls back to
 *   nominal-length cycles, flagged +1, so current is still measured.
 *
 * ── Commands (newline terminated, USB serial or TCP) ────────────────────────
 *   !START / !STOP        stream $PWR on / all streams off
 *   !SMASK,<bits>         streams: 1 $PWR, 2 $CYC, 4 $HARM, 8 $AVG, 16 $INFO
 *   !CFG                  re-print $CFG $ADC $ACFG $NET
 *   !STAT                 one $INFO line now
 *   !HGET / !AVGW         one $HARM / one $AVG from the last window
 *   !RELAY,<0|1>          open / close the load relay (D5); close clears a trip
 *   !RLYINV,<0|1>         relay driver is active-low
 *   !VSCALE,<adcV>,<V>    !ISCALE,<adcV>,<A>[,<range>]
 *   !PHASECAL,<us>[,<range>]  current front end's extra delay vs voltage, us (0)
 *                         ISCALE, PHASECAL and IOFF set the active current
 *                         range, or the one named.
 *   !IRANGE,<0|1|A>       current range: fixed 0, fixed 1, or auto
 *   !IRINV,<0|1>          1 = D6 HIGH selects range 0
 *   !IRSETTLE,<ms>        settle time after a switch (default 20): those current
 *                         samples are replaced by the ones a cycle earlier, flagged
 *   !IRLIM,<up%>,<down%>  auto thresholds, % of the sensitive range's full
 *                         scale (default 90, 60)
 *   !CYCLES,<n>           mains cycles per window (default 12 -> 5 Hz at 60 Hz)
 *   !MAINS,<hz>           nominal mains frequency (default 60)
 *   !VMIN,<volts>         smallest V peak that counts as mains (default 5 % of VSCALE)
 *   !TRIP,<amps>          open the relay when a cycle's Irms exceeds this; 0 off
 *   !TRIPPK,<amps>        open the relay on any |i| sample above this; 0 off
 *   !DCREM,<0|1>          remove DC from each cycle / window (default 1)
 *   !ZERO                 average 0.2 s and keep it as the channel offsets:
 *                         V, then each current range in turn (~0.5 s)
 *   !VOFF,<uV> !IOFF,<uV>[,<range>]  set those offsets directly (ADC volts)
 *   !EZERO                reset the energy accumulator
 *   !VREF,<volts>         ADC reference (default 3.3)
 *   !HARMN,<n>            harmonics computed, 1..63 (default 40)
 *   !WAVE                 capture now (WPRE of it is from before the command)
 *   !WLEN,<sec>           capture length (default 0.5 s, max ~6 s at 8 kS/s)
 *   !WPRE,<0..1>          fraction of a capture taken before its trigger (0.25)
 *   !EVT,<dA>,<dV>        capture when a cycle's Irms / Vrms moves this far from
 *                         the previous window; 0 disables that trigger
 *   !RLYCAP,<0|1>         capture every relay close (inrush)
 *   !RATE,<hz>            ADC scan rate (default 80000)
 *   !DECIM,<n>            scans averaged per analysed sample (default 10)
 *   !BITS,<8|10|12|14|16> resolution (default 16)
 *   !SMP,<0-7>            sample time 1.5/2.5/8.5/16.5/32.5/64.5/387.5/810.5 cyc (4)
 *   !SE,<v>,<i>           1 = single-ended: ignore that channel's - pin
 *   !WIFI                 rejoin Wi-Fi (in the background; $NET reports)
 *   !SCAN                 Wi-Fi module firmware + visible networks ($SCAN, $AP)
 *   !SYNTH,<on>[,vrms,irms,deg,h3%,hz]  replace the ADC data with a synthetic
 *                         V/I pair (exact test of the maths and the link; the
 *                         ADC still paces it). !SYNTH,0 returns to the ADC.
 *
 * ── Output ──────────────────────────────────────────────────────────────────
 *   $PWR,vrms,irms,w,va,var,pf,hz,vpk,ipk,wh,t_s,n,fs,relay,flags,
 *        vthd,ithd,dpf,cycles,seq,irange
 *        t_s is sample-clock time (the Giga lesson: time from sample counts,
 *        never from millis()).
 *        flags: +1 no V crossings  +2 V clipped  +4 I clipped  +8 tripped
 *               +16 samples lost (gap)  +32 ADC error  +64 synthetic data
 *               +128 current range switched (settling, or a switch inside)
 *   $CYC,seq,vrms,irms,w,hz,ipk,flags
 *   $HARM,seq,H,hz,vthd,ithd,dpf,p1, then H x (vrms_h,vdeg_h,irms_h,ideg_h)
 *        magnitudes are RMS in V and A, corrected for the boxcar's droop.
 *        Phases are cosine-referenced, phi_h - h*phi_V1, in degrees, so I1's
 *        is minus the lag of the current.
 *   $AVG,seq,M,cycles,hz, M voltages, M currents (one averaged cycle, from
 *        the rising crossing)
 *   $EVT,reason,t_s,value         then the capture:
 *   $WAVE,n,dt_us,kv,bv,ki,bi,alpha,pre,reason,flags,seq
 *        then n x $WD,<v_raw>,<i_raw>, then $WEND.
 *        v = v_raw*kv + bv volts, i = i_raw*ki + bi amps. Current sample k
 *        was taken alpha samples after voltage sample k. i_raw is in units of
 *        the sensitive range's LSB on either range (bi = 0), so a capture that
 *        spans a range switch is still in one unit.
 *   $CFG,2,relay,smask
 *   $ADC,2,rate_req,rate_used,decim,fs,bits,smp,v_se,i_se,lag_us,rate_max,synth,lincal
 *        lincal 1 = both ADCs took their linearity calibration at the last start
 *   $ACFG,v_adc_fs,v_fs,i_adc_fs,i_fs,phasecal_us,cycles,mains,trip,dcrem,
 *         v_off_uv,i_off_uv,rly_inv,wave_len,2,vref,trippk,wpre,harmn,vmin,
 *         ev_di,ev_dv,rlycap
 *   $IRNG,range,auto,inv,settle_ms,up_pct,down_pct,switches,fs0_a,fs1_a,
 *         adc0,fs0,ioff0_uv,phasecal0_us,adc1,fs1,ioff1_uv,phasecal1_us
 *        sent on !CFG, on any range setting, and after every switch.
 *        fsN_a = range N's full scale in amps peak. $ACFG's current fields
 *        are the active range's.
 *   $NET,ip,port,rssi,wifi_status,ps,ssid  ip = "joining" while the join runs;
 *        ps = the radio's power-save mode after the join (0 = off, which
 *        this sketch sets; -1 = unknown)
 *   $INFO,rate_meas,gaps,resync,load_pct,loop_max_ms,backlog,txdrop_tcp,
 *         txdrop_ser,tcp_busy,heapmax,adcfix,uptime_s,wifi_rejoins,lost_ms,
 *         bulk_dropped
 *        gaps/lost_ms count only real losses, not deliberate restarts.
 *        rate_meas and loop_max_ms cover the time since the previous $INFO.
 *        load_pct includes time the RTOS gives other threads (Wi-Fi) while
 *        the loop is mid-task, so it reads high; loop_max_ms is what matters.
 *   $RLY,<0|1>   $TRIP,<reason>,<value>,<limit>   $ERR,<message>
 */

#include <Arduino_AdvancedAnalog.h>
#include <WiFi.h>
#include "USB/PluggableUSBSerial.h"   // _SerialUSB: the USBSerial behind Serial
#include "WhdSTAInterface.h"          // the Giga's Wi-Fi driver, for power-save
#include "whd_wifi_api.h"
#include "SDRAM.h"
#include "ac_types.h"
#include "adc_access.h"
#include "net_sink.h"

#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#else
#error "Copy arduino_secrets.example.h to arduino_secrets.h and put your Wi-Fi name/password in it"
#endif

#define FW_VERSION   "1.1"
#define BACKEND_GIGA 2
static const uint16_t CMD_PORT = 8080;

// ── board wiring ──────────────────────────────────────────────────────────────
#define RELAY_PIN 5
#define IRANGE_PIN 6      // current range select: LOW = range 0 (see !IRINV)
// A0..A6 can sit on ADC1 or ADC2 (A7 is ADC1 only), so any pins from A0..A6
// work here. Keep each pair's + on adcA and - on adcB, so the pair converts
// at one instant.
AdvancedADC     adcA(A0, A2);       // ADC1: rank 0 = V+, rank 1 = I+
AdvancedADC     adcB(A1, A3);       // ADC2: rank 0 = V-, rank 1 = I-
AdvancedADCDual adcDual(adcA, adcB);
static const int ADC_RANKS = 2;

// ── result flags ──────────────────────────────────────────────────────────────
#define FLG_NOCROSS  0x01
#define FLG_VCLIP    0x02
#define FLG_ICLIP    0x04
#define FLG_TRIP     0x08
#define FLG_GAP      0x10
#define FLG_ADCERR   0x20
#define FLG_SYNTH    0x40
#define FLG_IRANGE   0x80

// ── stream mask ───────────────────────────────────────────────────────────────
#define SM_PWR   0x01
#define SM_CYC   0x02
#define SM_HARM  0x04
#define SM_AVG   0x08
#define SM_INFO  0x10

// ── buffers ───────────────────────────────────────────────────────────────────
// The ring and the capture buffer live in the 8 MB SDRAM. The DMA pool and the
// Wi-Fi stack need the internal 512 KB.
#define RING_BITS  16
#define RING_N     (1UL << RING_BITS)        // 65536 samples: 8.2 s at 8 kS/s
#define RING_MASK  (RING_N - 1)
#define WAVE_MAX   49152UL                   // per capture, <= 3/4 of the ring
#define H_MAX      63
#define AVG_M      128
#define POOL_BYTES (64UL * 1024UL)           // DMA pool per ADC

static float*   g_rv = nullptr;   // decimated V, raw differential counts
static float*   g_ri = nullptr;   // decimated I, in amps (each scan scaled by its own range)
static uint8_t* g_rf = nullptr;   // FLG_* per sample (clip, gap, synth)
static float*   g_wv = nullptr;   // capture copy, so the ring can keep moving
static float*   g_wi = nullptr;

// ── ADC settings ──────────────────────────────────────────────────────────────
// AdvancedADC clocks the ADCs from per_ck (64 MHz) /1, and rev-V silicon
// halves it: 32 MHz. Measured 2026-11-04 (Giga_SimultaneousADC):
// one conversion = sample time + (bits/2 + 0.5) cycles.
static const double ADC_CLK_HZ = 32.0e6;
static const float  SMP_CYC[8] = {1.5f, 2.5f, 8.5f, 16.5f, 32.5f, 64.5f, 387.5f, 810.5f};
static const adc_sample_time_t SMP_T[8] = {
  AN_ADC_SAMPLETIME_1_5,  AN_ADC_SAMPLETIME_2_5,  AN_ADC_SAMPLETIME_8_5,
  AN_ADC_SAMPLETIME_16_5, AN_ADC_SAMPLETIME_32_5, AN_ADC_SAMPLETIME_64_5,
  AN_ADC_SAMPLETIME_387_5, AN_ADC_SAMPLETIME_810_5
};

static uint32_t g_rate_req = 80000;   // scans per second requested
static double   g_rate     = 80000;   // what the timer actually runs at
static uint16_t g_decim    = 10;
static uint8_t  g_bits     = 16;
static uint8_t  g_smp      = 4;       // 32.5 cycles: kinder to a source impedance
static bool     g_v_se = false, g_i_se = false;
static float    g_vref     = 3.3f;

// derived, by applyDerived()
static double   g_fs = 8000, g_dt = 1.0 / 8000;   // analysed sample rate
static double   g_lag_s  = 0;                      // I+ converts this long after V+
static int32_t  g_mid = 32768, g_clipLo = 0, g_clipHi = 65535;

// ── scaling / calibration ─────────────────────────────────────────────────────
static float g_v_adc_fs = 1.5f, g_v_fs = 170.0f;
static float g_v_off = 0.0f;                     // ADC volts
// current: one calibration per range (IRANGE_PIN picks the front end's range)
static float g_ir_adc[2] = {1.5f, 1.5f};         // "this many ADC volts differential
static float g_ir_fs[2]  = {25.0f, 2.5f};        //  = this many amps"
static float g_ir_off[2] = {0.0f, 0.0f};         // ADC volts
static float g_ir_pc[2]  = {0.0f, 0.0f};         // PHASECAL, us
static bool  g_dcrem = true;
static uint8_t g_cycles = 12;
static float g_mains_hz = 60.0f;
static float g_vmin = 8.5f;                      // volts peak
static uint8_t g_harmN = 40;

// derived: real = raw*K + B, alpha = I-sample lag in samples, Lagrange weights.
// Per current range: K, B, alpha, weights, and full scale in amps peak.
// g_alpha / g_W are the ones in use: processCycle loads the range of its cycle.
static double g_KV, g_BV, g_alpha, g_N0;
static double g_W[4];
static double g_KIr[2], g_BIr[2], g_alphaR[2], g_WR[2][4], g_irFsA[2];
static double g_kiRef = 1.0;                     // the sensitive range's |K|: $WAVE's unit

// ── relay / protection ────────────────────────────────────────────────────────
static bool  g_relay = false, g_rly_inv = false;
static float g_trip_a = 0.0f, g_trippk = 0.0f;
static bool  g_trip_latch = false;

// ── current range ─────────────────────────────────────────────────────────────
// g_irWant is what was asked for (a command, auto, a zero). serviceRange()
// moves the pin to it when no switch is in flight: g_irReq is the pin, g_irCur
// the range the incoming scans are scaled with. drainAdc() moves g_irCur over
// at the scan the switch happened on, from the DMA buffers' timestamps.
static uint8_t  g_irCur = 0, g_irReq = 0, g_irWant = 0;
static bool     g_irAuto = false, g_irInv = false;
static bool     g_irPending = false, g_irHistDone = false, g_irReport = false;
static uint32_t g_irSwUs = 0, g_irLastSwMs = 0, g_irSwitches = 0;
static float    g_irSettleMs = 20.0f, g_irUp = 0.90f, g_irDn = 0.60f;
static uint8_t  g_irDnCount = 0;
static IrHist   g_irHist[8];           // recent switch points, for the analysis's alpha
static uint8_t  g_irHistN = 0;
static uint8_t  g_zeroStep = 0;        // !ZERO in progress (auto range waits)

// ── run state ─────────────────────────────────────────────────────────────────
static uint8_t g_smask = 0;
static double  g_wh = 0.0;
static bool    g_adcOn = false;

// ── capture / events ──────────────────────────────────────────────────────────
static float g_wave_len_s = 0.5f, g_wpre = 0.25f;
static float g_ev_di = 0.0f, g_ev_dv = 0.0f;
static bool  g_rlycap = false;

// ── synthetic source ──────────────────────────────────────────────────────────
static bool  g_synth = false;
static float g_syn_vrms = 120.0f, g_syn_irms = 5.0f, g_syn_deg = 30.0f, g_syn_h3 = 0.0f, g_syn_hz = 60.0f;
static double g_synPh = 0.0;

// ── diagnostics ───────────────────────────────────────────────────────────────
static uint32_t g_gaps = 0, g_resyncs = 0, g_adcFixes = 0, g_dmaResets = 0, g_tcpBusy = 0;
static uint64_t g_lostUs = 0;          // sample time lost to gaps (not in the energy count)
static uint32_t g_lowDrops = 0;        // low-priority lines dropped to keep latency down
static uint32_t g_wifiReconnects = 0;
static uint32_t g_busyUs = 0, g_loopMaxUs = 0, g_lastLoopUs = 0;
static float    g_loadPct = 0.0f, g_loopMaxMs = 0.0f;
static uint32_t g_bufScans = 0, g_bufUs = 0, g_capBufs = 0, g_capFirstTs = 0, g_capLastTs = 0, g_prevTsA = 0;
static char     g_adcDiag[112] = "";

// ═════════════════════════════════════════════════════════════════════════════
//  output plumbing
// ═════════════════════════════════════════════════════════════════════════════

static TxRing     g_txSer, g_txTcp;
static LineBuf    g_ln;
static WiFiServer g_server(CMD_PORT);
static WiFiClient g_client;
static bool       g_clientOn = false, g_wifiOk = false;
static char       g_ipStr[20] = "none";
static int        g_wifiStatus = 0;     // last WiFi.status(): 3 = connected, 255 = no module
static int        g_wifiPs = -1;        // radio power-save mode after the join
static volatile bool g_wifiBusy = false, g_wifiDone = false, g_wifiDisc = false;   // join thread, see wifiTask()
static volatile int  g_wifiResult = 0;
// On this core Serial is a UART wrapper around _SerialUSB (a USBSerial), and
// `if (Serial)` runs delay(1) before it answers (operator bool in
// PluggableUSBSerial.h). Asked once per emitted line, that throttled the loop
// to a few hundred lines a second and the ADC overran (2026-11-08: 117 ms
// loops, lost buffers). So the USB state is read once per loop instead.
static bool       g_serOn = false;

// Hand the line in g_ln to every connected output, then clear it.
static void emit() {
  if (g_ln.size()) {
    if (g_serOn)    g_txSer.put(g_ln.data(), g_ln.size());
    if (g_clientOn) g_txTcp.put(g_ln.data(), g_ln.size());
  }
  g_ln.reset();
}
// Bulk lines ($CYC $HARM $AVG): queued only while that output is keeping up.
// On a slow link the ring would otherwise hold seconds of backlog, and every
// $PWR and command reply would wait behind it. Dropped lines are counted.
// 12 KB, half the ring: one window's $HARM + $AVG is a ~3.5 KB burst, and a
// 4 KB limit shed it on every slow Wi-Fi round trip (2026-11-08: 15-25 % of
// bulk lines lost at 12 KB/s on a link that carries 85 KB/s). At Wi-Fi rates
// 12 KB drains in ~0.2 s, so $PWR still is not held back for long.
static const size_t LOW_BACKLOG = 12288;
// a capture's $WD lines keep the queue under this, so live and bulk lines
// still get through while it streams
static const size_t CAP_BACKLOG = 3072;
static void emitLow() {
  if (g_ln.size()) {
    if (g_serOn)    { if (g_txSer.used() < LOW_BACKLOG) g_txSer.put(g_ln.data(), g_ln.size()); else g_lowDrops++; }
    if (g_clientOn) { if (g_txTcp.used() < LOW_BACKLOG) g_txTcp.put(g_ln.data(), g_ln.size()); else g_lowDrops++; }
  }
  g_ln.reset();
}
static void emitStr(const char* s) { g_ln.reset(); g_ln.println(s); emit(); }
static void emitErr(const char* s) { g_ln.reset(); g_ln.print("$ERR,"); g_ln.println(s); emit(); }

// Unsent bytes in the slowest connected output (paces the $WD sender).
static size_t sinkBacklog() {
  size_t r = 0;
  if (g_serOn)    r = max(r, g_txSer.used());
  if (g_clientOn) r = max(r, g_txTcp.used());
  return r;
}
static bool anySink() { return g_serOn || g_clientOn; }

// Non-blocking drain of both rings. The socket timeout is 0, so write() takes
// what lwIP has room for and returns at once; the rest stays queued.
static void serviceTx() {
  const uint8_t* p;
  if (g_serOn) {
    size_t n = g_txSer.peek(&p);
    // send_nb, not write(): USBSerial::write() blocks until each packet is out,
    // so a slow host would stall the loop and overrun the ADC
    if (n) { uint32_t got = 0; _SerialUSB.send_nb((uint8_t*)p, n, &got, true); g_txSer.consume(got); }
  } else {
    g_txSer.clear();
  }
  if (g_clientOn) {
    for (int k = 0; k < 6; k++) {
      size_t n = g_txTcp.peek(&p);
      if (!n) break;
      if (n > 1436) n = 1436;
      size_t w = g_client.write(p, n);
      if (w == 0) { g_tcpBusy++; break; }
      g_txTcp.consume(w);
      if (w < n) break;
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  ADC: start/stop, the stuck-DMA repair, buffer pairing
// ═════════════════════════════════════════════════════════════════════════════

static uint32_t toResolution(int bits) {
  if (bits <= 8)  return AN_RESOLUTION_8;
  if (bits <= 10) return AN_RESOLUTION_10;
  if (bits <= 12) return AN_RESOLUTION_12;
  if (bits <= 14) return AN_RESOLUTION_14;
  return AN_RESOLUTION_16;
}
static double rankTime() { return (SMP_CYC[g_smp] + g_bits / 2.0 + 0.5) / ADC_CLK_HZ; }
static uint32_t maxRate() { return (uint32_t)(0.9 / (ADC_RANKS * rankTime())); }

// The timer clock as Arduino_AdvancedAnalog computes it (HALConfig.cpp
// hal_tim_freq). With PSC/ARR read back, this gives the rate the timer really
// runs at. hal_tim_config rounds the divider, so 76.8 kHz would really be
// 76.82 kHz, and every frequency reading would be off by that much.
static uint32_t timClock(TIM_TypeDef* t) {
  if (((uint32_t)t & ~0xFFFFUL) == D2_APB1PERIPH_BASE)
    return HAL_RCC_GetPCLK1Freq() * ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) ? 2 : 1);
  return HAL_RCC_GetPCLK2Freq() * ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) ? 2 : 1);
}
static double actualRate() {
  AdcDescrMirror* d = adcDescr(adcA);
  if (!adcDescrValid(d) || !d->tim.Instance) return (double)g_rate_req;
  TIM_TypeDef* t = d->tim.Instance;
  return (double)timClock(t) / ((double)(t->PSC + 1) * (double)(t->ARR + 1));
}

static size_t largestFreeBlock() {
  size_t lo = 0, hi = 512 * 1024;
  while (hi - lo > 256) {
    size_t mid = (lo + hi) / 2; void* p = malloc(mid);
    if (p) { free(p); lo = mid; } else hi = mid;
  }
  return lo;
}

// Ported from Giga_RemoteController_WiFi (2026-11-05): a stop can leave
// ADC2's DMA stream stuck with EN=1 and its HAL handle in ERROR, and every
// later begin() fails until reset. adcSanitize() puts a stopped ADC and its
// stream back to the state begin() expects. It writes only after
// adcDescrValid() confirms the mirrored library struct.
static const uint32_t ADC_ERR_STATE = HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_CONFIG | HAL_ADC_STATE_ERROR_DMA;
static bool adcNeedsFix(AdvancedADC& a) {
  AdcDescrMirror* d = adcDescr(a);
  if (!adcDescrValid(d)) return false;
  DMA_Stream_TypeDef* st = (DMA_Stream_TypeDef*)d->dma.Instance;
  return (d->adc.State & ADC_ERR_STATE) || (d->adc.State & (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY)) ||
         d->adc.Lock != HAL_UNLOCKED || (d->adc.Instance->CR & (ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_JADSTART)) ||
         (st->CR & DMA_SxCR_EN) || d->dma.State == HAL_DMA_STATE_BUSY || d->dma.State == HAL_DMA_STATE_ERROR ||
         d->dma.State == HAL_DMA_STATE_ABORT || d->dma.Lock != HAL_UNLOCKED;
}
static void adcSanitize(AdvancedADC& a) {
  if (!adcNeedsFix(a)) return;
  AdcDescrMirror* d = adcDescr(a);
  snprintf(g_adcDiag, sizeof(g_adcDiag), "ADC%d st=%lX cr=%lX dma=%X",
           d->adc.Instance == ADC1 ? 1 : 2, (unsigned long)d->adc.State,
           (unsigned long)d->adc.Instance->CR, (unsigned)d->dma.State);
  ADC_TypeDef* r = d->adc.Instance;
  DMA_Stream_TypeDef* st = (DMA_Stream_TypeDef*)d->dma.Instance;
  const uint32_t RS = ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART |
                      ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN;   // write-1 bits: mask on RMW
  uint32_t t0 = micros();
  if (r->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) {
    r->CR = (r->CR & ~RS) | ADC_CR_ADSTP | ADC_CR_JADSTP;
    while ((r->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) && micros() - t0 < 2000) {}
  }
  if (r->CR & ADC_CR_ADEN) {
    r->CR = (r->CR & ~RS) | ADC_CR_ADDIS;
    while ((r->CR & ADC_CR_ADEN) && micros() - t0 < 4000) {}
  }
  st->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  st->FCR &= ~DMA_SxFCR_FEIE;
  t0 = micros();
  while ((st->CR & DMA_SxCR_EN) && micros() - t0 < 5000) {}
  if (st->CR & DMA_SxCR_EN) {
    DMA_Stream_TypeDef* const S[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
                                      DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
    bool others = false;
    for (DMA_Stream_TypeDef* o : S)
      if (o != DMA1_Stream1 && o != DMA1_Stream2 && (o->CR & DMA_SxCR_EN)) others = true;
    if (!others) {
      RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST; __DSB();
      RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA1RST; __DSB();
      NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn); NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
      g_dmaResets++;
      AdcDescrMirror* o1 = adcDescr(adcA); AdcDescrMirror* o2 = adcDescr(adcB);
      if (adcDescrValid(o1)) { o1->dma.State = HAL_DMA_STATE_RESET; o1->dma.ErrorCode = 0; o1->dma.Lock = HAL_UNLOCKED; }
      if (adcDescrValid(o2)) { o2->dma.State = HAL_DMA_STATE_RESET; o2->dma.ErrorCode = 0; o2->dma.Lock = HAL_UNLOCKED; }
    }
  }
  const uint32_t off = (st == DMA1_Stream1) ? 6 : (st == DMA1_Stream2) ? 16 : 22;
  DMA1->LIFCR = 0x3DUL << off;
  d->adc.State = HAL_ADC_STATE_RESET; d->adc.ErrorCode = HAL_ADC_ERROR_NONE; d->adc.Lock = HAL_UNLOCKED;
  d->dma.State = HAL_DMA_STATE_RESET; d->dma.ErrorCode = HAL_DMA_ERROR_NONE; d->dma.Lock = HAL_UNLOCKED;
  g_adcFixes++;
}

static void adcStop() {
  if (g_adcOn) adcDual.stop();
  g_adcOn = false;
}

static void markGap();
static void applyDerived();

// The H7 ADC has a linearity self-calibration (ADCALLIN), and AdvancedADC
// never runs it: hal_adc_config() asks for ADC_CALIB_OFFSET only. Without it,
// measured 2026-11-08 on a bench PSU through an isolated front end, every
// channel read +-1 mV (about 20 LSB at 16 bits) off a straight line in a
// repeatable, non-monotonic pattern, with one 6.5 mV spot. It runs here on a
// handle of our own, with the ADC disabled, just before AdvancedADC starts.
// Its offset-only calibration then leaves the linearity factors in place, and
// stop() only disables the ADC (no deep power-down), so they are kept.
static bool g_linCal = false;
static bool adcLinearityCal(ADC_TypeDef* inst) {
  ADC_HandleTypeDef h;
  memset(&h, 0, sizeof(h));
  h.Instance = inst;
  h.Init.Resolution = g_bits >= 16 ? ADC_RESOLUTION_16B : g_bits >= 14 ? ADC_RESOLUTION_14B
                    : g_bits >= 12 ? ADC_RESOLUTION_12B : g_bits >= 10 ? ADC_RESOLUTION_10B
                    : ADC_RESOLUTION_8B;
  h.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;          // as AdvancedADC
  h.Init.ScanConvMode = ADC_SCAN_DISABLE;
  h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  h.Init.LowPowerAutoWait = DISABLE;
  h.Init.ContinuousConvMode = DISABLE;
  h.Init.NbrOfConversion = 1;
  h.Init.DiscontinuousConvMode = DISABLE;
  h.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  h.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  h.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  h.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  h.Init.OversamplingMode = DISABLE;
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);
  __HAL_RCC_ADC12_CLK_ENABLE();
  if (inst->CR & ADC_CR_ADEN) return false;               // must be disabled to calibrate
  if (HAL_ADC_Init(&h) != HAL_OK) return false;
  return HAL_ADCEx_Calibration_Start(&h, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) == HAL_OK;
}

static bool adcStart() {
  adcStop();
  uint32_t lim = maxRate();
  if (g_rate_req > lim) g_rate_req = lim;
  if (g_rate_req < 1000) g_rate_req = 1000;

  // ~5 ms per buffer, a whole number of boxcars, and as deep a pool as
  // POOL_BYTES allows (at least 8). At the defaults that is 400 scans x 40
  // buffers = 200 ms of slack before a slow loop costs samples.
  uint32_t ns = g_rate_req / 200;
  if (ns < 32) ns = 32;
  ns = (ns / g_decim) * g_decim;
  if (ns < g_decim) ns = g_decim;
  size_t depth = POOL_BYTES / (ns * ADC_RANKS * sizeof(Sample));
  if (depth > 48) depth = 48;
  if (depth < 8)  depth = 8;

  for (int attempt = 0; attempt < 2 && !g_adcOn; attempt++) {
    adcDual.stop();
    adcSanitize(adcA); adcSanitize(adcB);
    g_linCal = adcLinearityCal(ADC1) && adcLinearityCal(ADC2);
    g_adcOn = adcDual.begin(toResolution(g_bits), g_rate_req, ns, depth, SMP_T[g_smp]) != 0;
  }
  if (!g_adcOn) {
    adcDual.stop();
    g_ln.reset(); g_ln.print("$ERR,ADC begin failed,heap "); g_ln.print(largestFreeBlock());
    g_ln.print(','); g_ln.println(g_adcDiag); emit();
    return false;
  }
  g_rate = actualRate();
  g_bufScans = ns;
  g_bufUs = (uint32_t)(ns * 1.0e6 / g_rate);
  g_capBufs = 0; g_prevTsA = 0;
  applyDerived();
  markGap();      // a restart is a discontinuity, by definition
  return true;
}

static inline bool adcAvailable() { return g_adcOn && adcA.available() && adcB.available(); }

// Each ADC fills its own buffers on the same triggers, so a matching pair
// completes microseconds apart. If the reader falls behind, the library reuses
// a full buffer instead of queuing a new one, and decides that per ADC. So the
// pair is checked by timestamp and re-paired if the sides drifted apart;
// otherwise V+ from one moment would be subtracted from V- of another.
static ScanBuf adcRead() {
  DMABuffer<Sample>* a = &adcA.read();
  DMABuffer<Sample>* b = &adcB.read();
  bool gap = false;
  for (int k = 0; k < 4; k++) {
    int32_t d = (int32_t)(a->timestamp() - b->timestamp());
    if ((uint32_t)abs(d) <= g_bufUs / 2) break;
    g_resyncs++; gap = true;
    if (d < 0) { a->release(); a = &adcA.read(); }
    else       { b->release(); b = &adcB.read(); }
  }
  if (a->get_flags(DMA_BUFFER_DISCONT) || b->get_flags(DMA_BUFFER_DISCONT)) gap = true;
  uint32_t ts = a->timestamp();
  // DISCONT catches every overwrite. This catches a whole buffer missing
  // between two we did see, with room for interrupt-latency jitter. Either
  // way the time lost is what the energy count is missing.
  if (g_prevTsA) {
    uint32_t dts = (uint32_t)(ts - g_prevTsA);
    if (dts > (g_bufUs * 5) / 2) gap = true;
    if (gap && dts > g_bufUs) g_lostUs += dts - g_bufUs;
  }
  g_prevTsA = ts;
  if (g_capBufs++ == 0) g_capFirstTs = ts;
  g_capLastTs = ts;
  ScanBuf s = { a, b, (int)(min(a->size(), b->size()) / ADC_RANKS), gap, ts };
  return s;
}
static inline void adcRelease(ScanBuf& s) { s.a->release(); s.b->release(); }

// Scans per second over the buffers read since the last call (it restarts the
// count), so a gap shows up in the interval it happened in, not averaged away.
static uint32_t measuredRate() {
  uint32_t r = 0;
  if (g_capBufs >= 3 && g_capLastTs != g_capFirstTs)
    r = (uint32_t)((uint64_t)(g_capBufs - 1) * g_bufScans * 1000000ULL / (uint32_t)(g_capLastTs - g_capFirstTs));
  g_capBufs = 0;
  return r;
}

// ═════════════════════════════════════════════════════════════════════════════
//  derived constants
// ═════════════════════════════════════════════════════════════════════════════

static inline float lsb() { return g_vref / (float)(1UL << g_bits); }

// Cubic Lagrange weights for nodes at -1, 0, +1, +2, evaluated at a. This is
// what places the voltage at the instant each current sample was taken. A
// straight chord shaves the amplitude by cos(pi*f*dt); a cubic through four
// points is far closer, and the residual is divided out by interpGain().
static void lagrange4(double a, double* w) {
  w[0] = -a * (a - 1.0) * (a - 2.0) / 6.0;
  w[1] =  (a + 1.0) * (a - 1.0) * (a - 2.0) / 2.0;
  w[2] = -(a + 1.0) * a * (a - 2.0) / 2.0;
  w[3] =  (a + 1.0) * a * (a - 1.0) / 6.0;
}
// Magnitude response of that interpolator at theta radians per sample.
static double interpGain(const double* w, double theta) {
  double c1 = cos(theta), s1 = sin(theta);
  double c2 = cos(2.0 * theta), s2 = sin(2.0 * theta);
  double re = w[0] * c1 + w[1] + w[2] * c1 + w[3] * c2;
  double im = -w[0] * s1 + w[2] * s1 + w[3] * s2;
  return sqrt(re * re + im * im);
}

static void resetAnalyzer();

static void applyDerived() {
  g_fs = g_rate / (double)g_decim;
  g_dt = 1.0 / g_fs;
  g_lag_s = rankTime();
  uint32_t full = (1UL << g_bits) - 1UL;
  g_mid = (int32_t)(1UL << (g_bits - 1));
  g_clipLo = (int32_t)(full / 256);
  g_clipHi = (int32_t)(full - full / 256);

  double sv = (double)g_v_fs / (double)g_v_adc_fs;
  g_KV = (double)lsb() * sv;  g_BV = -(double)g_v_off * sv;
  for (int r = 0; r < 2; r++) {
    double si = (double)g_ir_fs[r] / (double)g_ir_adc[r];
    g_KIr[r] = (double)lsb() * si;  g_BIr[r] = -(double)g_ir_off[r] * si;
    g_irFsA[r] = 0.5 * (double)g_vref * fabs(si);
    double al = (g_lag_s - (double)g_ir_pc[r] * 1.0e-6) / g_dt;
    // +-1 sample keeps every read inside processCycle's index budget (and is
    // +-125 us, +-2.7 deg at 60 Hz, at the defaults: far more than a CT needs)
    if (al < -1.0) al = -1.0;
    if (al >  1.0) al =  1.0;
    g_alphaR[r] = al;
    lagrange4(al, g_WR[r]);
  }
  g_kiRef = min(fabs(g_KIr[0]), fabs(g_KIr[1]));
  if (!(g_kiRef > 0.0)) g_kiRef = 1.0;
  g_alpha = g_alphaR[g_irCur];
  memcpy(g_W, g_WR[g_irCur], sizeof(g_W));

  g_N0 = g_fs / (double)g_mains_hz;
  int hmax = (int)(g_N0 / 2.0) - 1;
  if (hmax > H_MAX) hmax = H_MAX;
  if (g_harmN > hmax) g_harmN = (uint8_t)max(hmax, 1);
}

// ═════════════════════════════════════════════════════════════════════════════
//  acquisition: DMA buffers -> boxcar -> ring
// ═════════════════════════════════════════════════════════════════════════════

static uint64_t g_wr = 0;          // decimated samples written, ever
static uint64_t g_validFrom = 0;   // first sample after the latest gap
static int64_t  g_accV = 0;
static double   g_accIA = 0.0;     // current, already in amps
static uint16_t g_accN = 0;
static uint8_t  g_accF = 0, g_pendF = 0;
static double   g_imean = 0.0;     // last cycle's mean current, for the peak trip

static void tripRelay(const char* why, float value, float limit);

// A break in the sample stream. Deliberate ones (a restart, a source change)
// just restart the analyser. A loss is also counted in $INFO gaps.
static void markGap() {
  g_accV = 0; g_accIA = 0.0; g_accN = 0; g_accF = 0;
  g_pendF |= FLG_GAP;
  g_validFrom = g_wr;
  resetAnalyzer();
}

static inline uint8_t irSensitive() { return g_irFsA[0] <= g_irFsA[1] ? 0 : 1; }
static inline bool irTwoRanges() {
  return fabs(g_irFsA[0] - g_irFsA[1]) > 1.0e-6 * max(g_irFsA[0], g_irFsA[1]);
}
// auto may act: on, idle, and the two ranges really differ
static inline bool irAutoIdle() {
  return g_irAuto && !g_irPending && g_irWant == g_irReq && !g_zeroStep && irTwoRanges();
}

static inline void pushSample() {
  uint32_t idx = (uint32_t)g_wr & RING_MASK;
  float v = (float)g_accV / (float)g_decim;
  float i = (float)(g_accIA / (double)g_decim);
  uint8_t f = g_accF | g_pendF | (g_synth ? FLG_SYNTH : 0);
  // Still settling after a range switch: the front end reads garbage here
  // (measured 2026-11-08 with a relay range switch: 3-5 ms of the old path on
  // the new scale, then up to 8 ms of contact bounce saturating the amplifier
  // at ~60x the real current). The sample is replaced by the one a nominal
  // mains cycle earlier: exact for DC, close for steady AC, and the RMS, the
  // energy, the trips and auto range never see the spike. It stays flagged.
  const uint32_t back = (uint32_t)lround(g_N0);
  if ((f & FLG_IRANGE) && back > 0 && g_wr >= g_validFrom + back) {
    i = g_ri[(uint32_t)(g_wr - back) & RING_MASK];
    f &= (uint8_t)~FLG_ICLIP;
  }
  g_rv[idx] = v;
  g_ri[idx] = i;
  g_rf[idx] = f;
  g_pendF = 0;
  g_wr++;
  g_accV = 0; g_accIA = 0.0; g_accN = 0; g_accF = 0;

  if (g_trippk > 0.0f && !g_trip_latch) {
    float ia = fabsf(i - (float)g_imean);
    if (ia > g_trippk) tripRelay("PEAK", ia, g_trippk);
  }
  // auto range up: at once, on the first sample near the top of the
  // sensitive range (or clipped)
  if (g_irReq == irSensitive() && irAutoIdle() &&
      ((f & FLG_ICLIP) || fabsf(i) > g_irUp * (float)g_irFsA[g_irReq]))
    g_irWant = 1 - g_irReq;
}

static void irHistPush(uint64_t idx, uint8_t r) {
  if (g_irHistN == 8) { memmove(g_irHist, g_irHist + 1, 7 * sizeof(IrHist)); g_irHistN = 7; }
  g_irHist[g_irHistN].idx = idx; g_irHist[g_irHistN].r = r; g_irHistN++;
}
// the range sample k was taken on (for its phase delay; the scale is already applied)
static uint8_t rangeAt(uint64_t k) {
  for (int j = (int)g_irHistN - 1; j >= 0; j--) if (g_irHist[j].idx <= k) return g_irHist[j].r;
  return g_irHistN ? g_irHist[0].r : g_irCur;
}
// In an ns-scan buffer whose last scan was d us after an event, scan k was at
// d - (ns-1-k)*scanUs: the first scan at or after the event.
static int scanIndexAfter(int ns, double d, double scanUs) {
  double k = ceil((double)(ns - 1) - d / scanUs);
  if (k < 0.0) return 0;
  if (k > (double)ns) return ns;
  return (int)k;
}

static void drainAdc() {
  uint32_t t0 = micros();
  // the synthetic source: V in raw counts, counts = (real/scale + off)/lsb.
  // I straight in amps: an ideal front end, the same on either range.
  double syAv = 0, syAi = 0, syVo = 0, syStep = 0, syLag = 0, syPhi = 0;
  if (g_synth) {
    double sv = (double)g_v_fs / g_v_adc_fs;
    syAv = g_syn_vrms * M_SQRT2 / sv / lsb();
    syAi = g_syn_irms * M_SQRT2 / sqrt(1.0 + (double)g_syn_h3 * g_syn_h3 * 1.0e-4);
    syVo = g_v_off / lsb();
    syStep = 2.0 * M_PI * g_syn_hz / g_rate;
    syLag  = 2.0 * M_PI * g_syn_hz * g_lag_s;     // I+ really is converted later
    syPhi  = g_syn_deg * M_PI / 180.0;
  }

  const double scanUs = 1.0e6 / g_rate;
  int guard = 0;
  while (adcAvailable() && guard++ < 48) {
    ScanBuf s = adcRead();
    if (s.gap && g_wr) { g_gaps++; markGap(); }
    const Sample* pa = s.a->data();
    const Sample* pb = s.b->data();
    // A range switch in flight: scans from kSw on were taken on the new range,
    // and up to kSet the front end was still settling. Buffers filled before
    // the switch are still coming out of the pool, so this goes by each
    // buffer's own timestamp, not by when it is read.
    int kSw = s.ns, kSet = s.ns;
    if (g_irPending) {
      double d = (double)(int32_t)(s.ts - g_irSwUs);   // switch -> this buffer's last scan, us
      kSw  = scanIndexAfter(s.ns, d, scanUs);
      kSet = scanIndexAfter(s.ns, d - (double)g_irSettleMs * 1000.0, scanUs);
    }
    const double Ka = g_KIr[g_irCur], Ba = g_BIr[g_irCur];
    const double Kb = g_KIr[g_irReq], Bb = g_BIr[g_irReq];
    for (int k = 0; k < s.ns; k++) {
      int32_t vp = pa[2 * k], ip = pa[2 * k + 1];
      int32_t vn = pb[2 * k], in = pb[2 * k + 1];
      uint8_t f = 0;
      if (vp <= g_clipLo || vp >= g_clipHi || (!g_v_se && (vn <= g_clipLo || vn >= g_clipHi))) f |= FLG_VCLIP;
      if (ip <= g_clipLo || ip >= g_clipHi || (!g_i_se && (in <= g_clipLo || in >= g_clipHi))) f |= FLG_ICLIP;
      int32_t v = vp - (g_v_se ? g_mid : vn);
      int32_t i = ip - (g_i_se ? g_mid : in);
      double ia = (k < kSw) ? (double)i * Ka + Ba : (double)i * Kb + Bb;
      if (g_synth) {
        float ph = (float)g_synPh;
        g_synPh += syStep;
        if (g_synPh >= 2.0 * M_PI) g_synPh -= 2.0 * M_PI;
        float pi_ = ph + (float)(syLag - syPhi);
        v = (int32_t)lrintf((float)syAv * sinf(ph) + (float)syVo);
        ia = syAi * (double)(sinf(pi_) + g_syn_h3 * 0.01f * sinf(3.0f * pi_));
        f = 0;
      }
      if (k >= kSw) {
        if (k < kSet) f |= FLG_IRANGE;
        if (!g_irHistDone) { irHistPush(g_wr, g_irReq); g_irHistDone = true; }
      }
      g_accV += v; g_accIA += ia; g_accF |= f;
      if (++g_accN >= g_decim) pushSample();
    }
    // settled inside this buffer: the switch is done
    if (g_irPending && kSet < s.ns) { g_irCur = g_irReq; g_irPending = false; g_irReport = true; }
    adcRelease(s);
  }
  g_busyUs += micros() - t0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  analysis: crossings -> cycles -> windows
// ═════════════════════════════════════════════════════════════════════════════

static inline double vAt(uint64_t k) { return (double)g_rv[(uint32_t)k & RING_MASK] * g_KV + g_BV; }
static inline double iAt(uint64_t k) { return (double)g_ri[(uint32_t)k & RING_MASK]; }   // amps
// cubic (4-point Lagrange) interpolation at a fractional index, for the
// averaged cycle; reads floor(x)-1 .. floor(x)+2
static inline float vCub(double x) {
  uint64_t k = (uint64_t)x; double w[4]; lagrange4(x - (double)k, w);
  return (float)(w[0] * vAt(k - 1) + w[1] * vAt(k) + w[2] * vAt(k + 1) + w[3] * vAt(k + 2));
}
static inline float iCub(double x) {
  uint64_t k = (uint64_t)x; double w[4]; lagrange4(x - (double)k, w);
  return (float)(w[0] * iAt(k - 1) + w[1] * iAt(k) + w[2] * iAt(k + 1) + w[3] * iAt(k + 2));
}

// crossing detector
static uint64_t g_scan = 0;
static double   g_cycStart = -1.0;
static bool     g_cycReal = false, g_armed = false;
static double   g_cand = -1.0;
static double   g_base = 0.0;
static float    g_lastVpk = 0.0f;

// window accumulators
static SpanSums g_win;
static uint16_t g_winCycles = 0, g_winReal = 0;
static double   g_winRealL = 0.0;
static double   g_wVr[H_MAX + 1], g_wVi[H_MAX + 1], g_wIr[H_MAX + 1], g_wIi[H_MAX + 1];
static double   g_wVp[H_MAX + 1], g_wIp[H_MAX + 1];   // L-weighted |phasor|^2 (power average)
static float    g_avgAccV[AVG_M], g_avgAccI[AVG_M];
static uint16_t g_avgAccN = 0;
static double   g_winEnd = 0.0;
static uint32_t g_seq = 0, g_cycSeq = 0;

// last window, kept for !HGET / !AVGW and as the event reference
static float    g_hVr[H_MAX + 1], g_hVd[H_MAX + 1], g_hIr[H_MAX + 1], g_hId[H_MAX + 1];
static uint8_t  g_hN = 0;
static float    g_hHz = 0, g_hVthd = 0, g_hIthd = 0, g_hDpf = 0, g_hP1 = 0;
static float    g_avgV[AVG_M], g_avgI[AVG_M];
static uint16_t g_avgN = 0;
static float    g_avgHz = 0;
static bool     g_haveRef = false;
static float    g_refVrms = 0, g_refIrms = 0;

static void clearWindow() {
  memset(&g_win, 0, sizeof(g_win));
  g_win.vmax = g_win.imax = -1e30f;
  g_win.vmin = g_win.imin = 1e30f;
  g_winCycles = g_winReal = 0; g_winRealL = 0.0;
  memset(g_wVr, 0, sizeof(g_wVr)); memset(g_wVi, 0, sizeof(g_wVi));
  memset(g_wIr, 0, sizeof(g_wIr)); memset(g_wIi, 0, sizeof(g_wIi));
  memset(g_wVp, 0, sizeof(g_wVp)); memset(g_wIp, 0, sizeof(g_wIp));
  memset(g_avgAccV, 0, sizeof(g_avgAccV)); memset(g_avgAccI, 0, sizeof(g_avgAccI));
  g_avgAccN = 0;
}

static void resetAnalyzer() {
  g_cycStart = -1.0; g_cycReal = false; g_armed = false; g_cand = -1.0;
  if (g_scan < g_validFrom + 3) g_scan = g_validFrom + 3;
  g_win.flags |= FLG_GAP;
}

// Gain of the DECIM-scan boxcar at theta radians per analysed sample:
// sin(theta/2) / (D sin(theta/2D)). At the defaults it is 0.99991 at 60 Hz.
// Measured 2026-11-08 with !SYNTH: Vrms and Irms read exactly that much low
// (-0.009 %) and P twice that, so it is divided out at the measured frequency.
static double boxcarGain(double theta) {
  if (g_decim <= 1 || theta <= 0.0) return 1.0;
  double d = sin(theta / (2.0 * g_decim));
  return d > 0.0 ? sin(theta / 2.0) / ((double)g_decim * d) : 1.0;
}

// RMS, power etc. from span sums. theta = radians per sample at the measured
// frequency; 0 (no crossings) corrects at the nominal mains frequency instead.
static void reduceSums(const SpanSums& s, double theta, PowerResult& r) {
  double L = s.L > 0 ? s.L : 1.0;
  double mv = s.sv / L, mi = s.si / L;
  double vv = s.svv / L, ii = s.sii / L, vi = s.svi / L;
  if (g_dcrem) { vv -= mv * mv; ii -= mi * mi; vi -= mv * mi; }
  if (theta <= 0.0) theta = 2.0 * M_PI * g_mains_hz * g_dt;
  double gain = interpGain(g_W, theta);
  if (gain < 0.5) gain = 1.0;
  double bg = boxcarGain(theta);
  if (bg < 0.5) bg = 1.0;
  r.vrms = vv > 0 ? (float)(sqrt(vv) / bg) : 0.0f;
  r.irms = ii > 0 ? (float)(sqrt(ii) / bg) : 0.0f;
  r.p    = (float)(vi / (gain * bg * bg));
  r.s    = r.vrms * r.irms;
  double qq = (double)r.s * r.s - (double)r.p * r.p;
  r.q  = qq > 0 ? (float)sqrt(qq) : 0.0f;
  r.pf = r.s > 1e-9f ? r.p / r.s : 0.0f;
  r.vdc = (float)mv; r.idc = (float)mi;
  float c = g_dcrem ? (float)mv : 0.0f, ci = g_dcrem ? (float)mi : 0.0f;
  r.vpk = max(s.vmax - c, c - s.vmin);
  r.ipk = max(s.imax - ci, ci - s.imin);
  r.hz = 0.0f;
}

static void finishWindow();
static void onCycle(const PowerResult& r, double a, double b, uint8_t flags);

// ∫ from 0 to u of the cubic Lagrange basis polynomial j, nodes -1, 0, 1, 2.
static inline double lagInt(int j, double u) {
  const double u2 = u * u, u3 = u2 * u, u4 = u3 * u;
  switch (j) {
    case 0:  return -(u4 / 4.0 - u3 + u2) / 6.0;
    case 1:  return (u4 / 4.0 - 2.0 * u3 / 3.0 - u2 / 2.0 + 2.0 * u) / 2.0;
    case 2:  return -(u4 / 4.0 - u3 / 3.0 - u2) / 2.0;
    default: return (u4 / 4.0 - u2 / 2.0) / 6.0;
  }
}

// Weight of node k in the integral over [a, b] (fractional ends) of the
// piecewise cubic through the samples: each segment [s, s+1] is integrated
// using the cubic through s-1 .. s+2. Interior nodes come out at exactly 1;
// only the three nodes nearest each end differ.
//
// Why not the trapezoid rule (what ac_power_monitor uses): its end error is
// O(h^2), and here it multiplies a harmonic's own phase ramp, so it grows with
// the harmonic number squared. Measured 2026-11-08 with !SYNTH, a pure sine
// read 0.1-0.2 % current THD whenever it was not in phase with V (its value at
// the cycle ends was not zero). This rule is O(h^4).
static double nodeWeight(int64_t k, int64_t ka, int64_t kb, double a, double b) {
  if (k >= ka + 3 && k <= kb - 2) return 1.0;
  double w = 0.0;
  for (int64_t s = k - 2; s <= k + 1; s++) {
    if (s < ka || s > kb) continue;
    double u0 = a - (double)s; if (u0 < 0.0) u0 = 0.0;
    double u1 = b - (double)s; if (u1 > 1.0) u1 = 1.0;
    if (u1 <= u0) continue;
    const int j = (int)(k - s + 1);
    w += lagInt(j, u1) - lagInt(j, u0);
  }
  return w;
}

// One span [a, b] of the sample stream (fractional indices), usually one mains
// cycle between two rising crossings, integrated to its fractional ends. The
// weights reach node floor(a)-1 .. floor(b)+2 and the power integrand reads one
// more each side, so this reads samples floor(a)-2 .. floor(b)+4; the detector
// only hands over spans whose samples are all in the ring.
static void processCycle(double a, double b, uint8_t flags) {
  double L = b - a;
  if (L < 2.0) return;
  const int64_t ka = (int64_t)floor(a), kb = (int64_t)floor(b);
  const bool real = !(flags & FLG_NOCROSS);
  // the current front end's phase delay is that of the range the cycle was
  // measured on; a cycle with a switch inside is flagged
  const uint8_t rr = rangeAt((uint64_t)ka);
  g_alpha = g_alphaR[rr];
  memcpy(g_W, g_WR[rr], sizeof(g_W));
  if (rangeAt((uint64_t)kb) != rr) flags |= FLG_IRANGE;

  SpanSums s;
  memset(&s, 0, sizeof(s));
  s.L = L;
  s.vmax = s.imax = -1e30f; s.vmin = s.imin = 1e30f;

  // harmonic phasors: theta(k) = 2 pi (k - a) / L, locked to this cycle's own
  // crossings so the phases line up from cycle to cycle. Current is analysed
  // at theta + alpha*delta, the instant it was actually sampled.
  const int H = g_harmN;
  float cVr[H_MAX + 1], cVi[H_MAX + 1], cIr[H_MAX + 1], cIi[H_MAX + 1];
  memset(cVr, 0, sizeof(float) * (H + 1)); memset(cVi, 0, sizeof(float) * (H + 1));
  memset(cIr, 0, sizeof(float) * (H + 1)); memset(cIi, 0, sizeof(float) * (H + 1));
  const double delta = 2.0 * M_PI / L;
  const double th0 = ((double)(ka - 1) - a) * delta;
  const double dr = cos(delta), di = -sin(delta);
  const double ar = cos(g_alpha * delta), ai = -sin(g_alpha * delta);

  // Pass 1: power sums, peaks, and the fundamental phasors (h = 1).
  double zr = cos(th0), zi = -sin(th0);            // e^{-j theta(k)}
  double v1r = 0, v1i = 0, i1r = 0, i1i = 0;
  for (int64_t k = ka - 1; k <= kb + 2; k++) {
    const double w = nodeWeight(k, ka, kb, a, b);
    double v = vAt(k), i = iAt(k);
    double vt = g_W[0] * vAt(k - 1) + g_W[1] * v + g_W[2] * vAt(k + 1) + g_W[3] * vAt(k + 2);
    s.sv  += w * v;      s.si  += w * i;
    s.svv += w * v * v;  s.sii += w * i * i;
    s.svi += w * vt * i;
    s.flags |= g_rf[(uint32_t)k & RING_MASK];
    if (k > ka && k <= kb) {
      float fv = (float)v, fi = (float)i;
      if (fv > s.vmax) s.vmax = fv;
      if (fv < s.vmin) s.vmin = fv;
      if (fi > s.imax) s.imax = fi;
      if (fi < s.imin) s.imin = fi;
    }
    const double qr = zr * ar - zi * ai, qi = zr * ai + zi * ar;   // current's kernel
    v1r += w * v * zr; v1i += w * v * zi;
    i1r += w * i * qr; i1i += w * i * qi;
    double t = zr * dr - zi * di; zi = zr * di + zi * dr; zr = t;
  }
  s.flags |= flags;

  // Pass 2: harmonics 2..H on what is left after removing DC and the
  // fundamental. The quadrature's end error scales with the amplitude it
  // integrates and grows steeply with h. On the raw signal, a pure sine read
  // ~0.1 % current THD (h29-h40) whenever a cycle was not a whole number of
  // samples (60.0 Hz = 133.33). The residual is only the real harmonic content,
  // so the artefact shrinks by the THD ratio, and the DFT stays on the native
  // sample grid with no interpolation loss.
  const double k2 = 2.0 / L;
  const double A1vr = k2 * v1r, A1vi = k2 * v1i, A1ir = k2 * i1r, A1ii = k2 * i1i;
  const double dcv = s.sv / L, dci = s.si / L;
  cVr[1] = (float)v1r; cVi[1] = (float)v1i; cIr[1] = (float)i1r; cIi[1] = (float)i1i;
  if (H >= 2) {
    zr = cos(th0); zi = -sin(th0);
    for (int64_t k = ka - 1; k <= kb + 2; k++) {
      const double w = nodeWeight(k, ka, kb, a, b);
      const double qr = zr * ar - zi * ai, qi = zr * ai + zi * ar;
      // x1 = Re(A1 e^{+j theta}) = A1r cos - A1i sin = A1r zr + A1i zi
      const float wv = (float)(w * (vAt(k) - dcv - (A1vr * zr + A1vi * zi)));
      const float wi = (float)(w * (iAt(k) - dci - (A1ir * qr + A1ii * qi)));
      const float fzr = (float)zr, fzi = (float)zi, fqr = (float)qr, fqi = (float)qi;
      float pr = fzr, pi = fzi;               // z^1
      float sr = fqr, si = fqi;
      for (int h = 2; h <= H; h++) {
        float t = pr * fzr - pi * fzi; pi = pr * fzi + pi * fzr; pr = t;
        cVr[h] += wv * pr; cVi[h] += wv * pi;
        t = sr * fqr - si * fqi; si = sr * fqi + si * fqr; sr = t;
        cIr[h] += wi * sr; cIi[h] += wi * si;
      }
      double t = zr * dr - zi * di; zi = zr * di + zi * dr; zr = t;
    }
  }

  PowerResult r;
  reduceSums(s, delta, r);
  r.hz = real ? (float)(g_fs / L) : 0.0f;

  // window accumulation (sums, not averages of averages: exact across cycles)
  g_win.L += s.L; g_win.sv += s.sv; g_win.si += s.si;
  g_win.svv += s.svv; g_win.sii += s.sii; g_win.svi += s.svi;
  g_win.vmax = max(g_win.vmax, s.vmax); g_win.vmin = min(g_win.vmin, s.vmin);
  g_win.imax = max(g_win.imax, s.imax); g_win.imin = min(g_win.imin, s.imin);
  g_win.flags |= s.flags;
  g_winCycles++;
  if (real) { g_winReal++; g_winRealL += L; }
  for (int h = 1; h <= H; h++) {
    g_wVr[h] += cVr[h]; g_wVi[h] += cVi[h];
    g_wIr[h] += cIr[h]; g_wIi[h] += cIi[h];
    g_wVp[h] += L * k2 * k2 * ((double)cVr[h] * cVr[h] + (double)cVi[h] * cVi[h]);
    g_wIp[h] += L * k2 * k2 * ((double)cIr[h] * cIr[h] + (double)cIi[h] * cIi[h]);
  }
  // averaged cycle: resample at AVG_M phase points from the crossing
  if (real) {
    for (int m = 0; m < AVG_M; m++) {
      double x = a + (double)m * L / AVG_M;
      g_avgAccV[m] += vCub(x);
      g_avgAccI[m] += iCub(x - g_alpha);
    }
    g_avgAccN++;
  }
  g_winEnd = b;

  // energy: every cycle, end to end, so nothing between windows is missed
  g_wh += (double)r.p * L * g_dt / 3600.0;
  g_base = r.vdc;
  g_imean = r.idc;
  g_lastVpk = r.vpk;

  onCycle(r, a, b, s.flags);
  if (g_winCycles >= g_cycles) finishWindow();
}

static float hysteresis() {
  float h = 0.2f * g_lastVpk;
  return h > g_vmin ? h : g_vmin;
}

static void onCrossing(double x) {
  double L = x - g_cycStart;
  if (g_cycReal) {
    if (L < 0.6 * g_N0) return;          // too soon: noise, keep the cycle going
    processCycle(g_cycStart, x, 0);
  } else {
    if (x <= g_cycStart) return;         // landed inside a span already measured
    if (L >= 2.0) processCycle(g_cycStart, x, FLG_NOCROSS);
  }
  g_cycStart = x;
  g_cycReal = true;
}

static void serviceAnalysis() {
  uint32_t t0 = micros();
  // fell more than half a ring behind: skip ahead rather than read stale data
  if (g_wr > g_scan + RING_N / 2) {
    g_lostUs += (uint64_t)((double)(g_wr - 64 - g_scan) * g_dt * 1.0e6);
    g_validFrom = g_wr - 64; g_gaps++;
    resetAnalyzer();
  }
  if (g_scan < g_validFrom + 3) g_scan = g_validFrom + 3;

  uint32_t budget = 4096;
  while (g_scan + 6 <= g_wr && budget--) {   // processCycle reads up to kb+4
    const uint64_t k = g_scan;
    if (g_cycStart < 0.0) {
      g_cycStart = (double)k; g_cycReal = false; g_armed = false; g_cand = -1.0;
      g_scan++;
      continue;
    }
    double v = vAt(k) - g_base, vp = vAt(k - 1) - g_base;
    double h = hysteresis();
    if (v < -h) g_armed = true;
    if (g_armed) {
      if (vp <= 0.0 && v > 0.0)      g_cand = (double)(k - 1) + (-vp / (v - vp));
      else if (vp > 0.0 && v <= 0.0) g_cand = -1.0;
      if (g_cand >= 0.0 && v > h) {
        onCrossing(g_cand);
        g_armed = false; g_cand = -1.0;
      }
    }
    // no crossing for 1.6 nominal cycles: cut a nominal one so current is
    // still measured without a voltage reference
    if (g_cycStart >= 0.0 && (double)k - g_cycStart > 1.6 * g_N0) {
      double b = g_cycStart + g_N0;
      processCycle(g_cycStart, b, FLG_NOCROSS);
      g_cycStart = b; g_cycReal = false;
    }
    g_scan++;
  }
  g_busyUs += micros() - t0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  relay, trips, events
// ═════════════════════════════════════════════════════════════════════════════

static void setLeds();
static void triggerCapture(const char* reason, uint64_t at, float value);

static void setRelay(bool on) {
  bool closing = on && !g_relay;
  g_relay = on;
  digitalWrite(RELAY_PIN, (on != g_rly_inv) ? HIGH : LOW);
  g_ln.reset(); g_ln.print("$RLY,"); g_ln.println(g_relay ? 1 : 0); emit();
  if (closing && g_rlycap) triggerCapture("RELAY", g_wr, 0.0f);
  setLeds();
}

static void tripRelay(const char* why, float value, float limit) {
  g_trip_latch = true;
  setRelay(false);
  g_ln.reset(); g_ln.print("$TRIP,"); g_ln.print(why); g_ln.print(',');
  g_ln.print(value, 4); g_ln.print(','); g_ln.println(limit, 4); emit();
  triggerCapture(why, g_wr, value);
}

static void onCycle(const PowerResult& r, double a, double b, uint8_t flags) {
  g_cycSeq++;
  if (g_trip_a > 0.0f && !g_trip_latch && r.irms > g_trip_a) tripRelay("IRMS", r.irms, g_trip_a);

  if (g_haveRef) {
    if (g_ev_di > 0.0f && fabsf(r.irms - g_refIrms) > g_ev_di)
      triggerCapture("ISTEP", (uint64_t)a, r.irms);
    else if (g_ev_dv > 0.0f && fabsf(r.vrms - g_refVrms) > g_ev_dv)
      triggerCapture(r.vrms < g_refVrms ? "VSAG" : "VSWELL", (uint64_t)a, r.vrms);
  }

  if (g_smask & SM_CYC) {
    g_ln.reset(); g_ln.print("$CYC,"); g_ln.print(g_cycSeq); g_ln.print(',');
    g_ln.print(r.vrms, 3); g_ln.print(','); g_ln.print(r.irms, 4); g_ln.print(',');
    g_ln.print(r.p, 3);    g_ln.print(','); g_ln.print(r.hz, 3);   g_ln.print(',');
    g_ln.print(r.ipk, 4);  g_ln.print(','); g_ln.println(flags | (g_trip_latch ? FLG_TRIP : 0));
    emitLow();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  window results
// ═════════════════════════════════════════════════════════════════════════════

static float phaseDeg(double re, double im) { return (float)(atan2(im, re) * 180.0 / M_PI); }
static float wrapDeg(float d) { while (d > 180.0f) d -= 360.0f; while (d <= -180.0f) d += 360.0f; return d; }

static void emitHarm(bool bulk) {
  g_ln.reset();
  g_ln.print("$HARM,"); g_ln.print(g_seq); g_ln.print(','); g_ln.print(g_hN); g_ln.print(',');
  g_ln.print(g_hHz, 3); g_ln.print(','); g_ln.print(g_hVthd, 3); g_ln.print(',');
  g_ln.print(g_hIthd, 3); g_ln.print(','); g_ln.print(g_hDpf, 4); g_ln.print(',');
  g_ln.print(g_hP1, 3);
  for (int h = 1; h <= g_hN; h++) {
    g_ln.print(','); g_ln.print(g_hVr[h], 4); g_ln.print(','); g_ln.print(g_hVd[h], 1);
    g_ln.print(','); g_ln.print(g_hIr[h], 5); g_ln.print(','); g_ln.print(g_hId[h], 1);
  }
  g_ln.println(); if (bulk) emitLow(); else emit();
}

static void emitAvg(bool bulk) {
  g_ln.reset();
  g_ln.print("$AVG,"); g_ln.print(g_seq); g_ln.print(','); g_ln.print(AVG_M); g_ln.print(',');
  g_ln.print(g_avgN); g_ln.print(','); g_ln.print(g_avgHz, 3);
  for (int m = 0; m < AVG_M; m++) { g_ln.print(','); g_ln.print(g_avgV[m], 3); }
  for (int m = 0; m < AVG_M; m++) { g_ln.print(','); g_ln.print(g_avgI[m], 4); }
  g_ln.println(); if (bulk) emitLow(); else emit();
}

static void finishWindow() {
  g_seq++;
  PowerResult r;
  double hz = (g_winReal > 0 && g_winRealL > 0) ? (double)g_winReal * g_fs / g_winRealL : 0.0;
  reduceSums(g_win, hz > 0 ? 2.0 * M_PI * hz * g_dt : 0.0, r);
  r.hz = (float)hz;

  // harmonics: phase from the coherent sum, magnitude from the power average
  // (a no-crossing window has no common phase reference, and the power average
  // stays right there; for a steady signal the two agree)
  const int H = g_harmN;
  const double WL = g_win.L > 0 ? g_win.L : 1.0;
  double v1d = 0.0;
  double vh2 = 0.0, ih2 = 0.0;
  const double th1 = (hz > 0 ? hz : (double)g_mains_hz) * 2.0 * M_PI * g_dt;
  for (int h = 1; h <= H; h++) {
    // the boxcar is a known filter: -0.08 % at h3, -32 % at h63 (defaults)
    double bg = boxcarGain(h * th1);
    if (bg < 0.2) bg = 0.2;
    float vr = (float)(sqrt(g_wVp[h] / WL / 2.0) / bg);
    float ir = (float)(sqrt(g_wIp[h] / WL / 2.0) / bg);
    float vd = phaseDeg(g_wVr[h], g_wVi[h]);
    float id = phaseDeg(g_wIr[h], g_wIi[h]);
    if (h == 1) v1d = vd;
    g_hVr[h] = vr; g_hIr[h] = ir;
    // relative to V1, scaled to that harmonic's own cycle
    g_hVd[h] = wrapDeg(vd - (float)(h * v1d));
    g_hId[h] = wrapDeg(id - (float)(h * v1d));
    if (h >= 2) { vh2 += (double)vr * vr; ih2 += (double)ir * ir; }
  }
  g_hN = (uint8_t)H; g_hHz = r.hz;
  g_hVthd = g_hVr[1] > 1e-9f ? (float)(100.0 * sqrt(vh2) / g_hVr[1]) : 0.0f;
  g_hIthd = g_hIr[1] > 1e-9f ? (float)(100.0 * sqrt(ih2) / g_hIr[1]) : 0.0f;
  float phi = (g_hId[1]) * (float)M_PI / 180.0f;     // I1 relative to V1
  g_hDpf = (g_hVr[1] > 1e-9f && g_hIr[1] > 1e-9f) ? cosf(phi) : 0.0f;
  g_hP1 = g_hVr[1] * g_hIr[1] * g_hDpf;

  if (g_avgAccN) {
    for (int m = 0; m < AVG_M; m++) { g_avgV[m] = g_avgAccV[m] / g_avgAccN; g_avgI[m] = g_avgAccI[m] / g_avgAccN; }
  } else {
    memset(g_avgV, 0, sizeof(g_avgV)); memset(g_avgI, 0, sizeof(g_avgI));
  }
  g_avgN = g_avgAccN; g_avgHz = r.hz;

  uint8_t flags = g_win.flags | (g_trip_latch ? FLG_TRIP : 0);
  if (g_winReal < g_winCycles) flags |= FLG_NOCROSS;
  const uint8_t wr = rangeAt((uint64_t)g_winEnd);

  // auto range down: two quiet windows in a row, and not within a second of
  // the last switch, so a load near the threshold doesn't make it hunt
  if (g_irReq != irSensitive() && irAutoIdle() && millis() - g_irLastSwMs > 1000) {
    float pk = max(fabsf(g_win.imax), fabsf(g_win.imin));
    if (!(flags & (FLG_ICLIP | FLG_IRANGE)) && pk < g_irDn * (float)g_irFsA[irSensitive()]) {
      if (++g_irDnCount >= 2) { g_irWant = irSensitive(); g_irDnCount = 0; }
    } else {
      g_irDnCount = 0;
    }
  }

  if (g_smask & SM_PWR) {
    g_ln.reset();
    g_ln.print("$PWR,");
    g_ln.print(r.vrms, 4);  g_ln.print(','); g_ln.print(r.irms, 5);  g_ln.print(',');
    g_ln.print(r.p, 4);     g_ln.print(','); g_ln.print(r.s, 4);     g_ln.print(',');
    g_ln.print(r.q, 4);     g_ln.print(','); g_ln.print(r.pf, 4);    g_ln.print(',');
    g_ln.print(r.hz, 3);    g_ln.print(','); g_ln.print(r.vpk, 4);   g_ln.print(',');
    g_ln.print(r.ipk, 5);   g_ln.print(','); g_ln.print((float)g_wh, 6); g_ln.print(',');
    g_ln.print(g_winEnd * g_dt, 3); g_ln.print(',');
    g_ln.print((uint32_t)(g_win.L + 0.5)); g_ln.print(',');
    g_ln.print(g_fs, 2);    g_ln.print(','); g_ln.print(g_relay ? 1 : 0); g_ln.print(',');
    g_ln.print(flags);      g_ln.print(',');
    g_ln.print(g_hVthd, 3); g_ln.print(','); g_ln.print(g_hIthd, 3); g_ln.print(',');
    g_ln.print(g_hDpf, 4);  g_ln.print(','); g_ln.print(g_winCycles); g_ln.print(',');
    g_ln.print(g_seq);      g_ln.print(','); g_ln.println(wr);
    emit();
  }
  if (g_smask & SM_HARM) emitHarm(true);
  if (g_smask & SM_AVG)  emitAvg(true);

  g_refVrms = r.vrms; g_refIrms = r.irms; g_haveRef = true;
  setLeds();
  clearWindow();
}

// ═════════════════════════════════════════════════════════════════════════════
//  waveform capture, sent in the background
// ═════════════════════════════════════════════════════════════════════════════

static uint8_t  g_capState = 0;        // 0 idle, 1 waiting for post-trigger data, 2 sending
static uint64_t g_capStart = 0, g_capEnd = 0;
static uint32_t g_capN = 0, g_capSent = 0, g_capPre = 0, g_capSeq = 0;
static uint8_t  g_capFlags = 0;
static double   g_capKi = 1.0;         // amps per $WD current unit, fixed per capture
static char     g_capReason[12] = "";

static void triggerCapture(const char* reason, uint64_t at, float value) {
  if (g_capState != 0 || !anySink() || !g_rv) return;   // one at a time; nobody listening
  uint32_t len = (uint32_t)(g_wave_len_s * g_fs);
  if (len < 16) len = 16;
  if (len > WAVE_MAX) len = WAVE_MAX;
  uint32_t pre = (uint32_t)(len * g_wpre);
  uint64_t start = at > pre ? at - pre : 0;
  if (start < g_validFrom) start = g_validFrom;                   // cannot reach across a gap
  if (g_wr > RING_N - 4096 && start < g_wr - (RING_N - 4096))     // already overwritten
    start = g_wr - (RING_N - 4096);
  g_capStart = start;
  g_capEnd = start + len;
  g_capPre = (at > start) ? (uint32_t)(at - start) : 0;
  g_capState = 1;
  strncpy(g_capReason, reason, sizeof(g_capReason) - 1);
  g_capReason[sizeof(g_capReason) - 1] = 0;
  g_ln.reset(); g_ln.print("$EVT,"); g_ln.print(reason); g_ln.print(',');
  g_ln.print((double)at * g_dt, 3); g_ln.print(','); g_ln.println(value, 4); emit();
}

static void serviceCapture() {
  if (g_capState == 1) {
    if (g_wr < g_capEnd) return;
    if (g_capStart < g_validFrom) g_capFlags = FLG_GAP; else g_capFlags = 0;
    g_capN = (uint32_t)(g_capEnd - g_capStart);
    for (uint32_t j = 0; j < g_capN; j++) {
      uint32_t idx = (uint32_t)(g_capStart + j) & RING_MASK;
      g_wv[j] = g_rv[idx]; g_wi[j] = g_ri[idx]; g_capFlags |= g_rf[idx];
    }
    g_capSeq++;
    g_capKi = g_kiRef;
    g_ln.reset();
    g_ln.print("$WAVE,"); g_ln.print(g_capN); g_ln.print(',');
    g_ln.print(g_dt * 1.0e6, 4); g_ln.print(',');
    g_ln.print(g_KV, 9); g_ln.print(','); g_ln.print(g_BV, 6); g_ln.print(',');
    g_ln.print(g_capKi, 9); g_ln.print(",0,");
    g_ln.print(g_alphaR[rangeAt(g_capStart)], 6); g_ln.print(','); g_ln.print(g_capPre); g_ln.print(',');
    g_ln.print(g_capReason); g_ln.print(','); g_ln.print(g_capFlags); g_ln.print(',');
    g_ln.println(g_capSeq);
    emit();
    g_capSent = 0;
    g_capState = 2;
    return;
  }
  if (g_capState == 2) {
    if (!anySink()) { g_capState = 0; return; }
    // Keep under CAP_BACKLOG queued, so live and bulk lines still get through
    // while a capture streams; a few hundred lines per pass keeps the ADC drained. $WD lines
    // go out whole and in order (emit, not emitLow): a capture is all or nothing.
    int burst = 300;
    while (g_capSent < g_capN && burst-- > 0 && sinkBacklog() < CAP_BACKLOG) {
      g_ln.reset(); g_ln.print("$WD,"); g_ln.print(g_wv[g_capSent], 2); g_ln.print(',');
      g_ln.println((float)(g_wi[g_capSent] / g_capKi), 2); emit();
      g_capSent++;
    }
    if (g_capSent >= g_capN && sinkBacklog() < CAP_BACKLOG) { emitStr("$WEND"); g_capState = 0; }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  zeroing
// ═════════════════════════════════════════════════════════════════════════════

// !ZERO: V and the active current range, then the other range, then back.
// Each range is averaged only once its switch has settled.
static uint64_t g_zeroFrom = 0;
static bool     g_zeroArmed = false;
static uint8_t  g_zeroBack = 0;        // range to return to

static void printAcConfig();
static void printIrange();

static void startZero() {
  g_zeroStep = 1; g_zeroArmed = false; g_zeroBack = g_irWant;
}

static void serviceZero() {
  if (!g_zeroStep) return;
  if (g_irPending || g_irWant != g_irReq) { g_zeroArmed = false; return; }
  if (!g_zeroArmed) { g_zeroArmed = true; g_zeroFrom = g_wr; return; }
  uint32_t n = (uint32_t)(0.2 * g_fs);
  if (g_wr < g_zeroFrom + n) return;
  if (g_zeroFrom < g_validFrom) {
    g_zeroStep = 0; g_irWant = g_zeroBack;
    emitErr("Zero interrupted by a gap"); return;
  }
  double sv = 0, si = 0;
  for (uint32_t j = 0; j < n; j++) {
    uint32_t idx = (uint32_t)(g_zeroFrom + j) & RING_MASK;
    sv += g_rv[idx]; si += g_ri[idx];
  }
  // the ring's current is in amps with the old offset in it, so the new
  // offset is the old one plus that mean, back in ADC volts
  const uint8_t r = g_irCur;
  g_ir_off[r] += (float)((si / n) * (double)g_ir_adc[r] / (double)g_ir_fs[r]);
  if (g_zeroStep == 1) g_v_off = (float)(sv / n * lsb());
  applyDerived();
  if (g_zeroStep == 1) { g_zeroStep = 2; g_zeroArmed = false; g_irWant = 1 - r; return; }
  g_zeroStep = 0;
  g_irWant = g_zeroBack;
  printAcConfig(); printIrange();
}

// ═════════════════════════════════════════════════════════════════════════════
//  current range switching
// ═════════════════════════════════════════════════════════════════════════════

static void setIrangePin(uint8_t r) { digitalWrite(IRANGE_PIN, ((r == 1) != g_irInv) ? HIGH : LOW); }

static void startRangeSwitch(uint8_t r) {
  g_irReq = r;
  setIrangePin(r);
  g_irSwUs = micros();
  g_irPending = true; g_irHistDone = false;
  g_irLastSwMs = millis();
  g_irSwitches++;
  g_irDnCount = 0;
}

static void serviceRange() {
  if (g_irPending && (!g_adcOn || millis() - g_irLastSwMs > 500)) {
    // no ADC data is coming to place the switch on: apply it from here on
    if (!g_irHistDone) irHistPush(g_wr, g_irReq);
    g_irCur = g_irReq; g_irPending = false; g_irReport = true;
  }
  if (g_irReport) {
    g_irReport = false;
    g_alpha = g_alphaR[g_irCur]; memcpy(g_W, g_WR[g_irCur], sizeof(g_W));
    printIrange(); printAcConfig();
  }
  if (!g_irPending && g_irWant != g_irReq) startRangeSwitch(g_irWant);
}

// ═════════════════════════════════════════════════════════════════════════════
//  status lines
// ═════════════════════════════════════════════════════════════════════════════

static void printConfig() {
  g_ln.reset();
  g_ln.print("$CFG,"); g_ln.print(BACKEND_GIGA); g_ln.print(',');
  g_ln.print(g_relay ? 1 : 0); g_ln.print(','); g_ln.println(g_smask); emit();
  g_ln.reset();
  g_ln.print("$ADC,2,"); g_ln.print(g_rate_req); g_ln.print(','); g_ln.print(g_rate, 3); g_ln.print(',');
  g_ln.print(g_decim); g_ln.print(','); g_ln.print(g_fs, 3); g_ln.print(',');
  g_ln.print(g_bits); g_ln.print(','); g_ln.print(g_smp); g_ln.print(',');
  g_ln.print(g_v_se ? 1 : 0); g_ln.print(','); g_ln.print(g_i_se ? 1 : 0); g_ln.print(',');
  g_ln.print(g_lag_s * 1.0e6, 4); g_ln.print(','); g_ln.print(maxRate()); g_ln.print(',');
  g_ln.print(g_synth ? 1 : 0); g_ln.print(','); g_ln.println(g_linCal ? 1 : 0); emit();
}

static void printAcConfig() {
  g_ln.reset();
  g_ln.print("$ACFG,");
  g_ln.print(g_v_adc_fs, 6);  g_ln.print(','); g_ln.print(g_v_fs, 4);  g_ln.print(',');
  g_ln.print(g_ir_adc[g_irCur], 6); g_ln.print(','); g_ln.print(g_ir_fs[g_irCur], 4); g_ln.print(',');
  g_ln.print(g_ir_pc[g_irCur], 3); g_ln.print(','); g_ln.print(g_cycles); g_ln.print(',');
  g_ln.print(g_mains_hz, 2);  g_ln.print(','); g_ln.print(g_trip_a, 3); g_ln.print(',');
  g_ln.print(g_dcrem ? 1 : 0); g_ln.print(',');
  g_ln.print(g_v_off * 1.0e6f, 2); g_ln.print(','); g_ln.print(g_ir_off[g_irCur] * 1.0e6f, 2); g_ln.print(',');
  g_ln.print(g_rly_inv ? 1 : 0); g_ln.print(','); g_ln.print(g_wave_len_s, 4); g_ln.print(",2,");
  g_ln.print(g_vref, 6); g_ln.print(','); g_ln.print(g_trippk, 3); g_ln.print(',');
  g_ln.print(g_wpre, 3); g_ln.print(','); g_ln.print(g_harmN); g_ln.print(',');
  g_ln.print(g_vmin, 3); g_ln.print(','); g_ln.print(g_ev_di, 4); g_ln.print(',');
  g_ln.print(g_ev_dv, 3); g_ln.print(','); g_ln.println(g_rlycap ? 1 : 0);
  emit();
}

static void printIrange() {
  g_ln.reset();
  g_ln.print("$IRNG,"); g_ln.print(g_irCur); g_ln.print(','); g_ln.print(g_irAuto ? 1 : 0); g_ln.print(',');
  g_ln.print(g_irInv ? 1 : 0); g_ln.print(','); g_ln.print(g_irSettleMs, 2); g_ln.print(',');
  g_ln.print(g_irUp * 100.0f, 1); g_ln.print(','); g_ln.print(g_irDn * 100.0f, 1); g_ln.print(',');
  g_ln.print(g_irSwitches); g_ln.print(','); g_ln.print(g_irFsA[0], 5); g_ln.print(',');
  g_ln.print(g_irFsA[1], 5);
  for (int r = 0; r < 2; r++) {
    g_ln.print(','); g_ln.print(g_ir_adc[r], 6); g_ln.print(','); g_ln.print(g_ir_fs[r], 5);
    g_ln.print(','); g_ln.print(g_ir_off[r] * 1.0e6f, 2); g_ln.print(','); g_ln.print(g_ir_pc[r], 3);
  }
  g_ln.println(); emit();
}

static void printNet() {
  g_ln.reset();
  g_ln.print("$NET,"); g_ln.print(g_ipStr); g_ln.print(','); g_ln.print(CMD_PORT); g_ln.print(',');
  g_ln.print(g_wifiOk ? (int)WiFi.RSSI() : 0); g_ln.print(',');
  g_ln.print(g_wifiStatus); g_ln.print(','); g_ln.print(g_wifiPs); g_ln.print(',');
  g_ln.println(SECRET_SSID); emit();
}

static void printInfo() {
  g_ln.reset();
  g_ln.print("$INFO,"); g_ln.print(measuredRate()); g_ln.print(','); g_ln.print(g_gaps); g_ln.print(',');
  g_ln.print(g_resyncs); g_ln.print(','); g_ln.print(g_loadPct, 1); g_ln.print(',');
  g_ln.print(g_loopMaxUs / 1000.0f, 2); g_ln.print(','); g_ln.print((uint32_t)(g_wr - g_scan)); g_ln.print(',');
  g_ln.print(g_txTcp.drops()); g_ln.print(','); g_ln.print(g_txSer.drops()); g_ln.print(',');
  g_ln.print(g_tcpBusy); g_ln.print(','); g_ln.print(largestFreeBlock()); g_ln.print(',');
  g_ln.print(g_adcFixes); g_ln.print(','); g_ln.print(millis() / 1000.0f, 1); g_ln.print(',');
  g_ln.print(g_wifiReconnects); g_ln.print(','); g_ln.print((float)(g_lostUs / 1000ULL)); g_ln.print(',');
  g_ln.println(g_lowDrops);
  emit();
  g_loopMaxUs = 0;   // worst loop time is reported per $INFO interval
}

static void printBanner() {
  g_ln.reset(); g_ln.print("$READY,GIGA,"); g_ln.println(FW_VERSION); emit();
  printConfig(); printAcConfig(); printIrange(); printNet();
}

// ═════════════════════════════════════════════════════════════════════════════
//  commands
// ═════════════════════════════════════════════════════════════════════════════

static float secondArg(const char* args) {
  const char* c = strchr(args, ',');
  return c ? atof(c + 1) : NAN;
}
// up to n comma-separated floats; returns how many were present
static int parseFloats(const char* args, float* out, int n) {
  int got = 0;
  const char* p = args;
  while (got < n && p && *p) {
    out[got++] = atof(p);
    p = strchr(p, ',');
    if (p) p++;
  }
  return got;
}

static void restartAcq() {
  if (!adcStart()) return;
  clearWindow();
  printConfig();
}

static void handleCommand(const char* cmd) {
  if (strcmp(cmd, "!START") == 0) {
    g_smask |= SM_PWR; printConfig();

  } else if (strcmp(cmd, "!STOP") == 0) {
    g_smask = 0; printConfig();

  } else if (strncmp(cmd, "!SMASK,", 7) == 0) {
    g_smask = (uint8_t)atoi(cmd + 7); printConfig();

  } else if (strcmp(cmd, "!CFG") == 0) {
    printConfig(); printAcConfig(); printIrange(); printNet();

  } else if (strcmp(cmd, "!STAT") == 0) {
    printInfo();

  } else if (strcmp(cmd, "!WIFI") == 0) {
    // rejoin in the background; $NET reports the result
    if (g_wifiBusy) emitErr("Already joining");
    else { dropClient(); g_wifiOk = false; g_wifiReconnects++; startWifiConnect(true); printNet(); }

  } else if (strcmp(cmd, "!SCAN") == 0 && g_wifiBusy) {
    emitErr("Wi-Fi is joining; try again");

  } else if (strcmp(cmd, "!SCAN") == 0) {
    // Wi-Fi diagnostics: module firmware and the networks it can see. A
    // firmware of v0.0.0 means WiFiFirmwareUpdater has never been run.
    const char* fw = WiFi.firmwareVersion();
    int n = WiFi.scanNetworks();
    g_ln.reset(); g_ln.print("$SCAN,"); g_ln.print(fw); g_ln.print(','); g_ln.println(n); emit();
    for (int k = 0; k < n && k < 30; k++) {
      g_ln.reset(); g_ln.print("$AP,"); g_ln.print(WiFi.SSID(k)); g_ln.print(',');
      g_ln.print((int)WiFi.RSSI(k)); g_ln.print(','); g_ln.println((int)WiFi.encryptionType(k)); emit();
    }
    markGap();

  } else if (strcmp(cmd, "!HGET") == 0) {
    emitHarm(false);

  } else if (strcmp(cmd, "!AVGW") == 0) {
    emitAvg(false);

  } else if (strncmp(cmd, "!RELAY,", 7) == 0) {
    bool on = (atoi(cmd + 7) != 0);
    if (on) g_trip_latch = false;
    setRelay(on);

  } else if (strncmp(cmd, "!RLYINV,", 8) == 0) {
    g_rly_inv = (atoi(cmd + 8) != 0);
    setRelay(g_relay); printAcConfig();

  } else if (strncmp(cmd, "!VSCALE,", 8) == 0) {
    float adc = atof(cmd + 8), real = secondArg(cmd + 8);
    if (adc > 0.0f && !isnan(real) && real != 0.0f) { g_v_adc_fs = adc; g_v_fs = real; applyDerived(); printAcConfig(); }
    else emitErr("VSCALE needs <adcVolts>,<realVolts>");

  } else if (strncmp(cmd, "!ISCALE,", 8) == 0) {
    float a[3];
    int n = parseFloats(cmd + 8, a, 3);
    int r = n >= 3 ? (int)a[2] : g_irCur;
    if (n >= 2 && a[0] > 0.0f && a[1] != 0.0f && (r == 0 || r == 1)) {
      g_ir_adc[r] = a[0]; g_ir_fs[r] = a[1]; applyDerived(); printAcConfig(); printIrange();
    } else emitErr("ISCALE needs <adcVolts>,<realAmps>[,<range 0|1>]");

  } else if (strncmp(cmd, "!PHASECAL,", 10) == 0) {
    float a[2];
    int n = parseFloats(cmd + 10, a, 2);
    int r = n >= 2 ? (int)a[1] : g_irCur;
    if (n >= 1 && a[0] >= -500.0f && a[0] <= 500.0f && (r == 0 || r == 1)) {
      g_ir_pc[r] = a[0]; applyDerived(); printAcConfig(); printIrange();
    } else emitErr("PHASECAL must be -500..500 us [,<range 0|1>]");

  } else if (strncmp(cmd, "!IRANGE,", 8) == 0) {
    char c = cmd[8];
    if (c == 'A' || c == 'a') { g_irAuto = true; g_irDnCount = 0; printIrange(); }
    else if (c == '0' || c == '1') { g_irAuto = false; g_irWant = (uint8_t)(c - '0'); printIrange(); }
    else emitErr("IRANGE must be 0, 1 or A");

  } else if (strncmp(cmd, "!IRINV,", 7) == 0) {
    g_irInv = (atoi(cmd + 7) != 0);
    startRangeSwitch(g_irReq);     // the pin flips, so the front end does too
    printIrange();

  } else if (strncmp(cmd, "!IRSETTLE,", 10) == 0) {
    float v = atof(cmd + 10);
    if (v >= 0.0f && v <= 200.0f) { g_irSettleMs = v; printIrange(); }
    else emitErr("IRSETTLE must be 0-200 ms");

  } else if (strncmp(cmd, "!IRLIM,", 7) == 0) {
    float a[2];
    int n = parseFloats(cmd + 7, a, 2);
    if (n == 2 && a[0] >= 50.0f && a[0] <= 100.0f && a[1] >= 1.0f && a[1] <= a[0] - 5.0f) {
      g_irUp = a[0] / 100.0f; g_irDn = a[1] / 100.0f; printIrange();
    } else emitErr("IRLIM needs <up 50-100 %>,<down 1 % .. up-5 %>");

  } else if (strncmp(cmd, "!CYCLES,", 8) == 0) {
    int v = atoi(cmd + 8);
    if (v >= 1 && v <= 250) { g_cycles = (uint8_t)v; printAcConfig(); }
    else emitErr("CYCLES must be 1-250");

  } else if (strncmp(cmd, "!MAINS,", 7) == 0) {
    float v = atof(cmd + 7);
    if (v >= 10.0f && v <= 400.0f) { g_mains_hz = v; applyDerived(); resetAnalyzer(); printAcConfig(); }
    else emitErr("MAINS must be 10-400 Hz");

  } else if (strncmp(cmd, "!VMIN,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v >= 0.0f) { g_vmin = v; printAcConfig(); } else emitErr("VMIN must be >= 0");

  } else if (strncmp(cmd, "!TRIP,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v >= 0.0f) { g_trip_a = v; g_trip_latch = false; printAcConfig(); }
    else emitErr("TRIP must be >= 0");

  } else if (strncmp(cmd, "!TRIPPK,", 8) == 0) {
    float v = atof(cmd + 8);
    if (v >= 0.0f) { g_trippk = v; g_trip_latch = false; printAcConfig(); }
    else emitErr("TRIPPK must be >= 0");

  } else if (strncmp(cmd, "!DCREM,", 7) == 0) {
    g_dcrem = (atoi(cmd + 7) != 0); printAcConfig();

  } else if (strcmp(cmd, "!ZERO") == 0) {
    startZero();

  } else if (strncmp(cmd, "!VOFF,", 6) == 0) {
    g_v_off = atof(cmd + 6) * 1.0e-6f; applyDerived(); printAcConfig();

  } else if (strncmp(cmd, "!IOFF,", 6) == 0) {
    float a[2];
    int n = parseFloats(cmd + 6, a, 2);
    int r = n >= 2 ? (int)a[1] : g_irCur;
    if (n >= 1 && (r == 0 || r == 1)) {
      g_ir_off[r] = a[0] * 1.0e-6f; applyDerived(); printAcConfig(); printIrange();
    } else emitErr("IOFF needs <uV>[,<range 0|1>]");

  } else if (strcmp(cmd, "!EZERO") == 0) {
    g_wh = 0.0;

  } else if (strncmp(cmd, "!VREF,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v > 0.1f && v < 6.0f) { g_vref = v; applyDerived(); printAcConfig(); }
    else emitErr("VREF must be 0.1-6 V");

  } else if (strncmp(cmd, "!HARMN,", 7) == 0) {
    int v = atoi(cmd + 7);
    if (v >= 1 && v <= H_MAX) { g_harmN = (uint8_t)v; applyDerived(); clearWindow(); printAcConfig(); }
    else emitErr("HARMN must be 1-63");

  } else if (strcmp(cmd, "!WAVE") == 0) {
    if (g_capState) emitErr("Capture already in progress");
    else triggerCapture("MANUAL", g_wr, 0.0f);

  } else if (strncmp(cmd, "!WLEN,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v > 0.0f) { g_wave_len_s = min(v, (float)(WAVE_MAX * g_dt)); printAcConfig(); }
    else emitErr("WLEN must be > 0");

  } else if (strncmp(cmd, "!WPRE,", 6) == 0) {
    float v = atof(cmd + 6);
    if (v >= 0.0f && v <= 1.0f) { g_wpre = v; printAcConfig(); } else emitErr("WPRE must be 0..1");

  } else if (strncmp(cmd, "!EVT,", 5) == 0) {
    float a = atof(cmd + 5), b = secondArg(cmd + 5);
    g_ev_di = a > 0 ? a : 0; g_ev_dv = (!isnan(b) && b > 0) ? b : 0; printAcConfig();

  } else if (strncmp(cmd, "!RLYCAP,", 8) == 0) {
    g_rlycap = (atoi(cmd + 8) != 0); printAcConfig();

  } else if (strncmp(cmd, "!RATE,", 6) == 0) {
    long v = atol(cmd + 6);
    if (v >= 1000 && v <= (long)maxRate()) { g_rate_req = (uint32_t)v; restartAcq(); }
    else { g_ln.reset(); g_ln.print("$ERR,RATE must be 1000-"); g_ln.println(maxRate()); emit(); }

  } else if (strncmp(cmd, "!DECIM,", 7) == 0) {
    int v = atoi(cmd + 7);
    if (v >= 1 && v <= 1000) { g_decim = (uint16_t)v; restartAcq(); }
    else emitErr("DECIM must be 1-1000");

  } else if (strncmp(cmd, "!BITS,", 6) == 0) {
    int v = atoi(cmd + 6);
    if (v == 8 || v == 10 || v == 12 || v == 14 || v == 16) { g_bits = (uint8_t)v; restartAcq(); printAcConfig(); }
    else emitErr("BITS must be 8, 10, 12, 14 or 16");

  } else if (strncmp(cmd, "!SMP,", 5) == 0) {
    int v = atoi(cmd + 5);
    if (v >= 0 && v <= 7) { g_smp = (uint8_t)v; if (g_rate_req > maxRate()) g_rate_req = maxRate(); restartAcq(); }
    else emitErr("SMP must be 0-7");

  } else if (strncmp(cmd, "!SE,", 4) == 0) {
    g_v_se = atoi(cmd + 4) != 0;
    float q = secondArg(cmd + 4);
    if (!isnan(q)) g_i_se = (q != 0.0f);
    markGap(); printConfig();

  } else if (strncmp(cmd, "!SYNTH,", 7) == 0) {
    float a[6];
    int n = parseFloats(cmd + 7, a, 6);
    if (n >= 2) g_syn_vrms = a[1];
    if (n >= 3) g_syn_irms = a[2];
    if (n >= 4) g_syn_deg  = a[3];
    if (n >= 5) g_syn_h3   = a[4];
    if (n >= 6) g_syn_hz   = a[5]; else if (n >= 1 && a[0] != 0) g_syn_hz = g_mains_hz;
    g_synth = (n >= 1 && a[0] != 0.0f);
    markGap(); printConfig();
    if (g_synth) {
      g_ln.reset(); g_ln.print("$SYNTH,"); g_ln.print(g_syn_vrms, 3); g_ln.print(',');
      g_ln.print(g_syn_irms, 4); g_ln.print(','); g_ln.print(g_syn_deg, 3); g_ln.print(',');
      g_ln.print(g_syn_h3, 3); g_ln.print(','); g_ln.println(g_syn_hz, 4); emit();
    }

  } else {
    g_ln.reset(); g_ln.print("$ERR,Unknown command: "); g_ln.println(cmd); emit();
  }
}

// line assembly, one per input (RxLine is in ac_types.h)
static RxLine g_rxSer = {{0}, 0}, g_rxTcp = {{0}, 0};

static void feed(RxLine& rx, int c) {
  if (c == '\n' || c == '\r') {
    if (rx.n) { rx.b[rx.n] = 0; handleCommand(rx.b); rx.n = 0; }
  } else if (rx.n < sizeof(rx.b) - 1) {
    rx.b[rx.n++] = (char)c;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Wi-Fi
// ═════════════════════════════════════════════════════════════════════════════

static uint32_t g_lastWifiTry = 0, g_lastWifiCheck = 0;

static void setLeds() {
  // the Giga's RGB LED is active-low. Red = tripped, green = a client is
  // connected (toggles each window), blue = no Wi-Fi.
  static bool tog = false;
  tog = !tog;
  digitalWrite(LEDR, g_trip_latch ? LOW : HIGH);
  digitalWrite(LEDG, (g_clientOn && tog) ? LOW : HIGH);
  digitalWrite(LEDB, g_wifiOk ? HIGH : LOW);
}

// Joining the network happens on its own RTOS thread. WiFi.begin() blocks for
// 5-20 s (longer on a weak signal), and on the loop thread that was 20+ s of
// lost samples per attempt (measured 2026-11-08: one reconnect = 22.6 s gap).
// On its own thread the loop keeps draining the ADC, so a dropped or missing
// network costs nothing but the link. The loop thread does not touch WiFi
// while g_wifiBusy is set; it picks up the result once the thread is done.
static rtos::Thread*  g_wifiThread = nullptr;

static void wifiTask() {
  int st = WiFi.status();
  if (st != WL_NO_MODULE) {
    if (g_wifiDisc) { WiFi.disconnect(); g_wifiDisc = false; }
    // a weak signal often fails the first join and takes the second
    for (int k = 0; k < 2 && st != WL_CONNECTED; k++) {
      WiFi.begin(SECRET_SSID, SECRET_PASS);
      uint32_t t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
      st = WiFi.status();
    }
  }
  g_wifiResult = st;
  g_wifiDone = true;
  g_wifiBusy = false;
}

static void startWifiConnect(bool disconnectFirst) {
  if (g_wifiBusy) return;
  if (g_wifiThread) { g_wifiThread->join(); delete g_wifiThread; g_wifiThread = nullptr; }
  g_wifiDisc = disconnectFirst;
  g_wifiBusy = true;
  g_lastWifiTry = millis();
  strcpy(g_ipStr, "joining");
  // same priority as the loop: RTX round-robins equal priorities, and the loop
  // never sleeps, so a lower priority would starve the join
  g_wifiThread = new rtos::Thread(osPriorityNormal, 8192, nullptr, "wifijoin");
  g_wifiThread->start(wifiTask);
}

// loop side: called once the join thread has finished
// The radio's power-save sleeps between beacons, so every round trip waits up to
// a beacon interval: pings measured 22-406 ms (avg 188). This core's lwIP has
// TCP_SND_BUF = 2 x 536 B (fixed in the prebuilt libmbed.a), so TCP throughput
// is ~1 KB per round trip, and power-save capped it at ~18 KB/s. Turning it off
// is the one lever a sketch has. Costs a few tens of mA, irrelevant here.
static void disableWifiPowerSave() {
  WhdSTAInterface* sta = static_cast<WhdSTAInterface*>(WiFiInterface::get_default_instance());
  whd_interface_t ifp = nullptr;
  g_wifiPs = -1;
  if (!sta || sta->wifi_get_ifp(&ifp) != 0 || !ifp) return;
  whd_wifi_disable_powersave(ifp);
  uint32_t mode = 0;
  if (whd_wifi_get_powersave_mode(ifp, &mode) == 0) g_wifiPs = (int)mode;
}

static void finishWifiConnect() {
  g_wifiDone = false;
  g_wifiStatus = g_wifiResult;
  if (g_wifiStatus == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(g_ipStr, sizeof(g_ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    g_server.begin();
    disableWifiPowerSave();
    g_wifiOk = true;
  } else {
    strcpy(g_ipStr, g_wifiStatus == WL_NO_MODULE ? "no-module" : "none");
    g_wifiOk = false;
  }
  setLeds();
  printNet();
}

static void dropClient() {
  if (g_clientOn) g_client.stop();
  g_clientOn = false;
  g_txTcp.clear();
  g_rxTcp.n = 0;
}

static void serviceNet() {
  uint32_t now = millis();
  if (g_wifiBusy) return;                 // the join thread owns WiFi right now
  if (g_wifiDone) finishWifiConnect();
  if (g_wifiOk && now - g_lastWifiCheck > 2000) {
    g_lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      g_wifiOk = false; dropClient(); strcpy(g_ipStr, "none"); setLeds(); printNet();
    }
  }
  if (!g_wifiOk) {
    // retry every 20 s; it costs the sampling nothing now
    if (g_wifiStatus != WL_NO_MODULE && now - g_lastWifiTry > 20000) {
      g_wifiReconnects++;
      startWifiConnect(true);
    }
    return;
  }
  WiFiClient c = g_server.available();
  if (c) {
    // newest connection wins: a half-open old socket must not hold the slot,
    // because the Giga's socket pool is tiny (the Giga_RemoteController lesson)
    dropClient();
    g_client = c;
    g_client.setSocketTimeout(0);   // non-blocking send; net_sink.h keeps the rest
    g_clientOn = true;
    printBanner();
    setLeds();
  }
  if (g_clientOn && !g_client.connected()) { dropClient(); setLeds(); }
  if (g_clientOn) {
    int guard = 256;
    while (g_client.available() && guard-- > 0) feed(g_rxTcp, g_client.read());
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  setup / loop
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  // the relay is driven open before anything else can go wrong
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, g_rly_inv ? HIGH : LOW);
  pinMode(IRANGE_PIN, OUTPUT);
  setIrangePin(0);                    // range 0 at boot
  irHistPush(0, 0);
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, LOW);

  Serial.begin(115200);   // no wait: this board is meant to run headless

  SDRAM.begin();
  g_rv = (float*)SDRAM.malloc(RING_N * sizeof(float));
  g_ri = (float*)SDRAM.malloc(RING_N * sizeof(float));
  g_rf = (uint8_t*)SDRAM.malloc(RING_N);
  g_wv = (float*)SDRAM.malloc(WAVE_MAX * sizeof(float));
  g_wi = (float*)SDRAM.malloc(WAVE_MAX * sizeof(float));
  if (!g_rv || !g_ri || !g_rf || !g_wv || !g_wi) {
    while (1) { Serial.println("$ERR,SDRAM allocation failed"); digitalWrite(LEDR, LOW); delay(1000); }
  }
  memset(g_rf, 0, RING_N);

  // measuring starts at once; the network joins in the background
  applyDerived();
  adcStart();
  clearWindow();
  g_gaps = 0;          // the start-up "gap" is not one
  startWifiConnect(false);
  setLeds();
  g_serOn = _SerialUSB.connected();
  printBanner();
}

void loop() {
  uint32_t now = micros();
  if (g_lastLoopUs) {
    uint32_t d = now - g_lastLoopUs;
    if (d > g_loopMaxUs) g_loopMaxUs = d;
  }
  g_lastLoopUs = now;
  g_serOn = _SerialUSB.connected();

  drainAdc();
  serviceAnalysis();
  drainAdc();
  serviceCapture();
  serviceZero();
  serviceRange();

  while (Serial.available()) feed(g_rxSer, Serial.read());
  serviceNet();
  serviceTx();

  // once a second: load and worst loop time, and the $INFO stream
  static uint32_t lastStat = 0, statInfo = 0;
  uint32_t ms = millis();
  if (ms - lastStat >= 1000) {
    float el = (float)(ms - lastStat) * 1000.0f;
    g_loadPct = lastStat ? 100.0f * (float)g_busyUs / el : 0.0f;
    g_loopMaxMs = g_loopMaxUs / 1000.0f;
    g_busyUs = 0;
    lastStat = ms;
    if ((g_smask & SM_INFO) && ++statInfo >= 2) { statInfo = 0; printInfo(); }
  }
}
