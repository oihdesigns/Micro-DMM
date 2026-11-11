/*!
 * Giga ADS1256 (Wi-Fi)
 * An ADS1256 24-bit ADC on an Arduino Giga R1 WiFi, driven over Wi-Fi (TCP port
 * 8080) or USB serial with the same line protocol. The front end is
 * gads_gui.py, the Wi-Fi version of "ADS1256 GUI V3_triggering.py" (the
 * Raspberry Pi GUI): live channel readings and graph, DMM math, GPIO controls,
 * and triggered Benchmark / Multi captures with CSV export.
 *
 * The Pi GUI did the SPI and the triggering in Python. Here the board does all
 * of the timing-critical work and the PC only draws:
 *
 *  - A high-priority RTOS thread owns the ADS1256. The DRDY falling edge is an
 *    interrupt that timestamps the conversion (us_ticker, 1 us) and wakes the
 *    thread, which reads it. The loop thread (Wi-Fi, USB, commands) never
 *    touches SPI, so a slow network can't stall a conversion.
 *  - Benchmark: one channel in RDATAC at the full data rate. Every DRDY edge is
 *    counted, so a conversion that was overwritten before it was read is
 *    counted as a gap, not silently lost.
 *  - Multi: the datasheet's pipelined mux cycling (on DRDY: write the next MUX,
 *    SYNC, WAKEUP, then RDATA reads the channel just finished), so each value
 *    is a fully settled conversion. The trigger channel is first in each frame.
 *  - Triggers (rising / falling through a level, raw code) are checked on the
 *    board on every conversion, with the pre-trigger history in an SDRAM ring:
 *    Start = the trigger frame is first, Mid = half before it, End = it is last.
 *  - Live readings: the enabled channels are scanned continuously and every
 *    $LIVE report is the mean (and sd, min, max) of all the scans in its
 *    period, not one conversion.
 *
 * ── Wiring (ADS1256 module / Isolated_ADS1256 board) ────────────────────────
 *   SCLK  <- D13      DIN  <- D11 (COPI)      DOUT -> D12 (CIPO)    (SPI1)
 *   CS    <- D10      DRDY -> D2 (interrupt)  RESET <- D7 (optional, PIN_RST)
 *   SYNC/PDWN: tie high.   Range relay / DMM high-range output: D6.
 *   3.3 V logic. Not D4 or D8 for DRDY: the Wi-Fi chip owns EXTI line 8.
 *   SCLK is 1.875 MHz, under the ADS1256's fCLKIN/4 limit (7.68 MHz crystal).
 *   D11/D13 are set to the slowest GPIO slew: at mbed's default (very high) the
 *   COPI edge coupled into SCLK over jumper wires and ~36 % of Multi values came
 *   back shifted one bit (x2). Found and fixed on hardware 2026-11-11.
 *
 * ── Commands (newline terminated) ───────────────────────────────────────────
 *   !ID                       $READY,GIGA_ADS1256,<fw>
 *   !STATUS                   $STATUS,key=value,...
 *   !DRATE,<code>             DRATE register (e.g. 0xF0 = 30k SPS); self-cal
 *   !GAIN,<0..6>              PGA 1..64; self-cal
 *   !BUF,<0|1>                input buffer (STATUS.BUFEN); self-cal
 *   !CAL                      self-calibration ($CAL,<ok>,<ms> when done)
 *   !CHMAP,<mux>,<mux>,...    up to 8 channels, MUX byte each (0x08 = AIN0 vs
 *                             AINCOM, 0x45 = AIN4-AIN5). Default: the Pi GUI's
 *                             CH0..CH3 SE, CH4-5, CH6-7
 *   !CH,<mask>                channels in the live scan
 *   !LIVE,<0|1>[,<ms>]        live reports on/off, report period (default 500)
 *   !RANGE,<0|1>              D6 (the DMM math high-range output)
 *   !CAP,<mode>,<n>,<mask>,<trig_ch>,<armed>,<level>,<slope>,<pos>,<timeout_ms>
 *        mode 0 benchmark (trig_ch only) / 1 multi (trig_ch + mask), n frames,
 *        level = raw code, slope 0 rising / 1 falling, pos 0 start / 1 mid /
 *        2 end, timeout 0 = none.  -> $OK,CAP then $CAPARM (armed),
 *        $CAPTRIG,<ms waited>, the data, or $CAPTMO,<TIMEOUT|ABORT> /
 *        $ERR,capture: ...
 *   !ABORT                    stop a capture (waiting, running or sending)
 *   !REG                      $REG,<11 register bytes, hex>
 *   !REINIT                   reset the ADS1256 and set it up again
 *   !GPIOLIST                 $GPIOLIST,<pin>,...   (the user GPIO pins)
 *   !GPIO,<pin>,<mode>        0 input, 1 output low, 2 output high,
 *                             3 input pull-up   -> $GPIO,<pin>,<mode>,<level>
 *   !GPIOS                    $GPIOS,<pin>:<mode>:<level>,...
 *   !NET / !WIFI / !SCAN      $NET line / rejoin Wi-Fi / list access points
 *   !SLEW,<sck>,<copi>        GPIO output speed of D13 / D11, 0 (default) .. 3
 *   !DAC,<0|1>,<volts>        test source: DC on A12 (DAC0) / A13 (DAC1), 0..3.3 V
 *   !DACSINE,<0|1>,<hz>,<vpp>,<offset_v>   test source: sine (loop-updated)
 *
 * ── Output ──────────────────────────────────────────────────────────────────
 *   $LIVE,<frames>,<period_ms>,<n>, then per channel:
 *         <ch>,<mean>,<sd>,<min>,<max>,<clip>     (raw codes)
 *   $CAL,<1 ok|0 timeout>,<ms>      $ADC,<OK|MISSING>,<status reg hex>
 *   $CAP,<frames>,<nch>,<ch:ch:...>,<trig_idx|-1>,<gaps>,<flags>,<dt_us>,
 *        <drate>,<pga>,<buf>
 *   $CD,<first frame>,<hex>   per frame 8 hex digits of time (us after frame
 *        0) then 6 hex digits (24-bit two's complement) per channel
 *   $CEND,<frames>
 *   flags: +1 conversions lost (gaps; in Benchmark this includes reads a new
 *          DRDY edge overlapped, which are dropped)  +2 a value at full scale
 *   $NET,<ip>,<port>,<rssi>,<status>,<powersave>,<ssid>,<net 1-5, 0 none>,
 *        <networks listed>,<bit mask of those seen in the last scan>
 *   $OK,<what>...   $ERR,<message>
 *
 * Volts = code * 2 * VREF / (gain * 0x7FFFFF), done in the GUI (VREF 2.5 V).
 */

#include <SPI.h>
#include <WiFi.h>
#include "mbed.h"
#include "pinDefinitions.h"
#include "USB/PluggableUSBSerial.h"   // _SerialUSB: the USBSerial behind Serial
#include "WhdSTAInterface.h"          // the Giga's Wi-Fi driver, for power-save
#include "whd_wifi_api.h"
#include "hal/us_ticker_api.h"
#include "SDRAM.h"
#include "gads_types.h"
#include "net_sink.h"

#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#else
#error "Copy arduino_secrets.example.h to arduino_secrets.h and put your Wi-Fi name/password in it"
#endif

#define FW_VERSION "1.0"
static const uint16_t CMD_PORT = 8080;

// ── wiring ────────────────────────────────────────────────────────────────────
static const int PIN_CS    = 10;
static const int PIN_DRDY  = 2;    // needs an edge interrupt: not D4 / D8
static const int PIN_RST   = -1;   // D7 if RESET is wired; -1 = not brought out (generic module)
static const int PIN_RANGE = 6;
#define ADS_SPI SPI1               // D11 / D12 / D13
static const uint32_t SPI_HZ = 1875000;   // <= fCLKIN / 4 = 1.92 MHz

// ADS1256 timing at fCLKIN = 7.68 MHz (datasheet t6, t11)
static const uint32_t T6_US     = 7;   // command -> data out: 50 tCLKIN = 6.5 us
static const uint32_t T11_US    = 1;   // after WREG / RREG / RDATA: 4 tCLKIN
static const uint32_t T11S_US   = 4;   // after SYNC / RDATAC: 24 tCLKIN = 3.1 us

// user GPIO pins offered to the GUI (everything not used above, not I2C/UART)
static const uint8_t GPIO_PINS[] = {3, 4, 5, 8, 9, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};
static const int N_GPIO = sizeof(GPIO_PINS);

// ── ADS1256 ───────────────────────────────────────────────────────────────────
enum : uint8_t {
  C_WAKEUP = 0x00, C_RDATA = 0x01, C_RDATAC = 0x03, C_SDATAC = 0x0F,
  C_RREG = 0x10, C_WREG = 0x50, C_SELFCAL = 0xF0, C_SYNC = 0xFC,
  C_STANDBY = 0xFD, C_RESET = 0xFE
};
enum : uint8_t { R_STATUS = 0, R_MUX = 1, R_ADCON = 2, R_DRATE = 3, N_REGS = 11 };

static const DrateInfo DRATES[] = {
  {0xF0, 30000}, {0xE0, 15000}, {0xD0, 7500}, {0xC0, 3750}, {0xB0, 2000},
  {0xA1, 1000},  {0x92, 500},   {0x82, 100},  {0x72, 60},   {0x63, 50},
  {0x53, 30},    {0x43, 25},    {0x33, 15},   {0x23, 10},   {0x13, 5}, {0x03, 2.5f},
};
static const int N_DRATES = sizeof(DRATES) / sizeof(DRATES[0]);

static float drateSps(uint8_t code) {
  for (int i = 0; i < N_DRATES; i++) if (DRATES[i].code == code) return DRATES[i].sps;
  return 0.0f;
}

// ── capture memory (SDRAM) ──────────────────────────────────────────────────
static const uint32_t CAP_VALS  = 1048576;   // int32 values: 4 MB
static const uint32_t CAP_FRAMES = 524288;   // uint32 frame times: 2 MB
static int32_t*  g_cv = nullptr;
static uint32_t* g_ct = nullptr;

// ── state shared between the loop thread and the ADC thread ─────────────────
// The loop thread writes g_cfg inside a critical section and bumps g_cfgSeq;
// the ADC thread (higher priority, so the loop can't run while it copies)
// takes a snapshot when the sequence moves.
static AdcCfg g_cfg = {0xF0, 0, 0, 6, {0x08, 0x18, 0x28, 0x38, 0x45, 0x67, 0x08, 0x08}, 0x3F};
static volatile uint32_t g_cfgSeq = 1;
static volatile bool g_liveOn = true;
static uint32_t      g_liveMs = 500;
static volatile bool g_calReq = false, g_initReq = false, g_regReq = false;

// ADC thread -> loop
enum : uint8_t { ADC_INIT = 0, ADC_OK = 1, ADC_MISSING = 2 };
static volatile uint8_t  g_adcState = ADC_INIT, g_adcStReg = 0;
static volatile uint32_t g_adcEvents = 0;           // bumped on every state change
static volatile uint32_t g_calCount = 0, g_calMs = 0;
static volatile bool     g_calOk = false;
static volatile bool     g_regReady = false;
static uint8_t           g_regs[N_REGS];
static volatile uint32_t g_drdyTimeouts = 0, g_convCount = 0;
static AdcCfg            g_applied;                 // what the ADC thread runs with
static volatile uint32_t g_appliedSeq = 0;

// live reports: the loop asks (g_liveReq), the ADC thread publishes at the
// end of the next complete scan (g_liveReady), the loop prints and clears
static LiveAcc g_acc[8], g_pub[8];
static volatile bool     g_liveReq = false, g_liveReady = false;
static volatile uint32_t g_pubFrames = 0, g_pubUs = 0;
static volatile uint8_t  g_pubMask = 0;

// capture state machine. The loop moves 0 -> 1 (request) and 4..7 -> 0;
// the ADC thread moves 1 -> 2 -> 3 -> 4 (or 5 / 6 / 7).
enum : uint8_t { CS_IDLE = 0, CS_REQ, CS_WAIT, CS_POST, CS_READY, CS_TIMEOUT, CS_ABORT, CS_FAIL };
static CapReq g_cr;
static volatile uint8_t  g_capState = CS_IDLE;
static volatile bool     g_capAbort = false;
static volatile uint32_t g_capWaitMs = 0, g_capGaps = 0, g_capFirst = 0;
static volatile uint8_t  g_capFlags = 0;
static char              g_capFail[48] = "";

// DRDY interrupt
static const uint32_t F_DRDY = 1, F_POKE = 2;
static mbed::InterruptIn*  g_drdyIn = nullptr;
static mbed::DigitalOut*   g_csOut = nullptr;
static rtos::Thread*       g_adcThread = nullptr;
static volatile osThreadId_t g_adcTid = nullptr;
static volatile uint32_t   g_drdyCnt = 0, g_drdyTs = 0;
static volatile bool       g_wantDrdy = false;

static void drdyIsr() {
  g_drdyTs = us_ticker_read();
  g_drdyCnt++;
  if (g_wantDrdy && g_adcTid) osThreadFlagsSet(g_adcTid, F_DRDY);
}
static void pokeAdc() { if (g_adcTid) osThreadFlagsSet(g_adcTid, F_POKE); }

// ── diagnostics ───────────────────────────────────────────────────────────────
static uint32_t g_wifiReconnects = 0, g_tcpBusy = 0, g_loopMaxUs = 0, g_lastLoopUs = 0;
static float    g_loopMaxMs = 0.0f, g_convRate = 0.0f;
static uint8_t  g_rangeHi = 1;
static uint8_t  g_gpioMode[N_GPIO];

// ═════════════════════════════════════════════════════════════════════════════
//  output plumbing (as Giga_AC_Power_WiFi: net_sink.h, never blocks)
// ═════════════════════════════════════════════════════════════════════════════

static TxRing     g_txSer, g_txTcp;
static LineBuf    g_ln;
static WiFiServer g_server(CMD_PORT);
static WiFiClient g_client;
static bool       g_clientOn = false, g_wifiOk = false;
static char       g_ipStr[20] = "none";
static int        g_wifiStatus = 0;     // last WiFi.status(): 3 = connected, 255 = no module
static int        g_wifiPs = -1;        // radio power-save mode after the join
static volatile bool g_wifiBusy = false, g_wifiDone = false, g_wifiDisc = false;
static volatile int  g_wifiResult = 0;
static volatile int  g_wifiJoined = -1;
static volatile uint8_t g_wifiSeen = 0;
static int           g_wifiNet = -1;

static const WifiNet WIFI_NETS[] = {
  {SECRET_SSID, SECRET_PASS},
#if defined(SECRET_SSID2) && defined(SECRET_PASS2)
  {SECRET_SSID2, SECRET_PASS2},
#endif
#if defined(SECRET_SSID3) && defined(SECRET_PASS3)
  {SECRET_SSID3, SECRET_PASS3},
#endif
#if defined(SECRET_SSID4) && defined(SECRET_PASS4)
  {SECRET_SSID4, SECRET_PASS4},
#endif
#if defined(SECRET_SSID5) && defined(SECRET_PASS5)
  {SECRET_SSID5, SECRET_PASS5},
#endif
};
static const int N_WIFI = sizeof(WIFI_NETS) / sizeof(WIFI_NETS[0]);
static const char* wifiSsid() { return g_wifiNet >= 0 ? WIFI_NETS[g_wifiNet].ssid : ""; }

// `if (Serial)` runs delay(1) on this core, so the USB state is read once per loop
static bool g_serOn = false;

static void emit() {
  if (g_ln.size()) {
    if (g_serOn)    g_txSer.put(g_ln.data(), g_ln.size());
    if (g_clientOn) g_txTcp.put(g_ln.data(), g_ln.size());
  }
  g_ln.reset();
}
static void emitErr(const char* s) { g_ln.reset(); g_ln.print("$ERR,"); g_ln.println(s); emit(); }

// a capture's $CD lines keep the queue under this, so live lines still get through
static const size_t CAP_BACKLOG = 4096;
static size_t sinkBacklog() {
  size_t r = 0;
  if (g_serOn)    r = max(r, g_txSer.used());
  if (g_clientOn) r = max(r, g_txTcp.used());
  return r;
}
static bool anySink() { return g_serOn || g_clientOn; }

static void serviceTx() {
  const uint8_t* p;
  if (g_serOn) {
    size_t n = g_txSer.peek(&p);
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
//  ADS1256 low level (ADC thread only)
// ═════════════════════════════════════════════════════════════════════════════

static inline void csLo() { g_csOut->write(0); }
static inline void csHi() { g_csOut->write(1); }
static inline bool drdyLow() { return g_drdyIn->read() == 0; }

static inline void xfer(uint8_t* b, size_t n) { ADS_SPI.transfer(b, n); }

static void adsCmd(uint8_t c, uint32_t after_us) {
  csLo(); xfer(&c, 1); csHi();
  delayMicroseconds(after_us);
}
static void adsWreg(uint8_t r, uint8_t v) {
  uint8_t b[3] = {(uint8_t)(C_WREG | r), 0x00, v};
  csLo(); xfer(b, 3); csHi();
  delayMicroseconds(T11_US);
}
static void adsRregs(uint8_t r, uint8_t* out, uint8_t n) {
  uint8_t b[2] = {(uint8_t)(C_RREG | r), (uint8_t)(n - 1)};
  csLo(); xfer(b, 2);
  delayMicroseconds(T6_US);
  memset(out, 0, n);
  xfer(out, n);
  csHi();
  delayMicroseconds(T11_US);
}
// DIN while reading data. Bench-found 2026-11-11 (Giga + generic module on
// jumper wires): a DIN edge right as the data clocks start couples into SCLK,
// the ADS1256 sees an extra clock, and the value arrives shifted (exactly x2,
// or wrapped negative). After the RDATA command (0x01, DIN ends high) the fill
// is 0xFF; in RDATAC (DIN already low) it is 0x00. Either way DIN doesn't move
// when the read starts. The real fix is the slow COPI/SCK slew (spiSlew()).
// GPIO output speed of SCK (D13 = PH6) and COPI (D11 = PJ10), 0 low .. 3 very
// high. mbed leaves them at very high; on jumper wires the fast COPI edge
// coupled into SCK (36 % of Multi values doubled). At 0 (or 1): none in 37 500.
static void spiSlew(uint32_t sck, uint32_t copi) {
  core_util_critical_section_enter();
  GPIOH->OSPEEDR = (GPIOH->OSPEEDR & ~(3UL << (6 * 2)))  | ((sck & 3)  << (6 * 2));
  GPIOJ->OSPEEDR = (GPIOJ->OSPEEDR & ~(3UL << (10 * 2))) | ((copi & 3) << (10 * 2));
  core_util_critical_section_exit();
}
static uint32_t slewSck()  { return (GPIOH->OSPEEDR >> (6 * 2)) & 3; }
static uint32_t slewCopi() { return (GPIOJ->OSPEEDR >> (10 * 2)) & 3; }

static inline int32_t read24(uint8_t fill) {
  uint8_t b[3] = {fill, fill, fill};
  xfer(b, 3);
  int32_t v = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2];
  if (v & 0x800000) v -= 0x1000000;
  return v;
}
static inline bool isClip(int32_t v) { return v >= 0x7FFFFF || v <= -0x800000; }

// Start a conversion on `mux`: the digital filter restarts, so the first DRDY
// is a fully settled result.
static void startConv(uint8_t mux) {
  uint8_t b[3] = {(uint8_t)(C_WREG | R_MUX), 0x00, mux};
  uint8_t q = C_SDATAC;
  csLo();
  xfer(&q, 1);                          // in case a scan left the chip in RDATAC
  delayMicroseconds(T11S_US);
  xfer(b, 3);
  delayMicroseconds(T11_US);
  uint8_t s = C_SYNC;   xfer(&s, 1);
  delayMicroseconds(T11S_US);
  uint8_t w = C_WAKEUP; xfer(&w, 1);
  csHi();
}

// One pipelined mux-cycling step (datasheet "cycling the input multiplexer"):
// called with DRDY low, it points the mux at the next channel, restarts the
// conversion, then reads the result of the channel that just finished.
static int32_t stepScan(uint8_t nextMux) {
  uint8_t b[3] = {(uint8_t)(C_WREG | R_MUX), 0x00, nextMux};
  csLo();
  xfer(b, 3);
  delayMicroseconds(T11_US);
  uint8_t s = C_SYNC;   xfer(&s, 1);
  delayMicroseconds(T11S_US);
  uint8_t w = C_WAKEUP; xfer(&w, 1);
  delayMicroseconds(T11_US);
  uint8_t r = C_RDATA;  xfer(&r, 1);
  delayMicroseconds(T6_US);
  int32_t v = read24(0xFF);
  csHi();
  return v;
}

// Wait for DRDY low, sleeping (so lower threads run) for waits over ~1 ms.
static bool waitDrdyLow(uint32_t ms) {
  uint32_t t0 = micros();
  while (!drdyLow()) {
    uint32_t el = micros() - t0;
    if (el > ms * 1000UL) return false;
    if (el > 1000) rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

// a conversion's worst-case time at the applied rate (settling after SYNC)
static uint32_t convTimeoutMs() {
  float sps = drateSps(g_applied.drate);
  if (sps <= 0) sps = 2.5f;
  return (uint32_t)(3000.0f / sps) + 50;
}

static bool adsSelfCal() {
  uint32_t t0 = millis();
  adsCmd(C_WAKEUP, T11_US);
  adsCmd(C_SELFCAL, 5);
  // DRDY goes high when the calibration starts, low when it is done
  uint32_t th = micros();
  while (drdyLow() && micros() - th < 2000) {}
  bool ok = waitDrdyLow(4000);
  g_calMs = millis() - t0;
  g_calOk = ok;
  g_calCount++;
  return ok;
}

static void adsWriteRegs(const AdcCfg& c) {
  adsCmd(C_WAKEUP, T11_US);
  adsCmd(C_SDATAC, T11S_US);
  adsWreg(R_STATUS, c.buf ? 0x02 : 0x00);   // BUFEN is bit 1 (ORDER 0, ACAL 0)
  adsWreg(R_ADCON, c.pga & 0x07);           // CLKOUT off, sensor detect off
  adsWreg(R_DRATE, c.drate);
}

static void setAdcState(uint8_t s, uint8_t st) {
  g_adcStReg = st;
  if (g_adcState != s) { g_adcState = s; g_adcEvents++; }
}

static bool adsInit() {
  csHi();
  if (PIN_RST >= 0) {
    digitalWrite(PIN_RST, LOW);
    delayMicroseconds(20);
    digitalWrite(PIN_RST, HIGH);
  }
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(30));
  adsCmd(C_RESET, 10);
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(30));
  waitDrdyLow(500);
  adsCmd(C_SDATAC, T11S_US);
  uint8_t st = 0;
  adsRregs(R_STATUS, &st, 1);
  // the top nibble of STATUS is the ID: 3 on an ADS1256 / ADS1255
  bool ok = (st >> 4) == 3;
  setAdcState(ok ? ADC_OK : ADC_MISSING, st);
  return ok;
}

static void snapshotCfg(AdcCfg& c, uint32_t& seq) {
  core_util_critical_section_enter();
  c = g_cfg;
  seq = g_cfgSeq;
  core_util_critical_section_exit();
}

// Bring the chip to the requested settings: registers + self-cal when the
// rate, gain or buffer changed (or a cal was asked for); mux map and mask
// changes need nothing here, the next scan picks them up.
static void applyCfg(bool force) {
  AdcCfg c; uint32_t seq;
  snapshotCfg(c, seq);
  bool regs = force || c.drate != g_applied.drate || c.pga != g_applied.pga || c.buf != g_applied.buf;
  bool cal = regs || g_calReq;
  g_calReq = false;
  g_applied = c;
  g_appliedSeq = seq;
  if (regs) adsWriteRegs(c);
  if (cal) adsSelfCal();
}

static void readRegs() {
  adsCmd(C_WAKEUP, T11_US);
  adsCmd(C_SDATAC, T11S_US);
  adsRregs(0, g_regs, N_REGS);
  g_regReq = false;
  g_regReady = true;
}

static bool needBreak() {
  return g_cfgSeq != g_appliedSeq || g_calReq || g_initReq || g_regReq || g_capState == CS_REQ || !g_liveOn;
}

// osThreadFlagsWait returns an error code (top bit set) on a timeout
static inline uint32_t waitFlags(uint32_t ms) {
  uint32_t f = osThreadFlagsWait(F_DRDY | F_POKE, osFlagsWaitAny, ms);
  return (f & 0x80000000UL) ? 0 : f;
}

// ── live scanning ─────────────────────────────────────────────────────────────
static void accReset(LiveAcc& a) { a.n = 0; a.sum = 0; a.sq = 0; a.clip = 0; a.mn = INT32_MAX; a.mx = INT32_MIN; a.ref = 0; }
static inline void accAdd(LiveAcc& a, int32_t v) {
  if (a.n == 0) a.ref = v;
  int32_t d = v - a.ref;
  a.sum += d; a.sq += (double)d * (double)d; a.n++;
  if (v < a.mn) a.mn = v;
  if (v > a.mx) a.mx = v;
  if (isClip(v)) a.clip = 1;
}

static void runLive() {
  uint8_t ord[8]; int n = 0;
  for (int i = 0; i < g_applied.nmap; i++) if (g_applied.mask & (1u << i)) ord[n++] = (uint8_t)i;
  if (n == 0 || g_adcState != ADC_OK) {
    adsCmd(C_STANDBY, T11_US);
    osThreadFlagsWait(F_POKE, osFlagsWaitAny, 500);
    return;
  }
  for (int i = 0; i < 8; i++) accReset(g_acc[i]);
  uint32_t frames = 0, lastPub = us_ticker_read();
  const uint32_t tmo = convTimeoutMs();
  uint32_t lastEdgeMs = millis();

  osThreadFlagsClear(F_DRDY);
  g_wantDrdy = true;
  startConv(g_applied.mux[ord[0]]);
  uint32_t cnt0 = g_drdyCnt;
  int k = 0;
  while (!needBreak()) {
    waitFlags(50);
    if (g_drdyCnt == cnt0 || !drdyLow()) {
      if (millis() - lastEdgeMs > tmo) {        // lost a conversion: start it again
        g_drdyTimeouts++;
        lastEdgeMs = millis();
        startConv(g_applied.mux[ord[k]]);
        cnt0 = g_drdyCnt;
      }
      continue;
    }
    int next = (k + 1 == n) ? 0 : k + 1;
    int32_t v = stepScan(g_applied.mux[ord[next]]);
    osThreadFlagsClear(F_DRDY);
    cnt0 = g_drdyCnt;
    lastEdgeMs = millis();
    g_convCount++;
    accAdd(g_acc[ord[k]], v);
    if (next == 0) {
      frames++;
      if (g_liveReq && !g_liveReady) {
        uint32_t now = us_ticker_read();
        memcpy(g_pub, g_acc, sizeof(g_pub));
        g_pubFrames = frames;
        g_pubUs = now - lastPub;
        g_pubMask = g_applied.mask;
        lastPub = now;
        frames = 0;
        for (int i = 0; i < 8; i++) accReset(g_acc[i]);
        g_liveReq = false;
        g_liveReady = true;
      }
    }
    k = next;
  }
  g_wantDrdy = false;
}

// ── captures ──────────────────────────────────────────────────────────────────
static inline bool crossed(uint8_t slope, int32_t prev, int32_t v, int32_t lvl) {
  return slope == 0 ? (prev < lvl && lvl <= v) : (prev > lvl && lvl >= v);
}

static void capFail(const char* why) {
  strncpy(g_capFail, why, sizeof(g_capFail) - 1);
  g_capFail[sizeof(g_capFail) - 1] = 0;
  g_capState = CS_FAIL;
}

static void runCapture() {
  const CapReq r = g_cr;
  const uint32_t nF = r.n, nch = r.nch, pre = r.pre;
  const uint32_t tmo = convTimeoutMs();
  uint32_t f = 0, slot = 0, gaps = 0;
  int64_t trig = r.armed ? -1 : 0;
  int32_t prev = 0;
  bool havePrev = false;
  uint8_t flags = 0;
  const uint32_t t0 = millis();
  uint32_t lastEdgeMs = t0;
  uint8_t ending = 0;          // 0 running, 1 done, 2 timeout, 3 abort, 4 no DRDY

  g_capState = r.armed ? CS_WAIT : CS_POST;

  if (r.mode == 0) {
    // ---- benchmark: one channel, read data continuous ----
    osThreadFlagsClear(F_DRDY);
    startConv(g_applied.mux[r.order[0]]);
    if (!waitDrdyLow(tmo)) { capFail("no DRDY"); return; }
    uint8_t c = C_RDATAC;
    csLo(); xfer(&c, 1); delayMicroseconds(T6_US); (void)read24(0x00); csHi();
    osThreadFlagsClear(F_DRDY);
    uint32_t lastCnt = g_drdyCnt;
    g_wantDrdy = true;
    while (!ending) {
      waitFlags(50);
      if (g_capAbort) { ending = 3; break; }
      uint32_t cnt = g_drdyCnt;
      if (cnt == lastCnt) {
        if (millis() - lastEdgeMs > tmo) ending = 4;
        continue;
      }
      uint32_t ts = g_drdyTs;
      csLo(); int32_t v = read24(0x00); csHi();
      if (cnt - lastCnt > 1) gaps += cnt - lastCnt - 1;
      lastCnt = cnt;
      // a DRDY edge during the read: the output register changed under it, so
      // the value may be torn. Drop it and count it as a lost conversion.
      if (g_drdyCnt != cnt) { gaps++; lastEdgeMs = millis(); continue; }
      lastEdgeMs = millis();
      g_convCount++;
      g_ct[slot] = ts;
      g_cv[slot] = v;
      if (isClip(v)) flags |= 2;
      if (trig < 0) {
        if (f >= pre && havePrev && crossed(r.slope, prev, v, r.level)) {
          trig = f; g_capWaitMs = millis() - t0; g_capState = CS_POST;
        } else if (r.timeoutMs && millis() - t0 > r.timeoutMs) {
          ending = 2;
        }
      }
      prev = v; havePrev = true;
      f++;
      if (++slot == nF) slot = 0;
      if (trig >= 0 && (int64_t)f >= trig + (int64_t)(nF - pre)) ending = 1;
    }
    g_wantDrdy = false;
    // leave RDATAC right after a conversion has been read
    waitDrdyLow(tmo);
    csLo(); c = C_SDATAC; xfer(&c, 1); csHi();
    delayMicroseconds(T11S_US);
  } else {
    // ---- multi: pipelined mux cycling, order[0] (trigger channel) first ----
    osThreadFlagsClear(F_DRDY);
    g_wantDrdy = true;
    startConv(g_applied.mux[r.order[0]]);
    uint32_t cnt0 = g_drdyCnt, frameTs = 0;
    uint32_t k = 0;
    while (!ending) {
      waitFlags(50);
      if (g_capAbort) { ending = 3; break; }
      if (g_drdyCnt == cnt0 || !drdyLow()) {
        if (millis() - lastEdgeMs > tmo) ending = 4;
        continue;
      }
      uint32_t ts = g_drdyTs;
      uint32_t next = (k + 1 == nch) ? 0 : k + 1;
      int32_t v = stepScan(g_applied.mux[r.order[next]]);
      osThreadFlagsClear(F_DRDY);
      cnt0 = g_drdyCnt;
      lastEdgeMs = millis();
      g_convCount++;
      g_cv[slot * nch + k] = v;
      if (isClip(v)) flags |= 2;
      if (k == 0) {
        frameTs = ts;
        if (trig < 0) {
          if (f >= pre && havePrev && crossed(r.slope, prev, v, r.level)) {
            trig = f; g_capWaitMs = millis() - t0; g_capState = CS_POST;
          } else if (r.timeoutMs && millis() - t0 > r.timeoutMs) {
            ending = 2;
          }
        }
        prev = v; havePrev = true;
      }
      if (next == 0) {
        g_ct[slot] = frameTs;
        f++;
        if (++slot == nF) slot = 0;
        if (trig >= 0 && (int64_t)f >= trig + (int64_t)(nF - pre)) ending = 1;
      }
      k = next;
    }
    g_wantDrdy = false;
  }
  adsCmd(C_STANDBY, T11_US);

  if (ending == 1) {
    if (gaps) flags |= 1;
    g_capGaps = gaps;
    g_capFlags = flags;
    g_capFirst = (uint32_t)((uint64_t)(trig - (int64_t)pre) % nF);   // ring slot of output frame 0
    g_capState = CS_READY;
  } else if (ending == 2) {
    g_capState = CS_TIMEOUT;
  } else if (ending == 3) {
    g_capState = CS_ABORT;
  } else {
    capFail("no DRDY");
  }
}

// ── the ADC thread ────────────────────────────────────────────────────────────
static void adcTask() {
  g_adcTid = osThreadGetId();
  for (;;) {
    if (g_initReq || g_adcState != ADC_OK) {
      g_initReq = false;
      if (adsInit()) {
        applyCfg(true);
      } else {
        if (g_capState == CS_REQ) capFail("ADS1256 not found");
        osThreadFlagsWait(F_POKE, osFlagsWaitAny, 3000);
      }
      continue;
    }
    if (g_cfgSeq != g_appliedSeq || g_calReq) { applyCfg(false); continue; }
    if (g_regReq) { readRegs(); continue; }
    if (g_capState == CS_REQ) { runCapture(); continue; }
    if (g_liveOn) { runLive(); continue; }
    adsCmd(C_STANDBY, T11_US);
    osThreadFlagsWait(F_POKE, osFlagsWaitAny, 500);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  loop-thread side: reports, capture sending, commands
// ═════════════════════════════════════════════════════════════════════════════

static const char HEXD[] = "0123456789ABCDEF";
static inline char* hex8(char* p, uint32_t v) { for (int s = 28; s >= 0; s -= 4) *p++ = HEXD[(v >> s) & 15]; return p; }
static inline char* hex6(char* p, int32_t v) {
  uint32_t u = (uint32_t)v & 0xFFFFFF;
  for (int s = 20; s >= 0; s -= 4) *p++ = HEXD[(u >> s) & 15];
  return p;
}

static void printHex2(uint8_t v) { g_ln.print(HEXD[v >> 4]); g_ln.print(HEXD[v & 15]); }

static void printNet() {
  g_ln.reset(); g_ln.print("$NET,"); g_ln.print(g_ipStr); g_ln.print(','); g_ln.print(CMD_PORT); g_ln.print(',');
  g_ln.print(g_wifiOk ? (int)WiFi.RSSI() : 0); g_ln.print(',');
  g_ln.print(g_wifiStatus); g_ln.print(','); g_ln.print(g_wifiPs); g_ln.print(',');
  g_ln.print(wifiSsid()); g_ln.print(','); g_ln.print(g_wifiNet + 1); g_ln.print(',');
  g_ln.print(N_WIFI); g_ln.print(','); g_ln.println((int)g_wifiSeen); emit();
}

static void printStatus() {
  static const char* ADCS[] = {"INIT", "OK", "MISSING"};
  g_ln.reset();
  g_ln.print("$STATUS,fw="); g_ln.print(FW_VERSION);
  g_ln.print(",adc="); g_ln.print(ADCS[g_adcState % 3]);
  g_ln.print(",streg=0x"); printHex2(g_adcStReg);
  g_ln.print(",drate=0x"); printHex2(g_cfg.drate);
  g_ln.print(",sps="); g_ln.print(drateSps(g_cfg.drate), 1);
  g_ln.print(",pga="); g_ln.print(g_cfg.pga);
  g_ln.print(",buf="); g_ln.print(g_cfg.buf);
  g_ln.print(",nmap="); g_ln.print(g_cfg.nmap);
  g_ln.print(",mask=0x"); printHex2(g_cfg.mask);
  g_ln.print(",live="); g_ln.print(g_liveOn ? 1 : 0);
  g_ln.print(",livems="); g_ln.print(g_liveMs);
  g_ln.print(",conv_s="); g_ln.print(g_convRate, 1);
  g_ln.print(",drdy_tmo="); g_ln.print(g_drdyTimeouts);
  g_ln.print(",cap="); g_ln.print(g_capState);
  g_ln.print(",capmax_frames="); g_ln.print(CAP_FRAMES);
  g_ln.print(",capmax_vals="); g_ln.print(CAP_VALS);
  g_ln.print(",range="); g_ln.print(g_rangeHi);
  g_ln.print(",cal_ms="); g_ln.print(g_calMs);
  g_ln.print(",slew="); g_ln.print(slewSck()); g_ln.print(':'); g_ln.print(slewCopi());
  g_ln.print(",ip="); g_ln.print(g_ipStr);
  g_ln.print(",rssi="); g_ln.print(g_wifiOk ? (int)WiFi.RSSI() : 0);
  g_ln.print(",ssid="); g_ln.print(wifiSsid());
  g_ln.print(",net="); g_ln.print(g_wifiNet + 1);
  g_ln.print(",uptime_s="); g_ln.print(millis() / 1000UL);
  g_ln.print(",loopms="); g_ln.print(g_loopMaxMs, 2);
  g_ln.print(",txdrops="); g_ln.print(g_txTcp.drops() + g_txSer.drops());
  g_ln.print(",tcpbusy="); g_ln.println(g_tcpBusy);
  emit();
}

static void printGpioList() {
  g_ln.reset(); g_ln.print("$GPIOLIST");
  for (int i = 0; i < N_GPIO; i++) { g_ln.print(','); g_ln.print(GPIO_PINS[i]); }
  g_ln.println(); emit();
}

static int gpioLevel(int i) {
  if (g_gpioMode[i] == 1) return 0;
  if (g_gpioMode[i] == 2) return 1;
  return digitalRead(GPIO_PINS[i]) ? 1 : 0;
}

static void printGpios() {
  g_ln.reset(); g_ln.print("$GPIOS");
  for (int i = 0; i < N_GPIO; i++) {
    g_ln.print(','); g_ln.print(GPIO_PINS[i]); g_ln.print(':');
    g_ln.print(g_gpioMode[i]); g_ln.print(':'); g_ln.print(gpioLevel(i));
  }
  g_ln.println(); emit();
}

static void setGpio(int i, uint8_t mode) {
  uint8_t pin = GPIO_PINS[i];
  if (mode == 0)      pinMode(pin, INPUT);
  else if (mode == 3) pinMode(pin, INPUT_PULLUP);
  else { pinMode(pin, OUTPUT); digitalWrite(pin, mode == 2 ? HIGH : LOW); }
  g_gpioMode[i] = mode;
}

static void printBanner() {
  g_ln.reset(); g_ln.print("$READY,GIGA_ADS1256,"); g_ln.println(FW_VERSION); emit();
  printStatus();
  printNet();
  printGpioList();
}

// ── reports from the ADC thread ─────────────────────────────────────────────
static uint32_t g_seenAdcEv = 0, g_seenCal = 0, g_lastLiveReq = 0;

static void printLive() {
  g_ln.reset();
  g_ln.print("$LIVE,"); g_ln.print(g_pubFrames); g_ln.print(',');
  g_ln.print(g_pubUs / 1000.0f, 1); g_ln.print(',');
  int n = 0;
  for (int i = 0; i < 8; i++) if ((g_pubMask & (1u << i)) && g_pub[i].n) n++;
  g_ln.print(n);
  for (int i = 0; i < 8; i++) {
    const LiveAcc& a = g_pub[i];
    if (!(g_pubMask & (1u << i)) || !a.n) continue;
    double mean = (double)a.sum / a.n;
    double var = a.sq / a.n - mean * mean;
    g_ln.print(','); g_ln.print(i);
    g_ln.print(','); g_ln.print((float)(a.ref + mean), 2);
    g_ln.print(','); g_ln.print((float)(var > 0 ? sqrt(var) : 0.0), 2);
    g_ln.print(','); g_ln.print(a.mn);
    g_ln.print(','); g_ln.print(a.mx);
    g_ln.print(','); g_ln.print(a.clip);
  }
  g_ln.println();
  emit();
}

// capture sending
static bool     g_sending = false;
static uint32_t g_sendIdx = 0, g_sendT0 = 0;
static uint8_t  g_seenCap = CS_IDLE;

static uint32_t capSlot(uint32_t i) {
  uint32_t s = g_capFirst + i;
  return s >= g_cr.n ? s - g_cr.n : s;
}

static void startSend() {
  const uint32_t n = g_cr.n, nch = g_cr.nch;
  uint32_t tFirst = g_ct[capSlot(0)], tLast = g_ct[capSlot(n - 1)];
  float dt = n > 1 ? (float)(tLast - tFirst) / (float)(n - 1) : 0.0f;
  g_ln.reset();
  g_ln.print("$CAP,"); g_ln.print(n); g_ln.print(','); g_ln.print(nch); g_ln.print(',');
  for (uint32_t k = 0; k < nch; k++) { if (k) g_ln.print(':'); g_ln.print(g_cr.order[k]); }
  g_ln.print(','); g_ln.print(g_cr.armed ? (long)g_cr.pre : -1L);
  g_ln.print(','); g_ln.print(g_capGaps);
  g_ln.print(','); g_ln.print(g_capFlags);
  g_ln.print(','); g_ln.print(dt, 3);
  g_ln.print(",0x"); printHex2(g_applied.drate);
  g_ln.print(','); g_ln.print(g_applied.pga);
  g_ln.print(','); g_ln.println(g_applied.buf);
  emit();
  g_sendT0 = tFirst;
  g_sendIdx = 0;
  g_sending = true;
}

static void endCapture() { g_sending = false; g_capState = CS_IDLE; g_seenCap = CS_IDLE; }

static void serviceSend() {
  if (!g_sending) return;
  if (!anySink()) { endCapture(); return; }
  if (g_capAbort) { g_ln.reset(); g_ln.println("$CAPTMO,ABORT"); emit(); endCapture(); return; }
  const uint32_t n = g_cr.n, nch = g_cr.nch;
  const uint32_t per = 8 + 6 * nch;
  uint32_t fpl = 240 / per;
  if (fpl < 1) fpl = 1;
  static char buf[400];
  for (int lines = 0; lines < 8 && g_sending && sinkBacklog() < CAP_BACKLOG; lines++) {
    uint32_t end = g_sendIdx + fpl;
    if (end > n) end = n;
    g_ln.reset();
    g_ln.print("$CD,"); g_ln.print(g_sendIdx); g_ln.print(',');
    char* p = buf;
    for (uint32_t i = g_sendIdx; i < end; i++) {
      uint32_t s = capSlot(i);
      p = hex8(p, g_ct[s] - g_sendT0);
      const int32_t* row = g_cv + (size_t)s * nch;
      for (uint32_t k = 0; k < nch; k++) p = hex6(p, row[k]);
    }
    *p++ = '\r'; *p++ = '\n';
    g_ln.write((const uint8_t*)buf, p - buf);
    emit();
    g_sendIdx = end;
    if (g_sendIdx >= n) {
      g_ln.reset(); g_ln.print("$CEND,"); g_ln.println(n); emit();
      endCapture();
    }
  }
}

static void serviceAdcReports() {
  if (g_adcEvents != g_seenAdcEv) {
    g_seenAdcEv = g_adcEvents;
    g_ln.reset(); g_ln.print("$ADC,"); g_ln.print(g_adcState == ADC_OK ? "OK" : "MISSING");
    g_ln.print(",0x"); printHex2(g_adcStReg); g_ln.println(); emit();
  }
  if (g_calCount != g_seenCal) {
    g_seenCal = g_calCount;
    g_ln.reset(); g_ln.print("$CAL,"); g_ln.print(g_calOk ? 1 : 0); g_ln.print(','); g_ln.println(g_calMs); emit();
  }
  if (g_regReady) {
    g_ln.reset(); g_ln.print("$REG");
    for (int i = 0; i < N_REGS; i++) { g_ln.print(','); printHex2(g_regs[i]); }
    g_ln.println(); emit();
    g_regReady = false;
  }
  if (g_liveReady) { printLive(); g_liveReady = false; }
  uint32_t ms = millis();
  if (g_liveOn && !g_liveReq && !g_liveReady && ms - g_lastLiveReq >= g_liveMs) {
    g_lastLiveReq = ms;
    g_liveReq = true;
  }

  uint8_t cs = g_capState;
  if (cs != g_seenCap) {
    g_seenCap = cs;
    if (cs == CS_WAIT) { g_ln.reset(); g_ln.println("$CAPARM"); emit(); }
    else if (cs == CS_POST && g_cr.armed) { g_ln.reset(); g_ln.print("$CAPTRIG,"); g_ln.println(g_capWaitMs); emit(); }
    else if (cs == CS_READY) startSend();
    else if (cs == CS_TIMEOUT) { g_ln.reset(); g_ln.println("$CAPTMO,TIMEOUT"); emit(); g_capState = CS_IDLE; g_seenCap = CS_IDLE; }
    else if (cs == CS_ABORT)   { g_ln.reset(); g_ln.println("$CAPTMO,ABORT");   emit(); g_capState = CS_IDLE; g_seenCap = CS_IDLE; }
    else if (cs == CS_FAIL)    { g_ln.reset(); g_ln.print("$ERR,capture: "); g_ln.println(g_capFail); emit(); g_capState = CS_IDLE; g_seenCap = CS_IDLE; }
  }
}

// ── commands ──────────────────────────────────────────────────────────────────
static bool capBusy() { return g_capState != CS_IDLE; }

// ── DAC test source (!DAC / !DACSINE) ─────────────────────────────────────────
static mbed::AnalogOut* g_dac[2] = {nullptr, nullptr};
static bool  g_dacSine[2] = {false, false};
static float g_dacHz[2], g_dacVpp[2], g_dacOff[2];

static void dacInit() {
  if (!g_dac[0]) g_dac[0] = new mbed::AnalogOut(PA_4);   // A12
  if (!g_dac[1]) g_dac[1] = new mbed::AnalogOut(PA_5);   // A13
}
static void serviceDac() {
  if (!g_dacSine[0] && !g_dacSine[1]) return;
  double t = us_ticker_read() * 1e-6;    // wraps every 71 min: one phase jump, fine for a test source
  for (int ch = 0; ch < 2; ch++) {
    if (!g_dacSine[ch]) continue;
    float v = g_dacOff[ch] + 0.5f * g_dacVpp[ch] * (float)sin(2.0 * M_PI * g_dacHz[ch] * t);
    g_dac[ch]->write(constrain(v, 0.0f, 3.3f) / 3.3f);
  }
}

// edit g_cfg between these two as one step the ADC thread will see whole
static void cfgLock() { core_util_critical_section_enter(); }
static void cfgUnlock() {
  g_cfgSeq = g_cfgSeq + 1;
  core_util_critical_section_exit();
  pokeAdc();
}

static int parseInts(const char* s, long* out, int n) {
  int k = 0;
  while (s && *s && k < n) {
    char* e;
    long v = strtol(s, &e, 0);
    if (e == s) break;
    out[k++] = v;
    s = (*e == ',') ? e + 1 : nullptr;
  }
  return k;
}

static void startCapture(const char* args) {
  long a[9];
  if (parseInts(args, a, 9) < 9) { emitErr("usage: !CAP,mode,n,mask,trig_ch,armed,level,slope,pos,timeout_ms"); return; }
  if (capBusy()) { emitErr("busy: a capture is running"); return; }
  if (g_adcState != ADC_OK) { emitErr("capture: ADS1256 not found"); return; }
  CapReq r;
  memset(&r, 0, sizeof(r));
  r.mode = a[0] ? 1 : 0;
  const int nmap = g_cfg.nmap;
  long trig = a[3];
  if (trig < 0 || trig >= nmap) { emitErr("capture: bad trigger channel"); return; }
  r.order[0] = (uint8_t)trig;
  r.nch = 1;
  if (r.mode == 1) {
    for (int i = 0; i < nmap; i++)
      if ((a[2] & (1L << i)) && i != trig) r.order[r.nch++] = (uint8_t)i;
  }
  if (a[1] < 1) { emitErr("capture: n must be >= 1"); return; }
  r.n = (uint32_t)a[1];
  if (r.n > CAP_FRAMES || (uint64_t)r.n * r.nch > CAP_VALS) {
    g_ln.reset(); g_ln.print("$ERR,capture: too long, max "); g_ln.print(min(CAP_FRAMES, CAP_VALS / r.nch));
    g_ln.println(" frames for this many channels"); emit();
    return;
  }
  r.armed = a[4] ? 1 : 0;
  r.level = (int32_t)a[5];
  r.slope = a[6] ? 1 : 0;
  long pos = a[7];
  r.pre = !r.armed ? 0 : pos == 1 ? r.n / 2 : pos == 2 ? r.n - 1 : 0;
  r.timeoutMs = a[8] > 0 ? (uint32_t)a[8] : 0;
  g_cr = r;
  g_capAbort = false;
  g_seenCap = CS_IDLE;
  g_ln.reset(); g_ln.print("$OK,CAP,"); g_ln.print(r.n); g_ln.print(','); g_ln.print(r.nch); g_ln.print(',');
  for (int k = 0; k < r.nch; k++) { if (k) g_ln.print(':'); g_ln.print(r.order[k]); }
  g_ln.print(','); g_ln.println(r.pre); emit();
  g_capState = CS_REQ;
  pokeAdc();
}

static void startWifiConnect(bool disconnectFirst);
static void dropClient();

static void handleCommand(const char* cmd) {
  const char* args = strchr(cmd, ',');
  if (args) args++;
  size_t nlen = args ? (size_t)(args - 1 - cmd) : strlen(cmd);
  auto is = [&](const char* name) { return strlen(name) == nlen && strncmp(cmd, name, nlen) == 0; };
  long a[8];
  int na = parseInts(args, a, 8);
  const bool cfgBusy = g_capState >= CS_REQ && g_capState <= CS_POST;

  if (is("!ID")) {
    g_ln.reset(); g_ln.print("$READY,GIGA_ADS1256,"); g_ln.println(FW_VERSION); emit();
  } else if (is("!STATUS")) {
    printStatus();
  } else if (is("!NET")) {
    printNet();
  } else if (is("!DRATE")) {
    if (na < 1 || drateSps((uint8_t)a[0]) <= 0) { emitErr("DRATE: not a DRATE code (0xF0 = 30k SPS ... 0x03 = 2.5 SPS)"); return; }
    if (cfgBusy) { emitErr("busy: capture running"); return; }
    cfgLock(); g_cfg.drate = (uint8_t)a[0]; cfgUnlock();
    g_ln.reset(); g_ln.print("$OK,DRATE,0x"); printHex2((uint8_t)a[0]); g_ln.println(); emit();
  } else if (is("!GAIN")) {
    if (na < 1 || a[0] < 0 || a[0] > 6) { emitErr("GAIN: 0..6 (PGA 1..64)"); return; }
    if (cfgBusy) { emitErr("busy: capture running"); return; }
    cfgLock(); g_cfg.pga = (uint8_t)a[0]; cfgUnlock();
    g_ln.reset(); g_ln.print("$OK,GAIN,"); g_ln.println(a[0]); emit();
  } else if (is("!BUF")) {
    if (na < 1) { emitErr("BUF: 0 or 1"); return; }
    if (cfgBusy) { emitErr("busy: capture running"); return; }
    cfgLock(); g_cfg.buf = a[0] ? 1 : 0; cfgUnlock();
    g_ln.reset(); g_ln.print("$OK,BUF,"); g_ln.println(a[0] ? 1 : 0); emit();
  } else if (is("!CAL")) {
    if (cfgBusy) { emitErr("busy: capture running"); return; }
    g_calReq = true; pokeAdc();
    g_ln.reset(); g_ln.println("$OK,CAL"); emit();
  } else if (is("!CHMAP")) {
    if (na < 1) { emitErr("CHMAP: 1..8 MUX bytes"); return; }
    if (cfgBusy) { emitErr("busy: capture running"); return; }
    for (int i = 0; i < na; i++) {
      int p = (a[i] >> 4) & 15, n = a[i] & 15;
      if (a[i] < 0 || a[i] > 0xFF || p > 8 || n > 8) { emitErr("CHMAP: each MUX byte is (AINP<<4)|AINN, 0..8"); return; }
    }
    cfgLock();
    g_cfg.nmap = (uint8_t)na;
    for (int i = 0; i < na; i++) g_cfg.mux[i] = (uint8_t)a[i];
    g_cfg.mask &= (uint8_t)((1u << na) - 1);
    cfgUnlock();
    g_ln.reset(); g_ln.print("$OK,CHMAP,"); g_ln.println(na); emit();
  } else if (is("!CH")) {
    if (na < 1) { emitErr("CH: channel bit mask"); return; }
    uint8_t m = (uint8_t)(a[0] & ((1u << g_cfg.nmap) - 1));
    cfgLock(); g_cfg.mask = m; cfgUnlock();
    g_ln.reset(); g_ln.print("$OK,CH,0x"); printHex2(m); g_ln.println(); emit();
  } else if (is("!LIVE")) {
    if (na >= 1) g_liveOn = a[0] != 0;
    if (na >= 2) g_liveMs = constrain(a[1], 50L, 10000L);
    pokeAdc();
    g_ln.reset(); g_ln.print("$OK,LIVE,"); g_ln.print(g_liveOn ? 1 : 0); g_ln.print(','); g_ln.println(g_liveMs); emit();
  } else if (is("!RANGE")) {
    if (na < 1) { emitErr("RANGE: 0 or 1"); return; }
    g_rangeHi = a[0] ? 1 : 0;
    digitalWrite(PIN_RANGE, g_rangeHi ? HIGH : LOW);
    g_ln.reset(); g_ln.print("$OK,RANGE,"); g_ln.println(g_rangeHi); emit();
  } else if (is("!CAP")) {
    startCapture(args);
  } else if (is("!ABORT")) {
    g_capAbort = true; pokeAdc();
    if (!capBusy()) { g_ln.reset(); g_ln.println("$OK,ABORT,idle"); emit(); }
  } else if (is("!REG")) {
    if (cfgBusy) { emitErr("busy: capture running"); return; }
    g_regReq = true; pokeAdc();
  } else if (is("!REINIT")) {
    if (cfgBusy) { emitErr("busy: capture running"); return; }
    g_initReq = true; pokeAdc();
    g_ln.reset(); g_ln.println("$OK,REINIT"); emit();
  } else if (is("!DAC") || is("!DACSINE")) {
    // test source on the Giga's DACs: A12 (DAC0) and A13 (DAC1), 0..3.3 V
    float f[5] = {0, 0, 0, 0, 0};
    const char* p = args;
    int nf = 0;
    while (p && *p && nf < 5) { char* e; f[nf++] = strtof(p, &e); p = (*e == ',') ? e + 1 : nullptr; }
    int ch = (int)f[0];
    if (nf < 2 || ch < 0 || ch > 1) { emitErr("DAC: !DAC,<0|1>,<volts>  or  !DACSINE,<0|1>,<hz>,<vpp>,<offset_v>"); return; }
    dacInit();
    if (is("!DAC")) {
      g_dacSine[ch] = false;
      g_dac[ch]->write(constrain(f[1], 0.0f, 3.3f) / 3.3f);
    } else {
      g_dacHz[ch] = f[1]; g_dacVpp[ch] = nf > 2 ? f[2] : 1.0f; g_dacOff[ch] = nf > 3 ? f[3] : 1.65f;
      g_dacSine[ch] = true;
    }
    g_ln.reset(); g_ln.print("$OK,"); g_ln.print(is("!DAC") ? "DAC," : "DACSINE,"); g_ln.print(ch);
    for (int i = 1; i < nf; i++) { g_ln.print(','); g_ln.print(f[i], 4); }
    g_ln.println(); emit();
  } else if (is("!SLEW")) {
    // GPIO output speed (0 low .. 3 very high) of SCK (D13 = PH6) and COPI (D11 = PJ10)
    if (na < 2) { emitErr("SLEW: <sck 0..3>,<copi 0..3>"); return; }
    spiSlew((uint32_t)a[0], (uint32_t)a[1]);
    g_ln.reset(); g_ln.print("$OK,SLEW,"); g_ln.print(slewSck()); g_ln.print(','); g_ln.println(slewCopi()); emit();
  } else if (is("!GPIOLIST")) {
    printGpioList();
  } else if (is("!GPIOS")) {
    printGpios();
  } else if (is("!GPIO")) {
    if (na < 2 || a[1] < 0 || a[1] > 3) { emitErr("GPIO: pin,mode (0 in, 1 low, 2 high, 3 pull-up)"); return; }
    int idx = -1;
    for (int i = 0; i < N_GPIO; i++) if (GPIO_PINS[i] == a[0]) idx = i;
    if (idx < 0) { emitErr("GPIO: not a user pin (see !GPIOLIST)"); return; }
    setGpio(idx, (uint8_t)a[1]);
    g_ln.reset(); g_ln.print("$GPIO,"); g_ln.print(GPIO_PINS[idx]); g_ln.print(','); g_ln.print(g_gpioMode[idx]);
    g_ln.print(','); g_ln.println(gpioLevel(idx)); emit();
  } else if (is("!WIFI")) {
    if (g_wifiBusy) emitErr("Already joining");
    else { dropClient(); g_wifiOk = false; g_wifiReconnects++; startWifiConnect(true); printNet(); }
  } else if (is("!SCAN")) {
    if (g_wifiBusy) { emitErr("Wi-Fi is joining, try again"); return; }
    const char* fw = WiFi.firmwareVersion();
    int n = WiFi.scanNetworks();
    g_ln.reset(); g_ln.print("$SCAN,"); g_ln.print(fw ? fw : "?"); g_ln.print(','); g_ln.println(n); emit();
    for (int k = 0; k < n; k++) {
      int listed = 0;
      for (int j = 0; j < N_WIFI && !listed; j++) if (strcmp(WiFi.SSID(k), WIFI_NETS[j].ssid) == 0) listed = j + 1;
      g_ln.reset(); g_ln.print("$AP,"); g_ln.print(WiFi.SSID(k)); g_ln.print(',');
      g_ln.print((int)WiFi.RSSI(k)); g_ln.print(','); g_ln.print((int)WiFi.encryptionType(k)); g_ln.print(',');
      g_ln.println(listed); emit();
    }
  } else {
    g_ln.reset(); g_ln.print("$ERR,unknown command: "); g_ln.println(cmd); emit();
  }
}

static RxLine g_rxSer = {{0}, 0}, g_rxTcp = {{0}, 0};

static void feed(RxLine& rx, int c) {
  if (c == '\n' || c == '\r') {
    if (rx.n) { rx.b[rx.n] = 0; handleCommand(rx.b); rx.n = 0; }
  } else if (rx.n < sizeof(rx.b) - 1) {
    rx.b[rx.n++] = (char)c;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Wi-Fi (from Giga_AC_Power_WiFi: join on its own thread, power-save off,
//  newest client wins, non-blocking socket)
// ═════════════════════════════════════════════════════════════════════════════

static uint32_t g_lastWifiTry = 0, g_lastWifiCheck = 0;
static uint8_t  g_led = 0xFF;
enum : uint8_t { L_OFF, L_RED, L_GREEN, L_BLUE };

// RGB LED (active low): blue while a capture waits for its trigger, red with
// no ADS1256, green with a client connected, off otherwise
static void setLeds() {
  uint8_t c = (g_capState == CS_WAIT) ? L_BLUE
            : g_adcState == ADC_MISSING ? L_RED
            : g_clientOn ? L_GREEN : L_OFF;
  if (c == g_led) return;
  g_led = c;
  digitalWrite(LEDR, c == L_RED   ? LOW : HIGH);
  digitalWrite(LEDG, c == L_GREEN ? LOW : HIGH);
  digitalWrite(LEDB, c == L_BLUE  ? LOW : HIGH);
}

static rtos::Thread* g_wifiThread = nullptr;

static void wifiTask() {
  int st = WiFi.status();
  int joined = -1;
  if (st != WL_NO_MODULE) {
    if (g_wifiDisc) { WiFi.disconnect(); g_wifiDisc = false; }
    uint8_t seen = 0;
    const int n = WiFi.scanNetworks();
    for (int k = 0; k < n; k++) {
      const char* s = WiFi.SSID(k);
      for (int j = 0; j < N_WIFI; j++) if (s && strcmp(s, WIFI_NETS[j].ssid) == 0) seen |= (uint8_t)(1u << j);
    }
    if (n < 0) seen = 0xFF;
    g_wifiSeen = seen;
    for (int j = 0; j < N_WIFI && st != WL_CONNECTED; j++) {
      if (!(seen & (1u << j))) continue;
      for (int t = 0; t < 2 && st != WL_CONNECTED; t++) {
        WiFi.begin(WIFI_NETS[j].ssid, WIFI_NETS[j].pass);
        st = WiFi.status();
      }
      if (st == WL_CONNECTED) joined = j;
    }
  }
  g_wifiJoined = joined;
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
  g_wifiThread = new rtos::Thread(osPriorityNormal, 8192, nullptr, "wifijoin");
  g_wifiThread->start(wifiTask);
}

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
    g_wifiNet = g_wifiJoined;
  } else {
    strcpy(g_ipStr, g_wifiStatus == WL_NO_MODULE ? "no-module" : "none");
    g_wifiOk = false;
    g_wifiNet = -1;
  }
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
  if (g_wifiBusy) return;
  if (g_wifiDone) finishWifiConnect();
  if (g_wifiOk && now - g_lastWifiCheck > 2000) {
    g_lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      g_wifiOk = false; g_wifiNet = -1; dropClient(); strcpy(g_ipStr, "none"); printNet();
    }
  }
  if (!g_wifiOk) {
    if (g_wifiStatus != WL_NO_MODULE && now - g_lastWifiTry > 20000) {
      g_wifiReconnects++;
      startWifiConnect(true);
    }
    return;
  }
  WiFiClient c = g_server.available();
  if (c) {
    // newest connection wins: the Giga's socket pool is tiny
    dropClient();
    g_client = c;
    g_client.setSocketTimeout(0);   // non-blocking send; net_sink.h keeps the rest
    g_clientOn = true;
    printBanner();
  }
  if (g_clientOn && !g_client.connected()) dropClient();
  if (g_clientOn) {
    int guard = 512;
    while (g_client.available() && guard-- > 0) feed(g_rxTcp, g_client.read());
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  setup / loop
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
  pinMode(PIN_RANGE, OUTPUT);
  digitalWrite(PIN_RANGE, g_rangeHi ? HIGH : LOW);   // the Pi GUI's default: high range
  if (PIN_RST >= 0) { pinMode(PIN_RST, OUTPUT); digitalWrite(PIN_RST, HIGH); }
  for (int i = 0; i < N_GPIO; i++) setGpio(i, 0);

  Serial.begin(115200);   // no wait: runs headless

  SDRAM.begin();
  g_cv = (int32_t*)SDRAM.malloc(CAP_VALS * sizeof(int32_t));
  g_ct = (uint32_t*)SDRAM.malloc(CAP_FRAMES * sizeof(uint32_t));
  if (!g_cv || !g_ct) {
    while (1) { Serial.println("$ERR,SDRAM allocation failed"); digitalWrite(LEDR, LOW); delay(1000); }
  }

  g_csOut = new mbed::DigitalOut(digitalPinToPinName(PIN_CS), 1);
  ADS_SPI.begin();
  ADS_SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));
  spiSlew(0, 0);
  g_drdyIn = new mbed::InterruptIn(digitalPinToPinName(PIN_DRDY), PullUp);
  g_drdyIn->fall(mbed::callback(drdyIsr));

  memset(&g_applied, 0, sizeof(g_applied));
  // above the loop: it sleeps between conversions, so the loop and Wi-Fi run
  g_adcThread = new rtos::Thread(osPriorityRealtime, 8192, nullptr, "ads1256");
  g_adcThread->start(adcTask);

  startWifiConnect(false);
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

  serviceAdcReports();
  serviceSend();
  serviceDac();
  setLeds();

  while (Serial.available()) feed(g_rxSer, Serial.read());
  serviceNet();
  serviceTx();

  // once a second: conversion rate and worst loop time
  static uint32_t lastStat = 0, lastConv = 0;
  uint32_t ms = millis();
  if (ms - lastStat >= 1000) {
    uint32_t c = g_convCount;
    g_convRate = lastStat ? (float)(c - lastConv) * 1000.0f / (float)(ms - lastStat) : 0.0f;
    lastConv = c;
    g_loopMaxMs = g_loopMaxUs / 1000.0f;
    g_loopMaxUs = 0;
    lastStat = ms;
  }
  // nothing to do: give the CPU to the Wi-Fi stack for a moment
  if (!g_sending && !g_txTcp.used() && !g_txSer.used()) rtos::ThisThread::yield();
}
