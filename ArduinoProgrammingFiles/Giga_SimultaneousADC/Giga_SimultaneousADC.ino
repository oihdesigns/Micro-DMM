/*
 * Giga_SimultaneousADC.ino - can the Arduino Giga R1 (STM32H747) sample two
 * pins at the same instant, the way 120VMonitor does on the Feather F405?
 *
 * ── Short answer ────────────────────────────────────────────────────────────
 *   Yes. The H747 has ADC1+ADC2 as a dual pair (ADC12_COMMON) with the same
 *   "injected simultaneous" mode (DUAL = 00101) the F405 build uses. One
 *   software start converts ADC1's injected sequence and ADC2's in lock-step,
 *   so rank 1 on ADC1 and rank 1 on ADC2 are sampled together, then rank 2
 *   and rank 2, and every result has its own JDRx register.
 *
 *   Giga pins and their ADC channels (STM32H747 datasheet):
 *     A0 PC4 ADC12_INP4      A4 PC3 ADC12_INP13
 *     A1 PC5 ADC12_INP8      A5 PC2 ADC12_INP12
 *            (= INN4)        A6 PC0 ADC123_INP10
 *     A2 PB0 ADC12_INP9      A7 PA0 ADC1_INP16 only   -> can't be on ADC2
 *            (= INN5)        A8/A9 (PC2_C/PC3_C)  ADC3 only -> not in the pair
 *     A3 PB1 ADC12_INP5      A10/A11 (PA1_C/PA0_C) ADC12_INP1/INP0
 *   So A0-A6 can go on EITHER ADC - any two of them make a simultaneous pair.
 *   The H747 also has TRUE differential inputs, and two of them sit on Giga
 *   headers: A0(+)/A1(-) = channel 4, A3(+)/A2(-) = channel 5. With channel 4
 *   on ADC1 and channel 5 on ADC2 in dual mode, BOTH pairs are converted at
 *   the same instant, so V and I would have no lag at all (the F405 build has
 *   one conversion, ~1.9 us, between them). That's the DIFF mode below.
 *
 * ── What this sketch measures ───────────────────────────────────────────────
 *   Pins as in 120VMonitor: VP=A0 VN=A1 IP=A3 IN=A2.
 *     SIM   ADC1 = VP,IP   ADC2 = VN,IN  dual injected simultaneous (= F405 build)
 *     SEQ   ADC1 = VP,VN,IP,IN           one ADC, one after the other (contrast)
 *     DIFF  ADC1 = A0-A1 (ch4 diff)  ADC2 = A3-A2 (ch5 diff), dual simultaneous
 *   The DAC drives a test signal: DAC0 (A12) = a hardware triangle wave
 *   (TIM6-paced, no CPU), DAC1 (A13) = DC mid-rail. A block of scans is
 *   timestamped with the cycle counter, and the fundamental of every channel is
 *   sine-fitted; the phase difference between two channels fed the SAME signal
 *   is how far apart in time they were sampled. SIM should show ~0 ns inside
 *   each pair; SEQ shows one conversion per rank, which also proves the sign.
 *   Random jitter between scans keeps the DAC's 1 us staircase from locking to
 *   the scan rate and biasing the result.
 *
 * ── Wiring ──────────────────────────────────────────────────────────────────
 *   SIM / SEQ:  jumper A12 to A0, A1, A2 and A3.
 *   DIFF:       jumper A12 to A0 and A3; A13 to A1 and A2.
 *   Any external sine works too (function generator, isolation amps on
 *   mains): !DAC,OFF and, for slow signals, !SPACE,10 so a block spans a few
 *   cycles. Keep inputs within 0..3.3 V.
 *
 * ── Commands (newline terminated, case-insensitive) ─────────────────────────
 *   !CMP              SIM then SEQ, with a verdict          (runs once at boot)
 *   !RUN              one block in the current mode
 *   !MODE,SIM|SEQ|DIFF
 *   !AUTO,<ms>        repeat !CMP every <ms> (0 = off)
 *   !DAC,<hz>         triangle on A12, <hz> = DAC step rate (default 1000000)
 *   !DAC,DC,<code>    A12 = fixed 12-bit code (noise test)   !DAC,OFF
 *   !N,<scans>        block size (16..8192)
 *   !SMP,<0..7>       sample time code: 1.5 2.5 8.5 16.5 32.5 64.5 387.5 810.5 cyc
 *   !SPACE,<us>       scan spacing (0 = back to back)    !JIT,<us> random jitter
 *   !FREQ,<hz>        force the fit frequency (0 = measure it)
 *   !IRQ,0|1          interrupts during a block (default 0 = off)
 *   !RAW,<n>          dump the first n scans of the last block as CSV
 *   !STATUS  !HELP
 *   Structured lines: $SKEW,<mode>,<f_hz>,<lag1_ns>,<lag2_ns>,<lag3_ns>,<scan_us>,<timeouts>
 *   (lags are each channel vs the first, in table order).
 */

#include <Arduino.h>
#include "adc_types.h"     // structs used in function signatures (Arduino prototype order)

// ═══ pins ═════════════════════════════════════════════════════════════════════

struct AnaPin {
  const char* name;  const char* label;
  uint8_t ard;       PinName pn;
  uint8_t port, bit; uint8_t ch;      // GPIO port (0 = A), bit, ADC12 channel
};
static const AnaPin PINS[4] = {
  { "VP", "A0", A0, PC_4, 2, 4, 4 },   // ADC12_INP4  (DIFF: ch4 +)
  { "VN", "A1", A1, PC_5, 2, 5, 8 },   // ADC12_INP8  (DIFF: ch4 -, INN4)
  { "IP", "A3", A3, PB_1, 1, 1, 5 },   // ADC12_INP5  (DIFF: ch5 +)
  { "IN", "A2", A2, PB_0, 1, 0, 9 },   // ADC12_INP9  (DIFF: ch5 -, INN5)
};
enum { P_VP, P_VN, P_IP, P_IN };
#define DIFF_CH_V  4                   // ADC1, A0(+) A1(-)
#define DIFF_CH_I  5                   // ADC2, A3(+) A2(-)

enum Mode : uint8_t { M_SIM, M_SEQ, M_DIFF };
static const char* const MODE_NAME[3] = { "SIM", "SEQ", "DIFF" };

// ═══ settings ═════════════════════════════════════════════════════════════════

#define NMAX 8192
static Mode     g_mode    = M_SIM;
static uint16_t g_n       = 8192;
static uint8_t  g_smp     = 3;          // 16.5 cycles
static float    g_spaceUs = 0.0f;
static float    g_jitUs   = 1.0f;
static float    g_freq    = 0.0f;       // 0 = measure
static bool     g_irq     = false;
static uint32_t g_autoMs  = 0;
static uint32_t g_dacRate = 1000000;    // 0 = off, 1 = DC
static uint16_t g_dacDc   = 2048;

static const float SMP_CYC[8] = { 1.5f, 2.5f, 8.5f, 16.5f, 32.5f, 64.5f, 387.5f, 810.5f };

// ═══ cycle counter ════════════════════════════════════════════════════════════

static uint32_t CPU_MHZ = 480;
static inline uint32_t cyc() { return DWT->CYCCNT; }

static void cycInit() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  *(volatile uint32_t*)0xE0001FB0 = 0xC5ACCE55;   // DWT LAR: unlock (Cortex-M7)
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CPU_MHZ = SystemCoreClock / 1000000UL;
}

static uint32_t g_rng = 0x12345678;
static inline uint32_t rnd() {               // xorshift32
  g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
  return g_rng;
}

// ═══ GPIO ═════════════════════════════════════════════════════════════════════

static void gpioAnalog(uint8_t port, uint8_t bit) {
  RCC->AHB4ENR |= (1UL << port);
  (void)RCC->AHB4ENR;
  GPIO_TypeDef* g = (GPIO_TypeDef*)(GPIOA_BASE + 0x400UL * port);
  g->PUPDR &= ~(3UL << (2 * bit));
  g->MODER |=  (3UL << (2 * bit));
}

// ═══ ADC, register level ══════════════════════════════════════════════════════

// "rs" bits of ADC_CR: writing 1 starts something, so every read-modify-write
// masks them out (the same trick the ST HAL uses).
#define CR_RS (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | \
               ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN)

static inline void crSet(ADC_TypeDef* a, uint32_t clr, uint32_t set) {
  a->CR = (a->CR & ~(CR_RS | clr)) | set;
}

static uint32_t g_rev = 0;
static uint32_t g_hclk = 0;
static float    g_fadcMHz = 0;           // assumed ADC clock
static uint32_t g_boost = 0;
static uint32_t g_cr1Start = 0;          // ADC1->CR value that starts the injected group
static volatile uint32_t g_tmo = 0;
static bool     g_adcOk = false;

static bool waitBit(volatile uint32_t* reg, uint32_t mask, bool set, uint32_t ms) {
  uint32_t t = millis();
  while (((*reg & mask) != 0) != set) {
    if (millis() - t > ms) return false;
  }
  return true;
}

static bool adcPowerUp(ADC_TypeDef* a, const char* name) {
  crSet(a, ADC_CR_DEEPPWD, 0);                       // out of deep power-down
  crSet(a, 0, ADC_CR_ADVREGEN);                      // regulator on
  delayMicroseconds(50);                             // tADCVREG_STUP (10 us)
  crSet(a, ADC_CR_BOOST, g_boost);
  // single-ended offset + linearity, then the differential offset
  crSet(a, ADC_CR_ADCALDIF, ADC_CR_ADCALLIN | ADC_CR_ADCAL);
  bool ok = waitBit(&a->CR, ADC_CR_ADCAL, false, 200);
  crSet(a, ADC_CR_ADCALLIN, ADC_CR_ADCALDIF | ADC_CR_ADCAL);
  ok = waitBit(&a->CR, ADC_CR_ADCAL, false, 200) && ok;
  crSet(a, ADC_CR_ADCALDIF, 0);
  if (!ok) { Serial.print(F("!! calibration timed out on ")); Serial.println(name); }
  return ok;
}

static bool adcEnable(ADC_TypeDef* a) {
  a->ISR = ADC_ISR_ADRDY;
  crSet(a, 0, ADC_CR_ADEN);
  bool ok = waitBit(&a->ISR, ADC_ISR_ADRDY, true, 50);
  a->ISR = ADC_ISR_ADRDY;
  return ok;
}

static void adcDisable(ADC_TypeDef* a) {
  if (!(a->CR & ADC_CR_ADEN)) return;
  if (a->CR & ADC_CR_JADSTART) { crSet(a, 0, ADC_CR_JADSTP); waitBit(&a->CR, ADC_CR_JADSTP, false, 10); }
  if (a->CR & ADC_CR_ADSTART)  { crSet(a, 0, ADC_CR_ADSTP);  waitBit(&a->CR, ADC_CR_ADSTP,  false, 10); }
  crSet(a, 0, ADC_CR_ADDIS);
  waitBit(&a->CR, ADC_CR_ADEN, false, 10);
}

static void adcInitOnce() {
  g_rev  = HAL_GetREVID();
  g_hclk = HAL_RCC_GetHCLKFreq();

  for (uint8_t k = 0; k < 4; k++) gpioAnalog(PINS[k].port, PINS[k].bit);

  RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
  (void)RCC->AHB1ENR;

  // Synchronous clock, HCLK/4: both ADCs run off the bus clock, so there is no
  // async-clock resync jitter between the software start and the sample.
  // Rev V silicon divides the ADC clock by a further 2 (RM0399).
  bool revV = (g_rev >= 0x2000);
  g_fadcMHz = (float)g_hclk / 4.0f / (revV ? 2.0f : 1.0f) / 1e6f;
  if (revV) {
    g_boost = (g_fadcMHz > 25.0f) ? (ADC_CR_BOOST_0 | ADC_CR_BOOST_1)
            : (g_fadcMHz > 12.5f) ? ADC_CR_BOOST_1
            : (g_fadcMHz > 6.25f) ? ADC_CR_BOOST_0 : 0;
  } else {
    g_boost = (g_fadcMHz > 20.0f) ? ADC_CR_BOOST_0 : 0;  // rev Y: one BOOST bit
  }

  adcDisable(ADC1); adcDisable(ADC2);
  ADC12_COMMON->CCR = (3UL << ADC_CCR_CKMODE_Pos);   // HCLK/4, independent for now
  g_adcOk  = adcPowerUp(ADC1, "ADC1");
  g_adcOk &= adcPowerUp(ADC2, "ADC2");
}

static uint32_t smprAll(uint8_t code, uint8_t from) {   // 10 channels per SMPR
  uint32_t r = 0;
  for (uint8_t k = 0; k < 10; k++) if (from + k < 20) r |= (uint32_t)code << (3 * k);
  return r;
}

static uint8_t ranksPerAdc() { return g_mode == M_SEQ ? 4 : g_mode == M_SIM ? 2 : 1; }
static uint8_t nChannels()   { return g_mode == M_DIFF ? 2 : 4; }

static bool adcConfigure(Mode m) {
  g_mode = m;
  adcDisable(ADC1); adcDisable(ADC2);

  // DUAL = 00101 injected simultaneous (what the F405 build calls MULTI = 5)
  ADC12_COMMON->CCR = (3UL << ADC_CCR_CKMODE_Pos) | (m == M_SEQ ? 0UL : 5UL);
  ADC1->DIFSEL = (m == M_DIFF) ? (1UL << DIFF_CH_V) : 0;   // only writable with ADEN = 0
  ADC2->DIFSEL = (m == M_DIFF) ? (1UL << DIFF_CH_I) : 0;

  ADC_TypeDef* const both[2] = { ADC1, ADC2 };
  for (ADC_TypeDef* a : both) {
    a->CFGR  = ADC_CFGR_JQDIS;          // 16 bit, no DMA, no context queue: plain JSQR
    a->CFGR2 = 0;                        // no oversampling, no shift
    a->SMPR1 = smprAll(g_smp, 0);        // every channel the same: simultaneous mode
    a->SMPR2 = smprAll(g_smp, 10);       // needs equal sample times on both ADCs
  }
  bool ok = adcEnable(ADC1);
  ok = adcEnable(ADC2) && ok;

  const uint8_t vp = PINS[P_VP].ch, vn = PINS[P_VN].ch, ip = PINS[P_IP].ch, in = PINS[P_IN].ch;
  if (m == M_SIM) {
    ADC1->PCSEL = (1UL << vp) | (1UL << ip);
    ADC2->PCSEL = (1UL << vn) | (1UL << in);
    ADC1->JSQR = 1UL | ((uint32_t)vp << ADC_JSQR_JSQ1_Pos) | ((uint32_t)ip << ADC_JSQR_JSQ2_Pos);
    ADC2->JSQR = 1UL | ((uint32_t)vn << ADC_JSQR_JSQ1_Pos) | ((uint32_t)in << ADC_JSQR_JSQ2_Pos);
  } else if (m == M_SEQ) {
    ADC1->PCSEL = (1UL << vp) | (1UL << vn) | (1UL << ip) | (1UL << in);
    ADC2->PCSEL = 0;
    ADC1->JSQR = 3UL | ((uint32_t)vp << ADC_JSQR_JSQ1_Pos) | ((uint32_t)vn << ADC_JSQR_JSQ2_Pos)
                     | ((uint32_t)ip << ADC_JSQR_JSQ3_Pos) | ((uint32_t)in << ADC_JSQR_JSQ4_Pos);
    ADC2->JSQR = 0;
  } else {
    ADC1->PCSEL = 1UL << DIFF_CH_V;
    ADC2->PCSEL = 1UL << DIFF_CH_I;
    ADC1->JSQR = (uint32_t)DIFF_CH_V << ADC_JSQR_JSQ1_Pos;
    ADC2->JSQR = (uint32_t)DIFF_CH_I << ADC_JSQR_JSQ1_Pos;
  }
  g_cr1Start = (ADC1->CR & ~CR_RS) | ADC_CR_JADSTART;
  if (!ok) Serial.println(F("!! ADC enable timed out"));
  return ok;
}

static inline bool waitJeos(ADC_TypeDef* a) {
  uint32_t k = 0;
  while (!(a->ISR & ADC_ISR_JEOS)) { if (++k > 20000) { g_tmo++; return false; } }
  return true;
}

// One software start. In SIM/DIFF the start on ADC1 (master) also starts ADC2.
static inline void adcScan(uint16_t* v) {
  ADC1->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
  ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
  ADC1->CR = g_cr1Start;
  waitJeos(ADC1);
  if (g_mode == M_SIM) {
    waitJeos(ADC2);
    v[P_VP] = ADC1->JDR1; v[P_IP] = ADC1->JDR2;
    v[P_VN] = ADC2->JDR1; v[P_IN] = ADC2->JDR2;
  } else if (g_mode == M_SEQ) {
    v[P_VP] = ADC1->JDR1; v[P_VN] = ADC1->JDR2;
    v[P_IP] = ADC1->JDR3; v[P_IN] = ADC1->JDR4;
  } else {
    waitJeos(ADC2);
    v[0] = ADC1->JDR1; v[1] = ADC2->JDR1; v[2] = 0; v[3] = 0;
  }
}

// ═══ DAC test source ══════════════════════════════════════════════════════════

#define DAC_BASE_CODE 1024        // triangle 1024..3071 = ~0.83..2.47 V
#define DAC_MAMP      10          // amplitude 2^11 - 1 = 2047

static uint32_t timClk1() {       // APB1 timer clock
  uint32_t p = HAL_RCC_GetPCLK1Freq();
  return (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) ? 2 * p : p;
}

static float dacTriFreq() {
  return g_dacRate > 1 ? (float)g_dacRate / (2.0f * ((2UL << DAC_MAMP) - 1)) : 0.0f;
}

static void dacApply() {
  RCC->APB1LENR |= RCC_APB1LENR_DAC12EN | RCC_APB1LENR_TIM6EN;
  (void)RCC->APB1LENR;
  gpioAnalog(0, 4);                                  // PA4 = A12 = DAC0
  gpioAnalog(0, 5);                                  // PA5 = A13 = DAC1
  DAC1->CR = 0;
  TIM6->CR1 = 0;
  if (g_dacRate == 0) return;

  DAC1->MCR = 0;                                     // both: buffered, to the pin
  if (g_dacRate == 1) {                              // DC
    DAC1->CR = DAC_CR_EN1 | DAC_CR_EN2;
  } else {
    uint32_t div = timClk1() / g_dacRate;
    if (div < 2) div = 2;
    uint32_t psc = (div - 1) / 65536;
    uint32_t arr = div / (psc + 1) - 1;
    TIM6->PSC = psc;
    TIM6->ARR = arr;
    TIM6->CR2 = TIM_CR2_MMS_1;                       // MMS = 010: update -> TRGO
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 = TIM_CR1_CEN;
    g_dacRate = timClk1() / ((psc + 1) * (arr + 1)); // what it actually is
    DAC1->CR = (5UL << DAC_CR_TSEL1_Pos) | DAC_CR_TEN1          // TIM6 TRGO
             | (2UL << DAC_CR_WAVE1_Pos) | ((uint32_t)DAC_MAMP << DAC_CR_MAMP1_Pos)
             | DAC_CR_EN2;
    DAC1->CR |= DAC_CR_EN1;
  }
  DAC1->DHR12R1 = (g_dacRate == 1) ? g_dacDc : DAC_BASE_CODE;
  DAC1->DHR12R2 = 2048;                              // A13: mid-rail reference
}

// ═══ capture ══════════════════════════════════════════════════════════════════

struct Scan { uint32_t t; uint16_t v[4]; };
static Scan     g_buf[NMAX];
static uint16_t g_have = 0;
static uint32_t g_blockCyc = 0;
static Mode     g_bufMode = M_SIM;

static float measureScanUs() {
  uint16_t v[4];
  for (uint8_t k = 0; k < 8; k++) adcScan(v);
  noInterrupts();
  uint32_t t = cyc();
  for (uint16_t k = 0; k < 256; k++) adcScan(v);
  t = cyc() - t;
  interrupts();
  return (float)t / (float)CPU_MHZ / 256.0f;
}

static void capture() {
  const uint32_t spc = (uint32_t)(g_spaceUs * CPU_MHZ);
  const uint32_t jit = (uint32_t)(g_jitUs * CPU_MHZ);
  g_tmo = 0;
  uint16_t warm[4];
  adcScan(warm);
  if (!g_irq) noInterrupts();
  const uint32_t t0 = cyc();
  uint32_t next = t0;
  for (uint16_t i = 0; i < g_n; i++) {
    const uint32_t target = next + (jit ? rnd() % jit : 0);
    while ((int32_t)(cyc() - target) < 0) {}
    const uint32_t ts = cyc();
    adcScan(g_buf[i].v);
    g_buf[i].t = ts - t0;
    next = spc ? next + spc : cyc();
  }
  g_blockCyc = cyc() - t0;
  interrupts();
  g_have = g_n;
  g_bufMode = g_mode;
}

// ═══ analysis ═════════════════════════════════════════════════════════════════

static inline double sampleVal(uint16_t i, uint8_t c) {
  double x = g_buf[i].v[c];
  return (g_bufMode == M_DIFF) ? x - 32768.0 : x;
}
static inline double sampleT(uint16_t i) { return (double)g_buf[i].t / ((double)CPU_MHZ * 1e6); }

static bool solve3(double A[3][3], double b[3], double x[3]) {
  double M[3][4];
  for (int r = 0; r < 3; r++) { for (int c = 0; c < 3; c++) M[r][c] = A[r][c]; M[r][3] = b[r]; }
  for (int c = 0; c < 3; c++) {
    int p = c;
    for (int r = c + 1; r < 3; r++) if (fabs(M[r][c]) > fabs(M[p][c])) p = r;
    if (fabs(M[p][c]) < 1e-30) return false;
    if (p != c) for (int k = 0; k < 4; k++) { double t = M[c][k]; M[c][k] = M[p][k]; M[p][k] = t; }
    for (int r = 0; r < 3; r++) {
      if (r == c) continue;
      double f = M[r][c] / M[c][c];
      for (int k = c; k < 4; k++) M[r][k] -= f * M[c][k];
    }
  }
  for (int r = 0; r < 3; r++) x[r] = M[r][3] / M[r][r];
  return true;
}

static void printPad(const String& s, uint8_t w) {
  Serial.print(s);
  for (uint8_t k = s.length(); k < w; k++) Serial.print(' ');
}

static const char* chName(uint8_t c) {
  if (g_bufMode == M_DIFF) return c == 0 ? "V" : "I";
  return PINS[c].name;
}
static String chWhere(uint8_t c) {
  if (g_bufMode == M_DIFF) return c == 0 ? F("A0-A1 ADC1") : F("A3-A2 ADC2");
  String s = PINS[c].label;
  s += (g_bufMode == M_SEQ) ? F(" ADC1") : ((c == P_VP || c == P_IP) ? F(" ADC1") : F(" ADC2"));
  return s;
}

static BlockResult analyze(float scanUs) {
  BlockResult R = {};
  R.nch = (g_bufMode == M_DIFF) ? 2 : 4;
  R.scanUs = scanUs;
  R.tmo = g_tmo;
  const uint16_t n = g_have;
  ChanFit CF[4] = {};

  // basic stats
  for (uint8_t c = 0; c < R.nch; c++) {
    double s = 0, s2 = 0; int32_t mn = 1 << 30, mx = -(1 << 30);
    for (uint16_t i = 0; i < n; i++) {
      double x = sampleVal(i, c);
      s += x; s2 += x * x;
      if (x < mn) mn = (int32_t)x;
      if (x > mx) mx = (int32_t)x;
    }
    CF[c].mean = s / n;
    CF[c].sd = sqrt(fmax(0.0, s2 / n - CF[c].mean * CF[c].mean));
    CF[c].mn = mn; CF[c].mx = mx;
  }

  // frequency from channel 0's rising crossings of its mean, with hysteresis
  const double pp0 = CF[0].mx - CF[0].mn;
  static double tc[256];
  uint16_t nc = 0;
  if (pp0 >= 200) {
    const double lo = CF[0].mean - 0.15 * pp0;
    bool armed = false;
    for (uint16_t i = 1; i < n && nc < 256; i++) {
      double x = sampleVal(i, 0);
      if (x < lo) armed = true;
      else if (armed && x >= CF[0].mean) {
        double xp = sampleVal(i - 1, 0);
        double tp = sampleT(i - 1), tn = sampleT(i);
        double fr = (xp < CF[0].mean && x != xp) ? (CF[0].mean - xp) / (x - xp) : 1.0;
        tc[nc++] = tp + fr * (tn - tp);
        armed = false;
      }
    }
  }

  double f = g_freq;
  uint16_t i0 = 0, i1 = n;
  if (f <= 0 && nc >= 2) {
    f = (nc - 1) / (tc[nc - 1] - tc[0]);
    while (i0 < n && sampleT(i0) < tc[0]) i0++;              // whole periods only
    while (i1 > i0 && sampleT(i1 - 1) > tc[nc - 1]) i1--;
  }

  Serial.print(F("block: ")); Serial.print(n); Serial.print(F(" scans in "));
  Serial.print((double)g_blockCyc / CPU_MHZ / 1000.0, 2); Serial.print(F(" ms, scan "));
  Serial.print(scanUs, 3); Serial.print(F(" us = ")); Serial.print(ranksPerAdc());
  Serial.print(F(" rank(s) x ")); Serial.print(scanUs / ranksPerAdc(), 3);
  Serial.print(F(" us; expected conversion ")); Serial.print((SMP_CYC[g_smp] + 8.5f) / g_fadcMHz, 3);
  Serial.print(F(" us at ")); Serial.print(g_fadcMHz, 1); Serial.println(F(" MHz"));
  if (R.tmo) {
    Serial.print(F("!! ")); Serial.print(R.tmo);
    Serial.println(F(" conversion timeouts - in SIM/DIFF that means ADC2 never started with ADC1: dual mode is NOT working"));
  }

  const bool doFit = f > 0 && (i1 - i0) >= 64;
  double w = 2.0 * PI * f;
  if (doFit) {
    Serial.print(F("signal: ")); Serial.print(f, 3); Serial.print(F(" Hz"));
    if (g_freq <= 0) { Serial.print(F(", ")); Serial.print(nc - 1); Serial.print(F(" whole period(s) fitted")); }
    Serial.println();
    for (uint8_t c = 0; c < R.nch; c++) {
      double A[3][3] = {}, b[3] = {}, x[3];
      for (uint16_t i = i0; i < i1; i++) {
        double t = sampleT(i), cs = cos(w * t), sn = sin(w * t), v = sampleVal(i, c);
        double r[3] = { cs, sn, 1.0 };
        for (int p = 0; p < 3; p++) { b[p] += r[p] * v; for (int q = 0; q < 3; q++) A[p][q] += r[p] * r[q]; }
      }
      if (!solve3(A, b, x)) continue;
      CF[c].amp = hypot(x[0], x[1]);
      CF[c].phase = atan2(-x[1], x[0]);
      double e2 = 0;
      for (uint16_t i = i0; i < i1; i++) {
        double t = sampleT(i);
        double e = sampleVal(i, c) - (x[0] * cos(w * t) + x[1] * sin(w * t) + x[2]);
        e2 += e * e;
      }
      CF[c].resid = sqrt(e2 / (i1 - i0));
      CF[c].sig = true;
    }
    R.ok = CF[0].sig;
  } else if (pp0 >= 200) {
    Serial.println(F("signal: fewer than 2 rising crossings on the first channel - longer block (!N / !SPACE) or !FREQ"));
  } else {
    Serial.println(F("signal: none on the first channel (p-p < 200 counts) - noise statistics only"));
  }

  // table
  Serial.println(F("ch  where        mean      p-p      sd       fund.amp  resid    sampled after ch0"));
  for (uint8_t c = 0; c < R.nch; c++) {
    printPad(chName(c), 4);
    printPad(chWhere(c), 13);
    printPad(String(CF[c].mean, 1), 10);
    printPad(String(CF[c].mx - CF[c].mn), 9);
    printPad(String(CF[c].sd, 1), 9);
    if (CF[c].sig) {
      printPad(String(CF[c].amp, 1), 10);
      printPad(String(CF[c].resid, 1), 9);
      if (c == 0) Serial.print(F("(reference)"));
      else if (CF[c].amp < 0.1 * CF[0].amp) Serial.print(F("(no signal on this pin)"));
      else {
        double d = CF[c].phase - CF[0].phase;
        while (d > PI) d -= 2 * PI;
        while (d < -PI) d += 2 * PI;
        R.lagNs[c] = d / w * 1e9;
        Serial.print(R.lagNs[c], 1); Serial.print(F(" ns"));
      }
    }
    Serial.println();
  }

  // A pair only counts if both pins carry the same signal. A floating pin
  // still shows a big "signal": the ghost of the channel converted before it,
  // left on the sampling capacitor. Matching fitted amplitudes (3 %) tell a
  // jumpered pin from a floating one.
  auto same = [&](uint8_t a, uint8_t b) {
    return CF[a].sig && CF[b].sig && CF[a].amp > 0.1 * CF[0].amp && CF[b].amp > 0.1 * CF[0].amp &&
           fabs(CF[a].amp - CF[b].amp) < 0.03 * fmax(CF[a].amp, CF[b].amp);
  };

  if (g_bufMode != M_DIFF) {
    R.pairOk[0] = same(P_VP, P_VN);
    R.pairOk[1] = same(P_IP, P_IN);
    R.viOk      = same(P_VP, P_IP);
    for (uint8_t p = 0; p < 2; p++) {
      const uint8_t a = p ? P_IP : P_VP, b2 = p ? P_IN : P_VN;
      double s = 0, s2 = 0;
      for (uint16_t i = 0; i < n; i++) { double d = sampleVal(i, a) - sampleVal(i, b2); s += d; s2 += d * d; }
      double m = s / n;
      R.pairRms[p] = sqrt(fmax(0.0, s2 / n - m * m));
    }
    Serial.print(F("pair difference rms (what a +/- pair reading sees): VP-VN "));
    Serial.print(R.pairRms[0], 1); Serial.print(F("  IP-IN ")); Serial.print(R.pairRms[1], 1);
    Serial.println(F(" counts"));
    if (R.ok) {
      Serial.print(F("pair skew VP->VN: "));
      if (R.pairOk[0]) { Serial.print(R.lagNs[P_VN], 1); Serial.println(F(" ns")); }
      else Serial.println(F("skipped (A0 and A1 don't carry the same signal)"));
      Serial.print(F("pair skew IP->IN: "));
      if (R.pairOk[1]) { Serial.print(R.lagNs[P_IN] - R.lagNs[P_IP], 1); Serial.println(F(" ns")); }
      else Serial.println(F("skipped (A3 and A2 don't carry the same signal - floating?)"));
      Serial.print(F("V->I lag:         "));
      if (R.viOk) {
        Serial.print(R.lagNs[P_IP], 1); Serial.print(F(" ns = "));
        Serial.print(R.lagNs[P_IP] * 1e-9 * 360.0 * 60.0, 4); Serial.println(F(" deg at 60 Hz"));
      } else Serial.println(F("skipped (A0 and A3 don't carry the same signal)"));
    }
  } else if (R.ok) {
    Serial.print(F("V->I lag ")); Serial.print(R.lagNs[1], 1); Serial.println(F(" ns"));
  } else {
    Serial.println(F("DIFF: wire A12 -> A0 and A3, A13 -> A1 and A2 to see a signal on both pairs"));
  }

  R.f = f;
  Serial.print(F("$SKEW,")); Serial.print(MODE_NAME[g_bufMode]); Serial.print(',');
  Serial.print(R.ok ? f : 0.0, 3);
  for (uint8_t c = 1; c < 4; c++) { Serial.print(','); Serial.print(c < R.nch ? R.lagNs[c] : 0.0, 1); }
  Serial.print(','); Serial.print(scanUs, 3); Serial.print(','); Serial.println(R.tmo);
  return R;
}

static BlockResult runBlock(Mode m) {
  adcConfigure(m);
  Serial.println();
  Serial.print(F("=== ")); Serial.print(MODE_NAME[m]); Serial.print(F(": "));
  if (m == M_SIM)      Serial.println(F("ADC1 = VP,IP  ADC2 = VN,IN  (dual injected simultaneous)"));
  else if (m == M_SEQ) Serial.println(F("ADC1 = VP,VN,IP,IN  (one ADC, sequential)"));
  else                 Serial.println(F("ADC1 = A0-A1 diff  ADC2 = A3-A2 diff  (dual injected simultaneous)"));
  float us = measureScanUs();
  capture();
  return analyze(us);
}

static void runCompare() {
  const Mode keep = g_mode;
  BlockResult s = runBlock(M_SIM);
  BlockResult q = runBlock(M_SEQ);
  adcConfigure(keep);
  Serial.println();
  Serial.println(F("=== SUMMARY ==="));
  if (!s.ok || !q.ok) {
    Serial.println(F("no usable signal on VP - check the A12 -> A0/A1/A2/A3 jumpers (or !DAC / !FREQ)"));
    return;
  }
  const bool use[2] = { s.pairOk[0] && q.pairOk[0], s.pairOk[1] && q.pairOk[1] };
  if (!use[0] && !use[1]) {
    Serial.println(F("no pair has the same signal on both pins - jumper A0+A1 and/or A3+A2 together"));
    return;
  }
  const double sk[2]  = { s.lagNs[P_VN], s.lagNs[P_IN] - s.lagNs[P_IP] };
  const double seq[2] = { q.lagNs[P_VN], q.lagNs[P_IN] - q.lagNs[P_IP] };
  const double conv = use[0] ? seq[0] : seq[1];    // one conversion, measured
  for (uint8_t p = 0; p < 2; p++) {
    Serial.print(p ? F("pair skew IP->IN:  ") : F("pair skew VP->VN:  "));
    if (!use[p]) { Serial.println(F("skipped (pins not carrying the same signal)")); continue; }
    Serial.print(F("SIM ")); Serial.print(sk[p], 1);
    Serial.print(F(" ns   SEQ ")); Serial.print(seq[p], 1); Serial.println(F(" ns"));
  }
  Serial.print(F("V->I lag:          "));
  if (s.viOk && q.viOk) {
    Serial.print(F("SIM ")); Serial.print(s.lagNs[P_IP], 1);
    Serial.print(F(" ns   SEQ ")); Serial.print(q.lagNs[P_IP], 1); Serial.println(F(" ns"));
  } else Serial.println(F("skipped (A0 and A3 not carrying the same signal)"));
  Serial.print(F("pair diff rms:     SIM ")); Serial.print(s.pairRms[0], 1); Serial.print('/');
  Serial.print(s.pairRms[1], 1); Serial.print(F("   SEQ ")); Serial.print(q.pairRms[0], 1);
  Serial.print('/'); Serial.print(q.pairRms[1], 1); Serial.println(F(" counts (VP-VN / IP-IN)"));
  const double lim = fmax(10.0, 0.05 * fabs(conv));
  double worst = 0;
  for (uint8_t p = 0; p < 2; p++) if (use[p]) worst = fmax(worst, fabs(sk[p]));
  const bool simOk = worst < lim && s.tmo == 0;
  const bool seqOk = conv > 100.0;
  Serial.print(F("VERDICT: "));
  if (simOk && seqOk) {
    Serial.print(F("SIMULTANEOUS. ADC1/ADC2 pair halves land within "));
    Serial.print(worst, 1);
    Serial.print(F(" ns of each other; the same pins on one ADC are "));
    Serial.print(conv, 0); Serial.println(F(" ns apart."));
  } else if (!seqOk) {
    Serial.println(F("INCONCLUSIVE: SEQ didn't show the expected one-conversion lag, so the method isn't resolving time here (signal too slow/noisy?)."));
  } else {
    Serial.println(F("NOT SIMULTANEOUS - SIM pair halves are as far apart as SEQ's; dual mode didn't engage."));
  }
}

// ═══ commands ═════════════════════════════════════════════════════════════════

static void printStatus() {
  Serial.println(F("--- status ---"));
  Serial.print(F("SystemCoreClock ")); Serial.print(SystemCoreClock / 1e6, 1);
  Serial.print(F(" MHz, HCLK ")); Serial.print(g_hclk / 1e6, 1);
  Serial.print(F(" MHz, silicon rev 0x")); Serial.print(g_rev, HEX);
  Serial.println(g_rev >= 0x2000 ? F(" (V)") : F(" (Y: ADC clock may be over spec)"));
  Serial.print(F("ADC clock HCLK/4")); if (g_rev >= 0x2000) Serial.print(F("/2"));
  Serial.print(F(" = ")); Serial.print(g_fadcMHz, 2); Serial.print(F(" MHz (assumed), BOOST 0x"));
  Serial.print(g_boost >> ADC_CR_BOOST_Pos, HEX); Serial.print(F(", calibration "));
  Serial.println(g_adcOk ? F("ok") : F("FAILED"));
  Serial.print(F("mode ")); Serial.print(MODE_NAME[g_mode]); Serial.print(F(", N ")); Serial.print(g_n);
  Serial.print(F(", SMP ")); Serial.print(g_smp); Serial.print(F(" (")); Serial.print(SMP_CYC[g_smp], 1);
  Serial.print(F(" cyc), space ")); Serial.print(g_spaceUs, 2); Serial.print(F(" us, jitter "));
  Serial.print(g_jitUs, 2); Serial.print(F(" us, freq ")); Serial.print(g_freq, 3);
  Serial.print(F(", irq ")); Serial.print(g_irq); Serial.print(F(", auto ")); Serial.println(g_autoMs);
  Serial.print(F("DAC A12: "));
  if (g_dacRate == 0) Serial.println(F("off"));
  else if (g_dacRate == 1) { Serial.print(F("DC code ")); Serial.println(g_dacDc); }
  else {
    Serial.print(F("triangle ")); Serial.print(DAC_BASE_CODE); Serial.print(F("..."));
    Serial.print(DAC_BASE_CODE + (2UL << DAC_MAMP) - 1); Serial.print(F(", step rate "));
    Serial.print(g_dacRate); Serial.print(F(" Hz -> ")); Serial.print(dacTriFreq(), 2); Serial.println(F(" Hz"));
  }
  Serial.println(F("A13: DC 2048 (mid-rail, the DIFF-mode reference)"));
  for (uint8_t k = 0; k < 4; k++) {
    Serial.print(PINS[k].name); Serial.print(' '); Serial.print(PINS[k].label);
    Serial.print(F(" ADC12 ch")); Serial.print(PINS[k].ch);
    Serial.println(digitalPinToPinName(PINS[k].ard) == PINS[k].pn ? F("  pin map ok") : F("  !! PIN MAP MISMATCH"));
  }
}

static void printHelp() {
  Serial.println(F("--- Giga simultaneous ADC test ---"));
  Serial.println(F("wiring SIM/SEQ: A12 -> A0,A1,A2,A3     DIFF: A12 -> A0,A3 and A13 -> A1,A2"));
  Serial.println(F("!CMP  !RUN  !MODE,SIM|SEQ|DIFF  !AUTO,ms  !DAC,hz|DC,code|OFF  !N,scans"));
  Serial.println(F("!SMP,0..7  !SPACE,us  !JIT,us  !FREQ,hz  !IRQ,0|1  !RAW,n  !STATUS  !HELP"));
}

static void dumpRaw(uint16_t n) {
  if (!g_have) { Serial.println(F("no block yet")); return; }
  if (n > g_have) n = g_have;
  Serial.print(F("$RAWHDR,")); Serial.print(MODE_NAME[g_bufMode]); Serial.print(F(",t_us"));
  for (uint8_t c = 0; c < ((g_bufMode == M_DIFF) ? 2 : 4); c++) { Serial.print(','); Serial.print(chName(c)); }
  Serial.println();
  for (uint16_t i = 0; i < n; i++) {
    Serial.print(F("$RAW,")); Serial.print((double)g_buf[i].t / CPU_MHZ, 3);
    for (uint8_t c = 0; c < ((g_bufMode == M_DIFF) ? 2 : 4); c++) { Serial.print(','); Serial.print((long)sampleVal(i, c)); }
    Serial.println();
  }
  Serial.println(F("$RAWEND"));
}

static void handle(String line) {
  line.trim();
  line.toUpperCase();
  if (!line.length()) return;
  if (line[0] == '!') line.remove(0, 1);
  String a[4];
  uint8_t na = 0;
  int from = 0;
  while (na < 4) {
    int c = line.indexOf(',', from);
    a[na++] = (c < 0) ? line.substring(from) : line.substring(from, c);
    if (c < 0) break;
    from = c + 1;
  }
  const String& cmd = a[0];
  if (cmd == "CMP") runCompare();
  else if (cmd == "RUN") runBlock(g_mode);
  else if (cmd == "MODE") {
    if (a[1] == "SIM") g_mode = M_SIM;
    else if (a[1] == "SEQ") g_mode = M_SEQ;
    else if (a[1] == "DIFF") g_mode = M_DIFF;
    else { Serial.println(F("$ERR,mode")); return; }
    adcConfigure(g_mode);
    Serial.print(F("$OK,mode,")); Serial.println(MODE_NAME[g_mode]);
  }
  else if (cmd == "AUTO") { g_autoMs = a[1].toInt(); Serial.print(F("$OK,auto,")); Serial.println(g_autoMs); }
  else if (cmd == "DAC") {
    if (a[1] == "OFF") g_dacRate = 0;
    else if (a[1] == "DC") { g_dacRate = 1; g_dacDc = constrain(a[2].toInt(), 0, 4095); }
    else { long r = a[1].toInt(); g_dacRate = (r > 1000) ? (uint32_t)r : 1000000UL; }
    dacApply();
    printStatus();
  }
  else if (cmd == "N")     { g_n = constrain(a[1].toInt(), 16, NMAX); Serial.print(F("$OK,n,")); Serial.println(g_n); }
  else if (cmd == "SMP")   { g_smp = constrain(a[1].toInt(), 0, 7); adcConfigure(g_mode); Serial.print(F("$OK,smp,")); Serial.println(g_smp); }
  else if (cmd == "SPACE") { g_spaceUs = fmax(0.0f, a[1].toFloat()); Serial.print(F("$OK,space,")); Serial.println(g_spaceUs, 2); }
  else if (cmd == "JIT")   { g_jitUs = fmax(0.0f, a[1].toFloat()); Serial.print(F("$OK,jit,")); Serial.println(g_jitUs, 2); }
  else if (cmd == "FREQ")  { g_freq = fmax(0.0f, a[1].toFloat()); Serial.print(F("$OK,freq,")); Serial.println(g_freq, 3); }
  else if (cmd == "IRQ")   { g_irq = a[1].toInt() != 0; Serial.print(F("$OK,irq,")); Serial.println(g_irq); }
  else if (cmd == "RAW")   dumpRaw(a[1].length() ? a[1].toInt() : 256);
  else if (cmd == "STATUS") printStatus();
  else if (cmd == "HELP" || cmd == "?") printHelp();
  else { Serial.print(F("$ERR,unknown,")); Serial.println(cmd); }
}

// ═══ main ═════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 5000) {}
  delay(200);
  cycInit();
  adcInitOnce();
  dacApply();
  adcConfigure(g_mode);
  printHelp();
  printStatus();
  delay(50);                       // let the DAC triangle settle into its sweep
  runCompare();
}

void loop() {
  static String line;
  static uint32_t lastAuto = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (line.length()) { handle(line); line = ""; } }
    else if (line.length() < 64) line += c;
  }
  if (g_autoMs && millis() - lastAuto >= g_autoMs) {
    lastAuto = millis();
    runCompare();
  }
}
