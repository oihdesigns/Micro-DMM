/*!
 * Types shared across Giga_AC_Power_WiFi.ino.
 *
 * They live in a header because the Arduino build hoists a prototype for every
 * function in the .ino above anything the sketch declares itself, so a
 * prototype that mentions one of these types only compiles if the type arrives
 * through an #include.
 */
#pragma once
#include <Arduino.h>
#include <Arduino_AdvancedAnalog.h>

/*! One matched pair of DMA buffers, ADC1 and ADC2, from the same triggers. */
struct ScanBuf {
  DMABuffer<Sample>* a;   /*!< ADC1: rank 0 = V+, rank 1 = I+ */
  DMABuffer<Sample>* b;   /*!< ADC2: rank 0 = V-, rank 1 = I- */
  int  ns;                /*!< scans in the pair */
  bool gap;               /*!< data before this pair was lost */
  uint32_t ts;            /*!< micros() when the pair finished filling (its last scan) */
};

/*! A current-range switch point: samples from idx on were taken on range r. */
struct IrHist {
  uint64_t idx;
  uint8_t  r;
};

/*!
 * Weighted integrals over one span of the decimated sample stream (a mains
 * cycle, or a window of them), in real units times samples. Dividing by L
 * turns any of them into a mean over the span.
 */
struct SpanSums {
  double L;                    /*!< span length in samples (fractional) */
  double sv, si;               /*!< integral of v, of i */
  double svv, sii, svi;        /*!< of v^2, i^2, and v(t_i) * i */
  float  vmax, vmin, imax, imin;
  uint8_t flags;               /*!< FLG_* bits seen in the span */
};

/*! The reduced result of a span, in real-world units. */
struct PowerResult {
  float vrms, irms;   /*!< RMS, DC removed when DCREM is on */
  float p, s, q, pf;  /*!< W, VA, var (reactive + distortion), P/S */
  float hz;           /*!< from the voltage crossings; 0 when there were none */
  float vpk, ipk;     /*!< peak deviation from the span mean */
  float vdc, idc;     /*!< span means */
};

/*! A command line being assembled from one input (USB serial or TCP). */
struct RxLine { char b[160]; uint8_t n; };

/*!
 * Every setting, as the 120VMonitor protocol names them (!CFG / !SET / !SAVE).
 * Saved to the last internal flash sector with a CRC over offsetof(crc).
 * Current range 0 is HIGH (IGAIN/IOFF), range 1 is LOW (IGAINLO/IOFFLO).
 */
struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  float    vGain, vOff;          /*!< volts per ADC volt; ADC volts at 0 V */
  float    iGain, iOff;          /*!< HIGH range: amps per ADC volt; ADC volts at 0 A */
  float    iGainLo, iOffLo;      /*!< LOW range */
  float    phaseUs;              /*!< current front end's extra delay vs voltage, us */
  float    adcVref;
  float    tripA, tripPk;
  float    rngUpA, rngDnA;
  uint16_t rngHoldMs;
  uint8_t  rngMode, rngSetMs, rngCyc;
  uint8_t  cycles, harmN, dcRem;
  float    mainsHz, vMin;
  uint32_t rate;
  uint16_t decim;
  uint8_t  bits, smp, vSe, iSe;
  uint8_t  relayPin, relayInv, rngPin, rngInv;
  uint32_t crc;
};

enum : uint8_t { FT_FLOAT, FT_U8, FT_U16, FT_U32 };

/*! One row of the config table: key, type, where it lives, limits, meaning. */
struct CfgKey {
  const char* key;
  uint8_t     type;
  uint16_t    off;
  float       lo, hi;
  const char* help;
  uint8_t     group;   /*!< index into CFG_GROUP_NAMES */
  uint8_t     apply;   /*!< AP_* bits: what has to happen after a change */
};
