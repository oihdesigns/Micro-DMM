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
struct RxLine { char b[96]; uint8_t n; };
