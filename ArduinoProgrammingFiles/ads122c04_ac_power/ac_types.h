/*!
 * Types shared across ads122c04_ac_power.ino.
 *
 * These live in a header rather than in the sketch because the Arduino build
 * hoists a prototype for every function to the top of the .ino, above anything
 * the sketch itself declares — a prototype mentioning PowerResult or
 * IntegrandFn would not compile there. Everything #included is visible to those
 * generated prototypes, so the types have to arrive this way.
 */

#ifndef ADS122C04_AC_TYPES_H
#define ADS122C04_AC_TYPES_H

#include <Arduino.h>

/*! One analysed measurement window, in real-world units. */
struct PowerResult {
  float vrms;      /*!< RMS voltage */
  float irms;      /*!< RMS current */
  float p;         /*!< real power, W */
  float s;         /*!< apparent power, VA */
  float q;         /*!< the non-real remainder, var (reactive + distortion) */
  float pf;        /*!< power factor, P/S */
  float hz;        /*!< mains frequency from the voltage zero crossings */
  float vpk;       /*!< peak voltage in the window */
  float ipk;       /*!< peak current in the window */
  float dt_s;      /*!< measured spacing between voltage samples */
  float span_s;    /*!< length of the analysed span */
  uint16_t n;      /*!< V/I pairs acquired */
  uint8_t  flags;  /*!< FLG_* bits */
};

/*! Evaluates m quantities at sample index k, for integrateMean(). */
typedef void (*IntegrandFn)(uint16_t k, double* out);

#endif  // ADS122C04_AC_TYPES_H
