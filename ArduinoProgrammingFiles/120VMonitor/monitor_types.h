/*!
 * Types shared across 120VMonitor.ino.
 *
 * These live in a header because the Arduino build hoists a prototype for
 * every function to the top of the .ino, above anything the sketch declares
 * itself — a prototype mentioning Config or PowerResult would not compile
 * there. Everything #included is visible to the generated prototypes.
 */

#ifndef MONITOR_TYPES_H
#define MONITOR_TYPES_H

#include <Arduino.h>
#include <stddef.h>

/*! Persistent settings. Layout changes REQUIRE bumping CFG_VERSION. */
struct Config {
  char     magic[4];     /*!< "M120" */
  uint16_t version;
  uint16_t size;         /*!< sizeof(Config) when written */

  // calibration: real = (adc_volts - off) * gain
  float vGain;           /*!< volts per ADC volt (differential) */
  float vOff;            /*!< ADC volts read with 0 V applied */
  float iGain;           /*!< amps per ADC volt (differential) */
  float iOff;            /*!< ADC volts read with 0 A flowing */
  float phaseUs;         /*!< current path delay vs voltage path, us */
  float adcVref;         /*!< ADC reference, V — calibration absorbs any error */

  // metering
  float    mainsHz;      /*!< nominal, sizes the window */
  uint16_t spc;          /*!< stored samples per mains cycle */
  uint8_t  cycles;       /*!< mains cycles per window */

  // protection
  float tripA;           /*!< open relay above this Irms (per window); 0 = off */
  float tripPk;          /*!< open relay above this |i| on any scan; 0 = off */

  // hardware
  uint8_t relayPin;
  uint8_t relayInv;      /*!< 1 = driver is active-low */
  uint8_t oled;          /*!< 1 = drive the SSD1306 */
  uint8_t oledAddr;      /*!< 0x3C or 0x3D */

  // v2: IR remote + status pixel
  uint8_t  irPin;        /*!< IR receiver pin, 0 = off; needs an edge interrupt */
  uint8_t  irConfirm;    /*!< NEC command that confirms a primed close */
  uint16_t irAddr;       /*!< NEC address (8-bit, or 16-bit for extended NEC) */
  uint8_t  irOk;         /*!< NEC command: prime close / open at once */
  uint8_t  pixel;        /*!< NeoPixel brightness, 0 = off */
  uint16_t irArmMs;      /*!< how long a prime waits for the confirm */
  float    liveVac;      /*!< pixel red at or above this Vrms */

  // v3: second current range (LOW = the 0.1 ohm shunt)
  float    iGainLo;      /*!< LOW range: amps per ADC volt */
  float    iOffLo;       /*!< LOW range: ADC volts at 0 A */
  float    rngUpA;       /*!< LOW range: any |i| above this switches to HIGH at once */
  float    rngDnA;       /*!< auto: a window peak below this drops back to LOW */
  uint16_t rngHoldMs;    /*!< auto: stay HIGH at least this long after an overload / close */
  uint8_t  rngPin;       /*!< range relay driver pin, 0 = single range */
  uint8_t  rngInv;       /*!< 0: pin HIGH selects LOW; 1: pin HIGH selects HIGH */
  uint8_t  rngMode;      /*!< 0 auto, 1 HIGH only, 2 LOW (an overload latches HIGH) */
  uint8_t  rngSetMs;     /*!< range relay settle time */

  // v4
  uint8_t  oledRot;      /*!< 0 = right side up, 1 = upside down */

  uint16_t crc;          /*!< CRC16-CCITT over [0, offsetof(Config, crc)) */
};

/*! The version-3 layout: v4 minus oledRot. */
struct ConfigV3 {
  char     magic[4];
  uint16_t version, size;
  float    vGain, vOff, iGain, iOff, phaseUs, adcVref;
  float    mainsHz;
  uint16_t spc;
  uint8_t  cycles;
  float    tripA, tripPk;
  uint8_t  relayPin, relayInv, oled, oledAddr;
  uint8_t  irPin, irConfirm;
  uint16_t irAddr;
  uint8_t  irOk, pixel;
  uint16_t irArmMs;
  float    liveVac;
  float    iGainLo, iOffLo, rngUpA, rngDnA;
  uint16_t rngHoldMs;
  uint8_t  rngPin, rngInv, rngMode, rngSetMs;
  uint16_t crc;
};

/*! The version-2 layout, read once to migrate a stored calibration. */
struct ConfigV2 {
  char     magic[4];     /*!< "M120" */
  uint16_t version;
  uint16_t size;         /*!< sizeof(Config) when written */

  // calibration: real = (adc_volts - off) * gain
  float vGain;           /*!< volts per ADC volt (differential) */
  float vOff;            /*!< ADC volts read with 0 V applied */
  float iGain;           /*!< amps per ADC volt (differential) */
  float iOff;            /*!< ADC volts read with 0 A flowing */
  float phaseUs;         /*!< current path delay vs voltage path, us */
  float adcVref;         /*!< ADC reference, V — calibration absorbs any error */

  // metering
  float    mainsHz;      /*!< nominal, sizes the window */
  uint16_t spc;          /*!< stored samples per mains cycle */
  uint8_t  cycles;       /*!< mains cycles per window */

  // protection
  float tripA;           /*!< open relay above this Irms (per window); 0 = off */
  float tripPk;          /*!< open relay above this |i| on any scan; 0 = off */

  // hardware
  uint8_t relayPin;
  uint8_t relayInv;      /*!< 1 = driver is active-low */
  uint8_t oled;          /*!< 1 = drive the SSD1306 */
  uint8_t oledAddr;      /*!< 0x3C or 0x3D */

  // v2: IR remote + status pixel
  uint8_t  irPin;        /*!< IR receiver pin, 0 = off; needs an edge interrupt */
  uint8_t  irConfirm;    /*!< NEC command that confirms a primed close */
  uint16_t irAddr;       /*!< NEC address (8-bit, or 16-bit for extended NEC) */
  uint8_t  irOk;         /*!< NEC command: prime close / open at once */
  uint8_t  pixel;        /*!< NeoPixel brightness, 0 = off */
  uint16_t irArmMs;      /*!< how long a prime waits for the confirm */
  float    liveVac;      /*!< pixel red at or above this Vrms */

  uint16_t crc;          /*!< CRC16-CCITT over [0, offsetof(Config, crc)) */
};

/*! The version-1 layout, read once to migrate a stored calibration. */
struct ConfigV1 {
  char     magic[4];
  uint16_t version, size;
  float    vGain, vOff, iGain, iOff, phaseUs, adcVref;
  float    mainsHz;
  uint16_t spc;
  uint8_t  cycles;
  float    tripA, tripPk;
  uint8_t  relayPin, relayInv, oled, oledAddr;
  uint16_t crc;
};

/*! One analysed metering window, in real-world units. */
struct PowerResult {
  float vrms, irms;      /*!< RMS, DC removed */
  float p, s, q, pf;     /*!< W, VA, var (non-real remainder), P/S */
  float hz;              /*!< from the voltage zero crossings */
  float vpk, ipk;        /*!< peak |v|, |i| in the window */
  float vdc, idc;        /*!< DC content of the window */
  float dt_s;            /*!< measured stored-sample spacing */
  float span_s;          /*!< analysed span */
  uint16_t n;            /*!< stored samples */
  uint16_t depth;        /*!< ADC scans averaged per stored sample */
  uint8_t  flags;        /*!< FLG_* */
};

/*! Evaluates m quantities at sample index k, for integrateMean(). */
typedef void (*IntegrandFn)(int32_t k, double* out);

/*! One row of the config key table. */
enum FieldType : uint8_t { FT_FLOAT, FT_U8, FT_U16 };
struct CfgField {
  const char* key;
  FieldType   type;
  uint16_t    offset;
  float       lo, hi;
  const char* help;
};

#endif  // MONITOR_TYPES_H
