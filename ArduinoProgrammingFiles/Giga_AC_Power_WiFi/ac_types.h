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
struct RxLine { char b[240]; uint8_t n; };

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
  // v2: the OLED and the IR remote (120VMonitor's UI keys)
  uint8_t  oled, oledAddr, oledRot, irPin;
  uint16_t irAddr, irArmMs;
  uint8_t  irOk, irConfirm;
  uint8_t  adcRaw;                /*!< 1 = no dead-band repair (was spare: 0 in older blocks = repair on) */
  uint8_t  spare;
  uint32_t crc;
};

/*! The v1 layout (fw 2.0): Config up to rngInv, then the CRC. Read so a saved
 *  v1 block carries over into v2 with the new keys at their defaults. */
struct ConfigV1 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  float    vGain, vOff, iGain, iOff, iGainLo, iOffLo, phaseUs, adcVref;
  float    tripA, tripPk, rngUpA, rngDnA;
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

/*! One isolation amp's leg pair, for rebuilding a leg that reads the ADCs'
 *  dead-band code from its partner (see mirrorScan in the .ino). */
struct Mirror {
  float    a = 0.0f, b = -1.0f, ib = -1.0f;    // p = a + b n, ib = 1 / b
  bool     seeded = false, learned = false;
  double   sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;   // forgetting regression of p on n
  int32_t  bp = 0, bn = 0;                     // the block being summed
  uint16_t bk = 0, holdP = 0, holdN = 0;
  uint32_t cnt = 0;                            // scans rebuilt ($STATUS)
};

/*! One Wi-Fi network from arduino_secrets.h. */
struct WifiNet { const char* ssid; const char* pass; };

/*! A Wi-Fi network saved at run time (!NETADD or the web page), tried after
 *  the ones built in from arduino_secrets.h. */
struct NetEntry { char ssid[33]; char pass[64]; uint8_t pad[3]; };
#define NET_SAVED_MAX 5
/*! The saved networks and the hotspot password: their own block with its own
 *  CRC in the config sector (NET_FLASH_OFF in the .ino). */
struct NetStore {
  uint32_t magic;
  uint16_t version, size;
  char     apPass[64];
  uint8_t  n, pad[3];
  NetEntry e[NET_SAVED_MAX];
  uint32_t crc;
};
/*! One network seen by a scan. */
struct ScanAp { char ssid[33]; int8_t rssi; uint8_t enc; };

enum : uint8_t { FT_FLOAT, FT_U8, FT_U16, FT_U32 };

/*! One row of the config table: key, type, where it lives, limits, meaning. */
struct CfgKey {
  const char* key;
  uint8_t     type;
  uint16_t    off;
  float       lo, hi;
  const char* help;
  uint8_t     group;   /*!< index into CFG_GROUP_NAMES */
  uint16_t    apply;   /*!< AP_* bits: what has to happen after a change */
};
