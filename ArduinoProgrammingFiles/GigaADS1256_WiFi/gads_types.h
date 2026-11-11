#pragma once
// Types shared by the sketch's functions. Kept in a header so the Arduino IDE's
// prototype generator sees them before the prototypes it inserts.
#include <Arduino.h>

struct WifiNet { const char* ssid; const char* pass; };

// one input line being assembled (USB and TCP each have one)
struct RxLine { char b[200]; uint8_t n; };

// ADS1256 DRATE register codes and their nominal rates
struct DrateInfo { uint8_t code; float sps; };

// ADC settings: written by the loop thread, applied by the ADC thread
struct AdcCfg {
  uint8_t drate;      // DRATE register code
  uint8_t pga;        // 0..6 = gain 1..64
  uint8_t buf;        // input buffer (STATUS.BUFEN)
  uint8_t nmap;       // channels defined in mux[]
  uint8_t mux[8];     // MUX register per channel: (AINP << 4) | AINN, 8 = AINCOM
  uint8_t mask;       // channels in the live scan
};

// live statistics of one channel over one report period (raw codes)
struct LiveAcc {
  int32_t  ref;       // first value of the period: sums are taken about it
  int32_t  mn, mx;
  int64_t  sum;       // sum of (v - ref)
  double   sq;        // sum of (v - ref)^2
  uint32_t n;
  uint8_t  clip;      // a value sat at full scale
};

// one capture request (filled by the loop thread, run by the ADC thread)
struct CapReq {
  uint8_t  mode;      // 0 benchmark (one channel, RDATAC), 1 multi (mux cycling)
  uint8_t  armed;     // 0 capture now, 1 wait for the trigger
  uint8_t  slope;     // 0 rising, 1 falling
  uint8_t  nch;       // channels per frame
  uint8_t  order[8];  // channel indices per frame; order[0] is the trigger channel
  uint32_t n;         // frames
  uint32_t pre;       // frames kept before the trigger frame
  uint32_t timeoutMs; // 0 = wait for the trigger forever
  int32_t  level;     // trigger level, raw code
};
