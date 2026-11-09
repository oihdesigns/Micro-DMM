#pragma once
#include <stdint.h>

// Kept out of the .ino so the IDE's generated prototypes can see them.

struct ChanFit {
  double mean, sd, amp, phase, resid;
  int32_t mn, mx;
  bool sig;
};

struct BlockResult {
  bool   ok;          // a frequency was found and fitted
  double f;
  double lagNs[4];    // vs channel 0
  double scanUs;
  uint32_t tmo;
  uint8_t nch;
  double pairRms[2];  // SIM/SEQ: VP-VN, IP-IN (counts)
  bool   pairOk[2];   // both halves carry the same signal (amplitudes match)
  bool   viOk;        // VP and IP carry the same signal
};
