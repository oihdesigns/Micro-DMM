// ─────────────────────────────────────────────────────────────────────────────
//  Giga_RemoteController_M4 — ADC co-processor firmware for the Giga R1
//
//  This is the Cortex-M4 half of the Giga_RemoteController_Dual pair. It owns
//  the ADC and nothing else: it configures AdvancedADC, drains the DMA queue,
//  smooths, reduces to statistics, decimates, and publishes everything through
//  the shared block in SRAM4. It never touches WiFi, USB, the display or the
//  file system — that is the M7's job, and keeping them apart is the entire
//  point of the split. The M7 can be busy pushing a 10 000-point DATA dump over
//  a TCP socket and the sample stream does not so much as hiccup.
//
//  == Flashing ================================================================
//  Tools > Flash split must be "1MB M7 + 1MB M4" for BOTH sketches. That is NOT
//  the IDE default: a board menu defaults to its first entry, which here is
//  "2MB M7 + M4 in SDRAM" - and that entry has no flash address to upload the
//  M4 image to, so the M4 would never start. The build refuses that setting
//  outright rather than letting it fail silently at runtime.
//
//  Flash THIS sketch FIRST:
//    Tools > Target core  > "M4 Co-processor"
//    Tools > Flash split  > "1MB M7 + 1MB M4"
//    Upload.
//  then Giga_RemoteController_Dual with Target core "Main Core" and the same
//  flash split. Both must agree, or bootM4() jumps to the wrong address.
//
//  The M4 stays halted until the M7 calls bootM4() in its setup() (unless the
//  BCM4 option bit is set, in which case this core boots at reset instead - the
//  handshake in giga_dual_shared.h copes with either order). Uploading only the
//  M7 sketch leaves the M4 halted; the M7 says so on its settings screen and
//  answers ERR:M4_NOT_RUNNING rather than capturing garbage.
//
//  arduino-cli, on this machine:
//    "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" \
//      --config-file "%USERPROFILE%\.arduinoIDE\arduino-cli.yaml" compile \
//      -b arduino:mbed_giga:giga:target_core=cm4,split=50_50 .
// ─────────────────────────────────────────────────────────────────────────────

#include "giga_dual_shared.h"

// The shared block lives at a fixed address in SRAM4 (D3 domain), which both
// cores address identically. SRAM4 has no clock gate on the STM32H747 and is
// untouched by this sketch pair (no RPC/OpenAMP, no PDM), so we own all of it.
static GigaShm* const shm = (GigaShm*)GIGA_SHM_BASE;

AdvancedADC adc(A0, A1, A2, A3, A4, A5, A6, A7);
static CapEngine eng;

void setup() {
    // Only engBegin's own region is initialised - deliberately NOT the whole
    // block. The M7 may already have written its boot id here (and with the
    // BCM4 option bit set this core boots at reset, so the order is not
    // guaranteed either way); wiping the host's region would strand the
    // handshake. Staleness is handled by the boot id, not by clearing memory.
    engBegin(eng, shm, &adc);
}

void loop() {
    engStep(eng);
}
