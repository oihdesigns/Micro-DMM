// GIGA_DAC_Tone_Trigger_D6.ino
// ------------------------------------------------------------
// Output a continuous sine tone on DAC2 (A13) that normally plays
// 400 Hz. When a HIGH is *detected* on digital pin D6 (rising edge),
// switch to 700 Hz for 500 ms, then automatically return to 400 Hz
// until triggered again.
//
// Uses a fixed DAC sample rate chosen high enough to render the
// highest requested tone (700 Hz) cleanly from a 64‑sample LUT.
// We generate the waveform in software using a phase accumulator so
// frequency changes do *not* require restarting the DAC DMA engine.
//
// Library: Arduino_AdvancedAnalog (Arduino GIGA R1 / Portenta, etc.)
// Device : Arduino GIGA R1
// DAC pin: A13 (dac2)
// Trigger: D6 (active HIGH)
// ------------------------------------------------------------

#include <Arduino_AdvancedAnalog.h>
#include "RPC.h"

// ---------------- Configuration ----------------
// NOTE: lut_size must match number of entries in lut[] below.
// The sample rate is set high enough for 700 Hz with 64 LUT samples/period.
// We keep the same sample rate for all output frequencies and vary the
// phase increment.

static const uint16_t TONE_BASE_HZ   = 400;   // Normal tone
static const uint16_t TONE_ALERT_HZ_HIGH  = 700;   // Tone when D6 goes HIGH
static const uint16_t TONE_ALERT_HZ_LOW  = 300;   // Tone when D6 goes HIGH

static const uint32_t ALERT_DURATION_MS = 250; // Duration of alert tone

static const uint8_t  PIN_TRIGGER = D6;        // Digital trigger input

volatile bool rxFlag = false;   // latest received value (always "the truth")
volatile bool humFlag = false;   // latest received value (always "the truth")


static const uint8_t rpcPin0 = D54;
static const uint8_t rpcPin1 = D55;

// Max “half‑amplitude” of your noise (0…2047). Lower = quieter.
#define NOISE_AMPLITUDE   2047  

// Smoothing alpha for a 1‑pole IIR low‑pass (0.0 = all raw noise, 1.0 = flat DC).
// Try values in the 0.98–0.995 range for a deep rumble.
#define NOISE_FILTER_ALPHA  0.97f

// (You can tweak this if you want a different oversampling ratio.)
// With 64‑sample LUT @ 700 Hz fundamental, the *minimum* sample rate
// that reproduces each LUT point exactly once is 700 * 64 = 44,800 sps.
// We'll run at that rate (matches user's prior example) to minimize load.
// You may raise this (e.g., 96 kS/s) for better DDS accuracy, but you must
// also update dac.begin() below.
static const uint32_t DAC_SAMPLE_RATE = 700UL * 64UL;   // 44,800 samples/sec

// AdvancedDAC(args): pin
AdvancedDAC dac1(A13);  // We'll actually output on A13; user sketch used dac1.

// 12‑bit, one full cycle, 64‑point sine LUT centered at midscale (0x0800).
// Copied verbatim from user's sketch.
uint16_t lut[] = {
    0x0800,0x08c8,0x098f,0x0a52,0x0b0f,0x0bc5,0x0c71,0x0d12,0x0da7,0x0e2e,0x0ea6,0x0f0d,0x0f63,0x0fa7,0x0fd8,0x0ff5,
    0x0fff,0x0ff5,0x0fd8,0x0fa7,0x0f63,0x0f0d,0x0ea6,0x0e2e,0x0da7,0x0d12,0x0c71,0x0bc5,0x0b0f,0x0a52,0x098f,0x08c8,
    0x0800,0x0737,0x0670,0x05ad,0x04f0,0x043a,0x038e,0x02ed,0x0258,0x01d1,0x0159,0x00f2,0x009c,0x0058,0x0027,0x000a,
    0x0000,0x000a,0x0027,0x0058,0x009c,0x00f2,0x0159,0x01d1,0x0258,0x02ed,0x038e,0x043a,0x04f0,0x05ad,0x0670,0x0737
};

static const size_t lut_size = sizeof(lut) / sizeof(lut[0]);  // should be 64



// ---------------- DDS State ----------------
// We use a 32‑bit phase accumulator.
// index = (phase * lut_size) >> 32
// phase_inc = (desiredFreq / sampleRate) * 2^32
//            = (desiredFreq * 2^32) / sampleRate
// For best accuracy we compute in 64‑bit math.

static uint32_t phase = 0;          // Current phase accumulator
static uint32_t phase_inc = 0;      // Current phase increment per DAC sample

// ---------------- Tone State Machine ----------------

enum ToneState : uint8_t {
  STATE_BASE = 0,   // 400 Hz
  STATE_ALERT       // 700 Hz for ALERT_DURATION_MS
};

static ToneState tone_state = STATE_BASE;
static uint32_t alert_start_ms = 0;  // millis() when alert started
static float noise_val = 0;  // holds last filtered noise sample

// Forward decl.
void setToneFrequency(uint32_t freq_hz);

void setup() {
  RPC.begin();
  pinMode(PIN_TRIGGER, INPUT);  // change to INPUT_PULLDOWN / _PULLUP as needed for your circuit
  //pinMode(rpcPin0, INPUT);  // change to INPUT_PULLDOWN / _PULLUP as needed for your circuit
  //pinMode(rpcPin1, INPUT);  // change to INPUT_PULLDOWN / _PULLUP as needed for your circuit

    randomSeed(analogRead(A0)); //For noise setup


  // Start DAC: 12‑bit resolution, DAC_SAMPLE_RATE, DMA buffers (nBuffers, bufSize)
  // Buffer counts chosen to balance latency and headroom; same as user's example.
  if (!dac1.begin(AN_RESOLUTION_12, DAC_SAMPLE_RATE, 4, 128)) {
    while (1) {
      // hang here if DAC init fails
    }
  }

  // Initialize DDS to base tone.
  setToneFrequency(TONE_BASE_HZ);
}

void loop() {
  // --------------- Trigger Edge Detect ---------------
  static int lastTrig = LOW;
  //static int lasthighTrig = LOW;
  //static int lastlowTrig = LOW;
  int curTrig = digitalRead(PIN_TRIGGER);
  //int lowTrig = digitalRead(rpcPin0);
  //int highTrig = digitalRead(rpcPin1);

  while (RPC.available() > 0) {
    int b = RPC.read();               // returns 0..255
    if (b >= 0) rxFlag = (b != 0);    // update the shared state
  }


  // Rising edge?
  if (curTrig == HIGH && lastTrig == LOW) {
    
    tone_state = STATE_ALERT;
    alert_start_ms = millis();
    if(rxFlag){
      setToneFrequency(TONE_ALERT_HZ_HIGH);
    }else{
      setToneFrequency(TONE_ALERT_HZ_LOW);
    }
    
  }
  lastTrig = curTrig;

/*
  // Rising edge?
  if (highTrig == HIGH && lasthighTrig == LOW) {
    tone_state = STATE_ALERT;
    alert_start_ms = millis();
    setToneFrequency(TONE_ALERT_HZ_HIGH);
  }
  lasthighTrig = highTrig;

  // Rising edge?
  if (lowTrig == HIGH && lastlowTrig == LOW) {
    tone_state = STATE_ALERT;
    alert_start_ms = millis();
    setToneFrequency(TONE_ALERT_HZ_LOW);
  }
  lastlowTrig = lowTrig;
*/  
  
  // Time out the alert state.
  if (tone_state == STATE_ALERT) {
    if ((millis() - alert_start_ms) >= ALERT_DURATION_MS) {
      tone_state = STATE_BASE;
      setToneFrequency(TONE_BASE_HZ);
    }
  }

  // --------------- DAC Buffer Service ---------------
  if (dac1.available()) {
    SampleBuffer buf = dac1.dequeue();

    for (size_t i = 0; i < buf.size(); i++) {
      if (tone_state == STATE_BASE) {
        // 1) raw random in ±NOISE_AMPLITUDE
        int32_t raw = random(-NOISE_AMPLITUDE, +NOISE_AMPLITUDE);

        // 2) 1‑pole low‑pass: keeps only the slow variations
        if(!humFlag){
          noise_val = NOISE_FILTER_ALPHA * noise_val
          + (1.0f - NOISE_FILTER_ALPHA) * raw;
        }else{
          noise_val = NOISE_FILTER_ALPHA-0.2 * noise_val
          + (1.0f - NOISE_FILTER_ALPHA-0.2) * raw;
        }


        // 3) center on mid‑scale (0x0800) and cast
        buf[i] = (uint16_t)(0x0800 + noise_val);
      }
      else {
        // your existing sine DDS path
        uint32_t idx = (uint64_t)phase * lut_size >> 32;
        buf[i] = lut[idx];
        phase += phase_inc;
      }
    
        /* THis is the steady tone output
    // Fill buffer using DDS.
    // NOTE: Because DAC_SAMPLE_RATE is constant, variable phase_inc controls freq.
    for (size_t i = 0; i < buf.size(); i++) {
      // Derive LUT index from phase (top bits).
      uint32_t idx = (uint64_t)phase * (uint64_t)lut_size >> 32; // 0..lut_size-1
      buf[i] = lut[idx];
      phase += phase_inc;
    }
    */
    
    
    
    
    }

    dac1.write(buf);

  }

  // loop() otherwise returns quickly -> low jitter.
}

// ------------------------------------------------------------
// setToneFrequency(freq_hz)
// Compute a new phase increment for the DDS.
// ------------------------------------------------------------
void setToneFrequency(uint32_t freq_hz) {
  // Guard: limit to Nyquist / table constraints if desired
  if (freq_hz > (DAC_SAMPLE_RATE / 2)) {
    freq_hz = DAC_SAMPLE_RATE / 2;  // clamp to Nyquist
  }
  // 64‑bit math to avoid overflow: ((uint64_t)freq_hz << 32) / DAC_SAMPLE_RATE
  uint64_t tmp = ((uint64_t)freq_hz << 32) / (uint64_t)DAC_SAMPLE_RATE;
  phase_inc = (uint32_t)tmp;
}
