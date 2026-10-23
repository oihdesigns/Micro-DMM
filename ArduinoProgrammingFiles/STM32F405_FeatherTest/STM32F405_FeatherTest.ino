// STM32F405_FeatherTest -- sine wave on A0 using the on-chip DAC
//
// Board: Adafruit Feather STM32F405 (STM32duino core)
//   Tools > Board            : Generic STM32F4 series
//   Tools > Board part number: Adafruit Feather STM32F405
//   Tools > USB support      : CDC (generic 'Serial' supersede U(S)ART)
//   Tools > Upload method    : STM32CubeProgrammer (DFU)
//
// A0 is PA4 = DAC1 channel 1, so this is a real analog output (not PWM).
// Output swings 0..3.3 V around 1.65 V. The DAC output is weak (~15 kohm
// with the buffer off, a few mA with it on) -- buffer it before driving a load.
//
// Serial commands (115200, newline terminated):
//   f<hz>    set frequency, e.g. f250      (0.1 .. 2000 Hz)
//   a<pct>   set amplitude, e.g. a50       (0 .. 100 % of full swing)
//   ?        print current settings

#include <math.h>

const int     SINE_PIN   = A0;
const int     TABLE_SIZE = 256;
const uint16_t DAC_MAX   = 4095;  // 12-bit DAC

uint16_t sineTable[TABLE_SIZE];

float    freqHz       = 100.0f;
float    amplitudePct = 100.0f;
uint32_t phaseInc     = 0;   // 32-bit phase accumulator step per microsecond
uint32_t phase        = 0;
uint32_t lastMicros   = 0;

void buildTable() {
  float amp = (amplitudePct / 100.0f) * (DAC_MAX / 2.0f);
  for (int i = 0; i < TABLE_SIZE; i++) {
    float s = sinf(2.0f * PI * i / TABLE_SIZE);
    sineTable[i] = (uint16_t)lroundf(DAC_MAX / 2.0f + amp * s);
  }
}

void setFrequency(float hz) {
  freqHz = constrain(hz, 0.1f, 2000.0f);
  // phase wraps at 2^32 once per period; advance per microsecond
  phaseInc = (uint32_t)(freqHz * 4294967296.0 / 1000000.0);
}

void printSettings() {
  Serial.print("Sine on A0: ");
  Serial.print(freqHz, 1);
  Serial.print(" Hz, ");
  Serial.print(amplitudePct, 0);
  Serial.println(" % amplitude");
}

void handleSerial() {
  static char buf[16];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      continue;
    }
    buf[len] = '\0';
    len = 0;
    if (buf[0] == 'f' || buf[0] == 'F') {
      setFrequency(atof(buf + 1));
    } else if (buf[0] == 'a' || buf[0] == 'A') {
      amplitudePct = constrain(atof(buf + 1), 0.0f, 100.0f);
      buildTable();
    } else if (buf[0] != '?') {
      Serial.println("Commands: f<hz>, a<pct>, ?");
      continue;
    }
    printSettings();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWriteResolution(12);

  buildTable();
  setFrequency(freqHz);
  lastMicros = micros();

  // Wait briefly for a serial monitor so the banner isn't lost
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  printSettings();
}

void loop() {
  uint32_t now = micros();
  phase += phaseInc * (now - lastMicros);
  lastMicros = now;

  analogWrite(SINE_PIN, sineTable[phase >> 24]);  // top 8 bits index the table

  handleSerial();

  // Heartbeat so you can tell the sketch is running
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
