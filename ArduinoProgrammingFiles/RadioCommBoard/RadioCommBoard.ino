/*****************************************************************************************
  KidComms (2x Feather RP2040 + RFM69 + SSD1306) - with Serial Boot Diagnostics

  What this adds:
    - Serial prints for each init step (OLED, I2C, buttons, radio init, freq, power, encryption).
    - Visible “stuck point” when something fails.
    - Optional: continue running without OLED if OLED init fails (set OLED_REQUIRED).

  Open Serial Monitor at 115200 baud right after reset.

  Libraries:
    - RadioHead
    - Adafruit GFX Library
    - Adafruit SSD1306
******************************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <RH_RF69.h>
#include <RHReliableDatagram.h>

// ============================ USER CONFIG ============================

// If true, we halt forever if OLED init fails.
// If false, we keep going (radio + serial still works).
#define OLED_REQUIRED  true

// ---- Node addressing ----
#define NODE_ADDRESS  2   // set to 1 on one board, 2 on the other
#define PEER_ADDRESS  1

// ---- RFM69 settings ----
#define RF69_FREQ_MHZ 915.0
const uint8_t RF69_TX_POWER_DBM = 13;
const bool RF69_IS_HIGHPWR = true; // true for RFM69HCW "high power" modules; false otherwise

uint8_t RF69_ENCRYPT_KEY[16] = { 'k','i','d','c','o','m','m','s','-','1','2','3','4','5','6','7' };

// ---- Pins (EDIT for your wiring) ----
const uint8_t I2C_SDA_PIN = 2;
const uint8_t I2C_SCL_PIN = 3;

const uint8_t RF69_CS_PIN  = 16;
const uint8_t RF69_RST_PIN = 17;
const uint8_t RF69_IRQ_PIN = 21;   // RFM69 DIO0 -> this pin

const uint8_t BTN_UP_PIN   = 10;
const uint8_t BTN_DOWN_PIN = 11;
const uint8_t BTN_SEND_PIN = 12;

const uint32_t PEER_TIMEOUT_MS   = 3500;
const uint32_t BEACON_PERIOD_MS  = 1000;
const uint32_t UI_REFRESH_MS     = 150;
const uint16_t RETRY_TIMEOUT_MS  = 120;
const uint8_t  RETRY_COUNT       = 6;

// ============================ END CONFIG =============================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RH_RF69 rf69(RF69_CS_PIN, RF69_IRQ_PIN);
RHReliableDatagram manager(rf69, NODE_ADDRESS);

enum PacketType : uint8_t { PKT_BEACON = 1, PKT_TEXT = 2 };
struct __attribute__((packed)) Packet {
  uint8_t  type;
  uint8_t  from;
  uint8_t  to;
  uint16_t seq;
  uint8_t  msgIndex;
};

const char* MESSAGES[] = {
  "Are you free?",
  "Yes",
  "No",
  "Later",
  "Come here",
  "Where are you?",
  "Help!",
  "OK",
  "On my way",
  "Call mom/dad"
};
const uint8_t MSG_COUNT = sizeof(MESSAGES) / sizeof(MESSAGES[0]);

uint8_t  myAddr = NODE_ADDRESS;
uint8_t  peerAddr = PEER_ADDRESS;

uint16_t txSeq = 0;
uint8_t selectedMsg = 0;
uint8_t lastRxMsgIndex = 255;
uint32_t lastRxAtMs = 0;
uint32_t lastPeerHeardMs = 0;
int16_t  lastRssi = -999;

uint32_t lastBeaconMs = 0;
uint32_t lastUiMs = 0;

bool oledOk = false;

// ---------- Diagnostics helpers ----------
void diagHeader(const __FlashStringHelper* msg) {
  Serial.print(F("[BOOT] "));
  Serial.println(msg);
}

void diagKV(const __FlashStringHelper* key, const char* val) {
  Serial.print(F("[BOOT] "));
  Serial.print(key);
  Serial.print(F(": "));
  Serial.println(val);
}

void diagKV(const __FlashStringHelper* key, float val) {
  Serial.print(F("[BOOT] "));
  Serial.print(key);
  Serial.print(F(": "));
  Serial.println(val, 3);
}

void diagKV(const __FlashStringHelper* key, int val) {
  Serial.print(F("[BOOT] "));
  Serial.print(key);
  Serial.print(F(": "));
  Serial.println(val);
}

void diagKVHex(const __FlashStringHelper* key, uint32_t val) {
  Serial.print(F("[BOOT] "));
  Serial.print(key);
  Serial.print(F(": 0x"));
  Serial.println(val, HEX);
}

void haltForever(const __FlashStringHelper* why) {
  Serial.print(F("[HALT] "));
  Serial.println(why);
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(250);
  }
}

// ---------- UI ----------
void setStatusLed(bool on) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

bool peerLinked(uint32_t nowMs) {
  return (lastPeerHeardMs != 0) && ((nowMs - lastPeerHeardMs) <= PEER_TIMEOUT_MS);
}

void drawUI(uint32_t nowMs) {
  if (!oledOk) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.print(F("Me "));
  display.print(myAddr);
  display.print(F(" -> "));
  display.print(peerAddr);

  display.setCursor(72,0);
  display.print(F("LINK?: "));
  display.print(peerLinked(nowMs) ? F("Y") : F("N"));

  display.setCursor(0,10);
  display.print(F("RSSI:"));
  display.print(lastRssi);

  display.setCursor(70,10);
  display.print(F("RX:"));
  if (lastRxAtMs == 0) display.print(F("--"));
  else {
    uint32_t ageS = (nowMs - lastRxAtMs) / 1000;
    display.print(ageS);
    display.print(F("s"));
  }

  display.drawLine(0, 20, 127, 20, SSD1306_WHITE);

  display.setCursor(0,24);
  display.print(F("Send:"));
  display.setCursor(40,24);
  display.println(MESSAGES[selectedMsg]);
  //display.setCursor(40,34);
  //display.print(F("Sent:"));
  //display.print(msgIndex);

  display.drawLine(0, 44, 127, 44, SSD1306_WHITE);

  display.setCursor(0,48);
  display.print(F("Last:"));
  display.setCursor(40,48);
  if (lastRxMsgIndex == 255) display.print(F("(none yet)"));
  else display.print(MESSAGES[lastRxMsgIndex]);

  display.display();

  setStatusLed(peerLinked(nowMs));
}

// ---------- Buttons (simple debouncer) ----------
struct Button {
  uint8_t pin;
  bool lastStable;
  bool lastRead;
  uint32_t lastChangeMs;
};
Button btnUp   { BTN_UP_PIN,   true, true, 0 };
Button btnDown { BTN_DOWN_PIN, true, true, 0 };
Button btnSend { BTN_SEND_PIN, true, true, 0 };

bool readButtonPressed(Button &b, uint32_t nowMs, uint32_t debounceMs = 25) {
  bool r = digitalRead(b.pin);
  if (r != b.lastRead) {
    b.lastRead = r;
    b.lastChangeMs = nowMs;
  }
  if ((nowMs - b.lastChangeMs) >= debounceMs && b.lastStable != b.lastRead) {
    b.lastStable = b.lastRead;
    if (b.lastStable == LOW) return true;
  }
  return false;
}

// ---------- Radio ----------
bool sendPacket(PacketType type, uint8_t msgIndex) {
  Packet p;
  p.type = (uint8_t)type;
  p.from = myAddr;
  p.to = peerAddr;
  p.seq = txSeq++;
  p.msgIndex = msgIndex;

  manager.setTimeout(RETRY_TIMEOUT_MS);
  manager.setRetries(RETRY_COUNT);

  bool ok = manager.sendtoWait((uint8_t*)&p, sizeof(p), peerAddr);
  if (!ok) {
    Serial.print(F("[TX] FAIL type="));
    Serial.print((int)type);
    Serial.print(F(" idx="));
    Serial.print(msgIndex);
    Serial.print(F(" to="));
    Serial.println(peerAddr);
  } else {
    Serial.print(F("[TX] OK type="));
    Serial.print((int)type);
    Serial.print(F(" idx="));
    Serial.print(msgIndex);
    Serial.print(F(" to="));
    Serial.println(peerAddr);
  }
  return ok;
}

void handleRx(uint32_t nowMs) {
  if (!manager.available()) return;

  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from = 0;

  if (manager.recvfromAck(buf, &len, &from)) {
    lastRssi = rf69.lastRssi();

    if (len < sizeof(Packet)) return;

    Packet p;
    memcpy(&p, buf, sizeof(Packet));

    if (from != peerAddr) {
      Serial.print(F("[RX] Ignored packet from "));
      Serial.println(from);
      return;
    }
    if (p.to != myAddr) return;

    lastPeerHeardMs = nowMs;

    if (p.type == PKT_TEXT) {
      if (p.msgIndex < MSG_COUNT) {
        lastRxMsgIndex = p.msgIndex;
        lastRxAtMs = nowMs;

        Serial.print(F("[RX] TEXT idx="));
        Serial.print(p.msgIndex);
        Serial.print(F(" \""));
        Serial.print(MESSAGES[p.msgIndex]);
        Serial.print(F("\" RSSI="));
        Serial.println(lastRssi);
      }
    } else if (p.type == PKT_BEACON) {
      Serial.print(F("[RX] BEACON RSSI="));
      Serial.println(lastRssi);
    }
  }
}

void setupOLED() {
  diagHeader(F("OLED/I2C init"));
#if defined(ARDUINO_ARCH_RP2040)
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
#endif
  Wire.begin();

  // Try a quick I2C probe for 0x3C (optional hint)
  Wire.beginTransmission(0x3C);
  uint8_t err = Wire.endTransmission();
  if (err == 0) diagHeader(F("I2C probe: found device @0x3C"));
  else {
    Serial.print(F("[BOOT] I2C probe @0x3C failed (err="));
    Serial.print(err);
    Serial.println(F("). OLED may be 0x3D or wiring issue."));
  }

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) {
    diagHeader(F("OLED init FAILED"));
    if (OLED_REQUIRED) haltForever(F("OLED required but failed (check addr/wiring)"));
    else diagHeader(F("Continuing without OLED"));
  } else {
    diagHeader(F("OLED init OK"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("KidComms"));
    display.println(F("Booting..."));
    display.display();
  }
}

void setupButtons() {
  diagHeader(F("Buttons init"));
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_SEND_PIN, INPUT_PULLUP);
  diagKV(F("BTN_UP_PIN"), (int)BTN_UP_PIN);
  diagKV(F("BTN_DOWN_PIN"), (int)BTN_DOWN_PIN);
  diagKV(F("BTN_SEND_PIN"), (int)BTN_SEND_PIN);
}

void setupRadio() {
  diagHeader(F("Radio reset"));
  pinMode(RF69_RST_PIN, OUTPUT);
  digitalWrite(RF69_RST_PIN, LOW);
  delay(10);
  digitalWrite(RF69_RST_PIN, HIGH);
  delay(10);
  digitalWrite(RF69_RST_PIN, LOW);
  delay(10);

  diagHeader(F("RadioHead manager.init()"));
  if (!manager.init()) {
    diagHeader(F("RFM69 init FAILED"));
    Serial.println(F("[BOOT] Check: CS/RST/IRQ wiring, DIO0->IRQ pin, SPI pins, power"));
    haltForever(F("Radio init failed"));
  }
  diagHeader(F("RFM69 init OK"));

  diagHeader(F("Setting frequency"));
  rf69.setFrequency(RF69_FREQ_MHZ);
  diagKV(F("RF69_FREQ_MHZ"), (float)RF69_FREQ_MHZ);

  diagHeader(F("Setting TX power"));
  rf69.setTxPower(RF69_TX_POWER_DBM, RF69_IS_HIGHPWR);
  diagKV(F("TX_POWER_DBM"), (int)RF69_TX_POWER_DBM);
  diagKV(F("HIGH_POWER_MODE"), RF69_IS_HIGHPWR ? "true" : "false");

  diagHeader(F("Setting encryption key"));
  rf69.setEncryptionKey(RF69_ENCRYPT_KEY);
  diagHeader(F("Encryption set"));

  // Manager settings
  manager.setTimeout(RETRY_TIMEOUT_MS);
  manager.setRetries(RETRY_COUNT);
  diagKV(F("RETRY_TIMEOUT_MS"), (int)RETRY_TIMEOUT_MS);
  diagKV(F("RETRY_COUNT"), (int)RETRY_COUNT);

  diagKV(F("RF69_CS_PIN"), (int)RF69_CS_PIN);
  diagKV(F("RF69_RST_PIN"), (int)RF69_RST_PIN);
  diagKV(F("RF69_IRQ_PIN"), (int)RF69_IRQ_PIN);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  // Wait a moment for USB-Serial to enumerate, but don't hang forever
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { delay(10); }

  Serial.println();
  Serial.println(F("======================================"));
  Serial.println(F("KidComms Boot Diagnostics"));
  Serial.println(F("======================================"));

  diagKV(F("NODE_ADDRESS"), (int)myAddr);
  diagKV(F("PEER_ADDRESS"), (int)peerAddr);

  setupOLED();
  setupButtons();
  setupRadio();

  if (oledOk) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("KidComms Ready"));
    display.print(F("Node ")); display.println(myAddr);
    display.print(F("Peer ")); display.println(peerAddr);
    display.display();
  }

  diagHeader(F("Boot complete"));
  lastUiMs = 0;
  lastBeaconMs = 0;
}

void loop() {
  const uint32_t nowMs = millis();

  handleRx(nowMs);

  // Beacon periodically so LINK status comes up even without sending texts
  if (nowMs - lastBeaconMs >= BEACON_PERIOD_MS) {
    lastBeaconMs = nowMs;
    sendPacket(PKT_BEACON, 0);
  }

  // Buttons
  if (readButtonPressed(btnUp, nowMs)) {
    selectedMsg = (selectedMsg == 0) ? (MSG_COUNT - 1) : (selectedMsg - 1);
    Serial.print(F("[UI] Select idx="));
    Serial.print(selectedMsg);
    Serial.print(F(" \""));
    Serial.print(MESSAGES[selectedMsg]);
    Serial.println(F("\""));
  }
  if (readButtonPressed(btnDown, nowMs)) {
    selectedMsg = (selectedMsg + 1) % MSG_COUNT;
    Serial.print(F("[UI] Select idx="));
    Serial.print(selectedMsg);
    Serial.print(F(" \""));
    Serial.print(MESSAGES[selectedMsg]);
    Serial.println(F("\""));
  }
  if (readButtonPressed(btnSend, nowMs)) {
    Serial.print(F("[UI] SEND idx="));
    Serial.print(selectedMsg);
    Serial.print(F(" \""));
    Serial.print(MESSAGES[selectedMsg]);
    Serial.println(F("\""));
    sendPacket(PKT_TEXT, selectedMsg);
  }

  // UI refresh
  if (nowMs - lastUiMs >= UI_REFRESH_MS) {
    lastUiMs = nowMs;
    drawUI(nowMs);
  }
}
