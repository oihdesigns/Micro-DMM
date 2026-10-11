/*
 * Ble.ino -- the host protocol over Bluetooth LE, and the Host output sink.
 *
 * Uses the Nordic UART Service, the de-facto "serial port over BLE": the host
 * writes command bytes to RX and subscribes to notifications on TX.  The
 * protocol is byte-for-byte the one on USB (SerialCmd.ino); BLE is just a
 * second pipe.  Any client with full access to the meter, same as USB.
 *
 * Packets: ArduinoBLE's ATT MTU is 23, so each notification carries 20 bytes
 * and a line goes out in several.  The host reassembles on '\n'.  Raising the
 * MTU (ATT.setMaxMtu) would cut the packet count, but the library does not
 * fragment ACL packets, so it needs checking against the controller's buffer
 * size on hardware first.
 *
 * Flow control: writeValue() blocks inside HCI while the controller's
 * buffers are full rather than dropping, so a long dump (!CFG, !DUMP) stalls
 * the loop for a second or two instead of losing lines.
 */

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // host -> meter
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // meter -> host
#define BLE_CHUNK        20                                        // ATT MTU 23 - 3

BLEService        nusService(NUS_SERVICE_UUID);
BLECharacteristic nusRx(NUS_RX_UUID, BLEWrite | BLEWriteWithoutResponse, CMD_BUF_LEN);
BLECharacteristic nusTx(NUS_TX_UUID, BLENotify, BLE_CHUNK);

static bool bleOk = false;     // stack up and advertising
static bool bleUp = false;     // a central is connected AND subscribed to TX
static char bleName[24] = "GigaMeter";

// Outgoing bytes are collected into a line and sent when it completes, so a
// record is not split across notifications more than it has to be.
static uint8_t bleOut[128];
static size_t  bleOutLen = 0;

// Incoming bytes are queued by the write callback (which runs inside
// BLE.poll()) and parsed in bleTick(), outside the stack.  Parsing in the
// callback would run handleLine -- and its replies' notifications -- from
// inside HCI processing.
#define BLE_RX_FIFO 256
static char     bleRxFifo[BLE_RX_FIFO];
static uint16_t bleRxHead = 0, bleRxTail = 0;

// 0 = Bluetooth unavailable, 1 = advertising, 2 = host linked.
int bleState() { return !bleOk ? 0 : (bleUp ? 2 : 1); }

static void bleFlush() {
  size_t off = 0;
  while (bleUp && off < bleOutLen) {
    size_t n = bleOutLen - off;
    if (n > BLE_CHUNK) n = BLE_CHUNK;
    int sent = nusTx.writeValue(bleOut + off, (int)n);
    if (sent <= 0) { bleUp = false; break; }   // link went away mid-send
    off += (size_t)sent;
  }
  bleOutLen = 0;
}

static void onRxWritten(BLEDevice, BLECharacteristic c) {
  const uint8_t *p = c.value();
  int n = c.valueLength();
  for (int i = 0; i < n; i++) {
    uint16_t next = (bleRxHead + 1) % BLE_RX_FIFO;
    if (next == bleRxTail) break;               // full: drop the rest
    bleRxFifo[bleRxHead] = (char)p[i];
    bleRxHead = next;
  }
}

// ==================================================================
//  HOST SINK
// ==================================================================
size_t HostOut::write(uint8_t c) {
  return write(&c, 1);
}

size_t HostOut::write(const uint8_t *buf, size_t n) {
  Serial.write(buf, n);
  if (!bleUp) return n;
  for (size_t i = 0; i < n; i++) {
    bleOut[bleOutLen++] = buf[i];
    if (buf[i] == '\n' || bleOutLen == sizeof(bleOut)) bleFlush();
  }
  return n;
}

// ==================================================================
//  SETUP / SERVICE
// ==================================================================
// Named after the unit so two meters can be told apart in a scan.  Set once
// at boot: a !SEEDCAL that changes HWREV shows up in the name after a reboot.
void bleSetup() {
  if (!BLE.begin()) {
    Host.println(F("$ERR,ble,init failed -- Bluetooth disabled"));
    return;
  }
  if (cfg.hwRev) snprintf(bleName, sizeof(bleName), "GigaMeter-%u", cfg.hwRev);
  BLE.setLocalName(bleName);
  BLE.setDeviceName(bleName);
  BLE.setAdvertisedService(nusService);
  nusService.addCharacteristic(nusRx);
  nusService.addCharacteristic(nusTx);
  BLE.addService(nusService);
  nusRx.setEventHandler(BLEWritten, onRxWritten);
  BLE.advertise();
  bleOk = true;
  Host.print(F("$INFO,ble,advertising,"));
  Host.println(bleName);
}

const char *bleDeviceName() { return bleName; }

void bleTick() {
  static LineIn bleLine = {};
  if (!bleOk) return;

  BLE.poll();

  bool up = BLE.connected() && nusTx.subscribed();
  if (up != bleUp) {
    bleUp     = up;
    bleOutLen = 0;
    bleLine.len = 0;
    Host.print(F("$INFO,ble,"));
    Host.println(up ? F("linked") : F("unlinked"));
  }

  while (bleRxTail != bleRxHead) {
    char c = bleRxFifo[bleRxTail];
    bleRxTail = (bleRxTail + 1) % BLE_RX_FIFO;
    lineFeed(bleLine, c);
  }
}
