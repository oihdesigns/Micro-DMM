/*
 * RelayMountTestJig.ino  --  relay load-selector jig
 *
 * Hardware: Arduino UNO R4 WiFi (Freenove V5) + two 4-channel relay boards
 *           on the RelayMount plate.  Relay channel n is driven by pin
 *           RELAY_PIN[n-1]; channel 8 is a spare.
 *
 * What the jig does (hand schematic 2026-09-26):
 *
 *   The DUT's +/- input leads go either straight to the function generator
 *   or into a relay-selected resistor network:
 *
 *     K1  DUT+   : NC -> FGen+             NO -> K3 common
 *     K2  DUT-   : NC -> FGen-             NO -> network return bus
 *     K3         : NC -> return (SHORT)    NO -> K4 common
 *     K4         : NC -> R1 1M -> K5       NO -> K6 common
 *     K5         : NC -> return (1M)       NO -> no connection (OPEN)
 *     K6         : NC -> R2 10k -> return  NO -> K7 common
 *     K7         : NC -> R3 10.6M -> return  NO -> R4 150k -> return
 *
 *   "NC" is the contact drawn as the rest position on the schematic, so with
 *   every relay released (power-up, reset, !ALL) the DUT sees the function
 *   generator.  If a relay was wired the other way round, flip its entry in
 *   RELAY_NO_SWAPPED below rather than editing the mode table.
 *
 *   Mode    K1 K2 K3 K4 K5 K6 K7   DUT sees
 *   FGEN     0  0  0  0  0  0  0   function generator
 *   SHORT    1  1  0  0  0  0  0   0 ohm
 *   OPEN     1  1  1  0  1  0  0   open circuit
 *   1M       1  1  1  0  0  0  0   R1
 *   10K      1  1  1  1  0  0  0   R2
 *   10.6M    1  1  1  1  0  1  0   R3
 *   150K     1  1  1  1  0  1  1   R4
 *
 * Break-before-make: every mode change walks through OPEN one relay at a
 * time (SETTLE ms apart), so the DUT never momentarily sees a *different*
 * resistance or a short while the relays are moving -- only open circuit.
 *
 * Serial protocol (115200, '\n' terminated):
 *   !MODE,<FGEN|SHORT|OPEN|1M|10K|10.6M|150K>   select a load
 *   !K,<1-8>,<0|1>      drive one relay directly (mode becomes RAW)
 *   !ALL                release every relay (= FGEN, no sequencing)
 *   !SETTLE,<ms>        relay settle time between steps (default 25)
 *   !STATE              report state
 *   !ID                 identify
 *   !HELP
 * Replies:
 *   $BOOT,RelayJig,<ver>
 *   $ID,RelayJig,<ver>
 *   $STATE,<mode>,<K1..K8 bits>,<what the DUT sees>,<settle ms>
 *   $ERR,<reason>
 *   anything not starting with '$' is human-readable debug.
 */

#define FW_VERSION "1.0"

// Most cheap 4-channel relay boards (opto input, jumper JD-VCC) energise the
// relay when IN is pulled LOW.  Set to 0 for boards that are active-high.
#define RELAY_ACTIVE_LOW 1

const uint8_t NUM_RELAYS = 8;
const uint8_t RELAY_PIN[NUM_RELAYS] = {2, 3, 4, 5, 6, 7, 8, 9};  // K1..K8

// Set an entry to true if that relay's contacts are wired opposite to the
// schematic (the "NC" wire is on the NO terminal).  The coil drive is then
// inverted so the logical state -- and the mode table -- stay correct.
const bool RELAY_NO_SWAPPED[NUM_RELAYS] = {false, false, false, false,
                                           false, false, false, false};

enum Mode : uint8_t { M_FGEN, M_SHORT, M_OPEN, M_1M, M_10K, M_10M6, M_150K, M_RAW };
const char *const MODE_NAME[] = {"FGEN", "SHORT", "OPEN", "1M", "10K", "10.6M", "150K", "RAW"};
const uint8_t NUM_NAMED_MODES = 7;

// Logical relay indices (0-based) so the sequencing code reads like the schematic.
enum { K1, K2, K3, K4, K5, K6, K7, K8 };

bool relayOn[NUM_RELAYS];          // logical state: true = moved to the NO contact
Mode currentMode = M_FGEN;
uint16_t settleMs = 25;

// ---------------------------------------------------------------------------
// Relay drive

void driveRelay(uint8_t k, bool on) {
  bool coil = on ^ RELAY_NO_SWAPPED[k];
  bool level = RELAY_ACTIVE_LOW ? !coil : coil;
  digitalWrite(RELAY_PIN[k], level ? HIGH : LOW);
  relayOn[k] = on;
}

// Move one relay and wait for its contacts to settle -- only if it changes.
void step(uint8_t k, bool on) {
  if (relayOn[k] == on) return;
  driveRelay(k, on);
  delay(settleMs);
}

// K1/K2 move together so both DUT leads change over in the same instant
// (never one lead on FGen and the other on the network).
void stepLeads(bool on) {
  if (relayOn[K1] == on && relayOn[K2] == on) return;
  driveRelay(K1, on);
  driveRelay(K2, on);
  delay(settleMs);
}

// ---------------------------------------------------------------------------
// What the DUT actually sees for the present relay state.  Used for the
// $STATE report, so RAW poking is always explained in load terms.

const char *resolveLoad() {
  if (!relayOn[K1] && !relayOn[K2]) return "FGEN";
  if (relayOn[K1] != relayOn[K2]) return "SPLIT";       // one lead on FGen, one on network
  if (!relayOn[K3]) return "SHORT";
  if (!relayOn[K4]) return relayOn[K5] ? "OPEN" : "1M";
  if (!relayOn[K6]) return "10K";
  return relayOn[K7] ? "150K" : "10.6M";
}

// ---------------------------------------------------------------------------
// Sequenced mode change

// Bring the network to OPEN.  Each step is safe from any starting state:
//   K5 on  : breaks the 1M path (harmless if K4 is routing elsewhere)
//   K4 off : routes through R1 -> K5(NO) = open, dropping 10k/10.6M/150k
//   K3 on  : lifts a short onto the now-open K4 path
void networkToOpen() {
  step(K5, true);
  step(K4, false);
  step(K3, true);
}

void setMode(Mode m) {
  // Open the network first (if the DUT is on FGen this happens off-line).
  networkToOpen();

  if (m == M_FGEN) {
    stepLeads(false);
    // Network is now disconnected; release its coils.
    for (uint8_t k = K3; k <= K7; k++) step(k, false);
    currentMode = m;
    return;
  }

  // Hand the DUT leads to the (open) network.
  stepLeads(true);

  // K6/K7 are out of the path while K4 is released -- preset them.
  step(K6, m == M_10M6 || m == M_150K);
  step(K7, m == M_150K);

  // Close the one relay that makes the target path, then release K5 if it
  // is no longer in the path (saves coil current, keeps the table canonical).
  switch (m) {
    case M_SHORT: step(K3, false); step(K5, false); break;
    case M_1M:    step(K5, false); break;
    case M_10K:
    case M_10M6:
    case M_150K:  step(K4, true);  step(K5, false); break;
    case M_OPEN:
    default:      break;
  }
  currentMode = m;
}

void releaseAll() {
  for (uint8_t k = 0; k < NUM_RELAYS; k++) driveRelay(k, false);
  currentMode = M_FGEN;
}

// ---------------------------------------------------------------------------
// Reporting

void reportState() {
  Serial.print(F("$STATE,"));
  Serial.print(MODE_NAME[currentMode]);
  Serial.print(',');
  for (uint8_t k = 0; k < NUM_RELAYS; k++) Serial.print(relayOn[k] ? '1' : '0');
  Serial.print(',');
  Serial.print(resolveLoad());
  Serial.print(',');
  Serial.println(settleMs);
}

void printHelp() {
  Serial.println(F("RelayJig commands:"));
  Serial.println(F("  !MODE,<FGEN|SHORT|OPEN|1M|10K|10.6M|150K>"));
  Serial.println(F("  !K,<1-8>,<0|1>   raw relay drive"));
  Serial.println(F("  !ALL             release all relays (FGEN)"));
  Serial.println(F("  !SETTLE,<ms>     settle time per relay step"));
  Serial.println(F("  !STATE  !ID  !HELP"));
}

// ---------------------------------------------------------------------------
// Command parsing

char lineBuf[64];
uint8_t lineLen = 0;

void handleCommand(char *cmd) {
  // Upper-case in place so "!mode,10k" works.
  for (char *p = cmd; *p; p++) *p = toupper(*p);

  char *arg = strchr(cmd, ',');
  if (arg) *arg++ = '\0';

  if (!strcmp(cmd, "!MODE")) {
    if (!arg) { Serial.println(F("$ERR,MODE needs an argument")); return; }
    for (uint8_t i = 0; i < NUM_NAMED_MODES; i++) {
      if (!strcmp(arg, MODE_NAME[i])) {
        Serial.print(F("Switching to ")); Serial.println(MODE_NAME[i]);
        setMode((Mode)i);
        reportState();
        return;
      }
    }
    Serial.print(F("$ERR,unknown mode ")); Serial.println(arg);
  } else if (!strcmp(cmd, "!K")) {
    char *val = arg ? strchr(arg, ',') : nullptr;
    if (!val) { Serial.println(F("$ERR,usage !K,<1-8>,<0|1>")); return; }
    *val++ = '\0';
    int n = atoi(arg);
    if (n < 1 || n > NUM_RELAYS || (*val != '0' && *val != '1')) {
      Serial.println(F("$ERR,usage !K,<1-8>,<0|1>"));
      return;
    }
    driveRelay(n - 1, *val == '1');
    currentMode = M_RAW;
    reportState();
  } else if (!strcmp(cmd, "!ALL")) {
    releaseAll();
    reportState();
  } else if (!strcmp(cmd, "!SETTLE")) {
    long ms = arg ? atol(arg) : -1;
    if (ms < 0 || ms > 2000) { Serial.println(F("$ERR,settle 0-2000 ms")); return; }
    settleMs = ms;
    reportState();
  } else if (!strcmp(cmd, "!STATE")) {
    reportState();
  } else if (!strcmp(cmd, "!ID")) {
    Serial.println(F("$ID,RelayJig," FW_VERSION));
  } else if (!strcmp(cmd, "!HELP")) {
    printHelp();
  } else {
    Serial.print(F("$ERR,unknown command ")); Serial.println(cmd);
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen && lineBuf[0] == '!') handleCommand(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                       // overlong line: drop it
      Serial.println(F("$ERR,line too long"));
    }
  }
}

// ---------------------------------------------------------------------------

void setup() {
  // Write the released level before and after pinMode: on the RA4M1 core
  // pinMode(OUTPUT) can briefly drive LOW, which would click an active-low
  // board.  A few microseconds is far too short to pull a relay in.
  for (uint8_t k = 0; k < NUM_RELAYS; k++) {
    driveRelay(k, false);
    pinMode(RELAY_PIN[k], OUTPUT);
    driveRelay(k, false);
  }

  Serial.begin(115200);
  delay(200);
  Serial.println(F("$BOOT,RelayJig," FW_VERSION));
  reportState();
}

void loop() {
  pollSerial();
}
