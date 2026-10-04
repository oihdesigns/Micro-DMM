/*
  BasementLightsDimmer_R4WiFi

  Arduino UNO R4 WiFi version of BasementLightsDimmer_V2. Same hardware and the same
  physical controls, plus a web page anyone on the WiFi network can open to run the
  lights.

  Hardware (unchanged from V2):
    D7  -> lights enable (relay / driver), HIGH = on
    D8  <- wall switch; ANY change of state toggles the lights (turning on goes to level 5)
    D9  <- push button to GND (internal pull-up); cycles off -> 1 -> 3 -> 5 -> off
    DS3502 digital pot @ 0x28 on I2C sets the dimmer level

    I2C: the R4 WiFi has two buses. A4/A5 (the header pins an Uno uses) are `Wire`;
    the Qwiic/STEMMA QT connector is `Wire1`. Set DIMMER_WIRE to match how the
    DS3502 is wired.

  WiFi:
    Put your network name/password in arduino_secrets.h. On boot the board prints its
    IP address on the Serial monitor (9600 baud) and scrolls it across the LED matrix.
    Browse to http://<that IP>/ from any phone or computer on the same network.
    Set a DHCP reservation in your router (or USE_STATIC_IP below) so the address
    doesn't move around.

  Web API (GET or POST; the page uses POST, curl/scripts can use either):
    /api/state            -> JSON status
    /api/on  /api/off  /api/toggle
    /api/level?n=1..5     -> preset level (also turns the lights on)
    /api/pct?v=0..100     -> fine dimmer position (also turns the lights on)

  Serial commands (9600 baud, newline terminated) -- V2's commands still work:
    TOGGLE D7 | SET LIGHTS n | ON | OFF | PCT n | STATUS | IP
*/

#include <Wire.h>
#include <WiFiS3.h>
#include "ArduinoGraphics.h"   // must come before Arduino_LED_Matrix for text scrolling
#include "Arduino_LED_Matrix.h"
#include "arduino_secrets.h"

// ---------------- configuration ----------------
#define DIMMER_WIRE      Wire     // Wire = A4/A5 header pins, Wire1 = Qwiic connector
#define DS3502_ADDRESS   0x28
#define DS3502_REG_WR    0x00     // wiper register
#define DS3502_MAX       127      // DS3502 wiper is 7 bits

#define HOSTNAME         "basement-lights"
#define USE_STATIC_IP    0        // 1 = use the addresses below instead of DHCP
IPAddress staticIP(192, 168, 1, 50);
IPAddress staticDNS(192, 168, 1, 1);
IPAddress staticGW(192, 168, 1, 1);
IPAddress staticMask(255, 255, 255, 0);

const int LightsPin  = 7;
const int SwitchPin  = 8;
const int Button     = 9;

const unsigned long DEBOUNCE_MS       = 50;
const unsigned long WIFI_RETRY_MS     = 15000;
const unsigned long HTTP_TIMEOUT_MS   = 1000;

// ---------------- state ----------------
bool    lightsOn = false;
uint8_t wiper    = 0;       // last value written to the DS3502 (0..127)
bool    dimmerOk = false;   // did the last I2C write ACK?

int           switchStable = LOW;   // debounced D8
int           switchRaw    = LOW;
unsigned long switchChangeMs = 0;
int           buttonStable = HIGH;  // debounced D9
int           buttonRaw    = HIGH;
unsigned long buttonChangeMs = 0;

unsigned long lastWifiCheckMs = 0;
bool          wasConnected = false;

WiFiServer        server(80);
ArduinoLEDMatrix  matrix;

// Wiper value for preset level 1..5. Same mapping as V2: level*1000 ohms over a
// 0-10000 range scaled to 0-255, so level 5 = 127 = full scale of the 7-bit wiper.
uint8_t levelToWiper(int level) {
  return (uint8_t)map(level * 1000L, 0, 10000, 0, 255);
}

// The preset level the dimmer is sitting on, or 0 if it was set to an in-between value.
int currentLevel() {
  for (int l = 1; l <= 5; l++) if (levelToWiper(l) == wiper) return l;
  return 0;
}

int currentPct() {
  return (wiper * 100 + DS3502_MAX / 2) / DS3502_MAX;
}

// ---------------- outputs ----------------
void writeLights(bool on) {
  lightsOn = on;
  digitalWrite(LightsPin, on ? HIGH : LOW);
  Serial.print("Lights ");
  Serial.println(on ? "ON" : "OFF");
}

void writeWiper(uint8_t value) {
  if (value > DS3502_MAX) value = DS3502_MAX;
  DIMMER_WIRE.beginTransmission(DS3502_ADDRESS);
  DIMMER_WIRE.write(DS3502_REG_WR);
  DIMMER_WIRE.write(value);
  dimmerOk = (DIMMER_WIRE.endTransmission() == 0);
  wiper = value;
  Serial.print("Dimmer wiper = ");
  Serial.print(value);
  Serial.print(" (");
  Serial.print(currentPct());
  Serial.print("%)");
  if (!dimmerOk) Serial.print("  ** DS3502 did not respond **");
  Serial.println();
}

bool setLevel(int level) {
  if (level < 1 || level > 5) {
    Serial.println("Invalid lights level. Please choose a value between 1 and 5.");
    return false;
  }
  writeWiper(levelToWiper(level));
  return true;
}

bool setPct(long pct) {
  if (pct < 0 || pct > 100) {
    Serial.println("Invalid percent. Please choose a value between 0 and 100.");
    return false;
  }
  writeWiper((uint8_t)((pct * DS3502_MAX + 50) / 100));
  return true;
}

// ---------------- physical controls ----------------
// Wall switch: any change of state toggles. Turning on always goes to full (level 5).
void onWallSwitchChanged() {
  if (!lightsOn) {
    writeLights(true);
    setLevel(5);
  } else {
    writeLights(false);
  }
}

// Push button: off -> 1 -> 3 -> 5 -> off. From an in-between level (set over the web)
// it steps up to the next of those presets.
void onButtonPressed() {
  static const int cycle[] = { 1, 3, 5 };
  if (!lightsOn) {
    writeLights(true);
    setLevel(1);
    return;
  }
  for (int l : cycle) {
    if (levelToWiper(l) > wiper) {
      setLevel(l);
      return;
    }
  }
  writeLights(false);
}

void pollInputs() {
  unsigned long now = millis();

  int s = digitalRead(SwitchPin);
  if (s != switchRaw) { switchRaw = s; switchChangeMs = now; }
  if (switchRaw != switchStable && now - switchChangeMs >= DEBOUNCE_MS) {
    switchStable = switchRaw;
    onWallSwitchChanged();
  }

  int b = digitalRead(Button);
  if (b != buttonRaw) { buttonRaw = b; buttonChangeMs = now; }
  if (buttonRaw != buttonStable && now - buttonChangeMs >= DEBOUNCE_MS) {
    buttonStable = buttonRaw;
    if (buttonStable == LOW) onButtonPressed();   // act on press, not release
  }
}

// ---------------- status ----------------
String stateJson() {
  String j = "{\"on\":";
  j += lightsOn ? "true" : "false";
  j += ",\"level\":";   j += currentLevel();
  j += ",\"pct\":";     j += currentPct();
  j += ",\"wiper\":";   j += wiper;
  j += ",\"dimmer\":";  j += dimmerOk ? "true" : "false";
  j += ",\"rssi\":";    j += WiFi.RSSI();
  j += ",\"uptime\":";  j += millis() / 1000;
  j += "}";
  return j;
}

void printStatus() {
  Serial.println(stateJson());
  Serial.print("Web page: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

// ---------------- serial ----------------
void handleSerial() {
  if (Serial.available() <= 0) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  if (command.length() == 0) return;
  String upper = command;
  upper.toUpperCase();

  if (upper == "TOGGLE D7" || upper == "TOGGLE") {
    writeLights(!lightsOn);
  } else if (upper == "ON") {
    writeLights(true);
  } else if (upper == "OFF") {
    writeLights(false);
  } else if (upper.startsWith("SET LIGHTS ")) {
    String levelStr = command.substring(11);
    levelStr.trim();
    int level = levelStr.toInt();
    if (level == 0) {  // toInt() returns 0 if the conversion fails
      Serial.println("Invalid lights level. Please send a number between 1 and 5.");
    } else {
      setLevel(level);
    }
  } else if (upper.startsWith("PCT ")) {
    String v = command.substring(4);
    v.trim();
    setPct(v.toInt());
  } else if (upper == "STATUS" || upper == "IP") {
    printStatus();
  } else {
    Serial.println("Commands: TOGGLE D7 | SET LIGHTS n | ON | OFF | PCT n | STATUS | IP");
  }
}

// ---------------- WiFi ----------------
void showOnMatrix(const String& text) {
  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(60);
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println("  " + text + "  ");
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();
}

// One connection attempt. WiFi.begin() blocks for a few seconds, so the physical
// controls pause while this runs -- it only happens at boot and after a dropout.
void connectWifi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not found -- running with physical controls only.");
    return;
  }
  WiFi.setHostname(HOSTNAME);
#if USE_STATIC_IP
  WiFi.config(staticIP, staticDNS, staticGW, staticMask);
#endif
  Serial.print("Connecting to ");
  Serial.println(SECRET_SSID);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
}

void maintainWifi() {
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wasConnected) {
    // DHCP can lag the association by a moment; wait for a real address.
    if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) return;
    server.begin();
    Serial.print("Connected. Web page: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    showOnMatrix(WiFi.localIP().toString());
  }
  if (!connected && wasConnected) {
    Serial.println("WiFi lost -- will retry.");
    lastWifiCheckMs = millis();
  }
  wasConnected = connected;

  if (!connected && millis() - lastWifiCheckMs >= WIFI_RETRY_MS) {
    lastWifiCheckMs = millis();
    WiFi.disconnect();
    connectWifi();
  }
}

// ---------------- web ----------------
const char PAGE_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Basement Lights</title>
<style>
:root{--bg:#f4f4f2;--card:#fff;--fg:#1d1d1b;--muted:#6b6b66;--line:#ddddd8;--accent:#e0a100;--accent-fg:#1d1d1b;--off:#c9c9c4;--bad:#c0392b}
@media (prefers-color-scheme:dark){:root{--bg:#141414;--card:#1f1f1f;--fg:#f0f0ec;--muted:#9a9a94;--line:#333;--accent:#ffbf1f;--off:#3a3a3a}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:420px;margin:0 auto;padding:24px 16px}
h1{font-size:1.4rem;margin:0 0 4px}
.sub{color:var(--muted);font-size:.9rem;margin-bottom:20px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:18px;margin-bottom:14px}
#power{width:100%;padding:22px;font-size:1.3rem;font-weight:600;border:0;border-radius:12px;cursor:pointer;background:var(--off);color:var(--fg);transition:background .15s}
#power.on{background:var(--accent);color:var(--accent-fg)}
.label{font-size:.85rem;color:var(--muted);margin-bottom:10px;display:flex;justify-content:space-between}
.levels{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}
.levels button{padding:14px 0;font-size:1.1rem;font-weight:600;border:1px solid var(--line);border-radius:10px;background:transparent;color:var(--fg);cursor:pointer}
.levels button.sel{background:var(--accent);color:var(--accent-fg);border-color:var(--accent)}
input[type=range]{width:100%;accent-color:var(--accent);height:32px}
#status{font-size:.85rem;color:var(--muted);text-align:center}
#status.bad{color:var(--bad)}
</style></head><body><main>
<h1>Basement Lights</h1>
<div class="sub">Changes from the wall switch and button show up here too.</div>
<div class="card"><button id="power">…</button></div>
<div class="card">
  <div class="label"><span>Level</span><span id="lvlText"></span></div>
  <div class="levels" id="levels"></div>
</div>
<div class="card">
  <div class="label"><span>Fine dimmer</span><span id="pctText"></span></div>
  <input type="range" id="pct" min="0" max="100" step="1">
</div>
<div id="status">Connecting…</div>
</main><script>
const $=id=>document.getElementById(id);
const lv=$('levels');
for(let i=1;i<=5;i++){const b=document.createElement('button');b.textContent=i;b.onclick=()=>send('/api/level?n='+i);lv.appendChild(b);}
let dragging=false;
function render(s){
  const p=$('power');p.textContent=s.on?'ON':'OFF';p.classList.toggle('on',s.on);
  [...lv.children].forEach((b,i)=>b.classList.toggle('sel',s.on&&s.level===i+1));
  $('lvlText').textContent=s.level?('Level '+s.level):'Custom';
  if(!dragging)$('pct').value=s.pct;
  $('pctText').textContent=s.pct+'%';
  const st=$('status');
  st.classList.toggle('bad',!s.dimmer);
  st.textContent=s.dimmer?('Connected · signal '+s.rssi+' dBm'):'Dimmer chip not responding (check I2C wiring)';
}
function fail(){const st=$('status');st.classList.add('bad');st.textContent='Lost contact with the lights controller';}
async function send(url){try{const r=await fetch(url,{method:'POST'});render(await r.json());}catch(e){fail();}}
async function poll(){try{const r=await fetch('/api/state');render(await r.json());}catch(e){fail();}}
$('power').onclick=()=>send('/api/toggle');
const sl=$('pct');
sl.addEventListener('input',()=>{dragging=true;$('pctText').textContent=sl.value+'%';});
sl.addEventListener('change',()=>{dragging=false;send('/api/pct?v='+sl.value);});
poll();setInterval(poll,2000);
</script></body></html>
)HTML";

// Value of ?key=... in a path, or "" if absent.
String queryParam(const String& path, const char* key) {
  int q = path.indexOf('?');
  if (q < 0) return "";
  String k = String(key) + "=";
  int start = q + 1;
  while (start < (int)path.length()) {
    int amp = path.indexOf('&', start);
    if (amp < 0) amp = path.length();
    if (path.substring(start, start + k.length()) == k)
      return path.substring(start + k.length(), amp);
    start = amp + 1;
  }
  return "";
}

void sendResponse(WiFiClient& c, int code, const char* type, const char* body, size_t len) {
  c.print("HTTP/1.1 ");
  c.print(code);
  c.println(code == 200 ? " OK" : (code == 404 ? " Not Found" : " Bad Request"));
  c.print("Content-Type: ");
  c.println(type);
  c.print("Content-Length: ");
  c.println(len);
  c.println("Cache-Control: no-store");
  c.println("Connection: close");
  c.println();
  // The R4's WiFi bridge is happier with modest writes than one big one.
  const size_t CHUNK = 512;
  for (size_t i = 0; i < len; i += CHUNK) {
    c.write((const uint8_t*)body + i, min(CHUNK, len - i));
  }
}

void sendJson(WiFiClient& c, int code, const String& json) {
  sendResponse(c, code, "application/json", json.c_str(), json.length());
}

void handleWeb() {
  if (!wasConnected) return;   // server isn't started until we have an address
  WiFiClient c = server.available();
  if (!c) return;

  c.setTimeout(HTTP_TIMEOUT_MS);
  String requestLine = c.readStringUntil('\n');
  requestLine.trim();
  // Drain the headers; we don't need any of them.
  unsigned long t0 = millis();
  while (c.connected() && millis() - t0 < HTTP_TIMEOUT_MS) {
    String h = c.readStringUntil('\n');
    if (h.length() <= 1) break;   // blank line ("\r") ends the headers
  }

  // "GET /path?query HTTP/1.1"
  int sp1 = requestLine.indexOf(' ');
  int sp2 = requestLine.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) { c.stop(); return; }
  String path = requestLine.substring(sp1 + 1, sp2);
  String route = path;
  int q = route.indexOf('?');
  if (q >= 0) route = route.substring(0, q);

  if (route == "/" || route == "/index.html") {
    sendResponse(c, 200, "text/html; charset=utf-8", PAGE_HTML, strlen(PAGE_HTML));
  } else if (route == "/api/state") {
    sendJson(c, 200, stateJson());
  } else if (route == "/api/on") {
    writeLights(true);
    sendJson(c, 200, stateJson());
  } else if (route == "/api/off") {
    writeLights(false);
    sendJson(c, 200, stateJson());
  } else if (route == "/api/toggle") {
    writeLights(!lightsOn);
    sendJson(c, 200, stateJson());
  } else if (route == "/api/level") {
    if (setLevel(queryParam(path, "n").toInt())) {
      writeLights(true);
      sendJson(c, 200, stateJson());
    } else {
      sendJson(c, 400, "{\"error\":\"n must be 1-5\"}");
    }
  } else if (route == "/api/pct") {
    String v = queryParam(path, "v");
    if (v.length() > 0 && setPct(v.toInt())) {
      writeLights(true);
      sendJson(c, 200, stateJson());
    } else {
      sendJson(c, 400, "{\"error\":\"v must be 0-100\"}");
    }
  } else {
    sendJson(c, 404, "{\"error\":\"not found\"}");
  }

  // Let the bytes leave before closing -- the R4 truncates if stop() comes too soon.
  c.flush();
  delay(2);
  c.stop();
}

// ---------------- main ----------------
void setup() {
  Serial.begin(9600);
  DIMMER_WIRE.begin();
  matrix.begin();

  pinMode(LightsPin, OUTPUT);
  pinMode(SwitchPin, INPUT);
  pinMode(Button, INPUT_PULLUP);

  writeLights(false);
  setLevel(3);

  // Take the switch's current position as the baseline so boot doesn't toggle.
  switchRaw = switchStable = digitalRead(SwitchPin);
  buttonRaw = buttonStable = digitalRead(Button);

  connectWifi();
  lastWifiCheckMs = millis();
}

void loop() {
  pollInputs();
  handleSerial();
  maintainWifi();
  handleWeb();
}
