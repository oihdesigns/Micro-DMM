// OLED_TextBoard — LonelyBinary ESP32-C3 dev board with the 0.42" 72x40 OLED.
//
// Joins your Wi-Fi and serves a web page with a text box. Whatever you submit
// is word-wrapped onto the OLED. Text that doesn't fit scrolls vertically.
// The last message is saved in flash so it comes back after a power cycle.
//
// Arduino IDE: board "ESP32C3 Dev Module", USB CDC On Boot: Enabled.
// Library: U8g2 (Library Manager).
//
// Use: open the board's IP (shown on the OLED at boot and printed on Serial),
// or http://textboard.local on machines with mDNS. Also scriptable:
//   curl "http://<ip>/set?text=Hello%20world&size=auto"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <vector>

// ── WiFi configuration ────────────────────────────────────────────────────────
#define WIFI_SSID   "MTBs & Science"
#define WIFI_PASS   "x"
#define MDNS_NAME   "textboard"          // -> http://textboard.local

// ── Board pins (from the LonelyBinary card) ───────────────────────────────────
#define SCK_PIN  6
#define SDA_PIN  5
#define LED_PIN  8                       // active LOW

U8G2_SSD1306_72X40_ER_F_SW_I2C u8g2(U8G2_R0, SCK_PIN, SDA_PIN);
WebServer   server(80);
Preferences prefs;

static const int    SCREEN_W   = 72;
static const int    SCREEN_H   = 40;
static const size_t MAX_TEXT   = 500;

// ── Fonts: size names offered on the web page ─────────────────────────────────
struct FontChoice { const char* name; const uint8_t* font; int lineH; };
static const FontChoice FONTS[] = {
  { "small",  u8g2_font_5x7_tf,  8  },   // 14 chars x 5 lines
  { "medium", u8g2_font_6x10_tf, 10 },   // 12 chars x 4 lines
  { "large",  u8g2_font_9x15_tf, 15 },   //  8 chars x 2 lines
};
static const int N_FONTS = sizeof(FONTS) / sizeof(FONTS[0]);

// ── Display state ─────────────────────────────────────────────────────────────
static String              curText  = "Hello!";
static String              curSize  = "auto";   // "auto" or a FONTS[].name
static int                 fontIdx  = 0;
static std::vector<String> lines;
static int                 scrollY  = 0;
static uint32_t            lastStep = 0;
static uint32_t            pauseUntil = 0;
static uint32_t            showIpUntil = 0;
static uint32_t            ledOffAt = 0;

static const int      SCROLL_GAP   = 12;     // blank px between loop copies
static const uint32_t SCROLL_MS    = 60;     // per-pixel step
static const uint32_t SCROLL_PAUSE = 1500;   // hold at top of each loop

// ── Word wrap ─────────────────────────────────────────────────────────────────
// Splits on '\n' and spaces; words wider than the screen are broken by
// character (UTF-8 aware so multibyte glyphs aren't split).
static int textWidth(const String& s) { return u8g2.getUTF8Width(s.c_str()); }

static void wrapText(const String& text, std::vector<String>& out) {
  out.clear();
  int start = 0;
  while (start <= (int)text.length()) {
    int nl = text.indexOf('\n', start);
    if (nl < 0) nl = text.length();
    String para = text.substring(start, nl);
    para.replace("\r", "");
    start = nl + 1;

    String line;
    int p = 0;
    while (p < (int)para.length()) {
      int sp = para.indexOf(' ', p);
      if (sp < 0) sp = para.length();
      String word = para.substring(p, sp);
      p = sp + 1;
      if (word.length() == 0) continue;

      String trial = line.length() ? line + " " + word : word;
      if (textWidth(trial) <= SCREEN_W) { line = trial; continue; }
      if (line.length()) { out.push_back(line); line = ""; }

      // Word alone too wide: hard-break it.
      while (textWidth(word) > SCREEN_W) {
        int cut = 0, i = 0;
        while (i < (int)word.length()) {
          int j = i + 1;
          while (j < (int)word.length() && ((uint8_t)word[j] & 0xC0) == 0x80) j++;
          if (textWidth(word.substring(0, j)) > SCREEN_W) break;
          cut = i = j;
        }
        if (cut == 0) cut = 1;
        out.push_back(word.substring(0, cut));
        word = word.substring(cut);
      }
      line = word;
    }
    out.push_back(line);   // keeps blank lines from blank paragraphs
  }
  while (!out.empty() && out.back().length() == 0) out.pop_back();
  if (out.empty()) out.push_back("");
}

// Pick font, wrap, reset scroll. "auto" = largest font that fits without scrolling.
static void layoutText() {
  fontIdx = -1;
  for (int i = 0; i < N_FONTS; i++)
    if (curSize == FONTS[i].name) fontIdx = i;

  if (fontIdx < 0) {
    fontIdx = 0;
    for (int i = N_FONTS - 1; i >= 0; i--) {
      u8g2.setFont(FONTS[i].font);
      wrapText(curText, lines);
      if ((int)lines.size() * FONTS[i].lineH <= SCREEN_H) { fontIdx = i; break; }
    }
  }
  u8g2.setFont(FONTS[fontIdx].font);
  wrapText(curText, lines);
  scrollY    = 0;
  pauseUntil = millis() + SCROLL_PAUSE;
}

static int blockHeight() { return lines.size() * FONTS[fontIdx].lineH; }

static void drawBlock(int top) {
  int lh = FONTS[fontIdx].lineH;
  for (size_t i = 0; i < lines.size(); i++) {
    int y = top + i * lh;
    if (y + lh < 0 || y >= SCREEN_H) continue;
    int x = (SCREEN_W - textWidth(lines[i])) / 2;
    u8g2.drawUTF8(x < 0 ? 0 : x, y, lines[i].c_str());
  }
}

static void render() {
  u8g2.clearBuffer();
  u8g2.setFont(FONTS[fontIdx].font);
  u8g2.setFontPosTop();
  int h = blockHeight();
  if (h <= SCREEN_H) {
    drawBlock((SCREEN_H - h) / 2);
  } else {
    drawBlock(-scrollY);
    drawBlock(-scrollY + h + SCROLL_GAP);   // second copy for seamless loop
  }
  u8g2.sendBuffer();
}

static void showStatus(const char* l1, const char* l2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  u8g2.drawStr(0, 4,  l1);
  u8g2.drawStr(0, 20, l2);
  u8g2.sendBuffer();
}

static void setText(const String& text, const String& size) {
  curText = text.substring(0, MAX_TEXT);
  curSize = size.length() ? size : "auto";
  prefs.putString("text", curText);
  prefs.putString("size", curSize);
  layoutText();
  showIpUntil = 0;
  render();
  digitalWrite(LED_PIN, LOW);              // blink LED on update
  ledOffAt = millis() + 150;
  Serial.printf("New text (%s, %u lines): %s\n",
                FONTS[fontIdx].name, (unsigned)lines.size(), curText.c_str());
}

// ── Web page ──────────────────────────────────────────────────────────────────
static String htmlEscape(const String& s) {
  String o;
  o.reserve(s.length() + 16);
  for (char c : s) {
    switch (c) {
      case '&': o += "&amp;";  break;
      case '<': o += "&lt;";   break;
      case '>': o += "&gt;";   break;
      case '"': o += "&quot;"; break;
      default:  o += c;
    }
  }
  return o;
}

static String sizeOption(const char* name, const char* label) {
  String o = "<option value=\"";
  o += name;
  o += '"';
  if (curSize == name) o += " selected";
  o += '>';
  o += label;
  o += "</option>";
  return o;
}

static void handleRoot() {
  String page;
  page.reserve(2500);
  page += F("<!doctype html><html><head><meta charset=utf-8>"
            "<meta name=viewport content='width=device-width,initial-scale=1'>"
            "<title>OLED Text Board</title><style>"
            "body{font-family:system-ui,sans-serif;background:#1a1a1a;color:#eee;"
            "max-width:480px;margin:0 auto;padding:16px}"
            "h1{font-size:1.3em;color:#00ff32}"
            "textarea{width:100%;box-sizing:border-box;height:8em;font-size:1.1em;"
            "background:#000;color:#00ff32;border:1px solid #444;border-radius:6px;padding:8px}"
            "select,button{font-size:1em;padding:8px 14px;border-radius:6px;margin-top:10px}"
            "button{background:#00ff32;color:#000;border:0;font-weight:600}"
            "small{color:#888}</style></head><body>"
            "<h1>OLED Text Board</h1>"
            "<form method=post action=/set>"
            "<textarea name=text maxlength=500 autofocus>");
  page += htmlEscape(curText);
  page += F("</textarea><br><label>Size <select name=size>");
  page += sizeOption("auto",   "Auto (largest that fits)");
  page += sizeOption("small",  "Small (14 x 5)");
  page += sizeOption("medium", "Medium (12 x 4)");
  page += sizeOption("large",  "Large (8 x 2)");
  page += F("</select></label> <button type=submit>Show</button></form>"
            "<p><small>Ctrl+Enter submits. Text that doesn't fit scrolls.<br>Now showing: ");
  page += FONTS[fontIdx].name;
  page += ", ";
  page += String(lines.size());
  page += F(" line(s)</small></p><script>"
            "document.querySelector('textarea').addEventListener('keydown',e=>{"
            "if(e.key==='Enter'&&(e.ctrlKey||e.metaKey))e.target.form.submit()});"
            "</script></body></html>");
  server.send(200, "text/html; charset=utf-8", page);
}

static void handleSet() {
  if (!server.hasArg("text")) { server.send(400, "text/plain", "missing 'text'\n"); return; }
  setText(server.arg("text"), server.arg("size"));
  if (server.method() == HTTP_POST) {     // browser form -> back to the page
    server.sendHeader("Location", "/");
    server.send(303);
  } else {                                // curl / scripts
    server.send(200, "text/plain", "ok\n");
  }
}

// ── Setup / loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  u8g2.begin();
  showStatus("Connecting", WIFI_SSID);

  prefs.begin("textboard", false);
  curText = prefs.getString("text", curText);
  curSize = prefs.getString("size", curSize);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);     // card: 20 dBm default is unstable on this antenna

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(250);
  }
  digitalWrite(LED_PIN, HIGH);

  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    Serial.printf("Connected. Open http://%s  (or http://%s.local)\n", ip.c_str(), MDNS_NAME);
    if (MDNS.begin(MDNS_NAME)) MDNS.addService("http", "tcp", 80);
    showStatus("Open in browser:", ip.c_str());
  } else {
    Serial.println("WiFi connect failed - check WIFI_SSID / WIFI_PASS. Will keep retrying.");
    showStatus("WiFi failed", "retrying...");
  }
  showIpUntil = millis() + 6000;

  server.on("/",    HTTP_GET, handleRoot);
  server.on("/set", handleSet);
  server.onNotFound([] { server.send(404, "text/plain", "not found\n"); });
  server.begin();

  layoutText();
}

void loop() {
  server.handleClient();
  uint32_t now = millis();

  if (ledOffAt && now >= ledOffAt) { digitalWrite(LED_PIN, HIGH); ledOffAt = 0; }

  // Wi-Fi came up after the boot timeout: show the IP again.
  static bool mdnsUp = WiFi.status() == WL_CONNECTED;
  if (!mdnsUp && WiFi.status() == WL_CONNECTED) {
    mdnsUp = true;
    String ip = WiFi.localIP().toString();
    Serial.printf("Connected. Open http://%s\n", ip.c_str());
    if (MDNS.begin(MDNS_NAME)) MDNS.addService("http", "tcp", 80);
    showStatus("Open in browser:", ip.c_str());
    showIpUntil = now + 6000;
  }

  // Boot status screen, then the message.
  if (showIpUntil) {
    if (now < showIpUntil) return;
    showIpUntil = 0;
    render();
  }

  // Scroll when the text is taller than the screen.
  int h = blockHeight();
  if (h > SCREEN_H && now >= pauseUntil && now - lastStep >= SCROLL_MS) {
    lastStep = now;
    if (++scrollY >= h + SCROLL_GAP) {
      scrollY    = 0;
      pauseUntil = now + SCROLL_PAUSE;
    }
    render();
  }
}
