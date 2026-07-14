#include <Arduino_AdvancedAnalog.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>
#include <WiFi.h>            // Giga R1 WiFi onboard Murata module

// ── WiFi configuration ─────────────────────────────────────────────────────────
// Fill in your network. The board joins your Wi-Fi and listens for the same
// line-based command protocol used over USB serial, but on a TCP socket.
#define WIFI_SSID   "MTBs & Science"
#define WIFI_PASS   "x"
static const uint16_t CMD_PORT = 8080;   // TCP port the GUI connects to

WiFiServer cmdServer(CMD_PORT);
static bool   wifiActive = false;
static char   wifiIPStr[20] = "no wifi";

// ── ADC ───────────────────────────────────────────────────────────────────────
static const int N_CHANNELS    = 8;
static const int MAX_LOG       = 10000;
static const int N_SAMPLES_MAX = 512;
static const int QUEUE_DEPTH   = 8;

AdvancedADC adc(A0, A1, A2, A3, A4, A5, A6, A7);
static const char* CH_NAMES[N_CHANNELS] = {"A0","A1","A2","A3","A4","A5","A6","A7"};
static int  logData[N_CHANNELS][MAX_LOG];
static bool adcRunning = false;

static uint32_t toResolution(int bits) {
    if (bits <= 8)  return AN_RESOLUTION_8;
    if (bits <= 10) return AN_RESOLUTION_10;
    if (bits <= 12) return AN_RESOLUTION_12;
    if (bits <= 14) return AN_RESOLUTION_14;
    return AN_RESOLUTION_16;
}

// ── Display / touch / USB ─────────────────────────────────────────────────────
GigaDisplay_GFX          gfx;
Arduino_GigaDisplayTouch touch;
bool displayPresent = false;

USBHostMSD          msd;
mbed::FATFileSystem usbFs("usb");
bool usbMounted = false;

// ── Colors ────────────────────────────────────────────────────────────────────
#define C_BG      0x0000u
#define C_PANEL   0x2104u
#define C_BORDER  0x4208u
#define C_TEXT    0xFFFFu
#define C_CYAN    0x07FFu
#define C_GREEN   0x07E0u
#define C_RED     0xF800u
#define C_YELLOW  0xFFE0u
#define C_MAGENTA 0xF81Fu

static const uint16_t CH_COLORS[8] = {
    0x1BB6u, 0xFBE1u, 0x2CC5u, 0xD009u,
    0x93D5u, 0x8B0Bu, 0xE336u, 0x7BEFu,
};
static const uint16_t DIFF_COLORS[2] = { C_RED, C_MAGENTA };

// ── Preset data (hardcoded from giga_presets.json) ────────────────────────────
struct PChan { bool act; int typ; float off, scl; };  // typ: 1=V 2=I
struct DiffCfg { bool en; int pos, neg, typ; float off, scl; };
struct Preset {
    const char* name;
    int  rateIdx, bits, timeMs, smooth, points;
    PChan ch[8];
    DiffCfg d1, d2;
};

// Rate options — idx matches RATE_OPTIONS[] below
// 0=10k 1=44.1k 2=50k 3=100k 4=250k 5=500k 6=1M
static const uint32_t RATE_OPTIONS[] =
    {10000UL,44100UL,50000UL,100000UL,250000UL,500000UL,1000000UL};
static const int N_RATE_OPT = 7;

// Generated from giga_presets.json — 7 preset(s)
static const Preset PRESETS[] = {
  { "Board1",
    1,16,25,1,5000,
    { {true,1,1.612548f,44.462994f},{true,2,1.613425f,10.900730f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {false,4,5,1,0,1}, {false,2,3,1,0,1} },
  { "Board1 DiffA",
    1,16,25,1,5000,
    { {true,1,1.612548f,44.462994f},{true,2,1.613425f,10.900730f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {false,4,5,2,0.0f,11.8f}, {false,2,3,1,0,1} },
  { "Board1 DiffA 5mOhm",
    3,14,30,2,2000,
    { {true,1,1.612548f,44.462994f},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,4,5,2,0.0f,44.107620f}, {false,2,3,1,0.0f,1.0f} },
  { "120VAC Board",
    3,14,50,1,4000,
    { {true,2,0,1},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,4,5,2,0.0f,1.181836f}, {true,2,3,1,0.0f,102.779803f} },
  { "120VAC Board 25A",
    0,16,100,5,4000,
    { {false,2,0,1},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,3,2,2,0.0f,11.917674f}, {true,5,4,1,0.0f,102.508232f} },
  { "47V board",
    1,16,50,3,4000,
    { {false,2,0,1},{false,2,1.613425f,41.124543f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},
      {false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f},{false,1,0.0f,1.0f} },
    {true,3,2,2,0.0f,10.603871f}, {true,5,4,1,0.0f,22.550535f} },
};
static const int N_PRESETS = 6;



// ── Screen state ──────────────────────────────────────────────────────────────
enum Screen { SCR_SETTINGS, SCR_CAPTURING, SCR_PLOT, SCR_PRESETS, SCR_DIFF, SCR_LOGGER, SCR_LOGGING };
static Screen currentScreen = SCR_SETTINGS;

// ── GUI parameters ────────────────────────────────────────────────────────────
// dChanType: 0=off 1=V 2=I
static int   dChanType[N_CHANNELS]   = {1,0,0,0,0,0,0,0};
static float dChanOffset[N_CHANNELS] = {0,0,0,0,0,0,0,0};
static float dChanScale[N_CHANNELS]  = {1,1,1,1,1,1,1,1};
static DiffCfg dDiff[2] = {{false,4,5,2,0,1},{false,2,3,1,0,1}};

static int dRateIdx = 4;   // 250 kHz default
static int dBits    = 12;
static int dTime    = 100;
static int dSmooth  = 1;
static int dPoints  = 1000;
static int dTrigThr = 0;

// ── Post-capture state ────────────────────────────────────────────────────────
static bool     captureReady  = false;
static bool     lastPinReq[N_CHANNELS]  = {};
static int      lastLogCnt[N_CHANNELS] = {};
static int      lastChanType[N_CHANNELS] = {};
static float    lastChanOffset[N_CHANNELS] = {};
static float    lastChanScale[N_CHANNELS]  = {};
static DiffCfg  lastDiff[2];
static int      lastBitRes    = 12;
static uint32_t lastElapsedMs = 0;
static bool     lastMidMode   = false;
static int      lastMidRingHead=0, lastMidRingFill=0, lastMidPostCount=0, lastMidPreSize=0;

// ── Continuous view ───────────────────────────────────────────────────────────
static bool     contMode   = false;
static uint32_t lastContMs = 0;

// ── Datalogger state ──────────────────────────────────────────────────────────
static float    logThresh[N_CHANNELS]    = {};
static float    logDiffThresh[2]         = {};
static int      logIntervalMs            = 1000;
static bool     logRunning               = false;
static FILE*    logFile                  = nullptr;
static int      logFileNum               = 0;
static int      logRowCount              = 0;
static uint32_t logStartMs               = 0;
static uint32_t logStartUs               = 0;
static uint32_t logLastWriteMs           = 0;
static float    logLastCh[N_CHANNELS]    = {};
static float    logLastDiff[2]           = {};
static long     logSmoothBuf[N_CHANNELS] = {};
static int      logSmoothTick            = 0;
static float    logCurCh[N_CHANNELS]     = {};
static float    logCurDiff[2]            = {};
static uint32_t logLastDispMs            = 0;

// ── Numpad overlay state ──────────────────────────────────────────────────────
static bool  numpadActive = false;
static char  numpadBuf[12] = "";
static int   numpadTarget  = -1;   // 0-7=logThresh[ch], 8-9=logDiffThresh[d], 10=logIntervalMs
static char  numpadTitle[24] = "";

// ── Touch debounce ────────────────────────────────────────────────────────────
static uint32_t       lastTouchMs        = 0;
static const uint32_t TOUCH_DEBOUNCE_MS  = 250;

// ── Layout constants ──────────────────────────────────────────────────────────
static const int DISP_W    = 800;
static const int DISP_H    = 480;
static const int HDR_H     = 44;
static const int CH_ROW_Y  = HDR_H + 4;
static const int CH_ROW_H  = 52;
static const int PARAM_Y   = CH_ROW_Y + CH_ROW_H + 4;
static const int PARAM_ROW = 40;
static const int N_PARAMS  = 6;
static const int DIFF_BTN_Y = PARAM_Y + N_PARAMS * PARAM_ROW + 4;
static const int DIFF_BTN_H = 44;
static const int CAP_BTN_Y  = DIFF_BTN_Y + DIFF_BTN_H + 4;
static const int CAP_BTN_H  = DISP_H - CAP_BTN_Y - 4;
// CAPTURE + CONT side-by-side split
static const int CAP_W  = 580;
static const int CONT_X = 4 + CAP_W + 4;
static const int CONT_W = DISP_W - 8 - CAP_W - 4;

static const int PL_X0      = 10;
static const int PL_LABEL_W = 185;
static const int PL_BTN_W   = 52;
static const int PL_VAL_W   = 160;

static const char* PARAM_LABELS[N_PARAMS] =
    {"Rate (Hz)","Bits","Time (ms)","Smooth","Log Points","Trig Thresh"};

// Plot area — narrower to leave room for right axis labels
static const int PLT_X      = 62;
static const int PLT_Y      = HDR_H + 4;
static const int PLT_W      = DISP_W - PLT_X - 54;
static const int STAT_ROW_H = 11;  // px per stats row below the plot

// ── Calibration math ──────────────────────────────────────────────────────────
static float vStep(int bits) { return 3.3f / (float)((1 << bits) - 1); }
static float calChan(int raw, float offset, float scale, int bits) {
    return (raw * vStep(bits) - offset) * scale;
}
static float calDiff(int rp, int rn, float offset, float scale, int bits) {
    return ((rp - rn) * vStep(bits) - offset) * scale;
}

// ── Drawing helpers ───────────────────────────────────────────────────────────
static void drawBtn(int x, int y, int w, int h,
                    uint16_t fill, uint16_t tc, const char* lbl, int ts=2) {
    gfx.fillRoundRect(x,y,w,h,5,fill);
    gfx.drawRoundRect(x,y,w,h,5,C_BORDER);
    gfx.setTextColor(tc); gfx.setTextSize(ts);
    int tw = strlen(lbl)*6*ts, th = 8*ts;
    gfx.setCursor(x+(w-tw)/2, y+(h-th)/2);
    gfx.print(lbl);
}
static bool hit(int bx,int by,int bw,int bh,int tx,int ty) {
    return tx>=bx && tx<bx+bw && ty>=by && ty<by+bh;
}

// ── Preset application ────────────────────────────────────────────────────────
static void applyPreset(int idx) {
    const Preset& p = PRESETS[idx];
    dRateIdx = p.rateIdx;
    dBits    = p.bits;
    dTime    = p.timeMs;
    dSmooth  = max(1, p.smooth);
    dPoints  = p.points;
    for (int i=0;i<N_CHANNELS;i++) {
        dChanType[i]   = p.ch[i].act ? p.ch[i].typ : 0;
        dChanOffset[i] = p.ch[i].off;
        dChanScale[i]  = p.ch[i].scl;
    }
    dDiff[0] = p.d1;
    dDiff[1] = p.d2;
}

// ── Settings screen ───────────────────────────────────────────────────────────
static void getParamStr(int r, char* buf, int sz) {
    switch(r) {
        case 0: snprintf(buf,sz,"%lu",(unsigned long)RATE_OPTIONS[dRateIdx]); break;
        case 1: snprintf(buf,sz,"%d",dBits);   break;
        case 2: snprintf(buf,sz,"%d",dTime);   break;
        case 3: snprintf(buf,sz,"%d",dSmooth); break;
        case 4: snprintf(buf,sz,"%d",dPoints); break;
        case 5: if(dTrigThr==0) snprintf(buf,sz,"OFF");
                else            snprintf(buf,sz,"%d",dTrigThr); break;
    }
}
static void refreshParamValue(int r) {
    int ry = PARAM_Y + r*PARAM_ROW;
    int vx = PL_X0 + PL_LABEL_W + PL_BTN_W;
    gfx.fillRect(vx,ry,PL_VAL_W,PARAM_ROW,(r&1)?C_PANEL:C_BG);
    char vbuf[32]; getParamStr(r,vbuf,sizeof(vbuf));
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    int vw = strlen(vbuf)*12;
    gfx.setCursor(vx+(PL_VAL_W-vw)/2, ry+(PARAM_ROW-16)/2);
    gfx.print(vbuf);
}
static void drawCaptureButtons() {
    int n=0;
    for(int i=0;i<N_CHANNELS;i++) if(dChanType[i]) n++;
    if(dDiff[0].en||dDiff[1].en) n++;
    bool ok=(n>0);
    drawBtn(4,      CAP_BTN_Y, CAP_W, CAP_BTN_H, ok?C_GREEN:C_BORDER, C_TEXT, ok?"CAPTURE":"Select a channel", 3);
    drawBtn(CONT_X, CAP_BTN_Y, CONT_W,CAP_BTN_H, ok?C_CYAN :C_BORDER, C_TEXT, "CONT", 2);
}
static void drawSettingsScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Giga ADC Capture");
    // WiFi endpoint (small, between title and buttons)
    gfx.setTextColor(wifiActive?C_GREEN:C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(320,18); gfx.print("WiFi: "); gfx.print(wifiIPStr);
    drawBtn(DISP_W-164,2,78,HDR_H-4,C_BORDER,C_TEXT,"Presets",2);
    drawBtn(DISP_W-82,2,78,HDR_H-4,C_BORDER,C_TEXT,"Logger",2);

    // Channel toggles — label shows type
    int cbw = DISP_W/N_CHANNELS;
    for(int i=0;i<N_CHANNELS;i++) {
        char lbl[8];
        if(dChanType[i]==0)      snprintf(lbl,sizeof(lbl),"%s",CH_NAMES[i]);
        else if(dChanType[i]==1) snprintf(lbl,sizeof(lbl),"%s V",CH_NAMES[i]);
        else                     snprintf(lbl,sizeof(lbl),"%s I",CH_NAMES[i]);
        drawBtn(i*cbw+2,CH_ROW_Y,cbw-4,CH_ROW_H-4,
                dChanType[i]?CH_COLORS[i]:C_PANEL, C_TEXT, lbl, 2);
    }

    // Param rows
    for(int r=0;r<N_PARAMS;r++) {
        int ry=PARAM_Y+r*PARAM_ROW;
        if(r&1) gfx.fillRect(0,ry,DISP_W,PARAM_ROW,C_PANEL);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        gfx.setCursor(PL_X0+4, ry+(PARAM_ROW-16)/2);
        gfx.print(PARAM_LABELS[r]);
        int bx=PL_X0+PL_LABEL_W;
        drawBtn(bx,ry+2,PL_BTN_W,PARAM_ROW-4,C_BORDER,C_TEXT,"-",3);
        char vbuf[32]; getParamStr(r,vbuf,sizeof(vbuf));
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        int vw=strlen(vbuf)*12;
        gfx.setCursor(bx+PL_BTN_W+(PL_VAL_W-vw)/2, ry+(PARAM_ROW-16)/2);
        gfx.print(vbuf);
        drawBtn(bx+PL_BTN_W+PL_VAL_W,ry+2,PL_BTN_W,PARAM_ROW-4,C_BORDER,C_TEXT,"+",3);
    }

    // Diff config button — shows current diff summary
    char dbuf[48] = "Diff: off";
    if(dDiff[0].en && dDiff[1].en)
        snprintf(dbuf,sizeof(dbuf),"Diff: %s-%s & %s-%s",
            CH_NAMES[dDiff[0].pos],CH_NAMES[dDiff[0].neg],
            CH_NAMES[dDiff[1].pos],CH_NAMES[dDiff[1].neg]);
    else if(dDiff[0].en)
        snprintf(dbuf,sizeof(dbuf),"Diff: %s-%s (%s)",
            CH_NAMES[dDiff[0].pos],CH_NAMES[dDiff[0].neg],
            dDiff[0].typ==1?"V":"I");
    else if(dDiff[1].en)
        snprintf(dbuf,sizeof(dbuf),"Diff: %s-%s (%s)",
            CH_NAMES[dDiff[1].pos],CH_NAMES[dDiff[1].neg],
            dDiff[1].typ==1?"V":"I");
    drawBtn(4,DIFF_BTN_Y,DISP_W-8,DIFF_BTN_H,C_PANEL,C_CYAN,dbuf,2);

    drawCaptureButtons();
}

// ── Presets screen ────────────────────────────────────────────────────────────
static const int PRE_COLS  = 5;
static const int PRE_ROWS  = 5;
static const int PRE_CW    = DISP_W / PRE_COLS;          // 160 px per cell
static const int PRE_RH    = (DISP_H - HDR_H) / PRE_ROWS; // 87 px per cell
static const int PRE_BTN_W = PRE_CW - 4;                 // 156 px
static const int PRE_BTN_H = PRE_RH - 3;                 // 84 px

static void drawPresetsScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Select Preset");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_BORDER,C_TEXT,"< Back",2);
    int n = min(N_PRESETS, PRE_COLS * PRE_ROWS);
    for(int i=0;i<n;i++) {
        int bx = (i % PRE_COLS) * PRE_CW + 2;
        int by = HDR_H + (i / PRE_COLS) * PRE_RH + 1;
        int ts = (strlen(PRESETS[i].name) * 12 <= PRE_BTN_W) ? 2 : 1;
        drawBtn(bx, by, PRE_BTN_W, PRE_BTN_H, C_PANEL, C_TEXT, PRESETS[i].name, ts);
    }
}

// ── Diff config screen ────────────────────────────────────────────────────────
static void drawDiffRow(int d) {
    int ry = 60 + d*110;
    uint16_t bg = (d==0) ? C_BG : C_PANEL;
    gfx.fillRect(0,ry-2,DISP_W,56,bg);

    DiffCfg& dc = dDiff[d];
    drawBtn(10,ry,160,50,dc.en?C_GREEN:C_BORDER,C_TEXT,dc.en?"ON":"OFF",2);

    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(182,ry+17); gfx.print("Pos:");
    drawBtn(228,ry,44,50,C_BORDER,C_TEXT,"<",3);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    gfx.setCursor(278,ry+17); gfx.print(CH_NAMES[dc.pos]);
    drawBtn(322,ry,44,50,C_BORDER,C_TEXT,">",3);

    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(382,ry+17); gfx.print("Neg:");
    drawBtn(428,ry,44,50,C_BORDER,C_TEXT,"<",3);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    gfx.setCursor(478,ry+17); gfx.print(CH_NAMES[dc.neg]);
    drawBtn(522,ry,44,50,C_BORDER,C_TEXT,">",3);

    drawBtn(588,ry,88,50,dc.typ==1?C_CYAN:C_BORDER,C_TEXT,"V",3);
    drawBtn(684,ry,88,50,dc.typ==2?C_CYAN:C_BORDER,C_TEXT,"I",3);
}
static void drawDiffScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Differential Inputs");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_GREEN,C_TEXT,"Done",2);

    gfx.setTextColor(C_BORDER); gfx.setTextSize(2);
    gfx.setCursor(10,54); gfx.print("── Diff 1 ──────────────────────────────────────────");
    drawDiffRow(0);
    gfx.setTextColor(C_BORDER); gfx.setTextSize(2);
    gfx.setCursor(10,166); gfx.print("── Diff 2 ──────────────────────────────────────────");
    drawDiffRow(1);

    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(10,300); gfx.print("Offset & scale are loaded from presets.");
}

// ── Capture progress screen ───────────────────────────────────────────────────
static void drawCaptureScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Capturing...");
    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(20,100); gfx.print("Running ADC capture");
}
static void updateProgress(uint32_t elapsed, uint32_t total) {
    const int bx=40,by=200,bw=DISP_W-80,bh=30;
    int fill=(total>0)?(int)((uint64_t)elapsed*bw/total):0;
    if(fill>bw) fill=bw;
    gfx.fillRect(bx,by,bw,bh,C_BORDER);
    if(fill>0) gfx.fillRect(bx,by,fill,bh,C_CYAN);
    gfx.drawRect(bx,by,bw,bh,C_TEXT);
    gfx.fillRect(bx,by+bh+8,280,20,C_BG);
    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(bx,by+bh+8);
    char buf[32]; snprintf(buf,sizeof(buf),"%lu / %d ms",(unsigned long)elapsed,(int)total);
    gfx.print(buf);
}

// ── Plot screen ───────────────────────────────────────────────────────────────
struct ChanSt { float mn, mx, sum, ssq; int cnt; };

static int getRawVal(int ch, int j) {
    if(lastMidMode && j<lastMidRingFill) {
        int si=(lastMidRingFill<lastMidPreSize)?0:lastMidRingHead;
        return logData[ch][(si+j)%lastMidPreSize];
    } else if(lastMidMode) {
        return logData[ch][lastMidPreSize+(j-lastMidRingFill)];
    }
    return logData[ch][j];
}

static void drawPlotScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    gfx.setCursor(12,14);
    char hbuf[48];
    snprintf(hbuf,sizeof(hbuf),"Done  %lu ms  %d-bit",(unsigned long)lastElapsedMs,lastBitRes);
    gfx.print(hbuf);
    drawBtn(DISP_W-220,2,100,HDR_H-4,contMode?C_RED:C_BORDER,C_TEXT,contMode?"Stop":"< Back",2);
    drawBtn(DISP_W-114,2,110,HDR_H-4,usbMounted?C_GREEN:C_BORDER,C_TEXT,
            usbMounted?"Export":"No USB",2);

    // ── Count active channels → dynamic plot height ───────────────────────
    int nActive = 0;
    for(int ch=0;ch<N_CHANNELS;ch++)
        if(lastPinReq[ch]&&lastChanType[ch]!=0&&lastLogCnt[ch]>0) nActive++;
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(dc.en&&lastLogCnt[dc.pos]>0&&lastLogCnt[dc.neg]>0) nActive++;
    }
    int pltH = DISP_H - PLT_Y - 4 - nActive * STAT_ROW_H - 2;
    if(pltH < 180) pltH = 180;
    int statY = PLT_Y + pltH + 2;

    // ── Determine Y ranges AND accumulate per-channel stats ───────────────
    ChanSt chSt[N_CHANNELS], dSt[2];
    for(int i=0;i<N_CHANNELS;i++) chSt[i]={1e30f,-1e30f,0.0f,0.0f,0};
    for(int i=0;i<2;i++)           dSt[i]={1e30f,-1e30f,0.0f,0.0f,0};

    float vMin=1e30f,vMax=-1e30f,iMin=1e30f,iMax=-1e30f;
    bool hasV=false,hasI=false;

    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0||lastLogCnt[ch]==0) continue;
        bool isI=(lastChanType[ch]==2);
        ChanSt& st=chSt[ch];
        for(int j=0;j<lastLogCnt[ch];j++) {
            float v=calChan(getRawVal(ch,j),lastChanOffset[ch],lastChanScale[ch],lastBitRes);
            if(v<st.mn)st.mn=v; if(v>st.mx)st.mx=v; st.sum+=v; st.ssq+=v*v; st.cnt++;
            if(isI){if(v<iMin)iMin=v;if(v>iMax)iMax=v;hasI=true;}
            else   {if(v<vMin)vMin=v;if(v>vMax)vMax=v;hasV=true;}
        }
    }
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en||lastLogCnt[dc.pos]==0||lastLogCnt[dc.neg]==0) continue;
        bool isI=(dc.typ==2);
        int cnt=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]);
        ChanSt& st=dSt[d];
        for(int j=0;j<cnt;j++) {
            float v=calDiff(getRawVal(dc.pos,j),getRawVal(dc.neg,j),dc.off,dc.scl,lastBitRes);
            if(v<st.mn)st.mn=v; if(v>st.mx)st.mx=v; st.sum+=v; st.ssq+=v*v; st.cnt++;
            if(isI){if(v<iMin)iMin=v;if(v>iMax)iMax=v;hasI=true;}
            else   {if(v<vMin)vMin=v;if(v>vMax)vMax=v;hasV=true;}
        }
    }
    if(!hasV&&!hasI){ vMin=0;vMax=1;hasV=true; }
    if(hasV&&vMax-vMin<1e-6f){ float m=(vMin+vMax)/2; vMin=m-1;vMax=m+1; }
    if(hasI&&iMax-iMin<1e-6f){ float m=(iMin+iMax)/2; iMin=m-1;iMax=m+1; }
    if(!hasV){ vMin=iMin;vMax=iMax; }
    if(!hasI){ iMin=vMin;iMax=vMax; }

    // ── Draw plot area ────────────────────────────────────────────────────
    gfx.fillRect(PLT_X,PLT_Y,PLT_W,pltH,0x0821u);
    gfx.drawRect(PLT_X,PLT_Y,PLT_W,pltH,C_BORDER);
    for(int g=1;g<4;g++) {
        int gy=PLT_Y+g*pltH/4;
        for(int x=PLT_X;x<PLT_X+PLT_W;x+=6) gfx.drawPixel(x,gy,C_BORDER);
    }

    // Y axis labels — V left, I right
    gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
    char lbuf[12];
    snprintf(lbuf,sizeof(lbuf),"%.3g",vMax); gfx.setCursor(2,PLT_Y+2);          gfx.print(lbuf);
    snprintf(lbuf,sizeof(lbuf),"%.3g",vMin); gfx.setCursor(2,PLT_Y+pltH-10);   gfx.print(lbuf);
    if(hasI) {
        int rx=PLT_X+PLT_W+4;
        snprintf(lbuf,sizeof(lbuf),"%.3g",iMax); gfx.setCursor(rx,PLT_Y+2);         gfx.print(lbuf);
        snprintf(lbuf,sizeof(lbuf),"%.3g",iMin); gfx.setCursor(rx,PLT_Y+pltH-10);  gfx.print(lbuf);
    }

    // Find max log count for X scaling
    int maxCnt=1;
    for(int ch=0;ch<N_CHANNELS;ch++) if(lastPinReq[ch]&&lastLogCnt[ch]>maxCnt) maxCnt=lastLogCnt[ch];
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(dc.en) { int c=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]); if(c>maxCnt) maxCnt=c; }
    }

    auto mapY = [&](float v, float mn, float mx) -> int {
        int py = PLT_Y+pltH-1-(int)((v-mn)*(pltH-2)/(mx-mn));
        return constrain(py, PLT_Y, PLT_Y+pltH-1);
    };

    // Draw standalone channels
    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0||lastLogCnt[ch]<2) continue;
        bool isI=(lastChanType[ch]==2);
        float mn=isI?iMin:vMin, mx=isI?iMax:vMax;
        int ppx=-1,ppy=-1;
        for(int j=0;j<lastLogCnt[ch];j++) {
            float v=calChan(getRawVal(ch,j),lastChanOffset[ch],lastChanScale[ch],lastBitRes);
            int px=PLT_X+(int)((int64_t)j*PLT_W/maxCnt);
            int py=mapY(v,mn,mx);
            if(ppx>=0) gfx.drawLine(ppx,ppy,px,py,CH_COLORS[ch]);
            ppx=px; ppy=py;
        }
    }
    // Draw diff channels
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en) continue;
        int cnt=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]);
        if(cnt<2) continue;
        bool isI=(dc.typ==2);
        float mn=isI?iMin:vMin, mx=isI?iMax:vMax;
        int ppx=-1,ppy=-1;
        for(int j=0;j<cnt;j++) {
            float v=calDiff(getRawVal(dc.pos,j),getRawVal(dc.neg,j),dc.off,dc.scl,lastBitRes);
            int px=PLT_X+(int)((int64_t)j*PLT_W/maxCnt);
            int py=mapY(v,mn,mx);
            if(ppx>=0) gfx.drawLine(ppx,ppy,px,py,DIFF_COLORS[d]);
            ppx=px; ppy=py;
        }
    }

    // ── Stats rows ────────────────────────────────────────────────────────
    int ry = statY;
    auto drawStat = [&](uint16_t col, const char* lbl,
                        float mn, float mx, float sum, float ssq, int cnt) {
        if(cnt == 0) return;
        float avg = sum / cnt;
        float rms = sqrtf(ssq / cnt);
        gfx.fillRect(4, ry, 10, 8, col);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
        gfx.setCursor(17, ry); gfx.print(lbl);
        char buf[64];
        snprintf(buf,sizeof(buf),"Min:%.4g  Max:%.4g  Avg:%.4g  RMS:%.4g",mn,mx,avg,rms);
        gfx.setCursor(80, ry); gfx.print(buf);
        ry += STAT_ROW_H;
    };
    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0) continue;
        char lb[10]; snprintf(lb,sizeof(lb),"%s(%s)",CH_NAMES[ch],lastChanType[ch]==1?"V":"I");
        ChanSt& st=chSt[ch];
        drawStat(CH_COLORS[ch], lb, st.mn, st.mx, st.sum, st.ssq, st.cnt);
    }
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en) continue;
        char lb[10]; snprintf(lb,sizeof(lb),"D%d(%s)",d+1,dc.typ==1?"V":"I");
        ChanSt& st=dSt[d];
        drawStat(DIFF_COLORS[d], lb, st.mn, st.mx, st.sum, st.ssq, st.cnt);
    }
}

// ── Datalogger screens & logic ────────────────────────────────────────────────
static int nextLogFileNum() {
    for(int n=0;n<1000;n++){
        char fn[32]; snprintf(fn,sizeof(fn),"/usb/logger_%03d.csv",n);
        FILE* f=fopen(fn,"r"); if(!f) return n; fclose(f);
    }
    return 999;
}
static float adjThreshDn(float v) {
    if(v<=0) return 0;
    if(v>10)    return v-1.0f;
    if(v>1)     return v-0.1f;
    if(v>0.1f)  return v-0.01f;
    return (v<=0.001f)?0.0f:v-0.001f;
}
static float adjThreshUp(float v) {
    if(v<0.001f) return 0.001f;
    if(v<0.1f)   return v+0.001f;
    if(v<1)      return v+0.01f;
    if(v<10)     return v+0.1f;
    return v+1.0f;
}

static const int LG_ROW_H = 36;
static const int LG_ROW0  = HDR_H + 2;

// Fills rowMap[] with channel index (0-7) or diff index+8 (8=d0, 9=d1); returns count.
static int buildLogRowMap(int* rowMap) {
    int n=0;
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        if(rowMap) rowMap[n]=ch; n++;
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        if(rowMap) rowMap[n]=8+d; n++;
    }
    return n;
}

// ── Numpad popup ──────────────────────────────────────────────────────────────
static const int NP_X     = 160;
static const int NP_Y     = 44;
static const int NP_W     = 480;
static const int NP_BTN_W = 144;
static const int NP_BTN_H = 52;
static const int NP_COL0  = NP_X + 16;
static const int NP_COL1  = NP_COL0 + NP_BTN_W + 8;
static const int NP_COL2  = NP_COL1 + NP_BTN_W + 8;
static const int NP_ROW0  = NP_Y + 92;
static const int NP_ROW1  = NP_ROW0 + 58;
static const int NP_ROW2  = NP_ROW1 + 58;
static const int NP_ROW3  = NP_ROW2 + 58;
static const int NP_ROWOK = NP_ROW3 + 58;

static void drawNumpad() {
    int h = NP_ROWOK + NP_BTN_H + 8 - NP_Y;
    gfx.fillRoundRect(NP_X+4, NP_Y+4, NP_W, h, 8, C_BORDER);
    gfx.fillRoundRect(NP_X,   NP_Y,   NP_W, h, 8, C_PANEL);
    gfx.drawRoundRect(NP_X,   NP_Y,   NP_W, h, 8, C_CYAN);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    gfx.setCursor(NP_X+16, NP_Y+10); gfx.print(numpadTitle);
    gfx.fillRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_BG);
    gfx.drawRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_CYAN);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(3);
    const char* disp = numpadBuf[0] ? numpadBuf : "0";
    int vw = strlen(disp)*18;
    gfx.setCursor(NP_X+16+(NP_W-32-vw)/2, NP_Y+48); gfx.print(disp);
    static const char* NP_LABELS[12] = {"7","8","9","4","5","6","1","2","3",".","0","<-"};
    for(int r=0;r<4;r++){
        int ry = NP_ROW0 + r*58;
        for(int c=0;c<3;c++){
            bool isDec = (r==3 && c==0);
            bool disabled = isDec && (numpadTarget==10 || strchr(numpadBuf,'.')!=nullptr);
            drawBtn(NP_COL0+c*(NP_BTN_W+8), ry, NP_BTN_W, NP_BTN_H,
                    disabled ? C_BORDER : 0x3186u, C_TEXT, NP_LABELS[r*3+c], 3);
        }
    }
    drawBtn(NP_COL0, NP_ROWOK, NP_BTN_W,           NP_BTN_H, C_RED,   C_TEXT, "Cancel", 2);
    drawBtn(NP_COL1, NP_ROWOK, 2*NP_BTN_W+8,       NP_BTN_H, C_GREEN, C_TEXT, "OK",     3);
}

static void openNumpad(int target, const char* title, float curVal) {
    numpadTarget = target;
    strncpy(numpadTitle, title, sizeof(numpadTitle)-1);
    numpadTitle[sizeof(numpadTitle)-1] = '\0';
    if(target == 10)
        snprintf(numpadBuf, sizeof(numpadBuf), "%d", (int)(curVal+0.5f));
    else {
        if(curVal <= 0) numpadBuf[0] = '\0';
        else snprintf(numpadBuf, sizeof(numpadBuf), "%.5g", (double)curVal);
    }
    numpadActive = true;
    drawNumpad();
}

static void handleNumpadTouch(int tx, int ty) {
    if(hit(NP_COL0, NP_ROWOK, NP_BTN_W, NP_BTN_H, tx, ty)){
        numpadActive = false; drawLoggerScreen(); return;
    }
    if(hit(NP_COL1, NP_ROWOK, 2*NP_BTN_W+8, NP_BTN_H, tx, ty)){
        float v = numpadBuf[0] ? (float)atof(numpadBuf) : 0.0f;
        if(numpadTarget == 10)
            logIntervalMs = constrain((int)(v+0.5f), 100, 60000);
        else if(numpadTarget >= 8)
            logDiffThresh[numpadTarget-8] = max(0.0f, v);
        else
            logThresh[numpadTarget] = max(0.0f, v);
        numpadActive = false; drawLoggerScreen(); return;
    }
    static const char NP_KEYS[12] = {'7','8','9','4','5','6','1','2','3','.','0','\x08'};
    for(int r=0;r<4;r++){
        int ry = NP_ROW0 + r*58;
        for(int c=0;c<3;c++){
            if(!hit(NP_COL0+c*(NP_BTN_W+8), ry, NP_BTN_W, NP_BTN_H, tx, ty)) continue;
            char k = NP_KEYS[r*3+c];
            int n = strlen(numpadBuf);
            if(k == '\x08'){
                if(n > 0) numpadBuf[n-1] = '\0';
            } else if(k == '.'){
                if(numpadTarget==10 || strchr(numpadBuf,'.')!=nullptr) return;
                if(n < 11){ numpadBuf[n]='.'; numpadBuf[n+1]='\0'; }
            } else {
                if(n < 11){ numpadBuf[n]=k; numpadBuf[n+1]='\0'; }
            }
            // Refresh input field + decimal button (enabled state may change)
            gfx.fillRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_BG);
            gfx.drawRoundRect(NP_X+16, NP_Y+36, NP_W-32, 44, 4, C_CYAN);
            gfx.setTextColor(C_YELLOW); gfx.setTextSize(3);
            const char* disp = numpadBuf[0] ? numpadBuf : "0";
            int vw = strlen(disp)*18;
            gfx.setCursor(NP_X+16+(NP_W-32-vw)/2, NP_Y+48); gfx.print(disp);
            bool disabled = (numpadTarget==10 || strchr(numpadBuf,'.')!=nullptr);
            drawBtn(NP_COL0, NP_ROW3, NP_BTN_W, NP_BTN_H,
                    disabled ? C_BORDER : 0x3186u, C_TEXT, ".", 3);
            return;
        }
    }
}

static void drawLoggerScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Datalogger Setup");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_BORDER,C_TEXT,"< Back",2);

    int rowMap[10]; int nRows=buildLogRowMap(rowMap);
    for(int row=0;row<nRows;row++){
        int ry=LG_ROW0+row*LG_ROW_H;
        gfx.fillRect(0,ry,DISP_W,LG_ROW_H,(row&1)?C_PANEL:C_BG);
        int idx=rowMap[row]; bool isDiff=(idx>=8); int didx=idx-8;
        uint16_t col = isDiff ? DIFF_COLORS[didx] : CH_COLORS[idx];
        const char* unit = isDiff ? (dDiff[didx].typ==1?"V":"A") : (dChanType[idx]==1?"V":"A");
        float thresh = isDiff ? logDiffThresh[didx] : logThresh[idx];
        char lb[20];
        if(isDiff)
            snprintf(lb,sizeof(lb),"D%d %s-%s(%s)",didx+1,CH_NAMES[dDiff[didx].pos],CH_NAMES[dDiff[didx].neg],dDiff[didx].typ==1?"V":"A");
        else
            snprintf(lb,sizeof(lb),"%s(%s)",CH_NAMES[idx],dChanType[idx]==1?"V":"A");
        gfx.fillRect(4,ry+8,12,20,col);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        gfx.setCursor(20,ry+(LG_ROW_H-16)/2); gfx.print(lb);
        drawBtn(244,ry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"-",3);
        char vb[16];
        if(thresh<=0) snprintf(vb,sizeof(vb),"always"); else snprintf(vb,sizeof(vb),"%.4g",thresh);
        gfx.fillRoundRect(292,ry+3,130,LG_ROW_H-6,3,C_BG);
        gfx.drawRoundRect(292,ry+3,130,LG_ROW_H-6,3,C_BORDER);
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        int vw=strlen(vb)*12; gfx.setCursor(292+(130-vw)/2,ry+(LG_ROW_H-16)/2); gfx.print(vb);
        drawBtn(426,ry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"+",3);
        gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
        gfx.setCursor(476,ry+(LG_ROW_H-8)/2); gfx.print(unit);
    }
    // Interval row
    int iry=LG_ROW0+nRows*LG_ROW_H+4;
    gfx.fillRect(0,iry,DISP_W,LG_ROW_H,(nRows&1)?C_PANEL:C_BG);
    gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
    gfx.setCursor(20,iry+(LG_ROW_H-16)/2); gfx.print("Max interval (ms)");
    drawBtn(244,iry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"-",3);
    char ib[16]; snprintf(ib,sizeof(ib),"%d",logIntervalMs);
    gfx.fillRoundRect(292,iry+3,130,LG_ROW_H-6,3,C_BG);
    gfx.drawRoundRect(292,iry+3,130,LG_ROW_H-6,3,C_BORDER);
    gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
    int iw=strlen(ib)*12; gfx.setCursor(292+(130-iw)/2,iry+(LG_ROW_H-16)/2); gfx.print(ib);
    drawBtn(426,iry+3,44,LG_ROW_H-6,C_BORDER,C_TEXT,"+",3);
    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(476,iry+(LG_ROW_H-8)/2); gfx.print("ms (fallback)");
    // Start button
    char sbuf[52];
    if(nRows==0)
        snprintf(sbuf,sizeof(sbuf),"Select channels on main screen");
    else if(usbMounted){
        int n=nextLogFileNum();
        snprintf(sbuf,sizeof(sbuf),"START -> logger_%03d.csv",n);
    } else
        snprintf(sbuf,sizeof(sbuf),"START (mount USB first)");
    drawBtn(4,DISP_H-52,DISP_W-8,48,(nRows>0&&usbMounted)?C_GREEN:C_BORDER,C_TEXT,sbuf,2);
}

static void updateLoggingDisplay() {
    int rx=DISP_W/2+8;
    char buf[48];
    gfx.fillRect(rx,HDR_H+20,360,24,C_BG);
    gfx.setTextColor(C_GREEN); gfx.setTextSize(2);
    snprintf(buf,sizeof(buf),"%d rows",logRowCount);
    gfx.setCursor(rx,HDR_H+20); gfx.print(buf);
    gfx.fillRect(rx,HDR_H+60,360,24,C_BG);
    uint32_t eSec=(millis()-logStartMs)/1000;
    snprintf(buf,sizeof(buf),"%02lu:%02lu:%02lu",(unsigned long)(eSec/3600),
             (unsigned long)((eSec%3600)/60),(unsigned long)(eSec%60));
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    gfx.setCursor(rx,HDR_H+60); gfx.print(buf);
    int ry=HDR_H+10;
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        gfx.fillRect(160,ry,180,20,C_BG);
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        snprintf(buf,sizeof(buf),"%.5g",(double)logCurCh[ch]);
        gfx.setCursor(160,ry); gfx.print(buf);
        ry+=28;
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        gfx.fillRect(160,ry,180,20,C_BG);
        gfx.setTextColor(C_YELLOW); gfx.setTextSize(2);
        snprintf(buf,sizeof(buf),"%.5g",(double)logCurDiff[d]);
        gfx.setCursor(160,ry); gfx.print(buf);
        ry+=28;
    }
}

static void drawLoggingScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,0x0360u);  // dark green header
    gfx.setTextColor(C_TEXT); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Logging...");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_RED,C_TEXT,"STOP",3);
    gfx.drawFastVLine(DISP_W/2,HDR_H,DISP_H-HDR_H,C_BORDER);
    int rx=DISP_W/2+8;
    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    gfx.setCursor(rx,HDR_H+8);  gfx.print("ROWS WRITTEN");
    gfx.setCursor(rx,HDR_H+48); gfx.print("ELAPSED");
    gfx.setCursor(rx,HDR_H+88); gfx.print("FILE");
    gfx.setTextColor(C_CYAN); gfx.setTextSize(2);
    char fnbuf[32]; snprintf(fnbuf,sizeof(fnbuf),"logger_%03d.csv",logFileNum);
    gfx.setCursor(rx,HDR_H+100); gfx.print(fnbuf);
    gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
    char sbuf[64];
    snprintf(sbuf,sizeof(sbuf),"%lu Hz  %d-bit  smooth:%d",
             (unsigned long)RATE_OPTIONS[dRateIdx],dBits,max(1,dSmooth));
    gfx.setCursor(rx,HDR_H+130); gfx.print(sbuf);
    gfx.setTextColor(C_BORDER); gfx.setTextSize(1);
    snprintf(sbuf,sizeof(sbuf),"Interval fallback: %d ms",logIntervalMs);
    gfx.setCursor(rx,HDR_H+145); gfx.print(sbuf);
    // Channel labels (left column)
    int ry=HDR_H+10;
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        gfx.fillRect(4,ry,12,16,CH_COLORS[ch]);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        char lb[16]; snprintf(lb,sizeof(lb),"%s(%s):",CH_NAMES[ch],dChanType[ch]==1?"V":"A");
        gfx.setCursor(20,ry); gfx.print(lb);
        ry+=28;
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        gfx.fillRect(4,ry,12,16,DIFF_COLORS[d]);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(2);
        char lb[16]; snprintf(lb,sizeof(lb),"D%d(%s):",d+1,dDiff[d].typ==1?"V":"A");
        gfx.setCursor(20,ry); gfx.print(lb);
        ry+=28;
    }
    updateLoggingDisplay();
}

static void startLogger() {
    if(!usbMounted){
        showOverlay(C_YELLOW,"Mounting USB...");
        tryMountUSB();
    }
    if(!usbMounted){ showOverlay(C_RED,"USB not found!"); delay(2000); drawLoggerScreen(); return; }
    logFileNum=nextLogFileNum();
    char fname[32]; snprintf(fname,sizeof(fname),"/usb/logger_%03d.csv",logFileNum);
    logFile=fopen(fname,"w");
    if(!logFile){ showOverlay(C_RED,"File open failed!"); delay(1500); drawLoggerScreen(); return; }
    fprintf(logFile,"time_us");
    for(int ch=0;ch<N_CHANNELS;ch++){
        if(!dChanType[ch]) continue;
        fprintf(logFile,",%s(%s)",CH_NAMES[ch],dChanType[ch]==1?"V":"A");
    }
    for(int d=0;d<2;d++){
        if(!dDiff[d].en) continue;
        fprintf(logFile,",D%d=%s-%s(%s)",d+1,CH_NAMES[dDiff[d].pos],CH_NAMES[dDiff[d].neg],dDiff[d].typ==1?"V":"A");
    }
    fprintf(logFile,"\n");
    logRowCount=0; logStartMs=millis(); logStartUs=micros(); logLastWriteMs=0;
    memset(logSmoothBuf,0,sizeof(logSmoothBuf)); logSmoothTick=0;
    for(int ch=0;ch<N_CHANNELS;ch++){logLastCh[ch]=1e30f;logCurCh[ch]=0;}
    for(int d=0;d<2;d++){logLastDiff[d]=1e30f;logCurDiff[d]=0;}
    logLastDispMs=0;
    uint32_t sr=RATE_OPTIONS[dRateIdx];
    int ns=(int)((sr/1000UL)*(uint32_t)max(dTime/4,1));
    if(ns<16) ns=16; if(ns>N_SAMPLES_MAX) ns=N_SAMPLES_MAX;
    if(adcRunning) adc.stop();
    if(!adc.begin(toResolution(dBits),sr,ns,QUEUE_DEPTH)){
        fclose(logFile); logFile=nullptr;
        showOverlay(C_RED,"ADC start failed!"); delay(2000);
        drawLoggerScreen(); return;
    }
    adcRunning=true; logRunning=true;
    currentScreen=SCR_LOGGING; drawLoggingScreen();
}

static void stopLogger() {
    logRunning=false;
    if(adcRunning){ adc.stop(); adcRunning=false; }
    if(logFile){ fflush(logFile); fclose(logFile); logFile=nullptr; }
    usbFs.unmount(); usbMounted=false;
    currentScreen=SCR_SETTINGS; drawSettingsScreen();
}

static void loggerStep() {
    if(!logRunning||!adcRunning||!adc.available()) return;
    SampleBuffer buf=adc.read();
    int ns=buf.size()/N_CHANNELS;
    int sc=max(1,dSmooth);
    for(int s=0;s<ns;s++){
        for(int ch=0;ch<N_CHANNELS;ch++) logSmoothBuf[ch]+=buf[s*N_CHANNELS+ch];
        if(++logSmoothTick>=sc){
            logSmoothTick=0;
            int sv[N_CHANNELS];
            for(int ch=0;ch<N_CHANNELS;ch++){sv[ch]=logSmoothBuf[ch]/sc;logSmoothBuf[ch]=0;}
            float ev[N_CHANNELS];
            for(int ch=0;ch<N_CHANNELS;ch++)
                ev[ch]=dChanType[ch]?calChan(sv[ch],dChanOffset[ch],dChanScale[ch],dBits):0.0f;
            float ed[2];
            for(int d=0;d<2;d++)
                ed[d]=dDiff[d].en?calDiff(sv[dDiff[d].pos],sv[dDiff[d].neg],dDiff[d].off,dDiff[d].scl,dBits):0.0f;
            for(int ch=0;ch<N_CHANNELS;ch++) if(dChanType[ch]) logCurCh[ch]=ev[ch];
            for(int d=0;d<2;d++) if(dDiff[d].en) logCurDiff[d]=ed[d];
            uint32_t now=millis();
            bool doLog=(now-logLastWriteMs>=(uint32_t)logIntervalMs);
            if(!doLog){
                for(int ch=0;ch<N_CHANNELS&&!doLog;ch++){
                    if(!dChanType[ch]) continue;
                    if(logThresh[ch]<=0||fabsf(ev[ch]-logLastCh[ch])>=logThresh[ch]) doLog=true;
                }
            }
            if(!doLog){
                for(int d=0;d<2&&!doLog;d++){
                    if(!dDiff[d].en) continue;
                    if(logDiffThresh[d]<=0||fabsf(ed[d]-logLastDiff[d])>=logDiffThresh[d]) doLog=true;
                }
            }
            if(doLog&&logFile){
                fprintf(logFile,"%lu",(unsigned long)(micros()-logStartUs));
                for(int ch=0;ch<N_CHANNELS;ch++){
                    if(!dChanType[ch]) continue;
                    fprintf(logFile,",%.5g",(double)ev[ch]);
                    logLastCh[ch]=ev[ch];
                }
                for(int d=0;d<2;d++){
                    if(!dDiff[d].en) continue;
                    fprintf(logFile,",%.5g",(double)ed[d]);
                    logLastDiff[d]=ed[d];
                }
                fprintf(logFile,"\n");
                logRowCount++;
                logLastWriteMs=now;
                if(logRowCount%50==0) fflush(logFile);
            }
        }
    }
    buf.release();
    uint32_t now2=millis();
    if(now2-logLastDispMs>=500){logLastDispMs=now2;updateLoggingDisplay();}
}

// ── USB export ────────────────────────────────────────────────────────────────
static void tryMountUSB() {
    pinMode(PA_15,OUTPUT); digitalWrite(PA_15,HIGH); delay(300);
    usbMounted=false;
    if(msd.connect() && usbFs.mount(&msd)==0) usbMounted=true;
}
static void showOverlay(uint16_t col, const char* msg) {
    gfx.fillRect(20,DISP_H/2-20,DISP_W-40,40,C_PANEL);
    gfx.setTextColor(col); gfx.setTextSize(2);
    gfx.setCursor(24,DISP_H/2-8); gfx.print(msg);
}
static void exportToUSB() {
    showOverlay(C_YELLOW,"Mounting USB...");
    tryMountUSB();
    if(!usbMounted){ showOverlay(C_RED,"USB not found!"); delay(1500); drawPlotScreen(); return; }
    FILE* f=fopen("/usb/capture.csv","w");
    if(!f){ showOverlay(C_RED,"File open failed!"); delay(1500); drawPlotScreen(); return; }

    // Header
    fprintf(f,"Time (ms)");
    for(int ch=0;ch<N_CHANNELS;ch++)
        if(lastPinReq[ch]&&lastChanType[ch]!=0)
            fprintf(f,",%s (%s)",CH_NAMES[ch],lastChanType[ch]==1?"V":"I");
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(dc.en) fprintf(f,",D%d=%s-%s (%s)",d+1,CH_NAMES[dc.pos],CH_NAMES[dc.neg],dc.typ==1?"V":"I");
    }
    fprintf(f,"\n");

    int maxCnt=1;
    for(int ch=0;ch<N_CHANNELS;ch++) if(lastPinReq[ch]&&lastLogCnt[ch]>maxCnt) maxCnt=lastLogCnt[ch];

    for(int j=0;j<maxCnt;j++) {
        double t_ms=(maxCnt>1)?(double)j*lastElapsedMs/(maxCnt-1):0.0;
        fprintf(f,"%.5g",t_ms);
        for(int ch=0;ch<N_CHANNELS;ch++) {
            if(!lastPinReq[ch]||lastChanType[ch]==0) continue;
            if(j<lastLogCnt[ch]) {
                float v=calChan(getRawVal(ch,j),lastChanOffset[ch],lastChanScale[ch],lastBitRes);
                fprintf(f,",%.5g",(double)v);
            } else fprintf(f,",");
        }
        for(int d=0;d<2;d++) {
            const DiffCfg& dc=lastDiff[d];
            if(!dc.en) continue;
            int cnt=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]);
            if(j<cnt) {
                float v=calDiff(getRawVal(dc.pos,j),getRawVal(dc.neg,j),dc.off,dc.scl,lastBitRes);
                fprintf(f,",%.5g",(double)v);
            } else fprintf(f,",");
        }
        fprintf(f,"\n");
    }
    fclose(f); usbFs.unmount(); usbMounted=false;
    showOverlay(C_GREEN,"Saved: /usb/capture.csv");
    delay(2000); drawPlotScreen();
}

// ── Display-mode capture ──────────────────────────────────────────────────────
static void runDisplayCapture() {
    if(!contMode) drawCaptureScreen();

    uint32_t sampleRate = RATE_OPTIONS[dRateIdx];
    int bitRes=dBits, duration=dTime, smoothCount=max(1,dSmooth);
    int logTarget=min(dPoints,MAX_LOG), trigThr=dTrigThr;

    // Build capture mask — standalone active + diff source pins
    bool doCapt[N_CHANNELS]={};
    for(int i=0;i<N_CHANNELS;i++) if(dChanType[i]) doCapt[i]=true;
    for(int d=0;d<2;d++) {
        if(dDiff[d].en){ doCapt[dDiff[d].pos]=true; doCapt[dDiff[d].neg]=true; }
    }

    uint32_t cnt_[N_CHANNELS]={};
    double   sum_[N_CHANNELS]={}, sq_[N_CHANNELS]={};
    int      mn_[N_CHANNELS], mx_[N_CHANNELS];
    for(int i=0;i<N_CHANNELS;i++){mn_[i]=65535;mx_[i]=0;}
    int captLogCnt[N_CHANNELS]={};

    long smoothBuf[N_CHANNELS]={};
    int  smoothTick=0;
    int  lastLoggedVal[N_CHANNELS];
    for(int i=0;i<N_CHANNELS;i++) lastLoggedVal[i]=-32768;

    uint32_t totalSmoothed=(sampleRate*(uint32_t)duration/1000)/(uint32_t)smoothCount;
    uint32_t logEvery=(totalSmoothed>(uint32_t)logTarget)?totalSmoothed/logTarget:1;
    uint32_t sampleTick=0, lastLogMs=0;

    int nSamples=(int)((sampleRate/1000UL)*(uint32_t)max(duration/4,1));
    if(nSamples<16)            nSamples=16;
    if(nSamples>N_SAMPLES_MAX) nSamples=N_SAMPLES_MAX;

    if(adcRunning) adc.stop();
    if(!adc.begin(toResolution(bitRes),sampleRate,nSamples,QUEUE_DEPTH)) {
        showOverlay(C_RED,"ADC start failed!"); delay(2000);
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    adcRunning=true;

    uint32_t startMs=millis(), lastDisp=0;
    while(millis()-startMs<(uint32_t)duration) {
        uint32_t now=millis();
        if(!contMode && now-lastDisp>80){ lastDisp=now; updateProgress(now-startMs,duration); }
        if(!adc.available()) continue;
        SampleBuffer buf=adc.read();
        int ns=buf.size()/N_CHANNELS;
        for(int s=0;s<ns;s++) {
            for(int ch=0;ch<N_CHANNELS;ch++) smoothBuf[ch]+=buf[s*N_CHANNELS+ch];
            if(++smoothTick>=smoothCount) {
                smoothTick=0;
                int sv[N_CHANNELS];
                for(int ch=0;ch<N_CHANNELS;ch++) {
                    sv[ch]=smoothBuf[ch]/smoothCount; smoothBuf[ch]=0;
                    if(!doCapt[ch]) continue;
                    if(sv[ch]<mn_[ch]) mn_[ch]=sv[ch];
                    if(sv[ch]>mx_[ch]) mx_[ch]=sv[ch];
                    if(dChanType[ch]){ sum_[ch]+=sv[ch]; sq_[ch]+=(double)sv[ch]*sv[ch]; cnt_[ch]++; }
                }
                sampleTick++;

                bool doLog;
                if(trigThr>0) {
                    doLog=false;
                    for(int ch=0;ch<N_CHANNELS;ch++)
                        if(dChanType[ch]&&abs(sv[ch]-lastLoggedVal[ch])>=trigThr){doLog=true;break;}
                } else {
                    doLog=(sampleTick%logEvery==0);
                }
                if(doLog) {
                    bool anyStored=false;
                    for(int ch=0;ch<N_CHANNELS;ch++) {
                        if(!doCapt[ch]||captLogCnt[ch]>=logTarget) continue;
                        logData[ch][captLogCnt[ch]++]=sv[ch];
                        if(dChanType[ch]) lastLoggedVal[ch]=sv[ch];
                        anyStored=true;
                    }
                    if(anyStored) lastLogMs=millis()-startMs;
                }
            }
        }
        buf.release();
    }
    adc.stop(); adcRunning=false;

    // Save post-capture state
    for(int i=0;i<N_CHANNELS;i++) {
        lastPinReq[i]    = doCapt[i];
        lastLogCnt[i]    = captLogCnt[i];
        lastChanType[i]  = dChanType[i];
        lastChanOffset[i]= dChanOffset[i];
        lastChanScale[i] = dChanScale[i];
    }
    memcpy(lastDiff,dDiff,sizeof(dDiff));
    lastBitRes    = bitRes;
    // Time span from the sample math (fixed ADC rate) rather than wall clock,
    // which is inflated by ADC start-up latency / loop jitter and would skew
    // the frequency read off the plot. Trigger mode logs on change (non-uniform)
    // so it must fall back to wall clock.
    if(trigThr>0){
        lastElapsedMs = lastLogMs>0 ? lastLogMs : millis()-startMs;
    } else {
        int nStored=0;
        for(int i=0;i<N_CHANNELS;i++) if(doCapt[i]&&captLogCnt[i]>nStored) nStored=captLogCnt[i];
        lastElapsedMs = (uint32_t)((uint64_t)nStored*logEvery*smoothCount*1000/sampleRate);
        if(lastElapsedMs==0) lastElapsedMs = lastLogMs>0 ? lastLogMs : millis()-startMs;
    }
    lastMidMode   = false;
    captureReady  = true;

    // Serial mirror (raw counts, Python GUI compatible)
    for(int i=0;i<N_CHANNELS;i++) {
        if(!dChanType[i]||cnt_[i]==0) continue;
        double mean=sum_[i]/cnt_[i], rms=sqrt(sq_[i]/cnt_[i]);
        double stdDev=sqrt(fabs(sq_[i]/cnt_[i]-mean*mean));
        Serial.print("PIN:");    Serial.print(CH_NAMES[i]);
        Serial.print("|BITS:");  Serial.print(bitRes);
        Serial.print("|COUNT:"); Serial.print(cnt_[i]);
        Serial.print("|MIN:");   Serial.print(mn_[i]);
        Serial.print("|MAX:");   Serial.print(mx_[i]);
        Serial.print("|MEAN:");  Serial.print(mean,4);
        Serial.print("|RMS:");   Serial.print(rms,4);
        Serial.print("|STD:");   Serial.print(stdDev,4);
        Serial.print("|LOGGED:");Serial.println(captLogCnt[i]);
    }
    for(int i=0;i<N_CHANNELS;i++) {
        if(!dChanType[i]||captLogCnt[i]==0) continue;
        Serial.print("DATA:"); Serial.print(CH_NAMES[i]); Serial.print("|VALS:");
        for(int j=0;j<captLogCnt[i];j++) {
            Serial.print(logData[i][j]);
            if(j<captLogCnt[i]-1) Serial.print(',');
        }
        Serial.println();
    }
    Serial.print("DONE|ELAPSED:"); Serial.println(lastElapsedMs);

    tryMountUSB();
    currentScreen=SCR_PLOT;
    drawPlotScreen();
}

// ── Touch handlers ────────────────────────────────────────────────────────────
static void handleSettingsTouch(int tx,int ty) {
    // Presets / Logger buttons
    if(hit(DISP_W-164,2,78,HDR_H-4,tx,ty)){
        currentScreen=SCR_PRESETS; drawPresetsScreen(); return;
    }
    if(hit(DISP_W-82,2,78,HDR_H-4,tx,ty)){
        contMode=false;
        currentScreen=SCR_LOGGER; drawLoggerScreen(); return;
    }
    // Channel type cycle (off→V→I→off)
    int cbw=DISP_W/N_CHANNELS;
    for(int i=0;i<N_CHANNELS;i++) {
        if(hit(i*cbw+2,CH_ROW_Y,cbw-4,CH_ROW_H-4,tx,ty)) {
            dChanType[i]=(dChanType[i]+1)%3;
            char lbl[8];
            if(dChanType[i]==0)      snprintf(lbl,sizeof(lbl),"%s",CH_NAMES[i]);
            else if(dChanType[i]==1) snprintf(lbl,sizeof(lbl),"%s V",CH_NAMES[i]);
            else                     snprintf(lbl,sizeof(lbl),"%s I",CH_NAMES[i]);
            drawBtn(i*cbw+2,CH_ROW_Y,cbw-4,CH_ROW_H-4,
                    dChanType[i]?CH_COLORS[i]:C_PANEL, C_TEXT, lbl, 2);
            drawCaptureButtons(); return;
        }
    }
    // Param +/-
    for(int r=0;r<N_PARAMS;r++) {
        int ry=PARAM_Y+r*PARAM_ROW, bx=PL_X0+PL_LABEL_W;
        bool changed=false;
        if(hit(bx,ry+2,PL_BTN_W,PARAM_ROW-4,tx,ty)) {
            switch(r){
                case 0:if(dRateIdx>0)dRateIdx--;break;
                case 1:if(dBits>8)dBits-=2;break;
                case 2:dTime=max(1,dTime-(dTime>1000?100:dTime>100?10:1));break;
                case 3:if(dSmooth>1)dSmooth--;break;
                case 4:dPoints=max(10,dPoints-(dPoints>1000?500:dPoints>100?100:10));break;
                case 5:dTrigThr=max(0,dTrigThr-(dTrigThr>100?100:dTrigThr>10?10:1));break;
            }
            changed=true;
        } else if(hit(bx+PL_BTN_W+PL_VAL_W,ry+2,PL_BTN_W,PARAM_ROW-4,tx,ty)) {
            switch(r){
                case 0:if(dRateIdx<N_RATE_OPT-1)dRateIdx++;break;
                case 1:if(dBits<16)dBits+=2;break;
                case 2:dTime=min(60000,dTime+(dTime>=1000?100:dTime>=100?10:1));break;
                case 3:if(dSmooth<64)dSmooth++;break;
                case 4:dPoints=min(MAX_LOG,dPoints+(dPoints>=1000?500:dPoints>=100?100:10));break;
                case 5:dTrigThr=min(4095,dTrigThr+(dTrigThr>=100?100:dTrigThr>=10?10:1));break;
            }
            changed=true;
        }
        if(changed){refreshParamValue(r);return;}
    }
    // Diff button
    if(hit(4,DIFF_BTN_Y,DISP_W-8,DIFF_BTN_H,tx,ty)){
        currentScreen=SCR_DIFF; drawDiffScreen(); return;
    }
    // Capture / Continuous buttons
    bool anyActive=false;
    for(int i=0;i<N_CHANNELS;i++) if(dChanType[i]) anyActive=true;
    if(dDiff[0].en||dDiff[1].en) anyActive=true;
    if(anyActive && hit(4,CAP_BTN_Y,CAP_W,CAP_BTN_H,tx,ty)){
        contMode=false;
        currentScreen=SCR_CAPTURING; runDisplayCapture();
    }
    if(anyActive && hit(CONT_X,CAP_BTN_Y,CONT_W,CAP_BTN_H,tx,ty)){
        contMode=true; lastContMs=0;
        runDisplayCapture();
    }
}

static void handlePresetsTouch(int tx,int ty) {
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)){
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    int n = min(N_PRESETS, PRE_COLS * PRE_ROWS);
    for(int i=0;i<n;i++) {
        int bx = (i % PRE_COLS) * PRE_CW + 2;
        int by = HDR_H + (i / PRE_COLS) * PRE_RH + 1;
        if(hit(bx, by, PRE_BTN_W, PRE_BTN_H, tx, ty)){
            applyPreset(i);
            currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
        }
    }
}

static void handleDiffTouch(int tx,int ty) {
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)){
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    for(int d=0;d<2;d++) {
        int ry=60+d*110; DiffCfg& dc=dDiff[d];
        if(hit(10,ry,160,50,tx,ty)){ dc.en=!dc.en; drawDiffRow(d); return; }
        if(hit(228,ry,44,50,tx,ty)){ dc.pos=(dc.pos+N_CHANNELS-1)%N_CHANNELS; drawDiffRow(d); return; }
        if(hit(322,ry,44,50,tx,ty)){ dc.pos=(dc.pos+1)%N_CHANNELS;            drawDiffRow(d); return; }
        if(hit(428,ry,44,50,tx,ty)){ dc.neg=(dc.neg+N_CHANNELS-1)%N_CHANNELS; drawDiffRow(d); return; }
        if(hit(522,ry,44,50,tx,ty)){ dc.neg=(dc.neg+1)%N_CHANNELS;            drawDiffRow(d); return; }
        if(hit(588,ry,88,50,tx,ty)){ dc.typ=1; drawDiffRow(d); return; }
        if(hit(684,ry,88,50,tx,ty)){ dc.typ=2; drawDiffRow(d); return; }
    }
}

static void handlePlotTouch(int tx,int ty) {
    if(hit(DISP_W-220,2,100,HDR_H-4,tx,ty)){
        contMode=false; currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)) exportToUSB();
}

static void handleLoggerTouch(int tx,int ty) {
    if(numpadActive){ handleNumpadTouch(tx,ty); return; }
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)){
        currentScreen=SCR_SETTINGS; drawSettingsScreen(); return;
    }
    int rowMap[10]; int nRows=buildLogRowMap(rowMap);
    for(int row=0;row<nRows;row++){
        int ry=LG_ROW0+row*LG_ROW_H;
        int idx=rowMap[row]; bool isDiff=(idx>=8); int didx=idx-8;
        if(hit(244,ry+3,44,LG_ROW_H-6,tx,ty)){
            if(isDiff) logDiffThresh[didx]=adjThreshDn(logDiffThresh[didx]);
            else       logThresh[idx]=adjThreshDn(logThresh[idx]);
            drawLoggerScreen(); return;
        }
        if(hit(426,ry+3,44,LG_ROW_H-6,tx,ty)){
            if(isDiff) logDiffThresh[didx]=adjThreshUp(logDiffThresh[didx]);
            else       logThresh[idx]=adjThreshUp(logThresh[idx]);
            drawLoggerScreen(); return;
        }
        if(hit(292,ry,130,LG_ROW_H,tx,ty)){
            char title[24];
            if(isDiff) snprintf(title,sizeof(title),"D%d delta thresh",didx+1);
            else       snprintf(title,sizeof(title),"%s delta thresh",CH_NAMES[idx]);
            openNumpad(isDiff?8+didx:idx, title, isDiff?logDiffThresh[didx]:logThresh[idx]);
            return;
        }
    }
    int iry=LG_ROW0+nRows*LG_ROW_H+4;
    if(hit(244,iry+3,44,LG_ROW_H-6,tx,ty)){
        logIntervalMs=max(100,logIntervalMs-(logIntervalMs>5000?1000:logIntervalMs>1000?500:100));
        drawLoggerScreen(); return;
    }
    if(hit(426,iry+3,44,LG_ROW_H-6,tx,ty)){
        logIntervalMs=min(60000,logIntervalMs+(logIntervalMs>=5000?1000:logIntervalMs>=1000?500:100));
        drawLoggerScreen(); return;
    }
    if(hit(292,iry,130,LG_ROW_H,tx,ty)){
        openNumpad(10,"Max interval (ms)",(float)logIntervalMs); return;
    }
    if(hit(4,DISP_H-52,DISP_W-8,48,tx,ty)){
        if(nRows>0) startLogger();
    }
}

static void handleLoggingTouch(int tx,int ty) {
    if(hit(DISP_W-114,2,110,HDR_H-4,tx,ty)) stopLogger();
}

// ── WiFi bring-up ─────────────────────────────────────────────────────────────
static void processCommand(Stream& io);   // fwd decl (Stream& param)

static void connectWiFi() {
    strcpy(wifiIPStr, "connecting...");
    if(WiFi.status() == WL_NO_MODULE){ strcpy(wifiIPStr,"no module"); return; }
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while(WiFi.status() != WL_CONNECTED && millis()-t0 < 15000) delay(300);
    if(WiFi.status() == WL_CONNECTED){
        IPAddress ip = WiFi.localIP();
        snprintf(wifiIPStr, sizeof(wifiIPStr), "%d.%d.%d.%d:%u",
                 ip[0], ip[1], ip[2], ip[3], (unsigned)CMD_PORT);
        cmdServer.begin();
        wifiActive = true;
        Serial.print("WiFi ready at "); Serial.println(wifiIPStr);
    } else {
        strcpy(wifiIPStr, "wifi failed");
        Serial.println("WiFi connect failed — USB serial still works.");
    }
}

// ── setup / loop ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    displayPresent=touch.begin();
    if(displayPresent){ gfx.begin(); gfx.setRotation(1); drawSettingsScreen(); }
    connectWiFi();
    if(displayPresent) drawSettingsScreen();   // redraw so IP shows
}

void loop() {
    if(logRunning) loggerStep();
    if(displayPresent) {
        // ── Continuous mode: fire next capture when interval has elapsed ──
        if(contMode && currentScreen==SCR_PLOT) {
            uint32_t waitMs = (uint32_t)dTime < 1000u ? 1000u-(uint32_t)dTime : 0u;
            if(millis()-lastContMs >= waitMs) {
                lastContMs=millis();
                runDisplayCapture();
                return;
            }
        }
        // ── Touch handling ────────────────────────────────────────────────
        GDTpoint_t pts[5];
        uint8_t n=touch.getTouchPoints(pts);
        if(n>0) {
            uint32_t now=millis();
            if(now-lastTouchMs>TOUCH_DEBOUNCE_MS) {
                lastTouchMs=now;
                int tx=pts[0].y, ty=DISP_H-1-pts[0].x;
                switch(currentScreen) {
                    case SCR_SETTINGS: handleSettingsTouch(tx,ty); break;
                    case SCR_PRESETS:  handlePresetsTouch(tx,ty);  break;
                    case SCR_DIFF:     handleDiffTouch(tx,ty);     break;
                    case SCR_PLOT:     handlePlotTouch(tx,ty);     break;
                    case SCR_LOGGER:   handleLoggerTouch(tx,ty);   break;
                    case SCR_LOGGING:  handleLoggingTouch(tx,ty);  break;
                    default: break;
                }
            }
        }
    }

    // ── Command channel: USB serial OR WiFi TCP (both are Streams) ───────────
    // USB serial takes priority when bytes are waiting.
    if(Serial.available()){ processCommand(Serial); return; }

    // WiFi: the GUI opens a fresh TCP connection per capture. Handle exactly one
    // command per connection, then release the socket — otherwise the Giga's
    // small socket pool leaks and the board stops responding until reset.
    if(wifiActive){
        WiFiClient c = cmdServer.available();
        if(c){
            // Wait briefly for the full command line to arrive.
            uint32_t t0 = millis();
            while(c.connected() && !c.available() && millis()-t0 < 1500) delay(1);
            if(c.available()) processCommand(c);
            // Let the GUI finish reading the whole reply (incl. large DATA dumps)
            // and close first, so stop() can't truncate the response. Capped so a
            // stalled client can never wedge the board.
            c.flush();
            uint32_t t1 = millis();
            while(c.connected() && millis()-t1 < 8000) delay(2);
            c.stop();
        }
    }
}

// ── Command processing: shared by USB serial and WiFi (both are Streams) ──────
static void processCommand(Stream& io) {
    // Read + validate the command line BEFORE any side effects. A partial or
    // stray read (common right after a socket opens/closes) must never trigger a
    // capture with the wrong parameters — that was the source of mislabeled
    // elapsed times over WiFi.
    String cmd=io.readStringUntil('\n'); cmd.trim();
    if(!cmd.startsWith("PINS:")){
        if(cmd.length()) io.println("ERR:BAD_CMD");
        return;
    }
    contMode=false;  // any incoming command stops continuous view
    if(logRunning){
        logRunning=false;
        if(logFile){ fflush(logFile); fclose(logFile); logFile=nullptr; }
        usbFs.unmount(); usbMounted=false;
    }
    if(displayPresent){ currentScreen=SCR_CAPTURING; drawCaptureScreen(); }

    int bitRes      =cmd.substring(cmd.indexOf("BITS:")+5,cmd.indexOf("|TIME:")).toInt();
    int duration    =cmd.substring(cmd.indexOf("TIME:")+5,cmd.indexOf("|SMOOTH:")).toInt();
    int smoothCount =cmd.substring(cmd.indexOf("SMOOTH:")+7,cmd.indexOf("|LOG:")).toInt();
    int logTarget   =cmd.substring(cmd.indexOf("LOG:")+4).toInt();
    int ri          =cmd.indexOf("|RATE:");
    uint32_t sampleRate=(ri>=0)?(uint32_t)cmd.substring(ri+6).toInt():500000;

    int trigIdx=cmd.indexOf("|TRIG:");
    int trigThresh[N_CHANNELS]={};
    if(trigIdx>=0){
        String ts=cmd.substring(trigIdx+6);
        for(int i=0;i<N_CHANNELS;i++){
            trigThresh[i]=ts.toInt();
            int c=ts.indexOf(','); if(c<0) break;
            ts=ts.substring(c+1);
        }
    }
    int midIdx=cmd.indexOf("|MID:");
    bool midpointMode=(midIdx>=0&&cmd.substring(midIdx+5).toInt()==1);

    if(bitRes<8)         bitRes=12;
    if(duration<1)       duration=1000;
    if(smoothCount<1)    smoothCount=1;
    if(logTarget<1)      logTarget=1;
    if(logTarget>MAX_LOG)logTarget=MAX_LOG;
    if(sampleRate<1000)  sampleRate=500000;

    bool pinReq[N_CHANNELS]={};
    String pinsStr=cmd.substring(5,cmd.indexOf("|BITS:"));
    int ci=0;
    while(ci>=0){
        int nc=pinsStr.indexOf(',',ci);
        String nm=(nc<0)?pinsStr.substring(ci):pinsStr.substring(ci,nc); nm.trim();
        for(int i=0;i<N_CHANNELS;i++) if(nm.equals(CH_NAMES[i])){pinReq[i]=true;break;}
        if(nc<0) break; ci=nc+1;
    }

    uint32_t cnt[N_CHANNELS]={};
    double   sum[N_CHANNELS]={},sq[N_CHANNELS]={};
    int      mn[N_CHANNELS],mx[N_CHANNELS];
    for(int i=0;i<N_CHANNELS;i++){mn[i]=65535;mx[i]=0;}
    int logCnt[N_CHANNELS]={};
    long smoothBuf[N_CHANNELS]={};
    int  smoothTick=0;
    int  lastLoggedVal[N_CHANNELS];
    for(int i=0;i<N_CHANNELS;i++) lastLoggedVal[i]=-32768;
    bool usingTrig=false;
    for(int i=0;i<N_CHANNELS;i++) if(pinReq[i]&&trigThresh[i]>0){usingTrig=true;break;}

    uint32_t totalSmoothed=(sampleRate*(uint32_t)duration/1000)/(uint32_t)smoothCount;
    uint32_t logEvery=(totalSmoothed>(uint32_t)logTarget)?totalSmoothed/logTarget:1;
    uint32_t sampleTick=0,lastLogMs=0;

    int  midPreSize=logTarget/2,midPostSize=logTarget-midPreSize;
    int  midRingHead=0,midRingFill=0,midPostCount=0;
    bool midTrigFired=false,midDone=false;
    int  midPrevVal[N_CHANNELS];
    for(int i=0;i<N_CHANNELS;i++) midPrevVal[i]=-32768;

    int nSamples=(int)((sampleRate/1000UL)*(uint32_t)max(duration/4,1));
    if(nSamples<16)            nSamples=16;
    if(nSamples>N_SAMPLES_MAX) nSamples=N_SAMPLES_MAX;

    if(adcRunning) adc.stop();
    if(!adc.begin(toResolution(bitRes),sampleRate,nSamples,QUEUE_DEPTH)){
        io.println("ERR:ADC_BEGIN_FAILED"); return;
    }
    adcRunning=true;

    uint32_t startMs=millis();
    while(!midDone&&(midpointMode||millis()-startMs<(uint32_t)duration)){
        if(midpointMode&&io.available()){
            String inc=io.readStringUntil('\n'); inc.trim();
            if(inc==F("!ABORT")){midDone=true;break;}
        }
        if(!adc.available()) continue;
        SampleBuffer buf=adc.read();
        int ns=buf.size()/N_CHANNELS;
        for(int s=0;s<ns;s++){
            for(int ch=0;ch<N_CHANNELS;ch++) smoothBuf[ch]+=buf[s*N_CHANNELS+ch];
            if(++smoothTick>=smoothCount){
                smoothTick=0;
                int sv[N_CHANNELS];
                for(int ch=0;ch<N_CHANNELS;ch++){
                    sv[ch]=smoothBuf[ch]/smoothCount; smoothBuf[ch]=0;
                    if(!pinReq[ch]) continue;
                    if(sv[ch]<mn[ch]) mn[ch]=sv[ch];
                    if(sv[ch]>mx[ch]) mx[ch]=sv[ch];
                    sum[ch]+=sv[ch]; sq[ch]+=(double)sv[ch]*sv[ch]; cnt[ch]++;
                }
                sampleTick++;
                if(midpointMode){
                    if(!midTrigFired){
                        for(int ch=0;ch<N_CHANNELS;ch++){if(!pinReq[ch])continue;logData[ch][midRingHead]=sv[ch];}
                        if(midRingFill<midPreSize) midRingFill++;
                        midRingHead=(midRingHead+1)%midPreSize;
                        for(int ch=0;ch<N_CHANNELS;ch++)
                            if(pinReq[ch]&&trigThresh[ch]>0&&midPrevVal[ch]!=-32768&&
                               abs(sv[ch]-midPrevVal[ch])>=trigThresh[ch]){midTrigFired=true;break;}
                    } else if(midPostCount<midPostSize){
                        for(int ch=0;ch<N_CHANNELS;ch++){if(!pinReq[ch])continue;logData[ch][midPreSize+midPostCount]=sv[ch];}
                        if(++midPostCount>=midPostSize) midDone=true;
                    }
                    for(int ch=0;ch<N_CHANNELS;ch++) if(pinReq[ch]) midPrevVal[ch]=sv[ch];
                } else {
                    bool doLog;
                    if(usingTrig){
                        doLog=false;
                        for(int ch=0;ch<N_CHANNELS;ch++)
                            if(pinReq[ch]&&trigThresh[ch]>0&&abs(sv[ch]-lastLoggedVal[ch])>=trigThresh[ch]){doLog=true;break;}
                    } else doLog=(sampleTick%logEvery==0);
                    if(doLog){
                        bool anyStored=false;
                        for(int ch=0;ch<N_CHANNELS;ch++){
                            if(!pinReq[ch]||logCnt[ch]>=logTarget) continue;
                            logData[ch][logCnt[ch]++]=sv[ch]; lastLoggedVal[ch]=sv[ch]; anyStored=true;
                        }
                        if(anyStored) lastLogMs=millis()-startMs;
                    }
                }
            }
        }
        buf.release();
    }
    adc.stop(); adcRunning=false;
    if(midpointMode) for(int i=0;i<N_CHANNELS;i++) if(pinReq[i]) logCnt[i]=midRingFill+midPostCount;

    for(int i=0;i<N_CHANNELS;i++){
        if(!pinReq[i]||cnt[i]==0) continue;
        double mean=sum[i]/cnt[i],rms=sqrt(sq[i]/cnt[i]),stdDev=sqrt(fabs(sq[i]/cnt[i]-mean*mean));
        io.print("PIN:");io.print(CH_NAMES[i]);io.print("|BITS:");io.print(bitRes);
        io.print("|COUNT:");io.print(cnt[i]);io.print("|MIN:");io.print(mn[i]);
        io.print("|MAX:");io.print(mx[i]);io.print("|MEAN:");io.print(mean,4);
        io.print("|RMS:");io.print(rms,4);io.print("|STD:");io.print(stdDev,4);
        io.print("|LOGGED:");io.println(logCnt[i]);
    }
    for(int i=0;i<N_CHANNELS;i++){
        if(!pinReq[i]||logCnt[i]==0) continue;
        io.print("DATA:");io.print(CH_NAMES[i]);io.print("|VALS:");
        if(midpointMode){
            int si=(midRingFill<midPreSize)?0:midRingHead;
            for(int j=0;j<midRingFill;j++){
                io.print(logData[i][(si+j)%midPreSize]);
                if(j<midRingFill-1||midPostCount>0) io.print(',');
            }
            for(int j=0;j<midPostCount;j++){
                io.print(logData[i][midPreSize+j]);
                if(j<midPostCount-1) io.print(',');
            }
        } else {
            for(int j=0;j<logCnt[i];j++){io.print(logData[i][j]);if(j<logCnt[i]-1)io.print(',');}
        }
        io.println();
    }
    io.print("DONE|ELAPSED:");
    uint32_t serialElapsed;
    if(midpointMode){
        serialElapsed=(uint32_t)((uint64_t)(midRingFill+midPostCount)*smoothCount*1000/sampleRate);
        if(serialElapsed==0) serialElapsed=1;
    } else if(usingTrig){
        // Trigger mode logs on change -> non-uniform spacing; wall clock is the
        // only meaningful span.
        serialElapsed=lastLogMs>0?lastLogMs:(millis()-startMs);
    } else {
        // Uniform decimation: derive the true signal span from the sample math.
        // Immune to ADC start-up latency and WiFi-induced wall-clock jitter,
        // which otherwise inflate elapsed time and skew the computed frequency.
        int nStored=0;
        for(int i=0;i<N_CHANNELS;i++) if(pinReq[i]&&logCnt[i]>nStored) nStored=logCnt[i];
        serialElapsed=(uint32_t)((uint64_t)nStored*logEvery*smoothCount*1000/sampleRate);
        if(serialElapsed==0) serialElapsed=lastLogMs>0?lastLogMs:(millis()-startMs);
    }
    io.println(serialElapsed);

    // ── If display present, show the serial-triggered capture on screen ───────
    if(displayPresent) {
        for(int i=0;i<N_CHANNELS;i++) {
            lastPinReq[i]     = pinReq[i];
            lastLogCnt[i]     = logCnt[i];
            lastChanType[i]   = pinReq[i] ? 1 : 0;  // raw/uncalibrated
            lastChanOffset[i] = 0.0f;
            lastChanScale[i]  = 1.0f;
        }
        memset(lastDiff, 0, sizeof(lastDiff));
        lastBitRes        = bitRes;
        lastElapsedMs     = serialElapsed;
        lastMidMode       = midpointMode;
        lastMidRingHead   = midRingHead;
        lastMidRingFill   = midRingFill;
        lastMidPostCount  = midPostCount;
        lastMidPreSize    = midPreSize;
        captureReady      = true;
        currentScreen     = SCR_PLOT;
        drawPlotScreen();
    }
}
