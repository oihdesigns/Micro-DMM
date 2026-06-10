#include <Arduino_AdvancedAnalog.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>

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

static const Preset PRESETS[] = {
  { "Board1",
    1,16,25,1,5000,
    { {true,1,1.612548f,44.462994f}, {true,2,1.613425f,10.900730f},
      {false,1,0,1},{false,1,0,1},{false,1,0,1},{false,1,0,1},
      {false,1,0,1},{false,1,0,1} },
    {false,4,5,2,0,1}, {false,2,3,1,0,1} },

  { "Board1 DiffA",
    1,16,25,1,5000,
    { {true,1,1.612548f,44.462994f}, {true,2,1.613425f,10.900730f},
      {false,1,0,1},{false,1,0,1},{false,1,0,1},{false,1,0,1},
      {false,1,0,1},{false,1,0,1} },
    {false,4,5,2,0.0f,11.8f}, {false,2,3,1,0,1} },

  { "Board1 5mOhm",
    3,14,30,2,2000,
    { {true,1,1.612548f,44.462994f}, {false,2,1.613425f,41.124543f},
      {false,1,0,1},{false,1,0,1},{false,1,0,1},{false,1,0,1},
      {false,1,0,1},{false,1,0,1} },
    {true,4,5,2,0.0f,44.107620f}, {false,2,3,1,0,1} },

  { "120VAC Board",
    3,14,50,1,4000,
    { {false,2,0.0f,1.0f}, {false,2,1.613425f,41.124543f},
      {false,1,0,1},{false,1,0,1},{false,1,0,1},{false,1,0,1},
      {false,1,0,1},{false,1,0,1} },
    {true,4,5,2,0.0f,1.181836f}, {true,3,2,1,0.0f,102.779803f} },

  { "120VAC R4Nano",
    1,16,50,1,1000,
    { {false,2,0,1},{false,2,0,1},{false,1,0,1},{false,1,0,1},
      {false,1,0,1},{false,1,0,1},{false,1,0,1},{false,1,0,1} },
    {true,4,5,2,0.0f,1.623f}, {true,7,6,1,0.0f,166.2f} },
};
static const int N_PRESETS = 5;

// ── Screen state ──────────────────────────────────────────────────────────────
enum Screen { SCR_SETTINGS, SCR_CAPTURING, SCR_PLOT, SCR_PRESETS, SCR_DIFF };
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
static const int PLT_X = 62;
static const int PLT_Y = HDR_H + 4;
static const int PLT_W = DISP_W - PLT_X - 54;
static const int PLT_H = DISP_H - HDR_H - 50;
static const int LEG_Y = PLT_Y + PLT_H + 4;

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
    drawBtn(DISP_W-160,2,156,HDR_H-4,C_BORDER,C_TEXT,"Presets",2);

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
static void drawPresetsScreen() {
    gfx.fillScreen(C_BG);
    gfx.fillRect(0,0,DISP_W,HDR_H,C_PANEL);
    gfx.setTextColor(C_CYAN); gfx.setTextSize(3);
    gfx.setCursor(12,8); gfx.print("Select Preset");
    drawBtn(DISP_W-114,2,110,HDR_H-4,C_BORDER,C_TEXT,"< Back",2);
    for(int i=0;i<N_PRESETS;i++) {
        drawBtn(4,50+i*82,DISP_W-8,78,C_PANEL,C_TEXT,PRESETS[i].name,2);
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

    // ── Determine Y ranges for V and I separately ─────────────────────────
    float vMin=1e30f, vMax=-1e30f, iMin=1e30f, iMax=-1e30f;
    bool hasV=false, hasI=false;

    // Standalone channels
    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0||lastLogCnt[ch]==0) continue;
        bool isI=(lastChanType[ch]==2);
        for(int j=0;j<lastLogCnt[ch];j++) {
            float v=calChan(getRawVal(ch,j),lastChanOffset[ch],lastChanScale[ch],lastBitRes);
            if(isI){if(v<iMin)iMin=v;if(v>iMax)iMax=v;hasI=true;}
            else   {if(v<vMin)vMin=v;if(v>vMax)vMax=v;hasV=true;}
        }
    }
    // Diff channels
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en||lastLogCnt[dc.pos]==0||lastLogCnt[dc.neg]==0) continue;
        bool isI=(dc.typ==2);
        int cnt=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]);
        for(int j=0;j<cnt;j++) {
            float v=calDiff(getRawVal(dc.pos,j),getRawVal(dc.neg,j),dc.off,dc.scl,lastBitRes);
            if(isI){if(v<iMin)iMin=v;if(v>iMax)iMax=v;hasI=true;}
            else   {if(v<vMin)vMin=v;if(v>vMax)vMax=v;hasV=true;}
        }
    }
    // Fallback / guard against flat lines
    if(!hasV&&!hasI){ vMin=0;vMax=1;hasV=true; }
    if(hasV&&vMax-vMin<1e-6f){ float m=(vMin+vMax)/2; vMin=m-1;vMax=m+1; }
    if(hasI&&iMax-iMin<1e-6f){ float m=(iMin+iMax)/2; iMin=m-1;iMax=m+1; }
    if(!hasV){ vMin=iMin;vMax=iMax; }
    if(!hasI){ iMin=vMin;iMax=vMax; }

    // ── Draw plot area ────────────────────────────────────────────────────
    gfx.fillRect(PLT_X,PLT_Y,PLT_W,PLT_H,0x0821u);
    gfx.drawRect(PLT_X,PLT_Y,PLT_W,PLT_H,C_BORDER);
    for(int g=1;g<4;g++) {
        int gy=PLT_Y+g*PLT_H/4;
        for(int x=PLT_X;x<PLT_X+PLT_W;x+=6) gfx.drawPixel(x,gy,C_BORDER);
    }

    // Y axis labels — V left, I right
    gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
    char lbuf[12];
    snprintf(lbuf,sizeof(lbuf),"%.3g",vMax); gfx.setCursor(2,PLT_Y+2);         gfx.print(lbuf);
    snprintf(lbuf,sizeof(lbuf),"%.3g",vMin); gfx.setCursor(2,PLT_Y+PLT_H-10); gfx.print(lbuf);
    if(hasI) {
        int rx=PLT_X+PLT_W+4;
        snprintf(lbuf,sizeof(lbuf),"%.3g",iMax); gfx.setCursor(rx,PLT_Y+2);         gfx.print(lbuf);
        snprintf(lbuf,sizeof(lbuf),"%.3g",iMin); gfx.setCursor(rx,PLT_Y+PLT_H-10); gfx.print(lbuf);
    }

    // Find max log count for X scaling
    int maxCnt=1;
    for(int ch=0;ch<N_CHANNELS;ch++) if(lastPinReq[ch]&&lastLogCnt[ch]>maxCnt) maxCnt=lastLogCnt[ch];
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(dc.en) { int c=min(lastLogCnt[dc.pos],lastLogCnt[dc.neg]); if(c>maxCnt) maxCnt=c; }
    }

    auto mapY = [&](float v, float mn, float mx) -> int {
        int py = PLT_Y+PLT_H-1-(int)((v-mn)*(PLT_H-2)/(mx-mn));
        return constrain(py, PLT_Y, PLT_Y+PLT_H-1);
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

    // Legend
    int lx=PLT_X;
    for(int ch=0;ch<N_CHANNELS;ch++) {
        if(!lastPinReq[ch]||lastChanType[ch]==0) continue;
        gfx.fillRect(lx,LEG_Y,14,10,CH_COLORS[ch]);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
        char lb[8]; snprintf(lb,sizeof(lb),"%s(%s)",CH_NAMES[ch],lastChanType[ch]==1?"V":"I");
        gfx.setCursor(lx+17,LEG_Y+1); gfx.print(lb);
        lx+=70; if(lx>DISP_W-70) break;
    }
    for(int d=0;d<2;d++) {
        const DiffCfg& dc=lastDiff[d];
        if(!dc.en||lx>DISP_W-90) continue;
        gfx.fillRect(lx,LEG_Y,14,10,DIFF_COLORS[d]);
        gfx.setTextColor(C_TEXT); gfx.setTextSize(1);
        char lb[16]; snprintf(lb,sizeof(lb),"D%d(%s)",d+1,dc.typ==1?"V":"I");
        gfx.setCursor(lx+17,LEG_Y+1); gfx.print(lb);
        lx+=70;
    }
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
    lastElapsedMs = lastLogMs>0 ? lastLogMs : millis()-startMs;
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
    // Presets button
    if(hit(DISP_W-160,2,156,HDR_H-4,tx,ty)){
        currentScreen=SCR_PRESETS; drawPresetsScreen(); return;
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
    for(int i=0;i<N_PRESETS;i++) {
        if(hit(4,50+i*82,DISP_W-8,78,tx,ty)){
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

// ── setup / loop ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    displayPresent=touch.begin();
    if(displayPresent){ gfx.begin(); gfx.setRotation(1); drawSettingsScreen(); }
    else               { while(!Serial); }
}

void loop() {
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
                    default: break;
                }
            }
        }
    }

    // ── Serial: works in both display and non-display mode ───────────────────
    if(!Serial.available()) return;
    contMode=false;  // any incoming command stops continuous view
    if(displayPresent){ currentScreen=SCR_CAPTURING; drawCaptureScreen(); }
    String cmd=Serial.readStringUntil('\n'); cmd.trim();

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
        Serial.println("ERR:ADC_BEGIN_FAILED"); return;
    }
    adcRunning=true;

    uint32_t startMs=millis();
    while(!midDone&&(midpointMode||millis()-startMs<(uint32_t)duration)){
        if(midpointMode&&Serial.available()){
            String inc=Serial.readStringUntil('\n'); inc.trim();
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
        Serial.print("PIN:");Serial.print(CH_NAMES[i]);Serial.print("|BITS:");Serial.print(bitRes);
        Serial.print("|COUNT:");Serial.print(cnt[i]);Serial.print("|MIN:");Serial.print(mn[i]);
        Serial.print("|MAX:");Serial.print(mx[i]);Serial.print("|MEAN:");Serial.print(mean,4);
        Serial.print("|RMS:");Serial.print(rms,4);Serial.print("|STD:");Serial.print(stdDev,4);
        Serial.print("|LOGGED:");Serial.println(logCnt[i]);
    }
    for(int i=0;i<N_CHANNELS;i++){
        if(!pinReq[i]||logCnt[i]==0) continue;
        Serial.print("DATA:");Serial.print(CH_NAMES[i]);Serial.print("|VALS:");
        if(midpointMode){
            int si=(midRingFill<midPreSize)?0:midRingHead;
            for(int j=0;j<midRingFill;j++){
                Serial.print(logData[i][(si+j)%midPreSize]);
                if(j<midRingFill-1||midPostCount>0) Serial.print(',');
            }
            for(int j=0;j<midPostCount;j++){
                Serial.print(logData[i][midPreSize+j]);
                if(j<midPostCount-1) Serial.print(',');
            }
        } else {
            for(int j=0;j<logCnt[i];j++){Serial.print(logData[i][j]);if(j<logCnt[i]-1)Serial.print(',');}
        }
        Serial.println();
    }
    Serial.print("DONE|ELAPSED:");
    uint32_t serialElapsed;
    if(midpointMode){
        serialElapsed=(uint32_t)((uint64_t)(midRingFill+midPostCount)*smoothCount*1000/sampleRate);
        if(serialElapsed==0) serialElapsed=1;
    } else {
        serialElapsed=lastLogMs>0?lastLogMs:(millis()-startMs);
    }
    Serial.println(serialElapsed);

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
