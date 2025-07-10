#include <SPI.h>
#include <ADS1256.h>                   // ADS1256 24-bit ADC library
#include <lvgl.h>                      // LittlevGL GUI framework
#include <Arduino_H7_Video.h>          // Display driver for GIGA
#include <Arduino_GigaDisplayTouch.h>  // Touch input for GIGA

Arduino_H7_Video          Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch  TouchDetector;


//–– CONFIGURATION
#define NUM_CHANNELS 8
#define VREF         2.5    // ADS1256 reference voltage

// ADS1256 constructor: (clockMHz, VREF, useResetPin)
ADS1256 adc(4, VREF, false);

// GUI state
static bool channelActive[NUM_CHANNELS];
static lv_obj_t* channelLabels[NUM_CHANNELS];
static const char* spsOptions =
  "2 SPS\n"
  "5 SPS\n"
  "10 SPS\n"
  "15 SPS\n"
  "25 SPS\n"
  "30 SPS\n"
  "60 SPS\n";
static const uint8_t drateValues[] = {
  ADS1256_DRATE_2, ADS1256_DRATE_5, ADS1256_DRATE_10,
  ADS1256_DRATE_15, ADS1256_DRATE_25, ADS1256_DRATE_30,
  ADS1256_DRATE_60
};

//–– CALLBACKS
static void onBufferChanged(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    bool bufOn = lv_dropdown_get_selected(e->target);
    adc.enableBuffer(bufOn);
  }
}

static void changeDataRate(uint8_t drate) {
  // stop continuous read
  adc.sendCommand(ADS1256_CMD_SDATAC);
  // write DRATE register (addr 0x03) :contentReference[oaicite:1]{index=1}
  adc.writeRegister(0x03, &drate, 1);
  // sync & wakeup
  adc.sendCommand(ADS1256_CMD_SYNC);
  delayMicroseconds(10);
  adc.sendCommand(ADS1256_CMD_WAKEUP);
  // restart continuous
  adc.sendCommand(ADS1256_CMD_RDATAC);
}

static void onSpsChanged(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    uint8_t sel = lv_dropdown_get_selected(e->target);
    changeDataRate(drateValues[sel]);
  }
}

static void onChannelToggled(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    int ch = (int)(intptr_t)lv_obj_get_user_data(e->target);
    channelActive[ch] = lv_checkbox_is_checked(e->target);
  }
}

//–– GUI SETUP
void setupGUI() {
  // Buffer ON/OFF drop-down
  lv_obj_t* ddBuf = lv_dropdown_create(lv_scr_act());
  lv_dropdown_set_options(ddBuf, "Off\nOn");
  lv_obj_align(ddBuf, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_set_event_cb(ddBuf, onBufferChanged);

  // SPS drop-down
  lv_obj_t* ddSps = lv_dropdown_create(lv_scr_act());
  lv_dropdown_set_options(ddSps, spsOptions);
  lv_obj_align(ddSps, LV_ALIGN_TOP_LEFT, 120, 10);
  lv_obj_set_event_cb(ddSps, onSpsChanged);

  // Channel check-boxes
  for (int i = 0; i < NUM_CHANNELS; ++i) {
    channelActive[i] = true;
    lv_obj_t* cb = lv_checkbox_create(lv_scr_act());
    lv_checkbox_set_text(cb, String(i).c_str());
    lv_obj_set_user_data(cb, (void*)(intptr_t)i);
    lv_obj_align(cb,
      LV_ALIGN_TOP_LEFT,
      10 + (i % 4) * 100,
      60 + (i / 4) * 30);
    lv_obj_set_event_cb(cb, onChannelToggled);
    lv_checkbox_set_checked(cb, true);
  }

  // Voltage labels
  for (int i = 0; i < NUM_CHANNELS; ++i) {
    channelLabels[i] = lv_label_create(lv_scr_act());
    lv_label_set_text_fmt(channelLabels[i], "Ch%d: --.- V", i);
    lv_obj_align(channelLabels[i],
      LV_ALIGN_TOP_LEFT,
      10, 200 + i * 25);
  }
}

//–– ARDUINO SETUP & LOOP
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // SPI for ADS1256
  SPI.begin();

  // ADS1256 init & continuous mode :contentReference[oaicite:2]{index=2}
  adc.begin();
  adc.reset();
  adc.setGain(1);
  changeDataRate(drateValues[2]); // default 10 SPS

  // LVGL + display driver init :contentReference[oaicite:3]{index=3}
  lv_init();
  Arduino_H7_Video.begin();
  Arduino_H7_Video.setRotation(1);
  Arduino_GigaDisplayTouch.begin();

  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf[480 * 10];
  lv_disp_draw_buf_init(&draw_buf, buf, nullptr, 480 * 10);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.flush_cb = Arduino_H7_Video.flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.hor_res = 800;
  disp_drv.ver_res = 480;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = Arduino_GigaDisplayTouch.read;
  lv_indev_drv_register(&indev_drv);

  setupGUI();
}

void loop() {
  lv_timer_handler();

  static uint32_t lastMillis = 0;
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();

    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
      if (channelActive[ch]) {
        adc.setChannel(ch);
        adc.waitDRDY();
        float volts = adc.readCurrentChannelVolts();  // returns volts :contentReference[oaicite:4]{index=4}
        lv_label_set_text_fmt(channelLabels[ch],
                              "Ch%d: %.3f V", ch, volts);
      } else {
        lv_label_set_text_fmt(channelLabels[ch],
                              "Ch%d: ---", ch);
      }
    }
  }
}
