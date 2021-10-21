
# GT911
GT911 Touch library for Arduino

### This is experimental, only tested with an ESP32 & DFRobot DFR0669 TFT Touchscreen

Based off these repos
 - https://github.com/nik-sharky/arduino-goodix
 - https://github.com/u4mzu4/Arduino_GT911_Library

I needed a GT911 library to use with a [DFRobot TFT LCD Capacitive Touchscreen](https://www.dfrobot.com/product-2107.html) but as the RST pin is shared with SPI I couldn't use the existing libs.

It's been changed for my own needs so it can be used with polling instead of an interrupt.
The GT911 config can be read and written too but make sure you check the docs for values;

### Polling example
````cpp
#include <Arduino.h>
#include <GT911.h>

GT911 ts = GT911();

void setup() {
  Serial.begin(115200);
  ts.begin();
}

void loop() {
  uint8_t touches = ts.touched(GT911_MODE_POLLING);

  if (touches) {
    GTPoint* tp = ts.getPoints();
    for (uint8_t  i = 0; i < touches; i++) {
      Serial.printf("#%d  %d,%d s:%d\n", tp[i].trackId, tp[i].x, tp[i].y, tp[i].area);
    }
  }
}
````

### Config update example
````cpp
void setup() {
  GTConfig *cfg = ts.readConfig();
  cfg->hSpace = (5 | (5 << 4));
  cfg->vSpace = (5 | (5 << 4));
  ts.writeConfig();
}
````
