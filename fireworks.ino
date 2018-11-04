
/***************************************************
  Fireworks - A simple touch fireworks demo

  Based on code written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"
#include "Adafruit_STMPE610.h"

#ifdef ESP8266
#define STMPE_CS 16
#define TFT_CS   0
#define TFT_DC   15
#define SD_CS    2
#endif
#ifdef ESP32
#define STMPE_CS 32
#define TFT_CS   15
#define TFT_DC   33
#define SD_CS    14
#endif
#ifdef __AVR_ATmega32U4__
#define STMPE_CS 6
#define TFT_CS   9
#define TFT_DC   10
#define SD_CS    5
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
#define STMPE_CS 6
#define TFT_CS   9
#define TFT_DC   10
#define SD_CS    5
#endif
#ifdef TEENSYDUINO
#define TFT_DC   10
#define TFT_CS   4
#define STMPE_CS 3
#define SD_CS    8
#endif
#ifdef ARDUINO_STM32_FEATHER
#define TFT_DC   PB4
#define TFT_CS   PA15
#define STMPE_CS PC7
#define SD_CS    PC5
#endif
#ifdef ARDUINO_FEATHER52
#define STMPE_CS 30
#define TFT_CS   13
#define TFT_DC   11
#define SD_CS    27
#endif
#if defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
#define TFT_DC   P5_4
#define TFT_CS   P5_3
#define STMPE_CS P3_3
#define SD_CS    P3_2
#endif

#define TFT_RST -1

#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

// Use hardware SPI and the above for CS/DC
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

#define PCOUNT 1024

bool pa[PCOUNT]; // Active?
float px[PCOUNT];
float py[PCOUNT];
float vx[PCOUNT];
float vy[PCOUNT];
float pd[PCOUNT];
float pv[PCOUNT];
uint16_t pcl[PCOUNT];



void setup() {
  Serial.begin(115200);

  tft.begin(HX8357D);

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(HX8357_RDPOWMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDCOLMOD);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDDIM);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDDSDR);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);

  Serial.println(F("Benchmark                Time (microseconds)"));

  tft.fillScreen(HX8357_BLACK);
  tft.setRotation(1);

  ts.begin();

}

void loop(void) {
  int fp = 0;
  int fps[64];
  int fc = 0;

  for (int i = 0; i < PCOUNT; i++) {
    if (!pa[i]) {
      if (fp < 64) {
        fps[fp++] = i;
      }
      fc++;
    } else {
      tft.drawPixel(px[i], py[i], 0);
      px[i] = px[i] + vx[i];
      py[i] = py[i] + vy[i];
      if (px[i] < 0 || px[i] >= tft.width() || py[i] < 0 || py[i] >= tft.height()) {
        pa[i] = false;
      } else {
        tft.drawPixel(px[i], py[i], pcl[i]);
      }
    }
  }

  // First pass - use free particles to drive random particles
  //  if(fp>0) {
  //      for(int f=0; f<fp; f++) {
  //        int i=fps[f];
  //      px[i]=random(tft.width());
  //      py[i]=random(tft.height());
  //      pd[i]=random(360);
  //      pv[i]=random(50,250)/100.0;
  //      vx[i]=pv[i]*sin(pd[i]);
  //      vy[i]=pv[i]*cos(pd[i]);
  //      pcl[i]=tft.color565(random(128,255),random(128,255),random(128,255));
  //      pa[i]=true;
  //      }
  //  }

  //  Second pass, group the particles
  //  if (fp > 0) {
  //    int sx = random(tft.width());
  //    int sy = random(tft.height());
  //    int sv = random(100, 300);
  //    int dgstep = 360 / fp;
  //    int sdg = random(360);
  //    int cl = tft.color565(random(128, 255), random(128, 255), random(128, 255));
  //    for (int f = 0; f < fp; f++) {
  //      int i = fps[f];
  //
  //      px[i] = sx;
  //      py[i] = sy;
  //      pd[i] = sdg;
  //      pv[i] = (sv + random(0, 5)) / 100.0;
  //      vx[i] = pv[i] * sin(pd[i]);
  //      vy[i] = pv[i] * cos(pd[i]);
  //      pcl[i] = cl;
  //      pa[i] = true;
  //      sdg = sdg + dgstep;
  //    }
  //    tft.drawPixel(sx, sy, cl);
  //
  //  }

  // Third pass, make groups happen where touch happens
  if (fp > 0) {
    if (!ts.bufferEmpty()) {
      TS_Point p = ts.getPoint();

      int sx = map(p.y, TS_MINX, TS_MAXX, 0, tft.width());
      int sy = tft.height() - map(p.x, TS_MINY, TS_MAXY, 0, tft.height());
      int sv = random(100, 300);
      int dgstep = 360 / fp;
      int sdg = random(360);
      int cl = tft.color565(random(128, 255), random(128, 255), random(128, 255));
      for (int f = 0; f < fp; f++) {
        int i = fps[f];

        px[i] = sx;
        py[i] = sy;
        pd[i] = sdg;
        pv[i] = (sv + random(0, 5)) / 100.0;
        vx[i] = pv[i] * sin(pd[i]);
        vy[i] = pv[i] * cos(pd[i]);
        pcl[i] = cl;
        pa[i] = true;
        sdg = sdg + dgstep;
      }
      tft.drawPixel(sx, sy, cl);
    }
  }

}
