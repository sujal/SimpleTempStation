// File: sketch.ino
// Description: A simple remote temp station, interrogatable over BLE
// Copyright: 2012 Forche, LLC.
// Author: Sujal Shah

#include <Boards.h>
#include "Wire.h"
#include <math.h>

#include <app.h>
#include <ble.h>
#include <hal_aci_tl.h>
#include <hal_platform.h>
#include <hal_power.h>
#include <hal_uart.h>
#include <system.h>

#include <SPI.h>
#include "ble.h"

#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_BMP085.h>

#define LED_PIN_OUT 3

Adafruit_BMP085 bmp;
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

#define POLLING_INTERVAL_SEC 600ul
#define MAX_HISTORY 12
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8
#define GREEN_HEIGHT 5
#define ORANGE_HEIGHT 1
#define RED_HEIGHT 2

// Communication stuff

#define COMMAND_LEN 2

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

float readingsHistory[MAX_HISTORY];
int mostRecentIndex = MAX_HISTORY-1;

boolean FIRST_CHECK = true;

unsigned long lastCheckMillis;
unsigned long pollingInterval;

void setup() {

  pollingInterval = POLLING_INTERVAL_SEC * 1000ul;
  lastCheckMillis = 0ul;

  for (int i = 0; i < MAX_HISTORY; ++i)
  {
    /* code */
    readingsHistory[i] = 0.0f;
  }


  Serial.begin(57600);

  Serial.print("RAM is ");
  Serial.println(freeRam());

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();

  ble_begin();

  // Serial.print("MAX_HISTORY IS ");
  // Serial.println(MAX_HISTORY);
  pinMode(LED_PIN_OUT, OUTPUT);
  matrix.begin(0x70);  // pass in the address
  bmp.begin(); // according to docs, at 0x77 on i2c bus
}

float lastTemp = 0.0;

void loop() {

    unsigned long currentMillis = millis();
    // Serial.println("HADKAHDKASDHASLKDJSADLKAJ");
    // Serial.println(currentMillis);

    if (FIRST_CHECK || (currentMillis - lastCheckMillis) > pollingInterval) {
          Serial.println(currentMillis);
      FIRST_CHECK=false;
      lastCheckMillis = currentMillis;
      float temp = bmp.readTemperature();

      float ftemp = (temp * (9.0/5.0)) + 32.0f;

      if (ftemp != readingsHistory[mostRecentIndex]) {
        digitalWrite(LED_PIN_OUT, HIGH);
      } else {
        digitalWrite(LED_PIN_OUT, LOW);
      }
      mostRecentIndex++;
      if (mostRecentIndex >= MAX_HISTORY) {
        mostRecentIndex = 0;
      }

      // Serial.print("storing in ");
      // Serial.println(mostRecentIndex);

      readingsHistory[mostRecentIndex] = ftemp;

      Serial.print("Temperature = ");
      Serial.print(temp);
      Serial.print(" *C / ");
      Serial.print(ftemp);
      Serial.println(" *F");

      Serial.print("Pressure = ");
      Serial.print(bmp.readPressure());
      Serial.println(" Pa");

      // // Calculate altitude assuming 'standard' barometric
      // // pressure of 1013.25 millibar = 101325 Pascal
      // Serial.print("Altitude = ");
      // Serial.print(bmp.readAltitude(101325));
      // Serial.println(" meters");

      // Serial.println();

      matrix.clear();
      int j = mostRecentIndex-(MATRIX_WIDTH-1);   // 0 - 8 = -8 -> 288 -8 = 280; index
      if (j < 0) {
        j = MAX_HISTORY + j;
      }

      for (int i=0; i<MATRIX_WIDTH; i++) {

        int orange = 0;
        int red = 0;
        int green = 0;

        int index = j+i;
        if (index >= MAX_HISTORY) {
          index-=MAX_HISTORY;
        }

        float thisFtemp = readingsHistory[index];

        green = (int)floor((thisFtemp-10.0f)/10.0f);

        // Serial.println(green);

        if (green > MATRIX_HEIGHT) {
          green = MATRIX_HEIGHT;
        }

        if (green < 0) {
          green = 0;
        }

        if (green > GREEN_HEIGHT+ORANGE_HEIGHT) {
          red = green - (GREEN_HEIGHT+ORANGE_HEIGHT);
          green -= red;
        }

        if (green > GREEN_HEIGHT) {
          orange = green - GREEN_HEIGHT;
          green -= orange;
        }

        int startPos = MATRIX_HEIGHT-1;
        int endPoint = startPos - green + 1;

        if (green > 0) {
          matrix.drawLine(i,startPos,i,endPoint,LED_GREEN);

        }

        startPos = endPoint-1;

        if (orange > 0) {
          endPoint -= orange;
          matrix.drawLine(i,startPos, i, endPoint, LED_YELLOW);
        }

        startPos = endPoint-1;

        if (red > 0) {
          endPoint -= red;
          matrix.drawLine(i,startPos, i, endPoint, LED_RED);
        }

      }
      matrix.writeDisplay();

    }

    char buf[COMMAND_LEN] = {0};
    unsigned char pos = 0;


    while (ble_available()) {
      // read up to one 4 digit command
      buf[pos] = ble_read();
      if (++pos >= COMMAND_LEN) {
        break;
      }
    }

    if (pos == 2) {
      // only accept full commands
      if (buf[0] == 0x54) {

        int j = mostRecentIndex-(MATRIX_WIDTH-1);   // 0 - 8 = -8 -> 288 -8 = 280; index
        if (j < 0) {
          j = MAX_HISTORY + j;
        }

        for (int i=0; i<MATRIX_WIDTH; i++) {
          int index = j+i;
          if (index >= MAX_HISTORY) {
            index-=MAX_HISTORY;
          }
          float thisFtemp = readingsHistory[index];
          char outbuf[4];
          memcpy(outbuf, &thisFtemp, sizeof(thisFtemp));
          ble_write(outbuf[0]);
          ble_write(outbuf[1]);
          ble_write(outbuf[2]);
          ble_write(outbuf[3]);

        }

      }
    }


    // delay(POLLING_INTERVAL_SEC*1000);
    ble_do_events();
}
