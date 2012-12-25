# File: sketch.ino
# Description: A simple remote temp station, interrogatable over BLE
# Copyright: 2012 Forche, LLC.
# Author: Sujal Shah

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
#include <BleFirmata.h>

#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_BMP085.h>

#define LED_PIN_OUT 3

Adafruit_BMP085 bmp;
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

#define POLLING_INTERVAL_SEC 300
#define MAX_HISTORY 86400/300
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8
#define GREEN_HEIGHT 5
#define ORANGE_HEIGHT 1
#define RED_HEIGHT 2

float readingsHistory[MAX_HISTORY];
int mostRecentIndex = MAX_HISTORY-1;

void setup() {
  Serial.begin(9600);
  Serial.print("MAX_HISTORY IS ");
  Serial.println(MAX_HISTORY);
  pinMode(LED_PIN_OUT, OUTPUT);
  matrix.begin(0x70);  // pass in the address
  bmp.begin(); // according to docs, at 0x77 on i2c bus
}

float lastTemp = 0.0;

void loop() {

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

    Serial.print("storing in ");
    Serial.println(mostRecentIndex);

    readingsHistory[mostRecentIndex] = ftemp;

    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.print(" *C / ");
    Serial.print(ftemp);
    Serial.println(" *F");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude(101422));
    Serial.println(" meters");

    Serial.println();

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

      Serial.println(green);

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

    delay(300000);
}
