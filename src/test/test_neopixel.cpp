#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

#define PIN       38
#define NUMPIXELS 6

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

#define DELAYVAL 500

void setup() {
  pixels.begin();
}

void loop() {
  pixels.clear();

  for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 255, 0));

    pixels.show();
  }
}