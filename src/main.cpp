#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include "FS.h"

#define SD_CS_PIN 5
SdFat SD;
File myFile;


// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);

  while

}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}