#include "Arduino.h"
#include "sTan.h"

void sTan::begin(uint16_t bufferSize) {
  inputArray = new float[bufferSize];
  bufSize = bufferSize;
  sTan::init(0);
}

void sTan::init(float reading) {
  index = 0;
  sum = reading * bufSize;
  for (uint16_t i = 0; i < bufSize; i++) {
    inputArray[i] = reading;
  }
}

float sTan::avgVal(float reading) {
  index++;
  if (index >= bufSize) index = 0;
  sum += reading - inputArray[index];
  inputArray[index] = reading;
  return float (sum / bufSize);
}

float sTan::startVal() {
  uint16_t tailIndex = index + 1;
  if (tailIndex >= bufSize) tailIndex = 0;
  return inputArray[tailIndex];
}

float sTan::slope(float reading) {
  return reading - sTan::startVal();
}

uint16_t sTan::length() {
  return bufSize;
}
