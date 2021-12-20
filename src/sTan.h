#pragma once
#ifndef S_TAN_H_
#define S_TAN_H_

class sTan  // circular buffer for a sliding tangent line
{
  public:

    sTan();
    sTan(uint16_t bufferSize) : bufSize(bufferSize) {}
    ~sTan() {};

    void begin(uint16_t bufferSize);
    void init(float reading);
    float avgVal(float reading);
    float startVal();
    float slope(float reading);
    uint16_t length();

  private:
    uint16_t bufSize;   // buffer size
    uint16_t index;     // index position
    float sum;          // sum of readings
    float *inputArray;  // pointer to input array
};
#endif
