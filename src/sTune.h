#pragma once
#ifndef S_TUNE_H_
#define S_TUNE_H_

class sTune {  // Inflection Point Autotuner

  public:

    enum Action : uint8_t {directIp, direct5t, reverseIp, reverse5t};
    enum SerialMode : uint8_t {serialOff, printAll, printSummary, printDebug, plot};
    enum TunerStatus : uint8_t {inOut, test, tunings, runPid, timerPid};
    enum TuningRule : uint8_t { zieglerNicholsPi, zieglerNicholsPid, tyreusLuybenPi, tyreusLuybenPid,
                                cianconeMarlinPi, cianconeMarlinPid, amigofPid, pessenIntegralPid,
                                someOvershootPid, noOvershootPid
                              };

    sTune();
    sTune(float *input, float *output, TuningRule tuningRule, Action action, SerialMode serialMode);
    ~sTune() {};

    void Configure(const float outputStart, const float outputStep, const uint32_t testTimeSec, const uint32_t settleTimeSec, const uint16_t samples);
    uint8_t Run();
    void Reset();
    void printTestRun();
    void printResults();
    void SetAutoTunings(float* kp, float* ki, float* kd, float* ku, float* tu, float* td);

  private:

    Action _action;
    SerialMode _serialMode;
    TunerStatus _tunerStatus;
    TuningRule _tuningRule;

    float *_input, *_output, _settlePeriodUs, _samplePeriodUs, _tangentPeriodUs, _outputStart, _outputStep;;
    float pvInst, pvAvg, pvIp, pvMax, pvPk, pvRes, slopeIp, pvTangent, pvTangentPrev = 0, pvStart;
    float _kp, _ki, _kd, _Ku, _Tu, _td;

    uint8_t dtCount = 0, ipCount = 0;
    uint16_t _bufferSize, _samples, sampleCount = 0, pvPkCount = 0;
    uint32_t _settleTimeSec, _testTimeSec, usPrev = 0, settlePrev = 0, usStart, us, ipUs;

    const float kexp = 4.3004; // (1 / exp(-1)) / (1 - exp(-1))
    const float epsilon = 0.0001;

    const uint16_t RulesContants[10][3] =
    { //kkp,  kki, kkd x 1000
      { 450,  540,   0 },  // zieglerNicholsPi
      { 600,  176,  75 },  // zieglerNicholsPid
      { 313,  142,   0 },  // tyreusLuybenPi
      { 454,  206,  72 },  // tyreusLuybenPid
      { 303, 1212,   0 },  // cianconeMarlinPi
      { 303, 1333,  37 },  // cianconeMarlinPid
      {   0,    0,   0 },  // amigofPid
      { 700, 1750, 105 },  // pessenIntegralPid
      { 333,  667, 111 },  // someOvershootPid
      { 333,  100,  67 }   // noOvershootPid
    };
};
#endif
