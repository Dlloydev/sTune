#pragma once
#ifndef S_TUNE_H_
#define S_TUNE_H_

class sTune {  // Inflection Point Autotuner

  public:

    enum Action : uint8_t {directIP, direct5T, reverseIP, reverse5T};
    enum SerialMode : uint8_t {serialOFF, printALL, printSUMMARY, printDEBUG, printPIDTUNER, serialPLOTTER};
    enum TunerStatus : uint8_t {inOut, test, tunings, runPid, timerPid};
    enum TuningMethod : uint8_t { ZN_PID, ZN_Half_PID, Damped_PID, NoOvershoot_PID, CohenCoon_PID,
                                ZN_PI, ZN_Half_PI, Damped_PI, NoOvershoot_PI, CohenCoon_PI
                              };
    sTune();
    sTune(float *input, float *output, TuningMethod tuningMethod, Action action, SerialMode serialMode);
    ~sTune() {};

    void Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,
                   uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples);
    uint8_t Run();
    void Reset();
    void printTestRun();
    void printResults();
    void printPidTuner(uint8_t everyNth);
    void plotter(float setpoint, uint8_t everyNth);

    // Set functions
    void SetControllerAction(Action Action);
    void SetSerialMode(SerialMode SerialMode);
    void SetTuningMethod(TuningMethod TuningMethod);

    // Query functions
    float GetKp();                  // proportional gain
    float GetKi();                  // integral gain
    float GetKd();                  // derivative gain
    float GetProcessGain();         // process gain
    float GetDeadTime();            // process dead time (seconds)
    float GetTau();                 // process time constant (seconds)
    float GetTimeIntegral();        // process time integral (seconds)
    float GetTimeDerivative();      // process time derivative (seconds)
    uint8_t GetControllerAction();
    uint8_t GetSerialMode();
    uint8_t GetTuningMethod();
    void GetAutoTunings(float * kp, float * ki, float * kd);

  private:

    Action _action;
    SerialMode _serialMode;
    TunerStatus _tunerStatus;
    TuningMethod _tuningMethod;

    float *_input, *_output, _settlePeriodUs, _samplePeriodUs, _tangentPeriodUs;
    float  _inputSpan, _outputSpan, _outputStart, _outputStep;
    float pvInst, pvAvg, pvIp, pvMax, pvPk, pvRes, slopeIp, pvTangent, pvTangentPrev = 0, pvStart;
    float _kp, _ki, _kd, _Ku, _Tu, _td, _R, _Ko, _TuMin, _tdMin;

    uint8_t ipCount = 0, plotCount = 0;
    uint16_t _bufferSize, _samples, sampleCount = 0, pvPkCount = 0, dtCount = 0;
    uint32_t _settleTimeSec, _testTimeSec, usPrev = 0, settlePrev = 0, usStart, us, ipUs;

    const float kexp = 4.3004; // (1 / exp(-1)) / (1 - exp(-1))
    const float epsilon = 0.0001f;
};
#endif
