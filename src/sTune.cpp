/****************************************************************************************
   sTune Library for Arduino - Version 2.0.0
   by dlloydev https://github.com/Dlloydev/sTune
   Licensed under the MIT License.

  This is an open loop PID autotuner using a novel s-curve inflection point test method.
  Tuning parameters are determined in about ½Tau on a first-order system with time delay.
  Full 5Tau testing and multiple serial output options are provided.
 ****************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "sTan.h"
#include "sTune.h"

sTan::sTan() {}
sTan tangent = sTan();

sTune::sTune() {
  _input = nullptr;
  _output = nullptr;
  sTune::Reset();
}

sTune::sTune(float *input, float *output, TuningMethod tuningMethod, Action action, SerialMode serialMode) {
  _input = input;
  _output = output;
  _tuningMethod = tuningMethod;
  _action = action;
  _serialMode = serialMode;
  sTune::Reset();
}

void sTune::Reset() {
  _tunerStatus = inOut;
  *_output = _outputStart;
  usPrev = micros();
  settlePrev = usPrev;
  ipUs = 0;
  us = 0;
  _Ku = 0.0f;
  _Tu = 0.0f;
  _td = 0.0f;
  _kp = 0.0f;
  _ki = 0.0f;
  _kd = 0.0f;
  pvIp = 0.0f;
  pvMax = 0.0f;
  pvPk = 0.0f;
  slopeIp = 0.0f;
  pvTangent = 0.0f;
  pvTangentPrev = 0.0f;
  pvAvg = pvInst;
  pvStart = pvInst;
  pvRes = pvInst;
  dtCount = 0;
  ipCount = 0;
  plotCount = 0;
  sampleCount = 0;
  pvPkCount = 0;
}

void sTune::Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,
                      uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples) {
  sTune::Reset();
  _inputSpan = inputSpan;
  _outputSpan = outputSpan;
  _outputStart = outputStart;
  _outputStep = outputStep;
  _testTimeSec = testTimeSec;
  _settleTimeSec = settleTimeSec;
  _samples = samples;
  _tunerStatus = inOut;
  _bufferSize = (uint16_t)(_samples * 0.1);
  _samplePeriodUs = (float)(_testTimeSec * 1000000.0f) / _samples;
  _tangentPeriodUs = _samplePeriodUs * (_bufferSize - 1);
  _settlePeriodUs = (float)(_settleTimeSec * 1000000.0f);
  tangent.begin(_bufferSize);
}

uint8_t sTune::Run() {

  uint32_t usNow = micros();
  uint32_t usElapsed = usNow - usPrev;
  uint32_t settleElapsed = usNow - settlePrev;
  us = usNow - usStart;

  switch (_tunerStatus) {

    case inOut:
      _tunerStatus = test;
      return test;
      break;

    case test:                                          // run inflection point test method
      if (settleElapsed >= _settlePeriodUs) {           // if settling period has expired
        if (sampleCount == 12) *_output = _outputStep;  // provides additional period using _outputStart
        if (usElapsed >= _samplePeriodUs) {             // ready to process a sample
          usPrev = usNow;
          if (sampleCount <= _samples) {                // if within testing period
            float lastPvAvg = pvAvg;
            pvInst = *_input;
            pvAvg = tangent.avgVal(pvInst);
            float pvDiff = abs(pvAvg - lastPvAvg);

            if (pvDiff > epsilon && pvDiff < pvRes) pvRes = pvDiff;  // check buffered input resolution

            if (sampleCount == 0) {                     // initialize at first sample
              tangent.init(pvInst);
              pvAvg = pvInst;
              pvDiff = 0.0f;
              pvRes = pvInst;
              pvStart = pvInst;
              usStart = usNow;
              us = 0;
            }

            //To determine the point of inflection, we'll use a sliding tangent line.
            //https://en.wikipedia.org/wiki/Inflection_point#/media/File:Animated_illustration_of_inflection_point.gif
            pvTangent = pvAvg - tangent.startVal();

            // check for dead time
            bool dt = false;
            if (_action == directIP || _action == direct5T) {
              (pvTangent - pvTangentPrev > 0 + epsilon) ?  dt = true : dt = false; // pvTangent ↗
            } else { // reverse
              (pvTangent - pvTangentPrev < 0 - epsilon) ?  dt = true : dt = false; // pvTangent ↘
            }
            if (dt) dtCount++;
            if (dtCount == 4) {  // update deadtime when tangent changes for 4 samples
              _td = us * 0.000001f;
            }

            // check for inflection point
            bool ipcount = false;
            if (_action == directIP || _action == direct5T) {
              if (pvTangent > slopeIp + epsilon) ipcount = true;
              if (pvTangent < 0 + epsilon) ipCount = 0;  // flat or negative tangent
            } else { // reverse
              if (pvTangent < slopeIp - epsilon) ipcount = true;
              if (pvTangent > 0 - epsilon) ipCount = 0;  // flat or positive tangent
            }
            if (ipcount) {
              ipCount = 0;
              slopeIp = pvTangent;
              ipUs = us;
              pvIp = pvAvg;
            }
            ipCount++;
            if ((_action == directIP || _action == reverseIP) && (ipCount == ((int)(_samples / 16)))) { // reached inflection point
              sampleCount = _samples;

              // apparent pvMax  Re: "Step response with arbitrary initial conditions" https://en.wikipedia.org/wiki/Time_constant
              pvMax = pvIp + slopeIp * kexp;

              //apparent tangent from pvStart to pvMax crossing points
              _Tu = (((pvMax - pvStart) / slopeIp) * _tangentPeriodUs * 0.000001f) - _td;
            }

            if (_action == direct5T || _action == reverse5T) { // continue testing to maximum input
              if (sampleCount >= _samples - 1) sampleCount = _samples - 2;
              if (us > _testTimeSec * 100000) {    // 10% of testTimeSec has elapsed
                if (pvAvg > pvPk) {
                  pvPk = pvAvg + 2 * pvRes;        // set a new boosted peak
                  pvPkCount = 0;                   // reset the "below peak" counter
                } else {
                  pvPkCount++;                     // count up while pvAvg is below the boosted peak
                }
                if (pvPkCount == ((uint16_t)(1.2 * _bufferSize))) {  // test done (assume 3.5τ)
                  pvPkCount++;
                  sampleCount = _samples;
                  pvMax = pvAvg + (pvInst - pvStart) * 0.03f;  // add 3% to pvMax
                  _Tu = (us * 0.000001f * 0.286f) - _td;       // multiply by 0.286 for τ
                }
              }
            }

            if (sampleCount == _samples) {  // testing complete

              _TuMin = _Tu * 0.1667; // units in minutes
              _tdMin = _td * 0.1667; // units in minutes
              _R = _tdMin / _TuMin;
              _Ku =  abs(((pvMax - pvStart) / _inputSpan) / ((_outputStep - _outputStart) / _outputSpan)); // process gain
              _Ko = ((_outputStep - _outputStart) / pvMax) * (_TuMin / _tdMin); // process gain

              _kp = sTune::GetKp();
              _ki = sTune::GetKi();
              _kd = sTune::GetKd();

              sTune::printResults();
              _tunerStatus = tunings;
              return tunings;
              break;
            }
            sTune::printTestRun();
            pvTangentPrev = pvTangent;
          } else _tunerStatus = tunings;
          sTune::printPidTuner(10);
          sampleCount++;
        }
      } else {  // settling
        if (usElapsed >= _samplePeriodUs) {
          *_output = _outputStart;
          usPrev = usNow;
          pvInst = *_input;
          if (_serialMode == printALL || _serialMode == printDEBUG) {
            Serial.print(F(" Sec: "));     Serial.print((float)((_settlePeriodUs - settleElapsed) * 0.000001f), 5);
            Serial.print(F("  pvInst: ")); Serial.print(pvInst, 4);
            Serial.println(F("  Settling  ⤳⤳⤳⤳"));
          }
        }
        _tunerStatus = inOut;
        return inOut;
      }
      _tunerStatus = inOut;
      return inOut;
      break;

    case tunings:
      _tunerStatus = runPid;
      return runPid;
      break;

    case runPid:
      _tunerStatus = timerPid;
      return timerPid;
      break;

    case timerPid:
      if (usElapsed >= _samplePeriodUs) {
        usPrev = usNow;
        _tunerStatus = runPid;
        return runPid;
      } else {
        _tunerStatus = timerPid;
        return timerPid;
      }
      break;

    default:
      _tunerStatus = timerPid;
      return timerPid;
      break;
  }
  return timerPid;
}

void sTune::SetControllerAction(Action Action) {
  _action = Action;
}

void sTune::SetSerialMode(SerialMode SerialMode) {
  _serialMode = SerialMode;
}

void sTune::SetTuningMethod(TuningMethod TuningMethod) {
  _tuningMethod = TuningMethod;
}

void sTune::printPidTuner(uint8_t everyNth) {
  if (_serialMode == printPIDTUNER && (_action == direct5T || _action == reverse5T)) {
    if (sampleCount < _samples) {
      if (plotCount == 0 || plotCount >= everyNth) {
        plotCount = 1;
        Serial.print(us * 0.000001f, 5);  Serial.print(F(", "));
        Serial.print(*_output);           Serial.print(F(", "));
        Serial.println(pvAvg);
      } else plotCount++;
    }
  }
}

void sTune::plotter(float setpoint, uint8_t everyNth) {
  if (_serialMode == serialPLOTTER) {
    if (plotCount >= everyNth) {
      plotCount = 1;
      Serial.print(F("Setpoint:"));  Serial.print(setpoint);        Serial.print(F(", "));
      Serial.print(F("Input:"));     Serial.print(*_input);         Serial.print(F(", "));
      Serial.print(F("Output:"));    Serial.print(*_output * 0.5);  Serial.println(F(","));
    } else plotCount++;
  }
}

void sTune::printTestRun() {

  if (sampleCount < _samples) {
    if (_serialMode == printALL || _serialMode == printDEBUG) {
      Serial.print(F(" Sec: "));           Serial.print(us * 0.000001f, 5);
      Serial.print(F("  pvInst: "));       Serial.print(pvInst, 4);
      Serial.print(F("  pvAvg: "));        Serial.print(pvAvg, 4);
      if (_serialMode == printDEBUG && (_action == direct5T || _action == reverse5T)) {
        Serial.print(F("  pvPk: "));       Serial.print(pvPk, 4);
        Serial.print(F("  pvPkCount: "));  Serial.print(pvPkCount);
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);
      }
      if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);
      }
      Serial.print(F("  pvTangent: "));                 Serial.print(pvTangent, 4);
      if (pvTangent - pvTangentPrev > 0 + epsilon)      Serial.println(F(" ↗"));
      else if (pvTangent - pvTangentPrev < 0 - epsilon) Serial.println(F(" ↘"));
      else                                              Serial.println(F(" →"));
    }
  }
}

void sTune::printResults() {
  if (_serialMode == printALL || _serialMode == printDEBUG || _serialMode == printSUMMARY) {
    Serial.println();
    Serial.print(F(" Controller Action: "));
    if (_action == directIP) Serial.println(F("directIP"));
    else if (_action == direct5T) Serial.println(F("direct5T"));
    else if (_action == reverseIP) Serial.println(F("reverseIP"));
    else Serial.println(F("reverse5T"));
    Serial.print(F(" Tuning Method:     "));
    if (_tuningMethod == ZN_PID) Serial.println(F("ZN_PID"));
    else if (_tuningMethod == ZN_Half_PID) Serial.println(F("ZN_Half_PID"));
    else if (_tuningMethod == Damped_PID) Serial.println(F("Damped_PID"));
    else if (_tuningMethod == NoOvershoot_PID) Serial.println(F("NoOvershoot_PID"));
    else if (_tuningMethod == CohenCoon_PID) Serial.println(F("CohenCoon_PID"));
    else if (_tuningMethod == ZN_PI) Serial.println(F("ZN_PI"));
    else if (_tuningMethod == ZN_Half_PI) Serial.println(F("ZN_Half_PI"));
    else if (_tuningMethod == Damped_PI) Serial.println(F("Damped_PI"));
    else if (_tuningMethod == NoOvershoot_PI) Serial.println(F("NoOvershoot_PI"));
    else Serial.println(F("CohenCoon_PI"));
    Serial.println();
    Serial.print(F(" Output Start:      "));  Serial.println(_outputStart);
    Serial.print(F(" Output Step:       "));  Serial.println(_outputStep);
    Serial.print(F(" Sample Sec:        "));  Serial.println(_samplePeriodUs * 0.000001f, 5);
    Serial.println();
    if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {
      Serial.print(F(" Ip Sec:            "));  Serial.println(ipUs * 0.000001f, 5);
      Serial.print(F(" Ip Slope:          "));  Serial.print(slopeIp, 4);
      if (_action == directIP || _action == direct5T) Serial.println(F(" ↑"));
      else  Serial.println(F(" ↓"));
      Serial.print(F(" Ip Pv:             "));  Serial.println(pvIp, 4);
    }
    Serial.print(F(" Pv Start:          "));  Serial.println(pvStart, 4);
    if (_action == directIP || _action == direct5T) Serial.print(F(" Pv Max:            "));
    else Serial.print(F(" Pv Min:            "));
    Serial.println(pvMax, 4);
    Serial.print(F(" Pv Diff:           "));  Serial.println(pvMax - pvStart, 4);
    Serial.println();
    Serial.print(F(" Process Gain:      "));  Serial.println(_Ku, 4);
    Serial.print(F(" Dead Time Sec:     "));  Serial.println(_td, 5);
    Serial.print(F(" Tau Sec:           "));  Serial.println(_Tu, 5);
    Serial.println();

    // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png
    float controllability = _Tu / _td + epsilon;
    if (controllability > 99.9) controllability = 99.9;
    Serial.print(F(" Tau/Dead Time:     "));  Serial.print(controllability, 1);
    if (controllability > 0.75) Serial.println(F(" (easy to control)"));
    else if (controllability > 0.25) Serial.println(F(" (average controllability)"));
    else Serial.println(F(" (difficult to control)"));

    // check “best practice” rule that sample time should be ≥ 10 times per process time constant
    // https://controlguru.com/sample-time-is-a-fundamental-design-and-tuning-specification/
    float sampleTimeCheck = _Tu / (_samplePeriodUs * 0.000001f);
    Serial.print(F(" Tau/Sample Period: "));  Serial.print(sampleTimeCheck, 1);
    if (sampleTimeCheck >= 10) Serial.println(F(" (good sample rate)"));
    else Serial.println(F(" (low sample rate)"));
    Serial.println();

    Serial.print(F(" Kp: "));  Serial.println(_kp, 4);
    Serial.print(F(" Ki: "));  Serial.println(_ki, 4);
    Serial.print(F(" Kd: "));  Serial.println(_kd, 4);
    Serial.println();
  }
  if (_serialMode == printPIDTUNER && (_action == direct5T || _action == reverse5T)) {
    Serial.println();
    Serial.print(F("Kp ")); Serial.println(_kp, 4);
    Serial.print(F("Ti ")); Serial.println(1.0f / _ki, 4);
    Serial.print(F("Td ")); Serial.println(1.0f / _kd, 4);
    sampleCount++;
  }
}

// Query functions

void sTune::GetAutoTunings(float * kp, float * ki, float * kd) {
  *kp = _kp;
  *ki = _ki;
  *kd = _kd;
}

float sTune::GetKp() {
  if (_tuningMethod == ZN_PID)                _kp = (1.2f * _TuMin) / (_Ko * _tdMin);
  else if (_tuningMethod == ZN_Half_PID)      _kp = (0.6f * _TuMin) / (_Ko * _tdMin);
  else if (_tuningMethod == Damped_PID)       _kp = (0.66f * _TuMin) / (_Ko * _tdMin);
  else if (_tuningMethod == NoOvershoot_PID)  _kp = (0.6f / _Ku) * (_TuMin / _tdMin);
  else if (_tuningMethod == CohenCoon_PID)    _kp = _Ko * (1.33f + (_R / 4.0f));
  else if (_tuningMethod == ZN_PI)            _kp = (0.9f * _TuMin) / (_Ko * _tdMin);
  else if (_tuningMethod == ZN_Half_PI)       _kp = (0.45f * _TuMin) / (_Ko * _tdMin);
  else if (_tuningMethod == Damped_PI)        _kp = (0.495f * _TuMin) / (_Ko * _tdMin);
  else if (_tuningMethod == NoOvershoot_PI)   _kp = (0.35f / _Ku) * (_TuMin / _tdMin);
  else                                        _kp = _Ko * (0.9f + (_R / 12.0f)); // CohenCoon_PI
  return _kp;
}

float sTune::GetKi() {
  if (_tuningMethod == ZN_PID)                _ki = 1 / (2.0f * _tdMin);
  else if (_tuningMethod == ZN_Half_PID)      _ki = 1 / (4.0f * _tdMin);
  else if (_tuningMethod == Damped_PID)       _ki = 1 / (_TuMin / 3.6f);
  else if (_tuningMethod == NoOvershoot_PID)  _ki = 1 / (_TuMin);
  else if (_tuningMethod == CohenCoon_PID)    _ki = 1 / (_tdMin * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));
  else if (_tuningMethod == ZN_PI)            _ki = 1 / (3.3333f * _tdMin);
  else if (_tuningMethod == ZN_Half_PI)       _ki = 1 / (6.6667f * _tdMin);
  else if (_tuningMethod == Damped_PI)        _ki = 1 / (_TuMin / 2.6f);
  else if (_tuningMethod == NoOvershoot_PI)   _ki = 1 / (1.2f * _TuMin);
  else                                        _ki = 1 / (_tdMin * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R))); // CohenCoon_PI
  return _ki;
}

float sTune::GetKd() {
  if (_tuningMethod == ZN_PID)                _kd = 1 / (0.5f * _tdMin);
  else if (_tuningMethod == ZN_Half_PID)      _kd = 1 / (1.0f * _tdMin);
  else if (_tuningMethod == Damped_PID)       _kd = 1 / (_TuMin / 9.0f);
  else if (_tuningMethod == NoOvershoot_PID)  _kd = 1 / (0.5f * _tdMin);
  else if (_tuningMethod == CohenCoon_PID)    _kd = 1 / ((4.0f * _tdMin) / (11.0f + (2.0f * _R)));
  else                                        _kd = 0.0f; // PI controller
  return _kd;
}

float sTune::GetProcessGain() {
  return _Ku;
}

float sTune::GetDeadTime() {
  return _td;
}

float sTune::GetTau() {
  return _Tu;
}

float sTune::GetTimeIntegral() {
  return 1.0f / _ki;
}

float sTune::GetTimeDerivative() {
  return 1.0f / _kd;
}

uint8_t sTune::GetControllerAction() {
  return static_cast<uint8_t>(_action);
}

uint8_t sTune::GetSerialMode() {
  return static_cast<uint8_t>(_serialMode);
}

uint8_t sTune::GetTuningMethod() {
  return static_cast<uint8_t>(_tuningMethod);
}
