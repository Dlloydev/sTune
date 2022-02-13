/****************************************************************************************
   sTune Library for Arduino - Version 2.4.0
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
  _tunerStatus = test;
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
  pvInstRes = pvInst;
  ipCount = 0;
  plotCount = 0;
  sampleCount = 0;
  pvPkCount = 0;
}

void sTune::Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,
                      uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples) {
  sTune::Reset();
  _inputSpan = inputSpan;
  eStop = inputSpan;
  _outputSpan = outputSpan;
  _outputStart = outputStart;
  _outputStep = outputStep;
  _testTimeSec = testTimeSec;
  _settleTimeSec = settleTimeSec;
  _samples = samples;
  _bufferSize = (uint16_t)(_samples * 0.06);
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

    case sample:
      _tunerStatus = test;
      return test;
      break;

    case test: // run inflection point test method
      if (pvInst > eStop && !eStopAbort) {
        sTune::Reset();
        sampleCount = _samples + 1;
        eStopAbort = 1;
        Serial.println(F(" ABORT: pvInst > eStop"));
        break;
      }
      if (settleElapsed >= _settlePeriodUs) { // if settling period has expired
        if (sampleCount == 1) *_output = _outputStep;
        if (usElapsed >= _samplePeriodUs) { // ready to process a sample
          usPrev = usNow;
          if (sampleCount <= _samples) { // continue testing

            // get pvInst and pvAvg (buffer) resolution
            float lastPvInst = pvInst;
            float lastPvAvg = pvAvg;
            pvInst = *_input;
            pvAvg = tangent.avgVal(pvInst);
            float pvInstResolution = fabs(pvInst - lastPvInst);
            float pvAvgResolution = fabs(pvAvg - lastPvAvg);
            if (pvInstResolution > epsilon && pvInstResolution < pvInstRes) pvInstRes = pvInstResolution;
            if (pvAvgResolution > epsilon && pvAvgResolution < pvAvgRes) pvAvgRes = pvAvgResolution;

            if (sampleCount == 0) { // initialize at first sample
              tangent.init(pvInst);
              pvAvg = pvInst;
              pvInstResolution = 0.0f;
              pvAvgResolution = 0.0f;
              pvInstRes = pvInst;
              pvAvgRes = pvInst;
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
              (pvAvg > pvStart + pvInstRes + epsilon) ?  dt = true : dt = false;
            } else { // reverse
              (pvAvg < pvStart - pvInstRes - epsilon) ?  dt = true : dt = false;
            }
            if (!_td && dt) _td = us * 0.000001f;

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
            }
            ipCount++;
            if ((_action == directIP || _action == reverseIP) && (ipCount == ((uint16_t)(_samples / 16)))) { // reached inflection point
              sampleCount = _samples;
              ipUs = us;
              pvIp = pvAvg;

              // apparent pvMax  Re: "Step response with arbitrary initial conditions" https://en.wikipedia.org/wiki/Time_constant
              pvMax = pvIp + (slopeIp * kexp);

              //apparent tangent from pvStart to pvMax crossing points
              _Tu = (((pvMax - pvStart) / slopeIp) * _tangentPeriodUs * 0.000001f) - _td;
            }

            if (_action == direct5T || _action == reverse5T) { // continue testing to maximum input
              if (sampleCount >= _samples - 1) sampleCount = _samples - 2;
              if (us > _testTimeSec * 100000) {  // 10% of testTimeSec has elapsed
                if (pvAvg > pvPk) {
                  pvPk = pvAvg + (_bufferSize * 0.2f * pvAvgRes); // set a new boosted peak
                  pvPkCount = 0;  // reset the "below peak" counter
                } else {
                  pvPkCount++;  // count up while pvAvg is below the boosted peak
                }
                if (pvPkCount == ((uint16_t)(1.2 * _bufferSize))) {  // test done
                  pvPkCount++;
                  sampleCount = _samples;
                  pvMax = pvAvg + (pvInst - pvStart) * 0.05f;  // assume 3τ, so increase pvMax by 5%
                  _Tu = (us * 1.6667 * 0.000001f * 0.286f) - _td; // scale us to 5τ in seconds, then multiply by 0.286 for τ
                }
              }
            }

            if (sampleCount == _samples) {  // testing complete
              _R = _td / _Tu;

              // process gain
              _Ku =  fabs(((pvMax - pvStart) / _inputSpan) / ((_outputStep - _outputStart) / _outputSpan));

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
          sampleCount++;
          _tunerStatus = sample;
          return sample;
        }

      } else {  // settling
        if (usElapsed >= _samplePeriodUs && !eStopAbort) {
          *_output = _outputStart;
          usPrev = usNow;
          pvInst = *_input;
          if (_serialMode == printALL || _serialMode == printDEBUG) {
            Serial.print(F(" sec: "));     Serial.print((float)((_settlePeriodUs - settleElapsed) * 0.000001f), 4);
            Serial.print(F("  out: ")); Serial.print(*_output);
            Serial.print(F("  pv: "));     Serial.print(pvInst, 3);
            Serial.println(F("  settling  ⤳⤳"));
          }
          _tunerStatus = sample;
          return sample;
        }
      }
      break;

    case tunings:
      _tunerStatus = timerPid;
      return timerPid;
      break;

    case runPid:
      if (pvInst > eStop && !eStopAbort) {
        sTune::Reset();
        sampleCount = _samples + 1;
        eStopAbort = 1;
        Serial.println(F(" ABORT: pvInst > eStop"));
      }
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

void sTune::SetEmergencyStop(float e_Stop) {
  eStop = e_Stop;
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
  if (sampleCount < _samples) {
    if (plotCount == 0 || plotCount >= everyNth) {
      plotCount = 1;
      Serial.print(us * 0.000001f, 4);  Serial.print(F(", "));
      Serial.print(*_output);           Serial.print(F(", "));
      Serial.println(pvAvg);
    } else plotCount++;
  }
}

void sTune::plotter(float input, float output, float setpoint, float outputScale, uint8_t everyNth) {
  if (plotCount >= everyNth) {
    plotCount = 1;
    Serial.print(F("Setpoint:"));  Serial.print(setpoint);              Serial.print(F(", "));
    Serial.print(F("Input:"));     Serial.print(input);                 Serial.print(F(", "));
    Serial.print(F("Output:"));    Serial.print(output * outputScale);  Serial.print(F(","));
    Serial.println();
  } else plotCount++;
}

void sTune::printTestRun() {

  if (sampleCount < _samples) {
    if (_serialMode == printALL || _serialMode == printDEBUG) {
      Serial.print(F(" sec: "));           Serial.print(us * 0.000001f, 4);
      Serial.print(F("  out: "));          Serial.print(*_output);
      Serial.print(F("  pv: "));           Serial.print(pvInst, 3);
      //Serial.print(F("  pvAvg: "));      Serial.print(pvAvg, 3);
      if (_serialMode == printDEBUG && (_action == direct5T || _action == reverse5T)) {
        Serial.print(F("  pvPk: "));       Serial.print(pvPk, 3);
        Serial.print(F("  pvPkCount: "));  Serial.print(pvPkCount);
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);
      }
      if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);
      }
      Serial.print(F("  tan: "));                       Serial.print(pvTangent, 3);
      if (pvInst > 0.9f * eStop)                        Serial.print(F(" ⚠"));
      if (pvTangent - pvTangentPrev > 0 + epsilon)      Serial.println(F(" ↗"));
      else if (pvTangent - pvTangentPrev < 0 - epsilon) Serial.println(F(" ↘"));
      else                                              Serial.println(F(" →"));
    }
  }
}

void sTune::printTunings() {
  Serial.print(F(" Tuning Method: "));
  if (_tuningMethod == ZN_PID) Serial.println(F("ZN_PID"));
  else if (_tuningMethod == DampedOsc_PID) Serial.println(F("Damped_PID"));
  else if (_tuningMethod == NoOvershoot_PID) Serial.println(F("NoOvershoot_PID"));
  else if (_tuningMethod == CohenCoon_PID) Serial.println(F("CohenCoon_PID"));
  else if (_tuningMethod == Mixed_PID) Serial.println(F("Mixed_PID"));
  else if (_tuningMethod == ZN_PI) Serial.println(F("ZN_PI"));
  else if (_tuningMethod == DampedOsc_PI) Serial.println(F("Damped_PI"));
  else if (_tuningMethod == NoOvershoot_PI) Serial.println(F("NoOvershoot_PI"));
  else if (_tuningMethod == CohenCoon_PI) Serial.println(F("CohenCoon_PI"));
  else Serial.println(F("Mixed_PI"));
  Serial.print(F("  Kp: ")); Serial.println(sTune::GetKp(), 3);
  Serial.print(F("  Ki: ")); Serial.print(sTune::GetKi(), 3); Serial.print(F("  Ti: ")); Serial.println(sTune::GetTi(), 3);
  Serial.print(F("  Kd: ")); Serial.print(sTune::GetKd(), 3); Serial.print(F("  Td: ")); Serial.println(sTune::GetTd(), 3);
  Serial.println();
}

void sTune::printResults() {
  if (_serialMode == printALL || _serialMode == printDEBUG || _serialMode == printSUMMARY) {
    Serial.println();
    Serial.print(F(" Controller Action: "));
    if (_action == directIP) Serial.println(F("directIP"));
    else if (_action == direct5T) Serial.println(F("direct5T"));
    else if (_action == reverseIP) Serial.println(F("reverseIP"));
    else Serial.println(F("reverse5T"));
    Serial.println();
    Serial.print(F(" Output Start:      "));  Serial.println(_outputStart);
    Serial.print(F(" Output Step:       "));  Serial.println(_outputStep);
    Serial.print(F(" Sample Sec:        "));  Serial.println(_samplePeriodUs * 0.000001f, 4);
    Serial.println();
    if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {
      Serial.print(F(" Ip Sec:            "));  Serial.println(ipUs * 0.000001f, 4);
      Serial.print(F(" Ip Slope:          "));  Serial.print(slopeIp, 3);
      if (_action == directIP || _action == direct5T) Serial.println(F(" ↑"));
      else  Serial.println(F(" ↓"));
      Serial.print(F(" Ip Pv:             "));  Serial.println(pvIp, 3);
    }
    Serial.print(F(" Pv Start:          "));  Serial.println(pvStart, 3);
    if (_action == directIP || _action == direct5T) Serial.print(F(" Pv Max:            "));
    else Serial.print(F(" Pv Min:            "));
    Serial.println(pvMax, 3);
    Serial.print(F(" Pv Diff:           "));  Serial.println(pvMax - pvStart, 3);
    Serial.println();
    Serial.print(F(" Process Gain:      "));  Serial.println(_Ku, 3);
    Serial.print(F(" Dead Time Sec:     "));  Serial.println(_td, 3);
    Serial.print(F(" Tau Sec:           "));  Serial.println(_Tu, 3);
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
    sTune::printTunings();
    sampleCount++;
  }
}

// Query functions

void sTune::GetAutoTunings(float * kp, float * ki, float * kd) {
  *kp = _kp;
  *ki = _ki;
  *kd = _kd;
}

// https://blog.opticontrols.com/archives/477

float sTune::GetKp() {
  float znPid = ((1.2f * _Tu) / (_Ku * _td)) / 2;
  float doPid = (0.66f * _Tu) / (_Ku * _td);
  float noPid = (0.6f / _Ku) * (_Tu / _td);
  float ccPid = _Ku * (1.33f + (_R / 4.0f));
  float znPi = ((0.9f * _Tu) / (_Ku * _td)) / 2;
  float doPi = (0.495f * _Tu) / (_Ku * _td);
  float noPi = (0.35f / _Ku) * (_Tu / _td);
  float ccPi = _Ku * (0.9f + (_R / 12.0f));
  if (_tuningMethod == ZN_PID)                _kp = znPid;
  else if (_tuningMethod == DampedOsc_PID)    _kp = doPid;
  else if (_tuningMethod == NoOvershoot_PID)  _kp = noPid;
  else if (_tuningMethod == CohenCoon_PID)    _kp = ccPid;
  else if (_tuningMethod == Mixed_PID)        _kp = 0.25f * (znPid + doPid + noPid + ccPid);
  else if (_tuningMethod == ZN_PI)            _kp = znPi;
  else if (_tuningMethod == DampedOsc_PI)     _kp = doPi;
  else if (_tuningMethod == NoOvershoot_PI)   _kp = noPi;
  else if (_tuningMethod == CohenCoon_PI)     _kp = ccPi;
  else                                        _kp = 0.25f * (znPi + doPi + noPi + ccPi); // Mixed_PI
  return _kp;
}

float sTune::GetKi() {
  float znPid = 1 / (2.0f * _td);
  float doPid = 1 / (_Tu / 3.6f);
  float noPid = 1 / (_Tu);
  float ccPid = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));
  float znPi = 1 / (3.3333f * _td);
  float doPi = 1 / (_Tu / 2.6f);
  float noPi = 1 / (1.2f * _Tu);
  float ccPi = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));

  if (_tuningMethod == ZN_PID)                _ki = znPid;
  else if (_tuningMethod == DampedOsc_PID)    _ki = doPid;
  else if (_tuningMethod == NoOvershoot_PID)  _ki = noPid;
  else if (_tuningMethod == CohenCoon_PID)    _ki = ccPid;
  else if (_tuningMethod == Mixed_PID)        _ki = 0.25f * (znPid + doPid + noPid + ccPid);
  else if (_tuningMethod == ZN_PI)            _ki = znPi;
  else if (_tuningMethod == DampedOsc_PI)     _ki = doPi;
  else if (_tuningMethod == NoOvershoot_PI)   _ki = noPi;
  else if (_tuningMethod == CohenCoon_PI)     _ki = ccPi;
  else                                        _ki = 0.25f * (znPi + doPi + noPi + ccPi); // Mixed_PI
  return _ki;
}

float sTune::GetKd() {
  float znPid = 1 / (0.5f * _td);
  float doPid = 1 / (_Tu / 9.0f);
  float noPid = 1 / (0.5f * _td);
  float ccPid = 1 / ((4.0f * _td) / (11.0f + (2.0f * _R)));
  if (_tuningMethod == ZN_PID)                _kd = znPid;
  else if (_tuningMethod == DampedOsc_PID)    _kd = doPid;
  else if (_tuningMethod == NoOvershoot_PID)  _kd = noPid;
  else if (_tuningMethod == CohenCoon_PID)    _kd = ccPid;
  else if (_tuningMethod == Mixed_PID)        _kd = 0.25f * (znPid + doPid + noPid + ccPid);
  else                                        _kd = 0.0f; // PI controller
  return _kd;
}

float sTune::GetTi() {
  return _kp / _ki;
}

float sTune::GetTd() {
  if (_tuningMethod == ZN_PID ||
      _tuningMethod == DampedOsc_PID ||
      _tuningMethod == NoOvershoot_PID ||
      _tuningMethod == CohenCoon_PID ||
      _tuningMethod == Mixed_PID) {
    return _kp / _kd;
  }
  else return 0;
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

uint8_t sTune::GetControllerAction() {
  return static_cast<uint8_t>(_action);
}

uint8_t sTune::GetSerialMode() {
  return static_cast<uint8_t>(_serialMode);
}

uint8_t sTune::GetTuningMethod() {
  return static_cast<uint8_t>(_tuningMethod);
}

float sTune::softPwm(const uint8_t relayPin, float input, float output, float setpoint, uint32_t windowSize, uint8_t debounce) {

  // software PWM timer
  uint32_t msNow = millis();
  static uint32_t  windowStartTime, nextSwitchTime;
  if (msNow - windowStartTime >= windowSize) {
    windowStartTime = msNow;
  }
  // SSR optimum AC half-cycle controller
  static float optimumOutput;
  static bool reachedSetpoint;

  if (input > setpoint) reachedSetpoint = true;
  if (reachedSetpoint && !debounce && setpoint > 0 && input > setpoint) optimumOutput = output - 8;
  else if (reachedSetpoint && !debounce && setpoint > 0 && input < setpoint) optimumOutput = output + 8;
  else  optimumOutput = output;
  if (optimumOutput < 0) optimumOutput = 0;

  // PWM relay output
  static bool relayStatus;
  if (!relayStatus && optimumOutput > (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = true;
      digitalWrite(relayPin, HIGH);
    }
  } else if (relayStatus && optimumOutput < (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = false;
      digitalWrite(relayPin, LOW);
    }
  }
  return optimumOutput;
}
