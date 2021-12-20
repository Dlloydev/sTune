#include "Arduino.h"
#include "sTan.h"
#include "sTune.h"

sTan::sTan() {}
sTan tangent = sTan();

sTune::sTune() {
  _input = nullptr;
  _output = nullptr;
  sTune::Reset();
}

sTune::sTune(float *input, float *output, TuningRule tuningRule, Action action, SerialMode serialMode) {
  _input = input;
  _output = output;
  _tuningRule = tuningRule;
  _action = action;
  _serialMode = serialMode;
  sTune::Reset();
}

void sTune::Reset() {
  _Ku = 0.0;
  _Tu = 0.0;
  _td = 0.0;
  _kp = 0.0;
  _ki = 0.0;
  _kd = 0.0;
  _tunerStatus = inOut;
}

void sTune::Configure(const float outputStart, const float outputStep, const uint32_t testTimeSec, const uint32_t settleTimeSec, const uint16_t samples) {
  _outputStart = outputStart;
  _outputStep = outputStep;
  _testTimeSec = testTimeSec;
  _settleTimeSec = settleTimeSec;
  _samples = samples;
  _tunerStatus = inOut;
  _bufferSize = (uint16_t)(_samples * 0.1);
  _samplePeriodUs = (float)(_testTimeSec * 1000000.0) / _samples;
  _tangentPeriodUs = _samplePeriodUs * (_bufferSize - 1);
  _settlePeriodUs = (float)(_settleTimeSec * 1000000.0);
  tangent.begin(_bufferSize);

}

uint8_t sTune::Run() {

  uint32_t usNow = micros();
  uint32_t usElapsed = usNow - usPrev;
  uint32_t settleElapsed = usNow - settlePrev;

  switch (_tunerStatus) {

    case inOut:
      _tunerStatus = test;
      return test;
      break;

    case test: // run inflection point test method

      if (settleElapsed >= _settlePeriodUs) {

        if (sampleCount == 0) *_output = _outputStep;

        if (usElapsed >= _samplePeriodUs) {
          usPrev = usNow;

          if (sampleCount <= _samples) {
            us = usNow - usStart;
            float lastPvAvg = pvAvg;
            pvInst = *_input;
            pvAvg = tangent.avgVal(pvInst);

            float pvDiff = pvAvg - lastPvAvg;
            if (pvDiff > epsilon && pvDiff < pvRes) pvRes = pvDiff; // determines input resolution

            //initialize at first sample
            if (sampleCount == 0) {
              tangent.init(pvInst);
              pvAvg = pvInst;
              pvStart = pvInst;
              pvRes = pvInst;
              pvIp = 0;
              pvMax = 0;
              pvPk = 0;
              slopeIp = 0;
              pvTangent = 0;
              pvTangentPrev = 0;
              ipUs = 0;
              dtCount = 0;
              ipCount = 0;
              pvPkCount = 0;
              _Tu = 0;
              _td = 0;
              usStart = usNow;
              us = 0;
            }

            //To determine the point of inflection, we'll use a sliding tangent line.
            //https://en.wikipedia.org/wiki/Inflection_point#/media/File:Animated_illustration_of_inflection_point.gif
            pvTangent = pvAvg - tangent.startVal();

            //apparent dead time
            if (pvTangent - pvTangentPrev > 0 + epsilon) {
              dtCount <<= 1;
              dtCount |= 0x01;
              if (dtCount == 0x1F) {
                _td = us * 0.000001;
              }
            }

            //Test for maximum tangent rate of rise (point of inflection)
            if (pvTangent > slopeIp + epsilon) {  // tangent increased
              ipCount = 0;
              slopeIp = pvTangent;
              ipUs = us;
              pvIp = pvAvg;
            } else if (pvTangent < 0 + epsilon) ipCount = 0;  // flat or negative tangent

            // continue testing
            ipCount++;

            if ((_action == directIp || _action == reverseIp) && (ipCount == ((int)(_samples / 16)))) { // reached inflection point
              sampleCount = _samples;

              // apparent pvMax  Re: "Step response with arbitrary initial conditions" https://en.wikipedia.org/wiki/Time_constant
              pvMax = pvIp + slopeIp * kexp;

              //apparent tangent from pvStart to pvMax crossing points
              _Tu = (((pvMax - pvStart) / slopeIp) * _tangentPeriodUs * 0.000001) - _td;
            }

            if (_action == direct5t || _action == reverse5t) { // continue testing to maximum input
              sampleCount = 1;

              if (us > _testTimeSec * 100000) {    // 10% of testTimeSec has elapsed
                //if (pvAvg > pvPk + 2 * pvRes) {  // if pvAvg has incresed 2 resolution values past the peak
                if (pvAvg > pvPk) {
                  pvPk = pvAvg + 2 * pvRes;        // set a new boosted peak
                  pvPkCount = 0;                   // reset the "below peak" counter
                } else {
                  pvPkCount++;                     // count up while pvAvg is below the boosted peak
                }

                if (pvPkCount == ((uint16_t)(1.2 * _bufferSize))) {  // test done (assume 3.5τ)
                  pvPkCount++;
                  sampleCount = _samples;
                  pvMax = pvAvg + (pvInst - pvStart) * 0.03;  // add 3% to pvAvg
                  _Tu = (us * 0.000001 * 0.286) - _td;        // multiply by 0.286 for τ
                }
              }
            }

            if (sampleCount == _samples) {

              // calculate the process gain
              _Ku = (pvMax - pvStart) / (_outputStep - _outputStart);

              // apply tuning rule constants
              if (_tuningRule == amigofPid) {
                (_td < 0.1) ? _td = 0.1 : _td = _td;
                _kp = (0.2 + 0.45 * (_Tu / _td)) / _Ku;
                float Ti = (((0.4 * _td) + (0.8 * _Tu)) / (_td + (0.1 * _Tu)) * _td);
                float Td = (0.5 * _td * _Tu) / ((0.3 * _td) + _Tu);
                _ki = _kp / Ti;
                _kd = Td * _kp;
              } else { //other rules
                _kp = (float)(RulesContants[_tuningRule][0] * 0.001) * _Ku;
                _ki = (float)((RulesContants[_tuningRule][1] * 0.001) * _Ku) / (_Tu / 60.0);
                _kd = (float)((RulesContants[_tuningRule][2] * 0.001) * _Ku) * (_Tu / 60.0);
              }
              sTune::printResults();
              *_output = _outputStart;
              _tunerStatus = tunings;
              return tunings;
              break;
            }
            sTune::printTestRun();
            pvTangentPrev = pvTangent;
            sampleCount++;

          } else _tunerStatus = tunings;
        }
      } else {  // settling
        if (usElapsed >= _samplePeriodUs) {
          usPrev = usNow;
          pvInst = *_input;
          if (_serialMode == printAll) {
            Serial.print(F(" Sec: "));     Serial.print((float)((_settlePeriodUs - settleElapsed) * 0.000001), 5);
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

void sTune::SetAutoTunings(float * kp, float * ki, float * kd, float * Ku, float * Tu, float * td) {
  *kp = _kp;
  *ki = _ki;
  *kd = _kd;
  *Ku = _Ku;
  *Tu = _Tu;
  *td = _td;
}

void sTune::printTestRun() {
  if (_serialMode == printAll && (sampleCount < _samples)) {
    Serial.print(F(" Sec: "));                        Serial.print(us * 0.000001, 5);
    Serial.print(F("  pvAvg: "));                     Serial.print(pvAvg, 4);
    //Serial.print(F("  ipCount: "));                   Serial.print(ipCount);    // for diagnostics
    //Serial.print(F("  pvRes: "));                     Serial.print(pvRes, 4);   // for diagnostics
    //Serial.print(F("  pvPkCount: "));                 Serial.print(pvPkCount);  // for diagnostics
    Serial.print(F("  pvTangent: "));                 Serial.print(pvTangent, 4);
    if (pvTangent - pvTangentPrev > 0 + epsilon)      Serial.println(F(" ↗"));
    else if (pvTangent - pvTangentPrev < 0 - epsilon) Serial.println(F(" ↘"));
    else                                              Serial.println(F(" →"));
  } else if (_serialMode == plot) {
    Serial.print(F("pvAvg ")); Serial.println(pvAvg, 4);
  }
}

void sTune::printResults() {
  if (_serialMode == printAll || _serialMode == printSummary) {
    Serial.println();
    Serial.print(F(" Output Start:    "));  Serial.println(_outputStart);
    Serial.print(F(" Output Step:     "));  Serial.println(_outputStep);
    Serial.print(F(" Sample Sec:      "));  Serial.println(_samplePeriodUs * 0.000001, 5);
    Serial.println();
    if (_action == directIp || _action == reverseIp) {
      Serial.print(F(" Ip Sec:          "));  Serial.println(ipUs * 0.000001, 5);
      Serial.print(F(" Ip Slope:        "));  Serial.print(slopeIp, 4);  Serial.println(F(" ↑"));
      Serial.print(F(" Ip Pv:           "));  Serial.println(pvIp, 4);
    }
    Serial.print(F(" Pv Start:        "));  Serial.println(pvStart, 4);
    Serial.print(F(" Pv Max:          "));  Serial.println(pvMax, 4);
    Serial.print(F(" Pv Diff:         "));  Serial.println(pvMax - pvStart, 4);
    Serial.println();
    Serial.print(F(" Process Gain:    "));  Serial.println(_Ku, 4);
    Serial.print(F(" Dead Time Sec:   "));  Serial.println(_td, 5);
    Serial.print(F(" Tau Sec:         "));  Serial.println(_Tu, 5);
    // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png
    float controllability = _Tu / _td + epsilon;
    if (controllability > 99.9) controllability = 99.9;
    if (controllability > 0.75) Serial.print(F(" Easy to control:"));
    else if (controllability > 0.25) Serial.print(F(" Average controllability:"));
    else Serial.print(F(" Difficult to control:"));
    Serial.print(F(" Tau/Dead Time = "));   Serial.println(controllability, 1);
    Serial.println();
    Serial.print(F(" Kp:              "));  Serial.println(_kp, 4);
    Serial.print(F(" Ki:              "));  Serial.println(_ki, 4);
    Serial.print(F(" Kd:              "));  Serial.println(_kd, 4);
    Serial.println();
  }
}
