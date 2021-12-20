// TODO:
// autotune in reverse direction

// sTune: PID Autotuner using "S" curve point of inflection method
// Complete test duration for tuning parameters in approximately 0.5τ

#include "sTune.h"
#include "sTan.h"
#include "QuickPID.h"

const uint8_t inputPin = 0;
const uint8_t outputPin = 3;

//user settings
const uint32_t testTimeSec =  300;
const float    outputStart =    0;
const float    outputStep =    5;  //50
uint16_t       samples =      500;
const uint32_t settleTimeSec = 15;

//input constants
const float mvResolution = 3300 / 1024.0;
const float bias = 50;

//               50
float Setpoint = 30, Input = 0, Output = 0, kp = 0, ki = 0, kd = 0, ku = 0, tu = 0, td = 0;
uint16_t plotCount = 0;

QuickPID myPID(&Input, &Output, &Setpoint, kp, ki, kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

sTune tuner = sTune(&Input, &Output, tuner.zieglerNicholsPid, tuner.directIp, tuner.printAll);
/*                                         zieglerNicholsPi         directIp        serialOff
                                           zieglerNicholsPid        direct5t        printAll
                                           tyreusLuybenPi           reverseIp       printSummary
                                           tyreusLuybenPid          reverse5t       printDebug
                                           cianconeMarlinPi                         plot
                                           cianconeMarlinPi
                                           amigofPid
                                           pessenIntegralPid
                                           someOvershootPid
                                           noOvershootPid
*/
void setup() {
  analogReference(EXTERNAL);
  Serial.begin(115200);
  analogWrite(outputPin, outputStart);
  tuner.Configure(outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {

  switch (tuner.Run()) {
    case tuner.inOut:
      Input = (analogRead(inputPin) / mvResolution) - bias;
      analogWrite(outputPin, Output);
      break;

    case tuner.tunings:
      tuner.SetAutoTunings(&kp, &ki, &kd, &ku, &tu, &td);
      myPID.SetMode(myPID.Control::automatic);
      myPID.SetSampleTimeUs((testTimeSec * 1000000) / samples);
      myPID.SetTunings(kp, ki, kd);
      break;

    case tuner.runPid:
      Input = (analogRead(inputPin) / mvResolution) - bias;
      myPID.Compute();
      analogWrite(outputPin, Output);
      if ((plotCount & 0x000F) == 10) {   //plot every 10th sample (6 sec)
        Serial.print(" Setpoint:");  Serial.print(Setpoint);      Serial.print(",");
        Serial.print(" Input:");     Serial.print(Input);         Serial.print(",");
        Serial.print(" Output:");    Serial.print(Output * 0.5);  Serial.println(",");
      }
      plotCount++;
      break;
  }
}
