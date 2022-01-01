/********************************************************************
   PID_v1 Autotune Example (using Temperature Control Lab)
   http://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl
 ********************************************************************/

#include <sTune.h>
#include <PID_v1.h>

const uint8_t inputPin = 0;
const uint8_t outputPin = 3;

//user settings
const uint32_t testTimeSec = 300;
const float outputStart = 0;
const float outputStep = 20;
const uint16_t samples = 500;
const uint32_t settleTimeSec = 10;

//input constants
const float mvResolution = 3300 / 1024.0f;
const float bias = 50;

double Input = 0, Output = 0, Setpoint = 30; //myPID
float input = 0, output = 0, kp = 0, ki = 0, kd = 0; //tuner

PID myPID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT);

sTune tuner = sTune(&input, &output, tuner.zieglerNicholsPID, tuner.directIP, tuner.printALL);
/*                                         zieglerNicholsPI         directIP        serialOFF
                                           zieglerNicholsPID        direct5T        printALL
                                           tyreusLuybenPI           reverseIP       printSUMMARY
                                           tyreusLuybenPID          reverse5T       printDEBUG
                                           cianconeMarlinPI                         printPIDTUNER
                                           cianconeMarlinPID                        serialPLOTTER
                                           amigofPID
                                           pessenIntegralPID
                                           someOvershootPID
                                           noOvershootPID
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
      input = (analogRead(inputPin) / mvResolution) - bias;
      analogWrite(outputPin, output);
      break;

    case tuner.tunings:
      tuner.SetAutoTunings(&kp, &ki, &kd);
      myPID.SetMode(AUTOMATIC);
      myPID.SetSampleTime((testTimeSec * 1000) / samples);
      myPID.SetTunings(kp, ki, kd);
      break;

    case tuner.runPid:
      Input = (analogRead(inputPin) / mvResolution) - bias;
      myPID.Compute();
      analogWrite(outputPin, Output);
      break;
  }
}
