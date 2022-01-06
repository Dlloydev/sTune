/********************************************************************
   QuickPID Autotune Example (using Temperature Control Lab)
   http://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl
 ********************************************************************/

#include <sTune.h>
#include <QuickPID.h>

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

float Input = 0, Output = 0, Setpoint = 30, Kp = 0, Ki = 0, Kd = 0;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::reverse);

sTune tuner = sTune(&Input, &Output, tuner.zieglerNicholsPID, tuner.directIP, tuner.printALL);
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
  analogReference(EXTERNAL); // AVR
  Serial.begin(115200);
  analogWrite(outputPin, outputStart);
  tuner.Configure(outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {
  switch (tuner.Run()) {  // active while sTune is testing (non-blocking)
    case tuner.inOut:
      Input = (analogRead(inputPin) / mvResolution) - bias;
      analogWrite(outputPin, Output);
      break;

    case tuner.tunings:                                         // active just once when sTune is done
      tuner.SetAutoTunings(&Kp, &Ki, &Kd);                      // sketch variables are updated by sTune
      myPID.SetMode(myPID.Control::automatic);                  // the PID is turned on (automatic)
      myPID.SetSampleTimeUs((testTimeSec * 1000000) / samples); // PID sample rate
      myPID.SetTunings(Kp, Ki, Kd);                             // update PID with the new tunings
      break;

    case tuner.runPid:  // this case runs continuously after case "tunings" (non-blocking)
      Input = (analogRead(inputPin) / mvResolution) - bias;
      myPID.Compute();
      analogWrite(outputPin, Output);
      break;
  }
  // put your main code here, to run repeatedly
}
