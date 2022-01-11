/********************************************************************
   Autotune Plotter Example (using Temperature Control Lab)
   http://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl
 ********************************************************************/

#include <sTune.h>
#include <QuickPID.h>

// pins
const uint8_t inputPin = 0;
const uint8_t outputPin = 3;

// test setup
uint32_t testTimeSec = 300;
const uint16_t samples = 500;
uint32_t settleTimeSec = 10;
const float inputSpan = 80;
const float outputSpan = 255;
float outputStart = 0;
float outputStep = 25;

// temperature
const float mvResolution = 3300 / 1024.0f;
const float bias = 50;

// test variables
float Input = 0, Output = 0, Setpoint = 30, Kp = 0, Ki = 0, Kd = 0;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.serialPLOTTER);
/*                                         ZN_PID        directIP        serialOFF
                                           ZN_Half_PID   direct5T        printALL
                                           Damped_PID    reverseIP       printSUMMARY
                                           NoOvershoot_PID reverse5T     printDEBUG
                                           CohenCoon_PID                 printPIDTUNER
                                           ZN_PI                         serialPLOTTER
                                           ZN_Half_PI
                                           Damped_PI
                                           NoOvershoot_PI
                                           CohenCoon_PI
*/
void setup() {
  analogReference(EXTERNAL); // AVR
  Serial.begin(115200);
  analogWrite(outputPin, outputStart);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {
  switch (tuner.Run()) {  // active while sTune is testing
    case tuner.inOut:
      Input = (analogRead(inputPin) / mvResolution) - bias;
      analogWrite(outputPin, Output);
      break;

    case tuner.tunings:                                          // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd);                       // sketch variables updated by sTune
      myPID.SetMode(myPID.Control::automatic);                   // the PID is turned on (automatic)
      myPID.SetSampleTimeUs((testTimeSec * 1000000) / samples);  // PID sample rate
      myPID.SetTunings(Kp, Ki, Kd);                              // update PID with the new tunings
      break;

    case tuner.runPid:  // this case runs once per sample period after case "tunings"
      Input = (analogRead(inputPin) / mvResolution) - bias;
      myPID.Compute();
      analogWrite(outputPin, Output);
      tuner.plotter(Setpoint, 10); // plots every 10th sample
      break;
  }
  // put your main code here, to run repeatedly
}
