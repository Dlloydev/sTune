/********************************************************************
   Autotune PID_v1 Example

  This sketch runs sTune then applies the tunings to PID_v1. Open
  the serial printer to view the test progress and results.
 ********************************************************************/

#include <sTune.h>
#include <PID_v1.h>

// pins
const uint8_t inputPin = 0;
const uint8_t outputPin = 3;

// user settings (sTune)
uint32_t settleTimeSec = 15;
uint32_t testTimeSec = 300;
const uint16_t samples = 500;
const float inputSpan = 80;
const float outputSpan = 255;
float outputStart = 0;
float outputStep = 25;

// user settings (PID)
double Setpoint = 30;

// variables
double Input, Output; // PID
float input, output, kp, ki, kd; // sTune

PID myPID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT);

sTune tuner = sTune(&input, &output, tuner.ZN_PID, tuner.directIP, tuner.printALL);
/*                                         ZN_PID           directIP        serialOFF
                                           DampedOsc_PID    direct5T        printALL
                                           NoOvershoot_PID  reverseIP       printSUMMARY
                                           CohenCoon_PID    reverse5T       printDEBUG
                                           Mixed_PID
                                           ZN_PI
                                           DampedOsc_PI
                                           NoOvershoot_PI
                                           CohenCoon_PI
                                           Mixed_PI
*/
void setup() {
  Serial.begin(115200);
  analogWrite(outputPin, outputStart);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input = analogRead(inputPin);
      analogWrite(outputPin, output);
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&kp, &ki, &kd); // sketch variables updated by sTune
      myPID.SetSampleTime((testTimeSec  * 1000) / samples); // PID sample rate
      // Output = 0; // optional output preset value
      myPID.SetMode(AUTOMATIC); // the PID is turned on
      myPID.SetTunings(kp, ki, kd); // update PID with the new tunings
      break;

    case tuner.runPid: // active once per sample after tunings
        Input = analogRead(inputPin);
        myPID.Compute();
        analogWrite(outputPin, Output);
        break;
      }
// put your main code here, to run repeatedly
}
