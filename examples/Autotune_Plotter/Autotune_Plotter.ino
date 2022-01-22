/********************************************************************
   Autotune Plotter Example

  This sketch runs sTune then applies the tunings to QuickPID. Open
  the serial plotter to view the input response through the complete
  tuning and PID control process.
 ********************************************************************/

#include <sTune.h>
#include <QuickPID.h>

// pins
const uint8_t inputPin = 0;
const uint8_t outputPin = 3;

// user settings (sTune)
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 300;
const uint16_t samples = 500;
const float inputSpan = 80;
const float outputSpan = 255;
float outputStart = 0;
float outputStep = 25;

// user settings (PID)
float Setpoint = 30;

// variables
float Input, Output, Kp, Ki, Kd;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
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
      analogWrite(outputPin, Output);
      tuner.plotter(Setpoint, 0.5, 6, 1); // output scaled 0.5, plot every 6th sample, averaged input
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetSampleTimeUs((testTimeSec / samples) * 1000000);
      // Output = 0; // optional output preset value
      myPID.SetMode(myPID.Control::automatic); // the PID is turned on
      myPID.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
      break;

    case tuner.runPid: // active once per sample after tunings
      Input = analogRead(inputPin);
      myPID.Compute();
      analogWrite(outputPin, Output);
      tuner.plotter(Setpoint, 0.5, 6, 1);
      break;
  }
  // put your main code here, to run repeatedly
}
