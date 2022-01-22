/****************************************************************************
  Autotune QuickPID Digital Output Example
  https://github.com/Dlloydev/sTune/wiki/Autotune_PID_Digital_Out_Reference

  The output is a digital pin controlling a mechanical relay, SSR, MOSFET or
  other device. To interface the PID output to the digital pin, we use
  software PWM. The PID compute rate controls the output update rate. All
  transitions are debounced and there's only one call to digitalWrite()
  per transition.

  This sketch runs sTune then applies the tunings to QuickPID. Open the
  serial plotter to view the input response through the complete tuning and
  PID control process.
 ****************************************************************************/

#include <sTune.h>
#include <QuickPID.h>

// pins
const uint8_t inputPin = 0;
const uint8_t relayPin = 3;

// user settings (sTune)
uint32_t settleTimeSec = 15;
uint32_t testTimeSec = 300;
const uint16_t samples = 300;
const float inputSpan = 80;
const float outputSpan = (testTimeSec / samples) * 1000;
float outputStart = 0;
float outputStep = 100;

// user settings (PID)
float Setpoint = 30;
const float windowSize = 5000;
const byte debounce = 50;

// status
unsigned long windowStartTime, nextSwitchTime, msNow;
boolean relayStatus = false;

// variables
float Input, Output, Kp, Ki, Kd;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
/*                                         ZN_PID           directIP        printOFF
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
  pinMode(relayPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {
  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      windowStartTime = msNow;
      Input = analogRead(inputPin);
      tuner.plotter(Setpoint, 0.05, 5, 1); // scaled output, every 5th sample, averaged input
      break;
  
    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, windowSize);
      myPID.SetSampleTimeUs(windowSize * 1000);
      // Output = 0; // optional output preset value
      myPID.SetMode(myPID.Control::automatic); // the PID is turned on
      myPID.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
      break;
  
    case tuner.runPid: // active once per sample after tunings
      tuner.plotter(Setpoint, 0.05, 5, 1);
      break;
  }
  // put your main code here, to run repeatedly
  msNow = millis();
  softPwmOutput();
  Input = analogRead(inputPin);
  if (myPID.Compute()) windowStartTime = msNow;
}

void softPwmOutput() {
  if (!relayStatus && Output > (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = true;
      digitalWrite(relayPin, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  } else if (relayStatus && Output < (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = false;
      digitalWrite(relayPin, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
