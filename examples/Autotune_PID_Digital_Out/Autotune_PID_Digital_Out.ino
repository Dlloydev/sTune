/****************************************************************************
   Autotune QuickPID Digital Output Example
   https://github.com/Dlloydev/sTune/wiki/Autotune_PID_Digital_Out_Reference
 ****************************************************************************/

#include <sTune.h>
#include <QuickPID.h>

// pins
const uint8_t inputPin = 0;
const uint8_t outputPin = 3;
const uint8_t ledPin = 9;

// test setup
uint32_t testTimeSec = 600;     // testTimeSec / samples = sample interval
const uint16_t samples = 300;

uint32_t settleTimeSec = 15;
const float inputSpan = 80;
const float outputSpan = 2000;  // window size for sTune and PID
const float minSpan = 50;
float outputStart = 0;
float outputStep = 200;
bool clearPidOutput = false;    // false: "on the fly" testing, true: PID starts at 0 output

// temperature
const float mvResolution = 3300 / 1024.0f;
const float bias = 50;

// variables
float Input, Output, Setpoint = 30, Kp, Ki, Kd;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);
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
  pinMode(outputPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(outputPin, LOW);
  analogReference(EXTERNAL); // used by TCLab
  Serial.begin(115200);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {
  softPwmOutput();
  switch (tuner.Run()) {
    case tuner.inOut: // active while sTune is testing
      Input = (analogRead(inputPin) / mvResolution) - bias;
      break;
    case tuner.tunings:                          // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd);       // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan);      // PID output spans from 0 to the full window size
      myPID.SetSampleTimeUs(outputSpan * 1000);  // PID sample rate matches sTune
      if (clearPidOutput) Output = 0;
      myPID.SetMode(myPID.Control::automatic);   // the PID is turned on (automatic)
      myPID.SetTunings(Kp, Ki, Kd);              // update PID with the new tunings
      break;
    case tuner.runPid:  // active once per sample period after case "tunings"
      Input = (analogRead(inputPin) / mvResolution) - bias;
      myPID.Compute();
      tuner.plotter(Setpoint, 0.1, 5, 1); // scaled output, every 5th sample, averaged input
      break;
  }
  // put your main code here, to run repeatedly
}

void softPwmOutput() {
  static uint32_t tPrev;
  uint32_t tNow = millis();
  uint32_t tElapsed = (tNow - tPrev);
  if (tElapsed >= outputSpan) tPrev = tNow;
  if (tElapsed > minSpan && tElapsed < outputSpan - minSpan) { // in range?
    if (tElapsed <= Output) {
      digitalWrite(outputPin, HIGH);
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(outputPin, LOW);
      digitalWrite(ledPin, LOW);
    }
  }
}
