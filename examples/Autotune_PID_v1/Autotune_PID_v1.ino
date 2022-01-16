/********************************************************************
   Autotune PID_v1 Example (using Temperature Control Lab)
   http://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl
 ********************************************************************/

#include <sTune.h>
#include <PID_v1.h>

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
bool clearPidOutput = false;    // false: "on the fly" testing, true: PID starts at 0 output

// temperature
const float mvResolution = 3300 / 1024.0f;
const float bias = 50;

// test variables
double Input = 0, Output = 0, Setpoint = 30; // myPID
float input = 0, output = 0, kp = 0, ki = 0, kd = 0; // tuner

PID myPID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT);

sTune tuner = sTune(&Input, &Output, tuner.Mixed_PID, tuner.directIP, tuner.printALL);
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
  analogReference(EXTERNAL); // used by TCLab
  Serial.begin(115200);
  analogWrite(outputPin, outputStart);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {

  switch (tuner.Run()) {  // active while sTune is testing
    case tuner.inOut:
      input = (analogRead(inputPin) / mvResolution) - bias;
      analogWrite(outputPin, output);
      break;

    case tuner.tunings:                                      // active just once when sTune is done
      tuner.GetAutoTunings(&kp, &ki, &kd);                   // sketch variables updated by sTune
      myPID.SetSampleTime((testTimeSec * 1000) / samples);   // PID sample rate
      if (clearPidOutput) Output = 0;
      myPID.SetMode(AUTOMATIC);                              // the PID is turned on (automatic)
      myPID.SetTunings(kp, ki, kd);                          // update PID with the new tunings
      break;

    case tuner.runPid:  // this case runs once per sample period after case "tunings"
      Input = (analogRead(inputPin) / mvResolution) - bias;
      myPID.Compute();
      analogWrite(outputPin, Output);
      break;
  }
  // put your main code here, to run repeatedly
}
