/********************************************************************************
  sTune QuickPID Adaptive Control Example
  This sketch does on-the-fly tunning and PID SSR control of a PTC heater
  using the ZN_PID tuning method. When sTune completes, we need the input
  to approach setpoint for the first time. As the input gets closer, the gains
  (tunings) are gradually applied. This helps keep the integral term to a
  minimum to help reduce initial overshoot. Final overshoot correction is made
  at the first crossover from setpoint. Here, a full AC cyle is dropped from
  the output and 100% PID control resumes with ½ AC cycle optimized SSR control.

  Open the serial plotter to view the graphical results.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX6675_PTC_SSR
  *******************************************************************************/

#include <max6675.h>
#include <sTune.h>
#include <QuickPID.h>

// pins
const uint8_t inputPin = 0;
const uint8_t relayPin = 3;
const uint8_t drdyPin = 5;
const uint8_t SO = 12;
const uint8_t CS = 10;
const uint8_t sck = 13;

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 500;
const uint16_t samples = 500;
const float inputSpan = 200;
const float outputSpan = 2000;
float outputStart = 0;
float outputStep = 60;
float tempLimit = 75;
uint8_t debounce = 1;
uint8_t  startup = 0;
// variables
float Input, Output, Setpoint = 50, Kp, Ki, Kd;

MAX6675 module(sck, CS, SO); //SPI
sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
QuickPID myPID(&Input, &Output, &Setpoint);

void setup() {
  pinMode(drdyPin, INPUT);
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(3000);
  Output = 0;
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
}

void loop() {
  float optimumOutput = tuner.softPwm(relayPin, Input, Output, Setpoint, outputSpan, debounce);

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input = module.readCelsius();
      tuner.plotter(Input, Output * 0.5, Setpoint, 1, 3);
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan * 0.1);
      myPID.SetSampleTimeUs(outputSpan * 1000 * 0.2);
      debounce = 0; // switch to SSR optimum cycle mode
      myPID.SetMode(myPID.Control::automatic); // the PID is turned on
      myPID.SetProportionalMode(myPID.pMode::pOnMeas);
      myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);
      myPID.SetTunings(Kp, 0, 0); // update PID with the new Kp (P-controller)
      break;

    case tuner.runPid: // active once per sample after tunings
      Input = module.readCelsius();
      if (Input > Setpoint && startup == 6) {
        Output -= 17; // drop full AC cycle
        myPID.SetMode(myPID.Control::manual);    // toggle PID control mode
        myPID.SetMode(myPID.Control::automatic); // PID uses new output value
        startup++;
      } else if (Input > Setpoint - 3 && startup == 5) {
        myPID.SetTunings(Kp, Ki, Kd);  // 100% of gains
        startup++;
      } else if (Input > Setpoint - 6 && startup == 4) {
        myPID.SetTunings(Kp, Ki * 0.8, Kd * 0.8);  // 80% of gains
        startup++;
      } else if (Input > Setpoint - 9 && startup == 3) {
        myPID.SetTunings(Kp, Ki * 0.6, Kd * 0.6);  // 60% of gains
        startup++;
      } else if (Input > Setpoint - 12 && startup == 2) {
        myPID.SetTunings(Kp, Ki * 0.4, Kd * 0.4);  // 40% of gains
        startup++;
      } else if (Input > Setpoint - 15 && startup == 1) {
        myPID.SetTunings(Kp, Ki * 0.2, Kd * 0.2);  // 20% of gains
        startup++;
      } else if (Input > Setpoint - 18 && startup == 0) {
        myPID.SetTunings(Kp, Ki * 0.1, Kd * 0.1);  // 10% of gains
        startup++;
      }
      myPID.Compute();
      tuner.plotter(Input, optimumOutput * 0.5, Setpoint, 1, 3);
      break;
  }
}
