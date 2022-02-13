/***************************************************************************
  sTune QuickPID Example (MAX6675, PTC Heater / SSR / Software PWM)
  This sketch does on-the-fly tuning and PID control. Tuning parameters are
  quickly determined and applied. During the initial temperature ramp-up,
  the output is reduced just prior to the input reaching setpoint. This
  reduces overshoot. View results using serial plotter.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX6675_PTC_SSR
  ***************************************************************************/
#include <max6675.h>
#include <sTune.h>
#include <QuickPID.h>

// pins
const uint8_t inputPin = 0;
const uint8_t relayPin = 3;
const uint8_t SO = 12;
const uint8_t CS = 10;
const uint8_t sck = 13;

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 500;  // runPid interval = testTimeSec / samples
const uint16_t samples = 500;
const float inputSpan = 200;
const float outputSpan = 1000;
float outputStart = 0;
float outputStep = 50;
float tempLimit = 150;
uint8_t debounce = 1;
bool startup = true;

// variables
float Input, Output, Setpoint = 80, Kp, Ki, Kd;

MAX6675 module(sck, CS, SO); //SPI
sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
QuickPID myPID(&Input, &Output, &Setpoint);

void setup() {
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
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
      tuner.plotter(Input, Output, Setpoint, 0.5f, 3); // output scale 0.5, plot every 3rd sample
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan * 0.1);
      myPID.SetSampleTimeUs((outputSpan - 1) * 1000);
      debounce = 0; // ssr mode
      Output = outputStep;
      myPID.SetMode(myPID.Control::automatic); // the PID is turned on
      myPID.SetProportionalMode(myPID.pMode::pOnMeas);
      myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);
      myPID.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
      break;

    case tuner.runPid: // active once per sample after tunings
      if (startup && Input > Setpoint - 5) { // reduce overshoot
        startup = false;
        Output -= 9;
        myPID.SetMode(myPID.Control::manual);
        myPID.SetMode(myPID.Control::automatic);
      }
      Input = module.readCelsius();
      myPID.Compute();
      tuner.plotter(Input, optimumOutput, Setpoint, 0.5f, 3);
      break;
  }
}
