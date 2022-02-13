/***************************************************************************
  QuickPID Basic Example (MAX6675, PTC Heater / SSR / Software PWM)
  Update your user settings and tuning parameters, then run and view results
  using serial plotter.
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

uint32_t testTimeSec = 1;  // runPid interval = testTimeSec / samples
const uint16_t samples = 1;
const float outputSpan = 1000; // window size
float tempLimit = 150;

// variables
float Input, Output, Setpoint = 80, Kp = 2.00, Ki = 0.02, Kd = 0.06;

MAX6675 module(sck, CS, SO); //SPI
sTune tuner = sTune(); // for softPWM and tempLimit
QuickPID myPID(&Input, &Output, &Setpoint);

void setup() {
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  delay(3000);
  tuner.Configure(0, 0, 0, 0, testTimeSec, 0, samples);
  tuner.SetEmergencyStop(tempLimit);
  myPID.SetOutputLimits(0, outputSpan * 0.1);
  myPID.SetSampleTimeUs((outputSpan - 1) * 1000);
  myPID.SetMode(myPID.Control::automatic); // the PID is turned on
  myPID.SetProportionalMode(myPID.pMode::pOnMeas);
  myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);
  myPID.SetTunings(Kp, Ki, Kd); // set PID gains
}

void loop() {
  float optimumOutput = tuner.softPwm(relayPin, Input, Output, Setpoint, outputSpan, 0); // ssr mode
  if (myPID.Compute()) {
    Input = module.readCelsius();
    tuner.plotter(Input, optimumOutput, Setpoint, 0.5, 3); // output scale 0.5, plot every 3rd sample
  }
}
