/***************************************************************************
  sTune QuickPID Example
  This sketch does on-the-fly tunning and PID SSR control of a
  PTC heater. Tunning parameters are quickly determined and
  applied during the temperature ramp-up to setpoint. Open
  the serial plotter to view the graphical results.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX6675_PTC_SSR
  ***************************************************************************/

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
const float outputSpan = 1000;
float outputStart = 0;
float outputStep = 30;
float tempLimit = 75;
uint8_t debounce = 1;

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
      tuner.plotter(Input, Output, Setpoint, 1, 3);
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan * 0.1);
      myPID.SetSampleTimeUs(outputSpan * 1000 * 0.2);
      debounce = 0; // switch to SSR optimum cycle mode
      myPID.SetMode(myPID.Control::automatic); // the PID is turned on
      myPID.SetProportionalMode(myPID.pMode::pOnMeas);
      myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);
      myPID.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
      break;

    case tuner.runPid: // active once per sample after tunings
      Input = module.readCelsius();
      myPID.Compute();
      tuner.plotter(Input, optimumOutput, Setpoint, 1, 3);
      break;
  }
}
