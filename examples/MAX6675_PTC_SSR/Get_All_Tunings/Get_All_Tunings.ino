/***************************************************************************
  sTune Get All Tunings Example (MAX6675, PTC Heater / SSR / Software PWM)
  This runs a fast inflection point test to determine tuning parameters.
  Open serial printer to view test progress and results.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX6675_PTC_SSR
 ***************************************************************************/
#include <max6675.h>
#include <sTune.h>

// pins
const uint8_t inputPin = 0;
const uint8_t relayPin = 3;
const uint8_t SO = 12;
const uint8_t CS = 10;
const uint8_t sck = 13;

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 500;  // sample interval = testTimeSec / samples
const uint16_t samples = 500;
const float inputSpan = 200;
const float outputSpan = 1000;
float outputStart = 0;
float outputStep = 50;
float tempLimit = 150;

// variables
float Input, Output;

MAX6675 module(sck, CS, SO); //SPI
sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);

void setup() {
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  delay(3000);
  Output = 0;
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
}

void loop() {
  tuner.softPwm(relayPin, Input, Output, 0, outputSpan, 1);

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input = module.readCelsius();
      break;

    case tuner.tunings: // active just once when sTune is done
      Output = 0;
      tuner.SetTuningMethod(tuner.TuningMethod::DampedOsc_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::NoOvershoot_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::CohenCoon_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::Mixed_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::ZN_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::DampedOsc_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::NoOvershoot_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::CohenCoon_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::Mixed_PI);
      tuner.printTunings();
      break;
  }
}
