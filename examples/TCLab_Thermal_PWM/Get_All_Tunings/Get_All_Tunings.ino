/****************************************************************************
  sTune Get All Tunings Example (TCLab - Thermal Heater using digital output)
 ****************************************************************************/
#include <sTune.h>

// pins
const uint8_t inputPin = 0;
const uint8_t relayPin = 3;

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 500;
const uint16_t samples = 500;
const float inputSpan = 150;
const float outputSpan = 1000;
float outputStart = 0;
float outputStep = 50;
float tempLimit = 90;

// variables
float Input, Output;

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);

void setup() {
  analogReference(EXTERNAL); // 3.3V
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(3000);
  Output = 0;
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
}

void loop() {
  tuner.softPwm(relayPin, Input, Output, 0, outputSpan, 1);

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input = analogRead(inputPin) * 0.322265625 - 50.0; // get degC (using 3.3v AREF)
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
