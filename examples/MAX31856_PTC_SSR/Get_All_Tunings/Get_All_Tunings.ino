/*****************************************************************************
  sTune Get All Tunings Example (MAX31856, PTC Heater / SSR / Software PWM)
  This runs a fast inflection point test to determine tuning parameters.
  Open serial printer to view test progress and results.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX31856_PTC_SSR
  ****************************************************************************/
#include <Adafruit_MAX31856.h>
#include <sTune.h>

// pins
const uint8_t inputPin = 0;
const uint8_t relayPin = 3;
const uint8_t drdyPin = 5;

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

Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10);
sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);

void setup() {
  pinMode(drdyPin, INPUT);
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  delay(3000);
  Output = 0;
  if (!maxthermo.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
}

void loop() {
  tuner.softPwm(relayPin, Input, Output, 0, outputSpan, 1);

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      if (!digitalRead(drdyPin)) Input = maxthermo.readThermocoupleTemperature();
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
