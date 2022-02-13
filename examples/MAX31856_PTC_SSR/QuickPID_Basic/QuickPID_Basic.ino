/***************************************************************************
  QuickPID Basic Example (MAX31856, PTC Heater / SSR / Software PWM)
  Update your user settings and tuning parameters, then run and view results
  using serial plotter.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX31856_PTC_SSR
  ***************************************************************************/
#include <Adafruit_MAX31856.h>
#include <sTune.h>
#include <QuickPID.h>

// pins
const uint8_t inputPin = 0;
const uint8_t relayPin = 3;
const uint8_t drdyPin = 5;

uint32_t testTimeSec = 1;  // runPid interval = testTimeSec / samples
const uint16_t samples = 1;
const float outputSpan = 1000; // window size
float tempLimit = 150;

// variables
float Input, Output, Setpoint = 80, Kp = 2.00, Ki = 0.02, Kd = 0.06;

Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10); //SPI
sTune tuner = sTune(); // for softPWM and tempLimit
QuickPID myPID(&Input, &Output, &Setpoint);

void setup() {
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  delay(3000);
  if (!maxthermo.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
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
    if (!digitalRead(drdyPin)) Input = maxthermo.readThermocoupleTemperature();
    tuner.plotter(Input, optimumOutput, Setpoint, 0.5f, 3); // output scale 0.5, plot every 3rd sample
  }
}
