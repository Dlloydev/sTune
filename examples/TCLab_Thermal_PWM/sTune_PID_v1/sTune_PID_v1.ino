/********************************************************************
  sTune PID_v1 Example
  This sketch does on-the-fly tunning and PID Digital Output control
  of a thermal heater (TIP31C). Tunning parameters are quickly
  determined and applied during the temperature ramp-up to setpoint.
  Open the serial plotter to view the graphical results.
  *****************************************************************/

#include <sTune.h>
#include <PID_v1.h>

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
float outputStep = 300;
float tempLimit = 75;

// variables
double input, output, setpoint = 50, kp, ki, kd; // PID_v1
float Input, Output, Setpoint = 50, Kp, Ki, Kd; // sTune

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
PID myPID(&input, &output, &setpoint, 0, 0, 0, P_ON_M, DIRECT);

void setup() {
  analogReference(EXTERNAL); // 3.3V
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(3000);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
}

void loop() {
  tuner.softPwm(relayPin, Input, Output, 0, outputSpan, 1);

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input = analogRead(inputPin) * 0.322265625 - 50.0; // get degC (using 3.3v AREF)
      tuner.plotter(Input, Output * 0.1, Setpoint, 1, 3);
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan);
      myPID.SetSampleTime(outputSpan - 1);
      output = outputStep, kp = Kp, ki = Ki, kd = Kd;
      myPID.SetMode(AUTOMATIC); // the PID is turned on
      myPID.SetTunings(kp, ki, kd); // update PID with the new tunings
      break;

    case tuner.runPid: // active once per sample after tunings
      input = analogRead(inputPin) * 0.322265625 - 50.0; // get degC (using 3.3v AREF)
      myPID.Compute();
      Input = input, Output = output;
      tuner.plotter(Input, Output * 0.1, Setpoint, 1, 3);
      break;
  }
}
