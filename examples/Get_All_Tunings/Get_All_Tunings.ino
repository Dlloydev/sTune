/********************************************************************
   sTune Get All Tunings Example (using Temperature Control Lab)
   http://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl
 ********************************************************************/

#include <sTune.h>

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

// temperature
const float mvResolution = 3300 / 1024.0f;
const float bias = 50;

// test variables
float Input = 0, Output = 0;

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);
/*                                         ZN_PID        directIP        serialOFF
                                           ZN_Half_PID   direct5T        printALL
                                           Damped_PID    reverseIP       printSUMMARY
                                           NoOvershoot_PID reverse5T     printDEBUG
                                           CohenCoon_PID                 printPIDTUNER
                                           ZN_PI                         serialPLOTTER
                                           ZN_Half_PI
                                           Damped_PI
                                           NoOvershoot_PI
                                           CohenCoon_PI
*/
void setup() {
  analogReference(EXTERNAL); // AVR
  Serial.begin(115200);
  delay(5000);
  analogWrite(outputPin, outputStart);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
}

void loop() {

  switch (tuner.Run()) {
    case tuner.inOut:
      Input = (analogRead(inputPin) / mvResolution) - bias;
      analogWrite(outputPin, Output);
      break;

    case tuner.tunings:
      analogWrite(outputPin, 0);
      tuner.SetTuningMethod(tuner.TuningMethod::ZN_PID);
      Serial.println(F(" ZN_PID"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::ZN_Half_PID);
      Serial.println(F(" ZN_Half_PID"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::Damped_PID);
      Serial.println(F(" Damped_PID"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::NoOvershoot_PID);
      Serial.println(F(" NoOvershoot_PID"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::CohenCoon_PID);
      Serial.println(F(" CohenCoon_PID"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::ZN_PI);
      Serial.println(F(" ZN_PI"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::ZN_Half_PI);
      Serial.println(F(" ZN_Half_PI"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::Damped_PI);
      Serial.println(F(" Damped_PI"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::NoOvershoot_PI);
      Serial.println(F(" NoOvershoot_PI"));
      PrintTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::CohenCoon_PI);
      Serial.println(F(" CohenCoon_PI"));
      PrintTunings();
      break;
  }
}

void PrintTunings() {
  Serial.print(F("  Kp: ")); Serial.println(tuner.GetKp());
  Serial.print(F("  Ki: ")); Serial.println(tuner.GetKi());
  Serial.print(F("  Kd: ")); Serial.println(tuner.GetKd());
  Serial.println();
}
