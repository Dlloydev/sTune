# sTune    [![arduino-library-badge](https://www.ardu-badge.com/badge/sTune.svg?)](https://www.ardu-badge.com/sTune) [![PlatformIO Registry](https://badges.registry.platformio.org/packages/dlloydev/library/sTune.svg)](https://registry.platformio.org/packages/libraries/dlloydev/sTune)

This is an open loop PID autotuner using a novel s-curve inflection point test method. Tuning parameters are typically determined in about ½Tau on a first-order system with time delay. Full 5Tau testing and multiple serial output options are provided. See [**WiKi**](https://github.com/Dlloydev/sTune/wiki) for test results and more.

### Inflection Point Tuning Method

This open-loop tuning method is used when the controller action is set to `directIP` or `reverseIP`. This method works best on processes that respond with an S-shaped reaction curve to a stepped output. Being an open-loop test, there is no setpoint and PID correction involved. The process gain, dead time, time constant and more is determined by doing a shortened step test that ends just after the [inflection point](http://en.wikipedia.org/wiki/Inflection_point) has been reached. From here, the apparent maximum PV (input) is mathematically determined and the  controller's tuning parameters are calculated. Test duration is typically only ½Tau.

#### Inflection Point Discovery

Accurate determination of the inflection point was given high priority for this test method. To accomplish this, a circular buffer for the input readings is created that's sized to 6% of the samples value. The buffer is used as a moving tangent line where the "head" of the tangent is based on the average of all readings in the buffer and the "tail" is based on the oldest instantaneous value in the buffer. The tangent line moves along the reaction curve one sample at a time. The slope of the tangent line is checked at every sample. When the sign of the change in slope changes (i.e. slope goes from increasing to decreasing or from decreasing to increasing), this is the point of inflection ([where the tangent turns red here](https://en.wikipedia.org/wiki/Inflection_point#/media/File:Animated_illustration_of_inflection_point.gif)). After 1⁄16 samples has occurred with the new slope direction, then it's known that the point of inflection has been reached. Final calculations are made and the test ends.

#### S-shaped Step Response

![Reaction Curve](https://user-images.githubusercontent.com/63488701/151890696-d574d77b-b849-4079-81e2-71e4ee416fa3.png)

#### Configuration

- First, the PID controller is placed in `manual` mode.

- The tuner action is set to `directIP` or `reverseIP`, then configure sTune:

- ```c++
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  ```

- Its expected that the user is already familiar with the controller and can make a rough estimation of what the system's time constant would be. Use this estimate for the `testTimeSec` constant.

- The `samples` constant is used to define the maximum number of samples used to perform the test. To get an accurate representation of the curve, the suggested range is 200-500. 

- `settleTimeSec` is used to provide additional settling time prior to starting the test. 

- `inputSpan` and `outputSpan` represent the maximum operating range of input and output. Examples:

  - If your input works with temp readings of 20C min and 220C max, then `inputSpan = 200;`
     If the output uses 8-bit PWM and your using its full range, then `outputSpan = 255;`
     If the output is digital relay controlled by millis() with window period of 2000ms, then `outputSpan = 2000;`

- `outputStart` is the initial control output value which is used for the `settleTimeSec` duration and sample 0.

- `outputStep` is the stepped output value used for sample 1 to test completion.

- after test completion, the setup can be updated for the next test and `tuner.Configure()` can be called again.

#### Test

- The `outputStep` value is applied at sample 1 and inflection point discovery begins.

- Dead time is determined when the averaged input has increased (or decreased) beyond one resolution value from the starting instantaneous input value.
- When the point of inflection is reached, the test ends. The apparent `pvMax` is calculated using:

- ```c++
  pvMax = pvIp + slopeIp * kexp;  // where kexp = 4.3004 = (1 / exp(-1)) / (1 - exp(-1))
  ```

- The process gain `Ku` and time constant `Tu` are determined and the selected tuning rule's constants are used to determine `Kp, Ki, Kd, Ti and Td`. Also, `controllability` and other details are provided (see comments in `sTune.cpp`).
- In the user's sketch, the PID controller is set to automatic, the tuning parameters are applied the PID controller is run.

### Full 5T Test

A full test to pvMax is used when the controller action is set to `direct5T` or `reverse5T`. Use this method if the `IP`testing isn't a good fit to the process or if you'd like to get test data for the complete input response. Here, it is assumed the test will complete at about 3.5τ, from which point the apparent `pvMax` is estimated and the tuning parameters are calculated.

![image](https://user-images.githubusercontent.com/63488701/150998367-a1999050-cab4-486f-a3ae-1c9dbdf11060.png)

### Functions

#### sTune Constructor

```c++
sTune(float *input, float *output, TuningRule tuningRule, Action action, SerialMode serialMode);
```

- `input` and `output` are pointers to the variables holding these values.
- `tuningRule` provides selection of 10 various tuning rules as described in the table below.
- `action` provides choices for controller action (direct or reverse) and whether to perform a fast inflection point test (IP) or a full 5 time constant test (5T). Choices are `directIP`, `direct5T`, `reverseIP` and `reverse5T`.
- `serialMode` provides 6 choices for serial output.

| Open Loop Tuning Methods | Autotune  Plot (using PWM output)                            | Description                                                  |
| ------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `ZN_PID`                 | [![Click to enlarge](https://user-images.githubusercontent.com/63488701/149276354-fcec7626-4f0a-4ce2-a8a2-078745a84dcd.png)](https://user-images.githubusercontent.com/63488701/149275791-a46f353a-3215-486c-a9ce-d96183897272.PNG) | Open Loop Ziegler-Nichols method with ¼ decay ratio          |
| `DampedOsc_PID`          | [![Click to enlarge](https://user-images.githubusercontent.com/63488701/149277219-48534317-cf0f-44f0-a5be-12653b2f6e0c.png)](https://user-images.githubusercontent.com/63488701/149275509-68d12ba6-269a-4ce1-b8a7-36037518c49b.PNG) | Damped Oscillation method can solve marginal stability issues |
| `NoOvershoot_PID`        | [![Click to enlarge](https://user-images.githubusercontent.com/63488701/149277649-7bdf4a72-4de6-41b2-9825-628d4d742423.png)](https://user-images.githubusercontent.com/63488701/149275728-4fca57f0-c975-4350-ab45-56d50b7c2cdf.PNG) | No Overshoot uses the C-H-R method (set point tracking) with 0% overshoot |
| `CohenCoon_PID`          | [![Click to enlarge](https://user-images.githubusercontent.com/63488701/149278074-6a01584e-0c9b-44bf-b595-f436f653f220.png)](https://user-images.githubusercontent.com/63488701/149275398-69586823-0267-4834-8183-d383713f2d27.PNG) | Open loop Cohen Coon method approximates closed loop response with a ¼ decay ratio |
| `Mixed_PID`              | [![Click to enlarge](https://user-images.githubusercontent.com/63488701/149278272-5057825c-5d92-4e69-9790-c90fd5235eaf.png)](https://user-images.githubusercontent.com/63488701/149275646-7aa0d8c9-397e-4bc5-8110-85b54a951c4f.PNG) | Mixed method averages the gains for `ZN_PID`, `DampedOsc_PID`, `NoOvershoot_PID` and `CohenCoon_PID` |
| `ZN_PI`                  |                                                              | Open Loop Ziegler-Nichols method with ¼ decay ratio          |
| `DampedOsc_PI`           |                                                              | Damped Oscillation method can solve marginal stability issues |
| `NoOvershoot_PI`         |                                                              | No Overshoot uses the C-H-R method (set point tracking) with 0% overshoot |
| `CohenCoon_PI`           |                                                              | Open loop Cohen Coon method approximates closed loop response with a ¼ decay ratio |
| `Mixed_PI`               |                                                              | Mixed method averages the gains for `ZN_PI`, `DampedOsc_PI`, `NoOvershoot_PI` and `CohenCoon_PI` |

| Serial Mode     | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| `serialOFF`     | No serial output will occur.                                 |
| `printALL`      | Prints test data while settling and during the test run. A summary of results is printed when testing completes. |
| `printSUMMARY`  | A summary of results is printed when testing completes.      |
| `printDEBUG`    | Same as `printALL`but includes printing diagnostic data during test run. |
| `printPIDTUNER` | ➩  Prints test data in csv format compatible with [pidtuner.com](https://pidtuner.com). <br />➩  Requires the controller `action` being set to `direct5T` or `reverse5T`<br />➩  Just copy the serial printer data and import (paste) into PID Tuner for further<br />      analysis, model identification, fine PID tuning and experimentation. <br />➩  Note that `Kp`, `Ti` and `Td` is also provided for PID Tuner. |
| `printPLOTTER`  | Plots `pvAvg` data for use with Serial Plotter.              |

#### Instantiate sTune

```c++
sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);
/*                                         ZN_PID           directIP     serialOFF
                                           DampedOsc_PID    direct5T     printALL
                                           NoOvershoot_PID  reverseIP    printSUMMARY
                                           CohenCoon_PID    reverse5T    printDEBUG
                                           Mixed_PID
                                           ZN_PI
                                           DampedOsc_PI
                                           NoOvershoot_PI
                                           CohenCoon_PI
                                           Mixed_PI
*/
```

#### Configure

This function applies the sTune test settings.

```c++
void Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,
uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples);
```

#### Set Functions

```c++
void SetEmergencyStop(float e_Stop);
void SetControllerAction(Action Action);
void SetSerialMode(SerialMode SerialMode);
void SetTuningMethod(TuningMethod TuningMethod);
```

#### Query Functions

```c++
float GetKp();                  // proportional gain
float GetKi();                  // integral gain
float GetKd();                  // derivative gain
float GetTi();                  // integral time
float GetTd();                  // derivative time
float GetProcessGain();         // process gain
float GetDeadTime();            // process dead time (seconds)
float GetTau();                 // process time constant (seconds)
uint8_t GetControllerAction();
uint8_t GetSerialMode();
uint8_t GetTuningMethod();
void GetAutoTunings(float * kp, float * ki, float * kd);

```

#### Controllability of the process

When the test ends, sTune determines [how difficult](https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png) the process is to control.

```c++
float controllability = _Tu / _td + epsilon;
```

#### References

- [Comparison of PID Controller Tuning Methods](http://maulana.lecture.ub.ac.id/files/2014/12/Jurnal-PID_Tunning_Comparison.pdf)
- [Ziegler-Nichols Open-Loop Tuning Rules](https://blog.opticontrols.com/archives/477)
- [Inflection point](https://en.wikipedia.org/wiki/Inflection_point)
- [Time Constant (Re: Step response with arbitrary initial conditions)](https://en.wikipedia.org/wiki/Time_constant)
- [Sample Time is a Fundamental Design and Tuning Specification](https://controlguru.com/sample-time-is-a-fundamental-design-and-tuning-specification/)

