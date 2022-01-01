# sTune

This is an open loop PID autotuner using a novel s-curve inflection point test method. Tuning parameters are typically determined in about ½Tau on a first-order system with time delay. Full 5Tau testing and multiple serial output options are provided.

#### Reaction Curve Inflection Point Tuning Method

This tuning method determines the process gain, dead time and time constant by doing a shortened step test that ends just after the [inflection point](http://en.wikipedia.org/wiki/Inflection_point) has been reached. From here, the apparent maximum PV (input) is mathematically determined and the remaining controller settings are calculated. Test duration is typically only ½Tau.  

#### Configuration

Its expected that the user is already familiar with the controller and can make a rough estimation of what the system's time constant would be. Use this estimate for the `testTimeSec` constant.

The `samples` constant is used to define the maximum number of samples used to perform the test. To get an accurate representation of the curve, the suggested range is 200-500. 

`settleTimeSec` is used to provide additional settling time prior to starting the test. 

`outputStart` is the initial control output value which is used for the first 12 samples.

`outputStep` is the stepped output value used for sample 13 to test completion.

#### Inflection Point Discovery

Accurate determination of the inflection point was given high priority for this test method. To accomplish this, a circular buffer for the input readings is created that's sized to 10% of samples. The buffer is used as a moving tangent line where the "head" of the tangent is based on the average of all readings in the buffer and the "tail" is based on the oldest instantaneous value in the buffer. The tangent line slides along the reaction curve where where the head (leading point) moves smoothly but with somewhat sluggish response, and the tail (trailing point) has some jitter due to being instantaneous, noise and input resolution.

The slope of the tangent line is checked at every sample. When the sign of the change in slope changes (i.e. slope goes from increasing to decreasing or from decreasing to increasing), this is the point of inflection ([where the tangent turns red here](https://en.wikipedia.org/wiki/Inflection_point#/media/File:Animated_illustration_of_inflection_point.gif)). After `samples/16` samples has occurred with the new slope direction, then the leading tangent point is used as the point of inflection in calculations.

#### Inflection Point Tuning Procedure

![Reaction Curve](https://user-images.githubusercontent.com/63488701/147842980-01d12e68-ed80-486c-823e-fcfaa6b89b41.png)

1.  To perform this test, the controller is placed in `manual` mode and controller action is set to `directIP` or `reverseIP`. The input should be stable while the output is at the `outputStart` value.

2. A step change to the `outputStep` value is made after 12 samples and inflection point discovery begins.

3. Dead time is determined when the input has increased (or decreased) for 4 samples.

4. When the point of inflection is reached, the test ends. The apparent `pvMax` is calculated using `pvMax = pvIp + slopeIp * kexp;` where  `kexp = 4.3004; // (1 / exp(-1)) / (1 - exp(-1))`  See [Time Constant (Re: Step response with arbitrary initial conditions)](https://en.wikipedia.org/wiki/Time_constant) 

    The process gain and time constant are determined and the selected tuning rule's constants are used to determine `Kp, Ki and Kd`. Also, `controllability` and other details are provided (see comments in `sTune.cpp`).

5. In the user's sketch, the PID controller is set to automatic, the tuning parameters are applied the PID controller is run.

### Functions

#### sTune Constructor

```c++
sTune(float *input, float *output, TuningRule tuningRule, Action action, SerialMode serialMode);
```

- `input` and `output` are pointers to the variables holding these values.
- `tuningRule` provides selection of 10 various tuning rules as described in the table below.
- `action` provides choices for controller action (direct or reverse) and whether to perform a fast inflection point test (IP) or a full 5 time constant test (5T). Choices are `directIP`, `direct5T`, `reverseIP` and `reverse5T`.
- `serialMode` provides 6 choices for serial output as described in the table below.

| Tuning Rule         | Description                                              |
| ------------------- | -------------------------------------------------------- |
| `zieglerNicholsPI`  | Good noise and disturbance rejection                     |
| `zieglerNicholsPID` | Good noise and disturbance rejection                     |
| `tyreusLuybenPI`    | Time-constant (lag) dominant processes (conservative)    |
| `tyreusLuybenPID`   | Time-constant (lag) dominant processes (conservative)    |
| `cianconeMarlinPI`  | Delay (dead-time) dominant processes                     |
| `cianconeMarlinPID` | Delay (dead-time) dominant processes                     |
| `amigofPID`         | More universal than ZN_PID (uses a dead time dependency) |
| `pessenIntegralPID` | Similar to ZN_PID but with better dynamic response       |
| `someOvershootPID`  | ZN_PID with lower proportional and integral gain         |
| `noOvershootPID`    | ZN_PID with much lower P,I,D gain settings               |

| Serial Mode     | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| `serialOFF`     | No serial output will occur.                                 |
| `printALL`      | Prints test data while settling and during the test run. A summary of results is printed when testing completes. |
| `printSUMMARY`  | A summary of results is printed when testing completes.      |
| `printDEBUG`    | Same as `printALL`but includes printing diagnostic data during test run. |
| `printPIDTUNER` | ○  Prints test data in csv format compatible with [pidtuner.com](https://pidtuner.com/#/). <br />○  Requires the controller `action` being set to `direct5T` or `reverse5T`<br />○  Just copy the serial printer data and import (paste) into PID Tuner for further analysis, model identification, fine PID tuning and experimentation. <br />○  Note that `Kp`, `Ti` and `Td` is also provided for PID Tuner. |
| `printPLOTTER`  | Plots `pvAvg` data for use with Serial Plotter.              |

#### Instantiate sTune

```c++
sTune tuner = sTune(&Input, &Output, tuner.zieglerNicholsPID, tuner.directIP, tuner.printALL);
/*                                         zieglerNicholsPI         directIP        serialOFF
                                           zieglerNicholsPID        direct5T        printALL
                                           tyreusLuybenPI           reverseIP       printSUMMARY
                                           tyreusLuybenPID          reverse5T       printDEBUG
                                           cianconeMarlinPI                         printPIDTUNER
                                           cianconeMarlinPID                        serialPLOTTER
                                           amigofPID
                                           pessenIntegralPID
                                           someOvershootPID
                                           noOvershootPID
*/
```

#### Configure

```c++
void Configure(const float outputStart, const float outputStep, const uint32_t testTimeSec, const uint32_t settleTimeSec, const uint16_t samples);
```

This function applies the sTune test settings.

#### Set Functions

```c++
void SetAutoTunings(float * kp, float * ki, float * kd);
void SetControllerAction(Action Action);
void SetSerialMode(SerialMode SerialMode);
void SetTuningRule(TuningRule TuningRule);
```

#### Query Functions

```c++
float GetKp();                 // proportional gain
float GetKi();                 // integral gain
float GetKd();                 // derivative gain
float GetProcessGain();        // process gain
float GetDeadTime();           // process dead time (seconds)
float GetTau();                // process time constant (seconds)
float GetTimeIntegral();       // process time integral (seconds)
float GetTimeDerivative();     // process time derivative (seconds)
uint8_t GetControllerAction();
uint8_t GetSerialMode();
uint8_t GetTuningRule();
```

#### Controllability of the process

```c++
float controllability = _Tu / _td + epsilon;
```

When the test ends, sTune determines [how difficult](https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png) the process is to control.

| Controllability   | Comment                 |
| ----------------- | ----------------------- |
| Greater than 0.75 | Easy to control         |
| 0.25 to 0.74      | Average controllability |
| 0.00 to 0.24      | Difficult to control    |

#### References

- [Ziegler-Nichols Open-Loop Tuning Rules](https://blog.opticontrols.com/archives/477)
- [Inflection point](https://en.wikipedia.org/wiki/Inflection_point)
- [Time Constant (Re: Step response with arbitrary initial conditions)](https://en.wikipedia.org/wiki/Time_constant)

