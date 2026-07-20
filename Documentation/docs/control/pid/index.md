---
title: "Proportional-Integral-Derivative (PID) Control"
slug: "/control/pid/"
---

[PID Control](https://en.wikipedia.org/wiki/PID_controller) is one of the most ubiquitous control schemes, perhaps only surpassed only by [bang-bang control](https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control). However when it comes to a PID controller available for embedded applications that includes industry standard features *and* will match the results obtained using MATLAB, it is hard to find! Therefore I have written a simple yet powerful PID controller with the following functionality:

- **Discrete**

  Required to be practically implementable on an embedded system.

- **2 Degree of Freedom (DOF)**

  Allows more flexibility in tuning the controller by providing the setpoint and plant output separately to the controller.

- **[P, PI, PD and PID](./controller-types.md)**

  Flexible controller type to suit the application.

- **[Anti WindUp](./anti-windup.md)**

  Applies back calculation to prevent integral windup.

- **[Process Variable Derivative](./process-variable-derivative.md)**

  Adjust the derivative weight to include or exclude changes in the setpoint from the derivative calculation.

- **[Derivative Filter](./derivative-filter.md)**

  Includes an optional 1st order derivative filter to minimize the effect of noise.

- **[Setpoint Weighting](./setpoint-weighting.md)**

  Apply weights to the proportional and derivative terms to aid setpoint tracking tuning.

- **[Setpoint Ramping](./setpoint-ramping.md)**

  Limit the rate of change of the setpoint to help prevent system instability.

- **[Tuning Guide](./tuning.md)**

  A handy guide to tuning PID controllers.

- **[Stability Guide](./stability.md)**

  A handy guide to ensuring a stable control system.

- **Native C**

  No external libraries other than the [C standard library](https://en.wikipedia.org/wiki/C_standard_library).

- **C++ Wrapper**

  If you prefer a C++ object, one is [available](https://github.com/jonathancurrie/gnc-tools/blob/master/Control/PID/Source/pid.hpp) with all functionality.

- **Documented**

  Fully documented source code suitable for [Doxygen](https://www.doxygen.nl/) auto-generated output.

- **Unit Tested**

  Validated against the MATLAB Control System Toolbox [pid2](https://au.mathworks.com/help/control/ref/pid2.html) class and Simulink [Discrete 2DOF PID](https://au.mathworks.com/help/simulink/slref/discretepidcontroller2dof.html) block.

Clone [GNCTools](https://github.com/jonathancurrie/gnc-tools) to access the PID source, including MEX interfaces and MATLAB examples and unit tests, or alternatively jump into the PID [Source Code](https://github.com/jonathancurrie/gnc-tools/tree/master/Control/PID/Source) to have a look! The examples via the links above can be found in `Control/PID/PID_Examples.m`.

## Nomenclature

The following diagram shows the format of the GNCTools PID controller, together with variable naming.

![PID Block Diagram](/img/gnc/PIDBlockDiagram.png)

|  |  |  |
| --- | --- | --- |
| **Variable** | **GNC Tools Name** | **Alternative Name(s)** |
| `r` | Setpoint | Reference |
| `y` | Plant Output | Process Variable, Process Measurement |
| <span class="pmwiki-inline-code">y<sub>true</sub></span> | True Plant Output | Simulated Plant Output |
| `noise` | Noise | Process Noise, Unmeasured Disturbance |
| `u` | Control Input | Manipulated Variable |
| <span class="pmwiki-inline-code">u<sub>man</sub></span> | Manual Control Input | Manual Input |
