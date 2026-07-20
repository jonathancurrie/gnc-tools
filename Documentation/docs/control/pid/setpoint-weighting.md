---
title: "PID Control: Setpoint Weighting"
slug: "/control/pid/setpoint-weighting/"
---

<small>This example follows on from the initial [PID example.](./controller-types.md)</small>

Two additional tuning parameters are available within 2-DOF PID controllers which allow increased performance, such as faster disturbance rejection reduced overshoot, in addition to being able to mitigate the influence of the setpoint changing on the derivative calculation, as described in the [Process Variable Derivative](./process-variable-derivative.md) section. These are the weights `b` and `c`, used as follows:

`proportionalInput = b*r - y;`

`derivativeInput = c*r - y;`

These weights must be >= 0 and can be tuned to provide fundamentally quite different control responses, without changing any of the traditional Kp, Ki or Kd gains:

![pid weightcompare](/img/gnc/pid_weightcompare.png)

For manual tuning, generally `b` is left as 1, and `c` close to 0, such as 0.1. However these two extra tuning dimensions can be powerful when automated tuning is used, as described in the [PID Tuning](./tuning.md) section.
