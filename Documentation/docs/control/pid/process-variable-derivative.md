---
title: "PID Control: Process Variable Derivative"
slug: "/control/pid/process-variable-derivative/"
---

<small>This example follows on from the initial [PID example.](./controller-types.md)</small>

While it is typical to place saturation constraints on the control input u for safety (i.e. a maximum voltage, maximum engine RPM, etc), one way to avoid the spikes we saw in the PID example is to only take the derivative of the process variable y, rather than of the error (r - y). This means changes in setpoint won't be seen by the derivative, only changes in the process variable, which is typically what we want. The GNCTools PID controller allows this via setpoint weighting on the `c` term, where it is used as:

`derivativeInput = c*r - y;`

```matlab
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0; % Kd setpoint weight

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, 0, 'PID Control w PV Only Derivative');
```

![pid pidpv](/img/gnc/pid_pidpv.png)

As can be seen above, the spikes have been substantially reduced, with the initial spike now just as a result of the proportional term (error = 1, Kp = 0.5, u = 0.5 = spike). Given `c` is a variable, it is possible to include some of the setpoint in the derivative calculation, as this normally improves the setpoint tracking behaviour (i.e. it can track faster if the derivative component 'sees' the change coming). Below is a comparison with 3 values of `c`:

![pid pvcompare](/img/gnc/pid_pvcompare.png)
