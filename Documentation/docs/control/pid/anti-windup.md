---
title: "PID Control: Anti WindUp"
slug: "/control/pid/anti-windup/"
---

<small>This example follows on from the initial [PID example.](./controller-types.md)</small>

One technique to address a spike in the control input, u, is to saturate the input at some specified values. This can be done as simply as `"if u > umax, u = umax"` in pseudocode, however it will lead to a quite poor control response. This is because even though the control input u is saturated and can't go any higher, the error integrator inside the controller is still seeing an error between the setpoint (r) and the plant output (y), and continues to integrate the error. This means when finally the computed control input u is not saturated (i.e. due to change in r or y, or tuning), the controller has to "unwind" the integrated error in order to start to control the system again. This can take a long time, depending on how long the integrator was "winding up". One technique to avoid this is to use back calculation to unwind the integrator during times when u is saturated, preventing this from occurring. This functionality is included automatically in the GNCTools PID Controller:

```matlab
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -1;
uMax = 1;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r*5, 0, 'PID Control w Anti Windup & U Saturation');
```

![pid pidwind](/img/gnc/pid_pidwind.png)

As seen above the control input u is saturated within the -1 -> +1 bounds specified, while as soon as the controller is no longer saturated, it begins to correct the control response (i.e. it has not wound up). To compare what saturation *without* anti-windup would look like, see the below example:

![pid pidnowind](/img/gnc/pid_pidnowind.png)

Showing substantial overshoot while the integrator is unwinding.
