---
title: "PID Control: Setpoint Ramping"
slug: "/control/pid/setpoint-ramping/"
---

<small>This example follows on from the initial [PID example.](./controller-types.md)</small>

Even the best tuned controller will suffer if asked to make large and/or frequent setpoint changes, especially if the system is oscillatory. One technique to reduce the effect of setpoint changes is to limit the rate of change of the setpoint, also known as a reference filter or setpoint governor. GNCTools provides a maximum rate of change of the setpoint per sample via the `rRampMax` parameter:

```matlab
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0;
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0; % Kd setpoint weight
rRampMax = 0.015;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c, rRampMax);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u,y,rOut] = simPID(Gs, pidParams, t, r);
```

![pid pidsetramp](/img/gnc/pid_pidsetramp.png)

In the above figure a comparison is plotted between no ramp and with ramp, showing an equivalent settling time can be reached without the overshoot present in the original tuning. It will be up to the application whether the reduction in overshoot is worth the slower rise time with the ramp. 

Also - be cautious when using setpoint ramp functionality to ensure you don't increase the gain(s) of the controller too high. You can easily move into a "false sense of security" with a super reactive controller that excellently follows small setpoint changes, but may suffer badly with respect to disturbance rejection.
