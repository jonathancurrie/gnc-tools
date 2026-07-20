---
title: "PID Control: Derivative Filter"
slug: "/control/pid/derivative-filter/"
---

<small>This example follows on from the initial [PID example.](./controller-types.md)</small>

So far our examples have been quite academic as we've had no noise on our measurements. Generally this is not the case however, and the computation of our derivative can end up severely amplifying the noise! To begin, let's add some noise to our process variable measurement y and see what the controller does:

```matlab
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
noiseVar = 0.01;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Noisy Measurements');
```

![pid pidnoise](/img/gnc/pid_pidnoise.png)

As above, our setpoint tracking has obviously degraded given we can't accurately measure the plant output y (for reference, the measured output is in blue, and the "true" no-noise output is in orange using the magic of simulation), but also our control input u has also severely degraded, and would be putting the control actuator through quite the workout! A common technique that allows us to reduce the impact of measurement noise on our derivative is a derivative filter. The GNCTools PID controller includes a first order filter on the derivative that can be specified using a time constant `Tf`:

```matlab
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0.5;
noiseVar = 0.01;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter and Noisy Measurements');
```

![pid pidnoisederiv](/img/gnc/pid_pidnoisederiv.png)

While the setpoint tracking performance has slightly decreased (expected, adding a filter adds delay), we have substantially reduced the magnitude of the control input u, which will really help our actuator! One last function to add in would be to also remove the setpoint changes from the derivative estimation:

```matlab
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0.5;
noiseVar = 0.01;
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0; % Kd setpoint weight

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter, PV Only Derivative and Noisy Measurements');
```

![pid pidnoisederivpv](/img/gnc/pid_pidnoisederivpv.png)

At this point the performance is looking OK, but there is still a lot that can be improved. Some retuning can minimize the impact of the noise while retaining some of the benefits in this example:

```matlab
Kp = 0.3;
Ki = 0.1;
Kd = 0.3;
Tf = 0.2;
noiseVar = 0.01;
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0.2; % Kd setpoint weight

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter, PV Only Derivative and Noisy Measurements RETUNED');
```

![pid pidnoisederivpvretuned](/img/gnc/pid_pidnoisederivpvretuned.png)
