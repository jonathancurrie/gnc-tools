%% Derivative Filter Examples
% J.Currie Apr 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/PID/Tests/mxPID_tests.m for unit testing.

%% Sample Plant Model
% Second Order, Underdamped, Stable
clc
clf

Gs = tf(1.2, [0.5 0.3 0.2]);
step(Gs)

% Constants
Ts = 0.1;
tFinal = 60;

%% PID with Noisy Measurements
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
noiseVar = 0.01;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts);
[t,r] = makeCPIDSimVec(Ts,tFinal);
simCPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Noisy Measurements');

%% PID with Noisy Measurements + Derivative Filter
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0.5;
noiseVar = 0.01;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf);
[t,r] = makeCPIDSimVec(Ts,tFinal);
simCPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter and Noisy Measurements');

%% PID with Noisy Measurements + PV Only Derivative + Derivative Filter
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0.5;
noiseVar = 0.01;
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0; % Kd setpoint weight

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeCPIDSimVec(Ts,tFinal);
simCPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter, PV Only Derivative and Noisy Measurements');

%% PID with Noisy Measurements + PV Only Derivative + Derivative Filter RETUNED
Kp = 0.3;
Ki = 0.1;
Kd = 0.3;
Tf = 0.4;
noiseVar = 0.01;
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0.2; % Kd setpoint weight

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeCPIDSimVec(Ts,tFinal);
simCPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter, PV Only Derivative and Noisy Measurements RETUNED');
