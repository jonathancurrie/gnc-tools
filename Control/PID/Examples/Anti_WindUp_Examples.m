%% Anti WindUp Examples
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

%% PID with Anti Windup + U Saturation
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -1;
uMax = 1;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax);
[t,r] = makeCPIDSimVec(Ts,tFinal);
simCPID(Gs, pidParams, t, r*5, 0, 'PID Control w Anti WindUp & U Saturation');
