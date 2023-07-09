%% Setpoint Ramping Examples
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

%% PID with Setpoint Ramp
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0;
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0; % Kd setpoint weight
rRampMax = 0.015;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c, rRampMax);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u,y,rOut] = simCPID(Gs, pidParams, t, r);

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[uNoRamp,yNoRamp] = simCPID(Gs, pidParams, t, r);

% Manual Plot
subplot(211)
plot(t,y,t,yNoRamp);
hold on;
stairs(t,r,'k:');
stairs(t,rOut,'k:');
hold off; grid on;
ylabel('Plant Output [y]');
title('PID Setpoint Ramp Comparison');
legend('With Ramp','No Ramp','location','best')

subplot(212);
stairs(t,u);
hold on;
stairs(t,uNoRamp);
hold off; grid on;
ylabel('Control Input [u]');
xlabel('Time [s]');
