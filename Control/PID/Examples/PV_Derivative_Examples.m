%% Process Variable (PV) Derivative Examples
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

%% PID with Process Variable Only Derivative
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0; % Kd setpoint weight

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_pv, y_pv] = simCPID(Gs, pidParams, t, r, 0, 'PID Control w PV Only Derivative');

%% Comparison Plot
clf
c = 1;
pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[u_pv1, y_pv1] = simCPID(Gs, pidParams, t, r);

c = 0.5;
pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[u_pv05, y_pv05] = simCPID(Gs, pidParams, t, r);

subplot(211)
plot(t,y_pv1,t,y_pv05,t,y_pv);
hold on;
stairs(t,r,'k:');
hold off; grid on;
ylabel('Plant Output [y]');
title('PID PV Only Derivative Comparison');
legend('PID c = 1','PID c = 0.5','PID c = 0')

subplot(212);
stairs(t,u_pv1);
hold on;
stairs(t,u_pv05);
stairs(t,u_pv);
hold off;
grid on;
ylabel('Control Input [u]');
xlabel('Time [s]');