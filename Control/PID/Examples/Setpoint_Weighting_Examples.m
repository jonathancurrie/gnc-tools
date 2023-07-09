%% Setpoint Weighting Examples
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

%% PID Setpoint Weighting
clf
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0;
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 1; % Kd setpoint weight
pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_1_1, y_1_1] = simCPID(Gs, pidParams, t, r);

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.5, c);
[u_05_1, y_05_1] = simCPID(Gs, pidParams, t, r);

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.0, c);
[u_0_1, y_0_1] = simCPID(Gs, pidParams, t, r);

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 1.0, 0.5);
[u_1_05, y_1_05] = simCPID(Gs, pidParams, t, r);

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.5, 0.5);
[u_05_05, y_05_05] = simCPID(Gs, pidParams, t, r);

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.0, 0.5);
[u_0_05, y_0_05] = simCPID(Gs, pidParams, t, r);

subplot(211)
plot(t,y_1_1,t,y_1_05,t,y_05_1,t,y_05_05,t,y_0_1,t,y_0_05);
hold on;
stairs(t,r,'k:');
hold off; grid on;
ylabel('Plant Output [y]');
title('PID Setpoint Weighting');
legend('PID b = 1, c = 1','PID b = 1, c = 0.5','PID b = 0.5, c = 1',...
       'PID b = 0.5, c = 0.5','PID b = 0, c = 1','PID b = 0, c = 0.5')

subplot(212);
stairs(t,u_1_1);
hold on;
stairs(t,u_1_05);
stairs(t,u_05_1);
stairs(t,u_05_05);
stairs(t,u_0_1);
stairs(t,u_0_05);
hold off;
grid on;
ylabel('Control Input [u]');
xlabel('Time [s]');
