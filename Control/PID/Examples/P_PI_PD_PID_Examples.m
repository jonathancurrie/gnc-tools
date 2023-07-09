%% P, PI, PD and PID Examples
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

%% P-Only
Kp = 0.5;
Ki = 0;
Kd = 0;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_p, y_p] = simCPID(Gs, pidParams, t, r, 0, 'P Only Control');

%% PI
Kp = 0.5;
Ki = 0.1;
Kd = 0;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_pi, y_pi] = simCPID(Gs, pidParams, t, r, 0, 'PI Control');

%% PD
Kp = 0.5;
Ki = 0;
Kd = 0.2;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_pd, y_pd] = simCPID(Gs, pidParams, t, r, 0, 'PD Control');

%% PID
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;

pidParams = makeCPIDParams(Kp, Ki, Kd, Ts);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_pid, y_pid] = simCPID(Gs, pidParams, t, r, 0, 'PID Control');

%% Comparison Plot
clf
subplot(211)
plot(t,y_p,t,y_pi,t,y_pd,t,y_pid);
hold on;
stairs(t,r,'k:');
hold off; grid on;
ylabel('Plant Output [y]');
title('Control Type Comparison');
legend('P','PI','PD','PID')

subplot(212);
stairs(t,u_p);
hold on;
stairs(t,u_pi);
stairs(t,u_pd);
stairs(t,u_pid);
hold off;
grid on;
ylabel('Control Input [u]');
xlabel('Time [s]');
