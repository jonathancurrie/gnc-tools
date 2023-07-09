%% Tuning Guide Examples
% J.Currie Apr 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/PID/Tests/mxPID_tests.m for unit testing.

%% PID Tuning: FOPDT True Plant Model
clc
clf

% Example slightly underdamped second order system
Gs_true = tf(1.2, [0.5 0.5 0.2]);
step(Gs_true)

% Constants
Ts = 0.1;
tFinal = 60;

%% PID Tuning: FOPDT Data Gen
clc
% Actual plant we will model, discrete
Gz_true = c2d(Gs_true, Ts);

% Generate some sample "measurement data"
t = 0:Ts:tFinal;
ustep = ones(size(t)); n = length(ustep);
ustep(ceil(n/2):end) = -0.5;
rng('default'); % Same seed each run, just for Wiki examples
ymeas = lsim(Gz_true,ustep,t) + sqrt(0.01) * randn(n,1);

% Manual Plot
subplot(211)
plot(t,ymeas,'.-');
grid on;
ylabel('Plant Output [y]');
title('Sample Plant Data from Step Tests');

subplot(212);
stairs(t,ustep,'.-');
grid on;
ylabel('Plant Input [u]');
xlabel('Time [s]');

%% PID Tuning: FOPDT Fit
clc
clf
% Fit Guesses 
K0 = 6;
theta0 = 0.5;
tau0 = 2.75 - theta0;

% Note requires OPTI Toolbox: https://www.controlengineering.co.nz/Wikis/OPTI/
Gs_fit = fitFOPDT(t,ustep,ymeas,K0,tau0,theta0,'FOPDT Fit')

%% PID Tuning: FOPDT with ML PID Tuner for 2DOF-PID
clc
Gz_fit = c2d(Gs_fit, Ts);
Wc = 1; % Crossover frequency
mlPID = pidtune(Gz_fit,'PIDF2',1)
uMin = -Inf;
uMax = +Inf;
rRampMax = +Inf;
noiseVar = 0.0;

pidParams = makeCPIDParams(mlPID.Kp, mlPID.Ki, mlPID.Kd, mlPID.Ts, mlPID.Tf, uMin, uMax, mlPID.b, mlPID.c, rRampMax);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_fo,y_fo,rOut] = simCPID(Gs_fit, pidParams, t, r, noiseVar, 'PIDTuner Tuned 2-DOF PID controlling FOPDT Model');

%% Sim on real plant (magic of simulation)
clc
[u,y,rOut] = simCPID(Gs_true, pidParams, t, r, noiseVar, 'PIDTuner Tuned 2-DOF PID controlling True Model');

%% PID Tuning: SOPDT True Plant Model
clc
clf

% Example quite underdamped second order system with dead time
Gs_true = tf(0.5, [0.5 0.2 0.2],'IODelay',0.35);
step(Gs_true)

% Constants
Ts = 0.1;
tFinal = 60;

%% PID Tuning: SOPDT Data Gen
clc
% Actual plant we will model, discrete
Gz_true = c2d(Gs_true, Ts);

% Generate some sample "measurement data"
t = 0:Ts:tFinal;
ustep = ones(size(t)); n = length(ustep);
ustep(ceil(n/2):end) = -0.5;
rng('default'); % Same seed each run, just for Wiki examples
ymeas = lsim(Gz_true,ustep,t) + sqrt(0.01) * randn(n,1);

% Manual Plot
subplot(211)
plot(t,ymeas,'.-');
grid on;
ylabel('Plant Output [y]');
title('Sample Plant Data from Step Tests');

subplot(212);
stairs(t,ustep,'.-');
grid on;
ylabel('Plant Input [u]');
xlabel('Time [s]');

%% PID Tuning: SOPDT Fit
clc
clf
% Fit Guesses 
K0 = 2.5;
wn0 = 1;
zeta0 = 0.5;
theta0 = 0.3;

% Note requires OPTI Toolbox: https://www.controlengineering.co.nz/Wikis/OPTI/
[Gs_fit,K,wn,zeta,theta] = fitSOPDT(t,ustep,ymeas,K0,wn0,zeta0,theta0,'SOPDT Fit')

%% PID Tuning: SOPDT with ML PID Tuner for 2DOF-PID
clc
Gz_fit = c2d(Gs_fit, Ts);
Wc = 1.3; % Crossover frequency
mlPID = pidtune(Gz_fit,'PIDF2',Wc)
uMin = -Inf;
uMax = +Inf;
rRampMax = +Inf;
noiseVar = 0.0;

pidParams = makeCPIDParams(mlPID.Kp, mlPID.Ki, mlPID.Kd, mlPID.Ts, mlPID.Tf, uMin, uMax, mlPID.b, mlPID.c, rRampMax);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_fo,y_fo] = simCPID(Gs_fit, pidParams, t, r, noiseVar, 'PIDTuner Tuned 2-DOF PID controlling SOPDT Model');

%% Sim on real plant (magic of simulation)
clc
[u,y,rOut] = simCPID(Gs_true, pidParams, t, r, noiseVar, 'PIDTuner Tuned 2-DOF PID controlling True Model');

