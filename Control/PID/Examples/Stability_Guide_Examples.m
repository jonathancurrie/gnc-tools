%% Stability Guide Examples
% J.Currie Apr 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/PID/Tests/mxPID_tests.m for unit testing.

%% Stable System
clc
clf
Gs = zpk([],[-2 -1-2i -1+2i],1);
pzmap(Gs)

%% Show Time Domain
step(Gs)
isstable(Gs)

%% Marginally Stable System
clc
clf
Gs = zpk([],[-2 -1e-6-2i -1e-6+2i],1);
step(Gs)
isstable(Gs)

%% Marginally Unstable System
clc
clf
Gs = zpk([],[-2 -2i +2i],1);
step(Gs)
isstable(Gs)

%% Unstable System
clc
clf
Gs = zpk([],[-2 +1-2i 1+2i],1);
step(Gs)
isstable(Gs)

%% Open Loop vs Closed Loop Stability: Integrator
clc
clf

Gs = tf(1,[1 0])
step(Gs)
isstable(Gs)

%% Closed Loop Stable with PID Controller
clc
clf

Ts = 0.1;
Gz = c2d(Gs, Ts);
Wc = 1; % Crossover frequency
mlPID = pidtune(Gz,'PIDF2',Wc)

% Convert ML PID controller to analyzable form
% https://au.mathworks.com/help/control/ref/pid2.getcomponents.html
[Cfr,Xfr] = getComponents(mlPID,'filter');

% Build Open Loop System with PID Tuned Controller in series
CG_OL = Cfr*Gz;
step(CG_OL)
isstable(CG_OL)

%% Build Closed Loop System with PID Tuned Controller
clc
clf
CG_CL = Xfr*feedback(Cfr*Gz,1);

step(CG_CL)
isstable(CG_CL)

%% Frequency Domain Example System
clc

% Example Second Order System
K = 0.1;        % Low gain
wn = 1*2*pi;    % 1Hz resonance
zeta = 0.025;   % Very underdamped
Gs = tf(K*wn^2, [1 2*zeta*wn wn^2]);

% Create input sine waves at 0.5, 1, 5Hz
t = 0:0.01:30;
u_05 = sin(2*pi*0.5*t);
u_1 = sin(2*pi*1*t);
u_5 = sin(2*pi*5*t);

% Simulate Each & Plot
y_05 = lsim(Gs,u_05,t);
y_1 = lsim(Gs,u_1,t);
y_5 = lsim(Gs,u_5,t);

subplot(311)
plot(t,u_05,t,y_05)
grid on; legend('Input','Output')
title('0.5Hz Input')

subplot(312)
plot(t,u_1,t,y_1)
grid on; ylabel('Magnitude')
title('1Hz Input')

subplot(313)
plot(t,u_5,t,y_5)
grid on; xlabel('Time [s]')
title('5Hz Input')

%% Zoom 1Hz
clc
clf

plot(t,u_1,t,y_1)
grid on; ylabel('Magnitude')
title('1Hz Input')
xlim([26 30]);

gain = 1.97668/1
gain_dB = 20*log10(gain)

dt = 28.25-28.5
phase_deg = dt * (360 / 1)

%% Manual bode
clc

% Extract transfer function coeffs
num = Gs.numerator{:};
den = Gs.denominator{:};

% Substitute s = jw, evaluate tf algebraically 
w = 1*2*pi; % Eval at 1Hz
s = j*w;
y = polyval(num,s) / polyval(den,s)
gain_dB = 20*log10(abs(y))
phase_deg = rad2deg(angle(y))


%% MATLAB bode
clf
h = bodeplot(Gs);
setoptions(h,'FreqUnits','Hz'); grid on;

%% MATLAB nichols
clf
h = nicholsplot(Gs);
setoptions(h,'FreqUnits','Hz'); grid on;

%% Design Controller
clc

% Constants
Ts = 0.1;
tFinal = 60;

% Discretize Plant
Gz = c2d(Gs, Ts);

% Design Controller
Wc = 0.025*2*pi; % Crossover frequency
mlPID = pidtune(Gz,'PIDF2',Wc)
uMin = -Inf;
uMax = +Inf;
rRampMax = +Inf;

pidParams = makeCPIDParams(mlPID.Kp, mlPID.Ki, mlPID.Kd, mlPID.Ts, mlPID.Tf, uMin, uMax, mlPID.b, mlPID.c, rRampMax);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_o,y_o] = simCPID(Gs, pidParams, t, r, 0, 'PIDTuner Tuned Controller');

%% Determine Stability Margins
clc
clf
% Convert ML PID controller to analyzable form
% https://au.mathworks.com/help/control/ref/pid2.getcomponents.html
[Cfr,Xfr] = getComponents(mlPID,'filter')

% Build Open Loop System
CG_OL = Cfr * Gz

% Compute & display stability margin
margin(CG_OL);

%% Remove Gain Margin via increasing gain
clc

% Build Closed Loop System with PID Tuned Gain
CG_CL_nom = Xfr*feedback(Gz*Cfr,1);

% Build Closed Loop System with 0db Gain Margin
[Gm,Pm] = margin(CG_OL);
CG_CL_0db = Xfr*feedback(Gz*Gm*Cfr,1);

% Plot tuned and increased gain system
stepplot(CG_CL_0db, CG_CL_nom)
xlim([0 tFinal])
grid on;
legend('0db GM','PID Tuned Original')

%% Reduce Phase Margin via Dead Time
clc
clf
s = tf('s');
theta = 6; % 6 seconds dead time - heaps!!
Gs_delay = exp(-theta*s);
Gz_delay = c2d(Gs_delay, Ts);

% Build Open Loop System
CG_OL_Delay = Cfr * Gz * Gz_delay;

% Compute stability margin
margin(CG_OL_Delay);

%% Show Time Domain with reduced PM

% Build Closed Loop System with reduced phase margin
CG_CL_reducedPM = Xfr*feedback(Gz*Gz_delay*Cfr,1);

% Plot tuned and increased gain system
stepplot(CG_CL_reducedPM, CG_CL_nom)
xlim([0 tFinal])
grid on;
legend('Reduced PM','PID Tuned Original')


%% Monte Carlo Simulation Example
clc
clf

% Monte Carlo Parameter Range: [min max]
K = [0.08 0.12];      % Low gain
wn = [0.7 1.3]*2*pi;  % ~1Hz resonance
zeta = [0.02 0.03];   % Very underdamped

% Design & Simulate Nominal Controller
K_nom = mean(K);
wn_nom = mean(wn);
zeta_nom = mean(zeta);
Gs_nom = tf(K_nom*wn_nom^2, [1 2*zeta_nom*wn_nom wn_nom^2]);
Gz_nom = c2d(Gs_nom, Ts);
Wc = 0.025*2*pi; % Crossover frequency
mlPID = pidtune(Gz_nom,'PIDF2',Wc);
pidParams = makeCPIDParams(mlPID.Kp, mlPID.Ki, mlPID.Kd, mlPID.Ts, mlPID.Tf, -Inf, Inf, mlPID.b, mlPID.c);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_nom,y_nom] = simCPID(Gs_nom, pidParams, t, r);

% Monte Carlo Sim with LHS paramset
params = lhs({K,wn,zeta},100);
h = zeros(length(params),1);
Gs_mc = tf(zeros(1,1,length(params)));
for i = 1:length(params)
    % Sim controller with scattered plant model
    K = params{i}(1);
    wn = params{i}(2);
    zeta = params{i}(3);
    Gs_mc(:,:,i) = tf(K*wn^2, [1 2*zeta*wn wn^2]);
    [u,y] = simCPID(Gs_mc(:,:,i), pidParams, t, r);

    % Add to plot
    subplot(211)
    h(i) = plot(t,y,'color',[0 0.447 0.741]);
    hold on;

    subplot(212)
    stairs(t,u,'color',[0 0.447 0.741]);
    hold on;
end

% Complete Plot
subplot(211)
h(end+1) = plot(t,y_nom,'color',[0.9290 0.6940 0.1250]);
stairs(t,r,'k:');
hold off; grid on;
ylabel('Plant Output [y]');
title('Monte Carlo Simulation of PI Control of Oscillatory System');
legend([h(1),h(end)],{'Scattered','Nominal'},'location','best')

subplot(212);
stairs(t,u_nom,'color',[0.9290 0.6940 0.1250]);
hold off; grid on;
ylabel('Control Input [u]');
xlabel('Time [s]');

%% Margin of all open loop systems
clf

[Cfr,Xfr] = getComponents(mlPID,'filter');
Gz_mc = c2d(Gs_mc,Ts);
CG_OL = Cfr * Gz_mc;

h = nicholsplot(CG_OL);
setoptions(h,'FreqUnits','Hz'); grid on;

%% Examine Unstable systems
clf
margin(Cfr * Gz_mc(:,:,5))
params{5}

%% Time Domain of Unstable System
simCPID(Gs_mc(:,:,5), pidParams, t, r,0,'Unstable System Zoom');

%% Extra for Experts: Add Notch across resonance
clc
clf

% Design Notch
N   = 4;     % Order
Fc1 = 0.8;  % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
h  = fdesign.bandstop('N,F3dB1,F3dB2', N, Fc1, Fc2, 1/Ts);
Hd = design(h, 'butter');
[num,den] = tf(Hd);
GzFilt = tf(num,den,Ts);

% Bode Comparison For Interest
h = bodeplot(Gz,GzFilt,GzFilt * Gz);
setoptions(h,'FreqUnits','Hz','PhaseWrapping','off'); grid on;
legend('Plant','Filter','Plant + Filter')

%% Design Controller with Notch in Place
clc
clf
Wc = 0.1*2*pi; % Crossover frequency
mlPID = pidtune(GzFilt * Gz,'PIDF2',Wc)
uMin = -Inf;
uMax = +Inf;
rRampMax = +Inf;

pidParams = makeCPIDParams(mlPID.Kp, mlPID.Ki, mlPID.Kd, mlPID.Ts, mlPID.Tf, uMin, uMax, mlPID.b, mlPID.c, rRampMax);
[t,r] = makeCPIDSimVec(Ts,tFinal);
[u_notch,y_notch] = simCPID(GzFilt * Gz, pidParams, t, r);

% Manual Plot
subplot(211)
plot(t,y_o,t,y_notch);
hold on;
stairs(t,r,'k:');
hold off; grid on;
ylabel('Plant Output [y]');
title('PI Control of Oscillatory System');
legend('Original','With Notch Filter','location','best')

subplot(212);
stairs(t,u_o);
hold on;
stairs(t,u_notch);
hold off; grid on;
ylabel('Control Input [u]');
xlabel('Time [s]');

%% Determine Stability Margins
clc
clf
% Convert ML PID controller to analyzable form
% https://au.mathworks.com/help/control/ref/pid2.getcomponents.html
[Cfr,Xfr] = getComponents(mlPID,'filter')

% Build Open Loop System
CG_OL = Cfr * GzFilt * Gz

% Compute stability margin
[Gm,Pm] = margin(CG_OL)
h = nicholsplot(CG_OL);
setoptions(h,'FreqUnits','Hz'); grid on;

%% Local Functions
function params = lhs(inRange,minSamples)
% This function is particularly inefficient... much better ways exist!
n = length(inRange);
if (n > 16)
    error('Too many parameters!');
end
if (nargin < 2)
    minSamples = 2^n;
end

numParams = max(2^n,minSamples);
params = cell(numParams,1);
% Extremums
for i = 1:2^n
    vec = zeros(1,n);
    bin = dec2bin(i-1,n); % lazy, better way?
    for j = 1:n      
        idx = (bin(j)=='1')+1;
        vec(j) = inRange{j}(idx);
    end
    params{i} = vec;
end
% Additional uniform distribution random samples
for i = 2^n+1:numParams
    vec = zeros(1,n);
    for j = 1:n
        iRange = inRange{j};
        vec(j) = iRange(1) + (iRange(2) - iRange(1))*rand();
    end
    params{i} = vec;
end
end