%% PID Examples
% J.Currie Apr 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/PID/Tests/mxCPID_tests.m for unit testing.

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

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u_p, y_p] = simPID(Gs, pidParams, t, r, 0, 'P Only Control');

%% PI
Kp = 0.5;
Ki = 0.1;
Kd = 0;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u_pi, y_pi] = simPID(Gs, pidParams, t, r, 0, 'PI Control');

%% PD
Kp = 0.5;
Ki = 0;
Kd = 0.2;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u_pd, y_pd] = simPID(Gs, pidParams, t, r, 0, 'PD Control');

%% PID
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u_pid, y_pid] = simPID(Gs, pidParams, t, r, 0, 'PID Control');

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

%% PID with Anti Windup + U Saturation
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -1;
uMax = 1;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r*5, 0, 'PID Control w U Saturation (NO Anti Windup)');

%% PID with Process Variable Only Derivative
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -Inf;
uMax = +Inf;
b = 1; % Kp setpoint weight
c = 0; % Kd setpoint weight

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u_pv, y_pv] = simPID(Gs, pidParams, t, r, 0, 'PID Control w PV Only Derivative');

%% Comparison Plot
clf
c = 0.5;
pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[u_pv05, y_pv05] = simPID(Gs, pidParams, t, r);

subplot(211)
plot(t,y_pid,t,y_pv05,t,y_pv);
hold on;
stairs(t,r,'k:');
hold off; grid on;
ylabel('Plant Output [y]');
title('PID PV Only Derivative Comparison');
legend('PID c = 1','PID c = 0.5','PID c = 0')

subplot(212);
stairs(t,u_pid);
hold on;
stairs(t,u_pv05);
stairs(t,u_pv);
hold off;
grid on;
ylabel('Control Input [u]');
xlabel('Time [s]');

%% PID with Noisy Measurements
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
noiseVar = 0.01;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Noisy Measurements');

%% PID with Noisy Measurements + Derivative Filter
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0.5;
noiseVar = 0.01;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter and Noisy Measurements');

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

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter, PV Only Derivative and Noisy Measurements');

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

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter, PV Only Derivative and Noisy Measurements RETUNED');

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
pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u_1_1, y_1_1] = simPID(Gs, pidParams, t, r);

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.5, c);
[u_05_1, y_05_1] = simPID(Gs, pidParams, t, r);

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.0, c);
[u_0_1, y_0_1] = simPID(Gs, pidParams, t, r);

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 1.0, 0.5);
[u_1_05, y_1_05] = simPID(Gs, pidParams, t, r);

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.5, 0.5);
[u_05_05, y_05_05] = simPID(Gs, pidParams, t, r);

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, 0.0, 0.5);
[u_0_05, y_0_05] = simPID(Gs, pidParams, t, r);

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

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c, rRampMax);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u,y,rOut] = simPID(Gs, pidParams, t, r);

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c);
[uNoRamp,yNoRamp] = simPID(Gs, pidParams, t, r);

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


%% PID Tuning
clc
Gz = c2d(Gs, Ts);
mlPID = pidtune(Gz,'PIDF2',1.5)
uMin = -Inf;
uMax = +Inf;
rRampMax = +Inf;
noiseVar = 0.0;

pidParams = makeControllerParams(mlPID.Kp, mlPID.Ki, mlPID.Kd, mlPID.Ts, mlPID.Tf, uMin, uMax, mlPID.b, mlPID.c, rRampMax);
[t,r] = makeTimeSetpoint(Ts,tFinal);
[u,y,rOut] = simPID(Gs, pidParams, t, r, noiseVar, 'PIDTuner Tuned');


%% Stability Analysis
clc

% Convert ML PID controller to analyzable form
% https://au.mathworks.com/help/control/ref/pid2.getcomponents.html
[Cfr,Xfr] = getComponents(mlPID,'filter')

% Build Open Loop System
CG_OL = Cfr * Gz

% And Closed Loop System
CG_CL_nom = Xfr*feedback(Gz*Cfr,1)

% Compute stability margin
[Gm,Pm] = margin(CG_OL)
margin(CG_OL);

%% Remove margin via increasing gain

stepplot(Xfr*feedback(Gz*Gm*Cfr,1), CG_CL_nom)


%% Local Functions
function [u, y, rOut] = simPID(Gs, params, t, r, noiseVar, plotTitle)

if (nargin < 5 || isempty(noiseVar)), noiseVar = 0; end
if (nargin < 6), plotTitle = []; end

% Same seed each run
rng('default');

% Discretize Plant
Gz = c2d(Gs, params.Ts);
Gzss = ss(Gz);

% Init Controller
if (mxCPID('init',params) ~= 0)
    error('Error initializing PID controller!');
end

% Create outputs
n = length(t);
u = zeros(size(t));
y = zeros(size(t));
ytrue = zeros(size(t));
rOut = zeros(size(t));
nx = size(Gzss.B,1);
x0 = zeros(nx,1);

% Simulate step by step
for i = 1:n-1
    [u(i),~,rOut(i)] = mxCPID('update', r(i), y(i));
    [Y,~,X] = lsim(Gzss,[u(i) u(i)],[t(i) t(i+1)],x0);
    y(i+1) = Y(end) + sqrt(noiseVar) * randn(1);
    ytrue(i+1) = Y(end);
    x0 = X(end,:);
end
% Copy final u
u(end) = u(end-1);

if (~isempty(plotTitle))
    % Plot
    subplot(211)
    plot(t,y);
    hold on;
    if (noiseVar ~= 0)
        plot(t,ytrue);        
    end
    stairs(t,r,'k:');
    hold off; grid on;
    ylabel('Plant Output [y]');
    title(plotTitle);
    if (noiseVar ~= 0)
        legend('y Measured','y True');
    end
    
    subplot(212);
    stairs(t,u);
    grid on;
    ylabel('Control Input [u]');
    xlabel('Time [s]');
end
end


function params = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c, rRampMax)

params = [];
params.Kp = Kp;
params.Ki = Ki;
params.Kd = Kd;
params.Ts = Ts;
if (nargin > 4), params.Tf = Tf; else params.Tf = 0; end
if (nargin > 5), params.uMin = uMin; else params.uMin = -Inf; end
if (nargin > 6), params.uMax = uMax; else params.uMax = +Inf; end
if (nargin > 7), params.b = b; else params.b = 1; end
if (nargin > 8), params.c = c; else params.c = 1; end
if (nargin > 9), params.rRampMax = rRampMax; else params.rRampMax = Inf; end

end

function [t, r] = makeTimeSetpoint(Ts, tFinal)

t = 0:Ts:tFinal;
r = ones(size(t));

% Delay initial step
tFinal8 = tFinal/8;
[~,idx] = min(abs(t - tFinal8));
r(1:idx) = 0;

% Add final step
[~,idx] = min(abs(t - tFinal8*5));
r(idx:end) = 0;

end