%% PID Examples
% J.Currie Apr 2023
clc
clear

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
simPID(Gs, pidParams, t, r, 0, 'P Only Control');

%% PI
Kp = 0.5;
Ki = 0.1;
Kd = 0;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, 0, 'PI Control');

%% PID
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, 0, 'PID Control');

%% PID with Anti Windup + U Saturation
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -1;
uMax = 1;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, 0, 'PID Control w Anti Windup & U Saturation');

%% PID with Process Variable Only Derivative
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0; % leave filter off
uMin = -Inf;
uMax = +Inf;
c = 1; % Kp setpoint weight
b = 0; % Kd setpoint weight

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, c, b);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, 0, 'PID Control w PV Only Derivative');

%% PID with Noisy Measurements
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
noiseVar = 0.5;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Noisy Measurements');

%% PID with Noisy Measurements + Derivative Filter
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0.2;
noiseVar = 0.5;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter and Noisy Measurements');

%% PID with Noisy Measurements + PV Only Derivative + Derivative Filter
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;
Tf = 0.2;
noiseVar = 0.5;
uMin = -Inf;
uMax = +Inf;
c = 1; % Kp setpoint weight
b = 0; % Kd setpoint weight

pidParams = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, c, b);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, noiseVar, 'PID Control with Derivative Filter, PV Only Derivative and Noisy Measurements');

%% PID with Setpoint Ramp
Kp = 0.5;
Ki = 0.2;
Kd = 0.2;

pidParams = makeControllerParams(Kp, Ki, Kd, Ts);
[t,r] = makeTimeSetpoint(Ts,tFinal);
simPID(Gs, pidParams, t, r, 0, 'PID Control');


%% Local Functions
function [u, y] = simPID(Gs, params, t, r, noiseVar, plotTitle)

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
nx = size(Gzss.B,1);
x0 = zeros(nx,1);

% Simulate step by step
for i = 1:n-1
    if (i == 1)
        yprev = 0;
    else
        yprev = y(i-1);
    end
    u(i) = mxCPID('update', r(i), yprev);
    [Y,~,X] = lsim(Gzss,[u(i) u(i)],[t(i) t(i+1)],x0);
    y(i+1) = Y(end) + sqrt(noiseVar) * rand(1);
    x0 = X(end,:);
end
% Copy final u
u(end) = u(end-1);

if (~isempty(plotTitle))
    % Plot
    subplot(211)
    plot(t,y);
    hold on;
    stairs(t,r,'k:');
    hold off; grid on;
    ylabel('Plant Output [y]');
    title(plotTitle);
    
    subplot(212);
    stairs(t,u);
    grid on;
    ylabel('Control Output [u]');
    xlabel('Time [s]');
end
end


function params = makeControllerParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c)

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