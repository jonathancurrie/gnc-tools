%% GNCTools Test Data Generation
clc
clear all

%% PID Data
Gs = tf(1.2, [0.5 0.3 0.2]);

% Constants
Ts = 0.1;
tFinal = 10;

% Parameters
params = [];
params.Kp = 2;
params.Ki = 1.2;
params.Kd = 1;
params.Ts = 0.1;
params.Tf = 0.1;
params.c  = 0.1; % D
params.b  = 1; % K
params.uMin = -1.5;
params.uMax = +1.5;
params.rRampMax = Inf;
pidData.params = params;

% Discrete plant and setpoint
Gz = c2d(Gs, Ts);
Gzss = ss(Gz);
nx = size(Gzss.B,1);
x0 = zeros(nx,1);
t = 0:Ts:tFinal;
n = length(t);
pidData.r = ones(n,1);
pidData.y = zeros(n,1);
pidData.cu = zeros(n,1);
pidData.cppu = zeros(n,1);

% Init Controllers
if (mxCPID('init',pidData.params) ~= 0)
    error('Error initializing CPID controller!');
end
if (mxPID('init',pidData.params) ~= 0)
    error('Error initializing PID controller!');
end

% Simulate step by step
for i = 1:n-1
    pidData.cu(i) = mxCPID('update', pidData.r(i), pidData.y(i));
    pidData.cppu(i) = mxPID('update', pidData.r(i), pidData.y(i));
    [Y,~,X] = lsim(Gzss,[pidData.cu(i) pidData.cu(i)],[t(i) t(i+1)],x0);
    pidData.y(i+1) = Y(end);
    x0 = X(end,:);
end
% Compute final u
pidData.cu(n) = mxCPID('update', pidData.r(n), pidData.y(n));
pidData.cppu(n) = mxPID('update', pidData.r(n), pidData.y(n));

% Save to .mat
save pidData pidData
