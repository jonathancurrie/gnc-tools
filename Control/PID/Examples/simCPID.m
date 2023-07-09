function [u, y, rOut] = simCPID(Gs, params, t, r, noiseVar, plotTitle)
% Closed-loop simulation of a mxCPID controller and tf plant.

if (nargin < 5 || isempty(noiseVar)), noiseVar = 0; end
if (nargin < 6), plotTitle = []; end

% Same seed each run
rng('default');

% Discretize Plant
if (Gs.Ts == 0)
    Gz = c2d(Gs, params.Ts);
else
    Gz = Gs;
end
Gzss = ss(Gz);
% Ensure delays within states
if (Gzss.InputDelay ~= 0 || Gzss.OutputDelay ~= 0)
    Gzss = delay2z(Gzss);
end

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