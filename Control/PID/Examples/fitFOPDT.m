function [Gs_fit,K,tau,theta,yfit] = fitFOPDT(t, u, y, K0, tau0, theta0, plotTitle)
% Fit First Order Plus Dead-Time Model

if (nargin < 7), plotTitle = []; end

% Initial guess & bounds
x0 = [K0, tau0, theta0];
lb = [-Inf, 0, 0];
ub = [Inf, Inf, Inf];
y = y(:); % Ensure column

    % Local FOPDT model build
    function Gs = buildFOPDT(K,tau,theta)
        s = tf('s');
        Gs = (K * exp(-theta*s)) / (tau*s + 1);
    end

    % Local objective function
    function [yfit,Gs] = objective(x0)
        Gs = buildFOPDT(x0(1), x0(2), x0(3));
        yfit = lsim(Gs,u,t);
    end

% Use OPTI to solve
x = opti_mkltrnls(@objective, [], x0, y, lb, ub);
K = x(1);
tau = x(2);
theta = x(3);
[yfit,Gs_fit] = objective(x);

if (~isempty(plotTitle))
    % Plot
    plot(t,y,'.-',t,yfit);
    grid on;
    ylabel('Plant Output [y]');
    xlabel('Time [s]');
    legend('Measured Data','FOPDT Fit');
    title(plotTitle);
end
end