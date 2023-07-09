function [Gs_fit,K,wn,zeta,theta,yfit] = fitSOPDT(t, u, y, K0, wn0, zeta0, theta0, plotTitle)
% Fit Second Order Plus Dead-Time Model

if (nargin < 8), plotTitle = []; end

% Initial guess & bounds
x0 = [K0, wn0, zeta0, theta0];
lb = [-Inf, 0, 0, 0];
ub = [Inf, Inf, Inf, Inf];
y = y(:); % Ensure column

    % Local SOPDT model build
    function Gs = buildSOPDT(K, wn,zeta,theta)
        s = tf('s');
        Gs = (K * exp(-theta*s) * wn^2) / (s^2 + 2*zeta*wn*s + wn^2);
    end

    % Local objective function
    function [yfit,Gs] = objective(x0)
        Gs = buildSOPDT(x0(1), x0(2), x0(3), x0(4));
        yfit = lsim(Gs,u,t);
    end

% Use OPTI to solve
x = opti_mkltrnls(@objective, [], x0, y, lb, ub);
K = x(1);
wn = x(2);
zeta = x(3);
theta = x(4);
[yfit,Gs_fit] = objective(x);

if (~isempty(plotTitle))
    % Plot
    plot(t,y,'.-',t,yfit);
    grid on;
    ylabel('Plant Output [y]');
    xlabel('Time [s]');
    legend('Measured Data','SOPDT Fit');
    title(plotTitle);
end
end