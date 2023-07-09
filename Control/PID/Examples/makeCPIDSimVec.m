function [t, r] = makeCPIDSimVec(Ts, tFinal)
% Make mxCPID time and reference vectors for simulations.

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