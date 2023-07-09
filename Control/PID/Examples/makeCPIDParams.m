function params = makeCPIDParams(Kp, Ki, Kd, Ts, Tf, uMin, uMax, b, c, rRampMax)
% Build mxCPID parameters structure from supplied arguments.

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