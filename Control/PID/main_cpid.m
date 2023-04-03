%% C PID Compilation and Testing
% J.Currie April 2023
clc
clear mxCPID

mex -v Control/PID/Source/mxCPID.cpp Control/PID/Source/cpid.c Common/Source/mexHelpers.cpp -I.
movefile('mxCPID.mexw64','Control/PID','f')

%% Unit Tests
clc
run(mxCPID_tests())

%% Test against Simulink
clc
params = [];
params.Kp = 2;
params.Ki = 1.2;
params.Kd = 1;
params.Ts = 0.1;
params.Tf = 0.3;
params.c  = 0.1; % D
params.b  = 1; % K
params.umin = -1.5;
params.umax = +1.5;

mxCPID('load',params)
      

Gs = tf(1.2, [0.5 0.3 0.2]);
Gz = c2d(Gs, params.Ts); % used in Simulink
tFinal = 10;

t = (0:params.Ts:tFinal)';
r = ones(size(t)); r(1:2) = 0;
simin.time = t;  % used in Simulink
simin.signals.values = r;

% sim('pidSim');
sim('pidSimFilter');

% Sim C++ Version, using simulink y output (only interested in u)
uj = zeros(size(r));
yj = zeros(size(r));
for i = 1:length(yj)
    uj(i) = mxCPID('update',r(i), sim_y(i));
end

clf
subplot(311)
plot(t, sim_u, t, uj);
grid on; ylabel('PID Output [u]');
legend('ML','JC','location','best');

subplot(312)
plot(t, sim_u - uj);
grid on; ylabel('PID Output Difference [um-uj]');

subplot(313)
plot(t, sim_y)
hold on;
stairs(t, r, 'k--');
hold off;
grid on; ylabel('Closed Loop Output [y]');
xlabel('Time');


%% Test against MATLAB
clc
params = [];
params.Kp = 3.5;
params.Ki = 2;
params.Kd = 1.5;
params.Ts = 0.1;
params.Tf = 0.1;
params.c  = 0;
params.b  = 1;
params.uMin = -inf; % note ML pid2 doesn't support limits, only SL
params.uMax = +inf;

mxCPID('Init',params)
      
mlPID = pid2(params.Kp, params.Ki, params.Kd, params.Tf, params.b, params.c, params.Ts, ...
             'IFormula', 'ForwardEuler', 'DFormula', 'ForwardEuler');
mlPID.InputName{1} = 'r';
mlPID.InputName{2} = 'y';
mlPID.OutputName = 'u';
         
Gs = tf(1.2, [0.5 0.3 0.2]);
Gz = c2d(Gs, params.Ts);
Gz.InputName = 'u';
Gz.OutputName = 'y';
step(Gz)

clModel = connect(Gz,mlPID,'r','y','u');

t = (0:params.Ts:10)';
r = ones(size(t)); r(1:2) = 0;
[y, t, x] = lsim(clModel, r, t);

uj = zeros(size(r));
yj = zeros(size(r));
for i = 1:length(yj)
    uj(i) = mxCPID('update',r(i), y(i));
end

um = lsim(mlPID, [r y], t);

clf
subplot(311)
plot(t, um, t, uj);
grid on; ylabel('PID Output [u]');
legend('ML','JC','location','best');

subplot(312)
plot(t, um - uj);
grid on; ylabel('PID Output Difference [um-uj]');

subplot(313)
plot(t, y)
hold on;
stairs(t, r, 'k--');
hold off;
grid on; ylabel('Closed Loop Output [y]');
xlabel('Time');