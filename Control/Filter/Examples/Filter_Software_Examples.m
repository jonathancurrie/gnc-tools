%% Filter Software Examples
% J.Currie May 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/Filter/Tests/mxFilter_tests.m for unit testing.


%% Software: FIR Moving Average
clc
clf

% 6th Order Moving Average FIR Filter
N = 6;
b = (1/(N+1))*ones(1,(N+1));
a = 1;

% Initialize MEX interface to C Filter
mxCFilter('Init',b,a)

% Filter input sine wave
Fs = 100;
t = 0:(1/Fs):0.5;
u = sin(5*2*pi*t); % 5Hz Input
y = mxCFilter('Update',u);

% Plot Output
plot(t,u,t,y)
grid on; xlabel('Time [s]'); ylabel('Magnitude')
legend('Input','Output')
title('FIR Example: Moving Average Filter')


%% Software: IIR HPF
clc
clf

% 2nd Order IIR HPF
Fs = 100;
Fc = 5; % Corner Freq [Hz]
N = 2;
h  = fdesign.highpass('N,F3dB', N, Fc, Fs);
Hd = design(h, 'butter');
[b,a] = tf(Hd);

% Initialize MEX interface to C Filter
mxCFilter('Init',b,a)

% Filter input sine wave
t = 0:(1/Fs):4;
u = sin(2*2*pi*t); % 2Hz Input
y = mxCFilter('Update',u);

% Plot Output
plot(t,u,t,y)
grid on; xlabel('Time [s]'); ylabel('Magnitude')
legend('Input','Output')
title('IIR Example: High Pass Filter')

%% Software: SOS Notch
clc
clf

% 2nd Order Notch as SOS
Fs = 100;
Fc1 = 0.8;   % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
N = 2;
h  = fdesign.bandstop('N,F3dB1,F3dB2', N, Fc1, Fc2, Fs);
Hd = design(h, 'butter');

% Initialize MEX interface to C SOS Filter using Filter Object Directly
mxCSOSFilter('Init',Hd)
% or via the coefficient structure
mxCSOSFilter('Init',Hd.coeffs)
% or via the SOS Matrix and Gain Vector individually
mxCSOSFilter('Init',Hd.sosMatrix, Hd.ScaleValues)

% Filter input sine wave
t = 0:(1/Fs):4;
u = sin(1*2*pi*t); % 1Hz Input
y = mxCSOSFilter('Update',u);

% Plot Output
plot(t,u,t,y)
grid on; xlabel('Time [s]'); ylabel('Magnitude')
legend('Input','Output')
title('SOS Example: Notch Filter')
