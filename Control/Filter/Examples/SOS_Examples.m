%% Second Order Section (SOS) Examples
% J.Currie May 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/Filter/Tests/mxFilter_tests.m for unit testing.


%% SOS: Order vs Stability, Double
clc
clf

% Design Notch Filters with Increasing Order
Ts  = 0.01;
Fc1 = 0.8;  % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
h6  = fdesign.bandstop('N,F3dB1,F3dB2', 6, Fc1, Fc2, 1/Ts);
Hd6 = design(h6, 'butter');
h8  = fdesign.bandstop('N,F3dB1,F3dB2', 8, Fc1, Fc2, 1/Ts);
Hd8 = design(h8, 'butter');
h10  = fdesign.bandstop('N,F3dB1,F3dB2', 10, Fc1, Fc2, 1/Ts);
Hd10 = design(h10, 'butter');
h12  = fdesign.bandstop('N,F3dB1,F3dB2', 12, Fc1, Fc2, 1/Ts);
Hd12 = design(h12, 'butter');

% Convert each to standard IIR format
[num6,den6] = tf(Hd6);
[num8,den8] = tf(Hd8);
[num10,den10] = tf(Hd10);
[num12,den12] = tf(Hd12);

% Simulate each
t = 0:Ts:10;
u = sin(1*2*pi*t);
y6 = filter(num6,den6,u); 
y8 = filter(num8,den8,u); 
y10 = filter(num10,den10,u); 
y12 = filter(num12,den12,u);

plot(t,u,t,y6,t,y8,t,y10,t,y12)
ylim([-2 2]); grid on;
xlabel('Time [s]');
ylabel('Magnitude')
legend('Input','N=6','N=8','N=10','N=12')
title('Notch Filter Response: Double Precision IIR')

%% SOS: Order vs Stability, Single
clc
clf

% Design Notch Filters with Increasing Order
Ts  = 0.01;
Fc1 = 0.8;  % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
h2  = fdesign.bandstop('N,F3dB1,F3dB2', 2, Fc1, Fc2, 1/Ts);
Hd2 = design(h2, 'butter');
h4  = fdesign.bandstop('N,F3dB1,F3dB2', 4, Fc1, Fc2, 1/Ts);
Hd4 = design(h4, 'butter');
h6  = fdesign.bandstop('N,F3dB1,F3dB2', 6, Fc1, Fc2, 1/Ts);
Hd6 = design(h6, 'butter');

% Convert each to standard IIR format
[num2,den2] = tf(Hd2);
[num4,den4] = tf(Hd4);
[num6,den6] = tf(Hd6);

% Simulate each
t = 0:Ts:10;
u = sin(1*2*pi*t);
y2 = filter(single(num2),single(den2),single(u)); 
y4 = filter(single(num4),single(den4),single(u)); 
y6 = filter(single(num6),single(den6),single(u)); 

plot(t,u,t,y2,t,y4,t,y6)
ylim([-2 2]); grid on;
xlabel('Time [s]');
ylabel('Magnitude')
legend('Input','N=2','N=4','N=6')
title('Notch Filter Response: Single Precision IIR')

%% SOS: Order vs Stability, SOS Double
clc
clf

% Design Notch Filters with Increasing Order
Ts  = 0.01;
Fc1 = 0.8;  % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
h6  = fdesign.bandstop('N,F3dB1,F3dB2', 6, Fc1, Fc2, 1/Ts);
Hd6 = design(h6, 'butter');
h8  = fdesign.bandstop('N,F3dB1,F3dB2', 8, Fc1, Fc2, 1/Ts);
Hd8 = design(h8, 'butter');
h10  = fdesign.bandstop('N,F3dB1,F3dB2', 10, Fc1, Fc2, 1/Ts);
Hd10 = design(h10, 'butter');
h12  = fdesign.bandstop('N,F3dB1,F3dB2', 12, Fc1, Fc2, 1/Ts);
Hd12 = design(h12, 'butter');

% Simulate each in standard SOS format
t = 0:Ts:10;
u = sin(1*2*pi*t);
y6 = filter(Hd6,u); 
y8 = filter(Hd8,u); 
y10 = filter(Hd10,u); 
y12 = filter(Hd12,u);

plot(t,u,t,y6,t,y8,t,y10,t,y12)
ylim([-2 2]); grid on;
xlabel('Time [s]');
ylabel('Magnitude')
legend('Input','N=6','N=8','N=10','N=12')
title('Notch Filter Response: Double Precision SOS')
