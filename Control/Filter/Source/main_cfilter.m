%% C Filter Compilation and Testing
% J.Currie April 2023
clc
clear
clear mxCFilter mxFilter

mex -v Control/Filter/Source/mxFilter.cpp Control/Filter/Source/cfilter.c Common/Source/mexHelpers.cpp -I. -DCFILTER
movefile('mxFilter.mexw64','Control/Filter/mxCFilter.mexw64','f')

%% Unit Tests
clc
run(mxCFilter_tests())

%% Test FIR
clc

% Moving average FIR
n = 6;
b = (1/n) * ones(n,1);
a = 1;

t = 0:0.1:2*pi;
u = sin(t);

y = filter(b,a,u);

mxCFilter('Init',b,a)
[y2,status] = mxCFilter('Update',u);

subplot(211)
plot(t,u,t,y,t,y2,'x')

subplot(212)
plot(t,y-y2)
ylabel('Error'); xlabel('Time [s]')


%% Test IIR
clc
% Design Notch
Ts  = 0.01;
N   = 4;     % Order
Fc1 = 0.8;  % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
h  = fdesign.bandstop('N,F3dB1,F3dB2', N, Fc1, Fc2, 1/Ts);
Hd = design(h, 'butter');
[b,a] = tf(Hd);

t = 0:Ts:10;
u = sin(1*2*pi*t);

y = filter(b,a,u); %filter(Hd,u); 
yj = jfilterhpf(b',a',u');

% maxA = max(a);
% a = a ./ max(a);
% b = b ./ max(b)
maxA = max(a)
maxB = max(b)
% b = b / maxB
% a = a / maxB
mxCFilter('Init',b,a)
[y2,status] = mxCFilter('Update',u);
% y2 = y2

subplot(211)
plot(t,u,t,y,t,yj,'o',t,y2,'x')
legend('Input','Filter','FilterHPF','CFilter')

subplot(212)
plot(t,yj'-y,t,yj'-y2)
ylabel('Error'); xlabel('Time [s]')
legend('Filter','CFilter')

%% Test IIR num 1
clc
Ts = 0.1;
b = [1];
a = [1 0.5 0.2 0.1 0.7];

t = 0:Ts:5;
u = sin(1*2*pi*t);

y = filter(b,a,u);

mxCFilter('Init',b,a)
[y2,status] = mxCFilter('Update',u);

subplot(211)
plot(t,u,t,y,t,y2,'x')

subplot(212)
plot(t,y-y2)
ylabel('Error'); xlabel('Time [s]')