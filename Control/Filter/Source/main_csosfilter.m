%% C SOS Filter Compilation and Testing
% J.Currie April 2023
clc
clear
clear mxCSOSFilter mxSOSFilter

mex -v Control/Filter/Source/mxSOSFilter.cpp Control/Filter/Source/csosfilter.c Common/Source/mexHelpers.cpp -I. -DCSOSFILTER
movefile('mxSOSFilter.mexw64','Control/Filter/mxCSOSFilter.mexw64','f')

%% Unit Tests
clc
run(mxSOSFilter_tests())

%% Test IIR
clc
% Design Notch
Ts  = 0.01;
N   = 4;     % Order
Fc1 = 0.8;  % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
h  = fdesign.bandstop('N,F3dB1,F3dB2', N, Fc1, Fc2, 1/Ts);
Hd = design(h, 'butter')

t = 0:Ts:10;
u = sin(1*2*pi*t);

y = filter(Hd,u); 

sosMatrix = Hd.sosMatrix;
sosGain = Hd.ScaleValues; 
mxCSOSFilter('Init',sosMatrix,sosGain)
[y2,status] = mxCSOSFilter('Update',u);
status

subplot(211)
plot(t,u,t,y,t,y2,'x')
legend('Input','Filter','CFilter')

subplot(212)
plot(t,y-y2)
ylabel('Error'); xlabel('Time [s]')


%% Confirm scale values
clc
% Design Notch
Ts  = 0.01;
N   = 4;     % Order
Fc1 = 0.8;  % First Cutoff Frequency
Fc2 = 1.25;  % Second Cutoff Frequency
h  = fdesign.bandstop('N,F3dB1,F3dB2', N, Fc1, Fc2, 1/Ts);
Hd = design(h, 'butter')

data = Hd.coeffs;
b1 = data.SOSMatrix(1,1:3)
a1 = data.SOSMatrix(1,4:end)
b2 = data.SOSMatrix(2,1:3)
a2 = data.SOSMatrix(2,4:end)
g = data.ScaleValues
g(end) = 0.1;

Hd(2) = dfilt.df2sos(b1,a1,b2,a2,g)

freqz(Hd)


