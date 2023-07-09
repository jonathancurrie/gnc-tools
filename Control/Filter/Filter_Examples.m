%% Filter Examples
% J.Currie May 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/Filter/Tests/mxFilter_tests.m for unit testing.


%% Aliasing: Example, Same Freq
clc
clf

% Generate "continuous" 10Hz sine wave
Fs = 1e3; % Assume 1kHz fast enough
Fsine = 10;
tFinal = 2;
t = 0:(1/Fs):tFinal;
u = sin(Fsine*2*pi*t); % remember w [rad/s] = 2*pi*f [Hz]

% Sample at same frequency of sine frequency
Fn = Fsine * 1;
tn = (0:(1/Fn):tFinal);
un = interp1(t,u,tn);

subplot(211)
plot(t,u,tn,un,'o-')
grid on; 
legend('Input Sine','Sampled Input')
title(sprintf('Sampled at %.2fx Max Freq of Interest',Fn/Fsine));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Samples')

%% Aliasing: Example, Same Freq, Phase Offset
clf

% Sample at same frequency of sine frequency, but add phase offset
Fn = Fsine * 1;
tn = (0:(1/Fn):tFinal) + 0.01;
un = interp1(t,u,tn);

subplot(211)
plot(t,u,tn,un,'o-')
grid on; 
legend('Input Sine','Sampled Input')
title(sprintf('Sampled at %.2fx Max Freq of Interest, with Phase Offset',Fn/Fsine));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Samples')

%% Aliasing: Example, Slightly Higher Freq
clf

% Sample just higher than sine frequency
Fn = Fsine * 1.05;
tn = (0:(1/Fn):tFinal);
un = interp1(t,u,tn);

subplot(211)
plot(t,u,tn,un,'o-')
grid on; 
legend('Input Sine','Sampled Input')
title(sprintf('Sampled at %.2fx Max Freq of Interest',Fn/Fsine));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Samples')

%% Aliasing: Example, Slightly Lower Freq
clf

% Sample just lower than sine frequency
Fn = Fsine * 0.95;
tn = (0:(1/Fn):tFinal);
un = interp1(t,u,tn);

subplot(211)
plot(t,u,tn,un,'o-')
grid on; 
legend('Input Sine','Sampled Input')
title(sprintf('Sampled at %.2fx Max Freq of Interest',Fn/Fsine));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Samples')

%% Aliasing: Example, At Nyquist
clf

% Sample at Nyquist Frequency
Fn = Fsine * 2;
tn = (0:(1/Fn):tFinal);
un = interp1(t,u,tn);

subplot(211)
plot(t,u,tn,un,'o-')
grid on; 
legend('Input Sine','Sampled Input')
title(sprintf('Sampled at %.2fx Max Freq of Interest',Fn/Fsine));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Samples')

%% Aliasing: Example, At 2*Nyquist
clf

% Sample at 2x Nyquist Frequency
Fn = Fsine * 4;
tn = (0:(1/Fn):tFinal);
un = interp1(t,u,tn);

subplot(211)
plot(t,u,tn,un,'o-')
grid on; 
legend('Input Sine','Sampled Input')
title(sprintf('Sampled at %.2fx Max Freq of Interest',Fn/Fsine));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Samples')

%% Aliasing: Anti-Aliasing no AA
clc
clf

% Generate "continuous" sine wave with 1Hz and 125Hz components
Fs = 1e3; % Assume 1kHz fast enough
Fsine1 = 1;
Fsine2 = 125;
tFinal = 4;
t = 0:(1/Fs):tFinal;
u = sin(Fsine1*2*pi*t) + 0.3*sin(Fsine2*2*pi*t);

% Sample at 10x max freq of interest (in this case, 10x1Hz)
Fn = Fsine1 * 10;
tn = (0:(1/Fn):tFinal) + 1e-2; % + Phase offset 
un = interp1(t,u,tn);

subplot(211)
plot(t,u,tn,un,'o-')
grid on; 
legend('Input Sine','Sampled Input','location','sw')
title(sprintf('Sampled at %.2fx Max Freq of Interest',Fn/Fsine1));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Samples')

%% Anti-Aliasing Analog LPF
clc
clf

% Anti-Aliasing Analog (continuous) LPF
N = 2; % order
Fc = 2; % 3dB corner frequency [Hz]
[b,a] = butter(N,2*pi*Fc,'s'); % Note analog filter
Gs = tf(b,a);
uAA = lsim(Gs,u,t); % Note filter before sample

% Sample at 10x max freq of interest (in this case, 10x1Hz)
Fn = Fsine1 * 10;
tn = (0:(1/Fn):tFinal);
un = interp1(t,uAA,tn);

subplot(211)
plot(t,u,t,uAA,tn,un,'o-')
grid on; 
legend('Input Signal','Analog LPF Anti-Aliased','Anti-Aliased Samples','location','sw')
title(sprintf('Sampled at %.2fx Max Freq of Interest with Analog AA LPF',Fn/Fsine1));

subplot(212)
plot(tn,un,'o-')
grid on; xlabel('Time [s]')
legend('Anti-Aliased Samples','location','sw')

%% Aliasing: Anti-Aliasing Digital LPF (only)
clc
clf

% Sample input > Nyquist Freq (>2x freq of interest)
Fn = Fsine1 * 2.5;
tn = (0:(1/Fn):tFinal);
un = interp1(t,u,tn);

% Anti-Aliasing Digital LPF at sample rate
N = 2; % order
Fc = 1.2; % 3dB corner frequency [Hz]
h  = fdesign.lowpass('N,F3dB', N, Fc, Fn); 
Hd = design(h, 'butter');

% Filter samples
ufilt = filter(Hd,un); % Note filter after sample

subplot(211)
plot(t,u,tn,un,'o-',tn,ufilt,'o-')
grid on; 
legend('Input Signal','Sampled Input','Digital LPF Anti-Aliased Samples','location','sw')
title(sprintf('Sampled at %.2fx Max Freq of Interest with Digital AA LPF',Fn/Fsine1));

subplot(212)
plot(tn,ufilt,'o-')
grid on; xlabel('Time [s]')
legend('Anti-Aliased Samples','location','sw')


%% Aliasing: Anti-Aliasing Digital LPF and Oversampling
clc
clf

% Oversampled samples with 4x Oversampling (4x Nyquist Freq)
FNyquist = Fsine1 * 2;
FOversample = FNyquist * 4;
tn = (0:(1/FOversample):tFinal) + 1e-2;
un = interp1(t,u,tn);

% Anti-Aliasing Digital LPF 
N = 2; % order
Fc = 2; % 3dB corner frequency [Hz]
h  = fdesign.lowpass('N,F3dB', N, Fc, FOversample); 
Hd = design(h, 'butter');

% Filter samples
ufilt = filter(Hd,un); % Note filter after sample

subplot(211)
plot(t,u,tn,un,'o-',tn,ufilt,'o-')
grid on; 
legend('Input Signal','Sampled Input','Anti-Aliased Samples','location','sw')
title(sprintf('Over Sampled at %.2fx Max Freq of Interest with Digital AA LPF',Fn/Fsine1));

subplot(212)
plot(tn,un,'o-',tn,ufilt,'o-')
grid on; xlabel('Time [s]')
legend('Oversampled Samples','Anti-Aliased Samples','location','sw')


%% Aliasing: Anti-Aliasing Combined Analog and Digital LPF and Oversampling
clc
clf

% Anti-Aliasing Analog (continuous) LPF via 1st Order Resistor-Capacitor
% https://www.allaboutcircuits.com/technical-articles/understanding-transfer-functions-for-low-pass-filters/
% Note: Fc [rad/s] = 1/RC
Fc = 10; % 3dB corner frequency [Hz]
R = 10e3; % Example, [ohms]
C = 1/(2*pi*Fc*R); % Capacitance [F]
Gs = tf(1,[R*C 1]);
uAA = lsim(Gs,u,t); % Note analog filter before sample

% Oversampled samples of analog filter output with 8x Oversampling (8x Nyquist Freq)
FNyquist = Fsine1 * 2;
FOversample = FNyquist * 8;
tn = (0:(1/FOversample):tFinal);
un = interp1(t,uAA,tn);

% Anti-Aliasing Digital LPF 
N = 4; % order
Fc = 2; % 3dB corner frequency [Hz]
h  = fdesign.lowpass('N,F3dB', N, Fc, FOversample); 
Hd = design(h, 'butter');

% Filter samples
ufilt = filter(Hd,un); % Note filter after sample

subplot(211)
plot(t,u,t,uAA,tn,un,'o-')
grid on; 
legend('Input Signal','Analog LPF Anti-Aliased','Over Sampled Input','location','sw')
title(sprintf('Over Sampled at %.2fx Max Freq of Interest with Analog+Digital AA LPFs',FOversample/Fsine1));

subplot(212)
plot(tn,un,'o-',tn,ufilt,'o-')
grid on; xlabel('Time [s]')
legend('Oversampled Samples','Combined Anti-Aliased Samples','location','sw')

%% PSD to confirm lumpiness
clc
clf

% PSD of each signal
[Pu,Fu] = periodogram(u,[],[],Fs,'power');
[PAA,~] = periodogram(uAA,[],[],Fs,'power');
[Pun,Fun] = periodogram(un,[],[],FOversample,'power');
[Pfilt,~] = periodogram(ufilt,[],[],FOversample,'power');

subplot(211)
plot(Fu,10*log10(Pu))
hold on
plot(Fu,10*log10(PAA))
hold off
grid on;
title('PSD Estimate')
ylabel('Power [dB]');
xlim([0 200]); ylim([-120 0])
legend('Input Signal','After Analog Filter')

subplot(212)
plot(Fun,10*log10(Pun))
hold on
plot(Fun,10*log10(Pfilt))
hold off;
grid on;
xlabel('Frequency [Hz]');
ylabel('Power [dB]');
xlim([0 8]); ylim([-60 0])
legend('Oversampled Samples','After Digital Filter')

%% FFT: Basic Usage
clc
clf

Fs = 1e3;      % Sampling Frequency [Hz]
T = 1/Fs;      % Sampling Interval [s]
L = 2000;      % Number of elements in input signal
t = (0:L-1)*T; % Time vector [s]

% Create a time series signal with 3 cosine waves, with varying amplitude and
% phase
% 0.7, 10Hz, +10deg
x = 0.7*cos(10*2*pi*t + deg2rad(10));
% 0.3, 60Hz, +30deg
x = x + 0.3*cos(60*2*pi*t + deg2rad(30));
% % 0.1, 110Hz, -45deg
x = x + 0.1*cos(110*2*pi*t - deg2rad(45));

% Perform FFT
y = fft(x);

% Compute the single sided (i.e. just positive frequencies) spectrum
P2 = abs(y/L); % double sided initially
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1); % single sided, even number of samples

% Due to numerical precision, remove small components for phase calculation
z = y;
tol = 1e-6;
z(abs(z) < tol) = 0;

% Compute single sided phase
phase = angle(z)*180/pi; % double sided initially
phase = phase(1:L/2+1);

% Single sided frequency vector (note resolution related to sample length!)
f = Fs*(0:(L/2))/L;

% Plot
subplot(211)
plot(f,P1) 
title("Single-Sided Spectrum")
ylabel("Amplitude"); grid on;

subplot(212)
plot(f,phase) 
xlabel("Frequency [Hz]")
ylabel("Phase [deg]"); grid on;
xlim([0 500])

%% Inverse ifft
clf
% Inverse fft
xI = ifft(y)

% Plot
plot(t,x,t,xI,'x')
xlim([0 0.4])
title("Inverse FFT")
ylabel("Amplitude"); grid on;
xlabel('Time [s]')
legend('Original Input Data','ifft Output')

%% Periodogram PS
clc
clf

window = [];        % rectangular window (default)
nfft = length(x);   % number of points in the fft

% Compute Power Spectrum
periodogram(x,window,nfft,Fs,'power')

% Manual Calculation of Power
% Remember Power = 2 * A^2/4 or A^2/2
cos1Power = 10*log10(0.7^2/2)
cos2Power = 10*log10(0.3^2/2)
cos3Power = 10*log10(0.1^2/2)

%% Periodgram PS vs Sampling Frequency: Broadband
clc
clf

% Generate some broadband (not just a single tone) data
xbb = chirp(t,1,0.5,100); % Linear chirp 0s = 1Hz, 0.5s = 100Hz 

% Same sampling frequency, but less samples
xbb1 = xbb(1:L/2);
xbb2 = xbb(1:L/4);

% Generate Power Spectrums
window = [];
[pxx,f] = periodogram(xbb,window,length(xbb),Fs,'power');
[pxx1,f1] = periodogram(xbb1,window,length(xbb1),Fs,'power');
[pxx2,f2] = periodogram(xbb2,window,length(xbb2),Fs,'power');

% Plot
plot(f2,10*log10(abs(pxx2)),f1,10*log10(abs(pxx1)),f,10*log10(abs(pxx)))
xlim([0 Fs2/2]); ylim([-80 0]); grid on;
xlabel('Frequency [Hz]')
ylabel('Power [dB]')
legend(sprintf('Freq Resolution %.2f Hz',mean(diff(f2))),sprintf('Freq Resolution %.2f Hz',mean(diff(f1))),sprintf('Freq Resolution %.2f Hz',mean(diff(f))))
title('Power Spectrum Estimate vs Frequency Resolution')

%% Periodgram PS vs Sampling Frequency: Pure Tones
clc
clf

% x contains our 3x pure tones from previous examples

% Same sampling frequency, but less samples
x1 = x(1:L/2);
x2 = x(1:L/4);

% Generate Power Spectrums
window = [];
[pxx,f] = periodogram(x,window,length(x),Fs,'power');
[pxx1,f1] = periodogram(x1,window,length(x1),Fs,'power');
[pxx2,f2] = periodogram(x2,window,length(x2),Fs,'power');

% Plot
plot(f2,10*log10(abs(pxx2)),'x',f1,10*log10(abs(pxx1)),'o',f,10*log10(abs(pxx)))
xlim([0 Fs2/2]); ylim([-80 0]); grid on;
xlabel('Frequency [Hz]')
ylabel('Power [dB]')
legend(sprintf('Freq Resolution %.2f Hz',mean(diff(f2))),sprintf('Freq Resolution %.2f Hz',mean(diff(f1))),sprintf('Freq Resolution %.2f Hz',mean(diff(f))))
title('Power Spectrum Estimate vs Frequency Resolution')

%% Periodogram PSD
clc
clf

window = [];        % rectangular window (default)
nfft = length(x);   % number of points in the fft

% Compute Power Spectral Density
periodogram(x,window,nfft,Fs)

% Freq Resolution
freqRes = Fs/nfft;

% Manual Calculation of Power Spectral Density (Power/freqRes)
% Remember Power = 2 * A^2/4 or A^2/2
cos1PSD = 10*log10(0.7^2/2/freqRes)
cos2PSD = 10*log10(0.3^2/2/freqRes)
cos3PSD = 10*log10(0.1^2/2/freqRes)

%% Periodgram PSD vs Sampling Frequency: Broadband
clc
clf

% xbb contains our chirp from the previos example

% Same sampling frequency, but less samples
xbb1 = xbb(1:L/2);
xbb2 = xbb(1:L/4);

% Generate Power Spectral Density Estimates
window = [];
[pxx,f] = periodogram(xbb,window,length(xbb),Fs);
[pxx1,f1] = periodogram(xbb1,window,length(xbb1),Fs);
[pxx2,f2] = periodogram(xbb2,window,length(xbb2),Fs);

% Plot
plot(f2,10*log10(abs(pxx2)),f1,10*log10(abs(pxx1)),f,10*log10(abs(pxx)))
xlim([0 Fs2/2]); ylim([-80 0]); grid on;
xlabel('Frequency [Hz]')
ylabel('Power Spectral Density [dB/Hz]')
legend(sprintf('Freq Resolution %.2f Hz',mean(diff(f2))),sprintf('Freq Resolution %.2f Hz',mean(diff(f1))),sprintf('Freq Resolution %.2f Hz',mean(diff(f))))
title('Power Spectral Density Estimate vs Frequency Resolution')

%% Periodgram PSD vs Sampling Frequency: Pure Tones
clc
clf

% x contains our 3x pure tones from previous examples

% Same sampling frequency, but less samples
x1 = x(1:L/2);
x2 = x(1:L/4);

% Generate Power Spectral Density Estimates
window = [];
[pxx,f] = periodogram(x,window,length(x),Fs);
[pxx1,f1] = periodogram(x1,window,length(x1),Fs);
[pxx2,f2] = periodogram(x2,window,length(x2),Fs);

% Plot
plot(f2,10*log10(abs(pxx2)),'x',f1,10*log10(abs(pxx1)),'o',f,10*log10(abs(pxx)))
xlim([0 Fs2/2]); ylim([-80 0]); grid on;
xlabel('Frequency [Hz]')
ylabel('Power Spectral Density [dB/Hz]')
legend(sprintf('Freq Resolution %.2f Hz',mean(diff(f2))),sprintf('Freq Resolution %.2f Hz',mean(diff(f1))),sprintf('Freq Resolution %.2f Hz',mean(diff(f))))
title('Power Spectral Density Estimate vs Frequency Resolution')

%% Spectral Leakge: nfft
clc
clf

Fs = 1e3;      % Sampling Frequency [Hz]
T = 1/Fs;      % Sampling Interval [s]
L = 2e3;       % Number of elements in input signal
t = (0:L-1)*T; % Time vector [s]

% Create a time series signal with 1x 10Hz cosine
x = cos(10*2*pi*t);

% Check periodogram defaults
window = [];
[Pxx,f] = periodogram(x,window,[],Fs,'power');
% Frequency Resolution returned
freqResHz = mean(diff(f))
% Expected Frequency Resolution from defaults
N = 2^nextpow2(length(x));
freqResExpHz = Fs/N

% With nfft = length(x)
nfft1 = length(x);
[Pxx1,f1] = periodogram(x,window,nfft1,Fs,'power');
% Frequency Resolution returned
freqResHz = mean(diff(f1))

% With much higher nfft
nfft2 = 2^14;
[Pxx2,f2] = periodogram(x,window,nfft2,Fs,'power');
% Frequency Resolution returned
freqResHz = mean(diff(f2))

% Plot Results
subplot(121)
plot(f2,10*log10(abs(Pxx2)),f,10*log10(abs(Pxx)),f1,10*log10(abs(Pxx1)))
ylim([-100 0]); xlim([0 500])
grid on; xlabel('Frequency [Hz]'); ylabel('Power [dB]')
legend(sprintf('nfft = %d',nfft2),sprintf('nfft = %d (default)',N),sprintf('nfft = length(x) = %d',nfft1))

subplot(122)
plot(f2,10*log10(abs(Pxx2)),f,10*log10(abs(Pxx)),f1,10*log10(abs(Pxx1)))
ylim([-100 0]); xlim([0 30])
grid on;  xlabel('Frequency [Hz]'); 

%% Spectral Leakge: 0 padding
clc
clf

% Use nfft = 2*length(x)
nfft = 2*length(x);

% Compute with function adding padding
window = rectwin(length(x)); % Default window
[Pxx,f] = periodogram(x,window,nfft,Fs,'power');

% Add padding manually
xPadded = [x zeros(1,length(x))];
windowPadded = [window; zeros(length(x),1)]; % Default window with padding
[Pxxp,fp] = periodogram(xPadded,windowPadded,nfft,Fs,'power');

plot(f,10*log10(abs(Pxx)),fp,10*log10(abs(Pxxp)))
grid on;  xlabel('Frequency [Hz]');  ylabel('Power [dB]')
legend('length(x) < nfft','x manually padded to nfft')
title('Periodogram Power Spectrum Estimate Comparison')
ylim([-100 0]); 

%% Padding: Benefit
clc
clf

Fs = 50;       % Sampling Frequency [Hz]
T = 1/Fs;      % Sampling Interval [s]
L = 0.8*Fs;    % Number of elements in input signal
t = (0:L-1)*T; % Time vector [s]

% Create a time series signal with 1x 2Hz cosine
x = cos(2*2*pi*t);

% len(x)
window = [];
nfft = length(x);
[Pxx,f] = periodogram(x,window,nfft,Fs,'power');

% 7*len(x)
nfft1 = 7*length(x);
[Pxx1,f1] = periodogram(x,window,nfft1,Fs,'power');

plot(f,10*log10(abs(Pxx)),f1,10*log10(abs(Pxx1)))
grid on; xlabel('Frequency [Hz]'); ylabel('Power [dB]')
legend(sprintf('nfft = length(x) = %d',nfft),sprintf('nfft = length(x) = %d',nfft1))

%% Padding: Problem

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
