%% Filter Examples
% J.Currie May 2023
clc
clear
% set(gcf,'position',[490   380   621   468])

% Also see Control/Filter/Tests/mxFilter_tests.m for unit testing.


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
