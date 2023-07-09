%% Aliasing Examples
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
