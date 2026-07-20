---
title: "Digital Filtering: Software"
slug: "/control/filtering/software/"
---

In order to provide an implementation example of digital filters, GNC Tools provides the following filter software functionality:

- **Discrete**

  Required to be practically implementable on an embedded system.

- **FIR/IIR**

  Functionality for both FIR and IIR filters up to 6th order.

- **SOS**

  Functionality for Direct Form II Second Order Section filters up to 6 sections.

- **Native C**

  No external libraries other than the [C standard library](https://en.wikipedia.org/wiki/C_standard_library).

- **C++ Wrapper**

  C++ wrappers included with all functionality.

- **Documented**

  Fully documented source code suitable for [Doxygen](https://www.doxygen.nl/) auto-generated output.

- **Unit Tested**

  Validated against the MATLAB Signal Processing Toolbox [digitalFilter](https://au.mathworks.com/help/signal/ref/digitalfilter.html) class.

Clone [GNCTools](https://github.com/jonathancurrie/gnc-tools) to access the filter source, including MEX interfaces and MATLAB examples and unit tests, or alternatively jump into the Filter [Source Code](https://github.com/jonathancurrie/gnc-tools/tree/master/Control/Filter/Source) to have a look! The examples via the links above can be found in `Control/Filter/Filter_Examples.m`.

## FIR Filtering
An FIR filter can be simulated by using the MEX interface to the C implementation `mxCFilter` or the C++ wrapper `mxFilter`.

```matlab
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
```

![filter sw FIR](/img/gnc/filter_swFIR.png)

## IIR Filtering
An IIR filter can be simulated by using the MEX interface to the C implementation `mxCFilter` or the C++ wrapper `mxFilter`.

```matlab
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
```

![filter sw IIR](/img/gnc/filter_swIIR.png)

## SOS Filtering
A Cascaded Section Order Section filter can be simulated by using the MEX interface to the C implementation `mxCSOSFilter` or the C++ wrapper `mxSOSFilter`.

```matlab
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
y = mxCFilter('Update',u);

% Plot Output
plot(t,u,t,y)
grid on; xlabel('Time [s]'); ylabel('Magnitude')
legend('Input','Output')
title('SOS Example: Notch Filter')
```

![filter sw SOS](/img/gnc/filter_swSOS.png)
