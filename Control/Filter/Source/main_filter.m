%% C++ Filter Compilation and Testing
% J.Currie May 2023
clc
clear
clear mxFilter mxFilter

mex -v Control/Filter/Source/mxFilter.cpp Control/Filter/Source/filter.cpp  Control/Filter/Source/cfilter.c Common/Source/mexHelpers.cpp -I.
movefile('mxFilter.mexw64','Control/Filter/mxFilter.mexw64','f')

%% Unit Tests
clc
run(mxFilter_tests())

%% Test FIR
clc

% Moving average FIR
n = 6;
b = (1/n) * ones(n,1);
a = 1;

t = 0:0.1:2*pi;
u = sin(t);

y = filter(b,a,u);

mxFilter('Init',b,a)
[y2,status] = mxFilter('Update',u);

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
yj = jfilterhpf(b',a',u',128);

mxFilter('Init',b,a)
[y2,status] = mxFilter('Update',u);

subplot(211)
plot(t,u,t,y,t,yj,'o',t,y2,'x')
legend('Input','Filter','FilterHPF','CFilter')

subplot(212)
plot(t,yj'-y,t,yj'-y2)
ylabel('Error vs HPF'); xlabel('Time [s]')
legend('Filter','CFilter')

%% Test IIR num 1
clc
Ts = 0.1;
b = [1];
a = [1 0.5 0.2 0.1 0.7];

t = 0:Ts:5;
u = sin(1*2*pi*t);

y = filter(b,a,u);

mxFilter('Init',b,a)
[y2,status] = mxFilter('Update',u);

subplot(211)
plot(t,u,t,y,t,y2,'x')

subplot(212)
plot(t,y-y2)
ylabel('Error'); xlabel('Time [s]')

%% Local Functions
function Y = jfilterhpf(B,A,u,ndigits,u0,y0)
%Jonny FILTER algorithm

lenA = length(A)-1;
lenB = length(B);
n = length(u);

%Fill u0 if not specified
if(~exist('u0','var') || isempty(u0)); u0 = zeros(lenB,1);
else if(length(u0) ~= lenB); error('Invalid u0 Length'); end
end
%Fill y0 if not specified
if(~exist('y0','var') || isempty(u0)); y0 = zeros(lenA,1);
else if(length(y0) ~= lenA); error('Invalid y0 Length'); end
end
if (nargin < 4 || isempty(ndigits)), ndigits = 256; end

%Determine Maximum Coefficient Order
maxlen = max(lenA,lenB);    
%Stack Input & Output while Padding
U = hpf([zeros(maxlen-lenB,1);u0;u],ndigits);
Y = hpf([zeros(maxlen-lenA,1);y0;zeros(n,1)],ndigits);
%Create Phi
phi = hpf([-A(2:end);B]',ndigits); %assumes monic

%Simulate
for i = maxlen+1:n+maxlen
    Y(i) = phi*[Y(i-1:-1:i-lenA);U(i:-1:i-lenB+1)];
end

%Collect the result
Y = Y(maxlen+1:end);
end