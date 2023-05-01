classdef mxFilter_tests < matlab.unittest.TestCase
    % Unit Tests for the mxFilter MEX Interface to the C and C++ Filters
    % J.Currie May 2023
    % (Yes I Know Actual & Expected are flipped, but it reads easier to me
    % this way)
    
    properties(Constant)
        absTol = 1e-12;       
        Fs = 100; % Sampling frequency [Hz]
        maxOrder = 6;
    end

    properties(TestParameter)
        filterFcn = {@mxCFilter,@mxFilter};
    end

    methods(Test)
        % Test methods

        function CheckInit(testCase, filterFcn)
            % No num
            clear(func2str(filterFcn));
            testCase.verifyError(@() filterFcn('init'),'GNCToolsMEX:mxFilter');

            % No den
            testCase.verifyError(@() filterFcn('init',1),'GNCToolsMEX:mxFilter');

            % Empty num
            testCase.verifyError(@() filterFcn('init',[],1),'GNCToolsMEX:mxFilter');

            % Empty den
            testCase.verifyError(@() filterFcn('init',1,[]),'GNCToolsMEX:mxFilter');

            % a[0] != 1
            testCase.verifyEqual(int8(-1), filterFcn('init',1,0.5));

            % Num contains Inf/NaN
            testCase.verifyEqual(int8(-1), filterFcn('init',[1 Inf],1));
            testCase.verifyEqual(int8(-1), filterFcn('init',[1 NaN],1));

            % Den contains Inf/NaN
            testCase.verifyEqual(int8(-1), filterFcn('init',1,[1 Inf]));
            testCase.verifyEqual(int8(-1), filterFcn('init',1,[1 NaN]));

            % Greater than max order
            testCase.verifyEqual(int8(-1), filterFcn('init',ones(mxFilter_tests.maxOrder+2,1),1));
            testCase.verifyEqual(int8(-1), filterFcn('init',1,ones(mxFilter_tests.maxOrder+2,1)));    
        end

        function CheckUpdate(testCase, filterFcn)
            % No init
            clear(func2str(filterFcn));
            testCase.verifyError(@() filterFcn('update',1,1),'GNCToolsMEX:mxFilter'); 

            % Do init
            [num,den] = testCase.makeMovingAverage(2);
            testCase.verifyEqual(int8(0), filterFcn('init',num,den));

            % Missing args
            testCase.verifyError(@() filterFcn('update'),'GNCToolsMEX:mxFilter');
            
            % Normal update
            [~,status] = filterFcn('update', 1);
            testCase.verifyEqual(int8(0), status);

            % Inf/NaN input
            [~,status] = filterFcn('update', Inf);
            testCase.verifyEqual(int8(-1), status);
             [~,status] = filterFcn('update', NaN);
            testCase.verifyEqual(int8(-1), status);  

            % Normal update
            [~,status] = filterFcn('update', 1);
            testCase.verifyEqual(int8(0), status);
        end

        function CheckReset(testCase, filterFcn)
            % No init OK
            testCase.verifyEqual(int8(0), filterFcn('reset'));

            % Do init and couple updates, then reset and check
            [num,den] = testCase.makeMovingAverage(2);
            testCase.verifyEqual(int8(0), filterFcn('init',num,den));
            [u0,status] = filterFcn('update', 1);
            testCase.verifyEqual(int8(0), status);
            [u1,status] = filterFcn('update', 1);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyNotEqual(u0,u1);

            testCase.verifyEqual(int8(0), filterFcn('reset'));
            [u2,status] = filterFcn('update', 1);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(u0, u2);
        end
        
        function MovingAverage(testCase, filterFcn)
            fInputHz = 1;
            [num1,den1] = testCase.makeMovingAverage(1);
            [num2,den2] = testCase.makeMovingAverage(3);
            [num3,den3] = testCase.makeMovingAverage(6);
            
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function FIRLPF(testCase, filterFcn)
            
            [num1,den1] = testCase.makeFIRLPF(3);
            [num2,den2] = testCase.makeFIRLPF(5);
            [num3,den3] = testCase.makeFIRLPF(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function FIRHPF(testCase, filterFcn)
            
            [num1,den1] = testCase.makeFIRHPF(4);
            [num2,den2] = testCase.makeFIRHPF(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRLPF(testCase, filterFcn)
            
            [num1,den1] = testCase.makeIIRLPF(2);
            [num2,den2] = testCase.makeIIRLPF(4);
            [num3,den3] = testCase.makeIIRLPF(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRHPF(testCase, filterFcn)
            
            [num1,den1] = testCase.makeIIRHPF(2);
            [num2,den2] = testCase.makeIIRHPF(4);
            [num3,den3] = testCase.makeIIRHPF(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRNotch(testCase, filterFcn)
            
            [num1,den1] = testCase.makeIIRNotch(2);
            [num2,den2] = testCase.makeIIRNotch(4);
            [num3,den3] = testCase.makeIIRNotch(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRDenOnly(testCase, filterFcn)
            
            [num1,den1] = testCase.makeIIRDenOnly(2);
            [num2,den2] = testCase.makeIIRDenOnly(4);
            [num3,den3] = testCase.makeIIRDenOnly(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num1, den1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num2, den2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, num3, den3, fInputHz), 'AbsTol', testCase.absTol);
        end
    end
    
    % Test Helpers
    methods(Static)
        function [b,a] = makeMovingAverage(N)
            b = (1/N)*ones(1,N);
            a = 1;
        end

        function [b,a] = makeFIRLPF(N)
            Fpass = 1;   % Passband Frequency
            Fstop = 3;   % Stopband Frequency
            Wpass = 1;   % Passband Weight
            Wstop = 1;   % Stopband Weight
            dens  = 20;  % Density Factor
            
            % Calculate the coefficients using the FIRPM function.
            b  = firpm(N, [0 Fpass Fstop mxFilter_tests.Fs/2]/(mxFilter_tests.Fs/2), [1 1 0 0], [Wpass Wstop], ...
                       {dens});
            Hd = dfilt.dffir(b);
            [b,a] = tf(Hd);
        end

        function [b,a] = makeFIRHPF(N)
            Fstop = 1;   % Stopband Frequency
            Fpass = 3;   % Passband Frequency
            Wstop = 1;   % Stopband Weight
            Wpass = 1;   % Passband Weight
            dens  = 20;  % Density Factor
            
            % Calculate the coefficients using the FIRPM function.
            b  = firpm(N, [0 Fstop Fpass mxFilter_tests.Fs/2]/(mxFilter_tests.Fs/2), [0 0 1 1], [Wstop Wpass], ...
                       {dens});
            Hd = dfilt.dffir(b);
            [b,a] = tf(Hd);
        end

        function [b,a] = makeIIRLPF(N)
            Fc = 3;  % Cutoff Frequency
            
            % Construct an FDESIGN object and call its BUTTER method.
            h  = fdesign.lowpass('N,F3dB', N, Fc, mxFilter_tests.Fs);
            Hd = design(h, 'butter');
            [b,a] = tf(Hd);
        end

        function [b,a] = makeIIRHPF(N)
            Fc = 3;  % Cutoff Frequency
            
            % Construct an FDESIGN object and call its BUTTER method.
            h  = fdesign.highpass('N,F3dB', N, Fc, mxFilter_tests.Fs);
            Hd = design(h, 'butter');
            [b,a] = tf(Hd);
        end

        function [b,a] = makeIIRNotch(N)
            Fc1 = 0.8;  % First Cutoff Frequency
            Fc2 = 1.25;  % Second Cutoff Frequency
            h  = fdesign.bandstop('N,F3dB1,F3dB2', N, Fc1, Fc2, mxFilter_tests.Fs);
            Hd = design(h, 'butter');
            [b,a] = tf(Hd);
        end

        function [b,a] = makeIIRDenOnly(N)
            b = 1;
            a = (1/N)*ones(1,N);
            a(1) = 1;
        end
        
        function sse = testFilter(filterFcn, num, den, fInputHz)
            
            % Generate Input
            t = 0:(1/mxFilter_tests.Fs):5;
            u = sin(fInputHz*2*pi*t);

            % Init Filter
            status = filterFcn('Init',num,den);
            if (status ~= int8(0))
                error("Error initializing filter");
            end

            % MATLAB Filter
            yML = filter(num,den,u);

            % C/C++ Filter
            yC = filterFcn('update',u);

            % Return Error
            sse = sum((yML - yC).^2);
            
            % Optional plotting
            subplot(211)
            plot(t, u, t, yML, t, yC);
            grid on; ylabel('PID Output [u]');
            legend('Input','ML','JC','location','best');
            
            subplot(212)
            plot(t, yML - yC);
            grid on; ylabel('Filter Output Difference [ym-yj]');
            title(sprintf('SSE: %g',sse));
        end
        
    end
    
end