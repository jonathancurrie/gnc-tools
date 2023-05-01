classdef mxSOSFilter_tests < matlab.unittest.TestCase
    % Unit Tests for the mxSOSFilter MEX Interface to the C and C++ SOS Filters
    % J.Currie May 2023
    % (Yes I Know Actual & Expected are flipped, but it reads easier to me
    % this way)
    
    properties(Constant)
        absTol = 1e-12;       
        Fs = 100; % Sampling frequency [Hz]
        maxSections = 6;
    end

    properties(TestParameter)
        filterFcn = {@mxCSOSFilter,@mxSOSFilter};
    end

    methods(Test)
        % Test methods

        function CheckInit(testCase, filterFcn)
            % No matrix
            clear(func2str(filterFcn));
            testCase.verifyError(@() filterFcn('init'),'GNCToolsMEX:mxSOSFilter');

            % No gain
            testCase.verifyError(@() filterFcn('init',ones(1,6)),'GNCToolsMEX:mxSOSFilter');

            % Empty matrix
            testCase.verifyError(@() filterFcn('init',[],1),'GNCToolsMEX:mxSOSFilter');

            % Empty gain
            testCase.verifyError(@() filterFcn('init',ones(1,6),[]),'GNCToolsMEX:mxSOSFilter');

            % Not 6 coeffs
            testCase.verifyError(@() filterFcn('init',ones(1,5),1),'GNCToolsMEX:mxSOSFilter');

            % nrows matrix != numel gain
            testCase.verifyEqual(int8(-1), filterFcn('init',ones(1,6), [1 1 1]));
            testCase.verifyEqual(int8(-1), filterFcn('init',ones(2,6), 1));

            % a[0] != 1
            testCase.verifyEqual(int8(-1), filterFcn('init',2*ones(1,6), 1));
            testCase.verifyEqual(int8(-1), filterFcn('init',0*ones(1,6), 1));

            % Matrix contains Inf/NaN
            numSect = 2;
            for i = 1:numSect                
                for j = 1:6
                    sosMatrix = ones(numSect,6);
                    sosMatrix(i,j) = Inf;
                    testCase.verifyEqual(int8(-1), filterFcn('init',sosMatrix, ones(numSect,1)));
                    sosMatrix(i,j) = NaN;
                    testCase.verifyEqual(int8(-1), filterFcn('init',sosMatrix, ones(numSect,1)));
                end
            end

            % Gain contains Inf/NaN
            for i = 1:numSect                
                sosGain = ones(numSect,1);
                sosGain(i) = Inf;
                testCase.verifyEqual(int8(-1), filterFcn('init',ones(numSect,6), sosGain));
                
                sosGain = ones(numSect,1);
                sosGain(i) = NaN;
                testCase.verifyEqual(int8(-1), filterFcn('init',ones(numSect,6), sosGain));

                sosGain = ones(numSect+1,1);
                sosGain(end) = Inf;
                testCase.verifyEqual(int8(-1), filterFcn('init',ones(numSect,6), sosGain));

                sosGain = ones(numSect+1,1);
                sosGain(end) = NaN;
                testCase.verifyEqual(int8(-1), filterFcn('init',ones(numSect,6), sosGain));
            end

            % Greater than max number sections
            testCase.verifyEqual(int8(-1), filterFcn('init',ones(mxSOSFilter_tests.maxSections+1,6),ones(mxSOSFilter_tests.maxSections+1,1)));   
        end

        function CheckUpdate(testCase, filterFcn)
            % No init
            clear(func2str(filterFcn));
            testCase.verifyError(@() filterFcn('update',1,1),'GNCToolsMEX:mxSOSFilter'); 

            % Do init
            c = testCase.makeIIRLPF(2);
            testCase.verifyEqual(int8(0), filterFcn('init',c.SOSMatrix,c.ScaleValues));

            % Missing args
            testCase.verifyError(@() filterFcn('update'),'GNCToolsMEX:mxSOSFilter');
            
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
            c = testCase.makeIIRLPF(2);
            testCase.verifyEqual(int8(0), filterFcn('init',c.SOSMatrix,c.ScaleValues));
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
            c1 = testCase.makeMovingAverage(1);
            c2 = testCase.makeMovingAverage(3);
            c3 = testCase.makeMovingAverage(6);
            
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRLPF(testCase, filterFcn)
            
            c1 = testCase.makeIIRLPF(2);
            c2 = testCase.makeIIRLPF(4);
            c3 = testCase.makeIIRLPF(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRHPF(testCase, filterFcn)
            
            c1 = testCase.makeIIRHPF(2);
            c2 = testCase.makeIIRHPF(4);
            c3 = testCase.makeIIRHPF(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRNotch(testCase, filterFcn)
            
            c1 = testCase.makeIIRNotch(2);
            c2 = testCase.makeIIRNotch(4);
            c3 = testCase.makeIIRNotch(6);
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function IIRDenOnly(testCase, filterFcn)
            
            c1 = testCase.makeIIRDenOnly(2);
            c2 = testCase.makeIIRDenOnly(4);
            c3 = testCase.makeIIRDenOnly(6);

            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);
        end

        function NonUnityOverallGain(testCase, filterFcn)
            
            c1 = testCase.makeIIRNotch(2); c1.ScaleValues(end) = 2;
            c2 = testCase.makeIIRNotch(4); c2.ScaleValues(end) = 4;
            c3 = testCase.makeIIRNotch(6); c3.ScaleValues(end) = 6;
            
            fInputHz = 1;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);

            fInputHz = 3;
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c1, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c2, fInputHz), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testFilter(filterFcn, c3, fInputHz), 'AbsTol', testCase.absTol);
        end
    end
    
    % Test Helpers
    methods(Static)
        function c = makeMovingAverage(N)
            order = 2;         
            b = (1/(order+1))*ones(1,order+1);
            a = [1 0 0];

            c.SOSMatrix = repmat([b a],N,1);
            c.ScaleValues = ones(1,N);
        end

        function c = makeIIRLPF(N)
            Fc = 3;  % Cutoff Frequency
            
            % Construct an FDESIGN object and call its BUTTER method.
            h  = fdesign.lowpass('N,F3dB', N, Fc, mxSOSFilter_tests.Fs);
            Hd = design(h, 'butter');
            c = Hd.coeffs;
        end

        function c = makeIIRHPF(N)
            Fc = 3;  % Cutoff Frequency
            
            % Construct an FDESIGN object and call its BUTTER method.
            h  = fdesign.highpass('N,F3dB', N, Fc, mxSOSFilter_tests.Fs);
            Hd = design(h, 'butter');
            c = Hd.coeffs;
        end

        function c = makeIIRNotch(N)
            Fc1 = 0.8;  % First Cutoff Frequency
            Fc2 = 1.25;  % Second Cutoff Frequency
            h  = fdesign.bandstop('N,F3dB1,F3dB2', N, Fc1, Fc2, mxSOSFilter_tests.Fs);
            Hd = design(h, 'butter');
            c = Hd.coeffs;
        end

        function c = makeIIRDenOnly(N)
            order = 2;
            b = [1 0 0];
            a = (1/(order+1))*ones(1,order+1);
            a(1) = 1;

            c.SOSMatrix = repmat([b a],N,1);
            c.ScaleValues = ones(1,N);
        end
        
        function sse = testFilter(filterFcn, coeffs, fInputHz)
            
            % Generate Input
            t = 0:(1/mxSOSFilter_tests.Fs):5;
            u = sin(fInputHz*2*pi*t);

            % Init Filter
            status = filterFcn('Init',coeffs.SOSMatrix,coeffs.ScaleValues);
            if (status ~= int8(0))
                error("Error initializing filter");
            end

            % MATLAB Filter
            Hd = dfilt.df2sos(coeffs.SOSMatrix, coeffs.ScaleValues);
            yML = filter(Hd,u);

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