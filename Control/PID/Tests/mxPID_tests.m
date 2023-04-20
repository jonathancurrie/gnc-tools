classdef mxPID_tests < matlab.unittest.TestCase
    % Unit Tests for the mxPID MEX Interface to the C and C++ PID
    % controllers
    % J.Currie Apr 2023
    % (Yes I Know Actual & Expected are flipped, but it reads easier to me
    % this way)
    
    properties
        absTol = 1e-12;       
    end

    properties(TestParameter)
        pidFcn = {@mxCPID, @mxPID};
    end

    methods(Test)
        % Test methods

        function CheckInit(testCase, pidFcn)
            % No params
            clear(func2str(pidFcn));
            testCase.verifyError(@() pidFcn('init'),'GNCToolsMEX:mxPID');

            % Check Inf/NaN for gains
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(Inf, 0, 0, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(NaN, 0, 0, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, Inf, 0, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, NaN, 0, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, Inf, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, NaN, 0.1)));
            
            % Check 0/I/ID/D only
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(0, 0, 0, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(0, 1, 0, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(0, 0, 1, 0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(0, 1, 1, 0.1)));

            % Check Ts
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 0, 0, 0)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 0, 0, -1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 0, 0, Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 0, 0, NaN)));

            % Check Tf
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 0, 0, 0.1, 0.1))); % no D gain
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0))); % disabled OK
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, -1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, NaN)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.05)));
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.051)));

            % Check b, c
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, -0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, NaN)));
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, -0.1)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, NaN)));

            % Check uMin, uMax
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, NaN)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, 0, NaN)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, Inf, Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, -Inf, -Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, 0, 0)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, 5, 2)));
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, -Inf, Inf)));     

            % Check rRampMax
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, -Inf, Inf, Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, -Inf, Inf, -Inf)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, -Inf, Inf, NaN)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, -Inf, Inf, 0)));
            testCase.verifyEqual(int8(-1), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1, 0, 0, -Inf, Inf, -1)));
        end

        function CheckUpdate(testCase, pidFcn)
            % No init
            clear(func2str(pidFcn));
            testCase.verifyError(@() pidFcn('update',1,1),'GNCToolsMEX:mxPID'); 

            % Do init
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1)));

            % Missing args
            testCase.verifyError(@() pidFcn('update'),'GNCToolsMEX:mxPID');
            testCase.verifyError(@() pidFcn('update', 1),'GNCToolsMEX:mxPID');
            
            % Get current control
            [~,status] = pidFcn('update', 1, 0);
            testCase.verifyEqual(int8(0), status);

            % Inf/NaN setpoint
            [~,status] = pidFcn('update', Inf, 0);
            testCase.verifyEqual(int8(-1), status);
             [~,status] = pidFcn('update', NaN, 0);
            testCase.verifyEqual(int8(-1), status);

            % Inf/NaN measurement
            [~,status] = pidFcn('update', 1, Inf);
            testCase.verifyEqual(int8(-1), status);
             [~,status] = pidFcn('update', 1, NaN);
            testCase.verifyEqual(int8(-1), status);                
        end

        function CheckReset(testCase, pidFcn)
            % No init OK
            testCase.verifyEqual(int8(0), pidFcn('reset'));

            % Do init and couple updates, then reset and check
            testCase.verifyEqual(int8(0), pidFcn('init',testCase.makeTestControllerParams(1, 1, 1, 0.1, 0.1)));
            [u0,status] = pidFcn('update', 1, 0);
            testCase.verifyEqual(int8(0), status);
            [u1,status] = pidFcn('update', 1, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyNotEqual(u0,u1);

            testCase.verifyEqual(int8(0), pidFcn('reset'));
            [u2,status] = pidFcn('update', 1, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(u0, u2);
        end

        function CheckGetParams(testCase, pidFcn)
            % No init
            clear(func2str(pidFcn));
            testCase.verifyError(@() pidFcn('getParams'),'GNCToolsMEX:mxPID'); 

            % Check each param
            testParams = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0.3, 0.7, 0.4, -0.4, 0.5, 10.5);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));

            retParams = pidFcn('getParams');
            testCase.verifyEqual(testParams.Kp, retParams.Kp);
            testCase.verifyEqual(testParams.Ki, retParams.Ki);
            testCase.verifyEqual(testParams.Kd, retParams.Kd);
            testCase.verifyEqual(testParams.Tf, retParams.Tf);
            testCase.verifyEqual(testParams.Ts, retParams.Ts);
            testCase.verifyEqual(testParams.uMin, retParams.uMin);
            testCase.verifyEqual(testParams.uMax, retParams.uMax);
            testCase.verifyEqual(testParams.b, retParams.b);
            testCase.verifyEqual(testParams.c, retParams.c);
            testCase.verifyEqual(testParams.rRampMax, retParams.rRampMax);
        end

        function CheckGetPID(testCase, pidFcn)
            % No init
            clear(func2str(pidFcn));
            testCase.verifyError(@() pidFcn('getPID'),'GNCToolsMEX:mxPID'); 

            % Check each compatible param - full PID
            testParams = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0.3, 0.7, 0.4, -Inf, Inf, Inf);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));

            % Local checking function
            function verifyPIDParams(testCase, testParams, retPID)
                testCase.verifyEqual(testParams.Kp, retPID.Kp);
                testCase.verifyEqual(testParams.Ki, retPID.Ki);
                testCase.verifyEqual(testParams.Kd, retPID.Kd);
                testCase.verifyEqual(testParams.Tf, retPID.Tf);
                testCase.verifyEqual(testParams.Ts, retPID.Ts);
                testCase.verifyEqual(testParams.b, retPID.b);
                testCase.verifyEqual(testParams.c, retPID.c);
            end

            retPID = pidFcn('getPID');           
            verifyPIDParams(testCase, testParams, retPID);

            % Check P/PI/PD
            testParams = testCase.makeTestControllerParams(2.5, 0, 0, 0.1);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));
            retPID = pidFcn('getPID');          
            verifyPIDParams(testCase, testParams, retPID);

            testParams = testCase.makeTestControllerParams(2.5, 1.2, 0, 0.1);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));
            retPID = pidFcn('getPID');        
            verifyPIDParams(testCase, testParams, retPID);

            testParams = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.1);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));
            retPID = pidFcn('getPID');         
            verifyPIDParams(testCase, testParams, retPID);

            % Check warnings for uMin/uMax/rRampMax
            testParams = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.1, 0, 1, 1, -0.5);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));
            testCase.verifyWarning(@() pidFcn('getPID'),'GNCToolsMEX:mxPID:uSatpid2'); 

            testParams = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.1, 0, 1, 1, -Inf, 0.5);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));
            testCase.verifyWarning(@() pidFcn('getPID'),'GNCToolsMEX:mxPID:uSatpid2'); 

            testParams = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.1, 0, 1, 1, -Inf, Inf, 0.5);
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));
            testCase.verifyWarning(@() pidFcn('getPID'),'GNCToolsMEX:mxPID:rRamppid2'); 
        end
        
        function POnly(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 0, 0, 0.1);
            testParams2 = testCase.makeTestControllerParams(2.5, 0, 0, 0.15);
            testParams3 = testCase.makeTestControllerParams(0.5, 0, 0, 0.01);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function POnlyWithB(testCase, pidFcn)
            Gs = tf(1.25, [0.45 0.35 0.21]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 0, 0, 0.1, 0, 0.9);
            testParams2 = testCase.makeTestControllerParams(2.5, 0, 0, 0.15, 0, 0.8);
            testParams3 = testCase.makeTestControllerParams(0.5, 0, 0, 0.01, 0, 0.7);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end
        
        function PIOnly(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 0.1, 0, 0.1);
            testParams2 = testCase.makeTestControllerParams(2.5, 0.3, 0, 0.15);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.4, 0, 0.01);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end
        
        function PIOnlyWithB(testCase, pidFcn)
            Gs = tf(1.25, [0.45 0.35 0.21]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 0.1, 0, 0.1, 0, 0.9);
            testParams2 = testCase.makeTestControllerParams(2.5, 0.3, 0, 0.15, 0, 0.8);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.4, 0, 0.01, 0, 0.7);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function PIOnlyAntiWindUp(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 0.1, 0, 0.1, 0, 1, 1, -0.5, 0.5);
            testParams2 = testCase.makeTestControllerParams(2.5, 0.3, 0, 0.15, 0, 1, 1, -0.25, 0.55);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.4, 0, 0.01, 0, 1, 1, 0, 1.5);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function PIOnlyWithBAndAntiWindUp(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 0.1, 0, 0.1, 0, 0.9, 1, -0.5, 0.5);
            testParams2 = testCase.makeTestControllerParams(2.5, 0.3, 0, 0.15, 0, 0.8, 1, -0.25, 0.55);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.4, 0, 0.01, 0, 0.7, 1, 0, 1.5);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        
        function PIDBasic(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end
        
        function PIDWithC(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0, 1, 0.8);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0, 1, 0.3);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01, 0, 1, 0.0);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function PIDWithBAndC(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0, 0.9, 0.8);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0, 0.7, 0.3);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01, 0, 0.6, 0.0);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function PIDWithBAndCAndAntiWindUp(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0, 0.9, 0.8, -2, 2);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0, 0.7, 0.3, -0.5, 0.5);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01, 0, 0.6, 0.0, 0.0, 0.2);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function PIDOnlyWithDFilt(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0.2);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0.3);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01, 0.05);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function PIDOnlyWithDFiltAndC(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0.2, 1, 0.8);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0.3, 1, 0.4);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01, 0.05, 1, 0.0);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end

        function PIDOnlyWithDFiltAndCAndB(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0.2, 0.9, 0.8);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0.3, 0.7, 0.4);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01, 0.05, 0.6, 0.0);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end
        
        function PIDOnlyWithDFiltAndCAndBAndAntiWindUp(testCase, pidFcn)
            Gs = tf(1.2, [0.5 0.3 0.2]);
            tFinal = 10;
            testParams1 = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0.2, 0.9, 0.8, -2, 2);
            testParams2 = testCase.makeTestControllerParams(2.5, 1.2, 1.5, 0.15, 0.3, 0.7, 0.4, -0.4, 0.5);
            testParams3 = testCase.makeTestControllerParams(0.5, 0.6, 0.5, 0.01, 0.05, 0.6, 0.0, 0.0, 0.3);
            
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams1, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams2, tFinal), 'AbsTol', testCase.absTol);
            testCase.verifyEqual(0, testCase.testController(pidFcn, Gs, testParams3, tFinal), 'AbsTol', testCase.absTol);
        end 

        function PIDSetpointRamp(testCase, pidFcn)            
            rRampMax = 0.3;
            r = 1;
            testParams = testCase.makeTestControllerParams(1.5, 1, 1.2, 0.1, 0, 1, 1, -Inf, Inf, rRampMax);

            % Do init
            testCase.verifyEqual(int8(0), pidFcn('init',testParams));
            
            % Do all updates one by one ramping up
            [~,status,rOut] = pidFcn('update', r, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(rRampMax, rOut, 'AbsTol', testCase.absTol);

            [~,status,rOut] = pidFcn('update', r, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(rRampMax*2, rOut, 'AbsTol', testCase.absTol);

            [~,status,rOut] = pidFcn('update', r, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(rRampMax*3, rOut, 'AbsTol', testCase.absTol);

            [~,status,rOut] = pidFcn('update', r, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(r, rOut);

            % Now ramping back down
            [~,status,rOut] = pidFcn('update', 0, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(r-rRampMax, rOut, 'AbsTol', testCase.absTol);

            [~,status,rOut] = pidFcn('update', 0, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(r-rRampMax*2, rOut, 'AbsTol', testCase.absTol);

            [~,status,rOut] = pidFcn('update', 0, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(r-rRampMax*3, rOut, 'AbsTol', testCase.absTol);

            [~,status,rOut] = pidFcn('update', 0, 0);
            testCase.verifyEqual(int8(0), status);
            testCase.verifyEqual(0, rOut);
        end
    end
    
    % Test Helpers
    methods(Static)
        function params = makeTestControllerParams(Kp, Ki, Kd, Ts, Tf, b, c, uMin, uMax, rRampMax)
            params.Kp = Kp;
            params.Ki = Ki;
            params.Kd = Kd;
            params.Ts = Ts;
            if (nargin > 4), params.Tf = Tf; else, params.Tf = 0; end
            if (nargin > 5), params.b = b; else, params.b = 1; end
            if (nargin > 6), params.c = c; else, params.c = 1; end
            if (nargin > 7), params.uMin = uMin; else, params.uMin = -inf; end
            if (nargin > 8), params.uMax = uMax; else, params.uMax = +inf; end
            if (nargin > 9), params.rRampMax = rRampMax; else, params.rRampMax = +inf; end
        end
        
        function sse = testController(pidFcn, Gs, params, tFinal)
            
            % Init C PID Controller
            pidFcn('Init',params);
    
            % Discretize incoming plant
            Gz = c2d(Gs, params.Ts); %#ok<NASGU>
            
            % Simulate Simulink closed loop response
            t = (0:params.Ts:tFinal)';
            r = ones(size(t)); r(1:2) = 0;
            simin.time = t;
            simin.signals.values = r; %#ok<STRNU>
            if (params.Tf > 0)
                sim('pidSimFilter', [0 tFinal], simset('SrcWorkspace','current'));
            else
                sim('pidSim', [0 tFinal], simset('SrcWorkspace','current'));
            end

            % Simulate C++ "closed loop" response
            uj = zeros(size(r));
            for i = 1:length(uj)
                uj(i) = pidFcn('update',r(i), sim_y(i));
            end
            sse = sum((sim_u - uj).^2);
            
            % Optional plotting
            subplot(311)
            plot(t, sim_u, t, uj);
            grid on; ylabel('PID Output [u]');
            legend('ML','JC','location','best');
            
            subplot(312)
            plot(t, sim_u - uj);
            grid on; ylabel('PID Output Difference [um-uj]');
            
            subplot(313)
            plot(t, sim_y)
            hold on;
            stairs(t, r, 'k--');
            hold off;
            grid on; ylabel('Closed Loop Output [y]');
            xlabel('Time');
        end
        
    end
    
end