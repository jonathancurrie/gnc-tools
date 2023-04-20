/**
 * @file mxPID.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 04-Apr-2023
 * @brief MATLAB MEX Interface to the C & C++ PID Controllers
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <string>
#include <cmath>
#include "Common/Source/mexHelpers.h"
#ifdef CPID
#include "cpid.h"
#else
#include "pid.hpp"
#endif

using namespace GNCTools;

// Argument Order
#define pCMD    prhs[0]
#define pPARAMS prhs[1]
#define pR      prhs[1]
#define pY      prhs[2]
#define pU      plhs[0]
#define pSTAT0  plhs[0]
#define pOUTP   plhs[0]
#define pSTAT1  plhs[1]
#define pROUT   plhs[2]

#ifdef CPID
// C PID controller stored between calls
static cpidData_t pid;
static bool initCalled=false;
#else
// PID controller stored between calls
static PID pid;
#endif

// Enum of possible commands
enum class Command {Unknown, Init, Update, Reset, GetPID, GetParams};

// Local Functions
Command checkInputs(int nrhs, const mxArray *prhs[]);
void checkInitialized(void);

//
// Main MEX Interface
//
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check Inputs, will throw on error
    Command cmd = checkInputs(nrhs, prhs);

    // Act on user command
    switch (cmd)
    {
        case Command::Init:
        {
            // Create output memory
            pSTAT0 = MEX::createInt8(CPID_FAILURE);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT0));
            double Kp = 0.0;
            double Ki = 0.0;
            double Kd = 0.0;
            double Tf = 0.0;
            double Ts = 0.0;
            double uMin = -std::numeric_limits<double>::infinity(); 
            double uMax = +std::numeric_limits<double>::infinity(); 
            double b = 1.0; 
            double c = 1.0; 
            double rRampMax = +std::numeric_limits<double>::infinity(); 

            // Get inputs from parameter structure
            if (MEX::isValidStruct(pPARAMS))
            {
                Kp = MEX::getDoubleScalarField(pPARAMS, "Kp");
                Ki = MEX::getDoubleScalarField(pPARAMS, "Ki");
                Kd = MEX::getDoubleScalarField(pPARAMS, "Kd");
                Tf = MEX::getDoubleScalarField(pPARAMS, "Tf");
                Ts = MEX::getDoubleScalarField(pPARAMS, "Ts");
                uMin = MEX::getDoubleScalarField(pPARAMS, "uMin");
                uMax = MEX::getDoubleScalarField(pPARAMS, "uMax");
                b = MEX::getDoubleScalarField(pPARAMS, "b");
                c = MEX::getDoubleScalarField(pPARAMS, "c");
                rRampMax = MEX::getDoubleScalarField(pPARAMS, "rRampMax");
            }
            // Or from pid2 object
            else
            {
                const char* className = "pid2";
                Kp = MEX::getDoubleScalarProperty(pPARAMS, className, "Kp");
                Ki = MEX::getDoubleScalarProperty(pPARAMS, className, "Ki");
                Kd = MEX::getDoubleScalarProperty(pPARAMS, className, "Kd");
                Tf = MEX::getDoubleScalarProperty(pPARAMS, className, "Tf");
                b = MEX::getDoubleScalarProperty(pPARAMS, className, "b");
                c = MEX::getDoubleScalarProperty(pPARAMS, className, "c");
                Ts = MEX::getDoubleScalarProperty(pPARAMS, className, "Ts");

                if (Ts == 0.0)
                {
                    MEX::error("GNCToolsMEX:mxPID","You cannot convert from a continuous (Ts=0) pid2 controller to a %s discrete controller", mexFunctionName());
                }
            }
            
            #ifdef CPID
            // Call init
            *status = cpidInit(&pid, Kp, Ki, Kd, Tf, Ts, uMin, uMax, b, c, rRampMax);
            if (*status == CPID_SUCCESS)
            {
                initCalled = true;
            }
            #else
            // Save inputs into parameter structure
            PIDParams params{};
            params.Kp = Kp;
            params.Ki = Ki;
            params.Kd = Kd;
            params.Tf = Tf;
            params.Ts = Ts;
            params.uMin = uMin;
            params.uMax = uMax;
            params.b = b;
            params.c = c;
            params.rRampMax = rRampMax;

            // Call constructor
            pid = PID(params);
            if (pid.isInitialized())
            {
                *status = CPID_SUCCESS;
            }
            #endif
            break;
        }
        case Command::Update:
        {
            // Ensure the controller is initialized first
            checkInitialized();

            // Create output memory
            pU     = MEX::createDoubleScalar(0.0);
            pSTAT1 = MEX::createInt8(0);
            pROUT  = MEX::createDoubleScalar(0.0);   
            // Get pointers
            double* u = mxGetPr(pU);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT1));
            double* rOut = mxGetPr(pROUT);
            double* r = mxGetPr(pR);
            double* y = mxGetPr(pY);    

            // Call update
            #ifdef CPID
            *status = cpidUpdate(&pid, *r, *y, u);
            *rOut = pid.xR; // For setpoint ramp plotting
            #else
            *status = pid.update(*r, *y, *u);
            *rOut = pid.getCPIDData().xR; // For setpoint ramp plotting
            #endif
            break;
        }
        case Command::GetPID:
        {
            // Ensure the controller is initialized first
            checkInitialized();

            // Get parameter structure
            #ifdef CPID
            const cpidData_t pidDataOut = pid; // simply copy
            #else
            const cpidData_t& pidDataOut = pid.getCPIDData();
            #endif

            // Check can be converted, warn if not (allows linear analysis)
            if ((std::isinf(pidDataOut.uMin) == false) || (std::isinf(pidDataOut.uMax) == false))
            {
                MEX::warning("GNCToolsMEX:mxPID:uSatpid2","MATLAB's pid2 controller does not support u saturation, dropping from specification.");
            }
            if (std::isinf(pidDataOut.rRampMax) == false)
            {
                MEX::warning("GNCToolsMEX:mxPID:rRamppid2","MATLAB's pid2 controller does not support setpoint ramping, dropping from specification.");
            }
            
            // Create pid2 args
            mxArray* pid2args[7];
            pid2args[0] = MEX::createDoubleScalar(pidDataOut.Kp);
            pid2args[1] = MEX::createDoubleScalar(pidDataOut.Ki);
            pid2args[2] = MEX::createDoubleScalar(pidDataOut.Kd);
            pid2args[3] = MEX::createDoubleScalar(pidDataOut.Tf);
            pid2args[4] = MEX::createDoubleScalar(pidDataOut.b);
            pid2args[5] = MEX::createDoubleScalar(pidDataOut.c);
            pid2args[6] = MEX::createDoubleScalar(pidDataOut.Ts);
            // Create return object
            mexCallMATLAB(1, plhs, 7, pid2args, "pid2");
            break;
        }
        case Command::GetParams:
        {
            // Ensure the controller is initialized first
            checkInitialized();

            // Get parameter structure
            #ifdef CPID
            const cpidData_t pidDataOut = pid; // simply copy
            #else
            const cpidData_t& pidDataOut = pid.getCPIDData();
            #endif

            // Create output structure
            const char* paramNames[] = {"Kp","Ki","Kd","Tf","Ts","uMin","uMax","b","c","rRampMax"};
            pOUTP = MEX::createStruct(paramNames, 10);
            // Assign args
            MEX::createFieldDoubleScalar(pOUTP, "Kp", pidDataOut.Kp);
            MEX::createFieldDoubleScalar(pOUTP, "Ki", pidDataOut.Ki);
            MEX::createFieldDoubleScalar(pOUTP, "Kd", pidDataOut.Kd);
            MEX::createFieldDoubleScalar(pOUTP, "Tf", pidDataOut.Tf);
            MEX::createFieldDoubleScalar(pOUTP, "Ts", pidDataOut.Ts);
            MEX::createFieldDoubleScalar(pOUTP, "uMin", pidDataOut.uMin);
            MEX::createFieldDoubleScalar(pOUTP, "uMax", pidDataOut.uMax);
            MEX::createFieldDoubleScalar(pOUTP, "b", pidDataOut.b);
            MEX::createFieldDoubleScalar(pOUTP, "c", pidDataOut.c);
            MEX::createFieldDoubleScalar(pOUTP, "rRampMax", pidDataOut.rRampMax);
            break;
        }
        case Command::Reset:
        {
            // Create output memory
            pSTAT0 = MEX::createInt8(0);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT0));

            // Call reset
            #ifdef CPID
            *status = cpidReset(&pid);
            #else
            *status = pid.reset();
            #endif
            break;
        }
        default:
        {
            MEX::error("GNCToolsMEX:mxPID", "Unknown user command!");
            break;
        }
    }
}


//
// Check User Inputs, decode command
//
Command checkInputs(int nrhs, const mxArray *prhs[])
{
    if (nrhs < 1)
    {
        const char* mexName = mexFunctionName();
        MEX::error("GNCToolsMEX:mxPID", "Usage: %s('Init',params), u = %s('Update',r,y), %s('getParams'), %s('getPID'), or %s('Reset')", mexName, mexName, mexName);
    }
    
    // Get command
    std::string commandStr = MEX::getString(pCMD);
    commandStr = Utils::toLower(commandStr);
    if (commandStr == "init")
    {
        // Check parameters structure
        if (nrhs < 2)
        {
            MEX::error("GNCToolsMEX:mxPID", "You must supply a parameters structure as the second argument when calling 'init'");
        }
        if (MEX::isValidStruct(pPARAMS))
        {
            MEX::checkForRequiredFields(pPARAMS, {"Kp","Ki","Kd","Tf","Ts","uMin","uMax","b","c","rRampMax"});    
        }
        else if (MEX::isValidClass(pPARAMS, "pid2"))
        {
            MEX::checkForRequiredProperties(pPARAMS, "pid2", {"Kp","Ki","Kd","Tf","Ts","b","c"});   
        }
        else
        {
            MEX::error("GNCToolsMEX:mxPID", "The parameters argument must be a valid structure or pid2 object.");
        }
        return Command::Init;
    }
    else if (commandStr == "update")
    {
        // Check r, y
        if (nrhs < 3)
        {
            MEX::error("GNCToolsMEX:mxPID", "You must supply both r (setpoint) and y (plant measurement) when calling 'update'");
        }

        return Command::Update;
    }    
    else if (commandStr == "reset")
    {
        return Command::Reset;
    }
    else if (commandStr == "getpid")
    {
        return Command::GetPID;
    }
    else if (commandStr == "getparams")
    {
        return Command::GetParams;
    }
    else
    {
        MEX::error("GNCToolsMEX:mxPID","Unknown command '%s'", commandStr.c_str());
        return Command::Unknown;
    }
}


//
// Check MEX PID Controller is Initialized
//
void checkInitialized(void)
{
    #ifdef CPID
    if (initCalled == false)
    #else
    if (pid.isInitialized() == false)
    #endif
    {
        MEX::error("GNCToolsMEX:mxPID","You must initialize the controller first using %s('Init',params)!",mexFunctionName());
    }
}