/**
 * @file mxCPID.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 03-Apr-2023
 * @brief MATLAB MEX Interface to the C PID Controller
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <string>
#include "Common/Source/mexHelpers.h"
#include "cpid.h"

using namespace GNCTools;

// Argument Order
#define pCMD    prhs[0]
#define pPARAMS prhs[1]
#define pR      prhs[1]
#define pY      prhs[2]
#define pU      plhs[0]
#define pSTAT0  plhs[0]
#define pSTAT1  plhs[1]

// C PID controller stored between calls
static cpidData_t pid;
static bool initCalled=false;

// Enum of possible commands
enum class Command {Unknown, Init, Update, Reset};

// Local Functions
Command checkInputs(int nrhs, const mxArray *prhs[]);

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
            pSTAT0 = MEX::createInt8(0);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT0));
            // Get inputs
            double Kp = MEX::getDoubleScalarField(pPARAMS, "Kp");
            double Ki = MEX::getDoubleScalarField(pPARAMS, "Ki");
            double Kd = MEX::getDoubleScalarField(pPARAMS, "Kd");
            double Tf = MEX::getDoubleScalarField(pPARAMS, "Tf");
            double Ts = MEX::getDoubleScalarField(pPARAMS, "Ts");
            double uMin = MEX::getDoubleScalarField(pPARAMS, "uMin");
            double uMax = MEX::getDoubleScalarField(pPARAMS, "uMax");
            double b = MEX::getDoubleScalarField(pPARAMS, "b");
            double c = MEX::getDoubleScalarField(pPARAMS, "c");

            // Call init
            *status = cpidInit(&pid, Kp, Ki, Kd, Tf, Ts, uMin, uMax, b, c);
            if (*status == CPID_SUCCESS)
            {
                initCalled = true;
            }
            break;
        }
        case Command::Update:
        {
            // Ensure the controller is initialized first
            if (initCalled == false)
            {
                MEX::error("You must initialize the controller first using mxCPID('Init',params)!");
                break;
            }

            // Create output memory
            pU     = MEX::createDoubleScalar(0.0);
            pSTAT1 = MEX::createInt8(0);
            // Get pointers
            double* u = mxGetPr(pU);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT1));
            double* r = mxGetPr(pR);
            double* y = mxGetPr(pY);    

            // Call update
            *status = cpidUpdate(&pid, *r, *y, u);
            break;
        }
        case Command::Reset:
        {
            // Create output memory
            pSTAT0 = MEX::createInt8(0);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT0));

            // Call reset
            *status = cpidReset(&pid);
            break;
        }
        default:
        {
            MEX::error("Unknown user command!");
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
        MEX::error("Usage: mxCPID('Init',params), u = mxCPID('Update',r,y), or mxCPID('Reset')");
    }
    
    // Get command
    std::string commandStr = MEX::getString(pCMD);
    commandStr = Utils::toLower(commandStr);
    if (commandStr == "init")
    {
        // Check parameters structure
        if (nrhs < 2)
        {
            MEX::error("You must supply a parameters structure as the second argument when calling 'init'");
        }
        if (MEX::isValidStruct(pPARAMS) == false)
        {
            MEX::error("The parameters argument must be a valid structure");
        }
        MEX::checkForRequiredFields(pPARAMS, {"Kp","Ki","Kd","Tf","Ts","uMin","uMax","b","c"});

        return Command::Init;
    }
    else if (commandStr == "update")
    {
        // Check r, y
        if (nrhs < 3)
        {
            MEX::error("You must supply both r (setpoint) and y (plant measurement) when calling 'update'");
        }

        return Command::Update;
    }
    else if (commandStr == "reset")
    {
        return Command::Reset;
    }
    else
    {
        MEX::error("GNCTools:mxCPID","Unknown command '%s'", commandStr.c_str());
        return Command::Unknown;
    }
}