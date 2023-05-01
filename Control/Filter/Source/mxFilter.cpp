/**
 * @file mxFilter.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 30-Apr-2023
 * @brief MATLAB MEX Interface to the C & C++ Digital Filters
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <string>
#include <cmath>
#include "Common/Source/mexHelpers.h"
#ifdef CFILTER
#include "cfilter.h"
#else
#include "filter.hpp"
#endif

using namespace GNCTools;

// Argument Order
#define pCMD    prhs[0]
#define pNUM    prhs[1]
#define pDEN    prhs[2]
#define pU      prhs[1]
#define pY      plhs[0]
#define pSTAT0  plhs[0]
#define pSTAT1  plhs[1]

#ifdef CFILTER
// C Filter stored between calls
static cfilterData_t filter;
static bool initCalled=false;
#else
// Filter stored between calls
static Filter filter;
#endif

// Enum of possible commands
enum class Command {Unknown, Init, Update, Reset};

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
            pSTAT0 = MEX::createInt8(CFILTER_FAILURE);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT0));
            
            // Get inputs
            double* num = mxGetPr(pNUM);
            double* den = mxGetPr(pDEN);
            uint8_t lenNum = static_cast<uint8_t>(MEX::getNumElem(pNUM));
            uint8_t lenDen = static_cast<uint8_t>(MEX::getNumElem(pDEN));
            
            #ifdef CFILTER
            // Call init
            initCalled = false;
            *status = cfilterInit(&filter, num, lenNum, den, lenDen);
            if (*status == CFILTER_SUCCESS)
            {
                initCalled = true;
            }
            #else
            // Save C arrays into C++ vectors
            std::vector<double> numVec(num, num + lenNum);
            std::vector<double> denVec(den, den + lenDen);

            // Call constructor
            filter = Filter(numVec, denVec);
            if (filter.isInitialized())
            {
                *status = CFILTER_SUCCESS;
            }
            #endif
            break;
        }
        case Command::Update:
        {
            // Ensure the filter is initialized first
            checkInitialized();

            // Check if scalar or vector evaluation
            size_t numRow = 1;
            size_t numCol = 1;
            if (MEX::isDoubleScalar(pU) == false)
            {
                numRow = MEX::getNumRows(pU);
                numCol = MEX::getNumCols(pU);
            }
            if ((numRow > 1) && (numCol > 1))
            {
                MEX::error("GNCToolsMEX:mxFilter", "This function does not accept matrices!");
            }

            // Create output memory
            pY     = MEX::createDoubleMatrix(numRow, numCol);
            pSTAT1 = MEX::createInt8(CFILTER_FAILURE);  
            // Get pointers
            double* y = mxGetPr(pY);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT1));
            double* u = mxGetPr(pU); 
            int statusSum = CFILTER_SUCCESS;

            // Call update
            for (size_t i = 0; i < numRow*numCol; i++)
            {
                #ifdef CFILTER
                statusSum += cfilterUpdate(&filter, u[i], &y[i]);
                #else
                statusSum += filter.update(u[i], y[i]);
                #endif
            }

            // Final Status
            if (statusSum == CFILTER_SUCCESS)
            {
                *status = CFILTER_SUCCESS;
                // otherwise initialized as fail
            }
            break;
        }
        case Command::Reset:
        {
            // Create output memory
            pSTAT0 = MEX::createInt8(0);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT0));

            // Call reset
            #ifdef CFILTER
            *status = cfilterReset(&filter);
            #else
            *status = filter.reset();
            #endif
            break;
        }
        default:
        {
            MEX::error("GNCToolsMEX:mxFilter", "Unknown user command!");
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
        MEX::error("GNCToolsMEX:mxFilter", "Usage: %s('Init',num,den), y = %s('Update',u), or %s('Reset')", mexName, mexName, mexName);
    }
    
    // Get command
    std::string commandStr = MEX::getString(pCMD);
    commandStr = Utils::toLower(commandStr);
    if (commandStr == "init")
    {
        // Check input args
        if (nrhs < 3)
        {
            MEX::error("GNCToolsMEX:mxFilter", "You must supply both the numerator and denominator when calling init!");
        }
        if ((MEX::isDoubleVector(pNUM) == false) && (MEX::isDoubleScalar(pNUM) == false))
        {
            MEX::error("GNCToolsMEX:mxFilter", "The numerator must be a double vector or scalar.");
        }
        if ((MEX::isDoubleVector(pDEN) == false) && (MEX::isDoubleScalar(pDEN) == false))
        {
            MEX::error("GNCToolsMEX:mxFilter", "The denominator must be a double vector or scalar.");
        }
        return Command::Init;
    }
    else if (commandStr == "update")
    {
        // Check u
        if (nrhs < 2)
        {
            MEX::error("GNCToolsMEX:mxFilter", "You must supply the input (u) when calling 'update'");
        }

        return Command::Update;
    }    
    else if (commandStr == "reset")
    {
        return Command::Reset;
    }
    else
    {
        MEX::error("GNCToolsMEX:mxFilter","Unknown command '%s'", commandStr.c_str());
        return Command::Unknown;
    }
}


//
// Check MEX Filter is Initialized
//
void checkInitialized(void)
{
    #ifdef CFILTER
    if (initCalled == false)
    #else
    if (filter.isInitialized() == false)
    #endif
    {
        MEX::error("GNCToolsMEX:mxFilter","You must initialize the filter first using %s('Init',params)!",mexFunctionName());
    }
}