/**
 * @file mxSOSFilter.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 01-May-2023
 * @brief MATLAB MEX Interface to the C & C++ Digital SOS Filters
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <string>
#include <cmath>
#include "Common/Source/mexHelpers.h"
#ifdef CSOSFILTER
#include "csosFilter.h"
#else
#include "sosFilter.hpp"
#endif

using namespace GNCTools;

// Argument Order
#define pCMD    prhs[0]
#define pMATRIX prhs[1]
#define pGAIN   prhs[2]
#define pU      prhs[1]
#define pY      plhs[0]
#define pSTAT0  plhs[0]
#define pSTAT1  plhs[1]

#ifdef CSOSFILTER
// C SOS Filter stored between calls
static csosFilterData_t filter;
static bool initCalled=false;
#else
// Filter stored between calls
static SOSFilter filter;
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
            pSTAT0 = MEX::createInt8(CSOSFILTER_FAILURE);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT0));
            
            // Get inputs
            double* sosGain = mxGetPr(pGAIN);
            uint8_t numSections = static_cast<uint8_t>(MEX::getNumRows(pMATRIX));
            uint8_t numGains = static_cast<uint8_t>(MEX::getNumElem(pGAIN));

            // MATLAB is column ordered, C/C++ is row ordered, need to transpose sosMatrix
            // Allocate a new Matrix the right size, then fill in from original
            mxArray* mlSOSMatrixT = MEX::createDoubleMatrix(CSOSFILTER_COEFFSPERSECTION, numSections);
            double* sosMatrixT = mxGetPr(mlSOSMatrixT);
            double *sosMatrixIn = mxGetPr(pMATRIX);
            for (uint8_t i = 0; i < numSections; i++)
            {
                for (uint8_t j = 0; j < CSOSFILTER_COEFFSPERSECTION; j++)
                {
                    sosMatrixT[i*CSOSFILTER_COEFFSPERSECTION + j] = sosMatrixIn[j*numSections + i];
                }
            }

            #ifdef CSOSFILTER
            // Call init
            initCalled = false;
            *status = csosFilterInit(&filter, sosMatrixT, numSections, sosGain, numGains);
            if (*status == CSOSFILTER_SUCCESS)
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
                *status = CSOSFILTER_SUCCESS;
            }
            #endif

            // Free memory
            mxDestroyArray(mlSOSMatrixT);
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
                MEX::error("GNCToolsMEX:mxSOSFilter", "This function does not accept matrices!");
            }

            // Create output memory
            pY     = MEX::createDoubleMatrix(numRow, numCol);
            pSTAT1 = MEX::createInt8(CSOSFILTER_FAILURE);  
            // Get pointers
            double* y = mxGetPr(pY);
            int8_t* status = static_cast<int8_t*>(mxGetData(pSTAT1));
            double* u = mxGetPr(pU); 
            int statusSum = CSOSFILTER_SUCCESS;

            // Call update
            for (size_t i = 0; i < numRow*numCol; i++)
            {
                #ifdef CSOSFILTER
                statusSum += csosFilterUpdate(&filter, u[i], &y[i]);
                #else
                statusSum += filter.update(u[i], y[i]);
                #endif
            }

            // Final Status
            if (statusSum == CSOSFILTER_SUCCESS)
            {
                *status = CSOSFILTER_SUCCESS;
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
            #ifdef CSOSFILTER
            *status = csosFilterReset(&filter);
            #else
            *status = filter.reset();
            #endif
            break;
        }
        default:
        {
            MEX::error("GNCToolsMEX:mxSOSFilter", "Unknown user command!");
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
        MEX::error("GNCToolsMEX:mxSOSFilter", "Usage: %s('Init',sosMatrix,sosGain), y = %s('Update',u), or %s('Reset')", mexName, mexName, mexName);
    }
    
    // Get command
    std::string commandStr = MEX::getString(pCMD);
    commandStr = Utils::toLower(commandStr);
    if (commandStr == "init")
    {
        // Check input args
        if (nrhs < 3)
        {
            MEX::error("GNCToolsMEX:mxSOSFilter", "You must supply both the sosMatrix and sosGain when calling init!");
        }
        if ((MEX::isDoubleMatrix(pMATRIX) == false) && (MEX::isDoubleVector(pMATRIX) == false))
        {
            MEX::error("GNCToolsMEX:mxSOSFilter", "The sosMatrix must be a double matrix or vector.");
        }
        if (MEX::getNumCols(pMATRIX) != CSOSFILTER_COEFFSPERSECTION)
        {
            MEX::error("GNCToolsMEX:mxSOSFilter", "The sosMatrix must contain %d columns.", CSOSFILTER_COEFFSPERSECTION);
        }
        if ((MEX::isDoubleVector(pGAIN) == false) && (MEX::isDoubleScalar(pGAIN) == false))
        {
            MEX::error("GNCToolsMEX:mxSOSFilter", "The sosGain must be a double vector or scalar.");
        }
        return Command::Init;
    }
    else if (commandStr == "update")
    {
        // Check u
        if (nrhs < 2)
        {
            MEX::error("GNCToolsMEX:mxSOSFilter", "You must supply the input (u) when calling 'update'");
        }

        return Command::Update;
    }    
    else if (commandStr == "reset")
    {
        return Command::Reset;
    }
    else
    {
        MEX::error("GNCToolsMEX:mxSOSFilter","Unknown command '%s'", commandStr.c_str());
        return Command::Unknown;
    }
}


//
// Check MEX Filter is Initialized
//
void checkInitialized(void)
{
    #ifdef CSOSFILTER
    if (initCalled == false)
    #else
    if (filter.isInitialized() == false)
    #endif
    {
        MEX::error("GNCToolsMEX:mxSOSFilter","You must initialize the filter first using %s('Init',sosMatrix,sosGain)!",mexFunctionName());
    }
}