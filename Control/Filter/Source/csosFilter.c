/**
 * @file csosFilter.c
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 01-May-2023
 * @brief Discrete Second Order Section Filter Implementation.
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <math.h>
#include <assert.h>
#include "csosFilter.h"

//
// Local Function Declarations
//
int8_t isValidReal(real_t num);
int8_t secondOrderIIRFilter(const real_t* sosNum, const real_t* sosDen, real_t gain,
                            real_t* uShift, real_t* yShift, real_t* inOut);


//
// Filter Update
//
int8_t csosFilterUpdate(csosFilterData_t* filter, real_t u, real_t* y)
{
    // Input pointer checks
    if (filter == NULL)
    {
        return CSOSFILTER_FAILURE;
    }
    if (y == NULL)
    {
        return CSOSFILTER_FAILURE;
    }

    // Input validity checks
    if (isValidReal(u) != CSOSFILTER_SUCCESS)
    {
        return CSOSFILTER_FAILURE;
    }

    // Section by section difference equation evaluation
    uint8_t i = 0;
    int8_t retCode = CSOSFILTER_SUCCESS;
    *y = u;
    for (i = 0; i < filter->numSections; i++)
    {                
        const uint8_t numIdx = 0;
        const uint8_t denIdx = CSOSFILTER_SECTIONORDER + 1;
        const uint8_t shiftIdx = i * CSOSFILTER_SECTIONORDER;
        retCode += secondOrderIIRFilter(&filter->sosMatrix[i][numIdx], 
                                        &filter->sosMatrix[i][denIdx], 
                                        filter->sosGain[i], 
                                        &filter->uShift[shiftIdx], 
                                        &filter->yShift[shiftIdx], 
                                        y);
    }

    // Final overall gain
    *y *= filter->overallGain;    
    return retCode;
}

//
// Second Order IIR Filter Discrete Update
//
int8_t secondOrderIIRFilter(const real_t* sosNum, const real_t* sosDen, real_t gain,
                            real_t* uShift, real_t* yShift, real_t* inOut)
{
    static_assert(CSOSFILTER_SECTIONORDER == 2, "Require CSOSFILTER_SECTIONORDER to be 2");

    // Input pointer checks
    if (sosNum == NULL || sosDen == NULL || uShift == NULL || yShift == NULL || inOut == NULL)
    {
        return CSOSFILTER_FAILURE;
    }
    // As local function (only), skip other checks, assume done by calling function

    // Unroll for-loops explicitly as we know the order
    real_t u = *inOut;
    // num = b[0]*u[k] + b[1]*u[k-1] + b[2]*u[k-2]
    real_t yNum = sosNum[0]*u + sosNum[1]*uShift[0] + sosNum[2]*uShift[1];
    // den = a[1]*y[k-1] + a[2]*y[k-2]
    real_t yDen = sosDen[1]*yShift[0] + sosDen[2]*yShift[1];
    // a[0]*y = num - den
    real_t y = yNum - yDen;

    // Update shift registers, unrolled as above
    uShift[1] = uShift[0]; uShift[0] = u;
    yShift[1] = yShift[0]; yShift[0] = y;

    // Add section gain
    *inOut = gain * y;
    return CSOSFILTER_SUCCESS;
}    

//
// Filter Data Structure Initialization
//
int8_t csosFilterInit(csosFilterData_t* filter, const real_t* sosMatrix, 
                      uint8_t numSections, const real_t* sosGain, uint8_t numGains)
{
    // Assert we can sum the failure code (compile time check)
    static_assert(CSOSFILTER_FAILURE != 0, "Require CSOSFILTER_FAILURE to be != 0");

    // Input pointer checks
    if (filter == NULL)
    {
        return CSOSFILTER_FAILURE;
    }
    if (sosMatrix == NULL)
    {
        return CSOSFILTER_FAILURE;
    }
    if (sosGain == NULL)
    {
        return CSOSFILTER_FAILURE;
    }

    // Check min and max matrix size
    if (numSections > CSOSFILTER_MAXSECTIONS)
    {
        return CSOSFILTER_FAILURE;
    }
    if ((numSections == 0) || (numGains == 0))
    {
        return CSOSFILTER_FAILURE;
    }

    // Check gain size (can be numSections, or numSections+1)
    if ((numGains != numSections) && (numGains-1 != numSections))
    {
        return CSOSFILTER_FAILURE;
    }

    // Ensure all a_n[0] are 1
    uint8_t i = 0;
    for (i = 0; i < numSections; i++)
    {
        if (sosMatrix[i*CSOSFILTER_COEFFSPERSECTION + CSOSFILTER_SECTIONORDER + 1] != (real_t)1.0)
        {
            return CSOSFILTER_FAILURE;
        }
    }

    // Check coefficients for Inf/NaN
    uint8_t retCode = CSOSFILTER_SUCCESS;
    for (i = 0; i < numSections*CSOSFILTER_COEFFSPERSECTION; i++)
    {
        retCode += isValidReal(sosMatrix[i]);
    }
    for (i = 0; i < numGains; i++)
    {
        retCode += isValidReal(sosGain[i]);
    }

    // Check if worth proceeding
    if (retCode != CSOSFILTER_SUCCESS)
    {
        return retCode;
    }

    // OK by here - start with internal state initialization via reset
    retCode += csosFilterReset(filter);

    // Assign inputs
    memcpy(filter->sosMatrix, sosMatrix, 
        numSections*CSOSFILTER_COEFFSPERSECTION*sizeof(filter->sosMatrix[0][0]));
    memcpy(filter->sosGain, sosGain, numSections*sizeof(filter->sosGain[0]));
    filter->numSections = (int8_t)numSections;
    if (numGains > numSections)
    {
        filter->overallGain = sosGain[numGains-1];
    }
    else
    {
        filter->overallGain = (real_t)1.0;
    }

    return retCode;
}

//
// Filter Reset
//
int8_t csosFilterReset(csosFilterData_t* filter)
{
    // Input pointer check
    if (filter == NULL)
    {
        return CSOSFILTER_FAILURE;
    }

    // Reset shift registers
    uint8_t i = 0;
    for (i = 0; i < CSOSFILTER_MAXSECTIONS*CSOSFILTER_STATESPERSECTION; i++)
    {
        filter->uShift[i] = (real_t)0.0;
        filter->yShift[i] = (real_t)0.0;
    }

    return CSOSFILTER_SUCCESS;
}


//
// Valid real number check
// Checks not NaN, Inf
//
int8_t isValidReal(real_t num)
{
    if (isinf(num))
    {
        return CSOSFILTER_FAILURE;
    }
    if (isnan(num))
    {
        return CSOSFILTER_FAILURE;
    }

    return CSOSFILTER_SUCCESS;
}