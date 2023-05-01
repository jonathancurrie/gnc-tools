/**
 * @file cfilter.c
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 30-Apr-2023
 * @brief Discrete Filter Implementation.
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <math.h>
#include <assert.h>
#include "cfilter.h"

//
// Local Function Declarations
//
int8_t isValidReal(real_t num);


//
// Filter Update
//
int8_t cfilterUpdate(cfilterData_t* filter, real_t u, real_t* y)
{
    // Input pointer checks
    if (filter == NULL)
    {
        return CFILTER_FAILURE;
    }
    if (y == NULL)
    {
        return CFILTER_FAILURE;
    }

    // Input validity checks
    if (isValidReal(u) != CFILTER_SUCCESS)
    {
        return CFILTER_FAILURE;
    }

    // Difference equation evaluation (assumes a[0] = 1, i.e. normalized)
    // Note these operations could be done with less code (and more efficiently), 
    // but it is written explicitly here for understanding.
    int8_t i = 0;    
    // num = b[0]*u[k] + b[1]*u[k-1] + b[2]*u[k-2] + ...
    // with uShift = (u[k-1], u[k-2], ...)
    real_t yNum = filter->num[0] * u;    
    for (i = 1; i < filter->lenNum; i++)
    {
        yNum += filter->num[i] * filter->uShift[i-1];
    }
    // den = a[1]*y[k-1] + a[2]*y[k-2] + ...
    // with yShift = (y[k-1], y[k-2], ...)
    real_t yDen = (real_t)0.0;
    for (i = 1; i < filter->lenDen; i++)
    {
        yDen += filter->den[i] * filter->yShift[i-1];
    }
    // We keep separate numerator and denominator sums for numerical accuracy
    // until this final difference.
    // a[0]*y = num - den
    *y = yNum - yDen;

    // Update shift registers
    // uShift becomes (u[k], u[k-1], ...) 
    for (i = filter->lenNum-2; i > 0; i--)
    {
        filter->uShift[i] = filter->uShift[i-1];
    }
    // yShift becomes (y[k], y[k-1], ...)
    for (i = filter->lenDen-2; i > 0; i--)
    {
        filter->yShift[i] = filter->yShift[i-1];
    }
    // Insert current elements
    filter->uShift[0] = u;
    filter->yShift[0] = *y;

    return CFILTER_SUCCESS;
}


//
// Filter Data Structure Initialization
//
int8_t cfilterInit(cfilterData_t* filter, const real_t* num, uint8_t lenNum, 
                   const real_t* den, uint8_t lenDen)
{
    // Assert we can sum the failure code (compile time check)
    static_assert(CFILTER_FAILURE != 0, "Require CFILTER_FAILURE to be != 0");

    // Input pointer checks
    if (filter == NULL)
    {
        return CFILTER_FAILURE;
    }
    if (num == NULL)
    {
        return CFILTER_FAILURE;
    }
    if (den == NULL)
    {
        return CFILTER_FAILURE;
    }

    // Check lengths vs order
    if (lenNum > (CFILTER_MAXORDER+1))
    {
        return CFILTER_FAILURE;
    }
    if (lenDen > (CFILTER_MAXORDER+1))
    {
        return CFILTER_FAILURE;
    }

    // Check we have coefficients in both arrays
    if ((lenNum == 0) || (lenDen == 0))
    {
        return CFILTER_FAILURE;
    }

    // Ensure a[0] == 1
    if (den[0] != (real_t)1.0)
    {
        return CFILTER_FAILURE;
    }

    // Check coefficients for Inf/NaN
    uint8_t retCode = CFILTER_SUCCESS;
    uint8_t i = 0;
    for (i = 0; i < lenNum; i++)
    {
        retCode += isValidReal(num[i]);
    }
    for (i = 0; i < lenDen; i++)
    {
        retCode += isValidReal(den[i]);
    }

    // Check if worth proceeding
    if (retCode != CFILTER_SUCCESS)
    {
        return retCode;
    }

    // OK by here - start with internal state initialization via reset
    retCode += cfilterReset(filter);

    // Assign inputs
    memcpy(filter->num, num, lenNum*sizeof(filter->num[0]));
    memcpy(filter->den, den, lenDen*sizeof(filter->den[0]));
    filter->lenNum = (int8_t)lenNum;
    filter->lenDen = (int8_t)lenDen;

    return retCode;
}


//
// Filter Reset
//
int8_t cfilterReset(cfilterData_t* filter)
{
    // Input pointer check
    if (filter == NULL)
    {
        return CFILTER_FAILURE;
    }

    // Reset shift registers
    uint8_t i = 0;
    for (i = 0; i < CFILTER_MAXORDER; i++)
    {
        filter->uShift[i] = (real_t)0.0;
        filter->yShift[i] = (real_t)0.0;
    }

    return CFILTER_SUCCESS;
}

//
// Valid real number check
// Checks not NaN, Inf
//
int8_t isValidReal(real_t num)
{
    if (isinf(num))
    {
        return CFILTER_FAILURE;
    }
    if (isnan(num))
    {
        return CFILTER_FAILURE;
    }

    return CFILTER_SUCCESS;
}