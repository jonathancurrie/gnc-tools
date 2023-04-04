/**
 * @file cpid.c
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 03-Apr-2023
 * @brief Discrete 2 Degree of Freedom (2DOF) Proportional-Integral-Derivative (PID) 
 *        Controller with Derivative Filter and Anti-Wind Up.
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <math.h>
#include <assert.h>
#include "cpid.h"

//
// Local Function Declarations
//
real_t forwardEuler(real_t y, real_t yprev, real_t ts);
int8_t isValidReal(real_t num);


//
// PID Controller Update
//
int8_t cpidUpdate(cpidData_t* pid, real_t r, real_t y, real_t* u)
{
    // Input pointer checks
    if (pid == NULL)
    {
        return CPID_FAILURE;
    }
    if (u == NULL)
    {
        return CPID_FAILURE;
    }

    // Input validity checks
    if (isValidReal(r) != CPID_SUCCESS)
    {
        return CPID_FAILURE;
    }
    if (isValidReal(y) != CPID_SUCCESS)
    {
        return CPID_FAILURE;
    }

    // Compute integral term
    real_t iError = (real_t)0.0;
    if (pid->useI)
    {
        iError = pid->Ki * (r - y);
    }

    // Compute derivative term
    real_t dError = (real_t)0.0;
    if (pid->useD)
    {
        // Save the previous derivative error state
        real_t lastDError = pid->xD;

        if (pid->useDFilt)
        {
            // Derivative term
            dError = pid->Kd * (pid->c * r - y);
            // Filter contribution
            dError = (dError - lastDError) / pid->Tf;
        }
        else
        {
            // Derivative term
            dError = pid->Kd * (pid->c * r - y) * pid->invTs;
            // Save as the most recent derivative * error
            pid->xD = dError;
            // Subtract the previous step
            dError -= lastDError;
        }
    }

    // Compute proportional term
    real_t kError = pid->Kp * (pid->b * r - y);

    // Compute controller output based on control law
    *u = kError + pid->xI + dError;

    // Saturate as required
    real_t backClamp = (real_t)0.0;
    if (*u > pid->uMax)
    {        
        backClamp = pid->uMax - *u;        
        *u = pid->uMax;
    }
    else if (*u < pid->uMin)
    {        
        backClamp = pid->uMin - *u;
        *u = pid->uMin;
    }

    // Update filters/integrators
    if (pid->useDFilt)
    {
        pid->xD = forwardEuler(dError, pid->xD, pid->Ts);
    }
    if (pid->useI)
    {
        pid->xI = forwardEuler(iError+backClamp, pid->xI, pid->Ts);
    }

    return CPID_SUCCESS;
}

//
// Forward Euler Integration Step
//
real_t forwardEuler(real_t y, real_t yprev, real_t ts)
{
    return yprev + y * ts;
}


//
// PID Controller Data Structure Initialization
//
int8_t cpidInit(cpidData_t* pid, real_t Kp, real_t Ki, real_t Kd, real_t Tf, real_t Ts, real_t uMin, real_t uMax, real_t b, real_t c)
{
    // Assert we can sum the failure code (compile time check)
    static_assert(CPID_FAILURE != 0, "Require CPID_FAILURE to be != 0");

    // Input pointer checks
    if (pid == NULL)
    {
        return CPID_FAILURE;
    }

    // Check for Inf/NaN
    uint8_t retCode = CPID_SUCCESS;
    retCode += isValidReal(Kp);
    retCode += isValidReal(Ki);
    retCode += isValidReal(Kd);
    retCode += isValidReal(Tf);
    retCode += isValidReal(Ts);
    retCode += isValidReal(b);
    retCode += isValidReal(c);
    // umin/umax may be Inf, check for NaN
    if (isnan(uMin))
    {
        retCode += CPID_FAILURE;
    }
    if (isnan(uMax))
    {
        retCode += CPID_FAILURE;
    }

    // Check valid numerical values for a PID controller
    if (Kp == (real_t)0.0) // need a proportional term
    {
        retCode += CPID_FAILURE;
    }
    if (Ts <= (real_t)0.0)
    {
        retCode += CPID_FAILURE;
    }
    if (Tf < (real_t)0.0)
    {
        retCode += CPID_FAILURE;
    }
    if (Tf > (real_t)0.0)
    {
        if (Kd == (real_t)0.0) // no D gain, but D filter?
        {
            retCode += CPID_FAILURE;
        }
        if (Tf <= ((real_t)0.5 * Ts)) // ensures stability
        {
            retCode += CPID_FAILURE;
        }
    }
    if (c < (real_t)0.0)
    {
        retCode += CPID_FAILURE;
    }
    if (b < (real_t)0.0)
    {
        retCode += CPID_FAILURE;
    }
    // Check saturation bounds are sensible
    if (uMin > uMax)
    {
        retCode += CPID_FAILURE;
    }
    if (uMin == uMax)
    {
        retCode += CPID_FAILURE;
    }

    // Check if worth proceeding
    if (retCode != CPID_SUCCESS)
    {
        return retCode;
    }

    // OK by here - start with integrated state initialization via reset
    retCode += cpidReset(pid);

    // Assign inputs
    pid->Kp         = Kp;
    pid->Ki         = Ki;
    pid->Kd         = Kd;
    pid->Tf         = Tf;
    pid->Ts         = Ts;
    pid->uMin       = uMin;
    pid->uMax       = uMax;
    pid->b          = b;
    pid->c          = c;

    // Compute derived internal parameters
    pid->invTs      = (real_t)1.0 / pid->Ts;
    pid->useI       = (pid->Ki != (real_t)0.0);
    pid->useD       = (pid->Kd != (real_t)0.0);
    pid->useDFilt   = (pid->Tf > (real_t)0.0);

    return retCode;
}


//
// PID Controller Reset
//
int8_t cpidReset(cpidData_t* pid)
{
    // Input pointer check
    if (pid == NULL)
    {
        return CPID_FAILURE;
    }

    // Reset integrated terms
    pid->xD = (real_t)0.0;
    pid->xI = (real_t)0.0;

    return CPID_SUCCESS;
}

//
// Valid real number check
// Checks not NaN, Inf
//
int8_t isValidReal(real_t num)
{
    if (isinf(num))
    {
        return CPID_FAILURE;
    }
    if (isnan(num))
    {
        return CPID_FAILURE;
    }

    return CPID_SUCCESS;
}