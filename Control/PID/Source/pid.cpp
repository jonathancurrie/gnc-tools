/**
 * @file pid.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 04-Apr-2023
 * @brief A wrapper around the CPID Discrete 2 DOF PID Controller (pid.c)
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include "pid.hpp"

namespace GNCTools
{
//
// PID Constructors
//
PID::PID(const PIDParams& params)
{
    _initFromParams(params); // checks parameters internally
}

PID::PID(double Kp, double Ki, double Kd, double Ts) : 
    PID::PID(PIDParams(Kp, Ki, Kd, Ts)) {}

//
// PID Update
//
int PID::update(double r, double y, double& u)
{
    if (isInitialized())
    {
        return cpidUpdate(&_pidData, r, y, &u);
    }
    return CPID_FAILURE;
}

//
// PID Reset
//
int PID::reset(void)
{
    if (isInitialized())
    {
        return cpidReset(&_pidData);
    }
    return CPID_FAILURE;
}

//
// Initialization
//
int PID::_initFromParams(const PIDParams& params)
{
    // Use the C PID conversion to initialize this object's C PID data structure
    int status = params.makeCData(&_pidData);

    if (status == CPID_SUCCESS)
    {
        _initialized = true;
    }                                
    return status;
}

//
// Parameter Checking
//
int PID::checkParams(const PIDParams& params)
{
    // Use the C PID conversion to do the checking for us
    cpidData_t tempCPID{};
    return params.makeCData(&tempCPID);
}


//
// Convert PIDParams to C PID Data Structure
//
int PIDParams::makeCData(cpidData_t* pidData) const
{
    // Use init function to convert (and check) to C struct
    // Null check internal
    return cpidInit(pidData, Kp, Ki, Kd, Tf, Ts, uMin, uMax, b, c, rRampMax);
}

}
