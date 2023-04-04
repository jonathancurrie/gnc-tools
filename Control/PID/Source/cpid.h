/**
 * @file cpid.h
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 03-Apr-2023
 * @brief Discrete 2 Degree of Freedom (2DOF) Proportional-Integral-Derivative (PID) 
 *        Controller with Derivative Filter and Anti-Wind Up.
 * 
 * Control Law:
 * u = Kp * (b*r - y) + Ki * IF(z) * (r - y) + Kd / (Tf + DF(z)) * (c*r - y)
 * 
 * where y is the current plant output (process variable), r is the current setpoint, 
 * u is the controller output (plant input) and the remainder of the parameters are 
 * detailed below.
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CPID_H
#define CPID_H

#include <stdint.h>     // For int8_t, etc

// Type Definitions for Target System (user to modify as required)
typedef double real_t;  //!< Floating point real number

// Constant Definitions
#define CPID_SUCCESS 0  //!< Success return code
#define CPID_FAILURE -1 //!< Failure return code (may be summed)

/**
 * @brief Contains PID tuning and internal calculation variables
 * 
 */
typedef struct  
{
	real_t Kp;			//!< Proportional Gain 
	real_t Ki;			//!< Integral Gain 
	real_t Kd;			//!< Derivative Gain 
	real_t Tf;			//!< Derivative filter time constant
	real_t Ts;			//!< Sampling interval (nominally seconds) 
	real_t uMin;		//!< Minimum control move
    real_t uMax;		//!< Maximum control move
    real_t b;           //!< Setpoint weight on proportional term
    real_t c;           //!< Setpoint weight on derivative term
	real_t invTs;       //!< Internal precalculated 1.0/Ts
	real_t xI;		    //!< Internal integral state
	real_t xD;		    //!< Internal derivative filter state		
	uint8_t useI;		//!< Internal integral control flag
	uint8_t useD;		//!< Internal derivative control flag
	uint8_t useDFilt;	//!< Internal derivative filter control flag
} cpidData_t;


/**
 * @brief               Initialize a PID controller data structure
 * 
 * @param pid           A pointer to a C PID data structure to initialize
 * @param Kp            Proportional gain 
 * @param Ki            Integral gain (0 to disable)
 * @param Kd            Derivative gain (0 to disable)
 * @param Tf            Derivative filter time constant (0 to disable, larger adds more filtering)
 * @param Ts            Sampling interval (nominally seconds)
 * @param uMin          Minimum control move (controller output, set to -Inf if not required)
 * @param uMax          Maximum control move (controller output, set to +Inf if not required)
 * @param b             Setpoint weight on proportional term (error = b*r - y), default 1, >= 0
 * @param c             Setpoint weight on derivative term (error = c*r - y), default 1, >= 0
 * @return int8_t       Return code (CPID_SUCCESS on success, -ve failure)
 */
int8_t cpidInit(cpidData_t* pid, real_t Kp, real_t Ki, real_t Kd, real_t Tf, real_t Ts, real_t uMin, real_t uMax, real_t b, real_t c);


/**
 * @brief               PID controller update
 *                      Must be called at a rate of 1/ts (sampling frequency)
 * 
 * @param pid           A pointer to an initialized C PID data structure
 * @param r             The current setpoint
 * @param y             The current plant output
 * @param u             A pointer to the computed control move (controller output)
 * @return int8_t       Return code (CPID_SUCCESS on success, -ve failure)
 */
int8_t cpidUpdate(cpidData_t* pid, real_t r, real_t y, real_t* u);


/**
 * @brief               PID controller reset
 * 
 * @param pid           A pointer to an initialized C PID data structure
 * @return int8_t       Return code (CPID_SUCCESS on success, -ve failure)
 */
int8_t cpidReset(cpidData_t* pid);

#endif /* CPID_H */

#ifdef __cplusplus
}
#endif
