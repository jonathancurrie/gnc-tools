/**
 * @file cpid.h
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 03-Apr-2023
 * @brief 2 Degree of Freedom (2DOF) Proportional-Integral-Derivative (PID) 
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

#ifndef CPID_H
#define CPID_H

#include <stdint.h>     // For int8_t, etc

/**
 * @brief Type Definitions for Target System (user to modify as required)
 * 
 */
typedef double real_t;  // Floating point real number


/**
 * @brief Contains PID tuning and internal calculation variables
 * 
 */
typedef struct  
{
	real_t kp;			//!< Proportional Gain 
	real_t ki;			//!< Integral Gain 
	real_t kd;			//!< Derivative Gain 
	real_t tf;			//!< Derivative filter time constant
    real_t b;           //!< Setpoint weight on proportional term
    real_t c;           //!< Setpoint weight on derivative term
	real_t ts;			//!< Sample interval [s] 
	real_t umax;		//!< Maximum control move
	real_t umin;		//!< Minimum control move
	real_t invts;       //!< Internal precalculated 1.0/Ts
	real_t xI;		    //!< Internal integral state
	real_t xD;		    //!< Internal derivative filter state		
	uint8_t useI;		//!< Internal integral control flag
	uint8_t useD;		//!< Internal derivative control flag
	uint8_t useDFilt;	//!< Internal derivative filter control flag
} cpidData;


/**
 * @brief               Initialize a PID controller data structure
 * 
 * @param pid           A pointer to a C PID data structure to initialize
 * @param kp            Proportional gain 
 * @param ki            Integral gain (0 to disable)
 * @param kd            Derivative gain (0 to disable)
 * @param tf            Derivative filter time constant (0 to disable, larger adds more filtering)
 * @param ts            Sampling interval (nominally seconds)
 * @param umin          Minimum control move (controller output)
 * @param umax          Maximum control move (controller output)
 * @param b             Setpoint weight on proportional term (error = b*r - y), default 1
 * @param c             Setpoint weight on derivative term (error = c*r - y), default 1
 * @return uint8_t      Return code (0 success, -ve failure)
 */
uint8_t cpidInit(cpidData* pid, real_t kp, real_t ki, real_t kd, real_t tf, real_t ts, real_t umin, real_t umax, real_t b=1.0, real_t c=1.0);


/**
 * @brief               PID controller update
 * 
 * @param pid           A pointer to an initialized C PID data structure
 * @param r             The current setpoint
 * @param y             The current plant output
 * @param u             A pointer to the computed control move (controller output)
 * @return uint8_t      Return code (0 success, -ve failure)
 */
uint8_t cpidUpdate(cpidData* pid, real_t r, real_t y, real_t* u);

#endif /* CPID_H */
