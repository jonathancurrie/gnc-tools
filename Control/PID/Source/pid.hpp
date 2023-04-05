/**
 * @file pid.hpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 04-Apr-2023
 * @brief A wrapper around the CPID Discrete 2 DOF PID Controller (pid.h)
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#ifndef PID_HPP
#define PID_HPP

#include <limits>
#include "cpid.h"

namespace GNCTools
{
    /**
     * @brief A container for storing and constructing PID tuning parameter sets
     * 
     */
    class PIDParams
    {
        public:
            /**
             * @brief       Default constructor
             * 
             */
            PIDParams(void) = default;

            /**
             * @brief       Construct a new PIDParams object with the minimum tuning parameters specified
             * 
             * @param KpIn  Proportional Gain
             * @param KiIn  Integral Gain
             * @param KdIn  Derivative Gain
             * @param TsIn  Sampling interval
             */
            PIDParams(double KpIn, double KiIn, double KdIn, double TsIn) : 
                        Kp(KpIn), Ki(KiIn), Kd(KdIn), Ts(TsIn) { }

            /**
             * @brief       Construct a new PIDParams object with derivative filter specified
             * 
             * @param KpIn  Proportional Gain
             * @param KiIn  Integral Gain
             * @param KdIn  Derivative Gain
             * @param TfIn  Derivative filter time constant
             * @param TsIn  Sampling interval
             */
            PIDParams(double KpIn, double KiIn, double KdIn, double TfIn, double TsIn) : 
                        Kp(KpIn), Ki(KiIn), Kd(KdIn), Tf(TfIn), Ts(TsIn) { }

            
            /**
             * @brief           Convert the internal parameters to a CPID data structure, including parameter checks
             * 
             * @param pidData   Pointer to a C PID data structure
             * @return int      Return code (CPID_SUCCESS on success, -ve failure)
             */
            int makeCData(cpidData_t* pidData) const;

            double Kp = 0.0;    //!< Proportional Gain
            double Ki = 0.0;    //!< Integral Gain (0 to disable)
            double Kd = 0.0;    //!< Derivative gain (0 to disable)
            double Tf = 0.0;    //!< Derivative filter time constant (0 to disable, larger adds more filtering)
            double Ts = 0.0;    //!< Sampling interval (nominally seconds)
            double b  = 1.0;    //!< Setpoint weight on proportional term (error = b*r - y), default 1, >= 0
            double c  = 1.0;    //!< Setpoint weight on derivative term (error = c*r - y), default 1, >= 0
            double uMin = -std::numeric_limits<double>::infinity();     //!< Minimum control move (controller output, set to -Inf if not required)
            double uMax = +std::numeric_limits<double>::infinity();     //!< Maximum control move (controller output, set to +Inf if not required)
            double rRampMax = +std::numeric_limits<double>::infinity(); //!< Maximum setpoint rate of change per sample (set to +Inf if not required)
    };

    /**
     * @brief A C++ wrapper around the C PID controller
     * 
     */
    class PID
    {
        public:
            /**
             * @brief           Default constructor
             * 
             */
            PID(void) = default;

            /**
             * @brief           Construct a new PID object with supplied parameters
             * 
             * @param params    PIDParams object containing the tuning of the controller
             */
            explicit PID(const PIDParams& params);

            /**
             * @brief           Construct a new PID object with the minimum set of configuration values required
             * 
             * @param Kp        Proportional gain
             * @param Ki        Integral gain
             * @param Kd        Derivative gain
             * @param Ts        Sampling interval (nominally seconds)  
             */
            PID(double Kp, double Ki, double Kd, double Ts);

            /**
             * @brief           Control law update
             * 
             * @param r         The current setpoint
             * @param y         The current plant output ()
             * @param u         A pointer to the computed control move (controller output)
             * @return int      Return code (CPID_SUCCESS on success, -ve failure)
             */
            int update(double r, double y, double& u);

            /**
             * @brief           PID controller reset
             * 
             * @return int      Return code (CPID_SUCCESS on success, -ve failure)
             */
            int reset(void);

            /**
             * @brief           Return if this controller is initialzed correctly
             * 
             * @return bool     True if initialized, false if not 
             */
            bool isInitialized(void) const { return _initialized; }

            /**
             * @brief           Return a constant reference to the internal C PID Data Structure
             * 
             * @return const cpidData_t& The C PID data structure
             */
            const cpidData_t& getCPIDData(void) const { return _pidData; }

            /**
             * @brief           Static helper to check parameter validity
             * 
             * @param params    A PIDParams object containing the tuning information
             * @return int      Return code (CPID_SUCCESS on success, -ve failure)
             */
            static int checkParams(const PIDParams& params);

            
        private:
            /**
             * @brief           Initializes a PID object from supplied parameter object
             *      
             * @param params    A PIDParams object containing the tuning information
             * @return int      Return code (CPID_SUCCESS on success, -ve failure)
             */
            int _initFromParams(const PIDParams& params);

            bool _initialized = false;   //!< Indicates whether the object is initialized
            cpidData_t _pidData{};       //!< Internal C PID data structure
    };
}

#endif /* PID_HPP */
