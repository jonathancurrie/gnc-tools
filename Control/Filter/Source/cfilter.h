/**
 * @file cfilter.h
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 30-Apr-2023
 * @brief Discrete Filter Implementation.
 * 
 * Difference Equation:
 * y[k] = b[0]*u[k] + b[1]*u[k-1] + b[2]*u[k-2] + ... - a[1]*y[k-1] - a[2]*y[k-2] - ...
 * 
 * where y is the filter output, u is the filter input, b are the
 * filter numerator coefficients, and a are the filter denominator
 * coefficients. It is assumed the coefficients are normalized such
 * that a[0] = 1. Both FIR and IIR filters are compatible with this
 * format.
 * 
 * In order to avoid dynamic memory allocation, the maximum order of
 * the filter is statically allocated via CFILTER_MAXORDER. 
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFILTER_H
#define CFILTER_H

#include <stdint.h>     // For int8_t, etc

// Type Definitions for Target System (user to modify as required)
typedef double real_t;  //!< Floating point real number

// Constant Definitions
#define CFILTER_SUCCESS 0   //!< Success return code
#define CFILTER_FAILURE -1  //!< Failure return code (may be summed)
#define CFILTER_MAXORDER 12  //!< Maximum order of filter (statically allocated memory)

/**
 * @brief Contains C Filter coefficients and internal state
 * 
 */
typedef struct 
{
    real_t num[CFILTER_MAXORDER+1];     //!< Numerator coefficients
    real_t den[CFILTER_MAXORDER+1];     //!< Denominator coefficients
    int8_t lenNum;                      //!< Numerator length
    int8_t lenDen;                      //!< Denominator length
    real_t uShift[CFILTER_MAXORDER];    //!< Input shift register
    real_t yShift[CFILTER_MAXORDER];    //!< Output shift register
} cfilterData_t;


/**
 * @brief               C Filter initialization
 * 
 * @param filter        A pointer to a C Filter data structure to initialize
 * @param num           Numerator coefficients (discrete, z^-1)
 * @param lenNum        Number of numerator coefficients
 * @param den           Denominator coefficients (discrete, z^-1)
 * @param lenDen        Number of denominator coefficients
 * @return int8_t       Return code (CFILTER_SUCCESS on success, -ve failure)
 */
int8_t cfilterInit(cfilterData_t* filter, const real_t* num, uint8_t lenNum, 
                   const real_t* den, uint8_t lenDen);


/**
 * @brief           C Filter discrete update
 *                  Must be called at a rate of 1/ts (sampling frequency)
 * 
 * @param filter    A pointer to an initialized C Filter data structure
 * @param u         Input to filter
 * @param y         Filter output
 * @return int8_t   Return code (CFILTER_SUCCESS on success, -ve failure)
 */
int8_t cfilterUpdate(cfilterData_t* filter, real_t u, real_t* y);


/**
 * @brief           C Filter reset
 * 
 * @param filter    A pointer to an initialized C Filter data structure
 * @return int8_t   Return code (CFILTER_SUCCESS on success, -ve failure)
 */
int8_t cfilterReset(cfilterData_t* filter);


#endif /* CFILTER_H */

#ifdef __cplusplus
}
#endif

