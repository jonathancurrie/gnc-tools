/**
 * @file csosFilter.h
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 01-May-2023
 * @brief Discrete Second Order Section Filter Implementation.
 * 
 * Difference Equation per Section:
 * y[k] = b[0]*u[k] + b[1]*u[k-1] + b[2]*u[k-2] + ... - a[1]*y[k-1] - a[2]*y[k-2] - ...
 * 
 * where y is the section output, u is the section input. Each section is
 * cascaded to form an overall filter in 'Direct-Form II, Second-Order Sections'
 * format. 
 * 
 * The sosMatrix is expected in the following row-major format:
 * 
 * [b11 b12 b13 a11 a12 a13; b21 b22 b23 a21 a22 a23; ...]
 * 
 * where b are the second order filter numerator coefficients, and 
 * a are the denominator coefficients. It is assumed the coefficients are 
 * normalized such that an1[0] = 1. Each row represents one second order
 * section of the filter, cascaded together. 
 * 
 * The sosGain vector is expected in the following format:
 * 
 * [g1, g2, ..., gOverall]
 * 
 * where g1 is the gain of section 1, g2 is the gain of section 2, and so forth.
 * Optionally, an additional overall filter gain, gOverall, may be specified, and
 * defaults to 1 if not supplied.
 * 
 * In order to avoid dynamic memory allocation, the maximum number of
 * sections of the filter is statically allocated via CSOSFILTER_MAXSECTIONS. 
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CSOSFILTER_H
#define CSOSFILTER_H

#include <stdint.h>     // For int8_t, etc

// Type Definitions for Target System (user to modify as required)
typedef double real_t;  //!< Floating point real number

// Constant Definitions
#define CSOSFILTER_SUCCESS  0           //!< Success return code
#define CSOSFILTER_FAILURE -1           //!< Failure return code (may be summed)
#define CSOSFILTER_MAXSECTIONS 6        //!< Maximum number of sections (statically allocated memory)
#define CSOSFILTER_SECTIONORDER 2       //!< Order per section
#define CSOSFILTER_COEFFSPERSECTION (2*(CSOSFILTER_SECTIONORDER+1)) //!< Order+1 per num/den, 2x for both
#define CSOSFILTER_STATESPERSECTION CSOSFILTER_SECTIONORDER //!< Order = num states

/**
 * @brief Contains C SOS Filter Coefficients and internal state
 * 
 */
typedef struct 
{
    real_t sosMatrix[CSOSFILTER_MAXSECTIONS][CSOSFILTER_COEFFSPERSECTION];  //!< SOS Matrix [b1,a1;b2,a2;...]    
    real_t sosGain[CSOSFILTER_MAXSECTIONS];                                 //!< SOS Gains [g1,g2]    
    real_t overallGain;                                                     //!< Overall filter gain
    int8_t numSections;                                                     //!< Number of SOS Sections
    real_t uShift[CSOSFILTER_MAXSECTIONS*CSOSFILTER_STATESPERSECTION];      //!< Input shift register
    real_t yShift[CSOSFILTER_MAXSECTIONS*CSOSFILTER_STATESPERSECTION];      //!< Output shift register
} csosFilterData_t;

/**
 * @brief               C SOS Filter Initialization
 * 
 * @param filter        A pointer to a C SOS Filter data structure to initialize
 * @param sosMatrix     SOS Filter Coefficients [b1,a1;b2,a2;...], all second order
 * @param numSections   Number of SOS Sections
 * @param sosGain       SOS Gains [g1,g2,gOverall] (note final gain optional, assumed 1)
 * @param numGains      Number of Section Gains
 * @return int8_t       Return code (CSOSFILTER_SUCCESS on success, -ve failure)
 */
int8_t csosFilterInit(csosFilterData_t* filter, const real_t* sosMatrix, 
                      uint8_t numSections, const real_t* sosGain, uint8_t numGains);

/**
 * @brief           C SOS Filter Discrete Update
 * 
 * @param filter    A pointer to an initialized C SOS Filter data structure
 * @param u         Input to filter
 * @param y         Filter output
 * @return int8_t   Return code (CSOSFILTER_SUCCESS on success, -ve failure)
 */
int8_t csosFilterUpdate(csosFilterData_t* filter, real_t u, real_t* y);

/**
 * @brief           C SOS Filter Reset
 *  
 * @param filter    A pointer to an initialized C SOS Filter data structure
 * @return int8_t   Return code (CSOSFILTER_SUCCESS on success, -ve failure)
 */
int8_t csosFilterReset(csosFilterData_t* filter);

#endif /* CSOSFILTER_H */

#ifdef __cplusplus
}
#endif