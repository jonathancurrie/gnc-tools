/**
 * @file sosFilter.hpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 01-May-2023
 * @brief A wrapper around the C SOS Filter Digital Filter (csosFilter.h)
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#ifndef SOSFILTER_HPP
#define SOSFILTER_HPP

#include <vector>
#include "csosFilter.h"

namespace GNCTools
{
/**
 * @brief A container for storing and constructing SOS filter parameter sets
 * 
 */
class SOSFilterParams
{
    public:
        /**
         * @brief Default constructor
         * 
         */
        SOSFilterParams(void) = default;

        /**
         * @brief Construct a new SOSFilterParams object
         * 
         * @param sosMatrix     SOS Matrix Coefficients [b11,b12,b13,a11,a12,a23;b21...]
         * @param sosGain       SOS Gain [g1,g2,gOverall] (gOverall optional)
         */
        SOSFilterParams(const std::vector<std::vector<double>>& sosMatrixIn,
                        const std::vector<double>& sosGainIn) :
                        sosMatrix(sosMatrixIn), sosGain(sosGainIn) { }

        /**
         * @brief   Convert the internal parameters to a C SOS Filter data structure
         * 
         * @param filterData    Pointer to a C SOS Filter data structure
         * @return int          Return code (CSOSFILTER_SUCCESS on success, -ve failure)
         */
        int makeCData(csosFilterData_t* filterData) const;

        std::vector<std::vector<double>> sosMatrix{};   //!< SOS Coefficient Matrix
        std::vector<double> sosGain{};                  //!< SOS Gain Vector
};

/**
 * @brief A C++ wrapper around the C SOS Filter
 * 
 */
class SOSFilter
{
    public:
        /**
         * @brief Default constructor
         * 
         */
        SOSFilter(void) = default;

        /**
         * @brief Construct a new SOSFilter object from supplied parameters
         * 
         * @param params SOSFilterParams object containing the filter matrix and gain
         */
        explicit SOSFilter(const SOSFilterParams& params);

        /**
         * @brief Construct a new SOSFilter object from specified matrix and gain
         * 
         * @param sosMatrix     SOS Matrix Coefficients [b11,b12,b13,a11,a12,a23;b21...]
         * @param sosGain       SOS Gain [g1,g2,gOverall]
         */
        SOSFilter(const std::vector<std::vector<double>>& sosMatrix,
                  const std::vector<double>& sosGain);

        /**
         * @brief  SOS Filter Update
         *  
         * @param u     Filter input
         * @param y     Filter output
         * @return int  Return code (CSOSFILTER_SUCCESS on success, -ve failure)
         */
        int update(double u, double& y);

        /**
         * @brief   SOS Filter Reset
         * 
         * @return int  Return code (CSOSFILTER_SUCCESS on success, -ve failure)
         */
        int reset(void);

        /**
         * @brief   Return true if this filter is initialized correctly
         * 
         * @return bool True if initialized, false if not 
         */
        bool isInitialized(void) const { return _initialized; }

        /**
         * @brief   Return a constant reference to the internal C SOS Filter Data
         * 
         * @return const csosFilterData_t& The C SOS Filter data structure
         */
        const csosFilterData_t& getCSOSFilterData(void) const { return _filterData; }

        /**
         * @brief   Static helper to check parameter validity
         * 
         * @param params    A SOSFilterParams object containing the filter coefficients
         * @return int      Return code (CSOSFILTER_SUCCESS on success, -ve failure)
         */
        static int checkParams(const SOSFilterParams& params);

    private:
        /**
         * @brief   Initializes a SOSFilter object from supplied parameter object
         * 
         * @param params    A SOSFilterParams object containing the filter matrix and gain
         * @return int      Return code (CSOSFILTER_SUCCESS on success, -ve failure)
         */
        int _initFromParams(const SOSFilterParams& params);

        bool _initialized = false;         //!< Indicates whether the object is initialized
        csosFilterData_t _filterData{};    //!< Internal C SOS Filter data structure
};
}

#endif /* SOSFILTER_HPP */
