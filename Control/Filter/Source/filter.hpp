/**
 * @file filter.hpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 01-May-2023
 * @brief A wrapper around the C Filter Digital Filter (cfilter.h)
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#ifndef FILTER_HPP
#define FILTER_HPP

#include <vector>
#include "cfilter.h"

namespace GNCTools
{
/**
 * @brief A container for storing and constructing filter parameter sets
 * 
 */
class FilterParams
{
    public:
        /**
         * @brief   Default constructor
         * 
         */
        FilterParams(void) = default;

        /**
         * @brief   Construct a new Filter Params object
         * 
         * @param numIn     Numerator Coefficients
         * @param denIn     Denominator Coefficients
         */
        FilterParams(const std::vector<double>& numIn, 
                        const std::vector<double>& denIn) :
                        num(numIn), den(denIn) { }

        /**
         * @brief   Convert the internal parameters to a C Filter data structure
         * 
         * @param filterData    Pointer to a C Filter data structure
         * @return int          Return code (CFILTER_SUCCESS on success, -ve failure)
         */
        int makeCData(cfilterData_t* filterData) const;

        std::vector<double> num{};  //!< Numerator Coefficients
        std::vector<double> den{};  //!< Denominator Coefficients
};

/**
 * @brief A C++ wrapper around the C Filter
 * 
 */
class Filter
{
    public:
        /**
         * @brief Default constructor
         * 
         */
        Filter(void) = default;

        /**
         * @brief Construct a new Filter object from supplied parameters
         * 
         * @param params FilterParams object containing the filter coefficients
         */
        explicit Filter(const FilterParams& params);

        /**
         * @brief Construct a new Filter object from specified numerator and
         *        denomiator coefficients
         * 
         * @param numIn     Numerator coefficients
         * @param denIn     Denominator coefficients
         */
        Filter(const std::vector<double>& numIn, 
                const std::vector<double>& denIn);

        /**
         * @brief  Filter Update
         *  
         * @param u     Filter input
         * @param y     Filter output
         * @return int  Return code (CFILTER_SUCCESS on success, -ve failure)
         */
        int update(double u, double& y);

        /**
         * @brief   Filter Reset
         * 
         * @return int  Return code (CFILTER_SUCCESS on success, -ve failure)
         */
        int reset(void);

        /**
         * @brief   Return true if this filter is initialized correctly
         * 
         * @return bool True if initialized, false if not 
         */
        bool isInitialized(void) const { return _initialized; }
        
        /**
         * @brief   Return a constant reference to the internal C Filter Data
         * 
         * @return const cfilterData_t& The C Filter data structure
         */
        const cfilterData_t& getCFilterData(void) const { return _filterData; }

        /**
         * @brief   Static helper to check parameter validity
         * 
         * @param params    A FilterParams object containing the filter coefficients
         * @return int      Return code (CFILTER_SUCCESS on success, -ve failure)
         */
        static int checkParams(const FilterParams& params);

    private:
        /**
         * @brief   Initializes a Filter object from supplied parameter object
         * 
         * @param params    A FilterParams object containing the filter coefficients
         * @return int      Return code (CFILTER_SUCCESS on success, -ve failure)
         */
        int _initFromParams(const FilterParams& params);

        bool _initialized = false;      //!< Indicates whether the object is initialized
        cfilterData_t _filterData{};    //!< Internal C Filter data structure
};
}


#endif /* FILTER_HPP */
