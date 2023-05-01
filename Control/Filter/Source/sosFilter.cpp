/**
 * @file sosFilter.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 01-May-2023
 * @brief A wrapper around the C SOS SOSFilter Digital SOSFilter (csosFilter.c)
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include "sosFilter.hpp"

namespace GNCTools
{
//
// SOSFilter Constructors
//
SOSFilter::SOSFilter(const SOSFilterParams& params)
{
    _initFromParams(params); // checks parameters internally
}

SOSFilter::SOSFilter(const std::vector<std::vector<double>>& sosMatrix,
                     const std::vector<double>& sosGain)
{
    _initFromParams(SOSFilterParams(sosMatrix, sosGain));
}

//
// SOSFilter Update
// 
int SOSFilter::update(double u, double& y)
{
    if (isInitialized())
    {
        return csosFilterUpdate(&_filterData, u, &y);
    }
    return CSOSFILTER_FAILURE;
}

//
// SOSFilter Reset
//
int SOSFilter::reset(void)
{
    if (isInitialized())
    {
        return csosFilterReset(&_filterData);
    }
    return CSOSFILTER_FAILURE;
}

//
// Initialization
//
int SOSFilter::_initFromParams(const SOSFilterParams& params)
{
    // Use the C SOSFilter conversion to initialize this object's C SOSFilter data structure
    int status = params.makeCData(&_filterData);

    if (status == CSOSFILTER_SUCCESS)
    {
        _initialized = true;
    }                                
    return status;
}

//
// Parameter Checking
//
int SOSFilter::checkParams(const SOSFilterParams& params)
{
    // Use the C SOSFilter conversion to do the checking for us
    csosFilterData_t tempCFilter{};
    return params.makeCData(&tempCFilter);
}

//
// Convert FilterParams to C SOSFilter Data Structure
//
int SOSFilterParams::makeCData(csosFilterData_t* filterData) const
{
    if (filterData == nullptr)
    {
        return CSOSFILTER_FAILURE;
    }

    // Ensure we have enough statically allocated memory to copy into
    if (sosMatrix.size() > CSOSFILTER_MAXSECTIONS)
    {
        return CSOSFILTER_FAILURE;
    }

    // Convert c++ std::vec matrix to c array, reuse filterData memory
    for (size_t i = 0; i < sosMatrix.size(); i++)
    {
        // Ensure the right number of coeffs per section
        if (sosMatrix[i].size() != CSOSFILTER_COEFFSPERSECTION)
        {
            return CSOSFILTER_FAILURE;
        }

        for (size_t j = 0; j < CSOSFILTER_COEFFSPERSECTION; j++)
        {
            filterData->sosMatrix[i][j] = sosMatrix[i][j];
        }
    }

    // Use init function to convert (and check) to C struct
    // Null check internal
    return csosFilterInit(filterData, &filterData->sosMatrix[0][0], sosMatrix.size(),
                          sosGain.data(), sosGain.size());
}

}