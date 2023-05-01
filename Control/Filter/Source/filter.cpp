/**
 * @file filter.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 01-May-2023
 * @brief A wrapper around the C Filter Digital Filter (cfilter.c)
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include "filter.hpp"

namespace GNCTools
{
//
// Filter Constructors
//
Filter::Filter(const FilterParams& params)
{
    _initFromParams(params); // checks parameters internally
}

Filter::Filter(const std::vector<double>& numIn,
               const std::vector<double>& denIn)
{
    _initFromParams(FilterParams(numIn, denIn));
}

//
// Filter Update
// 
int Filter::update(double u, double& y)
{
    if (isInitialized())
    {
        return cfilterUpdate(&_filterData, u, &y);
    }
    return CFILTER_FAILURE;
}

//
// Filter Reset
//
int Filter::reset(void)
{
    if (isInitialized())
    {
        return cfilterReset(&_filterData);
    }
    return CFILTER_FAILURE;
}

//
// Initialization
//
int Filter::_initFromParams(const FilterParams& params)
{
    // Use the C Filter conversion to initialize this object's C Filter data structure
    int status = params.makeCData(&_filterData);

    if (status == CFILTER_SUCCESS)
    {
        _initialized = true;
    }                                
    return status;
}

//
// Parameter Checking
//
int Filter::checkParams(const FilterParams& params)
{
    // Use the C Filter conversion to do the checking for us
    cfilterData_t tempCFilter{};
    return params.makeCData(&tempCFilter);
}

//
// Convert FilterParams to C Filter Data Structure
//
int FilterParams::makeCData(cfilterData_t* filterData) const
{
    // Use init function to convert (and check) to C struct
    // Null check internal
    return cfilterInit(filterData, num.data(), num.size(), den.data(), den.size());
}

}