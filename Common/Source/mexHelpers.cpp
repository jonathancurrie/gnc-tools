/**
 * @file mexHelpers.cpp
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 04-Apr-2023
 * @brief A collection of routines for helping with MATLAB MEX files
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#include <stdarg.h>
#include <cmath>
#include <algorithm>
#include "mexHelpers.h"

namespace GNCTools
{
    //
    // Data Creation
    //
    mxArray* MEX::createDoubleScalar(double val) noexcept
    {
        return mxCreateDoubleScalar(val);
    }

    mxArray* MEX::createDoubleMatrix(size_t nrow, size_t ncol) noexcept
    {
        return mxCreateDoubleMatrix(nrow, ncol, mxREAL);
    }

    mxArray* MEX::createUInt8(uint8_t val) noexcept
    {
        mxArray* ptr = mxCreateNumericMatrix(1, 1, mxUINT8_CLASS, mxREAL);
        uint8_t* dataPtr = static_cast<uint8_t*>(mxGetData(ptr));
        *dataPtr = val;
        return ptr;
    }

    mxArray* MEX::createInt8(int8_t val) noexcept
    {
        mxArray* ptr = mxCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
        int8_t* dataPtr = static_cast<int8_t*>(mxGetData(ptr));
        *dataPtr = val;
        return ptr;
    }

    mxArray* MEX::createStruct(const char* fieldNames[], int numFields)
    {
        if ((fieldNames == nullptr) || (numFields <= 0))
        {
            MEX::error("GNCToolsMEX:createStruct","Error creating structure with %d fields", numFields);
            return nullptr;
        }
        return mxCreateStructMatrix(1, 1, numFields, fieldNames);
    }

    double* MEX::createFieldDoubleScalar(mxArray* data, const char* fieldName, double val)
    {
        if (MEX::isValidStruct(data) && (fieldName != nullptr))
        { 
            mxArray* field = MEX::createDoubleScalar(val);
            mxSetField(data, 0, fieldName, field);
            return mxGetPr(field);
        }
        else
        {
            MEX::error("GNCToolsMEX:createFieldDoubleScalar","Cannot create double scalar within structure field '%s'", fieldName);
            return nullptr;
        }
    }

    double* MEX::createFieldDoubleMatrix(mxArray* data, const char* fieldName, size_t nrow, size_t ncol)
    {
        if (MEX::isValidStruct(data) && (fieldName != nullptr))
        { 
            mxArray* field = MEX::createDoubleMatrix(nrow, ncol);
            if (field != nullptr)
            {
                mxSetField(data, 0, fieldName, field);
                return mxGetPr(field);
            }
        }
        // Failed by here
        MEX::error("GNCToolsMEX:createFieldDoubleMatrix","Cannot create double matrix within structure field '%s'", fieldName);
        return nullptr;
    }

    mxArray* MEX::createFieldString(mxArray* data, const char* fieldName, const char* str)
    {
        if (MEX::isValidStruct(data) && (fieldName != nullptr) && (str != nullptr))
        { 
            mxArray* field = mxCreateString(str);
            if (field != nullptr)
            {
                mxSetField(data, 0, fieldName, field);
                return field;
            }
        }
        // Failed by here
        MEX::error("GNCToolsMEX:createFieldString","Cannot create string within structure field '%s'", fieldName);
        return nullptr;
    }


    //
    // Data Access
    //
    double MEX::getDoubleScalar(const mxArray* data)
    {
        if (MEX::isDoubleScalar(data))
        {
            return *mxGetPr(data);
        }
        MEX::error("GNCToolsMEX:getDoubleScalar","Does not contain a double scalar");
        return NAN;
    }

    double* MEX::getDoubleVector(const mxArray* data)
    {
        if (MEX::isDoubleVector(data))
        {
            return mxGetPr(data);
        }
        MEX::error("GNCToolsMEX:getDoubleVector","Does not contain a double vector");
        return nullptr;
    }

    double MEX::getDoubleScalarField(const mxArray* data, const char* field)
    {
        return MEX::getDoubleScalar(MEX::getField(data, field));
    }

    double MEX::getDoubleScalarProperty(const mxArray* data, const char* className, const char* property)
    {
        return MEX::getDoubleScalar(MEX::getProperty(data, className, property));
    }
    
    std::string MEX::getString(const mxArray* data)
    {
        if (MEX::isString(data))
        {
            char strBuf[2048];
            mxGetString(data, strBuf, 2048);
            return std::string(strBuf);
        }
        MEX::error("GNCToolsMEX:getString","Variable does not contain a string variable");
        return "";
    }

    // Access logical (or double) in standard mxArray
    bool MEX::getLogicalScalar(const mxArray* data)
    {
        if (MEX::isEmpty(data) == false)
        {
            if (MEX::isScalar(data) == true)
            {
                if (MEX::isRealDenseDouble(data) == true)
                {
                    return (MEX::getDoubleScalar(data) != 0.0);
                }
                else if (mxGetClassID(data) == mxLOGICAL_CLASS)
                {
                    return mxIsLogicalScalarTrue(data);
                }
                else
                {
                    MEX::error("GNCToolsMEX:getLogicalScalar","Variable is not a double or logical!");
                }
            }
            else
            {
                MEX::error("GNCToolsMEX:getLogicalScalar","Variable is not scalar!");
            }
        }
        else
        {
            MEX::error("GNCToolsMEX:getLogicalScalar","Variable is empty!");        
        }
        return false;
    }

    mxArray* MEX::getField(const mxArray* data, const char* fieldName)
    {
        if (MEX::isValidField(data, fieldName))
        {
            return mxGetField(data, 0, fieldName);
        }
        
        MEX::error("GNCToolsMEX:getField","Cannot access field '%s' in structure", fieldName);
        return nullptr;
    }

    mxArray* MEX::getProperty(const mxArray* data, const char* className, const char* propertyName)
    {
        if (MEX::isValidProperty(data, className, propertyName))
        {
            return mxGetProperty(data, 0, propertyName);
        }
        
        MEX::error("GNCToolsMEX:getProperty","Cannot access property '%s' in object '%s'", propertyName, className);
        return nullptr;
    }

    size_t MEX::getNumElem(const mxArray* data) noexcept
    {
        if (MEX::isEmpty(data) == false)
        {
            return mxGetNumberOfElements(data); 
        }
        return 0;
    }

    size_t MEX::getNumRows(const mxArray* data) noexcept
    {
        if (MEX::isEmpty(data) == false)
        {
            return mxGetM(data); 
        }
        return 0;
    }

    size_t MEX::getNumCols(const mxArray* data) noexcept
    {
        if (MEX::isEmpty(data) == false)
        {
            return mxGetN(data); 
        }
        return 0;
    }

    
    //
    // Data Validation
    //
    bool MEX::isDoubleScalar(const mxArray* data) noexcept
    {
        if (MEX::isRealDenseDouble(data) && (MEX::getNumElem(data) == 1))
        {
            return true;
        }
        return false;
    }

    bool MEX::isDoubleVector(const mxArray* data) noexcept
    {
        if (MEX::isRealDenseDouble(data) && MEX::isVector(data))
        {
            return true;
        }
        return false;
    }

    bool MEX::isDoubleMatrix(const mxArray* data) noexcept
    {
        if (MEX::isRealDenseDouble(data) && MEX::isMatrix(data))
        {
            return true;
        }
        return false;
    }

    bool MEX::isRealDenseDouble(const mxArray* data) noexcept
    {
        if ((data != nullptr) && (MEX::isEmpty(data) == false))
        {
            return ((mxIsComplex(data) == false) && 
                    (mxGetClassID(data) == mxDOUBLE_CLASS) &&
                    (mxIsSparse(data) == false));
        }
        return false;
    }

    bool MEX::isEmpty(const mxArray* data) noexcept
    {
        if (data != nullptr)
        {
            return mxIsEmpty(data);
        }
        return true; // assume nullptr is empty
    }

    bool MEX::isMatrix(const mxArray* data) noexcept
    {
        return (getNumRows(data) > 1) && (getNumCols(data) > 1);
    }

    bool MEX::isVector(const mxArray* data) noexcept
    {
        return (getNumRows(data) > 1) ^ (getNumCols(data) > 1);
    }

    bool MEX::isScalar(const mxArray* data) noexcept
    {
        return getNumElem(data) == 1;
    }

    bool MEX::isString(const mxArray* data) noexcept
    {
        if (data != nullptr)
        {
            return mxIsChar(data) && (mxIsEmpty(data) == false);
        }
        return false;
    }

    bool MEX::isFunction(const mxArray* data) noexcept
    {
        if (data != nullptr)
        {
            return (mxGetClassID(data) == mxFUNCTION_CLASS) && (mxIsEmpty(data) == false);
        }
        return false;
    }
        
    bool MEX::isValidStruct(const mxArray* data) noexcept
    {
        if (data != nullptr)
        {
            return mxIsStruct(data) && (mxIsEmpty(data) == false);
        }
        return false;
    }
    
    bool MEX::isValidField(const mxArray* data, const char* field) noexcept
    {
        if (MEX::isValidStruct(data) && (field != nullptr))
        {
            mxArray* temp = mxGetField(data, 0, field);
            if (temp != nullptr)
            {
                return (mxIsEmpty(temp) == false);
            }
        }
        return false;
    }

    bool MEX::isValidClass(const mxArray* data, const char* className) noexcept
    {
        if ((data != nullptr) && (className != nullptr))
        {
            return mxIsClass(data, className);
        }
        return false;
    }

    bool MEX::isValidProperty(const mxArray* data, const char* className, const char* property) noexcept
    {
        if (MEX::isValidClass(data, className) && (property != nullptr))
        {
            mxArray* temp = mxGetProperty(data, 0, property);
            if (temp != nullptr)
            {
                return (mxIsEmpty(temp) == false);
            }
        }
        return false;
    }
    
    bool MEX::checkForRequiredFields(const mxArray* data, const std::vector<std::string>& fields)
    {
        for (const std::string& str : fields)
        {
            if (MEX::isValidField(data, str.c_str()) == false)
            {
                MEX::error("GNCToolsMEX:checkForRequiredFields","The structure was missing the field '%s', or it was empty", str.c_str());
                return false;
            }
        }
        // OK by here
        return true;
    }

    bool MEX::checkForRequiredProperties(const mxArray* data, const char* className, const std::vector<std::string>& properties)
    {
        for (const std::string& str : properties)
        {
            if (MEX::isValidProperty(data, className, str.c_str()) == false)
            {
                MEX::error("GNCToolsMEX:checkForRequiredProperties","The object '%s' was missing the field '%s', or it was empty", className, str.c_str());
                return false;
            }
        }
        // OK by here
        return true;
    }

    bool MEX::containsNaNInf(const mxArray* data)
    {
        if (MEX::isRealDenseDouble(data))
        {
            size_t numElem = MEX::getNumElem(data);
            double* ptr    = mxGetPr(data);
            for (size_t i = 0; i < numElem; i++)
            {
                if (std::isnan(ptr[i]) || std::isinf(ptr[i]))
                {
                    return true;
                }
            }
            return false; // OK by here
        }
        else
        {
            MEX::error("GNCToolsMEX:containsNaNInf","Cannot check if data contains NaN/Inf, a double argument was not supplied");
            return true;
        }    
    }

    
    //
    // Error Reporting (throws exception too)
    //
    void MEX::error(const char* id, const char* format, ...)
    {
        char errBuf[2048];
        va_list args;
        va_start(args, format);
        vsnprintf(errBuf, 2048, format, args);
        va_end(args);
        
        // Generates Exception
        mexErrMsgIdAndTxt(id,errBuf);
    }
    
    void MEX::error(const char* format, ...)
    {
        char errBuf[2048];
        va_list args;
        va_start(args, format);
        vsnprintf(errBuf, 2048, format, args);
        va_end(args);
        
        // Generates Exception
        mexErrMsgIdAndTxt("GNCToolsMEX:Error",errBuf);
    }

    //
    // Warning Reporting (allows continuing)
    // 
    void MEX::warning(const char* id, const char* format, ...) noexcept
    {
        char warnBuf[2048];
        va_list args;
        va_start(args, format);
        vsnprintf(warnBuf, 2048, format, args);
        va_end(args);

        mexWarnMsgIdAndTxt(id, warnBuf);
    }

    void MEX::warning(const char* format, ...) noexcept
    {
        char warnBuf[2048];
        va_list args;
        va_start(args, format);
        vsnprintf(warnBuf, 2048, format, args);
        va_end(args);

        mexWarnMsgIdAndTxt("GNCToolsMEX:Warning", warnBuf);
    }


    //
    // Utilities
    //
    std::string Utils::toLower(const std::string& strIn) noexcept
    {
        // Take a copy of the string, then transform and return
        std::string strOut = strIn;
        std::transform(strOut.begin(), strOut.end(), strOut.begin(),
                        [](unsigned char c){ return std::tolower(c); });
        return strOut;
    }
}