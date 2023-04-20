/**
 * @file mexHelpers.h
 * @author Jonathan Currie (controlengineering.co.nz)
 * @date 03-Apr-2023
 * @brief A collection of routines for helping with MATLAB MEX files
 * 
 * Copyright (C) Jonathan Currie 2023 (www.controlengineering.co.nz)
 */

#ifndef MEXHELPERS_H
#define MEXHELPERS_H

#include <mex.h>
#include <vector>
#include <string>

namespace GNCTools
{

//
// MEX Helper Methods
// NOTE: All double methods assume real and dense (not complex or sparse)
//
class MEX
{
    public:
        // Data Creation
        static mxArray* createDoubleScalar(double val = 0.0) noexcept;
        static mxArray* createUInt8(uint8_t val = 0) noexcept;
        static mxArray* createInt8(int8_t val = 0) noexcept;

        // Data Access
        static double getDoubleScalar(const mxArray* data);
        static double getDoubleScalarField(const mxArray* data, const char* field);
        static double getDoubleScalarProperty(const mxArray* data, const char* className, const char* property);
        static std::string getString(const mxArray* data);
        static bool getLogicalScalar(const mxArray* data);
        static mxArray* getField(const mxArray* data, const char* fieldName);
        static mxArray* getProperty(const mxArray* data, const char* className, const char* propertyName);
        static size_t getNumElem(const mxArray* data) noexcept;
        static size_t getNumRows(const mxArray* data) noexcept;
        static size_t getNumCols(const mxArray* data) noexcept;

        // Data Validation (no throw)
        static bool isDoubleScalar(const mxArray* data) noexcept;
        static bool isDoubleVector(const mxArray* data) noexcept;
        static bool isDoubleMatrix(const mxArray* data) noexcept;
        static bool isRealDenseDouble(const mxArray* data) noexcept;
        static bool isEmpty(const mxArray* data) noexcept;
        static bool isMatrix(const mxArray* data) noexcept;
        static bool isVector(const mxArray* data) noexcept;
        static bool isScalar(const mxArray* data) noexcept;
        static bool isString(const mxArray* data) noexcept;
        static bool isValidStruct(const mxArray* data) noexcept;
        static bool isValidField(const mxArray* data, const char* field) noexcept;
        static bool isValidClass(const mxArray* data, const char* className) noexcept;
        static bool isValidProperty(const mxArray* data, const char* className, const char* property) noexcept;
        static bool isFunction(const mxArray* data) noexcept;
        static bool containsNaNInf(const mxArray* data); // throws if not double supplied

        // Data Validation (throws if not)
        static bool checkForRequiredFields(const mxArray* data, const std::vector<std::string>& fields);
        static bool checkForRequiredProperties(const mxArray* data, const char* className, const std::vector<std::string>& properties);

        // Error Reporting (throws exception too)
        static void error(const char* id, const char* format, ...); 
        static void error(const char* format, ...); 
        static void warning(const char* id, const char* format, ...) noexcept; 
        static void warning(const char* format, ...) noexcept; 
};

//
// Misc Utils
//
class Utils
{
    public:
        static std::string toLower(const std::string& strIn) noexcept;
};

}


#endif /* MEXHELPERS_H */
