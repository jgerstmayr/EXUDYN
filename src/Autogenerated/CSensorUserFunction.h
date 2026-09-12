/** ***********************************************************************************************
* @class        CSensorUserFunctionParameters
* @brief        Parameter class for CSensorUserFunction
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2025-05-06  23:20:45 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CSENSORUSERFUNCTIONPARAMETERS__H
#define CSENSORUSERFUNCTIONPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11

//! AUTO: Parameters for class CSensorUserFunctionParameters
class CSensorUserFunctionParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex sensorNumbers;                     //!< AUTO: optional list of \f$n\f$ sensor numbers for use in user function
    Vector factors;                               //!< AUTO: optional list of \f$m\f$ factors which can be used, e.g., for weighting sensor values
    bool writeToFile;                             //!< AUTO: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,StdArrayIndex,StdVector,ConfigurationType)> > sensorUserFunction;//!< AUTO: A Python function which defines the time-dependent user function, which usually evaluates one or several sensors and computes a new sensor value, see example
    bool storeInternal;                           //!< AUTO: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
    //! AUTO: default constructor with parameter initialization
    CSensorUserFunctionParameters()
    {
        sensorNumbers = ArrayIndex();
        factors = Vector();
        writeToFile = true;
        fileName = "";
        sensorUserFunction = 0;
        storeInternal = false;
    };
};


/** ***********************************************************************************************
* @class        CSensorUserFunction
* @brief        A sensor defined by a user function. The sensor is intended to collect sensor values of a list of given sensors and recombine the output into a new value for output or control purposes. It is also possible to use this sensor without any dependence on other sensors in order to generate output for, e.g., any quantities in mbs or solvers.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

//! AUTO: CSensorUserFunction
class CSensorUserFunction: public CSensor // AUTO: 
{
protected: // AUTO: 
    CSensorUserFunctionParameters parameters; //! AUTO: contains all parameters for CSensorUserFunction

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CSensorUserFunctionParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CSensorUserFunctionParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to sensor number
    virtual Index GetSensorNumber(Index localIndex) const override
    {
        return parameters.sensorNumbers[localIndex];
    }

    //! AUTO:  total number of dependent sensors
    virtual Index GetNumberOfSensors() const override
    {
        return parameters.sensorNumbers.NumberOfItems();
    }

    //! AUTO:  change sensorNumber
    virtual void SetSensorNumber(Index localIndex, Index sensorNumber) override
    {
        parameters.sensorNumbers[localIndex] = sensorNumber;
    }

    //! AUTO:  return sensor type
    virtual SensorType GetType() const override
    {
        return SensorType::UserFunction;
    }

    //! AUTO:  get writeToFile flag
    virtual bool GetWriteToFileFlag() const override
    {
        return parameters.writeToFile;
    }

    //! AUTO:  get storeInternal flag
    virtual bool GetStoreInternalFlag() const override
    {
        return parameters.storeInternal;
    }

    //! AUTO:  get file name
    virtual STDstring GetFileName() const override
    {
        return parameters.fileName;
    }

    //! AUTO:  get OutputVariableType
    virtual OutputVariableType GetOutputVariableType() const override
    {
        return OutputVariableType::_None;
    }

    //! AUTO:  main function to generate sensor output values
    virtual void GetSensorValues(const CSystemData& cSystemData, Vector& values, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunction(Vector& sensorValues, const MainSystemBase& mainSystem, Real t, ConfigurationType configuration) const;

};



#endif //#ifdef include once...
