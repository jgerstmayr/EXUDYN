/** ***********************************************************************************************
* @class        CSensorLoadParameters
* @brief        Parameter class for CSensorLoad
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

#ifndef CSENSORLOADPARAMETERS__H
#define CSENSORLOADPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CSensorLoadParameters
class CSensorLoadParameters // AUTO: 
{
public: // AUTO: 
    Index loadNumber;                             //!< AUTO: load number to which sensor is attached to
    bool writeToFile;                             //!< AUTO: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
    bool storeInternal;                           //!< AUTO: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
    //! AUTO: default constructor with parameter initialization
    CSensorLoadParameters()
    {
        loadNumber = EXUstd::InvalidIndex;
        writeToFile = true;
        fileName = "";
        storeInternal = false;
    };
};


/** ***********************************************************************************************
* @class        CSensorLoad
* @brief        A sensor attached to a load. The sensor measures the load values and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
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

//! AUTO: CSensorLoad
class CSensorLoad: public CSensor // AUTO: 
{
protected: // AUTO: 
    CSensorLoadParameters parameters; //! AUTO: contains all parameters for CSensorLoad

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CSensorLoadParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CSensorLoadParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to load number
    virtual Index GetLoadNumber() const override
    {
        return parameters.loadNumber;
    }

    //! AUTO:  change loadNumber
    virtual void SetLoadNumber(Index loadNumber) override
    {
        parameters.loadNumber = loadNumber;
    }

    //! AUTO:  return sensor type
    virtual SensorType GetType() const override
    {
        return SensorType::Load;
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

};



#endif //#ifdef include once...
