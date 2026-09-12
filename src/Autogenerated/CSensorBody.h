/** ***********************************************************************************************
* @class        CSensorBodyParameters
* @brief        Parameter class for CSensorBody
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2025-05-05  23:27:44 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CSENSORBODYPARAMETERS__H
#define CSENSORBODYPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CSensorBodyParameters
class CSensorBodyParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body (=object) number to which sensor is attached to
    Vector3D localPosition;                       //!< AUTO: local (body-fixed) body position of sensor
    bool writeToFile;                             //!< AUTO: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
    OutputVariableType outputVariableType;        //!< AUTO: OutputVariableType for sensor
    bool storeInternal;                           //!< AUTO: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
    //! AUTO: default constructor with parameter initialization
    CSensorBodyParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        localPosition = Vector3D({0.,0.,0.});
        writeToFile = true;
        fileName = "";
        outputVariableType = OutputVariableType::_None;
        storeInternal = false;
    };
};


/** ***********************************************************************************************
* @class        CSensorBody
* @brief        A sensor attached to a body-object with local position \f$\pLocB\f$. As a difference to SensorObject, the body sensor needs a local position at which the sensor is attached to. The sensor measures OutputVariableBody and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
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

//! AUTO: CSensorBody
class CSensorBody: public CSensor // AUTO: 
{
protected: // AUTO: 
    CSensorBodyParameters parameters; //! AUTO: contains all parameters for CSensorBody

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CSensorBodyParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CSensorBodyParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber() const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index bodyNumber) override
    {
        parameters.bodyNumber = bodyNumber;
    }

    //! AUTO:  return sensor type
    virtual SensorType GetType() const override
    {
        return SensorType::Body;
    }

    //! AUTO:  get local position
    Vector3D GetBodyLocalPosition() const
    {
        return parameters.localPosition;
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
        return parameters.outputVariableType;
    }

    //! AUTO:  main function to generate sensor output values
    virtual void GetSensorValues(const CSystemData& cSystemData, Vector& values, ConfigurationType configuration = ConfigurationType::Current) const override;

};



#endif //#ifdef include once...
