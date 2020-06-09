/** ***********************************************************************************************
* @class        CSensorSuperElementParameters
* @brief        Parameter class for CSensorSuperElement
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-01  20:10:12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"


//! AUTO: Parameters for class CSensorSuperElementParameters
class CSensorSuperElementParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body (=object) number to which sensor is attached to
    Index meshNodeNumber;                         //!< AUTO: mesh node number, which is a local node number with in the object (starting with 0); the node number may represent a real Node in mbs, or may be virtual and reconstructed from the object coordinates such as in ObjectFFRFreducedOrder
    bool writeToFile;                             //!< AUTO: true: write sensor output to file
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType
    OutputVariableType outputVariableType;        //!< AUTO: OutputVariableType for sensor
    //! AUTO: default constructor with parameter initialization
    CSensorSuperElementParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        meshNodeNumber = -1;
        writeToFile = true;
        fileName = "";
        outputVariableType = OutputVariableType::_None;
    };
};


/** ***********************************************************************************************
* @class        CSensorSuperElement
* @brief        A sensor attached to a SuperElement-object with mesh node number. As a difference to other ObjectSensors, the SuperElement sensor has a mesh node number at which the sensor is attached to. The sensor measures OutputVariableSuperElement and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. A user function can be attached to postprocess sensor values accordingly.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! AUTO: CSensorSuperElement
class CSensorSuperElement: public CSensor // AUTO: 
{
protected: // AUTO: 
    CSensorSuperElementParameters parameters; //! AUTO: contains all parameters for CSensorSuperElement

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CSensorSuperElementParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CSensorSuperElementParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber() const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  return sensor type
    virtual SensorType GetType() const override
    {
        return SensorType::SuperElement;
    }

    //! AUTO:  get local position
    Index GetMeshNodeNumber() const
    {
        return parameters.meshNodeNumber;
    }

    //! AUTO:  get writeToFile flag
    virtual bool GetWriteToFileFlag() const override
    {
        return parameters.writeToFile;
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


