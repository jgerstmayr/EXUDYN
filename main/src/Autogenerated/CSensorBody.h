/** ***********************************************************************************************
* @class        CSensorBodyParameters
* @brief        Parameter class for CSensorBody
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-08-25  12:46:29 (last modfied)
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


//! AUTO: Parameters for class CSensorBodyParameters
class CSensorBodyParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body (=object) number to which sensor is attached to
    Vector3D localPosition;                       //!< AUTO: local (body-fixed) body position of sensor
    bool writeToFile;                             //!< AUTO: true: write sensor output to file
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
    OutputVariableType outputVariableType;        //!< AUTO: OutputVariableType for sensor
    //! AUTO: default constructor with parameter initialization
    CSensorBodyParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        localPosition = Vector3D({0.,0.,0.});
        writeToFile = true;
        fileName = "";
        outputVariableType = OutputVariableType::_None;
    };
};


/** ***********************************************************************************************
* @class        CSensorBody
* @brief        A sensor attached to a body-object with local position. As a difference to other ObjectSensors, the body sensor has a local position at which the sensor is attached to. The sensor measures OutputVariableBody and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. A user function can be attached to postprocess sensor values accordingly.
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
