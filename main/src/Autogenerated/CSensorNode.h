/** ***********************************************************************************************
* @class        CSensorNodeParameters
* @brief        Parameter class for CSensorNode
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-07-20  12:33:24 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CSENSORNODEPARAMETERS__H
#define CSENSORNODEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"


//! AUTO: Parameters for class CSensorNodeParameters
class CSensorNodeParameters // AUTO: 
{
public: // AUTO: 
    Index nodeNumber;                             //!< AUTO: node number to which sensor is attached to
    bool writeToFile;                             //!< AUTO: true: write sensor output to file
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType
    OutputVariableType outputVariableType;        //!< AUTO: OutputVariableType for sensor
    //! AUTO: default constructor with parameter initialization
    CSensorNodeParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
        writeToFile = true;
        fileName = "";
        outputVariableType = OutputVariableType::_None;
    };
};


/** ***********************************************************************************************
* @class        CSensorNode
* @brief        A sensor attached to a node. The sensor measures OutputVariables and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. A user function can be attached to modify sensor values accordingly.
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

//! AUTO: CSensorNode
class CSensorNode: public CSensor // AUTO: 
{
protected: // AUTO: 
    CSensorNodeParameters parameters; //! AUTO: contains all parameters for CSensorNode

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CSensorNodeParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CSensorNodeParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to node number
    virtual Index GetNodeNumber() const override
    {
        return parameters.nodeNumber;
    }

    //! AUTO:  return sensor type
    virtual SensorType GetType() const override
    {
        return SensorType::Node;
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
