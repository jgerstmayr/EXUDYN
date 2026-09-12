/** ***********************************************************************************************
* @class        CSensorKinematicTreeParameters
* @brief        Parameter class for CSensorKinematicTree
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2025-05-05  23:27:45 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CSENSORKINEMATICTREEPARAMETERS__H
#define CSENSORKINEMATICTREEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CSensorKinematicTreeParameters
class CSensorKinematicTreeParameters // AUTO: 
{
public: // AUTO: 
    Index objectNumber;                           //!< AUTO: object number of KinematicTree to which sensor is attached to
    Index linkNumber;                             //!< AUTO: number of link in KinematicTree to measure quantities
    Vector3D localPosition;                       //!< AUTO: local (link-fixed) position of sensor, defined in link (\f$n_l\f$) coordinate system
    bool writeToFile;                             //!< AUTO: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
    OutputVariableType outputVariableType;        //!< AUTO: OutputVariableType for sensor
    bool storeInternal;                           //!< AUTO: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
    //! AUTO: default constructor with parameter initialization
    CSensorKinematicTreeParameters()
    {
        objectNumber = EXUstd::InvalidIndex;
        linkNumber = EXUstd::InvalidIndex;
        localPosition = Vector3D({0.,0.,0.});
        writeToFile = true;
        fileName = "";
        outputVariableType = OutputVariableType::_None;
        storeInternal = false;
    };
};


/** ***********************************************************************************************
* @class        CSensorKinematicTree
* @brief        A sensor attached to a KinematicTree with local position \f$\pLocB\f$ and link number \f$n_l\f$. As a difference to SensorBody, the KinematicTree sensor needs a local position and a link number, which defines the sub-body at which the sensor values are evaluated. The local position is given in sub-body (link) local coordinates. The sensor measures OutputVariableKinematicTree and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
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

//! AUTO: CSensorKinematicTree
class CSensorKinematicTree: public CSensor // AUTO: 
{
protected: // AUTO: 
    CSensorKinematicTreeParameters parameters; //! AUTO: contains all parameters for CSensorKinematicTree

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CSensorKinematicTreeParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CSensorKinematicTreeParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber() const override
    {
        return parameters.objectNumber;
    }

    //! AUTO:  change objectNumber
    virtual void SetObjectNumber(Index objectNumber) override
    {
        parameters.objectNumber = objectNumber;
    }

    //! AUTO:  return sensor type
    virtual SensorType GetType() const override
    {
        return SensorType::KinematicTree;
    }

    //! AUTO:  get local position
    Vector3D GetBodyLocalPosition() const
    {
        return parameters.localPosition;
    }

    //! AUTO:  general access to link number
    Index GetLinkNumber() const
    {
        return parameters.linkNumber;
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
