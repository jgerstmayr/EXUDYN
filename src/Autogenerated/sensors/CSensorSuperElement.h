/** ***********************************************************************************************
* @class        CSensorSuperElementParameters
* @brief        Parameter class for CSensorSuperElement
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-09-28  00:34:30 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CSENSORSUPERELEMENTPARAMETERS__H
#define CSENSORSUPERELEMENTPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CSensorSuperElementParameters
class CSensorSuperElementParameters // AUTO:
{
public: // AUTO:
    Index bodyNumber;                             //!< AUTO: body (=object) number to which sensor is attached to
    Index meshNodeNumber;                         //!< AUTO: must be >= 0; mesh node number, which is a local node number with in the object (starting with 0); the node number may represent a real Node in mbs, or may be virtual and reconstructed from the object coordinates such as in ObjectFFRFreducedOrder
    bool writeToFile;                             //!< AUTO: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
    std::string fileName;                         //!< AUTO: directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist
    OutputVariableType outputVariableType;        //!< AUTO: OutputVariableType for sensor, based on the output variables available for the mesh nodes (see special section for super element output variables, e.g, in ObjectFFRFreducedOrder, [](#sec-objectffrfreducedorder-superelementoutput))
    bool storeInternal;                           //!< AUTO: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
    //! AUTO: default constructor with parameter initialization
    CSensorSuperElementParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        meshNodeNumber = EXUstd::InvalidIndex;
        writeToFile = true;
        fileName = "";
        outputVariableType = OutputVariableType::_None;
        storeInternal = false;
    };
};


/** ***********************************************************************************************
* @class        CSensorSuperElement
* @brief        A sensor attached to a mesh node of a superelement, which measures one of the output variables of the superelement at that mesh node.
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

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index bodyNumber) override
    {
        parameters.bodyNumber = bodyNumber;
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
