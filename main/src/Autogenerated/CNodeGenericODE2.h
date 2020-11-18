/** ***********************************************************************************************
* @class        CNodeGenericODE2Parameters
* @brief        Parameter class for CNodeGenericODE2
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-11-12  23:05:47 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CNODEGENERICODE2PARAMETERS__H
#define CNODEGENERICODE2PARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodeGenericODE2Parameters
class CNodeGenericODE2Parameters // AUTO: 
{
public: // AUTO: 
    Vector referenceCoordinates;                  //!< AUTO: generic reference coordinates of node; must be consistent with numberOfODE2Coordinates
    Index numberOfODE2Coordinates;                //!< AUTO: number of generic ODE2 coordinates
    //! AUTO: default constructor with parameter initialization
    CNodeGenericODE2Parameters()
    {
        referenceCoordinates = Vector();
        numberOfODE2Coordinates = 0;
    };
};


/** ***********************************************************************************************
* @class        CNodeGenericODE2
* @brief        A node containing a number of ODE2 variables; use e.g. for scalar dynamic equations (Mass1D) or for the ALECable element.
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

//! AUTO: CNodeGenericODE2
class CNodeGenericODE2: public CNodeODE2 // AUTO: 
{
protected: // AUTO: 
    CNodeGenericODE2Parameters parameters; //! AUTO: contains all parameters for CNodeGenericODE2

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodeGenericODE2Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodeGenericODE2Parameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return parameters.numberOfODE2Coordinates;;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return Node::GenericODE2;
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector; this makes no sense for NodeGenericODE2, but necessary for consistency; FUTURE: add 'drawable' flag to nodes in order to exclude drawing
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override
    {
        return Vector3D({0.,0.,0.});
    }

    //! AUTO:  return internally stored reference coordinates of node
    virtual LinkedDataVector GetReferenceCoordinateVector() const override
    {
        return parameters.referenceCoordinates;
    }

    //! AUTO:  provide according output variable in 'value'; used e.g. for postprocessing and sensors
    virtual void GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration, Vector& value) const override;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t +
            (Index)OutputVariableType::Coordinates_tt );
    }

};



#endif //#ifdef include once...
