/** ***********************************************************************************************
* @class        CNodePointParameters
* @brief        Parameter class for CNodePoint
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-09  17:21:01 (last modfied)
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


//! AUTO: Parameters for class CNodePointParameters
class CNodePointParameters // AUTO: 
{
public: // AUTO: 
    Vector3D referenceCoordinates;                //!< AUTO: reference coordinates of node, e.g. ref. coordinates for finite elements; global position of node without displacement
    //! AUTO: default constructor with parameter initialization
    CNodePointParameters()
    {
        referenceCoordinates = Vector3D({0.,0.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodePoint
* @brief        A 3D point node for point masses or solid finite elements which has 3 displacement degrees of freedom for second order differential equations (ODE2).
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

//! AUTO: CNodePoint
class CNodePoint: public CNodeODE2 // AUTO: 
{
protected: // AUTO: 
    CNodePointParameters parameters; //! AUTO: contains all parameters for CNodePoint

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodePointParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodePointParameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 3;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return Node::Position;
    }

    //! AUTO:  return configuration dependent position of node
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  provide position jacobian of node
    virtual void GetPositionJacobian(Matrix& value) const override
    {
        value.SetScalarMatrix(3,1.);
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
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Acceleration +
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t +
            (Index)OutputVariableType::Coordinates_tt );
    }

};


