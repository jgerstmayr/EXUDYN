/** ***********************************************************************************************
* @class        CNodePoint2DParameters
* @brief        Parameter class for CNodePoint2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-10-10  10:22:41 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"


//! AUTO: Parameters for class CNodePoint2DParameters
class CNodePoint2DParameters // AUTO: 
{
public: // AUTO: 
    Vector2D referenceCoordinates;                //!< AUTO: reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
    //! AUTO: default constructor with parameter initialization
    CNodePoint2DParameters()
    {
        referenceCoordinates = Vector2D({0.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodePoint2D
* @brief        A 2D point node for point masses or solid finite elements which has 2 displacement degrees of freedom for second order differential equations.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! AUTO: CNodePoint2D
class CNodePoint2D: public CNodeODE2 // AUTO: 
{
protected: // AUTO: 
    CNodePoint2DParameters parameters; //! AUTO: contains all parameters for CNodePoint2D

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodePoint2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodePoint2DParameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 2;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual CNodeType GetType() const override
    {
        return CNodeType::Point;
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node; returns always a 3D Vector
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  provide position jacobian of node; derivative of 3D Position with respect to 2 coordinates
    virtual void GetPositionJacobian(Matrix& value) const override
    {
        value.SetMatrix(3,2,{1.f,0.f,0.f,1.f,0.f,0.f});
    }

    //! AUTO:  return internally stored reference coordinates of node
    virtual LinkedDataVector GetReferenceCoordinateVector() const override
    {
        return parameters.referenceCoordinates;
    }

    //! AUTO:  provide according output variable in "value"; used e.g. for postprocessing and sensors
    virtual void GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration, Vector& value) const override;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t );
    }

};


