/** ***********************************************************************************************
* @class        CNodePointGroundParameters
* @brief        Parameter class for CNodePointGround
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-09-08  18:14:39 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CNODEPOINTGROUNDPARAMETERS__H
#define CNODEPOINTGROUNDPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodePointGroundParameters
class CNodePointGroundParameters // AUTO: 
{
public: // AUTO: 
    Vector3D referenceCoordinates;                //!< AUTO: reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
    //! AUTO: default constructor with parameter initialization
    CNodePointGroundParameters()
    {
        referenceCoordinates = Vector3D({0.,0.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodePointGround
* @brief        A 3D point node fixed to ground. The node can be used as NodePoint, but it does not generate coordinates. Applied or reaction forces do not have any effect.
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

//! AUTO: CNodePointGround
class CNodePointGround: public CNodeODE2 // AUTO: 
{
protected: // AUTO: 
    CNodePointGroundParameters parameters; //! AUTO: contains all parameters for CNodePointGround

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodePointGroundParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodePointGroundParameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 0;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return (Node::Type)(Node::Position + Node::Position2D + Node::GenericODE2 + Node::Ground);
    }

    //! AUTO:  Returns position of node, which is the reference position for all configurations
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override
    {
        return parameters.referenceCoordinates;
    }

    //! AUTO:  Returns zero velocity
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override
    {
        return Vector3D(0.);
    }

    //! AUTO:  return zero sized matrix for ground node (no action)
    virtual void GetPositionJacobian(Matrix& value) const override
    {
        value.SetNumberOfRowsAndColumns(0,0);
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
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t );
    }

};



#endif //#ifdef include once...
