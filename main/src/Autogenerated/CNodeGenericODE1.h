/** ***********************************************************************************************
* @class        CNodeGenericODE1Parameters
* @brief        Parameter class for CNodeGenericODE1
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-10-26  18:53:42 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CNODEGENERICODE1PARAMETERS__H
#define CNODEGENERICODE1PARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodeGenericODE1Parameters
class CNodeGenericODE1Parameters // AUTO: 
{
public: // AUTO: 
    Vector referenceCoordinates;                  //!< AUTO: generic reference coordinates of node; must be consistent with numberOfODE1Coordinates
    Index numberOfODE1Coordinates;                //!< AUTO: number of generic \hac{ODE1} coordinates
    //! AUTO: default constructor with parameter initialization
    CNodeGenericODE1Parameters()
    {
        referenceCoordinates = Vector();
        numberOfODE1Coordinates = 0;
    };
};


/** ***********************************************************************************************
* @class        CNodeGenericODE1
* @brief        A node containing a number of \hac{ODE1} variables; use e.g. linear state space systems. Note that referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
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

//! AUTO: CNodeGenericODE1
class CNodeGenericODE1: public CNodeODE1 // AUTO: 
{
protected: // AUTO: 
    CNodeGenericODE1Parameters parameters; //! AUTO: contains all parameters for CNodeGenericODE1

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodeGenericODE1Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodeGenericODE1Parameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE1Coordinates() const override
    {
        return parameters.numberOfODE1Coordinates;;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return Node::GenericODE1;
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
            (Index)OutputVariableType::CoordinatesTotal +
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t );
    }

};



#endif //#ifdef include once...
