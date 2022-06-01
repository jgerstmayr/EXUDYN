/** ***********************************************************************************************
* @class        CNodeGenericAEParameters
* @brief        Parameter class for CNodeGenericAE
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-05-23  22:06:23 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CNODEGENERICAEPARAMETERS__H
#define CNODEGENERICAEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodeGenericAEParameters
class CNodeGenericAEParameters // AUTO: 
{
public: // AUTO: 
    Vector referenceCoordinates;                  //!< AUTO: generic reference coordinates of node; must be consistent with numberOfAECoordinates
    Index numberOfAECoordinates;                  //!< AUTO: number of generic \hac{AE} coordinates
    //! AUTO: default constructor with parameter initialization
    CNodeGenericAEParameters()
    {
        referenceCoordinates = Vector();
        numberOfAECoordinates = 0;
    };
};


/** ***********************************************************************************************
* @class        CNodeGenericAE
* @brief        A node containing a number of \hac{AE} variables; use e.g. linear state space systems. Note that referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
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

//! AUTO: CNodeGenericAE
class CNodeGenericAE: public CNodeAE // AUTO: 
{
protected: // AUTO: 
    CNodeGenericAEParameters parameters; //! AUTO: contains all parameters for CNodeGenericAE

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodeGenericAEParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodeGenericAEParameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfAECoordinates() const override
    {
        return parameters.numberOfAECoordinates;;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return Node::GenericAE;
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
            (Index)OutputVariableType::Coordinates );
    }

};



#endif //#ifdef include once...
