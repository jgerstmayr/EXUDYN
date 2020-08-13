/** ***********************************************************************************************
* @class        CNodeGenericDataParameters
* @brief        Parameter class for CNodeGenericData
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-07-20  12:33:23 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CNODEGENERICDATAPARAMETERS__H
#define CNODEGENERICDATAPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"


//! AUTO: Parameters for class CNodeGenericDataParameters
class CNodeGenericDataParameters // AUTO: 
{
public: // AUTO: 
    Index numberOfDataCoordinates;                //!< AUTO: number of generic data coordinates (history variables)
    //! AUTO: default constructor with parameter initialization
    CNodeGenericDataParameters()
    {
        numberOfDataCoordinates = 0;
    };
};


/** ***********************************************************************************************
* @class        CNodeGenericData
* @brief        A node containing a number of data (history) variables; use e.g. for contact (active set), friction or plasticity (history variable).
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

//! AUTO: CNodeGenericData
class CNodeGenericData: public CNodeData // AUTO: 
{
protected: // AUTO: 
    CNodeGenericDataParameters parameters; //! AUTO: contains all parameters for CNodeGenericData

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodeGenericDataParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodeGenericDataParameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of data coordinates
    virtual Index GetNumberOfDataCoordinates() const override
    {
        return parameters.numberOfDataCoordinates;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return Node::GenericData;
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
