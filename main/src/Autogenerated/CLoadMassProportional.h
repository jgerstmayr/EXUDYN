/** ***********************************************************************************************
* @class        CLoadMassProportionalParameters
* @brief        Parameter class for CLoadMassProportional
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-12-01  17:12:39 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CLOADMASSPROPORTIONALPARAMETERS__H
#define CLOADMASSPROPORTIONALPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function

//! AUTO: Parameters for class CLoadMassProportionalParameters
class CLoadMassProportionalParameters // AUTO: 
{
public: // AUTO: 
    Index markerNumber;                           //!< AUTO: marker's number to which load is applied
    Vector3D loadVector;                          //!< AUTO: vector-valued load [SI:N/kg = m/s\f$^2\f$]; typically, this will be the gravity vector in global coordinates
    std::function<StdVector(Real,StdVector3D)> loadVectorUserFunction;//!< AUTO: A python function which defines the time-dependent loadVector.
    //! AUTO: default constructor with parameter initialization
    CLoadMassProportionalParameters()
    {
        markerNumber = EXUstd::InvalidIndex;
        loadVector = Vector3D({0.,0.,0.});
        loadVectorUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CLoadMassProportional
* @brief        Load attached to MarkerBodyMass marker, applying a 3D vector load (e.g. the vector [0,-g,0] is used to apply gravitational loading of size g in negative y-direction).
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

//! AUTO: CLoadMassProportional
class CLoadMassProportional: public CLoad // AUTO: 
{
protected: // AUTO: 
    CLoadMassProportionalParameters parameters; //! AUTO: contains all parameters for CLoadMassProportional

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CLoadMassProportionalParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CLoadMassProportionalParameters& GetParameters() const { return parameters; }

    //! AUTO:  get according marker number where load is applied
    virtual Index GetMarkerNumber() const override
    {
        return parameters.markerNumber;
    }

    //! AUTO:  set according marker number where load is applied
    virtual void SetMarkerNumber(Index markerNumberInit) override
    {
        parameters.markerNumber = markerNumberInit;
    }

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::BodyMass;
    }

    //! AUTO:  return load type
    virtual LoadType GetType() const override
    {
        return (LoadType)((Index)LoadType::ForcePerMass);
    }

    //! AUTO:  true = load is of vector type
    virtual bool IsVector() const override
    {
        return true;
    }

    //! AUTO:  read access for force vector
    virtual Vector3D GetLoadVector(Real t) const override;

};



#endif //#ifdef include once...
