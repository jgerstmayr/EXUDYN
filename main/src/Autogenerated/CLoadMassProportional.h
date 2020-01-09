/** ***********************************************************************************************
* @class        CLoadMassProportionalParameters
* @brief        Parameter class for CLoadMassProportional
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-12-09  22:32:12 (last modfied)
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

#include <functional> //! AUTO: needed for std::function

//! AUTO: Parameters for class CLoadMassProportionalParameters
class CLoadMassProportionalParameters // AUTO: 
{
public: // AUTO: 
    Index markerNumber;                           //!< AUTO: marker"s number to which load is applied
    Vector3D loadVector;                          //!< AUTO: vector-valued load [SI:N/kg = m/s\f$^2\f$] 
    std::function<StdVector3D(Real,StdVector3D)> loadVectorUserFunction;//!< AUTO: A python function which defines the time-dependent load with parameters (Real t, Vector3D load); the load represents the current value of the load; WARNING: this factor does not work in combination with static computation (loadFactor); Example for python function: def f(t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]
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
* @brief        Load attached to BodyMass-based marker, applying a 3D vector load (e.g. the vector [0,-g,0] is used to apply gravitational loading of size g in negative y-direction).
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


