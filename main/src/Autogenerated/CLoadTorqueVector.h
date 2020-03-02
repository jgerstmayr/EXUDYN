/** ***********************************************************************************************
* @class        CLoadTorqueVectorParameters
* @brief        Parameter class for CLoadTorqueVector
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-02-19  19:12:08 (last modfied)
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

//! AUTO: Parameters for class CLoadTorqueVectorParameters
class CLoadTorqueVectorParameters // AUTO: 
{
public: // AUTO: 
    Index markerNumber;                           //!< AUTO: marker"s number to which load is applied
    Vector3D loadVector;                          //!< AUTO: vector-valued load [SI:N]
    bool bodyFixed;                               //!< AUTO: if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower torque; if false: global coordinates are used
    std::function<StdVector3D(Real,StdVector3D)> loadVectorUserFunction;//!< AUTO: A python function which defines the time-dependent load with parameters (Real t, Vector3D load); the load represents the current value of the load; WARNING: this factor does not work in combination with static computation (loadFactor); Example for python function: def f(t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]
    //! AUTO: default constructor with parameter initialization
    CLoadTorqueVectorParameters()
    {
        markerNumber = EXUstd::InvalidIndex;
        loadVector = Vector3D({0.,0.,0.});
        bodyFixed = false;
        loadVectorUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CLoadTorqueVector
* @brief        Load with (3D) torque vector; attached to rigidbody-based marker.
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

//! AUTO: CLoadTorqueVector
class CLoadTorqueVector: public CLoad // AUTO: 
{
protected: // AUTO: 
    CLoadTorqueVectorParameters parameters; //! AUTO: contains all parameters for CLoadTorqueVector

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CLoadTorqueVectorParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CLoadTorqueVectorParameters& GetParameters() const { return parameters; }

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
        return Marker::Orientation;
    }

    //! AUTO:  return load type
    virtual LoadType GetType() const override
    {
        return (LoadType)((Index)LoadType::Torque);
    }

    //! AUTO:  true = load is of vector type
    virtual bool IsVector() const override
    {
        return true;
    }

    //! AUTO:  read access for load vector
    virtual Vector3D GetLoadVector(Real t) const override;

    //! AUTO:  per default, forces/torques/... are applied in global coordinates; if IsBodyFixed()=true, the marker needs to provide a rotation (orientation) and forces/torques/... are applied in the local coordinate system
    virtual bool IsBodyFixed() const override
    {
        return parameters.bodyFixed;
    }

};


