/** ***********************************************************************************************
* @class        CLoadTorqueVectorParameters
* @brief        Parameter class for CLoadTorqueVector
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-03-01  20:14:21 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CLOADTORQUEVECTORPARAMETERS__H
#define CLOADTORQUEVECTORPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CLoadTorqueVectorParameters
class CLoadTorqueVectorParameters // AUTO: 
{
public: // AUTO: 
    Index markerNumber;                           //!< AUTO: marker's number to which load is applied
    Vector3D loadVector;                          //!< AUTO: vector-valued load [SI:N]
    bool bodyFixed;                               //!< AUTO: if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower torque; if false: global coordinates are used
    std::function<StdVector(const MainSystem&,Real,StdVector3D)> loadVectorUserFunction;//!< AUTO: A Python function which defines the time-dependent load; see description below; see also notes on loadFactor and drawing in LoadForceVector! Example for Python function: def f(mbs, t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]
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
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

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
    virtual Vector3D GetLoadVector(const MainSystemBase& mbs, Real t) const override;

    //! AUTO:  per default, forces/torques/... are applied in global coordinates; if IsBodyFixed()=true, the marker needs to provide a rotation (orientation) and forces/torques/... are applied in the local coordinate system
    virtual bool IsBodyFixed() const override
    {
        return parameters.bodyFixed;
    }

    //! AUTO:  tells system if loadFactor is used in static computation or if load is time dependent (assumed for any load user function)
    virtual bool HasUserFunction() const override
    {
        return parameters.loadVectorUserFunction != 0;
    }

};



#endif //#ifdef include once...
