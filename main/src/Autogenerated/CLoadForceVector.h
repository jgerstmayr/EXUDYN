/** ***********************************************************************************************
* @class        CLoadForceVectorParameters
* @brief        Parameter class for CLoadForceVector
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-03-20  12:03:09 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CLOADFORCEVECTORPARAMETERS__H
#define CLOADFORCEVECTORPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CLoadForceVectorParameters
class CLoadForceVectorParameters // AUTO: 
{
public: // AUTO: 
    Index markerNumber;                           //!< AUTO: marker's number to which load is applied
    Vector3D loadVector;                          //!< AUTO: vector-valued load [SI:N]
    bool bodyFixed;                               //!< AUTO: if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower force; if false: global coordinates are used
    std::function<StdVector(const MainSystem&,Real,StdVector3D)> loadVectorUserFunction;//!< AUTO: A python function which defines the time-dependent load
    //! AUTO: default constructor with parameter initialization
    CLoadForceVectorParameters()
    {
        markerNumber = EXUstd::InvalidIndex;
        loadVector = Vector3D({0.,0.,0.});
        bodyFixed = false;
        loadVectorUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CLoadForceVector
* @brief        Load with (3D) force vector; attached to position-based marker.
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

//! AUTO: CLoadForceVector
class CLoadForceVector: public CLoad // AUTO: 
{
protected: // AUTO: 
    CLoadForceVectorParameters parameters; //! AUTO: contains all parameters for CLoadForceVector

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CLoadForceVectorParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CLoadForceVectorParameters& GetParameters() const { return parameters; }

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
        return Marker::Position;
    }

    //! AUTO:  return force type
    virtual LoadType GetType() const override
    {
        return (LoadType)((Index)LoadType::Force);
    }

    //! AUTO:  true = load is of vector type
    virtual bool IsVector() const override
    {
        return true;
    }

    //! AUTO:  read access for force vector; returns user function result in case it is defined
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
