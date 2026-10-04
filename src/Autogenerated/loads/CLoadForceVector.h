/** ***********************************************************************************************
* @class        CLoadForceVectorParameters
* @brief        Parameter class for CLoadForceVector
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-04  17:30:16 (last modified)
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
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CLoadForceVectorParameters
class CLoadForceVectorParameters // AUTO:
{
public: // AUTO:
    Index markerNumber;                           //!< AUTO: marker's number to which load is applied
    Vector3D loadVector;                          //!< AUTO: vector-valued load [SI:N]; in case of a user function, this vector is ignored
    bool bodyFixed;                               //!< AUTO: if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower force; if false: global coordinates are used
    PythonUserFunctionBase< std::function<StdVector3D(const MainSystem&,Real,StdVector3D)> > loadVectorUserFunction;//!< AUTO: A Python function which defines the time-dependent load and replaces loadVector; see description below; NOTE that in static computations, the loadFactor is always 1 for forces computed by user functions (this means for the static computation, that a user function returning [t*5,t*1,0] corresponds to loadVector=[5,1,0] without a user function); the render window draws the load with the value of the user function if visualizationSettings.loads.drawWithUserFunction is True and the user function is symbolic, or a Python function with visualizationSettings.general.useMultiThreadedRendering = False - the render thread cannot call Python -, otherwise with loadVector; a sensor (SensorLoad) returns the force of the user function in any case
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

    //! AUTO:  read access for load vector; returns user function result in case it is defined
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
