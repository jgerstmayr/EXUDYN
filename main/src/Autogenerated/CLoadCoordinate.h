/** ***********************************************************************************************
* @class        CLoadCoordinateParameters
* @brief        Parameter class for CLoadCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-12-01  16:28:59 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CLOADCOORDINATEPARAMETERS__H
#define CLOADCOORDINATEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CLoadCoordinateParameters
class CLoadCoordinateParameters // AUTO: 
{
public: // AUTO: 
    Index markerNumber;                           //!< AUTO: marker's number to which load is applied
    Real load;                                    //!< AUTO: scalar load [SI:N]; in case of a user function, this value is ignored
    std::function<Real(const MainSystem&,Real,Real)> loadUserFunction;//!< AUTO: A Python function which defines the time-dependent load and replaces the load; see description below; see also notes on loadFactor and drawing in LoadForceVector!
    //! AUTO: default constructor with parameter initialization
    CLoadCoordinateParameters()
    {
        markerNumber = EXUstd::InvalidIndex;
        load = 0.;
        loadUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CLoadCoordinate
* @brief        Load with scalar value, which is attached to a coordinate-based marker; the load can be used e.g. to apply a force to a single axis of a body, a nodal coordinate of a finite element  or a torque to the rotatory DOF of a rigid body.
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

//! AUTO: CLoadCoordinate
class CLoadCoordinate: public CLoad // AUTO: 
{
protected: // AUTO: 
    CLoadCoordinateParameters parameters; //! AUTO: contains all parameters for CLoadCoordinate

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CLoadCoordinateParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CLoadCoordinateParameters& GetParameters() const { return parameters; }

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
        return Marker::Coordinate;
    }

    //! AUTO:  return load type
    virtual LoadType GetType() const override
    {
        return (LoadType)((Index)LoadType::Coordinate);
    }

    //! AUTO:  true = load is of vector type
    virtual bool IsVector() const override
    {
        return false;
    }

    //! AUTO:  tells system if loadFactor is used in static computation or if load is time dependent (assumed for any load user function)
    virtual bool HasUserFunction() const override
    {
        return parameters.loadUserFunction != 0;
    }

    //! AUTO:  read access for load value (IsVector=false)
    virtual Real GetLoadValue(const MainSystemBase& mbs, Real t) const override;

};



#endif //#ifdef include once...
