/** ***********************************************************************************************
* @class        VisualizationObjectRigidBody
* @brief        A 3D rigid body which is attached to a 3D rigid body node. The rotation parametrization of the rigid body follows the rotation parametrization of the node. Use Euler parameters in the general case (no singularities) in combination with implicit solvers (GeneralizedAlpha or TrapezoidalIndex2), Tait-Bryan angles for special cases, e.g., rotors where no singularities occur if you rotate about \f$x\f$ or \f$z\f$ axis, or use Lie-group formulation with rotation vector together with explicit solvers. REMARK: Use the class \texttt{RigidBodyInertia}, see \refSection{sec:rigidBodyUtilities:RigidBodyInertia:__init__} and \texttt{AddRigidBody(...)}, see \refSection{sec:rigidBodyUtilities:AddRigidBody}, of \texttt{exudyn.rigidBodyUtilities} to handle inertia, COM and mass.
 \addExampleImage{ObjectRigidBody}
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-07-08  15:40:50 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTRIGIDBODY__H
#define VISUALIZATIONOBJECTRIGIDBODY__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectRigidBody: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    std::function<py::object(const MainSystem&, Index)> graphicsDataUserFunction;//!< AUTO: A python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
    BodyGraphicsData graphicsData;                //!< AUTO: Structure contains data for body visualization; data is defined in special list / dictionary structure

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectRigidBody()
    {
        show = true;
        graphicsDataUserFunction = 0;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  user function which is called to update specific object graphics computed in python functions; this is rather slow, but useful for user elements
    virtual void CallUserFunction(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, const MainSystem& mainSystem, Index itemNumber) override;

    //! AUTO:  return true, if object has a user function to be called during redraw
    virtual bool HasUserFunction() override
    {
        return graphicsDataUserFunction!=0;
    }

    //! AUTO:  Write (Reference) access to:A python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
    void SetGraphicsDataUserFunction(const std::function<py::object(const MainSystem&, Index)>& value) { graphicsDataUserFunction = value; }
    //! AUTO:  Read (Reference) access to:A python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
    const std::function<py::object(const MainSystem&, Index)>& GetGraphicsDataUserFunction() const { return graphicsDataUserFunction; }
    //! AUTO:  Read (Reference) access to:A python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
    std::function<py::object(const MainSystem&, Index)>& GetGraphicsDataUserFunction() { return graphicsDataUserFunction; }

    //! AUTO:  Write (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    void SetGraphicsData(const BodyGraphicsData& value) { graphicsData = value; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    const BodyGraphicsData& GetGraphicsData() const { return graphicsData; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    BodyGraphicsData& GetGraphicsData() { return graphicsData; }

};



#endif //#ifdef include once...
