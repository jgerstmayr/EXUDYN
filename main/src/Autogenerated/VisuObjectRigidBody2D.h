/** ***********************************************************************************************
* @class        VisualizationObjectRigidBody2D
* @brief        A 2D rigid body which is attached to a rigid body 2D node. Equations of motion with the displacements \f$[u_x\;\; u_y]^T\f$ of the center of mass and the rotation \f$\varphi\f$ (positive rotation around z-axis), the mass \f$m\f$, inertia around z-axis \f$J\f$ and the residual of all forces and moments \f$[R_x\;\; R_y\;\; R_\varphi]^T\f$ are given as \f[ \vr{m \ddot u_x}{m \ddot u_y}{J \varphi} = \vr{R_x}{R_y}{R_\varphi}.\f]
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-09-12 (last modfied)
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

class VisualizationObjectRigidBody2D: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    BodyGraphicsData graphicsData;                //!< AUTO: Structure contains data for body visualization; data is defined in special list / dictionary structure

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectRigidBody2D()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  Write (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    void SetGraphicsData(const BodyGraphicsData& value) { graphicsData = value; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    const BodyGraphicsData& GetGraphicsData() const { return graphicsData; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    BodyGraphicsData& GetGraphicsData() { return graphicsData; }

};


