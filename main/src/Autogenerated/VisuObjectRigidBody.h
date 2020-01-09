/** ***********************************************************************************************
* @class        VisualizationObjectRigidBody
* @brief        A 3D rigid body which is attached to a 3D rigid body node. Equations of motion with the displacements \f$[u_x\;\; u_y\;\; u_z]^T\f$ of the center of mass and the rotation parameters (Euler parameters) \f$\mathbf{q}\f$, the mass \f$m\f$, inertia \f$\mathbf{J} = [J_{xx}, J_{xy}, J_{xz}; J_{yx}, J_{yy}, J_{yz}; J_{zx}, J_{zy}, J_{zz}]\f$ and the residual of all forces and moments \f$[R_x\;\; R_y\;\; R_z\;\; R_{q0}\;\; R_{q1}\;\; R_{q2}\;\; R_{q3}]^T\f$ are given as ...
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

class VisualizationObjectRigidBody: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    BodyGraphicsData graphicsData;                //!< AUTO: Structure contains data for body visualization; data is defined in special list / dictionary structure

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectRigidBody()
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


