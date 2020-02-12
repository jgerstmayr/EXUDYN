/** ***********************************************************************************************
* @class        VisualizationObjectConnectorRigidBodySpringDamper
* @brief        An 3D spring-damper element acting on relative displacements and relative rotations of two rigid body (position+orientation) markers; connects to (position+orientation)-based markers; represents a penalty-based rigid joint; the resulting force in the spring-damper reads (\f$m0 = marker[0]\f$ and \f$m1 = marker[1]\f$): \f[ force_x = (A0loc \cdot A0) \cdot stiffness_x \cdot (A0loc \cdot A0)^T(m1.position_x - m0.position_x - offset_x) + (A0loc \cdot A0) \cdot damping_x \cdot (A0loc \cdot A0)^T (m1.velocity_x - m0.velocity_x), etc. \f] and accordingly for rotation coordinates, which act on \f$(rotationMarker0 \cdot Rxyz0)^T \cdot (rotationMarker1 \cdot Rxyz1) \f$ rotations (0...rotation of marker0, 1...rotation of marker1).
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-02-09  17:52:25 (last modfied)
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

class VisualizationObjectConnectorRigidBodySpringDamper: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size = diameter of spring; size == -1.f means that default connector size is used
    Float4 color;                                 //!< AUTO: RGB connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorRigidBodySpringDamper()
    {
        show = true;
        drawSize = -1.f;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  this function is needed to distinguish connector objects from body objects
    virtual bool IsConnector() const override
    {
        return true;
    }

    //! AUTO:  Write (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    void SetDrawSize(const float& value) { drawSize = value; }
    //! AUTO:  Read (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    const float& GetDrawSize() const { return drawSize; }
    //! AUTO:  Read (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    float& GetDrawSize() { return drawSize; }

    //! AUTO:  Write (Reference) access to:RGB connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGB connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGB connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};


