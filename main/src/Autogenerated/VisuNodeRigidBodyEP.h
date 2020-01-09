/** ***********************************************************************************************
* @class        VisualizationNodeRigidBodyEP
* @brief        A 3D rigid body node based on Euler parameters for rigid bodies or beams; the node has 3 displacement coordinates (displacements of center of mass - COM: ux,uy,uz) and four rotation coordinates (Euler parameters = quaternions); all coordinates lead to second order differential equations; additionally there is one constraint equation for quaternions; The rotation matrix \f$\Am\f$, transforming local (body-fixed) 3D positions \f$\pv_{loc} = [p^x_{loc}\;\;p^y_{loc}\;\;p^z_{loc}]^T\f$ to global 3D positions \f$\pv_{glob} = [p^x_{glob}\;\;p^y_{glob}\;\;p^z_{glob}]^T\f$, \f[ \pv_{glob} = \Am \pv_{loc}, \f] is defined according to the book of Shabana, same with the transformation matrix \f$\mathbf{G}\f$ between time derivatives of Euler parameters and angular velocities.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-10-21  08:24:32 (last modfied)
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

class VisualizationNodeRigidBodyEP: public VisualizationNode // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
    Float4 color;                                 //!< AUTO: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationNodeRigidBodyEP()
    {
        show = true;
        drawSize = -1.f;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Write (Reference) access to:drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
    void SetDrawSize(const float& value) { drawSize = value; }
    //! AUTO:  Read (Reference) access to:drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
    const float& GetDrawSize() const { return drawSize; }
    //! AUTO:  Read (Reference) access to:drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
    float& GetDrawSize() { return drawSize; }

    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  Write (Reference) access to:Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used
    Float4& GetColor() { return color; }

};


