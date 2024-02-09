/** ***********************************************************************************************
* @class        VisualizationNodeRigidBody2D
* @brief        A 2D rigid body node for rigid bodies or beams; the node has 2 displacement degrees of freedom and one rotation coordinate (rotation around z-axis: uphi). All coordinates are \hac{ODE2}, used for second order differetial equations.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-03  15:27:05 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONNODERIGIDBODY2D__H
#define VISUALIZATIONNODERIGIDBODY2D__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationNodeRigidBody2D: public VisualizationNode // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
    Float4 color;                                 //!< AUTO: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationNodeRigidBody2D()
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



#endif //#ifdef include once...
