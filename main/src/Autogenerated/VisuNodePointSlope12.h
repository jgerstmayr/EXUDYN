/** ***********************************************************************************************
* @class        VisualizationNodePointSlope12
* @brief        A 3D point/slope vector node for thin ANCF (absolute nodal coordinate formulation) plate elements; the node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 \f$\times\f$ 3 for the slope vectors 'slopeX' and 'slopeY'); all coordinates lead to second order differential equations; the slopeX vector defines the directional derivative w.r.t the local axial (x) coordinate, etc.; in straight configuration aligned at the global x-axis, the slopeY vector reads \f$\rv_y^\prime=[0\;\;1\;\;0]^T\f$.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2023-10-15  22:27:18 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONNODEPOINTSLOPE12__H
#define VISUALIZATIONNODEPOINTSLOPE12__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationNodePointSlope12: public VisualizationNode // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
    Float4 color;                                 //!< AUTO: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationNodePointSlope12()
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
