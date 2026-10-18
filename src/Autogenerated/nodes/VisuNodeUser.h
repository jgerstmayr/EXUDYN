/** ***********************************************************************************************
* @class        VisualizationNodeUser
* @brief        A node with any number of ABRV:ODE2 coordinates, whose position, and optionally orientation, are computed from its coordinates by Python user functions: a node for elements whose nodes are not in Exudyn, such as an ANCF node with full gradients. The node markers (MarkerNodePosition, MarkerNodeRigid, MarkerNodeCoordinate), SensorNode, loads on these markers and every connector work on it.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-18  23:25:52 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONNODEUSER__H
#define VISUALIZATIONNODEUSER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationNodeUser: public VisualizationNode // AUTO:
{
protected: // AUTO:
    float drawSize;                               //!< AUTO: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
    Float4 color;                                 //!< AUTO: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationNodeUser()
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

    //! AUTO:  Write (Reference) access to:Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used
    Float4& GetColor() { return color; }

    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};



#endif //#ifdef include once...
