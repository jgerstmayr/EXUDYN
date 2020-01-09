/** ***********************************************************************************************
* @class        VisualizationObjectJointSliding2D
* @brief        A specialized sliding joint (without rotation) in 2D between a Cable2D (marker1) and a position-based marker (marker0); the data coordinates provide [0] the current index in slidingMarkerNumbers, and [1] the local position in the cable element at the beginning of the timestep; the algebraic variables are \f[ \qv_{AE}=[\lambda_x\;\; \lambda_y \;\; s]^T \f] in which \f$\lambda_x\f$ and \f$\lambda_y\f$ are the Lagrange multipliers for the position of the sliding joint and \f$s\f$ is the (algebraic) sliding coordinate relative to the value at the beginning at the solution step; the data coordinates are \f[ \qv_{Data} = [i_{marker} \;\; s_{0}]^T \f] in which \f$i_{marker}\f$ is the current local index to the slidingMarkerNumber list and  \f$s_{0}\f$ is the sliding coordinate (which is the total sliding length along all cable elements in the cableMarkerNumber list) at the beginning of the solution step.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-12-30  21:32:21 (last modfied)
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

class VisualizationObjectJointSliding2D: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size = radius of revolute joint; size == -1.f means that default connector size is used
    Float4 color;                                 //!< AUTO: RGB connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointSliding2D()
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

    //! AUTO:  Write (Reference) access to:drawing size = radius of revolute joint; size == -1.f means that default connector size is used
    void SetDrawSize(const float& value) { drawSize = value; }
    //! AUTO:  Read (Reference) access to:drawing size = radius of revolute joint; size == -1.f means that default connector size is used
    const float& GetDrawSize() const { return drawSize; }
    //! AUTO:  Read (Reference) access to:drawing size = radius of revolute joint; size == -1.f means that default connector size is used
    float& GetDrawSize() { return drawSize; }

    //! AUTO:  Write (Reference) access to:RGB connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGB connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGB connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};


