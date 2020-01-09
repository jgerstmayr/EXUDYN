/** ***********************************************************************************************
* @class        VisualizationObjectJointALEMoving2D
* @brief        A specialized axially moving joint (without rotation) in 2D between a ALE Cable2D (marker1) and a position-based marker (marker0); the data coordinate [0] provides the current index in slidingMarkerNumbers, and the ODE2 coordinate [0] provides the (given) moving coordinate in the cable element; the algebraic variables are \f[ \qv_{AE}=[\lambda_x\;\; \lambda_y]^T \f], in which \f$\lambda_x\f$ and \f$\lambda_y\f$ are the Lagrange multipliers for the position constraint of the moving joint; the data coordinate is \f[ \qv_{Data} = [i_{marker}]^T \f] in which \f$i_{marker}\f$ is the current local index to the slidingMarkerNumber list.
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

class VisualizationObjectJointALEMoving2D: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size = radius of revolute joint; size == -1.f means that default connector size is used
    Float4 color;                                 //!< AUTO: RGB connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointALEMoving2D()
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


