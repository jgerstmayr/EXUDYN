/** ***********************************************************************************************
* @class        VisualizationObjectJointGeneric
* @brief        A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers; an additional local rotation can be used to define three rotation axes and/or sliding axes
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-01  20:10:12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class VisualizationObjectJointGeneric: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float axesRadius;                             //!< AUTO: radius of joint axes to draw
    float axesLength;                             //!< AUTO: length of joint axes to draw
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointGeneric()
    {
        show = true;
        axesRadius = 0.1f;
        axesLength = 0.4f;
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

    //! AUTO:  Write (Reference) access to:radius of joint axes to draw
    void SetAxesRadius(const float& value) { axesRadius = value; }
    //! AUTO:  Read (Reference) access to:radius of joint axes to draw
    const float& GetAxesRadius() const { return axesRadius; }
    //! AUTO:  Read (Reference) access to:radius of joint axes to draw
    float& GetAxesRadius() { return axesRadius; }

    //! AUTO:  Write (Reference) access to:length of joint axes to draw
    void SetAxesLength(const float& value) { axesLength = value; }
    //! AUTO:  Read (Reference) access to:length of joint axes to draw
    const float& GetAxesLength() const { return axesLength; }
    //! AUTO:  Read (Reference) access to:length of joint axes to draw
    float& GetAxesLength() { return axesLength; }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};


