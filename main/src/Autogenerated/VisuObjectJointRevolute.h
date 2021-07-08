/** ***********************************************************************************************
* @class        VisualizationObjectJointRevolute
* @brief        A revolute joint in 3D; constrains the position of two rigid body markers and the rotation about two axes, while one common rotation axis can freely rotate. An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddRevoluteJoint(...) for two rigid bodies (or ground).
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-07-01  09:35:50 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTJOINTREVOLUTE__H
#define VISUALIZATIONOBJECTJOINTREVOLUTE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectJointRevolute: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float axesRadius;                             //!< AUTO: radius of joint axes to draw
    float axesLength;                             //!< AUTO: length of joint axes to draw
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointRevolute()
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



#endif //#ifdef include once...
