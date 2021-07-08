/** ***********************************************************************************************
* @class        VisualizationObjectJointRevoluteZ
* @brief        A revolute joint in 3D; constrains the position of two rigid body markers and the rotation about two axes, while the joint \f$z\f$-rotation axis can freely rotate. An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddRevoluteJoint(...), \refSection{sec:rigidBodyUtilities:AddRevoluteJoint}, for two rigid bodies (or ground).
 \addExampleImage{RevoluteJointZ}
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-07-08  15:40:51 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTJOINTREVOLUTEZ__H
#define VISUALIZATIONOBJECTJOINTREVOLUTEZ__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectJointRevoluteZ: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float axisRadius;                             //!< AUTO: radius of joint axis to draw
    float axisLength;                             //!< AUTO: length of joint axis to draw
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointRevoluteZ()
    {
        show = true;
        axisRadius = 0.1f;
        axisLength = 0.4f;
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

    //! AUTO:  Write (Reference) access to:radius of joint axis to draw
    void SetAxisRadius(const float& value) { axisRadius = value; }
    //! AUTO:  Read (Reference) access to:radius of joint axis to draw
    const float& GetAxisRadius() const { return axisRadius; }
    //! AUTO:  Read (Reference) access to:radius of joint axis to draw
    float& GetAxisRadius() { return axisRadius; }

    //! AUTO:  Write (Reference) access to:length of joint axis to draw
    void SetAxisLength(const float& value) { axisLength = value; }
    //! AUTO:  Read (Reference) access to:length of joint axis to draw
    const float& GetAxisLength() const { return axisLength; }
    //! AUTO:  Read (Reference) access to:length of joint axis to draw
    float& GetAxisLength() { return axisLength; }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
