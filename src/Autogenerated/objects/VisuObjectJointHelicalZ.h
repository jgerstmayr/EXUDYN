/** ***********************************************************************************************
* @class        VisualizationObjectJointHelicalZ
* @brief        A helical (screw) joint in 3D: the frame of marker 1 moves on a screw motion about the \f$z\f$-axis of marker 0, a rotation by the angle \f$\varphi\f$ together with the translation \f$h\varphi\f$ along the axis, with the pitch \f$h\f$ per radian. The joint coordinate system is the frame of the markers; a rotation of it is given to the markers as their localHT. Each of the five constraints can be switched off by `constrainedAxes`; with the pitch zero it is the revolute joint. For easier definition of the joint, use `mbs.CreateHelicalJoint(...)` for two rigid bodies (or ground).
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-11  08:46:20 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTJOINTHELICALZ__H
#define VISUALIZATIONOBJECTJOINTHELICALZ__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectJointHelicalZ: public VisualizationObject // AUTO:
{
protected: // AUTO:
    float axisRadius;                             //!< AUTO: radius of joint axis to draw
    float axisLength;                             //!< AUTO: length of joint axis to draw
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointHelicalZ()
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
