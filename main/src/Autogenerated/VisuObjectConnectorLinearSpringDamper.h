/** ***********************************************************************************************
* @class        VisualizationObjectConnectorLinearSpringDamper
* @brief        An linear spring-damper element acting on relative translations along given axis of local joint0 coordinate system; connects to position and orientation-based markers; the linear spring-damper is intended to act within prismatic joints or in situations where only one translational axis is free; if the two markers rotate relative to each other, the spring-damper will always act in the local joint0 coordinate system.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-06-01  23:25:08 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONNECTORLINEARSPRINGDAMPER__H
#define VISUALIZATIONOBJECTCONNECTORLINEARSPRINGDAMPER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectConnectorLinearSpringDamper: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size = diameter of spring; size == -1.f means that default connector size is used
    bool drawAsCylinder;                          //!< AUTO: if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorLinearSpringDamper()
    {
        show = true;
        drawSize = -1.f;
        drawAsCylinder = false;
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

    //! AUTO:  Write (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    void SetDrawSize(const float& value) { drawSize = value; }
    //! AUTO:  Read (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    const float& GetDrawSize() const { return drawSize; }
    //! AUTO:  Read (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    float& GetDrawSize() { return drawSize; }

    //! AUTO:  Write (Reference) access to:if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator
    void SetDrawAsCylinder(const bool& value) { drawAsCylinder = value; }
    //! AUTO:  Read (Reference) access to:if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator
    const bool& GetDrawAsCylinder() const { return drawAsCylinder; }
    //! AUTO:  Read (Reference) access to:if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator
    bool& GetDrawAsCylinder() { return drawAsCylinder; }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
