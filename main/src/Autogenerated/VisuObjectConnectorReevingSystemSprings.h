/** ***********************************************************************************************
* @class        VisualizationObjectConnectorReevingSystemSprings
* @brief        A rD reeving system defined by a list of torque-free and friction-free sheaves or points that are connected with one rope (modelled as massless spring). NOTE that the spring can undergo tension AND compression (in order to avoid compression, use a PreStepUserFunction to turn off stiffness and damping in this case!). The force is assumed to be constant all over the rope. The sheaves or connection points are defined by \f$nr\f$ rigid body markers \f$[m_0, \, m_1, \, \ldots, \, m_{nr-1}]\f$. At both ends of the rope there may be a prescribed motion coupled to a coordinate marker each, given by \f$m_{c0}\f$ and \f$m_{c1}\f$ .
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-03  15:27:07 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONNECTORREEVINGSYSTEMSPRINGS__H
#define VISUALIZATIONOBJECTCONNECTORREEVINGSYSTEMSPRINGS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectConnectorReevingSystemSprings: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float ropeRadius;                             //!< AUTO: radius of rope
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorReevingSystemSprings()
    {
        show = true;
        ropeRadius = 0.001f;
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

    //! AUTO:  Write (Reference) access to:radius of rope
    void SetRopeRadius(const float& value) { ropeRadius = value; }
    //! AUTO:  Read (Reference) access to:radius of rope
    const float& GetRopeRadius() const { return ropeRadius; }
    //! AUTO:  Read (Reference) access to:radius of rope
    float& GetRopeRadius() { return ropeRadius; }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
