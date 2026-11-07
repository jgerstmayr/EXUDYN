/** ***********************************************************************************************
* @class        VisualizationObjectContactConeCone
* @brief        A penalty contact between two truncated cones, each centered at its marker along the local z axis of the marker, with their sides, caps and rims (a radius 0 gives a pointed cone, equal radii a cylinder); up to 4 contact points carry a line or a face of contact. Contact law and friction as for ObjectContactSphereTriangle.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-07  16:02:06 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONTACTCONECONE__H
#define VISUALIZATIONOBJECTCONTACTCONECONE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectContactConeCone: public VisualizationObject // AUTO:
{
protected: // AUTO:
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectContactConeCone()
    {
        show = false;
        color = Float4({0.7f,0.7f,0.7f,1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  this function is needed to distinguish connector objects from body objects
    virtual bool IsConnector() const override
    {
        return true;
    }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
