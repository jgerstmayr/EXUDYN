/** ***********************************************************************************************
* @class        VisualizationObjectContactPrimitiveConvex
* @brief        A penalty contact between a primitive - a sphere, capsule, cylinder or truncated cone at marker 0 - and a convex polytope, given by its points and the triangles of its surface in the frame of marker 1; up to 4 contact points carry a line or a face of contact. Contact law and friction as for ObjectContactSphereSphere.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-08  17:11:13 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONTACTPRIMITIVECONVEX__H
#define VISUALIZATIONOBJECTCONTACTPRIMITIVECONVEX__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectContactPrimitiveConvex: public VisualizationObject // AUTO:
{
protected: // AUTO:
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectContactPrimitiveConvex()
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
