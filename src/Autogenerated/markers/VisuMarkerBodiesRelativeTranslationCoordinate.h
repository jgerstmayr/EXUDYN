/** ***********************************************************************************************
* @class        VisualizationMarkerBodiesRelativeTranslationCoordinate
* @brief        A coordinate-based Marker attached to two rigid bodies or beams which computes the relative translation between the bodies according to the given axis. This marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). NOTE: it is assumed that the two bodies can only move along the given axis (e.g., constrained by a prismatic joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-09-15  11:13:32 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONMARKERBODIESRELATIVETRANSLATIONCOORDINATE__H
#define VISUALIZATIONMARKERBODIESRELATIVETRANSLATIONCOORDINATE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationMarkerBodiesRelativeTranslationCoordinate: public VisualizationMarker // AUTO:
{
protected: // AUTO:

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerBodiesRelativeTranslationCoordinate()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};



#endif //#ifdef include once...
