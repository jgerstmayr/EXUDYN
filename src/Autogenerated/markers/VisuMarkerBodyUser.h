/** ***********************************************************************************************
* @class        VisualizationMarkerBodyUser
* @brief        A marker on any number of bodies whose position, and with providedMarkerType Position | Orientation its rotation, are computed by Python user functions from the coordinates of the nodes of these bodies, e.g. a point between several bodies or nodes, or a point of a mesh. Loads, connectors, joints and sensors use it as a position or rigid marker; a connector or joint on it needs its Jacobian functions (#3107).
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-04  00:02:40 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONMARKERBODYUSER__H
#define VISUALIZATIONMARKERBODYUSER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationMarkerBodyUser: public VisualizationMarker // AUTO:
{
protected: // AUTO:

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerBodyUser()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};



#endif //#ifdef include once...
