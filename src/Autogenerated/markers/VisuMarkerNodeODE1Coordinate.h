/** ***********************************************************************************************
* @class        VisualizationMarkerNodeODE1Coordinate
* @brief        A node-Marker attached to a ABRV:ODE1 coordinate of a node.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-04  17:26:47 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONMARKERNODEODE1COORDINATE__H
#define VISUALIZATIONMARKERNODEODE1COORDINATE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationMarkerNodeODE1Coordinate: public VisualizationMarker // AUTO:
{
protected: // AUTO:

public: // AUTO:

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override
    {
        ;
    }

};



#endif //#ifdef include once...
