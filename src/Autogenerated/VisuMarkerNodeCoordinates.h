/** ***********************************************************************************************
* @class        VisualizationMarkerNodeCoordinates
* @brief        A node-Marker attached to all ODE2 coordinates of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for ODE1 coordinates use MarkerNodeODE1Coordinates (under development).
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-06-27  17:08:55 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONMARKERNODECOORDINATES__H
#define VISUALIZATIONMARKERNODECOORDINATES__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationMarkerNodeCoordinates: public VisualizationMarker // AUTO: 
{
protected: // AUTO: 

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerNodeCoordinates()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override
    {
        ;
    }

};



#endif //#ifdef include once...
