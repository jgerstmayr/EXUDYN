/** ***********************************************************************************************
* @class        VisualizationMarkerBodyCable2DCoordinates
* @brief        A special Marker attached to the coordinates of a 2D ANCF beam finite element with cubic interpolation.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-01  20:10:12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class VisualizationMarkerBodyCable2DCoordinates: public VisualizationMarker // AUTO: 
{
protected: // AUTO: 

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerBodyCable2DCoordinates()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};


