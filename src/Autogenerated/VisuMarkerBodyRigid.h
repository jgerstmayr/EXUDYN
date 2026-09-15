/** ***********************************************************************************************
* @class        VisualizationMarkerBodyRigid
* @brief        A rigid-body (position+orientation) body-marker attached to a local (body-fixed) position \f$\pLocB = [b_0,\; b_1,\; b_2]\f$ (\f$x\f$, \f$y\f$, and \f$z\f$ coordinates) of the body. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.
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

#ifndef VISUALIZATIONMARKERBODYRIGID__H
#define VISUALIZATIONMARKERBODYRIGID__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationMarkerBodyRigid: public VisualizationMarker // AUTO:
{
protected: // AUTO:

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerBodyRigid()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};



#endif //#ifdef include once...
