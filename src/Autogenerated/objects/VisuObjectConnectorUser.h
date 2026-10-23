/** ***********************************************************************************************
* @class        VisualizationObjectConnectorUser
* @brief        A connector whose forces are computed by a Python user function with access to its markers: for each marker it reads the position, velocity, rotation and coordinates of the marker, and for a body marker the positions and velocities anywhere on the body, and it applies forces and torques through the marker - at the marker or at any local position of the marker's body. So a contact can search the closest point on a flexible body and act there. It links any number of markers of any type; its Jacobian is computed numerically.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-23  07:52:38 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONNECTORUSER__H
#define VISUALIZATIONOBJECTCONNECTORUSER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectConnectorUser: public VisualizationObject // AUTO:
{
protected: // AUTO:

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorUser()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  this function is needed to distinguish connector objects from body objects
    virtual bool IsConnector() const override
    {
        return true;
    }

};



#endif //#ifdef include once...
