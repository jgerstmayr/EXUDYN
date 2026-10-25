/** ***********************************************************************************************
* @class        VisualizationObjectJointUser
* @brief        A joint on two rigid markers whose constraint equations are a Python user function of the positions and rotation matrices of the markers. It has 1 to 6 equations, e.g. for a screw joint, a coupling of rotation and translation or a joint whose axes depend on the motion; the reaction forces and torques follow from the Lagrange multipliers, and the derivative of the equations is a second user function.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-25  23:42:47 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTJOINTUSER__H
#define VISUALIZATIONOBJECTJOINTUSER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectJointUser: public VisualizationObject // AUTO:
{
protected: // AUTO:

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointUser()
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
