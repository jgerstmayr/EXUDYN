/** ***********************************************************************************************
* @class        VisualizationNodeGenericODE1
* @brief        A node containing a number of ABRV:ODE1 variables. Use this node e.g. for linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-09-25  09:23:36 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONNODEGENERICODE1__H
#define VISUALIZATIONNODEGENERICODE1__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationNodeGenericODE1: public VisualizationNode // AUTO:
{
protected: // AUTO:

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationNodeGenericODE1()
    {
        show = false;
    };

    // AUTO: access functions
    //! AUTO:  Empty graphics update for now
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override
    {
        ;
    }

};



#endif //#ifdef include once...
