/** ***********************************************************************************************
* @class        VisualizationNodeGenericODE2
* @brief        A node containing a number of ABRV:ODE2 variables. Use this node e.g. for scalar dynamic equations (Mass1D), for ObjectGenericODE2 or for the Eulerian coordinate in the ALECable element. NOTE: referenceCoordinates and all initialCoordinates(_t) must be initialized, because no default values exist.
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

#ifndef VISUALIZATIONNODEGENERICODE2__H
#define VISUALIZATIONNODEGENERICODE2__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationNodeGenericODE2: public VisualizationNode // AUTO:
{
protected: // AUTO:

public: // AUTO:

    // AUTO: access functions
    //! AUTO:  Empty graphics update for now
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override
    {
        ;
    }

};



#endif //#ifdef include once...
