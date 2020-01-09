/** ***********************************************************************************************
* @class        VisualizationNodeGenericODE2
* @brief        A node containing a number of ODE2 variables; use e.g. for scalar dynamic equations (Mass1D) or for the ALECable element.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-09-12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class VisualizationNodeGenericODE2: public VisualizationNode // AUTO: 
{
protected: // AUTO: 

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationNodeGenericODE2()
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


