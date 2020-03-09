/** ***********************************************************************************************
* @class        VisualizationObjectGenericODE2
* @brief        A system of second order ordinary differential equations (ODE2), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces. It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-03-04  18:24:39 (last modfied)
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

class VisualizationObjectGenericODE2: public VisualizationObject // AUTO: 
{
protected: // AUTO: 

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectGenericODE2()
    {
        show = true;
    };

    // AUTO: access functions
};


