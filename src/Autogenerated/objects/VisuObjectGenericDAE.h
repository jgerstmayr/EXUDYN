/** ***********************************************************************************************
* @class        VisualizationObjectGenericDAE
* @brief        A system of ABRV:ODE2, ABRV:ODE1 and algebraic equations on the coordinates of its nodes: ABRV:ODE2 nodes, NodeGenericODE1 and NodeGenericAE, whose coordinates are the Lagrange multipliers \f$\tlambda\f$ of its algebraic equations. The linear terms are given by matrices, the nonlinear ones by user functions. NOTE that every matrix or vector must have the dimensions of the coordinates it multiplies, or be empty, using [] in Python.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-29  23:55:05 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTGENERICDAE__H
#define VISUALIZATIONOBJECTGENERICDAE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectGenericDAE: public VisualizationObject // AUTO:
{
protected: // AUTO:

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectGenericDAE()
    {
        show = true;
    };

    // AUTO: access functions
};



#endif //#ifdef include once...
