/** ***********************************************************************************************
* @class        VisualizationObjectGenericODE1
* @brief        A system of \f$n\f$ second order ordinary differential equations (ODE1), having a system matrix, a rhs vector, but mostly it will use a user function to describe special ODE1 systems. It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions \f$n\f$ or \f$(n \times n)\f$, or they must be empty \f$(0 \times 0)\f$, using [] in python.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-01-20  18:14:26 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTGENERICODE1__H
#define VISUALIZATIONOBJECTGENERICODE1__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectGenericODE1: public VisualizationObject // AUTO: 
{
protected: // AUTO: 

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectGenericODE1()
    {
        show = true;
    };

    // AUTO: access functions
};



#endif //#ifdef include once...
