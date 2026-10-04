/** ***********************************************************************************************
* @brief		Types of the kinematic tree (ObjectKinematicTree): the placements of its links as homogeneous transformations
*
* @author		Gerstmayr Johannes
* @date			2022-04-18 (generated)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
*
************************************************************************************************ */
#ifndef KINEMATICSBASICS__H
#define KINEMATICSBASICS__H

#include "Linalg/RigidBodyMath.h"


//! the kinematic tree computes on the placements of its links, homogeneous transformations; the name is that of
//! Featherstone's spatial transformations, which the tree used before (#2828, #2829); the algebra of motions, forces
//! and inertias is in CObjectKinematicTree.cpp (namespace KinematicTreeHT)
typedef HomogeneousTransformation Transformation66;
typedef ResizableArray<Transformation66> Transformation66List;


#endif
