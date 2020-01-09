/** ***********************************************************************************************
* @class	    BasicObject
* @brief		Basic object for Exudyn
* @details		Details:
 				- ...
*
* @author		Gerstmayr Johannes
* @date			2018-05-15 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#pragma once

#include "System/CObject.h"

class ObjectManager //all managed objects (Nodes, Objects, Sensors, ...) should be derived from one class
{
protected:
    CObject* computationalObject;

public:
    ObjectManager(CObject* computationalObjectInit)
    {
        computationalObject = computationalObjectInit;
    }



};



