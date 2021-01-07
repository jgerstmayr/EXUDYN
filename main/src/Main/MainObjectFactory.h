/** ***********************************************************************************************
* @class        MainObjectFactory
* @brief		
* @details		Details:
				- creation of objects in Exudyn
*
* @author		Gerstmayr Johannes
* @date			2019-04-19 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef MAINOBJECTFACTORY__H
#define MAINOBJECTFACTORY__H

#include "Main/MainSystemData.h"

class MainSystem;

//!This is the object factory which is extended by all objects
class MainObjectFactory
{
private:
	//*********************************************************************
	//helper functions:

	//! check whether dictionary has valid name (return true), or generate a name for certain item (node, object, marker, ...);
	//! the generated name is baseItem+string(currentNumber); in case of inconsistencies, errorFound is set to true
	bool DictHasValidName(const MainSystem& mainSystem, const py::dict& d, Index currentNumber, const STDstring& baseItem, bool& errorFound);

public:
	//*********************************************************************
	//object factory functions
	//! Create a specific node with nodeType; returns node=Null if no success
	MainNode* CreateMainNode(MainSystem& mainSystem, STDstring nodeType);
	//! Add a MainNode (and its according CNode) to the system container
	Index AddMainNode(MainSystem& mainSystem, py::dict d);

	//! Create a specific object with objectType; returns object=Null if no success
	MainObject* CreateMainObject(MainSystem& mainSystem, STDstring objectType);
	//! Add a MainObject (and its according CObject) to the system container
	Index AddMainObject(MainSystem& mainSystem, py::dict d);

	//! Create a specific marker with markerType; returns marker=Null if no success
	MainMarker* CreateMainMarker(MainSystem& mainSystem, STDstring markerType);
	//! Add a MainMarker (and its according CMarker) to the system container; return index in item list or EXUstd::InvalidIndex if failed
	Index AddMainMarker(MainSystem& mainSystem, py::dict d);

	//! Create a specific load with loadType; returns node=Null if no success
	MainLoad* CreateMainLoad(MainSystem& mainSystem, STDstring loadType);
	//! Add a MainLoad (and its according CLoad) to the system container
	Index AddMainLoad(MainSystem& mainSystem, py::dict d);

	//! Create a specific sensor with sensorType; returns sensor=Null if no success
	MainSensor* CreateMainSensor(MainSystem& mainSystem, STDstring sensorType);
	//! Add a MainSensor (and its according CSensor) to the system container; return index in item list or EXUstd::InvalidIndex if failed
	Index AddMainSensor(MainSystem& mainSystem, py::dict d);

};

#endif
