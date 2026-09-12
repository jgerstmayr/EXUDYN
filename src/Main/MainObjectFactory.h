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
	//! Get available items in object factory; returns a map, which can be implicitly converted to a py::dict
	py::dict GetAvailableFactoryItems();

	//*********************************************************************
	//object factory functions
	//! Create a specific node with nodeType; returns node=Null if no success
	MainNode* CreateMainNode(MainSystem& mainSystem, STDstring nodeType);
	//! Add a MainNode (and its according CNode) to the system container
	Index AddMainNode(MainSystem& mainSystem, const py::dict& d);

	//! Create a specific object with objectType; returns object=Null if no success
	MainObject* CreateMainObject(MainSystem& mainSystem, STDstring objectType);
	//! Add a MainObject (and its according CObject) to the system container
	Index AddMainObject(MainSystem& mainSystem, const py::dict& d);

	//! Create a specific marker with markerType; returns marker=Null if no success
	MainMarker* CreateMainMarker(MainSystem& mainSystem, STDstring markerType);
	//! Add a MainMarker (and its according CMarker) to the system container; return index in item list or EXUstd::InvalidIndex if failed
	Index AddMainMarker(MainSystem& mainSystem, const py::dict& d);

	//! Create a specific load with loadType; returns node=Null if no success
	MainLoad* CreateMainLoad(MainSystem& mainSystem, STDstring loadType);
	//! Add a MainLoad (and its according CLoad) to the system container
	Index AddMainLoad(MainSystem& mainSystem, const py::dict& d);

	//! Create a specific sensor with sensorType; returns sensor=Null if no success
	MainSensor* CreateMainSensor(MainSystem& mainSystem, STDstring sensorType);
	//! Add a MainSensor (and its according CSensor) to the system container; return index in item list or EXUstd::InvalidIndex if failed
	Index AddMainSensor(MainSystem& mainSystem, const py::dict& d);

};


//! new way of registrating objects
template <class TItem>
class ClassFactoryItemsSystemData {
public:
	bool RegisterClass(const std::string& className, std::function<TItem* (CSystemData*)> creator) {
		// Avoid duplicate registration
		if (creators.find(className) != creators.end())
		{
			CHECKandTHROWstring((STDstring("ClassFactoryObjects: received duplicate: ") + className).c_str());
			return false;
		}
		creators[className] = creator;
		return true;
	}

	TItem* CreateInstance(const std::string& className, CSystemData* cSystemData) {
		auto it = creators.find(className);
		if (it != creators.end()) {
			return it->second(cSystemData); // Call the create function
		}
		CHECKandTHROWstring((STDstring("ClassFactoryItemsSystemData: CreateInstance received unkown object: ") + className).c_str());
		return nullptr;
	}

	static ClassFactoryItemsSystemData& Get() {
		static ClassFactoryItemsSystemData<TItem> instance;
		return instance;
	}
	static const std::map<std::string, std::function<TItem* (CSystemData*)>>& GetCreators() { return Get().creators; }

private:
	std::map<std::string, std::function<TItem*(CSystemData*)>> creators;
};

//! registration of any other item
template <class TItem>
class ClassFactoryItem {
public:
	bool RegisterClass(const std::string& className, std::function<TItem*()> creator) {
		// Avoid duplicate registration
		if (creators.find(className) != creators.end())
		{
			CHECKandTHROWstring((STDstring("ClassFactoryItem: received duplicate: ") + className).c_str());
			return false;
		}
		creators[className] = creator;
		return true;
	}

	TItem* CreateInstance(const std::string& className) {
		auto it = creators.find(className);
		if (it != creators.end()) {
			return it->second(); // Call the create function
		}
		CHECKandTHROWstring((STDstring("ClassFactoryItem: CreateInstance received unkown object: ") + className).c_str());
		return nullptr;
	}

	static ClassFactoryItem& Get() {
		static ClassFactoryItem<TItem> instance;
		return instance;
	}

	static const std::map<std::string, std::function<TItem* ()>>& GetCreators() { return Get().creators; }

private:
	std::map<std::string, std::function<TItem*()>> creators;
};

#endif
