/** ***********************************************************************************************
* @class        MainSystem
* @brief		MainSystem and ObjectFactory
* @details		Details:
				- handling of CSystem
				- initialization
				- pybind11 interface
				- object factory
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
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

#include <chrono> //sleep_for()
#include <thread>

#include "Main/MainSystemData.h"
#include "Main/MainSystem.h"
#include "Pymodules/PybindUtilities.h"

#include "Pymodules/PyGeneralContact.h"
#include "Utilities/ExceptionsTemplates.h" //for exceptions in solver steps
#include "System/versionCpp.h"

#include "Main/Experimental.h"
extern PySpecial pySpecial;			//! special features; affects exudyn globally; treat with care

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  SYSTEM FUNCTIONS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! build main system (unconventional way!)
MainSystem::MainSystem()
{
	//CSystem* cSystem = new CSystem();

	//MainSystem* mainSystem = new MainSystem();
	cSystem.GetSystemData().SetMainSystemBacklink(this);
	//this->cSystem = cSystem;
	this->mainSystemData.SetCSystemData(&(cSystem.GetSystemData()));
	this->LinkToVisualizationSystem(); //links the system to be rendered in OpenGL
	this->SetInteractiveMode(false);

	this->SetMainSystemIndex(-1); //indicates that there is no system container so far
	this->SetMainSystemContainer(nullptr);
}



//! reset all lists and deallocate memory
void MainSystem::Reset()
{
	mainSystemData.Reset(); //
	GetCSystem().GetSystemData().Reset();
	GetCSystem().GetPythonUserFunctions().Reset();
	GetCSystem().Initialize();
	GetCSystem().GetPostProcessData()->Reset();
	GetCSystem().ResetGeneralContacts();

	visualizationSystem.Reset();

	interactiveMode = false;
	//mainSystemIndex = -1; //... check if this is correctly set? test several SC.Reset and similar operations ==> this MainSystem would not be usable any more, as it is not linked to SystemContainer
}

void MainSystem::SystemHasChanged()
{
	if (!HasMainSystemContainer()) { PyWarning("MainSystem has not been yet linked to a system container. Having a MainSystem mbs, you should do first:\nSC=exudyn.SystemContainer()\nSC.AppendSystem(mbs)\n"); }
	GetCSystem().SystemHasChanged();
	GetVisualizationSystem().SetSystemHasChanged(true);
}

//! consistent saving template; flag is related to graphicsData (objects only)
template <typename ItemType>
auto AddItemsToList = [](py::dict& dict, const STDstring& dictName, const ResizableArray<ItemType*>& items) {
	auto itemList = py::list();
	for (ItemType* item : items) {
		itemList.append(item->GetDictionary());
	}
	dict[dictName.c_str()] = itemList;
};


//! function for getting all data and state; for pickling
py::dict MainSystem::GetDictionary() const
{
	auto d = py::dict();
	d["__version__"] = EXUstd::exudynVersion;

	if (GetCSystem().GetGeneralContacts().NumberOfItems() != 0)
	{
		PyWarning(STDstring("GetDictionary (pickle/copy): MainSystem contains GeneralContact which cannot be copied!"));
	}

	//const CSystemData& csd = GetCSystem().GetSystemData();
	const MainSystemData& msd = GetMainSystemData();

	AddItemsToList<MainNode>(d, "nodeList", msd.GetMainNodes());

	//AddItemsToList<MainObject>(d, "objectList", msd.GetMainObjects());
	auto itemList = py::list();
	for (MainObject* item : msd.GetMainObjects()) 
	{
		if (item->GetCObject()->HasUserFunction())
		{
			if (pySpecial.exceptions.dictionaryNonCopyable)
			{
				PyError(STDstring("GetDictionary (pickle/copy): MainSystem object '") + item->GetName() + "' has a user function which cannot be copied!");
			}
		}
		itemList.append(item->GetDictionary(true));
	}
	d["objectList"] = itemList;

	AddItemsToList<MainMarker>(d, "markerList", msd.GetMainMarkers());
	AddItemsToList<MainLoad>(d, "loadList", msd.GetMainLoads());
	AddItemsToList<MainSensor>(d, "sensorList", msd.GetMainSensors());

	auto userFunctions = py::dict();
	userFunctions["preStepFunction"] = cSystem.GetPythonUserFunctions().preStepFunction.GetPythonDictionary();
	userFunctions["postStepFunction"] = cSystem.GetPythonUserFunctions().postStepFunction.GetPythonDictionary();
	userFunctions["postNewtonFunction"] = cSystem.GetPythonUserFunctions().postNewtonFunction.GetPythonDictionary();
	d["userFunctions"] = userFunctions;

	//if (pySpecial.exceptions.dictionaryNonCopyable)
	//{
	//	if (cSystem.GetPythonUserFunctions().preStepFunction.userFunction != 0
	//		|| cSystem.GetPythonUserFunctions().postStepFunction.userFunction != 0
	//		|| cSystem.GetPythonUserFunctions().postNewtonFunction.userFunction != 0)
	//	{
	//		PyError(STDstring("GetDictionary (pickle/copy): MainSystem contains preStep / postStep / postNewton user function which cannot be copied!"));
	//	}
	//}


	auto settings = py::dict();
	settings["interactiveMode"] = interactiveMode;
	d["settings"] = settings;

	d["variables"] = variables;
	d["systemVariables"] = systemVariables;

	//missing:
	//d["cSystemData"]
	//d["cData"]

	return d;
}

//! function for setting all data from dict; for pickling
void MainSystem::SetDictionary(const py::dict& d)
{
	Reset();

	if (EXUstd::exudynVersion != py::cast<STDstring>(d["__version__"]) && pySpecial.exceptions.dictionaryVersionMismatch)
	{
		PyError(STDstring("SetDictionary: Exudyn version is ") + EXUstd::exudynVersion +
			", but loaded dictionary has been built with version " + py::cast<STDstring>(d["__version__"])+"; you can disable this exception in exudyn.special.exceptions");
	}

	const MainSystemData& msd = GetMainSystemData();
	const CSystemData& csd = GetCSystem().GetSystemData();

	py::list nodeList   = py::cast<py::list>(d["nodeList"]);
	py::list objectList = py::cast<py::list>(d["objectList"]);
	py::list markerList = py::cast<py::list>(d["markerList"]);
	py::list loadList   = py::cast<py::list>(d["loadList"]);
	py::list sensorList = py::cast<py::list>(d["sensorList"]);
	for (auto item : nodeList  ) { mainObjectFactory.AddMainNode(*this, py::cast<py::dict>(item)); }
	for (auto item : objectList) { mainObjectFactory.AddMainObject(*this, py::cast<py::dict>(item)); }
	for (auto item : markerList) { mainObjectFactory.AddMainMarker(*this, py::cast<py::dict>(item)); }
	for (auto item : loadList  ) { mainObjectFactory.AddMainLoad(*this, py::cast<py::dict>(item)); }
	for (auto item : sensorList) { mainObjectFactory.AddMainSensor(*this, py::cast<py::dict>(item)); }

	cSystem.GetPythonUserFunctions().preStepFunction.SetPythonObject(d["userFunctions"]["preStepFunction"]);
	cSystem.GetPythonUserFunctions().postStepFunction.SetPythonObject(d["userFunctions"]["postStepFunction"]);
	cSystem.GetPythonUserFunctions().postNewtonFunction.SetPythonObject(d["userFunctions"]["postNewtonFunction"]);

	interactiveMode = py::cast<bool>(d["settings"]["interactiveMode"]);

	variables = d["variables"];
	systemVariables = d["systemVariables"];
}

MainSystemContainer& MainSystem::GetMainSystemContainer() 
{
	return *mainSystemContainerBacklink; 
}
const MainSystemContainer& MainSystem::GetMainSystemContainerConst() const
{ 
	return *mainSystemContainerBacklink; 
}

//!  if interAciveMode == true: causes Assemble() to be called; this guarantees that the system is always consistent to be drawn
void MainSystem::InteractiveModeActions()
{
	if (GetInteractiveMode())
	{
		GetCSystem().Assemble(*this);
		GetCSystem().GetPostProcessData()->SendRedrawSignal();
	}
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
void MainSystem::PySetPreStepUserFunction(const py::object& value)
{
    GenericExceptionHandling([&]
    {
		//cSystem.GetPythonUserFunctions().preStepFunction.userFunction = EPyUtils::GetSTDfunction< std::function<bool(const MainSystem&, Real)>>(value, "MainSystem::SetPreStepUserFunction");
		cSystem.GetPythonUserFunctions().preStepFunction.SetPythonUserFunction(value);

		cSystem.GetPythonUserFunctions().mainSystem = this;
    }, "MainSystem::SetPreStepUserFunction: argument must be Python function or 0");
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
py::object MainSystem::PyGetPreStepUserFunction(bool asDict)
{
	return cSystem.GetPythonUserFunctions().preStepFunction.GetPythonDictionary();
	//return preStepFunctionPython; //in future, but added to PythonUserFunctions
	//.def("GetPreStepUserFunction", &MainSystem::PyGetPreStepUserFunction, "...")
}

//! set user function to be called by solvers at end of step, just before writing results (static or dynamic step)
void MainSystem::PySetPostStepUserFunction(const py::object& value)
{
	GenericExceptionHandling([&]
		{
			//cSystem.GetPythonUserFunctions().postStepFunction.userFunction = EPyUtils::GetSTDfunction< std::function<bool(const MainSystem & mainSystem, Real t)>>(value, "MainSystem::SetPostStepUserFunction");
			cSystem.GetPythonUserFunctions().postStepFunction.SetPythonUserFunction(value);

			cSystem.GetPythonUserFunctions().mainSystem = this;
		}, "MainSystem::SetPostStepUserFunction: argument must be Python function or 0");
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
py::object MainSystem::PyGetPostStepUserFunction(bool asDict)
{
	return cSystem.GetPythonUserFunctions().postStepFunction.GetPythonDictionary();
	//return postStepFunctionPython; //in future, but added to PythonUserFunctions
	//.def("GetPostStepUserFunction", &MainSystem::PyGetPostStepUserFunction, "...")
}

//! set user function to be called immediately after Newton (after an update of the solution has been computed, but before discontinuous iteration)
void MainSystem::PySetPostNewtonUserFunction(const py::object& value)
{
    GenericExceptionHandling([&]
    {
		//cSystem.GetPythonUserFunctions().postNewtonFunction.userFunction = EPyUtils::GetSTDfunction< std::function<StdVector2D(const MainSystem & mainSystem, Real t)>>(value, "MainSystem::SetPostNewtonUserFunction");
		cSystem.GetPythonUserFunctions().postNewtonFunction.SetPythonUserFunction(value);
		
		cSystem.GetPythonUserFunctions().mainSystem = this;
    }, "MainSystem::SetPostNewtonUserFunction: argument must be Python function or 0");
}

py::object MainSystem::PyGetPostNewtonUserFunction(bool asDict)
{
	return cSystem.GetPythonUserFunctions().postNewtonFunction.GetPythonDictionary();
}

//create a new general contact and add to system
PyGeneralContact& MainSystem::AddGeneralContact()
{
	PyGeneralContact* gContact = new PyGeneralContact();
	cSystem.GetGeneralContacts().Append((GeneralContact*)(gContact)); //(GeneralContact*)
	return (PyGeneralContact&)*cSystem.GetGeneralContacts().Last();
}

//obtain read/write access to general contact
PyGeneralContact& MainSystem::GetGeneralContact(Index generalContactNumber)
{
	if (generalContactNumber >= 0 && generalContactNumber < cSystem.GetGeneralContacts().NumberOfItems())
	{
		return (PyGeneralContact&)*cSystem.GetGeneralContacts().Last();
	}
	else
	{
		PyError("MainSystem::GeneralContact: access to invalid index " + EXUstd::ToString(generalContactNumber));
		return (PyGeneralContact&)*cSystem.GetGeneralContacts().Last(); //code not reached ...
	}
}

//delete general contact, resort indices
void MainSystem::DeleteGeneralContact(Index generalContactNumber)
{
	if (generalContactNumber >= 0 && generalContactNumber < cSystem.GetGeneralContacts().NumberOfItems())
	{
		delete cSystem.GetGeneralContacts()[generalContactNumber];
		cSystem.GetGeneralContacts().Remove(generalContactNumber); //rearrange array
	}
	else
	{
		PyError("MainSystem::DeleteGeneralContact: access to invalid index " + EXUstd::ToString(generalContactNumber));
	}

}

Index MainSystem::NumberOfGeneralContacts() const
{
	return cSystem.GetGeneralContacts().NumberOfItems();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  VISUALIZATION FUNCTIONS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! set rendering true/false
void MainSystem::ActivateRendering(bool flag)
{
	visualizationSystem.ActivateRendering(flag);
}

//! this function links the VisualizationSystem to a render engine, such that the changes in the graphics structure drawn upon updates, etc.
//  This function is called on creation of a main system and automatically links to renderer
bool MainSystem::LinkToVisualizationSystem()
{
	visualizationSystem.LinkToSystemData(&GetCSystem().GetSystemData());
	visualizationSystem.LinkToMainSystem(this);
	visualizationSystem.LinkPostProcessData(GetCSystem().GetPostProcessData());
	visualizationSystem.ActivateRendering(true); //activate rendering on startup
	return true; 
}

//! for future, unregister mbs from renderer
bool MainSystem::UnlinkVisualizationSystem()
{
	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  NODE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainNode(const py::dict& d)
{
	SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainNode(*this, d);
	InteractiveModeActions();
	return ind;
};

NodeIndex MainSystem::AddMainNodePyClass(const py::object& pyObject)
{
	py::dict dictObject;
	Index itemIndex = 0;

	try
	{
		if (py::isinstance<py::dict>(pyObject))
		{
			dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		}
		else //must be itemInterface convertable to dict ==> otherwise raises pybind error
		{
			dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		}
		itemIndex = AddMainNode(dictObject);
	}
	catch (const EXUexception& ex)
	{
		//will fail, if dictObject is invalid: PyError("Error in AddNode(...) with dictionary=\n" + EXUstd::ToString(dictObject) +
		PyError(STDstring("Error in AddNode(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\nException message=\n" + STDstring(ex.what()));
		//not needed due to change of PyError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		PyError(STDstring("Error in AddNode(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\n");
	}
	return itemIndex;


	//if (py::isinstance<py::dict>(pyObject))
	//{
	//	py::dict dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
	//	return AddMainNode(dictObject);

	//}
	//else //must be itemInterface convertable to dict ==> otherwise raises pybind error
	//{
	//	py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
	//	return AddMainNode(dictObject);
	//}
}

//! get node's dictionary by name; does not throw a error message
NodeIndex MainSystem::PyGetNodeNumber(STDstring nodeName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainNodes(), nodeName);

	if (ind != EXUstd::InvalidIndex)
	{
		return ind;
	}
	else
	{
		return EXUstd::InvalidIndex;
	}
}

//! hook to read node's dictionary
py::dict MainSystem::PyGetNode(const py::object& itemIndex)
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetNode: invalid access to node number ") + EXUstd::ToString(nodeNumber));
		py::dict d;
		return d;
	}
}

////! get node's dictionary by name
//py::dict MainSystem::PyGetNodeByName(STDstring nodeName)
//{
//	Index ind = (Index)PyGetNodeNumber(nodeName);
//	if (ind != EXUstd::InvalidIndex) { return PyGetNode(ind); }
//	else
//	{
//		PyError(STDstring("MainSystem::GetNode: invalid access to node '") + nodeName + "'");
//		return py::dict();
//	}
//}

//! modify node's dictionary
void MainSystem::PyModifyNode(const py::object& itemIndex, py::dict nodeDict)
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		SystemHasChanged();
		mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetWithDictionary(nodeDict);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyNode: invalid access to node number ") + EXUstd::ToString(nodeNumber));
	}
}

////! modify node's dictionary
//void MainSystem::PyModifyNode(STDstring nodeName, py::dict d)
//{
//	Index nodeNumber = PyGetNodeNumber(nodeName);
//	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
//	{
//		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetWithDictionary(d);
//	}
//	else
//	{
//		PyError(STDstring("ModifyNodeDictionary: invalid access to node '") + nodeName + "'");
//	}
//}

//! get node's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetNodeDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetNodeDefaults: typeName needed'"));
		return d;
	}
	
	MainNode* node = mainObjectFactory.CreateMainNode(*this, typeName); //create node with name

	if (node)
	{
		d = node->GetDictionary();
		delete node->GetCNode();
		delete node;
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeDefaults: unknown node type '") + typeName + "'");
	}
	return d;
}

py::object MainSystem::PyGetNodeOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{

	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetNodeOutput", configuration, nodeNumber, ItemType::Node);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetNodeOutput", variableType, configuration, nodeNumber, ItemType::Node);

		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetOutputVariable(variableType, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeOutputVariable: invalid access to node number ") + EXUstd::ToString(nodeNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! get index in global ODE2 coordinate vector for first node coordinate
Index MainSystem::PyGetNodeODE2Index(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::ODE2variables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalODE2CoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeODE2Index: invalid access to node number ") + EXUstd::ToString(nodeNumber) + ": not an ODE2 node");
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeODE2Index: invalid access to node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)");
		return EXUstd::InvalidIndex;
	}
}

//! get index in global ODE1 coordinate vector for first node coordinate
Index MainSystem::PyGetNodeODE1Index(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::ODE1variables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalODE1CoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeODE1Index: invalid access to node number ") + EXUstd::ToString(nodeNumber) + ": not an ODE1 node");
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeODE1Index: invalid access to node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)");
		return EXUstd::InvalidIndex;
	}
}

//! get index in global AE coordinate vector for first node coordinate
Index MainSystem::PyGetNodeAEIndex(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::AEvariables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalAECoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeAEIndex: invalid access to node number ") + EXUstd::ToString(nodeNumber) + ": not an AE node");
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeAEIndex: invalid access to node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)");
		return EXUstd::InvalidIndex;
	}
}



////! call pybind object function, possibly with arguments; empty function, to be overwritten in specialized class
//py::object MainSystem::PyCallNodeFunction(Index nodeNumber, STDstring functionName, py::dict args)
//{
//	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
//	{
//		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->CallFunction(functionName, args);
//	}
//	else
//	{
//		PyError(STDstring("MainSystem::ModifyObject: invalid access to node number ") + EXUstd::ToString(nodeNumber));
//		return py::int_(EXUstd::InvalidIndex);
//		//return py::object();
//	}
//
//}


//! Get (read) parameter 'parameterName' of 'nodeNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetNodeParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeParameter: invalid access to node number ") + EXUstd::ToString(nodeNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'nodeNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetNodeParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetNodeParameter: invalid access to node number ") + EXUstd::ToString(nodeNumber));
	}
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  OBJECT
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainObject(const py::dict& d)
{
	SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainObject(*this, d);
	InteractiveModeActions();

	return ind;
};

ObjectIndex MainSystem::AddMainObjectPyClass(const py::object& pyObject)
{
	py::dict dictObject;
	Index itemIndex = 0;
	try
	{
		if (py::isinstance<py::dict>(pyObject))
		{
			dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		}
		else //must be itemInterface convertable to dict ==> otherwise raises pybind error
		{
			dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		}
		itemIndex = AddMainObject(dictObject);
	}
	catch (const EXUexception& ex)
	{
		//will fail, if dictObject is invalid: PyError("Error in AddObject(...) with dictionary=\n" + EXUstd::ToString(dictObject) +
		PyError(STDstring("Error in AddObject(...):") +
				"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\nException message=\n" + STDstring(ex.what()));
		//not needed due to change of PyError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		PyError(STDstring("Error in AddObject(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\n");
	}
	return itemIndex;
}

//! get object's dictionary by name; does not throw a error message
ObjectIndex MainSystem::PyGetObjectNumber(STDstring itemName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainObjects(), itemName);
	if (ind != EXUstd::InvalidIndex)
	{
		return ind;
	}
	else
	{
		return EXUstd::InvalidIndex;
	}
}

//! hook to read object's dictionary
py::dict MainSystem::PyGetObject(const py::object& itemIndex, bool addGraphicsData)
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetDictionary(addGraphicsData);
	}
	else
	{
		PyError(STDstring("MainSystem::GetObject: invalid access to object number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

////! get object's dictionary by name
//py::dict MainSystem::PyGetObjectByName(STDstring itemName)
//{
//	Index ind = (Index)PyGetObjectNumber(itemName);
//	if (ind != EXUstd::InvalidIndex) { return PyGetObject(ind); }
//	else
//	{
//		PyError(STDstring("MainSystem::GetObject: invalid access to object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifyObject(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		SystemHasChanged();
		mainSystemData.GetMainObjects().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyObject: invalid access to object number ") + EXUstd::ToString(itemNumber));
	}
}

//! get object's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetObjectDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetObjectDefaults: typeName needed'"));
		return d;
	}

	MainObject* object = mainObjectFactory.CreateMainObject(*this, typeName); //create object with typeName

	if (object)
	{
		d = object->GetDictionary();
		delete object->GetCObject();
		delete object;
	}
	else
	{
		PyError(STDstring("MainSystem::GetObjectDefaults: unknown object type '") + typeName + "'");
	}
	return d;
}

////! call pybind object function, possibly with arguments; empty function, to be overwritten in specialized class
//py::object MainSystem::PyCallObjectFunction(const py::object& itemIndex, STDstring functionName, py::dict args)
//{
//	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
//	{
//		return mainSystemData.GetMainObjects().GetItem(itemNumber)->CallFunction(functionName, args);
//	}
//	else
//	{
//		PyError(STDstring("MainSystem::ModifyObject: invalid access to object number ") + EXUstd::ToString(itemNumber));
//		return py::int_(EXUstd::InvalidIndex);
//		//return py::object();
//	}
//}

//! Get specific output variable with variable type
py::object MainSystem::PyGetObjectOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		GetMainSystemData().RaiseIfNotConsistentOrIllegalConfiguration("GetObjectOutput", configuration, itemNumber, ItemType::Object);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutput", variableType, configuration, itemNumber, ItemType::Object);

		if ((Index)mainSystemData.GetMainObjects().GetItem(itemNumber)->GetCObject()->GetType() & (Index)CObjectType::Connector)
		{
			CHECKandTHROW(configuration == ConfigurationType::Current, "GetObjectOutput: may only be called for connectors with Current configuration");
			MarkerDataStructure markerDataStructure;
			const bool computeJacobian = false; //not needed for OutputVariables
			CObjectConnector* connector = (CObjectConnector*)(mainSystemData.GetMainObjects().GetItem(itemNumber)->GetCObject());
			GetCSystem().GetSystemData().ComputeMarkerDataStructure(connector, computeJacobian, markerDataStructure);

			return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariableConnector(variableType, markerDataStructure, itemNumber);

		} else
		{
			return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariable(variableType, configuration, itemNumber);
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetObjectOutputVariable: invalid access to object number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Get specific output variable with variable type; ONLY for bodies;
//py::object MainSystem::PyGetObjectOutputBody(Index objectNumber, OutputVariableType variableType,
//	const Vector3D& localPosition, ConfigurationType configuration) //no conversion from py to Vector3D!
py::object MainSystem::PyGetObjectOutputVariableBody(const py::object& itemIndex, OutputVariableType variableType,
		const std::vector<Real>& localPosition, ConfigurationType configuration) const
{

		Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
		if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
		{
			GetMainSystemData().RaiseIfNotConsistentNorReference("GetObjectOutputBody", configuration, itemNumber, ItemType::Object);
			GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutputBody", variableType, configuration, itemNumber, ItemType::Object);

			if (localPosition.size() == 3)
			{
				const MainObject* mo = mainSystemData.GetMainObjects().GetItem(itemNumber);

				return mo->GetOutputVariableBody(variableType, localPosition, configuration, itemNumber);
			}
			else
			{
				PyError(STDstring("MainSystem::GetOutputVariableBody: invalid localPosition: expected vector with 3 real values; object number ") +
					EXUstd::ToString(itemNumber));
				return py::int_(EXUstd::InvalidIndex);
				//return py::object();
			}
		}
		else
		{
			PyError(STDstring("MainSystem::GetObjectOutputVariableBody: invalid access to object number ") + EXUstd::ToString(itemNumber));
			return py::int_(EXUstd::InvalidIndex);
			//return py::object();
		}
}

//! get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType
py::object MainSystem::PyGetObjectOutputVariableSuperElement(const py::object& itemIndex, OutputVariableType variableType, 
	Index meshNodeNumber, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetObjectOutputSuperElement", configuration, itemNumber, ItemType::Object);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutputVariableSuperElement", variableType, configuration, itemNumber, ItemType::Object);
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariableSuperElement(variableType, meshNodeNumber, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::PyGetObjectOutputVariableSuperElement: invalid access to object number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! Get (read) parameter 'parameterName' of 'objectNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetObjectParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetObjectParameter: invalid access to object number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'objectNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetObjectParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		mainSystemData.GetMainObjects().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetObjectParameter: invalid access to object number ") + EXUstd::ToString(itemNumber));
	}
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  MARKER
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainMarker(const py::dict& d)
{
	SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainMarker(*this, d);
	InteractiveModeActions();
	return ind;
};

MarkerIndex MainSystem::AddMainMarkerPyClass(const py::object& pyObject)
{
	py::dict dictObject;
	Index itemIndex = 0;

	try
	{
		if (py::isinstance<py::dict>(pyObject))
		{
			dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		}
		else //must be itemInterface convertable to dict ==> otherwise raises pybind error
		{
			dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		}
		itemIndex = AddMainMarker(dictObject);
	}
	catch (const EXUexception& ex)
	{
		//will fail, if dictObject is invalid: PyError("Error in AddMarker(...) with dictionary=\n" + EXUstd::ToString(dictObject) +
		PyError(STDstring("Error in AddMarker(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\nException message=\n" + STDstring(ex.what()));
		//not needed due to change of PyError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		PyError(STDstring("Error in AddMarker(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\n");
	}
	return itemIndex;
}

//! get object's dictionary by name; does not throw a error message
MarkerIndex MainSystem::PyGetMarkerNumber(STDstring itemName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainMarkers(), itemName);
	if (ind != EXUstd::InvalidIndex)
	{
		return ind;
	}
	else
	{
		return EXUstd::InvalidIndex;
	}
}

//! hook to read object's dictionary
py::dict MainSystem::PyGetMarker(const py::object& itemIndex)
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarker: invalid access to marker number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

////! get object's dictionary by name
//py::dict MainSystem::PyGetMarkerByName(STDstring itemName)
//{
//	Index ind = (Index)PyGetMarkerNumber(itemName);
//	if (ind != EXUstd::InvalidIndex) { return PyGetMarker(ind); }
//	else
//	{
//		PyError(STDstring("MainSystem::GetMarker: invalid access to object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifyMarker(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		SystemHasChanged();
		mainSystemData.GetMainMarkers().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyMarker: invalid access to marker number ") + EXUstd::ToString(itemNumber));
	}
}

//! get marker's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetMarkerDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetMarkerDefaults: typeName needed'"));
		return d;
	}

	MainMarker* object = mainObjectFactory.CreateMainMarker(*this, typeName); //create object with typeName

	if (object)
	{
		d = object->GetDictionary();
		delete object->GetCMarker();
		delete object;
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerDefaults: unknown object type '") + typeName + "'");
	}
	return d;
}

//! Get (read) parameter 'parameterName' of 'markerNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetMarkerParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerParameter: invalid access to marker number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'markerNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetMarkerParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		mainSystemData.GetMainMarkers().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetMarkerParameter: invalid access to marker number ") + EXUstd::ToString(itemNumber));
	}
}

//! Get specific output variable with variable type
py::object MainSystem::PyGetMarkerOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetMarkerOutput", configuration, itemNumber, ItemType::Marker);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutputVariableSuperElement", variableType, configuration, itemNumber, ItemType::Marker);

		//the marker function itself will raise an error, if it is not able to return the according output variable
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetOutputVariable(GetCSystem().GetSystemData(), variableType, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerOutput: invalid access to marker number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  LOAD
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainLoad(const py::dict& d)
{
	SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainLoad(*this, d);
	InteractiveModeActions();
	return ind;
};

LoadIndex MainSystem::AddMainLoadPyClass(const py::object& pyObject)
{
	py::dict dictObject;
	Index itemIndex = 0;

	try
	{
		if (py::isinstance<py::dict>(pyObject))
		{
			dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		}
		else //must be itemInterface convertable to dict ==> otherwise raises pybind error
		{
			dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		}
		itemIndex = AddMainLoad(dictObject);
	}
	catch (const EXUexception& ex)
	{
		//will fail, if dictObject is invalid: PyError("Error in AddLoad(...) with dictionary=\n" + EXUstd::ToString(dictObject) +
		PyError(STDstring("Error in AddLoad(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\nException message=\n" + STDstring(ex.what()));
		//not needed due to change of PyError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		PyError(STDstring("Error in AddLoad(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\n");
	}
	return itemIndex;
}

//! get object's dictionary by name; does not throw a error message
LoadIndex MainSystem::PyGetLoadNumber(STDstring itemName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainLoads(), itemName);
	if (ind != EXUstd::InvalidIndex)
	{
		return ind;
	}
	else
	{
		return EXUstd::InvalidIndex;
	}
}

//! hook to read object's dictionary
py::dict MainSystem::PyGetLoad(const py::object& itemIndex)
{
	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoad: invalid access to load number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

////! get object's dictionary by name
//py::dict MainSystem::PyGetLoadByName(STDstring itemName)
//{
//	Index ind = (Index)PyGetLoadNumber(itemName);
//	if (ind != EXUstd::InvalidIndex) { return PyGetLoad(ind); }
//	else
//	{
//		PyError(STDstring("MainSystem::GetLoad: invalid access to object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifyLoad(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		SystemHasChanged();
		mainSystemData.GetMainLoads().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyLoad: invalid access to load number ") + EXUstd::ToString(itemNumber));
	}
}

//! get LoadPoint default values, which helps for manual writing of python input
py::dict MainSystem::PyGetLoadDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetLoadDefaults: typeName needed'"));
		return d;
	}

	MainLoad* object = mainObjectFactory.CreateMainLoad(*this, typeName); //create object with typeName

	if (object)
	{
		d = object->GetDictionary();
		delete object->GetCLoad();
		delete object;
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadDefaults: unknown load type '") + typeName + "'");
	}
	return d;
}

//! Get current load values, specifically if user-defined loads are used
py::object MainSystem::PyGetLoadValues(const py::object& itemIndex) const
{

	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		GetMainSystemData().RaiseIfNotConsistent("GetLoadValues", itemNumber, ItemType::Load);
		Real t = GetCSystem().GetSystemData().GetCData().GetCurrent().GetTime(); //only current time available
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetLoadValues(GetCSystem().GetSystemData().GetMainSystemBacklink(), t);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadValues: invalid access to load number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! Get (read) parameter 'parameterName' of 'loadNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetLoadParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadParameter: invalid access to load number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'loadNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetLoadParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		mainSystemData.GetMainLoads().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetLoadParameter: invalid access to load number ") + EXUstd::ToString(itemNumber));
	}
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  SENSOR
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainSensor(const py::dict& d)
{
	SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainSensor(*this, d);
	InteractiveModeActions();
	return ind;
};

SensorIndex MainSystem::AddMainSensorPyClass(const py::object& pyObject)
{
	py::dict dictObject;
	Index itemIndex = 0;

	try
	{
		if (py::isinstance<py::dict>(pyObject))
		{
			dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		}
		else //must be itemInterface convertable to dict ==> otherwise raises pybind error
		{
			dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		}
		itemIndex = AddMainSensor(dictObject);
	}
	catch (const EXUexception& ex)
	{
		//will fail, if dictObject is invalid: PyError("Error in AddSensor(...) with dictionary=\n" + EXUstd::ToString(dictObject) +
		PyError(STDstring("Error in AddSensor(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\nException message=\n" + STDstring(ex.what()));
		//not needed due to change of PyError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		PyError(STDstring("Error in AddSensor(...):") +
			"\nCheck your python code (negative indices, invalid or undefined parameters, ...)\n");
	}
	return itemIndex;
}

//! get object's dictionary by name; does not throw a error message
SensorIndex MainSystem::PyGetSensorNumber(STDstring itemName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainSensors(), itemName);
	if (ind != EXUstd::InvalidIndex)
	{
		return ind;
	}
	else
	{
		return EXUstd::InvalidIndex;
	}
}

//! hook to read object's dictionary
py::dict MainSystem::PyGetSensor(const py::object& itemIndex)
{
	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensor: invalid access to sensor number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

////! get object's dictionary by name
//py::dict MainSystem::PyGetSensorByName(STDstring itemName)
//{
//	Index ind = (Index)PyGetSensorNumber(itemName);
//	if (ind != EXUstd::InvalidIndex) { return PyGetSensor(ind); }
//	else
//	{
//		PyError(STDstring("MainSystem::GetSensor: invalid access to object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifySensor(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		SystemHasChanged();
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifySensor: invalid access to sensor number ") + EXUstd::ToString(itemNumber));
	}
}

//! get Sensor's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetSensorDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetSensorDefaults: typeName needed'"));
		return d;
	}

	MainSensor* object = mainObjectFactory.CreateMainSensor(*this, typeName); //create object with typeName

	if (object)
	{
		d = object->GetDictionary();
		delete object->GetCSensor();
		delete object;
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorDefaults: unknown sensor type '") + typeName + "'");
	}
	return d;
}

//! get sensor's values
py::object MainSystem::PyGetSensorValues(const py::object& itemIndex, ConfigurationType configuration)
{

	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetSensorValues", configuration, itemNumber, ItemType::Sensor);
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetSensorValues(GetCSystem().GetSystemData(), configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorValues: invalid access to sensor number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! get sensor's stored data (if it exists ...)
py::array_t<Real> MainSystem::PyGetSensorStoredData(const py::object& itemIndex)
{

	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		if (!mainSystemData.GetMainSensors().GetItem(itemNumber)->GetCSensor()->GetStoreInternalFlag())
		{
			PyError(STDstring("MainSystem::GetSensorStoredData: sensor number ") + EXUstd::ToString(itemNumber)+" has no internal data as storeInternal==False");
			return py::int_(EXUstd::InvalidIndex);
		}
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetInternalStorage();
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorStoredData: invalid access to sensor number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}



//! Get (read) parameter 'parameterName' of 'sensorNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetSensorParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorParameter: invalid access to sensor number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'SensorNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetSensorParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetSensorParameter: invalid access to sensor number ") + EXUstd::ToString(itemNumber));
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//MainSystemData functions
void MainSystemData::RaiseIfConfigurationIllegal(const char* functionName, ConfigurationType configuration, Index itemIndex, ItemType itemType) const
{
	if ((Index)configuration <= (Index)ConfigurationType::_None)
	{
		STDstring s = STDstring("MainSystem::") + functionName;
		if (itemIndex >= 0) { s += STDstring("(") + EXUstd::ToString(itemType) + " " + EXUstd::ToString(itemIndex) + ")"; }
		s += ": called with illegal configuration ConfigurationType._None";
		CHECKandTHROWstring(s);
	}
	else if ((Index)configuration >= (Index)ConfigurationType::EndOfEnumList)
	{
		STDstring s = STDstring("MainSystem::") + functionName;
		if (itemIndex >= 0) { s += STDstring("(") + EXUstd::ToString(itemType) + " " + EXUstd::ToString(itemIndex) + ")"; }
		s += ": called with illegal configuration ConfigurationType.???";
		CHECKandTHROWstring(s);
	}
	//else if (configuration == ConfigurationType::StartOfStep) //StartOfStep currently also initialized in CSystem
	//{
	//	STDstring s = STDstring("MainSystem::") + functionName;
	//	s += ": called with illegal configuration ConfigurationType.StartOfStep";
	//	CHECKandTHROWstring(s);
	//}
}

void MainSystemData::RaiseIfNotConsistentNorReference(const char* functionName, ConfigurationType configuration, Index itemIndex, ItemType itemType) const
{
	if (!GetCSystemData().GetCData().IsSystemConsistent() && configuration != ConfigurationType::Reference)
	{
		STDstring s = STDstring("MainSystem::") + functionName;
		if (itemIndex >= 0) { s += STDstring("(") + EXUstd::ToString(itemType) + " " + EXUstd::ToString(itemIndex) + ")"; }
		s += ": called with illegal configuration for inconsistent system; it may be either called for consistent system (needs mbs.Assemble() prior to this call and not change in mbs any more) or use configuration = ConfigurationType.Reference";
		CHECKandTHROWstring(s);
	}
}

void MainSystemData::RaiseIfNotConsistent(const char* functionName, Index itemIndex, ItemType itemType) const
{
	if (!GetCSystemData().GetCData().IsSystemConsistent())
	{
		STDstring s = STDstring("MainSystem::") + functionName;
		if (itemIndex >= 0) { s += STDstring("(") + EXUstd::ToString(itemType) + " " + EXUstd::ToString(itemIndex) + ")"; }
		s += ": called for inconsistent system; a call to mbs.Assemble() is necessary prior to this function call (such that mbs.systemIsConsistent returns True)";
		CHECKandTHROWstring(s);
	}
}

void MainSystemData::RaiseIfNotConsistentOrIllegalConfiguration(const char* functionName, ConfigurationType configuration,
	Index itemIndex, ItemType itemType) const
{
	RaiseIfNotConsistent(functionName, itemIndex, itemType);
	RaiseIfConfigurationIllegal(functionName, configuration, itemIndex, itemType);
}

void MainSystemData::RaiseIfNotOutputVariableTypeForReferenceConfiguration(const char* functionName, 
	OutputVariableType variableType, ConfigurationType configuration, Index itemIndex, ItemType itemType) const
{
	if (configuration == ConfigurationType::Reference && !IsOutputVariableTypeForReferenceConfiguration(variableType))
	{
		STDstring s = functionName;
		if (itemIndex >= 0) { s += STDstring("(") + EXUstd::ToString(itemType) + " " + EXUstd::ToString(itemIndex) + ")"; }
		s += ": called with ConfigurationType.Reference is only possible with an OutputVariableType suitable for reference configuration, being Position, Displacement, Distance, Rotation or Coordinate-like, but not Velocity, Acceleration, Force, Stress, etc.";
		CHECKandTHROWstring(s);
	}
}

