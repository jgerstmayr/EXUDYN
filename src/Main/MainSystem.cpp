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
	cSystem.GetSystemData().SetMainSystemBacklink(this);
	this->mainSystemData.SetCSystemData(&(cSystem.GetSystemData()));
	this->LinkToVisualizationSystem(); //links the system to be rendered in OpenGL
	this->SetInteractiveMode(false);

	this->SetMainSystemIndex(-1); //indicates that there is no system container so far
	this->SetMainSystemContainer(nullptr);

	//=> Reset() is called in MainSystemContainer!
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
}

void MainSystem::SystemHasChanged()
{
	if (!HasMainSystemContainer()) { PyWarning("MainSystem has not been yet linked to a system container. Having a MainSystem mbs, you should do first:\nSC=exudyn.SystemContainer()\nSC.Append(mbs)\n"); }
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
	userFunctions["preNewtonResidualFunction"] = cSystem.GetPythonUserFunctions().preNewtonResidualFunction.GetPythonDictionary();
	userFunctions["systemJacobianFunction"] = cSystem.GetPythonUserFunctions().systemJacobianFunction.GetPythonDictionary();
	d["userFunctions"] = userFunctions;

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

	//const MainSystemData& msd = GetMainSystemData();
	//const CSystemData& csd = GetCSystem().GetSystemData();

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
	cSystem.GetPythonUserFunctions().preNewtonResidualFunction.SetPythonObject(d["userFunctions"]["preNewtonResidualFunction"]);
	cSystem.GetPythonUserFunctions().systemJacobianFunction.SetPythonObject(d["userFunctions"]["systemJacobianFunction"]);

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
		cSystem.GetPythonUserFunctions().preStepFunction.SetPythonUserFunction(value);

		cSystem.GetPythonUserFunctions().mainSystem = this;
    }, "MainSystem::SetPreStepUserFunction: argument must be Python function or 0");
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
py::object MainSystem::PyGetPreStepUserFunction(bool asDict)
{
	return cSystem.GetPythonUserFunctions().preStepFunction.GetPythonDictionary();
}

//! set user function to be called by solvers at end of step, just before writing results (static or dynamic step)
void MainSystem::PySetPostStepUserFunction(const py::object& value)
{
	GenericExceptionHandling([&]
		{
			cSystem.GetPythonUserFunctions().postStepFunction.SetPythonUserFunction(value);

			cSystem.GetPythonUserFunctions().mainSystem = this;
		}, "MainSystem::SetPostStepUserFunction: argument must be Python function or 0");
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
py::object MainSystem::PyGetPostStepUserFunction(bool asDict)
{
	return cSystem.GetPythonUserFunctions().postStepFunction.GetPythonDictionary();
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

//! set user function to be called by solvers at beginning of step (static or dynamic step)
void MainSystem::PySetPreNewtonResidualUserFunction(const py::object& value)
{
	GenericExceptionHandling([&]
		{
			cSystem.GetPythonUserFunctions().preNewtonResidualFunction.SetPythonUserFunction(value);

			cSystem.GetPythonUserFunctions().mainSystem = this;
		}, "MainSystem::SetPreStepUserFunction: argument must be Python function or 0");
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
py::object MainSystem::PyGetPreNewtonResidualUserFunction(bool asDict)
{
	return cSystem.GetPythonUserFunctions().preNewtonResidualFunction.GetPythonDictionary();
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
void MainSystem::PySetSystemJacobianUserFunction(const py::object& value)
{
	GenericExceptionHandling([&]
		{
			cSystem.GetPythonUserFunctions().systemJacobianFunction.SetPythonUserFunction(value);

			cSystem.GetPythonUserFunctions().mainSystem = this;
		}, "MainSystem::SetPreStepUserFunction: argument must be Python function or 0");
}

//! set user function to be called by solvers at beginning of step (static or dynamic step)
py::object MainSystem::PyGetSystemJacobianUserFunction(bool asDict)
{
	return cSystem.GetPythonUserFunctions().systemJacobianFunction.GetPythonDictionary();
	return py::object();
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

//! interrupt further computation until user input --> 'pause' function
void MainSystem::WaitForUserToContinue(bool printMessage, bool deprecationWarning)
{ 
	if (deprecationWarning) { PyWarning("MainSystem.WaitForUserToContinue(): function is deprecated; for SystemContainer SC use set SC.renderer.DoIdleTasks() instead"); }

	GetCSystem().GetPostProcessData()->WaitForUserToContinue(printMessage);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  NODE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

py::dict MainSystem::GetAvailableFactoryItems()
{
	return GetMainObjectFactory().GetAvailableFactoryItems();
}

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

//! Consistently deleta a MainNode from Python
void MainSystem::PyDeleteNode(const py::object& nodeNumber, bool suppressWarnings)
{
	Index deleteItemNumber = EPyUtils::GetNodeIndexSafely(nodeNumber);
	SystemHasChanged();
	DeleteNode(deleteItemNumber, suppressWarnings);
	InteractiveModeActions();
}

//! Consistently deleta a MainNode from Python
void MainSystem::DeleteNode(Index deleteItemNumber, bool suppressWarnings)
{
	if (EXUstd::IndexIsInRange(deleteItemNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		//delete Node pointers:
		delete GetCSystem().GetSystemData().GetCNodes()[deleteItemNumber];
		delete GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationNodes()[deleteItemNumber];
		delete GetMainSystemData().GetMainNodes()[deleteItemNumber];

		//remove item from list
		GetCSystem().GetSystemData().GetCNodes().Remove(deleteItemNumber);
		GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationNodes().Remove(deleteItemNumber);
		GetMainSystemData().GetMainNodes().Remove(deleteItemNumber);

		//adapt standard names
		STDstring nodeStr = "node";

		for (Index i = deleteItemNumber; i < GetMainSystemData().GetMainNodes().NumberOfItems(); i++)
		{
			MainNode* node = GetMainSystemData().GetMainNodes()[i];
			if (node->GetName() == nodeStr + EXUstd::ToString(i + 1))
			{
				node->GetName() = nodeStr + EXUstd::ToString(i);
			}
		}

		//change indices in objects:
		Index cntObjects = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCObjects())
		{
			for (Index i = 0; i < item->GetNumberOfNodes(); i++)
			{
				if (item->GetNodeNumber(i) == deleteItemNumber)
				{
					if (!suppressWarnings) {
						PyWarning("DeleteNode: WARNING: Object with ID " +
							EXUstd::ToString(cntObjects) +
							" references to deleted node " +
							EXUstd::ToString(deleteItemNumber));
					}
					item->SetNodeNumber(i, EXUstd::InvalidIndex);
				}
				else if (item->GetNodeNumber(i) > deleteItemNumber)
				{
					item->SetNodeNumber(i, item->GetNodeNumber(i) - 1);
				}
			}
			cntObjects++;
		}

		//change indices in markers:
		Index cntMarkers = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCMarkers())
		{
			if (EXUstd::IsOfType(item->GetType(), Marker::Node)) //might also be Marker::Body
			{
				if (item->GetNodeNumber() == deleteItemNumber) //also works for InvalidIndex
				{
					if (!suppressWarnings) {
						PyWarning("DeleteNode: WARNING: Marker with ID " +
							EXUstd::ToString(cntMarkers) +
							" references to deleted node " +
							EXUstd::ToString(deleteItemNumber));
					}
					item->SetNodeNumber(EXUstd::InvalidIndex);
				}
				else if (item->GetNodeNumber() > deleteItemNumber)
				{
					item->SetNodeNumber(item->GetNodeNumber() - 1);
				}
			}
			cntMarkers++;
		}

		//change indices in sensors:
		Index cntSensors = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCSensors())
		{
			//pout << "sensor" << cntSensors << ": type = " << GetSensorTypeString(item->GetType()) << "\n";
			if (EXUstd::IsOfType(item->GetType(), SensorType::Node) )
			{
				if (item->GetNodeNumber() == deleteItemNumber)
				{
					if (!suppressWarnings) {
						PyWarning("DeleteNode: WARNING: Sensor with ID " +
							EXUstd::ToString(cntSensors) +
							" references to deleted node " +
							EXUstd::ToString(deleteItemNumber));
					}
					item->SetNodeNumber(EXUstd::InvalidIndex);
				}
				else if (item->GetNodeNumber() > deleteItemNumber)
				{
					item->SetNodeNumber(item->GetNodeNumber() - 1);
				}
			}
			cntSensors++;
		}

	}
	else
	{
		PyError(STDstring("MainSystem::DeleteNode: access to invalid node number ") + EXUstd::ToString(deleteItemNumber));
	}
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
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()) )
	{
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetNode: access to invalid node number ") + EXUstd::ToString(nodeNumber));
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
//		PyError(STDstring("MainSystem::GetNode: access to invalid node '") + nodeName + "'");
//		return py::dict();
//	}
//}

//! modify node's dictionary
void MainSystem::PyModifyNode(const py::object& itemIndex, py::dict nodeDict)
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetWithDictionary(nodeDict);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyNode: access to invalid node number ") + EXUstd::ToString(nodeNumber));
	}
}

////! modify node's dictionary
//void MainSystem::PyModifyNode(STDstring nodeName, py::dict d)
//{
//	Index nodeNumber = PyGetNodeNumber(nodeName);
//  if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
//	{
//		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetWithDictionary(d);
//	}
//	else
//	{
//		PyError(STDstring("ModifyNodeDictionary: access to invalid node '") + nodeName + "'");
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
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetNodeOutput", configuration, nodeNumber, ItemType::Node);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetNodeOutput", variableType, configuration, nodeNumber, ItemType::Node);

		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetOutputVariable(variableType, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeOutputVariable: access to invalid node number ") + EXUstd::ToString(nodeNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! get index in global ODE2 coordinate vector for first node coordinate
Index MainSystem::PyGetNodeODE2Index(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::ODE2variables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalODE2CoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeODE2Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + ": not an ODE2 node");
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeODE2Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)");
		return EXUstd::InvalidIndex;
	}
}

//! get index in global ODE1 coordinate vector for first node coordinate
Index MainSystem::PyGetNodeODE1Index(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::ODE1variables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalODE1CoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeODE1Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + ": not an ODE1 node");
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeODE1Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)");
		return EXUstd::InvalidIndex;
	}
}

//! get index in global AE coordinate vector for first node coordinate
Index MainSystem::PyGetNodeAEIndex(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::AEvariables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalAECoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeAEIndex: access to invalid node number ") + EXUstd::ToString(nodeNumber) + ": not an AE node");
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeAEIndex: access to invalid node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)");
		return EXUstd::InvalidIndex;
	}
}



////! call pybind object function, possibly with arguments; empty function, to be overwritten in specialized class
//py::object MainSystem::PyCallNodeFunction(Index nodeNumber, STDstring functionName, py::dict args)
//{
//	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
//	{
//		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->CallFunction(functionName, args);
//	}
//	else
//	{
//		PyError(STDstring("MainSystem::ModifyObject: access to invalid node number ") + EXUstd::ToString(nodeNumber));
//		return py::int_(EXUstd::InvalidIndex);
//		//return py::object();
//	}
//
//}


//! Get (read) parameter 'parameterName' of 'nodeNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetNodeParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeParameter: access to invalid node number ") + EXUstd::ToString(nodeNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'nodeNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetNodeParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index nodeNumber = EPyUtils::GetNodeIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetNodeParameter: access to invalid node number ") + EXUstd::ToString(nodeNumber));
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

//! Consistently deleta a MainObject from Python
void MainSystem::PyDeleteObject(const py::object& objectNumber, bool deleteDependentItems, bool suppressWarnings)
{
	Index deleteItemNumber = EPyUtils::GetObjectIndexSafely(objectNumber);
	if (EXUstd::IndexIsInRange(deleteItemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		SystemHasChanged();
		//if deleteDependentItems
		//collect nodes
		ArrayIndex dependentNodes;
		ArrayIndex dependentMarkers;
		if (deleteDependentItems) //use if here, as it may cause problems in some cases
		{
			for (Index i=0; i < GetCSystem().GetSystemData().GetCObjects()[deleteItemNumber]->GetNumberOfNodes(); i++)
			{
				dependentNodes.Append(GetCSystem().GetSystemData().GetCObjects()[deleteItemNumber]->GetNodeNumber(i));
			}

			//collect markers in case of constraints
			if (EXUstd::IsOfType(GetMainSystemData().GetMainObjects()[deleteItemNumber]->GetCObject()->GetType(), CObjectType::Connector))
			{
				CObjectConnector* connector = ((CObjectConnector*)GetCSystem().GetSystemData().GetCObjects()[deleteItemNumber]);
				dependentMarkers = connector->GetMarkerNumbers();
			}
		}

		//delete object pointers:
		delete GetCSystem().GetSystemData().GetCObjects()[deleteItemNumber];
		delete GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationObjects()[deleteItemNumber];
		delete GetMainSystemData().GetMainObjects()[deleteItemNumber];

		//remove item from list
		GetCSystem().GetSystemData().GetCObjects().Remove(deleteItemNumber);
		GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationObjects().Remove(deleteItemNumber);
		GetMainSystemData().GetMainObjects().Remove(deleteItemNumber);


		//adapt standard names
		STDstring objectStr = "object";

		for (Index i=deleteItemNumber; i < GetMainSystemData().GetMainObjects().NumberOfItems(); i++)
		{
			MainObject* object = GetMainSystemData().GetMainObjects()[i];
			if (object->GetName() == objectStr + EXUstd::ToString(i+1))
			{
				//pout << "RENAME:" << object->GetName() << " into " << objectStr + EXUstd::ToString(i) << "\n";
				object->GetName() = objectStr + EXUstd::ToString(i);
			}
		}

		//change indices in markers:
		Index cntMarkers = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCMarkers())
		{
			if (EXUstd::IsOfType(item->GetType(), Marker::Object)) //might also be Marker::Body
			{
				for (Index iLocal = 0; iLocal < item->GetNumberOfObjects(); iLocal++)
				{
					if (item->GetObjectNumber(iLocal) == deleteItemNumber)
					{
						if (!suppressWarnings) {
							PyWarning("DeleteObject: WARNING: Marker with ID " +
								EXUstd::ToString(cntMarkers) +
								" references to deleted object " +
								EXUstd::ToString(deleteItemNumber));
						}
						item->SetObjectNumber(EXUstd::InvalidIndex, iLocal);
					}
					else if (item->GetObjectNumber() > deleteItemNumber)
					{
						item->SetObjectNumber(item->GetObjectNumber() - 1, iLocal);
					}
				}
			}
			cntMarkers++;
		}

		//change indices in sensors:
		Index cntSensors = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCSensors())
		{
			if (item->HasObjectNumber())
			{
				if (item->GetObjectNumber() == deleteItemNumber)
				{
					if (!suppressWarnings) {
						PyWarning("DeleteObject: WARNING: Sensor with ID " +
							EXUstd::ToString(cntSensors) +
							" references to deleted object " +
							EXUstd::ToString(deleteItemNumber));
					}
					item->SetObjectNumber(EXUstd::InvalidIndex);
				}
				else if (item->GetObjectNumber() > deleteItemNumber)
				{
					item->SetObjectNumber(item->GetObjectNumber() - 1);
				}
			}
			cntSensors++;
		}

		if (deleteDependentItems)
		{
			//inplace sort; we need to erase items with highest index first
			EXUstd::QuickSort(dependentMarkers);
			EXUstd::QuickSort(dependentNodes);
			for (auto item : EXUstd::Reverse(dependentMarkers))
			{
				DeleteMarker(item, suppressWarnings);
			}
			for (auto item : EXUstd::Reverse(dependentNodes))
			{
				DeleteNode(item, suppressWarnings);
			}
		}
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::DeleteObject: access to invalid object number ") + EXUstd::ToString(deleteItemNumber));
	}
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
	if (EXUstd::IndexIsInRange(itemNumber,0,mainSystemData.GetMainObjects().NumberOfItems()) )
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetDictionary(addGraphicsData);
	}
	else
	{
		PyError(STDstring("MainSystem::GetObject: access to invalid object number ") + EXUstd::ToString(itemNumber));
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
//		PyError(STDstring("MainSystem::GetObject: access to invalid object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifyObject(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainObjects().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyObject: access to invalid object number ") + EXUstd::ToString(itemNumber));
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
//		PyError(STDstring("MainSystem::ModifyObject: access to invalid object number ") + EXUstd::ToString(itemNumber));
//		return py::int_(EXUstd::InvalidIndex);
//		//return py::object();
//	}
//}

//! Get specific output variable with variable type
py::object MainSystem::PyGetObjectOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
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
		PyError(STDstring("MainSystem::GetObjectOutputVariable: access to invalid object number ") + EXUstd::ToString(itemNumber));
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
		if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
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
			PyError(STDstring("MainSystem::GetObjectOutputVariableBody: access to invalid object number ") + EXUstd::ToString(itemNumber));
			return py::int_(EXUstd::InvalidIndex);
			//return py::object();
		}
}

//! get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType
py::object MainSystem::PyGetObjectOutputVariableSuperElement(const py::object& itemIndex, OutputVariableType variableType, 
	Index meshNodeNumber, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetObjectOutputSuperElement", configuration, itemNumber, ItemType::Object);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutputVariableSuperElement", variableType, configuration, itemNumber, ItemType::Object);
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariableSuperElement(variableType, meshNodeNumber, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::PyGetObjectOutputVariableSuperElement: access to invalid object number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! Get (read) parameter 'parameterName' of 'objectNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetObjectParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetObjectParameter: access to invalid object number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'objectNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetObjectParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetObjectIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		mainSystemData.GetMainObjects().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetObjectParameter: access to invalid object number ") + EXUstd::ToString(itemNumber));
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

//! Consistently delete a MainMarker from Python
void MainSystem::PyDeleteMarker(const py::object& markerNumber, bool suppressWarnings)
{
	Index deleteItemNumber = EPyUtils::GetMarkerIndexSafely(markerNumber);
	SystemHasChanged();
	DeleteMarker(deleteItemNumber, suppressWarnings);
	InteractiveModeActions();
}

//! Consistently delete a MainMarker from Python
void MainSystem::DeleteMarker(Index deleteItemNumber, bool suppressWarnings)
{
	if (EXUstd::IndexIsInRange(deleteItemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		//delete object pointers:
		delete GetCSystem().GetSystemData().GetCMarkers()[deleteItemNumber];
		delete GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationMarkers()[deleteItemNumber];
		delete GetMainSystemData().GetMainMarkers()[deleteItemNumber];

		//remove item from list
		GetCSystem().GetSystemData().GetCMarkers().Remove(deleteItemNumber);
		GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationMarkers().Remove(deleteItemNumber);
		GetMainSystemData().GetMainMarkers().Remove(deleteItemNumber);

		//adapt standard names
		STDstring markerStr = "marker";
		for (Index i = deleteItemNumber; i < GetMainSystemData().GetMainMarkers().NumberOfItems(); i++)
		{
			MainMarker* marker = GetMainSystemData().GetMainMarkers()[i];
			if (marker->GetName() == markerStr + EXUstd::ToString(i + 1))
			{
				//pout << "RENAME:" << marker->GetName() << " into " << markerStr + EXUstd::ToString(i) << "\n";
				marker->GetName() = markerStr + EXUstd::ToString(i);
			}
		}

		//change indices in connectors:
		Index cntObjects = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCObjects())
		{
			if (EXUstd::IsOfType(item->GetType(), CObjectType::Connector))
			{
				CObjectConnector* connector = (CObjectConnector*)item;
				const ArrayIndex& markerNumbers = connector->GetMarkerNumbers();
				for (Index i=0; i < markerNumbers.NumberOfItems(); i++)
				{
					Index marker = markerNumbers[i];
					if (marker == deleteItemNumber)
					{
						if (!suppressWarnings) {
							PyWarning("DeleteMarker: WARNING: Object with ID " +
								EXUstd::ToString(cntObjects) +
								" references to deleted Marker " +
								EXUstd::ToString(deleteItemNumber));
						}
						connector->GetMarkerNumbers()[i] = EXUstd::InvalidIndex;
					}
					else if (marker > deleteItemNumber)
					{
						//pout << "  object " << cntObjects << ": change marker number " << marker << " to " << marker - 1 << "\n";
						connector->GetMarkerNumbers()[i] = marker - 1;
					}
				}
			}
			cntObjects++;
		}

		//change indices in loads:
		Index cntLoads = 0;
		for (CLoad* item : GetCSystem().GetSystemData().GetCLoads())
		{
			if (item->GetMarkerNumber() == deleteItemNumber)
			{
				if (!suppressWarnings) {
					PyWarning("DeleteMarker: WARNING: Load with ID " +
						EXUstd::ToString(deleteItemNumber) +
						" references to deleted Marker " +
						EXUstd::ToString(deleteItemNumber));
				}
				//pout << "Load " << cntLoads << ": deleted marker " << item->GetMarkerNumber() << " is invalid\n";
				item->SetMarkerNumber(EXUstd::InvalidIndex);
			}
			else if (item->GetMarkerNumber() > deleteItemNumber)
			{
				//pout << "  load " << cntLoads << ": change marker number " << item->GetMarkerNumber() << " to " << item->GetMarkerNumber() - 1 << "\n";
				item->SetMarkerNumber(item->GetMarkerNumber()-1);
			}
			cntLoads++;
		}


		//change marker indices in sensors:
		Index cntSensors = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCSensors())
		{
			//pout << "sensor" << cntSensors << ": type = " << GetSensorTypeString(item->GetType()) << "\n";
			if (EXUstd::IsOfType(item->GetType(), SensorType::Marker))
			{
				if (item->GetMarkerNumber() == deleteItemNumber)
				{
					if (!suppressWarnings) {
						PyWarning("DeleteObject: WARNING: Sensor with ID " +
							EXUstd::ToString(cntSensors) +
							" references to deleted marker " +
							EXUstd::ToString(deleteItemNumber));
					}
					item->SetMarkerNumber(EXUstd::InvalidIndex);
				}
				else if (item->GetMarkerNumber() > deleteItemNumber)
				{
					//pout << "  sensor" << cntSensors << ": change marker " << item->GetMarkerNumber() << " to " << item->GetMarkerNumber() - 1 << "\n";
					item->SetMarkerNumber(item->GetMarkerNumber() - 1);
				}
			}
			cntSensors++;
		}
		//possibly object numbers also in other data structures (visualization, etc.?)

	}
	else
	{
		PyError(STDstring("MainSystem::DeleteMarker: access to invalid marker number ") + EXUstd::ToString(deleteItemNumber));
	}
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
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()) )
	{
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarker: access to invalid marker number ") + EXUstd::ToString(itemNumber));
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
//		PyError(STDstring("MainSystem::GetMarker: access to invalid object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifyMarker(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainMarkers().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyMarker: access to invalid marker number ") + EXUstd::ToString(itemNumber));
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
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerParameter: access to invalid marker number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'markerNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetMarkerParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		mainSystemData.GetMainMarkers().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetMarkerParameter: access to invalid marker number ") + EXUstd::ToString(itemNumber));
	}
}

//! Get specific output variable with variable type
py::object MainSystem::PyGetMarkerOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::GetMarkerIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetMarkerOutput", configuration, itemNumber, ItemType::Marker);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutputVariableSuperElement", variableType, configuration, itemNumber, ItemType::Marker);

		//the marker function itself will raise an error, if it is not able to return the according output variable
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetOutputVariable(GetCSystem().GetSystemData(), variableType, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerOutput: access to invalid marker number ") + EXUstd::ToString(itemNumber));
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

//! Consistently delete a MainLoad from Python
void MainSystem::PyDeleteLoad(const py::object& loadNumber, bool deleteDependentMarkers, bool suppressWarnings)
{
	Index deleteItemNumber = EPyUtils::GetLoadIndexSafely(loadNumber);
	SystemHasChanged();
	DeleteLoad(deleteItemNumber, deleteDependentMarkers, suppressWarnings);
	InteractiveModeActions();
}

//! Consistently delete a MainLoad from Python
void MainSystem::DeleteLoad(Index deleteItemNumber, bool deleteDependentMarkers, bool suppressWarnings)
{
	if (EXUstd::IndexIsInRange(deleteItemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		//save marker from load
		Index dependentMarkerNumber = GetCSystem().GetSystemData().GetCLoads()[deleteItemNumber]->GetMarkerNumber();

		//delete object pointers:
		delete GetCSystem().GetSystemData().GetCLoads()[deleteItemNumber];
		delete GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationLoads()[deleteItemNumber];
		delete GetMainSystemData().GetMainLoads()[deleteItemNumber];

		//remove item from list
		GetCSystem().GetSystemData().GetCLoads().Remove(deleteItemNumber);
		GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationLoads().Remove(deleteItemNumber);
		GetMainSystemData().GetMainLoads().Remove(deleteItemNumber);

		//adapt standard names
		STDstring loadStr = "load";
		for (Index i = deleteItemNumber; i < GetMainSystemData().GetMainLoads().NumberOfItems(); i++)
		{
			MainLoad* load = GetMainSystemData().GetMainLoads()[i];
			if (load->GetName() == loadStr + EXUstd::ToString(i + 1))
			{
				//pout << "RENAME:" << load->GetName() << " into " << loadStr + EXUstd::ToString(i) << "\n";
				load->GetName() = loadStr + EXUstd::ToString(i);
			}
		}

		//change marker indices in sensors:
		Index cntSensors = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCSensors())
		{
			if (EXUstd::IsOfType(item->GetType(), SensorType::Load))
			{
				if (item->GetLoadNumber() == deleteItemNumber)
				{
					if (!suppressWarnings) {
						PyWarning("DeleteSensor: WARNING: Sensor with ID " +
							EXUstd::ToString(cntSensors) +
							" references to deleted load " +
							EXUstd::ToString(deleteItemNumber));
					}
					item->SetLoadNumber(EXUstd::InvalidIndex);
				}
				else if (item->GetLoadNumber() > deleteItemNumber)
				{
					//pout << "  sensor" << cntSensors << ": change load " << item->GetLoadNumber() << " to " << item->GetLoadNumber() - 1 << "\n";
					item->SetLoadNumber(item->GetLoadNumber() - 1);
				}
			}
			cntSensors++;
		}

		if (deleteDependentMarkers)
		{
			DeleteMarker(dependentMarkerNumber);
		}

	}
	else
	{
		PyError(STDstring("MainSystem::DeleteLoad: access to invalid load number ") + EXUstd::ToString(deleteItemNumber));
	}
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
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()) )
	{
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoad: access to invalid load number ") + EXUstd::ToString(itemNumber));
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
//		PyError(STDstring("MainSystem::GetLoad: access to invalid object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifyLoad(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainLoads().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyLoad: access to invalid load number ") + EXUstd::ToString(itemNumber));
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
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistent("GetLoadValues", itemNumber, ItemType::Load);
		Real t = GetCSystem().GetSystemData().GetCData().GetCurrent().GetTime(); //only current time available
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetLoadValues(GetCSystem().GetSystemData().GetMainSystemBacklink(), t);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadValues: access to invalid load number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! Get (read) parameter 'parameterName' of 'loadNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetLoadParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadParameter: access to invalid load number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'loadNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetLoadParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetLoadIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		mainSystemData.GetMainLoads().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetLoadParameter: access to invalid load number ") + EXUstd::ToString(itemNumber));
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

//! Consistently delete a MainSensor from Python
void MainSystem::PyDeleteSensor(const py::object& sensorNumber, bool suppressWarnings)
{
	Index deleteItemNumber = EPyUtils::GetSensorIndexSafely(sensorNumber);
	SystemHasChanged();
	DeleteSensor(deleteItemNumber, suppressWarnings);
	InteractiveModeActions();
}

//! Consistently delete a MainSensor from Python
void MainSystem::DeleteSensor(Index deleteItemNumber, bool suppressWarnings)
{
	if (EXUstd::IndexIsInRange(deleteItemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		//delete object pointers:
		delete GetCSystem().GetSystemData().GetCSensors()[deleteItemNumber];
		delete GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationSensors()[deleteItemNumber];
		delete GetMainSystemData().GetMainSensors()[deleteItemNumber];

		//remove item from list
		GetCSystem().GetSystemData().GetCSensors().Remove(deleteItemNumber);
		GetVisualizationSystem().GetVisualizationSystemData().GetVisualizationSensors().Remove(deleteItemNumber);
		GetMainSystemData().GetMainSensors().Remove(deleteItemNumber);

		//adapt standard names
		STDstring sensorStr = "sensor";
		for (Index i = deleteItemNumber; i < GetMainSystemData().GetMainSensors().NumberOfItems(); i++)
		{
			MainSensor* sensor = GetMainSystemData().GetMainSensors()[i];
			if (sensor->GetName() == sensorStr + EXUstd::ToString(i + 1))
			{
				//pout << "RENAME:" << sensor->GetName() << " into " << sensorStr + EXUstd::ToString(i) << "\n";
				sensor->GetName() = sensorStr + EXUstd::ToString(i);
			}
		}

		//change marker indices in sensors:
		Index cntSensors = 0;
		for (auto* item : GetCSystem().GetSystemData().GetCSensors())
		{
			if (EXUstd::IsOfType(item->GetType(), SensorType::UserFunction))
			{
				for (Index i = 0; i < item->GetNumberOfSensors(); i++)
				{
					if (item->GetSensorNumber(i) == deleteItemNumber)
					{
						if (!suppressWarnings) {
							PyWarning("DeleteSensor: WARNING: Sensor with ID " +
								EXUstd::ToString(cntSensors) + " [" + EXUstd::ToString(i) + "]" +
								" references to deleted sensor " +
								EXUstd::ToString(deleteItemNumber));
						}
						item->SetSensorNumber(i, EXUstd::InvalidIndex);
					}
					else if (item->GetSensorNumber(i) > deleteItemNumber)
					{
						pout << "  sensor" << cntSensors << ": change sensor " << item->GetSensorNumber(i) << " [" << EXUstd::ToString(i) <<  "]" << " to " << item->GetSensorNumber(i) - 1 << "\n";
						item->SetSensorNumber(i, item->GetSensorNumber(i) - 1);
					}
				}
			}
			cntSensors++;
		}

	}
	else
	{
		PyError(STDstring("MainSystem::DeleteSensor: access to invalid sensor number ") + EXUstd::ToString(deleteItemNumber));
	}
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
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()) )
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensor: access to invalid sensor number ") + EXUstd::ToString(itemNumber));
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
//		PyError(STDstring("MainSystem::GetSensor: access to invalid object '") + itemName + "'");
//		return py::dict();
//	}
//}

//! modify object's dictionary
void MainSystem::PyModifySensor(const py::object& itemIndex, py::dict d)
{
	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifySensor: access to invalid sensor number ") + EXUstd::ToString(itemNumber));
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
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetSensorValues", configuration, itemNumber, ItemType::Sensor);
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetSensorValues(GetCSystem().GetSystemData(), configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorValues: access to invalid sensor number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! get sensor's stored data (if it exists ...)
py::array_t<Real> MainSystem::PyGetSensorStoredData(const py::object& itemIndex)
{

	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
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
		PyError(STDstring("MainSystem::GetSensorStoredData: access to invalid sensor number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}



//! Get (read) parameter 'parameterName' of 'sensorNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetSensorParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorParameter: access to invalid sensor number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'SensorNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetSensorParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::GetSensorIndexSafely(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetSensorParameter: access to invalid sensor number ") + EXUstd::ToString(itemNumber));
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

