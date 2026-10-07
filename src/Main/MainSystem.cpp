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
				PyError(STDstring("GetDictionary (pickle/copy): MainSystem object '") + item->GetName() + "' has a user function which cannot be copied!", PyErrorType::modelError);
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
			", but loaded dictionary has been built with version " + py::cast<STDstring>(d["__version__"])+"; you can disable this exception in exudyn.special.exceptions", PyErrorType::valueError);
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
		PyError("MainSystem::GeneralContact: access to invalid index " + EXUstd::ToString(generalContactNumber), PyErrorType::indexError);
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
		PyError("MainSystem::DeleteGeneralContact: access to invalid index " + EXUstd::ToString(generalContactNumber), PyErrorType::indexError);
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
	if (deprecationWarning) { PyDeprecated("functions", "MainSystem.WaitForUserToContinue", "MainSystem.WaitForUserToContinue(): function is deprecated; for SystemContainer SC use set SC.renderer.DoIdleTasks() instead"); }

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
	//a parameter error that named its Python exception type keeps it; its message was
	//already printed where it was raised (#2432).
	//NOTE: this MUST come before catch(EXUexception), which is std::runtime_error and would
	//otherwise catch it first (ReleaseAssert.h:30) - and the same holds for the Exudyn exception
	//classes, which derive from EXUexception for exactly that reason (#2516)
	catch (const py::builtin_exception&)
	{
		throw;
	}
	catch (const ExudynError&)
	{
		throw;
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
	Index deleteItemNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(nodeNumber);
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
		PyError(STDstring("MainSystem::DeleteNode: access to invalid node number ") + EXUstd::ToString(deleteItemNumber), PyErrorType::indexError);
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
	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()) )
	{
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetNode: access to invalid node number ") + EXUstd::ToString(nodeNumber), PyErrorType::indexError);
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
	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetWithDictionary(nodeDict);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyNode: access to invalid node number ") + EXUstd::ToString(nodeNumber), PyErrorType::indexError);
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
		PyError(STDstring("MainSystem::GetNodeDefaults: typeName needed'"), PyErrorType::valueError);
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
		PyError(STDstring("MainSystem::GetNodeDefaults: unknown node type '") + typeName + "'", PyErrorType::valueError);
	}
	return d;
}

py::object MainSystem::PyGetNodeOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{

	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetNodeOutput", configuration, nodeNumber, ItemType::Node);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetNodeOutput", variableType, configuration, nodeNumber, ItemType::Node);

		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetOutputVariable(variableType, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeOutputVariable: access to invalid node number ") + EXUstd::ToString(nodeNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! get index in global ODE2 coordinate vector for first node coordinate
Index MainSystem::PyGetNodeODE2Index(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::ODE2variables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalODE2CoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeODE2Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + ": not an ODE2 node", PyErrorType::indexError);
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeODE2Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)", PyErrorType::indexError);
		return EXUstd::InvalidIndex;
	}
}

//! get index in global ODE1 coordinate vector for first node coordinate
Index MainSystem::PyGetNodeODE1Index(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::ODE1variables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalODE1CoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeODE1Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + ": not an ODE1 node", PyErrorType::indexError);
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeODE1Index: access to invalid node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)", PyErrorType::indexError);
		return EXUstd::InvalidIndex;
	}
}

//! get index in global AE coordinate vector for first node coordinate
Index MainSystem::PyGetNodeAEIndex(const py::object& itemIndex) const
{
	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		if (EXUstd::IsOfType(mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup(), CNodeGroup::AEvariables)) //CNodeRigidBodyEP also has AEvariables
		{
			return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetGlobalAECoordinateIndex();
		}
		else
		{
			PyError(STDstring("MainSystem::GetNodeAEIndex: access to invalid node number ") + EXUstd::ToString(nodeNumber) + ": not an AE node", PyErrorType::indexError);
			return EXUstd::InvalidIndex;
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeAEIndex: access to invalid node number ") + EXUstd::ToString(nodeNumber) + " (index does not exist)", PyErrorType::indexError);
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
	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetNodeParameter: access to invalid node number ") + EXUstd::ToString(nodeNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'nodeNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetNodeParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index nodeNumber = EPyUtils::ItemIndexFromPython<NodeIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(nodeNumber, 0, mainSystemData.GetMainNodes().NumberOfItems()))
	{
		mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetNodeParameter: access to invalid node number ") + EXUstd::ToString(nodeNumber), PyErrorType::indexError);
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
	//a parameter error that named its Python exception type keeps it; its message was
	//already printed where it was raised (#2432).
	//NOTE: this MUST come before catch(EXUexception), which is std::runtime_error and would
	//otherwise catch it first (ReleaseAssert.h:30) - and the same holds for the Exudyn exception
	//classes, which derive from EXUexception for exactly that reason (#2516)
	catch (const py::builtin_exception&)
	{
		throw;
	}
	catch (const ExudynError&)
	{
		throw;
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
	Index deleteItemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(objectNumber);
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
		PyError(STDstring("MainSystem::DeleteObject: access to invalid object number ") + EXUstd::ToString(deleteItemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber,0,mainSystemData.GetMainObjects().NumberOfItems()) )
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetDictionary(addGraphicsData);
	}
	else
	{
		PyError(STDstring("MainSystem::GetObject: access to invalid object number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainObjects().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyObject: access to invalid object number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
	}
}

//! get object's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetObjectDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetObjectDefaults: typeName needed'"), PyErrorType::valueError);
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
		PyError(STDstring("MainSystem::GetObjectDefaults: unknown object type '") + typeName + "'", PyErrorType::valueError);
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
//! the single flags set in mask, as the members of the Python enumeration of TEnum (#2203); members that are not one bit
//! (_None) are left out, so a combination of C++ flags becomes the list a script compares against
template<class TEnum>
py::list InspectFlags(Index64 mask)
{
	py::list result;
	if (mask < 0 && mask >= INT32_MIN) { mask &= 0xFFFFFFFF; }
	py::dict members = py::type::of<TEnum>().attr("__members__");
	for (auto member : members)
	{
		Index64 value = (Index64)(member.second.cast<TEnum>());
		if (value < 0 && value >= INT32_MIN) { value &= 0xFFFFFFFF; } //a flag in bit 31 of a 32-bit enumeration, e.g. AccessFunctionType
		if (value > 0 && (value & (value - 1)) == 0 && (mask & value) == value) { result.append(member.second); }
	}
	return result;
}

//! mbs.ComputeItem (#2779): what an item computes, at the current state; the typed index says the kind of item, as for
//! mbs.Inspect, and the computation goes through the functions the solver uses
py::object MainSystem::PyComputeItem(const py::object& itemIndex, const py::object& what, const std::vector<Real>& localPositionList,
	const py::object& vector)
{
	const MainSystemData& data = GetMainSystemData();
	ItemType itemType = ItemType::_None;
	Index number = EXUstd::InvalidIndex;
	Index numberOfItems = 0;
	if (py::isinstance<ObjectIndex>(itemIndex)) { itemType = ItemType::Object; number = py::cast<ObjectIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainObjects().NumberOfItems(); }
	else if (py::isinstance<NodeIndex>(itemIndex)) { itemType = ItemType::Node; number = py::cast<NodeIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainNodes().NumberOfItems(); }
	else if (py::isinstance<MarkerIndex>(itemIndex)) { itemType = ItemType::Marker; number = py::cast<MarkerIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainMarkers().NumberOfItems(); }
	else
	{
		PyError(STDstring("ComputeItem: itemIndex must be the typed index of an object, node or marker - an ObjectIndex, NodeIndex or MarkerIndex, as mbs.AddObject(...) and the other Add functions return it; got ")
			+ STDstring(py::str(itemIndex)), PyErrorType::typeError);
		return py::none();
	}
	if (!EXUstd::IndexIsInRange(number, 0, numberOfItems))
	{
		PyError("ComputeItem: " + EXUstd::ToString(itemType) + " number " + EXUstd::ToString(number) + " does not exist", PyErrorType::indexError);
		return py::none();
	}
	data.RaiseIfNotConsistent("ComputeItem", number, itemType);
	CHECKandTHROW(localPositionList.size() == 3, "ComputeItem: localPosition must have 3 components");
	const Vector3D localPosition({ localPositionList[0], localPositionList[1], localPositionList[2] });
	CSystemData& cSystemData = GetCSystem().GetSystemData();

	//what applies to this item
	std::vector<ComputeItemType> applicable;
	CObject* object = nullptr;
	CNodeODE2* node = nullptr;
	const CMarker* marker = nullptr;
	if (itemType == ItemType::Object)
	{
		object = cSystemData.GetCObjects()[number];
		if (EXUstd::IsOfType(object->GetType(), CObjectType::Body))
		{
			Index access = (Index)object->GetAccessFunctionTypes();
			//a superelement or kinematic tree is reached through its own markers, not at a local position of the body
			const bool atLocalPosition = !(access & ((Index)AccessFunctionType::SuperElement + (Index)AccessFunctionType::KinematicTree
				+ (Index)AccessFunctionType::OwnMarkersOnly));
			if (atLocalPosition && (access & (Index)AccessFunctionType::TranslationalVelocity_qt)) { applicable.push_back(ComputeItemType::PositionJacobian); }
			if (atLocalPosition && (access & (Index)AccessFunctionType::AngularVelocity_qt)) { applicable.push_back(ComputeItemType::RotationJacobian); }
			if (atLocalPosition && (access & (Index)AccessFunctionType::JacobianTtimesVector_q)) { applicable.push_back(ComputeItemType::JacobianTTimesVectorDerivative); }
			if (access & (Index)AccessFunctionType::DisplacementMassIntegral_q) { applicable.push_back(ComputeItemType::MassWeightedPositionJacobian); }
			applicable.push_back(ComputeItemType::ODE2LHS);
			applicable.push_back(ComputeItemType::MassMatrix);
		}
		else if (EXUstd::IsOfType(object->GetType(), CObjectType::Connector)
			&& !EXUstd::IsOfType(object->GetType(), CObjectType::Constraint))
		{
			applicable.push_back(ComputeItemType::ODE2LHS);
		}
		Index jacobianTypes = (Index)object->GetAvailableJacobians();
		if (!EXUstd::IsOfType(object->GetType(), CObjectType::Constraint))
		{
			if (jacobianTypes & (Index)JacobianType::ODE2_ODE2_function) { applicable.push_back(ComputeItemType::JacobianODE2); }
			if (jacobianTypes & (Index)JacobianType::ODE2_ODE2_t_function) { applicable.push_back(ComputeItemType::JacobianODE2_t); }
		}
		if (object->GetAlgebraicEquationsSize() != 0)
		{
			applicable.push_back(ComputeItemType::AlgebraicEquations);
			applicable.push_back(ComputeItemType::ConstraintJacobian);
			applicable.push_back(ComputeItemType::ReactionForces);
		}
	}
	else if (itemType == ItemType::Node)
	{
		CNode* cNode = cSystemData.GetCNodes()[number];
		if ((Index)cNode->GetNodeGroup() & (Index)CNodeGroup::ODE2variables)
		{
			node = (CNodeODE2*)cNode;
			if ((Index)node->GetType() & ((Index)Node::Position + (Index)Node::Position2D)) { applicable.push_back(ComputeItemType::PositionJacobian); }
			if ((Index)node->GetType() & ((Index)Node::Orientation + (Index)Node::Orientation2D))
			{
				applicable.push_back(ComputeItemType::RotationJacobian);
				if ((Index)node->GetType() & (Index)Node::Orientation) { applicable.push_back(ComputeItemType::JacobianTTimesVectorDerivative); }
			}
		}
	}
	else
	{
		marker = cSystemData.GetCMarkers()[number];
		Index type = (Index)marker->GetType();
		const bool special = type & ((Index)Marker::BodyMass + (Index)Marker::Beam3DShape + (Index)Marker::Coordinates);
		if (!special)
		{
			applicable.push_back(ComputeItemType::Kinematics);
			if (type & (Index)Marker::Position) { applicable.push_back(ComputeItemType::PositionJacobian); }
			if (type & (Index)Marker::Orientation) { applicable.push_back(ComputeItemType::RotationJacobian); }
			if (type & (Index)Marker::Coordinate) { applicable.push_back(ComputeItemType::CoordinateJacobian); }
			if (type & (Index)Marker::JacobianDerivativeAvailable) { applicable.push_back(ComputeItemType::JacobianTTimesVectorDerivative); }
		}
	}

	py::list applicableList;
	STDstring applicableNames;
	for (ComputeItemType t : applicable)
	{
		applicableList.append(py::cast(t));
		applicableNames += (applicableNames.size() ? ", " : "") + STDstring("ComputeItemType.") + EXUstd::ToString(t);
	}
	if (what.is_none()) { return applicableList; }
	if (!py::isinstance<ComputeItemType>(what))
	{
		PyError("ComputeItem: what must be a member of exu.ComputeItemType or None; got " + STDstring(py::str(what)), PyErrorType::typeError);
		return py::none();
	}
	ComputeItemType computeType = py::cast<ComputeItemType>(what);
	if (std::find(applicable.begin(), applicable.end(), computeType) == applicable.end())
	{
		PyError("ComputeItem: ComputeItemType." + EXUstd::ToString(computeType) + " does not apply to " + EXUstd::ToString(itemType) + " "
			+ EXUstd::ToString(number) + "; what applies: [" + applicableNames + "]", PyErrorType::valueError);
		return py::none();
	}

	//the vector of a Jacobian derivative
	Vector6D forceTorque(0.);
	if (computeType == ComputeItemType::JacobianTTimesVectorDerivative)
	{
		std::vector<Real> values = vector.is_none() ? std::vector<Real>() : py::cast<std::vector<Real>>(vector);
		const size_t size = (itemType == ItemType::Node) ? 3 : 6;
		if (values.size() != size)
		{
			PyError("ComputeItem: JacobianTTimesVectorDerivative needs vector with " + EXUstd::ToString((Index)size)
				+ (size == 3 ? " values, the torque" : " values, the force and the torque"), PyErrorType::valueError);
			return py::none();
		}
		for (size_t i = 0; i < size; i++) { forceTorque[(Index)(i + 6 - size)] = values[i]; }
	}

	TemporaryComputationData temp;
	Matrix matrix;
	Vector local;
	if (itemType == ItemType::Object)
	{
		//a body only for a body: the cases that use it apply to bodies alone, and casting a connector to a body is
		//undefined behaviour even when the pointer is never used (#2892)
		const CObjectBody* body = EXUstd::IsOfType(object->GetType(), CObjectType::Body) ? static_cast<const CObjectBody*>(object) : nullptr;
		switch (computeType)
		{
		case ComputeItemType::PositionJacobian: body->GetPositionJacobian(localPosition, matrix); return EPyUtils::ToPython(matrix);
		case ComputeItemType::RotationJacobian: body->GetRotationJacobian(localPosition, matrix); return EPyUtils::ToPython(matrix);
		case ComputeItemType::JacobianTTimesVectorDerivative:
			if (!body->GetJacobianTransposedTimesVectorDerivative(localPosition, forceTorque, matrix)) { matrix.SetNumberOfRowsAndColumns(0, 0); }
			return EPyUtils::ToPython(matrix);
		case ComputeItemType::MassWeightedPositionJacobian: body->GetMassWeightedPositionJacobian(matrix); return EPyUtils::ToPython(matrix);
		case ComputeItemType::ODE2LHS:
			GetCSystem().ComputeObjectODE2LHS(temp, object, local, number);
			return EPyUtils::ToPython(local);
		case ComputeItemType::MassMatrix:
			temp.massMatrix.SetUseDenseMatrix(true);
			body->ComputeMassMatrix(temp.massMatrix, cSystemData.GetLocalToGlobalODE2()[number], number);
			return EPyUtils::ToPython(temp.massMatrix.GetInternalDenseMatrix());
		case ComputeItemType::AlgebraicEquations:
			GetCSystem().ComputeObjectAlgebraicEquations(temp, number, local);
			return EPyUtils::ToPython(local);
		case ComputeItemType::ConstraintJacobian:
		{
			bool usesVelocityLevel;
			JacobianType::Type filledJacobians;
			GetCSystem().ComputeObjectJacobianAE(number, temp, usesVelocityLevel, filledJacobians);
			if ((filledJacobians & JacobianType::AE_ODE2) && !usesVelocityLevel) { return EPyUtils::ToPython(temp.localJacobianAE_ODE2); }
			if (filledJacobians & JacobianType::AE_ODE2_t) { return EPyUtils::ToPython(temp.localJacobianAE_ODE2_t); }
			matrix.SetNumberOfRowsAndColumns(0, 0); //an inactive constraint: no Jacobian by the coordinates
			return EPyUtils::ToPython(matrix);
		}
		case ComputeItemType::ReactionForces:
			GetCSystem().ComputeObjectReactionForces(temp, number, local);
			return EPyUtils::ToPython(local);
		case ComputeItemType::JacobianODE2:
		case ComputeItemType::JacobianODE2_t:
		{
			const bool byVelocities = computeType == ComputeItemType::JacobianODE2_t;
			NumericalDifferentiationSettings numDiff; //the defaults: analytic Jacobians where available
			temp.jacobianODE2Container.SetUseDenseMatrix(true);
			if (!GetCSystem().ComputeObjectJacobianODE2(temp, numDiff, number, byVelocities ? 0. : 1., byVelocities ? 1. : 0.))
			{
				PyError("ComputeItem: " + EXUstd::ToString(itemType) + " " + EXUstd::ToString(number) + " has no analytic Jacobian here (a marker without the derivative of its Jacobian); "
					"exudyn.advancedUtilities.NumericalJacobian of ComputeItemType.ODE2LHS gives the numerical one", PyErrorType::valueError);
				return py::none();
			}
			if (temp.jacobianODE2Container.UseDenseMatrix()) { return EPyUtils::ToPython(temp.jacobianODE2Container.GetInternalDenseMatrix()); }
			//sparse, in global indices: back to the coordinates of the object
			const ArrayIndex& ltg = cSystemData.GetLocalToGlobalODE2()[number];
			matrix.SetNumberOfRowsAndColumns(ltg.NumberOfItems(), ltg.NumberOfItems());
			matrix.SetAll(0.);
			for (const EXUmath::Triplet& item : temp.jacobianODE2Container.GetInternalSparseTripletMatrix().GetTriplets())
			{
				Index row = ltg.GetIndexOfItem(item.row());
				Index column = ltg.GetIndexOfItem(item.col());
				CHECKandTHROW(row != EXUstd::InvalidIndex && column != EXUstd::InvalidIndex, "ComputeItem: JacobianODE2 outside the coordinates of the object");
				matrix(row, column) += item.value();
			}
			return EPyUtils::ToPython(matrix);
		}
		default: break;
		}
	}
	else if (itemType == ItemType::Node)
	{
		switch (computeType)
		{
		case ComputeItemType::PositionJacobian: node->GetPositionJacobian(matrix); return EPyUtils::ToPython(matrix);
		case ComputeItemType::RotationJacobian: node->GetRotationJacobian(matrix); return EPyUtils::ToPython(matrix);
		case ComputeItemType::JacobianTTimesVectorDerivative:
			node->GetRotationJacobianTTimesVector_q(Vector3D({ forceTorque[3], forceTorque[4], forceTorque[5] }), matrix);
			return EPyUtils::ToPython(matrix);
		default: break;
		}
	}
	else
	{
		TemporaryMarkerDataStructure temporary;
		temporary.Get().SetNumberOfMarkerData(1);
		MarkerData& markerData = temporary.Get().GetMarkerData(0);
		marker->ComputeMarkerData(cSystemData, computeType != ComputeItemType::Kinematics, markerData);
		switch (computeType)
		{
		case ComputeItemType::Kinematics:
		{
			py::dict kinematics;
			Index type = (Index)marker->GetType();
			if (type & (Index)Marker::Position)
			{
				kinematics["position"] = EPyUtils::ToPython(markerData.position);
				kinematics["velocity"] = EPyUtils::ToPython(markerData.velocity);
			}
			if (type & (Index)Marker::Orientation)
			{
				kinematics["rotationMatrix"] = EPyUtils::ToPython(markerData.orientation);
				kinematics["angularVelocityLocal"] = EPyUtils::ToPython(markerData.angularVelocityLocal);
			}
			if (type & (Index)Marker::Coordinate)
			{
				kinematics["value"] = EPyUtils::ToPython(markerData.vectorValue);
				kinematics["value_t"] = EPyUtils::ToPython(markerData.vectorValue_t);
			}
			return kinematics;
		}
		case ComputeItemType::PositionJacobian: return EPyUtils::ToPython(markerData.positionJacobian);
		case ComputeItemType::RotationJacobian: return EPyUtils::ToPython(markerData.rotationJacobian);
		case ComputeItemType::CoordinateJacobian: return EPyUtils::ToPython(markerData.jacobian);
		case ComputeItemType::JacobianTTimesVectorDerivative:
			marker->ComputeMarkerDataJacobianDerivative(cSystemData, forceTorque, markerData);
			return EPyUtils::ToPython(markerData.jacobianDerivative);
		default: break;
		}
	}
	return py::none();
}

//! mbs.Inspect (#2203): the typed index says the kind of item; the answers are lists of the exported enumerations
py::object MainSystem::PyInspect(const py::object& itemIndex, const py::object& what) const
{
	const MainSystemData& data = GetMainSystemData();
	ItemType itemType = ItemType::_None;
	Index number = EXUstd::InvalidIndex;
	Index numberOfItems = 0;
	if (py::isinstance<ObjectIndex>(itemIndex)) { itemType = ItemType::Object; number = py::cast<ObjectIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainObjects().NumberOfItems(); }
	else if (py::isinstance<NodeIndex>(itemIndex)) { itemType = ItemType::Node; number = py::cast<NodeIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainNodes().NumberOfItems(); }
	else if (py::isinstance<MarkerIndex>(itemIndex)) { itemType = ItemType::Marker; number = py::cast<MarkerIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainMarkers().NumberOfItems(); }
	else if (py::isinstance<LoadIndex>(itemIndex)) { itemType = ItemType::Load; number = py::cast<LoadIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainLoads().NumberOfItems(); }
	else if (py::isinstance<SensorIndex>(itemIndex)) { itemType = ItemType::Sensor; number = py::cast<SensorIndex>(itemIndex).GetIndex(); numberOfItems = data.GetMainSensors().NumberOfItems(); }
	else
	{
		PyError(STDstring("Inspect: itemIndex must be the typed index of an item - an ObjectIndex, NodeIndex, MarkerIndex, LoadIndex or SensorIndex, as mbs.AddObject(...) and the other Add functions return it - because the type says the kind of item; got ")
			+ STDstring(py::str(itemIndex)) + " of type " + STDstring(py::str(itemIndex.get_type().attr("__name__"))), PyErrorType::typeError);
		return py::none();
	}
	if (!EXUstd::IndexIsInRange(number, 0, numberOfItems))
	{
		PyError("Inspect: " + EXUstd::ToString(itemType) + " number " + EXUstd::ToString(number) + " does not exist", PyErrorType::indexError);
		return py::none();
	}

	//what applies to this item
	std::vector<InspectType> applicable;
	const CObject* object = nullptr;
	if (itemType == ItemType::Object)
	{
		object = data.GetMainObjects()[number]->GetCObject();
		applicable = { InspectType::OutputVariables, InspectType::ObjectType };
		if (object->GetNumberOfNodes() != 0) { applicable.push_back(InspectType::RequestedNodeTypes); }
		if (EXUstd::IsOfType(object->GetType(), CObjectType::Connector)) { applicable.push_back(InspectType::RequestedMarkerTypes); }
		if (EXUstd::IsOfType(object->GetType(), CObjectType::Body)) { applicable.push_back(InspectType::AccessFunctions); }
	}
	else if (itemType == ItemType::Node) { applicable = { InspectType::OutputVariables, InspectType::NodeType }; }
	else if (itemType == ItemType::Marker)
	{
		applicable = { InspectType::OutputVariables, InspectType::MarkerType };
		if (data.GetMainMarkers()[number]->GetRequestedNodeTypes().size() != 0) { applicable.push_back(InspectType::RequestedNodeTypes); }
	}
	else if (itemType == ItemType::Load) { applicable = { InspectType::RequestedMarkerTypes }; }

	auto Answer = [&](InspectType inspectType) -> py::object
	{
		switch (inspectType)
		{
		case InspectType::OutputVariables:
		{
			Index64 types = 0;
			if (itemType == ItemType::Object)
			{
				types = (Index64)object->GetOutputVariableTypes();
				if (!object->PotentialEnergyAvailable()) { types &= ~(Index64)OutputVariableType::PotentialEnergy; }
			}
			else if (itemType == ItemType::Node) { types = (Index64)data.GetMainNodes()[number]->GetCNode()->GetOutputVariableTypes(); }
			else { types = (Index64)data.GetMainMarkers()[number]->GetCMarker()->GetOutputVariableTypes(); }
			return InspectFlags<OutputVariableType>(types);
		}
		case InspectType::ObjectType: return InspectFlags<CObjectType>((Index64)object->GetType());
		case InspectType::NodeType: return InspectFlags<Node::Type>((Index64)data.GetMainNodes()[number]->GetCNode()->GetType());
		case InspectType::MarkerType: return InspectFlags<Marker::Type>((Index64)data.GetMainMarkers()[number]->GetCMarker()->GetType());
		case InspectType::AccessFunctions: return InspectFlags<AccessFunctionType>((Index64)object->GetAccessFunctionTypes());
		case InspectType::RequestedNodeTypes:
		{
			py::list perNode;
			if (itemType == ItemType::Marker) //one node; a list of requirements, each a list of alternatives (#2817)
			{
				py::list requirements;
				for (const std::vector<Node::Type>& alternatives : data.GetMainMarkers()[number]->GetRequestedNodeTypes())
				{
					py::list members;
					for (Node::Type alternative : alternatives) { members.append(py::cast(alternative)); }
					requirements.append(members);
				}
				perNode.append(requirements);
				return perNode;
			}
			for (Index i = 0; i < object->GetNumberOfNodes(); i++)
			{
				perNode.append(InspectFlags<Node::Type>((Index64)data.GetMainObjects()[number]->GetRequestedNodeType()));
			}
			return perNode;
		}
		case InspectType::RequestedMarkerTypes:
		{
			py::list perMarker;
			if (itemType == ItemType::Load)
			{
				perMarker.append(InspectFlags<Marker::Type>((Index64)data.GetMainLoads()[number]->GetCLoad()->GetRequestedMarkerType()));
			}
			else
			{
				const CObjectConnector* connector = (const CObjectConnector*)object;
				for (Index i = 0; i < connector->GetMarkerNumbers().NumberOfItems(); i++)
				{
					perMarker.append(InspectFlags<Marker::Type>((Index64)connector->GetRequestedMarkerType()));
				}
			}
			return perMarker;
		}
		default: return py::none();
		}
	};

	if (what.is_none())
	{
		py::dict all;
		for (InspectType inspectType : applicable) { all[py::cast(inspectType)] = Answer(inspectType); }
		return all;
	}
	if (!py::isinstance<InspectType>(what))
	{
		PyError("Inspect: what must be a member of exu.InspectType or None; got " + STDstring(py::str(what)), PyErrorType::typeError);
		return py::none();
	}
	InspectType inspectType = py::cast<InspectType>(what);
	if (std::find(applicable.begin(), applicable.end(), inspectType) == applicable.end())
	{
		STDstring list;
		for (InspectType t : applicable) { list += (list.size() ? ", " : "") + STDstring("InspectType.") + EXUstd::ToString(t); }
		PyError("Inspect: InspectType." + EXUstd::ToString(inspectType) + " does not apply to " + EXUstd::ToString(itemType) + " " + EXUstd::ToString(number)
			+ "; what applies: [" + list + "]", PyErrorType::valueError);
		return py::none();
	}
	return Answer(inspectType);
}

py::object MainSystem::PyGetObjectOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentOrIllegalConfiguration("GetObjectOutput", configuration, itemNumber, ItemType::Object);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutput", variableType, configuration, itemNumber, ItemType::Object);

		if ((Index)mainSystemData.GetMainObjects().GetItem(itemNumber)->GetCObject()->GetType() & (Index)CObjectType::Connector)
		{
			CHECKandTHROW(configuration == ConfigurationType::Current, "GetObjectOutput: may only be called for connectors with Current configuration", ExudynValueError);
			TemporaryMarkerDataStructure temporary; //no allocation per call (#2745)
			MarkerDataStructure& markerDataStructure = temporary.Get();
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
		PyError(STDstring("MainSystem::GetObjectOutputVariable: access to invalid object number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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

		Index itemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(itemIndex);
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
					EXUstd::ToString(itemNumber), PyErrorType::valueError);
				return py::int_(EXUstd::InvalidIndex);
				//return py::object();
			}
		}
		else
		{
			PyError(STDstring("MainSystem::GetObjectOutputVariableBody: access to invalid object number ") + EXUstd::ToString(itemNumber), PyErrorType::typeError);
			return py::int_(EXUstd::InvalidIndex);
			//return py::object();
		}
}

//! get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType
py::object MainSystem::PyGetObjectOutputVariableSuperElement(const py::object& itemIndex, OutputVariableType variableType, 
	Index meshNodeNumber, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetObjectOutputSuperElement", configuration, itemNumber, ItemType::Object);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutputVariableSuperElement", variableType, configuration, itemNumber, ItemType::Object);
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariableSuperElement(variableType, meshNodeNumber, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::PyGetObjectOutputVariableSuperElement: access to invalid object number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! Get (read) parameter 'parameterName' of 'objectNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetObjectParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetObjectParameter: access to invalid object number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'objectNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetObjectParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<ObjectIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainObjects().NumberOfItems()))
	{
		mainSystemData.GetMainObjects().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetObjectParameter: access to invalid object number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
	//a parameter error that named its Python exception type keeps it; its message was
	//already printed where it was raised (#2432).
	//NOTE: this MUST come before catch(EXUexception), which is std::runtime_error and would
	//otherwise catch it first (ReleaseAssert.h:30) - and the same holds for the Exudyn exception
	//classes, which derive from EXUexception for exactly that reason (#2516)
	catch (const py::builtin_exception&)
	{
		throw;
	}
	catch (const ExudynError&)
	{
		throw;
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
	Index deleteItemNumber = EPyUtils::ItemIndexFromPython<MarkerIndex>(markerNumber);
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
		PyError(STDstring("MainSystem::DeleteMarker: access to invalid marker number ") + EXUstd::ToString(deleteItemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<MarkerIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()) )
	{
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarker: access to invalid marker number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<MarkerIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainMarkers().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyMarker: access to invalid marker number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
	}
}

//! get marker's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetMarkerDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetMarkerDefaults: typeName needed'"), PyErrorType::valueError);
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
		PyError(STDstring("MainSystem::GetMarkerDefaults: unknown object type '") + typeName + "'", PyErrorType::valueError);
	}
	return d;
}

//! Get (read) parameter 'parameterName' of 'markerNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetMarkerParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<MarkerIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerParameter: access to invalid marker number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'markerNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetMarkerParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<MarkerIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		mainSystemData.GetMainMarkers().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetMarkerParameter: access to invalid marker number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
	}
}

//! Get specific output variable with variable type
py::object MainSystem::PyGetMarkerOutputVariable(const py::object& itemIndex, OutputVariableType variableType, ConfigurationType configuration) const
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<MarkerIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainMarkers().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetMarkerOutput", configuration, itemNumber, ItemType::Marker);
		GetMainSystemData().RaiseIfNotOutputVariableTypeForReferenceConfiguration("GetObjectOutputVariableSuperElement", variableType, configuration, itemNumber, ItemType::Marker);

		//the marker function itself will raise an error, if it is not able to return the according output variable
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetOutputVariable(GetCSystem().GetSystemData(), variableType, configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerOutput: access to invalid marker number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
	//a parameter error that named its Python exception type keeps it; its message was
	//already printed where it was raised (#2432).
	//NOTE: this MUST come before catch(EXUexception), which is std::runtime_error and would
	//otherwise catch it first (ReleaseAssert.h:30) - and the same holds for the Exudyn exception
	//classes, which derive from EXUexception for exactly that reason (#2516)
	catch (const py::builtin_exception&)
	{
		throw;
	}
	catch (const ExudynError&)
	{
		throw;
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
	Index deleteItemNumber = EPyUtils::ItemIndexFromPython<LoadIndex>(loadNumber);
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
		PyError(STDstring("MainSystem::DeleteLoad: access to invalid load number ") + EXUstd::ToString(deleteItemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<LoadIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()) )
	{
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoad: access to invalid load number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<LoadIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainLoads().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyLoad: access to invalid load number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
	}
}

//! get LoadPoint default values, which helps for manual writing of python input
py::dict MainSystem::PyGetLoadDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetLoadDefaults: typeName needed'"), PyErrorType::valueError);
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
		PyError(STDstring("MainSystem::GetLoadDefaults: unknown load type '") + typeName + "'", PyErrorType::valueError);
	}
	return d;
}

//! Get current load values, specifically if user-defined loads are used
py::object MainSystem::PyGetLoadValues(const py::object& itemIndex) const
{

	Index itemNumber = EPyUtils::ItemIndexFromPython<LoadIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistent("GetLoadValues", itemNumber, ItemType::Load);
		Real t = GetCSystem().GetSystemData().GetCData().GetCurrent().GetTime(); //only current time available
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetLoadValues(GetCSystem().GetSystemData().GetMainSystemBacklink(), t);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadValues: access to invalid load number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! Get (read) parameter 'parameterName' of 'loadNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetLoadParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<LoadIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadParameter: access to invalid load number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'loadNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetLoadParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<LoadIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainLoads().NumberOfItems()))
	{
		mainSystemData.GetMainLoads().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetLoadParameter: access to invalid load number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
	//a parameter error that named its Python exception type keeps it; its message was
	//already printed where it was raised (#2432).
	//NOTE: this MUST come before catch(EXUexception), which is std::runtime_error and would
	//otherwise catch it first (ReleaseAssert.h:30) - and the same holds for the Exudyn exception
	//classes, which derive from EXUexception for exactly that reason (#2516)
	catch (const py::builtin_exception&)
	{
		throw;
	}
	catch (const ExudynError&)
	{
		throw;
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
	Index deleteItemNumber = EPyUtils::ItemIndexFromPython<SensorIndex>(sensorNumber);
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
		PyError(STDstring("MainSystem::DeleteSensor: access to invalid sensor number ") + EXUstd::ToString(deleteItemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<SensorIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()) )
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensor: access to invalid sensor number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
	Index itemNumber = EPyUtils::ItemIndexFromPython<SensorIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		SystemHasChanged();
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifySensor: access to invalid sensor number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
	}
}

//! get Sensor's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetSensorDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		PyError(STDstring("MainSystem::GetSensorDefaults: typeName needed'"), PyErrorType::valueError);
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
		PyError(STDstring("MainSystem::GetSensorDefaults: unknown sensor type '") + typeName + "'", PyErrorType::valueError);
	}
	return d;
}

//! get sensor's values
py::object MainSystem::PyGetSensorValues(const py::object& itemIndex, ConfigurationType configuration)
{

	Index itemNumber = EPyUtils::ItemIndexFromPython<SensorIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		GetMainSystemData().RaiseIfNotConsistentNorReference("GetSensorValues", configuration, itemNumber, ItemType::Sensor);
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetSensorValues(GetCSystem().GetSystemData(), configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorValues: access to invalid sensor number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! get sensor's stored data (if it exists ...)
py::array_t<Real> MainSystem::PyGetSensorStoredData(const py::object& itemIndex)
{

	Index itemNumber = EPyUtils::ItemIndexFromPython<SensorIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		if (!mainSystemData.GetMainSensors().GetItem(itemNumber)->GetCSensor()->GetStoreInternalFlag())
		{
			PyError(STDstring("MainSystem::GetSensorStoredData: sensor number ") + EXUstd::ToString(itemNumber)+" has no internal data as storeInternal==False", PyErrorType::modelError);
			return py::int_(EXUstd::InvalidIndex);
		}
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetInternalStorage();
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorStoredData: access to invalid sensor number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
	}
}



//! Get (read) parameter 'parameterName' of 'sensorNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetSensorParameter(const py::object& itemIndex, const STDstring& parameterName) const
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<SensorIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorParameter: access to invalid sensor number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'SensorNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetSensorParameter(const py::object& itemIndex, const STDstring& parameterName, const py::object& value)
{
	Index itemNumber = EPyUtils::ItemIndexFromPython<SensorIndex>(itemIndex);
	if (EXUstd::IndexIsInRange(itemNumber, 0, mainSystemData.GetMainSensors().NumberOfItems()))
	{
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetSensorParameter: access to invalid sensor number ") + EXUstd::ToString(itemNumber), PyErrorType::indexError);
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
		CHECKandTHROWstring(s, ExudynValueError);
	}
	else if ((Index)configuration >= (Index)ConfigurationType::EndOfEnumList)
	{
		STDstring s = STDstring("MainSystem::") + functionName;
		if (itemIndex >= 0) { s += STDstring("(") + EXUstd::ToString(itemType) + " " + EXUstd::ToString(itemIndex) + ")"; }
		s += ": called with illegal configuration ConfigurationType.???";
		CHECKandTHROWstring(s, ExudynValueError);
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
		CHECKandTHROWstring(s, ExudynModelError);
	}
}

void MainSystemData::RaiseIfNotConsistent(const char* functionName, Index itemIndex, ItemType itemType) const
{
	if (!GetCSystemData().GetCData().IsSystemConsistent())
	{
		STDstring s = STDstring("MainSystem::") + functionName;
		if (itemIndex >= 0) { s += STDstring("(") + EXUstd::ToString(itemType) + " " + EXUstd::ToString(itemIndex) + ")"; }
		s += ": called for inconsistent system; a call to mbs.Assemble() is necessary prior to this function call (such that mbs.systemIsConsistent returns True)";
		CHECKandTHROWstring(s, ExudynModelError);
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
		CHECKandTHROWstring(s, ExudynValueError);
	}
}

