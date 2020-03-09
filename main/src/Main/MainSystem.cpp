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
* 				- weblink: missing
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#pragma once

#include <chrono> //sleep_for()
#include <thread>

#include "Main/MainSystemData.h"
#include "Main/MainSystem.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  SYSTEM FUNCTIONS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! reset all lists and deallocate memory
void MainSystem::Reset()
{
	mainSystemData.Reset(); //
	GetCSystem()->GetSystemData().Reset();
	GetCSystem()->Initialize();
	visualizationSystem.Reset();
	interactiveMode = false;
}

//!  if interAciveMode == true: causes Assemble() to be called; this guarantees that the system is always consistent to be drawn
void MainSystem::InteractiveModeActions()
{
	if (GetInteractiveMode())
	{
		GetCSystem()->Assemble(*this);
		GetCSystem()->GetPostProcessData()->SendRedrawSignal();
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  VISUALIZATION FUNCTIONS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this function links the VisualizationSystem to a render engine, such that the changes in the graphics structure drawn upon updates, etc.
//  This function is called on creation of a main system and automatically links to renderer
bool MainSystem::LinkToRenderEngine()
{
	visualizationSystem.LinkToSystemData(&GetCSystem()->GetSystemData());
	visualizationSystem.LinkPostProcessData(GetCSystem()->GetPostProcessData());
	return true; // visualizationSystem.LinkToRenderEngine(*GetCSystem());
}

//! this function releases the VisualizationSystem from the render engine;
bool MainSystem::DetachRenderEngine()
{
	//at the moment nothing is done; but it could remove the links to systemData and postProcessData
	return true;
	//visualizationSystem.DetachRenderEngine();
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  NODE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainNode(py::dict d)
{
	GetCSystem()->SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainNode(*this, d);
	InteractiveModeActions();
	return ind;
};

Index MainSystem::AddMainNodePyClass(py::object pyObject)
{
	//py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
	//return AddMainNode(dictObject);
	if (py::isinstance<py::dict>(pyObject))
	{
		py::dict dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		return AddMainNode(dictObject);

	}
	else //must be itemInterface convertable to dict ==> otherwise raises pybind error
	{
		py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		return AddMainNode(dictObject);
	}
}

//! get node's dictionary by name; does not throw a error message
Index MainSystem::PyGetNodeNumber(STDstring nodeName)
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
py::dict MainSystem::PyGetNode(Index nodeNumber)
{
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

//! get node's dictionary by name
py::dict MainSystem::PyGetNodeByName(STDstring nodeName)
{
	Index ind = PyGetNodeNumber(nodeName);
	if (ind != EXUstd::InvalidIndex) { return PyGetNode(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetNode: invalid access to node '") + nodeName + "'");
		return py::dict();
	}
}

//! modify node's dictionary
void MainSystem::PyModifyNode(Index nodeNumber, py::dict nodeDict)
{
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
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
		pout << "available node types are: [Point]\n";
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

py::object MainSystem::PyGetNodeOutputVariable(Index nodeNumber, OutputVariableType variableType, ConfigurationType configuration) const
{
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
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
Index MainSystem::PyGetNodeODE2Index(Index nodeNumber) const
{
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		if (mainSystemData.GetMainNodes().GetItem(nodeNumber)->GetCNode()->GetNodeGroup() == CNodeGroup::ODE2variables)
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



//! call pybind object function, possibly with arguments; empty function, to be overwritten in specialized class
py::object MainSystem::PyCallNodeFunction(Index nodeNumber, STDstring functionName, py::dict args)
{
	if (nodeNumber < mainSystemData.GetMainNodes().NumberOfItems())
	{
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->CallFunction(functionName, args);
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyObject: invalid access to node number ") + EXUstd::ToString(nodeNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}

}


//! Get (read) parameter 'parameterName' of 'nodeNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetNodeParameter(Index nodeNumber, const STDstring& parameterName) const
{
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
void MainSystem::PySetNodeParameter(Index nodeNumber, const STDstring& parameterName, const py::object& value)
{
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
Index MainSystem::AddMainObject(py::dict d)
{
	GetCSystem()->SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainObject(*this, d);
	InteractiveModeActions();
	return ind;
};

Index MainSystem::AddMainObjectPyClass(py::object pyObject)
{
	//py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
	//return AddMainObject(dictObject);
	if (py::isinstance<py::dict>(pyObject))
	{
		py::dict dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		return AddMainObject(dictObject);

	}
	else //must be itemInterface convertable to dict ==> otherwise raises pybind error
	{
		py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		return AddMainObject(dictObject);
	}
}

//! get object's dictionary by name; does not throw a error message
Index MainSystem::PyGetObjectNumber(STDstring itemName)
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
py::dict MainSystem::PyGetObject(Index itemNumber)
{
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetObject: invalid access to object number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

//! get object's dictionary by name
py::dict MainSystem::PyGetObjectByName(STDstring itemName)
{
	Index ind = PyGetObjectNumber(itemName);
	if (ind != EXUstd::InvalidIndex) { return PyGetObject(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetObject: invalid access to object '") + itemName + "'");
		return py::dict();
	}
}

//! modify object's dictionary
void MainSystem::PyModifyObject(Index itemNumber, py::dict d)
{
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
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
		pout << "available object types are: [MassPoint,SpringDamper,Distance]\n";
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

//! call pybind object function, possibly with arguments; empty function, to be overwritten in specialized class
py::object MainSystem::PyCallObjectFunction(Index itemNumber, STDstring functionName, py::dict args)
{
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		return mainSystemData.GetMainObjects().GetItem(itemNumber)->CallFunction(functionName, args);
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyObject: invalid access to object number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Get specific output variable with variable type
py::object MainSystem::PyGetObjectOutputVariable(Index itemNumber, OutputVariableType variableType)
{
	if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		if ((Index)mainSystemData.GetMainObjects().GetItem(itemNumber)->GetCObject()->GetType() & (Index)CObjectType::Connector)
		{
			MarkerDataStructure markerDataStructure;
			const bool computeJacobian = false; //not needed for OutputVariables
			CObjectConnector* connector = (CObjectConnector*)(mainSystemData.GetMainObjects().GetItem(itemNumber)->GetCObject());
			GetCSystem()->ComputeMarkerDataStructure(connector, computeJacobian, markerDataStructure);

			return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariableConnector(variableType, markerDataStructure);

		} else
		{
			return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariable(variableType);
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
py::object MainSystem::PyGetObjectOutputVariableBody(Index itemNumber, OutputVariableType variableType,
		const std::vector<Real>& localPosition, ConfigurationType configuration)
{
	if (localPosition.size() == 3)
	{
		if (itemNumber < mainSystemData.GetMainObjects().NumberOfItems())
		{
			return mainSystemData.GetMainObjects().GetItem(itemNumber)->GetOutputVariableBody(variableType, localPosition, configuration);
		}
		else
		{
			PyError(STDstring("MainSystem::GetObjectOutputVariableBody: invalid access to object number ") + EXUstd::ToString(itemNumber));
			return py::int_(EXUstd::InvalidIndex);
			//return py::object();
		}
	}
	else
	{
		PyError(STDstring("MainSystem::GetOutputVariableBody: invalid localPosition: expected vector with 3 real values"));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Get (read) parameter 'parameterName' of 'objectNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetObjectParameter(Index itemNumber, const STDstring& parameterName) const
{
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
void MainSystem::PySetObjectParameter(Index itemNumber, const STDstring& parameterName, const py::object& value)
{
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
Index MainSystem::AddMainMarker(py::dict d)
{
	GetCSystem()->SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainMarker(*this, d);
	InteractiveModeActions();
	return ind;
};

Index MainSystem::AddMainMarkerPyClass(py::object pyObject)
{
	//py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
	//return AddMainMarker(dictObject);
	if (py::isinstance<py::dict>(pyObject))
	{
		py::dict dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		return AddMainMarker(dictObject);

	}
	else //must be itemInterface convertable to dict ==> otherwise raises pybind error
	{
		py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		return AddMainMarker(dictObject);
	}
}

//! get object's dictionary by name; does not throw a error message
Index MainSystem::PyGetMarkerNumber(STDstring itemName)
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
py::dict MainSystem::PyGetMarker(Index itemNumber)
{
	if (itemNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		return mainSystemData.GetMainMarkers().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarker: invalid access to object number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

//! get object's dictionary by name
py::dict MainSystem::PyGetMarkerByName(STDstring itemName)
{
	Index ind = PyGetMarkerNumber(itemName);
	if (ind != EXUstd::InvalidIndex) { return PyGetMarker(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetMarker: invalid access to object '") + itemName + "'");
		return py::dict();
	}
}

//! modify object's dictionary
void MainSystem::PyModifyMarker(Index itemNumber, py::dict d)
{
	if (itemNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
		mainSystemData.GetMainMarkers().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyMarker: invalid access to object number ") + EXUstd::ToString(itemNumber));
	}
}

//! get marker's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetMarkerDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		pout << "available load types are: [BodyPosition]\n";
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
py::object MainSystem::PyGetMarkerParameter(Index markerNumber, const STDstring& parameterName) const
{
	if (markerNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		return mainSystemData.GetMainMarkers().GetItem(markerNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarkerParameter: invalid access to marker number ") + EXUstd::ToString(markerNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'markerNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetMarkerParameter(Index markerNumber, const STDstring& parameterName, const py::object& value)
{
	if (markerNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		mainSystemData.GetMainMarkers().GetItem(markerNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetMarkerParameter: invalid access to marker number ") + EXUstd::ToString(markerNumber));
	}
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  LOAD
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainLoad(py::dict d)
{
	GetCSystem()->SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainLoad(*this, d);
	InteractiveModeActions();
	return ind;
};

Index MainSystem::AddMainLoadPyClass(py::object pyObject)
{
	//py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
	//return AddMainLoad(dictObject);
	if (py::isinstance<py::dict>(pyObject))
	{
		py::dict dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		return AddMainLoad(dictObject);

	}
	else //must be itemInterface convertable to dict ==> otherwise raises pybind error
	{
		py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		return AddMainLoad(dictObject);
	}
}

//! get object's dictionary by name; does not throw a error message
Index MainSystem::PyGetLoadNumber(STDstring itemName)
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
py::dict MainSystem::PyGetLoad(Index itemNumber)
{
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoad: invalid access to object number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

//! get object's dictionary by name
py::dict MainSystem::PyGetLoadByName(STDstring itemName)
{
	Index ind = PyGetLoadNumber(itemName);
	if (ind != EXUstd::InvalidIndex) { return PyGetLoad(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetLoad: invalid access to object '") + itemName + "'");
		return py::dict();
	}
}

//! modify object's dictionary
void MainSystem::PyModifyLoad(Index itemNumber, py::dict d)
{
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
		mainSystemData.GetMainLoads().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyLoad: invalid access to object number ") + EXUstd::ToString(itemNumber));
	}
}

//! get LoadPoint default values, which helps for manual writing of python input
py::dict MainSystem::PyGetLoadDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		pout << "available load types are: [ForceVector]\n";
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
		PyError(STDstring("MainSystem::GetLoadDefaults: unknown object type '") + typeName + "'");
	}
	return d;
}

//! Get current load values, specifically if user-defined loads are used
py::object MainSystem::PyGetLoadValues(Index itemNumber) const
{
	if (itemNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		Real t = GetCSystem()->GetSystemData().GetCData().GetCurrent().GetTime(); //only current time available
		return mainSystemData.GetMainLoads().GetItem(itemNumber)->GetLoadValues(t);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadValues: invalid access to load number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! Get (read) parameter 'parameterName' of 'loadNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetLoadParameter(Index loadNumber, const STDstring& parameterName) const
{
	if (loadNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		return mainSystemData.GetMainLoads().GetItem(loadNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoadParameter: invalid access to load number ") + EXUstd::ToString(loadNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'loadNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetLoadParameter(Index loadNumber, const STDstring& parameterName, const py::object& value)
{
	if (loadNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		mainSystemData.GetMainLoads().GetItem(loadNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetLoadParameter: invalid access to load number ") + EXUstd::ToString(loadNumber));
	}
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  SENSOR
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this is the hook to the object factory, handling all kinds of objects, nodes, ...
Index MainSystem::AddMainSensor(py::dict d)
{
	GetCSystem()->SystemHasChanged();
	Index ind = GetMainObjectFactory().AddMainSensor(*this, d);
	InteractiveModeActions();
	return ind;
};

Index MainSystem::AddMainSensorPyClass(py::object pyObject)
{
	if (py::isinstance<py::dict>(pyObject))
	{
		py::dict dictObject = py::cast<py::dict>(pyObject); //convert py::object to dict
		return AddMainSensor(dictObject);

	}
	else //must be itemInterface convertable to dict ==> otherwise raises pybind error
	{
		py::dict dictObject = py::dict(pyObject); //applies dict command to pyObject ==> converts object class to dictionary
		return AddMainSensor(dictObject);
	}
}

//! get object's dictionary by name; does not throw a error message
Index MainSystem::PyGetSensorNumber(STDstring itemName)
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
py::dict MainSystem::PyGetSensor(Index itemNumber)
{
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensor: invalid access to object number ") + EXUstd::ToString(itemNumber));
		py::dict d;
		return d;
	}
}

//! get object's dictionary by name
py::dict MainSystem::PyGetSensorByName(STDstring itemName)
{
	Index ind = PyGetSensorNumber(itemName);
	if (ind != EXUstd::InvalidIndex) { return PyGetSensor(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetSensor: invalid access to object '") + itemName + "'");
		return py::dict();
	}
}

//! modify object's dictionary
void MainSystem::PyModifySensor(Index itemNumber, py::dict d)
{
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetWithDictionary(d);
		InteractiveModeActions();
	}
	else
	{
		PyError(STDstring("MainSystem::ModifySensor: invalid access to object number ") + EXUstd::ToString(itemNumber));
	}
}

//! get Sensor's default values, which helps for manual writing of python input
py::dict MainSystem::PyGetSensorDefaults(STDstring typeName)
{
	py::dict d;
	if (typeName.size() == 0) //in case of empty string-->return available default names!
	{
		pout << "available load types are: [BodyPosition]\n";
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
		PyError(STDstring("MainSystem::GetSensorDefaults: unknown object type '") + typeName + "'");
	}
	return d;
}

//! get sensor's values
py::object MainSystem::PyGetSensorValues(Index itemNumber, ConfigurationType configuration)
{
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		return mainSystemData.GetMainSensors().GetItem(itemNumber)->GetSensorValues(GetCSystem()->GetSystemData(), configuration);
	}
	else
	{
		PyError(STDstring("MainSystem::GetSensorValues: invalid access to node number ") + EXUstd::ToString(itemNumber));
		return py::int_(EXUstd::InvalidIndex);
	}
}



//! Get (read) parameter 'parameterName' of 'sensorNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetSensorParameter(Index itemNumber, const STDstring& parameterName) const
{
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
void MainSystem::PySetSensorParameter(Index itemNumber, const STDstring& parameterName, const py::object& value)
{
	if (itemNumber < mainSystemData.GetMainSensors().NumberOfItems())
	{
		mainSystemData.GetMainSensors().GetItem(itemNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::SetSensorParameter: invalid access to Sensor number ") + EXUstd::ToString(itemNumber));
	}
}

