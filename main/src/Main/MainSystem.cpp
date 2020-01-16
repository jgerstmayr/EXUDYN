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
	return GetMainObjectFactory().AddMainNode(*this, d);
};

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
		return mainSystemData.GetMainNodes().GetItem(nodeNumber)->SetWithDictionary(nodeDict);
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
		PyError(STDstring("MainSystem::PyGetNodeOutputVariable: invalid access to node number ") + EXUstd::ToString(nodeNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
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
		PyError(STDstring("MainSystem::PyGetNodeParameter: invalid access to node number ") + EXUstd::ToString(nodeNumber));
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
		PyError(STDstring("MainSystem::PySetNodeParameter: invalid access to node number ") + EXUstd::ToString(nodeNumber));
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
	return GetMainObjectFactory().AddMainObject(*this, d);
};

//! get object's dictionary by name; does not throw a error message
Index MainSystem::PyGetObjectNumber(STDstring objectName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainObjects(), objectName);
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
py::dict MainSystem::PyGetObject(Index objectNumber)
{
	if (objectNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		return mainSystemData.GetMainObjects().GetItem(objectNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetObject: invalid access to object number ") + EXUstd::ToString(objectNumber));
		py::dict d;
		return d;
	}
}

//! get object's dictionary by name
py::dict MainSystem::PyGetObjectByName(STDstring objectName)
{
	Index ind = PyGetObjectNumber(objectName);
	if (ind != EXUstd::InvalidIndex) { return PyGetObject(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetObject: invalid access to object '") + objectName + "'");
		return py::dict();
	}
}

//! modify object's dictionary
void MainSystem::PyModifyObject(Index objectNumber, py::dict d)
{
	if (objectNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
		return mainSystemData.GetMainObjects().GetItem(objectNumber)->SetWithDictionary(d);
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyObject: invalid access to object number ") + EXUstd::ToString(objectNumber));
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
py::object MainSystem::PyCallObjectFunction(Index objectNumber, STDstring functionName, py::dict args)
{
	if (objectNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		return mainSystemData.GetMainObjects().GetItem(objectNumber)->CallFunction(functionName, args);
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyObject: invalid access to object number ") + EXUstd::ToString(objectNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Get specific output variable with variable type
py::object MainSystem::PyGetObjectOutputVariable(Index objectNumber, OutputVariableType variableType)
{
	if (objectNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		if ((Index)mainSystemData.GetMainObjects().GetItem(objectNumber)->GetCObject()->GetType() & (Index)CObjectType::Connector)
		{
			MarkerDataStructure markerDataStructure;
			const bool computeJacobian = false; //not needed for OutputVariables
			CObjectConnector* connector = (CObjectConnector*)(mainSystemData.GetMainObjects().GetItem(objectNumber)->GetCObject());
			GetCSystem()->ComputeMarkerDataStructure(connector, computeJacobian, markerDataStructure);

			return mainSystemData.GetMainObjects().GetItem(objectNumber)->GetOutputVariableConnector(variableType, markerDataStructure);

		} else
		{
			return mainSystemData.GetMainObjects().GetItem(objectNumber)->GetOutputVariable(variableType);
		}
	}
	else
	{
		PyError(STDstring("MainSystem::PyGetObjectOutputVariable: invalid access to object number ") + EXUstd::ToString(objectNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Get specific output variable with variable type; ONLY for bodies;
//py::object MainSystem::PyGetObjectOutputBody(Index objectNumber, OutputVariableType variableType,
//	const Vector3D& localPosition, ConfigurationType configuration) //no conversion from py to Vector3D!
py::object MainSystem::PyGetObjectOutputVariableBody(Index objectNumber, OutputVariableType variableType,
		const std::vector<Real>& localPosition, ConfigurationType configuration)
{
	if (localPosition.size() == 3)
	{
		if (objectNumber < mainSystemData.GetMainObjects().NumberOfItems())
		{
			return mainSystemData.GetMainObjects().GetItem(objectNumber)->GetOutputVariableBody(variableType, localPosition, configuration);
		}
		else
		{
			PyError(STDstring("MainSystem::PyGetObjectOutputVariableBody: invalid access to object number ") + EXUstd::ToString(objectNumber));
			return py::int_(EXUstd::InvalidIndex);
			//return py::object();
		}
	}
	else
	{
		PyError(STDstring("MainSystem::PyGetOutputVariableBody: invalid localPosition: expected vector with 3 real values"));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Get (read) parameter 'parameterName' of 'objectNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetObjectParameter(Index objectNumber, const STDstring& parameterName) const
{
	if (objectNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		return mainSystemData.GetMainObjects().GetItem(objectNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::PyGetObjectParameter: invalid access to object number ") + EXUstd::ToString(objectNumber));
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! Set (write) parameter 'parameterName' of 'objectNumber' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
void MainSystem::PySetObjectParameter(Index objectNumber, const STDstring& parameterName, const py::object& value)
{
	if (objectNumber < mainSystemData.GetMainObjects().NumberOfItems())
	{
		mainSystemData.GetMainObjects().GetItem(objectNumber)->SetParameter(parameterName, value);
	}
	else
	{
		PyError(STDstring("MainSystem::PySetObjectParameter: invalid access to object number ") + EXUstd::ToString(objectNumber));
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
	return GetMainObjectFactory().AddMainMarker(*this, d);
};

//! get object's dictionary by name; does not throw a error message
Index MainSystem::PyGetMarkerNumber(STDstring objectName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainMarkers(), objectName);
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
py::dict MainSystem::PyGetMarker(Index objectNumber)
{
	if (objectNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		return mainSystemData.GetMainMarkers().GetItem(objectNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetMarker: invalid access to object number ") + EXUstd::ToString(objectNumber));
		py::dict d;
		return d;
	}
}

//! get object's dictionary by name
py::dict MainSystem::PyGetMarkerByName(STDstring objectName)
{
	Index ind = PyGetMarkerNumber(objectName);
	if (ind != EXUstd::InvalidIndex) { return PyGetMarker(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetMarker: invalid access to object '") + objectName + "'");
		return py::dict();
	}
}

//! modify object's dictionary
void MainSystem::PyModifyMarker(Index objectNumber, py::dict d)
{
	if (objectNumber < mainSystemData.GetMainMarkers().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
		return mainSystemData.GetMainMarkers().GetItem(objectNumber)->SetWithDictionary(d);
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyMarker: invalid access to object number ") + EXUstd::ToString(objectNumber));
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
		PyError(STDstring("MainSystem::PyGetMarkerParameter: invalid access to marker number ") + EXUstd::ToString(markerNumber));
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
		PyError(STDstring("MainSystem::PySetMarkerParameter: invalid access to marker number ") + EXUstd::ToString(markerNumber));
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
	return GetMainObjectFactory().AddMainLoad(*this, d);
};

//! get object's dictionary by name; does not throw a error message
Index MainSystem::PyGetLoadNumber(STDstring objectName)
{
	Index ind = EXUstd::GetIndexByName(mainSystemData.GetMainLoads(), objectName);
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
py::dict MainSystem::PyGetLoad(Index objectNumber)
{
	if (objectNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		return mainSystemData.GetMainLoads().GetItem(objectNumber)->GetDictionary();
	}
	else
	{
		PyError(STDstring("MainSystem::GetLoad: invalid access to object number ") + EXUstd::ToString(objectNumber));
		py::dict d;
		return d;
	}
}

//! get object's dictionary by name
py::dict MainSystem::PyGetLoadByName(STDstring objectName)
{
	Index ind = PyGetLoadNumber(objectName);
	if (ind != EXUstd::InvalidIndex) { return PyGetLoad(ind); }
	else
	{
		PyError(STDstring("MainSystem::GetLoad: invalid access to object '") + objectName + "'");
		return py::dict();
	}
}

//! modify object's dictionary
void MainSystem::PyModifyLoad(Index objectNumber, py::dict d)
{
	if (objectNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		GetCSystem()->SystemHasChanged();
		return mainSystemData.GetMainLoads().GetItem(objectNumber)->SetWithDictionary(d);
	}
	else
	{
		PyError(STDstring("MainSystem::ModifyLoad: invalid access to object number ") + EXUstd::ToString(objectNumber));
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

//! Get (read) parameter 'parameterName' of 'loadNumber' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
py::object MainSystem::PyGetLoadParameter(Index loadNumber, const STDstring& parameterName) const
{
	if (loadNumber < mainSystemData.GetMainLoads().NumberOfItems())
	{
		return mainSystemData.GetMainLoads().GetItem(loadNumber)->GetParameter(parameterName);
	}
	else
	{
		PyError(STDstring("MainSystem::PyGetLoadParameter: invalid access to load number ") + EXUstd::ToString(loadNumber));
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
		PyError(STDstring("MainSystem::PySetLoadParameter: invalid access to load number ") + EXUstd::ToString(loadNumber));
	}
}


