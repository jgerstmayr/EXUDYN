/** ***********************************************************************************************
* @brief		Implementation for MainObject / MainObjectBody
*
* @author		Gerstmayr Johannes
* @date			2019-05-01 (generated)
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

#include "Main/CSystem.h"
#include "Graphics/VisualizationSystemContainer.h"

#include "System/MainMaterial.h"
#include "System/MainMarker.h"
#include "System/MainLoad.h"
#include "System/MainNode.h"
#include "System/MainObject.h"


//! GetOutputVariable with type and return value; copies values==>slow!; can be scalar or vector-valued! maps to CObject GetOutputVariable(...)
py::object MainObject::GetOutputVariable(OutputVariableType variableType) const
{
	if ((Index)GetCObject()->GetType() & (Index)CObjectType::Connector)
	{
		SysError("GetOutputVariable may not be called for Connector");
		return py::object();
	}

	Vector value;
	//check if type is valid:
	if ((Index)GetCObject()->GetOutputVariableTypes() & (Index)variableType)
	{
		GetCObject()->GetOutputVariable(variableType, value);
		//now check if it is scalar or a vector-valued:
		if (value.NumberOfItems() == 1) { return py::float_(value[0]); }
		else { return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer()); }
	}
	else
	{
		PyError(STDstring("Invalid OutputVariableType in MainObject::GetOutputVariable: '") + GetOutputVariableTypeString(variableType) + "'");
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

py::object MainObject::GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index objectNumber) const
{
	Vector value;
	//check if type is valid:
	if ((Index)GetCObject()->GetOutputVariableTypes() & (Index)variableType)
	{
		if ((Index)GetCObject()->GetType() & (Index)CObjectType::Connector)
		{
			const CObjectConnector* connector = (CObjectConnector*)GetCObject();
			connector->GetOutputVariableConnector(variableType, markerData, objectNumber, value);

			//now check if it is scalar or a vector-valued:
			if (value.NumberOfItems() == 1) { return py::float_(value[0]); }
			else { return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer()); }
		}
		else
		{ SysError("GetOutputVariableConnector may only be called for Connector"); return py::object(); }
	}
	else
	{
		//PyError(STDstring("Invalid OutputVariableType in MainObject::Object") + GetTypeName() + ": '" + GetOutputVariableTypeString(variableType) + "'");
		PyError(STDstring("Object") + GetTypeName() + " has no OutputVariableType '" + GetOutputVariableTypeString(variableType) + "'");
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}


//! GetOutputVariable with type and return value; copies values==>slow!; can be scalar or vector-valued! maps to CObject GetOutputVariable(...)
py::object MainObject::GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Index objectNumber) const
{
	Vector value;
	//check if type is valid:
	if ((Index)GetCObject()->GetType() & (Index)CObjectType::Body) //use '&': might be CObjectType::Body and also CObjectType::MultiNoded
	{
		if (EXUstd::IsOfType(GetCObject()->GetOutputVariableTypes(),  variableType))
		{
			const CObjectBody* cObjectBody = (const CObjectBody*)GetCObject();

			cObjectBody->GetOutputVariableBody(variableType, localPosition, configuration, value, objectNumber);
			//now check if it is scalar or a vector-valued:
			if (value.NumberOfItems() == 1) { return py::float_(value[0]); }
			else { return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer()); }
		}
		else
		{
			PyError(STDstring("Object") + GetTypeName() + " (a body) has no OutputVariableType '" + GetOutputVariableTypeString(variableType) + "'");
			//PyError(STDstring("Invalid OutputVariableType in MainObjectBody::GetOutputVariableBody: '") + GetOutputVariableTypeString(variableType) + "'");
			return py::int_(EXUstd::InvalidIndex);
			//return py::object();
		}
	}
	else
	{
		PyError(STDstring("Incalid call to GetOutputVariableBody(...) for Object") + GetTypeName() + ": access to objects of type 'Body' only");
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}

}

//! GetOutputVariable with type and return value; copies values==>slow!; can be scalar or vector-valued! maps to CObject GetOutputVariable(...)
py::object MainObject::GetOutputVariableSuperElement(OutputVariableType variableType, Index meshNodeNumber, ConfigurationType configuration) const
{
	Vector value;
	//check if type is valid:
	if ((Index)GetCObject()->GetType() & (Index)CObjectType::SuperElement) //use '&': might contain also other types
	{
		const CObjectSuperElement* cObjectSuperElement = (const CObjectSuperElement*)GetCObject();
		//if ((Index)GetCObject()->GetOutputVariableTypes() & (Index)variableType)
		if (EXUstd::IsOfType(cObjectSuperElement->GetOutputVariableTypesSuperElement(meshNodeNumber), variableType))
		{

			cObjectSuperElement->GetOutputVariableSuperElement(variableType, meshNodeNumber, configuration, value);
			//now check if it is scalar or a vector-valued:
			if (value.NumberOfItems() == 1) { return py::float_(value[0]); }
			else { return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer()); }
		}
		else
		{
			PyError(STDstring("Object") + GetTypeName() + " has no OutputVariableType '" + GetOutputVariableTypeString(variableType) + "'");
			//PyError(STDstring("Invalid OutputVariableType in MainObjectBody::GetOutputVariableBody: '") + GetOutputVariableTypeString(variableType) + "'");
			return py::int_(EXUstd::InvalidIndex);
			//return py::object();
		}
	}
	else
	{
		PyError(STDstring("Incalid call to GetOutputVariableSuperElement(...) for Object") + GetTypeName() + ": access to objects of type 'SuperElement' only");
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}

}

