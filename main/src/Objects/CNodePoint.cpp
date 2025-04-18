/** ***********************************************************************************************
* @brief		Implementation for NodePoint
*
* @author		Gerstmayr Johannes
* @date			2019-05-02 (generated)
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

#include "Main/CSystemData.h"
#include "Autogenerated/CNodePoint.h"

////for CallFunction(...)
//#include "Main/MainSystem.h"
//#include "Pymodules/PybindUtilities.h"
//#include "Autogenerated/MainNodePoint.h"


Vector3D CNodePoint::GetPosition(ConfigurationType configuration) const
{
	if (configuration == ConfigurationType::Reference) {return Vector3D(GetReferenceCoordinateVector()); }

	Vector3D displacement(GetCoordinateVector(configuration));
	Vector3D referencePosition(GetReferenceCoordinateVector());
	return referencePosition + displacement;
}

Vector3D CNodePoint::GetVelocity(ConfigurationType configuration) const
{
	return Vector3D(GetCoordinateVector_t(configuration));
}

Vector3D CNodePoint::GetAcceleration(ConfigurationType configuration) const
{
	//LinkedDataVector u2D_tt = GetCoordinateVector_tt(configuration);
	return Vector3D(GetCoordinateVector_tt(configuration));
}


////! Flags to determine, which output variables are available (displacment, velocity, stress, ...)
//OutputVariableType CNodePoint::GetOutputVariableTypes() const
//{
//	return (OutputVariableType)((Index)OutputVariableType::Position + (Index)OutputVariableType::Velocity + 
//		(Index)OutputVariableType::Coordinates + (Index)OutputVariableType::Coordinates_t);
//}

//! provide according output variable in "value"
void CNodePoint::GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration, Vector& value) const
{
	switch (variableType)
	{
	case OutputVariableType::Position: value.CopyFrom(GetPosition(configuration)); break;
	case OutputVariableType::Displacement: value.CopyFrom(GetPosition(configuration) - GetPosition(ConfigurationType::Reference)); break;
	case OutputVariableType::Velocity: value.CopyFrom(GetVelocity(configuration)); break;
	//case OutputVariableType::Acceleration: value.CopyFrom(GetCoordinateVector_tt(configuration)); break;
	case OutputVariableType::Acceleration: value.CopyFrom(GetAcceleration(configuration)); break;
	case OutputVariableType::CoordinatesTotal:
	{
		if (IsValidConfiguration(configuration))
		{
			GetODE2CoordinateVectorWithReference(value, configuration);
		} else
		{ 
			PyError("CNodePoint::GetOutputVariable: invalid configuration"); 
		}
		break;
	}
	case OutputVariableType::Coordinates:
	{
		if (IsValidConfiguration(configuration))
		{
			value = GetCoordinateVector(configuration);
		}
		else
		{
			PyError("CNodePoint::GetOutputVariable: invalid configuration");
		}
		break;
	}
	case OutputVariableType::Coordinates_t:
	{
		if (IsValidConfigurationButNotReference(configuration)) 
		{
			value = GetCoordinateVector_t(configuration);
		}
		else
		{
			PyError("CNodePoint::GetOutputVariable: invalid configuration");
		}
		break;

	}
	case OutputVariableType::Coordinates_tt:
	{
		if (IsValidConfigurationButNotReference(configuration)) 
		{
			value = GetCoordinateVector_tt(configuration);
		}
		else
		{
			PyError("CNodePoint::GetOutputVariable: invalid configuration");
		}
		break;

	}
	default:
		SysError("CNodePoint::GetOutputVariable failed"); //error should not occur, because types are checked!
	}
	//pout << "Pos=" << GetPosition() << "\n";
	//pout << "value=" << value << "\n";
	//pout << "type=" << GetOutputVariableTypeString(variableType) << "\n";
}

////delete:
////! call a certain function of object (autogenerated in future!)
//py::object MainNodePoint::CallFunction(STDstring functionName, py::dict args) const
//{
//	//these calls should be automated by python script ...
//	if (functionName == "GetTypeName")
//	{
//		return py::str(GetTypeName());
//	}
//	else if (functionName == "GetNodeType")
//	{
//		return py::cast(GetCNodePoint()->GetType());
//	}
//	else if (functionName == "GetNodeGroup")
//	{
//		return py::cast(GetCNodePoint()->GetNodeGroup());
//	}
//	else if (functionName == "GetNumberOfODE2Coordinates")
//	{
//		return py::int_(GetCNodePoint()->GetNumberOfODE2Coordinates());
//	}
//	else if (functionName == "GetGlobalODE2CoordinateIndex")
//	{
//		return py::int_(GetCNodePoint()->GetGlobalODE2CoordinateIndex());
//	}
//	else if (functionName == "GetCurrentCoordinateVector")
//	{
//		LinkedDataVector v(GetCNodePoint()->GetCurrentCoordinateVector());
//		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
//	}
//	else if (functionName == "GetInitialCoordinateVector")
//	{
//		LinkedDataVector v(GetCNodePoint()->GetInitialCoordinateVector());
//		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
//	}
//	else if (functionName == "GetCurrentCoordinateVector_t")
//	{
//		LinkedDataVector v(GetCNodePoint()->GetCurrentCoordinateVector_t());
//		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
//	}
//	else if (functionName == "GetInitialCoordinateVector_t")
//	{
//		LinkedDataVector v(GetCNodePoint()->GetInitialCoordinateVector_t());
//		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
//	}
//	else if (functionName == "GetCurrentPosition")
//	{
//		Vector3D v = GetCNodePoint()->GetPosition(ConfigurationType::Current);
//		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
//	}
//
//	PyError(STDstring("MainNodePoint::CallFunction called with invalid functionName '" + functionName + "'"));
//	return py::int_(EXUstd::InvalidIndex);
//}
