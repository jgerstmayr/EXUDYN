/** ***********************************************************************************************
* @brief		Implementation for MainNode
*
* @author		Gerstmayr Johannes
* @date			2019-07-23 (generated)
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

#include "Main/MainSystem.h"
#include "Linalg/RigidBodyMath.h"
//#include "Pymodules/PybindUtilities.h"



//! GetOutputVariable with type and return value; copies values==>slow!; can be scalar or vector-valued! maps to CNode GetOutputVariable(...)
py::object MainNode::GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration) const
{
	Vector value;
	//check if type is valid:
	if ((Index)GetCNode()->GetOutputVariableTypes() & (Index)variableType)
	{
		GetCNode()->GetOutputVariable(variableType, configuration, value);

		//now check if it is scalar or a vector-valued:
		if (value.NumberOfItems() == 1) { return py::float_(value[0]); }
		else { return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer()); }
	}
	else
	{
		PyError(STDstring("Invalid OutputVariableType in MainNode::GetOutputVariable: '") + GetOutputVariableTypeString(variableType) + 
			"'; the node '" + GetName() + "' cannot compute the requested variable type");
		return py::int_(EXUstd::InvalidIndex);
		//return py::object();
	}
}

//! GetOutputVariable with type and return value; restricted to certain number of types
py::object MainMarker::GetOutputVariable(const CSystemData& cSystemData, OutputVariableType variableType, ConfigurationType configuration) const
{
	Vector values;
	if (GetCMarker()->GetOutputVariable(cSystemData, variableType, configuration, values))
	{
		return py::array_t<Real>(values.NumberOfItems(), values.GetDataPointer());
	}
	else
	{
		PyError(STDstring("Invalid OutputVariableType in MainMarker::GetOutputVariable: '") + GetOutputVariableTypeString(variableType) + 
			"'; the marker '" + GetName() + "' cannot compute the requested variable type");
		return py::int_(EXUstd::InvalidIndex);
	}
}

//! GetOutputVariable with type; writes values into value and returns true, if successful
bool CMarker::GetOutputVariable(const CSystemData& cSystemData, OutputVariableType variableType, ConfigurationType configuration, Vector& value) const
{
	Vector3D value3D;

	bool isSuccess = true;
	if (EXUstd::IsOfType((Index)OutputVariableType::Coordinates + (Index)OutputVariableType::Coordinates_t, (Index)variableType))
	{
		if (configuration != ConfigurationType::Current || 
            (((Index)GetType() & (Index)(Marker::Coordinates + Marker::Coordinate)) == 0)) //added 2023-05-01: otherwise, returns any data stored in markerData
		{
			isSuccess = false;
		}
		else
		{
			bool computeJacobian = false;
			MarkerData markerData; //slow!
			ComputeMarkerData(cSystemData, computeJacobian, markerData);

			if (variableType == OutputVariableType::Coordinates)
			{
				value.CopyFrom(markerData.vectorValue);
			}
			if (variableType == OutputVariableType::Coordinates_t)
			{
				if (!markerData.velocityAvailable) { isSuccess = false; }
				else { value.CopyFrom(markerData.vectorValue_t); }
			}
		}
		return isSuccess;
	}

	switch (variableType)
	{
	case OutputVariableType::Displacement:
	{
		if (EXUstd::IsOfType(GetType(), Marker::Position))
		{
			Vector3D refPos;
			GetPosition(cSystemData, refPos, ConfigurationType::Reference);
			GetPosition(cSystemData, value3D, configuration);
			value.CopyFrom(value3D - refPos);
		}
		else { isSuccess = false; }
		break;
	}
	case OutputVariableType::Position:
	{
		if (EXUstd::IsOfType(GetType(), Marker::Position))
		{
			GetPosition(cSystemData, value3D, configuration); value.CopyFrom(value3D);
		}
		else { isSuccess = false; }
		break;
	}
	case OutputVariableType::Velocity:
	{
		if (EXUstd::IsOfType(GetType(), Marker::Position))
		{
			GetVelocity(cSystemData, value3D, configuration); value.CopyFrom(value3D);
		}
		else { isSuccess = false; }
		break;
	}
	case OutputVariableType::AngularVelocity:
	{
		if (EXUstd::IsOfType(GetType(), Marker::Orientation))
		{
			GetAngularVelocity(cSystemData, value3D, configuration); value.CopyFrom(value3D);
		}
		else { isSuccess = false; }
		break;
	}
	case OutputVariableType::AngularVelocityLocal:
	{
		if (EXUstd::IsOfType(GetType(), Marker::Orientation))
		{
			GetAngularVelocityLocal(cSystemData, value3D, configuration); value.CopyFrom(value3D);
		}
		else { isSuccess = false; }
		break;
	}
	case OutputVariableType::RotationMatrix:
	{
		if (EXUstd::IsOfType(GetType(), Marker::Orientation))
		{
			Matrix3D rot;
			GetRotationMatrix(cSystemData, rot, configuration);
			LinkedDataVector rotVector(rot.GetDataPointer(), rot.NumberOfRows()*rot.NumberOfColumns());
			value.CopyFrom(rotVector);
		}
		else { isSuccess = false; }
		break;
	}
	case OutputVariableType::Rotation:
	{
		if (EXUstd::IsOfType(GetType(), Marker::Orientation))
		{
			Matrix3D rot;
			GetRotationMatrix(cSystemData, rot, configuration);
			Vector3D rot3D = RigidBodyMath::RotationMatrix2RotXYZ(rot);
			value.CopyFrom(rot3D);
		}
		else { isSuccess = false; }
		break;
	}
	default: {isSuccess = false; }
	}

	return isSuccess;
}

OutputVariableType CMarker::GetOutputVariableTypes() const
{
	Index ovt = 0;
	if (EXUstd::IsOfType(GetType(), Marker::Position))
	{
		ovt += (Index)OutputVariableType::Displacement + (Index)OutputVariableType::Position + (Index)OutputVariableType::Velocity;
	}
	if (EXUstd::IsOfType(GetType(), Marker::Orientation))
	{
		ovt += (Index)OutputVariableType::AngularVelocity + (Index)OutputVariableType::AngularVelocityLocal + 
			(Index)OutputVariableType::RotationMatrix + (Index)OutputVariableType::Rotation;
	}
	if (EXUstd::IsOfType(GetType(), Marker::Coordinate) || EXUstd::IsOfType(GetType(), Marker::Coordinates))
	{
		ovt += (Index)OutputVariableType::Coordinates + (Index)OutputVariableType::Coordinates_t; //via MarkerData
	}

	return (OutputVariableType)ovt;
}

