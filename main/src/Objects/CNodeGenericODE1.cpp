/** ***********************************************************************************************
* @brief		Implementation for CNodeGenericODE1
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
#include "Autogenerated/CNodeGenericODE1.h"


//! provide according output variable in "value"
void CNodeGenericODE1::GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration, Vector& value) const
{
	switch (variableType)
	{
	case OutputVariableType::Coordinates:
	{
		if (IsValidConfiguration(configuration))
		{
			value = GetCoordinateVector(configuration);
		}
		else
		{
			PyError("CNodeGenericODE1::GetOutputVariable: invalid configuration");
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
			PyError("CNodeGenericODE1::GetOutputVariable: invalid configuration");
		}
		break;
	}
	default:
		SysError("CNodeGenericODE1::GetOutputVariable failed"); //error should not occur, because types are checked!
	}
}
