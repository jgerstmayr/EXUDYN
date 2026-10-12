/** ***********************************************************************************************
* @brief		the names of the user types (#2989)
* @details		Details:
*				- OutputVariableType, Node::Type and Marker::Type have the values UserType0 to UserType3 for user
*				  items; exudyn.special.userTypes.SetName gives one a name, which the string functions of the types
*				  write (sensor file header, error messages, ...) instead of UserType0
*				- header only, without pybind11, as the generated enumeration headers include it
*
* @author		Gerstmayr Johannes
* @date			2026-10-12
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef USERTYPES__H
#define USERTYPES__H

#include <string>
#include "Utilities/BasicDefinitions.h"

namespace UserTypes {

	//! the enumerations that have user types
	enum Kind { OutputVariable = 0, Node = 1, Marker = 2, numberOfKinds = 3 };
	static constexpr Index numberOfUserTypes = 4;

	//! the name the user set for user type index of kind; empty if none was set
	inline std::string& RegisteredName(Index kind, Index index)
	{
		static std::string names[numberOfKinds][numberOfUserTypes];
		return names[kind][index];
	}

	//! the name of user type index of kind: the one the user set, or UserType0, ...
	inline const char* Name(Index kind, Index index)
	{
		static const char* defaultNames[numberOfUserTypes] = { "UserType0", "UserType1", "UserType2", "UserType3" };
		const std::string& name = RegisteredName(kind, index);
		return name.empty() ? defaultNames[index] : name.c_str();
	}

} //namespace UserTypes

#endif
