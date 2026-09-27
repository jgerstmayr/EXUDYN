/** ***********************************************************************************************
* @brief        what the checkPreAssembleConsistencies*.cpp files share: the two tolerances and
*               the IsInRange helper. checkPreAssembleConsistencies.cpp is one file
*               per item kind (#2554).
*
* @author       Gerstmayr Johannes
* @date         2020-12-09 (generated)
* @date         2026-09-20 (split off)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef CHECKPREASSEMBLECONSISTENCIES__H
#define CHECKPREASSEMBLECONSISTENCIES__H

const Real toleranceChecks = sqrt(EXUstd::EPSILONREAL) * 0.01;
const Real toleranceNorm = EXUstd::EPSILONREAL * 100.; //should not be too small, as small errors may already cause problems!

//! check if parameter is in range and return true/false; if out of range, set errorString and return false
template<typename TValue>
bool IsInRange(STDstring& errorString, TValue parameter, TValue minValue, TValue maxValue,
	const STDstring& preString, const STDstring& parameterName)
{
	if (parameter < minValue || parameter > maxValue)
	{
		errorString = preString + "parameter " + parameterName + " is out of valid range [" +
			EXUstd::ToString(minValue) + "," + EXUstd::ToString(maxValue) + "]";
		return false;
	}
	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif
