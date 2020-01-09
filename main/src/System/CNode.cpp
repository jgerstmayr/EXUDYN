/** ***********************************************************************************************
* @brief		Implementation for computational nodes, which define coordinates
*
* @author		Gerstmayr Johannes
* @date			2019-04-28 (generated)
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

//includes follow includes list of CSystemData.h up to CNode
#include "Linalg/BasicLinalg.h"		//includes Vector.h

#include "Main/OutputVariable.h" 
#include "Main/CData.h"		//includes ReleaseAssert.h and BasicDefinitions.h

#include "System/CMaterial.h"			//includes ReleaseAssert.h 
#include "System/CObjectBody.h"			//includes OutputVariable.h and CObject.h
#include "System/CNode.h"				//includes ReleaseAssert.h, BasicDefinitions.h, ResizeableArray.h, LinkedDataVector.h

CNode::CNode() 
{ 
	computationalData = nullptr; 
}
    
CData* CNode::GetCData() const 
{ 
	return computationalData; 
}
CData*& CNode::GetCData() 
{ 
	return computationalData; 
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CNodeODE2

const Real& CNodeODE2::GetCurrentCoordinate(Index i) const
{
	release_assert(i < GetNumberOfODE2Coordinates() && "ERROR: CNodeODE2::GetCurrentCoordinate: index out of range");

	return computationalData->currentState.ODE2Coords[globalODE2CoordinateIndex + i];
	//old, not used?: return computationalData->currentState.ODE2Coords_t[globalODE2CoordinateIndex + i - GetNumberOfODE2Coordinates()];
}

const Real& CNodeODE2::GetCurrentCoordinate_t(Index i) const
{
	release_assert(i < GetNumberOfODE2Coordinates() && "ERROR: CNodeODE2::GetCurrentCoordinate_t: index out of range");

	return computationalData->currentState.ODE2Coords_t[globalODE2CoordinateIndex + i];
}

//! get vector with current coordinates; corresponds to displacements
LinkedDataVector CNodeODE2::GetCurrentCoordinateVector() const
{
    return LinkedDataVector(computationalData->currentState.ODE2Coords, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

//! get vector with current vcoordinates_t; corresponds to velocities
LinkedDataVector CNodeODE2::GetCurrentCoordinateVector_t() const
{
    return LinkedDataVector(computationalData->currentState.ODE2Coords_t, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetInitialCoordinateVector() const
{
    return LinkedDataVector(computationalData->initialState.ODE2Coords, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetInitialCoordinateVector_t() const
{
    return LinkedDataVector(computationalData->initialState.ODE2Coords_t, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetVisualizationCoordinateVector() const
{
	return LinkedDataVector(computationalData->visualizationState.ODE2Coords, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetVisualizationCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->visualizationState.ODE2Coords_t, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CNodeData

//! read single coordinate in current configuration
const Real& CNodeData::GetCurrentCoordinate(Index i) const
{
	return computationalData->currentState.dataCoords[globalDataCoordinateIndex + i];
}

//! get vector with current coordinates; corresponds to displacements
LinkedDataVector CNodeData::GetCurrentCoordinateVector() const
{
	return LinkedDataVector(computationalData->currentState.dataCoords, globalDataCoordinateIndex, GetNumberOfDataCoordinates());
}

//! read globally stored initial coordinates (displacements)
LinkedDataVector CNodeData::GetInitialCoordinateVector() const
{
	return LinkedDataVector(computationalData->initialState.dataCoords, globalDataCoordinateIndex, GetNumberOfDataCoordinates());
}

//! read visualization coordinates (displacements)
LinkedDataVector CNodeData::GetVisualizationCoordinateVector() const
{
	return LinkedDataVector(computationalData->visualizationState.dataCoords, globalDataCoordinateIndex, GetNumberOfDataCoordinates());
}

