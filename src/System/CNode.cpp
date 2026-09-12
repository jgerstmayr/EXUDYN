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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */

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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CNodeODE2

const Real& CNodeODE2::GetCurrentCoordinate(Index i) const
{
	//CHECKandTHROW(i < GetNumberOfODE2Coordinates(), "ERROR: CNodeODE2::GetCurrentCoordinate: index out of range");

	return computationalData->currentState.ODE2Coords[globalODE2CoordinateIndex + i];
	//old, not used?: return computationalData->currentState.ODE2Coords_t[globalODE2CoordinateIndex + i - GetNumberOfODE2Coordinates()];
}

const Real& CNodeODE2::GetCurrentCoordinate_t(Index i) const
{
	//CHECKandTHROW(i < GetNumberOfODE2Coordinates(), "ERROR: CNodeODE2::GetCurrentCoordinate_t: index out of range");
	return computationalData->currentState.ODE2Coords_t[globalODE2CoordinateIndex + i];
}

const Real& CNodeODE2::GetCurrentCoordinate_tt(Index i) const
{
	//CHECKandTHROW(i < GetNumberOfODE2Coordinates(), "ERROR: CNodeODE2::GetCurrentCoordinate_tt: index out of range");
	return computationalData->currentState.ODE2Coords_tt[globalODE2CoordinateIndex + i];
}

LinkedDataVector CNodeODE2::GetInitialCoordinateVector() const
{
	return LinkedDataVector(computationalData->initialState.ODE2Coords, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetInitialCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->initialState.ODE2Coords_t, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetInitialCoordinateVector_tt() const
{
	return LinkedDataVector(computationalData->initialState.ODE2Coords_tt, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetStartOfStepCoordinateVector() const
{
	return LinkedDataVector(computationalData->startOfStepState.ODE2Coords, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetStartOfStepCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->startOfStepState.ODE2Coords_t, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetStartOfStepCoordinateVector_tt() const
{
	return LinkedDataVector(computationalData->startOfStepState.ODE2Coords_tt, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetVisualizationCoordinateVector() const
{
	return LinkedDataVector(computationalData->visualizationState.ODE2Coords, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetVisualizationCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->visualizationState.ODE2Coords_t, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}

LinkedDataVector CNodeODE2::GetVisualizationCoordinateVector_tt() const
{
	return LinkedDataVector(computationalData->visualizationState.ODE2Coords_tt, globalODE2CoordinateIndex, GetNumberOfODE2Coordinates());
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CNodeODE1

const Real& CNodeODE1::GetCurrentCoordinate(Index i) const
{
	//CHECKandTHROW(i < GetNumberOfODE1Coordinates(), "ERROR: CNodeODE1::GetCurrentCoordinate: index out of range");
	return computationalData->currentState.ODE1Coords[globalODE1CoordinateIndex + i];
}

const Real& CNodeODE1::GetCurrentCoordinate_t(Index i) const
{
	//CHECKandTHROW(i < GetNumberOfODE1Coordinates(), "ERROR: CNodeODE1::GetCurrentCoordinate_t: index out of range");
	return computationalData->currentState.ODE1Coords_t[globalODE1CoordinateIndex + i];
}

//! get vector with current coordinates; corresponds to displacements
LinkedDataVector CNodeODE1::GetCurrentCoordinateVector() const
{
	return LinkedDataVector(computationalData->currentState.ODE1Coords, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}

//! get vector with current coordinates_t; corresponds to velocities
LinkedDataVector CNodeODE1::GetCurrentCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->currentState.ODE1Coords_t, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}

LinkedDataVector CNodeODE1::GetInitialCoordinateVector() const
{
	return LinkedDataVector(computationalData->initialState.ODE1Coords, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}

LinkedDataVector CNodeODE1::GetInitialCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->initialState.ODE1Coords_t, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}

LinkedDataVector CNodeODE1::GetStartOfStepCoordinateVector() const
{
	return LinkedDataVector(computationalData->startOfStepState.ODE1Coords, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}

LinkedDataVector CNodeODE1::GetStartOfStepCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->startOfStepState.ODE1Coords_t, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}

LinkedDataVector CNodeODE1::GetVisualizationCoordinateVector() const
{
	return LinkedDataVector(computationalData->visualizationState.ODE1Coords, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}

LinkedDataVector CNodeODE1::GetVisualizationCoordinateVector_t() const
{
	return LinkedDataVector(computationalData->visualizationState.ODE1Coords_t, globalODE1CoordinateIndex, GetNumberOfODE1Coordinates());
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CNodeAE

const Real& CNodeAE::GetCurrentCoordinate(Index i) const
{
	//CHECKandTHROW(i < GetNumberOfAECoordinates(), "ERROR: CNodeAE::GetCurrentCoordinate: index out of range");
	return computationalData->currentState.AECoords[globalAEcoordinateIndex + i];
}

//! get vector with current coordinates; corresponds to displacements
LinkedDataVector CNodeAE::GetCurrentCoordinateVector() const
{
	return LinkedDataVector(computationalData->currentState.AECoords, globalAEcoordinateIndex, GetNumberOfAECoordinates());
}

LinkedDataVector CNodeAE::GetInitialCoordinateVector() const
{
	return LinkedDataVector(computationalData->initialState.AECoords, globalAEcoordinateIndex, GetNumberOfAECoordinates());
}

LinkedDataVector CNodeAE::GetStartOfStepCoordinateVector() const
{
	return LinkedDataVector(computationalData->startOfStepState.AECoords, globalAEcoordinateIndex, GetNumberOfAECoordinates());
}

LinkedDataVector CNodeAE::GetVisualizationCoordinateVector() const
{
	return LinkedDataVector(computationalData->visualizationState.AECoords, globalAEcoordinateIndex, GetNumberOfAECoordinates());
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CNodeData

//! read single coordinate in current configuration
const Real& CNodeData::GetCurrentCoordinate(Index i) const
{
	return computationalData->currentState.dataCoords[globalDataCoordinateIndex + i];
}

//! get vector with current coordinates
LinkedDataVector CNodeData::GetCurrentCoordinateVector() const
{
	return LinkedDataVector(computationalData->currentState.dataCoords, globalDataCoordinateIndex, GetNumberOfDataCoordinates());
}

//! read globally stored initial coordinates
LinkedDataVector CNodeData::GetInitialCoordinateVector() const
{
	return LinkedDataVector(computationalData->initialState.dataCoords, globalDataCoordinateIndex, GetNumberOfDataCoordinates());
}

//! read globally stored start of step coordinates 
LinkedDataVector CNodeData::GetStartOfStepCoordinateVector() const
{
	return LinkedDataVector(computationalData->startOfStepState.dataCoords, globalDataCoordinateIndex, GetNumberOfDataCoordinates());
}

//! read visualization coordinates
LinkedDataVector CNodeData::GetVisualizationCoordinateVector() const
{
	return LinkedDataVector(computationalData->visualizationState.dataCoords, globalDataCoordinateIndex, GetNumberOfDataCoordinates());
}

