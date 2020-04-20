/** ***********************************************************************************************
* @brief		CObjectBody implementation
* @details		Details:
 				- base class for computational body
*
* @author		Gerstmayr Johannes
* @date			2019-04-126 (generated)
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

#include "Main/CSystemData.h"


CNode*& CObject::GetCNode(Index localIndex)
{
	return cSystemData->GetCNodes()[GetNodeNumber(localIndex)];
}

const CNode* CObject::GetCNode(Index localIndex) const
{
	return cSystemData->GetCNodes()[GetNodeNumber(localIndex)];
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CObjectBody::Print(std::ostream& os) const {
	os << "CObjectBody(";
	for (Index i = 0; i < GetNumberOfNodes(); i++) {
		os << "Node" << i << "=" << *GetCNode(i);
		if (i < GetNumberOfNodes() - 1) { os << ", "; }
	}
	os << "):";
	CObject::Print(os);
}

void CObjectBody::GetODE2LocalToGlobalCoordinates(ArrayIndex& ltg) const
{
	ltg.SetNumberOfItems(GetODE2Size()); //do not reset data
	Index cnt = 0;

	//for (CNode* node : nodes)
	//loop over local node numbers:
	for (Index nodeNumber = 0; nodeNumber < GetNumberOfNodes(); nodeNumber++)
	{
		const CNode* node = GetCNode(nodeNumber);
		Index n = node->GetNumberOfODE2Coordinates();

		for (Index j = 0; j < n; j++)
		{
			ltg[cnt++] = node->GetGlobalODE2CoordinateIndex() + j;
		}
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! get current algebraic coordinate with local index
Real CObjectConstraint::GetCurrentAEcoordinate(Index localIndex) const
{ 
	return cSystemData->GetCData().GetCurrent().AECoords[globalAECoordinateIndex + localIndex]; 
}

