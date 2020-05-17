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
#include "Utilities/RigidBodyMath.h"


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

void CObjectSuperElement::GetAccessFunctionSuperElement(AccessFunctionType accessType, const Matrix& weightingMatrix, const ArrayIndex& meshNodeNumbers, Matrix& value) const
{ 
	//CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetAccessFunctionSuperElement"); 
	switch ((Index)accessType)
	{
	case (Index)AccessFunctionType::TranslationalVelocity_qt + (Index)AccessFunctionType::SuperElement: //global translational velocity at mesh position derivative w.r.t. all q_t: without reference frame: [0,..., 0, w0*nodeJac0, 0, ..., 0, w1*nodeJac1, 0,...]; with reference frame: [I, -A * pLocalTilde * Glocal, A*(0,...,0, w0*nodeJac0, 0,..., 0, w1*nodeJac1, ...)]
	{

		value.SetNumberOfRowsAndColumns(nDim3D, GetODE2Size());
		value.SetAll(0.);

		Matrix3D A;
		Index referenceNodeIndex;
		bool hasReferenceFrame = HasReferenceFrame(referenceNodeIndex);
		Index refFrameOffset = 0;

		if (hasReferenceFrame)
		{
			A = ((const CNodeODE2*)(GetCNode(referenceNodeIndex)))->GetRotationMatrix();
			refFrameOffset++;
		}
		else
		{
			A = EXUmath::unitMatrix3D;
		}

		//Index cOffset = 0; //coordinates offset
		for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
		{
			Index iNode = meshNodeNumbers[i] + refFrameOffset;

			if (GetCNode(iNode)->GetNumberOfODE2Coordinates() >= CNodeRigidBody::maxDisplacementCoordinates + CNodeRigidBody::maxRotationCoordinates) 
			{ 
				CHECKandTHROWstring("CObjectSuperElement::GetAccessFunctionSuperElement: MarkerSuperElement only available in case of nodes with equal or less than 7 coordinates!"); 
			}

			//use temporary jacobian structure, to get node jacobian
			ConstSizeMatrix<CNodeRigidBody::nDim3D * (CNodeRigidBody::maxDisplacementCoordinates + CNodeRigidBody::maxRotationCoordinates)> posJac0;
			((const CNodeODE2*)GetCNode(iNode))->GetPositionJacobian(posJac0);

			//assume that the first 3 coordinates of the node are the displacement coordinates!!!
			Matrix3D jac = A;
			if (weightingMatrix.NumberOfColumns() == 1)
			{
				jac *= weightingMatrix(i, 0);
			}
			else
			{
				for (Index j = 0; j < 3; j++)
				{
					for (Index k = 0; k < 3; k++)
					{
						jac(j, k) *= weightingMatrix(i, k); //add weighting to columns, because every column corresponds to local x,y and z direction (weigthing effects needed on local coordinates)
					}
				}
			}
			EXUmath::ApplyTransformation<3>(jac, posJac0); //size=3: always 3D

			Index offset = GetLocalODE2CoordinateIndexPerNode(iNode); //gives correct coordinates also in case of referenceFrame node
			//pout << "offsetFFRF" << i << "=" << offset << "\n";
			value.SetSubmatrix(posJac0, 0, offset);
		}
		//pout << "posJac=" << markerData.positionJacobian << "\n";

		if (hasReferenceFrame)
		{
			//\partial vMarker / \partial q_t = [I, -A * pLocalTilde * Glocal, A*(w0*nodeJac0, w1*nodeJac1, ...)]

			Vector3D localPosition({ 0,0,0 });
			for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
			{
				if (weightingMatrix.NumberOfColumns() == 1)
				{
					localPosition += weightingMatrix(i, 0) * GetMeshNodeLocalPosition(meshNodeNumbers[i]); 
				}
				else
				{
					for (Index j = 0; j < 3; j++)
					{
						localPosition[j] += weightingMatrix(i, j) * GetMeshNodeLocalPosition(meshNodeNumbers[i])[j]; 
					}
				}
			}

			const CNodeRigidBody* cNode = (const CNodeRigidBody*)GetCNode(referenceNodeIndex);

			ConstSizeMatrix<CNodeRigidBody::maxRotationCoordinates*CNodeRigidBody::nDim3D> Glocal;

			//compute: -A*pLocalTilde*GLocal
			cNode->GetGlocal(Glocal);
			EXUmath::ApplyTransformation<3>(RigidBodyMath::Vector2SkewMatrixTemplate(-localPosition), Glocal);
			EXUmath::ApplyTransformation<3>(A, Glocal);

			//now compute remaining jacobian terms for reference frame motion:
			ConstSizeMatrix<CNodeRigidBody::nDim3D * (CNodeRigidBody::maxDisplacementCoordinates + CNodeRigidBody::maxRotationCoordinates)> posJac0;
			((const CNodeODE2*)GetCNode(referenceNodeIndex))->GetPositionJacobian(posJac0);

			value.SetSubmatrix(posJac0, 0, 0);
			value.SetSubmatrix(Glocal, 0, CNodeRigidBody::maxDisplacementCoordinates);
		}
		break;
	}
	default:
		CHECKandTHROWstring("CObjectSuperElement:GetAccessFunctionSuperElement illegal accessType");
	}

}

