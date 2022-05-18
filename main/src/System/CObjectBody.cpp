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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */

#include "Main/CSystemData.h"
#include "Linalg/RigidBodyMath.h"


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

void CObjectSuperElement::GetAccessFunctionSuperElement(AccessFunctionType accessType, const Matrix& weightingMatrix, 
	const ArrayIndex& meshNodeNumbers, Matrix& value) const
{ 
	bool useAlternativeApproach = false;
	if (EXUstd::IsOfType(accessType, AccessFunctionType::SuperElementAlternativeRotationMode))
	{
		useAlternativeApproach = true; //must be same as in CMarkerSuperElementRigid! alternative approach uses skew symmetric matrix of reference position; follows the inertia concept
		accessType = (AccessFunctionType)((Index)accessType - (Index)AccessFunctionType::SuperElementAlternativeRotationMode);
	}
	Index localReferenceNodeIndex; //local node number!!!
	bool hasReferenceFrame = HasReferenceFrame(localReferenceNodeIndex);

	//CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetAccessFunctionSuperElement"); 
	switch ((Index)accessType)
	{
	case (Index)AccessFunctionType::TranslationalVelocity_qt + (Index)AccessFunctionType::SuperElement: //global translational velocity at mesh position derivative w.r.t. all q_t: without reference frame: [0,..., 0, w0*nodeJac0, 0, ..., 0, w1*nodeJac1, 0,...]; with reference frame: [I, -A * pLocalTilde * Glocal, A*(0,...,0, w0*nodeJac0, 0,..., 0, w1*nodeJac1, ...)]
	{

		value.SetNumberOfRowsAndColumns(nDim3D, GetODE2Size());
		value.SetAll(0.);

		Matrix3D A;
		Index refFrameOffset = 0;

		if (hasReferenceFrame)
		{
			A = ((const CNodeODE2*)(GetCNode(localReferenceNodeIndex)))->GetRotationMatrix();
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
			Index nodeODE2 = GetCNode(iNode)->GetNumberOfODE2Coordinates();
			if (nodeODE2 >= CNodeRigidBody::maxDisplacementCoordinates + CNodeRigidBody::maxRotationCoordinates)
			{ 
				CHECKandTHROWstring("CObjectSuperElement::GetAccessFunctionSuperElement: MarkerSuperElement only available in case of nodes with equal or less than 7 coordinates!"); 
			}

			//use temporary jacobian structure, to get node jacobian; posJac0 MUST have already correct size, because LinkedDataMatrix cannot change size!
			ConstSizeMatrix<CNodeRigidBody::nDim3D * (CNodeRigidBody::maxDisplacementCoordinates + 
				CNodeRigidBody::maxRotationCoordinates)> posJac0(nDim3D, nodeODE2);
			LinkedDataMatrix linkedPosJac0(posJac0);
			((const CNodeODE2*)GetCNode(iNode))->GetPositionJacobian(linkedPosJac0);

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
			EXUmath::ApplyTransformation33Template< ConstSizeMatrix<CNodeRigidBody::nDim3D * (CNodeRigidBody::maxDisplacementCoordinates +
				CNodeRigidBody::maxRotationCoordinates)>>(jac, posJac0); //size=3: always 3D

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

			const CNodeRigidBody* cNode = (const CNodeRigidBody*)GetCNode(localReferenceNodeIndex);
			Index nodeODE2 = cNode->GetNumberOfODE2Coordinates();
			//if (nodeODE2 >= CNodeRigidBody::maxDisplacementCoordinates + CNodeRigidBody::maxRotationCoordinates)
			//{
			//	CHECKandTHROWstring("CObjectSuperElement::GetAccessFunctionSuperElement: MarkerSuperElement only available in case of reference node with equal or less than 7 coordinates!");
			//}


			ConstSizeMatrix<CNodeRigidBody::maxRotationCoordinates*CNodeRigidBody::nDim3D> Glocal;

			//compute: -A*pLocalTilde*GLocal
			cNode->GetGlocal(Glocal);
			EXUmath::ApplyTransformation33(RigidBodyMath::Vector2SkewMatrixTemplate(-localPosition), Glocal);
			EXUmath::ApplyTransformation33(A, Glocal);

			//now compute remaining jacobian terms for reference frame motion:
			//posJac0 MUST have already correct size, because LinkedDataMatrix cannot change size!
			ConstSizeMatrix<CNodeRigidBody::nDim3D * (CNodeRigidBody::maxDisplacementCoordinates + 
				CNodeRigidBody::maxRotationCoordinates)> posJac0(nDim3D, nodeODE2);
			LinkedDataMatrix linkedPosJac0(posJac0);
			((const CNodeODE2*)GetCNode(localReferenceNodeIndex))->GetPositionJacobian(linkedPosJac0);

			value.SetSubmatrix(posJac0, 0, 0);
			value.SetSubmatrix(Glocal, 0, CNodeRigidBody::maxDisplacementCoordinates);
		}
		break;
	}
	case (Index)AccessFunctionType::AngularVelocity_qt + (Index)AccessFunctionType::SuperElement: //global translational velocity at mesh position derivative w.r.t. all q_t: without reference frame: [0,..., 0, w0*nodeJac0, 0, ..., 0, w1*nodeJac1, 0,...]; with reference frame: [I, -A * pLocalTilde * Glocal, A*(0,...,0, w0*nodeJac0, 0,..., 0, w1*nodeJac1, ...)]
	{
		//[0, A * Glocal, A*(0, ..., 0, w0*pRefTilde0*nodeJac0, 0, ..., 0, w1*pRefTilde1*nodeJac1, ...)]
		//CHECKandTHROWstring("CObjectSuperElement:GetAccessFunctionSuperElement: AngularVelocity_qt not implemented; cannot compute jacobian for orientation");
		//break;

		CHECKandTHROW(weightingMatrix.NumberOfColumns() == 1, "CObjectFFRFreducedOrder::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, weightingMatrix must have 1 row!");
		CHECKandTHROW(!hasReferenceFrame, "CObjectSuperElement::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, only possible for ObjectGenericODE2 and ObjectFFRFreducedOrder!");
		CHECKandTHROW(GetODE2Size() == 3*GetNumberOfMeshNodes(), "CObjectSuperElement::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, only possible if all mesh nodes have dimensionality 3!");
		

		value.SetNumberOfRowsAndColumns(nDim3D, GetODE2Size());
		value.SetAll(0.);

		//++++++++++++++++++++++++++++++++++++++
		//compute global factor
		Real factor = 0; //sum w_i * |pRef_i|^2
		Matrix3D factorMatrix(3, 3, 0.);  //W in docu
		Vector3D pRef; //mesh node local reference position

		Vector3D pRef0(0);			 //this is the midpoint of the Marker, computed from reference positions

		for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
		{
			pRef0 += weightingMatrix(i, 0) * GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference);
		}

		for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
		{
			pRef = GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference) - pRef0;
			if (useAlternativeApproach)
			{
				factorMatrix -= weightingMatrix(i, 0) * RigidBodyMath::Vector2SkewMatrix(pRef) * RigidBodyMath::Vector2SkewMatrix(pRef); //negative sign!
			}
			else
			{
				factor += weightingMatrix(i, 0) * pRef.GetL2NormSquared();
			}
		}

		if (useAlternativeApproach)
		{
			factorMatrix = factorMatrix.GetInverse();
		}
		else
		{
			factor = 1. / factor; //factor is now inverted
		}

		//++++++++++++++++++++++++++++++++++++++
		for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
		{
			//assume that the first 3 coordinates of the node are the displacement coordinates!!!
			pRef = GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference) - pRef0;
			//Matrix3D jac = A * RigidBodyMath::Vector2SkewMatrix(pRef);  //A must be multiplied from left!
			Matrix3D jac = RigidBodyMath::Vector2SkewMatrix(pRef);
			if (useAlternativeApproach)
			{
				jac = weightingMatrix(i, 0) * factorMatrix * jac;
			}
			else
			{
				jac = factor * weightingMatrix(i, 0) * jac;
			}

			Index offset = meshNodeNumbers[i] * 3;
			for (Index j = 0; j < 3; j++)
			{
				for (Index k = 0; k < 3; k++)
				{
					value(j, offset + k) += jac(j, k);
				}
			}
		}

		break;

	}
	default:
		CHECKandTHROWstring("CObjectSuperElement:GetAccessFunctionSuperElement illegal accessType");
	}

}

