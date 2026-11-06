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
#include "Main/TemporaryComputationData.h"

//! dv/dq of a point of a rigid body node and domega/dq (#3201)
bool RigidBodyNodeVelocity_q(const CNodeODE2& node, const Vector3D& localPosition, Matrix& velocity_q, Matrix& angularVelocity_q)
{
	const Node::Type type = node.GetType();
	if (!(type & Node::RigidBody)) { return false; }
	const bool eulerParameters = (type & Node::RotationEulerParameters) != 0;
	const bool taitBryan = (type & Node::RotationRxyz) != 0;
	const bool planar = (type & Node::Orientation2D) != 0;
	const bool lieGroup = (type & Node::RotationRotationVector) != 0;
	if (!eulerParameters && !taitBryan && !planar && !lieGroup) { return false; }

	const Index n = node.GetNumberOfODE2Coordinates();
	velocity_q.SetNumberOfRowsAndColumns(3, n);
	velocity_q.SetAll(0.);
	angularVelocity_q.SetNumberOfRowsAndColumns(3, n);
	angularVelocity_q.SetAll(0.);
	if (lieGroup) //the Lie group node (#3217): the velocity coordinates are the local angular velocity omega, a change of
	{             //the rotation coordinates composes the rotation locally, A Exp(delta), so d(A v)/d delta = -A v~
		const Index nDisplacement = ((const CNodeRigidBody&)node).GetNumberOfDisplacementCoordinates();
		const Matrix3D A = node.GetRotationMatrix();
		const Vector3D omega = ((const CNodeRigidBody&)node).GetAngularVelocityLocal();
		const Matrix3D angular = -1. * (A * RigidBodyMath::Vector2SkewMatrix(omega));
		const Matrix3D linear = -1. * (A * RigidBodyMath::Vector2SkewMatrix(omega.CrossProduct(localPosition)));
		for (Index r = 0; r < 3; r++)
		{
			for (Index k = 0; k < 3; k++)
			{
				angularVelocity_q(r, nDisplacement + k) = angular(r, k);
				velocity_q(r, nDisplacement + k) = linear(r, k);
			}
		}
		return true;
	}
	if (planar) //omega = theta_t e_z does not depend on theta; d(A p)/dtheta = e_z x (A p)
	{
		const Real theta_t = node.GetCurrentCoordinateVector_t()[2];
		const Vector3D point = node.GetRotationMatrix() * localPosition;
		velocity_q(0, 2) = -theta_t * point[0]; //theta_t e_z x (e_z x A p)
		velocity_q(1, 2) = -theta_t * point[1];
		return true;
	}

	const CNodeRigidBody& rigidNode = (const CNodeRigidBody&)node;
	const Index nDisplacement = rigidNode.GetNumberOfDisplacementCoordinates();
	const Index nRotation = rigidNode.GetNumberOfRotationCoordinates();
	CSVector4D theta = rigidNode.GetRotationParameters();
	LinkedDataVector theta_t = rigidNode.GetRotationParameters_t();
	//as the markers give them: the local angular velocity Glocal theta_t, the global one A Glocal theta_t, the velocity of
	//the point A (omega_local x localPosition) - for Euler parameters off the unit sphere these differ from G theta_t,
	//and the numerical differentiation of the system sees the forms of the markers
	auto Velocities = [&](const CSVector4D& rotation, Vector3D& velocity, Vector3D& angularVelocity)
	{
		Matrix3D A;
		ConstSizeMatrix<CNodeRigidBody::maxRotationCoordinates * 3> Glocal;
		if (eulerParameters)
		{
			A = RigidBodyMath::EP2RotationMatrix(rotation);
			Glocal = RigidBodyMath::EP2GlocalTemplate<CSVector4D>(rotation);
		}
		else
		{
			A = RigidBodyMath::RotXYZ2RotationMatrix(rotation);
			Glocal = RigidBodyMath::RotXYZ2GlocalTemplate<CSVector4D>(rotation);
		}
		Vector3D angularVelocityLocal;
		EXUmath::MultMatrixVector(Glocal, theta_t, angularVelocityLocal);
		angularVelocity = A * angularVelocityLocal;
		velocity = A * angularVelocityLocal.CrossProduct(localPosition); //without the velocity of the node, which is constant
	};
	Vector3D velocity0, angularVelocity0, velocity1, angularVelocity1;
	Velocities(theta, velocity0, angularVelocity0);
	for (Index k = 0; k < nRotation; k++)
	{
		const Real eps = 1e-7 * EXUstd::Maximum(0.01, std::fabs(theta[k])); //the defaults of NumericalDifferentiationSettings
		CSVector4D theta1 = theta;
		theta1[k] += eps;
		Velocities(theta1, velocity1, angularVelocity1);
		for (Index r = 0; r < 3; r++)
		{
			velocity_q(r, nDisplacement + k) = (velocity1[r] - velocity0[r]) / eps;
			angularVelocity_q(r, nDisplacement + k) = (angularVelocity1[r] - angularVelocity0[r]) / eps;
		}
	}
	return true;
}

//! the derivative along the Euler parameters of a point of a rigid body node and of its rotation matrix (#3201)
bool RigidBodyNodeRadialKinematics(const CNodeODE2& node, const Vector3D& localPosition, Vector3D& position_r, Matrix3D& rotation_r, Vector& radial_q)
{
	if (!(node.GetType() & Node::RotationEulerParameters)) { return false; }
	const CNodeRigidBody& rigidNode = (const CNodeRigidBody&)node;
	rotation_r = 2. * (rigidNode.GetRotationMatrix() - EXUmath::unitMatrix3D);
	position_r = rotation_r * localPosition;
	const CSVector4D e = rigidNode.GetRotationParameters();
	const Index nDisplacement = rigidNode.GetNumberOfDisplacementCoordinates();
	radial_q.SetNumberOfItems(rigidNode.GetNumberOfODE2Coordinates());
	radial_q.SetAll(0.);
	const Real ee = e * e;
	for (Index k = 0; k < e.NumberOfItems(); k++) { radial_q[nDisplacement + k] = e[k] / ee; }
	return true;
}

//! the object's own local Jacobian of its algebraic equations (#3201): a body with algebraic equations (ObjectGenericDAE,
//! the Euler parameter constraint attached to a body); moved from CSystem::ComputeObjectJacobianAE
void CObject::ComputeLocalJacobianAE(const CSystemData& systemData, TemporaryComputationData& temp, bool& usesVelocityLevel,
	JacobianType::Type& filledJacobians) const
{
	usesVelocityLevel = false;
	filledJacobians = JacobianType::_None;
	CHECKandTHROW((Index)GetType() & (Index)CObjectType::Body, "CObject::ComputeLocalJacobianAE: object type not implemented",
		ExudynNotImplementedError);
	if (GetAlgebraicEquationsSize())
	{
		filledJacobians = GetAvailableJacobians();
		ComputeJacobianAE(temp.localJacobianAE_ODE2, temp.localJacobianAE_ODE2_t, temp.localJacobianAE_ODE1, temp.localJacobianAE_AE); //all blocks are set
	}
}

//! the object's own local Jacobian of its ODE2 rows (#3201): ComputeJacobianODE2_ODE2, for every object that is not a
//! connector
bool CObject::ComputeLocalJacobianODE2(const CSystemData& systemData, TemporaryComputationData& temp,
	const NumericalDifferentiationSettings& numDiff, Real factorODE2, Real factorODE2_t, const ArrayIndex& ltgODE2) const
{
	ComputeJacobianODE2_ODE2(temp.jacobianODE2Container, temp.jacobianTemp, factorODE2, factorODE2_t, ltgODE2);
	return true;
}


CNode*& CObject::GetCNode(Index localIndex)
{
	return cSystemData->GetCNodes()[GetNodeNumber(localIndex)];
}

const CNode* CObject::GetCNode(Index localIndex) const
{
	return cSystemData->GetCNodes()[GetNodeNumber(localIndex)];
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Real CObjectBody::ComputeKineticEnergyFromMassMatrix(ConfigurationType configuration, const char* itemName) const
{
	if (configuration != ConfigurationType::Current)
	{
		PyError(STDstring(itemName) + ": KineticEnergy is computed from the mass matrix of the current state, so in the current configuration only",
			PyErrorType::notImplementedError);
	}
	Index n = 0;
	for (Index i = 0; i < GetNumberOfNodes(); i++) { n += GetCNode(i)->GetNumberOfODE2Coordinates(); }
	Vector qt(n); //the velocities of the object's ODE2 coordinates, in the order of its nodes
	Index k = 0;
	for (Index i = 0; i < GetNumberOfNodes(); i++)
	{
		if (GetCNode(i)->GetNumberOfODE2Coordinates() != 0)
		{
			for (Real v : ((const CNodeODE2*)GetCNode(i))->GetCoordinateVector_t(configuration)) { qt[k++] = v; }
		}
	}
	ArrayIndex ltg; //local indices, so that sparse triplets are local as well
	for (Index i = 0; i < qt.NumberOfItems(); i++) { ltg.Append(i); }

	EXUmath::MatrixContainer massMatrix;
	massMatrix.SetUseDenseMatrix(true);
	ComputeMassMatrix(massMatrix, ltg, false);

	Real energy = 0.;
	if (massMatrix.UseDenseMatrix())
	{
		const Matrix& M = massMatrix.GetInternalDenseMatrix();
		for (Index i = 0; i < M.NumberOfRows(); i++)
		{
			for (Index j = 0; j < M.NumberOfColumns(); j++) { energy += qt[i] * M(i, j) * qt[j]; }
		}
	}
	else
	{
		for (const EXUmath::Triplet& triplet : massMatrix.GetInternalSparseTripletMatrix().GetTriplets())
		{
			energy += qt[triplet.row()] * triplet.value() * qt[triplet.col()];
		}
	}
	return 0.5*energy;
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
	const ArrayIndex& meshNodeNumbers, const Vector3D& localOffset, Matrix& value, const Matrix3D& rotTangentCorrection, bool alternativeRotationMode) const
{ 
	const bool useAlternativeApproach = alternativeRotationMode; //must be same as in CMarkerSuperElementRigid! alternative approach uses skew symmetric matrix of reference position; follows the inertia concept
	Index localReferenceNodeIndex; //local node number!!!
	bool hasReferenceFrame = HasReferenceFrame(localReferenceNodeIndex);

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
		//without reference frame, the localOffset does not influence position jacobian!
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

			Vector3D localPosition;// ({ 0,0,0 });
			localPosition = localOffset;
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

		//according to theDoc, localOffset has no influence on rotations, as this position also does not influence angular velocity

		CHECKandTHROW(weightingMatrix.NumberOfColumns() == 1, "CObjectFFRFreducedOrder::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, weightingMatrix must have 1 row!", ExudynValueError);
		CHECKandTHROW(!hasReferenceFrame, "CObjectSuperElement::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, only possible for ObjectGenericODE2 and ObjectFFRFreducedOrder!", ExudynNotImplementedError);
		CHECKandTHROW(GetODE2Size() == 3*GetNumberOfMeshNodes(), "CObjectSuperElement::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, only possible if all mesh nodes have dimensionality 3!", ExudynNotImplementedError);
		

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
			factorMatrix = rotTangentCorrection*factorMatrix.GetInverse();
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

//! the ODE2 coordinates of the nodes and their velocities (#2995)
void CObjectBody::GatherCoordinates(VectorBase<Real>& coordinates, VectorBase<Real>& coordinates_t, ConfigurationType configuration,
	bool addReference) const
{
	Index n = 0;
	for (Index i = 0; i < GetNumberOfNodes(); i++) { n += GetCNode(i)->GetNumberOfODE2Coordinates(); }
	coordinates.SetNumberOfItems(n);
	coordinates_t.SetNumberOfItems(n);
	Index cnt = 0;
	for (Index i = 0; i < GetNumberOfNodes(); i++)
	{
		const CNodeODE2* node = (const CNodeODE2*)GetCNode(i);
		Index nc = node->GetNumberOfODE2Coordinates();
		if (configuration == ConfigurationType::Reference)
		{
			LinkedDataVector reference(node->GetReferenceCoordinateVector());
			for (Index k = 0; k < nc; k++)
			{
				coordinates[cnt + k] = addReference ? reference[k] : 0.;
				coordinates_t[cnt + k] = 0.;
			}
		}
		else
		{
			LinkedDataVector u(node->GetCoordinateVector(configuration));
			LinkedDataVector u_t(node->GetCoordinateVector_t(configuration));
			for (Index k = 0; k < nc; k++)
			{
				coordinates[cnt + k] = u[k];
				coordinates_t[cnt + k] = u_t[k];
			}
			if (addReference)
			{
				LinkedDataVector reference(node->GetReferenceCoordinateVector());
				for (Index k = 0; k < nc; k++) { coordinates[cnt + k] += reference[k]; }
			}
		}
		cnt += nc;
	}
}
