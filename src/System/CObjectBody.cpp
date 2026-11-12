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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//THE INTERFACE OF THE SUPERELEMENT MARKERS (#3450): the sums over the mesh nodes, for all superelements;
//ObjectFFRFreducedOrder computes them from the modes instead (#2184)

void CObjectSuperElement::GetReferenceFrame(Vector3D& framePosition, Matrix3D& frameRotationMatrix, Vector3D& frameVelocity,
	Vector3D& frameAngularVelocityLocal, ConfigurationType configuration) const
{
	Index localReferenceNodeIndex; //local node number
	if (HasReferenceFrame(localReferenceNodeIndex))
	{
		const CNodeRigidBody* frameNode = (const CNodeRigidBody*)GetCNode(localReferenceNodeIndex);
		framePosition = frameNode->GetPosition(configuration);
		frameRotationMatrix = frameNode->GetRotationMatrix(configuration);
		if (configuration != ConfigurationType::Reference)
		{
			frameVelocity = frameNode->GetVelocity(configuration);
			frameAngularVelocityLocal = frameNode->GetAngularVelocityLocal(configuration);
		}
		else
		{
			frameVelocity.SetAll(0.);
			frameAngularVelocityLocal.SetAll(0.);
		}
	}
	else
	{
		framePosition.SetAll(0.);
		frameRotationMatrix = EXUmath::unitMatrix3D;
		frameVelocity.SetAll(0.);
		frameAngularVelocityLocal.SetAll(0.);
	}
}

void CObjectSuperElement::InitializeMarkerInterface(const ArrayIndex& meshNodeNumbers, const Vector& weightingFactors, bool computeRotation,
	bool alternativeApproach, SuperElementMarkerInterface& markerInterface) const
{
	markerInterface.hasRotation = computeRotation;
	markerInterface.alternativeApproach = alternativeApproach;
	markerInterface.positionModes.SetNumberOfRowsAndColumns(0, 0);
	markerInterface.rotationModes.SetNumberOfRowsAndColumns(0, 0);

	Vector3D& p0 = markerInterface.referencePosition;
	p0.SetAll(0.);
	for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
	{
		p0 += weightingFactors[i] * GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference);
	}

	if (computeRotation)
	{
		Real factor = 0.; //W = sum w_i * |p_i|^2
		Matrix3D factorMatrix(3, 3, 0.); //W = -sum w_i * pTilde_i^2, alternative approach
		for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
		{
			Vector3D p = GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference) - p0;
			if (alternativeApproach)
			{
				factorMatrix -= weightingFactors[i] * RigidBodyMath::Vector2SkewMatrix(p) * RigidBodyMath::Vector2SkewMatrix(p);
			}
			else
			{
				factor += weightingFactors[i] * p.GetL2NormSquared();
			}
		}
		markerInterface.rotationFactor = factor;
		if (alternativeApproach) { markerInterface.rotationMatrixInverse = factorMatrix.GetInverse(); } //checked to be regular in CheckPreAssembleConsistency
	}
	markerInterface.isInitialized = true;
}

Vector3D CObjectSuperElement::GetMarkerLocalPosition(const SuperElementMarkerInterface& markerInterface, const ArrayIndex& meshNodeNumbers,
	const Vector& weightingFactors, ConfigurationType configuration) const
{
	Vector3D position(0.);
	for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
	{
		position += weightingFactors[i] * GetMeshNodeLocalPosition(meshNodeNumbers[i], configuration);
	}
	return position;
}

Vector3D CObjectSuperElement::GetMarkerLocalVelocity(const SuperElementMarkerInterface& markerInterface, const ArrayIndex& meshNodeNumbers,
	const Vector& weightingFactors, ConfigurationType configuration) const
{
	Vector3D velocity(0.);
	for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
	{
		velocity += weightingFactors[i] * GetMeshNodeLocalVelocity(meshNodeNumbers[i], configuration);
	}
	return velocity;
}

Vector3D CObjectSuperElement::GetMarkerLocalRotation(const SuperElementMarkerInterface& markerInterface, const ArrayIndex& meshNodeNumbers,
	const Vector& weightingFactors, ConfigurationType configuration) const
{
	CHECKandTHROW(markerInterface.hasRotation, "CObjectSuperElement::GetMarkerLocalRotation: the marker interface has no rotation");
	Vector3D rotation(0.);
	if (configuration == ConfigurationType::Reference) { return rotation; }

	const Vector3D& p0 = markerInterface.referencePosition;
	//sum_j w_j u_j: a translation of the mesh nodes that are not symmetric about p0 would otherwise rotate the marker
	Vector3D translation = CObjectSuperElement::GetMarkerLocalPosition(markerInterface, meshNodeNumbers, weightingFactors, configuration) - p0;
	for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
	{
		Vector3D referencePosition = GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference);
		rotation += weightingFactors[i] * (referencePosition - p0).CrossProduct(
			GetMeshNodeLocalPosition(meshNodeNumbers[i], configuration) - referencePosition - translation);
	}
	return markerInterface.ApplyRotationWeighting(rotation);
}

Vector3D CObjectSuperElement::GetMarkerLocalAngularVelocity(const SuperElementMarkerInterface& markerInterface, const ArrayIndex& meshNodeNumbers,
	const Vector& weightingFactors, ConfigurationType configuration) const
{
	CHECKandTHROW(markerInterface.hasRotation, "CObjectSuperElement::GetMarkerLocalAngularVelocity: the marker interface has no rotation");
	const Vector3D& p0 = markerInterface.referencePosition;
	Vector3D translation = CObjectSuperElement::GetMarkerLocalVelocity(markerInterface, meshNodeNumbers, weightingFactors, configuration);
	Vector3D angularVelocity(0.);
	for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
	{
		angularVelocity += weightingFactors[i] * (GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference) - p0).CrossProduct(
			GetMeshNodeLocalVelocity(meshNodeNumbers[i], configuration) - translation);
	}
	return markerInterface.ApplyRotationWeighting(angularVelocity);
}

void CObjectSuperElement::GetAccessFunctionSuperElement(AccessFunctionType accessType, const SuperElementMarkerInterface& markerInterface,
	const ArrayIndex& meshNodeNumbers, const Vector& weightingFactors, const Vector3D& localOffset,
	const Matrix3D& rotTangentCorrection, Matrix& value) const
{
	Index localReferenceNodeIndex; //local node number
	const bool hasReferenceFrame = HasReferenceFrame(localReferenceNodeIndex);
	typedef ConstSizeMatrix<CNodeRigidBody::nDim3D * (CNodeRigidBody::maxDisplacementCoordinates + CNodeRigidBody::maxRotationCoordinates)> NodeJacobian;

	value.SetNumberOfRowsAndColumns(nDim3D, GetODE2Size());
	value.SetAll(0.);

	switch ((Index)accessType)
	{
	case (Index)AccessFunctionType::TranslationalVelocity_qt + (Index)AccessFunctionType::SuperElement:
	//without reference frame: [w0*nodeJac0, w1*nodeJac1, ...] at the coordinates of the mesh nodes;
	//with reference frame: [I, -A * pLocalTilde * Glocal, A*(w0*nodeJac0, w1*nodeJac1, ...)]
	{
		Matrix3D A = EXUmath::unitMatrix3D;
		Index referenceFrameOffset = 0;
		if (hasReferenceFrame)
		{
			A = ((const CNodeODE2*)(GetCNode(localReferenceNodeIndex)))->GetRotationMatrix();
			referenceFrameOffset = 1;
		}

		for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
		{
			Index iNode = meshNodeNumbers[i] + referenceFrameOffset;
			Index nodeODE2 = GetCNode(iNode)->GetNumberOfODE2Coordinates();
			CHECKandTHROW(nodeODE2 < CNodeRigidBody::maxDisplacementCoordinates + CNodeRigidBody::maxRotationCoordinates,
				"CObjectSuperElement::GetAccessFunctionSuperElement: MarkerSuperElement only available for nodes with less than 7 coordinates");

			//the node Jacobian: posJac0 needs its size before it is linked
			NodeJacobian posJac0(nDim3D, nodeODE2);
			LinkedDataMatrix linkedPosJac0(posJac0);
			((const CNodeODE2*)GetCNode(iNode))->GetPositionJacobian(linkedPosJac0);

			Matrix3D jac = A;
			jac *= weightingFactors[i];
			EXUmath::ApplyTransformation33Template<NodeJacobian>(jac, posJac0);
			value.SetSubmatrix(posJac0, 0, GetLocalODE2CoordinateIndexPerNode(iNode));
		}

		if (hasReferenceFrame)
		{
			Vector3D localPosition = localOffset + GetMarkerLocalPosition(markerInterface, meshNodeNumbers, weightingFactors, ConfigurationType::Current);
			const CNodeRigidBody* cNode = (const CNodeRigidBody*)GetCNode(localReferenceNodeIndex);

			//-A*pLocalTilde*GLocal
			ConstSizeMatrix<CNodeRigidBody::maxRotationCoordinates*CNodeRigidBody::nDim3D> Glocal;
			cNode->GetGlocal(Glocal);
			EXUmath::ApplyTransformation33(RigidBodyMath::Vector2SkewMatrixTemplate(-localPosition), Glocal);
			EXUmath::ApplyTransformation33(A, Glocal);

			NodeJacobian posJac0(nDim3D, cNode->GetNumberOfODE2Coordinates());
			LinkedDataMatrix linkedPosJac0(posJac0);
			cNode->GetPositionJacobian(linkedPosJac0);

			value.SetSubmatrix(posJac0, 0, 0);
			value.SetSubmatrix(Glocal, 0, CNodeRigidBody::maxDisplacementCoordinates);
		}
		break;
	}
	case (Index)AccessFunctionType::AngularVelocity_qt + (Index)AccessFunctionType::SuperElement:
	//[W^{-1}*w0*pRefTilde0, W^{-1}*w1*pRefTilde1, ...] at the coordinates of the mesh nodes; the localOffset has no influence
	{
		CHECKandTHROW(!hasReferenceFrame, "CObjectSuperElement::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, only possible for ObjectGenericODE2 and ObjectFFRFreducedOrder", ExudynNotImplementedError);
		CHECKandTHROW(GetODE2Size() == 3*GetNumberOfMeshNodes(), "CObjectSuperElement::GetAccessFunctionSuperElement: AccessFunctionType::AngularVelocity_qt, only possible if all mesh nodes have dimensionality 3", ExudynNotImplementedError);
		CHECKandTHROW(markerInterface.hasRotation, "CObjectSuperElement::GetAccessFunctionSuperElement: the marker interface has no rotation");

		Matrix3D weighting = EXUmath::unitMatrix3D; //rotTangentCorrection*W^{-1} or W^{-1}
		if (markerInterface.alternativeApproach) { weighting = rotTangentCorrection * markerInterface.rotationMatrixInverse; }
		else { weighting *= 1. / markerInterface.rotationFactor; }

		for (Index i = 0; i < meshNodeNumbers.NumberOfItems(); i++)
		{
			Vector3D p = GetMeshNodeLocalPosition(meshNodeNumbers[i], ConfigurationType::Reference) - markerInterface.referencePosition;
			Matrix3D jac = weightingFactors[i] * weighting * RigidBodyMath::Vector2SkewMatrix(p);
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
