/** ***********************************************************************************************
* @brief		CObjectConnector implementation
* @details		Details:
 				- implementation for connectors and constraints
*
* @author		Gerstmayr Johannes
* @date			2021-12-23 (generated)
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
//#include "Linalg/RigidBodyMath.h"
#include "System/CObjectConnector.h"
#include "Main/TemporaryComputationData.h"

//! function to compute jacobian for connectors having a simple structure with a local jacobian 
//! using K=d(F)/(dq), D=d(F)/(dq_t) ==> localJac must be: localJac = factorODE2*K + factorODE_t*D
//! jacobianODE2 is computed using the marker jacobians and the jacobianDerivative stored in markerData
//! dense mode is used here; if activeConnector=false, jacobian becomes a zeros matrix
void CObjectConnector::ComputeJacobianODE2_ODE2generic(ResizableMatrix& localJac, EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp,
	Real factorODE2, Real factorODE2_t, Index objectNumber, const MarkerDataStructure& markerData, 
	bool activeConnector, bool isCoordinateConnector, bool hasRotationJacobian) const
{
	const ResizableMatrix& jac0 = (isCoordinateConnector ? markerData.GetMarkerData(0).jacobian : markerData.GetMarkerData(0).positionJacobian);
	const ResizableMatrix& jac1 = (isCoordinateConnector ? markerData.GetMarkerData(1).jacobian : markerData.GetMarkerData(1).positionJacobian);

	//CHECKandTHROWstring("ERROR: illegal call to CObjectConnectorCartesianSpringDamper::ComputeJacobianODE2_ODE2");
	Index n0 = jac0.NumberOfColumns();
	Index n1 = jac1.NumberOfColumns();

	CHECKandTHROW(hasRotationJacobian == false, "CObjectConnector::ComputeJacobianODE2_ODE2generic: not implemented for rotationJacobian", ExudynNotImplementedError);

	jacobianODE2.SetUseDenseMatrix();
	jacobianODE2.GetInternalDenseMatrix().SetNumberOfRowsAndColumns(n0 + n1, n0 + n1);
	//jacobianODE2.GetInternalDenseMatrix().SetAll(0.); //not needed, everything is filled
	if (activeConnector) //this function is only called manually, but CSystem checks already earlier if IsActive() = false
	{
		//compute jacobian:
		//jacobian part 1:
		//[-Jpos0.T*F,pos0*Jpos0 , -Jpos0.T*F,pos1*Jpos1]
		//[+Jpos1.T*F,pos0*Jpos0 , +Jpos1.T*F,pos1*Jpos1]
		//F,pos0 = -K, F,pos1=K
		//[ Jpos0.T*K*Jpos0 , -Jpos0.T*K*Jpos1]
		//[-Jpos1.T*K*Jpos0 , +Jpos1.T*K*Jpos1]

		if (n0)
		{
			//J_pos0.T*K 
			//pout << "1: " << jac0 << ",\n" << localJac << "\n";
			EXUmath::MultMatrixTransposedMatrixTemplate(jac0, localJac, temp.matrix0);
			//J_pos0.T*K*Jpos0
			//EXUmath::MultMatrixMatrixTemplate<ResizableMatrix, ResizableMatrix, ResizableMatrix>(temp.matrix0, jac0, temp.matrix1);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac0, jacobianODE2.GetInternalDenseMatrix(), 0, 0);

			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac0.GetTransposed()*localJac*jac0, 0, 0, 1.);
		}
		if (n1)
		{
			//pout << "2: " << jac1 << ",\n" << localJac << "\n";
			//J_pos1.T*K*Jpos1
			EXUmath::MultMatrixTransposedMatrixTemplate(jac1, localJac, temp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac1, jacobianODE2.GetInternalDenseMatrix(), n0, n0);
			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac1.GetTransposed()*localJac*jac1, n0, n0, 1.);
		}
		if (n0 != 0 && n1 != 0)
		{
			localJac *= -1.;
			//-J_pos0.T*K*Jpos1
			EXUmath::MultMatrixTransposedMatrixTemplate(jac0, localJac, temp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac1, jacobianODE2.GetInternalDenseMatrix(), 0, n0);

			//-J_pos1.T*K*Jpos0
			EXUmath::MultMatrixTransposedMatrixTemplate(jac1, localJac, temp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac0, jacobianODE2.GetInternalDenseMatrix(), n0, 0);

			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac0.GetTransposed()*localJac*jac1, 0, n0, 1.);
			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac1.GetTransposed()*localJac*jac0, n0, 0, 1.);

		}

		////add jacobian derivative:
		if (true)
		{
			if (n0 != 0 && markerData.GetMarkerData(0).jacobianDerivative.NumberOfRows() != 0)
			{
				jacobianODE2.GetInternalDenseMatrix().AddSubmatrixWithFactor(markerData.GetMarkerData(0).jacobianDerivative, -factorODE2, 0, 0); //force on marker0 acts with negative sign!
			}
			if (n1 != 0 && markerData.GetMarkerData(1).jacobianDerivative.NumberOfRows() != 0)
			{
				jacobianODE2.GetInternalDenseMatrix().AddSubmatrixWithFactor(markerData.GetMarkerData(1).jacobianDerivative, factorODE2, n0, n0);
			}
		}
	}
	else
	{
		jacobianODE2.GetInternalDenseMatrix().SetAll(0.);
	}
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//the L2 chains of the connector interface (#2745): the kinematics of the markers (L0), the force or the equations of the
//connector (L1), and their projection by the markers; a connector on the interface calls the chain of its kind of markers
//from ComputeODE2LHSConnector, ComputeJacobianODE2Connector and, for constraints, the three functions of CObjectConstraint

//! L2 of the connector interface for connectors on position markers (#2745): the kinematics of the two markers (L0),
//! the connector's force (L1), and its projection by each marker, into the local vector [marker 0, marker 1]
void ConnectorODE2LHSPositionMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConnector& connector, Vector& localODE2Lhs, Index objectNumber)
{
	const CMarker* marker0 = cSystemData.GetCMarkers()[connector.GetMarkerNumbers()[0]];
	const CMarker* marker1 = cSystemData.GetCMarkers()[connector.GetMarkerNumbers()[1]];
	MarkerPosition<Real> kinematics[2];
	marker0->GetKinematicsPosition(cSystemData, kinematics[0]);
	marker1->GetKinematicsPosition(cSystemData, kinematics[1]);

	Index n0 = marker0->GetODE2Size(cSystemData, temp.markerTemp[0]);
	Index n1 = marker1->GetODE2Size(cSystemData, temp.markerTemp[1]);
	localODE2Lhs.SetNumberOfItems(n0 + n1);
	localODE2Lhs.SetAll(0.);

	Vector3D force;
	connector.ComputeConnectorForcePosition(kinematics, cSystemData.GetCData().currentState.time, objectNumber, force);
	if (n1 != 0)
	{
		LinkedDataVector ode2Lhs1(localODE2Lhs, n0, n1);
		marker1->AddGeneralizedForce(cSystemData, force, temp.markerTemp[1], ode2Lhs1);
	}
	if (n0 != 0)
	{
		LinkedDataVector ode2Lhs0(localODE2Lhs, 0, n0);
		marker0->AddGeneralizedForce(cSystemData, -force, temp.markerTemp[0], ode2Lhs0);
	}
}

//! the chain of the Jacobian of the connector interface (#2745): the force on marker 1 carries in direction dim*k+c its
//! derivative by component c of the kinematics of marker k, already scaled by factorODE2 and factorODE2_t; block (i, k)
//! of the connector's Jacobian is J_i^T s_i K_k J_k, s_0 = -1 (marker 0 gets the reaction), s_1 = 1; then the derivative of
//! J_i^T f for markers whose Jacobian depends on the coordinates; dense, into temp.jacobianODE2Container
template<Index dim, class TForce>
static void ChainConnectorJacobian(const CSystemData& cSystemData, const ArrayIndex& markerNumbers, const TForce& force,
	const ResizableMatrix& markerJacobian0, const ResizableMatrix& markerJacobian1, Real factorODE2, bool jacobianDerivativeNonZero,
	TemporaryComputationData& temp)
{
	const ResizableMatrix* markerJacobian[2] = { &markerJacobian0, &markerJacobian1 };
	const Index n[2] = { markerJacobian0.NumberOfColumns(), markerJacobian1.NumberOfColumns() };
	const Index offset[2] = { 0, n[0] };
	const Real sign[2] = { -1., 1. };

	temp.jacobianODE2Container.SetUseDenseMatrix();
	ResizableMatrix& jacobian = temp.jacobianODE2Container.GetInternalDenseMatrix();
	jacobian.SetNumberOfRowsAndColumns(n[0] + n[1], n[0] + n[1]);

	ConstSizeMatrix<dim * dim> innerJacobian(dim, dim);
	for (Index k = 0; k < 2; k++) //columns: the marker the force is differentiated for
	{
		if (n[k] == 0) { continue; }
		for (Index i = 0; i < 2; i++) //rows: the marker the force acts on
		{
			if (n[i] == 0) { continue; }
			for (Index r = 0; r < dim; r++)
			{
				for (Index c = 0; c < dim; c++)
				{
					innerJacobian(r, c) = sign[i] * force[r].DValue((int)(dim * k + c));
				}
			}
			EXUmath::MultMatrixTransposedMatrixTemplate(*markerJacobian[i], innerJacobian, temp.jacobianTemp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.jacobianTemp.matrix0, *markerJacobian[k], jacobian, offset[i], offset[k]);
		}
	}

	if (jacobianDerivativeNonZero)
	{
		Vector6D force6D(0.);
		for (Index r = 0; r < dim; r++) { force6D[r] = force[r].Value(); }
		for (Index k = 0; k < 2; k++)
		{
			if (n[k] == 0) { continue; }
			cSystemData.GetCMarkers()[markerNumbers[k]]->AddJacobianDerivative(cSystemData, force6D, sign[k] * factorODE2, offset[k], jacobian);
		}
	}
}

//! the Jacobian of L2 for connectors on position markers (#2745): the connector's force once with automatic
//! differentiation, the directions [marker 0, marker 1] x 3 seeded with factorODE2 at the positions and factorODE2_t at the
//! velocities, which gives K_k = factorODE2*dF/dp_k + factorODE2_t*dF/dv_k; chained with the position Jacobians as
//! J_i^T s_i K_k J_k (s_0 = -1, s_1 = 1), plus the derivative of J_i^T f for markers whose Jacobian depends on the
//! coordinates; dense, into temp.jacobianODE2Container; dv/dq is neglected
void ConnectorJacobianODE2PositionMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConnector& connector,
	Real factorODE2, Real factorODE2_t, Index objectNumber, bool jacobianDerivativeNonZero)
{
	const ArrayIndex& markerNumbers = connector.GetMarkerNumbers();
	MarkerPosition<DRealPositionMarkers> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		MarkerPosition<Real> markerKinematics;
		cSystemData.GetCMarkers()[markerNumbers[k]]->GetKinematicsJacobianPosition(cSystemData, markerKinematics, temp.markerTemp[k]);
		EXUmath::SeedAutoDiff(kinematics[k].position, markerKinematics.position, 3 * (int)k, factorODE2);
		EXUmath::SeedAutoDiff(kinematics[k].velocity, markerKinematics.velocity, 3 * (int)k, factorODE2_t);
	}

	SlimVectorBase<DRealPositionMarkers, 3> force;
	connector.ComputeConnectorForcePositionDiff(kinematics, cSystemData.GetCData().currentState.time, objectNumber, force);

	ChainConnectorJacobian<3>(cSystemData, markerNumbers, force, temp.markerTemp[0].positionJacobian,
		temp.markerTemp[1].positionJacobian, factorODE2, jacobianDerivativeNonZero, temp);
}

//! L0 for the chain of rigid markers: a marker without orientation - the contact connectors take one where they need no
//! torque on it - gives its position and velocity with the unit matrix as rotation and no angular velocity (#2745)
static Index GetKinematicsRigidOrPosition(const CSystemData& cSystemData, const CMarker& marker, MarkerRigid<Real>& kinematics, MarkerTemp& markerTemp)
{
	if (marker.GetType() & Marker::Orientation) { return marker.GetKinematicsRigid(cSystemData, kinematics, markerTemp); }
	MarkerPosition<Real> position;
	marker.GetKinematicsPosition(cSystemData, position);
	kinematics.frame = HomogeneousTransformation(EXUmath::unitMatrix3D, position.position);
	kinematics.velocity = position.velocity;
	kinematics.angularVelocityLocal.SetAll(0.);
	return marker.GetODE2Size(cSystemData, markerTemp);
}

//! L2 of the connector interface for connectors on rigid markers (#2745): the frames and velocities of the two markers
//! (L0), the connector's forces and torques (L1), and their projection by each marker, into [marker 0, marker 1]; a
//! marker without orientation takes the force only
void ConnectorODE2LHSRigidMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConnector& connector, Vector& localODE2Lhs, Index objectNumber)
{
	const CMarker* markers[2] = { cSystemData.GetCMarkers()[connector.GetMarkerNumbers()[0]], cSystemData.GetCMarkers()[connector.GetMarkerNumbers()[1]] };
	MarkerRigid<Real> kinematics[2];
	Index n[2];
	for (Index k = 0; k < 2; k++) { n[k] = GetKinematicsRigidOrPosition(cSystemData, *markers[k], kinematics[k], temp.markerTemp[k]); }
	localODE2Lhs.SetNumberOfItems(n[0] + n[1]);
	localODE2Lhs.SetAll(0.);

	Vector3D forces[2], torques[2];
	connector.ComputeConnectorForceRigid(kinematics, cSystemData.GetCData().currentState.time, objectNumber, forces, torques);
	for (Index k = 0; k < 2; k++)
	{
		if (n[k] == 0) { continue; }
		LinkedDataVector ode2Lhs(localODE2Lhs, k == 0 ? 0 : n[0], n[k]);
		if (markers[k]->GetType() & Marker::Orientation)
		{
			markers[k]->AddGeneralizedForceTorque(cSystemData, forces[k], torques[k], temp.markerTemp[k], ode2Lhs);
		}
		else { markers[k]->AddGeneralizedForce(cSystemData, forces[k], temp.markerTemp[k], ode2Lhs); }
	}
}

//! the Jacobian of L2 for connectors on rigid markers (#2745): the connector's forces and torques once with automatic
//! differentiation in 12 directions - per marker 3 translations and 3 rotation increments, global, A(dtheta) = (I + skew(dtheta)) A,
//! so that they chain with the marker's rotation Jacobian (omega = J_rot q_t, global); the position and rotation seeded with
//! factorODE2, the velocity and the global angular velocity with factorODE2_t in the same directions, and the local
//! angular velocity formed from them as A(dtheta)^T (A omega_local + domega). The connector gives the force and torque per
//! marker, so the inner Jacobian is four 6x6 blocks K_ik = d(f_i, tau_i)/d(p_k, theta_k), chained as
//! [J_pos,i; J_rot,i]^T K_ik [J_pos,k; J_rot,k]; then the derivative of J_i^T (f_i, tau_i) for markers whose Jacobian depends
//! on the coordinates, each with its own force and torque; dv/dq and domega/dq are neglected, as for position markers
void ConnectorJacobianODE2RigidMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConnector& connector,
	Real factorODE2, Real factorODE2_t, Index objectNumber, bool jacobianDerivativeNonZero)
{
	const ArrayIndex& markerNumbers = connector.GetMarkerNumbers();
	MarkerRigid<DRealRigidMarkers> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		MarkerRigid<Real> markerKinematics;
		cSystemData.GetCMarkers()[markerNumbers[k]]->GetKinematicsJacobianRigid(cSystemData, markerKinematics, temp.markerTemp[k]);
		const int offset = 6 * (int)k;
		SlimVectorBase<DRealRigidMarkers, 3> position, velocity, angularVelocity;
		EXUmath::SeedAutoDiff(position, markerKinematics.frame.GetTranslation(), offset, factorODE2);
		EXUmath::SeedAutoDiff(velocity, markerKinematics.velocity, offset, factorODE2_t);

		const Matrix3D A = markerKinematics.frame.GetRotation();
		ConstSizeMatrixBase<DRealRigidMarkers, 9> rotation(3, 3);
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				rotation(i, j) = A(i, j); //all derivatives zero
			}
		}
		for (Index m = 0; m < 3; m++) //d/dtheta_m (I + skew(theta)) A = skew(e_m) A: column j is e_m x A_j
		{
			for (Index j = 0; j < 3; j++)
			{
				const Index m1 = (m + 1) % 3, m2 = (m + 2) % 3;
				rotation(m1, j).DValue(offset + 3 + (int)m) = -factorODE2 * A(m2, j);
				rotation(m2, j).DValue(offset + 3 + (int)m) = factorODE2 * A(m1, j);
			}
		}
		Vector3D omega = A * markerKinematics.angularVelocityLocal; //global
		EXUmath::SeedAutoDiff(angularVelocity, omega, offset + 3, factorODE2_t);

		kinematics[k].frame = HomogeneousTransformationBase<DRealRigidMarkers>(rotation, position);
		kinematics[k].velocity = velocity;
		kinematics[k].angularVelocityLocal = rotation.GetTransposed() * angularVelocity;
	}

	SlimVectorBase<DRealRigidMarkers, 3> forces[2], torques[2];
	connector.ComputeConnectorForceRigidDiff(kinematics, cSystemData.GetCData().currentState.time, objectNumber, forces, torques);

	//the chain, into temp.jacobianODE2Container
	const Index n[2] = { temp.markerTemp[0].positionJacobian.NumberOfColumns(), temp.markerTemp[1].positionJacobian.NumberOfColumns() };
	const Index offsetJacobian[2] = { 0, n[0] };
	temp.jacobianODE2Container.SetUseDenseMatrix();
	ResizableMatrix& jacobian = temp.jacobianODE2Container.GetInternalDenseMatrix();
	jacobian.SetNumberOfRowsAndColumns(n[0] + n[1], n[0] + n[1]);
	jacobian.SetAll(0.);
	for (Index k = 0; k < 2; k++) //the stacked Jacobian [J_pos; J_rot], 6 x n_k
	{
		if (n[k] == 0) { continue; }
		ResizableMatrix& stacked = temp.markerTemp[k].tempMatrix;
		stacked.SetNumberOfRowsAndColumns(6, n[k]);
		stacked.SetSubmatrix(temp.markerTemp[k].positionJacobian, 0, 0);
		stacked.SetSubmatrix(temp.markerTemp[k].rotationJacobian, 3, 0);
	}
	ConstSizeMatrix<36> innerJacobian(6, 6);
	for (Index k = 0; k < 2; k++) //columns: the marker the forces are differentiated for
	{
		if (n[k] == 0) { continue; }
		for (Index i = 0; i < 2; i++) //rows: the marker the force and torque act on
		{
			if (n[i] == 0) { continue; }
			for (Index c = 0; c < 6; c++)
			{
				for (Index r = 0; r < 3; r++)
				{
					innerJacobian(r, c) = forces[i][r].DValue((int)(6 * k + c));
					innerJacobian(r + 3, c) = torques[i][r].DValue((int)(6 * k + c));
				}
			}
			EXUmath::MultMatrixTransposedMatrixTemplate(temp.markerTemp[i].tempMatrix, innerJacobian, temp.jacobianTemp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.jacobianTemp.matrix0, temp.markerTemp[k].tempMatrix, jacobian, offsetJacobian[i], offsetJacobian[k]);
		}
	}

	if (jacobianDerivativeNonZero)
	{
		for (Index k = 0; k < 2; k++)
		{
			if (n[k] == 0) { continue; }
			Vector6D forceTorque;
			for (Index r = 0; r < 3; r++)
			{
				forceTorque[r] = forces[k][r].Value();
				forceTorque[r + 3] = torques[k][r].Value();
			}
			cSystemData.GetCMarkers()[markerNumbers[k]]->AddJacobianDerivative(cSystemData, forceTorque, factorODE2, offsetJacobian[k], jacobian);
		}
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//constraints on the connector interface (#2745)

//! the equations of a constraint on rigid markers (#2745)
void ConstraintEquationsRigidMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp,
	const CObjectConstraint& constraint, Index objectNumber, bool velocityLevel, Vector& localAE)
{
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	MarkerRigid<Real> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		cSystemData.GetCMarkers()[markerNumbers[k]]->GetKinematicsRigid(cSystemData, kinematics[k], temp.markerTemp[k]);
	}
	LinkedDataVector lambda(cSystemData.GetCData().currentState.AECoords, constraint.GetGlobalAECoordinateIndex(), constraint.GetAlgebraicEquationsSize());
	ConstSizeVector<maxConstraintEquations> equations;
	constraint.ComputeConstraintEquationsRigid(kinematics, lambda, cSystemData.GetCData().currentState.time, objectNumber, velocityLevel, equations);
	localAE.CopyFrom(equations);
}

//! the equations of a constraint on rigid markers with automatic differentiation in the 12 directions of RG14.2.8.1 -
//! per marker 3 translations and 3 global rotation increments, A(dtheta) = (I + skew(dtheta)) A; with Jacobians, the
//! marker data is in temp.markerTemp, else the frames come from GetKinematicsRigid
static void SeedConstraintEquationsRigid(const CSystemData& cSystemData, TemporaryComputationData& temp,
	const CObjectConstraint& constraint, Index objectNumber, bool withJacobians, ConstSizeVectorBase<DRealRigidMarkers, maxConstraintEquations>& equations,
	Index* numberOfCoordinates = nullptr)
{
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	MarkerRigid<DRealRigidMarkers> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		const CMarker* marker = cSystemData.GetCMarkers()[markerNumbers[k]];
		Vector3D position;
		Matrix3D A;
		if (withJacobians)
		{
			MarkerRigid<Real> markerKinematics;
			marker->GetKinematicsJacobianRigid(cSystemData, markerKinematics, temp.markerTemp[k]);
			position = markerKinematics.frame.GetTranslation();
			A = markerKinematics.frame.GetRotation();
		}
		else
		{
			MarkerRigid<Real> frame;
			Index n = marker->GetKinematicsRigid(cSystemData, frame, temp.markerTemp[k]);
			if (numberOfCoordinates) { numberOfCoordinates[k] = n; }
			position = frame.frame.GetTranslation();
			A = frame.frame.GetRotation();
		}
		const int offset = 6 * (int)k;
		SlimVectorBase<DRealRigidMarkers, 3> positionAD;
		EXUmath::SeedAutoDiff(positionAD, position, offset);
		ConstSizeMatrixBase<DRealRigidMarkers, 9> rotation(3, 3);
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++) { rotation(i, j) = A(i, j); }
		}
		for (Index m = 0; m < 3; m++) //d/dtheta_m (I + skew(theta)) A = skew(e_m) A: column j is e_m x A_j
		{
			const Index m1 = (m + 1) % 3, m2 = (m + 2) % 3;
			for (Index j = 0; j < 3; j++)
			{
				rotation(m1, j).DValue(offset + 3 + (int)m) = -A(m2, j);
				rotation(m2, j).DValue(offset + 3 + (int)m) = A(m1, j);
			}
		}
		kinematics[k].frame = HomogeneousTransformationBase<DRealRigidMarkers>(rotation, positionAD);
		kinematics[k].velocity.SetAll(0.); //position level: the equations do not read them
		kinematics[k].angularVelocityLocal.SetAll(0.);
	}
	LinkedDataVector lambda(cSystemData.GetCData().currentState.AECoords, constraint.GetGlobalAECoordinateIndex(), constraint.GetAlgebraicEquationsSize());
	constraint.ComputeConstraintEquationsRigidDiff(kinematics, lambda, cSystemData.GetCData().currentState.time, objectNumber, equations);
}

//! the equations of a constraint on coordinate markers (#2745)
void ConstraintEquationsCoordinateMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp,
	const CObjectConstraint& constraint, Index objectNumber, bool velocityLevel, Vector& localAE)
{
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	MarkerCoordinate<Real> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		cSystemData.GetCMarkers()[markerNumbers[k]]->GetKinematicsCoordinate(cSystemData, kinematics[k], temp.markerTemp[k]);
	}
	LinkedDataVector lambda(cSystemData.GetCData().currentState.AECoords, constraint.GetGlobalAECoordinateIndex(), constraint.GetAlgebraicEquationsSize());
	ConstSizeVector<maxConstraintEquations> equations;
	constraint.ComputeConstraintEquationsCoordinate(kinematics, lambda, cSystemData.GetCData().currentState.time, objectNumber, velocityLevel, equations);
	localAE.CopyFrom(equations);
}

//! the equations of a constraint on coordinate markers with automatic differentiation by the values of the markers, one
//! direction per marker; the marker data with the Jacobians is in temp.markerTemp
static void SeedConstraintEquationsCoordinate(const CSystemData& cSystemData, TemporaryComputationData& temp,
	const CObjectConstraint& constraint, Index objectNumber, ConstSizeVectorBase<DRealCoordinateMarkers, maxConstraintEquations>& equations)
{
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	MarkerCoordinate<DRealCoordinateMarkers> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		MarkerCoordinate<Real> markerKinematics;
		cSystemData.GetCMarkers()[markerNumbers[k]]->GetKinematicsJacobianCoordinate(cSystemData, markerKinematics, temp.markerTemp[k]);
		kinematics[k].value = markerKinematics.value;
		kinematics[k].value.DValue((int)k) = 1.;
		kinematics[k].value_t = 0.; //position level: the equations do not read it
	}
	LinkedDataVector lambda(cSystemData.GetCData().currentState.AECoords, constraint.GetGlobalAECoordinateIndex(), constraint.GetAlgebraicEquationsSize());
	constraint.ComputeConstraintEquationsCoordinateDiff(kinematics, lambda, cSystemData.GetCData().currentState.time, objectNumber, equations);
}

//! the equations of a constraint on position markers (#2745): the kinematics of its markers (L0) and its Lagrange
//! multipliers, the equations by the constraint (L1)
void ConstraintEquationsPositionMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConstraint& constraint,
	Index objectNumber, bool velocityLevel, Vector& localAE)
{
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	MarkerPosition<Real> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		cSystemData.GetCMarkers()[markerNumbers[k]]->GetKinematicsPosition(cSystemData, kinematics[k]);
	}
	LinkedDataVector lambda(cSystemData.GetCData().currentState.AECoords, constraint.GetGlobalAECoordinateIndex(), constraint.GetAlgebraicEquationsSize());
	ConstSizeVector<maxConstraintEquations> equations;
	constraint.ComputeConstraintEquationsPosition(kinematics, lambda, cSystemData.GetCData().currentState.time, objectNumber, velocityLevel, equations);
	localAE.CopyFrom(equations);
}

//! the derivatives dg/dp_k of the equations of a constraint on position markers, into equations, seeded as for position
//! connectors with the 3 directions per marker (#2745); with Jacobians, the marker data is in temp.markerTemp
static void SeedConstraintEquationsPosition(const CSystemData& cSystemData, TemporaryComputationData& temp,
	const CObjectConstraint& constraint, Index objectNumber, bool withJacobians, ConstSizeVectorBase<DRealPositionMarkers, maxConstraintEquations>& equations)
{
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	MarkerPosition<DRealPositionMarkers> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		const CMarker* marker = cSystemData.GetCMarkers()[markerNumbers[k]];
		Vector3D position;
		if (withJacobians)
		{
			MarkerPosition<Real> markerKinematics;
			marker->GetKinematicsJacobianPosition(cSystemData, markerKinematics, temp.markerTemp[k]);
			position = markerKinematics.position;
		}
		else
		{
			marker->GetPosition(cSystemData, position);
		}
		EXUmath::SeedAutoDiff(kinematics[k].position, position, 3 * (int)k);
		kinematics[k].velocity.SetAll(0.); //position level: the equations do not read it
	}
	LinkedDataVector lambda(cSystemData.GetCData().currentState.AECoords, constraint.GetGlobalAECoordinateIndex(), constraint.GetAlgebraicEquationsSize());
	constraint.ComputeConstraintEquationsPositionDiff(kinematics, lambda, cSystemData.GetCData().currentState.time, objectNumber, equations);
}

//! C_q = [dg/dp_0 J_pos,0, dg/dp_1 J_pos,1] of a constraint on position markers, the derivatives by automatic
//! differentiation of the constraint's own equations (#2745) - no hand-written Jacobian
void ConstraintJacobianPositionMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConstraint& constraint, Index objectNumber,
	ResizableMatrix& jacobian)
{
	ConstSizeVectorBase<DRealPositionMarkers, maxConstraintEquations> equations;
	SeedConstraintEquationsPosition(cSystemData, temp, constraint, objectNumber, true, equations);

	const ResizableMatrix* markerJacobian[2] = { &temp.markerTemp[0].positionJacobian, &temp.markerTemp[1].positionJacobian };
	const Index n0 = markerJacobian[0]->NumberOfColumns();
	const Index nEquations = equations.NumberOfItems();
	jacobian.SetNumberOfRowsAndColumns(nEquations, n0 + markerJacobian[1]->NumberOfColumns());
	for (Index r = 0; r < nEquations; r++)
	{
		for (Index k = 0; k < 2; k++)
		{
			const Index offset = k == 0 ? 0 : n0;
			for (Index j = 0; j < markerJacobian[k]->NumberOfColumns(); j++)
			{
				Real value = 0.;
				for (Index c = 0; c < 3; c++) { value += equations[r].DValue((int)(3 * k + c)) * (*markerJacobian[k])(c, j); }
				jacobian(r, offset + j) = value;
			}
		}
	}
}

//! C_q = [dg/d(p,theta)_k [J_pos,k; J_rot,k]] of a constraint on rigid markers, the derivatives by automatic
//! differentiation of the constraint's own equations (#2745)
void ConstraintJacobianRigidMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConstraint& constraint, Index objectNumber,
	ResizableMatrix& jacobian)
{
	ConstSizeVectorBase<DRealRigidMarkers, maxConstraintEquations> equations;
	SeedConstraintEquationsRigid(cSystemData, temp, constraint, objectNumber, true, equations);
	const Index n0 = temp.markerTemp[0].positionJacobian.NumberOfColumns();
	const Index n1 = temp.markerTemp[1].positionJacobian.NumberOfColumns();
	jacobian.SetNumberOfRowsAndColumns(equations.NumberOfItems(), n0 + n1);
	for (Index r = 0; r < equations.NumberOfItems(); r++)
	{
		for (Index k = 0; k < 2; k++)
		{
			const MarkerTemp& markerData = temp.markerTemp[k];
			for (Index j = 0; j < (k == 0 ? n0 : n1); j++)
			{
				Real value = 0.;
				for (Index c = 0; c < 3; c++)
				{
					value += equations[r].DValue((int)(6 * k + c)) * markerData.positionJacobian(c, j)
						+ equations[r].DValue((int)(6 * k + 3 + c)) * markerData.rotationJacobian(c, j);
				}
				jacobian(r, (k == 0 ? 0 : n0) + j) = value;
			}
		}
	}
}

//! C_q = [dg/dv_0 J_0, dg/dv_1 J_1] of a constraint on coordinate markers, J_k the 1 x n_k marker Jacobian, by automatic
//! differentiation of the constraint's own equations (#2745)
void ConstraintJacobianCoordinateMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConstraint& constraint, Index objectNumber,
	ResizableMatrix& jacobian)
{
	ConstSizeVectorBase<DRealCoordinateMarkers, maxConstraintEquations> equations;
	SeedConstraintEquationsCoordinate(cSystemData, temp, constraint, objectNumber, equations);
	const ResizableMatrix& jacobian0 = temp.markerTemp[0].coordinateJacobian;
	const ResizableMatrix& jacobian1 = temp.markerTemp[1].coordinateJacobian;
	jacobian.SetNumberOfRowsAndColumns(equations.NumberOfItems(), jacobian0.NumberOfColumns() + jacobian1.NumberOfColumns());
	for (Index r = 0; r < equations.NumberOfItems(); r++)
	{
		for (Index j = 0; j < jacobian0.NumberOfColumns(); j++) { jacobian(r, j) = equations[r].DValue(0) * jacobian0(0, j); }
		for (Index j = 0; j < jacobian1.NumberOfColumns(); j++) { jacobian(r, jacobian0.NumberOfColumns() + j) = equations[r].DValue(1) * jacobian1(0, j); }
	}
}

//! C_q^T lambda of a constraint on position markers (#2745): per marker the force f_k = (dg/dp_k)^T lambda, projected by
//! the marker as for a connector force - C_q itself is not formed
void ConstraintReactionForcesPositionMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConstraint& constraint, Index objectNumber,
	const Vector& reactionForces, Vector& localODE2)
{
	ConstSizeVectorBase<DRealPositionMarkers, maxConstraintEquations> equations;
	SeedConstraintEquationsPosition(cSystemData, temp, constraint, objectNumber, false, equations);

	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	const ArrayIndex& ltgAE = cSystemData.GetLocalToGlobalAE()[objectNumber];
	const CMarker* marker[2] = { cSystemData.GetCMarkers()[markerNumbers[0]], cSystemData.GetCMarkers()[markerNumbers[1]] };
	const Index n[2] = { marker[0]->GetODE2Size(cSystemData, temp.markerTemp[0]), marker[1]->GetODE2Size(cSystemData, temp.markerTemp[1]) };
	localODE2.SetNumberOfItems(n[0] + n[1]);
	localODE2.SetAll(0.);
	for (Index k = 0; k < 2; k++)
	{
		if (n[k] == 0) { continue; }
		Vector3D force(0.);
		for (Index r = 0; r < equations.NumberOfItems(); r++)
		{
			for (Index c = 0; c < 3; c++) { force[c] += reactionForces[ltgAE[r]] * equations[r].DValue((int)(3 * k + c)); }
		}
		LinkedDataVector localODE2k(localODE2, k == 0 ? 0 : n[0], n[k]);
		marker[k]->AddGeneralizedForce(cSystemData, force, temp.markerTemp[k], localODE2k);
	}
}

//! C_q^T lambda of a constraint on rigid markers (#2745): per marker the force and torque (dg/d(p,theta)_k)^T lambda,
//! projected by the marker - C_q itself is not formed
void ConstraintReactionForcesRigidMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConstraint& constraint, Index objectNumber,
	const Vector& reactionForces, Vector& localODE2)
{
	ConstSizeVectorBase<DRealRigidMarkers, maxConstraintEquations> equations;
	Index n[2];
	SeedConstraintEquationsRigid(cSystemData, temp, constraint, objectNumber, false, equations, n); //GetKinematicsRigid prepares temp for the projection
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	const ArrayIndex& ltgAE = cSystemData.GetLocalToGlobalAE()[objectNumber];
	const CMarker* marker[2] = { cSystemData.GetCMarkers()[markerNumbers[0]], cSystemData.GetCMarkers()[markerNumbers[1]] };
	localODE2.SetNumberOfItems(n[0] + n[1]);
	localODE2.SetAll(0.);
	for (Index k = 0; k < 2; k++)
	{
		if (n[k] == 0) { continue; }
		Vector3D force(0.), torque(0.);
		for (Index r = 0; r < equations.NumberOfItems(); r++)
		{
			for (Index c = 0; c < 3; c++)
			{
				force[c] += reactionForces[ltgAE[r]] * equations[r].DValue((int)(6 * k + c));
				torque[c] += reactionForces[ltgAE[r]] * equations[r].DValue((int)(6 * k + 3 + c));
			}
		}
		LinkedDataVector localODE2k(localODE2, k == 0 ? 0 : n[0], n[k]);
		marker[k]->AddGeneralizedForceTorque(cSystemData, force, torque, temp.markerTemp[k], localODE2k);
	}
}

//! C_q^T lambda of a constraint on coordinate markers (#2745): per marker the generalized force (dg/dv_k)^T lambda -
//! C_q itself is not formed
void ConstraintReactionForcesCoordinateMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConstraint& constraint, Index objectNumber,
	const Vector& reactionForces, Vector& localODE2)
{
	ConstSizeVectorBase<DRealCoordinateMarkers, maxConstraintEquations> equations;
	SeedConstraintEquationsCoordinate(cSystemData, temp, constraint, objectNumber, equations);
	const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
	const ArrayIndex& ltgAE = cSystemData.GetLocalToGlobalAE()[objectNumber];
	MarkerCoordinate<Real> kinematics;
	const CMarker* marker[2] = { cSystemData.GetCMarkers()[markerNumbers[0]], cSystemData.GetCMarkers()[markerNumbers[1]] };
	const Index n[2] = { marker[0]->GetKinematicsCoordinate(cSystemData, kinematics, temp.markerTemp[0]),
		marker[1]->GetKinematicsCoordinate(cSystemData, kinematics, temp.markerTemp[1]) };
	localODE2.SetNumberOfItems(n[0] + n[1]);
	localODE2.SetAll(0.);
	for (Index k = 0; k < 2; k++)
	{
		if (n[k] == 0) { continue; }
		Real force = 0.;
		for (Index r = 0; r < equations.NumberOfItems(); r++) { force += reactionForces[ltgAE[r]] * equations[r].DValue((int)k); }
		LinkedDataVector localODE2k(localODE2, k == 0 ? 0 : n[0], n[k]);
		marker[k]->AddGeneralizedForceCoordinate(cSystemData, force, temp.markerTemp[k], localODE2k);
	}
}

//! L2 of the connector interface for connectors on coordinate markers (#2745): the values of the two markers (L0), the
//! connector's generalized force (L1), and its projection by each marker, into the local vector [marker 0, marker 1]
void ConnectorODE2LHSCoordinateMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConnector& connector, Vector& localODE2Lhs, Index objectNumber)
{
	const CMarker* marker0 = cSystemData.GetCMarkers()[connector.GetMarkerNumbers()[0]];
	const CMarker* marker1 = cSystemData.GetCMarkers()[connector.GetMarkerNumbers()[1]];
	MarkerCoordinate<Real> kinematics[2];
	Index n0 = marker0->GetKinematicsCoordinate(cSystemData, kinematics[0], temp.markerTemp[0]);
	Index n1 = marker1->GetKinematicsCoordinate(cSystemData, kinematics[1], temp.markerTemp[1]);
	localODE2Lhs.SetNumberOfItems(n0 + n1);
	localODE2Lhs.SetAll(0.);

	Real forces[2];
	connector.ComputeConnectorForcesCoordinate(kinematics, cSystemData.GetCData().currentState.time, objectNumber, forces);
	if (n1 != 0)
	{
		LinkedDataVector ode2Lhs1(localODE2Lhs, n0, n1);
		marker1->AddGeneralizedForceCoordinate(cSystemData, forces[1], temp.markerTemp[1], ode2Lhs1);
	}
	if (n0 != 0)
	{
		LinkedDataVector ode2Lhs0(localODE2Lhs, 0, n0);
		marker0->AddGeneralizedForceCoordinate(cSystemData, forces[0], temp.markerTemp[0], ode2Lhs0);
	}
}

//! the Jacobian of L2 for connectors on coordinate markers (#2745): as for position markers, with one direction per
//! marker, the value seeded with factorODE2 and its time derivative with factorODE2_t
void ConnectorJacobianODE2CoordinateMarkers(const CSystemData& cSystemData, TemporaryComputationData& temp, const CObjectConnector& connector,
	Real factorODE2, Real factorODE2_t, Index objectNumber, bool jacobianDerivativeNonZero)
{
	const ArrayIndex& markerNumbers = connector.GetMarkerNumbers();
	MarkerCoordinate<DRealCoordinateMarkers> kinematics[2];
	for (Index k = 0; k < 2; k++)
	{
		MarkerCoordinate<Real> markerKinematics;
		cSystemData.GetCMarkers()[markerNumbers[k]]->GetKinematicsJacobianCoordinate(cSystemData, markerKinematics, temp.markerTemp[k]);
		kinematics[k].value = markerKinematics.value;
		kinematics[k].value.DValue((int)k) = factorODE2;
		kinematics[k].value_t = markerKinematics.value_t;
		kinematics[k].value_t.DValue((int)k) = factorODE2_t;
	}

	SlimVectorBase<DRealCoordinateMarkers, 1> force;
	connector.ComputeConnectorForceCoordinateDiff(kinematics, cSystemData.GetCData().currentState.time, objectNumber, force[0]);

	ChainConnectorJacobian<1>(cSystemData, markerNumbers, force, temp.markerTemp[0].coordinateJacobian,
		temp.markerTemp[1].coordinateJacobian, factorODE2, jacobianDerivativeNonZero, temp);
}
