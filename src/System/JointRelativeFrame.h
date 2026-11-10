/** ***********************************************************************************************
* @brief        The constraint equations of the 3D joints on the relative frame of their two joint frames
* @details      Details:
*               ObjectJointGeneric, ObjectJointRevoluteZ and ObjectJointPrismaticX constrain components of the
*               relative frame of J1 = (A1, p1) in J0 = (A0, p0) (#2884): the translations p1-p0, and the rotations
*               by scalar products of the axes of J0 and J1, which are entries of A0^T A1. The derivatives of the
*               equations by the variations of the two frames, [dp0, dtheta0, dp1, dtheta1] with the global rotation
*               increments dA = skew(dtheta) A, are analytic; the time derivative of the equations is the same row
*               times the twists of the frames, [v0, omega0, v1, omega1] with the global angular velocities.
*
* @author       Gerstmayr Johannes
* @date         2026-11-10 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */
#ifndef JOINTRELATIVEFRAME__H
#define JOINTRELATIVEFRAME__H

#include "Main/MarkerData.h" //MarkerRigid, RigidMarkersDerivatives

//! the free rotation axis of J0 and the two locked axes of J1 of a joint with two constrained rotations (revolute)
inline void JointFreeRotationAxis(const Index* constrainedAxes, Index& freeAxis, Index& lockedAxis1, Index& lockedAxis2)
{
	if (constrainedAxes[3] == 0) { freeAxis = 0; lockedAxis1 = 1; lockedAxis2 = 2; }
	else if (constrainedAxes[4] == 0) { freeAxis = 1; lockedAxis1 = 0; lockedAxis2 = 2; }
	else if (constrainedAxes[5] == 0) { freeAxis = 2; lockedAxis1 = 0; lockedAxis2 = 1; }
	else { CHECKandTHROWstring("JointFreeRotationAxis: two rotations must be constrained"); }
}

//! the locked rotation axis and the free axes, the first of J0 and the second of J1, of a joint with one constrained
//! rotation (universal)
inline void JointLockedRotationAxis(const Index* constrainedAxes, Index& lockedAxis, Index& freeAxis1, Index& freeAxis2)
{
	if (constrainedAxes[3] == 1) { lockedAxis = 0; freeAxis1 = 1; freeAxis2 = 2; }
	else if (constrainedAxes[4] == 1) { lockedAxis = 1; freeAxis1 = 0; freeAxis2 = 2; }
	else if (constrainedAxes[5] == 1) { lockedAxis = 2; freeAxis1 = 0; freeAxis2 = 1; }
	else { CHECKandTHROWstring("JointLockedRotationAxis: one rotation must be constrained"); }
}

//! the equation a*b of the axis a of J0 and the axis b of J1 and its row: d(a*b) = (a x b)*(dtheta0 - dtheta1)
inline Real JointAxesEquation(const Vector3D& a, const Vector3D& b, Index row, RigidMarkersDerivatives* derivatives)
{
	if (derivatives)
	{
		Vector3D w = a.CrossProduct(b);
		for (Index c = 0; c < 3; c++)
		{
			(*derivatives)(row, 3 + c) = w[c];
			(*derivatives)(row, 9 + c) = -w[c];
		}
	}
	return a * b;
}

//! the equation a*(b x c) - 1 of the axes a, c of J0 and b of J1 (the alternative constraints of ObjectJointGeneric) and
//! its row: d(a*(b x c)) = (b x (c x a))*(dtheta1 - dtheta0), as the product does not change if both frames rotate
inline Real JointTripleEquation(const Vector3D& a, const Vector3D& b, const Vector3D& c, Index row, RigidMarkersDerivatives* derivatives)
{
	if (derivatives)
	{
		Vector3D w = b.CrossProduct(c.CrossProduct(a));
		for (Index k = 0; k < 3; k++)
		{
			(*derivatives)(row, 3 + k) = -w[k];
			(*derivatives)(row, 9 + k) = w[k];
		}
	}
	return a * b.CrossProduct(c) - 1.;
}

//! the 6 equations of a joint between J0 = (A0, p0) and J1 = (A1, p1), with relativePosition = p1 - p0 (#2884), set for
//! the axes i with constrainedAxes[i] == 1, else zero:
//! - the translations, global if all three are constrained, else in J0, A0^T (p1 - p0);
//! - three rotations: z0*y1, z0*x1, x0*y1 (alternative: the triple products of ObjectJointGeneric), two rotations
//!   (revolute): the free axis of J0 perpendicular to the two locked axes of J1, one rotation (universal): the first free
//!   axis of J0 perpendicular to the second free axis of J1;
//! and, if derivatives is given, their 6 rows in the 12 variations of the frames, zero for the free axes
inline void JointRelativeFrameEquations(const Index* constrainedAxes, bool alternativeConstraints, const Matrix3D& A0,
	const Matrix3D& A1, const Vector3D& relativePosition, Vector6D& equations, RigidMarkersDerivatives* derivatives = nullptr)
{
	equations.SetAll(0.);
	if (derivatives)
	{
		derivatives->SetNumberOfRowsAndColumns(6, 12);
		derivatives->SetAll(0.);
	}
	if (constrainedAxes[0] == 1 && constrainedAxes[1] == 1 && constrainedAxes[2] == 1)
	{
		for (Index i = 0; i < 3; i++)
		{
			equations[i] = relativePosition[i];
			if (derivatives) { (*derivatives)(i, i) = -1.; (*derivatives)(i, 6 + i) = 1.; }
		}
	}
	else
	{
		for (Index i = 0; i < 3; i++)
		{
			if (constrainedAxes[i] != 1) { continue; }
			Vector3D a = A0.GetColumnVector<3>(i); //d(a*(p1-p0)) = a*(dp1 - dp0) + (a x (p1-p0))*dtheta0
			equations[i] = a * relativePosition;
			if (derivatives)
			{
				Vector3D w = a.CrossProduct(relativePosition);
				for (Index c = 0; c < 3; c++)
				{
					(*derivatives)(i, c) = -a[c];
					(*derivatives)(i, 3 + c) = w[c];
					(*derivatives)(i, 6 + c) = a[c];
				}
			}
		}
	}

	const Index constrainedRotations = constrainedAxes[3] + constrainedAxes[4] + constrainedAxes[5];
	if (constrainedRotations == 3)
	{
		Vector3D x0 = A0.GetColumnVector<3>(0), y0 = A0.GetColumnVector<3>(1), z0 = A0.GetColumnVector<3>(2);
		Vector3D x1 = A1.GetColumnVector<3>(0), y1 = A1.GetColumnVector<3>(1), z1 = A1.GetColumnVector<3>(2);
		if (!alternativeConstraints)
		{
			equations[3] = JointAxesEquation(z0, y1, 3, derivatives);
			equations[4] = JointAxesEquation(z0, x1, 4, derivatives);
			equations[5] = JointAxesEquation(x0, y1, 5, derivatives);
		}
		else
		{
			equations[3] = JointTripleEquation(x0, y1, z0, 3, derivatives);
			equations[4] = JointTripleEquation(y0, z1, x0, 4, derivatives);
			equations[5] = JointTripleEquation(z0, x1, y0, 5, derivatives);
		}
	}
	else if (constrainedRotations == 2)
	{
		Index freeAxis, lockedAxis1, lockedAxis2;
		JointFreeRotationAxis(constrainedAxes, freeAxis, lockedAxis1, lockedAxis2);
		Vector3D a = A0.GetColumnVector<3>(freeAxis);
		equations[lockedAxis1 + 3] = JointAxesEquation(a, A1.GetColumnVector<3>(lockedAxis1), lockedAxis1 + 3, derivatives);
		equations[lockedAxis2 + 3] = JointAxesEquation(a, A1.GetColumnVector<3>(lockedAxis2), lockedAxis2 + 3, derivatives);
	}
	else if (constrainedRotations == 1)
	{
		Index lockedAxis, freeAxis1, freeAxis2;
		JointLockedRotationAxis(constrainedAxes, lockedAxis, freeAxis1, freeAxis2);
		equations[lockedAxis + 3] = JointAxesEquation(A0.GetColumnVector<3>(freeAxis1), A1.GetColumnVector<3>(freeAxis2), lockedAxis + 3, derivatives);
	}
}

//! the time derivatives of the equations of JointRelativeFrameEquations: their rows times the twists [v0, omega0, v1,
//! omega1] of the frames, the angular velocities global
inline void JointRelativeFrameEquations_t(const Index* constrainedAxes, bool alternativeConstraints, const Matrix3D& A0,
	const Matrix3D& A1, const Vector3D& relativePosition, const Vector3D& velocity0, const Vector3D& angularVelocity0,
	const Vector3D& velocity1, const Vector3D& angularVelocity1, Vector6D& equations_t)
{
	RigidMarkersDerivatives derivatives;
	JointRelativeFrameEquations(constrainedAxes, alternativeConstraints, A0, A1, relativePosition, equations_t, &derivatives);
	const Vector3D* twist[4] = { &velocity0, &angularVelocity0, &velocity1, &angularVelocity1 };
	for (Index i = 0; i < 6; i++)
	{
		Real value = 0.;
		for (Index k = 0; k < 4; k++)
		{
			for (Index c = 0; c < 3; c++) { value += derivatives(i, 3 * k + c) * (*twist[k])[c]; }
		}
		equations_t[i] = value;
	}
}

//! the equations of the constrained axes only, in their order, and their rows - for a joint without Lagrange multipliers
//! for its free axes (ObjectJointRevoluteZ, ObjectJointPrismaticX)
inline void JointConstrainedEquations(const Index* constrainedAxes, const Vector6D& values, ConstSizeVector<maxConstraintEquations>& equations,
	RigidMarkersDerivatives* derivatives = nullptr)
{
	Index n = 0;
	for (Index i = 0; i < 6; i++) { n += constrainedAxes[i]; }
	equations.SetNumberOfItems(n);
	n = 0;
	for (Index i = 0; i < 6; i++)
	{
		if (constrainedAxes[i] != 1) { continue; }
		equations[n] = values[i];
		if (derivatives && n != i)
		{
			for (Index j = 0; j < 12; j++) { (*derivatives)(n, j) = (*derivatives)(i, j); }
		}
		n++;
	}
	if (derivatives) { derivatives->SetNumberOfRowsAndColumns(n, 12); } //the rows stay in place, the matrix is stored by rows
}

//! the equations of a joint with fixed constrained axes and without Lagrange multipliers for its free axes, on two rigid
//! markers whose frames are rotated by rotationMarker0/1 into the joint frames (ObjectJointRevoluteZ,
//! ObjectJointPrismaticX): at position level, with their rows if derivatives is given; at velocity level their time
//! derivatives
inline void JointFixedAxesEquations(const Index* constrainedAxes, const Matrix3D& rotationMarker0, const Matrix3D& rotationMarker1,
	const MarkerRigid<Real>& marker0, const MarkerRigid<Real>& marker1, bool velocityLevel, ConstSizeVector<maxConstraintEquations>& equations,
	RigidMarkersDerivatives* derivatives = nullptr)
{
	const Matrix3D& A0 = marker0.frame.GetRotation();
	const Matrix3D& A1 = marker1.frame.GetRotation();
	Matrix3D A0all = ApplyRotationMarker(A0, rotationMarker0);
	Matrix3D A1all = ApplyRotationMarker(A1, rotationMarker1);
	Vector3D relativePosition = marker1.frame.GetTranslation() - marker0.frame.GetTranslation();
	Vector6D values;
	if (!velocityLevel)
	{
		JointRelativeFrameEquations(constrainedAxes, false, A0all, A1all, relativePosition, values, derivatives);
		JointConstrainedEquations(constrainedAxes, values, equations, derivatives);
	}
	else
	{
		JointRelativeFrameEquations_t(constrainedAxes, false, A0all, A1all, relativePosition, marker0.velocity, A0 * marker0.angularVelocityLocal,
			marker1.velocity, A1 * marker1.angularVelocityLocal, values);
		JointConstrainedEquations(constrainedAxes, values, equations);
	}
}

#endif
