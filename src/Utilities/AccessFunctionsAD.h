/** ***********************************************************************************************
* @brief		The access functions of a body by automatic differentiation of its templated position and rotation
* @details		Details:
				- a body that provides its position p(q) and its rotation matrix A(q) as templates of the number type
				  gets the position Jacobian dp/dq, the rotation Jacobian (column j is the axial vector of dA/dq_j A^T)
				  and d(J_pos^T f + J_rot^T tau)/dq (second derivatives, by nested automatic differentiation) from them
				  (#2744, evaluated in revision2026b step RG9.3.5)
				- q are the coordinates of the body (reference + current), n of them; only the coordinates
				  first ... first+nDiff-1 are differentiated - those the position is nonlinear in -, the derivatives by
				  the others are zero here and the caller adds what is linear (the identity of a displacement)
				- the velocity is v = dp/dq q_t, which holds for coordinates whose time derivatives are the velocity
				  coordinates (not for the Lie group nodes)
*
* @author		Gerstmayr Johannes
* @date			2026-10-02
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef ACCESSFUNCTIONSAD__H
#define ACCESSFUNCTIONSAD__H

#include "Utilities/AutomaticDifferentiation.h"
#include "Linalg/ConstSizeMatrix.h"
#include "Linalg/RigidBodyMath.h" //SymmetricBasis, the frame of two slopes (#2228)
#include <type_traits>

namespace AccessFunctionsAD {

	template<int nDiff> using DReal = EXUmath::AutoDiff<nDiff, Real>;
	template<int nDiff> using DDReal = EXUmath::AutoDiff<nDiff, EXUmath::AutoDiff<nDiff, Real>>;
	const Index maxCoordinates = 32; //of a body that uses these functions

	//! the number of differentiated coordinates
	template<int nDiff>
	inline Index Directions(Index n, Index first) { return EXUstd::Minimum((Index)nDiff, n - first); }

	//! the coordinates as AD numbers, direction i-first for coordinate i
	template<int nDiff>
	inline void Seed(const Real* q, Index n, Index first, DReal<nDiff>* qAD)
	{
		CHECKandTHROW(n <= maxCoordinates, "AccessFunctionsAD: too many coordinates");
		for (Index i = 0; i < n; i++) { qAD[i] = DReal<nDiff>(q[i]); }
		for (Index i = 0; i < Directions<nDiff>(n, first); i++) { qAD[first + i].DValue((int)i) = 1.; }
	}

	//! the coordinates as nested AD numbers, direction i-first on both levels for coordinate i
	template<int nDiff>
	inline void Seed(const Real* q, Index n, Index first, DDReal<nDiff>* qAD)
	{
		CHECKandTHROW(n <= maxCoordinates, "AccessFunctionsAD: too many coordinates");
		for (Index i = 0; i < n; i++) { qAD[i] = DDReal<nDiff>(DReal<nDiff>(q[i])); }
		for (Index i = 0; i < Directions<nDiff>(n, first); i++)
		{
			qAD[first + i].Value().DValue((int)i) = 1.;
			qAD[first + i].DValue((int)i) = DReal<nDiff>(1.);
		}
	}

	//! the derivative dg/dq, m x n, of a vector function g(q) with m <= maxCoordinates components, in the columns of the
	//! differentiated coordinates; function(q, g) computes g from the coordinates q, a generic lambda
	template<int nDiff, class TFunction>
	inline void Derivative(const Real* q, Index n, Index first, Index m, const TFunction& function, Matrix& value)
	{
		DReal<nDiff> qAD[maxCoordinates];
		Seed<nDiff>(q, n, first, qAD);
		DReal<nDiff> g[maxCoordinates];
		function(qAD, g);
		value.SetNumberOfRowsAndColumns(m, n);
		value.SetAll(0.);
		for (Index i = 0; i < m; i++)
		{
			for (Index j = 0; j < Directions<nDiff>(n, first); j++) { value(i, first + j) = g[i].DValue((int)j); }
		}
	}

	//! J_pos = dp/dq, 3 x n, the columns of the differentiated coordinates; position(q, p) computes p from the
	//! coordinates q, a generic lambda
	template<int nDiff, class TPosition>
	inline void PositionJacobian(const Real* q, Index n, Index first, const TPosition& position, Matrix& value)
	{
		DReal<nDiff> qAD[maxCoordinates];
		Seed<nDiff>(q, n, first, qAD);
		DReal<nDiff> p[3];
		position(qAD, p);
		value.SetNumberOfRowsAndColumns(3, n);
		value.SetAll(0.);
		for (Index r = 0; r < 3; r++)
		{
			for (Index j = 0; j < Directions<nDiff>(n, first); j++) { value(r, first + j) = p[r].DValue((int)j); }
		}
	}

	//! J_rot, 3 x n: column j is the axial vector of dA/dq_j A^T; rotation(q, A) computes the 3x3 rotation matrix
	template<int nDiff, class TRotation>
	inline void RotationJacobian(const Real* q, Index n, Index first, const TRotation& rotation, Matrix& value)
	{
		DReal<nDiff> qAD[maxCoordinates];
		Seed<nDiff>(q, n, first, qAD);
		ConstSizeMatrixBase<DReal<nDiff>, 9> A(3, 3);
		rotation(qAD, A);
		value.SetNumberOfRowsAndColumns(3, n);
		value.SetAll(0.);
		for (Index j = 0; j < Directions<nDiff>(n, first); j++)
		{
			Real W[3][3];
			for (Index r = 0; r < 3; r++)
			{
				for (Index c = 0; c < 3; c++)
				{
					Real sum = 0.;
					for (Index k = 0; k < 3; k++) { sum += A(r, k).DValue((int)j) * A(c, k).Value(); }
					W[r][c] = sum;
				}
			}
			value(0, first + j) = 0.5 * (W[2][1] - W[1][2]);
			value(1, first + j) = 0.5 * (W[0][2] - W[2][0]);
			value(2, first + j) = 0.5 * (W[1][0] - W[0][1]);
		}
	}

	//! d(J_pos^T force + J_rot^T torque)/dq, n x n, by nested automatic differentiation, in the rows and columns of the
	//! differentiated coordinates (the others are zero: the position is linear in them); withRotation = false for a
	//! body without rotation Jacobian
	template<int nDiff, class TPosition, class TRotation>
	inline void JacobianTransposedTimesVectorDerivative(const Real* q, Index n, Index first, const TPosition& position,
		const TRotation& rotation, bool withRotation, const Vector6D& forceTorque, Matrix& value)
	{
		DDReal<nDiff> qAD[maxCoordinates];
		Seed<nDiff>(q, n, first, qAD);
		DDReal<nDiff> p[3];
		position(qAD, p);
		value.SetNumberOfRowsAndColumns(n, n);
		value.SetAll(0.);
		const Index m = Directions<nDiff>(n, first);
		for (Index j = 0; j < m; j++)
		{
			DReal<nDiff> g(0.); //(J_pos^T f)_j with its derivatives
			for (Index r = 0; r < 3; r++) { g = g + forceTorque[r] * p[r].DValue((int)j); }
			for (Index k = 0; k < m; k++) { value(first + j, first + k) = g.DValue((int)k); }
		}
		if (!withRotation) { return; }

		ConstSizeMatrixBase<DDReal<nDiff>, 9> A(3, 3);
		rotation(qAD, A);
		for (Index j = 0; j < m; j++)
		{
			DReal<nDiff> W[3][3];
			for (Index r = 0; r < 3; r++)
			{
				for (Index c = 0; c < 3; c++)
				{
					DReal<nDiff> sum(0.);
					for (Index k = 0; k < 3; k++) { sum = sum + A(r, k).DValue((int)j) * A(c, k).Value(); }
					W[r][c] = sum;
				}
			}
			DReal<nDiff> h = 0.5 * (forceTorque[3] * (W[2][1] - W[1][2]) + forceTorque[4] * (W[0][2] - W[2][0])
				+ forceTorque[5] * (W[1][0] - W[0][1])); //(J_rot^T tau)_j with its derivatives
			for (Index k = 0; k < m; k++) { value(first + j, first + k) += h.DValue((int)k); }
		}
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//the frame of two slopes s = [s0, s1], the symmetric basis e1, e2, e3 of RigidBodyMath::SymmetricBasis (#2228): the
	//slopes x, y of NodePointSlope12 and ObjectANCFThinPlate give the columns [e1, e2, e3], the slopes y, z of
	//NodePointSlope23 and ObjectANCFBeam the columns [e3, e1, e2] (x = y x z); the angular velocity, the rotation
	//Jacobian and its derivative do not depend on the order of the columns

	//! which slopes s holds
	enum class SlopesFrame { XY, YZ };

	//! the rotation matrix of the slopes s
	template<SlopesFrame frame, class TReal>
	inline void SlopesRotation(const TReal* s, ConstSizeMatrixBase<TReal, 9>& A)
	{
		TReal e1[3], e2[3], e3[3];
		RigidBodyMath::SymmetricBasis(&s[0], &s[3], e1, e2, e3);
		A.SetNumberOfRowsAndColumns(3, 3);
		for (Index r = 0; r < 3; r++)
		{
			if (frame == SlopesFrame::XY) { A(r, 0) = e1[r]; A(r, 1) = e2[r]; A(r, 2) = e3[r]; }
			else { A(r, 0) = e3[r]; A(r, 1) = e1[r]; A(r, 2) = e2[r]; }
		}
	}

	//! the rotation matrix of the slopes s, for Real
	template<SlopesFrame frame>
	inline Matrix3D SlopesRotationMatrix(const Real* s)
	{
		ConstSizeMatrixBase<Real, 9> A;
		SlopesRotation<frame>(s, A);
		Matrix3D rotation;
		rotation.CopyFrom(A);
		return rotation;
	}

	//! d(omega)/d(s_t), 3 x 6, in closed form (RigidBodyMath::SymmetricBasisRotationJacobian)
	inline void SlopesRotationJacobian(const Real* s, Matrix& value)
	{
		Real G[3][6];
		RigidBodyMath::SymmetricBasisRotationJacobian(&s[0], &s[3], G);
		value.SetNumberOfRowsAndColumns(3, 6);
		for (Index r = 0; r < 3; r++)
		{
			for (Index j = 0; j < 6; j++) { value(r, j) = G[r][j]; }
		}
	}

	//! the angular velocity of the slopes s at the rates s_t, G s_t
	inline Vector3D SlopesAngularVelocity(const Real* s, const Real* s_t)
	{
		Real G[3][6];
		RigidBodyMath::SymmetricBasisRotationJacobian(&s[0], &s[3], G);
		Vector3D omega(0.);
		for (Index r = 0; r < 3; r++)
		{
			for (Index j = 0; j < 6; j++) { omega[r] += G[r][j] * s_t[j]; }
		}
		return omega;
	}

	//! d(G^T torque)/ds, 6 x 6, by automatic differentiation of the closed-form G
	inline void SlopesRotationJacobianTTimesTorqueDerivative(const Real* s, const Vector3D& torque, Matrix& value)
	{
		Derivative<6>(s, 6, 0, 6, [&torque](const auto* ss, auto* g)
		{
			using TReal = typename std::remove_const<typename std::remove_reference<decltype(ss[0])>::type>::type;
			TReal G[3][6];
			RigidBodyMath::SymmetricBasisRotationJacobian(&ss[0], &ss[3], G);
			for (Index j = 0; j < 6; j++) { g[j] = torque[0] * G[0][j] + torque[1] * G[1][j] + torque[2] * G[2][j]; }
		}, value);
	}

} //namespace AccessFunctionsAD

#endif
