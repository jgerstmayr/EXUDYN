/** ***********************************************************************************************
* @brief		namespace RigidBodyMath
*				Functions and objects for rigid body kinematics and dynamics calculations;
*				Uses namespace RigidBodyMath;
*				Abbreviations: EP = Euler Parameters, TB = Tait Bryan; EA = Euler Angles (ZXZ)
*
* @author		Gerstmayr Johannes
* @date			2019-10-18 (generated)
* @date			2019-10-18 (last modified)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* 
*
************************************************************************************************ */
#ifndef RIGIDBODYMATH__H
#define RIGIDBODYMATH__H

#include "Linalg/BasicLinalg.h" 

namespace RigidBodyMath {

	//! check whether the rotation is zero, i.e., the rotation matrix is a diagonal matrix with ones
	//! this check is simplified in order to speed up constraint computations, does not check off-diagonal components!
	inline bool IsNoRotation(const Matrix3D& rot)
	{
		if (rot(0, 0) == 1. && rot(1, 1) == 1. && rot(2, 2) == 1.) { return true; } //third check would not be necessary
		return false;
	}

	const Index maxRotCoordinates = 4; //for Euler parameters

	//! compute 3x3 skew(tilde)-matrix from vector v;
	template<class TVector>
	inline ConstSizeMatrix<9> Vector2SkewMatrixTemplate(const TVector& v)
	{
		CHECKandTHROW(v.NumberOfItems() == 3, "Vector2SkewMatrixTemplate: vector must contain 3 items!");
		return ConstSizeMatrix<9>(3, 3, { 0.,  -v[2], v[1],
										  v[2],    0,-v[0],
										 -v[1], v[0],    0 });
	}

	//! compute vector from 3x3 skew(tilde)-matrix
	template<class TMatrix>
	inline Vector3D SkewMatrix2Vector(const TMatrix& m)
	{
		CHECKandTHROW(m.NumberOfRows() == 3 && m.NumberOfColumns() == 3, "SkewMatrix2Vector: matrix must be of size 3x3");
		return Vector3D({ -m(1,2), m(0,2), -m(0,1)});
	}

	////template<class TVector>
	////! compute matrix B = Skew(v) * A from matrix A; A must have 3 rows
	template<class TMatrixA, class TMatrixB>
	inline void ApplySkewMatrixTemplate(const Vector3D& v, const TMatrixA& A, TMatrixB& B)
	{
		CHECKandTHROW(A.NumberOfRows() == 3, "ApplySkewMatrixTemplate: MatrixA must contain 3 rows!");
		B.SetNumberOfRowsAndColumns(3, A.NumberOfColumns());
		for (Index i = 0; i < A.NumberOfColumns(); i++)
		{
			B.GetUnsafe(0, i) =                          - A.GetUnsafe(1, i) * v[2] + A.GetUnsafe(2, i) * v[1];
			B.GetUnsafe(1, i) = A.GetUnsafe(0, i) * v[2]                            - A.GetUnsafe(2, i) * v[0];
			B.GetUnsafe(2, i) =-A.GetUnsafe(0, i) * v[1] + A.GetUnsafe(1, i) * v[0] ;
		}
	}

	//! compute matrix Skew(v) * A from matrix A; A must have 3 rows; fill result into B matrix at columnOffset
	template<class TMatrixA, class TMatrixB>
	inline void ApplySkewMatrixTemplate(const Vector3D& v, const TMatrixA& A, TMatrixB& B, Index columnOffset)
	{
		CHECKandTHROW(A.NumberOfRows() == 3, "ApplySkewMatrixTemplate: MatrixA must contain 3 rows!");
		//B.SetNumberOfRowsAndColumns(3, A.NumberOfColumns()); //do not change size of B!
		for (Index i = 0; i < A.NumberOfColumns(); i++)
		{
			B.GetUnsafe(0, i+ columnOffset) = -A.GetUnsafe(1, i) * v[2] + A.GetUnsafe(2, i) * v[1];
			B.GetUnsafe(1, i+ columnOffset) = A.GetUnsafe(0, i) * v[2] - A.GetUnsafe(2, i) * v[0];
			B.GetUnsafe(2, i+ columnOffset) = -A.GetUnsafe(0, i) * v[1] + A.GetUnsafe(1, i) * v[0];
		}
	}

	//********************************************************************************
	//functions containing EULER PARAMETERS (QUATERNIONS)

	//! compute G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	template<class TVector>
	inline ConstSizeMatrix<3*maxRotCoordinates> EP2GTemplate(const TVector& ep)
	{
		return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, { -2.*ep[1], 2.*ep[0],-2.*ep[3], 2.*ep[2],
															-2.*ep[2], 2.*ep[3], 2.*ep[0],-2.*ep[1],
															-2.*ep[3],-2.*ep[2], 2.*ep[1], 2.*ep[0] });
	}

	//! compute time derivative of G-Matrix from Euler Parameters ep_t
	template<class TVector>
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2G_tTemplate(const TVector& ep_t)
	{
		return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, {	-2.*ep_t[1], 2.*ep_t[0],-2.*ep_t[3], 2.*ep_t[2],
															-2.*ep_t[2], 2.*ep_t[3], 2.*ep_t[0],-2.*ep_t[1],
															-2.*ep_t[3],-2.*ep_t[2], 2.*ep_t[1], 2.*ep_t[0] });
	}

	////! compute transposed G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	//template<class TVector>
	//inline ConstSizeMatrix<3*maxRotCoordinates> EP2GTTemplate(const TVector& ep)
	//{
	//	return ConstSizeMatrix<3*maxRotCoordinates>(4, 3, {  -2.*ep[1],-2.*ep[2],-2.*ep[3],
	//										 2.*ep[0], 2.*ep[3],-2.*ep[2],
	//										-2.*ep[3], 2.*ep[0], 2.*ep[1],
	//										 2.*ep[2],-2.*ep[1], 2.*ep[0] });
	//}

	//! compute local G-Matrix from Euler Parameters ep; Glocal is defined such that the angular velocity given in local
	//! coordinates omegaLocal follows from: omegaLocal = Glocal*ep_t
	template<class TVector>
	inline ConstSizeMatrix<3*maxRotCoordinates> EP2GlocalTemplate(const TVector& ep)
	{
		return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, { -2.*ep[1], 2.*ep[0], 2.*ep[3],-2.*ep[2],
															-2.*ep[2],-2.*ep[3], 2.*ep[0], 2.*ep[1],
															-2.*ep[3], 2.*ep[2],-2.*ep[1], 2.*ep[0] });
	}

	////! compute transposed Glocal-Matrix from Euler Parameters ep;
	//template<class TVector>
	//inline ConstSizeMatrix<3*maxRotCoordinates> EP2GlocalTTemplate(const TVector& ep)
	//{
	//	return ConstSizeMatrix<3*maxRotCoordinates>(4, 3, { -2.*ep[1],-2.*ep[2],-2.*ep[3],
	//										 2.*ep[0],-2.*ep[3], 2.*ep[2],
	//										 2.*ep[3], 2.*ep[0],-2.*ep[1],
	//										-2.*ep[2], 2.*ep[1], 2.*ep[0] });
	//}

	//! compute time derivative of local G-Matrix from time derivative of Euler Parameters: ep_t
	template<class TVector>
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2Glocal_tTemplate(const TVector& ep_t)
	{
		return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, { -2.*ep_t[1], 2.*ep_t[0], 2.*ep_t[3],-2.*ep_t[2],
															-2.*ep_t[2],-2.*ep_t[3], 2.*ep_t[0], 2.*ep_t[1],
															-2.*ep_t[3], 2.*ep_t[2],-2.*ep_t[1], 2.*ep_t[0] });
	}

	//! compute d(G^T*v)/dq for Euler parameters
	inline ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates> EPGTv_q(const Vector3D& v)
	{
		//G^T*v:
		//(4, 4, { -2.*ep[1] * v[0] -2.*ep[2] * v[1] -2.*ep[3] * v[2],
		//		    2.*ep[0] * v[0] +2.*ep[3] * v[1] -2.*ep[2] * v[2],
		//		   -2.*ep[3] * v[0] +2.*ep[0] * v[1] +2.*ep[1] * v[2],
		//		    2.*ep[2] * v[0] -2.*ep[1] * v[1] +2.*ep[0] * v[2] });

		return ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates>(4, 4, {       0., -2.*v[0], -2.*v[1], -2.*v[2],
																			 2.*v[0],       0., -2.*v[2],  2.*v[1],
																			 2.*v[1],  2.*v[2],       0., -2.*v[0],
																			 2.*v[2], -2.*v[1],  2.*v[0],       0. });
	}

	//! compute d(G^T*v)/dq for Euler parameters
	inline ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates> EPGlocalTv_q(const Vector3D& v)
	{
		//Glocal^T*v:
		//(4, 4, { -2.*ep[1] * v[0] -2.*ep[2] * v[1] -2.*ep[3] * v[2],
		//		    2.*ep[0] * v[0] -2.*ep[3] * v[1] +2.*ep[2] * v[2],
		//		    2.*ep[3] * v[0] +2.*ep[0] * v[1] -2.*ep[1] * v[2],
		//		   -2.*ep[2] * v[0] +2.*ep[1] * v[1] +2.*ep[0] * v[2] });

		return ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates>(4, 4, {       0., -2.*v[0], -2.*v[1], -2.*v[2],
																			 2.*v[0],       0.,  2.*v[2], -2.*v[1],
																			 2.*v[1], -2.*v[2],       0.,  2.*v[0],
																			 2.*v[2],  2.*v[1], -2.*v[0],       0. });
	}

	//! compute rotation matrix from 4-components vector of Euler parameters ep
	template<class TVector>
	inline Matrix3D EP2RotationMatrixTemplate(const TVector& ep)
	{
		return Matrix3D(3, 3, { -2.0*ep[3]*ep[3] - 2.0*ep[2]*ep[2] + 1.0, -2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], 2.0*ep[3]*ep[1] + 2.0*ep[2]*ep[0],
							2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], -2.0*ep[3]*ep[3] - 2.0*ep[1]*ep[1] + 1.0, 2.0*ep[3]*ep[2] - 2.0*ep[1]*ep[0],
							-2.0*ep[2]*ep[0] + 2.0*ep[3]*ep[1], 2.0*ep[3]*ep[2] + 2.0*ep[1]*ep[0], -2.0*ep[2]*ep[2] - 2.0*ep[1]*ep[1] + 1.0 });
	}

	//! compute time derivative of rotation matrix from 4-components vector of Euler parameters ep and 
	//! 4-components vector of time derivative of Euler parameters ep_t
	template<class TVector>
	inline Matrix3D EP2RotationMatrix_tTemplate(const TVector& ep, const TVector& ep_t)
	{
		//shall be copy of function from linalg3d
		return Matrix3D(3, 3, { -4.0*ep[3] * ep_t[3] - 4.0*ep[2] * ep_t[2],
								-2.0*ep_t[3] * ep[0] - 2.0*ep[3] * ep_t[0] + 2.0*ep_t[2] * ep[1] + 2.0*ep[2] * ep_t[1],
								2.0*ep_t[3] * ep[1] + 2.0*ep[3] * ep_t[1] + 2.0*ep_t[2] * ep[0] + 2.0*ep[2] * ep_t[0],
								2.0*ep_t[3] * ep[0] + 2.0*ep[3] * ep_t[0] + 2.0*ep_t[2] * ep[1] + 2.0*ep[2] * ep_t[1],
								-4.0*ep[3] * ep_t[3] - 4.0*ep[1] * ep_t[1],
								2.0*ep_t[3] * ep[2] + 2.0*ep[3] * ep_t[2] - 2.0*ep_t[1] * ep[0] - 2.0*ep[1] * ep_t[0],
								-2.0*ep_t[2] * ep[0] - 2.0*ep[2] * ep_t[0] + 2.0*ep_t[3] * ep[1] + 2.0*ep[3] * ep_t[1],
								2.0*ep_t[3] * ep[2] + 2.0*ep[3] * ep_t[2] + 2.0*ep_t[1] * ep[0] + 2.0*ep[1] * ep_t[0],
								-4.0*ep[2] * ep_t[2] - 4.0*ep[1] * ep_t[1] });
	}

	inline void EP2Glocal(ConstSizeMatrix<3 * maxRotCoordinates>& Glocal, Real ep0, Real ep1, Real ep2, Real ep3)
	{
		Glocal.SetNumberOfRowsAndColumns(3, 4);
		Real* p = Glocal.GetDataPointer();
		p[0] = -2.*ep1; 
		p[1] = 2.*ep0;
		p[2] = 2.*ep3;
		p[3] = -2.*ep2;

		p[4] = -2.*ep2;
		p[5] = -2.*ep3;
		p[6] = 2.*ep0;
		p[7] = 2.*ep1;

		p[8] = -2.*ep3;
		p[9] = 2.*ep2;
		p[10]= -2.*ep1;
		p[11]= 2.*ep0;
	}

	inline void EP2G(ConstSizeMatrix<3 * maxRotCoordinates>& G, Real ep0, Real ep1, Real ep2, Real ep3)
	{
		G.SetNumberOfRowsAndColumns(3, 4);
		Real* p = G.GetDataPointer();
		p[0] = -2.*ep1;
		p[1] = 2.*ep0;
		p[2] = -2.*ep3;
		p[3] = 2.*ep2;
		
		p[4] = -2.*ep2;
		p[5] = 2.*ep3;
		p[6] = 2.*ep0;
		p[7] = -2.*ep1;
		
		p[8] = -2.*ep3;
		p[9] = -2.*ep2;
		p[10] = 2.*ep1;
		p[11] = 2.*ep0;
	}


	//! compute rotation matrix from Euler parameters ep0, ..., ep3
	inline Matrix3D EP2RotationMatrix(Real ep0, Real ep1, Real ep2, Real ep3)
	{
		return Matrix3D(3, 3, { -2.0*ep3*ep3 - 2.0*ep2*ep2 + 1.0, -2.0*ep3*ep0 + 2.0*ep2*ep1, 2.0*ep3*ep1 + 2.0*ep2*ep0,
							2.0*ep3*ep0 + 2.0*ep2*ep1, -2.0*ep3*ep3 - 2.0*ep1*ep1 + 1.0, 2.0*ep3*ep2 - 2.0*ep1*ep0,
							-2.0*ep2*ep0 + 2.0*ep3*ep1, 2.0*ep3*ep2 + 2.0*ep1*ep0, -2.0*ep2*ep2 - 2.0*ep1*ep1 + 1.0 });
	}

	//! compute rotation matrix from Euler parameters ep0, ..., ep3
	inline void EP2RotationMatrix(Matrix3D& A, Real ep0, Real ep1, Real ep2, Real ep3)
	{
		A.SetNumberOfRowsAndColumns(3, 3);
		Real* p = A.GetDataPointer();
		p[0] = -2.0*ep3*ep3 - 2.0*ep2*ep2 + 1.0;
		p[1] = -2.0*ep3*ep0 + 2.0*ep2*ep1;
		p[2] = 2.0*ep3*ep1 + 2.0*ep2*ep0;
		p[3] = 2.0*ep3*ep0 + 2.0*ep2*ep1;
		p[4] = -2.0*ep3*ep3 - 2.0*ep1*ep1 + 1.0;
		p[5] = 2.0*ep3*ep2 - 2.0*ep1*ep0;
		p[6] = -2.0*ep2*ep0 + 2.0*ep3*ep1;
		p[7] = 2.0*ep3*ep2 + 2.0*ep1*ep0;
		p[8] = -2.0*ep2*ep2 - 2.0*ep1*ep1 + 1.0;

		//optimized version, slower!:
		//Real ep3ep3 = ep3 * ep3;
		//Real ep2ep2 = ep2 * ep2;
		//Real ep1ep1 = ep1 * ep1;
		//Real ep0ep0 = ep0 * ep0;
		//Real ep3ep0 = ep3 * ep0;
		//Real ep3ep1 = ep3 * ep1;
		//Real ep3ep2 = ep3 * ep2;
		//Real ep2ep0 = ep2 * ep0;
		//Real ep2ep1 = ep2 * ep1;
		//Real ep1ep0 = ep1 * ep0;

		//p[0] = -2.0*(ep3ep3 + ep2ep2) + 1.0;
		//p[1] = -2.0*(ep3ep0 - ep2ep1);
		//p[2] = 2.0*(ep3ep1 + ep2ep0);
		//p[3] = 2.0*(ep3ep0 + ep2ep1);
		//p[4] = -2.0*(ep3ep3 + ep1ep1) + 1.0;
		//p[5] = 2.0*(ep3ep2 - ep1ep0);
		//p[6] = -2.0*(ep2ep0 - ep3ep1);
		//p[7] = 2.0*(ep3ep2 + ep1ep0);
		//p[8] = -2.0*(ep2ep2 + ep1ep1) + 1.0;
	}

	//! compute euler parameters ep0, ..., ep3 from rotation matrix A
	inline void RotationMatrix2EP(const Matrix3D& A,
		Real& ep0, Real& ep1, Real& ep2, Real& ep3)
	{
		Real trace = A(0, 0) + A(1, 1) + A(2, 2) + 1.0;
		Real M_EPSILON = 1e-15; //small number to avoid division by zero

		if (fabs(trace) > M_EPSILON)
		{
			Real s = 0.5 / sqrt(fabs(trace));
			ep0 = 0.25 / s;
			ep1 = (A(2, 1) - A(1, 2)) * s;
			ep2 = (A(0, 2) - A(2, 0)) * s;
			ep3 = (A(1, 0) - A(0, 1)) * s;
		}
		else
		{
			if (A(0, 0) > A(1, 1) && A(0, 0) > A(2, 2)) {
				Real s = 2.0 * sqrt(fabs(1.0 + A(0, 0) - A(1, 1) - A(2, 2)));
				ep1 = 0.25 * s;
				ep2 = (A(0, 1) + A(1, 0)) / s;
				ep3 = (A(0, 2) + A(2, 0)) / s;
				ep0 = (A(1, 2) - A(2, 1)) / s;

			}
			else if (A(1, 1) > A(2, 2)) {
				Real s = 2.0 * sqrt(fabs(1.0 + A(1, 1) - A(0, 0) - A(2, 2)));
				ep1 = (A(0, 1) + A(1, 0)) / s;
				ep2 = 0.25 * s;
				ep3 = (A(1, 2) + A(2, 1)) / s;
				ep0 = (A(0, 2) - A(2, 0)) / s;
			}
			else {
				Real s = 2.0 * sqrt(fabs(1.0 + A(2, 2) - A(0, 0) - A(1, 1)));
				ep1 = (A(0, 2) + A(2, 0)) / s;
				ep2 = (A(1, 2) + A(2, 1)) / s;
				ep3 = 0.25 * s;
				ep0 = (A(0, 1) - A(1, 0)) / s;
			}
		}
	}

	//********************************************************************************
	//Euler angle / Tait-Bryan functions (xyz-rotations sequence, i.e., Rxyz = Rx*Ry*Rz)
	// References:
	// Nikravesh(Computer - Aided Analysis of Mechanical Systems, P 347 ff)
	//   and 
	// Geradin and Cardona(Flexible Multibody Dynamics - A Finite Element Approach) page 84 ff.
	
	//! get rotation matrix around X-axis (inefficient, use only for test purposes)
	inline Matrix3D RotationMatrixX(Real angleRad)
	{
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		return Matrix3D(3, 3, { 
			1., 0., 0.,
			0., c, -s,
			0., s,  c });
	}

	//! get rotation matrix around Y-axis (inefficient, use only for test purposes)
	inline Matrix3D RotationMatrixY(Real angleRad)
	{
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		return Matrix3D(3, 3, { 
			 c , 0.,  s,
			 0., 1.,  0.,
			-s , 0.,  c });
	}

	//! get rotation matrix around Z-axis (inefficient, use only for test purposes)
	inline Matrix3D RotationMatrixZ(Real angleRad)
	{
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		return Matrix3D(3, 3, {
			c, -s,  0.,
			s,  c,  0.,
			0., 0., 1.});
	}

	//! convert Euler angles (Tait-Bryan angles) to rotation matrix
	template<class TVector>
	inline Matrix3D RotXYZ2RotationMatrixTemplate(const TVector& rot)
	{
		//Real psi = rot[0];
		//Real theta = rot[1];
		//Real phi = rot[2];
		Real c0 = cos(rot[0]);
		Real s0 = sin(rot[0]);
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);
		Real c2 = cos(rot[2]);
		Real s2 = sin(rot[2]);

		return Matrix3D(3,3,{ c1*c2,-c1 * s2,s1, 
			                  s0*s1*c2 + c0 * s2, -s0 * s1*s2 + c0 * c2,-s0 * c1,
			                 -c0 * s1*c2 + s0 * s2,c0*s1*s2 + s0 * c2,c0*c1 });
	}
	inline Matrix3D RotXYZ2RotationMatrix(const Vector3D& rot) { return RotXYZ2RotationMatrixTemplate(rot); }
	inline Matrix3D RotXYZ2RotationMatrix(const CSVector3D& rot) { return RotXYZ2RotationMatrixTemplate(rot); }
	inline Matrix3D RotXYZ2RotationMatrix(const CSVector4D& rot) { return RotXYZ2RotationMatrixTemplate(rot); } //for NodeRigidBody compatibility functions

	//! convert rotation matrix to Euler angles Rxyz (Tait-Bryan angles)
	inline Vector3D RotationMatrix2RotXYZ(const Matrix3D& R)
	{
		Vector3D rot;

		Real absC1 = sqrt(EXUstd::Square(-R(1, 2)) + EXUstd::Square(R(2, 2)) );
		rot[1] = atan2(R(0, 2), absC1);
		if (absC1 > 1e-14)
		{
			rot[0] = atan2(-R(1, 2), R(2, 2));
			rot[2] = atan2(-R(0, 1), R(0, 0));
		}
		else //c1 = 0, s0 = 0, c0 = 1:  #rot[0] and rot[2] represent same axes, set one of them zero!
		{
			rot[0] = 0.;
			//s0*s1*c2 + c0 * s2, -s0 * s1*s2 + c0 * c2 = > c0*s2, c0*c2
			rot[2] = atan2(R(1, 0), R(1, 1));
		}

		//Vector3D rot;
		//rot[0] = atan2(-R(1, 2), R(2, 2));
		//rot[1] = atan2(R(0, 2), sqrt(fabs(1. - R(0, 2) * R(0, 2)))); //fabs for safety, if small round up error in rotation matrix ...
		//rot[2] = atan2(-R(0, 1), R(0, 0));
		return rot;
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	template<class TVector>
	inline ConstSizeMatrix<3*maxRotCoordinates> RotXYZ2GTemplate(const TVector& rot)
	{
		Real c0 = cos(rot[0]);
		Real s0 = sin(rot[0]);
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);

		return ConstSizeMatrix<3*maxRotCoordinates>(3, 3, 
			{ 1, 0, s1,
			  0, c0, -c1*s0,
			  0, s0,  c0*c1 });
	}

	//! compute time derivative of G-Matrix from Tait Bryan angles rot_t
	template<class TVector1, class TVector2>
	inline ConstSizeMatrix<3*maxRotCoordinates> RotXYZ2G_tTemplate(const TVector1& rot, const TVector2& rot_t)
	{
		Real c0 = cos(rot[0]);
		Real s0 = sin(rot[0]);
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);

		return ConstSizeMatrix<3*maxRotCoordinates>(3, 3,
			{ 0, 0, rot_t[1]*c1,
			  0, -rot_t[0]*s0, rot_t[1]*s0*s1 - rot_t[0]*c0*c1,
			  0, rot_t[0]*c0, -rot_t[0]*c1*s0 - rot_t[1]*c0*s1 });
	}


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	template<class TVector>
	inline ConstSizeMatrix<3*maxRotCoordinates> RotXYZ2GlocalTemplate(const TVector& rot)
	{
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);
		Real c2 = cos(rot[2]);
		Real s2 = sin(rot[2]);

		return ConstSizeMatrix<3*maxRotCoordinates>(3, 3, 
			{ c1*c2, s2, 0,
			  -c1*s2, c2, 0,
			  s1, 0, 1 });
	}

	//! compute time derivative of local G-Matrix from time derivative of Tait Bryan angles: ep_t
	template<class TVector1, class TVector2>
	inline ConstSizeMatrix<3*maxRotCoordinates> RotXYZ2Glocal_tTemplate(const TVector1& rot, const TVector2& rot_t)
	{
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);
		Real c2 = cos(rot[2]);
		Real s2 = sin(rot[2]);

		return ConstSizeMatrix<3*maxRotCoordinates>(3, 3, 
			{ -rot_t[2]*c1*s2 - rot_t[1]*c2*s1, rot_t[2]*c2, 0,
			   rot_t[1]*s2*s1 - rot_t[2]*c2*c1, -rot_t[2]*s2, 0,
			   rot_t[1]*c1, 0, 0 });
	}

	//! compute d(G^T*v)/dq for RotXYZ parameters
	template<class TVector>
	inline ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates> RotXYZGTv_qTemplate(const TVector& rot, const Vector3D& v)
	{
		Real c0 = cos(rot[0]);
		Real s0 = sin(rot[0]);
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);

		//G^T
		//return ConstSizeMatrix<3 * maxRotCoordinates>(3, 3,
		//	{ 1,        0,     0,
		//	  0,       c0,    s0,
		//	 s1, -c1 * s0, c0*c1 });

		//G^T*v:
		//	{ 1*v[0]+       0*v[1]+     0*v[2],
		//	  0*v[0]+      c0*v[1]+    s0*v[2],
		//	 s1*v[0]- c1 * s0*v[1]+ c0*c1*v[2] });

		return ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates>(3, 3, {
				                     0.,                             0., 0.,
				       -s0*v[1]+c0*v[2],                             0., 0.,
				 -c1*c0*v[1]-s0*c1*v[2], -c1*v[0]+s1*s0*v[1]-c0*s1*v[2], 0. });
	}

	//! compute d(Glocal^T*v)/dq for RotXYZ parameters
	template<class TVector>
	inline ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates> RotXYZGlocalTv_qTemplate(const TVector& rot, const Vector3D& v)
	{
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);
		Real c2 = cos(rot[2]);
		Real s2 = sin(rot[2]);

		//Glocal^T
		//return ConstSizeMatrix<3 * maxRotCoordinates>(3, 3,
		//	{ c1*c2, -c1 * s2, s1,
		//	     s2,       c2,  0,
		//	      0,        0,  1 });

		//Glocal^T*v:
		//	{ c1*c2*v[0]  -c1 * s2*v[1] +s1*v[2],
		//	     s2*v[0]  +     c2*v[1] + 0*v[2],
		//	      0*v[0]  +      0*v[1] + 1*v[2] });

		return ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates>(3, 3, {
				0.,-s1*c2*v[0] + s1 * s2*v[1] + c1 * v[2],-c1*s2*v[0] - c1 * c2*v[1],
				0.,                                    0.,       c2*v[0] - s2 * v[1],
				0.,                                    0.,                        0. });
	}


	//inline Matrix3D RotationVector2RotationMatrix(const Vector3D& rot) { return RotationVector2RotationMatrixTemplate(rot); }
	//inline Matrix3D RotationVector2RotationMatrix(const CSVector4D& rot) { return RotationVector2RotationMatrixTemplate(rot); } //for NodeRigidBody compatibility functions
	//********************************************************************************

	//! specializations of templates:
	inline ConstSizeMatrix<9> Vector2SkewMatrix(const Vector3D& v) { return Vector2SkewMatrixTemplate<Vector3D>(v); }
	inline ConstSizeMatrix<9> Vector2SkewMatrix(const CSVector3D& v) { return Vector2SkewMatrixTemplate<CSVector3D>(v); }

	//! compute G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	inline ConstSizeMatrix<3*maxRotCoordinates> EP2G(const Vector4D& ep) { return EP2GTemplate<Vector4D>(ep); }
	inline ConstSizeMatrix<3*maxRotCoordinates> EP2G(const CSVector4D& ep) { return EP2GTemplate<CSVector4D>(ep); }

	//! compute time derivative of G-Matrix from Euler Parameters ep_t
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2G_t(const Vector4D& ep_t) { return EP_t2G_tTemplate<Vector4D>(ep_t); }
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2G_t(const CSVector4D& ep_t) { return EP_t2G_tTemplate<CSVector4D>(ep_t); }
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2G_t(const LinkedDataVector& ep_t) { return EP_t2G_tTemplate<LinkedDataVector>(ep_t); }

	////! compute transposed G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	//inline ConstSizeMatrix<3*maxRotCoordinates> EP2GT(const Vector4D& ep) { return EP2GTTemplate<Vector4D>(ep); }
	//inline ConstSizeMatrix<3*maxRotCoordinates> EP2GT(const CSVector4D& ep) { return EP2GTTemplate<CSVector4D>(ep); }

	//! compute local G-Matrix from Euler Parameters ep; Glocal is defined such that the angular velocity given in local
	//! coordinates omegaLocal follows from: omegaLocal = Glocal*ep_t
	inline ConstSizeMatrix<3*maxRotCoordinates> EP2Glocal(const Vector4D& ep) { return EP2GlocalTemplate<Vector4D>(ep); }
	inline ConstSizeMatrix<3*maxRotCoordinates> EP2Glocal(const CSVector4D& ep) { return EP2GlocalTemplate<CSVector4D>(ep); }

	////! compute transposed Glocal-Matrix from Euler Parameters ep;
	//inline ConstSizeMatrix<3*maxRotCoordinates> EP2GlocalT(const Vector4D& ep) { return EP2GlocalTTemplate<Vector4D>(ep); }
	//inline ConstSizeMatrix<3*maxRotCoordinates> EP2GlocalT(const CSVector4D& ep) { return EP2GlocalTTemplate<CSVector4D>(ep); }

	//! compute time derivative of local G-Matrix from time derivative of Euler Parameters: ep_t
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2Glocal_t(const Vector4D& ep_t) { return EP_t2Glocal_tTemplate<Vector4D>(ep_t); }
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2Glocal_t(const CSVector4D& ep_t) { return EP_t2Glocal_tTemplate<CSVector4D>(ep_t); }
	inline ConstSizeMatrix<3*maxRotCoordinates> EP_t2Glocal_t(const LinkedDataVector& ep_t) { return EP_t2Glocal_tTemplate<LinkedDataVector>(ep_t); }

	//! compute rotation matrix from 4-components vector of Euler parameters ep
	inline Matrix3D EP2RotationMatrix(const Vector4D& ep) { return EP2RotationMatrixTemplate(ep); }
	inline Matrix3D EP2RotationMatrix(const CSVector4D& ep) { return EP2RotationMatrixTemplate(ep); }

	//! compute time derivative of rotation matrix from 4-components vector of Euler parameters ep and 
	//! 4-components vector of time derivative of Euler parameters ep_t
	inline Matrix3D EP2RotationMatrix_t(const Vector4D& ep, const Vector4D& ep_t) { return EP2RotationMatrix_tTemplate<Vector4D>(ep, ep_t); }
	inline Matrix3D EP2RotationMatrix_t(const CSVector4D& ep, const CSVector4D& ep_t) { return EP2RotationMatrix_tTemplate<CSVector4D>(ep, ep_t); }


	//********************************************************************************
	//Rotation vector; see paper Holzinger, Gerstmayr; Multibody System Dynamics 2020, submitted

	//! convert rotation vector to rotation matrix
	template<class TVector>
	inline Matrix3D RotationVector2RotationMatrix(const TVector& rot)
	{
		Vector3D v;
		v.CopyFrom(rot);
		Real angle = rot.GetL2Norm();

		if (angle == 0) {
			return EXUmath::unitMatrix3D;
		}
		else
		{
			const double twoPi = 2.0 * EXUstd::pi;
			if (angle > twoPi)
			{
				angle = std::fmod(angle, twoPi);
			}

			Matrix3D mat(EXUmath::unitMatrix3D);
			Matrix3D vTilde(Vector2SkewMatrix(v));

			Real sAngle = sin(angle);
			mat += (sAngle / angle)*vTilde;

			//Real cAngle = cos(angle);
			//mat += ((1. - cAngle) / (angle * angle))*vTilde*vTilde; //very inaccurate around angle=0

			Real sAngle2 = sin(0.5*angle);
			mat += (2. * sAngle2 * sAngle2 / (angle * angle))*vTilde*vTilde;

			return mat;
		}
	}

	//! compute d(G^T*v)/dq for rotation vector (Glocal = I, G = RotationMatrix)
	//template<class TVector>
	//inline ConstSizeMatrix<maxRotCoordinates*maxRotCoordinates> RotationVectorGTv_qTemplate(const TVector& rot, const Vector3D& v)
	//==> use autodiff!, see CNodeRigidBodyRotVecLG.cpp

	//********************************************************************************
	//simple functions for ROTATION MATRICES

	//! compute rotation matrix from single rotation around axis 1 (x-axis)
	template<class T>
	inline ConstSizeMatrixBase<T, 9> RotationMatrix1(T phi)
	{
		return ConstSizeMatrixBase<T, 9>(3, 3, 
			{ 1, 0, 0,
			  0, cos(phi),-sin(phi),
			  0, sin(phi), cos(phi) });
	}

	//! compute rotation matrix from single rotation around axis 2 (y-axis)
	template<class T>
	inline ConstSizeMatrixBase<T, 9> RotationMatrix2(T phi)
	{
		return ConstSizeMatrixBase<T, 9>(3, 3,
			{ cos(phi), 0, sin(phi),
			  0,        1, 0,
			  -sin(phi),0, cos(phi) });
	}

	//! compute rotation matrix from single rotation around axis 3 (z-axis)
	template<class T>
	inline ConstSizeMatrixBase<T, 9> RotationMatrix3(T phi)
	{
		return ConstSizeMatrixBase<T, 9>(3, 3,
			{ cos(phi),-sin(phi), 0,
			  sin(phi), cos(phi), 0,
			  0,	    0,        1 });
	}

	//! compute local (=body fixed) inertia matrix from 6 scalar inertia components [J_{xx}, J_{yy}, J_{zz}, J_{yz}, J_{xz}, J_{xy}]
	inline void ComputeInertiaMatrix(const Vector6D& inertiaParameters, ConstSizeMatrix<9>& inertiaMatrix)
	{
		inertiaMatrix.SetMatrix(3, 3, {
			inertiaParameters[0], inertiaParameters[5], inertiaParameters[4],
			inertiaParameters[5], inertiaParameters[1], inertiaParameters[3],
			inertiaParameters[4], inertiaParameters[3], inertiaParameters[2] });
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//helper functions mostly for FFRF Object computations

	//!apply transformation A (\in n x n) to vector (which must have a multiple of n components):
	inline void ApplyTransformation(const Matrix3D& A, Vector& vector)
	{
		Index nDim = A.NumberOfColumns();
		Index nNodes = vector.NumberOfItems() / nDim;
		CHECKandTHROW(nNodes*nDim == vector.NumberOfItems(), "ApplyTransformation: vector must have appropriate size");

		for (Index i = 0; i < nNodes; i++)
		{
			LinkedDataVector vSub(vector, i*nDim, nDim);
			Vector3D v(vSub, 0);
			EXUmath::MultMatrixVector(A, v, vSub);
		}
	}

	//!apply transformation A (\in n x n) to (temporary) vector (which must have a multiple of n components) and add it to result (which must have same size):
	inline void ApplyTransformationAndAdd(const Matrix3D& A, const Vector& vector, Vector& result)
	{
		Index nDim = A.NumberOfColumns();
		Index nNodes = vector.NumberOfItems() / nDim;
		CHECKandTHROW(nNodes*nDim == vector.NumberOfItems(), "ApplyTransformationAndAdd: vector must have appropriate size");
		CHECKandTHROW(nNodes*nDim == result.NumberOfItems(), "ApplyTransformationAndAdd: result must have appropriate size");

		for (Index i = 0; i < nNodes; i++)
		{
			//Vector3D v(vector, i*nDim); //needs copy
			LinkedDataVector vectorSub(vector, i*nDim, nDim);
			LinkedDataVector resultSub(result, i*nDim, nDim);
			EXUmath::MultMatrixVectorAdd(A, vectorSub, resultSub);
		}
	}

	//! fill n-times a Matrix3D matrix A into a column matrix of size (3*n x 3)
	inline void ComputeBlockColumnMatrix(Index n, const Matrix3D& A, Matrix& destination)
	{
		destination.SetNumberOfRowsAndColumns(3 * n, 3);
		for (Index i = 0; i < n; i++)
		{
			destination.SetSubmatrix(A, i * 3, 0);
		}
	}

	//! compute the skew matrix of all (x,y,z) groups of the vector (3*n) and write into skewMatrix (3*n x 3)
	inline void ComputeSkewMatrix(const Vector& vector, Matrix& skewMatrix)
	{
		Index nn = vector.NumberOfItems() / 3;
		CHECKandTHROW(nn * 3 == vector.NumberOfItems(), "ComputeSkewMatrix: vector must have length which can be divided by 3");

		skewMatrix.SetNumberOfRowsAndColumns(vector.NumberOfItems(), 3);
		for (Index i = 0; i < nn; i++)
		{
			Index j = i * 3;
			skewMatrix.SetSubmatrix(RigidBodyMath::Vector2SkewMatrix(Vector3D({ vector[j], vector[j + 1], vector[j + 2] })), j, 0);
		}
	}

	//! compute kronecker product of vector (in R^n) and 3D unit matrix, giving (3*n x 3) components
	inline void VectorKroneckerUnitMatrix3D(const Vector& vector, Matrix& result)
	{
		//def VectorDiadicUnitMatrix3D(v) :
		//	return np.kron(np.array(v), np.eye(3)).T
		Index n = vector.NumberOfItems();
		result.SetNumberOfRowsAndColumns(3 * n, 3);
		for (Index i = 0; i < n; i++)
		{
			Real v = vector[i];
			result(3 * i    , 0) = v; result(3 * i    , 1) = 0; result(3 * i    , 2) = 0;
			result(3 * i + 1, 0) = 0; result(3 * i + 1, 1) = v; result(3 * i + 1, 2) = 0;
			result(3 * i + 2, 0) = 0; result(3 * i + 2, 1) = 0; result(3 * i + 2, 2) = v;
		}
	}


} //namespace RigidBodyMath







//! class representing (efficiently) homogeneous transformations; 
//! follows widely the Python implementation in exudyn.rigidBodyUtilities and lieGroupBasics
//! UNTESTED (not in unit tests)
template<typename T>
class HomogeneousTransformationBase
{
private:
	ConstSizeMatrixBase<T, 9> A; //!< rotation part, [0:3,0:3] of matrix
	SlimVectorBase<T,3> v; //!< translation part, [3,0:3] of matrix
public:
	//! initialize with identity transformation
	HomogeneousTransformationBase(bool initialize = true)
	{
		A.SetScalarMatrix(3, (T)1.);
		v.SetAll(0);
	}

	//! initialize with rotation matrix and translation
	HomogeneousTransformationBase(const ConstSizeMatrixBase<T, 9>& rotation, const SlimVectorBase<T,3>& translation) : A(rotation), v(translation)
	{
	}

	//! return 4x4 HT matrix; this is less efficient and should only be used for output or at the end of computations
	ConstSizeMatrixBase<T, 16> GetHT44() const
	{
		ConstSizeMatrixBase<T, 16> HT(4, 4);
		for (Index i = 0; i < 3; i++)
		{
			HT(3, i) = 0.;
			HT(i, 3) = v[i];
			for (Index j = 0; j < 3; j++)
			{
				HT(i, j) = A(i, j);
			}
		}
		HT(3, 3) = (T)1.;
		return HT;
	}

	//! return 4x4 HT matrix as float; this is less efficient and should only be used for output or at the end of computations
	ConstSizeMatrixBase<T, 16> GetHT44F() const
	{
		ConstSizeMatrixF<16> HT(4, 4);
		for (Index i = 0; i < 3; i++)
		{
			HT(3, i) = 0.f;
			HT(i, 3) = (float)v[i];
			for (Index j = 0; j < 3; j++)
			{
				HT(i, j) = (float)A(i, j);
			}
		}
		HT(3, 3) = 1.f;
		return HT;
	}

	//! use 4x4 HT matrix to set values of homogeneous transformation
	void SetHT44(const ConstSizeMatrixBase<T, 16>& HT)
	{
		for (Index i = 0; i < 3; i++)
		{
			v[i] = HT(i, 3);
			for (Index j = 0; j < 3; j++)
			{
				A(i, j) = HT(i, j);
			}
		}
	}

	//! set with any kind of 4x4 HT matrix
	template<typename MatrixT>
	void SetHT44(const MatrixT& HT)
	{
		CHECKandTHROW(HT.NumberOfRows() == 4 && HT.NumberOfColumns() == 4, "HomogeneousTransformationBase::SetHT44: wrong matrix dimension");
		for (Index i = 0; i < 3; i++)
		{
			v[i] = HT(i, 3);
			for (Index j = 0; j < 3; j++)
			{
				A(i, j) = HT(i, j);
			}
		}
	}

	//! set a translation and identity rotation
	void SetTranslation(const SlimVectorBase<T,3>& translation)
	{
		A.SetScalarMatrix(3, (T)1.);
		v = translation;
	}

	//! set a translation along x-axis and identity rotation
	void SetTranslationX(T x)
	{
		A.SetScalarMatrix(3, (T)1.);
		v = SlimVectorBase<T,3>({ x,0,0 });
	}

	//! set a translation along y-axis and identity rotation
	void SetTranslationY(T y)
	{
		A.SetScalarMatrix(3, (T)1.);
		v = SlimVectorBase<T,3>({ 0,y,0 });
	}

	//! set a translation along z-axis and identity rotation
	void SetTranslationZ(T z)
	{
		A.SetScalarMatrix(3, (T)1.);
		v = SlimVectorBase<T,3>({ 0,0,z });
	}

	//! set identity
	void SetIdentity()
	{
		A.SetScalarMatrix(3, (T)1.);
		v.SetAll(0);
	}

	//! set a rotation around x-axis and zero translation
	void SetRotationX(T angleRad)
	{
		v = SlimVectorBase<T,3>({ 0.,0.,0. });
		T c = cos(angleRad);
		T s = sin(angleRad);
		A.SetMatrix(3, 3, {
			(T)1.,0.,0.,
			0., c,-s,
			0., s, c });
	}

	//! set a rotation around y-axis and zero translation
	void SetRotationY(T angleRad)
	{
		v = SlimVectorBase<T,3>({ 0.,0.,0. });
		T c = cos(angleRad);
		T s = sin(angleRad);
		A.SetMatrix(3, 3, {
			c ,0., s,
			0.,(T)1.,0.,
			-s,0., c });
	}

	//! set a rotation around z-axis and zero translation
	void SetRotationZ(T angleRad)
	{
		v = SlimVectorBase<T,3>({ 0.,0.,0. });
		T c = cos(angleRad);
		T s = sin(angleRad);
		A.SetMatrix(3, 3, {
			c ,-s,0.,
			s , c,0.,
			0.,0.,(T)1. });
	}

	//! set a rotation and zero translation
	void SetRotation(const ConstSizeMatrixBase<T, 9>& rotation)
	{
		A = rotation;
		v = SlimVectorBase<T, 3>({ 0.,0.,0. });
	}

	//! set a rotation and zero translation from rotation vector
	void SetRotation(const SlimVectorBase<T, 3>& rotation);


	//! get translation part (read)
	const SlimVectorBase<T, 3>& GetTranslation() const
	{
		return v;
	}

	//! get translation part (read)
	Float3 GetTranslationF() const
	{
		return Float3({ (float)v[0], (float)v[1], (float)v[2]});
	}

	//! get translation part (write)
	SlimVectorBase<T,3>& GetTranslation()
	{
		return v;
	}

	//! get rotation part (read)
	const ConstSizeMatrixBase<T, 9>& GetRotation() const
	{
		return A;
	}

	//! get rotation part (write)
	ConstSizeMatrixBase<T, 9>& GetRotation()
	{
		return A;
	}

	//! get translation part (read)
	Matrix3DF GetRotationF() const
	{
		return Matrix3DF(3,3,{ 
			(float)A(0,0), (float)A(0,1), (float)A(0,2), 
			(float)A(1,0), (float)A(1,1), (float)A(1,2),
			(float)A(2,0), (float)A(2,1), (float)A(2,2)
			});
	}

	//! invert transfromation
	void Invert()
	{
		A.TransposeYourself();
		v = -(A * v);
	}

	//! return inverse homogeneous transformation
	HomogeneousTransformationBase GetInverse() const
	{
		HomogeneousTransformationBase HTinv;
		HTinv.GetRotation() = A.GetTransposed();
		HTinv.GetTranslation() = -(HTinv.GetRotation() * v);
		return HTinv;
	}

	//! convert skew matrix representation (as returned e.g. by LogSE3) to inremental rotation and displacement
	void Skew2Vector(SlimVectorBase<T,3>& incDisp, SlimVectorBase<T,3>& incRot)
	{
		incRot[0] = A(2, 1);
		incRot[1] = A(0, 2);
		incRot[2] = A(1, 0); //Python uses -A(0,1)
		incDisp[0] = v[0];
		incDisp[1] = v[1];
		incDisp[2] = v[2];
	}

	//! get difference of *thi frame to HT1 as logarithm of relative homogeneous transformations
	//! note that *this*ExpSE(incDisp, incRot) = *this * this->GetInverse() * HT1 = HT1
	void GetRelativeMotionTo(const HomogeneousTransformationBase& HT1,
		SlimVectorBase<T,3>& incDisp, SlimVectorBase<T,3>& incRot);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! comparison operator, component-wise compare; MATRIX DIMENSIONS MUST BE SAME; returns true, if all components are equal
	bool operator== (const HomogeneousTransformationBase& HT) const
	{
		return (HT.GetRotation() == A && HT.GetTranslation() == v);
	}

	//! multiplication of two homogeneous transformations
	friend HomogeneousTransformationBase operator* (const HomogeneousTransformationBase& HT1, const HomogeneousTransformationBase& HT2)
	{
		HomogeneousTransformationBase result;
		result.GetRotation() = HT1.GetRotation() * HT2.GetRotation();
		result.GetTranslation() = HT1.GetRotation() * HT2.GetTranslation() + HT1.GetTranslation();
		return result;
	}

	//! multiplication of homogeneous transformation with translation: [0r20 1] = [01T 0r10; 0 1] * [1r21 1]
	friend SlimVectorBase<T,3> operator* (const HomogeneousTransformationBase& HT1, const SlimVectorBase<T,3>& translation)
	{
		return HT1.GetRotation() * translation + HT1.GetTranslation();
	}

	HomogeneousTransformationBase& operator*= (const HomogeneousTransformationBase& other)
	{
		A = A * other.GetRotation();
		v += A * other.GetTranslation();
		return *this;
	}

	friend std::ostream& operator<<(std::ostream& os, const HomogeneousTransformationBase& HT)
	{
		os << "[" << HT.GetRotation() << ", " << HT.GetTranslation() << "]";
		return os;
	}


};

namespace EXUlie {

	//! compute matrix exponential of 3D rotation vector rot; (rot is often denoted as Omega in literature ...)
	template<typename T>
	inline ConstSizeMatrixBase<T, 9> ExpSO3(const SlimVectorBase<T,3>& rot)
	{
		T phi = rot.GetL2Norm();
		ConstSizeMatrixBase<T, 9> R(EXUmath::Identity3D<T>);
		ConstSizeMatrixBase<T, 9> rotSkew;
		rotSkew.SetSkewMatrix(rot);
		R += EXUmath::Sinc(phi) * rotSkew + ((T)0.5 * EXUstd::Square(EXUmath::Sinc((T)0.5 * phi))) * rotSkew * rotSkew;
		return R;
	}


	//! compute the matrix logarithmic map on the Lie group SO(3)
	template<typename T>
	inline ConstSizeMatrixBase<T, 9> LogSO3(const ConstSizeMatrixBase<T, 9>& R)
	{
		T ep0;
		SlimVectorBase<T,3> n;
		RigidBodyMath::RotationMatrix2EP(R, ep0, n[0], n[1], n[2]);
		T norm = n.GetL2Norm();

		T phi = (T)2. * atan2(norm, ep0);
		if (norm != 0.) { n = ((T)1. / norm) * n; }

		return RigidBodyMath::Vector2SkewMatrix(phi * n);

		//old, less accurate!!!
		//ConstSizeMatrixBase<T, 9> X;
		//T val = 0.5*(R.Trace() - (T)1.);
		//	
		//if (fabs(val) > (T)1.) //#if slightly larger than 1, due to numerical differentiation
		//{ 
		//	val = val / fabs(val); 
		//}
		//
		//T phi = acos(val);
		//if (phi == 0.)
		//{
		//	X.SetScalarMatrix(3,0.); 
		//}
		//else
		//{
		//	X = R - R.GetTransposed();
		//	X *= phi / (2. * sin(phi));
		//}
		//return X;
	}

	//! compute the vector of matrix logarithmic map on the Lie group SO(3)
	template<typename T>
	inline SlimVectorBase<T,3> LogSO3Vector(const ConstSizeMatrixBase<T, 9>& R)
	{
		T ep0;
		SlimVectorBase<T,3> n;
		RigidBodyMath::RotationMatrix2EP(R, ep0, n[0], n[1], n[2]);
		T norm = n.GetL2Norm();

		T phi = (T)2. * atan2(norm, ep0);
		if (norm != 0.) { n = ((T)1. / norm) * n; }

		return phi * n;
	}

	//! compute the tangent operator corresponding to ExpSO3, see \cite{Bruels2011}; improved version using polynomial expansion
	template<typename T>
	inline ConstSizeMatrixBase<T, 9> TExpSO3(const SlimVectorBase<T,3>& rot)
	{
		T phi = rot.GetL2Norm();
		ConstSizeMatrixBase<T, 9> Rot(false);
		Rot.SetScalarMatrix(3, (T)1.);

		if (phi != (T)0.)
		{
			ConstSizeMatrixBase<T, 9> rotSkew(false);
			rotSkew.SetSkewMatrix(rot);
			T t1 = -(T)0.5 * EXUstd::Square(EXUmath::Sinc(phi / 2));
			T t2;
			T phi2 = phi * phi;
			if (phi < 0.01)
			{
				t2 = (T)1. / 6 - ((T)1. / 120) * phi2 + ((T)1. / 5040) * phi2 * phi2;
			}
			else
			{
				t2 = ((T)1. / (phi2)) * (1 - (sin(phi) / phi)); //sinc not needed here, because phi >= 0.01
			}
			Rot += t1 * rotSkew;
			Rot += t2 * rotSkew * rotSkew;
		}
		return Rot;
	}

	//! compute the inverse of the tangent operator TExpSO3, see \cite{Sonneville2014}; improved version of Stefan Holzinger
	//! outputs 3x3 matrix
	template<typename T>
	inline ConstSizeMatrixBase<T, 9> TExpSO3Inv(const SlimVectorBase<T,3>& Omega)
	{
		T phi = Omega.GetL2Norm();
		ConstSizeMatrixBase<T, 9> Tinv = EXUmath::Identity3D<T>;
		if (phi != (T)0.)
		{
			T phi2 = EXUstd::Square(phi);
			ConstSizeMatrixBase<T, 9> omegaDiadicOmega(3, 3);
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					omegaDiadicOmega(i, j) = Omega[i] * Omega[j];
				}
			}

			if (phi <= (T)0.02)
			{
				T c = ((T)1. / 12) + ((T)1. / 720) * phi2 + ((T)1. / 30240) * EXUstd::Square(phi2);//  + ((T)1. / 1209600)*phi**6 # + ((T)1. / 47900160)*phi**8
				T b = 1 - c * phi2;
				Tinv *= b; //Tinv = b*I

				Tinv += RigidBodyMath::Vector2SkewMatrix(0.5 * Omega) + c * omegaDiadicOmega;
			}
			else
			{
				T epsilon = (T)0.5 * phi;
				T beta = epsilon * EXUmath::Cot(epsilon);
				T gamma = (1 - beta) / (phi2);
				Tinv *= beta;
				Tinv += RigidBodyMath::Vector2SkewMatrix((T)0.5 * Omega) + gamma * omegaDiadicOmega;

			}
		}
		return Tinv;
	}

	//! compute rotation axis from given rotation vector
	template <class T>
	inline void RotationVector2RotationAxis(const SlimVectorBase<T, 3>& rotationVector, SlimVectorBase<T, 3>& rotationAxis)
	{
		//compute rotation angle
		T rotationAngle = rotationVector.GetL2Norm();

		if (rotationAngle == (T)0.) { rotationAxis.SetAll(0); }
		else
		{
			rotationAxis.CopyFrom(rotationVector);
			rotationAxis *= (T)1. / rotationAngle;
		}
	}

	//! compute composition operation for rotation vectors v0 and incremental rotation vector Omega, see \cite{Holzinger2021}
	template<typename T>
	inline SlimVectorBase<T,3> CompositionRotationVector(const SlimVectorBase<T,3>& v0, const SlimVectorBase<T,3>& Omega)
	{
		T w1Half = (T)0.5 * v0.GetL2Norm();
		T w2Half = (T)0.5 * Omega.GetL2Norm();
		T c0 = cos(w1Half);
		T c1 = cos(w2Half);
		T s0 = EXUmath::Sinc(w1Half);
		T s1 = EXUmath::Sinc(w2Half);
		T x = c0 * c1 - (T)0.25 * s0 * s1 * (v0 * Omega);
		T xTemp = sqrt(fabs(1 - EXUstd::Square(x))); //fabs added, because term may be slightly smaller than zero
		T w = EXUstd::pi - (T)2. * atan2(x, xTemp);
		SlimVectorBase<T,3> rho = s0 * c1 * v0 + c0 * s1 * Omega + (T)0.5 * s0 * s1 * v0.CrossProduct(Omega);
		SlimVectorBase<T,3> n;
		RotationVector2RotationAxis(rho, n);
		return w * n;
	}

	//! compute tangent operator of R3xSO(3)
	template<typename T>
	inline ConstSizeMatrixBase<T, 36> TExpR3xSO3(const Vector6D& incrementalMotion)
	{
		// incremental position/rotation
		//SlimVectorBase<T,3> incrementalPosition = { incrementalMotion[0], incrementalMotion[1], incrementalMotion[2] };
		SlimVectorBase<T,3> incrementalRotation = { incrementalMotion[3], incrementalMotion[4], incrementalMotion[5] };

		ConstSizeMatrixBase<T, 36> Texp(6, 6);
		Texp.SetSubmatrix(EXUmath::Identity3D<T>, 0, 0);
		Texp.SetSubmatrix(EXUmath::Zero3D<T>, 0, 3);
		Texp.SetSubmatrix(EXUmath::Zero3D<T>, 3, 0);
		Texp.SetSubmatrix(TExpSO3(incrementalRotation), 3, 3);

		return Texp;
	}


	//! compute tangent operator of R3xSO(3)
	template<typename T>
	inline ConstSizeMatrixBase<T, 36> TExpR3xSO3Inv(const Vector6D& incrementalMotion)
	{
		// incremental position/rotation
		//SlimVectorBase<T,3> incrementalPosition = { incrementalMotion[0], incrementalMotion[1], incrementalMotion[2] };
		SlimVectorBase<T,3> incrementalRotation = { incrementalMotion[3], incrementalMotion[4], incrementalMotion[5] };

		ConstSizeMatrixBase<T, 36> TexpInv(6, 6);
		TexpInv.SetSubmatrix(EXUmath::Identity3D<T>, 0, 0);
		TexpInv.SetSubmatrix(EXUmath::Zero3D<T>, 0, 3);
		TexpInv.SetSubmatrix(EXUmath::Zero3D<T>, 3, 0);
		TexpInv.SetSubmatrix(TExpSO3Inv(incrementalRotation), 3, 3);

		return TexpInv;
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! compute homogeneous transformation from incremental displacement and incremental rotation
	//! compute the matrix exponential map on the Lie group SE(3), see \cite{Bruels2011}
	template<typename T>
	inline HomogeneousTransformationBase<T> ExpSE3(const SlimVectorBase<T,3>& incDisp, const SlimVectorBase<T,3>& incRot)
	{
		SlimVectorBase<T,3> x = incDisp * TExpSO3(incRot); //TExpSO3(incRot).T * incDisp
		return HomogeneousTransformationBase(ExpSO3(incRot), x);
	}

	//! matrix logarithm of SE3, returns HomogeneousTransformationBase
	template<typename T>
	inline HomogeneousTransformationBase<T> LogSE3(const HomogeneousTransformationBase<T>& H)
	{
		//ConstSizeMatrixBase<T, 9> aSkew = LogSO3(H.GetRotation());
		//SlimVectorBase<T,3> a = RigidBodyMath::SkewMatrix2Vector(aSkew);

		SlimVectorBase<T,3> a = LogSO3Vector(H.GetRotation());

		HomogeneousTransformationBase<T> logH;
		ConstSizeMatrixBase<T, 9> A = TExpSO3Inv(a).GetTransposed();
		logH.GetTranslation() = A * H.GetTranslation();
		logH.GetRotation() = RigidBodyMath::Vector2SkewMatrix(a);

		return logH;
	}

	//! matrix logarithm of SE3, returns incr. displacement and incr. rotation
	template<typename T>
	inline void LogSE3Vector(const HomogeneousTransformationBase<T>& H, SlimVectorBase<T,3>& incDisp, SlimVectorBase<T,3>& incRot)
	{
		//ConstSizeMatrixBase<T, 9> aSkew = LogSO3(H.GetRotation());
		//SlimVectorBase<T,3> a = RigidBodyMath::SkewMatrix2Vector(aSkew);

		incRot = LogSO3Vector(H.GetRotation());

		HomogeneousTransformationBase<T> logH;
		ConstSizeMatrixBase<T, 9> A = TExpSO3Inv(incRot).GetTransposed();
		incDisp = A * H.GetTranslation();
	}

	////! compute the tangent operator TExpSE3 corresponding to ExpSE3, see \cite{Bruels2011}
	//inline ConstSizeMatrixBase<T, 36> TExpSE3(const SlimVectorBase<T,3>& incDisp, const SlimVectorBase<T,3>& incRot)
	//{
	//	ConstSizeMatrixBase<T, 9> dispSkew = RigidBodyMath::Vector2SkewMatrix(incDisp);
	//	ConstSizeMatrixBase<T, 9> rotSkew = RigidBodyMath::Vector2SkewMatrix(incRot);

	//	T phi = incRot.GetL2Norm();
	//	T phiHalf = phi * 0.5;
	//	ConstSizeMatrixBase<T, 9> TDispRotPlus = (-0.5)*dispSkew;
	//	if (phi != 0.)
	//	{
	//		T phi2 = phi * phi;
	//		T a = (2 * sin(phiHalf)*cos(phiHalf)) / phi;
	//		T b = 4 * EXUstd::Square(sin(phiHalf)) / (phi2);
	//		TDispRotPlus += 0.5*((T)1. - b)*dispSkew;
	//		TDispRotPlus += (((T)1. - a) / (phi2))*(dispSkew*rotSkew + rotSkew*dispSkew); //could be optimized with transposed!
	//		TDispRotPlus += -(((a - b) / (phi2))*incRot*incDisp)* rotSkew;
	//		TDispRotPlus += (((T)1. / (phi2))*(0.5*b - (3. / (phi2))*((T)1. - a))*(incRot*incDisp))*(rotSkew*rotSkew);
	//	}
	//	ConstSizeMatrixBase<T, 9> MTexpSO3 = TExpSO3(incRot);
	//	ConstSizeMatrixBase<T, 36> Texp(6, 6);
	//	Texp.SetSubmatrix(MTexpSO3, 0, 0);
	//	Texp.SetSubmatrix(TDispRotPlus, 0, 3);
	//	Texp.SetSubmatrix(EXUmath::Zero3D<T>, 3, 0);
	//	Texp.SetSubmatrix(MTexpSO3, 3, 3);

	//	return Texp;
	//}


	//SH*
	//! compute the tangent operator TExpSE3 corresponding to ExpSE3, see Phd thesis Stefan Hante (Hante, S.: Geometric Integration of a Constrained Cosserat Beam Model.
	//! PhD thesis, Martin Luther University Halle - Wittenberg(2022). https://doi.org/10.25673/91397
	template<typename T>
	inline ConstSizeMatrixBase<T, 36> TExpSE3(const SlimVectorBase<T,3>& incDisp, const SlimVectorBase<T,3>& incRot)
	{
		ConstSizeMatrixBase<T, 9> dispSkew = RigidBodyMath::Vector2SkewMatrix(incDisp);
		ConstSizeMatrixBase<T, 9> rotSkew = RigidBodyMath::Vector2SkewMatrix(incRot);
		T phi = incRot.GetL2Norm();
		T phiSquared = EXUstd::Square(phi);
		T f2, f3, f4, f5;

		// coefficient f2, see Phd thesis Stefan Hante, Table. 1, page 121
		if (phi >= (T)1e-2)
		{
			f2 = (cos(phi) - 1) / phiSquared;
		}
		else
		{
			f2 = -(T)0.5 + ((T)1. / 24) * phiSquared - ((T)1. / 720) * phiSquared * phiSquared;
		}

		// coefficient f3, see Phd thesis Stefan Hante, Table. 1, page 121
		if (phi >= 1e-4)
		{
			f3 = (phi - sin(phi)) / (phi * phiSquared);
		}
		else
		{
			f3 = (T)1. / 6 - ((T)1. / 24) * phiSquared - ((T)1. / 720) * phiSquared * phiSquared;
		}

		// coefficient f4, see Phd thesis Stefan Hante, Table. 1, page 121
		if (phi >= 1e-1)
		{
			f4 = (2 - 2 * cos(phi) - phi * sin(phi)) / (phiSquared * phiSquared);
		}
		else
		{
			f4 = (T)1. / 12 - ((T)1. / 180) * phiSquared + ((T)1. / 6720) * phiSquared * phiSquared - ((T)1. / 453600) * phiSquared * phiSquared * phiSquared;
		}

		// coefficient f5, see Phd thesis Stefan Hante, Table. 1, page 121
		if (phi >= 1e-1)
		{
			f5 = (phi * (2 + cos(phi)) - 3 * sin(phi)) / (phi * phiSquared * phiSquared);
		}
		else
		{
			f5 = (T)1. / 60 - ((T)1. / 1260) * phiSquared + ((T)1. / 60480) * phiSquared * phiSquared - ((T)1. / 4989600) * phiSquared * phiSquared * phiSquared;
		}

		ConstSizeMatrixBase<T, 9> TDispRotPlus = f2 * dispSkew;
		TDispRotPlus += f3 * (dispSkew * rotSkew + rotSkew * dispSkew);
		TDispRotPlus += f4 * (incRot * incDisp) * rotSkew;
		TDispRotPlus -= f5 * (incRot * incDisp) * rotSkew * rotSkew;

		ConstSizeMatrixBase<T, 9> MTexpSO3 = TExpSO3(incRot);
		ConstSizeMatrixBase<T, 36> Texp(6, 6);
		Texp.SetSubmatrix(MTexpSO3, 0, 0);
		Texp.SetSubmatrix(TDispRotPlus, 0, 3);
		Texp.SetSubmatrix(EXUmath::Zero3D<T>, 3, 0);
		Texp.SetSubmatrix(MTexpSO3, 3, 3);

		return Texp;
	}


	////! compute the inverse of tangent operator TExpSE3, see \cite{Sonneville2014}
	//inline ConstSizeMatrixBase<T, 36> TExpSE3Inv(const SlimVectorBase<T,3>& incDisp, const SlimVectorBase<T,3>& incRot)
	//{
	//	T phi = incRot.GetL2Norm();
	//	ConstSizeMatrixBase<T, 9> Tuwm;
	//	if (phi == 0.)
	//	{
	//		Tuwm = 0.5*RigidBodyMath::Vector2SkewMatrix(incDisp);
	//	}
	//	else
	//	{
	//		T phi2 = phi * phi;
	//		T alpha = EXUmath::Sinc(phi);
	//		T beta = 2. * ((T)1. - cos(phi)) / (phi2);
	//		ConstSizeMatrixBase<T, 9> dispSkew = RigidBodyMath::Vector2SkewMatrix(incDisp);
	//		ConstSizeMatrixBase<T, 9> rotSkew = RigidBodyMath::Vector2SkewMatrix(incRot);
	//		
	//		Tuwm = 0.5*dispSkew;
	//		Tuwm += ((beta - alpha) / (beta*phi2))*(dispSkew*rotSkew + rotSkew*dispSkew);
	//		Tuwm += (((T)1. + alpha -2. * beta) / (beta*phi2*phi2))*(incRot*incDisp)*(rotSkew * rotSkew);
	//	}
	//	ConstSizeMatrixBase<T, 9> MTexpSO3Inv = TExpSO3Inv(incRot);
	//	ConstSizeMatrixBase<T, 36> Tinv(6,6);
	//	Tinv.SetSubmatrix(MTexpSO3Inv, 0, 0);
	//	Tinv.SetSubmatrix(Tuwm, 0, 3);
	//	Tinv.SetSubmatrix(EXUmath::Zero3D<T>, 3, 0);
	//	Tinv.SetSubmatrix(MTexpSO3Inv, 3, 3);

	//	return Tinv;
	//}

	//SH*
	//! compute the inverse of tangent operator TExpSE3, see Phd thesis Stefan Hante (Hante, S.: Geometric Integration of a Constrained Cosserat Beam Model.
	//! PhD thesis, Martin Luther University Halle - Wittenberg(2022). https://doi.org/10.25673/91397
	template<typename T>
	inline ConstSizeMatrixBase<T, 36> TExpSE3Inv(const SlimVectorBase<T,3>& incDisp, const SlimVectorBase<T,3>& incRot)
	{
		T phi = incRot.GetL2Norm();
		T phiSquared = EXUstd::Square(phi);
		T f6, f8;
		ConstSizeMatrixBase<T, 9> dispSkew = RigidBodyMath::Vector2SkewMatrix(incDisp);
		ConstSizeMatrixBase<T, 9> rotSkew = RigidBodyMath::Vector2SkewMatrix(incRot);

		// coefficient f6, see Phd thesis Stefan Hante, Table. 1, page 121
		if (phi >= 1e-2)
		{
			f6 = (2 - phi * EXUmath::Cot((T)0.5 * phi)) / (2 * phiSquared);
		}
		else
		{
			f6 = (T)1. / 12 + ((T)1. / 720) * phiSquared + ((T)1. / 30240) * EXUstd::Square(phiSquared);
		}

		// coefficient f8, see Phd thesis Stefan Hante, Table. 1, page 121
		if (phi >= 2e-1)
		{
			f8 = (phi * sin(phi) + 4 * cos(phi) + phiSquared - 4) / (4 * EXUstd::Square(sin(0.5 * phi)) * EXUstd::Square(phiSquared));
		}
		else
		{
			f8 = (T)1. / 360 + ((T)1. / 7560) * phiSquared + ((T)1. / 201600) * EXUstd::Square(phiSquared) + ((T)1. / 5987520) * phiSquared * phiSquared * phiSquared + ((T)691. / 130767436800) * phiSquared * phiSquared * phiSquared * phiSquared;
		}

		// Matrix C2, see Phd thesis Stefan Hante, Sect.A.3, page 117
		ConstSizeMatrixBase<T, 9> Tuwm = 0.5 * dispSkew + f6 * (dispSkew * rotSkew + rotSkew * dispSkew) + f8 * (incRot * incDisp) * (rotSkew * rotSkew);

		ConstSizeMatrixBase<T, 9> MTexpSO3Inv = TExpSO3Inv(incRot);
		ConstSizeMatrixBase<T, 36> Tinv(6, 6);
		Tinv.SetSubmatrix(MTexpSO3Inv, 0, 0);
		Tinv.SetSubmatrix(Tuwm, 0, 3);
		Tinv.SetSubmatrix(EXUmath::Zero3D<T>, 3, 0);
		Tinv.SetSubmatrix(MTexpSO3Inv, 3, 3);

		return Tinv;
	}

}; //namespace ExuLie

//! set a rotation and zero translation from rotation vector
template<typename T>
void HomogeneousTransformationBase<T>::SetRotation(const SlimVectorBase<T, 3>& rotation)
{
	A = EXUlie::ExpSO3(rotation);
	v = SlimVectorBase<T, 3>({ 0.,0.,0. });
}


typedef HomogeneousTransformationBase<Real> HomogeneousTransformation;
typedef HomogeneousTransformationBase<float> HomogeneousTransformationF;


//! get difference of *thi frame to HT1 as logarithm of relative homogeneous transformations
//! note that *this*ExpSE(incDisp, incRot) = *this * this->GetInverse() * HT1 = HT1
template<typename T>
inline void HomogeneousTransformationBase<T>::GetRelativeMotionTo(const HomogeneousTransformationBase<T>& HT1,
	SlimVectorBase<T,3>& incDisp, SlimVectorBase<T,3>& incRot)
{
	EXUlie::LogSE3Vector(GetInverse() * HT1, incDisp, incRot);
}




#endif
