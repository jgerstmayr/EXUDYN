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
		return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, {  -2.*ep[1], 2.*ep[0],-2.*ep[3], 2.*ep[2],
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
		return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, {  -2.*ep[1], 2.*ep[0], 2.*ep[3],-2.*ep[2],
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
		rot[0] = atan2(-R(1, 2), R(2, 2));
		rot[1] = atan2(R(0, 2), sqrt(fabs(1. - R(0, 2) * R(0, 2)))); //fabs for safety, if small round up error in rotation matrix ...
		rot[2] = atan2(-R(0, 1), R(0, 0));
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

namespace EXUlie {

	//! compute matrix exponential of 3D rotation vector rot; (rot is often denoted as Omega in literature ...)
	inline Matrix3D ExpSO3(const Vector3D& rot)
	{
		Real phi = rot.GetL2Norm();
		Matrix3D R(EXUmath::unitMatrix3D);
		Matrix3D rotSkew(RigidBodyMath::Vector2SkewMatrix(rot));
		R += EXUmath::Sinc(phi)*rotSkew + (0.5*EXUstd::Square(EXUmath::Sinc(0.5*phi)))*rotSkew * rotSkew;
		return R;
	}

	//! compute rotation axis from given rotation vector
	template <class T>
	inline void RotationVector2RotationAxis(const T& rotationVector, T& rotationAxis)
	{
		//compute rotation angle
		Real rotationAngle = rotationVector.GetL2Norm();

		if (rotationAngle == 0.) { rotationAxis.SetAll(0.); }
		else
		{
			rotationAxis.CopyFrom(rotationVector);
			rotationAxis *= 1. / rotationAngle;
		}
	}

	//! compute composition operation for rotation vectors v0 and incremental rotation vector Omega, see \cite{Holzinger2021}
	inline Vector3D CompositionRotationVector(const Vector3D& v0, const Vector3D& Omega)
	{
		Real w1Half = 0.5*v0.GetL2Norm();
		Real w2Half = 0.5*Omega.GetL2Norm();
		Real c0 = cos(w1Half);
		Real c1 = cos(w2Half);
		Real s0 = EXUmath::Sinc(w1Half);
		Real s1 = EXUmath::Sinc(w2Half);
		Real x = c0*c1 - 0.25*s0*s1*(v0 * Omega);
		Real xTemp = sqrt(fabs(1 - EXUstd::Square(x))); //fabs added, because term may be slightly smaller than zero
		Real w = EXUstd::pi - 2.*atan2(x, xTemp);
		Vector3D rho = s0*c1 * v0 + c0*s1 * Omega + 0.5*s0*s1* v0.CrossProduct(Omega);
		Vector3D n;
		RotationVector2RotationAxis(rho, n);
		return w * n;
	}

	//! compute the inverse of the tangent operator TExpSO3, see \cite{Sonneville2014}; improved version of Stefan Holzinger
	//! outputs 3x3 matrix
	inline Matrix3D TExpSO3Inv(const Vector3D& Omega)
	{
		Real phi = Omega.GetL2Norm();
		Matrix3D Tinv = EXUmath::unitMatrix3D;
		if (phi != 0.)
		{
			Real phi2 = EXUstd::Square(phi);
			Matrix3D omegaDiadicOmega(3,3);
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					omegaDiadicOmega(i, j) = Omega[i] * Omega[j];
				}
			}

			if (phi <= 0.02)
			{
				Real c = (1. / 12.) + (1. / 720.)*phi2 + (1. / 30240.)*EXUstd::Square(phi2);//  + (1. / 1209600.)*phi**6 # + (1. / 47900160.)*phi**8
				Real b = 1 - c * phi2;
				Tinv *= b; //Tinv = b*I

				Tinv += RigidBodyMath::Vector2SkewMatrix(0.5*Omega) + c * omegaDiadicOmega;
			}
			else
			{
				Real epsilon = 0.5*phi;
				Real beta = epsilon * EXUmath::Cot(epsilon);
				Real gamma = (1 - beta) / (phi2);
				Tinv *= beta;
				Tinv += RigidBodyMath::Vector2SkewMatrix(0.5*Omega) + gamma * omegaDiadicOmega;

			}
		}
		return Tinv;

	}

} //namespace LieGroup
#endif
