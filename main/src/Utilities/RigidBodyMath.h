/** ***********************************************************************************************
* @brief		Functions and objects for rigid body kinematics and dynamics calculations;
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

#pragma once

#include "Linalg/BasicLinalg.h" 

namespace RigidBodyMath {

	//********************************************************************************
	//functions containing EULER PARAMETERS (QUATERNIONS)

	//! compute G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	template<class TVector>
	inline ConstSizeMatrix<12> EP2GTemplate(const TVector& ep)
	{
		return ConstSizeMatrix<12>(3, 4, {  -2.*ep[1], 2.*ep[0],-2.*ep[3], 2.*ep[2],
											-2.*ep[2], 2.*ep[3], 2.*ep[0],-2.*ep[1],
											-2.*ep[3],-2.*ep[2], 2.*ep[1], 2.*ep[0] });
	}

	//! compute time derivative of G-Matrix from Euler Parameters ep_t
	template<class TVector>
	inline ConstSizeMatrix<12> EP_t2G_tTemplate(const TVector& ep_t)
	{
		return ConstSizeMatrix<12>(3, 4, {	-2.*ep_t[1], 2.*ep_t[0],-2.*ep_t[3], 2.*ep_t[2],
											-2.*ep_t[2], 2.*ep_t[3], 2.*ep_t[0],-2.*ep_t[1],
											-2.*ep_t[3],-2.*ep_t[2], 2.*ep_t[1], 2.*ep_t[0] });
	}

	//! compute transposed G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	template<class TVector>
	inline ConstSizeMatrix<12> EP2GTTemplate(const TVector& ep)
	{
		return ConstSizeMatrix<12>(4, 3, {  -2.*ep[1],-2.*ep[2],-2.*ep[3],
											 2.*ep[0], 2.*ep[3],-2.*ep[2],
											-2.*ep[3], 2.*ep[0], 2.*ep[1],
											 2.*ep[2],-2.*ep[1], 2.*ep[0] });
	}

	//! compute local G-Matrix from Euler Parameters ep; Glocal is defined such that the angular velocity given in local
	//! coordinates omegaLocal follows from: omegaLocal = Glocal*ep_t
	template<class TVector>
	inline ConstSizeMatrix<12> EP2GlocalTemplate(const TVector& ep)
	{
		return ConstSizeMatrix<12>(3, 4, {  -2.*ep[1], 2.*ep[0], 2.*ep[3],-2.*ep[2],
											-2.*ep[2],-2.*ep[3], 2.*ep[0], 2.*ep[1],
											-2.*ep[3], 2.*ep[2],-2.*ep[1], 2.*ep[0] });
	}

	//! compute transposed Glocal-Matrix from Euler Parameters ep;
	template<class TVector>
	inline ConstSizeMatrix<12> EP2GlocalTTemplate(const TVector& ep)
	{
		return ConstSizeMatrix<12>(4, 3, { -2.*ep[1],-2.*ep[2],-2.*ep[3],
											 2.*ep[0],-2.*ep[3], 2.*ep[2],
											 2.*ep[3], 2.*ep[0],-2.*ep[1],
											-2.*ep[2], 2.*ep[1], 2.*ep[0] });
	}

	//! compute 3x3 skew(tilde)-matrix from vector v;
	template<class TVector>
	inline ConstSizeMatrix<9> Vector2SkewMatrixTemplate(const TVector& v)
	{
		CHECKandTHROW(v.NumberOfItems() == 3, "Vector2SkewMatrixTemplate: vector must contain 3 items!");
		return ConstSizeMatrix<9>(3, 3, { 0.,  -v[2], v[1],
										  v[2],    0,-v[0],
										 -v[1], v[0],    0 });
	}

	//! compute time derivative of local G-Matrix from time derivative of Euler Parameters: ep_t
	template<class TVector>
	inline ConstSizeMatrix<12> EP_t2Glocal_tTemplate(const TVector& ep_t)
	{
		return ConstSizeMatrix<12>(3, 4, { -2.*ep_t[1], 2.*ep_t[0], 2.*ep_t[3],-2.*ep_t[2],
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

	//! compute rotation matrix from Euler parameters ep0, ..., ep3
	inline Matrix3D EP2RotationMatrix(Real ep0, Real ep1, Real ep2, Real ep3)
	{
		return Matrix3D(3, 3, { -2.0*ep3*ep3 - 2.0*ep2*ep2 + 1.0, -2.0*ep3*ep0 + 2.0*ep2*ep1, 2.0*ep3*ep1 + 2.0*ep2*ep0,
							2.0*ep3*ep0 + 2.0*ep2*ep1, -2.0*ep3*ep3 - 2.0*ep1*ep1 + 1.0, 2.0*ep3*ep2 - 2.0*ep1*ep0,
							-2.0*ep2*ep0 + 2.0*ep3*ep1, 2.0*ep3*ep2 + 2.0*ep1*ep0, -2.0*ep2*ep2 - 2.0*ep1*ep1 + 1.0 });
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
	//Euler angle functions (xyz-rotations sequence, i.e., Rxyz = Rx*Ry*Rz
	
	//! convert Euler angles (Tait-Bryan angles) to rotation matrix
	template<class TVector>
	inline Matrix3D AngXYZ2RotationMatrixTemplate(const TVector& rot)
	{
		Real psi = rot[0];
		Real theta = rot[1];
		Real phi = rot[2];
		Real c0 = cos(rot[0]);
		Real s0 = sin(rot[0]);
		Real c1 = cos(rot[1]);
		Real s1 = sin(rot[1]);
		Real c2 = cos(rot[2]);
		Real s2 = sin(rot[2]);

		return Matrix3D(3,3,{ c1*c2,-c1 * s2,s1, s0*s1*c2 + c0 * s2,-s0 * s1*s2 + c0 * c2,-s0 * c1, -c0 * s1*c2 + s0 * s2,c0*s1*s2 + s0 * c2,c0*c1 });
	}
	inline Matrix3D AngXYZ2RotationMatrix(const Vector3D& rot) { return AngXYZ2RotationMatrixTemplate(rot); }
	inline Matrix3D AngXYZ2RotationMatrix(const CSVector3D& rot) { return AngXYZ2RotationMatrixTemplate(rot); }

	//! convert rotation matrix to Euler angles (Tait-Bryan angles)
	inline Vector3D RotationMatrix2AngXYZ(const Matrix3D& R)
	{
		Vector3D rot;
		rot[0] = atan2(-R(1, 2), R(2, 2));
		rot[1] = atan2(R(0, 2), sqrt(fabs(1. - R(0, 2) * R(0, 2)))); //fabs for safety, if small round up error in rotation matrix ...
		rot[2] = atan2(-R(0, 1), R(0, 0));
		return rot;
	}

	//********************************************************************************

	//! specializations for Euler parameter and related functions:
	inline ConstSizeMatrix<9> Vector2SkewMatrix(const Vector3D& v) { return Vector2SkewMatrixTemplate<Vector3D>(v); }
	inline ConstSizeMatrix<9> Vector2SkewMatrix(const CSVector3D& v) { return Vector2SkewMatrixTemplate<CSVector3D>(v); }

	//! compute G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	inline ConstSizeMatrix<12> EP2G(const Vector4D& ep) { return EP2GTemplate<Vector4D>(ep); }
	inline ConstSizeMatrix<12> EP2G(const CSVector4D& ep) { return EP2GTemplate<CSVector4D>(ep); }

	//! compute time derivative of G-Matrix from Euler Parameters ep_t
	inline ConstSizeMatrix<12> EP_t2G_t(const Vector4D& ep_t) { return EP_t2G_tTemplate<Vector4D>(ep_t); }
	inline ConstSizeMatrix<12> EP_t2G_t(const CSVector4D& ep_t) { return EP_t2G_tTemplate<CSVector4D>(ep_t); }

	//! compute transposed G-Matrix from Euler Parameters ep; G is defined such that the global angular velocity vector omega follows from: omega = G*ep_t
	inline ConstSizeMatrix<12> EP2GT(const Vector4D& ep) { return EP2GTTemplate<Vector4D>(ep); }
	inline ConstSizeMatrix<12> EP2GT(const CSVector4D& ep) { return EP2GTTemplate<CSVector4D>(ep); }

	//! compute local G-Matrix from Euler Parameters ep; Glocal is defined such that the angular velocity given in local
	//! coordinates omegaLocal follows from: omegaLocal = Glocal*ep_t
	inline ConstSizeMatrix<12> EP2Glocal(const Vector4D& ep) { return EP2GlocalTemplate<Vector4D>(ep); }
	inline ConstSizeMatrix<12> EP2Glocal(const CSVector4D& ep) { return EP2GlocalTemplate<CSVector4D>(ep); }

	//! compute transposed Glocal-Matrix from Euler Parameters ep;
	inline ConstSizeMatrix<12> EP2GlocalT(const Vector4D& ep) { return EP2GlocalTTemplate<Vector4D>(ep); }
	inline ConstSizeMatrix<12> EP2GlocalT(const CSVector4D& ep) { return EP2GlocalTTemplate<CSVector4D>(ep); }

	//! compute time derivative of local G-Matrix from time derivative of Euler Parameters: ep_t
	inline ConstSizeMatrix<12> EP_t2Glocal_t(const Vector4D& ep_t) { return EP_t2Glocal_tTemplate<Vector4D>(ep_t); }
	inline ConstSizeMatrix<12> EP_t2Glocal_t(const CSVector4D& ep_t) { return EP_t2Glocal_tTemplate<CSVector4D>(ep_t); }

	//! compute rotation matrix from 4-components vector of Euler parameters ep
	inline Matrix3D EP2RotationMatrix(const Vector4D& ep) { return EP2RotationMatrixTemplate(ep); }
	inline Matrix3D EP2RotationMatrix(const CSVector4D& ep) { return EP2RotationMatrixTemplate(ep); }

	//! compute time derivative of rotation matrix from 4-components vector of Euler parameters ep and 
	//! 4-components vector of time derivative of Euler parameters ep_t
	inline Matrix3D EP2RotationMatrix_t(const Vector4D& ep, const Vector4D& ep_t) { return EP2RotationMatrix_tTemplate<Vector4D>(ep, ep_t); }
	inline Matrix3D EP2RotationMatrix_t(const CSVector4D& ep, const CSVector4D& ep_t) { return EP2RotationMatrix_tTemplate<CSVector4D>(ep, ep_t); }


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





}



