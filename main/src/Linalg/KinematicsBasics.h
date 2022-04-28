/** ***********************************************************************************************
* @brief		Functions and objects for extended rigid body kinematics: Homogeneous transformations (HT) and Pluecker transformations (T66);
*               namespace RigidBodyMath
*
* @author		Gerstmayr Johannes
* @date			2022-04-18 (generated)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* 
*
************************************************************************************************ */
#ifndef KINEMATICSBASICS__H
#define KINEMATICSBASICS__H

#include "Linalg/RigidBodyMath.h"

//! class representing (efficiently) homogeneous transformations
//! UNTESTED
class HomogeneousTransformation
{
private:
	Matrix3D A; //!< rotation part, [0:3,0:3] of matrix
	Vector3D v; //!< translation part, [3,0:3] of matrix
public:
	//! initialize with identity transformation
	HomogeneousTransformation(bool initialize = true)
	{
		A.SetScalarMatrix(3, 1.);
		v.SetAll(0.);
	}

	//! initialize with rotation matrix and translation
	HomogeneousTransformation(const Matrix3D& rotation, const Vector3D& translation): A(rotation), v(translation)
	{
	}

	//! return 4x4 HT matrix; this is less efficient and should only be used for output or at the end of computations
	Matrix4D GetHT44() const
	{
		Matrix4D HT(4,4);
		for (Index i = 0; i < 3; i++)
		{
			HT(3, i) = 0.;
			HT(i, 3) = v[i];
			for (Index j = 0; j < 3; j++)
			{
				HT(i, j) = A(i, j);
			}
		}
		HT(3, 3) = 1.;
		return HT;
	}

	//! use 4x4 HT matrix to set values of homogeneous transformation
	void SetHT44(const Matrix4D& HT)
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

	//! set a translation and identity rotation
	void SetTranslation(const Vector3D& translation)
	{
		A.SetScalarMatrix(3, 1.);
		v = translation;
	}

	//! set a translation along x-axis and identity rotation
	void SetTranslationX(Real x)
	{
		A.SetScalarMatrix(3, 1.);
		v = Vector3D({ x,0.,0. });
	}

	//! set a translation along y-axis and identity rotation
	void SetTranslationY(Real y)
	{
		A.SetScalarMatrix(3, 1.);
		v = Vector3D({ 0.,y,0. });
	}

	//! set a translation along z-axis and identity rotation
	void SetTranslationZ(Real z)
	{
		A.SetScalarMatrix(3, 1.);
		v = Vector3D({ 0.,0.,z });
	}

	//! set identity
	void SetIdentity()
	{
		A.SetScalarMatrix(3, 1.);
		v.SetAll(0.);
	}

	//! set a rotation around x-axis and zero translation
	void SetRotationX(Real angleRad)
	{
		v = Vector3D({ 0.,0.,0. });
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		A.SetMatrix(3, 3, {
			1.,0.,0.,
			0., c,-s,
			0., s, c });
	}

	//! set a rotation around y-axis and zero translation
	void SetRotationY(Real angleRad)
	{
		v = Vector3D({ 0.,0.,0. });
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		A.SetMatrix(3, 3, {
			c ,0., s,
			0.,1.,0.,
			-s,0., c });
	}

	//! set a rotation around z-axis and zero translation
	void SetRotationZ(Real angleRad)
	{
		v = Vector3D({ 0.,0.,0. });
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		A.SetMatrix(3, 3, {
			c ,-s,0.,
			s , c,0.,
			0.,0.,1. });
	}

	//! get translation part (read)
	const Vector3D& GetTranslation() const
	{
		return v;
	}

	//! get translation part (write)
	Vector3D& GetTranslation()
	{
		return v;
	}

	//! get rotation part (read)
	const Matrix3D& GetRotation() const
	{
		return A;
	}

	//! get rotation part (write)
	Matrix3D& GetRotation()
	{
		return A;
	}

	//! invert transfromation
	void Invert()
	{
		A.TransposeYourself();
		v = -(A * v);
	}

	//! return inverse homogeneous transformation
	HomogeneousTransformation GetInverse() const
	{
		HomogeneousTransformation HTinv;
		HTinv.GetRotation() = A.GetTransposed();
		HTinv.GetTranslation() = -(HTinv.GetRotation() * v);
	}


	//! comparison operator, component-wise compare; MATRIX DIMENSIONS MUST BE SAME; returns true, if all components are equal
	bool operator== (const HomogeneousTransformation& HT) const
	{
		return (HT.GetRotation() == A && HT.GetTranslation() == v);
	}

	//! multiplication of two homogeneous transformations
	friend HomogeneousTransformation operator* (const HomogeneousTransformation& HT1, const HomogeneousTransformation& HT2)
	{
		HomogeneousTransformation result;
		result.GetRotation() = HT1.GetRotation() * HT2.GetRotation();
		result.GetTranslation() = HT1.GetRotation() * HT2.GetTranslation() + HT1.GetTranslation();
		return result;
	}

	//! multiplication of homogeneous transformation with translation: [0r20 1] = [01T 0r10; 0 1] * [1r21 1]
	friend Vector3D operator* (const HomogeneousTransformation& HT1, const Vector3D& translation)
	{
		return HT1.GetRotation() * translation + HT1.GetTranslation();
	}

	HomogeneousTransformation& operator*= (const HomogeneousTransformation& other)
	{
		A = A * other.GetRotation();
		v += A * other.GetTranslation();
		return *this;
	}

};


namespace RigidBodyMath {
	//+++++++++++++++++++++++++++++++++++++++
	//(inefficient) T66 Pluecker transformations
	//follows Siciliano/Kathib Handbook of Robotics 2016, Chapter 3 (Featherstone) notion with some adaptations
	//most transformations noted by Featherstone are denoted as T66Inverse here!
	//6D vectors are v=[vRot, vTrans] with the angular velocity vRot and translation component vTrans, same for forces :[torque, force]
	//use these functions only for comparison, more efficient class will be used for fast operations

	//! compute Pluecker transformation T66 from rotation around X axis
	inline Matrix6D RotationX2T66(Real angleRad)
	{
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		Matrix6D A(false);
		A.SetMatrix(6, 6, {
			1.,0.,0.,0.,0.,0.,
			0., c,-s,0.,0.,0.,
			0., s, c,0.,0.,0.,
			0.,0.,0.,1.,0.,0.,
			0.,0.,0.,0., c,-s,
			0.,0.,0.,0., s, c
			});
		return A;
	}

	//! compute Pluecker transformation T66 from rotation around Y axis
	inline Matrix6D RotationY2T66(Real angleRad)
	{
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		Matrix6D A(false);
		A.SetMatrix(6, 6, {
			c ,0., s,0.,0.,0.,
			0.,1.,0.,0.,0.,0.,
			-s,0., c,0.,0.,0.,
			0.,0.,0.,c ,0., s,
			0.,0.,0.,0.,1.,0.,
			0.,0.,0.,-s,0., c
			});
		return A;
	}

	//! compute Pluecker transformation T66 from rotation around Z axis
	inline Matrix6D RotationZ2T66(Real angleRad)
	{
		Real c = cos(angleRad);
		Real s = sin(angleRad);
		Matrix6D A(false);
		A.SetMatrix(6, 6, {
			c ,-s,0.,0.,0.,0.,
			s , c,0.,0.,0.,0.,
			0.,0.,1.,0.,0.,0.,
			0.,0.,0., c,-s,0.,
			0.,0.,0., s, c,0.,
			0.,0.,0.,0.,0.,1.
			});
		return A;
	}

	//! compute Pluecker identity transformation T66 
	inline Matrix6D IdentityT66(Real angleRad)
	{
		Matrix6D A(false);
		A.SetScalarMatrix(6, 1.);
		return A;
	}

	//! compute Pluecker transformation T66 from translation vector
	inline Matrix6D RotationZ2T66(const Vector3D& t)
	{
		Matrix6D A(false);
		A.SetMatrix(6, 6, {
			1.,   0.,   0.,   0.,0.,0.,
			0.,   1.,   0.,   0.,0.,0.,
			0.,   0.,   1.,   0.,0.,0.,
			0.,   t[2], -t[1],1.,0.,0.,
			-t[2],0.,   t[0], 0.,1.,0.,
			t[1], -t[0],0.,   0.,0.,1.
			});
	}

	//! convert Pluecker (motion) transformation T66 into rotation matrix A and translation vector v
	inline void T66toRotationTranslation(const Matrix6D& T66, Matrix3D A, Vector3D& v)
	{
		Matrix3D vSkew;
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				A(i, j) = T66(i, j);
				vSkew(i, j) = T66(i+3, j);
			}
		}
		vSkew = vSkew*A.GetTransposed();
		v = RigidBodyMath::SkewMatrix2Vector(vSkew);
	}

	//! convert Pluecker (motion) transformation T66 into rotation matrix A and translation vector v
	inline Matrix6D RotationTranslation2T66(const Matrix3D A, const Vector3D& v)
	{
		Matrix6D T66(6,6);
		Matrix3D vSkewA = RigidBodyMath::Vector2SkewMatrix(v)*A;
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				T66(i, j) = A(i, j);
				T66(i+3, j+3) = A(i, j);
				T66(i, j+3) = 0.;
				T66(i+3, j) = vSkewA(i,j);
			}
		}
		return T66;
	}

	//! compute inverse of 6x6 Pluecker transform (motion); not very efficient!
	inline Matrix6D T66MotionInverse(const Matrix6D& T66)
	{
		Matrix3D AT;
		Vector3D v;
		T66toRotationTranslation(T66, AT, v);
		AT.TransposeYourself();
		return RotationTranslation2T66(AT, -(AT*v));
	}

	//! compute homogeneous transformation from Pluecker transformation
	inline HomogeneousTransformation T66toHT(const Matrix6D& T66)
	{
		Matrix3D A;
		Vector3D v;
		T66toRotationTranslation(T66, A, v);

		return HomogeneousTransformation(A, v);
	}

	//! compute inertia parameters in T66 form from parameters (at center of mass!)
	inline Matrix6D InertiaT66FromInertiaParameters(Real mass, const Vector3D& centerOfMass, Matrix3D& inertiaCOM)
	{
		Matrix3D skewCOM = RigidBodyMath::Vector2SkewMatrix(centerOfMass);
		Matrix3D massCOMCOM = (-mass) * skewCOM * skewCOM; //minus represents transposed
		Matrix6D A(6,6);

		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				A(i, j) = inertiaCOM(i, j) + massCOMCOM(i, j);
				A(i + 3, j + 3) = 0.;
				A(i + 3, j) = mass*skewCOM(i,j);
				A(i, j + 3) = mass*skewCOM(j,i); //transposed
			}
			A(i + 3, i + 3) = mass;
		}
	}

	//! compute skew matrix for T66 motion vectors v=[vRot, vTrans]
	inline Matrix6D T66SkewMotion(const Vector6D v)
	{
		return Matrix6D(6, 6, {
			   0.,-v[2], v[1],   0.,   0.,   0.,
			 v[2],   0.,-v[0],   0.,   0.,   0.,
			-v[1], v[0],   0.,   0.,   0.,   0.,
			   0.,-v[5], v[4],   0.,-v[2], v[1],
			 v[5],   0.,-v[3], v[2],   0.,-v[0],
			-v[4], v[3],   0.,-v[1], v[0],   0. });
	}

	//! compute skew matrix for T66 force vectors v=[vRot, vTrans]
	inline Matrix6D T66SkewForce(const Vector6D v)
	{
		//return T66SkewMotion(-v).GetTransposed();
		return Matrix6D(6, 6, {
			   0.,-v[2], v[1],   0.,-v[5], v[4],
			 v[2],   0.,-v[0], v[5],   0.,-v[3],
			-v[1], v[0],   0.,-v[4], v[3],   0.,
			   0.,   0.,   0.,   0.,-v[2], v[1],
			   0.,   0.,   0., v[2],   0.,-v[0],
			   0.,   0.,   0.,-v[1], v[0],   0. });
	}

} //namespace RigidBodyMath


#endif
