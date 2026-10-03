/** ***********************************************************************************************
* @brief		Functions and objects for extended rigid body dynamics (KinematicTree): Homogeneous transformations (HT), used as the spatial (Pluecker) transformations of Featherstone (T66), and 6D motion and force vectors;
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


//! the transformations of the kinematic tree are homogeneous transformations; the 6x6 Pluecker matrices of the
//! first implementation were removed (#2828)
typedef HomogeneousTransformation Transformation66;

namespace RigidBodyMath {

	//! class to store inertia parameters (used w.r.t. reference point != center of mass)
	class InertiaAtRefPoint
	{
	private:
		Matrix3D inertiaTensorAtRefPoint;
		Vector3D massCOM;
		Real mass;
	public:

		//! replace default constructor
		InertiaAtRefPoint(): inertiaTensorAtRefPoint(Matrix3D(3,3,0.)), massCOM(Vector3D(0.)), mass(0.) {}

		//! initialize inertia class with parameters at COM !
		void SetWithInertiaAtCOM(Real massInit, const Vector3D& comInit, const Matrix3D& inertiaTensorCOMinit)
		{
			//Matrix3D skewCOM(RigidBodyMath::Vector2SkewMatrix(comInit));
			Matrix3D skewCOM(Vector2SkewMatrix(comInit));
			inertiaTensorAtRefPoint = inertiaTensorCOMinit;
			inertiaTensorAtRefPoint -= massInit * skewCOM*skewCOM;
			massCOM = massInit*comInit;
			mass = massInit;
		}

		//! initialize inertia class with parameters at COM !
		void SetWithInertiaAtRefPoint(Real massInit, const Vector3D& comInit, const Matrix3D& inertiaTensorAtRefPointInit)
		{
			inertiaTensorAtRefPoint = inertiaTensorAtRefPointInit;
			massCOM = massInit*comInit;
			mass = massInit;
		}

		Matrix3D GetInertiaTensorCOM() const 
		{ 
			//Matrix3D skewMassCOM(RigidBodyMath::Vector2SkewMatrix(massCOM));
			Matrix3D skewMassCOM(Vector2SkewMatrix(massCOM));
			return inertiaTensorAtRefPoint + (1./mass) * skewMassCOM*skewMassCOM; //transposed replaces minus (-)
		}
		Vector3D GetCOM() const { return (1./mass)*massCOM; }

		const Matrix3D& GetInertiaTensorAtRefPoint() const { return inertiaTensorAtRefPoint; }
		const Vector3D& GetMassCOM() const { return massCOM; }
		const Real& GetMass() const { return mass; }

		Matrix3D& GetInertiaTensorAtRefPoint() { return inertiaTensorAtRefPoint; }
		Vector3D& GetMassCOM() { return massCOM; }
		Real& GetMass() { return mass; }

		//! add two inertias with same reference point!
		InertiaAtRefPoint& operator+= (const InertiaAtRefPoint& other)
		{
			mass += other.mass;
			massCOM += other.massCOM;
			inertiaTensorAtRefPoint += other.inertiaTensorAtRefPoint;

			return *this;
		}

	};

	//! convert Pluecker vector into omega and v
	inline void Vector6DtoVector3D(const Vector6D& v6D, Vector3D& v0, Vector3D& v1)
	{
		v0.SetVector({ v6D.GetUnsafe(0), v6D.GetUnsafe(1), v6D.GetUnsafe(2) });
		v1.SetVector({ v6D.GetUnsafe(3), v6D.GetUnsafe(4), v6D.GetUnsafe(5) });
	}


	//! compute Pluecker transformation T66 from rotation around X axis
	inline HomogeneousTransformation RotationX2T66(Real angleRad)
	{
		HomogeneousTransformation A(false);
		A.SetRotationX(angleRad);
		return A;
	}

	//! compute Pluecker transformation T66 from rotation around Y axis
	inline HomogeneousTransformation RotationY2T66(Real angleRad)
	{
		HomogeneousTransformation A(false);
		A.SetRotationY(angleRad);
		return A;
	}

	//! compute Pluecker transformation T66 from rotation around Z axis
	inline HomogeneousTransformation RotationZ2T66(Real angleRad)
	{
		HomogeneousTransformation A(false);
		A.SetRotationZ(angleRad);
		return A;
	}

	//! compute Pluecker identity transformation T66 
	inline HomogeneousTransformation IdentityT66(Real angleRad)
	{
		HomogeneousTransformation A(false);
		A.SetIdentity();
		return A;
	}

	//! compute Pluecker transformation T66 from translation vector
	inline HomogeneousTransformation Translation2T66(const Vector3D& t)
	{
		HomogeneousTransformation A(false);
		A.SetTranslation(t);
		return A;
	}

	//! convert Pluecker (motion) transformation T66 into rotation matrix A and translation vector v
	inline void T66toRotationTranslation(const HomogeneousTransformation& T66, Matrix3D& A, Vector3D& v)
	{
		A = T66.GetRotation();
		v = T66.GetTranslation();
	}

	//! convert Pluecker (motion) transformation T66 inverse into rotation matrix A and translation vector v
	inline void T66toRotationTranslationInverse(const HomogeneousTransformation& T66, Matrix3D& A, Vector3D& v)
	{
		//R.T, -R*p
		A = T66.GetRotation();
		A.TransposeYourself();
		v = -(A*T66.GetTranslation());
	}

	//! convert rotation matrix A and translation vector v into Pluecker (motion) transformation T66 
	inline HomogeneousTransformation RotationTranslation2T66(const Matrix3D& A, const Vector3D& v)
	{
		return HomogeneousTransformation(A, v);
	}

	//! convert rotation matrix A and translation vector v into inverse Pluecker (motion) transformation T66 
	inline HomogeneousTransformation RotationTranslation2T66Inverse(const Matrix3D& A, const Vector3D& v)
	{
		return HomogeneousTransformation(A.GetTransposed(), -(v*A)); //A.T*v
	}

	//! compute homogeneous transformation from Pluecker transformation
	inline HomogeneousTransformation T66toHT(const HomogeneousTransformation& T66)
	{
		return T66;
	}

	//! compute inertia parameters in T66 form from parameters (at center of mass!)
	inline InertiaAtRefPoint InertiaT66FromInertiaParameters(Real mass, 
		const Vector3D& centerOfMass, const Matrix3D& inertiaCOM)
	{
		InertiaAtRefPoint inertia;
		inertia.SetWithInertiaAtCOM(mass, centerOfMass, inertiaCOM);
		return inertia;
	}

	//! compute v x w for motion
	inline Vector6D MultT66SkewMotion(const Vector6D& v, const Vector6D& w)
	{
		//ret[0:3] = v[0:3] x w[0:3] 
		//ret[3:6] = v[3:6] x w[0:3] + v[0:3] x w[3:6]
		return Vector6D({ 
			v[1] * w[2] - v[2] * w[1],
			v[2] * w[0] - v[0] * w[2],
			v[0] * w[1] - v[1] * w[0],
			v[1 + 3] * w[2] - v[2 + 3] * w[1] + v[1] * w[2 + 3] - v[2] * w[1 + 3],
			v[2 + 3] * w[0] - v[0 + 3] * w[2] + v[2] * w[0 + 3] - v[0] * w[2 + 3],
			v[0 + 3] * w[1] - v[1 + 3] * w[0] + v[0] * w[1 + 3] - v[1] * w[0 + 3],
			});
	}

	//! compute v x w for force
	inline Vector6D MultT66SkewForce(const Vector6D& v, const Vector6D& w)
	{
		//ret[0:3] = v[0:3] x w[0:3] + v[3:6] x w[3:6]
		//ret[3:6] = v[0:3] x w[3:6]
		return Vector6D({
			v[1] * w[2] - v[2] * w[1] + v[1 + 3] * w[2 + 3] - v[2 + 3] * w[1 + 3],
			v[2] * w[0] - v[0] * w[2] + v[2 + 3] * w[0 + 3] - v[0 + 3] * w[2 + 3],
			v[0] * w[1] - v[1] * w[0] + v[0 + 3] * w[1 + 3] - v[1 + 3] * w[0 + 3],
			v[1] * w[2 + 3] - v[2] * w[1 + 3],
			v[2] * w[0 + 3] - v[0] * w[2 + 3],
			v[0] * w[1 + 3] - v[1] * w[0 + 3],
			});
	}

	//! multiply with velocity vector v=[vRot, vTrans]
	//! NOTE: while in Siciliano/Khatib 2016, p.44, the meaning of p is -ApB, here it is BpA; so ApB = -RT*BpA
	inline Vector6D T66Mult(const HomogeneousTransformation& T66, const Vector6D& velocity)
	{
		Vector6D result;
		LinkedDataVector resultOmega(result, 0, 3);
		LinkedDataVector resultV(result, 3, 3);
		
		Vector3D omega;
		Vector3D vO;
		Vector6DtoVector3D(velocity, omega, vO);

		//EXUmath::MultMatrixVectorTemplate(T66.GetRotation(), omega, resultOmega);
		resultOmega = T66.GetRotation() * omega;
		resultV = T66.GetRotation()*vO + T66.GetTranslation().CrossProduct(T66.GetRotation()*omega);


		return result;
	}

	//! multiply transposed T66 matrix with force vector v=[vRot, vTrans]
	inline Vector6D T66MultTransposed(const HomogeneousTransformation& T66, const Vector6D& force)
	{
		Vector6D result;
		LinkedDataVector resultOmega(result, 0, 3);
		LinkedDataVector resultV(result, 3, 3);

		Vector3D nO;
		Vector3D f;
		Vector6DtoVector3D(force, nO, f);

		//as compared to compact formulas in Siciliano p. 44, T66 represents the transposed structure (A.T, -p)
		EXUmath::MultMatrixTransposedVectorTemplate(T66.GetRotation(), nO - T66.GetTranslation().CrossProduct(f), resultOmega);
		EXUmath::MultMatrixTransposedVectorTemplate(T66.GetRotation(), f, resultV);

		return result;
	}

	//! multiply inverse of transposed T66 with force vector v
	inline Vector6D T66MultTransposedInverse(const HomogeneousTransformation& T66, const Vector6D& force)
	{
		//as compared to compact formulas in Siciliano p. 44, T66 represents the transposed structure (A.T, -p)

		Vector6D result;
		LinkedDataVector resultOmega(result, 0, 3);
		LinkedDataVector resultV(result, 3, 3);

		Vector3D nO;
		Vector3D f;
		Vector6DtoVector3D(force, nO, f);

		resultOmega = T66.GetTranslation().CrossProduct(T66.GetRotation()*f);
		resultOmega += T66.GetRotation() * nO;
		resultV = T66.GetRotation() * f;

		return result;
	}

	//! multiply inertia with vector v=[vRot, vTrans]
	inline Vector6D T66MultInertia(const InertiaAtRefPoint& inertia, const Vector6D& v)
	{
		Vector6D result;
		LinkedDataVector resultOmega(result, 0, 3);
		LinkedDataVector resultV(result, 3, 3);

		Vector3D omega;
		Vector3D vO;
		Vector6DtoVector3D(v, omega, vO);

		resultOmega.CopyFrom(inertia.GetInertiaTensorAtRefPoint()*omega + inertia.GetMassCOM().CrossProduct(vO));
		resultV.CopyFrom(inertia.GetMass()*vO - inertia.GetMassCOM().CrossProduct(omega));
		return result;
	}

	//! multiply with vector v=[vRot, vTrans]
	inline InertiaAtRefPoint T66TransformInertia(const HomogeneousTransformation& T66, const InertiaAtRefPoint& I66)
	{
		Matrix3D RT = T66.GetRotation();
		RT.TransposeYourself();

		//Real m = I66.GetMass();
		//Vector3D RTh = RT*(I66.GetMassCOM());
		//Matrix3D skewP = RigidBodyMath::Vector2SkewMatrix(RT*T66.GetTranslation());
		//Vector3D RThmp = RTh + m * (RT*T66.GetTranslation());

		//InertiaAtRefPoint inertia;
		//inertia.GetMass() = m;
		//inertia.GetMassCOM() = RTh - m * (RT*T66.GetTranslation());
		//inertia.GetInertiaTensorAtRefPoint() = RT * I66.GetInertiaTensorAtRefPoint()*T66.GetRotation() 
		//	- skewP * RigidBodyMath::Vector2SkewMatrix(RTh) - RigidBodyMath::Vector2SkewMatrix(RThmp)*skewP;
		//return inertia;

		Real m = I66.GetMass();
		Vector3D RTh = RT * (I66.GetMassCOM());

		InertiaAtRefPoint inertia;
		inertia.GetMass() = m;
		inertia.GetMassCOM() = RTh - m * (RT*T66.GetTranslation());
		//Matrix3D skewMassCOM = RigidBodyMath::Vector2SkewMatrix(inertia.GetMassCOM());
		Matrix3D skewMassCOM = Vector2SkewMatrix(inertia.GetMassCOM());
		inertia.GetInertiaTensorAtRefPoint() = RT * I66.GetInertiaTensorCOM()*T66.GetRotation()
			- (1. / m) * skewMassCOM*skewMassCOM;
		return inertia;

	}


} //namespace RigidBodyMath




typedef ResizableArray<Transformation66> Transformation66List;
typedef ResizableArray<RigidBodyMath::InertiaAtRefPoint> InertiaList;
typedef ResizableArray<Matrix6D> Matrix6DList;


#endif
