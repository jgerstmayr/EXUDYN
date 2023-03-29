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


#define USE_EFFICIENT_TRANSFORMATION66


//! class representing (efficiently) homogeneous transformations; 
//! follows widely the Python implementation in exudyn.rigidBodyUtilities and lieGroupBasics
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
		return HTinv;
	}

	//! convert skew matrix representation (as returned e.g. by LogSE3) to inremental rotation and displacement
	void Skew2Vector(Vector3D& incDisp, Vector3D& incRot)
	{
		incRot[0] = A(2,1);
		incRot[1] = A(0,2);
		incRot[2] = A(1,0); //Python uses -A(0,1)
		incDisp[0] = v[0];
		incDisp[1] = v[1];
		incDisp[2] = v[2];
	}

	//! get difference of *thi frame to HT1 as logarithm of relative homogeneous transformations
	//! note that *this*ExpSE(incDisp, incRot) = *this * this->GetInverse() * HT1 = HT1
	void GetRelativeMotionTo(const HomogeneousTransformation& HT1,
		Vector3D& incDisp, Vector3D& incRot);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

	friend std::ostream& operator<<(std::ostream& os, const HomogeneousTransformation& HT)
	{
		os << "[" << HT.GetRotation() << ", " << HT.GetTranslation() << "]";
		return os;
	}


};

namespace EXUlie {

	//! compute homogeneous transformation from incremental displacemnet and incremental rotation
	//! compute the matrix exponential map on the Lie group SE(3), see \cite{Bruels2011}
	inline HomogeneousTransformation ExpSE3(const Vector3D& incDisp, const Vector3D& incRot)
	{
		Vector3D x = incDisp * TExpSO3(incRot); //TExpSO3(incRot).T * incDisp
		return HomogeneousTransformation(ExpSO3(incRot), x);
	}

	//! matrix logarithm of SE3, returns HomogeneousTransformation
	inline HomogeneousTransformation LogSE3(const HomogeneousTransformation& H)
	{
		//Matrix3D aSkew = LogSO3(H.GetRotation());
		//Vector3D a = RigidBodyMath::SkewMatrix2Vector(aSkew);

		Vector3D a = LogSO3Vector(H.GetRotation());

		HomogeneousTransformation logH;
		Matrix3D A = TExpSO3Inv(a).GetTransposed();
		logH.GetTranslation() = A * H.GetTranslation();
		logH.GetRotation() = RigidBodyMath::Vector2SkewMatrix(a);

		return logH;
	}

	//! matrix logarithm of SE3, returns incr. displacement and incr. rotation
	inline void LogSE3Vector(const HomogeneousTransformation& H, Vector3D& incDisp, Vector3D& incRot)
	{
		//Matrix3D aSkew = LogSO3(H.GetRotation());
		//Vector3D a = RigidBodyMath::SkewMatrix2Vector(aSkew);

		incRot = LogSO3Vector(H.GetRotation());

		HomogeneousTransformation logH;
		Matrix3D A = TExpSO3Inv(incRot).GetTransposed();
		incDisp = A * H.GetTranslation();
	}

	//! compute the tangent operator TExpSE3 corresponding to ExpSE3, see \cite{Bruels2011}
	inline Matrix6D TExpSE3(const Vector3D& incDisp, const Vector3D& incRot)
	{
		Matrix3D dispSkew = RigidBodyMath::Vector2SkewMatrix(incDisp);
		Matrix3D rotSkew = RigidBodyMath::Vector2SkewMatrix(incRot);

		Real phi = incRot.GetL2Norm();
		Real phiHalf = phi * 0.5;
		Matrix3D TDispRotPlus = (-0.5)*dispSkew;
		if (phi != 0.)
		{
			Real phi2 = phi * phi;
			Real a = (2 * sin(phiHalf)*cos(phiHalf)) / phi;
			Real b = 4 * EXUstd::Square(sin(phiHalf)) / (phi2);
			TDispRotPlus += 0.5*(1. - b)*dispSkew;
			TDispRotPlus += ((1. - a) / (phi2))*(dispSkew*rotSkew + rotSkew*dispSkew); //could be optimized with transposed!
			TDispRotPlus += -(((a - b) / (phi2))*incRot*incDisp)* rotSkew;
			TDispRotPlus += ((1. / (phi2))*(0.5*b - (3. / (phi2))*(1. - a))*(incRot*incDisp))*(rotSkew*rotSkew);
		}
		Matrix3D MTexpSO3 = TExpSO3(incRot);
		Matrix6D Texp(6, 6);
		Texp.SetSubmatrix(MTexpSO3, 0, 0);
		Texp.SetSubmatrix(TDispRotPlus, 0, 3);
		Texp.SetSubmatrix(EXUmath::zeroMatrix3D, 3, 0);
		Texp.SetSubmatrix(MTexpSO3, 3, 3);

		return Texp;
	}


	//! compute the inverse of tangent operator TExpSE3, see \cite{Sonneville2014}
	inline Matrix6D TExpSE3Inv(const Vector3D& incDisp, const Vector3D& incRot)
	{
		Real phi = incRot.GetL2Norm();
		Matrix3D Tuwm;
		if (phi == 0.)
		{
			Tuwm = 0.5*RigidBodyMath::Vector2SkewMatrix(incDisp);
		}
		else
		{
			Real phi2 = phi * phi;
			Real alpha = EXUmath::Sinc(phi);
			Real beta = 2. * (1. - cos(phi)) / (phi2);
			Matrix3D dispSkew = RigidBodyMath::Vector2SkewMatrix(incDisp);
			Matrix3D rotSkew = RigidBodyMath::Vector2SkewMatrix(incRot);
			
			Tuwm = 0.5*dispSkew;
			Tuwm += ((beta - alpha) / (beta*phi2))*(dispSkew*rotSkew + rotSkew*dispSkew);
			Tuwm += ((1. + alpha -2. * beta) / (beta*phi2*phi2))*(incRot*incDisp)*(rotSkew * rotSkew);
		}
		Matrix3D MTexpSO3Inv = TExpSO3Inv(incRot);
		Matrix6D Tinv(6,6);
		Tinv.SetSubmatrix(MTexpSO3Inv, 0, 0);
		Tinv.SetSubmatrix(Tuwm, 0, 3);
		Tinv.SetSubmatrix(EXUmath::zeroMatrix3D, 3, 0);
		Tinv.SetSubmatrix(MTexpSO3Inv, 3, 3);

		return Tinv;
	}
};

//! get difference of *thi frame to HT1 as logarithm of relative homogeneous transformations
//! note that *this*ExpSE(incDisp, incRot) = *this * this->GetInverse() * HT1 = HT1
inline void HomogeneousTransformation::GetRelativeMotionTo(const HomogeneousTransformation& HT1,
	Vector3D& incDisp, Vector3D& incRot)
{
	EXUlie::LogSE3Vector(GetInverse() * HT1, incDisp, incRot);
}

#ifndef USE_EFFICIENT_TRANSFORMATION66
	typedef Matrix6D Transformation66;
#else
	typedef HomogeneousTransformation Transformation66;
#endif


#ifndef USE_EFFICIENT_TRANSFORMATION66
	namespace RigidBodyMath {
		//+++++++++++++++++++++++++++++++++++++++
		//(inefficient) T66 Pluecker transformations
		//follows Siciliano/Kathib Handbook of Robotics 2016, Chapter 3 (Featherstone) notion with some adaptations
		//most transformations noted by Featherstone are denoted as T66Inverse here!
		//6D vectors are v=[vRot, vTrans] with the angular velocity vRot and translation component vTrans, same for forces :[torque, force]
		//use these functions only for comparison, more efficient class will be used for fast operations

		typedef Matrix6D InertiaAtRefPoint;

		//! compute Pluecker transformation T66 from rotation around X axis
		inline Transformation66 RotationX2T66(Real angleRad)
		{
			Real c = cos(angleRad);
			Real s = sin(angleRad);
			Transformation66 A(false);
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
		inline Transformation66 RotationY2T66(Real angleRad)
		{
			Real c = cos(angleRad);
			Real s = sin(angleRad);
			Transformation66 A(false);
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
		inline Transformation66 RotationZ2T66(Real angleRad)
		{
			Real c = cos(angleRad);
			Real s = sin(angleRad);
			Transformation66 A(false);
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
		inline Transformation66 IdentityT66(Real angleRad)
		{
			Transformation66 A(false);
			A.SetScalarMatrix(6, 1.);
			return A;
		}

		//! compute Pluecker transformation T66 from translation vector
		inline Transformation66 Translation2T66(const Vector3D& t)
		{
			Transformation66 A(false);
			A.SetMatrix(6, 6, {
				1.,   0.,   0.,   0.,0.,0.,
				0.,   1.,   0.,   0.,0.,0.,
				0.,   0.,   1.,   0.,0.,0.,
				0.,   t[2], -t[1],1.,0.,0.,
				-t[2],0.,   t[0], 0.,1.,0.,
				t[1], -t[0],0.,   0.,0.,1.
				});
			return A;
		}

		//! convert Pluecker (motion) transformation T66 into rotation matrix A and translation vector v
		inline void T66toRotationTranslation(const Transformation66& T66, Matrix3D& A, Vector3D& v)
		{
			Matrix3D vSkew(3, 3);
			A.SetNumberOfRowsAndColumns(3, 3);
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					A(i, j) = T66(i, j);
					vSkew(i, j) = T66(i + 3, j);
				}
			}
			vSkew = vSkew * A.GetTransposed();
			v = SkewMatrix2Vector(vSkew);
		}

		//! convert Pluecker (motion) transformation T66 inverse into rotation matrix A and translation vector v
		inline void T66toRotationTranslationInverse(const Transformation66& T66, Matrix3D& A, Vector3D& v)
		{
			//(AT, -(AT*v))
			Matrix3D vSkew(3, 3);
			A.SetNumberOfRowsAndColumns(3, 3);
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					A(i, j) = T66(j, i);
					vSkew(i, j) = T66(i + 3, j);
				}
			}
			vSkew = vSkew * A;
			v = -(A*SkewMatrix2Vector(vSkew));
		}

		//! convert rotation matrix A and translation vector v into Pluecker (motion) transformation T66 
		inline Transformation66 RotationTranslation2T66(const Matrix3D& A, const Vector3D& v)
		{
			Transformation66 T66(6, 6);
			Matrix3D vSkewA = RigidBodyMath::Vector2SkewMatrix(v)*A;
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					T66(i, j) = A(i, j);
					T66(i + 3, j + 3) = A(i, j);
					T66(i, j + 3) = 0.;
					T66(i + 3, j) = vSkewA(i, j);
				}
			}
			return T66;
		}

		//! convert rotation matrix A and translation vector v into inverse Pluecker (motion) transformation T66 
		inline Transformation66 RotationTranslation2T66Inverse(const Matrix3D& A, const Vector3D& v)
		{
			Transformation66 T66(6, 6);
			Matrix3D vSkewA = RigidBodyMath::Vector2SkewMatrix(v)*A;
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					T66(i, j) = A(j, i); //A.T
					T66(i + 3, j + 3) = A(j, i); //A.T
					T66(i, j + 3) = 0.;
					T66(i + 3, j) = vSkewA(j, i); //-A.T*Skew(v)  = Skew(v)*A
				}
			}
			return T66;
		}

		//! compute inverse of 6x6 Pluecker transform (motion); not very efficient, but used only for testing!
		inline Transformation66 T66MotionInverse(const Transformation66& T66)
		{
			return T66.GetInverse();

			//delete: this is not correct if T66 is transposed (force transformation)!
			//Matrix3D AT;
			//Vector3D v;
			//T66toRotationTranslation(T66, AT, v);
			//AT.TransposeYourself();
			//return RotationTranslation2T66(AT, -(AT*v));
		}

		//! compute homogeneous transformation from Pluecker transformation
		inline HomogeneousTransformation T66toHT(const Transformation66& T66)
		{
			Matrix3D A;
			Vector3D v;
			T66toRotationTranslation(T66, A, v);

			return HomogeneousTransformation(A, v);
		}

		//! compute inertia parameters in T66 form from parameters (inertiaCOM at center of mass!)
		inline InertiaAtRefPoint InertiaT66FromInertiaParameters(Real mass, const Vector3D& centerOfMass, const Matrix3D& inertiaCOM)
		{
			Matrix3D skewCOM = RigidBodyMath::Vector2SkewMatrix(centerOfMass);
			Matrix3D massCOMCOMT = (-mass) * skewCOM * skewCOM; //minus represents transposed
			InertiaAtRefPoint A(6, 6);

			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					A(i, j) = inertiaCOM(i, j) + massCOMCOMT(i, j);
					A(i, j + 3) = mass * skewCOM(i, j);
					A(i + 3, j) = mass * skewCOM(j, i); //transposed
					if (i != j) { A(i + 3, j + 3) = 0.; }
					else { A(i + 3, j + 3) = mass; }
				}
			}
			return A;
		}

		//! compute inertia parameters from T66 representation (inertiaCOM at center of mass!);
		//! used for verification only!
		inline void InertiaParametersFromInertiaT66ATCOM(const InertiaAtRefPoint& I66,
			Real& mass, Vector3D& centerOfMass, Matrix3D& inertiaCOM)
		{
			mass = I66(3, 3);
			Matrix3D mS(3, 3);
			inertiaCOM.SetNumberOfRowsAndColumns(3, 3);
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					inertiaCOM(i, j) = I66(i, j);
					mS(i, j) = I66(i, j + 3);
				}
			}
			centerOfMass = (1. / mass)*RigidBodyMath::SkewMatrix2Vector(mS);
			inertiaCOM -= (-1. / mass)*mS*mS; //transposed replaces (-)
		}

		//! compute skew matrix for T66 motion vectors v=[vRot, vTrans]
		inline Transformation66 T66SkewMotion(const Vector6D& v)
		{
			return Transformation66(6, 6, {
				   0.,-v[2], v[1],   0.,   0.,   0.,
				 v[2],   0.,-v[0],   0.,   0.,   0.,
				-v[1], v[0],   0.,   0.,   0.,   0.,
				   0.,-v[5], v[4],   0.,-v[2], v[1],
				 v[5],   0.,-v[3], v[2],   0.,-v[0],
				-v[4], v[3],   0.,-v[1], v[0],   0. });
		}

		//! compute skew matrix for T66 force vectors v=[vRot, vTrans]
		inline Transformation66 T66SkewForce(const Vector6D& v)
		{
			//return T66SkewMotion(-v).GetTransposed();
			return Transformation66(6, 6, {
				   0.,-v[2], v[1],   0.,-v[5], v[4],
				 v[2],   0.,-v[0], v[5],   0.,-v[3],
				-v[1], v[0],   0.,-v[4], v[3],   0.,
				   0.,   0.,   0.,   0.,-v[2], v[1],
				   0.,   0.,   0., v[2],   0.,-v[0],
				   0.,   0.,   0.,-v[1], v[0],   0. });
		}

		//! compute v x w for motion
		inline Vector6D MultT66SkewMotion(const Vector6D& v, const Vector6D& w)
		{
			return T66SkewMotion(v) * w;
		}

		//! compute v x w for force
		inline Vector6D MultT66SkewForce(const Vector6D& v, const Vector6D& w)
		{
			return T66SkewForce(v) * w;
		}


		//! multiply with vector v=[vRot, vTrans]
		inline Vector6D T66Mult(const Transformation66& T66, const Vector6D& v)
		{
			return T66 * v;
		}

		//! multiply inertia with vector v=[vRot, vTrans]
		inline Vector6D T66MultInertia(const InertiaAtRefPoint& T66, const Vector6D& v)
		{
			return T66 * v;
		}

		//! multiply transposed T66 matrix with vector v=[vRot, vTrans]
		inline Vector6D T66MultTransposed(const Transformation66& T66, const Vector6D& v)
		{
			return v * T66;
		}

		//! multiply with vector v=[vRot, vTrans]
		inline InertiaAtRefPoint T66TransformInertia(const Transformation66& T66, const InertiaAtRefPoint& I66)
		{
			return T66.GetTransposed() * I66 * T66;
		}

		//! multiply inverse of transposed T66 with vector v (force transformation)
		inline Vector6D T66MultTransposedInverse(const Transformation66& T66, const Vector6D& v)
		{
			//return T66.GetTransposed().GetInverse() * v;
			//gives the same result, if T66 is motion transform:
			Transformation66 TinvT(6, 6); //inv.T

			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					TinvT(i, j) = T66(i, j); //rotation matrix stays same
					TinvT(i + 3, j + 3) = T66(i + 3, j + 3);

					TinvT(i, j + 3) = T66(i + 3, j);
					TinvT(i + 3, j) = 0.;
				}
			}

			return TinvT * v;
		}
	};
#else
//#endif
//#define TEST_T66_NEW
//#ifdef TEST_T66_NEW
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
		InertiaAtRefPoint() = default;

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

#endif //USE_EFFICIENT_TRANSFORMATION66



typedef ResizableArray<Transformation66> Transformation66List;
typedef ResizableArray<RigidBodyMath::InertiaAtRefPoint> InertiaList;
typedef ResizableArray<Matrix6D> Matrix6DList;


#endif
