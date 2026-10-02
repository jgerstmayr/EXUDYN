/** ***********************************************************************************************
* @brief		class HomogeneousTransformationBase
*				A homogeneous transformation (HT): a rotation matrix A and a translation p, as the 4x4 matrix
*				[A p; 0 1]; the frame of a rigid body, marker or joint; follows the Python implementation in
*				exudyn.rigidBodyUtilities and is bound to Python as exudyn.HT (#2780)
* @details		Details:
*				- stored are the 12 numbers it needs, the rotation row by row and the translation, and a flag for a
*				  transformation without rotation, which the products use to skip the rotation; the flag is set by
*				  the functions that set no rotation (identity, translation), never by comparing a given matrix;
*				  homogeneousTransformationUseIdentityFlag switches it off, to measure both
*				- the operations that are hot - H*v, H^-1, H1*H2, set and get - are written out with fixed-size loops
*				- the Lie group operations (SetRotation of a rotation vector, GetRelativeMotionTo) are defined in
*				  RigidBodyMath.h, with the exponential and logarithmic maps
*
* @author		Gerstmayr Johannes
* @date			2026-10-02 (moved from RigidBodyMath.h)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef HOMOGENEOUSTRANSFORMATION__H
#define HOMOGENEOUSTRANSFORMATION__H

#include "Linalg/BasicLinalg.h"

//! true: a transformation set without rotation skips the rotation in its products (#2780)
constexpr bool homogeneousTransformationUseIdentityFlag = true;

template<typename T>
class HomogeneousTransformationBase
{
private:
	T R[9];						//!< the rotation matrix, row by row
	SlimVectorBase<T, 3> p;		//!< the translation
	bool noRotation;			//!< true if the rotation is the unit matrix by construction (identity, translation only)

	void SetUnitRotation()
	{
		for (Index i = 0; i < 9; i++) { R[i] = (T)0.; }
		R[0] = (T)1.; R[4] = (T)1.; R[8] = (T)1.;
		noRotation = homogeneousTransformationUseIdentityFlag;
	}
	bool SkipRotation() const { return homogeneousTransformationUseIdentityFlag && noRotation; }

public:
	//! the identity transformation
	HomogeneousTransformationBase(bool initialize = true)
	{
		SetUnitRotation();
		p.SetAll((T)0.);
	}

	//! a rotation matrix and a translation
	HomogeneousTransformationBase(const ConstSizeMatrixBase<T, 9>& rotation, const SlimVectorBase<T, 3>& translation) : p(translation)
	{
		SetRotationMatrix(rotation);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//set and get

	//! set rotation and translation
	void Set(const ConstSizeMatrixBase<T, 9>& rotation, const SlimVectorBase<T, 3>& translation)
	{
		SetRotationMatrix(rotation);
		p = translation;
	}

	//! get rotation and translation
	void Get(ConstSizeMatrixBase<T, 9>& rotation, SlimVectorBase<T, 3>& translation) const
	{
		rotation = GetRotation();
		translation = p;
	}

	//! the rotation matrix; the translation is kept
	void SetRotationMatrix(const ConstSizeMatrixBase<T, 9>& rotation)
	{
		CHECKandTHROW(rotation.NumberOfRows() == 3 && rotation.NumberOfColumns() == 3, "HomogeneousTransformation: the rotation must be a 3x3 matrix");
		const T* data = rotation.GetDataPointer();
		for (Index i = 0; i < 9; i++) { R[i] = data[i]; }
		noRotation = false;
	}

	//! set the flag of no rotation if the rotation is exactly the unit matrix; for parameters, which are set once (#2793)
	void UpdateNoRotationFlag()
	{
		noRotation = homogeneousTransformationUseIdentityFlag && R[0] == (T)1. && R[1] == (T)0. && R[2] == (T)0. && R[3] == (T)0.
			&& R[4] == (T)1. && R[5] == (T)0. && R[6] == (T)0. && R[7] == (T)0. && R[8] == (T)1.;
	}

	//! set a rotation and zero translation
	void SetRotation(const ConstSizeMatrixBase<T, 9>& rotation)
	{
		SetRotationMatrix(rotation);
		p.SetAll((T)0.);
	}

	//! set a rotation and zero translation from rotation vector (exponential map, in RigidBodyMath.h)
	void SetRotation(const SlimVectorBase<T, 3>& rotation);

	//! set a translation and identity rotation
	void SetTranslation(const SlimVectorBase<T, 3>& translation)
	{
		SetUnitRotation();
		p = translation;
	}

	void SetTranslationX(T x) { SetTranslation(SlimVectorBase<T, 3>({ x, (T)0., (T)0. })); }
	void SetTranslationY(T y) { SetTranslation(SlimVectorBase<T, 3>({ (T)0., y, (T)0. })); }
	void SetTranslationZ(T z) { SetTranslation(SlimVectorBase<T, 3>({ (T)0., (T)0., z })); }

	//! set identity
	void SetIdentity()
	{
		SetUnitRotation();
		p.SetAll((T)0.);
	}

	//! set a rotation around x-axis and zero translation
	void SetRotationX(T angleRad)
	{
		T c = cos(angleRad), s = sin(angleRad);
		SetRotation(ConstSizeMatrixBase<T, 9>(3, 3, { (T)1., (T)0., (T)0., (T)0., c, -s, (T)0., s, c }));
	}

	//! set a rotation around y-axis and zero translation
	void SetRotationY(T angleRad)
	{
		T c = cos(angleRad), s = sin(angleRad);
		SetRotation(ConstSizeMatrixBase<T, 9>(3, 3, { c, (T)0., s, (T)0., (T)1., (T)0., -s, (T)0., c }));
	}

	//! set a rotation around z-axis and zero translation
	void SetRotationZ(T angleRad)
	{
		T c = cos(angleRad), s = sin(angleRad);
		SetRotation(ConstSizeMatrixBase<T, 9>(3, 3, { c, -s, (T)0., s, c, (T)0., (T)0., (T)0., (T)1. }));
	}

	//! the rotation matrix (a copy of the 9 numbers)
	ConstSizeMatrixBase<T, 9> GetRotation() const
	{
		ConstSizeMatrixBase<T, 9> A(3, 3);
		T* data = A.GetDataPointer();
		for (Index i = 0; i < 9; i++) { data[i] = R[i]; }
		return A;
	}

	//! one entry of the rotation matrix
	T Rotation(Index row, Index column) const { return R[3 * row + column]; }

	//! true if the transformation was set without rotation (the flag; a given unit matrix does not set it)
	bool HasNoRotation() const { return SkipRotation(); }

	//! the translation (read)
	const SlimVectorBase<T, 3>& GetTranslation() const { return p; }

	//! the translation (write); the rotation is kept
	SlimVectorBase<T, 3>& GetTranslation() { return p; }

	//! the translation as floats
	Float3 GetTranslationF() const { return Float3({ (float)p[0], (float)p[1], (float)p[2] }); }

	//! the rotation as floats
	Matrix3DF GetRotationF() const
	{
		return Matrix3DF(3, 3, { (float)R[0], (float)R[1], (float)R[2], (float)R[3], (float)R[4], (float)R[5], (float)R[6], (float)R[7], (float)R[8] });
	}

	//! the 4x4 matrix; for output or at the end of computations
	ConstSizeMatrixBase<T, 16> GetHT44() const
	{
		ConstSizeMatrixBase<T, 16> HT(4, 4);
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++) { HT(i, j) = R[3 * i + j]; }
			HT(i, 3) = p[i];
			HT(3, i) = (T)0.;
		}
		HT(3, 3) = (T)1.;
		return HT;
	}

	//! the 4x4 matrix as floats
	ConstSizeMatrixF<16> GetHT44F() const
	{
		ConstSizeMatrixF<16> HT(4, 4);
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++) { HT(i, j) = (float)R[3 * i + j]; }
			HT(i, 3) = (float)p[i];
			HT(3, i) = 0.f;
		}
		HT(3, 3) = 1.f;
		return HT;
	}

	//! the 4x4 matrix row by row, 16 values, into any vector type (the output variable HomogeneousTransformation)
	template<typename VectorT>
	void GetHT44RowByRow(VectorT& value) const
	{
		value.SetNumberOfItems(16);
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++) { value[4 * i + j] = R[3 * i + j]; }
			value[4 * i + 3] = p[i];
		}
		value[12] = (T)0.; value[13] = (T)0.; value[14] = (T)0.; value[15] = (T)1.;
	}

	//! set with any 4x4 matrix type
	template<typename MatrixT>
	void SetHT44(const MatrixT& HT)
	{
		CHECKandTHROW(HT.NumberOfRows() == 4 && HT.NumberOfColumns() == 4, "HomogeneousTransformation::SetHT44: wrong matrix dimension");
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++) { R[3 * i + j] = HT(i, j); }
			p[i] = HT(i, 3);
		}
		noRotation = false;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//the operations

	//! A*v, the rotation of a vector
	SlimVectorBase<T, 3> RotateVector(const SlimVectorBase<T, 3>& v) const
	{
		if (SkipRotation()) { return v; }
		return SlimVectorBase<T, 3>({ R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
									  R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
									  R[6] * v[0] + R[7] * v[1] + R[8] * v[2] });
	}

	//! A^T*v
	SlimVectorBase<T, 3> RotateVectorTransposed(const SlimVectorBase<T, 3>& v) const
	{
		if (SkipRotation()) { return v; }
		return SlimVectorBase<T, 3>({ R[0] * v[0] + R[3] * v[1] + R[6] * v[2],
									  R[1] * v[0] + R[4] * v[1] + R[7] * v[2],
									  R[2] * v[0] + R[5] * v[1] + R[8] * v[2] });
	}

	//! invert the transformation: A^T, -A^T p
	void Invert()
	{
		if (!SkipRotation())
		{
			EXUstd::Swap(R[1], R[3]);
			EXUstd::Swap(R[2], R[6]);
			EXUstd::Swap(R[5], R[7]);
		}
		p = -RotateVector(p);
	}

	//! the inverse transformation
	HomogeneousTransformationBase GetInverse() const
	{
		HomogeneousTransformationBase inverse(*this);
		inverse.Invert();
		return inverse;
	}

	//! H1*H2
	friend HomogeneousTransformationBase operator* (const HomogeneousTransformationBase& H1, const HomogeneousTransformationBase& H2)
	{
		HomogeneousTransformationBase result(H2);
		result.p = H1.RotateVector(H2.p);
		result.p += H1.p;
		if (H1.SkipRotation()) { return result; } //the rotation of H2
		if (H2.SkipRotation()) //the rotation of H1
		{
			for (Index i = 0; i < 9; i++) { result.R[i] = H1.R[i]; }
			result.noRotation = false;
			return result;
		}
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				result.R[3 * i + j] = H1.R[3 * i] * H2.R[j] + H1.R[3 * i + 1] * H2.R[3 + j] + H1.R[3 * i + 2] * H2.R[6 + j];
			}
		}
		result.noRotation = false;
		return result;
	}

	//! H*v, a point transformed: A*v + p
	friend SlimVectorBase<T, 3> operator* (const HomogeneousTransformationBase& H, const SlimVectorBase<T, 3>& v)
	{
		SlimVectorBase<T, 3> result = H.RotateVector(v);
		result += H.p;
		return result;
	}

	HomogeneousTransformationBase& operator*= (const HomogeneousTransformationBase& other)
	{
		*this = *this * other;
		return *this;
	}

	//! component-wise comparison of rotation and translation
	bool operator== (const HomogeneousTransformationBase& H) const
	{
		for (Index i = 0; i < 9; i++) { if (!(R[i] == H.R[i])) { return false; } }
		return p == H.p;
	}

	//! the skew matrix in the rotation part (as LogSE3 returns it) to incremental rotation and displacement
	void Skew2Vector(SlimVectorBase<T, 3>& incDisp, SlimVectorBase<T, 3>& incRot)
	{
		incRot[0] = R[7]; //A(2,1)
		incRot[1] = R[2]; //A(0,2)
		incRot[2] = R[3]; //A(1,0)
		incDisp = p;
	}

	//! the difference of *this frame to HT1 as logarithm of the relative transformation (in RigidBodyMath.h);
	//! *this*ExpSE(incDisp, incRot) = HT1
	void GetRelativeMotionTo(const HomogeneousTransformationBase& HT1, SlimVectorBase<T, 3>& incDisp, SlimVectorBase<T, 3>& incRot);

	friend std::ostream& operator<<(std::ostream& os, const HomogeneousTransformationBase& HT)
	{
		os << "[" << HT.GetRotation() << ", " << HT.GetTranslation() << "]";
		return os;
	}
};

typedef HomogeneousTransformationBase<Real> HomogeneousTransformation;
typedef HomogeneousTransformationBase<float> HomogeneousTransformationF;

#endif
