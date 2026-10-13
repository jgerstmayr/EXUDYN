/** ***********************************************************************************************
* @brief		namespace PyCUtilities: the functions of exudyn.cUtilities, the kinematics of rigid bodies in C++ for
*				Python (#2960)
* @details		Details:
*				- each function takes lists or numpy arrays and returns numpy arrays; the computation is that of
*				  RigidBodyMath, EXUmath and EXUlie, with the math functions of Exudyn (EM::Sin, ...), so a value is
*				  the same bits on every platform
*				- bound in definitions/pybindCUtilities.py, which also writes the documentation; the Python names are
*				  those of exudyn.rigidBodyUtilities, which calls these
*
* @author		Gerstmayr Johannes
* @date			2026-10-11
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef PYCUTILITIES__H
#define PYCUTILITIES__H

#include "Linalg/RigidBodyMath.h"
#include "Pymodules/PybindUtilities.h"

namespace PyCUtilities {

	using ArrayReal = py::array_t<Real, py::array::c_style | py::array::forcecast>;

	//! the value as array of floats; a list, a tuple, a numpy array of any float or int type
	inline ArrayReal ToArray(const py::object& value, const char* functionName, const char* argumentName)
	{
		ArrayReal array = ArrayReal::ensure(value);
		if (!array) { PyError(STDstring(functionName) + ": " + argumentName + " must be a list or numpy array of floats", PyErrorType::typeError); }
		return array;
	}

	inline STDstring ShapeString(const ArrayReal& array)
	{
		STDstring s = "(";
		for (py::ssize_t i = 0; i < array.ndim(); i++) { s += (i ? ", " : "") + EXUstd::ToString((Index)array.shape(i)); }
		return s + (array.ndim() == 1 ? ",)" : ")");
	}

	//! a vector of size n: shape (n,), (n,1) or (1,n)
	template<Index n>
	inline SlimVectorBase<Real, n> GetVector(const py::object& value, const char* functionName, const char* argumentName)
	{
		ArrayReal array = ToArray(value, functionName, argumentName);
		bool isVector = array.ndim() == 1 || (array.ndim() == 2 && (array.shape(0) == 1 || array.shape(1) == 1));
		if (!isVector || array.size() != n)
		{
			PyError(STDstring(functionName) + ": " + argumentName + " must have " + EXUstd::ToString(n) + " components; received shape "
				+ ShapeString(array), PyErrorType::valueError);
		}
		SlimVectorBase<Real, n> v;
		for (Index i = 0; i < n; i++) { v[i] = array.data()[i]; }
		return v;
	}

	//! a 3x3 matrix: shape (3,3)
	inline Matrix3D GetMatrix3D(const py::object& value, const char* functionName, const char* argumentName)
	{
		ArrayReal array = ToArray(value, functionName, argumentName);
		if (array.ndim() != 2 || array.shape(0) != 3 || array.shape(1) != 3)
		{
			PyError(STDstring(functionName) + ": " + argumentName + " must be a 3x3 matrix; received shape " + ShapeString(array),
				PyErrorType::valueError);
		}
		Matrix3D m(3, 3);
		for (Index i = 0; i < 9; i++) { m.GetDataPointer()[i] = array.data()[i]; }
		return m;
	}

	template<Index n>
	inline py::array_t<Real> ToNumPy(const SlimVectorBase<Real, n>& v)
	{
		return py::array_t<Real>(n, v.GetDataPointer());
	}

	template<Index dataSize>
	inline py::array_t<Real> ToNumPy(const ConstSizeMatrixBase<Real, dataSize>& m)
	{
		return py::array_t<Real>(std::vector<py::ssize_t>({ m.NumberOfRows(), m.NumberOfColumns() }), m.GetDataPointer());
	}

	//! three vectors as the rows of a 3x3 array
	inline py::array_t<Real> RowsToNumPy(const Vector3D& v0, const Vector3D& v1, const Vector3D& v2)
	{
		return ToNumPy(Matrix3D(3, 3, { v0[0], v0[1], v0[2], v1[0], v1[1], v1[2], v2[0], v2[1], v2[2] }));
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//basis vectors and skew matrices

	//! the normalized vector0 and two normals; any vector0 except zero, for which the normals are the x and y axes
	inline Vector3D OrthonormalBasisVectors(const py::object& vector0, Vector3D& n1, Vector3D& n2, const char* functionName)
	{
		Vector3D v = GetVector<3>(vector0, functionName, "vector0");
		EXUmath::ComputeOrthogonalBasisVectors(v, n1, n2);
		Real L0 = v.GetL2Norm();
		if (L0 != 0.) { v *= 1. / L0; }
		return v;
	}

	inline py::array_t<Real> ComputeOrthonormalBasisVectors(const py::object& vector0)
	{
		Vector3D n1, n2;
		Vector3D v = OrthonormalBasisVectors(vector0, n1, n2, "ComputeOrthonormalBasisVectors");
		return RowsToNumPy(v, n1, n2);
	}

	inline py::array_t<Real> ComputeOrthonormalBasis(const py::object& vector0)
	{
		Vector3D n1, n2;
		Vector3D v = OrthonormalBasisVectors(vector0, n1, n2, "ComputeOrthonormalBasis");
		return ToNumPy(Matrix3D(3, 3, { v[0], n1[0], n2[0], v[1], n1[1], n2[1], v[2], n1[2], n2[2] }));
	}

	inline py::array_t<Real> GramSchmidt(const py::object& vector0, const py::object& vector1)
	{
		Vector3D v0 = GetVector<3>(vector0, "GramSchmidt", "vector0");
		Vector3D v1 = GetVector<3>(vector1, "GramSchmidt", "vector1");
		Real L0 = v0.GetL2Norm();
		Real L1 = v1.GetL2Norm();
		if (L0 == 0. || L1 == 0.) { PyError("GramSchmidt: vector0 and vector1 must not be zero", PyErrorType::valueError); }
		v0 *= 1. / L0;
		v1 *= 1. / L1;
		v1 -= (v1 * v0) * v0;
		Real L = v1.GetL2Norm();
		if (L == 0.) { PyError("GramSchmidt: vector0 and vector1 must not be parallel", PyErrorType::valueError); }
		v1 *= 1. / L;
		return RowsToNumPy(v0, v1, v0.CrossProduct(v1));
	}

	//! the two vectors, checked to be neither zero nor parallel
	inline void GetTwoVectors(const py::object& vector0, const py::object& vector1, const char* functionName, Vector3D& v0, Vector3D& v1)
	{
		v0 = GetVector<3>(vector0, functionName, "vector0");
		v1 = GetVector<3>(vector1, functionName, "vector1");
		Real L0 = v0.GetL2Norm();
		Real L1 = v1.GetL2Norm();
		if (L0 == 0. || L1 == 0.) { PyError(STDstring(functionName) + ": vector0 and vector1 must not be zero", PyErrorType::valueError); }
		if (v0.CrossProduct(v1).GetL2Norm() <= 1e-14 * L0 * L1) { PyError(STDstring(functionName) + ": vector0 and vector1 must not be parallel", PyErrorType::valueError); }
	}

	//! the symmetric basis of two non-parallel vectors, the columns e1, e2, e3 of a rotation matrix (#2228)
	inline py::array_t<Real> SymmetricBasis(const py::object& vector0, const py::object& vector1)
	{
		Vector3D v0, v1;
		GetTwoVectors(vector0, vector1, "SymmetricBasis", v0, v1);
		Vector3D e1, e2, e3;
		RigidBodyMath::SymmetricBasis(v0.GetDataPointer(), v1.GetDataPointer(), e1.GetDataPointer(), e2.GetDataPointer(), e3.GetDataPointer());
		return ToNumPy(Matrix3D(3, 3, { e1[0], e2[0], e3[0], e1[1], e2[1], e3[1], e1[2], e2[2], e3[2] }));
	}

	//! the rotation Jacobian G (3 x 6) of SymmetricBasis: omega = G [vector0_t; vector1_t] (#2228)
	inline py::array_t<Real> SymmetricBasisRotationJacobian(const py::object& vector0, const py::object& vector1)
	{
		Vector3D v0, v1;
		GetTwoVectors(vector0, vector1, "SymmetricBasisRotationJacobian", v0, v1);
		Real G[3][6];
		RigidBodyMath::SymmetricBasisRotationJacobian(v0.GetDataPointer(), v1.GetDataPointer(), G);
		ConstSizeMatrix<18> jacobian(3, 6);
		for (Index r = 0; r < 3; r++) { for (Index j = 0; j < 6; j++) { jacobian(r, j) = G[r][j]; } }
		return ToNumPy(jacobian);
	}

	//! the skew matrix of a 3D vector; for a vector of 3n components the n skew matrices one below the other, (3n,3);
	//! for a matrix of shape (3n,m) the skew matrices of its columns side by side, (3n,3m)
	inline py::array_t<Real> Skew(const py::object& vector)
	{
		ArrayReal array = ToArray(vector, "Skew", "vector");
		bool isRow = array.ndim() == 2 && array.shape(0) == 1;
		Index rows = array.ndim() == 1 || isRow ? (Index)array.size() : (array.ndim() == 2 ? (Index)array.shape(0) : 0);
		Index columns = array.ndim() == 1 || isRow ? 1 : (array.ndim() == 2 ? (Index)array.shape(1) : 0);
		if (rows == 0 || rows % 3 != 0 || columns == 0)
		{
			PyError("Skew: vector must have 3*n components, or be a matrix of 3*n rows; received shape " + ShapeString(array),
				PyErrorType::valueError);
		}
		py::array_t<Real> result(std::vector<py::ssize_t>({ rows, 3 * columns }));
		auto r = result.mutable_unchecked<2>();
		const Real* v = array.data();
		for (Index j = 0; j < columns; j++)
		{
			for (Index i = 0; i < rows; i += 3)
			{
				Real x = v[i*columns + j];
				Real y = v[(i + 1)*columns + j];
				Real z = v[(i + 2)*columns + j];
				r(i, 3*j) = 0.; r(i, 3*j + 1) = -z; r(i, 3*j + 2) = y;
				r(i + 1, 3*j) = z; r(i + 1, 3*j + 1) = 0.; r(i + 1, 3*j + 2) = -x;
				r(i + 2, 3*j) = -y; r(i + 2, 3*j + 1) = x; r(i + 2, 3*j + 2) = 0.;
			}
		}
		return result;
	}

	//! the vector of a skew matrix (3,3); of a 4x4 matrix of se(3) the 6 values [u, w], u the translational part
	inline py::array_t<Real> Skew2Vec(const py::object& skew)
	{
		ArrayReal array = ToArray(skew, "Skew2Vec", "skew");
		auto m = [&array](Index i, Index j) { return array.data()[i*array.shape(1) + j]; };
		if (array.ndim() == 2 && array.shape(0) == 3 && array.shape(1) == 3)
		{
			return ToNumPy(Vector3D({ m(2, 1), m(0, 2), -m(0, 1) }));
		}
		if (array.ndim() == 2 && array.shape(0) == 4 && array.shape(1) == 4)
		{
			return ToNumPy(Vector6D({ m(0, 3), m(1, 3), m(2, 3), m(2, 1), m(0, 2), -m(0, 1) }));
		}
		PyError("Skew2Vec: skew must be a 3x3 or 4x4 matrix; received shape " + ShapeString(array), PyErrorType::valueError);
		return py::array_t<Real>();
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Euler parameters

	inline py::array_t<Real> EulerParameters2G(const py::object& eulerParameters)
	{
		return ToNumPy(RigidBodyMath::EP2G(GetVector<4>(eulerParameters, "EulerParameters2G", "eulerParameters")));
	}

	inline py::array_t<Real> EulerParameters2GLocal(const py::object& eulerParameters)
	{
		return ToNumPy(RigidBodyMath::EP2Glocal(GetVector<4>(eulerParameters, "EulerParameters2GLocal", "eulerParameters")));
	}

	inline py::array_t<Real> EulerParameters2RotationMatrix(const py::object& eulerParameters)
	{
		return ToNumPy(RigidBodyMath::EP2RotationMatrix(GetVector<4>(eulerParameters, "EulerParameters2RotationMatrix", "eulerParameters")));
	}

	//! normalized, so that an inaccurate rotation matrix gives unit Euler parameters, which the checks of the nodes require
	inline py::array_t<Real> RotationMatrix2EulerParameters(const py::object& rotationMatrix)
	{
		Vector4D ep;
		RigidBodyMath::RotationMatrix2EP(GetMatrix3D(rotationMatrix, "RotationMatrix2EulerParameters", "rotationMatrix"), ep[0], ep[1], ep[2], ep[3]);
		Real norm = ep.GetL2Norm();
		if (norm != 0.) { ep *= 1. / norm; }
		return ToNumPy(ep);
	}

	//! 0.25*G^T*omega, for unit Euler parameters
	inline py::array_t<Real> AngularVelocity2EulerParameters_t(const py::object& angularVelocity, const py::object& eulerParameters)
	{
		Vector3D omega = GetVector<3>(angularVelocity, "AngularVelocity2EulerParameters_t", "angularVelocity");
		ConstSizeMatrix<12> G = RigidBodyMath::EP2G(GetVector<4>(eulerParameters, "AngularVelocity2EulerParameters_t", "eulerParameters"));
		Vector4D ep_t;
		for (Index j = 0; j < 4; j++) { ep_t[j] = 0.25*(G(0, j)*omega[0] + G(1, j)*omega[1] + G(2, j)*omega[2]); }
		return ToNumPy(ep_t);
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//rotation vector

	inline py::array_t<Real> RotationVector2RotationMatrix(const py::object& rotationVector)
	{
		return ToNumPy(RigidBodyMath::RotationVector2RotationMatrix(GetVector<3>(rotationVector, "RotationVector2RotationMatrix", "rotationVector")));
	}

	inline py::array_t<Real> RotationMatrix2RotationVector(const py::object& rotationMatrix)
	{
		return ToNumPy(EXUlie::LogSO3Vector(GetMatrix3D(rotationMatrix, "RotationMatrix2RotationVector", "rotationMatrix")));
	}

	inline py::array_t<Real> ComputeRotationAxisFromRotationVector(const py::object& rotationVector)
	{
		Vector3D axis;
		EXUlie::RotationVector2RotationAxis(GetVector<3>(rotationVector, "ComputeRotationAxisFromRotationVector", "rotationVector"), axis);
		return ToNumPy(axis);
	}

	inline py::array_t<Real> ExpSO3(const py::object& rotationVector)
	{
		return ToNumPy(EXUlie::ExpSO3(GetVector<3>(rotationVector, "ExpSO3", "rotationVector")));
	}

	inline py::array_t<Real> LogSO3(const py::object& rotationMatrix)
	{
		return ToNumPy(EXUlie::LogSO3(GetMatrix3D(rotationMatrix, "LogSO3", "rotationMatrix")));
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Tait-Bryan angles xyz and Euler angles zyz

	inline py::array_t<Real> RotXYZ2RotationMatrix(const py::object& rot)
	{
		return ToNumPy(RigidBodyMath::RotXYZ2RotationMatrix(GetVector<3>(rot, "RotXYZ2RotationMatrix", "rot")));
	}

	inline py::array_t<Real> RotationMatrix2RotXYZ(const py::object& rotationMatrix)
	{
		return ToNumPy(RigidBodyMath::RotationMatrix2RotXYZ(GetMatrix3D(rotationMatrix, "RotationMatrix2RotXYZ", "rotationMatrix")));
	}

	inline py::array_t<Real> RotXYZ2G(const py::object& rot)
	{
		return ToNumPy(RigidBodyMath::RotXYZ2GTemplate(GetVector<3>(rot, "RotXYZ2G", "rot")));
	}

	inline py::array_t<Real> RotXYZ2G_t(const py::object& rot, const py::object& rot_t)
	{
		return ToNumPy(RigidBodyMath::RotXYZ2G_tTemplate(GetVector<3>(rot, "RotXYZ2G_t", "rot"), GetVector<3>(rot_t, "RotXYZ2G_t", "rot_t")));
	}

	inline py::array_t<Real> RotXYZ2GLocal(const py::object& rot)
	{
		return ToNumPy(RigidBodyMath::RotXYZ2GlocalTemplate(GetVector<3>(rot, "RotXYZ2GLocal", "rot")));
	}

	inline py::array_t<Real> RotXYZ2GLocal_t(const py::object& rot, const py::object& rot_t)
	{
		return ToNumPy(RigidBodyMath::RotXYZ2Glocal_tTemplate(GetVector<3>(rot, "RotXYZ2GLocal_t", "rot"), GetVector<3>(rot_t, "RotXYZ2GLocal_t", "rot_t")));
	}

	inline py::array_t<Real> AngularVelocity2RotXYZ_t(const py::object& angularVelocity, const py::object& rotation)
	{
		Vector3D rot = GetVector<3>(rotation, "AngularVelocity2RotXYZ_t", "rotation");
		if (fabs(EM::Cos(rot[1])) < 1e-14) //cos(pi/2) is 6e-17, never 0
		{
			PyError("AngularVelocity2RotXYZ_t: not possible for rotation[1] = pi/2, 3*pi/2, ... (|cos(rotation[1])| < 1e-14)", PyErrorType::valueError);
		}
		return ToNumPy(RigidBodyMath::AngularVelocity2RotXYZ_t(GetVector<3>(angularVelocity, "AngularVelocity2RotXYZ_t", "angularVelocity"), rot));
	}

	inline py::array_t<Real> RotXYZ2EulerParameters(const py::object& alpha)
	{
		return ToNumPy(RigidBodyMath::RotXYZ2EP(GetVector<3>(alpha, "RotXYZ2EulerParameters", "alpha")));
	}

	inline py::array_t<Real> RotationMatrix2RotZYZ(const py::object& rotationMatrix, bool flip)
	{
		return ToNumPy(RigidBodyMath::RotationMatrix2RotZYZ(GetMatrix3D(rotationMatrix, "RotationMatrix2RotZYZ", "rotationMatrix"), flip));
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//elementary rotations

	inline py::array_t<Real> RotationMatrixX(Real angleRad) { return ToNumPy(RigidBodyMath::RotationMatrixX(angleRad)); }
	inline py::array_t<Real> RotationMatrixY(Real angleRad) { return ToNumPy(RigidBodyMath::RotationMatrixY(angleRad)); }
	inline py::array_t<Real> RotationMatrixZ(Real angleRad) { return ToNumPy(RigidBodyMath::RotationMatrixZ(angleRad)); }

	inline py::array_t<Real> RotationMatrix2D(Real angleRad)
	{
		Real c = EM::Cos(angleRad);
		Real s = EM::Sin(angleRad);
		return ToNumPy(ConstSizeMatrix<4>(2, 2, { c, -s, s, c }));
	}

} //namespace PyCUtilities

#endif
