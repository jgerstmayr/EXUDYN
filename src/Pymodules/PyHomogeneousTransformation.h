/** ***********************************************************************************************
* @brief		class PyHT: the homogeneous transformation of Exudyn's C++ core for Python, exudyn.HT (#2780)
* @details		Details:
*				- the C++ class HomogeneousTransformation with the conversions from and to numpy arrays and lists;
*				  bound in definitions/pybindDataStructures.py
*
* @author		Gerstmayr Johannes
* @date			2026-10-02
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef PYHOMOGENEOUSTRANSFORMATION__H
#define PYHOMOGENEOUSTRANSFORMATION__H

#include "Linalg/RigidBodyMath.h"
#include "Pymodules/PyConversion.h"
#include "Pymodules/PybindUtilities.h"

class PyHT : public HomogeneousTransformation
{
public:
	//! the identity
	PyHT() : HomogeneousTransformation() {}
	PyHT(const HomogeneousTransformation& other) : HomogeneousTransformation(other) {}

	//! from a 3x3 rotation matrix and a translation, either of them None for the unit matrix or zero, the rotation also as
	//! Euler parameters, Tait-Bryan angles Rxyz or a rotation vector (#2810); or from a 4x4 matrix, or its 16 values row by
	//! row (as a sensor stores the output variable HomogeneousTransformation, #2792), given as rotation (the others None)
	PyHT(const py::object& rotation, const py::object& translation, const py::object& eulerParameters,
		const py::object& Rxyz, const py::object& rotationVector) : HomogeneousTransformation()
	{
		if (py::isinstance<PyHT>(rotation)) //a copy of an exu.HT, so that exu.HT(H) takes an HT as it takes a 4x4 matrix (#2824)
		{
			CHECKandTHROW(translation.is_none() && eulerParameters.is_none() && Rxyz.is_none() && rotationVector.is_none(),
				"HT: with an HT, the other arguments must be None");
			*this = py::cast<const PyHT&>(rotation);
			return;
		}
		if (!rotation.is_none())
		{
			std::vector<py::object> rows = py::cast<std::vector<py::object>>(rotation);
			if (rows.size() == 16 || rows.size() == 4)
			{
				CHECKandTHROW(translation.is_none() && eulerParameters.is_none() && Rxyz.is_none() && rotationVector.is_none(),
					"HT: with a 4x4 matrix or its 16 values, the other arguments must be None");
				ConstSizeMatrix<16> matrix44(4, 4);
				if (rows.size() == 16)
				{
					std::vector<Real> values = py::cast<std::vector<Real>>(rotation);
					for (Index i = 0; i < 16; i++) { matrix44.GetDataPointer()[i] = values[i]; }
				}
				else { EPyUtils::FromPython<Real, 4, 4>(rotation, matrix44); }
				SetHT44(matrix44);
				UpdateNoRotationFlag();
				return;
			}
		}
		SetPy(rotation, translation, eulerParameters, Rxyz, rotationVector);
	}

	py::array_t<Real> GetRotationPy() const { return EPyUtils::ToPython(GetRotation()); }

	//! the rotation, a 3x3 matrix as list of lists or numpy array; the translation is kept; a unit matrix sets the flag of
	//! no rotation, which the C++ side sets only by construction (#2810)
	void SetRotationPy(const py::object& rotation)
	{
		Matrix3D A;
		EPyUtils::FromPython<Real, 3, 3>(rotation, A);
		SetRotationMatrix(A);
		UpdateNoRotationFlag();
	}

	//! the rotation from at most one of: a rotation matrix, Euler parameters, Tait-Bryan angles Rxyz, a rotation vector;
	//! none given keeps the rotation
	void SetRotationParametersPy(const py::object& rotation, const py::object& eulerParameters, const py::object& Rxyz,
		const py::object& rotationVector)
	{
		Index given = (Index)!rotation.is_none() + (Index)!eulerParameters.is_none() + (Index)!Rxyz.is_none() + (Index)!rotationVector.is_none();
		CHECKandTHROW(given <= 1, "HT: give at most one of rotation, eulerParameters, Rxyz and rotationVector");
		if (!rotation.is_none()) { SetRotationPy(rotation); }
		else if (!eulerParameters.is_none())
		{
			Vector4D ep;
			EPyUtils::FromPython(eulerParameters, ep);
			CHECKandTHROW(fabs(ep.GetL2NormSquared() - 1.) < 1e-10, "HT: the Euler parameters must have unit length");
			SetRotationMatrixChecked(RigidBodyMath::EP2RotationMatrix(ep));
		}
		else if (!Rxyz.is_none())
		{
			Vector3D angles;
			EPyUtils::FromPython(Rxyz, angles);
			SetRotationMatrixChecked(RigidBodyMath::RotXYZ2RotationMatrix(angles));
		}
		else if (!rotationVector.is_none())
		{
			Vector3D v;
			EPyUtils::FromPython(rotationVector, v);
			SetRotationMatrixChecked(EXUlie::ExpSO3(v));
		}
	}

	//! a rotation computed from parameters; the flag of no rotation for a zero rotation
	void SetRotationMatrixChecked(const Matrix3D& A)
	{
		SetRotationMatrix(A);
		UpdateNoRotationFlag();
	}

	py::array_t<Real> GetTranslationPy() const { return EPyUtils::ToPython(GetTranslation()); }

	//! the translation; the rotation is kept
	void SetTranslationPy(const py::object& translation)
	{
		Vector3D p;
		EPyUtils::FromPython(translation, p);
		GetTranslation() = p;
	}

	//! [rotation, translation]
	py::list GetPy() const
	{
		py::list list;
		list.append(GetRotationPy());
		list.append(GetTranslationPy());
		return list;
	}

	//! the parts given; a part that is None stays as it is
	void SetPy(const py::object& rotation, const py::object& translation, const py::object& eulerParameters,
		const py::object& Rxyz, const py::object& rotationVector)
	{
		SetRotationParametersPy(rotation, eulerParameters, Rxyz, rotationVector);
		if (!translation.is_none()) { SetTranslationPy(translation); }
	}

	//! a translation and the unit rotation
	void SetTranslationOnlyPy(const py::object& translation)
	{
		Vector3D p;
		EPyUtils::FromPython(translation, p);
		SetTranslation(p);
	}

	py::array_t<Real> GetHT44Py() const { return EPyUtils::ToPython(GetHT44()); }

	//! numpy indexing of the 4x4 matrix for reading, H[0:3,3], H[2][3]; the result does not refer back to the HT (#2821)
	py::object GetItemPy(const py::object& key) const { return GetHT44Py().attr("__getitem__")(key); }

	//! numpy indexing of the 4x4 matrix for writing the rotation and the translation, H[0:3,3] = p; the last row
	//! [0,0,0,1] cannot change (#2821)
	void SetItemPy(const py::object& key, const py::object& value)
	{
		py::array_t<Real> matrix = GetHT44Py();
		matrix.attr("__setitem__")(key, value);
		ConstSizeMatrix<16> matrix44(4, 4);
		EPyUtils::FromPython<Real, 4, 4>(matrix, matrix44);
		CHECKandTHROW(matrix44(3, 0) == 0. && matrix44(3, 1) == 0. && matrix44(3, 2) == 0. && matrix44(3, 3) == 1.,
			"HT[...] = value: the last row of an HT is [0,0,0,1] and cannot be written");
		SetHT44(matrix44);
		UpdateNoRotationFlag();
	}

	PyHT GetInversePy() const { return PyHT(GetInverse()); }

	//! H1*H2
	PyHT MultiplyHT(const PyHT& other) const { return PyHT(*this * other); }

	//! H*other: an HT for an HT, a numpy array for a 3D vector
	py::object Multiply(const py::object& other) const
	{
		if (py::isinstance<PyHT>(other)) { return py::cast(MultiplyHT(py::cast<const PyHT&>(other))); }
		return MultiplyVector(other);
	}

	//! H1@H2, an HT as H1*H2; H@x for anything else as numpy computes it with the 4x4 matrix, so that scripts written
	//! for 4x4 numpy arrays keep working (#2821)
	py::object MatMul(const py::object& other) const
	{
		if (py::isinstance<PyHT>(other)) { return py::cast(MultiplyHT(py::cast<const PyHT&>(other))); }
		return GetHT44Py().attr("__matmul__")(other);
	}

	//! H*v, a point transformed
	py::array_t<Real> MultiplyVector(const py::object& vector) const
	{
		Vector3D v;
		EPyUtils::FromPython(vector, v);
		return EPyUtils::ToPython((const HomogeneousTransformation&)*this * v);
	}

	py::array_t<Real> RotateVectorPy(const py::object& vector) const
	{
		Vector3D v;
		EPyUtils::FromPython(vector, v);
		return EPyUtils::ToPython(RotateVector(v));
	}

	py::array_t<Real> RotateVectorTransposedPy(const py::object& vector) const
	{
		Vector3D v;
		EPyUtils::FromPython(vector, v);
		return EPyUtils::ToPython(RotateVectorTransposed(v));
	}

	//! the rotation parameters of the rigid body nodes: Euler parameters (NodeRigidBodyEP), Tait-Bryan angles
	//! (NodeRigidBodyRxyz), rotation vector (NodeRigidBodyRotVecLG)
	Vector4D GetEP() const
	{
		Vector4D ep;
		RigidBodyMath::RotationMatrix2EP(GetRotation(), ep[0], ep[1], ep[2], ep[3]);
		return ep;
	}
	Vector3D GetRxyz() const { return RigidBodyMath::RotationMatrix2RotXYZ(GetRotation()); }
	Vector3D GetRotationVector() const { return EXUlie::LogSO3Vector(GetRotation()); }

	py::array_t<Real> GetEPPy() const { return EPyUtils::ToPython(GetEP()); }
	py::array_t<Real> GetRxyzPy() const { return EPyUtils::ToPython(GetRxyz()); }
	py::array_t<Real> GetRotationVectorPy() const { return EPyUtils::ToPython(GetRotationVector()); }

	//! the reference coordinates of a rigid body node: the translation and the rotation parameters
	template<Index size, class TVector>
	py::array_t<Real> CoordinatesPy(const TVector& rotationParameters) const
	{
		std::vector<Real> q(size);
		for (Index i = 0; i < 3; i++) { q[i] = GetTranslation()[i]; }
		for (Index i = 3; i < size; i++) { q[i] = rotationParameters[i - 3]; }
		return py::array_t<Real>(size, q.data());
	}
	py::array_t<Real> GetCoordinatesEPPy() const { return CoordinatesPy<7>(GetEP()); }
	py::array_t<Real> GetCoordinatesRxyzPy() const { return CoordinatesPy<6>(GetRxyz()); }
	py::array_t<Real> GetCoordinatesRotationVectorPy() const { return CoordinatesPy<6>(GetRotationVector()); }

	//! set from the reference coordinates of a rigid body node: 7 (translation, Euler parameters) or 6 (translation and
	//! Tait-Bryan angles or rotation vector)
	void SetCoordinatesPy(const py::object& coordinates, Index kind)
	{
		std::vector<Real> q = py::cast<std::vector<Real>>(coordinates);
		Index size = (kind == 0) ? 7 : 6;
		CHECKandTHROW((Index)q.size() == size, (STDstring("HT: the coordinates must have ") + EXUstd::ToString(size) + " components").c_str());
		py::list rotationParameters;
		for (Index i = 3; i < size; i++) { rotationParameters.append(q[i]); }
		py::list translation;
		for (Index i = 0; i < 3; i++) { translation.append(q[i]); }
		py::object none = py::none();
		SetPy(none, translation, kind == 0 ? (py::object)rotationParameters : none, kind == 1 ? (py::object)rotationParameters : none,
			kind == 2 ? (py::object)rotationParameters : none);
	}

	//! the angle of the rotation, the norm of the rotation vector
	Real RotationAnglePy() const { return GetRotationVector().GetL2Norm(); }

	//! the unit axis of the rotation; for no rotation an error, or the zero vector if raiseError is False
	py::array_t<Real> RotationAxisPy(bool raiseError) const
	{
		Vector3D v = GetRotationVector();
		Real angle = v.GetL2Norm();
		if (angle == 0.)
		{
			CHECKandTHROW(!raiseError, "HT.RotationAxis(): the transformation has no rotation; use raiseError=False to get [0,0,0]");
			return EPyUtils::ToPython(v);
		}
		return EPyUtils::ToPython((1. / angle) * v);
	}

	//! a rotation by angle about axis (any length but zero) and zero translation
	void SetRotationAxisPy(const py::object& axis, Real angle)
	{
		Vector3D n;
		EPyUtils::FromPython(axis, n);
		Real norm = n.GetL2Norm();
		CHECKandTHROW(norm != 0., "HT.SetRotationAxis(...): the axis must not be zero");
		SetRotationMatrixChecked(EXUlie::ExpSO3((angle / norm) * n));
		GetTranslation().SetAll(0.);
	}

	//! the frame of other seen from this one, H0^-1*H1: rotation A0^T*A1, translation A0^T*(p1-p0)
	PyHT Relative(const PyHT& other) const
	{
		PyHT result;
		Vector3D p = other.GetTranslation() - GetTranslation();
		result.GetTranslation() = RotateVectorTransposed(p);
		if (!(HasNoRotation() && other.HasNoRotation()))
		{
			result.SetRotationMatrix(GetRotation().GetTransposed() * other.GetRotation());
		}
		return result;
	}

	//! between this (factor 0) and other (factor 1): the translation linear, the rotation on SO(3), about one fixed axis
	PyHT InterpolateSO3(const PyHT& other, Real factor) const
	{
		PyHT result;
		result.GetTranslation() = (1. - factor) * GetTranslation() + factor * other.GetTranslation();
		Matrix3D A0 = GetRotation();
		Vector3D v = EXUlie::LogSO3Vector(Matrix3D(A0.GetTransposed() * other.GetRotation()));
		result.SetRotationMatrixChecked(A0 * EXUlie::ExpSO3(factor * v));
		return result;
	}

	//! between this (factor 0) and other (factor 1) on SE(3): a screw motion, translation and rotation coupled
	PyHT InterpolateSE3(const PyHT& other, Real factor) const
	{
		Vector3D incDisp, incRot;
		EXUlie::LogSE3Vector(GetInverse() * other, incDisp, incRot);
		PyHT result(*this * EXUlie::ExpSE3(factor * incDisp, factor * incRot));
		result.UpdateNoRotationFlag();
		return result;
	}

	//! the 6 components [U, Omega] of a motion vector as numpy array
	static py::array_t<Real> MotionVectorPy(const Vector3D& U, const Vector3D& Omega)
	{
		Real v[6] = { U[0], U[1], U[2], Omega[0], Omega[1], Omega[2] };
		return py::array_t<Real>(6, v);
	}

	//! a motion vector [U, Omega] from Python, 6 components
	static void MotionVectorFromPython(const py::object& vector, Vector3D& U, Vector3D& Omega, const char* where)
	{
		std::vector<Real> v = py::cast<std::vector<Real>>(vector);
		CHECKandTHROW(v.size() == 6, (STDstring(where) + ": the vector must have 6 components").c_str());
		U = Vector3D({ v[0], v[1], v[2] });
		Omega = Vector3D({ v[3], v[4], v[5] });
	}

	//! the logarithm on SE(3): the motion vector [U, Omega] with H = ExpSE3([U, Omega]) (#2819)
	py::array_t<Real> LogSE3Py() const
	{
		Vector3D U, Omega;
		EXUlie::LogSE3Vector(static_cast<const HomogeneousTransformation&>(*this), U, Omega);
		return MotionVectorPy(U, Omega);
	}

	//! set from the exponential map on SE(3) of the motion vector [U, Omega] (#2819)
	void SetExpSE3Py(const py::object& vector)
	{
		Vector3D U, Omega;
		MotionVectorFromPython(vector, U, Omega, "HT.SetExpSE3(...)");
		HomogeneousTransformation H = EXUlie::ExpSE3(U, Omega);
		SetRotationMatrixChecked(H.GetRotation());
		GetTranslation() = H.GetTranslation();
	}

	//! the logarithm on R3xSO(3): the translation and the rotation vector, each on its own (#2819)
	py::array_t<Real> LogR3xSO3Py() const { return MotionVectorPy(GetTranslation(), GetRotationVector()); }

	//! set from the exponential map on R3xSO(3) of [U, Omega]: the translation U, the rotation ExpSO3(Omega) (#2819)
	void SetExpR3xSO3Py(const py::object& vector)
	{
		Vector3D U, Omega;
		MotionVectorFromPython(vector, U, Omega, "HT.SetExpR3xSO3(...)");
		SetRotationMatrixChecked(EXUlie::ExpSO3(Omega));
		GetTranslation() = U;
	}

	STDstring ToString() const
	{
		return "HT(rotation=" + EXUstd::ToString(GetRotation()) + ", translation=" + EXUstd::ToString(GetTranslation()) + ")";
	}
};

//! the HT parameters of items (#2793): an HT parameter, e.g. referenceHT, and its parts, a position and a rotation
//! parameter (referencePosition, referenceRotation), are views of one stored HomogeneousTransformation
namespace EPyUtils
{
	//! the HT as 4x4 numpy array, as the dictionary of an item holds it
	inline py::array_t<Real> ToPython(const HomogeneousTransformation& ht) { return ToPython(ht.GetHT44()); }

	//! an HT from an exu.HT, a 4x4 matrix or its 16 values row by row
	inline void HTFromPython(const py::object& value, HomogeneousTransformation& ht, const char* context)
	{
		if (py::isinstance<PyHT>(value)) { ht = py::cast<const PyHT&>(value); }
		else
		{
			Index size = -1;
			if (py::isinstance<py::sequence>(value) || py::isinstance<py::array>(value)) { size = (Index)py::len(value); }
			if (size != 4 && size != 16)
			{
				PyError(STDstring(context) + ": an HT must be an exu.HT, a 4x4 matrix or its 16 values", PyErrorType::typeError);
				return;
			}
			ht = PyHT(value, py::none(), py::none(), py::none(), py::none());
		}
		ht.UpdateNoRotationFlag();
	}

	//! the position part of an HT; its rotation is kept
	inline void HTPositionFromPython(const py::object& value, HomogeneousTransformation& ht, const char* context)
	{
		FromPython(value, ht.GetTranslation());
	}

	//! the rotation part of an HT; its position is kept
	inline void HTRotationFromPython(const py::object& value, HomogeneousTransformation& ht, const char* context)
	{
		Matrix3D rotation;
		FromPython<Real, 3, 3>(value, rotation);
		ht.SetRotationMatrix(rotation);
		ht.UpdateNoRotationFlag();
	}

	//! the HT and its parts from a dictionary: what is given (and not None) is written; the HT and a part given
	//! together must agree, as in a dictionary read from the item; positionName or rotationName may be nullptr
	inline void HTFromDictionary(const py::dict& d, const char* htName, const char* positionName, const char* rotationName,
		HomogeneousTransformation& ht, const char* className)
	{
		auto Given = [&d](const char* name) { return name != nullptr && DictItemExists(d, name) && !d[name].is_none(); };
		if (Given(htName))
		{
			HomogeneousTransformation value;
			HTFromPython(d[htName], value, (STDstring(className) + "." + htName).c_str());
			HomogeneousTransformation part = value;
			if (Given(positionName)) { HTPositionFromPython(d[positionName], part, className); }
			if (Given(rotationName)) { HTRotationFromPython(d[rotationName], part, className); }
			Real difference = (part.GetTranslation() - value.GetTranslation()).GetL2Norm();
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++) { difference += fabs(part.Rotation(i, j) - value.Rotation(i, j)); }
			}
			if (difference > 1e-12 * (1. + value.GetTranslation().GetL2Norm()))
			{
				PyError(STDstring(className) + ": " + htName + " and " + (positionName ? positionName : "") +
					(positionName && rotationName ? " or " : "") + (rotationName ? rotationName : "") +
					" are given and differ; give one of them, the other None", PyErrorType::valueError);
				return;
			}
			ht = value;
		}
		else
		{
			if (Given(positionName)) { HTPositionFromPython(d[positionName], ht, className); }
			if (Given(rotationName)) { HTRotationFromPython(d[rotationName], ht, className); }
		}
	}
}

//! a list of HTs, of an item that stores it as a list of rotations and a list of translations (#2798)
namespace EPyUtils
{
	//! the HTs as a Python list of 4x4 numpy arrays
	inline py::list HTListToPython(const Matrix3DList& rotations, const Vector3DList& translations)
	{
		py::list list;
		Index n = EXUstd::Minimum(rotations.NumberOfItems(), translations.NumberOfItems());
		for (Index i = 0; i < n; i++) { list.append(ToPython(HomogeneousTransformation(rotations[i], translations[i]))); }
		return list;
	}

	//! rotations and translations from a list of exu.HT, 4x4 matrices or their 16 values
	inline void HTListFromPython(const py::object& value, Matrix3DList& rotations, Vector3DList& translations, const char* context)
	{
		if (value.is_none() || !(py::isinstance<py::sequence>(value) || py::isinstance<py::array>(value)))
		{
			PyError(STDstring(context) + ": a list of HTs must be a list of exu.HT, 4x4 matrices or their 16 values", PyErrorType::typeError);
			return;
		}
		rotations.SetNumberOfItems(0);
		translations.SetNumberOfItems(0);
		for (py::handle item : value)
		{
			HomogeneousTransformation ht;
			HTFromPython(py::reinterpret_borrow<py::object>(item), ht, context);
			rotations.Append(ht.GetRotation());
			translations.Append(ht.GetTranslation());
		}
	}

	//! the list of HTs from a dictionary, if given and not None: it writes the rotations and translations, which the
	//! dictionary may give as well (written before): then they must agree
	inline void HTListFromDictionary(const py::dict& d, const char* htName, const char* rotationsName, const char* translationsName,
		Matrix3DList& rotations, Vector3DList& translations, const char* className)
	{
		if (!DictItemExists(d, htName) || d[htName].is_none()) { return; }
		Matrix3DList newRotations;
		Vector3DList newTranslations;
		HTListFromPython(d[htName], newRotations, newTranslations, (STDstring(className) + "." + htName).c_str());
		auto Given = [&d](const char* name) { return DictItemExists(d, name) && !d[name].is_none() && py::len(d[name]) != 0; };
		if (Given(rotationsName) || Given(translationsName))
		{
			bool agree = rotations.NumberOfItems() == newRotations.NumberOfItems() && translations.NumberOfItems() == newTranslations.NumberOfItems();
			for (Index i = 0; agree && i < newRotations.NumberOfItems(); i++)
			{
				Real difference = (translations[i] - newTranslations[i]).GetL2Norm();
				for (Index k = 0; k < 9; k++) { difference += fabs(rotations[i].GetDataPointer()[k] - newRotations[i].GetDataPointer()[k]); }
				agree = difference <= 1e-12 * (1. + newTranslations[i].GetL2Norm());
			}
			if (!agree)
			{
				PyError(STDstring(className) + ": " + htName + " and " + rotationsName + " or " + translationsName +
					" are given and differ; give one of them, the other None", PyErrorType::valueError);
				return;
			}
		}
		rotations = newRotations;
		translations = newTranslations;
	}
}

//! an output variable as Python object (#2789): HomogeneousTransformation as exu.HT, from its 16 values row by row; a single
//! value as float; else a numpy array
inline py::object OutputVariableToPython(OutputVariableType variableType, const Vector& value)
{
	if (variableType == OutputVariableType::HomogeneousTransformation)
	{
		ConstSizeMatrix<16> matrix44(4, 4);
		for (Index i = 0; i < 16; i++) { matrix44.GetDataPointer()[i] = value[i]; }
		PyHT ht;
		ht.SetHT44(matrix44);
		return py::cast(ht);
	}
	if (value.NumberOfItems() == 1) { return py::float_(value[0]); }
	return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer());
}

#endif
