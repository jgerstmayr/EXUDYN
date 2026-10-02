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

class PyHT : public HomogeneousTransformation
{
public:
	//! the identity
	PyHT() : HomogeneousTransformation() {}
	PyHT(const HomogeneousTransformation& other) : HomogeneousTransformation(other) {}

	//! from a 3x3 rotation matrix and a translation, either of them None for the unit matrix or zero; or from a 4x4 matrix
	//! given as rotation (translation then None)
	PyHT(const py::object& rotation, const py::object& translation) : HomogeneousTransformation()
	{
		if (!rotation.is_none())
		{
			std::vector<py::object> rows = py::cast<std::vector<py::object>>(rotation);
			if (rows.size() == 4)
			{
				CHECKandTHROW(translation.is_none(), "HT: with a 4x4 matrix, translation must be None");
				ConstSizeMatrix<16> matrix44;
				EPyUtils::FromPython<Real, 4, 4>(rotation, matrix44);
				SetHT44(matrix44);
				return;
			}
			SetRotationPy(rotation);
		}
		if (!translation.is_none()) { SetTranslationPy(translation); }
	}

	py::array_t<Real> GetRotationPy() const { return EPyUtils::ToPython(GetRotation()); }

	//! the rotation, a 3x3 matrix as list of lists or numpy array; the translation is kept
	void SetRotationPy(const py::object& rotation)
	{
		Matrix3D A;
		EPyUtils::FromPython<Real, 3, 3>(rotation, A);
		SetRotationMatrix(A);
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

	void SetPy(const py::object& rotation, const py::object& translation)
	{
		SetRotationPy(rotation);
		SetTranslationPy(translation);
	}

	//! a translation and the unit rotation
	void SetTranslationOnlyPy(const py::object& translation)
	{
		Vector3D p;
		EPyUtils::FromPython(translation, p);
		SetTranslation(p);
	}

	py::array_t<Real> GetHT44Py() const { return EPyUtils::ToPython(GetHT44()); }

	PyHT GetInversePy() const { return PyHT(GetInverse()); }

	//! H1*H2
	PyHT MultiplyHT(const PyHT& other) const { return PyHT(*this * other); }

	//! H*other: an HT for an HT, a numpy array for a 3D vector
	py::object Multiply(const py::object& other) const
	{
		if (py::isinstance<PyHT>(other)) { return py::cast(MultiplyHT(py::cast<const PyHT&>(other))); }
		return MultiplyVector(other);
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

	STDstring ToString() const
	{
		return "HT(rotation=" + EXUstd::ToString(GetRotation()) + ", translation=" + EXUstd::ToString(GetTranslation()) + ")";
	}
};

#endif
