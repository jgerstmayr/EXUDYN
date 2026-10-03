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

	//! from a 3x3 rotation matrix and a translation, either of them None for the unit matrix or zero; or from a 4x4 matrix,
	//! or its 16 values row by row (as a sensor stores the output variable HomogeneousTransformation, #2792), given as
	//! rotation (translation then None)
	PyHT(const py::object& rotation, const py::object& translation) : HomogeneousTransformation()
	{
		if (!rotation.is_none())
		{
			std::vector<py::object> rows = py::cast<std::vector<py::object>>(rotation);
			if (rows.size() == 16)
			{
				CHECKandTHROW(translation.is_none(), "HT: with the 16 values of a 4x4 matrix, translation must be None");
				std::vector<Real> values = py::cast<std::vector<Real>>(rotation);
				ConstSizeMatrix<16> matrix44(4, 4);
				for (Index i = 0; i < 16; i++) { matrix44.GetDataPointer()[i] = values[i]; }
				SetHT44(matrix44);
				return;
			}
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
			ht = PyHT(value, py::none());
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
