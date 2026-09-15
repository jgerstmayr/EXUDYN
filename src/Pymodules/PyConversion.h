/** ***********************************************************************************************
* @file			PyConversion.h
* @brief		One conversion layer between Python objects and the C++ parameter types
* @details		Details:
* 				- FromPython(value, destination): converts a Python object into a C++ parameter and
* 				  raises a Python error (PyError) if it cannot; no return value to test
* 				- ToPython(value): converts a C++ parameter into the Python object handed to the user;
* 				  Real vectors and matrices become numpy arrays
* 				- item indices carry their kind as template argument (NodeIndex, ObjectIndex, ...),
* 				  so an ObjectIndex is rejected where a NodeIndex is expected
* 				- revision2026 step R4.4.3: introduced in 34c2 with the behaviour of the helpers in
* 				  PybindUtilities.h, which forward here; the generated code switches over in 34c4/34c5
* 				- deliberately independent of PybindUtilities.h, which is rewritten later
*
* @author		Gerstmayr Johannes, Claude-JG
* @date			2026-09-14 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef PYCONVERSION__H
#define PYCONVERSION__H

#include "Linalg/MatrixContainer.h"
#include "System/ItemIndices.h"
#include "Utilities/ExceptionsTemplates.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace EPyUtils {

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//type tests
	namespace Conversion
	{
		//! any Python or numpy integer or float
		inline bool IsScalar(const py::object& value)
		{
			return py::isinstance<py::float_>(value) || py::isinstance<std::int32_t>(value) || py::isinstance<py::int_>(value);
		}

		//! list or numpy array
		inline bool IsListOrArray(const py::object& value)
		{
			return py::isinstance<py::list>(value) || py::isinstance<py::array>(value);
		}

		//! list, tuple or numpy array: what the parameter conversions with a context accept as a sequence
		inline bool IsSequence(const py::object& value)
		{
			return IsListOrArray(value) || py::isinstance<py::tuple>(value);
		}

		//! the name used in error messages for an item index kind
		template<class TItemIndex> const char* ItemIndexName();
		template<> inline const char* ItemIndexName<NodeIndex>() { return "NodeIndex"; }
		template<> inline const char* ItemIndexName<ObjectIndex>() { return "ObjectIndex"; }
		template<> inline const char* ItemIndexName<MarkerIndex>() { return "MarkerIndex"; }
		template<> inline const char* ItemIndexName<LoadIndex>() { return "LoadIndex"; }
		template<> inline const char* ItemIndexName<SensorIndex>() { return "SensorIndex"; }

		//! true unless value is an index of one of the other four kinds; plain integers are accepted
		template<class TItemIndex>
		inline bool IsItemIndexOfKind(const py::object& value)
		{
			if (py::isinstance<TItemIndex>(value)) { return true; }
			return !(py::isinstance<NodeIndex>(value) || py::isinstance<ObjectIndex>(value) ||
				py::isinstance<MarkerIndex>(value) || py::isinstance<LoadIndex>(value) ||
				py::isinstance<SensorIndex>(value));
		}

		//! true for an item index of any kind (NodeIndex, ObjectIndex, MarkerIndex, LoadIndex, SensorIndex)
		inline bool IsItemIndex(const py::object& value)
		{
			return py::isinstance<NodeIndex>(value) || py::isinstance<ObjectIndex>(value) ||
				py::isinstance<MarkerIndex>(value) || py::isinstance<LoadIndex>(value) ||
				py::isinstance<SensorIndex>(value);
		}
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Python -> C++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! the range a scalar parameter must lie in (the U... and P... types of definitions/)
	enum class RangeCheck { nonNegative, positive };

	//! raises for None, which pybind11 would convert silently (bool: False; lists: empty); context names
	//! item and parameter for the message, e.g. "ObjectMassPoint.physicsMass"
	inline void RejectNone(const py::object& value, const char* context)
	{
		if (value.is_none())
		{
			PyError(STDstring("parameter ") + context + " received None; a value is required");
		}
	}

	//! a bool, Real, float or Index scalar; context names item and parameter for the message;
	//! an item index (NodeIndex, ...) is accepted only by Index, where it stands for its number
	template<class T>
	inline void FromPython(const py::object& value, T& destination, const char* context)
	{
		RejectNone(value, context);
		if (!std::is_same<T, Index>::value && Conversion::IsItemIndex(value))
		{
			PyError(STDstring("parameter ") + context + " expects a " + (std::is_same<T, bool>::value ? "bool" : (std::is_floating_point<T>::value ? "float" : "value of its enum type")) +
				", but received the item index " + EXUstd::ToString(value) + " of type " + EXUstd::ToString(value.get_type()));
		}
		destination = py::cast<T>(value);
	}

	//! raises if a must-be-given parameter (CFMustBeGiven in definitions/) still holds its placeholder default,
	//! e.g. MarkerNodeCoordinate.coordinate = InvalidIndex; like the range checks, switched off by
	//! exudyn.special.exceptions.parameterRangeChecks = False
	inline void RequireGiven(const py::object& value, Real placeholder, const char* context)
	{
		if (Conversion::IsScalar(value) && py::cast<Real>(value) == placeholder && EXUstd::ParameterRangeChecksActive())
		{
			PyError(STDstring("parameter ") + context + " must be given; the default " + EXUstd::ToString(placeholder) +
				" is only a placeholder");
		}
	}

	//! a Real, float or Index scalar with a range; context names item and parameter for the message,
	//! e.g. "ObjectMassPoint.physicsMass"; exudyn.special.exceptions.parameterRangeChecks = False accepts any value
	template<class T>
	inline void FromPython(const py::object& value, T& destination, RangeCheck range, const char* context)
	{
		T scalar;
		FromPython(value, scalar, context);
		bool valid = (range == RangeCheck::positive) ? (scalar > 0) : (scalar >= 0);
		if (!valid && EXUstd::ParameterRangeChecksActive())
		{
			PyError(STDstring("parameter ") + context + (range == RangeCheck::positive ? " must be positive (> 0)" : " may not be negative") +
				", but received " + EXUstd::ToString(scalar) +
				" (range checks can be switched off with exudyn.special.exceptions.parameterRangeChecks = False)");
		}
		destination = scalar;
	}

	//! a string; any other type raises
	inline void FromPython(const py::object& value, STDstring& destination)
	{
		if (!py::isinstance<py::str>(value))
		{
			PyError(STDstring("failed to convert to string: " + py::cast<std::string>(value)));
		}
		destination = py::cast<std::string>(value);
	}

	//! a fixed-size vector (Vector3D, Float4, ...) from a list or numpy array of exactly that size
	template<class T, Index size>
	inline void FromPython(const py::object& value, SlimVectorBase<T, size>& destination)
	{
		if (!Conversion::IsListOrArray(value)) //test first: a string would otherwise cast into a list of characters
		{
			PyError(STDstring("failed to convert SlimVector" + EXUstd::ToString(size) + ": " + py::cast<std::string>(value)));
		}
		std::vector<T> stdlist = py::cast<std::vector<T>>(value);
		if ((Index)stdlist.size() != size)
		{
			PyError("Vector" + EXUstd::ToString(size) + "D size mismatch: expected " + EXUstd::ToString(size) + " items in list!");
		}
		destination = stdlist;
	}

	//! a fixed-size matrix (Matrix3D, Matrix6D) from a list of lists or a 2D numpy array; rows and
	//! columns cannot be deduced from the storage size, so they are given: FromPython<Real, 3, 3>(value, m).
	//! The destination gets its size first, as an uninitialized ConstSizeMatrix has size zero
	template<class T, Index rows, Index columns>
	inline void FromPython(const py::object& value, ConstSizeMatrixBase<T, rows*columns>& destination)
	{
		destination.SetNumberOfRowsAndColumns(rows, columns);
		bool isList = py::isinstance<py::list>(value);
		if (!isList && !py::isinstance<py::array>(value))
		{
			PyError(STDstring("failed to convert to Matrix: " + py::cast<std::string>(value)));
		}
		std::vector<py::object> stdlist = py::cast<std::vector<py::object>>(value);
		if ((Index)stdlist.size() != rows)
		{
			PyError("Matrix size mismatch: expected " + EXUstd::ToString(rows) + " rows!");
		}
		for (Index i = 0; i < rows; i++)
		{
			if (isList && !py::isinstance<py::list>(stdlist[i]))
			{
				PyError("Matrix size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!');
			}
			std::vector<T> rowVector = py::cast<std::vector<T>>(stdlist[i]);
			if ((Index)rowVector.size() == columns)
			{
				for (Index j = 0; j < columns; j++)
				{
					destination(i, j) = rowVector[j];
				}
			}
			else if (!isList) //a list row of wrong length is left unchanged, as before 34c2
			{
				PyError("Matrix size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!');
			}
		}
	}

	namespace Conversion
	{
		//! numpy array (or list of lists) into any matrix class with SetNumberOfRowsAndColumns and operator();
		//! also used by the forwarding helpers in PybindUtilities.h for ConstSizeMatrix
		template<class T, class TMatrix>
		inline void NumpyToMatrix(const py::object& value, TMatrix& destination)
		{
			if (IsScalar(value)) //a scalar becomes a 1x1 matrix
			{
				T scalar = py::cast<T>(value);
				destination.SetMatrix(1, 1, { scalar });
				return;
			}
			py::array_t<T> pyArray = py::cast<py::array_t<T>>(value);
			if (pyArray.size() == 0) //an empty array has no second dimension
			{
				destination.SetNumberOfRowsAndColumns(0, 0);
			}
			else if (pyArray.ndim() == 2)
			{
				auto matrix = pyArray.template unchecked<2>(); //template keyword needed for gcc, see: https://github.com/pybind/pybind11/issues/1412
				destination.SetNumberOfRowsAndColumns((Index)matrix.shape(0), (Index)matrix.shape(1));
				for (Index i = 0; i < (Index)matrix.shape(0); i++)
				{
					for (Index j = 0; j < (Index)matrix.shape(1); j++)
					{
						destination(i, j) = matrix(i, j);
					}
				}
			}
			else
			{
				CHECKandTHROWstring("NumPy2Matrix: failed to convert numpy array to matrix: array must have dimension 2 (rows x columns)");
			}
		}

		//! numpy array (or list) into any vector class with SetNumberOfItems and operator[]
		template<class T, class TVector>
		inline void NumpyToVector(const py::object& value, TVector& destination)
		{
			if (IsScalar(value)) //a scalar becomes a vector of size 1
			{
				destination.SetNumberOfItems(1);
				destination[0] = py::cast<T>(value);
				return;
			}
			py::array_t<T> pyArray = py::cast<py::array_t<T>>(value);
			if (pyArray.ndim() != 1)
			{
				CHECKandTHROWstring("failed to convert numpy array to vector: array must have dimension 1 (list / matrix with 1 row, no columns)");
			}
			auto vector = pyArray.template unchecked<1>();
			destination.SetNumberOfItems((Index)vector.shape(0));
			for (Index i = 0; i < (Index)vector.shape(0); i++)
			{
				destination[i] = vector(i);
			}
		}
	}

	//! a matrix of any size (Matrix, MatrixI, ResizableMatrix) from a 2D numpy array or list of lists; a scalar becomes a 1x1 matrix
	template<class T>
	inline void FromPython(const py::object& value, MatrixBase<T>& destination)
	{
		Conversion::NumpyToMatrix<T>(value, destination);
	}

	//! a vector of any size (Vector, ResizableVector) from a 1D numpy array or list; a scalar becomes a vector of size 1
	template<class T>
	inline void FromPython(const py::object& value, VectorBase<T>& destination)
	{
		Conversion::NumpyToVector<T>(value, destination);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//item indices: the kind (NodeIndex, ...) is the template argument

	//! one index; a plain integer or an index of this kind, not of another kind
	template<class TItemIndex>
	inline void ItemIndexFromPython(const py::object& value, Index& destination)
	{
		if (!Conversion::IsItemIndexOfKind<TItemIndex>(value))
		{
			PyError(STDstring("Expected ") + Conversion::ItemIndexName<TItemIndex>() + ", but received '" + EXUstd::ToString(value) +
				"', type=" + EXUstd::ToString(value.get_type()) + "'; check potential mixing of different indices (ObjectIndex, NodeIndex, MarkerIndex, ...)!");
		}
		destination = py::cast<Index>(value);
	}

	//! one index as return value, e.g. Index nodeNumber = ItemIndexFromPython<NodeIndex>(value)
	template<class TItemIndex>
	inline Index ItemIndexFromPython(const py::object& value)
	{
		Index index;
		ItemIndexFromPython<TItemIndex>(value, index);
		return index;
	}

	//! a list or numpy array of indices of this kind; None raises
	template<class TItemIndex>
	inline void ItemIndexFromPython(const py::object& value, ArrayIndex& destination)
	{
		destination.SetNumberOfItems(0);
		if (!Conversion::IsListOrArray(value))
		{
			PyError(STDstring("Expected list of ") + Conversion::ItemIndexName<TItemIndex>() + ", but received '" + EXUstd::ToString(value) +
				"'; check potential mixing of different indices (ObjectIndex, NodeIndex, MarkerIndex, ...) or inconsistent arrays for nodeNumbers, markerNumbers, ...!");
		}
		py::list pylist = py::cast<py::list>(value); //also works for numpy arrays
		for (auto item : pylist)
		{
			Index index;
			ItemIndexFromPython<TItemIndex>(py::cast<py::object>(item), index);
			destination.Append(index);
		}
	}

	//! a fixed number of indices of this kind (NodeIndex2, NodeIndex3, NodeIndex4)
	template<class TItemIndex, Index size>
	inline void ItemIndexFromPython(const py::object& value, SlimArray<Index, size>& destination)
	{
		ArrayIndex arrayIndex;
		ItemIndexFromPython<TItemIndex>(value, arrayIndex);
		if (arrayIndex.NumberOfItems() != size)
		{
			PyError(STDstring("Expected list of ") + EXUstd::ToString(size) + " " + Conversion::ItemIndexName<TItemIndex>() + ", but received " +
				EXUstd::ToString(arrayIndex.NumberOfItems()) + " items in list");
		}
		destination = SlimArray<Index, size>(arrayIndex, 0);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//structure members (SimulationSettings, VisualizationSettings, ...): the same conversion with a context
	//"Class.member" for the message (revision2026 step R4.4.3.5); declared after all FromPython overloads,
	//because MemberSetter must see them

	//! a string member; None and other types raise
	inline void FromPython(const py::object& value, STDstring& destination, const char* context)
	{
		if (!py::isinstance<py::str>(value))
		{
			PyError(STDstring("parameter ") + context + " expects a string, but received " + EXUstd::ToString(value));
		}
		destination = py::cast<std::string>(value);
	}

	//! a list or numpy array of exactly size values (Float3, Float4, Vector3D, ...)
	template<class T, Index size>
	inline void FromPython(const py::object& value, SlimVectorBase<T, size>& destination, const char* context)
	{
		if (!Conversion::IsSequence(value))
		{
			PyError(STDstring("parameter ") + context + " expects a list or array of " + EXUstd::ToString(size) + " values, but received " + EXUstd::ToString(value));
		}
		std::vector<T> stdlist = py::cast<std::vector<T>>(value);
		if ((Index)stdlist.size() != size)
		{
			PyError(STDstring("parameter ") + context + " expects " + EXUstd::ToString(size) + " values, but received " + EXUstd::ToString((Index)stdlist.size()));
		}
		destination = stdlist;
	}

	//! a list or numpy array of integers of any length (plain indices, no item kind)
	inline void FromPython(const py::object& value, ArrayIndex& destination, const char* context)
	{
		if (!Conversion::IsSequence(value))
		{
			PyError(STDstring("parameter ") + context + " expects a list of integers, but received " + EXUstd::ToString(value));
		}
		destination = ArrayIndex(py::cast<std::vector<Index>>(value));
	}

	//! a list, tuple or 1D numpy array of any length (Vector)
	template<class T>
	inline void FromPython(const py::object& value, VectorBase<T>& destination, const char* context)
	{
		if (!Conversion::IsSequence(value))
		{
			PyError(STDstring("parameter ") + context + " expects a list or array, but received " + EXUstd::ToString(value));
		}
		std::vector<T> stdlist = py::cast<std::vector<T>>(value);
		destination = stdlist;
	}

	//! a list or numpy array of exactly size integers (Index2, ...)
	template<Index size>
	inline void FromPython(const py::object& value, SlimArray<Index, size>& destination, const char* context)
	{
		ArrayIndex arrayIndex;
		FromPython(value, arrayIndex, context);
		if (arrayIndex.NumberOfItems() != size)
		{
			PyError(STDstring("parameter ") + context + " expects " + EXUstd::ToString(size) + " integers, but received " + EXUstd::ToString(arrayIndex.NumberOfItems()));
		}
		destination = SlimArray<Index, size>(arrayIndex, 0);
	}

	//! the Python value of a structure member: scalars, strings and enums as they are; float vectors
	//! and index arrays as lists (colours are split and appended, not added)
	template<class T>
	inline const T& ToPythonMember(const T& value) { return value; }

	template<class T, Index size>
	inline std::array<T, size> ToPythonMember(const SlimVectorBase<T, size>& value)
	{
		std::array<T, size> list;
		for (Index i = 0; i < size; i++) { list[i] = value[i]; }
		return list;
	}

	template<Index size>
	inline std::array<Index, size> ToPythonMember(const SlimArray<Index, size>& value)
	{
		std::array<Index, size> list;
		for (Index i = 0; i < size; i++) { list[i] = value[i]; }
		return list;
	}

	inline std::vector<Index> ToPythonMember(const ArrayIndex& value) { return std::vector<Index>(value.begin(), value.end()); }

	//! pybind11 getter and setter of a structure data member, bound as
	//! .def_property("name", MemberGetter(&C::name), MemberSetter(&C::name, [range,] "C.name"))
	template<class TClass, class T>
	inline auto MemberGetter(T TClass::* member)
	{
		return [member](const TClass& object) { return ToPythonMember(object.*member); };
	}

	template<class TClass, class T>
	inline auto MemberSetter(T TClass::* member, const char* context)
	{
		return [member, context](TClass& object, const py::object& value) { FromPython(value, object.*member, context); };
	}

	template<class TClass, class T>
	inline auto MemberSetter(T TClass::* member, RangeCheck range, const char* context)
	{
		return [member, range, context](TClass& object, const py::object& value) { FromPython(value, object.*member, range, context); };
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//C++ -> Python
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! a fixed-size Real vector as numpy array (copy)
	template<Index size>
	inline py::array_t<Real> ToPython(const SlimVectorBase<Real, size>& value)
	{
		return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer());
	}

	//! a vector of any size as numpy array (copy)
	template<class T>
	inline py::array_t<T> ToPython(const VectorBase<T>& value)
	{
		return py::array_t<T>(value.NumberOfItems(), value.GetDataPointer());
	}

	//! a matrix of any size as 2D numpy array (copy)
	template<class T>
	inline py::array_t<T> ToPython(const MatrixBase<T>& value)
	{
		return py::array_t<T>(std::vector<std::ptrdiff_t>{(int)value.NumberOfRows(), (int)value.NumberOfColumns()}, value.GetDataPointer());
	}

	//! a fixed-size matrix as 2D numpy array (copy)
	template<class T, Index dataSize>
	inline py::array_t<T> ToPython(const ConstSizeMatrixBase<T, dataSize>& value)
	{
		return py::array_t<T>(std::vector<std::ptrdiff_t>{(int)value.NumberOfRows(), (int)value.NumberOfColumns()}, value.GetDataPointer());
	}

	//! indices as a list of their kind, e.g. [NodeIndex(0), NodeIndex(3)]
	template<class TItemIndex>
	inline std::vector<TItemIndex> ItemIndexToPython(const ArrayIndex& value)
	{
		std::vector<TItemIndex> list;
		for (auto item : value)
		{
			list.push_back(TItemIndex(item));
		}
		return list;
	}

} //namespace EPyUtils

#endif
