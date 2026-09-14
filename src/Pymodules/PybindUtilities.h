/** ***********************************************************************************************
* @file			PybindUtilities.h
* @brief		This file contains helper functions and utilities for pybind11 integration
* @details		Details:
* 				- Helper functions for manipulating arrays, vectors, etc.
*
* @author		Gerstmayr Johannes
* @date			2019-04-24 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef PYBINDUTILITIES__H
#define PYBINDUTILITIES__H

#include "Linalg/MatrixContainer.h"	
#include "System/ItemIndices.h"	
//#include "Pymodules/PyMatrixVector.h"
#include "Utilities/ExceptionsTemplates.h"
#include "Pymodules/PyConversion.h"  //FromPython / ToPython; the helpers below forward there (step 34c2)

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
//#include <pybind11/operators.h>
#include <pybind11/numpy.h>			//interface to numpy
//#include <pybind11/cast.h>		//
#include <pybind11/functional.h>    //not sure if this is needed, but for safety as functions are converted here as well; for function handling ... otherwise gives a python error (no compilation error in C++ !)

//typedef py::array_t<Real> PyNumpyArray; //PyNumpyArray is used at some places to avoid include of pybind

namespace py = pybind11;            //! namespace 'py' used throughout in code

//! Exudyn python utilities namespace
namespace EPyUtils { 

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++ forwarding to PyConversion.h (revision plan step 34c2); removed in 34c6 +++++++++++++++++++
	//these keep the old names for the generated code and the hand-written callers until they call
	//FromPython / ToPython / ItemIndexFromPython / ItemIndexToPython directly
	inline bool IsNodeIndex(const py::object& pyObject) { return Conversion::IsItemIndexOfKind<NodeIndex>(pyObject); }
	inline bool IsObjectIndex(const py::object& pyObject) { return Conversion::IsItemIndexOfKind<ObjectIndex>(pyObject); }
	inline bool IsMarkerIndex(const py::object& pyObject) { return Conversion::IsItemIndexOfKind<MarkerIndex>(pyObject); }
	inline bool IsLoadIndex(const py::object& pyObject) { return Conversion::IsItemIndexOfKind<LoadIndex>(pyObject); }
	inline bool IsSensorIndex(const py::object& pyObject) { return Conversion::IsItemIndexOfKind<SensorIndex>(pyObject); }

	template<class TItemIndex>
	inline Index GetItemIndexSafelyForward(const py::object& pyObject) { Index index; ItemIndexFromPython<TItemIndex>(pyObject, index); return index; }
	inline Index GetNodeIndexSafely(const py::object& pyObject) { return GetItemIndexSafelyForward<NodeIndex>(pyObject); }
	inline Index GetObjectIndexSafely(const py::object& pyObject) { return GetItemIndexSafelyForward<ObjectIndex>(pyObject); }
	inline Index GetMarkerIndexSafely(const py::object& pyObject) { return GetItemIndexSafelyForward<MarkerIndex>(pyObject); }
	inline Index GetLoadIndexSafely(const py::object& pyObject) { return GetItemIndexSafelyForward<LoadIndex>(pyObject); }
	inline Index GetSensorIndexSafely(const py::object& pyObject) { return GetItemIndexSafelyForward<SensorIndex>(pyObject); }

	template<class TItemIndex>
	inline ArrayIndex GetArrayItemIndexSafelyForward(const py::object& pyObject) { ArrayIndex indices; ItemIndexFromPython<TItemIndex>(pyObject, indices); return indices; }
	inline ArrayIndex GetArrayNodeIndexSafely(const py::object& pyObject) { return GetArrayItemIndexSafelyForward<NodeIndex>(pyObject); }
	inline ArrayIndex GetArrayObjectIndexSafely(const py::object& pyObject) { return GetArrayItemIndexSafelyForward<ObjectIndex>(pyObject); }
	inline ArrayIndex GetArrayMarkerIndexSafely(const py::object& pyObject) { return GetArrayItemIndexSafelyForward<MarkerIndex>(pyObject); }
	inline ArrayIndex GetArraySensorIndexSafely(const py::object& pyObject) { return GetArrayItemIndexSafelyForward<SensorIndex>(pyObject); }

	inline Index2 GetNodeIndex2Safely(const py::object& pyObject) { Index2 indices; ItemIndexFromPython<NodeIndex>(pyObject, indices); return indices; }
	inline Index3 GetNodeIndex3Safely(const py::object& pyObject) { Index3 indices; ItemIndexFromPython<NodeIndex>(pyObject, indices); return indices; }
	inline Index4 GetNodeIndex4Safely(const py::object& pyObject) { Index4 indices; ItemIndexFromPython<NodeIndex>(pyObject, indices); return indices; }

	inline std::vector<NodeIndex> GetArrayNodeIndex(const ArrayIndex& arrayIndex) { return ItemIndexToPython<NodeIndex>(arrayIndex); }
	inline std::vector<ObjectIndex> GetArrayObjectIndex(const ArrayIndex& arrayIndex) { return ItemIndexToPython<ObjectIndex>(arrayIndex); }
	inline std::vector<MarkerIndex> GetArrayMarkerIndex(const ArrayIndex& arrayIndex) { return ItemIndexToPython<MarkerIndex>(arrayIndex); }
	inline std::vector<SensorIndex> GetArraySensorIndex(const ArrayIndex& arrayIndex) { return ItemIndexToPython<SensorIndex>(arrayIndex); }

	inline bool SetStringSafely(const py::object& value, STDstring& destination) { FromPython(value, destination); return true; }
	inline bool SetStringSafely(const py::dict& d, const char* itemName, STDstring& destination)
	{
		if (!d.contains(itemName) || !py::isinstance<py::str>(d[itemName]))
		{
			PyError(STDstring("ERROR: failed to convert '") + itemName + "' into string; dictionary:\n" + EXUstd::ToString(d));
		}
		FromPython(d[itemName], destination);
		return true;
	}

	template<class T, Index size>
	inline bool SetSlimVectorTemplateSafely(const py::object& value, SlimVectorBase<T, size>& destination) { FromPython(value, destination); return true; }
	template<typename T, Index size>
	inline bool SetSlimVectorTemplateSafely(const py::dict& d, const char* item, SlimVectorBase<T, size>& destination)
	{
		if (!d.contains(item) || !Conversion::IsListOrArray(d[item]))
		{
			PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector" + EXUstd::ToString(size) + "D; dictionary:\n" + EXUstd::ToString(d));
		}
		FromPython(d[item], destination);
		return true;
	}

	template<typename T, Index rows, Index columns>
	inline bool SetConstMatrixTypeTemplateSafely(const py::object& value, ConstSizeMatrixBase<T, rows*columns>& destination) { FromPython<T, rows, columns>(value, destination); return true; }

	template<typename T, class TMatrix>
	inline bool SetNumpyMatrixSafelyTemplate(const py::object& value, TMatrix& destination) { Conversion::NumpyToMatrix<T>(value, destination); return true; }
	template<typename T, class TVector>
	inline bool SetNumpyVectorSafelyTemplate(const py::object& value, TVector& destination) { Conversion::NumpyToVector<T>(value, destination); return true; }

	template<typename T, class TMatrix>
	inline void NumPy2Matrix(const py::array_t<T>& pyArray, TMatrix& m) { Conversion::NumpyToMatrix<T>(pyArray, m); }
	template<typename T>
	inline void NumPy2Vector(const py::array_t<T>& pyArray, VectorBase<T>& v) { Conversion::NumpyToVector<T>(pyArray, v); }

	inline py::array_t<Real> Vector2NumPy(const Vector& v) { return ToPython(v); }
	template<Index dataSize>
	inline py::array_t<Real> SlimVector2NumPy(const SlimVector<dataSize>& v) { return ToPython(v); }
	template<class TMatrix>
	py::array_t<Real> Matrix2NumPyTemplate(const TMatrix& matrix) { return ToPython(matrix); }
	inline py::array_t<Real> Matrix2NumPy(const Matrix& matrix) { return ToPython(matrix); }
	inline py::array_t<Index> MatrixI2NumPy(const MatrixI& matrix) { return ToPython(matrix); }
	//+++ end of forwarding to PyConversion.h ++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//! function to check if a specific item exists (but type is not checked) in the dictionary
	inline bool DictItemExists(const py::dict& d, const char* itemName)
	{
		if (d.contains(itemName)) { return true; }
		return false;
	}

	//! return true, if dictionary contains item 'itemName' with valid string
	inline bool DictItemIsValidString(const py::dict& d, const char* itemName)
	{
		if (d.contains(itemName))
		{
			py::object other = d[itemName]; //this is necessary to make isinstance work
			if (py::isinstance<py::str>(other))
			{
				return true; //yes, item is a string
			}
		}
		return false;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	inline bool IsPyTypeString(const py::object& pyObject)
	{
		return py::isinstance<py::str>(pyObject);
	}

	inline bool IsPyTypeReal(const py::object& pyObject)
	{
		return py::isinstance<py::float_>(pyObject);
	}

	//! check if python type is any kind of integer; np.array uses int32_t, while standard lists use int_ types
	inline bool IsPyTypeInteger(const py::object& pyObject)
	{
		return //py::isinstance<std::int64_t>(pyObject) || //may be needed by other compilers?
			//py::isinstance<std::int8_t>(pyObject) ||
			//py::isinstance<std::int16_t>(pyObject) ||
			py::isinstance<std::int32_t>(pyObject) ||
			//py::isinstance<std::int64_t>(pyObject) ||
			//py::isinstance<std::intptr_t>(pyObject) ||
			//py::isinstance<std::int_fast32_t>(pyObject) ||
			py::isinstance<py::int_>(pyObject);
	}

	inline bool IsPyTypeScalar(const py::object& pyObject)
	{
		return IsPyTypeReal(pyObject) || IsPyTypeInteger(pyObject);
	}

	//! return true if is list or numpy array
	inline bool IsPyTypeListOrArray(const py::object& pyObject)
	{
		return (py::isinstance<py::list>(pyObject) || py::isinstance<py::array>(pyObject));
	}

	//! check if py::object is list (list of lists) or numpy array
	//! if yes, return true; columns=0 means vector (list or 1D numpy array); otherwise it is a matrix
	inline bool GetPyArrayOrListDimensions(const py::object& obj, int& rows, int& columns) 
	{
		rows = 0;
		columns = 0;

		//Check if the object is a numpy array
		if (py::isinstance<py::array>(obj)) {
			py::array arr = py::cast<py::array>(obj);
			py::buffer_info info = arr.request();

			rows = (Index)info.shape[0];
			if (info.ndim == 2) {
				columns = (Index)info.shape[1];
			}
			else
			{
				//return false => superfunction will raise Error anyways
				PyWarning("Received numpy array with invalid dimension " + EXUstd::ToString(info.ndim));
				return false;
			}
		}
		//Check if the object is a list (or list of lists)
		else if (py::isinstance<py::list>(obj)) 
		{
			py::list lst = py::cast<py::list>(obj);

			rows = (Index)lst.size();
			if (rows > 0 && py::isinstance<py::list>(lst[0])) {
				// Object is a list of lists
				py::list first_row = py::cast<py::list>(lst[0]);
				columns = (Index)first_row.size();
			}
		}
		else
		{
			return false;
		}
		return true;
	}


	









	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// some conversion functions for conversion of (internal, C++) index arrays to arrays of NodeIndex, MarkerIndex, ...





	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	inline bool CheckForValidFunction(const py::object pyObject)
	{
		if (py::isinstance<py::function>(pyObject))
		{
			return true;
		}
		else if (IsPyTypeInteger(pyObject))
		{
			if (py::cast<int>(pyObject) != 0) 
			{ 
				PyError(STDstring("Failed to convert PyFunction: must be either valid Python function or 0, but got ")+EXUstd::ToString(pyObject)); 
			}
			return false; //this is a valid value, but no function (0-function pointer means empty function (in C++: nullptr))
		}
		else
		{
			PyError(STDstring("Failed to convert PyFunction: must be either valid Python function or int, but got ")+ EXUstd::ToString(pyObject));
		}
		return false;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! cast py::object to std::function; accept also zero
	template<typename STDfunction>
	STDfunction GetSTDfunction(const py::object& function, const char* info="GetSTDfunction")
	{
		if (py::isinstance<py::int_>(function))
		{
			Index num = py::cast<py::int_>(function);
			if (num != 0)
			{
				CHECKandTHROWstring( (STDstring(info)+": parameter must be either a Python function or 0 but received: "+EXUstd::ToString(function)).c_str() );
			}
			else
			{
				return 0;
			}
		}
		else if (py::isinstance<py::function>(function))

		{
			return py::cast<STDfunction>(function);
		}
		else
		{
			CHECKandTHROWstring((STDstring(info) + ": parameter must be either a Python function or 0 but received '" + EXUstd::ToString(function)+"'").c_str());
		}
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//inline bool SetVector2DSafely(const py::dict& d, const char* item, Vector2D& destination) {
	//	return SetSlimVectorTemplateSafely<Real, 2>(d, item, destination); }

	//Delete:
	//inline bool SetVector3DSafely(const py::dict& d, const char* item, Vector3D& destination) {
	//	return SetSlimVectorTemplateSafely<Real, 3>(d, item, destination);}

	//inline bool SetVector4DSafely(const py::dict& d, const char* item, Vector4D& destination) {
	//	return SetSlimVectorTemplateSafely<Real, 4>(d, item, destination);}

	//inline bool SetVector6DSafely(const py::dict& d, const char* item, Vector6D& destination) {
	//	return SetSlimVectorTemplateSafely<Real, 6>(d, item, destination);}

	//inline bool SetVector7DSafely(const py::dict& d, const char* item, Vector7D& destination) {
	//	return SetSlimVectorTemplateSafely<Real, 7>(d, item, destination);}


	//! Set a ConstMatrix of any size from a py::object safely and return false (if failed) and true if value has been set
	template<Index rows, Index columns>
	inline bool SetConstMatrixTemplateSafely(const py::object& value, ConstSizeMatrix<rows*columns>& destination)
	{
		return SetConstMatrixTypeTemplateSafely<Real, rows, columns>(value, destination);
	}

	template<Index rows, Index columns>
	inline bool SetConstMatrixTemplateSafely(const py::dict& d, const char* item, ConstSizeMatrix<rows*columns>& destination)
	{

		if (d.contains(item))
		{
			py::object other = d[item]; //this is necessary to make isinstance work
			return SetConstMatrixTemplateSafely<rows,columns>(other, destination);
		}
		PyError(STDstring("ERROR: failed to convert '") + item + "' into Matrix; dictionary:\n" + EXUstd::ToString(d));

		return false;
	}



	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! Set a general sized Matrix from a py::object safely and return false (if failed) and true if value has been set
	inline bool SetMatrixSafely(const py::object& value, Matrix& destination)
	{
		if (py::isinstance<py::list>(value))
		{
			std::vector<py::object> stdlist = py::cast<std::vector<py::object>>(value); //! # read out dictionary and cast to C++ type
			Index rows = (Index)stdlist.size();
			Index columns;
			for (Index i = 0; i < rows; i++)
			{
				if (py::isinstance<py::list>(stdlist[i]) || py::isinstance<py::array>(stdlist[i]))
				{
					std::vector<Real> rowVector = py::cast<std::vector<Real>>(stdlist[i]);
					if (i == 0) 
					{ 
						columns = (Index)rowVector.size();
						destination.SetNumberOfRowsAndColumns(rows, columns);
					}
					if ((Index)rowVector.size() == columns)
					{
						for (Index j = 0; j < columns; j++)
						{
							destination(i, j) = rowVector[j];
						}
					}
					else
					{
						PyError("Matrix size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!');
					}
				}
				else
				{
					PyError("Matrix in illegal format!");
				}
			}
			return true;
		}
		else if (py::isinstance<py::array>(value))
		{
			std::vector<py::object> stdlist = py::cast<std::vector<py::object>>(value); //! # read out dictionary and cast to C++ type
			Index rows = (Index)stdlist.size();
			Index columns;
			for (Index i = 0; i < rows; i++)
			{
				std::vector<Real> rowVector = py::cast<std::vector<Real>>(stdlist[i]);
				if (i == 0) 
				{ 
					columns = (Index)rowVector.size(); 
					destination.SetNumberOfRowsAndColumns(rows, columns);
				}
				if ((Index)rowVector.size() == columns)
				{
					for (Index j = 0; j < columns; j++)
					{
						destination(i, j) = rowVector[j];
					}
				}
				else
				{
					PyError("Matrix size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!');
				}
			}
			return true;
		}
		PyError(STDstring("failed to convert to Matrix: " + py::cast<std::string>(value)));
		return false;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! convert pyObject as list of lists, list of arrays or 2D numpy array into destArray; works for SlimVector and SlimArray
	//! Titem can be Real or Index
	template<class TArray, class Titem>
	inline bool SetListOfArraysSafely(const py::object& pyObject, ResizableArray<TArray>& destArray)
	{
		TArray test;
		Index columns = test.NumberOfItems();
		if (py::isinstance<py::list>(pyObject))
		{
			std::vector<py::object> stdlist = py::cast<std::vector<py::object>>(pyObject); //! # read out dictionary and cast to C++ type
			Index rows = (Index)stdlist.size();
			destArray.SetNumberOfItems(rows);
			for (Index i = 0; i < rows; i++)
			{
				if (py::isinstance<py::list>(stdlist[i]) || py::isinstance<py::array>(stdlist[i]))
				{
					std::vector<Titem> rowVector = py::cast<std::vector<Titem>>(stdlist[i]);
					if ((Index)rowVector.size() == columns)
					{
						for (Index j = 0; j < columns; j++)
						{
							destArray[i][j] = rowVector[j];
						}
					}
					else
					{
						PyError("List of arrays size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!');
					}
				}
				else
				{
					PyError("List of arrays with illegal format!");
				}
			}
			return true;
		}
		else if (py::isinstance<py::array>(pyObject))
		{
			std::vector<py::object> stdlist = py::cast<std::vector<py::object>>(pyObject); //! # read out dictionary and cast to C++ type
			Index rows = (Index)stdlist.size();
			destArray.SetNumberOfItems(rows);
			for (Index i = 0; i < rows; i++)
			{
				std::vector<Titem> rowVector = py::cast<std::vector<Titem>>(stdlist[i]);
				if ((Index)rowVector.size() == columns)
				{
					for (Index j = 0; j < columns; j++)
					{
						destArray[i][j] = rowVector[j];
					}
				}
				else
				{
					PyError("List of arrays size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!');
				}
			}
			return true;
		}
		PyError(STDstring("Failed to convert to list of arrays: " + py::cast<std::string>(pyObject)));
		return false;
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions for py::object safe conversion:



	inline bool SetVector2DSafely(const py::object& value, Vector2D& destination) {
		return SetSlimVectorTemplateSafely<Real,2>(value, destination);
	}
	inline bool SetVector3DSafely(const py::object& value, Vector3D& destination) {
		return SetSlimVectorTemplateSafely<Real, 3>(value, destination);
	}
	inline bool SetVector4DSafely(const py::object& value, Vector4D& destination) {
		return SetSlimVectorTemplateSafely<Real, 4>(value, destination);
	}
	inline bool SetVector6DSafely(const py::object& value, Vector6D& destination) {
		return SetSlimVectorTemplateSafely<Real, 6>(value, destination);
	}
	inline bool SetVector7DSafely(const py::object& value, Vector7D& destination) {
		return SetSlimVectorTemplateSafely<Real, 7>(value, destination);
	}

	template<class T>
	inline bool SetResizableArraySafely(const py::object& value, ResizableArray<T>& destination)
	{
		if (py::isinstance<py::list>(value) || py::isinstance<py::array>(value))
		{
			std::vector<T> stdlist = py::cast<std::vector<T>>(value); //! # read out dictionary and cast to C++ type
			destination = stdlist;
			return true;
		}
		PyError(STDstring("failed to convert array to ResizableArray: " + py::cast<std::string>(value)));
		return false;
	}

	template<class T, Index size>
	inline bool SetSlimArraySafely(const py::object& value, SlimArray<T, size>& destination)
	{
		if (py::isinstance<py::list>(value) || py::isinstance<py::array>(value))
		{
			std::vector<T> stdlist = py::cast<std::vector<T>>(value); //! # read out dictionary and cast to C++ type
			if ((Index)stdlist.size() == size)
			{
				destination = stdlist;
				return true;
			}
			else
			{
				PyError("SlimArray" + EXUstd::ToString(size) + " size mismatch: expected " + EXUstd::ToString(size) + " items in list or numpy array!");
			}
		}
		PyError(STDstring("failed to convert to SlimArray" + EXUstd::ToString(size) + ": " + py::cast<std::string>(value)));
		return false;
	}


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//!convert Vector to numpy vector
	template<typename TVector>
	inline py::array_t<Real> VectorRef2NumPy(const TVector& v, bool reference)
	{
		if (reference)
		{
			return py::array_t<Real>(
				{ v.NumberOfItems() },	//shape of the array (1D array with 'len' elements)
				{ sizeof(double) },		//stride (size of one element)
				v.GetDataPointer(),		//pointer to the data
				py::none()				//none(): pybind11 does not own the memory
			);
		} 
		else
		{
			//copy data using this constructor:
			return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
		}
	}



	//!convert ArrayIndex to numpy vector; COPY
	inline py::array_t<Index> ArrayIndex2NumPy(const ArrayIndex& v)
	{
		return py::array_t<Index>(v.NumberOfItems(), v.GetDataPointer()); 
	}

	//!convert SlimVector to numpy vector; COPY
	template<Index dataSize>
	inline py::array_t<Index> SlimArrayIndex2NumPy(const SlimArray<Index, dataSize>& v)
	{
		return py::array_t<Index>(v.NumberOfItems(), v.GetDataPointer());
	}



	//!convert Matrix to numpy matrix; COPY
	template<class TMatrix>
	py::array_t<float> MatrixF2NumPyTemplate(const TMatrix& matrix)
	{
		return py::array_t<float>(std::vector<std::ptrdiff_t>{(int)matrix.NumberOfRows(), (int)matrix.NumberOfColumns()}, matrix.GetDataPointer());
	}




	//!convert MatrixF to numpy matrix; COPY
	inline py::array_t<float> MatrixF2NumPy(const MatrixF& matrix)
	{
		return py::array_t<float>(std::vector<std::ptrdiff_t>{(int)matrix.NumberOfRows(), (int)matrix.NumberOfColumns()}, matrix.GetDataPointer());
	}




	//!convert numpy matrix to Matrix
	inline Matrix NumPy2Matrix(const py::array_t<Real>& pyArray)
	{
		Matrix m;
		NumPy2Matrix(pyArray, m);
		return m;
	}

	//!convert numpy matrix to Matrix
	inline MatrixI NumPy2MatrixI(const py::array_t<Index>& pyArray)
	{
		MatrixI m;
		NumPy2Matrix(pyArray, m);
		return m;
	}

	//!convert numpy matrix to ResizableMatrix
	inline ResizableMatrix NumPy2ResizableMatrix(const py::array_t<Real>& pyArray)
	{
		ResizableMatrix m;
		NumPy2Matrix(pyArray, m);
		return m;
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//numpy conversions
	template<typename T>
	inline bool SetNumpyMatrixSafelyTemplate(const py::dict& d, const char* itemName, MatrixBase<T>& destination)
	{
		if (d.contains(itemName))
		{
			py::object other = d[itemName]; //this is necessary to make isinstance work
			SetNumpyMatrixSafelyTemplate<T, MatrixBase<T>>(other, destination); //includes silent conversion from Real (e.g. for ANCFPlate)

			//NumPy2Matrix<T>(py::cast<py::array_t<T>>(other), destination);
			return true;
		}
		PyError(STDstring("ERROR: failed to convert '") + itemName + "' (expected: numpy matrix) into Matrix; dictionary:\n" + EXUstd::ToString(d));
		return false;
	}

	inline bool SetNumpyMatrixISafely(const py::dict& d, const char* itemName, MatrixI& destination)
	{
		return SetNumpyMatrixSafelyTemplate<Index>(d, itemName, destination);
	}

	inline bool SetNumpyMatrixSafely(const py::dict& d, const char* itemName, Matrix& destination)
	{
		return SetNumpyMatrixSafelyTemplate<Real>(d, itemName, destination);
	}



	inline bool SetNumpyVectorSafely(const py::dict& d, const char* itemName, Vector& destination)
	{
		if (d.contains(itemName))
		{
			py::object other = d[itemName]; //this is necessary to make isinstance work

			return SetNumpyVectorSafelyTemplate<Real, Vector>(other, destination);
		}
		PyError(STDstring("ERROR: failed to convert '") + itemName + "' (expected: numpy vector) into Vector; dictionary:\n" + EXUstd::ToString(d));
		return false;
	}

	template<class TMatrix>
	inline bool SetNumpyMatrixSafely(const py::object& value, TMatrix& destination)
	{
		return SetNumpyMatrixSafelyTemplate<Real, TMatrix>(value, destination);
	}
	inline bool SetNumpyMatrixISafely(const py::object& value, MatrixI& destination)
	{
		return SetNumpyMatrixSafelyTemplate<Index, MatrixI>(value, destination);
		//NumPy2Matrix<Index>(py::cast<py::array_t<Index>>(value), destination);
		//return true;
	}

	inline bool SetNumpyVectorSafely(const py::object& value, Vector& destination)
	{
		return SetNumpyVectorSafelyTemplate<Real, Vector>(value, destination);
		//NumPy2Vector(py::cast<py::array_t<Real>>(value), destination);
		//return true;
	}



} //namespace HPyUtils

#endif
