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
#include "Pymodules/PyConversion.h"  //FromPython / ToPython (revision2026 step R4.4.3)

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


	


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
				CHECKandTHROWstring( (STDstring(info)+": parameter must be either a Python function or 0 but received: "+EXUstd::ToString(function)).c_str() , ExudynTypeError);
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
			CHECKandTHROWstring((STDstring(info) + ": parameter must be either a Python function or 0 but received '" + EXUstd::ToString(function)+"'").c_str(), ExudynTypeError);
		}
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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
						PyError("Matrix size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!', PyErrorType::valueError);
					}
				}
				else
				{
					PyError("Matrix in illegal format!", PyErrorType::typeError);
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
					PyError("Matrix size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!', PyErrorType::valueError);
				}
			}
			return true;
		}
		PyError(STDstring("failed to convert to Matrix: " + py::cast<std::string>(value)), PyErrorType::typeError);
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
						PyError("List of arrays size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!', PyErrorType::valueError);
					}
				}
				else
				{
					PyError("List of arrays with illegal format!", PyErrorType::typeError);
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
					PyError("List of arrays size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!', PyErrorType::valueError);
				}
			}
			return true;
		}
		PyError(STDstring("Failed to convert to list of arrays: " + py::cast<std::string>(pyObject)), PyErrorType::typeError);
		return false;
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions for py::object safe conversion:


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
				PyError("SlimArray" + EXUstd::ToString(size) + " size mismatch: expected " + EXUstd::ToString(size) + " items in list or numpy array!", PyErrorType::valueError);
			}
		}
		PyError(STDstring("failed to convert to SlimArray" + EXUstd::ToString(size) + ": " + py::cast<std::string>(value)), PyErrorType::typeError);
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


	//!convert numpy matrix to Matrix
	inline Matrix NumPy2Matrix(const py::array_t<Real>& pyArray)
	{
		Matrix m;
		FromPython(pyArray, m);
		return m;
	}


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


} //namespace HPyUtils

#endif
