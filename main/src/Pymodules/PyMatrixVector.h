/** ***********************************************************************************************
* @brief		binding to some Exudyn Matrix and Vector classes,
*               especially to lists of Exudyn ConstSizeVector and ConstSizeMatrix for interaction with Python
*
* @author		Gerstmayr Johannes
* @date			2022-04-24 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */

#ifndef PYMATRIXVECTOR__H
#define PYMATRIXVECTOR__H

#include "Linalg/BasicLinalg.h"	
#include "Pymodules/PybindUtilities.h"	

//! a list of SlimVectors for interaction with Python (used in KinematicTree)
template<Index dataSize>
class PyVectorList : public VectorList<dataSize>
{

public:
	//! create empty (dense) container
	//?remove default constructor to enable conversion from py::object in constructor?
	PyVectorList() : VectorList <dataSize>() {}

	PyVectorList(const VectorList <dataSize>& other) : VectorList<dataSize>(other) {}

	//! initialize array with list of py::array or with emtpy list (default value)
	PyVectorList(const py::object& listOfArrays)
	{
		if (py::isinstance<py::list>(listOfArrays))
		{
			py::list pyList = py::cast<py::list>(listOfArrays);
			this->SetMaxNumberOfItems((Index)pyList.size());

			//empty list does also work!
			for (const auto& item : pyList)
			{
				this->PyAppend((const py::object&)item);
			}
			return;
		}
		else
		{
			PyError(STDstring("Vector" + EXUstd::ToString(dataSize) + "DList: Expected list of " + EXUstd::ToString(dataSize) + "D numpy arrays, but received '" +
				EXUstd::ToString(listOfArrays) + "'"));
		}
	}

	//! operator for casting, only needed for linux / gcc
	operator const VectorList<dataSize>&() const { return (VectorList<dataSize>)(*this); }
	operator VectorList<dataSize>&() { return (VectorList<dataSize>)(*this); }

	//! set
	virtual void PySetItem(Index index, const py::object& pyArray)
	{
		if (index < 0 || index >= this->NumberOfItems())
		{
			PyError("Vector" + EXUstd::ToString(dataSize) + "DList::SetItem operator[]: trying to access list with size "+EXUstd::ToString(this->NumberOfItems()) +
				" with index "+ EXUstd::ToString(index));
		}
		else
		{
			try
			{
				if (py::isinstance<py::array>(pyArray))
				{
					this->GetItemUnsafe(index) = SlimVector<dataSize>(py::cast<std::array<Real, dataSize>>(pyArray));
				}
				else if (py::isinstance<py::list>(pyArray))
				{
					py::list pyList = py::cast<py::list>(pyArray);
					SlimVector<dataSize> v;
					if (pyList.size() == dataSize)
					{
						Index cnt = 0;
						for (auto item : pyList)
						{
							v[cnt] = py::cast<Real>(item);
							cnt++;
						}
						this->GetItemUnsafe(index) = SlimVector<dataSize>(py::cast<std::array<Real, dataSize>>(pyArray));
					}
					else
					{
						PyError(STDstring("Vector" + EXUstd::ToString(dataSize) + "DList::SetItem operator[]: Expected list of list of " + EXUstd::ToString(dataSize) + " floats, but item " + EXUstd::ToString(this->NumberOfItems()) +
							" is invalid: '" + EXUstd::ToString(pyArray) + "'"));
					}

				}
				else
				{
					PyError(STDstring("Vector" + EXUstd::ToString(dataSize) + "DList::SetItem operator[]: Expected list of " + EXUstd::ToString(dataSize) + "D numpy arrays, but item " + EXUstd::ToString(this->NumberOfItems()) +
						" is invalid: '" + EXUstd::ToString(pyArray) + "'"));
				}
			}
			//mostly catches python errors:
			catch (const pybind11::error_already_set& ex)
			{
				SysError("Error in Vector" + EXUstd::ToString(dataSize) + "DList::SetItem operator[]'" + STDstring(ex.what()) + "; check your Python code!");
			}

			catch (const EXUexception& ex)
			{
				SysError("Error in Vector" + EXUstd::ToString(dataSize) + "DList::SetItem operator[]'" + STDstring(ex.what()) + "; check your Python code!");
			}
			catch (...) //any other exception
			{
				SysError("Error in Vector" + EXUstd::ToString(dataSize) + "DList::SetItem operator[]; check your Python code!");
			}

		}
	}

	//! append single vector to list
	virtual void PyAppend(const py::object& pyArray)
	{
		Index i = this->Append(SlimVector<dataSize>(0.));
		PySetItem(i, pyArray);
	}


	//! return Python object as list of numpy arrays
	virtual py::object GetPythonObject() const //return list of numpy arrays
	{
		//this function is not fast, use only during initialization phase!
		auto list = py::list();

		for (const SlimVector<dataSize>& item : *this)
		{
			list.append(py::array_t<Real>(item.NumberOfItems(), item.GetDataPointer())); //gives weird results!!!
			//list.append((std::vector<Real>)(item)); //creates a list
		}
		return list;
	}

};

typedef  PyVectorList<2> PyVector2DList;
typedef  PyVectorList<3> PyVector3DList;
typedef  PyVectorList<6> PyVector6DList; //needs to be bound by pybind


//! a list of ConstSizeMatrices for interaction with Python (used in KinematicTree)
template<Index numberOfRowsColumns>
class PyMatrixList : public MatrixList<numberOfRowsColumns>
{

public:
	//! create empty (dense) container
	//?remove default constructor to enable conversion from py::object in constructor?
	PyMatrixList() : MatrixList<numberOfRowsColumns>() {}

	//!conversion from non-Python MatrixList:
	PyMatrixList(const MatrixList<numberOfRowsColumns>& other) : MatrixList<numberOfRowsColumns>(other) {}

	//! initialize array with list of py::array or with emtpy list (default value)
	PyMatrixList(const py::object& listOfArrays)
	{
		if (py::isinstance<py::list>(listOfArrays))
		{
			py::list pyList = py::cast<py::list>(listOfArrays);
			this->SetMaxNumberOfItems((Index)pyList.size());

			//empty list does also work!
			for (const auto& item : pyList)
			{
				this->PyAppend((const py::object&)item);
			}
			return;
		}
		else
		{
			PyError(STDstring("Matrix" + EXUstd::ToString(numberOfRowsColumns) + "DList: Expected list of " + EXUstd::ToString(numberOfRowsColumns) + "D numpy matrices, but received '" +
				EXUstd::ToString(listOfArrays) + "'"));
		}
	}

	virtual py::object PyGetItem(Index index) const
	{
		if (index < 0 || index >= this->NumberOfItems())
		{
			PyError("Matrix" + EXUstd::ToString(numberOfRowsColumns) + "DList::GetItem operator[]: trying to access list with size " +
				EXUstd::ToString(this->NumberOfItems()) + " with index " + EXUstd::ToString(index));
			return py::cast<int>(0);
		}
		else
		{
			return EPyUtils::Matrix2NumPyTemplate<ConstSizeMatrix<numberOfRowsColumns*numberOfRowsColumns>>(this->GetItemUnsafe(index));
		}
	}

	//! set
	virtual void PySetItem(Index index, const py::object& pyArray)
	{
		if (index < 0 || index >= this->NumberOfItems())
		{
			PyError("Matrix" + EXUstd::ToString(numberOfRowsColumns) + "DList::SetItem  operator[]: trying to access list with size " + 
				EXUstd::ToString(this->NumberOfItems()) + " with index " + EXUstd::ToString(index));
		}
		else
		{
			try
			{
				EPyUtils::SetConstMatrixTemplateSafely<numberOfRowsColumns, numberOfRowsColumns>(pyArray, this->GetItemUnsafe(index));
			}
			//mostly catches python errors:
			catch (const pybind11::error_already_set& ex)
			{
				SysError("Error in Matrix" + EXUstd::ToString(numberOfRowsColumns) + "DList::SetItem operator[]'" + STDstring(ex.what()) + "; check your Python code!");
			}

			catch (const EXUexception& ex)
			{
				SysError("Error in Matrix" + EXUstd::ToString(numberOfRowsColumns) + "DList::SetItem operator[]'" + STDstring(ex.what()) + "; check your Python code!");
			}
			catch (...) //any other exception
			{
				SysError("Error in Matrix" + EXUstd::ToString(numberOfRowsColumns) + "DList::SetItem operator[]; check your Python code!");
			}

		}
	}

	//! append single Matrix to list
	virtual void PyAppend(const py::object& pyArray)
	{
		Index i = this->Append(ConstSizeMatrix<numberOfRowsColumns*numberOfRowsColumns>(numberOfRowsColumns, numberOfRowsColumns,0.));
		PySetItem(i, pyArray);
	}


	//! return Python object as list of numpy arrays
	virtual py::object GetPythonObject() const //return list of numpy arrays
	{
		//this function is not fast, use only during initialization phase!
		auto list = py::list();

		for (const ConstSizeMatrix<numberOfRowsColumns*numberOfRowsColumns>& item : *this)
		{
			//list.append(py::array_t<Real>(item.NumberOfItems(), item.GetDataPointer())); //gives weird results!!!
			list.append(EPyUtils::Matrix2NumPyTemplate<ConstSizeMatrix<numberOfRowsColumns*numberOfRowsColumns>>(item));
		}
		return list;
	}

};

typedef  PyMatrixList<3> PyMatrix3DList;
typedef  PyMatrixList<6> PyMatrix6DList; 
typedef  PyMatrixList<6> PyTransformation66List; //needs to be bound by pybind

//! add to EPyUtils here as otherwise cyclic inclusion between PyMatrixVector.h and PybindUtilities (makes problems with gcc)
namespace EPyUtils
{
	template<class TPyList, class TList, Index size, bool isVector>
	bool SetMatrixVectorListSafely(const py::object& value, TList& destination)
	//bool SetMatrixVectorListSafely(py::object value, TList& destination)
	{
		bool rv = false;
		STDstring sType = "Matrix";
		if (isVector) { sType = "Vector"; }
		STDstring listType = sType + EXUstd::ToString(size) + "DList";

		GenericExceptionHandling([&]
		{
			if (py::isinstance<py::list>(value))
			{
				py::list pylist = py::cast<py::list>(value); //also works for numpy arrays (but gives different type!)

				destination.Flush();
				if (pylist.size() != 0)
				{
					rv = false;
					PyError(STDstring("Set " + listType + ": Either empty list [] or " + listType + " allowed, but received: ") +
						STDstring(py::str(value))); //here we do not use py::cast<std::string>(value), because value may be Vector3DList directly, which cannot be casted to Python!
				}
				rv = true;
			}
			else if (py::isinstance<TPyList>(value)) //the py instance is e.g. PyVector3DList, but it is casted to the pure C++ type Vector3DList
			{
#ifdef __EXUDYN__WINDOWS__
				destination = (TList&)(py::cast<TPyList>(value)); //only casting necessary!
#else
				//gcc (Ubuntu18.04) has problems to cast templates ...
				TPyList pyList = (py::cast<TPyList>(value));
				destination = (TList&)(pyList); //only casting necessary!
#endif
				rv = true;
			}
			else
			{
				rv = false;
				PyError(STDstring("Set " + listType + ": Either empty list [] or " + listType + " allowed, but received: ") +
					STDstring(py::str(value))); //here we do not use py::cast<std::string>(value), because value may be Vector3DList directly, which cannot be casted to Python!
			}
		}, "Set [Vector/Matrix][3/6]DList");
		if (rv) { return true; }
		else
		{
			PyError(STDstring("Set " + listType + " failed when received: ") +
				STDstring(py::str(value))); //here we do not use py::cast<std::string>(value), because value may be Vector3DList directly, which cannot be casted to Python!
			return false;
		}
	}

	inline bool SetVector3DListSafely(const py::object& value, VectorList<3>& destination) {
		return SetMatrixVectorListSafely<PyVector3DList, VectorList<3>, 3, true>(value, destination);
	}
	inline bool SetMatrix3DListSafely(const py::object& value, MatrixList<3>& destination) {
		return SetMatrixVectorListSafely<PyMatrix3DList, MatrixList<3>, 3, false>(value, destination);
	}

	inline bool SetVector3DListSafely(const py::dict& d, const char* item, Vector3DList& destination)
	{
		if (d.contains(item))
		{
			py::object other = d[item]; //this is necessary to make isinstance work
			return SetVector3DListSafely(other, destination);
		}
		PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector3DList; dictionary:\n" + EXUstd::ToString(d));
		return false;
	}

	inline bool SetMatrix3DListSafely(const py::dict& d, const char* item, MatrixList<3>& destination)
	{
		if (d.contains(item))
		{
			py::object other = d[item]; //this is necessary to make isinstance work
			return SetMatrix3DListSafely(other, destination);
		}
		PyError(STDstring("ERROR: failed to convert '") + item + "' into Matrix3DList; dictionary:\n" + EXUstd::ToString(d));
		return false;
	}

};


#endif
