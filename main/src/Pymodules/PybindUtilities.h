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
* 				- weblink: missing
* 				
*
************************************************************************************************ */
#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
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

	inline bool CheckForValidFunction(const py::object pyObject)
	{
		if (py::isinstance<py::function>(pyObject))
		{
			return true;
		}
		else if (py::isinstance<py::int_>(pyObject))
		{
			if (py::cast<int>(pyObject) != 0) 
			{ 
				PyError(STDstring("Failed to convert PyFunction: must be either valid python function or 0, but got ")+EXUstd::ToString(pyObject)); 
			}
			return false; //this is a valid value, but no function (0-function pointer means empty function (in C++: nullptr))
		}
		else
		{
			PyError(STDstring("Failed to convert PyFunction: must be either valid py::function or int, but got ")+ EXUstd::ToString(pyObject));
		}
		return false;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! assign a string 'item' of a dictionary 'd' safely to 'str' and return false (if failed) and 1 of value has been set
	inline bool SetStringSafely(const py::dict& d, const char* itemName, STDstring& destination)
	{
		if (d.contains(itemName))
		{
			py::object other = d[itemName]; //this is necessary to make isinstance work
			if (py::isinstance<py::str>(other))
			{
				destination = py::cast<std::string>(other); //! read out dictionary and cast to C++ type
				return true;
			}
		}
		PyError(STDstring("ERROR: failed to convert '") + itemName + "' into string; dictionary:\n" + EXUstd::ToString(d));
		return false;
		//pout << "ERROR: failed to convert '" << itemName << "' into string; dictionary:\n";
		//pout << d << "\n\n";
		//return false;
	}
	template<Index size>
	inline bool SetVectorTemplateSafely(const py::dict& d, const char* item, SlimVector<size>& destination)
	{
		if (d.contains(item))
		{
			py::object other = d[item]; //this is necessary to make isinstance work
			if (py::isinstance<py::list>(other))
			{
				std::vector<Real> stdlist = py::cast<std::vector<Real>>(other); //! # read out dictionary and cast to C++ type
				if (stdlist.size() == size)
				{
					destination = stdlist;
					return true;
				}
				else
				{
					PyError("Vector" + EXUstd::ToString(size) + "D size mismatch: expected " + EXUstd::ToString(size) + " items in list!");
				}
			}
		}
		PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector" + EXUstd::ToString(size) + "D; dictionary:\n" + EXUstd::ToString(d));
		return false;
	}

	inline bool SetVector2DSafely(const py::dict& d, const char* item, Vector2D& destination) {
		return SetVectorTemplateSafely<2>(d, item, destination); }

	inline bool SetVector3DSafely(const py::dict& d, const char* item, Vector3D& destination) {
		return SetVectorTemplateSafely<3>(d, item, destination);}

	inline bool SetVector4DSafely(const py::dict& d, const char* item, Vector4D& destination) {
		return SetVectorTemplateSafely<4>(d, item, destination);}

	inline bool SetVector6DSafely(const py::dict& d, const char* item, Vector6D& destination) {
		return SetVectorTemplateSafely<6>(d, item, destination);}

	inline bool SetVector7DSafely(const py::dict& d, const char* item, Vector7D& destination) {
		return SetVectorTemplateSafely<7>(d, item, destination);}


	//inline bool SetVector2DSafely(const py::dict& d, const char* item, Vector2D& destination)
	//{
	//	if (d.contains(item))
	//	{
	//		py::object other = d[item]; //this is necessary to make isinstance work
	//		if (py::isinstance<py::list>(other))
	//		{
	//			std::vector<Real> stdlist = py::cast<std::vector<Real>>(other); //! # read out dictionary and cast to C++ type
	//			if (stdlist.size() == 2)
	//			{
	//				destination = stdlist;
	//				return true;
	//			}
	//			else
	//			{
	//				PyError("Vector2D size mismatch: expected 2 items in list!");
	//			}
	//		}
	//	}
	//	PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector2D; dictionary:\n" + EXUstd::ToString(d));
	//	return false;
	//}

	//inline bool SetVector3DSafely(const py::dict& d, const char* item, Vector3D& destination)
	//{
	//	if (d.contains(item))
	//	{
	//		py::object other = d[item]; //this is necessary to make isinstance work
	//		if (py::isinstance<py::list>(other))
	//		{
	//			std::vector<Real> stdlist = py::cast<std::vector<Real>>(other); //! # read out dictionary and cast to C++ type
	//			if (stdlist.size() == 3)
	//			{
	//				destination = stdlist;
	//				return true;
	//			}
	//			{
	//				PyError("Vector3D size mismatch: expected 3 items in list!");
	//				//pout << "ERROR: Vector3D size mismatch: expected 3 items in list!\n";
	//			}
	//		}
	//	}
	//	PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector3D; dictionary:\n" + EXUstd::ToString(d));
	//	//pout << "ERROR: failed to convert '" << item << "' into Vector3D; dictionary:\n";
	//	//pout << d << "\n\n";
	//	return false;
	//}

	//inline bool SetVector4DSafely(const py::dict& d, const char* item, Vector4D& destination)
	//{
	//	if (d.contains(item))
	//	{
	//		py::object other = d[item]; //this is necessary to make isinstance work
	//		if (py::isinstance<py::list>(other))
	//		{
	//			std::vector<Real> stdlist = py::cast<std::vector<Real>>(other); //! # read out dictionary and cast to C++ type
	//			if (stdlist.size() == 4)
	//			{
	//				destination = stdlist;
	//				return true;
	//			}
	//			{
	//				PyError("Vector4D size mismatch: expected 4 items in list!");
	//				//pout << "ERROR: Vector4D size mismatch: expected 4 items in list!\n";
	//			}
	//		}
	//	}
	//	PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector4D; dictionary:\n" + EXUstd::ToString(d));
	//	//pout << "ERROR: failed to convert '" << item << "' into Vector4D; dictionary:\n";
	//	//pout << d << "\n\n";
	//	return false;
	//}

	//inline bool SetVector6DSafely(const py::dict& d, const char* item, Vector6D& destination)
	//{
	//	if (d.contains(item))
	//	{
	//		py::object other = d[item]; //this is necessary to make isinstance work
	//		if (py::isinstance<py::list>(other))
	//		{
	//			std::vector<Real> stdlist = py::cast<std::vector<Real>>(other); //! # read out dictionary and cast to C++ type
	//			if (stdlist.size() == 6)
	//			{
	//				destination = stdlist;
	//				return true;
	//			}
	//			{
	//				PyError("Vector6D size mismatch: expected 6 items in list!");
	//				//pout << "ERROR: Vector6D size mismatch: expected 4 items in list!\n";
	//			}
	//		}
	//	}
	//	PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector6D; dictionary:\n" + EXUstd::ToString(d));
	//	//pout << "ERROR: failed to convert '" << item << "' into Vector6D; dictionary:\n";
	//	//pout << d << "\n\n";
	//	return false;
	//}

	//inline bool SetVector7DSafely(const py::dict& d, const char* item, Vector7D& destination)
	//{
	//	if (d.contains(item))
	//	{
	//		py::object other = d[item]; //this is necessary to make isinstance work
	//		if (py::isinstance<py::list>(other))
	//		{
	//			std::vector<Real> stdlist = py::cast<std::vector<Real>>(other); //! # read out dictionary and cast to C++ type
	//			if (stdlist.size() == 7)
	//			{
	//				destination = stdlist;
	//				return true;
	//			}
	//			{
	//				PyError("Vector7D size mismatch: expected 7 items in list!");
	//				//pout << "ERROR: Vector7D size mismatch: expected 4 items in list!\n";
	//			}
	//		}
	//	}
	//	PyError(STDstring("ERROR: failed to convert '") + item + "' into Vector7D; dictionary:\n" + EXUstd::ToString(d));
	//	//pout << "ERROR: failed to convert '" << item << "' into Vector7D; dictionary:\n";
	//	//pout << d << "\n\n";
	//	return false;
	//}

	//! Set a Matrix6D from a py::object safely and return false (if failed) and true if value has been set
	template<Index rows, Index columns>
	inline bool SetMatrixTemplateSafely(const py::object& value, ConstSizeMatrix<rows*columns>& destination)
	{
		if (py::isinstance<py::list>(value))
		{
			std::vector<py::object> stdlist = py::cast<std::vector<py::object>>(value); //! # read out dictionary and cast to C++ type
			if (stdlist.size() == rows)
			{
				for (Index i = 0; i < rows; i++)
				{
					if (py::isinstance<py::list>(stdlist[i]))
					{
						std::vector<Real> rowVector = py::cast<std::vector<Real>>(stdlist[i]);
						if (rowVector.size() == columns)
						{
							for (Index j = 0; j < columns; j++)
							{
								destination(i, j) = rowVector[j];
							}
						}
					}
					else
					{
						PyError("Matrix size mismatch: expected " + EXUstd::ToString(columns) + " columns in row " + EXUstd::ToString(i) + '!');
					}
				}
				return true;
			}
			else
			{
				PyError("Matrix size mismatch: expected " + EXUstd::ToString(rows) + " rows!");
				//pout << "ERROR: Vector7D size mismatch: expected 4 items in list!\n";
			}
		}
		else if (py::isinstance<py::array>(value))
		{
			std::vector<py::object> stdlist = py::cast<std::vector<py::object>>(value); //! # read out dictionary and cast to C++ type
			if (stdlist.size() == rows)
			{
				for (Index i = 0; i < rows; i++)
				{
					std::vector<Real> rowVector = py::cast<std::vector<Real>>(stdlist[i]);
					if (rowVector.size() == columns)
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
			else
			{
				PyError("Matrix size mismatch: expected " + EXUstd::ToString(rows) + " rows!");
				//pout << "ERROR: Vector7D size mismatch: expected 4 items in list!\n";
			}
		}
		PyError(STDstring("failed to convert to Matrix: " + py::cast<std::string>(value)));
		return false;
	}

	template<Index rows, Index columns>
	inline bool SetMatrixTemplateSafely(const py::dict& d, const char* item, ConstSizeMatrix<rows*columns>& destination)
	{

		if (d.contains(item))
		{
			py::object other = d[item]; //this is necessary to make isinstance work
			return SetMatrixTemplateSafely<rows,columns>(other, destination);
		}
		PyError(STDstring("ERROR: failed to convert '") + item + "' into Matrix; dictionary:\n" + EXUstd::ToString(d));

		return false;
	}

	inline bool SetMatrix6DSafely(const py::object& value, Matrix6D& destination) 
	{
		return SetMatrixTemplateSafely<6, 6>(value, destination);
	}

	inline bool SetMatrix6DSafely(const py::dict& d, const char* item, Matrix6D& destination) 
	{
		return SetMatrixTemplateSafely<6, 6>(d, item, destination);
	}

	inline bool SetMatrix3DSafely(const py::object& value, Matrix3D& destination) 
	{
		return SetMatrixTemplateSafely<3, 3>(value, destination);
	}

	inline bool SetMatrix3DSafely(const py::dict& d, const char* item, Matrix3D& destination) 
	{
		return SetMatrixTemplateSafely<3, 3>(d, item, destination);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions for py::object safe conversion:

	//! assign a string of a py::object safely to 'str' and return false (if failed) and 1 of value has been set
	inline bool SetStringSafely(const py::object& value, STDstring& destination)
	{
		if (py::isinstance<py::str>(value))
		{
			destination = py::cast<std::string>(value); //! read out dictionary and cast to C++ type
			return true;
		}
		//PyError(STDstring("failed to convert to string: " + py::str(value)));
		PyError(STDstring("failed to convert to string: " + py::cast<std::string>(value)));
		return false;
	}

	template<Index size>
	inline bool SetVectorTemplateSafely(const py::object& value, SlimVector<size>& destination)
	{
		if (py::isinstance<py::list>(value))
		{
			std::vector<Real> stdlist = py::cast<std::vector<Real>>(value); //! # read out dictionary and cast to C++ type
			if (stdlist.size() == size)
			{
				destination = stdlist;
				return true;
			}
			else
			{
				PyError("Vector" + EXUstd::ToString(size) + "D size mismatch: expected " + EXUstd::ToString(size) + " items in list!");
			}
		}
		PyError(STDstring("failed to convert Vector" + EXUstd::ToString(size) + ": " + py::cast<std::string>(value)));
		return false;
	}

	inline bool SetVector2DSafely(const py::object& value, Vector2D& destination) {
		return SetVectorTemplateSafely<2>(value, destination);
	}
	inline bool SetVector3DSafely(const py::object& value, Vector3D& destination) {
		return SetVectorTemplateSafely<3>(value, destination);
	}
	inline bool SetVector4DSafely(const py::object& value, Vector4D& destination) {
		return SetVectorTemplateSafely<4>(value, destination);
	}
	inline bool SetVector6DSafely(const py::object& value, Vector6D& destination) {
		return SetVectorTemplateSafely<6>(value, destination);
	}
	inline bool SetVector7DSafely(const py::object& value, Vector7D& destination) {
		return SetVectorTemplateSafely<7>(value, destination);
	}

	////! Set a Vector2D from a py::object safely and return false (if failed) and 1 of value has been set
	//inline bool SetVector2DSafely(const py::object& value, Vector2D& destination)
	//{
	//	if (py::isinstance<py::list>(value))
	//	{
	//		std::vector<Real> stdlist = py::cast<std::vector<Real>>(value); //! # read out dictionary and cast to C++ type
	//		if (stdlist.size() == 2)
	//		{
	//			destination = stdlist;
	//			return true;
	//		}
	//		else
	//		{
	//			PyError("Vector2D size mismatch: expected 2 items in list!");
	//		}
	//	}
	//	PyError(STDstring("failed to convert to Vector2D: " + py::cast<std::string>(value)));
	//	return false;
	//}

	////! Set a Vector3D from a py::object safely and return false (if failed) and 1 of value has been set
	//inline bool SetVector3DSafely(const py::object& value, Vector3D& destination)
	//{
	//	if (py::isinstance<py::list>(value))
	//	{
	//		std::vector<Real> stdlist = py::cast<std::vector<Real>>(value); //! # read out dictionary and cast to C++ type
	//		if (stdlist.size() == 3)
	//		{
	//			destination = stdlist;
	//			return true;
	//		}
	//		{
	//			PyError("Vector3D size mismatch: expected 3 items in list!");
	//		}
	//	}
	//	PyError(STDstring("failed to convert to Vector3D: " + py::cast<std::string>(value)));
	//	return false;
	//}

	////! Set a Vector4D from a py::object safely and return false (if failed) and 1 of value has been set
	//inline bool SetVector4DSafely(const py::object& value, Vector4D& destination)
	//{
	//	if (py::isinstance<py::list>(value))
	//	{
	//		std::vector<Real> stdlist = py::cast<std::vector<Real>>(value); //! # read out dictionary and cast to C++ type
	//		if (stdlist.size() == 4)
	//		{
	//			destination = stdlist;
	//			return true;
	//		}
	//		{
	//			PyError("Vector4D size mismatch: expected 4 items in list!");
	//		}
	//	}
	//	PyError(STDstring("failed to convert to Vector4D: " + py::cast<std::string>(value)));
	//	return false;
	//}

	////! Set a Vector6D from a py::object safely and return false (if failed) and 1 of value has been set
	//inline bool SetVector6DSafely(const py::object& value, Vector6D& destination)
	//{
	//	if (py::isinstance<py::list>(value))
	//	{
	//		std::vector<Real> stdlist = py::cast<std::vector<Real>>(value); //! # read out dictionary and cast to C++ type
	//		if (stdlist.size() == 6)
	//		{
	//			destination = stdlist;
	//			return true;
	//		}
	//		{
	//			PyError("Vector6D size mismatch: expected 6 items in list!");
	//		}
	//	}
	//	PyError(STDstring("failed to convert to Vector6D: " + py::cast<std::string>(value)));
	//	return false;
	//}

	////! Set a Vector7D from a py::object safely and return false (if failed) and 1 of value has been set
	//inline bool SetVector7DSafely(const py::object& value, Vector7D& destination)
	//{
	//	if (py::isinstance<py::list>(value))
	//	{
	//		std::vector<Real> stdlist = py::cast<std::vector<Real>>(value); //! # read out dictionary and cast to C++ type
	//		if (stdlist.size() == 7)
	//		{
	//			destination = stdlist;
	//			return true;
	//		}
	//		{
	//			PyError("Vector7D size mismatch: expected 7 items in list!");
	//		}
	//	}
	//	PyError(STDstring("failed to convert to Vector7D: " + py::cast<std::string>(value)));
	//	return false;
	//}


	inline py::array_t<Real> PyVector(const Vector& v)
	{
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer()); //copy array (could also be referenced!)
	}

	inline py::array_t<Real> PyVector(const SlimVector<3>& v)
	{
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer()); //copy array (could also be referenced!)
	}

	inline py::array_t<Real> Matrix2NumPy(const Matrix& matrix)
	{
		return py::array_t<Real>(std::vector<std::ptrdiff_t>{(int)matrix.NumberOfRows(), (int)matrix.NumberOfColumns()}, matrix.GetDataPointer());
	}

	inline void NumPy2Matrix(const py::array_t<Real>& pyArray, Matrix& m)
	{
		if (pyArray.ndim() == 2)
		{
			auto mat = pyArray.unchecked<2>();
			Index nrows = mat.shape(0);
			Index ncols = mat.shape(1);

			m.SetNumberOfRowsAndColumns(nrows, ncols);
			for (Index i = 0; i < nrows; i++)
			{
				for (Index j = 0; j < ncols; j++)
				{
					m(i, j) = mat(i, j);
				}
			}
		}
		else
		{
			SysError("failed to convert numpy array to matrix: array must have dimension 2 (rows x columns)");
		}
	}

	inline Matrix NumPy2Matrix(const py::array_t<Real>& pyArray)
	{
		Matrix m;
		NumPy2Matrix(pyArray, m);
		return m;
	}

	inline ResizableMatrix NumPy2ResizableMatrix(const py::array_t<Real>& pyArray)
	{
		ResizableMatrix m;
		NumPy2Matrix(pyArray, m);
		return m;
	}


} //namespace HPyUtils

