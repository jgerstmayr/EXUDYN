/** ***********************************************************************************************
* @brief		Implementation of Vector
*
* @author		Gerstmayr Johannes
* @date			1997-05-15 (generated)
* @date			2019-05-01 (last modified)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use Vector for large vector sizes; uses dynamic allocation (slow)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @todo         Vector::operators, check if data[cnt++] works and is faster
*
************************************************************************************************ */

#pragma once

//#include <initializer_list> //for initializer_list in constructor
//#include <ostream>
//#include <stdlib.h> // for initialization with std::vector
//#include <vector>
//#include "Utilities/ReleaseAssert.h"
//#include "Utilities/BasicDefinitions.h" //defines Real
#include "Linalg/Vector.h" 
#include "Linalg/SlimVector.h" 

//put this into ResizableArray.cpp
#ifdef __EXUDYN_RUNTIME_CHECKS__
Index array_new_counts = 0; //global counter of item allocations; is increased every time a new is called
Index array_delete_counts = 0; //global counter of item deallocations; is increased every time a delete is called
#endif

#ifdef __EXUDYN_RUNTIME_CHECKS__
Index vector_new_counts = 0; //global counter of item allocations; is increased every time a new is called
Index vector_delete_counts = 0; //global counter of item deallocations; is increased every time a delete is called
#endif

//put this into Matrix.cpp
#ifdef __EXUDYN_RUNTIME_CHECKS__
Index matrix_new_counts = 0; //global counter of item allocations; is increased every time a new is called
Index matrix_delete_counts = 0; //global counter of item deallocations; is increased every time a delete is called
#endif


//! add two vectors, result = v1+v2 (for each component)
template<typename T>
VectorBase<T> operator+(const VectorBase<T>& v1, const VectorBase<T>& v2)
{
	CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "Vector::operator+: incompatible size of vectors");
	VectorBase<T> result(v1.NumberOfItems());
	Index cnt = 0;
	for (auto &item : result) {
		item = v1[cnt] + v2[cnt];
		cnt++;
	}
	return result;
}

//! add two vectors, result = v1-v2 (for each component)
template<typename T>
VectorBase<T> operator-(const VectorBase<T>& v1, const VectorBase<T>& v2)
{
	CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "Vector::operator-: incompatible size of vectors");
	VectorBase<T> result(v1.NumberOfItems());
	Index cnt = 0;
	for (auto &item : result) {
		item = v1[cnt] - v2[cnt];
		cnt++;
	}
	return result;
}

//! scalar multiply, result = scalar * v (for each component)
template<typename T>
VectorBase<T> operator*(const VectorBase<T>& v, Real scalar)
{
	VectorBase<T> result(v.NumberOfItems());
	Index cnt = 0;
	for (auto &item : result) {
		item = scalar * v[cnt++];
	}
	return result;
}

//! scalar multiply, result = v * scalar (for each component)
template<typename T>
VectorBase<T> operator*(Real scalar, const VectorBase<T>& v)
{
	VectorBase<T> result(v.NumberOfItems());
	Index cnt = 0;
	for (auto &item : result) {
		item = scalar * v[cnt++];
	}
	return result;
}


//! append a vector to Vector(*this) and return result=[*this,vector]; does not modify *this
template<typename T>
VectorBase<T> VectorBase<T>::Append(const VectorBase<T>& vector) const
{
	VectorBase<T> newVector(NumberOfItems() + vector.NumberOfItems());
	newVector.CopyFrom(*this, 0, 0, NumberOfItems());
	newVector.CopyFrom(vector, 0, NumberOfItems(), vector.NumberOfItems());
	return newVector;
}
