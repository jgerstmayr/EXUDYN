/** ***********************************************************************************************
* @class        FlatArray
* @brief        A pointer to items and their number: transports an array owned by someone else (#3368); no resize, no math
*
* @author       Gerstmayr Johannes
* @date         2026-11-08
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
************************************************************************************************ */
#ifndef FLATARRAY__H
#define FLATARRAY__H

#include <cstddef>
#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Utilities/ResizableArray.h" //includes SlimArray

//! passed by value, as cheap as a raw pointer; the constness of the items is that of T: FlatArray<const X> for reading
template <class T>
class FlatArray
{
	T* data;
	Index numberOfItems;

public:
	FlatArray() : data(nullptr), numberOfItems(0) {}
	FlatArray(T* dataInit, Index numberOfItemsInit) : data(dataInit), numberOfItems(numberOfItemsInit) {}

	//! a local array, for example MarkerRigid<Real> kinematics[2]
	template <class U, std::size_t n> FlatArray(U (&array)[n]) : data(array), numberOfItems((Index)n) {}

	//! the same items with const added
	template <class U> FlatArray(const FlatArray<U>& other) : data(other.GetDataPointer()), numberOfItems(other.NumberOfItems()) {}

	template <class U, Index n> FlatArray(SlimArray<U, n>& array) : data(array.GetDataPointer()), numberOfItems(n) {}
	template <class U> FlatArray(ResizableArray<U>& array) : data(array.GetDataPointer()), numberOfItems(array.NumberOfItems()) {}

	Index NumberOfItems() const { return numberOfItems; }

	//! the pointer to the first item; nullptr if empty
	T* GetDataPointer() const { return data; }

	T& operator[](Index item) const
	{
		CHECKandTHROW(item >= 0 && item < numberOfItems, "FlatArray::operator[]: index out of range");
		return data[item];
	}

	T* begin() const { return data; }
	T* end() const { return data + numberOfItems; }
};

#endif
