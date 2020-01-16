/** ***********************************************************************************************
* @class		ResizableVectorBase
* @brief		A vector which can have a different numberOfItems compared to allocated data (maxNumberOfItems)
* @details		Details:
                - allow a Vector to be resized, without deleting data if Vector is shrinked
                - use this Vector, e.g. if algorithms need different sized temporary vectors (e.g. element routines)
                - works similar to ResizableArray, however, dynamic enlargement is not possible (numberOfItems must be set properly)

* @author		Gerstmayr Johannes
* @date			2018-05-04
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
* @code{.cpp}
*   ResizableVector v1(10);	//create a vector, allocate memory
*   v1.SetVector({2, 3});   //use only part of vector; no delete/allocate as compared to 'VectorBase<T>'
*   Vector v2 = v1;
*  	v1.SetVector({3, 4, 5}); //no memory allocation; v1.NumberOfItems() is still 10
*  	cout << v1 << "\n";		 //write "[3 4 5]" to cout
*  	cout << "length=" << v1.NumberOfItems() << "\n";  //write "length=3" to cout
* @endcode
************************************************************************************************ */
#pragma once

#include "Linalg/Vector.h"

template<typename T>
class ResizableVectorBase: public VectorBase<T>
{
protected:
    Index maxNumberOfItems;

public:
    //! default constructor
    ResizableVectorBase(): VectorBase<T>(), maxNumberOfItems(0) {}

    //! initialize ResizableVectorBase numberOfItemsInit
    ResizableVectorBase(Index numberOfItemsInit): VectorBase<T>(numberOfItemsInit), maxNumberOfItems(numberOfItemsInit) {}

    //! initialize ResizableVectorBase with numberOfItemsInit Reals; assign all data items with 'initializationValue'
    ResizableVectorBase(Index numberOfItemsInit, T initializationValue): VectorBase<T>(numberOfItemsInit, initializationValue),
        maxNumberOfItems(numberOfItemsInit) {}

    //! constructor with initializer list; memory allocation!
    ResizableVectorBase(std::initializer_list<T> listOfReals): VectorBase<T>(listOfReals), maxNumberOfItems(listOfReals.size()) {}

    //! @todo ResizableVectorBase:copy constructor should not be needed
    //! copy constructor; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items]
    ResizableVectorBase(const ResizableVectorBase& vector): VectorBase<T>(vector), maxNumberOfItems(vector.NumberOfItems()) {}

	//! constructor with copy from VectorBase<T>; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items]
	ResizableVectorBase(const VectorBase<T>&& vector) noexcept : VectorBase<T>(vector), maxNumberOfItems(vector.NumberOfItems()) {}

    //! delete[] of VectorBase<T> called which does the job
    virtual ~ResizableVectorBase() {}

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    Index MaxNumberOfItems() const { return maxNumberOfItems; }             //!< Get dataSize (available memory) of ConstSizeVectorBase

    //! Change the currently used numberOfItems, which can be larger than previous numberOfItems (leads to delete/new) or smaller/equal to previous numberOfItems (no delete/new)
    virtual void SetNumberOfItems(Index newNumberOfItems)
    {
        if (newNumberOfItems > MaxNumberOfItems())
        {
			this->FreeMemory();
			this->AllocateMemory(newNumberOfItems);
			maxNumberOfItems = newNumberOfItems;
        }
        else
        {
			this->numberOfItems = newNumberOfItems;
        }
    }

	//! reset to zero size and free memory (in derived classes!)
	virtual void Reset() override
	{
		this->FreeMemory();
		this->numberOfItems = 0;
		maxNumberOfItems = 0;
		this->data = nullptr;
	}

	void CopyFrom(const VectorBase<T>& vector)
	{
		SetNumberOfItems(vector.NumberOfItems());

		Index cnt = 0;
		for (auto value : vector) {
			this->data[cnt++] = value;
		}
	}

    //! @todo: ResizableVectorBase: check if operator+,-,* need to be overloaded (compare ConstSizeVectorBase)

	//! overloaded operator=, because it needs to return a ResizableVectorBase
	ResizableVectorBase& operator=(const ResizableVectorBase& vector)
	{
		if (this == &vector) { return *this; }

		SetNumberOfItems(vector.NumberOfItems());

		Index cnt = 0;
		for (auto item : vector) {
			(*this)[cnt++] = item;
		}
		return *this;
	}

	//! add vector v to *this vector (for each component); both vectors must have same size
	ResizableVectorBase& operator+=(const ResizableVectorBase& v)
	{
		CHECKandTHROW((this->NumberOfItems() == v.NumberOfItems()), "ResizableVectorBase::operator+=: incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			(*this)[cnt++] += item;
		}
		return *this;
	}

	//! substract vector v from *this vector (for each component); both vectors must have same size
	ResizableVectorBase& operator-=(const ResizableVectorBase& v)
	{
		CHECKandTHROW((this->NumberOfItems() == v.NumberOfItems()), "ResizableVectorBase::operator-=: incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			(*this)[cnt++] -= item;
		}
		return *this;
	}

	//! scalar multiply vector *this with scalar (for each component)
	ResizableVectorBase& operator*=(T scalar)
	{
		for (auto &item : *this) {
			item *= scalar;
		}
		return *this;
	}

	//! scalar division of vector v through scalar (for each component)
	ResizableVectorBase& operator/=(T scalar)
	{
		for (auto &item : *this) {
			item /= scalar;
		}
		return *this;
	}
};

typedef ResizableVectorBase<Real> ResizableVector;
typedef ResizableVectorBase<float> ResizableVectorF; //always float, used for graphics

