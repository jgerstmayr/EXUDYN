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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
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
#ifndef RESIZABLEVECTORBASE__H
#define RESIZABLEVECTORBASE__H

#include "Linalg/Vector.h"
#include "Linalg/ConstSizeVector.h"

typedef std::vector<Real> StdVector; //needed for user functions

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
    ResizableVectorBase(std::initializer_list<T> listOfReals): VectorBase<T>(listOfReals), maxNumberOfItems((Index)listOfReals.size()) {}
private:
	ResizableVectorBase(std::initializer_list<Index>) = delete; //constructor forbidden, as it would convert wrongly for ResizableVector({2}) into ResizableVector(2)
public:

    //! @todo ResizableVectorBase:copy constructor should not be needed
    //! copy constructor; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items]
    ResizableVectorBase(const ResizableVectorBase& vector): VectorBase<T>(vector), maxNumberOfItems(vector.NumberOfItems()) {}

	//! constructor with copy from VectorBase<T>; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items]
	ResizableVectorBase(const VectorBase<T>&& vector) noexcept : VectorBase<T>(vector), maxNumberOfItems(vector.NumberOfItems()) {}


	//! constructor with std::vector
	ResizableVectorBase(const std::vector<T> vector)
	{
		maxNumberOfItems = (Index)vector.size();
		this->AllocateMemory((Index)vector.size());

		std::copy(vector.begin(), vector.end(), this->begin());
	}

	//! delete[] of VectorBase<T> called which does the job
    virtual ~ResizableVectorBase() {}

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	virtual VectorType GetType() const override { return VectorType::ResizableVector; }

    Index MaxNumberOfItems() const { return maxNumberOfItems; }             //!< Get dataSize (available memory) of ConstSizeVectorBase

    //! Change the currently used numberOfItems, which can be larger than previous numberOfItems (leads to delete/new) or smaller/equal to previous numberOfItems (no delete/new)
    virtual void SetNumberOfItems(Index newNumberOfItems) override
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

	//! copy numberOfCopiedItems items of a vector at vectorPosition to VectorBase(*this) at thisPosition, 
	template<class TVector>
	void CopyFrom(const TVector& vector, Index vectorPosition, Index thisPosition, Index numberOfCopiedItems)
	{
		CHECKandTHROW((thisPosition + numberOfCopiedItems <= this->NumberOfItems()), "ResizableVectorBase::CopyFrom(...): thisPosition index mismatch");
		CHECKandTHROW((vectorPosition + numberOfCopiedItems <= vector.NumberOfItems()), "ResizableVectorBase::CopyFrom(...): vectorPosition index mismatch");

		for (Index i = 0; i < numberOfCopiedItems; i++)
		{
			this->GetUnsafe(i + thisPosition) = vector.GetUnsafe(i + vectorPosition);
		}
	}

	//! copy from other vector (or even array) and perform type conversion (e.g. for graphics)
	template<class TVector>
	void CopyFrom(const TVector& vector)
	{
		SetNumberOfItems(vector.NumberOfItems());

		Index cnt = 0;
		for (auto val : vector) {
			this->GetUnsafe(cnt++) = (T)val;
		}
	}

    //! @todo: ResizableVectorBase: check if operator+,-,* need to be overloaded (compare ConstSizeVectorBase)

	//! overloaded operator=, because it needs to return a ResizableVectorBase
	template <class TVector>
	ResizableVectorBase& operator=(const TVector& vector)
	{
		if (this == &vector) { return *this; }

		SetNumberOfItems(vector.NumberOfItems());

		Index cnt = 0;
		for (auto item : vector) {
			this->GetUnsafe(cnt++) = item;
		}
		return *this;
	}

	//! add vector v to *this vector (for each component); both vectors must have same size
	//ResizableVectorBase& operator+=(const ResizableVectorBase& v)
	template <class TVector>
	ResizableVectorBase& operator+=(const TVector& v)
	{
		CHECKandTHROW((this->NumberOfItems() == v.NumberOfItems()), "ResizableVectorBase::operator+=: incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			this->GetUnsafe(cnt++) += item;
		}
		return *this;
	}

	//! substract vector v from *this vector (for each component); both vectors must have same size
	//ResizableVectorBase& operator-=(const ResizableVectorBase& v)
	template <class TVector>
	ResizableVectorBase& operator-=(const TVector& v)
	{
		CHECKandTHROW((this->NumberOfItems() == v.NumberOfItems()), "ResizableVectorBase::operator-=: incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			this->GetUnsafe(cnt++) -= item;
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


////! constructor with std::vector
//template<typename T>
//ResizableVectorBase<T>::ResizableVectorBase(const std::vector<T> vector)
//{
//	maxNumberOfItems = (Index)vector.size();
//	AllocateMemory((Index)vector.size());
//
//	std::copy(vector.begin(), vector.end(), this->begin());
//}

#endif
