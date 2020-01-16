/** ***********************************************************************************************
* @class		ConstSizeVectorBase
* @brief		A templated vector with constant size and data to be allocated on stack
* @details		Details:
                    - a vector of T items with constant size;
                    - templated number of available Reals using 'dataSize'
                    - memory allocation usually on stack
                    - this vector can NOT be used in dynamic arrays (DO NOT USE WITHIN std::vector, ResizableArray and similar); use SlimVector instead
                    - data must be copied with copy constructor / operator=
                    - useful for short vectors (e.g. coordinates in ComputationalObjects)
                    - ConstSizeVectorBase has templated available memory 'dataSize' and current length 'numberOfItems'<='dataSize'
                    - Only a portion of the available memory can be used

* @author		Gerstmayr Johannes
* @date			2018-04-30 (created)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use ConstSizeVectorBase for small vector sizes (<100; better: <=12)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
* @code{.cpp}
*   ConstSizeVectorBase<Real,3> v1({1.1, 2.7, 3.0});	//create a vector with 3 Real
*   ConstSizeVectorBase<Real,4> v2;
*  	v2 = v1;							//assign v1 to v2
*  	v1 += v2;							//add v2 to v1
*  	cout << v1 << "\n";					//write "[1.1, 2.7, 3.0]" to cout
*  	ConstSizeVectorBase<Real,3> v3({ 1,2 });
*  	ConstSizeVectorBase<Real,3> v4({ 4,5,6.6 });
*  	ConstSizeVectorBase<Real,6> v = v3.Append(v4); //make [1 2 4 5 6.6] without memory allocation
*   cout << "v6=" << v << "\n";
* @endcode
************************************************************************************************ */
#pragma once

#include "Linalg/Vector.h"


template<typename T, Index dataSize>
class ConstSizeVectorBase: public VectorBase<T>
{
protected:
    T constData[dataSize];

public:
    //! default constructor, calls special constructor if vector for linking data, no allocation; no data item initialization (undefined values)
    ConstSizeVectorBase()
    {
    	this->numberOfItems = dataSize;
		this->data = &constData[0];
    }


    //! initialize ConstSizeVectorBase with numberOfItemsInit <= dataSize; no data item initialization (undefined values)
    //! @todo: add private constructors Vector(T), Vector(T,T), Vector(int, int) in order to avoid programming errors; same for ConstSizeVectorBase
    ConstSizeVectorBase(Index numberOfItemsInit)
    {
        release_assert(numberOfItemsInit <= dataSize && "ERROR: call to ConstSizeVectorBase(Index): dataSize mismatch");
		this->data = &constData[0];
		this->numberOfItems = numberOfItemsInit;
	}

    //! initialize ConstSizeVectorBase with numberOfItemsInit Reals; assign all data items with 'initializationValue'
    ConstSizeVectorBase(Index numberOfItemsInit, T initializationValue)
    {
        release_assert(numberOfItemsInit <= dataSize && "ERROR: call to ConstSizeVectorBase(Index): dataSize mismatch");

		this->data = &constData[0];
		this->numberOfItems = numberOfItemsInit;

        Index cnt = 0;
        for (auto &value : *this) {
            value = initializationValue;
        }
    }

    //! constructor with initializer list; condition: listOfReals.size() <= dataSize
    ConstSizeVectorBase(std::initializer_list<T> listOfReals) //pass by value as a standard in C++11
    {
        release_assert(listOfReals.size() <= dataSize && "ERROR: ConstSizeVectorBase::constructor, dataSize mismatch with initializer_list");
        //static_assert supported by C++14 (supports listOfReals.size() as constexpr) ==> needs /std:c++17 flag

		this->data = &constData[0];
		this->numberOfItems = listOfReals.size();

        Index cnt = 0;
        for (auto val : listOfReals) {
            constData[cnt++] = val;
        }
    }

    //! copy constructor; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items]
    ConstSizeVectorBase(const ConstSizeVectorBase<T,dataSize>& vector)
    {
		this->data = &constData[0];
		this->numberOfItems = vector.numberOfItems;

		Index cnt = 0;
        for (auto value : vector) {
            constData[cnt++] = value;
        }
    }

    //! @brief Initialize ConstSizeVectorBase by data given from vector at startPositionVector; 
    //! copies 'dataSize' items, independently of array size (might cause memory access error)
    ConstSizeVectorBase(const VectorBase<T>& vector, Index startPositionVector)
    {
        release_assert(startPositionVector >= 0 && "ERROR: ConstSizeVectorBase(const VectorBase<T>&, Index), startPositionVector < 0");
        release_assert(dataSize + startPositionVector <= vector.NumberOfItems() && "ERROR: ConstSizeVectorBase(const VectorBase<T>&, Index), dataSize mismatch");

		this->data = &constData[0];
		this->numberOfItems = dataSize;

        Index cnt = startPositionVector;
        for (auto &value : *this) {
            value = vector[cnt++];
        }
    }

    //! override destructor / delete[] from VectorBase<T>; no memory deallocated
    virtual ~ConstSizeVectorBase()
    {
		this->data = nullptr; //because destructor ~VectorBase<T> & VectorBase<T>::FreeMemory() are called hereafter ==> this will cause ~VectorBase<T> not to delete anything!
    };

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // BASIC FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //override iterator functions from VectorBase<T>, check performance!
    T* begin() const override { return &this->data[0]; }				//!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
    T* end() const override { return &this->data[this->numberOfItems]; }		//!< C++11 std::end() for iterators; iterator range is always the currently used numberOfItems.
    Index MaxNumberOfItems() const { return dataSize; }             //!< Get dataSize (available memory) of ConstSizeVectorBase

    //! call to ConstSizeVectorBase::SetNumberOfItems leads to run-time error if newNumberOfItems > dataSize; used in SetVector({...}) and in operator=; does not reset data
    virtual void SetNumberOfItems(Index newNumberOfItems) override
    {
        release_assert(newNumberOfItems <= dataSize && "ERROR: call to ConstSizeVectorBase::SetNumberOfItems with newNumberOfItems > dataSize");
		this->numberOfItems = newNumberOfItems;
    }

	//! reset to zero size; no memory delete!
	virtual void Reset() override
	{
		this->numberOfItems = 0;
	}

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // OPERATORS: some operators need to be overwritten because of vector generation
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! @brief copy assignment operator; copies the currently used entries in range [0,numberOfItems]
    ConstSizeVectorBase<T,dataSize>& operator= (const ConstSizeVectorBase<T, dataSize>& vector)
    {
        if (this == &vector) { return *this; }

        SetNumberOfItems(vector.NumberOfItems());

        Index cnt = 0;
        for (auto item : vector) {
            constData[cnt++] = item;
        }
        return *this;
    }

    //! add two vectors, result = v1+v2 (for each component); only operated in range [0,numberOfItems]
    friend ConstSizeVectorBase<T, dataSize> operator+ (const ConstSizeVectorBase<T, dataSize>& v1, const ConstSizeVectorBase<T, dataSize>& v2)
    {
        release_assert((v1.NumberOfItems() == v2.NumberOfItems()) && "ConstSizeVectorBase::operator+: incompatible size of vectors");
        release_assert((v1.NumberOfItems() <= dataSize) && "ConstSizeVectorBase::operator+: incompatible size of vectors: dataSize");
        
        ConstSizeVectorBase<T, dataSize> result(v1.NumberOfItems());
        Index cnt = 0;
        for (auto &item : result) {
            item = v1[cnt] + v2[cnt];
            cnt++;
        }
        return result;
    }

    //! add two vectors, result = v1-v2 (for each component); only operated in range [0,numberOfItems]
    friend ConstSizeVectorBase<T, dataSize> operator- (const ConstSizeVectorBase<T, dataSize>& v1, const ConstSizeVectorBase<T, dataSize>& v2)
    {
        release_assert((v1.NumberOfItems() == v2.NumberOfItems()) && "ConstSizeVectorBase::operator-: incompatible size of vectors");
        release_assert((v1.NumberOfItems() <= dataSize) && "ConstSizeVectorBase::operator-: incompatible size of vectors: dataSize");
        ConstSizeVectorBase<T, dataSize> result(v1.NumberOfItems());
        Index cnt = 0;
        for (auto &item : result) {
            item = v1[cnt] - v2[cnt];
            cnt++;
        }
        return result;
    }

    //! scalar multiply, result = scalar * v (for each component); only operated in range [0,numberOfItems]
    friend ConstSizeVectorBase<T, dataSize> operator* (const ConstSizeVectorBase<T, dataSize>& v, T scalar)
    {
        release_assert((v.NumberOfItems() <= dataSize) && "ConstSizeVectorBase::operator* (scalar * v): incompatible size of vectors");
        ConstSizeVectorBase<T, dataSize> result(v.NumberOfItems());
        Index cnt = 0;
        for (auto &item : result) {
            item = scalar * v[cnt++];
        }
        return result;
    }

    //! scalar multiply, result = v * scalar (for each component); only operated in range [0,numberOfItems]
    friend ConstSizeVectorBase<T, dataSize> operator* (T scalar, const ConstSizeVectorBase<T, dataSize>& v)
    {
        release_assert((v.NumberOfItems() <= dataSize) && "ConstSizeVectorBase::operator* (v * scalar): incompatible size of vectors");
        ConstSizeVectorBase<T, dataSize> result(v.NumberOfItems());
        Index cnt = 0;
        for (auto &item : result) {
            item = scalar * v[cnt++];
        }
        return result;
    }
    
    //! append a ConstSizeVectorBase<n> to ConstSizeVectorBase<n>; both vectors must have same templated length (but could have different actual length); RETURNS a ConstSizeVectorBase<2*n>
    auto Append(const ConstSizeVectorBase<T, dataSize>& vector) const
    {
        //constexpr Index size = dataSize;
        ConstSizeVectorBase<T, 2 * dataSize> newVector(this->NumberOfItems() + vector.NumberOfItems()); //NumberOfItems might be smaller than (dataSize1+dataSize2)
        newVector.CopyFrom(*this, 0, 0, this->NumberOfItems());
        newVector.CopyFrom(vector, 0, this->NumberOfItems(), vector.NumberOfItems());
        return newVector;
    }


protected: //functions cannot be called from outside 
    //! call to ConstSizeVectorBase::AllocateMemory leads to run-time error (must not be called)
    void AllocateMemory(Index numberOfRealsInit)
    {
        CHECKandTHROWstring("ERROR: call to ConstSizeVectorBase::AllocateMemory(...) forbidden");
        //static_assert(0, "ERROR: call to ConstSizeVectorBase::AllocateMemory(...) forbidden");
    }

    //! ConstSizeVectorBase must not delete[] data; function called because of VectorBase<T> destructor 
    virtual void FreeMemory() {}

    //! append is forbidden for ConstSizeVectorBase, because to many problems with dataSize expected (v1.Append(v2) would crash if v1 does not have length of v1.length+v2.length)
	VectorBase<T> Append(const VectorBase<T>& vector) const
    {
        CHECKandTHROWstring("ERROR: Append called for ConstSizeVectorBase");
        return *this;
    }

};

template<Index dataSize>
using ConstSizeVector = ConstSizeVectorBase<Real, dataSize>;

typedef ConstSizeVector<1> CSVector1D;
typedef ConstSizeVector<2> CSVector2D;
typedef ConstSizeVector<3> CSVector3D;
typedef ConstSizeVector<4> CSVector4D;

template<Index dataSize>
using ConstSizeVectorF = ConstSizeVectorBase<float, dataSize>;

