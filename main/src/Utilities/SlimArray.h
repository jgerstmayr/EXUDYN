/** ***********************************************************************************************
* @class	    SlimArray
* @brief		A 'slim', fast, templated array with constant size of an array of type 'T', to be allocated on stack or within large (dynamic) arrays
* @details		Details:
* 				- a array of templated 'T' items (e.g. int, double, etc.);
* 				- templated number of 'T' using 'dataSize'
* 				- this array can be used in (possibly large) dynamic arrays (std::vector, ResizableArray)
* 				- data can be copied with memcopy
* 				- convenient operations (cyclic access, find, etc.) for small-sized arrays
*
* @author		Gerstmayr Johannes
* @date			1997-06-19 (generated)
* @date			2018-05-01 (last modified)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use SlimArray for small vector sizes (<100; better: <=12)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
* @code{.cpp}
* SlimArray<int, 3> a1({1, 5, 7});  //create an array with 3 int
* SlimArray<int, 3> a2 = a1;        //assign a1 to a2
* a2[0] = 42;                       //modify component
* cout << a2 << "\n";               //write "[42, 5, 7]" to cout
* @endcode
************************************************************************************************ */
#ifndef SLIMARRAY__H
#define SLIMARRAY__H

#include "Utilities/ReleaseAssert.h"
#include <initializer_list>
#include <vector>
#include <array>

template <class T, Index dataSize> class SlimArray;
template <class T> class ResizableArray;

//typedef SlimArray<Index, 1> Index1; //uncomment as soon it is needed
typedef SlimArray<Index, 2> Index2; //!< a pair of integer values
typedef SlimArray<Index, 3> Index3; //!< a triple of integer values
typedef SlimArray<Index, 4> Index4; //!< 4 integer values
typedef SlimArray<Index, 6> Index6; //!< 6 integer values for SearchTree

template <class T, Index dataSize>
class SlimArray 
{
protected:
    T data[dataSize]; //!< const number of Real given by template parameter 'dataSize'

public:
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CONSTRUCTOR, DESTRUCTOR
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//not needed
    //!default constructor: no initialization.
    SlimArray() {}; 

	//the following seems to be slower, even if no default constructor is supplied and it could be better?!
	//SlimArray(EXUstd::Dummy init= EXUstd::Dummy()) {};

    //! constructor with a single scalar value used for all vector components.
    SlimArray(T scalarValue)
    {
        for (auto &item : *this) {
            item = scalarValue;
        }
    }

    //! initializer list (CHECK PERFORMANCE?): SlimArray<int, 3> (1,2,3);
	SlimArray(std::initializer_list<T> listOfItems) //not compatible with static_assert //pass by value as a standard in C++11
	//SlimArray(const T(&listOfItems)[dataSize]) //pass by value as a standard in C++11
    {
		CHECKandTHROW(dataSize == (Index)listOfItems.size(), "ERROR: SlimArray::constructor, dataSize mismatch with initializer_list");
		//static_assert supported by C++14 (supports listOfItems.size() as constexpr)

        Index cnt = 0;
        for (auto val : listOfItems) {
            (*this)[cnt++] = val;
        }
    }

    //! @brief Initialize SlimArray by data given from array at startPositionArray; 
    //! copies 'dataSize' items, independently of array size (might cause memory access error)
    SlimArray(const ResizableArray<T>& array, Index startPositionArray=0)
    {
		CHECKandTHROW(startPositionArray >= 0, "ERROR: SlimArray::(const ResizableArray<T>&, Index), startPositionArray < 0");
		CHECKandTHROW(dataSize + startPositionArray <= array.NumberOfItems(), "ERROR: SlimArray::(const ResizableArray<T>&, Index), dataSize mismatch with initializer_list");

        Index cnt = startPositionArray;
        for (auto &value : *this) {
            value = array[cnt++];
        }
    }

	SlimArray(const std::vector<T> vector)
	{
		CHECKandTHROW(vector.size() == (Index)dataSize, "ERROR: SlimArray(const std::vector<T> vector), dataSize mismatch");

		Index cnt = 0;
		for (auto& item : *this) {
			item = vector[cnt++];
		}
	}

	SlimArray(const std::array<T, dataSize> vector)
	{
		CHECKandTHROW(vector.size() == (Index)dataSize, "ERROR: SlimArray(const std::array<T> vector), dataSize mismatch");

		Index cnt = 0;
		for (auto& item : *this) {
			item = vector[cnt++];
		}
	}

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // BASIC FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    T* begin() { return &data[0]; }						//!< C++11 std::begin() for iterators.
    T* end() { return &data[dataSize]; }				//!< C++11 std::end() for iterators.
    const T* begin() const { return &data[0]; }			//!< C++11 std::begin() for iterators, const version needed for ==, +=, etc.
    const T* end() const { return &data[dataSize]; }	//!< C++11 std::end() for iterators, const version needed for ==, +=, etc.

    Index NumberOfItems() const { return dataSize; }	//!< number of ('T') items in vector ('SlimArray<3> v;' ==> NumberOfItems() returns 3).
    Index MaxNumberOfItems() const { return dataSize; }	//!< number of ('T') items in vector; for COMPATIBILITY with 'class ResizeableArray' ('SlimArray<3> v;' ==> Size() returns 3).
    T* GetDataPointer() { return &data[0]; }			//!< return pointer to first data containing T* items.

    //! set all items in array to 'scalarValue'
    void SetAll(const T& scalarValue) {
        for (auto &item : *this) { item = scalarValue; }
    }

    //! read access to last item at dataSize-1
    const T& Last() const
    {
        return data[dataSize - 1];
    }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // OPERATORS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! by reference (write) access-operator.
    T& operator[](Index item)
    {
		CHECKandTHROW((item >= 0), "ERROR: SlimArray T& operator[]: item < 0");
		CHECKandTHROW((item < dataSize), "ERROR: SlimArray T& operator[]: item >= dataSize");
        return data[item];
    };

    //! const (read) access-operator; elements are copied (assumed to be small)
    T operator[](Index item) const
    {
		CHECKandTHROW((item >= 0), "ERROR: SlimArray T operator[] const: item < 0");
		CHECKandTHROW((item < dataSize), "ERROR: SlimArray T operator[] const: item >= dataSize");
        return data[item];
    };

    //! assign a scalar value to all components of the vector
    SlimArray<T, dataSize>& operator= (T scalarValue)
    {
        for (auto &item : *this) {
            item = scalarValue;
        }
        return *this;
    }

    //! comparison operator, component-wise compare; returns true, if all components are equal
    bool operator== (const SlimArray<T, dataSize>& array) const
    {
        Index cnt = 0;
        for (auto item : array)
        {
            if (item != (*this)[cnt++]) { return false; }
        }
        return true;
    }

	//! comparison operator for scalar value; returns true, if all components are equal to value
	bool operator==(T value) const
	{
		for (auto item : (*this))
		{
			if (item != value) { return false; }
		}
		return true;
	}

	//! conversion of SlimArray into std::vector (needed e.g. in pybind)
	operator std::vector<T>() const
	{
		return std::vector<T>(begin(), end());
	}
	
	//! conversion of SlimArray into std::array (needed e.g. in pybind)
	operator std::array<T, dataSize>() const
	{
		std::array<T, dataSize> v;
		std::copy(begin(), end(), v.begin());

		return v;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // EXTENDED FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! const (read) access of item with index 'i' 
    const T& GetItem(Index index) const
    {
		CHECKandTHROW((index >= 0), "ERROR: SlimArray const T& GetItem: index < 0");
		CHECKandTHROW((index < dataSize), "ERROR: SlimArray const T& GetItem: index >= dataSize");
        return data[index];
    }

    //! by reference (write) access of item with index 'i' 
    T& GetItem(Index index)
    {
		CHECKandTHROW((index >= 0), "ERROR: SlimArray T& GetItem: index < 0");
		CHECKandTHROW((index < dataSize), "ERROR: SlimArray T& GetItem: index >= dataSize");
        return data[index];
    }

    //! const (read) CYCLIC access of item with index 'i' 
    const T& GetItemCyclic(Index index) const
    {
        return data[index%dataSize]; //access is safe, always inside array range!
    }

    //! by reference (write) CYCLIC access of item with index 'i' 
    T& GetItemCyclic(Index index)
    {
        return data[index%dataSize]; //access is safe, always inside array range!
    }

    //! search for item; return array index of FIRST item (if found), otherwise return; different return value from HOTINT1 tarray::find(...): FindIndexOfItem(...) returns EXUstd::InvalidIndex in case it is not found!!!
    Index GetIndexOfItem(const T& item) const
    {
        for (Index j = 0; j < dataSize; j++)
        {
            if (data[j] == item) return j;
        }
        return EXUstd::InvalidIndex;
    }

    //! Sort items in ascending order, using external Quicksort(...) function; 
	//does not compile on MacOS:
	//void Sort() { EXUstd::QuickSort(*this); }

    //! set items according to initializer list: SlimArray<3> ({1.0, 3.14, 5.5});
    void Set(std::initializer_list<T> listOfItems) //pass by value as a standard in C++11
    {
		CHECKandTHROW(dataSize == (Index)listOfItems.size(), "ERROR: SlimArray::Set(...), dataSize mismatch with initializer_list");
        //static_assert supported by C++14 (supports listOfItems.size() as constexpr)

        Index cnt = 0;
        for (auto value : listOfItems) {
            (*this)[cnt++] = value;
        }
    }

};

//!output stream operator for array; for template class only one definition
template <class T, Index dataSize>
std::ostream& operator<<(std::ostream& os, const SlimArray<T, dataSize>& array)
{
	char s = ' ';
	if (linalgPrintUsePythonFormat) { s = ','; }
	os << "[";
    for (Index i = 0; i < array.NumberOfItems(); i++) {
        os << array[i];
        if (i < array.NumberOfItems() - 1) { os << s; }
    }

    os << "]";
    return os;

}

#endif
