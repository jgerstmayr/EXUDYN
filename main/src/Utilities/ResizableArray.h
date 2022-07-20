/** ***********************************************************************************************
* @class	    ResizableArray
* @brief		Templated array with dynamic (re-)allocation of memory (similar to std::vector<>)
* @details		Details:
                - array enlarges dynamically its size, redoubles if allocated memory is insufficient
                - index runs from 0 to n-1, the currently used number of items is NumberOfItems()
                - maximum currently available Memory: MaxNumberOfItems()
                - ResizableArray<ResizableArray> is not possible!
                - ResizableArray is not designed for classes with dynamically allocated data (or linked date as in ConstSizeVector ) or
                    initalization in constructors. Use array of pointers instead: ResizableArray<SpecialClass*> .

* @author		Gerstmayr Johannes
* @date			1997-06-19 (first created)
* @date			2018-05-01 (refactored)
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
* ResizableArray<int> a1({1, 5, 7}); //create an array with 3 int
* ResizableArray<int> a2 = a1;       //assign a1 to a2
* a2[0] = 42;                        //modify component
* cout << a2 << "\n";                //write "[42, 5, 7]" to cout
* @endcode
************************************************************************************************ */
#ifndef RESIZABLEARRAY__H
#define RESIZABLEARRAY__H

#include "Utilities/BasicFunctions.h"
#include "Utilities/SlimArray.h"

#include <algorithm> //for std::min / std::max, for_each
#include <cstring>
#include <array>
#include <cstring>

typedef ResizableArray<Real> ArrayReal;
typedef ResizableArray<Index> ArrayIndex;
typedef std::vector<Index> StdArrayIndex; //needed for user functions

typedef std::array<std::array<float, 3>, 3> StdArray33F;

#ifdef __EXUDYN_RUNTIME_CHECKS__
extern Index array_new_counts; //global counter of item allocations; is increased every time a new is called
extern Index array_delete_counts; //global counter of item deallocations; is increased every time a delete is called
#endif


template <class T>
class ResizableArray
{
private: //make protected if derived class shall be created
	T * data;				//!<container for stored items
	Index maxNumberOfItems;	//!<maximum number of items currently storable
	Index numberOfItems;	//!<current number of items stored; this is the size which is operated at

public:
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// CONSTRUCTOR, DESTRUCTOR
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! default constructor: no memory allocation, reset pointers and sizes; data = nullptr; maxNumberOfItems = 0; numberOfItems = 0;
	ResizableArray() { Init(); }

	//initialization with array size; elements not initialized!
	ResizableArray(Index maxNumberOfItemsInit) {
		data = nullptr;
		numberOfItems = 0;
		maxNumberOfItems = maxNumberOfItemsInit;

		if (maxNumberOfItems > 0) 
		{ 
			try
			{
				data = new T[maxNumberOfItems];
			}
			catch (const std::bad_alloc& e) {
				pout << "Allocation failed: " << e.what() << '\n';
				pout << "requested memory = " << sizeof(T)*maxNumberOfItems / pow(2, 20) << " MB, number of items = " << maxNumberOfItems << "\n";

				CHECKandTHROWstring("ResizableArray(maxNumberOfItems): Allocation failed");
			}
		}
	}

	//! initializer list (CHECK PERFORMANCE?)
	ResizableArray(std::initializer_list<T> listOfItems) //pass by value as a standard in C++11
	{
		//EnlargeMaxNumberOfItemsTo((Index)listOfItems.size());
		data = nullptr;
		numberOfItems = (Index)listOfItems.size();
		maxNumberOfItems = (Index)listOfItems.size();
		if (maxNumberOfItems > 0) 
		{ 
			data = new T[maxNumberOfItems]; 
#ifdef __EXUDYN_RUNTIME_CHECKS__
			array_new_counts++;
#endif
		}

		Index cnt = 0;
		for (auto val : listOfItems) {
			(*this)[cnt++] = val;
		}
	}

	//! constructor with std::vector
	ResizableArray(const std::vector<T> vector)
	{
		data = nullptr;
		numberOfItems = (Index)vector.size();
		maxNumberOfItems = (Index)vector.size();
		if (maxNumberOfItems > 0)
		{
			data = new T[maxNumberOfItems];
#ifdef __EXUDYN_RUNTIME_CHECKS__
			array_new_counts++;
#endif
		}

		std::copy(vector.begin(), vector.end(), this->begin());
	}

	//! constructor with SlimVector, allows only explicit conversion!
	template<Index slimArraySize>
	explicit ResizableArray(const SlimArray<T,slimArraySize> slimArray)
	{
		data = nullptr;
		numberOfItems = slimArray.NumberOfItems();
		maxNumberOfItems = numberOfItems;
		if (maxNumberOfItems > 0)
		{
			data = new T[maxNumberOfItems];
#ifdef __EXUDYN_RUNTIME_CHECKS__
			array_new_counts++;
#endif
		}

		std::copy(slimArray.begin(), slimArray.end(), this->begin());
	}

	//! copy constructor; copies data (e.g. Index, Real or SlimVector<3>)
	ResizableArray(const ResizableArray<T>& array) {
		//not all cases of CopyFrom will initialize array ==> do it here!
		data = nullptr;
		numberOfItems = 0;
		maxNumberOfItems = 0;
		CopyFrom(array);
	}

	//move constructor not allowed; could cause troubles with ownership of data ==> use class LinkedDataArray
	//ResizableArray(ResizableArray<T>&&) = delete;

	//move constructor, e.g. for local arrays in return values, avoiding copying; noexcept used, because "user-defined move constructors should not throw exceptions"
	ResizableArray(ResizableArray<T>&& other) noexcept :
		data(std::exchange(other.data, nullptr)),
		maxNumberOfItems(std::exchange(other.maxNumberOfItems, 0)),
		numberOfItems(std::exchange(other.numberOfItems, 0)) {}
	//ResizableArray(ResizableArray<T>&& other) noexcept :
	//	data(std::move(other.data)),
	//	maxNumberOfItems(std::move(other.maxNumberOfItems)),
	//	numberOfItems(std::move(other.numberOfItems)) {
	//	other.data = nullptr;
	//	other.maxNumberOfItems = 0;
	//	other.numberOfItems = 0;
	//}

	//! destructor to handle dynamically allocated data
	~ResizableArray() {
		if (data)
		{
			delete[] data;
			data = nullptr;
			numberOfItems = 0;
			maxNumberOfItems = 0;
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// BASIC FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! get an exact clone of *this, must be implemented in all derived classes! Necessary for handling in ObjectContainer
	ResizableArray<T>* GetClone() const { return new ResizableArray<T>(*this); }
	//! helper access function which just returns class itself
	const ResizableArray<T>& This() const { return *this; }

	T* begin() const { return &data[0]; }						//!< C++11 std::begin() for iterators
	T* end() const { return &data[numberOfItems]; }				//!< C++11 std::end() for iterators
	//const T* begin() const { return &data[0]; }				//!< C++11 std::begin() for iterators, const version needed for ==, +=, etc.
	//const T* end() const { return &data[numberOfItems]; }	    //!< C++11 std::end() for iterators, const version needed for ==, +=, etc.

	Index NumberOfItems() const { return numberOfItems; }		//!< get number of items (current length)
	Index MaxNumberOfItems() const { return maxNumberOfItems; }	//!< get max number of items (allocated data length)
	//T* GetDataPointer() { return &data[0]; }					//!< return pointer to first data containing T* items.
	T* GetDataPointer() { return data; }					    //!< return pointer to first data containing T* items.
	T* GetDataPointer() const { return data; }					    //!< return pointer to first data containing T* items.

	//! set all items in array to 'value'
	void SetAll(const T& scalarValue) {
		for (auto &item : *this) { item = scalarValue; }
	}

	//! read access to last item at numberOfItems-1
	const T& Last() const
	{
		CHECKandTHROW(numberOfItems, "ERROR: ResizableArray<T>::Last const, numberOfItems == 0");
		return data[numberOfItems - 1];
	}

	//! write access to last item at numberOfItems-1
	T& Last()
	{
		CHECKandTHROW(numberOfItems, "ERROR: ResizableArray<T>::Last, numberOfItems == 0");
		return data[numberOfItems - 1];
	}

	//! swap content of this and other matrix without copying
	void Swap(ResizableArray& other)
	{
		std::swap(data, other.data);
		std::swap(numberOfItems, other.numberOfItems);
		std::swap(maxNumberOfItems, other.maxNumberOfItems);
	}

protected:
	//! unified initialization of member variables; protected, because call from outside is dangerous (no data deleted); use SetNumberOfItems(0) or Flush() to reset ResizableArray
	void Init() { data = nullptr; maxNumberOfItems = 0; numberOfItems = 0; }

public:
	//! set number of items to 'n'; used to reset array with 'SetNumberOfItems(0)', does not allocated memory/delete if newNumberOfItems<=maxNumberOfItems; copies data if array is enlarged
	void SetNumberOfItems(Index newNumberOfItems) 
	{ 
		if (newNumberOfItems > maxNumberOfItems)
		{ 
			EnlargeMaxNumberOfItemsTo(newNumberOfItems);
		};
		numberOfItems = newNumberOfItems;

		//WRONG: 2022-07-11: numberOfItems is needed in SetMaxNumberOfItems; set hereafter anyway!
		//numberOfItems = newNumberOfItems;
		//if (numberOfItems > maxNumberOfItems) { EnlargeMaxNumberOfItemsTo(numberOfItems); };
	}

	//! reset function without argument (needed in contact, but may be erased?)
	void SetNumberOfItems0() {
		numberOfItems = 0;
	}

	//! check if an index is in range of valid items
	bool IsValidIndex(Index index) const { return (index >= 0) && (index < NumberOfItems()); }

	void EnlargeMaxNumberOfItemsTo(Index minSize)
	{
		if (minSize <= maxNumberOfItems || minSize == 0) { return; } //do nothing if size fits!

		////current array too small, try to double size
		//Index newSize = 2 * maxNumberOfItems;

		////check if array fits now, otherwise enlarge
		//if (newSize < minSize) { newSize = minSize; }

		//SetMaxNumberOfItems(newSize);

		SetMaxNumberOfItems(std::max(2 * maxNumberOfItems, minSize));
	}


	//! set allocated memory/maxNumberOfItems exactly to given value; if (newNumberOfItems!=maxNumberOfItems) it results in memory delete and allocation; NOTE: numberOfItems is reduced to fit newNumberOfItems, otherwise unchanged; data is copied in the range up to to min(newNumberOfItems, maxNumberOfItems).  
	void SetMaxNumberOfItems(Index newNumberOfItems);

	//! @brief copy array entries from given array; copy elements starting at 'beginPosition' until 'endPosition-1' (according to iterators convention)
	//! 'endPosition' = EXUstd::InvalidIndex indicates a complete copy of the array up to numberOfItems;
	//!	indices follow std::begin/end convention; beginPosition=0, endPosition=1 copies the first element; beginPosition=0, endPosition=n copies the first n elements
	//! CopyFrom is not virtual, because derived class has different arguments
	void CopyFrom(const ResizableArray<T>& array, Index beginPosition = 0, Index endPosition = EXUstd::InvalidIndex);

	void Flush();		//!< delete data and set numberOfItems AND maxNumberOfItems to 0

   // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // OPERATORS
   // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //!random write access operator; will increase size of array if necessary (possible non-initialized data!).
	T& operator[] (Index i)
	{
		CHECKandTHROW(i >= 0, "ResizableArray<T>::operator[], i < 0");

		//! @todo use SetNumberOfItems in T& operator[] (Index i)
		if (i >= maxNumberOfItems) { EnlargeMaxNumberOfItemsTo(i + 1); }
		if (i >= numberOfItems) { numberOfItems = i + 1; }
		return data[i];
	}

	//! random read access operator; read access only to elements in range[0,numberOfItems].
	const T& operator[] (Index i) const
	{
		CHECKandTHROW((i >= 0), "ResizableArray<T>::const operator[], i < 0");
		CHECKandTHROW((i < numberOfItems), "ResizableArray<T>::const operator[], i >= numberOfItems"); //read access to non-initialized data makes no sense!

		return data[i];
	}


	//!assignment operator; not virtual, because derived class has different arguments
	ResizableArray<T> & operator= (const ResizableArray<T> &t) {
		if (this == &t) { return *this; }
		CopyFrom(t);
		return *this;
	}

	//! move assignement operator
	//  move assignment operator originally not allowed; troubles with ownership of data? ==> use class LinkedDataArray
	ResizableArray<T> & operator= (ResizableArray<T> && other)
	{
		if (this != &other)
		{
			Flush(); //delete data and reset sizes

			data = std::exchange(other.data, nullptr);
			maxNumberOfItems = std::exchange(other.maxNumberOfItems, 0);
			numberOfItems = std::exchange(other.numberOfItems, 0);
		}
		return *this;
	}

    //! comparison operator, component-wise compare; returns true, if all components are equal; not virtual, because derived class has different arguments
    bool operator== (const ResizableArray<T>& array) const
    {
		CHECKandTHROW((NumberOfItems() == array.NumberOfItems()), "ResizableArray::operator==: incompatible size of arrays");
        Index cnt = 0;
        for (auto item : array)
        {
            if (!(item == (*this)[cnt++])) { return false; }
        }
        return true;
    }

	//! conversion of ResizableArray into std::vector (e.g. for usage in pybind11)
	operator std::vector<T>() const { return std::vector<T>(begin(), end()); }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // EXTENDED FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! const (read) access of item with index 'i' 
     const T& GetItem(Index index) const
    {
		CHECKandTHROW((index >= 0), "ERROR: ResizableArray const T& GetItem: index < 0");
		CHECKandTHROW((index < numberOfItems), "ERROR: ResizableArray const T& GetItem: index >= dataSize");
        return data[index];
    }

    //! by reference (write) access of item with index 'i'; DOES NOT AUTOMATICALLY ENLARGE ARRAY (compatibility with SlimArray<>)
     T& GetItem(Index index)
    {
		CHECKandTHROW((index >= 0), "ERROR: ResizableArray T& GetItem: index < 0");
		CHECKandTHROW((index < numberOfItems), "ERROR: ResizableArray T& GetItem: index >= dataSize");
        return data[index];
    }

	 //! const (read) access of item with index 'i'; no index checks
	 const T& GetItemUnsafe(Index index) const
	 {
		 return data[index];
	 }

	 //! by reference (write) access of item with index 'i'; no index checks
	 T& GetItemUnsafe(Index index)
	 {
		 return data[index];
	 }

	 //! const (read) CYCLIC access of item with index 'i' 
     const T& GetItemCyclic(Index index) const
    {
        return data[index%numberOfItems]; //access is safe, always inside array range!
    }

    //! by reference (write) CYCLIC access of item with index 'i' 
     T& GetItemCyclic(Index index)
    {
        return data[index%numberOfItems]; //access is safe, always inside array range!
    }

    //! @brief append an item after last element (*this[numberOfItems]);
    //! increases automatically the array size if necessary
    //! returns index of item which has been appended
    Index Append(const T& item) //== > push_back in std::vector
	{
		(*this)[numberOfItems] = item;  //numberOfItems increased by one
		return numberOfItems - 1;       //Index of last element
	}

	//! @brief append an array after last element (*this[numberOfItems]);
	//! increases automatically the array size if necessary
	//! internally no checks on otherArray
	void AppendArray(const ResizableArray<T>& otherArray)
	{
		Index n = this->NumberOfItems();
		//this->EnlargeMaxNumberOfItemsTo(n + otherArray.NumberOfItems());
		this->SetNumberOfItems(n + otherArray.NumberOfItems());
		for (Index i = 0; i < otherArray.NumberOfItems(); i++)
		{
			data[i + n] = otherArray.GetItemUnsafe(i);
		}
	}

	//! @brief append an item after last element (*this[numberOfItems]);
	 //! increases automatically the array size if necessary
	 //! no return; faster than Append()
	 void AppendPure(const T& item) //== > push_back in std::vector
	 {
		 EnlargeMaxNumberOfItemsTo(numberOfItems + 1);
		 data[numberOfItems] = item;
		 numberOfItems++;
	 }

	 //! search for item; return array index of FIRST item (if found), otherwise return; different return value from HOTINT1 tarray::find(...): FindIndexOfItem(...) returns EXUstd::InvalidIndex in case it is not found!!!
     Index GetIndexOfItem(const T& item) const
    {
        for (Index j = 0; j < numberOfItems; j++)
        {
            if (data[j] == item) return j;
        }
        return EXUstd::InvalidIndex;
    }

    //! Append item (see Append() if item not found in array
     Index AppendIfItemNotFound(const T& item)
    {
        Index f = GetIndexOfItem(item);

        if (f == EXUstd::InvalidIndex) { return Append(item); }
        else { return f; }
    }

    //! insert item at position; moves items previously at position and items>position backwards; increases numberOfItems by one; insert at data[numberOfItems] allowed!
     void Insert(Index position, const T& item)
    {
		CHECKandTHROW((position >= 0), "ERROR: ResizableArray const T& Insert: position < 0");
		CHECKandTHROW((position <= numberOfItems), "ERROR: ResizableArray const T& Insert: position > numberOfItems");

        numberOfItems++;
        EnlargeMaxNumberOfItemsTo(numberOfItems);
        for (Index j = numberOfItems - 1; j > position; j--)
        {
            data[j] = data[j - 1];
        }
        data[position] = item;
    }

    //! erase item at position; move forward all elements in range(position+1,numberOfItems) ; decreases numberOfItems by one; does nothing if position == -1 ==> compatible with GetIndexOfItem(...)
     void Remove(Index position)
    {
        if (position == EXUstd::InvalidIndex) return;
        for (Index j = position; j < numberOfItems - 1; j++)
        {
            data[j] = data[j + 1];
        }
        numberOfItems--;
    }

	//! Sort items in ascending order, using external Quicksort(...) function; 
	void Sort() { EXUstd::QuickSort(*this); }
};

//! output stream operator for array; for template class only one definition
template <class T>
std::ostream& operator<<(std::ostream& os, const ResizableArray<T>& array)
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

//**********************************************************************
// Implementation
//**********************************************************************

template <class T>
void ResizableArray<T>::CopyFrom(const ResizableArray<T>& array, Index beginPosition, Index endPosition)
{
    if (endPosition == EXUstd::InvalidIndex) { endPosition = array.numberOfItems; }

	CHECKandTHROW(beginPosition >= 0, "ResizableArray<T>::CopyFrom, beginPosition < 0");
	CHECKandTHROW(endPosition <= array.NumberOfItems(), "ResizableArray<T>::CopyFrom, endPosition > numberOfItems");

    if (array.numberOfItems == 0) { numberOfItems = 0; return; }

    if ((endPosition - beginPosition) > maxNumberOfItems) { EnlargeMaxNumberOfItemsTo(endPosition - beginPosition); }

    std::memcpy(data, &array.data[beginPosition], (size_t)(endPosition - beginPosition) * sizeof(T));
    numberOfItems = endPosition - beginPosition;
}

//! delete data and reset size to 0
template <class T>
void ResizableArray<T>::Flush()
{
    if (data) delete[] data;
    Init();
}

////! if maxNumberOfItems is smaller than minSize, enlarge array to fit at least 'minSize'; NOTE(different from HOTINT1): only 'numberOfItems' copied; this function corresponds to old TArray::ReSize(); NOTE: numberOfItems stays UNCHANGED; copies data if array is enlarged
//template <class T>
//void ResizableArray<T>::EnlargeMaxNumberOfItemsTo(Index minSize)
//{
//    if (minSize == 0 || minSize <= maxNumberOfItems) { return; } //do nothing if size fits!
//
//    //current array too small, try to double size
//    Index newSize = 2 * maxNumberOfItems;
//
//    //check if array fits now, otherwise enlarge
//    if (newSize < minSize) { newSize = minSize; }
//
//    SetMaxNumberOfItems(newSize);
//}

//! set allocated memory/maxNumberOfItems exactly to given value; if (newNumberOfItems!=maxNumberOfItems) it results in memory delete and allocation; NOTE: numberOfItems is reduced to fit newNumberOfItems, otherwise unchanged; data is copied in the range up to to min(newNumberOfItems, maxNumberOfItems).  
template <class T>
void ResizableArray<T>::SetMaxNumberOfItems(Index newNumberOfItems)
{
    if (newNumberOfItems != 0)
    {
		T* ndata = nullptr;
		try
		{
			ndata = new T[newNumberOfItems];
		}
		catch (const std::bad_alloc& e) {
			pout << "Allocation failed: " << e.what() << '\n';
			pout << "requested memory = " << sizeof(T)*newNumberOfItems / pow(2, 20) << " MB, number of items = " << newNumberOfItems << "\n";

			CHECKandTHROWstring("ResizableArray: Allocation failed");
		}

#ifdef __EXUDYN_RUNTIME_CHECKS__
		array_new_counts++;
#endif

        if (data != nullptr && std::max(maxNumberOfItems, newNumberOfItems) != 0)
        {
            std::memcpy(ndata, data, (size_t)(std::min(numberOfItems, newNumberOfItems)) * sizeof(T)); //only copy available items
        }
        if (data != nullptr) 
		{
            delete[] data;
#ifdef __EXUDYN_RUNTIME_CHECKS__
			array_delete_counts++;
#endif
		}
        data = ndata;
    }
    else
    {
        if (data != nullptr) {
            delete[] data;
#ifdef __EXUDYN_RUNTIME_CHECKS__
			array_delete_counts++;
#endif
		}
        data = nullptr;
    }
    maxNumberOfItems = newNumberOfItems;
    numberOfItems = std::min(newNumberOfItems, numberOfItems); //in case of size reduction, numberOfItems is decreased
}



namespace EXUstd {
	//! set all arrays to zero size
	template <class T>
	inline void ArrayOfArraysSetNumberOfItems0(ResizableArray<T*> & arrayOfArrays)
	{
		std::for_each(arrayOfArrays.begin(), arrayOfArrays.end(), [](T* item) { item->SetNumberOfItems0(); });
		
		//std::for_each(arrayOfArrays.begin(), arrayOfArrays.end(), T::SetNumberOfItems0);
		//std::for_each(arrayOfArrays.begin(), arrayOfArrays.end(), std::bind2nd(std::mem_fun_ref(&T::SetNumberOfItems),0));
		//for (T* a : arrayOfArrays)
		//{
		//	a->SetNumberOfItems(0);
		//}
	}
};



#endif
