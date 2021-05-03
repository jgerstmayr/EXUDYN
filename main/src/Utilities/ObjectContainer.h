/** ***********************************************************************************************
* @class	    ObjectContainer
* @brief		Templated array of pointers with dynamic (re-)allocation of memory (similar to std::vector<>)
* @details		Details:
                - T must be a class, not a pointer (e.g. RigidBody, Vector, ResizableArray )
                - objects T ARE ALWAYS COPIED as soon as they are added (Append, Insert, operator[]
                - the Obects are stored as pointers T* in the parent class ResizableArray; be careful with member functions, which are partially pointers (ResizableArray) and instances (ObjectContainer)
                - member functions in ObjectContainer work with items T
                - internally, the objects are stored in ResizableArray 'data' as pointers T*
                - array enlarges dynamically its size, redoubles if allocated memory is insufficient
                - index runs from 0 to n-1, the currently used number of items is NumberOfItems()
                - maximum currently available Memory: MaxNumberOfItems()
                - ObjectContainer is designed for classes (objects) with dynamically allocated data (or linked data as in ConstSizeVector ) such as Vector, RigidBody, Node, etc.
                - ObjectContainer takes care of adding/deleting object data as well (new/delete of data which the objects point to)

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
#ifndef OBJECTCONTAINER__H
#define OBJECTCONTAINER__H

#include "Utilities/BasicFunctions.h"
#include "Utilities/SlimArray.h"
#include "Utilities/ResizableArray.h"

//! this simple template leads to compile-time error, if T does not have GetClone() implemented; fails, because cast from e.g. CObject* to COMassPoint* is not possible
// if this error occurs, the according object did not implement the GetClone() function, which is mandatory for objects handled with class ObjectContainer
template<class T>
void CheckGetClone(const T& object)
{
    T* copiedObject = object.GetClone();
    delete copiedObject; //immediately delete object in order to avoid memory leaks!
}

template<class T>
class ObjectContainer
{
private:
    ResizableArray<T*> data;
public:
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CONSTRUCTOR, DESTRUCTOR
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! default constructor, no data allocated
    ObjectContainer() {}

    //! allocate maxNumberOfItems; objects not initialized; numberOfItems=0
    ObjectContainer(Index maxNumberOfItemsInit) : data(maxNumberOfItemsInit) {}

    //! initializer list with a list of objects; objects copied with copy constructor() and new
     //template<class TDerivedClass>
    ObjectContainer(std::initializer_list<T> listOfItems) //pass by value as a standard in C++11
    {
        CHECKandTHROWstring("ObjectContainer: generic constructor with initializer_list is forbidden");
        //! @todo erase assert in constructor ObjectContainer(std::initializer_list<T> listOfItems) 

        data.EnlargeMaxNumberOfItemsTo((Index)listOfItems.size());

        Index cnt = 0;
        for (const T& item : listOfItems) {
            data[cnt++] = item.GetClone();
        }
    }

    //!copy constructor
    // does not work for CObject, CNode, ... for their derived classes
    ObjectContainer(const ObjectContainer<T>& array)
    {
        CopyFrom(array);
    }

    //! move constructor not allowed; could cause troubles with ownership of data ==> use class LinkedDataArray
    ObjectContainer(const ObjectContainer<T>&&) = delete;

    //!destructor to handle dynamically allocated data; all objects deleted
    ~ObjectContainer()
    {

        if (data.NumberOfItems())
        {
            for (T* item : data) { delete item; } //! @todo check in debug mode if delete works for ObjectContainer
        }
    }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // BASIC FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    T** begin() const { return data.begin(); }                              //!< C++11 std::begin() for iterators
    T** end() const { return data.end(); }                  //!< C++11 std::end() for iterators
    //T* begin() const { return (data[0]); }                              //!< C++11 std::begin() for iterators
    //T* end() const { return (data[NumberOfItems()]); }                  //!< C++11 std::end() for iterators
    //! @todo: ObjectContainer, iterators begin/end do not work!

    Index NumberOfItems() const { return data.NumberOfItems(); }       //!< get number of items (current length)
    Index MaxNumberOfItems() const { return data.MaxNumberOfItems(); } //!< get max number of items (allocated data length)

                                                                       //! read access to last item at numberOfItems-1
    const T& Last() const
    {
        CHECKandTHROW(NumberOfItems(), "ERROR: ObjectContainer<T>::Last() const, NumberOfItems() == 0");
        return *(data[NumberOfItems() - 1]);
    }

    //! read access to last item at numberOfItems-1
    T& Last()
    {
        CHECKandTHROW(NumberOfItems(), "ERROR: ObjectContainer<T>::Last(), NumberOfItems() == 0");
        return *(data[NumberOfItems() - 1]);
    }

    //! read access to last item at numberOfItems-1
    const T* LastPtr()
    {
        CHECKandTHROW(NumberOfItems(), "ERROR: ObjectContainer<T>::LastPtr(), NumberOfItems() == 0");
        return (data[NumberOfItems() - 1]);
    }

    //! not available, because (non-initialized) default objects would need to be allocated for consistency
    //void SetNumberOfItems(Index newNumberOfItems);

    //! set maximum number of items (e.g. if known that 100000 objects are needed), allocated in ResizableArray data; all objects are erased
    void SetMaxNumberOfItems(Index newNumberOfItems)
    {
        Flush(); //delete all objects
        data.SetMaxNumberOfItems(newNumberOfItems);
    }

    // @brief copy objects (using copy constructor and new) from given array; copy elements starting at 'beginPosition' until 'endPosition-1'
    // deletes all existing objects before copying;
    // 'endPosition' = EXUstd::InvalidIndex indicates a complete copy of the array up to numberOfItems;
    //	indices follow std::begin/end convention; beginPosition=0, endPosition=1 copies the first element; beginPosition=0, endPosition=n copies the first n elements
    void CopyFrom(const ObjectContainer<T>& array, Index beginPosition = 0,
        Index endPosition = EXUstd::InvalidIndex)
    {
        if (endPosition == EXUstd::InvalidIndex) { endPosition = array.NumberOfItems(); }

        CHECKandTHROW(beginPosition >= 0, "ObjectContainer<T>::CopyFrom, beginPosition < 0");
        CHECKandTHROW(endPosition <= array.NumberOfItems(), "ObjectContainer<T>::CopyFrom, endPosition > numberOfItems");

        Flush(); //delete all existing objects
        if (array.NumberOfItems() == 0) { return; }

        data.SetNumberOfItems(endPosition - beginPosition);

        Index cnt = 0;
        for (T* item : array.data) //copy items with new item-by-item, using copy constructor
        {
            data.GetItem(cnt++) = item->GetClone();
        }
    }

    //! delete objects, delete[] data (T* pointers) and set numberOfItems AND maxNumberOfItems to 0 (completely re-initialize array)
     void Flush() 
    {
        for (T* item : data) { delete item; } 

        data.Flush();
    }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // OPERATORS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //!random write access operator; enables access to objects, not to pointers; will NOT increase size of array
    T& operator[] (Index i)
    {
        CHECKandTHROW(i >= 0, "ObjectContainer<T>::operator[], i < 0");
        CHECKandTHROW((i < NumberOfItems()), "ObjectContainer<T>::operator[], i >= numberOfItems"); //read access to non-initialized data makes no sense!

        return *(data[i]);
    }

    //! random read access operator; read access only to elements in range[0,numberOfItems].
    const T& operator[] (Index i) const
    {
        CHECKandTHROW((i >= 0), "ObjectContainer<T>::const operator[], i < 0");
        CHECKandTHROW((i < NumberOfItems()), "ObjectContainer<T>::const operator[], i >= NumberOfItems()"); //read access to non-initialized data makes no sense!

        return *(data[i]);
    }
     
    //!assignment operator; COPIES THE ITEMS OF ARRAY!
    ObjectContainer & operator= (const ObjectContainer<T> &array)
    {
        if (this == &array) { return *this; }

        CopyFrom(array);
        return *this;
    }

    //! move assignment operator not allowed; could cause troubles with ownership of data ==> use class LinkedDataArray
    ObjectContainer<T> & operator= (const ObjectContainer<T> &&) = delete;

    //! comparison operator, component-wise compare for items T, not T*; returns true, if all components are equal; not virtual, because derived class has different arguments
    bool operator== (const ObjectContainer<T>& array) const
    {
        CHECKandTHROW((NumberOfItems() == array.NumberOfItems()), "ObjectContainer::operator==: incompatible size of arrays");
        Index cnt = 0;
        for (T* item : array.data)
        {
            //compare items, not their pointers!
            if (!(*item == (*this)[cnt++]) ) { return false; }
        }
        return true;
    }

    //not needed:friend ostream& operator<<(ostream& os, const ObjectContainer<T>& array);

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // EXTENDED FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! the following trick would work for 'Append', but does not work for CopyFrom(...) / needed in copy constructor
    //template<class TDerivedClass>
    //Index Append(const TDerivedClass& item)
    //{
    //    //typedef decltype(&item) Tnew;
    //    T* myItem = new TDerivedClass(item);

    //    return data.Append(myItem);
    //}

    //! MAKE A COPY OF T and append to ObjectContainer at last position
    //! Use additional template argument TDerivedClass in order to copy the right derived class object!
    Index Append(const T& item)
    {
        //! @todo: check whether there exists a specific Tderived::GetClone method (derived class)
#ifdef __EXUDYN_RUNTIME_CHECKS__
        CheckGetClone(item);
#endif
        return data.Append(item.GetClone());
    }

    //! MAKE A COPY OF T and insert into ObjectContainer at 'position'; moves items previously at position and items>position backwards; increases numberOfItems by one; insert at data[numberOfItems] allowed!
    //! @todo ObjectContainer::Append/Insert makes (hard to find) compiler problems if argument TDerivedClass is not a derived class of T
    void Insert(Index position, const T& item)
    {
#ifdef __EXUDYN_RUNTIME_CHECKS__
        CheckGetClone(item);
#endif
        data.Insert(position, item.GetClone());
        //! @todo unit test for ObjectContainer::Last() missing!
      }

    //! remove (and delete) item at position; move forward all elements in range(position+1,numberOfItems) ; decreases numberOfItems by one; does nothing if position == -1 ==> compatible with GetIndexOfItem(...)
    void Remove(Index position)
    {
        CHECKandTHROW(position >= 0 && position < NumberOfItems(), "ObjectContainer::Remove: invalid position");
        CHECKandTHROW(data[position] != nullptr, "ObjectContainer::Remove: invalid position pointer"); //should not happen

        delete data[position];
        data.Remove(position);
    }

    ////does not work, if we do not have a list of conecutive pointers (usually not the case!!!)
    ////calculate index of pointer to item; FAST!
    //Index GetIndexOfItemPointer(const T* itemPointer) const
    //{
    //    char* p1 = (char*)itemPointer;
    //    char* p0 = (char*)data.GetDataPointer()[0];
    //    Index s = sizeof(T*);
    //    return (p1-p0)/s;
    //    //return (Index)((char*)itemPointer - (char*)data.GetDataPointer())/sizeof(T*);
    //}

    //! search for item; return array index of FIRST item (if found), otherwise return; different return value from HOTINT1 tarray::find(...): FindIndexOfItem(...) returns EXUstd::InvalidIndex in case it is not found!!! SLOW!
     Index GetIndexOfItem(const T& item) const
    {
        for (Index j = 0; j < NumberOfItems(); j++)
        {
            if (*(data[j]) == item) return j;
        }
        return EXUstd::InvalidIndex;
    }

};


//! output stream operator for ObjectContainer; DIFFERENT FROM ResizableArray: streams each object with a line feed "\n"
//! @todo Redirect operator<< to a GetString() method in every class; operator << MUST ONLY BE implemented in base class; GetString should give Python-compatible string!
template <class T>
std::ostream& operator<<(std::ostream& os, const ObjectContainer<T>& array)
{
    for (Index i = 0; i < array.NumberOfItems(); i++) {
        os << array[i] << "\n";
    }

    return os;
}

//template specialization, as CopyFrom would not work for derived class (e.g. if LinkedDataVector would be used in ObjectContainer<Vector>)!!!
//DO NOT SPECIALIZE for CObject, because it would lead to slicing and would not copy derived class!
//class ObjectContainer<Vector>
//{
// @brief copy objects (using copy constructor and new) from given array; copy elements starting at 'beginPosition' until 'endPosition-1'
// deletes all existing objects before copying;
// ONLY available for ObjectContainer of base classes "Vector" or "ResizableArray", not using derived classes!
// 'endPosition' = EXUstd::InvalidIndex indicates a complete copy of the array up to numberOfItems;
//	indices follow std::begin/end convention; beginPosition=0, endPosition=1 copies the first element; beginPosition=0, endPosition=n copies the first n elements
//template<>
//inline void ObjectContainer<Vector>::CopyFrom(const ObjectContainer<Vector>& array, Index beginPosition,
//    Index endPosition)
//{
//    if (endPosition == EXUstd::InvalidIndex) { endPosition = array.NumberOfItems(); }
//
//    CHECKandTHROW(beginPosition >= 0, "ObjectContainer<Vector>::CopyFrom, beginPosition < 0");
//    CHECKandTHROW(endPosition <= array.NumberOfItems(), "ObjectContainer<Vector>::CopyFrom, endPosition > numberOfItems");
//
//    Flush(); //delete all existing objects
//    if (array.NumberOfItems() == 0) { return; }
//
//    data.SetNumberOfItems(endPosition - beginPosition);
//
//    Index cnt = 0;
//    for (Vector* item : array.data) //copy items with new item-by-item, using copy constructor
//    {
//        data.GetItem(cnt++) = new Vector(*item);
//    }
//
//}
//
template<>
inline void ObjectContainer<Real>::CopyFrom(const ObjectContainer<Real>& array, Index beginPosition,
    Index endPosition)
{
    if (endPosition == EXUstd::InvalidIndex) { endPosition = array.NumberOfItems(); }

    CHECKandTHROW(beginPosition >= 0, "ObjectContainer<Real>::CopyFrom, beginPosition < 0");
    CHECKandTHROW(endPosition <= array.NumberOfItems(), "ObjectContainer<Real>::CopyFrom, endPosition > numberOfItems");

    Flush(); //delete all existing objects
    if (array.NumberOfItems() == 0) { return; }

    data.SetNumberOfItems(endPosition - beginPosition);

    Index cnt = 0;
    for (Real* item : array.data) //copy items with new item-by-item, using copy constructor
    {
        data.GetItem(cnt++) = new Real(*item);
    }

}


// enable ObjectContainer with Real, for unit tests
template<>
inline void ObjectContainer<Real>::Insert(Index position, const Real& item)
{
    data.Insert(position, new Real(item));
    //! @todo unit test for ObjectContainer::Last() missing!
}

template<>
inline Index ObjectContainer<Real>::Append(const Real& item)
{
    //! @todo: check whether there exists a specific Tderived::GetClone method (derived class)
    return data.Append(new Real(item));
}


template<>
inline ObjectContainer<Real>::ObjectContainer(std::initializer_list<Real> listOfItems) //pass by value as a standard in C++11
{
    data.EnlargeMaxNumberOfItemsTo((Index)listOfItems.size());

    Index cnt = 0;
    for (const Real& item : listOfItems) {
        data[cnt++] = new Real(item);
    }
}

template<>
inline ObjectContainer<Vector>::ObjectContainer(std::initializer_list<Vector> listOfItems) //pass by value as a standard in C++11
{
    data.EnlargeMaxNumberOfItemsTo((Index)listOfItems.size());

    Index cnt = 0;
    for (const Vector& item : listOfItems) {
        data[cnt++] = new Vector(item);
    }
}

#endif
