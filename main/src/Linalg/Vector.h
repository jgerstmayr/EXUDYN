/** ***********************************************************************************************
* @class		VectorBase
* @brief		A base vector for vector math operations with memory allocated on heap; a change of size leads to new/delete
* @details		Details:
					- BaseVector: templated vector of any item T
					- Vector: a vector of Real items (depending on solver precision: double/float);
					- VectorF: a vector of float items;
					- each constructor Vector(size), operator=, operator+, operator-, etc. performs dynamic memory allocation; change of Vector size requires memory allocation
                    - for operation on large vector data
                    - use SlimVector for many short vectors
                    - use ConstSizeVector for few short vectors allocated on stack; use for temporary vectors in computation
                    - use LinkedDataVector to link data to a (part of a) vector (without memory allocation)
                    - use ResizableVector to allow a vector to allocate more data than currently needed (no memory allocation when vector size changes)
*
* @author		Gerstmayr Johannes
* @date			1997-05-15 (generated)
* @date			2018-04-30 (last modified)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use Vector for large vector sizes; uses dynamic allocation (slow)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @todo         Vector::operators, check if data[cnt++] works and is faster
*
* *** Example code ***
*
* @code{.cpp}
* Vector v1(1000);                  //create a vector with 1000 Real; uses dynamic allocation
* for (Index i = 0; i < 1000; i++) {
*   v1[i] = i;
* }
* Vector v2 = v1;                   //assign v1 to v2
* v1 += v2;                         //add v2 to v1
* cout << v1 << "\n";               //write v1 to cout
* @endcode
************************************************************************************************ */
#ifndef VECTORBASE__H
#define VECTORBASE__H

#include <initializer_list> //for initializer_list in constructor
#include <ostream>
//#include <stdlib.h> //only works in MSVC for initialization with std::vector
#include <array>
#include <vector>

//#include <cmath> //for sqrt
#include <utility> //for sqrt

//#include "Utilities/ReleaseAssert.h"
//#include "Utilities/BasicDefinitions.h" //defines Real
#include "Utilities/BasicFunctions.h"   //for Minimum


#ifdef __EXUDYN_RUNTIME_CHECKS__
extern Index vector_new_counts; //global counter of item allocations; is increased every time a new is called
extern Index linkedDataVectorCast_counts; //global counter for unwanted type conversion from LinkedDataVector to Vector
extern Index vector_delete_counts; //global counter of item deallocations; is increased every time a delete is called
#endif

typedef std::vector<Real> StdVector; //needed for user functions

enum class VectorType {
	Vector = 1,
	LinkedDataVector = 2,
	ResizableVector = 3,
	ConstVector = 4
};

template <typename T, Index dataSize> class SlimVectorBase;
template <typename T> class LinkedDataVectorBase;

template<typename T>
class VectorBase
{
protected:
    mutable T* data;		//!< pointer to data containing Reals; in derived class pointer to linked data
    Index numberOfItems;	//!< currently used number of Reals; represents size of VectorBase (equivalent to numberOfPReals in VectorX)

    //! constructor which links data, no memory allocation; ONLY for ConstDataVector and LinkedDataVector
    //VectorBase(T* dataPointer, Index numberOfRealsInit): data(dataPointer), numberOfItems(numberOfRealsInit) {};

public:
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CONSTRUCTOR, DESTRUCTOR
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! Default constructor: no memory allocation!
	VectorBase() : data(nullptr), numberOfItems(0) {};


    //! Allocate numberOfItemsInit Reals in memory; no data initialization (data[...] = undefined!)!!!
	VectorBase(Index numberOfItemsInit);

    //! Allocate numberOfItemsInit Reals in memory; assign all data items with 'initializationValue'
	VectorBase(Index numberOfItemsInit, T initializationValue);

    //! copy constructor; memory allocation!
	VectorBase(const VectorBase& vector);

	VectorBase(VectorBase&& other) noexcept :
		data(std::exchange(other.data, nullptr)),
		numberOfItems(std::exchange(other.numberOfItems, 0)) {}

private:
	VectorBase(LinkedDataVectorBase<T>&& other) = delete; //this move constructor is forbidden, as it will lead to crash as memory will be deleted wrongly
	VectorBase(const LinkedDataVectorBase<T>& other) = delete; //this copy constructor is forbidden, as it will unintendedly copy memory
	VectorBase(std::initializer_list<Index>) = delete; //constructor forbidden, as it would convert wrongly for Vector({2}) into Vector(2)
public:

	//! constructor with std::vector
	VectorBase(const std::vector<T> vector);
	
	//! constructor with initializer list; memory allocation!
    //! @todo check if data[cnt++] is faster than (*this)[cnt++]
	VectorBase(std::initializer_list<T> listOfReals);

    //! constructor with SlimVector; VALUES ARE LINKED; memory allocation ==> SLOW!
	//  use e.g. to create common interfaces to pybind ==> do not use in computation!
	//template<Index dataSize>
	//VectorBase(const SlimVectorBase<T,dataSize>& vector)
	//{
	//	AllocateMemory(vector.NumberOfItems());
	//	
	//	Index cnt = 0;
	//	for (auto value : vector) {
	//		data[cnt++] = value;
	//	}
	//}

    //! constructor with ResizableArray; VALUES ARE LINKED; memory allocation!
    //VectorBase(const ResizableArray<T>& array) { SetVector(array); } //makes only sense for LinkedDataVector; if you want to copy, assign this to a vector!

    //! constructor with pointer to (c-)array of Reals; VALUES ARE COPIED; memory allocation!
	//  better to use LinkedDataVector to avoid copies!
	//  must initialize VectorBase first, before call of SetVector
    VectorBase(Index numberOfItemsInit, T* arrayOfReals): numberOfItems(0), data(nullptr)
	{ 
		SetVector(numberOfItemsInit, arrayOfReals); 
	}

    virtual ~VectorBase()
    {
        FreeMemory();
    };

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // BASIC FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    //! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
    virtual VectorBase* GetClone() const { return new VectorBase<T>(*this); }

	virtual VectorType GetType() const { return VectorType::Vector; }

protected:
    //! allocate memory if numberOfRealsInit!=0; set data to allocated array of Reals or to nullptr
	virtual void AllocateMemory(Index numberOfRealsInit)
    {
        numberOfItems = numberOfRealsInit;
        if (numberOfItems == 0) { data = nullptr; }//for case that list is zero length? ==> TEST CASE
        else
        {
            data = new T[numberOfItems];
#ifdef __EXUDYN_RUNTIME_CHECKS__
			vector_new_counts++;
#endif
		}
    }

    //! free memory if data!=nullptr
    virtual void FreeMemory()
    {
        if (data != nullptr)
        {
            delete[] data;
            data = nullptr;
#ifdef __EXUDYN_RUNTIME_CHECKS__
			vector_delete_counts++;
#endif
		}
    }

public:
    //the iterator functions are declared virtual in order to override in ConstSizeVector (CHECK PERFORMANCE)
    //virtual T* begin() { return &data[0]; }						//!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
    //virtual const T* begin() const { return &data[0]; }			//!< C++11 std::begin() for iterators, const version needed for ==, +=, etc.; iterator range is always the currently used numberOfItems.
	//virtual const T* begin() const { return data; }				    //!< C++11 std::begin() for iterators, const version needed for ==, +=, etc.; iterator range is always the currently used numberOfItems.
	//virtual const T* end() const { return &data[numberOfItems]; }	//!< C++11 std::end() for iterators, const version needed for ==, +=, etc.; iterator range is always the currently used numberOfItems.
	
	//OLD: until 2021-07-04:
	//virtual T* begin() const { return data; }							    //!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
	//virtual T* end() const { return &data[numberOfItems]; }				//!< C++11 std::end() for iterators; iterator range is always the currently used numberOfItems.
	
	//T* begin() const { return &data[0]; }					    //!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
	T* begin() const { return data; }					    //!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
	T* end() const { return &data[numberOfItems]; }				//!< C++11 std::end() for iterators; iterator range is always the currently used numberOfItems.

    Index NumberOfItems() const { return numberOfItems; }	            //!< Number of currently used Reals; WILL BE DIFFERENT in ResizableVector and in VectorX
	bool IsValidIndex(Index index) const { return (index >= 0) && (index < NumberOfItems()); } 	//!< check if an index is in range of valid items

	T* GetDataPointer() const { return data; }                       //!< return pointer to first data containing T numbers; const needed for LinkedDataVectors.

    //! set a new numberOfItems for vector; if (numberOfItems==newSize) ==> do nothing; ALL DATA GETS LOST in case of resize!
    virtual void SetNumberOfItems(Index newNumberOfItems)
    {
        if (newNumberOfItems != NumberOfItems())
        {
            FreeMemory();
            AllocateMemory(newNumberOfItems);
        }
    }

	//! reset to zero size and free memory (in derived classes!)
	virtual void Reset()
	{
		FreeMemory();
		numberOfItems = 0;
		data = nullptr; 
	}

	//! set all Reals to given value.
    void SetAll(T value)
    {
        for (auto &item : *this) {
            item = value;
        }
    }

    //! set vector to data given by initializer list; possibly memory allocation!
    void SetVector(std::initializer_list<T> listOfReals)
    {
        SetNumberOfItems((Index)listOfReals.size());

        Index cnt = 0;
        for (auto value : listOfReals) {
            data[cnt++] = value;
        }
    }

	void SetVector(Index numberOfItems, T* arrayOfReals)
	{
		SetNumberOfItems(numberOfItems);

		Index cnt = 0;
		for (auto &item : *this) {
			item = arrayOfReals[cnt++];
		}
	}

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // OPERATORS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //VectorBase& operator= (T scalarValue) //would be nice for compatibility with SlimVector, but is dangerous to use for VectorBase (which size?)

	//unsafe Referencing access-operator, ZERO-based, without CHECKS
	T& GetUnsafe(Index item)
	{
		return data[item];
	};

	//unsafe const Referencing access-operator, ZERO-based, without CHECKS
	const T& GetUnsafe(Index item) const
	{
		return data[item];
	};


    //Referencing access-operator, ZERO-based
    T& operator[](Index item)
    {
		CHECKandTHROW((item >= 0) && (item < numberOfItems), "VectorBase::operator[] const: request of invalid item");
		return data[item];
    };

    //const Referencing access-operator, ZERO-based
    const T& operator[](Index item) const
    {
		CHECKandTHROW((item >= 0) && (item < numberOfItems), "VectorBase::operator[] const: request of invalid item");
		return data[item];
    };

    //! @brief copy assignment operator; copies the currently used entries (LinkedDataVector; ResizableVector)
    //! @todo VectorBase::operator=, check if memcopy is faster
    VectorBase& operator=(const VectorBase& vector)
    {
        if (this == &vector) { return *this; }

        SetNumberOfItems(vector.NumberOfItems());

        Index cnt = 0;
        for (auto item : vector) {
            (*this)[cnt++] = item;
        }
        return *this;
    }

	//! copy assignment operator for SlimVector
	template <Index dataSize>
	VectorBase& operator=(const SlimVectorBase<T, dataSize>& vector)
	{
		SetNumberOfItems(vector.NumberOfItems());

		Index cnt = 0;
		for (auto item : vector) {
			(*this)[cnt++] = item;
		}
		return *this;
	}


    //! comparison operator, component-wise compare; returns true, if all components are equal
    bool operator==(const VectorBase& v) const
    {
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "VectorBase::operator==: incompatible size of vectors");
        Index cnt = 0;
        for (auto item : v)
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

	//! add Tvector v to *this vector (for each component); both vectors must have same size
	template <class Tvector>
	VectorBase& operator+=(const Tvector& v)
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "VectorBase::operator+=(Tvector): incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			(*this)[cnt++] += item;
		}
		return *this;
	}
	////! add ConstSizeVectorBase v to *this vector (for each component); both vectors must have same size
	//template <Index dataSize>
	//VectorBase& operator+=(const ConstSizeVectorBase<T, dataSize>& v)
	//{
	//	CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "VectorBase::operator+=(ConstSizeVectorBase): incompatible size of vectors");
	//	Index cnt = 0;
	//	for (auto item : v) {
	//		(*this)[cnt++] += item;
	//	}
	//	return *this;
	//}

	//! substract vector v from *this vector (for each component); both vectors must have same size
	template <class Tvector>
	VectorBase& operator-=(const Tvector& v)
	//VectorBase& operator-=(const VectorBase& v)
    {
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "VectorBase::operator-=: incompatible size of vectors");
        Index cnt = 0;
        for (auto item : v) {
            (*this)[cnt++] -= item;
        }
        return *this;
    }

	//! subtract SlimVector v from *this vector (for each component); both vectors must have same size
	template <Index dataSize>
	VectorBase& operator-=(const SlimVectorBase<T, dataSize>& v)
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "VectorBase::operator-=(SlimVectorBase): incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			(*this)[cnt++] -= item;
		}
		return *this;
	}

	//! scalar multiply vector *this with scalar (for each component)
    VectorBase& operator*=(T scalar)
    {
        for (auto &item : *this) {
            item *= scalar;
        }
        return *this;
    }

    //! scalar division of vector v through scalar (for each component)
    VectorBase& operator/=(T scalar)
    {
        for (auto &item : *this) {
            item /= scalar;
        }
        return *this;
    }

    //! add two vectors, result = v1+v2 (for each component)
	friend VectorBase operator+(const VectorBase& v1, const VectorBase& v2)
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
	friend VectorBase operator-(const VectorBase& v1, const VectorBase& v2)
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
	friend VectorBase operator*(const VectorBase& v, T scalar)
	{
		VectorBase<T> result(v.NumberOfItems());
		Index cnt = 0;
		for (auto &item : result) {
			item = scalar * v[cnt++];
		}
		return result;
	}

    //! scalar multiply, result = v * scalar (for each component)
	friend VectorBase operator*(T scalar, const VectorBase& v)
	{
		VectorBase<T> result(v.NumberOfItems());
		Index cnt = 0;
		for (auto &item : result) {
			item = scalar * v[cnt++];
		}
		return result;
	}

    //! scalar product, result = v1 * v2 (scalar result)
    friend T operator*(const VectorBase& v1, const VectorBase& v2)
    {
		CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "T VectorBase::operator*: incompatible size of vectors");
        T result = 0;
        Index cnt = 0;
        for (auto &item : v1) {
            result += item * v2[cnt++];
        }
        return result;
    }

    //! @brief Output operator << generates ostream "[v[0] v[1] .... v[dataSize-]]" for a vector v;
    //! the FORMAT IS DIFFERENT TO HOTINT1 ==> no separating comma ','
	//! VS2017: leads to linker error, if put into .cpp file
	friend std::ostream& operator<<(std::ostream& os, const VectorBase& v)
	{
		char s = ' ';
		if (linalgPrintUsePythonFormat) { s = ','; }
		os << "[";
		for (Index i = 0; i < v.NumberOfItems(); i++) {
			os << v[i];
			if (i < v.NumberOfItems() - 1) { os << s; }
		}
		os << "]";
		return os;
	}


	//! conversion of VectorBase into std::vector (e.g. for usage in pybind11)
	operator std::vector<T>() const { return std::vector<T>(begin(), end()); }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // EXTENDED FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! add vector scalar * v to *this vector
	template<class Tvector>
	void MultAdd(T scalar, const Tvector& v)
	{
		CHECKandTHROW((v.NumberOfItems() == NumberOfItems()), "VectorBase::MultAdd: incompatible size of vectors");
		for (Index i = 0; i < NumberOfItems(); i++)
		{
			data[i] += scalar * v[i];
		}
	}

	//! multiply components of this vector with components of other vector
	template<class Tvector>
	void MultComponentWise(const Tvector& v)
	{
		CHECKandTHROW((v.NumberOfItems() == NumberOfItems()), "VectorBase::MultComponentWise: incompatible size of vectors");
		for (Index i = 0; i < NumberOfItems(); i++)
		{
			data[i] *= v[i];
		}
	}

	//! add vector scalar * v to *this vector
	//large vector: use AVX
	//huge vector: use AVX + multithreading
	//may not be used if LinkedDataVector is linked to unaligned part of vector
	//template<class Tvector, bool largeVector = false, bool hugeVector = false>
	//void MultAdd(T scalar, const Tvector& v)
	//{
	//	copy first part of this function to LinkedDataVector to avoid operation on non-aligned data!

	//	CHECKandTHROW((v.NumberOfItems() == NumberOfItems()), "VectorBase::MultAdd: incompatible size of vectors");
	//	if constexpr (largeVector == false || exuVectorLengthAlignment == 1)
	//	{
	//		for (Index i = 0; i < NumberOfItems(); i++)
	//		{
	//			data[i] += scalar * v[i];
	//		}
	//	}
	//	else
	//	{
	//		for (Index i = 0; i < NumberOfItems(); i++)
	//		{
	//			data[i] += scalar * v[i];
	//		}

	//		Index nShift = NumberOfItems() / exuVectorLengthAlignment;
	//		PReal* vP = (PReal*)v.GetDataPointer();
	//		PReal* dataP = (PReal*)data.GetDataPointer();
	//		for (Index i = 0; i < (nShift*(exuVectorLengthAlignment/exuMemoryAlignment)); i+= (exuVectorLengthAlignment / exuMemoryAlignment))
	//		{
	//			//optimize for latency; using 4 PReal speeds up only slightly
	//			PReal a = dataP[i] + scalar * vP[i];
	//			PReal b = dataP[i + 1] + scalar * vP[i + 1];
	//			dataP[i] = a;
	//			dataP[i + 1] = b;
	//		}
	//		for (Index i = nShift * exuVectorLengthAlignment; i < NumberOfItems(); i++)
	//		{
	//			data[i] += scalar * v[i];
	//		}
	//	}
	//}


    //! returns the sum of squared components (v[0]^2 + v[1]^2 + v[2]^2 ....)
    T GetL2NormSquared() const
    {
        T result = 0.;
        for (auto item : *this) { result += item * item; }
        return result;
    }

    //! returns the square norm of a vector
    T GetL2Norm() const
    {
        return sqrt(GetL2NormSquared());
    }

    //! normalizes the vector; divide each component by vector square norm
    void Normalize()
    {
        T norm = GetL2Norm();
		CHECKandTHROW(norm != 0., "VectorBase::Normalized() called with GetL2Norm() == 0.");
		norm = 1 / norm; //if T=int, this would not work but anyway outcome would be int ...!

        for (auto &item : *this) { item *= norm; }
 	}

	//! copy numberOfCopiedItems items of a vector at vectorPosition to VectorBase(*this) at thisPosition, 
	template<class Tvector>
	void CopyFrom(const Tvector& vector, Index vectorPosition, Index thisPosition, Index numberOfCopiedItems)
	{
		//CHECKandTHROW((vectorPosition >= 0), "VectorBase::CopyFrom(...): vectorPosition < 0");
		//CHECKandTHROW((thisPosition >= 0), "VectorBase::CopyFrom(...): thisPosition < 0");
		CHECKandTHROW((thisPosition + numberOfCopiedItems <= NumberOfItems()), "VectorBase::CopyFrom(...): thisPosition index mismatch");
		CHECKandTHROW((vectorPosition + numberOfCopiedItems <= vector.NumberOfItems()), "VectorBase::CopyFrom(...): vectorPosition index mismatch");

		for (Index i = 0; i < numberOfCopiedItems; i++)
		{
			this->GetUnsafe(i + thisPosition) = vector.GetUnsafe(i + vectorPosition);
			//this->GetUnsafe(i + thisPosition) = vector[i + vectorPosition];
		}
	}

	//! copy from other std::vector
	void CopyFrom(const StdVector& vector)
	{
		SetNumberOfItems((Index)vector.size());

		//may be faster => test ...
		//std::copy(vector.begin(), vector.end(), this->begin());

		Index cnt = 0;
		for (Real val : vector) 
		{
			this->GetUnsafe(cnt++) = val;
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
	
	//! append a vector to VectorBase(*this) and return result=[*this,vector]; does not modify *this
	//VectorBase Append(const VectorBase& vector) const;
	VectorBase<T> Append(const VectorBase<T>& vector) const;


    //! Returns the sum of all components of a vector in range [0, numberOfItems]
    T Sum() const
    {
        T sum = 0;
        for (auto item : *this) { sum += item; }
        return sum;
    }

	//! Returns the minimum of all components of a vector in range [0, numberOfItems]
	T Minimum() const
	{
		T min = EXUstd::_MAXFLOAT;
		for (auto item : *this) { min = EXUstd::Minimum(min, item); }
		return min;
	}

	//! Returns the minimum of all components of a vector in range [0, numberOfItems]
	T Maximum() const
	{
		T max = EXUstd::_MINFLOAT;
		for (auto item : *this) { max = EXUstd::Maximum(max, item); }
		return max;
	}
};

//! Allocate numberOfItemsInit Reals in memory; no data initialization (data[...] = undefined!)!!!
template<typename T>
VectorBase<T>::VectorBase(Index numberOfItemsInit)
{
	AllocateMemory(numberOfItemsInit);
}

//! Allocate numberOfItemsInit Reals in memory; assign all data items with 'initializationValue'
template<typename T>
VectorBase<T>::VectorBase(Index numberOfItemsInit, T initializationValue)
{
	AllocateMemory(numberOfItemsInit);

	//Index cnt = 0;
	for (auto &value : *this) {
		value = initializationValue;
	}
}

//! copy constructor; memory allocation!
template<typename T>
VectorBase<T>::VectorBase(const VectorBase<T>& vector)
{
#ifdef __EXUDYN_RUNTIME_CHECKS__
	if (vector.GetType() == VectorType::LinkedDataVector)
	{
		linkedDataVectorCast_counts++;
	}
#endif

	AllocateMemory(vector.NumberOfItems());

	Index cnt = 0;
	for (auto value : vector) {
		data[cnt++] = value;
	}
}

//! constructor with std::vector
template<typename T>
VectorBase<T>::VectorBase(const std::vector<T> vector)
{
	AllocateMemory((Index)vector.size());

	std::copy(vector.begin(), vector.end(), this->begin());
}

//! constructor with initializer list; memory allocation!
//! @todo check if data[cnt++] is faster than (*this)[cnt++]
template<typename T>
VectorBase<T>::VectorBase(std::initializer_list<T> listOfReals)
{
	AllocateMemory((Index)listOfReals.size());

	Index cnt = 0;
	for (auto value : listOfReals) {
		//(*this)[cnt++] = value;
		data[cnt++] = value; //faster???
	}
}

typedef VectorBase<Real> Vector;
typedef VectorBase<float> VectorF; //always float, used for graphics

#endif
