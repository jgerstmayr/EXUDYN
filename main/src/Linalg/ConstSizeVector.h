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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
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
#ifndef CONSTSIZEVECTORBASE__H
#define CONSTSIZEVECTORBASE__H

#include "Linalg/Vector.h"

#ifdef USE_NEW_CONSTSIZEVECTOR

template<typename T, Index dataSize>
class ConstSizeVectorBase
{
protected:
	mutable T data[dataSize];
	Index numberOfItems;	//!< currently used number of Reals

public:
	//! default constructor, calls special constructor if vector for linking data, no allocation; no data item initialization (undefined values), rule of 5
	//ConstSizeVectorBase() //don't use, rule of 5
	//{
	//}

	//! initialize ConstSizeVectorBase if init==true, using default numberOfItems = dataSize, otherwise nothing
	ConstSizeVectorBase(bool init = true)
	{
		if (init) { numberOfItems = dataSize; }
	}

	//! initialize ConstSizeVectorBase with numberOfItemsInit <= dataSize; no data item initialization (undefined values)
	ConstSizeVectorBase(Index numberOfItemsInit) 
	{
		CHECKandTHROW(numberOfItemsInit <= dataSize, "ERROR: call to ConstSizeVectorBase(Index): dataSize mismatch");
		numberOfItems = numberOfItemsInit;
	}

	//! initialize ConstSizeVectorBase with numberOfItemsInit Reals; assign all data items with 'initializationValue'
	ConstSizeVectorBase(Index numberOfItemsInit, T initializationValue)
	{
		CHECKandTHROW(numberOfItemsInit <= dataSize, "ERROR: call to ConstSizeVectorBase(Index): dataSize mismatch");

		numberOfItems = numberOfItemsInit;

		//Index cnt = 0;
		for (auto &value : *this) {
			value = initializationValue;
		}
	}

	//! constructor with initializer list; condition: listOfReals.size() <= dataSize
	ConstSizeVectorBase(std::initializer_list<T> listOfReals) //pass by value as a standard in C++11
	{
		CHECKandTHROW((Index)listOfReals.size() <= dataSize, "ERROR: ConstSizeVectorBase::constructor, dataSize mismatch with initializer_list");

		numberOfItems = (Index)listOfReals.size();

		Index cnt = 0;
		for (auto val : listOfReals) {
			data[cnt++] = val;
		}
	}

private:
	ConstSizeVectorBase(std::initializer_list<Index>) = delete; //constructor forbidden, as it would convert wrongly for ConstSizeVector({2}) into ConstSizeVector(2)
public:

	////! copy constructor; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items], rule of 5
	//ConstSizeVectorBase(const ConstSizeVectorBase<T, dataSize>& vector)
	//{
	//	numberOfItems = vector.numberOfItems;

	//	Index cnt = 0;
	//	for (auto value : vector) {
	//		data[cnt++] = value;
	//	}
	//}

	//! move constructor, rule of 5 (but does not work yet)
	//ConstSizeVectorBase(ConstSizeVectorBase<T, dataSize>&& other) = default; 
	//ConstSizeVectorBase(ConstSizeVectorBase<T, dataSize>&& other) noexcept
	//{
	//	data = &data[0];

	//	Index cnt = 0;
	//	for (auto val : other)
	//	{
	//		data[cnt++] = std::exchange(val, (T)0.);
	//	}
	//	numberOfItems = std::exchange(other.numberOfItems, 0);
	//}

	//! @brief Initialize ConstSizeVectorBase by data given from vector at startPositionVector; 
	//! copies 'dataSize' items, independently of array size (might cause memory access error)
	ConstSizeVectorBase(const ConstSizeVectorBase& vector, Index startPositionVector)
	{
		CHECKandTHROW(startPositionVector >= 0, "ERROR: ConstSizeVectorBase(const ConstSizeVectorBase<T>&, Index), startPositionVector < 0");
		CHECKandTHROW(dataSize + startPositionVector <= vector.NumberOfItems(), "ERROR: ConstSizeVectorBase(const ConstSizeVectorBase<T>&, Index), dataSize mismatch");

		numberOfItems = dataSize;

		Index cnt = startPositionVector;
		for (auto &value : *this) {
			value = vector.data[cnt++];
		}
	}

	//! @brief Initialize ConstSizeVectorBase by data given from vector at startPositionVector; 
	//! copies 'dataSize' items, independently of array size (might cause memory access error)
	ConstSizeVectorBase(const VectorBase<T>& vector, Index startPositionVector)
	{
		CHECKandTHROW(startPositionVector >= 0, "ERROR: ConstSizeVectorBase(const VectorBase<T>&, Index), startPositionVector < 0");
		CHECKandTHROW(dataSize + startPositionVector <= vector.NumberOfItems(), "ERROR: ConstSizeVectorBase(const VectorBase<T>&, Index), dataSize mismatch");

		numberOfItems = dataSize;

		Index cnt = startPositionVector;
		for (auto &value : *this) {
			value = vector.GetUnsafe(cnt++);
		}
	}

	////! rule of 5
	//~ConstSizeVectorBase()
	//{
	//};

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// BASIC FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	VectorType GetType() const { return VectorType::ConstVector; }

	Index MaxNumberOfItems() const { return dataSize; }             //!< Get dataSize (available memory) of ConstSizeVectorBase

	//! call to ConstSizeVectorBase::SetNumberOfItems leads to run-time error if newNumberOfItems > dataSize; used in SetVector({...}) and in operator=; does not reset data
	void SetNumberOfItems(Index newNumberOfItems)
	{
		CHECKandTHROW(newNumberOfItems <= dataSize, "ERROR: call to ConstSizeVectorBase::SetNumberOfItems with newNumberOfItems > dataSize");
		numberOfItems = newNumberOfItems;
	}

	//! reset to zero size; no memory delete!
	void Reset()
	{
		numberOfItems = 0;
	}

	T* begin() const { return data; }					    //!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
	T* end() const { return &data[numberOfItems]; }				//!< C++11 std::end() for iterators; iterator range is always the currently used numberOfItems.

	Index NumberOfItems() const { return numberOfItems; }	            //!< Number of currently used Reals; WILL BE DIFFERENT in ResizableVector and in VectorX
	T* GetDataPointer() const { return &data[0]; }                       //!< return pointer to first data containing T numbers; const needed for LinkedDataVectors.

	//! set numberOfItems Reals to given value.
	void SetAll(T value)
	{
		for (auto &item : *this) {
			item = value;
		}
	}

	//! set vector to data given by initializer list
	void SetVector(std::initializer_list<T> listOfReals)
	{
		SetNumberOfItems((Index)listOfReals.size());

		Index cnt = 0;
		for (auto value : listOfReals) {
			data[cnt++] = value;
		}
	}

	//! set vector to data given by initializer list; uses copy!
	void SetVector(Index numberOfItems, T* arrayOfReals)
	{
		SetNumberOfItems(numberOfItems);

		Index cnt = 0;
		for (auto &item : *this) {
			item = arrayOfReals[cnt++];
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// OPERATORS: some operators need to be overwritten because of vector generation
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//Referencing access-operator, ZERO-based
	T& operator[](Index item)
	{
		CHECKandTHROW((item >= 0) && (item < numberOfItems), "ConstSizeVectorBase::operator[]: request of invalid item");
		return data[item];
	};

	//Referencing access-operator, ZERO-based
	const T& operator[](Index item) const
	{
		CHECKandTHROW((item >= 0) && (item < numberOfItems), "ConstSizeVectorBase::operator[] const: request of invalid item");
		return data[item];
	};

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


	////! @brief copy assignment operator; copies the currently used entries in range [0,numberOfItems]; rule of 5
	//ConstSizeVectorBase<T, dataSize>& operator= (const ConstSizeVectorBase<T, dataSize>& vector)
	//{
	//	if (this == &vector) { return *this; }

	//	//no check necessary, because only possible for equal datasizes!!!
	//	//CHECKandTHROW(vector.NumberOfItems() <= dataSize, "ERROR: call to ConstSizeVectorBase::operator= with vector.dataSize > dataSize");
	//	numberOfItems = vector.NumberOfItems();

	//	Index cnt = 0;
	//	for (auto item : vector) {
	//		data[cnt++] = item;
	//	}
	//	return *this;
	//}

	//fails to complile, compiler wants to use move assignment operator:
//private:
//	ConstSizeVectorBase<T, dataSize>& operator= (ConstSizeVectorBase<T, dataSize>&& other) = delete;
//public:
	//! move assignment operator, rule of 5 (does not work in such a way!)
	//ConstSizeVectorBase<T, dataSize>& operator= (ConstSizeVectorBase<T, dataSize>&& other) noexcept
	//{
	//	//no check necessary, because only possible for equal datasizes!!!
	//	//CHECKandTHROW((other.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator=: incompatible size of vectors: dataSize");

	//	for (Index i = 0; i < other.NumberOfItems(); i++) { //copy only items
	//		std::swap(data[i], other.data[i]);
	//	}

	//	std::swap(numberOfItems, other.numberOfItems);
	//	return *this;
	//}


	//! copy assignment operator for SlimVector
	ConstSizeVectorBase& operator=(const SlimVectorBase<T, dataSize>& vector)
	{
		SetNumberOfItems(vector.NumberOfItems());

		Index cnt = 0;
		for (auto item : vector) {
			data[cnt++] = item;
		}
		return *this;
	}


	//! comparison operator, component-wise compare; returns true, if all components are equal
	bool operator==(const ConstSizeVectorBase& v) const
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "ConstSizeVectorBase::operator==: incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v)
		{
			if (item != data[cnt++]) { return false; }
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

	//! add vector v to *this vector (for each component); both vectors must have same size
	ConstSizeVectorBase& operator+=(const ConstSizeVectorBase& v)
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "ConstSizeVectorBase::operator+=: incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			data[cnt++] += item;
		}
		return *this;
	}

	//! add SlimVector v to *this vector (for each component); both vectors must have same size
	ConstSizeVectorBase& operator+=(const SlimVectorBase<T, dataSize>& v)
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "ConstSizeVectorBase::operator+=(SlimVectorBase): incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			data[cnt++] += item;
		}
		return *this;
	}

	//! add VectorBase v to *this vector (for each component); both vectors must have same size
	ConstSizeVectorBase& operator+=(const VectorBase<T>& v)
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "ConstSizeVectorBase::operator+=(VectorBase): incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			data[cnt++] += item;
		}
		return *this;
	}

	//! substract vector v from *this vector (for each component); both vectors must have same size
	ConstSizeVectorBase& operator-=(const ConstSizeVectorBase& v)
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "ConstSizeVectorBase::operator-=: incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			data[cnt++] -= item;
		}
		return *this;
	}

	//! subtract SlimVector v from *this vector (for each component); both vectors must have same size
	ConstSizeVectorBase& operator-=(const SlimVectorBase<T, dataSize>& v)
	{
		CHECKandTHROW((NumberOfItems() == v.NumberOfItems()), "ConstSizeVectorBase::operator-=(SlimVectorBase): incompatible size of vectors");
		Index cnt = 0;
		for (auto item : v) {
			data[cnt++] -= item;
		}
		return *this;
	}

	//! scalar multiply vector *this with scalar (for each component)
	ConstSizeVectorBase& operator*=(T scalar)
	{
		for (auto &item : *this) {
			item *= scalar;
		}
		return *this;
	}

	//! scalar division of vector v through scalar (for each component)
	ConstSizeVectorBase& operator/=(T scalar)
	{
		for (auto &item : *this) {
			item /= scalar;
		}
		return *this;
	}


	//! add two vectors, result = v1+v2 (for each component); only operated in range [0,numberOfItems]
	friend ConstSizeVectorBase operator+ (const ConstSizeVectorBase<T, dataSize>& v1, const ConstSizeVectorBase<T, dataSize>& v2)
	{
		CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator+: incompatible size of vectors");
		//not necessary, because impossible: CHECKandTHROW((v1.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator+: incompatible size of vectors: dataSize");

		ConstSizeVectorBase result(v1.NumberOfItems());
		Index cnt = 0;
		for (auto &item : result) {
			item = v1.data[cnt] + v2.data[cnt];
			cnt++;
		}
		return result;
	}

	//! add two vectors, result = v1-v2 (for each component); only operated in range [0,numberOfItems]
	friend ConstSizeVectorBase<T, dataSize> operator- (const ConstSizeVectorBase<T, dataSize>& v1, const ConstSizeVectorBase<T, dataSize>& v2)
	{
		CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator-: incompatible size of vectors");
		//not necessary, because impossible: CHECKandTHROW((v1.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator-: incompatible size of vectors: dataSize");
		ConstSizeVectorBase<T, dataSize> result(v1.NumberOfItems());
		Index cnt = 0;
		for (auto &item : result) {
			item = v1.data[cnt] - v2.data[cnt];
			cnt++;
		}
		return result;
	}

	//! scalar multiply two vectors, result = v1*v2; only operated in range [0,numberOfItems]
	friend T operator* (const ConstSizeVectorBase<T, dataSize>& v1, const ConstSizeVectorBase<T, dataSize>& v2)
	{
		CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator*: incompatible size of vectors");

		T result = 0;
		for (Index i = 0; i < v1.NumberOfItems(); i++)
		{
			result += v1.data[i] * v2.data[i];
		}
		return result;
	}

	//! scalar multiply two vectors, result = v1*v2; only operated in range [0,numberOfItems]
	template<class Tvector>
	friend T operator* (const ConstSizeVectorBase<T, dataSize>& v1, const Tvector& v2)
	{
		CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator*: incompatible size of vectors");

		T result = 0;
		for (Index i = 0; i < v1.NumberOfItems(); i++)
		{
			result += v1.data[i] * v2.GetUnsafe(i);
		}
		return result;
	}

	//! scalar multiply two vectors, result = v1*v2; only operated in range [0,numberOfItems]
	template<class Tvector>
	friend T operator* (const Tvector& v1, const ConstSizeVectorBase<T, dataSize>& v2)
	{
		CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator*: incompatible size of vectors");

		T result = 0;
		for (Index i = 0; i < v1.NumberOfItems(); i++)
		{
			result += v1.GetUnsafe(i) * v2.data[i];
		}
		return result;
	}

	////! scalar multiply two vectors, result = v1*v2; only operated in range [0,numberOfItems]
	//T operator* (const ConstSizeVectorBase<T, dataSize>& v2)
	//{
	//	CHECKandTHROW((NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator*: incompatible size of vectors");

	//	T result = 0;
	//	for (Index i = 0; i < NumberOfItems(); i++)
	//	{
	//		result += data[i] * v2.data[i];
	//	}
	//	return result;
	//}

	//! scalar multiply, result = scalar * v (for each component); only operated in range [0,numberOfItems]
	friend ConstSizeVectorBase<T, dataSize> operator* (const ConstSizeVectorBase<T, dataSize>& v, T scalar)
	{
		//CHECKandTHROW((v.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator* (scalar * v): incompatible size of vectors");
		ConstSizeVectorBase<T, dataSize> result(v.NumberOfItems());
		Index cnt = 0;
		for (auto &item : result) {
			item = scalar * v.data[cnt++];
		}
		return result;
	}

	//! scalar multiply, result = v * scalar (for each component); only operated in range [0,numberOfItems]
	friend ConstSizeVectorBase<T, dataSize> operator* (T scalar, const ConstSizeVectorBase<T, dataSize>& v)
	{
		//CHECKandTHROW((v.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator* (v * scalar): incompatible size of vectors");
		ConstSizeVectorBase<T, dataSize> result(v.NumberOfItems());
		Index cnt = 0;
		for (auto &item : result) {
			item = scalar * v.data[cnt++];
		}
		return result;
	}

	//! append a ConstSizeVectorBase<n> to ConstSizeVectorBase<n>; both vectors must have same templated length (but could have different actual length); RETURNS a ConstSizeVectorBase<2*n>
	auto Append(const ConstSizeVectorBase<T, dataSize>& vector) const
	{
		//constexpr Index size = dataSize;
		ConstSizeVectorBase<T, 2 * dataSize> newVector(NumberOfItems() + vector.NumberOfItems()); //NumberOfItems might be smaller than (dataSize1+dataSize2)
		newVector.CopyFrom(*this, 0, 0, NumberOfItems());
		newVector.CopyFrom(vector, 0, NumberOfItems(), vector.NumberOfItems());
		return newVector;
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! @brief Output operator << generates ostream "[v[0] v[1] .... v[dataSize-]]" for a vector v;
	//! the FORMAT IS DIFFERENT TO HOTINT1 ==> no separating comma ','
	//! VS2017: leads to linker error, if put into .cpp file
	friend std::ostream& operator<<(std::ostream& os, const ConstSizeVectorBase& v)
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


	//! conversion of ConstSizeVectorBase into std::vector (e.g. for usage in pybind11)
	operator std::vector<T>() const { return std::vector<T>(begin(), end()); }

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// EXTENDED FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! add vector scalar * v to *this vector
	void MultAdd(T scalar, const ConstSizeVectorBase& v)
	{
		CHECKandTHROW((v.NumberOfItems() == NumberOfItems()), "ConstSizeVectorBase::MultAdd: incompatible size of vectors");
		for (Index i = 0; i < NumberOfItems(); i++)
		{
			data[i] += scalar * v.data[i];
		}
	}
	
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
		CHECKandTHROW(norm != 0., "ConstSizeVectorBase::Normalized() called with GetL2Norm() == 0.");

		for (auto &item : *this) { item /= norm; }
	}

	//! copy numberOfCopiedItems items of a vector at vectorPosition to ConstSizeVectorBase(*this) at thisPosition, does not initialize!
	void CopyFrom(const ConstSizeVectorBase& vector, Index vectorPosition, Index thisPosition, Index numberOfCopiedItems)
	{
		//CHECKandTHROW((vectorPosition >= 0), "ConstSizeVectorBase::CopyFrom(...): vectorPosition < 0");
		//CHECKandTHROW((thisPosition >= 0), "ConstSizeVectorBase::CopyFrom(...): thisPosition < 0");
		CHECKandTHROW((thisPosition + numberOfCopiedItems <= dataSize), "ConstSizeVectorBase::CopyFrom(...): thisPosition index mismatch");
		CHECKandTHROW((vectorPosition + numberOfCopiedItems <= vector.NumberOfItems()), "ConstSizeVectorBase::CopyFrom(...): vectorPosition index mismatch");

		for (Index i = 0; i < numberOfCopiedItems; i++)
		{
			(*this)[i + thisPosition] = vector[i + vectorPosition];
		}
	}

	//! copy from other vector (or even array) and perform type conversion (e.g. for graphics)
	template<class TVector>
	void CopyFrom(const TVector& vector)
	{
		SetNumberOfItems(vector.NumberOfItems());

		Index cnt = 0;
		for (auto val : vector) {
			data[cnt++] = (T)val;
		}
	}

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





















//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#else

template<typename T, Index dataSize>
class ConstSizeVectorBase: public VectorBase<T>
{
protected:
    T constData[dataSize];

public:
    //! default constructor, calls special constructor if vector for linking data, no allocation; no data item initialization (undefined values), rule of 5
    ConstSizeVectorBase()
    {
    	this->numberOfItems = dataSize;
		this->data = &constData[0];
    }


    //! initialize ConstSizeVectorBase with numberOfItemsInit <= dataSize; no data item initialization (undefined values)
    //! @todo: add private constructors Vector(T), Vector(T,T), Vector(int, int) in order to avoid programming errors; same for ConstSizeVectorBase
    ConstSizeVectorBase(Index numberOfItemsInit)
    {
        CHECKandTHROW(numberOfItemsInit <= dataSize, "ERROR: call to ConstSizeVectorBase(Index): dataSize mismatch");
		this->data = &constData[0];
		this->numberOfItems = numberOfItemsInit;
	}

    //! initialize ConstSizeVectorBase with numberOfItemsInit Reals; assign all data items with 'initializationValue'
    ConstSizeVectorBase(Index numberOfItemsInit, T initializationValue)
    {
        CHECKandTHROW(numberOfItemsInit <= dataSize, "ERROR: call to ConstSizeVectorBase(Index): dataSize mismatch");

		this->data = &constData[0];
		this->numberOfItems = numberOfItemsInit;

        //Index cnt = 0;
        for (auto &value : *this) {
            value = initializationValue;
        }
    }

    //! constructor with initializer list; condition: listOfReals.size() <= dataSize
    ConstSizeVectorBase(std::initializer_list<T> listOfReals) //pass by value as a standard in C++11
    {
        CHECKandTHROW((Index)listOfReals.size() <= dataSize, "ERROR: ConstSizeVectorBase::constructor, dataSize mismatch with initializer_list");
        //static_assert supported by C++14 (supports listOfReals.size() as constexpr) ==> needs /std:c++17 flag

		this->data = &constData[0];
		this->numberOfItems = (Index)listOfReals.size();

        Index cnt = 0;
        for (auto val : listOfReals) {
            constData[cnt++] = val;
        }
    }

private:
	ConstSizeVectorBase(std::initializer_list<Index>) = delete; //constructor forbidden, as it would convert wrongly for ConstSizeVector({2}) into ConstSizeVector(2)
public:

    //! copy constructor; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items], rule of 5
    ConstSizeVectorBase(const ConstSizeVectorBase<T,dataSize>& vector)
    {
		this->data = &constData[0];
		this->numberOfItems = vector.numberOfItems;

		Index cnt = 0;
        for (auto value : vector) {
            constData[cnt++] = value;
        }
    }

	//! move constructor, rule of 5 (but does not work yet)
	//ConstSizeVectorBase(ConstSizeVectorBase<T, dataSize>&& other) = default; 
	//ConstSizeVectorBase(ConstSizeVectorBase<T, dataSize>&& other) noexcept
	//{
	//	this->data = &constData[0];

	//	Index cnt = 0;
	//	for (auto val : other)
	//	{
	//		this->constData[cnt++] = std::exchange(val, (T)0.);
	//	}
	//	this->numberOfItems = std::exchange(other.numberOfItems, 0);
	//}

	//! @brief Initialize ConstSizeVectorBase by data given from vector at startPositionVector; 
    //! copies 'dataSize' items, independently of array size (might cause memory access error)
    ConstSizeVectorBase(const VectorBase<T>& vector, Index startPositionVector)
    {
        CHECKandTHROW(startPositionVector >= 0, "ERROR: ConstSizeVectorBase(const VectorBase<T>&, Index), startPositionVector < 0");
        CHECKandTHROW(dataSize + startPositionVector <= vector.NumberOfItems(), "ERROR: ConstSizeVectorBase(const VectorBase<T>&, Index), dataSize mismatch");

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
	virtual VectorType GetType() const { return VectorType::ConstVector; }

    //override iterator functions from VectorBase<T>, check performance!
    //T* begin() const override { return &this->data[0]; }				//!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
    //T* end() const override { return &this->data[this->numberOfItems]; }		//!< C++11 std::end() for iterators; iterator range is always the currently used numberOfItems.
    Index MaxNumberOfItems() const { return dataSize; }             //!< Get dataSize (available memory) of ConstSizeVectorBase

    //! call to ConstSizeVectorBase::SetNumberOfItems leads to run-time error if newNumberOfItems > dataSize; used in SetVector({...}) and in operator=; does not reset data
    virtual void SetNumberOfItems(Index newNumberOfItems) override
    {
        CHECKandTHROW(newNumberOfItems <= dataSize, "ERROR: call to ConstSizeVectorBase::SetNumberOfItems with newNumberOfItems > dataSize");
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

    //! @brief copy assignment operator; copies the currently used entries in range [0,numberOfItems]; rule of 5
    ConstSizeVectorBase<T,dataSize>& operator= (const ConstSizeVectorBase<T, dataSize>& vector)
    {
        if (this == &vector) { return *this; }

		//no check necessary, because only possible for equal datasizes!!!
		//CHECKandTHROW(vector.NumberOfItems() <= dataSize, "ERROR: call to ConstSizeVectorBase::operator= with vector.dataSize > dataSize");
		this->numberOfItems = vector.NumberOfItems();

        Index cnt = 0;
        for (auto item : vector) {
            constData[cnt++] = item;
        }
        return *this;
    }

	//fails to complile, compiler wants to use move assignment operator:
//private:
//	ConstSizeVectorBase<T, dataSize>& operator= (ConstSizeVectorBase<T, dataSize>&& other) = delete;
//public:
	//! move assignment operator, rule of 5 (does not work in such a way!)
	//ConstSizeVectorBase<T, dataSize>& operator= (ConstSizeVectorBase<T, dataSize>&& other) noexcept
	//{
	//	//no check necessary, because only possible for equal datasizes!!!
	//	//CHECKandTHROW((other.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator=: incompatible size of vectors: dataSize");

	//	for (Index i = 0; i < other.NumberOfItems(); i++) { //copy only items
	//		std::swap(constData[i], other.constData[i]);
	//	}

	//	std::swap(this->numberOfItems, other.numberOfItems);
	//	return *this;
	//}

    //! add two vectors, result = v1+v2 (for each component); only operated in range [0,numberOfItems]
    friend ConstSizeVectorBase<T, dataSize> operator+ (const ConstSizeVectorBase<T, dataSize>& v1, const ConstSizeVectorBase<T, dataSize>& v2)
    {
        CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator+: incompatible size of vectors");
        //not necessary, because impossible: CHECKandTHROW((v1.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator+: incompatible size of vectors: dataSize");
        
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
        CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "ConstSizeVectorBase::operator-: incompatible size of vectors");
		//not necessary, because impossible: CHECKandTHROW((v1.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator-: incompatible size of vectors: dataSize");
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
        CHECKandTHROW((v.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator* (scalar * v): incompatible size of vectors");
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
        CHECKandTHROW((v.NumberOfItems() <= dataSize), "ConstSizeVectorBase::operator* (v * scalar): incompatible size of vectors");
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
#endif //USE_NEW_CONSTSIZEVECTOR

template<Index dataSize>
using ConstSizeVector = ConstSizeVectorBase<Real, dataSize>;

typedef ConstSizeVector<1> CSVector1D;
typedef ConstSizeVector<2> CSVector2D;
typedef ConstSizeVector<3> CSVector3D;
typedef ConstSizeVector<4> CSVector4D;
typedef ConstSizeVector<6> CSVector6D; //geometrically exact beam2D
typedef ConstSizeVector<8> CSVector8D; //ANCF

template<Index dataSize>
using ConstSizeVectorF = ConstSizeVectorBase<float, dataSize>;


#endif
