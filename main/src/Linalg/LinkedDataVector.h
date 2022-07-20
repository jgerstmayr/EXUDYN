/** ***********************************************************************************************
* @class		LinkedDataVectorBase
* @brief		A vector with data linked to other Vector, c-array(Real), SlimVector or similar (no memory allocation!)
* @details		Details:
                - link to c-array Real[x], Vector, LinkedDataVectorBase or SlimVector data
                - Size of LinkedDataVectorBase unchangeable (this would make no sense; instead just link to other data)
                - LinkedDataVectorBase should be used to avoid copying (portions of) vectors for efficient operation on subvectors
                - does not perform any memory allocation or delete; LinkedDataVectorBase assumes that memory management is done by data which it is linked to

* @author		Gerstmayr Johannes
* @date			2018-05-04 (created)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
 				- Use LinkedDataVectorBase for small and large vector sizes
                - Take care that the data to which LinkedDataVectorBase is linked is valid until destruction of LinkedDataVectorBase
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
* @code{.cpp}
*   Vector v1({1 2 3 4 5 6});	    //create a vector, allocate memory
*   LinkedDataVector v2(v1, 2, 6);  //link to data of v1, using items in range [2,6]
*   v2[1] += 10;                    //modify also v1[3]
*   Real x = v2.GetL2Norm();        //
*  	cout << v1 << "\n";		        //write "[1 2 3 14 5 6]" to cout
* @endcode
************************************************************************************************ */
#ifndef LINKEDDATAVECTORBASE__H
#define LINKEDDATAVECTORBASE__H

#include "Linalg/Vector.h"
#include "Linalg/SlimVector.h"

#include "Linalg/ConstSizeVector.h"

template<typename T>
class LinkedDataVectorBase : public VectorBase<T>
{

public:
    //! default constructor, does not link yet (data=nullptr means no linking)
    LinkedDataVectorBase() : VectorBase<T>() {}

	//! links data to VectorBase<T> (also resizable VectorBase<T>); no copying!
    //! @todo check a way to eliminate the const_cast for LinkedDataVectorBase(const VectorBase<T>& vector)
    //LinkedDataVectorBase(const VectorBase<T>& vector) : VectorBase<T>(vector.GetDataPointer(), vector.NumberOfItems())
    LinkedDataVectorBase(const VectorBase<T>& vector) : VectorBase<T>()
    {
        //const T* ptr = &(vector[0]);
        //const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
		this->data = vector.GetDataPointer();
		this->numberOfItems = vector.NumberOfItems();
    }

    //! links data to SlimVector<dataSize>; no copying!
    template<Index dataSize>
    LinkedDataVectorBase(const SlimVectorBase<T,dataSize>& vector) : VectorBase<T>()
    {
        //const T* ptr = &(vector[0]);
		const T* ptr = vector.GetDataPointer();
		this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
		this->numberOfItems = vector.NumberOfItems();
    }

	//! Initialize LinkedDataVectorBase by data given by vector at startPosition, using numberOfItemsLinked items (LinkedDataVectorBase has 'numberOfItemsLinked' virtual items); 
	LinkedDataVectorBase(const VectorBase<T>& vector, Index startPosition, Index numberOfItemsLinked) : VectorBase<T>()
	{
		CHECKandTHROW(startPosition >= 0, "ERROR: LinkedDataVectorBase(const VectorBase<T>&, Index), startPosition < 0");
		CHECKandTHROW(numberOfItemsLinked + startPosition <= vector.NumberOfItems(), "ERROR: LinkedDataVectorBase(const VectorBase<T>&, Index, Index), size mismatch");

		if (numberOfItemsLinked) //otherwise, data and numberOfItems are initialized as 0 / nullptr
		{
			const T* ptr = &vector[startPosition];
			this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
			this->numberOfItems = numberOfItemsLinked; //0 is also possible (appears, if e.g. no ODE2, AE or ODE1 coordinates)
		}
	}

	//! links data to SlimVector<dataSize>; data given by vector at startPosition, using numberOfItemsLinked items (LinkedDataVectorBase has 'numberOfItemsLinked' virtual items);
	template<Index dataSize>
	LinkedDataVectorBase(const SlimVectorBase<T, dataSize>& vector, Index startPosition, Index numberOfItemsLinked) : VectorBase<T>()
	{
		CHECKandTHROW(startPosition >= 0, "ERROR: LinkedDataVectorBase(const SlimVectorBase<T, dataSize>&, Index), startPosition < 0");
		CHECKandTHROW(numberOfItemsLinked + startPosition <= vector.NumberOfItems(), "ERROR: LinkedDataVectorBase(const SlimVectorBase<T, dataSize>&, Index, Index), size mismatch");

		const T* ptr = &vector[startPosition];
		this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
		this->numberOfItems = numberOfItemsLinked;
	}

	//! Initialize LinkedDataVectorBase by data given data pointer and size 
	LinkedDataVectorBase(const T* ptr, Index numberOfItemsLinked) : VectorBase<T>()
	{
		if (numberOfItemsLinked) //otherwise, data and numberOfItems are initialized as 0 / nullptr
		{
			this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
			this->numberOfItems = numberOfItemsLinked; //0 is also possible (appears, if e.g. no ODE2, AE or ODE1 coordinates)
		}
	}

	template<Index dataSize>
	LinkedDataVectorBase(const ConstSizeVectorBase<T, dataSize>& vector) : VectorBase<T>()
	{
		//const T* ptr = &(vector[0]);
		const T* ptr = vector.GetDataPointer();
		this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
		this->numberOfItems = vector.NumberOfItems();
	}
	//! links data to SlimVector<dataSize>; data given by vector at startPosition, using numberOfItemsLinked items (LinkedDataVectorBase has 'numberOfItemsLinked' virtual items);
	template<Index dataSize>
	LinkedDataVectorBase(const ConstSizeVectorBase<T, dataSize>& vector, Index startPosition, Index numberOfItemsLinked) : VectorBase<T>()
	{
		CHECKandTHROW(startPosition >= 0, "ERROR: LinkedDataVectorBase(const Tvector&, Index), startPosition < 0");
		CHECKandTHROW(numberOfItemsLinked + startPosition <= vector.NumberOfItems(), "ERROR: LinkedDataVectorBase(const Tvector&, Index, Index), size mismatch");
		
		const T* ptr = &vector.GetUnsafe(startPosition);
		this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
		this->numberOfItems = numberOfItemsLinked;
	}


//! override destructor / delete[] from VectorBase<T>; no memory deallocated
    virtual ~LinkedDataVectorBase()
    {
		this->data = nullptr; //because destructor ~VectorBase<T> & VectorBase<T>::FreeMemory() are called hereafter ==> this will cause ~VectorBase<T> not to delete anything!
		this->numberOfItems = 0;
    };

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // MEMBER FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	virtual VectorType GetType() const { return VectorType::LinkedDataVector; }

	//! set vector to values given by initializer list; used to modify data which the LinkedDataVectorBase is linked to
	void SetVector(std::initializer_list<T> listOfReals)
	{
		CHECKandTHROW(this->numberOfItems == (Index)listOfReals.size(), "ERROR: LinkedDataVectorBase::SetVector(...), initializer_list must have same size as LinkedDataVectorBase");

		Index cnt = 0;
		for (auto value : listOfReals) {
			this->data[cnt++] = value;
		}
	}

	//! set vector to values given by initializer list; used to modify data which the LinkedDataVectorBase is linked to
	template<Index dataSize>
	void SetVector(const SlimVectorBase<T, dataSize>& vector)
	{
		CHECKandTHROW(this->numberOfItems == dataSize, "ERROR: LinkedDataVectorBase::SetVector(SlimVectorBase<>...), SlimVectorBase must have same size as LinkedDataVectorBase");

		Index cnt = 0;
		for (auto value : vector) {
			this->data[cnt++] = value;
		}
	}

	//! link data to VectorBase<T> (also resizable VectorBase<T>); no copying!
    void LinkDataTo(const VectorBase<T>& vector)
    {
		this->data = vector.GetDataPointer();
		this->numberOfItems = vector.NumberOfItems();
    }

    //! Link this to data given by 'vector' starting at startPosition, using numberOfItemsLinked items (LinkedDataVectorBase has 'numberOfItemsLinked' virtual items); 
    void LinkDataTo(const VectorBase<T>& vector, Index startPosition, Index numberOfItemsLinked)
    {
        CHECKandTHROW(startPosition >= 0, "ERROR: LinkedDataVectorBase::LinkDataTo(const VectorBase<T>&, Index), startPosition < 0");
        CHECKandTHROW(numberOfItemsLinked + startPosition <= vector.NumberOfItems(), "ERROR: LinkedDataVectorBase::LinkDataTo(const VectorBase<T>&, Index, Index), size mismatch");

        const T* ptr = &vector[startPosition];
		this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
		this->numberOfItems = numberOfItemsLinked;
    }

    ////operators which return a new VectorBase<T> are inefficient ==> use operator+=, *=, -= and similar for LinkedData

    //WRONG!:
    ////! assignment operator; assign LinkedDataVectorBase to data of VectorBase<T> (linked only); no data copy, no memory allocation
    //LinkedDataVectorBase& operator= (const VectorBase<T>& vector)
    //{
    //    numberOfItems = vector.NumberOfItems(); //do not need to check size, because it is just linked
    //    data = vector.GetDataPointer();
    //    return *this;
    //}

    //! assignment operator, if LinkedDataVectorBase is assigned to another LinkedDataVectorBase (MUST NOT invoke VectorBase<T>::operator=)
    LinkedDataVectorBase& operator= (const LinkedDataVectorBase& vector)
    {
        if (this == &vector) { return *this; }

		//this is the case, if assigned to empty LinkedDataVector ==> will not invoke copy
		if (vector.GetDataPointer() == nullptr && vector.NumberOfItems() == 0)
		{
			this->data = nullptr;
			this->numberOfItems = 0;
		} 
		else
		{
			CHECKandTHROW(this->numberOfItems == vector.NumberOfItems(), "ERROR: LinkedDataVectorBase::operator=(const LinkedDataVectorBase&), size mismatch");
			//do not call SetNumberOfItems, because size already fits and SetNumberOfItems would allocate memory!

			Index cnt = 0;
			for (auto item : vector) {
				(*this)[cnt++] = item;
			}
		}
        return *this;
    }


    //! assignment operator, if LinkedDataVectorBase is assigned to VectorBase<T> (MUST NOT invoke VectorBase<T>::operator=)
    LinkedDataVectorBase& operator= (const VectorBase<T>& vector)
    {
        if (this == &vector) { return *this; }

        CHECKandTHROW(this->numberOfItems == vector.NumberOfItems(), "ERROR: LinkedDataVectorBase::operator=(const LinkedDataVectorBase&), size mismatch");

        //do not call SetNumberOfItems, because size already fits and SetNumberOfItems would allocate memory!

        Index cnt = 0;
        for (auto item : vector) {
            (*this)[cnt++] = item;
        }
        return *this;
    }

	//! SetNumberOfItems is only allowed to make size smaller or keep equal; this helps a lot when linking with ConstSizeVector, knowing maximum size
	virtual void SetNumberOfItems(Index numberOfItemsInit) override
	{
		CHECKandTHROW((numberOfItemsInit <= this->numberOfItems), "ERROR: call to LinkedDataVectorBase::SetNumberOfItems only allowed if new size smaller/equal original size");
		this->numberOfItems = numberOfItemsInit;
	}

	//! copy from other vector (or even array) and perform type conversion (e.g. for graphics)
	template<class TVector>
	void CopyFrom(const TVector& vector)
	{
		CHECKandTHROW(this->numberOfItems == vector.NumberOfItems(), "ERROR: LinkedDataVectorBase::CopyFrom(const TVector&), size mismatch");

		Index cnt = 0;
		for (auto item : vector) {
			(*this)[cnt++] = (T)item;
		}
	}

protected: //functions cannot be called from outside 
    //! call to LinkedDataVectorBase::AllocateMemory is not called, but for safety add assertion
    virtual void AllocateMemory(Index numberOfRealsInit) override
    {
        CHECKandTHROWstring("ERROR: call to LinkedDataVectorBase::AllocateMemory(...) forbidden");
    }

    //! LinkedDataVectorBase must not delete[] data; function called because of VectorBase<T> destructor 
    virtual void FreeMemory() override {}

    //! CopyFrom makes no sense in case of LinkedDataVectorBase; disabled
    void CopyFrom(const VectorBase<T>& vector, Index vectorPosition, Index thisPosition, Index numberOfCopiedItems) { CHECKandTHROWstring("LinkedDataVectorBase::CopyFrom"); }

    //! Append makes no sense in case of LinkedDataVectorBase; disabled
    VectorBase<T> Append(const VectorBase<T>& vector) const {CHECKandTHROWstring("LinkedDataVectorBase::Append"); return vector;}
};

typedef LinkedDataVectorBase<Real> LinkedDataVector;
typedef LinkedDataVectorBase<float> LinkedDataVectorF; //always float, used for graphics

#endif
