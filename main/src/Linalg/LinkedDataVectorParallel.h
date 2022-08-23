/** ***********************************************************************************************
* @class		LinkedDataVectorParallelBase
* @brief		A vector with data linked to other Vector, c-array(Real), SlimVector or similar (no memory allocation!)
* @details		Details:
                - link to c-array Real[x], Vector, LinkedDataVectorParallelBase or SlimVector data
                - Size of LinkedDataVectorParallelBase unchangeable (this would make no sense; instead just link to other data)
                - LinkedDataVectorParallelBase should be used to avoid copying (portions of) vectors for efficient operation on subvectors
                - does not perform any memory allocation or delete; LinkedDataVectorParallelBase assumes that memory management is done by data which it is linked to

* @author		Gerstmayr Johannes
* @date			2018-05-04 (created)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
 				- Use LinkedDataVectorParallelBase for small and large vector sizes
                - Take care that the data to which LinkedDataVectorParallelBase is linked is valid until destruction of LinkedDataVectorParallelBase
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
*   LinkedDataVectorParallel v2(v1, 2, 6);  //link to data of v1, using items in range [2,6]
*   v2[1] += 10;                    //modify also v1[3]
*   Real x = v2.GetL2Norm();        //
*  	cout << v1 << "\n";		        //write "[1 2 3 14 5 6]" to cout
* @endcode
************************************************************************************************ */
#ifndef LINKEDDATAVECTORPARALLELBASE__H
#define LINKEDDATAVECTORPARALLELBASE__H

#include "Linalg/Vector.h"
#include "Linalg/SlimVector.h"

#include "Linalg/ConstSizeVector.h"
#include "Linalg/LinkedDataVector.h"

#include "Linalg/Use_avx.h" //include before NGsolve includes!!

extern void ParallelPRealCopyFrom(Index nAVX, PReal* ptrData, PReal* ptrVector);
extern void ParallelPRealAdd(Index nAVX, PReal* ptrData, PReal* ptrVector);
extern void ParallelPRealSub(Index nAVX, PReal* ptrData, PReal* ptrVector);
extern void ParallelPRealMult(Index nAVX, PReal* ptrData, const PReal& scalarPD);
extern void ParallelPRealDiv(Index nAVX, PReal* ptrData, const PReal& scalarPD);
extern void ParallelPRealMultAdd(Index nAVX, PReal* ptrData, PReal* ptrVector, const PReal& scalarPD);
extern Index ParallelGetNumThreads(); //returns number of Threads without including NGS

template<typename T>
class LinkedDataVectorParallelBase : public LinkedDataVectorBase<T>
{
	static Index constexpr multithreadingLimit = ResizableVectorParallelThreadingLimit; //lower limit below which multithreading acceleration will not be used

public:
	using LinkedDataVectorBase<T>::LinkedDataVectorBase; //no need to copy base class constructors ...
 //   //! default constructor, does not link yet (data=nullptr means no linking)
 //   LinkedDataVectorParallelBase() : LinkedDataVectorBase<T>() {}

	////! links data to VectorBase<T> (also resizable VectorBase<T>); no copying!
 //   LinkedDataVectorParallelBase(const VectorBase<T>& vector) : VectorBase<T>()
 //   {
 //       //const T* ptr = &(vector[0]);
 //       //const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
	//	this->data = vector.GetDataPointer();
	//	this->numberOfItems = vector.NumberOfItems();
 //   }

 //   //! links data to SlimVector<dataSize>; no copying!
 //   template<Index dataSize>
 //   LinkedDataVectorParallelBase(const SlimVectorBase<T,dataSize>& vector) : LinkedDataVectorBase<T>()
 //   {
 //       //const T* ptr = &(vector[0]);
	//	const T* ptr = vector.GetDataPointer();
	//	this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
	//	this->numberOfItems = vector.NumberOfItems();
 //   }

	////! Initialize LinkedDataVectorParallelBase by data given by vector at startPosition, using numberOfItemsLinked items (LinkedDataVectorParallelBase has 'numberOfItemsLinked' virtual items); 
	//LinkedDataVectorParallelBase(const VectorBase<T>& vector, Index startPosition, Index numberOfItemsLinked) : VectorBase<T>()
	//{
	//	CHECKandTHROW(startPosition >= 0, "ERROR: LinkedDataVectorParallelBase(const VectorBase<T>&, Index), startPosition < 0");
	//	CHECKandTHROW(numberOfItemsLinked + startPosition <= vector.NumberOfItems(), "ERROR: LinkedDataVectorParallelBase(const VectorBase<T>&, Index, Index), size mismatch");

	//	if (numberOfItemsLinked) //otherwise, data and numberOfItems are initialized as 0 / nullptr
	//	{
	//		const T* ptr = &vector[startPosition];
	//		this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
	//		this->numberOfItems = numberOfItemsLinked; //0 is also possible (appears, if e.g. no ODE2, AE or ODE1 coordinates)
	//	}
	//}

	////! links data to SlimVector<dataSize>; data given by vector at startPosition, using numberOfItemsLinked items (LinkedDataVectorParallelBase has 'numberOfItemsLinked' virtual items);
	//template<Index dataSize>
	//LinkedDataVectorParallelBase(const SlimVectorBase<T, dataSize>& vector, Index startPosition, Index numberOfItemsLinked) : VectorBase<T>()
	//{
	//	CHECKandTHROW(startPosition >= 0, "ERROR: LinkedDataVectorParallelBase(const SlimVectorBase<T, dataSize>&, Index), startPosition < 0");
	//	CHECKandTHROW(numberOfItemsLinked + startPosition <= vector.NumberOfItems(), "ERROR: LinkedDataVectorParallelBase(const SlimVectorBase<T, dataSize>&, Index, Index), size mismatch");

	//	const T* ptr = &vector[startPosition];
	//	this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
	//	this->numberOfItems = numberOfItemsLinked;
	//}

	////! Initialize LinkedDataVectorParallelBase by data given data pointer and size 
	//LinkedDataVectorParallelBase(const T* ptr, Index numberOfItemsLinked) : VectorBase<T>()
	//{
	//	if (numberOfItemsLinked) //otherwise, data and numberOfItems are initialized as 0 / nullptr
	//	{
	//		this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
	//		this->numberOfItems = numberOfItemsLinked; //0 is also possible (appears, if e.g. no ODE2, AE or ODE1 coordinates)
	//	}
	//}

	//template<Index dataSize>
	//LinkedDataVectorParallelBase(const ConstSizeVectorBase<T, dataSize>& vector) : VectorBase<T>()
	//{
	//	//const T* ptr = &(vector[0]);
	//	const T* ptr = vector.GetDataPointer();
	//	this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
	//	this->numberOfItems = vector.NumberOfItems();
	//}
	////! links data to SlimVector<dataSize>; data given by vector at startPosition, using numberOfItemsLinked items (LinkedDataVectorParallelBase has 'numberOfItemsLinked' virtual items);
	//template<Index dataSize>
	//LinkedDataVectorParallelBase(const ConstSizeVectorBase<T, dataSize>& vector, Index startPosition, Index numberOfItemsLinked) : VectorBase<T>()
	//{
	//	CHECKandTHROW(startPosition >= 0, "ERROR: LinkedDataVectorParallelBase(const Tvector&, Index), startPosition < 0");
	//	CHECKandTHROW(numberOfItemsLinked + startPosition <= vector.NumberOfItems(), "ERROR: LinkedDataVectorParallelBase(const Tvector&, Index, Index), size mismatch");
	//	
	//	const T* ptr = &vector.GetUnsafe(startPosition);
	//	this->data = const_cast<T*>(ptr); //needed, if vector passed as const ... workaround
	//	this->numberOfItems = numberOfItemsLinked;
	//}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//                      PARALLEL
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! copy from other vector (or even array) and perform type conversion (e.g. for graphics)
	template<class TVector>
	void CopyFrom(const TVector& vector)
	{
		CHECKandTHROW(this->numberOfItems == vector.NumberOfItems(), "ERROR: LinkedDataVectorParallelBase::CopyFrom(const TVector&), size mismatch");
		Index nItems = vector.NumberOfItems();

		Index nAVX = nItems >> AVXRealShift;
		if (nItems < multithreadingLimit || ParallelGetNumThreads() == 1)
		{

			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());
			for (Index i = 0; i < nAVX; i++)
			{
				ptrData[i] = ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
			}
		}
		else
		{
			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());

			ParallelPRealCopyFrom(nAVX, ptrData, ptrVector);
		}
		//process remaining items:
		for (Index i = (nAVX << AVXRealShift); i < nItems; i++)
		{
			this->data[i] = vector[i];
		}
	}

	//! assignment operator, if LinkedDataVectorBase is assigned to another LinkedDataVectorBase (MUST NOT invoke VectorBase<T>::operator=)
	LinkedDataVectorParallelBase& operator= (const LinkedDataVectorBase<T>& vector)
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
			CHECKandTHROW(this->numberOfItems == vector.NumberOfItems(), "ERROR: LinkedDataVectorParallelBase::operator=(const LinkedDataVectorBase&), size mismatch");
			//do not call SetNumberOfItems, because size already fits and SetNumberOfItems would allocate memory!

			CopyFrom(vector);
		}
		return *this;
	}


	//! assignment operator, if LinkedDataVectorBase is assigned to VectorBase<T> (MUST NOT invoke VectorBase<T>::operator=)
	LinkedDataVectorParallelBase& operator= (const VectorBase<T>& vector)
	{
		if (this == &vector) { return *this; }

		CHECKandTHROW(this->numberOfItems == vector.NumberOfItems(), "ERROR: LinkedDataVectorParallelBase::operator=(const LinkedDataVectorBase&), size mismatch");
		//do not call SetNumberOfItems, because size already fits and SetNumberOfItems would allocate memory!

		CopyFrom(vector);
		return *this;
	}


	template <class TVector>
	LinkedDataVectorParallelBase& operator+=(const TVector& vector)
	{
		Index nItems = this->NumberOfItems();
		CHECKandTHROW((nItems == vector.NumberOfItems()), "LinkedDataVectorParallelBase::operator+=: incompatible size of vectors");

		Index nAVX = nItems >> AVXRealShift;
		if (nItems < multithreadingLimit || ParallelGetNumThreads() == 1)
		{
			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());
			for (Index i = 0; i < nAVX; i++)
			{
				ptrData[i] += ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
			}
		}
		else
		{
			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());

			ParallelPRealAdd(nAVX, ptrData, ptrVector);
		}
		//process remaining items:
		for (Index i = (nAVX << AVXRealShift); i < nItems; i++)
		{
			this->data[i] += vector[i];
		}

		return *this;
	}

	template <class TVector>
	LinkedDataVectorParallelBase& operator-=(const TVector& vector)
	{
		Index nItems = this->NumberOfItems();
		CHECKandTHROW((nItems == vector.NumberOfItems()), "LinkedDataVectorParallelBase::operator-=: incompatible size of vectors");

		Index nAVX = nItems >> AVXRealShift;
		if (nItems < multithreadingLimit || ParallelGetNumThreads() == 1)
		{
			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());
			for (Index i = 0; i < nAVX; i++)
			{
				ptrData[i] -= ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
			}
		}
		else
		{
			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());

			ParallelPRealSub(nAVX, ptrData, ptrVector);
		}
		//process remaining items:
		for (Index i = (nAVX << AVXRealShift); i < nItems; i++)
		{
			this->data[i] -= vector[i];
		}

		return *this;
	}


	//! scalar multiply vector *this with scalar (for each component)
	LinkedDataVectorParallelBase& operator*=(T scalar)
	{
		Index nItems = this->NumberOfItems();

		Index nAVX = nItems >> AVXRealShift;
		PReal scalarPD = _mm_set1_(scalar);
		if (nItems < multithreadingLimit || ParallelGetNumThreads() == 1)
		{
			PReal* ptrData = (PReal*)(this->data);
			for (Index i = 0; i < nAVX; i++)
			{
				ptrData[i] *= scalarPD; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
			}
		}
		else
		{
			PReal* ptrData = (PReal*)(this->data);
			ParallelPRealMult(nAVX, ptrData, scalarPD);

		}
		//process remaining items:
		for (Index i = (nAVX << AVXRealShift); i < nItems; i++)
		{
			this->data[i] *= scalar;
		}

		return *this;
	}

	//! scalar division of vector v through scalar (for each component)
	LinkedDataVectorParallelBase& operator/=(T scalar)
	{
		Index nItems = this->NumberOfItems();

		Index nAVX = nItems >> AVXRealShift;
		PReal scalarPD = _mm_set1_(scalar);
		if (nItems < multithreadingLimit || ParallelGetNumThreads() == 1)
		{
			PReal* ptrData = (PReal*)(this->data);
			for (Index i = 0; i < nAVX; i++)
			{
				ptrData[i] /= scalarPD; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
			}
		}
		else
		{
			PReal* ptrData = (PReal*)(this->data);

			ParallelPRealDiv(nAVX, ptrData, scalarPD);
		}
		//process remaining items:
		for (Index i = (nAVX << AVXRealShift); i < nItems; i++)
		{
			this->data[i] /= scalar;
		}
		return *this;
	}


	//! add vector scalar * v to *this vector
	template<class TVector>
	void MultAdd(T scalar, const TVector& vector)
	{
		Index nItems = this->NumberOfItems();
		CHECKandTHROW((vector.NumberOfItems() == nItems), "LinkedDataVectorParallelBase::MultAdd: incompatible size of vectors");

		PReal scalarPD = _mm_set1_(scalar);
		Index nAVX = nItems >> AVXRealShift;

		if (nItems < multithreadingLimit || ParallelGetNumThreads() == 1)
		{
			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());
			for (Index i = 0; i < nAVX; i++)
			{
				ptrData[i] = _mm_fmadd_(scalarPD, ptrVector[i], ptrData[i]);
				//ptrData[i] += scalar * ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
			}
		}
		else
		{
			PReal* ptrData = (PReal*)(this->data);
			PReal* ptrVector = (PReal*)(vector.GetDataPointer());

			ParallelPRealMultAdd(nAVX, ptrData, ptrVector, scalarPD);
		}

		//process remaining items:
		for (Index i = (nAVX << AVXRealShift); i < nItems; i++)
		{
			this->data[i] += scalar * vector[i];
		}

	}

};

typedef LinkedDataVectorParallelBase<Real> LinkedDataVectorParallel;
//typedef LinkedDataVectorParallelBase<float> LinkedDataVectorParallelF; //always float, used for graphics

#endif
