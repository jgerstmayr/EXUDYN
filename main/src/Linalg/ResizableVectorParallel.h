/** ***********************************************************************************************
* @class		ResizableVectorParallelBase
* @brief		A resizable vector using AVX and multithreading for performance speedup
*
* @author		Gerstmayr Johannes
* @date			2022-07-10
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef RESIZABLEVECTORPARALLELBASE__H
#define RESIZABLEVECTORPARALLELBASE__H

#include "Linalg/Vector.h"
#include "Linalg/ConstSizeVector.h"

#ifdef USE_RESIZABLE_VECTOR_PARALLEL 

typedef std::vector<Real> StdVector; //needed for user functions

#include "Linalg/Use_avx.h" //include before NGsolve includes!!

extern void ParallelPRealCopyFrom(Index nAVX, PReal* ptrData, PReal* ptrVector);
extern void ParallelPRealAdd(Index nAVX, PReal* ptrData, PReal* ptrVector);
extern void ParallelPRealSub(Index nAVX, PReal* ptrData, PReal* ptrVector);
extern void ParallelPRealMult(Index nAVX, PReal* ptrData, const PReal& scalarPD);
extern void ParallelPRealDiv(Index nAVX, PReal* ptrData, const PReal& scalarPD);
extern void ParallelPRealMultAdd(Index nAVX, PReal* ptrData, PReal* ptrVector, const PReal& scalarPD);
extern Index ParallelGetNumThreads(); //returns number of Threads without including NGS


template<typename T>
class ResizableVectorParallelBase: public ResizableVectorBase<T>
{
private:
	static Index constexpr multithreadingLimit = ResizableVectorParallelThreadingLimit; //lower limit below which multithreading acceleration will not be used
	//! add vector v to *this vector (for each component); both vectors must have same size
	//ResizableVectorBase& operator+=(const ResizableVectorBase& v)

public:
	//! default constructor
	ResizableVectorParallelBase() : ResizableVectorBase<T>() {}

	//! initialize ResizableVectorBase numberOfItemsInit
	ResizableVectorParallelBase(Index numberOfItemsInit) : ResizableVectorBase<T>(numberOfItemsInit) {}

	//! initialize ResizableVectorBase with numberOfItemsInit Reals; assign all data items with 'initializationValue'
	ResizableVectorParallelBase(Index numberOfItemsInit, T initializationValue) : ResizableVectorBase<T>(numberOfItemsInit, initializationValue) {}

	//! constructor with initializer list; memory allocation!
	ResizableVectorParallelBase(std::initializer_list<T> listOfReals) : ResizableVectorBase<T>(listOfReals) {}

    //! @todo ResizableVectorParallelBase:copy constructor should not be needed
    //! copy constructor; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items]
    ResizableVectorParallelBase(const ResizableVectorParallelBase& vector): ResizableVectorBase<T>(vector) {}

	//! constructor with copy from ResizableVectorBase<T>; compile-time error, if dataSize mismatch!; copies only in range [0,vector.numberOfItems items]
	ResizableVectorParallelBase(const ResizableVectorBase<T>&& vector) noexcept : ResizableVectorBase<T>(vector) {}


	//! constructor with std::vector
	ResizableVectorParallelBase(const std::vector<T> vector)
	{
		this->maxNumberOfItems = (Index)vector.size();
		this->AllocateMemory((Index)vector.size());

		std::copy(vector.begin(), vector.end(), this->begin());
	}



    //! delete[] of ResizableVectorBase<T> called which does the job
    virtual ~ResizableVectorParallelBase() {}

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//virtual VectorType GetType() const override { return VectorType::ResizableVector; }

	//available in Vector, not needed! 2022-05-04
	////! copy numberOfCopiedItems items of a vector at vectorPosition to ResizableVectorBase(*this) at thisPosition, 
	//void CopyFrom(const ResizableVectorBase<T>& vector, Index vectorPosition, Index thisPosition, Index numberOfCopiedItems)
	//{
	//	CHECKandTHROW((thisPosition + numberOfCopiedItems <= this->NumberOfItems()), "ResizableVectorParallelBase::CopyFrom(...): thisPosition index mismatch");
	//	CHECKandTHROW((vectorPosition + numberOfCopiedItems <= vector.NumberOfItems()), "ResizableVectorParallelBase::CopyFrom(...): vectorPosition index mismatch");

	//	for (Index i = 0; i < numberOfCopiedItems; i++)
	//	{
	//		(*this)[i + thisPosition] = vector[i + vectorPosition];
	//	}
	//}


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
		Index nItems = vector.NumberOfItems();
		this->SetNumberOfItems(nItems);

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

    //! @todo: ResizableVectorParallelBase: check if operator+,-,* need to be overloaded (compare ConstSizeVectorBase)

	//! overloaded operator=, because it needs to return a ResizableVectorParallelBase
	//ResizableVectorParallelBase& operator=(const ResizableVectorParallelBase& vector)
	template <class TVector>
	ResizableVectorParallelBase& operator=(const TVector& vector)
	{
		if (this == &vector) { return *this; }

		CopyFrom(vector);
		return *this;
	}

	template <class TVector>
	ResizableVectorParallelBase& operator+=(const TVector& vector)
	{
		Index nItems = this->NumberOfItems();
		CHECKandTHROW((nItems == vector.NumberOfItems()), "ResizableVectorParallelBase::operator+=: incompatible size of vectors");

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
	ResizableVectorParallelBase& operator-=(const TVector& vector)
	{
		Index nItems = this->NumberOfItems();
		CHECKandTHROW((nItems == vector.NumberOfItems()), "ResizableVectorParallelBase::operator-=: incompatible size of vectors");

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
	ResizableVectorParallelBase& operator*=(T scalar)
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

		//for (auto &item : *this) {
		//	item *= scalar;
		//}
		return *this;
	}

	//! scalar division of vector v through scalar (for each component)
	ResizableVectorParallelBase& operator/=(T scalar)
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
		//for (auto &item : *this) {
		//	item /= scalar;
		//}
		return *this;
	}


	//! add vector scalar * v to *this vector
	template<class TVector>
	void MultAdd(T scalar, const TVector& vector)
	{
		Index nItems = this->NumberOfItems();
		CHECKandTHROW((vector.NumberOfItems() == nItems), "ResizableVectorParallelBase::MultAdd: incompatible size of vectors");

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
			//exuThreading::ParallelFor((int)(nAVX), [&nAVX, &ptrData, &ptrVector, &scalar](NGSsizeType i)
			//{
			//	ptrData[i] += scalar * ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
			//}, nThreads); //for numberOfItems=400.000, ideal factor=32, numberOfItems=100.000, ideal factor=8;
		}

		//process remaining items:
		for (Index i = (nAVX << AVXRealShift); i < nItems; i++)
		{
			this->data[i] += scalar * vector[i];
		}

	}


};

#else
template<class TReal> 
using ResizableVectorParallelBase = ResizableVectorBase<TReal>;
#endif

typedef ResizableVectorParallelBase<Real> ResizableVectorParallel;
//typedef ResizableVectorParallelBase<float> ResizableVectorParallelF; //always float, used for graphics



//additional trials with additional unrolling
//template <class TVector>
//ResizableVectorParallelBase& operator+=(const TVector& vector)
//{
//	CHECKandTHROW((this->NumberOfItems() == vector.NumberOfItems()), "ResizableVectorParallelBase::operator+=: incompatible size of vectors");
//
//	const Index unrollAddShift = 0; //additional shift for loop unrolling
//	const Index unrollSize = 1 << unrollAddShift;
//	const Index myAVXRealShift = AVXRealShift + unrollAddShift;
//	const Index myAVXRealSize = 1 << myAVXRealShift;
//	Index nAVX = (this->numberOfItems >> myAVXRealShift) << unrollAddShift;
//
//	//std::cout << "nAVX=" << nAVX << "\n";
//	//force loop unrolling:
//	Index i = 0;
//	PReal* ptrData = (PReal*)(this->data);
//	PReal* ptrVector = (PReal*)(vector.GetDataPointer());
//	while (i < nAVX)
//	{
//		ptrData[i] += ptrVector[i]; //AVX operation
//		//ptrData[i + 1] += ptrVector[i + 1]; //AVX operation //additional unrolling does not give speedup!
//		i += unrollSize;
//	}
//	//std::cout << "i=" << i << "\n";
//	i <<= AVXRealShift;
//	//std::cout << "ishift=" << i << "\n";
//
//	//this type of grouping does not lead to speedup, even with O2 and avx flags:
//	//Index nAVX = (this->numberOfItems >> myAVXRealShift) << myAVXRealShift;
//	//while (i < nAVX)
//	//{
//	//	for (Index j = 0; j < myAVXRealSize; j++)
//	//	{
//	//		this->data[i + j] += v[i + j];
//	//	}
//	//	i += myAVXRealSize;
//	//}
//
//	//process remaining items:
//	while (i < this->numberOfItems)
//	{
//		this->data[i] += v[i];
//		i++;
//	}
//
//	return *this;
//}


#endif
