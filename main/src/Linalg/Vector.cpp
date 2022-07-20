/** ***********************************************************************************************
* @brief		Implementation of Vector
*
* @author		Gerstmayr Johannes
* @date			1997-05-15 (generated)
* @date			2019-05-01 (last modified)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use Vector for large vector sizes; uses dynamic allocation (slow)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @todo         Vector::operators, check if data[cnt++] works and is faster
*
************************************************************************************************ */

#include "Linalg/Vector.h" 
#include "Linalg/SlimVector.h" 

#include "Utilities/Parallel.h" //ParallelFor, requires lots of includes, decoupled from .h files!


//put this into ResizableArray.cpp
#ifdef __EXUDYN_RUNTIME_CHECKS__
Index array_new_counts = 0; //global counter of item allocations; is increased every time a new is called
Index array_delete_counts = 0; //global counter of item deallocations; is increased every time a delete is called
#endif

#ifdef __EXUDYN_RUNTIME_CHECKS__
Index vector_new_counts = 0; //global counter of item allocations; is increased every time a new is called
Index vector_delete_counts = 0; //global counter of item deallocations; is increased every time a delete is called
Index linkedDataVectorCast_counts = 0; //global counter for unwanted type conversion from LinkedDataVector to Vector
#endif

//put this into Matrix.cpp
#ifdef __EXUDYN_RUNTIME_CHECKS__
Index matrix_new_counts = 0; //global counter of item allocations; is increased every time a new is called
Index matrix_delete_counts = 0; //global counter of item deallocations; is increased every time a delete is called
#endif

bool linalgPrintUsePythonFormat = true; //!< true: use python format for output of vectors and matrices; false: use matlab format

////! add two vectors, result = v1+v2 (for each component)
//template<typename T>
//VectorBase<T> operator+(const VectorBase<T>& v1, const VectorBase<T>& v2)
//{
//	CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "Vector::operator+: incompatible size of vectors");
//	VectorBase<T> result(v1.NumberOfItems());
//	Index cnt = 0;
//	for (auto &item : result) {
//		item = v1[cnt] + v2[cnt];
//		cnt++;
//	}
//	return result;
//}

////! add two vectors, result = v1-v2 (for each component)
//template<typename T>
//VectorBase<T> operator-(const VectorBase<T>& v1, const VectorBase<T>& v2)
//{
//	CHECKandTHROW((v1.NumberOfItems() == v2.NumberOfItems()), "Vector::operator-: incompatible size of vectors");
//	VectorBase<T> result(v1.NumberOfItems());
//	Index cnt = 0;
//	for (auto &item : result) {
//		item = v1[cnt] - v2[cnt];
//		cnt++;
//	}
//	return result;
//}

//the following code gives a linker error on MSVC17; cannot resolve this for double: why?
////! scalar multiply, result = scalar * v (for each component)
//template<typename T>
//VectorBase<T> operator*(const VectorBase<T>& v, T scalar)
//{
//	VectorBase<T> result(v.NumberOfItems());
//	Index cnt = 0;
//	for (auto &item : result) {
//		item = scalar * v[cnt++];
//	}
//	return result;
//}
//
////! scalar multiply, result = v * scalar (for each component)
//template<typename T>
//VectorBase<T> operator*(T scalar, const VectorBase<T>& v)
//{
//	VectorBase<T> result(v.NumberOfItems());
//	Index cnt = 0;
//	for (auto &item : result) {
//		item = scalar * v[cnt++];
//	}
//	return result;
//}
//

//! append a vector to Vector(*this) and return result=[*this,vector]; does not modify *this
template<typename T>
VectorBase<T> VectorBase<T>::Append(const VectorBase<T>& vector) const
{
	VectorBase<T> newVector(NumberOfItems() + vector.NumberOfItems());
	newVector.CopyFrom(*this, 0, 0, NumberOfItems());
	newVector.CopyFrom(vector, 0, NumberOfItems(), vector.NumberOfItems());
	return newVector;
}

template<>
VectorBase<Real> VectorBase<Real>::Append(const VectorBase<Real>& vector) const
{
	VectorBase<Real> newVector(NumberOfItems() + vector.NumberOfItems());
	newVector.CopyFrom(*this, 0, 0, NumberOfItems());
	newVector.CopyFrom(vector, 0, NumberOfItems(), vector.NumberOfItems());
	return newVector;
}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ResizableVectorParallelBase implementations

void ParallelPRealCopyFrom(Index nAVX, PReal* ptrData, PReal* ptrVector)
{
	exuThreading::ParallelFor((int)(nAVX), [&nAVX, &ptrData, &ptrVector](NGSsizeType i)
	{
		ptrData[i] = ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
	}); //"antasks": for numberOfItems=400.000, ideal factor=32*nThreads, numberOfItems=100.000, ideal factor=8*nThreads;
}

void ParallelPRealAdd(Index nAVX, PReal* ptrData, PReal* ptrVector)
{
	exuThreading::ParallelFor((int)(nAVX), [&nAVX, &ptrData, &ptrVector](NGSsizeType i)
	{
		ptrData[i] += ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
	});
}

void ParallelPRealSub(Index nAVX, PReal* ptrData, PReal* ptrVector)
{
	exuThreading::ParallelFor((int)(nAVX), [&nAVX, &ptrData, &ptrVector](NGSsizeType i)
	{
		ptrData[i] -= ptrVector[i]; //AVX operation, gives ~4 time speedup for AVX2 in chached operations
	});
}
void ParallelPRealMult(Index nAVX, PReal* ptrData, const PReal& scalarPD)
{
	exuThreading::ParallelFor((int)(nAVX), [&nAVX, &ptrData, &scalarPD](NGSsizeType i)
	{
		ptrData[i] *= scalarPD;
	});
}

void ParallelPRealDiv(Index nAVX, PReal* ptrData, const PReal& scalarPD)
{
	exuThreading::ParallelFor((int)(nAVX), [&nAVX, &ptrData, &scalarPD](NGSsizeType i)
	{
		ptrData[i] /= scalarPD;
	});
}
void ParallelPRealMultAdd(Index nAVX, PReal* ptrData, PReal* ptrVector, const PReal& scalarPD)
{
	exuThreading::ParallelFor((int)(nAVX), [&nAVX, &ptrData, &ptrVector, &scalarPD](NGSsizeType i)
	{
		ptrData[i] = _mm_fmadd_(scalarPD, ptrVector[i], ptrData[i]);
	});
}

Index ParallelGetNumThreads()
{
	return exuThreading::TaskManager::GetNumThreads();
}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//performance measurements on i9-14 core; threadingLowerLimit = 1; 
//1 thread: MicroThreading
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 2e-07, GFlops = 0, result = 0
//factor threads = 1, vector size = 801
//parallel vector operations needed = 0.497835, GFlops = 6.43587, error = 0.285848
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 0.498694, GFlops = 6.42078, error = 0.202126
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 0.76751, GFlops = 4.17063, error = 0.142925
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 0.737499, GFlops = 4.33967, error = 0.101062
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 0.83845, GFlops = 3.81686, error = 0.0714616
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.869056, GFlops = 3.6823, error = 0.050532
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.904095, GFlops = 3.53952, error = 0.0357317
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 1.64209, GFlops = 1.94876, error = 0.025263
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 1.93442, GFlops = 1.65425, error = 0.017862
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 1.9383, GFlops = 1.65083, error = 0.0126293
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 2.17327, GFlops = 1.47234, error = 0.00892973
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 2.75778, GFlops = 1.16028, error = 0.0063126

//4 threads: MicroThreading
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 4e-07, GFlops = 0, result = 0
//factor threads = 1, vector size = 801
//parallel vector operations needed = 3.60887, GFlops = 0.887813, error = 0.285848
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 1.88474, GFlops = 1.69891, error = 0.202126
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 1.01511, GFlops = 3.15334, error = 0.142925
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 0.66659, GFlops = 4.8013, error = 0.101062
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 0.552556, GFlops = 5.79172, error = 0.0714616
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.454911, GFlops = 7.03462, error = 0.050532
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.272304, GFlops = 11.7518, error = 0.0357317
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 0.386363, GFlops = 8.28245, error = 0.025263
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 0.252986, GFlops = 12.649, error = 0.017862
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 0.509174, GFlops = 6.2843, error = 0.0126293
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 0.624811, GFlops = 5.12123, error = 0.00892973
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 0.800735, GFlops = 3.99607, error = 0.0063126

//14 threads
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 3e-07, GFlops = 0, result = 0
//factor threads = 1, vector size = 801
//parallel vector operations needed = 5.12845, GFlops = 0.624751, error = 0.285848
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 2.83649, GFlops = 1.12886, error = 0.202126
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 1.48834, GFlops = 2.15072, error = 0.142925
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 0.742378, GFlops = 4.31115, error = 0.101062
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 0.42223, GFlops = 7.57939, error = 0.0714616
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.281045, GFlops = 11.3865, error = 0.050532
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.201685, GFlops = 15.8666, error = 0.0357317
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 0.180425, GFlops = 17.7361, error = 0.025263
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 0.1113, GFlops = 28.7512, error = 0.017862
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 0.105448, GFlops = 30.3449, error = 0.0126293
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 0.109076, GFlops = 29.3354, error = 0.00892973
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 0.333331, GFlops = 9.59946, error = 0.0063126

//4 threads ngstd:
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 6e-07, GFlops = 0, result = 0
//factor threads = 1, vector size = 801
//parallel vector operations needed = 4.49234, GFlops = 0.713214, error = 0.285848
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 2.95898, GFlops = 1.08213, error = 0.202126
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 1.81223, GFlops = 1.76633, error = 0.142925
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 1.05844, GFlops = 3.02378, error = 0.101062
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 0.788925, GFlops = 4.05647, error = 0.0714616
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.589233, GFlops = 5.431, error = 0.050532
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.462714, GFlops = 6.91585, error = 0.0357317
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 0.562914, GFlops = 5.68476, error = 0.025263
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 0.520092, GFlops = 6.15279, error = 0.017862
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 0.552402, GFlops = 5.79252, error = 0.0126293
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 0.604779, GFlops = 5.29086, error = 0.00892973
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 0.836587, GFlops = 3.82482, error = 0.0063126

//14 threads ngstd:
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 1e-07, GFlops = 0, result = 0
//factor threads = 1, vector size = 801
//parallel vector operations needed = 14.7635, GFlops = 0.217022, error = 0.285848
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 7.85411, GFlops = 0.407684, error = 0.202126
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 4.1323, GFlops = 0.774629, error = 0.142925
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 2.06342, GFlops = 1.55107, error = 0.101062
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 1.16957, GFlops = 2.73626, error = 0.0714616
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.579812, GFlops = 5.51925, error = 0.050532
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.353016, GFlops = 9.06492, error = 0.0357317
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 0.279753, GFlops = 11.4388, error = 0.025263
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 0.232142, GFlops = 13.7848, error = 0.017862
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 0.226625, GFlops = 14.1194, error = 0.0126293
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 0.236043, GFlops = 13.556, error = 0.00892973
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 0.323114, GFlops = 9.903, error = 0.0063126
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

