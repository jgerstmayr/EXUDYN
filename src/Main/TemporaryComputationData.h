/** ***********************************************************************************************
* @class        TemporaryComputationData
* @brief		includes temporary storage data for efficient computation
* @details		Details: 
*
* @author		Gerstmayr Johannes
* @date			2021-11-01 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef TEMPORARYCOMPUTATIONDATA__H
#define TEMPORARYCOMPUTATIONDATA__H

#include "Main/MarkerData.h"

//! @brief class for temporary data during computation (time integration, static solver, etc.)
//! use multiple instances for parallelization
//! only contains resizable data structures for reuse in different objects
class TemporaryComputationData
{
public:
	//ResizableMatrix localMass;          //!< body mass matrix
	EXUmath::MatrixContainer massMatrix; //!< DENSE: body mass matrix, SPARSE: (linked) system mass matrix
	ResizableVector localODE2LHS;       //!< body ODE2LHS vector
	ResizableVector localODE1RHS;       //!< body ODE1LHS vector
	ResizableVector localAE;			//!< object (local) algebraic equations evaluation

	ResizableMatrix localJacobian;      //!< local (object)-jacobian during numerical/automatic differentiation
	ResizableMatrix localJacobian_t;    //!< local velocity (object)-jacobian during numerical/automatic differentiation

	JacobianTemp jacobianTemp;          //!< additional temporary data needed for jacobian computation
	//ResizableVector jacobianForce;      //!< for computation of jacobian derivative
	EXUmath::MatrixContainer jacobianODE2Container;  //!< DENSE: local object jacobian matrix, SPARSE: (linked) system jacobian

	ResizableVector generalizedLoad;    //!< generalized load vector added to ODE2 right-hand-side
	ResizableMatrix loadJacobian;       //!< Jacobian for application of load
	ResizableMatrix localJacobianAE_ODE1;//!< local constraint Jacobian (w.r.t. ODE2 part) during constraint jacobian computation
	ResizableMatrix localJacobianAE_ODE2;    //!< local constraint Jacobian (w.r.t. ODE2 part) during constraint jacobian computation
	ResizableMatrix localJacobianAE_ODE2_t;  //!< local constraint Jacobian (w.r.t. ODE2_t part) during constraint jacobian computation
	ResizableMatrix localJacobianAE_AE; //!< local constraint Jacobian w.r.t. algebraic variables; during constraint jacobian computation
	//ResizableMatrix localJacobianODE2;  //!< local (object) Jacobian during jacobian computation

	ResizableVector numericalJacobianf0;	//!< temporary vector for numerical differentiation
	ResizableVector numericalJacobianf1;	//!< temporary vector for numerical differentiation

    ArrayIndex tempIndex;					//!< for PostNewton ltg rebuild; e.g. for local to global coordinate mapping; JacobianODE2Loads
    ArrayIndex tempIndex2;					//!< for JacobianODE2Loads
	ArrayIndex tempIndex3;					//!< for JacobianODE2Loads
	ArrayIndex tempIndex4;					//!< for ComputeODE2SingleLoad, multi-object markers
	EXUmath::SparseVector sparseVector;		//!< used for temporary assembly of ode2RHS, contact, etc.

	//these sparsetriplets are either filled directly or as a buffer, if the regular section in the global sparse matrix is full
	SparseTripletVector sparseTriplets;		//!< used for temporary assembly of jacobian, mass matrix, etc.

	MarkerDataStructure markerDataStructure;
	Real tempValue; //!< used for PostNewton PNerror, maybe also for other procedures in future
	Real tempValue2; //!< used for PostNewton recommendedStepSize, maybe also for other procedures in future
};

//! array of temporary data used for parallelized (multithreaded) computations
class TemporaryComputationDataArray
{
private:
	ResizableArray<TemporaryComputationData*> data;

public:
	//! default constructor: 1 TemporaryComputationData for serial computation
	TemporaryComputationDataArray()
	{
		SetNumberOfItems(1); //this is the default value for serial computation
	}
	~TemporaryComputationDataArray()
	{
		EraseData();
	}

	TemporaryComputationData** begin() const { return data.begin(); }   //!< C++11 std::begin() for iterators
	TemporaryComputationData** end() const { return data.end(); }		//!< C++11 std::end() for iterators

	void EraseData()
	{
		for (Index i = 0; i < data.NumberOfItems(); i++)
		{
			delete data[i];
		}
		data.Flush();
	}

	//! obtain number of available data items
	Index NumberOfItems() const { return data.NumberOfItems(); }

	//! set new size, taking care of allocating/deallocating TemporaryComputationData
	void SetNumberOfItems(Index size)
	{
		if (size != data.NumberOfItems()) //only changed if new size requested! may be called often
		{
			CHECKandTHROW(size > 0, "TemporaryComputationDataArray::SetNumberOfItems: size must be always > 0");
			if (data.NumberOfItems()) { EraseData(); }
			data.SetNumberOfItems(size);
			for (Index i = 0; i < data.NumberOfItems(); i++)
			{
				data[i] = new TemporaryComputationData();
			}
		}
	}

	//! get write access to TemporaryComputationData with certain index
	TemporaryComputationData& operator[] (Index i)
	{
		CHECKandTHROW((i >= 0) && (i < data.NumberOfItems()), "TemporaryComputationDataArray::operator[]: index out of range");

		return *data[i];
	}

	//! get read access to TemporaryComputationData with certain index
	const TemporaryComputationData& operator[] (Index i) const
	{
		CHECKandTHROW((i >= 0) && (i < data.NumberOfItems()), "TemporaryComputationDataArray::operator[] const : index out of range");

		return *data[i];
	}

	////! call this at beginning of computation function; clears all sparseTriplets
	//void ClearSparseMatrices(Index nThreads)
	//{
	//	SetNumberOfItems(nThreads); //only affected if changed; will be moved to CSystem!

	//	for (auto item : data) 
	//	{ 
	//		item->sparseTriplets.SetNumberOfItems(0); 
	//	}

	//}
};


#endif
