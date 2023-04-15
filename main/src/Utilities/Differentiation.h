/** ***********************************************************************************************
* @file			Differentiation.h
* @brief		This file contains methods to differentiate e.g. vector functions using AutoDiff or numerical differentiation
* @details		Details:
* 				- interface to AutomaticDifferentiation.h as well as numerical methods to differentiate functions
*
* @author		Gerstmayr Johannes
* @date			2022-05-13 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
*
************************************************************************************************ */

#ifndef DIFFERENTIATION__H
#define DIFFERENTIATION__H

#include "Linalg/BasicLinalg.h"
#include <cmath> 
#include "Utilities/AutomaticDifferentiation.h"

namespace EXUmath
{
    //! numerical differentiation for CSystem Jacobians; including reference values
    //! NumDiffSettings must be a structure, containing the parameters: addReferenceCoordinatesToEpsilon, relativeEpsilon, minimumCoordinateSize;	
    //! columnOffset and rowOffset are for destination in jacobian
	//! ComputeF1() needs to compute f1 in local scope
    template <typename TFUNC, typename NumDiffSettings, typename TMatrix>
    void NumericalDifferentiation(Index iBegin, Index iEnd, NumDiffSettings numDiff,
        Real factor, Vector& x, const Vector& xRef,
        TMatrix& jacobianGM, const Vector& f0, Vector& f1,
        TFUNC ComputeF1, Index columnOffset = 0, Index rowOffset = 0)
    {
        Real xRefVal = 0;
        Real xStore, eps;// , epsInv;
        for (Index i = iBegin; i < iEnd; i++) //compute column i
        {
            if (numDiff.addReferenceCoordinatesToEpsilon) { xRefVal = xRef[i]; }
            eps = numDiff.relativeEpsilon * (EXUstd::Maximum(numDiff.minimumCoordinateSize, std::fabs(x[i] + xRefVal)));

            xStore = x[i];
            x[i] += eps;
			ComputeF1(); //compute f1
            x[i] = xStore;

            jacobianGM.AddColumnVectorDiff(i+ columnOffset, f1, f0, (1. / eps) * factor, rowOffset);
        }
    }

    //! numerical differentiation for CSystem Jacobians
    //! NumDiffSettings must be a structure, containing the parameters: addReferenceCoordinatesToEpsilon, relativeEpsilon, minimumCoordinateSize;	
    //! columnOffset and rowOffset are for destination in jacobian
	//! ComputeF1() needs to compute f1 in local scope
	template <typename TFUNC, typename NumDiffSettings, typename TMatrix>
    void NumericalDifferentiation(Index iBegin, Index iEnd, NumDiffSettings numDiff,
        Real factor, Vector& x, 
        TMatrix& jacobianGM, const Vector& f0, Vector& f1,
        TFUNC ComputeF1, Index columnOffset = 0, Index rowOffset = 0)
    {
        Real xStore, eps;// , epsInv;
        for (Index i = iBegin; i < iEnd; i++) //compute column i
        {
            eps = numDiff.relativeEpsilon * (EXUstd::Maximum(numDiff.minimumCoordinateSize, std::fabs(x[i])));

            xStore = x[i];
            x[i] += eps;
			ComputeF1(); //compute f1
            x[i] = xStore;

            jacobianGM.AddColumnVectorDiff(i + columnOffset, f1, f0, (1. / eps) * factor, rowOffset);
        }
    }

    
    //inline auto NumIntegrate = [](Real(*function)(Real), auto& points, auto& weights, Real a, Real b)
	//{
	//	Index cnt = 0; Real value = 0.;
	//	for (auto item : points) { Real x = 0.5*(b - a)*item + 0.5*(b + a); value += 0.5*(b - a)*weights[cnt++] * function(x); }
	//};

	//Example how to define a function locally:
	//auto function = [](Real x) {return x*x; }; 
	//void function = [](const VectorX& x, VectorF& f) {f[0] = x[0]; f[1] = x[0]*x[1]; }; 

	//inline void NumDiff(void(*f)(const CSVector6D&, CSVector9D&), Index sizeF, CSVector6D& x, CSVector9D& tempF0, CSVector9D& tempF1,
	//	ConstSizeMatrix<6*9>& jacobian, Real relEps = 1e-8, Real minCoord = 1e-2)

	//! templated numerical differentiation function to compute jacobian
	//! needs to provide temporary vectors tempF0 and tempF1
	//! x is the point of evaluation and sizeF provides the size of function f; x will not be changed but must be non-const
	//! function f takes first argument Vector x and second argument Vector f
	//! see EXAMPLE in NodePoint3DSlope23
	template<class TVectorX, class TVectorF, class TMatrixJac, Index sizeF>
	void NumDiffVectors(void(*f)(const TVectorX&, TVectorF&), TVectorX& x, TVectorF& tempF0, TVectorF& tempF1,
		TMatrixJac& jacobian, Real relEps = 1e-8, Real minCoord = 1e-2)
	{
		Real eps, epsInv; //coordinate(column)-wise differentiation parameter; depends on size of coordinate
		Real xStore; //store value of x; avoid roundoff error effects in numerical differentiation
		Index sizeX = x.NumberOfItems();

		jacobian.SetNumberOfRowsAndColumns(sizeF, sizeX); //not initialized as all values are written

		tempF0.SetNumberOfItems(sizeF);
		tempF1.SetNumberOfItems(sizeF);

		f(x, tempF0);

		//differentiation w.r.t. ODE2 coordinates
		for (Index i = 0; i < sizeX; i++)
		{
			eps = relEps * (EXUstd::Maximum(minCoord, ::fabs(x[i])));

			xStore = x[i];
			x[i] += eps;
			f(x, tempF1);
			x[i] = xStore;

			epsInv = 1. / eps;

			for (Index j = 0; j < sizeF; j++)
			{
				jacobian(j, i) = epsInv * (tempF1[j] - tempF0[j]);
			}
		}
	}

	//! same as NumDiffVectors, but just for const vectors
	template<class TVectorX, class TVectorF, class TMatrixJac, Index sizeF>
	void NumDiffConstSizeVectors(void(*f)(const TVectorX&, TVectorF&), TVectorX& x, TMatrixJac& jacobian, Real relEps = 1e-8, Real minCoord = 1e-2)
	{
		ConstSizeVector<sizeF> tempF0(sizeF);
		ConstSizeVector<sizeF> tempF1(sizeF);

		NumDiffVectors(f, x, tempF0, tempF1, jacobian, relEps, minCoord);
	}

	//! templated automatic differentiation function to compute jacobian
	//! x is the point of evaluation and sizeF provides the size of function VectorFunction
	//! function  takes first argument const Vector& x and second argument Vector& result
	//! sizeF is the size of the function, sizeX is the number of variables for which is differentiated; jacobian has dimension (sizeF, sizeX)
	//! optional arguments (fixed values) can be passed to VectorFunction via optArgs, which need to be added to AutoDiffVectors call at the end
	template<Index sizeF, Index sizeX, class... OptArgs>
	void AutoDiffVectors(void(*VectorFunction)(const ConstSizeVectorBase<EXUmath::AutoDiff<sizeX, Real>, sizeX> &, 
		ConstSizeVectorBase<EXUmath::AutoDiff<sizeX, Real>, sizeF>&, OptArgs...),
		const ConstSizeVector<sizeX>& x, ConstSizeMatrix<sizeX*sizeF>& jacobian, OptArgs... optArgs)
	{
		jacobian.SetNumberOfRowsAndColumns(sizeF, sizeX); //not initialized as all values are written

		ConstSizeVectorBase<AutoDiff<sizeX, Real>, sizeX> qDiff;
		for (Index i = 0; i < sizeX; i++)
		{
			qDiff[i] = x[i];
			qDiff[i].DValue((int)i) = 1; //mark that this is the corresponding derivative
		}
		ConstSizeVectorBase<AutoDiff<sizeX, Real>, sizeF> fDiff;

		VectorFunction(qDiff, fDiff, optArgs...);

		//now copy autodifferentiated result:
		for (Index i = 0; i < sizeF; i++)
		{
			for (Index j = 0; j < sizeX; j++)
			{
				jacobian(i, j) = fDiff[i].DValue((int)j);
			}
		}
	}


};

//inline void EXUmath::NumDiff(void(*f)(const Vector&, Vector&), Index sizeF, Vector& x, Vector& tempF0, Vector& tempF1, Matrix& jacobian, Real relEps = 1e-8, Real minCoord = 1e-2)



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
