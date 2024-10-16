/** ***********************************************************************************************
* @brief		Implentation for computational solver CSolver structure classes
* @details		Details:
* 				- base classes for computational solver (as compared to main solver, which interacts with Python, etc.)
*
* @author		Gerstmayr Johannes
* @date			2019-12-11 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

#include <pybind11/pybind11.h> //for integrated python connectivity (==>put functionality into separate file ...!!!)
#include <pybind11/eval.h>

#include "System/versionCpp.h"
#include "Linalg/BasicLinalg.h" //for Resizable Vector
#include "Main/CSystem.h"
#include "Autogenerated/CMarkerBodyPosition.h"
#include "Solver/CSolverBase.h" 
#include <fstream>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//CSolverTimer
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! compute sum of all timer parts (should be nearly the total time)
Real CSolverTimer::Sum() const
{
	return factorization + newtonIncrement + integrationFormula + ODE2RHS + ODE1RHS + AERHS + totalJacobian + massMatrix + reactionForces +
		postNewton + errorEstimator + writeSolution + overhead + python + visualization;
}

STDstring CSolverTimer::ToString() const
{
	if (!useTimer) { return ""; }

	std::ostringstream ostr;
	ostr.precision(3); //reduced precision for nicer output...
	ostr << "====================\nCPU-time statistics:\n";
	if (Sum() != 0.) // avoid division by zero ...
	{
		Real sum = Sum() / 100.;
		Real limit = 0.;
		ostr << "  total time   = " << total << " seconds\n";
		ostr << "  measured time= " << Sum() << " seconds (=" << 100.*Sum() / total << "%) \n";
		ostr << "  non-zero timer [__ sub-timer]:\n";
		if (factorization / sum > limit) ostr << "  factorization     = " << factorization / sum << "%\n";
		if (newtonIncrement / sum > limit) ostr << "  newtonIncrement   = " << newtonIncrement / sum << "%\n";
		if (integrationFormula / sum > limit) ostr << "  integrationFormula= " << integrationFormula / sum << "%\n";
		if (ODE2RHS / sum > limit) ostr << "  ODE2RHS           = " << ODE2RHS / sum << "%\n";
		if (ODE1RHS / sum > limit) ostr << "  ODE1RHS           = " << ODE1RHS / sum << "%\n";
		if (AERHS / sum > limit) ostr << "  AERHS             = " << AERHS / sum << "%\n";
		if (totalJacobian / sum > limit) ostr << "  totalJacobian     = " << totalJacobian / sum << "%\n";
		if (jacobianODE2_t / sum > limit) ostr << "  __jacobianODE2_t  = " << jacobianODE2_t / sum << "%\n";
		if (jacobianODE2 / sum > limit) ostr << "  __jacobianODE2    = " << jacobianODE2 / sum << "%\n";
		if (jacobianODE1 / sum > limit) ostr << "  __jacobianODE1    = " << jacobianODE1 / sum << "%\n";
		if (jacobianAE / sum > limit) ostr << "  __jacobianAE      = " << jacobianAE / sum << "%\n";
		if (massMatrix / sum > limit) ostr << "  massMatrix        = " << massMatrix / sum << "%\n";
		if (reactionForces / sum > limit) ostr << "  reactionForces    = " << reactionForces / sum << "%\n";
		//not computed, implemented as special timer: if (postNewton / sum > limit) ostr << "  postNewtonStep    = " << postNewton / sum << "%\n";
		if (errorEstimator / sum > limit) ostr << "  errorEstimator    = " << errorEstimator / sum << "%\n";
		if (postNewton / sum > limit) ostr << "  postNewton    = " << postNewton / sum << "%\n";
		if (python / sum > limit) ostr << "  Python          = " << python / sum << "%\n";
		if (writeSolution / sum > limit) ostr << "  writeSolution     = " << writeSolution / sum << "%\n";
		if (overhead / sum > limit) ostr << "  overhead          = " << overhead / sum << "%\n";
		if (visualization / sum > limit) ostr << "  visualization/user= " << visualization / sum << "%\n";
	}
	else
	{
		ostr << "  total CPU time is zero or not measured; no statistics available!\n\n";
	}
	return ostr.str();
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//SolverLocalData
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! if desired, temporary data is cleaned up to safe memory
void SolverLocalData::CleanUpMemory()
{
	nODE2 = 0;
	nODE1 = 0;
	nAE = 0;
	nData = 0;
	nSys = 0;
	startAE = 0;

	//there is no other way to free memory than to assign to a new matrix
	systemJacobianDense = GeneralMatrixEXUdense();
	systemMassMatrixDense = GeneralMatrixEXUdense();
	jacobianAEdense = GeneralMatrixEXUdense();

	systemJacobianSparse.Reset();
	systemMassMatrixSparse.Reset();
	jacobianAEsparse.Reset();

	//+++++++++++++++++++++++++++
	systemResidual.Reset();
	newtonSolution.Reset();
	tempODE2.Reset();
	temp2ODE2.Reset();
	tempODE2F0.Reset();
	tempODE2F1.Reset();

	//startOfDiscIteration.Reset();
	startOfStepStateAAlgorithmic.Reset();

	aAlgorithmic.Reset();
}

//! function links system matrices to according dense/sparse versions
void SolverLocalData::SetLinearSolverType(LinearSolverType linearSolverType, bool reuseAnalyzedPattern, 
    bool ignoreSingularJacobian, Real pivotThreshold)
{
	//std::cout << "SetLinearSolverType" << std::flush;
	//pout << "linearSolverType=" << linearSolverType << "\n";
	if (EXUstd::IsOfType(LinearSolverType::Dense, linearSolverType))
	{
		//std::cout << "SetLinearSolverType1a" << std::flush;
		systemJacobian = &systemJacobianDense;
		systemMassMatrix = &systemMassMatrixDense;
		jacobianAE = &jacobianAEdense;
#ifdef USE_EIGEN_DENSE_SOLVER

        Index flagEigen = (Index)(linearSolverType == LinearSolverType::EigenDense);
        flagEigen += flagEigen * (Index)ignoreSingularJacobian; //flag only added in EigenDense case

        systemJacobianDense.UseEigenSolverType() = flagEigen;
        systemMassMatrixDense.UseEigenSolverType() = flagEigen;
        jacobianAEdense.UseEigenSolverType() = flagEigen;
#endif
    }
	else
	{
		//std::cout << "SetLinearSolverType1b" << std::flush;
		systemJacobian = &systemJacobianSparse;
		systemMassMatrix = &systemMassMatrixSparse;
		jacobianAE = &jacobianAEsparse;
	}
	
	//std::cout << "SetLinearSolverType2" << std::flush;
	if (linearSolverType == LinearSolverType::EigenSparseSymmetric)
	{
		systemJacobianSparse.AssumeSymmetric();
		systemMassMatrixSparse.AssumeSymmetric();
		jacobianAEsparse.AssumeSymmetric();
	}
	//{
	//	CHECKandTHROWstring("SolverLocalData::SetLinearSolverType: invalid linearSolverType");
	//}

	if (!EXUstd::IsOfType(LinearSolverType::Dense, linearSolverType))
	{
		//std::cout << "SetLinearSolverType3" << std::flush;
		systemJacobianSparse.SetReuseAnalyzedPattern(reuseAnalyzedPattern);
		systemMassMatrixSparse.SetReuseAnalyzedPattern(reuseAnalyzedPattern);
		jacobianAEsparse.SetReuseAnalyzedPattern(reuseAnalyzedPattern);
	}
	//std::cout << "SetLinearSolverType4" << std::flush;
	systemJacobian->PivotThreshold() = pivotThreshold;
    systemMassMatrix->PivotThreshold() = pivotThreshold;
    jacobianAE->PivotThreshold() = pivotThreshold;

	//std::cout << "SetLinearSolverType5" << std::flush;
}


//! convert iteration statistics to string
std::string SolverIterationData::ToString() const
{
	std::ostringstream ostr;
	ostr << "Solver iteration statistics:\n";
	ostr << "total number of steps:        " << currentStepIndex - 1 << "\n"; //last step not counted (first step=initialization)
	ostr << "total number of Newton iterations: " << newtonStepsCount << "\n";
	ostr << "total number of Newton Jacobians:  " << newtonJacobiCount << "\n";
	if (rejectedModifiedNewtonSteps)
	{
		ostr << "rejected modified Newton steps:      " << rejectedModifiedNewtonSteps << "\n";
	}

	return ostr.str();
}

