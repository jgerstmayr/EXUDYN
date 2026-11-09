/** ***********************************************************************************************
* @class		StepSizeController
* @brief		The step size controller of the adaptive solvers: the PI controller of Gustafsson (#3245)
* @details		Details:
*				- after an accepted step of the error e (the norm of the tolerances, accepted for e <= 1) and the
*				  previous one e_p, the next step h safety (1/e)^(0.7/k) e_p^(0.4/k), k the order of the local error;
*				  the first accepted step and a rejected step by the elementary rule h safety (1/e)^(1/k)
*				- after a rejection the step does not grow; at most maxIncrease times h and hMax, at least hMin, where
*				  a step is accepted with a warning (minimumReached)
*				- a step that the post-Newton step or the Newton iteration failed: ReducedStepSize, by the step size that
*				  was recommended or else by the factor of the settings; no growth afterwards either
*				- the Python version is _ErrorStepSize of exudyn.symbolics.advancedSolvers
*
* @author		Gerstmayr Johannes
* @date			2026-11-05
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef STEPSIZECONTROLLER__H
#define STEPSIZECONTROLLER__H

class StepSizeController
{
public:
	Real safety = 0.9;
	Real maxIncrease = 2.;
	Real hMin = 0.;
	Real hMax = 0.;
	Real errorPrevious = -1.;	//!< the error of the last accepted step; -1: none
	bool rejectedLast = false;	//!< the last step was rejected
	Index rejected = 0;			//!< the number of steps rejected by the error

	void Initialize(Real safetyInit, Real maxIncreaseInit, Real hMinInit, Real hMaxInit)
	{
		safety = safetyInit;
		maxIncrease = maxIncreaseInit;
		hMin = hMinInit;
		hMax = hMaxInit;
		errorPrevious = -1.;
		rejectedLast = false;
		rejected = 0;
	}

	//! the step size after a step h of the given error and order k of the local error: true and hNext if the step is
	//! accepted, false and the step to repeat it with in hNext if it is rejected; minimumReached if accepted at hMin
	bool Control(Real error, Real h, Real k, Real& hNext, bool& minimumReached)
	{
		minimumReached = false;
		error = EXUstd::Maximum(error, 1e-12);
		if (error > 1.)
		{
			if (h > hMin)
			{
				hNext = EXUstd::Maximum(hMin, safety * h * EM::Pow(1. / error, 1. / k));
				rejected++;
				rejectedLast = true;
				return false;
			}
			minimumReached = true;
		}
		Real factor = (errorPrevious < 0.) ? safety * EM::Pow(1. / error, 1. / k)
			: safety * EM::Pow(1. / error, 0.7 / k) * EM::Pow(errorPrevious, 0.4 / k);
		if (rejectedLast) { factor = EXUstd::Minimum(factor, 1.); }
		errorPrevious = error;
		rejectedLast = false;
		hNext = EXUstd::Minimum(EXUstd::Minimum(hMax, maxIncrease * h), EXUstd::Maximum(hMin, factor * h));
		return true;
	}

	//! the step size to repeat a step h that was reduced outside of the error control; recommended = -1: none was given;
	//! 0.75: a recommended step size enforces some reduction, because otherwise the iterations could get stuck
	Real ReducedStepSize(Real h, Real recommended, Real decrease)
	{
		rejectedLast = true;
		return (recommended != -1.) ? EXUstd::Minimum(0.75 * h, recommended) : decrease * h;
	}
};

#endif
