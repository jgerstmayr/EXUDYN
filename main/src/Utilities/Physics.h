/** ***********************************************************************************************
* @brief		namespace ExuPhysics
*				helper functions and classes related to physics properties, etc.
*
* @author		Manzl Peter, Gerstmayr Johannes
* @date			2021-08-11 (generated)
* @date			2021-08-11 (last modified)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* 
*
************************************************************************************************ */
#ifndef UTILITIESPHYSICS__H
#define UTILITIESPHYSICS__H

namespace ExuPhysics {


	/*  StribeckFunction:   calculates the friction for a regularized friction model with a viscous part
	*   vel:                the velocity for which the friction is evaluated
	*   muDynamic:          dynamic friction coefficient
	*   muStaticOffset:     the friction for zero velocity without regularization
	*   muViscous:          viscous partr acting proportional to velocity above regVel
	*   expVel:             exponential decay with which the muStaticOffset decreases. At vel=expVel the factor to muStaticOffset is exp(-1) = 36.8%
	*   regVel:             small regularization velocity in which the friction is linear around zero velocity (to get newton converged)
	*   output:             returns velocity depending friction coefficient
	*   author: Manzl Peter, 08/2021
	*/
	inline Real StribeckFunction(Real vel, Real muDynamic, Real muStaticOffset, Real muViscous, Real expVel, Real regVel)
	{
		Real result = 0;
        if (fabs(vel) <= regVel && regVel != 0) //2023-01-22: extended for case regVel = 0
		{
			result = (muDynamic + muStaticOffset) * vel / regVel;
		}
		else
		{
			Real s = EXUstd::SignReal(vel);
			Real v = fabs(vel) - regVel;
			if (expVel != 0) {
				result = s * (muDynamic + muStaticOffset * exp(-v / expVel) + muViscous * v);
			}
			else
			{
				result = s * (muDynamic + muStaticOffset + muViscous * v); // if expVel == 0: exp(-v/expVel) should be 1 but div0 is triggered
			}
		}
		return result;
	}



} //namespace ExuPhysics
#endif
