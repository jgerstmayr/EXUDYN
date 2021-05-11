/** ***********************************************************************************************
* @file			PythonUserFunctions.h
* @class		PythonUserFunctions
* @details		Details:
* 				adds container for user functions, that includes mainSystem for solver
*
* @author		Gerstmayr Johannes
* @date			2021-05-07 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef PYTHONUSERFUNCTIONS__H
#define PYTHONUSERFUNCTIONS__H

class MainSystem;

//class PythonUserFunctions; //!< linked in MainSystem, because CSystem does not know about MainSystem
//! @python user functions to be called e.g., by solver ==> instantiated in CSystem, but MainSystem is not available there!
class PythonUserFunctions
{
public:
	MainSystem* mainSystem; //!< stored for call to preStepFunction
	std::function<bool(const MainSystem& mainSystem, Real t)> preStepFunction;//!< function called prior to the computation of a single step
	std::function<StdVector2D(const MainSystem& mainSystem, Real t)> postNewtonFunction;//!< function called after Newton method

	PythonUserFunctions()
	{
		Reset();
	}
	void Reset()
	{
		mainSystem = 0;
		preStepFunction = 0;
		postNewtonFunction = 0;
	}
};


#endif
