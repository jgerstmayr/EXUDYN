/** ***********************************************************************************************
* @brief		Implementation for computational loads;
*               This file covers implementation parts of loads, specifically python bindings
*
* @author		Gerstmayr Johannes
* @date			2019-12-01 (generated)
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

#include "Utilities/ExceptionsTemplates.h"
#include <pybind11/stl.h> 
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h> //accept numpy arrays: numpy array automatically converted to std::vector<Real,...> ==> accepts np.array([1,0,0]) and [1,0,0] as return value!
//#include "Main/CSystemData.h"
#include "Main/MainSystem.h" //for GetLoadValues
#include "Pymodules/PybindUtilities.h"

#include "Autogenerated/CLoadCoordinate.h"
#include "Autogenerated/CLoadForceVector.h"
#include "Autogenerated/CLoadTorqueVector.h"
#include "Autogenerated/CLoadMassProportional.h"

#include <pybind11/stl.h> 
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h> //accept numpy arrays: numpy array automatically converted to std::vector<Real,...> ==> accepts np.array([1,0,0]) and [1,0,0] as return value!


//#include "Autogenerated/MainLoadForceVector.h" //for CheckPreAssembleConsistency

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//here, all implementations to compute load values are collected

Real CLoadCoordinate::GetLoadValue(const MainSystemBase& mbs, Real t) const
{
	if (!parameters.loadUserFunction)
	{
		return parameters.load;
	}
	else
	{
		//user function args:(Real t, Real load)
		//return parameters.loadUserFunction(t, parameters.load);
		Real returnValue;
		UserFunctionExceptionHandling([&] //lambda function to add consistent try{..} catch(...) block
		{
			returnValue = parameters.loadUserFunction.userFunction((const MainSystem&)mbs, t, parameters.load);
		}, "LoadCoordinate::loadVectorUserFunction");
		return returnValue;
	}
}

Vector3D CLoadForceVector::GetLoadVector(const MainSystemBase& mbs, Real t) const
{
	if (!parameters.loadVectorUserFunction)
	{
		return parameters.loadVector;
	}
	else
	{
		//user function args:(Real t, Real load)
		//try {
		//	returnValue = (Vector3D)parameters.loadVectorUserFunction(t, parameters.loadVector);
		//}
		Vector3D returnValue;
		UserFunctionExceptionHandling([&] //lambda function to add consistent try{..} catch(...) block
		{
			returnValue = (Vector3D)parameters.loadVectorUserFunction.userFunction((const MainSystem&)mbs, t, parameters.loadVector);
		},"LoadForceVector::loadVectorUserFunction");

		return returnValue;
	}
}

Vector3D CLoadTorqueVector::GetLoadVector(const MainSystemBase& mbs, Real t) const
{
	if (!parameters.loadVectorUserFunction)
	{
		return parameters.loadVector;
	}
	else
	{
		//user function args:(Real t, Real load)
		//return (Vector3D)parameters.loadVectorUserFunction(t, parameters.loadVector);
		Vector3D returnValue;
		UserFunctionExceptionHandling([&] //lambda function to add consistent try{..} catch(...) block
		{
			returnValue = (Vector3D)parameters.loadVectorUserFunction.userFunction((const MainSystem&)mbs, t, parameters.loadVector);
		}, "LoadTorqueVector::loadVectorUserFunction");
		return returnValue;

	}
}

Vector3D CLoadMassProportional::GetLoadVector(const MainSystemBase& mbs, Real t) const
{
	if (!parameters.loadVectorUserFunction)
	{
		return parameters.loadVector;
	}
	else
	{
		//user function args:(Real t, Real load)
		//return (Vector3D)parameters.loadVectorUserFunction(t, parameters.loadVector);
		Vector3D returnValue;
		UserFunctionExceptionHandling([&] //lambda function to add consistent try{..} catch(...) block
		{
			returnValue = (Vector3D)parameters.loadVectorUserFunction.userFunction((const MainSystem&)mbs, t, parameters.loadVector);
		}, "LoadMassProportional::loadVectorUserFunction");
		return returnValue;
	}
}

//! Get current load value(s); copies values==>slow!; can be scalar or vector-valued!
py::object MainLoad::GetLoadValues(const MainSystemBase& mbs, Real time) const
{
	if (GetCLoad()->IsVector()) //vector
	{
		Vector3D value = GetCLoad()->GetLoadVector(mbs, time);
		return py::array_t<Real>(value.NumberOfItems(), value.GetDataPointer());
	}
	else //scalar
	{
		return py::float_(GetCLoad()->GetLoadValue(mbs, time));
	}
}
