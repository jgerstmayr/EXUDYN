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

//+++++++++++++++++++++++++++++++++++++++++++++
//capsule for Python function + conversion to std::function
//class PyFunction; //!< pybind11::function

//class PyUFobject; //!< pybind11::object, to store either function or information (e.g., string to describe C++ function)
//typedef Real PyUFobject; //!< pybind11::object, to store either function or information (e.g., string to describe C++ function)
namespace pybind11 {
	class object;
};


enum class UserFunctionType {
	_None = 0,		//! marks that no user function exists
	Python = 1,		//! Python user functions with corresponding std::function
	Symbolic = 2,	//! Python user function with corresponding symbolic function (as std::function)
	InternalCpp = 3,//! Internal user function
	Jit = 4,		//! Jitted user function
};

//! convert SensorType to a string (used for output, type comparison, ...)
inline const char* GetUserFunctionTypeString(UserFunctionType var)
{
	switch (var)
	{
	case UserFunctionType::_None: return "_None";
	case UserFunctionType::Python: return "Python";
	case UserFunctionType::Symbolic: return "Symbolic";
	case UserFunctionType::InternalCpp: return "InternalCpp";
	case UserFunctionType::Jit: return "Jit";
	default: SysError("GetUserFunctionTypeString: invalid variable type");  return "Invalid";
	}
}

//! class to encapsulate user functions, according to UserFunctionType
template <typename UFT>
class PythonUserFunctionBase
{
private:
	pybind11::object* pyObject;
	UserFunctionType ufType;
public:
	UFT userFunction;

	//! constructor to set empty function
	PythonUserFunctionBase();
	~PythonUserFunctionBase();

	//! set with int, py::function, or dict containing additional info (e.g. internal C++ function or Jit-info)
	void SetPythonObject(const pybind11::object& pyObjectInit);

	void SetPythonUserFunction(const pybind11::object& pyObjectInit);

	//! always returns either 0 or Dictionary with meta-info (Python function + ufType as string)
	pybind11::object GetPythonDictionary() const;



	//! flag: true=stored valid user function (UF!=0); false=no user function (UF==0)
	void SetUFtype(UserFunctionType typeInit) { ufType = typeInit; }
	UserFunctionType GetUFtype() { return ufType; }
	//! return true, if user functions exists
	bool IsValid() const { return ufType != UserFunctionType::_None; }
	void Reset();

	//! comparison operator, just for comparison with 0
	bool operator==(Index value) const
	{
		CHECKandTHROW(value == 0, "PythonUserFunctionBase::operator==(): only allowed for comparison with 0");
		return !IsValid();
	}
	//! comparison operator, just for comparison with 0
	bool operator!=(Index value) const
	{
		CHECKandTHROW(value == 0, "PythonUserFunctionBase::operator!=(): only allowed for comparison with 0");
		return IsValid();
	}

	explicit operator bool() const {
		return IsValid();
	}

	//! cast to Python object: convet to dict
	explicit operator pybind11::object() const;
	//bool operator!() const { return !IsValid(); }

	//! assignment operator, just for assign to 0, indicating reset of user function
	PythonUserFunctionBase<UFT>& operator= (Index value)
	{
		CHECKandTHROW(value == 0, "PythonUserFunctionBase::operator=(): only allowed for assignment to 0");
		Reset();

		return *this;
	}

	void SetSymbolicUserFunction(const UFT& uft)
	{
		Reset(); //pyObject
		userFunction = uft;
		ufType = UserFunctionType::Symbolic;
	}


	//Needed?
	////! copy assignment operator
	//PythonUserFunctionBase<UFT>& operator= (const PythonUserFunctionBase<UFT>& other);

	//! assignment operator for regular assignment with pybind object
	PythonUserFunctionBase<UFT>& operator=(const pybind11::object& pyObjectInit);

	// not needed:
	////! assignment operator, just for assign to UFT, indicating reset of user function
	//PythonUserFunctionBase<UFT>& operator= (const UFT& uft)
	//{
	//	Reset(); //because only UFT provided!
	//	userFunction = uft;
	//	ufType = UserFunctionType::_None;
	//	
	//	CHECKandTHROW(uft==0, "PythonUserFunctionBase: copy assignment operator called for UFT unintentionally; check C++ code!");

	//	return *this;
	//}

	operator UFT() const { return userFunction; }

};

//todo:
//allow settings user functions directly as py::object (py:isinstance<PyUserFunction...> )
//create C++ user functions:
// 1) create base class (templated carrier?)
//systematically create PythonUserFunctionBase / replace with old remove "TransUserFunctions"
//test with preStepUserFunction
//consider storing Python function in SetupUserFunction; alternatively: add copy method to Symbolic user functions?

//class PythonUserFunctions; //!< linked in MainSystem, because CSystem does not know about MainSystem
//! @python user functions to be called e.g., by solver ==> instantiated in CSystem, but MainSystem is not available there!
class PythonUserFunctions
{
public:
	MainSystem* mainSystem; //!< stored for call to preStepFunction

	//std::function<bool(const MainSystem& mainSystem, Real t)> preStepFunction;	//!< function called prior to the computation of a single step
	//std::function<bool(const MainSystem& mainSystem, Real t)> postStepFunction;	//!< function called at end of computation step, just before writing results
	//std::function<StdVector2D(const MainSystem& mainSystem, Real t)> postNewtonFunction;//!< function called after Newton method

	PythonUserFunctionBase< std::function<bool(const MainSystem& mainSystem, Real t)> > preStepFunction;
	PythonUserFunctionBase< std::function<bool(const MainSystem& mainSystem, Real t)> > postStepFunction;
	PythonUserFunctionBase< std::function<StdVector2D(const MainSystem& mainSystem, Real t)> > postNewtonFunction;

	PythonUserFunctions()
	{
		Reset();
	}
	void Reset()
	{
		mainSystem = nullptr;
		//preStepFunction = 0;
		//postStepFunction = 0;
		//postNewtonFunction = 0;
		preStepFunction.Reset();
		postStepFunction.Reset();
		postNewtonFunction.Reset();
	}
};


#endif
