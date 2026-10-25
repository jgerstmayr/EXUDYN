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
class PyMatrixContainer;

//+++++++++++++++++++++++++++++++++++++++++++++
//capsule for Python function + conversion to std::function
//class PyFunction; //!< pybind11::function

//class PyUFobject; //!< pybind11::object, to store either function or information (e.g., string to describe C++ function)
//typedef Real PyUFobject; //!< pybind11::object, to store either function or information (e.g., string to describe C++ function)
namespace pybind11 {
	class object;
};
namespace Symbolic {
	class PySymbolicUserFunction;
};


enum class UserFunctionType {
	_None = 0,		//! marks that no user function exists
	Python = 1,		//! Python user functions with corresponding std::function
	Symbolic = 2,	//! Python user function with corresponding symbolic function (as std::function)
	InternalCpp = 3,//! Internal user function
	Jit = 4,		//! Jitted user function
};

//! the main thread, which imports exudyn and runs the solvers and Python; the render thread draws and may not call user
//! functions (#3003)
void SetExudynMainThread();
bool IsExudynMainThread();
//! a user function was called from another thread than the main thread: set, reported once in the main thread
void ReportUserFunctionCallOutsideMainThread();
//! true once after a call of a user function outside the main thread was reported
bool UserFunctionCalledOutsideMainThread();

//! true if Python user functions get their vector arguments as lists instead of numpy arrays:
//! exudyn.special.deprecations.userFunctionListArguments, for the transition (#3008); reports the deprecated use
bool UserFunctionListArguments();

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

//! the number of user functions set so far, Python or symbolic, to any item: the lists of the parallel loops follow
//! a user function set during a simulation (#2978)
Index UserFunctionAssignmentCount();

//! what every user function has, whatever its arguments: its kind and its symbolic function; for the lists of the
//! parallel loops (#2978)
class UserFunctionBase
{
protected:
	UserFunctionType ufType;
	Symbolic::PySymbolicUserFunction* symbolicFunction; //!< the symbolic function held by pyObject, or nullptr (#3006)
	bool pythonArgument; //!< an argument is a Python object (ObjectConnectorUser): computed in the main thread
public:
	UserFunctionBase() : ufType(UserFunctionType::_None), symbolicFunction(nullptr), pythonArgument(false) {}

	UserFunctionType GetUFtype() const { return ufType; }
	//! the symbolic function, for an evaluation without the std::function (EvaluateUserFunctionIntoVector); nullptr for
	//! a Python function, and for a symbolic one set only as std::function (#3006)
	Symbolic::PySymbolicUserFunction* GetSymbolicFunction() const { return symbolicFunction; }
	//! return true, if user functions exists
	bool IsValid() const { return ufType != UserFunctionType::_None; }
	//! true, if it can be computed in the threads of the parallel loops: none, or a symbolic function evaluated by its
	//! flat program, which has a copy of its recording per thread; a Python function holds the GIL (#2978)
	bool ComputableInThreads() const;
	//! the copies of the recording of a symbolic function for numberOfThreads threads; nothing for the other kinds (#2978)
	void PrepareThreads(Index numberOfThreads) const;
};

//! true, if an argument of the user function is a Python object
template<typename T> struct UserFunctionHasPythonArgument : std::false_type {};
template<typename R, typename... Args> struct UserFunctionHasPythonArgument<std::function<R(Args...)>>
	: std::integral_constant<bool, (std::is_same_v<std::decay_t<Args>, pybind11::object> || ...)> {};

//! class to encapsulate user functions, according to UserFunctionType
template <typename UFT>
class PythonUserFunctionBase : public UserFunctionBase
{
private:
	pybind11::object* pyObject;
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
	//! the Python function (UserFunctionType::Python), for a call without the std::function (#3008)
	const pybind11::object& GetPythonObject() const { return *pyObject; }
	void Reset();

	//! comparison operator, just for comparison with 0
	bool operator==(Index value) const
	{
		CHECKandTHROW(value == 0, "PythonUserFunctionBase::operator==(): only allowed for comparison with 0", ExudynValueError);
		return !IsValid();
	}
	//! comparison operator, just for comparison with 0
	bool operator!=(Index value) const
	{
		CHECKandTHROW(value == 0, "PythonUserFunctionBase::operator!=(): only allowed for comparison with 0", ExudynValueError);
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
		CHECKandTHROW(value == 0, "PythonUserFunctionBase::operator=(): only allowed for assignment to 0", ExudynValueError);
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

	PythonUserFunctionBase< std::function<bool(const MainSystem& mainSystem, Real t)> > preStepFunction;
	PythonUserFunctionBase< std::function<bool(const MainSystem& mainSystem, Real t)> > postStepFunction;
	PythonUserFunctionBase< std::function<StdVector2D(const MainSystem& mainSystem, Real t)> > postNewtonFunction;

	PythonUserFunctionBase< std::function<void(const MainSystem& mainSystem, Real t, Index newtonIt, Index discontinuousIt)> > preNewtonResidualFunction;
	PythonUserFunctionBase< std::function<PyMatrixContainer(const MainSystem& mainSystem, Real t, Real factorODE2, Real factorODE2_t, Real factorODE1)> > systemJacobianFunction;

	PythonUserFunctions()
	{
		Reset();
	}
	void Reset()
	{
		mainSystem = nullptr;
		preStepFunction.Reset();
		postStepFunction.Reset();
		postNewtonFunction.Reset();

		preNewtonResidualFunction.Reset();
		systemJacobianFunction.Reset();
	}
};


#endif
