/** ***********************************************************************************************
* @brief		symbolic computation with vectors and matrices
*
* @author		Gerstmayr Johannes
* @date			2023-12-06
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef SYMBOLICUTILITIES__H
#define SYMBOLICUTILITIES__H

#include <initializer_list> //for initializer_list in constructor

extern bool linalgPrintUsePythonFormat; 

namespace Symbolic
{


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef py::array_t<Real> NumpyMatrix; //for NumpyMatrix return value 

//! class which holds any Symbolic type, like SReal and SVector
class SymbolicGeneric
{
private:
	Index type; //!< for now 0=SReal, 1=SVector, 2=SMatrix, 3=Index
	SReal* real;
	SymbolicRealVector* vector;
	SymbolicRealMatrix* matrix;
public:

	SymbolicGeneric() : type(0), real(nullptr), vector(nullptr), matrix(nullptr) {}
	//~SymbolicGeneric() { Delete(); } //do not add destructur, because data won't be initialized in ResizableArray! (consider resize, etc.!)
	void Initialize() { type = 0; real = nullptr; vector = nullptr; matrix = nullptr; }
	bool IsReal() const { return type == 0; }
	bool IsVector() const { return type == 1; }
	bool IsMatrix() const { return type == 2; }
	void SetType(Index t) { type = t; }
	void Delete()
	{
		//BECAUSE THERE IS NO new, we also do not delete!!!
		//if (type == 0) { if (real) { delete real; } }
		//else if (type == 1) { if (vector) { delete vector; } }
		//else if (type == 2) { if (matrix) { delete matrix;} }
		real = nullptr;
		vector = nullptr;
		matrix = nullptr;
		type = 0;
	}

	template<typename T>
	void Set(T value)
	{
		SetValue(value);
	}
	const SReal* GetReal() const { return real; }
	const SymbolicRealVector* GetVector() const { return vector; }
	const SymbolicRealMatrix* GetMatrix() const { return matrix; }
	SReal*& GetReal() { return real; }
	SymbolicRealVector*& GetVector() { return vector; }
	SymbolicRealMatrix*& GetMatrix() { return matrix; }

	STDstring ToString() const
	{
		if (IsReal()) { return real->ToString(); }
		else if (IsVector()) { return vector->ToString(); }
		else if (IsMatrix()) { return matrix->ToString(); }
		return ""; //should not happen ...
	}

private:
	void SetValue(SReal* value) 
	{ 
		Delete();
		type = 0;
		vector = nullptr;
		real = value;
		matrix = nullptr;
	}
	void SetValue(SymbolicRealVector* value) 
	{ 
		Delete();
		type = 1;
		real = nullptr;
		vector = value;
		matrix = nullptr;
	}
	void SetValue(SymbolicRealMatrix* value) 
	{ 
		Delete();
		type = 2;
		real = nullptr;
		vector = nullptr;
		matrix = value;
	}
};



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

//! base class for symbolic Real: can either be Real or contain a symbolic expression
//! a user function stores a SymbolicFunction, this is stored per object (but not necessarily replicating the object-tree!)
class SymbolicFunction
{
protected:
	//ResizableArray<ExpressionNamedReal*> argRealList;
	ResizableArray<SymbolicGeneric> argRealList;
	SymbolicGeneric returnValue;
	STDstring functionName; //!< will be set by derived class

public:
	SymbolicFunction() : //scalarType(true), vectorSize(0), 
		returnValue() {};

	//! set function name
	void SetFunctionName(const STDstring& s) { functionName = s; }
	const STDstring& GetFunctionName() const { return functionName; }
	//bool IsScalarType() const { return scalarType; }
	//Index VectorSize() const { return vectorSize; }

	void SetScalarType(SReal* sReal)
	{ 
		//scalarType = true; 
		//vectorSize = 0; 
		returnValue.Set(sReal);
	}
	void SetVectorType(Index size, SymbolicRealVector* sVector) 
	{ 
		//scalarType = false; 
		//vectorSize = size; 
		returnValue.Set(sVector);
	}

	//! free allocated linked reals and SReal
	virtual ~SymbolicFunction()
	{
		Delete();
	}

	virtual void Delete()
	{
		//done by destructors:
		for (SymbolicGeneric& linkedRealPtr : argRealList)
		{
			linkedRealPtr.Delete();
		}
		returnValue.Delete();
		argRealList.SetMaxNumberOfItems(0);
	}

	virtual STDstring ToString() const
	{
		return returnValue.ToString();
	}

	virtual Real EvaluateReturnValue() const
	{
		return returnValue.GetReal()->Evaluate();
	}

	virtual ResizableConstVector EvaluateReturnVector() const
	{
		return returnValue.GetVector()->Evaluate();
	}

	//! the returned vector without a copy, valid until the function is evaluated again (#3006)
	const ResizableConstVector& EvaluateReturnVectorRef() const
	{
		return returnValue.GetVector()->EvaluateRef();
	}

	virtual ResizableConstMatrix EvaluateReturnMatrix() const
	{
		return returnValue.GetMatrix()->Evaluate();
	}
	////evaluate all expression trees, storing results in Real*, which are linked to destination of function 
	//void EvaluateReturnValues()
	//{
	//	returnValues.resize(returnRealList.NumberOfItems());
	//	for (Index i = 0; i < returnRealList.NumberOfItems(); ++i)
	//	{
	//		returnValues[i] = returnRealList[i]->Evaluate();
	//	}
	//}



};

//! symbolic function for Python user functions
class PySymbolicUserFunction : public SymbolicFunction
{
	py::dict functionDict; //!< store a copy of the dictionary, to keep the expression trees alive

	//! list here different kinds of user functions:
	//std::function<Real(const MainSystem&, Real, Index, Real, Real, Real, Real, Real)> mbsScalarIndexScalar5;
public:
	PySymbolicUserFunction() : SymbolicFunction() {}

	//! read out and store user function from given Python dictionary
	//! this function is called by particular user function
	virtual void SetupUserFunction(py::dict pyObject, const STDstring& itemTypeName, const STDstring& userFunctionName)
	{
		const STDstring keyFunction = "functionName";
		const STDstring keyArgList = "argList";
		const STDstring keyArgTypeList = "argTypeList";
		const STDstring keyReturnValue = "returnValue";
		const STDstring keyReturnType = "returnType";

		auto where = [&itemTypeName, &userFunctionName]() { return STDstring("In ") + itemTypeName + ", user function " + userFunctionName + ": "; };

		functionDict = py::cast<py::dict>(pyObject);
		if (!functionDict.contains(keyFunction) || !py::isinstance<py::str>(functionDict[keyFunction.c_str()]))
		{
			PyError(STDstring(where() + "expected dict with key '") + keyFunction + "' representing a string", PyErrorType::valueError);
		}
		SetFunctionName(py::cast<STDstring>(functionDict[keyFunction.c_str()]));

		if (!functionDict.contains(keyArgList) || !py::isinstance<py::list>(functionDict[keyArgList.c_str()]))
		{
			PyError(STDstring(where() + "expected dict with key '") + keyArgList + "' representing a list", PyErrorType::valueError);
		}
		py::list argList = py::cast<py::list>(functionDict[keyArgList.c_str()]);

		if (!functionDict.contains(keyArgTypeList) || !py::isinstance<py::list>(functionDict[keyArgTypeList.c_str()]))
		{
			PyError(STDstring(where() + "expected dict with key '") + keyArgTypeList + "' representing a list", PyErrorType::valueError);
		}
		py::list argTypeList = py::cast<py::list>(functionDict[keyArgTypeList.c_str()]);

		if (!functionDict.contains(keyReturnValue) )
		{
			PyError(STDstring(where() + "expected dict with key '") + keyReturnValue + "'", PyErrorType::valueError);
		}
		if (!functionDict.contains(keyReturnType) || !py::isinstance<py::str>(functionDict[keyReturnType.c_str()]))
		{
			PyError(STDstring(where() + "expected dict with key '") + keyReturnType + "' representing a string", PyErrorType::valueError);
		}
		STDstring returnTypeStr = py::cast<STDstring>(functionDict[keyReturnType.c_str()]);
		py::object pyReturnValue = functionDict[keyReturnValue.c_str()];

		Delete(); //delete argRealList, if an earlier list exists

		//py::print("argList=", argList);
		std::vector<py::object> stdArgList = py::cast<std::vector<py::object>>(argList);
		Index nArgs = (Index)stdArgList.size();

		//process arguments
		argRealList.SetMaxNumberOfItems(nArgs);
		for (Index i = 0; i < nArgs; i++)
		{
			if (i != 0) //first arg is mbs, but not convertible for now; // if (!py::isinstance<MainSystem>(stdArgList[i]))
			{
				//if instance = SReal ...
				SymbolicGeneric symGeneric; 
				symGeneric.Initialize(); //initialize symGeneric
				const py::object& arg = stdArgList[i];
				STDstring argType = py::cast<std::string>(argTypeList[i]);
				if (argType == "Real" || argType == "Index" || argType == "bool" || argType == "OutputVariableType") //a flag, an enum: as Real (#2985)
				{
					symGeneric.Set(py::cast<SReal*>(arg));
				}
				else if (ArgTypeIsVector(argType)) //no need to distinguish 3D, 6D and general vector
				{
					symGeneric.Set(py::cast<SymbolicRealVector*>(arg));
				}
				else if (ArgTypeIsMatrix(argType)) //no need to distinguish 3D, 6D and general vector
				{
					symGeneric.Set(py::cast<SymbolicRealMatrix*>(arg));
				}
				else
				{
					CHECKandTHROWstring("SymbolicUserFunction::SetupUserFunction: invalid argument type '"+argType+"' found.Valid args are float, int, MainSystem, list, list of lists and numpy array");
				}
				argRealList.Append(symGeneric);
			}
		}

		GenericExceptionHandling([&]
		{
			//process return type and set function
			if (returnTypeStr == "Real" || returnTypeStr == "bool")
			{
				SetScalarType(py::cast<SReal*>(pyReturnValue)); //pyReturnValue is Real
			}
			else if (ArgTypeIsVector(returnTypeStr))
			{
				Index returnSize = -1; //unknown vector size => not needed here
				if (returnTypeStr.find("2D") != std::string::npos) { returnSize = 2; }
				if (returnTypeStr.find("3D") != std::string::npos) { returnSize = 3; }
				if (returnTypeStr.find("6D") != std::string::npos) { returnSize = 6; }
				SetVectorType(returnSize, py::cast<SymbolicRealVector*>(pyReturnValue)); //pyReturnValue is list/numpy array
			}
			else if (ArgTypeIsMatrix(returnTypeStr))
			{
				CHECKandTHROWstring("SymbolicUserFunction::SetupUserFunction: Matrix returnType not implemented");
			}
			else
			{
				pout << "returnType: '" << returnTypeStr << "'\n";
				CHECKandTHROWstring("SymbolicUserFunction::SetupUserFunction: invalid returnType");
			}
		}, "SymbolicUserFunction::SetupUserFunction (probably casting of return values failed)");
		//for (Index i = 0; i < nReturn; i++)
		//{
		//	returnRealList[i] = (py::cast<SReal*>(stdReturnList[i]));
		//}

	}

	virtual bool ArgTypeIsVector(const STDstring& argType) const
	{
		return (argType == "StdVector" || argType == "StdVector2D" || argType == "StdVector3D" || argType == "StdVector6D");
	}

	virtual bool ArgTypeIsMatrix(const STDstring& argType) const
	{
		return (argType == "NumpyMatrix" || argType == "StdMatrix3D" || argType == "StdMatrix6D");
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	Real EvaluateReal(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return EvaluateReturnValue();
		//EvaluateReturnValues(); //evaluate symbolic expression trees

		//return returnValues[0];
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	bool EvaluateBool(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return bool(EvaluateReturnValue());
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdVector2D EvaluateStdVector2D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdVector2D)EvaluateReturnVectorRef();
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdVector3D EvaluateStdVector3D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdVector3D)EvaluateReturnVectorRef();
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdVector6D EvaluateStdVector6D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdVector6D)EvaluateReturnVectorRef();
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	std::vector<Real> EvaluateStdVector(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdVector)EvaluateReturnVectorRef();
	}

	//! a user function that returns a vector, evaluated into result, which keeps its memory when it has the size;
	//! the arguments may be Exudyn vectors, which are copied into the named variables (#3006)
	template<typename... Args>
	void EvaluateIntoVector(Vector& result, const MainSystem& mainSystem, const Args&... args)
	{
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		result.CopyFrom(EvaluateReturnVectorRef());
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdMatrix3D EvaluateStdMatrix3D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdMatrix3D)EvaluateReturnMatrix();
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdMatrix6D EvaluateStdMatrix6D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdMatrix6D)EvaluateReturnMatrix();
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	NumpyMatrix EvaluateNumpyMatrix(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (NumpyMatrix)EvaluateReturnMatrix();
	}




	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! test evaluating user function
	//! this would not run in parallel!
	py::object PyEvaluateUF(/*const MainSystem& mainSystem, */const py::args& args)
	{
		//namespace py = pybind11;
		//the arguments as the function was recorded with them, after mbs: a Real - also an int, a bool and an
		//OutputVariableType -, a vector or a matrix (#2985)
		Index argIndex = 0;
		for (const auto& arg : args) {
			if (py::isinstance<MainSystem>(arg)) { continue; } //MainSystem is not stored for now
			CHECKandTHROW(argIndex < argRealList.NumberOfItems(), "Symbolic::SymbolicFunction::Evaluate: too many arguments");
			SymbolicGeneric& symbolicArg = argRealList[argIndex];
			if (symbolicArg.IsReal())
			{
				Real value = 0;
				if (py::isinstance<OutputVariableType>(arg)) { value = (Real)(Index64)py::cast<OutputVariableType>(arg); }
				else if (py::isinstance<py::float_>(arg) || py::isinstance<py::int_>(arg)) { value = arg.cast<Real>(); }
				else { PyError(STDstring("Symbolic::SymbolicFunction::Evaluate: argument ") + EXUstd::ToString(argIndex) + " must be a number", PyErrorType::valueError); }
				symbolicArg.GetReal()->SetExpressionNamedReal(value);
			}
			else if (symbolicArg.IsVector())
			{
				symbolicArg.GetVector()->SetExpressionNamedVector(ResizableConstVector(arg.cast<std::vector<Real> >()));
			}
			else
			{
				symbolicArg.GetMatrix()->SetSymbolicMatrix(py::array_t<Real>::ensure(arg));
			}
			argIndex++;
		}

		if (returnValue.IsReal())
		{
			return py::cast<Real>(EvaluateReturnValue());
		}
		else
		{
			return py::cast<StdVector>(EvaluateReturnVector());
		}
		//EvaluateReturnValues(); //evaluate symbolic expression trees

		//if (returnValues.size() == 1)
		//{
		//	return py::cast(returnValues[0]);
		//}
		//else
		//{
		//	return py::cast(returnValues);
		//	//return py::cast((std::vector<Real>)returnValues);
		//}
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! get itemName from itemIndex
	void GetItemTypeName(MainSystem& mainSystem, const py::object& pyItemIndex, 
		STDstring& indexType, STDstring& itemTypeName, Index& itemNumber)
	{

		if (py::isinstance<ObjectIndex>(pyItemIndex)) //sType == "ObjectIndex")
		{
			ObjectIndex itemIndex = py::cast<ObjectIndex>(pyItemIndex);
			itemNumber = itemIndex.GetIndex();
			indexType = itemIndex.GetTypeString();

			CHECKandTHROW(itemNumber < mainSystem.GetCSystem().GetSystemData().GetCObjects().NumberOfItems(),
				"Symbolic::SymbolicUserFunction: illegal objectNumber");
			itemTypeName = STDstring("Object")+mainSystem.GetMainSystemData().GetMainObjects()[itemNumber]->GetTypeName();
		}
		else if (py::isinstance<LoadIndex>(pyItemIndex)) //(sType == "LoadIndex")
		{
			LoadIndex itemIndex = py::cast<LoadIndex>(pyItemIndex);
			itemNumber = itemIndex.GetIndex();
			indexType = itemIndex.GetTypeString();

			CHECKandTHROW(itemNumber < mainSystem.GetCSystem().GetSystemData().GetCLoads().NumberOfItems(),
				"Symbolic::SymbolicUserFunction: illegal loadNumber");
			itemTypeName = STDstring("Load") + mainSystem.GetMainSystemData().GetMainLoads()[itemNumber]->GetTypeName();
		}
		else if (py::isinstance<py::int_>(pyItemIndex)) //MainSystem
		{
			//Index itemIndex = py::cast<Index>(pyItemIndex);
			CHECKandTHROW(py::cast<Index>(pyItemIndex) == -1, "Symbolic::SymbolicUserFunction: invalid item type (must be ObjectIndex, LoadIndex or -1 for MainSystem)");
			indexType = "None";
			itemTypeName = "MainSystem";
			itemNumber = -1;
		}
		else
		{
			PyError(STDstring("Symbolic::GetItemTypeName") + ": invalid item type (must be ObjectIndex, LoadIndex or -1 for MainSystem)", PyErrorType::typeError);
		}
	}


	//automatically generated functions for all kinds of user functions
#include "Autogenerated/PySymbolicUserFunctionSet.h"

public:
	//! this function allows access to all std::functions by name, e.g.:
	//! auto f = GetSTDfunction(PySymbolicUserFunction::mbsScalar2);
	//! WORKS?
	//template<typename T>
	//const T& GetSTDfunction(T PySymbolicUserFunction::*stdFunction) const {
	//	return this->*stdFunction;
	//}

	////! for specific user function type, get user function by type and name
	//template<typename UFT>
	//UFT GetSTDfunction() const
	//{
	//	if constexpr (std::is_same_v<UFT, std::function<bool(const MainSystem &, Real)>>)
	//	{
	//		return boolMbsScalar;
	//	}
	//	else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&, Real, Real)>>)
	//	{
	//		return mbsScalar2;
	//	}
	//	return UFT(0); //will never happen, but is needed
	//}



protected:
	// Function to process Real arguments
	void processArgument(Real arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		argRealList[argIndex++].GetReal()->SetExpressionNamedReal(arg);
	}
	//! an OutputVariableType as the Real of its value, as an Index is passed (#2989)
	void processArgument(OutputVariableType arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		argRealList[argIndex++].GetReal()->SetExpressionNamedReal((Real)(Index64)arg);
	}
	void processArgument(const ResizableConstVector& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		argRealList[argIndex++].GetVector()->SetExpressionNamedVector(arg);
	}
	//! a vector argument, copied into the memory of the named vector, which keeps it: no allocation (#3006)
	void processArgument(const StdVector& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		argRealList[argIndex++].GetVector()->GetExpressionNamedReal().SetVector(arg);
	}
	//! a vector of fixed size, e.g. the Vector3D of a local position (#2995)
	template<Index n>
	void processArgument(const SlimVectorBase<Real, n>& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		ResizableConstVector& vector = argRealList[argIndex++].GetVector()->GetExpressionNamedReal().GetVector();
		vector.SetNumberOfItems(n);
		std::copy(arg.begin(), arg.end(), vector.begin());
	}
	void processArgument(const VectorBase<Real>& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		ResizableConstVector& vector = argRealList[argIndex++].GetVector()->GetExpressionNamedReal().GetVector();
		vector.SetNumberOfItems(arg.NumberOfItems());
		std::copy(arg.begin(), arg.end(), vector.begin());
	}
	void processArgument(const StdVector3D& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		//directly inject values into stored vector (no new for larger vectors)
		ResizableConstVector& vector = argRealList[argIndex++].GetVector()->GetExpressionNamedReal().GetVector();
		vector.SetVector({ arg[0], arg[1], arg[2] });

		//argRealList[argIndex++].GetVector()->SetExpressionNamedVector(ResizableConstVector({ arg[0], arg[1], arg[2]}));
	}
	void processArgument(const StdVector6D& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		ResizableConstVector& vector = argRealList[argIndex++].GetVector()->GetExpressionNamedReal().GetVector();
		vector.SetVector({ arg[0], arg[1], arg[2],  arg[3], arg[4], arg[5] });
		//argRealList[argIndex++].GetVector()->SetExpressionNamedVector(
		//	ResizableConstVector({ arg[0], arg[1], arg[2],  arg[3], arg[4], arg[5] }));
	}

	void processArgument(const ResizableConstMatrix& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		argRealList[argIndex++].GetMatrix()->SetExpressionNamedMatrix(arg);
	}
	void processArgument(const StdMatrix3D & arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		//directly inject values into stored matrix (no new for larger vectors)
		ResizableConstMatrix& matrix = argRealList[argIndex++].GetMatrix()->GetExpressionNamedReal().GetMatrix();
		matrix.SetMatrix(3,3, 
				{ arg[0][0], arg[0][1], arg[0][2],
				  arg[1][0], arg[1][1], arg[1][2],
				  arg[2][0], arg[2][1], arg[2][2]
				});

		//argRealList[argIndex++].GetMatrix()->SetExpressionNamedMatrix(ResizableConstMatrix(3, 3,
		//	{ arg[0][0], arg[0][1], arg[0][2],
		//	  arg[1][0], arg[1][1], arg[1][2],
		//	  arg[2][0], arg[2][1], arg[2][2]
		//	}));
	}
	void processArgument(const StdMatrix6D& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		ResizableConstMatrix& matrix = argRealList[argIndex++].GetMatrix()->GetExpressionNamedReal().GetMatrix();
		matrix.SetNumberOfRowsAndColumns(6, 6);
		//ResizableConstMatrix matrix(6, 6);
		for (Index i = 0; i < 6; i++)
		{
			for (Index j = 0; j < 6; j++)
			{
				matrix(i, j) = arg[i][j];
			}
		}
		//argRealList[argIndex++].GetMatrix()->SetExpressionNamedMatrix(m);
	}
	// Function to process Index arguments
	//void processArgument(Index arg, ResizableArray<SReal*>& argRealList, Index& argIndex)
	//{
	//	argRealList[argIndex++]->SetExpressionNamedReal((Real)arg);
	//	//argRealList[argIndex++]->SetExpressionNamedIndex(arg);
	//}

};


}; //namespace Symbolic

//! true for SlimVectorBase<Real, n>, e.g. Vector3D (#2995)
template<typename T> struct IsSlimVectorOfReal : std::false_type {};
template<Index n> struct IsSlimVectorOfReal<SlimVectorBase<Real, n>> : std::true_type { static constexpr Index size = n; };

//! the argument of a user function's std::function: an Exudyn vector as std::vector, a SlimVector as std::array,
//! everything else as it is (#3006, #2995)
template<typename T>
inline decltype(auto) UserFunctionArgument(const T& argument)
{
	if constexpr (std::is_base_of_v<VectorBase<Real>, T>) { return (StdVector)argument; }
	else if constexpr (IsSlimVectorOfReal<T>::value)
	{
		std::array<Real, IsSlimVectorOfReal<T>::size> array;
		std::copy(argument.begin(), argument.end(), array.begin());
		return array;
	}
	else { return (const T&)argument; }
}

//! true for std::array<Real, n>, the vectors of fixed size of the user functions (StdVector2D, StdVector3D, StdVector6D)
template<typename T> struct IsStdArrayOfReal : std::false_type {};
template<std::size_t n> struct IsStdArrayOfReal<std::array<Real, n>> : std::true_type {};
//! true for std::array<std::array<Real, n>, m>, the matrices of fixed size (StdMatrix3D, StdMatrix6D)
template<typename T> struct IsStdArrayMatrixOfReal : std::false_type {};
template<std::size_t m, std::size_t n> struct IsStdArrayMatrixOfReal<std::array<std::array<Real, n>, m>> : std::true_type {};

//! the argument of a Python user function: a vector - of run-time size, std::vector or Exudyn vector, or of fixed size,
//! std::array - and a matrix of fixed size as read-only numpy array that views its memory, valid during the call, or as
//! list with userFunctionListArguments; everything else as it is, cast by pybind11 as its own wrapper of a function
//! does (#3008, #3011: a single item of a small numpy array is slower to compute with than one of a list, the
//! vectorized form is what numpy is for)
template<typename T>
inline decltype(auto) PythonUserFunctionArgument(const T& argument)
{
	if constexpr (IsStdArrayOfReal<T>::value)
	{
		if (UserFunctionListArguments()) { return py::object(py::cast(argument)); }
		return EPyUtils::ToNumpyView(argument.data(), (Index)argument.size());
	}
	else if constexpr (IsStdArrayMatrixOfReal<T>::value)
	{
		static_assert(sizeof(T) == sizeof(Real) * std::tuple_size<T>::value * std::tuple_size<typename T::value_type>::value,
			"PythonUserFunctionArgument: the rows of the matrix must be contiguous");
		if (UserFunctionListArguments()) { return py::object(py::cast(argument)); }
		return EPyUtils::ToNumpyView(argument[0].data(), (Index)argument.size(), (Index)argument[0].size());
	}
	else
	if constexpr (std::is_same_v<T, StdVector>)
	{
		if (UserFunctionListArguments()) { return py::object(py::cast(argument)); }
		return EPyUtils::ToNumpyView(argument.data(), (Index)argument.size());
	}
	else if constexpr (std::is_base_of_v<VectorBase<Real>, T>)
	{
		if (UserFunctionListArguments()) { return py::object(py::cast((StdVector)argument)); }
		return EPyUtils::ToNumpyView(argument.GetDataPointer(), argument.NumberOfItems());
	}
	else if constexpr (IsSlimVectorOfReal<T>::value) //Vector3D, ... (#2995)
	{
		if (UserFunctionListArguments()) { return py::object(py::cast((StdVector)argument)); }
		return EPyUtils::ToNumpyView(argument.GetDataPointer(), argument.NumberOfItems());
	}
	else if constexpr (std::is_same_v<T, MainSystem>)
	{
		//by reference: a const reference would be copied - MainSystem cannot - when its Python object no longer exists,
		//e.g. a model built in a function that returned only the SystemContainer (#3003)
		return py::cast(&argument, py::return_value_policy::reference);
	}
	else { return (const T&)argument; }
}

//! the result of a Python user function as the C++ type R; a vector from numpy in one pass (#3008)
template<typename R>
inline R PythonUserFunctionResult(const py::object& result)
{
	if constexpr (std::is_same_v<R, StdVector>)
	{
		StdVector vector;
		EPyUtils::CopyFromPython(result, vector);
		return vector;
	}
	else if constexpr (IsStdArrayOfReal<R>::value)
	{
		if (py::isinstance<py::array>(result)) //a numpy array of the size in one pass
		{
			auto array = py::array_t<Real, py::array::c_style | py::array::forcecast>::ensure(result);
			if (array && array.ndim() == 1 && (std::size_t)array.size() == std::tuple_size<R>::value)
			{
				R vector;
				std::copy(array.data(), array.data() + array.size(), vector.begin());
				return vector;
			}
		}
		return result.cast<R>();
	}
	else { return result.cast<R>(); }
}

//! holds the Python function of a std::function and acquires the GIL when it releases it, as pybind11 does (#3008)
class PythonFunctionHandle
{
public:
	py::object function;
	PythonFunctionHandle(const py::object& functionInit) : function(functionInit) {}
	~PythonFunctionHandle()
	{
		py::gil_scoped_acquire gil;
		py::object release(std::move(function));
	}
};

//! the std::function of a Python user function, with numpy arguments instead of lists (#3008); the second argument
//! selects the signature
template<typename R, typename... Args>
std::function<R(Args...)> PythonUserFunctionToStdFunction(const py::object& function, std::function<R(Args...)>*)
{
	std::shared_ptr<PythonFunctionHandle> handle = std::make_shared<PythonFunctionHandle>(function);
	return [handle](Args... args) -> R
	{
		if (!IsExudynMainThread()) { ReportUserFunctionCallOutsideMainThread(); } //the render thread (#3003)
		py::gil_scoped_acquire gil;
		py::object result = handle->function(PythonUserFunctionArgument(args)...);
		if constexpr (!std::is_void_v<R>) { return PythonUserFunctionResult<R>(result); }
	};
}

//! evaluate a user function to its result type: a Python function directly with numpy views of the arguments, also of
//! Exudyn vectors, a symbolic one through its std::function (#2995)
template<typename UFT, typename... Args>
typename UFT::result_type EvaluateUserFunction(const PythonUserFunctionBase<UFT>& userFunction, const MainSystem& mainSystem,
	const Args&... args)
{
	using R = typename UFT::result_type;
	if (userFunction.GetUFtype() == UserFunctionType::Python)
	{
		if (!IsExudynMainThread()) { ReportUserFunctionCallOutsideMainThread(); } //the render thread (#3003)
		py::gil_scoped_acquire gil;
		py::object result = userFunction.GetPythonObject()(PythonUserFunctionArgument(mainSystem), PythonUserFunctionArgument(args)...);
		if constexpr (!std::is_void_v<R>) { return PythonUserFunctionResult<R>(result); }
	}
	else
	{
		return userFunction.userFunction(mainSystem, UserFunctionArgument(args)...);
	}
}

//! evaluate a user function that returns a vector into result, which keeps its memory when it has the size: a symbolic
//! function directly - its arguments copied into its named vectors, its result into result, without std::function and
//! std::vector -, a Python function through its std::function; the arguments may be Exudyn vectors (#3006)
template<typename UFT, typename... Args>
void EvaluateUserFunctionIntoVector(const PythonUserFunctionBase<UFT>& userFunction, Vector& result,
	const MainSystem& mainSystem, const Args&... args)
{
	if (userFunction.GetSymbolicFunction())
	{
		userFunction.GetSymbolicFunction()->EvaluateIntoVector(result, mainSystem, args...);
	}
	else if (userFunction.GetUFtype() == UserFunctionType::Python) //the Python function directly, with numpy views (#3008)
	{
		py::gil_scoped_acquire gil;
		py::object pyResult = userFunction.GetPythonObject()(PythonUserFunctionArgument(mainSystem),
			PythonUserFunctionArgument(args)...);
		EPyUtils::CopyFromPython(pyResult, result);
	}
	else
	{
		result.CopyFrom(userFunction.userFunction(mainSystem, UserFunctionArgument(args)...));
	}
}

#endif //SYMBOLIC__H
