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
	//bool scalarType;
	//Index vectorSize; //may be -1 if unknown
	//ResizableConstVector returnValues; //temporary values; as argRealList is required, returnvalues can also be stored
	//alternative, but not better: std::vector<Real> returnValues; //temporary values; immediate cast to Python or internal std::function
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
			PyError(STDstring(where() + "expected dict with key '") + keyFunction + "' representing a string");
		}
		SetFunctionName(py::cast<STDstring>(functionDict[keyFunction.c_str()]));

		if (!functionDict.contains(keyArgList) || !py::isinstance<py::list>(functionDict[keyArgList.c_str()]))
		{
			PyError(STDstring(where() + "expected dict with key '") + keyArgList + "' representing a list");
		}
		py::list argList = py::cast<py::list>(functionDict[keyArgList.c_str()]);

		if (!functionDict.contains(keyArgTypeList) || !py::isinstance<py::list>(functionDict[keyArgTypeList.c_str()]))
		{
			PyError(STDstring(where() + "expected dict with key '") + keyArgTypeList + "' representing a list");
		}
		py::list argTypeList = py::cast<py::list>(functionDict[keyArgTypeList.c_str()]);

		if (!functionDict.contains(keyReturnValue) )
		{
			PyError(STDstring(where() + "expected dict with key '") + keyReturnValue + "'");
		}
		if (!functionDict.contains(keyReturnType) || !py::isinstance<py::str>(functionDict[keyReturnType.c_str()]))
		{
			PyError(STDstring(where() + "expected dict with key '") + keyReturnType + "' representing a string");
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
				if (argType == "Real" || argType == "Index")
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
			if (returnTypeStr == "Real")
			{
				SetScalarType(py::cast<SReal*>(pyReturnValue)); //pyReturnValue is Real
			}
			else if (ArgTypeIsVector(returnTypeStr))
			{
				Index returnSize = -1; //unknown vector size => not needed here
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
		return (argType == "StdVector" || argType == "StdVector3D" || argType == "StdVector6D");
	}

	virtual bool ArgTypeIsMatrix(const STDstring& argType) const
	{
		return (argType == "NumpyMatrix" || argType == "StdMatrix3D" || argType == "StdMatrix6D");
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	Real EvaluateReal(const MainSystem& mainSystem, Args... args)
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
	StdVector3D EvaluateStdVector3D(const MainSystem& mainSystem, Args... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdVector3D)EvaluateReturnVector();
		//EvaluateReturnValues(); //evaluate symbolic expression trees
		//CHECKandTHROW(returnValues.size() == 3, "SymbolicUserFunction::EvaluateVector3D: size mismatch");
		//return StdVector3D({ returnValues[0],returnValues[1],returnValues[2] });
		//return StdVector3D(returnValues);
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdVector6D EvaluateStdVector6D(const MainSystem& mainSystem, Args... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdVector6D)EvaluateReturnVector();
		//EvaluateReturnValues(); //evaluate symbolic expression trees

		//CHECKandTHROW(returnValues.size() == 6, "SymbolicUserFunction::EvaluateVector6D: size mismatch");
		//return StdVector6D({ returnValues[0],returnValues[1],returnValues[2],
		//	returnValues[3],returnValues[4],returnValues[5],
		//	});
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	std::vector<Real> EvaluateStdVector(const MainSystem& mainSystem, Args... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdVector)EvaluateReturnVector(); //slow, but for bigger objects, this should not affect so much
		//EvaluateReturnValues(); //evaluate symbolic expression trees

		//return returnValues;
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdMatrix3D EvaluateStdMatrix3D(const MainSystem& mainSystem, Args... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdMatrix3D)EvaluateReturnMatrix();
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdMatrix6D EvaluateStdMatrix6D(const MainSystem& mainSystem, Args... args)
	{
		// Process the rest of the arguments
		Index argIndex = 0;
		(..., processArgument(args, argRealList, argIndex));

		return (StdMatrix6D)EvaluateReturnMatrix();
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	NumpyMatrix EvaluateNumpyMatrix(const MainSystem& mainSystem, Args... args)
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
		Index argIndex = 0;
		for (const auto& arg : args) {
			// Check if the argument is a float
			if (py::isinstance<py::float_>(arg))
			{
				argRealList[argIndex++].GetReal()->SetExpressionNamedReal(arg.cast<Real>());
			}
			// Check if the argument is a string
			else if (py::isinstance<py::int_>(arg))
			{
				argRealList[argIndex++].GetReal()->SetExpressionNamedReal(arg.cast<Index>());
			}
			else if (py::isinstance<py::list>(arg) || py::isinstance<py::array>(arg))
			{
				argRealList[argIndex++].GetVector()->SetExpressionNamedVector(arg.cast<std::vector<Real> >());
			}
			else if (py::isinstance<MainSystem>(arg)) {
				//argIndex++; //do nothing, as MainSystem is not stored for now
			}
			else {
				PyError(STDstring("Symbolic::SymbolicFunction::Evaluate: invalid argument ") + EXUstd::ToString(argIndex) + ", type is not supported");
			}
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
	template<typename TItemIndex>
	STDstring GetItemTypeName(MainSystem& mainSystem, const TItemIndex& itemIndex)
	{
		STDstring sType = itemIndex.GetTypeString();
		Index itemNumber = itemIndex.GetIndex();
		if (sType == "ObjectIndex")
		{
			CHECKandTHROW(itemNumber < mainSystem.GetCSystem()->GetSystemData().GetCObjects().NumberOfItems(),
				"Symbolic::SymbolicUserFunction: illegal objectNumber");
			return mainSystem.GetMainSystemData().GetMainObjects()[itemNumber]->GetTypeName();
		}
		else if (sType == "LoadIndex")
		{
			CHECKandTHROW(itemNumber < mainSystem.GetCSystem()->GetSystemData().GetCLoads().NumberOfItems(),
				"Symbolic::SymbolicUserFunction: illegal loadNumber");
			return mainSystem.GetMainSystemData().GetMainLoads()[itemNumber]->GetTypeName();
		}
		else
		{
			PyError(STDstring("Symbolic::GetItemTypeName") + ": invalid item type(must be Object or Load)");
			return "";
		}
	}


	//automatically generated functions for all kinds of user functions
#include "Autogenerated/PySymbolicUserFunctionTransfer.h"
#include "Autogenerated/PySymbolicUserFunctionSet.h"





protected:
	// Function to process Real arguments
	void processArgument(Real arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		argRealList[argIndex++].GetReal()->SetExpressionNamedReal(arg);
	}
	void processArgument(const ResizableConstVector& arg, ResizableArray<SymbolicGeneric>& argRealList, Index& argIndex)
	{
		argRealList[argIndex++].GetVector()->SetExpressionNamedVector(arg);
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

#endif //SYMBOLIC__H
