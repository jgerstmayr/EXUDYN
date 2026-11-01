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
#include "Linalg/MatrixContainer.h" //matrix results of symbolic user functions (#3020)
#include "Utilities/Threading.h" //the thread of an evaluation in the parallel loops (#2978)
#include "Linalg/SymbolicTable.h" //SymbolicProgram, the evaluation by the table of operations (#3153)
#include "Pymodules/PyHomogeneousTransformation.h" //exudyn.HT and exudyn.Twist, arguments of the user connectors (#3183)

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
	//! a matrix result built from components with constant zeros: its components and the pattern of the others, which
	//! are evaluated into triplets of a sparse MatrixContainer (#3022); nullptr for a dense result
	const MatrixExpressionSReal* resultComponents = nullptr;
	std::vector<std::pair<Index, Index>> resultPattern;

	//the flat evaluation (#3053, #3054): the result is evaluated by a list of operations built once after the
	//recording, computed by the table of operations (#3153, #3171); the tree evaluation stays for comparison
	//(flatEvaluation = false)
	FlatProgram flatProgram;
	bool flatEvaluation = true;
	SymbolicProgram symbolicProgram;

	//the evaluation in the threads of the parallel loops (#2978): the thread t > 0 evaluates its own copy of the
	//recording - its operations, which keep the values of their vectors and matrices, its arguments and its flat
	//program; the numbers and the variables other than the arguments are shared, as an evaluation only reads them
	struct ThreadCopy
	{
		FlatNode root;					//!< the result of the copy, referenced by it
		std::vector<FlatNode> arguments; //!< the leaf of argument i in the copy, referenced by it; empty if the recording does not use it
		FlatProgram flatProgram;
		SymbolicProgram symbolicProgram;
	};
	std::vector<ThreadCopy*> threadCopies; //!< the copy of thread t is threadCopies[t-1]
	bool copyable = false;				 //!< the recording can be evaluated in the threads, see ThreadCopiesPossible()

	//the outputs of items the recording reads (#2984): they get the MainSystem of the call while it evaluates; a
	//recording with them is evaluated in the main thread, as they are no leaves of values
	std::vector<const VectorExpressionItemOutput*> itemOutputs;
	bool evaluatingItemOutputs = false;
	//! sets the MainSystem of the call into the outputs of items and takes it back, also on an error; an output that
	//! calls this function again - the item reading itself - raises instead of overwriting the evaluation
	struct ItemOutputScope
	{
		SymbolicFunction& function;
		ItemOutputScope(SymbolicFunction& functionInit, const MainSystem& mainSystem) : function(functionInit)
		{
			if (function.itemOutputs.empty()) { return; }
			CHECKandTHROW(!function.evaluatingItemOutputs, "symbolic user function " + function.functionName
				+ ": an output that it reads by the MainSystem calls the function again; an item cannot read its own output in its user function");
			function.evaluatingItemOutputs = true;
			for (const VectorExpressionItemOutput* output : function.itemOutputs) { output->mainSystem = &mainSystem; }
		}
		~ItemOutputScope()
		{
			if (function.itemOutputs.empty()) { return; }
			for (const VectorExpressionItemOutput* output : function.itemOutputs) { output->mainSystem = nullptr; }
			function.evaluatingItemOutputs = false;
		}
	};

	//! the copy of the thread that evaluates, nullptr for the main thread
	ThreadCopy* CurrentThreadCopy() const
	{
		Index thread = ExuThreading::TaskManager::GetThreadId();
		if (thread == 0) { return nullptr; }
		CHECKandTHROW(thread <= (Index)threadCopies.size(), "symbolic user function " + functionName
			+ ": evaluated in a thread for which it has no copy of its recording");
		return threadCopies[thread - 1];
	}
	//! true, if the recording can be evaluated in the threads (#2978): by copies of its flat program, if every operation
	//! without operands is a number, a vector, a matrix or a variable; without a flat program, if the result is a
	//! constant or a leaf other than an argument, which an evaluation only reads
	bool ThreadCopiesPossible() const;
	//! the node of the result; nullptr for a constant result
	FlatNode ResultNode() const
	{
		if (returnValue.IsReal() && returnValue.GetReal()) { return FlatNode(returnValue.GetReal()->GetExpression()); }
		if (returnValue.IsVector() && returnValue.GetVector()) { return FlatNode(returnValue.GetVector()->GetExpression()); }
		if (returnValue.IsMatrix() && returnValue.GetMatrix()) { return FlatNode(returnValue.GetMatrix()->GetExpression()); }
		return FlatNode();
	}
//! free the copies of the threads; they refer to the leaves of the recording
	void DeleteThreadCopies();

	//! build the flat program of the result
	void BuildFlatProgram()
	{
		DeleteThreadCopies();
		flatProgram.Clear();
		FlatNode root = ResultNode();
		itemOutputs.clear();
		if (root.node != nullptr) //not a constant result
		{
			flatProgram.Build({ root });
			for (Index k = 0; k < flatProgram.NumberOfOperations(); k++) //the outputs of items (#2984)
			{
				const FlatNode& node = flatProgram.OperationNode(k);
				const VectorExpressionItemOutput* output = node.kind == FlatKind::Vector ?
					dynamic_cast<const VectorExpressionItemOutput*>((const VectorExpressionBase*)node.node) : nullptr;
				if (output) { itemOutputs.push_back(output); }
			}
			if (flatProgram.NumberOfComputations() == 0) { flatProgram.Clear(); } //a leaf: nothing to sort
		}
		symbolicProgram.Clear();
		if (flatProgram.IsBuilt()) { symbolicProgram.Build(flatProgram); }
		copyable = ThreadCopiesPossible();
	}
	//! true, if the result is evaluated by the flat program
	bool UseFlatProgram() const { return flatEvaluation && flatProgram.IsBuilt(); }

	//! the leaf of argument i that an evaluation writes: of the recording, or of the copy of its thread; nullptr if the
	//! copy does not use the argument (#2978)
	ExpressionNamedReal* ArgumentReal(ThreadCopy* copy, Index i)
	{
		if (copy) { return (ExpressionNamedReal*)copy->arguments[i].node; }
		SReal* real = argRealList[i].GetReal();
		CHECKandTHROW(real->IsExpressionNamedReal(), "SReal::SetExpressionNamedReal expects ExpressionNamedReal in expression");
		return &real->GetExpressionNamedReal();
	}
	VectorExpressionNamedReal* ArgumentVector(ThreadCopy* copy, Index i)
	{
		if (copy) { return (VectorExpressionNamedReal*)copy->arguments[i].node; }
		return &argRealList[i].GetVector()->GetExpressionNamedReal();
	}
	MatrixExpressionNamedReal* ArgumentMatrix(ThreadCopy* copy, Index i)
	{
		if (copy) { return (MatrixExpressionNamedReal*)copy->arguments[i].node; }
		return &argRealList[i].GetMatrix()->GetExpressionNamedReal();
	}

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
		BuildFlatProgram();
	}
	void SetVectorType(Index size, SymbolicRealVector* sVector)
	{
		//scalarType = false;
		//vectorSize = size;
		returnValue.Set(sVector);
		BuildFlatProgram();
	}
	//! a matrix result (#3020); built from components with constant zeros, it is sparse with the pattern of the others (#3022)
	void SetMatrixType(SymbolicRealMatrix* sMatrix)
	{
		returnValue.Set(sMatrix);
		resultComponents = dynamic_cast<const MatrixExpressionSReal*>(sMatrix->GetExpression());
		resultPattern.clear();
		if (resultComponents)
		{
			resultComponents->StructuralNonZeros(resultPattern);
			if ((Index)resultPattern.size() == resultComponents->NumberOfRows()*resultComponents->NumberOfColumns()) { resultComponents = nullptr; }
		}
		BuildFlatProgram();
	}

	//! True (default): the result is evaluated by the flat program, if it has one; False: as a tree (#3053)
	void SetFlatEvaluation(bool flag)
	{
		if (!flag) { CheckTreeDepth(ResultNode(), "the tree evaluation (flatEvaluation = False)"); } //it recurses (#3090)
		flatEvaluation = flag;
	}
	bool GetFlatEvaluation() const { return flatEvaluation; }

	//! true, if it can be evaluated in the threads of the parallel loops: by its flat program, which can be copied (#2978)
	bool CanEvaluateInThreads() const { return copyable && (flatEvaluation || !flatProgram.IsBuilt()); }
	//! the copies of the recording for the threads 1 ... numberOfThreads-1 that are missing (#2978)
	void PrepareThreads(Index numberOfThreads);
	//! the number of copies of the recording for the threads
	Index NumberOfThreadCopies() const { return (Index)threadCopies.size(); }

	//! the flat program as text, one operation per line (#3053); flatten = false: the result as one expression (#3087)
	STDstring FlatProgramToString(bool flatten = true) const
	{
		if (!flatProgram.IsBuilt()) { return "#" + functionName + ": no flat program; the result is evaluated as a tree\n"; }
		return "#" + functionName + ": " + EXUstd::ToString(flatProgram.NumberOfComputations()) + " operations\n"
			+ flatProgram.ToString({ "result" }, flatten);
	}
	//! the recorded result, for GetFlatProgramData() (#3087)
	const SymbolicGeneric& GetReturnValue() const { return returnValue; }

	//! true, if the matrix result is sparse (#3022)
	bool HasSparseMatrixResult() const { return resultComponents != nullptr; }

	//! free allocated linked reals and SReal
	virtual ~SymbolicFunction()
	{
		Delete();
	}

	virtual void Delete()
	{
		DeleteThreadCopies(); //before the recording, whose leaves they share
		//done by destructors:
		for (SymbolicGeneric& linkedRealPtr : argRealList)
		{
			linkedRealPtr.Delete();
		}
		returnValue.Delete();
		argRealList.SetMaxNumberOfItems(0);
		flatProgram.Clear(); //it points to the nodes of the recording
		symbolicProgram.Clear();
	}

	virtual STDstring ToString() const
	{
		return returnValue.ToString();
	}

	//! the flat program that evaluates: of the copy of the thread, of the recording, or none for the tree and for a
	//! constant result, which a copy only reads (#2978)
	const FlatProgram* EvaluatingProgram(const ThreadCopy* copy) const
	{
		if (copy) { return copy->flatProgram.IsBuilt() ? &copy->flatProgram : nullptr; }
		return UseFlatProgram() ? &flatProgram : nullptr;
	}
	//! the program of the table for the flat program that evaluates (#3153)
	const SymbolicProgram& TableProgram(const ThreadCopy* copy) const { return copy ? copy->symbolicProgram : symbolicProgram; }

	//! the result; copy: the copy of the recording of the thread that evaluates, nullptr in the main thread (#2978)
	virtual Real EvaluateReturnValue(const ThreadCopy* copy = nullptr) const
	{
		if (EvaluatingProgram(copy)) { TableProgram(copy).Evaluate(); return TableProgram(copy).Result(0); }
		return returnValue.GetReal()->Evaluate();
	}

	virtual ResizableConstVector EvaluateReturnVector(const ThreadCopy* copy = nullptr) const
	{
		if (EvaluatingProgram(copy)) { return EvaluateReturnVectorRef(copy); }
		return returnValue.GetVector()->Evaluate();
	}

	//! the returned vector without a copy, valid until the function is evaluated again (#3006)
	const ResizableConstVector& EvaluateReturnVectorRef(const ThreadCopy* copy = nullptr) const
	{
		if (EvaluatingProgram(copy)) { TableProgram(copy).Evaluate(); return TableProgram(copy).VectorResult(0); }
		return returnValue.GetVector()->EvaluateRef();
	}

	virtual ResizableConstMatrix EvaluateReturnMatrix(const ThreadCopy* copy = nullptr) const
	{
		if (EvaluatingProgram(copy)) { return EvaluateReturnMatrixRef(copy); }
		return returnValue.GetMatrix()->Evaluate();
	}

	//! the returned matrix without a copy, valid until the function is evaluated again (#3021)
	const ResizableConstMatrix& EvaluateReturnMatrixRef(const ThreadCopy* copy = nullptr) const
	{
		if (EvaluatingProgram(copy)) { TableProgram(copy).Evaluate(); return TableProgram(copy).MatrixResult(0); }
		return returnValue.GetMatrix()->EvaluateRef();
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

	//! the dictionary of the recording: the arguments and their types, for GetFlatProgramData() (#3087)
	const py::dict& GetFunctionDict() const { return functionDict; }

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
			else if (ArgTypeIsMatrix(returnTypeStr) || returnTypeStr == "py::object") //py::object: a MatrixContainer result (#3020)
			{
				if (!py::isinstance<SymbolicRealMatrix>(pyReturnValue))
				{
					PyError(STDstring("SymbolicUserFunction: ") + userFunctionName + " of " + itemTypeName
						+ " returns a matrix: the recorded function must return an exudyn.symbolic.Matrix", PyErrorType::typeError);
				}
				SetMatrixType(py::cast<SymbolicRealMatrix*>(pyReturnValue));
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
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return EvaluateReturnValue(copy);
		//EvaluateReturnValues(); //evaluate symbolic expression trees

		//return returnValues[0];
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	bool EvaluateBool(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return bool(EvaluateReturnValue(copy));
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdVector2D EvaluateStdVector2D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return (StdVector2D)EvaluateReturnVectorRef(copy);
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdVector3D EvaluateStdVector3D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return (StdVector3D)EvaluateReturnVectorRef(copy);
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdVector6D EvaluateStdVector6D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return (StdVector6D)EvaluateReturnVectorRef(copy);
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	std::vector<Real> EvaluateStdVector(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return (StdVector)EvaluateReturnVectorRef(copy);
	}

	//! a user function that returns a vector, evaluated into result, which keeps its memory when it has the size;
	//! the arguments may be Exudyn vectors, which are copied into the named variables (#3006)
	template<typename... Args>
	void EvaluateIntoVector(Vector& result, const MainSystem& mainSystem, const Args&... args)
	{
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		result.CopyFrom(EvaluateReturnVectorRef(copy));
	}

	//! factor times the derivative of the vector result by the vector argument of index argument (after mbs: t is 0), at
	//! the arguments args, added to jacobian (rows: the result, columns: the argument); in one pass over the program of
	//! the table, also in the threads, for the analytic Jacobian of an item (#3189). The result must be a vector of
	//! jacobian.NumberOfRows() components and the argument a vector of jacobian.NumberOfColumns()
	template<typename... Args>
	void EvaluateJacobianAdd(ResizableMatrix& jacobian, Index argument, Real factor, const MainSystem& mainSystem, const Args&... args)
	{
		ThreadCopy* copy = CurrentThreadCopy();
		CHECKandTHROW(itemOutputs.empty(), "symbolic user function " + functionName + ": its Jacobian: the function reads outputs of items, which have no derivative");
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));
		const void* variable = copy ? copy->arguments[argument].node : (const void*)&argRealList[argument].GetVector()->GetExpressionNamedReal();
		const FlatNode root = copy ? copy->root : ResultNode();
		Index rows = jacobian.NumberOfRows();
		Index columns = jacobian.NumberOfColumns();
		if (variable == nullptr || root.node == nullptr) { return; } //the argument is not used, or the result is constant
		const FlatProgram& program = copy ? copy->flatProgram : flatProgram;
		if (!program.IsBuilt()) //the result is a leaf: the unit matrix if it is the argument
		{
			if (root.node == variable) { for (Index i = 0; i < EXUstd::Minimum(rows, columns); i++) { jacobian(i, i) += factor; } }
			return;
		}
		SymbolicProgram& table = copy ? copy->symbolicProgram : symbolicProgram;
		table.Evaluate();
		table.Differentiate(variable, columns);
		CHECKandTHROW(table.ResultComponents(0) == rows, "symbolic user function " + functionName + ": its Jacobian: the result has "
			+ EXUstd::ToString(table.ResultComponents(0)) + " components, but " + EXUstd::ToString(rows) + " are needed");
		const Real* derivative = table.ResultDerivative(0);
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < columns; j++) { jacobian(i, j) += factor*derivative[i*columns + j]; }
		}
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdMatrix3D EvaluateStdMatrix3D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return (StdMatrix3D)EvaluateReturnMatrix(copy);
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	StdMatrix6D EvaluateStdMatrix6D(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return (StdMatrix6D)EvaluateReturnMatrix(copy);
	}

	//! variadic template to evaluate all kinds of user functions (templates created/defined by Pybind interface)
	template<typename... Args>
	NumpyMatrix EvaluateNumpyMatrix(const MainSystem& mainSystem, const Args&... args)
	{
		// Process the rest of the arguments
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));

		return EPyUtils::ToPython(EvaluateReturnMatrix(copy)); //a numpy array of the rows and columns
	}

	//! a matrix result as Python object, for the std::function of a user function that returns a MatrixContainer (#3020)
	template<typename... Args>
	py::object EvaluatePyObject(const MainSystem& mainSystem, const Args&... args)
	{
		return EvaluateNumpyMatrix(mainSystem, args...);
	}

	//! a matrix result evaluated into result, which keeps its memory when it has the size (#3020)
	template<typename... Args>
	void EvaluateIntoMatrix(Matrix& result, const MainSystem& mainSystem, const Args&... args)
	{
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));
		result.CopyFrom(EvaluateReturnMatrixRef(copy));
	}

	//! a matrix result evaluated into a MatrixContainer as CopyOrAddTriplets does it with the result of a Python function:
	//! a dense result is copied, ltg is for the sparse results (#3020, #3022)
	template<typename... Args>
	void EvaluateIntoMatrixContainer(EXUmath::MatrixContainer& result, const ArrayIndex& ltg, const MainSystem& mainSystem, const Args&... args)
	{
		ThreadCopy* copy = CurrentThreadCopy();
		ItemOutputScope itemOutputScope(*this, mainSystem);
		Index argIndex = 0;
		(..., processArgument(args, copy, argIndex));
		if (resultComponents && ltg.NumberOfItems() != 0) //sparse: only the components that are not the constant 0 (#3022)
		{
			CHECKandTHROW(resultComponents->NumberOfRows() <= ltg.NumberOfItems() && resultComponents->NumberOfColumns() <= ltg.NumberOfItems(),
				"symbolic user function: the matrix result is larger than the coordinates of the item");
			result.SetUseDenseMatrix(false);
			ResizableArray<EXUmath::Triplet>& triplets = result.GetInternalSparseTripletMatrix().GetTriplets();
			if (EvaluatingProgram(copy)) //all components in one pass (#3053), in a thread by its copy (#2978)
			{
				const ResizableMatrix& matrix = EvaluateReturnMatrixRef(copy);
				for (const std::pair<Index, Index>& item : resultPattern)
				{
					triplets.AppendPure(EXUmath::Triplet(ltg[item.first], ltg[item.second], matrix(item.first, item.second)));
				}
				return;
			}
			for (const std::pair<Index, Index>& item : resultPattern)
			{
				triplets.AppendPure(EXUmath::Triplet(ltg[item.first], ltg[item.second], resultComponents->EvaluateComponent(item.first, item.second)));
			}
			return;
		}
		result.SetUseDenseMatrix(true);
		result.GetInternalDenseMatrix().CopyFrom(EvaluateReturnMatrixRef(copy));
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
		const MainSystem* mainSystem = nullptr; //for the outputs of items the recording reads (#2984)
		for (const auto& arg : args) {
			if (py::isinstance<MainSystem>(arg)) { mainSystem = &py::cast<const MainSystem&>(arg); continue; }
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

		CHECKandTHROW(mainSystem || itemOutputs.empty(), "symbolic user function " + functionName
			+ ": Evaluate needs the MainSystem as first argument, as the function reads outputs of items");
		std::unique_ptr<ItemOutputScope> itemOutputScope(mainSystem ? new ItemOutputScope(*this, *mainSystem) : nullptr);
		if (returnValue.IsReal())
		{
			return py::cast<Real>(EvaluateReturnValue());
		}
		else if (returnValue.IsMatrix()) //a numpy array (#3020)
		{
			return py::object(EPyUtils::ToPython(EvaluateReturnMatrix()));
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

	//! the Jacobian of the result by the argument of this name, at the arguments of the call (as for Evaluate): in one
	//! pass over the program of the table, all components of the argument at once (#3116, #3171): a Real result gives a
	//! vector, a vector result a matrix (size of result x size of argument), a matrix result an array (rows x columns x
	//! size of argument)
	py::object PyEvaluateJacobian(const STDstring& argumentName, const py::args& args)
	{
		py::object value = PyEvaluateUF(args); //sets the arguments: the values at which the derivative is taken
		const void* variable = nullptr;
		Index size = 0;
		for (SymbolicGeneric& argument : argRealList)
		{
			if (argument.IsReal() && argument.GetReal()->IsExpressionNamedReal() && argument.GetReal()->GetExpressionNamedReal().GetName() == argumentName)
			{
				variable = &argument.GetReal()->GetExpressionNamedReal(); size = -1;
			}
			else if (argument.IsVector() && argument.GetVector()->IsExpressionNamedReal() && argument.GetVector()->GetExpressionNamedReal().GetName() == argumentName)
			{
				variable = &argument.GetVector()->GetExpressionNamedReal(); size = argument.GetVector()->GetExpressionNamedReal().NumberOfItems();
			}
			else if (argument.IsMatrix() && argument.GetMatrix()->IsExpressionNamedReal() && argument.GetMatrix()->GetExpressionNamedReal().GetName() == argumentName)
			{
				const MatrixExpressionNamedReal& matrix = argument.GetMatrix()->GetExpressionNamedReal();
				variable = &matrix; size = matrix.NumberOfRows()*matrix.NumberOfColumns();
			}
		}
		CHECKandTHROW(variable != nullptr, "symbolic user function " + functionName + ": EvaluateJacobian: no argument named '" + argumentName + "'");
		CHECKandTHROW(itemOutputs.empty(), "symbolic user function " + functionName + ": EvaluateJacobian: the function reads outputs of items, which have no derivative");
		Index columns = (size < 0) ? 1 : size;
		FlatNode root = ResultNode();
		std::vector<Real> leafDerivative; //of a result without operations: the unit matrix if it is the variable, else zero
		const Real* onePass = nullptr; //the derivatives of the result, component c by component j at [c*columns + j]
		if (flatProgram.IsBuilt()) //by the table, also with flatEvaluation = False (#3153, #3171)
		{
			symbolicProgram.Evaluate();
			symbolicProgram.Differentiate(variable, columns);
			onePass = symbolicProgram.ResultDerivative(0);
		}
		else
		{
			Index components = returnValue.IsReal() ? 1 : (returnValue.IsVector() ? (Index)py::len(value)
				: (Index)(py::array_t<Real>::ensure(value).shape(0)*py::array_t<Real>::ensure(value).shape(1)));
			leafDerivative.assign(components*columns, 0.);
			if (root.node == variable) { for (Index c = 0; c < components; c++) { leafDerivative[c*columns + c] = 1.; } }
			onePass = leafDerivative.data();
		}
		if (returnValue.IsReal())
		{
			py::array_t<Real> jacobian(columns);
			for (Index j = 0; j < columns; j++) { jacobian.mutable_at(j) = onePass[j]; }
			return std::move(jacobian);
		}
		else if (returnValue.IsVector())
		{
			Index rows = (Index)py::len(value);
			py::array_t<Real> jacobian(std::vector<ptrdiff_t>{ rows, columns });
			for (Index i = 0; i < rows; i++) { for (Index j = 0; j < columns; j++) { jacobian.mutable_at(i, j) = onePass[i*columns + j]; } }
			return std::move(jacobian);
		}
		else
		{
			py::array_t<Real> valueArray = py::array_t<Real>::ensure(value);
			Index rows = (Index)valueArray.shape(0);
			Index matrixColumns = (Index)valueArray.shape(1);
			py::array_t<Real> jacobian(std::vector<ptrdiff_t>{ rows, matrixColumns, columns });
			for (Index i = 0; i < rows; i++)
			{
				for (Index k = 0; k < matrixColumns; k++)
				{
					for (Index j = 0; j < columns; j++) { jacobian.mutable_at(i, k, j) = onePass[(i*matrixColumns + k)*columns + j]; }
				}
			}
			return std::move(jacobian);
		}
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
	//the arguments, written into the leaves of the evaluation, of the recording or of the copy of the thread; an
	//argument the copy does not use has no leaf (#2978)
	// Function to process Real arguments
	void processArgument(Real arg, ThreadCopy* copy, Index& argIndex)
	{
		ExpressionNamedReal* leaf = ArgumentReal(copy, argIndex++);
		if (leaf) { leaf->SetValue(arg); }
	}
	//! an OutputVariableType as the Real of its value, as an Index is passed (#2989)
	void processArgument(OutputVariableType arg, ThreadCopy* copy, Index& argIndex)
	{
		processArgument((Real)(Index64)arg, copy, argIndex);
	}
	void processArgument(const ResizableConstVector& arg, ThreadCopy* copy, Index& argIndex)
	{
		VectorExpressionNamedReal* leaf = ArgumentVector(copy, argIndex++);
		if (leaf) { leaf->SetVector(arg); }
	}
	//! a vector argument, copied into the memory of the named vector, which keeps it: no allocation (#3006)
	void processArgument(const StdVector& arg, ThreadCopy* copy, Index& argIndex)
	{
		VectorExpressionNamedReal* leaf = ArgumentVector(copy, argIndex++);
		if (leaf) { leaf->SetVector(arg); }
	}
	//! a vector of fixed size, e.g. the Vector3D of a local position (#2995)
	template<Index n>
	void processArgument(const SlimVectorBase<Real, n>& arg, ThreadCopy* copy, Index& argIndex)
	{
		VectorExpressionNamedReal* leaf = ArgumentVector(copy, argIndex++);
		if (!leaf) { return; }
		ResizableConstVector& vector = leaf->GetVector();
		vector.SetNumberOfItems(n);
		std::copy(arg.begin(), arg.end(), vector.begin());
	}
	void processArgument(const VectorBase<Real>& arg, ThreadCopy* copy, Index& argIndex)
	{
		VectorExpressionNamedReal* leaf = ArgumentVector(copy, argIndex++);
		if (!leaf) { return; }
		ResizableConstVector& vector = leaf->GetVector();
		vector.SetNumberOfItems(arg.NumberOfItems());
		std::copy(arg.begin(), arg.end(), vector.begin());
	}
	//! a vector of fixed size, std::array, copied into the memory of the named vector in one pass (#3039)
	template<std::size_t n>
	void processArgument(const std::array<Real, n>& arg, ThreadCopy* copy, Index& argIndex)
	{
		VectorExpressionNamedReal* leaf = ArgumentVector(copy, argIndex++);
		if (!leaf) { return; }
		ResizableConstVector& vector = leaf->GetVector();
		if (vector.NumberOfItems() != (Index)n) { vector.SetNumberOfItems(n); }
		std::copy(arg.begin(), arg.end(), vector.begin());
	}

	//! a frame: its rotation into the named matrix, its translation into the named vector of the argument, the two
	//! leaves of a kinematics.HT in the recording (#3183)
	void processArgument(const HomogeneousTransformation& arg, ThreadCopy* copy, Index& argIndex)
	{
		MatrixExpressionNamedReal* leaf = ArgumentMatrix(copy, argIndex++);
		if (leaf)
		{
			ResizableConstMatrix& matrix = leaf->GetMatrix();
			if (matrix.NumberOfRows() != 3 || matrix.NumberOfColumns() != 3) { matrix.SetNumberOfRowsAndColumns(3, 3); }
			Real* data = matrix.GetDataPointer();
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++) { data[3*i + j] = arg.Rotation(i, j); }
			}
		}
		processArgument(arg.GetTranslation(), copy, argIndex);
	}
	//! a twist: its velocity and angular velocity into the two named vectors of a kinematics.Twist (#3183)
	void processArgument(const Twist& arg, ThreadCopy* copy, Index& argIndex)
	{
		processArgument(arg.velocity, copy, argIndex);
		processArgument(arg.angularVelocity, copy, argIndex);
	}

	void processArgument(const ResizableConstMatrix& arg, ThreadCopy* copy, Index& argIndex)
	{
		MatrixExpressionNamedReal* leaf = ArgumentMatrix(copy, argIndex++);
		if (leaf) { leaf->SetMatrix(arg); }
	}
	void processArgument(const StdMatrix3D & arg, ThreadCopy* copy, Index& argIndex)
	{
		processArgumentMatrix(arg, copy, argIndex);
	}
	void processArgument(const StdMatrix6D& arg, ThreadCopy* copy, Index& argIndex)
	{
		processArgumentMatrix(arg, copy, argIndex);
	}
	//! a matrix of fixed size, std::array of rows, copied into the memory of the named matrix in one pass, as both are
	//! stored by rows; the copy by components cost about a fifth of a call of RigidBodySpringDamper (#3039)
	template<std::size_t rows, std::size_t columns>
	void processArgumentMatrix(const std::array<std::array<Real, columns>, rows>& arg, ThreadCopy* copy, Index& argIndex)
	{
		static_assert(sizeof(arg) == rows*columns*sizeof(Real), "a std::array matrix must be contiguous");
		MatrixExpressionNamedReal* leaf = ArgumentMatrix(copy, argIndex++);
		if (!leaf) { return; }
		ResizableConstMatrix& matrix = leaf->GetMatrix();
		if (matrix.NumberOfRows() != (Index)rows || matrix.NumberOfColumns() != (Index)columns) { matrix.SetNumberOfRowsAndColumns(rows, columns); }
		std::copy(arg[0].data(), arg[0].data() + rows*columns, matrix.GetDataPointer());
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
	else if constexpr (std::is_same_v<T, HomogeneousTransformation>) { return py::cast(PyHT(argument)); } //exudyn.HT (#3183)
	else if constexpr (std::is_same_v<T, Twist>) { return py::cast(PyTwist(argument)); } //exudyn.Twist
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
