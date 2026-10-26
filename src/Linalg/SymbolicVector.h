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
#ifndef SYMBOLICVECTOR__H
#define SYMBOLICVECTOR__H

#include <initializer_list> //for initializer_list in constructor

//this header is built ON Symbolic.h and uses py::list, py::array_t and EPyUtils; it included
//neither, and compiled only because Symbolic.cpp includes both before it (#2480)
#include "Linalg/Symbolic.h"
#include "Pymodules/PybindUtilities.h"

extern bool linalgPrintUsePythonFormat;
class MainSystem; //VectorExpressionItemOutput (#2984)

namespace Symbolic
{

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef ResizableArray<ExpressionBase*> ExpressionList;
//typedef ResizableVector ResizableConstVector;



//! class which represents a vector of expressions
class VectorExpressionReal: public VectorExpressionBase
{
protected:
	ResizableConstVector vector;
public:
	VectorExpressionReal(const Vector& vectorInit): vector(vectorInit) 
	{
		IncreaseReferenceCounter(); //is referenced immediately, otherwise will lead to negative reference
	}
	virtual void Destroy() override {}

	virtual const ResizableConstVector& EvaluateRef() const override
	{
		return vector;
	}
	//! the derivative (#3115): the unit vector of the component for the seed of this vector, zero otherwise
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		diffBuffer.SetNumberOfItems(vector.NumberOfItems());
		diffBuffer.SetAll(0.);
		if (var.variable == (const void*)this)
		{
			CHECKandTHROW(var.component >= 0 && var.component < vector.NumberOfItems(), "symbolic Diff: the component of the vector variable is out of range");
			diffBuffer[var.component] = 1.;
		}
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		return vector.ToString();
	}
	virtual Index NumberOfItems() const { return vector.NumberOfItems(); }
	virtual void SetVector(const ResizableConstVector& vectorInit) { vector = vectorInit; }
	//! the value of an argument, copied into the memory of the vector, which is kept (#3006)
	virtual void SetVector(const std::vector<Real>& vectorInit)
	{
		vector.SetNumberOfItems((Index)vectorInit.size());
		std::copy(vectorInit.begin(), vectorInit.end(), vector.begin());
	}
	virtual void SetComponent(Index i, Real value) { vector[i] = value; }
	virtual const ResizableConstVector& GetVector() const { return vector; }
	virtual ResizableConstVector& GetVector() { return vector; }
};

class VectorExpressionNamedReal: public VectorExpressionReal
{
	STDstring name;
public:
	VectorExpressionNamedReal(const Vector& vectorInit, const STDstring& nameInit = ""): VectorExpressionReal(vectorInit), name(nameInit)
	{
		CHECKandTHROW(ExpressionBase::IsRegularVariableName(nameInit), "VectorExpressionNamedVector(vector, name): string must start with letter and contain only regular letters or numbers");
	}

	virtual STDstring ToString() const override
	{
		return name; //returns name of variable ...
	}

	virtual void SetName(const STDstring& nameInit) { name = nameInit; }
	virtual const STDstring& GetName() const { return name; }
};

//! the output of an item, read from the MainSystem at each evaluation (#2984): mbs.GetNodeOutput, GetObjectOutput,
//! GetObjectOutputBody, GetMarkerOutput and GetSensorValues in a function recorded by CreateSymbolicUserFunction; a
//! leaf; the symbolic function sets the MainSystem of its call, without one the value is the one at recording
class VectorExpressionItemOutput : public VectorExpressionBase
{
public:
	enum class Kind { Node, Object, ObjectBody, Marker, Sensor };
	Kind kind;
	Index itemNumber;
	Index64 variableType;		//!< an OutputVariableType
	Index configuration;		//!< a ConfigurationType
	Vector3D localPosition;		//!< of ObjectBody
	mutable const MainSystem* mainSystem = nullptr;

	VectorExpressionItemOutput(Kind kindInit, Index itemNumberInit, Index64 variableTypeInit, Index configurationInit,
		const Vector3D& localPositionInit, const Vector& valueAtRecording) : VectorExpressionBase(), kind(kindInit),
		itemNumber(itemNumberInit), variableType(variableTypeInit), configuration(configurationInit), localPosition(localPositionInit)
	{
		buffer = valueAtRecording;
	}
	virtual void Destroy() override {}
	virtual const ResizableConstVector& EvaluateRef() const override; //in Symbolic.cpp, which has the MainSystem
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		CHECKandTHROWstring("symbolic Diff: an output of an item read from the MainSystem has no derivative");
		return diffBuffer;
	}
	virtual STDstring ToString() const override;
};

//! class which represents a vector of expressions
//! in contrast to SReal, SymbolicRealVector stores a VectorExpressionBase*, thus needing VectorExpressionSReal to store a list of expressions
class VectorExpressionSReal: public VectorExpressionBase
{
protected:
	ExpressionList exprList; //list of expressions; if an item exists, it always refers to an expression
public:
	VectorExpressionSReal() : VectorExpressionBase() 
	{
		IncreaseReferenceCounter(); //immediately referenced ...
	}

	virtual ~VectorExpressionSReal()
	{
		Destroy();
	}
	//! build from initializer list, only using SReal
	VectorExpressionSReal(std::initializer_list<SReal> listOfSReals) : VectorExpressionBase()
	{
		IncreaseReferenceCounter();
		exprList.SetNumberOfItems((Index)listOfSReals.size());
		Index cnt = 0;
		for (const auto& sreal : listOfSReals) 
		{
			ExpressionBase* expr = sreal.GetExpression();
			ExpressionBase::NewCount() += (expr == 0);
			if (expr) { expr->IncreaseReferenceCounter(); }
			exprList[cnt] = expr ? expr : new ExpressionReal(sreal.GetValue());
			cnt++;
		}
	}
	VectorExpressionSReal(std::vector<SReal> listOfSReals) : VectorExpressionBase()
	{
		IncreaseReferenceCounter();
		exprList.SetNumberOfItems((Index)listOfSReals.size());
		Index cnt = 0;
		for (const auto& sreal : listOfSReals)
		{
			ExpressionBase* expr = sreal.GetExpression();
			ExpressionBase::NewCount() += (expr == 0);
			if (expr) { expr->IncreaseReferenceCounter(); }
			exprList[cnt] = expr ? expr : new ExpressionReal(sreal.GetValue());
			cnt++;
		}
	}
	void SetNumberOfItems(Index size)
	{
		Destroy(); //if previous list exists
		exprList.SetNumberOfItems(size);
	}
	//! set a particular component of vector; used in SymbolicRealVector py::list constructor
	void SetSReal(Index index, const SReal& value)
	{
		ExpressionBase* expr = value.GetExpression();
		ExpressionBase::NewCount() += (expr == 0);
		if (expr) { expr->IncreaseReferenceCounter(); }
		exprList[index] = expr ? expr : new ExpressionReal(value.GetValue());
	}
	virtual void Destroy() override
	{
		for (ExpressionBase* item : exprList)
		{
			item->DecreaseReferenceCounter();
			if (item->ReferenceCounter() == 0)
			{
				DeleteNode(item);
			}
		}
		//released: the destructor calls Destroy() again after SymbolicRealVector::Destroy() did, which freed them twice (#2876)
		exprList.SetNumberOfItems(0);
	}
	virtual const ResizableConstVector& EvaluateRef() const override
	{
		buffer.SetNumberOfItems(exprList.NumberOfItems());
		for (Index i = 0; i < exprList.NumberOfItems(); i++)
		{
			buffer[i] = exprList.GetItem(i)->Evaluate();
		}
		return buffer;
	}
	//the flat evaluation (#3054): the components are the operands
	virtual Index NumberOfOperands() const override { return exprList.NumberOfItems(); }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(exprList.GetItem(i)); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override
	{
		buffer.SetNumberOfItems(exprList.NumberOfItems());
		for (Index i = 0; i < exprList.NumberOfItems(); i++) { buffer[i] = values[i].real; }
		return buffer;
	}
	virtual const char* FlatName() const override { return "vector"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override
	{
		VectorExpressionSReal* components = new VectorExpressionSReal();
		components->DecreaseReferenceCounter(); //referenced by its users, as the other nodes (#3085)
		components->exprList.SetNumberOfItems(exprList.NumberOfItems());
		for (Index i = 0; i < exprList.NumberOfItems(); i++) { components->exprList[i] = (ExpressionBase*)operands[i].node; }
		return FlatNode(components);
	}
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override
	{
		STDstring str = "[";
		for (Index i = 0; i < exprList.NumberOfItems(); i++) { str += (i > 0 ? ", " : "") + operands[i]; }
		return str + "]";
	}
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		diffBuffer.SetNumberOfItems(exprList.NumberOfItems());
		for (Index i = 0; i < exprList.NumberOfItems(); i++) { diffBuffer[i] = exprList.GetItem(i)->Diff(var); }
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		STDstring str = "[";
		char sep = ' ';
		if (linalgPrintUsePythonFormat) { sep = ','; }
		for (Index i = 0; i < exprList.NumberOfItems(); i++)
		{
			if (i > 0) { str += sep; }
			str += exprList.GetItem(i)->ToString();
		}
		return str + "]";
	}

	//private: //should all not be called in derived class => put to private
	//	virtual Real Evaluate() const { return 0; };
	//	virtual STDstring ToString() const { return ""; }; 
	//	virtual void Destroy() {}; 
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class VectorExpressionOperatorPlus: public VectorExpressionBase
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorPlus(VectorExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override
	{
		const ResizableConstVector& leftExpr = left->EvaluateRef();
		const ResizableConstVector& rightExpr = right->EvaluateRef();
		CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(),
			"symbolic.Vector::operator+ : inconsistent vector sizes");
		buffer.SetNumberOfItems(leftExpr.NumberOfItems());
		for (Index i = 0; i < leftExpr.NumberOfItems(); i++) { buffer[i] = leftExpr[i] + rightExpr[i]; }
		return buffer;
	}
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		const ResizableConstVector& leftDiff = left->DiffRef(var);
		const ResizableConstVector& rightDiff = right->DiffRef(var);
		diffBuffer.SetNumberOfItems(leftDiff.NumberOfItems());
		for (Index i = 0; i < leftDiff.NumberOfItems(); i++) { diffBuffer[i] = leftDiff[i] + rightDiff[i]; }
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " + " + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { const ResizableConstVector& leftExpr = (*values[0].vector); const ResizableConstVector& rightExpr = (*values[1].vector); CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(), "symbolic.Vector::operator+ : inconsistent vector sizes"); buffer.SetNumberOfItems(leftExpr.NumberOfItems()); for (Index i = 0; i < leftExpr.NumberOfItems(); i++) { buffer[i] = leftExpr[i] + rightExpr[i]; } return buffer; }
	virtual const char* FlatName() const override { return "+"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorPlus((VectorExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " + " + operands[1] + ")"; }
};

class VectorExpressionOperatorMinus: public VectorExpressionBase
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorMinus(VectorExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override
	{
		const ResizableConstVector& leftExpr = left->EvaluateRef();
		const ResizableConstVector& rightExpr = right->EvaluateRef();
		CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(),
			"symbolic.Vector::operator- : inconsistent vector sizes");
		buffer.SetNumberOfItems(leftExpr.NumberOfItems());
		for (Index i = 0; i < leftExpr.NumberOfItems(); i++) { buffer[i] = leftExpr[i] - rightExpr[i]; }
		return buffer;
	}
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		const ResizableConstVector& leftDiff = left->DiffRef(var);
		const ResizableConstVector& rightDiff = right->DiffRef(var);
		diffBuffer.SetNumberOfItems(leftDiff.NumberOfItems());
		for (Index i = 0; i < leftDiff.NumberOfItems(); i++) { diffBuffer[i] = leftDiff[i] - rightDiff[i]; }
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " - " + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { const ResizableConstVector& leftExpr = (*values[0].vector); const ResizableConstVector& rightExpr = (*values[1].vector); CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(), "symbolic.Vector::operator- : inconsistent vector sizes"); buffer.SetNumberOfItems(leftExpr.NumberOfItems()); for (Index i = 0; i < leftExpr.NumberOfItems(); i++) { buffer[i] = leftExpr[i] - rightExpr[i]; } return buffer; }
	virtual const char* FlatName() const override { return "-"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorMinus((VectorExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " - " + operands[1] + ")"; }
};

class VectorExpressionOperatorMultScalarVector: public VectorExpressionBase
{
	ExpressionBase* left;
	VectorExpressionBase* right;
public:
	VectorExpressionOperatorMultScalarVector(ExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override
	{
		//always possible:
		Real factor = left->Evaluate();
		const ResizableConstVector& vector = right->EvaluateRef();
		buffer.SetNumberOfItems(vector.NumberOfItems());
		for (Index i = 0; i < vector.NumberOfItems(); i++) { buffer[i] = factor * vector[i]; }
		return buffer;
	}
	//! d(s v) = ds v + s dv
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		Real factor = left->Evaluate();
		Real factorDiff = left->Diff(var);
		const ResizableConstVector& vector = right->EvaluateRef();
		const ResizableConstVector& vectorDiff = right->DiffRef(var);
		diffBuffer.SetNumberOfItems(vector.NumberOfItems());
		for (Index i = 0; i < vector.NumberOfItems(); i++) { diffBuffer[i] = factorDiff * vector[i] + factor * vectorDiff[i]; }
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { Real factor = values[0].real; const ResizableConstVector& vector = (*values[1].vector); buffer.SetNumberOfItems(vector.NumberOfItems()); for (Index i = 0; i < vector.NumberOfItems(); i++) { buffer[i] = factor * vector[i]; } return buffer; }
	virtual const char* FlatName() const override { return "*"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorMultScalarVector((ExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " * " + operands[1] + ")"; }
};


class VectorExpressionOperatorMultMatrix : public VectorExpressionBase
{
	MatrixExpressionBase* left;
	VectorExpressionBase* right;
public:
	VectorExpressionOperatorMultMatrix(MatrixExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override
	{
		//CHECKandTHROW(left->NumberOfColumns() == right->NumberOfItems(),
		//	"symbolic.Vector::operator*(Matrix) : inconsistent matrix/vector sizes");
		EXUmath::MultMatrixVector(left->EvaluateRef(), right->EvaluateRef(), buffer);
		return buffer;
	}

	//! d(A v) = dA v + A dv
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		const ResizableMatrix& matrix = left->EvaluateRef();
		const ResizableMatrix& matrixDiff = left->DiffRef(var);
		const ResizableConstVector& vector = right->EvaluateRef();
		const ResizableConstVector& vectorDiff = right->DiffRef(var);
		CHECKandTHROW(matrix.NumberOfColumns() == vector.NumberOfItems(), "symbolic Diff of matrix @ vector: inconsistent sizes");
		diffBuffer.SetNumberOfItems(matrix.NumberOfRows());
		for (Index i = 0; i < matrix.NumberOfRows(); i++)
		{
			Real value = 0.;
			for (Index j = 0; j < matrix.NumberOfColumns(); j++) { value += matrixDiff(i, j) * vector[j] + matrix(i, j) * vectorDiff[j]; }
			diffBuffer[i] = value;
		}
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " @ " + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { EXUmath::MultMatrixVector((*values[0].matrix), (*values[1].vector), buffer); return buffer; }
	virtual const char* FlatName() const override { return "@"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorMultMatrix((MatrixExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " @ " + operands[1] + ")"; }
};

class VectorExpressionOperatorMultMatrixTransposed : public VectorExpressionBase
{
	MatrixExpressionBase* left;
	VectorExpressionBase* right;
public:
	VectorExpressionOperatorMultMatrixTransposed(MatrixExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override
	{
		//CHECKandTHROW(left->NumberOfColumns() == right->NumberOfItems(),
		//	"symbolic.Vector::operator*(Matrix) : inconsistent matrix/vector sizes");
		EXUmath::MultMatrixTransposedVector(left->EvaluateRef(), right->EvaluateRef(), buffer);
		return buffer;
	}

	//! d(A^T v) = dA^T v + A^T dv
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		const ResizableMatrix& matrix = left->EvaluateRef();
		const ResizableMatrix& matrixDiff = left->DiffRef(var);
		const ResizableConstVector& vector = right->EvaluateRef();
		const ResizableConstVector& vectorDiff = right->DiffRef(var);
		CHECKandTHROW(matrix.NumberOfRows() == vector.NumberOfItems(), "symbolic Diff of vector @ matrix: inconsistent sizes");
		diffBuffer.SetNumberOfItems(matrix.NumberOfColumns());
		for (Index j = 0; j < matrix.NumberOfColumns(); j++)
		{
			Real value = 0.;
			for (Index i = 0; i < matrix.NumberOfRows(); i++) { value += matrixDiff(i, j) * vector[i] + matrix(i, j) * vectorDiff[i]; }
			diffBuffer[j] = value;
		}
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		return "(" + right->ToString() + " @ " + left->ToString() + ")"; //vector @ matrix
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { EXUmath::MultMatrixTransposedVector((*values[0].matrix), (*values[1].vector), buffer); return buffer; }
	virtual const char* FlatName() const override { return "vector@matrix"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorMultMatrixTransposed((MatrixExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[1] + " @ " + operands[0] + ")"; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class VectorExpressionUnaryPlus: public VectorExpressionBase {
	VectorExpressionBase* operand;
public:
	VectorExpressionUnaryPlus(VectorExpressionBase* op) : VectorExpressionBase(), operand(op) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override {
		return operand->EvaluateRef();
	}
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override { return operand->DiffRef(var); }
	virtual STDstring ToString() const override {
		return operand->ToString();
		//return "(+" + operand->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { return (*values[0].vector); }
	virtual const char* FlatName() const override { return "pos"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionUnaryPlus((VectorExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return operands[0]; }
};

class VectorExpressionUnaryMinus: public VectorExpressionBase {
	VectorExpressionBase* operand;
public:
	VectorExpressionUnaryMinus(VectorExpressionBase* op) : VectorExpressionBase(), operand(op) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override {
		const ResizableConstVector& vector = operand->EvaluateRef();
		buffer.SetNumberOfItems(vector.NumberOfItems());
		for (Index i = 0; i < vector.NumberOfItems(); i++) { buffer[i] = -vector[i]; }
		return buffer;
	}
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		const ResizableConstVector& vectorDiff = operand->DiffRef(var);
		diffBuffer.SetNumberOfItems(vectorDiff.NumberOfItems());
		for (Index i = 0; i < vectorDiff.NumberOfItems(); i++) { diffBuffer[i] = -vectorDiff[i]; }
		return diffBuffer;
	}
	virtual STDstring ToString() const override {
		return "(-" + operand->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { const ResizableConstVector& vector = (*values[0].vector); buffer.SetNumberOfItems(vector.NumberOfItems()); for (Index i = 0; i < vector.NumberOfItems(); i++) { buffer[i] = -vector[i]; } return buffer; }
	virtual const char* FlatName() const override { return "neg"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionUnaryMinus((VectorExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(-" + operands[0] + ")"; }
};


class VectorExpressionMultComponents : public VectorExpressionBase
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionMultComponents(VectorExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual const ResizableConstVector& EvaluateRef() const override
	{
		const ResizableConstVector& l = left->EvaluateRef();
		const ResizableConstVector& r = right->EvaluateRef();

		CHECKandTHROW(l.NumberOfItems() == r.NumberOfItems(),
			"symbolic.MultComponents(...): inconsistent vector sizes");

		buffer.SetNumberOfItems(l.NumberOfItems());
		EXUmath::MultVectorComponents(l, r, buffer);
		return buffer;
	}

	//! d(u_i v_i) = du_i v_i + u_i dv_i
	virtual const ResizableConstVector& DiffRef(const DiffSeed& var) const override
	{
		const ResizableConstVector& l = left->EvaluateRef();
		const ResizableConstVector& r = right->EvaluateRef();
		const ResizableConstVector& lDiff = left->DiffRef(var);
		const ResizableConstVector& rDiff = right->DiffRef(var);
		diffBuffer.SetNumberOfItems(l.NumberOfItems());
		for (Index i = 0; i < l.NumberOfItems(); i++) { diffBuffer[i] = lDiff[i] * r[i] + l[i] * rDiff[i]; }
		return diffBuffer;
	}
	virtual STDstring ToString() const override
	{
		return left->ToString() + ".multiply(" + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const override { const ResizableConstVector& l = (*values[0].vector); const ResizableConstVector& r = (*values[1].vector); CHECKandTHROW(l.NumberOfItems() == r.NumberOfItems(), "symbolic.MultComponents(...): inconsistent vector sizes"); buffer.SetNumberOfItems(l.NumberOfItems()); EXUmath::MultVectorComponents(l, r, buffer); return buffer; }
	virtual const char* FlatName() const override { return "multiply"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionMultComponents((VectorExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return operands[0] + ".multiply(" + operands[1] + ")"; }
};





//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class VectorExpressionOperatorMultVectorVector: public ExpressionBase
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorMultVectorVector(VectorExpressionBase* l, VectorExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override
	{
		const ResizableConstVector& leftExpr = left->EvaluateRef();
		const ResizableConstVector& rightExpr = right->EvaluateRef();
		CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(),
			"symbolic.Vector::operator* (scalar vector multiplication) : inconsistent vector sizes");

		return leftExpr * rightExpr;

		//return left->Evaluate() * right->Evaluate();
	}
	//! d(u . v) = du . v + u . dv (#3115)
	virtual Real Diff(const DiffSeed& var) const override
	{
		const ResizableConstVector& l = left->EvaluateRef();
		const ResizableConstVector& r = right->EvaluateRef();
		const ResizableConstVector& lDiff = left->DiffRef(var);
		const ResizableConstVector& rDiff = right->DiffRef(var);
		CHECKandTHROW(l.NumberOfItems() == r.NumberOfItems(), "symbolic Diff of vector @ vector: inconsistent sizes");
		Real value = 0.;
		for (Index i = 0; i < l.NumberOfItems(); i++) { value += lDiff[i] * r[i] + l[i] * rDiff[i]; }
		return value;
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " @ " + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { const ResizableConstVector& leftExpr = (*values[0].vector); const ResizableConstVector& rightExpr = (*values[1].vector); CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(), "symbolic.Vector::operator* (scalar vector multiplication) : inconsistent vector sizes"); return leftExpr * rightExpr; }
	virtual const char* FlatName() const override { return "@"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorMultVectorVector((VectorExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " @ " + operands[1] + ")"; }
};

class VectorExpressionNormL2: public ExpressionBase
{
	VectorExpressionBase* operand;
public:
	VectorExpressionNormL2(VectorExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override 
	{ 
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } }
	}

	virtual Real Evaluate() const override 
	{ 
		return operand->EvaluateRef().GetL2Norm(); 
	}
	//! d|v| = (v . dv) / |v|, not defined at v = 0 (#3115)
	virtual Real Diff(const DiffSeed& var) const override
	{
		const ResizableConstVector& v = operand->EvaluateRef();
		const ResizableConstVector& vDiff = operand->DiffRef(var);
		Real norm = v.GetL2Norm();
		CHECKandTHROW(norm != 0., "symbolic Diff of NormL2: the derivative of the norm of a zero vector is not defined");
		Real value = 0.;
		for (Index i = 0; i < v.NumberOfItems(); i++) { value += v[i] * vDiff[i]; }
		return value / norm;
	}
	virtual STDstring ToString() const override { return "NormL2(" + operand->ToString() + ")"; }

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return (*values[0].vector).GetL2Norm(); }
	virtual const char* FlatName() const override { return "NormL2"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionNormL2((VectorExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "NormL2(" + operands[0] + ")"; }
};


class VectorExpressionOperatorBracket: public ExpressionBase
{
	VectorExpressionBase* operand;
	ExpressionBase* index;
public:
	VectorExpressionOperatorBracket(VectorExpressionBase* op, ExpressionBase* i) : 
		ExpressionBase(), operand(op), index(i) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } }
		if (index) { index->DecreaseReferenceCounter(); if (index->ReferenceCounter() == 0) { DeleteNode(index); } }
	}

	virtual Real Evaluate() const override
	{
		Real iReal = index->Evaluate();
		Index i = (Index)iReal;
		CHECKandTHROW((Real)i == iReal, "VectorExpressionBase::operator[] index must be integer");
		const ResizableConstVector& v = operand->EvaluateRef();
		//while a user function is recorded, its vector arguments have no size yet: the component has no value then (#3007)
		if (SReal::recordExpressions && v.NumberOfItems() == 0 && i >= 0) { return 0.; }
		CHECKandTHROW((i >= 0) && (i < v.NumberOfItems()), "VectorExpressionBase::operator[] index out of range");
		return v[i];
	}
	//! the component of the derivative of the vector (#3115)
	virtual Real Diff(const DiffSeed& var) const override
	{
		Real iReal = index->Evaluate();
		Index i = (Index)iReal;
		CHECKandTHROW((Real)i == iReal, "VectorExpressionBase::operator[] index must be integer");
		const ResizableConstVector& vectorDiff = operand->DiffRef(var);
		CHECKandTHROW((i >= 0) && (i < vectorDiff.NumberOfItems()), "VectorExpressionBase::operator[] index out of range");
		return vectorDiff[i];
	}
	virtual STDstring ToString() const override
	{
		return operand->ToString() + "[" + index->ToString() + "]";
	}

	virtual bool InlineInFlatText() const override { return true; } //a component, q[1]
	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(operand) : FlatNode(index); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { Real iReal = values[1].real; Index i = (Index)iReal; CHECKandTHROW((Real)i == iReal, "VectorExpressionBase::operator[] index must be integer"); const ResizableConstVector& v = (*values[0].vector); if (SReal::recordExpressions && v.NumberOfItems() == 0 && i >= 0) { return 0.; } CHECKandTHROW((i >= 0) && (i < v.NumberOfItems()), "VectorExpressionBase::operator[] index out of range"); return v[i]; }
	virtual const char* FlatName() const override { return "component"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorBracket((VectorExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return operands[0] + "[" + operands[1] + "]"; }
};

class VectorExpressionOperatorEQ: public ExpressionBase 
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorEQ(VectorExpressionBase* l, VectorExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override
	{
		return Real(left->EvaluateRef() == right->EvaluateRef());
	}
	virtual Real Diff(const DiffSeed& var) const override { CHECKandTHROWstring("Symbolic::operator==::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " == " + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real((*values[0].vector) == (*values[1].vector)); }
	virtual const char* FlatName() const override { return "=="; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorEQ((VectorExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " == " + operands[1] + ")"; }
};

class VectorExpressionOperatorNEQ: public ExpressionBase 
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorNEQ(VectorExpressionBase* l, VectorExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return Real(!(left->EvaluateRef() == right->EvaluateRef()));
	}
	virtual Real Diff(const DiffSeed& var) const override { CHECKandTHROWstring("Symbolic::operator!=::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " != " + right->ToString() + ")";
	}

	//the flat evaluation (#3054)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real(!((*values[0].vector) == (*values[1].vector))); }
	virtual const char* FlatName() const override { return "!="; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new VectorExpressionOperatorNEQ((VectorExpressionBase*)operands[0].node, (VectorExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " != " + operands[1] + ")"; }
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! symbolic vector, e.g., for use in pythonUserFunctions
class SymbolicRealVector
{
	VectorExpressionBase* exprList;
	ResizableConstVector vector;
public:
	SymbolicRealVector(): exprList(nullptr) {}
	//cast from Vector, but also from py::list as well as numpy array:
	SymbolicRealVector(const Vector& vectorInit) : exprList(nullptr), vector(vectorInit) {}
	//! constructor with value and name, gives subexpression; casts from std::vector<Real> as well
	SymbolicRealVector(const STDstring& name, const Vector& vectorInit) : exprList(nullptr), vector(vectorInit)
	{
		if (SReal::recordExpressions) {
			VectorExpressionBase::NewCount()++;
			exprList = new VectorExpressionNamedReal(vectorInit, name);
		}
	}

	//not needed:
	//SymbolicRealVector(const py::array_t<Real>& vectorInit) : exprList(nullptr)
	//{
	//	EPyUtils::NumPy2Vector(vectorInit, vector);
	//	pout << "v=" << vector << "\n";
	//} 
	SymbolicRealVector(py::list listInit)
	{
		bool isSymbolic = false;
		Index cnt = 0;
		for (auto item : listInit)
		{
			if (py::isinstance<SReal>(item))
			{
				isSymbolic = true;
				if (GetFlagDebug()) { pout << "SymbolicRealVector[" << cnt << "] is symbolic\n"; }
				cnt++;
			}
		}
		if (!isSymbolic || !SReal::recordExpressions)
		{
			exprList = nullptr;
			Index cnt = 0;
			vector.SetNumberOfItems((Index)listInit.size());
			for (auto item : listInit)
			{
				vector[cnt++] = py::cast<Real>(item);
			}
		}
		else
		{
			VectorExpressionBase::NewCount()++;
			VectorExpressionSReal* vReal = new VectorExpressionSReal();
			exprList = vReal;
			vReal->SetNumberOfItems((Index)listInit.size());

			Index cnt = 0;
			for (auto item : listInit)
			{
				if (py::isinstance<SReal>(item))
				{
					vReal->SetSReal(cnt, py::cast<SReal>(item) );
				}
				else
				{
					Real value = py::cast<Real>(item);
					vReal->SetSReal(cnt, SReal(value));
				}
				cnt++;
			}
		}

	} 
	//! take ownership of an expression; its value from the recorded values of its operands (#3090)
	SymbolicRealVector(VectorExpressionBase* e) : exprList(e)
	{
		if (e)
		{
			e->IncreaseReferenceCounter();
			try { vector = RecordedVector(e); }
			catch (...) //as SReal: this object never completes, so the expression is released here (#2481)
			{
				exprList = nullptr;
				e->DecreaseReferenceCounter();
				if (e->ReferenceCounter() == 0) { DeleteNode(e); }
				throw;
			}
		}
	}
	SymbolicRealVector(const SymbolicRealVector& other) : exprList(other.exprList), vector(other.vector)
	{
		if (GetFlagDebug()) { std::cout << "copy constructor: " << ToString() << "\n"; }
		if (exprList) { exprList->IncreaseReferenceCounter(); }
	}

	void SetSRealVector(std::initializer_list<SReal> listOfSReals)
	{
		Destroy(); //if old tree exists

		if (!SReal::recordExpressions) {
			vector.SetNumberOfItems((Index)listOfSReals.size());
			Index cnt = 0;
			for (const auto& value : listOfSReals)
			{
				vector[cnt] = value.Evaluate();
				cnt++;
			}
		}
		else
		{
			VectorExpressionBase::NewCount()++;
			exprList = new VectorExpressionSReal(listOfSReals);
		}
	}

	//! check if there is a ExpressionNamedVector in expr 
	bool IsExpressionNamedReal()  const
	{
		if (exprList == nullptr) { return false; }

		return (typeid(*exprList) == typeid(VectorExpressionNamedReal));
	}
	VectorExpressionNamedReal& GetExpressionNamedReal() 
	{ 
		CHECKandTHROW(IsExpressionNamedReal(), "SymbolicRealVector::GetExpressionNamedVector expects VectorExpressionNamedReal in expression");
		return (VectorExpressionNamedReal&)(*exprList);
	}
	const VectorExpressionNamedReal& GetExpressionNamedReal() const 
	{ 
		CHECKandTHROW(IsExpressionNamedReal(), "SymbolicRealVector::GetExpressionNamedVector (const) expects VectorExpressionNamedReal in expression");
		return (const VectorExpressionNamedReal&)(*exprList);
	}

	//! assume that expression e holds a NamedReal and override its value
	void SetExpressionNamedVector(const ResizableConstVector& vectorInit)
	{
		CHECKandTHROW(IsExpressionNamedReal(), "SymbolicRealVector::SetExpressionNamedVector expects VectorExpressionNamedReal in expression");
		GetExpressionNamedReal().SetVector(vectorInit);
	}

	//! set with numpy array or list
	void SetSymbolicVector(const std::vector<Real>& vectorInit)
	{
		if (IsExpressionNamedReal())
		{
			SReal::CheckNotRecordingFunction();
			GetExpressionNamedReal().SetVector(ResizableConstVector(vectorInit));
			vector = ResizableConstVector(vectorInit); //for consistency
		}
		else
		{
			CHECKandTHROW(exprList == nullptr, "SymbolicRealVector::SetValue can only be called for symbolic Real variables");
			vector = ResizableConstVector(vectorInit);
		}
	}

	//! assume that expression e holds a NamedReal and override its value
	void SetSymbolicVectorComponent(Index index, Real value)
	{
		if (IsExpressionNamedReal())
		{
			SReal::CheckNotRecordingFunction();
			CHECKandTHROW(index < GetExpressionNamedReal().NumberOfItems(),
				"SymbolicRealVector::SetExpressionNamedVectorComponent: index out of range");
			GetExpressionNamedReal().SetComponent(index, value);
		}
		else
		{
			CHECKandTHROW(exprList == nullptr, "SymbolicRealVector::SetVector can only be called for symbolic Vector variables");
			CHECKandTHROW(index < vector.NumberOfItems(),
				"SymbolicRealVector::SetExpressionNamedVectorComponent: index out of range");
			vector[index] = value;
		}
	}

	//virtual void SetComponent(Index i, Real value) { vector[i] = value; }

	static void SetFlagDebug(bool flag) { SReal::flagDebug = flag; }
	static bool GetFlagDebug() { return SReal::flagDebug; }
	void SetVector(const ResizableConstVector& vectorInit) { vector = vectorInit; }
	const ResizableConstVector& GetVector() const { return vector; }
	ResizableConstVector& GetVector() { return vector; }

	void SetExpression(VectorExpressionBase* e) { exprList = e; if (e) { e->IncreaseReferenceCounter(); } }
	VectorExpressionBase* GetExpression() const { return exprList; }

	//! for C++ usage: return Vector
	virtual ResizableConstVector Evaluate() const {
		if (exprList) { return exprList->Evaluate(); }
		return vector;
	}
	//! the value without a copy, valid until the expression is evaluated again (#3006)
	const ResizableConstVector& EvaluateRef() const {
		if (exprList) { return exprList->EvaluateRef(); }
		return vector;
	}

	//! for Python usage: return numpy array:
	//! 
	virtual py::array_t<Real> PyEvaluate() const {
		if (exprList) { return EPyUtils::ToPython(exprList->Evaluate()); }
		return EPyUtils::ToPython(vector);
	}

	virtual Index NumberOfItems() const {
		if (exprList) { return RecordedVector(exprList).NumberOfItems(); }
		return vector.NumberOfItems();
	}

	virtual STDstring ToString() const {
		if (exprList) { return ExpressionText(FlatNode(exprList)); }
		return vector.ToString();
	}

	virtual ~SymbolicRealVector()
	{
		Destroy();
	}
	//! destroy expression tree, before overriding by new tree
	void Destroy()
	{
		if (exprList)
		{
			exprList->DecreaseReferenceCounter();
			if (exprList->ReferenceCounter() == 0)
			{
				if (GetFlagDebug()) { std::cout << "now we delete: " << ToString() << "\n"; }
				DeleteNode(exprList);
			}
		}
	}

	// Modified implicit conversion to ResizableConstVector
	explicit operator ResizableConstVector() const
	{
		return Evaluate();
	}

	// Modified implicit conversion to Vector
	explicit operator Vector() const
	{
		return Evaluate();
	}

	//virtual STDstring ToString() const {
	//	if (exprList.NumberOfItems())
	//	{
	//		STDstring s = "[";
	//		STDstring sep = "";
	//		for (Index i = 0; i < exprList.NumberOfItems(); i++)
	//		{
	//			s += sep + exprList[i]->ToString();
	//		}
	//		s += "]";
	//		return s;
	//	}

	//	return vector.ToString();
	//}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	SymbolicRealVector& operator=(const SymbolicRealVector& other)
	{
		if (this != &other) // Protect against self-assignment
		{
			if (exprList) //if expression is not used any more, destroy!
			{
				exprList->DecreaseReferenceCounter();
				if (exprList->ReferenceCounter() == 0) { DeleteNode(exprList); }
			}

			vector = other.vector;
			exprList = other.exprList;
			if (exprList) { exprList->IncreaseReferenceCounter(); }

			if (GetFlagDebug()) { std::cout << "**********\nassign vector\n**********\n"; }
		}
		return *this;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


private:
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//add some helper functions to automatically treat Real and SReal in templates

	static Real Number2Real(Real x)
	{
		return x;
	}
	static Real Number2Real(Index x)
	{
		return (Real)x;
	}
	static Real Number2Real(SReal x)
	{
		return x.GetValue();
	}

	//! Get Expression* either for SymbolicRealVector or Vector
	static VectorExpressionBase* GetFunctionExpression(const SymbolicRealVector& x, bool increaseReferenceCounter = true)
	{
		VectorExpressionBase::NewCount() += (x.exprList == 0);
		if (x.exprList && increaseReferenceCounter) { x.exprList->IncreaseReferenceCounter(); }
		return x.exprList ? x.exprList : new VectorExpressionReal(x.vector);
	}
	static VectorExpressionBase* GetFunctionExpression(const Vector& x, bool increaseReferenceCounter = true)
	{
		VectorExpressionBase::NewCount() += 1;
		return new VectorExpressionReal(x);
	}

	//for SReal * Vector operations ...
	//! Get Expression* either for SReal or Real
	static ExpressionBase* GetFunctionExpressionSReal(const SReal& x)
	{
		ExpressionBase::NewCount() += (x.GetExpression() == 0);
		if (x.GetExpression()) { x.GetExpression()->IncreaseReferenceCounter(); }
		return x.GetExpression() ? x.GetExpression() : new ExpressionReal(x.GetValue() );
	}
	static ExpressionBase* GetFunctionExpressionSReal(Real x)
	{
		ExpressionBase::NewCount() += 1;
		return new ExpressionReal(x);
	}
public:
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	friend SymbolicRealVector operator+(const SymbolicRealVector& left, const SymbolicRealVector& right)
	{
		if (!SReal::recordExpressions) {
			return left.vector + right.vector;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionOperatorPlus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SymbolicRealVector operator-(const SymbolicRealVector& left, const SymbolicRealVector& right)
	{
		if (!SReal::recordExpressions) {
			return left.vector - right.vector;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionOperatorMinus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SymbolicRealVector operator*(const SReal& left, const SymbolicRealVector& right)
	{
		if (!SReal::recordExpressions) {
			return left.GetValue() * right.vector;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionOperatorMultScalarVector(GetFunctionExpressionSReal(left), GetFunctionExpression(right));
	}

	friend SymbolicRealVector operator*(const SymbolicRealVector& left, const SReal& right)
	{
		if (!SReal::recordExpressions) {
			return left.vector * right.GetValue();
		}
		VectorExpressionBase::NewCount()++;
		//NOTE: change of order!
		return new VectorExpressionOperatorMultScalarVector(GetFunctionExpressionSReal(right), GetFunctionExpression(left));
	}

	
	SymbolicRealVector& operator+=(const SymbolicRealVector& rhs) {
		if (!SReal::recordExpressions) {
			vector += rhs.vector;
			return *this;
		}
		VectorExpressionBase::NewCount() += 1;
		exprList = new VectorExpressionOperatorPlus(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		exprList->IncreaseReferenceCounter();
		return *this;
	}
	
	SymbolicRealVector& operator-=(const SymbolicRealVector& rhs) {
		if (!SReal::recordExpressions) {
			vector -= rhs.vector;
			return *this;
		}
		VectorExpressionBase::NewCount() += 1;
		exprList = new VectorExpressionOperatorMinus(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		exprList->IncreaseReferenceCounter();
		return *this;
	}

	SymbolicRealVector& operator*=(const SReal& rhs) {
		if (!SReal::recordExpressions) {
			vector *= rhs.GetValue();
			return *this;
		}
		VectorExpressionBase::NewCount() += 1;
		exprList = new VectorExpressionOperatorMultScalarVector(GetFunctionExpressionSReal(rhs), GetFunctionExpression(*this, false));
		exprList->IncreaseReferenceCounter();
		return *this;
	}


	SymbolicRealVector operator-() const
	{
		if (!SReal::recordExpressions) {
			return -1.*vector;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionUnaryMinus(GetFunctionExpression(*this, true));
	}

	SymbolicRealVector operator+() const
	{
		if (!SReal::recordExpressions) {
			return vector;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionUnaryPlus(GetFunctionExpression(*this, true));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	friend SReal operator*(const SymbolicRealVector& left, const SymbolicRealVector& right)
	{
		if (!SReal::recordExpressions) {
			return left.vector * right.vector;
		}
		ExpressionBase::NewCount()++;
		return new VectorExpressionOperatorMultVectorVector(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	friend SReal operator==(const SymbolicRealVector& left, const SymbolicRealVector& right)
	{
		if (!SReal::recordExpressions) {
			return (Real)(left.vector == right.vector);
		}
		ExpressionBase::NewCount()++;
		return new VectorExpressionOperatorEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator!=(const SymbolicRealVector& left, const SymbolicRealVector& right)
	{
		if (!SReal::recordExpressions) {
			return (Real)(!(left.vector == right.vector));
		}
		ExpressionBase::NewCount()++;
		return new VectorExpressionOperatorNEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! component-wise multiplication, requiring special treatment
	SymbolicRealVector MultComponents(const SymbolicRealVector & right)
	{
		if (!SReal::recordExpressions) {
			ResizableConstVector result;
			EXUmath::MultVectorComponents(vector, right.vector, result);
			return result;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionMultComponents(GetFunctionExpression(*this, true), GetFunctionExpression(right));
	}

	SReal NormL2() const
	{
		if (!SReal::recordExpressions) {
			return vector.GetL2Norm();
		}
		ExpressionBase::NewCount()++;
		return new VectorExpressionNormL2(GetFunctionExpression(*this, true));
	}

	template<typename RealType>
	SReal operator[](const RealType& index) const
	{
		if (!SReal::recordExpressions) 
		{
			Real iReal = Number2Real(index);
			Index i = (Index)iReal;
			CHECKandTHROW((i >= 0) && (i < vector.NumberOfItems()), "SymbolicRealVector::operator[]: invalid index");
			CHECKandTHROW(iReal == (Real)i, "SymbolicRealVector::operator[]: index must be integer");
			return vector[i];
		}
		ExpressionBase::NewCount()++;
		return new VectorExpressionOperatorBracket(GetFunctionExpression(*this, true),
			GetFunctionExpressionSReal(index));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	friend std::ostream& operator<<(std::ostream& os, const SymbolicRealVector& thisExpr)
	{
		os << thisExpr.ToString();
		return os;
	}

};


























}; //namespace Symbolic

#endif //SYMBOLIC__H
