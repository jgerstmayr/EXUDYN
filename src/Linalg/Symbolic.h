/** ***********************************************************************************************
* @brief		implementation of tools for high-performant symbolic computation and expression trees
* @details		Details:
				- self-recording expression trees
				- record C++ code, imitating Real in the regular mode
				- fast evaluation of trees, potential for parallel computation
				- recording of Python math expressions and code for fast user functions
*
* @author		Gerstmayr Johannes
* @date			2023-11-22
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef SYMBOLIC__H
#define SYMBOLIC__H

//this header uses Real, Index, ResizableArray and STDstring, an unordered_map in VariableSet and
//typeid in SReal; it included none of them and compiled only because Symbolic.cpp includes them
//first (#2480)
#include "Linalg/BasicLinalg.h"
#include <unordered_map>    //VariableSet
#include <typeinfo>         //typeid, in SetExpressionNamedReal and IsExpressionNamedReal
#include <memory>           //shared_ptr, the data of a lookup table

namespace Symbolic
{
class ExpressionBase;
class VectorExpressionBase;
class MatrixExpressionBase;

//! the kinds of nodes of the flat evaluation (#3053, #3054)
enum class FlatKind { Real, Vector, Matrix };

//! a node of a recording of any kind, for the flat evaluation (#3054)
struct FlatNode
{
	FlatKind kind = FlatKind::Real;
	const void* node = nullptr;
	FlatNode() {}
	FlatNode(const ExpressionBase* e) : kind(FlatKind::Real), node(e) {}
	FlatNode(const VectorExpressionBase* e) : kind(FlatKind::Vector), node(e) {}
	FlatNode(const MatrixExpressionBase* e) : kind(FlatKind::Matrix), node(e) {}
};

//! the value of an operand in the flat evaluation: a number, or the vector or matrix its node evaluated into (#3054)
struct FlatValue
{
	Real real = 0.;
	const ResizableConstVector* vector = nullptr;
	const ResizableMatrix* matrix = nullptr;
};
class ExpressionNamedReal;
//! the derivatives of an expression by the n components of a named variable by the table of operations, in
//! SymbolicTable.cpp (#3171)
void OnePassDerivative(const FlatNode& root, const void* variable, Index n, std::vector<Real>& derivatives);



//! base class for expression tree
class ExpressionBase
{
protected:
	int referenceCounter;
public:
	static int newCount;
	static int& NewCount()
	{
		return newCount;
	}
	static int deleteCount;
	ExpressionBase() : referenceCounter(0) {}

	void IncreaseReferenceCounter() { referenceCounter++; }
	void DecreaseReferenceCounter() 
	{ 
		referenceCounter--; 
		//CHECKandTHROW(referenceCounter >= 0, "ExpressionBase::referenceCounter < 0");
	}
	int ReferenceCounter() const { return referenceCounter; }
	void SetReferenceCounter(Index value) { referenceCounter = value; }

	virtual ~ExpressionBase() = default;
	// Evaluate the expression tree
	virtual Real Evaluate() const = 0;
	virtual STDstring ToString() const = 0; // Convert the expression tree to a string

	//the flat evaluation (#3053): a node computes its value from the values of its operands, which the flat program has
	//computed before; a node without this interface - a leaf, IfThenElse, a component of a vector - has no operands for
	//the program and is evaluated as a tree
	virtual Index NumberOfOperands() const { return 0; }
	virtual FlatNode GetOperand(Index i) const { return FlatNode(); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const { return Evaluate(); }
	//! the name of the operation in GetFlatProgramData() (#3087); empty for a leaf
	virtual const char* FlatName() const { return ""; }
	//! a node of the same class with these operands, for Optimize (#3085); the operands are not referenced yet
	virtual FlatNode NewWithOperands(const FlatNode* operands) const { return FlatNode(); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const { return ToString(); }
	//! true for a component of a vector or matrix argument, q[1]: the text of the flat program writes it into the
	//! operations that use it, as a leaf, and does not count it
	virtual bool InlineInFlatText() const { return false; }
	//virtual ExpressionBase* clone() const = 0; // Pure virtual clone method
	virtual void Destroy() = 0; //Pure virtual destroy method, which deletes tree reversely

	//the value of an operation when it was recorded, from the recorded values of its operands, and the number of
	//operations on its longest path to a leaf; 0 until it is recorded (#3090, RecordNode)
	mutable Real recordedValue = 0.;
	mutable Index recordedDepth = 0;

	//variable must start with character, then may also contain numbers and '_'
	static bool IsRegularVariableName(const STDstring str)
	{
		return true;
		if (str.empty()) { return false; }

		if (!EXUstd::IsAlpha(str[0])) { return false; }

		for (char ch : str)
		{
			if (!EXUstd::IsAlphaNumeric(ch) && ch != '_') { return false; }
		}

		//if (!std::isalpha(str[0])) { return false; }

		//for (char ch : str) 
		//{
		//	if (!std::isalnum(ch) && ch != '_') { return false; }
		//}

		return true;
	}
};

//! class which represents a vector of expressions
class VectorExpressionBase
{
protected:
	int referenceCounter;
public:
	static int newCount;
	static int& NewCount()
	{
		return newCount;
	}
	static int deleteCount;

	VectorExpressionBase() : referenceCounter(0) {}
	virtual ~VectorExpressionBase() = default;

	void IncreaseReferenceCounter() { referenceCounter++; }
	void DecreaseReferenceCounter() 
	{ 
		referenceCounter--; 
		//CHECKandTHROW(referenceCounter >= 0, "VectorExpressionBase::referenceCounter < 0");
	}
	int ReferenceCounter() const { return referenceCounter; }
	void SetReferenceCounter(Index value) { referenceCounter = value; }
	virtual void Destroy() = 0;
	//! the value as a new vector
	virtual ResizableConstVector Evaluate() const { return EvaluateRef(); }
	//! the value, evaluated into the memory of this node (buffer), which is kept for the next evaluation: no allocation
	//! once the sizes are set; valid until the node is evaluated again (#3006)
	virtual const ResizableConstVector& EvaluateRef() const = 0;
	virtual STDstring ToString() const = 0;
	//the flat evaluation (#3054): as ExpressionBase; the value is the vector the node evaluated into
	virtual Index NumberOfOperands() const { return 0; }
	virtual FlatNode GetOperand(Index i) const { return FlatNode(); }
	virtual const ResizableConstVector& EvaluateFromOperands(const FlatValue* values) const { return EvaluateRef(); }
	//! the name of the operation in GetFlatProgramData() (#3087); empty for a leaf
	virtual const char* FlatName() const { return ""; }
	//! a node of the same class with these operands, for Optimize (#3085); the operands are not referenced yet
	virtual FlatNode NewWithOperands(const FlatNode* operands) const { return FlatNode(); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const { return ToString(); }
	//! as ExpressionBase (#3090): the vector of the recording, the buffer of this node or of an operand
	mutable const ResizableConstVector* recordedValue = nullptr;
	mutable Index recordedDepth = 0;
protected:
	mutable ResizableConstVector buffer; //!< the value of an operator node, see EvaluateRef()
};


//! class which represents a matrix of expressions
class MatrixExpressionBase
{
protected:
	int referenceCounter;
public:
	static int newCount;
	static int& NewCount()
	{
		return newCount;
	}
	static int deleteCount;

	MatrixExpressionBase() : referenceCounter(0) {}
	virtual ~MatrixExpressionBase() = default;

	void IncreaseReferenceCounter() { referenceCounter++; }
	void DecreaseReferenceCounter() 
	{ 
		referenceCounter--; 
		//CHECKandTHROW(referenceCounter >= 0, "MatrixExpressionBase::referenceCounter < 0");
	}
	int ReferenceCounter() const { return referenceCounter; }
	void SetReferenceCounter(Index value) { referenceCounter = value; }
	virtual void Destroy() = 0;
	//! the value as a new matrix
	virtual ResizableMatrix Evaluate() const { return EvaluateRef(); }
	//! the value, evaluated into the memory of this node (buffer), which is kept for the next evaluation: no allocation
	//! once the sizes are set; valid until the node is evaluated again (#3021, as #3006 for vectors)
	virtual const ResizableMatrix& EvaluateRef() const = 0;
	virtual Real EvaluateComponent(Index row, Index column) const = 0;
	virtual Index NumberOfRows() const = 0;
	virtual Index NumberOfColumns() const = 0;
	virtual STDstring ToString() const = 0;
	//the flat evaluation (#3054): as ExpressionBase; the value is the matrix the node evaluated into
	virtual Index NumberOfOperands() const { return 0; }
	virtual FlatNode GetOperand(Index i) const { return FlatNode(); }
	virtual const ResizableMatrix& EvaluateFromOperands(const FlatValue* values) const { return EvaluateRef(); }
	//! the name of the operation in GetFlatProgramData() (#3087); empty for a leaf
	virtual const char* FlatName() const { return ""; }
	//! a node of the same class with these operands, for Optimize (#3085); the operands are not referenced yet
	virtual FlatNode NewWithOperands(const FlatNode* operands) const { return FlatNode(); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const { return ToString(); }
	//! as ExpressionBase (#3090): the matrix of the recording, the buffer of this node or of an operand
	mutable const ResizableMatrix* recordedValue = nullptr;
	mutable Index recordedDepth = 0;
protected:
	mutable ResizableMatrix buffer; //!< the value of an operator node, see EvaluateRef()
};

//! delete a node whose reference counter reached zero: its Destroy() releases its operands, and those that reach
//! zero are deleted by the same loop instead of a recursion, so that a deep recording is deleted without a stack
//! overflow (#3090)
void DeleteNode(ExpressionBase* node);
void DeleteNode(VectorExpressionBase* node);
void DeleteNode(MatrixExpressionBase* node);

//! a new operation computes its value from the recorded values of its operands instead of evaluating the tree below
//! it: the recording is linear in its size and does not recurse; an operation not recorded yet - made by Optimize or
//! for a thread - is recorded with its operands, depth first without recursion; a leaf is not recorded, its value is
//! its present one (#3090)
void RecordNode(const FlatNode& node);
Real RecordedReal(const ExpressionBase* node);
const ResizableConstVector& RecordedVector(const VectorExpressionBase* node);
const ResizableMatrix& RecordedMatrix(const MatrixExpressionBase* node);
//! the number of operations on the longest path from a node to a leaf, 0 for a leaf (esym.GetDepth)
Index RecordedDepth(const FlatNode& node);
//! the tree evaluation, Evaluate() of an expression, recurses: deeper than this, it raises (#3090)
const Index maximumTreeDepth = 10000;
void CheckTreeDepth(const FlatNode& node, const char* what);
//! the text of str(), written by the flat program without recursion (#3090)
STDstring ExpressionText(const FlatNode& node);


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ExpressionOperatorPlus: public ExpressionBase
{
	ExpressionBase* left, * right;
public:
	ExpressionOperatorPlus(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}
	virtual Real Evaluate() const override
	{
		return left->Evaluate() + right->Evaluate();
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " + " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return values[0].real + values[1].real; }
	virtual const char* FlatName() const override { return "+"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorPlus((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " + " + operands[1] + ")"; }
};

class ExpressionOperatorMinus: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorMinus(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}
	virtual Real Evaluate() const override {
		return left->Evaluate() - right->Evaluate();
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " - " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return values[0].real - values[1].real; }
	virtual const char* FlatName() const override { return "-"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorMinus((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " - " + operands[1] + ")"; }
};

class ExpressionOperatorMul: public ExpressionBase
{
	ExpressionBase* left, * right;
public:
	ExpressionOperatorMul(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override
	{
		return left->Evaluate() * right->Evaluate();
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return values[0].real * values[1].real; }
	virtual const char* FlatName() const override { return "*"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorMul((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " * " + operands[1] + ")"; }
};

class ExpressionOperatorDiv: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorDiv(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return left->Evaluate() / right->Evaluate(); // Consider handling division by zero
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " / " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return values[0].real / values[1].real; }
	virtual const char* FlatName() const override { return "/"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorDiv((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " / " + operands[1] + ")"; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class ExpressionOperatorEQ: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorEQ(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() == right->Evaluate()); // Consider handling division by zero
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " == " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real(values[0].real == values[1].real); }
	virtual const char* FlatName() const override { return "=="; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorEQ((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " == " + operands[1] + ")"; }
};

class ExpressionOperatorNEQ: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorNEQ(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() != right->Evaluate()); // Consider handling division by zero
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " != " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real(values[0].real != values[1].real); }
	virtual const char* FlatName() const override { return "!="; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorNEQ((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " != " + operands[1] + ")"; }
};

class ExpressionOperatorGT: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorGT(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() > right->Evaluate()); // Consider handling division by zero
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " > " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real(values[0].real > values[1].real); }
	virtual const char* FlatName() const override { return ">"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorGT((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " > " + operands[1] + ")"; }
};

class ExpressionOperatorGE: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorGE(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() >= right->Evaluate()); // Consider handling division by zero
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " >= " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real(values[0].real >= values[1].real); }
	virtual const char* FlatName() const override { return ">="; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorGE((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " >= " + operands[1] + ")"; }
};

class ExpressionOperatorLE: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorLE(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() <= right->Evaluate()); // Consider handling division by zero
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " <= " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real(values[0].real <= values[1].real); }
	virtual const char* FlatName() const override { return "<="; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorLE((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " <= " + operands[1] + ")"; }
};

class ExpressionOperatorLT: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorLT(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { DeleteNode(left); } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { DeleteNode(right); } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() < right->Evaluate()); // Consider handling division by zero
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " < " + right->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(left) : FlatNode(right); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return Real(values[0].real < values[1].real); }
	virtual const char* FlatName() const override { return "<"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionOperatorLT((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(" + operands[0] + " < " + operands[1] + ")"; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ExpressionUnaryPlus: public ExpressionBase {
	ExpressionBase* operand;
public:
	ExpressionUnaryPlus(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } }
	}

	virtual Real Evaluate() const override {
		return operand->Evaluate();
	}
	virtual STDstring ToString() const override {
		return operand->ToString();
		//return "(+" + operand->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return values[0].real; }
	virtual const char* FlatName() const override { return "pos"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionUnaryPlus((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return operands[0]; }
};

class ExpressionUnaryMinus: public ExpressionBase {
	ExpressionBase* operand;
public:
	ExpressionUnaryMinus(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } }
	}

	virtual Real Evaluate() const override {
		return -operand->Evaluate();
	}
	virtual STDstring ToString() const override {
		return "(-" + operand->ToString() + ")";
	}

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return -values[0].real; }
	virtual const char* FlatName() const override { return "neg"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionUnaryMinus((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "(-" + operands[0] + ")"; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//functions

class ExpressionIsFinite: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionIsFinite(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::isfinite(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "isfinite(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::isfinite(values[0].real); }
	virtual const char* FlatName() const override { return "isfinite"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionIsFinite((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "isfinite(" + operands[0] + ")"; }
};

class ExpressionAbs: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionAbs(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::fabs(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "abs(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::fabs(values[0].real); }
	virtual const char* FlatName() const override { return "abs"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionAbs((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "abs(" + operands[0] + ")"; }
};

class ExpressionSign: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSign(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EXUstd::SignReal(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "sign(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EXUstd::SignReal(values[0].real); }
	virtual const char* FlatName() const override { return "sign"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionSign((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "sign(" + operands[0] + ")"; }
};


class ExpressionNot : public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionNot(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return !(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "Not(" + operand->ToString() + ")"; } //"not" is reserved in C++ and Python

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return !(values[0].real); }
	virtual const char* FlatName() const override { return "Not"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionNot((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "Not(" + operands[0] + ")"; }
};

class ExpressionRound: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionRound(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::round(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "round(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::round(values[0].real); }
	virtual const char* FlatName() const override { return "round"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionRound((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "round(" + operands[0] + ")"; }
};

class ExpressionCeil: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionCeil(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::ceil(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "ceil(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::ceil(values[0].real); }
	virtual const char* FlatName() const override { return "ceil"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionCeil((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "ceil(" + operands[0] + ")"; }
};

class ExpressionFloor: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionFloor(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::floor(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "floor(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::floor(values[0].real); }
	virtual const char* FlatName() const override { return "floor"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionFloor((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "floor(" + operands[0] + ")"; }
};


//+++++++++++++++++++++++++++++++++++++++++++++
class ExpressionSqrt: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSqrt(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::sqrt(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "sqrt(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::sqrt(values[0].real); }
	virtual const char* FlatName() const override { return "sqrt"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionSqrt((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "sqrt(" + operands[0] + ")"; }
};

class ExpressionExp: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionExp(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Exp(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "exp(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Exp(values[0].real); }
	virtual const char* FlatName() const override { return "exp"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionExp((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "exp(" + operands[0] + ")"; }
};

class ExpressionLog: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionLog(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Log(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "log(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Log(values[0].real); }
	virtual const char* FlatName() const override { return "log"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionLog((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "log(" + operands[0] + ")"; }
};

//+++++++++++++++++++++++++++++++++++++++++++++
class ExpressionSin: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSin(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Sin(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "sin(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Sin(values[0].real); }
	virtual const char* FlatName() const override { return "sin"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionSin((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "sin(" + operands[0] + ")"; }
};

class ExpressionCos: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionCos(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Cos(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "cos(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Cos(values[0].real); }
	virtual const char* FlatName() const override { return "cos"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionCos((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "cos(" + operands[0] + ")"; }
};

class ExpressionTan: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionTan(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Tan(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "tan(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Tan(values[0].real); }
	virtual const char* FlatName() const override { return "tan"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionTan((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "tan(" + operands[0] + ")"; }
};

class ExpressionASin: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionASin(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Asin(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "asin(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Asin(values[0].real); }
	virtual const char* FlatName() const override { return "asin"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionASin((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "asin(" + operands[0] + ")"; }
};

class ExpressionACos: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionACos(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Acos(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "acos(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Acos(values[0].real); }
	virtual const char* FlatName() const override { return "acos"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionACos((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "acos(" + operands[0] + ")"; }
};

class ExpressionATan: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionATan(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Atan(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "atan(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Atan(values[0].real); }
	virtual const char* FlatName() const override { return "atan"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionATan((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "atan(" + operands[0] + ")"; }
};


//+++++++++++++++++++++++++++++++++++++++++++++
class ExpressionSinh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSinh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Sinh(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "sinh(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Sinh(values[0].real); }
	virtual const char* FlatName() const override { return "sinh"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionSinh((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "sinh(" + operands[0] + ")"; }
};

class ExpressionCosh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionCosh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Cosh(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "cosh(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Cosh(values[0].real); }
	virtual const char* FlatName() const override { return "cosh"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionCosh((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "cosh(" + operands[0] + ")"; }
};

class ExpressionTanh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionTanh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return EM::Tanh(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "tanh(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Tanh(values[0].real); }
	virtual const char* FlatName() const override { return "tanh"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionTanh((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "tanh(" + operands[0] + ")"; }
};

class ExpressionASinh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionASinh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::asinh(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "asinh(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::asinh(values[0].real); }
	virtual const char* FlatName() const override { return "asinh"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionASinh((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "asinh(" + operands[0] + ")"; }
};

class ExpressionACosh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionACosh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::acosh(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "acosh(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::acosh(values[0].real); }
	virtual const char* FlatName() const override { return "acosh"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionACosh((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "acosh(" + operands[0] + ")"; }
};

class ExpressionATanh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionATanh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return std::atanh(operand->Evaluate()); }
	virtual STDstring ToString() const override { return "atanh(" + operand->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::atanh(values[0].real); }
	virtual const char* FlatName() const override { return "atanh"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionATanh((ExpressionBase*)operands[0].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "atanh(" + operands[0] + ")"; }
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//2-args functions

class ExpressionPower: public ExpressionBase {
	ExpressionBase* base, * exponent;
public:
	ExpressionPower(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: base(baseExpr), exponent(exponentExpr) {}

	virtual void Destroy() override
	{
		if (base) { base->DecreaseReferenceCounter(); if (base->ReferenceCounter() == 0) { DeleteNode(base); } }
		if (exponent) { exponent->DecreaseReferenceCounter(); if (exponent->ReferenceCounter() == 0) { DeleteNode(exponent); } }
	}

	virtual Real Evaluate() const override { return EM::Pow(base->Evaluate(), exponent->Evaluate()); }


	virtual STDstring ToString() const override { return "pow(" + base->ToString() + ", " + exponent->ToString() + ")"; }

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(base) : FlatNode(exponent); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Pow(values[0].real, values[1].real); }
	virtual const char* FlatName() const override { return "pow"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionPower((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "pow(" + operands[0] + ", " + operands[1] + ")"; }
};

class ExpressionAtan2: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionAtan2(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy() override
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { DeleteNode(arg1); } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { DeleteNode(arg2); } }
	}

	virtual Real Evaluate() const override { return EM::Atan2(arg1->Evaluate(), arg2->Evaluate()); }
	virtual STDstring ToString() const override { return "atan2(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(arg1) : FlatNode(arg2); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return EM::Atan2(values[0].real, values[1].real); }
	virtual const char* FlatName() const override { return "atan2"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionAtan2((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "atan2(" + operands[0] + ", " + operands[1] + ")"; }
};

class ExpressionMod: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionMod(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy() override
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { DeleteNode(arg1); } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { DeleteNode(arg2); } }
	}

	virtual Real Evaluate() const override { return std::fmod(arg1->Evaluate(), arg2->Evaluate()); }
	virtual STDstring ToString() const override { return "mod(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(arg1) : FlatNode(arg2); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::fmod(values[0].real, values[1].real); }
	virtual const char* FlatName() const override { return "mod"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionMod((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "mod(" + operands[0] + ", " + operands[1] + ")"; }
};

class ExpressionMin: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionMin(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy() override
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { DeleteNode(arg1); } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { DeleteNode(arg2); } }
	}

	virtual Real Evaluate() const override { return std::min(arg1->Evaluate(), arg2->Evaluate()); }
	virtual STDstring ToString() const override { return "min(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(arg1) : FlatNode(arg2); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::min(values[0].real, values[1].real); }
	virtual const char* FlatName() const override { return "min"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionMin((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "min(" + operands[0] + ", " + operands[1] + ")"; }
};

class ExpressionMax: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionMax(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy() override
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { DeleteNode(arg1); } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { DeleteNode(arg2); } }
	}

	virtual Real Evaluate() const override { return std::max(arg1->Evaluate(), arg2->Evaluate()); }
	virtual STDstring ToString() const override { return "max(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...

	//the flat evaluation (#3053)
	virtual Index NumberOfOperands() const override { return 2; }
	virtual FlatNode GetOperand(Index i) const override { return i == 0 ? FlatNode(arg1) : FlatNode(arg2); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return std::max(values[0].real, values[1].real); }
	virtual const char* FlatName() const override { return "max"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionMax((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "max(" + operands[0] + ", " + operands[1] + ")"; }
};




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//3-args functions

class ExpressionIfThenElse: public ExpressionBase
{
	ExpressionBase* condition;
	ExpressionBase* ifTrue;
	ExpressionBase* ifFalse;
public:
	ExpressionIfThenElse(ExpressionBase* conditionInit, 
		ExpressionBase* ifTrueInit, ExpressionBase* ifFalseInit) : ExpressionBase(),
		condition(conditionInit), ifTrue(ifTrueInit), ifFalse(ifFalseInit) {}

	virtual void Destroy() override
	{
		if (condition) { condition->DecreaseReferenceCounter(); if (condition->ReferenceCounter() == 0) { DeleteNode(condition); } }
		if (ifTrue) { ifTrue->DecreaseReferenceCounter(); if (ifTrue->ReferenceCounter() == 0) { DeleteNode(ifTrue); } }
		if (ifFalse) { ifFalse->DecreaseReferenceCounter(); if (ifFalse->ReferenceCounter() == 0) { DeleteNode(ifFalse); } }
	}

	virtual Real Evaluate() const override {
		Real checkCondition = condition->Evaluate();
		if (checkCondition)
		{
			return ifTrue->Evaluate();
		}
		else
		{
			return ifFalse->Evaluate();
		}
	}
	virtual STDstring ToString() const override
	{
		return "IfThenElse(" + condition->ToString() +"," + ifTrue->ToString() +"," + ifFalse->ToString() + ")";
	}
	//the flat evaluation (#3089): the condition and both branches are operands, computed before it, as a flat program
	//has no branches; the result is that of the tree, which computes only the branch that the condition selects
	virtual Index NumberOfOperands() const override { return 3; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(i == 0 ? condition : (i == 1 ? ifTrue : ifFalse)); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return values[0].real ? values[1].real : values[2].real; }
	virtual const char* FlatName() const override { return "IfThenElse"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionIfThenElse((ExpressionBase*)operands[0].node, (ExpressionBase*)operands[1].node, (ExpressionBase*)operands[2].node)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return "IfThenElse(" + operands[0] + "," + operands[1] + "," + operands[2] + ")"; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! the data of esym.LookupTable (#3175): values at equal steps from start, linearly interpolated; outside the
//! table zero, the end values (hold) or the table repeated with its length, the number of values times step
struct LookupTableData
{
	enum Outside { Zero = 0, Hold = 1, Periodic = 2 };
	std::vector<Real> values;
	Real step = 1.;
	Real start = 0.;
	Outside outside = Zero;

	//! the value at x, or with derivative its derivative, the slope of the interval of x
	Real Value(Real x, bool derivative) const
	{
		const Index n = (Index)values.size();
		if (n == 0) { return 0.; }
		Real u = (x - start)/step;
		if (outside == Periodic)
		{
			Real floorU = std::floor(u);
			Real fraction = u - floorU;
			Index i = (Index)(floorU - (Real)n*std::floor(floorU/(Real)n)); //floorU modulo n, also for negative u
			if (i >= n) { i = 0; } //rounding of the modulo
			Real a = values[i], b = values[(i + 1) % n];
			return derivative ? (b - a)/step : a + (b - a)*fraction;
		}
		if (u < 0. || u > (Real)(n - 1))
		{
			if (derivative || outside == Zero) { return 0.; }
			return u < 0. ? values[0] : values[n - 1];
		}
		if (n == 1) { return derivative ? 0. : values[0]; }
		Index i = EXUstd::Minimum((Index)std::floor(u), n - 2);
		Real a = values[i], b = values[i + 1];
		return derivative ? (b - a)/step : a + (b - a)*(u - (Real)i);
	}
	//! the name of the mode outside the table
	const char* OutsideName() const { return outside == Zero ? "zero" : (outside == Hold ? "hold" : "periodic"); }
};

//! esym.LookupTable (#3175): the data, stored once and shared by every lookup of the table
struct PyLookupTable
{
	std::shared_ptr<const LookupTableData> data;
	PyLookupTable(const std::vector<Real>& values, Real step, Real start, const STDstring& outside, const STDstring& interpolation)
	{
		CHECKandTHROW(values.size() != 0, "esym.LookupTable: values must have at least one value", ExudynValueError);
		CHECKandTHROW(step > 0., "esym.LookupTable: step must be positive, not " + EXUstd::ToString(step), ExudynValueError);
		CHECKandTHROW(interpolation == "linear", "esym.LookupTable: interpolation must be 'linear', not '" + interpolation
			+ "'; 'closest' and 'quadratic' are not available yet", ExudynValueError);
		LookupTableData* table = new LookupTableData();
		table->values = values;
		table->step = step;
		table->start = start;
		if (outside == "zero") { table->outside = LookupTableData::Zero; }
		else if (outside == "hold") { table->outside = LookupTableData::Hold; }
		else if (outside == "periodic") { table->outside = LookupTableData::Periodic; }
		else
		{
			delete table;
			CHECKandTHROWstring("esym.LookupTable: outside must be 'zero', 'hold' or 'periodic', not '" + outside + "'");
		}
		data.reset(table);
	}
};

//! the value of a lookup table at its operand, or its derivative (#3175); the table is kept by every node that uses it
class ExpressionLookupTable: public ExpressionBase
{
	ExpressionBase* operand;
	std::shared_ptr<const LookupTableData> table;
	bool derivative;
public:
	ExpressionLookupTable(ExpressionBase* op, const std::shared_ptr<const LookupTableData>& tableInit, bool derivativeInit) :
		ExpressionBase(), operand(op), table(tableInit), derivative(derivativeInit) {}

	virtual void Destroy() override { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { DeleteNode(operand); } } }

	virtual Real Evaluate() const override { return table->Value(operand->Evaluate(), derivative); }
	virtual STDstring ToString() const override { return STDstring(FlatName()) + "(" + operand->ToString() + ")"; }

	virtual Index NumberOfOperands() const override { return 1; }
	virtual FlatNode GetOperand(Index i) const override { return FlatNode(operand); }
	virtual Real EvaluateFromOperands(const FlatValue* values) const override { return table->Value(values[0].real, derivative); }
	virtual const char* FlatName() const override { return derivative ? "LookupTableDerivative" : "LookupTable"; }
	virtual FlatNode NewWithOperands(const FlatNode* operands) const override { return FlatNode(new ExpressionLookupTable((ExpressionBase*)operands[0].node, table, derivative)); }
	virtual STDstring ToStringFromOperands(const STDstring* operands) const override { return STDstring(FlatName()) + "(" + operands[0] + ")"; }
	const std::shared_ptr<const LookupTableData>& GetTable() const { return table; }
	bool IsDerivative() const { return derivative; }
};


//! expression carrying a lightweight real number finally
class ExpressionReal: public ExpressionBase
{
protected:
	Real value;
public:
	ExpressionReal(Real valueInit) : ExpressionBase(), value(valueInit)
	{
		IncreaseReferenceCounter(); //is referenced immediately
	}

	//ExpressionBase* clone() const override {
	//	NewCount()++;
	//	return new ExpressionReal(*this);
	//}
	virtual void Destroy() override
	{
	}
	virtual Real Evaluate() const override {
		return value;
	}
	virtual STDstring ToString() const override
	{
		return EXUstd::ToString(value);
	}

	virtual void SetValue(Real valueInit) { value = valueInit; }
	virtual Real GetValue() const { return value; }

};

//! expression carrying a lightweight real number finally
class ExpressionNamedReal: public ExpressionReal
{
	STDstring name;
public:
	ExpressionNamedReal(const STDstring& nameInit, Real valueInit) : ExpressionReal(valueInit), name(nameInit)
	{
		CHECKandTHROW(IsRegularVariableName(nameInit), "ExpressionNamedReal(value, name): string must start with letter and contain only regular letters or numbers", ExudynValueError);
		//pout << "initialize NamedReal: " << name << ": " << value << "\n";
	}

	virtual STDstring ToString() const override
	{
		return name; //returns name of variable ...
	}

	virtual void SetName(const STDstring& nameInit) { name = nameInit; }
	virtual const STDstring& GetName() const { return name; }
};

//not needed now; maybe later with C++ recording (link to parameters or unknowns / ODE coords)
////! expression linking to a Real in C++ (e.g. for functions arguments or for linking to item parameters)
//class ExpressionLinkedReal: public ExpressionBase
//{
//	Real* valuePtr;
//	STDstring name;
//public:
//	ExpressionLinkedReal(Real* valuePtrInit, const STDstring& nameInit) : ExpressionBase(), 
//		valuePtr(valuePtrInit), name(nameInit)
//	{
//		IncreaseReferenceCounter(); //is referenced immediately
//	}
//
//	virtual void Destroy() //! does not own memory
//	{
//	}
//
//	void Set(Real* valuePtrInit, const STDstring& nameInit) 
//	{
//		SetReferenceCounter(1); //is referenced immediately
//		valuePtr = valuePtrInit;
//		name = nameInit;
//	}
//
//	const STDstring& GetName() const { return name; }
//
//	void SetName(const STDstring& nameInit) { name = nameInit; }
//
//	//! set to specific value
//	virtual void SetValue(Real value) { *valuePtr = value; }
//
//	//! get evaluated / current value
//	virtual Real Evaluate() const override {
//		CHECKandTHROW(valuePtr != nullptr, "ExpressionLinkedReal: encountered valuePtr == nullptr; possible problem with Symbolic module");
//		return *valuePtr;
//	}
//	virtual STDstring ToString() const override
//	{
//		return EXUstd::ToString(*valuePtr);
//	}
//};




//! base class for symbolic Real: can either be Real or contain a symbolic expression
//! call it Symbolic::SReal, to distinguish from Real !
class SReal
{
	ExpressionBase* expr;
	Real value;
public:
	static bool recordExpressions; // Static flag to control expression recording
	//! true while CreateSymbolicUserFunction records a function: the value of a named variable set then would be
	//! read by the recording at once, and not at the call (#3090)
	static bool recordingFunction;
	static void CheckNotRecordingFunction()
	{
		if (recordingFunction) { PyError("exudyn.symbolic: SetValue, SetVector, SetMatrix or a component of a named variable "
			"is set while a user function is recorded, which the recording would not follow: compute a new symbolic value instead", PyErrorType::valueError); }
	}
	static bool flagDebug;

	SReal() : expr(nullptr), value(0.) {}
	SReal(const Real& val) : expr(nullptr), value(val) {}
	SReal(const Index& val) : expr(nullptr), value((Real)val) {}
	//! take ownership of an expression and cache its value, from the recorded values of its operands (#3090); every
	//! operator returns through here
	SReal(ExpressionBase* e) : expr(e), value(0.)
	{
		if (e)
		{
			e->IncreaseReferenceCounter();
			try
			{
				value = RecordedReal(e);
			}
			catch (...)
			{
				//the value may throw - inconsistent vector sizes, an index out of range. This
				//object then never completes, so ~SReal will NOT run and the tree would be leaked
				//together with everything it owns (#2481). Release it exactly as the destructor
				//would, then let the error through
				expr = nullptr;
				e->DecreaseReferenceCounter();
				if (e->ReferenceCounter() == 0)
				{
					DeleteNode(e);
				}
				throw;
			}
		}
	}
	//! constructor with value and name, gives subexpression
	//! used for variables
	SReal(const STDstring& name, const Real& val) : expr(nullptr), value(val)
	{
		if (recordExpressions) {
			ExpressionBase::NewCount()++;
			expr = new ExpressionNamedReal(name, val);
		}
	}
	//! copy constructor
	SReal(const SReal& other) : expr(other.expr), value(other.value)
	{
		if (GetFlagDebug()) { std::cout << "copy constructor: " << ToString() << "\n"; }
		if (expr) { expr->IncreaseReferenceCounter(); }
	}
	//SReal(SReal&& other) = delete;

	//! assume that expression e holds a NamedReal and override its value
	void SetExpressionNamedReal(Real value)
	{
		CHECKandTHROW((expr != nullptr) && (typeid(*expr) == typeid(ExpressionNamedReal) ), 
			"SReal::SetExpressionNamedReal expects ExpressionNamedReal in expression", ExudynValueError);
		((ExpressionNamedReal*)expr)->SetValue(value);
	}

	//! check if there is a ExpressionNamedReal in expr 
	bool IsExpressionNamedReal()  const
	{
		if (expr == nullptr) { return false; }

		if (dynamic_cast<ExpressionNamedReal*>(expr) != nullptr) 
		{
			return true;
		}
		return false;
	}
	ExpressionNamedReal& GetExpressionNamedReal() { return (ExpressionNamedReal&)(*expr); }
	const ExpressionNamedReal& GetExpressionNamedReal() const { return (const ExpressionNamedReal&)(*expr); }

	//virtual void Destroy()
	~SReal()
	{
		if (expr)
		{
			expr->DecreaseReferenceCounter();
			if (expr->ReferenceCounter() == 0)
			{
				if (GetFlagDebug()) { std::cout << "now we delete: " << ToString() << "\n"; }
				DeleteNode(expr);
			}
		}
	}


	// Modified implicit conversion to Real
	explicit operator Real() const
	{
		if (expr)
		{
			return expr->Evaluate();
		}
		return value;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	static void SetRecording(bool flag) { recordExpressions = flag; }
	static bool GetRecording() { return recordExpressions; }
	static void SetFlagDebug(bool flag) { flagDebug = flag; }
	static bool GetFlagDebug() { return flagDebug; }

	//access to internal value, not intended for Python
	void SetValue(Real valueInit)
	{
		CHECKandTHROW(expr == nullptr, "value can only be accessed if Real does not contain an expression", ExudynValueError);
		value = valueInit;
	}
	Real GetValue() const 
	{ 
		CHECKandTHROW(expr == nullptr, "value can only be accessed if Real does not contain an expression", ExudynValueError);
		return value;
	}

	//! set value either for named real or for internal value
	template<typename RealType>
	void SetSymbolicValue(RealType valueInit)
	{
		CHECKandTHROW((typeid(RealType) == typeid(Real)),
			"SetValue can only be called with float numbers", ExudynValueError);
		if (IsExpressionNamedReal())
		{
			SReal::CheckNotRecordingFunction();
			GetExpressionNamedReal().SetValue((Real)valueInit);
			value = (Real)valueInit; //to make it consistent!
		}
		else
		{
			CHECKandTHROW(expr == nullptr, "SetValue can only be called for symbolic Real variables", ExudynValueError);
			value = (Real)valueInit;
		}
	}

	//! set value either for named real or for internal value
	void SetSymbolicValue(const SReal& valueInit)
	{
		CHECKandTHROWstring("SetValue can only be called with float numbers");
	}

	void SetExpression(ExpressionBase* e) { expr = e; if (e) { e->IncreaseReferenceCounter(); } }
	ExpressionBase* GetExpression() const { return expr; }

	virtual Real Evaluate() const {
		if (expr) { return expr->Evaluate(); }
		return value;
	}

	//! the derivative by a named variable at the current values, by the one-pass derivative of the table (#3171)
	Real DiffSReal(const SReal& var) const
	{
		CHECKandTHROW(var.IsExpressionNamedReal(), "Symbolic::SReal::DiffSReal expects ExpressionNamedReal in expression", ExudynValueError);
		if (!expr) { return 0.; } //diff(value) = 0
		std::vector<Real> derivative;
		OnePassDerivative(FlatNode(expr), var.GetExpression(), 1, derivative);
		return derivative[0];
	}

	virtual STDstring ToString() const {
		if (expr) { return ExpressionText(FlatNode(expr)); }
		return EXUstd::ToString(value);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	SReal& operator=(const SReal& other)
	{
		if (this != &other) // Protect against self-assignment
		{
			if (expr) //if expression is not used any more, destroy!
			{
				expr->DecreaseReferenceCounter();
				if (expr->ReferenceCounter() == 0) { DeleteNode(expr); }
			}

			value = other.value;
			expr = other.expr;
			if (expr) { expr->IncreaseReferenceCounter(); }

			if (GetFlagDebug()) { std::cout << "**********\nassign\n**********\n"; }

			//expr = other.expr ? other.expr->clone() : nullptr;
		}
		return *this;
	}

private:
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//add some helper functions to automatically treat Real and SReal in templates
	//! convert Real, int or SReal to value
	static Real Number2Real(Real x)
	{
		return x;
	}
	static Real Number2Real(SReal x)
	{
		return x.value;
	}
	//! an operation is recorded if recording is on and an operand holds an expression or a named variable; on numbers
	//! alone it gives a number, also while recording, so that esym.sin(0.5)*10 in a Python user function is a float (#3017)
	static bool HasExpression(const SReal& x) { return x.expr != nullptr; }
	static bool HasExpression(Real x) { return false; }
	template<typename... Args>
	static bool RecordExpressions(const Args&... operands) { return recordExpressions && (HasExpression(operands) || ...); }
	//! Get Expression* either for SReal or Real
	static ExpressionBase* GetFunctionExpression(const SReal& x, bool increaseReferenceCounter = true)
	{
		ExpressionBase::NewCount() += (x.expr == 0);
		if (x.expr && increaseReferenceCounter) { x.expr->IncreaseReferenceCounter(); }
		return x.expr ? x.expr : new ExpressionReal(x.value);
	}
	static ExpressionBase* GetFunctionExpression(Real x, bool increaseReferenceCounter = true)
	{
		ExpressionBase::NewCount() += 1;
		return new ExpressionReal(x);
	}
public:
	//SReal operator-(const SReal& rhs) const {
	//	if (!recordExpressions) {
	//		return value - rhs.value;
	//	}
	//	ExpressionBase::NewCount() += 1;
	//	return new ExpressionOperatorMinus(GetFunctionExpression(*this),
	//		GetFunctionExpression(rhs));
	//}

	SReal& operator+=(const SReal& rhs) {
		if (!RecordExpressions(*this, rhs)) {
			value += rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorPlus(GetFunctionExpression(*this, false), //for inplace operatoins, expression is moved; no new reference!
			GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		value = RecordedReal(expr);
		return *this;
	}

	SReal& operator-=(const SReal& rhs) {
		if (!RecordExpressions(*this, rhs)) {
			value -= rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorMinus(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		value = RecordedReal(expr);
		return *this;
	}

	SReal& operator*=(const SReal& rhs) {
		if (!RecordExpressions(*this, rhs)) {
			value *= rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorMul(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		value = RecordedReal(expr);
		return *this;
	}

	SReal& operator/=(const SReal& rhs) {
		if (!RecordExpressions(*this, rhs)) {
			value /= rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorDiv(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		value = RecordedReal(expr);
		return *this;
	}

	// Overloaded addition operator
	friend SReal operator+(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value + right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorPlus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	// Overloaded subtraction operator
	friend SReal operator-(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value - right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorMinus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	// Overloaded subtraction operator
	friend SReal operator*(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value * right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorMul(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	// Overloaded division operator
	friend SReal operator/(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value / right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorDiv(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded comparison operators
	friend SReal operator==(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value == right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator!=(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value != right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorNEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator>(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value > right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorGT(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator>=(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value >= right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorGE(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator<=(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value <= right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorLE(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator<(const SReal& left, const SReal& right)
	{
		if (!RecordExpressions(left, right)) {
			return left.value < right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorLT(GetFunctionExpression(left), GetFunctionExpression(right));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Unary plus operator
	SReal operator+() const {
		if (!RecordExpressions(*this)) {
			return +value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionUnaryPlus(GetFunctionExpression(*this));
	}

	// Unary minus operator
	SReal operator-() const {
		if (!RecordExpressions(*this)) {
			return -value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionUnaryMinus(GetFunctionExpression(*this));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded isfinite function
	template<typename RealType>
	static SReal isfinite(const RealType& x) {
		if (!RecordExpressions(x)) { return std::isfinite(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionIsFinite(GetFunctionExpression(x));
	}

	// Overloaded abs function
	template<typename RealType>
	static SReal abs(const RealType& x) {
		if (!RecordExpressions(x)) { return std::fabs(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionAbs(GetFunctionExpression(x));
	}

	// Overloaded sign function
	template<typename RealType>
	static SReal sign(const RealType& x) {
		if (!RecordExpressions(x)) { return EXUstd::SignReal(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSign(GetFunctionExpression(x));
	}

	// Overloaded not function; "not" is reserved in C++
	template<typename RealType>
	static SReal Not(const RealType& x) { 
		if (!RecordExpressions(x)) { return !(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionNot(GetFunctionExpression(x));
	}

	// Overloaded round function
	template<typename RealType>
	static SReal round(const RealType& x) {
		if (!RecordExpressions(x)) { return std::round(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionRound(GetFunctionExpression(x));
	}

	// Overloaded ceil function
	template<typename RealType>
	static SReal ceil(const RealType& x) {
		if (!RecordExpressions(x)) { return std::ceil(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionCeil(GetFunctionExpression(x));
	}

	// Overloaded floor function
	template<typename RealType>
	static SReal floor(const RealType& x) {
		if (!RecordExpressions(x)) { return std::floor(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionFloor(GetFunctionExpression(x));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded sqrt function
	template<typename RealType>
	static SReal sqrt(const RealType& x) {
		if (!RecordExpressions(x)) { return std::sqrt(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSqrt(GetFunctionExpression(x));
	}

	// Overloaded exp function
	template<typename RealType>
	static SReal exp(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Exp(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionExp(GetFunctionExpression(x));
	}

	// Overloaded log function
	template<typename RealType>
	static SReal log(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Log(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionLog(GetFunctionExpression(x));
	}





	//++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded sin function
	template<typename RealType>
	static SReal sin(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Sin(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSin(GetFunctionExpression(x));
	}

	// Overloaded cos function
	template<typename RealType>
	static SReal cos(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Cos(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionCos(GetFunctionExpression(x));
	}

	// Overloaded tan function
	template<typename RealType>
	static SReal tan(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Tan(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionTan(GetFunctionExpression(x));
	}

	// Overloaded asin function
	template<typename RealType>
	static SReal asin(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Asin(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionASin(GetFunctionExpression(x));
	}

	// Overloaded acos function
	template<typename RealType>
	static SReal acos(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Acos(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionACos(GetFunctionExpression(x));
	}

	// Overloaded atan function
	template<typename RealType>
	static SReal atan(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Atan(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionATan(GetFunctionExpression(x));
	}

	//++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded sin function
	template<typename RealType>
	static SReal sinh(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Sinh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSinh(GetFunctionExpression(x));
	}

	// Overloaded cos function
	template<typename RealType>
	static SReal cosh(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Cosh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionCosh(GetFunctionExpression(x));
	}

	// Overloaded tan function
	template<typename RealType>
	static SReal tanh(const RealType& x) {
		if (!RecordExpressions(x)) { return EM::Tanh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionTanh(GetFunctionExpression(x));
	}

	// Overloaded asin function
	template<typename RealType>
	static SReal asinh(const RealType& x) {
		if (!RecordExpressions(x)) { return std::asinh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionASinh(GetFunctionExpression(x));
	}

	// Overloaded acos function
	template<typename RealType>
	static SReal acosh(const RealType& x) {
		if (!RecordExpressions(x)) { return std::acosh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionACosh(GetFunctionExpression(x));
	}

	// Overloaded atan function
	template<typename RealType>
	static SReal atanh(const RealType& x) {
		if (!RecordExpressions(x)) { return std::atanh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionATanh(GetFunctionExpression(x));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions with 2 arguments

	// Overloaded pow function
	template<typename RealType, typename RealTypeExp>
	static SReal pow(const RealType& base, const RealTypeExp& exponent) {
		if (!RecordExpressions(base, exponent)) { return EM::Pow(Number2Real(base), Number2Real(exponent)); }
		ExpressionBase::NewCount()++;
		return new ExpressionPower(GetFunctionExpression(base), GetFunctionExpression(exponent));
	}

	// Overloaded atan2 function
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal atan2(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!RecordExpressions(arg1, arg2)) { return EM::Atan2(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionAtan2(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}

	// Overloaded modulo function (floating-point remainder of division)
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal mod(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!RecordExpressions(arg1, arg2)) { return std::fmod(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionMod(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}

	// Overloaded min function
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal min(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!RecordExpressions(arg1, arg2)) { return std::min(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionMin(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}

	// Overloaded max function
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal max(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!RecordExpressions(arg1, arg2)) { return std::max(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionMax(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions with 3 arguments

	// If-then-else construct, for expressions
	template<typename RealType, typename RealTypeTrue, typename RealTypeFalse>
	static SReal IfThenElse(const RealType& condition, const RealTypeTrue& ifTrue, const RealTypeFalse& ifFalse) {
		if (!RecordExpressions(condition, ifTrue, ifFalse)) {return Number2Real(condition) ? Number2Real(ifTrue) : Number2Real(ifFalse); }
		ExpressionBase::NewCount()++;
		return new ExpressionIfThenElse(GetFunctionExpression(condition), GetFunctionExpression(ifTrue), GetFunctionExpression(ifFalse));
	}

	//! the value of a lookup table at x, or its derivative; a number for a number (#3175)
	template<typename RealType>
	static SReal LookupTable(const std::shared_ptr<const LookupTableData>& table, const RealType& x, bool derivative) {
		if (!RecordExpressions(x)) { return table->Value(Number2Real(x), derivative); }
		ExpressionBase::NewCount()++;
		return new ExpressionLookupTable(GetFunctionExpression(x), table, derivative);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	friend std::ostream& operator<<(std::ostream& os, const SReal& thisExpr)
	{
		os << thisExpr.ToString();
		return os;
	}

};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! THE FLAT EVALUATION (#3053, #3054): the graph of a recording, sorted once into a list of operations so that every
//! operand comes before its parent; a node that several parents share - a number, a vector or a matrix - is one
//! operation and is computed once per evaluation. Operation k writes slot k: the number, or a pointer to the vector or
//! matrix its node evaluated into. Nothing is used by the next evaluation, which computes every operation again
//! (decision D18). A node without operands for the program - a leaf, IfThenElse - is one operation that evaluates its
//! own subtree as a tree.
class FlatProgram
{
	struct Operation
	{
		FlatNode node;
		Index firstOperand;	 //!< the slots of its operands are operandSlots[firstOperand ...]
		Index numberOfOperands;
		bool inlineText;		//!< written into its users in the text: a leaf, or a component of an argument
	};
	std::vector<Operation> operations;  //!< operation k writes slot k
	std::vector<Index> operandSlots;
	std::vector<Index> results;		 //!< the slots of the results, in the order of the roots

	static Index NumberOfOperands(const FlatNode& node)
	{
		switch (node.kind)
		{
		case FlatKind::Real: return ((const ExpressionBase*)node.node)->NumberOfOperands();
		case FlatKind::Vector: return ((const VectorExpressionBase*)node.node)->NumberOfOperands();
		default: return ((const MatrixExpressionBase*)node.node)->NumberOfOperands();
		}
	}
	static FlatNode GetOperand(const FlatNode& node, Index i)
	{
		switch (node.kind)
		{
		case FlatKind::Real: return ((const ExpressionBase*)node.node)->GetOperand(i);
		case FlatKind::Vector: return ((const VectorExpressionBase*)node.node)->GetOperand(i);
		default: return ((const MatrixExpressionBase*)node.node)->GetOperand(i);
		}
	}
public:
	//! build the list for these roots
	void Build(const std::vector<FlatNode>& roots)
	{
		Clear();
		std::unordered_map<const void*, Index> slotOfNode; //where each node of the recording was put, while building
		std::vector<std::pair<FlatNode, Index>> stack;	 //node and the number of its operands already done
		for (const FlatNode& root : roots)
		{
			CHECKandTHROW(root.node != nullptr, "FlatProgram::Build: a result is no expression");
			if (slotOfNode.find(root.node) == slotOfNode.end()) { stack.push_back({ root, 0 }); }
			while (stack.size())
			{
				FlatNode node = stack.back().first;
				Index done = stack.back().second;
				Index n = NumberOfOperands(node);
				if (done < n) //first the operands, depth first, without recursion
				{
					FlatNode operand = GetOperand(node, done);
					stack.back().second++;
					if (slotOfNode.find(operand.node) == slotOfNode.end()) { stack.push_back({ operand, 0 }); }
					continue;
				}
				Operation operation;
				operation.node = node;
				operation.firstOperand = (Index)operandSlots.size();
				operation.numberOfOperands = n;
				for (Index i = 0; i < n; i++) { operandSlots.push_back(slotOfNode[GetOperand(node, i).node]); }
				operation.inlineText = (n == 0);
				if (n != 0 && node.kind == FlatKind::Real && ((const ExpressionBase*)node.node)->InlineInFlatText())
				{
					operation.inlineText = true;
					for (Index i = 0; i < n; i++) { operation.inlineText &= operations[operandSlots[operation.firstOperand + i]].inlineText; }
				}
				slotOfNode[node.node] = (Index)operations.size();
				operations.push_back(operation);
				stack.pop_back();
			}
			results.push_back(slotOfNode[root.node]);
		}
	}

	//! true, if Build has been called with at least one root
	bool IsBuilt() const { return results.size() != 0; }
	void Clear() { operations.clear(); operandSlots.clear(); results.clear(); }

	Index NumberOfResults() const { return (Index)results.size(); }
	//! the operations in their order, for a copy of the recording (#2978)
	Index NumberOfOperations() const { return (Index)operations.size(); }
	const FlatNode& OperationNode(Index k) const { return operations[k].node; }
	Index NumberOfOperationOperands(Index k) const { return operations[k].numberOfOperands; }
	//! the operation that computes operand i of operation k
	Index OperandSlot(Index k, Index i) const { return operandSlots[operations[k].firstOperand + i]; }
	//! the operation that computes result i
	Index ResultSlot(Index i) const { return results[i]; }
	//! the number of operations that have operands, the arithmetic of one evaluation
	Index NumberOfComputations() const
	{
		Index n = 0;
		for (const Operation& operation : operations) { n += !operation.inlineText; }
		return n;
	}

	//! the list as text: one line per operation with operands, tK = op(...); a node without operands is written into
	//! the operations that use it; then the results; flatten = false: every operation written into its users, the
	//! results as expressions (#3087)
	STDstring ToString(const std::vector<STDstring>& resultNames, bool flatten = true) const
	{
		std::vector<STDstring> names(operations.size());
		std::vector<STDstring> operands;
		STDstring text;
		Index k = 0;
		for (const Operation& operation : operations)
		{
			const void* node = operation.node.node;
			if (operation.numberOfOperands == 0)
			{
				switch (operation.node.kind)
				{
				case FlatKind::Real: names[k] = ((const ExpressionBase*)node)->ToString(); break;
				case FlatKind::Vector: names[k] = ((const VectorExpressionBase*)node)->ToString(); break;
				default: names[k] = ((const MatrixExpressionBase*)node)->ToString(); break;
				}
			}
			else
			{
				operands.resize(operation.numberOfOperands);
				for (Index i = 0; i < operation.numberOfOperands; i++) { operands[i] = names[operandSlots[operation.firstOperand + i]]; }
				STDstring value;
				switch (operation.node.kind)
				{
				case FlatKind::Real: value = ((const ExpressionBase*)node)->ToStringFromOperands(operands.data()); break;
				case FlatKind::Vector: value = ((const VectorExpressionBase*)node)->ToStringFromOperands(operands.data()); break;
				default: value = ((const MatrixExpressionBase*)node)->ToStringFromOperands(operands.data()); break;
				}
				if (operation.inlineText || !flatten) { names[k] = value; }
				else
				{
					names[k] = "t" + EXUstd::ToString(k);
					text += names[k] + " = " + value + "\n";
				}
			}
			k++;
		}
		for (Index i = 0; i < (Index)results.size(); i++)
		{
			text += (i < (Index)resultNames.size() ? resultNames[i] : "result[" + EXUstd::ToString(i) + "]") + " = " + names[results[i]] + "\n";
		}
		return text;
	}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! a set of named variables and the scope of their names (#3149), available as exudyn.symbolic.variables and as sets
//! of its own; scalars as SReal, which C++ reads, named vectors and matrices as their Python objects, esym.Vector and
//! esym.Matrix; the names in the order they were added, the inputs of esym.Function in this order
class VariableSet
{
private:
	std::unordered_map<std::string, SReal> variables;
	std::unordered_map<std::string, py::object> arrays;	//!< the named vectors and matrices
	std::vector<STDstring> order;						//!< all names, in the order they were added
public:
	VariableSet() {}

	void AddVariable(const STDstring& name, Real value)
	{
		SReal sreal(name, value);
		AddVariable(sreal);
	}

	void AddVariable(const SReal& sreal)
	{
		if (!sreal.IsExpressionNamedReal())
		{
			PyError("VariableSet::AddVariable(symbolic.Real): only accepts named variables created as Real(value, name)", PyErrorType::valueError);
		}

		STDstring name = sreal.GetExpressionNamedReal().GetName();
		if (HasVariable(name))
		{
			PyError("VariableSet::AddVariable(symbolic.Real): variable name already exists", PyErrorType::valueError);
		}
		variables[name] = sreal;
		order.push_back(name);
	}

	//! add a named vector or matrix, esym.Vector or esym.Matrix, under its name
	void AddArray(const STDstring& name, const py::object& variable)
	{
		if (HasVariable(name))
		{
			PyError("VariableSet::Add: variable name '" + name + "' already exists", PyErrorType::valueError);
		}
		arrays[name] = variable;
		order.push_back(name);
	}

	//! returns true, if variable exists
	bool HasVariable(const STDstring& name) const
	{
		return (variables.find(name) != variables.end()) || (arrays.find(name) != arrays.end());
	}

	//! true, if the variable is a named vector or matrix
	bool IsArray(const STDstring& name) const { return arrays.find(name) != arrays.end(); }
	//! the named vector or matrix of this name
	const py::object& GetArray(const STDstring& name) const { return arrays.at(name); }

	//! returns specific variable, a scalar
	const SReal& GetVariable(const STDstring& name)
	{
		auto search = variables.find(name);
		if (search == variables.end())
		{
			PyError("VariableSet::GetVariable(name): name does not exist", PyErrorType::valueError);
		}
		return variables[name];
	}

	//! override existing variable or set new variable
	void SetVariable(const STDstring& name, Real value)
	{
		auto search = variables.find(name);
		if (search == variables.end())
		{
			AddVariable(name, value);
		}
		variables[name].SetExpressionNamedReal(value);
	}

	//! remove the variable of that name, which esym.Real(name, value, scope) replaces by one of another kind (#3208)
	void RemoveVariable(const STDstring& name)
	{
		variables.erase(name);
		arrays.erase(name);
		order.erase(std::remove(order.begin(), order.end(), name), order.end());
	}

	//! number of variables in container
	Index NumberOfItems() const { return (Index)order.size(); }

	//! the names of the variables, in the order they were added
	std::vector<STDstring> GetNames() const { return order; }

	//! clear all variables
	void Reset()
	{
		variables.clear();
		arrays.clear();
		order.clear();
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! create output, which is compatible to Python dict
	virtual STDstring ToString() const
	{
		STDstring s = "{";
		STDstring sep = "";
		for (const STDstring& key : order)
		{
			auto scalar = variables.find(key);
			s += sep + "'" + key + "': " + (scalar != variables.end() ? EXUstd::ToString(scalar->second.Evaluate())
				: py::cast<STDstring>(py::str(arrays.at(key).attr("Evaluate")())));
			sep = ", ";
		}
		s += "}";
		return s;
	}

	friend std::ostream& operator<<(std::ostream& os, const VariableSet& varSpace)
	{
		os << varSpace.ToString();
		return os;
	}
};






}; //namespace Symbolic

#endif //SYMBOLIC__H
