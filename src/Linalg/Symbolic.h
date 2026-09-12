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

namespace Symbolic
{
class ExpressionNamedReal;


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
	//Build derivative w.r.t. var
	virtual Real Diff(ExpressionNamedReal* var) const = 0;
	virtual STDstring ToString() const = 0; // Convert the expression tree to a string
	//virtual ExpressionBase* clone() const = 0; // Pure virtual clone method
	virtual void Destroy() = 0; //Pure virtual destroy method, which deletes tree reversely

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
	virtual ResizableConstVector Evaluate() const = 0;
	virtual STDstring ToString() const = 0;
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
	virtual ResizableMatrix Evaluate() const = 0;
	virtual Real EvaluateComponent(Index row, Index column) const = 0;
	virtual Index NumberOfRows() const = 0;
	virtual Index NumberOfColumns() const = 0;
	virtual STDstring ToString() const = 0;
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ExpressionOperatorPlus: public ExpressionBase
{
	ExpressionBase* left, * right;
public:
	ExpressionOperatorPlus(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}
	virtual Real Evaluate() const override
	{
		return left->Evaluate() + right->Evaluate();
	}
	virtual Real Diff(ExpressionNamedReal* var) const override
	{
		return left->Diff(var) + right->Diff(var);
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " + " + right->ToString() + ")";
	}
};

class ExpressionOperatorMinus: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorMinus(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}
	virtual Real Evaluate() const override {
		return left->Evaluate() - right->Evaluate();
	}
	virtual Real Diff(ExpressionNamedReal* var) const override
	{
		return left->Diff(var) - right->Diff(var);
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " - " + right->ToString() + ")";
	}
};

class ExpressionOperatorMul: public ExpressionBase
{
	ExpressionBase* left, * right;
public:
	ExpressionOperatorMul(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override
	{
		return left->Evaluate() * right->Evaluate();
	}
	virtual Real Diff(ExpressionNamedReal* var) const override
	{
		return left->Diff(var) * right->Evaluate() + left->Evaluate() * right->Diff(var);
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}
};

class ExpressionOperatorDiv: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorDiv(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return left->Evaluate() / right->Evaluate(); // Consider handling division by zero
	}
	virtual double Diff(ExpressionNamedReal* var) const override {
		double numVal = left->Evaluate();
		double denVal = right->Evaluate();
		double numDiff = left->Diff(var);
		double denDiff = right->Diff(var);

		if (denVal == 0) {
			return std::numeric_limits<Real>::quiet_NaN(); // Handle division by zero
		}

		return (numDiff * denVal - numVal * denDiff) / (denVal * denVal);
	}
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " / " + right->ToString() + ")";
	}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class ExpressionOperatorEQ: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorEQ(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() == right->Evaluate()); // Consider handling division by zero
	}
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::operator==::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " == " + right->ToString() + ")";
	}
};

class ExpressionOperatorNEQ: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorNEQ(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() != right->Evaluate()); // Consider handling division by zero
	}
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::operator!=::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " != " + right->ToString() + ")";
	}
};

class ExpressionOperatorGT: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorGT(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() > right->Evaluate()); // Consider handling division by zero
	}
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::operator>::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " > " + right->ToString() + ")";
	}
};

class ExpressionOperatorGE: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorGE(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() >= right->Evaluate()); // Consider handling division by zero
	}
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::operator>=::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " >= " + right->ToString() + ")";
	}
};

class ExpressionOperatorLE: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorLE(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() <= right->Evaluate()); // Consider handling division by zero
	}
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::operator<=::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " <= " + right->ToString() + ")";
	}
};

class ExpressionOperatorLT: public ExpressionBase {
	ExpressionBase* left, * right;
public:
	ExpressionOperatorLT(ExpressionBase* l, ExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return Real(left->Evaluate() < right->Evaluate()); // Consider handling division by zero
	}
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::operator<::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " < " + right->ToString() + ")";
	}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ExpressionUnaryPlus: public ExpressionBase {
	ExpressionBase* operand;
public:
	ExpressionUnaryPlus(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy()
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return operand->Evaluate();
	}
	virtual double Diff(ExpressionNamedReal* var) const override { return operand->Diff(var); }
	virtual STDstring ToString() const override {
		return operand->ToString();
		//return "(+" + operand->ToString() + ")";
	}
};

class ExpressionUnaryMinus: public ExpressionBase {
	ExpressionBase* operand;
public:
	ExpressionUnaryMinus(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy()
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return -operand->Evaluate();
	}
	virtual double Diff(ExpressionNamedReal* var) const override { return -operand->Diff(var); }
	virtual STDstring ToString() const override {
		return "(-" + operand->ToString() + ")";
	}
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

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::isfinite(operand->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::isfinite::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "isfinite(" + operand->ToString() + ")"; }
};

class ExpressionAbs: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionAbs(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::fabs(operand->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override {
		Real val = operand->Evaluate();
		if (val == 0)
			return std::numeric_limits<Real>::quiet_NaN();
		return (val > 0. ? 1 : -1) * operand->Diff(var);
	}	virtual STDstring ToString() const override { return "abs(" + operand->ToString() + ")"; }
};

class ExpressionSign: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSign(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return EXUstd::SignReal(operand->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { return 0.; }
	virtual STDstring ToString() const override { return "sign(" + operand->ToString() + ")"; }
};


class ExpressionNot : public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionNot(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return !(operand->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::not::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "Not(" + operand->ToString() + ")"; } //"not" is reserved in C++ and Python
};

class ExpressionRound: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionRound(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::round(operand->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::round::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "round(" + operand->ToString() + ")"; }
};

class ExpressionCeil: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionCeil(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::ceil(operand->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::ceil::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "ceil(" + operand->ToString() + ")"; }
};

class ExpressionFloor: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionFloor(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::floor(operand->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::floor::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "floor(" + operand->ToString() + ")"; }
};


//+++++++++++++++++++++++++++++++++++++++++++++
class ExpressionSqrt: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSqrt(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::sqrt(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		double val = operand->Evaluate();
		if (val <= 0) { return std::numeric_limits<double>::quiet_NaN(); }
		return 0.5 / std::sqrt(val) * operand->Diff(var);
	}
	virtual STDstring ToString() const override { return "sqrt(" + operand->ToString() + ")"; }
};

class ExpressionExp: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionExp(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::exp(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override { return std::exp(operand->Evaluate()) * operand->Diff(var); }
	virtual STDstring ToString() const override { return "exp(" + operand->ToString() + ")"; }
};

class ExpressionLog: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionLog(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::log(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override { return (1 / operand->Evaluate()) * operand->Diff(var); }
	virtual STDstring ToString() const override { return "log(" + operand->ToString() + ")"; }
};

//+++++++++++++++++++++++++++++++++++++++++++++
class ExpressionSin: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSin(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::sin(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override { return std::cos(operand->Evaluate()) * operand->Diff(var); }	
	virtual STDstring ToString() const override { return "sin(" + operand->ToString() + ")"; }
};

class ExpressionCos: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionCos(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::cos(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override { return -std::sin(operand->Evaluate()) * operand->Diff(var); }
	virtual STDstring ToString() const override { return "cos(" + operand->ToString() + ")"; }
};

class ExpressionTan: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionTan(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::tan(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		return (1 + EXUstd::Square(std::tan(operand->Evaluate())) ) * operand->Diff(var);
	}
	virtual STDstring ToString() const override { return "tan(" + operand->ToString() + ")"; }
};

class ExpressionASin: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionASin(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::asin(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		double val = operand->Evaluate();
		if (val <= -1 || val >= 1) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return operand->Diff(var) / std::sqrt(1 - val * val);
	}
	virtual STDstring ToString() const override { return "asin(" + operand->ToString() + ")"; }
};

class ExpressionACos: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionACos(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::acos(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		double val = operand->Evaluate();
		if (val <= -1 || val >= 1) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return -operand->Diff(var) / std::sqrt(1 - val * val);
	}
	virtual STDstring ToString() const override { return "acos(" + operand->ToString() + ")"; }
};

class ExpressionATan: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionATan(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::atan(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		return operand->Diff(var) / (1 + EXUstd::Square(operand->Evaluate()));
	}
	virtual STDstring ToString() const override { return "atan(" + operand->ToString() + ")"; }
};


//+++++++++++++++++++++++++++++++++++++++++++++
class ExpressionSinh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionSinh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::sinh(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override { return std::cosh(operand->Evaluate()) * operand->Diff(var); }
	virtual STDstring ToString() const override { return "sinh(" + operand->ToString() + ")"; }
};

class ExpressionCosh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionCosh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::cosh(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override { return std::sinh(operand->Evaluate()) * operand->Diff(var); }
	virtual STDstring ToString() const override { return "cosh(" + operand->ToString() + ")"; }
};

class ExpressionTanh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionTanh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::tanh(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override { return (1 - EXUstd::Square(std::tanh(operand->Evaluate()))) * operand->Diff(var); }
	virtual STDstring ToString() const override { return "tanh(" + operand->ToString() + ")"; }
};

class ExpressionASinh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionASinh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::asinh(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		return operand->Diff(var) / std::sqrt(EXUstd::Square(operand->Evaluate()) + 1);
	}
	virtual STDstring ToString() const override { return "asinh(" + operand->ToString() + ")"; }
};

class ExpressionACosh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionACosh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::acosh(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		double val = operand->Evaluate();
		if (val < 1) { return std::numeric_limits<double>::quiet_NaN(); }
		return operand->Diff(var) / std::sqrt(val * val - 1);
	}
	virtual STDstring ToString() const override { return "acosh(" + operand->ToString() + ")"; }
};

class ExpressionATanh: public ExpressionBase
{
	ExpressionBase* operand;
public:
	ExpressionATanh(ExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; deleteCount++; } } }

	virtual Real Evaluate() const override { return std::atanh(operand->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		double val = operand->Evaluate();
		if (val <= -1 || val >= 1) { return std::numeric_limits<double>::quiet_NaN(); }
		return operand->Diff(var) / (1 - val * val);
	}
	virtual STDstring ToString() const override { return "atanh(" + operand->ToString() + ")"; }
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

	virtual void Destroy()
	{
		if (base) { base->DecreaseReferenceCounter(); if (base->ReferenceCounter() == 0) { base->Destroy(); delete base; deleteCount++; } }
		if (exponent) { exponent->DecreaseReferenceCounter(); if (exponent->ReferenceCounter() == 0) { exponent->Destroy(); delete exponent; deleteCount++; } }
	}

	virtual Real Evaluate() const override { return std::pow(base->Evaluate(), exponent->Evaluate()); }

	virtual double Diff(ExpressionNamedReal* var) const override 
	{
		double baseVal = base->Evaluate();
		double exponentVal = exponent->Evaluate();

		if (baseVal <= 0) { return std::numeric_limits<Real>::quiet_NaN(); }

		return exponentVal * std::pow(baseVal, exponentVal - 1) * base->Diff(var) +
			std::log(baseVal) * std::pow(baseVal, exponentVal) * exponent->Diff(var);
	}

	virtual STDstring ToString() const override { return "pow(" + base->ToString() + ", " + exponent->ToString() + ")"; }
};

class ExpressionAtan2: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionAtan2(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy()
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { arg1->Destroy(); delete arg1; deleteCount++; } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { arg2->Destroy(); delete arg2; deleteCount++; } }
	}

	virtual Real Evaluate() const override { return std::atan2(arg1->Evaluate(), arg2->Evaluate()); }
	virtual double Diff(ExpressionNamedReal* var) const override
	{
		double arg1Val = arg1->Evaluate();
		double arg2Val = arg2->Evaluate();
		double denominator = EXUstd::Square(arg1Val) + EXUstd::Square(arg2Val);
		if (denominator == 0) { return std::numeric_limits<double>::quiet_NaN(); }

		return (arg1Val * arg2->Diff(var) - arg2Val * arg1->Diff(var)) / denominator;
	}
	virtual STDstring ToString() const override { return "atan2(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...
};

class ExpressionMod: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionMod(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy()
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { arg1->Destroy(); delete arg1; deleteCount++; } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { arg2->Destroy(); delete arg2; deleteCount++; } }
	}

	virtual Real Evaluate() const override { return std::fmod(arg1->Evaluate(), arg2->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::mod::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "mod(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...
};

class ExpressionMin: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionMin(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy()
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { arg1->Destroy(); delete arg1; deleteCount++; } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { arg2->Destroy(); delete arg2; deleteCount++; } }
	}

	virtual Real Evaluate() const override { return std::min(arg1->Evaluate(), arg2->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::min::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "min(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...
};

class ExpressionMax: public ExpressionBase {
	ExpressionBase* arg1, * arg2;
public:
	ExpressionMax(ExpressionBase* baseExpr, ExpressionBase* exponentExpr)
		: arg1(baseExpr), arg2(exponentExpr) {}

	virtual void Destroy()
	{
		if (arg1) { arg1->DecreaseReferenceCounter(); if (arg1->ReferenceCounter() == 0) { arg1->Destroy(); delete arg1; deleteCount++; } }
		if (arg2) { arg2->DecreaseReferenceCounter(); if (arg2->ReferenceCounter() == 0) { arg2->Destroy(); delete arg2; deleteCount++; } }
	}

	virtual Real Evaluate() const override { return std::max(arg1->Evaluate(), arg2->Evaluate()); }
	virtual Real Diff(ExpressionNamedReal* var) const override { CHECKandTHROWstring("Symbolic::max::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override { return "max(" + arg1->ToString() + ", " + arg2->ToString() + ")"; } //use Python name ...
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

	virtual void Destroy()
	{
		if (condition) { condition->DecreaseReferenceCounter(); if (condition->ReferenceCounter() == 0) { condition->Destroy(); delete condition; deleteCount++; } }
		if (ifTrue) { ifTrue->DecreaseReferenceCounter(); if (ifTrue->ReferenceCounter() == 0) { ifTrue->Destroy(); delete ifTrue; deleteCount++; } }
		if (ifFalse) { ifFalse->DecreaseReferenceCounter(); if (ifFalse->ReferenceCounter() == 0) { ifFalse->Destroy(); delete ifFalse; deleteCount++; } }
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
	virtual Real Diff(ExpressionNamedReal* var) const override {
		Real checkCondition = condition->Evaluate();
		if (checkCondition)
		{
			return ifTrue->Diff(var);
		}
		else
		{
			return ifFalse->Diff(var);
		}
	}
	virtual STDstring ToString() const override
	{
		return "IfThenElse(" + condition->ToString() +"," + ifTrue->ToString() +"," + ifFalse->ToString() + ")";
	}
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
	virtual void Destroy()
	{
	}
	virtual Real Evaluate() const override {
		return value;
	}
	virtual Real Diff(ExpressionNamedReal* var) const override 
	{
		return this == (ExpressionReal*)var ? 1 : 0;
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
		CHECKandTHROW(IsRegularVariableName(nameInit), "ExpressionNamedReal(value, name): string must start with letter and contain only regular letters or numbers");
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
	static bool flagDebug;

	SReal() : expr(nullptr), value(0.) {}
	SReal(const Real& val) : expr(nullptr), value(val) {}
	SReal(const Index& val) : expr(nullptr), value((Real)val) {}
	SReal(ExpressionBase* e) : expr(e), value(e ? e->Evaluate() : 0) //needed???
	{
		if (e) { e->IncreaseReferenceCounter(); }
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
			"SReal::SetExpressionNamedReal expects ExpressionNamedReal in expression");
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
				expr->Destroy();
				delete expr;
				ExpressionBase::deleteCount++;
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
		CHECKandTHROW(expr == nullptr, "value can only be accessed if Real does not contain an expression");
		value = valueInit;
	}
	Real GetValue() const 
	{ 
		CHECKandTHROW(expr == nullptr, "value can only be accessed if Real does not contain an expression");
		return value;
	}

	//! set value either for named real or for internal value
	template<typename RealType>
	void SetSymbolicValue(RealType valueInit)
	{
		CHECKandTHROW((typeid(RealType) == typeid(Real)),
			"SetValue can only be called with float numbers");
		if (IsExpressionNamedReal())
		{
			GetExpressionNamedReal().SetValue((Real)valueInit);
			value = (Real)valueInit; //to make it consistent!
		}
		else
		{
			CHECKandTHROW(expr == nullptr, "SetValue can only be called for symbolic Real variables");
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
	virtual Real Diff(ExpressionNamedReal* var) const
	{
		if (expr) { return expr->Diff(var); }
		return 0.; //diff(value) = 0
	}

	virtual Real DiffSReal(const SReal& var) const
	{
		CHECKandTHROW(var.IsExpressionNamedReal(), "Symbolic::SReal::SetExpressionNamedReal expects ExpressionNamedReal in expression");

		if (expr) { return expr->Diff((ExpressionNamedReal*)var.GetExpression()); }
		return 0.; //diff(value) = 0
	}

	virtual STDstring ToString() const {
		if (expr) { return expr->ToString(); }
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
				if (expr->ReferenceCounter() == 0) { expr->Destroy(); delete expr; ExpressionBase::deleteCount++; }
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
		if (!recordExpressions) {
			value += rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorPlus(GetFunctionExpression(*this, false), //for inplace operatoins, expression is moved; no new reference!
			GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		return *this;
	}

	SReal& operator-=(const SReal& rhs) {
		if (!recordExpressions) {
			value -= rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorMinus(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		return *this;
	}

	SReal& operator*=(const SReal& rhs) {
		if (!recordExpressions) {
			value *= rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorMul(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		return *this;
	}

	SReal& operator/=(const SReal& rhs) {
		if (!recordExpressions) {
			value /= rhs.value;
			return *this;
		}
		ExpressionBase::NewCount() += 1;
		expr = new ExpressionOperatorDiv(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		expr->IncreaseReferenceCounter();
		return *this;
	}

	// Overloaded addition operator
	friend SReal operator+(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value + right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorPlus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	// Overloaded subtraction operator
	friend SReal operator-(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value - right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorMinus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	// Overloaded subtraction operator
	friend SReal operator*(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value * right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorMul(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	// Overloaded division operator
	friend SReal operator/(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
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
		if (!recordExpressions) {
			return left.value == right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator!=(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value != right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorNEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator>(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value > right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorGT(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator>=(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value >= right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorGE(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator<=(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value <= right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorLE(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SReal operator<(const SReal& left, const SReal& right)
	{
		if (!recordExpressions) {
			return left.value < right.value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionOperatorLT(GetFunctionExpression(left), GetFunctionExpression(right));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Unary plus operator
	SReal operator+() const {
		if (!recordExpressions) {
			return +value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionUnaryPlus(GetFunctionExpression(*this));
	}

	// Unary minus operator
	SReal operator-() const {
		if (!recordExpressions) {
			return -value;
		}
		ExpressionBase::NewCount()++;
		return new ExpressionUnaryMinus(GetFunctionExpression(*this));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded isfinite function
	template<typename RealType>
	static SReal isfinite(const RealType& x) {
		if (!recordExpressions) { return std::isfinite(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionIsFinite(GetFunctionExpression(x));
	}

	// Overloaded abs function
	template<typename RealType>
	static SReal abs(const RealType& x) {
		if (!recordExpressions) { return std::fabs(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionAbs(GetFunctionExpression(x));
	}

	// Overloaded sign function
	template<typename RealType>
	static SReal sign(const RealType& x) {
		if (!recordExpressions) { return EXUstd::SignReal(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSign(GetFunctionExpression(x));
	}

	// Overloaded not function; "not" is reserved in C++
	template<typename RealType>
	static SReal Not(const RealType& x) { 
		if (!recordExpressions) { return !(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionNot(GetFunctionExpression(x));
	}

	// Overloaded round function
	template<typename RealType>
	static SReal round(const RealType& x) {
		if (!recordExpressions) { return std::round(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionRound(GetFunctionExpression(x));
	}

	// Overloaded ceil function
	template<typename RealType>
	static SReal ceil(const RealType& x) {
		if (!recordExpressions) { return std::ceil(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionCeil(GetFunctionExpression(x));
	}

	// Overloaded floor function
	template<typename RealType>
	static SReal floor(const RealType& x) {
		if (!recordExpressions) { return std::floor(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionFloor(GetFunctionExpression(x));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded sqrt function
	template<typename RealType>
	static SReal sqrt(const RealType& x) {
		if (!recordExpressions) { return std::sqrt(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSqrt(GetFunctionExpression(x));
	}

	// Overloaded exp function
	template<typename RealType>
	static SReal exp(const RealType& x) {
		if (!recordExpressions) { return std::exp(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionExp(GetFunctionExpression(x));
	}

	// Overloaded log function
	template<typename RealType>
	static SReal log(const RealType& x) {
		if (!recordExpressions) { return std::log(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionLog(GetFunctionExpression(x));
	}





	//++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded sin function
	template<typename RealType>
	static SReal sin(const RealType& x) {
		if (!recordExpressions) { return std::sin(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSin(GetFunctionExpression(x));
	}

	// Overloaded cos function
	template<typename RealType>
	static SReal cos(const RealType& x) {
		if (!recordExpressions) { return std::cos(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionCos(GetFunctionExpression(x));
	}

	// Overloaded tan function
	template<typename RealType>
	static SReal tan(const RealType& x) {
		if (!recordExpressions) { return std::tan(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionTan(GetFunctionExpression(x));
	}

	// Overloaded asin function
	template<typename RealType>
	static SReal asin(const RealType& x) {
		if (!recordExpressions) { return std::asin(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionASin(GetFunctionExpression(x));
	}

	// Overloaded acos function
	template<typename RealType>
	static SReal acos(const RealType& x) {
		if (!recordExpressions) { return std::acos(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionACos(GetFunctionExpression(x));
	}

	// Overloaded atan function
	template<typename RealType>
	static SReal atan(const RealType& x) {
		if (!recordExpressions) { return std::atan(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionATan(GetFunctionExpression(x));
	}

	//++++++++++++++++++++++++++++++++++++++++++++++
	// Overloaded sin function
	template<typename RealType>
	static SReal sinh(const RealType& x) {
		if (!recordExpressions) { return std::sinh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionSinh(GetFunctionExpression(x));
	}

	// Overloaded cos function
	template<typename RealType>
	static SReal cosh(const RealType& x) {
		if (!recordExpressions) { return std::cosh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionCosh(GetFunctionExpression(x));
	}

	// Overloaded tan function
	template<typename RealType>
	static SReal tanh(const RealType& x) {
		if (!recordExpressions) { return std::tanh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionTanh(GetFunctionExpression(x));
	}

	// Overloaded asin function
	template<typename RealType>
	static SReal asinh(const RealType& x) {
		if (!recordExpressions) { return std::asinh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionASinh(GetFunctionExpression(x));
	}

	// Overloaded acos function
	template<typename RealType>
	static SReal acosh(const RealType& x) {
		if (!recordExpressions) { return std::acosh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionACosh(GetFunctionExpression(x));
	}

	// Overloaded atan function
	template<typename RealType>
	static SReal atanh(const RealType& x) {
		if (!recordExpressions) { return std::atanh(Number2Real(x)); }
		ExpressionBase::NewCount()++;
		return new ExpressionATanh(GetFunctionExpression(x));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions with 2 arguments

	// Overloaded pow function
	template<typename RealType, typename RealTypeExp>
	static SReal pow(const RealType& base, const RealTypeExp& exponent) {
		if (!recordExpressions) { return std::pow(Number2Real(base), Number2Real(exponent)); }
		ExpressionBase::NewCount()++;
		return new ExpressionPower(GetFunctionExpression(base), GetFunctionExpression(exponent));
	}

	// Overloaded atan2 function
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal atan2(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!recordExpressions) { return std::atan2(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionAtan2(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}

	// Overloaded modulo function (floating-point remainder of division)
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal mod(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!recordExpressions) { return std::fmod(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionMod(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}

	// Overloaded min function
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal min(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!recordExpressions) { return std::min(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionMin(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}

	// Overloaded max function
	template<typename RealTypeArg1, typename RealTypeArg2>
	static SReal max(const RealTypeArg1& arg1, const RealTypeArg2& arg2) {
		if (!recordExpressions) { return std::max(Number2Real(arg1), Number2Real(arg2)); }
		ExpressionBase::NewCount()++;
		return new ExpressionMax(GetFunctionExpression(arg1), GetFunctionExpression(arg2));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions with 3 arguments

	// If-then-else construct, for expressions
	template<typename RealType, typename RealTypeTrue, typename RealTypeFalse>
	static SReal IfThenElse(const RealType& condition, const RealTypeTrue& ifTrue, const RealTypeFalse& ifFalse) {
		if (!recordExpressions) {return Number2Real(condition) ? Number2Real(ifTrue) : Number2Real(ifFalse); }
		ExpressionBase::NewCount()++;
		return new ExpressionIfThenElse(GetFunctionExpression(condition), GetFunctionExpression(ifTrue), GetFunctionExpression(ifFalse));
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! variable space, available as exudyn.symbolic.variables as well as mbs.symbolic.variables
class VariableSet
{
private:
	//std::vector<SReal> variables;
	std::unordered_map<std::string, SReal> variables;
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
			PyError("VariableSet::AddVariable(symbolic.Real): only accepts named variables created as Real(value, name)");
		}

		STDstring name = sreal.GetExpressionNamedReal().GetName();
		if (HasVariable(name))
		{
			PyError("VariableSet::AddVariable(symbolic.Real): variable name already exists");
		}
		variables[name] = sreal;
	}

	//! returns true, if variable exists
	bool HasVariable(const STDstring& name)
	{
		return (variables.find(name) != variables.end());
	}

	//! returns specific variable
	const SReal& GetVariable(const STDstring& name)
	{
		auto search = variables.find(name);
		if (search == variables.end())
		{
			PyError("VariableSet::GetVariable(name): name does not exist");
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

	//! number of variables in container
	Index NumberOfItems() const { return (Index)variables.size(); }

	//! get list of variable names
	std::vector<STDstring> GetNames() const 
	{ 
		std::vector<STDstring> keys;
		keys.reserve(variables.size());

		for (auto keyValue : variables) {
			keys.push_back(keyValue.first);
		}
		return keys;
	}

	//! clear all variables
	void Reset()
	{
		variables.clear();
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! create output, which is compatible to Python dict
	virtual STDstring ToString() const
	{
		STDstring s = "{";
		STDstring sep = "";
		for (const auto& [key, value] : variables)
		{
			s+= sep + "'" + key + "': " + EXUstd::ToString(value.Evaluate());
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
