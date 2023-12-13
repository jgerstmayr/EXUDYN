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

extern bool linalgPrintUsePythonFormat; 

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

	virtual ResizableConstVector Evaluate() const
	{
		return vector;
	}
	virtual STDstring ToString() const
	{
		return vector.ToString();
	}
	virtual Index NumberOfItems() const { return vector.NumberOfItems(); }
	virtual void SetVector(const ResizableConstVector& vectorInit) { vector = vectorInit; }
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


//! class which represents a vector of expressions
//! in contrast to SReal, SymbolicRealVector stores a VectorExpressionBase*, thus needing VectorExpressionSReal to store a list of expressions
class VectorExpressionSReal: public VectorExpressionBase
{
protected:
	ExpressionList exprList; //list of expressions; if an item exists, it always refers to an expression
public:
	VectorExpressionSReal() : VectorExpressionBase() {}

	virtual ~VectorExpressionSReal()
	{
		Destroy();
	}
	//! build from initializer list, only using SReal
	VectorExpressionSReal(std::initializer_list<SReal> listOfSReals) : VectorExpressionBase()
	{
		exprList.SetNumberOfItems((Index)listOfSReals.size());
		Index cnt = 0;
		for (const auto& sreal : listOfSReals) 
		{
			ExpressionBase* expr = sreal.GetExpression();
			Real value = sreal.GetValue();
			ExpressionBase::NewCount() += (expr == 0);
			if (expr) { expr->IncreaseReferenceCounter(); }
			exprList[cnt] = expr ? expr : new ExpressionReal(sreal.GetValue());
			cnt++;
		}
	}
	VectorExpressionSReal(std::vector<SReal> listOfSReals) : VectorExpressionBase()
	{
		exprList.SetNumberOfItems((Index)listOfSReals.size());
		Index cnt = 0;
		for (const auto& sreal : listOfSReals)
		{
			ExpressionBase* expr = sreal.GetExpression();
			Real value = sreal.GetValue();
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
				item->Destroy();
				delete item;
				ExpressionBase::deleteCount++;
			}
		}
	}
	virtual ResizableConstVector Evaluate() const
	{
		ResizableConstVector result(exprList.NumberOfItems());
		for (Index i = 0; i < exprList.NumberOfItems(); i++)
		{
			result[i] = exprList.GetItem(i)->Evaluate();
		}
		return result;
	}
	virtual STDstring ToString() const
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

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; VectorExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override
	{
		ResizableConstVector leftExpr = left->Evaluate();
		ResizableConstVector rightExpr = right->Evaluate();
		CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(),
			"symbolic.Vector::operator+ : inconsistent vector sizes");
		leftExpr += rightExpr;
		return leftExpr;
		//return left->Evaluate() + right->Evaluate();
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " + " + right->ToString() + ")";
	}
};

class VectorExpressionOperatorMinus: public VectorExpressionBase
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorMinus(VectorExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; VectorExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override
	{
		ResizableConstVector leftExpr = left->Evaluate();
		ResizableConstVector rightExpr = right->Evaluate();
		CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(),
			"symbolic.Vector::operator- : inconsistent vector sizes");
		leftExpr -= rightExpr;
		return leftExpr;
		//return left->Evaluate() - right->Evaluate();
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " - " + right->ToString() + ")";
	}
};

class VectorExpressionOperatorMultScalarVector: public VectorExpressionBase
{
	ExpressionBase* left;
	VectorExpressionBase* right;
public:
	VectorExpressionOperatorMultScalarVector(ExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; ExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override
	{
		//always possible:
		return left->Evaluate() * right->Evaluate();
	}
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}
};


class VectorExpressionOperatorMultMatrix : public VectorExpressionBase
{
	MatrixExpressionBase* left;
	VectorExpressionBase* right;
public:
	VectorExpressionOperatorMultMatrix(MatrixExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; MatrixExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override
	{
		//CHECKandTHROW(left->NumberOfColumns() == right->NumberOfItems(),
		//	"symbolic.Vector::operator*(Matrix) : inconsistent matrix/vector sizes");
		ResizableConstVector result;
		EXUmath::MultMatrixVector(left->Evaluate(), right->Evaluate(), result);
		return result;
	}

	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}
};

class VectorExpressionOperatorMultMatrixTransposed : public VectorExpressionBase
{
	MatrixExpressionBase* left;
	VectorExpressionBase* right;
public:
	VectorExpressionOperatorMultMatrixTransposed(MatrixExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; MatrixExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override
	{
		//CHECKandTHROW(left->NumberOfColumns() == right->NumberOfItems(),
		//	"symbolic.Vector::operator*(Matrix) : inconsistent matrix/vector sizes");
		ResizableConstVector result;
		EXUmath::MultMatrixTransposedVector(left->Evaluate(), right->Evaluate(), result);
		return result;
	}

	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class VectorExpressionUnaryPlus: public VectorExpressionBase {
	VectorExpressionBase* operand;
public:
	VectorExpressionUnaryPlus(VectorExpressionBase* op) : VectorExpressionBase(), operand(op) {}

	virtual void Destroy()
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override {
		return operand->Evaluate();
	}
	virtual STDstring ToString() const override {
		return operand->ToString();
		//return "(+" + operand->ToString() + ")";
	}
};

class VectorExpressionUnaryMinus: public VectorExpressionBase {
	VectorExpressionBase* operand;
public:
	VectorExpressionUnaryMinus(VectorExpressionBase* op) : VectorExpressionBase(), operand(op) {}

	virtual void Destroy()
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override {
		return -1.*operand->Evaluate();
	}
	virtual STDstring ToString() const override {
		return "(-" + operand->ToString() + ")";
	}
};


class VectorExpressionMultComponents : public VectorExpressionBase
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionMultComponents(VectorExpressionBase* l, VectorExpressionBase* r) : VectorExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; VectorExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual ResizableConstVector Evaluate() const override
	{
		ResizableConstVector l = left->Evaluate();
		ResizableConstVector r = right->Evaluate();

		CHECKandTHROW(l.NumberOfItems() == r.NumberOfItems(),
			"symbolic.MultComponents(...): inconsistent vector sizes");

		ResizableConstVector result(l.NumberOfItems());
		EXUmath::MultVectorComponents(l, r, result);
		return result;
	}

	virtual STDstring ToString() const override
	{
		return left->ToString() + ".MultComponents(" + right->ToString() + ")";
	}
};





//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class VectorExpressionOperatorMultVectorVector: public ExpressionBase
{
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorMultVectorVector(VectorExpressionBase* l, VectorExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; VectorExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual Real Evaluate() const override
	{
		ResizableConstVector leftExpr = left->Evaluate();
		ResizableConstVector rightExpr = right->Evaluate();
		CHECKandTHROW(leftExpr.NumberOfItems() == rightExpr.NumberOfItems(),
			"symbolic.Vector::operator* (scalar vector multiplication) : inconsistent vector sizes");

		return leftExpr * rightExpr;

		//return left->Evaluate() * right->Evaluate();
	}
	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::operator*::Diff(vector, vector): not implemented"); return 0.; }
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}
};

class VectorExpressionNormL2: public ExpressionBase
{
	VectorExpressionBase* operand;
public:
	VectorExpressionNormL2(VectorExpressionBase* op) : ExpressionBase(), operand(op) {}

	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; ExpressionBase::deleteCount++; } } }

	virtual Real Evaluate() const override 
	{ 
		return operand->Evaluate().GetL2Norm(); 
	}
	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::Vector::NormL2(vector): not implemented"); return 0.; }
	virtual STDstring ToString() const override { return "NormL2(" + operand->ToString() + ")"; }
};


class VectorExpressionOperatorBracket: public ExpressionBase
{
	VectorExpressionBase* operand;
	ExpressionBase* index;
public:
	VectorExpressionOperatorBracket(VectorExpressionBase* op, ExpressionBase* i) : 
		ExpressionBase(), operand(op), index(i) {}

	virtual void Destroy()
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; VectorExpressionBase::deleteCount++; } }
		if (index) { index->DecreaseReferenceCounter(); if (index->ReferenceCounter() == 0) { index->Destroy(); delete index; ExpressionBase::deleteCount++; } }
	}

	virtual Real Evaluate() const override
	{
		Real iReal = index->Evaluate();
		Index i = (Index)iReal;
		CHECKandTHROW((Real)i == iReal, "VectorExpressionBase::operator[] index must be integer");
		ResizableConstVector v = operand->Evaluate(); //this is not super fast, but anyway we need to evaluate expression
		CHECKandTHROW((i >= 0) && (i < v.NumberOfItems()), "VectorExpressionBase::operator[] index out of range");
		return v[i];
	}
	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::operator[]::Diff(vector, vector): not implemented"); return 0.; }
	virtual STDstring ToString() const override
	{
		return operand->ToString() + "[" + index->ToString() + "]";
	}
};

class VectorExpressionOperatorEQ: public ExpressionBase {
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorEQ(VectorExpressionBase* l, VectorExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; VectorExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual Real Evaluate() const override
	{
		return Real(left->Evaluate() == right->Evaluate());
	}
	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::operator==::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " == " + right->ToString() + ")";
	}
};

class VectorExpressionOperatorNEQ: public ExpressionBase {
	VectorExpressionBase* left, * right;
public:
	VectorExpressionOperatorNEQ(VectorExpressionBase* l, VectorExpressionBase* r) : ExpressionBase(), left(l), right(r) {}

	virtual void Destroy()
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; VectorExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; VectorExpressionBase::deleteCount++; } }
	}

	virtual Real Evaluate() const override {
		return Real(!(left->Evaluate() == right->Evaluate()));
	}
	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::operator!=::Diff(...): not possible"); return 0.; }
	virtual STDstring ToString() const override {
		return "(" + left->ToString() + " != " + right->ToString() + ")";
	}
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
	SymbolicRealVector(const STDstring& name, const Vector& vectorInit) : vector(vectorInit), exprList(nullptr)
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
			exprList->IncreaseReferenceCounter();

		}

	} 
	SymbolicRealVector(VectorExpressionBase* e) : exprList(e), vector(e ? e->Evaluate() : 0)
	{
		if (e) { e->IncreaseReferenceCounter(); }
	}
	SymbolicRealVector(const SymbolicRealVector& other) : vector(other.vector), exprList(other.exprList)
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
			exprList->IncreaseReferenceCounter();
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

	void SetSymbolicVector(const ResizableConstVector& vectorInit)
	{
		if (IsExpressionNamedReal())
		{
			GetExpressionNamedReal().SetVector(vectorInit);
		}
		else
		{
			CHECKandTHROW(exprList == nullptr, "SymbolicRealVector::SetValue can only be called for symbolic Real variables");
			vector = vectorInit;
		}
	}

	//! assume that expression e holds a NamedReal and override its value
	void SetSymbolicVectorComponent(Index index, Real value)
	{
		if (IsExpressionNamedReal())
		{
			CHECKandTHROW(GetExpressionNamedReal().NumberOfItems() <= index,
				"SymbolicRealVector::SetExpressionNamedVectorComponent: index out of range");
			GetExpressionNamedReal().SetComponent(index, value);
		}
		else
		{
			CHECKandTHROW(exprList == nullptr, "SymbolicRealVector::SetVector can only be called for symbolic Vector variables");
			CHECKandTHROW(vector.NumberOfItems() <= index,
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

	//! for Python usage: return numpy array:
	//! 
	virtual py::array_t<Real> PyEvaluate() const {
		if (exprList) { return EPyUtils::Vector2NumPy(exprList->Evaluate()); }
		return EPyUtils::Vector2NumPy(vector);
	}

	virtual Index NumberOfItems() const {
		if (exprList) { return exprList->Evaluate().NumberOfItems(); }
		return vector.NumberOfItems();
	}

	virtual STDstring ToString() const {
		if (exprList) { return exprList->ToString(); }
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
				exprList->Destroy();
				delete exprList;
				VectorExpressionBase::deleteCount++;
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
				if (exprList->ReferenceCounter() == 0) { exprList->Destroy(); delete exprList; VectorExpressionBase::deleteCount++; }
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
	static VectorExpressionBase* GetFunctionExpression(const SymbolicRealVector& x)
	{
		VectorExpressionBase::NewCount() += (x.exprList == 0);
		if (x.exprList) { x.exprList->IncreaseReferenceCounter(); }
		return x.exprList ? x.exprList : new VectorExpressionReal(x.vector);
	}
	static VectorExpressionBase* GetFunctionExpression(const Vector& x)
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
		exprList = new VectorExpressionOperatorPlus(GetFunctionExpression(*this), GetFunctionExpression(rhs));
		exprList->IncreaseReferenceCounter();
		return *this;
	}

	SymbolicRealVector& operator-=(const SymbolicRealVector& rhs) {
		if (!SReal::recordExpressions) {
			vector -= rhs.vector;
			return *this;
		}
		VectorExpressionBase::NewCount() += 1;
		exprList = new VectorExpressionOperatorMinus(GetFunctionExpression(*this), GetFunctionExpression(rhs));
		exprList->IncreaseReferenceCounter();
		return *this;
	}

	SymbolicRealVector& operator*=(const SReal& rhs) {
		if (!SReal::recordExpressions) {
			vector *= rhs.GetValue();
			return *this;
		}
		VectorExpressionBase::NewCount() += 1;
		exprList = new VectorExpressionOperatorMultScalarVector(GetFunctionExpressionSReal(rhs), GetFunctionExpression(*this));
		exprList->IncreaseReferenceCounter();
		return *this;
	}


	SymbolicRealVector operator-() const
	{
		if (!SReal::recordExpressions) {
			return -1.*vector;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionUnaryMinus(GetFunctionExpression(*this));
	}

	SymbolicRealVector operator+() const
	{
		if (!SReal::recordExpressions) {
			return vector;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionUnaryPlus(GetFunctionExpression(*this));
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
		return new VectorExpressionMultComponents(GetFunctionExpression(*this), GetFunctionExpression(right));
	}

	SReal NormL2() const
	{
		if (!SReal::recordExpressions) {
			return vector.GetL2Norm();
		}
		ExpressionBase::NewCount()++;
		return new VectorExpressionNormL2(GetFunctionExpression(*this));
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
		return new VectorExpressionOperatorBracket(GetFunctionExpression(*this), 
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
