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
#ifndef SYMBOLICMATRIX__H
#define SYMBOLICMATRIX__H

namespace Symbolic
{


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef ResizableMatrix ResizableConstMatrix; //in future, will be more efficient type
typedef std::vector<std::vector<ExpressionBase*> > ExpressionArray;


//! class which represents a matrix of expressions
class MatrixExpressionReal : public MatrixExpressionBase
{
protected:
	ResizableMatrix matrix;
public:
	MatrixExpressionReal(const Matrix& matrixInit) : matrix(matrixInit) 
	{
		IncreaseReferenceCounter(); //is referenced immediately, otherwise will lead to negative reference
	}
	virtual void Destroy() override {}

	virtual ResizableMatrix Evaluate() const override
	{
		return matrix;
	}
	virtual Real EvaluateComponent(Index row, Index column) const override
	{ 
		CHECKandTHROW(row < matrix.NumberOfRows() && column < matrix.NumberOfColumns(), "MatrixExpressionReal: invalid row/column");
		return matrix(row, column); 
	};
	virtual STDstring ToString() const override
	{
		return matrix.ToString();
	}
	virtual Index NumberOfRows() const override { return matrix.NumberOfRows(); }
	virtual Index NumberOfColumns() const override { return matrix.NumberOfColumns(); }
	virtual void SetMatrix(const ResizableMatrix& matrixInit) { matrix = matrixInit; }
	virtual void SetComponent(Index row, Index column, Real value) { matrix(row, column) = value; }
	virtual const ResizableMatrix& GetMatrix() const { return matrix; }
	//! write access to override existing matrix
	virtual ResizableMatrix& GetMatrix() { return matrix; }
};

class MatrixExpressionNamedReal : public MatrixExpressionReal
{
	STDstring name;
public:
	MatrixExpressionNamedReal(const Matrix& matrixInit, const STDstring& nameInit = "") : MatrixExpressionReal(matrixInit), name(nameInit)
	{
		CHECKandTHROW(ExpressionBase::IsRegularVariableName(nameInit), "MatrixExpressionNamedmatrix(matrix, name): string must start with letter and contain only regular letters or numbers");
	}

	virtual STDstring ToString() const override
	{
		return name; //returns name of variable ...
	}

	virtual void SetName(const STDstring& nameInit) { name = nameInit; }
	virtual const STDstring& GetName() const { return name; }
};


//! class which represents a matrix of expressions
//! in contrast to SReal, SymbolicRealVector stores a MatrixExpressionBase*, thus needing MatrixExpressionSReal to store a list of expressions
class MatrixExpressionSReal : public MatrixExpressionBase
{
protected:
	ExpressionArray exprMatrix; //list of expressions; if an item exists, it always refers to an expression
public:
	MatrixExpressionSReal() : MatrixExpressionBase() {}

	MatrixExpressionSReal(std::vector < std::vector<SReal> > matrixOfSReals) : MatrixExpressionBase()
	{
		Index rows = (Index)matrixOfSReals.size();
		Index columns = (Index)matrixOfSReals[0].size(); //all columns must be same

		exprMatrix.assign(rows, std::vector <Symbolic::ExpressionBase*>(columns, nullptr));
		Index row = 0;
		for (const auto& sVector: matrixOfSReals)
		{
			Index column = 0;
			for (const auto& sreal : sVector)
			{
				ExpressionBase* expr = sreal.GetExpression();
				Real value = sreal.GetValue();
				ExpressionBase::NewCount() += (expr == 0);
				if (expr) { expr->IncreaseReferenceCounter(); }
				exprMatrix[row][column] = expr ? expr : new ExpressionReal(sreal.GetValue());
				column++;
			}
			row++;
		}
	}

	Index NumberOfRows() const override { return (Index)exprMatrix.size(); }
	Index NumberOfColumns() const override
	{ 
		return NumberOfRows() ? (Index)exprMatrix[0].size() : 0;
	}

	//! set sizes and reset
	void SetNumberOfRowsAndColumns(Index rows, Index columns)
	{
		Destroy();
		exprMatrix.assign(rows, std::vector <Symbolic::ExpressionBase*>(columns, nullptr));
	}

	//! set a particular component of matrix; used in SymbolicRealMatrix py::list constructor
	void SetSReal(Index row, Index column, const SReal& value)
	{
		ExpressionBase* expr = value.GetExpression();
		ExpressionBase::NewCount() += (expr == 0);
		if (expr) { expr->IncreaseReferenceCounter(); }
		exprMatrix[row][column] = expr ? expr : new ExpressionReal(value.GetValue());
	}
	virtual void Destroy() override
	{
		Index row = 0;
		for (const auto& sVector : exprMatrix)
		{
			for (const auto& item : sVector)
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
	}
	virtual ResizableMatrix Evaluate() const override
	{
		ResizableMatrix result(NumberOfRows(), NumberOfColumns());
		Index row = 0;
		for (const auto& sVector : exprMatrix)
		{
			Index column = 0;
			for (const auto& sreal : sVector)
			{
				result(row, column) = sreal->Evaluate();

				column++;
			}
			row++;
		}

		return result;
	}
	virtual Real EvaluateComponent(Index row, Index column) const override
	{ 
		CHECKandTHROW(row < exprMatrix.size() && column < exprMatrix[0].size(), "MatrixExpressionSReal: invalid row/column");
		return exprMatrix[row][column]->Evaluate();
	};
	virtual STDstring ToString() const override
	{
		STDstring str = "[";
		char sep = ' ';
		if (linalgPrintUsePythonFormat) { sep = ','; }

		STDstring sepRow = "";
		for (const auto& sVector : exprMatrix)
		{
			STDstring sepCol = "";
			str += sepRow;
			sepRow = ",\n";
			str += "[";
			for (const auto& sreal : sVector)
			{
				str += sepCol;
				sepCol = sep;
				str += sreal->ToString();
			}
			str += "]";
		}
		return str + "]";
	}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class MatrixExpressionOperatorPlus : public MatrixExpressionBase
{
	MatrixExpressionBase* left, * right;
public:
	MatrixExpressionOperatorPlus(MatrixExpressionBase* l, MatrixExpressionBase* r) : MatrixExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; MatrixExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; MatrixExpressionBase::deleteCount++; } }
	}

	//! get sizes
	virtual Index NumberOfRows() const override
	{ 
		Index leftRows = left->NumberOfRows();
		CHECKandTHROW(leftRows == right->NumberOfRows(), "symbolic.Matrix::operator+ : inconsistent number of rows");
		return leftRows;
	}
	virtual Index NumberOfColumns() const override
	{ 
		Index leftColumns = left->NumberOfColumns();
		CHECKandTHROW(leftColumns == right->NumberOfColumns(), "symbolic.Matrix::operator+ : inconsistent number of columns");
		return leftColumns;
	}

	virtual ResizableMatrix Evaluate() const override
	{
		ResizableMatrix leftExpr = left->Evaluate();
		ResizableMatrix rightExpr = right->Evaluate();
		CHECKandTHROW( (leftExpr.NumberOfRows() == rightExpr.NumberOfRows()) && (leftExpr.NumberOfColumns() == rightExpr.NumberOfColumns()),
			"symbolic.Matrix::operator+ : inconsistent matrix sizes");
		leftExpr += rightExpr;
		return leftExpr;
	}

	virtual Real EvaluateComponent(Index row, Index column) const override
	{
		CHECKandTHROW((row < left->NumberOfRows()) && (column < left->NumberOfColumns()) &&
			(row < right->NumberOfRows()) && (column < right->NumberOfColumns()),
			"symbolic.Matrix::operator+: invalid row/column");
		return left->EvaluateComponent(row,column) + right->EvaluateComponent(row, column);
	};

	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " + " + right->ToString() + ")";
	}
};

class MatrixExpressionOperatorMinus : public MatrixExpressionBase
{
	MatrixExpressionBase* left, * right;
public:
	MatrixExpressionOperatorMinus(MatrixExpressionBase* l, MatrixExpressionBase* r) : MatrixExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; MatrixExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; MatrixExpressionBase::deleteCount++; } }
	}

	//! get sizes
	virtual Index NumberOfRows() const override
	{
		Index leftRows = left->NumberOfRows();
		CHECKandTHROW(leftRows == right->NumberOfRows(), "symbolic.Matrix::operator+ : inconsistent number of rows");
		return leftRows;
	}
	virtual Index NumberOfColumns() const override
	{
		Index leftColumns = left->NumberOfColumns();
		CHECKandTHROW(leftColumns == right->NumberOfColumns(), "symbolic.Matrix::operator+ : inconsistent number of columns");
		return leftColumns;
	}

	virtual ResizableMatrix Evaluate() const override
	{
		ResizableMatrix leftExpr = left->Evaluate();
		ResizableMatrix rightExpr = right->Evaluate();
		CHECKandTHROW((leftExpr.NumberOfRows() == rightExpr.NumberOfRows()) && (leftExpr.NumberOfColumns() == rightExpr.NumberOfColumns()),
			"symbolic.Matrix::operator- : inconsistent matrix sizes");
		leftExpr -= rightExpr;
		return leftExpr;
		//return left->Evaluate() - right->Evaluate();
	}
	virtual Real EvaluateComponent(Index row, Index column) const override
	{
		CHECKandTHROW((row < left->NumberOfRows()) && (column < left->NumberOfColumns()) &&
			(row < right->NumberOfRows()) && (column < right->NumberOfColumns()),
			"symbolic.Matrix::operator-: invalid row/column");
		return left->EvaluateComponent(row, column) - right->EvaluateComponent(row, column);
	};

	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " - " + right->ToString() + ")";
	}
};

class MatrixExpressionOperatorMultScalarMatrix : public MatrixExpressionBase
{
	ExpressionBase* left;
	MatrixExpressionBase* right;
public:
	MatrixExpressionOperatorMultScalarMatrix(ExpressionBase* l, MatrixExpressionBase* r) : MatrixExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; ExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; MatrixExpressionBase::deleteCount++; } }
	}

	//! get sizes
	virtual Index NumberOfRows() const override
	{
		return right->NumberOfRows();
	}
	virtual Index NumberOfColumns() const override
	{
		return right->NumberOfColumns();
	}

	virtual ResizableMatrix Evaluate() const override
	{
		//always possible:
		return left->Evaluate() * right->Evaluate();
	}
	virtual Real EvaluateComponent(Index row, Index column) const override
	{
		CHECKandTHROW((row < right->NumberOfRows()) && (column < right->NumberOfColumns()),
			"symbolic.Matrix::operator* (scalar): invalid row/column");
		return left->Evaluate() * right->EvaluateComponent(row, column);
	};

	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class MatrixExpressionUnaryPlus : public MatrixExpressionBase {
	MatrixExpressionBase* operand;
public:
	MatrixExpressionUnaryPlus(MatrixExpressionBase* op) : MatrixExpressionBase(), operand(op) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; MatrixExpressionBase::deleteCount++; } }
	}

	//! get sizes
	virtual Index NumberOfRows() const override
	{
		return operand->NumberOfRows();
	}
	virtual Index NumberOfColumns() const override
	{
		return operand->NumberOfColumns();
	}

	virtual ResizableMatrix Evaluate() const override {
		return operand->Evaluate();
	}
	virtual Real EvaluateComponent(Index row, Index column) const override
	{
		CHECKandTHROW(row < operand->NumberOfRows() && column < operand->NumberOfColumns(),
			"symbolic.Matrix::UnaryPlus: invalid row/column");
		return operand->EvaluateComponent(row, column);
	};

	virtual STDstring ToString() const override {
		return operand->ToString();
	}
};

class MatrixExpressionUnaryMinus : public MatrixExpressionBase {
	MatrixExpressionBase* operand;
public:
	MatrixExpressionUnaryMinus(MatrixExpressionBase* op) : MatrixExpressionBase(), operand(op) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; MatrixExpressionBase::deleteCount++; } }
	}

	//! get sizes
	virtual Index NumberOfRows() const override
	{
		return operand->NumberOfRows();
	}
	virtual Index NumberOfColumns() const override
	{
		return operand->NumberOfColumns();
	}

	virtual ResizableMatrix Evaluate() const override {
		return -1. * operand->Evaluate();
	}
	virtual Real EvaluateComponent(Index row, Index column) const override
	{
		CHECKandTHROW((row < operand->NumberOfRows()) && (column < operand->NumberOfColumns()),
			"symbolic.Matrix::UnaryMinus: invalid row/column");
		return -operand->EvaluateComponent(row, column);
	};
	virtual STDstring ToString() const override {
		return "(-" + operand->ToString() + ")";
	}
};


//class MatrixExpressionMultComponents : public MatrixExpressionBase
//{
//	MatrixExpressionBase* left, * right;
//public:
//	MatrixExpressionMultComponents(MatrixExpressionBase* l, MatrixExpressionBase* r) : MatrixExpressionBase(), left(l), right(r) {}
//
//	virtual void Destroy()
//	{
//		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; MatrixExpressionBase::deleteCount++; } }
//		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; MatrixExpressionBase::deleteCount++; } }
//	}
//
//	virtual ResizableMatrix Evaluate() const override
//	{
//		ResizableMatrix l = left->Evaluate();
//		ResizableMatrix r = right->Evaluate();
//
//		CHECKandTHROW(l.NumberOfItems() == r.NumberOfItems(),
//			"symbolic.MultComponents(...): inconsistent matrix sizes");
//
//		ResizableMatrix result(l.NumberOfItems());
//		EXUmath::MultMatrixComponents(l, r, result);
//		return result;
//	}
//
//	virtual STDstring ToString() const override
//	{
//		return left->ToString() + ".MultComponents(" + right->ToString() + ")";
//	}
//};





//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class MatrixExpressionOperatorMultMatrixMatrix : public MatrixExpressionBase
{
	MatrixExpressionBase* left, * right;
public:
	MatrixExpressionOperatorMultMatrixMatrix(MatrixExpressionBase* l, MatrixExpressionBase* r) : MatrixExpressionBase(), left(l), right(r) {}

	virtual void Destroy() override
	{
		if (left) { left->DecreaseReferenceCounter(); if (left->ReferenceCounter() == 0) { left->Destroy(); delete left; MatrixExpressionBase::deleteCount++; } }
		if (right) { right->DecreaseReferenceCounter(); if (right->ReferenceCounter() == 0) { right->Destroy(); delete right; MatrixExpressionBase::deleteCount++; } }
	}

	//! get sizes
	virtual Index NumberOfRows() const override
	{
		return left->NumberOfRows();
	}
	virtual Index NumberOfColumns() const override
	{
		return right->NumberOfColumns();
	}

	virtual ResizableMatrix Evaluate() const override
	{
		ResizableMatrix leftExpr = left->Evaluate();
		ResizableMatrix rightExpr = right->Evaluate();
		CHECKandTHROW((leftExpr.NumberOfColumns() == rightExpr.NumberOfRows()),
			"symbolic.Matrix::operator* (scalar matrix multiplication) : inconsistent matrix sizes");

		return leftExpr * rightExpr;
		//return left->Evaluate() * right->Evaluate();
	}
	virtual Real EvaluateComponent(Index row, Index column) const override
	{
		Index nSum = left->NumberOfColumns();
		CHECKandTHROW((row < left->NumberOfRows()) && (column < right->NumberOfColumns()) && (nSum == row < right->NumberOfRows()),
			"symbolic.Matrix::operator* (matrix): invalid row/column");
		Real result = 0;

		//this may be slower or faster than whole multiplication
		for (Index i = 0; i < nSum; i++)
		{
			result += left->EvaluateComponent(row, i) * right->EvaluateComponent(i, column);
		}
		return result;
	};

	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::operator*::Diff(matrix, matrix): not implemented"); return 0.; }
	virtual STDstring ToString() const override
	{
		return "(" + left->ToString() + " * " + right->ToString() + ")";
	}
};

//class MatrixExpressionNormL2 : public ExpressionBase
//{
//	MatrixExpressionBase* operand;
//public:
//	MatrixExpressionNormL2(MatrixExpressionBase* op) : ExpressionBase(), operand(op) {}
//
//	virtual void Destroy() { if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; MatrixExpressionBase::deleteCount++; } } }
//
//	virtual Real Evaluate() const override
//	{
//		return operand->Evaluate().GetL2Norm();
//	}
//	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::Matrix::NormL2(matrix): not implemented"); return 0.; }
//	virtual STDstring ToString() const override { return "NormL2(" + operand->ToString() + ")"; }
//};
//

class MatrixExpressionOperatorBracket : public ExpressionBase
{
	MatrixExpressionBase* operand;
	ExpressionBase* indexRow;
	ExpressionBase* indexColumn;
public:
	MatrixExpressionOperatorBracket(MatrixExpressionBase* op, ExpressionBase* row, ExpressionBase* column) :
		ExpressionBase(), operand(op), indexRow(row), indexColumn(column) {}

	virtual void Destroy() override
	{
		if (operand) { operand->DecreaseReferenceCounter(); if (operand->ReferenceCounter() == 0) { operand->Destroy(); delete operand; MatrixExpressionBase::deleteCount++; } }
		if (indexRow) { indexRow->DecreaseReferenceCounter(); if (indexRow->ReferenceCounter() == 0) { indexRow->Destroy(); delete indexRow; ExpressionBase::deleteCount++; } }
		if (indexColumn) { indexColumn->DecreaseReferenceCounter(); if (indexColumn->ReferenceCounter() == 0) { indexColumn->Destroy(); delete indexColumn; ExpressionBase::deleteCount++; } }
	}

	//! get sizes
	//virtual Index NumberOfRows() const
	//{
	//	return operand->NumberOfRows();
	//}
	//virtual Index NumberOfColumns() const
	//{
	//	return operand->NumberOfColumns();
	//}

	virtual Real Evaluate() const override
	{
		Real rowReal = indexRow->Evaluate();
		Index row = (Index)rowReal;
		Real columnReal = indexColumn->Evaluate();
		Index column = (Index)columnReal;

		CHECKandTHROW(((Real)row == rowReal) && ((Real)column == columnReal), "MatrixExpressionBase::operator[] index must contain integer indices");
		//ResizableMatrix m = operand->Evaluate(); //this is not super fast, but anyway we need to evaluate expression
		//CHECKandTHROW((row >= 0) && (row < m.NumberOfRows()) && 
		//	(column >= 0) && (column < m.NumberOfColumns()), "MatrixExpressionBase::operator[] index out of range");
		//return m(row,column);
		return operand->EvaluateComponent(row, column);
	}
	virtual Real Diff(ExpressionNamedReal* var) const { CHECKandTHROWstring("Symbolic::operator[]::Diff(matrix, matrix): not implemented"); return 0.; }
	virtual STDstring ToString() const override
	{
		return operand->ToString() + "[" + indexRow->ToString() +"," + indexColumn->ToString() + "]";
	}
};














//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! symbolic matrix, e.g., for use in pythonUserFunctions
class SymbolicRealMatrix
{
	MatrixExpressionBase* exprMatrix;
	ResizableConstMatrix matrix;
public:
	SymbolicRealMatrix() : exprMatrix(nullptr) {}

	//! cast from Matrix; for internal usage:
	SymbolicRealMatrix(const Matrix& matrixInit) : exprMatrix(nullptr), matrix(matrixInit) {}
	//! cast from name and Matrix; for internal usage:
	SymbolicRealMatrix(const STDstring& name, const Matrix& matrixInit) : matrix(matrixInit), exprMatrix(nullptr)
	{
		if (SReal::recordExpressions) {
			MatrixExpressionBase::NewCount()++;
			exprMatrix = new MatrixExpressionNamedReal(matrixInit, name);
		}
	}

	SymbolicRealMatrix(const py::array_t<Real>& matrixInit) : exprMatrix(nullptr)
	{
		EPyUtils::NumPy2Matrix(matrixInit, matrix);
		//pout << "v=" << matrix << "\n";
	}

	SymbolicRealMatrix(const STDstring& name, const py::array_t<Real>& matrixInit) : exprMatrix(nullptr)
	{
		EPyUtils::NumPy2Matrix(matrixInit, matrix);
		if (SReal::recordExpressions) {
			MatrixExpressionBase::newCount += 1 + (exprMatrix == 0);
			exprMatrix = new MatrixExpressionNamedReal(matrix, name);
		}
	}



	SymbolicRealMatrix(py::list matrixInit)
	{
		Index rows = (Index)matrixInit.size();
		if (rows == 0)
		{
			PyError(STDstring("SymbolicRealMatrix init received list with size zero"));
		}

		if (!py::isinstance<py::list>(matrixInit[0]))
		{
			PyError(STDstring("SymbolicRealMatrix init with Python list: expected list of lists, but received:\n") + EXUstd::ToString(matrixInit));
		}
		py::list column0 = py::cast<py::list>(matrixInit[0]);
		Index columns = (Index)column0.size();

		//check if any list item is symbolic => needs Evaluate()
		bool isSymbolic = false;
		Index row = 0;
		for (const auto& columnObject : matrixInit)
		{
			py::list columnVector = py::cast<py::list>(columnObject);
			Index column = 0;
			for (const auto& item : columnVector)
			{
				if (py::isinstance<SReal>(item))
				{
					isSymbolic = true;
					if (GetFlagDebug()) { pout << "SymbolicRealMatrix[" << row << "," << column << "] is symbolic\n"; }
					column++;
				}
			}
			row++;
		}

		if (!isSymbolic || !SReal::recordExpressions)
		{
			exprMatrix = nullptr;
			matrix.SetNumberOfRowsAndColumns(rows, columns);
			Index row = 0;
			for (const auto& columnObject : matrixInit)
			{
				py::list columnVector = py::cast<py::list>(columnObject);
				Index column = 0;
				for (const auto& item : columnVector)
				{
					matrix(row, column) = py::cast<Real>(item);
					column++;
				}
				row++;
			}
		}
		else
		{
			MatrixExpressionBase::NewCount()++;
			MatrixExpressionSReal* vReal = new MatrixExpressionSReal();
			exprMatrix = vReal;
			vReal->SetNumberOfRowsAndColumns(rows, columns);

			Index row = 0;
			for (const auto& columnObject : matrixInit)
			{
				py::list columnVector = py::cast<py::list>(columnObject);
				Index column = 0;
				for (const auto& item : columnVector)
				{
					if (py::isinstance<SReal>(item))
					{
						vReal->SetSReal(row, column, py::cast<SReal>(item));
					}
					else
					{
						Real value = py::cast<Real>(item);
						vReal->SetSReal(row, column, SReal(value));
					}
					column++;
				}
				row++;
			}
			exprMatrix->IncreaseReferenceCounter();

		}

	}
	SymbolicRealMatrix(MatrixExpressionBase* e) : exprMatrix(e)
	{
		if (e) 
		{ 
			matrix = e->Evaluate();
			e->IncreaseReferenceCounter(); 
		}
		else
		{
			matrix = Matrix();
		}
	}
	SymbolicRealMatrix(const SymbolicRealMatrix& other) : matrix(other.matrix), exprMatrix(other.exprMatrix)
	{
		if (GetFlagDebug()) { std::cout << "copy constructor: " << ToString() << "\n"; }
		if (exprMatrix) { exprMatrix->IncreaseReferenceCounter(); }
	}

	//! check if there is a ExpressionNamedMatrix in expr 
	bool IsExpressionNamedReal()  const
	{
		if (exprMatrix == nullptr) { return false; }

		return (typeid(*exprMatrix) == typeid(MatrixExpressionNamedReal));
		//if (dynamic_cast<MatrixExpressionNamedReal*>(exprMatrix) != nullptr)
		//{
		//	return true;
		//}
		//return false;
	}
	MatrixExpressionNamedReal& GetExpressionNamedReal() 
	{ 
		CHECKandTHROW(IsExpressionNamedReal(), "SymbolicRealMatrix::GetExpressionNamedReal expects MatrixExpressionNamedReal in expression");
		return (MatrixExpressionNamedReal&)(*exprMatrix);
	}
	const MatrixExpressionNamedReal& GetExpressionNamedReal() const 
	{ 
		CHECKandTHROW(IsExpressionNamedReal(), "SymbolicRealMatrix::GetExpressionNamedReal (const) expects MatrixExpressionNamedReal in expression");
		return (const MatrixExpressionNamedReal&)(*exprMatrix);
	}

	//! assume that expression e holds a NamedReal and override its value
	void SetExpressionNamedMatrix(const ResizableConstMatrix& matrixInit)
	{
		CHECKandTHROW(IsExpressionNamedReal(), "SymbolicRealMatrix::SetExpressionNamedMatrix expects MatrixExpressionNamedReal in expression");
		GetExpressionNamedReal().SetMatrix(matrixInit);
	}

	//! set with numpy matrix
	void SetSymbolicMatrix(const py::array_t<Real>& matrixInit)
	{
		ResizableConstMatrix temp;
		EPyUtils::NumPy2Matrix(matrixInit, temp);
		if (IsExpressionNamedReal())
		{
			GetExpressionNamedReal().SetMatrix(temp);
		}
		else
		{
			CHECKandTHROW(exprMatrix == nullptr, "SymbolicRealMatrix::SetValue can only be called for symbolic Real variables");
			matrix = temp;
		}
	}

	//! assume that expression e holds a NamedReal and override its value
	void SetSymbolicMatrixComponent(Index row, Index column, Real value)
	{
		if (IsExpressionNamedReal())
		{
			CHECKandTHROW(row < GetExpressionNamedReal().NumberOfRows(),
				"SymbolicRealMatrix::SetExpressionNamedMatrixComponent: row out of range");
			CHECKandTHROW(column < GetExpressionNamedReal().NumberOfColumns(),
				"SymbolicRealMatrix::SetExpressionNamedMatrixComponent: column out of range");
			GetExpressionNamedReal().SetComponent(row, column, value);
		}
		else
		{
			CHECKandTHROW(exprMatrix == nullptr, "SymbolicRealMatrix::SetMatrix can only be called for symbolic Matrix variables");
			CHECKandTHROW(row < matrix.NumberOfRows(), "SymbolicRealMatrix::SetExpressionNamedMatrixComponent: row out of range");
			CHECKandTHROW(column < matrix.NumberOfColumns(), "SymbolicRealMatrix::SetExpressionNamedMatrixComponent: column out of range");
			matrix(row, column) = value;
		}
	}

	//virtual void SetComponent(Index i, Real value) { matrix[i] = value; }

	static void SetFlagDebug(bool flag) { SReal::flagDebug = flag; }
	static bool GetFlagDebug() { return SReal::flagDebug; }
	void SetMatrix(const ResizableConstMatrix& matrixInit) { matrix = matrixInit; }
	const ResizableConstMatrix& GetMatrix() const { return matrix; }
	ResizableConstMatrix& GetMatrix() { return matrix; }

	void SetExpression(MatrixExpressionBase* e) { exprMatrix = e; if (e) { e->IncreaseReferenceCounter(); } }
	MatrixExpressionBase* GetExpression() const { return exprMatrix; }

	//! for C++ usage: return Matrix
	virtual ResizableConstMatrix Evaluate() const {
		if (exprMatrix) { return exprMatrix->Evaluate(); }
		return matrix;
	}

	//! for Python usage: return numpy array:
	//! 
	virtual py::array_t<Real> PyEvaluate() const {
		if (exprMatrix) { return EPyUtils::Matrix2NumPy(exprMatrix->Evaluate()); }
		return EPyUtils::Matrix2NumPy(matrix);
	}

	virtual Index NumberOfColumns() const {
		if (exprMatrix) { return exprMatrix->Evaluate().NumberOfColumns(); }
		return matrix.NumberOfColumns();
	}

	virtual Index NumberOfRows() const {
		if (exprMatrix) { return exprMatrix->Evaluate().NumberOfRows(); }
		return matrix.NumberOfRows();
	}

	virtual STDstring ToString() const {
		if (exprMatrix) { return exprMatrix->ToString(); }
		return matrix.ToString();
	}

	virtual ~SymbolicRealMatrix()
	{
		Destroy();
	}
	//! destroy expression tree, before overriding by new tree
	void Destroy()
	{
		if (exprMatrix)
		{
			exprMatrix->DecreaseReferenceCounter();
			if (exprMatrix->ReferenceCounter() == 0)
			{
				if (GetFlagDebug()) { std::cout << "now we delete: " << ToString() << "\n"; }
				exprMatrix->Destroy();
				delete exprMatrix;
				MatrixExpressionBase::deleteCount++;
			}
		}
	}

	// Modified implicit conversion to ResizableConstMatrix
	explicit operator ResizableConstMatrix() const
	{
		return Evaluate();
	}

	// Modified implicit conversion to Matrix
	explicit operator Matrix() const
	{
		return Evaluate();
	}

	//virtual STDstring ToString() const {
	//	if (exprMatrix.NumberOfItems())
	//	{
	//		STDstring s = "[";
	//		STDstring sep = "";
	//		for (Index i = 0; i < exprMatrix.NumberOfItems(); i++)
	//		{
	//			s += sep + exprMatrix[i]->ToString();
	//		}
	//		s += "]";
	//		return s;
	//	}

	//	return matrix.ToString();
	//}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	SymbolicRealMatrix& operator=(const SymbolicRealMatrix& other)
	{
		if (this != &other) // Protect against self-assignment
		{
			if (exprMatrix) //if expression is not used any more, destroy!
			{
				exprMatrix->DecreaseReferenceCounter();
				if (exprMatrix->ReferenceCounter() == 0) { exprMatrix->Destroy(); delete exprMatrix; MatrixExpressionBase::deleteCount++; }
			}

			matrix = other.matrix;
			exprMatrix = other.exprMatrix;
			if (exprMatrix) { exprMatrix->IncreaseReferenceCounter(); }

			if (GetFlagDebug()) { std::cout << "**********\nassign matrix\n**********\n"; }
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

	//! Get Expression* either for SymbolicRealMatrix or Matrix
	static MatrixExpressionBase* GetFunctionExpression(const SymbolicRealMatrix& x, bool increaseReferenceCounter = true)
	{
		MatrixExpressionBase::newCount += (x.exprMatrix == 0);
		if (x.exprMatrix && increaseReferenceCounter) { x.exprMatrix->IncreaseReferenceCounter(); }
		return x.exprMatrix ? x.exprMatrix : new MatrixExpressionReal(x.matrix);
	}
	static MatrixExpressionBase* GetFunctionExpression(const Matrix& x, bool increaseReferenceCounter = true)
	{
		MatrixExpressionBase::newCount += 1;
		return new MatrixExpressionReal(x);
	}

	//for SReal * Matrix operations ...
	//! Get Expression* either for SReal or Real
	static ExpressionBase* GetFunctionExpressionSReal(const SReal& x)
	{
		ExpressionBase::newCount += (x.GetExpression() == 0);
		if (x.GetExpression()) { x.GetExpression()->IncreaseReferenceCounter(); }
		return x.GetExpression() ? x.GetExpression() : new ExpressionReal(x.GetValue());
	}
	static ExpressionBase* GetFunctionExpressionSReal(Real x)
	{
		ExpressionBase::newCount += 1;
		return new ExpressionReal(x);
	}
	//! Get Expression* either for SymbolicRealVector or Vector
	static VectorExpressionBase* GetFunctionExpressionVector(const SymbolicRealVector& x)
	{
		VectorExpressionBase::newCount += (x.GetExpression() == 0);
		if (x.GetExpression()) { x.GetExpression()->IncreaseReferenceCounter(); }
		return x.GetExpression() ? x.GetExpression() : new VectorExpressionReal(x.GetVector());
	}
	static VectorExpressionBase* GetFunctionExpressionVector(const Vector& x)
	{
		VectorExpressionBase::newCount += 1;
		return new VectorExpressionReal(x);
	}
public:
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	friend SymbolicRealMatrix operator+(const SymbolicRealMatrix& left, const SymbolicRealMatrix& right)
	{
		if (!SReal::recordExpressions) {
			return left.matrix + right.matrix;
		}
		MatrixExpressionBase::NewCount()++;
		return new MatrixExpressionOperatorPlus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SymbolicRealMatrix operator-(const SymbolicRealMatrix& left, const SymbolicRealMatrix& right)
	{
		if (!SReal::recordExpressions) {
			return left.matrix - right.matrix;
		}
		MatrixExpressionBase::NewCount()++;
		return new MatrixExpressionOperatorMinus(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	friend SymbolicRealMatrix operator*(const SReal& left, const SymbolicRealMatrix& right)
	{
		if (!SReal::recordExpressions) {
			return left.GetValue() * right.matrix;
		}
		MatrixExpressionBase::NewCount()++;
		return new MatrixExpressionOperatorMultScalarMatrix(GetFunctionExpressionSReal(left), GetFunctionExpression(right));
	}

	friend SymbolicRealMatrix operator*(const SymbolicRealMatrix& left, const SReal& right)
	{
		if (!SReal::recordExpressions) {
			return left.matrix * right.GetValue();
		}
		MatrixExpressionBase::NewCount()++;
		//NOTE: change of order!
		return new MatrixExpressionOperatorMultScalarMatrix(GetFunctionExpressionSReal(right), GetFunctionExpression(left));
	}

	friend SymbolicRealVector operator*(const SymbolicRealVector& left, const SymbolicRealMatrix& right)
	{
		if (!SReal::recordExpressions)
		{
			SymbolicRealVector result(right.NumberOfColumns());
			EXUmath::MultMatrixTransposedVectorTemplate(right.matrix, left.GetVector(), result.GetVector());
			return result;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionOperatorMultMatrixTransposed(GetFunctionExpression(right), GetFunctionExpressionVector(left));
	}

	friend SymbolicRealVector operator*(const SymbolicRealMatrix& left, const SymbolicRealVector& right)
	{
		if (!SReal::recordExpressions)
		{
			SymbolicRealVector result(left.NumberOfRows());
			EXUmath::MultMatrixVectorTemplate(left.matrix, right.GetVector(), result.GetVector());
			return result;
		}
		VectorExpressionBase::NewCount()++;
		return new VectorExpressionOperatorMultMatrix(GetFunctionExpression(left), GetFunctionExpressionVector(right));
	}



	SymbolicRealMatrix& operator+=(const SymbolicRealMatrix& rhs) {
		if (!SReal::recordExpressions) {
			matrix += rhs.matrix;
			return *this;
		}
		MatrixExpressionBase::newCount += 1;
		exprMatrix = new MatrixExpressionOperatorPlus(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		exprMatrix->IncreaseReferenceCounter();
		return *this;
	}

	SymbolicRealMatrix& operator-=(const SymbolicRealMatrix& rhs) {
		if (!SReal::recordExpressions) {
			matrix -= rhs.matrix;
			return *this;
		}
		MatrixExpressionBase::newCount += 1;
		exprMatrix = new MatrixExpressionOperatorMinus(GetFunctionExpression(*this, false), GetFunctionExpression(rhs));
		exprMatrix->IncreaseReferenceCounter();
		return *this;
	}

	SymbolicRealMatrix& operator*=(const SReal& rhs) {
		if (!SReal::recordExpressions) {
			matrix *= rhs.GetValue();
			return *this;
		}
		MatrixExpressionBase::newCount += 1;
		exprMatrix = new MatrixExpressionOperatorMultScalarMatrix(GetFunctionExpressionSReal(rhs), GetFunctionExpression(*this, false));
		exprMatrix->IncreaseReferenceCounter();
		return *this;
	}


	SymbolicRealMatrix operator-() const
	{
		if (!SReal::recordExpressions) {
			return -1. * matrix;
		}
		MatrixExpressionBase::NewCount()++;
		return new MatrixExpressionUnaryMinus(GetFunctionExpression(*this, true));
	}

	SymbolicRealMatrix operator+() const
	{
		if (!SReal::recordExpressions) {
			return matrix;
		}
		MatrixExpressionBase::NewCount()++;
		return new MatrixExpressionUnaryPlus(GetFunctionExpression(*this, true));
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	friend SymbolicRealMatrix operator*(const SymbolicRealMatrix& left, const SymbolicRealMatrix& right)
	{
		if (!SReal::recordExpressions) {
			return left.matrix * right.matrix;
		}
		MatrixExpressionBase::NewCount()++;
		return new MatrixExpressionOperatorMultMatrixMatrix(GetFunctionExpression(left), GetFunctionExpression(right));
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//friend SReal operator==(const SymbolicRealMatrix& left, const SymbolicRealMatrix& right)
	//{
	//	if (!SReal::recordExpressions) {
	//		return (Real)(left.matrix == right.matrix);
	//	}
	//	MatrixExpressionBase::NewCount()++;
	//	return new MatrixExpressionOperatorEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	//}

	//friend SReal operator!=(const SymbolicRealMatrix& left, const SymbolicRealMatrix& right)
	//{
	//	if (!SReal::recordExpressions) {
	//		return (Real)(!(left.matrix == right.matrix));
	//	}
	//	MatrixExpressionBase::NewCount()++;
	//	return new MatrixExpressionOperatorNEQ(GetFunctionExpression(left), GetFunctionExpression(right));
	//}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	////! component-wise multiplication, requiring special treatment
	//SymbolicRealMatrix MultComponents(const SymbolicRealMatrix& right)
	//{
	//	if (!SReal::recordExpressions) {
	//		ResizableConstMatrix result;
	//		EXUmath::MultMatrixComponents(matrix, right.matrix, result);
	//		return result;
	//	}
	//	MatrixExpressionBase::NewCount()++;
	//	return new MatrixExpressionMultComponents(GetFunctionExpression(*this), GetFunctionExpression(right));
	//}

	//SReal NormL2() const
	//{
	//	if (!SReal::recordExpressions) {
	//		return matrix.GetL2Norm();
	//	}
	//	MatrixExpressionBase::NewCount()++;
	//	return new MatrixExpressionNormL2(GetFunctionExpression(*this));
	//}

	template<typename RealType>
	SReal GetComponent(const RealType& srow, const RealType& scolumn) const
	{
		if (!SReal::recordExpressions)
		{
			Real rowReal = Number2Real(srow);
			Index row = (Index)rowReal;
			CHECKandTHROW((row >= 0) && (row < matrix.NumberOfRows()), "SymbolicRealMatrix::operator(row,column): invalid row");
			CHECKandTHROW(rowReal == (Real)row, "SymbolicRealMatrix::operator(row,column): row index must be integer");

			Real columnReal = Number2Real(scolumn);
			Index column = (Index)columnReal;
			CHECKandTHROW((column >= 0) && (column < matrix.NumberOfColumns()), "SymbolicRealMatrix::operator(row,column): invalid column");
			CHECKandTHROW(columnReal == (Real)column, "SymbolicRealMatrix::operator(row,column): column index must be integer");

			return matrix(row, column);
		}
		ExpressionBase::NewCount()++;
		return new MatrixExpressionOperatorBracket(GetFunctionExpression(*this, true),
			GetFunctionExpressionSReal(srow), GetFunctionExpressionSReal(scolumn));
	}

	template<typename RealType>
	SReal operator()(const RealType& srow, const RealType& scolumn) const
	{
		return GetComponent<RealType>(srow, scolumn);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	friend std::ostream& operator<<(std::ostream& os, const SymbolicRealMatrix& thisExpr)
	{
		os << thisExpr.ToString();
		return os;
	}

};











}; //namespace Symbolic

#endif //SYMBOLIC__H
