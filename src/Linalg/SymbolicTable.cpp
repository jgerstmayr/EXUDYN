/** ***********************************************************************************************
* @brief		the table of the symbolic operations and SymbolicProgram (#3153); see SymbolicTable.h
*
* @author		Gerstmayr Johannes
* @date			2026-10-29
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */

#include "Utilities/BasicDefinitions.h"
#include <pybind11/pybind11.h> //Symbolic.h keeps Python objects in VariableSet
namespace py = pybind11;
#include "Linalg/BasicLinalg.h"
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <cmath>

#include "Linalg/Symbolic.h"
#include "Linalg/SymbolicTable.h"
#include "Linalg/SymbolicVector.h" //the derivative as new nodes
#include "Linalg/SymbolicMatrix.h"

namespace Symbolic
{

//! the table: one row per code, in the order of OperationCode
static const FlatKind R = FlatKind::Real;
static const FlatKind V = FlatKind::Vector;
static const FlatKind M = FlatKind::Matrix;
static const OperationInfo operationTable[] =
{
	{ OperationCode::Leaf, "", R, "", Zero, false }, //the variable itself: Variable, at the derivative
	{ OperationCode::Node, "", R, "", NoDerivative, false },
	{ OperationCode::Plus, "+", R, "RR", RealPlus, true },
	{ OperationCode::Minus, "-", R, "RR", RealMinus, false },
	{ OperationCode::Multiply, "*", R, "RR", RealMul, true },
	{ OperationCode::Divide, "/", R, "RR", RealDiv, false },
	{ OperationCode::Equal, "==", R, "RR", Zero, true }, //piecewise constant: zero where the derivative exists
	{ OperationCode::NotEqual, "!=", R, "RR", Zero, true },
	{ OperationCode::Greater, ">", R, "RR", Zero, false },
	{ OperationCode::GreaterEqual, ">=", R, "RR", Zero, false },
	{ OperationCode::LessEqual, "<=", R, "RR", Zero, false },
	{ OperationCode::Less, "<", R, "RR", Zero, false },
	{ OperationCode::Positive, "pos", R, "R", RealPos, false },
	{ OperationCode::Negative, "neg", R, "R", RealNeg, false },
	{ OperationCode::IsFinite, "isfinite", R, "R", Zero, false },
	{ OperationCode::Abs, "abs", R, "R", RealAbs, false },
	{ OperationCode::Sign, "sign", R, "R", Zero, false },
	{ OperationCode::Not, "Not", R, "R", Zero, false },
	{ OperationCode::Round, "round", R, "R", Zero, false },
	{ OperationCode::Ceil, "ceil", R, "R", Zero, false },
	{ OperationCode::Floor, "floor", R, "R", Zero, false },
	{ OperationCode::Sqrt, "sqrt", R, "R", RealSqrt, false },
	{ OperationCode::Exp, "exp", R, "R", RealExp, false },
	{ OperationCode::Log, "log", R, "R", RealLog, false },
	{ OperationCode::Sin, "sin", R, "R", RealSin, false },
	{ OperationCode::Cos, "cos", R, "R", RealCos, false },
	{ OperationCode::Tan, "tan", R, "R", RealTan, false },
	{ OperationCode::ASin, "asin", R, "R", RealASin, false },
	{ OperationCode::ACos, "acos", R, "R", RealACos, false },
	{ OperationCode::ATan, "atan", R, "R", RealATan, false },
	{ OperationCode::Sinh, "sinh", R, "R", RealSinh, false },
	{ OperationCode::Cosh, "cosh", R, "R", RealCosh, false },
	{ OperationCode::Tanh, "tanh", R, "R", RealTanh, false },
	{ OperationCode::ASinh, "asinh", R, "R", RealASinh, false },
	{ OperationCode::ACosh, "acosh", R, "R", RealACosh, false },
	{ OperationCode::ATanh, "atanh", R, "R", RealATanh, false },
	{ OperationCode::Pow, "pow", R, "RR", RealPow, false },
	{ OperationCode::Atan2, "atan2", R, "RR", RealAtan2, false },
	{ OperationCode::Mod, "mod", R, "RR", NoDerivative, false },
	{ OperationCode::Min, "min", R, "RR", NoDerivative, false },
	{ OperationCode::Max, "max", R, "RR", NoDerivative, false },
	{ OperationCode::IfThenElse, "IfThenElse", R, "RRR", RealIfThenElse, false },
	{ OperationCode::LookupTable, "LookupTable", R, "R", RealLookupTable, false }, //by the data of its node (#3175)
	{ OperationCode::LookupTableDerivative, "LookupTableDerivative", R, "R", Zero, false }, //piecewise constant
	//the operations of vectors and matrices (#3176); not commutative for Optimize, which merges them as before by their class
	{ OperationCode::Dot, "@", R, "VV", RealDot, false },
	{ OperationCode::Norm, "NormL2", R, "V", RealNorm, false },
	{ OperationCode::VectorComponent, "component", R, "VR", RealVectorComponent, false },
	{ OperationCode::VectorEqual, "==", R, "VV", Zero, false },
	{ OperationCode::VectorNotEqual, "!=", R, "VV", Zero, false },
	{ OperationCode::MatrixComponent, "component", R, "MRR", RealMatrixComponent, false },
	{ OperationCode::VectorOfComponents, "vector", V, "*", VectorComponents, false },
	{ OperationCode::VectorPlus, "+", V, "VV", VectorPlus, false },
	{ OperationCode::VectorMinus, "-", V, "VV", VectorMinus, false },
	{ OperationCode::VectorScalar, "*", V, "RV", VectorScalar, false },
	{ OperationCode::MatrixVector, "@", V, "MV", VectorMatrixVector, false },
	{ OperationCode::MatrixTransposedVector, "vector@matrix", V, "MV", VectorMatrixTransposedVector, false },
	{ OperationCode::VectorPositive, "pos", V, "V", VectorPos, false },
	{ OperationCode::VectorNegative, "neg", V, "V", VectorNeg, false },
	{ OperationCode::VectorMultiply, "multiply", V, "VV", VectorMultiply, false },
	{ OperationCode::MatrixOfComponents, "matrix", M, "*", MatrixComponents, false },
	{ OperationCode::MatrixPlus, "+", M, "MM", MatrixPlus, false },
	{ OperationCode::MatrixMinus, "-", M, "MM", MatrixMinus, false },
	{ OperationCode::MatrixScalar, "*", M, "RM", MatrixScalar, false },
	{ OperationCode::MatrixPositive, "pos", M, "M", MatrixPos, false },
	{ OperationCode::MatrixNegative, "neg", M, "M", MatrixNeg, false },
	{ OperationCode::MatrixMatrix, "@", M, "MM", MatrixMatrix, false },
	{ OperationCode::MatrixTransposed, "T", M, "M", MatrixTransposed, false },
};
static_assert(sizeof(operationTable)/sizeof(operationTable[0]) == (size_t)OperationCode::NumberOfCodes, "a row per operation code");

const OperationInfo& OperationOfCode(OperationCode code) { return operationTable[(int)code]; }

//! the letter of a kind in the column operands of the table
static char KindLetter(FlatKind kind) { return kind == FlatKind::Real ? 'R' : (kind == FlatKind::Vector ? 'V' : 'M'); }

//! the name of a node of any kind
static const char* FlatNameOf(const FlatNode& node);

OperationCode OperationCodeOf(const FlatNode& node, const std::vector<FlatKind>& operandKinds)
{
	if (operandKinds.size() == 0) { return OperationCode::Leaf; }
	//the key of a row: the kind of the result, the name, the kinds of the operands
	static const std::unordered_map<STDstring, OperationCode> codeOfKey = []()
	{
		std::unordered_map<STDstring, OperationCode> codes;
		for (const OperationInfo& row : operationTable)
		{
			if (row.name[0]) { codes[STDstring(1, KindLetter(row.kind)) + row.name + ":" + row.operands] = row.code; }
		}
		return codes;
	}();
	STDstring key = STDstring(1, KindLetter(node.kind)) + FlatNameOf(node) + ":";
	bool allReal = true;
	for (FlatKind kind : operandKinds) { key += KindLetter(kind); allReal = allReal && kind == FlatKind::Real; }
	auto found = codeOfKey.find(key);
	if (found == codeOfKey.end() && allReal) //the components of a vector or a matrix, any number of them
	{
		found = codeOfKey.find(STDstring(1, KindLetter(node.kind)) + FlatNameOf(node) + ":*");
	}
	return found == codeOfKey.end() ? OperationCode::Node : found->second;
}

//! the operand i of a node of any kind
static FlatNode OperandOfNode(const FlatNode& node, Index i)
{
	switch (node.kind)
	{
	case FlatKind::Real: return ((const ExpressionBase*)node.node)->GetOperand(i);
	case FlatKind::Vector: return ((const VectorExpressionBase*)node.node)->GetOperand(i);
	default: return ((const MatrixExpressionBase*)node.node)->GetOperand(i);
	}
}

//! the number of operands of a node of any kind
static Index NumberOfOperandsOfNode(const FlatNode& node)
{
	switch (node.kind)
	{
	case FlatKind::Real: return ((const ExpressionBase*)node.node)->NumberOfOperands();
	case FlatKind::Vector: return ((const VectorExpressionBase*)node.node)->NumberOfOperands();
	default: return ((const MatrixExpressionBase*)node.node)->NumberOfOperands();
	}
}

OperationCode OperationCodeOfNode(const FlatNode& node)
{
	Index n = NumberOfOperandsOfNode(node);
	std::vector<FlatKind> kinds(n);
	for (Index i = 0; i < n; i++) { kinds[i] = OperandOfNode(node, i).kind; }
	return OperationCodeOf(node, kinds);
}

static const char* FlatNameOf(const FlatNode& node);
const char* OperationName(const FlatNode& node)
{
	OperationCode code = OperationCodeOfNode(node);
	return IsTableCode(code) ? OperationOfCode(code).name : FlatNameOf(node);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//the rules of the forward derivative in one pass (#3116, design docs/dev/design/symbolicSystem.md section 11 S2)

static const char* FlatNameOf(const FlatNode& node)
{
	switch (node.kind)
	{
	case FlatKind::Real: return ((const ExpressionBase*)node.node)->FlatName();
	case FlatKind::Vector: return ((const VectorExpressionBase*)node.node)->FlatName();
	default: return ((const MatrixExpressionBase*)node.node)->FlatName();
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//the operations of vectors and matrices (#3176), with the arithmetic and the messages of the EvaluateFromOperands
//of their nodes, so that the results are the same to the bit

//! the size of the result of an operation of vectors and matrices from its operands; node: for the shape of a
//! matrix of components
static void ArrayResultSize(OperationCode code, const FlatNode& node, const SlotValue* x, Index numberOfOperands,
	Index& rows, Index& columns)
{
	rows = 1;
	columns = 1;
	switch (code)
	{
	case OperationCode::Dot:
		if (!(x[0].rows == x[1].rows)) { PyError("symbolic.Vector::operator* (scalar vector multiplication) : inconsistent vector sizes", PyErrorType::valueError); }
		break;
	case OperationCode::VectorEqual: case OperationCode::VectorNotEqual:
		CHECKandTHROW(x[0].rows == x[1].rows, "VectorBase::operator==: incompatible size of vectors");
		break;
	case OperationCode::Norm: case OperationCode::VectorComponent: case OperationCode::MatrixComponent: break;
	case OperationCode::VectorOfComponents: rows = numberOfOperands; break;
	case OperationCode::VectorPlus:
		if (x[0].rows != x[1].rows) { PyError("symbolic.Vector::operator+ : inconsistent vector sizes", PyErrorType::valueError); }
		rows = x[0].rows;
		break;
	case OperationCode::VectorMinus:
		if (!(x[0].rows == x[1].rows)) { PyError("symbolic.Vector::operator- : inconsistent vector sizes", PyErrorType::valueError); }
		rows = x[0].rows;
		break;
	case OperationCode::VectorScalar: rows = x[1].rows; break;
	case OperationCode::MatrixVector:
		CHECKandTHROW(x[0].columns == x[1].rows, "EXUmath::MultMatrixVector(matrix,vector,result,T): Size mismatch");
		rows = x[0].rows;
		break;
	case OperationCode::MatrixTransposedVector:
		CHECKandTHROW(x[0].rows == x[1].rows, "EXUmath::MultMatrixTransposedVectorTemplate(matrix,vector,result): Size mismatch");
		rows = x[0].columns;
		break;
	case OperationCode::VectorPositive: case OperationCode::VectorNegative: rows = x[0].rows; break;
	case OperationCode::VectorMultiply:
		if (!(x[0].rows == x[1].rows)) { PyError("symbolic.MultComponents(...): inconsistent vector sizes", PyErrorType::valueError); }
		rows = x[0].rows;
		break;
	case OperationCode::MatrixOfComponents:
		rows = ((const MatrixExpressionBase*)node.node)->NumberOfRows();
		columns = ((const MatrixExpressionBase*)node.node)->NumberOfColumns();
		break;
	case OperationCode::MatrixPlus:
		if (!(x[0].rows == x[1].rows && x[0].columns == x[1].columns)) { PyError("symbolic.Matrix::operator+ : inconsistent matrix sizes", PyErrorType::valueError); }
		rows = x[0].rows; columns = x[0].columns;
		break;
	case OperationCode::MatrixMinus:
		if (!(x[0].rows == x[1].rows && x[0].columns == x[1].columns)) { PyError("symbolic.Matrix::operator- : inconsistent matrix sizes", PyErrorType::valueError); }
		rows = x[0].rows; columns = x[0].columns;
		break;
	case OperationCode::MatrixScalar: rows = x[1].rows; columns = x[1].columns; break;
	case OperationCode::MatrixPositive: case OperationCode::MatrixNegative: rows = x[0].rows; columns = x[0].columns; break;
	case OperationCode::MatrixMatrix:
		if (!(x[0].columns == x[1].rows)) { PyError("symbolic.Matrix::operator@ (matrix matrix multiplication) : inconsistent matrix sizes", PyErrorType::valueError); }
		rows = x[0].rows; columns = x[1].columns;
		break;
	case OperationCode::MatrixTransposed: rows = x[0].columns; columns = x[0].rows; break;
	default: CHECKandTHROWstring("ArrayResultSize: the code is no operation of vectors and matrices");
	}
}

Real ArrayOperation(OperationCode code, const SlotValue* x, Index numberOfOperands, Real* out)
{
	switch (code)
	{
	case OperationCode::Dot:
	{
		Real result = 0;
		for (Index c = 0; c < x[0].rows; c++) { result += x[0].data[c] * x[1].data[c]; }
		return result;
	}
	case OperationCode::Norm:
	{
		Real result = 0.;
		for (Index c = 0; c < x[0].rows; c++) { result += x[0].data[c] * x[0].data[c]; }
		return std::sqrt(result);
	}
	case OperationCode::VectorComponent:
	{
		Real iReal = x[1].real;
		Index i = (Index)iReal;
		CHECKandTHROW((Real)i == iReal, "VectorExpressionBase::operator[] index must be integer");
		//while a user function is recorded, its vector arguments have no size yet: the component has no value then (#3007)
		if (SReal::recordExpressions && x[0].rows == 0 && i >= 0) { return 0.; }
		CHECKandTHROW((i >= 0) && (i < x[0].rows), "VectorExpressionBase::operator[] index out of range");
		return x[0].data[i];
	}
	case OperationCode::VectorEqual: case OperationCode::VectorNotEqual:
	{
		bool equal = true;
		for (Index c = 0; c < x[0].rows; c++) { if (x[1].data[c] != x[0].data[c]) { equal = false; break; } }
		return Real(code == OperationCode::VectorEqual ? equal : !equal);
	}
	case OperationCode::MatrixComponent:
	{
		Real rowReal = x[1].real;
		Index row = (Index)rowReal;
		Real columnReal = x[2].real;
		Index column = (Index)columnReal;
		CHECKandTHROW(((Real)row == rowReal) && ((Real)column == columnReal), "MatrixExpressionBase::operator[] index must contain integer indices");
		CHECKandTHROW((row >= 0) && (row < x[0].rows), "Matrix::operator()(Index, Index) const: request of invalid row");
		CHECKandTHROW((column >= 0) && (column < x[0].columns), "Matrix::operator()(Index, Index) const: request of invalid column");
		return x[0].data[row*x[0].columns + column];
	}
	case OperationCode::VectorOfComponents: case OperationCode::MatrixOfComponents:
		for (Index i = 0; i < numberOfOperands; i++) { out[i] = x[i].real; }
		return 0.;
	case OperationCode::VectorPlus: case OperationCode::MatrixPlus:
		for (Index i = 0; i < x[0].Components(); i++) { out[i] = x[0].data[i] + x[1].data[i]; }
		return 0.;
	case OperationCode::VectorMinus: case OperationCode::MatrixMinus:
		for (Index i = 0; i < x[0].Components(); i++) { out[i] = x[0].data[i] - x[1].data[i]; }
		return 0.;
	case OperationCode::VectorScalar: case OperationCode::MatrixScalar:
	{
		Real factor = x[0].real;
		for (Index i = 0; i < x[1].Components(); i++) { out[i] = factor * x[1].data[i]; }
		return 0.;
	}
	case OperationCode::MatrixVector: //as EXUmath::MultMatrixVectorTemplate
	{
		const Real* mm = x[0].data;
		const Real* vv = x[1].data;
		for (Index i = 0; i < x[0].rows; i++)
		{
			out[i] = 0;
			const Real* mr = &mm[i*x[0].columns];
			for (Index j = 0; j < x[1].rows; j++) { out[i] += mr[j] * vv[j]; }
		}
		return 0.;
	}
	case OperationCode::MatrixTransposedVector: //as EXUmath::MultMatrixTransposedVectorTemplate
	{
		const Real* mm = x[0].data;
		const Real* vv = x[1].data;
		Index resultLength = x[0].columns;
		for (Index i = 0; i < resultLength; i++)
		{
			Real val = 0;
			const Real* mr = &mm[i];
			for (Index j = 0; j < x[1].rows; j++)
			{
				val += *mr * vv[j];
				mr += resultLength;
			}
			out[i] = val;
		}
		return 0.;
	}
	case OperationCode::VectorPositive: case OperationCode::MatrixPositive:
		for (Index i = 0; i < x[0].Components(); i++) { out[i] = x[0].data[i]; }
		return 0.;
	case OperationCode::VectorNegative: case OperationCode::MatrixNegative:
		for (Index i = 0; i < x[0].Components(); i++) { out[i] = -x[0].data[i]; }
		return 0.;
	case OperationCode::VectorMultiply:
		for (Index i = 0; i < x[0].rows; i++) { out[i] = x[0].data[i] * x[1].data[i]; }
		return 0.;
	case OperationCode::MatrixMatrix:
	{
		const Real* a = x[0].data;
		const Real* b = x[1].data;
		Index inner = x[0].columns, columns = x[1].columns;
		for (Index i = 0; i < x[0].rows; i++)
		{
			for (Index j = 0; j < columns; j++)
			{
				Real value = 0.;
				for (Index k = 0; k < inner; k++) { value += a[i*inner + k]*b[k*columns + j]; }
				out[i*columns + j] = value;
			}
		}
		return 0.;
	}
	case OperationCode::MatrixTransposed:
	{
		Index rows = x[0].rows, columns = x[0].columns;
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < columns; j++) { out[j*rows + i] = x[0].data[i*columns + j]; }
		}
		return 0.;
	}
	default: CHECKandTHROWstring("ArrayOperation: the code is no operation of vectors and matrices"); return 0.;
	}
}

void ForwardDerivative(int rule, const FlatNode& node, const SlotValue& value, const SlotValue* v, Index numberOfOperands,
	const Real* const* d, Index m, Index n, Real* out)
{
	const Real nan = std::numeric_limits<Real>::quiet_NaN();
	switch (rule)
	{
	case Variable: //the unit matrix
		for (Index c = 0; c < m; c++) { for (Index j = 0; j < n; j++) { out[c*n + j] = (c == j) ? 1. : 0.; } }
		break;
	case Zero: for (Index i = 0; i < m*n; i++) { out[i] = 0.; } break;
	case RealIfThenElse: { const Real* chosen = v[0].real ? d[1] : d[2]; for (Index j = 0; j < n; j++) { out[j] = chosen[j]; } break; }
	case RealLookupTable: //the slope of the interval, from the data of the node (#3175)
	{
		Real slope = ((const ExpressionLookupTable*)node.node)->GetTable()->Value(v[0].real, true);
		for (Index j = 0; j < n; j++) { out[j] = slope*d[0][j]; }
		break;
	}
	case RealPlus: for (Index j = 0; j < n; j++) { out[j] = d[0][j] + d[1][j]; } break;
	case RealMinus: for (Index j = 0; j < n; j++) { out[j] = d[0][j] - d[1][j]; } break;
	case RealMul: for (Index j = 0; j < n; j++) { out[j] = d[0][j]*v[1].real + v[0].real*d[1][j]; } break;
	case RealDiv:
	{
		Real den = v[1].real;
		for (Index j = 0; j < n; j++) { out[j] = (den == 0) ? nan : (d[0][j]*den - v[0].real*d[1][j])/(den*den); }
		break;
	}
	case RealPos: for (Index j = 0; j < n; j++) { out[j] = d[0][j]; } break;
	case RealNeg: for (Index j = 0; j < n; j++) { out[j] = -d[0][j]; } break;
	case RealAbs: //no derivative at 0
	{
		Real x = v[0].real;
		for (Index j = 0; j < n; j++) { out[j] = (x == 0) ? nan : (x > 0 ? d[0][j] : -d[0][j]); }
		break;
	}
	case RealPow: //as ExpressionPower::Diff: an exponent constant in a column also for a base <= 0 (#3135)
	{
		Real a = v[0].real, b = v[1].real;
		for (Index j = 0; j < n; j++)
		{
			if (d[1][j] == 0.) { out[j] = (d[0][j] == 0.) ? 0. : b*EM::Pow(a, b - 1)*d[0][j]; }
			else { out[j] = (a <= 0) ? nan : b*EM::Pow(a, b - 1)*d[0][j] + EM::Log(a)*EM::Pow(a, b)*d[1][j]; }
		}
		break;
	}
	case RealAtan2:
	{
		Real y = v[0].real, x = v[1].real, den = y*y + x*x;
		for (Index j = 0; j < n; j++) { out[j] = (den == 0) ? nan : (x*d[0][j] - y*d[1][j])/den; }
		break;
	}
	case RealSin: case RealCos: case RealTan: case RealExp: case RealLog: case RealSqrt: case RealASin: case RealACos:
	case RealATan: case RealSinh: case RealCosh: case RealTanh: case RealASinh: case RealACosh: case RealATanh:
	{
		Real x = v[0].real, f = value.real, factor = 0.;
		switch (rule) //f'(x), with the domains of the Diff of each function
		{
		case RealSin: factor = EM::Cos(x); break;
		case RealCos: factor = -EM::Sin(x); break;
		case RealTan: factor = 1 + f*f; break;
		case RealExp: factor = f; break;
		case RealLog: factor = 1/x; break;
		case RealSqrt: factor = (x <= 0) ? nan : 0.5/f; break;
		case RealASin: factor = (x <= -1 || x >= 1) ? nan : 1/std::sqrt(1 - x*x); break;
		case RealACos: factor = (x <= -1 || x >= 1) ? nan : -1/std::sqrt(1 - x*x); break;
		case RealATan: factor = 1/(1 + x*x); break;
		case RealSinh: factor = EM::Cosh(x); break;
		case RealCosh: factor = EM::Sinh(x); break;
		case RealTanh: factor = 1 - f*f; break;
		case RealASinh: factor = 1/std::sqrt(x*x + 1); break;
		case RealACosh: factor = (x < 1) ? nan : 1/std::sqrt(x*x - 1); break;
		default: factor = (x <= -1 || x >= 1) ? nan : 1/(1 - x*x); break; //atanh
		}
		for (Index j = 0; j < n; j++) { out[j] = factor*d[0][j]; }
		break;
	}
	case RealDot:
	{
		const Real* a = v[0].data, * b = v[1].data;
		for (Index j = 0; j < n; j++)
		{
			Real sum = 0.;
			for (Index c = 0; c < v[0].rows; c++) { sum += d[0][c*n + j]*b[c] + a[c]*d[1][c*n + j]; }
			out[j] = sum;
		}
		break;
	}
	case RealNorm:
	{
		const Real* a = v[0].data;
		Real norm = value.real;
		for (Index j = 0; j < n; j++)
		{
			Real sum = 0.;
			for (Index c = 0; c < v[0].rows; c++) { sum += a[c]*d[0][c*n + j]; }
			out[j] = (norm == 0) ? nan : sum/norm;
		}
		break;
	}
	case RealVectorComponent:
	{
		Index c = (Index)v[1].real;
		for (Index j = 0; j < n; j++) { out[j] = d[0][c*n + j]; }
		break;
	}
	case RealMatrixComponent:
	{
		Index c = (Index)v[1].real*v[0].columns + (Index)v[2].real;
		for (Index j = 0; j < n; j++) { out[j] = d[0][c*n + j]; }
		break;
	}
	case VectorComponents: case MatrixComponents: //component c is operand c
		for (Index c = 0; c < m; c++) { for (Index j = 0; j < n; j++) { out[c*n + j] = d[c][j]; } }
		break;
	case VectorPlus: case MatrixPlus: for (Index i = 0; i < m*n; i++) { out[i] = d[0][i] + d[1][i]; } break;
	case VectorMinus: case MatrixMinus: for (Index i = 0; i < m*n; i++) { out[i] = d[0][i] - d[1][i]; } break;
	case VectorPos: case MatrixPos: for (Index i = 0; i < m*n; i++) { out[i] = d[0][i]; } break;
	case VectorNeg: case MatrixNeg: for (Index i = 0; i < m*n; i++) { out[i] = -d[0][i]; } break;
	case VectorScalar: case MatrixScalar: //s x: ds x + s dx
	{
		Real factor = v[0].real;
		const Real* x = v[1].data;
		for (Index c = 0; c < m; c++) { for (Index j = 0; j < n; j++) { out[c*n + j] = d[0][j]*x[c] + factor*d[1][c*n + j]; } }
		break;
	}
	case VectorMultiply:
	{
		const Real* a = v[0].data, * b = v[1].data;
		for (Index c = 0; c < m; c++) { for (Index j = 0; j < n; j++) { out[c*n + j] = d[0][c*n + j]*b[c] + a[c]*d[1][c*n + j]; } }
		break;
	}
	case VectorMatrixVector: case VectorMatrixTransposedVector: //A x or A^T x: dA x + A dx
	{
		const Real* A = v[0].data;
		const Real* x = v[1].data;
		Index columns = v[0].columns;
		bool transposed = (rule == VectorMatrixTransposedVector);
		Index inner = transposed ? v[0].rows : columns;
		for (Index c = 0; c < m; c++)
		{
			for (Index j = 0; j < n; j++)
			{
				Real sum = 0.;
				for (Index l = 0; l < inner; l++)
				{
					Index a = transposed ? l*columns + c : c*columns + l; //component of A
					sum += d[0][a*n + j]*x[l] + A[a]*d[1][l*n + j];
				}
				out[c*n + j] = sum;
			}
		}
		break;
	}
	case MatrixMatrix: //A B: dA B + A dB
	{
		const Real* A = v[0].data, * B = v[1].data;
		Index rows = v[0].rows, inner = v[0].columns, columns = v[1].columns;
		for (Index r = 0; r < rows; r++)
		{
			for (Index c = 0; c < columns; c++)
			{
				Real* target = out + (r*columns + c)*n;
				for (Index j = 0; j < n; j++) { target[j] = 0.; }
				for (Index l = 0; l < inner; l++)
				{
					Real a = A[r*inner + l], b = B[l*columns + c];
					const Real* dA = d[0] + (r*inner + l)*n;
					const Real* dB = d[1] + (l*columns + c)*n;
					for (Index j = 0; j < n; j++) { target[j] += dA[j]*b + a*dB[j]; }
				}
			}
		}
		break;
	}
	case MatrixTransposed:
	{
		Index rows = v[0].rows, columns = v[0].columns; //of A; the result is columns x rows
		for (Index r = 0; r < rows; r++)
		{
			for (Index c = 0; c < columns; c++)
			{
				for (Index j = 0; j < n; j++) { out[(c*rows + r)*n + j] = d[0][(r*columns + c)*n + j]; }
			}
		}
		break;
	}
	default: //NoDerivative: zero, if no operand depends on the variable, else it raises
	{
		for (Index i = 0; i < numberOfOperands; i++)
		{
			for (Index c = 0; c < v[i].Components()*n; c++)
			{
				if (d[i][c] != 0.) { PyError(STDstring("exudyn.symbolic: the operation '") + OperationName(node) + "' has no derivative", PyErrorType::valueError); }
			}
		}
		for (Index i = 0; i < m*n; i++) { out[i] = 0.; }
		break;
	}
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//SymbolicProgram

void SymbolicProgram::Build(const FlatProgram& program)
{
	Clear();
	const Index n = program.NumberOfOperations();
	steps.resize(n);
	std::vector<FlatKind> kinds;
	Index maxOperands = 0;
	for (Index k = 0; k < n; k++)
	{
		Step& step = steps[k];
		step.node = program.OperationNode(k);
		step.firstOperand = (Index)operandSlots.size();
		step.numberOfOperands = program.NumberOfOperationOperands(k);
		kinds.resize(step.numberOfOperands);
		for (Index i = 0; i < step.numberOfOperands; i++)
		{
			Index slot = program.OperandSlot(k, i);
			operandSlots.push_back(slot);
			kinds[i] = steps[slot].node.kind;
		}
		maxOperands = EXUstd::Maximum(maxOperands, step.numberOfOperands);
		step.code = OperationCodeOf(step.node, kinds);
		CHECKandTHROW(step.code != OperationCode::Node, STDstring("SymbolicProgram: the operation '") + FlatNameOf(step.node)
			+ "' has no code of the table of operations");
		step.derivativeRule = OperationOfCode(step.code).derivativeRule;
		if (step.code == OperationCode::Leaf) { leaves.push_back(k); }
	}
	for (Index i = 0; i < program.NumberOfResults(); i++) { results.push_back(program.ResultSlot(i)); }
	slots.resize(n);
	operandValues.resize(maxOperands);
	operandDerivatives.resize(maxOperands);
	resultVectors.resize(results.size()); //not resized later: the references to the results stay valid
	resultMatrices.resize(results.size());
}

Index SymbolicProgram::NumberOfTableSteps() const
{
	Index count = 0;
	for (const Step& step : steps) { count += IsTableCode(step.code); }
	return count;
}

//! the value of a leaf, read from its node: a number, or the vector or matrix that the node keeps
static void LeafValue(const FlatNode& node, SlotValue& value)
{
	switch (node.kind)
	{
	case FlatKind::Real: value.real = ((const ExpressionBase*)node.node)->Evaluate(); break;
	case FlatKind::Vector:
	{
		const ResizableConstVector& vector = ((const VectorExpressionBase*)node.node)->EvaluateRef();
		value.data = vector.GetDataPointer();
		value.rows = vector.NumberOfItems();
		value.columns = 1;
		break;
	}
	default:
	{
		const ResizableMatrix& matrix = ((const MatrixExpressionBase*)node.node)->EvaluateRef();
		value.data = matrix.GetDataPointer();
		value.rows = matrix.NumberOfRows();
		value.columns = matrix.NumberOfColumns();
		break;
	}
	}
}

void SymbolicProgram::Layout() const
{
	const Index n = (Index)steps.size();
	laidOut = false;
	offsets.assign(n, -1);
	Index size = 0;
	for (Index k = 0; k < n; k++)
	{
		const Step& step = steps[k];
		if (step.code == OperationCode::Leaf || IsScalarCode(step.code) || IsLookupTableCode(step.code)) { continue; }
		for (Index i = 0; i < step.numberOfOperands; i++) { operandValues[i] = slots[operandSlots[step.firstOperand + i]]; }
		ArrayResultSize(step.code, step.node, operandValues.data(), step.numberOfOperands, slots[k].rows, slots[k].columns);
		if (step.node.kind != FlatKind::Real)
		{
			offsets[k] = size;
			size += slots[k].Components();
		}
	}
	memory.resize(size);
	for (Index k = 0; k < n; k++) { if (offsets[k] >= 0) { slots[k].data = memory.data() + offsets[k]; } }
	laidOut = true;
}

void SymbolicProgram::Evaluate() const
{
	SlotValue* slot = slots.data();
	bool sizesChanged = !laidOut;
	for (Index k : leaves)
	{
		Index rows = slot[k].rows, columns = slot[k].columns;
		LeafValue(steps[k].node, slot[k]);
		sizesChanged = sizesChanged || rows != slot[k].rows || columns != slot[k].columns;
	}
	if (sizesChanged) { Layout(); }

	const Index* operandSlot = operandSlots.data();
	const Index n = (Index)steps.size();
	for (Index k = 0; k < n; k++)
	{
		const Step& step = steps[k];
		if (step.code == OperationCode::Leaf) { continue; }
		const Index* o = operandSlot + step.firstOperand;
		if (IsScalarCode(step.code)) //a scalar operation, from the values of the slots of its operands
		{
			slot[k].real = ScalarOperation(step.code, [slot, o](Index i) { return slot[o[i]].real; });
			continue;
		}
		if (IsLookupTableCode(step.code)) { slot[k].real = LookupTableOperation(step.node, slot[o[0]].real); continue; }
		SlotValue* x = operandValues.data(); //an operation of vectors and matrices, into the memory of the program
		for (Index i = 0; i < step.numberOfOperands; i++) { x[i] = slot[o[i]]; }
		slot[k].real = ArrayOperation(step.code, x, step.numberOfOperands, offsets[k] >= 0 ? memory.data() + offsets[k] : nullptr);
	}
}

const ResizableConstVector& SymbolicProgram::VectorResult(Index i) const
{
	const SlotValue& value = slots[results[i]];
	ResizableConstVector& result = resultVectors[i];
	result.SetNumberOfItems(value.rows);
	std::copy(value.data, value.data + value.rows, result.GetDataPointer());
	return result;
}

const ResizableMatrix& SymbolicProgram::MatrixResult(Index i) const
{
	const SlotValue& value = slots[results[i]];
	ResizableMatrix& result = resultMatrices[i];
	result.SetNumberOfRowsAndColumns(value.rows, value.columns);
	std::copy(value.data, value.data + value.Components(), result.GetDataPointer());
	return result;
}

void SymbolicProgram::Differentiate(const void* variable, Index n) const
{
	const Index numberOfSteps = (Index)steps.size();
	//the offsets of the slots in the block; the sizes of vectors and matrices are those of this evaluation
	derivativeOffsets.resize(numberOfSteps + 1);
	derivativeOffsets[0] = 0;
	for (Index k = 0; k < numberOfSteps; k++) { derivativeOffsets[k + 1] = derivativeOffsets[k] + slots[k].Components()*n; }
	derivatives.resize(derivativeOffsets[numberOfSteps]); //allocates only when the block grows

	for (Index k = 0; k < numberOfSteps; k++)
	{
		const Step& step = steps[k];
		const Index* o = operandSlots.data() + step.firstOperand;
		for (Index i = 0; i < step.numberOfOperands; i++)
		{
			operandValues[i] = slots[o[i]];
			operandDerivatives[i] = derivatives.data() + derivativeOffsets[o[i]];
		}
		int rule = (step.node.node == variable) ? (int)Variable : step.derivativeRule;
		ForwardDerivative(rule, step.node, slots[k], operandValues.data(), step.numberOfOperands, operandDerivatives.data(),
			slots[k].Components(), n, derivatives.data() + derivativeOffsets[k]);
	}
}

void OnePassDerivative(const FlatNode& root, const void* variable, Index n, std::vector<Real>& derivatives)
{
	FlatProgram program;
	program.Build({ root });
	SymbolicProgram table;
	table.Build(program);
	table.Evaluate();
	table.Differentiate(variable, n);
	const Real* result = table.ResultDerivative(0);
	derivatives.assign(result, result + table.ResultComponents(0)*n);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//THE DERIVATIVE AS NEW NODES (#3163): per operation its derivative by one component of the variable, an expression of
//the operands and of the derivatives of the operands, by the rule of the table - the same rules as ForwardDerivative,
//which computes their numbers

//! a derivative of one slot by one component of the variable: zero, or an expression of the kind of the slot
struct SymbolicDerivativeValue
{
	bool zero = true;
	SReal real;
	SymbolicRealVector vector;
	SymbolicRealMatrix matrix;
};

//! the expression of a node of the recording, as the operand of a derivative
struct SymbolicOperand
{
	SReal real;
	SymbolicRealVector vector;
	SymbolicRealMatrix matrix;
	SymbolicOperand(const FlatNode& node)
	{
		switch (node.kind)
		{
		case FlatKind::Real: real = SReal((ExpressionBase*)node.node); break;
		case FlatKind::Vector: vector = SymbolicRealVector((VectorExpressionBase*)node.node); break;
		default: matrix = SymbolicRealMatrix((MatrixExpressionBase*)node.node); break;
		}
	}
};

static SymbolicDerivativeValue DerivativeOfReal(const SReal& x) { SymbolicDerivativeValue d; d.zero = false; d.real = x; return d; }
static SymbolicDerivativeValue DerivativeOfVector(const SymbolicRealVector& x) { SymbolicDerivativeValue d; d.zero = false; d.vector = x; return d; }
static SymbolicDerivativeValue DerivativeOfMatrix(const SymbolicRealMatrix& x) { SymbolicDerivativeValue d; d.zero = false; d.matrix = x; return d; }

//! a vector of scalar expressions, a node "vector" of the recording
static SymbolicRealVector VectorOfComponents(const std::vector<SReal>& components)
{
	VectorExpressionBase::NewCount()++;
	VectorExpressionSReal* e = new VectorExpressionSReal(components); //referenced once by itself
	SymbolicRealVector result(e);
	e->DecreaseReferenceCounter();
	return result;
}

//! a matrix of scalar expressions, a node "matrix" of the recording
static SymbolicRealMatrix MatrixOfComponents(const std::vector<std::vector<SReal>>& components)
{
	MatrixExpressionBase::NewCount()++;
	return SymbolicRealMatrix(new MatrixExpressionSReal(components));
}

//! the derivative of operation k by one component j of the variable, by its rule; d: the derivatives of its operands
static SymbolicDerivativeValue SymbolicDerivativeOfOperation(int rule, const FlatNode& node, const SlotValue& value,
	const SymbolicOperand& f, const std::vector<const SymbolicOperand*>& x, const std::vector<const SymbolicDerivativeValue*>& d)
{
	auto IsZero = [](Index i, const std::vector<const SymbolicDerivativeValue*>& d) { return d[i]->zero; };
	SymbolicDerivativeValue result;
	bool allZero = true;
	for (const SymbolicDerivativeValue* operand : d) { allZero &= operand->zero; }
	if (allZero) { return result; } //no operand depends on the variable
	switch (rule)
	{
	case Zero: return result;
	case RealIfThenElse:
		if (IsZero(1, d) && IsZero(2, d)) { return result; }
		return DerivativeOfReal(SReal::IfThenElse(x[0]->real, IsZero(1, d) ? SReal(0.) : d[1]->real, IsZero(2, d) ? SReal(0.) : d[2]->real));
	case RealLookupTable: //the slope of the table at the operand, a lookup of the same table (#3175)
		return DerivativeOfReal(SReal::LookupTable(((const ExpressionLookupTable*)node.node)->GetTable(), x[0]->real, true)*d[0]->real);
	case RealPlus: case RealMinus:
		if (IsZero(1, d)) { return *d[0]; }
		if (IsZero(0, d)) { return DerivativeOfReal(rule == RealPlus ? d[1]->real : -d[1]->real); }
		return DerivativeOfReal(rule == RealPlus ? d[0]->real + d[1]->real : d[0]->real - d[1]->real);
	case RealPos: return *d[0];
	case RealNeg: return IsZero(0, d) ? result : DerivativeOfReal(-d[0]->real);
	case RealAbs: return IsZero(0, d) ? result : DerivativeOfReal((x[0]->real/f.real)*d[0]->real); //nan at 0, as the one-pass derivative
	case RealMul:
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfReal(d[0]->real*x[1]->real); }
		if (IsZero(0, d)) { return DerivativeOfReal(x[0]->real*d[1]->real); }
		return DerivativeOfReal(d[0]->real*x[1]->real + x[0]->real*d[1]->real);
	case RealDiv: //(dx0 - f dx1)/x1
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfReal(d[0]->real/x[1]->real); }
		if (IsZero(0, d)) { return DerivativeOfReal(-(f.real*d[1]->real)/x[1]->real); }
		return DerivativeOfReal((d[0]->real - f.real*d[1]->real)/x[1]->real);
	case RealPow: //b a^(b-1) da + log(a) a^b db
	{
		SymbolicDerivativeValue byBase, byExponent;
		if (!IsZero(0, d)) { byBase = DerivativeOfReal(x[1]->real*SReal::pow(x[0]->real, x[1]->real - SReal(1.))*d[0]->real); }
		if (IsZero(1, d)) { return byBase; }
		byExponent = DerivativeOfReal(SReal::log(x[0]->real)*f.real*d[1]->real);
		return byBase.zero ? byExponent : DerivativeOfReal(byBase.real + byExponent.real);
	}
	case RealAtan2: //(x dy - y dx)/(x^2 + y^2) of atan2(y, x)
	{
		if (IsZero(0, d) && IsZero(1, d)) { return result; }
		SReal denominator = x[0]->real*x[0]->real + x[1]->real*x[1]->real;
		if (IsZero(1, d)) { return DerivativeOfReal(x[1]->real*d[0]->real/denominator); }
		if (IsZero(0, d)) { return DerivativeOfReal(-(x[0]->real*d[1]->real)/denominator); }
		return DerivativeOfReal((x[1]->real*d[0]->real - x[0]->real*d[1]->real)/denominator);
	}
	case RealSin: case RealCos: case RealTan: case RealExp: case RealLog: case RealSqrt: case RealASin: case RealACos:
	case RealATan: case RealSinh: case RealCosh: case RealTanh: case RealASinh: case RealACosh: case RealATanh:
	{
		if (IsZero(0, d)) { return result; }
		const SReal& a = x[0]->real;
		const SReal& da = d[0]->real;
		switch (rule) //f'(a) da
		{
		case RealSin: return DerivativeOfReal(SReal::cos(a)*da);
		case RealCos: return DerivativeOfReal(-(SReal::sin(a)*da));
		case RealTan: return DerivativeOfReal((SReal(1.) + f.real*f.real)*da);
		case RealExp: return DerivativeOfReal(f.real*da);
		case RealLog: return DerivativeOfReal(da/a);
		case RealSqrt: return DerivativeOfReal(da/(SReal(2.)*f.real));
		case RealASin: return DerivativeOfReal(da/SReal::sqrt(SReal(1.) - a*a));
		case RealACos: return DerivativeOfReal(-(da/SReal::sqrt(SReal(1.) - a*a)));
		case RealATan: return DerivativeOfReal(da/(SReal(1.) + a*a));
		case RealSinh: return DerivativeOfReal(SReal::cosh(a)*da);
		case RealCosh: return DerivativeOfReal(SReal::sinh(a)*da);
		case RealTanh: return DerivativeOfReal((SReal(1.) - f.real*f.real)*da);
		case RealASinh: return DerivativeOfReal(da/SReal::sqrt(a*a + SReal(1.)));
		case RealACosh: return DerivativeOfReal(da/SReal::sqrt(a*a - SReal(1.)));
		default: return DerivativeOfReal(da/(SReal(1.) - a*a)); //atanh
		}
	}
	case RealDot:
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfReal(d[0]->vector*x[1]->vector); }
		if (IsZero(0, d)) { return DerivativeOfReal(x[0]->vector*d[1]->vector); }
		return DerivativeOfReal(d[0]->vector*x[1]->vector + x[0]->vector*d[1]->vector);
	case RealNorm: return IsZero(0, d) ? result : DerivativeOfReal((x[0]->vector*d[0]->vector)/f.real);
	case RealVectorComponent: return IsZero(0, d) ? result : DerivativeOfReal(d[0]->vector[x[1]->real]);
	case RealMatrixComponent: return IsZero(0, d) ? result : DerivativeOfReal(d[0]->matrix(x[1]->real, x[2]->real));
	case VectorComponents: case MatrixComponents: //component c is operand c
	{
		bool allZero = true;
		for (const SymbolicDerivativeValue* operand : d) { allZero &= operand->zero; }
		if (allZero) { return result; }
		if (rule == VectorComponents)
		{
			std::vector<SReal> components;
			for (const SymbolicDerivativeValue* operand : d) { components.push_back(operand->zero ? SReal(0.) : operand->real); }
			return DerivativeOfVector(VectorOfComponents(components));
		}
		Index rows = value.rows, columns = value.columns;
		std::vector<std::vector<SReal>> components(rows, std::vector<SReal>(columns));
		for (Index c = 0; c < rows*columns; c++) { components[c/columns][c%columns] = d[c]->zero ? SReal(0.) : d[c]->real; }
		return DerivativeOfMatrix(MatrixOfComponents(components));
	}
	case VectorPlus: case VectorMinus:
		if (IsZero(1, d)) { return *d[0]; }
		if (IsZero(0, d)) { return DerivativeOfVector(rule == VectorPlus ? d[1]->vector : -d[1]->vector); }
		return DerivativeOfVector(rule == VectorPlus ? d[0]->vector + d[1]->vector : d[0]->vector - d[1]->vector);
	case MatrixPlus: case MatrixMinus:
		if (IsZero(1, d)) { return *d[0]; }
		if (IsZero(0, d)) { return DerivativeOfMatrix(rule == MatrixPlus ? d[1]->matrix : SReal(-1.)*d[1]->matrix); }
		return DerivativeOfMatrix(rule == MatrixPlus ? d[0]->matrix + d[1]->matrix : d[0]->matrix - d[1]->matrix);
	case VectorPos: case MatrixPos: return *d[0];
	case VectorNeg: return IsZero(0, d) ? result : DerivativeOfVector(-d[0]->vector);
	case MatrixNeg: return IsZero(0, d) ? result : DerivativeOfMatrix(SReal(-1.)*d[0]->matrix);
	case VectorScalar: //s x: ds x + s dx
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfVector(d[0]->real*x[1]->vector); }
		if (IsZero(0, d)) { return DerivativeOfVector(x[0]->real*d[1]->vector); }
		return DerivativeOfVector(d[0]->real*x[1]->vector + x[0]->real*d[1]->vector);
	case MatrixScalar:
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfMatrix(d[0]->real*x[1]->matrix); }
		if (IsZero(0, d)) { return DerivativeOfMatrix(x[0]->real*d[1]->matrix); }
		return DerivativeOfMatrix(d[0]->real*x[1]->matrix + x[0]->real*d[1]->matrix);
	case VectorMultiply:
	{
		SymbolicRealVector a = x[0]->vector, b = x[1]->vector;
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfVector(SymbolicRealVector(d[0]->vector).MultComponents(b)); }
		if (IsZero(0, d)) { return DerivativeOfVector(a.MultComponents(d[1]->vector)); }
		return DerivativeOfVector(SymbolicRealVector(d[0]->vector).MultComponents(b) + a.MultComponents(d[1]->vector));
	}
	case VectorMatrixVector: //A x: dA x + A dx
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfVector(d[0]->matrix*x[1]->vector); }
		if (IsZero(0, d)) { return DerivativeOfVector(x[0]->matrix*d[1]->vector); }
		return DerivativeOfVector(d[0]->matrix*x[1]->vector + x[0]->matrix*d[1]->vector);
	case VectorMatrixTransposedVector: //A^T x, written x A: x dA + dx A
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfVector(x[1]->vector*d[0]->matrix); }
		if (IsZero(0, d)) { return DerivativeOfVector(d[1]->vector*x[0]->matrix); }
		return DerivativeOfVector(x[1]->vector*d[0]->matrix + d[1]->vector*x[0]->matrix);
	case MatrixMatrix: //A B: dA B + A dB
		if (IsZero(1, d)) { return IsZero(0, d) ? result : DerivativeOfMatrix(d[0]->matrix*x[1]->matrix); }
		if (IsZero(0, d)) { return DerivativeOfMatrix(x[0]->matrix*d[1]->matrix); }
		return DerivativeOfMatrix(d[0]->matrix*x[1]->matrix + x[0]->matrix*d[1]->matrix);
	case MatrixTransposed: return IsZero(0, d) ? result : DerivativeOfMatrix(d[0]->matrix.Transposed());
	default:
		PyError(STDstring("exudyn.symbolic.Jacobian: the operation '") + OperationName(node)
			+ "' has no derivative as an expression", PyErrorType::valueError);
		return result;
	}
}

//! the derivative of a leaf by component j: the unit vector of the variable, else zero
static SymbolicDerivativeValue SymbolicDerivativeOfLeaf(const FlatNode& node, const SlotValue& value, const void* variable, Index j)
{
	SymbolicDerivativeValue result;
	if (node.node != variable) { return result; }
	switch (node.kind)
	{
	case FlatKind::Real: return DerivativeOfReal(SReal(1.));
	case FlatKind::Vector:
	{
		Vector derivative(value.rows, 0.);
		derivative[j] = 1.;
		return DerivativeOfVector(SymbolicRealVector(derivative));
	}
	default:
	{
		Matrix derivative(value.rows, value.columns, 0.);
		derivative.GetDataPointer()[j] = 1.;
		return DerivativeOfMatrix(SymbolicRealMatrix(derivative));
	}
	}
}

//! esym.Jacobian (#3163)
py::object SymbolicJacobian(const py::object& expression, const py::object& variable)
{
	FlatNode root, variableNode;
	if (py::isinstance<SReal>(expression)) { root = FlatNode(py::cast<const SReal&>(expression).GetExpression()); }
	else if (py::isinstance<SymbolicRealVector>(expression)) { root = FlatNode(py::cast<const SymbolicRealVector&>(expression).GetExpression()); }
	else { CHECKandTHROW(false, "exudyn.symbolic.Jacobian: the expression must be an exudyn.symbolic.Real or Vector", ExudynValueError); }
	bool scalarVariable = py::isinstance<SReal>(variable);
	Index n = 1;
	if (scalarVariable)
	{
		const SReal& x = py::cast<const SReal&>(variable);
		CHECKandTHROW(x.IsExpressionNamedReal(), "exudyn.symbolic.Jacobian: the variable must be a named exudyn.symbolic.Real or Vector", ExudynValueError);
		variableNode = FlatNode(x.GetExpression());
	}
	else
	{
		CHECKandTHROW(py::isinstance<SymbolicRealVector>(variable) && py::cast<const SymbolicRealVector&>(variable).IsExpressionNamedReal(),
			"exudyn.symbolic.Jacobian: the variable must be a named exudyn.symbolic.Real or Vector", ExudynValueError);
		const SymbolicRealVector& x = py::cast<const SymbolicRealVector&>(variable);
		variableNode = FlatNode(x.GetExpression());
		n = x.NumberOfItems();
	}
	bool vectorResult = (root.kind == FlatKind::Vector);
	if (root.node == nullptr) //a number: the derivative is zero
	{
		Index m = vectorResult ? py::cast<const SymbolicRealVector&>(expression).GetVector().NumberOfItems() : 1;
		if (!vectorResult) { return scalarVariable ? py::cast(SReal(0.)) : py::cast(SymbolicRealVector(Vector(n, 0.))); }
		return scalarVariable ? py::cast(SymbolicRealVector(Vector(m, 0.))) : py::cast(SymbolicRealMatrix(Matrix(m, n, 0.)));
	}

	bool recording = SReal::recordExpressions;
	SReal::recordExpressions = true; //the operations below build nodes
	py::object resultObject;
	try
	{
		FlatProgram program;
		program.Build({ root });
		SymbolicProgram table; //the values, for the sizes of the vectors and matrices
		table.Build(program);
		table.Evaluate();
		const Index numberOfOperations = program.NumberOfOperations();
		std::vector<SymbolicOperand> values;
		values.reserve(numberOfOperations);
		std::vector<std::vector<SymbolicDerivativeValue>> derivatives(numberOfOperations, std::vector<SymbolicDerivativeValue>(n));
		std::vector<FlatKind> kinds;
		std::vector<const SymbolicOperand*> x;
		std::vector<const SymbolicDerivativeValue*> d;
		for (Index k = 0; k < numberOfOperations; k++)
		{
			const FlatNode& node = program.OperationNode(k);
			values.emplace_back(node);
			Index numberOfOperands = program.NumberOfOperationOperands(k);
			if (numberOfOperands == 0)
			{
				for (Index j = 0; j < n; j++) { derivatives[k][j] = SymbolicDerivativeOfLeaf(node, table.Value(k), variableNode.node, j); }
				continue;
			}
			kinds.resize(numberOfOperands);
			x.resize(numberOfOperands);
			d.resize(numberOfOperands);
			for (Index i = 0; i < numberOfOperands; i++)
			{
				Index slot = program.OperandSlot(k, i);
				kinds[i] = program.OperationNode(slot).kind;
				x[i] = &values[slot];
			}
			int rule = (node.node == variableNode.node) ? (int)Variable : OperationOfCode(OperationCodeOf(node, kinds)).derivativeRule;
			for (Index j = 0; j < n; j++)
			{
				for (Index i = 0; i < numberOfOperands; i++) { d[i] = &derivatives[program.OperandSlot(k, i)][j]; }
				derivatives[k][j] = SymbolicDerivativeOfOperation(rule, node, table.Value(k), values[k], x, d);
			}
		}
		//the result: of a Real, its derivative or the vector of the n derivatives; of a vector of m components, the
		//vector of the derivatives or the m x n matrix of their components
		const std::vector<SymbolicDerivativeValue>& result = derivatives[program.ResultSlot(0)];
		if (!vectorResult)
		{
			std::vector<SReal> components;
			for (const SymbolicDerivativeValue& value : result) { components.push_back(value.zero ? SReal(0.) : value.real); }
			resultObject = scalarVariable ? py::cast(components[0]) : py::cast(VectorOfComponents(components));
		}
		else
		{
			Index m = table.ResultComponents(0);
			if (scalarVariable) { resultObject = result[0].zero ? py::cast(SymbolicRealVector(Vector(m, 0.))) : py::cast(result[0].vector); }
			else
			{
				std::vector<std::vector<SReal>> components(m, std::vector<SReal>(n));
				for (Index j = 0; j < n; j++)
				{
					for (Index c = 0; c < m; c++) { components[c][j] = result[j].zero ? SReal(0.) : SReal(result[j].vector[(Real)c]); }
				}
				resultObject = py::cast(MatrixOfComponents(components));
			}
		}
	}
	catch (...)
	{
		SReal::recordExpressions = recording;
		throw;
	}
	SReal::recordExpressions = recording;
	return resultObject;
}

} //namespace Symbolic
