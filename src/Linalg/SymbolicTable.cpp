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

namespace Symbolic
{

bool symbolicProgramEvaluation = true;

//! the table: one row per code, in the order of OperationCode
static const OperationInfo operationTable[] =
{
	{ OperationCode::Leaf, "", -1, Fallback },
	{ OperationCode::Node, "", -1, Fallback },
	{ OperationCode::Plus, "+", 2, RealPlus },
	{ OperationCode::Minus, "-", 2, RealMinus },
	{ OperationCode::Multiply, "*", 2, RealMul },
	{ OperationCode::Divide, "/", 2, RealDiv },
	{ OperationCode::Equal, "==", 2, Zero }, //piecewise constant: zero where the derivative exists
	{ OperationCode::NotEqual, "!=", 2, Zero },
	{ OperationCode::Greater, ">", 2, Zero },
	{ OperationCode::GreaterEqual, ">=", 2, Zero },
	{ OperationCode::LessEqual, "<=", 2, Zero },
	{ OperationCode::Less, "<", 2, Zero },
	{ OperationCode::Positive, "pos", 1, RealPos },
	{ OperationCode::Negative, "neg", 1, RealNeg },
	{ OperationCode::IsFinite, "isfinite", 1, Zero },
	{ OperationCode::Abs, "abs", 1, Fallback },
	{ OperationCode::Sign, "sign", 1, Zero },
	{ OperationCode::Not, "Not", 1, Zero },
	{ OperationCode::Round, "round", 1, Zero },
	{ OperationCode::Ceil, "ceil", 1, Zero },
	{ OperationCode::Floor, "floor", 1, Zero },
	{ OperationCode::Sqrt, "sqrt", 1, RealSqrt },
	{ OperationCode::Exp, "exp", 1, RealExp },
	{ OperationCode::Log, "log", 1, RealLog },
	{ OperationCode::Sin, "sin", 1, RealSin },
	{ OperationCode::Cos, "cos", 1, RealCos },
	{ OperationCode::Tan, "tan", 1, RealTan },
	{ OperationCode::ASin, "asin", 1, RealASin },
	{ OperationCode::ACos, "acos", 1, RealACos },
	{ OperationCode::ATan, "atan", 1, RealATan },
	{ OperationCode::Sinh, "sinh", 1, RealSinh },
	{ OperationCode::Cosh, "cosh", 1, RealCosh },
	{ OperationCode::Tanh, "tanh", 1, RealTanh },
	{ OperationCode::ASinh, "asinh", 1, RealASinh },
	{ OperationCode::ACosh, "acosh", 1, RealACosh },
	{ OperationCode::ATanh, "atanh", 1, RealATanh },
	{ OperationCode::Pow, "pow", 2, RealPow },
	{ OperationCode::Atan2, "atan2", 2, RealAtan2 },
	{ OperationCode::Mod, "mod", 2, Fallback },
	{ OperationCode::Min, "min", 2, Fallback },
	{ OperationCode::Max, "max", 2, Fallback },
	{ OperationCode::IfThenElse, "IfThenElse", 3, RealIfThenElse },
};
static_assert(sizeof(operationTable)/sizeof(operationTable[0]) == (size_t)OperationCode::NumberOfCodes, "a row per operation code");

const OperationInfo& OperationOfCode(OperationCode code) { return operationTable[(int)code]; }

OperationCode OperationCodeOf(const FlatNode& node, const std::vector<FlatKind>& operandKinds)
{
	if (operandKinds.size() == 0) { return OperationCode::Leaf; }
	if (node.kind != FlatKind::Real) { return OperationCode::Node; }
	for (FlatKind kind : operandKinds) { if (kind != FlatKind::Real) { return OperationCode::Node; } }
	static const std::unordered_map<STDstring, OperationCode> codeOfName = []()
	{
		std::unordered_map<STDstring, OperationCode> codes;
		for (const OperationInfo& row : operationTable) { if (row.numberOfOperands > 0) { codes[row.name] = row.code; } }
		return codes;
	}();
	auto found = codeOfName.find(((const ExpressionBase*)node.node)->FlatName());
	if (found == codeOfName.end() || OperationOfCode(found->second).numberOfOperands != (Index)operandKinds.size()) { return OperationCode::Node; }
	return found->second;
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

int FindDerivativeRule(const FlatNode& node, const std::vector<FlatKind>& operandKinds)
{
	const STDstring name = FlatNameOf(node);
	Index n = (Index)operandKinds.size();
	auto Kinds = [&](FlatKind a, FlatKind b) { return n == 2 && operandKinds[0] == a && operandKinds[1] == b; };
	if (n == 0) { return Fallback; }
	if (node.kind == FlatKind::Real)
	{
		//piecewise constant: the conditions of IfThenElse; their derivative is zero where it exists
		static const std::unordered_map<STDstring, int> constant = { {"==", Zero}, {"!=", Zero}, {">", Zero}, {">=", Zero},
			{"<=", Zero}, {"<", Zero}, {"Not", Zero}, {"isfinite", Zero}, {"sign", Zero}, {"round", Zero}, {"ceil", Zero},
			{"floor", Zero} };
		if (constant.find(name) != constant.end()) { return Zero; }
		if (n == 1 && operandKinds[0] == FlatKind::Real)
		{
			static const std::unordered_map<STDstring, int> unary = { {"pos", RealPos}, {"neg", RealNeg}, {"sin", RealSin},
				{"cos", RealCos}, {"tan", RealTan}, {"exp", RealExp}, {"log", RealLog}, {"sqrt", RealSqrt}, {"asin", RealASin},
				{"acos", RealACos}, {"atan", RealATan}, {"sinh", RealSinh}, {"cosh", RealCosh}, {"tanh", RealTanh},
				{"asinh", RealASinh}, {"acosh", RealACosh}, {"atanh", RealATanh} };
			auto found = unary.find(name);
			return found == unary.end() ? Fallback : found->second;
		}
		if (Kinds(FlatKind::Real, FlatKind::Real))
		{
			static const std::unordered_map<STDstring, int> binary = { {"+", RealPlus}, {"-", RealMinus}, {"*", RealMul},
				{"/", RealDiv}, {"pow", RealPow}, {"atan2", RealAtan2} };
			auto found = binary.find(name);
			return found == binary.end() ? Fallback : found->second;
		}
		if (name == "IfThenElse" && n == 3) { return RealIfThenElse; }
		if (name == "@" && Kinds(FlatKind::Vector, FlatKind::Vector)) { return RealDot; }
		if (name == "NormL2" && n == 1 && operandKinds[0] == FlatKind::Vector) { return RealNorm; }
		if (name == "component" && n == 2 && operandKinds[0] == FlatKind::Vector) { return RealVectorComponent; }
		if (name == "component" && n == 3 && operandKinds[0] == FlatKind::Matrix) { return RealMatrixComponent; }
		return Fallback;
	}
	if (node.kind == FlatKind::Vector)
	{
		if (name == "vector") { return VectorComponents; }
		if (name == "+" && Kinds(FlatKind::Vector, FlatKind::Vector)) { return VectorPlus; }
		if (name == "-" && Kinds(FlatKind::Vector, FlatKind::Vector)) { return VectorMinus; }
		if (name == "*" && Kinds(FlatKind::Real, FlatKind::Vector)) { return VectorScalar; }
		if (name == "@" && Kinds(FlatKind::Matrix, FlatKind::Vector)) { return VectorMatrixVector; }
		if (name == "vector@matrix" && Kinds(FlatKind::Matrix, FlatKind::Vector)) { return VectorMatrixTransposedVector; }
		if (name == "pos" && n == 1) { return VectorPos; }
		if (name == "neg" && n == 1) { return VectorNeg; }
		if (name == "multiply" && Kinds(FlatKind::Vector, FlatKind::Vector)) { return VectorMultiply; }
		return Fallback;
	}
	if (name == "matrix") { return MatrixComponents; }
	if (name == "+" && Kinds(FlatKind::Matrix, FlatKind::Matrix)) { return MatrixPlus; }
	if (name == "-" && Kinds(FlatKind::Matrix, FlatKind::Matrix)) { return MatrixMinus; }
	if (name == "*" && Kinds(FlatKind::Real, FlatKind::Matrix)) { return MatrixScalar; }
	if (name == "pos" && n == 1) { return MatrixPos; }
	if (name == "neg" && n == 1) { return MatrixNeg; }
	if (name == "@" && Kinds(FlatKind::Matrix, FlatKind::Matrix)) { return MatrixMatrix; }
	if (name == "T" && n == 1) { return MatrixTransposed; }
	return Fallback;
}

Index SlotComponents(const FlatNode& node, const FlatValue& value)
{
	switch (node.kind)
	{
	case FlatKind::Real: return 1;
	case FlatKind::Vector: return value.vector->NumberOfItems();
	default: return value.matrix->NumberOfRows()*value.matrix->NumberOfColumns();
	}
}

void ForwardDerivative(int rule, const FlatNode& node, const FlatValue& value, const FlatValue* v, const Real* const* d,
	Index m, Index n, Real* out, const void* variable, bool scalarVariable)
{
	const Real nan = std::numeric_limits<Real>::quiet_NaN();
	switch (rule)
	{
	case Variable: //the unit matrix
		for (Index c = 0; c < m; c++) { for (Index j = 0; j < n; j++) { out[c*n + j] = (c == j) ? 1. : 0.; } }
		break;
	case Zero: for (Index i = 0; i < m*n; i++) { out[i] = 0.; } break;
	case RealIfThenElse: { const Real* chosen = v[0].real ? d[1] : d[2]; for (Index j = 0; j < n; j++) { out[j] = chosen[j]; } break; }
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
		const ResizableConstVector& a = *v[0].vector, & b = *v[1].vector;
		for (Index j = 0; j < n; j++)
		{
			Real sum = 0.;
			for (Index c = 0; c < a.NumberOfItems(); c++) { sum += d[0][c*n + j]*b[c] + a[c]*d[1][c*n + j]; }
			out[j] = sum;
		}
		break;
	}
	case RealNorm:
	{
		const ResizableConstVector& a = *v[0].vector;
		Real norm = value.real;
		for (Index j = 0; j < n; j++)
		{
			Real sum = 0.;
			for (Index c = 0; c < a.NumberOfItems(); c++) { sum += a[c]*d[0][c*n + j]; }
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
		Index c = (Index)v[1].real*v[0].matrix->NumberOfColumns() + (Index)v[2].real;
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
		const Real* x = (rule == VectorScalar) ? v[1].vector->GetDataPointer() : v[1].matrix->GetDataPointer();
		for (Index c = 0; c < m; c++) { for (Index j = 0; j < n; j++) { out[c*n + j] = d[0][j]*x[c] + factor*d[1][c*n + j]; } }
		break;
	}
	case VectorMultiply:
	{
		const ResizableConstVector& a = *v[0].vector, & b = *v[1].vector;
		for (Index c = 0; c < m; c++) { for (Index j = 0; j < n; j++) { out[c*n + j] = d[0][c*n + j]*b[c] + a[c]*d[1][c*n + j]; } }
		break;
	}
	case VectorMatrixVector: case VectorMatrixTransposedVector: //A x or A^T x: dA x + A dx
	{
		const ResizableMatrix& A = *v[0].matrix;
		const ResizableConstVector& x = *v[1].vector;
		Index columns = A.NumberOfColumns();
		bool transposed = (rule == VectorMatrixTransposedVector);
		Index inner = transposed ? A.NumberOfRows() : columns;
		for (Index c = 0; c < m; c++)
		{
			for (Index j = 0; j < n; j++)
			{
				Real sum = 0.;
				for (Index l = 0; l < inner; l++)
				{
					Index a = transposed ? l*columns + c : c*columns + l; //component of A
					sum += d[0][a*n + j]*x[l] + A.GetDataPointer()[a]*d[1][l*n + j];
				}
				out[c*n + j] = sum;
			}
		}
		break;
	}
	case MatrixMatrix: //A B: dA B + A dB
	{
		const ResizableMatrix& A = *v[0].matrix, & B = *v[1].matrix;
		Index rows = A.NumberOfRows(), inner = A.NumberOfColumns(), columns = B.NumberOfColumns();
		for (Index r = 0; r < rows; r++)
		{
			for (Index c = 0; c < columns; c++)
			{
				Real* target = out + (r*columns + c)*n;
				for (Index j = 0; j < n; j++) { target[j] = 0.; }
				for (Index l = 0; l < inner; l++)
				{
					Real a = A(r, l), b = B(l, c);
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
		const ResizableMatrix& A = *v[0].matrix;
		Index rows = A.NumberOfRows(), columns = A.NumberOfColumns(); //of A; the result is columns x rows
		for (Index r = 0; r < rows; r++)
		{
			for (Index c = 0; c < columns; c++)
			{
				for (Index j = 0; j < n; j++) { out[(c*rows + r)*n + j] = d[0][(r*columns + c)*n + j]; }
			}
		}
		break;
	}
	default: //Fallback: the derivative of the node per component of the variable, as Diff and DiffRef compute it
	{
		for (Index j = 0; j < n; j++)
		{
			DiffSeed seed(variable, scalarVariable ? -1 : j);
			switch (node.kind)
			{
			case FlatKind::Real: out[j] = ((const ExpressionBase*)node.node)->Diff(seed); break;
			case FlatKind::Vector:
			{
				const ResizableConstVector& column = ((const VectorExpressionBase*)node.node)->DiffRef(seed);
				for (Index c = 0; c < m; c++) { out[c*n + j] = column[c]; }
				break;
			}
			default:
			{
				const ResizableMatrix& column = ((const MatrixExpressionBase*)node.node)->DiffRef(seed);
				for (Index c = 0; c < m; c++) { out[c*n + j] = column.GetDataPointer()[c]; }
				break;
			}
			}
		}
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
		step.derivativeRule = (step.code == OperationCode::Node || step.code == OperationCode::Leaf) ?
			FindDerivativeRule(step.node, kinds) : OperationOfCode(step.code).derivativeRule;
	}
	for (Index i = 0; i < program.NumberOfResults(); i++) { results.push_back(program.ResultSlot(i)); }
	slots.resize(n);
	operandValues.resize(maxOperands);
	operandDerivatives.resize(maxOperands);
}

Index SymbolicProgram::NumberOfTableSteps() const
{
	Index count = 0;
	for (const Step& step : steps) { count += (step.code != OperationCode::Leaf && step.code != OperationCode::Node); }
	return count;
}

void SymbolicProgram::Evaluate() const
{
	FlatValue* slot = slots.data();
	const Index* operandSlot = operandSlots.data();
	const Index n = (Index)steps.size();
	for (Index k = 0; k < n; k++)
	{
		const Step& step = steps[k];
		const Index* o = operandSlot + step.firstOperand;
		//the operands of a scalar operation: the values of the slots, as the nodes compute them in EvaluateFromOperands
		#define EXU_A slot[o[0]].real
		#define EXU_B slot[o[1]].real
		Real& out = slot[k].real;
		switch (step.code)
		{
		case OperationCode::Plus: out = EXU_A + EXU_B; break;
		case OperationCode::Minus: out = EXU_A - EXU_B; break;
		case OperationCode::Multiply: out = EXU_A * EXU_B; break;
		case OperationCode::Divide: out = EXU_A / EXU_B; break;
		case OperationCode::Equal: out = Real(EXU_A == EXU_B); break;
		case OperationCode::NotEqual: out = Real(EXU_A != EXU_B); break;
		case OperationCode::Greater: out = Real(EXU_A > EXU_B); break;
		case OperationCode::GreaterEqual: out = Real(EXU_A >= EXU_B); break;
		case OperationCode::LessEqual: out = Real(EXU_A <= EXU_B); break;
		case OperationCode::Less: out = Real(EXU_A < EXU_B); break;
		case OperationCode::Positive: out = EXU_A; break;
		case OperationCode::Negative: out = -EXU_A; break;
		case OperationCode::IsFinite: out = std::isfinite(EXU_A); break;
		case OperationCode::Abs: out = std::fabs(EXU_A); break;
		case OperationCode::Sign: out = EXUstd::SignReal(EXU_A); break;
		case OperationCode::Not: out = !(EXU_A); break;
		case OperationCode::Round: out = std::round(EXU_A); break;
		case OperationCode::Ceil: out = std::ceil(EXU_A); break;
		case OperationCode::Floor: out = std::floor(EXU_A); break;
		case OperationCode::Sqrt: out = std::sqrt(EXU_A); break;
		case OperationCode::Exp: out = EM::Exp(EXU_A); break;
		case OperationCode::Log: out = EM::Log(EXU_A); break;
		case OperationCode::Sin: out = EM::Sin(EXU_A); break;
		case OperationCode::Cos: out = EM::Cos(EXU_A); break;
		case OperationCode::Tan: out = EM::Tan(EXU_A); break;
		case OperationCode::ASin: out = EM::Asin(EXU_A); break;
		case OperationCode::ACos: out = EM::Acos(EXU_A); break;
		case OperationCode::ATan: out = EM::Atan(EXU_A); break;
		case OperationCode::Sinh: out = EM::Sinh(EXU_A); break;
		case OperationCode::Cosh: out = EM::Cosh(EXU_A); break;
		case OperationCode::Tanh: out = EM::Tanh(EXU_A); break;
		case OperationCode::ASinh: out = std::asinh(EXU_A); break;
		case OperationCode::ACosh: out = std::acosh(EXU_A); break;
		case OperationCode::ATanh: out = std::atanh(EXU_A); break;
		case OperationCode::Pow: out = EM::Pow(EXU_A, EXU_B); break;
		case OperationCode::Atan2: out = EM::Atan2(EXU_A, EXU_B); break;
		case OperationCode::Mod: out = std::fmod(EXU_A, EXU_B); break;
		case OperationCode::Min: out = std::min(EXU_A, EXU_B); break;
		case OperationCode::Max: out = std::max(EXU_A, EXU_B); break;
		case OperationCode::IfThenElse: out = EXU_A ? EXU_B : slot[o[2]].real; break;
		default: //a leaf or an operation of its node, as FlatProgram::Evaluate
		{
			FlatValue* values = operandValues.data();
			for (Index i = 0; i < step.numberOfOperands; i++) { values[i] = slot[o[i]]; }
			switch (step.node.kind)
			{
			case FlatKind::Real: out = ((const ExpressionBase*)step.node.node)->EvaluateFromOperands(values); break;
			case FlatKind::Vector: slot[k].vector = &((const VectorExpressionBase*)step.node.node)->EvaluateFromOperands(values); break;
			default: slot[k].matrix = &((const MatrixExpressionBase*)step.node.node)->EvaluateFromOperands(values); break;
			}
			break;
		}
		}
		#undef EXU_A
		#undef EXU_B
	}
}

void SymbolicProgram::Differentiate(const void* variable, Index n, bool scalarVariable) const
{
	const Index numberOfSteps = (Index)steps.size();
	//the offsets of the slots in the block; the sizes of vectors and matrices are those of this evaluation
	derivativeOffsets.resize(numberOfSteps + 1);
	derivativeOffsets[0] = 0;
	for (Index k = 0; k < numberOfSteps; k++) { derivativeOffsets[k + 1] = derivativeOffsets[k] + SlotComponents(steps[k].node, slots[k])*n; }
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
		ForwardDerivative(rule, step.node, slots[k], operandValues.data(), operandDerivatives.data(),
			SlotComponents(step.node, slots[k]), n, derivatives.data() + derivativeOffsets[k], variable, scalarVariable);
	}
}

} //namespace Symbolic
