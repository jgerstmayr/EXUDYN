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
static const OperationInfo operationTable[] =
{
	{ OperationCode::Leaf, "", -1, Zero, false }, //the variable itself: Variable, at the derivative
	{ OperationCode::Node, "", -1, NoDerivative, false },
	{ OperationCode::Plus, "+", 2, RealPlus, true },
	{ OperationCode::Minus, "-", 2, RealMinus, false },
	{ OperationCode::Multiply, "*", 2, RealMul, true },
	{ OperationCode::Divide, "/", 2, RealDiv, false },
	{ OperationCode::Equal, "==", 2, Zero, true }, //piecewise constant: zero where the derivative exists
	{ OperationCode::NotEqual, "!=", 2, Zero, true },
	{ OperationCode::Greater, ">", 2, Zero, false },
	{ OperationCode::GreaterEqual, ">=", 2, Zero, false },
	{ OperationCode::LessEqual, "<=", 2, Zero, false },
	{ OperationCode::Less, "<", 2, Zero, false },
	{ OperationCode::Positive, "pos", 1, RealPos, false },
	{ OperationCode::Negative, "neg", 1, RealNeg, false },
	{ OperationCode::IsFinite, "isfinite", 1, Zero, false },
	{ OperationCode::Abs, "abs", 1, RealAbs, false },
	{ OperationCode::Sign, "sign", 1, Zero, false },
	{ OperationCode::Not, "Not", 1, Zero, false },
	{ OperationCode::Round, "round", 1, Zero, false },
	{ OperationCode::Ceil, "ceil", 1, Zero, false },
	{ OperationCode::Floor, "floor", 1, Zero, false },
	{ OperationCode::Sqrt, "sqrt", 1, RealSqrt, false },
	{ OperationCode::Exp, "exp", 1, RealExp, false },
	{ OperationCode::Log, "log", 1, RealLog, false },
	{ OperationCode::Sin, "sin", 1, RealSin, false },
	{ OperationCode::Cos, "cos", 1, RealCos, false },
	{ OperationCode::Tan, "tan", 1, RealTan, false },
	{ OperationCode::ASin, "asin", 1, RealASin, false },
	{ OperationCode::ACos, "acos", 1, RealACos, false },
	{ OperationCode::ATan, "atan", 1, RealATan, false },
	{ OperationCode::Sinh, "sinh", 1, RealSinh, false },
	{ OperationCode::Cosh, "cosh", 1, RealCosh, false },
	{ OperationCode::Tanh, "tanh", 1, RealTanh, false },
	{ OperationCode::ASinh, "asinh", 1, RealASinh, false },
	{ OperationCode::ACosh, "acosh", 1, RealACosh, false },
	{ OperationCode::ATanh, "atanh", 1, RealATanh, false },
	{ OperationCode::Pow, "pow", 2, RealPow, false },
	{ OperationCode::Atan2, "atan2", 2, RealAtan2, false },
	{ OperationCode::Mod, "mod", 2, NoDerivative, false },
	{ OperationCode::Min, "min", 2, NoDerivative, false },
	{ OperationCode::Max, "max", 2, NoDerivative, false },
	{ OperationCode::IfThenElse, "IfThenElse", 3, RealIfThenElse, false },
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

int FindDerivativeRule(const FlatNode& node, const std::vector<FlatKind>& operandKinds)
{
	const STDstring name = FlatNameOf(node);
	Index n = (Index)operandKinds.size();
	auto Kinds = [&](FlatKind a, FlatKind b) { return n == 2 && operandKinds[0] == a && operandKinds[1] == b; };
	if (n == 0) { return Zero; } //a leaf; the variable itself is Variable
	if (node.kind == FlatKind::Real)
	{
		//piecewise constant: the conditions of IfThenElse; their derivative is zero where it exists
		static const std::unordered_map<STDstring, int> constant = { {"==", Zero}, {"!=", Zero}, {">", Zero}, {">=", Zero},
			{"<=", Zero}, {"<", Zero}, {"Not", Zero}, {"isfinite", Zero}, {"sign", Zero}, {"round", Zero}, {"ceil", Zero},
			{"floor", Zero} };
		if (constant.find(name) != constant.end()) { return Zero; }
		if (n == 1 && operandKinds[0] == FlatKind::Real)
		{
			static const std::unordered_map<STDstring, int> unary = { {"pos", RealPos}, {"neg", RealNeg}, {"abs", RealAbs}, {"sin", RealSin},
				{"cos", RealCos}, {"tan", RealTan}, {"exp", RealExp}, {"log", RealLog}, {"sqrt", RealSqrt}, {"asin", RealASin},
				{"acos", RealACos}, {"atan", RealATan}, {"sinh", RealSinh}, {"cosh", RealCosh}, {"tanh", RealTanh},
				{"asinh", RealASinh}, {"acosh", RealACosh}, {"atanh", RealATanh} };
			auto found = unary.find(name);
			return found == unary.end() ? NoDerivative : found->second;
		}
		if (Kinds(FlatKind::Real, FlatKind::Real))
		{
			static const std::unordered_map<STDstring, int> binary = { {"+", RealPlus}, {"-", RealMinus}, {"*", RealMul},
				{"/", RealDiv}, {"pow", RealPow}, {"atan2", RealAtan2} };
			auto found = binary.find(name);
			return found == binary.end() ? NoDerivative : found->second;
		}
		if (name == "IfThenElse" && n == 3) { return RealIfThenElse; }
		if (name == "@" && Kinds(FlatKind::Vector, FlatKind::Vector)) { return RealDot; }
		if (name == "NormL2" && n == 1 && operandKinds[0] == FlatKind::Vector) { return RealNorm; }
		if (name == "component" && n == 2 && operandKinds[0] == FlatKind::Vector) { return RealVectorComponent; }
		if (name == "component" && n == 3 && operandKinds[0] == FlatKind::Matrix) { return RealMatrixComponent; }
		return NoDerivative;
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
		return NoDerivative;
	}
	if (name == "matrix") { return MatrixComponents; }
	if (name == "+" && Kinds(FlatKind::Matrix, FlatKind::Matrix)) { return MatrixPlus; }
	if (name == "-" && Kinds(FlatKind::Matrix, FlatKind::Matrix)) { return MatrixMinus; }
	if (name == "*" && Kinds(FlatKind::Real, FlatKind::Matrix)) { return MatrixScalar; }
	if (name == "pos" && n == 1) { return MatrixPos; }
	if (name == "neg" && n == 1) { return MatrixNeg; }
	if (name == "@" && Kinds(FlatKind::Matrix, FlatKind::Matrix)) { return MatrixMatrix; }
	if (name == "T" && n == 1) { return MatrixTransposed; }
	return NoDerivative;
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
	Index m, Index n, Real* out)
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
	default: //NoDerivative: zero, if no operand depends on the variable, else it raises
	{
		Index numberOfOperands = NumberOfOperandsOfNode(node);
		for (Index i = 0; i < numberOfOperands; i++)
		{
			Index components = SlotComponents(OperandOfNode(node, i), v[i]);
			for (Index c = 0; c < components*n; c++)
			{
				CHECKandTHROW(d[i][c] == 0., STDstring("exudyn.symbolic: the operation '") + OperationName(node) + "' has no derivative", ExudynValueError);
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
		if (IsTableCode(step.code)) //a scalar operation, from the values of the slots of its operands
		{
			slot[k].real = ScalarOperation(step.code, [slot, o](Index i) { return slot[o[i]].real; });
			continue;
		}
		FlatValue* values = operandValues.data(); //a leaf or an operation of its node, as FlatProgram::Evaluate
		for (Index i = 0; i < step.numberOfOperands; i++) { values[i] = slot[o[i]]; }
		switch (step.node.kind)
		{
		case FlatKind::Real: slot[k].real = ((const ExpressionBase*)step.node.node)->EvaluateFromOperands(values); break;
		case FlatKind::Vector: slot[k].vector = &((const VectorExpressionBase*)step.node.node)->EvaluateFromOperands(values); break;
		default: slot[k].matrix = &((const MatrixExpressionBase*)step.node.node)->EvaluateFromOperands(values); break;
		}
	}
}

void SymbolicProgram::Differentiate(const void* variable, Index n) const
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
			SlotComponents(step.node, slots[k]), n, derivatives.data() + derivativeOffsets[k]);
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
static SymbolicDerivativeValue SymbolicDerivativeOfOperation(int rule, const FlatNode& node, const FlatValue& value,
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
		Index rows = value.matrix->NumberOfRows(), columns = value.matrix->NumberOfColumns();
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
		CHECKandTHROW(false, STDstring("exudyn.symbolic.Jacobian: the operation '") + OperationName(node)
			+ "' has no derivative as an expression", ExudynValueError);
		return result;
	}
}

//! the derivative of a leaf by component j: the unit vector of the variable, else zero
static SymbolicDerivativeValue SymbolicDerivativeOfLeaf(const FlatNode& node, const FlatValue& value, const void* variable, Index j)
{
	SymbolicDerivativeValue result;
	if (node.node != variable) { return result; }
	switch (node.kind)
	{
	case FlatKind::Real: return DerivativeOfReal(SReal(1.));
	case FlatKind::Vector:
	{
		Vector derivative(value.vector->NumberOfItems(), 0.);
		derivative[j] = 1.;
		return DerivativeOfVector(SymbolicRealVector(derivative));
	}
	default:
	{
		Matrix derivative(value.matrix->NumberOfRows(), value.matrix->NumberOfColumns(), 0.);
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
				for (Index j = 0; j < n; j++) { derivatives[k][j] = SymbolicDerivativeOfLeaf(node, table.SlotValue(k), variableNode.node, j); }
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
			int rule = (node.node == variableNode.node) ? (int)Variable : FindDerivativeRule(node, kinds);
			for (Index j = 0; j < n; j++)
			{
				for (Index i = 0; i < numberOfOperands; i++) { d[i] = &derivatives[program.OperandSlot(k, i)][j]; }
				derivatives[k][j] = SymbolicDerivativeOfOperation(rule, node, table.SlotValue(k), values[k], x, d);
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
