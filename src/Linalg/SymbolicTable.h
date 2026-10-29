/** ***********************************************************************************************
* @brief		the table of the symbolic operations and the program that evaluates and differentiates by it (#3153)
* @details		Details:
				- one row per operation code: its name, the number and kind of its operands, the rule of its forward
				  derivative; the evaluation of a code is its case in SymbolicProgram::Evaluate
				- SymbolicProgram: the operations of a flat program by their codes; the scalar operations computed by the
				  table, the operations of vectors and matrices and the leaves by their nodes, as in FlatProgram
				- the forward derivative of one operation, shared by FlatProgram and SymbolicProgram
				- design: docs/dev/design/symbolicSystem.md section 12.7, R1; the expression tree stays for the recording
*
* @author		Gerstmayr Johannes
* @date			2026-10-29
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef SYMBOLICTABLE__H
#define SYMBOLICTABLE__H

#include "Linalg/Symbolic.h" //FlatNode, FlatValue, FlatProgram
#include <vector>
#include <cmath>
#include <algorithm>

namespace Symbolic
{

//! the operation codes of the table: a leaf and an operation computed by its node - of vectors and matrices, or with
//! operands of other kinds -, then the scalar operations of scalar operands, computed by the table
enum class OperationCode : int
{
	Leaf = 0, Node,
	Plus, Minus, Multiply, Divide, Equal, NotEqual, Greater, GreaterEqual, LessEqual, Less,
	Positive, Negative, IsFinite, Abs, Sign, Not, Round, Ceil, Floor, Sqrt, Exp, Log, Sin, Cos, Tan, ASin, ACos, ATan,
	Sinh, Cosh, Tanh, ASinh, ACosh, ATanh, Pow, Atan2, Mod, Min, Max, IfThenElse,
	NumberOfCodes
};

//! the rules of the forward derivative (#3116); NoDerivative: an operation without derivative, min, max, mod, which
//! raises when it is differentiated (#3171)
enum DerivativeRule
{
	NoDerivative = 0, Variable, Zero, RealIfThenElse, RealAbs,
	RealPlus, RealMinus, RealMul, RealDiv, RealPos, RealNeg, RealSin, RealCos, RealTan, RealExp, RealLog, RealSqrt,
	RealASin, RealACos, RealATan, RealSinh, RealCosh, RealTanh, RealASinh, RealACosh, RealATanh, RealPow, RealAtan2,
	RealDot, RealNorm, RealVectorComponent, RealMatrixComponent,
	VectorComponents, VectorPlus, VectorMinus, VectorScalar, VectorMatrixVector, VectorMatrixTransposedVector,
	VectorPos, VectorNeg, VectorMultiply,
	MatrixComponents, MatrixPlus, MatrixMinus, MatrixScalar, MatrixPos, MatrixNeg, MatrixMatrix, MatrixTransposed
};

//! a row of the table of operations
struct OperationInfo
{
	OperationCode code;
	const char* name;		 //!< the name of the node, FlatName(); empty for Leaf and Node
	Index numberOfOperands;  //!< scalar operands; -1 for Leaf and Node
	int derivativeRule;		 //!< its DerivativeRule; for Node found by FindDerivativeRule
	bool commutative;		 //!< the two operands may be swapped exactly, for merging equal operations in Optimize
};

//! the row of a code
const OperationInfo& OperationOfCode(OperationCode code);
//! the code of a node of a flat program by its kind, its name and the kinds of its operands
OperationCode OperationCodeOf(const FlatNode& node, const std::vector<FlatKind>& operandKinds);
//! the code of a node, the kinds of its operands read from the node
OperationCode OperationCodeOfNode(const FlatNode& node);
//! true for a code computed by the table, a scalar operation of scalar operands
inline bool IsTableCode(OperationCode code) { return code != OperationCode::Leaf && code != OperationCode::Node; }
//! the name of an operation: of its row of the table, else FlatName() of its node (#3153, the export and the count)
const char* OperationName(const FlatNode& node);

//! the value of a scalar operation of the table from its operands x(0), x(1), x(2), as the EvaluateFromOperands of its
//! node computes it, with the same functions; the one evaluation of the table, for SymbolicProgram and Optimize
template<class TOperand>
inline Real ScalarOperation(OperationCode code, const TOperand& x)
{
	switch (code)
	{
	case OperationCode::Plus: return x(0) + x(1);
	case OperationCode::Minus: return x(0) - x(1);
	case OperationCode::Multiply: return x(0) * x(1);
	case OperationCode::Divide: return x(0) / x(1);
	case OperationCode::Equal: return Real(x(0) == x(1));
	case OperationCode::NotEqual: return Real(x(0) != x(1));
	case OperationCode::Greater: return Real(x(0) > x(1));
	case OperationCode::GreaterEqual: return Real(x(0) >= x(1));
	case OperationCode::LessEqual: return Real(x(0) <= x(1));
	case OperationCode::Less: return Real(x(0) < x(1));
	case OperationCode::Positive: return x(0);
	case OperationCode::Negative: return -x(0);
	case OperationCode::IsFinite: return std::isfinite(x(0));
	case OperationCode::Abs: return std::fabs(x(0));
	case OperationCode::Sign: return EXUstd::SignReal(x(0));
	case OperationCode::Not: return !(x(0));
	case OperationCode::Round: return std::round(x(0));
	case OperationCode::Ceil: return std::ceil(x(0));
	case OperationCode::Floor: return std::floor(x(0));
	case OperationCode::Sqrt: return std::sqrt(x(0));
	case OperationCode::Exp: return EM::Exp(x(0));
	case OperationCode::Log: return EM::Log(x(0));
	case OperationCode::Sin: return EM::Sin(x(0));
	case OperationCode::Cos: return EM::Cos(x(0));
	case OperationCode::Tan: return EM::Tan(x(0));
	case OperationCode::ASin: return EM::Asin(x(0));
	case OperationCode::ACos: return EM::Acos(x(0));
	case OperationCode::ATan: return EM::Atan(x(0));
	case OperationCode::Sinh: return EM::Sinh(x(0));
	case OperationCode::Cosh: return EM::Cosh(x(0));
	case OperationCode::Tanh: return EM::Tanh(x(0));
	case OperationCode::ASinh: return std::asinh(x(0));
	case OperationCode::ACosh: return std::acosh(x(0));
	case OperationCode::ATanh: return std::atanh(x(0));
	case OperationCode::Pow: return EM::Pow(x(0), x(1));
	case OperationCode::Atan2: return EM::Atan2(x(0), x(1));
	case OperationCode::Mod: return std::fmod(x(0), x(1));
	case OperationCode::Min: return std::min(x(0), x(1));
	case OperationCode::Max: return std::max(x(0), x(1));
	case OperationCode::IfThenElse: return x(0) ? x(1) : x(2);
	default: CHECKandTHROWstring("ScalarOperation: the code is no scalar operation of the table"); return 0.;
	}
}

//! the rule of the derivative of a node by its kind, its name and the kinds of its operands
int FindDerivativeRule(const FlatNode& node, const std::vector<FlatKind>& operandKinds);
//! the number of components of a value: 1, the size of a vector or rows*columns of a matrix
Index SlotComponents(const FlatNode& node, const FlatValue& value);
//! the forward derivative of one operation by its rule: its m components by the n components of the variable into out,
//! component c by component j at [c*n + j], from its value, the values v and the derivatives d of its operands
void ForwardDerivative(int rule, const FlatNode& node, const FlatValue& value, const FlatValue* v, const Real* const* d,
	Index m, Index n, Real* out);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! THE PROGRAM OF THE TABLE (#3153): the operations of a flat program - the same order and slots - by their codes; a
//! scalar operation of scalar operands is computed by its case of the table, without a call of its node; a leaf and
//! the other operations by their nodes, as FlatProgram computes them. The results are those of FlatProgram to the bit.
class SymbolicProgram
{
	struct Step
	{
		OperationCode code;
		FlatNode node;
		Index firstOperand;		//!< the slots of its operands are operandSlots[firstOperand ...]
		Index numberOfOperands;
		int derivativeRule;
	};
	std::vector<Step> steps;			//!< step k writes slot k
	std::vector<Index> operandSlots;
	std::vector<Index> results;
	mutable std::vector<FlatValue> slots;
	mutable std::vector<FlatValue> operandValues;
	//the forward derivative in one pass, as FlatProgram
	mutable std::vector<Real> derivatives;
	mutable std::vector<Index> derivativeOffsets;
	mutable std::vector<const Real*> operandDerivatives;
public:
	//! the steps of a built flat program, in its order
	void Build(const FlatProgram& program);
	bool IsBuilt() const { return results.size() != 0; }
	void Clear() { steps.clear(); operandSlots.clear(); results.clear(); slots.clear(); operandValues.clear(); }

	//! compute all steps in their order
	void Evaluate() const;
	//! after Evaluate(): the derivatives of all slots by the n components of variable - a named Real (n = 1), vector or
	//! matrix (n = rows*columns) node of the recording - in one pass in the order of the program, each operation by the
	//! rule of the table from the values and derivatives of its operands (#3116, #3171)
	void Differentiate(const void* variable, Index n) const;
	const Real* ResultDerivative(Index i) const { return derivatives.data() + derivativeOffsets[results[i]]; }

	Real Result(Index i) const { return slots[results[i]].real; }
	const ResizableConstVector& VectorResult(Index i) const { return *slots[results[i]].vector; }
	const ResizableMatrix& MatrixResult(Index i) const { return *slots[results[i]].matrix; }
	Index NumberOfSteps() const { return (Index)steps.size(); }
	//! the value of step k of the last evaluation
	const FlatValue& SlotValue(Index k) const { return slots[k]; }
	//! the number of components of result i of the last evaluation
	Index ResultComponents(Index i) const { return SlotComponents(steps[results[i]].node, slots[results[i]]); }
	//! the number of steps computed by the table, without a call of their node
	Index NumberOfTableSteps() const;
};

//! the derivatives of the components of an expression by the n components of a named variable, at the current values,
//! by the one-pass derivative of its program (#3171): component c by component j at [c*n + j]; the Diff of exudyn.symbolic
void OnePassDerivative(const FlatNode& root, const void* variable, Index n, std::vector<Real>& derivatives);

//! esym.Jacobian(expression, variable) (#3163): the derivative of an exudyn.symbolic Real or Vector by a named Real or
//! Vector as new expressions - a Real, a Vector of the derivatives by the components, or the Jacobian matrix -, built
//! in the order of the flat program, each operation by the rule of its row of the table; not optimized
py::object SymbolicJacobian(const py::object& expression, const py::object& variable);

} //namespace Symbolic

#endif //SYMBOLICTABLE__H
