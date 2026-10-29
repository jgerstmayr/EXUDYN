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

//! the rules of the forward derivative; Fallback: Diff or DiffRef of the node per component of the variable (#3116)
enum DerivativeRule
{
	Fallback = 0, Variable, Zero, RealIfThenElse,
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
};

//! the row of a code
const OperationInfo& OperationOfCode(OperationCode code);
//! the code of a node of a flat program by its kind, its name and the kinds of its operands
OperationCode OperationCodeOf(const FlatNode& node, const std::vector<FlatKind>& operandKinds);

//! the rule of the derivative of a node by its kind, its name and the kinds of its operands
int FindDerivativeRule(const FlatNode& node, const std::vector<FlatKind>& operandKinds);
//! the number of components of a value: 1, the size of a vector or rows*columns of a matrix
Index SlotComponents(const FlatNode& node, const FlatValue& value);
//! the forward derivative of one operation by its rule: its m components by the n components of the variable into out,
//! component c by component j at [c*n + j], from its value, the values v and the derivatives d of its operands
void ForwardDerivative(int rule, const FlatNode& node, const FlatValue& value, const FlatValue* v, const Real* const* d,
	Index m, Index n, Real* out, const void* variable, bool scalarVariable);

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
	//! after Evaluate(): the derivatives of all slots by the n components of the variable, as FlatProgram::Differentiate
	void Differentiate(const void* variable, Index n, bool scalarVariable) const;
	const Real* ResultDerivative(Index i) const { return derivatives.data() + derivativeOffsets[results[i]]; }

	Real Result(Index i) const { return slots[results[i]].real; }
	const ResizableConstVector& VectorResult(Index i) const { return *slots[results[i]].vector; }
	const ResizableMatrix& MatrixResult(Index i) const { return *slots[results[i]].matrix; }
	Index NumberOfSteps() const { return (Index)steps.size(); }
	//! the number of steps computed by the table, without a call of their node
	Index NumberOfTableSteps() const;
};

//! the evaluation of the user functions: True (default), by SymbolicProgram; False, by FlatProgram, the fallback (#3153)
extern bool symbolicProgramEvaluation;

} //namespace Symbolic

#endif //SYMBOLICTABLE__H
