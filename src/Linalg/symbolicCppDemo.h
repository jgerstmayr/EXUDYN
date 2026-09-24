/** ***********************************************************************************************
* @file			symbolicCppDemo.h
* @brief		DEMONSTRATION of how the symbolic types are used from C++. Nothing calls this.
* @details		Details:
*				- WHAT THIS IS: worked examples of Symbolic::SReal, SymbolicRealVector and
*				  SymbolicRealMatrix, written to be READ. Python users see the symbolic module
*				  through exudyn.symbolic and its test models; this file is the C++ side, which has
*				  no other documentation.
*				- WHAT THIS IS NOT: it is not a test and it checks nothing. The tests are in
*				  src/Tests/SymbolicUnitTests.h, and the numeric
*				  comparison against Python lives in python/TestModels/symbolicModuleTest.py.
*				- NOTHING CALLS IT, deliberately. It is included by Symbolic.cpp so that it keeps
*				  COMPILING - a demo that no longer compiles is worse than no demo - but the
*				  functions are inline and unused, so they cost nothing in the module. Call
*				  SymbolicDemoAll() from a debugger or a scratch main() to watch the output.
*				- HISTORY: this was PyTest_unused() at the end of Symbolic.cpp, a single function of
*				  six 'if (false)' blocks that had not been run in a long time. Reshaped into named
*				  functions on maintainer request (#2485).
*
* @author		Gerstmayr Johannes
* @date			2026-09-17 (reshaped from PyTest_unused in Symbolic.cpp)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef SYMBOLICCPPDEMO__H
#define SYMBOLICCPPDEMO__H

#include "Linalg/Symbolic.h"
#include "Linalg/SymbolicVector.h"
#include "Linalg/SymbolicMatrix.h"

#include <iostream>

//! ONE RULE BEFORE READING ON: a named variable is created as Symbolic::SReal("x", value), never as
//! a stack Symbolic::ExpressionNamedReal handed to SReal(&node). An operator node OWNS its operands
//! and deletes them when their reference counter reaches zero, so a node that lives on the stack is
//! freed with delete sooner or later - heap corruption, and hard to trace. The original of this file
//! used the stack form; it survived only because its expressions were never destroyed in the wrong
//! order. See src/Tests/SymbolicUnitTests.h, which says the same thing.

//! the two building blocks: a plain value carries no expression, a named variable does
inline void SymbolicDemoPlainValues()
{
	std::cout << "\n--- plain values ---\n";
	Symbolic::SReal::SetRecording(false); //no tree is built; every operator computes at once

	Symbolic::SReal a = 5.;
	Symbolic::SReal b(3.);
	Symbolic::SReal c = 2. + (a + b) * 7.;

	std::cout << "c            = " << c.Evaluate() << "   (expected 58)\n";
	std::cout << "c.ToString() = " << c.ToString() << "   (the VALUE: there is no expression)\n";

	Symbolic::SReal::SetRecording(true);
}

//! the same expression, recorded: now it is a tree that can be printed and re-evaluated
inline void SymbolicDemoNamedVariable()
{
	std::cout << "\n--- a named variable and its expression ---\n";
	Symbolic::SReal::SetRecording(true); //turn on expression recording

	Symbolic::SReal x("x", 42.);
	Symbolic::SReal b(3.);
	Symbolic::SReal c = 2. + (x + b) * 7.;

	std::cout << "c            = " << c.Evaluate() << "\n";
	std::cout << "c.ToString() = " << c.ToString() << "   (the TREE, with the variable by name)\n";

	//the point of recording: the same expression answers again after the variable changes
	x.SetExpressionNamedReal(1.);
	std::cout << "x := 1  ->  c = " << c.Evaluate() << "   (nothing was rebuilt)\n";
}

//! vectors of expressions: built from a Vector, from single expressions, or from other vectors
inline void SymbolicDemoVectors()
{
	std::cout << "\n--- symbolic vectors ---\n";
	typedef Symbolic::SReal SR;
	typedef Symbolic::SymbolicRealVector SRV;
	Symbolic::SReal::SetRecording(true);

	SR a("a", 42.);
	SR b("b", 2.5);
	SR three(3.);

	SRV c = Vector({ 3.3, 2.2, 1.1 });          //from plain values
	std::cout << "c  = " << c << " = " << c.Evaluate() << "\n";

	SRV d = three * c;                          //scalar times vector
	std::cout << "d  = " << d << " = " << d.Evaluate() << "\n";

	SR e = c * c;                               //vector times vector is the SCALAR product
	std::cout << "e  = " << e << " = " << e.Evaluate() << "\n";

	//a vector of EXPRESSIONS; this constructor exists only in C++ (it is not bound to Python)
	SRV v1;
	v1.SetSRealVector({ a, b, SR(3.1) });
	SRV v2;
	v2.SetSRealVector({ a / 2., b + 1., SR(3.) });

	SRV v3 = -v1 + 2. * v2;
	std::cout << "v1 = " << v1 << " = " << v1.Evaluate() << "\n";
	std::cout << "v2 = " << v2 << " = " << v2.Evaluate() << "\n";
	std::cout << "v3 = " << v3 << " = " << v3.Evaluate() << "\n";

	//scalar functions and vector expressions mix freely
	SRV weights(Vector({ 0.5 * EXUstd::pi / 42., 0., 0. }));
	SRV v4 = -v1 + 2. * v2 + SR::sin(v1 * weights) * v1;
	std::cout << "v4 = " << v4 << " = " << v4.Evaluate() << "\n";

	//and everything follows the variables it was built from
	a.SetExpressionNamedReal(0.);
	std::cout << "a := 0  ->  v1 = " << v1.Evaluate() << "\n";
}

//! matrices of expressions, and the component access the matrix nodes use internally
inline void SymbolicDemoMatrices()
{
	std::cout << "\n--- symbolic matrices ---\n";
	Symbolic::SReal::SetRecording(true);

	Symbolic::SymbolicRealMatrix A("A", Matrix(2, 2, { 1., 2., 3., 4. }));
	std::cout << "A          = " << A << " = " << A.Evaluate() << "\n";

	Symbolic::SymbolicRealMatrix B = A * A;
	std::cout << "A*A        = " << B.Evaluate() << "\n";

	Symbolic::SymbolicRealMatrix C = Symbolic::SReal(2.) * A;
	std::cout << "2*A        = " << C.Evaluate() << "\n";

	//NOTE: SymbolicRealMatrix(const Matrix&) - without a name - stores the VALUES and builds no
	//node, so GetExpression() is then nullptr. The named constructor above records one
	std::cout << "A has a tree: " << (A.GetExpression() != nullptr) << "\n";
	std::cout << "A(0,1)     = " << A.GetExpression()->EvaluateComponent(0, 1) << "\n";
}

//! Diff: numeric forward-mode differentiation, with respect to ONE named variable
inline void SymbolicDemoDiff()
{
	std::cout << "\n--- derivatives ---\n";
	typedef Symbolic::SReal SR;
	Symbolic::SReal::SetRecording(true);

	SR x("x", 3.);
	SR one(1.);

	SR f = one * (5. - x) * SR::tan(x) + SR::atan2(x, 1.) + SR::abs(x) + SR::sin(x) * SR::cos(x) * (x + 2.);
	std::cout << "f          = " << f.ToString() << "\n";
	std::cout << "f(3)       = " << f.Evaluate() << "\n";
	std::cout << "df/dx (3)  = " << f.DiffSReal(x) << "\n";

	//Diff identifies the variable by POINTER: another variable of the same name is a different one
	SR otherX("x", 3.);
	std::cout << "df/d(other x) = " << f.DiffSReal(otherX) << "   (0: f does not contain it)\n";

	//not every node can be differentiated: round, floor, mod, min, max and the comparisons throw,
	//and the singular points of abs, sqrt, asin, ... answer with NaN. See SymbolicUnitTests.h
}

//! the scalar functions, printed one by one - the quickest way to see what ToString() produces
inline void SymbolicDemoFunctions()
{
	std::cout << "\n--- the scalar functions ---\n";
	typedef Symbolic::SReal SR;
	Symbolic::SReal::SetRecording(true);

	SR a("a", 5.);
	SR b("b", EXUstd::pi);
	SR c("c", 7.42);

	SR f;
	f = a + b - a * c / c;    std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::sin(b);           std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::cos(b);           std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::tan(b);           std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::sign(a);          std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::sign(-a);         std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::min(a, b);        std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::max(a, b);        std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::round(c);         std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::round(-c);        std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::round(c + 0.3);   std::cout << f.ToString() << " = " << f.Evaluate() << "\n";

	//the answers at the edges are worth knowing: atan2(1, 0) is defined, 1/0 is not finite, and
	//sqrt(pow(x,2)) is |x|
	f = SR::atan2(1., c - c);             std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::sqrt(SR::pow(a - a + 4.33, 2.)); std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::isfinite(1. / (a - a));       std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
	f = SR::isfinite(1. * (a - a));       std::cout << f.ToString() << " = " << f.Evaluate() << "\n";
}

//! what an expression costs: evaluating a recorded tree against rebuilding it every time
inline void SymbolicDemoTiming(Index numberOfRuns = 1000000)
{
	std::cout << "\n--- cost of an expression ---\n";
	Symbolic::SReal::SetRecording(true);

	Symbolic::SReal a(5.), b(3.), d(7.);
	Symbolic::SReal e = d * (a + 8.) + 3. * b * 2.;

	std::cout << "e            = " << e.ToString() << " = " << e.Evaluate() << "\n";

	//RECORDED: the tree exists, and evaluating it walks the nodes
	Real time = -EXUstd::GetTimeInSeconds();
	Real sum = 0.;
	for (Index i = 0; i < numberOfRuns; i++) { sum += e.Evaluate(); }
	time += EXUstd::GetTimeInSeconds();
	std::cout << "evaluate     : " << time / numberOfRuns * 1e9 << " ns per evaluation (sum=" << sum << ")\n";

	//REBUILT: with recording ON this allocates nodes every time, which is the expensive case
	time = -EXUstd::GetTimeInSeconds();
	sum = 0.;
	for (Index i = 0; i < numberOfRuns; i++)
	{
		Symbolic::SReal f = d * (a + 8.) + 3. * b * 2.;
		sum += f.Evaluate();
	}
	time += EXUstd::GetTimeInSeconds();
	std::cout << "build+eval   : " << time / numberOfRuns * 1e9 << " ns per expression (sum=" << sum << ")\n";

	//with recording OFF the same line is plain arithmetic - this is what a user function does
	//once it has been converted
	Symbolic::SReal::SetRecording(false);
	time = -EXUstd::GetTimeInSeconds();
	sum = 0.;
	for (Index i = 0; i < numberOfRuns; i++)
	{
		Symbolic::SReal f = d * (a + 8.) + 3. * b * 2.;
		sum += f.Evaluate();
	}
	time += EXUstd::GetTimeInSeconds();
	std::cout << "no recording : " << time / numberOfRuns * 1e9 << " ns per expression (sum=" << sum << ")\n";
	Symbolic::SReal::SetRecording(true);
}

//! how the memory is accounted: every node is counted when it is allocated and when it is freed,
//! and the two counts must meet once the expressions are gone
inline void SymbolicDemoReferenceCounting()
{
	std::cout << "\n--- references and memory ---\n";
	Symbolic::SReal::SetRecording(true);

	const int newBefore = Symbolic::ExpressionBase::newCount;
	const int deleteBefore = Symbolic::ExpressionBase::deleteCount;
	{
		Symbolic::SReal a("a", 5.);
		Symbolic::SReal b("b", 3.);
		Symbolic::SReal x = a + 2. * b;

		std::cout << "x            = " << x.ToString() << "\n";
		std::cout << "references of the tree of x: " << x.GetExpression()->ReferenceCounter() << "\n";
		{
			Symbolic::SReal alias = x;   //a copy SHARES the tree, it does not copy it
			std::cout << "with a copy alive         : " << x.GetExpression()->ReferenceCounter() << "\n";
			std::cout << "alias                     = " << alias.Evaluate() << "\n";
		}
		std::cout << "after the copy is gone    : " << x.GetExpression()->ReferenceCounter() << "\n";
	}
	std::cout << "allocated here: " << (Symbolic::ExpressionBase::newCount - newBefore)
		<< ", freed here: " << (Symbolic::ExpressionBase::deleteCount - deleteBefore)
		<< "   (they must be equal)\n";
}

//! run every demonstration above; not called anywhere - call it from a debugger or a scratch main()
inline void SymbolicDemoAll()
{
	const bool recording = Symbolic::SReal::GetRecording(); //this is a GLOBAL; put it back afterwards

	SymbolicDemoPlainValues();
	SymbolicDemoNamedVariable();
	SymbolicDemoVectors();
	SymbolicDemoMatrices();
	SymbolicDemoDiff();
	SymbolicDemoFunctions();
	SymbolicDemoTiming();
	SymbolicDemoReferenceCounting();

	std::cout << "\nnewCount    = " << Symbolic::ExpressionBase::newCount << "\n";
	std::cout << "deleteCount = " << Symbolic::ExpressionBase::deleteCount << "\n";

	Symbolic::SReal::SetRecording(recording);
}

#endif //include header once
