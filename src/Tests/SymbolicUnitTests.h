/** ***********************************************************************************************
* @file			SymbolicUnitTests.h
* @brief		Unit tests for the symbolic expression system: Symbolic.h, SymbolicVector.h and
*				SymbolicMatrix.h.
* @details		Details:
*				- symbolicModuleTest.py already compares the NUMBERS against Python's math module,
*				  for every operator and function, with recording on and off. Repeating that here
*				  would add nothing, so these cases go where Python cannot: the expression TREE.
*				- Diff is barely covered anywhere: it is numeric forward-mode differentiation that
*				  identifies the variable by POINTER, so two variables that share a name are
*				  different variables.
*				- the value accessors, the non-recording path with named variables, the exact
*				  ToString() output and the reference counting are all unreachable from Python.
*				- a named variable is ALWAYS created as Symbolic::SReal(name, value), never as a
*				  stack ExpressionNamedReal: an operator node owns its operands and deletes them
*				  when their reference counter reaches zero, so a node on the stack is a heap
*				  corruption waiting to happen (it is exactly what happened while writing this).
*				- IMPORTANT: Symbolic keeps three globals - recordExpressions and the new/delete
*				  counters. symbolicModuleTest.py asserts new-delete==0, and in serial mode the
*				  models and RunCppUnitTests() share one interpreter, so every case here restores
*				  what it changed and leaves the counters balanced.
*
* @author		Gerstmayr Johannes
* @date			2026-09-17 (created; revision2026 step R5.4.2, issue #2479)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef SYMBOLICUNITTESTS__H
#define SYMBOLICUNITTESTS__H

//SymbolicVector.h and SymbolicMatrix.h use py::list, py::array_t and EPyUtils WITHOUT including
//anything for them - they compile only because Symbolic.cpp includes pybind11 first. Any other
//translation unit has to do the same, in the same order (#2480)
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "Pymodules/PybindUtilities.h"

#include "Linalg/Symbolic.h"
#include "Linalg/SymbolicVector.h"
#include "Linalg/SymbolicMatrix.h"

#include <cmath>

//! restores the three Symbolic globals when it goes out of scope, whatever a case did to them;
//! without this, a case would change what symbolicModuleTest.py measures (#2479)
class SymbolicTestGuard
{
	bool recording;
	int newCount;
	int deleteCount;
public:
	SymbolicTestGuard()
	{
		recording = Symbolic::SReal::GetRecording();
		newCount = Symbolic::ExpressionBase::newCount;
		deleteCount = Symbolic::ExpressionBase::deleteCount;
		Symbolic::SReal::SetRecording(true); //the cases are about the tree, so record by default
	}
	//! nodes allocated but not yet freed since this guard was created
	int OpenNodes() const
	{
		return (Symbolic::ExpressionBase::newCount - newCount)
			- (Symbolic::ExpressionBase::deleteCount - deleteCount);
	}
	~SymbolicTestGuard()
	{
		Symbolic::SReal::SetRecording(recording);
		Symbolic::ExpressionBase::newCount = newCount;
		Symbolic::ExpressionBase::deleteCount = deleteCount;
	}
};

const lest::test symbolic_specific_test[] =
{

	CASE("Symbolic: an expression follows its variable, it is not a snapshot")
	{
		SymbolicTestGuard guard;
		{
			Symbolic::SReal x("x", 2.);
			Symbolic::SReal f = x * x + Symbolic::SReal(3.);

			EXPECT(f.Evaluate() == 7.);          //2*2 + 3

			x.SetExpressionNamedReal(5.);        //the VARIABLE changes ...
			EXPECT(f.Evaluate() == 28.);         //... and the same expression answers anew: 5*5+3

			//the tree is what makes this work: a value copy would still say 7
			EXPECT(f.GetExpression() != nullptr);
			EXPECT(x.Evaluate() == 5.);
			EXPECT(x.IsExpressionNamedReal());
		}
		EXPECT(guard.OpenNodes() == 0); //no node leaked out of the scope
	},

	CASE("Symbolic: Diff identifies the variable by POINTER, not by name")
	{
		//this is the case Python cannot express: two variables that are both called 'x'
		SymbolicTestGuard guard;
		{
			Symbolic::SReal x("x", 3.);
			Symbolic::SReal other("x", 3.); //same NAME, different variable

			Symbolic::SReal f = x * x;
			EXPECT(f.DiffSReal(x) == 6.);        //d(x^2)/dx = 2x = 6
			EXPECT(f.DiffSReal(other) == 0.);    //f does not contain THAT variable, however it is named

			//a plain value has no derivative at all
			Symbolic::SReal c(7.);
			EXPECT(c.DiffSReal(x) == 0.);

			//and differentiating with respect to something that is not a variable is refused
			EXPECT_THROWS(f.DiffSReal(f));
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: the chain rule through nested nodes")
	{
		SymbolicTestGuard guard;
		{
			const Real xValue = 0.7;
			Symbolic::SReal x("x", xValue);

			//d/dx sin(x*x) = cos(x*x)*2x
			Symbolic::SReal f = Symbolic::SReal::sin(x * x);
			EXPECT(fabs(f.DiffSReal(x) - cos(xValue*xValue) * 2.*xValue) < 1e-14);

			//d/dx exp(x)/x = exp(x)/x - exp(x)/x^2
			Symbolic::SReal g = Symbolic::SReal::exp(x) / x;
			EXPECT(fabs(g.DiffSReal(x) - (exp(xValue) / xValue - exp(xValue) / (xValue*xValue))) < 1e-14);

			//d/dx x^3 = 3x^2
			Symbolic::SReal h = Symbolic::SReal::pow(x, 3.);
			EXPECT(fabs(h.DiffSReal(x) - 3.*xValue*xValue) < 1e-13);

			//the rules compose: the derivative of a sum is the sum of the derivatives
			Symbolic::SReal s = f + g;
			EXPECT(fabs(s.DiffSReal(x) - (f.DiffSReal(x) + g.DiffSReal(x))) < 1e-14);

			//and the derivative follows the variable, like the value does
			x.SetExpressionNamedReal(1.3);
			EXPECT(fabs(f.DiffSReal(x) - cos(1.3*1.3) * 2.*1.3) < 1e-14);
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: what Diff REFUSES and what it answers with NaN")
	{
		//two different answers to 'not differentiable here', and both are deliberate
		SymbolicTestGuard guard;
		{
			Symbolic::SReal x("x", 0.5);

			//THROWS: these have no derivative at all
			Symbolic::SReal rounded = Symbolic::SReal::round(x);
			EXPECT_THROWS(rounded.DiffSReal(x));
			Symbolic::SReal floored = Symbolic::SReal::floor(x);
			EXPECT_THROWS(floored.DiffSReal(x));
			Symbolic::SReal smaller = (x < Symbolic::SReal(1.));
			EXPECT_THROWS(smaller.DiffSReal(x));
			Symbolic::SReal minimum = Symbolic::SReal::min(x, Symbolic::SReal(1.));
			EXPECT_THROWS(minimum.DiffSReal(x));

			//NaN: the function exists, the derivative does not exist AT THIS POINT
			Symbolic::SReal z("z", 0.);
			Symbolic::SReal absolute = Symbolic::SReal::abs(z);
			EXPECT(std::isnan(absolute.DiffSReal(z)));      //|z| has no derivative at 0
			Symbolic::SReal root = Symbolic::SReal::sqrt(z);
			EXPECT(std::isnan(root.DiffSReal(z)));          //sqrt has an infinite slope at 0

			//sign is the odd one out: its derivative is 0 everywhere, including at 0
			Symbolic::SReal sgn = Symbolic::SReal::sign(z);
			EXPECT(sgn.DiffSReal(z) == 0.);
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: ToString prints the tree, and says two surprising things")
	{
		SymbolicTestGuard guard;
		{
			Symbolic::SReal x("x", 2.);
			Symbolic::SReal y("y", 3.);

			EXPECT(x.ToString() == "x");                       //a named variable prints its NAME
			EXPECT((x + y).ToString() == "(x + y)");
			EXPECT((x * y).ToString() == "(x * y)");
			EXPECT(Symbolic::SReal::sqrt(x).ToString() == "sqrt(x)");
			EXPECT(Symbolic::SReal::Not(x).ToString() == "Not(x)"); //capital N: 'not' is a keyword

			//SURPRISE 1: unary plus prints NOTHING - the node is transparent
			EXPECT((+x).ToString() == "x");

			//SURPRISE 2: a comparison is an EXPRESSION returning 0.0 or 1.0, not a bool
			Symbolic::SReal isSmaller = (x < y);
			EXPECT(isSmaller.Evaluate() == 1.);
			EXPECT((y < x).Evaluate() == 0.);
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: recording off is a SECOND implementation of every operator")
	{
		//every operator has an 'if (!recordExpressions)' branch that computes immediately; the two
		//paths share no code, so the only thing that ties them together is a test like this
		SymbolicTestGuard guard;
		const Real a = 1.7, b = -0.35;

		Real sum = 0., product = 0., function = 0., compare = 0.;
		{
			Symbolic::SReal::SetRecording(true);
			Symbolic::SReal x(a), y(b);
			sum = (x + y).Evaluate();
			product = (x * y - x / y).Evaluate();
			function = (Symbolic::SReal::sin(x) + Symbolic::SReal::exp(y)).Evaluate();
			compare = (x > y).Evaluate();
		}

		{
			Symbolic::SReal::SetRecording(false);
			Symbolic::SReal x(a), y(b);
			EXPECT((x + y).Evaluate() == sum);
			EXPECT((x * y - x / y).Evaluate() == product);
			EXPECT((Symbolic::SReal::sin(x) + Symbolic::SReal::exp(y)).Evaluate() == function);
			EXPECT((x > y).Evaluate() == compare);

			//and with recording off, a named variable is just a value: no tree is built at all
			Symbolic::SReal named("x", a);
			EXPECT(named.GetExpression() == nullptr);
			EXPECT(!named.IsExpressionNamedReal());
			EXPECT(named.Evaluate() == a);
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: the value accessors refuse as soon as an expression is present")
	{
		SymbolicTestGuard guard;
		{
			Symbolic::SReal plain(2.5);
			EXPECT(plain.GetValue() == 2.5);       //no expression: the value is right there
			plain.SetValue(-1.);
			EXPECT(plain.Evaluate() == -1.);

			Symbolic::SReal x("x", 2.);
			Symbolic::SReal compound = x + Symbolic::SReal(1.);

			//a compound expression has no single 'value' to read or write
			EXPECT_THROWS(compound.GetValue());
			EXPECT_THROWS(compound.SetValue(3.));
			EXPECT_THROWS(compound.SetSymbolicValue(3.));

			//a named variable is the exception: SetSymbolicValue reaches THROUGH to the node
			x.SetSymbolicValue(4.);
			EXPECT(compound.Evaluate() == 5.);
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: a vector of expressions, built the C++ way")
	{
		//SetSRealVector takes an initializer list; the pybind equivalent is deliberately not bound,
		//so this constructor had no caller before
		SymbolicTestGuard guard;
		{
			Symbolic::SReal x("x", 2.);

			Symbolic::SymbolicRealVector v;
			v.SetSRealVector({ x, x * Symbolic::SReal(2.), Symbolic::SReal(3.) });
			EXPECT(v.NumberOfItems() == 3);

			ResizableConstVector evaluated = v.Evaluate();
			EXPECT(evaluated[0] == 2.);
			EXPECT(evaluated[1] == 4.);
			EXPECT(evaluated[2] == 3.);

			//the vector follows the variable, exactly as the scalar does
			x.SetExpressionNamedReal(10.);
			evaluated = v.Evaluate();
			EXPECT(evaluated[0] == 10.);
			EXPECT(evaluated[1] == 20.);
			EXPECT(evaluated[2] == 3.);

			//dot product and norm return a SCALAR expression
			Symbolic::SymbolicRealVector w(Vector({ 1., 0., 2. }));
			Symbolic::SReal dot = v * w;
			EXPECT(dot.Evaluate() == 10.*1. + 20.*0. + 3.*2.);

			Symbolic::SymbolicRealVector unit(Vector({ 3., 4. }));
			EXPECT(fabs(unit.NormL2().Evaluate() - 5.) < 1e-14);
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: a size mismatch is reported while the expression is BUILT")
	{
		//worth pinning down, because it is not obvious: SReal(ExpressionBase*) evaluates the node
		//immediately to cache its value, so building the product already runs it, and the size
		//check inside Evaluate() fires at the construction site rather than at first use
		SymbolicTestGuard guard;
		{
			Symbolic::SymbolicRealVector two("two", Vector({ 1., 2. }));
			Symbolic::SymbolicRealVector three("three", Vector({ 1., 2., 3. }));

			EXPECT_THROWS((void)(two * three));
		}
		//NOTE: OpenNodes() is deliberately NOT required to be 0 here - the exception escapes
		//before any SReal takes ownership of the new node, so the failed product leaks it (#2481)
	},

	CASE("Symbolic: a matrix answers component by component, without evaluating itself")
	{
		//EvaluateComponent is not bound to Python at all; it is how the matrix nodes ask each
		//other for single entries
		SymbolicTestGuard guard;
		{
			Matrix m(2, 2, { 1., 2., 3., 4. });

			//NOTE: SymbolicRealMatrix(const Matrix&) stores the VALUES and builds no node at all -
			//only the NAMED constructor records one. Worth knowing before asking for the tree
			Symbolic::SymbolicRealMatrix plain(m);
			EXPECT(plain.GetExpression() == nullptr);

			Symbolic::SymbolicRealMatrix a("A", m);
			EXPECT(a.NumberOfRows() == 2);
			EXPECT(a.NumberOfColumns() == 2);

			ResizableMatrix evaluated = a.Evaluate();
			EXPECT(evaluated(0, 0) == 1.);
			EXPECT(evaluated(1, 1) == 4.);

			//the same questions through the expression node itself
			Symbolic::MatrixExpressionBase* node = a.GetExpression();
			EXPECT(node != nullptr);
			EXPECT(node->NumberOfRows() == 2);
			EXPECT(node->EvaluateComponent(0, 1) == 2.);
			EXPECT(node->EvaluateComponent(1, 0) == 3.);

			//a product asks its operands for components, so the result must agree with the plain
			//matrix product of the same values
			Symbolic::SymbolicRealMatrix product = a * a;
			ResizableMatrix productEvaluated = product.Evaluate();
			EXPECT(productEvaluated(0, 0) == 1.*1. + 2.*3.);
			EXPECT(productEvaluated(0, 1) == 1.*2. + 2.*4.);
			EXPECT(productEvaluated(1, 0) == 3.*1. + 4.*3.);
			EXPECT(productEvaluated(1, 1) == 3.*2. + 4.*4.);
		}
		EXPECT(guard.OpenNodes() == 0);
	},

	CASE("Symbolic: copies SHARE the tree, and the last one standing frees it")
	{
		//the reference counting is hand-rolled, with the safety check commented out, so this is
		//the only place that states what it promises
		SymbolicTestGuard guard;
		{
			Symbolic::SReal x("x", 2.);

			Symbolic::SReal f = x + Symbolic::SReal(1.);
			Symbolic::ExpressionBase* tree = f.GetExpression();
			const int referencesBefore = tree->ReferenceCounter();
			{
				Symbolic::SReal alias = f;                  //a copy does NOT copy the tree
				EXPECT(alias.GetExpression() == tree);
				EXPECT(tree->ReferenceCounter() == referencesBefore + 1);

				//writing through the shared variable is visible in both
				x.SetExpressionNamedReal(9.);
				EXPECT(alias.Evaluate() == 10.);
				EXPECT(f.Evaluate() == 10.);
			}
			EXPECT(tree->ReferenceCounter() == referencesBefore); //the copy released its reference
		}
		EXPECT(guard.OpenNodes() == 0); //and the whole tree is gone with the last owner
	},

};

#endif //include header once
