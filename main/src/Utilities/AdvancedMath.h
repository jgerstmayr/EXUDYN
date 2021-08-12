/** ***********************************************************************************************
* @brief		namespace ExuMath
*				advanced math operations, e.g., on polynomials
*
* @author		Manzl Peter, Gerstmayr Johannes
* @date			2021-08-11 (generated)
* @date			2021-08-11 (last modified)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* 
*
************************************************************************************************ */
#ifndef ADVANCEDMATH__H
#define ADVANCEDMATH__H

namespace ExuMath {

	/* ComputePolynomialDerivative: Calculate the coefficients of the derivative of a given polynomial
	*	poly:		The coefficients of a polynomial poly[0]x^(n-1) + poly[1]x^(n-2) + ... + poly[n-1]
	*	output:		The coefficients of the derivative of the polynomial with coefficients poly.
	*/
	inline void ComputePolynomialDerivative(const Vector& poly, Vector& dpoly) {
		Index n = poly.NumberOfItems();
		dpoly.SetNumberOfItems(n - 1);
		for (int i = 0; i < (n - 1); i++) {
			dpoly[i] = (n - i - 1)*poly[i];
		}
	}

	/* horners method calculates poly[0]x(n-1) + poly[1]x(n-2) + .. + poly[n-1]
	*   poly        the coefficients of the polynomial
	*   n           order of the polynomial
	*   x           value of x for which the polynomial is evaluated
	*/
	inline Real Polyval(const Vector& poly, Real x)
	{
		Real result = poly[0]; // init result
		for (Index i = 1; i < poly.NumberOfItems(); i++)
		{
			result = result * x + poly[i];
		}
		return result;
	}


	/* rootOfPolynomial: returns the root of poly[0]x^(n-1) + poly[1]x^(n-2) + ... + poly[n-1] - offset = 0
	*               For strictly monotonous polynomials only a single solution exists, otherwise one root
	*               near the starting value x_0 will be found.
	*   poly        the coefficients of the polynomial
	*   x_0         starting value for the algorithm
	*   offset      set an offset to find the root of (polynomial-offset)
	*   xBoundary   the minimum and maximum value of x
	*   tol         numerical tolerance in which the solution must lie
	*   maxIter     maximum number of iterations for the algorithm after which it stops
	*   iterations  returns iterations needed
	*/
	inline Real RootOfPolynomial(const Vector& poly, const Vector& dpoly, Real x_0, Real offset, Real xBoundary[], Real tol,
		Index maxIter, Real maxStep, Index& iterations)
	{
		iterations = 0;
		Real x = x_0;
		Real err = -Polyval(poly, x) - offset;
		Real poly_ = -Polyval(dpoly, x);
		Real dx = 0;


		while (iterations < maxIter && std::fabs(err) > tol)
		{
			err = -Polyval(poly, x) - offset;
			poly_ = -Polyval(dpoly, x);
			dx = err / poly_;
			if (std::fabs(dx) > maxStep)
			{
				dx = maxStep * std::fabs(dx) / dx;
			}
			x = x - dx; // err/polyval(dpoly, n-1, x);  
			iterations++;
			// std::cout << "err=" << std::to_string(err) << "\t x=" << std::to_string(x) <<std::endl;
		}
		// std::cout << "took iterations=" << std::to_string(iterations) << "\t iterations" << std::endl;    // for debugging
		return x;
	}








} //namespace ExuMath
#endif
