/** ***********************************************************************************************
* @file			AllABasicLinalgUnitTests.h
* @brief		This file contains specific unit tests for all kinds of Exudyn array classes
* @details		Details:
                - tests on small sizes of arrays
                - order of tests according to order of member functions in according array class
                - all non-templated tests are placed here!

* @author		Gerstmayr Johannes
* @date			2021-12-10 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef ALLABASICLINALGUNITTESTS__H
#define ALLABASICLINALGUNITTESTS__H

#pragma once

const lest::test basic_linalg_poly_test[] = 
{
	//template<class TVector>
	//inline Real EvaluatePolynomial(const TVector& coeffs, Real x)

	//template<class TVector>
	//inline Real EvaluatePolynomial_x(const TVector& coeffs, Real x)

    CASE("EvaluatePolynomial<ConstVector<4>>: polynomial")
    {
		ConstSizeVector<4> coeffs({3.3,-1.2,0.7,0.121});
		Real x = 1.341245;
        EXPECT(3.3 - 1.2*x + 0.7*x*x + 0.121*x*x*x  == 
			EXUmath::EvaluatePolynomial(coeffs, x));

		EXPECT(-1.2 + 2.*0.7*x + 3.*0.121*x*x ==
			EXUmath::EvaluatePolynomial_x(coeffs, x));
    },
};

#endif
