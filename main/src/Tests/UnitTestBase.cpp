/** ***********************************************************************************************
* @file			UnitTests.cpp
* @brief		This file contains implementation of UnitTestBase class, handles Exudyn unit tests
* @author		Gerstmayr Johannes
* @date			2 May 2018
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

#include "Tests/UnitTestBase.h" 

#ifdef PERFORM_UNIT_TESTS
using namespace std;
#define PerformUnitTests

//enable memory leak checks by leak number {xxx}
#ifdef _DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

//put this at beginning of your code:
//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

// *** OR: *** 
//int flag = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);
//flag |= _CRTDBG_LEAK_CHECK_DF; // This turns on the leak-checking bit
//_CrtSetDbgFlag(flag);

//_CrtSetBreakAlloc(366); //this will lead to a break-point where the memory leak happens
#endif


#include "lest/lest.hpp"

#include "Linalg/BasicLinalg.h"	//includes all necessary math libraries to test
#include "Utilities/ObjectContainer.h" 

using namespace EXUstd;
using namespace EXUmath;


#ifdef PerformUnitTests
    #include "AllVectorUnitTests.h" 
    #include "AllArrayUnitTests.h" 
    #include "AllMatrixUnitTests.h" 
    #include "TemplatedVectorArrayUnitTests.h" 
	#include "AllBasicLinalgUnitTests.h"
#endif

extern bool linalgPrintUsePythonFormat; //!< true: use python format for output of vectors and matrices; false: use matlab format

//run all unit tests; return 0 on success, otherwise the number of fails
Index RunUnitTests(bool reportOnPass=false, bool printOutput=true)
{
#ifdef _DEBUG
	//enable memory leak checks by leak number {xxx}
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(366); //this will lead to a break-point where the memory leak happens
#endif

	bool oldPrintFormat = linalgPrintUsePythonFormat;
	linalgPrintUsePythonFormat = false; //tests have been defined in the old format
	//int returnValue = lest::run(specification, { "-p", "-v" }, pout);
	//int returnValue = lest::run(specification, {"-p", "-v"}, file); //option "-p" to also report on passing tests!; option "-v" also to report on sections


	UnitTestBase tests;
	Index returnValue = (Index)tests.PerformVectorAndArrayTests(reportOnPass * UnitTestFlags::reportOnPass + UnitTestFlags::reportSections
		+ printOutput * UnitTestFlags::writeToPout + 0 * UnitTestFlags::writeToFile);

	linalgPrintUsePythonFormat = oldPrintFormat; //set back to original user-defined format

	if (returnValue && printOutput) 
	{ 
		pout << "unit tests FAILED\n"; 
		pout << "Number of failed tests=" << returnValue << "\n***********************\n\n";
	}
	if (!returnValue && printOutput) { pout << "unit tests SUCCESSFUL\n"; }


	if (0) //write output to file 
	{
		ofstream testfile("UnitTestsOutput.txt");

		testfile << "Exudyn UNIT TESTS\n";
		testfile << "*****************\n\n";
		testfile << tests.GetOutputString().c_str() << "\n";
	}

#ifdef _DEBUG
	_CrtDumpMemoryLeaks(); //dump memory leaks to Visual Studio after exit
#endif
	return returnValue;
}

//add these tests to test suite:
	//++++++++++++++++++++++++++++++++++++++++++
	//test linalg

	//Matrix A23(2, 3, { 1,2,3,4,5,6 });
	//Matrix A32(3, 2, { 1,2,3,4,5,6 });
	//Vector v2({ 1,2 });
	//Vector v3({ 2,3,4 });
	//Vector result;

	//pout << "A23*v3=" << A23 * v3 << "\n";
	//EXUmath::MultMatrixVector(A23, v3, result);
	//pout << "A23*v3=" << result << "\n";

	//pout << "A23^T*v2=" << A23.GetTransposed() * v2 << "\n";
	//EXUmath::MultMatrixTransposedVector(A23, v2, result);
	//pout << "A23^T*v2=" << result << "\n";

	//ResizableVector v1({ 1,2,3 });
	//ResizableVector v2({ 1,2,3 });

	//Vector w = v1 + v2;
	//ResizableVector v;// = v1 + v2;
	//v = v1;
	//v *= 4.;
	//v = v1 + v2;

	//pout << v1 + v2 << "\n";
	//pout << v1 - v2 << "\n";
	//pout << v1 * v2 << "\n";

	//std::thread th(GlfwRenderer::SetupRenderer);


	//++++++++++++++++++++++++++++++++++++++++++++
	//test projection:
	//Vector2D lp0({ 0, 2.5 });
	//Vector2D lp1({ 1, 3.5 });
	//Real d;
	//Vector2D p({ 0.5, 2.5 });
	//Vector2D pold = p;

	//d = HGeometry::ShortestDistanceEndPointsProject(lp0, lp1, p);
	//pout << "p=" << p << ", d=" << d << ", D=" << (p-pold).GetL2Norm() << "\n";

	//p = Vector2D({ 0,0 }); pold = p;
	//d = HGeometry::ShortestDistanceEndPointsProject(lp0, lp1, p);
	//pout << "p=" << p << ", d=" << d << ", D=" << (p - pold).GetL2Norm() << "\n";

	//p = Vector2D({ 3.5,0 }); pold = p;
	//d = HGeometry::ShortestDistanceEndPointsProject(lp0, lp1, p);
	//pout << "p=" << p << ", d=" << d << ", D=" << (p - pold).GetL2Norm() << "\n";

	//p = Vector2D({ 4.5,0 }); pold = p;
	//d = HGeometry::ShortestDistanceEndPointsProject(lp0, lp1, p);
	//pout << "p=" << p << ", d=" << d << ", D=" << (p - pold).GetL2Norm() << "\n";

	//p = Vector2D({ 0, 3.5 }); pold = p;
	//d = HGeometry::ShortestDistanceEndPointsProject(lp0, lp1, p);
	//pout << "p=" << p << ", d=" << d << ", D=" << (p - pold).GetL2Norm() << "\n";

	//lp0 = Vector2D({ 1,2 });
	//lp1 = Vector2D({ 3,2 });
	//Real relPos;

	//p = Vector2D({ 0, 0 });
	//d = HGeometry::ShortestDistanceEndPointsRelativePosition(lp0, lp1, p, relPos);
	//pout << "p=" << p << ", d=" << d << ", relpos=" << relPos << "\n";

	//p = Vector2D({ 1.5, 0 });
	//d = HGeometry::ShortestDistanceEndPointsRelativePosition(lp0, lp1, p, relPos);
	//pout << "p=" << p << ", d=" << d << ", relpos=" << relPos << "\n";

	//p = Vector2D({ 3.1, 0 });
	//d = HGeometry::ShortestDistanceEndPointsRelativePosition(lp0, lp1, p, relPos);
	//pout << "p=" << p << ", d=" << d << ", relpos=" << relPos << "\n";







//detect memory leaks for ObjectContainer
//! @todo check with VS2017 if memory leaks are solved for typeinfo / lest.hpp
void CheckMemoryLeaksObjectPointerArray()
{
    if(1)
    {
        ObjectContainer<Real> areal;

        Real r[5] = { 1.1,2,3,4,5 };

        areal.Append(r[0]);
        areal.Append(r[1]);
        areal.Append(r[2]);

        areal.Insert(0, 8.8);

        areal.Insert(2, 7.);

        areal.Remove(1);

        areal.Remove(3);

        areal.Remove(0);

        areal.Flush();

        areal.Append(r[0]);
        areal.Append(r[1]);
        areal.Append(r[2]);
        pout << areal << "\n";

        areal.Flush();
        pout << areal << "\n";
    }
    if(1)
    {
        ObjectContainer<Vector> avector;

        Vector v1({ 1.,2.,3. });
        Vector v2({ 2.5,3.,4. });
        Vector v3({ 4.,6.,8. });

        avector.Append(v1);
        avector.Append(v2);
        avector.Append(v3);

        avector.Remove(1);
        pout << avector << "\n";
    }
    if (1)
    {
        ObjectContainer<Vector> avector({ { 1., 2. },{ 3.3 },{ 4., 5. } });
        Vector v1({ 1. });
        Vector v2({ 2.2, 3. });

        avector.Insert(0, v1);
        avector.Insert(2, v2);

        avector.Remove(1);
        avector.Remove(2);
        avector.Remove(2);
        pout << avector << "\n";

        avector.Flush();

        int n = 100;
        avector.SetMaxNumberOfItems(n);
        for (int i=0; i < n; i++)
        {
            avector.Append(Vector({ 1.,2.,3.,4. }));
        }
        avector.SetMaxNumberOfItems(50);

        avector.SetMaxNumberOfItems(0); //check memory leakage due to SetMaxNumberOfItems

    }
    if (1)
    {
        ObjectContainer<Real> areal({ 1, 2, 3.3 });
        ObjectContainer<Real> areal2 = areal;

        ObjectContainer<Vector> avector({ { 1., 2. },{ 3., 4., 5.5 } });
        ObjectContainer<Vector> avector2;
        ObjectContainer<Vector> avector3;
        avector2 = avector;
        avector3 = avector;
        avector3[1][2] = 7.7;
        pout << ToString(avector3) << "\n";

        pout << "areal == areal2" << (areal == areal2) << "\n";
        pout << "avector == avector2" << (avector == avector2) << "\n";
    };
}


int UnitTestBase::PerformVectorAndArrayTests(int flags)
{
    //CheckMemoryLeaksObjectPointerArray(); //additional memory leak check

    int failCounter = 0; //lest returns 0 if all passed, 1 if at least one failed

    std::ostringstream stringStream; //instead of pout

    stringStream << "******************\nExudyn unit tests\n******************\n";

    lest::texts arglist_lest = {};
    //lest::texts arglist_lest = { "-p", "-v" };

    if (flags & UnitTestFlags::reportOnPass) { arglist_lest.push_back("-p"); }
    if (flags & UnitTestFlags::reportSections) { arglist_lest.push_back("-v"); }

#ifdef PerformUnitTests
    //array and array_vector tests:
    failCounter += lest::run(vectorarray_templated_tests, arglist_lest, stringStream);

    failCounter += lest::run(array_templated_tests, arglist_lest, stringStream);

    failCounter += lest::run(resizableArray_specific_test, arglist_lest, stringStream);

    failCounter += lest::run(slimArray_specific_test, arglist_lest, stringStream);

    failCounter += lest::run(objectContainer_specific_test, arglist_lest, stringStream);
    
    //vector tests:
    failCounter += lest::run(vector_templated_tests, arglist_lest, stringStream);

    failCounter += lest::run(vector_specific_test, arglist_lest, stringStream);

    failCounter += lest::run(constSizeVector_specific_test, arglist_lest, stringStream);

    failCounter += lest::run(SlimVector_specific_test, arglist_lest, stringStream);

    failCounter += lest::run(linkedDataVector_specific_test, arglist_lest, stringStream);

    failCounter += lest::run(resizableVector_specific_test, arglist_lest, stringStream);

    //matrix tests:
    failCounter += lest::run(matrix_specific_test, arglist_lest, stringStream);

	//basiclinalg tests:
	failCounter += lest::run(basic_linalg_poly_test, arglist_lest, stringStream);

#endif
    if (failCounter) {
        stringStream << "\n\n\n***********************\n AT LEAST ONE TEST FAILED!\n***********************\n\n";
    }
    else {
        stringStream << "\n\n\n***********************\n ALL TESTS PASSED!\n***********************\n\n";
    }

    testOutput += stringStream.str();

    if (flags & UnitTestFlags::writeToPout) { pout << testOutput; }

    return failCounter;
}


#endif //PERFORM_UNIT_TESTS
