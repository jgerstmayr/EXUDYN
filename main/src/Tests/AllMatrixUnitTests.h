/** ***********************************************************************************************
* @file			AllMatrixUnitTests.h
* @brief		This file contains specific unit tests for all kinds of Exudyn matrix classes
* @details		Details:
                - tests on small sizes of matrices
                - order of tests according to order of member functions in according vector class
                - all non-templated tests are placed here!

* @author		Gerstmayr Johannes
* @date			5 May 2018
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef ALLMATRIXUNITTESTS__H
#define ALLMATRIXUNITTESTS__H

//! Helper class for forcing of const access functions
class ConstCompareAllMatrixUnitTest
{
public:
    bool Compare(const Matrix& m, Real* pointerToRow0) const
    {
        return (m[0] == pointerToRow0);
    }
};

const lest::test matrix_specific_test[] =
{
    //ToString, operator(Index,Index), GetDataPointer, operator<< tested in several CASEs
    CASE("Matrix: default constructor, size 0")
    {
        Matrix x;
        EXPECT(x.NumberOfRows() == 0);
        EXPECT(x.NumberOfColumns() == 0);
        EXPECT(x.GetDataPointer() == nullptr);

        x.SetNumberOfRowsAndColumns(2, 3);
        EXPECT(x.NumberOfRows() == 2);
        EXPECT(x.NumberOfColumns() == 3);
        EXPECT(x.GetDataPointer() != nullptr);
    },
    CASE("Matrix: constructor with rows/columns")
    {
        Matrix x(2,2);
        EXPECT(x.NumberOfRows() == 2);
        EXPECT(x.NumberOfColumns() == 2);
        EXPECT(x.GetDataPointer() != nullptr);
    },
    CASE("Matrix: constructor with rows/columns/value")
    {
        Matrix x(2,2, 3.3);
        EXPECT(x.NumberOfRows() == 2);
        EXPECT(x.NumberOfColumns() == 2);
        EXPECT(ToString(x) == "[3.3 3.3; 3.3 3.3]");
    },
    CASE("Matrix: constructor with initializer list")
    {
        Matrix x(2,2,{1.1,2.2,3.3,4.4});
        EXPECT(x.NumberOfRows() == 2);
        EXPECT(x.NumberOfColumns() == 2);
        EXPECT(ToString(x) == "[1.1 2.2; 3.3 4.4]");
    },

    CASE("Matrix: copy constructor")
    {
        Matrix x1(2,2,{ 1.1,2.2,3.3,4.4 });
        Matrix x2;

        Matrix y1(x1);
        Matrix y2(x2);

        EXPECT(ToString(y1) == "[1.1 2.2; 3.3 4.4]");
        EXPECT(y2.NumberOfRows() == 0);
        EXPECT(y2.NumberOfColumns() == 0);
        EXPECT(y2.GetDataPointer() == nullptr);
    },

    CASE("Matrix: iterators")
    {
        Matrix x(2,2,{ 1.1,2.2,3.3,4.4 });

        EXPECT(x.GetDataPointer() == x.begin());
        EXPECT(&x(1,1) == (x.end() - 1)); //access to value '4.4'

        EXPECT(x.NumberOfRows() == 2);
        EXPECT(x.NumberOfColumns() == 2);
        EXPECT(ToString(x) == "[1.1 2.2; 3.3 4.4]");
    },

    CASE("Matrix: NumberOfRows/Columns, SetNumberOfRowsAndColumns")
    {
        Matrix x(2,2,{ 1.1,2.2,3.3,4.4 }); //x has size 2 x 2
        Matrix y;

        Real* p;
        p = x.GetDataPointer();
        EXPECT(p[2] == 3.3);

        x.SetNumberOfRowsAndColumns(1, 2); //data deleted!, size now 1 x 2
        EXPECT(x.NumberOfRows() == 1);
        EXPECT(x.NumberOfColumns() == 2);

        x.SetNumberOfRowsAndColumns(50, 70); //data deleted!
        x.SetAll(7.7);
        EXPECT(x.NumberOfRows() == 50);
        EXPECT(x.NumberOfColumns() == 70);
        EXPECT(x(49,69) == 7.7);
    },

    CASE("Matrix: CopyFrom")
    {
        Matrix x(2,3,{ 1.1,2.2,3.3,4.4,5.5,6.6 });
        Matrix y;
        Matrix z;

        y.CopyFrom(z, 0, 0, 0, 0); //copy zero part from matrix

        EXPECT(y.GetDataPointer() == nullptr);
        EXPECT(y.NumberOfRows() == 0);
        EXPECT(y.NumberOfColumns() == 0);

        z.CopyFrom(x, 0, 0, 2, 3); //copy zero part from matrix
        EXPECT(ToString(z) == "[1.1 2.2 3.3; 4.4 5.5 6.6]");

        z.CopyFrom(x, 1, 1, 2, 3); //copy zero part from matrix
        EXPECT(ToString(z) == "[5.5 6.6]"); //only (part of) last row copied
        EXPECT(z.NumberOfRows() == 1);
        EXPECT(z.NumberOfColumns() == 2);

        Matrix w;
        w.CopyFrom(x, 1, 0, 2, 3); //copy zero part from matrix
        EXPECT(ToString(w) == "[4.4 5.5 6.6]"); //only last row copied
        EXPECT(w.NumberOfRows() == 1);
        EXPECT(w.NumberOfColumns() == 3);
    },

    CASE("Matrix: SetAll")
    {
        Matrix x(2, 2);
        x.SetAll(2.2);
        EXPECT(ToString(x) == "[2.2 2.2; 2.2 2.2]");

        Matrix y;
        y.SetAll(1.1);
        EXPECT(ToString(y) == "[]");
    },

    CASE("Matrix: SetMatrix")
    {
        Matrix x;
        x.SetMatrix(2, 3, { 1.1,2.2,3.3,4.4,5.5,6.6 });
        EXPECT(ToString(x) == "[1.1 2.2 3.3; 4.4 5.5 6.6]");

        x.SetMatrix(3, 2, { 1.1,2.2,3.3,4.4,5.5,6.6 }); //same initializer_list, other dimensions
        EXPECT(ToString(x) == "[1.1 2.2; 3.3 4.4; 5.5 6.6]");

        x.SetMatrix(0, 0, { }); //same initializer_list
        EXPECT(ToString(x) == "[]");

    },

    CASE("Matrix: SetScalarMatrix")
    {
        Matrix x;
        x.SetScalarMatrix(2, 3.3);
        EXPECT(ToString(x) == "[3.3 0; 0 3.3]");

        x.SetScalarMatrix(0, 3.3);
        EXPECT(ToString(x) == "[]");
    },

    CASE("Matrix: GetItem")
    {
        Matrix x;
        x.SetMatrix(2, 3, { 1.1,2.2,3.3,4.4,5.5,6.6 });

        const Index n = 6;
        Real y[n];
        for (Index i = 0; i < 6; i++)
        {
            y[i] = x.GetItem(i); //const (read) access to Matrix::GetItem
            y[i] *= 2.2;
        }

        EXPECT(y[0] == 1.1*2.2);
        //EXPECT(fabs(y[0] - 2.42) < 1e-14);

        for (Index i = 0; i < 6; i++)
        {
            x.GetItem(i) = y[i]; //referencing (write) access
        }

        x /= 2.2;
        EXPECT(ToString(x) == "[1.1 2.2 3.3; 4.4 5.5 6.6]");
    },

    CASE("Matrix: operator[]")
    {
        Matrix x(2, 3, { 1.1,2.2,3.3,4.4,5.5,6.6 });

        Real* row0 = x[0];
        Real* row1 = x[1];
        EXPECT(row1[0] == 4.4);
        EXPECT(row1[1] == 5.5);
        EXPECT(row1[2] == 6.6);

        ConstCompareAllMatrixUnitTest tester;
        bool test = tester.Compare(x, x[0]); //this should invoke the const version of operator[]
        EXPECT(test == true);

        row0[1] *= 2.;
        row1[1] *= 2.;
        EXPECT(ToString(x) == "[1.1 4.4 3.3; 4.4 11 6.6]");
    },

    CASE("Matrix: operator(Index, Index)")
    {
        Matrix x(2, 3, { 1.1,2.2,3.3,4.4,5.5,6.6 });

        EXPECT(x(0, 0) == 1.1);
        EXPECT(x(0, 1) == 2.2);
        EXPECT(x(0, 2) == 3.3);
        EXPECT(x(1, 0) == 4.4);
        EXPECT(x(1, 1) == 5.5);
        EXPECT(x(1, 2) == 6.6);

        x(1, 1) = 8.8;
        EXPECT(ToString(x) == "[1.1 2.2 3.3; 4.4 8.8 6.6]");
    },

    CASE("Matrix: operator=")
    {
        Matrix x(2,2,{ 1.1,2.2,3.3,4.4 });
        Matrix y;
        y = x;

        EXPECT(y.NumberOfRows() == 2);
        EXPECT(y.NumberOfColumns() == 2);
        EXPECT(ToString(y) == "[1.1 2.2; 3.3 4.4]");
    },
    CASE("Matrix: operator==")
    {
        Matrix x(2,3,{ 1.1,2.2,3.3,4.4,5.5,6.6 });
        Matrix y;
        y = x;

        EXPECT(ToString(y) == "[1.1 2.2 3.3; 4.4 5.5 6.6]");
        EXPECT(x == y);
    },
    CASE("Matrix: operator+=,-=,*=,/=")
    {
        Matrix x(2,3,{ 1.1,2.2,3.3,4.4,5.5,6.6 });
        Matrix y, z;
        y.SetNumberOfRowsAndColumns(2, 3);
        z.SetNumberOfRowsAndColumns(2, 3);

        y.SetAll(0.);
        z.SetAll(0.);

        y += x;
        EXPECT(x == y);

        y -= x;
        z.SetAll(0.);
        EXPECT(y == z);

        y = x;
        y *= 2.5;
        EXPECT(ToString(y) == "[2.75 5.5 8.25; 11 13.75 16.5]");

        y /= 2.5;
        EXPECT(ToString(y) == "[1.1 2.2 3.3; 4.4 5.5 6.6]");
    },

    CASE("Matrix: operator*(Matrix, Matrix)")
    {
        Matrix m1(2, 3, { 1,2.2,3,4,5,6 });
        Matrix m2(3, 4, { 1,2,3,4,5.5,6,7,8,9,0,1,2 });

        Matrix m3 = m1*m2; //contains 2 rows, 4 columns

        EXPECT(ToString(m3) == "[40.1 15.2 21.4 27.6; 85.5 38 53 68]");

        Matrix m1b(3, 1, { 1, 2, 3.3});
        Matrix m2b(1, 1, { 1.1 });

        Matrix m3b = m1b*m2b; //contains 3 rows, 1 columns
        EXPECT(ToString(m3b) == "[1.1; 2.2; 3.63]");
    },

    CASE("Matrix: operator*(Matrix, Real), operator*(Real, Matrix)")
    {
        Matrix m1(2, 3, { 1,2,3.3,4,5,6 });
        Matrix m2(2, 2, { 1,2,3.3,4 });

        Matrix m3 = 2.5*m1;
        Matrix m4 = 2.5*m2;
        Matrix m5 = m1*2.5;
        Matrix m6 = m2*2.5;

        EXPECT(ToString(m3) == "[2.5 5 8.25; 10 12.5 15]");
        EXPECT(ToString(m4) == "[2.5 5; 8.25 10]");
        EXPECT(m3 == m5);
        EXPECT(m4 == m6);
    },

    CASE("Matrix: operator*(Matrix, Vector); MultMatrixVector")
    {
        Matrix m(2, 3, { 1.1,2.2,3.3,4.4,5.5,6.6 });
        Vector v({3,2,2});
        Vector y;
        Vector z(5); //z has a size different from result of m * v
        Vector w(2);
        y = m * v;
        z = m * v;
        w = m * v;

        EXPECT(ToString(y) == "[14.3 37.4]");
        EXPECT(w == y);
        EXPECT(z == y);

        y.SetNumberOfItems(0); //reset Vector to initial state
        z.SetNumberOfItems(5);
        w.SetNumberOfItems(2);

        MultMatrixVector(m, v, y);
        MultMatrixVector(m, v, z);
        MultMatrixVector(m, v, w);

        EXPECT(ToString(y) == "[14.3 37.4]");
        EXPECT(w == y);
        EXPECT(z == y);
    },
    CASE("Matrix: GetTransposed, TransposeYourself, IsSquare")
    {
        Matrix m(2, 2, { 1.1,2.2,3.3,4.4 });
        Matrix m2(3, 3, { 1, 2, 3, 4, 5, 6, 7, 8, 9 });
        Matrix m3;

        m3.TransposeYourself();
        EXPECT(ToString(m3) == "[]");

        Matrix mT = m.GetTransposed();
        EXPECT(ToString(mT) == "[1.1 3.3; 2.2 4.4]");

        m = m2;
        m2.TransposeYourself();
        EXPECT(ToString(m2) == "[1 4 7; 2 5 8; 3 6 9]");
        EXPECT(m2.GetTransposed() == m);

        EXPECT(m2.IsSquare() == true);
        EXPECT(m.IsSquare() == true);

        m.SetMatrix(2, 3, { 1.1,2.2,3.3,4.4,5.5,6.6 });
        EXPECT(m.IsSquare() == false);
    }
};

#endif
