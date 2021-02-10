/** ***********************************************************************************************
* @file			AllVectorUnitTests.h
* @brief		This file contains specific unit tests for all kinds of Exudyn vector classes
* @details		Details:
                - tests on small sizes of vectors
                - order of tests according to order of member functions in according vector class
                - all non-templated tests are placed here!

* @author		Gerstmayr Johannes
* @date			2 May 2018
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef ALLVECTORUNITTESTS__H
#define ALLVECTORUNITTESTS__H

const lest::test vector_specific_test[] =
{

    CASE("Vector: default constructor, size 0")
    {
        Vector x;
        EXPECT(x.NumberOfItems() == 0);
        EXPECT(x.GetDataPointer() == nullptr);
    },

    CASE("Vector(0), constructor with size 0, no initialization")
    {
        Vector x(0);
        EXPECT(x.NumberOfItems() == 0);
    },

    CASE("Vector(1)")
    {
        Vector x(1);
        EXPECT(x.NumberOfItems() == 1);
    },

    CASE("Vector(10)")
    {
        Vector x(10);
        EXPECT(x.NumberOfItems() == 10);
    },

    CASE("Vector(0, 0.), constructor with size 0")
    {
        Vector x(0, 0.);
        EXPECT(x.NumberOfItems() == 0);
        EXPECT(x.GetDataPointer() == nullptr);
    },

    CASE("Vector(1, 1.7), constructor with size 1, initialize with 1.7")
    {
        Vector x(1, 1.7);
        EXPECT(x.NumberOfItems() == 1);
        EXPECT(x[0] == 1.7);
    },

	CASE("Vector: copy constructor, size 0")
	{
		Vector x;
		Vector y = x;
		EXPECT(y.NumberOfItems() == 0);
		EXPECT(y.GetDataPointer() == nullptr);
	},

	CASE("Vector: copy constructor x(y)")
	{
		Vector x({3,5,9,1});
		Vector y(x);
		EXPECT(y.NumberOfItems() == 4);
		EXPECT(ToString(y) == "[3 5 9 1]");
	},

	CASE("Vector({1}), initializer list with size 1")
    {
        Vector x({ 1. });
        EXPECT(x.NumberOfItems() == 1);
        EXPECT(x[0] == 1.);
    },

    CASE("Vector.SetNumberOfItems()")
    {
        Vector x;
        Vector y(5);
        EXPECT(x.NumberOfItems() == 0);
        EXPECT(y.NumberOfItems() == 5);

        x.SetNumberOfItems(2);
        y.SetNumberOfItems(0); //vector deleted

        EXPECT(x.NumberOfItems() == 2);
        EXPECT(y.NumberOfItems() == 0);

        x[0] = 1.1;
        x[1] = 2.2;
        y = x; //new vector

        y.SetNumberOfItems(3); //new vector y
        EXPECT(y.NumberOfItems() == 3);

        y[2] = 3.3;
        EXPECT(y[2] == 3.3);
    },
    CASE("Vector, SetVector")
    {
        Vector y;
        y.SetVector({ 3,5,9,1 });
        EXPECT(y.NumberOfItems() == 4);
        EXPECT(ToString(y) == "[3 5 9 1]");

        Vector x(3);
        x.SetVector({ 3,5,9,1 });
        EXPECT(x.NumberOfItems() == 4);
        EXPECT(ToString(x) == "[3 5 9 1]");

        Vector z;
        z.SetVector({ 3,5,9,1 });
        EXPECT(z.NumberOfItems() == 4);
        EXPECT(ToString(z) == "[3 5 9 1]");

        Vector w;
        w.SetVector({ });
        EXPECT(w.NumberOfItems() == 0);

        w.SetVector({7});
        EXPECT(w.NumberOfItems() == 1);
        EXPECT(w[0] == 7);
    },
    CASE("Vector::Append")
    {
        Vector v1({ 3, 4 });
        Vector v2({ 4, 5, 7 });
        Vector v3({ 8,9 });
        v1 = v1.Append(v3);
        EXPECT(ToString(v1) == "[3 4 8 9]");

        Vector v4({ 10 });
        v1 = v4.Append(v2);
        EXPECT(ToString(v1) == "[10 4 5 7]");

        Vector v5(0);
        v2 = v1.Append(v5);
        EXPECT(ToString(v2) == "[10 4 5 7]");

        v3 = v5.Append(v1);
        EXPECT(ToString(v3) == "[10 4 5 7]");
    },
};

const lest::test constSizeVector_specific_test[] =
{

    CASE("ConstSizeVector: default constructor, size 1")
    {
        ConstSizeVector<1> x; //DIFFERENT FROM default constructor of Vector !!!
        EXPECT(x.NumberOfItems() == 1);
        EXPECT(x.GetDataPointer() == &x[0]);
    },

    CASE("ConstSizeVector constructors, SetNumberOfItems, (Max)NumberOfItems")
    {
        ConstSizeVector<4> x(4);
        EXPECT(x.NumberOfItems() == 4);
        EXPECT(x.MaxNumberOfItems() == 4);

        ConstSizeVector<4> y(3);
        EXPECT(y.NumberOfItems() == 3);
        EXPECT(y.MaxNumberOfItems() == 4);

        y = ConstSizeVector<4>({ 1.1, 2, 3, 4 });
        x[3] = 10;
        x.SetNumberOfItems(3);
        y.SetNumberOfItems(3);
        x = y; //x[3] should be left unchanged
        EXPECT(x.NumberOfItems() == 3);
        EXPECT(y.NumberOfItems() == 3);
        x.SetNumberOfItems(4);
        EXPECT(ToString(x) == "[1.1 2 3 10]");
    },

    CASE("ConstSizeVector(const Vector&, Index)")
    {
        Vector x({3, 1, 4, 5.5});
        ConstSizeVector<2> y(x, 0);
        ConstSizeVector<2> z(x, 2);
        EXPECT(ToString(y) == "[3 1]");
        EXPECT(ToString(z) == "[4 5.5]");
    },

    CASE("ConstSizeVector(Index, Real), numberOfItems during operator= and copy constructor")
    {
        ConstSizeVector<4> x(4, 13.); //numberOfItems=4
        ConstSizeVector<4> y({1, 2, 5}); //numberOfItems=3
        EXPECT(y.NumberOfItems() == 3);

        x = y; //x:numberOfItems becomes 3
        EXPECT(x.NumberOfItems() == 3);
        EXPECT(ToString(x) == "[1 2 5]");

        x.SetNumberOfItems(4);
        EXPECT(ToString(x) == "[1 2 5 13]");

        ConstSizeVector<4> z(y); //numberOfItems=4
        EXPECT(z.NumberOfItems() == 3);
        EXPECT(ToString(z) == "[1 2 5]");

        ConstSizeVector<2> w(0, 7.); //numberOfItems=0
        EXPECT(w.NumberOfItems() == 0);
    },

    CASE("ConstSizeVector.SetNumberOfItems()")
    {
        ConstSizeVector<2> x;
        ConstSizeVector<2> y;
        EXPECT(x.NumberOfItems() == 2);
        EXPECT(y.NumberOfItems() == 2);

        x.SetNumberOfItems(2); //OK, if NumberOfItems <= dataSize 
        y.SetNumberOfItems(2);

        EXPECT(x.NumberOfItems() == 2);
        EXPECT(y.NumberOfItems() == 2);

        x.SetAll(5.5);
        y[0] = 1.1;
        y[1] = 2.2;
        x += y;
        x.SetNumberOfItems(2);

        EXPECT(x.NumberOfItems() == 2);
        EXPECT(x[0] == 6.6);
        EXPECT(x[1] == 7.7);
    },

    CASE("ConstSizeVector operators for numberOfItems < dataSize")
    {
        ConstSizeVector<4> x({ 3, 1, 5.5, 6 });
        ConstSizeVector<4> y({ 1, 2, 3, 4 });

        ConstSizeVector<4> z(4, 4.4);
        x.SetNumberOfItems(3);
        y.SetNumberOfItems(3);

        z = x + y; //z.numberOfItems==3 now
        EXPECT(z.NumberOfItems() == 3);

        z.SetNumberOfItems(4);
        EXPECT(ToString(x) == "[3 1 5.5]");
        EXPECT(ToString(y) == "[1 2 3]");
        EXPECT(ToString(z) == "[4 3 8.5 4.4]"); //component [3] not touched by operator

        z = x - y; //z.numberOfItems==3 now
        z.SetNumberOfItems(4);
        EXPECT(ToString(z) == "[2 -1 2.5 4.4]"); //component [3] not touched by operator

        z.SetNumberOfItems(3);
        z += y;
        z.SetNumberOfItems(4);
        EXPECT(ToString(z) == "[3 1 5.5 4.4]");

        z.SetNumberOfItems(3);
        z -= y;
        z.SetNumberOfItems(4);
        EXPECT(ToString(z) == "[2 -1 2.5 4.4]");

        z.SetNumberOfItems(3);
        z *= 2.2;
        z.SetNumberOfItems(4);
        EXPECT(ToString(z) == "[4.4 -2.2 5.5 4.4]");

        z.SetNumberOfItems(3);
        z /= 2.2;
        z.SetNumberOfItems(4);
        EXPECT(ToString(z) == "[2 -1 2.5 4.4]");

        z.SetNumberOfItems(3);
        Real s = z*x;
        EXPECT(s == (2 * 3 - 1 * 1 + 2.5 * 5.5));

        Real s2 = x*z;
        EXPECT(s2 == (2 * 3 - 1 * 1 + 2.5 * 5.5));
    },

};


const lest::test SlimVector_specific_test[] =
{

    CASE("SlimVector<1>:")
    {
        SlimVector<1> v;
        EXPECT(v.NumberOfItems() == 1);
    },
    CASE("SlimVector<1>:")
    {
        SlimVector<1> v({ 1.1 }); //constructor SlimVector<1> v(1.1) called!
        EXPECT(v.NumberOfItems() == 1);
        EXPECT(v[0] == 1.1);
        EXPECT(v.GetDataPointer() == &v[0]);
    },
    CASE("SlimVector<2>:")
    {
        SlimVector<2> v({ 1.1,2.2 });
        EXPECT(v.NumberOfItems() == 2);
        EXPECT(v[0] == 1.1);
        EXPECT(v[1] == 2.2);
        EXPECT((int)(v.end() - v.begin()) == 2);
        EXPECT(v.begin() == &v[0]);
    },
    CASE("SlimVector<3>:")
    {
        SlimVector<3> v({ 1.1,2.2,3.3 });
        EXPECT(v.NumberOfItems() == 3);
        EXPECT(v[0] == 1.1);
        EXPECT(v[1] == 2.2);
        EXPECT(v[2] == 3.3);
        EXPECT((int)(v.end() - v.begin()) == 3);
        EXPECT(v.begin() == &v[0]);
    },

    CASE("SlimVector<1,2,3>:.X(),.Y(),.Z()")
    {
        SlimVector<1> v1({ 1.1 });
        SlimVector<2> v2({ 2.2, 3.3 });
        SlimVector<3> v3({ 4.4, 5.5, 6.6 });
        //SlimVector<0> v4;

        EXPECT(v1[0] == v1.X());

        EXPECT(v2[0] == v2.X());
        EXPECT(v2[1] == v2.Y());

        EXPECT(v3[0] == v3.X());
        EXPECT(v3[1] == v3.Y());
        EXPECT(v3[2] == v3.Z());
    },
    CASE("SlimVector<2>(const Vector&):")
    {
        Vector v({ 1.1, 2.2, 3.3 });
        SlimVector<2> sv(v);

        EXPECT(sv.NumberOfItems() == 2);
        EXPECT(sv[0] == 1.1);
        EXPECT(sv[1] == 2.2);

        SlimVector<1> sv2(v,2);
        EXPECT(sv2.NumberOfItems() == 1);
        EXPECT(sv2[0] == 3.3);

        SlimVector<3> sv3(v);
        EXPECT(sv3.NumberOfItems() == 3);
        EXPECT(sv3[0] == 1.1);
        EXPECT(sv3[1] == 2.2);
        EXPECT(sv3[2] == 3.3);

    },
};

const lest::test linkedDataVector_specific_test[] =
{

    CASE("LinkedDataVector: default constructor")
    {
        Vector v({ 3, 4, 5.5 });
        LinkedDataVector lv;
        LinkedDataVector lv2;

        EXPECT(lv.NumberOfItems() == 0);
        EXPECT(lv.GetDataPointer() == nullptr);

        lv.LinkDataTo(v);
        EXPECT(lv.NumberOfItems() == 3);
        EXPECT(lv.GetDataPointer() == v.GetDataPointer());
        EXPECT(ToString(lv) == "[3 4 5.5]");

        lv2.LinkDataTo(lv); //double linking is also possible, just passing of pointers to original vector v
        EXPECT(lv2.NumberOfItems() == 3);
        EXPECT(lv2.GetDataPointer() == v.GetDataPointer());
        EXPECT(ToString(lv2) == "[3 4 5.5]");
    },

    CASE("LinkedDataVector: link data constructor")
    {
        Vector v({ 3, 4, 5.5 });
        LinkedDataVector lv(v);

        EXPECT(lv.NumberOfItems() == 3);
        EXPECT(lv.GetDataPointer() == v.GetDataPointer());
        EXPECT(ToString(lv) == "[3 4 5.5]");

        Vector x = lv + v;
        EXPECT(ToString(x) == "[6 8 11]");
    },

    CASE("LinkedDataVector: link data constructor(SlimVector<3>)")
    {
        SlimVector<3> v({ 3, 4, 5.5 });
        LinkedDataVector lv(v);
        LinkedDataVector lv2(v);

        EXPECT(lv.NumberOfItems() == 3);
        EXPECT(lv.GetDataPointer() == v.GetDataPointer());
        EXPECT(ToString(lv) == "[3 4 5.5]");

        Vector x = lv + lv2;
        EXPECT(ToString(x) == "[6 8 11]");
    },

    CASE("LinkedDataVector: LinkDataTo(Vector)), operator=, operator==")
    {
        Vector v1({ 3, 4, 5.5 });
        Vector v2({ 4, 3, 1.5 });
        Vector v3({ 1, 2.2, 3 });
        LinkedDataVector lv1, lv2;

        lv1.LinkDataTo(v1); lv2.LinkDataTo(v2);

        EXPECT(lv1.NumberOfItems() == 3);
        EXPECT(lv1.GetDataPointer() == v1.GetDataPointer());
        EXPECT(lv1 == v1);
        EXPECT(ToString(lv1) == "[3 4 5.5]");

        Vector x = lv1 - lv2;
        EXPECT(ToString(x) == "[-1 1 4]");

        v1 = v2;
        EXPECT(lv1 == lv2);

        lv1 = v3; //assign components of v1 with v3, but DO NOT LINK TO v3
        EXPECT(lv1 == v3);
        EXPECT(lv1 == v1);
        EXPECT(lv1.GetDataPointer() == v1.GetDataPointer());
        EXPECT_NOT(lv1.GetDataPointer() == v3.GetDataPointer()); //impossible, since v3 has own data

    },

    CASE("LinkedDataVector: LinkDataTo(Vector,Index,Index)), operator[]")
    {
        Vector v1({ 3, 4, 5.5 });
        Vector v2({ 4, 3, 1.5 });
        LinkedDataVector lv1, lv2;

        lv1.LinkDataTo(v1,0,2); lv2.LinkDataTo(v2,1,2);

        EXPECT(lv1.NumberOfItems() == 2);
        EXPECT(lv1.GetDataPointer() == &v1[0]);
        EXPECT(lv2.GetDataPointer() == &v2[1]);
        EXPECT(ToString(lv1) == "[3 4]");
        EXPECT(ToString(lv2) == "[3 1.5]");

        lv1[0] = 7.7;
        lv1[1] = 8.8;
        EXPECT(ToString(v1) == "[7.7 8.8 5.5]");

        v1[0] = 9.9;
        EXPECT(lv1[0] == 9.9);
        EXPECT(lv1[1] == 8.8);
    },

    CASE("LinkedDataVector: LinkDataTo(Vector,Index,Index)), operator +=, -=, *=, /=")
    {
        Vector v1({ 3, 4, 5.5 });
        Vector v2({ 4, 3, 1.5 });
        LinkedDataVector lv1, lv2;

        lv1.LinkDataTo(v1,0,2); lv2.LinkDataTo(v2,1,2);

        lv1 += lv2;
        EXPECT(ToString(lv1) == "[6 5.5]");
        EXPECT(ToString(v1) == "[6 5.5 5.5]");

        lv1 -= lv2;
        EXPECT(ToString(lv1) == "[3 4]");

        lv2 *= 2.5;
        EXPECT(ToString(lv2) == "[7.5 3.75]");
        EXPECT(ToString(v2) ==  "[4 7.5 3.75]");

        lv2 /= 2.5;
        EXPECT(ToString(lv2) == "[3 1.5]");
    },

    CASE("LinkedDataVector: ")
    {
        SETUP("extended math")
        {
            Vector x1({ 3, 4 });
            Vector x2({ 4, 5});
            LinkedDataVector v1(x1);
            LinkedDataVector v2(x2);
            SECTION("GetL2NormSquared")
            {
                EXPECT(v1.GetL2NormSquared() == 25);
            }
            SECTION("GetL2Norm")
            {
                EXPECT(v1.GetL2Norm() == sqrt(25));
            }
            SECTION("Normalize")
            {
                v1.Normalize();
                EXPECT(v1[0] == 3. / sqrt(25.));
                EXPECT(v1[1] == 4. / sqrt(25.));
            }
            SECTION("MultAdd and Sum, SetVector")
            {
                v1.SetVector({ 1, 2});
                v2.MultAdd(2.5, v1);
                EXPECT(ToString(v2) == "[6.5 10]");

                EXPECT(v1.Sum() == 3);
                EXPECT(v2.Sum() == 16.5);
            }
        }
    },
};


const lest::test resizableVector_specific_test[] =
{

    CASE("ResizableVector: constructor and SetNumberOfItems")
    {
        ResizableVector v1({ 3, 4, 5.5 });
        ResizableVector v2;
        ResizableVector v3(4);
        ResizableVector v4(2, 3.3);

        EXPECT(v1.NumberOfItems() == 3);
        EXPECT(v2.NumberOfItems() == 0);
        EXPECT(v3.NumberOfItems() == 4);
        EXPECT(v4[0] == 3.3);
        EXPECT(v4[1] == 3.3);

        v1.SetNumberOfItems(2);
        v2.SetNumberOfItems(2);
        EXPECT(v1.NumberOfItems() == 2);
        EXPECT(v2.NumberOfItems() == 2);
        EXPECT(ToString(v1) == "[3 4]");

        v2 = v4;
        v2.SetNumberOfItems(1);
        EXPECT(v2.NumberOfItems() == 1);
        EXPECT(v2[0] == 3.3);

        v2.SetNumberOfItems(100);
        EXPECT(v2.NumberOfItems() == 100);
    },
};

#endif
