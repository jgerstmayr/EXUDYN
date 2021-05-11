/** ***********************************************************************************************
* @file			TemplatedVectorArrayUnitTests.h
* @brief		This file contains templated tests for all kinds of Exudyn array and vector classes
* @details		Details:
                - tests on small sizes of arrays/vectors
                - templates used to reduce number of hand-coded-tests
                - order of tests according to order of member functions in according vector/array class

* @author		Gerstmayr Johannes
* @date			2018-05-02 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef TEMPLATEDVECTORARRAYUNITTESTS__H
#define TEMPLATEDVECTORARRAYUNITTESTS__H


//works with lest_cpp03.hpp
//should also work with #define lest_FEATURE_AUTO_REGISTER
//#define CASE( name ) lest_CASE( auto_specification, name )
//
//static lest::tests auto_specification;

static const int vector_testsize = 2;

//! @brief templated test for Vector, ConstSizeVector, SlimVector, ResizableVector, ResizableArray<Real>, SlimArray<Real>
//! LinkedDataVector needs to be tested separately!
//! special functionality of ***Vector tested separately!
template< typename T >
void case_template_vectorarray_test(lest::env & lest_env)
{
    SETUP("basic test")
    {
        T v1({ 0.,0. });
        T v2({ 1.1,2.2 });

        SECTION("NumberOfItems")
        {
            EXPECT(v1.NumberOfItems() == vector_testsize);
            EXPECT(v2.NumberOfItems() == vector_testsize);
        }

        SECTION("initializer_list")
        {
            EXPECT(v1[0] == 0.);
            EXPECT(v1[1] == 0.);
            EXPECT(v2[0] == 1.1);
            EXPECT(v2[1] == 2.2);
        }

        SECTION("SetAll")
        {
            v1.SetAll(1.7);

            EXPECT(v1[0] == 1.7);
            EXPECT(v1[1] == 1.7);
        }

        SECTION("begin/end")
        {
            EXPECT((int)(v1.end() - v1.begin()) == 2);
            EXPECT(v1.begin() == &v1[0]);
        }

        SECTION("operator=")
        {
            T v3;
            v3 = v2;
            EXPECT(v3[0] == 1.1);
            EXPECT(v3[1] == 2.2);
            EXPECT((v3 == v2) == 1);
        }

        SECTION("operator= (check copy)")
        {
            T v3;
            v3 = v2;
            v2[0] = 7; //check, if v3 is really copied or just a reference ...
            EXPECT(v3[0] == 1.1);
            EXPECT(v2[0] == 7);
        }

        SECTION("operator==")
        {
            EXPECT_NOT((v1 == v2) == 1);
            v2 = v1;
            EXPECT((v1 == v2) == 1);
        }

        SECTION("ostream& operator<<")
        {
            std::ostringstream strStream;
            strStream << v2;
            EXPECT(strStream.str() == string("[1.1 2.2]"));
            EXPECT_NOT(strStream.str() == string("[1.1,2.2]"));
        }

    }
}

//! @brief mathematical test for Vector, ConstSizeVector, ResizableVector; NOT for SlimVector
//! special functionality of ***Vector tested separately!
template< typename T >
void case_template_purevector_test(lest::env & lest_env)
{
    SETUP("extended functions test")
    {
        T v1({ 3., 4. });
        T v2({ 4., 5., 7. });
        SECTION("GetL2NormSquared")
        {
            EXPECT(v1.GetL2NormSquared() == 25);
            EXPECT(v2.GetL2NormSquared() == (Square(4) + Square(5) + Square(7)));
        }
        SECTION("GetL2Norm")
        {
            EXPECT(v1.GetL2Norm() == sqrt(25));
            EXPECT(v2.GetL2Norm() == sqrt(Square(4) + Square(5) + Square(7)));
        }
        SECTION("Normalize")
        {
            v1.Normalize();
            EXPECT(v1[0] == 3. / sqrt(25.));
            EXPECT(v1[1] == 4. / sqrt(25.));
        }
        SECTION("CopyFrom")
        {
            T v3({ 6., 3., 4.4, 5.5 });
            T v4(3);

            v1.CopyFrom(v3, 0, 0, 2);
            EXPECT(v1[0] == 6);
            EXPECT(v1[1] == 3);

            v4.CopyFrom(v3, 1, 0, 3);
            EXPECT(v4[0] == 3);
            EXPECT(v4[1] == 4.4);
            EXPECT(v4[2] == 5.5);

            v2.CopyFrom(v3, 2, 1, 2);
            EXPECT(v2[0] == 4);
            EXPECT(v2[1] == 4.4);
            EXPECT(v2[2] == 5.5);
        }
        SECTION("MultAdd  and Sum")
        {
            v1.SetVector({ 1, 2, 4.4 });
            v2.MultAdd(2.5, v1);
            EXPECT(ToString(v2) == "[6.5 10 18]");

            EXPECT(v1.Sum() == 7.4);
            EXPECT(v2.Sum() == 34.5);
        }
    }
};

//! @brief mathematical test for Vector, ConstSizeVector, SlimVector, ResizableVector
//! special functionality of ***Vector tested separately!
template< typename T >
void case_template_vector_test(lest::env & lest_env)
{
    SETUP("Math test")
    {
        T v1({ 1.1, 2.2 });
        T v2({ 4.4, 5.5 });
        SECTION("operator+=")
        {
            v1 += v2;
            EXPECT(v1[0] == (1.1 + 4.4));
            EXPECT(v1[1] == (2.2 + 5.5));
        }
        SECTION("operator-=")
        {
            v1 -= v2;
            EXPECT(v1[0] == (1.1 - 4.4));
            EXPECT(v1[1] == (2.2 - 5.5));
        }
        SECTION("operator*=")
        {
            v1 *= 4.4;
            EXPECT(v1[0] == (1.1 * 4.4));
            EXPECT(v1[1] == (2.2 * 4.4));
        }
        SECTION("operator/=")
        {
            v1 /= 4.4;
            EXPECT(v1[0] == (1.1 / 4.4));
            EXPECT(v1[1] == (2.2 / 4.4));
        }
        SECTION("operator+")
        {
            T v3;

            v3 = v1 + v2;
            EXPECT(v3[0] == (v1[0] + v2[0]));
            EXPECT(v3[1] == (v1[1] + v2[1]));
            v3 = v2;
            v2 = v1 + v2;
            EXPECT(v2[0] == (1.1 + 4.4));
            EXPECT(v2[1] == (2.2 + 5.5));

            v1 = v1 + v2;
            EXPECT(v1[0] == (1.1 + (1.1 + 4.4))); //bracketing necessary due to rounding errors!
            EXPECT(v1[1] == (2.2 + (2.2 + 5.5)));

        }
        SECTION("operator*(Vector, scalar)")
        {
            T v3;
            v3 = v1 * 4.4;
            EXPECT(v3[0] == (4.4 * v1[0]));
            EXPECT(v3[1] == (4.4 * v1[1]));
        }
        SECTION("operator*(scalar, Vector)")
        {
            T v3;
            v3 = 4.4 * v1;
            EXPECT(v3[0] == (4.4 * v1[0]));
            EXPECT(v3[1] == (4.4 * v1[1]));
        }
        SECTION("operator*(Vector, Vector): scalar product")
        {
            Real r = v1 * v2;
            EXPECT(r == (1.1*4.4 + 2.2*5.5));
        }

    }
};

//! @brief mathematical test for Vector, ConstSizeVector, SlimVector, ResizableVector
//! special functionality of ***Vector tested separately!
template< typename T >
void case_template_array_basic_test(lest::env & lest_env)
{
    SETUP("Basic Functions test")
    {
        T a1({ 1.1, 2.2 });
        //T a2({ 4.4, 5.5 });

        SECTION("NumberOfItems")
        {
            EXPECT(a1.NumberOfItems() == 2);
            EXPECT(a1.MaxNumberOfItems() == 2);
        }
        SECTION("GetDataPointer")
        {
            EXPECT(a1.GetDataPointer() == &a1[0]);
        }
        SECTION("SetAll")
        {
            a1.SetAll(1.7);
            EXPECT(a1[0] == 1.7);
            EXPECT(a1[1] == 1.7);
            EXPECT(a1.NumberOfItems() == 2);
        }
        SECTION("Last")
        {
            EXPECT(a1.Last() == 2.2);
        }
    }
};

template< typename T >
void case_template_array_extended_test(lest::env & lest_env)
{
    SETUP("Extended Functions test")
    {
        T a1({ 1.1, 2.2 });
        SECTION("GetItem(...) const")
        {
            EXPECT(a1.GetItem(0) == 1.1);
            EXPECT(a1.GetItem(1) == 2.2);
        }
        SECTION("GetItem(...)")
        {
            a1.GetItem(0) = 3.3;
            a1.GetItem(1) = 4.4;
            EXPECT(a1.GetItem(0) == 3.3);
            EXPECT(a1.GetItem(1) == 4.4);
            EXPECT(a1.NumberOfItems() == 2);
        }
        SECTION("GetItemCyclic(...) const")
        {
            EXPECT(a1.GetItemCyclic(0) == 1.1);
            EXPECT(a1.GetItemCyclic(1) == 2.2);
            EXPECT(a1.GetItemCyclic(2) == 1.1);
            EXPECT(a1.GetItemCyclic(3) == 2.2);
            EXPECT(a1.GetItemCyclic(1000) == 1.1);
            EXPECT(a1.NumberOfItems() == 2);
        }
        SECTION("GetItemCyclic(...)")
        {
            a1.GetItemCyclic(0) = 3.3;
            a1.GetItemCyclic(1) = 4.4;
            EXPECT(a1.GetItemCyclic(0) == 3.3);
            EXPECT(a1.GetItemCyclic(1) == 4.4);
            a1.GetItemCyclic(2) = 5.5;
            EXPECT(a1.GetItemCyclic(0) == 5.5);
            EXPECT(a1.GetItemCyclic(1) == 4.4);
            EXPECT(a1.NumberOfItems() == 2);
            EXPECT(a1.MaxNumberOfItems() == 2);
        }
        SECTION("GetIndexOfItem (Find)")
        {
            Index a = a1.GetIndexOfItem(1.1);
            Index b = a1.GetIndexOfItem(2.2);
            Index c = a1.GetIndexOfItem(19.1);
            EXPECT(a == 0);
            EXPECT(b == 1);
            EXPECT(c == EXUstd::InvalidIndex);
        };
        //SECTION("Sort")
        //{
        //    a1[0] = 100;
        //    a1.Sort();
        //    EXPECT(a1[0] == 2.2);
        //    EXPECT(a1[1] == 100);
        //}
    }
};

const lest::test array_templated_tests[] =
{
    CASE("Templated 'SlimArray<Real,2>' test")
    {
        case_template_array_basic_test<SlimArray<Real,vector_testsize>>(lest_env);
    },
    CASE("Templated 'ResizableArray<Real>' test")
    {
        case_template_array_basic_test<ResizableArray<Real>>(lest_env);
    },
    CASE("Templated 'SlimArray<Real,2>' test")
    {
        case_template_array_extended_test<SlimArray<Real,vector_testsize>>(lest_env);
    },
    CASE("Templated 'ResizableArray<Real>' test")
    {
        case_template_array_extended_test<ResizableArray<Real>>(lest_env);
    },
};

const lest::test vector_templated_tests[] =
{
    CASE("Templated 'Vector' test")
    {
        case_template_vector_test<Vector>(lest_env);
    },

    CASE("Templated 'ResizableVector' test")
    {
        case_template_vector_test<ResizableVector>(lest_env);
    },

    CASE("Templated 'ConstSizeVector<2>' test")
    {
        case_template_vector_test<ConstSizeVector<vector_testsize> >(lest_env);
    },

    CASE("Templated 'SlimVector<2>' test")
    {
        case_template_vector_test<SlimVector<vector_testsize> >(lest_env);
    },

    CASE("Templated 'Vector' test")
    {
        case_template_purevector_test<Vector>(lest_env);
    },

    CASE("Templated 'ConstSizeVector<4>' test") //maximum needed vector size for this templated test is 4
    {
        case_template_purevector_test<ConstSizeVector<4>>(lest_env);
    },

};

const lest::test vectorarray_templated_tests[] =
{
    CASE("Templated 'Vector' test")
    {
        case_template_vectorarray_test<Vector>(lest_env);
    },

    CASE("Templated 'ResizableVector' test")
    {
        case_template_vectorarray_test<ResizableVector>(lest_env);
    },

    CASE("Templated 'ConstSizeVector<2>' test")
    {
        case_template_vectorarray_test<ConstSizeVector<vector_testsize> >(lest_env);
    },

    CASE("Templated 'SlimVector<2>' test")
    {
        case_template_vectorarray_test<SlimVector<vector_testsize> >(lest_env);
    },

    CASE("Templated 'ResizableArray<Real>' test")
    {
        case_template_vectorarray_test<ResizableArray<Real> >(lest_env);
    },

    CASE("Templated 'SlimArray<Real,2>' test")
    {
        case_template_vectorarray_test<SlimArray<Real,2> >(lest_env);
    },
};

#endif
