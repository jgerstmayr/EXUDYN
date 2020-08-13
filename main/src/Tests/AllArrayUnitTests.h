/** ***********************************************************************************************
* @file			AllArrayUnitTests.h
* @brief		This file contains specific unit tests for all kinds of Exudyn array classes
* @details		Details:
                - tests on small sizes of arrays
                - order of tests according to order of member functions in according array class
                - all non-templated tests are placed here!

* @author		Gerstmayr Johannes
* @date			2018-05-02 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
************************************************************************************************ */
#ifndef ALLARRAYUNITTESTS__H
#define ALLARRAYUNITTESTS__H

#pragma once

const lest::test resizableArray_specific_test[] =
{

    CASE("ResizableArray<int>: default constructor, size 0")
    {
        ResizableArray<int> x;
        EXPECT(x.NumberOfItems() == 0);
        EXPECT(x.GetDataPointer() == nullptr);
    },
    CASE("ResizableArray<int>: initializer list, Last()")
    {
        ResizableArray<int> x({3,5,9});
        EXPECT(x.NumberOfItems() == 3);
        EXPECT(x[0] == 3);
        EXPECT(x[1] == 5);
        EXPECT(x[2] == 9);
        EXPECT(x.Last() == 9);
    },
    CASE("ResizableArray<Index2>:initializer list")
    {
        ResizableArray<Index2> x(2);
        x[0] = Index2({ 3,4 });
        x[1] = Index2({ 6,7 });

        EXPECT(x.NumberOfItems() == 2);
        EXPECT(x[0][0] == 3);
        EXPECT(x[0][1] == 4);
        EXPECT(x[1][0] == 6);
        EXPECT(x[1][1] == 7);
    },
    CASE("ResizableArray<Vector3D>:initializer list + copy constructor + operator=")
    {
        ResizableArray<Vector3D> x(2);
        x[0] = Vector3D({ 3.3,4.4,5.5 });
        x[1] = Vector3D({ 6.6,7.7,9.9 });
        x[2] = Vector3D({ 8.1,8.2,8.3 });

        ResizableArray<Vector3D> y = x;
        ResizableArray<Vector3D> z;
        z = x;

        EXPECT(x.NumberOfItems() == 3);
        EXPECT(x.MaxNumberOfItems() == 4);
        EXPECT(x[0][0] == 3.3);
        EXPECT(x[0][1] == 4.4);
        EXPECT(x[0][2] == 5.5);
        EXPECT(x[1].GetL2Norm() == sqrt(6.6*6.6 + 7.7*7.7 + 9.9*9.9));
        EXPECT(x[2][1] == 8.2);
        EXPECT((x == y) == 1);
        EXPECT((x == z) == 1);
    },
    CASE("ResizableArray: SetNumberOfItems")
    {
        ResizableArray<int> x({ 3,5,9 });
        x.SetNumberOfItems(4); //enlarge array & copy data
        EXPECT(x.NumberOfItems() == 4);
        EXPECT(x.MaxNumberOfItems() == 6);
        x[3] = 7; //to be defined
        EXPECT(x[0] == 3);
        EXPECT(x[1] == 5);
        EXPECT(x[2] == 9);
        EXPECT(x[3] == 7);
        EXPECT(x.MaxNumberOfItems() == 6);

        x.SetNumberOfItems(2);
        EXPECT(x.NumberOfItems() == 2);
        EXPECT(x.MaxNumberOfItems() == 6);
        EXPECT(x[0] == 3);
        EXPECT(x[1] == 5);

        x.SetNumberOfItems(0);
        EXPECT(x.NumberOfItems() == 0);
        EXPECT(x.MaxNumberOfItems() == 6);
    },
    CASE("ResizableArray: EnlargeMaxNumberOfItemsTo")
    {
        ResizableArray<int> x({ 3,5 });
        x.EnlargeMaxNumberOfItemsTo(5);
        EXPECT(x.NumberOfItems() == 2);
        EXPECT(x.MaxNumberOfItems() == 5);

        x.EnlargeMaxNumberOfItemsTo(6);
        EXPECT(x.NumberOfItems() == 2);
        EXPECT(x.MaxNumberOfItems() == 10);
    },
    CASE("ResizableArray: SetMaxNumberOfItems")
    {
        ResizableArray<int> x({ 3,5,9 });
        x.SetMaxNumberOfItems(4); //enlarge array & copy data
        EXPECT(x.NumberOfItems() == 3);
        EXPECT(x.MaxNumberOfItems() == 4);
        EXPECT(x[0] == 3);
        EXPECT(x[1] == 5);
        EXPECT(x[2] == 9);
        x.SetMaxNumberOfItems(2); //shrink array
        EXPECT(x.NumberOfItems() == 2);
        EXPECT(x.MaxNumberOfItems() == 2);
        EXPECT(x[0] == 3);
        EXPECT(x[1] == 5);

        x.SetNumberOfItems(7);
        EXPECT(x.NumberOfItems() == 7);
        EXPECT(x.MaxNumberOfItems() == 7);

        x.SetNumberOfItems(8);
        EXPECT(x.NumberOfItems() == 8);
        EXPECT(x.MaxNumberOfItems() == 14);
    },
    CASE("ResizableArray: Flush")
    {
        ResizableArray<int> x({ 3,5 });
        ResizableArray<int> y;
        x.Flush();
        EXPECT(x.NumberOfItems() == 0);
        EXPECT(x.MaxNumberOfItems() == 0);
        EXPECT(x.GetDataPointer() == nullptr);
        y.Flush();
        EXPECT(y.NumberOfItems() == 0);
        EXPECT(y.MaxNumberOfItems() == 0);
        EXPECT(y.GetDataPointer() == nullptr);

        x[0] = 13;
        EXPECT(x.NumberOfItems() == 1);
        EXPECT(x.MaxNumberOfItems() == 1);
        EXPECT(x[0] == 13);
    },
    CASE("ResizableArray: Append")
    {
        ResizableArray<Real> x({ 3.3,5.5 });
        ResizableArray<int>  y;
        x.Append(6.6);
        EXPECT(x.NumberOfItems() == 3);
        EXPECT(x.MaxNumberOfItems() == 4);
        EXPECT(x[0] == 3.3);
        EXPECT(x[1] == 5.5);
        EXPECT(x[2] == 6.6);

        x.Append(7.7);
        EXPECT(x.NumberOfItems() == 4);
        EXPECT(x.MaxNumberOfItems() == 4);
        EXPECT(x[3] == 7.7);

        x.Append(8.8);
        EXPECT(x.NumberOfItems() == 5);
        EXPECT(x.MaxNumberOfItems() == 8);
        EXPECT(x[4] == 8.8);

        EXPECT(y.NumberOfItems() == 0);
        EXPECT(y.MaxNumberOfItems() == 0);

        y.Append(17);
        EXPECT(y.NumberOfItems() == 1);
        EXPECT(y.MaxNumberOfItems() == 1);
        EXPECT(y[0] == 17);

        y.Append(21);
        EXPECT(y.NumberOfItems() == 2);
        EXPECT(y.MaxNumberOfItems() == 2);
        EXPECT(y[1] == 21);
    },
    CASE("ResizableArray: CopyFrom")
    {
        ResizableArray<int> x({ 3,5,9,1,13,2 });
        ResizableArray<int> y({ 1,2 });
        ResizableArray<int> z;
        ResizableArray<int> w;

        y.CopyFrom(x);
        z.CopyFrom(x,0,2);
        w.CopyFrom(x,0, EXUstd::InvalidIndex);

        EXPECT(x.NumberOfItems() == 6);
        EXPECT(ToString(x) == "[3 5 9 1 13 2]");
        EXPECT(ToString(y) == "[3 5 9 1 13 2]");
        EXPECT(ToString(w) == "[3 5 9 1 13 2]");
        EXPECT(ToString(z) == "[3 5]");
    },
    CASE("ResizableArray: GetIndexOfItem (Find)")
    {
        ResizableArray<Real> x({ 3.3, 5, 2.2, 9, 3, 13, 2.2 });
        ResizableArray<Real> y;

        Index a = x.GetIndexOfItem(2.2);
        Index b = x.GetIndexOfItem(3.3);
        Index c = x.GetIndexOfItem(13);
        Index d = x.GetIndexOfItem(14);
        Index e = y.GetIndexOfItem(0);
        EXPECT(a == 2);
        EXPECT(b == 0);
        EXPECT(c == 5);
        EXPECT(d == EXUstd::InvalidIndex);
        EXPECT(e == EXUstd::InvalidIndex);
    },
    CASE("ResizableArray: AppendIfItemNotFound")
    {
        ResizableArray<Real> x({ 3.3, 5, 2.2, 9, 3, 13, 2.2 });
        ResizableArray<Real> y;

        Index a = x.AppendIfItemNotFound(2.2);
        Index b = x.AppendIfItemNotFound(4.4);
        Index c = y.AppendIfItemNotFound(2.2);
        EXPECT(a == 2);
        EXPECT(b == 7);
        EXPECT(c == 0);
    },
    CASE("ResizableArray: Insert")
    {
        ResizableArray<Real> x({ 3.3,5,2.2});

        x.Insert(2, 8.1);
        EXPECT(ToString(x) == "[3.3 5 8.1 2.2]");
        x.Insert(0, 8.2);
        x.Insert(5, 8.3); //this is the highest possible position (== append)
        EXPECT(ToString(x) == "[8.2 3.3 5 8.1 2.2 8.3]");
    },
    CASE("ResizableArray: Insert into emtpy array")
    {
        ResizableArray<Real> y;

        y.Insert(0, 8.4);
        EXPECT(y[0] == 8.4);
    },
    CASE("ResizableArray: Remove")
    {
        ResizableArray<Real> x({ 3.3,5,2.2,1.1 });

        x.Remove(3);
        EXPECT(ToString(x) == "[3.3 5 2.2]");

        x.Remove(0);
        EXPECT(ToString(x) == "[5 2.2]");
    },
    CASE("ResizableArray: Sort")
    {
        ResizableArray<Real> x({ 3.3,5,2.2,9,3,13,2.2 });
        x.Sort();
        EXPECT(ToString(x) == "[2.2 2.2 3 3.3 5 9 13]");
    }
};

const lest::test slimArray_specific_test[] =
{

    CASE("SlimArray<Real>: constructor with scalar value, size 0")
    {
        SlimArray<Real, 2> x(2); //initalizes with 2 Reals

        EXPECT(x.NumberOfItems() == 2);
        EXPECT(x[0] == 2.0);
        EXPECT(x[1] == 2.0);
    },
    CASE("SlimArray<Real>: constructor sub-ResizableArray")
    {
        ResizableArray<Real> x({ 3.3,5,2.2,1.1 });
        SlimArray<Real, 2> y(x,1); //initializes with 2 components, starting at position 1 in array x

        EXPECT(y.NumberOfItems() == 2);
        EXPECT(y[0] == 5.);
        EXPECT(y[1] == 2.2);
    },
};

const lest::test objectContainer_specific_test[] =
{

    CASE("ObjectContainer<Real>: constructor, Append")
    {
        ObjectContainer<Real> areal;
        EXPECT(areal.NumberOfItems() == 0);
        EXPECT(areal.MaxNumberOfItems() == 0);

        Real r[5] = { 1.1,2,3,4,5 };

        areal.Append(r[0]);
        EXPECT(areal.NumberOfItems() == 1);
        EXPECT(areal.MaxNumberOfItems() == 1);
        EXPECT(areal[0] == 1.1);

        areal.Append(r[1]);
        EXPECT(areal.NumberOfItems() == 2);
        EXPECT(areal.MaxNumberOfItems() == 2);
        EXPECT(areal[0] == 1.1);
        EXPECT(areal[1] == 2);

        areal.Append(r[2]);
        EXPECT(areal.NumberOfItems() == 3);
        EXPECT(areal.MaxNumberOfItems() == 4);
        EXPECT(areal[0] == 1.1);
        EXPECT(areal[1] == 2);
        EXPECT(areal[2] == 3);
    },
    CASE("ObjectContainer<Real>: Initializer list, copy constructor, operator[], operator=, operator==")
    {
        ObjectContainer<Real> areal({ 1, 2, 3.3 });
        ObjectContainer<Vector> avector({ {1, 2}, {3, 4, 5.5} });

        EXPECT(areal.NumberOfItems() == 3);
        EXPECT(ToString(areal) == "1\n2\n3.3\n");

        EXPECT(avector.NumberOfItems() == 2);
        EXPECT(ToString(avector) == "[1 2]\n[3 4 5.5]\n");
        EXPECT(ToString(avector[0]) == "[1 2]");
        EXPECT(ToString(avector[1]) == "[3 4 5.5]");

        ObjectContainer<Real> areal2 = areal;
        ObjectContainer<Vector> avector2;
        ObjectContainer<Vector> avector3;
        avector2 = avector;
        avector3 = avector;
        avector3[1][2] = 7.7;
        EXPECT(ToString(avector3) == "[1 2]\n[3 4 7.7]\n");

        //EXPECT(areal == areal2);
        //EXPECT(avector == avector2);
        //EXPECT_NOT(avector == avector3);

    },
    CASE("ObjectContainer<Real>: Insert, Remove, Flush")
    {
        ObjectContainer<Real> areal;

        Real r[5] = { 1.1,2,3,4,5 };

        areal.Append(r[0]);
        areal.Append(r[1]);
        areal.Append(r[2]);

        areal.Insert(0, 8.8);
        EXPECT(ToString(areal) == "8.8\n1.1\n2\n3\n");

        areal.Insert(2, 7.);
        EXPECT(ToString(areal) == "8.8\n1.1\n7\n2\n3\n");
        EXPECT(areal.NumberOfItems() == 5);
        EXPECT(areal.MaxNumberOfItems() == 8);

        areal.Remove(1);
        EXPECT(ToString(areal) == "8.8\n7\n2\n3\n");

        areal.Remove(3);
        EXPECT(ToString(areal) == "8.8\n7\n2\n");

        areal.Remove(0);
        EXPECT(ToString(areal) == "7\n2\n");
        EXPECT(areal.NumberOfItems() == 2);
        EXPECT(areal.MaxNumberOfItems() == 8);

        areal.Flush();
        EXPECT(ToString(areal) == "");
        EXPECT(areal.NumberOfItems() == 0);
        EXPECT(areal.MaxNumberOfItems() == 0);

        areal.Append(r[0]);
        areal.Append(r[1]);
        areal.Append(r[2]);
        EXPECT(areal.NumberOfItems() == 3);
        EXPECT(areal.MaxNumberOfItems() == 4);
        EXPECT(ToString(areal) == "1.1\n2\n3\n");

        areal.SetMaxNumberOfItems(0);
        EXPECT(areal.NumberOfItems() == 0);
        EXPECT(areal.MaxNumberOfItems() == 0);

    },
    CASE("ObjectContainer<Vector>: constructor, append, SetMaxNumberOfItems")
    {
        ObjectContainer<Vector> avector;

        Vector v1({ 1,2,3 });
        Vector v2({ 2.5,3,4 });
        Vector v3({ 4,6,8 });

        avector.Append(v1);
        avector.Append(v2);
        avector.Append(v3);

        EXPECT(avector.NumberOfItems() == 3);
        EXPECT(avector.MaxNumberOfItems() == 4);
        EXPECT(ToString(avector) == "[1 2 3]\n[2.5 3 4]\n[4 6 8]\n");

        avector.Remove(1);
        EXPECT(avector.NumberOfItems() == 2);
        EXPECT(avector.MaxNumberOfItems() == 4);
        EXPECT(ToString(avector) == "[1 2 3]\n[4 6 8]\n");

        avector.Flush();
        avector.Flush(); //check if twise flush works

        avector.SetMaxNumberOfItems(49);
        for (int i=0; i < 49; i++)
        {
            avector.Append(Vector({ 1,2,3,4 }));
        }
        EXPECT(avector.NumberOfItems() == 49);
        EXPECT(avector.MaxNumberOfItems() == 49);

        for (int i=0; i < 48; i++) //inefficient remove, takes approx. 2500 pointer relocations
        {
            avector.Remove(0);
        }
        EXPECT(ToString(avector) == "[1 2 3 4]\n"); //one Vector left

    },
    CASE("ObjectContainer<Vector>: initializer list, insert, remove")
    {
        ObjectContainer<Vector> avector({ { 1., 2. },{ 3.3 }, {4., 5.} });
        Vector v1({ 1 });
        Vector v2({ 2.2, 3 });

        avector.Insert(0, v1);
        avector.Insert(2, v2);

        avector.Remove(1);
        avector.Remove(2);
        avector.Remove(2);
        EXPECT(avector.NumberOfItems() == 2);
        EXPECT(avector.MaxNumberOfItems() == 6);
        EXPECT(ToString(avector) == "[1]\n[2.2 3]\n");
    },
    CASE("ObjectContainer<Vector>: initializer list, GetIndexOfItemPointer")
    {
        ObjectContainer<Vector> avector({ { 1, 2 },{ 3, 2.2 },{ 4, 5 } });
        Vector v1({ 3, 2.2 });
        Vector v2({ 4, 5 });
        Vector v3({ 1, 2 });

        EXPECT(avector.GetIndexOfItem(v1) == 1); //operator== must have equal numberOfItems in Vector; otherwise: assertion
        EXPECT(avector.GetIndexOfItem(v2) == 2);
        EXPECT(avector.GetIndexOfItem(v3) == 0);
    },
};

#endif
