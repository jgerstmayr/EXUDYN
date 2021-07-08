/** ***********************************************************************************************
* @brief		file including test routines, used in Python
* @details		Details:
				- implementation of some test functions of PybindTests.h
*
* @author		Gerstmayr Johannes
* @date			2021-05-02 
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */

//++++++++++++++++++++++++++
//#include <iostream>
//#include <cstdlib>
//#include <array>
//#include <vector>
//#include <signal.h>
////++++++++++++++++++++++++++

// pybind11 includes
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
//#include <pybind11/operators.h>
#include <pybind11/numpy.h>       //interface to numpy
//#include <pybind11/buffer_info.h> //passing reference to matrix to numpy
#include <pybind11/embed.h>       //scoped interpreter
////does not work globally: #include <pybind11/iostream.h> //used to redirect cout:  py::scoped_ostream_redirect output;
//#include <pybind11/cast.h> //for arguments
//#include <pybind11/functional.h> //for function handling ... otherwise gives a python error (no compilation error in C++ !)
namespace py = pybind11;
//using namespace pybind11::literals; //brings in the '_a' literals; e.g. for short arguments definition


#include "Pymodules/PybindTests.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//test functions, will be eliminated from time to time
#include "Main/OutputVariable.h"

void PyTest()
{
	//add testing here
	if (0)
	{
		Vector v1({ 1.,2.,3. });
		Vector v2({ 8.,7.,6. });
		Matrix m(3, 2, { 1,2,3, 4,5,6 });
		Vector r({ 0.,1. });

		EXUmath::MultMatrixVectorAdd(m, v1, r);
		pout << "r1=" << r << "\n";				//=> (14, 33)
		EXUmath::MultMatrixVectorAdd(m, v2, r);
		pout << "r2=" << r << "\n";				//=> (54,136)
	}
	if (0)
	{
		//Matrix m(3, 3, { 1,1,1, 1,1,0, 0,0,1 });
		Matrix m(3, 3, { 1,1,1, 0,1,0, 0,1,0 });
		Matrix minv = m;
		ResizableMatrix temp;
		ArrayIndex rows;
		//minv.Invert();
		Index success = minv.InvertSpecial(temp, rows, true, 0, 0.);
		pout << "success=" << success << "\n";
		//pout << "Minv=" << minv << ", rows=" << rows << ", I=" << minv * m << "\n";
		//pout << "Minv*[1,-2,3]=" << minv * Vector({1,-2,3}) << "\n";

		//minv = m;
		//success = minv.Invert();
		//pout << "success2=" << success << "\n";
		//pout << "Minv2=" << minv << "\n";
	}
	if (0) //test if conversion to sensor and back works
	{
		Index mbsNumber = 0;
		Index i = 0;
		//ItemType myType = ItemType::Sensor;
		ItemType myType = ItemType::Marker;
		Index itemID = Index2ItemID(i, myType, mbsNumber);
		pout << "itemID=" << itemID << "\n";

		Index iNew;
		ItemType typeNew;
		ItemID2IndexType(itemID, iNew, typeNew, mbsNumber);
		pout << "type=" << typeNew << ", index="<< iNew << ", mbs=" << mbsNumber << "\n";
	}
	if (0)
	{
		////! access to internal module
		//void PyGetInternalSysDictionary()
		//{
		//	//internalSystemDictionary["abc"] = 123;
		//	//py::print(internalSystemDictionary["abc"]);
		//	STDstring key = "aaa";
		//	Real item = 1.23;
		//	py::module exudynCPP = py::module::import("exudyn");
		//	exudynCPP.attr("sys")[key.c_str()] = item;
		//
		//	pout << "test\n";
		//}
	}
	if (1)
	{
		//SlimArray<Real, 3> a({ 3,5,1 });
		//pout << "a=" << a << "\n";
		//a.Sort();
		//pout << "a sorted=" << a << "\n";

	}
}


py::object GetVector()
{
	Vector v({ 42.1234567890123456,43.,44. }); //double precision maintained in NumPy array in python
	return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer()); //copy array (could also be referenced!)
}

//py::object GetMatrix()
py::array_t<Real> GetMatrix()
{
	ResizableMatrix m(2, 3, { 12.5,13,14,  15,16,17 }); //double precision maintained in NumPy array in python

	return py::array_t<Real>(std::vector<std::ptrdiff_t>{(int)m.NumberOfRows(), (int)m.NumberOfColumns()}, m.GetDataPointer()); //copy array (could also be referenced!)
}

void SeeMatrix(py::array_t<Real> pyArray)
{
	pout << "ndim=" << pyArray.ndim() << "\n";
	pout << "size=" << pyArray.size() << "\n";
	if (pyArray.ndim() == 2)
	{
		auto mat = pyArray.unchecked<2>();
		Index nrows = (Index)mat.shape(0);
		Index ncols = (Index)mat.shape(1);

		Matrix m(nrows, ncols);
		for (Index i = 0; i < nrows; i++)
		{
			for (Index j = 0; j < ncols; j++)
			{
				m(i, j) = mat(i, j);
			}
		}
		pout << "Matrix m=" << m << "\n";
	}
}


//void PythonCreateSys10()
//{
//	py::exec(R"(
//		systemContainer = exu.SystemContainer()
//		mbs = systemContainer.AddSystem()
//		for i in range(10): mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [i, 0.0, 0.0],'initialCoordinates': [0.0, 0.0, (i+1)/10], 'initialVelocities': [0.0, i*2, 0.0],})
//		for i in range(10): mbs.AddObject({'objectType': 'MassPoint', 'physicsMass': 20-i, 'nodeNumber': i})
//		for i in range(10): mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': i,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})
//		for i in range(9): mbs.AddObject({'objectType': 'SpringDamper', 'stiffness': 4, 'damping': 0, 'force': 0, 'referenceLength':1, 'markerNumbers': [i,i+1]})		
//		mbs
//    )");
//	//pout << "ready to go\n";
//}

//py::object GetIndex()
//{
//	return py::int_(42);
//}
//
//py::object GetString()
//{
//	return py::str("A string");
//}
//
//py::object GetReal()
//{
//	return py::float_(double(42.1234567890123456)); //precision maintained in python: 42.123456789012344
//}
//
//py::object GetVector()
//{
//	Vector v({ 42.1234567890123456,43,44 }); //double precision maintained in NumPy array in python
//	return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer()); //copy array (could also be referenced!)
//}
//
//py::object GetOutputVariableType() //as OutputVariableType is known, it can return the special type
//{
//	return py::cast(OutputVariableType::Position);
//}

////Exudyn-access to test Vector
//const Vector& GetTest() { return test; }
//void SetTest(const Vector& vector) { test = vector; }

//Numpy-access to test Vector
//py::array_t<Real> GetNumpyTest() { return py::array_t<Real>(test.NumberOfItems(), test.GetDataPointer()); }
//void SetNumpyTest(const std::vector<Real>& v) { test = v; }

//end test functions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PythonAlive()
{
	pout << "pout:Exudyn is alive\n";

	EXUmath::MatrixTests(); //perform matrix inverse tests
}

void PythonGo()
{
	py::exec(R"(
import exudyn
systemContainer = exudyn.SystemContainer()
mbs = systemContainer.AddSystem()
    )");
	pout << "main variables:\n systemContainer=exudyn.SystemContainer()\n mbs = systemContainer.AddSystem()\n";
	//pout << "ready to go\n";
}

#ifdef _MYDEBUG //only in debug mode!
void CreateTestSystem(Index systemNumber, Index arg0, Index arg1)
{
	if (systemNumber == 0)
	{
		STDstring str = "n = " + EXUstd::ToString(arg0) + "\n";
		py::exec(str.c_str());
		py::exec(R"(
		from itemInterface import *
		SC = exu.SystemContainer()
		mbs = SC.AddSystem()

		#n = 5    #nBodies
		k = 1000 #stiffness
		d = 10   #damping
		L = 0.25 #distance between nodes
		lastBodyMarker = -1
		mBody = 0
		body = 0
		node = 0

		nodeGround = mbs.AddNode(NodePointGround(referenceCoordinates = [0, 0, 0])) #ground node for coordinate constraint
		mNodeGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nodeGround, coordinate = 0)) #Ground node == > no action

		for j in range(n) :
			if (j == 0) :
				body = mbs.AddObject({ 'objectType': 'Ground', 'referencePosition' : [0,j,0] })
			else :
				node = mbs.AddNode(NodePoint2D(referenceCoordinates = [L*(j + 1), 0], initialCoordinates = [0, 0]))
				body = mbs.AddObject(MassPoint2D(physicsMass = 10, nodeNumber = node))
				#constrain y - direction:

			mNode = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = node, coordinate = 1)) #constrain y - direction
			mbs.AddObject(ObjectConnectorCoordinateSpringDamper(markerNumbers = [mNodeGround, mNode], stiffness = k, damping = d, visualization = VObjectConnectorCoordinateSpringDamper(show = False)))

				#load:
			mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber = body, localPosition = [0, 0, 0]))
			if j != 0 : mbs.AddLoad({ 'loadType': 'ForceVector',  'markerNumber' : mBody,  'loadVector' : [0, -25, 0] })

			#print("mBody =", mBody, ", lastBodyMarker =", lastBodyMarker)
			#add spring - damper between objects
			if lastBodyMarker != -1:
				mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers = [mBody, lastBodyMarker], referenceLength = L, stiffness = k, damping = d))

			lastBodyMarker = mBody

		mbs.Assemble()
	    )");
	}
	else { PyWarning("CreateTestSystem: no valid System number!"); }

}
#endif






