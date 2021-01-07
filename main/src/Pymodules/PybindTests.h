/** ***********************************************************************************************
* @file			PybindTests.h
* @brief		This file contains test functions for pybind11 
* @details		Details:
* 				- put tests here, which are useful for later developements
*
* @author		Gerstmayr Johannes
* @date			2019-05-02 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef PYBINDTESTS__H
#define PYBINDTESTS__H

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//PYBIND TESTS

#include "Linalg/BasicLinalg.h"

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

#endif
