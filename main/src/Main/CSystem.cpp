/** ***********************************************************************************************
* @brief		Implementation of CSystem
*
* @author		Gerstmayr Johannes
* @date			2019-04-18 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#pragma once

//++++++++++++++++ sleep_for():
#include <iostream>
#include <chrono>
#include <thread>
//++++++++++++++++

#include <functional>					//for std::invoke
#include "Main/CSystemData.h"			//Basics, Vector/Array, OutputVariable, CData, Material, Body, Node, Marker, Load
#include "Main/CSystem.h"	

#include "Main/MainSystem.h"
#include "Linalg/LinearSolver.h" //for GeneralMatrixEXUdense
#include "Main/OutputVariable.h" //for GeneralMatrixEXUdense

#include "Utilities/TimerStructure.h" //for local CPU time measurement

//#define USE_AUTODIFF


//! Prepare a newly created System of nodes, objects, loads, ... for computation
void CSystem::Assemble(const MainSystem& mainSystem)
{
	//pout << "++++++++++++++++\nCheckSystemIntegrity ...\n";
	if (CheckSystemIntegrity(mainSystem)) //checks prior to Assemble() ==> after Assemble(), everything shall be ok.
	{
		//pout << "                        ... ok\n";
		AssembleCoordinates(mainSystem);
		AssembleLTGLists(mainSystem);
		AssembleInitializeSystemCoordinates(mainSystem); //mainSystem needed for initial displacements

		//now system is consistent and can safely be drawn
		SetSystemIsConsistent(true);
		postProcessData.postProcessDataReady = true;
		cSystemData.isODE2RHSjacobianComputation = false; //hack
	}
	else
	{
		SetSystemIsConsistent(false);
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! Check if all references are valid (body->node number, marker->body/nodenumber, load->marker, etc.);
//  Function is called before 
bool CSystem::CheckSystemIntegrity(const MainSystem& mainSystem)
{
	//check that MarkerNodeCoordinate has a valid coordinate
	//check that initial values for generic nodes are consistent (e.g. NodeGenericData)
	//general check for length of initial coordinate (and time derivatives) is correct is checked in AssembleInitializeSystemCoordinates()
	//add CheckSystemIntegrity function to all MainItems?

	STDstring errorString;
	Index itemIndex;
	bool systemIsInteger = true;

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Index numberOfNodes = mainSystem.GetMainSystemData().GetMainNodes().NumberOfItems();
	Index numberOfObjects = mainSystem.GetMainSystemData().GetMainObjects().NumberOfItems();
	Index numberOfMarkers = mainSystem.GetMainSystemData().GetMainMarkers().NumberOfItems();
	Index numberOfLoads = mainSystem.GetMainSystemData().GetMainLoads().NumberOfItems();

	itemIndex = 0;
	for (MainNode* mainNode : mainSystem.GetMainSystemData().GetMainNodes())
	{
		CNode* node = mainNode->GetCNode();
		if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::ODE2variables)
		{
			Index numberOfCoordinates = node->GetNumberOfODE2Coordinates();
			if (numberOfCoordinates)
			{
				if (numberOfCoordinates != mainNode->GetInitialVector().NumberOfItems()) {
					PyError(STDstring("Node ") + EXUstd::ToString(itemIndex) + " '" + mainNode->GetName() + "'" + "(type=" + mainNode->GetTypeName() + ") has inconsistent size of initial displacement coordinates vector (" +
						EXUstd::ToString(mainNode->GetInitialVector().NumberOfItems()) + ") != number of nodal ODE2 coordinates (" + EXUstd::ToString(numberOfCoordinates) + ")");
					systemIsInteger = false;
				}

				if (numberOfCoordinates != mainNode->GetInitialVector_t().NumberOfItems()) {
					PyError(STDstring("Node ") + EXUstd::ToString(itemIndex) + " '" + mainNode->GetName() + "'" + "(type=" + mainNode->GetTypeName() + ") has inconsistent size of initial velocity coordinate vector (" +
						EXUstd::ToString(mainNode->GetInitialVector_t().NumberOfItems()) + ") != number of nodal ODE2 coordinates (" + EXUstd::ToString(numberOfCoordinates) + ")");
					systemIsInteger = false;
				}
			}
		}
		else if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::DataVariables)
		{
			Index numberOfCoordinates = node->GetNumberOfDataCoordinates();
			if (numberOfCoordinates)
			{
				if (numberOfCoordinates != mainNode->GetInitialVector().NumberOfItems()) {
					PyError(STDstring("Node ") + EXUstd::ToString(itemIndex) + " '" + mainNode->GetName() + "'" + "(type=" + mainNode->GetTypeName() + ") has inconsistent size of initial coordinates vector (" +
						EXUstd::ToString(mainNode->GetInitialVector().NumberOfItems()) + ") != number of nodal Data coordinates (" + EXUstd::ToString(numberOfCoordinates) + ")");
					systemIsInteger = false;
				}
			}
		}
		else // ODE1 or AE variables
		{
			Index numberOfCoordinates = node->GetNumberOfAccessibleCoordinates(); //ODE2+ODE1+AE
			if (numberOfCoordinates)
			{
				if (numberOfCoordinates != mainNode->GetInitialVector().NumberOfItems()) {
					PyError(STDstring("Node ") + EXUstd::ToString(itemIndex) + " '" + mainNode->GetName() + "'" + "(type=" + mainNode->GetTypeName() + ") has inconsistent size of initial coordinates vector (" +
						EXUstd::ToString(mainNode->GetInitialVector().NumberOfItems()) + ") != number of nodal coordinates (" + EXUstd::ToString(numberOfCoordinates) + ")");
					systemIsInteger = false;
				}
			}
		}
		itemIndex++;
	}
	if (!systemIsInteger) { return false; }

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//check for valid node numbers (objects)
	//check for valid marker numbers in connectors
	itemIndex = 0;
	for (auto* item : mainSystem.GetMainSystemData().GetMainObjects())
	{
		//GetRequestedNodeType() must be implemented for all objects with nodes
		for (Index i = 0; i < item->GetCObject()->GetNumberOfNodes(); i++)
		{
			Index itemIndex = item->GetCObject()->GetNodeNumber(i);
			if (!EXUstd::IndexIsInRange(itemIndex, 0, numberOfNodes))
			{
				PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() + ", local node " +
					EXUstd::ToString(i) + " contains invalid (global) node number " + EXUstd::ToString(itemIndex));
				systemIsInteger = false;
			}
			else //check if right nodeTypes are used
			{
				CNode* cNode = mainSystem.GetMainSystemData().GetMainNodes()[itemIndex]->GetCNode();
				//if ((item->GetRequestedNodeType() & cNode->GetType()) != item->GetRequestedNodeType())
				if (!EXUstd::IsOfType(cNode->GetType(), item->GetRequestedNodeType()))
				{
					PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() + ", local node " +
						EXUstd::ToString(i) + " (global index = " + EXUstd::ToString(itemIndex) + ")" + 
						" contains invalid node type " + Node::GetTypeString(cNode->GetType()) +
						" while the requested node type was '" + Node::GetTypeString(item->GetRequestedNodeType()) + "'");
					systemIsInteger = false;
				}
			}
		}

		if ((Index)item->GetCObject()->GetType() & (Index)CObjectType::Connector)
		{
			CObjectConnector* connector = (CObjectConnector*)item->GetCObject();
			if (connector->GetMarkerNumbers().NumberOfItems() != 2)
			{
				PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() + 
					" must have two markers, but got " + EXUstd::ToString(connector->GetMarkerNumbers().NumberOfItems()) + " markers");
				systemIsInteger = false;
			}
			else
			{
				if (connector->GetMarkerNumbers().NumberOfItems() == 2) //for future cases
				{
					if (connector->GetMarkerNumbers()[0] == connector->GetMarkerNumbers()[1])
					{
						PyWarning(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + ", contains two identical markers");
					}
				}
				for (Index i = 0; i < connector->GetMarkerNumbers().NumberOfItems(); i++)
				{
					Index itemIndex = connector->GetMarkerNumbers()[i];
					if (!EXUstd::IndexIsInRange(itemIndex, 0, numberOfMarkers))
					{
						PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() + ", local marker " +
							EXUstd::ToString(i) + " contains invalid (global) marker number " + EXUstd::ToString(itemIndex));
						systemIsInteger = false;
					}
					else
					{
						//now check Marker::Type flags
						CMarker* marker = mainSystem.GetMainSystemData().GetMainMarkers()[itemIndex]->GetCMarker();
						if ((connector->GetRequestedMarkerType() & marker->GetType()) != connector->GetRequestedMarkerType()) //marker must contain all requested flags
						{
							PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() + ", local marker " +
								EXUstd::ToString(i) + " contains marker with invalid type '" + Marker::GetTypeString(marker->GetType()) +
								"', but expected marker type '" + Marker::GetTypeString(connector->GetRequestedMarkerType()) + "'");
							systemIsInteger = false;
						}
					}
				}

				if ((Index)item->GetCObject()->GetType() & (Index)CObjectType::Constraint)
				{
					if (((connector->GetAvailableJacobians() & JacobianType::AE_ODE2) != 0) != ((connector->GetAvailableJacobians() & JacobianType::AE_ODE2_function) != 0))
					{
						PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() +
							": Internal error: connector JacobianType::AE_ODE2 must be consistent with JacobianType::AE_ODE2_function");
						systemIsInteger = false;
					}
					if (((connector->GetAvailableJacobians() & JacobianType::AE_AE) != 0) != ((connector->GetAvailableJacobians() & JacobianType::AE_AE_function) != 0))
					{
						PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() +
							": Internal error: connector JacobianType::AE_AE must be consistent with JacobianType::AE_AE_function");
						systemIsInteger = false;
					}
				}
			}

		}


		itemIndex++;
	}


	if (!systemIsInteger) { return false; }

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//check for valid node/object numbers in markers; special markers can contain node+body!
	for (auto* item : mainSystem.GetMainSystemData().GetMainMarkers())
	{
		if (item->GetCMarker()->GetType() & Marker::Node)
		{
			Index itemIndex = item->GetCMarker()->GetNodeNumber();
			if (!EXUstd::IndexIsInRange(itemIndex, 0, numberOfNodes))
			{
				PyError(STDstring("Marker ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() + 
					", contains invalid (global) node number " + EXUstd::ToString(itemIndex));
				systemIsInteger = false;
			}
		}
		//else //must be object (usually body, but could also be connector)
		if (item->GetCMarker()->GetType() & Marker::Object) //might also be Marker::Body
		{
			Index itemIndex = item->GetCMarker()->GetObjectNumber();
			if (!EXUstd::IndexIsInRange(itemIndex, 0, numberOfObjects))
			{
				PyError(STDstring("Marker ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() +
					", contains invalid (global) object number " + EXUstd::ToString(itemIndex));
				systemIsInteger = false;
			}
		}

		itemIndex++;
	}


	if (!systemIsInteger) { return false; }

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//check for valid marker numbers in loads; check requested MarkerType; 
	for (auto* item : mainSystem.GetMainSystemData().GetMainLoads())
	{
		Index itemIndex = item->GetCLoad()->GetMarkerNumber();

		if (!EXUstd::IndexIsInRange(itemIndex, 0, numberOfMarkers))
		{
			PyError(STDstring("Load ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() + 
				", contains invalid marker number " + EXUstd::ToString(itemIndex));
			systemIsInteger = false;
		}
		
		if (systemIsInteger) //only if markerNumber is valid
		{
			Marker::Type requestedType = item->GetCLoad()->GetRequestedMarkerType();
			Marker::Type markerType = mainSystem.GetMainSystemData().GetMainMarkers()[itemIndex]->GetCMarker()->GetType();
			if ((requestedType & markerType) != requestedType)
			{
				PyError(STDstring("Load ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() +
					", contains marker with invalid type '" + Marker::GetTypeString(markerType) +
					"', but expected marker type '" + Marker::GetTypeString(requestedType) + "'");
				systemIsInteger = false;
			}
		}
		if (systemIsInteger) //only if markerNumber is valid
		{
			CLoad* cLoad = item->GetCLoad();

			if (cLoad->IsBodyFixed() && ((mainSystem.GetCSystem()->GetSystemData().GetCMarker(cLoad->GetMarkerNumber()).GetType() & Marker::Orientation) == 0))
			{
				PyError(STDstring("Load ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type=" + item->GetTypeName() +
					": marker (marker number = " + EXUstd::ToString(cLoad->GetMarkerNumber()) + 
					") must provide orientation (e.g. RigidBody marker) in case that bodyFixed == True");
				systemIsInteger = false;
			}
		}

		itemIndex++;
	}

	if (!systemIsInteger) { return false; } //avoid crashes due to further checks!

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//check for valid sensors: valid node/object/... numbers and valid OutputVariableTypes

	itemIndex = 0;
	for (auto* item : mainSystem.GetMainSystemData().GetMainSensors())
	{
		if (item->GetCSensor()->GetType() == SensorType::Node)
		{
			Index n = item->GetCSensor()->GetNodeNumber();
			if (!EXUstd::IndexIsInRange(n, 0, numberOfNodes))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type = SensorType::Node, contains invalid node number " + EXUstd::ToString(n));
			}
			else if (!EXUstd::IsOfTypeAndNotNone(mainSystem.GetMainSystemData().GetMainNode(n).GetCNode()->GetOutputVariableTypes(), item->GetCSensor()->GetOutputVariableType()))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + 
					"', type = SensorType::Node: OutputVariableType '" + GetOutputVariableTypeString(item->GetCSensor()->GetOutputVariableType()) + "' is not available in node with node number " + EXUstd::ToString(n));
			}
		}
		else if (item->GetCSensor()->GetType() == SensorType::Body)
		{
			Index n = item->GetCSensor()->GetObjectNumber();
			if (!EXUstd::IndexIsInRange(n, 0, numberOfObjects))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type = SensorType::Body, contains invalid object number " + EXUstd::ToString(n));
			} 
			else if (((Index)mainSystem.GetMainSystemData().GetMainObjects()[n]->GetCObject()->GetType() & (Index)CObjectType::Body) == 0)
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type = SensorType::Body, contains invalid object (ID=" + EXUstd::ToString(n) + ") which is not of ObjectType::Body");
			}
			else if (!EXUstd::IsOfTypeAndNotNone(mainSystem.GetMainSystemData().GetMainObjects()[n]->GetCObject()->GetOutputVariableTypes(), item->GetCSensor()->GetOutputVariableType()))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() +
					"', type = SensorType::Body: OutputVariableType '" + GetOutputVariableTypeString(item->GetCSensor()->GetOutputVariableType()) + "' is not available in object with object number " + EXUstd::ToString(n));
			}
		}
		else if (item->GetCSensor()->GetType() == SensorType::Object)
		{
			Index n = item->GetCSensor()->GetObjectNumber();
			if (!EXUstd::IndexIsInRange(n, 0, numberOfObjects))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type = SensorType::Object, contains invalid object number " + EXUstd::ToString(n));
			}
			else if (!EXUstd::IsOfTypeAndNotNone(mainSystem.GetMainSystemData().GetMainObjects()[n]->GetCObject()->GetOutputVariableTypes(), item->GetCSensor()->GetOutputVariableType()))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() +
					"', type = SensorType::Object: OutputVariableType '" + GetOutputVariableTypeString(item->GetCSensor()->GetOutputVariableType()) + "' is not available in object with object number " + EXUstd::ToString(n));
			}
		}
		else if (item->GetCSensor()->GetType() == SensorType::Marker)
		{
			CHECKandTHROWstring("SensorType::Marker: CheckSystemIntegrity not implemented!");
			Index n = item->GetCSensor()->GetMarkerNumber();
			if (!EXUstd::IndexIsInRange(n, 0, numberOfMarkers))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type = SensorType::Marker, contains invalid marker number " + EXUstd::ToString(n));
			}
			//else if (!EXUstd::IsOfType(mainSystem.GetMainSystemData().GetMainMarkers()[n]->GetCMarker()->GetOutputVariableTypes(), item->GetCSensor()->GetOutputVariableType()))
			//{
			//	PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() +
			//		"', type = SensorType::Marker: OutputVariableType '" + GetOutputVariableTypeString(item->GetCSensor()->GetOutputVariableType()) + "' is not available in marker with marker number " + EXUstd::ToString(n));
			//}
		}
		else if (item->GetCSensor()->GetType() == SensorType::Load)
		{
			Index n = item->GetCSensor()->GetLoadNumber();
			if (!EXUstd::IndexIsInRange(n, 0, numberOfLoads))
			{
				PyError(STDstring("Sensor ") + EXUstd::ToString(itemIndex) + ", name = '" + item->GetName() + "', type = SensorType::Load, contains invalid load number " + EXUstd::ToString(n));
			}
		}
		else
		{
			PyWarning("CheckSystemIntegrity: sensor type not implemented");
		}

		itemIndex++;
	}
	if (!systemIsInteger) { return false; } //avoid crashes due to further checks!

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//now do object-specific checks!
	itemIndex = 0;
	for (MainNode* item : mainSystem.GetMainSystemData().GetMainNodes())
	{
		if (!item->CheckPreAssembleConsistency(mainSystem, errorString))
		{
			PyError(STDstring("Node ") + EXUstd::ToString(itemIndex) + STDstring(" contains inconsistent data:\n") + errorString);
			systemIsInteger = false;
		}
		itemIndex++;
	}

	itemIndex = 0;
	for (MainObject* item : mainSystem.GetMainSystemData().GetMainObjects())
	{
		if (!item->CheckPreAssembleConsistency(mainSystem, errorString))
		{
			PyError(STDstring("Object ") + EXUstd::ToString(itemIndex) + STDstring(" contains inconsistent data:\n") + errorString);
			systemIsInteger = false;
		}
		itemIndex++;
	}

	itemIndex = 0;
	for (MainMarker* item : mainSystem.GetMainSystemData().GetMainMarkers())
	{
		if (!item->CheckPreAssembleConsistency(mainSystem, errorString))
		{
			PyError(STDstring("Marker ") + EXUstd::ToString(itemIndex) + STDstring(" contains inconsistent data:\n") + errorString);
			systemIsInteger = false;
		}
		itemIndex++;
	}

	itemIndex = 0;
	for (MainLoad* item : mainSystem.GetMainSystemData().GetMainLoads())
	{
		if (!item->CheckPreAssembleConsistency(mainSystem, errorString))
		{
			PyError(STDstring("Load ") + EXUstd::ToString(itemIndex) + STDstring(" contains inconsistent data:\n") + errorString);
			systemIsInteger = false;
		}
		itemIndex++;
	}


	return systemIsInteger;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! assign coordinate numbers to nodes; LATER: prepare LinkedDataVectors (of coordinates) for nodes
void CSystem::AssembleCoordinates(const MainSystem& mainSystem)
{
	//pout << "Assemble nodes:\n";

	//make global node_DOF lists
	Index globalODE2Index = 0;
	Index globalODE1Index = 0;
	Index globalAEIndex = 0;
	Index globalDataIndex = 0;

	//+++++++++++++++++++++++++++
	//process nodes:
	Index node_ind = 0;
	for (CNode* node : cSystemData.GetCNodes())
	{
		//pout << "Assembling node " << node_ind << "\n";
		if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::ODE2variables)
		{
			//pout << "  NodeODE2 found\n";
			node->SetGlobalODE2CoordinateIndex(globalODE2Index); //use current index
			globalODE2Index += node->GetNumberOfODE2Coordinates(); //add counter in order to track number of ODE2-coordinates
			//pout << "  number of coordinates = " << node->GetNumberOfODE2Coordinates() << "\n";

			if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::AEvariables) //nodes might contain algebraic variables in addition to ODE-coordinates, e.g. Euler parameters
			{
				node->SetGlobalAECoordinateIndex(globalAEIndex); //use current index
				globalAEIndex += node->GetNumberOfAECoordinates(); //add counter in order to track number of AE-coordinates
			}

		}
		else if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::ODE1variables)
		{
			node->SetGlobalODE1CoordinateIndex(globalODE1Index); //use current index
			globalODE1Index += node->GetNumberOfODE1Coordinates(); //add counter in order to track number of ODE1-coordinates
		}
		else if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::AEvariables) //nodes might contain algebraic variables
		{
			node->SetGlobalAECoordinateIndex(globalAEIndex); //use current index
			globalAEIndex += node->GetNumberOfAECoordinates(); //add counter in order to track number of AE-coordinates
		}
		else if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::DataVariables) //
		{
			node->SetGlobalDataCoordinateIndex(globalDataIndex); //use current index
			globalDataIndex += node->GetNumberOfDataCoordinates(); //add counter in order to track number of Data-coordinates
		}
		else { CHECKandTHROWstring("CSystem::Assemble(): invalid node type!"); }
		node_ind++;
	}

	//+++++++++++++++++++++++++++
	//process constraints:
	for (CObject* object : cSystemData.GetCObjects())
	{
		if ((Index)object->GetType() & (Index)CObjectType::Constraint) //only constraints have algebraic variables not linked to nodes
		{
			CObjectConstraint* constraint = (CObjectConstraint*)object;

			constraint->SetGlobalAECoordinateIndex(globalAEIndex);
			//add counter in order to track number of AE-coordinates
			globalAEIndex += constraint->GetAlgebraicEquationsSize(); //number of Lagrange multipliers equals number of algebraic equations
		}
	}

	cSystemData.GetNumberOfCoordinatesODE2() = globalODE2Index;
	cSystemData.GetNumberOfCoordinatesODE1() = globalODE1Index;
	cSystemData.GetNumberOfCoordinatesAE() = globalAEIndex;
	cSystemData.GetNumberOfCoordinatesData() = globalDataIndex;
	//pout << "global ODE2 coordinates = " << globalODE2Index << "\n";
	//pout << "global ODE1 coordinates = " << globalODE1Index << "\n";
	//pout << "global AE coordinates = " << globalAEIndex << "\n";
	//pout << "global Data coordinates = " << globalDataIndex << "\n";
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! build ltg-coordinate lists for objects (used to build global ODE2RHS, MassMatrix, etc. vectors and matrices)
void CSystem::AssembleLTGLists(const MainSystem& mainSystem)
{
	//pout << "Assemble LTG Lists\n";

	//build system-wide local to global lists for objects:
	ObjectContainer<ArrayIndex>& listODE2 = cSystemData.GetLocalToGlobalODE2();
	ObjectContainer<ArrayIndex>& listODE1 = cSystemData.GetLocalToGlobalODE1();
	ObjectContainer<ArrayIndex>& listAE = cSystemData.GetLocalToGlobalAE();
	ObjectContainer<ArrayIndex>& listData = cSystemData.GetLocalToGlobalData();
	listODE2.Flush();
	listODE1.Flush();
	listAE.Flush();
	listData.Flush();

	//temporary lists per object:
	ArrayIndex ltgListODE2;
	ArrayIndex ltgListODE1;
	ArrayIndex ltgListAE;
	ArrayIndex ltgListData;

	//compute localToGlobalODE2 coordinate indices
	for (Index i = 0; i < cSystemData.GetCObjects().NumberOfItems(); i++)
	{
		AssembleObjectLTGLists(i, ltgListODE2, ltgListODE1, ltgListAE, ltgListData);

		listODE2.Append(ltgListODE2);
		listODE1.Append(ltgListODE1);
		listAE.Append(ltgListAE);
		listData.Append(ltgListData);
	}
	//pout << "local to global ODE2 Indices:\n" << listODE2 << "\n\n";
	//pout << "local to global ODE1 Indices:\n" << listODE1 << "\n\n";
	//pout << "local to global AE Indices:\n" << listAE << "\n\n";
	//pout << "local to global Data Indices:\n" << listData << "\n\n";
}


//! build ltg-coordinate lists for object with 'objectIndex' and set indices in ltg lists
void CSystem::AssembleObjectLTGLists(Index objectIndex, ArrayIndex& ltgListODE2, ArrayIndex& ltgListODE1,
	ArrayIndex& ltgListAE, ArrayIndex& ltgListData)
{
	ltgListODE2.SetNumberOfItems(0);
	ltgListODE1.SetNumberOfItems(0);
	ltgListAE.SetNumberOfItems(0);
	ltgListData.SetNumberOfItems(0);

	CObject* object = cSystemData.GetCObjects()[objectIndex];

	if ((Index)object->GetType() & (Index)CObjectType::Body) //single and multinoded objects
	{
		CObjectBody* objectBody = (CObjectBody*)object;

		//bool algebraicEquationsInNodes = false;
		//node-based elements (bodies, finite elements, ...)
		//for (Index nodeNumber : objectBody->GetCNode())
		for (Index j = 0; j < objectBody->GetNumberOfNodes(); j++)
		{
			CNode* node = objectBody->GetCNode(j);
			if (node->GetNumberOfODE2Coordinates())
			{
				Index gIndex = node->GetGlobalODE2CoordinateIndex();
				for (Index i = 0; i < node->GetNumberOfODE2Coordinates(); i++)
				{
					ltgListODE2.Append(gIndex + i);
				}
			}
			//if AEcoordinates==0 the function node->GetGlobalAECoordinateIndex() might be invalid!
			if (node->GetNumberOfAECoordinates()) //this is for algebraic nodes (not used yet, because Lagrange multipliers are not part of nodes)
			{
				//pout << "SYSTEMINFO: found algebraic coordinates = " << node->GetNumberOfAECoordinates() << "\n";
				//algebraicEquationsInNodes = true; //algebraic coordinates treated with nodes, not with object
				//assert(0 && "CSystem::AssembleObjectLTGLists: Nodes with algebraic constraints not implemented"); //would not work, because it would lead to double
				Index gIndex = node->GetGlobalAECoordinateIndex();
				for (Index i = 0; i < node->GetNumberOfAECoordinates(); i++)
				{
					ltgListAE.Append(gIndex + i);
				}
			}
			//if Datacoordinates==0 the function node->GetGlobalAECoordinateIndex() might be invalid!
			if (node->GetNumberOfDataCoordinates()) //data/history variables - contact, friction, plasticity
			{
				Index gIndex = node->GetGlobalDataCoordinateIndex();
				for (Index i = 0; i < node->GetNumberOfDataCoordinates(); i++)
				{
					ltgListData.Append(gIndex + i);
				}
			}
		}
		//WOULD be an option to add object-constraints(e.g. Euler parameters) //now process algebraic coordinates of objects itself (may not be included in nodes)
		//???
		//if (object->GetAlgebraicEquationsSize() && !algebraicEquationsInNodes)
		//{
		//	ltgListAE.Append(gIndex + i);
		//}
	}
	else if ((Index)object->GetType() & (Index)CObjectType::Connector)
	{
		CObjectConnector* connector = (CObjectConnector*)object;

		//connector may also contain Data nodes
		for (Index j = 0; j < connector->GetNumberOfNodes(); j++)
		{
			CNode* node = connector->GetCNode(j);
			if (node->GetNumberOfDataCoordinates()) //data/history variables - contact, friction, plasticity
			{
				Index gIndex = node->GetGlobalDataCoordinateIndex();
				for (Index i = 0; i < node->GetNumberOfDataCoordinates(); i++)
				{
					ltgListData.Append(gIndex + i);
				}
			}
		}

		//+++++++++++++++++++++++++++++++++++++++
		//process markers --> they have associated coordinates
		for (Index markerNumber : connector->GetMarkerNumbers())
		{
			//pout << "build LTG for " << objectIndex << " (=connector), marker " << markerNumber << "\n";
			CMarker* marker = cSystemData.GetCMarkers()[markerNumber];
			if (marker->GetType() & Marker::Object) //was before::Object
			{
				Index objectNumber = marker->GetObjectNumber();
				const CObject& object = *(cSystemData.GetCObjects()[objectNumber]);

				//pout << "  nNodes=" << object.GetNumberOfNodes() << "\n";

				//object2 can't be a connector, so must have nodes
				for (Index j = 0; j < object.GetNumberOfNodes(); j++)
				{
					CNode* node = object.GetCNode(j);
					//pout << "  node ODE2=" << node->GetNumberOfODE2Coordinates() << "\n";
					if (node->GetNumberOfODE2Coordinates())
					{
						Index gIndex = node->GetGlobalODE2CoordinateIndex();
						for (Index i = 0; i < node->GetNumberOfODE2Coordinates(); i++)
						{
							ltgListODE2.Append(gIndex + i);
						}
					}
					//exclude AE-coordinates, because markers should not act on algebraic coordinates (e.g. rigid body nodes with Euler parameters)
					//if (node->GetNumberOfAECoordinates())
					//{
					//	Index gIndex = node->GetGlobalAECoordinateIndex();
					//	for (Index i = 0; i < node->GetNumberOfAECoordinates(); i++)
					//	{
					//		ltgListAE.Append(gIndex + i);
					//	}
					//}
					if (node->GetNumberOfDataCoordinates())
					{
						Index gIndex = node->GetGlobalDataCoordinateIndex();
						for (Index i = 0; i < node->GetNumberOfDataCoordinates(); i++)
						{
							ltgListData.Append(gIndex + i);
						}
					}
				}
			}
			//was before: else 
			if (marker->GetType() & Marker::Node) //marker can be object + node ==> sliding joing
			{
				Index nodeNumber = marker->GetNodeNumber();
				CNode* node = cSystemData.GetCNodes()[nodeNumber];

				if (node->GetNumberOfODE2Coordinates())
				{
					Index gIndex = node->GetGlobalODE2CoordinateIndex();
					for (Index i = 0; i < node->GetNumberOfODE2Coordinates(); i++)
					{
						ltgListODE2.Append(gIndex + i);
					}
				}
				//exclude AE-coordinates, because markers should not act on algebraic coordinates (e.g. rigid body nodes with Euler parameters)
				//if (node->GetNumberOfAECoordinates())
				//{
				//	Index gIndex = node->GetGlobalAECoordinateIndex();
				//	for (Index i = 0; i < node->GetNumberOfAECoordinates(); i++)
				//	{
				//		ltgListAE.Append(gIndex + i);
				//	}
				//}
				if (node->GetNumberOfDataCoordinates())
				{
					Index gIndex = node->GetGlobalDataCoordinateIndex();
					for (Index i = 0; i < node->GetNumberOfDataCoordinates(); i++)
					{
						ltgListData.Append(gIndex + i);
					}
				}
			}
			else if (!(marker->GetType() & Marker::Node) && !(marker->GetType() & Marker::Object))
			{
				pout << "ERROR: invalid MarkerType: not implemented in CSystem::AssembleLTGLists\n";
			}
		}

		//+++++++++++++++++++++++++++++++++++++++
		//now process algebraic equations of connector (algebraic variables are treated earlier!)
		//this is because LAGRANGE MULTIPLIERS DO NOT REQUIRE NODES
		if ((Index)object->GetType() & (Index)CObjectType::Constraint)
		{
			CObjectConstraint* constraint = (CObjectConstraint*)object;
			//build ltg-list directly from connector

			Index gIndex = constraint->GetGlobalAECoordinateIndex();
			for (Index i = 0; i < constraint->GetAlgebraicEquationsSize(); i++)
			{
				ltgListAE.Append(gIndex + i);
			}
		}
	}
	else
	{
		pout << "ERROR: ObjectType Nr. " << (Index)object->GetType() << " not implemented in CSystem::AssembleLTGLists!\n";
	}
}


//! Use initial values of nodes to compute system-wide initial coordinate vectors
void CSystem::AssembleInitializeSystemCoordinates(const MainSystem& mainSystem)
{
	pout << "Set initial system coordinates (for ODE2, ODE1 and Data coordinates) ...\n";

	//initial system vectors
	Vector ODE2u =  Vector(cSystemData.GetNumberOfCoordinatesODE2());
	Vector ODE2v =  Vector(cSystemData.GetNumberOfCoordinatesODE2());
	Vector ODE1x =  Vector(cSystemData.GetNumberOfCoordinatesODE1());
	Vector data =	Vector(cSystemData.GetNumberOfCoordinatesData());
	Vector AE =		Vector(cSystemData.GetNumberOfCoordinatesAE(), 0.); //algebraic variables initialized with zero!

	Vector ODE2uRef = Vector(cSystemData.GetNumberOfCoordinatesODE2()); //reference coordinates currently only exist for ODE2 coordinates
	//Vector ODE2vRef = Vector(cSystemData.GetNumberOfCoordinatesODE2(), 0.);
	//Vector ODE1xRef = Vector(cSystemData.GetNumberOfCoordinatesODE1(), 0.); //might be useful in future ...?
	//Vector dataRef =  Vector(cSystemData.GetNumberOfCoordinatesData(), 0.);
	//Vector AERef =    Vector(cSystemData.GetNumberOfCoordinatesAE(), 0.);

	const MainSystemData& mainSystemData = mainSystem.mainSystemData;

	//now extract initial coordinate information from nodes:
	Index nodeIndex = 0;
	for (auto mainNode : mainSystemData.GetMainNodes())
	{
		CNode* node = mainNode->GetCNode();
		if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::ODE2variables)
		{
			Index coordIndex = node->GetGlobalODE2CoordinateIndex();

			if (node->GetNumberOfODE2Coordinates())
			{
				Index numberOfCoordinates = node->GetNumberOfODE2Coordinates();

				//link to sublist of global coordinate vector:
				LinkedDataVector u(ODE2u, coordIndex, numberOfCoordinates);
				LinkedDataVector v(ODE2v, coordIndex, numberOfCoordinates);

				u = mainNode->GetInitialVector();	//size must be compatible and is not checked!
				v = mainNode->GetInitialVector_t();	//size must be compatible and is not checked!

				//also initialize global reference coordinate vector (used for differentiation and in finite elements)
				LinkedDataVector uRef(ODE2uRef, coordIndex, numberOfCoordinates);
				uRef = node->GetReferenceCoordinateVector();
			}
		}
		else if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::ODE1variables)
		{
			Index coordIndex = node->GetGlobalODE1CoordinateIndex();

			if (node->GetNumberOfODE1Coordinates())
			{
				Index numberOfCoordinates = node->GetNumberOfODE1Coordinates();
				LinkedDataVector x(ODE1x, coordIndex, numberOfCoordinates);

				x = mainNode->GetInitialVector(); //size must be compatible and is not checked!
			}
		}
		else if ((Index)node->GetNodeGroup() & (Index)CNodeGroup::DataVariables)
		{
			Index coordIndex = node->GetGlobalDataCoordinateIndex();

			if (node->GetNumberOfDataCoordinates())
			{
				Index numberOfCoordinates = node->GetNumberOfDataCoordinates();
				LinkedDataVector x(data, coordIndex, numberOfCoordinates);

				x = mainNode->GetInitialVector(); //size must be compatible and is not checked!
			}
		}
		else { CHECKandTHROWstring("CSystem::AssembleInitializeSystemCoordinates: invalid Node type, not implemented"); }
		nodeIndex++;
	}

	cSystemData.GetCData().initialState.ODE2Coords = ODE2u;
	cSystemData.GetCData().initialState.ODE2Coords_t = ODE2v;
	cSystemData.GetCData().initialState.ODE1Coords = ODE1x;
	cSystemData.GetCData().initialState.dataCoords = data;
	cSystemData.GetCData().initialState.AECoords = AE;

	cSystemData.GetCData().referenceState.ODE2Coords = ODE2uRef;
	//cSystemData.GetCData().referenceState.ODE2Coords_t = ODE2vRef; //access to these coordinates will lead to crash
	//cSystemData.GetCData().referenceState.ODE1Coords = ODE1xRef;
	//cSystemData.GetCData().referenceState.dataCoords = dataRef;
	//cSystemData.GetCData().referenceState.AECoords = AERef; 



	//initial values are also used for current step ==> from here on, the system can be visualized!
	cSystemData.GetCData().currentState = cSystemData.GetCData().initialState;
	cSystemData.GetCData().visualizationState = cSystemData.GetCData().initialState; //from this point on, drawing should be possible
	//done at beginning of solver/time integration: cSystemData.GetCData().startOfStepState = cSystemData.GetCData().initialState;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// CSystem computation functions
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! compute system massmatrix; massmatrix must have according size
void CSystem::ComputeMassMatrix(TemporaryComputationData& temp, GeneralMatrix& massMatrix)
{
	//size needs to be set accordingly in the caller function; components are addd to massMatrix!

	for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
	{
		//work over bodies, connectors, etc.
		CObject& object = *(cSystemData.GetCObjects()[j]);

		//if object is a body, it must have a mass matrix
		if ((Index)object.GetType() & (Index)CObjectType::Body)
		{
			ArrayIndex& ltg = cSystemData.GetLocalToGlobalODE2()[j];
			if (ltg.NumberOfItems() != 0) //to exclude bodies attached to ground nodes
			{
				((CObjectBody&)object).ComputeMassMatrix(temp.localMass);

				massMatrix.AddSubmatrix(temp.localMass, 1., ltg, ltg);
			}
		}
	}

}


Index TScomputeODE2RHSobject;
TimerStructureRegistrator TSRcomputeODE2RHSobject("computeODE2RHSobject", TScomputeODE2RHSobject, globalTimers);
Index TScomputeODE2RHSconnector;
TimerStructureRegistrator TSRcomputeODE2RHSconnector("computeODE2RHSconnector", TScomputeODE2RHSconnector, globalTimers);
Index TScomputeODE2RHSmarkerData;
TimerStructureRegistrator TSRcomputeODE2RHSmarkerData("computeODE2RHSmarkerData", TScomputeODE2RHSmarkerData, globalTimers);
Index TScomputeLoads;
TimerStructureRegistrator TSRcomputeLoads("computeLoads", TScomputeLoads, globalTimers);

//! compute right-hand-side (RHS) of second order ordinary differential equations (ODE) for every object (used in numerical differentiation and in RHS computation)
//! return true, if object has localODE2Rhs, false otherwise
bool CSystem::ComputeObjectODE2RHS(TemporaryComputationData& temp, CObject* object, Vector& localODE2Rhs)
{
	Index i = 0;
	//std::cout << "ComputeODE2RHS" << i++ << ": " << (Index)object->GetType() << "\n";
	if (!((Index)object->GetType() & (Index)CObjectType::Constraint)) //only if ODE2 exists and if not constraint (Constraint force action added in solver)
	{
		//if object is a body, it must have ODE2RHS
		if ((Index)object->GetType() & (Index)CObjectType::Body)
		{
			STARTGLOBALTIMER(TScomputeODE2RHSobject);
			object->ComputeODE2RHS(localODE2Rhs);
			STOPGLOBALTIMER(TScomputeODE2RHSobject);
			//pout << "temp.localODE2RHS=" << temp.localODE2RHS << "\n";
		}
		else if ((Index)object->GetType() & (Index)CObjectType::Connector)
		{
			CObjectConnector* connector = (CObjectConnector*)object;

			//compute MarkerData for connector:
			const bool computeJacobian = true;
			STARTGLOBALTIMER(TScomputeODE2RHSmarkerData);
			ComputeMarkerDataStructure(connector, computeJacobian, temp.markerDataStructure);
			STOPGLOBALTIMER(TScomputeODE2RHSmarkerData);

			//pout << "ComputeODE2RHS " << i++ << "\n";
			//Real t = cSystemData.GetCData().currentState.time; //==>done in markerdatastructure
			STARTGLOBALTIMER(TScomputeODE2RHSconnector);
			connector->ComputeODE2RHS(localODE2Rhs, temp.markerDataStructure);
			STOPGLOBALTIMER(TScomputeODE2RHSconnector);

		}
		else { CHECKandTHROWstring("CSystem::ComputeODE2RHS(...): object type not implemented"); return false; }

		return true;
	}
	return false;
}

//! compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to 'ode2rhs' for ODE2 part
void CSystem::ComputeODE2RHS(TemporaryComputationData& temp, Vector& ode2Rhs)
{
	ode2Rhs.SetAll(0.);

	for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
	{
		if ((cSystemData.GetCObjects()[j])->IsActive())
		{
			//work over bodies, connectors, etc.
			ArrayIndex& ltgODE2 = cSystemData.GetLocalToGlobalODE2()[j];

			if (ltgODE2.NumberOfItems() && ComputeObjectODE2RHS(temp, cSystemData.GetCObjects()[j], temp.localODE2RHS))//temp.localODE2RHS))
			{
				//now add RHS to system vector
				for (Index k = 0; k < temp.localODE2RHS.NumberOfItems(); k++)
				{
					ode2Rhs[ltgODE2[k]] -= temp.localODE2RHS[k]; //negative sign ==> stiffness/damping on LHS of equations
				}
			}
		}
	}
	//pout << "ode2Rhs=" << ode2Rhs << "\n";

	ComputeLoads(temp, ode2Rhs);
}

//! compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to 'ode2rhs' for ODE2 part
void CSystem::ComputeLoads(TemporaryComputationData& temp, Vector& ode2Rhs)
{
	//++++++++++++++++++++++++++++++++++++++++++++++++++
	//compute loads ==> not needed in jacobian, except for follower loads, 
	//  using e.g. local body coordinate system

	STARTGLOBALTIMER(TScomputeLoads);

	Index nLoads = cSystemData.GetCLoads().NumberOfItems();
	Vector3D loadVector3D;
	Vector1D loadVector1D; //scalar loads...
	Real currentTime = cSystemData.GetCData().currentState.time;
	for (Index j = 0; j < nLoads; j++)
	{
		if (cSystemData.GetCLoads()[j]->IsVector()) { loadVector3D = cSystemData.GetCLoads()[j]->GetLoadVector(currentTime); }
		else { loadVector1D = Vector1D(cSystemData.GetCLoads()[j]->GetLoadValue(currentTime)); }

		Index markerNumber = cSystemData.GetCLoads()[j]->GetMarkerNumber();
		CMarker* marker = cSystemData.GetCMarkers()[markerNumber];
		LoadType loadType = cSystemData.GetCLoads()[j]->GetType();

		ArrayIndex* ltg = nullptr;		//for objects
		Index nodeCoordinate;	//starting index for nodes (consecutively numbered)
		bool applyLoad = false; //loads are not applied to ground objects/nodes

		//loads only applied to Marker::Body or Marker::Node
		if (marker->GetType() & Marker::Body) //code for body markers
		{
			Index markerBodyNumber = marker->GetObjectNumber();
			if (!((Index)cSystemData.GetCObjectBody(markerBodyNumber).GetType() & (Index)CObjectType::Ground)) //no action on ground objects!
			{
				ltg = &cSystemData.GetLocalToGlobalODE2()[markerBodyNumber];
				if (ltg->NumberOfItems() != 0) { applyLoad = true; } //only apply load, if object is not attached to ground node!
			}
		}
		else if (marker->GetType() & Marker::Node) //code for body markers
		{
			Index markerNodeNumber = marker->GetNodeNumber();
			if (!cSystemData.GetCNodes()[markerNodeNumber]->IsGroundNode()) //if node has zero coordinates ==> ground node; no action on ground nodes!
			{
				if ((marker->GetType() & Marker::Position) || (marker->GetType() & Marker::Coordinate))
				{
					nodeCoordinate = cSystemData.GetCNodes()[markerNodeNumber]->GetGlobalODE2CoordinateIndex();
					applyLoad = true;
				}
				else
				{
					CHECKandTHROWstring("ERROR: CSystem::ComputeODE2RHS, marker type not implemented!");
				}
			}
		}
		else { pout << "ERROR: CSystem::ComputeODE2RHS: marker must be Body or Node type\n"; }

		if (applyLoad)
		{
			//AccessFunctionType aft = GetAccessFunctionType(loadType, marker->GetType());
			//==> lateron: depending on AccessFunctionType compute jacobians, put into markerDataStructure as in connectors
			//    and call according jacobian function
			//    marker->GetAccessFunctionJacobian(AccessFunctionType, ...) ==> handles automatically the jacobian

			//bodyFixed (local) follower loads:
			bool bodyFixed = false;
			if (cSystemData.GetCLoads()[j]->IsBodyFixed())
			{
				bodyFixed = true;
			}

			if (loadType == LoadType::Force || loadType == LoadType::ForcePerMass)
			{
				const bool computeJacobian = true;
				marker->ComputeMarkerData(cSystemData, computeJacobian, temp.markerDataStructure.GetMarkerData(0)); //currently, too much is computed; but could be pre-processed in parallel
				if (bodyFixed) { loadVector3D = temp.markerDataStructure.GetMarkerData(0).orientation * loadVector3D; }
				EXUmath::MultMatrixTransposedVector(temp.markerDataStructure.GetMarkerData(0).positionJacobian, loadVector3D, temp.generalizedLoad); //generalized load: Q = (dPos/dq)^T * Force

				//marker->GetPositionJacobian(cSystemData, temp.loadJacobian);
				//EXUmath::MultMatrixVector(temp.loadJacobian, loadVector3D, temp.generalizedLoad);
			}
			else if (loadType == LoadType::Torque)
			{
				const bool computeJacobian = true;
				marker->ComputeMarkerData(cSystemData, computeJacobian, temp.markerDataStructure.GetMarkerData(0)); //currently, too much is computed; but could be pre-processed in parallel
				if (bodyFixed) { loadVector3D = temp.markerDataStructure.GetMarkerData(0).orientation * loadVector3D; }
				EXUmath::MultMatrixTransposedVector(temp.markerDataStructure.GetMarkerData(0).rotationJacobian, loadVector3D, temp.generalizedLoad); //generalized load: Q = (dRot/dq)^T * Torque
				//pout << "rotationJacobian=" << temp.markerDataStructure.GetMarkerData(0).rotationJacobian << "\n";
				//pout << "loadVector3D=" << loadVector3D << "\n";
			}
			else if (loadType == LoadType::Coordinate)
			{
				const bool computeJacobian = true;
				marker->ComputeMarkerData(cSystemData, computeJacobian, temp.markerDataStructure.GetMarkerData(0)); //currently, too much is computed; but could be pre-processed in parallel
				EXUmath::MultMatrixTransposedVector(temp.markerDataStructure.GetMarkerData(0).jacobian, loadVector1D, temp.generalizedLoad); //generalized load: Q = (dRot/dq)^T * Torque
				//pout << "jacobian=" << temp.markerDataStructure.GetMarkerData(0).jacobian << "\n";
				//pout << "generalizedLoad=" << temp.generalizedLoad << "\n";
				//pout << "loadVector1D=" << loadVector1D << "\n";
			}
			else { CHECKandTHROWstring("ERROR: CSystem::ComputeODE2RHS, LoadType not implemented!"); }

			//ResizableArray<CObject*>& objectList = cSystemData.GetCObjects();
			//pout << "genLoad=" << temp.generalizedLoad << "\n";

			if (ltg != nullptr) //must be object
			{

				//pout << "load=" << temp.generalizedLoad << ", LF=" << solverData.loadFactor << ", rotJac=" << temp.markerDataStructure.GetMarkerData(0).positionJacobian << "\n";
				for (Index k = 0; k < temp.generalizedLoad.NumberOfItems(); k++)
				{
					ode2Rhs[(*ltg)[k]] += solverData.loadFactor * temp.generalizedLoad[k]; //if the loadfactor shall not be used for static case: add LoadRampType structure to define behavior: StaticRampDynamicStep=0, StaticStep=1, DynamicRamp=2, ...
				}
			}
			else //must be node
			{
				//pout << "  nodeCoordinate=" << nodeCoordinate << "\n";
				for (Index k = 0; k < temp.generalizedLoad.NumberOfItems(); k++)
				{
					ode2Rhs[nodeCoordinate + k] += solverData.loadFactor * temp.generalizedLoad[k];
				}

			}
		}

	}
	STOPGLOBALTIMER(TScomputeLoads);
}

//! compute right-hand-side (RHS) of algebraic equations (AE) to vector 'AERhs'
void CSystem::ComputeAlgebraicEquations(TemporaryComputationData& temp, Vector& algebraicEquations, bool velocityLevel)
{
	//Still needed? algebraicEquations.SetNumberOfItems(cSystemData.GetNumberOfCoordinatesAE()); //needed for numerical differentiation
	algebraicEquations.SetAll(0.);

	//algebraic equations only origin from objects (e.g. Euler parameters) and constraints
	for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
	{
		//work over bodies, connectors, etc.
		CObject& object = *(cSystemData.GetCObjects()[j]);
		ArrayIndex& ltg = cSystemData.GetLocalToGlobalAE()[j];

		if (ltg.NumberOfItems())
		{
			//for body, evaluate algebraic equations directly --> depend only on body coordinates
			if ((Index)object.GetType() & (Index)CObjectType::Body)
			{
				if (object.GetAlgebraicEquationsSize()) //either body or constraint
				{
					object.ComputeAlgebraicEquations(temp.localAE, velocityLevel); //no time given for objects for now (only Euler parameters...)
				}

			}
			//for constraint, algebraic equations depend on Markers
			else if ((Index)object.GetType() & (Index)CObjectType::Constraint)
			{
				CObjectConstraint& constraint = (CObjectConstraint&)object;

				//const ArrayIndex& markerNumbers = constraint.GetMarkerNumbers();
				//Index nMarkers = constraint.GetMarkerNumbers().NumberOfItems();
				//if (nMarkers != 2) { CHECKandTHROWstring("SolveSystem(...): Number of constraint markers != 2 not implemented"); }

				//for (Index k = 0; k < 2; k++)
				//{
				//	//lateron: use accessFunctionType to determine necessary computations!

				//	const bool computeJacobian = false;
				//	cSystemData.GetCMarkers()[markerNumbers[k]]->ComputeMarkerData(cSystemData, computeJacobian, temp.markerDataStructure.GetMarkerData(k));
				//}

				const bool computeJacobian = false;
				ComputeMarkerDataStructure(&constraint, computeJacobian, temp.markerDataStructure);

				constraint.ComputeAlgebraicEquations(temp.localAE, temp.markerDataStructure, cSystemData.GetCData().currentState.time, velocityLevel);

			}
			//for connectors, linked to objects that contain algebraic variables (e.g. Euler-Parameter based rigid bodies)
			else if ((Index)object.GetType() & (Index)CObjectType::Connector)
			{
				CHECKandTHROW(object.GetAlgebraicEquationsSize() == 0, "CSystem::ComputeAlgebraicEquations: ltg size mismatch");
				temp.localAE.SetNumberOfItems(0); //no algebraic equations are processed
			}
			else { CHECKandTHROWstring("CSystem::ComputeAlgebraicEquations(...): object type not implemented"); }

			CHECKandTHROW(ltg.NumberOfItems() == temp.localAE.NumberOfItems(), "CSystem::ComputeAlgebraicEquations: ltg size mismatch");
			//now add RHS to system vector
			for (Index k = 0; k < temp.localAE.NumberOfItems(); k++)
			{
				algebraicEquations[ltg[k]] += temp.localAE[k]; //negative sign ==> check sign of Lagrange multipliers
			}
		}
	}


}

//void CSystem::ComputeMarkerDataStructure(const CObjectConnector* connector, bool computeJacobian, MarkerDataStructure& markerDataStructure) const
//{
//	const ArrayIndex& markerNumbers = connector->GetMarkerNumbers();
//	Index nMarkers = connector->GetMarkerNumbers().NumberOfItems();
//	if (nMarkers != 2) { CHECKandTHROWstring("CSystem::ComputeMarkerDataStructure(...): Number of connector markers != 2 not implemented"); }
//	markerDataStructure.SetTime(cSystemData.GetCData().currentState.GetTime());
//
//	if ((Index)connector->GetType() & (Index)CObjectType::Constraint)
//	{
//		const CObjectConstraint* constraint = (CObjectConstraint*)connector;
//		Index AEindex = constraint->GetGlobalAECoordinateIndex();
//		Index nAEcoords = constraint->GetAlgebraicEquationsSize();
//		markerDataStructure.GetLagrangeMultipliers().LinkDataTo(cSystemData.GetCData().currentState.AECoords, AEindex, nAEcoords);
//	}
//	for (Index k = 0; k < 2; k++)
//	{
//		cSystemData.GetCMarkers()[markerNumbers[k]]->ComputeMarkerData(cSystemData, computeJacobian, markerDataStructure.GetMarkerData(k));
//	}
//}

//markerdata computed in CSystemData because needed for sensors
void CSystemData::ComputeMarkerDataStructure(const CObjectConnector* connector, bool computeJacobian, MarkerDataStructure& markerDataStructure) const
{
	const ArrayIndex& markerNumbers = connector->GetMarkerNumbers();
	Index nMarkers = connector->GetMarkerNumbers().NumberOfItems();
	if (nMarkers != 2) { CHECKandTHROWstring("CSystemData::ComputeMarkerDataStructure(...): Number of connector markers != 2 not implemented"); }
	markerDataStructure.SetTime(GetCData().currentState.GetTime());

	if ((Index)connector->GetType() & (Index)CObjectType::Constraint)
	{
		const CObjectConstraint* constraint = (CObjectConstraint*)connector;
		Index AEindex = constraint->GetGlobalAECoordinateIndex();
		Index nAEcoords = constraint->GetAlgebraicEquationsSize();
		markerDataStructure.GetLagrangeMultipliers().LinkDataTo(GetCData().currentState.AECoords, AEindex, nAEcoords);
	}
	for (Index k = 0; k < 2; k++)
	{
		GetCMarkers()[markerNumbers[k]]->ComputeMarkerData(*this, computeJacobian, markerDataStructure.GetMarkerData(k));
	}
}

//! PostNewtonStep: do this for every object (connector), which has a PostNewtonStep ->discontinuous iteration e.g. to resolve contact, friction or plasticity
Real CSystem::PostNewtonStep(TemporaryComputationData& temp)
{
	Real PNerror = 0;
	PostNewtonFlags::Type postNewtonFlags;
	//algebraic equations only origin from objects (e.g. Euler parameters) and constraints

	for (Index objectIndex = 0; objectIndex < cSystemData.GetCObjects().NumberOfItems(); objectIndex++)
	{
		CObject* object = cSystemData.GetCObjects()[objectIndex];

		//for constraint, algebraic equations depend on Markers
		if ((Index)object->GetType() & (Index)CObjectType::Connector)
		{
			CObjectConnector* connector = (CObjectConnector*)object;

			if (connector->HasDiscontinuousIteration())
			{

				const bool computeJacobian = true; //why needed for PostNewtonStep?==> check Issue #241
				ComputeMarkerDataStructure(connector, computeJacobian, temp.markerDataStructure);

				PNerror = EXUstd::Maximum(connector->PostNewtonStep(temp.markerDataStructure, postNewtonFlags), PNerror);

				if (postNewtonFlags&PostNewtonFlags::UpdateLTGLists)
				{
					//now update specific ltg lists, if e.g. due to contact or switching the connectivity has changed
					AssembleObjectLTGLists(objectIndex, cSystemData.GetLocalToGlobalODE2()[objectIndex], cSystemData.GetLocalToGlobalODE1()[objectIndex],
						cSystemData.GetLocalToGlobalAE()[objectIndex], cSystemData.GetLocalToGlobalData()[objectIndex]);

					//pout << "Connector" << objectIndex << ", new LTGlist=" << cSystemData.GetLocalToGlobalODE2()[objectIndex] << "\n";
				}
			}
		}
	}
	return PNerror;
}

//! function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
void CSystem::PostDiscontinuousIterationStep()
{
	//algebraic equations only origin from objects (e.g. Euler parameters) and constraints
	for (CObject* object: cSystemData.GetCObjects())
	{
		//for constraint, algebraic equations depend on Markers
		if ((Index)object->GetType() & (Index)CObjectType::Connector)
		{
			CObjectConnector* connector = (CObjectConnector*)object;
			if (connector->HasDiscontinuousIteration()) { connector->PostDiscontinuousIterationStep(); }
		}
	}
}







//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                          JACOBIANS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! compute numerical differentiation of ODE2RHS; result is a jacobian;  multiply the added entries with scalarFactor
//template<class TGeneralMatrix>
void CSystem::NumericalJacobianODE2RHS(TemporaryComputationData& temp, const NumericalDifferentiationSettings& numDiff,
	Vector& f0, Vector& f1, GeneralMatrix& jacobianGM, Real scalarFactor) // , ResizableMatrix& jacobian)
{
	//size needs to be set accordingly in the caller function; components are addd to massMatrix!

	cSystemData.isODE2RHSjacobianComputation = true; //hack, use rarely this flag or only for debug!
	
	Real relEps = numDiff.relativeEpsilon;			//relative differentiation parameter
	Real minCoord = numDiff.minimumCoordinateSize;	//absolute differentiation parameter is limited to this minimum
	Real eps, epsInv; //coordinate(column)-wise differentiation parameter; depends on size of coordinate

	Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
	Vector& x = cSystemData.GetCData().currentState.ODE2Coords;			//current coordinates ==> this is what is differentiated for
	Vector& xRef = cSystemData.GetCData().referenceState.ODE2Coords;	//reference coordinates; might be important for numerical differentiation
	Real xStore; //store value of x; avoid roundoff error effects in numerical differentiation

	if (!numDiff.doSystemWideDifferentiation)
	{
		//GeneralMatrixEXUdense mat;
		//if (jacobianGM.GetSystemMatrixType() != LinearSolverType::EXUdense) { CHECKandTHROWstring("CSystem::NumericalJacobianODE2RHS: illegal LinearSolverType!"); }
		//ResizableMatrix& jacobian = jacobianGM.GetMatrixEXUdense();
		ResizableMatrix& localJacobian = temp.localJacobian;

		//size already set by solver: jacobian.SetNumberOfRowsAndColumns(nODE2, nODE2);
		//NEW: set whole matrix zero
		//jacobianGM.SetAllZero(); //now done outside

		//this only sets a sub-part to zero:
		//for (Index i = 0; i < nODE2; i++) {//SetAll(0.) does not work, because jacobian matrix could be larger
		//	for (Index j = 0; j < nODE2; j++) { jacobian(i, j) = 0.; } }

		//++++++++++++++++++++++++++++++++++++++++++++++++

		for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
		{
			ArrayIndex& ltgODE2 = cSystemData.GetLocalToGlobalODE2()[j];
			Index nLocalODE2 = ltgODE2.NumberOfItems();
			f0.SetNumberOfItems(nLocalODE2);
			f1.SetNumberOfItems(nLocalODE2);
			CObject* object = cSystemData.GetCObjects()[j];

			if (object->IsActive() && ltgODE2.NumberOfItems())
			{
#ifdef USE_AUTODIFF
				if (object->GetAvailableJacobians() & (JacobianType::ODE2_ODE2_function))
				{
					localJacobian.SetNumberOfRowsAndColumns(nLocalODE2, nLocalODE2); //needs not to be initialized, because the matrix is fully computed and then added to jacobianGM
					temp.localJacobian_t.SetNumberOfRowsAndColumns(nLocalODE2, nLocalODE2); //needs not to be initialized, because the matrix is fully computed and then added to jacobianGM
					object->ComputeJacobianODE2_ODE2(localJacobian, temp.localJacobian_t);
					jacobianGM.AddSubmatrix(localJacobian, -1., ltgODE2, ltgODE2); //-1. because in numerical mode, f0-f1 leads to negative sign (RHS ==> LHS)
				}
				else 
#endif
				if (ComputeObjectODE2RHS(temp, object, f0)) //check if it is a constraint, etc. which is not differentiated for ODE2 jacobian
				{
					localJacobian.SetNumberOfRowsAndColumns(nLocalODE2, nLocalODE2); //needs not to be initialized, because the matrix is fully computed and then added to jacobianGM
					Real xRefVal = 0;
					for (Index i = 0; i < nLocalODE2; i++) //differentiate w.r.t. every ltgODE2 coordinate
					{
						Real& xVal = x[ltgODE2[i]];
						if (numDiff.addReferenceCoordinatesToEpsilon) { xRefVal = xRef[ltgODE2[i]]; }

						eps = relEps * (EXUstd::Maximum(minCoord, fabs(xVal + xRefVal)));

						xStore = xVal;
						xVal += eps;
						ComputeObjectODE2RHS(temp, object, f1);
						xVal = xStore;

						epsInv = (1. / eps) * scalarFactor;

						for (Index k = 0; k < nLocalODE2; k++)
						{
							//use local jacobian:
							localJacobian(k, i) = epsInv * (f0[k] - f1[k]); //-(f1-f0) == (f0-f1): negative sign, because object ODE2RHS is subtracted from global RHS-vector
						}
					}
					jacobianGM.AddSubmatrix(localJacobian, 1., ltgODE2, ltgODE2);
				}
			}
		}
	}
	else
	{
		//done in solver: jacobian.SetNumberOfRowsAndColumns(nODE2, nODE2);

		//++++++++++++++++++++++++++++++++++++++++++++++++
		f0.SetNumberOfItems(nODE2);
		f1.SetNumberOfItems(nODE2);
		ComputeODE2RHS(temp, f0); //compute nominal value for jacobian
		Real xRefVal = 0;

		for (Index i = 0; i < nODE2; i++) //compute column i
		{
			if (numDiff.addReferenceCoordinatesToEpsilon) { xRefVal = xRef[i]; }
			eps = relEps * (EXUstd::Maximum(minCoord, fabs(x[i]+xRefVal)));

			xStore = x[i];
			x[i] += eps;
			ComputeODE2RHS(temp, f1);
			x[i] = xStore;

			epsInv = (1. / eps) * scalarFactor;

			f1 -= f0;
			f1 *= epsInv;
			jacobianGM.AddColumnVector(i, f1);
		}
	}
	//pout << "ODE2jac=" << jacobian << "\n";
	cSystemData.isODE2RHSjacobianComputation = false; //hack! only for debugging
}

//! compute numerical differentiation of ODE2RHS; result is a jacobian;  multiply the added entries with scalarFactor
void CSystem::NumericalJacobianODE2RHS_t(TemporaryComputationData& temp, const NumericalDifferentiationSettings& numDiff,
	Vector& f0, Vector& f1, GeneralMatrix& jacobianGM, Real scalarFactor)
{
	//size needs to be set accordingly in the caller function; components are addd to massMatrix!

	Real relEps = numDiff.relativeEpsilon;			//relative differentiation parameter
	Real minCoord = numDiff.minimumCoordinateSize;	//absolute differentiation parameter is limited to this minimum
	Real eps, epsInv;								//coordinate(column)-wise differentiation parameter; depends on size of coordinate
	Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
	Vector& x = cSystemData.GetCData().currentState.ODE2Coords_t;
	Real xStore; //store value of x; avoid roundoff error effects in numerical differentiation

	if (!numDiff.doSystemWideDifferentiation)
	{
		//size already set by solver: jacobian.SetNumberOfRowsAndColumns(nODE2, nODE2);

		ResizableMatrix& localJacobian_t = temp.localJacobian_t;

		//size already set by solver: jacobian.SetNumberOfRowsAndColumns(nODE2, nODE2);
		//jacobianGM.SetAllZero(); //now done in caller function

		for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
		{
			ArrayIndex& ltgODE2 = cSystemData.GetLocalToGlobalODE2()[j];
			Index nLocalODE2 = ltgODE2.NumberOfItems();
			f0.SetNumberOfItems(nLocalODE2);
			f1.SetNumberOfItems(nLocalODE2);
			CObject* object = cSystemData.GetCObjects()[j];

			if (object->IsActive() && ltgODE2.NumberOfItems())
			{
#ifdef USE_AUTODIFF
				if (object->GetAvailableJacobians() & (JacobianType::ODE2_ODE2_t_function))
				{
					//temp.localJacobian.SetNumberOfRowsAndColumns(nLocalODE2, nLocalODE2); //needs not to be initialized, because the matrix is fully computed and then added to jacobianGM
					//localJacobian_t.SetNumberOfRowsAndColumns(nLocalODE2, nLocalODE2); //needs not to be initialized, because the matrix is fully computed and then added to jacobianGM
					//object->ComputeJacobianODE2_ODE2(temp.localJacobian, localJacobian_t);
					//jacobianGM.AddSubmatrix(localJacobian_t, -1., ltgODE2, ltgODE2);//-1. because in numerical mode, f0-f1 leads to negative sign (RHS ==> LHS)
				}
				else
#endif
				if (ComputeObjectODE2RHS(temp, object, f0)) //check if it is a constraint, etc. which is not differentiated for ODE2 jacobian
				{
					localJacobian_t.SetNumberOfRowsAndColumns(nLocalODE2, nLocalODE2); //needs not to be initialized, because the matrix is fully computed and then added to jacobianGM
					for (Index i = 0; i < nLocalODE2; i++) //differentiate w.r.t. every ltgODE2 coordinate
					{
						Real& xVal = x[ltgODE2[i]];
						eps = relEps * (EXUstd::Maximum(minCoord, fabs(xVal)));

						xStore = xVal;
						xVal += eps;
						ComputeObjectODE2RHS(temp, object, f1);
						xVal = xStore;

						epsInv = (1. / eps) * scalarFactor;

						for (Index k = 0; k < nLocalODE2; k++)
						{
							//use local jacobian:
							localJacobian_t(k, i) = epsInv * (f0[k] - f1[k]); //-(f1-f0) == (f0-f1): negative sign, because object ODE2RHS is subtracted from global RHS-vector
						}
					}
					jacobianGM.AddSubmatrix(localJacobian_t, 1., ltgODE2, ltgODE2);
				}
			}
		}

	}
	else
	{
		//jacobianGM.SetAllZero(); //now done outside

		f0.SetNumberOfItems(nODE2);
		f1.SetNumberOfItems(nODE2);
		ComputeODE2RHS(temp, f0); //compute nominal value for jacobian


		for (Index i = 0; i < nODE2; i++)
		{
			eps = relEps * (EXUstd::Maximum(minCoord, fabs(x[i])));

			xStore = x[i];
			x[i] += eps;
			ComputeODE2RHS(temp, f1);
			x[i] = xStore;

			epsInv = (1. / eps) * scalarFactor;

			//for (Index j = 0; j < nODE2; j++)
			//{
			//	jacobian(j, i) = epsInv * (f1[j] - f0[j]);
			//}
			f1 -= f0;
			f1 *= epsInv;
			jacobianGM.AddColumnVector(i, f1);
		}
	}
	//pout << "ODE2jac_t=" << jacobian << "\n";
}

//! numerical computation of constraint jacobian with respect to ODE2 and ODE1 (fillIntoSystemMatrix=true: also w.r.t. AE) coordinates
//! factorODE2 is used to scale the ODE2-part of the jacobian (to avoid postmultiplication); 
//! velocityLevel = velocityLevel constraints are used, if available; 
//! fillIntoSystemMatrix=true: fill in g_q_ODE2, g_q_ODE2^T AND g_q_AE into system matrix at according positions
//! fillIntoSystemMatrix=false: fill in g_q_ODE2 into jacobian matrix at (0,0)
template<class TGeneralMatrix>
void CSystem::NumericalJacobianAE(TemporaryComputationData& temp, const NumericalDifferentiationSettings& numDiff,
	Vector& f0, Vector& f1, TGeneralMatrix& jacobianGM, Real factorAE_ODE2, Real factorAE_ODE2_t,
	bool velocityLevel, bool fillIntoSystemMatrix)
{
	Real relEps = numDiff.relativeEpsilon;			//relative differentiation parameter
	Real minCoord = numDiff.minimumCoordinateSize;	//absolute differentiation parameter is limited to this minimum
	Real eps, epsInv; //coordinate(column)-wise differentiation parameter; depends on size of coordinate

	Index nAE = cSystemData.GetNumberOfCoordinatesAE();
	Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
	Vector& x = cSystemData.GetCData().currentState.ODE2Coords;
	Vector& x_t = cSystemData.GetCData().currentState.ODE2Coords_t; //velocity coordinates
	Real xStore; //store value of x; avoid roundoff error effects in numerical differentiation

	Vector& z = cSystemData.GetCData().currentState.AECoords;
	Real zStore; //store value of x; avoid roundoff error effects in numerical differentiation

	//++++++++++++++++++++++++++++++++++++++++++++++++
	//compute total jacobian ==> very time consuming ==> change this to local jacobian (use flag in numDiffParameters?)

	if (jacobianGM.GetSystemMatrixType() != LinearSolverType::EXUdense) { CHECKandTHROWstring("CSystem::NumericalJacobianAE: illegal LinearSolverType!"); }
	ResizableMatrix& jacobian = ((GeneralMatrixEXUdense&)jacobianGM).GetMatrixEXUdense();

	f0.SetNumberOfItems(nAE);
	f1.SetNumberOfItems(nAE);

	ComputeAlgebraicEquations(temp, f0, velocityLevel); //compute nominal value for jacobian

	//differentiation w.r.t. ODE2 coordinates
	for (Index i = 0; i < nODE2; i++)
	{
		eps = relEps * (EXUstd::Maximum(minCoord, fabs(x[i])));
		//pout << "Jac i=" << i << ", diffEps = " << eps << "\n";

		xStore = x[i];
		x[i] += eps;
		ComputeAlgebraicEquations(temp, f1, velocityLevel);
		x[i] = xStore;

		epsInv = 1. / eps;

		for (Index j = 0; j < nAE; j++)
		{
			Real x = epsInv * (f1[j] - f0[j]);
			if (fillIntoSystemMatrix)
			{
				jacobian(nODE2 + j, i) = factorAE_ODE2 * x; //add Cq ==> factor only used for Position constraints ...
				jacobian(i, nODE2 + j) = /*factorAE_ODE2 * */ x; //add CqT
			}
			else
			{
				jacobian(j, i) = /*factorAE_ODE2 * */ x; //add Cq which is lateron used as transposed matrix; factor is usually 1 in this case
			}
		}
	}

	//differentiation w.r.t. ODE2_t coordinates
	for (Index i = 0; i < nODE2; i++)
	{
		eps = relEps * (EXUstd::Maximum(minCoord, fabs(x[i])));
		//pout << "Jac i=" << i << ", diffEps = " << eps << "\n";

		xStore = x_t[i];
		x_t[i] += eps;
		ComputeAlgebraicEquations(temp, f1, velocityLevel);
		x_t[i] = xStore;

		epsInv = 1. / eps;

		for (Index j = 0; j < nAE; j++)
		{
			Real x = epsInv * (f1[j] - f0[j]);
			if (fillIntoSystemMatrix)
			{
				jacobian(nODE2 + j, i) += factorAE_ODE2_t * x; //add Cq ==> factor only used for Position constraints ...
				jacobian(i, nODE2 + j) += x; //add CqT; this term MUST be added for purly velocity-formulated constraints (e.g. velocity coordinate constraint, rolling joint, ...)
			}
			else
			{
				jacobian(j, i) += x; //add Cq which is lateron used as transposed matrix; this term MUST be added for purly velocity-formulated constraints (e.g. velocity coordinate constraint, rolling joint, ...)
			}
		}
	}

	//differentiation w.r.t. AE coordinates
	if (fillIntoSystemMatrix) //also add AE_AE terms
	{
		for (Index i = 0; i < nAE; i++)
		{
			eps = relEps * (EXUstd::Maximum(minCoord, fabs(z[i])));

			zStore = z[i];
			z[i] += eps;
			ComputeAlgebraicEquations(temp, f1, velocityLevel);
			z[i] = zStore;
			epsInv = 1. / eps;

			for (Index j = 0; j < nAE; j++)
			{
				jacobian(nODE2 + j, nODE2 + i) = epsInv * (f1[j] - f0[j]);
			}
		}
	}
}

//the NumericalJacobianAE ... _t  is never used
////! compute numerical differentiation of AE with respect to ODE2 velocity; factor is used to scale the Jacobian; if fillIntoSystemMatrix==true, the jacobian is filled directly into the system matrix; result is a jacobian; THIS FUNCTION IS ONLY FOR COMPARISON (SLOW!!!)
//void CSystem::NumericalJacobianAE_ODE2_t(const NumericalDifferentiation& numDiff,
//	TemporaryComputationData& temp, Vector& f0, Vector& f1, ResizableMatrix& jacobian, Real factor, bool velocityLevel)
//{
//	Real relEps = numDiff.relativeEpsilon;			//relative differentiation parameter
//	Real minCoord = numDiff.minimumCoordinateSize;	//absolute differentiation parameter is limited to this minimum
//	Real eps, epsInv; //coordinate(column)-wise differentiation parameter; depends on size of coordinate
//
//	Index nAE = cSystemData.GetNumberOfCoordinatesAE();
//	Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
//	Vector& x = cSystemData.GetCData().currentState.ODE2Coords_t;
//	Real xStore; //store value of x; avoid roundoff error effects in numerical differentiation
//
//	//++++++++++++++++++++++++++++++++++++++++++++++++
//	//compute total jacobian ==> very time consuming ==> change this to local jacobian (use flag in numDiffParameters?)
//
//	f0.SetNumberOfItems(nAE);
//	f1.SetNumberOfItems(nAE);
//
//	ComputeAlgebraicEquations(temp, f0, velocityLevel); //compute nominal value for jacobian
//
//
//	for (Index i = 0; i < nODE2; i++)
//	{
//		eps = relEps * (EXUstd::Maximum(minCoord, fabs(x[i])));
//		//pout << "Jac i=" << i << ", diffEps = " << eps << "\n";
//
//		xStore = x[i];
//		x[i] += eps;
//		ComputeAlgebraicEquations(temp, f1, velocityLevel);
//		x[i] = xStore;
//
//		epsInv = 1. / eps;
//
//		for (Index j = 0; j < nAE; j++)
//		{
//			Real x = epsInv * (f1[j] - f0[j]);
//			jacobian(j, i) = factor * x; //add CqT
//			//if (fillIntoSystemMatrix)
//			//{
//			//	jacobian(nODE2 + j, i) = factor * x; //add Cq
//			//	jacobian(i, nODE2 + j) = x; //add CqT
//			//}
//			//else
//			//{
//			//	jacobian(i, j) = x; //add CqT
//			//}
//		}
//	}
//}



//!compute per-object jacobians for object j, providing TemporaryComputationData;
//! returns ltgAE and ltgODE2 lists, and several flags on object velocity level equation and which jacobian parts have been computed
//! returns true, if jacobian is available, or false if not (e.g. body or ground object)
void CSystem::ComputeObjectJacobianAE(Index j, TemporaryComputationData& temp,
	bool& objectUsesVelocityLevel, bool& flagAE_ODE2filled, bool& flagAE_ODE2_tFilled, bool& flagAE_AEfilled)
{
	objectUsesVelocityLevel = false;
	CObject& object = *(cSystemData.GetCObjects()[j]);

	//Index markerType[2]; //markertypes stored; NOT USED
	flagAE_ODE2filled = false; //true, if the jacobian AE_ODE2 is inserted
	flagAE_ODE2_tFilled = false; //true, if the jacobian AE_ODE2 is inserted
	flagAE_AEfilled = false;   //true, if the jacobian AE_AE is inserted
	Real currentTime = cSystemData.GetCData().currentState.time;

	//for body, evaluate algebraic equations directly --> depend only on body coordinates
	if ((Index)object.GetType() & (Index)CObjectType::Body)
	{
		if (object.GetAlgebraicEquationsSize()) //either body or constraint
		{
			object.ComputeJacobianAE(temp.localJacobianAE, temp.localJacobianAE_t, temp.localJacobianAE_AE); //for objects, all jacobians need to be set!
			if (temp.localJacobianAE.NumberOfColumns()   * temp.localJacobianAE.NumberOfRows() != 0) { flagAE_ODE2filled = true; }
			if (temp.localJacobianAE_t.NumberOfColumns() * temp.localJacobianAE_t.NumberOfRows() != 0) { flagAE_ODE2_tFilled = true; }
			if (temp.localJacobianAE_AE.NumberOfColumns()* temp.localJacobianAE_AE.NumberOfRows() != 0) { flagAE_AEfilled = true; }
		}
	}
	//for constraint, algebraic equations depend on Markers
	else if ((Index)object.GetType() & (Index)CObjectType::Constraint)
	{
		CObjectConstraint& constraint = (CObjectConstraint&)object;

		const bool computeJacobian = true; //why needed for PostNewtonStep?==> check Issue #241
		ComputeMarkerDataStructure(&constraint, computeJacobian, temp.markerDataStructure);


		if (constraint.GetAvailableJacobians() & JacobianType::AE_ODE2)
		{
			flagAE_ODE2filled = true;
			CHECKandTHROW((constraint.GetAvailableJacobians() & JacobianType::AE_ODE2_function), "CSystem::JacobianAE: jacobian AE_ODE2 not implemented");
		}
		if (constraint.GetAvailableJacobians() & JacobianType::AE_ODE2_t)
		{
			flagAE_ODE2_tFilled = true;
			CHECKandTHROW((constraint.GetAvailableJacobians() & JacobianType::AE_ODE2_t_function), "CSystem::JacobianAE: jacobian AE_ODE2_t not implemented");
		}
		if (constraint.GetAvailableJacobians() & JacobianType::AE_AE)
		{
			flagAE_AEfilled = true;
			CHECKandTHROW((constraint.GetAvailableJacobians() & JacobianType::AE_AE_function), "CSystem::JacobianAE: jacobian AE_AE not implemented");
		}

		if (flagAE_ODE2filled || flagAE_ODE2_tFilled || flagAE_AEfilled)
		{
			constraint.ComputeJacobianAE(temp.localJacobianAE, temp.localJacobianAE_t, temp.localJacobianAE_AE, temp.markerDataStructure, currentTime);
			objectUsesVelocityLevel = constraint.UsesVelocityLevel();
			//if (constraint.UsesVelocityLevel()) {factorVelocityLevel = factorAE_ODE2_t; } //in this case, always use the velocity level factor; then, the jacobian is interpreted as diff(AE_t, ODE2_t)
		}
		//else
		//{} //FUTURE: alternatively compute numerically ==> currently only analytical computation possible!!!

	}
	else
	{
		CHECKandTHROWstring("CSystem::ComputeObjectJacobianAE(...): object type not implemented");
	}
}

//! compute constraint jacobian of AE with respect to ODE2 (fillIntoSystemMatrix=true: also w.r.t. ODE1 and AE) coordinates ==> direct computation given by access functions
//! factorODE2 is used to scale the ODE2-part of the jacobian (to avoid postmultiplication); 
//! velocityLevel = velocityLevel constraints are used, if available; 
//! fillIntoSystemMatrix: fill in both Cq and Cq^T into system matrix
//template<class TGeneralMatrix>
bool warnedCSystemJacobianAE = false;
void CSystem::JacobianAE(TemporaryComputationData& temp, const NewtonSettings& newton, GeneralMatrix& jacobianGM,
	Real factorAE_ODE2, Real factorAE_ODE2_t, bool velocityLevel, bool fillIntoSystemMatrix)
{
	//size needs to be set accordingly in the caller function; components are addd to massMatrix!

	//bool debugJacobian = true;
	//ResizableMatrix numJac; //for debug ...
	if (newton.useNumericalDifferentiation) 
	{
		NumericalJacobianAE(temp, newton.numericalDifferentiation, temp.numericalJacobianf0, temp.numericalJacobianf1, jacobianGM, factorAE_ODE2, factorAE_ODE2_t, velocityLevel, fillIntoSystemMatrix);
		//if (debugJacobian) { numJac = jacobian; }
	}
	else
	{
		if (velocityLevel) { CHECKandTHROWstring("CSystem::JacobianAE_ODE2: velocityLevel=true not implemented"); }
		//Index nAE = cSystemData.GetNumberOfCoordinatesAE();
		Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
		Index nODE1 = cSystemData.GetNumberOfCoordinatesODE1();

		if (!fillIntoSystemMatrix) //FUTURE: delete the whole if ... else... clause!
		{
			//this case should not be in use any more!
			if (!warnedCSystemJacobianAE) { PyWarning("CSystem::JacobianAE: Cq^T mode should not be used any more!"); warnedCSystemJacobianAE = true; }
		}

		//algebraic equations only origin from objects (e.g. Euler parameters) and constraints
		for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
		{
			//work over bodies, connectors, etc.
			//CObject& object = *(cSystemData.GetCObjects()[j]);
			ArrayIndex& ltgAE = cSystemData.GetLocalToGlobalAE()[j];
			ArrayIndex& ltgODE2 = cSystemData.GetLocalToGlobalODE2()[j];
			
			bool objectUsesVelocityLevel;// = false;
			bool flagAE_ODE2filled; //true, if the jacobian AE_ODE2 is inserted
			bool flagAE_ODE2_tFilled; //true, if the jacobian AE_ODE2 is inserted
			bool flagAE_AEfilled;   //true, if the jacobian AE_AE is inserted

			if (ltgAE.NumberOfItems()!=0 /*&& ltgODE2.NumberOfItems()!=0*/) //omit bodies and ground objects ...
			{

				ComputeObjectJacobianAE(j, temp, objectUsesVelocityLevel, flagAE_ODE2filled, flagAE_ODE2_tFilled, flagAE_AEfilled);

				//now add local jacobian to system jacobian
				if (!fillIntoSystemMatrix) //this is either the dC/dq or the dC_t/dq_t matrix for reaction forces ==> does not need factor for time integration and may only be added once (e.g. for non-holonomic constraints such as rolling wheel)
				{
					CHECKandTHROW((factorAE_ODE2 == 1.) && (factorAE_ODE2_t == 1.), "CSystem::JacobianAE(...): for reaction jacobian, factors must be 1.");
					if (flagAE_ODE2filled && !objectUsesVelocityLevel)
					{
						//for (Index ii = 0; ii < temp.localJacobianAE.NumberOfRows(); ii++)
						//{
						//	for (Index jj = 0; jj < temp.localJacobianAE.NumberOfColumns(); jj++)
						//	{
						//		jacobian(ltgAE[ii], ltgODE2[jj]) += temp.localJacobianAE(ii, jj); //factorVelocityLevel
						//	}
						//}
						jacobianGM.AddSubmatrix(temp.localJacobianAE, 1., ltgAE, ltgODE2);

					}
					else if (flagAE_ODE2_tFilled) //newly added; only add one of the two jacobians!
					{
						//for (Index ii = 0; ii < temp.localJacobianAE_t.NumberOfRows(); ii++)
						//{
						//	for (Index jj = 0; jj < temp.localJacobianAE_t.NumberOfColumns(); jj++)
						//	{
						//		jacobian(ltgAE[ii], ltgODE2[jj]) += temp.localJacobianAE_t(ii, jj); //
						//	}
						//}
						jacobianGM.AddSubmatrix(temp.localJacobianAE_t, 1., ltgAE, ltgODE2);
					}
					//else  //for pure algebraic constraints(e.g. if joints are deactivated) this is OK! {CHECKandTHROWstring("CSystem::JacobianAE(...): constraint jacobian must be consistent with UsesVelocityLevel flag");}
				}
				else
				{
					//now add all dependencies; this is the derivative of the constraint equations w.r.t. position and velocity level coordinates, but all transformed to the quantities needed in time integration (usually acceleration)
					if (flagAE_ODE2filled)
					{
						//for (Index ii = 0; ii < temp.localJacobianAE.NumberOfRows(); ii++)
						//{
						//	for (Index jj = 0; jj < temp.localJacobianAE.NumberOfColumns(); jj++)
						//	{
						//		jacobian(nODE2 + ltgAE[ii], ltgODE2[jj]) += factorAE_ODE2 * temp.localJacobianAE(ii, jj); //depends, if velocity or position level is used //factorVelocityLevel
						//	}
						//}
						jacobianGM.AddSubmatrix(temp.localJacobianAE, factorAE_ODE2, ltgAE, ltgODE2, nODE2);//depends, if velocity or position level is used

					}
					if (flagAE_ODE2_tFilled) //newly added
					{
						//for (Index ii = 0; ii < temp.localJacobianAE_t.NumberOfRows(); ii++)
						//{
						//	for (Index jj = 0; jj < temp.localJacobianAE_t.NumberOfColumns(); jj++)
						//	{
						//		jacobian(nODE2 + ltgAE[ii], ltgODE2[jj]) += factorAE_ODE2_t * temp.localJacobianAE_t(ii, jj); //depends, if velocity or position level is used 
						//	}
						//}
						jacobianGM.AddSubmatrix(temp.localJacobianAE_t, factorAE_ODE2_t, ltgAE, ltgODE2, nODE2); //depends, if velocity or position level is used 
					}

					//this is either the dC/dq or the dC_t/dq_t matrix for reaction forces ==> does not need factor for time integration and may only be added once (e.g. for non-holonomic constraints such as rolling wheel)
					if (flagAE_ODE2filled && !objectUsesVelocityLevel)
					{
						//for (Index ii = 0; ii < temp.localJacobianAE.NumberOfRows(); ii++)
						//{
						//	for (Index jj = 0; jj < temp.localJacobianAE.NumberOfColumns(); jj++)
						//	{
						//		jacobian(ltgODE2[jj], nODE2 + ltgAE[ii]) += temp.localJacobianAE(ii, jj); //this is the dC/dq^T part, which is independent of index reduction
						//	}
						//}
						jacobianGM.AddSubmatrixTransposed(temp.localJacobianAE, 1., ltgODE2, ltgAE, 0, nODE2); //this is the dC/dq^T part, which is independent of index reduction
					}
					else if (flagAE_ODE2_tFilled) //newly added
					{
						//for (Index ii = 0; ii < temp.localJacobianAE_t.NumberOfRows(); ii++)
						//{
						//	for (Index jj = 0; jj < temp.localJacobianAE_t.NumberOfColumns(); jj++)
						//	{
						//		jacobian(ltgODE2[jj], nODE2 + ltgAE[ii]) += temp.localJacobianAE_t(ii, jj); //this is the dC_t/dq_t^T part, which is independent of index reduction
						//	}
						//}
						jacobianGM.AddSubmatrixTransposed(temp.localJacobianAE_t, 1., ltgODE2, ltgAE, 0, nODE2); //this is the dC_t/dq_t^T part, which is independent of index reduction
					}
					//else  //for pure algebraic constraints(e.g. if joints are deactivated) this is OK! {CHECKandTHROWstring("CSystem::JacobianAE(...): constraint jacobian must be consistent with UsesVelocityLevel flag"); }

					if (flagAE_AEfilled) //pure algebraic equations: only depend on their algebraic part ...
					{
						//for (Index ii = 0; ii < temp.localJacobianAE_AE.NumberOfRows(); ii++)
						//{
						//	for (Index jj = 0; jj < temp.localJacobianAE_AE.NumberOfColumns(); jj++)
						//	{
						//		jacobian(nODE2 + ltgAE[ii], nODE2 + ltgAE[jj]) += temp.localJacobianAE_AE(ii, jj);
						//	}
						//}
						jacobianGM.AddSubmatrix(temp.localJacobianAE_AE, 1., ltgAE, ltgAE, nODE2, nODE2);
					}
				}


			}
		}//for cSystemData.GetCObjects()
	}//if(newton.useNumericalDifferentiation)

	//pout << "numJac   = " << numJac << "\n";
	//pout << "Jacobian = " << jacobian << "\n";
	//jacobian = numJac;

	//if (/*!fillIntoSystemMatrix && */debugJacobian) 
	//{
	//	Real diff = 0;
	//	if (fillIntoSystemMatrix)
	//	{
	//		Index nAE = cSystemData.GetNumberOfCoordinatesAE();
	//		Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
	//		Index nRows = nAE + nODE2;	//nAE / nAE+nODE2
	//		Index rowStart = 0;			//0 / nODE2

	//		diff = (jacobian.GetSubmatrix(rowStart,0,nRows,nODE2+nAE) - numJac.GetSubmatrix(rowStart, 0, nRows, nODE2 + nAE)).MaxNorm();
	//		if (diff > 1e-8) {
	//			pout << "jacobianAnalytical=" << jacobian.GetSubmatrix(rowStart, 0, nRows, nODE2 + nAE) << "\n";
	//			pout << "jacobianNumerical =" << numJac.GetSubmatrix(rowStart, 0, nRows, nODE2 + nAE) << "\n";
	//		}
	//	}
	//	else { 
	//		diff = (jacobian - numJac).MaxNorm(); 
	//		if (diff > 1e-8) {
	//			pout << "jacobianAnalytical=" << jacobian << "\n";
	//			pout << "jacobianNumerical =" << numJac << "\n";
	//		}
	//	}
	//	pout << "diff=" << diff << "\n";
	//	//WaitForUserToContinue();
	//}
}

//! add the projected action of Lagrange multipliers (reaction forces) to the ODE2 coordinates and add it to the ode2ReactionForces residual:
//! ode2ReactionForces += C_{q2}^T * \lambda
void CSystem::ComputeODE2ProjectedReactionForces(TemporaryComputationData& temp, const Vector& reactionForces, Vector& ode2ReactionForces)
{
	Index nAE = cSystemData.GetNumberOfCoordinatesAE();
	Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
	Index nODE1 = cSystemData.GetNumberOfCoordinatesODE1();

	CHECKandTHROW(reactionForces.NumberOfItems() == nAE, "CSystem::ComputeODE2ProjectedReactionForces: reactionForces size mismatch!");
	CHECKandTHROW(ode2ReactionForces.NumberOfItems() == nODE2, "CSystem::ComputeODE2ProjectedReactionForces: ode2ReactionForces size mismatch!");

	//algebraic equations only origin from objects (e.g. Euler parameters) and constraints
	for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
	{
		//work over bodies, connectors, etc.
		//CObject& object = *(cSystemData.GetCObjects()[j]);
		ArrayIndex& ltgAE = cSystemData.GetLocalToGlobalAE()[j];
		ArrayIndex& ltgODE2 = cSystemData.GetLocalToGlobalODE2()[j];

		bool objectUsesVelocityLevel;// = false;
		bool flagAE_ODE2filled; //true, if the jacobian AE_ODE2 is inserted
		bool flagAE_ODE2_tFilled; //true, if the jacobian AE_ODE2 is inserted
		bool flagAE_AEfilled;   //true, if the jacobian AE_AE is inserted

		if (ltgAE.NumberOfItems() != 0 && ltgODE2.NumberOfItems() != 0) //omit bodies and ground objects ...; for deactivated constraints, ltgAE!=0 and ltgODE2!=0
		{

			ComputeObjectJacobianAE(j, temp, objectUsesVelocityLevel, flagAE_ODE2filled, flagAE_ODE2_tFilled, flagAE_AEfilled);
			
			if (flagAE_ODE2filled || flagAE_ODE2_tFilled) //otherwise, no jacobians exist
			{
				if ((flagAE_ODE2filled && !objectUsesVelocityLevel) || flagAE_ODE2_tFilled) //must be consistent
				{
					ResizableMatrix& jac = flagAE_ODE2filled ? temp.localJacobianAE : temp.localJacobianAE_t;

					//multiply Cq^T * lambda:
					for (Index ii = 0; ii < jac.NumberOfRows(); ii++)
					{
						for (Index jj = 0; jj < jac.NumberOfColumns(); jj++)
						{
							ode2ReactionForces[ltgODE2[jj]] += reactionForces[ltgAE[ii]] * jac(ii, jj);  //add terms to existing residual forces
						}
					}
				}
				//else  //for pure algebraic constraints(e.g. if joints are deactivated) this is OK! CHECKandTHROWstring("CSystem::ComputeODE2ProjectedReactionForces(...): ObjectJacobians inconsistent!");
					//pout << "object " << j << ": CSystem::ComputeODE2ProjectedReactionForces(...): ObjectJacobians inconsistent\n";
					//pout << "flagAE_ODE2filled=" << flagAE_ODE2filled << "\n";
					//pout << "objectUsesVelocityLevel=" << objectUsesVelocityLevel << "\n";
					//pout << "flagAE_ODE2_tFilled=" << flagAE_ODE2_tFilled << "\n";
			}

		}
	}
}

//! compute numerically the derivative of (C_{q2} * v), v being an arbitrary vector
//! jacobianCqV = scalarFactor*d/dq2(C_{q2} * v)
void CSystem::ComputeConstraintJacobianDerivative(TemporaryComputationData& temp, const NumericalDifferentiationSettings& numDiff, Vector& f0, Vector& f1, 
	const Vector& v, GeneralMatrix& jacobianCqV, Real scalarFactor, Index rowOffset, Index columnOffset)
{

	//+++++++++++++++++++++++++++++++++++++++++++++++++++
	//jacobian
	Real relEps = numDiff.relativeEpsilon;			//relative differentiation parameter
	Real minCoord = numDiff.minimumCoordinateSize;	//absolute differentiation parameter is limited to this minimum
	Real eps, epsInv; //coordinate(column)-wise differentiation parameter; depends on size of coordinate

	Index nAE = cSystemData.GetNumberOfCoordinatesAE();
	Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
	Vector& x = cSystemData.GetCData().currentState.ODE2Coords;
	//Vector& x_t = cSystemData.GetCData().currentState.ODE2Coords_t; //velocity coordinates
	Real xStore; //store value of x; avoid roundoff error effects in numerical differentiation

	//Vector& z = cSystemData.GetCData().currentState.AECoords;
	//Real zStore; //store value of x; avoid roundoff error effects in numerical differentiation

	//++++++++++++++++++++++++++++++++++++++++++++++++

	if (jacobianCqV.GetSystemMatrixType() != LinearSolverType::EXUdense) { CHECKandTHROWstring("CSystem::ComputeConstraintJacobianDerivative: illegal LinearSolverType, only possible for dense matrix!"); }
	ResizableMatrix& jacobian = ((GeneralMatrixEXUdense&)jacobianCqV).GetMatrixEXUdense();
	
	f0.SetNumberOfItems(nAE);
	f1.SetNumberOfItems(nAE);
	ComputeConstraintJacobianTimesVector(temp, v, f0);
	//pout << "f0=" << f0 << "\n";
	//bool velocityLevel = false;

	//brute force approach, done for full matrix!
	//differentiation w.r.t. ODE2 coordinates
	for (Index i = 0; i < nODE2; i++)
	{
		eps = relEps * (EXUstd::Maximum(minCoord, fabs(x[i])));

		xStore = x[i];
		x[i] += eps;
		ComputeConstraintJacobianTimesVector(temp, v, f1);
		x[i] = xStore;
		//pout << "f1=" << f1 << "\n";

		epsInv = scalarFactor / eps;

		for (Index j = 0; j < nAE; j++)
		{
			Real x = epsInv * (f1[j] - f0[j]);
			jacobian(j + rowOffset, i + columnOffset) = x;
		}
	}

}

//! compute (C_{q2} * v), v being an arbitrary vector
void CSystem::ComputeConstraintJacobianTimesVector(TemporaryComputationData& temp, const Vector& v, Vector& result)
{
	Index nAE = cSystemData.GetNumberOfCoordinatesAE();
	Index nODE2 = cSystemData.GetNumberOfCoordinatesODE2();
	Index nODE1 = cSystemData.GetNumberOfCoordinatesODE1();

	CHECKandTHROW(v.NumberOfItems() == nODE2, "CSystem::ComputeConstraintJacobianTimesVector: v size mismatch!");
	result.SetNumberOfItems(nAE);
	result.SetAll(0.);

	//algebraic equations only origin from objects (e.g. Euler parameters) and constraints
	for (Index j = 0; j < cSystemData.GetCObjects().NumberOfItems(); j++)
	{
		//work over bodies, connectors, etc.
		//CObject& object = *(cSystemData.GetCObjects()[j]);
		ArrayIndex& ltgAE = cSystemData.GetLocalToGlobalAE()[j];
		ArrayIndex& ltgODE2 = cSystemData.GetLocalToGlobalODE2()[j];

		bool objectUsesVelocityLevel;// = false;
		bool flagAE_ODE2filled; //true, if the jacobian AE_ODE2 is inserted
		bool flagAE_ODE2_tFilled; //true, if the jacobian AE_ODE2 is inserted
		bool flagAE_AEfilled;   //true, if the jacobian AE_AE is inserted

		if (ltgAE.NumberOfItems() && ltgODE2.NumberOfItems()) //omit bodies and ground objects ...
		{

			ComputeObjectJacobianAE(j, temp, objectUsesVelocityLevel, flagAE_ODE2filled, flagAE_ODE2_tFilled, flagAE_AEfilled);
			
			if (flagAE_ODE2filled && !objectUsesVelocityLevel)
			{
				ResizableMatrix& jac = flagAE_ODE2filled ? temp.localJacobianAE : temp.localJacobianAE_t;

				//multiply Cq^T * lambda:
				for (Index ii = 0; ii < jac.NumberOfRows(); ii++)
				{
					for (Index jj = 0; jj < jac.NumberOfColumns(); jj++)
					{
						result[ltgAE[ii]] += jac(ii, jj) * v[ltgODE2[jj]];  //add terms to existing residual forces
					}
				}
			}
			else 
			{ 
				STDstring str = "CSystem::ComputeConstraintJacobianTimesVector(...) : not implemented for velocity level, objectNr = ";
				str += EXUstd::ToString(j);
				PyWarning(str);
				//CHECKandTHROWstring("CSystem::ComputeConstraintJacobianTimesVector(...): not implemented for velocity level!"); 
			} //needs different strategies for velocity level constraints!

		}
	}

}


//! this function is used to copy the current state to the visualization state and to send a signal that the PostProcessData has been updated
void CSystem::UpdatePostProcessData(bool recordImage)
{
	Index timeOut = 1000;		 //max iterations to wait, before frame is redrawn and saved
	Index timerMilliseconds = 2; //this is a hard-coded value, as visualizationSettings are not available here ...

	Index i = 0;
	//wait with new update until last image recording has been finished
	while (i++ < timeOut && (postProcessData.recordImageCounter == postProcessData.updateCounter))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(timerMilliseconds));
	}

	if (postProcessData.recordImageCounter == postProcessData.updateCounter)
	{
		PyWarning("CSystem::UpdatePostProcessData:: timeout for record image; try to decrease scene complexity");
	}

	//use semaphores, because the postProcessData.state is also accessed from the visualization thread
	postProcessData.accessState.test_and_set(std::memory_order_acquire); //lock PostProcessData

	postProcessData.updateCounter++;
	postProcessData.postProcessDataReady = true;
	if (recordImage) { postProcessData.recordImageCounter = postProcessData.updateCounter; } //this is the condition to record an image

	GetSystemData().GetCData().visualizationState = GetSystemData().GetCData().currentState; //copy current (computation step result) to post process state
	postProcessData.accessState.clear(std::memory_order_release); //clear PostProcessData
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this function is used to only send a signal that the scene shall be redrawn because the visualization state has been updated;
void PostProcessData::SendRedrawSignal()
{
	//use semaphores, because the postProcessData.state is also accessed from the visualization thread
	accessState.test_and_set(std::memory_order_acquire); //lock PostProcessData

	updateCounter++;
	postProcessDataReady = true;

	accessState.clear(std::memory_order_release); //clear PostProcessData
}

//! wait for user to press space
void PostProcessData::WaitForUserToContinue()
{
	if (visualizationIsRunning)
	{
		simulationPaused = true;
		STDstring str = GetVisualizationMessage();
		SetVisualizationMessage("Computation paused... (press SPACE to continue)");
		pout << "Computation paused... (press SPACE in render window to continue)\n";

		while (simulationPaused && visualizationIsRunning && !stopSimulation)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10)); //give thread time to finish the stop simulation command
			PyProcessExecuteQueue(); //use time to execute incoming python tasks
		}
		simulationPaused = false; //in case that visualization system was closed in the meantime
		SetVisualizationMessage(str);
	}
}
