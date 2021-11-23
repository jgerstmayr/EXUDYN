/** ***********************************************************************************************
* @brief		Implementation for general contact computation
* @details		Details:
*
* @author		Gerstmayr Johannes
* @date			2021-10-23 (generated)
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

#include "Main/CSystemData.h"			//Basics, Vector/Array, OutputVariable, CData, Material, Body, Node, Marker, Load
#include "Main/CSystem.h"	

#include "System/CContact.h"
#include "Utilities/TimerStructure.h" //for local CPU time measurement

#ifdef USE_GENERAL_CONTACT

#ifdef USE_NGSOLVE_TASKMANAGER
#include "ngs-core-master/ngs_core.hpp"
#endif

//! perform operations in case that number of threads have been changed or initialize arrays
void GeneralContact::SetNumberOfThreads(Index nThreads)
{
	//assume that all arrays have same size!
	if (addedObjects.NumberOfItems() != nThreads)
	{
		for (Index i = 0; i < addedObjects.NumberOfItems(); i++)
		{
			delete addedObjects[i];
			delete addedObjectsFlags[i];
		}

		addedObjects.SetNumberOfItems(nThreads);
		addedObjectsFlags.SetNumberOfItems(nThreads);
		for (Index i = 0; i < addedObjects.NumberOfItems(); i++)
		{
			addedObjects[i] = new ArrayIndex();
			addedObjectsFlags[i] = new ResizableArray<bool>();

			addedObjects[i]->SetNumberOfItems(0);
			addedObjectsFlags[i]->SetNumberOfItems(TotalContactObjects());
			addedObjectsFlags[i]->SetAll(false); //need to initialize all to false; list must contain all false between operations

		}
	}
}

Index TSboundingBoxes;
TimerStructureRegistrator TSRboundingBoxes("Contact:BoundingBoxes", TSboundingBoxes, globalTimers);

Index TSsearchTree;
TimerStructureRegistrator TSRsearchTree("Contact:SearchTree", TSsearchTree, globalTimers);

Index TScontactODE2RHS;
TimerStructureRegistrator TSRcontactODE2RHS("Contact:ODE2RHS", TScontactODE2RHS, globalTimers);

Index TScontactJacobian;
TimerStructureRegistrator TSRcontactJacobian("Contact:Jacobian", TScontactJacobian, globalTimers);

Index TScontactPostNewton;
TimerStructureRegistrator TSRcontactPostNewton("Contact:PostNewton", TScontactPostNewton, globalTimers);

//! add contact object using a marker (Position or Rigid), radius and contact/friction parameters; 
//contact is possible between spheres (if intraSphereContact = true) and between sphere (=circle) and ANCFCable2D or sphere and line
void GeneralContact::AddSphereWithMarker(Index markerIndex, Real radius, Real contactStiffness, Real contactDamping, Index frictionMaterialIndex)
{
	ContactSpheresMarkerBased item;
	item.markerIndex = markerIndex;
	item.radius = radius;
	item.contactStiffness = contactStiffness;
	item.contactDamping = contactDamping;
	item.frictionMaterialIndex = frictionMaterialIndex;

	if (contactStiffness <= 0)
	{
		PyError("GeneralConact: AddSphereWithMarker(...): contactStiffness must be non-zero and positive (markerIndex=" +
			EXUstd::ToString(markerIndex) + ")");
	}

	spheresMarkerBased.Append(item);
}

//! set up necessary parameters for contact: friction, SearchTree, etc.; done after all contacts have been added
//! at this point, can also be checked if something is wrong (illegal pairings or frictionMaterial coeffs, etc.)
//! empty box will autocompute size!
void GeneralContact::FinalizeContact(const CSystem& cSystem, Index3 searchTreeSize, const Matrix& frictionPairingsInit, Vector3D searchTreeBoxMin, Vector3D searchTreeBoxMax)
{
	//pout << "finalize contact\n";
	if (!cSystem.IsSystemConsistent())
	{
		PyError("GeneralContact::FinalizeContact(...): must be called after Assemble()");
	}

	frictionPairings = frictionPairingsInit;

	if (frictionPairings.NumberOfColumns() != frictionPairings.NumberOfRows())
	{
		PyError("GeneralContact::FinalizeContact(...): frictionPairings matrix must have equal number of rows and columns");
	}

	//check if frictionPairings matrix is large enough:
	maxFrictionMaterialIndex = 0;
	CallOnAllContacts(This, NOARG, ComputeMaximumFrictionIndex);
	if (maxFrictionMaterialIndex >= frictionPairings.NumberOfRows())
	{
		//pout << "maxFric=" << maxFrictionMaterialIndex << "\n";
		PyError("GeneralContact::FinalizeContact(...): frictionMaterialIndex is larger than size of frictionPairings matrix");
	}

	//create relation between local and global contact indices
	globalContactIndexOffsets.Flush();
	globalContactIndexOffsets.Append(0); //first entry is first offset
	//get length of every contact list:
	EvaluateOnAllContacts(NumberOfItems, NOARG, globalContactIndexOffsets, Append);
	Index cnt = 0;
	for (Index& n : globalContactIndexOffsets)
	{
		n += cnt;
		cnt = n;
	}
	//==>globalContactIndexOffsets.Last(); //contains number of contact objects!
	allBoundingBoxes.SetNumberOfItems(TotalContactObjects());


	//create jacobians where needed (marker-based objects)
	allPositionJacobians.SetNumberOfItems(TotalContactObjects());
	allRotationJacobians.SetNumberOfItems(TotalContactObjects());
	allLTGs.SetNumberOfItems(TotalContactObjects());
	allActiveContacts.SetNumberOfItems(TotalContactObjects());
	//compute jacobians and LTGs for marker-based and other contact objects where jacobian is needed
	//pout << "initialize data, total objects=" << TotalContactObjects() << "\n";
	//pout << globalContactIndexOffsets << "\n";
	for (Index gi = 0; gi < TotalContactObjects(); gi++)
	{
		allActiveContacts[gi] = new ArrayIndex();

		Index i = gi - globalContactIndexOffsets[spheresMarkerBasedIndex];
		//pout << "i=" << i << ", gi =" << ", max(i)=" << globalContactIndexOffsets[spheresMarkerBasedIndex + 1] << "\n";
		if (i < globalContactIndexOffsets[spheresMarkerBasedIndex + 1]) //first are the spheres
		{
			allPositionJacobians[gi] = new ResizableMatrix();
			allRotationJacobians[gi] = new ResizableMatrix();
			allLTGs[gi] = new ArrayIndex();
			Index markerNumber = spheresMarkerBased[i].markerIndex;
			if (markerNumber >= cSystem.GetSystemData().GetCMarkers().NumberOfItems())
			{
				PyError("FinalizeContact: illegal marker Number of sphere with marker " + EXUstd::ToString(i));
			}
			cSystem.GetSystemData().ComputeMarkerODE2LTGarray(markerNumber, *(allLTGs[gi]), true); //true=reset list before appending
			//pout << "marker=" << markerNumber << ", LTG" << gi << ":" << *allLTGs[gi] << "\n";
		}
		else
		{
			allPositionJacobians[gi] = nullptr;
			allRotationJacobians[gi] = nullptr;
			allLTGs[gi] = nullptr;
		}
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//search tree autocompute must be done after creation of bounding boxes and initializations of jacobians
	Box3D searchTreeBox(searchTreeBoxMin, searchTreeBoxMax);
	TemporaryComputationDataArray tempArray; //will allocate memory, but just done in finalize contact
	if (searchTreeBox.Empty())
	{
		//pout << "auto compute searchTree box\n";
		//CHECKandTHROWstring("GeneralContact::FinalizeContact(...): autocompute searchTreeBox not implemented (specify a valid range!)");
		ComputeContactDataAndBoundingBoxes<true>(cSystem, tempArray, false); //compute initial bounding boxes for auto-compute searchTree size

		for (const Box3D& box : allBoundingBoxes)
		{
			//pout << "add box " << box << " ==> " << searchTreeBox << "\n";
			searchTreeBox.Add(box);
		}
		if (verboseMode >= 1) { pout << "auto computed searchTree box=" << searchTreeBox << "\n"; }
	}
	searchTree.ResetSearchTree(searchTreeSize[0], searchTreeSize[1], searchTreeSize[2], searchTreeBox);
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Real recommendedStepSize = 1; //not used
	PostNewtonStep(cSystem, tempArray, recommendedStepSize);
}

//! compute temporary data and bounding boxes
template<bool updateBoundingBoxes>
void GeneralContact::ComputeContactDataAndBoundingBoxes(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, bool addToSearchTree)
{
	STARTGLOBALTIMER(TSboundingBoxes);

	if (verboseMode >= 2) pout << "  **update Data, BB=" << updateBoundingBoxes << ", ST=" << addToSearchTree << "\n";
	Index nThreads = ngstd::TaskManager::GetNumThreads(); //must agree with tempArray
	//tempArray.SetNumberOfItems(nThreads); //only affected if changed

	const CSystemData& systemData = cSystem.GetSystemData();

	//allBoundingBoxes must be initialized in FinalizeContact(...)

	size_t nItems = (size_t)spheresMarkerBased.NumberOfItems();
	Index taskSplit = nThreads; //shall be multiple of number of treads (Default=nThreads), but better 8*nThreads or larger for large problems
	if ((Index)nItems > 400 * nThreads) { taskSplit = 100 * nThreads; }

	ngstd::ParallelFor(nItems, [this, &systemData, &tempArray, &nItems](size_t j) //(size_t j)
	//for (auto& item : spheresMarkerBased)
	{
		const bool computeJacobian = true; //good question, if jacobians should be precomputed; if many active contacts, this is better
		auto& item = spheresMarkerBased[(Index)j];
		Index threadID = ngstd::task_manager->GetThreadId();
		//MarkerData& markerData = temp.markerDataStructure.GetMarkerData(0);
		MarkerData& markerData = tempArray[threadID].markerDataStructure.GetMarkerData(0);
		Index gi = (Index)j + spheresMarkerBasedIndex; //
		//pout << "compute bounding box\n";
		//temporary data:
		systemData.GetCMarker(item.markerIndex).ComputeMarkerData(systemData, computeJacobian, markerData);
		item.position = markerData.position;
		item.orientation = markerData.orientation;
		item.velocity = markerData.velocity;
		this->allPositionJacobians[gi]->CopyFrom(markerData.positionJacobian);
		if (EXUstd::IsOfType(systemData.GetCMarker(item.markerIndex).GetType(), Marker::Orientation))
		{
			item.angularVelocity = markerData.angularVelocityLocal;
			this->allRotationJacobians[gi]->CopyFrom(markerData.rotationJacobian);
		}
		else
		{
			item.angularVelocity.SetAll(0.);
			this->allRotationJacobians[gi]->SetNumberOfRowsAndColumns(markerData.positionJacobian.NumberOfRows(), markerData.positionJacobian.NumberOfColumns());
			this->allRotationJacobians[gi]->SetAll(0.);
		}

		if (updateBoundingBoxes)
		{
			//bounding box:
			Vector3D vr({ item.radius, item.radius, item.radius });
			this->allBoundingBoxes[gi].PMin() = item.position - vr;
			this->allBoundingBoxes[gi].PMax() = item.position + vr;
		}
	},taskSplit); //nTasksPerThread
	STOPGLOBALTIMER(TSboundingBoxes);

	if (updateBoundingBoxes && addToSearchTree)
	{
		STARTGLOBALTIMER(TSsearchTree);
		searchTree.ClearItems();
		Index gi = 0;
		for (const auto& box : allBoundingBoxes)
		{
			searchTree.AddItem(box, gi++);
		}
		STOPGLOBALTIMER(TSsearchTree);
	}
}

template<Index opMode>
void GeneralContact::ComputeContact(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Vector& systemODE2Rhs)
{
	Index nThreads = ngstd::TaskManager::GetNumThreads();
	SetNumberOfThreads(nThreads);

	//not needed if CCode2rhsFromActiveSets:
	if (opMode & (CCode2rhsFull + CCactiveSets))
	{
		ComputeContactDataAndBoundingBoxes<true>(cSystem, tempArray, true);
	}
	else
	{
		ComputeContactDataAndBoundingBoxes<false>(cSystem, tempArray, false);
	}

	//only in case of ODE2rhs computation:
	if (opMode & (CCode2rhsFull + CCode2rhsFromActiveSets))
	{
		for (Index i = 0; i < nThreads; i++)
		{
			tempArray[i].sparseVector.SetAllZero();
		}
	}

	if (opMode & CCactiveSets)
	{
		if (verboseMode >= 2) pout  << "** clear active contacts **\n";
		EXUstd::ArrayOfArraysSetNumberOfItems0(allActiveContacts);
	}

	//loop over all contact spheres:
	size_t nItems = (size_t)spheresMarkerBased.NumberOfItems();

	Index taskSplit = nThreads; //shall be multiple of number of treads (Default=nThreads), but better 8*nThreads or larger for large problems
	if ((Index)nItems > 400 * nThreads) { taskSplit = 100 * nThreads; }

	ngstd::ParallelFor(nItems, [this, &cSystem, &tempArray, &nItems](size_t i)
		//for (Index i = 0; i < spheresMarkerBased.NumberOfItems(); i++)
	{
		//+++++++++++++++++++++++++++++
		//went inside parallel loop:
		Index threadID = ngstd::task_manager->GetThreadId();
		ResizableVector& ode2Lhs = tempArray[threadID].localODE2LHS;

		//+++++++++++++++++++++++++++++

		Index gi = (Index)i + spheresMarkerBasedIndex; //i is local, gi is global index (which is the same for the first contact objects)
		const ContactSpheresMarkerBased& sphereI = spheresMarkerBased[(Index)i];

		ResizableArray<Index>* contactObjects;
		if (opMode & CCode2rhsFromActiveSets)
		{
			contactObjects = allActiveContacts[gi];
		}
		else
		{
			contactObjects = addedObjects[threadID];
			//determine potential contacts using bounding boxes:
			//const Box3D& boxI = allBoundingBoxes[gi];
			searchTree.GetSingleItemsInBoxHalf(allBoundingBoxes[gi], *contactObjects, *addedObjectsFlags[threadID], gi);
		}

		//check if there is really contact:
		for (Index gj : *contactObjects)
		{
			//j = already global index! Index gj = j+spheresMarkerBasedIndex;
			const ContactSpheresMarkerBased& sphereJ = spheresMarkerBased[gj - spheresMarkerBasedIndex];
			Vector3D deltaP(sphereJ.position - sphereI.position);
			Real dist2 = deltaP.GetL2NormSquared();

			//in active set strategy, we always compute the contact, even if there is separation
			if (((opMode == CCode2rhsFromActiveSets) || (dist2 < EXUstd::Square(sphereI.radius + sphereJ.radius)) ) && dist2 != 0.)
			{ //now we have contact! sqrt and other functions are slow, but only called for contacts
				Real dist = sqrt(dist2); //distance of midpoints
				Real pen = dist - sphereI.radius - sphereJ.radius; //penetration
				if (opMode != CCactiveSets)
				{
					Vector3D deltaP0 = (1. / dist)*deltaP;
					if (verboseMode >= 2) pout  << "  ** inside contact computation\n";
					Real deltaV = deltaP0 * (sphereI.velocity - sphereJ.velocity); //penetration velocity, = -(v.J-v.I) !

					Real kContact = 1. / (1. / sphereI.contactStiffness + 1. / sphereJ.contactStiffness);
					Real dContact = 0;

					if (sphereI.contactDamping != 0 && sphereJ.contactDamping)
					{
						dContact = 1. / (1. / sphereI.contactDamping + 1. / sphereJ.contactDamping);
					}

					//compute scalar force:
					Real contactPressure = kContact * pen - dContact * deltaV;
					Vector3D fVec = contactPressure * deltaP0;

					//if (verboseMode >= 3) pout  << "contact f=" << fVec << "\n";

					//add generalized forces:
					//marker J (positive):    (according to computation of relative position)
					if (allPositionJacobians[gj]->NumberOfColumns() != 0) //special case: COGround has (0,0) Jacobian
					{
						EXUmath::MultMatrixTransposedVector(*allPositionJacobians[gj], fVec, ode2Lhs);
						//add to systemvector:
						for (Index k = 0; k < ode2Lhs.NumberOfItems(); k++)
						{
							tempArray[threadID].sparseVector.AddIndexAndValue(allLTGs[gj]->GetItem(k), ode2Lhs[k]); //added positive (LHS)
							//systemODE2Rhs[allLTGs[gj]->GetItem(k)] -= ode2Lhs[k]; //minus: LHS->RHS
						}
					}

					//marker I (negative):
					if (allPositionJacobians[gi]->NumberOfColumns() != 0) //special case: COGround has (0,0) Jacobian
					{
						EXUmath::MultMatrixTransposedVector(*allPositionJacobians[gi], -fVec, ode2Lhs);
						//add to systemvector:
						for (Index k = 0; k < ode2Lhs.NumberOfItems(); k++)
						{
							tempArray[threadID].sparseVector.AddIndexAndValue(allLTGs[gi]->GetItem(k), ode2Lhs[k]); //added positive (LHS)
							//systemODE2Rhs[allLTGs[gi]->GetItem(k)] -= ode2Lhs[k]; //minus: LHS->RHS
						}
					}
				}
				else //compute active sets/PostNewton
				{
					if (verboseMode >= 2) pout  << "  ** add active contact " << gj << " to " << gi << "\n";
					allActiveContacts[gi]->AppendPure(gj);
					//errorMax = EXUstd::Maximum(errorMax, fabs(pen));
				}
			}
		}
		if (!(opMode & CCode2rhsFromActiveSets))
		{
			(addedObjects[threadID])->SetNumberOfItems(0);
		}
	}, taskSplit); //must be multiple of number of treads, but better 8*nThreads or larger for large problems

	if (opMode & (CCode2rhsFull + CCode2rhsFromActiveSets))
	{
		//serial section for writing into system vector
		for (Index i = 0; i < nThreads; i++)
		{
			for (const EXUmath::IndexValue& item : tempArray[i].sparseVector.GetSparseIndexValues())
			{
				systemODE2Rhs[item.GetIndex()] -= item.GetValue(); //minus: LHS->RHS
			}
		}
	}
}


//! compute contact forces and add to global system vector
void GeneralContact::ComputeODE2RHS(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Vector& systemODE2Rhs)
{
	//if (verboseMode >= 3) pout  << "computeODE2RHS\n";
	if (!isActive) {return;}

	STARTGLOBALTIMER(TScontactODE2RHS);
	//pout << "doPostNewton=" << cSystem.GetSolverData().doPostNewtonIteration << "\n";
	if (cSystem.GetSolverData().doPostNewtonIteration)
	{
		ComputeContact<CCode2rhsFromActiveSets>(cSystem, tempArray, systemODE2Rhs);
		if (verboseMode >= 2) pout << "  systemODE2RhsActiveSet=" << systemODE2Rhs << ", c=" << cSystem.GetSystemData().GetCData().currentState.GetODE2Coords() << "\n";
	}
	else
	{
		ComputeContact<CCode2rhsFull>(cSystem, tempArray, systemODE2Rhs);
		if (verboseMode >= 2) pout << "  systemODE2RhsFull=" << systemODE2Rhs << ", c=" << cSystem.GetSystemData().GetCData().currentState.GetODE2Coords() << "\n";
	}
	STOPGLOBALTIMER(TScontactODE2RHS);


}

//! compute jacobian of ODE2RHS w.r.t. ODE2 and ODE2_t quantities; 
//! multiply (before added to jacobianGM) ODE2 with factorODE2 and ODE2_t with factorODE2_t
//! only sparse option available
void GeneralContact::JacobianODE2RHS(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, const NumericalDifferentiationSettings& numDiff,
	GeneralMatrix& jacobianGM, Real factorODE2, Real factorODE2_t)
{
	STARTGLOBALTIMER(TScontactJacobian);
	//if (verboseMode >= 3) pout  << "computeODE2RHS\n";
	if (!isActive) { return; }

	ComputeContactDataAndBoundingBoxes<false>(cSystem, tempArray, false);

	Index nThreads = ngstd::TaskManager::GetNumThreads();
	SetNumberOfThreads(nThreads);

	tempArray.SetNumberOfItems(nThreads); //only affected if changed; will be moved to CSystem!

	for (auto item : tempArray) {item->sparseTriplets.SetNumberOfItems(0);}

	//STARTGLOBALTIMER(TScontactODE2RHS);
	//loop over all contact spheres:
	size_t nItems = (size_t)spheresMarkerBased.NumberOfItems();

	//Index taskSplit = nThreads; //shall be multiple of number of treads (Default=nThreads), but better 8*nThreads or larger for large problems
	//if ((Index)nItems > 400 * nThreads) { taskSplit = 100 * nThreads; }
	//ngstd::ParallelFor(nItems, [this, &cSystem, &nItems](size_t i)
	//{
	////+++++++++++++++++++++++++++++
	////went inside parallel loop:
	//Index threadID = ngstd::task_manager->GetThreadId();

	for (Index i = 0; i < spheresMarkerBased.NumberOfItems(); i++)
	{
		Index threadID = 0;
		//ResizableVector& ode2Lhs = tempArray[threadID].localODE2LHS;
		auto& triplets = tempArray[threadID].sparseTriplets;
		//+++++++++++++++++++++++++++++

		Index gi = (Index)i + spheresMarkerBasedIndex; //i is local, gi is global index (which is the same for the first contact objects)
		const ContactSpheresMarkerBased& sphereI = spheresMarkerBased[(Index)i];

		////determine potential contacts using bounding boxes:
		//const Box3D& boxI = allBoundingBoxes[gi];
		//searchTree.GetSingleItemsInBoxHalf(boxI, *addedObjects[threadID], *addedObjectsFlags[threadID], gi);

		//run through all active contacts of sphere i
		for (Index gj : *(allActiveContacts[gi]))
		{
			//we assume contact!
			const ContactSpheresMarkerBased& sphereJ = spheresMarkerBased[gj - spheresMarkerBasedIndex];
			Vector3D deltaP(sphereJ.position - sphereI.position);

			//Real dist = sqrt(deltaP.GetL2NormSquared()); //distance of midpoints
			//Real pen = dist - sphereI.radius - sphereJ.radius; //penetration
			//Real deltaV = (1. / dist)*deltaP * (sphereI.velocity - sphereJ.velocity); //penetration velocity, = -(v.J-v.I) !

			Real kContact = 1. / (1. / sphereI.contactStiffness + 1. / sphereJ.contactStiffness);
			Real dContact = 0;

			if (sphereI.contactDamping != 0 && sphereJ.contactDamping)
			{
				dContact = 1. / (1. / sphereI.contactDamping + 1. / sphereJ.contactDamping);
			}

			//compute stiffness matrix in direction of contact, analogously to SpringDamper
			//Jj = d(k*deltaP^T*dp/dqj)/dqj, Ji = d(-k*deltaP^T*dp/dqi)/dqi
			//Jj = k*(dp/dqj)^T*dp/dqj, ...

			//rows of jacobian = dimension of position = always 3
			//columns of jacobians = coordinates, may be different

			Real fact = factorODE2 * kContact + factorODE2_t * dContact;

			const ResizableMatrix& JACj = *allPositionJacobians[gj];
			const ResizableMatrix& JACi = *allPositionJacobians[gi];
			Index columnsj = JACj.NumberOfColumns();
			Index columnsi = JACi.NumberOfColumns();

			ResizableMatrix& m = tempArray[threadID].localJacobian;
			if (columnsj) //in case of ground elements ...
			{
				EXUmath::MultMatrixTransposedMatrixTemplate<ResizableMatrix, ResizableMatrix, ResizableMatrix>(*allPositionJacobians[gj], *allPositionJacobians[gj], m);
				for (Index i = 0; i < m.NumberOfRows(); i++) {
					for (Index j = 0; j < m.NumberOfColumns(); j++) {
						Real value = m(i, j);
						if (value != 0.)
						{
							triplets.AppendPure(SparseTriplet(allLTGs[gj]->GetItem(i), allLTGs[gj]->GetItem(j), -fact * value));
						}
					}
				}
				if (columnsi)
				{
					EXUmath::MultMatrixTransposedMatrixTemplate<ResizableMatrix, ResizableMatrix, ResizableMatrix>(*allPositionJacobians[gj], *allPositionJacobians[gi], m);
					for (Index i = 0; i < m.NumberOfRows(); i++) {
						for (Index j = 0; j < m.NumberOfColumns(); j++) {
							Real value = m(i, j);
							if (value != 0.)
							{
								triplets.AppendPure(SparseTriplet(allLTGs[gj]->GetItem(i), allLTGs[gi]->GetItem(j), fact * value));
								triplets.AppendPure(SparseTriplet(allLTGs[gi]->GetItem(j), allLTGs[gj]->GetItem(i), fact * value)); //transposed
							}
						}
					}
				}
			}

			if (columnsi)
			{
				EXUmath::MultMatrixTransposedMatrixTemplate<ResizableMatrix, ResizableMatrix, ResizableMatrix>(*allPositionJacobians[gi], *allPositionJacobians[gi], m);
				for (Index i = 0; i < m.NumberOfRows(); i++) {
					for (Index j = 0; j < m.NumberOfColumns(); j++) {
						Real value = m(i, j);
						if (value != 0.)
						{
							triplets.AppendPure(SparseTriplet(allLTGs[gi]->GetItem(i), allLTGs[gi]->GetItem(j), -fact * value));
						}
					}
				}
			}


		}
	}//serial
	//}, taskSplit); //must be multiple of number of treads, but better 8*nThreads or larger for large problems

	//serial section for writing all triplets into Jacobian
	for (Index i = 0; i < nThreads; i++)
	{
		jacobianGM.AddSparseTriplets(tempArray[i].sparseTriplets);
	}

	if (verboseMode >= 3) pout  << "  jac=" << jacobianGM.GetEXUdenseMatrix() <<"\n";

	STOPGLOBALTIMER(TScontactJacobian);
}

//! PostNewtonStep: update active sets for contact
//! recommended step size \f$h_{recom}\f$ after PostNewton(...): \f$h_{recom} < 0\f$: no recommendation, \f$h_{recom}==0\f$: use minimum step size, \f$h_{recom}>0\f$: use specific step size, if no smaller size requested by other reason
Real GeneralContact::PostNewtonStep(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Real& recommendedStepSize)
{
	if (verboseMode >= 2) pout  << "\n****************\n  Post Newton\nt=" << cSystem.GetSystemData().GetCData().currentState.GetTime() << "\n";

	if (cSystem.GetSolverData().doPostNewtonIteration)
	{
		STARTGLOBALTIMER(TScontactPostNewton);
		Index oldCnt = EXUstd::ArrayOfArraysTotalCount(allActiveContacts);
		Vector systemODE2Rhs; //dummy, unused
		ComputeContact<CCactiveSets>(cSystem, tempArray, systemODE2Rhs);

		//Index activeSet = 0;
		//for (auto* cset : allActiveContacts)
		//{
		//	activeSet += cset->NumberOfItems();
		//}
		//if (verboseMode >= 2) pout << "  active set =" << activeSet << ", oldActiveSet=" << oldCnt << "\n";

		STOPGLOBALTIMER(TScontactPostNewton);
		return fabs(EXUstd::ArrayOfArraysTotalCount(allActiveContacts) - oldCnt); //use change of total size of active sets for now
	}
	return 0.;
}











//OLD, DELETE:
//! compute contact forces and add to global system vector
//void GeneralContact::ComputeODE2RHS(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Vector& systemODE2Rhs)
//{
//	//pout << "computeODE2RHS\n";
//	if (!isActive) { return; }
//
//	ComputeContactDataAndBoundingBoxes(cSystem, tempArray);
//
//	Index nThreads = ngstd::TaskManager::GetNumThreads();
//	SetNumberOfThreads(nThreads);
//
//	tempArray.SetNumberOfItems(nThreads); //only affected if changed; will be moved to CSystem!
//	for (Index i = 0; i < nThreads; i++)
//	{
//		tempArray[i].sparseVector.SetAllZero();
//	}
//
//	STARTGLOBALTIMER(TScontactODE2RHS);
//	//loop over all contact spheres:
//	size_t nItems = (size_t)spheresMarkerBased.NumberOfItems();
//
//	Index taskSplit = nThreads; //shall be multiple of number of treads (Default=nThreads), but better 8*nThreads or larger for large problems
//	if (nItems > 400 * nThreads) { taskSplit = 100 * nThreads; }
//
//	ngstd::ParallelFor(nItems, [this, &cSystem, &tempArray, &nItems](size_t i)
//		//for (Index i = 0; i < spheresMarkerBased.NumberOfItems(); i++)
//	{
//		//+++++++++++++++++++++++++++++
//		//went inside parallel loop:
//		Index threadID = ngstd::task_manager->GetThreadId();
//		//MarkerData& markerData = temp.markerDataStructure.GetMarkerData(0);
//		//TemporaryComputationData& temp = this->tempArray[threadID];
//		ResizableVector& ode2Lhs = tempArray[threadID].localODE2LHS;
//
//		//+++++++++++++++++++++++++++++
//
//		Index gi = (Index)i + spheresMarkerBasedIndex; //i is local, gi is global index (which is the same for the first contact objects)
//		const ContactSpheresMarkerBased& sphereI = spheresMarkerBased[(Index)i];
//
//		//determine potential contacts using bounding boxes:
//		const Box3D& boxI = allBoundingBoxes[gi];
//		searchTree.GetSingleItemsInBoxHalf(boxI, *addedObjects[threadID], *addedObjectsFlags[threadID], gi);
//
//		//if (addedObjects.NumberOfItems()) pout << "added objects=" << addedObjects << "\n";
//
//		//check if there is really contact:
//		//for (Index j = 0; j < addedObjects.NumberOfItems(); j++)
//		for (Index gj : *addedObjects[threadID])
//		{
//			//j = already global index! Index gj = j+spheresMarkerBasedIndex;
//			//if (gj < gi) //every contact only once, no self-contact
//			{
//				const ContactSpheresMarkerBased& sphereJ = spheresMarkerBased[gj - spheresMarkerBasedIndex];
//				Vector3D deltaP(sphereJ.position - sphereI.position);
//				Real dist2 = (deltaP).GetL2NormSquared();
//				if (dist2 < EXUstd::Square(sphereI.radius + sphereJ.radius) && dist2 != 0.)
//				{ //now we have contact! sqrt and other functions are slow, but only called for contacts
//					Real dist = sqrt(dist2); //distance of midpoints
//					Real pen = dist - sphereI.radius - sphereJ.radius; //penetration
//					Real deltaV = (1. / dist)*deltaP * (sphereI.velocity - sphereJ.velocity); //penetration velocity, = -(v.J-v.I) !
//
//					Real kContact = 1. / (1. / sphereI.contactStiffness + 1. / sphereJ.contactStiffness);
//					Real dContact = 0;
//
//					if (sphereI.contactDamping != 0 && sphereJ.contactDamping)
//					{
//						dContact = 1. / (1. / sphereI.contactDamping + 1. / sphereJ.contactDamping);
//					}
//
//					//compute scalar force:
//					Real contactPressure = kContact * pen - dContact * deltaV;
//					Vector3D fVec = contactPressure * deltaP;
//
//					//pout << "contact f=" << fVec << "\n";
//
//					//add generalized forces:
//					//marker J (positive):    (according to computation of relative position)
//					if (allPositionJacobians[gj]->NumberOfColumns() != 0) //special case: COGround has (0,0) Jacobian
//					{
//						EXUmath::MultMatrixTransposedVector(*allPositionJacobians[gj], fVec, ode2Lhs);
//						//add to systemvector:
//						for (Index k = 0; k < ode2Lhs.NumberOfItems(); k++)
//						{
//							tempArray[threadID].sparseVector.AddIndexAndValue(allLTGs[gj]->GetItem(k), ode2Lhs[k]); //added positive (LHS)
//							//systemODE2Rhs[allLTGs[gj]->GetItem(k)] -= ode2Lhs[k]; //minus: LHS->RHS
//						}
//					}
//
//					//marker I (negative):
//					if (allPositionJacobians[gi]->NumberOfColumns() != 0) //special case: COGround has (0,0) Jacobian
//					{
//						EXUmath::MultMatrixTransposedVector(*allPositionJacobians[gi], -fVec, ode2Lhs);
//						//add to systemvector:
//						for (Index k = 0; k < ode2Lhs.NumberOfItems(); k++)
//						{
//							tempArray[threadID].sparseVector.AddIndexAndValue(allLTGs[gi]->GetItem(k), ode2Lhs[k]); //added positive (LHS)
//							//systemODE2Rhs[allLTGs[gi]->GetItem(k)] -= ode2Lhs[k]; //minus: LHS->RHS
//						}
//					}
//
//				}
//			}
//		}
//		(addedObjects[threadID])->SetNumberOfItems(0);
//	}, taskSplit); //must be multiple of number of treads, but better 8*nThreads or larger for large problems
//
//	//serial section for writing into system vector
//	for (Index i = 0; i < nThreads; i++)
//	{
//		for (const EXUmath::IndexValue& item : tempArray[i].sparseVector.GetSparseIndexValues())
//		{
//			systemODE2Rhs[item.GetIndex()] -= item.GetValue(); //minus: LHS->RHS
//		}
//	}
//	STOPGLOBALTIMER(TScontactODE2RHS);
//}
//





#endif