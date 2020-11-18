/** ***********************************************************************************************
* @class        CData
* @brief        data needed from nodes in order to perform evaluation of MassMatrix, ODE2LHS, ...;@details                - note: initial position is referenceCoordinates + initialCoordinates;                - note: current position is referenceCoordinates + currentCoordinates
*
* @author       Gerstmayr Johannes
* @date         2018-05-18 (generated)
* @date         2019-05-09 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
				- email: johannes.gerstmayr@uibk.ac.at
				- weblink: missing
				
************************************************************************************************ */
#ifndef CDATA__H
#define CDATA__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include "Main/CSystemState.h"

class CData // 
{
public: // 
	CSystemState referenceState;                         //!< reference state coordinates: mainly position (mesh), but could also be stationary (velocity)
	CSystemState initialState;                           //!< initial state coordinates (initial conditions for time integration or Newton method)
	CSystemState currentState;                           //!< current state coordinates (e.g. during Newton, static solution or time integration)
	CSystemState startOfStepState;                       //!< state coordinates at beginning of computation step (static or time step); corresponds to current coordinates at beginning of step
	CSystemState visualizationState;                     //!< visualization state coordinates; usually updated at end of computation step; uses semaphores between rendering and computation threads

private:
	bool systemIsConsistent;							 //!< variable is set after check of system consistency ==> in order to draw or compute system; usually set after Assemble()

public: // 

	virtual ~CData() {} //added for correct deletion of derived classes
	// access functions
  //! clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer
  //virtual CData* GetClone() const { return new CData(*this); }

  //! Write (Reference) access to: current state coordinates (e.g. during Newton, static solution or time integration)
	CSystemState& GetCurrent() { return currentState; }
	//! Read (Reference) access to: current state coordinates (e.g. during Newton, static solution or time integration)
	const CSystemState& GetCurrent() const { return currentState; }

	//! Write (Reference) access to: initial state coordinates (initial conditions for time integration or Newton method)
	CSystemState& GetInitial() { return initialState; }
	//! Read (Reference) access to: initial state coordinates (initial conditions for time integration or Newton method)
	const CSystemState& GetInitial() const { return initialState; }

	//! Write (Reference) access to: reference state coordinates: mainly position (mesh), but could also be stationary (velocity)
	CSystemState& GetReference() { return referenceState; }
	//! Read (Reference) access to: reference state coordinates: mainly position (mesh), but could also be stationary (velocity)
	const CSystemState& GetReference() const { return referenceState; }

	//! Write (Reference) access to: state coordinates at beginning of computation step (static or time step); corresponds to current coordinates at beginning of step
	CSystemState& GetStartOfStep() { return startOfStepState; }
	//! Read (Reference) access to: state coordinates at beginning of computation step (static or time step); corresponds to current coordinates at beginning of step
	const CSystemState& GetStartOfStep() const { return startOfStepState; }

	//! Write (Reference) access to: state coordinates for visualization
	CSystemState& GetVisualization() { return visualizationState; }
	//! Read (Reference) access to: state coordinates for visualization
	const CSystemState& GetVisualization() const { return visualizationState; }

	//! return if systemStates are consistent (if not, e.g., system cannot be drawn using state coordinates)
	bool IsSystemConsistent() const { return systemIsConsistent; }

	//! Set true, if system is assembled, initialized and ready to be computed; special draw functions can be applied (visualization state exists)
	void SetSystemIsConsistent(bool flag) { systemIsConsistent = flag; }

	virtual void Print(std::ostream& os) const
	{
		os << "CData:\n";
		os << "  currentState = " << currentState << "\n";
		os << "  initialState = " << initialState << "\n";
		os << "  referenceState = " << referenceState << "\n";
		os << "  startOfStepState = " << startOfStepState << "\n";
		os << "\n";
	}

	friend std::ostream& operator<<(std::ostream& os, const CData& object)
	{
		object.Print(os);
		return os;
	}

};


#endif
