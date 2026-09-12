/** ***********************************************************************************************
* @class	    MainNode
* @brief		Class for main nodes (all handling, python interfaces, etc. to CNodes)
* @details		Details:
 				- nodes define coordinates of computational objects (CObjects)
                - nodes can be of one category: ODE1coordinates, ODE2coordinates, AEvariables, DataVariables
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
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
#ifndef MAINNODE__H
#define MAINNODE__H

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <pybind11/pybind11.h>      //! include pybind for dictionary access
#include <pybind11/stl.h>           //! needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!

namespace py = pybind11;            //! "py" used throughout in code

//! this is the general CNode interface class, needed to list all Node Objects
class MainNode
{
protected:
	STDstring name;
public:
	virtual ~MainNode() {} //added for correct deletion of derived classes
	virtual MainNode* GetClone() const { return new MainNode(*this); }

	virtual STDstring& GetName() { return name; }
	virtual const STDstring& GetName() const { return name; }

	virtual const char* GetTypeName() const { SysError("Invalid call to MainNode::GetTypeName");  return "Node::Invalid"; }

	virtual void SetWithDictionary(const py::dict& d) { SysError("Invalid call to MainNode::SetWithDictionary"); }
	virtual py::dict GetDictionary() const { SysError("Invalid call to MainNode::GetDictionary");  return py::dict(); }

	//! Get const pointer to computational base class item
	virtual CNode* GetCNode() const { SysError("Invalid call to MainNode::GetCNode");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetCNode(CNode* pCNode) { SysError("Invalid call to MainNode::SetCNode"); }

	//! Get const pointer to visualization base class item
	virtual VisualizationNode* GetVisualizationNode() const { SysError("Invalid call to MainNode::GetVisualizationNode");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetVisualizationNode(VisualizationNode* pVisualizationNode) { SysError("Invalid call to MainNode::SetVisualizationNode"); }

	//! Hook to initial values vector (displacements) implemented in derived class
	virtual LinkedDataVector GetInitialCoordinateVector() const { PyError("Node does not support GetInitialCoordinateVector functionality"); return LinkedDataVector(); }
	//! Hook to initial values vector (velocities) implemented in derived class
	virtual LinkedDataVector GetInitialCoordinateVector_t() const { PyError("Node does not support GetInitialCoordinateVector_t functionality"); return LinkedDataVector(); }

	//! set initial coordinates, usually using default; special classes (Lie group) override
	virtual void SetInitialCoordinateVector(LinkedDataVector& initialVector) { initialVector = GetInitialCoordinateVector(); }
	//! set initial coordinates, usually using default; special classes (Lie group) override
	virtual void SetInitialCoordinateVector_t(LinkedDataVector& initialVector_t) { initialVector_t = GetInitialCoordinateVector_t(); }

	//! set initial coordinates, usually using default; special classes (Lie group) override
	virtual void SetInitialDataCoordinateVector(LinkedDataVector& initialVector) { initialVector = GetInitialCoordinateVector(); }


	//! GetOutputVariable with type and return value; copies values==>slow!; can be scalar or vector-valued! maps to CNode GetOutputVariable(...)
	virtual py::object GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration) const;

	//! call pybind object function, possibly with arguments
	virtual py::object CallFunction(STDstring functionName, py::dict args) const { PyError("Node does not support CallFunction(...) functionality"); return py::object(); }

	//! Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail; override by according classes
	virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const { return true; }

	//! Get (read) parameter 'parameterName' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
	virtual py::object GetParameter(const STDstring& parameterName) const { SysError("Invalid call to MainNode::GetParameter"); return py::object(); }
	//! Set (write) parameter 'parameterName' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
	virtual void SetParameter(const STDstring& parameterName, const py::object& value) { SysError("Invalid call to MainNode::SetParameter(...)"); }
};

#endif
