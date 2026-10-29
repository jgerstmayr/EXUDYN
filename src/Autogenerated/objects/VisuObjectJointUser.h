/** ***********************************************************************************************
* @class        VisualizationObjectJointUser
* @brief        A joint on two rigid markers whose constraint equations are a Python user function of the positions and rotation matrices of the markers. It has 1 to 6 equations, e.g. for a screw joint, a coupling of rotation and translation or a joint whose axes depend on the motion; the reaction forces and torques follow from the Lagrange multipliers, and the derivative of the equations is a second user function.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-29  11:13:44 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTJOINTUSER__H
#define VISUALIZATIONOBJECTJOINTUSER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class VisualizationObjectJointUser: public VisualizationObject // AUTO:
{
protected: // AUTO:
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> > graphicsDataUserFunction;//!< AUTO: A Python function which returns a list of graphics data in global coordinates, drawn for the joint, e.g. a cylinder along the axis of a revolute joint (#3147); it is called in the main thread and may slow down the visualization

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointUser()
    {
        show = true;
        graphicsDataUserFunction = 0;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  this function is needed to distinguish connector objects from body objects
    virtual bool IsConnector() const override
    {
        return true;
    }

    //! AUTO:  user function which is called to update specific object graphics computed in Python functions; this is rather slow, but useful for user elements
    virtual void CallUserFunction(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, const MainSystem& mainSystem, Index itemNumber) override;

    //! AUTO:  true, if the joint has a graphicsDataUserFunction to be called during redraw (#3147)
    virtual bool HasUserFunction() const override
    {
        return graphicsDataUserFunction!=0;
    }

    //! AUTO:  Write (Reference) access to:A Python function which returns a list of graphics data in global coordinates, drawn for the joint, e.g. a cylinder along the axis of a revolute joint (#3147); it is called in the main thread and may slow down the visualization
    void SetGraphicsDataUserFunction(const PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& value) { graphicsDataUserFunction = value; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a list of graphics data in global coordinates, drawn for the joint, e.g. a cylinder along the axis of a revolute joint (#3147); it is called in the main thread and may slow down the visualization
    const PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& GetGraphicsDataUserFunction() const { return graphicsDataUserFunction; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a list of graphics data in global coordinates, drawn for the joint, e.g. a cylinder along the axis of a revolute joint (#3147); it is called in the main thread and may slow down the visualization
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& GetGraphicsDataUserFunction() { return graphicsDataUserFunction; }

};



#endif //#ifdef include once...
