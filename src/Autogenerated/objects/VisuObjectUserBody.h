/** ***********************************************************************************************
* @class        VisualizationObjectUserBody
* @brief        A body whose equations of motion and whose position, and optionally orientation, at a local position are computed by Python user functions: a user finite element or a user rigid body on existing nodes. As it provides the access functions of a body, the body markers (MarkerBodyPosition, MarkerBodyRigid), SensorBody, the loads on these markers and every connector work on it.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-12  01:02:16 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTUSERBODY__H
#define VISUALIZATIONOBJECTUSERBODY__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class VisualizationObjectUserBody: public VisualizationObject // AUTO:
{
protected: // AUTO:
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> > graphicsDataUserFunction;//!< AUTO: A Python function which returns a list of graphics data in global coordinates, see ObjectGenericODE2; the only drawing of the body

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectUserBody()
    {
        show = true;
        graphicsDataUserFunction = 0;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  user function which is called to update specific object graphics computed in Python functions; this is rather slow, but useful for user elements
    virtual void CallUserFunction(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, const MainSystem& mainSystem, Index itemNumber) override;

    //! AUTO:  return true, if object has a user function to be called during redraw
    virtual bool HasUserFunction() const override
    {
        return true;
    }

    //! AUTO:  Write (Reference) access to:A Python function which returns a list of graphics data in global coordinates, see ObjectGenericODE2; the only drawing of the body
    void SetGraphicsDataUserFunction(const PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& value) { graphicsDataUserFunction = value; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a list of graphics data in global coordinates, see ObjectGenericODE2; the only drawing of the body
    const PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& GetGraphicsDataUserFunction() const { return graphicsDataUserFunction; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a list of graphics data in global coordinates, see ObjectGenericODE2; the only drawing of the body
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& GetGraphicsDataUserFunction() { return graphicsDataUserFunction; }

};



#endif //#ifdef include once...
