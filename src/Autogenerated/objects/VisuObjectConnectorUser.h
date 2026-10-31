/** ***********************************************************************************************
* @class        VisualizationObjectConnectorUser
* @brief        A connector of two markers whose forces are a user function of the frames and twists of the markers - their positions, rotations, velocities and angular velocities -: it returns the forces and torques on the markers, which the connector applies through the marker Jacobians - a spring-damper, a bushing, a contact of two points -, also as symbolic user function. For forces at points of a body that the function computes itself, as a contact searching the closest point on a flexible body, a second user function reads any number of markers of any type and applies forces through them. A NodeGenericData keeps a state, which a postNewtonStepUserFunction updates, as for contact or friction; its Jacobian is computed numerically.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-01  00:21:53 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONNECTORUSER__H
#define VISUALIZATIONOBJECTCONNECTORUSER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class VisualizationObjectConnectorUser: public VisualizationObject // AUTO:
{
protected: // AUTO:
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> > graphicsDataUserFunction;//!< AUTO: A Python function which returns a list of graphics data in global coordinates, drawn for the connector, e.g. a spring as a line (#3119); it is called in the main thread and may slow down the visualization

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorUser()
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

    //! AUTO:  true, if the connector has a graphicsDataUserFunction to be called during redraw (#3119)
    virtual bool HasUserFunction() const override
    {
        return graphicsDataUserFunction!=0;
    }

    //! AUTO:  Write (Reference) access to:A Python function which returns a list of graphics data in global coordinates, drawn for the connector, e.g. a spring as a line (#3119); it is called in the main thread and may slow down the visualization
    void SetGraphicsDataUserFunction(const PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& value) { graphicsDataUserFunction = value; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a list of graphics data in global coordinates, drawn for the connector, e.g. a spring as a line (#3119); it is called in the main thread and may slow down the visualization
    const PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& GetGraphicsDataUserFunction() const { return graphicsDataUserFunction; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a list of graphics data in global coordinates, drawn for the connector, e.g. a spring as a line (#3119); it is called in the main thread and may slow down the visualization
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Index)> >& GetGraphicsDataUserFunction() { return graphicsDataUserFunction; }

};



#endif //#ifdef include once...
