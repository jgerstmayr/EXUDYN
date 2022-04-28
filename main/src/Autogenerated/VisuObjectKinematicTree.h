/** ***********************************************************************************************
* @class        VisualizationObjectKinematicTree
* @brief        A special object to represent open kinematic trees using minimum coordinate formulation (UNDER DEVELOPMENT!).
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-04-21  02:15:43 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTKINEMATICTREE__H
#define VISUALIZATIONOBJECTKINEMATICTREE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectKinematicTree: public VisualizationObjectSuperElement // AUTO: 
{
protected: // AUTO: 
    Float4 color;                                 //!< AUTO: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    MatrixI triangleMesh;                         //!< AUTO: a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    bool showNodes;                               //!< AUTO: set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    std::function<py::object(const MainSystem&, Index)> graphicsDataUserFunction;//!< AUTO: A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectKinematicTree()
    {
        show = true;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
        triangleMesh = MatrixI();
        showNodes = false;
        graphicsDataUserFunction = 0;
    };

    // AUTO: access functions
    //! AUTO:  Write (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    Float4& GetColor() { return color; }

    //! AUTO:  Write (Reference) access to:a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    void SetTriangleMesh(const MatrixI& value) { triangleMesh = value; }
    //! AUTO:  Read (Reference) access to:a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    const MatrixI& GetTriangleMesh() const { return triangleMesh; }
    //! AUTO:  Read (Reference) access to:a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    MatrixI& GetTriangleMesh() { return triangleMesh; }

    //! AUTO:  Write (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    void SetShowNodes(const bool& value) { showNodes = value; }
    //! AUTO:  Read (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    const bool& GetShowNodes() const { return showNodes; }
    //! AUTO:  Read (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    bool& GetShowNodes() { return showNodes; }

    //! AUTO:  user function which is called to update specific object graphics computed in Python functions; this is rather slow, but useful for user elements
    virtual void CallUserFunction(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, const MainSystem& mainSystem, Index itemNumber) override;

    //! AUTO:  return true, if object has a user function to be called during redraw
    virtual bool HasUserFunction() const override
    {
        return graphicsDataUserFunction!=0;
    }

    //! AUTO:  Write (Reference) access to:A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization
    void SetGraphicsDataUserFunction(const std::function<py::object(const MainSystem&, Index)>& value) { graphicsDataUserFunction = value; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization
    const std::function<py::object(const MainSystem&, Index)>& GetGraphicsDataUserFunction() const { return graphicsDataUserFunction; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization
    std::function<py::object(const MainSystem&, Index)>& GetGraphicsDataUserFunction() { return graphicsDataUserFunction; }

};



#endif //#ifdef include once...
