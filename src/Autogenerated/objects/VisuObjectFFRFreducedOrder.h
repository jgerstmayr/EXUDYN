/** ***********************************************************************************************
* @class        VisualizationObjectFFRFreducedOrder
* @brief        This object is used to represent modally reduced flexible bodies using the ABRV:FFRF and the ABRV:CMS. It can be used to model real-life mechanical systems imported from finite element codes or Python tools such as NETGEN/NGsolve, see the `FEMinterface` in [](#sec-fem-feminterface---init--). It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class `ObjectFFRFreducedOrderInterface`, especially the user functions `UFmassFFRFreducedOrder` and `UFforceFFRFreducedOrder`, [](#sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions).
*
* @author       Gerstmayr Johannes, Zwölfer Andreas
* @date         2019-07-01 (generated)
* @date         2026-10-02  08:56:17 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTFFRFREDUCEDORDER__H
#define VISUALIZATIONOBJECTFFRFREDUCEDORDER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectFFRFreducedOrder: public VisualizationObjectSuperElement // AUTO:
{
protected: // AUTO:
    Float4 color;                                 //!< AUTO: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    MatrixI triangleMesh;                         //!< AUTO: a matrix of node numbers referring to the mesh nodes of the object, one triangle per row: 3 columns for flat triangles, or 6 for 6-node triangles drawn curved - the corners counterclockwise seen from outside, then the mid nodes of the edges 0-1, 1-2 and 2-0, as in GraphicsData triangles6; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    bool showNodes;                               //!< AUTO: set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectFFRFreducedOrder()
    {
        show = true;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
        triangleMesh = MatrixI();
        showNodes = false;
    };

    // AUTO: access functions
    //! AUTO:  Write (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    Float4& GetColor() { return color; }

    //! AUTO:  Write (Reference) access to:a matrix of node numbers referring to the mesh nodes of the object, one triangle per row: 3 columns for flat triangles, or 6 for 6-node triangles drawn curved - the corners counterclockwise seen from outside, then the mid nodes of the edges 0-1, 1-2 and 2-0, as in GraphicsData triangles6; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    void SetTriangleMesh(const MatrixI& value) { triangleMesh = value; }
    //! AUTO:  Read (Reference) access to:a matrix of node numbers referring to the mesh nodes of the object, one triangle per row: 3 columns for flat triangles, or 6 for 6-node triangles drawn curved - the corners counterclockwise seen from outside, then the mid nodes of the edges 0-1, 1-2 and 2-0, as in GraphicsData triangles6; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    const MatrixI& GetTriangleMesh() const { return triangleMesh; }
    //! AUTO:  Read (Reference) access to:a matrix of node numbers referring to the mesh nodes of the object, one triangle per row: 3 columns for flat triangles, or 6 for 6-node triangles drawn curved - the corners counterclockwise seen from outside, then the mid nodes of the edges 0-1, 1-2 and 2-0, as in GraphicsData triangles6; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    MatrixI& GetTriangleMesh() { return triangleMesh; }

    //! AUTO:  Write (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    void SetShowNodes(const bool& value) { showNodes = value; }
    //! AUTO:  Read (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    const bool& GetShowNodes() const { return showNodes; }
    //! AUTO:  Read (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    bool& GetShowNodes() { return showNodes; }

};



#endif //#ifdef include once...
