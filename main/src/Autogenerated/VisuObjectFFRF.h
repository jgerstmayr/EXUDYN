/** ***********************************************************************************************
* @class        VisualizationObjectFFRF
* @brief        This object is used to represent equations modelled by the floating frame of reference formulation (FFRF). It contains a RigidBodyNode (always node 0) and a list of other nodes representing the finite element nodes used in the FFRF. Note that temporary matrices and vectors are subject of change in future.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-09-08  18:14:39 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTFFRF__H
#define VISUALIZATIONOBJECTFFRF__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectFFRF: public VisualizationObjectSuperElement // AUTO: 
{
protected: // AUTO: 
    Float4 color;                                 //!< AUTO: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    MatrixI triangleMesh;                         //!< AUTO: a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    bool showNodes;                               //!< AUTO: set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectFFRF()
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

};



#endif //#ifdef include once...
