/** ***********************************************************************************************
* @class        VisualizationObjectGenericODE2
* @brief        A system of \f$n\f$ second order ordinary differential equations (ODE2), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces. It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates. NOTE that all matrices, vectors, etc. must have the same dimensions \f$n\f$ or \f$(n \times n)\f$, or they must be empty \f$(0 \times 0)\f$, except for the mass matrix which always needs to have dimensions \f$(n \times n)\f$.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-04-14  23:21:10 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class VisualizationObjectGenericODE2: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    Float4 color;                                 //!< AUTO: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    Matrix triangleMesh;                          //!< AUTO: a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    bool showNodes;                               //!< AUTO: set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectGenericODE2()
    {
        show = true;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
        triangleMesh = Matrix();
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
    void SetTriangleMesh(const Matrix& value) { triangleMesh = value; }
    //! AUTO:  Read (Reference) access to:a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    const Matrix& GetTriangleMesh() const { return triangleMesh; }
    //! AUTO:  Read (Reference) access to:a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
    Matrix& GetTriangleMesh() { return triangleMesh; }

    //! AUTO:  Write (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    void SetShowNodes(const bool& value) { showNodes = value; }
    //! AUTO:  Read (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    const bool& GetShowNodes() const { return showNodes; }
    //! AUTO:  Read (Reference) access to:set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
    bool& GetShowNodes() { return showNodes; }

    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};


