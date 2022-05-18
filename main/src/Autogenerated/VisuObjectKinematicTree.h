/** ***********************************************************************************************
* @class        VisualizationObjectKinematicTree
* @brief        A special object to represent open kinematic trees using minimum coordinate formulation (UNDER DEVELOPMENT!). The kinematic tree is defined by lists of joint types, parents, inertia parameters, etc.\ per link (body). Every link is defined by a previous joint and a coordinate transformation from the previous link to this link's joint coordinates. Use specialized settings in VisualizationSettings.bodies.kinematicTree for showing joint frames and other properties.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-05-05  09:56:57 (last modified)
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
    bool showLinks;                               //!< AUTO: set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link
    bool showJoints;                              //!< AUTO: set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
    Float4 color;                                 //!< AUTO: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    MatrixI triangleMesh;                         //!< AUTO: unused in KinematicTree
    bool showNodes;                               //!< AUTO: unused in KinematicTree
    BodyGraphicsDataList graphicsDataList;        //!< AUTO: Structure contains data for link/joint visualization; data is defined as list of BodyGraphicdData where every BodyGraphicdData corresponds to one link/joint; must either be emtpy list or length must agree with number of links

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectKinematicTree()
    {
        show = true;
        showLinks = true;
        showJoints = true;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
        triangleMesh = MatrixI();
        showNodes = false;
    };

    // AUTO: access functions
    //! AUTO:  Write (Reference) access to:set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link
    void SetShowLinks(const bool& value) { showLinks = value; }
    //! AUTO:  Read (Reference) access to:set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link
    const bool& GetShowLinks() const { return showLinks; }
    //! AUTO:  Read (Reference) access to:set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link
    bool& GetShowLinks() { return showLinks; }

    //! AUTO:  Write (Reference) access to:set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
    void SetShowJoints(const bool& value) { showJoints = value; }
    //! AUTO:  Read (Reference) access to:set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
    const bool& GetShowJoints() const { return showJoints; }
    //! AUTO:  Read (Reference) access to:set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
    bool& GetShowJoints() { return showJoints; }

    //! AUTO:  Write (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
    Float4& GetColor() { return color; }

    //! AUTO:  Write (Reference) access to:unused in KinematicTree
    void SetTriangleMesh(const MatrixI& value) { triangleMesh = value; }
    //! AUTO:  Read (Reference) access to:unused in KinematicTree
    const MatrixI& GetTriangleMesh() const { return triangleMesh; }
    //! AUTO:  Read (Reference) access to:unused in KinematicTree
    MatrixI& GetTriangleMesh() { return triangleMesh; }

    //! AUTO:  Write (Reference) access to:unused in KinematicTree
    void SetShowNodes(const bool& value) { showNodes = value; }
    //! AUTO:  Read (Reference) access to:unused in KinematicTree
    const bool& GetShowNodes() const { return showNodes; }
    //! AUTO:  Read (Reference) access to:unused in KinematicTree
    bool& GetShowNodes() { return showNodes; }

    //! AUTO:  return true, if object has a user function to be called during redraw
    virtual bool HasUserFunction() const override
    {
        return false;
    }

    //! AUTO:  Write (Reference) access to:Structure contains data for link/joint visualization; data is defined as list of BodyGraphicdData where every BodyGraphicdData corresponds to one link/joint; must either be emtpy list or length must agree with number of links
    void SetGraphicsDataList(const BodyGraphicsDataList& value) { graphicsDataList = value; }
    //! AUTO:  Read (Reference) access to:Structure contains data for link/joint visualization; data is defined as list of BodyGraphicdData where every BodyGraphicdData corresponds to one link/joint; must either be emtpy list or length must agree with number of links
    const BodyGraphicsDataList& GetGraphicsDataList() const { return graphicsDataList; }
    //! AUTO:  Read (Reference) access to:Structure contains data for link/joint visualization; data is defined as list of BodyGraphicdData where every BodyGraphicdData corresponds to one link/joint; must either be emtpy list or length must agree with number of links
    BodyGraphicsDataList& GetGraphicsDataList() { return graphicsDataList; }

    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};



#endif //#ifdef include once...
