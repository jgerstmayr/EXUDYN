/** ***********************************************************************************************
* @class        VisualizationMarkerSuperElementRigid
* @brief        A position and orientation (rigid-body) marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it may be inefficient). The marker acts on the mesh nodes, not on the underlying nodes of the object. Note that in contrast to the MarkerSuperElementPosition, this marker needs a set of interface nodes which are not aligned at one line, such that the can represent rigid body motion. Note that definitions of marker positions are slightly different from MarkerSuperElementPosition.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-01  20:10:12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class VisualizationMarkerSuperElementRigid: public VisualizationMarker // AUTO: 
{
protected: // AUTO: 
    bool showMarkerNodes;                         //!< AUTO: set true, if all nodes are shown (similar to marker, but with less intensity)

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerSuperElementRigid()
    {
        show = true;
        showMarkerNodes = true;
    };

    // AUTO: access functions
    //! AUTO:  Write (Reference) access to:set true, if all nodes are shown (similar to marker, but with less intensity)
    void SetShowMarkerNodes(const bool& value) { showMarkerNodes = value; }
    //! AUTO:  Read (Reference) access to:set true, if all nodes are shown (similar to marker, but with less intensity)
    const bool& GetShowMarkerNodes() const { return showMarkerNodes; }
    //! AUTO:  Read (Reference) access to:set true, if all nodes are shown (similar to marker, but with less intensity)
    bool& GetShowMarkerNodes() { return showMarkerNodes; }

    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};


