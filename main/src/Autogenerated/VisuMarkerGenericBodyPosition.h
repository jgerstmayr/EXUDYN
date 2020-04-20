/** ***********************************************************************************************
* @class        VisualizationMarkerGenericBodyPosition
* @brief        A position marker attached to a generic, discretized body, such as GenericODE2 or bodies modelled with the floating frame of reference formulation. The nodes of the body must provide position information. For a list of \f$n\f$ local node numbers, referencing to node points \f$\LU{b}{\pv_i}\f$ and weights \f$w_i\f$, the body-fixed marker position \f$\LU{b}{\pv_m}\f$ results in \f$\LU{b}{\pv_m} = \sum_{i=0}^{n-1}w_i \cdot \LU{b}{\pv_i}\f$. If the flag \texttt{useFirstNodeAsReferenceFrame} = \texttt{False}, then it follows that \f$\LU{0}{\pv_m} = \LU{b}{\pv_m}\f$. Otherwise \f$\LU{0}{\pv_m} = \LU{0b}{\Rot} \LU{b}{\pv_m}\f$, in which \f$\LU{0b}{\Rot}\f$ is the rotation matrix provided by the first node of the body, which also must provide orientation information.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-04-10  01:14:50 (last modfied)
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

class VisualizationMarkerGenericBodyPosition: public VisualizationMarker // AUTO: 
{
protected: // AUTO: 
    bool showMarkerNodes;                         //!< AUTO: set true, if all nodes are shown (similar to marker, but with less intensity)

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerGenericBodyPosition()
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


