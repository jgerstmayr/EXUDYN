/** ***********************************************************************************************
* @class        VisualizationMarkerSuperElementPosition
* @brief        A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is inefficient!!!). The marker acts on the mesh nodes, not on the underlying nodes of the object. For a list of \f$n\f$ mesh node numbers, referencing to mesh node points \f$\LU{b}{\pv_i}\f$ and weights \f$w_i\f$, the body-fixed marker position \f$\LU{b}{\pv_m}\f$ results in \f$\LU{b}{\pv_m} = \sum_{i=0}^{n-1}w_i \cdot \LU{b}{\pv_i}\f$. EXAMPLE for single node marker on body 4, mesh node 10: MarkerSuperElementPosition(bodyNumber=4, meshNodeNumber=[10], weightingFactors=[1])
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-05-15  18:40:41 (last modfied)
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

class VisualizationMarkerSuperElementPosition: public VisualizationMarker // AUTO: 
{
protected: // AUTO: 
    bool showMarkerNodes;                         //!< AUTO: set true, if all nodes are shown (similar to marker, but with less intensity)

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerSuperElementPosition()
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


