/** ***********************************************************************************************
* @class        VisualizationObjectMassPoint
* @brief        A 3D mass point which is attached to a position-based node. Equations of motion with the displacements \f$[u_x\;\; u_y\;\; u_z]^T\f$, the mass \f$m\f$ and the residual of all forces \f$[R_x\;\; R_y\;\; R_z]^T\f$ are given as \f[ \vr{\ddot u_x m}{\ddot u_y m}{\ddot u_z m} = \vr{R_x}{R_y}{R_z}.\f]
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-09-12 (last modfied)
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

class VisualizationObjectMassPoint: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    BodyGraphicsData graphicsData;                //!< AUTO: Structure contains data for body visualization; data is defined in special list / dictionary structure

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectMassPoint()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  Write (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    void SetGraphicsData(const BodyGraphicsData& value) { graphicsData = value; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    const BodyGraphicsData& GetGraphicsData() const { return graphicsData; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    BodyGraphicsData& GetGraphicsData() { return graphicsData; }

};


