/** ***********************************************************************************************
* @class        VisualizationObjectGround
* @brief        A ground object behaving like a rigid body, but having no degrees of freedom; used to attach body-connectors without an action.
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

class VisualizationObjectGround: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    Float4 color;                                 //!< AUTO: RGB node color; if R==-1, use default color
    BodyGraphicsData graphicsData;                //!< AUTO: Structure contains data for body visualization; data is defined in special list / dictionary structure

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectGround()
    {
        show = true;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  Write (Reference) access to:RGB node color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGB node color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGB node color; if R==-1, use default color
    Float4& GetColor() { return color; }

    //! AUTO:  Write (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    void SetGraphicsData(const BodyGraphicsData& value) { graphicsData = value; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    const BodyGraphicsData& GetGraphicsData() const { return graphicsData; }
    //! AUTO:  Read (Reference) access to:Structure contains data for body visualization; data is defined in special list / dictionary structure
    BodyGraphicsData& GetGraphicsData() { return graphicsData; }

};


