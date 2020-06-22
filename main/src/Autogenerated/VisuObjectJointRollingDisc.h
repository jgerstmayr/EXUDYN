/** ***********************************************************************************************
* @class        VisualizationObjectJointRollingDisc
* @brief        A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global \f$x\f$-\f$y\f$ plane. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-18  19:28:23 (last modfied)
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

class VisualizationObjectJointRollingDisc: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float discWidth;                              //!< AUTO: width of disc for drawing
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectJointRollingDisc()
    {
        show = true;
        discWidth = 0.1f;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  this function is needed to distinguish connector objects from body objects
    virtual bool IsConnector() const override
    {
        return true;
    }

    //! AUTO:  Write (Reference) access to:width of disc for drawing
    void SetDiscWidth(const float& value) { discWidth = value; }
    //! AUTO:  Read (Reference) access to:width of disc for drawing
    const float& GetDiscWidth() const { return discWidth; }
    //! AUTO:  Read (Reference) access to:width of disc for drawing
    float& GetDiscWidth() { return discWidth; }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};


