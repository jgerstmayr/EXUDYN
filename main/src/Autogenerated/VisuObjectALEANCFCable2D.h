/** ***********************************************************************************************
* @class        VisualizationObjectALEANCFCable2D
* @brief        A 2D cable finite element using 2 nodes of type NodePoint2DSlope1 and a axially moving coordinate of type NodeGenericODE2; the element has 8+1 coordinates and uses cubic polynomials for position interpolation; the element in addition to ANCFCable2D adds an Eulerian axial velocity by the GenericODE2 coordiante
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

class VisualizationObjectALEANCFCable2D: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float drawHeight;                             //!< AUTO: if beam is drawn with rectangular shape, this is the drawing height
    Float4 color;                                 //!< AUTO: RGBA color of the object; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectALEANCFCable2D()
    {
        show = true;
        drawHeight = 0.f;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  Write (Reference) access to:if beam is drawn with rectangular shape, this is the drawing height
    void SetDrawHeight(const float& value) { drawHeight = value; }
    //! AUTO:  Read (Reference) access to:if beam is drawn with rectangular shape, this is the drawing height
    const float& GetDrawHeight() const { return drawHeight; }
    //! AUTO:  Read (Reference) access to:if beam is drawn with rectangular shape, this is the drawing height
    float& GetDrawHeight() { return drawHeight; }

    //! AUTO:  Write (Reference) access to:RGBA color of the object; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA color of the object; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA color of the object; if R==-1, use default color
    Float4& GetColor() { return color; }

};


