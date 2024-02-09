/** ***********************************************************************************************
* @class        VisualizationObjectConnectorSpringDamper
* @brief        An simple spring-damper element with additional force; connects to position-based markers.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-03  15:27:06 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONNECTORSPRINGDAMPER__H
#define VISUALIZATIONOBJECTCONNECTORSPRINGDAMPER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectConnectorSpringDamper: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size = diameter of spring; size == -1.f means that default connector size is used
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorSpringDamper()
    {
        show = true;
        drawSize = -1.f;
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

    //! AUTO:  Write (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    void SetDrawSize(const float& value) { drawSize = value; }
    //! AUTO:  Read (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    const float& GetDrawSize() const { return drawSize; }
    //! AUTO:  Read (Reference) access to:drawing size = diameter of spring; size == -1.f means that default connector size is used
    float& GetDrawSize() { return drawSize; }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
