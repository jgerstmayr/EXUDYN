/** ***********************************************************************************************
* @class        VisualizationObjectConnectorDistance
* @brief        Connector which enforces constant or prescribed distance between two bodies/nodes.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-04  17:26:47 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONNECTORDISTANCE__H
#define VISUALIZATIONOBJECTCONNECTORDISTANCE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectConnectorDistance: public VisualizationObject // AUTO:
{
protected: // AUTO:
    float drawSize;                               //!< AUTO: the diameter of the rod drawn between the markers if visualizationSettings.connectors.drawSimplified is False; -1 means a tenth of connectors.defaultSize; with drawSimplified, the connector is a line
    Float4 color;                                 //!< AUTO: RGBA connector color; if R==-1, use default color

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorDistance()
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

    //! AUTO:  Write (Reference) access to:the diameter of the rod drawn between the markers if visualizationSettings.connectors.drawSimplified is False; -1 means a tenth of connectors.defaultSize; with drawSimplified, the connector is a line
    void SetDrawSize(const float& value) { drawSize = value; }
    //! AUTO:  Read (Reference) access to:the diameter of the rod drawn between the markers if visualizationSettings.connectors.drawSimplified is False; -1 means a tenth of connectors.defaultSize; with drawSimplified, the connector is a line
    const float& GetDrawSize() const { return drawSize; }
    //! AUTO:  Read (Reference) access to:the diameter of the rod drawn between the markers if visualizationSettings.connectors.drawSimplified is False; -1 means a tenth of connectors.defaultSize; with drawSimplified, the connector is a line
    float& GetDrawSize() { return drawSize; }

    //! AUTO:  Write (Reference) access to:RGBA connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
