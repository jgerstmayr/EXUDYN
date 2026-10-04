/** ***********************************************************************************************
* @class        VisualizationObjectConnectorCoordinateVector
* @brief        A constraint which constrains the coordinate vectors of two markers Marker[Node|Object|Body]Coordinates attached to nodes or bodies. The marker uses the objects ABRV:LTG-lists to build the according coordinate mappings.
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

#ifndef VISUALIZATIONOBJECTCONNECTORCOORDINATEVECTOR__H
#define VISUALIZATIONOBJECTCONNECTORCOORDINATEVECTOR__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectConnectorCoordinateVector: public VisualizationObject // AUTO:
{
protected: // AUTO:

public: // AUTO:

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override
    {
        ;
    }

    //! AUTO:  this function is needed to distinguish connector objects from body objects
    virtual bool IsConnector() const override
    {
        return true;
    }

};



#endif //#ifdef include once...
