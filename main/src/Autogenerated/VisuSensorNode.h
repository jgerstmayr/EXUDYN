/** ***********************************************************************************************
* @class        VisualizationSensorNode
* @brief        A sensor attached to a node. The sensor outputs values into a file, showing time, sensorValue[0], sensorValue[1], ... . A user function can be attached to modify sensor values accordingly.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-01-24  13:27:00 (last modfied)
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

class VisualizationSensorNode: public VisualizationSensor // AUTO: 
{
protected: // AUTO: 

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationSensorNode()
    {
        show = true;
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override
    {
        ;
    }

};


