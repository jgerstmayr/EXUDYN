/** ***********************************************************************************************
* @class        VisualizationMarkerBody
* @brief        A marker attached to the whole body (mass/volume).
*
* @author       Gerstmayr Johannes
* @date         2018-06-15 (generated)
* @date         2019-07-12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class VisualizationMarkerBody: public VisualizationMarker // AUTO: 
{
protected: // AUTO: 
    bool active;                                  //!< AUTO: set true, if item is shown in visualization and false if it is not shown

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationMarkerBody()
    {
        active = true;
    };

    // AUTO: access functions
    //! AUTO:  Write (Reference) access to:set true, if item is shown in visualization and false if it is not shown
    void SetActive(const bool& value) { active = value; }
    //! AUTO:  Read (Reference) access to:set true, if item is shown in visualization and false if it is not shown
    const bool& GetActive() const { return active; }
    //! AUTO:  Read (Reference) access to:set true, if item is shown in visualization and false if it is not shown
    bool& GetActive() { return active; }

    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

};


