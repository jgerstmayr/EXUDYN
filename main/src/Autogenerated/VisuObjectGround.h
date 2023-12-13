/** ***********************************************************************************************
* @class        VisualizationObjectGround
* @brief        A ground object behaving like a rigid body, but having no degrees of freedom; used to attach body-connectors without an action. For examples see spring dampers and joints.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2023-12-03  23:26:45 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTGROUND__H
#define VISUALIZATIONOBJECTGROUND__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectGround: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    std::function<py::object(const MainSystem&,Index)> graphicsDataUserFunction;//!< AUTO: A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function
    Float4 color;                                 //!< AUTO: RGB node color; if R==-1, use default color
    BodyGraphicsData graphicsData;                //!< AUTO: Structure contains data for body visualization; data is defined in special list / dictionary structure

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectGround()
    {
        show = true;
        graphicsDataUserFunction = 0;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  user function which is called to update specific object graphics computed in Python functions; this is rather slow, but useful for user elements
    virtual void CallUserFunction(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, const MainSystem& mainSystem, Index itemNumber) override;

    //! AUTO:  return true, if object has a user function to be called during redraw
    virtual bool HasUserFunction() const override
    {
        return graphicsDataUserFunction!=0;
    }

    //! AUTO:  Write (Reference) access to:A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function
    void SetGraphicsDataUserFunction(const std::function<py::object(const MainSystem&,Index)>& value) { graphicsDataUserFunction = value; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function
    const std::function<py::object(const MainSystem&,Index)>& GetGraphicsDataUserFunction() const { return graphicsDataUserFunction; }
    //! AUTO:  Read (Reference) access to:A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function
    std::function<py::object(const MainSystem&,Index)>& GetGraphicsDataUserFunction() { return graphicsDataUserFunction; }

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



#endif //#ifdef include once...
