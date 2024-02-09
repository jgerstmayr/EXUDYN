/** ***********************************************************************************************
* @class        VisualizationObjectJointRollingDisc
* @brief        A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global \f$x\f$-\f$y\f$ plane. The contraint is based on an idealized rolling formulation with no slip. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. It must be assured that the disc has contact to ground in the initial configuration (adjust z-position of body accordingly). The ground body can be a rigid body which is moving. In this case, the flat surface is assumed to be in the \f$x\f$-\f$y\f$-plane at \f$z=0\f$. Note that the rolling body must have the reference point at the center of the disc. NOTE: the cases of normal other than \f$z\f$-direction, wheel axis other than \f$x\f$-axis and moving ground body needs to be tested further, check your results!
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-03  15:27:07 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTJOINTROLLINGDISC__H
#define VISUALIZATIONOBJECTJOINTROLLINGDISC__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

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



#endif //#ifdef include once...
