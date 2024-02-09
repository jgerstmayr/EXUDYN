/** ***********************************************************************************************
* @class        VisualizationObjectConnectorHydraulicActuatorSimple
* @brief        A basic hydraulic actuator with pressure build up equations. The actuator follows a valve input value, which results in a in- or outflow of fluid depending on the pressure difference. Valve values can be prescribed by user functions (not yet available) or with the MainSystem PreStepUserFunction(...).
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

#ifndef VISUALIZATIONOBJECTCONNECTORHYDRAULICACTUATORSIMPLE__H
#define VISUALIZATIONOBJECTCONNECTORHYDRAULICACTUATORSIMPLE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectConnectorHydraulicActuatorSimple: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float cylinderRadius;                         //!< AUTO: radius for drawing of cylinder
    float rodRadius;                              //!< AUTO: radius for drawing of rod
    float pistonRadius;                           //!< AUTO: radius for drawing of piston (if drawn transparent)
    float pistonLength;                           //!< AUTO: radius for drawing of piston (if drawn transparent)
    float rodMountRadius;                         //!< AUTO: radius for drawing of rod mount sphere
    float baseMountRadius;                        //!< AUTO: radius for drawing of base mount sphere
    float baseMountLength;                        //!< AUTO: radius for drawing of base mount sphere
    Float4 colorCylinder;                         //!< AUTO: RGBA cylinder color; if R==-1, use default connector color
    Float4 colorPiston;                           //!< AUTO: RGBA piston color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorHydraulicActuatorSimple()
    {
        show = true;
        cylinderRadius = 0.05f;
        rodRadius = 0.03f;
        pistonRadius = 0.04f;
        pistonLength = 0.001f;
        rodMountRadius = 0.0f;
        baseMountRadius = 0.0f;
        baseMountLength = 0.0f;
        colorCylinder = Float4({-1.f,-1.f,-1.f,-1.f});
        colorPiston = Float4({0.8f,0.8f,0.8f,1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  this function is needed to distinguish connector objects from body objects
    virtual bool IsConnector() const override
    {
        return true;
    }

    //! AUTO:  Write (Reference) access to:radius for drawing of cylinder
    void SetCylinderRadius(const float& value) { cylinderRadius = value; }
    //! AUTO:  Read (Reference) access to:radius for drawing of cylinder
    const float& GetCylinderRadius() const { return cylinderRadius; }
    //! AUTO:  Read (Reference) access to:radius for drawing of cylinder
    float& GetCylinderRadius() { return cylinderRadius; }

    //! AUTO:  Write (Reference) access to:radius for drawing of rod
    void SetRodRadius(const float& value) { rodRadius = value; }
    //! AUTO:  Read (Reference) access to:radius for drawing of rod
    const float& GetRodRadius() const { return rodRadius; }
    //! AUTO:  Read (Reference) access to:radius for drawing of rod
    float& GetRodRadius() { return rodRadius; }

    //! AUTO:  Write (Reference) access to:radius for drawing of piston (if drawn transparent)
    void SetPistonRadius(const float& value) { pistonRadius = value; }
    //! AUTO:  Read (Reference) access to:radius for drawing of piston (if drawn transparent)
    const float& GetPistonRadius() const { return pistonRadius; }
    //! AUTO:  Read (Reference) access to:radius for drawing of piston (if drawn transparent)
    float& GetPistonRadius() { return pistonRadius; }

    //! AUTO:  Write (Reference) access to:radius for drawing of piston (if drawn transparent)
    void SetPistonLength(const float& value) { pistonLength = value; }
    //! AUTO:  Read (Reference) access to:radius for drawing of piston (if drawn transparent)
    const float& GetPistonLength() const { return pistonLength; }
    //! AUTO:  Read (Reference) access to:radius for drawing of piston (if drawn transparent)
    float& GetPistonLength() { return pistonLength; }

    //! AUTO:  Write (Reference) access to:radius for drawing of rod mount sphere
    void SetRodMountRadius(const float& value) { rodMountRadius = value; }
    //! AUTO:  Read (Reference) access to:radius for drawing of rod mount sphere
    const float& GetRodMountRadius() const { return rodMountRadius; }
    //! AUTO:  Read (Reference) access to:radius for drawing of rod mount sphere
    float& GetRodMountRadius() { return rodMountRadius; }

    //! AUTO:  Write (Reference) access to:radius for drawing of base mount sphere
    void SetBaseMountRadius(const float& value) { baseMountRadius = value; }
    //! AUTO:  Read (Reference) access to:radius for drawing of base mount sphere
    const float& GetBaseMountRadius() const { return baseMountRadius; }
    //! AUTO:  Read (Reference) access to:radius for drawing of base mount sphere
    float& GetBaseMountRadius() { return baseMountRadius; }

    //! AUTO:  Write (Reference) access to:radius for drawing of base mount sphere
    void SetBaseMountLength(const float& value) { baseMountLength = value; }
    //! AUTO:  Read (Reference) access to:radius for drawing of base mount sphere
    const float& GetBaseMountLength() const { return baseMountLength; }
    //! AUTO:  Read (Reference) access to:radius for drawing of base mount sphere
    float& GetBaseMountLength() { return baseMountLength; }

    //! AUTO:  Write (Reference) access to:RGBA cylinder color; if R==-1, use default connector color
    void SetColorCylinder(const Float4& value) { colorCylinder = value; }
    //! AUTO:  Read (Reference) access to:RGBA cylinder color; if R==-1, use default connector color
    const Float4& GetColorCylinder() const { return colorCylinder; }
    //! AUTO:  Read (Reference) access to:RGBA cylinder color; if R==-1, use default connector color
    Float4& GetColorCylinder() { return colorCylinder; }

    //! AUTO:  Write (Reference) access to:RGBA piston color
    void SetColorPiston(const Float4& value) { colorPiston = value; }
    //! AUTO:  Read (Reference) access to:RGBA piston color
    const Float4& GetColorPiston() const { return colorPiston; }
    //! AUTO:  Read (Reference) access to:RGBA piston color
    Float4& GetColorPiston() { return colorPiston; }

};



#endif //#ifdef include once...
