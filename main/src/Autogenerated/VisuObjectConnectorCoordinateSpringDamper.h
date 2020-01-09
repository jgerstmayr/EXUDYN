/** ***********************************************************************************************
* @class        VisualizationObjectConnectorCoordinateSpringDamper
* @brief        A 1D (scalar) spring-damper element acting on single ODE2 coordinates; connects to coordinate-based markers; NOTE that the coordinate markers only measure the coordinate (=displacement), but the reference position is not included as compared to position-based markers!; the spring-damper can also act on rotational coordinates; the resulting force in the spring-damper reads (\f$m0 = marker[0]\f$ and \f$m1 = marker[1]\f$): \f[ f = (m1.coordinate - m0.coordinate - offset)\cdot stiffness + (m1.coordinate_t - m0.coordinate_t)\cdot damping \f] If dry (Coulomb) friction is non-zero, an additional term \f[ \text{sign}(m1.coordinate_t - m0.coordinate_t)\cdot dryFriction \f] is added to the force \f$f\f$.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-12-30  21:32:20 (last modfied)
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

class VisualizationObjectConnectorCoordinateSpringDamper: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float drawSize;                               //!< AUTO: drawing size = diameter of spring; size == -1.f means that default connector size is used
    Float4 color;                                 //!< AUTO: RGB connector color; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectConnectorCoordinateSpringDamper()
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

    //! AUTO:  Write (Reference) access to:RGB connector color; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGB connector color; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGB connector color; if R==-1, use default color
    Float4& GetColor() { return color; }

};


