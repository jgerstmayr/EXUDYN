/** ***********************************************************************************************
* @class        VisualizationObjectANCFCable
* @brief        A 3D cable finite element using 2 nodes of type NodePointSlope1. The localPosition of the beam with length \f$L\f$=physicsLength and height \f$h\f$ ranges in \f$X\f$-direction in range \f$[0, L]\f$ and in \f$Y\f$-direction in range \f$[-h/2,h/2]\f$ (which is in fact not needed in the \hac{EOM}). For description see ObjectANCFCable2D, which is almost identical to 3D case. Note that this element does not include torsion, therfore a torque cannot be applied along the local x-axis.
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

#ifndef VISUALIZATIONOBJECTANCFCABLE__H
#define VISUALIZATIONOBJECTANCFCABLE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectANCFCable: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    float radius;                                 //!< AUTO: if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling
    Float4 color;                                 //!< AUTO: RGBA color of the object; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectANCFCable()
    {
        show = true;
        radius = 0.f;
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  Write (Reference) access to:if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling
    void SetRadius(const float& value) { radius = value; }
    //! AUTO:  Read (Reference) access to:if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling
    const float& GetRadius() const { return radius; }
    //! AUTO:  Read (Reference) access to:if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling
    float& GetRadius() { return radius; }

    //! AUTO:  Write (Reference) access to:RGBA color of the object; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA color of the object; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA color of the object; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
