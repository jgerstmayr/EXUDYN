/** ***********************************************************************************************
* @class        VisualizationObjectBeamGeometricallyExact
* @brief        OBJECT UNDER CONSTRUCTION: A 3D geometrically exact beam finite element, currently using two 3D rigid body nodes. The localPosition \f$x\f$ of the beam ranges from \f$-L/2\f$ (at node 0) to \f$L/2\f$ (at node 1). The axial coordinate is \f$x\f$ (first coordinate) and the cross section is spanned by local \f$y\f$/\f$z\f$ axes.
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

#ifndef VISUALIZATIONOBJECTBEAMGEOMETRICALLYEXACT__H
#define VISUALIZATIONOBJECTBEAMGEOMETRICALLYEXACT__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectBeamGeometricallyExact: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    BeamSectionGeometry sectionGeometry;          //!< AUTO: defines cross section shape used for visualization and contact
    Float4 color;                                 //!< AUTO: RGBA color of the object; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectBeamGeometricallyExact()
    {
        show = true;
        sectionGeometry = BeamSectionGeometry();
        color = Float4({-1.f,-1.f,-1.f,-1.f});
    };

    // AUTO: access functions
    //! AUTO:  Write (Reference) access to:defines cross section shape used for visualization and contact
    void SetSectionGeometry(const BeamSectionGeometry& value) { sectionGeometry = value; }
    //! AUTO:  Read (Reference) access to:defines cross section shape used for visualization and contact
    const BeamSectionGeometry& GetSectionGeometry() const { return sectionGeometry; }
    //! AUTO:  Read (Reference) access to:defines cross section shape used for visualization and contact
    BeamSectionGeometry& GetSectionGeometry() { return sectionGeometry; }

    //! AUTO:  Update visualizationSystem -> graphicsData for item; index shows item Number in CData
    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    //! AUTO:  Write (Reference) access to:RGBA color of the object; if R==-1, use default color
    void SetColor(const Float4& value) { color = value; }
    //! AUTO:  Read (Reference) access to:RGBA color of the object; if R==-1, use default color
    const Float4& GetColor() const { return color; }
    //! AUTO:  Read (Reference) access to:RGBA color of the object; if R==-1, use default color
    Float4& GetColor() { return color; }

};



#endif //#ifdef include once...
