/** ***********************************************************************************************
* @class        VisualizationObjectANCFBeam3D
* @brief        OBJECT UNDER CONSTRUCTION: A 3D beam finite element based on the absolute nodal coordinate formulation, using two . The localPosition \f$x\f$ of the beam ranges from \f$-L/2\f$ (at node 0) to \f$L/2\f$ (at node 1). The axial coordinate is \f$x\f$ (first coordinate) and the cross section is spanned by local \f$y\f$/\f$z\f$ axes; assuming dimensions \f$w_y\f$ and \f$w_z\f$ in cross section, the local position range is \f$\in [[-L/2,L/2],\, [-wy/2,wy/2],\, [-wz/2,wz/2] ]\f$.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-10-17  16:28:10 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTANCFBEAM3D__H
#define VISUALIZATIONOBJECTANCFBEAM3D__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationObjectANCFBeam3D: public VisualizationObject // AUTO: 
{
protected: // AUTO: 
    BeamSectionGeometry sectionGeometry;          //!< AUTO: defines cross section shape used for visualization and contact
    Float4 color;                                 //!< AUTO: RGBA color of the object; if R==-1, use default color

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    VisualizationObjectANCFBeam3D()
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
