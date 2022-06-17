/** ***********************************************************************************************
* @class        BeamSectionGeometry
* @brief        Data structure for definition of 2D and 3D beam (cross) section geometrical properties. Used for visualization and contact.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2022-06-16 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/

#ifndef BEAMSECTIONGEOMETRY__H
#define BEAMSECTIONGEOMETRY__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class BeamSectionGeometry // AUTO: 
{
public: // AUTO: 
  Real crossSectionRadiusY;                       //!< AUTO: \f$c_Y\,\f$ [SI:m] \f$Y\f$ radius for circular cross section
  Real crossSectionRadiusZ;                       //!< AUTO: \f$c_Z\,\f$ [SI:m] \f$Z\f$ radius for circular cross section
  CrossSectionType crossSectionType;              //!< AUTO: Type of cross section: Polygon, Circular, etc.
  Vector2DList polygonalPoints;                   //!< AUTO: \f$\pv_{pg}\,\f$ [SI: (m,m) ] list of polygonal (\f$Y,Z\f$) points in local beam cross section coordinates, defined in positive rotation direction


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  BeamSectionGeometry()
  {
    crossSectionRadiusY = 0.;
    crossSectionRadiusZ = 0.;
    crossSectionType = CrossSectionType::Polygon;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: \f$c_Y\,\f$ [SI:m] \f$Y\f$ radius for circular cross section
  void PySetCrossSectionRadiusY(const Real& crossSectionRadiusYInit) { crossSectionRadiusY = EXUstd::GetSafelyUReal(crossSectionRadiusYInit,"crossSectionRadiusY"); }
  //! AUTO: Read (Copy) access to: \f$c_Y\,\f$ [SI:m] \f$Y\f$ radius for circular cross section
  Real PyGetCrossSectionRadiusY() const { return Real(crossSectionRadiusY); }

  //! AUTO: Set function (needed in pybind) for: \f$c_Z\,\f$ [SI:m] \f$Z\f$ radius for circular cross section
  void PySetCrossSectionRadiusZ(const Real& crossSectionRadiusZInit) { crossSectionRadiusZ = EXUstd::GetSafelyUReal(crossSectionRadiusZInit,"crossSectionRadiusZ"); }
  //! AUTO: Read (Copy) access to: \f$c_Z\,\f$ [SI:m] \f$Z\f$ radius for circular cross section
  Real PyGetCrossSectionRadiusZ() const { return Real(crossSectionRadiusZ); }

  //! AUTO: Set function (needed in pybind) for: \f$\pv_{pg}\,\f$ [SI: (m,m) ] list of polygonal (\f$Y,Z\f$) points in local beam cross section coordinates, defined in positive rotation direction
  void PySetPolygonalPoints(const PyVector2DList& polygonalPointsInit) { polygonalPoints=(const Vector2DList&)polygonalPointsInit; }
  //! AUTO: Read (Copy) access to: \f$\pv_{pg}\,\f$ [SI: (m,m) ] list of polygonal (\f$Y,Z\f$) points in local beam cross section coordinates, defined in positive rotation direction
  PyVector2DList PyGetPolygonalPoints() const { return PyVector2DList(polygonalPoints); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "BeamSectionGeometry" << ":\n";
    os << "  crossSectionRadiusY = " << crossSectionRadiusY << "\n";
    os << "  crossSectionRadiusZ = " << crossSectionRadiusZ << "\n";
    os << "  polygonalPoints = " << polygonalPoints << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const BeamSectionGeometry& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
