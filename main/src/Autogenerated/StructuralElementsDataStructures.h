/** ***********************************************************************************************
* @class        BeamSection
* @brief        Data structure for definition of 2D and 3D beam (cross) section mechanical properties. The beam has local coordinates, in which \f$X\f$ represents the beam centerline (beam axis) coordinate, being the neutral fiber w.r.t.\ bending; \f$Y\f$ and \f$Z\f$ are the local cross section coordinates. Note that most elements do not accept all parameters, which results in an error if those parameters (e.g., stiffness parameters) are non-zero.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2022-04-20 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/

#ifndef STRUCTURALELEMENTSDATASTRUCTURES__H
#define STRUCTURALELEMENTSDATASTRUCTURES__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class BeamSection // AUTO: 
{
public: // AUTO: 
  Real length;                                    //!< AUTO: \f$l_b\f$ [SI:m] length of beam element
  Real massPerLength;                             //!< AUTO: \f$\rho A\,\f$ [SI:kg/m] mass per unit length of the beam
  Matrix6D sectionalDampingMatrix;                //!< AUTO: \f$\LU{c}{\Dm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nsm\f$^2\f$, Nsm and Ns (mixed)] sectional linear damping matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Dm} \vp{\LU{c}{\tepsDot}}{\LU{c}{\tkappaDot}}\f$; note that this damping models is highly simplified and usually, it cannot be derived from material parameters; however, it can be used to adjust model damping to observed damping behavior.
  Matrix6D sectionalStiffnessMatrix;              //!< AUTO: \f$\LU{c}{\Cm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nm\f$^2\f$, Nm and N (mixed)] sectional stiffness matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Cm} \vp{\LU{c}{\teps}}{\LU{c}{\tkappa}}\f$ with sectional normal force \f$\LU{c}{\nv}\f$, torque \f$\LU{c}{\mv}\f$, strain \f$\LU{c}{\teps}\f$ and curvature \f$\LU{c}{\tkappa}\f$, all quantities expressed in the cross section frame \f$c\f$.


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  BeamSection()
  {
    length = 0.;
    massPerLength = 0.;
    sectionalDampingMatrix = Matrix6D(0.);
    sectionalStiffnessMatrix = Matrix6D(0.);
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: \f$l_b\f$ [SI:m] length of beam element
  void PySetLength(const Real& lengthInit) { length = EXUstd::GetSafelyPReal(lengthInit,"length"); }
  //! AUTO: Read (Copy) access to: \f$l_b\f$ [SI:m] length of beam element
  Real PyGetLength() const { return (Real)(length); }

  //! AUTO: Set function (needed in pybind) for: \f$\rho A\,\f$ [SI:kg/m] mass per unit length of the beam
  void PySetMassPerLength(const Real& massPerLengthInit) { massPerLength = EXUstd::GetSafelyUReal(massPerLengthInit,"massPerLength"); }
  //! AUTO: Read (Copy) access to: \f$\rho A\,\f$ [SI:kg/m] mass per unit length of the beam
  Real PyGetMassPerLength() const { return (Real)(massPerLength); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "BeamSection" << ":\n";
    os << "  length = " << length << "\n";
    os << "  massPerLength = " << massPerLength << "\n";
    os << "  sectionalDampingMatrix = " << sectionalDampingMatrix << "\n";
    os << "  sectionalStiffnessMatrix = " << sectionalStiffnessMatrix << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const BeamSection& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        BeamSectionGeometry
* @brief        Data structure for definition of 2D and 3D beam (cross) section geometrical properties. Used for visualization and contact.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2022-04-20 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
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
  std::vector<Vector2D> polygonalPoints;          //!< AUTO: \f$\pv_{pg}\,\f$ [SI: (m,m) ] list of polygonal (\f$Y,Z\f$) points in local beam cross section coordinates, defined in positive rotation direction


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
  Real PyGetCrossSectionRadiusY() const { return (Real)(crossSectionRadiusY); }

  //! AUTO: Set function (needed in pybind) for: \f$c_Z\,\f$ [SI:m] \f$Z\f$ radius for circular cross section
  void PySetCrossSectionRadiusZ(const Real& crossSectionRadiusZInit) { crossSectionRadiusZ = EXUstd::GetSafelyUReal(crossSectionRadiusZInit,"crossSectionRadiusZ"); }
  //! AUTO: Read (Copy) access to: \f$c_Z\,\f$ [SI:m] \f$Z\f$ radius for circular cross section
  Real PyGetCrossSectionRadiusZ() const { return (Real)(crossSectionRadiusZ); }

  //! AUTO: Set function (needed in pybind) for: \f$\pv_{pg}\,\f$ [SI: (m,m) ] list of polygonal (\f$Y,Z\f$) points in local beam cross section coordinates, defined in positive rotation direction
  void PySetPolygonalPoints(const std::vector<Vector2D>& polygonalPointsInit) { polygonalPoints = polygonalPointsInit; }
  //! AUTO: Read (Copy) access to: \f$\pv_{pg}\,\f$ [SI: (m,m) ] list of polygonal (\f$Y,Z\f$) points in local beam cross section coordinates, defined in positive rotation direction
  std::vector<Vector2D> PyGetPolygonalPoints() const { return (std::vector<Vector2D>)(polygonalPoints); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "BeamSectionGeometry" << ":\n";
    os << "  crossSectionRadiusY = " << crossSectionRadiusY << "\n";
    os << "  crossSectionRadiusZ = " << crossSectionRadiusZ << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const BeamSectionGeometry& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
