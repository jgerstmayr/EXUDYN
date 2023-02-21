/** ***********************************************************************************************
* @class        PyBeamSection
* @brief        Data structure for definition of 2D and 3D beam (cross) section mechanical properties. The beam has local coordinates, in which \f$X\f$ represents the beam centerline (beam axis) coordinate, being the neutral fiber w.r.t.\ bending; \f$Y\f$ and \f$Z\f$ are the local cross section coordinates. Note that most elements do not accept all parameters, which results in an error if those parameters (e.g., stiffness parameters) are non-zero.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-02-20 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/

#ifndef PYSTRUCTURALELEMENTSDATASTRUCTURES__H
#define PYSTRUCTURALELEMENTSDATASTRUCTURES__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

#include "Main/StructuralElementsDataStructures.h"

class PyBeamSection: public BeamSection // AUTO: 
{

public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  PyBeamSection()
  {
    dampingMatrix = Matrix6D(6,6,0.);
    inertia = EXUmath::zeroMatrix3D;
    massPerLength = 0.;
    stiffnessMatrix = Matrix6D(6,6,0.);
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: \f$\LU{c}{\Dm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nsm\f$^2\f$, Nsm and Ns (mixed)] sectional linear damping matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Dm} \vp{\LU{c}{\tepsDot}}{\LU{c}{\tkappaDot}}\f$; note that this damping models is highly simplified and usually, it cannot be derived from material parameters; however, it can be used to adjust model damping to observed damping behavior.
  void PySetDampingMatrix(const std::array<std::array<Real,6>,6>& dampingMatrixInit) { dampingMatrix=(const Matrix6D&)dampingMatrixInit; }
  //! AUTO: Read (Copy) access to: \f$\LU{c}{\Dm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nsm\f$^2\f$, Nsm and Ns (mixed)] sectional linear damping matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Dm} \vp{\LU{c}{\tepsDot}}{\LU{c}{\tkappaDot}}\f$; note that this damping models is highly simplified and usually, it cannot be derived from material parameters; however, it can be used to adjust model damping to observed damping behavior.
  py::array_t<Real> PyGetDampingMatrix() const { return EPyUtils::Matrix2NumPyTemplate(dampingMatrix); }

  //! AUTO: Set function (needed in pybind) for: \f$\LU{c}{\Jm} \in \Rcal^{3 \times 3}\,\f$ [SI:kg\f$\,\f$m\f$^2\f$] sectional inertia for shear-deformable beams.
  void PySetInertia(const std::array<std::array<Real,3>,3>& inertiaInit) { inertia=(const Matrix3D&)inertiaInit; }
  //! AUTO: Read (Copy) access to: \f$\LU{c}{\Jm} \in \Rcal^{3 \times 3}\,\f$ [SI:kg\f$\,\f$m\f$^2\f$] sectional inertia for shear-deformable beams.
  py::array_t<Real> PyGetInertia() const { return EPyUtils::Matrix2NumPyTemplate(inertia); }

  //! AUTO: Set function (needed in pybind) for: \f$\rho A\,\f$ [SI:kg/m] mass per unit length of the beam
  void PySetMassPerLength(const Real& massPerLengthInit) { massPerLength = EXUstd::GetSafelyUReal(massPerLengthInit,"massPerLength"); }
  //! AUTO: Read (Copy) access to: \f$\rho A\,\f$ [SI:kg/m] mass per unit length of the beam
  Real PyGetMassPerLength() const { return Real(massPerLength); }

  //! AUTO: Set function (needed in pybind) for: \f$\LU{c}{\Cm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nm\f$^2\f$, Nm and N (mixed)] sectional stiffness matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Cm} \vp{\LU{c}{\teps}}{\LU{c}{\tkappa}}\f$ with sectional normal force \f$\LU{c}{\nv}\f$, torque \f$\LU{c}{\mv}\f$, strain \f$\LU{c}{\teps}\f$ and curvature \f$\LU{c}{\tkappa}\f$, all quantities expressed in the cross section frame \f$c\f$.
  void PySetStiffnessMatrix(const std::array<std::array<Real,6>,6>& stiffnessMatrixInit) { stiffnessMatrix=(const Matrix6D&)stiffnessMatrixInit; }
  //! AUTO: Read (Copy) access to: \f$\LU{c}{\Cm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nm\f$^2\f$, Nm and N (mixed)] sectional stiffness matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Cm} \vp{\LU{c}{\teps}}{\LU{c}{\tkappa}}\f$ with sectional normal force \f$\LU{c}{\nv}\f$, torque \f$\LU{c}{\mv}\f$, strain \f$\LU{c}{\teps}\f$ and curvature \f$\LU{c}{\tkappa}\f$, all quantities expressed in the cross section frame \f$c\f$.
  py::array_t<Real> PyGetStiffnessMatrix() const { return EPyUtils::Matrix2NumPyTemplate(stiffnessMatrix); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "PyBeamSection" << ":\n";
    os << ":"; 
    BeamSection::Print(os);
    os << "\n";
  }

};



#endif //#ifdef include once...
