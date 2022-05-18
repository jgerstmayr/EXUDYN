/** ***********************************************************************************************
* @class        BeamSection, BeamSectionGeometry
* @brief        Data structure for definition of 2D and 3D beam (cross) section mechanical properties. 
*               This class is the pure cpp class without Python bindings, used in computation and visualization classes
*               For further information see PyStructuralElementsDataStructures.h
*
* @author       Gerstmayr Johannes
* @date         2022-05-08 (created)
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

class BeamSection // 
{
public: // 
	Matrix6D dampingMatrix;                         //!< \f$\LU{c}{\Dm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nsm\f$^2\f$, Nsm and Ns (mixed)] sectional linear damping matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Dm} \vp{\LU{c}{\tepsDot}}{\LU{c}{\tkappaDot}}\f$; note that this damping models is highly simplified and usually, it cannot be derived from material parameters; however, it can be used to adjust model damping to observed damping behavior.
	Matrix3D inertia;                               //!< \f$\LU{c}{\Jm} \in \Rcal^{3 \times 3}\,\f$ [SI:kg\f$\,\f$m\f$^2\f$] sectional inertia for shear-deformable beams.
	Real massPerLength;                             //!< \f$\rho A\,\f$ [SI:kg/m] mass per unit length of the beam
	Matrix6D stiffnessMatrix;                       //!< \f$\LU{c}{\Cm} \in \Rcal^{6 \times 6}\,\f$ [SI:Nm\f$^2\f$, Nm and N (mixed)] sectional stiffness matrix related to \f$\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Cm} \vp{\LU{c}{\teps}}{\LU{c}{\tkappa}}\f$ with sectional normal force \f$\LU{c}{\nv}\f$, torque \f$\LU{c}{\mv}\f$, strain \f$\LU{c}{\teps}\f$ and curvature \f$\LU{c}{\tkappa}\f$, all quantities expressed in the cross section frame \f$c\f$.


public: // 
  //! default constructor with parameter initialization
	BeamSection()
	{
		dampingMatrix = Matrix6D(0.);
		inertia = Matrix3D(0.);
		massPerLength = 0.;
		stiffnessMatrix = Matrix6D(0.);
	};

	//! print function used in ostream operator (print is virtual and can thus be overloaded)
	virtual void Print(std::ostream& os) const
	{
		os << "BeamSection" << ":\n";
		os << "  dampingMatrix = " << dampingMatrix << "\n";
		os << "  inertia = " << inertia << "\n";
		os << "  massPerLength = " << massPerLength << "\n";
		os << "  stiffnessMatrix = " << stiffnessMatrix << "\n";
		os << "\n";
	}

	//! ostream operator also used in derived object!
	friend std::ostream& operator<<(std::ostream& os, const BeamSection& object)
	{
		object.Print(os);
		return os;
	}

};

//class BeamSectionGeometry
//{
//public: 
//	Real crossSectionRadiusY;                       //!< \f$c_Y\,\f$ [SI:m] \f$Y\f$ radius for circular cross section
//	Real crossSectionRadiusZ;                       //!< \f$c_Z\,\f$ [SI:m] \f$Z\f$ radius for circular cross section
//	CrossSectionType crossSectionType;              //!< Type of cross section: Polygon, Circular, etc.
//	Vector2DList polygonalPoints;                   //!< \f$\pv_{pg}\,\f$ [SI: (m,m) ] list of polygonal (\f$Y,Z\f$) points in local beam cross section coordinates, defined in positive rotation direction
//
//
//public: // 
//  //! default constructor with parameter initialization
//	BeamSectionGeometry()
//	{
//		crossSectionRadiusY = 0.;
//		crossSectionRadiusZ = 0.;
//		crossSectionType = CrossSectionType::Polygon;
//	};
//
//	//! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
//	virtual void Print(std::ostream& os) const
//	{
//		os << "BeamSectionGeometry" << ":\n";
//		os << "  crossSectionRadiusY = " << crossSectionRadiusY << "\n";
//		os << "  crossSectionRadiusZ = " << crossSectionRadiusZ << "\n";
//		os << "  polygonalPoints = " << polygonalPoints << "\n";
//		os << "\n";
//	}
//
//	//! ostream operator also used in derived object!
//	friend std::ostream& operator<<(std::ostream& os, const BeamSectionGeometry& object)
//	{
//		object.Print(os);
//		return os;
//	}
//
//};



#endif //#ifdef include once...
