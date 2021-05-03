/** ***********************************************************************************************
* @class        CLoad
* @brief        A general class to handle loads which are applied to markers
*
* @author       Gerstmayr Johannes
* @date         2018-05-18 (generated)
* @date         2019-05-02 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef CLOAD__H
#define CLOAD__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class CLoad //
{
protected: //
  //Index markerNumber;                             //!< pointer to CMarker
  //moved to CLoadForceVector.parameters! Vector loadVector;                              //!< general load vector (e.g. force or torque)

public: //
  //! default constructor with parameter initialization
  //CLoad()
  //{
  //  markerNumber = 0;
  //  //loadVector = Vector({0.,0.,0.});
  //};

  virtual ~CLoad() {} //added for correct deletion of derived classes

  // access functions
  //! clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer
  virtual CLoad* GetClone() const { return new CLoad(*this); }
  
  //! Write (Reference) access to: marker index (defined in specialized class)
  virtual void SetMarkerNumber(Index markerNumberInit) { CHECKandTHROWstring("ERROR: illegal call to CLoad::SetMarkerNumber(...)"); }
  //! Read (Reference) access to: marker index (defined in specialized class)
  virtual Index GetMarkerNumber() const { CHECKandTHROWstring("ERROR: illegal call to CLoad::GetMarkerNumber() const"); return 0; }

  //! determine type of load in order to decide according action in assembly
  virtual LoadType GetType() const { CHECKandTHROWstring("ERROR: illegal call to CLoad::GetType"); return LoadType::_None; }

  const static Index VectorSize = 3; //dimensionality of vector loads
  //! determine type of load in order to decide according action in assembly
  virtual bool IsVector() const { CHECKandTHROWstring("ERROR: illegal call to CLoad::IsVector"); return true; }

  //! per default, forces/torques/... are applied in global coordinates; if IsBodyFixed()=true, the marker needs to provide a rotation (orientation) and forces/torques/... are applied in the local coordinate system
  virtual bool IsBodyFixed() const { return false; }

  //! determine if load has user function, used for Static computations, avoiding to conflict between load user function and loadFactor
  virtual bool HasUserFunction() const { CHECKandTHROWstring("ERROR: illegal call to CLoad::IsVector"); return false; }

  //! Read (Reference) access to: general load vector (e.g. force or torque) as a function of time; used if LoadType::IsVector = 1
  virtual Vector3D GetLoadVector(const MainSystemBase& mbs, Real t) const { CHECKandTHROWstring("ERROR: illegal call to CLoad::GetLoadVector(Real t) const"); Vector3D* v = new Vector3D(0.); return *v;
  }

  //! Write (Reference) access to: scalar load value (e.g. object/node coordinate); used if LoadType::IsVector = 0
  //DELETE: should not be needed: virtual Real& GetLoadValue() { CHECKandTHROWstring("ERROR: illegal call to CLoad::GetLoadValue"); Real* v = new Real(0.); return *v; }
  //! Read (Reference) access to: scalar load value (e.g. object/node coordinate) as a function of time; used if LoadType::IsVector = 0
  virtual Real GetLoadValue(const MainSystemBase& mbs, Real t) const { CHECKandTHROWstring("ERROR: illegal call to CLoad::GetLoadValue(Real t) const");  Real* v = new Real(0.); return *v; }

  virtual Marker::Type GetRequestedMarkerType() const { CHECKandTHROWstring("ERROR: illegal call to CLoad::RequestedMarkerType"); return Marker::_None; }

  virtual void Print(std::ostream& os) const
  {
	  os << "CLoad";
	  os << "  markerNumber = " << GetMarkerNumber() << "\n";
	  //os << "  loadVector = " << loadVector << "\n";
	  os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const CLoad& object)
  {
	  object.Print(os);
	  return os;
  }
};

#endif
