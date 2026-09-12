/** ***********************************************************************************************
* @class        SystemContainer
* @brief        Container class for all several computable systems (with according AdminSystem); several CSystems could be used in parallel.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-05-07 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#ifndef SYSTEMCONTAINER__H
#define SYSTEMCONTAINER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

//remove 2024-01-30:
//class SystemContainer // 
//{
//public: // 
//  ResizableArray<CSystem*> cSystems;              //!< contains one or a set of complete multibody/finite element systems
//
//
//public: // 
//
//  // access functions
//  //! Read (Reference) access to: contains one or a set of complete multibody/finite element systems
//  const ResizableArray<CSystem*>& GetCSystems() const { return cSystems; }
//  //! Write (Reference) access to: contains one or a set of complete multibody/finite element systems
//  ResizableArray<CSystem*>& GetCSystems() { return cSystems; }
//
//  //! print function used in ostream operator (print is virtual and can thus be overloaded)
//  virtual void Print(std::ostream& os) const
//  {
//    os << "SystemContainer" << ":\n";
//    os << "  cSystems = " << cSystems << "\n";
//    os << "\n";
//  }
//
//  friend std::ostream& operator<<(std::ostream& os, const SystemContainer& object)
//  {
//    object.Print(os);
//    return os;
//  }
//
//};

#endif
