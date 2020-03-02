/** ***********************************************************************************************
* @class        SolverContainer
* @brief        Container for handling all different available solvers
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-02-19 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class SolverContainer // AUTO: 
{
public: // AUTO: 
  SolverRK1 solverRK1;                            //!< AUTO: first order Runge-Kutta solver (explicit Euler)
  SolverStatic solverStatic;                      //!< AUTO: static (non-)linear solver; requires a statically solvable system
  SolverGeneralizedAlpha solverGeneralizedAlpha;  //!< AUTO: second order generalized-alpha, implicit trapezoidal rule or Newmark


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: Read (Reference) access to: first order Runge-Kutta solver (explicit Euler)
  const SolverRK1& GetSolverRK1() const { return solverRK1; }
  //! AUTO: Write (Reference) access to: first order Runge-Kutta solver (explicit Euler)
  SolverRK1& GetSolverRK1() { return solverRK1; }

  //! AUTO: Read (Reference) access to: static (non-)linear solver; requires a statically solvable system
  const SolverStatic& GetSolverStatic() const { return solverStatic; }
  //! AUTO: Write (Reference) access to: static (non-)linear solver; requires a statically solvable system
  SolverStatic& GetSolverStatic() { return solverStatic; }

  //! AUTO: Read (Reference) access to: second order generalized-alpha, implicit trapezoidal rule or Newmark
  const SolverGeneralizedAlpha& GetSolverGeneralizedAlpha() const { return solverGeneralizedAlpha; }
  //! AUTO: Write (Reference) access to: second order generalized-alpha, implicit trapezoidal rule or Newmark
  SolverGeneralizedAlpha& GetSolverGeneralizedAlpha() { return solverGeneralizedAlpha; }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverContainer" << ":\n";
    os << "  solverRK1 = " << solverRK1 << "\n";
    os << "  solverStatic = " << solverStatic << "\n";
    os << "  solverGeneralizedAlpha = " << solverGeneralizedAlpha << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolverContainer& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SystemContainer
* @brief        Container class for all several computable systems (with according AdminSystem); several CSystems could be used in parallel.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-02-19 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class SystemContainer // AUTO: 
{
public: // AUTO: 
  ResizableArray<CSystem*> cSystems;              //!< AUTO: contains one or a set of complete multibody/finite element systems
  SolverContainer solvers;                        //!< AUTO: contains a structure with all solver-relevant structures (dynamic, static, etc.)


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: Read (Reference) access to: contains one or a set of complete multibody/finite element systems
  const ResizableArray<CSystem*>& GetCSystems() const { return cSystems; }
  //! AUTO: Write (Reference) access to: contains one or a set of complete multibody/finite element systems
  ResizableArray<CSystem*>& GetCSystems() { return cSystems; }

  //! AUTO: Read (Reference) access to: contains a structure with all solver-relevant structures (dynamic, static, etc.)
  const SolverContainer& GetSolvers() const { return solvers; }
  //! AUTO: Write (Reference) access to: contains a structure with all solver-relevant structures (dynamic, static, etc.)
  SolverContainer& GetSolvers() { return solvers; }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SystemContainer" << ":\n";
    os << "  cSystems = " << cSystems << "\n";
    os << "  solvers = " << solvers << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SystemContainer& object)
  {
    object.Print(os);
    return os;
  }

};


