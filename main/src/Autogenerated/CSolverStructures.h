/** ***********************************************************************************************
* @class        CSolverTimer
* @brief        Structure for timing in solver. Each Real variable is used to measure the CPU time which certain parts of the solver need. This structure is only active if the code is not compiled with the __FAST_EXUDYN_LINALG option and if displayComputationTime is set True. Timings will only be filled, if useTimer is True.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-07-20 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/

#ifndef CSOLVERSTRUCTURES__H
#define CSOLVERSTRUCTURES__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class CSolverTimer // AUTO: 
{
public: // AUTO: 
  bool useTimer;                                  //!< AUTO: flag to decide, whether the timer is used (true) or not
  Real total;                                     //!< AUTO: total time measured between start and end of computation (static/dynamics)
  Real factorization;                             //!< AUTO: solve or inverse
  Real newtonIncrement;                           //!< AUTO: Jac\f$^{-1}\f$ * RHS; backsubstitution
  Real integrationFormula;                        //!< AUTO: time spent for evaluation of integration formulas
  Real ODE2RHS;                                   //!< AUTO: time for residual evaluation of ODE2 right-hand-side
  Real AERHS;                                     //!< AUTO: time for residual evaluation of algebraic equations right-hand-side
  Real totalJacobian;                             //!< AUTO: time for all jacobian computations
  Real jacobianODE2;                              //!< AUTO: jacobian w.r.t. coordinates of ODE2 equations (not counted in sum)
  Real jacobianODE2_t;                            //!< AUTO: jacobian w.r.t. coordinates\_t of ODE2 equations (not counted in sum)
  Real jacobianAE;                                //!< AUTO: jacobian of algebraic equations (not counted in sum)
  Real massMatrix;                                //!< AUTO: mass matrix computation
  Real reactionForces;                            //!< AUTO: CqT * lambda
  Real postNewton;                                //!< AUTO: post newton step
  Real writeSolution;                             //!< AUTO: time for writing solution
  Real overhead;                                  //!< AUTO: overhead, such as initialization, copying and some matrix-vector multiplication
  Real python;                                    //!< AUTO: time spent for python functions
  Real visualization;                             //!< AUTO: time spent for visualization in computation thread


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  CSolverTimer()
  {
    useTimer = true;
    total = 0.;
    factorization = 0.;
    newtonIncrement = 0.;
    integrationFormula = 0.;
    ODE2RHS = 0.;
    AERHS = 0.;
    totalJacobian = 0.;
    jacobianODE2 = 0.;
    jacobianODE2_t = 0.;
    jacobianAE = 0.;
    massMatrix = 0.;
    reactionForces = 0.;
    postNewton = 0.;
    writeSolution = 0.;
    overhead = 0.;
    python = 0.;
    visualization = 0.;
  };

  // AUTO: access functions
  //! AUTO: reset solver timings to initial state by assigning default values; useSolverTimer sets the useTimer flag
  void Reset(bool useSolverTimer) {
    *this = CSolverTimer(); useTimer = useSolverTimer;
  }

  //! AUTO: compute sum of all timers (except for those counted multiple, e.g., jacobians
  Real Sum() const;
  //! AUTO: start timer function for a given variable; subtracts current CPU time from value
  void StartTimer(Real& value);
  //! AUTO: stop timer function for a given variable; adds current CPU time to value
  void StopTimer(Real& value);
  //! AUTO: converts the current timings to a string
  std::string ToString() const;
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "CSolverTimer" << ":\n";
    os << "  useTimer = " << useTimer << "\n";
    os << "  total = " << total << "\n";
    os << "  factorization = " << factorization << "\n";
    os << "  newtonIncrement = " << newtonIncrement << "\n";
    os << "  integrationFormula = " << integrationFormula << "\n";
    os << "  ODE2RHS = " << ODE2RHS << "\n";
    os << "  AERHS = " << AERHS << "\n";
    os << "  totalJacobian = " << totalJacobian << "\n";
    os << "  jacobianODE2 = " << jacobianODE2 << "\n";
    os << "  jacobianODE2_t = " << jacobianODE2_t << "\n";
    os << "  jacobianAE = " << jacobianAE << "\n";
    os << "  massMatrix = " << massMatrix << "\n";
    os << "  reactionForces = " << reactionForces << "\n";
    os << "  postNewton = " << postNewton << "\n";
    os << "  writeSolution = " << writeSolution << "\n";
    os << "  overhead = " << overhead << "\n";
    os << "  python = " << python << "\n";
    os << "  visualization = " << visualization << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const CSolverTimer& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolverLocalData
* @brief        Solver local data structure for solution vectors, system matrices and temporary vectors and data structures.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-07-20 (last modfied)
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

class SolverLocalData // AUTO: 
{
public: // AUTO: 
  Index nODE2;                                    //!< AUTO: number of second order ordinary diff. eq. coordinates
  Index nODE1;                                    //!< AUTO: number of first order ordinary diff. eq. coordinates
  Index nAE;                                      //!< AUTO: number of algebraic coordinates
  Index nData;                                    //!< AUTO: number of data coordinates
  Index nSys;                                     //!< AUTO: number of system (unknown) coordinates = nODE2+nODE1+nAE
  Index startAE;                                  //!< AUTO: start of algebraic coordinates, but set to zero if nAE==0
  ResizableVector systemResidual;                 //!< AUTO: system residual vector (vectors will be linked to this vector!)
  ResizableVector newtonSolution;                 //!< AUTO: Newton decrement (computed from residual and jacobian)
  ResizableVector tempODE2;                       //!< AUTO: temporary vector for ODE2 quantities; use in initial accelerations and during Newton
  ResizableVector temp2ODE2;                      //!< AUTO: second temporary vector for ODE2 quantities; use in static computation
  ResizableVector tempODE2F0;                     //!< AUTO: temporary vector for ODE2 Jacobian
  ResizableVector tempODE2F1;                     //!< AUTO: temporary vector for ODE2 Jacobian
  ResizableVector startOfStepStateAAlgorithmic;   //!< AUTO: additional term needed for generalized alpha (startOfStep state)
  ResizableVector aAlgorithmic;                   //!< AUTO: additional term needed for generalized alpha (current state)
  GeneralMatrix* systemJacobian;                  //!< AUTO: link to dense or sparse system jacobian
  GeneralMatrix* systemMassMatrix;                //!< AUTO: link to dense or sparse system matrix
  GeneralMatrix* jacobianAE;                      //!< AUTO: link to dense or sparse algebraic equations jacobian
  TemporaryComputationData tempCompData;          //!< AUTO: temporary data used during item-related residual and jacobian computation; duplicated for parallel computation

private: // AUTO: 
  LinearSolverType linearSolverType;              //!< AUTO: contains linear solver type value; cannot be accessed directly, because a change requires new linking of system matrices
  GeneralMatrixEXUdense systemJacobianDense;      //!< AUTO: dense system jacobian
  GeneralMatrixEXUdense systemMassMatrixDense;    //!< AUTO: dense mass matrix
  GeneralMatrixEXUdense jacobianAEdense;          //!< AUTO: dense AE jacobian
  GeneralMatrixEigenSparse systemJacobianSparse;  //!< AUTO: sparse system jacobian
  GeneralMatrixEigenSparse systemMassMatrixSparse;//!< AUTO: sparse mass matrix
  GeneralMatrixEigenSparse jacobianAEsparse;      //!< AUTO: sparse AE jacobian


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverLocalData()
  {
    nODE2 = 0;
    nODE1 = 0;
    nAE = 0;
    nData = 0;
    nSys = 0;
    startAE = 0;
    systemJacobian = nullptr;
    systemMassMatrix = nullptr;
    jacobianAE = nullptr;
    SetLinearSolverType(LinearSolverType::EXUdense); //for safety, data is linked initially
  };

  // AUTO: access functions
  //! AUTO: if desired, temporary data is cleaned up to safe memory
  void CleanUpMemory();
  //! AUTO: set linear solver type and matrix version: links system matrices to according dense/sparse versions
  void SetLinearSolverType(LinearSolverType linearSolverType);
  //! AUTO: return current linear solver type (dense/sparse)
  LinearSolverType GetLinearSolverType() const {
    return linearSolverType;
  }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverLocalData" << ":\n";
    os << "  nODE2 = " << nODE2 << "\n";
    os << "  nODE1 = " << nODE1 << "\n";
    os << "  nAE = " << nAE << "\n";
    os << "  nData = " << nData << "\n";
    os << "  nSys = " << nSys << "\n";
    os << "  startAE = " << startAE << "\n";
    os << "  systemJacobian = " << *systemJacobian << "\n";
    os << "  systemMassMatrix = " << *systemMassMatrix << "\n";
    os << "  jacobianAE = " << *jacobianAE << "\n";
    os << "  systemResidual = " << systemResidual << "\n";
    os << "  newtonSolution = " << newtonSolution << "\n";
    os << "  tempODE2 = " << tempODE2 << "\n";
    os << "  temp2ODE2 = " << temp2ODE2 << "\n";
    os << "  tempODE2F0 = " << tempODE2F0 << "\n";
    os << "  tempODE2F1 = " << tempODE2F1 << "\n";
    os << "  startOfStepStateAAlgorithmic = " << startOfStepStateAAlgorithmic << "\n";
    os << "  aAlgorithmic = " << aAlgorithmic << "\n";
    os << "  linearSolverType = " << linearSolverType << "\n";
    os << "  systemJacobianDense = " << systemJacobianDense << "\n";
    os << "  systemMassMatrixDense = " << systemMassMatrixDense << "\n";
    os << "  jacobianAEdense = " << jacobianAEdense << "\n";
    os << "  systemJacobianSparse = " << systemJacobianSparse << "\n";
    os << "  systemMassMatrixSparse = " << systemMassMatrixSparse << "\n";
    os << "  jacobianAEsparse = " << jacobianAEsparse << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolverLocalData& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolverIterationData
* @brief        Solver internal structure for counters, steps, step size, time, etc.; solution vectors, residuals, etc. are SolverLocalData. The given default values are overwritten by the simulationSettings when initializing the solver.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-07-20 (last modfied)
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

class SolverIterationData // AUTO: 
{
public: // AUTO: 
  Real maxStepSize;                               //!< AUTO: constant or maximum stepSize
  Real minStepSize;                               //!< AUTO: minimum stepSize for static/dynamic solver; only used, if adaptive step is activated
  Real currentStepSize;                           //!< AUTO: stepSize of current step
  Index numberOfSteps;                            //!< AUTO: number of time steps (if fixed size); \f$n\f$
  Index currentStepIndex;                         //!< AUTO: current step index; \f$i\f$
  bool adaptiveStep;                              //!< AUTO: if true, the step size may be adaptively controlled
  Real currentTime;                               //!< AUTO: holds the current simulation time, copy of state.current.time; interval is [startTime,tEnd]; in static solver, duration is loadStepDuration
  Real startTime;                                 //!< AUTO: time at beginning of time integration
  Real endTime;                                   //!< AUTO: end time of static/dynamic solver
  Index discontinuousIteration;                   //!< AUTO: number of current discontinuous iteration
  Index newtonSteps;                              //!< AUTO: number of current newton steps
  Index newtonStepsCount;                         //!< AUTO: count total Newton steps
  Index newtonJacobiCount;                        //!< AUTO: count total Newton jacobian computations
  Index rejectedModifiedNewtonSteps;              //!< AUTO: count the number of rejected modified Newton steps (switch to full Newton)
  Index discontinuousIterationsCount;             //!< AUTO: count total number of discontinuous iterations (min. 1 per step)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverIterationData()
  {
    maxStepSize = 0.;
    minStepSize = 0.;
    currentStepSize = 0.;
    numberOfSteps = 0;
    currentStepIndex = 0;
    adaptiveStep = true;
    currentTime = 0.;
    startTime = 0.;
    endTime = 0.;
    discontinuousIteration = 0;
    newtonSteps = 0;
    newtonStepsCount = 0;
    newtonJacobiCount = 0;
    rejectedModifiedNewtonSteps = 0;
    discontinuousIterationsCount = 0;
  };

  // AUTO: access functions
  //! AUTO: convert iteration statistics to string; used for displayStatistics option
  std::string ToString() const;
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverIterationData" << ":\n";
    os << "  maxStepSize = " << maxStepSize << "\n";
    os << "  minStepSize = " << minStepSize << "\n";
    os << "  currentStepSize = " << currentStepSize << "\n";
    os << "  numberOfSteps = " << numberOfSteps << "\n";
    os << "  currentStepIndex = " << currentStepIndex << "\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  currentTime = " << currentTime << "\n";
    os << "  startTime = " << startTime << "\n";
    os << "  endTime = " << endTime << "\n";
    os << "  discontinuousIteration = " << discontinuousIteration << "\n";
    os << "  newtonSteps = " << newtonSteps << "\n";
    os << "  newtonStepsCount = " << newtonStepsCount << "\n";
    os << "  newtonJacobiCount = " << newtonJacobiCount << "\n";
    os << "  rejectedModifiedNewtonSteps = " << rejectedModifiedNewtonSteps << "\n";
    os << "  discontinuousIterationsCount = " << discontinuousIterationsCount << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolverIterationData& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolverConvergenceData
* @brief        Solver internal structure for convergence information: residua, iteration loop errors and error flags. For detailed behavior of these flags, visit the source code!
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-07-20 (last modfied)
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

class SolverConvergenceData // AUTO: 
{
public: // AUTO: 
  bool stepReductionFailed;                       //!< AUTO: true, if iterations over time/static steps failed (finally, cannot be recovered)
  bool discontinuousIterationsFailed;             //!< AUTO: true, if discontinuous iterations failed (may be recovered if adaptive step is active)
  bool linearSolverFailed;                        //!< AUTO: true, if linear solver failed to factorize
  bool newtonConverged;                           //!< AUTO: true, if Newton has (finally) converged
  bool newtonSolutionDiverged;                    //!< AUTO: true, if Newton diverged (may be recovered)
  bool jacobianUpdateRequested;                   //!< AUTO: true, if a jacobian update is requested in modified Newton (determined in previous step)
  bool massMatrixNotInvertible;                   //!< AUTO: true, if mass matrix is not invertable during initialization or solution (explicit solver)
  Real discontinuousIterationError;               //!< AUTO: error of discontinuous iterations (contact, friction, ...) outside of Newton iteration
  Real residual;                                  //!< AUTO: current Newton residual
  Real lastResidual;                              //!< AUTO: last Newton residual to determine contractivity
  Real contractivity;                             //!< AUTO: Newton contractivity = geometric decay of error in every step
  Real errorCoordinateFactor;                     //!< AUTO: factor may include the number of system coordinates to reduce the residual


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverConvergenceData()
  {
    stepReductionFailed = false;
    discontinuousIterationsFailed = false;
    linearSolverFailed = false;
    newtonConverged = false;
    newtonSolutionDiverged = false;
    jacobianUpdateRequested = true;
    massMatrixNotInvertible = true;
    discontinuousIterationError = 0.;
    residual = 0.;
    lastResidual = 0.;
    contractivity = 0.;
    errorCoordinateFactor = 1.;
  };

  // AUTO: access functions
  //! AUTO: initialize SolverConvergenceData by assigning default values
  void InitializeData() {
    *this = SolverConvergenceData();
  }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverConvergenceData" << ":\n";
    os << "  stepReductionFailed = " << stepReductionFailed << "\n";
    os << "  discontinuousIterationsFailed = " << discontinuousIterationsFailed << "\n";
    os << "  linearSolverFailed = " << linearSolverFailed << "\n";
    os << "  newtonConverged = " << newtonConverged << "\n";
    os << "  newtonSolutionDiverged = " << newtonSolutionDiverged << "\n";
    os << "  jacobianUpdateRequested = " << jacobianUpdateRequested << "\n";
    os << "  massMatrixNotInvertible = " << massMatrixNotInvertible << "\n";
    os << "  discontinuousIterationError = " << discontinuousIterationError << "\n";
    os << "  residual = " << residual << "\n";
    os << "  lastResidual = " << lastResidual << "\n";
    os << "  contractivity = " << contractivity << "\n";
    os << "  errorCoordinateFactor = " << errorCoordinateFactor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolverConvergenceData& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolverOutputData
* @brief        Solver internal structure for output modes, output timers and counters.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-07-20 (last modfied)
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

class SolverOutputData // AUTO: 
{
public: // AUTO: 
  bool finishedSuccessfully;                      //!< AUTO: flag is false until solver finshed successfully (can be used as external trigger)
  Index verboseMode;                              //!< AUTO: this is a copy of the solvers verboseMode used for console output
  Index verboseModeFile;                          //!< AUTO: this is a copy of the solvers verboseModeFile used for file
  bool writeToSolutionFile;                       //!< AUTO: if false, no solution file is generated and no file is written
  bool writeToSolverFile;                         //!< AUTO: if false, no solver output file is generated and no file is written
  ResizableVector sensorValuesTemp;               //!< AUTO: temporary vector for per sensor values (overwritten for every sensor; usually contains last sensor)
  Real lastSolutionWritten;                       //!< AUTO: simulation time when last solution has been written
  Real lastSensorsWritten;                        //!< AUTO: simulation time when last sensors have been written
  Real lastImageRecorded;                         //!< AUTO: simulation time when last image has been recorded
  Real cpuStartTime;                              //!< AUTO: CPU start time of computation (starts counting at computation of initial conditions)
  Real cpuLastTimePrinted;                        //!< AUTO: CPU time when output has been printed last time


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverOutputData()
  {
    finishedSuccessfully = false;
    verboseMode = 0;
    verboseModeFile = 0;
    writeToSolutionFile = false;
    writeToSolverFile = false;
    lastSolutionWritten = 0.;
    lastSensorsWritten = 0.;
    lastImageRecorded = 0.;
    cpuStartTime = 0.;
    cpuLastTimePrinted = 0.;
  };

  // AUTO: access functions
  //! AUTO: initialize SolverOutputData by assigning default values
  void InitializeData() {
    *this = SolverOutputData();
  }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverOutputData" << ":\n";
    os << "  finishedSuccessfully = " << finishedSuccessfully << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
    os << "  writeToSolutionFile = " << writeToSolutionFile << "\n";
    os << "  writeToSolverFile = " << writeToSolverFile << "\n";
    os << "  sensorValuesTemp = " << sensorValuesTemp << "\n";
    os << "  lastSolutionWritten = " << lastSolutionWritten << "\n";
    os << "  lastSensorsWritten = " << lastSensorsWritten << "\n";
    os << "  lastImageRecorded = " << lastImageRecorded << "\n";
    os << "  cpuStartTime = " << cpuStartTime << "\n";
    os << "  cpuLastTimePrinted = " << cpuLastTimePrinted << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolverOutputData& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolverFileData
* @brief        Solver internal structure for output files. This structure is not linked to pybind, because std::ofstream is not supported.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-07-20 (last modfied)
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

class SolverFileData // AUTO: 
{
public: // AUTO: 
  std::ofstream solutionFile;                     //!< AUTO: solution file with coordinate data
  std::ofstream solverFile;                       //!< AUTO: file with detailed solver information
  std::vector<std::ofstream*> sensorFileList;     //!< AUTO: files for sensor output; the ofstream list corresponds exactly to the sensors in the computationalSystem (i.e., sensorFileList[0] is the ofstream for sensor 0, etc.); file lists need to be closed and deleted at end of simulation!


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverFileData" << ":\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolverFileData& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
