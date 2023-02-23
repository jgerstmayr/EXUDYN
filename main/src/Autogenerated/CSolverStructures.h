/** ***********************************************************************************************
* @class        CSolverTimer
* @brief        Structure for timing in solver. Each Real variable is used to measure the CPU time which certain parts of the solver need. This structure is only active if the code is not compiled with the __FAST_EXUDYN_LINALG option and if displayComputationTime is set True. Timings will only be filled, if useTimer is True.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-02-23 (last modfied)
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
  Real AERHS;                                     //!< AUTO: time for residual evaluation of algebraic equations right-hand-side
  Real errorEstimator;                            //!< AUTO: for explicit solvers, additional evaluation
  Real factorization;                             //!< AUTO: solve or inverse
  Real integrationFormula;                        //!< AUTO: time spent for evaluation of integration formulas
  Real jacobianAE;                                //!< AUTO: jacobian of algebraic equations (not counted in sum)
  Real jacobianODE1;                              //!< AUTO: jacobian w.r.t. coordinates of \hac{ODE1} equations (not counted in sum)
  Real jacobianODE2;                              //!< AUTO: jacobian w.r.t. coordinates of \hac{ODE2} equations (not counted in sum)
  Real jacobianODE2_t;                            //!< AUTO: jacobian w.r.t. coordinates_t of \hac{ODE2} equations (not counted in sum)
  Real massMatrix;                                //!< AUTO: mass matrix computation
  Real newtonIncrement;                           //!< AUTO: Jac\f$^{-1}\f$ * RHS; backsubstitution
  Real ODE1RHS;                                   //!< AUTO: time for residual evaluation of \hac{ODE1} right-hand-side
  Real ODE2RHS;                                   //!< AUTO: time for residual evaluation of \hac{ODE2} right-hand-side
  Real overhead;                                  //!< AUTO: overhead, such as initialization, copying and some matrix-vector multiplication
  Real postNewton;                                //!< AUTO: discontinuous iteration / PostNewtonStep
  Real python;                                    //!< AUTO: time spent for Python functions
  Real reactionForces;                            //!< AUTO: CqT * lambda
  Real total;                                     //!< AUTO: total time measured between start and end of computation (static/dynamics)
  Real totalJacobian;                             //!< AUTO: time for all jacobian computations
  bool useTimer;                                  //!< AUTO: flag to decide, whether the timer is used (true) or not
  Real visualization;                             //!< AUTO: time spent for visualization in computation thread
  Real writeSolution;                             //!< AUTO: time for writing solution


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  CSolverTimer()
  {
    AERHS = 0.;
    errorEstimator = 0.;
    factorization = 0.;
    integrationFormula = 0.;
    jacobianAE = 0.;
    jacobianODE1 = 0.;
    jacobianODE2 = 0.;
    jacobianODE2_t = 0.;
    massMatrix = 0.;
    newtonIncrement = 0.;
    ODE1RHS = 0.;
    ODE2RHS = 0.;
    overhead = 0.;
    postNewton = 0.;
    python = 0.;
    reactionForces = 0.;
    total = 0.;
    totalJacobian = 0.;
    useTimer = true;
    visualization = 0.;
    writeSolution = 0.;
  };

  // AUTO: access functions
  //! AUTO: reset solver timings to initial state by assigning default values; useSolverTimer sets the useTimer flag
  void Reset(bool useSolverTimer) {
    *this = CSolverTimer(); useTimer = useSolverTimer;
  }

  //! AUTO: start timer function for a given variable; subtracts current CPU time from value
  void StartTimer(Real& value) {
    if (useTimer) { value -= EXUstd::GetTimeInSeconds();}
  }

  //! AUTO: stop timer function for a given variable; adds current CPU time to value
  void StopTimer(Real& value) {
    if (useTimer) { value += EXUstd::GetTimeInSeconds();}
  }

  //! AUTO: compute sum of all timers (except for those counted multiple, e.g., jacobians
  Real Sum() const;
  //! AUTO: converts the current timings to a string
  std::string ToString() const;
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "CSolverTimer" << ":\n";
    os << "  AERHS = " << AERHS << "\n";
    os << "  errorEstimator = " << errorEstimator << "\n";
    os << "  factorization = " << factorization << "\n";
    os << "  integrationFormula = " << integrationFormula << "\n";
    os << "  jacobianAE = " << jacobianAE << "\n";
    os << "  jacobianODE1 = " << jacobianODE1 << "\n";
    os << "  jacobianODE2 = " << jacobianODE2 << "\n";
    os << "  jacobianODE2_t = " << jacobianODE2_t << "\n";
    os << "  massMatrix = " << massMatrix << "\n";
    os << "  newtonIncrement = " << newtonIncrement << "\n";
    os << "  ODE1RHS = " << ODE1RHS << "\n";
    os << "  ODE2RHS = " << ODE2RHS << "\n";
    os << "  overhead = " << overhead << "\n";
    os << "  postNewton = " << postNewton << "\n";
    os << "  python = " << python << "\n";
    os << "  reactionForces = " << reactionForces << "\n";
    os << "  total = " << total << "\n";
    os << "  totalJacobian = " << totalJacobian << "\n";
    os << "  useTimer = " << useTimer << "\n";
    os << "  visualization = " << visualization << "\n";
    os << "  writeSolution = " << writeSolution << "\n";
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
* @date         AUTO: 2023-02-23 (last modfied)
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
  ResizableVectorParallel aAlgorithmic;           //!< AUTO: additional term needed for generalized alpha (current state)
  Index nAE;                                      //!< AUTO: number of algebraic coordinates
  Index nData;                                    //!< AUTO: number of data coordinates
  ResizableVectorParallel newtonSolution;         //!< AUTO: Newton decrement (computed from residual and jacobian)
  Index nODE1;                                    //!< AUTO: number of first order ordinary diff. eq. coordinates
  Index nODE2;                                    //!< AUTO: number of second order ordinary diff. eq. coordinates
  Index nSys;                                     //!< AUTO: number of system (unknown) coordinates = nODE2+nODE1+nAE
  Index startAE;                                  //!< AUTO: start of algebraic coordinates, but set to zero if nAE==0
  ResizableVectorParallel startOfStepStateAAlgorithmic;//!< AUTO: additional term needed for generalized alpha (startOfStep state)
  ResizableVectorParallel systemResidual;         //!< AUTO: system residual vector (vectors will be linked to this vector!)
  ResizableVectorParallel temp2ODE2;              //!< AUTO: second temporary vector for \hac{ODE2} quantities; use in static computation
  ResizableVectorParallel tempODE1F0;             //!< AUTO: temporary vector for \hac{ODE1} Jacobian
  ResizableVectorParallel tempODE1F1;             //!< AUTO: temporary vector for \hac{ODE1} Jacobian
  ResizableVectorParallel tempODE2;               //!< AUTO: temporary vector for \hac{ODE2} quantities; use in initial accelerations and during Newton
  ResizableVectorParallel tempODE2F0;             //!< AUTO: temporary vector for \hac{ODE2} Jacobian
  ResizableVectorParallel tempODE2F1;             //!< AUTO: temporary vector for \hac{ODE2} Jacobian
  GeneralMatrix* jacobianAE;                      //!< AUTO: link to dense or sparse algebraic equations jacobian
  GeneralMatrix* systemJacobian;                  //!< AUTO: link to dense or sparse system jacobian
  GeneralMatrix* systemMassMatrix;                //!< AUTO: link to dense or sparse system mass matrix; in explicit solver, after a step, this will contain the factorized mass matrix
  TemporaryComputationData tempCompData;          //!< AUTO: temporary data used during item-related residual and jacobian computation; duplicated for serial computation, will be removed in future
  TemporaryComputationDataArray tempCompDataArray;//!< AUTO: temporary data per thread, used during item-related residual and jacobian computation; for parallel computation

private: // AUTO: 
  GeneralMatrixEXUdense jacobianAEdense;          //!< AUTO: dense \hac{AE} jacobian
  GeneralMatrixEigenSparse jacobianAEsparse;      //!< AUTO: sparse \hac{AE} jacobian
  LinearSolverType linearSolverType;              //!< AUTO: contains linear solver type value; cannot be accessed directly, because a change requires new linking of system matrices
  GeneralMatrixEXUdense systemJacobianDense;      //!< AUTO: dense system jacobian
  GeneralMatrixEigenSparse systemJacobianSparse;  //!< AUTO: sparse system jacobian
  GeneralMatrixEXUdense systemMassMatrixDense;    //!< AUTO: dense mass matrix
  GeneralMatrixEigenSparse systemMassMatrixSparse;//!< AUTO: sparse mass matrix


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverLocalData()
  {
    jacobianAE = nullptr;
    nAE = 0;
    nData = 0;
    nODE1 = 0;
    nODE2 = 0;
    nSys = 0;
    startAE = 0;
    systemJacobian = nullptr;
    systemMassMatrix = nullptr;
    SetLinearSolverType(LinearSolverType::EXUdense, false); //for safety, data is linked initially
  };

  // AUTO: access functions
  //! AUTO: if desired, temporary data is cleaned up to safe memory
  void CleanUpMemory();
  //! AUTO: return current linear solver type (dense/sparse)
  LinearSolverType GetLinearSolverType() const {
    return linearSolverType;
  }

  //! AUTO: set linear solver type and matrix version: links system matrices to according dense/sparse versions
  void SetLinearSolverType(LinearSolverType linearSolverType, bool reuseAnalyzedPattern);
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverLocalData" << ":\n";
    os << "  aAlgorithmic = " << aAlgorithmic << "\n";
    os << "  jacobianAE = " << *jacobianAE << "\n";
    os << "  jacobianAEdense = " << jacobianAEdense << "\n";
    os << "  jacobianAEsparse = " << jacobianAEsparse << "\n";
    os << "  linearSolverType = " << linearSolverType << "\n";
    os << "  nAE = " << nAE << "\n";
    os << "  nData = " << nData << "\n";
    os << "  newtonSolution = " << newtonSolution << "\n";
    os << "  nODE1 = " << nODE1 << "\n";
    os << "  nODE2 = " << nODE2 << "\n";
    os << "  nSys = " << nSys << "\n";
    os << "  startAE = " << startAE << "\n";
    os << "  startOfStepStateAAlgorithmic = " << startOfStepStateAAlgorithmic << "\n";
    os << "  systemJacobian = " << *systemJacobian << "\n";
    os << "  systemJacobianDense = " << systemJacobianDense << "\n";
    os << "  systemJacobianSparse = " << systemJacobianSparse << "\n";
    os << "  systemMassMatrix = " << *systemMassMatrix << "\n";
    os << "  systemMassMatrixDense = " << systemMassMatrixDense << "\n";
    os << "  systemMassMatrixSparse = " << systemMassMatrixSparse << "\n";
    os << "  systemResidual = " << systemResidual << "\n";
    os << "  temp2ODE2 = " << temp2ODE2 << "\n";
    os << "  tempODE1F0 = " << tempODE1F0 << "\n";
    os << "  tempODE1F1 = " << tempODE1F1 << "\n";
    os << "  tempODE2 = " << tempODE2 << "\n";
    os << "  tempODE2F0 = " << tempODE2F0 << "\n";
    os << "  tempODE2F1 = " << tempODE2F1 << "\n";
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
* @date         AUTO: 2023-02-23 (last modfied)
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
  bool adaptiveStep;                              //!< AUTO: True: the step size may be reduced if step fails; no automatic stepsize control
  bool automaticStepSize;                         //!< AUTO: True: if timeIntegration.automaticStepSize == True AND chosen integrators supports automatic step size control (e.g., DOPRI5); False: constant step size used (step may be reduced if adaptiveStep=True)
  Real automaticStepSizeError;                    //!< AUTO: estimated error (relative to atol + rtol*solution) of last step; must be \f$\le 1\f$  for a step to be accepted
  Index currentStepIndex;                         //!< AUTO: current step index; \f$i\f$
  Real currentStepSize;                           //!< AUTO: stepSize of current step
  Real currentTime;                               //!< AUTO: holds the current simulation time, copy of state.current.time; interval is [startTime,tEnd]; in static solver, duration is loadStepDuration
  Index discontinuousIteration;                   //!< AUTO: number of current discontinuous iteration
  Index discontinuousIterationsCount;             //!< AUTO: count total number of discontinuous iterations (min. 1 per step)
  Real endTime;                                   //!< AUTO: end time of static/dynamic solver
  Real initialStepSize;                           //!< AUTO: initial stepSize for dynamic solver; only used, if automaticStepSize is activated
  Real lastStepSize;                              //!< AUTO: stepSize suggested from last step or by initial step size; only used, if automaticStepSize is activated
  Real maxStepSize;                               //!< AUTO: constant or maximum stepSize
  Real minStepSize;                               //!< AUTO: minimum stepSize for static/dynamic solver; only used, if automaticStepSize is activated
  Index newtonJacobiCount;                        //!< AUTO: count total Newton jacobian computations
  Index newtonSteps;                              //!< AUTO: number of current newton steps
  Index newtonStepsCount;                         //!< AUTO: count total Newton steps
  Index numberOfSteps;                            //!< AUTO: number of time steps (if fixed size); \f$n\f$
  Real recommendedStepSize;                       //!< AUTO: recommended step size \f$h_{recom}\f$ after PostNewton(...): \f$h_{recom} < 0\f$: no recommendation, \f$h_{recom}==0\f$: use minimum step size, \f$h_{recom}>0\f$: use specific step size, if no smaller size requested by other reason
  Index rejectedAutomaticStepSizeSteps;           //!< AUTO: count the number of rejected steps in case of automatic step size control (rejected steps are repeated with smaller step size)
  Index rejectedModifiedNewtonSteps;              //!< AUTO: count the number of rejected modified Newton steps (switch to full Newton)
  Real startTime;                                 //!< AUTO: time at beginning of time integration


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverIterationData()
  {
    adaptiveStep = true;
    automaticStepSize = true;
    automaticStepSizeError = 0;
    currentStepIndex = 0;
    currentStepSize = 0.;
    currentTime = 0.;
    discontinuousIteration = 0;
    discontinuousIterationsCount = 0;
    endTime = 0.;
    initialStepSize = 1e-6;
    lastStepSize = 0.;
    maxStepSize = 0.;
    minStepSize = 0.;
    newtonJacobiCount = 0;
    newtonSteps = 0;
    newtonStepsCount = 0;
    numberOfSteps = 0;
    recommendedStepSize = -1.;
    rejectedAutomaticStepSizeSteps = 0;
    rejectedModifiedNewtonSteps = 0;
    startTime = 0.;
  };

  // AUTO: access functions
  //! AUTO: convert iteration statistics to string; used for displayStatistics option
  std::string ToString() const;
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverIterationData" << ":\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  automaticStepSize = " << automaticStepSize << "\n";
    os << "  automaticStepSizeError = " << automaticStepSizeError << "\n";
    os << "  currentStepIndex = " << currentStepIndex << "\n";
    os << "  currentStepSize = " << currentStepSize << "\n";
    os << "  currentTime = " << currentTime << "\n";
    os << "  discontinuousIteration = " << discontinuousIteration << "\n";
    os << "  discontinuousIterationsCount = " << discontinuousIterationsCount << "\n";
    os << "  endTime = " << endTime << "\n";
    os << "  initialStepSize = " << initialStepSize << "\n";
    os << "  lastStepSize = " << lastStepSize << "\n";
    os << "  maxStepSize = " << maxStepSize << "\n";
    os << "  minStepSize = " << minStepSize << "\n";
    os << "  newtonJacobiCount = " << newtonJacobiCount << "\n";
    os << "  newtonSteps = " << newtonSteps << "\n";
    os << "  newtonStepsCount = " << newtonStepsCount << "\n";
    os << "  numberOfSteps = " << numberOfSteps << "\n";
    os << "  recommendedStepSize = " << recommendedStepSize << "\n";
    os << "  rejectedAutomaticStepSizeSteps = " << rejectedAutomaticStepSizeSteps << "\n";
    os << "  rejectedModifiedNewtonSteps = " << rejectedModifiedNewtonSteps << "\n";
    os << "  startTime = " << startTime << "\n";
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
* @date         AUTO: 2023-02-23 (last modfied)
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
  Real contractivity;                             //!< AUTO: Newton contractivity = geometric decay of error in every step
  Real discontinuousIterationError;               //!< AUTO: error of discontinuous iterations (contact, friction, ...) outside of Newton iteration
  bool discontinuousIterationSuccessful;          //!< AUTO: true, if last discontinuous iteration had success (failure may be recovered by adaptive step)
  Real errorCoordinateFactor;                     //!< AUTO: factor may include the number of system coordinates to reduce the residual
  bool jacobianUpdateRequested;                   //!< AUTO: true, if a jacobian update is requested in modified Newton (determined in previous step)
  Real lastResidual;                              //!< AUTO: last Newton residual to determine contractivity
  Index linearSolverCausingRow;                   //!< AUTO: -1 if successful, 0 ... n-1, the system equation (=coordinate) index which may have caused the problem, at which the linear solver failed
  bool linearSolverFailed;                        //!< AUTO: true, if linear solver failed to factorize
  bool massMatrixNotInvertible;                   //!< AUTO: true, if mass matrix is not invertable during initialization or solution (explicit solver)
  bool newtonConverged;                           //!< AUTO: true, if Newton has (finally) converged
  bool newtonSolutionDiverged;                    //!< AUTO: true, if Newton diverged (may be recovered)
  Real residual;                                  //!< AUTO: current Newton residual
  bool stepReductionFailed;                       //!< AUTO: true, if iterations over time/static steps failed (finally, cannot be recovered)
  bool stopNewton;                                //!< AUTO: set true by Newton, if Newton was stopped, e.g., because of exceeding iterations or linear solver failed


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverConvergenceData()
  {
    contractivity = 0.;
    discontinuousIterationError = 0.;
    discontinuousIterationSuccessful = true;
    errorCoordinateFactor = 1.;
    jacobianUpdateRequested = true;
    lastResidual = 0.;
    linearSolverCausingRow = -1;
    linearSolverFailed = false;
    massMatrixNotInvertible = false;
    newtonConverged = false;
    newtonSolutionDiverged = false;
    residual = 0.;
    stepReductionFailed = false;
    stopNewton = false;
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
    os << "  contractivity = " << contractivity << "\n";
    os << "  discontinuousIterationError = " << discontinuousIterationError << "\n";
    os << "  discontinuousIterationSuccessful = " << discontinuousIterationSuccessful << "\n";
    os << "  errorCoordinateFactor = " << errorCoordinateFactor << "\n";
    os << "  jacobianUpdateRequested = " << jacobianUpdateRequested << "\n";
    os << "  lastResidual = " << lastResidual << "\n";
    os << "  linearSolverCausingRow = " << linearSolverCausingRow << "\n";
    os << "  linearSolverFailed = " << linearSolverFailed << "\n";
    os << "  massMatrixNotInvertible = " << massMatrixNotInvertible << "\n";
    os << "  newtonConverged = " << newtonConverged << "\n";
    os << "  newtonSolutionDiverged = " << newtonSolutionDiverged << "\n";
    os << "  residual = " << residual << "\n";
    os << "  stepReductionFailed = " << stepReductionFailed << "\n";
    os << "  stopNewton = " << stopNewton << "\n";
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
* @date         AUTO: 2023-02-23 (last modfied)
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
  Real cpuLastTimePrinted;                        //!< AUTO: CPU time when output has been printed last time
  Real cpuStartTime;                              //!< AUTO: CPU start time of computation (starts counting at computation of initial conditions)
  bool finishedSuccessfully;                      //!< AUTO: flag is false until solver finshed successfully (can be used as external trigger)
  Index lastDiscontinuousIterationsCount;         //!< AUTO: discontinuous iterations count when written to console (or file) last time
  Real lastImageRecorded;                         //!< AUTO: simulation time when last image has been recorded
  Index lastNewtonJacobiCount;                    //!< AUTO: jacobian update count when written to console (or file) last time
  Index lastNewtonStepsCount;                     //!< AUTO: newton steps count when written to console (or file) last time
  Real lastSensorsWritten;                        //!< AUTO: simulation time when last sensors have been written
  Real lastSolutionWritten;                       //!< AUTO: simulation time when last solution has been written
  Index lastVerboseStepIndex;                     //!< AUTO: step index when last time written to console (or file)
  Index multiThreadingMode;                       //!< AUTO: multithreading mode that has been used: 0=None (serial), 1=NGsolve taskmanager, 2=MicroThreading (Exudyn)
  Index numberOfThreadsUsed;                      //!< AUTO: number of threads that have been used in simulation
  ResizableVector sensorValuesTemp;               //!< AUTO: temporary vector for per sensor values (overwritten for every sensor; usually contains last sensor values)
  ResizableVector sensorValuesTemp2;              //!< AUTO: additional temporary vector for per sensor values (overwritten for every sensor; usually contains time+last sensor values)
  Index stepInformation;                          //!< AUTO: this is a copy of the solvers stepInformation used for console output
  Index verboseMode;                              //!< AUTO: this is a copy of the solvers verboseMode used for console output
  Index verboseModeFile;                          //!< AUTO: this is a copy of the solvers verboseModeFile used for file
  bool writeToSolutionFile;                       //!< AUTO: if false, no solution file is generated and no file is written
  bool writeToSolverFile;                         //!< AUTO: if false, no solver output file is generated and no file is written


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolverOutputData()
  {
    cpuLastTimePrinted = 0.;
    cpuStartTime = 0.;
    finishedSuccessfully = false;
    lastDiscontinuousIterationsCount = 0;
    lastImageRecorded = 0.;
    lastNewtonJacobiCount = 0;
    lastNewtonStepsCount = 0;
    lastSensorsWritten = 0.;
    lastSolutionWritten = 0.;
    lastVerboseStepIndex = 0;
    multiThreadingMode = 0;
    numberOfThreadsUsed = 0;
    stepInformation = 0;
    verboseMode = 0;
    verboseModeFile = 0;
    writeToSolutionFile = false;
    writeToSolverFile = false;
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
    os << "  cpuLastTimePrinted = " << cpuLastTimePrinted << "\n";
    os << "  cpuStartTime = " << cpuStartTime << "\n";
    os << "  finishedSuccessfully = " << finishedSuccessfully << "\n";
    os << "  lastDiscontinuousIterationsCount = " << lastDiscontinuousIterationsCount << "\n";
    os << "  lastImageRecorded = " << lastImageRecorded << "\n";
    os << "  lastNewtonJacobiCount = " << lastNewtonJacobiCount << "\n";
    os << "  lastNewtonStepsCount = " << lastNewtonStepsCount << "\n";
    os << "  lastSensorsWritten = " << lastSensorsWritten << "\n";
    os << "  lastSolutionWritten = " << lastSolutionWritten << "\n";
    os << "  lastVerboseStepIndex = " << lastVerboseStepIndex << "\n";
    os << "  multiThreadingMode = " << multiThreadingMode << "\n";
    os << "  numberOfThreadsUsed = " << numberOfThreadsUsed << "\n";
    os << "  sensorValuesTemp = " << sensorValuesTemp << "\n";
    os << "  sensorValuesTemp2 = " << sensorValuesTemp2 << "\n";
    os << "  stepInformation = " << stepInformation << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
    os << "  writeToSolutionFile = " << writeToSolutionFile << "\n";
    os << "  writeToSolverFile = " << writeToSolverFile << "\n";
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
* @date         AUTO: 2023-02-23 (last modfied)
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
  ExuFile::BinaryFileSettings binaryFileSettings; //!< AUTO: settings for binary file write mode
  std::vector<std::ofstream*> sensorFileList;     //!< AUTO: files for sensor output; the ofstream list corresponds exactly to the sensors in the computationalSystem (i.e., sensorFileList[0] is the ofstream for sensor 0, etc.); file lists need to be closed and deleted at end of simulation!
  std::ofstream solutionFile;                     //!< AUTO: solution file with coordinate data
  std::ofstream solverFile;                       //!< AUTO: file with detailed solver information


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolverFileData" << ":\n";
    os << "  binaryFileSettings = " << binaryFileSettings << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolverFileData& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
