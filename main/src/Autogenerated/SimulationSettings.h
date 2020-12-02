/** ***********************************************************************************************
* @class        SolutionSettings
* @brief        General settings for exporting the solution (results) of a simulation.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-12-02 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/

#ifndef SIMULATIONSETTINGS__H
#define SIMULATIONSETTINGS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class SolutionSettings // AUTO: 
{
public: // AUTO: 
  bool writeSolutionToFile;                       //!< AUTO: flag (true/false), which determines if (global) solution vector is written to file
  bool appendToFile;                              //!< AUTO: flag (true/false); if true, solution and solverInformation is appended to existing file (otherwise created)
  bool writeFileHeader;                           //!< AUTO: flag (true/false); if true, file header is written (turn off, e.g. for multiple runs of time integration)
  bool writeFileFooter;                           //!< AUTO: flag (true/false); if true, information at end of simulation is written: convergence, total solution time, statistics
  Real solutionWritePeriod;                       //!< AUTO: time span (period), determines how often the solution is written during a simulation
  bool sensorsAppendToFile;                       //!< AUTO: flag (true/false); if true, sensor output is appended to existing file (otherwise created)
  bool sensorsWriteFileHeader;                    //!< AUTO: flag (true/false); if true, file header is written for sensor output (turn off, e.g. for multiple runs of time integration)
  Real sensorsWritePeriod;                        //!< AUTO: time span (period), determines how often the sensor output is written during a simulation
  bool exportVelocities;                          //!< AUTO: solution is written as displacements, velocities[, accelerations] [,algebraicCoordinates] [,DataCoordinates]
  bool exportAccelerations;                       //!< AUTO: solution is written as displacements, [velocities,] accelerations [,algebraicCoordinates] [,DataCoordinates]
  bool exportAlgebraicCoordinates;                //!< AUTO: solution is written as displacements, [velocities,] [accelerations,], algebraicCoordinates (=Lagrange multipliers) [,DataCoordinates]
  bool exportDataCoordinates;                     //!< AUTO: solution is written as displacements, [velocities,] [accelerations,] [,algebraicCoordinates (=Lagrange multipliers)] ,DataCoordinates
  std::string coordinatesSolutionFileName;        //!< AUTO: filename and (relative) path of solution file containing all coordinates versus time; directory will be created if it does not exist
  std::string solverInformationFileName;          //!< AUTO: filename and (relative) path of text file showing detailed information during solving; detail level according to yourSolver.verboseModeFile; if solutionSettings.appendToFile is true, the information is appended in every solution step; directory will be created if it does not exist
  std::string solutionInformation;                //!< AUTO: special information added to header of solution file (e.g. parameters and settings, modes, ...)
  Index outputPrecision;                          //!< AUTO: precision for floating point numbers written to solution and sensor files
  Real recordImagesInterval;                      //!< AUTO: record frames (images) during solving: amount of time to wait until next image (frame) is recorded; set recordImages = -1. if no images shall be recorded; set, e.g., recordImages = 0.01 to record an image every 10 milliseconds (requires that the time steps / load steps are sufficiently small!); for file names, etc., see VisualizationSettings.exportImages


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionSettings()
  {
    writeSolutionToFile = true;
    appendToFile = false;
    writeFileHeader = true;
    writeFileFooter = true;
    solutionWritePeriod = 0.01;
    sensorsAppendToFile = false;
    sensorsWriteFileHeader = true;
    sensorsWritePeriod = 0.01;
    exportVelocities = true;
    exportAccelerations = true;
    exportAlgebraicCoordinates = true;
    exportDataCoordinates = true;
    coordinatesSolutionFileName = "coordinatesSolution.txt";
    solverInformationFileName = "solverInformation.txt";
    outputPrecision = 10;
    recordImagesInterval = -1.;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionSettings" << ":\n";
    os << "  writeSolutionToFile = " << writeSolutionToFile << "\n";
    os << "  appendToFile = " << appendToFile << "\n";
    os << "  writeFileHeader = " << writeFileHeader << "\n";
    os << "  writeFileFooter = " << writeFileFooter << "\n";
    os << "  solutionWritePeriod = " << solutionWritePeriod << "\n";
    os << "  sensorsAppendToFile = " << sensorsAppendToFile << "\n";
    os << "  sensorsWriteFileHeader = " << sensorsWriteFileHeader << "\n";
    os << "  sensorsWritePeriod = " << sensorsWritePeriod << "\n";
    os << "  exportVelocities = " << exportVelocities << "\n";
    os << "  exportAccelerations = " << exportAccelerations << "\n";
    os << "  exportAlgebraicCoordinates = " << exportAlgebraicCoordinates << "\n";
    os << "  exportDataCoordinates = " << exportDataCoordinates << "\n";
    os << "  coordinatesSolutionFileName = " << coordinatesSolutionFileName << "\n";
    os << "  solverInformationFileName = " << solverInformationFileName << "\n";
    os << "  solutionInformation = " << solutionInformation << "\n";
    os << "  outputPrecision = " << outputPrecision << "\n";
    os << "  recordImagesInterval = " << recordImagesInterval << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolutionSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        NumericalDifferentiationSettings
* @brief        Settings for numerical differentiation of a function (needed for computation of numerical jacobian e.g. in implizit integration); HOTINT1: relativeEpsilon * Maximum(minimumCoordinateSize, fabs(x(i))).
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-12-02 (last modfied)
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

class NumericalDifferentiationSettings // AUTO: 
{
public: // AUTO: 
  Real relativeEpsilon;                           //!< AUTO: relative differentiation parameter epsilon; the numerical differentiation parameter \f$\varepsilon\f$ follows from the formula (\f$\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)\f$, with \f$\varepsilon_\mathrm{relative}\f$=relativeEpsilon, \f$q_{min} = \f$minimumCoordinateSize, \f$q_i\f$ is the current coordinate which is differentiated, and \f$qRef_i\f$ is the reference coordinate of the current coordinate
  Real minimumCoordinateSize;                     //!< AUTO: minimum size of coordinates in relative differentiation parameter
  bool doSystemWideDifferentiation;               //!< AUTO: true: system wide differentiation (e.g. all ODE2 equations w.r.t. all ODE2 coordinates); false: only local (object) differentiation
  bool addReferenceCoordinatesToEpsilon;          //!< AUTO: true: for the size estimation of the differentiation parameter, the reference coordinate \f$q^{Ref}_i\f$ is added to ODE2 coordinates --> see; false: only the current coordinate is used for size estimation of the differentiation parameter


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  NumericalDifferentiationSettings()
  {
    relativeEpsilon = 1e-7;
    minimumCoordinateSize = 1e-2;
    doSystemWideDifferentiation = false;
    addReferenceCoordinatesToEpsilon = false;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "NumericalDifferentiationSettings" << ":\n";
    os << "  relativeEpsilon = " << relativeEpsilon << "\n";
    os << "  minimumCoordinateSize = " << minimumCoordinateSize << "\n";
    os << "  doSystemWideDifferentiation = " << doSystemWideDifferentiation << "\n";
    os << "  addReferenceCoordinatesToEpsilon = " << addReferenceCoordinatesToEpsilon << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const NumericalDifferentiationSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        NewtonSettings
* @brief        Settings for Newton method used in static or dynamic simulation.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-12-02 (last modfied)
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

class NewtonSettings // AUTO: 
{
public: // AUTO: 
  NumericalDifferentiationSettings numericalDifferentiation;//!< AUTO: numerical differentiation parameters for numerical jacobian (e.g. Newton in static solver or implicit time integration)
  bool useNumericalDifferentiation;               //!< AUTO: flag (true/false); false = perform direct computation of jacobian, true = use numerical differentiation for jacobian
  bool useNewtonSolver;                           //!< AUTO: flag (true/false); false = linear computation, true = use Newton solver for nonlinear solution
  Real relativeTolerance;                         //!< AUTO: relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
  Real absoluteTolerance;                         //!< AUTO: absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
  bool weightTolerancePerCoordinate;              //!< AUTO: flag (true/false); false = compute error as L2-Norm of residual; true = compute error as (L2-Norm of residual) / (sqrt(number of coordinates)), which can help to use common tolerance independent of system size
  Index newtonResidualMode;                       //!< AUTO: 0 ... use residual for computation of error (standard); 1 ... use change of solution increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  bool adaptInitialResidual;                      //!< AUTO: flag (true/false); false = standard; true: if initialResidual is very small (or zero), it may increas dramatically in first step; to achieve relativeTolerance, the initialResidual will by updated by a higher residual within the first Newton iteration
  Real modifiedNewtonContractivity;               //!< AUTO: maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  bool useModifiedNewton;                         //!< AUTO: true: compute Jacobian only at first step; no Jacobian updates per step; false: Jacobian computed in every step
  bool modifiedNewtonJacUpdatePerStep;            //!< AUTO: true: compute Jacobian at every time step, but not in every iteration (except for bad convergence ==> switch to full Newton)
  Index maxIterations;                            //!< AUTO: maximum number of iterations (including modified + restart Newton steps); after that iterations, the static/dynamic solver stops with error
  Index maxModifiedNewtonIterations;              //!< AUTO: maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  Index maxModifiedNewtonRestartIterations;       //!< AUTO: maximum number of iterations for modified Newton after aJacobian update; after that number of iterations, the full Newton method is started for this step
  Real maximumSolutionNorm;                       //!< AUTO: this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors are larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
  Index maxDiscontinuousIterations;               //!< AUTO: maximum number of discontinuous (post Newton) iterations
  bool ignoreMaxDiscontinuousIterations;          //!< AUTO: continue solver if maximum number of discontinuous (post Newton) iterations is reached (ignore tolerance)
  Real discontinuousIterationTolerance;           //!< AUTO: absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
  Index stepInformation;                          //!< AUTO: 0 ... only current step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step, 3 ... show discontinuous iterations (Dit) and newton jacobians (jac) per step


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  NewtonSettings()
  {
    useNumericalDifferentiation = false;
    useNewtonSolver = true;
    relativeTolerance = 1e-8;
    absoluteTolerance = 1e-10;
    weightTolerancePerCoordinate = false;
    newtonResidualMode = 0;
    adaptInitialResidual = true;
    modifiedNewtonContractivity = 0.5;
    useModifiedNewton = false;
    modifiedNewtonJacUpdatePerStep = false;
    maxIterations = 25;
    maxModifiedNewtonIterations = 8;
    maxModifiedNewtonRestartIterations = 7;
    maximumSolutionNorm = 1e38;
    maxDiscontinuousIterations = 5;
    ignoreMaxDiscontinuousIterations = true;
    discontinuousIterationTolerance = 1;
    stepInformation = 2;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "NewtonSettings" << ":\n";
    os << "  numericalDifferentiation = " << numericalDifferentiation << "\n";
    os << "  useNumericalDifferentiation = " << useNumericalDifferentiation << "\n";
    os << "  useNewtonSolver = " << useNewtonSolver << "\n";
    os << "  relativeTolerance = " << relativeTolerance << "\n";
    os << "  absoluteTolerance = " << absoluteTolerance << "\n";
    os << "  weightTolerancePerCoordinate = " << weightTolerancePerCoordinate << "\n";
    os << "  newtonResidualMode = " << newtonResidualMode << "\n";
    os << "  adaptInitialResidual = " << adaptInitialResidual << "\n";
    os << "  modifiedNewtonContractivity = " << modifiedNewtonContractivity << "\n";
    os << "  useModifiedNewton = " << useModifiedNewton << "\n";
    os << "  modifiedNewtonJacUpdatePerStep = " << modifiedNewtonJacUpdatePerStep << "\n";
    os << "  maxIterations = " << maxIterations << "\n";
    os << "  maxModifiedNewtonIterations = " << maxModifiedNewtonIterations << "\n";
    os << "  maxModifiedNewtonRestartIterations = " << maxModifiedNewtonRestartIterations << "\n";
    os << "  maximumSolutionNorm = " << maximumSolutionNorm << "\n";
    os << "  maxDiscontinuousIterations = " << maxDiscontinuousIterations << "\n";
    os << "  ignoreMaxDiscontinuousIterations = " << ignoreMaxDiscontinuousIterations << "\n";
    os << "  discontinuousIterationTolerance = " << discontinuousIterationTolerance << "\n";
    os << "  stepInformation = " << stepInformation << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const NewtonSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        GeneralizedAlphaSettings
* @brief        Settings for generalized-alpha, implicit trapezoidal or Newmark time integration methods.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-12-02 (last modfied)
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

class GeneralizedAlphaSettings // AUTO: 
{
public: // AUTO: 
  Real newmarkBeta;                               //!< AUTO: value beta for Newmark method; default value beta = \f$\frac 1 4\f$ corresponds to (undamped) trapezoidal rule
  Real newmarkGamma;                              //!< AUTO: value gamma for Newmark method; default value gamma = \f$\frac 1 2\f$ corresponds to (undamped) trapezoidal rule
  bool useIndex2Constraints;                      //!< AUTO: set useIndex2Constraints = true in order to use index2 (velocity level constraints) formulation
  bool useNewmark;                                //!< AUTO: if true, use Newmark method with beta and gamma instead of generalized-Alpha
  Real spectralRadius;                            //!< AUTO: spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
  bool computeInitialAccelerations;               //!< AUTO: true: compute initial accelerations from system EOM in acceleration form; NOTE that initial accelerations that are following from user functions in constraints are not considered for now! false: use zero accelerations


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  GeneralizedAlphaSettings()
  {
    newmarkBeta = 0.25;
    newmarkGamma = 0.5;
    useIndex2Constraints = false;
    useNewmark = false;
    spectralRadius = 0.9;
    computeInitialAccelerations = true;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "GeneralizedAlphaSettings" << ":\n";
    os << "  newmarkBeta = " << newmarkBeta << "\n";
    os << "  newmarkGamma = " << newmarkGamma << "\n";
    os << "  useIndex2Constraints = " << useIndex2Constraints << "\n";
    os << "  useNewmark = " << useNewmark << "\n";
    os << "  spectralRadius = " << spectralRadius << "\n";
    os << "  computeInitialAccelerations = " << computeInitialAccelerations << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const GeneralizedAlphaSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        TimeIntegrationSettings
* @brief        General parameters used in time integration; specific parameters are provided in the according solver settings, e.g. for generalizedAlpha.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-12-02 (last modfied)
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

class TimeIntegrationSettings // AUTO: 
{
public: // AUTO: 
  NewtonSettings newton;                          //!< AUTO: parameters for Newton method; used for implicit time integration methods only
  Real startTime;                                 //!< AUTO: start time of time integration (usually set to zero)
  Real endTime;                                   //!< AUTO: end time of time integration
  Index numberOfSteps;                            //!< AUTO: number of steps in time integration; stepsize is computed from (endTime-startTime)/numberOfSteps
  bool adaptiveStep;                              //!< AUTO: true: use step reduction if step fails; false: constant step size
  Real minimumStepSize;                           //!< AUTO: lower limit of time step size, before integrator stops
  Index verboseMode;                              //!< AUTO: 0 ... no output, 1 ... show short step information every 2 seconds (error), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  Index verboseModeFile;                          //!< AUTO: same behaviour as verboseMode, but outputs all solver information to file
  GeneralizedAlphaSettings generalizedAlpha;      //!< AUTO: parameters for generalized-alpha, implicit trapezoidal rule or Newmark (options only apply for these methods)
  std::string preStepPyExecute;                   //!< AUTO: DEPRECATED, use preStepFunction in simulation settings; Python code to be executed prior to every step and after last step, e.g. for postprocessing


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  TimeIntegrationSettings()
  {
    startTime = 0;
    endTime = 1;
    numberOfSteps = 100;
    adaptiveStep = true;
    minimumStepSize = 1e-8;
    verboseMode = 0;
    verboseModeFile = 0;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "TimeIntegrationSettings" << ":\n";
    os << "  newton = " << newton << "\n";
    os << "  startTime = " << startTime << "\n";
    os << "  endTime = " << endTime << "\n";
    os << "  numberOfSteps = " << numberOfSteps << "\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  minimumStepSize = " << minimumStepSize << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
    os << "  generalizedAlpha = " << generalizedAlpha << "\n";
    os << "  preStepPyExecute = " << preStepPyExecute << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const TimeIntegrationSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        StaticSolverSettings
* @brief        Settings for static solver linear or nonlinear (Newton).
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-12-02 (last modfied)
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

class StaticSolverSettings // AUTO: 
{
public: // AUTO: 
  NewtonSettings newton;                          //!< AUTO: parameters for Newton method (e.g. in static solver or time integration)
  Index numberOfLoadSteps;                        //!< AUTO: number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  Real loadStepDuration;                          //!< AUTO: quasi-time for all load steps (added to current time in load steps)
  Real loadStepStart;                             //!< AUTO: a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  bool loadStepGeometric;                         //!< AUTO: if loadStepGeometric=false, the load steps are incremental (arithmetic series, e.g. 0.1,0.2,0.3,...); if true, the load steps are increased in a geometric series, e.g. for \f$n=8\f$ numberOfLoadSteps and \f$d = 1000\f$ loadStepGeometricRange, it follows: \f$1000^{1/8}/1000=0.00237\f$, \f$1000^{2/8}/1000=0.00562\f$, \f$1000^{3/8}/1000=0.0133\f$, ..., \f$1000^{7/8}/1000=0.422\f$, \f$1000^{8/8}/1000=1\f$
  Real loadStepGeometricRange;                    //!< AUTO: if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  bool useLoadFactor;                             //!< AUTO: true: compute a load factor \f$\in [0,1]\f$ from static step time; all loads are scaled by the load factor; false: loads are always scaled with 1 -- use this option if time dependent loads use a userFunction
  Real stabilizerODE2term;                        //!< AUTO: add mass-proportional stabilizer term in ODE2 part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  bool adaptiveStep;                              //!< AUTO: true: use step reduction if step fails; false: fixed step size
  Real minimumStepSize;                           //!< AUTO: lower limit of step size, before nonlinear solver stops
  Index verboseMode;                              //!< AUTO: 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  Index verboseModeFile;                          //!< AUTO: same behaviour as verboseMode, but outputs all solver information to file
  std::string preStepPyExecute;                   //!< AUTO: Python code to be executed prior to every load step and after last step, e.g. for postprocessing


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  StaticSolverSettings()
  {
    numberOfLoadSteps = 1;
    loadStepDuration = 1;
    loadStepStart = 0;
    loadStepGeometric = false;
    loadStepGeometricRange = 1000;
    useLoadFactor = true;
    stabilizerODE2term = 0;
    adaptiveStep = true;
    minimumStepSize = 1e-8;
    verboseMode = 1;
    verboseModeFile = 0;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "StaticSolverSettings" << ":\n";
    os << "  newton = " << newton << "\n";
    os << "  numberOfLoadSteps = " << numberOfLoadSteps << "\n";
    os << "  loadStepDuration = " << loadStepDuration << "\n";
    os << "  loadStepStart = " << loadStepStart << "\n";
    os << "  loadStepGeometric = " << loadStepGeometric << "\n";
    os << "  loadStepGeometricRange = " << loadStepGeometricRange << "\n";
    os << "  useLoadFactor = " << useLoadFactor << "\n";
    os << "  stabilizerODE2term = " << stabilizerODE2term << "\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  minimumStepSize = " << minimumStepSize << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
    os << "  preStepPyExecute = " << preStepPyExecute << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const StaticSolverSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SimulationSettings
* @brief        General Settings for simulation; according settings for solution and solvers are given in subitems of this structure
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-12-02 (last modfied)
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

class SimulationSettings // AUTO: 
{
public: // AUTO: 
  TimeIntegrationSettings timeIntegration;        //!< AUTO: time integration parameters
  SolutionSettings solutionSettings;              //!< AUTO: settings for solution files
  StaticSolverSettings staticSolver;              //!< AUTO: static solver parameters
  LinearSolverType linearSolverType;              //!< AUTO: selection of numerical linear solver: exu.LinearSolverType.EXUdense (dense matrix inverse), exu.LinearSolverType.EigenSparse (sparse matrix LU-factorization), ... (enumeration type)
  bool cleanUpMemory;                             //!< AUTO: true: solvers will free memory at exit (recommended for large systems); false: keep allocated memory for repeated computations to increase performance
  bool displayStatistics;                         //!< AUTO: display general computation information at end of time step (steps, iterations, function calls, step rejections, ...
  bool displayComputationTime;                    //!< AUTO: display computation time statistics at end of solving
  bool pauseAfterEachStep;                        //!< AUTO: pause after every time step or static load step(user press SPACE)
  Index outputPrecision;                          //!< AUTO: precision for floating point numbers written to console; e.g. values written by solver
  Index numberOfThreads;                          //!< AUTO: number of threads used for parallel computation (1 == scalar processing); not yet implemented (status: Nov 2019)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SimulationSettings()
  {
    linearSolverType = LinearSolverType::EXUdense;
    cleanUpMemory = false;
    displayStatistics = false;
    displayComputationTime = false;
    pauseAfterEachStep = false;
    outputPrecision = 6;
    numberOfThreads = 1;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SimulationSettings" << ":\n";
    os << "  timeIntegration = " << timeIntegration << "\n";
    os << "  solutionSettings = " << solutionSettings << "\n";
    os << "  staticSolver = " << staticSolver << "\n";
    os << "  linearSolverType = " << linearSolverType << "\n";
    os << "  cleanUpMemory = " << cleanUpMemory << "\n";
    os << "  displayStatistics = " << displayStatistics << "\n";
    os << "  displayComputationTime = " << displayComputationTime << "\n";
    os << "  pauseAfterEachStep = " << pauseAfterEachStep << "\n";
    os << "  outputPrecision = " << outputPrecision << "\n";
    os << "  numberOfThreads = " << numberOfThreads << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SimulationSettings& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
