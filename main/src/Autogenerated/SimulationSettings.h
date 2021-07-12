/** ***********************************************************************************************
* @class        SolutionSettings
* @brief        General settings for exporting the solution (results) of a simulation.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-07-09 (last modfied)
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
  bool writeSolutionToFile;                       //!< AUTO: flag (true/false), which determines if (global) solution vector is written to file; standard quantities that are written are: solution is written as displacements and coordinatesODE1; for additional coordinates in the solution file, see the options below
  bool appendToFile;                              //!< AUTO: flag (true/false); if true, solution and solverInformation is appended to existing file (otherwise created)
  bool writeFileHeader;                           //!< AUTO: flag (true/false); if true, file header is written (turn off, e.g. for multiple runs of time integration)
  bool writeFileFooter;                           //!< AUTO: flag (true/false); if true, information at end of simulation is written: convergence, total solution time, statistics
  Real solutionWritePeriod;                       //!< AUTO: time span (period), determines how often the solution is written during a simulation
  bool exportVelocities;                          //!< AUTO: add ODE2 velocities to solution file
  bool exportAccelerations;                       //!< AUTO: add ODE2 accelerations to solution file
  bool exportODE1Velocities;                      //!< AUTO: add coordinatesODE1\_t to solution file
  bool exportAlgebraicCoordinates;                //!< AUTO: add algebraicCoordinates (=Lagrange multipliers) to solution file
  bool exportDataCoordinates;                     //!< AUTO: add DataCoordinates to solution file
  std::string coordinatesSolutionFileName;        //!< AUTO: filename and (relative) path of solution file containing all coordinates versus time; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '\_' only
  bool sensorsAppendToFile;                       //!< AUTO: flag (true/false); if true, sensor output is appended to existing file (otherwise created)
  bool sensorsWriteFileHeader;                    //!< AUTO: flag (true/false); if true, file header is written for sensor output (turn off, e.g. for multiple runs of time integration)
  Real sensorsWritePeriod;                        //!< AUTO: time span (period), determines how often the sensor output is written during a simulation
  std::string solverInformationFileName;          //!< AUTO: filename and (relative) path of text file showing detailed information during solving; detail level according to yourSolver.verboseModeFile; if solutionSettings.appendToFile is true, the information is appended in every solution step; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '\_' only
  std::string solutionInformation;                //!< AUTO: special information added to header of solution file (e.g. parameters and settings, modes, ...); character encoding my be UTF-8, restricted to characters in \refSection{sec:utf8}, but for compatibility, it is recommended to use ASCII characters only (95 characters, see wiki)
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
    exportVelocities = true;
    exportAccelerations = true;
    exportODE1Velocities = true;
    exportAlgebraicCoordinates = true;
    exportDataCoordinates = true;
    coordinatesSolutionFileName = "coordinatesSolution.txt";
    sensorsAppendToFile = false;
    sensorsWriteFileHeader = true;
    sensorsWritePeriod = 0.01;
    solverInformationFileName = "solverInformation.txt";
    outputPrecision = 10;
    recordImagesInterval = -1.;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: time span (period), determines how often the solution is written during a simulation
  void PySetSolutionWritePeriod(const Real& solutionWritePeriodInit) { solutionWritePeriod = EXUstd::GetSafelyUReal(solutionWritePeriodInit,"solutionWritePeriod"); }
  //! AUTO: Read (Copy) access to: time span (period), determines how often the solution is written during a simulation
  Real PyGetSolutionWritePeriod() const { return (Real)(solutionWritePeriod); }

  //! AUTO: Set function (needed in pybind) for: time span (period), determines how often the sensor output is written during a simulation
  void PySetSensorsWritePeriod(const Real& sensorsWritePeriodInit) { sensorsWritePeriod = EXUstd::GetSafelyUReal(sensorsWritePeriodInit,"sensorsWritePeriod"); }
  //! AUTO: Read (Copy) access to: time span (period), determines how often the sensor output is written during a simulation
  Real PyGetSensorsWritePeriod() const { return (Real)(sensorsWritePeriod); }

  //! AUTO: Set function (needed in pybind) for: precision for floating point numbers written to solution and sensor files
  void PySetOutputPrecision(const Index& outputPrecisionInit) { outputPrecision = EXUstd::GetSafelyUInt(outputPrecisionInit,"outputPrecision"); }
  //! AUTO: Read (Copy) access to: precision for floating point numbers written to solution and sensor files
  Index PyGetOutputPrecision() const { return (Index)(outputPrecision); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionSettings" << ":\n";
    os << "  writeSolutionToFile = " << writeSolutionToFile << "\n";
    os << "  appendToFile = " << appendToFile << "\n";
    os << "  writeFileHeader = " << writeFileHeader << "\n";
    os << "  writeFileFooter = " << writeFileFooter << "\n";
    os << "  solutionWritePeriod = " << solutionWritePeriod << "\n";
    os << "  exportVelocities = " << exportVelocities << "\n";
    os << "  exportAccelerations = " << exportAccelerations << "\n";
    os << "  exportODE1Velocities = " << exportODE1Velocities << "\n";
    os << "  exportAlgebraicCoordinates = " << exportAlgebraicCoordinates << "\n";
    os << "  exportDataCoordinates = " << exportDataCoordinates << "\n";
    os << "  coordinatesSolutionFileName = " << coordinatesSolutionFileName << "\n";
    os << "  sensorsAppendToFile = " << sensorsAppendToFile << "\n";
    os << "  sensorsWriteFileHeader = " << sensorsWriteFileHeader << "\n";
    os << "  sensorsWritePeriod = " << sensorsWritePeriod << "\n";
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
* @date         AUTO: 2021-07-09 (last modfied)
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
  bool doSystemWideDifferentiation;               //!< AUTO: True: system wide differentiation (e.g. all ODE2 equations w.r.t. all ODE2 coordinates); False: only local (object) differentiation
  bool addReferenceCoordinatesToEpsilon;          //!< AUTO: True: for the size estimation of the differentiation parameter, the reference coordinate \f$q^{Ref}_i\f$ is added to ODE2 coordinates --> see; False: only the current coordinate is used for size estimation of the differentiation parameter


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
  //! AUTO: Set function (needed in pybind) for: relative differentiation parameter epsilon; the numerical differentiation parameter \f$\varepsilon\f$ follows from the formula (\f$\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)\f$, with \f$\varepsilon_\mathrm{relative}\f$=relativeEpsilon, \f$q_{min} = \f$minimumCoordinateSize, \f$q_i\f$ is the current coordinate which is differentiated, and \f$qRef_i\f$ is the reference coordinate of the current coordinate
  void PySetRelativeEpsilon(const Real& relativeEpsilonInit) { relativeEpsilon = EXUstd::GetSafelyUReal(relativeEpsilonInit,"relativeEpsilon"); }
  //! AUTO: Read (Copy) access to: relative differentiation parameter epsilon; the numerical differentiation parameter \f$\varepsilon\f$ follows from the formula (\f$\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)\f$, with \f$\varepsilon_\mathrm{relative}\f$=relativeEpsilon, \f$q_{min} = \f$minimumCoordinateSize, \f$q_i\f$ is the current coordinate which is differentiated, and \f$qRef_i\f$ is the reference coordinate of the current coordinate
  Real PyGetRelativeEpsilon() const { return (Real)(relativeEpsilon); }

  //! AUTO: Set function (needed in pybind) for: minimum size of coordinates in relative differentiation parameter
  void PySetMinimumCoordinateSize(const Real& minimumCoordinateSizeInit) { minimumCoordinateSize = EXUstd::GetSafelyUReal(minimumCoordinateSizeInit,"minimumCoordinateSize"); }
  //! AUTO: Read (Copy) access to: minimum size of coordinates in relative differentiation parameter
  Real PyGetMinimumCoordinateSize() const { return (Real)(minimumCoordinateSize); }

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
* @class        DiscontinuousSettings
* @brief        Settings for discontinuous iterations, as in contact, friction, plasticity and general switching phenomena.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-07-09 (last modfied)
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

class DiscontinuousSettings // AUTO: 
{
public: // AUTO: 
  Index maxIterations;                            //!< AUTO: maximum number of discontinuous (post Newton) iterations
  bool ignoreMaxIterations;                       //!< AUTO: continue solver if maximum number of discontinuous (post Newton) iterations is reached (ignore tolerance)
  Real iterationTolerance;                        //!< AUTO: absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  DiscontinuousSettings()
  {
    maxIterations = 5;
    ignoreMaxIterations = true;
    iterationTolerance = 1;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: maximum number of discontinuous (post Newton) iterations
  void PySetMaxIterations(const Index& maxIterationsInit) { maxIterations = EXUstd::GetSafelyUInt(maxIterationsInit,"maxIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of discontinuous (post Newton) iterations
  Index PyGetMaxIterations() const { return (Index)(maxIterations); }

  //! AUTO: Set function (needed in pybind) for: absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
  void PySetIterationTolerance(const Real& iterationToleranceInit) { iterationTolerance = EXUstd::GetSafelyUReal(iterationToleranceInit,"iterationTolerance"); }
  //! AUTO: Read (Copy) access to: absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
  Real PyGetIterationTolerance() const { return (Real)(iterationTolerance); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "DiscontinuousSettings" << ":\n";
    os << "  maxIterations = " << maxIterations << "\n";
    os << "  ignoreMaxIterations = " << ignoreMaxIterations << "\n";
    os << "  iterationTolerance = " << iterationTolerance << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const DiscontinuousSettings& object)
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
* @date         AUTO: 2021-07-09 (last modfied)
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
  Index newtonResidualMode;                       //!< AUTO: 0 ... use residual for computation of error (standard); 1 ... use ODE2 and ODE1 newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  bool adaptInitialResidual;                      //!< AUTO: flag (true/false); false = standard; True: if initialResidual is very small (or zero), it may increas dramatically in first step; to achieve relativeTolerance, the initialResidual will by updated by a higher residual within the first Newton iteration
  Real modifiedNewtonContractivity;               //!< AUTO: maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  bool useModifiedNewton;                         //!< AUTO: True: compute Jacobian only at first step; no Jacobian updates per step; False: Jacobian computed in every step
  bool modifiedNewtonJacUpdatePerStep;            //!< AUTO: True: compute Jacobian at every time step, but not in every iteration (except for bad convergence ==> switch to full Newton)
  Index maxIterations;                            //!< AUTO: maximum number of iterations (including modified + restart Newton steps); after that iterations, the static/dynamic solver stops with error
  Index maxModifiedNewtonIterations;              //!< AUTO: maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  Index maxModifiedNewtonRestartIterations;       //!< AUTO: maximum number of iterations for modified Newton after aJacobian update; after that number of iterations, the full Newton method is started for this step
  Real maximumSolutionNorm;                       //!< AUTO: this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors are larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)


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
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
  void PySetRelativeTolerance(const Real& relativeToleranceInit) { relativeTolerance = EXUstd::GetSafelyUReal(relativeToleranceInit,"relativeTolerance"); }
  //! AUTO: Read (Copy) access to: relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
  Real PyGetRelativeTolerance() const { return (Real)(relativeTolerance); }

  //! AUTO: Set function (needed in pybind) for: absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
  void PySetAbsoluteTolerance(const Real& absoluteToleranceInit) { absoluteTolerance = EXUstd::GetSafelyUReal(absoluteToleranceInit,"absoluteTolerance"); }
  //! AUTO: Read (Copy) access to: absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
  Real PyGetAbsoluteTolerance() const { return (Real)(absoluteTolerance); }

  //! AUTO: Set function (needed in pybind) for: 0 ... use residual for computation of error (standard); 1 ... use ODE2 and ODE1 newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  void PySetNewtonResidualMode(const Index& newtonResidualModeInit) { newtonResidualMode = EXUstd::GetSafelyUInt(newtonResidualModeInit,"newtonResidualMode"); }
  //! AUTO: Read (Copy) access to: 0 ... use residual for computation of error (standard); 1 ... use ODE2 and ODE1 newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  Index PyGetNewtonResidualMode() const { return (Index)(newtonResidualMode); }

  //! AUTO: Set function (needed in pybind) for: maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  void PySetModifiedNewtonContractivity(const Real& modifiedNewtonContractivityInit) { modifiedNewtonContractivity = EXUstd::GetSafelyPReal(modifiedNewtonContractivityInit,"modifiedNewtonContractivity"); }
  //! AUTO: Read (Copy) access to: maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  Real PyGetModifiedNewtonContractivity() const { return (Real)(modifiedNewtonContractivity); }

  //! AUTO: Set function (needed in pybind) for: maximum number of iterations (including modified + restart Newton steps); after that iterations, the static/dynamic solver stops with error
  void PySetMaxIterations(const Index& maxIterationsInit) { maxIterations = EXUstd::GetSafelyUInt(maxIterationsInit,"maxIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of iterations (including modified + restart Newton steps); after that iterations, the static/dynamic solver stops with error
  Index PyGetMaxIterations() const { return (Index)(maxIterations); }

  //! AUTO: Set function (needed in pybind) for: maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  void PySetMaxModifiedNewtonIterations(const Index& maxModifiedNewtonIterationsInit) { maxModifiedNewtonIterations = EXUstd::GetSafelyUInt(maxModifiedNewtonIterationsInit,"maxModifiedNewtonIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  Index PyGetMaxModifiedNewtonIterations() const { return (Index)(maxModifiedNewtonIterations); }

  //! AUTO: Set function (needed in pybind) for: maximum number of iterations for modified Newton after aJacobian update; after that number of iterations, the full Newton method is started for this step
  void PySetMaxModifiedNewtonRestartIterations(const Index& maxModifiedNewtonRestartIterationsInit) { maxModifiedNewtonRestartIterations = EXUstd::GetSafelyUInt(maxModifiedNewtonRestartIterationsInit,"maxModifiedNewtonRestartIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of iterations for modified Newton after aJacobian update; after that number of iterations, the full Newton method is started for this step
  Index PyGetMaxModifiedNewtonRestartIterations() const { return (Index)(maxModifiedNewtonRestartIterations); }

  //! AUTO: Set function (needed in pybind) for: this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors are larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
  void PySetMaximumSolutionNorm(const Real& maximumSolutionNormInit) { maximumSolutionNorm = EXUstd::GetSafelyUReal(maximumSolutionNormInit,"maximumSolutionNorm"); }
  //! AUTO: Read (Copy) access to: this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors are larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
  Real PyGetMaximumSolutionNorm() const { return (Real)(maximumSolutionNorm); }

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
* @date         AUTO: 2021-07-09 (last modfied)
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
  bool computeInitialAccelerations;               //!< AUTO: True: compute initial accelerations from system EOM in acceleration form; NOTE that initial accelerations that are following from user functions in constraints are not considered for now! False: use zero accelerations


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
  //! AUTO: Set function (needed in pybind) for: value beta for Newmark method; default value beta = \f$\frac 1 4\f$ corresponds to (undamped) trapezoidal rule
  void PySetNewmarkBeta(const Real& newmarkBetaInit) { newmarkBeta = EXUstd::GetSafelyUReal(newmarkBetaInit,"newmarkBeta"); }
  //! AUTO: Read (Copy) access to: value beta for Newmark method; default value beta = \f$\frac 1 4\f$ corresponds to (undamped) trapezoidal rule
  Real PyGetNewmarkBeta() const { return (Real)(newmarkBeta); }

  //! AUTO: Set function (needed in pybind) for: value gamma for Newmark method; default value gamma = \f$\frac 1 2\f$ corresponds to (undamped) trapezoidal rule
  void PySetNewmarkGamma(const Real& newmarkGammaInit) { newmarkGamma = EXUstd::GetSafelyUReal(newmarkGammaInit,"newmarkGamma"); }
  //! AUTO: Read (Copy) access to: value gamma for Newmark method; default value gamma = \f$\frac 1 2\f$ corresponds to (undamped) trapezoidal rule
  Real PyGetNewmarkGamma() const { return (Real)(newmarkGamma); }

  //! AUTO: Set function (needed in pybind) for: spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
  void PySetSpectralRadius(const Real& spectralRadiusInit) { spectralRadius = EXUstd::GetSafelyUReal(spectralRadiusInit,"spectralRadius"); }
  //! AUTO: Read (Copy) access to: spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
  Real PyGetSpectralRadius() const { return (Real)(spectralRadius); }

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
* @class        ExplicitIntegrationSettings
* @brief        Settings for generalized-alpha, implicit trapezoidal or Newmark time integration methods.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-07-09 (last modfied)
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

class ExplicitIntegrationSettings // AUTO: 
{
public: // AUTO: 
  bool eliminateConstraints;                      //!< AUTO: True: make explicit solver work for simple CoordinateConstraints, which are eliminated for ground constraints (e.g. fixed nodes in finite element models). False: incompatible constraints are ignored (BE CAREFUL)!
  bool useLieGroupIntegration;                    //!< AUTO: True: use Lie group integration for rigid body nodes; must be turned on for Lie group nodes, but also improves integration of other rigid body nodes. Only available for RK44 integrator.
  DynamicSolverType dynamicSolverType;            //!< AUTO: selection of explicit solver type (DOPRI5, ExplicitEuler, ExplicitMidpoint, RK44, RK67, ...), for detailed description see DynamicSolverType, \refSection{sec:DynamicSolverType}, but only referring to explicit solvers.


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  ExplicitIntegrationSettings()
  {
    eliminateConstraints = true;
    useLieGroupIntegration = true;
    dynamicSolverType = DynamicSolverType::DOPRI5;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "ExplicitIntegrationSettings" << ":\n";
    os << "  eliminateConstraints = " << eliminateConstraints << "\n";
    os << "  useLieGroupIntegration = " << useLieGroupIntegration << "\n";
    os << "  dynamicSolverType = " << dynamicSolverType << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const ExplicitIntegrationSettings& object)
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
* @date         AUTO: 2021-07-09 (last modfied)
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
  DiscontinuousSettings discontinuous;            //!< AUTO: parameters for treatment of discontinuities
  Real startTime;                                 //!< AUTO: \f$t_{start}\f$: start time of time integration (usually set to zero)
  Real endTime;                                   //!< AUTO: \f$t_{end}\f$: end time of time integration
  Index numberOfSteps;                            //!< AUTO: \f$n_{steps}\f$: number of steps in time integration; (maximum) stepSize \f$h\f$ is computed from \f$h = \frac{t_{end} - t_{start}}{n_{steps}}\f$; for automatic stepsize control, this stepSize is the maximum steps size, \f$h_{max} = h\f$
  bool adaptiveStep;                              //!< AUTO: True: the step size may be reduced if step fails; no automatic stepsize control
  Index adaptiveStepRecoverySteps;                //!< AUTO: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  Real adaptiveStepIncrease;                      //!< AUTO: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Real adaptiveStepDecrease;                      //!< AUTO: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  bool automaticStepSize;                         //!< AUTO: True: for specific integrators with error control (e.g., DOPRI5), compute automatic step size based on error estimation; False: constant step size (step may be reduced if adaptiveStep=True); the maximum stepSize reads \f$h = h_{max} = \frac{t_{end} - t_{start}}{n_{steps}}\f$
  Real minimumStepSize;                           //!< AUTO: \f$h_{min}\f$: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
  Real initialStepSize;                           //!< AUTO: \f$h_{init}\f$: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
  Real absoluteTolerance;                         //!< AUTO: \f$a_{tol}\f$: if automaticStepSize=True, absolute tolerance for the error control; must fulfill \f$a_{tol} > 0\f$; see \refSection{sec:ExplicitSolver}
  Real relativeTolerance;                         //!< AUTO: \f$r_{tol}\f$: if automaticStepSize=True, relative tolerance for the error control; must fulfill \f$r_{tol} \ge 0\f$; see \refSection{sec:ExplicitSolver}
  Real stepSizeSafety;                            //!< AUTO: \f$r_{sfty}\f$: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see \refSection{sec:ExplicitSolver}. Make this factor smaller if many steps are rejected.
  Real stepSizeMaxIncrease;                       //!< AUTO: \f$f_{maxInc}\f$: if automaticStepSize=True, maximum increase of step size per step, see \refSection{sec:ExplicitSolver}; make this factor smaller (but \f$> 1\f$) if too many rejected steps
  std::string preStepPyExecute;                   //!< AUTO: DEPRECATED, use mbs.SetPreStepUserFunction(...); Python code to be executed prior to every step and after last step, e.g. for postprocessing
  bool simulateInRealtime;                        //!< AUTO: True: simulate in realtime; the solver waits for computation of the next step until the CPU time reached the simulation time; if the simulation is slower than realtime, it simply continues
  Real realtimeFactor;                            //!< AUTO: if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
  Index verboseMode;                              //!< AUTO: 0 ... no output, 1 ... show short step information every 2 seconds (error), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  Index verboseModeFile;                          //!< AUTO: same behaviour as verboseMode, but outputs all solver information to file
  Index stepInformation;                          //!< AUTO: 0 ... only current step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step, 3 ... show discontinuous iterations (Dit) and newton jacobians (jac) per step
  GeneralizedAlphaSettings generalizedAlpha;      //!< AUTO: parameters for generalized-alpha, implicit trapezoidal rule or Newmark (options only apply for these methods)
  ExplicitIntegrationSettings explicitIntegration;//!< AUTO: special parameters for explicit time integration


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  TimeIntegrationSettings()
  {
    startTime = 0;
    endTime = 1;
    numberOfSteps = 100;
    adaptiveStep = true;
    adaptiveStepRecoverySteps = 10;
    adaptiveStepIncrease = 2;
    adaptiveStepDecrease = 0.5;
    automaticStepSize = true;
    minimumStepSize = 1e-8;
    initialStepSize = 0;
    absoluteTolerance = 1e-8;
    relativeTolerance = 1e-8;
    stepSizeSafety = 0.90;
    stepSizeMaxIncrease = 2;
    simulateInRealtime = false;
    realtimeFactor = 1;
    verboseMode = 0;
    verboseModeFile = 0;
    stepInformation = 2;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: \f$t_{start}\f$: start time of time integration (usually set to zero)
  void PySetStartTime(const Real& startTimeInit) { startTime = EXUstd::GetSafelyUReal(startTimeInit,"startTime"); }
  //! AUTO: Read (Copy) access to: \f$t_{start}\f$: start time of time integration (usually set to zero)
  Real PyGetStartTime() const { return (Real)(startTime); }

  //! AUTO: Set function (needed in pybind) for: \f$t_{end}\f$: end time of time integration
  void PySetEndTime(const Real& endTimeInit) { endTime = EXUstd::GetSafelyUReal(endTimeInit,"endTime"); }
  //! AUTO: Read (Copy) access to: \f$t_{end}\f$: end time of time integration
  Real PyGetEndTime() const { return (Real)(endTime); }

  //! AUTO: Set function (needed in pybind) for: \f$n_{steps}\f$: number of steps in time integration; (maximum) stepSize \f$h\f$ is computed from \f$h = \frac{t_{end} - t_{start}}{n_{steps}}\f$; for automatic stepsize control, this stepSize is the maximum steps size, \f$h_{max} = h\f$
  void PySetNumberOfSteps(const Index& numberOfStepsInit) { numberOfSteps = EXUstd::GetSafelyPInt(numberOfStepsInit,"numberOfSteps"); }
  //! AUTO: Read (Copy) access to: \f$n_{steps}\f$: number of steps in time integration; (maximum) stepSize \f$h\f$ is computed from \f$h = \frac{t_{end} - t_{start}}{n_{steps}}\f$; for automatic stepsize control, this stepSize is the maximum steps size, \f$h_{max} = h\f$
  Index PyGetNumberOfSteps() const { return (Index)(numberOfSteps); }

  //! AUTO: Set function (needed in pybind) for: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepRecoverySteps(const Index& adaptiveStepRecoveryStepsInit) { adaptiveStepRecoverySteps = EXUstd::GetSafelyUInt(adaptiveStepRecoveryStepsInit,"adaptiveStepRecoverySteps"); }
  //! AUTO: Read (Copy) access to: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  Index PyGetAdaptiveStepRecoverySteps() const { return (Index)(adaptiveStepRecoverySteps); }

  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepIncrease(const Real& adaptiveStepIncreaseInit) { adaptiveStepIncrease = EXUstd::GetSafelyUReal(adaptiveStepIncreaseInit,"adaptiveStepIncrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepIncrease() const { return (Real)(adaptiveStepIncrease); }

  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepDecrease(const Real& adaptiveStepDecreaseInit) { adaptiveStepDecrease = EXUstd::GetSafelyUReal(adaptiveStepDecreaseInit,"adaptiveStepDecrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepDecrease() const { return (Real)(adaptiveStepDecrease); }

  //! AUTO: Set function (needed in pybind) for: \f$h_{min}\f$: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
  void PySetMinimumStepSize(const Real& minimumStepSizeInit) { minimumStepSize = EXUstd::GetSafelyPReal(minimumStepSizeInit,"minimumStepSize"); }
  //! AUTO: Read (Copy) access to: \f$h_{min}\f$: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
  Real PyGetMinimumStepSize() const { return (Real)(minimumStepSize); }

  //! AUTO: Set function (needed in pybind) for: \f$h_{init}\f$: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
  void PySetInitialStepSize(const Real& initialStepSizeInit) { initialStepSize = EXUstd::GetSafelyUReal(initialStepSizeInit,"initialStepSize"); }
  //! AUTO: Read (Copy) access to: \f$h_{init}\f$: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
  Real PyGetInitialStepSize() const { return (Real)(initialStepSize); }

  //! AUTO: Set function (needed in pybind) for: \f$a_{tol}\f$: if automaticStepSize=True, absolute tolerance for the error control; must fulfill \f$a_{tol} > 0\f$; see \refSection{sec:ExplicitSolver}
  void PySetAbsoluteTolerance(const Real& absoluteToleranceInit) { absoluteTolerance = EXUstd::GetSafelyUReal(absoluteToleranceInit,"absoluteTolerance"); }
  //! AUTO: Read (Copy) access to: \f$a_{tol}\f$: if automaticStepSize=True, absolute tolerance for the error control; must fulfill \f$a_{tol} > 0\f$; see \refSection{sec:ExplicitSolver}
  Real PyGetAbsoluteTolerance() const { return (Real)(absoluteTolerance); }

  //! AUTO: Set function (needed in pybind) for: \f$r_{tol}\f$: if automaticStepSize=True, relative tolerance for the error control; must fulfill \f$r_{tol} \ge 0\f$; see \refSection{sec:ExplicitSolver}
  void PySetRelativeTolerance(const Real& relativeToleranceInit) { relativeTolerance = EXUstd::GetSafelyUReal(relativeToleranceInit,"relativeTolerance"); }
  //! AUTO: Read (Copy) access to: \f$r_{tol}\f$: if automaticStepSize=True, relative tolerance for the error control; must fulfill \f$r_{tol} \ge 0\f$; see \refSection{sec:ExplicitSolver}
  Real PyGetRelativeTolerance() const { return (Real)(relativeTolerance); }

  //! AUTO: Set function (needed in pybind) for: \f$r_{sfty}\f$: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see \refSection{sec:ExplicitSolver}. Make this factor smaller if many steps are rejected.
  void PySetStepSizeSafety(const Real& stepSizeSafetyInit) { stepSizeSafety = EXUstd::GetSafelyUReal(stepSizeSafetyInit,"stepSizeSafety"); }
  //! AUTO: Read (Copy) access to: \f$r_{sfty}\f$: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see \refSection{sec:ExplicitSolver}. Make this factor smaller if many steps are rejected.
  Real PyGetStepSizeSafety() const { return (Real)(stepSizeSafety); }

  //! AUTO: Set function (needed in pybind) for: \f$f_{maxInc}\f$: if automaticStepSize=True, maximum increase of step size per step, see \refSection{sec:ExplicitSolver}; make this factor smaller (but \f$> 1\f$) if too many rejected steps
  void PySetStepSizeMaxIncrease(const Real& stepSizeMaxIncreaseInit) { stepSizeMaxIncrease = EXUstd::GetSafelyUReal(stepSizeMaxIncreaseInit,"stepSizeMaxIncrease"); }
  //! AUTO: Read (Copy) access to: \f$f_{maxInc}\f$: if automaticStepSize=True, maximum increase of step size per step, see \refSection{sec:ExplicitSolver}; make this factor smaller (but \f$> 1\f$) if too many rejected steps
  Real PyGetStepSizeMaxIncrease() const { return (Real)(stepSizeMaxIncrease); }

  //! AUTO: Set function (needed in pybind) for: if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
  void PySetRealtimeFactor(const Real& realtimeFactorInit) { realtimeFactor = EXUstd::GetSafelyPReal(realtimeFactorInit,"realtimeFactor"); }
  //! AUTO: Read (Copy) access to: if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
  Real PyGetRealtimeFactor() const { return (Real)(realtimeFactor); }

  //! AUTO: Set function (needed in pybind) for: 0 ... no output, 1 ... show short step information every 2 seconds (error), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  void PySetVerboseMode(const Index& verboseModeInit) { verboseMode = EXUstd::GetSafelyUInt(verboseModeInit,"verboseMode"); }
  //! AUTO: Read (Copy) access to: 0 ... no output, 1 ... show short step information every 2 seconds (error), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  Index PyGetVerboseMode() const { return (Index)(verboseMode); }

  //! AUTO: Set function (needed in pybind) for: same behaviour as verboseMode, but outputs all solver information to file
  void PySetVerboseModeFile(const Index& verboseModeFileInit) { verboseModeFile = EXUstd::GetSafelyUInt(verboseModeFileInit,"verboseModeFile"); }
  //! AUTO: Read (Copy) access to: same behaviour as verboseMode, but outputs all solver information to file
  Index PyGetVerboseModeFile() const { return (Index)(verboseModeFile); }

  //! AUTO: Set function (needed in pybind) for: 0 ... only current step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step, 3 ... show discontinuous iterations (Dit) and newton jacobians (jac) per step
  void PySetStepInformation(const Index& stepInformationInit) { stepInformation = EXUstd::GetSafelyUInt(stepInformationInit,"stepInformation"); }
  //! AUTO: Read (Copy) access to: 0 ... only current step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step, 3 ... show discontinuous iterations (Dit) and newton jacobians (jac) per step
  Index PyGetStepInformation() const { return (Index)(stepInformation); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "TimeIntegrationSettings" << ":\n";
    os << "  newton = " << newton << "\n";
    os << "  discontinuous = " << discontinuous << "\n";
    os << "  startTime = " << startTime << "\n";
    os << "  endTime = " << endTime << "\n";
    os << "  numberOfSteps = " << numberOfSteps << "\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  adaptiveStepRecoverySteps = " << adaptiveStepRecoverySteps << "\n";
    os << "  adaptiveStepIncrease = " << adaptiveStepIncrease << "\n";
    os << "  adaptiveStepDecrease = " << adaptiveStepDecrease << "\n";
    os << "  automaticStepSize = " << automaticStepSize << "\n";
    os << "  minimumStepSize = " << minimumStepSize << "\n";
    os << "  initialStepSize = " << initialStepSize << "\n";
    os << "  absoluteTolerance = " << absoluteTolerance << "\n";
    os << "  relativeTolerance = " << relativeTolerance << "\n";
    os << "  stepSizeSafety = " << stepSizeSafety << "\n";
    os << "  stepSizeMaxIncrease = " << stepSizeMaxIncrease << "\n";
    os << "  preStepPyExecute = " << preStepPyExecute << "\n";
    os << "  simulateInRealtime = " << simulateInRealtime << "\n";
    os << "  realtimeFactor = " << realtimeFactor << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
    os << "  stepInformation = " << stepInformation << "\n";
    os << "  generalizedAlpha = " << generalizedAlpha << "\n";
    os << "  explicitIntegration = " << explicitIntegration << "\n";
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
* @date         AUTO: 2021-07-09 (last modfied)
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
  DiscontinuousSettings discontinuous;            //!< AUTO: parameters for treatment of discontinuities
  Index numberOfLoadSteps;                        //!< AUTO: number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  Real loadStepDuration;                          //!< AUTO: quasi-time for all load steps (added to current time in load steps)
  Real loadStepStart;                             //!< AUTO: a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  bool loadStepGeometric;                         //!< AUTO: if loadStepGeometric=false, the load steps are incremental (arithmetic series, e.g. 0.1,0.2,0.3,...); if true, the load steps are increased in a geometric series, e.g. for \f$n=8\f$ numberOfLoadSteps and \f$d = 1000\f$ loadStepGeometricRange, it follows: \f$1000^{1/8}/1000=0.00237\f$, \f$1000^{2/8}/1000=0.00562\f$, \f$1000^{3/8}/1000=0.0133\f$, ..., \f$1000^{7/8}/1000=0.422\f$, \f$1000^{8/8}/1000=1\f$
  Real loadStepGeometricRange;                    //!< AUTO: if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  bool useLoadFactor;                             //!< AUTO: True: compute a load factor \f$\in [0,1]\f$ from static step time; all loads are scaled by the load factor; False: loads are always scaled with 1 -- use this option if time dependent loads use a userFunction
  Real stabilizerODE2term;                        //!< AUTO: add mass-proportional stabilizer term in ODE2 part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  bool adaptiveStep;                              //!< AUTO: True: use step reduction if step fails; False: fixed step size
  Index adaptiveStepRecoverySteps;                //!< AUTO: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  Real adaptiveStepIncrease;                      //!< AUTO: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Real adaptiveStepDecrease;                      //!< AUTO: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real minimumStepSize;                           //!< AUTO: lower limit of step size, before nonlinear solver stops
  Index verboseMode;                              //!< AUTO: 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  Index verboseModeFile;                          //!< AUTO: same behaviour as verboseMode, but outputs all solver information to file
  Index stepInformation;                          //!< AUTO: 0 ... only current step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step, 3 ... show discontinuous iterations (Dit) and newton jacobians (jac) per step
  std::string preStepPyExecute;                   //!< AUTO: DEPRECATED, use mbs.SetPreStepUserFunction(...); Python code to be executed prior to every load step and after last step, e.g. for postprocessing


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
    adaptiveStepRecoverySteps = 4;
    adaptiveStepIncrease = 2;
    adaptiveStepDecrease = 0.25;
    minimumStepSize = 1e-8;
    verboseMode = 1;
    verboseModeFile = 0;
    stepInformation = 2;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  void PySetNumberOfLoadSteps(const Index& numberOfLoadStepsInit) { numberOfLoadSteps = EXUstd::GetSafelyPInt(numberOfLoadStepsInit,"numberOfLoadSteps"); }
  //! AUTO: Read (Copy) access to: number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  Index PyGetNumberOfLoadSteps() const { return (Index)(numberOfLoadSteps); }

  //! AUTO: Set function (needed in pybind) for: quasi-time for all load steps (added to current time in load steps)
  void PySetLoadStepDuration(const Real& loadStepDurationInit) { loadStepDuration = EXUstd::GetSafelyPReal(loadStepDurationInit,"loadStepDuration"); }
  //! AUTO: Read (Copy) access to: quasi-time for all load steps (added to current time in load steps)
  Real PyGetLoadStepDuration() const { return (Real)(loadStepDuration); }

  //! AUTO: Set function (needed in pybind) for: a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  void PySetLoadStepStart(const Real& loadStepStartInit) { loadStepStart = EXUstd::GetSafelyUReal(loadStepStartInit,"loadStepStart"); }
  //! AUTO: Read (Copy) access to: a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  Real PyGetLoadStepStart() const { return (Real)(loadStepStart); }

  //! AUTO: Set function (needed in pybind) for: if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  void PySetLoadStepGeometricRange(const Real& loadStepGeometricRangeInit) { loadStepGeometricRange = EXUstd::GetSafelyPReal(loadStepGeometricRangeInit,"loadStepGeometricRange"); }
  //! AUTO: Read (Copy) access to: if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  Real PyGetLoadStepGeometricRange() const { return (Real)(loadStepGeometricRange); }

  //! AUTO: Set function (needed in pybind) for: add mass-proportional stabilizer term in ODE2 part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  void PySetStabilizerODE2term(const Real& stabilizerODE2termInit) { stabilizerODE2term = EXUstd::GetSafelyUReal(stabilizerODE2termInit,"stabilizerODE2term"); }
  //! AUTO: Read (Copy) access to: add mass-proportional stabilizer term in ODE2 part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  Real PyGetStabilizerODE2term() const { return (Real)(stabilizerODE2term); }

  //! AUTO: Set function (needed in pybind) for: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepRecoverySteps(const Index& adaptiveStepRecoveryStepsInit) { adaptiveStepRecoverySteps = EXUstd::GetSafelyUInt(adaptiveStepRecoveryStepsInit,"adaptiveStepRecoverySteps"); }
  //! AUTO: Read (Copy) access to: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  Index PyGetAdaptiveStepRecoverySteps() const { return (Index)(adaptiveStepRecoverySteps); }

  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepIncrease(const Real& adaptiveStepIncreaseInit) { adaptiveStepIncrease = EXUstd::GetSafelyUReal(adaptiveStepIncreaseInit,"adaptiveStepIncrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepIncrease() const { return (Real)(adaptiveStepIncrease); }

  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepDecrease(const Real& adaptiveStepDecreaseInit) { adaptiveStepDecrease = EXUstd::GetSafelyUReal(adaptiveStepDecreaseInit,"adaptiveStepDecrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepDecrease() const { return (Real)(adaptiveStepDecrease); }

  //! AUTO: Set function (needed in pybind) for: lower limit of step size, before nonlinear solver stops
  void PySetMinimumStepSize(const Real& minimumStepSizeInit) { minimumStepSize = EXUstd::GetSafelyPReal(minimumStepSizeInit,"minimumStepSize"); }
  //! AUTO: Read (Copy) access to: lower limit of step size, before nonlinear solver stops
  Real PyGetMinimumStepSize() const { return (Real)(minimumStepSize); }

  //! AUTO: Set function (needed in pybind) for: 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  void PySetVerboseMode(const Index& verboseModeInit) { verboseMode = EXUstd::GetSafelyUInt(verboseModeInit,"verboseMode"); }
  //! AUTO: Read (Copy) access to: 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  Index PyGetVerboseMode() const { return (Index)(verboseMode); }

  //! AUTO: Set function (needed in pybind) for: same behaviour as verboseMode, but outputs all solver information to file
  void PySetVerboseModeFile(const Index& verboseModeFileInit) { verboseModeFile = EXUstd::GetSafelyUInt(verboseModeFileInit,"verboseModeFile"); }
  //! AUTO: Read (Copy) access to: same behaviour as verboseMode, but outputs all solver information to file
  Index PyGetVerboseModeFile() const { return (Index)(verboseModeFile); }

  //! AUTO: Set function (needed in pybind) for: 0 ... only current step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step, 3 ... show discontinuous iterations (Dit) and newton jacobians (jac) per step
  void PySetStepInformation(const Index& stepInformationInit) { stepInformation = EXUstd::GetSafelyUInt(stepInformationInit,"stepInformation"); }
  //! AUTO: Read (Copy) access to: 0 ... only current step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step, 3 ... show discontinuous iterations (Dit) and newton jacobians (jac) per step
  Index PyGetStepInformation() const { return (Index)(stepInformation); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "StaticSolverSettings" << ":\n";
    os << "  newton = " << newton << "\n";
    os << "  discontinuous = " << discontinuous << "\n";
    os << "  numberOfLoadSteps = " << numberOfLoadSteps << "\n";
    os << "  loadStepDuration = " << loadStepDuration << "\n";
    os << "  loadStepStart = " << loadStepStart << "\n";
    os << "  loadStepGeometric = " << loadStepGeometric << "\n";
    os << "  loadStepGeometricRange = " << loadStepGeometricRange << "\n";
    os << "  useLoadFactor = " << useLoadFactor << "\n";
    os << "  stabilizerODE2term = " << stabilizerODE2term << "\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  adaptiveStepRecoverySteps = " << adaptiveStepRecoverySteps << "\n";
    os << "  adaptiveStepIncrease = " << adaptiveStepIncrease << "\n";
    os << "  adaptiveStepDecrease = " << adaptiveStepDecrease << "\n";
    os << "  minimumStepSize = " << minimumStepSize << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
    os << "  stepInformation = " << stepInformation << "\n";
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
* @class        LinearSolverSettings
* @brief        Settings for linear solver, both dense and sparse (Eigen).
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-07-09 (last modfied)
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

class LinearSolverSettings // AUTO: 
{
public: // AUTO: 
  Real pivotTreshold;                             //!< AUTO: treshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
  bool ignoreRedundantConstraints;                //!< AUTO: [ONLY implemented for dense matrices] False: standard way, fails if redundant equations or singular matrices occur; True: if redundant constraints appear, the solver tries to resolve them by setting according Lagrange multipliers to zero; in case of redundant constraints, this may help, but it may lead to erroneous behaviour
  bool ignoreSingularJacobian;                    //!< AUTO: [ONLY implemented for dense matrices] False: standard way, fails if jacobian is singular; True: if singularities appear in jacobian (e.g. no equation attributed to a node, redundant equations, zero mass matrix, zero eigenvalue for static problem, etc.), the jacobian inverse is resolved such that according solution variables are set to zero; this may help, but it MAY LEAD TO ERRONEOUS BEHAVIOUR; for static problems, this may suppress static motion or resolve problems in case of instabilities, but should in general be considered with care!
  bool showCausingItems;                          //!< AUTO: False: no output, if solver fails; True: if redundant equations appear, they are resolved such that according solution variables are set to zero; in case of redundant constraints, this may help, but it may lead to erroneous behaviour; for static problems, this may suppress static motion or resolve problems in case of instabilities, but should in general be considered with care!


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  LinearSolverSettings()
  {
    pivotTreshold = 0;
    ignoreRedundantConstraints = false;
    ignoreSingularJacobian = false;
    showCausingItems = true;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: treshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
  void PySetPivotTreshold(const Real& pivotTresholdInit) { pivotTreshold = EXUstd::GetSafelyPReal(pivotTresholdInit,"pivotTreshold"); }
  //! AUTO: Read (Copy) access to: treshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
  Real PyGetPivotTreshold() const { return (Real)(pivotTreshold); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "LinearSolverSettings" << ":\n";
    os << "  pivotTreshold = " << pivotTreshold << "\n";
    os << "  ignoreRedundantConstraints = " << ignoreRedundantConstraints << "\n";
    os << "  ignoreSingularJacobian = " << ignoreSingularJacobian << "\n";
    os << "  showCausingItems = " << showCausingItems << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const LinearSolverSettings& object)
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
* @date         AUTO: 2021-07-09 (last modfied)
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
  LinearSolverSettings linearSolverSettings;      //!< AUTO: linear solver parameters (used for dense and sparse solvers)
  LinearSolverType linearSolverType;              //!< AUTO: selection of numerical linear solver: exu.LinearSolverType.EXUdense (dense matrix inverse), exu.LinearSolverType.EigenSparse (sparse matrix LU-factorization), ... (enumeration type)
  bool cleanUpMemory;                             //!< AUTO: True: solvers will free memory at exit (recommended for large systems); False: keep allocated memory for repeated computations to increase performance
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
  //! AUTO: Set function (needed in pybind) for: precision for floating point numbers written to console; e.g. values written by solver
  void PySetOutputPrecision(const Index& outputPrecisionInit) { outputPrecision = EXUstd::GetSafelyUInt(outputPrecisionInit,"outputPrecision"); }
  //! AUTO: Read (Copy) access to: precision for floating point numbers written to console; e.g. values written by solver
  Index PyGetOutputPrecision() const { return (Index)(outputPrecision); }

  //! AUTO: Set function (needed in pybind) for: number of threads used for parallel computation (1 == scalar processing); not yet implemented (status: Nov 2019)
  void PySetNumberOfThreads(const Index& numberOfThreadsInit) { numberOfThreads = EXUstd::GetSafelyPInt(numberOfThreadsInit,"numberOfThreads"); }
  //! AUTO: Read (Copy) access to: number of threads used for parallel computation (1 == scalar processing); not yet implemented (status: Nov 2019)
  Index PyGetNumberOfThreads() const { return (Index)(numberOfThreads); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SimulationSettings" << ":\n";
    os << "  timeIntegration = " << timeIntegration << "\n";
    os << "  solutionSettings = " << solutionSettings << "\n";
    os << "  staticSolver = " << staticSolver << "\n";
    os << "  linearSolverSettings = " << linearSolverSettings << "\n";
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
