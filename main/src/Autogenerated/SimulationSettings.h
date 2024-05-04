/** ***********************************************************************************************
* @class        SolutionSettings
* @brief        General settings for exporting the solution (results) of a simulation.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2024-04-01 (last modfied)
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
  bool appendToFile;                              //!< AUTO: flag (true/false); if true, solution and solverInformation is appended to existing file (otherwise created); in BINARY mode, files are always replaced and this parameter is ineffective!
  bool binarySolutionFile;                        //!< AUTO: if true, the solution file is written in binary format for improved speed and smaller file sizes; setting outputPrecision >= 8 uses double (8 bytes), otherwise float (4 bytes) is used; note that appendToFile is ineffective and files are always replaced without asking! If not provided, file ending will read .sol in case of binary files and .txt in case of text files
  std::string coordinatesSolutionFileName;        //!< AUTO: filename and (relative) path of solution file (coordinatesSolutionFile) containing all multibody system coordinates versus time; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only; filename ending will be added automatically if not provided: .txt in case of text mode and .sol in case of binary solution files (binarySolutionFile=True)
  bool exportAccelerations;                       //!< AUTO: add \hac{ODE2} accelerations to solution file (coordinatesSolutionFile)
  bool exportAlgebraicCoordinates;                //!< AUTO: add algebraicCoordinates (=Lagrange multipliers) to solution file (coordinatesSolutionFile)
  bool exportDataCoordinates;                     //!< AUTO: add DataCoordinates to solution file (coordinatesSolutionFile)
  bool exportODE1Velocities;                      //!< AUTO: add coordinatesODE1_t to solution file (coordinatesSolutionFile)
  bool exportVelocities;                          //!< AUTO: add \hac{ODE2} velocities to solution file (coordinatesSolutionFile)
  Index flushFilesDOF;                            //!< AUTO: number of DOF, above which solution file (coordinatesSolutionFile) buffers are always flushed, irrespectively of whether flushFilesImmediately is set True or False (see also flushFilesImmediately); for larger files, writing takes so much time that flushing does not add considerable time
  bool flushFilesImmediately;                     //!< AUTO: flush file buffers after every solution period written (coordinatesSolutionFile and sensor files); if set False, the output is written through a buffer, which is highly efficient, but during simulation, files may be always in an incomplete state; if set True, this may add a large amount of CPU time as the process waits until files are really written to hard disc (especially for simulation of small scale systems, writing 10.000s of time steps; at least 5us per step/file, depending on hardware)
  Index outputPrecision;                          //!< AUTO: precision for floating point numbers written to solution and sensor files
  Real recordImagesInterval;                      //!< AUTO: record frames (images) during solving: amount of time to wait until next image (frame) is recorded; set recordImages = -1. if no images shall be recorded; set, e.g., recordImages = 0.01 to record an image every 10 milliseconds (requires that the time steps / load steps are sufficiently small!); for file names, etc., see VisualizationSettings.exportImages
  std::string restartFileName;                    //!< AUTO: filename and (relative) path of text file for storing solution after every restartWritePeriod if writeRestartFile=True; backup file is created with ending .bck, which should be used if restart file is crashed; use Python utility function InitializeFromRestartFile(...) to consistently restart
  Real restartWritePeriod;                        //!< AUTO: time span (period), determines how often the restart file is updated; this should be often enough to enable restart without too much loss of data; too low values may influence performance
  bool sensorsAppendToFile;                       //!< AUTO: flag (true/false); if true, sensor output is appended to existing file (otherwise created) or in case of internal storage, it is appended to existing currently stored data; this allows storing sensor values over different simulations
  bool sensorsStoreAndWriteFiles;                 //!< AUTO: flag (true/false); if false, no sensor files will be created and no sensor data will be stored; this may be advantageous for benchmarking as well as for special solvers which should not overwrite existing results (e.g. ComputeODE2Eigenvalues); settings this value to False may cause problems if sensors are required to perform operations which are needed e.g. in UserSensors as input of loads, etc.
  bool sensorsWriteFileFooter;                    //!< AUTO: flag (true/false); if true, file footer is written for sensor output (turn off, e.g. for multiple runs of time integration)
  bool sensorsWriteFileHeader;                    //!< AUTO: flag (true/false); if true, file header is written for sensor output (turn off, e.g. for multiple runs of time integration)
  Real sensorsWritePeriod;                        //!< AUTO: time span (period), determines how often the sensor output is written to file or internal storage during a simulation
  std::string solutionInformation;                //!< AUTO: special information added to header of solution file (e.g. parameters and settings, modes, ...); character encoding my be UTF-8, restricted to characters in \refSection{sec:utf8}, but for compatibility, it is recommended to use ASCII characters only (95 characters, see wiki)
  Real solutionWritePeriod;                       //!< AUTO: time span (period), determines how often the solution file (coordinatesSolutionFile) is written during a simulation
  std::string solverInformationFileName;          //!< AUTO: filename and (relative) path of text file showing detailed information during solving; detail level according to yourSolver.verboseModeFile; if solutionSettings.appendToFile is true, the information is appended in every solution step; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only
  bool writeFileFooter;                           //!< AUTO: flag (true/false); if true, information at end of simulation is written: convergence, total solution time, statistics
  bool writeFileHeader;                           //!< AUTO: flag (true/false); if true, file header is written (turn off, e.g. for multiple runs of time integration)
  bool writeInitialValues;                        //!< AUTO: flag (true/false); if true, initial values are exported for the start time; applies to coordinatesSolution and sensor files; this may not be wanted in the append file mode if the initial values are identical to the final values of a previous computation
  bool writeRestartFile;                          //!< AUTO: flag (true/false), which determines if restart file is written regularly, see restartFileName for details
  bool writeSolutionToFile;                       //!< AUTO: flag (true/false), which determines if (global) solution vector is written to the solution file (coordinatesSolutionFile); standard quantities that are written are: solution is written as displacements and coordinatesODE1; for additional coordinates in the solution file, see the options below


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionSettings()
  {
    appendToFile = false;
    binarySolutionFile = false;
    coordinatesSolutionFileName = "coordinatesSolution";
    exportAccelerations = true;
    exportAlgebraicCoordinates = true;
    exportDataCoordinates = true;
    exportODE1Velocities = true;
    exportVelocities = true;
    flushFilesDOF = 10000;
    flushFilesImmediately = false;
    outputPrecision = 10;
    recordImagesInterval = -1.;
    restartFileName = "restartFile.txt";
    restartWritePeriod = 0.01;
    sensorsAppendToFile = false;
    sensorsStoreAndWriteFiles = true;
    sensorsWriteFileFooter = false;
    sensorsWriteFileHeader = true;
    sensorsWritePeriod = 0.01;
    solutionWritePeriod = 0.01;
    solverInformationFileName = "solverInformation.txt";
    writeFileFooter = true;
    writeFileHeader = true;
    writeInitialValues = true;
    writeRestartFile = false;
    writeSolutionToFile = true;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: number of DOF, above which solution file (coordinatesSolutionFile) buffers are always flushed, irrespectively of whether flushFilesImmediately is set True or False (see also flushFilesImmediately); for larger files, writing takes so much time that flushing does not add considerable time
  void PySetFlushFilesDOF(const Index& flushFilesDOFInit) { flushFilesDOF = EXUstd::GetSafelyPInt(flushFilesDOFInit,"flushFilesDOF"); }
  //! AUTO: Read (Copy) access to: number of DOF, above which solution file (coordinatesSolutionFile) buffers are always flushed, irrespectively of whether flushFilesImmediately is set True or False (see also flushFilesImmediately); for larger files, writing takes so much time that flushing does not add considerable time
  Index PyGetFlushFilesDOF() const { return Index(flushFilesDOF); }

  //! AUTO: Set function (needed in pybind) for: precision for floating point numbers written to solution and sensor files
  void PySetOutputPrecision(const Index& outputPrecisionInit) { outputPrecision = EXUstd::GetSafelyUInt(outputPrecisionInit,"outputPrecision"); }
  //! AUTO: Read (Copy) access to: precision for floating point numbers written to solution and sensor files
  Index PyGetOutputPrecision() const { return Index(outputPrecision); }

  //! AUTO: Set function (needed in pybind) for: time span (period), determines how often the restart file is updated; this should be often enough to enable restart without too much loss of data; too low values may influence performance
  void PySetRestartWritePeriod(const Real& restartWritePeriodInit) { restartWritePeriod = EXUstd::GetSafelyUReal(restartWritePeriodInit,"restartWritePeriod"); }
  //! AUTO: Read (Copy) access to: time span (period), determines how often the restart file is updated; this should be often enough to enable restart without too much loss of data; too low values may influence performance
  Real PyGetRestartWritePeriod() const { return Real(restartWritePeriod); }

  //! AUTO: Set function (needed in pybind) for: time span (period), determines how often the sensor output is written to file or internal storage during a simulation
  void PySetSensorsWritePeriod(const Real& sensorsWritePeriodInit) { sensorsWritePeriod = EXUstd::GetSafelyUReal(sensorsWritePeriodInit,"sensorsWritePeriod"); }
  //! AUTO: Read (Copy) access to: time span (period), determines how often the sensor output is written to file or internal storage during a simulation
  Real PyGetSensorsWritePeriod() const { return Real(sensorsWritePeriod); }

  //! AUTO: Set function (needed in pybind) for: time span (period), determines how often the solution file (coordinatesSolutionFile) is written during a simulation
  void PySetSolutionWritePeriod(const Real& solutionWritePeriodInit) { solutionWritePeriod = EXUstd::GetSafelyUReal(solutionWritePeriodInit,"solutionWritePeriod"); }
  //! AUTO: Read (Copy) access to: time span (period), determines how often the solution file (coordinatesSolutionFile) is written during a simulation
  Real PyGetSolutionWritePeriod() const { return Real(solutionWritePeriod); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionSettings" << ":\n";
    os << "  appendToFile = " << appendToFile << "\n";
    os << "  binarySolutionFile = " << binarySolutionFile << "\n";
    os << "  coordinatesSolutionFileName = " << coordinatesSolutionFileName << "\n";
    os << "  exportAccelerations = " << exportAccelerations << "\n";
    os << "  exportAlgebraicCoordinates = " << exportAlgebraicCoordinates << "\n";
    os << "  exportDataCoordinates = " << exportDataCoordinates << "\n";
    os << "  exportODE1Velocities = " << exportODE1Velocities << "\n";
    os << "  exportVelocities = " << exportVelocities << "\n";
    os << "  flushFilesDOF = " << flushFilesDOF << "\n";
    os << "  flushFilesImmediately = " << flushFilesImmediately << "\n";
    os << "  outputPrecision = " << outputPrecision << "\n";
    os << "  recordImagesInterval = " << recordImagesInterval << "\n";
    os << "  restartFileName = " << restartFileName << "\n";
    os << "  restartWritePeriod = " << restartWritePeriod << "\n";
    os << "  sensorsAppendToFile = " << sensorsAppendToFile << "\n";
    os << "  sensorsStoreAndWriteFiles = " << sensorsStoreAndWriteFiles << "\n";
    os << "  sensorsWriteFileFooter = " << sensorsWriteFileFooter << "\n";
    os << "  sensorsWriteFileHeader = " << sensorsWriteFileHeader << "\n";
    os << "  sensorsWritePeriod = " << sensorsWritePeriod << "\n";
    os << "  solutionInformation = " << solutionInformation << "\n";
    os << "  solutionWritePeriod = " << solutionWritePeriod << "\n";
    os << "  solverInformationFileName = " << solverInformationFileName << "\n";
    os << "  writeFileFooter = " << writeFileFooter << "\n";
    os << "  writeFileHeader = " << writeFileHeader << "\n";
    os << "  writeInitialValues = " << writeInitialValues << "\n";
    os << "  writeRestartFile = " << writeRestartFile << "\n";
    os << "  writeSolutionToFile = " << writeSolutionToFile << "\n";
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
* @brief        Settings for numerical differentiation of a function (needed for computation of numerical jacobian e.g. in implizit integration).
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2024-04-01 (last modfied)
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
  bool addReferenceCoordinatesToEpsilon;          //!< AUTO: True: for the size estimation of the differentiation parameter, the reference coordinate \f$q^{Ref}_i\f$ is added to \hac{ODE2} coordinates --> see; False: only the current coordinate is used for size estimation of the differentiation parameter
  bool doSystemWideDifferentiation;               //!< AUTO: True: system wide differentiation (e.g. all \hac{ODE2} equations w.r.t. all \hac{ODE2} coordinates); False: only local (object) differentiation
  bool forAE;                                     //!< AUTO: flag (true/false); false = perform direct computation of jacobian for algebraic equations (AE), true = use numerical differentiation; as there must always exist an analytical implemented jacobian for AE, 'true' should only be used for verification
  bool forODE2;                                   //!< AUTO: flag (true/false); false = perform direct computation (e.g., using autodiff) of jacobian for ODE2 equations, true = use numerical differentiation; numerical differentiation is less efficient and may lead to numerical problems, but may smoothen problems of analytical derivatives; sometimes the analytical derivative may neglect terms
  bool forODE2connectors;                         //!< AUTO: flag (true/false); false: if also forODE2==false, perform direct computation of jacobian for ODE2 terms for connectors; else: use numerical differentiation; NOTE: THIS FLAG IS FOR DEVELOPMENT AND WILL BE ERASED IN FUTURE
  bool jacobianConnectorDerivative;               //!< AUTO: True: for analytic Jacobians of connectors, the Jacobian derivative is computed, causing additional CPU costs and not beeing available for all connectors or markers (thus switching to numerical differentiation); False: Jacobian derivative is neglected in analytic Jacobians (but included in numerical Jacobians), which often has only minor influence on convergence
  Real minimumCoordinateSize;                     //!< AUTO: minimum size of coordinates in relative differentiation parameter
  Real relativeEpsilon;                           //!< AUTO: relative differentiation parameter epsilon; the numerical differentiation parameter \f$\varepsilon\f$ follows from the formula (\f$\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)\f$, with \f$\varepsilon_\mathrm{relative}\f$=relativeEpsilon, \f$q_{min} = \f$minimumCoordinateSize, \f$q_i\f$ is the current coordinate which is differentiated, and \f$qRef_i\f$ is the reference coordinate of the current coordinate


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  NumericalDifferentiationSettings()
  {
    addReferenceCoordinatesToEpsilon = false;
    doSystemWideDifferentiation = false;
    forAE = false;
    forODE2 = false;
    forODE2connectors = false;
    jacobianConnectorDerivative = true;
    minimumCoordinateSize = 1e-2;
    relativeEpsilon = 1e-7;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: minimum size of coordinates in relative differentiation parameter
  void PySetMinimumCoordinateSize(const Real& minimumCoordinateSizeInit) { minimumCoordinateSize = EXUstd::GetSafelyUReal(minimumCoordinateSizeInit,"minimumCoordinateSize"); }
  //! AUTO: Read (Copy) access to: minimum size of coordinates in relative differentiation parameter
  Real PyGetMinimumCoordinateSize() const { return Real(minimumCoordinateSize); }

  //! AUTO: Set function (needed in pybind) for: relative differentiation parameter epsilon; the numerical differentiation parameter \f$\varepsilon\f$ follows from the formula (\f$\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)\f$, with \f$\varepsilon_\mathrm{relative}\f$=relativeEpsilon, \f$q_{min} = \f$minimumCoordinateSize, \f$q_i\f$ is the current coordinate which is differentiated, and \f$qRef_i\f$ is the reference coordinate of the current coordinate
  void PySetRelativeEpsilon(const Real& relativeEpsilonInit) { relativeEpsilon = EXUstd::GetSafelyUReal(relativeEpsilonInit,"relativeEpsilon"); }
  //! AUTO: Read (Copy) access to: relative differentiation parameter epsilon; the numerical differentiation parameter \f$\varepsilon\f$ follows from the formula (\f$\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)\f$, with \f$\varepsilon_\mathrm{relative}\f$=relativeEpsilon, \f$q_{min} = \f$minimumCoordinateSize, \f$q_i\f$ is the current coordinate which is differentiated, and \f$qRef_i\f$ is the reference coordinate of the current coordinate
  Real PyGetRelativeEpsilon() const { return Real(relativeEpsilon); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "NumericalDifferentiationSettings" << ":\n";
    os << "  addReferenceCoordinatesToEpsilon = " << addReferenceCoordinatesToEpsilon << "\n";
    os << "  doSystemWideDifferentiation = " << doSystemWideDifferentiation << "\n";
    os << "  forAE = " << forAE << "\n";
    os << "  forODE2 = " << forODE2 << "\n";
    os << "  forODE2connectors = " << forODE2connectors << "\n";
    os << "  jacobianConnectorDerivative = " << jacobianConnectorDerivative << "\n";
    os << "  minimumCoordinateSize = " << minimumCoordinateSize << "\n";
    os << "  relativeEpsilon = " << relativeEpsilon << "\n";
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
* @date         AUTO: 2024-04-01 (last modfied)
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
  bool ignoreMaxIterations;                       //!< AUTO: continue solver if maximum number of discontinuous (post Newton) iterations is reached (ignore tolerance)
  Real iterationTolerance;                        //!< AUTO: absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
  Index maxIterations;                            //!< AUTO: maximum number of discontinuous (post Newton) iterations


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  DiscontinuousSettings()
  {
    ignoreMaxIterations = true;
    iterationTolerance = 1;
    maxIterations = 5;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
  void PySetIterationTolerance(const Real& iterationToleranceInit) { iterationTolerance = EXUstd::GetSafelyUReal(iterationToleranceInit,"iterationTolerance"); }
  //! AUTO: Read (Copy) access to: absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
  Real PyGetIterationTolerance() const { return Real(iterationTolerance); }

  //! AUTO: Set function (needed in pybind) for: maximum number of discontinuous (post Newton) iterations
  void PySetMaxIterations(const Index& maxIterationsInit) { maxIterations = EXUstd::GetSafelyUInt(maxIterationsInit,"maxIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of discontinuous (post Newton) iterations
  Index PyGetMaxIterations() const { return Index(maxIterations); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "DiscontinuousSettings" << ":\n";
    os << "  ignoreMaxIterations = " << ignoreMaxIterations << "\n";
    os << "  iterationTolerance = " << iterationTolerance << "\n";
    os << "  maxIterations = " << maxIterations << "\n";
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
* @date         AUTO: 2024-04-01 (last modfied)
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
  Real absoluteTolerance;                         //!< AUTO: absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
  bool adaptInitialResidual;                      //!< AUTO: flag (true/false); false = standard; True: if initialResidual is very small (or zero), it may increase significantely in the first Newton iteration; to achieve relativeTolerance, the initialResidual will by updated by a higher residual within the first Newton iteration
  Real maximumSolutionNorm;                       //!< AUTO: this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (i.e., value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors is larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
  Index maxIterations;                            //!< AUTO: maximum number of iterations (including modified + restart Newton iterations); after that total number of iterations, the static/dynamic solver refines the step size or stops with an error
  Index maxModifiedNewtonIterations;              //!< AUTO: maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  Index maxModifiedNewtonRestartIterations;       //!< AUTO: maximum number of iterations for modified Newton after a Jacobian update; after that number of iterations, the full Newton method is started for this step
  Real modifiedNewtonContractivity;               //!< AUTO: maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  bool modifiedNewtonJacUpdatePerStep;            //!< AUTO: True: compute Jacobian at every time step (or static step), but not in every Newton iteration (except for bad convergence ==> switch to full Newton)
  Index newtonResidualMode;                       //!< AUTO: 0 ... use residual for computation of error (standard); 1 ... use \hac{ODE2} and \hac{ODE1} newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  Real relativeTolerance;                         //!< AUTO: relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
  bool useModifiedNewton;                         //!< AUTO: True: compute Jacobian only at first call to solver; the Jacobian (and its factorizations) is not computed in each Newton iteration, even not in every (time integration) step; False: Jacobian (and factorization) is computed in every Newton iteration (default, but may be costly)
  bool useNewtonSolver;                           //!< AUTO: flag (true/false); false = linear computation, true = use Newton solver for nonlinear solution
  bool weightTolerancePerCoordinate;              //!< AUTO: flag (true/false); false = compute error as L2-Norm of residual; true = compute error as (L2-Norm of residual) / (sqrt(number of coordinates)), which can help to use common tolerance independent of system size


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  NewtonSettings()
  {
    absoluteTolerance = 1e-10;
    adaptInitialResidual = true;
    maximumSolutionNorm = 1e38;
    maxIterations = 25;
    maxModifiedNewtonIterations = 8;
    maxModifiedNewtonRestartIterations = 7;
    modifiedNewtonContractivity = 0.5;
    modifiedNewtonJacUpdatePerStep = false;
    newtonResidualMode = 0;
    relativeTolerance = 1e-8;
    useModifiedNewton = false;
    useNewtonSolver = true;
    weightTolerancePerCoordinate = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
  void PySetAbsoluteTolerance(const Real& absoluteToleranceInit) { absoluteTolerance = EXUstd::GetSafelyUReal(absoluteToleranceInit,"absoluteTolerance"); }
  //! AUTO: Read (Copy) access to: absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
  Real PyGetAbsoluteTolerance() const { return Real(absoluteTolerance); }

  //! AUTO: Set function (needed in pybind) for: this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (i.e., value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors is larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
  void PySetMaximumSolutionNorm(const Real& maximumSolutionNormInit) { maximumSolutionNorm = EXUstd::GetSafelyUReal(maximumSolutionNormInit,"maximumSolutionNorm"); }
  //! AUTO: Read (Copy) access to: this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (i.e., value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors is larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
  Real PyGetMaximumSolutionNorm() const { return Real(maximumSolutionNorm); }

  //! AUTO: Set function (needed in pybind) for: maximum number of iterations (including modified + restart Newton iterations); after that total number of iterations, the static/dynamic solver refines the step size or stops with an error
  void PySetMaxIterations(const Index& maxIterationsInit) { maxIterations = EXUstd::GetSafelyUInt(maxIterationsInit,"maxIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of iterations (including modified + restart Newton iterations); after that total number of iterations, the static/dynamic solver refines the step size or stops with an error
  Index PyGetMaxIterations() const { return Index(maxIterations); }

  //! AUTO: Set function (needed in pybind) for: maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  void PySetMaxModifiedNewtonIterations(const Index& maxModifiedNewtonIterationsInit) { maxModifiedNewtonIterations = EXUstd::GetSafelyUInt(maxModifiedNewtonIterationsInit,"maxModifiedNewtonIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  Index PyGetMaxModifiedNewtonIterations() const { return Index(maxModifiedNewtonIterations); }

  //! AUTO: Set function (needed in pybind) for: maximum number of iterations for modified Newton after a Jacobian update; after that number of iterations, the full Newton method is started for this step
  void PySetMaxModifiedNewtonRestartIterations(const Index& maxModifiedNewtonRestartIterationsInit) { maxModifiedNewtonRestartIterations = EXUstd::GetSafelyUInt(maxModifiedNewtonRestartIterationsInit,"maxModifiedNewtonRestartIterations"); }
  //! AUTO: Read (Copy) access to: maximum number of iterations for modified Newton after a Jacobian update; after that number of iterations, the full Newton method is started for this step
  Index PyGetMaxModifiedNewtonRestartIterations() const { return Index(maxModifiedNewtonRestartIterations); }

  //! AUTO: Set function (needed in pybind) for: maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  void PySetModifiedNewtonContractivity(const Real& modifiedNewtonContractivityInit) { modifiedNewtonContractivity = EXUstd::GetSafelyPReal(modifiedNewtonContractivityInit,"modifiedNewtonContractivity"); }
  //! AUTO: Read (Copy) access to: maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  Real PyGetModifiedNewtonContractivity() const { return Real(modifiedNewtonContractivity); }

  //! AUTO: Set function (needed in pybind) for: 0 ... use residual for computation of error (standard); 1 ... use \hac{ODE2} and \hac{ODE1} newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  void PySetNewtonResidualMode(const Index& newtonResidualModeInit) { newtonResidualMode = EXUstd::GetSafelyUInt(newtonResidualModeInit,"newtonResidualMode"); }
  //! AUTO: Read (Copy) access to: 0 ... use residual for computation of error (standard); 1 ... use \hac{ODE2} and \hac{ODE1} newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  Index PyGetNewtonResidualMode() const { return Index(newtonResidualMode); }

  //! AUTO: Set function (needed in pybind) for: relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
  void PySetRelativeTolerance(const Real& relativeToleranceInit) { relativeTolerance = EXUstd::GetSafelyUReal(relativeToleranceInit,"relativeTolerance"); }
  //! AUTO: Read (Copy) access to: relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
  Real PyGetRelativeTolerance() const { return Real(relativeTolerance); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "NewtonSettings" << ":\n";
    os << "  numericalDifferentiation = " << numericalDifferentiation << "\n";
    os << "  absoluteTolerance = " << absoluteTolerance << "\n";
    os << "  adaptInitialResidual = " << adaptInitialResidual << "\n";
    os << "  maximumSolutionNorm = " << maximumSolutionNorm << "\n";
    os << "  maxIterations = " << maxIterations << "\n";
    os << "  maxModifiedNewtonIterations = " << maxModifiedNewtonIterations << "\n";
    os << "  maxModifiedNewtonRestartIterations = " << maxModifiedNewtonRestartIterations << "\n";
    os << "  modifiedNewtonContractivity = " << modifiedNewtonContractivity << "\n";
    os << "  modifiedNewtonJacUpdatePerStep = " << modifiedNewtonJacUpdatePerStep << "\n";
    os << "  newtonResidualMode = " << newtonResidualMode << "\n";
    os << "  relativeTolerance = " << relativeTolerance << "\n";
    os << "  useModifiedNewton = " << useModifiedNewton << "\n";
    os << "  useNewtonSolver = " << useNewtonSolver << "\n";
    os << "  weightTolerancePerCoordinate = " << weightTolerancePerCoordinate << "\n";
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
* @date         AUTO: 2024-04-01 (last modfied)
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
  bool computeInitialAccelerations;               //!< AUTO: True: compute initial accelerations from system EOM in acceleration form; NOTE that initial accelerations that are following from user functions in constraints are not considered for now! False: use zero accelerations
  bool lieGroupAddTangentOperator;                //!< AUTO: True: for Lie group nodes, the integrator adds the tangent operator for stiffness and constraint matrices, for improved Newton convergence; not available for sparse matrix mode (EigenSparse)
  Real newmarkBeta;                               //!< AUTO: value beta for Newmark method; default value beta = \f$\frac 1 4\f$ corresponds to (undamped) trapezoidal rule
  Real newmarkGamma;                              //!< AUTO: value gamma for Newmark method; default value gamma = \f$\frac 1 2\f$ corresponds to (undamped) trapezoidal rule
  bool resetAccelerations;                        //!< AUTO: this flag only affects if computeInitialAccelerations=False: if resetAccelerations=True, accelerations are set zero in the solver function InitializeSolverInitialConditions; this may be unwanted in case of repeatedly called SolveSteps() and in cases where solutions shall be prolonged from previous computations
  Real spectralRadius;                            //!< AUTO: spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
  bool useIndex2Constraints;                      //!< AUTO: set useIndex2Constraints = true in order to use index2 (velocity level constraints) formulation
  bool useNewmark;                                //!< AUTO: if true, use Newmark method with beta and gamma instead of generalized-Alpha


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  GeneralizedAlphaSettings()
  {
    computeInitialAccelerations = true;
    lieGroupAddTangentOperator = true;
    newmarkBeta = 0.25;
    newmarkGamma = 0.5;
    resetAccelerations = false;
    spectralRadius = 0.9;
    useIndex2Constraints = false;
    useNewmark = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: value beta for Newmark method; default value beta = \f$\frac 1 4\f$ corresponds to (undamped) trapezoidal rule
  void PySetNewmarkBeta(const Real& newmarkBetaInit) { newmarkBeta = EXUstd::GetSafelyUReal(newmarkBetaInit,"newmarkBeta"); }
  //! AUTO: Read (Copy) access to: value beta for Newmark method; default value beta = \f$\frac 1 4\f$ corresponds to (undamped) trapezoidal rule
  Real PyGetNewmarkBeta() const { return Real(newmarkBeta); }

  //! AUTO: Set function (needed in pybind) for: value gamma for Newmark method; default value gamma = \f$\frac 1 2\f$ corresponds to (undamped) trapezoidal rule
  void PySetNewmarkGamma(const Real& newmarkGammaInit) { newmarkGamma = EXUstd::GetSafelyUReal(newmarkGammaInit,"newmarkGamma"); }
  //! AUTO: Read (Copy) access to: value gamma for Newmark method; default value gamma = \f$\frac 1 2\f$ corresponds to (undamped) trapezoidal rule
  Real PyGetNewmarkGamma() const { return Real(newmarkGamma); }

  //! AUTO: Set function (needed in pybind) for: spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
  void PySetSpectralRadius(const Real& spectralRadiusInit) { spectralRadius = EXUstd::GetSafelyUReal(spectralRadiusInit,"spectralRadius"); }
  //! AUTO: Read (Copy) access to: spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
  Real PyGetSpectralRadius() const { return Real(spectralRadius); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "GeneralizedAlphaSettings" << ":\n";
    os << "  computeInitialAccelerations = " << computeInitialAccelerations << "\n";
    os << "  lieGroupAddTangentOperator = " << lieGroupAddTangentOperator << "\n";
    os << "  newmarkBeta = " << newmarkBeta << "\n";
    os << "  newmarkGamma = " << newmarkGamma << "\n";
    os << "  resetAccelerations = " << resetAccelerations << "\n";
    os << "  spectralRadius = " << spectralRadius << "\n";
    os << "  useIndex2Constraints = " << useIndex2Constraints << "\n";
    os << "  useNewmark = " << useNewmark << "\n";
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
* @brief        Settings for explicit solvers, like Explicit Euler, RK44, ODE23, DOPRI5 and others. The settings may significantely influence performance.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2024-04-01 (last modfied)
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
  bool computeEndOfStepAccelerations;             //!< AUTO: accelerations are computed at stages of the explicit integration scheme; if the user needs accelerations at the end of a step, this flag needs to be activated; if True, this causes a second call to the RHS of the equations, which may DOUBLE COMPUTATIONAL COSTS for one-step-methods; if False, the accelerations are re-used from the last stage, being slightly different
  bool computeMassMatrixInversePerBody;           //!< AUTO: If true, the solver assumes the bodies to be independent and computes the inverse of the mass matrix for all bodies independently; this may lead to WRONG RESULTS, if bodies share nodes, e.g., two MassPoint objects put on the same node or a beam with a mass point attached at a shared node; however, it may speed up explicit time integration for large systems significantly (multi-threaded)
  DynamicSolverType dynamicSolverType;            //!< AUTO: selection of explicit solver type (DOPRI5, ExplicitEuler, ExplicitMidpoint, RK44, RK67, ...), for detailed description see DynamicSolverType, \refSection{sec:DynamicSolverType}, but only referring to explicit solvers.
  bool eliminateConstraints;                      //!< AUTO: True: make explicit solver work for simple CoordinateConstraints, which are eliminated for ground constraints (e.g. fixed nodes in finite element models). False: incompatible constraints are ignored (BE CAREFUL)!
  bool useLieGroupIntegration;                    //!< AUTO: True: use Lie group integration for rigid body nodes; must be turned on for Lie group nodes (without data coordinates) to work properly; does not work for nodes with data coordinates!


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  ExplicitIntegrationSettings()
  {
    computeEndOfStepAccelerations = true;
    computeMassMatrixInversePerBody = false;
    dynamicSolverType = DynamicSolverType::DOPRI5;
    eliminateConstraints = true;
    useLieGroupIntegration = true;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "ExplicitIntegrationSettings" << ":\n";
    os << "  computeEndOfStepAccelerations = " << computeEndOfStepAccelerations << "\n";
    os << "  computeMassMatrixInversePerBody = " << computeMassMatrixInversePerBody << "\n";
    os << "  dynamicSolverType = " << dynamicSolverType << "\n";
    os << "  eliminateConstraints = " << eliminateConstraints << "\n";
    os << "  useLieGroupIntegration = " << useLieGroupIntegration << "\n";
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
* @date         AUTO: 2024-04-01 (last modfied)
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
  DiscontinuousSettings discontinuous;            //!< AUTO: parameters for treatment of discontinuities
  ExplicitIntegrationSettings explicitIntegration;//!< AUTO: special parameters for explicit time integration
  GeneralizedAlphaSettings generalizedAlpha;      //!< AUTO: parameters for generalized-alpha, implicit trapezoidal rule or Newmark (options only apply for these methods)
  NewtonSettings newton;                          //!< AUTO: parameters for Newton method; used for implicit time integration methods only
  Real absoluteTolerance;                         //!< AUTO: \f$a_{tol}\f$: if automaticStepSize=True, absolute tolerance for the error control; must fulfill \f$a_{tol} > 0\f$; see \refSection{sec:ExplicitSolver}
  bool adaptiveStep;                              //!< AUTO: True: the step size may be reduced if step fails; no automatic stepsize control
  Real adaptiveStepDecrease;                      //!< AUTO: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real adaptiveStepIncrease;                      //!< AUTO: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Index adaptiveStepRecoveryIterations;           //!< AUTO: Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  Index adaptiveStepRecoverySteps;                //!< AUTO: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  bool automaticStepSize;                         //!< AUTO: True: for specific integrators with error control (e.g., DOPRI5), compute automatic step size based on error estimation; False: constant step size (step may be reduced if adaptiveStep=True); the maximum stepSize reads \f$h = h_{max} = \frac{t_{end} - t_{start}}{n_{steps}}\f$
  Index computeLoadsJacobian;                     //!< AUTO: 0:  jacobian of loads not considered (may lead to slow convergence or Newton failure); 1: in case of implicit integrators, compute (numerical) Jacobian of ODE2 and ODE1 coordinates for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; 2: also compute ODE2_t dependencies for jacobian; note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
  Real endTime;                                   //!< AUTO: \f$t_{end}\f$: end time of time integration
  Real initialStepSize;                           //!< AUTO: \f$h_{init}\f$: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
  Real minimumStepSize;                           //!< AUTO: \f$h_{min}\f$: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
  Index numberOfSteps;                            //!< AUTO: \f$n_{steps}\f$: number of steps in time integration; (maximum) stepSize \f$h\f$ is computed from \f$h = \frac{t_{end} - t_{start}}{n_{steps}}\f$; for automatic stepsize control, this stepSize is the maximum steps size, \f$h_{max} = h\f$
  Real realtimeFactor;                            //!< AUTO: if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
  Index realtimeWaitMicroseconds;                 //!< AUTO: if simulateInRealtime=True, a loop runs which waits realtimeWaitMicroseconds until checking again if the realtime is reached; using larger values leads to less CPU usage but less accurate realtime accuracy; smaller values (< 1000) increase CPU usage but improve realtime accuracy
  Real relativeTolerance;                         //!< AUTO: \f$r_{tol}\f$: if automaticStepSize=True, relative tolerance for the error control; must fulfill \f$r_{tol} \ge 0\f$; see \refSection{sec:ExplicitSolver}
  bool reuseConstantMassMatrix;                   //!< AUTO: True: does not recompute constant mass matrices (e.g. of some finite elements, mass points, etc.); if False, it always recomputes the mass matrix (e.g. needed, if user changes mass parameters via Python)
  bool simulateInRealtime;                        //!< AUTO: True: simulate in realtime; the solver waits for computation of the next step until the CPU time reached the simulation time; if the simulation is slower than realtime, it simply continues
  Real startTime;                                 //!< AUTO: \f$t_{start}\f$: start time of time integration (usually set to zero)
  Index stepInformation;                          //!< AUTO: add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  Real stepSizeMaxIncrease;                       //!< AUTO: \f$f_{maxInc}\f$: if automaticStepSize=True, maximum increase of step size per step, see \refSection{sec:ExplicitSolver}; make this factor smaller (but \f$> 1\f$) if too many rejected steps
  Real stepSizeSafety;                            //!< AUTO: \f$r_{sfty}\f$: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see \refSection{sec:ExplicitSolver}. Make this factor smaller if many steps are rejected.
  Index verboseMode;                              //!< AUTO: 0 ... no output, 1 ... show short step information every 2 seconds (every 30 seconds after 1 hour CPU time), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  Index verboseModeFile;                          //!< AUTO: same behaviour as verboseMode, but outputs all solver information to file


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  TimeIntegrationSettings()
  {
    absoluteTolerance = 1e-8;
    adaptiveStep = true;
    adaptiveStepDecrease = 0.5;
    adaptiveStepIncrease = 2;
    adaptiveStepRecoveryIterations = 7;
    adaptiveStepRecoverySteps = 10;
    automaticStepSize = true;
    computeLoadsJacobian = 0;
    endTime = 1;
    initialStepSize = 0;
    minimumStepSize = 1e-8;
    numberOfSteps = 100;
    realtimeFactor = 1;
    realtimeWaitMicroseconds = 1000;
    relativeTolerance = 1e-8;
    reuseConstantMassMatrix = true;
    simulateInRealtime = false;
    startTime = 0;
    stepInformation = 67;
    stepSizeMaxIncrease = 2;
    stepSizeSafety = 0.90;
    verboseMode = 0;
    verboseModeFile = 0;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: \f$a_{tol}\f$: if automaticStepSize=True, absolute tolerance for the error control; must fulfill \f$a_{tol} > 0\f$; see \refSection{sec:ExplicitSolver}
  void PySetAbsoluteTolerance(const Real& absoluteToleranceInit) { absoluteTolerance = EXUstd::GetSafelyUReal(absoluteToleranceInit,"absoluteTolerance"); }
  //! AUTO: Read (Copy) access to: \f$a_{tol}\f$: if automaticStepSize=True, absolute tolerance for the error control; must fulfill \f$a_{tol} > 0\f$; see \refSection{sec:ExplicitSolver}
  Real PyGetAbsoluteTolerance() const { return Real(absoluteTolerance); }

  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepDecrease(const Real& adaptiveStepDecreaseInit) { adaptiveStepDecrease = EXUstd::GetSafelyUReal(adaptiveStepDecreaseInit,"adaptiveStepDecrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepDecrease() const { return Real(adaptiveStepDecrease); }

  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepIncrease(const Real& adaptiveStepIncreaseInit) { adaptiveStepIncrease = EXUstd::GetSafelyUReal(adaptiveStepIncreaseInit,"adaptiveStepIncrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepIncrease() const { return Real(adaptiveStepIncrease); }

  //! AUTO: Set function (needed in pybind) for: Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  void PySetAdaptiveStepRecoveryIterations(const Index& adaptiveStepRecoveryIterationsInit) { adaptiveStepRecoveryIterations = EXUstd::GetSafelyUInt(adaptiveStepRecoveryIterationsInit,"adaptiveStepRecoveryIterations"); }
  //! AUTO: Read (Copy) access to: Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  Index PyGetAdaptiveStepRecoveryIterations() const { return Index(adaptiveStepRecoveryIterations); }

  //! AUTO: Set function (needed in pybind) for: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepRecoverySteps(const Index& adaptiveStepRecoveryStepsInit) { adaptiveStepRecoverySteps = EXUstd::GetSafelyUInt(adaptiveStepRecoveryStepsInit,"adaptiveStepRecoverySteps"); }
  //! AUTO: Read (Copy) access to: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  Index PyGetAdaptiveStepRecoverySteps() const { return Index(adaptiveStepRecoverySteps); }

  //! AUTO: Set function (needed in pybind) for: 0:  jacobian of loads not considered (may lead to slow convergence or Newton failure); 1: in case of implicit integrators, compute (numerical) Jacobian of ODE2 and ODE1 coordinates for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; 2: also compute ODE2_t dependencies for jacobian; note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
  void PySetComputeLoadsJacobian(const Index& computeLoadsJacobianInit) { computeLoadsJacobian = EXUstd::GetSafelyUInt(computeLoadsJacobianInit,"computeLoadsJacobian"); }
  //! AUTO: Read (Copy) access to: 0:  jacobian of loads not considered (may lead to slow convergence or Newton failure); 1: in case of implicit integrators, compute (numerical) Jacobian of ODE2 and ODE1 coordinates for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; 2: also compute ODE2_t dependencies for jacobian; note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
  Index PyGetComputeLoadsJacobian() const { return Index(computeLoadsJacobian); }

  //! AUTO: Set function (needed in pybind) for: \f$t_{end}\f$: end time of time integration
  void PySetEndTime(const Real& endTimeInit) { endTime = EXUstd::GetSafelyUReal(endTimeInit,"endTime"); }
  //! AUTO: Read (Copy) access to: \f$t_{end}\f$: end time of time integration
  Real PyGetEndTime() const { return Real(endTime); }

  //! AUTO: Set function (needed in pybind) for: \f$h_{init}\f$: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
  void PySetInitialStepSize(const Real& initialStepSizeInit) { initialStepSize = EXUstd::GetSafelyUReal(initialStepSizeInit,"initialStepSize"); }
  //! AUTO: Read (Copy) access to: \f$h_{init}\f$: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
  Real PyGetInitialStepSize() const { return Real(initialStepSize); }

  //! AUTO: Set function (needed in pybind) for: \f$h_{min}\f$: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
  void PySetMinimumStepSize(const Real& minimumStepSizeInit) { minimumStepSize = EXUstd::GetSafelyPReal(minimumStepSizeInit,"minimumStepSize"); }
  //! AUTO: Read (Copy) access to: \f$h_{min}\f$: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
  Real PyGetMinimumStepSize() const { return Real(minimumStepSize); }

  //! AUTO: Set function (needed in pybind) for: \f$n_{steps}\f$: number of steps in time integration; (maximum) stepSize \f$h\f$ is computed from \f$h = \frac{t_{end} - t_{start}}{n_{steps}}\f$; for automatic stepsize control, this stepSize is the maximum steps size, \f$h_{max} = h\f$
  void PySetNumberOfSteps(const Index& numberOfStepsInit) { numberOfSteps = EXUstd::GetSafelyPInt(numberOfStepsInit,"numberOfSteps"); }
  //! AUTO: Read (Copy) access to: \f$n_{steps}\f$: number of steps in time integration; (maximum) stepSize \f$h\f$ is computed from \f$h = \frac{t_{end} - t_{start}}{n_{steps}}\f$; for automatic stepsize control, this stepSize is the maximum steps size, \f$h_{max} = h\f$
  Index PyGetNumberOfSteps() const { return Index(numberOfSteps); }

  //! AUTO: Set function (needed in pybind) for: if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
  void PySetRealtimeFactor(const Real& realtimeFactorInit) { realtimeFactor = EXUstd::GetSafelyPReal(realtimeFactorInit,"realtimeFactor"); }
  //! AUTO: Read (Copy) access to: if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
  Real PyGetRealtimeFactor() const { return Real(realtimeFactor); }

  //! AUTO: Set function (needed in pybind) for: if simulateInRealtime=True, a loop runs which waits realtimeWaitMicroseconds until checking again if the realtime is reached; using larger values leads to less CPU usage but less accurate realtime accuracy; smaller values (< 1000) increase CPU usage but improve realtime accuracy
  void PySetRealtimeWaitMicroseconds(const Index& realtimeWaitMicrosecondsInit) { realtimeWaitMicroseconds = EXUstd::GetSafelyPInt(realtimeWaitMicrosecondsInit,"realtimeWaitMicroseconds"); }
  //! AUTO: Read (Copy) access to: if simulateInRealtime=True, a loop runs which waits realtimeWaitMicroseconds until checking again if the realtime is reached; using larger values leads to less CPU usage but less accurate realtime accuracy; smaller values (< 1000) increase CPU usage but improve realtime accuracy
  Index PyGetRealtimeWaitMicroseconds() const { return Index(realtimeWaitMicroseconds); }

  //! AUTO: Set function (needed in pybind) for: \f$r_{tol}\f$: if automaticStepSize=True, relative tolerance for the error control; must fulfill \f$r_{tol} \ge 0\f$; see \refSection{sec:ExplicitSolver}
  void PySetRelativeTolerance(const Real& relativeToleranceInit) { relativeTolerance = EXUstd::GetSafelyUReal(relativeToleranceInit,"relativeTolerance"); }
  //! AUTO: Read (Copy) access to: \f$r_{tol}\f$: if automaticStepSize=True, relative tolerance for the error control; must fulfill \f$r_{tol} \ge 0\f$; see \refSection{sec:ExplicitSolver}
  Real PyGetRelativeTolerance() const { return Real(relativeTolerance); }

  //! AUTO: Set function (needed in pybind) for: \f$t_{start}\f$: start time of time integration (usually set to zero)
  void PySetStartTime(const Real& startTimeInit) { startTime = EXUstd::GetSafelyUReal(startTimeInit,"startTime"); }
  //! AUTO: Read (Copy) access to: \f$t_{start}\f$: start time of time integration (usually set to zero)
  Real PyGetStartTime() const { return Real(startTime); }

  //! AUTO: Set function (needed in pybind) for: add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  void PySetStepInformation(const Index& stepInformationInit) { stepInformation = EXUstd::GetSafelyUInt(stepInformationInit,"stepInformation"); }
  //! AUTO: Read (Copy) access to: add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  Index PyGetStepInformation() const { return Index(stepInformation); }

  //! AUTO: Set function (needed in pybind) for: \f$f_{maxInc}\f$: if automaticStepSize=True, maximum increase of step size per step, see \refSection{sec:ExplicitSolver}; make this factor smaller (but \f$> 1\f$) if too many rejected steps
  void PySetStepSizeMaxIncrease(const Real& stepSizeMaxIncreaseInit) { stepSizeMaxIncrease = EXUstd::GetSafelyUReal(stepSizeMaxIncreaseInit,"stepSizeMaxIncrease"); }
  //! AUTO: Read (Copy) access to: \f$f_{maxInc}\f$: if automaticStepSize=True, maximum increase of step size per step, see \refSection{sec:ExplicitSolver}; make this factor smaller (but \f$> 1\f$) if too many rejected steps
  Real PyGetStepSizeMaxIncrease() const { return Real(stepSizeMaxIncrease); }

  //! AUTO: Set function (needed in pybind) for: \f$r_{sfty}\f$: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see \refSection{sec:ExplicitSolver}. Make this factor smaller if many steps are rejected.
  void PySetStepSizeSafety(const Real& stepSizeSafetyInit) { stepSizeSafety = EXUstd::GetSafelyUReal(stepSizeSafetyInit,"stepSizeSafety"); }
  //! AUTO: Read (Copy) access to: \f$r_{sfty}\f$: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see \refSection{sec:ExplicitSolver}. Make this factor smaller if many steps are rejected.
  Real PyGetStepSizeSafety() const { return Real(stepSizeSafety); }

  //! AUTO: Set function (needed in pybind) for: 0 ... no output, 1 ... show short step information every 2 seconds (every 30 seconds after 1 hour CPU time), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  void PySetVerboseMode(const Index& verboseModeInit) { verboseMode = EXUstd::GetSafelyUInt(verboseModeInit,"verboseMode"); }
  //! AUTO: Read (Copy) access to: 0 ... no output, 1 ... show short step information every 2 seconds (every 30 seconds after 1 hour CPU time), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  Index PyGetVerboseMode() const { return Index(verboseMode); }

  //! AUTO: Set function (needed in pybind) for: same behaviour as verboseMode, but outputs all solver information to file
  void PySetVerboseModeFile(const Index& verboseModeFileInit) { verboseModeFile = EXUstd::GetSafelyUInt(verboseModeFileInit,"verboseModeFile"); }
  //! AUTO: Read (Copy) access to: same behaviour as verboseMode, but outputs all solver information to file
  Index PyGetVerboseModeFile() const { return Index(verboseModeFile); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "TimeIntegrationSettings" << ":\n";
    os << "  discontinuous = " << discontinuous << "\n";
    os << "  explicitIntegration = " << explicitIntegration << "\n";
    os << "  generalizedAlpha = " << generalizedAlpha << "\n";
    os << "  newton = " << newton << "\n";
    os << "  absoluteTolerance = " << absoluteTolerance << "\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  adaptiveStepDecrease = " << adaptiveStepDecrease << "\n";
    os << "  adaptiveStepIncrease = " << adaptiveStepIncrease << "\n";
    os << "  adaptiveStepRecoveryIterations = " << adaptiveStepRecoveryIterations << "\n";
    os << "  adaptiveStepRecoverySteps = " << adaptiveStepRecoverySteps << "\n";
    os << "  automaticStepSize = " << automaticStepSize << "\n";
    os << "  computeLoadsJacobian = " << computeLoadsJacobian << "\n";
    os << "  endTime = " << endTime << "\n";
    os << "  initialStepSize = " << initialStepSize << "\n";
    os << "  minimumStepSize = " << minimumStepSize << "\n";
    os << "  numberOfSteps = " << numberOfSteps << "\n";
    os << "  realtimeFactor = " << realtimeFactor << "\n";
    os << "  realtimeWaitMicroseconds = " << realtimeWaitMicroseconds << "\n";
    os << "  relativeTolerance = " << relativeTolerance << "\n";
    os << "  reuseConstantMassMatrix = " << reuseConstantMassMatrix << "\n";
    os << "  simulateInRealtime = " << simulateInRealtime << "\n";
    os << "  startTime = " << startTime << "\n";
    os << "  stepInformation = " << stepInformation << "\n";
    os << "  stepSizeMaxIncrease = " << stepSizeMaxIncrease << "\n";
    os << "  stepSizeSafety = " << stepSizeSafety << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
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
* @date         AUTO: 2024-04-01 (last modfied)
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
  DiscontinuousSettings discontinuous;            //!< AUTO: parameters for treatment of discontinuities
  NewtonSettings newton;                          //!< AUTO: parameters for Newton method (e.g. in static solver or time integration)
  bool adaptiveStep;                              //!< AUTO: True: use step reduction if step fails; False: fixed step size
  Real adaptiveStepDecrease;                      //!< AUTO: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real adaptiveStepIncrease;                      //!< AUTO: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Index adaptiveStepRecoveryIterations;           //!< AUTO: Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  Index adaptiveStepRecoverySteps;                //!< AUTO: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  bool computeLoadsJacobian;                      //!< AUTO: True: compute (currently numerical) Jacobian for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; False: jacobian of loads not considered (may lead to slow convergence or Newton failure); note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
  bool constrainODE1coordinates;                  //!< AUTO: True: ODE1coordinates are constrained to initial values; False: undefined behavior, currently not supported
  Real loadStepDuration;                          //!< AUTO: quasi-time for all load steps (added to current time in load steps)
  bool loadStepGeometric;                         //!< AUTO: if loadStepGeometric=false, the load steps are incremental (arithmetic series, e.g. 0.1,0.2,0.3,...); if true, the load steps are increased in a geometric series, e.g. for \f$n=8\f$ numberOfLoadSteps and \f$d = 1000\f$ loadStepGeometricRange, it follows: \f$1000^{1/8}/1000=0.00237\f$, \f$1000^{2/8}/1000=0.00562\f$, \f$1000^{3/8}/1000=0.0133\f$, ..., \f$1000^{7/8}/1000=0.422\f$, \f$1000^{8/8}/1000=1\f$
  Real loadStepGeometricRange;                    //!< AUTO: if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  Real loadStepStart;                             //!< AUTO: a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  Real minimumStepSize;                           //!< AUTO: lower limit of step size, before nonlinear solver stops
  Index numberOfLoadSteps;                        //!< AUTO: number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  Real stabilizerODE2term;                        //!< AUTO: add mass-proportional stabilizer term in \hac{ODE2} part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  Index stepInformation;                          //!< AUTO: add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  bool useLoadFactor;                             //!< AUTO: True: compute a load factor \f$\in [0,1]\f$ from static step time; all loads are scaled by the load factor; False: loads are always scaled with 1 -- use this option if time dependent loads use a userFunction
  Index verboseMode;                              //!< AUTO: 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  Index verboseModeFile;                          //!< AUTO: same behaviour as verboseMode, but outputs all solver information to file


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  StaticSolverSettings()
  {
    adaptiveStep = true;
    adaptiveStepDecrease = 0.25;
    adaptiveStepIncrease = 2;
    adaptiveStepRecoveryIterations = 7;
    adaptiveStepRecoverySteps = 4;
    computeLoadsJacobian = true;
    constrainODE1coordinates = true;
    loadStepDuration = 1;
    loadStepGeometric = false;
    loadStepGeometricRange = 1000;
    loadStepStart = 0;
    minimumStepSize = 1e-8;
    numberOfLoadSteps = 1;
    stabilizerODE2term = 0;
    stepInformation = 67;
    useLoadFactor = true;
    verboseMode = 1;
    verboseModeFile = 0;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepDecrease(const Real& adaptiveStepDecreaseInit) { adaptiveStepDecrease = EXUstd::GetSafelyUReal(adaptiveStepDecreaseInit,"adaptiveStepDecrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepDecrease() const { return Real(adaptiveStepDecrease); }

  //! AUTO: Set function (needed in pybind) for: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepIncrease(const Real& adaptiveStepIncreaseInit) { adaptiveStepIncrease = EXUstd::GetSafelyUReal(adaptiveStepIncreaseInit,"adaptiveStepIncrease"); }
  //! AUTO: Read (Copy) access to: Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Real PyGetAdaptiveStepIncrease() const { return Real(adaptiveStepIncrease); }

  //! AUTO: Set function (needed in pybind) for: Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  void PySetAdaptiveStepRecoveryIterations(const Index& adaptiveStepRecoveryIterationsInit) { adaptiveStepRecoveryIterations = EXUstd::GetSafelyUInt(adaptiveStepRecoveryIterationsInit,"adaptiveStepRecoveryIterations"); }
  //! AUTO: Read (Copy) access to: Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  Index PyGetAdaptiveStepRecoveryIterations() const { return Index(adaptiveStepRecoveryIterations); }

  //! AUTO: Set function (needed in pybind) for: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  void PySetAdaptiveStepRecoverySteps(const Index& adaptiveStepRecoveryStepsInit) { adaptiveStepRecoverySteps = EXUstd::GetSafelyUInt(adaptiveStepRecoveryStepsInit,"adaptiveStepRecoverySteps"); }
  //! AUTO: Read (Copy) access to: Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  Index PyGetAdaptiveStepRecoverySteps() const { return Index(adaptiveStepRecoverySteps); }

  //! AUTO: Set function (needed in pybind) for: quasi-time for all load steps (added to current time in load steps)
  void PySetLoadStepDuration(const Real& loadStepDurationInit) { loadStepDuration = EXUstd::GetSafelyPReal(loadStepDurationInit,"loadStepDuration"); }
  //! AUTO: Read (Copy) access to: quasi-time for all load steps (added to current time in load steps)
  Real PyGetLoadStepDuration() const { return Real(loadStepDuration); }

  //! AUTO: Set function (needed in pybind) for: if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  void PySetLoadStepGeometricRange(const Real& loadStepGeometricRangeInit) { loadStepGeometricRange = EXUstd::GetSafelyPReal(loadStepGeometricRangeInit,"loadStepGeometricRange"); }
  //! AUTO: Read (Copy) access to: if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  Real PyGetLoadStepGeometricRange() const { return Real(loadStepGeometricRange); }

  //! AUTO: Set function (needed in pybind) for: a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  void PySetLoadStepStart(const Real& loadStepStartInit) { loadStepStart = EXUstd::GetSafelyUReal(loadStepStartInit,"loadStepStart"); }
  //! AUTO: Read (Copy) access to: a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  Real PyGetLoadStepStart() const { return Real(loadStepStart); }

  //! AUTO: Set function (needed in pybind) for: lower limit of step size, before nonlinear solver stops
  void PySetMinimumStepSize(const Real& minimumStepSizeInit) { minimumStepSize = EXUstd::GetSafelyPReal(minimumStepSizeInit,"minimumStepSize"); }
  //! AUTO: Read (Copy) access to: lower limit of step size, before nonlinear solver stops
  Real PyGetMinimumStepSize() const { return Real(minimumStepSize); }

  //! AUTO: Set function (needed in pybind) for: number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  void PySetNumberOfLoadSteps(const Index& numberOfLoadStepsInit) { numberOfLoadSteps = EXUstd::GetSafelyPInt(numberOfLoadStepsInit,"numberOfLoadSteps"); }
  //! AUTO: Read (Copy) access to: number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  Index PyGetNumberOfLoadSteps() const { return Index(numberOfLoadSteps); }

  //! AUTO: Set function (needed in pybind) for: add mass-proportional stabilizer term in \hac{ODE2} part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  void PySetStabilizerODE2term(const Real& stabilizerODE2termInit) { stabilizerODE2term = EXUstd::GetSafelyUReal(stabilizerODE2termInit,"stabilizerODE2term"); }
  //! AUTO: Read (Copy) access to: add mass-proportional stabilizer term in \hac{ODE2} part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  Real PyGetStabilizerODE2term() const { return Real(stabilizerODE2term); }

  //! AUTO: Set function (needed in pybind) for: add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  void PySetStepInformation(const Index& stepInformationInit) { stepInformation = EXUstd::GetSafelyUInt(stepInformationInit,"stepInformation"); }
  //! AUTO: Read (Copy) access to: add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  Index PyGetStepInformation() const { return Index(stepInformation); }

  //! AUTO: Set function (needed in pybind) for: 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  void PySetVerboseMode(const Index& verboseModeInit) { verboseMode = EXUstd::GetSafelyUInt(verboseModeInit,"verboseMode"); }
  //! AUTO: Read (Copy) access to: 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  Index PyGetVerboseMode() const { return Index(verboseMode); }

  //! AUTO: Set function (needed in pybind) for: same behaviour as verboseMode, but outputs all solver information to file
  void PySetVerboseModeFile(const Index& verboseModeFileInit) { verboseModeFile = EXUstd::GetSafelyUInt(verboseModeFileInit,"verboseModeFile"); }
  //! AUTO: Read (Copy) access to: same behaviour as verboseMode, but outputs all solver information to file
  Index PyGetVerboseModeFile() const { return Index(verboseModeFile); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "StaticSolverSettings" << ":\n";
    os << "  discontinuous = " << discontinuous << "\n";
    os << "  newton = " << newton << "\n";
    os << "  adaptiveStep = " << adaptiveStep << "\n";
    os << "  adaptiveStepDecrease = " << adaptiveStepDecrease << "\n";
    os << "  adaptiveStepIncrease = " << adaptiveStepIncrease << "\n";
    os << "  adaptiveStepRecoveryIterations = " << adaptiveStepRecoveryIterations << "\n";
    os << "  adaptiveStepRecoverySteps = " << adaptiveStepRecoverySteps << "\n";
    os << "  computeLoadsJacobian = " << computeLoadsJacobian << "\n";
    os << "  constrainODE1coordinates = " << constrainODE1coordinates << "\n";
    os << "  loadStepDuration = " << loadStepDuration << "\n";
    os << "  loadStepGeometric = " << loadStepGeometric << "\n";
    os << "  loadStepGeometricRange = " << loadStepGeometricRange << "\n";
    os << "  loadStepStart = " << loadStepStart << "\n";
    os << "  minimumStepSize = " << minimumStepSize << "\n";
    os << "  numberOfLoadSteps = " << numberOfLoadSteps << "\n";
    os << "  stabilizerODE2term = " << stabilizerODE2term << "\n";
    os << "  stepInformation = " << stepInformation << "\n";
    os << "  useLoadFactor = " << useLoadFactor << "\n";
    os << "  verboseMode = " << verboseMode << "\n";
    os << "  verboseModeFile = " << verboseModeFile << "\n";
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
* @date         AUTO: 2024-04-01 (last modfied)
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
  bool ignoreSingularJacobian;                    //!< AUTO: [ONLY implemented for dense, Eigen matrix mode] False: standard way, fails if jacobian is singular; True: use Eigen's FullPivLU (thus only works with LinearSolverType.EigenDense) which handles over- and underdetermined systems; can often resolve redundant constraints, but MAY ALSO LEAD TO ERRONEOUS RESULTS!
  Real pivotThreshold;                            //!< AUTO: [ONLY available for EXUdense and EigenDense (FullPivot) solver] threshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
  bool reuseAnalyzedPattern;                      //!< AUTO: [ONLY available for sparse matrices] True: the Eigen SparseLU solver offers the possibility to reuse an analyzed pattern of a previous factorization; this may reduce total factorization time by a factor of 2 or 3, depending on the matrix type; however, if the matrix patterns heavily change between computations, this may even slow down performance; this flag is set for SparseMatrices in InitializeSolverData(...) and should be handled with care!
  bool showCausingItems;                          //!< AUTO: False: no output, if solver fails; True: if redundant equations appear, they are resolved such that according solution variables are set to zero; in case of redundant constraints, this may help, but it may lead to erroneous behaviour; for static problems, this may suppress static motion or resolve problems in case of instabilities, but should in general be considered with care!


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  LinearSolverSettings()
  {
    ignoreSingularJacobian = false;
    pivotThreshold = 0;
    reuseAnalyzedPattern = false;
    showCausingItems = true;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: [ONLY available for EXUdense and EigenDense (FullPivot) solver] threshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
  void PySetPivotThreshold(const Real& pivotThresholdInit) { pivotThreshold = EXUstd::GetSafelyUReal(pivotThresholdInit,"pivotThreshold"); }
  //! AUTO: Read (Copy) access to: [ONLY available for EXUdense and EigenDense (FullPivot) solver] threshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
  Real PyGetPivotThreshold() const { return Real(pivotThreshold); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "LinearSolverSettings" << ":\n";
    os << "  ignoreSingularJacobian = " << ignoreSingularJacobian << "\n";
    os << "  pivotThreshold = " << pivotThreshold << "\n";
    os << "  reuseAnalyzedPattern = " << reuseAnalyzedPattern << "\n";
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
* @class        Parallel
* @brief        Settings for linear solver, both dense and sparse (Eigen).
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2024-04-01 (last modfied)
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

class Parallel // AUTO: 
{
public: // AUTO: 
  Index multithreadedLLimitJacobians;             //!< AUTO: compute jacobians (ODE2, AE, ...) multi-threaded; this is the limit number of according objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index multithreadedLLimitLoads;                 //!< AUTO: compute loads multi-threaded; this is the limit number of loads from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index multithreadedLLimitMassMatrices;          //!< AUTO: compute bodies mass matrices multi-threaded; this is the limit number of bodies from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index multithreadedLLimitResiduals;             //!< AUTO: compute RHS vectors, AE, and reaction forces multi-threaded; this is the limit number of objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index numberOfThreads;                          //!< AUTO: number of threads used for parallel computation (1 == scalar processing); do not use more threads than available threads (in most cases it is good to restrict to the number of cores); currently, only one solver can be started with multithreading; if you use several mbs in parallel (co-simulation), you should use serial computing
  Index taskSplitMinItems;                        //!< AUTO: number of items from which on the tasks are split into subtasks (which slightly increases threading performance; this may be critical for smaller number of objects, should be roughly between 50 and 5000; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index taskSplitTasksPerThread;                  //!< AUTO: this is the number of subtasks that every thread receives; minimum is 1, the maximum should not be larger than 100; this factor is 1 as long as the taskSplitMinItems is not reached; flag is copied into MainSystem internal flag at InitializeSolverData(...)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  Parallel()
  {
    multithreadedLLimitJacobians = 20;
    multithreadedLLimitLoads = 20;
    multithreadedLLimitMassMatrices = 20;
    multithreadedLLimitResiduals = 20;
    numberOfThreads = 1;
    taskSplitMinItems = 50;
    taskSplitTasksPerThread = 16;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: compute jacobians (ODE2, AE, ...) multi-threaded; this is the limit number of according objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  void PySetMultithreadedLLimitJacobians(const Index& multithreadedLLimitJacobiansInit) { multithreadedLLimitJacobians = EXUstd::GetSafelyPInt(multithreadedLLimitJacobiansInit,"multithreadedLLimitJacobians"); }
  //! AUTO: Read (Copy) access to: compute jacobians (ODE2, AE, ...) multi-threaded; this is the limit number of according objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index PyGetMultithreadedLLimitJacobians() const { return Index(multithreadedLLimitJacobians); }

  //! AUTO: Set function (needed in pybind) for: compute loads multi-threaded; this is the limit number of loads from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  void PySetMultithreadedLLimitLoads(const Index& multithreadedLLimitLoadsInit) { multithreadedLLimitLoads = EXUstd::GetSafelyPInt(multithreadedLLimitLoadsInit,"multithreadedLLimitLoads"); }
  //! AUTO: Read (Copy) access to: compute loads multi-threaded; this is the limit number of loads from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index PyGetMultithreadedLLimitLoads() const { return Index(multithreadedLLimitLoads); }

  //! AUTO: Set function (needed in pybind) for: compute bodies mass matrices multi-threaded; this is the limit number of bodies from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  void PySetMultithreadedLLimitMassMatrices(const Index& multithreadedLLimitMassMatricesInit) { multithreadedLLimitMassMatrices = EXUstd::GetSafelyPInt(multithreadedLLimitMassMatricesInit,"multithreadedLLimitMassMatrices"); }
  //! AUTO: Read (Copy) access to: compute bodies mass matrices multi-threaded; this is the limit number of bodies from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index PyGetMultithreadedLLimitMassMatrices() const { return Index(multithreadedLLimitMassMatrices); }

  //! AUTO: Set function (needed in pybind) for: compute RHS vectors, AE, and reaction forces multi-threaded; this is the limit number of objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  void PySetMultithreadedLLimitResiduals(const Index& multithreadedLLimitResidualsInit) { multithreadedLLimitResiduals = EXUstd::GetSafelyPInt(multithreadedLLimitResidualsInit,"multithreadedLLimitResiduals"); }
  //! AUTO: Read (Copy) access to: compute RHS vectors, AE, and reaction forces multi-threaded; this is the limit number of objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index PyGetMultithreadedLLimitResiduals() const { return Index(multithreadedLLimitResiduals); }

  //! AUTO: Set function (needed in pybind) for: number of threads used for parallel computation (1 == scalar processing); do not use more threads than available threads (in most cases it is good to restrict to the number of cores); currently, only one solver can be started with multithreading; if you use several mbs in parallel (co-simulation), you should use serial computing
  void PySetNumberOfThreads(const Index& numberOfThreadsInit) { numberOfThreads = EXUstd::GetSafelyPInt(numberOfThreadsInit,"numberOfThreads"); }
  //! AUTO: Read (Copy) access to: number of threads used for parallel computation (1 == scalar processing); do not use more threads than available threads (in most cases it is good to restrict to the number of cores); currently, only one solver can be started with multithreading; if you use several mbs in parallel (co-simulation), you should use serial computing
  Index PyGetNumberOfThreads() const { return Index(numberOfThreads); }

  //! AUTO: Set function (needed in pybind) for: number of items from which on the tasks are split into subtasks (which slightly increases threading performance; this may be critical for smaller number of objects, should be roughly between 50 and 5000; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  void PySetTaskSplitMinItems(const Index& taskSplitMinItemsInit) { taskSplitMinItems = EXUstd::GetSafelyPInt(taskSplitMinItemsInit,"taskSplitMinItems"); }
  //! AUTO: Read (Copy) access to: number of items from which on the tasks are split into subtasks (which slightly increases threading performance; this may be critical for smaller number of objects, should be roughly between 50 and 5000; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index PyGetTaskSplitMinItems() const { return Index(taskSplitMinItems); }

  //! AUTO: Set function (needed in pybind) for: this is the number of subtasks that every thread receives; minimum is 1, the maximum should not be larger than 100; this factor is 1 as long as the taskSplitMinItems is not reached; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  void PySetTaskSplitTasksPerThread(const Index& taskSplitTasksPerThreadInit) { taskSplitTasksPerThread = EXUstd::GetSafelyPInt(taskSplitTasksPerThreadInit,"taskSplitTasksPerThread"); }
  //! AUTO: Read (Copy) access to: this is the number of subtasks that every thread receives; minimum is 1, the maximum should not be larger than 100; this factor is 1 as long as the taskSplitMinItems is not reached; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index PyGetTaskSplitTasksPerThread() const { return Index(taskSplitTasksPerThread); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "Parallel" << ":\n";
    os << "  multithreadedLLimitJacobians = " << multithreadedLLimitJacobians << "\n";
    os << "  multithreadedLLimitLoads = " << multithreadedLLimitLoads << "\n";
    os << "  multithreadedLLimitMassMatrices = " << multithreadedLLimitMassMatrices << "\n";
    os << "  multithreadedLLimitResiduals = " << multithreadedLLimitResiduals << "\n";
    os << "  numberOfThreads = " << numberOfThreads << "\n";
    os << "  taskSplitMinItems = " << taskSplitMinItems << "\n";
    os << "  taskSplitTasksPerThread = " << taskSplitTasksPerThread << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const Parallel& object)
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
* @date         AUTO: 2024-04-01 (last modfied)
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
  LinearSolverSettings linearSolverSettings;      //!< AUTO: linear solver parameters (used for dense and sparse solvers)
  Parallel parallel;                              //!< AUTO: parameters for vectorized and parallelized (multi-threaded) computations
  SolutionSettings solutionSettings;              //!< AUTO: settings for solution files
  StaticSolverSettings staticSolver;              //!< AUTO: static solver parameters
  TimeIntegrationSettings timeIntegration;        //!< AUTO: time integration parameters
  bool cleanUpMemory;                             //!< AUTO: True: solvers will free memory at exit (recommended for large systems); False: keep allocated memory for repeated computations to increase performance
  bool displayComputationTime;                    //!< AUTO: display computation time statistics at end of solving
  bool displayGlobalTimers;                       //!< AUTO: display global timer statistics at end of solving (e.g., for contact, but also for internal timings during development)
  bool displayStatistics;                         //!< AUTO: display general computation information at end of time step (steps, iterations, function calls, step rejections, ...
  LinearSolverType linearSolverType;              //!< AUTO: selection of numerical linear solver: exu.LinearSolverType.EXUdense (dense matrix inverse), exu.LinearSolverType.EigenSparse (sparse matrix LU-factorization), ... (enumeration type)
  Index outputPrecision;                          //!< AUTO: precision for floating point numbers written to console; e.g. values written by solver
  bool pauseAfterEachStep;                        //!< AUTO: pause after every time step or static load step(user press SPACE)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SimulationSettings()
  {
    cleanUpMemory = false;
    displayComputationTime = false;
    displayGlobalTimers = true;
    displayStatistics = false;
    linearSolverType = LinearSolverType::EXUdense;
    outputPrecision = 6;
    pauseAfterEachStep = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: precision for floating point numbers written to console; e.g. values written by solver
  void PySetOutputPrecision(const Index& outputPrecisionInit) { outputPrecision = EXUstd::GetSafelyUInt(outputPrecisionInit,"outputPrecision"); }
  //! AUTO: Read (Copy) access to: precision for floating point numbers written to console; e.g. values written by solver
  Index PyGetOutputPrecision() const { return Index(outputPrecision); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SimulationSettings" << ":\n";
    os << "  linearSolverSettings = " << linearSolverSettings << "\n";
    os << "  parallel = " << parallel << "\n";
    os << "  solutionSettings = " << solutionSettings << "\n";
    os << "  staticSolver = " << staticSolver << "\n";
    os << "  timeIntegration = " << timeIntegration << "\n";
    os << "  cleanUpMemory = " << cleanUpMemory << "\n";
    os << "  displayComputationTime = " << displayComputationTime << "\n";
    os << "  displayGlobalTimers = " << displayGlobalTimers << "\n";
    os << "  displayStatistics = " << displayStatistics << "\n";
    os << "  linearSolverType = " << linearSolverType << "\n";
    os << "  outputPrecision = " << outputPrecision << "\n";
    os << "  pauseAfterEachStep = " << pauseAfterEachStep << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SimulationSettings& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
