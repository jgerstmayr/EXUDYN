/** ***********************************************************************************************
* @class        SolutionFileExportSettings
* @brief        The quantities written into the coordinates solution file in addition to the coordinates.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
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

class SimulationSettings; //! AUTO: forward declaration for backlink

class SolutionFileExportSettings // AUTO: 
{
public: // AUTO: 
  bool accelerations;                             //!< AUTO: add ABRV:ODE2 accelerations to the solution file
  bool algebraicCoordinates;                      //!< AUTO: add algebraicCoordinates (=Lagrange multipliers) to the solution file
  bool dataCoordinates;                           //!< AUTO: add DataCoordinates to the solution file
  bool ODE1Velocities;                            //!< AUTO: add coordinatesODE1_t to the solution file
  bool velocities;                                //!< AUTO: add ABRV:ODE2 velocities to the solution file

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionFileExportSettings()
  {
    backlink=nullptr;
    accelerations = true;
    algebraicCoordinates = true;
    dataCoordinates = true;
    ODE1Velocities = true;
    velocities = true;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionFileExportSettings" << ":\n";
    os << "  accelerations = " << accelerations << "\n";
    os << "  algebraicCoordinates = " << algebraicCoordinates << "\n";
    os << "  dataCoordinates = " << dataCoordinates << "\n";
    os << "  ODE1Velocities = " << ODE1Velocities << "\n";
    os << "  velocities = " << velocities << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolutionFileExportSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolutionFileSettings
* @brief        The coordinates solution file: all coordinates of the system versus time, read by the SolutionViewer and exudyn.utilities.LoadSolutionFile.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class SolutionFileSettings // AUTO: 
{
public: // AUTO: 
  SolutionFileExportSettings exportSettings;      //!< AUTO: which quantities are written in addition to the coordinates
  bool append;                                    //!< AUTO: flag (true/false); if true, the solution and the solver information are appended to existing files (otherwise created); in BINARY mode, files are always replaced and this parameter is ineffective!
  bool binary;                                    //!< AUTO: if true, the solution file is written in binary format for improved speed and smaller file sizes; setting solution.precision >= 8 uses double (8 bytes), otherwise float (4 bytes) is used; note that append is ineffective and files are always replaced without asking! If not provided, file ending will read .sol in case of binary files and .txt in case of text files
  Index flushAboveCoordinates;                    //!< AUTO: must be > 0; number of coordinates above which the buffers of the solution file are always flushed, irrespectively of solution.flushFilesImmediately; for larger files, writing takes so much time that flushing does not add considerable time
  std::string information;                        //!< AUTO: special information added to header of solution file (e.g. parameters and settings, modes, ...); character encoding my be UTF-8, restricted to characters in [](#sec-utf8), but for compatibility, it is recommended to use ASCII characters only (95 characters, see wiki)
  std::string name;                               //!< AUTO: filename and (relative) path of the solution file containing all multibody system coordinates versus time; the default is in the directory solution/, like every file a run writes by default, so that nothing is written beside the script; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only; filename ending will be added automatically if not provided: .txt in case of text mode and .sol in case of binary solution files (binary=True)
  bool write;                                     //!< AUTO: flag (true/false), which determines if the coordinates are written to the solution file; standard quantities that are written are: solution is written as displacements and coordinatesODE1; for additional quantities, see export
  bool writeFooter;                               //!< AUTO: flag (true/false); if true, information at end of simulation is written: convergence, total solution time, statistics
  bool writeHeader;                               //!< AUTO: flag (true/false); if true, file header is written (turn off, e.g. for multiple runs of time integration)
  bool writeInitialValues;                        //!< AUTO: flag (true/false); if true, initial values are exported for the start time; applies to the solution file and the sensor files; this may not be wanted in the append file mode if the initial values are identical to the final values of a previous computation
  Real writePeriod;                               //!< AUTO: must be >= 0; time span (period), determines how often the solution file is written during a simulation

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionFileSettings()
  {
    backlink=nullptr;
    append = false;
    binary = false;
    flushAboveCoordinates = 10000;
    name = "solution/coordinatesSolution";
    write = true;
    writeFooter = true;
    writeHeader = true;
    writeInitialValues = true;
    writePeriod = 0.01;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    exportSettings.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionFileSettings" << ":\n";
    os << "  export = " << exportSettings << "\n";
    os << "  append = " << append << "\n";
    os << "  binary = " << binary << "\n";
    os << "  flushAboveCoordinates = " << flushAboveCoordinates << "\n";
    os << "  information = " << information << "\n";
    os << "  name = " << name << "\n";
    os << "  write = " << write << "\n";
    os << "  writeFooter = " << writeFooter << "\n";
    os << "  writeHeader = " << writeHeader << "\n";
    os << "  writeInitialValues = " << writeInitialValues << "\n";
    os << "  writePeriod = " << writePeriod << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolutionFileSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolutionSensorsSettings
* @brief        Storing and writing of the sensors.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class SolutionSensorsSettings // AUTO: 
{
public: // AUTO: 
  bool active;                                    //!< AUTO: flag (true/false); if false, no sensor files will be created and no sensor data will be stored; this may be advantageous for benchmarking as well as for special solvers which should not overwrite existing results (e.g. ComputeODE2Eigenvalues); settings this value to False may cause problems if sensors are required to perform operations which are needed e.g. in UserSensors as input of loads, etc.
  bool append;                                    //!< AUTO: flag (true/false); if true, sensor output is appended to existing file (otherwise created) or in case of internal storage, it is appended to existing currently stored data; this allows storing sensor values over different simulations
  bool writeFooter;                               //!< AUTO: flag (true/false); if true, file footer is written for sensor output (turn off, e.g. for multiple runs of time integration)
  bool writeHeader;                               //!< AUTO: flag (true/false); if true, file header is written for sensor output (turn off, e.g. for multiple runs of time integration)
  Real writePeriod;                               //!< AUTO: must be >= 0; time span (period), determines how often the sensor output is written to file or internal storage during a simulation

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionSensorsSettings()
  {
    backlink=nullptr;
    active = true;
    append = false;
    writeFooter = false;
    writeHeader = true;
    writePeriod = 0.01;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionSensorsSettings" << ":\n";
    os << "  active = " << active << "\n";
    os << "  append = " << append << "\n";
    os << "  writeFooter = " << writeFooter << "\n";
    os << "  writeHeader = " << writeHeader << "\n";
    os << "  writePeriod = " << writePeriod << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolutionSensorsSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolutionRestartSettings
* @brief        The restart file: the state of the system written regularly, from which a simulation can be continued.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class SolutionRestartSettings // AUTO: 
{
public: // AUTO: 
  std::string name;                               //!< AUTO: filename and (relative) path of text file for storing the solution after every writePeriod if write=True; directory will be created if it does not exist; backup file is created with ending .bck, which should be used if restart file is crashed; use Python utility function InitializeFromRestartFile(...) to consistently restart
  bool write;                                     //!< AUTO: flag (true/false), which determines if the restart file is written regularly, see name for details
  Real writePeriod;                               //!< AUTO: must be >= 0; time span (period), determines how often the restart file is updated; this should be often enough to enable restart without too much loss of data; too low values may influence performance

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionRestartSettings()
  {
    backlink=nullptr;
    name = "solution/restartFile.txt";
    write = false;
    writePeriod = 0.01;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionRestartSettings" << ":\n";
    os << "  name = " << name << "\n";
    os << "  write = " << write << "\n";
    os << "  writePeriod = " << writePeriod << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolutionRestartSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolutionSettings
* @brief        General settings for exporting the solution (results) of a simulation: the solution file, the sensors and the restart file.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class SolutionSettings // AUTO: 
{
public: // AUTO: 
  SolutionFileSettings file;                      //!< AUTO: the coordinates solution file
  SolutionRestartSettings restart;                //!< AUTO: the restart file
  SolutionSensorsSettings sensors;                //!< AUTO: storing and writing of the sensors
  bool flushFilesImmediately;                     //!< AUTO: flush file buffers after every period written (solution file and sensor files); if set False, the output is written through a buffer, which is highly efficient, but during simulation, files may be always in an incomplete state; if set True, this may add a large amount of CPU time as the process waits until files are really written to hard disc (especially for simulation of small scale systems, writing 10.000s of time steps; at least 5us per step/file, depending on hardware)
  Index precision;                                //!< AUTO: must be >= 0; precision for floating point numbers written to the solution and sensor files; the precision of the output to the console is consolePrecision
  Real recordImagesInterval;                      //!< AUTO: record frames of the main view in the renderer (images) during solving: amount of time to wait until next image (frame) is recorded; set recordImages = -1. if no images shall be recorded; set, e.g., recordImages = 0.01 to record an image every 10 milliseconds (requires that the time steps / load steps are sufficiently small!); for file names, etc., see VisualizationSettings.exportImages; note that only the main view (0) can be saved in this way, while for multiple views, you have to aquire data via renderer.RedrawAndGetImage()
  std::string solverInformationFileName;          //!< AUTO: filename and (relative) path of text file showing detailed information during solving; detail level according to yourSolver.verboseModeFile; if file.append is true, the information is appended in every solution step; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionSettings()
  {
    backlink=nullptr;
    flushFilesImmediately = false;
    precision = 10;
    recordImagesInterval = -1.;
    solverInformationFileName = "solution/solverInformation.txt";
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    file.Init(backlinkInit);
    restart.Init(backlinkInit);
    sensors.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionSettings" << ":\n";
    os << "  file = " << file << "\n";
    os << "  restart = " << restart << "\n";
    os << "  sensors = " << sensors << "\n";
    os << "  flushFilesImmediately = " << flushFilesImmediately << "\n";
    os << "  precision = " << precision << "\n";
    os << "  recordImagesInterval = " << recordImagesInterval << "\n";
    os << "  solverInformationFileName = " << solverInformationFileName << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolutionSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        SolutionSettingsDeprecated
* @brief        The settings of the solution as they were named up to Exudyn 1.11: each forwards to its place in simulationSettings.solution, with a DeprecationWarning.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class SolutionSettingsDeprecated // AUTO: 
{
private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SolutionSettingsDeprecated()
  {
    backlink=nullptr;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.append
  void PySetAppendToFile(const bool& appendInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.append
  bool PyGetAppendToFile() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.binary
  void PySetBinarySolutionFile(const bool& binaryInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.binary
  bool PyGetBinarySolutionFile() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.name
  void PySetCoordinatesSolutionFileName(const std::string& nameInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.name
  std::string PyGetCoordinatesSolutionFileName() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.export.accelerations
  void PySetExportAccelerations(const bool& accelerationsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.export.accelerations
  bool PyGetExportAccelerations() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.export.algebraicCoordinates
  void PySetExportAlgebraicCoordinates(const bool& algebraicCoordinatesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.export.algebraicCoordinates
  bool PyGetExportAlgebraicCoordinates() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.export.dataCoordinates
  void PySetExportDataCoordinates(const bool& dataCoordinatesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.export.dataCoordinates
  bool PyGetExportDataCoordinates() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.export.ODE1Velocities
  void PySetExportODE1Velocities(const bool& ODE1VelocitiesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.export.ODE1Velocities
  bool PyGetExportODE1Velocities() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.export.velocities
  void PySetExportVelocities(const bool& velocitiesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.export.velocities
  bool PyGetExportVelocities() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.flushAboveCoordinates
  void PySetFlushFilesDOF(const Index& flushAboveCoordinatesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.flushAboveCoordinates
  Index PyGetFlushFilesDOF() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.flushFilesImmediately
  void PySetFlushFilesImmediately(const bool& flushFilesImmediatelyInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.flushFilesImmediately
  bool PyGetFlushFilesImmediately() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.precision
  void PySetOutputPrecision(const Index& precisionInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.precision
  Index PyGetOutputPrecision() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.recordImagesInterval
  void PySetRecordImagesInterval(const Real& recordImagesIntervalInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.recordImagesInterval
  Real PyGetRecordImagesInterval() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.restart.name
  void PySetRestartFileName(const std::string& nameInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.restart.name
  std::string PyGetRestartFileName() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.restart.writePeriod
  void PySetRestartWritePeriod(const Real& writePeriodInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.restart.writePeriod
  Real PyGetRestartWritePeriod() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.sensors.append
  void PySetSensorsAppendToFile(const bool& appendInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.sensors.append
  bool PyGetSensorsAppendToFile() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.sensors.active
  void PySetSensorsStoreAndWriteFiles(const bool& activeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.sensors.active
  bool PyGetSensorsStoreAndWriteFiles() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.sensors.writeFooter
  void PySetSensorsWriteFileFooter(const bool& writeFooterInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.sensors.writeFooter
  bool PyGetSensorsWriteFileFooter() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.sensors.writeHeader
  void PySetSensorsWriteFileHeader(const bool& writeHeaderInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.sensors.writeHeader
  bool PyGetSensorsWriteFileHeader() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.sensors.writePeriod
  void PySetSensorsWritePeriod(const Real& writePeriodInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.sensors.writePeriod
  Real PyGetSensorsWritePeriod() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.information
  void PySetSolutionInformation(const std::string& informationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.information
  std::string PyGetSolutionInformation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.writePeriod
  void PySetSolutionWritePeriod(const Real& writePeriodInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.writePeriod
  Real PyGetSolutionWritePeriod() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.solverInformationFileName
  void PySetSolverInformationFileName(const std::string& solverInformationFileNameInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.solverInformationFileName
  std::string PyGetSolverInformationFileName() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.writeFooter
  void PySetWriteFileFooter(const bool& writeFooterInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.writeFooter
  bool PyGetWriteFileFooter() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.writeHeader
  void PySetWriteFileHeader(const bool& writeHeaderInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.writeHeader
  bool PyGetWriteFileHeader() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.writeInitialValues
  void PySetWriteInitialValues(const bool& writeInitialValuesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.writeInitialValues
  bool PyGetWriteInitialValues() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.restart.write
  void PySetWriteRestartFile(const bool& writeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.restart.write
  bool PyGetWriteRestartFile() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use solution.file.write
  void PySetWriteSolutionToFile(const bool& writeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use solution.file.write
  bool PyGetWriteSolutionToFile() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SolutionSettingsDeprecated" << ":\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SolutionSettingsDeprecated& object)
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class NumericalDifferentiationSettings // AUTO: 
{
public: // AUTO: 
  bool addReferenceCoordinatesToEpsilon;          //!< AUTO: True: for the size estimation of the differentiation parameter, the reference coordinate \f$q^{Ref}_i\f$ is added to ABRV:ODE2 coordinates --> see; False: only the current coordinate is used for size estimation of the differentiation parameter
  bool doSystemWideDifferentiation;               //!< AUTO: True: system wide differentiation (e.g. all ABRV:ODE2 equations w.r.t. all ABRV:ODE2 coordinates); False: only local (object) differentiation
  bool forAE;                                     //!< AUTO: flag (true/false); false = perform direct computation of jacobian for algebraic equations (AE), true = use numerical differentiation; as there must always exist an analytical implemented jacobian for AE, 'true' should only be used for verification
  bool forODE2;                                   //!< AUTO: flag (true/false); false = perform direct computation (e.g., using autodiff) of jacobian for ODE2 equations, true = use numerical differentiation; numerical differentiation is less efficient and may lead to numerical problems, but may smoothen problems of analytical derivatives; sometimes the analytical derivative may neglect terms
  bool forODE2Connectors;                         //!< AUTO: flag (true/false); false: if also forODE2==false, perform direct computation of jacobian for ODE2 terms for connectors; else: use numerical differentiation; NOTE: THIS FLAG IS FOR DEVELOPMENT AND WILL BE ERASED IN FUTURE
  bool jacobianConnectorDerivative;               //!< AUTO: True: for analytic Jacobians of connectors, the Jacobian derivative is computed, causing additional CPU costs and not beeing available for all connectors or markers (thus switching to numerical differentiation); False: Jacobian derivative is neglected in analytic Jacobians (but included in numerical Jacobians), which often has only minor influence on convergence
  Real minimumCoordinateSize;                     //!< AUTO: must be >= 0; minimum size of coordinates in relative differentiation parameter
  Real relativeEpsilon;                           //!< AUTO: must be >= 0; relative differentiation parameter epsilon; the numerical differentiation parameter \f$\varepsilon\f$ follows from the formula (\f$\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)\f$, with \f$\varepsilon_\mathrm{relative}\f$=relativeEpsilon, \f$q_{min} = \f$minimumCoordinateSize, \f$q_i\f$ is the current coordinate which is differentiated, and \f$qRef_i\f$ is the reference coordinate of the current coordinate

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  NumericalDifferentiationSettings()
  {
    backlink=nullptr;
    addReferenceCoordinatesToEpsilon = false;
    doSystemWideDifferentiation = false;
    forAE = false;
    forODE2 = false;
    forODE2Connectors = false;
    jacobianConnectorDerivative = true;
    minimumCoordinateSize = 0.01;
    relativeEpsilon = 1e-7;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use forODE2Connectors
  void PySetForODE2connectors(const bool& forODE2ConnectorsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use forODE2Connectors
  bool PyGetForODE2connectors() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "NumericalDifferentiationSettings" << ":\n";
    os << "  addReferenceCoordinatesToEpsilon = " << addReferenceCoordinatesToEpsilon << "\n";
    os << "  doSystemWideDifferentiation = " << doSystemWideDifferentiation << "\n";
    os << "  forAE = " << forAE << "\n";
    os << "  forODE2 = " << forODE2 << "\n";
    os << "  forODE2Connectors = " << forODE2Connectors << "\n";
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class DiscontinuousSettings // AUTO: 
{
public: // AUTO: 
  bool ignoreMaxIterations;                       //!< AUTO: continue solver if maximum number of discontinuous (post Newton) iterations is reached (ignore tolerance)
  Real iterationTolerance;                        //!< AUTO: must be >= 0; absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
  Index maxIterations;                            //!< AUTO: must be >= 0; maximum number of discontinuous (post Newton) iterations
  bool useRecommendedStepSize;                    //!< AUTO: some objects (contact-related) provide a recommendedStepSize; if True, this recommendation is used, but may lead to very small step sizes and solver could fail if restrictions are too hard; set to False to ignore this recommendation

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  DiscontinuousSettings()
  {
    backlink=nullptr;
    ignoreMaxIterations = true;
    iterationTolerance = 1;
    maxIterations = 5;
    useRecommendedStepSize = true;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "DiscontinuousSettings" << ":\n";
    os << "  ignoreMaxIterations = " << ignoreMaxIterations << "\n";
    os << "  iterationTolerance = " << iterationTolerance << "\n";
    os << "  maxIterations = " << maxIterations << "\n";
    os << "  useRecommendedStepSize = " << useRecommendedStepSize << "\n";
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class NewtonSettings // AUTO: 
{
public: // AUTO: 
  NumericalDifferentiationSettings numericalDifferentiation;//!< AUTO: numerical differentiation parameters for numerical jacobian (e.g. Newton in static solver or implicit time integration)
  Real absoluteTolerance;                         //!< AUTO: must be >= 0; absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
  bool active;                                    //!< AUTO: flag (true/false); false = linear computation, true = use Newton solver for nonlinear solution
  bool adaptInitialResidual;                      //!< AUTO: flag (true/false); false = standard; True: if initialResidual is very small (or zero), it may increase significantely in the first Newton iteration; to achieve relativeTolerance, the initialResidual will by updated by a higher residual within the first Newton iteration
  Real maximumSolutionNorm;                       //!< AUTO: must be >= 0; this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (i.e., value=\f$u_1^2\f$+\f$u_2^2\f$+...), and solutionV/A...; if the norm of solution vectors is larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
  Index maxIterations;                            //!< AUTO: must be >= 0; maximum number of iterations (including modified + restart Newton iterations); after that total number of iterations, the static/dynamic solver refines the step size or stops with an error
  Index maxModifiedNewtonIterations;              //!< AUTO: must be >= 0; maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
  Index maxModifiedNewtonRestartIterations;       //!< AUTO: must be >= 0; maximum number of iterations for modified Newton after a Jacobian update; after that number of iterations, the full Newton method is started for this step
  Real modifiedNewtonContractivity;               //!< AUTO: must be > 0; maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
  bool modifiedNewtonJacUpdatePerStep;            //!< AUTO: True: compute Jacobian at every time step (or static step), but not in every Newton iteration (except for bad convergence ==> switch to full Newton)
  Real relativeTolerance;                         //!< AUTO: must be >= 0; relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
  Index residualMode;                             //!< AUTO: must be >= 0; 0 ... use residual for computation of error (standard); 1 ... use ABRV:ODE2 and ABRV:ODE1 newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
  bool useModifiedNewton;                         //!< AUTO: True: compute Jacobian only at first call to solver; the Jacobian (and its factorizations) is not computed in each Newton iteration, even not in every (time integration) step; False: Jacobian (and factorization) is computed in every Newton iteration (default, but may be costly)
  bool weightTolerancePerCoordinate;              //!< AUTO: flag (true/false); false = compute error as L2-Norm of residual; true = compute error as (L2-Norm of residual) / (sqrt(number of coordinates)), which can help to use common tolerance independent of system size

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  NewtonSettings()
  {
    backlink=nullptr;
    absoluteTolerance = 1e-10;
    active = true;
    adaptInitialResidual = true;
    maximumSolutionNorm = 1e38;
    maxIterations = 25;
    maxModifiedNewtonIterations = 8;
    maxModifiedNewtonRestartIterations = 7;
    modifiedNewtonContractivity = 0.5;
    modifiedNewtonJacUpdatePerStep = false;
    relativeTolerance = 1e-8;
    residualMode = 0;
    useModifiedNewton = false;
    weightTolerancePerCoordinate = false;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    numericalDifferentiation.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use residualMode
  void PySetNewtonResidualMode(const Index& residualModeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use residualMode
  Index PyGetNewtonResidualMode() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use active
  void PySetUseNewtonSolver(const bool& activeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use active
  bool PyGetUseNewtonSolver() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "NewtonSettings" << ":\n";
    os << "  numericalDifferentiation = " << numericalDifferentiation << "\n";
    os << "  absoluteTolerance = " << absoluteTolerance << "\n";
    os << "  active = " << active << "\n";
    os << "  adaptInitialResidual = " << adaptInitialResidual << "\n";
    os << "  maximumSolutionNorm = " << maximumSolutionNorm << "\n";
    os << "  maxIterations = " << maxIterations << "\n";
    os << "  maxModifiedNewtonIterations = " << maxModifiedNewtonIterations << "\n";
    os << "  maxModifiedNewtonRestartIterations = " << maxModifiedNewtonRestartIterations << "\n";
    os << "  modifiedNewtonContractivity = " << modifiedNewtonContractivity << "\n";
    os << "  modifiedNewtonJacUpdatePerStep = " << modifiedNewtonJacUpdatePerStep << "\n";
    os << "  relativeTolerance = " << relativeTolerance << "\n";
    os << "  residualMode = " << residualMode << "\n";
    os << "  useModifiedNewton = " << useModifiedNewton << "\n";
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class GeneralizedAlphaSettings // AUTO: 
{
public: // AUTO: 
  bool computeInitialAccelerations;               //!< AUTO: True: compute initial accelerations from system EOM in acceleration form; NOTE that initial accelerations that are following from user functions in constraints are not considered for now! False: use zero accelerations
  bool lieGroupAddTangentOperator;                //!< AUTO: True: for Lie group nodes, in case that lieGroupSimplifiedKinematicRelations=True, the integrator adds the tangent operator for stiffness and constraint matrices, for improved Newton convergence; not available for sparse matrix mode (EigenSparse)
  bool lieGroupSimplifiedKinematicRelations;      //!< AUTO: True: for Lie group nodes, the integrator uses the original kinematic relations of the Bruls and Cardona 2010 paper; False (recommended): higher accuracy as proposed in paper by Holzinger, Arnold, Gerstmayr, sigma-modified Lie group generalized alpha methods for constrained multibody systems, 2025 (to be sumitted)
  Real newmarkBeta;                               //!< AUTO: must be >= 0; value beta for Newmark method; default value beta = \f$\frac 1 4\f$ corresponds to (undamped) trapezoidal rule
  Real newmarkGamma;                              //!< AUTO: must be >= 0; value gamma for Newmark method; default value gamma = \f$\frac 1 2\f$ corresponds to (undamped) trapezoidal rule
  bool resetAccelerations;                        //!< AUTO: this flag only affects if computeInitialAccelerations=False: if resetAccelerations=True, accelerations are set zero in the solver function InitializeSolverInitialConditions; this may be unwanted in case of repeatedly called SolveSteps() and in cases where solutions shall be prolonged from previous computations
  Real spectralRadius;                            //!< AUTO: must be >= 0; spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
  bool storeInitialAlgebraicCoordinates;          //!< AUTO: True: IF computeInitialAccelerations=True, store initial algebraic coordinates (usually the Lagrange multipliers) in the initial coordinates vector (and thus in the first line of the coordinates solution file); for further details on limitations, see computeInitialAccelerations
  bool useIndex2Constraints;                      //!< AUTO: set useIndex2Constraints = true in order to use index2 (velocity level constraints) formulation
  bool useNewmark;                                //!< AUTO: if true, use Newmark method with beta and gamma instead of generalized-Alpha

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  GeneralizedAlphaSettings()
  {
    backlink=nullptr;
    computeInitialAccelerations = true;
    lieGroupAddTangentOperator = true;
    lieGroupSimplifiedKinematicRelations = false;
    newmarkBeta = 0.25;
    newmarkGamma = 0.5;
    resetAccelerations = false;
    spectralRadius = 0.9;
    storeInitialAlgebraicCoordinates = true;
    useIndex2Constraints = false;
    useNewmark = false;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "GeneralizedAlphaSettings" << ":\n";
    os << "  computeInitialAccelerations = " << computeInitialAccelerations << "\n";
    os << "  lieGroupAddTangentOperator = " << lieGroupAddTangentOperator << "\n";
    os << "  lieGroupSimplifiedKinematicRelations = " << lieGroupSimplifiedKinematicRelations << "\n";
    os << "  newmarkBeta = " << newmarkBeta << "\n";
    os << "  newmarkGamma = " << newmarkGamma << "\n";
    os << "  resetAccelerations = " << resetAccelerations << "\n";
    os << "  spectralRadius = " << spectralRadius << "\n";
    os << "  storeInitialAlgebraicCoordinates = " << storeInitialAlgebraicCoordinates << "\n";
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class ExplicitIntegrationSettings // AUTO: 
{
public: // AUTO: 
  bool computeEndOfStepAccelerations;             //!< AUTO: accelerations are computed at stages of the explicit integration scheme; if the user needs accelerations at the end of a step, this flag needs to be activated; if True, this causes a second call to the RHS of the equations, which may DOUBLE COMPUTATIONAL COSTS for one-step-methods; if False, the accelerations are re-used from the last stage, being slightly different
  bool computeMassMatrixInversePerBody;           //!< AUTO: If true, the solver assumes the bodies to be independent and computes the inverse of the mass matrix for all bodies independently; this may lead to WRONG RESULTS, if bodies share nodes, e.g., two MassPoint objects put on the same node or a beam with a mass point attached at a shared node; however, it may speed up explicit time integration for large systems significantly (multi-threaded) - together with a sparse solver, linearSolver.solverType = exu.LinearSolverType.EigenSparse: with the dense default the inverse is stored as a dense matrix and every step costs O(n^2) (#2400)
  bool eliminateConstraints;                      //!< AUTO: True: make explicit solver work for simple CoordinateConstraints, which are eliminated for ground constraints (e.g. fixed nodes in finite element models). False: incompatible constraints are ignored (BE CAREFUL)!
  bool useLieGroupIntegration;                    //!< AUTO: True: use Lie group integration for rigid body nodes; must be turned on for Lie group nodes (without data coordinates) to work properly; does not work for nodes with data coordinates!

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  ExplicitIntegrationSettings()
  {
    backlink=nullptr;
    computeEndOfStepAccelerations = true;
    computeMassMatrixInversePerBody = false;
    eliminateConstraints = true;
    useLieGroupIntegration = true;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "ExplicitIntegrationSettings" << ":\n";
    os << "  computeEndOfStepAccelerations = " << computeEndOfStepAccelerations << "\n";
    os << "  computeMassMatrixInversePerBody = " << computeMassMatrixInversePerBody << "\n";
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
* @class        ExplicitIntegrationSettingsDeprecated
* @brief        The settings of the explicit solvers as they were named up to Exudyn 1.11: each forwards to its place in simulationSettings.timeIntegration, with a DeprecationWarning.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class ExplicitIntegrationSettingsDeprecated // AUTO: 
{
private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  ExplicitIntegrationSettingsDeprecated()
  {
    backlink=nullptr;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.explicit.computeEndOfStepAccelerations
  void PySetComputeEndOfStepAccelerations(const bool& computeEndOfStepAccelerationsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.explicit.computeEndOfStepAccelerations
  bool PyGetComputeEndOfStepAccelerations() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.explicit.computeMassMatrixInversePerBody
  void PySetComputeMassMatrixInversePerBody(const bool& computeMassMatrixInversePerBodyInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.explicit.computeMassMatrixInversePerBody
  bool PyGetComputeMassMatrixInversePerBody() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.solverType
  void PySetDynamicSolverType(const DynamicSolverType& solverTypeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.solverType
  DynamicSolverType PyGetDynamicSolverType() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.explicit.eliminateConstraints
  void PySetEliminateConstraints(const bool& eliminateConstraintsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.explicit.eliminateConstraints
  bool PyGetEliminateConstraints() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.explicit.useLieGroupIntegration
  void PySetUseLieGroupIntegration(const bool& useLieGroupIntegrationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.explicit.useLieGroupIntegration
  bool PyGetUseLieGroupIntegration() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "ExplicitIntegrationSettingsDeprecated" << ":\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const ExplicitIntegrationSettingsDeprecated& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        RealtimeSettings
* @brief        Simulation in realtime: the time integration waits until the CPU time has reached the simulation time.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class RealtimeSettings // AUTO: 
{
public: // AUTO: 
  bool active;                                    //!< AUTO: True: simulate in realtime; the solver waits for computation of the next step until the CPU time reached the simulation time; if the simulation is slower than realtime, it simply continues
  Real factor;                                    //!< AUTO: must be > 0; if active=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
  Index waitMicroseconds;                         //!< AUTO: must be > 0; if active=True, a loop runs which waits waitMicroseconds until checking again if the realtime is reached; using larger values leads to less CPU usage but less accurate realtime accuracy; smaller values (< 1000) increase CPU usage but improve realtime accuracy

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  RealtimeSettings()
  {
    backlink=nullptr;
    active = false;
    factor = 1;
    waitMicroseconds = 1000;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "RealtimeSettings" << ":\n";
    os << "  active = " << active << "\n";
    os << "  factor = " << factor << "\n";
    os << "  waitMicroseconds = " << waitMicroseconds << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const RealtimeSettings& object)
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class TimeIntegrationSettings // AUTO: 
{
public: // AUTO: 
  DiscontinuousSettings discontinuous;            //!< AUTO: parameters for treatment of discontinuities
  ExplicitIntegrationSettings explicitSettings;   //!< AUTO: special parameters for explicit time integration
  ExplicitIntegrationSettingsDeprecated explicitIntegration;//!< AUTO: DEPRECATED; Instead use deprecated, use timeIntegration.explicit and timeIntegration.solverType
  GeneralizedAlphaSettings generalizedAlpha;      //!< AUTO: parameters for generalized-alpha, implicit trapezoidal rule or Newmark (options only apply for these methods)
  NewtonSettings newton;                          //!< AUTO: parameters for Newton method; used for implicit time integration methods only
  RealtimeSettings realtime;                      //!< AUTO: simulation in realtime
  Real absoluteTolerance;                         //!< AUTO: must be >= 0; \f$a_{tol}\f$: if automaticStepSize=True, absolute tolerance for the error control; must fulfill \f$a_{tol} > 0\f$; see [](#sec-explicitsolver)
  bool adaptiveStep;                              //!< AUTO: True: the step size may be reduced if step fails; no automatic stepsize control
  Real adaptiveStepDecrease;                      //!< AUTO: must be >= 0; Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real adaptiveStepIncrease;                      //!< AUTO: must be >= 0; Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Index adaptiveStepRecoveryIterations;           //!< AUTO: must be >= 0; Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  Index adaptiveStepRecoverySteps;                //!< AUTO: must be >= 0; Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  bool automaticStepSize;                         //!< AUTO: True: for specific integrators with error control (e.g., DOPRI5), compute automatic step size based on error estimation; False: constant step size (step may be reduced if adaptiveStep=True); the maximum stepSize reads \f$h = h_{max} = \frac{t_{end} - t_{start}}{n_{steps}}\f$
  Index computeLoadsJacobian;                     //!< AUTO: must be >= 0; 0:  jacobian of loads not considered (may lead to slow convergence or Newton failure); 1: in case of implicit integrators, compute (numerical) Jacobian of ODE2 and ODE1 coordinates for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; 2: also compute ODE2_t dependencies for jacobian; note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
  Real endTime;                                   //!< AUTO: must be >= 0; \f$t_{end}\f$: end time of time integration
  Real initialStepSize;                           //!< AUTO: must be >= 0; \f$h_{init}\f$: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
  Real minimumStepSize;                           //!< AUTO: must be > 0; \f$h_{min}\f$: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
  Real numberOfSteps;                             //!< AUTO: must be > 0; \f$n_{steps}\f$: number of steps in time integration; (maximum) stepSize \f$h\f$ is computed from \f$h = \frac{t_{end} - t_{start}}{n_{steps}}\f$; for automatic stepsize control, this stepSize is the maximum steps size, \f$h_{max} = h\f$; numberOfSteps can also be a float type, but must be close to an integer (relative tolerance \f$100\cdot\varepsilon\f$) as it is silently rounded to int
  Real relativeTolerance;                         //!< AUTO: must be >= 0; \f$r_{tol}\f$: if automaticStepSize=True, relative tolerance for the error control; must fulfill \f$r_{tol} \ge 0\f$; see [](#sec-explicitsolver)
  bool reuseConstantMassMatrix;                   //!< AUTO: True: does not recompute constant mass matrices (e.g. of some finite elements, mass points, etc.); if False, it always recomputes the mass matrix (e.g. needed, if user changes mass parameters via Python)
  DynamicSolverType solverType;                   //!< AUTO: the solver of mbs.SolveDynamic(...): an implicit one (GeneralizedAlpha, TrapezoidalIndex2, ...) or an explicit one (DOPRI5, ExplicitEuler, RK44, ...), see DynamicSolverType, [](#sec-dynamicsolvertype); the argument solverType of mbs.SolveDynamic, if given, takes its place for that run
  Real startTime;                                 //!< AUTO: must be >= 0; \f$t_{start}\f$: start time of time integration (usually set to zero)
  Index stepInformation;                          //!< AUTO: must be >= 0; add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  Real stepSizeMaxIncrease;                       //!< AUTO: must be >= 0; \f$f_{maxInc}\f$: if automaticStepSize=True, maximum increase of step size per step, see [](#sec-explicitsolver); make this factor smaller (but \f$> 1\f$) if too many rejected steps
  Real stepSizeSafety;                            //!< AUTO: must be >= 0; \f$r_{sfty}\f$: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see [](#sec-explicitsolver). Make this factor smaller if many steps are rejected.
  Index verboseMode;                              //!< AUTO: must be >= 0; 0 ... no output, 1 ... show short step information every 2 seconds (every 30 seconds after 1 hour CPU time), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
  Index verboseModeFile;                          //!< AUTO: must be >= 0; same behaviour as verboseMode, but outputs all solver information to file

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  TimeIntegrationSettings()
  {
    backlink=nullptr;
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
    relativeTolerance = 1e-8;
    reuseConstantMassMatrix = true;
    solverType = DynamicSolverType::GeneralizedAlpha;
    startTime = 0;
    stepInformation = 67;
    stepSizeMaxIncrease = 2;
    stepSizeSafety = 0.9;
    verboseMode = 0;
    verboseModeFile = 0;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    discontinuous.Init(backlinkInit);
    explicitSettings.Init(backlinkInit);
    explicitIntegration.Init(backlinkInit);
    generalizedAlpha.Init(backlinkInit);
    newton.Init(backlinkInit);
    realtime.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.realtime.factor
  void PySetRealtimeFactor(const Real& factorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.realtime.factor
  Real PyGetRealtimeFactor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.realtime.waitMicroseconds
  void PySetRealtimeWaitMicroseconds(const Index& waitMicrosecondsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.realtime.waitMicroseconds
  Index PyGetRealtimeWaitMicroseconds() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use timeIntegration.realtime.active
  void PySetSimulateInRealtime(const bool& activeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use timeIntegration.realtime.active
  bool PyGetSimulateInRealtime() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "TimeIntegrationSettings" << ":\n";
    os << "  discontinuous = " << discontinuous << "\n";
    os << "  explicit = " << explicitSettings << "\n";
    os << "  generalizedAlpha = " << generalizedAlpha << "\n";
    os << "  newton = " << newton << "\n";
    os << "  realtime = " << realtime << "\n";
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
    os << "  relativeTolerance = " << relativeTolerance << "\n";
    os << "  reuseConstantMassMatrix = " << reuseConstantMassMatrix << "\n";
    os << "  solverType = " << solverType << "\n";
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class StaticSolverSettings // AUTO: 
{
public: // AUTO: 
  DiscontinuousSettings discontinuous;            //!< AUTO: parameters for treatment of discontinuities
  NewtonSettings newton;                          //!< AUTO: parameters for Newton method (e.g. in static solver or time integration)
  bool adaptiveStep;                              //!< AUTO: True: use step reduction if step fails; False: fixed step size
  Real adaptiveStepDecrease;                      //!< AUTO: must be >= 0; Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
  Real adaptiveStepIncrease;                      //!< AUTO: must be >= 0; Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
  Index adaptiveStepRecoveryIterations;           //!< AUTO: must be >= 0; Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
  Index adaptiveStepRecoverySteps;                //!< AUTO: must be >= 0; Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
  bool computeLoadsJacobian;                      //!< AUTO: True: compute (currently numerical) Jacobian for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; False: jacobian of loads not considered (may lead to slow convergence or Newton failure); note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
  bool constrainODE1Coordinates;                  //!< AUTO: True: ODE1coordinates are constrained to initial values; False: undefined behavior, currently not supported
  Real loadStepDuration;                          //!< AUTO: must be > 0; quasi-time for all load steps (added to current time in load steps)
  bool loadStepGeometric;                         //!< AUTO: if loadStepGeometric=false, the load steps are incremental (arithmetic series, e.g. 0.1,0.2,0.3,...); if true, the load steps are increased in a geometric series, e.g. for \f$n=8\f$ numberOfLoadSteps and \f$d = 1000\f$ loadStepGeometricRange, it follows: \f$1000^{1/8}/1000=0.00237\f$, \f$1000^{2/8}/1000=0.00562\f$, \f$1000^{3/8}/1000=0.0133\f$, ..., \f$1000^{7/8}/1000=0.422\f$, \f$1000^{8/8}/1000=1\f$
  Real loadStepGeometricRange;                    //!< AUTO: must be > 0; if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
  Real loadStepStart;                             //!< AUTO: must be >= 0; a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
  Real minimumStepSize;                           //!< AUTO: must be > 0; lower limit of step size, before nonlinear solver stops
  Index numberOfLoadSteps;                        //!< AUTO: must be > 0; number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
  Real stabilizerODE2term;                        //!< AUTO: must be >= 0; add mass-proportional stabilizer term in ABRV:ODE2 part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
  Index stepInformation;                          //!< AUTO: must be >= 0; add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
  bool useLoadFactor;                             //!< AUTO: True: compute a load factor \f$\in [0,1]\f$ from static step time; all loads are scaled by the load factor; False: loads are always scaled with 1 -- use this option if time dependent loads use a userFunction
  Index verboseMode;                              //!< AUTO: must be >= 0; 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
  Index verboseModeFile;                          //!< AUTO: must be >= 0; same behaviour as verboseMode, but outputs all solver information to file

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  StaticSolverSettings()
  {
    backlink=nullptr;
    adaptiveStep = true;
    adaptiveStepDecrease = 0.25;
    adaptiveStepIncrease = 2;
    adaptiveStepRecoveryIterations = 7;
    adaptiveStepRecoverySteps = 4;
    computeLoadsJacobian = true;
    constrainODE1Coordinates = true;
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
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    discontinuous.Init(backlinkInit);
    newton.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use constrainODE1Coordinates
  void PySetConstrainODE1coordinates(const bool& constrainODE1CoordinatesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use constrainODE1Coordinates
  bool PyGetConstrainODE1coordinates() const ;

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
    os << "  constrainODE1Coordinates = " << constrainODE1Coordinates << "\n";
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class LinearSolverSettings // AUTO: 
{
public: // AUTO: 
  bool ignoreSingularJacobian;                    //!< AUTO: [ONLY implemented for dense, Eigen matrix mode] False: standard way, fails if jacobian is singular; True: use Eigen's FullPivLU (thus only works with LinearSolverType.EigenDense) which handles over- and underdetermined systems; can often resolve redundant constraints, but MAY ALSO LEAD TO ERRONEOUS RESULTS!
  Real pivotThreshold;                            //!< AUTO: must be >= 0; [ONLY available for EXUdense and EigenDense (FullPivot) solver] threshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
  bool reuseAnalyzedPattern;                      //!< AUTO: [ONLY available for sparse matrices] True: the Eigen SparseLU solver offers the possibility to reuse an analyzed pattern of a previous factorization; this may reduce total factorization time by a factor of 2 or 3, depending on the matrix type; however, if the matrix patterns heavily change between computations, this may even slow down performance; this flag is set for SparseMatrices in InitializeSolverData(...) and should be handled with care!
  bool showCausingItems;                          //!< AUTO: False: no output, if solver fails; True: if redundant equations appear, they are resolved such that according solution variables are set to zero; in case of redundant constraints, this may help, but it may lead to erroneous behaviour; for static problems, this may suppress static motion or resolve problems in case of instabilities, but should in general be considered with care!
  LinearSolverType solverType;                    //!< AUTO: selection of numerical linear solver: exu.LinearSolverType.EXUdense (dense matrix inverse), exu.LinearSolverType.EigenSparse (sparse matrix LU-factorization), ... (enumeration type)

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  LinearSolverSettings()
  {
    backlink=nullptr;
    ignoreSingularJacobian = false;
    pivotThreshold = 0;
    reuseAnalyzedPattern = false;
    showCausingItems = true;
    solverType = LinearSolverType::EXUdense;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "LinearSolverSettings" << ":\n";
    os << "  ignoreSingularJacobian = " << ignoreSingularJacobian << "\n";
    os << "  pivotThreshold = " << pivotThreshold << "\n";
    os << "  reuseAnalyzedPattern = " << reuseAnalyzedPattern << "\n";
    os << "  showCausingItems = " << showCausingItems << "\n";
    os << "  solverType = " << solverType << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const LinearSolverSettings& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        LinearSolverSettingsDeprecated
* @brief        The settings of the linear solver as they were named up to Exudyn 1.11: each forwards to its place in simulationSettings.linearSolver, with a DeprecationWarning.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class LinearSolverSettingsDeprecated // AUTO: 
{
private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  LinearSolverSettingsDeprecated()
  {
    backlink=nullptr;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use linearSolver.ignoreSingularJacobian
  void PySetIgnoreSingularJacobian(const bool& ignoreSingularJacobianInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use linearSolver.ignoreSingularJacobian
  bool PyGetIgnoreSingularJacobian() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use linearSolver.pivotThreshold
  void PySetPivotThreshold(const Real& pivotThresholdInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use linearSolver.pivotThreshold
  Real PyGetPivotThreshold() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use linearSolver.reuseAnalyzedPattern
  void PySetReuseAnalyzedPattern(const bool& reuseAnalyzedPatternInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use linearSolver.reuseAnalyzedPattern
  bool PyGetReuseAnalyzedPattern() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use linearSolver.showCausingItems
  void PySetShowCausingItems(const bool& showCausingItemsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use linearSolver.showCausingItems
  bool PyGetShowCausingItems() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "LinearSolverSettingsDeprecated" << ":\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const LinearSolverSettingsDeprecated& object)
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class Parallel // AUTO: 
{
public: // AUTO: 
  Index multithreadedLowerLimitJacobians;         //!< AUTO: must be > 0; compute jacobians (ODE2, AE, ...) multi-threaded; this is the limit number of according objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index multithreadedLowerLimitLoads;             //!< AUTO: must be > 0; compute loads multi-threaded; this is the limit number of loads from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index multithreadedLowerLimitMassMatrices;      //!< AUTO: must be > 0; compute bodies mass matrices multi-threaded; this is the limit number of bodies from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index multithreadedLowerLimitResiduals;         //!< AUTO: must be > 0; compute RHS vectors, AE, and reaction forces multi-threaded; this is the limit number of objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index numberOfThreads;                          //!< AUTO: must be > 0; number of threads used for parallel computation (1 == scalar processing); do not use more threads than available threads (in most cases it is good to restrict to the number of cores); currently, only one solver can be started with multithreading; if you use several mbs in parallel (co-simulation), you should use serial computing
  Index taskSplitMinItems;                        //!< AUTO: must be > 0; number of items from which on the tasks are split into subtasks (which slightly increases threading performance; this may be critical for smaller number of objects, should be roughly between 50 and 5000; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  Index taskSplitTasksPerThread;                  //!< AUTO: must be > 0; this is the number of subtasks that every thread receives; minimum is 1, the maximum should not be larger than 100; this factor is 1 as long as the taskSplitMinItems is not reached; flag is copied into MainSystem internal flag at InitializeSolverData(...)
  bool useLoadBalancing;                          //!< AUTO: if True, parallel computation uses load balancing, which may give better performance in case of non-equilibrated loads; (mobile) Intel CPUs may perform better without load balancing; this flag is coupled to exudyn.special.solver.multiThreadingLoadBalancing (overwritten when solver starts with multithreading)

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  Parallel()
  {
    backlink=nullptr;
    multithreadedLowerLimitJacobians = 20;
    multithreadedLowerLimitLoads = 20;
    multithreadedLowerLimitMassMatrices = 20;
    multithreadedLowerLimitResiduals = 20;
    numberOfThreads = 1;
    taskSplitMinItems = 50;
    taskSplitTasksPerThread = 16;
    useLoadBalancing = true;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use multithreadedLowerLimitJacobians
  void PySetMultithreadedLLimitJacobians(const Index& multithreadedLowerLimitJacobiansInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use multithreadedLowerLimitJacobians
  Index PyGetMultithreadedLLimitJacobians() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use multithreadedLowerLimitLoads
  void PySetMultithreadedLLimitLoads(const Index& multithreadedLowerLimitLoadsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use multithreadedLowerLimitLoads
  Index PyGetMultithreadedLLimitLoads() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use multithreadedLowerLimitMassMatrices
  void PySetMultithreadedLLimitMassMatrices(const Index& multithreadedLowerLimitMassMatricesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use multithreadedLowerLimitMassMatrices
  Index PyGetMultithreadedLLimitMassMatrices() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use multithreadedLowerLimitResiduals
  void PySetMultithreadedLLimitResiduals(const Index& multithreadedLowerLimitResidualsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use multithreadedLowerLimitResiduals
  Index PyGetMultithreadedLLimitResiduals() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "Parallel" << ":\n";
    os << "  multithreadedLowerLimitJacobians = " << multithreadedLowerLimitJacobians << "\n";
    os << "  multithreadedLowerLimitLoads = " << multithreadedLowerLimitLoads << "\n";
    os << "  multithreadedLowerLimitMassMatrices = " << multithreadedLowerLimitMassMatrices << "\n";
    os << "  multithreadedLowerLimitResiduals = " << multithreadedLowerLimitResiduals << "\n";
    os << "  numberOfThreads = " << numberOfThreads << "\n";
    os << "  taskSplitMinItems = " << taskSplitMinItems << "\n";
    os << "  taskSplitTasksPerThread = " << taskSplitTasksPerThread << "\n";
    os << "  useLoadBalancing = " << useLoadBalancing << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const Parallel& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        ShowSettings
* @brief        What the solvers show in the console at the end of solving.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class ShowSettings // AUTO: 
{
public: // AUTO: 
  bool computationTime;                           //!< AUTO: display computation time statistics at end of solving
  bool globalTimers;                              //!< AUTO: display global timer statistics at end of solving (e.g., for contact, but also for internal timings during development)
  bool statistics;                                //!< AUTO: display general computation information at end of time step (steps, iterations, function calls, step rejections, ...

private: // AUTO: 
  SimulationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  ShowSettings()
  {
    backlink=nullptr;
    computationTime = false;
    globalTimers = true;
    statistics = false;
  };
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "ShowSettings" << ":\n";
    os << "  computationTime = " << computationTime << "\n";
    os << "  globalTimers = " << globalTimers << "\n";
    os << "  statistics = " << statistics << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const ShowSettings& object)
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
* @date         AUTO: 2026-10-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class SimulationSettings // AUTO: 
{
public: // AUTO: 
  LinearSolverSettings linearSolver;              //!< AUTO: linear solver parameters (used for dense and sparse solvers)
  LinearSolverSettingsDeprecated linearSolverSettings;//!< AUTO: DEPRECATED; Instead use deprecated, use linearSolver
  Parallel parallel;                              //!< AUTO: parameters for vectorized and parallelized (multi-threaded) computations
  ShowSettings show;                              //!< AUTO: what the solvers show in the console at the end of solving
  SolutionSettings solution;                      //!< AUTO: settings for solution files
  SolutionSettingsDeprecated solutionSettings;    //!< AUTO: DEPRECATED; Instead use deprecated, use solution
  StaticSolverSettings staticSolver;              //!< AUTO: static solver parameters
  TimeIntegrationSettings timeIntegration;        //!< AUTO: time integration parameters
  bool cleanUpMemory;                             //!< AUTO: True: solvers will free memory at exit (recommended for large systems); False: keep allocated memory for repeated computations to increase performance
  Index consolePrecision;                         //!< AUTO: must be >= 0; precision for floating point numbers written to the console, e.g. the values written by the solver; the precision of the solution and sensor files is solution.precision
  bool pauseAfterEachStep;                        //!< AUTO: pause after every time step or static load step(user press SPACE)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  SimulationSettings()
  {
    cleanUpMemory = false;
    consolePrecision = 6;
    pauseAfterEachStep = false;
    Init(this);
  };
  //! AUTO: copy constructor: a copy links ITSELF, not the original (#2603)
  SimulationSettings(const SimulationSettings& other)
  {
    linearSolver = other.linearSolver;
    linearSolverSettings = other.linearSolverSettings;
    parallel = other.parallel;
    show = other.show;
    solution = other.solution;
    solutionSettings = other.solutionSettings;
    staticSolver = other.staticSolver;
    timeIntegration = other.timeIntegration;
    cleanUpMemory = other.cleanUpMemory;
    consolePrecision = other.consolePrecision;
    pauseAfterEachStep = other.pauseAfterEachStep;
    Init(this);
  }
  //! AUTO: copy assignment, for the same reason
  SimulationSettings& operator=(const SimulationSettings& other)
  {
    if (this != &other)
    {
      linearSolver = other.linearSolver;
      linearSolverSettings = other.linearSolverSettings;
      parallel = other.parallel;
      show = other.show;
      solution = other.solution;
      solutionSettings = other.solutionSettings;
      staticSolver = other.staticSolver;
      timeIntegration = other.timeIntegration;
      cleanUpMemory = other.cleanUpMemory;
      consolePrecision = other.consolePrecision;
      pauseAfterEachStep = other.pauseAfterEachStep;
      Init(this);
    }
    return *this;
  }
  void Init(SimulationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    linearSolver.Init(backlinkInit);
    linearSolverSettings.Init(backlinkInit);
    parallel.Init(backlinkInit);
    show.Init(backlinkInit);
    solution.Init(backlinkInit);
    solutionSettings.Init(backlinkInit);
    staticSolver.Init(backlinkInit);
    timeIntegration.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use show.computationTime
  void PySetDisplayComputationTime(const bool& computationTimeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use show.computationTime
  bool PyGetDisplayComputationTime() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use show.globalTimers
  void PySetDisplayGlobalTimers(const bool& globalTimersInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use show.globalTimers
  bool PyGetDisplayGlobalTimers() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use show.statistics
  void PySetDisplayStatistics(const bool& statisticsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use show.statistics
  bool PyGetDisplayStatistics() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use linearSolver.solverType
  void PySetLinearSolverType(const LinearSolverType& solverTypeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use linearSolver.solverType
  LinearSolverType PyGetLinearSolverType() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use consolePrecision
  void PySetOutputPrecision(const Index& consolePrecisionInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use consolePrecision
  Index PyGetOutputPrecision() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "SimulationSettings" << ":\n";
    os << "  linearSolver = " << linearSolver << "\n";
    os << "  parallel = " << parallel << "\n";
    os << "  show = " << show << "\n";
    os << "  solution = " << solution << "\n";
    os << "  staticSolver = " << staticSolver << "\n";
    os << "  timeIntegration = " << timeIntegration << "\n";
    os << "  cleanUpMemory = " << cleanUpMemory << "\n";
    os << "  consolePrecision = " << consolePrecision << "\n";
    os << "  pauseAfterEachStep = " << pauseAfterEachStep << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const SimulationSettings& object)
  {
    object.Print(os);
    return os;
  }

};




//! implementation:

inline void SolutionSettingsDeprecated::PySetAppendToFile(const bool& appendInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.appendToFile", "SimulationSettings parameter solutionSettings.appendToFile is deprecated! use solution.file.append instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.appendToFile is deprecated and forwards to solution.file.append, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.append= (const bool&)appendInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetAppendToFile() const { 
    PyDeprecated("simulationSettings", "solutionSettings.appendToFile", "SimulationSettings parameter solutionSettings.appendToFile is deprecated! use solution.file.append instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.appendToFile is deprecated and forwards to solution.file.append, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.append); 
    }

inline void SolutionSettingsDeprecated::PySetBinarySolutionFile(const bool& binaryInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.binarySolutionFile", "SimulationSettings parameter solutionSettings.binarySolutionFile is deprecated! use solution.file.binary instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.binarySolutionFile is deprecated and forwards to solution.file.binary, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.binary= (const bool&)binaryInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetBinarySolutionFile() const { 
    PyDeprecated("simulationSettings", "solutionSettings.binarySolutionFile", "SimulationSettings parameter solutionSettings.binarySolutionFile is deprecated! use solution.file.binary instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.binarySolutionFile is deprecated and forwards to solution.file.binary, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.binary); 
    }

inline void SolutionSettingsDeprecated::PySetCoordinatesSolutionFileName(const std::string& nameInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.coordinatesSolutionFileName", "SimulationSettings parameter solutionSettings.coordinatesSolutionFileName is deprecated! use solution.file.name instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.coordinatesSolutionFileName is deprecated and forwards to solution.file.name, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.name= (const std::string&)nameInit; 
    }
inline std::string SolutionSettingsDeprecated::PyGetCoordinatesSolutionFileName() const { 
    PyDeprecated("simulationSettings", "solutionSettings.coordinatesSolutionFileName", "SimulationSettings parameter solutionSettings.coordinatesSolutionFileName is deprecated! use solution.file.name instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.coordinatesSolutionFileName is deprecated and forwards to solution.file.name, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::string(backlink->solution.file.name); 
    }

inline void SolutionSettingsDeprecated::PySetExportAccelerations(const bool& accelerationsInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.exportAccelerations", "SimulationSettings parameter solutionSettings.exportAccelerations is deprecated! use solution.file.export.accelerations instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportAccelerations is deprecated and forwards to solution.file.export.accelerations, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.exportSettings.accelerations= (const bool&)accelerationsInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetExportAccelerations() const { 
    PyDeprecated("simulationSettings", "solutionSettings.exportAccelerations", "SimulationSettings parameter solutionSettings.exportAccelerations is deprecated! use solution.file.export.accelerations instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportAccelerations is deprecated and forwards to solution.file.export.accelerations, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.exportSettings.accelerations); 
    }

inline void SolutionSettingsDeprecated::PySetExportAlgebraicCoordinates(const bool& algebraicCoordinatesInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.exportAlgebraicCoordinates", "SimulationSettings parameter solutionSettings.exportAlgebraicCoordinates is deprecated! use solution.file.export.algebraicCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportAlgebraicCoordinates is deprecated and forwards to solution.file.export.algebraicCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.exportSettings.algebraicCoordinates= (const bool&)algebraicCoordinatesInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetExportAlgebraicCoordinates() const { 
    PyDeprecated("simulationSettings", "solutionSettings.exportAlgebraicCoordinates", "SimulationSettings parameter solutionSettings.exportAlgebraicCoordinates is deprecated! use solution.file.export.algebraicCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportAlgebraicCoordinates is deprecated and forwards to solution.file.export.algebraicCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.exportSettings.algebraicCoordinates); 
    }

inline void SolutionSettingsDeprecated::PySetExportDataCoordinates(const bool& dataCoordinatesInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.exportDataCoordinates", "SimulationSettings parameter solutionSettings.exportDataCoordinates is deprecated! use solution.file.export.dataCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportDataCoordinates is deprecated and forwards to solution.file.export.dataCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.exportSettings.dataCoordinates= (const bool&)dataCoordinatesInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetExportDataCoordinates() const { 
    PyDeprecated("simulationSettings", "solutionSettings.exportDataCoordinates", "SimulationSettings parameter solutionSettings.exportDataCoordinates is deprecated! use solution.file.export.dataCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportDataCoordinates is deprecated and forwards to solution.file.export.dataCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.exportSettings.dataCoordinates); 
    }

inline void SolutionSettingsDeprecated::PySetExportODE1Velocities(const bool& ODE1VelocitiesInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.exportODE1Velocities", "SimulationSettings parameter solutionSettings.exportODE1Velocities is deprecated! use solution.file.export.ODE1Velocities instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportODE1Velocities is deprecated and forwards to solution.file.export.ODE1Velocities, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.exportSettings.ODE1Velocities= (const bool&)ODE1VelocitiesInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetExportODE1Velocities() const { 
    PyDeprecated("simulationSettings", "solutionSettings.exportODE1Velocities", "SimulationSettings parameter solutionSettings.exportODE1Velocities is deprecated! use solution.file.export.ODE1Velocities instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportODE1Velocities is deprecated and forwards to solution.file.export.ODE1Velocities, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.exportSettings.ODE1Velocities); 
    }

inline void SolutionSettingsDeprecated::PySetExportVelocities(const bool& velocitiesInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.exportVelocities", "SimulationSettings parameter solutionSettings.exportVelocities is deprecated! use solution.file.export.velocities instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportVelocities is deprecated and forwards to solution.file.export.velocities, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.exportSettings.velocities= (const bool&)velocitiesInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetExportVelocities() const { 
    PyDeprecated("simulationSettings", "solutionSettings.exportVelocities", "SimulationSettings parameter solutionSettings.exportVelocities is deprecated! use solution.file.export.velocities instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.exportVelocities is deprecated and forwards to solution.file.export.velocities, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.exportSettings.velocities); 
    }

inline void SolutionSettingsDeprecated::PySetFlushFilesDOF(const Index& flushAboveCoordinatesInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.flushFilesDOF", "SimulationSettings parameter solutionSettings.flushFilesDOF is deprecated! use solution.file.flushAboveCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.flushFilesDOF is deprecated and forwards to solution.file.flushAboveCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.flushAboveCoordinates= (const Index&)flushAboveCoordinatesInit; 
    }
inline Index SolutionSettingsDeprecated::PyGetFlushFilesDOF() const { 
    PyDeprecated("simulationSettings", "solutionSettings.flushFilesDOF", "SimulationSettings parameter solutionSettings.flushFilesDOF is deprecated! use solution.file.flushAboveCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.flushFilesDOF is deprecated and forwards to solution.file.flushAboveCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->solution.file.flushAboveCoordinates); 
    }

inline void SolutionSettingsDeprecated::PySetFlushFilesImmediately(const bool& flushFilesImmediatelyInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.flushFilesImmediately", "SimulationSettings parameter solutionSettings.flushFilesImmediately is deprecated! use solution.flushFilesImmediately instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.flushFilesImmediately is deprecated and forwards to solution.flushFilesImmediately, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.flushFilesImmediately= (const bool&)flushFilesImmediatelyInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetFlushFilesImmediately() const { 
    PyDeprecated("simulationSettings", "solutionSettings.flushFilesImmediately", "SimulationSettings parameter solutionSettings.flushFilesImmediately is deprecated! use solution.flushFilesImmediately instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.flushFilesImmediately is deprecated and forwards to solution.flushFilesImmediately, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.flushFilesImmediately); 
    }

inline void SolutionSettingsDeprecated::PySetOutputPrecision(const Index& precisionInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.outputPrecision", "SimulationSettings parameter solutionSettings.outputPrecision is deprecated! use solution.precision instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.outputPrecision is deprecated and forwards to solution.precision, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.precision= (const Index&)precisionInit; 
    }
inline Index SolutionSettingsDeprecated::PyGetOutputPrecision() const { 
    PyDeprecated("simulationSettings", "solutionSettings.outputPrecision", "SimulationSettings parameter solutionSettings.outputPrecision is deprecated! use solution.precision instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.outputPrecision is deprecated and forwards to solution.precision, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->solution.precision); 
    }

inline void SolutionSettingsDeprecated::PySetRecordImagesInterval(const Real& recordImagesIntervalInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.recordImagesInterval", "SimulationSettings parameter solutionSettings.recordImagesInterval is deprecated! use solution.recordImagesInterval instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.recordImagesInterval is deprecated and forwards to solution.recordImagesInterval, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.recordImagesInterval= (const Real&)recordImagesIntervalInit; 
    }
inline Real SolutionSettingsDeprecated::PyGetRecordImagesInterval() const { 
    PyDeprecated("simulationSettings", "solutionSettings.recordImagesInterval", "SimulationSettings parameter solutionSettings.recordImagesInterval is deprecated! use solution.recordImagesInterval instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.recordImagesInterval is deprecated and forwards to solution.recordImagesInterval, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Real(backlink->solution.recordImagesInterval); 
    }

inline void SolutionSettingsDeprecated::PySetRestartFileName(const std::string& nameInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.restartFileName", "SimulationSettings parameter solutionSettings.restartFileName is deprecated! use solution.restart.name instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.restartFileName is deprecated and forwards to solution.restart.name, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.restart.name= (const std::string&)nameInit; 
    }
inline std::string SolutionSettingsDeprecated::PyGetRestartFileName() const { 
    PyDeprecated("simulationSettings", "solutionSettings.restartFileName", "SimulationSettings parameter solutionSettings.restartFileName is deprecated! use solution.restart.name instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.restartFileName is deprecated and forwards to solution.restart.name, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::string(backlink->solution.restart.name); 
    }

inline void SolutionSettingsDeprecated::PySetRestartWritePeriod(const Real& writePeriodInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.restartWritePeriod", "SimulationSettings parameter solutionSettings.restartWritePeriod is deprecated! use solution.restart.writePeriod instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.restartWritePeriod is deprecated and forwards to solution.restart.writePeriod, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.restart.writePeriod= (const Real&)writePeriodInit; 
    }
inline Real SolutionSettingsDeprecated::PyGetRestartWritePeriod() const { 
    PyDeprecated("simulationSettings", "solutionSettings.restartWritePeriod", "SimulationSettings parameter solutionSettings.restartWritePeriod is deprecated! use solution.restart.writePeriod instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.restartWritePeriod is deprecated and forwards to solution.restart.writePeriod, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Real(backlink->solution.restart.writePeriod); 
    }

inline void SolutionSettingsDeprecated::PySetSensorsAppendToFile(const bool& appendInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsAppendToFile", "SimulationSettings parameter solutionSettings.sensorsAppendToFile is deprecated! use solution.sensors.append instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsAppendToFile is deprecated and forwards to solution.sensors.append, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.sensors.append= (const bool&)appendInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetSensorsAppendToFile() const { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsAppendToFile", "SimulationSettings parameter solutionSettings.sensorsAppendToFile is deprecated! use solution.sensors.append instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsAppendToFile is deprecated and forwards to solution.sensors.append, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.sensors.append); 
    }

inline void SolutionSettingsDeprecated::PySetSensorsStoreAndWriteFiles(const bool& activeInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsStoreAndWriteFiles", "SimulationSettings parameter solutionSettings.sensorsStoreAndWriteFiles is deprecated! use solution.sensors.active instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsStoreAndWriteFiles is deprecated and forwards to solution.sensors.active, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.sensors.active= (const bool&)activeInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetSensorsStoreAndWriteFiles() const { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsStoreAndWriteFiles", "SimulationSettings parameter solutionSettings.sensorsStoreAndWriteFiles is deprecated! use solution.sensors.active instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsStoreAndWriteFiles is deprecated and forwards to solution.sensors.active, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.sensors.active); 
    }

inline void SolutionSettingsDeprecated::PySetSensorsWriteFileFooter(const bool& writeFooterInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsWriteFileFooter", "SimulationSettings parameter solutionSettings.sensorsWriteFileFooter is deprecated! use solution.sensors.writeFooter instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsWriteFileFooter is deprecated and forwards to solution.sensors.writeFooter, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.sensors.writeFooter= (const bool&)writeFooterInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetSensorsWriteFileFooter() const { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsWriteFileFooter", "SimulationSettings parameter solutionSettings.sensorsWriteFileFooter is deprecated! use solution.sensors.writeFooter instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsWriteFileFooter is deprecated and forwards to solution.sensors.writeFooter, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.sensors.writeFooter); 
    }

inline void SolutionSettingsDeprecated::PySetSensorsWriteFileHeader(const bool& writeHeaderInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsWriteFileHeader", "SimulationSettings parameter solutionSettings.sensorsWriteFileHeader is deprecated! use solution.sensors.writeHeader instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsWriteFileHeader is deprecated and forwards to solution.sensors.writeHeader, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.sensors.writeHeader= (const bool&)writeHeaderInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetSensorsWriteFileHeader() const { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsWriteFileHeader", "SimulationSettings parameter solutionSettings.sensorsWriteFileHeader is deprecated! use solution.sensors.writeHeader instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsWriteFileHeader is deprecated and forwards to solution.sensors.writeHeader, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.sensors.writeHeader); 
    }

inline void SolutionSettingsDeprecated::PySetSensorsWritePeriod(const Real& writePeriodInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsWritePeriod", "SimulationSettings parameter solutionSettings.sensorsWritePeriod is deprecated! use solution.sensors.writePeriod instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsWritePeriod is deprecated and forwards to solution.sensors.writePeriod, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.sensors.writePeriod= (const Real&)writePeriodInit; 
    }
inline Real SolutionSettingsDeprecated::PyGetSensorsWritePeriod() const { 
    PyDeprecated("simulationSettings", "solutionSettings.sensorsWritePeriod", "SimulationSettings parameter solutionSettings.sensorsWritePeriod is deprecated! use solution.sensors.writePeriod instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.sensorsWritePeriod is deprecated and forwards to solution.sensors.writePeriod, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Real(backlink->solution.sensors.writePeriod); 
    }

inline void SolutionSettingsDeprecated::PySetSolutionInformation(const std::string& informationInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.solutionInformation", "SimulationSettings parameter solutionSettings.solutionInformation is deprecated! use solution.file.information instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.solutionInformation is deprecated and forwards to solution.file.information, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.information= (const std::string&)informationInit; 
    }
inline std::string SolutionSettingsDeprecated::PyGetSolutionInformation() const { 
    PyDeprecated("simulationSettings", "solutionSettings.solutionInformation", "SimulationSettings parameter solutionSettings.solutionInformation is deprecated! use solution.file.information instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.solutionInformation is deprecated and forwards to solution.file.information, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::string(backlink->solution.file.information); 
    }

inline void SolutionSettingsDeprecated::PySetSolutionWritePeriod(const Real& writePeriodInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.solutionWritePeriod", "SimulationSettings parameter solutionSettings.solutionWritePeriod is deprecated! use solution.file.writePeriod instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.solutionWritePeriod is deprecated and forwards to solution.file.writePeriod, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.writePeriod= (const Real&)writePeriodInit; 
    }
inline Real SolutionSettingsDeprecated::PyGetSolutionWritePeriod() const { 
    PyDeprecated("simulationSettings", "solutionSettings.solutionWritePeriod", "SimulationSettings parameter solutionSettings.solutionWritePeriod is deprecated! use solution.file.writePeriod instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.solutionWritePeriod is deprecated and forwards to solution.file.writePeriod, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Real(backlink->solution.file.writePeriod); 
    }

inline void SolutionSettingsDeprecated::PySetSolverInformationFileName(const std::string& solverInformationFileNameInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.solverInformationFileName", "SimulationSettings parameter solutionSettings.solverInformationFileName is deprecated! use solution.solverInformationFileName instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.solverInformationFileName is deprecated and forwards to solution.solverInformationFileName, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.solverInformationFileName= (const std::string&)solverInformationFileNameInit; 
    }
inline std::string SolutionSettingsDeprecated::PyGetSolverInformationFileName() const { 
    PyDeprecated("simulationSettings", "solutionSettings.solverInformationFileName", "SimulationSettings parameter solutionSettings.solverInformationFileName is deprecated! use solution.solverInformationFileName instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.solverInformationFileName is deprecated and forwards to solution.solverInformationFileName, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::string(backlink->solution.solverInformationFileName); 
    }

inline void SolutionSettingsDeprecated::PySetWriteFileFooter(const bool& writeFooterInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.writeFileFooter", "SimulationSettings parameter solutionSettings.writeFileFooter is deprecated! use solution.file.writeFooter instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeFileFooter is deprecated and forwards to solution.file.writeFooter, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.writeFooter= (const bool&)writeFooterInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetWriteFileFooter() const { 
    PyDeprecated("simulationSettings", "solutionSettings.writeFileFooter", "SimulationSettings parameter solutionSettings.writeFileFooter is deprecated! use solution.file.writeFooter instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeFileFooter is deprecated and forwards to solution.file.writeFooter, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.writeFooter); 
    }

inline void SolutionSettingsDeprecated::PySetWriteFileHeader(const bool& writeHeaderInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.writeFileHeader", "SimulationSettings parameter solutionSettings.writeFileHeader is deprecated! use solution.file.writeHeader instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeFileHeader is deprecated and forwards to solution.file.writeHeader, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.writeHeader= (const bool&)writeHeaderInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetWriteFileHeader() const { 
    PyDeprecated("simulationSettings", "solutionSettings.writeFileHeader", "SimulationSettings parameter solutionSettings.writeFileHeader is deprecated! use solution.file.writeHeader instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeFileHeader is deprecated and forwards to solution.file.writeHeader, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.writeHeader); 
    }

inline void SolutionSettingsDeprecated::PySetWriteInitialValues(const bool& writeInitialValuesInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.writeInitialValues", "SimulationSettings parameter solutionSettings.writeInitialValues is deprecated! use solution.file.writeInitialValues instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeInitialValues is deprecated and forwards to solution.file.writeInitialValues, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.writeInitialValues= (const bool&)writeInitialValuesInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetWriteInitialValues() const { 
    PyDeprecated("simulationSettings", "solutionSettings.writeInitialValues", "SimulationSettings parameter solutionSettings.writeInitialValues is deprecated! use solution.file.writeInitialValues instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeInitialValues is deprecated and forwards to solution.file.writeInitialValues, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.writeInitialValues); 
    }

inline void SolutionSettingsDeprecated::PySetWriteRestartFile(const bool& writeInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.writeRestartFile", "SimulationSettings parameter solutionSettings.writeRestartFile is deprecated! use solution.restart.write instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeRestartFile is deprecated and forwards to solution.restart.write, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.restart.write= (const bool&)writeInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetWriteRestartFile() const { 
    PyDeprecated("simulationSettings", "solutionSettings.writeRestartFile", "SimulationSettings parameter solutionSettings.writeRestartFile is deprecated! use solution.restart.write instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeRestartFile is deprecated and forwards to solution.restart.write, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.restart.write); 
    }

inline void SolutionSettingsDeprecated::PySetWriteSolutionToFile(const bool& writeInit) { 
    PyDeprecated("simulationSettings", "solutionSettings.writeSolutionToFile", "SimulationSettings parameter solutionSettings.writeSolutionToFile is deprecated! use solution.file.write instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeSolutionToFile is deprecated and forwards to solution.file.write, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->solution.file.write= (const bool&)writeInit; 
    }
inline bool SolutionSettingsDeprecated::PyGetWriteSolutionToFile() const { 
    PyDeprecated("simulationSettings", "solutionSettings.writeSolutionToFile", "SimulationSettings parameter solutionSettings.writeSolutionToFile is deprecated! use solution.file.write instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("solutionSettings.writeSolutionToFile is deprecated and forwards to solution.file.write, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->solution.file.write); 
    }

inline void NumericalDifferentiationSettings::PySetForODE2connectors(const bool& forODE2ConnectorsInit) { 
    PyDeprecated("simulationSettings", "numericalDifferentiation.forODE2connectors", "SimulationSettings parameter numericalDifferentiation.forODE2connectors is deprecated! use numericalDifferentiation.forODE2Connectors instead!");
    forODE2Connectors= (const bool&)forODE2ConnectorsInit; 
    }
inline bool NumericalDifferentiationSettings::PyGetForODE2connectors() const { 
    PyDeprecated("simulationSettings", "numericalDifferentiation.forODE2connectors", "SimulationSettings parameter numericalDifferentiation.forODE2connectors is deprecated! use numericalDifferentiation.forODE2Connectors instead!");
    return bool(forODE2Connectors); 
    }

inline void NewtonSettings::PySetNewtonResidualMode(const Index& residualModeInit) { 
    PyDeprecated("simulationSettings", "newton.newtonResidualMode", "SimulationSettings parameter newton.newtonResidualMode is deprecated! use newton.residualMode instead!");
    residualMode= (const Index&)residualModeInit; 
    }
inline Index NewtonSettings::PyGetNewtonResidualMode() const { 
    PyDeprecated("simulationSettings", "newton.newtonResidualMode", "SimulationSettings parameter newton.newtonResidualMode is deprecated! use newton.residualMode instead!");
    return Index(residualMode); 
    }

inline void NewtonSettings::PySetUseNewtonSolver(const bool& activeInit) { 
    PyDeprecated("simulationSettings", "newton.useNewtonSolver", "SimulationSettings parameter newton.useNewtonSolver is deprecated! use newton.active instead!");
    active= (const bool&)activeInit; 
    }
inline bool NewtonSettings::PyGetUseNewtonSolver() const { 
    PyDeprecated("simulationSettings", "newton.useNewtonSolver", "SimulationSettings parameter newton.useNewtonSolver is deprecated! use newton.active instead!");
    return bool(active); 
    }

inline void ExplicitIntegrationSettingsDeprecated::PySetComputeEndOfStepAccelerations(const bool& computeEndOfStepAccelerationsInit) { 
    PyDeprecated("simulationSettings", "explicitIntegration.computeEndOfStepAccelerations", "SimulationSettings parameter explicitIntegration.computeEndOfStepAccelerations is deprecated! use timeIntegration.explicit.computeEndOfStepAccelerations instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.computeEndOfStepAccelerations is deprecated and forwards to timeIntegration.explicit.computeEndOfStepAccelerations, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.explicitSettings.computeEndOfStepAccelerations= (const bool&)computeEndOfStepAccelerationsInit; 
    }
inline bool ExplicitIntegrationSettingsDeprecated::PyGetComputeEndOfStepAccelerations() const { 
    PyDeprecated("simulationSettings", "explicitIntegration.computeEndOfStepAccelerations", "SimulationSettings parameter explicitIntegration.computeEndOfStepAccelerations is deprecated! use timeIntegration.explicit.computeEndOfStepAccelerations instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.computeEndOfStepAccelerations is deprecated and forwards to timeIntegration.explicit.computeEndOfStepAccelerations, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->timeIntegration.explicitSettings.computeEndOfStepAccelerations); 
    }

inline void ExplicitIntegrationSettingsDeprecated::PySetComputeMassMatrixInversePerBody(const bool& computeMassMatrixInversePerBodyInit) { 
    PyDeprecated("simulationSettings", "explicitIntegration.computeMassMatrixInversePerBody", "SimulationSettings parameter explicitIntegration.computeMassMatrixInversePerBody is deprecated! use timeIntegration.explicit.computeMassMatrixInversePerBody instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.computeMassMatrixInversePerBody is deprecated and forwards to timeIntegration.explicit.computeMassMatrixInversePerBody, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.explicitSettings.computeMassMatrixInversePerBody= (const bool&)computeMassMatrixInversePerBodyInit; 
    }
inline bool ExplicitIntegrationSettingsDeprecated::PyGetComputeMassMatrixInversePerBody() const { 
    PyDeprecated("simulationSettings", "explicitIntegration.computeMassMatrixInversePerBody", "SimulationSettings parameter explicitIntegration.computeMassMatrixInversePerBody is deprecated! use timeIntegration.explicit.computeMassMatrixInversePerBody instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.computeMassMatrixInversePerBody is deprecated and forwards to timeIntegration.explicit.computeMassMatrixInversePerBody, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->timeIntegration.explicitSettings.computeMassMatrixInversePerBody); 
    }

inline void ExplicitIntegrationSettingsDeprecated::PySetDynamicSolverType(const DynamicSolverType& solverTypeInit) { 
    PyDeprecated("simulationSettings", "explicitIntegration.dynamicSolverType", "SimulationSettings parameter explicitIntegration.dynamicSolverType is deprecated! use timeIntegration.solverType instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.dynamicSolverType is deprecated and forwards to timeIntegration.solverType, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.solverType= (const DynamicSolverType&)solverTypeInit; 
    }
inline DynamicSolverType ExplicitIntegrationSettingsDeprecated::PyGetDynamicSolverType() const { 
    PyDeprecated("simulationSettings", "explicitIntegration.dynamicSolverType", "SimulationSettings parameter explicitIntegration.dynamicSolverType is deprecated! use timeIntegration.solverType instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.dynamicSolverType is deprecated and forwards to timeIntegration.solverType, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return DynamicSolverType(backlink->timeIntegration.solverType); 
    }

inline void ExplicitIntegrationSettingsDeprecated::PySetEliminateConstraints(const bool& eliminateConstraintsInit) { 
    PyDeprecated("simulationSettings", "explicitIntegration.eliminateConstraints", "SimulationSettings parameter explicitIntegration.eliminateConstraints is deprecated! use timeIntegration.explicit.eliminateConstraints instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.eliminateConstraints is deprecated and forwards to timeIntegration.explicit.eliminateConstraints, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.explicitSettings.eliminateConstraints= (const bool&)eliminateConstraintsInit; 
    }
inline bool ExplicitIntegrationSettingsDeprecated::PyGetEliminateConstraints() const { 
    PyDeprecated("simulationSettings", "explicitIntegration.eliminateConstraints", "SimulationSettings parameter explicitIntegration.eliminateConstraints is deprecated! use timeIntegration.explicit.eliminateConstraints instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.eliminateConstraints is deprecated and forwards to timeIntegration.explicit.eliminateConstraints, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->timeIntegration.explicitSettings.eliminateConstraints); 
    }

inline void ExplicitIntegrationSettingsDeprecated::PySetUseLieGroupIntegration(const bool& useLieGroupIntegrationInit) { 
    PyDeprecated("simulationSettings", "explicitIntegration.useLieGroupIntegration", "SimulationSettings parameter explicitIntegration.useLieGroupIntegration is deprecated! use timeIntegration.explicit.useLieGroupIntegration instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.useLieGroupIntegration is deprecated and forwards to timeIntegration.explicit.useLieGroupIntegration, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.explicitSettings.useLieGroupIntegration= (const bool&)useLieGroupIntegrationInit; 
    }
inline bool ExplicitIntegrationSettingsDeprecated::PyGetUseLieGroupIntegration() const { 
    PyDeprecated("simulationSettings", "explicitIntegration.useLieGroupIntegration", "SimulationSettings parameter explicitIntegration.useLieGroupIntegration is deprecated! use timeIntegration.explicit.useLieGroupIntegration instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("explicitIntegration.useLieGroupIntegration is deprecated and forwards to timeIntegration.explicit.useLieGroupIntegration, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->timeIntegration.explicitSettings.useLieGroupIntegration); 
    }

inline void TimeIntegrationSettings::PySetRealtimeFactor(const Real& factorInit) { 
    PyDeprecated("simulationSettings", "timeIntegration.realtimeFactor", "SimulationSettings parameter timeIntegration.realtimeFactor is deprecated! use timeIntegration.realtime.factor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("timeIntegration.realtimeFactor is deprecated and forwards to timeIntegration.realtime.factor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.realtime.factor= (const Real&)factorInit; 
    }
inline Real TimeIntegrationSettings::PyGetRealtimeFactor() const { 
    PyDeprecated("simulationSettings", "timeIntegration.realtimeFactor", "SimulationSettings parameter timeIntegration.realtimeFactor is deprecated! use timeIntegration.realtime.factor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("timeIntegration.realtimeFactor is deprecated and forwards to timeIntegration.realtime.factor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Real(backlink->timeIntegration.realtime.factor); 
    }

inline void TimeIntegrationSettings::PySetRealtimeWaitMicroseconds(const Index& waitMicrosecondsInit) { 
    PyDeprecated("simulationSettings", "timeIntegration.realtimeWaitMicroseconds", "SimulationSettings parameter timeIntegration.realtimeWaitMicroseconds is deprecated! use timeIntegration.realtime.waitMicroseconds instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("timeIntegration.realtimeWaitMicroseconds is deprecated and forwards to timeIntegration.realtime.waitMicroseconds, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.realtime.waitMicroseconds= (const Index&)waitMicrosecondsInit; 
    }
inline Index TimeIntegrationSettings::PyGetRealtimeWaitMicroseconds() const { 
    PyDeprecated("simulationSettings", "timeIntegration.realtimeWaitMicroseconds", "SimulationSettings parameter timeIntegration.realtimeWaitMicroseconds is deprecated! use timeIntegration.realtime.waitMicroseconds instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("timeIntegration.realtimeWaitMicroseconds is deprecated and forwards to timeIntegration.realtime.waitMicroseconds, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->timeIntegration.realtime.waitMicroseconds); 
    }

inline void TimeIntegrationSettings::PySetSimulateInRealtime(const bool& activeInit) { 
    PyDeprecated("simulationSettings", "timeIntegration.simulateInRealtime", "SimulationSettings parameter timeIntegration.simulateInRealtime is deprecated! use timeIntegration.realtime.active instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("timeIntegration.simulateInRealtime is deprecated and forwards to timeIntegration.realtime.active, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->timeIntegration.realtime.active= (const bool&)activeInit; 
    }
inline bool TimeIntegrationSettings::PyGetSimulateInRealtime() const { 
    PyDeprecated("simulationSettings", "timeIntegration.simulateInRealtime", "SimulationSettings parameter timeIntegration.simulateInRealtime is deprecated! use timeIntegration.realtime.active instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("timeIntegration.simulateInRealtime is deprecated and forwards to timeIntegration.realtime.active, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->timeIntegration.realtime.active); 
    }

inline void StaticSolverSettings::PySetConstrainODE1coordinates(const bool& constrainODE1CoordinatesInit) { 
    PyDeprecated("simulationSettings", "staticSolver.constrainODE1coordinates", "SimulationSettings parameter staticSolver.constrainODE1coordinates is deprecated! use staticSolver.constrainODE1Coordinates instead!");
    constrainODE1Coordinates= (const bool&)constrainODE1CoordinatesInit; 
    }
inline bool StaticSolverSettings::PyGetConstrainODE1coordinates() const { 
    PyDeprecated("simulationSettings", "staticSolver.constrainODE1coordinates", "SimulationSettings parameter staticSolver.constrainODE1coordinates is deprecated! use staticSolver.constrainODE1Coordinates instead!");
    return bool(constrainODE1Coordinates); 
    }

inline void LinearSolverSettingsDeprecated::PySetIgnoreSingularJacobian(const bool& ignoreSingularJacobianInit) { 
    PyDeprecated("simulationSettings", "linearSolverSettings.ignoreSingularJacobian", "SimulationSettings parameter linearSolverSettings.ignoreSingularJacobian is deprecated! use linearSolver.ignoreSingularJacobian instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.ignoreSingularJacobian is deprecated and forwards to linearSolver.ignoreSingularJacobian, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->linearSolver.ignoreSingularJacobian= (const bool&)ignoreSingularJacobianInit; 
    }
inline bool LinearSolverSettingsDeprecated::PyGetIgnoreSingularJacobian() const { 
    PyDeprecated("simulationSettings", "linearSolverSettings.ignoreSingularJacobian", "SimulationSettings parameter linearSolverSettings.ignoreSingularJacobian is deprecated! use linearSolver.ignoreSingularJacobian instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.ignoreSingularJacobian is deprecated and forwards to linearSolver.ignoreSingularJacobian, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->linearSolver.ignoreSingularJacobian); 
    }

inline void LinearSolverSettingsDeprecated::PySetPivotThreshold(const Real& pivotThresholdInit) { 
    PyDeprecated("simulationSettings", "linearSolverSettings.pivotThreshold", "SimulationSettings parameter linearSolverSettings.pivotThreshold is deprecated! use linearSolver.pivotThreshold instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.pivotThreshold is deprecated and forwards to linearSolver.pivotThreshold, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->linearSolver.pivotThreshold= (const Real&)pivotThresholdInit; 
    }
inline Real LinearSolverSettingsDeprecated::PyGetPivotThreshold() const { 
    PyDeprecated("simulationSettings", "linearSolverSettings.pivotThreshold", "SimulationSettings parameter linearSolverSettings.pivotThreshold is deprecated! use linearSolver.pivotThreshold instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.pivotThreshold is deprecated and forwards to linearSolver.pivotThreshold, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Real(backlink->linearSolver.pivotThreshold); 
    }

inline void LinearSolverSettingsDeprecated::PySetReuseAnalyzedPattern(const bool& reuseAnalyzedPatternInit) { 
    PyDeprecated("simulationSettings", "linearSolverSettings.reuseAnalyzedPattern", "SimulationSettings parameter linearSolverSettings.reuseAnalyzedPattern is deprecated! use linearSolver.reuseAnalyzedPattern instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.reuseAnalyzedPattern is deprecated and forwards to linearSolver.reuseAnalyzedPattern, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->linearSolver.reuseAnalyzedPattern= (const bool&)reuseAnalyzedPatternInit; 
    }
inline bool LinearSolverSettingsDeprecated::PyGetReuseAnalyzedPattern() const { 
    PyDeprecated("simulationSettings", "linearSolverSettings.reuseAnalyzedPattern", "SimulationSettings parameter linearSolverSettings.reuseAnalyzedPattern is deprecated! use linearSolver.reuseAnalyzedPattern instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.reuseAnalyzedPattern is deprecated and forwards to linearSolver.reuseAnalyzedPattern, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->linearSolver.reuseAnalyzedPattern); 
    }

inline void LinearSolverSettingsDeprecated::PySetShowCausingItems(const bool& showCausingItemsInit) { 
    PyDeprecated("simulationSettings", "linearSolverSettings.showCausingItems", "SimulationSettings parameter linearSolverSettings.showCausingItems is deprecated! use linearSolver.showCausingItems instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.showCausingItems is deprecated and forwards to linearSolver.showCausingItems, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->linearSolver.showCausingItems= (const bool&)showCausingItemsInit; 
    }
inline bool LinearSolverSettingsDeprecated::PyGetShowCausingItems() const { 
    PyDeprecated("simulationSettings", "linearSolverSettings.showCausingItems", "SimulationSettings parameter linearSolverSettings.showCausingItems is deprecated! use linearSolver.showCausingItems instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("linearSolverSettings.showCausingItems is deprecated and forwards to linearSolver.showCausingItems, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->linearSolver.showCausingItems); 
    }

inline void Parallel::PySetMultithreadedLLimitJacobians(const Index& multithreadedLowerLimitJacobiansInit) { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitJacobians", "SimulationSettings parameter parallel.multithreadedLLimitJacobians is deprecated! use parallel.multithreadedLowerLimitJacobians instead!");
    multithreadedLowerLimitJacobians= (const Index&)multithreadedLowerLimitJacobiansInit; 
    }
inline Index Parallel::PyGetMultithreadedLLimitJacobians() const { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitJacobians", "SimulationSettings parameter parallel.multithreadedLLimitJacobians is deprecated! use parallel.multithreadedLowerLimitJacobians instead!");
    return Index(multithreadedLowerLimitJacobians); 
    }

inline void Parallel::PySetMultithreadedLLimitLoads(const Index& multithreadedLowerLimitLoadsInit) { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitLoads", "SimulationSettings parameter parallel.multithreadedLLimitLoads is deprecated! use parallel.multithreadedLowerLimitLoads instead!");
    multithreadedLowerLimitLoads= (const Index&)multithreadedLowerLimitLoadsInit; 
    }
inline Index Parallel::PyGetMultithreadedLLimitLoads() const { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitLoads", "SimulationSettings parameter parallel.multithreadedLLimitLoads is deprecated! use parallel.multithreadedLowerLimitLoads instead!");
    return Index(multithreadedLowerLimitLoads); 
    }

inline void Parallel::PySetMultithreadedLLimitMassMatrices(const Index& multithreadedLowerLimitMassMatricesInit) { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitMassMatrices", "SimulationSettings parameter parallel.multithreadedLLimitMassMatrices is deprecated! use parallel.multithreadedLowerLimitMassMatrices instead!");
    multithreadedLowerLimitMassMatrices= (const Index&)multithreadedLowerLimitMassMatricesInit; 
    }
inline Index Parallel::PyGetMultithreadedLLimitMassMatrices() const { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitMassMatrices", "SimulationSettings parameter parallel.multithreadedLLimitMassMatrices is deprecated! use parallel.multithreadedLowerLimitMassMatrices instead!");
    return Index(multithreadedLowerLimitMassMatrices); 
    }

inline void Parallel::PySetMultithreadedLLimitResiduals(const Index& multithreadedLowerLimitResidualsInit) { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitResiduals", "SimulationSettings parameter parallel.multithreadedLLimitResiduals is deprecated! use parallel.multithreadedLowerLimitResiduals instead!");
    multithreadedLowerLimitResiduals= (const Index&)multithreadedLowerLimitResidualsInit; 
    }
inline Index Parallel::PyGetMultithreadedLLimitResiduals() const { 
    PyDeprecated("simulationSettings", "parallel.multithreadedLLimitResiduals", "SimulationSettings parameter parallel.multithreadedLLimitResiduals is deprecated! use parallel.multithreadedLowerLimitResiduals instead!");
    return Index(multithreadedLowerLimitResiduals); 
    }

inline void SimulationSettings::PySetDisplayComputationTime(const bool& computationTimeInit) { 
    PyDeprecated("simulationSettings", "displayComputationTime", "SimulationSettings parameter displayComputationTime is deprecated! use show.computationTime instead!");
    show.computationTime= (const bool&)computationTimeInit; 
    }
inline bool SimulationSettings::PyGetDisplayComputationTime() const { 
    PyDeprecated("simulationSettings", "displayComputationTime", "SimulationSettings parameter displayComputationTime is deprecated! use show.computationTime instead!");
    return bool(show.computationTime); 
    }

inline void SimulationSettings::PySetDisplayGlobalTimers(const bool& globalTimersInit) { 
    PyDeprecated("simulationSettings", "displayGlobalTimers", "SimulationSettings parameter displayGlobalTimers is deprecated! use show.globalTimers instead!");
    show.globalTimers= (const bool&)globalTimersInit; 
    }
inline bool SimulationSettings::PyGetDisplayGlobalTimers() const { 
    PyDeprecated("simulationSettings", "displayGlobalTimers", "SimulationSettings parameter displayGlobalTimers is deprecated! use show.globalTimers instead!");
    return bool(show.globalTimers); 
    }

inline void SimulationSettings::PySetDisplayStatistics(const bool& statisticsInit) { 
    PyDeprecated("simulationSettings", "displayStatistics", "SimulationSettings parameter displayStatistics is deprecated! use show.statistics instead!");
    show.statistics= (const bool&)statisticsInit; 
    }
inline bool SimulationSettings::PyGetDisplayStatistics() const { 
    PyDeprecated("simulationSettings", "displayStatistics", "SimulationSettings parameter displayStatistics is deprecated! use show.statistics instead!");
    return bool(show.statistics); 
    }

inline void SimulationSettings::PySetLinearSolverType(const LinearSolverType& solverTypeInit) { 
    PyDeprecated("simulationSettings", "linearSolverType", "SimulationSettings parameter linearSolverType is deprecated! use linearSolver.solverType instead!");
    linearSolver.solverType= (const LinearSolverType&)solverTypeInit; 
    }
inline LinearSolverType SimulationSettings::PyGetLinearSolverType() const { 
    PyDeprecated("simulationSettings", "linearSolverType", "SimulationSettings parameter linearSolverType is deprecated! use linearSolver.solverType instead!");
    return LinearSolverType(linearSolver.solverType); 
    }

inline void SimulationSettings::PySetOutputPrecision(const Index& consolePrecisionInit) { 
    PyDeprecated("simulationSettings", "outputPrecision", "SimulationSettings parameter outputPrecision is deprecated! use simulationSettings.consolePrecision instead!");
    consolePrecision= (const Index&)consolePrecisionInit; 
    }
inline Index SimulationSettings::PyGetOutputPrecision() const { 
    PyDeprecated("simulationSettings", "outputPrecision", "SimulationSettings parameter outputPrecision is deprecated! use simulationSettings.consolePrecision instead!");
    return Index(consolePrecision); 
    }

#endif //#ifdef include once...
