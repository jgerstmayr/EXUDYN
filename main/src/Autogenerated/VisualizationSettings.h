/** ***********************************************************************************************
* @class        VSettingsGeneral
* @brief        General settings for visualization.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/

#ifndef VISUALIZATIONSETTINGS__H
#define VISUALIZATIONSETTINGS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class VSettingsGeneral // AUTO: 
{
public: // AUTO: 
  float graphicsUpdateInterval;                   //!< AUTO: interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
  bool autoFitScene;                              //!< AUTO: automatically fit scene within first second after StartRenderer()
  float textSize;                                 //!< AUTO: general text size (font size) in pixels if not overwritten; if useWindowsMonitorScaleFactor=True, the the textSize is multplied with the windows monitor scaling factor for larger texts on on high resolution monitors; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
  Float4 textColor;                               //!< AUTO: general text color (default); used for system texts in render window
  bool useWindowsMonitorScaleFactor;              //!< AUTO: the windows monitor scaling is used for increased visibility of texts on high resolution monitors; based on GLFW glfwGetWindowContentScale
  bool useBitmapText;                             //!< AUTO: if true, texts are displayed using pre-defined bitmaps for the text; may increase the complexity of your scene, e.g., if many (>10000) node numbers shown
  float minSceneSize;                             //!< AUTO: minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
  Float4 backgroundColor;                         //!< AUTO: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  Float4 backgroundColorBottom;                   //!< AUTO: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  bool useGradientBackground;                     //!< AUTO: true = use vertical gradient for background; 
  float coordinateSystemSize;                     //!< AUTO: size of coordinate system relative to font size
  bool drawCoordinateSystem;                      //!< AUTO: false = no coordinate system shown
  bool drawWorldBasis;                            //!< AUTO: true = draw world basis coordinate system at (0,0,0)
  float worldBasisSize;                           //!< AUTO: size of world basis coordinate system
  Index showHelpOnStartup;                        //!< AUTO: seconds to show help message on startup (0=deactivate)
  bool showComputationInfo;                       //!< AUTO: true = show (hide) all computation information including EXUDYN and version
  bool showSolutionInformation;                   //!< AUTO: true = show solution information (from simulationSettings.solution)
  bool showSolverInformation;                     //!< AUTO: true = solver name and further information shown in render window
  bool showSolverTime;                            //!< AUTO: true = solver current time shown in render window
  std::string renderWindowString;                 //!< AUTO: string shown in render window (use this, e.g., for debugging, etc.; written below EXUDYN, similar to solutionInformation in SimulationSettings.solutionSettings)
  float pointSize;                                //!< AUTO: global point size (absolute)
  Index circleTiling;                             //!< AUTO: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  Index cylinderTiling;                           //!< AUTO: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  Index sphereTiling;                             //!< AUTO: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  Index axesTiling;                               //!< AUTO: global number of segments for drawing axes cylinders and cones (reduce this number, e.g. to 4, if many axes are drawn)
  bool threadSafeGraphicsUpdate;                  //!< AUTO: true = updating of visualization is threadsafe, but slower for complicated models; deactivate this to speed up computation, but activate for generation of animations; may be improved in future by adding a safe visualizationUpdate state
  bool useMultiThreadedRendering;                 //!< AUTO: true = rendering is done in separate thread; false = no separate thread, which may be more stable but has lagging interaction for large models (do not interact with models during simulation); set this parameter before call to exudyn.StartRenderer(); MAC OS: uses always false, because MAC OS does not support multi threaded GLFW


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsGeneral()
  {
    graphicsUpdateInterval = 0.1f;
    autoFitScene = true;
    textSize = 12.f;
    textColor = Float4({0.f,0.f,0.f,1.0f});
    useWindowsMonitorScaleFactor = true;
    useBitmapText = true;
    minSceneSize = 0.1f;
    backgroundColor = Float4({1.0f,1.0f,1.0f,1.0f});
    backgroundColorBottom = Float4({0.8f,0.8f,1.0f,1.0f});
    useGradientBackground = false;
    coordinateSystemSize = 5.f;
    drawCoordinateSystem = true;
    drawWorldBasis = false;
    worldBasisSize = 1.0f;
    showHelpOnStartup = 5;
    showComputationInfo = true;
    showSolutionInformation = true;
    showSolverInformation = true;
    showSolverTime = true;
    pointSize = 0.01f;
    circleTiling = 16;
    cylinderTiling = 16;
    sphereTiling = 6;
    axesTiling = 12;
    threadSafeGraphicsUpdate = true;
    useMultiThreadedRendering = true;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: general text color (default); used for system texts in render window
  void PySetTextColor(const std::array<float,4>& textColorInit) { textColor = textColorInit; }
  //! AUTO: Read (Copy) access to: general text color (default); used for system texts in render window
  std::array<float,4> PyGetTextColor() const { return (std::array<float,4>)(textColor); }

  //! AUTO: Set function (needed in pybind) for: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  void PySetBackgroundColor(const std::array<float,4>& backgroundColorInit) { backgroundColor = backgroundColorInit; }
  //! AUTO: Read (Copy) access to: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  std::array<float,4> PyGetBackgroundColor() const { return (std::array<float,4>)(backgroundColor); }

  //! AUTO: Set function (needed in pybind) for: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  void PySetBackgroundColorBottom(const std::array<float,4>& backgroundColorBottomInit) { backgroundColorBottom = backgroundColorBottomInit; }
  //! AUTO: Read (Copy) access to: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  std::array<float,4> PyGetBackgroundColorBottom() const { return (std::array<float,4>)(backgroundColorBottom); }

  //! AUTO: Set function (needed in pybind) for: seconds to show help message on startup (0=deactivate)
  void PySetShowHelpOnStartup(const Index& showHelpOnStartupInit) { showHelpOnStartup = EXUstd::GetSafelyPInt(showHelpOnStartupInit,"showHelpOnStartup"); }
  //! AUTO: Read (Copy) access to: seconds to show help message on startup (0=deactivate)
  Index PyGetShowHelpOnStartup() const { return (Index)(showHelpOnStartup); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  void PySetCircleTiling(const Index& circleTilingInit) { circleTiling = EXUstd::GetSafelyPInt(circleTilingInit,"circleTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  Index PyGetCircleTiling() const { return (Index)(circleTiling); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  void PySetCylinderTiling(const Index& cylinderTilingInit) { cylinderTiling = EXUstd::GetSafelyPInt(cylinderTilingInit,"cylinderTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  Index PyGetCylinderTiling() const { return (Index)(cylinderTiling); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  void PySetSphereTiling(const Index& sphereTilingInit) { sphereTiling = EXUstd::GetSafelyPInt(sphereTilingInit,"sphereTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  Index PyGetSphereTiling() const { return (Index)(sphereTiling); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for drawing axes cylinders and cones (reduce this number, e.g. to 4, if many axes are drawn)
  void PySetAxesTiling(const Index& axesTilingInit) { axesTiling = EXUstd::GetSafelyPInt(axesTilingInit,"axesTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for drawing axes cylinders and cones (reduce this number, e.g. to 4, if many axes are drawn)
  Index PyGetAxesTiling() const { return (Index)(axesTiling); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsGeneral" << ":\n";
    os << "  graphicsUpdateInterval = " << graphicsUpdateInterval << "\n";
    os << "  autoFitScene = " << autoFitScene << "\n";
    os << "  textSize = " << textSize << "\n";
    os << "  textColor = " << textColor << "\n";
    os << "  useWindowsMonitorScaleFactor = " << useWindowsMonitorScaleFactor << "\n";
    os << "  useBitmapText = " << useBitmapText << "\n";
    os << "  minSceneSize = " << minSceneSize << "\n";
    os << "  backgroundColor = " << backgroundColor << "\n";
    os << "  backgroundColorBottom = " << backgroundColorBottom << "\n";
    os << "  useGradientBackground = " << useGradientBackground << "\n";
    os << "  coordinateSystemSize = " << coordinateSystemSize << "\n";
    os << "  drawCoordinateSystem = " << drawCoordinateSystem << "\n";
    os << "  drawWorldBasis = " << drawWorldBasis << "\n";
    os << "  worldBasisSize = " << worldBasisSize << "\n";
    os << "  showHelpOnStartup = " << showHelpOnStartup << "\n";
    os << "  showComputationInfo = " << showComputationInfo << "\n";
    os << "  showSolutionInformation = " << showSolutionInformation << "\n";
    os << "  showSolverInformation = " << showSolverInformation << "\n";
    os << "  showSolverTime = " << showSolverTime << "\n";
    os << "  renderWindowString = " << renderWindowString << "\n";
    os << "  pointSize = " << pointSize << "\n";
    os << "  circleTiling = " << circleTiling << "\n";
    os << "  cylinderTiling = " << cylinderTiling << "\n";
    os << "  sphereTiling = " << sphereTiling << "\n";
    os << "  axesTiling = " << axesTiling << "\n";
    os << "  threadSafeGraphicsUpdate = " << threadSafeGraphicsUpdate << "\n";
    os << "  useMultiThreadedRendering = " << useMultiThreadedRendering << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsGeneral& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsContour
* @brief        Settings for contour plots; use these options to visualize field data, such as displacements, stresses, strains, etc. for bodies, nodes and finite elements.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsContour // AUTO: 
{
public: // AUTO: 
  int outputVariableComponent;                    //!< AUTO: select the component of the chosen output variable; e.g., for displacements, 3 components are available: 0 == x, 1 == y, 2 == z component; for stresses, 6 components are available, see OutputVariableType description; to draw the norm of a outputVariable, set component to -1; if a certain component is not available by certain objects or nodes, no value is drawn (using default color)
  OutputVariableType outputVariable;              //!< AUTO: selected contour plot output variable type; select OutputVariableType.\_None to deactivate contour plotting.
  float minValue;                                 //!< AUTO: minimum value for contour plot; set manually, if automaticRange == False
  float maxValue;                                 //!< AUTO: maximum value for contour plot; set manually, if automaticRange == False
  bool automaticRange;                            //!< AUTO: if true, the contour plot value range is chosen automatically to the maximum range
  bool reduceRange;                               //!< AUTO: if true, the contour plot value range is also reduced; better for static computation; in dynamic computation set this option to false, it can reduce visualization artifacts; you should also set minVal to max(float) and maxVal to min(float)
  bool showColorBar;                              //!< AUTO: show the colour bar with minimum and maximum values for the contour plot
  Index colorBarTiling;                           //!< AUTO: number of tiles (segements) shown in the colorbar for the contour plot


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsContour()
  {
    outputVariableComponent = 0;
    outputVariable = OutputVariableType::_None;
    minValue = 0;
    maxValue = 1;
    automaticRange = true;
    reduceRange = true;
    showColorBar = true;
    colorBarTiling = 12;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: number of tiles (segements) shown in the colorbar for the contour plot
  void PySetColorBarTiling(const Index& colorBarTilingInit) { colorBarTiling = EXUstd::GetSafelyPInt(colorBarTilingInit,"colorBarTiling"); }
  //! AUTO: Read (Copy) access to: number of tiles (segements) shown in the colorbar for the contour plot
  Index PyGetColorBarTiling() const { return (Index)(colorBarTiling); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsContour" << ":\n";
    os << "  outputVariableComponent = " << outputVariableComponent << "\n";
    os << "  outputVariable = " << GetOutputVariableTypeString(outputVariable) << "\n";
    os << "  minValue = " << minValue << "\n";
    os << "  maxValue = " << maxValue << "\n";
    os << "  automaticRange = " << automaticRange << "\n";
    os << "  reduceRange = " << reduceRange << "\n";
    os << "  showColorBar = " << showColorBar << "\n";
    os << "  colorBarTiling = " << colorBarTiling << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsContour& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsNodes
* @brief        Visualization settings for nodes.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsNodes // AUTO: 
{
public: // AUTO: 
  bool show;                                      //!< AUTO: flag to decide, whether the nodes are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the node number is shown
  bool drawNodesAsPoint;                          //!< AUTO: simplified/faster drawing of nodes; uses general->pointSize as drawing size; if drawNodesAsPoint==True, the basis of the node will be drawn with lines
  bool showBasis;                                 //!< AUTO: show basis (three axes) of coordinate system in 3D nodes
  float basisSize;                                //!< AUTO: size of basis for nodes
  Index tiling;                                   //!< AUTO: tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4
  float defaultSize;                              //!< AUTO: global node size; if -1.f, node size is relative to openGL.initialMaxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for nodes; 4th value is alpha-transparency
  Index showNodalSlopes;                          //!< AUTO: draw nodal slope vectors, e.g. in ANCF beam finite elements


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsNodes()
  {
    show = true;
    showNumbers = false;
    drawNodesAsPoint = true;
    showBasis = false;
    basisSize = 0.2f;
    tiling = 4;
    defaultSize = -1.f;
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
    showNodalSlopes = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4
  void PySetTiling(const Index& tilingInit) { tiling = EXUstd::GetSafelyPInt(tilingInit,"tiling"); }
  //! AUTO: Read (Copy) access to: tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4
  Index PyGetTiling() const { return (Index)(tiling); }

  //! AUTO: Set function (needed in pybind) for: default cRGB olor for nodes; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB olor for nodes; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return (std::array<float,4>)(defaultColor); }

  //! AUTO: Set function (needed in pybind) for: draw nodal slope vectors, e.g. in ANCF beam finite elements
  void PySetShowNodalSlopes(const Index& showNodalSlopesInit) { showNodalSlopes = EXUstd::GetSafelyUInt(showNodalSlopesInit,"showNodalSlopes"); }
  //! AUTO: Read (Copy) access to: draw nodal slope vectors, e.g. in ANCF beam finite elements
  Index PyGetShowNodalSlopes() const { return (Index)(showNodalSlopes); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsNodes" << ":\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  drawNodesAsPoint = " << drawNodesAsPoint << "\n";
    os << "  showBasis = " << showBasis << "\n";
    os << "  basisSize = " << basisSize << "\n";
    os << "  tiling = " << tiling << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  showNodalSlopes = " << showNodalSlopes << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsNodes& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsBeams
* @brief        Visualization settings for beam finite elements.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsBeams // AUTO: 
{
public: // AUTO: 
  Index axialTiling;                              //!< AUTO: number of segments to discretise the beams axis


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsBeams()
  {
    axialTiling = 8;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: number of segments to discretise the beams axis
  void PySetAxialTiling(const Index& axialTilingInit) { axialTiling = EXUstd::GetSafelyPInt(axialTilingInit,"axialTiling"); }
  //! AUTO: Read (Copy) access to: number of segments to discretise the beams axis
  Index PyGetAxialTiling() const { return (Index)(axialTiling); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsBeams" << ":\n";
    os << "  axialTiling = " << axialTiling << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsBeams& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsBodies
* @brief        Visualization settings for bodies.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsBodies // AUTO: 
{
public: // AUTO: 
  bool show;                                      //!< AUTO: flag to decide, whether the bodies are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the body(=object) number is shown
  Float3 defaultSize;                             //!< AUTO: global body size of xyz-cube
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for bodies; 4th value is 
  float deformationScaleFactor;                   //!< AUTO: global deformation scale factor; also applies to nodes, if drawn; used for scaled drawing of (linear) finite elements, beams, etc.
  VSettingsBeams beams;                           //!< AUTO: visualization settings for beams (e.g. ANCFCable or other beam elements)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsBodies()
  {
    show = true;
    showNumbers = false;
    defaultSize = Float3({1.f,1.f,1.f});
    defaultColor = Float4({0.3f,0.3f,1.f,1.f});
    deformationScaleFactor = 1;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: global body size of xyz-cube
  void PySetDefaultSize(const std::array<float,3>& defaultSizeInit) { defaultSize = defaultSizeInit; }
  //! AUTO: Read (Copy) access to: global body size of xyz-cube
  std::array<float,3> PyGetDefaultSize() const { return (std::array<float,3>)(defaultSize); }

  //! AUTO: Set function (needed in pybind) for: default cRGB olor for bodies; 4th value is 
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB olor for bodies; 4th value is 
  std::array<float,4> PyGetDefaultColor() const { return (std::array<float,4>)(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsBodies" << ":\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  deformationScaleFactor = " << deformationScaleFactor << "\n";
    os << "  beams = " << beams << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsBodies& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsConnectors
* @brief        Visualization settings for connectors.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsConnectors // AUTO: 
{
public: // AUTO: 
  bool show;                                      //!< AUTO: flag to decide, whether the connectors are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the connector(=object) number is shown
  float defaultSize;                              //!< AUTO: global connector size; if -1.f, connector size is relative to maxSceneSize
  bool showJointAxes;                             //!< AUTO: flag to decide, whether contact joint axes of 3D joints are shown
  float jointAxesLength;                          //!< AUTO: global joint axes length
  float jointAxesRadius;                          //!< AUTO: global joint axes radius
  bool showContact;                               //!< AUTO: flag to decide, whether contact points, lines, etc. are shown
  Index springNumberOfWindings;                   //!< AUTO: number of windings for springs drawn as helical spring
  float contactPointsDefaultSize;                 //!< AUTO: global contact points size; if -1.f, connector size is relative to maxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for connectors; 4th value is alpha-transparency


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsConnectors()
  {
    show = true;
    showNumbers = false;
    defaultSize = 0.1f;
    showJointAxes = false;
    jointAxesLength = 0.2f;
    jointAxesRadius = 0.02f;
    showContact = false;
    springNumberOfWindings = 8;
    contactPointsDefaultSize = 0.02f;
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: number of windings for springs drawn as helical spring
  void PySetSpringNumberOfWindings(const Index& springNumberOfWindingsInit) { springNumberOfWindings = EXUstd::GetSafelyPInt(springNumberOfWindingsInit,"springNumberOfWindings"); }
  //! AUTO: Read (Copy) access to: number of windings for springs drawn as helical spring
  Index PyGetSpringNumberOfWindings() const { return (Index)(springNumberOfWindings); }

  //! AUTO: Set function (needed in pybind) for: default cRGB olor for connectors; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB olor for connectors; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return (std::array<float,4>)(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsConnectors" << ":\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  showJointAxes = " << showJointAxes << "\n";
    os << "  jointAxesLength = " << jointAxesLength << "\n";
    os << "  jointAxesRadius = " << jointAxesRadius << "\n";
    os << "  showContact = " << showContact << "\n";
    os << "  springNumberOfWindings = " << springNumberOfWindings << "\n";
    os << "  contactPointsDefaultSize = " << contactPointsDefaultSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsConnectors& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsMarkers
* @brief        Visualization settings for markers.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsMarkers // AUTO: 
{
public: // AUTO: 
  bool show;                                      //!< AUTO: flag to decide, whether the markers are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the marker numbers are shown
  bool drawSimplified;                            //!< AUTO: draw markers with simplified symbols
  float defaultSize;                              //!< AUTO: global marker size; if -1.f, marker size is relative to maxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for markers; 4th value is alpha-transparency


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsMarkers()
  {
    show = true;
    showNumbers = false;
    drawSimplified = true;
    defaultSize = -1.f;
    defaultColor = Float4({0.1f,0.5f,0.1f,1.f});
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB olor for markers; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB olor for markers; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return (std::array<float,4>)(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsMarkers" << ":\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsMarkers& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsLoads
* @brief        Visualization settings for loads.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsLoads // AUTO: 
{
public: // AUTO: 
  bool show;                                      //!< AUTO: flag to decide, whether the loads are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the load numbers are shown
  float defaultSize;                              //!< AUTO: global load size; if -1.f, load size is relative to maxSceneSize
  float defaultRadius;                            //!< AUTO: global radius of load axis if drawn in 3D
  bool fixedLoadSize;                             //!< AUTO: if true, the load is drawn with a fixed vector length in direction of the load vector, independently of the load size
  bool drawSimplified;                            //!< AUTO: draw markers with simplified symbols
  float loadSizeFactor;                           //!< AUTO: if fixedLoadSize=false, then this scaling factor is used to draw the load vector
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for loads; 4th value is alpha-transparency


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsLoads()
  {
    show = true;
    showNumbers = false;
    defaultSize = 0.2f;
    defaultRadius = 0.005f;
    fixedLoadSize = true;
    drawSimplified = true;
    loadSizeFactor = 0.1f;
    defaultColor = Float4({0.7f,0.1f,0.1f,1.f});
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB olor for loads; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB olor for loads; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return (std::array<float,4>)(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsLoads" << ":\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  defaultRadius = " << defaultRadius << "\n";
    os << "  fixedLoadSize = " << fixedLoadSize << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  loadSizeFactor = " << loadSizeFactor << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsLoads& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsSensors
* @brief        Visualization settings for sensors.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsSensors // AUTO: 
{
public: // AUTO: 
  bool show;                                      //!< AUTO: flag to decide, whether the sensors are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the sensor numbers are shown
  bool drawSimplified;                            //!< AUTO: draw sensors with simplified symbols
  float defaultSize;                              //!< AUTO: global sensor size; if -1.f, sensor size is relative to maxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for sensors; 4th value is alpha-transparency


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsSensors()
  {
    show = true;
    showNumbers = false;
    drawSimplified = true;
    defaultSize = -1.f;
    defaultColor = Float4({0.6f,0.6f,0.1f,1.f});
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB olor for sensors; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB olor for sensors; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return (std::array<float,4>)(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsSensors" << ":\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsSensors& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsWindow
* @brief        Window and interaction settings for visualization; handle changes with care, as they might lead to unexpected results or crashes.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsWindow // AUTO: 
{
public: // AUTO: 
  Index2 renderWindowSize;                        //!< AUTO: initial size of OpenGL render window in pixel
  Index startupTimeout;                           //!< AUTO: OpenGL render window startup timeout in ms (change might be necessary if CPU is very slow)
  bool alwaysOnTop;                               //!< AUTO: True: OpenGL render window will be always on top of all other windows
  bool maximize;                                  //!< AUTO: True: OpenGL render window will be maximized at startup
  bool showWindow;                                //!< AUTO: True: OpenGL render window is shown on startup; False: window will be iconified at startup (e.g. if you are starting multiple computations automatically)
  std::function<bool(int, int, int)> keyPressUserFunction;//!< AUTO: add a Python function f(key, action, mods) here, which is called every time a key is pressed; function shall return true, if key has been processed; Example: \tabnewline def f(key, action, mods):\tabnewline \phantom{XXX} print('key=',key);\tabnewline use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
  bool showMouseCoordinates;                      //!< AUTO: True: show OpenGL coordinates and distance to last left mouse button pressed position; switched on/off with key 'F3'
  bool ignoreKeys;                                //!< AUTO: True: ignore keyboard input except escape and 'F2' keys; used for interactive mode, e.g., to perform kinematic analysis; This flag can be switched with key 'F2'


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsWindow()
  {
    renderWindowSize = Index2({1024,768});
    startupTimeout = 2500;
    alwaysOnTop = false;
    maximize = false;
    showWindow = true;
    keyPressUserFunction = 0;
    showMouseCoordinates = false;
    ignoreKeys = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: initial size of OpenGL render window in pixel
  void PySetRenderWindowSize(const std::array<Index,2>& renderWindowSizeInit) { renderWindowSize = renderWindowSizeInit; }
  //! AUTO: Read (Copy) access to: initial size of OpenGL render window in pixel
  std::array<Index,2> PyGetRenderWindowSize() const { return (std::array<Index,2>)(renderWindowSize); }

  //! AUTO: Set function (needed in pybind) for: OpenGL render window startup timeout in ms (change might be necessary if CPU is very slow)
  void PySetStartupTimeout(const Index& startupTimeoutInit) { startupTimeout = EXUstd::GetSafelyPInt(startupTimeoutInit,"startupTimeout"); }
  //! AUTO: Read (Copy) access to: OpenGL render window startup timeout in ms (change might be necessary if CPU is very slow)
  Index PyGetStartupTimeout() const { return (Index)(startupTimeout); }

  //! AUTO: set keyPressUserFunction to zero (no function); because this cannot be assign to the variable itself
  void ResetKeyPressUserFunction() {
    keyPressUserFunction = 0;
  }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsWindow" << ":\n";
    os << "  renderWindowSize = " << renderWindowSize << "\n";
    os << "  startupTimeout = " << startupTimeout << "\n";
    os << "  alwaysOnTop = " << alwaysOnTop << "\n";
    os << "  maximize = " << maximize << "\n";
    os << "  showWindow = " << showWindow << "\n";
    os << "  showMouseCoordinates = " << showMouseCoordinates << "\n";
    os << "  ignoreKeys = " << ignoreKeys << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsWindow& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsOpenGL
* @brief        OpenGL settings for 2D and 2D rendering. For further details, see the OpenGL functionality
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsOpenGL // AUTO: 
{
public: // AUTO: 
  Float3 initialCenterPoint;                      //!< AUTO: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  float initialZoom;                              //!< AUTO: initial zoom of scene; overwritten/ignored if autoFitScene = True
  float initialMaxSceneSize;                      //!< AUTO: initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  StdArray33F initialModelRotation;               //!< AUTO: initial model rotation matrix for OpenGl; in python use e.g.: initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]
  Index multiSampling;                            //!< AUTO: multi sampling turned off (<=1) or turned on to given values (2, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  float lineWidth;                                //!< AUTO: width of lines used for representation of lines, circles, points, etc.
  bool lineSmooth;                                //!< AUTO: draw lines smooth
  float textLineWidth;                            //!< AUTO: width of lines used for representation of text
  bool textLineSmooth;                            //!< AUTO: draw lines for representation of text smooth
  bool showFaces;                                 //!< AUTO: show faces of triangles, etc.; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
  bool facesTransparent;                          //!< AUTO: True: show faces transparent independent of transparency (A)-value in color of objects; allow to show otherwise hidden node/marker/object numbers
  bool showFaceEdges;                             //!< AUTO: show edges of faces; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
  bool shadeModelSmooth;                          //!< AUTO: True: turn on smoothing for shaders, which uses vertex normals to smooth surfaces
  Float4 materialAmbientAndDiffuse;               //!< AUTO: 4f ambient color of material
  float materialShininess;                        //!< AUTO: shininess of material
  Float4 materialSpecular;                        //!< AUTO: 4f specular color of material
  bool enableLighting;                            //!< AUTO: generally enable lighting (otherwise, colors of objects are used); OpenGL: glEnable(GL\_LIGHTING)
  bool lightModelLocalViewer;                     //!< AUTO: select local viewer for light; maps to OpenGL glLightModeli(GL\_LIGHT\_MODEL\_LOCAL\_VIEWER,...)
  bool lightModelTwoSide;                         //!< AUTO: enlighten also backside of object; maps to OpenGL glLightModeli(GL\_LIGHT\_MODEL\_TWO\_SIDE,...)
  Float4 lightModelAmbient;                       //!< AUTO: global ambient light; maps to OpenGL glLightModeli(GL\_LIGHT\_MODEL\_AMBIENT,[r,g,b,a])
  bool enableLight0;                              //!< AUTO: turn on/off light0
  Float4 light0position;                          //!< AUTO: 4f position vector of GL\_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  float light0ambient;                            //!< AUTO: ambient value of GL\_LIGHT0
  float light0diffuse;                            //!< AUTO: diffuse value of GL\_LIGHT0
  float light0specular;                           //!< AUTO: specular value of GL\_LIGHT0
  float light0constantAttenuation;                //!< AUTO: constant attenuation coefficient of GL\_LIGHT0, this is a constant factor that attenuates the light source; attenuation factor = 1/(kx +kl*d + kq*d*d); (kc,kl,kq)=(1,0,0) means no attenuation; only used for lights, where last component of light position is 1
  float light0linearAttenuation;                  //!< AUTO: linear attenuation coefficient of GL\_LIGHT0, this is a linear factor for attenuation of the light source with distance
  float light0quadraticAttenuation;               //!< AUTO: quadratic attenuation coefficient of GL\_LIGHT0, this is a quadratic factor for attenuation of the light source with distance
  bool enableLight1;                              //!< AUTO: turn on/off light1
  Float4 light1position;                          //!< AUTO: 4f position vector of GL\_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  float light1ambient;                            //!< AUTO: ambient value of GL\_LIGHT1
  float light1diffuse;                            //!< AUTO: diffuse value of GL\_LIGHT1
  float light1specular;                           //!< AUTO: specular value of GL\_LIGHT1
  float light1constantAttenuation;                //!< AUTO: constant attenuation coefficient of GL\_LIGHT1, this is a constant factor that attenuates the light source; attenuation factor = 1/(kx +kl*d + kq*d*d); only used for lights, where last component of light position is 1
  float light1linearAttenuation;                  //!< AUTO: linear attenuation coefficient of GL\_LIGHT1, this is a linear factor for attenuation of the light source with distance
  float light1quadraticAttenuation;               //!< AUTO: quadratic attenuation coefficient of GL\_LIGHT1, this is a quadratic factor for attenuation of the light source with distance
  bool drawFaceNormals;                           //!< AUTO: draws triangle normals, e.g. at center of triangles; used for debugging of faces
  bool drawVertexNormals;                         //!< AUTO: draws vertex normals; used for debugging
  float drawNormalsLength;                        //!< AUTO: length of normals; used for debugging


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsOpenGL()
  {
    initialCenterPoint = Float3({0.f,0.f,0.f});
    initialZoom = 1.f;
    initialMaxSceneSize = 1.f;
    initialModelRotation = EXUmath::Matrix3DFToStdArray33(Matrix3DF(3,3,{1.f,0.f,0.f, 0.f,1.f,0.f, 0.f,0.f,1.f}));
    multiSampling = 1;
    lineWidth = 1.f;
    lineSmooth = true;
    textLineWidth = 1.f;
    textLineSmooth = false;
    showFaces = true;
    facesTransparent = false;
    showFaceEdges = false;
    shadeModelSmooth = true;
    materialAmbientAndDiffuse = Float4({0.6f,0.6f,0.6f,1.f});
    materialShininess = 32.f;
    materialSpecular = Float4({0.6f,0.6f,0.6f,1.f});
    enableLighting = true;
    lightModelLocalViewer = false;
    lightModelTwoSide = true;
    lightModelAmbient = Float4({0.f,0.f,0.f,1.f});
    enableLight0 = true;
    light0position = Float4({0.2f,0.2f,10.f,0.f});
    light0ambient = 0.3f;
    light0diffuse = 0.6f;
    light0specular = 0.5f;
    light0constantAttenuation = 1.0f;
    light0linearAttenuation = 0.0f;
    light0quadraticAttenuation = 0.0f;
    enableLight1 = true;
    light1position = Float4({1.f,1.f,-10.f,0.f});
    light1ambient = 0.0f ;
    light1diffuse = 0.5f;
    light1specular = 0.6f;
    light1constantAttenuation = 1.0f;
    light1linearAttenuation = 0.0f;
    light1quadraticAttenuation = 0.0f;
    drawFaceNormals = false;
    drawVertexNormals = false;
    drawNormalsLength = 0.1f;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  void PySetInitialCenterPoint(const std::array<float,3>& initialCenterPointInit) { initialCenterPoint = initialCenterPointInit; }
  //! AUTO: Read (Copy) access to: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  std::array<float,3> PyGetInitialCenterPoint() const { return (std::array<float,3>)(initialCenterPoint); }

  //! AUTO: Set function (needed in pybind) for: multi sampling turned off (<=1) or turned on to given values (2, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  void PySetMultiSampling(const Index& multiSamplingInit) { multiSampling = EXUstd::GetSafelyPInt(multiSamplingInit,"multiSampling"); }
  //! AUTO: Read (Copy) access to: multi sampling turned off (<=1) or turned on to given values (2, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  Index PyGetMultiSampling() const { return (Index)(multiSampling); }

  //! AUTO: Set function (needed in pybind) for: 4f ambient color of material
  void PySetMaterialAmbientAndDiffuse(const std::array<float,4>& materialAmbientAndDiffuseInit) { materialAmbientAndDiffuse = materialAmbientAndDiffuseInit; }
  //! AUTO: Read (Copy) access to: 4f ambient color of material
  std::array<float,4> PyGetMaterialAmbientAndDiffuse() const { return (std::array<float,4>)(materialAmbientAndDiffuse); }

  //! AUTO: Set function (needed in pybind) for: 4f specular color of material
  void PySetMaterialSpecular(const std::array<float,4>& materialSpecularInit) { materialSpecular = materialSpecularInit; }
  //! AUTO: Read (Copy) access to: 4f specular color of material
  std::array<float,4> PyGetMaterialSpecular() const { return (std::array<float,4>)(materialSpecular); }

  //! AUTO: Set function (needed in pybind) for: global ambient light; maps to OpenGL glLightModeli(GL\_LIGHT\_MODEL\_AMBIENT,[r,g,b,a])
  void PySetLightModelAmbient(const std::array<float,4>& lightModelAmbientInit) { lightModelAmbient = lightModelAmbientInit; }
  //! AUTO: Read (Copy) access to: global ambient light; maps to OpenGL glLightModeli(GL\_LIGHT\_MODEL\_AMBIENT,[r,g,b,a])
  std::array<float,4> PyGetLightModelAmbient() const { return (std::array<float,4>)(lightModelAmbient); }

  //! AUTO: Set function (needed in pybind) for: 4f position vector of GL\_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  void PySetLight0position(const std::array<float,4>& light0positionInit) { light0position = light0positionInit; }
  //! AUTO: Read (Copy) access to: 4f position vector of GL\_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  std::array<float,4> PyGetLight0position() const { return (std::array<float,4>)(light0position); }

  //! AUTO: Set function (needed in pybind) for: 4f position vector of GL\_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  void PySetLight1position(const std::array<float,4>& light1positionInit) { light1position = light1positionInit; }
  //! AUTO: Read (Copy) access to: 4f position vector of GL\_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  std::array<float,4> PyGetLight1position() const { return (std::array<float,4>)(light1position); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsOpenGL" << ":\n";
    os << "  initialCenterPoint = " << initialCenterPoint << "\n";
    os << "  initialZoom = " << initialZoom << "\n";
    os << "  initialMaxSceneSize = " << initialMaxSceneSize << "\n";
#ifndef __APPLE__
    os << "  initialModelRotation = " << Matrix3DF(initialModelRotation) << "\n";
#endif
    os << "  multiSampling = " << multiSampling << "\n";
    os << "  lineWidth = " << lineWidth << "\n";
    os << "  lineSmooth = " << lineSmooth << "\n";
    os << "  textLineWidth = " << textLineWidth << "\n";
    os << "  textLineSmooth = " << textLineSmooth << "\n";
    os << "  showFaces = " << showFaces << "\n";
    os << "  facesTransparent = " << facesTransparent << "\n";
    os << "  showFaceEdges = " << showFaceEdges << "\n";
    os << "  shadeModelSmooth = " << shadeModelSmooth << "\n";
    os << "  materialAmbientAndDiffuse = " << materialAmbientAndDiffuse << "\n";
    os << "  materialShininess = " << materialShininess << "\n";
    os << "  materialSpecular = " << materialSpecular << "\n";
    os << "  enableLighting = " << enableLighting << "\n";
    os << "  lightModelLocalViewer = " << lightModelLocalViewer << "\n";
    os << "  lightModelTwoSide = " << lightModelTwoSide << "\n";
    os << "  lightModelAmbient = " << lightModelAmbient << "\n";
    os << "  enableLight0 = " << enableLight0 << "\n";
    os << "  light0position = " << light0position << "\n";
    os << "  light0ambient = " << light0ambient << "\n";
    os << "  light0diffuse = " << light0diffuse << "\n";
    os << "  light0specular = " << light0specular << "\n";
    os << "  light0constantAttenuation = " << light0constantAttenuation << "\n";
    os << "  light0linearAttenuation = " << light0linearAttenuation << "\n";
    os << "  light0quadraticAttenuation = " << light0quadraticAttenuation << "\n";
    os << "  enableLight1 = " << enableLight1 << "\n";
    os << "  light1position = " << light1position << "\n";
    os << "  light1ambient = " << light1ambient << "\n";
    os << "  light1diffuse = " << light1diffuse << "\n";
    os << "  light1specular = " << light1specular << "\n";
    os << "  light1constantAttenuation = " << light1constantAttenuation << "\n";
    os << "  light1linearAttenuation = " << light1linearAttenuation << "\n";
    os << "  light1quadraticAttenuation = " << light1quadraticAttenuation << "\n";
    os << "  drawFaceNormals = " << drawFaceNormals << "\n";
    os << "  drawVertexNormals = " << drawVertexNormals << "\n";
    os << "  drawNormalsLength = " << drawNormalsLength << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsOpenGL& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsExportImages
* @brief        Functionality to export images to files (.tga format) which can be used to create animations; to activate image recording during the solution process, set SolutionSettings.recordImagesInterval accordingly.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsExportImages // AUTO: 
{
public: // AUTO: 
  Index saveImageTimeOut;                         //!< AUTO: timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  std::string saveImageFileName;                  //!< AUTO: filename (without extension!) and (relative) path for image file(s) with consecutive numbering (e.g., frame0000.tga, frame0001.tga,...); ; directory will be created if it does not exist
  Index saveImageFileCounter;                     //!< AUTO: current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  bool saveImageSingleFile;                       //!< AUTO: True: only save single files with given filename, not adding numbering; False: add numbering to files, see saveImageFileName


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsExportImages()
  {
    saveImageTimeOut = 5000;
    saveImageFileName = "images/frame";
    saveImageFileCounter = 0;
    saveImageSingleFile = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  void PySetSaveImageTimeOut(const Index& saveImageTimeOutInit) { saveImageTimeOut = EXUstd::GetSafelyPInt(saveImageTimeOutInit,"saveImageTimeOut"); }
  //! AUTO: Read (Copy) access to: timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  Index PyGetSaveImageTimeOut() const { return (Index)(saveImageTimeOut); }

  //! AUTO: Set function (needed in pybind) for: current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  void PySetSaveImageFileCounter(const Index& saveImageFileCounterInit) { saveImageFileCounter = EXUstd::GetSafelyUInt(saveImageFileCounterInit,"saveImageFileCounter"); }
  //! AUTO: Read (Copy) access to: current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  Index PyGetSaveImageFileCounter() const { return (Index)(saveImageFileCounter); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsExportImages" << ":\n";
    os << "  saveImageTimeOut = " << saveImageTimeOut << "\n";
    os << "  saveImageFileName = " << saveImageFileName << "\n";
    os << "  saveImageFileCounter = " << saveImageFileCounter << "\n";
    os << "  saveImageSingleFile = " << saveImageSingleFile << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsExportImages& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsInteractive
* @brief        Functionality to interact with render window; will include left and right mouse press actions and others in future.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VSettingsInteractive // AUTO: 
{
public: // AUTO: 
  float keypressRotationStep;                     //!< AUTO: rotation increment per keypress in degree (full rotation = 360 degree)
  float mouseMoveRotationFactor;                  //!< AUTO: rotation increment per 1 pixel mouse movement in degree
  float keypressTranslationStep;                  //!< AUTO: translation increment per keypress relative to window size
  float zoomStepFactor;                           //!< AUTO: change of zoom per keypress (keypad +/-) or mouse wheel increment
  int highlightItemIndex;                         //!< AUTO: index of item that shall be highlighted (e.g., need to find item due to errors); if set -1, no item is highlighted
  ItemType highlightItemType;                     //!< AUTO: item type (Node, Object, ...) that shall be highlighted (e.g., need to find item due to errors)
  Index highlightMbsNumber;                       //!< AUTO: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  Float4 highlightColor;                          //!< AUTO: cRGB color for highlighted item; 4th value is alpha-transparency
  Float4 highlightOtherColor;                     //!< AUTO: cRGB color for other items (which are not highlighted); 4th value is alpha-transparency
  bool selectionLeftMouse;                        //!< AUTO: True: left mouse click on items and show basic information
  bool selectionRightMouse;                       //!< AUTO: True: right mouse click on items and show dictionary (read only!)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsInteractive()
  {
    keypressRotationStep = 5.f;
    mouseMoveRotationFactor = 1.f;
    keypressTranslationStep = 0.1f;
    zoomStepFactor = 1.15f;
    highlightItemIndex = -1;
    highlightItemType = ItemType::_None;
    highlightMbsNumber = 0;
    highlightColor = Float4({0.8f,0.05f,0.05f,0.75f});
    highlightOtherColor = Float4({0.5f,0.5f,0.5f,0.4f});
    selectionLeftMouse = true;
    selectionRightMouse = true;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  void PySetHighlightMbsNumber(const Index& highlightMbsNumberInit) { highlightMbsNumber = EXUstd::GetSafelyUInt(highlightMbsNumberInit,"highlightMbsNumber"); }
  //! AUTO: Read (Copy) access to: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  Index PyGetHighlightMbsNumber() const { return (Index)(highlightMbsNumber); }

  //! AUTO: Set function (needed in pybind) for: cRGB color for highlighted item; 4th value is alpha-transparency
  void PySetHighlightColor(const std::array<float,4>& highlightColorInit) { highlightColor = highlightColorInit; }
  //! AUTO: Read (Copy) access to: cRGB color for highlighted item; 4th value is alpha-transparency
  std::array<float,4> PyGetHighlightColor() const { return (std::array<float,4>)(highlightColor); }

  //! AUTO: Set function (needed in pybind) for: cRGB color for other items (which are not highlighted); 4th value is alpha-transparency
  void PySetHighlightOtherColor(const std::array<float,4>& highlightOtherColorInit) { highlightOtherColor = highlightOtherColorInit; }
  //! AUTO: Read (Copy) access to: cRGB color for other items (which are not highlighted); 4th value is alpha-transparency
  std::array<float,4> PyGetHighlightOtherColor() const { return (std::array<float,4>)(highlightOtherColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsInteractive" << ":\n";
    os << "  keypressRotationStep = " << keypressRotationStep << "\n";
    os << "  mouseMoveRotationFactor = " << mouseMoveRotationFactor << "\n";
    os << "  keypressTranslationStep = " << keypressTranslationStep << "\n";
    os << "  zoomStepFactor = " << zoomStepFactor << "\n";
    os << "  highlightItemIndex = " << highlightItemIndex << "\n";
    os << "  highlightItemType = " << highlightItemType << "\n";
    os << "  highlightMbsNumber = " << highlightMbsNumber << "\n";
    os << "  highlightColor = " << highlightColor << "\n";
    os << "  highlightOtherColor = " << highlightOtherColor << "\n";
    os << "  selectionLeftMouse = " << selectionLeftMouse << "\n";
    os << "  selectionRightMouse = " << selectionRightMouse << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsInteractive& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VisualizationSettings
* @brief        Settings for visualization
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2021-08-12 (last modfied)
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

class VisualizationSettings // AUTO: 
{
public: // AUTO: 
  VSettingsGeneral general;                       //!< AUTO: general visualization settings
  VSettingsContour contour;                       //!< AUTO: contour plot visualization settings
  VSettingsNodes nodes;                           //!< AUTO: node visualization settings
  VSettingsBodies bodies;                         //!< AUTO: body visualization settings
  VSettingsConnectors connectors;                 //!< AUTO: connector visualization settings
  VSettingsMarkers markers;                       //!< AUTO: marker visualization settings
  VSettingsLoads loads;                           //!< AUTO: load visualization settings
  VSettingsSensors sensors;                       //!< AUTO: sensor visualization settings
  VSettingsWindow window;                         //!< AUTO: visualization window and interaction settings
  VSettingsOpenGL openGL;                         //!< AUTO: OpenGL rendering settings
  VSettingsInteractive interactive;               //!< AUTO: Settings for interaction with renderer
  VSettingsExportImages exportImages;             //!< AUTO: settings for exporting (saving) images to files in order to create animations


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VisualizationSettings" << ":\n";
    os << "  general = " << general << "\n";
    os << "  contour = " << contour << "\n";
    os << "  nodes = " << nodes << "\n";
    os << "  bodies = " << bodies << "\n";
    os << "  connectors = " << connectors << "\n";
    os << "  markers = " << markers << "\n";
    os << "  loads = " << loads << "\n";
    os << "  sensors = " << sensors << "\n";
    os << "  window = " << window << "\n";
    os << "  openGL = " << openGL << "\n";
    os << "  interactive = " << interactive << "\n";
    os << "  exportImages = " << exportImages << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VisualizationSettings& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
