/** ***********************************************************************************************
* @class        VSettingsGeneral
* @brief        General settings for visualization.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-04-07 (last modfied)
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
  bool autoFitScene;                              //!< AUTO: automatically fit scene within startup after StartRenderer()
  Index axesTiling;                               //!< AUTO: global number of segments for drawing axes cylinders and cones (reduce this number, e.g. to 4, if many axes are drawn)
  Float4 backgroundColor;                         //!< AUTO: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  Float4 backgroundColorBottom;                   //!< AUTO: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  Index circleTiling;                             //!< AUTO: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  float coordinateSystemSize;                     //!< AUTO: size of coordinate system relative to font size
  Index cylinderTiling;                           //!< AUTO: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  bool drawCoordinateSystem;                      //!< AUTO: false = no coordinate system shown
  bool drawWorldBasis;                            //!< AUTO: true = draw world basis coordinate system at (0,0,0)
  float graphicsUpdateInterval;                   //!< AUTO: interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
  float minSceneSize;                             //!< AUTO: minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
  float pointSize;                                //!< AUTO: global point size (absolute)
  Index rendererPrecision;                        //!< AUTO: precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
  std::string renderWindowString;                 //!< AUTO: string shown in render window (use this, e.g., for debugging, etc.; written below EXUDYN, similar to solutionInformation in SimulationSettings.solutionSettings)
  bool showComputationInfo;                       //!< AUTO: true = show (hide) all computation information including Exudyn and version
  Index showHelpOnStartup;                        //!< AUTO: seconds to show help message on startup (0=deactivate)
  bool showSolutionInformation;                   //!< AUTO: true = show solution information (from simulationSettings.solution)
  bool showSolverInformation;                     //!< AUTO: true = solver name and further information shown in render window
  bool showSolverTime;                            //!< AUTO: true = solver current time shown in render window
  Index sphereTiling;                             //!< AUTO: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  bool textAlwaysInFront;                         //!< AUTO: if true, text for item numbers and other item-related text is drawn in front; this may be unwanted in case that you only with to see numbers of objects in front; currently does not work with perspective
  Float4 textColor;                               //!< AUTO: general text color (default); used for system texts in render window
  bool textHasBackground;                         //!< AUTO: if true, text for item numbers and other item-related text have a background (depending on text color), allowing for better visibility if many numbers are shown; the text itself is black; therefore, dark background colors are ignored and shown as white
  float textOffsetFactor;                         //!< AUTO: This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes
  float textSize;                                 //!< AUTO: general text size (font size) in pixels if not overwritten; if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
  bool threadSafeGraphicsUpdate;                  //!< AUTO: true = updating of visualization is threadsafe, but slower for complicated models; deactivate this to speed up computation, but activate for generation of animations; may be improved in future by adding a safe visualizationUpdate state
  bool useBitmapText;                             //!< AUTO: if true, texts are displayed using pre-defined bitmaps for the text; may increase the complexity of your scene, e.g., if many (>10000) node numbers shown
  bool useGradientBackground;                     //!< AUTO: true = use vertical gradient for background; 
  bool useMultiThreadedRendering;                 //!< AUTO: true = rendering is done in separate thread; false = no separate thread, which may be more stable but has lagging interaction for large models (do not interact with models during simulation); set this parameter before call to exudyn.StartRenderer(); MAC OS: uses always false, because MAC OS does not support multi threaded GLFW
  bool useWindowsDisplayScaleFactor;              //!< AUTO: the Windows display scaling (monitor scaling; content scaling) factor is used for increased visibility of texts on high resolution displays; based on GLFW glfwGetWindowContentScale; deactivated on linux compilation as it leads to crashes (adjust textSize manually!)
  float worldBasisSize;                           //!< AUTO: size of world basis coordinate system


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsGeneral()
  {
    autoFitScene = true;
    axesTiling = 12;
    backgroundColor = Float4({1.0f,1.0f,1.0f,1.0f});
    backgroundColorBottom = Float4({0.8f,0.8f,1.0f,1.0f});
    circleTiling = 16;
    coordinateSystemSize = 5.f;
    cylinderTiling = 16;
    drawCoordinateSystem = true;
    drawWorldBasis = false;
    graphicsUpdateInterval = 0.1f;
    minSceneSize = 0.1f;
    pointSize = 0.01f;
    rendererPrecision = 4;
    showComputationInfo = true;
    showHelpOnStartup = 5;
    showSolutionInformation = true;
    showSolverInformation = true;
    showSolverTime = true;
    sphereTiling = 6;
    textAlwaysInFront = true;
    textColor = Float4({0.f,0.f,0.f,1.0f});
    textHasBackground = false;
    textOffsetFactor = 0.005f;
    textSize = 12.f;
    threadSafeGraphicsUpdate = true;
    useBitmapText = true;
    useGradientBackground = false;
    useMultiThreadedRendering = true;
    useWindowsDisplayScaleFactor = true;
    worldBasisSize = 1.0f;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: global number of segments for drawing axes cylinders and cones (reduce this number, e.g. to 4, if many axes are drawn)
  void PySetAxesTiling(const Index& axesTilingInit) { axesTiling = EXUstd::GetSafelyPInt(axesTilingInit,"axesTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for drawing axes cylinders and cones (reduce this number, e.g. to 4, if many axes are drawn)
  Index PyGetAxesTiling() const { return Index(axesTiling); }

  //! AUTO: Set function (needed in pybind) for: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  void PySetBackgroundColor(const std::array<float,4>& backgroundColorInit) { backgroundColor = backgroundColorInit; }
  //! AUTO: Read (Copy) access to: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  std::array<float,4> PyGetBackgroundColor() const { return std::array<float,4>(backgroundColor); }

  //! AUTO: Set function (needed in pybind) for: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  void PySetBackgroundColorBottom(const std::array<float,4>& backgroundColorBottomInit) { backgroundColorBottom = backgroundColorBottomInit; }
  //! AUTO: Read (Copy) access to: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  std::array<float,4> PyGetBackgroundColorBottom() const { return std::array<float,4>(backgroundColorBottom); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  void PySetCircleTiling(const Index& circleTilingInit) { circleTiling = EXUstd::GetSafelyPInt(circleTilingInit,"circleTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  Index PyGetCircleTiling() const { return Index(circleTiling); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  void PySetCylinderTiling(const Index& cylinderTilingInit) { cylinderTiling = EXUstd::GetSafelyPInt(cylinderTilingInit,"cylinderTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  Index PyGetCylinderTiling() const { return Index(cylinderTiling); }

  //! AUTO: Set function (needed in pybind) for: precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
  void PySetRendererPrecision(const Index& rendererPrecisionInit) { rendererPrecision = EXUstd::GetSafelyPInt(rendererPrecisionInit,"rendererPrecision"); }
  //! AUTO: Read (Copy) access to: precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
  Index PyGetRendererPrecision() const { return Index(rendererPrecision); }

  //! AUTO: Set function (needed in pybind) for: seconds to show help message on startup (0=deactivate)
  void PySetShowHelpOnStartup(const Index& showHelpOnStartupInit) { showHelpOnStartup = EXUstd::GetSafelyPInt(showHelpOnStartupInit,"showHelpOnStartup"); }
  //! AUTO: Read (Copy) access to: seconds to show help message on startup (0=deactivate)
  Index PyGetShowHelpOnStartup() const { return Index(showHelpOnStartup); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  void PySetSphereTiling(const Index& sphereTilingInit) { sphereTiling = EXUstd::GetSafelyPInt(sphereTilingInit,"sphereTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  Index PyGetSphereTiling() const { return Index(sphereTiling); }

  //! AUTO: Set function (needed in pybind) for: general text color (default); used for system texts in render window
  void PySetTextColor(const std::array<float,4>& textColorInit) { textColor = textColorInit; }
  //! AUTO: Read (Copy) access to: general text color (default); used for system texts in render window
  std::array<float,4> PyGetTextColor() const { return std::array<float,4>(textColor); }

  //! AUTO: Set function (needed in pybind) for: This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes
  void PySetTextOffsetFactor(const float& textOffsetFactorInit) { textOffsetFactor = EXUstd::GetSafelyUFloat(textOffsetFactorInit,"textOffsetFactor"); }
  //! AUTO: Read (Copy) access to: This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes
  float PyGetTextOffsetFactor() const { return float(textOffsetFactor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsGeneral" << ":\n";
    os << "  autoFitScene = " << autoFitScene << "\n";
    os << "  axesTiling = " << axesTiling << "\n";
    os << "  backgroundColor = " << backgroundColor << "\n";
    os << "  backgroundColorBottom = " << backgroundColorBottom << "\n";
    os << "  circleTiling = " << circleTiling << "\n";
    os << "  coordinateSystemSize = " << coordinateSystemSize << "\n";
    os << "  cylinderTiling = " << cylinderTiling << "\n";
    os << "  drawCoordinateSystem = " << drawCoordinateSystem << "\n";
    os << "  drawWorldBasis = " << drawWorldBasis << "\n";
    os << "  graphicsUpdateInterval = " << graphicsUpdateInterval << "\n";
    os << "  minSceneSize = " << minSceneSize << "\n";
    os << "  pointSize = " << pointSize << "\n";
    os << "  rendererPrecision = " << rendererPrecision << "\n";
    os << "  renderWindowString = " << renderWindowString << "\n";
    os << "  showComputationInfo = " << showComputationInfo << "\n";
    os << "  showHelpOnStartup = " << showHelpOnStartup << "\n";
    os << "  showSolutionInformation = " << showSolutionInformation << "\n";
    os << "  showSolverInformation = " << showSolverInformation << "\n";
    os << "  showSolverTime = " << showSolverTime << "\n";
    os << "  sphereTiling = " << sphereTiling << "\n";
    os << "  textAlwaysInFront = " << textAlwaysInFront << "\n";
    os << "  textColor = " << textColor << "\n";
    os << "  textHasBackground = " << textHasBackground << "\n";
    os << "  textOffsetFactor = " << textOffsetFactor << "\n";
    os << "  textSize = " << textSize << "\n";
    os << "  threadSafeGraphicsUpdate = " << threadSafeGraphicsUpdate << "\n";
    os << "  useBitmapText = " << useBitmapText << "\n";
    os << "  useGradientBackground = " << useGradientBackground << "\n";
    os << "  useMultiThreadedRendering = " << useMultiThreadedRendering << "\n";
    os << "  useWindowsDisplayScaleFactor = " << useWindowsDisplayScaleFactor << "\n";
    os << "  worldBasisSize = " << worldBasisSize << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  bool automaticRange;                            //!< AUTO: if true, the contour plot value range is chosen automatically to the maximum range
  Index colorBarPrecision;                        //!< AUTO: precision of floating point values shown in color bar; total number of digits used (max. 16)
  Index colorBarTiling;                           //!< AUTO: number of tiles (segements) shown in the colorbar for the contour plot
  float maxValue;                                 //!< AUTO: maximum value for contour plot; set manually, if automaticRange == False
  float minValue;                                 //!< AUTO: minimum value for contour plot; set manually, if automaticRange == False
  bool nodesColored;                              //!< AUTO: if true, the contour color is also applied to nodes (except mesh nodes), otherwise node drawing is not influenced by contour settings
  OutputVariableType outputVariable;              //!< AUTO: selected contour plot output variable type; select OutputVariableType._None to deactivate contour plotting.
  Index outputVariableComponent;                  //!< AUTO: select the component of the chosen output variable; e.g., for displacements, 3 components are available: 0 == x, 1 == y, 2 == z component; for stresses, 6 components are available, see OutputVariableType description; to draw the norm of a outputVariable, set component to -1; if a certain component is not available by certain objects or nodes, no value is drawn (using default color)
  bool reduceRange;                               //!< AUTO: if true, the contour plot value range is also reduced; better for static computation; in dynamic computation set this option to false, it can reduce visualization artifacts; you should also set minVal to max(float) and maxVal to min(float)
  bool rigidBodiesColored;                        //!< AUTO: if true, the contour color is also applied to triangular faces of rigid bodies and mass points, otherwise the rigid body drawing are not influenced by contour settings; for general rigid bodies (except for ObjectGround), Position, Displacement, DisplacementLocal(=0), Velocity, VelocityLocal, AngularVelocity, and AngularVelocityLocal are available; may slow down visualization!
  bool showColorBar;                              //!< AUTO: show the colour bar with minimum and maximum values for the contour plot


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsContour()
  {
    automaticRange = true;
    colorBarPrecision = 4;
    colorBarTiling = 12;
    maxValue = 1;
    minValue = 0;
    nodesColored = true;
    outputVariable = OutputVariableType::_None;
    outputVariableComponent = 0;
    reduceRange = true;
    rigidBodiesColored = true;
    showColorBar = true;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: precision of floating point values shown in color bar; total number of digits used (max. 16)
  void PySetColorBarPrecision(const Index& colorBarPrecisionInit) { colorBarPrecision = EXUstd::GetSafelyPInt(colorBarPrecisionInit,"colorBarPrecision"); }
  //! AUTO: Read (Copy) access to: precision of floating point values shown in color bar; total number of digits used (max. 16)
  Index PyGetColorBarPrecision() const { return Index(colorBarPrecision); }

  //! AUTO: Set function (needed in pybind) for: number of tiles (segements) shown in the colorbar for the contour plot
  void PySetColorBarTiling(const Index& colorBarTilingInit) { colorBarTiling = EXUstd::GetSafelyPInt(colorBarTilingInit,"colorBarTiling"); }
  //! AUTO: Read (Copy) access to: number of tiles (segements) shown in the colorbar for the contour plot
  Index PyGetColorBarTiling() const { return Index(colorBarTiling); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsContour" << ":\n";
    os << "  automaticRange = " << automaticRange << "\n";
    os << "  colorBarPrecision = " << colorBarPrecision << "\n";
    os << "  colorBarTiling = " << colorBarTiling << "\n";
    os << "  maxValue = " << maxValue << "\n";
    os << "  minValue = " << minValue << "\n";
    os << "  nodesColored = " << nodesColored << "\n";
    os << "  outputVariable = " << GetOutputVariableTypeString(outputVariable) << "\n";
    os << "  outputVariableComponent = " << outputVariableComponent << "\n";
    os << "  reduceRange = " << reduceRange << "\n";
    os << "  rigidBodiesColored = " << rigidBodiesColored << "\n";
    os << "  showColorBar = " << showColorBar << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  float basisSize;                                //!< AUTO: size of basis for nodes
  Float4 defaultColor;                            //!< AUTO: default cRGB color for nodes; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global node size; if -1.f, node size is relative to openGL.initialMaxSceneSize
  bool drawNodesAsPoint;                          //!< AUTO: simplified/faster drawing of nodes; uses general->pointSize as drawing size; if drawNodesAsPoint==True, the basis of the node will be drawn with lines
  bool show;                                      //!< AUTO: flag to decide, whether the nodes are shown
  bool showBasis;                                 //!< AUTO: show basis (three axes) of coordinate system in 3D nodes
  Index showNodalSlopes;                          //!< AUTO: draw nodal slope vectors, e.g. in ANCF beam finite elements
  bool showNumbers;                               //!< AUTO: flag to decide, whether the node number is shown
  Index tiling;                                   //!< AUTO: tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsNodes()
  {
    basisSize = 0.2f;
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
    defaultSize = -1.f;
    drawNodesAsPoint = true;
    show = true;
    showBasis = false;
    showNodalSlopes = false;
    showNumbers = false;
    tiling = 4;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB color for nodes; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB color for nodes; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

  //! AUTO: Set function (needed in pybind) for: draw nodal slope vectors, e.g. in ANCF beam finite elements
  void PySetShowNodalSlopes(const Index& showNodalSlopesInit) { showNodalSlopes = EXUstd::GetSafelyUInt(showNodalSlopesInit,"showNodalSlopes"); }
  //! AUTO: Read (Copy) access to: draw nodal slope vectors, e.g. in ANCF beam finite elements
  Index PyGetShowNodalSlopes() const { return Index(showNodalSlopes); }

  //! AUTO: Set function (needed in pybind) for: tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4
  void PySetTiling(const Index& tilingInit) { tiling = EXUstd::GetSafelyPInt(tilingInit,"tiling"); }
  //! AUTO: Read (Copy) access to: tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4
  Index PyGetTiling() const { return Index(tiling); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsNodes" << ":\n";
    os << "  basisSize = " << basisSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  drawNodesAsPoint = " << drawNodesAsPoint << "\n";
    os << "  show = " << show << "\n";
    os << "  showBasis = " << showBasis << "\n";
    os << "  showNodalSlopes = " << showNodalSlopes << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  tiling = " << tiling << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  bool crossSectionFilled;                        //!< AUTO: if implemented for element, cross section is drawn as solid (filled) instead of wire-frame; NOTE: some quantities may not be interpolated correctly over cross section in visualization
  Index crossSectionTiling;                       //!< AUTO: number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing
  bool drawVertical;                              //!< AUTO: draw contour plot outputVariables 'vertical' along beam height; contour.outputVariable must be set accordingly
  Float4 drawVerticalColor;                       //!< AUTO: color for outputVariable to be drawn along cross section (vertically)
  float drawVerticalFactor;                       //!< AUTO: factor for outputVariable to be drawn along cross section (vertically)
  bool drawVerticalLines;                         //!< AUTO: draw additional vertical lines for better visibility
  float drawVerticalOffset;                       //!< AUTO: offset for vertical drawn lines; offset is added before multiplication with drawVerticalFactor
  bool drawVerticalValues;                        //!< AUTO: show values at vertical lines; note that these numbers are interpolated values and may be different from values evaluated directly at this point!
  bool reducedAxialInterploation;                 //!< AUTO: if True, the interpolation along the beam axis may be lower than the beam element order; this may be, however, show more consistent values than a full interpolation, e.g. for strains or forces


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsBeams()
  {
    axialTiling = 8;
    crossSectionFilled = true;
    crossSectionTiling = 4;
    drawVertical = false;
    drawVerticalColor = Float4({0.2f,0.2f,0.2f,1.f});
    drawVerticalFactor = 1.f;
    drawVerticalLines = true;
    drawVerticalOffset = 0.f;
    drawVerticalValues = false;
    reducedAxialInterploation = true;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: number of segments to discretise the beams axis
  void PySetAxialTiling(const Index& axialTilingInit) { axialTiling = EXUstd::GetSafelyPInt(axialTilingInit,"axialTiling"); }
  //! AUTO: Read (Copy) access to: number of segments to discretise the beams axis
  Index PyGetAxialTiling() const { return Index(axialTiling); }

  //! AUTO: Set function (needed in pybind) for: number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing
  void PySetCrossSectionTiling(const Index& crossSectionTilingInit) { crossSectionTiling = EXUstd::GetSafelyPInt(crossSectionTilingInit,"crossSectionTiling"); }
  //! AUTO: Read (Copy) access to: number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing
  Index PyGetCrossSectionTiling() const { return Index(crossSectionTiling); }

  //! AUTO: Set function (needed in pybind) for: color for outputVariable to be drawn along cross section (vertically)
  void PySetDrawVerticalColor(const std::array<float,4>& drawVerticalColorInit) { drawVerticalColor = drawVerticalColorInit; }
  //! AUTO: Read (Copy) access to: color for outputVariable to be drawn along cross section (vertically)
  std::array<float,4> PyGetDrawVerticalColor() const { return std::array<float,4>(drawVerticalColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsBeams" << ":\n";
    os << "  axialTiling = " << axialTiling << "\n";
    os << "  crossSectionFilled = " << crossSectionFilled << "\n";
    os << "  crossSectionTiling = " << crossSectionTiling << "\n";
    os << "  drawVertical = " << drawVertical << "\n";
    os << "  drawVerticalColor = " << drawVerticalColor << "\n";
    os << "  drawVerticalFactor = " << drawVerticalFactor << "\n";
    os << "  drawVerticalLines = " << drawVerticalLines << "\n";
    os << "  drawVerticalOffset = " << drawVerticalOffset << "\n";
    os << "  drawVerticalValues = " << drawVerticalValues << "\n";
    os << "  reducedAxialInterploation = " << reducedAxialInterploation << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsBeams& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsKinematicTree
* @brief        Visualization settings for kinematic trees.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-04-07 (last modfied)
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

class VSettingsKinematicTree // AUTO: 
{
public: // AUTO: 
  float frameSize;                                //!< AUTO: size of COM and joint frames
  bool showCOMframes;                             //!< AUTO: if True, a frame is attached to every center of mass
  bool showFramesNumbers;                         //!< AUTO: if True, numbers are drawn for joint frames (O[i]J[j]) and COM frames (O[i]COM[j]) for object [i] and local joint [j]
  bool showJointFrames;                           //!< AUTO: if True, a frame is attached to the origin of every joint frame


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsKinematicTree()
  {
    frameSize = 0.2f;
    showCOMframes = false;
    showFramesNumbers = true;
    showJointFrames = true;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsKinematicTree" << ":\n";
    os << "  frameSize = " << frameSize << "\n";
    os << "  showCOMframes = " << showCOMframes << "\n";
    os << "  showFramesNumbers = " << showFramesNumbers << "\n";
    os << "  showJointFrames = " << showJointFrames << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsKinematicTree& object)
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  VSettingsBeams beams;                           //!< AUTO: visualization settings for beams (e.g. ANCFCable or other beam elements)
  VSettingsKinematicTree kinematicTree;           //!< AUTO: visualization settings for kinematic tree
  Float4 defaultColor;                            //!< AUTO: default cRGB color for bodies; 4th value is 
  Float3 defaultSize;                             //!< AUTO: global body size of xyz-cube
  float deformationScaleFactor;                   //!< AUTO: global deformation scale factor; also applies to nodes, if drawn; used for scaled drawing of (linear) finite elements, beams, etc.
  bool show;                                      //!< AUTO: flag to decide, whether the bodies are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the body(=object) number is shown


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsBodies()
  {
    defaultColor = Float4({0.3f,0.3f,1.f,1.f});
    defaultSize = Float3({1.f,1.f,1.f});
    deformationScaleFactor = 1;
    show = true;
    showNumbers = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB color for bodies; 4th value is 
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB color for bodies; 4th value is 
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

  //! AUTO: Set function (needed in pybind) for: global body size of xyz-cube
  void PySetDefaultSize(const std::array<float,3>& defaultSizeInit) { defaultSize = defaultSizeInit; }
  //! AUTO: Read (Copy) access to: global body size of xyz-cube
  std::array<float,3> PyGetDefaultSize() const { return std::array<float,3>(defaultSize); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsBodies" << ":\n";
    os << "  beams = " << beams << "\n";
    os << "  kinematicTree = " << kinematicTree << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  deformationScaleFactor = " << deformationScaleFactor << "\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  float contactPointsDefaultSize;                 //!< AUTO: DEPRECATED: do not use! global contact points size; if -1.f, connector size is relative to maxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB color for connectors; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global connector size; if -1.f, connector size is relative to maxSceneSize
  float jointAxesLength;                          //!< AUTO: global joint axes length
  float jointAxesRadius;                          //!< AUTO: global joint axes radius
  bool show;                                      //!< AUTO: flag to decide, whether the connectors are shown
  bool showContact;                               //!< AUTO: flag to decide, whether contact points, lines, etc. are shown
  bool showJointAxes;                             //!< AUTO: flag to decide, whether contact joint axes of 3D joints are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the connector(=object) number is shown
  Index springNumberOfWindings;                   //!< AUTO: number of windings for springs drawn as helical spring


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsConnectors()
  {
    contactPointsDefaultSize = 0.02f;
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
    defaultSize = 0.1f;
    jointAxesLength = 0.2f;
    jointAxesRadius = 0.02f;
    show = true;
    showContact = false;
    showJointAxes = false;
    showNumbers = false;
    springNumberOfWindings = 8;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB color for connectors; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB color for connectors; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

  //! AUTO: Set function (needed in pybind) for: number of windings for springs drawn as helical spring
  void PySetSpringNumberOfWindings(const Index& springNumberOfWindingsInit) { springNumberOfWindings = EXUstd::GetSafelyPInt(springNumberOfWindingsInit,"springNumberOfWindings"); }
  //! AUTO: Read (Copy) access to: number of windings for springs drawn as helical spring
  Index PyGetSpringNumberOfWindings() const { return Index(springNumberOfWindings); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsConnectors" << ":\n";
    os << "  contactPointsDefaultSize = " << contactPointsDefaultSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  jointAxesLength = " << jointAxesLength << "\n";
    os << "  jointAxesRadius = " << jointAxesRadius << "\n";
    os << "  show = " << show << "\n";
    os << "  showContact = " << showContact << "\n";
    os << "  showJointAxes = " << showJointAxes << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  springNumberOfWindings = " << springNumberOfWindings << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  Float4 defaultColor;                            //!< AUTO: default cRGB color for markers; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global marker size; if -1.f, marker size is relative to maxSceneSize
  bool drawSimplified;                            //!< AUTO: draw markers with simplified symbols
  bool show;                                      //!< AUTO: flag to decide, whether the markers are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the marker numbers are shown


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsMarkers()
  {
    defaultColor = Float4({0.1f,0.5f,0.1f,1.f});
    defaultSize = -1.f;
    drawSimplified = true;
    show = true;
    showNumbers = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB color for markers; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB color for markers; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsMarkers" << ":\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  Float4 defaultColor;                            //!< AUTO: default cRGB color for loads; 4th value is alpha-transparency
  float defaultRadius;                            //!< AUTO: global radius of load axis if drawn in 3D
  float defaultSize;                              //!< AUTO: global load size; if -1.f, load size is relative to maxSceneSize
  bool drawSimplified;                            //!< AUTO: draw markers with simplified symbols
  bool fixedLoadSize;                             //!< AUTO: if true, the load is drawn with a fixed vector length in direction of the load vector, independently of the load size
  float loadSizeFactor;                           //!< AUTO: if fixedLoadSize=false, then this scaling factor is used to draw the load vector
  bool show;                                      //!< AUTO: flag to decide, whether the loads are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the load numbers are shown


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsLoads()
  {
    defaultColor = Float4({0.7f,0.1f,0.1f,1.f});
    defaultRadius = 0.005f;
    defaultSize = 0.2f;
    drawSimplified = true;
    fixedLoadSize = true;
    loadSizeFactor = 0.1f;
    show = true;
    showNumbers = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB color for loads; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB color for loads; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsLoads" << ":\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultRadius = " << defaultRadius << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  fixedLoadSize = " << fixedLoadSize << "\n";
    os << "  loadSizeFactor = " << loadSizeFactor << "\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  Float4 defaultColor;                            //!< AUTO: default cRGB color for sensors; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global sensor size; if -1.f, sensor size is relative to maxSceneSize
  bool drawSimplified;                            //!< AUTO: draw sensors with simplified symbols
  bool show;                                      //!< AUTO: flag to decide, whether the sensors are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the sensor numbers are shown


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsSensors()
  {
    defaultColor = Float4({0.6f,0.6f,0.1f,1.f});
    defaultSize = -1.f;
    drawSimplified = true;
    show = true;
    showNumbers = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB color for sensors; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB color for sensors; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsSensors" << ":\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsSensors& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsContact
* @brief        Global visualization settings for GeneralContact. This allows to easily switch on/off during visualization
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-04-07 (last modfied)
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

class VSettingsContact // AUTO: 
{
public: // AUTO: 
  Float4 colorBoundingBoxes;                      //!< AUTO: cRGB color
  Float4 colorSearchTree;                         //!< AUTO: cRGB color
  float contactForcesFactor;                      //!< AUTO: factor used for scaling of contact forces is showContactForces=True
  float contactPointsDefaultSize;                 //!< AUTO: global contact points size; if -1.f, connector size is relative to maxSceneSize; used for some contacts, e.g., in ContactFrictionCircle
  bool showBoundingBoxes;                         //!< AUTO: show bounding boxes of all GeneralContacts
  bool showContactForces;                         //!< AUTO: if True, contact forces are drawn for certain contact models
  bool showContactForcesValues;                   //!< AUTO: if True and showContactForces=True, numerical values for  contact forces are shown at certain points
  bool showSearchTree;                            //!< AUTO: show search tree of all GeneralContacts
  bool showSearchTreeCells;                       //!< AUTO: show cells inside search tree


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsContact()
  {
    colorBoundingBoxes = Float4({0.9f,0.1f,0.1f,1.f});
    colorSearchTree = Float4({0.1f,0.1f,0.9f,1.f});
    contactForcesFactor = 0.001f;
    contactPointsDefaultSize = 0.001f;
    showBoundingBoxes = false;
    showContactForces = false;
    showContactForcesValues = false;
    showSearchTree = false;
    showSearchTreeCells = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: cRGB color
  void PySetColorBoundingBoxes(const std::array<float,4>& colorBoundingBoxesInit) { colorBoundingBoxes = colorBoundingBoxesInit; }
  //! AUTO: Read (Copy) access to: cRGB color
  std::array<float,4> PyGetColorBoundingBoxes() const { return std::array<float,4>(colorBoundingBoxes); }

  //! AUTO: Set function (needed in pybind) for: cRGB color
  void PySetColorSearchTree(const std::array<float,4>& colorSearchTreeInit) { colorSearchTree = colorSearchTreeInit; }
  //! AUTO: Read (Copy) access to: cRGB color
  std::array<float,4> PyGetColorSearchTree() const { return std::array<float,4>(colorSearchTree); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsContact" << ":\n";
    os << "  colorBoundingBoxes = " << colorBoundingBoxes << "\n";
    os << "  colorSearchTree = " << colorSearchTree << "\n";
    os << "  contactForcesFactor = " << contactForcesFactor << "\n";
    os << "  contactPointsDefaultSize = " << contactPointsDefaultSize << "\n";
    os << "  showBoundingBoxes = " << showBoundingBoxes << "\n";
    os << "  showContactForces = " << showContactForces << "\n";
    os << "  showContactForcesValues = " << showContactForcesValues << "\n";
    os << "  showSearchTree = " << showSearchTree << "\n";
    os << "  showSearchTreeCells = " << showSearchTreeCells << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsContact& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsWindow
* @brief        OpenGL Window and interaction settings for visualization; handle changes with care, as they might lead to unexpected results or crashes.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-04-07 (last modfied)
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
  bool alwaysOnTop;                               //!< AUTO: True: OpenGL render window will be always on top of all other windows
  bool ignoreKeys;                                //!< AUTO: True: ignore keyboard input except escape and 'F2' keys; used for interactive mode, e.g., to perform kinematic analysis; This flag can be switched with key 'F2'
  std::function<bool(int, int, int)> keyPressUserFunction;//!< AUTO: add a Python function f(key, action, mods) here, which is called every time a key is pressed; function shall return true, if key has been processed; Example: \tabnewline def f(key, action, mods):\tabnewline \phantom{XXX} print('key=',key);\tabnewline use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
  bool limitWindowToScreenSize;                   //!< AUTO: True: render window size is limited to screen size; False: larger window sizes (e.g. for rendering) allowed according to renderWindowSize
  bool maximize;                                  //!< AUTO: True: OpenGL render window will be maximized at startup
  Index2 renderWindowSize;                        //!< AUTO: initial size of OpenGL render window in pixel
  bool showMouseCoordinates;                      //!< AUTO: True: show OpenGL coordinates and distance to last left mouse button pressed position; switched on/off with key 'F3'
  bool showWindow;                                //!< AUTO: True: OpenGL render window is shown on startup; False: window will be iconified at startup (e.g. if you are starting multiple computations automatically)
  Index startupTimeout;                           //!< AUTO: OpenGL render window startup timeout in ms (change might be necessary if CPU is very slow)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsWindow()
  {
    alwaysOnTop = false;
    ignoreKeys = false;
    keyPressUserFunction = 0;
    limitWindowToScreenSize = true;
    maximize = false;
    renderWindowSize = Index2({1024,768});
    showMouseCoordinates = false;
    showWindow = true;
    startupTimeout = 2500;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: initial size of OpenGL render window in pixel
  void PySetRenderWindowSize(const std::array<Index,2>& renderWindowSizeInit) { renderWindowSize = renderWindowSizeInit; }
  //! AUTO: Read (Copy) access to: initial size of OpenGL render window in pixel
  std::array<Index,2> PyGetRenderWindowSize() const { return std::array<Index,2>(renderWindowSize); }

  //! AUTO: set keyPressUserFunction to zero (no function); because this cannot be assign to the variable itself
  void ResetKeyPressUserFunction() {
    keyPressUserFunction = 0;
  }

  //! AUTO: Set function (needed in pybind) for: OpenGL render window startup timeout in ms (change might be necessary if CPU is very slow)
  void PySetStartupTimeout(const Index& startupTimeoutInit) { startupTimeout = EXUstd::GetSafelyPInt(startupTimeoutInit,"startupTimeout"); }
  //! AUTO: Read (Copy) access to: OpenGL render window startup timeout in ms (change might be necessary if CPU is very slow)
  Index PyGetStartupTimeout() const { return Index(startupTimeout); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsWindow" << ":\n";
    os << "  alwaysOnTop = " << alwaysOnTop << "\n";
    os << "  ignoreKeys = " << ignoreKeys << "\n";
    os << "  limitWindowToScreenSize = " << limitWindowToScreenSize << "\n";
    os << "  maximize = " << maximize << "\n";
    os << "  renderWindowSize = " << renderWindowSize << "\n";
    os << "  showMouseCoordinates = " << showMouseCoordinates << "\n";
    os << "  showWindow = " << showWindow << "\n";
    os << "  startupTimeout = " << startupTimeout << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsWindow& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsDialogs
* @brief        Settings related to dialogs (e.g., visualization settings dialog).
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-04-07 (last modfied)
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

class VSettingsDialogs // AUTO: 
{
public: // AUTO: 
  float alphaTransparency;                        //!< AUTO: alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)
  bool alwaysTopmost;                             //!< AUTO: True: dialogs are always topmost (otherwise, they are sometimes hidden)
  float fontScalingMacOS;                         //!< AUTO: font scaling value for MacOS systems (on Windows, system display scaling is used)
  bool multiThreadedDialogs;                      //!< AUTO: True: During dialogs, the OpenGL render window will still get updates of changes in dialogs, etc., which may cause problems on some platforms or for some (complicated) models; False: changes of dialogs will take effect when dialogs are closed
  bool openTreeView;                              //!< AUTO: True: all sub-trees of the visusalization dialog are opened when opening the dialog; False: only some sub-trees are opened


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsDialogs()
  {
    alphaTransparency = 0.94f;
    alwaysTopmost = true;
    fontScalingMacOS = 1.35f;
    multiThreadedDialogs = true;
    openTreeView = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)
  void PySetAlphaTransparency(const float& alphaTransparencyInit) { alphaTransparency = EXUstd::GetSafelyUFloat(alphaTransparencyInit,"alphaTransparency"); }
  //! AUTO: Read (Copy) access to: alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)
  float PyGetAlphaTransparency() const { return float(alphaTransparency); }

  //! AUTO: Set function (needed in pybind) for: font scaling value for MacOS systems (on Windows, system display scaling is used)
  void PySetFontScalingMacOS(const float& fontScalingMacOSInit) { fontScalingMacOS = EXUstd::GetSafelyUFloat(fontScalingMacOSInit,"fontScalingMacOS"); }
  //! AUTO: Read (Copy) access to: font scaling value for MacOS systems (on Windows, system display scaling is used)
  float PyGetFontScalingMacOS() const { return float(fontScalingMacOS); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsDialogs" << ":\n";
    os << "  alphaTransparency = " << alphaTransparency << "\n";
    os << "  alwaysTopmost = " << alwaysTopmost << "\n";
    os << "  fontScalingMacOS = " << fontScalingMacOS << "\n";
    os << "  multiThreadedDialogs = " << multiThreadedDialogs << "\n";
    os << "  openTreeView = " << openTreeView << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsDialogs& object)
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  bool drawFaceNormals;                           //!< AUTO: draws triangle normals, e.g. at center of triangles; used for debugging of faces
  float drawNormalsLength;                        //!< AUTO: length of normals; used for debugging
  bool drawVertexNormals;                         //!< AUTO: draws vertex normals; used for debugging
  bool enableLight0;                              //!< AUTO: turn on/off light0
  bool enableLight1;                              //!< AUTO: turn on/off light1
  bool enableLighting;                            //!< AUTO: generally enable lighting (otherwise, colors of objects are used); OpenGL: glEnable(GL_LIGHTING)
  Float4 faceEdgesColor;                          //!< AUTO: global RGBA color for face edges
  bool facesTransparent;                          //!< AUTO: True: show faces transparent independent of transparency (A)-value in color of objects; allow to show otherwise hidden node/marker/object numbers
  Float3 initialCenterPoint;                      //!< AUTO: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  float initialMaxSceneSize;                      //!< AUTO: initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  StdArray33F initialModelRotation;               //!< AUTO: initial model rotation matrix for OpenGl; in python use e.g.: initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]
  float initialZoom;                              //!< AUTO: initial zoom of scene; overwritten/ignored if autoFitScene = True
  float light0ambient;                            //!< AUTO: ambient value of GL_LIGHT0
  float light0constantAttenuation;                //!< AUTO: constant attenuation coefficient of GL_LIGHT0, this is a constant factor that attenuates the light source; attenuation factor = 1/(kx +kl*d + kq*d*d); (kc,kl,kq)=(1,0,0) means no attenuation; only used for lights, where last component of light position is 1
  float light0diffuse;                            //!< AUTO: diffuse value of GL_LIGHT0
  float light0linearAttenuation;                  //!< AUTO: linear attenuation coefficient of GL_LIGHT0, this is a linear factor for attenuation of the light source with distance
  Float4 light0position;                          //!< AUTO: 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position; see opengl manuals
  float light0quadraticAttenuation;               //!< AUTO: quadratic attenuation coefficient of GL_LIGHT0, this is a quadratic factor for attenuation of the light source with distance
  float light0specular;                           //!< AUTO: specular value of GL_LIGHT0
  float light1ambient;                            //!< AUTO: ambient value of GL_LIGHT1
  float light1constantAttenuation;                //!< AUTO: constant attenuation coefficient of GL_LIGHT1, this is a constant factor that attenuates the light source; attenuation factor = 1/(kx +kl*d + kq*d*d); only used for lights, where last component of light position is 1
  float light1diffuse;                            //!< AUTO: diffuse value of GL_LIGHT1
  float light1linearAttenuation;                  //!< AUTO: linear attenuation coefficient of GL_LIGHT1, this is a linear factor for attenuation of the light source with distance
  Float4 light1position;                          //!< AUTO: 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  float light1quadraticAttenuation;               //!< AUTO: quadratic attenuation coefficient of GL_LIGHT1, this is a quadratic factor for attenuation of the light source with distance
  float light1specular;                           //!< AUTO: specular value of GL_LIGHT1
  Float4 lightModelAmbient;                       //!< AUTO: global ambient light; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a])
  bool lightModelLocalViewer;                     //!< AUTO: select local viewer for light; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,...)
  bool lightModelTwoSide;                         //!< AUTO: enlighten also backside of object; may cause problems on some graphics cards and lead to slower performance; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,...)
  bool lineSmooth;                                //!< AUTO: draw lines smooth
  float lineWidth;                                //!< AUTO: width of lines used for representation of lines, circles, points, etc.
  Float4 materialAmbientAndDiffuse;               //!< AUTO: 4f ambient color of material
  float materialShininess;                        //!< AUTO: shininess of material
  Float4 materialSpecular;                        //!< AUTO: 4f specular color of material
  Index multiSampling;                            //!< AUTO: NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  float perspective;                              //!< AUTO: parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 0.5 (large amount of perspective); mouse coordinates will not work with perspective
  float polygonOffset;                            //!< AUTO: general polygon offset for polygons, except for shadows; use this parameter to draw polygons behind lines to reduce artifacts for very large or small models
  bool shadeModelSmooth;                          //!< AUTO: True: turn on smoothing for shaders, which uses vertex normals to smooth surfaces
  float shadow;                                   //!< AUTO: parameter \f$\in [0 ... 1]\f$ prescribes amount of shadow for light0 (using light0position, etc.); if this parameter is different from 1, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows
  float shadowPolygonOffset;                      //!< AUTO: some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
  bool showFaceEdges;                             //!< AUTO: show edges of faces; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
  bool showFaces;                                 //!< AUTO: show faces of triangles, etc.; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
  bool showLines;                                 //!< AUTO: show lines (different from edges of faces)
  bool showMeshEdges;                             //!< AUTO: show edges of finite elements; independent of showFaceEdges
  bool showMeshFaces;                             //!< AUTO: show faces of finite elements; independent of showFaces
  bool textLineSmooth;                            //!< AUTO: draw lines for representation of text smooth
  float textLineWidth;                            //!< AUTO: width of lines used for representation of text


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsOpenGL()
  {
    drawFaceNormals = false;
    drawNormalsLength = 0.1f;
    drawVertexNormals = false;
    enableLight0 = true;
    enableLight1 = true;
    enableLighting = true;
    faceEdgesColor = Float4({0.2f,0.2f,0.2f,1.f});
    facesTransparent = false;
    initialCenterPoint = Float3({0.f,0.f,0.f});
    initialMaxSceneSize = 1.f;
    initialModelRotation = EXUmath::Matrix3DFToStdArray33(Matrix3DF(3,3,{1.f,0.f,0.f, 0.f,1.f,0.f, 0.f,0.f,1.f}));
    initialZoom = 1.f;
    light0ambient = 0.3f;
    light0constantAttenuation = 1.0f;
    light0diffuse = 0.6f;
    light0linearAttenuation = 0.0f;
    light0position = Float4({0.2f,0.2f,10.f,0.f});
    light0quadraticAttenuation = 0.0f;
    light0specular = 0.5f;
    light1ambient = 0.0f ;
    light1constantAttenuation = 1.0f;
    light1diffuse = 0.5f;
    light1linearAttenuation = 0.0f;
    light1position = Float4({1.f,1.f,-10.f,0.f});
    light1quadraticAttenuation = 0.0f;
    light1specular = 0.6f;
    lightModelAmbient = Float4({0.f,0.f,0.f,1.f});
    lightModelLocalViewer = false;
    lightModelTwoSide = false;
    lineSmooth = true;
    lineWidth = 1.f;
    materialAmbientAndDiffuse = Float4({0.6f,0.6f,0.6f,1.f});
    materialShininess = 32.f;
    materialSpecular = Float4({0.6f,0.6f,0.6f,1.f});
    multiSampling = 1;
    perspective = 0.f;
    polygonOffset = 0.01f;
    shadeModelSmooth = true;
    shadow = 0.f;
    shadowPolygonOffset = 0.1f;
    showFaceEdges = false;
    showFaces = true;
    showLines = true;
    showMeshEdges = true;
    showMeshFaces = true;
    textLineSmooth = false;
    textLineWidth = 1.f;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: length of normals; used for debugging
  void PySetDrawNormalsLength(const float& drawNormalsLengthInit) { drawNormalsLength = EXUstd::GetSafelyPFloat(drawNormalsLengthInit,"drawNormalsLength"); }
  //! AUTO: Read (Copy) access to: length of normals; used for debugging
  float PyGetDrawNormalsLength() const { return float(drawNormalsLength); }

  //! AUTO: Set function (needed in pybind) for: global RGBA color for face edges
  void PySetFaceEdgesColor(const std::array<float,4>& faceEdgesColorInit) { faceEdgesColor = faceEdgesColorInit; }
  //! AUTO: Read (Copy) access to: global RGBA color for face edges
  std::array<float,4> PyGetFaceEdgesColor() const { return std::array<float,4>(faceEdgesColor); }

  //! AUTO: Set function (needed in pybind) for: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  void PySetInitialCenterPoint(const std::array<float,3>& initialCenterPointInit) { initialCenterPoint = initialCenterPointInit; }
  //! AUTO: Read (Copy) access to: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  std::array<float,3> PyGetInitialCenterPoint() const { return std::array<float,3>(initialCenterPoint); }

  //! AUTO: Set function (needed in pybind) for: initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  void PySetInitialMaxSceneSize(const float& initialMaxSceneSizeInit) { initialMaxSceneSize = EXUstd::GetSafelyPFloat(initialMaxSceneSizeInit,"initialMaxSceneSize"); }
  //! AUTO: Read (Copy) access to: initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  float PyGetInitialMaxSceneSize() const { return float(initialMaxSceneSize); }

  //! AUTO: Set function (needed in pybind) for: initial zoom of scene; overwritten/ignored if autoFitScene = True
  void PySetInitialZoom(const float& initialZoomInit) { initialZoom = EXUstd::GetSafelyUFloat(initialZoomInit,"initialZoom"); }
  //! AUTO: Read (Copy) access to: initial zoom of scene; overwritten/ignored if autoFitScene = True
  float PyGetInitialZoom() const { return float(initialZoom); }

  //! AUTO: Set function (needed in pybind) for: 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position; see opengl manuals
  void PySetLight0position(const std::array<float,4>& light0positionInit) { light0position = light0positionInit; }
  //! AUTO: Read (Copy) access to: 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position; see opengl manuals
  std::array<float,4> PyGetLight0position() const { return std::array<float,4>(light0position); }

  //! AUTO: Set function (needed in pybind) for: 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  void PySetLight1position(const std::array<float,4>& light1positionInit) { light1position = light1positionInit; }
  //! AUTO: Read (Copy) access to: 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
  std::array<float,4> PyGetLight1position() const { return std::array<float,4>(light1position); }

  //! AUTO: Set function (needed in pybind) for: global ambient light; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a])
  void PySetLightModelAmbient(const std::array<float,4>& lightModelAmbientInit) { lightModelAmbient = lightModelAmbientInit; }
  //! AUTO: Read (Copy) access to: global ambient light; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a])
  std::array<float,4> PyGetLightModelAmbient() const { return std::array<float,4>(lightModelAmbient); }

  //! AUTO: Set function (needed in pybind) for: width of lines used for representation of lines, circles, points, etc.
  void PySetLineWidth(const float& lineWidthInit) { lineWidth = EXUstd::GetSafelyUFloat(lineWidthInit,"lineWidth"); }
  //! AUTO: Read (Copy) access to: width of lines used for representation of lines, circles, points, etc.
  float PyGetLineWidth() const { return float(lineWidth); }

  //! AUTO: Set function (needed in pybind) for: 4f ambient color of material
  void PySetMaterialAmbientAndDiffuse(const std::array<float,4>& materialAmbientAndDiffuseInit) { materialAmbientAndDiffuse = materialAmbientAndDiffuseInit; }
  //! AUTO: Read (Copy) access to: 4f ambient color of material
  std::array<float,4> PyGetMaterialAmbientAndDiffuse() const { return std::array<float,4>(materialAmbientAndDiffuse); }

  //! AUTO: Set function (needed in pybind) for: 4f specular color of material
  void PySetMaterialSpecular(const std::array<float,4>& materialSpecularInit) { materialSpecular = materialSpecularInit; }
  //! AUTO: Read (Copy) access to: 4f specular color of material
  std::array<float,4> PyGetMaterialSpecular() const { return std::array<float,4>(materialSpecular); }

  //! AUTO: Set function (needed in pybind) for: NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  void PySetMultiSampling(const Index& multiSamplingInit) { multiSampling = EXUstd::GetSafelyPInt(multiSamplingInit,"multiSampling"); }
  //! AUTO: Read (Copy) access to: NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  Index PyGetMultiSampling() const { return Index(multiSampling); }

  //! AUTO: Set function (needed in pybind) for: parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 0.5 (large amount of perspective); mouse coordinates will not work with perspective
  void PySetPerspective(const float& perspectiveInit) { perspective = EXUstd::GetSafelyUFloat(perspectiveInit,"perspective"); }
  //! AUTO: Read (Copy) access to: parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 0.5 (large amount of perspective); mouse coordinates will not work with perspective
  float PyGetPerspective() const { return float(perspective); }

  //! AUTO: Set function (needed in pybind) for: parameter \f$\in [0 ... 1]\f$ prescribes amount of shadow for light0 (using light0position, etc.); if this parameter is different from 1, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows
  void PySetShadow(const float& shadowInit) { shadow = EXUstd::GetSafelyUFloat(shadowInit,"shadow"); }
  //! AUTO: Read (Copy) access to: parameter \f$\in [0 ... 1]\f$ prescribes amount of shadow for light0 (using light0position, etc.); if this parameter is different from 1, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows
  float PyGetShadow() const { return float(shadow); }

  //! AUTO: Set function (needed in pybind) for: some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
  void PySetShadowPolygonOffset(const float& shadowPolygonOffsetInit) { shadowPolygonOffset = EXUstd::GetSafelyPFloat(shadowPolygonOffsetInit,"shadowPolygonOffset"); }
  //! AUTO: Read (Copy) access to: some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
  float PyGetShadowPolygonOffset() const { return float(shadowPolygonOffset); }

  //! AUTO: Set function (needed in pybind) for: width of lines used for representation of text
  void PySetTextLineWidth(const float& textLineWidthInit) { textLineWidth = EXUstd::GetSafelyUFloat(textLineWidthInit,"textLineWidth"); }
  //! AUTO: Read (Copy) access to: width of lines used for representation of text
  float PyGetTextLineWidth() const { return float(textLineWidth); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsOpenGL" << ":\n";
    os << "  drawFaceNormals = " << drawFaceNormals << "\n";
    os << "  drawNormalsLength = " << drawNormalsLength << "\n";
    os << "  drawVertexNormals = " << drawVertexNormals << "\n";
    os << "  enableLight0 = " << enableLight0 << "\n";
    os << "  enableLight1 = " << enableLight1 << "\n";
    os << "  enableLighting = " << enableLighting << "\n";
    os << "  faceEdgesColor = " << faceEdgesColor << "\n";
    os << "  facesTransparent = " << facesTransparent << "\n";
    os << "  initialCenterPoint = " << initialCenterPoint << "\n";
    os << "  initialMaxSceneSize = " << initialMaxSceneSize << "\n";
#ifndef __APPLE__
    os << "  initialModelRotation = " << Matrix3DF(initialModelRotation) << "\n";
#endif
    os << "  initialZoom = " << initialZoom << "\n";
    os << "  light0ambient = " << light0ambient << "\n";
    os << "  light0constantAttenuation = " << light0constantAttenuation << "\n";
    os << "  light0diffuse = " << light0diffuse << "\n";
    os << "  light0linearAttenuation = " << light0linearAttenuation << "\n";
    os << "  light0position = " << light0position << "\n";
    os << "  light0quadraticAttenuation = " << light0quadraticAttenuation << "\n";
    os << "  light0specular = " << light0specular << "\n";
    os << "  light1ambient = " << light1ambient << "\n";
    os << "  light1constantAttenuation = " << light1constantAttenuation << "\n";
    os << "  light1diffuse = " << light1diffuse << "\n";
    os << "  light1linearAttenuation = " << light1linearAttenuation << "\n";
    os << "  light1position = " << light1position << "\n";
    os << "  light1quadraticAttenuation = " << light1quadraticAttenuation << "\n";
    os << "  light1specular = " << light1specular << "\n";
    os << "  lightModelAmbient = " << lightModelAmbient << "\n";
    os << "  lightModelLocalViewer = " << lightModelLocalViewer << "\n";
    os << "  lightModelTwoSide = " << lightModelTwoSide << "\n";
    os << "  lineSmooth = " << lineSmooth << "\n";
    os << "  lineWidth = " << lineWidth << "\n";
    os << "  materialAmbientAndDiffuse = " << materialAmbientAndDiffuse << "\n";
    os << "  materialShininess = " << materialShininess << "\n";
    os << "  materialSpecular = " << materialSpecular << "\n";
    os << "  multiSampling = " << multiSampling << "\n";
    os << "  perspective = " << perspective << "\n";
    os << "  polygonOffset = " << polygonOffset << "\n";
    os << "  shadeModelSmooth = " << shadeModelSmooth << "\n";
    os << "  shadow = " << shadow << "\n";
    os << "  shadowPolygonOffset = " << shadowPolygonOffset << "\n";
    os << "  showFaceEdges = " << showFaceEdges << "\n";
    os << "  showFaces = " << showFaces << "\n";
    os << "  showLines = " << showLines << "\n";
    os << "  showMeshEdges = " << showMeshEdges << "\n";
    os << "  showMeshFaces = " << showMeshFaces << "\n";
    os << "  textLineSmooth = " << textLineSmooth << "\n";
    os << "  textLineWidth = " << textLineWidth << "\n";
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
* @brief        Functionality to export images to files (PNG or TGA format) which can be used to create animations; to activate image recording during the solution process, set SolutionSettings.recordImagesInterval accordingly.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-04-07 (last modfied)
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
  Index heightAlignment;                          //!< AUTO: alignment of exported image height; using a value of 2 helps to reduce problems with video conversion (additional horizontal lines are lost)
  bool saveImageAsTextCircles;                    //!< AUTO: export circles in save image (only in TXT format)
  bool saveImageAsTextLines;                      //!< AUTO: export lines in save image (only in TXT format)
  bool saveImageAsTextTexts;                      //!< AUTO: export text in save image (only in TXT format)
  bool saveImageAsTextTriangles;                  //!< AUTO: export triangles in save image (only in TXT format)
  Index saveImageFileCounter;                     //!< AUTO: current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  std::string saveImageFileName;                  //!< AUTO: filename (without extension!) and (relative) path for image file(s) with consecutive numbering (e.g., frame0000.png, frame0001.png,...); ; directory will be created if it does not exist
  std::string saveImageFormat;                    //!< AUTO: format for exporting figures: currently only PNG, TGA and TXT available; while PNG and TGA represent the according image file formats, the TXT format results in a text file containing the 3D graphics data information as lists of lines, triangles, etc; PNG is not available for Ubuntu18.04 (check  use TGA has highest compatibility with all platforms
  bool saveImageSingleFile;                       //!< AUTO: True: only save single files with given filename, not adding numbering; False: add numbering to files, see saveImageFileName
  Index saveImageTimeOut;                         //!< AUTO: timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  Index widthAlignment;                           //!< AUTO: alignment of exported image width; using a value of 4 helps to reduce problems with video conversion (additional vertical lines are lost)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsExportImages()
  {
    heightAlignment = 2;
    saveImageAsTextCircles = true;
    saveImageAsTextLines = true;
    saveImageAsTextTexts = false;
    saveImageAsTextTriangles = false;
    saveImageFileCounter = 0;
    saveImageFileName = "images/frame";
    saveImageFormat = "PNG";
    saveImageSingleFile = false;
    saveImageTimeOut = 5000;
    widthAlignment = 4;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: alignment of exported image height; using a value of 2 helps to reduce problems with video conversion (additional horizontal lines are lost)
  void PySetHeightAlignment(const Index& heightAlignmentInit) { heightAlignment = EXUstd::GetSafelyPInt(heightAlignmentInit,"heightAlignment"); }
  //! AUTO: Read (Copy) access to: alignment of exported image height; using a value of 2 helps to reduce problems with video conversion (additional horizontal lines are lost)
  Index PyGetHeightAlignment() const { return Index(heightAlignment); }

  //! AUTO: Set function (needed in pybind) for: current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  void PySetSaveImageFileCounter(const Index& saveImageFileCounterInit) { saveImageFileCounter = EXUstd::GetSafelyUInt(saveImageFileCounterInit,"saveImageFileCounter"); }
  //! AUTO: Read (Copy) access to: current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  Index PyGetSaveImageFileCounter() const { return Index(saveImageFileCounter); }

  //! AUTO: Set function (needed in pybind) for: timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  void PySetSaveImageTimeOut(const Index& saveImageTimeOutInit) { saveImageTimeOut = EXUstd::GetSafelyPInt(saveImageTimeOutInit,"saveImageTimeOut"); }
  //! AUTO: Read (Copy) access to: timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  Index PyGetSaveImageTimeOut() const { return Index(saveImageTimeOut); }

  //! AUTO: Set function (needed in pybind) for: alignment of exported image width; using a value of 4 helps to reduce problems with video conversion (additional vertical lines are lost)
  void PySetWidthAlignment(const Index& widthAlignmentInit) { widthAlignment = EXUstd::GetSafelyPInt(widthAlignmentInit,"widthAlignment"); }
  //! AUTO: Read (Copy) access to: alignment of exported image width; using a value of 4 helps to reduce problems with video conversion (additional vertical lines are lost)
  Index PyGetWidthAlignment() const { return Index(widthAlignment); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsExportImages" << ":\n";
    os << "  heightAlignment = " << heightAlignment << "\n";
    os << "  saveImageAsTextCircles = " << saveImageAsTextCircles << "\n";
    os << "  saveImageAsTextLines = " << saveImageAsTextLines << "\n";
    os << "  saveImageAsTextTexts = " << saveImageAsTextTexts << "\n";
    os << "  saveImageAsTextTriangles = " << saveImageAsTextTriangles << "\n";
    os << "  saveImageFileCounter = " << saveImageFileCounter << "\n";
    os << "  saveImageFileName = " << saveImageFileName << "\n";
    os << "  saveImageFormat = " << saveImageFormat << "\n";
    os << "  saveImageSingleFile = " << saveImageSingleFile << "\n";
    os << "  saveImageTimeOut = " << saveImageTimeOut << "\n";
    os << "  widthAlignment = " << widthAlignment << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsExportImages& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsOpenVR
* @brief        Functionality to interact openVR; requires special hardware or software emulator, see steam / openVR descriptions
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2023-04-07 (last modfied)
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

class VSettingsOpenVR // AUTO: 
{
public: // AUTO: 
  std::string actionManifestFileName;             //!< AUTO: This string must contain a string representing a valid absolute path to a vr_actions.json manifest, which describes all HMD, tracker, etc. devices as given by openVR
  bool enable;                                    //!< AUTO: True: openVR enabled (if compiled with according flag and installed openVR)
  Index logLevel;                                 //!< AUTO: integer value setting log level of openVR: -1 (no output), 0 (error), 1 (warning), 2 (info), 3 (debug); increase log level to get more output
  bool showCompanionWindow;                       //!< AUTO: True: openVR will show companion window containing left and right eye view


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsOpenVR()
  {
    actionManifestFileName = "C:/openVRactionsManifest.json";
    enable = false;
    logLevel = 1;
    showCompanionWindow = true;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsOpenVR" << ":\n";
    os << "  actionManifestFileName = " << actionManifestFileName << "\n";
    os << "  enable = " << enable << "\n";
    os << "  logLevel = " << logLevel << "\n";
    os << "  showCompanionWindow = " << showCompanionWindow << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsOpenVR& object)
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  VSettingsOpenVR openVR;                         //!< AUTO: openVR visualization settings
  Float4 highlightColor;                          //!< AUTO: cRGB color for highlighted item; 4th value is alpha-transparency
  Index highlightItemIndex;                       //!< AUTO: index of item that shall be highlighted (e.g., need to find item due to errors); if set -1, no item is highlighted
  ItemType highlightItemType;                     //!< AUTO: item type (Node, Object, ...) that shall be highlighted (e.g., need to find item due to errors)
  Index highlightMbsNumber;                       //!< AUTO: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  Float4 highlightOtherColor;                     //!< AUTO: cRGB color for other items (which are not highlighted); 4th value is alpha-transparency
  float joystickScaleRotation;                    //!< AUTO: rotation scaling factor for joystick input
  float joystickScaleTranslation;                 //!< AUTO: translation scaling factor for joystick input
  float keypressRotationStep;                     //!< AUTO: rotation increment per keypress in degree (full rotation = 360 degree)
  float keypressTranslationStep;                  //!< AUTO: translation increment per keypress relative to window size
  bool lockModelView;                             //!< AUTO: True: all movements (with mouse/keys), rotations, zoom are disabled; initial values are considered ==> initial zoom, rotation and center point need to be adjusted, approx. 0.4*maxSceneSize is a good value
  float mouseMoveRotationFactor;                  //!< AUTO: rotation increment per 1 pixel mouse movement in degree
  bool pauseWithSpacebar;                         //!< AUTO: True: during simulation, space bar can be pressed to pause simulation
  bool selectionHighlights;                       //!< AUTO: True: mouse click highlights item (default: red)
  bool selectionLeftMouse;                        //!< AUTO: True: left mouse click on items and show basic information
  bool selectionRightMouse;                       //!< AUTO: True: right mouse click on items and show dictionary (read only!)
  bool selectionRightMouseGraphicsData;           //!< AUTO: True: right mouse click on items also shows GraphicsData information for inspectation (may sometimes be very large and may not fit into dialog for large graphics objects!)
  Index trackMarker;                              //!< AUTO: if valid marker index is provided and marker provides position (and orientation), the centerpoint of the scene follows the marker (and orientation); depends on trackMarkerPosition and trackMarkerOrientation; by default, only position is tracked
  Index trackMarkerMbsNumber;                     //!< AUTO: number of main system which is used to track marker; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number
  Float3 trackMarkerOrientation;                  //!< AUTO: choose which orientation axes (x,y,z) are tracked; currently can only be all zero or all one
  Float3 trackMarkerPosition;                     //!< AUTO: choose which coordinates or marker are tracked (x,y,z)
  bool useJoystickInput;                          //!< AUTO: True: read joystick input (use 6-axis joystick with lowest ID found when starting renderer window) and interpret as (x,y,z) position and (rotx, roty, rotz) rotation: as available from 3Dconnexion space mouse and maybe others as well; set to False, if external joystick makes problems ...
  float zoomStepFactor;                           //!< AUTO: change of zoom per keypress (keypad +/-) or mouse wheel increment


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsInteractive()
  {
    highlightColor = Float4({0.8f,0.05f,0.05f,0.75f});
    highlightItemIndex = -1;
    highlightItemType = ItemType::_None;
    highlightMbsNumber = 0;
    highlightOtherColor = Float4({0.5f,0.5f,0.5f,0.4f});
    joystickScaleRotation = 200.f;
    joystickScaleTranslation = 6.f;
    keypressRotationStep = 5.f;
    keypressTranslationStep = 0.1f;
    lockModelView = false;
    mouseMoveRotationFactor = 1.f;
    pauseWithSpacebar = true;
    selectionHighlights = true;
    selectionLeftMouse = true;
    selectionRightMouse = true;
    selectionRightMouseGraphicsData = false;
    trackMarker = -1;
    trackMarkerMbsNumber = 0;
    trackMarkerOrientation = Float3({0.f,0.f,0.f});
    trackMarkerPosition = Float3({1.f,1.f,1.f});
    useJoystickInput = true;
    zoomStepFactor = 1.15f;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: cRGB color for highlighted item; 4th value is alpha-transparency
  void PySetHighlightColor(const std::array<float,4>& highlightColorInit) { highlightColor = highlightColorInit; }
  //! AUTO: Read (Copy) access to: cRGB color for highlighted item; 4th value is alpha-transparency
  std::array<float,4> PyGetHighlightColor() const { return std::array<float,4>(highlightColor); }

  //! AUTO: Set function (needed in pybind) for: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  void PySetHighlightMbsNumber(const Index& highlightMbsNumberInit) { highlightMbsNumber = EXUstd::GetSafelyUInt(highlightMbsNumberInit,"highlightMbsNumber"); }
  //! AUTO: Read (Copy) access to: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  Index PyGetHighlightMbsNumber() const { return Index(highlightMbsNumber); }

  //! AUTO: Set function (needed in pybind) for: cRGB color for other items (which are not highlighted); 4th value is alpha-transparency
  void PySetHighlightOtherColor(const std::array<float,4>& highlightOtherColorInit) { highlightOtherColor = highlightOtherColorInit; }
  //! AUTO: Read (Copy) access to: cRGB color for other items (which are not highlighted); 4th value is alpha-transparency
  std::array<float,4> PyGetHighlightOtherColor() const { return std::array<float,4>(highlightOtherColor); }

  //! AUTO: Set function (needed in pybind) for: choose which orientation axes (x,y,z) are tracked; currently can only be all zero or all one
  void PySetTrackMarkerOrientation(const std::array<float,3>& trackMarkerOrientationInit) { trackMarkerOrientation = trackMarkerOrientationInit; }
  //! AUTO: Read (Copy) access to: choose which orientation axes (x,y,z) are tracked; currently can only be all zero or all one
  std::array<float,3> PyGetTrackMarkerOrientation() const { return std::array<float,3>(trackMarkerOrientation); }

  //! AUTO: Set function (needed in pybind) for: choose which coordinates or marker are tracked (x,y,z)
  void PySetTrackMarkerPosition(const std::array<float,3>& trackMarkerPositionInit) { trackMarkerPosition = trackMarkerPositionInit; }
  //! AUTO: Read (Copy) access to: choose which coordinates or marker are tracked (x,y,z)
  std::array<float,3> PyGetTrackMarkerPosition() const { return std::array<float,3>(trackMarkerPosition); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsInteractive" << ":\n";
    os << "  openVR = " << openVR << "\n";
    os << "  highlightColor = " << highlightColor << "\n";
    os << "  highlightItemIndex = " << highlightItemIndex << "\n";
    os << "  highlightItemType = " << highlightItemType << "\n";
    os << "  highlightMbsNumber = " << highlightMbsNumber << "\n";
    os << "  highlightOtherColor = " << highlightOtherColor << "\n";
    os << "  joystickScaleRotation = " << joystickScaleRotation << "\n";
    os << "  joystickScaleTranslation = " << joystickScaleTranslation << "\n";
    os << "  keypressRotationStep = " << keypressRotationStep << "\n";
    os << "  keypressTranslationStep = " << keypressTranslationStep << "\n";
    os << "  lockModelView = " << lockModelView << "\n";
    os << "  mouseMoveRotationFactor = " << mouseMoveRotationFactor << "\n";
    os << "  pauseWithSpacebar = " << pauseWithSpacebar << "\n";
    os << "  selectionHighlights = " << selectionHighlights << "\n";
    os << "  selectionLeftMouse = " << selectionLeftMouse << "\n";
    os << "  selectionRightMouse = " << selectionRightMouse << "\n";
    os << "  selectionRightMouseGraphicsData = " << selectionRightMouseGraphicsData << "\n";
    os << "  trackMarker = " << trackMarker << "\n";
    os << "  trackMarkerMbsNumber = " << trackMarkerMbsNumber << "\n";
    os << "  trackMarkerOrientation = " << trackMarkerOrientation << "\n";
    os << "  trackMarkerPosition = " << trackMarkerPosition << "\n";
    os << "  useJoystickInput = " << useJoystickInput << "\n";
    os << "  zoomStepFactor = " << zoomStepFactor << "\n";
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
* @date         AUTO: 2023-04-07 (last modfied)
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
  VSettingsBodies bodies;                         //!< AUTO: body visualization settings
  VSettingsConnectors connectors;                 //!< AUTO: connector visualization settings
  VSettingsContact contact;                       //!< AUTO: contact visualization settings
  VSettingsContour contour;                       //!< AUTO: contour plot visualization settings
  VSettingsDialogs dialogs;                       //!< AUTO: dialogs settings
  VSettingsExportImages exportImages;             //!< AUTO: settings for exporting (saving) images to files in order to create animations
  VSettingsGeneral general;                       //!< AUTO: general visualization settings
  VSettingsInteractive interactive;               //!< AUTO: Settings for interaction with renderer
  VSettingsLoads loads;                           //!< AUTO: load visualization settings
  VSettingsMarkers markers;                       //!< AUTO: marker visualization settings
  VSettingsNodes nodes;                           //!< AUTO: node visualization settings
  VSettingsOpenGL openGL;                         //!< AUTO: OpenGL rendering settings
  VSettingsSensors sensors;                       //!< AUTO: sensor visualization settings
  VSettingsWindow window;                         //!< AUTO: visualization window and interaction settings


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VisualizationSettings" << ":\n";
    os << "  bodies = " << bodies << "\n";
    os << "  connectors = " << connectors << "\n";
    os << "  contact = " << contact << "\n";
    os << "  contour = " << contour << "\n";
    os << "  dialogs = " << dialogs << "\n";
    os << "  exportImages = " << exportImages << "\n";
    os << "  general = " << general << "\n";
    os << "  interactive = " << interactive << "\n";
    os << "  loads = " << loads << "\n";
    os << "  markers = " << markers << "\n";
    os << "  nodes = " << nodes << "\n";
    os << "  openGL = " << openGL << "\n";
    os << "  sensors = " << sensors << "\n";
    os << "  window = " << window << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VisualizationSettings& object)
  {
    object.Print(os);
    return os;
  }

};



#endif //#ifdef include once...
