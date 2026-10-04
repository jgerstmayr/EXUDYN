/** ***********************************************************************************************
* @class        VSettingsGeneral
* @brief        General settings for visualization that influence all windows, default values, autofit, multithreading, etc.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
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

class VisualizationSettings; //! AUTO: forward declaration for backlink

class VSettingsGeneral // AUTO: 
{
public: // AUTO: 
  bool autoFitScene;                              //!< AUTO: automatically fit scene within startup after SC.renderer.Start()
  Index axesTiling;                               //!< AUTO: must be > 0; global number of segments for drawing cylinders for axes and cones for arrows (reduce this number, e.g. to 4, if many axes are drawn)
  Float4 backgroundColor;                         //!< AUTO: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  Float4 backgroundColorBottom;                   //!< AUTO: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  float boundingBoxZoomAllFactor;                 //!< AUTO: must be > 0; factor on boundingBox for zoom all (without minimum offset)
  float boundingBoxZoomAllOffset;                 //!< AUTO: must be >= 0; minimum offset to bounding box of scene in window - width or height, whatever is smaller; adjust for very small or large scenes; may be negative
  Index circleTiling;                             //!< AUTO: must be > 0; global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  float coordinateSystemSize;                     //!< AUTO: must be > 0; size of coordinate system relative to font size
  Index cylinderTiling;                           //!< AUTO: must be > 0; global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  float graphicsUpdateInterval;                   //!< AUTO: must be >= 0; interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
  bool limitWindowToScreenSize;                   //!< AUTO: True: size for render window of respective view is limited to screen size; False: larger window sizes (e.g. for rendering) allowed according to renderWindowSize
  float linuxDisplayScaleFactor;                  //!< AUTO: must be > 0; Scaling factor for linux, which cannot determined from system by now; adjust this value to scale dialog fonts and renderer fonts
  float minSceneSize;                             //!< AUTO: must be > 0; minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
  float pointSize;                                //!< AUTO: must be > 0; global point size (absolute)
  Real reallyQuitTimeLimit;                       //!< AUTO: must be >= 0; number of seconds after which user is asked a security question before stopping simulation and closing renderer; set to 0 in order to always get asked; set to 1e10 to (nearly) never get asked
  Index rendererPrecision;                        //!< AUTO: must be > 0; precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
  Index rendererStartupTimeout;                   //!< AUTO: must be > 0; OpenGL render windows startup timeout in ms (change might be necessary if CPU is very slow)
  std::string renderWindowString;                 //!< AUTO: string shown in render window (use this, e.g., for debugging, etc.; written below EXUDYN, similar to information in simulationSettings.solution.file)
  Index showHelpOnStartup;                        //!< AUTO: must be >= 0; seconds to show help message on startup (0=deactivate)
  bool showSolutionInformation;                   //!< AUTO: true = show solution information (from simulationSettings.solution)
  bool showSolverInformation;                     //!< AUTO: true = solver name and further information shown in render window
  bool showSolverTime;                            //!< AUTO: true = solver current time shown in render window
  Index sphereTiling;                             //!< AUTO: must be > 0; global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  bool textAlwaysInFront;                         //!< AUTO: if true, text for item numbers and other item-related text is drawn in front; this may be unwanted in case that you only with to see numbers of objects in front; currently does not work with perspective
  Float4 textColor;                               //!< AUTO: general text color (default); used for system texts in render window
  bool textHasBackground;                         //!< AUTO: if true, text for item numbers and other item-related text have a background (depending on text color), allowing for better visibility if many numbers are shown; the text itself is black; therefore, dark background colors are ignored and shown as white
  float textOffsetFactor;                         //!< AUTO: must be >= 0; This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes
  bool threadSafeGraphicsUpdate;                  //!< AUTO: true = updating of visualization is threadsafe, but slower for complicated models; deactivate this to speed up computation, but activate for generation of animations; may be improved in future by adding a safe visualizationUpdate state
  bool useBitmapText;                             //!< AUTO: if true, texts are displayed using pre-defined bitmaps for the text; may increase the complexity of your scene, e.g., if many (>10000) node numbers shown
  bool useGradientBackground;                     //!< AUTO: true = use vertical gradient for background; 
  bool useMultiThreadedRendering;                 //!< AUTO: true = rendering is done in separate thread; false = no separate thread, which may be more stable but has lagging interaction for large models (do not interact with models during simulation); you MUST set this parameter BEFORE call to SC.renderer.Start(); MAC OS: uses always false, because MAC OS does not support multi threaded GLFW
  bool useWindowsDisplayScaleFactor;              //!< AUTO: the Windows display scaling (monitor scaling; content scaling) factor is used for increased visibility of texts on high resolution displays; based on GLFW glfwGetWindowContentScale; deactivated on linux compilation as it leads to crashes (adjust textSize manually!)
  bool zoomAllUseBoundingBox;                     //!< AUTO: if true, use exact scene bounding box (but not including texts) for zoom; does not include perspective effects!

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsGeneral()
  {
    backlink=nullptr;
    autoFitScene = true;
    axesTiling = 12;
    backgroundColor = Float4({1.0f,1.0f,1.0f,1.0f});
    backgroundColorBottom = Float4({0.8f,0.8f,1.0f,1.0f});
    boundingBoxZoomAllFactor = 1.2f;
    boundingBoxZoomAllOffset = 0.01f;
    circleTiling = 16;
    coordinateSystemSize = 5.f;
    cylinderTiling = 16;
    graphicsUpdateInterval = 0.1f;
    limitWindowToScreenSize = true;
    linuxDisplayScaleFactor = 1.;
    minSceneSize = 0.1f;
    pointSize = 0.01f;
    reallyQuitTimeLimit = 900;
    rendererPrecision = 4;
    rendererStartupTimeout = 2500;
    showHelpOnStartup = 5;
    showSolutionInformation = true;
    showSolverInformation = true;
    showSolverTime = true;
    sphereTiling = 6;
    textAlwaysInFront = true;
    textColor = Float4({0.f,0.f,0.f,1.0f});
    textHasBackground = false;
    textOffsetFactor = 0.005f;
    threadSafeGraphicsUpdate = true;
    useBitmapText = true;
    useGradientBackground = false;
    useMultiThreadedRendering = true;
    useWindowsDisplayScaleFactor = true;
    zoomAllUseBoundingBox = true;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.drawCoordinateSystem
  void PySetDrawCoordinateSystem(const Index& drawCoordinateSystemInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.drawCoordinateSystem
  Index PyGetDrawCoordinateSystem() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.drawWorldBasis
  void PySetDrawWorldBasis(const bool& drawWorldBasisInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.drawWorldBasis
  bool PyGetDrawWorldBasis() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.showComputationInfo
  void PySetShowComputationInfo(const bool& showComputationInfoInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.showComputationInfo
  bool PyGetShowComputationInfo() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.globalFontSize
  void PySetTextSize(const float& globalFontSizeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.globalFontSize
  float PyGetTextSize() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.worldBasisSize
  void PySetWorldBasisSize(const float& worldBasisSizeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.worldBasisSize
  float PyGetWorldBasisSize() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsGeneral" << ":\n";
    os << "  autoFitScene = " << autoFitScene << "\n";
    os << "  axesTiling = " << axesTiling << "\n";
    os << "  backgroundColor = " << backgroundColor << "\n";
    os << "  backgroundColorBottom = " << backgroundColorBottom << "\n";
    os << "  boundingBoxZoomAllFactor = " << boundingBoxZoomAllFactor << "\n";
    os << "  boundingBoxZoomAllOffset = " << boundingBoxZoomAllOffset << "\n";
    os << "  circleTiling = " << circleTiling << "\n";
    os << "  coordinateSystemSize = " << coordinateSystemSize << "\n";
    os << "  cylinderTiling = " << cylinderTiling << "\n";
    os << "  graphicsUpdateInterval = " << graphicsUpdateInterval << "\n";
    os << "  limitWindowToScreenSize = " << limitWindowToScreenSize << "\n";
    os << "  linuxDisplayScaleFactor = " << linuxDisplayScaleFactor << "\n";
    os << "  minSceneSize = " << minSceneSize << "\n";
    os << "  pointSize = " << pointSize << "\n";
    os << "  reallyQuitTimeLimit = " << reallyQuitTimeLimit << "\n";
    os << "  rendererPrecision = " << rendererPrecision << "\n";
    os << "  rendererStartupTimeout = " << rendererStartupTimeout << "\n";
    os << "  renderWindowString = " << renderWindowString << "\n";
    os << "  showHelpOnStartup = " << showHelpOnStartup << "\n";
    os << "  showSolutionInformation = " << showSolutionInformation << "\n";
    os << "  showSolverInformation = " << showSolverInformation << "\n";
    os << "  showSolverTime = " << showSolverTime << "\n";
    os << "  sphereTiling = " << sphereTiling << "\n";
    os << "  textAlwaysInFront = " << textAlwaysInFront << "\n";
    os << "  textColor = " << textColor << "\n";
    os << "  textHasBackground = " << textHasBackground << "\n";
    os << "  textOffsetFactor = " << textOffsetFactor << "\n";
    os << "  threadSafeGraphicsUpdate = " << threadSafeGraphicsUpdate << "\n";
    os << "  useBitmapText = " << useBitmapText << "\n";
    os << "  useGradientBackground = " << useGradientBackground << "\n";
    os << "  useMultiThreadedRendering = " << useMultiThreadedRendering << "\n";
    os << "  useWindowsDisplayScaleFactor = " << useWindowsDisplayScaleFactor << "\n";
    os << "  zoomAllUseBoundingBox = " << zoomAllUseBoundingBox << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsGeneral& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsContourAdvanced
* @brief        Advanced settings for contour plots.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsContourAdvanced // AUTO: 
{
public: // AUTO: 
  Index colorBarPrecision;                        //!< AUTO: must be > 0; precision of floating point values shown in color bar; total number of digits used (max. 16)
  Index colorBarTiling;                           //!< AUTO: must be > 0; number of tiles (segements) shown in the colorbar for the contour plot
  Float4 contourColor0;                           //!< AUTO: RGBA color for relative value 0 used for contour plot; alpha is ignored
  Float4 contourColor1;                           //!< AUTO: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  Float4 contourColor2;                           //!< AUTO: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  Float4 contourColor3;                           //!< AUTO: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  Float4 contourColor4;                           //!< AUTO: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  Float4 contourColorMax;                         //!< AUTO: RGBA color if relative value in contour plot is larger than 1 (if automaticRange=False); alpha is ignored
  Float4 contourColorMin;                         //!< AUTO: RGBA color if relative value in contour plot is smaller than 0 (if automaticRange=False); alpha is ignored
  bool showColorBar;                              //!< AUTO: show the colour bar with minimum and maximum values for the contour plot

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsContourAdvanced()
  {
    backlink=nullptr;
    colorBarPrecision = 4;
    colorBarTiling = 12;
    contourColor0 = Float4({0.1f,0.1f,0.9f,1.f});
    contourColor1 = Float4({0.1f,0.9f,0.9f,1.f});
    contourColor2 = Float4({0.1f,0.9f,0.1f,1.f});
    contourColor3 = Float4({0.9f,0.9f,0.1f,1.f});
    contourColor4 = Float4({0.9f,0.1f,0.1f,1.f});
    contourColorMax = Float4({0.9f,0.9f,0.9f,1.f});
    contourColorMin = Float4({0.1f,0.1f,0.1f,1.f});
    showColorBar = true;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsContourAdvanced" << ":\n";
    os << "  colorBarPrecision = " << colorBarPrecision << "\n";
    os << "  colorBarTiling = " << colorBarTiling << "\n";
    os << "  contourColor0 = " << contourColor0 << "\n";
    os << "  contourColor1 = " << contourColor1 << "\n";
    os << "  contourColor2 = " << contourColor2 << "\n";
    os << "  contourColor3 = " << contourColor3 << "\n";
    os << "  contourColor4 = " << contourColor4 << "\n";
    os << "  contourColorMax = " << contourColorMax << "\n";
    os << "  contourColorMin = " << contourColorMin << "\n";
    os << "  showColorBar = " << showColorBar << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsContourAdvanced& object)
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsContour // AUTO: 
{
public: // AUTO: 
  VSettingsContourAdvanced advanced;              //!< AUTO: advanced settings for contour
  float alphaTransparency;                        //!< AUTO: default value for contour alpha transparency (RGB color computed from contour value)
  bool automaticRange;                            //!< AUTO: if true, the contour plot value range is chosen automatically to the maximum range
  float maxValue;                                 //!< AUTO: maximum value for contour plot; set manually, if automaticRange == False
  float minValue;                                 //!< AUTO: minimum value for contour plot; set manually, if automaticRange == False
  bool nodesColored;                              //!< AUTO: if true, the contour color is also applied to nodes (except mesh nodes), otherwise node drawing is not influenced by contour settings
  OutputVariableType outputVariable;              //!< AUTO: selected contour plot output variable type; select OutputVariableType._None to deactivate contour plotting.
  Index outputVariableComponent;                  //!< AUTO: select the component of the chosen output variable; e.g., for displacements, 3 components are available: 0 == x, 1 == y, 2 == z component; for stresses, 6 components are available, see OutputVariableType description; to draw the norm of a outputVariable, set component to -1; if a certain component is not available by certain objects or nodes, no value is drawn (using default color)
  bool reduceRange;                               //!< AUTO: if true, the contour plot value range is also reduced; better for static computation; in dynamic computation set this option to false, it can reduce visualization artifacts; you should also set minVal to max(float) and maxVal to min(float)
  bool rigidBodiesColored;                        //!< AUTO: if true, the contour color is also applied to triangular faces of rigid bodies and mass points, otherwise the rigid body drawing are not influenced by contour settings; for general rigid bodies (except for ObjectGround), Position, Displacement, DisplacementLocal(=0), Velocity, VelocityLocal, AngularVelocity, and AngularVelocityLocal are available; may slow down visualization!

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsContour()
  {
    backlink=nullptr;
    alphaTransparency = 1;
    automaticRange = true;
    maxValue = 1;
    minValue = 0;
    nodesColored = true;
    outputVariable = OutputVariableType::_None;
    outputVariableComponent = 0;
    reduceRange = true;
    rigidBodiesColored = true;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    advanced.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use contour.advanced.colorBarPrecision
  void PySetColorBarPrecision(const Index& colorBarPrecisionInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use contour.advanced.colorBarPrecision
  Index PyGetColorBarPrecision() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use contour.advanced.colorBarTiling
  void PySetColorBarTiling(const Index& colorBarTilingInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use contour.advanced.colorBarTiling
  Index PyGetColorBarTiling() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use contour.advanced.showColorBar
  void PySetShowColorBar(const bool& showColorBarInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use contour.advanced.showColorBar
  bool PyGetShowColorBar() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsContour" << ":\n";
    os << "  advanced = " << advanced << "\n";
    os << "  alphaTransparency = " << alphaTransparency << "\n";
    os << "  automaticRange = " << automaticRange << "\n";
    os << "  maxValue = " << maxValue << "\n";
    os << "  minValue = " << minValue << "\n";
    os << "  nodesColored = " << nodesColored << "\n";
    os << "  outputVariable = " << GetOutputVariableTypeString(outputVariable) << "\n";
    os << "  outputVariableComponent = " << outputVariableComponent << "\n";
    os << "  reduceRange = " << reduceRange << "\n";
    os << "  rigidBodiesColored = " << rigidBodiesColored << "\n";
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsNodes // AUTO: 
{
public: // AUTO: 
  float basisSize;                                //!< AUTO: size of basis for nodes
  Float4 defaultColor;                            //!< AUTO: default RGBA color for nodes; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global node size; if -1.f, node size is relative to openGL.initialMaxSceneSize
  bool drawNodesAsPoint;                          //!< AUTO: simplified/faster drawing of nodes; uses general->pointSize as drawing size; if drawNodesAsPoint==True, the basis of the node will be drawn with lines
  bool show;                                      //!< AUTO: flag to decide, whether the nodes are shown
  bool showBasis;                                 //!< AUTO: show basis (three axes) of coordinate system in 3D nodes
  bool showNodalSlopes;                           //!< AUTO: draw nodal slope vectors, e.g. in ANCF beam finite elements
  bool showNumbers;                               //!< AUTO: flag to decide, whether the node number is shown
  Index tiling;                                   //!< AUTO: must be > 0; tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsNodes()
  {
    backlink=nullptr;
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
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsBeams // AUTO: 
{
public: // AUTO: 
  Index axialTiling;                              //!< AUTO: must be > 0; number of segments to discretise the beams axis
  bool crossSectionFilled;                        //!< AUTO: if implemented for element, cross section is drawn as solid (filled) instead of wire-frame; NOTE: some quantities may not be interpolated correctly over cross section in visualization; equivalent to drawSolid of shells
  Index crossSectionTiling;                       //!< AUTO: must be > 0; number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing
  bool drawVertical;                              //!< AUTO: draw contour plot outputVariables 'vertical' along beam height; contour.outputVariable must be set accordingly
  Float4 drawVerticalColor;                       //!< AUTO: color for outputVariable to be drawn along cross section (vertically)
  float drawVerticalFactor;                       //!< AUTO: must be >= 0; factor for outputVariable to be drawn along cross section (vertically)
  bool drawVerticalLines;                         //!< AUTO: draw additional vertical lines for better visibility
  float drawVerticalOffset;                       //!< AUTO: offset for vertical drawn lines; offset is added before multiplication with drawVerticalFactor
  bool drawVerticalValues;                        //!< AUTO: show values at vertical lines; note that these numbers are interpolated values and may be different from values evaluated directly at this point!
  bool reducedAxialInterploation;                 //!< AUTO: if True, the interpolation along the beam axis may be lower than the beam element order; this may, however, show more consistent values than a full interpolation, e.g. for strains or forces

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsBeams()
  {
    backlink=nullptr;
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
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
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
* @class        VSettingsShells
* @brief        Visualization settings for plate/shell finite elements.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsShells // AUTO: 
{
public: // AUTO: 
  bool drawSolid;                                 //!< AUTO: if true: to draw plates/shells as 3D objects; false: only the element surface is drawn; equivalent to crossSectionFilled in beams
  float thicknessFactor;                          //!< AUTO: must be > 0; a factor multiplied with the thickness of shells/plates only for visualization (e.g. to make some effects more visible)

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsShells()
  {
    backlink=nullptr;
    drawSolid = true;
    thicknessFactor = 1.f;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsShells" << ":\n";
    os << "  drawSolid = " << drawSolid << "\n";
    os << "  thicknessFactor = " << thicknessFactor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsShells& object)
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsKinematicTree // AUTO: 
{
public: // AUTO: 
  float frameSize;                                //!< AUTO: size of COM and joint frames
  bool showCOMframes;                             //!< AUTO: if True, a frame is attached to every center of mass
  bool showFramesNumbers;                         //!< AUTO: if True, numbers are drawn for joint frames (O[i]J[j]) and COM frames (O[i]COM[j]) for object [i] and local joint [j]
  bool showJointFrames;                           //!< AUTO: if True, a frame is attached to the origin of every joint frame

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsKinematicTree()
  {
    backlink=nullptr;
    frameSize = 0.2f;
    showCOMframes = false;
    showFramesNumbers = false;
    showJointFrames = true;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsBodies // AUTO: 
{
public: // AUTO: 
  VSettingsBeams beams;                           //!< AUTO: visualization settings for beams (e.g. ANCFCable or other beam elements)
  VSettingsKinematicTree kinematicTree;           //!< AUTO: visualization settings for kinematic tree
  VSettingsShells shells;                         //!< AUTO: visualization settings for plates and shells
  Float4 defaultColor;                            //!< AUTO: default RGBA color for bodies; 4th value is alpha-transparency
  Float3 defaultSize;                             //!< AUTO: global body size of xyz-cube
  float deformationScaleFactor;                   //!< AUTO: global deformation scale factor for the drawing of superelements (FFRF, FFRFreducedOrder, GenericODE2 with a mesh): their mesh nodes and the markers on them are drawn with the local deformation scaled by it (#1813)
  bool show;                                      //!< AUTO: flag to decide, whether the bodies are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the body(=object) number is shown

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsBodies()
  {
    backlink=nullptr;
    defaultColor = Float4({0.3f,0.3f,1.f,1.f});
    defaultSize = Float3({1.f,1.f,1.f});
    deformationScaleFactor = 1;
    show = true;
    showNumbers = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    beams.Init(backlinkInit);
    kinematicTree.Init(backlinkInit);
    shells.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsBodies" << ":\n";
    os << "  beams = " << beams << "\n";
    os << "  kinematicTree = " << kinematicTree << "\n";
    os << "  shells = " << shells << "\n";
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsConnectors // AUTO: 
{
public: // AUTO: 
  float contactPointsDefaultSize;                 //!< AUTO: DEPRECATED: do not use! global contact points size; if -1.f, connector size is relative to maxSceneSize
  Index curveTiling;                              //!< AUTO: must be > 0; number of segments of a full turn of a curve drawn by a connector: a winding of a spring, the arc of a rope on a sheave (ObjectConnectorReevingSystemSprings); an arc gets the share of its angle, at least one segment
  Float4 defaultColor;                            //!< AUTO: default RGBA color for connectors; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global connector size; if -1.f, connector size is relative to maxSceneSize
  float jointAxesLength;                          //!< AUTO: global joint axes length
  float jointAxesRadius;                          //!< AUTO: global joint axes radius
  bool show;                                      //!< AUTO: flag to decide, whether the connectors are shown
  bool showContact;                               //!< AUTO: flag to decide, whether contact points, lines, etc. are shown for special cable-circle contacts; for spheres, triangles, tori, see visualizationSettings.contact
  bool showJointAxes;                             //!< AUTO: flag to decide, whether contact joint axes of 3D joints are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the connector(=object) number is shown
  bool springDraw3D;                              //!< AUTO: flag to draw the windings of springs as a tube with a tenth of the spring radius, instead of lines
  Index springNumberOfWindings;                   //!< AUTO: must be > 0; number of windings for springs drawn as helical spring

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsConnectors()
  {
    backlink=nullptr;
    contactPointsDefaultSize = 0.02f;
    curveTiling = 32;
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
    defaultSize = 0.1f;
    jointAxesLength = 0.2f;
    jointAxesRadius = 0.02f;
    show = true;
    showContact = false;
    showJointAxes = false;
    showNumbers = false;
    springDraw3D = false;
    springNumberOfWindings = 8;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsConnectors" << ":\n";
    os << "  contactPointsDefaultSize = " << contactPointsDefaultSize << "\n";
    os << "  curveTiling = " << curveTiling << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  jointAxesLength = " << jointAxesLength << "\n";
    os << "  jointAxesRadius = " << jointAxesRadius << "\n";
    os << "  show = " << show << "\n";
    os << "  showContact = " << showContact << "\n";
    os << "  showJointAxes = " << showJointAxes << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
    os << "  springDraw3D = " << springDraw3D << "\n";
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsMarkers // AUTO: 
{
public: // AUTO: 
  float basisSize;                                //!< AUTO: size of the frame of the markers
  Float4 defaultColor;                            //!< AUTO: default RGBA color for markers; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global marker size; if -1.f, marker size is relative to maxSceneSize
  bool drawSimplified;                            //!< AUTO: draw markers with simplified symbols
  bool show;                                      //!< AUTO: flag to decide, whether the markers are shown
  bool showBasis;                                 //!< AUTO: show the frame (three axes) of the markers with position and orientation; with drawSimplified as three lines in red, green and blue, else as three arrows, whose heads are half as long as those of a node basis
  bool showNumbers;                               //!< AUTO: flag to decide, whether the marker numbers are shown

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsMarkers()
  {
    backlink=nullptr;
    basisSize = 0.2f;
    defaultColor = Float4({0.1f,0.5f,0.1f,1.f});
    defaultSize = -1.f;
    drawSimplified = true;
    show = true;
    showBasis = false;
    showNumbers = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsMarkers" << ":\n";
    os << "  basisSize = " << basisSize << "\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  show = " << show << "\n";
    os << "  showBasis = " << showBasis << "\n";
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsLoads // AUTO: 
{
public: // AUTO: 
  Float4 defaultColor;                            //!< AUTO: default RGBA color for loads; 4th value is alpha-transparency
  float defaultRadius;                            //!< AUTO: global radius of load axis if drawn in 3D
  float defaultSize;                              //!< AUTO: global load size; if -1.f, load size is relative to maxSceneSize
  bool drawSimplified;                            //!< AUTO: draw markers with simplified symbols
  bool drawWithUserFunction;                      //!< AUTO: draw loads like force vectors time dependent; make sure that fixedLoadSize=false, while otherwise only the direction will change; user functions can only be drawn, if they are either symbolic or for Python user functions if useMultiThreadedRendering=False
  bool fixedLoadSize;                             //!< AUTO: if true, the load is drawn with a fixed vector length in direction of the load vector, independently of the load size
  float loadSizeFactor;                           //!< AUTO: if fixedLoadSize=false, then this scaling factor is used to draw the load vector
  bool show;                                      //!< AUTO: flag to decide, whether the loads are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the load numbers are shown

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsLoads()
  {
    backlink=nullptr;
    defaultColor = Float4({0.7f,0.1f,0.1f,1.f});
    defaultRadius = 0.005f;
    defaultSize = 0.2f;
    drawSimplified = true;
    drawWithUserFunction = true;
    fixedLoadSize = true;
    loadSizeFactor = 0.1f;
    show = true;
    showNumbers = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsLoads" << ":\n";
    os << "  defaultColor = " << defaultColor << "\n";
    os << "  defaultRadius = " << defaultRadius << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
    os << "  drawSimplified = " << drawSimplified << "\n";
    os << "  drawWithUserFunction = " << drawWithUserFunction << "\n";
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
* @class        VSettingsTraces
* @brief        Visualization settings for traces of sensors. Note that a large number of time points (influenced by simulationSettings.solution.sensors.writePeriod) may lead to slow graphics.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsTraces // AUTO: 
{
public: // AUTO: 
  float lineWidth;                                //!< AUTO: must be >= 0; line width for traces
  ArrayIndex listOfPositionSensors;               //!< AUTO: list of position sensors which can be shown as trace inside render window if sensors have storeInternal=True; if this list is empty and showPositionTrace=True, then all available sensors are shown
  ArrayIndex listOfTriadSensors;                  //!< AUTO: list of sensors of with OutputVariableType RotationMatrix; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showTriads=True; the triad is drawn at the related position
  ArrayIndex listOfVectorSensors;                 //!< AUTO: list of sensors with 3D vector quantities; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showVectors=True; the vector quantity is drawn relative to the related position
  Index positionsShowEvery;                       //!< AUTO: must be > 0; integer value i; out of available sensor data, show every i-th position
  Index sensorsMbsNumber;                         //!< AUTO: number of main system which is used to for sensor lists; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number
  bool showCurrent;                               //!< AUTO: show current trace position (and especially vector quantity) related to current visualization state; this only works in solution viewer if sensor values are stored at time grid points of the solution file (up to a precision of 1e-10) and may therefore be temporarily unavailable
  bool showFuture;                                //!< AUTO: show trace future to current visualization state if already computed (e.g. in SolutionViewer)
  bool showPast;                                  //!< AUTO: show trace previous to current visualization state
  bool showPositionTrace;                         //!< AUTO: show position trace of all position sensors if listOfPositionSensors=[] or of specified sensors; sensors need to activate storeInternal=True
  bool showTriads;                                //!< AUTO: if True, show basis vectors from rotation matrices provided by sensors
  bool showVectors;                               //!< AUTO: if True, show vector quantities according to description in showPositionTrace
  Real timeSpan;                                  //!< AUTO: must be >= 0; maximum trace time span of past or future trace; given in seconds of simulation time; if zero, it is unused
  ArrayFloat traceColors;                         //!< AUTO: RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue
  float triadSize;                                //!< AUTO: length of triad axes if shown
  Index triadsShowEvery;                          //!< AUTO: must be > 0; integer value i; out of available sensor data, show every i-th triad
  float vectorScaling;                            //!< AUTO: scaling of vector quantities; if, e.g., loads, this factor has to be adjusted significantly
  Index vectorsShowEvery;                         //!< AUTO: must be > 0; integer value i; out of available sensor data, show every i-th vector

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsTraces()
  {
    backlink=nullptr;
    lineWidth = 2.f;
    listOfPositionSensors = ArrayIndex();
    listOfTriadSensors = ArrayIndex();
    listOfVectorSensors = ArrayIndex();
    positionsShowEvery = 1;
    sensorsMbsNumber = 0;
    showCurrent = true;
    showFuture = false;
    showPast = true;
    showPositionTrace = false;
    showTriads = false;
    showVectors = false;
    timeSpan = 0;
    traceColors = ArrayFloat({0.2f,0.2f,0.2f,1.f, 0.8f,0.2f,0.2f,1.f, 0.2f,0.8f,0.2f,1.f, 0.2f,0.2f,0.8f,1.f, 0.2f,0.8f,0.8f,1.f, 0.8f,0.2f,0.8f,1.f, 0.8f,0.4f,0.1f,1.f});
    triadSize = 0.1f ;
    triadsShowEvery = 1;
    vectorScaling = 0.01f;
    vectorsShowEvery = 1;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue
  void PySetTraceColors(const std::vector<float>& traceColorsInit) { traceColors = traceColorsInit; }
  //! AUTO: Read (Copy) access to: RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue
  std::vector<float> PyGetTraceColors() const { return std::vector<float>(traceColors); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsTraces" << ":\n";
    os << "  lineWidth = " << lineWidth << "\n";
    os << "  listOfPositionSensors = " << listOfPositionSensors << "\n";
    os << "  listOfTriadSensors = " << listOfTriadSensors << "\n";
    os << "  listOfVectorSensors = " << listOfVectorSensors << "\n";
    os << "  positionsShowEvery = " << positionsShowEvery << "\n";
    os << "  sensorsMbsNumber = " << sensorsMbsNumber << "\n";
    os << "  showCurrent = " << showCurrent << "\n";
    os << "  showFuture = " << showFuture << "\n";
    os << "  showPast = " << showPast << "\n";
    os << "  showPositionTrace = " << showPositionTrace << "\n";
    os << "  showTriads = " << showTriads << "\n";
    os << "  showVectors = " << showVectors << "\n";
    os << "  timeSpan = " << timeSpan << "\n";
    os << "  traceColors = " << traceColors << "\n";
    os << "  triadSize = " << triadSize << "\n";
    os << "  triadsShowEvery = " << triadsShowEvery << "\n";
    os << "  vectorScaling = " << vectorScaling << "\n";
    os << "  vectorsShowEvery = " << vectorsShowEvery << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsTraces& object)
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsSensors // AUTO: 
{
public: // AUTO: 
  VSettingsTraces traces;                         //!< AUTO: settings for showing (position/triad) sensor traces and vector plots in the render window
  Float4 defaultColor;                            //!< AUTO: default RGBA color for sensors; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global sensor size; if -1.f, sensor size is relative to maxSceneSize
  bool drawSimplified;                            //!< AUTO: draw sensors with simplified symbols
  bool show;                                      //!< AUTO: flag to decide, whether the sensors are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the sensor numbers are shown

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsSensors()
  {
    backlink=nullptr;
    defaultColor = Float4({0.6f,0.6f,0.1f,1.f});
    defaultSize = -1.f;
    drawSimplified = true;
    show = true;
    showNumbers = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    traces.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsSensors" << ":\n";
    os << "  traces = " << traces << "\n";
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
* @brief        Global visualization settings for GeneralContact. This allows to easily switch on/off during visualization; also used for contact objects, such as ObjectContactSphereSphere or ObjectContactSphereTriangle
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsContact // AUTO: 
{
public: // AUTO: 
  Float4 colorBoundingBoxes;                      //!< AUTO: RGBA color for boudnding boxes, see showBoundingBoxes
  Float4 colorSearchTree;                         //!< AUTO: RGBA color for search tree, see showSearchTree
  Float4 colorSpheres;                            //!< AUTO: RGBA color for contact spheres, see showSpheres
  Float4 colorTori;                               //!< AUTO: RGBA color for contact tori, see showTori
  Float4 colorTriangles;                          //!< AUTO: RGBA color for contact triangles, see showTriangles
  float contactForcesFactor;                      //!< AUTO: factor used for scaling of contact forces is showContactForces=True
  float contactPointsDefaultSize;                 //!< AUTO: global contact points size; if -1.f, connector size is relative to maxSceneSize; used for some contacts, e.g., in ContactFrictionCircle
  bool showBoundingBoxes;                         //!< AUTO: show computed bounding boxes of all GeneralContacts; Warning: avoid for large number of contact objects!
  bool showContactForces;                         //!< AUTO: if True, contact forces are drawn for certain contact models
  bool showContactForcesValues;                   //!< AUTO: if True and showContactForces=True, numerical values for  contact forces are shown at certain points
  bool showSearchTree;                            //!< AUTO: show outer box of search tree for all GeneralContacts
  bool showSearchTreeCells;                       //!< AUTO: show all cells of search tree; empty cells have colorSearchTree, cells with contact objects have higher red value; Warning: avoid for large number of search tree cells!
  bool showSpheres;                               //!< AUTO: show contact spheres (SpheresWithMarker, ...)
  bool showTori;                                  //!< AUTO: show each contact torus
  bool showTriangles;                             //!< AUTO: show contact triangles (TrianglesRigidBodyBased, ...)
  Index tilingCurves;                             //!< AUTO: must be > 0; tiling for nonlinear/polynomial curves; higher values give smoother curves
  Index tilingSpheres;                            //!< AUTO: must be > 0; tiling for spheres; higher values give smoother spheres, but may lead to lower frame rates

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsContact()
  {
    backlink=nullptr;
    colorBoundingBoxes = Float4({0.9f,0.1f,0.1f,1.f});
    colorSearchTree = Float4({0.1f,0.1f,0.9f,1.f});
    colorSpheres = Float4({0.8f,0.5f,0.2f,1.f});
    colorTori = Float4({0.8f,0.2f,0.8f,1.f});
    colorTriangles = Float4({0.5f,0.5f,0.5f,1.f});
    contactForcesFactor = 0.001f;
    contactPointsDefaultSize = 0.001f;
    showBoundingBoxes = false;
    showContactForces = false;
    showContactForcesValues = false;
    showSearchTree = false;
    showSearchTreeCells = false;
    showSpheres = false;
    showTori = false;
    showTriangles = false;
    tilingCurves = 8;
    tilingSpheres = 4;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsContact" << ":\n";
    os << "  colorBoundingBoxes = " << colorBoundingBoxes << "\n";
    os << "  colorSearchTree = " << colorSearchTree << "\n";
    os << "  colorSpheres = " << colorSpheres << "\n";
    os << "  colorTori = " << colorTori << "\n";
    os << "  colorTriangles = " << colorTriangles << "\n";
    os << "  contactForcesFactor = " << contactForcesFactor << "\n";
    os << "  contactPointsDefaultSize = " << contactPointsDefaultSize << "\n";
    os << "  showBoundingBoxes = " << showBoundingBoxes << "\n";
    os << "  showContactForces = " << showContactForces << "\n";
    os << "  showContactForcesValues = " << showContactForcesValues << "\n";
    os << "  showSearchTree = " << showSearchTree << "\n";
    os << "  showSearchTreeCells = " << showSearchTreeCells << "\n";
    os << "  showSpheres = " << showSpheres << "\n";
    os << "  showTori = " << showTori << "\n";
    os << "  showTriangles = " << showTriangles << "\n";
    os << "  tilingCurves = " << tilingCurves << "\n";
    os << "  tilingSpheres = " << tilingSpheres << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsContact& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsCamera
* @brief        Settings for camera like perspective, marker tracking, clipping plane, etc. Note that some options may also be found in openGL settings.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsCamera // AUTO: 
{
public: // AUTO: 
  Float3 cameraPosition;                          //!< AUTO: if modelCentricView=True: offset to camera position in model view (and, if used, relative to tracked marker - instead of a tracked marker position, you could also just change the camera position in camera-centric views); camera rotation follows modelRotation in renderState
  float clippingPlaneDistance;                    //!< AUTO: distance of clipping plane on normal vector; see also clippingPlaneNormal and openGL.advanced.clippingPlaneColor
  Float3 clippingPlaneNormal;                     //!< AUTO: normal vector of clipping plane, e.g. [0,0,1] to set a xy-clipping plane; the clipped half-space is in direction of the normal; use [0,0,0] to deactivate clipping plane; Note that clipping is mainly made for triangles in order to visualize hidden objects and currently it only fully clips triangles, but does not exactly cut them; see also clippingPlaneDistance and openGL.advanced.clippingPlaneColor
  bool modelCentricView;                          //!< AUTO: True: rotations and translations are applied to model, while camera stays far enough away from the model and always captures the whole model (everything is in front of camera plane); False: camera moves and rotates while model stays in physical space; only geometry in front of camera is visible; note that the behavior of trackMarker changes with modelCentricView and some features are not available in case of modelCentricView=False.
  Float3 nearFarPlaneOffset;                      //!< AUTO: the three values are [nearPlaneOffset, farPlaneOffset, flag]; if flag=0, the offsets are ignored and computed automatically, using x = 2 * maxSceneSize * zMaxSceneFactor, setting near plane to -x and far plane to +x in case of modelCentricView=True and setting near plane to 0.01 (minimal offset to eye point) and far plane to +x if modelCentricView=False; if flag=1, the near and far plane values are just overwritten; note that positive values for near plane make objects in front of the camera invisible while negative values make objects behind the camera plane visible; in case of camera-centric view, the eyepoint can be shifted backwards using cameraPosition accordingly.
  float perspective;                              //!< AUTO: must be >= 0; parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 1 (extreme: 5), where larger values are possible but should be used with care; NOTE that the relation to the common field of view (FOV) angle alpha, with alpha=90°, is given by perspective = tan(alpha/2) = 1; mouse coordinates (F3) can not be shown with perspective>0
  Index trackMarker;                              //!< AUTO: if valid marker index is provided and marker provides position (and orientation), the centerpoint of the scene follows the marker (and orientation); depends on trackMarkerPosition and trackMarkerOrientation; by default, only position is tracked
  Index trackMarkerMbsNumber;                     //!< AUTO: number of main system which is used to track marker; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number
  Float3 trackMarkerOrientation;                  //!< AUTO: choose which orientation axes (x,y,z) are tracked; currently can only be all zero or all one
  Float3 trackMarkerPosition;                     //!< AUTO: choose which coordinates or marker are tracked (x,y,z)
  bool useRaytracer;                              //!< AUTO: True: use (software) raytracer for this view; False: use standard OpenGL renderer

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsCamera()
  {
    backlink=nullptr;
    cameraPosition = Float3({0.f,0.f,0.f});
    clippingPlaneDistance = 0.f;
    clippingPlaneNormal = Float3({0.f,0.f,0.f});
    modelCentricView = true;
    nearFarPlaneOffset = Float3({0.f,0.f,0.f});
    perspective = 0.f;
    trackMarker = -1;
    trackMarkerMbsNumber = 0;
    trackMarkerOrientation = Float3({0.f,0.f,0.f});
    trackMarkerPosition = Float3({1.f,1.f,1.f});
    useRaytracer = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsCamera" << ":\n";
    os << "  cameraPosition = " << cameraPosition << "\n";
    os << "  clippingPlaneDistance = " << clippingPlaneDistance << "\n";
    os << "  clippingPlaneNormal = " << clippingPlaneNormal << "\n";
    os << "  modelCentricView = " << modelCentricView << "\n";
    os << "  nearFarPlaneOffset = " << nearFarPlaneOffset << "\n";
    os << "  perspective = " << perspective << "\n";
    os << "  trackMarker = " << trackMarker << "\n";
    os << "  trackMarkerMbsNumber = " << trackMarkerMbsNumber << "\n";
    os << "  trackMarkerOrientation = " << trackMarkerOrientation << "\n";
    os << "  trackMarkerPosition = " << trackMarkerPosition << "\n";
    os << "  useRaytracer = " << useRaytracer << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsCamera& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsScene
* @brief        Settings change scene representation (show edges, show faces, global transparency), adding world basis, etc., in particular settings that are individual to each view. Note that some scene settings that are global to all views may be found in general and in openGL settings
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsScene // AUTO: 
{
public: // AUTO: 
  Index drawCoordinateSystem;                     //!< AUTO: must be >= 0; 0 = no coordinate system shown, 1 = draw lines with text, 2 = draw arrows, 3 = draw arrows with text
  bool drawWorldBasis;                            //!< AUTO: true = draw world basis coordinate system at (0,0,0)
  bool facesTransparent;                          //!< AUTO: True: show faces transparent independent of transparency (A)-value in color of objects; allow to show otherwise hidden node/marker/object numbers
  bool showFaceEdges;                             //!< AUTO: True: show edges of triangles; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
  bool showFaces;                                 //!< AUTO: True: show faces of triangles, etc.; using the options showFaces=false and showFaceEdges=true gives are wireframe representation
  bool showLines;                                 //!< AUTO: True: show lines (other lines than face and mesh edges)
  bool showMeshEdges;                             //!< AUTO: True: show edges of finite elements; independent of showFaceEdges
  bool showMeshFaces;                             //!< AUTO: True: show faces of finite elements; independent of showFaces
  float worldBasisSize;                           //!< AUTO: must be > 0; size of world basis coordinate system

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsScene()
  {
    backlink=nullptr;
    drawCoordinateSystem = 2;
    drawWorldBasis = false;
    facesTransparent = false;
    showFaceEdges = false;
    showFaces = true;
    showLines = true;
    showMeshEdges = true;
    showMeshFaces = true;
    worldBasisSize = 1.f;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsScene" << ":\n";
    os << "  drawCoordinateSystem = " << drawCoordinateSystem << "\n";
    os << "  drawWorldBasis = " << drawWorldBasis << "\n";
    os << "  facesTransparent = " << facesTransparent << "\n";
    os << "  showFaceEdges = " << showFaceEdges << "\n";
    os << "  showFaces = " << showFaces << "\n";
    os << "  showLines = " << showLines << "\n";
    os << "  showMeshEdges = " << showMeshEdges << "\n";
    os << "  showMeshFaces = " << showMeshFaces << "\n";
    os << "  worldBasisSize = " << worldBasisSize << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsScene& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsWindow
* @brief        Settings for window that are individual to each view; in particular initial size, and behavior. Note that some of the settings are only used during creation of the window
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsWindow // AUTO: 
{
public: // AUTO: 
  bool alwaysOnTop;                               //!< AUTO: True: render window of respective view will be always on top of all other windows
  float globalFontSize;                           //!< AUTO: must be > 0; general text font size (roughly measured in pixels); if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
  bool lockModelView;                             //!< AUTO: True: all movements (with mouse/keys), rotations, zoom are disabled; the view is either based on initial values (or on the current state) ==> initial zoom, rotation and center point need to be adjusted, approx. 0.4*maxSceneSize is a good value
  bool maximize;                                  //!< AUTO: True: render window of respective view will be maximized at startup
  Index2 renderWindowPosition;                    //!< AUTO: position of the top left corner of the render window of this view, in pixels; a NEGATIVE coordinate - which is the default - means that the window manager places the window, as it did before this setting existed. Set both to place the window, or store them in `~/.exudyn/config.json` to have every run start there, see Section [](#sec-usersettings). NOTE: this is the position of the OpenGL area, not of the title bar, so a small value hides part of the title bar and 0 hides it completely - which still leaves the escape key, and is a way to have a view without one. The position is only used while the window is created, and one that lies outside the screens you have now puts the window where you cannot reach it
  Index2 renderWindowSize;                        //!< AUTO: initial size of the render window of this view, in pixels
  bool showComputationInfo;                       //!< AUTO: true = show (hide) all computation information including Exudyn and version
  bool showMouseCoordinates;                      //!< AUTO: True: show OpenGL coordinates and distance to last left mouse button pressed position in renderer status message; switched on/off with key 'F3'; only works for axis-aligned ortho-projections
  bool showRenderStateInfo;                       //!< AUTO: True: show renderer.state infos regarding zoom, offset and rotation in renderer status message; switched on/off with 'CTRL-F3'
  bool showWindow;                                //!< AUTO: True: render window of respective view is shown when created; False: window will be iconified when created (e.g. if you are starting multiple computations automatically)
  bool storeRenderWindowGeometry;                 //!< AUTO: True: when the render window of this view closes, where it was is written into `renderWindowSize` and `renderWindowPosition` - so that storing the settings keeps the window where you left it, see Section [](#sec-usersettings). False (default): the settings are only ever what you set, which is why *diff to default* does not report a window position after every run

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsWindow()
  {
    backlink=nullptr;
    alwaysOnTop = false;
    globalFontSize = 12.f;
    lockModelView = false;
    maximize = false;
    renderWindowPosition = Index2({-1,-1});
    renderWindowSize = Index2({1024,768});
    showComputationInfo = true;
    showMouseCoordinates = false;
    showRenderStateInfo = false;
    showWindow = true;
    storeRenderWindowGeometry = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsWindow" << ":\n";
    os << "  alwaysOnTop = " << alwaysOnTop << "\n";
    os << "  globalFontSize = " << globalFontSize << "\n";
    os << "  lockModelView = " << lockModelView << "\n";
    os << "  maximize = " << maximize << "\n";
    os << "  renderWindowPosition = " << renderWindowPosition << "\n";
    os << "  renderWindowSize = " << renderWindowSize << "\n";
    os << "  showComputationInfo = " << showComputationInfo << "\n";
    os << "  showMouseCoordinates = " << showMouseCoordinates << "\n";
    os << "  showRenderStateInfo = " << showRenderStateInfo << "\n";
    os << "  showWindow = " << showWindow << "\n";
    os << "  storeRenderWindowGeometry = " << storeRenderWindowGeometry << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsWindow& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsView
* @brief        Settings for view including camera, scene, window, and advanced options to setup a view or view window.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsView // AUTO: 
{
public: // AUTO: 
  VSettingsCamera camera;                         //!< AUTO: settings for camera like perspective, marker tracking or clipping plane
  VSettingsScene scene;                           //!< AUTO: settings which change scene representation, showing edges, faces or world basis
  VSettingsWindow window;                         //!< AUTO: visualization settings for window that are individual to each view

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsView()
  {
    backlink=nullptr;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    camera.Init(backlinkInit);
    scene.Init(backlinkInit);
    window.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsView" << ":\n";
    os << "  camera = " << camera << "\n";
    os << "  scene = " << scene << "\n";
    os << "  window = " << window << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsView& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsWindowDeprecated
* @brief        OpenGL Window and interaction settings for visualization; handle changes with care, as they might lead to unexpected results or crashes.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsWindowDeprecated // AUTO: 
{
private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsWindowDeprecated()
  {
    backlink=nullptr;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.alwaysOnTop
  void PySetAlwaysOnTop(const bool& alwaysOnTopInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.alwaysOnTop
  bool PyGetAlwaysOnTop() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.ignoreKeys
  void PySetIgnoreKeys(const bool& ignoreKeysInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.ignoreKeys
  bool PyGetIgnoreKeys() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.keyPressUserFunction
  void PySetKeyPressUserFunction(const std::function<bool(int, int, int)>& keyPressUserFunctionInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.keyPressUserFunction
  std::function<bool(int, int, int)> PyGetKeyPressUserFunction() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use general.limitWindowToScreenSize
  void PySetLimitWindowToScreenSize(const bool& limitWindowToScreenSizeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use general.limitWindowToScreenSize
  bool PyGetLimitWindowToScreenSize() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.maximize
  void PySetMaximize(const bool& maximizeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.maximize
  bool PyGetMaximize() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use general.reallyQuitTimeLimit
  void PySetReallyQuitTimeLimit(const Real& reallyQuitTimeLimitInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use general.reallyQuitTimeLimit
  Real PyGetReallyQuitTimeLimit() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.renderWindowSize
  void PySetRenderWindowSize(const std::array<Index,2>& renderWindowSizeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.renderWindowSize
  std::array<Index,2> PyGetRenderWindowSize() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.showMouseCoordinates
  void PySetShowMouseCoordinates(const bool& showMouseCoordinatesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.showMouseCoordinates
  bool PyGetShowMouseCoordinates() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.showRenderStateInfo
  void PySetShowRenderStateInfo(const bool& showRenderStateInfoInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.showRenderStateInfo
  bool PyGetShowRenderStateInfo() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.showWindow
  void PySetShowWindow(const bool& showWindowInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.showWindow
  bool PyGetShowWindow() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use general.rendererStartupTimeout
  void PySetStartupTimeout(const Index& rendererStartupTimeoutInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use general.rendererStartupTimeout
  Index PyGetStartupTimeout() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsWindowDeprecated" << ":\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsWindowDeprecated& object)
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
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsDialogs // AUTO: 
{
public: // AUTO: 
  float alphaTransparency;                        //!< AUTO: must be >= 0; alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)
  bool alwaysTopmost;                             //!< AUTO: True: dialogs are always topmost (otherwise, they are sometimes hidden)
  float columnWidthName;                          //!< AUTO: must be >= 0; width of the name column of a settings dialog, as a fraction of the width of the dialog; the description column takes what the three columns leave
  float columnWidthType;                          //!< AUTO: must be >= 0; width of the type column of a settings dialog, as a fraction of the width of the dialog
  float columnWidthValue;                         //!< AUTO: must be >= 0; width of the value column of a settings dialog, as a fraction of the width of the dialog
  float fontScaling;                              //!< AUTO: must be >= 0; scaling of the font in dialogs; 0 = automatic, which is the system display scaling on Windows and Linux and a fixed factor on MacOS. Any value > 0 sets the font scaling on EVERY platform, which is the way to make the dialogs readable on a Linux desktop
  bool multiThreadedDialogs;                      //!< AUTO: True: During dialogs, the OpenGL render windows will still get updates of changes in dialogs, etc., which may cause problems on some platforms or for some (complicated) models; False: changes of dialogs will take effect when dialogs are closed
  bool openTreeView;                              //!< AUTO: True: all sub-trees of the visusalization dialog are opened when opening the dialog; False: only some sub-trees are opened
  bool storeDialogPositions;                      //!< AUTO: True: a dialog stores its size and position in `~/.exudyn/config.json` when it closes, so that the next dialog of the same kind starts with them. A geometry that IS stored - by this flag, by the store button of the settings dialog, or by a script - is used whenever such a dialog opens, whatever this flag says: the size always, the position only if the window would still be reachable on the current screen. See Section [](#sec-usersettings)

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsDialogs()
  {
    backlink=nullptr;
    alphaTransparency = 0.94f;
    alwaysTopmost = true;
    columnWidthName = 0.31f;
    columnWidthType = 0.11f;
    columnWidthValue = 0.18f;
    fontScaling = 0.f;
    multiThreadedDialogs = true;
    openTreeView = false;
    storeDialogPositions = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use dialogs.fontScaling
  void PySetFontScalingMacOS(const float& fontScalingInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use dialogs.fontScaling
  float PyGetFontScalingMacOS() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsDialogs" << ":\n";
    os << "  alphaTransparency = " << alphaTransparency << "\n";
    os << "  alwaysTopmost = " << alwaysTopmost << "\n";
    os << "  columnWidthName = " << columnWidthName << "\n";
    os << "  columnWidthType = " << columnWidthType << "\n";
    os << "  columnWidthValue = " << columnWidthValue << "\n";
    os << "  fontScaling = " << fontScaling << "\n";
    os << "  multiThreadedDialogs = " << multiThreadedDialogs << "\n";
    os << "  openTreeView = " << openTreeView << "\n";
    os << "  storeDialogPositions = " << storeDialogPositions << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsDialogs& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsMaterial
* @brief        Settings for rendering materials, in particular for the Raytracer (may be available also in the OpenGL renderer in the future). This material (widely follows Phong model) can be either accessed via SC.renderer.materials or directly in visualizationSettings.raytracer.material0, material1, etc.; the ten materials of the raytracer each start from their own values, which are listed at material0 to material9.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsMaterial // AUTO: 
{
public: // AUTO: 
  float alpha;                                    //!< AUTO: must be >= 0; alpha-transparency, same as in alpha channel in RGBA colors; 1=opaque, 0=fully transparent; leads to extra rendering costs per transparent pixel
  Float3 baseColor;                               //!< AUTO: RGB default material color if face color has R-color channel -1
  Float3 emission;                                //!< AUTO: RGB emissive material color (enlightened material)
  float ior;                                      //!< AUTO: must be >= 0; index of refraction for transparent materials (1=no refraction), >1 represents refraction
  std::string name;                               //!< AUTO: material name for easier handling
  float reflectivity;                             //!< AUTO: must be >= 0; controls reflectivity of material; 0=no reflections (rough, e.g. rubber), 1=fully reflective (mirror); this leads to large extra rendering costs per visible reflective pixel
  float shininess;                                //!< AUTO: must be >= 0; controls shininess of specular component of lights; values < 5 is not very shiny, while > 50 is very shiny
  Float3 specular;                                //!< AUTO: RGB specular material color

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsMaterial()
  {
    backlink=nullptr;
    alpha = 1.f;
    baseColor = Float3({0.5f,0.5f,0.5f});
    emission = Float3({0.f,0.f,0.f});
    ior = 1.f;
    name = "undefined";
    reflectivity = 0.f;
    shininess = 32.f;
    specular = Float3({0.5f,0.5f,0.5f});
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsMaterial" << ":\n";
    os << "  alpha = " << alpha << "\n";
    os << "  baseColor = " << baseColor << "\n";
    os << "  emission = " << emission << "\n";
    os << "  ior = " << ior << "\n";
    os << "  name = " << name << "\n";
    os << "  reflectivity = " << reflectivity << "\n";
    os << "  shininess = " << shininess << "\n";
    os << "  specular = " << specular << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsMaterial& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsRaytracerAdvanced
* @brief        Advanced settings for raytracer.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsRaytracerAdvanced // AUTO: 
{
public: // AUTO: 
  Float4 backgroundColorReflections;              //!< AUTO: scene RGBA color for background that is hit by reflection material; while openGL.backgroundColor is used for rays that do not hit an object, this background may - if black or white - not be a suitable color for computing reflections; this is generally needed, as our scenes are usually not inside a closed geometry (like inside a room); this color is also used if maxReflectionDepth is reached
  Index searchTreeFactor;                         //!< AUTO: must be > 0; This factor can be used to increase the number of search tree bins, which can improve performance in case of inequilibrated scense; range=1..128
  Index shadowScalingFactor;                      //!< AUTO: must be >= 0; if lightRadiusVariations>1, this defines the downscaling factor of the shadow map, where 2 means that the resolution is 2 times smaller than the image resolution; additionally, multisampling is not used for shadow map computation if shadowScalingFactor>0, thus reducing the computational effort for shadow computation also in case of 1; range=0..16; larger values cause significant artifacts at shadow boundaries
  Index shadowSmoothingSteps;                     //!< AUTO: must be >= 0; if lightRadiusVariations>1, this defines the number of smoothing steps at the low-resolution shadow map; smoothing reduces shadow artifacts caused by smaller values of lightRadiusVariations; range=0..32; smoothing  steps may cause artifacts at shadow boundaries; only works for lights with a position (the 4th component of the light position should be 1)
  bool showText;                                  //!< AUTO: True: show any kind of status text, node numbers, object numbers, etc. (depending on settings); False: do not show any text in raytracer, independently of settings
  Index tilesPerThread;                           //!< AUTO: must be > 0; Total number of sub-tiles per thread, used to evenly distribute rendering load to threads
  float zBiasLines;                               //!< AUTO: offset for lines to draw in front of faces; relative to scene radius

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsRaytracerAdvanced()
  {
    backlink=nullptr;
    backgroundColorReflections = Float4({0.4f,0.4f,0.4f,1.f});
    searchTreeFactor = 1;
    shadowScalingFactor = 3;
    shadowSmoothingSteps = 3;
    showText = true;
    tilesPerThread = 12;
    zBiasLines = 0.001f;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsRaytracerAdvanced" << ":\n";
    os << "  backgroundColorReflections = " << backgroundColorReflections << "\n";
    os << "  searchTreeFactor = " << searchTreeFactor << "\n";
    os << "  shadowScalingFactor = " << shadowScalingFactor << "\n";
    os << "  shadowSmoothingSteps = " << shadowSmoothingSteps << "\n";
    os << "  showText = " << showText << "\n";
    os << "  tilesPerThread = " << tilesPerThread << "\n";
    os << "  zBiasLines = " << zBiasLines << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsRaytracerAdvanced& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsRaytracer
* @brief        Settings for raytracer (software renderer) which can be used as alternative to classic OpenGL rendering; this option may be erased in future in favor of a modern GPU rendering. To activate the raytracer, simply switch the enable flag to True. The raytracer uses CPU-based rendering and is therefore comparably slow (may take seconds to render one frame). Thus, take care with the window dimension (start with small window size like 400 x 300) and use openGL.multiSampling=1. Note that many parameters are used from openGL settings, like backgroundColor, lineWidth, multiSampling, shadow (only on/off), and lights. See the options to improve appearance and performance.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsRaytracer // AUTO: 
{
public: // AUTO: 
  VSettingsRaytracerAdvanced advanced;            //!< AUTO: advanced settings for raytracer
  VSettingsMaterial material0;                    //!< AUTO: settings for material0
  VSettingsMaterial material1;                    //!< AUTO: settings for material1
  VSettingsMaterial material2;                    //!< AUTO: settings for material2
  VSettingsMaterial material3;                    //!< AUTO: settings for material3
  VSettingsMaterial material4;                    //!< AUTO: settings for material4
  VSettingsMaterial material5;                    //!< AUTO: settings for material5
  VSettingsMaterial material6;                    //!< AUTO: settings for material6
  VSettingsMaterial material7;                    //!< AUTO: settings for material7
  VSettingsMaterial material8;                    //!< AUTO: settings for material8
  VSettingsMaterial material9;                    //!< AUTO: settings for material9
  Float4 globalFogColor;                          //!< AUTO: scene RGBA fog color
  float globalFogDensity;                         //!< AUTO: must be >= 0; global fog density; fog is deactivated if fogDensity=0, otherwise it is a density relative to scene max size; as it is relative, the factor has to be relatively high to be visible (usually >1)
  Index imageSizeFactor;                          //!< AUTO: must be > 0; Special size factor (1-16) to allow drawing with smaller resolution (faster); use this for long rendering times for adjustments, etc.
  bool keepWindowActive;                          //!< AUTO: Special flag, handle with care; True: sends some glfw functions to keep window reactive for long render times (>2 seconds); otherwise, the rendering may not finish due to timeout
  Index lightRadiusVariations;                    //!< AUTO: must be > 0; if lightRadiusVariations>1, this defines the number of positions that are used to compute the effect of distributed lights (larger is slower but better quality); range=1..256; avoid squares of integers; good values: 1 (hard shadow boundaries), 6, 13, 20, 31, 72, 130, 240; for lower values, use shadowSmoothingSteps=2..8
  Index maxReflectionDepth;                       //!< AUTO: must be >= 0; Maximum number of reflections computed for one ray (note that for each transparent face passed, the reflection depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  Index maxTransparencyDepth;                     //!< AUTO: must be >= 0; Maximum number of transparent faces that can be passed (note that for each reflection, the transparency depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  Index multiSampling;                            //!< AUTO: must be > 0; Multi-sampling used for rendering of faces, lines and text; increases image quality along edges (lines, etc.) but INCREASES rendering costs dramatically (multiSampling=3 => 3x3=9 times slower); also used for shadow if shadowScalingFactor=0; values only accepted in range [1..4]
  Index numberOfThreads;                          //!< AUTO: must be > 0; Number of CPU-threads (max: 256) used for software rendering (should be approx. the number of available threads)
  Index verbose;                                  //!< AUTO: 1: print out some debug information on rendering, in particular rendering timings and counter; 2 and higher: advanced debug information

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsRaytracer()
  {
    backlink=nullptr;
    globalFogColor = Float4({0.5f,0.5f,0.5f,1.f});
    globalFogDensity = 0.;
    imageSizeFactor = 1;
    keepWindowActive = false;
    lightRadiusVariations = 1;
    maxReflectionDepth = 2;
    maxTransparencyDepth = 2;
    multiSampling = 1;
    numberOfThreads = 8;
    verbose = 0;
    material0.name = "default";
    material0.baseColor = Float3({0.4f,0.4f,0.9f});
    material0.specular = Float3({0.6f,0.6f,0.6f});
    material1.name = "matt";
    material1.baseColor = Float3({0.f,1.f,0.f});
    material1.specular = Float3({0.3f,0.3f,0.3f});
    material1.shininess = 5.f;
    material2.name = "steel";
    material2.baseColor = Float3({0.6f,0.6f,0.6f});
    material2.specular = Float3({0.3f,0.33f,0.4f});
    material2.shininess = 25.f;
    material2.reflectivity = 0.1f;
    material3.name = "plastic";
    material3.baseColor = Float3({1.f,0.f,0.f});
    material3.specular = Float3({0.4f,0.45f,0.45f});
    material3.shininess = 20.f;
    material3.reflectivity = 0.1f;
    material4.name = "chrome";
    material4.baseColor = Float3({0.75f,0.75f,0.75f});
    material4.specular = Float3({0.6f,0.62f,0.67f});
    material4.shininess = 60.f;
    material4.reflectivity = 0.25f;
    material5.name = "shiny";
    material5.baseColor = Float3({1.f,0.5f,0.f});
    material5.specular = Float3({0.7f,0.65f,0.7f});
    material5.shininess = 100.f;
    material5.reflectivity = 0.5f;
    material6.name = "transparent";
    material6.baseColor = Float3({0.75f,0.75f,0.75f});
    material6.specular = Float3({0.4f,0.4f,0.45f});
    material6.shininess = 20.f;
    material6.ior = 1.05f;
    material6.alpha = 0.3f;
    material7.name = "glass";
    material7.baseColor = Float3({0.8f,0.8f,0.8f});
    material7.specular = Float3({0.6f,0.68f,0.63f});
    material7.shininess = 50.f;
    material7.reflectivity = 0.6f;
    material7.ior = 1.5f;
    material7.alpha = 0.15f;
    material8.name = "mirror";
    material8.baseColor = Float3({0.8f,0.8f,0.8f});
    material8.specular = Float3({0.4f,0.4f,0.4f});
    material8.shininess = 50.f;
    material8.reflectivity = 0.8f;
    material9.name = "emission";
    material9.baseColor = Float3({0.85f,0.85f,0.7f});
    material9.specular = Float3({0.6f,0.6f,0.6f});
    material9.shininess = 20.f;
    material9.emission = Float3({0.8f,0.8f,0.7f});
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    advanced.Init(backlinkInit);
    material0.Init(backlinkInit);
    material1.Init(backlinkInit);
    material2.Init(backlinkInit);
    material3.Init(backlinkInit);
    material4.Init(backlinkInit);
    material5.Init(backlinkInit);
    material6.Init(backlinkInit);
    material7.Init(backlinkInit);
    material8.Init(backlinkInit);
    material9.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.lightModelAmbient
  void PySetAmbientLightColor(const std::array<float,4>& lightModelAmbientInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.lightModelAmbient
  std::array<float,4> PyGetAmbientLightColor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use raytracer.advanced.backgroundColorReflections
  void PySetBackgroundColorReflections(const std::array<float,4>& backgroundColorReflectionsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use raytracer.advanced.backgroundColorReflections
  std::array<float,4> PyGetBackgroundColorReflections() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.useRaytracer
  void PySetEnable(const bool& useRaytracerInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.useRaytracer
  bool PyGetEnable() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.lightRadius
  void PySetLightRadius(const float& lightRadiusInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.lightRadius
  float PyGetLightRadius() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use raytracer.advanced.searchTreeFactor
  void PySetSearchTreeFactor(const Index& searchTreeFactorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use raytracer.advanced.searchTreeFactor
  Index PyGetSearchTreeFactor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use raytracer.advanced.shadowScalingFactor
  void PySetShadowScalingFactor(const Index& shadowScalingFactorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use raytracer.advanced.shadowScalingFactor
  Index PyGetShadowScalingFactor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use raytracer.advanced.shadowSmoothingSteps
  void PySetShadowSmoothingSteps(const Index& shadowSmoothingStepsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use raytracer.advanced.shadowSmoothingSteps
  Index PyGetShadowSmoothingSteps() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use raytracer.advanced.showText
  void PySetShowText(const bool& showTextInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use raytracer.advanced.showText
  bool PyGetShowText() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use raytracer.advanced.tilesPerThread
  void PySetTilesPerThread(const Index& tilesPerThreadInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use raytracer.advanced.tilesPerThread
  Index PyGetTilesPerThread() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use raytracer.advanced.zBiasLines
  void PySetZBiasLines(const float& zBiasLinesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use raytracer.advanced.zBiasLines
  float PyGetZBiasLines() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.dummy
  void PySetZOffsetCamera(const float& dummyInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.dummy
  float PyGetZOffsetCamera() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsRaytracer" << ":\n";
    os << "  advanced = " << advanced << "\n";
    os << "  material0 = " << material0 << "\n";
    os << "  material1 = " << material1 << "\n";
    os << "  material2 = " << material2 << "\n";
    os << "  material3 = " << material3 << "\n";
    os << "  material4 = " << material4 << "\n";
    os << "  material5 = " << material5 << "\n";
    os << "  material6 = " << material6 << "\n";
    os << "  material7 = " << material7 << "\n";
    os << "  material8 = " << material8 << "\n";
    os << "  material9 = " << material9 << "\n";
    os << "  globalFogColor = " << globalFogColor << "\n";
    os << "  globalFogDensity = " << globalFogDensity << "\n";
    os << "  imageSizeFactor = " << imageSizeFactor << "\n";
    os << "  keepWindowActive = " << keepWindowActive << "\n";
    os << "  lightRadiusVariations = " << lightRadiusVariations << "\n";
    os << "  maxReflectionDepth = " << maxReflectionDepth << "\n";
    os << "  maxTransparencyDepth = " << maxTransparencyDepth << "\n";
    os << "  multiSampling = " << multiSampling << "\n";
    os << "  numberOfThreads = " << numberOfThreads << "\n";
    os << "  verbose = " << verbose << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsRaytracer& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsOpenGLAdvanced
* @brief        Advanced settings for openGL.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsOpenGLAdvanced // AUTO: 
{
public: // AUTO: 
  Float4 clippingPlaneColor;                      //!< AUTO: RGBA color for clipping plane; if alpha-channel is 0, the cutting plane is not drawn; if alpha-channel is 1, the clippingPlaneColor is used; if alpha-channel is 2, the color of the object interior is used as clipping plane color (which may look strange in case of object-in-object); see also view.camera for clipping plane options
  Index curvedTriangleMaxTiling;                  //!< AUTO: must be >= 0; maximum number of subdivisions per edge of a 6-node (curved) triangle; see curvedTriangleTilingAngle
  float curvedTriangleTilingAngle;                //!< AUTO: must be >= 0; 6-node (curved) triangles of a TriangleList (key triangles6) are drawn as flat triangles, split when they are drawn: each edge is subdivided until the angle between its end tangents, and between the given normals of its nodes, falls below this angle (in degrees), at most curvedTriangleMaxTiling times, and the inside follows its three edges - 15 degrees give 24 segments around a full cylinder, and a surface curved in one direction is not subdivided along the other; 0 draws each as 1 flat triangle; used by the raytracer as well
  bool depthSorting;                              //!< AUTO: True (slower): sort triangles by Z-depth to remove transparency artifacts: only works if triangles do not intersect or come close (you may like to refine triangle meshes); False: no depth-sort (faster)
  bool enableLighting;                            //!< AUTO: generally enable lighting (otherwise, colors of objects are used); OpenGL: glEnable(GL_LIGHTING)
  Float4 faceNormalsColor;                        //!< AUTO: global RGBA color for face normals
  Float3 initialCenterPoint;                      //!< AUTO: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True; only used in case that modelCentricView=True
  float initialMaxSceneSize;                      //!< AUTO: must be > 0; initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  StdArray33F initialModelRotation;               //!< AUTO: initial model rotation matrix for OpenGl; in python use e.g.: initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]; only used in case that modelCentricView=True
  float initialZoom;                              //!< AUTO: must be >= 0; initial zoom of scene; overwritten/ignored if autoFitScene = True
  bool lightModelLocalViewer;                     //!< AUTO: True: the camera origin is used to compute shininess effects (more realistic); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,...)
  bool lightModelTwoSide;                         //!< AUTO: enlighten also backside of object; may cause problems on some graphics cards and lead to slower performance; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,...)
  bool lineSmooth;                                //!< AUTO: draw lines smooth
  float polygonOffset;                            //!< AUTO: general polygon offset for polygons, except for shadows; use this parameter to draw polygons behind lines to reduce artifacts for very large or small models
  bool shadeModelSmooth;                          //!< AUTO: True: turn on smoothing for shaders, which uses vertex normals to smooth surfaces
  float shadowPolygonOffset;                      //!< AUTO: must be > 0; some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
  bool showBoundingBox;                           //!< AUTO: show scene bounding box (red), as available in renderState.boundingBox; NOTE that the bounding box is only updated with ZoomAll or at startup; this is a debug flag and it may show reasongs for strange ZoomAll behavior, as ZoomAll should zoom to the bounding box; does only work for perspective=0
  bool textLineSmooth;                            //!< AUTO: draw lines for representation of text smooth
  float textLineWidth;                            //!< AUTO: must be >= 0; width of lines used for representation of text
  Float4 vertexNormalsColor;                      //!< AUTO: global RGBA color for vertex normals

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsOpenGLAdvanced()
  {
    backlink=nullptr;
    clippingPlaneColor = Float4({0.7f,0.5f,0.5f,0.f});
    curvedTriangleMaxTiling = 8;
    curvedTriangleTilingAngle = 15.f;
    depthSorting = false;
    enableLighting = true;
    faceNormalsColor = Float4({0.8f,0.2f,0.2f,1.f});
    initialCenterPoint = Float3({0.f,0.f,0.f});
    initialMaxSceneSize = 1.f;
    initialModelRotation = EXUmath::Matrix3DFToStdArray33(Matrix3DF(3,3,{1.f,0.f,0.f, 0.f,1.f,0.f, 0.f,0.f,1.f}));
    initialZoom = 1.f;
    lightModelLocalViewer = false;
    lightModelTwoSide = false;
    lineSmooth = true;
    polygonOffset = 0.05f;
    shadeModelSmooth = true;
    shadowPolygonOffset = 0.1f;
    showBoundingBox = false;
    textLineSmooth = false;
    textLineWidth = 1.f;
    vertexNormalsColor = Float4({0.8f,0.2f,0.2f,1.f});
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsOpenGLAdvanced" << ":\n";
    os << "  clippingPlaneColor = " << clippingPlaneColor << "\n";
    os << "  curvedTriangleMaxTiling = " << curvedTriangleMaxTiling << "\n";
    os << "  curvedTriangleTilingAngle = " << curvedTriangleTilingAngle << "\n";
    os << "  depthSorting = " << depthSorting << "\n";
    os << "  enableLighting = " << enableLighting << "\n";
    os << "  faceNormalsColor = " << faceNormalsColor << "\n";
    os << "  initialCenterPoint = " << initialCenterPoint << "\n";
    os << "  initialMaxSceneSize = " << initialMaxSceneSize << "\n";
#ifndef __APPLE__
    os << "  initialModelRotation = " << Matrix3DF(initialModelRotation) << "\n";
#endif
    os << "  initialZoom = " << initialZoom << "\n";
    os << "  lightModelLocalViewer = " << lightModelLocalViewer << "\n";
    os << "  lightModelTwoSide = " << lightModelTwoSide << "\n";
    os << "  lineSmooth = " << lineSmooth << "\n";
    os << "  polygonOffset = " << polygonOffset << "\n";
    os << "  shadeModelSmooth = " << shadeModelSmooth << "\n";
    os << "  shadowPolygonOffset = " << shadowPolygonOffset << "\n";
    os << "  showBoundingBox = " << showBoundingBox << "\n";
    os << "  textLineSmooth = " << textLineSmooth << "\n";
    os << "  textLineWidth = " << textLineWidth << "\n";
    os << "  vertexNormalsColor = " << vertexNormalsColor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsOpenGLAdvanced& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsLight
* @brief        Settings for lights.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsLight // AUTO: 
{
public: // AUTO: 
  float constantAttenuation;                      //!< AUTO: constant attenuation coefficient of this light, this is a constant factor that attenuates the light source; attenuation factor = 1/(kc +kl*d + kq*d*d); (kc,kl,kq)=(1,0,0) means no attenuation; only used for lights, where last component of light position is 1
  float diffuse;                                  //!< AUTO: diffuse value of this light
  bool enable;                                    //!< AUTO: turn this light on or off; the four lights light0 to light3 of visualizationSettings.openGL are OpenGL GL_LIGHT0 to GL_LIGHT3, and each of them can cast a shadow - see shadow below
  float lightRadius;                              //!< AUTO: only used by raytracers: radius of light used to compute smooth shadows (approximated by raytracer.lightRadiusVariations); if lightRadiusVariations>1, this value defines the radius of the light, converting point lights into distributed lights (slower)
  float linearAttenuation;                        //!< AUTO: linear attenuation coefficient of this light, this is a linear factor for attenuation of the light source with distance
  Float4 position;                                //!< AUTO: 4D position vector of this light; the 4th value should be 0 for directional lights that are (almost) infinitely far away, like the sun, but 1 for position-based lights (and for the attenuation factor to be computed); if this light casts a shadow, its position decides where the shadow falls, so it has to be at a reasonable place for the scene; see opengl manuals
  float quadraticAttenuation;                     //!< AUTO: quadratic attenuation coefficient of this light, this is a quadratic factor for attenuation of the light source with distance
  float shadow;                                   //!< AUTO: must be >= 0; in OpenGL renderer, the shadow parameter \f$\in [0 ... 1]\f$ prescribes the amount of shadow of this light that is added to the scene, using its position (or only its direction); every light can cast a shadow and the effects accumulate; if this parameter is different from 0, rendering of triangles becomes approx. 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows; for raytracer, shadow is included by a physics-based model for each light if shadow>0, accumulating effects of each light source; the openGL renderer computes shadows with shadow volumes and approximates a directional light by enlarging its direction to a multiple of maxSceneSize, while the raytracer uses the direction itself
  float specular;                                 //!< AUTO: specular value of this light
  bool useCameraFrame;                            //!< AUTO: set False to set light positions and directions relative to model frame; True: lights are in camera frame, not following the visual transformations; this was True up to Exudyn 1.9.174

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsLight()
  {
    backlink=nullptr;
    constantAttenuation = 1.f;
    diffuse = 0.5f;
    enable = true;
    lightRadius = 0.1f;
    linearAttenuation = 0.f;
    position = Float4({2.f,2.f,10.f,0.f});
    quadraticAttenuation = 0.f;
    shadow = 0.f;
    specular = 0.5f;
    useCameraFrame = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsLight" << ":\n";
    os << "  constantAttenuation = " << constantAttenuation << "\n";
    os << "  diffuse = " << diffuse << "\n";
    os << "  enable = " << enable << "\n";
    os << "  lightRadius = " << lightRadius << "\n";
    os << "  linearAttenuation = " << linearAttenuation << "\n";
    os << "  position = " << position << "\n";
    os << "  quadraticAttenuation = " << quadraticAttenuation << "\n";
    os << "  shadow = " << shadow << "\n";
    os << "  specular = " << specular << "\n";
    os << "  useCameraFrame = " << useCameraFrame << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsLight& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsOpenGL
* @brief        OpenGL settings for 2D and 3D rendering - with many settings also used for raytracer. For further details and backgrounds also see OpenGL 1.3 functionality on the web.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsOpenGL // AUTO: 
{
public: // AUTO: 
  VSettingsOpenGLAdvanced advanced;               //!< AUTO: advanced settings for openGL
  VSettingsLight light0;                          //!< AUTO: settings for light0 and shadow
  VSettingsLight light1;                          //!< AUTO: settings for light1 and shadow
  VSettingsLight light2;                          //!< AUTO: settings for light2 and shadow
  VSettingsLight light3;                          //!< AUTO: settings for light3 and shadow
  bool drawFaceNormals;                           //!< AUTO: draws triangle normals, e.g. at center of triangles; used for debugging of faces
  float drawNormalsLength;                        //!< AUTO: must be > 0; length of normals; used for debugging
  bool drawVertexNormals;                         //!< AUTO: draws vertex normals; used for debugging
  Float4 faceEdgesColor;                          //!< AUTO: global RGBA color for face edges
  float faceTransparencyGlobal;                   //!< AUTO: must be >= 0; in case that facesTransparent=True this represents the max alpha-transparency
  Float4 lightModelAmbient;                       //!< AUTO: global ambient light (needed for faces that are close to orthogonal to light or faces in shadow region); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a]); also used by raytracer
  float lineWidth;                                //!< AUTO: must be >= 0; width of lines used for representation of lines, circles, points, etc.
  float materialShininess;                        //!< AUTO: shininess of material
  Float4 materialSpecular;                        //!< AUTO: RGBA specular color of material
  Index multiSampling;                            //!< AUTO: must be > 0; NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 3, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  float zMaxSceneFactor;                          //!< AUTO: must be > 0; factor multiplied with maxSceneSize to avoid clipping of modelview; larger values reduce clipping of near or far objects, but may lead to artifacts (so-called Z-fighting)
  float dummy;                                    //!< AUTO: unused dummy variable, used to redirect deprecated values

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsOpenGL()
  {
    backlink=nullptr;
    drawFaceNormals = false;
    drawNormalsLength = 0.1f;
    drawVertexNormals = false;
    dummy = 0.f;
    faceEdgesColor = Float4({0.2f,0.2f,0.2f,1.f});
    faceTransparencyGlobal = 0.4f;
    lightModelAmbient = Float4({0.4f,0.4f,0.4f,1.f});
    lineWidth = 1.f;
    materialShininess = 32.f;
    materialSpecular = Float4({0.6f,0.6f,0.6f,1.f});
    multiSampling = 1;
    zMaxSceneFactor = 2.f;
    light1.diffuse = 0.25f;
    light1.specular = 0.25f;
    light1.position = Float4({2.f,2.f,-10.f,0.f});
    light2.diffuse = 0.2f;
    light2.specular = 0.2f;
    light2.enable = false;
    light3.diffuse = 0.2f;
    light3.specular = 0.2f;
    light3.enable = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    advanced.Init(backlinkInit);
    light0.Init(backlinkInit);
    light1.Init(backlinkInit);
    light2.Init(backlinkInit);
    light3.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.clippingPlaneColor
  void PySetClippingPlaneColor(const std::array<float,4>& clippingPlaneColorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.clippingPlaneColor
  std::array<float,4> PyGetClippingPlaneColor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.clippingPlaneDistance
  void PySetClippingPlaneDistance(const float& clippingPlaneDistanceInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.clippingPlaneDistance
  float PyGetClippingPlaneDistance() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.clippingPlaneNormal
  void PySetClippingPlaneNormal(const std::array<float,3>& clippingPlaneNormalInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.clippingPlaneNormal
  std::array<float,3> PyGetClippingPlaneNormal() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.depthSorting
  void PySetDepthSorting(const bool& depthSortingInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.depthSorting
  bool PyGetDepthSorting() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.enable
  void PySetEnableLight0(const bool& enableInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.enable
  bool PyGetEnableLight0() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light1.enable
  void PySetEnableLight1(const bool& enableInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light1.enable
  bool PyGetEnableLight1() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.enableLighting
  void PySetEnableLighting(const bool& enableLightingInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.enableLighting
  bool PyGetEnableLighting() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.facesTransparent
  void PySetFacesTransparent(const bool& facesTransparentInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.facesTransparent
  bool PyGetFacesTransparent() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.initialCenterPoint
  void PySetInitialCenterPoint(const std::array<float,3>& initialCenterPointInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.initialCenterPoint
  std::array<float,3> PyGetInitialCenterPoint() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.initialMaxSceneSize
  void PySetInitialMaxSceneSize(const float& initialMaxSceneSizeInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.initialMaxSceneSize
  float PyGetInitialMaxSceneSize() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.initialModelRotation
  void PySetInitialModelRotation(const StdArray33F& initialModelRotationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.initialModelRotation
  StdArray33F PyGetInitialModelRotation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.initialZoom
  void PySetInitialZoom(const float& initialZoomInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.initialZoom
  float PyGetInitialZoom() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.dummy
  void PySetLight0ambient(const float& dummyInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.dummy
  float PyGetLight0ambient() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.constantAttenuation
  void PySetLight0constantAttenuation(const float& constantAttenuationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.constantAttenuation
  float PyGetLight0constantAttenuation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.diffuse
  void PySetLight0diffuse(const float& diffuseInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.diffuse
  float PyGetLight0diffuse() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.linearAttenuation
  void PySetLight0linearAttenuation(const float& linearAttenuationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.linearAttenuation
  float PyGetLight0linearAttenuation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.position
  void PySetLight0position(const std::array<float,4>& positionInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.position
  std::array<float,4> PyGetLight0position() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.quadraticAttenuation
  void PySetLight0quadraticAttenuation(const float& quadraticAttenuationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.quadraticAttenuation
  float PyGetLight0quadraticAttenuation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.specular
  void PySetLight0specular(const float& specularInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.specular
  float PyGetLight0specular() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.dummy
  void PySetLight1ambient(const float& dummyInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.dummy
  float PyGetLight1ambient() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light1.constantAttenuation
  void PySetLight1constantAttenuation(const float& constantAttenuationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light1.constantAttenuation
  float PyGetLight1constantAttenuation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light1.diffuse
  void PySetLight1diffuse(const float& diffuseInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light1.diffuse
  float PyGetLight1diffuse() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light1.linearAttenuation
  void PySetLight1linearAttenuation(const float& linearAttenuationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light1.linearAttenuation
  float PyGetLight1linearAttenuation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light1.position
  void PySetLight1position(const std::array<float,4>& positionInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light1.position
  std::array<float,4> PyGetLight1position() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light1.quadraticAttenuation
  void PySetLight1quadraticAttenuation(const float& quadraticAttenuationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light1.quadraticAttenuation
  float PyGetLight1quadraticAttenuation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light1.specular
  void PySetLight1specular(const float& specularInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light1.specular
  float PyGetLight1specular() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.lightModelLocalViewer
  void PySetLightModelLocalViewer(const bool& lightModelLocalViewerInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.lightModelLocalViewer
  bool PyGetLightModelLocalViewer() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.lightModelTwoSide
  void PySetLightModelTwoSide(const bool& lightModelTwoSideInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.lightModelTwoSide
  bool PyGetLightModelTwoSide() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.useCameraFrame
  void PySetLightPositionsInCameraFrame(const bool& useCameraFrameInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.useCameraFrame
  bool PyGetLightPositionsInCameraFrame() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.lineSmooth
  void PySetLineSmooth(const bool& lineSmoothInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.lineSmooth
  bool PyGetLineSmooth() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.materialSpecular
  void PySetMaterialAmbientAndDiffuse(const std::array<float,4>& materialSpecularInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.materialSpecular
  std::array<float,4> PyGetMaterialAmbientAndDiffuse() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.perspective
  void PySetPerspective(const float& perspectiveInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.perspective
  float PyGetPerspective() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.polygonOffset
  void PySetPolygonOffset(const float& polygonOffsetInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.polygonOffset
  float PyGetPolygonOffset() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.shadeModelSmooth
  void PySetShadeModelSmooth(const bool& shadeModelSmoothInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.shadeModelSmooth
  bool PyGetShadeModelSmooth() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.shadow
  void PySetShadow(const float& shadowInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.shadow
  float PyGetShadow() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.shadowPolygonOffset
  void PySetShadowPolygonOffset(const float& shadowPolygonOffsetInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.shadowPolygonOffset
  float PyGetShadowPolygonOffset() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.showFaceEdges
  void PySetShowFaceEdges(const bool& showFaceEdgesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.showFaceEdges
  bool PyGetShowFaceEdges() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.showFaces
  void PySetShowFaces(const bool& showFacesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.showFaces
  bool PyGetShowFaces() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.showLines
  void PySetShowLines(const bool& showLinesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.showLines
  bool PyGetShowLines() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.showMeshEdges
  void PySetShowMeshEdges(const bool& showMeshEdgesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.showMeshEdges
  bool PyGetShowMeshEdges() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.showMeshFaces
  void PySetShowMeshFaces(const bool& showMeshFacesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.showMeshFaces
  bool PyGetShowMeshFaces() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.textLineSmooth
  void PySetTextLineSmooth(const bool& textLineSmoothInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.textLineSmooth
  bool PyGetTextLineSmooth() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.advanced.textLineWidth
  void PySetTextLineWidth(const float& textLineWidthInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.advanced.textLineWidth
  float PyGetTextLineWidth() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsOpenGL" << ":\n";
    os << "  advanced = " << advanced << "\n";
    os << "  light0 = " << light0 << "\n";
    os << "  light1 = " << light1 << "\n";
    os << "  light2 = " << light2 << "\n";
    os << "  light3 = " << light3 << "\n";
    os << "  drawFaceNormals = " << drawFaceNormals << "\n";
    os << "  drawNormalsLength = " << drawNormalsLength << "\n";
    os << "  drawVertexNormals = " << drawVertexNormals << "\n";
    os << "  dummy = " << dummy << "\n";
    os << "  faceEdgesColor = " << faceEdgesColor << "\n";
    os << "  faceTransparencyGlobal = " << faceTransparencyGlobal << "\n";
    os << "  lightModelAmbient = " << lightModelAmbient << "\n";
    os << "  lineWidth = " << lineWidth << "\n";
    os << "  materialShininess = " << materialShininess << "\n";
    os << "  materialSpecular = " << materialSpecular << "\n";
    os << "  multiSampling = " << multiSampling << "\n";
    os << "  zMaxSceneFactor = " << zMaxSceneFactor << "\n";
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
* @brief        Functionality to export images of view0 to files (PNG or TGA format) which can be used to create animations; in order to activate image recording during the solution process, set SolutionSettings.recordImagesInterval accordingly.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsExportImages // AUTO: 
{
public: // AUTO: 
  Index heightAlignment;                          //!< AUTO: must be > 0; alignment of exported image height; using a value of 2 helps to reduce problems with video conversion (additional horizontal lines are lost)
  Index saveImageFileCounter;                     //!< AUTO: must be >= 0; current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  std::string saveImageFileName;                  //!< AUTO: filename (without extension!) and (relative) path for image file(s) with consecutive numbering (e.g., frame0000.png, frame0001.png,...); ; directory will be created if it does not exist
  std::string saveImageFormat;                    //!< AUTO: format of an exported image, `PNG` or `TGA`; `TGA` has the highest compatibility with all platforms. The drawing elements of a scene as data - lines, triangles, texts, each with the item that drew it - are `SC.renderer.GetGraphicsData()`
  bool saveImageSingleFile;                       //!< AUTO: True: only save single files with given filename, not adding numbering; False: add numbering to files, see saveImageFileName
  Index saveImageTimeOut;                         //!< AUTO: must be > 0; timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  Index widthAlignment;                           //!< AUTO: must be > 0; alignment of exported image width; using a value of 4 helps to reduce problems with video conversion (additional vertical lines are lost)

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsExportImages()
  {
    backlink=nullptr;
    heightAlignment = 2;
    saveImageFileCounter = 0;
    saveImageFileName = "images/frame";
    saveImageFormat = "PNG";
    saveImageSingleFile = false;
    saveImageTimeOut = 5000;
    widthAlignment = 4;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsExportImages" << ":\n";
    os << "  heightAlignment = " << heightAlignment << "\n";
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
* @class        VSettingsInteractiveAdvanced
* @brief        Advanced settings for interactive.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsInteractiveAdvanced // AUTO: 
{
public: // AUTO: 
  Float4 highlightColor;                          //!< AUTO: RGBA color for highlighted item; 4th value is alpha-transparency
  Float4 highlightOtherColor;                     //!< AUTO: RGBA color for other items (which are not highlighted); 4th value is alpha-transparency
  float joystickScaleRotation;                    //!< AUTO: rotation scaling factor for joystick input
  float joystickScaleTranslation;                 //!< AUTO: translation scaling factor for joystick input
  float keypressRotationStep;                     //!< AUTO: rotation increment per keypress in degree (full rotation = 360 degree)
  float keypressTranslationStep;                  //!< AUTO: translation increment per keypress relative to window size
  float mouseMoveRotationFactor;                  //!< AUTO: rotation increment per 1 pixel mouse movement in degree
  bool pauseWithSpacebar;                         //!< AUTO: True: during simulation, space bar can be pressed to pause simulation
  bool selectionHighlights;                       //!< AUTO: True: enable mouse click to highlights item (default: red)
  bool selectionLeftMouse;                        //!< AUTO: True: enable left mouse click on items to show basic information
  Index selectionLeftMouseItemTypes;              //!< AUTO: binary flags (1,2,4,8,16) for (Node,Object,Marker,Load,Sensor) that are identified with left mouse click selection
  bool selectionRightMouse;                       //!< AUTO: True: enable right mouse click on items to show dictionary (read only!)
  bool selectionRightMouseGraphicsData;           //!< AUTO: True: right mouse click on items also shows GraphicsData information for inspectation (may sometimes be very large and may not fit into dialog for large graphics objects!)
  float zoomStepFactor;                           //!< AUTO: change of zoom per keypress (keypad +/-) or mouse wheel increment

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsInteractiveAdvanced()
  {
    backlink=nullptr;
    highlightColor = Float4({0.8f,0.05f,0.05f,0.75f});
    highlightOtherColor = Float4({0.5f,0.5f,0.5f,0.4f});
    joystickScaleRotation = 200.f;
    joystickScaleTranslation = 6.f;
    keypressRotationStep = 5.f;
    keypressTranslationStep = 0.1f;
    mouseMoveRotationFactor = 1.f;
    pauseWithSpacebar = true;
    selectionHighlights = true;
    selectionLeftMouse = true;
    selectionLeftMouseItemTypes = 31;
    selectionRightMouse = true;
    selectionRightMouseGraphicsData = false;
    zoomStepFactor = 1.15f;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsInteractiveAdvanced" << ":\n";
    os << "  highlightColor = " << highlightColor << "\n";
    os << "  highlightOtherColor = " << highlightOtherColor << "\n";
    os << "  joystickScaleRotation = " << joystickScaleRotation << "\n";
    os << "  joystickScaleTranslation = " << joystickScaleTranslation << "\n";
    os << "  keypressRotationStep = " << keypressRotationStep << "\n";
    os << "  keypressTranslationStep = " << keypressTranslationStep << "\n";
    os << "  mouseMoveRotationFactor = " << mouseMoveRotationFactor << "\n";
    os << "  pauseWithSpacebar = " << pauseWithSpacebar << "\n";
    os << "  selectionHighlights = " << selectionHighlights << "\n";
    os << "  selectionLeftMouse = " << selectionLeftMouse << "\n";
    os << "  selectionLeftMouseItemTypes = " << selectionLeftMouseItemTypes << "\n";
    os << "  selectionRightMouse = " << selectionRightMouse << "\n";
    os << "  selectionRightMouseGraphicsData = " << selectionRightMouseGraphicsData << "\n";
    os << "  zoomStepFactor = " << zoomStepFactor << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsInteractiveAdvanced& object)
  {
    object.Print(os);
    return os;
  }

};


/** ***********************************************************************************************
* @class        VSettingsInteractive
* @brief        Functionality to interact with render window; includes special rotation and zoom factors, item-highlighting, marker tracking, item selection and keyPressUserFunction.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsInteractive // AUTO: 
{
public: // AUTO: 
  VSettingsInteractiveAdvanced advanced;          //!< AUTO: advanced interactive visualization settings
  bool autoRotateModelView;                       //!< AUTO: True: rotate model view with autorotation
  Float3 autoRotationVelocity;                    //!< AUTO: Angular velocity vector for auto-rotation of scene (only visualization view is rotated, not the model itself!)
  Index highlightItemIndex;                       //!< AUTO: index of item that shall be highlighted (e.g., to find item which cauess problems); if set -1, no item is highlighted
  ItemType highlightItemType;                     //!< AUTO: item type (Node, Object, ...) that shall be highlighted (e.g., to find item which cauess problems)
  Index highlightMbsNumber;                       //!< AUTO: must be >= 0; index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  bool ignoreKeys;                                //!< AUTO: True: ignore keyboard input except escape and 'F2' keys; used for interactive mode, e.g., to perform kinematic analysis; This flag can be switched with key 'F2'; if ignoreKeys=True, then keyPressUserFunction can be used!
  std::function<bool(int, int, int)> keyPressUserFunction;//!< AUTO: add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example: `def f(key, action, mods): print('key=',key)`; use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
  bool logMouseCoordinates;                       //!< AUTO: True: if showMouseCoordinates=True, also log mouse coordinates (transformed to model coordinates); only works for axis-aligned ortho-projections and shows the coordinates of the current plane
  bool useJoystickInput;                          //!< AUTO: True: read joystick input (use 6-axis joystick with lowest ID found when starting renderer window) and interpret as (x,y,z) position and (rotx, roty, rotz) rotation: as available from 3Dconnexion space mouse and maybe others as well; set to False, if external joystick makes problems ...

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsInteractive()
  {
    backlink=nullptr;
    autoRotateModelView = false;
    autoRotationVelocity = Float3({0.f,0.f,1.047198f});
    highlightItemIndex = -1;
    highlightItemType = ItemType::_None;
    highlightMbsNumber = 0;
    ignoreKeys = false;
    keyPressUserFunction = 0;
    logMouseCoordinates = true;
    useJoystickInput = true;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
    advanced.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.highlightColor
  void PySetHighlightColor(const std::array<float,4>& highlightColorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.highlightColor
  std::array<float,4> PyGetHighlightColor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.highlightOtherColor
  void PySetHighlightOtherColor(const std::array<float,4>& highlightOtherColorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.highlightOtherColor
  std::array<float,4> PyGetHighlightOtherColor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.joystickScaleRotation
  void PySetJoystickScaleRotation(const float& joystickScaleRotationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.joystickScaleRotation
  float PyGetJoystickScaleRotation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.joystickScaleTranslation
  void PySetJoystickScaleTranslation(const float& joystickScaleTranslationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.joystickScaleTranslation
  float PyGetJoystickScaleTranslation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.keypressRotationStep
  void PySetKeypressRotationStep(const float& keypressRotationStepInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.keypressRotationStep
  float PyGetKeypressRotationStep() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.keypressTranslationStep
  void PySetKeypressTranslationStep(const float& keypressTranslationStepInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.keypressTranslationStep
  float PyGetKeypressTranslationStep() const ;

  //! AUTO: Set function (needed in pybind) for: add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example: `def f(key, action, mods): print('key=',key)`; use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
  void PySetKeyPressUserFunction(const std::function<bool(int, int, int)>& keyPressUserFunctionInit) { keyPressUserFunction= (const std::function<bool(int, int, int)>&)keyPressUserFunctionInit; }
  //! AUTO: Read (Copy) access to: add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example: `def f(key, action, mods): print('key=',key)`; use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
  std::function<bool(int, int, int)> PyGetKeyPressUserFunction() const { return std::function<bool(int, int, int)>(keyPressUserFunction); }

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.lockModelView
  void PySetLockModelView(const bool& lockModelViewInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.lockModelView
  bool PyGetLockModelView() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.mouseMoveRotationFactor
  void PySetMouseMoveRotationFactor(const float& mouseMoveRotationFactorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.mouseMoveRotationFactor
  float PyGetMouseMoveRotationFactor() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.pauseWithSpacebar
  void PySetPauseWithSpacebar(const bool& pauseWithSpacebarInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.pauseWithSpacebar
  bool PyGetPauseWithSpacebar() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.selectionHighlights
  void PySetSelectionHighlights(const bool& selectionHighlightsInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.selectionHighlights
  bool PyGetSelectionHighlights() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.selectionLeftMouse
  void PySetSelectionLeftMouse(const bool& selectionLeftMouseInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.selectionLeftMouse
  bool PyGetSelectionLeftMouse() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.selectionLeftMouseItemTypes
  void PySetSelectionLeftMouseItemTypes(const Index& selectionLeftMouseItemTypesInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.selectionLeftMouseItemTypes
  Index PyGetSelectionLeftMouseItemTypes() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.selectionRightMouse
  void PySetSelectionRightMouse(const bool& selectionRightMouseInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.selectionRightMouse
  bool PyGetSelectionRightMouse() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.selectionRightMouseGraphicsData
  void PySetSelectionRightMouseGraphicsData(const bool& selectionRightMouseGraphicsDataInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.selectionRightMouseGraphicsData
  bool PyGetSelectionRightMouseGraphicsData() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.trackMarker
  void PySetTrackMarker(const Index& trackMarkerInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.trackMarker
  Index PyGetTrackMarker() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.trackMarkerMbsNumber
  void PySetTrackMarkerMbsNumber(const Index& trackMarkerMbsNumberInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.trackMarkerMbsNumber
  Index PyGetTrackMarkerMbsNumber() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.trackMarkerOrientation
  void PySetTrackMarkerOrientation(const std::array<float,3>& trackMarkerOrientationInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.trackMarkerOrientation
  std::array<float,3> PyGetTrackMarkerOrientation() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.camera.trackMarkerPosition
  void PySetTrackMarkerPosition(const std::array<float,3>& trackMarkerPositionInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.camera.trackMarkerPosition
  std::array<float,3> PyGetTrackMarkerPosition() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.zoomStepFactor
  void PySetZoomStepFactor(const float& zoomStepFactorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.zoomStepFactor
  float PyGetZoomStepFactor() const ;

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsInteractive" << ":\n";
    os << "  advanced = " << advanced << "\n";
    os << "  autoRotateModelView = " << autoRotateModelView << "\n";
    os << "  autoRotationVelocity = " << autoRotationVelocity << "\n";
    os << "  highlightItemIndex = " << highlightItemIndex << "\n";
    os << "  highlightItemType = " << highlightItemType << "\n";
    os << "  highlightMbsNumber = " << highlightMbsNumber << "\n";
    os << "  ignoreKeys = " << ignoreKeys << "\n";
    os << "  logMouseCoordinates = " << logMouseCoordinates << "\n";
    os << "  useJoystickInput = " << useJoystickInput << "\n";
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
* @brief        Top structure for all visualization settings in Exudyn
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-10-04 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
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
  VSettingsRaytracer raytracer;                   //!< AUTO: Raytracer settings (builds on OpenGL rendering settings)
  VSettingsSensors sensors;                       //!< AUTO: sensor visualization settings
  VSettingsView view0;                            //!< AUTO: Settings for main view 0
  VSettingsView view1;                            //!< AUTO: Settings for sub-view 1
  VSettingsView view2;                            //!< AUTO: Settings for sub-view 2
  VSettingsView view3;                            //!< AUTO: Settings for sub-view 3
  VSettingsWindowDeprecated window;               //!< AUTO: DEPRECATED; Instead use Deprecated visualization settings for window; DO NOT USE


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VisualizationSettings()
  {
    Init(this);
  };
  //! AUTO: copy constructor: a copy links ITSELF, not the original (#2603)
  VisualizationSettings(const VisualizationSettings& other)
  {
    bodies = other.bodies;
    connectors = other.connectors;
    contact = other.contact;
    contour = other.contour;
    dialogs = other.dialogs;
    exportImages = other.exportImages;
    general = other.general;
    interactive = other.interactive;
    loads = other.loads;
    markers = other.markers;
    nodes = other.nodes;
    openGL = other.openGL;
    raytracer = other.raytracer;
    sensors = other.sensors;
    view0 = other.view0;
    view1 = other.view1;
    view2 = other.view2;
    view3 = other.view3;
    window = other.window;
    Init(this);
  }
  //! AUTO: copy assignment, for the same reason
  VisualizationSettings& operator=(const VisualizationSettings& other)
  {
    if (this != &other)
    {
      bodies = other.bodies;
      connectors = other.connectors;
      contact = other.contact;
      contour = other.contour;
      dialogs = other.dialogs;
      exportImages = other.exportImages;
      general = other.general;
      interactive = other.interactive;
      loads = other.loads;
      markers = other.markers;
      nodes = other.nodes;
      openGL = other.openGL;
      raytracer = other.raytracer;
      sensors = other.sensors;
      view0 = other.view0;
      view1 = other.view1;
      view2 = other.view2;
      view3 = other.view3;
      window = other.window;
      Init(this);
    }
    return *this;
  }
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    bodies.Init(backlinkInit);
    connectors.Init(backlinkInit);
    contact.Init(backlinkInit);
    contour.Init(backlinkInit);
    dialogs.Init(backlinkInit);
    exportImages.Init(backlinkInit);
    general.Init(backlinkInit);
    interactive.Init(backlinkInit);
    loads.Init(backlinkInit);
    markers.Init(backlinkInit);
    nodes.Init(backlinkInit);
    openGL.Init(backlinkInit);
    raytracer.Init(backlinkInit);
    sensors.Init(backlinkInit);
    view0.Init(backlinkInit);
    view1.Init(backlinkInit);
    view2.Init(backlinkInit);
    view3.Init(backlinkInit);
    window.Init(backlinkInit);
  }

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
    os << "  raytracer = " << raytracer << "\n";
    os << "  sensors = " << sensors << "\n";
    os << "  view0 = " << view0 << "\n";
    os << "  view1 = " << view1 << "\n";
    os << "  view2 = " << view2 << "\n";
    os << "  view3 = " << view3 << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VisualizationSettings& object)
  {
    object.Print(os);
    return os;
  }

};




//! implementation:

inline void VSettingsGeneral::PySetDrawCoordinateSystem(const Index& drawCoordinateSystemInit) { 
    PyDeprecated("visualizationSettings", "general.drawCoordinateSystem", "VisualizationSettings parameter general.drawCoordinateSystem is deprecated! use view0.scene.drawCoordinateSystem instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.drawCoordinateSystem is deprecated and forwards to view0.scene.drawCoordinateSystem, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.drawCoordinateSystem= (const Index&)drawCoordinateSystemInit; 
    }
inline Index VSettingsGeneral::PyGetDrawCoordinateSystem() const { 
    PyDeprecated("visualizationSettings", "general.drawCoordinateSystem", "VisualizationSettings parameter general.drawCoordinateSystem is deprecated! use view0.scene.drawCoordinateSystem instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.drawCoordinateSystem is deprecated and forwards to view0.scene.drawCoordinateSystem, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->view0.scene.drawCoordinateSystem); 
    }

inline void VSettingsGeneral::PySetDrawWorldBasis(const bool& drawWorldBasisInit) { 
    PyDeprecated("visualizationSettings", "general.drawWorldBasis", "VisualizationSettings parameter general.drawWorldBasis is deprecated! use view0.scene.drawWorldBasis instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.drawWorldBasis is deprecated and forwards to view0.scene.drawWorldBasis, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.drawWorldBasis= (const bool&)drawWorldBasisInit; 
    }
inline bool VSettingsGeneral::PyGetDrawWorldBasis() const { 
    PyDeprecated("visualizationSettings", "general.drawWorldBasis", "VisualizationSettings parameter general.drawWorldBasis is deprecated! use view0.scene.drawWorldBasis instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.drawWorldBasis is deprecated and forwards to view0.scene.drawWorldBasis, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.scene.drawWorldBasis); 
    }

inline void VSettingsGeneral::PySetShowComputationInfo(const bool& showComputationInfoInit) { 
    PyDeprecated("visualizationSettings", "general.showComputationInfo", "VisualizationSettings parameter general.showComputationInfo is deprecated! use view0.window.showComputationInfo instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.showComputationInfo is deprecated and forwards to view0.window.showComputationInfo, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.showComputationInfo= (const bool&)showComputationInfoInit; 
    }
inline bool VSettingsGeneral::PyGetShowComputationInfo() const { 
    PyDeprecated("visualizationSettings", "general.showComputationInfo", "VisualizationSettings parameter general.showComputationInfo is deprecated! use view0.window.showComputationInfo instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.showComputationInfo is deprecated and forwards to view0.window.showComputationInfo, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.window.showComputationInfo); 
    }

inline void VSettingsGeneral::PySetTextSize(const float& globalFontSizeInit) { 
    PyDeprecated("visualizationSettings", "general.textSize", "VisualizationSettings parameter general.textSize is deprecated! use view0.window.globalFontSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.textSize is deprecated and forwards to view0.window.globalFontSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.globalFontSize= (const float&)globalFontSizeInit; 
    }
inline float VSettingsGeneral::PyGetTextSize() const { 
    PyDeprecated("visualizationSettings", "general.textSize", "VisualizationSettings parameter general.textSize is deprecated! use view0.window.globalFontSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.textSize is deprecated and forwards to view0.window.globalFontSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->view0.window.globalFontSize); 
    }

inline void VSettingsGeneral::PySetWorldBasisSize(const float& worldBasisSizeInit) { 
    PyDeprecated("visualizationSettings", "general.worldBasisSize", "VisualizationSettings parameter general.worldBasisSize is deprecated! use view0.scene.worldBasisSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.worldBasisSize is deprecated and forwards to view0.scene.worldBasisSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.worldBasisSize= (const float&)worldBasisSizeInit; 
    }
inline float VSettingsGeneral::PyGetWorldBasisSize() const { 
    PyDeprecated("visualizationSettings", "general.worldBasisSize", "VisualizationSettings parameter general.worldBasisSize is deprecated! use view0.scene.worldBasisSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("general.worldBasisSize is deprecated and forwards to view0.scene.worldBasisSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->view0.scene.worldBasisSize); 
    }

inline void VSettingsContour::PySetColorBarPrecision(const Index& colorBarPrecisionInit) { 
    PyDeprecated("visualizationSettings", "contour.colorBarPrecision", "VisualizationSettings parameter contour.colorBarPrecision is deprecated! use contour.advanced.colorBarPrecision instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("contour.colorBarPrecision is deprecated and forwards to contour.advanced.colorBarPrecision, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->contour.advanced.colorBarPrecision= (const Index&)colorBarPrecisionInit; 
    }
inline Index VSettingsContour::PyGetColorBarPrecision() const { 
    PyDeprecated("visualizationSettings", "contour.colorBarPrecision", "VisualizationSettings parameter contour.colorBarPrecision is deprecated! use contour.advanced.colorBarPrecision instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("contour.colorBarPrecision is deprecated and forwards to contour.advanced.colorBarPrecision, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->contour.advanced.colorBarPrecision); 
    }

inline void VSettingsContour::PySetColorBarTiling(const Index& colorBarTilingInit) { 
    PyDeprecated("visualizationSettings", "contour.colorBarTiling", "VisualizationSettings parameter contour.colorBarTiling is deprecated! use contour.advanced.colorBarTiling instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("contour.colorBarTiling is deprecated and forwards to contour.advanced.colorBarTiling, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->contour.advanced.colorBarTiling= (const Index&)colorBarTilingInit; 
    }
inline Index VSettingsContour::PyGetColorBarTiling() const { 
    PyDeprecated("visualizationSettings", "contour.colorBarTiling", "VisualizationSettings parameter contour.colorBarTiling is deprecated! use contour.advanced.colorBarTiling instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("contour.colorBarTiling is deprecated and forwards to contour.advanced.colorBarTiling, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->contour.advanced.colorBarTiling); 
    }

inline void VSettingsContour::PySetShowColorBar(const bool& showColorBarInit) { 
    PyDeprecated("visualizationSettings", "contour.showColorBar", "VisualizationSettings parameter contour.showColorBar is deprecated! use contour.advanced.showColorBar instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("contour.showColorBar is deprecated and forwards to contour.advanced.showColorBar, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->contour.advanced.showColorBar= (const bool&)showColorBarInit; 
    }
inline bool VSettingsContour::PyGetShowColorBar() const { 
    PyDeprecated("visualizationSettings", "contour.showColorBar", "VisualizationSettings parameter contour.showColorBar is deprecated! use contour.advanced.showColorBar instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("contour.showColorBar is deprecated and forwards to contour.advanced.showColorBar, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->contour.advanced.showColorBar); 
    }

inline void VSettingsWindowDeprecated::PySetAlwaysOnTop(const bool& alwaysOnTopInit) { 
    PyDeprecated("visualizationSettings", "window.alwaysOnTop", "VisualizationSettings parameter window.alwaysOnTop is deprecated! use view0.window.alwaysOnTop instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.alwaysOnTop is deprecated and forwards to view0.window.alwaysOnTop, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.alwaysOnTop= (const bool&)alwaysOnTopInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetAlwaysOnTop() const { 
    PyDeprecated("visualizationSettings", "window.alwaysOnTop", "VisualizationSettings parameter window.alwaysOnTop is deprecated! use view0.window.alwaysOnTop instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.alwaysOnTop is deprecated and forwards to view0.window.alwaysOnTop, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.window.alwaysOnTop); 
    }

inline void VSettingsWindowDeprecated::PySetIgnoreKeys(const bool& ignoreKeysInit) { 
    PyDeprecated("visualizationSettings", "window.ignoreKeys", "VisualizationSettings parameter window.ignoreKeys is deprecated! use interactive.ignoreKeys instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.ignoreKeys is deprecated and forwards to interactive.ignoreKeys, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.ignoreKeys= (const bool&)ignoreKeysInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetIgnoreKeys() const { 
    PyDeprecated("visualizationSettings", "window.ignoreKeys", "VisualizationSettings parameter window.ignoreKeys is deprecated! use interactive.ignoreKeys instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.ignoreKeys is deprecated and forwards to interactive.ignoreKeys, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->interactive.ignoreKeys); 
    }

inline void VSettingsWindowDeprecated::PySetKeyPressUserFunction(const std::function<bool(int, int, int)>& keyPressUserFunctionInit) { 
    PyDeprecated("visualizationSettings", "window.keyPressUserFunction", "VisualizationSettings parameter window.keyPressUserFunction is deprecated! use interactive.keyPressUserFunction instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.keyPressUserFunction is deprecated and forwards to interactive.keyPressUserFunction, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.keyPressUserFunction= (const std::function<bool(int, int, int)>&)keyPressUserFunctionInit; 
    }
inline std::function<bool(int, int, int)> VSettingsWindowDeprecated::PyGetKeyPressUserFunction() const { 
    PyDeprecated("visualizationSettings", "window.keyPressUserFunction", "VisualizationSettings parameter window.keyPressUserFunction is deprecated! use interactive.keyPressUserFunction instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.keyPressUserFunction is deprecated and forwards to interactive.keyPressUserFunction, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::function<bool(int, int, int)>(backlink->interactive.keyPressUserFunction); 
    }

inline void VSettingsWindowDeprecated::PySetLimitWindowToScreenSize(const bool& limitWindowToScreenSizeInit) { 
    PyDeprecated("visualizationSettings", "window.limitWindowToScreenSize", "VisualizationSettings parameter window.limitWindowToScreenSize is deprecated! use general.limitWindowToScreenSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.limitWindowToScreenSize is deprecated and forwards to general.limitWindowToScreenSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->general.limitWindowToScreenSize= (const bool&)limitWindowToScreenSizeInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetLimitWindowToScreenSize() const { 
    PyDeprecated("visualizationSettings", "window.limitWindowToScreenSize", "VisualizationSettings parameter window.limitWindowToScreenSize is deprecated! use general.limitWindowToScreenSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.limitWindowToScreenSize is deprecated and forwards to general.limitWindowToScreenSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->general.limitWindowToScreenSize); 
    }

inline void VSettingsWindowDeprecated::PySetMaximize(const bool& maximizeInit) { 
    PyDeprecated("visualizationSettings", "window.maximize", "VisualizationSettings parameter window.maximize is deprecated! use view0.window.maximize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.maximize is deprecated and forwards to view0.window.maximize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.maximize= (const bool&)maximizeInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetMaximize() const { 
    PyDeprecated("visualizationSettings", "window.maximize", "VisualizationSettings parameter window.maximize is deprecated! use view0.window.maximize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.maximize is deprecated and forwards to view0.window.maximize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.window.maximize); 
    }

inline void VSettingsWindowDeprecated::PySetReallyQuitTimeLimit(const Real& reallyQuitTimeLimitInit) { 
    PyDeprecated("visualizationSettings", "window.reallyQuitTimeLimit", "VisualizationSettings parameter window.reallyQuitTimeLimit is deprecated! use general.reallyQuitTimeLimit instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.reallyQuitTimeLimit is deprecated and forwards to general.reallyQuitTimeLimit, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->general.reallyQuitTimeLimit= (const Real&)reallyQuitTimeLimitInit; 
    }
inline Real VSettingsWindowDeprecated::PyGetReallyQuitTimeLimit() const { 
    PyDeprecated("visualizationSettings", "window.reallyQuitTimeLimit", "VisualizationSettings parameter window.reallyQuitTimeLimit is deprecated! use general.reallyQuitTimeLimit instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.reallyQuitTimeLimit is deprecated and forwards to general.reallyQuitTimeLimit, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Real(backlink->general.reallyQuitTimeLimit); 
    }

inline void VSettingsWindowDeprecated::PySetRenderWindowSize(const std::array<Index,2>& renderWindowSizeInit) { 
    PyDeprecated("visualizationSettings", "window.renderWindowSize", "VisualizationSettings parameter window.renderWindowSize is deprecated! use view0.window.renderWindowSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.renderWindowSize is deprecated and forwards to view0.window.renderWindowSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.renderWindowSize= (const Index2&)renderWindowSizeInit; 
    }
inline std::array<Index,2> VSettingsWindowDeprecated::PyGetRenderWindowSize() const { 
    PyDeprecated("visualizationSettings", "window.renderWindowSize", "VisualizationSettings parameter window.renderWindowSize is deprecated! use view0.window.renderWindowSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.renderWindowSize is deprecated and forwards to view0.window.renderWindowSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<Index,2>(backlink->view0.window.renderWindowSize); 
    }

inline void VSettingsWindowDeprecated::PySetShowMouseCoordinates(const bool& showMouseCoordinatesInit) { 
    PyDeprecated("visualizationSettings", "window.showMouseCoordinates", "VisualizationSettings parameter window.showMouseCoordinates is deprecated! use view0.window.showMouseCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.showMouseCoordinates is deprecated and forwards to view0.window.showMouseCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.showMouseCoordinates= (const bool&)showMouseCoordinatesInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetShowMouseCoordinates() const { 
    PyDeprecated("visualizationSettings", "window.showMouseCoordinates", "VisualizationSettings parameter window.showMouseCoordinates is deprecated! use view0.window.showMouseCoordinates instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.showMouseCoordinates is deprecated and forwards to view0.window.showMouseCoordinates, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.window.showMouseCoordinates); 
    }

inline void VSettingsWindowDeprecated::PySetShowRenderStateInfo(const bool& showRenderStateInfoInit) { 
    PyDeprecated("visualizationSettings", "window.showRenderStateInfo", "VisualizationSettings parameter window.showRenderStateInfo is deprecated! use view0.window.showRenderStateInfo instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.showRenderStateInfo is deprecated and forwards to view0.window.showRenderStateInfo, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.showRenderStateInfo= (const bool&)showRenderStateInfoInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetShowRenderStateInfo() const { 
    PyDeprecated("visualizationSettings", "window.showRenderStateInfo", "VisualizationSettings parameter window.showRenderStateInfo is deprecated! use view0.window.showRenderStateInfo instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.showRenderStateInfo is deprecated and forwards to view0.window.showRenderStateInfo, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.window.showRenderStateInfo); 
    }

inline void VSettingsWindowDeprecated::PySetShowWindow(const bool& showWindowInit) { 
    PyDeprecated("visualizationSettings", "window.showWindow", "VisualizationSettings parameter window.showWindow is deprecated! use view0.window.showWindow instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.showWindow is deprecated and forwards to view0.window.showWindow, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.showWindow= (const bool&)showWindowInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetShowWindow() const { 
    PyDeprecated("visualizationSettings", "window.showWindow", "VisualizationSettings parameter window.showWindow is deprecated! use view0.window.showWindow instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.showWindow is deprecated and forwards to view0.window.showWindow, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.window.showWindow); 
    }

inline void VSettingsWindowDeprecated::PySetStartupTimeout(const Index& rendererStartupTimeoutInit) { 
    PyDeprecated("visualizationSettings", "window.startupTimeout", "VisualizationSettings parameter window.startupTimeout is deprecated! use general.rendererStartupTimeout instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.startupTimeout is deprecated and forwards to general.rendererStartupTimeout, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->general.rendererStartupTimeout= (const Index&)rendererStartupTimeoutInit; 
    }
inline Index VSettingsWindowDeprecated::PyGetStartupTimeout() const { 
    PyDeprecated("visualizationSettings", "window.startupTimeout", "VisualizationSettings parameter window.startupTimeout is deprecated! use general.rendererStartupTimeout instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("window.startupTimeout is deprecated and forwards to general.rendererStartupTimeout, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->general.rendererStartupTimeout); 
    }

inline void VSettingsDialogs::PySetFontScalingMacOS(const float& fontScalingInit) { 
    PyDeprecated("visualizationSettings", "dialogs.fontScalingMacOS", "VisualizationSettings parameter dialogs.fontScalingMacOS is deprecated! use dialogs.fontScaling instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("dialogs.fontScalingMacOS is deprecated and forwards to dialogs.fontScaling, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->dialogs.fontScaling= (const float&)fontScalingInit; 
    }
inline float VSettingsDialogs::PyGetFontScalingMacOS() const { 
    PyDeprecated("visualizationSettings", "dialogs.fontScalingMacOS", "VisualizationSettings parameter dialogs.fontScalingMacOS is deprecated! use dialogs.fontScaling instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("dialogs.fontScalingMacOS is deprecated and forwards to dialogs.fontScaling, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->dialogs.fontScaling); 
    }

inline void VSettingsRaytracer::PySetAmbientLightColor(const std::array<float,4>& lightModelAmbientInit) { 
    PyDeprecated("visualizationSettings", "raytracer.ambientLightColor", "VisualizationSettings parameter raytracer.ambientLightColor is deprecated! use openGL.lightModelAmbient instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.ambientLightColor is deprecated and forwards to openGL.lightModelAmbient, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.lightModelAmbient= (const Float4&)lightModelAmbientInit; 
    }
inline std::array<float,4> VSettingsRaytracer::PyGetAmbientLightColor() const { 
    PyDeprecated("visualizationSettings", "raytracer.ambientLightColor", "VisualizationSettings parameter raytracer.ambientLightColor is deprecated! use openGL.lightModelAmbient instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.ambientLightColor is deprecated and forwards to openGL.lightModelAmbient, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->openGL.lightModelAmbient); 
    }

inline void VSettingsRaytracer::PySetBackgroundColorReflections(const std::array<float,4>& backgroundColorReflectionsInit) { 
    PyDeprecated("visualizationSettings", "raytracer.backgroundColorReflections", "VisualizationSettings parameter raytracer.backgroundColorReflections is deprecated! use raytracer.advanced.backgroundColorReflections instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.backgroundColorReflections is deprecated and forwards to raytracer.advanced.backgroundColorReflections, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->raytracer.advanced.backgroundColorReflections= (const Float4&)backgroundColorReflectionsInit; 
    }
inline std::array<float,4> VSettingsRaytracer::PyGetBackgroundColorReflections() const { 
    PyDeprecated("visualizationSettings", "raytracer.backgroundColorReflections", "VisualizationSettings parameter raytracer.backgroundColorReflections is deprecated! use raytracer.advanced.backgroundColorReflections instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.backgroundColorReflections is deprecated and forwards to raytracer.advanced.backgroundColorReflections, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->raytracer.advanced.backgroundColorReflections); 
    }

inline void VSettingsRaytracer::PySetEnable(const bool& useRaytracerInit) { 
    PyDeprecated("visualizationSettings", "raytracer.enable", "VisualizationSettings parameter raytracer.enable is deprecated! use view0.camera.useRaytracer instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.enable is deprecated and forwards to view0.camera.useRaytracer, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.useRaytracer= (const bool&)useRaytracerInit; 
    }
inline bool VSettingsRaytracer::PyGetEnable() const { 
    PyDeprecated("visualizationSettings", "raytracer.enable", "VisualizationSettings parameter raytracer.enable is deprecated! use view0.camera.useRaytracer instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.enable is deprecated and forwards to view0.camera.useRaytracer, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.camera.useRaytracer); 
    }

inline void VSettingsRaytracer::PySetLightRadius(const float& lightRadiusInit) { 
    PyDeprecated("visualizationSettings", "raytracer.lightRadius", "VisualizationSettings parameter raytracer.lightRadius is deprecated! use openGL.light0.lightRadius instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.lightRadius is deprecated and forwards to openGL.light0.lightRadius, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.lightRadius= (const float&)lightRadiusInit; 
    }
inline float VSettingsRaytracer::PyGetLightRadius() const { 
    PyDeprecated("visualizationSettings", "raytracer.lightRadius", "VisualizationSettings parameter raytracer.lightRadius is deprecated! use openGL.light0.lightRadius instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.lightRadius is deprecated and forwards to openGL.light0.lightRadius, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light0.lightRadius); 
    }

inline void VSettingsRaytracer::PySetSearchTreeFactor(const Index& searchTreeFactorInit) { 
    PyDeprecated("visualizationSettings", "raytracer.searchTreeFactor", "VisualizationSettings parameter raytracer.searchTreeFactor is deprecated! use raytracer.advanced.searchTreeFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.searchTreeFactor is deprecated and forwards to raytracer.advanced.searchTreeFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->raytracer.advanced.searchTreeFactor= (const Index&)searchTreeFactorInit; 
    }
inline Index VSettingsRaytracer::PyGetSearchTreeFactor() const { 
    PyDeprecated("visualizationSettings", "raytracer.searchTreeFactor", "VisualizationSettings parameter raytracer.searchTreeFactor is deprecated! use raytracer.advanced.searchTreeFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.searchTreeFactor is deprecated and forwards to raytracer.advanced.searchTreeFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->raytracer.advanced.searchTreeFactor); 
    }

inline void VSettingsRaytracer::PySetShadowScalingFactor(const Index& shadowScalingFactorInit) { 
    PyDeprecated("visualizationSettings", "raytracer.shadowScalingFactor", "VisualizationSettings parameter raytracer.shadowScalingFactor is deprecated! use raytracer.advanced.shadowScalingFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.shadowScalingFactor is deprecated and forwards to raytracer.advanced.shadowScalingFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->raytracer.advanced.shadowScalingFactor= (const Index&)shadowScalingFactorInit; 
    }
inline Index VSettingsRaytracer::PyGetShadowScalingFactor() const { 
    PyDeprecated("visualizationSettings", "raytracer.shadowScalingFactor", "VisualizationSettings parameter raytracer.shadowScalingFactor is deprecated! use raytracer.advanced.shadowScalingFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.shadowScalingFactor is deprecated and forwards to raytracer.advanced.shadowScalingFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->raytracer.advanced.shadowScalingFactor); 
    }

inline void VSettingsRaytracer::PySetShadowSmoothingSteps(const Index& shadowSmoothingStepsInit) { 
    PyDeprecated("visualizationSettings", "raytracer.shadowSmoothingSteps", "VisualizationSettings parameter raytracer.shadowSmoothingSteps is deprecated! use raytracer.advanced.shadowSmoothingSteps instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.shadowSmoothingSteps is deprecated and forwards to raytracer.advanced.shadowSmoothingSteps, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->raytracer.advanced.shadowSmoothingSteps= (const Index&)shadowSmoothingStepsInit; 
    }
inline Index VSettingsRaytracer::PyGetShadowSmoothingSteps() const { 
    PyDeprecated("visualizationSettings", "raytracer.shadowSmoothingSteps", "VisualizationSettings parameter raytracer.shadowSmoothingSteps is deprecated! use raytracer.advanced.shadowSmoothingSteps instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.shadowSmoothingSteps is deprecated and forwards to raytracer.advanced.shadowSmoothingSteps, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->raytracer.advanced.shadowSmoothingSteps); 
    }

inline void VSettingsRaytracer::PySetShowText(const bool& showTextInit) { 
    PyDeprecated("visualizationSettings", "raytracer.showText", "VisualizationSettings parameter raytracer.showText is deprecated! use raytracer.advanced.showText instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.showText is deprecated and forwards to raytracer.advanced.showText, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->raytracer.advanced.showText= (const bool&)showTextInit; 
    }
inline bool VSettingsRaytracer::PyGetShowText() const { 
    PyDeprecated("visualizationSettings", "raytracer.showText", "VisualizationSettings parameter raytracer.showText is deprecated! use raytracer.advanced.showText instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.showText is deprecated and forwards to raytracer.advanced.showText, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->raytracer.advanced.showText); 
    }

inline void VSettingsRaytracer::PySetTilesPerThread(const Index& tilesPerThreadInit) { 
    PyDeprecated("visualizationSettings", "raytracer.tilesPerThread", "VisualizationSettings parameter raytracer.tilesPerThread is deprecated! use raytracer.advanced.tilesPerThread instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.tilesPerThread is deprecated and forwards to raytracer.advanced.tilesPerThread, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->raytracer.advanced.tilesPerThread= (const Index&)tilesPerThreadInit; 
    }
inline Index VSettingsRaytracer::PyGetTilesPerThread() const { 
    PyDeprecated("visualizationSettings", "raytracer.tilesPerThread", "VisualizationSettings parameter raytracer.tilesPerThread is deprecated! use raytracer.advanced.tilesPerThread instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.tilesPerThread is deprecated and forwards to raytracer.advanced.tilesPerThread, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->raytracer.advanced.tilesPerThread); 
    }

inline void VSettingsRaytracer::PySetZBiasLines(const float& zBiasLinesInit) { 
    PyDeprecated("visualizationSettings", "raytracer.zBiasLines", "VisualizationSettings parameter raytracer.zBiasLines is deprecated! use raytracer.advanced.zBiasLines instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.zBiasLines is deprecated and forwards to raytracer.advanced.zBiasLines, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->raytracer.advanced.zBiasLines= (const float&)zBiasLinesInit; 
    }
inline float VSettingsRaytracer::PyGetZBiasLines() const { 
    PyDeprecated("visualizationSettings", "raytracer.zBiasLines", "VisualizationSettings parameter raytracer.zBiasLines is deprecated! use raytracer.advanced.zBiasLines instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.zBiasLines is deprecated and forwards to raytracer.advanced.zBiasLines, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->raytracer.advanced.zBiasLines); 
    }

inline void VSettingsRaytracer::PySetZOffsetCamera(const float& dummyInit) { 
    PyDeprecated("visualizationSettings", "raytracer.zOffsetCamera", "VisualizationSettings parameter raytracer.zOffsetCamera is deprecated! use openGL.dummy instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.zOffsetCamera is deprecated and forwards to openGL.dummy, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.dummy= (const float&)dummyInit; 
    }
inline float VSettingsRaytracer::PyGetZOffsetCamera() const { 
    PyDeprecated("visualizationSettings", "raytracer.zOffsetCamera", "VisualizationSettings parameter raytracer.zOffsetCamera is deprecated! use openGL.dummy instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("raytracer.zOffsetCamera is deprecated and forwards to openGL.dummy, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.dummy); 
    }

inline void VSettingsOpenGL::PySetClippingPlaneColor(const std::array<float,4>& clippingPlaneColorInit) { 
    PyDeprecated("visualizationSettings", "openGL.clippingPlaneColor", "VisualizationSettings parameter openGL.clippingPlaneColor is deprecated! use openGL.advanced.clippingPlaneColor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.clippingPlaneColor is deprecated and forwards to openGL.advanced.clippingPlaneColor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.clippingPlaneColor= (const Float4&)clippingPlaneColorInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetClippingPlaneColor() const { 
    PyDeprecated("visualizationSettings", "openGL.clippingPlaneColor", "VisualizationSettings parameter openGL.clippingPlaneColor is deprecated! use openGL.advanced.clippingPlaneColor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.clippingPlaneColor is deprecated and forwards to openGL.advanced.clippingPlaneColor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->openGL.advanced.clippingPlaneColor); 
    }

inline void VSettingsOpenGL::PySetClippingPlaneDistance(const float& clippingPlaneDistanceInit) { 
    PyDeprecated("visualizationSettings", "openGL.clippingPlaneDistance", "VisualizationSettings parameter openGL.clippingPlaneDistance is deprecated! use view0.camera.clippingPlaneDistance instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.clippingPlaneDistance is deprecated and forwards to view0.camera.clippingPlaneDistance, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.clippingPlaneDistance= (const float&)clippingPlaneDistanceInit; 
    }
inline float VSettingsOpenGL::PyGetClippingPlaneDistance() const { 
    PyDeprecated("visualizationSettings", "openGL.clippingPlaneDistance", "VisualizationSettings parameter openGL.clippingPlaneDistance is deprecated! use view0.camera.clippingPlaneDistance instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.clippingPlaneDistance is deprecated and forwards to view0.camera.clippingPlaneDistance, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->view0.camera.clippingPlaneDistance); 
    }

inline void VSettingsOpenGL::PySetClippingPlaneNormal(const std::array<float,3>& clippingPlaneNormalInit) { 
    PyDeprecated("visualizationSettings", "openGL.clippingPlaneNormal", "VisualizationSettings parameter openGL.clippingPlaneNormal is deprecated! use view0.camera.clippingPlaneNormal instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.clippingPlaneNormal is deprecated and forwards to view0.camera.clippingPlaneNormal, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.clippingPlaneNormal= (const Float3&)clippingPlaneNormalInit; 
    }
inline std::array<float,3> VSettingsOpenGL::PyGetClippingPlaneNormal() const { 
    PyDeprecated("visualizationSettings", "openGL.clippingPlaneNormal", "VisualizationSettings parameter openGL.clippingPlaneNormal is deprecated! use view0.camera.clippingPlaneNormal instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.clippingPlaneNormal is deprecated and forwards to view0.camera.clippingPlaneNormal, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,3>(backlink->view0.camera.clippingPlaneNormal); 
    }

inline void VSettingsOpenGL::PySetDepthSorting(const bool& depthSortingInit) { 
    PyDeprecated("visualizationSettings", "openGL.depthSorting", "VisualizationSettings parameter openGL.depthSorting is deprecated! use openGL.advanced.depthSorting instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.depthSorting is deprecated and forwards to openGL.advanced.depthSorting, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.depthSorting= (const bool&)depthSortingInit; 
    }
inline bool VSettingsOpenGL::PyGetDepthSorting() const { 
    PyDeprecated("visualizationSettings", "openGL.depthSorting", "VisualizationSettings parameter openGL.depthSorting is deprecated! use openGL.advanced.depthSorting instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.depthSorting is deprecated and forwards to openGL.advanced.depthSorting, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.advanced.depthSorting); 
    }

inline void VSettingsOpenGL::PySetEnableLight0(const bool& enableInit) { 
    PyDeprecated("visualizationSettings", "openGL.enableLight0", "VisualizationSettings parameter openGL.enableLight0 is deprecated! use openGL.light0.enable instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.enableLight0 is deprecated and forwards to openGL.light0.enable, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.enable= (const bool&)enableInit; 
    }
inline bool VSettingsOpenGL::PyGetEnableLight0() const { 
    PyDeprecated("visualizationSettings", "openGL.enableLight0", "VisualizationSettings parameter openGL.enableLight0 is deprecated! use openGL.light0.enable instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.enableLight0 is deprecated and forwards to openGL.light0.enable, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.light0.enable); 
    }

inline void VSettingsOpenGL::PySetEnableLight1(const bool& enableInit) { 
    PyDeprecated("visualizationSettings", "openGL.enableLight1", "VisualizationSettings parameter openGL.enableLight1 is deprecated! use openGL.light1.enable instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.enableLight1 is deprecated and forwards to openGL.light1.enable, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light1.enable= (const bool&)enableInit; 
    }
inline bool VSettingsOpenGL::PyGetEnableLight1() const { 
    PyDeprecated("visualizationSettings", "openGL.enableLight1", "VisualizationSettings parameter openGL.enableLight1 is deprecated! use openGL.light1.enable instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.enableLight1 is deprecated and forwards to openGL.light1.enable, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.light1.enable); 
    }

inline void VSettingsOpenGL::PySetEnableLighting(const bool& enableLightingInit) { 
    PyDeprecated("visualizationSettings", "openGL.enableLighting", "VisualizationSettings parameter openGL.enableLighting is deprecated! use openGL.advanced.enableLighting instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.enableLighting is deprecated and forwards to openGL.advanced.enableLighting, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.enableLighting= (const bool&)enableLightingInit; 
    }
inline bool VSettingsOpenGL::PyGetEnableLighting() const { 
    PyDeprecated("visualizationSettings", "openGL.enableLighting", "VisualizationSettings parameter openGL.enableLighting is deprecated! use openGL.advanced.enableLighting instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.enableLighting is deprecated and forwards to openGL.advanced.enableLighting, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.advanced.enableLighting); 
    }

inline void VSettingsOpenGL::PySetFacesTransparent(const bool& facesTransparentInit) { 
    PyDeprecated("visualizationSettings", "openGL.facesTransparent", "VisualizationSettings parameter openGL.facesTransparent is deprecated! use view0.scene.facesTransparent instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.facesTransparent is deprecated and forwards to view0.scene.facesTransparent, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.facesTransparent= (const bool&)facesTransparentInit; 
    }
inline bool VSettingsOpenGL::PyGetFacesTransparent() const { 
    PyDeprecated("visualizationSettings", "openGL.facesTransparent", "VisualizationSettings parameter openGL.facesTransparent is deprecated! use view0.scene.facesTransparent instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.facesTransparent is deprecated and forwards to view0.scene.facesTransparent, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.scene.facesTransparent); 
    }

inline void VSettingsOpenGL::PySetInitialCenterPoint(const std::array<float,3>& initialCenterPointInit) { 
    PyDeprecated("visualizationSettings", "openGL.initialCenterPoint", "VisualizationSettings parameter openGL.initialCenterPoint is deprecated! use openGL.advanced.initialCenterPoint instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialCenterPoint is deprecated and forwards to openGL.advanced.initialCenterPoint, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.initialCenterPoint= (const Float3&)initialCenterPointInit; 
    }
inline std::array<float,3> VSettingsOpenGL::PyGetInitialCenterPoint() const { 
    PyDeprecated("visualizationSettings", "openGL.initialCenterPoint", "VisualizationSettings parameter openGL.initialCenterPoint is deprecated! use openGL.advanced.initialCenterPoint instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialCenterPoint is deprecated and forwards to openGL.advanced.initialCenterPoint, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,3>(backlink->openGL.advanced.initialCenterPoint); 
    }

inline void VSettingsOpenGL::PySetInitialMaxSceneSize(const float& initialMaxSceneSizeInit) { 
    PyDeprecated("visualizationSettings", "openGL.initialMaxSceneSize", "VisualizationSettings parameter openGL.initialMaxSceneSize is deprecated! use openGL.advanced.initialMaxSceneSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialMaxSceneSize is deprecated and forwards to openGL.advanced.initialMaxSceneSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.initialMaxSceneSize= (const float&)initialMaxSceneSizeInit; 
    }
inline float VSettingsOpenGL::PyGetInitialMaxSceneSize() const { 
    PyDeprecated("visualizationSettings", "openGL.initialMaxSceneSize", "VisualizationSettings parameter openGL.initialMaxSceneSize is deprecated! use openGL.advanced.initialMaxSceneSize instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialMaxSceneSize is deprecated and forwards to openGL.advanced.initialMaxSceneSize, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.advanced.initialMaxSceneSize); 
    }

inline void VSettingsOpenGL::PySetInitialModelRotation(const StdArray33F& initialModelRotationInit) { 
    PyDeprecated("visualizationSettings", "openGL.initialModelRotation", "VisualizationSettings parameter openGL.initialModelRotation is deprecated! use openGL.advanced.initialModelRotation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialModelRotation is deprecated and forwards to openGL.advanced.initialModelRotation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.initialModelRotation= (const StdArray33F&)initialModelRotationInit; 
    }
inline StdArray33F VSettingsOpenGL::PyGetInitialModelRotation() const { 
    PyDeprecated("visualizationSettings", "openGL.initialModelRotation", "VisualizationSettings parameter openGL.initialModelRotation is deprecated! use openGL.advanced.initialModelRotation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialModelRotation is deprecated and forwards to openGL.advanced.initialModelRotation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return StdArray33F(backlink->openGL.advanced.initialModelRotation); 
    }

inline void VSettingsOpenGL::PySetInitialZoom(const float& initialZoomInit) { 
    PyDeprecated("visualizationSettings", "openGL.initialZoom", "VisualizationSettings parameter openGL.initialZoom is deprecated! use openGL.advanced.initialZoom instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialZoom is deprecated and forwards to openGL.advanced.initialZoom, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.initialZoom= (const float&)initialZoomInit; 
    }
inline float VSettingsOpenGL::PyGetInitialZoom() const { 
    PyDeprecated("visualizationSettings", "openGL.initialZoom", "VisualizationSettings parameter openGL.initialZoom is deprecated! use openGL.advanced.initialZoom instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.initialZoom is deprecated and forwards to openGL.advanced.initialZoom, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.advanced.initialZoom); 
    }

inline void VSettingsOpenGL::PySetLight0ambient(const float& dummyInit) { 
    PyDeprecated("visualizationSettings", "openGL.light0ambient", "VisualizationSettings parameter openGL.light0ambient is deprecated! use openGL.dummy instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0ambient is deprecated and forwards to openGL.dummy, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.dummy= (const float&)dummyInit; 
    }
inline float VSettingsOpenGL::PyGetLight0ambient() const { 
    PyDeprecated("visualizationSettings", "openGL.light0ambient", "VisualizationSettings parameter openGL.light0ambient is deprecated! use openGL.dummy instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0ambient is deprecated and forwards to openGL.dummy, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.dummy); 
    }

inline void VSettingsOpenGL::PySetLight0constantAttenuation(const float& constantAttenuationInit) { 
    PyDeprecated("visualizationSettings", "openGL.light0constantAttenuation", "VisualizationSettings parameter openGL.light0constantAttenuation is deprecated! use openGL.light0.constantAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0constantAttenuation is deprecated and forwards to openGL.light0.constantAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.constantAttenuation= (const float&)constantAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight0constantAttenuation() const { 
    PyDeprecated("visualizationSettings", "openGL.light0constantAttenuation", "VisualizationSettings parameter openGL.light0constantAttenuation is deprecated! use openGL.light0.constantAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0constantAttenuation is deprecated and forwards to openGL.light0.constantAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light0.constantAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight0diffuse(const float& diffuseInit) { 
    PyDeprecated("visualizationSettings", "openGL.light0diffuse", "VisualizationSettings parameter openGL.light0diffuse is deprecated! use openGL.light0.diffuse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0diffuse is deprecated and forwards to openGL.light0.diffuse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.diffuse= (const float&)diffuseInit; 
    }
inline float VSettingsOpenGL::PyGetLight0diffuse() const { 
    PyDeprecated("visualizationSettings", "openGL.light0diffuse", "VisualizationSettings parameter openGL.light0diffuse is deprecated! use openGL.light0.diffuse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0diffuse is deprecated and forwards to openGL.light0.diffuse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light0.diffuse); 
    }

inline void VSettingsOpenGL::PySetLight0linearAttenuation(const float& linearAttenuationInit) { 
    PyDeprecated("visualizationSettings", "openGL.light0linearAttenuation", "VisualizationSettings parameter openGL.light0linearAttenuation is deprecated! use openGL.light0.linearAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0linearAttenuation is deprecated and forwards to openGL.light0.linearAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.linearAttenuation= (const float&)linearAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight0linearAttenuation() const { 
    PyDeprecated("visualizationSettings", "openGL.light0linearAttenuation", "VisualizationSettings parameter openGL.light0linearAttenuation is deprecated! use openGL.light0.linearAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0linearAttenuation is deprecated and forwards to openGL.light0.linearAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light0.linearAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight0position(const std::array<float,4>& positionInit) { 
    PyDeprecated("visualizationSettings", "openGL.light0position", "VisualizationSettings parameter openGL.light0position is deprecated! use openGL.light0.position instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0position is deprecated and forwards to openGL.light0.position, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.position= (const Float4&)positionInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetLight0position() const { 
    PyDeprecated("visualizationSettings", "openGL.light0position", "VisualizationSettings parameter openGL.light0position is deprecated! use openGL.light0.position instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0position is deprecated and forwards to openGL.light0.position, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->openGL.light0.position); 
    }

inline void VSettingsOpenGL::PySetLight0quadraticAttenuation(const float& quadraticAttenuationInit) { 
    PyDeprecated("visualizationSettings", "openGL.light0quadraticAttenuation", "VisualizationSettings parameter openGL.light0quadraticAttenuation is deprecated! use openGL.light0.quadraticAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0quadraticAttenuation is deprecated and forwards to openGL.light0.quadraticAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.quadraticAttenuation= (const float&)quadraticAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight0quadraticAttenuation() const { 
    PyDeprecated("visualizationSettings", "openGL.light0quadraticAttenuation", "VisualizationSettings parameter openGL.light0quadraticAttenuation is deprecated! use openGL.light0.quadraticAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0quadraticAttenuation is deprecated and forwards to openGL.light0.quadraticAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light0.quadraticAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight0specular(const float& specularInit) { 
    PyDeprecated("visualizationSettings", "openGL.light0specular", "VisualizationSettings parameter openGL.light0specular is deprecated! use openGL.light0.specular instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0specular is deprecated and forwards to openGL.light0.specular, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.specular= (const float&)specularInit; 
    }
inline float VSettingsOpenGL::PyGetLight0specular() const { 
    PyDeprecated("visualizationSettings", "openGL.light0specular", "VisualizationSettings parameter openGL.light0specular is deprecated! use openGL.light0.specular instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light0specular is deprecated and forwards to openGL.light0.specular, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light0.specular); 
    }

inline void VSettingsOpenGL::PySetLight1ambient(const float& dummyInit) { 
    PyDeprecated("visualizationSettings", "openGL.light1ambient", "VisualizationSettings parameter openGL.light1ambient is deprecated! use openGL.dummy instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1ambient is deprecated and forwards to openGL.dummy, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.dummy= (const float&)dummyInit; 
    }
inline float VSettingsOpenGL::PyGetLight1ambient() const { 
    PyDeprecated("visualizationSettings", "openGL.light1ambient", "VisualizationSettings parameter openGL.light1ambient is deprecated! use openGL.dummy instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1ambient is deprecated and forwards to openGL.dummy, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.dummy); 
    }

inline void VSettingsOpenGL::PySetLight1constantAttenuation(const float& constantAttenuationInit) { 
    PyDeprecated("visualizationSettings", "openGL.light1constantAttenuation", "VisualizationSettings parameter openGL.light1constantAttenuation is deprecated! use openGL.light1.constantAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1constantAttenuation is deprecated and forwards to openGL.light1.constantAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light1.constantAttenuation= (const float&)constantAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight1constantAttenuation() const { 
    PyDeprecated("visualizationSettings", "openGL.light1constantAttenuation", "VisualizationSettings parameter openGL.light1constantAttenuation is deprecated! use openGL.light1.constantAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1constantAttenuation is deprecated and forwards to openGL.light1.constantAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light1.constantAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight1diffuse(const float& diffuseInit) { 
    PyDeprecated("visualizationSettings", "openGL.light1diffuse", "VisualizationSettings parameter openGL.light1diffuse is deprecated! use openGL.light1.diffuse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1diffuse is deprecated and forwards to openGL.light1.diffuse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light1.diffuse= (const float&)diffuseInit; 
    }
inline float VSettingsOpenGL::PyGetLight1diffuse() const { 
    PyDeprecated("visualizationSettings", "openGL.light1diffuse", "VisualizationSettings parameter openGL.light1diffuse is deprecated! use openGL.light1.diffuse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1diffuse is deprecated and forwards to openGL.light1.diffuse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light1.diffuse); 
    }

inline void VSettingsOpenGL::PySetLight1linearAttenuation(const float& linearAttenuationInit) { 
    PyDeprecated("visualizationSettings", "openGL.light1linearAttenuation", "VisualizationSettings parameter openGL.light1linearAttenuation is deprecated! use openGL.light1.linearAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1linearAttenuation is deprecated and forwards to openGL.light1.linearAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light1.linearAttenuation= (const float&)linearAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight1linearAttenuation() const { 
    PyDeprecated("visualizationSettings", "openGL.light1linearAttenuation", "VisualizationSettings parameter openGL.light1linearAttenuation is deprecated! use openGL.light1.linearAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1linearAttenuation is deprecated and forwards to openGL.light1.linearAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light1.linearAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight1position(const std::array<float,4>& positionInit) { 
    PyDeprecated("visualizationSettings", "openGL.light1position", "VisualizationSettings parameter openGL.light1position is deprecated! use openGL.light1.position instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1position is deprecated and forwards to openGL.light1.position, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light1.position= (const Float4&)positionInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetLight1position() const { 
    PyDeprecated("visualizationSettings", "openGL.light1position", "VisualizationSettings parameter openGL.light1position is deprecated! use openGL.light1.position instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1position is deprecated and forwards to openGL.light1.position, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->openGL.light1.position); 
    }

inline void VSettingsOpenGL::PySetLight1quadraticAttenuation(const float& quadraticAttenuationInit) { 
    PyDeprecated("visualizationSettings", "openGL.light1quadraticAttenuation", "VisualizationSettings parameter openGL.light1quadraticAttenuation is deprecated! use openGL.light1.quadraticAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1quadraticAttenuation is deprecated and forwards to openGL.light1.quadraticAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light1.quadraticAttenuation= (const float&)quadraticAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight1quadraticAttenuation() const { 
    PyDeprecated("visualizationSettings", "openGL.light1quadraticAttenuation", "VisualizationSettings parameter openGL.light1quadraticAttenuation is deprecated! use openGL.light1.quadraticAttenuation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1quadraticAttenuation is deprecated and forwards to openGL.light1.quadraticAttenuation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light1.quadraticAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight1specular(const float& specularInit) { 
    PyDeprecated("visualizationSettings", "openGL.light1specular", "VisualizationSettings parameter openGL.light1specular is deprecated! use openGL.light1.specular instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1specular is deprecated and forwards to openGL.light1.specular, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light1.specular= (const float&)specularInit; 
    }
inline float VSettingsOpenGL::PyGetLight1specular() const { 
    PyDeprecated("visualizationSettings", "openGL.light1specular", "VisualizationSettings parameter openGL.light1specular is deprecated! use openGL.light1.specular instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.light1specular is deprecated and forwards to openGL.light1.specular, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light1.specular); 
    }

inline void VSettingsOpenGL::PySetLightModelLocalViewer(const bool& lightModelLocalViewerInit) { 
    PyDeprecated("visualizationSettings", "openGL.lightModelLocalViewer", "VisualizationSettings parameter openGL.lightModelLocalViewer is deprecated! use openGL.advanced.lightModelLocalViewer instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lightModelLocalViewer is deprecated and forwards to openGL.advanced.lightModelLocalViewer, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.lightModelLocalViewer= (const bool&)lightModelLocalViewerInit; 
    }
inline bool VSettingsOpenGL::PyGetLightModelLocalViewer() const { 
    PyDeprecated("visualizationSettings", "openGL.lightModelLocalViewer", "VisualizationSettings parameter openGL.lightModelLocalViewer is deprecated! use openGL.advanced.lightModelLocalViewer instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lightModelLocalViewer is deprecated and forwards to openGL.advanced.lightModelLocalViewer, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.advanced.lightModelLocalViewer); 
    }

inline void VSettingsOpenGL::PySetLightModelTwoSide(const bool& lightModelTwoSideInit) { 
    PyDeprecated("visualizationSettings", "openGL.lightModelTwoSide", "VisualizationSettings parameter openGL.lightModelTwoSide is deprecated! use openGL.advanced.lightModelTwoSide instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lightModelTwoSide is deprecated and forwards to openGL.advanced.lightModelTwoSide, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.lightModelTwoSide= (const bool&)lightModelTwoSideInit; 
    }
inline bool VSettingsOpenGL::PyGetLightModelTwoSide() const { 
    PyDeprecated("visualizationSettings", "openGL.lightModelTwoSide", "VisualizationSettings parameter openGL.lightModelTwoSide is deprecated! use openGL.advanced.lightModelTwoSide instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lightModelTwoSide is deprecated and forwards to openGL.advanced.lightModelTwoSide, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.advanced.lightModelTwoSide); 
    }

inline void VSettingsOpenGL::PySetLightPositionsInCameraFrame(const bool& useCameraFrameInit) { 
    PyDeprecated("visualizationSettings", "openGL.lightPositionsInCameraFrame", "VisualizationSettings parameter openGL.lightPositionsInCameraFrame is deprecated! use openGL.light0.useCameraFrame instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lightPositionsInCameraFrame is deprecated and forwards to openGL.light0.useCameraFrame, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.useCameraFrame= (const bool&)useCameraFrameInit; 
    }
inline bool VSettingsOpenGL::PyGetLightPositionsInCameraFrame() const { 
    PyDeprecated("visualizationSettings", "openGL.lightPositionsInCameraFrame", "VisualizationSettings parameter openGL.lightPositionsInCameraFrame is deprecated! use openGL.light0.useCameraFrame instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lightPositionsInCameraFrame is deprecated and forwards to openGL.light0.useCameraFrame, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.light0.useCameraFrame); 
    }

inline void VSettingsOpenGL::PySetLineSmooth(const bool& lineSmoothInit) { 
    PyDeprecated("visualizationSettings", "openGL.lineSmooth", "VisualizationSettings parameter openGL.lineSmooth is deprecated! use openGL.advanced.lineSmooth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lineSmooth is deprecated and forwards to openGL.advanced.lineSmooth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.lineSmooth= (const bool&)lineSmoothInit; 
    }
inline bool VSettingsOpenGL::PyGetLineSmooth() const { 
    PyDeprecated("visualizationSettings", "openGL.lineSmooth", "VisualizationSettings parameter openGL.lineSmooth is deprecated! use openGL.advanced.lineSmooth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.lineSmooth is deprecated and forwards to openGL.advanced.lineSmooth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.advanced.lineSmooth); 
    }

inline void VSettingsOpenGL::PySetMaterialAmbientAndDiffuse(const std::array<float,4>& materialSpecularInit) { 
    PyDeprecated("visualizationSettings", "openGL.materialAmbientAndDiffuse", "VisualizationSettings parameter openGL.materialAmbientAndDiffuse is deprecated! use openGL.materialSpecular instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.materialAmbientAndDiffuse is deprecated and forwards to openGL.materialSpecular, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.materialSpecular= (const Float4&)materialSpecularInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetMaterialAmbientAndDiffuse() const { 
    PyDeprecated("visualizationSettings", "openGL.materialAmbientAndDiffuse", "VisualizationSettings parameter openGL.materialAmbientAndDiffuse is deprecated! use openGL.materialSpecular instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.materialAmbientAndDiffuse is deprecated and forwards to openGL.materialSpecular, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->openGL.materialSpecular); 
    }

inline void VSettingsOpenGL::PySetPerspective(const float& perspectiveInit) { 
    PyDeprecated("visualizationSettings", "openGL.perspective", "VisualizationSettings parameter openGL.perspective is deprecated! use view0.camera.perspective instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.perspective is deprecated and forwards to view0.camera.perspective, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.perspective= (const float&)perspectiveInit; 
    }
inline float VSettingsOpenGL::PyGetPerspective() const { 
    PyDeprecated("visualizationSettings", "openGL.perspective", "VisualizationSettings parameter openGL.perspective is deprecated! use view0.camera.perspective instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.perspective is deprecated and forwards to view0.camera.perspective, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->view0.camera.perspective); 
    }

inline void VSettingsOpenGL::PySetPolygonOffset(const float& polygonOffsetInit) { 
    PyDeprecated("visualizationSettings", "openGL.polygonOffset", "VisualizationSettings parameter openGL.polygonOffset is deprecated! use openGL.advanced.polygonOffset instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.polygonOffset is deprecated and forwards to openGL.advanced.polygonOffset, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.polygonOffset= (const float&)polygonOffsetInit; 
    }
inline float VSettingsOpenGL::PyGetPolygonOffset() const { 
    PyDeprecated("visualizationSettings", "openGL.polygonOffset", "VisualizationSettings parameter openGL.polygonOffset is deprecated! use openGL.advanced.polygonOffset instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.polygonOffset is deprecated and forwards to openGL.advanced.polygonOffset, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.advanced.polygonOffset); 
    }

inline void VSettingsOpenGL::PySetShadeModelSmooth(const bool& shadeModelSmoothInit) { 
    PyDeprecated("visualizationSettings", "openGL.shadeModelSmooth", "VisualizationSettings parameter openGL.shadeModelSmooth is deprecated! use openGL.advanced.shadeModelSmooth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.shadeModelSmooth is deprecated and forwards to openGL.advanced.shadeModelSmooth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.shadeModelSmooth= (const bool&)shadeModelSmoothInit; 
    }
inline bool VSettingsOpenGL::PyGetShadeModelSmooth() const { 
    PyDeprecated("visualizationSettings", "openGL.shadeModelSmooth", "VisualizationSettings parameter openGL.shadeModelSmooth is deprecated! use openGL.advanced.shadeModelSmooth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.shadeModelSmooth is deprecated and forwards to openGL.advanced.shadeModelSmooth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.advanced.shadeModelSmooth); 
    }

inline void VSettingsOpenGL::PySetShadow(const float& shadowInit) { 
    PyDeprecated("visualizationSettings", "openGL.shadow", "VisualizationSettings parameter openGL.shadow is deprecated! use openGL.light0.shadow instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.shadow is deprecated and forwards to openGL.light0.shadow, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.light0.shadow= (const float&)shadowInit; 
    }
inline float VSettingsOpenGL::PyGetShadow() const { 
    PyDeprecated("visualizationSettings", "openGL.shadow", "VisualizationSettings parameter openGL.shadow is deprecated! use openGL.light0.shadow instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.shadow is deprecated and forwards to openGL.light0.shadow, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.light0.shadow); 
    }

inline void VSettingsOpenGL::PySetShadowPolygonOffset(const float& shadowPolygonOffsetInit) { 
    PyDeprecated("visualizationSettings", "openGL.shadowPolygonOffset", "VisualizationSettings parameter openGL.shadowPolygonOffset is deprecated! use openGL.advanced.shadowPolygonOffset instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.shadowPolygonOffset is deprecated and forwards to openGL.advanced.shadowPolygonOffset, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.shadowPolygonOffset= (const float&)shadowPolygonOffsetInit; 
    }
inline float VSettingsOpenGL::PyGetShadowPolygonOffset() const { 
    PyDeprecated("visualizationSettings", "openGL.shadowPolygonOffset", "VisualizationSettings parameter openGL.shadowPolygonOffset is deprecated! use openGL.advanced.shadowPolygonOffset instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.shadowPolygonOffset is deprecated and forwards to openGL.advanced.shadowPolygonOffset, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.advanced.shadowPolygonOffset); 
    }

inline void VSettingsOpenGL::PySetShowFaceEdges(const bool& showFaceEdgesInit) { 
    PyDeprecated("visualizationSettings", "openGL.showFaceEdges", "VisualizationSettings parameter openGL.showFaceEdges is deprecated! use view0.scene.showFaceEdges instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showFaceEdges is deprecated and forwards to view0.scene.showFaceEdges, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.showFaceEdges= (const bool&)showFaceEdgesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowFaceEdges() const { 
    PyDeprecated("visualizationSettings", "openGL.showFaceEdges", "VisualizationSettings parameter openGL.showFaceEdges is deprecated! use view0.scene.showFaceEdges instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showFaceEdges is deprecated and forwards to view0.scene.showFaceEdges, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.scene.showFaceEdges); 
    }

inline void VSettingsOpenGL::PySetShowFaces(const bool& showFacesInit) { 
    PyDeprecated("visualizationSettings", "openGL.showFaces", "VisualizationSettings parameter openGL.showFaces is deprecated! use view0.scene.showFaces instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showFaces is deprecated and forwards to view0.scene.showFaces, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.showFaces= (const bool&)showFacesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowFaces() const { 
    PyDeprecated("visualizationSettings", "openGL.showFaces", "VisualizationSettings parameter openGL.showFaces is deprecated! use view0.scene.showFaces instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showFaces is deprecated and forwards to view0.scene.showFaces, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.scene.showFaces); 
    }

inline void VSettingsOpenGL::PySetShowLines(const bool& showLinesInit) { 
    PyDeprecated("visualizationSettings", "openGL.showLines", "VisualizationSettings parameter openGL.showLines is deprecated! use view0.scene.showLines instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showLines is deprecated and forwards to view0.scene.showLines, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.showLines= (const bool&)showLinesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowLines() const { 
    PyDeprecated("visualizationSettings", "openGL.showLines", "VisualizationSettings parameter openGL.showLines is deprecated! use view0.scene.showLines instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showLines is deprecated and forwards to view0.scene.showLines, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.scene.showLines); 
    }

inline void VSettingsOpenGL::PySetShowMeshEdges(const bool& showMeshEdgesInit) { 
    PyDeprecated("visualizationSettings", "openGL.showMeshEdges", "VisualizationSettings parameter openGL.showMeshEdges is deprecated! use view0.scene.showMeshEdges instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showMeshEdges is deprecated and forwards to view0.scene.showMeshEdges, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.showMeshEdges= (const bool&)showMeshEdgesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowMeshEdges() const { 
    PyDeprecated("visualizationSettings", "openGL.showMeshEdges", "VisualizationSettings parameter openGL.showMeshEdges is deprecated! use view0.scene.showMeshEdges instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showMeshEdges is deprecated and forwards to view0.scene.showMeshEdges, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.scene.showMeshEdges); 
    }

inline void VSettingsOpenGL::PySetShowMeshFaces(const bool& showMeshFacesInit) { 
    PyDeprecated("visualizationSettings", "openGL.showMeshFaces", "VisualizationSettings parameter openGL.showMeshFaces is deprecated! use view0.scene.showMeshFaces instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showMeshFaces is deprecated and forwards to view0.scene.showMeshFaces, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.scene.showMeshFaces= (const bool&)showMeshFacesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowMeshFaces() const { 
    PyDeprecated("visualizationSettings", "openGL.showMeshFaces", "VisualizationSettings parameter openGL.showMeshFaces is deprecated! use view0.scene.showMeshFaces instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.showMeshFaces is deprecated and forwards to view0.scene.showMeshFaces, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.scene.showMeshFaces); 
    }

inline void VSettingsOpenGL::PySetTextLineSmooth(const bool& textLineSmoothInit) { 
    PyDeprecated("visualizationSettings", "openGL.textLineSmooth", "VisualizationSettings parameter openGL.textLineSmooth is deprecated! use openGL.advanced.textLineSmooth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.textLineSmooth is deprecated and forwards to openGL.advanced.textLineSmooth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.textLineSmooth= (const bool&)textLineSmoothInit; 
    }
inline bool VSettingsOpenGL::PyGetTextLineSmooth() const { 
    PyDeprecated("visualizationSettings", "openGL.textLineSmooth", "VisualizationSettings parameter openGL.textLineSmooth is deprecated! use openGL.advanced.textLineSmooth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.textLineSmooth is deprecated and forwards to openGL.advanced.textLineSmooth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->openGL.advanced.textLineSmooth); 
    }

inline void VSettingsOpenGL::PySetTextLineWidth(const float& textLineWidthInit) { 
    PyDeprecated("visualizationSettings", "openGL.textLineWidth", "VisualizationSettings parameter openGL.textLineWidth is deprecated! use openGL.advanced.textLineWidth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.textLineWidth is deprecated and forwards to openGL.advanced.textLineWidth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->openGL.advanced.textLineWidth= (const float&)textLineWidthInit; 
    }
inline float VSettingsOpenGL::PyGetTextLineWidth() const { 
    PyDeprecated("visualizationSettings", "openGL.textLineWidth", "VisualizationSettings parameter openGL.textLineWidth is deprecated! use openGL.advanced.textLineWidth instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("openGL.textLineWidth is deprecated and forwards to openGL.advanced.textLineWidth, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->openGL.advanced.textLineWidth); 
    }

inline void VSettingsInteractive::PySetHighlightColor(const std::array<float,4>& highlightColorInit) { 
    PyDeprecated("visualizationSettings", "interactive.highlightColor", "VisualizationSettings parameter interactive.highlightColor is deprecated! use interactive.advanced.highlightColor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.highlightColor is deprecated and forwards to interactive.advanced.highlightColor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.highlightColor= (const Float4&)highlightColorInit; 
    }
inline std::array<float,4> VSettingsInteractive::PyGetHighlightColor() const { 
    PyDeprecated("visualizationSettings", "interactive.highlightColor", "VisualizationSettings parameter interactive.highlightColor is deprecated! use interactive.advanced.highlightColor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.highlightColor is deprecated and forwards to interactive.advanced.highlightColor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->interactive.advanced.highlightColor); 
    }

inline void VSettingsInteractive::PySetHighlightOtherColor(const std::array<float,4>& highlightOtherColorInit) { 
    PyDeprecated("visualizationSettings", "interactive.highlightOtherColor", "VisualizationSettings parameter interactive.highlightOtherColor is deprecated! use interactive.advanced.highlightOtherColor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.highlightOtherColor is deprecated and forwards to interactive.advanced.highlightOtherColor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.highlightOtherColor= (const Float4&)highlightOtherColorInit; 
    }
inline std::array<float,4> VSettingsInteractive::PyGetHighlightOtherColor() const { 
    PyDeprecated("visualizationSettings", "interactive.highlightOtherColor", "VisualizationSettings parameter interactive.highlightOtherColor is deprecated! use interactive.advanced.highlightOtherColor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.highlightOtherColor is deprecated and forwards to interactive.advanced.highlightOtherColor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,4>(backlink->interactive.advanced.highlightOtherColor); 
    }

inline void VSettingsInteractive::PySetJoystickScaleRotation(const float& joystickScaleRotationInit) { 
    PyDeprecated("visualizationSettings", "interactive.joystickScaleRotation", "VisualizationSettings parameter interactive.joystickScaleRotation is deprecated! use interactive.advanced.joystickScaleRotation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.joystickScaleRotation is deprecated and forwards to interactive.advanced.joystickScaleRotation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.joystickScaleRotation= (const float&)joystickScaleRotationInit; 
    }
inline float VSettingsInteractive::PyGetJoystickScaleRotation() const { 
    PyDeprecated("visualizationSettings", "interactive.joystickScaleRotation", "VisualizationSettings parameter interactive.joystickScaleRotation is deprecated! use interactive.advanced.joystickScaleRotation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.joystickScaleRotation is deprecated and forwards to interactive.advanced.joystickScaleRotation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->interactive.advanced.joystickScaleRotation); 
    }

inline void VSettingsInteractive::PySetJoystickScaleTranslation(const float& joystickScaleTranslationInit) { 
    PyDeprecated("visualizationSettings", "interactive.joystickScaleTranslation", "VisualizationSettings parameter interactive.joystickScaleTranslation is deprecated! use interactive.advanced.joystickScaleTranslation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.joystickScaleTranslation is deprecated and forwards to interactive.advanced.joystickScaleTranslation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.joystickScaleTranslation= (const float&)joystickScaleTranslationInit; 
    }
inline float VSettingsInteractive::PyGetJoystickScaleTranslation() const { 
    PyDeprecated("visualizationSettings", "interactive.joystickScaleTranslation", "VisualizationSettings parameter interactive.joystickScaleTranslation is deprecated! use interactive.advanced.joystickScaleTranslation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.joystickScaleTranslation is deprecated and forwards to interactive.advanced.joystickScaleTranslation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->interactive.advanced.joystickScaleTranslation); 
    }

inline void VSettingsInteractive::PySetKeypressRotationStep(const float& keypressRotationStepInit) { 
    PyDeprecated("visualizationSettings", "interactive.keypressRotationStep", "VisualizationSettings parameter interactive.keypressRotationStep is deprecated! use interactive.advanced.keypressRotationStep instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.keypressRotationStep is deprecated and forwards to interactive.advanced.keypressRotationStep, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.keypressRotationStep= (const float&)keypressRotationStepInit; 
    }
inline float VSettingsInteractive::PyGetKeypressRotationStep() const { 
    PyDeprecated("visualizationSettings", "interactive.keypressRotationStep", "VisualizationSettings parameter interactive.keypressRotationStep is deprecated! use interactive.advanced.keypressRotationStep instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.keypressRotationStep is deprecated and forwards to interactive.advanced.keypressRotationStep, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->interactive.advanced.keypressRotationStep); 
    }

inline void VSettingsInteractive::PySetKeypressTranslationStep(const float& keypressTranslationStepInit) { 
    PyDeprecated("visualizationSettings", "interactive.keypressTranslationStep", "VisualizationSettings parameter interactive.keypressTranslationStep is deprecated! use interactive.advanced.keypressTranslationStep instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.keypressTranslationStep is deprecated and forwards to interactive.advanced.keypressTranslationStep, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.keypressTranslationStep= (const float&)keypressTranslationStepInit; 
    }
inline float VSettingsInteractive::PyGetKeypressTranslationStep() const { 
    PyDeprecated("visualizationSettings", "interactive.keypressTranslationStep", "VisualizationSettings parameter interactive.keypressTranslationStep is deprecated! use interactive.advanced.keypressTranslationStep instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.keypressTranslationStep is deprecated and forwards to interactive.advanced.keypressTranslationStep, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->interactive.advanced.keypressTranslationStep); 
    }

inline void VSettingsInteractive::PySetLockModelView(const bool& lockModelViewInit) { 
    PyDeprecated("visualizationSettings", "interactive.lockModelView", "VisualizationSettings parameter interactive.lockModelView is deprecated! use view0.window.lockModelView instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.lockModelView is deprecated and forwards to view0.window.lockModelView, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.window.lockModelView= (const bool&)lockModelViewInit; 
    }
inline bool VSettingsInteractive::PyGetLockModelView() const { 
    PyDeprecated("visualizationSettings", "interactive.lockModelView", "VisualizationSettings parameter interactive.lockModelView is deprecated! use view0.window.lockModelView instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.lockModelView is deprecated and forwards to view0.window.lockModelView, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->view0.window.lockModelView); 
    }

inline void VSettingsInteractive::PySetMouseMoveRotationFactor(const float& mouseMoveRotationFactorInit) { 
    PyDeprecated("visualizationSettings", "interactive.mouseMoveRotationFactor", "VisualizationSettings parameter interactive.mouseMoveRotationFactor is deprecated! use interactive.advanced.mouseMoveRotationFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.mouseMoveRotationFactor is deprecated and forwards to interactive.advanced.mouseMoveRotationFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.mouseMoveRotationFactor= (const float&)mouseMoveRotationFactorInit; 
    }
inline float VSettingsInteractive::PyGetMouseMoveRotationFactor() const { 
    PyDeprecated("visualizationSettings", "interactive.mouseMoveRotationFactor", "VisualizationSettings parameter interactive.mouseMoveRotationFactor is deprecated! use interactive.advanced.mouseMoveRotationFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.mouseMoveRotationFactor is deprecated and forwards to interactive.advanced.mouseMoveRotationFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->interactive.advanced.mouseMoveRotationFactor); 
    }

inline void VSettingsInteractive::PySetPauseWithSpacebar(const bool& pauseWithSpacebarInit) { 
    PyDeprecated("visualizationSettings", "interactive.pauseWithSpacebar", "VisualizationSettings parameter interactive.pauseWithSpacebar is deprecated! use interactive.advanced.pauseWithSpacebar instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.pauseWithSpacebar is deprecated and forwards to interactive.advanced.pauseWithSpacebar, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.pauseWithSpacebar= (const bool&)pauseWithSpacebarInit; 
    }
inline bool VSettingsInteractive::PyGetPauseWithSpacebar() const { 
    PyDeprecated("visualizationSettings", "interactive.pauseWithSpacebar", "VisualizationSettings parameter interactive.pauseWithSpacebar is deprecated! use interactive.advanced.pauseWithSpacebar instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.pauseWithSpacebar is deprecated and forwards to interactive.advanced.pauseWithSpacebar, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->interactive.advanced.pauseWithSpacebar); 
    }

inline void VSettingsInteractive::PySetSelectionHighlights(const bool& selectionHighlightsInit) { 
    PyDeprecated("visualizationSettings", "interactive.selectionHighlights", "VisualizationSettings parameter interactive.selectionHighlights is deprecated! use interactive.advanced.selectionHighlights instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionHighlights is deprecated and forwards to interactive.advanced.selectionHighlights, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.selectionHighlights= (const bool&)selectionHighlightsInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionHighlights() const { 
    PyDeprecated("visualizationSettings", "interactive.selectionHighlights", "VisualizationSettings parameter interactive.selectionHighlights is deprecated! use interactive.advanced.selectionHighlights instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionHighlights is deprecated and forwards to interactive.advanced.selectionHighlights, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->interactive.advanced.selectionHighlights); 
    }

inline void VSettingsInteractive::PySetSelectionLeftMouse(const bool& selectionLeftMouseInit) { 
    PyDeprecated("visualizationSettings", "interactive.selectionLeftMouse", "VisualizationSettings parameter interactive.selectionLeftMouse is deprecated! use interactive.advanced.selectionLeftMouse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionLeftMouse is deprecated and forwards to interactive.advanced.selectionLeftMouse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.selectionLeftMouse= (const bool&)selectionLeftMouseInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionLeftMouse() const { 
    PyDeprecated("visualizationSettings", "interactive.selectionLeftMouse", "VisualizationSettings parameter interactive.selectionLeftMouse is deprecated! use interactive.advanced.selectionLeftMouse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionLeftMouse is deprecated and forwards to interactive.advanced.selectionLeftMouse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->interactive.advanced.selectionLeftMouse); 
    }

inline void VSettingsInteractive::PySetSelectionLeftMouseItemTypes(const Index& selectionLeftMouseItemTypesInit) { 
    PyDeprecated("visualizationSettings", "interactive.selectionLeftMouseItemTypes", "VisualizationSettings parameter interactive.selectionLeftMouseItemTypes is deprecated! use interactive.advanced.selectionLeftMouseItemTypes instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionLeftMouseItemTypes is deprecated and forwards to interactive.advanced.selectionLeftMouseItemTypes, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.selectionLeftMouseItemTypes= (const Index&)selectionLeftMouseItemTypesInit; 
    }
inline Index VSettingsInteractive::PyGetSelectionLeftMouseItemTypes() const { 
    PyDeprecated("visualizationSettings", "interactive.selectionLeftMouseItemTypes", "VisualizationSettings parameter interactive.selectionLeftMouseItemTypes is deprecated! use interactive.advanced.selectionLeftMouseItemTypes instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionLeftMouseItemTypes is deprecated and forwards to interactive.advanced.selectionLeftMouseItemTypes, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->interactive.advanced.selectionLeftMouseItemTypes); 
    }

inline void VSettingsInteractive::PySetSelectionRightMouse(const bool& selectionRightMouseInit) { 
    PyDeprecated("visualizationSettings", "interactive.selectionRightMouse", "VisualizationSettings parameter interactive.selectionRightMouse is deprecated! use interactive.advanced.selectionRightMouse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionRightMouse is deprecated and forwards to interactive.advanced.selectionRightMouse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.selectionRightMouse= (const bool&)selectionRightMouseInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionRightMouse() const { 
    PyDeprecated("visualizationSettings", "interactive.selectionRightMouse", "VisualizationSettings parameter interactive.selectionRightMouse is deprecated! use interactive.advanced.selectionRightMouse instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionRightMouse is deprecated and forwards to interactive.advanced.selectionRightMouse, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->interactive.advanced.selectionRightMouse); 
    }

inline void VSettingsInteractive::PySetSelectionRightMouseGraphicsData(const bool& selectionRightMouseGraphicsDataInit) { 
    PyDeprecated("visualizationSettings", "interactive.selectionRightMouseGraphicsData", "VisualizationSettings parameter interactive.selectionRightMouseGraphicsData is deprecated! use interactive.advanced.selectionRightMouseGraphicsData instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionRightMouseGraphicsData is deprecated and forwards to interactive.advanced.selectionRightMouseGraphicsData, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.selectionRightMouseGraphicsData= (const bool&)selectionRightMouseGraphicsDataInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionRightMouseGraphicsData() const { 
    PyDeprecated("visualizationSettings", "interactive.selectionRightMouseGraphicsData", "VisualizationSettings parameter interactive.selectionRightMouseGraphicsData is deprecated! use interactive.advanced.selectionRightMouseGraphicsData instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.selectionRightMouseGraphicsData is deprecated and forwards to interactive.advanced.selectionRightMouseGraphicsData, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return bool(backlink->interactive.advanced.selectionRightMouseGraphicsData); 
    }

inline void VSettingsInteractive::PySetTrackMarker(const Index& trackMarkerInit) { 
    PyDeprecated("visualizationSettings", "interactive.trackMarker", "VisualizationSettings parameter interactive.trackMarker is deprecated! use view0.camera.trackMarker instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarker is deprecated and forwards to view0.camera.trackMarker, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.trackMarker= (const Index&)trackMarkerInit; 
    }
inline Index VSettingsInteractive::PyGetTrackMarker() const { 
    PyDeprecated("visualizationSettings", "interactive.trackMarker", "VisualizationSettings parameter interactive.trackMarker is deprecated! use view0.camera.trackMarker instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarker is deprecated and forwards to view0.camera.trackMarker, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->view0.camera.trackMarker); 
    }

inline void VSettingsInteractive::PySetTrackMarkerMbsNumber(const Index& trackMarkerMbsNumberInit) { 
    PyDeprecated("visualizationSettings", "interactive.trackMarkerMbsNumber", "VisualizationSettings parameter interactive.trackMarkerMbsNumber is deprecated! use view0.camera.trackMarkerMbsNumber instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarkerMbsNumber is deprecated and forwards to view0.camera.trackMarkerMbsNumber, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.trackMarkerMbsNumber= (const Index&)trackMarkerMbsNumberInit; 
    }
inline Index VSettingsInteractive::PyGetTrackMarkerMbsNumber() const { 
    PyDeprecated("visualizationSettings", "interactive.trackMarkerMbsNumber", "VisualizationSettings parameter interactive.trackMarkerMbsNumber is deprecated! use view0.camera.trackMarkerMbsNumber instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarkerMbsNumber is deprecated and forwards to view0.camera.trackMarkerMbsNumber, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return Index(backlink->view0.camera.trackMarkerMbsNumber); 
    }

inline void VSettingsInteractive::PySetTrackMarkerOrientation(const std::array<float,3>& trackMarkerOrientationInit) { 
    PyDeprecated("visualizationSettings", "interactive.trackMarkerOrientation", "VisualizationSettings parameter interactive.trackMarkerOrientation is deprecated! use view0.camera.trackMarkerOrientation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarkerOrientation is deprecated and forwards to view0.camera.trackMarkerOrientation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.trackMarkerOrientation= (const Float3&)trackMarkerOrientationInit; 
    }
inline std::array<float,3> VSettingsInteractive::PyGetTrackMarkerOrientation() const { 
    PyDeprecated("visualizationSettings", "interactive.trackMarkerOrientation", "VisualizationSettings parameter interactive.trackMarkerOrientation is deprecated! use view0.camera.trackMarkerOrientation instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarkerOrientation is deprecated and forwards to view0.camera.trackMarkerOrientation, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,3>(backlink->view0.camera.trackMarkerOrientation); 
    }

inline void VSettingsInteractive::PySetTrackMarkerPosition(const std::array<float,3>& trackMarkerPositionInit) { 
    PyDeprecated("visualizationSettings", "interactive.trackMarkerPosition", "VisualizationSettings parameter interactive.trackMarkerPosition is deprecated! use view0.camera.trackMarkerPosition instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarkerPosition is deprecated and forwards to view0.camera.trackMarkerPosition, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->view0.camera.trackMarkerPosition= (const Float3&)trackMarkerPositionInit; 
    }
inline std::array<float,3> VSettingsInteractive::PyGetTrackMarkerPosition() const { 
    PyDeprecated("visualizationSettings", "interactive.trackMarkerPosition", "VisualizationSettings parameter interactive.trackMarkerPosition is deprecated! use view0.camera.trackMarkerPosition instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.trackMarkerPosition is deprecated and forwards to view0.camera.trackMarkerPosition, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return std::array<float,3>(backlink->view0.camera.trackMarkerPosition); 
    }

inline void VSettingsInteractive::PySetZoomStepFactor(const float& zoomStepFactorInit) { 
    PyDeprecated("visualizationSettings", "interactive.zoomStepFactor", "VisualizationSettings parameter interactive.zoomStepFactor is deprecated! use interactive.advanced.zoomStepFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.zoomStepFactor is deprecated and forwards to interactive.advanced.zoomStepFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    backlink->interactive.advanced.zoomStepFactor= (const float&)zoomStepFactorInit; 
    }
inline float VSettingsInteractive::PyGetZoomStepFactor() const { 
    PyDeprecated("visualizationSettings", "interactive.zoomStepFactor", "VisualizationSettings parameter interactive.zoomStepFactor is deprecated! use interactive.advanced.zoomStepFactor instead!");
    if (backlink == nullptr) { CHECKandTHROWstring("interactive.zoomStepFactor is deprecated and forwards to interactive.advanced.zoomStepFactor, which needs the settings structure it belongs to; this one was constructed on its own and is not linked"); }
    return float(backlink->interactive.advanced.zoomStepFactor); 
    }

#endif //#ifdef include once...
