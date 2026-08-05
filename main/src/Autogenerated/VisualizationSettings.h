/** ***********************************************************************************************
* @class        VSettingsGeneral
* @brief        General settings for visualization that influence all windows, default values, autofit, multithreading, etc.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-04-03 (last modfied)
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
  Index axesTiling;                               //!< AUTO: global number of segments for drawing cylinders for axes and cones for arrows (reduce this number, e.g. to 4, if many axes are drawn)
  Float4 backgroundColor;                         //!< AUTO: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  Float4 backgroundColorBottom;                   //!< AUTO: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  float boundingBoxZoomAllFactor;                 //!< AUTO: factor on boundingBox for zoom all (without minimum offset)
  float boundingBoxZoomAllOffset;                 //!< AUTO: minimum offset to bounding box of scene in window - width or height, whatever is smaller; adjust for very small or large scenes; may be negative
  Index circleTiling;                             //!< AUTO: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  float coordinateSystemSize;                     //!< AUTO: size of coordinate system relative to font size
  Index cylinderTiling;                           //!< AUTO: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  float graphicsUpdateInterval;                   //!< AUTO: interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
  bool limitWindowToScreenSize;                   //!< AUTO: True: size for render window of respective view is limited to screen size; False: larger window sizes (e.g. for rendering) allowed according to renderWindowSize
  float linuxDisplayScaleFactor;                  //!< AUTO: Scaling factor for linux, which cannot determined from system by now; adjust this value to scale dialog fonts and renderer fonts
  float minSceneSize;                             //!< AUTO: minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
  float pointSize;                                //!< AUTO: global point size (absolute)
  Real reallyQuitTimeLimit;                       //!< AUTO: number of seconds after which user is asked a security question before stopping simulation and closing renderer; set to 0 in order to always get asked; set to 1e10 to (nearly) never get asked
  Index rendererPrecision;                        //!< AUTO: precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
  Index rendererStartupTimeout;                   //!< AUTO: OpenGL render windows startup timeout in ms (change might be necessary if CPU is very slow)
  std::string renderWindowString;                 //!< AUTO: string shown in render window (use this, e.g., for debugging, etc.; written below EXUDYN, similar to solutionInformation in SimulationSettings.solutionSettings)
  Index showHelpOnStartup;                        //!< AUTO: seconds to show help message on startup (0=deactivate)
  bool showSolutionInformation;                   //!< AUTO: true = show solution information (from simulationSettings.solution)
  bool showSolverInformation;                     //!< AUTO: true = solver name and further information shown in render window
  bool showSolverTime;                            //!< AUTO: true = solver current time shown in render window
  Index sphereTiling;                             //!< AUTO: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
  bool textAlwaysInFront;                         //!< AUTO: if true, text for item numbers and other item-related text is drawn in front; this may be unwanted in case that you only with to see numbers of objects in front; currently does not work with perspective
  Float4 textColor;                               //!< AUTO: general text color (default); used for system texts in render window
  bool textHasBackground;                         //!< AUTO: if true, text for item numbers and other item-related text have a background (depending on text color), allowing for better visibility if many numbers are shown; the text itself is black; therefore, dark background colors are ignored and shown as white
  float textOffsetFactor;                         //!< AUTO: This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes
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
  //! AUTO: Set function (needed in pybind) for: global number of segments for drawing cylinders for axes and cones for arrows (reduce this number, e.g. to 4, if many axes are drawn)
  void PySetAxesTiling(const Index& axesTilingInit) { axesTiling = EXUstd::GetSafelyPInt(axesTilingInit,"axesTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for drawing cylinders for axes and cones for arrows (reduce this number, e.g. to 4, if many axes are drawn)
  Index PyGetAxesTiling() const { return Index(axesTiling); }

  //! AUTO: Set function (needed in pybind) for: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  void PySetBackgroundColor(const std::array<float,4>& backgroundColorInit) { backgroundColor = backgroundColorInit; }
  //! AUTO: Read (Copy) access to: red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
  std::array<float,4> PyGetBackgroundColor() const { return std::array<float,4>(backgroundColor); }

  //! AUTO: Set function (needed in pybind) for: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  void PySetBackgroundColorBottom(const std::array<float,4>& backgroundColorBottomInit) { backgroundColorBottom = backgroundColorBottomInit; }
  //! AUTO: Read (Copy) access to: red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
  std::array<float,4> PyGetBackgroundColorBottom() const { return std::array<float,4>(backgroundColorBottom); }

  //! AUTO: Set function (needed in pybind) for: factor on boundingBox for zoom all (without minimum offset)
  void PySetBoundingBoxZoomAllFactor(const float& boundingBoxZoomAllFactorInit) { boundingBoxZoomAllFactor = EXUstd::GetSafelyPFloat(boundingBoxZoomAllFactorInit,"boundingBoxZoomAllFactor"); }
  //! AUTO: Read (Copy) access to: factor on boundingBox for zoom all (without minimum offset)
  float PyGetBoundingBoxZoomAllFactor() const { return float(boundingBoxZoomAllFactor); }

  //! AUTO: Set function (needed in pybind) for: minimum offset to bounding box of scene in window - width or height, whatever is smaller; adjust for very small or large scenes; may be negative
  void PySetBoundingBoxZoomAllOffset(const float& boundingBoxZoomAllOffsetInit) { boundingBoxZoomAllOffset = EXUstd::GetSafelyUFloat(boundingBoxZoomAllOffsetInit,"boundingBoxZoomAllOffset"); }
  //! AUTO: Read (Copy) access to: minimum offset to bounding box of scene in window - width or height, whatever is smaller; adjust for very small or large scenes; may be negative
  float PyGetBoundingBoxZoomAllOffset() const { return float(boundingBoxZoomAllOffset); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  void PySetCircleTiling(const Index& circleTilingInit) { circleTiling = EXUstd::GetSafelyPInt(circleTilingInit,"circleTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  Index PyGetCircleTiling() const { return Index(circleTiling); }

  //! AUTO: Set function (needed in pybind) for: size of coordinate system relative to font size
  void PySetCoordinateSystemSize(const float& coordinateSystemSizeInit) { coordinateSystemSize = EXUstd::GetSafelyPFloat(coordinateSystemSizeInit,"coordinateSystemSize"); }
  //! AUTO: Read (Copy) access to: size of coordinate system relative to font size
  float PyGetCoordinateSystemSize() const { return float(coordinateSystemSize); }

  //! AUTO: Set function (needed in pybind) for: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  void PySetCylinderTiling(const Index& cylinderTilingInit) { cylinderTiling = EXUstd::GetSafelyPInt(cylinderTilingInit,"cylinderTiling"); }
  //! AUTO: Read (Copy) access to: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  Index PyGetCylinderTiling() const { return Index(cylinderTiling); }

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.drawCoordinateSystem
  void PySetDrawCoordinateSystem(const Index& drawCoordinateSystemInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.drawCoordinateSystem
  Index PyGetDrawCoordinateSystem() const ;

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.drawWorldBasis
  void PySetDrawWorldBasis(const bool& drawWorldBasisInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.drawWorldBasis
  bool PyGetDrawWorldBasis() const ;

  //! AUTO: Set function (needed in pybind) for: interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
  void PySetGraphicsUpdateInterval(const float& graphicsUpdateIntervalInit) { graphicsUpdateInterval = EXUstd::GetSafelyUFloat(graphicsUpdateIntervalInit,"graphicsUpdateInterval"); }
  //! AUTO: Read (Copy) access to: interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
  float PyGetGraphicsUpdateInterval() const { return float(graphicsUpdateInterval); }

  //! AUTO: Set function (needed in pybind) for: Scaling factor for linux, which cannot determined from system by now; adjust this value to scale dialog fonts and renderer fonts
  void PySetLinuxDisplayScaleFactor(const float& linuxDisplayScaleFactorInit) { linuxDisplayScaleFactor = EXUstd::GetSafelyPFloat(linuxDisplayScaleFactorInit,"linuxDisplayScaleFactor"); }
  //! AUTO: Read (Copy) access to: Scaling factor for linux, which cannot determined from system by now; adjust this value to scale dialog fonts and renderer fonts
  float PyGetLinuxDisplayScaleFactor() const { return float(linuxDisplayScaleFactor); }

  //! AUTO: Set function (needed in pybind) for: minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
  void PySetMinSceneSize(const float& minSceneSizeInit) { minSceneSize = EXUstd::GetSafelyPFloat(minSceneSizeInit,"minSceneSize"); }
  //! AUTO: Read (Copy) access to: minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
  float PyGetMinSceneSize() const { return float(minSceneSize); }

  //! AUTO: Set function (needed in pybind) for: global point size (absolute)
  void PySetPointSize(const float& pointSizeInit) { pointSize = EXUstd::GetSafelyPFloat(pointSizeInit,"pointSize"); }
  //! AUTO: Read (Copy) access to: global point size (absolute)
  float PyGetPointSize() const { return float(pointSize); }

  //! AUTO: Set function (needed in pybind) for: number of seconds after which user is asked a security question before stopping simulation and closing renderer; set to 0 in order to always get asked; set to 1e10 to (nearly) never get asked
  void PySetReallyQuitTimeLimit(const Real& reallyQuitTimeLimitInit) { reallyQuitTimeLimit = EXUstd::GetSafelyUReal(reallyQuitTimeLimitInit,"reallyQuitTimeLimit"); }
  //! AUTO: Read (Copy) access to: number of seconds after which user is asked a security question before stopping simulation and closing renderer; set to 0 in order to always get asked; set to 1e10 to (nearly) never get asked
  Real PyGetReallyQuitTimeLimit() const { return Real(reallyQuitTimeLimit); }

  //! AUTO: Set function (needed in pybind) for: precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
  void PySetRendererPrecision(const Index& rendererPrecisionInit) { rendererPrecision = EXUstd::GetSafelyPInt(rendererPrecisionInit,"rendererPrecision"); }
  //! AUTO: Read (Copy) access to: precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
  Index PyGetRendererPrecision() const { return Index(rendererPrecision); }

  //! AUTO: Set function (needed in pybind) for: OpenGL render windows startup timeout in ms (change might be necessary if CPU is very slow)
  void PySetRendererStartupTimeout(const Index& rendererStartupTimeoutInit) { rendererStartupTimeout = EXUstd::GetSafelyPInt(rendererStartupTimeoutInit,"rendererStartupTimeout"); }
  //! AUTO: Read (Copy) access to: OpenGL render windows startup timeout in ms (change might be necessary if CPU is very slow)
  Index PyGetRendererStartupTimeout() const { return Index(rendererStartupTimeout); }

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.window.showComputationInfo
  void PySetShowComputationInfo(const bool& showComputationInfoInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.window.showComputationInfo
  bool PyGetShowComputationInfo() const ;

  //! AUTO: Set function (needed in pybind) for: seconds to show help message on startup (0=deactivate)
  void PySetShowHelpOnStartup(const Index& showHelpOnStartupInit) { showHelpOnStartup = EXUstd::GetSafelyUInt(showHelpOnStartupInit,"showHelpOnStartup"); }
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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsContourAdvanced // AUTO: 
{
public: // AUTO: 
  Index colorBarPrecision;                        //!< AUTO: precision of floating point values shown in color bar; total number of digits used (max. 16)
  Index colorBarTiling;                           //!< AUTO: number of tiles (segements) shown in the colorbar for the contour plot
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
  //! AUTO: Set function (needed in pybind) for: precision of floating point values shown in color bar; total number of digits used (max. 16)
  void PySetColorBarPrecision(const Index& colorBarPrecisionInit) { colorBarPrecision = EXUstd::GetSafelyPInt(colorBarPrecisionInit,"colorBarPrecision"); }
  //! AUTO: Read (Copy) access to: precision of floating point values shown in color bar; total number of digits used (max. 16)
  Index PyGetColorBarPrecision() const { return Index(colorBarPrecision); }

  //! AUTO: Set function (needed in pybind) for: number of tiles (segements) shown in the colorbar for the contour plot
  void PySetColorBarTiling(const Index& colorBarTilingInit) { colorBarTiling = EXUstd::GetSafelyPInt(colorBarTilingInit,"colorBarTiling"); }
  //! AUTO: Read (Copy) access to: number of tiles (segements) shown in the colorbar for the contour plot
  Index PyGetColorBarTiling() const { return Index(colorBarTiling); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for relative value 0 used for contour plot; alpha is ignored
  void PySetContourColor0(const std::array<float,4>& contourColor0Init) { contourColor0 = contourColor0Init; }
  //! AUTO: Read (Copy) access to: RGBA color for relative value 0 used for contour plot; alpha is ignored
  std::array<float,4> PyGetContourColor0() const { return std::array<float,4>(contourColor0); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  void PySetContourColor1(const std::array<float,4>& contourColor1Init) { contourColor1 = contourColor1Init; }
  //! AUTO: Read (Copy) access to: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  std::array<float,4> PyGetContourColor1() const { return std::array<float,4>(contourColor1); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  void PySetContourColor2(const std::array<float,4>& contourColor2Init) { contourColor2 = contourColor2Init; }
  //! AUTO: Read (Copy) access to: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  std::array<float,4> PyGetContourColor2() const { return std::array<float,4>(contourColor2); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  void PySetContourColor3(const std::array<float,4>& contourColor3Init) { contourColor3 = contourColor3Init; }
  //! AUTO: Read (Copy) access to: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  std::array<float,4> PyGetContourColor3() const { return std::array<float,4>(contourColor3); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  void PySetContourColor4(const std::array<float,4>& contourColor4Init) { contourColor4 = contourColor4Init; }
  //! AUTO: Read (Copy) access to: RGBA color for relative value 0.25 used for contour plot; alpha is ignored
  std::array<float,4> PyGetContourColor4() const { return std::array<float,4>(contourColor4); }

  //! AUTO: Set function (needed in pybind) for: RGBA color if relative value in contour plot is larger than 1 (if automaticRange=False); alpha is ignored
  void PySetContourColorMax(const std::array<float,4>& contourColorMaxInit) { contourColorMax = contourColorMaxInit; }
  //! AUTO: Read (Copy) access to: RGBA color if relative value in contour plot is larger than 1 (if automaticRange=False); alpha is ignored
  std::array<float,4> PyGetContourColorMax() const { return std::array<float,4>(contourColorMax); }

  //! AUTO: Set function (needed in pybind) for: RGBA color if relative value in contour plot is smaller than 0 (if automaticRange=False); alpha is ignored
  void PySetContourColorMin(const std::array<float,4>& contourColorMinInit) { contourColorMin = contourColorMinInit; }
  //! AUTO: Read (Copy) access to: RGBA color if relative value in contour plot is smaller than 0 (if automaticRange=False); alpha is ignored
  std::array<float,4> PyGetContourColorMin() const { return std::array<float,4>(contourColorMin); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
* @date         AUTO: 2026-04-03 (last modfied)
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
  Index showNodalSlopes;                          //!< AUTO: draw nodal slope vectors, e.g. in ANCF beam finite elements
  bool showNumbers;                               //!< AUTO: flag to decide, whether the node number is shown
  Index tiling;                                   //!< AUTO: tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4

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
  //! AUTO: Set function (needed in pybind) for: default RGBA color for nodes; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default RGBA color for nodes; 4th value is alpha-transparency
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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsBeams // AUTO: 
{
public: // AUTO: 
  Index axialTiling;                              //!< AUTO: number of segments to discretise the beams axis
  bool crossSectionFilled;                        //!< AUTO: if implemented for element, cross section is drawn as solid (filled) instead of wire-frame; NOTE: some quantities may not be interpolated correctly over cross section in visualization; equivalent to drawSolid of shells
  Index crossSectionTiling;                       //!< AUTO: number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing
  bool drawVertical;                              //!< AUTO: draw contour plot outputVariables 'vertical' along beam height; contour.outputVariable must be set accordingly
  Float4 drawVerticalColor;                       //!< AUTO: color for outputVariable to be drawn along cross section (vertically)
  float drawVerticalFactor;                       //!< AUTO: factor for outputVariable to be drawn along cross section (vertically)
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

  //! AUTO: Set function (needed in pybind) for: factor for outputVariable to be drawn along cross section (vertically)
  void PySetDrawVerticalFactor(const float& drawVerticalFactorInit) { drawVerticalFactor = EXUstd::GetSafelyUFloat(drawVerticalFactorInit,"drawVerticalFactor"); }
  //! AUTO: Read (Copy) access to: factor for outputVariable to be drawn along cross section (vertically)
  float PyGetDrawVerticalFactor() const { return float(drawVerticalFactor); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  float thicknessFactor;                          //!< AUTO: a factor multiplied with the thickness of shells/plates only for visualization (e.g. to make some effects more visible)

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
  //! AUTO: Set function (needed in pybind) for: a factor multiplied with the thickness of shells/plates only for visualization (e.g. to make some effects more visible)
  void PySetThicknessFactor(const float& thicknessFactorInit) { thicknessFactor = EXUstd::GetSafelyPFloat(thicknessFactorInit,"thicknessFactor"); }
  //! AUTO: Read (Copy) access to: a factor multiplied with the thickness of shells/plates only for visualization (e.g. to make some effects more visible)
  float PyGetThicknessFactor() const { return float(thicknessFactor); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
* @date         AUTO: 2026-04-03 (last modfied)
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
  float deformationScaleFactor;                   //!< AUTO: global deformation scale factor; also applies to nodes, if drawn; currently only used for scaled drawing of (linear) finite elements in FFRF and FFRFreducedOrder objects
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
  //! AUTO: Set function (needed in pybind) for: default RGBA color for bodies; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default RGBA color for bodies; 4th value is alpha-transparency
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
* @date         AUTO: 2026-04-03 (last modfied)
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
  Float4 defaultColor;                            //!< AUTO: default RGBA color for connectors; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global connector size; if -1.f, connector size is relative to maxSceneSize
  float jointAxesLength;                          //!< AUTO: global joint axes length
  float jointAxesRadius;                          //!< AUTO: global joint axes radius
  bool show;                                      //!< AUTO: flag to decide, whether the connectors are shown
  bool showContact;                               //!< AUTO: flag to decide, whether contact points, lines, etc. are shown for special cable-circle contacts; for spheres, triangles, tori, see visualizationSettings.contact
  bool showJointAxes;                             //!< AUTO: flag to decide, whether contact joint axes of 3D joints are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the connector(=object) number is shown
  Index springNumberOfWindings;                   //!< AUTO: number of windings for springs drawn as helical spring

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsConnectors()
  {
    backlink=nullptr;
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
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default RGBA color for connectors; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default RGBA color for connectors; 4th value is alpha-transparency
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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsMarkers // AUTO: 
{
public: // AUTO: 
  Float4 defaultColor;                            //!< AUTO: default RGBA color for markers; 4th value is alpha-transparency
  float defaultSize;                              //!< AUTO: global marker size; if -1.f, marker size is relative to maxSceneSize
  bool drawSimplified;                            //!< AUTO: draw markers with simplified symbols
  bool show;                                      //!< AUTO: flag to decide, whether the markers are shown
  bool showNumbers;                               //!< AUTO: flag to decide, whether the marker numbers are shown

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsMarkers()
  {
    backlink=nullptr;
    defaultColor = Float4({0.1f,0.5f,0.1f,1.f});
    defaultSize = -1.f;
    drawSimplified = true;
    show = true;
    showNumbers = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default RGBA color for markers; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default RGBA color for markers; 4th value is alpha-transparency
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
* @date         AUTO: 2026-04-03 (last modfied)
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
  //! AUTO: Set function (needed in pybind) for: default RGBA color for loads; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default RGBA color for loads; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

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
* @brief        Visualization settings for traces of sensors. Note that a large number of time points (influenced by simulationSettings.solutionSettings.sensorsWritePeriod) may lead to slow graphics.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsTraces // AUTO: 
{
public: // AUTO: 
  float lineWidth;                                //!< AUTO: line width for traces
  ArrayIndex listOfPositionSensors;               //!< AUTO: list of position sensors which can be shown as trace inside render window if sensors have storeInternal=True; if this list is empty and showPositionTrace=True, then all available sensors are shown
  ArrayIndex listOfTriadSensors;                  //!< AUTO: list of sensors of with OutputVariableType RotationMatrix; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showTriads=True; the triad is drawn at the related position
  ArrayIndex listOfVectorSensors;                 //!< AUTO: list of sensors with 3D vector quantities; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showVectors=True; the vector quantity is drawn relative to the related position
  Index positionsShowEvery;                       //!< AUTO: integer value i; out of available sensor data, show every i-th position
  Index sensorsMbsNumber;                         //!< AUTO: number of main system which is used to for sensor lists; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number
  bool showCurrent;                               //!< AUTO: show current trace position (and especially vector quantity) related to current visualization state; this only works in solution viewer if sensor values are stored at time grid points of the solution file (up to a precision of 1e-10) and may therefore be temporarily unavailable
  bool showFuture;                                //!< AUTO: show trace future to current visualization state if already computed (e.g. in SolutionViewer)
  bool showPast;                                  //!< AUTO: show trace previous to current visualization state
  bool showPositionTrace;                         //!< AUTO: show position trace of all position sensors if listOfPositionSensors=[] or of specified sensors; sensors need to activate storeInternal=True
  bool showTriads;                                //!< AUTO: if True, show basis vectors from rotation matrices provided by sensors
  bool showVectors;                               //!< AUTO: if True, show vector quantities according to description in showPositionTrace
  Real timeSpan;                                  //!< AUTO: maximum trace time span of past or future trace; given in seconds of simulation time; if zero, it is unused
  ArrayFloat traceColors;                         //!< AUTO: RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue
  float triadSize;                                //!< AUTO: length of triad axes if shown
  Index triadsShowEvery;                          //!< AUTO: integer value i; out of available sensor data, show every i-th triad
  float vectorScaling;                            //!< AUTO: scaling of vector quantities; if, e.g., loads, this factor has to be adjusted significantly
  Index vectorsShowEvery;                         //!< AUTO: integer value i; out of available sensor data, show every i-th vector

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
  //! AUTO: Set function (needed in pybind) for: line width for traces
  void PySetLineWidth(const float& lineWidthInit) { lineWidth = EXUstd::GetSafelyUFloat(lineWidthInit,"lineWidth"); }
  //! AUTO: Read (Copy) access to: line width for traces
  float PyGetLineWidth() const { return float(lineWidth); }

  //! AUTO: Set function (needed in pybind) for: list of position sensors which can be shown as trace inside render window if sensors have storeInternal=True; if this list is empty and showPositionTrace=True, then all available sensors are shown
  void PySetListOfPositionSensors(const std::vector<Index>& listOfPositionSensorsInit) { listOfPositionSensors = listOfPositionSensorsInit; }
  //! AUTO: Read (Copy) access to: list of position sensors which can be shown as trace inside render window if sensors have storeInternal=True; if this list is empty and showPositionTrace=True, then all available sensors are shown
  std::vector<Index> PyGetListOfPositionSensors() const { return std::vector<Index>(listOfPositionSensors); }

  //! AUTO: Set function (needed in pybind) for: list of sensors of with OutputVariableType RotationMatrix; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showTriads=True; the triad is drawn at the related position
  void PySetListOfTriadSensors(const std::vector<Index>& listOfTriadSensorsInit) { listOfTriadSensors = listOfTriadSensorsInit; }
  //! AUTO: Read (Copy) access to: list of sensors of with OutputVariableType RotationMatrix; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showTriads=True; the triad is drawn at the related position
  std::vector<Index> PyGetListOfTriadSensors() const { return std::vector<Index>(listOfTriadSensors); }

  //! AUTO: Set function (needed in pybind) for: list of sensors with 3D vector quantities; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showVectors=True; the vector quantity is drawn relative to the related position
  void PySetListOfVectorSensors(const std::vector<Index>& listOfVectorSensorsInit) { listOfVectorSensors = listOfVectorSensorsInit; }
  //! AUTO: Read (Copy) access to: list of sensors with 3D vector quantities; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showVectors=True; the vector quantity is drawn relative to the related position
  std::vector<Index> PyGetListOfVectorSensors() const { return std::vector<Index>(listOfVectorSensors); }

  //! AUTO: Set function (needed in pybind) for: integer value i; out of available sensor data, show every i-th position
  void PySetPositionsShowEvery(const Index& positionsShowEveryInit) { positionsShowEvery = EXUstd::GetSafelyPInt(positionsShowEveryInit,"positionsShowEvery"); }
  //! AUTO: Read (Copy) access to: integer value i; out of available sensor data, show every i-th position
  Index PyGetPositionsShowEvery() const { return Index(positionsShowEvery); }

  //! AUTO: Set function (needed in pybind) for: maximum trace time span of past or future trace; given in seconds of simulation time; if zero, it is unused
  void PySetTimeSpan(const Real& timeSpanInit) { timeSpan = EXUstd::GetSafelyUReal(timeSpanInit,"timeSpan"); }
  //! AUTO: Read (Copy) access to: maximum trace time span of past or future trace; given in seconds of simulation time; if zero, it is unused
  Real PyGetTimeSpan() const { return Real(timeSpan); }

  //! AUTO: Set function (needed in pybind) for: RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue
  void PySetTraceColors(const std::vector<float>& traceColorsInit) { traceColors = traceColorsInit; }
  //! AUTO: Read (Copy) access to: RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue
  std::vector<float> PyGetTraceColors() const { return std::vector<float>(traceColors); }

  //! AUTO: Set function (needed in pybind) for: integer value i; out of available sensor data, show every i-th triad
  void PySetTriadsShowEvery(const Index& triadsShowEveryInit) { triadsShowEvery = EXUstd::GetSafelyPInt(triadsShowEveryInit,"triadsShowEvery"); }
  //! AUTO: Read (Copy) access to: integer value i; out of available sensor data, show every i-th triad
  Index PyGetTriadsShowEvery() const { return Index(triadsShowEvery); }

  //! AUTO: Set function (needed in pybind) for: integer value i; out of available sensor data, show every i-th vector
  void PySetVectorsShowEvery(const Index& vectorsShowEveryInit) { vectorsShowEvery = EXUstd::GetSafelyPInt(vectorsShowEveryInit,"vectorsShowEvery"); }
  //! AUTO: Read (Copy) access to: integer value i; out of available sensor data, show every i-th vector
  Index PyGetVectorsShowEvery() const { return Index(vectorsShowEvery); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  //! AUTO: Set function (needed in pybind) for: default RGBA color for sensors; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default RGBA color for sensors; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return std::array<float,4>(defaultColor); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  Index tilingCurves;                             //!< AUTO: tiling for nonlinear/polynomial curves; higher values give smoother curves
  Index tilingSpheres;                            //!< AUTO: tiling for spheres; higher values give smoother spheres, but may lead to lower frame rates

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
  //! AUTO: Set function (needed in pybind) for: RGBA color for boudnding boxes, see showBoundingBoxes
  void PySetColorBoundingBoxes(const std::array<float,4>& colorBoundingBoxesInit) { colorBoundingBoxes = colorBoundingBoxesInit; }
  //! AUTO: Read (Copy) access to: RGBA color for boudnding boxes, see showBoundingBoxes
  std::array<float,4> PyGetColorBoundingBoxes() const { return std::array<float,4>(colorBoundingBoxes); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for search tree, see showSearchTree
  void PySetColorSearchTree(const std::array<float,4>& colorSearchTreeInit) { colorSearchTree = colorSearchTreeInit; }
  //! AUTO: Read (Copy) access to: RGBA color for search tree, see showSearchTree
  std::array<float,4> PyGetColorSearchTree() const { return std::array<float,4>(colorSearchTree); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for contact spheres, see showSpheres
  void PySetColorSpheres(const std::array<float,4>& colorSpheresInit) { colorSpheres = colorSpheresInit; }
  //! AUTO: Read (Copy) access to: RGBA color for contact spheres, see showSpheres
  std::array<float,4> PyGetColorSpheres() const { return std::array<float,4>(colorSpheres); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for contact tori, see showTori
  void PySetColorTori(const std::array<float,4>& colorToriInit) { colorTori = colorToriInit; }
  //! AUTO: Read (Copy) access to: RGBA color for contact tori, see showTori
  std::array<float,4> PyGetColorTori() const { return std::array<float,4>(colorTori); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for contact triangles, see showTriangles
  void PySetColorTriangles(const std::array<float,4>& colorTrianglesInit) { colorTriangles = colorTrianglesInit; }
  //! AUTO: Read (Copy) access to: RGBA color for contact triangles, see showTriangles
  std::array<float,4> PyGetColorTriangles() const { return std::array<float,4>(colorTriangles); }

  //! AUTO: Set function (needed in pybind) for: tiling for nonlinear/polynomial curves; higher values give smoother curves
  void PySetTilingCurves(const Index& tilingCurvesInit) { tilingCurves = EXUstd::GetSafelyPInt(tilingCurvesInit,"tilingCurves"); }
  //! AUTO: Read (Copy) access to: tiling for nonlinear/polynomial curves; higher values give smoother curves
  Index PyGetTilingCurves() const { return Index(tilingCurves); }

  //! AUTO: Set function (needed in pybind) for: tiling for spheres; higher values give smoother spheres, but may lead to lower frame rates
  void PySetTilingSpheres(const Index& tilingSpheresInit) { tilingSpheres = EXUstd::GetSafelyPInt(tilingSpheresInit,"tilingSpheres"); }
  //! AUTO: Read (Copy) access to: tiling for spheres; higher values give smoother spheres, but may lead to lower frame rates
  Index PyGetTilingSpheres() const { return Index(tilingSpheres); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  float perspective;                              //!< AUTO: parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 1 (extreme: 5), where larger values are possible but should be used with care; NOTE that the relation to the common field of view (FOV) angle alpha, with alpha=90°, is given by perspective = tan(alpha/2) = 1; mouse coordinates (F3) can not be shown with perspective>0
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
  //! AUTO: Set function (needed in pybind) for: if modelCentricView=True: offset to camera position in model view (and, if used, relative to tracked marker - instead of a tracked marker position, you could also just change the camera position in camera-centric views); camera rotation follows modelRotation in renderState
  void PySetCameraPosition(const std::array<float,3>& cameraPositionInit) { cameraPosition = cameraPositionInit; }
  //! AUTO: Read (Copy) access to: if modelCentricView=True: offset to camera position in model view (and, if used, relative to tracked marker - instead of a tracked marker position, you could also just change the camera position in camera-centric views); camera rotation follows modelRotation in renderState
  std::array<float,3> PyGetCameraPosition() const { return std::array<float,3>(cameraPosition); }

  //! AUTO: Set function (needed in pybind) for: normal vector of clipping plane, e.g. [0,0,1] to set a xy-clipping plane; the clipped half-space is in direction of the normal; use [0,0,0] to deactivate clipping plane; Note that clipping is mainly made for triangles in order to visualize hidden objects and currently it only fully clips triangles, but does not exactly cut them; see also clippingPlaneDistance and openGL.advanced.clippingPlaneColor
  void PySetClippingPlaneNormal(const std::array<float,3>& clippingPlaneNormalInit) { clippingPlaneNormal = clippingPlaneNormalInit; }
  //! AUTO: Read (Copy) access to: normal vector of clipping plane, e.g. [0,0,1] to set a xy-clipping plane; the clipped half-space is in direction of the normal; use [0,0,0] to deactivate clipping plane; Note that clipping is mainly made for triangles in order to visualize hidden objects and currently it only fully clips triangles, but does not exactly cut them; see also clippingPlaneDistance and openGL.advanced.clippingPlaneColor
  std::array<float,3> PyGetClippingPlaneNormal() const { return std::array<float,3>(clippingPlaneNormal); }

  //! AUTO: Set function (needed in pybind) for: the three values are [nearPlaneOffset, farPlaneOffset, flag]; if flag=0, the offsets are ignored and computed automatically, using x = 2 * maxSceneSize * zMaxSceneFactor, setting near plane to -x and far plane to +x in case of modelCentricView=True and setting near plane to 0.01 (minimal offset to eye point) and far plane to +x if modelCentricView=False; if flag=1, the near and far plane values are just overwritten; note that positive values for near plane make objects in front of the camera invisible while negative values make objects behind the camera plane visible; in case of camera-centric view, the eyepoint can be shifted backwards using cameraPosition accordingly.
  void PySetNearFarPlaneOffset(const std::array<float,3>& nearFarPlaneOffsetInit) { nearFarPlaneOffset = nearFarPlaneOffsetInit; }
  //! AUTO: Read (Copy) access to: the three values are [nearPlaneOffset, farPlaneOffset, flag]; if flag=0, the offsets are ignored and computed automatically, using x = 2 * maxSceneSize * zMaxSceneFactor, setting near plane to -x and far plane to +x in case of modelCentricView=True and setting near plane to 0.01 (minimal offset to eye point) and far plane to +x if modelCentricView=False; if flag=1, the near and far plane values are just overwritten; note that positive values for near plane make objects in front of the camera invisible while negative values make objects behind the camera plane visible; in case of camera-centric view, the eyepoint can be shifted backwards using cameraPosition accordingly.
  std::array<float,3> PyGetNearFarPlaneOffset() const { return std::array<float,3>(nearFarPlaneOffset); }

  //! AUTO: Set function (needed in pybind) for: parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 1 (extreme: 5), where larger values are possible but should be used with care; NOTE that the relation to the common field of view (FOV) angle alpha, with alpha=90°, is given by perspective = tan(alpha/2) = 1; mouse coordinates (F3) can not be shown with perspective>0
  void PySetPerspective(const float& perspectiveInit) { perspective = EXUstd::GetSafelyUFloat(perspectiveInit,"perspective"); }
  //! AUTO: Read (Copy) access to: parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 1 (extreme: 5), where larger values are possible but should be used with care; NOTE that the relation to the common field of view (FOV) angle alpha, with alpha=90°, is given by perspective = tan(alpha/2) = 1; mouse coordinates (F3) can not be shown with perspective>0
  float PyGetPerspective() const { return float(perspective); }

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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsScene // AUTO: 
{
public: // AUTO: 
  Index drawCoordinateSystem;                     //!< AUTO: 0 = no coordinate system shown, 1 = draw lines with text, 2 = draw arrows, 3 = draw arrows with text
  bool drawWorldBasis;                            //!< AUTO: true = draw world basis coordinate system at (0,0,0)
  bool facesTransparent;                          //!< AUTO: True: show faces transparent independent of transparency (A)-value in color of objects; allow to show otherwise hidden node/marker/object numbers
  bool showFaceEdges;                             //!< AUTO: True: show edges of triangles; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
  bool showFaces;                                 //!< AUTO: True: show faces of triangles, etc.; using the options showFaces=false and showFaceEdges=true gives are wireframe representation
  bool showLines;                                 //!< AUTO: True: show lines (other lines than face and mesh edges)
  bool showMeshEdges;                             //!< AUTO: True: show edges of finite elements; independent of showFaceEdges
  bool showMeshFaces;                             //!< AUTO: True: show faces of finite elements; independent of showFaces
  float worldBasisSize;                           //!< AUTO: size of world basis coordinate system

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
    worldBasisSize = 1.0f;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: 0 = no coordinate system shown, 1 = draw lines with text, 2 = draw arrows, 3 = draw arrows with text
  void PySetDrawCoordinateSystem(const Index& drawCoordinateSystemInit) { drawCoordinateSystem = EXUstd::GetSafelyUInt(drawCoordinateSystemInit,"drawCoordinateSystem"); }
  //! AUTO: Read (Copy) access to: 0 = no coordinate system shown, 1 = draw lines with text, 2 = draw arrows, 3 = draw arrows with text
  Index PyGetDrawCoordinateSystem() const { return Index(drawCoordinateSystem); }

  //! AUTO: Set function (needed in pybind) for: size of world basis coordinate system
  void PySetWorldBasisSize(const float& worldBasisSizeInit) { worldBasisSize = EXUstd::GetSafelyPFloat(worldBasisSizeInit,"worldBasisSize"); }
  //! AUTO: Read (Copy) access to: size of world basis coordinate system
  float PyGetWorldBasisSize() const { return float(worldBasisSize); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  float globalFontSize;                           //!< AUTO: general text font size (roughly measured in pixels); if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
  bool lockModelView;                             //!< AUTO: True: all movements (with mouse/keys), rotations, zoom are disabled; the view is either based on initial values (or on the current state) ==> initial zoom, rotation and center point need to be adjusted, approx. 0.4*maxSceneSize is a good value
  bool maximize;                                  //!< AUTO: True: render window of respective view will be maximized at startup
  Index2 renderWindowSize;                        //!< AUTO: initial size of render window of respective view for specific view in pixels for
  bool showComputationInfo;                       //!< AUTO: true = show (hide) all computation information including Exudyn and version
  bool showMouseCoordinates;                      //!< AUTO: True: show OpenGL coordinates and distance to last left mouse button pressed position in renderer status message; switched on/off with key 'F3'; only works for axis-aligned ortho-projections
  bool showRenderStateInfo;                       //!< AUTO: True: show renderer.state infos regarding zoom, offset and rotation in renderer status message; switched on/off with 'CTRL-F3'
  bool showWindow;                                //!< AUTO: True: render window of respective view is shown when created; False: window will be iconified when created (e.g. if you are starting multiple computations automatically)

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
    renderWindowSize = Index2({1024,768});
    showComputationInfo = true;
    showMouseCoordinates = false;
    showRenderStateInfo = false;
    showWindow = true;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: general text font size (roughly measured in pixels); if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
  void PySetGlobalFontSize(const float& globalFontSizeInit) { globalFontSize = EXUstd::GetSafelyPFloat(globalFontSizeInit,"globalFontSize"); }
  //! AUTO: Read (Copy) access to: general text font size (roughly measured in pixels); if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
  float PyGetGlobalFontSize() const { return float(globalFontSize); }

  //! AUTO: Set function (needed in pybind) for: initial size of render window of respective view for specific view in pixels for
  void PySetRenderWindowSize(const std::array<Index,2>& renderWindowSizeInit) { renderWindowSize = renderWindowSizeInit; }
  //! AUTO: Read (Copy) access to: initial size of render window of respective view for specific view in pixels for
  std::array<Index,2> PyGetRenderWindowSize() const { return std::array<Index,2>(renderWindowSize); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsWindow" << ":\n";
    os << "  alwaysOnTop = " << alwaysOnTop << "\n";
    os << "  globalFontSize = " << globalFontSize << "\n";
    os << "  lockModelView = " << lockModelView << "\n";
    os << "  maximize = " << maximize << "\n";
    os << "  renderWindowSize = " << renderWindowSize << "\n";
    os << "  showComputationInfo = " << showComputationInfo << "\n";
    os << "  showMouseCoordinates = " << showMouseCoordinates << "\n";
    os << "  showRenderStateInfo = " << showRenderStateInfo << "\n";
    os << "  showWindow = " << showWindow << "\n";
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
* @date         AUTO: 2026-04-03 (last modfied)
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
* @date         AUTO: 2026-04-03 (last modfied)
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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsDialogs // AUTO: 
{
public: // AUTO: 
  float alphaTransparency;                        //!< AUTO: alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)
  bool alwaysTopmost;                             //!< AUTO: True: dialogs are always topmost (otherwise, they are sometimes hidden)
  float fontScalingMacOS;                         //!< AUTO: font scaling value for MacOS systems (on Windows, system display scaling is used)
  bool multiThreadedDialogs;                      //!< AUTO: True: During dialogs, the OpenGL render windows will still get updates of changes in dialogs, etc., which may cause problems on some platforms or for some (complicated) models; False: changes of dialogs will take effect when dialogs are closed
  bool openTreeView;                              //!< AUTO: True: all sub-trees of the visusalization dialog are opened when opening the dialog; False: only some sub-trees are opened

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsDialogs()
  {
    backlink=nullptr;
    alphaTransparency = 0.94f;
    alwaysTopmost = true;
    fontScalingMacOS = 1.35f;
    multiThreadedDialogs = true;
    openTreeView = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

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
* @class        VSettingsMaterial
* @brief        Settings for rendering materials, in particular for the Raytracer (may be available also in the OpenGL renderer in the future). This material (widely follows Phong model) can be either accessed via SC.renderer.materials or directly in visualizationSettings.raytracer.material0, material1, etc.; note that the default values shown in the documentation only reflect material0 but not all 10 default materials.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsMaterial // AUTO: 
{
public: // AUTO: 
  float alpha;                                    //!< AUTO: alpha-transparency, same as in alpha channel in RGBA colors; 1=opaque, 0=fully transparent; leads to extra rendering costs per transparent pixel
  Float3 baseColor;                               //!< AUTO: RGB default material color if face color has R-color channel -1
  Float3 emission;                                //!< AUTO: RGB emissive material color (enlightened material)
  float ior;                                      //!< AUTO: index of refraction for transparent materials (1=no refraction), >1 represents refraction
  std::string name;                               //!< AUTO: material name for easier handling
  float reflectivity;                             //!< AUTO: controls reflectivity of material; 0=no reflections (rough, e.g. rubber), 1=fully reflective (mirror); this leads to large extra rendering costs per visible reflective pixel
  float shininess;                                //!< AUTO: controls shininess of specular component of lights; values < 5 is not very shiny, while > 50 is very shiny
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
  //! AUTO: Set function (needed in pybind) for: alpha-transparency, same as in alpha channel in RGBA colors; 1=opaque, 0=fully transparent; leads to extra rendering costs per transparent pixel
  void PySetAlpha(const float& alphaInit) { alpha = EXUstd::GetSafelyUFloat(alphaInit,"alpha"); }
  //! AUTO: Read (Copy) access to: alpha-transparency, same as in alpha channel in RGBA colors; 1=opaque, 0=fully transparent; leads to extra rendering costs per transparent pixel
  float PyGetAlpha() const { return float(alpha); }

  //! AUTO: Set function (needed in pybind) for: RGB default material color if face color has R-color channel -1
  void PySetBaseColor(const std::array<float,3>& baseColorInit) { baseColor = baseColorInit; }
  //! AUTO: Read (Copy) access to: RGB default material color if face color has R-color channel -1
  std::array<float,3> PyGetBaseColor() const { return std::array<float,3>(baseColor); }

  //! AUTO: Set function (needed in pybind) for: RGB emissive material color (enlightened material)
  void PySetEmission(const std::array<float,3>& emissionInit) { emission = emissionInit; }
  //! AUTO: Read (Copy) access to: RGB emissive material color (enlightened material)
  std::array<float,3> PyGetEmission() const { return std::array<float,3>(emission); }

  //! AUTO: Set function (needed in pybind) for: index of refraction for transparent materials (1=no refraction), >1 represents refraction
  void PySetIor(const float& iorInit) { ior = EXUstd::GetSafelyUFloat(iorInit,"ior"); }
  //! AUTO: Read (Copy) access to: index of refraction for transparent materials (1=no refraction), >1 represents refraction
  float PyGetIor() const { return float(ior); }

  //! AUTO: Set function (needed in pybind) for: controls reflectivity of material; 0=no reflections (rough, e.g. rubber), 1=fully reflective (mirror); this leads to large extra rendering costs per visible reflective pixel
  void PySetReflectivity(const float& reflectivityInit) { reflectivity = EXUstd::GetSafelyUFloat(reflectivityInit,"reflectivity"); }
  //! AUTO: Read (Copy) access to: controls reflectivity of material; 0=no reflections (rough, e.g. rubber), 1=fully reflective (mirror); this leads to large extra rendering costs per visible reflective pixel
  float PyGetReflectivity() const { return float(reflectivity); }

  //! AUTO: Set function (needed in pybind) for: controls shininess of specular component of lights; values < 5 is not very shiny, while > 50 is very shiny
  void PySetShininess(const float& shininessInit) { shininess = EXUstd::GetSafelyUFloat(shininessInit,"shininess"); }
  //! AUTO: Read (Copy) access to: controls shininess of specular component of lights; values < 5 is not very shiny, while > 50 is very shiny
  float PyGetShininess() const { return float(shininess); }

  //! AUTO: Set function (needed in pybind) for: RGB specular material color
  void PySetSpecular(const std::array<float,3>& specularInit) { specular = specularInit; }
  //! AUTO: Read (Copy) access to: RGB specular material color
  std::array<float,3> PyGetSpecular() const { return std::array<float,3>(specular); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  Index searchTreeFactor;                         //!< AUTO: This factor can be used to increase the number of search tree bins, which can improve performance in case of inequilibrated scense; range=1..128
  Index shadowScalingFactor;                      //!< AUTO: if lightRadiusVariations>1, this defines the downscaling factor of the shadow map, where 2 means that the resolution is 2 times smaller than the image resolution; additionally, multisampling is not used for shadow map computation if shadowScalingFactor>0, thus reducing the computational effort for shadow computation also in case of 1; range=0..16; larger values cause significant artifacts at shadow boundaries
  Index shadowSmoothingSteps;                     //!< AUTO: if lightRadiusVariations>1, this defines the number of smoothing steps at the low-resolution shadow map; smoothing reduces shadow artifacts caused by smaller values of lightRadiusVariations; range=0..32; smoothing  steps may cause artifacts at shadow boundaries; only works for directional lights with position (e.g. 4th component in light0Position should be 1)
  bool showText;                                  //!< AUTO: True: show any kind of status text, node numbers, object numbers, etc. (depending on settings); False: do not show any text in raytracer, independently of settings
  Index tilesPerThread;                           //!< AUTO: Total number of sub-tiles per thread, used to evenly distribute rendering load to threads
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
    zBiasLines = 1e-3f;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: scene RGBA color for background that is hit by reflection material; while openGL.backgroundColor is used for rays that do not hit an object, this background may - if black or white - not be a suitable color for computing reflections; this is generally needed, as our scenes are usually not inside a closed geometry (like inside a room); this color is also used if maxReflectionDepth is reached
  void PySetBackgroundColorReflections(const std::array<float,4>& backgroundColorReflectionsInit) { backgroundColorReflections = backgroundColorReflectionsInit; }
  //! AUTO: Read (Copy) access to: scene RGBA color for background that is hit by reflection material; while openGL.backgroundColor is used for rays that do not hit an object, this background may - if black or white - not be a suitable color for computing reflections; this is generally needed, as our scenes are usually not inside a closed geometry (like inside a room); this color is also used if maxReflectionDepth is reached
  std::array<float,4> PyGetBackgroundColorReflections() const { return std::array<float,4>(backgroundColorReflections); }

  //! AUTO: Set function (needed in pybind) for: This factor can be used to increase the number of search tree bins, which can improve performance in case of inequilibrated scense; range=1..128
  void PySetSearchTreeFactor(const Index& searchTreeFactorInit) { searchTreeFactor = EXUstd::GetSafelyPInt(searchTreeFactorInit,"searchTreeFactor"); }
  //! AUTO: Read (Copy) access to: This factor can be used to increase the number of search tree bins, which can improve performance in case of inequilibrated scense; range=1..128
  Index PyGetSearchTreeFactor() const { return Index(searchTreeFactor); }

  //! AUTO: Set function (needed in pybind) for: if lightRadiusVariations>1, this defines the downscaling factor of the shadow map, where 2 means that the resolution is 2 times smaller than the image resolution; additionally, multisampling is not used for shadow map computation if shadowScalingFactor>0, thus reducing the computational effort for shadow computation also in case of 1; range=0..16; larger values cause significant artifacts at shadow boundaries
  void PySetShadowScalingFactor(const Index& shadowScalingFactorInit) { shadowScalingFactor = EXUstd::GetSafelyUInt(shadowScalingFactorInit,"shadowScalingFactor"); }
  //! AUTO: Read (Copy) access to: if lightRadiusVariations>1, this defines the downscaling factor of the shadow map, where 2 means that the resolution is 2 times smaller than the image resolution; additionally, multisampling is not used for shadow map computation if shadowScalingFactor>0, thus reducing the computational effort for shadow computation also in case of 1; range=0..16; larger values cause significant artifacts at shadow boundaries
  Index PyGetShadowScalingFactor() const { return Index(shadowScalingFactor); }

  //! AUTO: Set function (needed in pybind) for: if lightRadiusVariations>1, this defines the number of smoothing steps at the low-resolution shadow map; smoothing reduces shadow artifacts caused by smaller values of lightRadiusVariations; range=0..32; smoothing  steps may cause artifacts at shadow boundaries; only works for directional lights with position (e.g. 4th component in light0Position should be 1)
  void PySetShadowSmoothingSteps(const Index& shadowSmoothingStepsInit) { shadowSmoothingSteps = EXUstd::GetSafelyUInt(shadowSmoothingStepsInit,"shadowSmoothingSteps"); }
  //! AUTO: Read (Copy) access to: if lightRadiusVariations>1, this defines the number of smoothing steps at the low-resolution shadow map; smoothing reduces shadow artifacts caused by smaller values of lightRadiusVariations; range=0..32; smoothing  steps may cause artifacts at shadow boundaries; only works for directional lights with position (e.g. 4th component in light0Position should be 1)
  Index PyGetShadowSmoothingSteps() const { return Index(shadowSmoothingSteps); }

  //! AUTO: Set function (needed in pybind) for: Total number of sub-tiles per thread, used to evenly distribute rendering load to threads
  void PySetTilesPerThread(const Index& tilesPerThreadInit) { tilesPerThread = EXUstd::GetSafelyPInt(tilesPerThreadInit,"tilesPerThread"); }
  //! AUTO: Read (Copy) access to: Total number of sub-tiles per thread, used to evenly distribute rendering load to threads
  Index PyGetTilesPerThread() const { return Index(tilesPerThread); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  float globalFogDensity;                         //!< AUTO: global fog density; fog is deactivated if fogDensity=0, otherwise it is a density relative to scene max size; as it is relative, the factor has to be relatively high to be visible (usually >1)
  Index imageSizeFactor;                          //!< AUTO: Special size factor (1-16) to allow drawing with smaller resolution (faster); use this for long rendering times for adjustments, etc.
  bool keepWindowActive;                          //!< AUTO: Special flag, handle with care; True: sends some glfw functions to keep window reactive for long render times (>2 seconds); otherwise, the rendering may not finish due to timeout
  Index lightRadiusVariations;                    //!< AUTO: if lightRadiusVariations>1, this defines the number of positions that are used to compute the effect of distributed lights (larger is slower but better quality); range=1..256; avoid squares of integers; good values: 1 (hard shadow boundaries), 6, 13, 20, 31, 72, 130, 240; for lower values, use shadowSmoothingSteps=2..8
  Index maxReflectionDepth;                       //!< AUTO: Maximum number of reflections computed for one ray (note that for each transparent face passed, the reflection depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  Index maxTransparencyDepth;                     //!< AUTO: Maximum number of transparent faces that can be passed (note that for each reflection, the transparency depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  Index multiSampling;                            //!< AUTO: Multi-sampling used for rendering of faces, lines and text; increases image quality along edges (lines, etc.) but INCREASES rendering costs dramatically (multiSampling=3 => 3x3=9 times slower); also used for shadow if shadowScalingFactor=0; values only accepted in range [1..4]
  Index numberOfThreads;                          //!< AUTO: Number of CPU-threads (max: 256) used for software rendering (should be approx. the number of available threads)
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

  //! AUTO: Set function (needed in pybind) for: scene RGBA fog color
  void PySetGlobalFogColor(const std::array<float,4>& globalFogColorInit) { globalFogColor = globalFogColorInit; }
  //! AUTO: Read (Copy) access to: scene RGBA fog color
  std::array<float,4> PyGetGlobalFogColor() const { return std::array<float,4>(globalFogColor); }

  //! AUTO: Set function (needed in pybind) for: global fog density; fog is deactivated if fogDensity=0, otherwise it is a density relative to scene max size; as it is relative, the factor has to be relatively high to be visible (usually >1)
  void PySetGlobalFogDensity(const float& globalFogDensityInit) { globalFogDensity = EXUstd::GetSafelyUFloat(globalFogDensityInit,"globalFogDensity"); }
  //! AUTO: Read (Copy) access to: global fog density; fog is deactivated if fogDensity=0, otherwise it is a density relative to scene max size; as it is relative, the factor has to be relatively high to be visible (usually >1)
  float PyGetGlobalFogDensity() const { return float(globalFogDensity); }

  //! AUTO: Set function (needed in pybind) for: Special size factor (1-16) to allow drawing with smaller resolution (faster); use this for long rendering times for adjustments, etc.
  void PySetImageSizeFactor(const Index& imageSizeFactorInit) { imageSizeFactor = EXUstd::GetSafelyPInt(imageSizeFactorInit,"imageSizeFactor"); }
  //! AUTO: Read (Copy) access to: Special size factor (1-16) to allow drawing with smaller resolution (faster); use this for long rendering times for adjustments, etc.
  Index PyGetImageSizeFactor() const { return Index(imageSizeFactor); }

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.light0.lightRadius
  void PySetLightRadius(const float& lightRadiusInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.light0.lightRadius
  float PyGetLightRadius() const ;

  //! AUTO: Set function (needed in pybind) for: if lightRadiusVariations>1, this defines the number of positions that are used to compute the effect of distributed lights (larger is slower but better quality); range=1..256; avoid squares of integers; good values: 1 (hard shadow boundaries), 6, 13, 20, 31, 72, 130, 240; for lower values, use shadowSmoothingSteps=2..8
  void PySetLightRadiusVariations(const Index& lightRadiusVariationsInit) { lightRadiusVariations = EXUstd::GetSafelyPInt(lightRadiusVariationsInit,"lightRadiusVariations"); }
  //! AUTO: Read (Copy) access to: if lightRadiusVariations>1, this defines the number of positions that are used to compute the effect of distributed lights (larger is slower but better quality); range=1..256; avoid squares of integers; good values: 1 (hard shadow boundaries), 6, 13, 20, 31, 72, 130, 240; for lower values, use shadowSmoothingSteps=2..8
  Index PyGetLightRadiusVariations() const { return Index(lightRadiusVariations); }

  //! AUTO: Set function (needed in pybind) for: Maximum number of reflections computed for one ray (note that for each transparent face passed, the reflection depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  void PySetMaxReflectionDepth(const Index& maxReflectionDepthInit) { maxReflectionDepth = EXUstd::GetSafelyUInt(maxReflectionDepthInit,"maxReflectionDepth"); }
  //! AUTO: Read (Copy) access to: Maximum number of reflections computed for one ray (note that for each transparent face passed, the reflection depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  Index PyGetMaxReflectionDepth() const { return Index(maxReflectionDepth); }

  //! AUTO: Set function (needed in pybind) for: Maximum number of transparent faces that can be passed (note that for each reflection, the transparency depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  void PySetMaxTransparencyDepth(const Index& maxTransparencyDepthInit) { maxTransparencyDepth = EXUstd::GetSafelyUInt(maxTransparencyDepthInit,"maxTransparencyDepth"); }
  //! AUTO: Read (Copy) access to: Maximum number of transparent faces that can be passed (note that for each reflection, the transparency depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
  Index PyGetMaxTransparencyDepth() const { return Index(maxTransparencyDepth); }

  //! AUTO: Set function (needed in pybind) for: Multi-sampling used for rendering of faces, lines and text; increases image quality along edges (lines, etc.) but INCREASES rendering costs dramatically (multiSampling=3 => 3x3=9 times slower); also used for shadow if shadowScalingFactor=0; values only accepted in range [1..4]
  void PySetMultiSampling(const Index& multiSamplingInit) { multiSampling = EXUstd::GetSafelyPInt(multiSamplingInit,"multiSampling"); }
  //! AUTO: Read (Copy) access to: Multi-sampling used for rendering of faces, lines and text; increases image quality along edges (lines, etc.) but INCREASES rendering costs dramatically (multiSampling=3 => 3x3=9 times slower); also used for shadow if shadowScalingFactor=0; values only accepted in range [1..4]
  Index PyGetMultiSampling() const { return Index(multiSampling); }

  //! AUTO: Set function (needed in pybind) for: Number of CPU-threads (max: 256) used for software rendering (should be approx. the number of available threads)
  void PySetNumberOfThreads(const Index& numberOfThreadsInit) { numberOfThreads = EXUstd::GetSafelyPInt(numberOfThreadsInit,"numberOfThreads"); }
  //! AUTO: Read (Copy) access to: Number of CPU-threads (max: 256) used for software rendering (should be approx. the number of available threads)
  Index PyGetNumberOfThreads() const { return Index(numberOfThreads); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  bool depthSorting;                              //!< AUTO: True (slower): sort triangles by Z-depth to remove transparency artifacts: only works if triangles do not intersect or come close (you may like to refine triangle meshes); False: no depth-sort (faster)
  bool enableLighting;                            //!< AUTO: generally enable lighting (otherwise, colors of objects are used); OpenGL: glEnable(GL_LIGHTING)
  Float4 faceNormalsColor;                        //!< AUTO: global RGBA color for face normals
  Float3 initialCenterPoint;                      //!< AUTO: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True; only used in case that modelCentricView=True
  float initialMaxSceneSize;                      //!< AUTO: initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  StdArray33F initialModelRotation;               //!< AUTO: initial model rotation matrix for OpenGl; in python use e.g.: initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]; only used in case that modelCentricView=True
  float initialZoom;                              //!< AUTO: initial zoom of scene; overwritten/ignored if autoFitScene = True
  bool lightModelLocalViewer;                     //!< AUTO: True: the camera origin is used to compute shininess effects (more realistic); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,...)
  bool lightModelTwoSide;                         //!< AUTO: enlighten also backside of object; may cause problems on some graphics cards and lead to slower performance; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,...)
  bool lineSmooth;                                //!< AUTO: draw lines smooth
  float polygonOffset;                            //!< AUTO: general polygon offset for polygons, except for shadows; use this parameter to draw polygons behind lines to reduce artifacts for very large or small models
  bool shadeModelSmooth;                          //!< AUTO: True: turn on smoothing for shaders, which uses vertex normals to smooth surfaces
  float shadowPolygonOffset;                      //!< AUTO: some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
  bool showBoundingBox;                           //!< AUTO: show scene bounding box (red), as available in renderState.boundingBox; NOTE that the bounding box is only updated with ZoomAll or at startup; this is a debug flag and it may show reasongs for strange ZoomAll behavior, as ZoomAll should zoom to the bounding box; does only work for perspective=0
  bool textLineSmooth;                            //!< AUTO: draw lines for representation of text smooth
  float textLineWidth;                            //!< AUTO: width of lines used for representation of text
  Float4 vertexNormalsColor;                      //!< AUTO: global RGBA color for vertex normals

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsOpenGLAdvanced()
  {
    backlink=nullptr;
    clippingPlaneColor = Float4({0.7f,0.5f,0.5f,0.f});
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
  //! AUTO: Set function (needed in pybind) for: RGBA color for clipping plane; if alpha-channel is 0, the cutting plane is not drawn; if alpha-channel is 1, the clippingPlaneColor is used; if alpha-channel is 2, the color of the object interior is used as clipping plane color (which may look strange in case of object-in-object); see also view.camera for clipping plane options
  void PySetClippingPlaneColor(const std::array<float,4>& clippingPlaneColorInit) { clippingPlaneColor = clippingPlaneColorInit; }
  //! AUTO: Read (Copy) access to: RGBA color for clipping plane; if alpha-channel is 0, the cutting plane is not drawn; if alpha-channel is 1, the clippingPlaneColor is used; if alpha-channel is 2, the color of the object interior is used as clipping plane color (which may look strange in case of object-in-object); see also view.camera for clipping plane options
  std::array<float,4> PyGetClippingPlaneColor() const { return std::array<float,4>(clippingPlaneColor); }

  //! AUTO: Set function (needed in pybind) for: global RGBA color for face normals
  void PySetFaceNormalsColor(const std::array<float,4>& faceNormalsColorInit) { faceNormalsColor = faceNormalsColorInit; }
  //! AUTO: Read (Copy) access to: global RGBA color for face normals
  std::array<float,4> PyGetFaceNormalsColor() const { return std::array<float,4>(faceNormalsColor); }

  //! AUTO: Set function (needed in pybind) for: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True; only used in case that modelCentricView=True
  void PySetInitialCenterPoint(const std::array<float,3>& initialCenterPointInit) { initialCenterPoint = initialCenterPointInit; }
  //! AUTO: Read (Copy) access to: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True; only used in case that modelCentricView=True
  std::array<float,3> PyGetInitialCenterPoint() const { return std::array<float,3>(initialCenterPoint); }

  //! AUTO: Set function (needed in pybind) for: initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  void PySetInitialMaxSceneSize(const float& initialMaxSceneSizeInit) { initialMaxSceneSize = EXUstd::GetSafelyPFloat(initialMaxSceneSizeInit,"initialMaxSceneSize"); }
  //! AUTO: Read (Copy) access to: initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
  float PyGetInitialMaxSceneSize() const { return float(initialMaxSceneSize); }

  //! AUTO: Set function (needed in pybind) for: initial zoom of scene; overwritten/ignored if autoFitScene = True
  void PySetInitialZoom(const float& initialZoomInit) { initialZoom = EXUstd::GetSafelyUFloat(initialZoomInit,"initialZoom"); }
  //! AUTO: Read (Copy) access to: initial zoom of scene; overwritten/ignored if autoFitScene = True
  float PyGetInitialZoom() const { return float(initialZoom); }

  //! AUTO: Set function (needed in pybind) for: some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
  void PySetShadowPolygonOffset(const float& shadowPolygonOffsetInit) { shadowPolygonOffset = EXUstd::GetSafelyPFloat(shadowPolygonOffsetInit,"shadowPolygonOffset"); }
  //! AUTO: Read (Copy) access to: some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
  float PyGetShadowPolygonOffset() const { return float(shadowPolygonOffset); }

  //! AUTO: Set function (needed in pybind) for: width of lines used for representation of text
  void PySetTextLineWidth(const float& textLineWidthInit) { textLineWidth = EXUstd::GetSafelyUFloat(textLineWidthInit,"textLineWidth"); }
  //! AUTO: Read (Copy) access to: width of lines used for representation of text
  float PyGetTextLineWidth() const { return float(textLineWidth); }

  //! AUTO: Set function (needed in pybind) for: global RGBA color for vertex normals
  void PySetVertexNormalsColor(const std::array<float,4>& vertexNormalsColorInit) { vertexNormalsColor = vertexNormalsColorInit; }
  //! AUTO: Read (Copy) access to: global RGBA color for vertex normals
  std::array<float,4> PyGetVertexNormalsColor() const { return std::array<float,4>(vertexNormalsColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsOpenGLAdvanced" << ":\n";
    os << "  clippingPlaneColor = " << clippingPlaneColor << "\n";
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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsLight // AUTO: 
{
public: // AUTO: 
  float constantAttenuation;                      //!< AUTO: constant attenuation coefficient of GL_LIGHT[0,1,2,3], this is a constant factor that attenuates the light source; attenuation factor = 1/(kc +kl*d + kq*d*d); (kc,kl,kq)=(1,0,0) means no attenuation; only used for lights, where last component of light position is 1
  float diffuse;                                  //!< AUTO: diffuse value of GL_LIGHT[0,1,2,3]
  bool enable;                                    //!< AUTO: turn on/off light
  float lightRadius;                              //!< AUTO: only used by raytracers: radius of light used to compute smooth shadows (approximated by raytracer.lightRadiusVariations); if lightRadiusVariations>1, this value defines the radius of the light, converting point lights into distributed lights (slower)
  float linearAttenuation;                        //!< AUTO: linear attenuation coefficient of GL_LIGHT[0,1,2,3], this is a linear factor for attenuation of the light source with distance
  Float4 position;                                //!< AUTO: 4D position vector of GL_LIGHT[0,1,2,3]; 4th value should be 0 for directional lights that are (almost) infinitely far away, like the sun, but 1 for position-based lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position to be located at a reasonable location; the openGL renderer uses shadow volumes and approximates directional lights by enlarging the direction to 200 times maxSceneSize, while the raytracer uses the correct direction; see opengl manuals
  float quadraticAttenuation;                     //!< AUTO: quadratic attenuation coefficient of GL_LIGHT[0,1,2,3], this is a quadratic factor for attenuation of the light source with distance
  float shadow;                                   //!< AUTO: in OpenGL renderer, the shadow parameter \f$\in [0 ... 1]\f$ prescribes amount of shadow of light [0,1,2,3] that is added to the scene, using light position (or only direction), accumulating for each light; if this parameter is different from 0, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows; for raytracer, shadow is included by a physics-based model for each light if shadow>0, accumulating effects of each light source
  float specular;                                 //!< AUTO: specular value of GL_LIGHT[0,1,2,3]
  bool useCameraFrame;                            //!< AUTO: set False to set light positions and directions relative to model frame; True: lights are in camera frame, not following the visual transformations; this was True up to Exudyn 1.9.174

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsLight()
  {
    backlink=nullptr;
    constantAttenuation = 1.0f;
    diffuse = 0.5f;
    enable = true;
    lightRadius = 0.1f;
    linearAttenuation = 0.0f;
    position = Float4({2.f,2.f,10.f,0.f});
    quadraticAttenuation = 0.0f;
    shadow = 0.f;
    specular = 0.5f;
    useCameraFrame = false;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: 4D position vector of GL_LIGHT[0,1,2,3]; 4th value should be 0 for directional lights that are (almost) infinitely far away, like the sun, but 1 for position-based lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position to be located at a reasonable location; the openGL renderer uses shadow volumes and approximates directional lights by enlarging the direction to 200 times maxSceneSize, while the raytracer uses the correct direction; see opengl manuals
  void PySetPosition(const std::array<float,4>& positionInit) { position = positionInit; }
  //! AUTO: Read (Copy) access to: 4D position vector of GL_LIGHT[0,1,2,3]; 4th value should be 0 for directional lights that are (almost) infinitely far away, like the sun, but 1 for position-based lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position to be located at a reasonable location; the openGL renderer uses shadow volumes and approximates directional lights by enlarging the direction to 200 times maxSceneSize, while the raytracer uses the correct direction; see opengl manuals
  std::array<float,4> PyGetPosition() const { return std::array<float,4>(position); }

  //! AUTO: Set function (needed in pybind) for: in OpenGL renderer, the shadow parameter \f$\in [0 ... 1]\f$ prescribes amount of shadow of light [0,1,2,3] that is added to the scene, using light position (or only direction), accumulating for each light; if this parameter is different from 0, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows; for raytracer, shadow is included by a physics-based model for each light if shadow>0, accumulating effects of each light source
  void PySetShadow(const float& shadowInit) { shadow = EXUstd::GetSafelyUFloat(shadowInit,"shadow"); }
  //! AUTO: Read (Copy) access to: in OpenGL renderer, the shadow parameter \f$\in [0 ... 1]\f$ prescribes amount of shadow of light [0,1,2,3] that is added to the scene, using light position (or only direction), accumulating for each light; if this parameter is different from 0, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows; for raytracer, shadow is included by a physics-based model for each light if shadow>0, accumulating effects of each light source
  float PyGetShadow() const { return float(shadow); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  float drawNormalsLength;                        //!< AUTO: length of normals; used for debugging
  bool drawVertexNormals;                         //!< AUTO: draws vertex normals; used for debugging
  Float4 faceEdgesColor;                          //!< AUTO: global RGBA color for face edges
  float faceTransparencyGlobal;                   //!< AUTO: in case that facesTransparent=True this represents the max alpha-transparency
  Float4 lightModelAmbient;                       //!< AUTO: global ambient light (needed for faces that are close to orthogonal to light or faces in shadow region); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a]); also used by raytracer
  float lineWidth;                                //!< AUTO: width of lines used for representation of lines, circles, points, etc.
  float materialShininess;                        //!< AUTO: shininess of material
  Float4 materialSpecular;                        //!< AUTO: RGBA specular color of material
  Index multiSampling;                            //!< AUTO: NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 3, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  float zMaxSceneFactor;                          //!< AUTO: factor multiplied with maxSceneSize to avoid clipping of modelview; larger values reduce clipping of near or far objects, but may lead to artifacts (so-called Z-fighting)
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
    dummy = 0.0f;
    faceEdgesColor = Float4({0.2f,0.2f,0.2f,1.f});
    faceTransparencyGlobal = 0.4f;
    lightModelAmbient = Float4({0.4f,0.4f,0.4f,1.f});
    lineWidth = 1.f;
    materialShininess = 32.f;
    materialSpecular = Float4({0.6f,0.6f,0.6f,1.f});
    multiSampling = 1;
    zMaxSceneFactor = 2.f;
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

  //! AUTO: Set function (needed in pybind) for: length of normals; used for debugging
  void PySetDrawNormalsLength(const float& drawNormalsLengthInit) { drawNormalsLength = EXUstd::GetSafelyPFloat(drawNormalsLengthInit,"drawNormalsLength"); }
  //! AUTO: Read (Copy) access to: length of normals; used for debugging
  float PyGetDrawNormalsLength() const { return float(drawNormalsLength); }

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

  //! AUTO: Set function (needed in pybind) for: global RGBA color for face edges
  void PySetFaceEdgesColor(const std::array<float,4>& faceEdgesColorInit) { faceEdgesColor = faceEdgesColorInit; }
  //! AUTO: Read (Copy) access to: global RGBA color for face edges
  std::array<float,4> PyGetFaceEdgesColor() const { return std::array<float,4>(faceEdgesColor); }

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use view0.scene.facesTransparent
  void PySetFacesTransparent(const bool& facesTransparentInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use view0.scene.facesTransparent
  bool PyGetFacesTransparent() const ;

  //! AUTO: Set function (needed in pybind) for: in case that facesTransparent=True this represents the max alpha-transparency
  void PySetFaceTransparencyGlobal(const float& faceTransparencyGlobalInit) { faceTransparencyGlobal = EXUstd::GetSafelyUFloat(faceTransparencyGlobalInit,"faceTransparencyGlobal"); }
  //! AUTO: Read (Copy) access to: in case that facesTransparent=True this represents the max alpha-transparency
  float PyGetFaceTransparencyGlobal() const { return float(faceTransparencyGlobal); }

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

  //! AUTO: Set function (needed in pybind) for: global ambient light (needed for faces that are close to orthogonal to light or faces in shadow region); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a]); also used by raytracer
  void PySetLightModelAmbient(const std::array<float,4>& lightModelAmbientInit) { lightModelAmbient = lightModelAmbientInit; }
  //! AUTO: Read (Copy) access to: global ambient light (needed for faces that are close to orthogonal to light or faces in shadow region); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a]); also used by raytracer
  std::array<float,4> PyGetLightModelAmbient() const { return std::array<float,4>(lightModelAmbient); }

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

  //! AUTO: Set function (needed in pybind) for: width of lines used for representation of lines, circles, points, etc.
  void PySetLineWidth(const float& lineWidthInit) { lineWidth = EXUstd::GetSafelyUFloat(lineWidthInit,"lineWidth"); }
  //! AUTO: Read (Copy) access to: width of lines used for representation of lines, circles, points, etc.
  float PyGetLineWidth() const { return float(lineWidth); }

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use openGL.materialSpecular
  void PySetMaterialAmbientAndDiffuse(const std::array<float,4>& materialSpecularInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use openGL.materialSpecular
  std::array<float,4> PyGetMaterialAmbientAndDiffuse() const ;

  //! AUTO: Set function (needed in pybind) for: RGBA specular color of material
  void PySetMaterialSpecular(const std::array<float,4>& materialSpecularInit) { materialSpecular = materialSpecularInit; }
  //! AUTO: Read (Copy) access to: RGBA specular color of material
  std::array<float,4> PyGetMaterialSpecular() const { return std::array<float,4>(materialSpecular); }

  //! AUTO: Set function (needed in pybind) for: NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 3, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  void PySetMultiSampling(const Index& multiSamplingInit) { multiSampling = EXUstd::GetSafelyPInt(multiSamplingInit,"multiSampling"); }
  //! AUTO: Read (Copy) access to: NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 3, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
  Index PyGetMultiSampling() const { return Index(multiSampling); }

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

  //! AUTO: Set function (needed in pybind) for: factor multiplied with maxSceneSize to avoid clipping of modelview; larger values reduce clipping of near or far objects, but may lead to artifacts (so-called Z-fighting)
  void PySetZMaxSceneFactor(const float& zMaxSceneFactorInit) { zMaxSceneFactor = EXUstd::GetSafelyPFloat(zMaxSceneFactorInit,"zMaxSceneFactor"); }
  //! AUTO: Read (Copy) access to: factor multiplied with maxSceneSize to avoid clipping of modelview; larger values reduce clipping of near or far objects, but may lead to artifacts (so-called Z-fighting)
  float PyGetZMaxSceneFactor() const { return float(zMaxSceneFactor); }

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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
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

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsExportImages()
  {
    backlink=nullptr;
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
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

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
* @date         AUTO: 2026-04-03 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
class VSettingsOpenVR // AUTO: 
{
public: // AUTO: 
  std::string actionManifestFileName;             //!< AUTO: This string must contain a string representing a valid absolute path to a vr_actions.json manifest, which describes all HMD, tracker, etc. devices as given by openVR
  bool enable;                                    //!< AUTO: True: openVR enabled (if compiled with according flag and installed openVR)
  Index logLevel;                                 //!< AUTO: integer value setting log level of openVR: -1 (no output), 0 (error), 1 (warning), 2 (info), 3 (debug); increase log level to get more output
  bool showCompanionWindow;                       //!< AUTO: True: openVR will show companion window containing left and right eye view

private: // AUTO: 
  VisualizationSettings* backlink; //!< AUTO: backlink for global access of structure


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsOpenVR()
  {
    backlink=nullptr;
    actionManifestFileName = "C:/openVRactionsManifest.json";
    enable = false;
    logLevel = 1;
    showCompanionWindow = true;
  };
  void Init(VisualizationSettings* backlinkInit) //!< AUTO: called from parent structure
  {
    backlink = backlinkInit;
  }

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
* @class        VSettingsInteractiveAdvanced
* @brief        Advanced settings for interactive.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2026-04-03 (last modfied)
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
  //! AUTO: Set function (needed in pybind) for: RGBA color for highlighted item; 4th value is alpha-transparency
  void PySetHighlightColor(const std::array<float,4>& highlightColorInit) { highlightColor = highlightColorInit; }
  //! AUTO: Read (Copy) access to: RGBA color for highlighted item; 4th value is alpha-transparency
  std::array<float,4> PyGetHighlightColor() const { return std::array<float,4>(highlightColor); }

  //! AUTO: Set function (needed in pybind) for: RGBA color for other items (which are not highlighted); 4th value is alpha-transparency
  void PySetHighlightOtherColor(const std::array<float,4>& highlightOtherColorInit) { highlightOtherColor = highlightOtherColorInit; }
  //! AUTO: Read (Copy) access to: RGBA color for other items (which are not highlighted); 4th value is alpha-transparency
  std::array<float,4> PyGetHighlightOtherColor() const { return std::array<float,4>(highlightOtherColor); }

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
* @date         AUTO: 2026-04-03 (last modfied)
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
  VSettingsOpenVR openVR;                         //!< AUTO: openVR visualization settings
  bool autoRotateModelView;                       //!< AUTO: True: rotate model view with autorotation
  Float3 autoRotationVelocity;                    //!< AUTO: Angular velocity vector for auto-rotation of scene (only visualization view is rotated, not the model itself!)
  Index highlightItemIndex;                       //!< AUTO: index of item that shall be highlighted (e.g., to find item which cauess problems); if set -1, no item is highlighted
  ItemType highlightItemType;                     //!< AUTO: item type (Node, Object, ...) that shall be highlighted (e.g., to find item which cauess problems)
  Index highlightMbsNumber;                       //!< AUTO: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  bool ignoreKeys;                                //!< AUTO: True: ignore keyboard input except escape and 'F2' keys; used for interactive mode, e.g., to perform kinematic analysis; This flag can be switched with key 'F2'; if ignoreKeys=True, then keyPressUserFunction can be used!
  std::function<bool(int, int, int)> keyPressUserFunction;//!< AUTO: add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example: \tabnewline def f(key, action, mods):\tabnewline \phantom{XXX} print('key=',key);\tabnewline use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
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
    openVR.Init(backlinkInit);
  }

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: Angular velocity vector for auto-rotation of scene (only visualization view is rotated, not the model itself!)
  void PySetAutoRotationVelocity(const std::array<float,3>& autoRotationVelocityInit) { autoRotationVelocity = autoRotationVelocityInit; }
  //! AUTO: Read (Copy) access to: Angular velocity vector for auto-rotation of scene (only visualization view is rotated, not the model itself!)
  std::array<float,3> PyGetAutoRotationVelocity() const { return std::array<float,3>(autoRotationVelocity); }

  //! AUTO: Set function (needed in pybind) for: DEPRECATED; Instead use interactive.advanced.highlightColor
  void PySetHighlightColor(const std::array<float,4>& highlightColorInit) ;
  //! AUTO: Read (Copy) access to: DEPRECATED; Instead use interactive.advanced.highlightColor
  std::array<float,4> PyGetHighlightColor() const ;

  //! AUTO: Set function (needed in pybind) for: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  void PySetHighlightMbsNumber(const Index& highlightMbsNumberInit) { highlightMbsNumber = EXUstd::GetSafelyUInt(highlightMbsNumberInit,"highlightMbsNumber"); }
  //! AUTO: Read (Copy) access to: index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
  Index PyGetHighlightMbsNumber() const { return Index(highlightMbsNumber); }

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

  //! AUTO: Set function (needed in pybind) for: add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example: \tabnewline def f(key, action, mods):\tabnewline \phantom{XXX} print('key=',key);\tabnewline use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
  void PySetKeyPressUserFunction(const std::function<bool(int, int, int)>& keyPressUserFunctionInit) { keyPressUserFunction= (const std::function<bool(int, int, int)>&)keyPressUserFunctionInit; }
  //! AUTO: Read (Copy) access to: add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example: \tabnewline def f(key, action, mods):\tabnewline \phantom{XXX} print('key=',key);\tabnewline use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
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
    os << "  openVR = " << openVR << "\n";
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
* @date         AUTO: 2026-04-03 (last modfied)
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
  };
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
    PyWarning("VisualizationSettings parameter general.drawCoordinateSystem is deprecated! use view0.scene.drawCoordinateSystem instead!");
    backlink->view0.scene.drawCoordinateSystem= (const Index&)drawCoordinateSystemInit; 
    }
inline Index VSettingsGeneral::PyGetDrawCoordinateSystem() const { 
    PyWarning("VisualizationSettings parameter general.drawCoordinateSystem is deprecated! use view0.scene.drawCoordinateSystem instead!");
    return Index(backlink->view0.scene.drawCoordinateSystem); 
    }

inline void VSettingsGeneral::PySetDrawWorldBasis(const bool& drawWorldBasisInit) { 
    PyWarning("VisualizationSettings parameter general.drawWorldBasis is deprecated! use view0.scene.drawWorldBasis instead!");
    backlink->view0.scene.drawWorldBasis= (const bool&)drawWorldBasisInit; 
    }
inline bool VSettingsGeneral::PyGetDrawWorldBasis() const { 
    PyWarning("VisualizationSettings parameter general.drawWorldBasis is deprecated! use view0.scene.drawWorldBasis instead!");
    return bool(backlink->view0.scene.drawWorldBasis); 
    }

inline void VSettingsGeneral::PySetShowComputationInfo(const bool& showComputationInfoInit) { 
    PyWarning("VisualizationSettings parameter general.showComputationInfo is deprecated! use view0.window.showComputationInfo instead!");
    backlink->view0.window.showComputationInfo= (const bool&)showComputationInfoInit; 
    }
inline bool VSettingsGeneral::PyGetShowComputationInfo() const { 
    PyWarning("VisualizationSettings parameter general.showComputationInfo is deprecated! use view0.window.showComputationInfo instead!");
    return bool(backlink->view0.window.showComputationInfo); 
    }

inline void VSettingsGeneral::PySetTextSize(const float& globalFontSizeInit) { 
    PyWarning("VisualizationSettings parameter general.textSize is deprecated! use view0.window.globalFontSize instead!");
    backlink->view0.window.globalFontSize= (const float&)globalFontSizeInit; 
    }
inline float VSettingsGeneral::PyGetTextSize() const { 
    PyWarning("VisualizationSettings parameter general.textSize is deprecated! use view0.window.globalFontSize instead!");
    return float(backlink->view0.window.globalFontSize); 
    }

inline void VSettingsGeneral::PySetWorldBasisSize(const float& worldBasisSizeInit) { 
    PyWarning("VisualizationSettings parameter general.worldBasisSize is deprecated! use view0.scene.worldBasisSize instead!");
    backlink->view0.scene.worldBasisSize= (const float&)worldBasisSizeInit; 
    }
inline float VSettingsGeneral::PyGetWorldBasisSize() const { 
    PyWarning("VisualizationSettings parameter general.worldBasisSize is deprecated! use view0.scene.worldBasisSize instead!");
    return float(backlink->view0.scene.worldBasisSize); 
    }

inline void VSettingsContour::PySetColorBarPrecision(const Index& colorBarPrecisionInit) { 
    PyWarning("VisualizationSettings parameter contour.colorBarPrecision is deprecated! use contour.advanced.colorBarPrecision instead!");
    backlink->contour.advanced.colorBarPrecision= (const Index&)colorBarPrecisionInit; 
    }
inline Index VSettingsContour::PyGetColorBarPrecision() const { 
    PyWarning("VisualizationSettings parameter contour.colorBarPrecision is deprecated! use contour.advanced.colorBarPrecision instead!");
    return Index(backlink->contour.advanced.colorBarPrecision); 
    }

inline void VSettingsContour::PySetColorBarTiling(const Index& colorBarTilingInit) { 
    PyWarning("VisualizationSettings parameter contour.colorBarTiling is deprecated! use contour.advanced.colorBarTiling instead!");
    backlink->contour.advanced.colorBarTiling= (const Index&)colorBarTilingInit; 
    }
inline Index VSettingsContour::PyGetColorBarTiling() const { 
    PyWarning("VisualizationSettings parameter contour.colorBarTiling is deprecated! use contour.advanced.colorBarTiling instead!");
    return Index(backlink->contour.advanced.colorBarTiling); 
    }

inline void VSettingsContour::PySetShowColorBar(const bool& showColorBarInit) { 
    PyWarning("VisualizationSettings parameter contour.showColorBar is deprecated! use contour.advanced.showColorBar instead!");
    backlink->contour.advanced.showColorBar= (const bool&)showColorBarInit; 
    }
inline bool VSettingsContour::PyGetShowColorBar() const { 
    PyWarning("VisualizationSettings parameter contour.showColorBar is deprecated! use contour.advanced.showColorBar instead!");
    return bool(backlink->contour.advanced.showColorBar); 
    }

inline void VSettingsWindowDeprecated::PySetAlwaysOnTop(const bool& alwaysOnTopInit) { 
    PyWarning("VisualizationSettings parameter window.alwaysOnTop is deprecated! use view0.window.alwaysOnTop instead!");
    backlink->view0.window.alwaysOnTop= (const bool&)alwaysOnTopInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetAlwaysOnTop() const { 
    PyWarning("VisualizationSettings parameter window.alwaysOnTop is deprecated! use view0.window.alwaysOnTop instead!");
    return bool(backlink->view0.window.alwaysOnTop); 
    }

inline void VSettingsWindowDeprecated::PySetIgnoreKeys(const bool& ignoreKeysInit) { 
    PyWarning("VisualizationSettings parameter window.ignoreKeys is deprecated! use interactive.ignoreKeys instead!");
    backlink->interactive.ignoreKeys= (const bool&)ignoreKeysInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetIgnoreKeys() const { 
    PyWarning("VisualizationSettings parameter window.ignoreKeys is deprecated! use interactive.ignoreKeys instead!");
    return bool(backlink->interactive.ignoreKeys); 
    }

inline void VSettingsWindowDeprecated::PySetKeyPressUserFunction(const std::function<bool(int, int, int)>& keyPressUserFunctionInit) { 
    PyWarning("VisualizationSettings parameter window.keyPressUserFunction is deprecated! use interactive.keyPressUserFunction instead!");
    backlink->interactive.keyPressUserFunction= (const std::function<bool(int, int, int)>&)keyPressUserFunctionInit; 
    }
inline std::function<bool(int, int, int)> VSettingsWindowDeprecated::PyGetKeyPressUserFunction() const { 
    PyWarning("VisualizationSettings parameter window.keyPressUserFunction is deprecated! use interactive.keyPressUserFunction instead!");
    return std::function<bool(int, int, int)>(backlink->interactive.keyPressUserFunction); 
    }

inline void VSettingsWindowDeprecated::PySetLimitWindowToScreenSize(const bool& limitWindowToScreenSizeInit) { 
    PyWarning("VisualizationSettings parameter window.limitWindowToScreenSize is deprecated! use general.limitWindowToScreenSize instead!");
    backlink->general.limitWindowToScreenSize= (const bool&)limitWindowToScreenSizeInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetLimitWindowToScreenSize() const { 
    PyWarning("VisualizationSettings parameter window.limitWindowToScreenSize is deprecated! use general.limitWindowToScreenSize instead!");
    return bool(backlink->general.limitWindowToScreenSize); 
    }

inline void VSettingsWindowDeprecated::PySetMaximize(const bool& maximizeInit) { 
    PyWarning("VisualizationSettings parameter window.maximize is deprecated! use view0.window.maximize instead!");
    backlink->view0.window.maximize= (const bool&)maximizeInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetMaximize() const { 
    PyWarning("VisualizationSettings parameter window.maximize is deprecated! use view0.window.maximize instead!");
    return bool(backlink->view0.window.maximize); 
    }

inline void VSettingsWindowDeprecated::PySetReallyQuitTimeLimit(const Real& reallyQuitTimeLimitInit) { 
    PyWarning("VisualizationSettings parameter window.reallyQuitTimeLimit is deprecated! use general.reallyQuitTimeLimit instead!");
    backlink->general.reallyQuitTimeLimit= (const Real&)reallyQuitTimeLimitInit; 
    }
inline Real VSettingsWindowDeprecated::PyGetReallyQuitTimeLimit() const { 
    PyWarning("VisualizationSettings parameter window.reallyQuitTimeLimit is deprecated! use general.reallyQuitTimeLimit instead!");
    return Real(backlink->general.reallyQuitTimeLimit); 
    }

inline void VSettingsWindowDeprecated::PySetRenderWindowSize(const std::array<Index,2>& renderWindowSizeInit) { 
    PyWarning("VisualizationSettings parameter window.renderWindowSize is deprecated! use view0.window.renderWindowSize instead!");
    backlink->view0.window.renderWindowSize= (const Index2&)renderWindowSizeInit; 
    }
inline std::array<Index,2> VSettingsWindowDeprecated::PyGetRenderWindowSize() const { 
    PyWarning("VisualizationSettings parameter window.renderWindowSize is deprecated! use view0.window.renderWindowSize instead!");
    return std::array<Index,2>(backlink->view0.window.renderWindowSize); 
    }

inline void VSettingsWindowDeprecated::PySetShowMouseCoordinates(const bool& showMouseCoordinatesInit) { 
    PyWarning("VisualizationSettings parameter window.showMouseCoordinates is deprecated! use view0.window.showMouseCoordinates instead!");
    backlink->view0.window.showMouseCoordinates= (const bool&)showMouseCoordinatesInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetShowMouseCoordinates() const { 
    PyWarning("VisualizationSettings parameter window.showMouseCoordinates is deprecated! use view0.window.showMouseCoordinates instead!");
    return bool(backlink->view0.window.showMouseCoordinates); 
    }

inline void VSettingsWindowDeprecated::PySetShowRenderStateInfo(const bool& showRenderStateInfoInit) { 
    PyWarning("VisualizationSettings parameter window.showRenderStateInfo is deprecated! use view0.window.showRenderStateInfo instead!");
    backlink->view0.window.showRenderStateInfo= (const bool&)showRenderStateInfoInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetShowRenderStateInfo() const { 
    PyWarning("VisualizationSettings parameter window.showRenderStateInfo is deprecated! use view0.window.showRenderStateInfo instead!");
    return bool(backlink->view0.window.showRenderStateInfo); 
    }

inline void VSettingsWindowDeprecated::PySetShowWindow(const bool& showWindowInit) { 
    PyWarning("VisualizationSettings parameter window.showWindow is deprecated! use view0.window.showWindow instead!");
    backlink->view0.window.showWindow= (const bool&)showWindowInit; 
    }
inline bool VSettingsWindowDeprecated::PyGetShowWindow() const { 
    PyWarning("VisualizationSettings parameter window.showWindow is deprecated! use view0.window.showWindow instead!");
    return bool(backlink->view0.window.showWindow); 
    }

inline void VSettingsWindowDeprecated::PySetStartupTimeout(const Index& rendererStartupTimeoutInit) { 
    PyWarning("VisualizationSettings parameter window.startupTimeout is deprecated! use general.rendererStartupTimeout instead!");
    backlink->general.rendererStartupTimeout= (const Index&)rendererStartupTimeoutInit; 
    }
inline Index VSettingsWindowDeprecated::PyGetStartupTimeout() const { 
    PyWarning("VisualizationSettings parameter window.startupTimeout is deprecated! use general.rendererStartupTimeout instead!");
    return Index(backlink->general.rendererStartupTimeout); 
    }

inline void VSettingsRaytracer::PySetAmbientLightColor(const std::array<float,4>& lightModelAmbientInit) { 
    PyWarning("VisualizationSettings parameter raytracer.ambientLightColor is deprecated! use openGL.lightModelAmbient instead!");
    backlink->openGL.lightModelAmbient= (const Float4&)lightModelAmbientInit; 
    }
inline std::array<float,4> VSettingsRaytracer::PyGetAmbientLightColor() const { 
    PyWarning("VisualizationSettings parameter raytracer.ambientLightColor is deprecated! use openGL.lightModelAmbient instead!");
    return std::array<float,4>(backlink->openGL.lightModelAmbient); 
    }

inline void VSettingsRaytracer::PySetBackgroundColorReflections(const std::array<float,4>& backgroundColorReflectionsInit) { 
    PyWarning("VisualizationSettings parameter raytracer.backgroundColorReflections is deprecated! use raytracer.advanced.backgroundColorReflections instead!");
    backlink->raytracer.advanced.backgroundColorReflections= (const Float4&)backgroundColorReflectionsInit; 
    }
inline std::array<float,4> VSettingsRaytracer::PyGetBackgroundColorReflections() const { 
    PyWarning("VisualizationSettings parameter raytracer.backgroundColorReflections is deprecated! use raytracer.advanced.backgroundColorReflections instead!");
    return std::array<float,4>(backlink->raytracer.advanced.backgroundColorReflections); 
    }

inline void VSettingsRaytracer::PySetEnable(const bool& useRaytracerInit) { 
    PyWarning("VisualizationSettings parameter raytracer.enable is deprecated! use view0.camera.useRaytracer instead!");
    backlink->view0.camera.useRaytracer= (const bool&)useRaytracerInit; 
    }
inline bool VSettingsRaytracer::PyGetEnable() const { 
    PyWarning("VisualizationSettings parameter raytracer.enable is deprecated! use view0.camera.useRaytracer instead!");
    return bool(backlink->view0.camera.useRaytracer); 
    }

inline void VSettingsRaytracer::PySetLightRadius(const float& lightRadiusInit) { 
    PyWarning("VisualizationSettings parameter raytracer.lightRadius is deprecated! use openGL.light0.lightRadius instead!");
    backlink->openGL.light0.lightRadius= (const float&)lightRadiusInit; 
    }
inline float VSettingsRaytracer::PyGetLightRadius() const { 
    PyWarning("VisualizationSettings parameter raytracer.lightRadius is deprecated! use openGL.light0.lightRadius instead!");
    return float(backlink->openGL.light0.lightRadius); 
    }

inline void VSettingsRaytracer::PySetSearchTreeFactor(const Index& searchTreeFactorInit) { 
    PyWarning("VisualizationSettings parameter raytracer.searchTreeFactor is deprecated! use raytracer.advanced.searchTreeFactor instead!");
    backlink->raytracer.advanced.searchTreeFactor= (const Index&)searchTreeFactorInit; 
    }
inline Index VSettingsRaytracer::PyGetSearchTreeFactor() const { 
    PyWarning("VisualizationSettings parameter raytracer.searchTreeFactor is deprecated! use raytracer.advanced.searchTreeFactor instead!");
    return Index(backlink->raytracer.advanced.searchTreeFactor); 
    }

inline void VSettingsRaytracer::PySetShadowScalingFactor(const Index& shadowScalingFactorInit) { 
    PyWarning("VisualizationSettings parameter raytracer.shadowScalingFactor is deprecated! use raytracer.advanced.shadowScalingFactor instead!");
    backlink->raytracer.advanced.shadowScalingFactor= (const Index&)shadowScalingFactorInit; 
    }
inline Index VSettingsRaytracer::PyGetShadowScalingFactor() const { 
    PyWarning("VisualizationSettings parameter raytracer.shadowScalingFactor is deprecated! use raytracer.advanced.shadowScalingFactor instead!");
    return Index(backlink->raytracer.advanced.shadowScalingFactor); 
    }

inline void VSettingsRaytracer::PySetShadowSmoothingSteps(const Index& shadowSmoothingStepsInit) { 
    PyWarning("VisualizationSettings parameter raytracer.shadowSmoothingSteps is deprecated! use raytracer.advanced.shadowSmoothingSteps instead!");
    backlink->raytracer.advanced.shadowSmoothingSteps= (const Index&)shadowSmoothingStepsInit; 
    }
inline Index VSettingsRaytracer::PyGetShadowSmoothingSteps() const { 
    PyWarning("VisualizationSettings parameter raytracer.shadowSmoothingSteps is deprecated! use raytracer.advanced.shadowSmoothingSteps instead!");
    return Index(backlink->raytracer.advanced.shadowSmoothingSteps); 
    }

inline void VSettingsRaytracer::PySetShowText(const bool& showTextInit) { 
    PyWarning("VisualizationSettings parameter raytracer.showText is deprecated! use raytracer.advanced.showText instead!");
    backlink->raytracer.advanced.showText= (const bool&)showTextInit; 
    }
inline bool VSettingsRaytracer::PyGetShowText() const { 
    PyWarning("VisualizationSettings parameter raytracer.showText is deprecated! use raytracer.advanced.showText instead!");
    return bool(backlink->raytracer.advanced.showText); 
    }

inline void VSettingsRaytracer::PySetTilesPerThread(const Index& tilesPerThreadInit) { 
    PyWarning("VisualizationSettings parameter raytracer.tilesPerThread is deprecated! use raytracer.advanced.tilesPerThread instead!");
    backlink->raytracer.advanced.tilesPerThread= (const Index&)tilesPerThreadInit; 
    }
inline Index VSettingsRaytracer::PyGetTilesPerThread() const { 
    PyWarning("VisualizationSettings parameter raytracer.tilesPerThread is deprecated! use raytracer.advanced.tilesPerThread instead!");
    return Index(backlink->raytracer.advanced.tilesPerThread); 
    }

inline void VSettingsRaytracer::PySetZBiasLines(const float& zBiasLinesInit) { 
    PyWarning("VisualizationSettings parameter raytracer.zBiasLines is deprecated! use raytracer.advanced.zBiasLines instead!");
    backlink->raytracer.advanced.zBiasLines= (const float&)zBiasLinesInit; 
    }
inline float VSettingsRaytracer::PyGetZBiasLines() const { 
    PyWarning("VisualizationSettings parameter raytracer.zBiasLines is deprecated! use raytracer.advanced.zBiasLines instead!");
    return float(backlink->raytracer.advanced.zBiasLines); 
    }

inline void VSettingsRaytracer::PySetZOffsetCamera(const float& dummyInit) { 
    PyWarning("VisualizationSettings parameter raytracer.zOffsetCamera is deprecated! use openGL.dummy instead!");
    backlink->openGL.dummy= (const float&)dummyInit; 
    }
inline float VSettingsRaytracer::PyGetZOffsetCamera() const { 
    PyWarning("VisualizationSettings parameter raytracer.zOffsetCamera is deprecated! use openGL.dummy instead!");
    return float(backlink->openGL.dummy); 
    }

inline void VSettingsOpenGL::PySetClippingPlaneColor(const std::array<float,4>& clippingPlaneColorInit) { 
    PyWarning("VisualizationSettings parameter openGL.clippingPlaneColor is deprecated! use openGL.advanced.clippingPlaneColor instead!");
    backlink->openGL.advanced.clippingPlaneColor= (const Float4&)clippingPlaneColorInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetClippingPlaneColor() const { 
    PyWarning("VisualizationSettings parameter openGL.clippingPlaneColor is deprecated! use openGL.advanced.clippingPlaneColor instead!");
    return std::array<float,4>(backlink->openGL.advanced.clippingPlaneColor); 
    }

inline void VSettingsOpenGL::PySetClippingPlaneDistance(const float& clippingPlaneDistanceInit) { 
    PyWarning("VisualizationSettings parameter openGL.clippingPlaneDistance is deprecated! use view0.camera.clippingPlaneDistance instead!");
    backlink->view0.camera.clippingPlaneDistance= (const float&)clippingPlaneDistanceInit; 
    }
inline float VSettingsOpenGL::PyGetClippingPlaneDistance() const { 
    PyWarning("VisualizationSettings parameter openGL.clippingPlaneDistance is deprecated! use view0.camera.clippingPlaneDistance instead!");
    return float(backlink->view0.camera.clippingPlaneDistance); 
    }

inline void VSettingsOpenGL::PySetClippingPlaneNormal(const std::array<float,3>& clippingPlaneNormalInit) { 
    PyWarning("VisualizationSettings parameter openGL.clippingPlaneNormal is deprecated! use view0.camera.clippingPlaneNormal instead!");
    backlink->view0.camera.clippingPlaneNormal= (const Float3&)clippingPlaneNormalInit; 
    }
inline std::array<float,3> VSettingsOpenGL::PyGetClippingPlaneNormal() const { 
    PyWarning("VisualizationSettings parameter openGL.clippingPlaneNormal is deprecated! use view0.camera.clippingPlaneNormal instead!");
    return std::array<float,3>(backlink->view0.camera.clippingPlaneNormal); 
    }

inline void VSettingsOpenGL::PySetDepthSorting(const bool& depthSortingInit) { 
    PyWarning("VisualizationSettings parameter openGL.depthSorting is deprecated! use openGL.advanced.depthSorting instead!");
    backlink->openGL.advanced.depthSorting= (const bool&)depthSortingInit; 
    }
inline bool VSettingsOpenGL::PyGetDepthSorting() const { 
    PyWarning("VisualizationSettings parameter openGL.depthSorting is deprecated! use openGL.advanced.depthSorting instead!");
    return bool(backlink->openGL.advanced.depthSorting); 
    }

inline void VSettingsOpenGL::PySetEnableLight0(const bool& enableInit) { 
    PyWarning("VisualizationSettings parameter openGL.enableLight0 is deprecated! use openGL.light0.enable instead!");
    backlink->openGL.light0.enable= (const bool&)enableInit; 
    }
inline bool VSettingsOpenGL::PyGetEnableLight0() const { 
    PyWarning("VisualizationSettings parameter openGL.enableLight0 is deprecated! use openGL.light0.enable instead!");
    return bool(backlink->openGL.light0.enable); 
    }

inline void VSettingsOpenGL::PySetEnableLight1(const bool& enableInit) { 
    PyWarning("VisualizationSettings parameter openGL.enableLight1 is deprecated! use openGL.light1.enable instead!");
    backlink->openGL.light1.enable= (const bool&)enableInit; 
    }
inline bool VSettingsOpenGL::PyGetEnableLight1() const { 
    PyWarning("VisualizationSettings parameter openGL.enableLight1 is deprecated! use openGL.light1.enable instead!");
    return bool(backlink->openGL.light1.enable); 
    }

inline void VSettingsOpenGL::PySetEnableLighting(const bool& enableLightingInit) { 
    PyWarning("VisualizationSettings parameter openGL.enableLighting is deprecated! use openGL.advanced.enableLighting instead!");
    backlink->openGL.advanced.enableLighting= (const bool&)enableLightingInit; 
    }
inline bool VSettingsOpenGL::PyGetEnableLighting() const { 
    PyWarning("VisualizationSettings parameter openGL.enableLighting is deprecated! use openGL.advanced.enableLighting instead!");
    return bool(backlink->openGL.advanced.enableLighting); 
    }

inline void VSettingsOpenGL::PySetFacesTransparent(const bool& facesTransparentInit) { 
    PyWarning("VisualizationSettings parameter openGL.facesTransparent is deprecated! use view0.scene.facesTransparent instead!");
    backlink->view0.scene.facesTransparent= (const bool&)facesTransparentInit; 
    }
inline bool VSettingsOpenGL::PyGetFacesTransparent() const { 
    PyWarning("VisualizationSettings parameter openGL.facesTransparent is deprecated! use view0.scene.facesTransparent instead!");
    return bool(backlink->view0.scene.facesTransparent); 
    }

inline void VSettingsOpenGL::PySetInitialCenterPoint(const std::array<float,3>& initialCenterPointInit) { 
    PyWarning("VisualizationSettings parameter openGL.initialCenterPoint is deprecated! use openGL.advanced.initialCenterPoint instead!");
    backlink->openGL.advanced.initialCenterPoint= (const Float3&)initialCenterPointInit; 
    }
inline std::array<float,3> VSettingsOpenGL::PyGetInitialCenterPoint() const { 
    PyWarning("VisualizationSettings parameter openGL.initialCenterPoint is deprecated! use openGL.advanced.initialCenterPoint instead!");
    return std::array<float,3>(backlink->openGL.advanced.initialCenterPoint); 
    }

inline void VSettingsOpenGL::PySetInitialMaxSceneSize(const float& initialMaxSceneSizeInit) { 
    PyWarning("VisualizationSettings parameter openGL.initialMaxSceneSize is deprecated! use openGL.advanced.initialMaxSceneSize instead!");
    backlink->openGL.advanced.initialMaxSceneSize= (const float&)initialMaxSceneSizeInit; 
    }
inline float VSettingsOpenGL::PyGetInitialMaxSceneSize() const { 
    PyWarning("VisualizationSettings parameter openGL.initialMaxSceneSize is deprecated! use openGL.advanced.initialMaxSceneSize instead!");
    return float(backlink->openGL.advanced.initialMaxSceneSize); 
    }

inline void VSettingsOpenGL::PySetInitialModelRotation(const StdArray33F& initialModelRotationInit) { 
    PyWarning("VisualizationSettings parameter openGL.initialModelRotation is deprecated! use openGL.advanced.initialModelRotation instead!");
    backlink->openGL.advanced.initialModelRotation= (const StdArray33F&)initialModelRotationInit; 
    }
inline StdArray33F VSettingsOpenGL::PyGetInitialModelRotation() const { 
    PyWarning("VisualizationSettings parameter openGL.initialModelRotation is deprecated! use openGL.advanced.initialModelRotation instead!");
    return StdArray33F(backlink->openGL.advanced.initialModelRotation); 
    }

inline void VSettingsOpenGL::PySetInitialZoom(const float& initialZoomInit) { 
    PyWarning("VisualizationSettings parameter openGL.initialZoom is deprecated! use openGL.advanced.initialZoom instead!");
    backlink->openGL.advanced.initialZoom= (const float&)initialZoomInit; 
    }
inline float VSettingsOpenGL::PyGetInitialZoom() const { 
    PyWarning("VisualizationSettings parameter openGL.initialZoom is deprecated! use openGL.advanced.initialZoom instead!");
    return float(backlink->openGL.advanced.initialZoom); 
    }

inline void VSettingsOpenGL::PySetLight0ambient(const float& dummyInit) { 
    PyWarning("VisualizationSettings parameter openGL.light0ambient is deprecated! use openGL.dummy instead!");
    backlink->openGL.dummy= (const float&)dummyInit; 
    }
inline float VSettingsOpenGL::PyGetLight0ambient() const { 
    PyWarning("VisualizationSettings parameter openGL.light0ambient is deprecated! use openGL.dummy instead!");
    return float(backlink->openGL.dummy); 
    }

inline void VSettingsOpenGL::PySetLight0constantAttenuation(const float& constantAttenuationInit) { 
    PyWarning("VisualizationSettings parameter openGL.light0constantAttenuation is deprecated! use openGL.light0.constantAttenuation instead!");
    backlink->openGL.light0.constantAttenuation= (const float&)constantAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight0constantAttenuation() const { 
    PyWarning("VisualizationSettings parameter openGL.light0constantAttenuation is deprecated! use openGL.light0.constantAttenuation instead!");
    return float(backlink->openGL.light0.constantAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight0diffuse(const float& diffuseInit) { 
    PyWarning("VisualizationSettings parameter openGL.light0diffuse is deprecated! use openGL.light0.diffuse instead!");
    backlink->openGL.light0.diffuse= (const float&)diffuseInit; 
    }
inline float VSettingsOpenGL::PyGetLight0diffuse() const { 
    PyWarning("VisualizationSettings parameter openGL.light0diffuse is deprecated! use openGL.light0.diffuse instead!");
    return float(backlink->openGL.light0.diffuse); 
    }

inline void VSettingsOpenGL::PySetLight0linearAttenuation(const float& linearAttenuationInit) { 
    PyWarning("VisualizationSettings parameter openGL.light0linearAttenuation is deprecated! use openGL.light0.linearAttenuation instead!");
    backlink->openGL.light0.linearAttenuation= (const float&)linearAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight0linearAttenuation() const { 
    PyWarning("VisualizationSettings parameter openGL.light0linearAttenuation is deprecated! use openGL.light0.linearAttenuation instead!");
    return float(backlink->openGL.light0.linearAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight0position(const std::array<float,4>& positionInit) { 
    PyWarning("VisualizationSettings parameter openGL.light0position is deprecated! use openGL.light0.position instead!");
    backlink->openGL.light0.position= (const Float4&)positionInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetLight0position() const { 
    PyWarning("VisualizationSettings parameter openGL.light0position is deprecated! use openGL.light0.position instead!");
    return std::array<float,4>(backlink->openGL.light0.position); 
    }

inline void VSettingsOpenGL::PySetLight0quadraticAttenuation(const float& quadraticAttenuationInit) { 
    PyWarning("VisualizationSettings parameter openGL.light0quadraticAttenuation is deprecated! use openGL.light0.quadraticAttenuation instead!");
    backlink->openGL.light0.quadraticAttenuation= (const float&)quadraticAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight0quadraticAttenuation() const { 
    PyWarning("VisualizationSettings parameter openGL.light0quadraticAttenuation is deprecated! use openGL.light0.quadraticAttenuation instead!");
    return float(backlink->openGL.light0.quadraticAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight0specular(const float& specularInit) { 
    PyWarning("VisualizationSettings parameter openGL.light0specular is deprecated! use openGL.light0.specular instead!");
    backlink->openGL.light0.specular= (const float&)specularInit; 
    }
inline float VSettingsOpenGL::PyGetLight0specular() const { 
    PyWarning("VisualizationSettings parameter openGL.light0specular is deprecated! use openGL.light0.specular instead!");
    return float(backlink->openGL.light0.specular); 
    }

inline void VSettingsOpenGL::PySetLight1ambient(const float& dummyInit) { 
    PyWarning("VisualizationSettings parameter openGL.light1ambient is deprecated! use openGL.dummy instead!");
    backlink->openGL.dummy= (const float&)dummyInit; 
    }
inline float VSettingsOpenGL::PyGetLight1ambient() const { 
    PyWarning("VisualizationSettings parameter openGL.light1ambient is deprecated! use openGL.dummy instead!");
    return float(backlink->openGL.dummy); 
    }

inline void VSettingsOpenGL::PySetLight1constantAttenuation(const float& constantAttenuationInit) { 
    PyWarning("VisualizationSettings parameter openGL.light1constantAttenuation is deprecated! use openGL.light1.constantAttenuation instead!");
    backlink->openGL.light1.constantAttenuation= (const float&)constantAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight1constantAttenuation() const { 
    PyWarning("VisualizationSettings parameter openGL.light1constantAttenuation is deprecated! use openGL.light1.constantAttenuation instead!");
    return float(backlink->openGL.light1.constantAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight1diffuse(const float& diffuseInit) { 
    PyWarning("VisualizationSettings parameter openGL.light1diffuse is deprecated! use openGL.light1.diffuse instead!");
    backlink->openGL.light1.diffuse= (const float&)diffuseInit; 
    }
inline float VSettingsOpenGL::PyGetLight1diffuse() const { 
    PyWarning("VisualizationSettings parameter openGL.light1diffuse is deprecated! use openGL.light1.diffuse instead!");
    return float(backlink->openGL.light1.diffuse); 
    }

inline void VSettingsOpenGL::PySetLight1linearAttenuation(const float& linearAttenuationInit) { 
    PyWarning("VisualizationSettings parameter openGL.light1linearAttenuation is deprecated! use openGL.light1.linearAttenuation instead!");
    backlink->openGL.light1.linearAttenuation= (const float&)linearAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight1linearAttenuation() const { 
    PyWarning("VisualizationSettings parameter openGL.light1linearAttenuation is deprecated! use openGL.light1.linearAttenuation instead!");
    return float(backlink->openGL.light1.linearAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight1position(const std::array<float,4>& positionInit) { 
    PyWarning("VisualizationSettings parameter openGL.light1position is deprecated! use openGL.light1.position instead!");
    backlink->openGL.light1.position= (const Float4&)positionInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetLight1position() const { 
    PyWarning("VisualizationSettings parameter openGL.light1position is deprecated! use openGL.light1.position instead!");
    return std::array<float,4>(backlink->openGL.light1.position); 
    }

inline void VSettingsOpenGL::PySetLight1quadraticAttenuation(const float& quadraticAttenuationInit) { 
    PyWarning("VisualizationSettings parameter openGL.light1quadraticAttenuation is deprecated! use openGL.light1.quadraticAttenuation instead!");
    backlink->openGL.light1.quadraticAttenuation= (const float&)quadraticAttenuationInit; 
    }
inline float VSettingsOpenGL::PyGetLight1quadraticAttenuation() const { 
    PyWarning("VisualizationSettings parameter openGL.light1quadraticAttenuation is deprecated! use openGL.light1.quadraticAttenuation instead!");
    return float(backlink->openGL.light1.quadraticAttenuation); 
    }

inline void VSettingsOpenGL::PySetLight1specular(const float& specularInit) { 
    PyWarning("VisualizationSettings parameter openGL.light1specular is deprecated! use openGL.light1.specular instead!");
    backlink->openGL.light1.specular= (const float&)specularInit; 
    }
inline float VSettingsOpenGL::PyGetLight1specular() const { 
    PyWarning("VisualizationSettings parameter openGL.light1specular is deprecated! use openGL.light1.specular instead!");
    return float(backlink->openGL.light1.specular); 
    }

inline void VSettingsOpenGL::PySetLightModelLocalViewer(const bool& lightModelLocalViewerInit) { 
    PyWarning("VisualizationSettings parameter openGL.lightModelLocalViewer is deprecated! use openGL.advanced.lightModelLocalViewer instead!");
    backlink->openGL.advanced.lightModelLocalViewer= (const bool&)lightModelLocalViewerInit; 
    }
inline bool VSettingsOpenGL::PyGetLightModelLocalViewer() const { 
    PyWarning("VisualizationSettings parameter openGL.lightModelLocalViewer is deprecated! use openGL.advanced.lightModelLocalViewer instead!");
    return bool(backlink->openGL.advanced.lightModelLocalViewer); 
    }

inline void VSettingsOpenGL::PySetLightModelTwoSide(const bool& lightModelTwoSideInit) { 
    PyWarning("VisualizationSettings parameter openGL.lightModelTwoSide is deprecated! use openGL.advanced.lightModelTwoSide instead!");
    backlink->openGL.advanced.lightModelTwoSide= (const bool&)lightModelTwoSideInit; 
    }
inline bool VSettingsOpenGL::PyGetLightModelTwoSide() const { 
    PyWarning("VisualizationSettings parameter openGL.lightModelTwoSide is deprecated! use openGL.advanced.lightModelTwoSide instead!");
    return bool(backlink->openGL.advanced.lightModelTwoSide); 
    }

inline void VSettingsOpenGL::PySetLightPositionsInCameraFrame(const bool& useCameraFrameInit) { 
    PyWarning("VisualizationSettings parameter openGL.lightPositionsInCameraFrame is deprecated! use openGL.light0.useCameraFrame instead!");
    backlink->openGL.light0.useCameraFrame= (const bool&)useCameraFrameInit; 
    }
inline bool VSettingsOpenGL::PyGetLightPositionsInCameraFrame() const { 
    PyWarning("VisualizationSettings parameter openGL.lightPositionsInCameraFrame is deprecated! use openGL.light0.useCameraFrame instead!");
    return bool(backlink->openGL.light0.useCameraFrame); 
    }

inline void VSettingsOpenGL::PySetLineSmooth(const bool& lineSmoothInit) { 
    PyWarning("VisualizationSettings parameter openGL.lineSmooth is deprecated! use openGL.advanced.lineSmooth instead!");
    backlink->openGL.advanced.lineSmooth= (const bool&)lineSmoothInit; 
    }
inline bool VSettingsOpenGL::PyGetLineSmooth() const { 
    PyWarning("VisualizationSettings parameter openGL.lineSmooth is deprecated! use openGL.advanced.lineSmooth instead!");
    return bool(backlink->openGL.advanced.lineSmooth); 
    }

inline void VSettingsOpenGL::PySetMaterialAmbientAndDiffuse(const std::array<float,4>& materialSpecularInit) { 
    PyWarning("VisualizationSettings parameter openGL.materialAmbientAndDiffuse is deprecated! use openGL.materialSpecular instead!");
    backlink->openGL.materialSpecular= (const Float4&)materialSpecularInit; 
    }
inline std::array<float,4> VSettingsOpenGL::PyGetMaterialAmbientAndDiffuse() const { 
    PyWarning("VisualizationSettings parameter openGL.materialAmbientAndDiffuse is deprecated! use openGL.materialSpecular instead!");
    return std::array<float,4>(backlink->openGL.materialSpecular); 
    }

inline void VSettingsOpenGL::PySetPerspective(const float& perspectiveInit) { 
    PyWarning("VisualizationSettings parameter openGL.perspective is deprecated! use view0.camera.perspective instead!");
    backlink->view0.camera.perspective= (const float&)perspectiveInit; 
    }
inline float VSettingsOpenGL::PyGetPerspective() const { 
    PyWarning("VisualizationSettings parameter openGL.perspective is deprecated! use view0.camera.perspective instead!");
    return float(backlink->view0.camera.perspective); 
    }

inline void VSettingsOpenGL::PySetPolygonOffset(const float& polygonOffsetInit) { 
    PyWarning("VisualizationSettings parameter openGL.polygonOffset is deprecated! use openGL.advanced.polygonOffset instead!");
    backlink->openGL.advanced.polygonOffset= (const float&)polygonOffsetInit; 
    }
inline float VSettingsOpenGL::PyGetPolygonOffset() const { 
    PyWarning("VisualizationSettings parameter openGL.polygonOffset is deprecated! use openGL.advanced.polygonOffset instead!");
    return float(backlink->openGL.advanced.polygonOffset); 
    }

inline void VSettingsOpenGL::PySetShadeModelSmooth(const bool& shadeModelSmoothInit) { 
    PyWarning("VisualizationSettings parameter openGL.shadeModelSmooth is deprecated! use openGL.advanced.shadeModelSmooth instead!");
    backlink->openGL.advanced.shadeModelSmooth= (const bool&)shadeModelSmoothInit; 
    }
inline bool VSettingsOpenGL::PyGetShadeModelSmooth() const { 
    PyWarning("VisualizationSettings parameter openGL.shadeModelSmooth is deprecated! use openGL.advanced.shadeModelSmooth instead!");
    return bool(backlink->openGL.advanced.shadeModelSmooth); 
    }

inline void VSettingsOpenGL::PySetShadow(const float& shadowInit) { 
    PyWarning("VisualizationSettings parameter openGL.shadow is deprecated! use openGL.light0.shadow instead!");
    backlink->openGL.light0.shadow= (const float&)shadowInit; 
    }
inline float VSettingsOpenGL::PyGetShadow() const { 
    PyWarning("VisualizationSettings parameter openGL.shadow is deprecated! use openGL.light0.shadow instead!");
    return float(backlink->openGL.light0.shadow); 
    }

inline void VSettingsOpenGL::PySetShadowPolygonOffset(const float& shadowPolygonOffsetInit) { 
    PyWarning("VisualizationSettings parameter openGL.shadowPolygonOffset is deprecated! use openGL.advanced.shadowPolygonOffset instead!");
    backlink->openGL.advanced.shadowPolygonOffset= (const float&)shadowPolygonOffsetInit; 
    }
inline float VSettingsOpenGL::PyGetShadowPolygonOffset() const { 
    PyWarning("VisualizationSettings parameter openGL.shadowPolygonOffset is deprecated! use openGL.advanced.shadowPolygonOffset instead!");
    return float(backlink->openGL.advanced.shadowPolygonOffset); 
    }

inline void VSettingsOpenGL::PySetShowFaceEdges(const bool& showFaceEdgesInit) { 
    PyWarning("VisualizationSettings parameter openGL.showFaceEdges is deprecated! use view0.scene.showFaceEdges instead!");
    backlink->view0.scene.showFaceEdges= (const bool&)showFaceEdgesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowFaceEdges() const { 
    PyWarning("VisualizationSettings parameter openGL.showFaceEdges is deprecated! use view0.scene.showFaceEdges instead!");
    return bool(backlink->view0.scene.showFaceEdges); 
    }

inline void VSettingsOpenGL::PySetShowFaces(const bool& showFacesInit) { 
    PyWarning("VisualizationSettings parameter openGL.showFaces is deprecated! use view0.scene.showFaces instead!");
    backlink->view0.scene.showFaces= (const bool&)showFacesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowFaces() const { 
    PyWarning("VisualizationSettings parameter openGL.showFaces is deprecated! use view0.scene.showFaces instead!");
    return bool(backlink->view0.scene.showFaces); 
    }

inline void VSettingsOpenGL::PySetShowLines(const bool& showLinesInit) { 
    PyWarning("VisualizationSettings parameter openGL.showLines is deprecated! use view0.scene.showLines instead!");
    backlink->view0.scene.showLines= (const bool&)showLinesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowLines() const { 
    PyWarning("VisualizationSettings parameter openGL.showLines is deprecated! use view0.scene.showLines instead!");
    return bool(backlink->view0.scene.showLines); 
    }

inline void VSettingsOpenGL::PySetShowMeshEdges(const bool& showMeshEdgesInit) { 
    PyWarning("VisualizationSettings parameter openGL.showMeshEdges is deprecated! use view0.scene.showMeshEdges instead!");
    backlink->view0.scene.showMeshEdges= (const bool&)showMeshEdgesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowMeshEdges() const { 
    PyWarning("VisualizationSettings parameter openGL.showMeshEdges is deprecated! use view0.scene.showMeshEdges instead!");
    return bool(backlink->view0.scene.showMeshEdges); 
    }

inline void VSettingsOpenGL::PySetShowMeshFaces(const bool& showMeshFacesInit) { 
    PyWarning("VisualizationSettings parameter openGL.showMeshFaces is deprecated! use view0.scene.showMeshFaces instead!");
    backlink->view0.scene.showMeshFaces= (const bool&)showMeshFacesInit; 
    }
inline bool VSettingsOpenGL::PyGetShowMeshFaces() const { 
    PyWarning("VisualizationSettings parameter openGL.showMeshFaces is deprecated! use view0.scene.showMeshFaces instead!");
    return bool(backlink->view0.scene.showMeshFaces); 
    }

inline void VSettingsOpenGL::PySetTextLineSmooth(const bool& textLineSmoothInit) { 
    PyWarning("VisualizationSettings parameter openGL.textLineSmooth is deprecated! use openGL.advanced.textLineSmooth instead!");
    backlink->openGL.advanced.textLineSmooth= (const bool&)textLineSmoothInit; 
    }
inline bool VSettingsOpenGL::PyGetTextLineSmooth() const { 
    PyWarning("VisualizationSettings parameter openGL.textLineSmooth is deprecated! use openGL.advanced.textLineSmooth instead!");
    return bool(backlink->openGL.advanced.textLineSmooth); 
    }

inline void VSettingsOpenGL::PySetTextLineWidth(const float& textLineWidthInit) { 
    PyWarning("VisualizationSettings parameter openGL.textLineWidth is deprecated! use openGL.advanced.textLineWidth instead!");
    backlink->openGL.advanced.textLineWidth= (const float&)textLineWidthInit; 
    }
inline float VSettingsOpenGL::PyGetTextLineWidth() const { 
    PyWarning("VisualizationSettings parameter openGL.textLineWidth is deprecated! use openGL.advanced.textLineWidth instead!");
    return float(backlink->openGL.advanced.textLineWidth); 
    }

inline void VSettingsInteractive::PySetHighlightColor(const std::array<float,4>& highlightColorInit) { 
    PyWarning("VisualizationSettings parameter interactive.highlightColor is deprecated! use interactive.advanced.highlightColor instead!");
    backlink->interactive.advanced.highlightColor= (const Float4&)highlightColorInit; 
    }
inline std::array<float,4> VSettingsInteractive::PyGetHighlightColor() const { 
    PyWarning("VisualizationSettings parameter interactive.highlightColor is deprecated! use interactive.advanced.highlightColor instead!");
    return std::array<float,4>(backlink->interactive.advanced.highlightColor); 
    }

inline void VSettingsInteractive::PySetHighlightOtherColor(const std::array<float,4>& highlightOtherColorInit) { 
    PyWarning("VisualizationSettings parameter interactive.highlightOtherColor is deprecated! use interactive.advanced.highlightOtherColor instead!");
    backlink->interactive.advanced.highlightOtherColor= (const Float4&)highlightOtherColorInit; 
    }
inline std::array<float,4> VSettingsInteractive::PyGetHighlightOtherColor() const { 
    PyWarning("VisualizationSettings parameter interactive.highlightOtherColor is deprecated! use interactive.advanced.highlightOtherColor instead!");
    return std::array<float,4>(backlink->interactive.advanced.highlightOtherColor); 
    }

inline void VSettingsInteractive::PySetJoystickScaleRotation(const float& joystickScaleRotationInit) { 
    PyWarning("VisualizationSettings parameter interactive.joystickScaleRotation is deprecated! use interactive.advanced.joystickScaleRotation instead!");
    backlink->interactive.advanced.joystickScaleRotation= (const float&)joystickScaleRotationInit; 
    }
inline float VSettingsInteractive::PyGetJoystickScaleRotation() const { 
    PyWarning("VisualizationSettings parameter interactive.joystickScaleRotation is deprecated! use interactive.advanced.joystickScaleRotation instead!");
    return float(backlink->interactive.advanced.joystickScaleRotation); 
    }

inline void VSettingsInteractive::PySetJoystickScaleTranslation(const float& joystickScaleTranslationInit) { 
    PyWarning("VisualizationSettings parameter interactive.joystickScaleTranslation is deprecated! use interactive.advanced.joystickScaleTranslation instead!");
    backlink->interactive.advanced.joystickScaleTranslation= (const float&)joystickScaleTranslationInit; 
    }
inline float VSettingsInteractive::PyGetJoystickScaleTranslation() const { 
    PyWarning("VisualizationSettings parameter interactive.joystickScaleTranslation is deprecated! use interactive.advanced.joystickScaleTranslation instead!");
    return float(backlink->interactive.advanced.joystickScaleTranslation); 
    }

inline void VSettingsInteractive::PySetKeypressRotationStep(const float& keypressRotationStepInit) { 
    PyWarning("VisualizationSettings parameter interactive.keypressRotationStep is deprecated! use interactive.advanced.keypressRotationStep instead!");
    backlink->interactive.advanced.keypressRotationStep= (const float&)keypressRotationStepInit; 
    }
inline float VSettingsInteractive::PyGetKeypressRotationStep() const { 
    PyWarning("VisualizationSettings parameter interactive.keypressRotationStep is deprecated! use interactive.advanced.keypressRotationStep instead!");
    return float(backlink->interactive.advanced.keypressRotationStep); 
    }

inline void VSettingsInteractive::PySetKeypressTranslationStep(const float& keypressTranslationStepInit) { 
    PyWarning("VisualizationSettings parameter interactive.keypressTranslationStep is deprecated! use interactive.advanced.keypressTranslationStep instead!");
    backlink->interactive.advanced.keypressTranslationStep= (const float&)keypressTranslationStepInit; 
    }
inline float VSettingsInteractive::PyGetKeypressTranslationStep() const { 
    PyWarning("VisualizationSettings parameter interactive.keypressTranslationStep is deprecated! use interactive.advanced.keypressTranslationStep instead!");
    return float(backlink->interactive.advanced.keypressTranslationStep); 
    }

inline void VSettingsInteractive::PySetLockModelView(const bool& lockModelViewInit) { 
    PyWarning("VisualizationSettings parameter interactive.lockModelView is deprecated! use view0.window.lockModelView instead!");
    backlink->view0.window.lockModelView= (const bool&)lockModelViewInit; 
    }
inline bool VSettingsInteractive::PyGetLockModelView() const { 
    PyWarning("VisualizationSettings parameter interactive.lockModelView is deprecated! use view0.window.lockModelView instead!");
    return bool(backlink->view0.window.lockModelView); 
    }

inline void VSettingsInteractive::PySetMouseMoveRotationFactor(const float& mouseMoveRotationFactorInit) { 
    PyWarning("VisualizationSettings parameter interactive.mouseMoveRotationFactor is deprecated! use interactive.advanced.mouseMoveRotationFactor instead!");
    backlink->interactive.advanced.mouseMoveRotationFactor= (const float&)mouseMoveRotationFactorInit; 
    }
inline float VSettingsInteractive::PyGetMouseMoveRotationFactor() const { 
    PyWarning("VisualizationSettings parameter interactive.mouseMoveRotationFactor is deprecated! use interactive.advanced.mouseMoveRotationFactor instead!");
    return float(backlink->interactive.advanced.mouseMoveRotationFactor); 
    }

inline void VSettingsInteractive::PySetPauseWithSpacebar(const bool& pauseWithSpacebarInit) { 
    PyWarning("VisualizationSettings parameter interactive.pauseWithSpacebar is deprecated! use interactive.advanced.pauseWithSpacebar instead!");
    backlink->interactive.advanced.pauseWithSpacebar= (const bool&)pauseWithSpacebarInit; 
    }
inline bool VSettingsInteractive::PyGetPauseWithSpacebar() const { 
    PyWarning("VisualizationSettings parameter interactive.pauseWithSpacebar is deprecated! use interactive.advanced.pauseWithSpacebar instead!");
    return bool(backlink->interactive.advanced.pauseWithSpacebar); 
    }

inline void VSettingsInteractive::PySetSelectionHighlights(const bool& selectionHighlightsInit) { 
    PyWarning("VisualizationSettings parameter interactive.selectionHighlights is deprecated! use interactive.advanced.selectionHighlights instead!");
    backlink->interactive.advanced.selectionHighlights= (const bool&)selectionHighlightsInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionHighlights() const { 
    PyWarning("VisualizationSettings parameter interactive.selectionHighlights is deprecated! use interactive.advanced.selectionHighlights instead!");
    return bool(backlink->interactive.advanced.selectionHighlights); 
    }

inline void VSettingsInteractive::PySetSelectionLeftMouse(const bool& selectionLeftMouseInit) { 
    PyWarning("VisualizationSettings parameter interactive.selectionLeftMouse is deprecated! use interactive.advanced.selectionLeftMouse instead!");
    backlink->interactive.advanced.selectionLeftMouse= (const bool&)selectionLeftMouseInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionLeftMouse() const { 
    PyWarning("VisualizationSettings parameter interactive.selectionLeftMouse is deprecated! use interactive.advanced.selectionLeftMouse instead!");
    return bool(backlink->interactive.advanced.selectionLeftMouse); 
    }

inline void VSettingsInteractive::PySetSelectionLeftMouseItemTypes(const Index& selectionLeftMouseItemTypesInit) { 
    PyWarning("VisualizationSettings parameter interactive.selectionLeftMouseItemTypes is deprecated! use interactive.advanced.selectionLeftMouseItemTypes instead!");
    backlink->interactive.advanced.selectionLeftMouseItemTypes= (const Index&)selectionLeftMouseItemTypesInit; 
    }
inline Index VSettingsInteractive::PyGetSelectionLeftMouseItemTypes() const { 
    PyWarning("VisualizationSettings parameter interactive.selectionLeftMouseItemTypes is deprecated! use interactive.advanced.selectionLeftMouseItemTypes instead!");
    return Index(backlink->interactive.advanced.selectionLeftMouseItemTypes); 
    }

inline void VSettingsInteractive::PySetSelectionRightMouse(const bool& selectionRightMouseInit) { 
    PyWarning("VisualizationSettings parameter interactive.selectionRightMouse is deprecated! use interactive.advanced.selectionRightMouse instead!");
    backlink->interactive.advanced.selectionRightMouse= (const bool&)selectionRightMouseInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionRightMouse() const { 
    PyWarning("VisualizationSettings parameter interactive.selectionRightMouse is deprecated! use interactive.advanced.selectionRightMouse instead!");
    return bool(backlink->interactive.advanced.selectionRightMouse); 
    }

inline void VSettingsInteractive::PySetSelectionRightMouseGraphicsData(const bool& selectionRightMouseGraphicsDataInit) { 
    PyWarning("VisualizationSettings parameter interactive.selectionRightMouseGraphicsData is deprecated! use interactive.advanced.selectionRightMouseGraphicsData instead!");
    backlink->interactive.advanced.selectionRightMouseGraphicsData= (const bool&)selectionRightMouseGraphicsDataInit; 
    }
inline bool VSettingsInteractive::PyGetSelectionRightMouseGraphicsData() const { 
    PyWarning("VisualizationSettings parameter interactive.selectionRightMouseGraphicsData is deprecated! use interactive.advanced.selectionRightMouseGraphicsData instead!");
    return bool(backlink->interactive.advanced.selectionRightMouseGraphicsData); 
    }

inline void VSettingsInteractive::PySetTrackMarker(const Index& trackMarkerInit) { 
    PyWarning("VisualizationSettings parameter interactive.trackMarker is deprecated! use view0.camera.trackMarker instead!");
    backlink->view0.camera.trackMarker= (const Index&)trackMarkerInit; 
    }
inline Index VSettingsInteractive::PyGetTrackMarker() const { 
    PyWarning("VisualizationSettings parameter interactive.trackMarker is deprecated! use view0.camera.trackMarker instead!");
    return Index(backlink->view0.camera.trackMarker); 
    }

inline void VSettingsInteractive::PySetTrackMarkerMbsNumber(const Index& trackMarkerMbsNumberInit) { 
    PyWarning("VisualizationSettings parameter interactive.trackMarkerMbsNumber is deprecated! use view0.camera.trackMarkerMbsNumber instead!");
    backlink->view0.camera.trackMarkerMbsNumber= (const Index&)trackMarkerMbsNumberInit; 
    }
inline Index VSettingsInteractive::PyGetTrackMarkerMbsNumber() const { 
    PyWarning("VisualizationSettings parameter interactive.trackMarkerMbsNumber is deprecated! use view0.camera.trackMarkerMbsNumber instead!");
    return Index(backlink->view0.camera.trackMarkerMbsNumber); 
    }

inline void VSettingsInteractive::PySetTrackMarkerOrientation(const std::array<float,3>& trackMarkerOrientationInit) { 
    PyWarning("VisualizationSettings parameter interactive.trackMarkerOrientation is deprecated! use view0.camera.trackMarkerOrientation instead!");
    backlink->view0.camera.trackMarkerOrientation= (const Float3&)trackMarkerOrientationInit; 
    }
inline std::array<float,3> VSettingsInteractive::PyGetTrackMarkerOrientation() const { 
    PyWarning("VisualizationSettings parameter interactive.trackMarkerOrientation is deprecated! use view0.camera.trackMarkerOrientation instead!");
    return std::array<float,3>(backlink->view0.camera.trackMarkerOrientation); 
    }

inline void VSettingsInteractive::PySetTrackMarkerPosition(const std::array<float,3>& trackMarkerPositionInit) { 
    PyWarning("VisualizationSettings parameter interactive.trackMarkerPosition is deprecated! use view0.camera.trackMarkerPosition instead!");
    backlink->view0.camera.trackMarkerPosition= (const Float3&)trackMarkerPositionInit; 
    }
inline std::array<float,3> VSettingsInteractive::PyGetTrackMarkerPosition() const { 
    PyWarning("VisualizationSettings parameter interactive.trackMarkerPosition is deprecated! use view0.camera.trackMarkerPosition instead!");
    return std::array<float,3>(backlink->view0.camera.trackMarkerPosition); 
    }

inline void VSettingsInteractive::PySetZoomStepFactor(const float& zoomStepFactorInit) { 
    PyWarning("VisualizationSettings parameter interactive.zoomStepFactor is deprecated! use interactive.advanced.zoomStepFactor instead!");
    backlink->interactive.advanced.zoomStepFactor= (const float&)zoomStepFactorInit; 
    }
inline float VSettingsInteractive::PyGetZoomStepFactor() const { 
    PyWarning("VisualizationSettings parameter interactive.zoomStepFactor is deprecated! use interactive.advanced.zoomStepFactor instead!");
    return float(backlink->interactive.advanced.zoomStepFactor); 
    }

#endif //#ifdef include once...
