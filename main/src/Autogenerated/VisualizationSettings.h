/** ***********************************************************************************************
* @class        VSettingsGeneral
* @brief        General settings for visualization.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  float textSize;                                 //!< AUTO: general text size if not overwritten
  float minSceneSize;                             //!< AUTO: minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
  Float4 backgroundColor;                         //!< AUTO: red, green, blue and alpha values for background of render window (white=[1,1,1,1]; black = [0,0,0,1])
  float coordinateSystemSize;                     //!< AUTO: size of coordinate system relative to screen
  bool drawCoordinateSystem;                      //!< AUTO: false = no coordinate system shown
  bool showComputationInfo;                       //!< AUTO: false = no info about computation (current time, solver, etc.) shown
  float pointSize;                                //!< AUTO: global point size (absolute)
  Index circleTiling;                             //!< AUTO: global number of segments for circles; if smaller than 2, 2 segments are used (flat)
  Index cylinderTiling;                           //!< AUTO: global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
  Index sphereTiling;                             //!< AUTO: global number of segments for spheres; if smaller than 2, 2 segments are used (flat)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsGeneral()
  {
    graphicsUpdateInterval = 0.1f;
    autoFitScene = true;
    textSize = 12.f;
    minSceneSize = 0.1f;
    backgroundColor = Float4({1.f,1.f,1.f,1.f});
    coordinateSystemSize = 0.4f;
    drawCoordinateSystem = true;
    showComputationInfo = true;
    pointSize = 0.01f;
    circleTiling = 16;
    cylinderTiling = 16;
    sphereTiling = 8;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: red, green, blue and alpha values for background of render window (white=[1,1,1,1]; black = [0,0,0,1])
  void PySetBackgroundColor(const std::array<float,4>& backgroundColorInit) { backgroundColor = backgroundColorInit; }
  //! AUTO: Read (Copy) access to: red, green, blue and alpha values for background of render window (white=[1,1,1,1]; black = [0,0,0,1])
  std::array<float,4> PyGetBackgroundColor() const { return (std::array<float,4>)(backgroundColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsGeneral" << ":\n";
    os << "  graphicsUpdateInterval = " << graphicsUpdateInterval << "\n";
    os << "  autoFitScene = " << autoFitScene << "\n";
    os << "  textSize = " << textSize << "\n";
    os << "  minSceneSize = " << minSceneSize << "\n";
    os << "  backgroundColor = " << backgroundColor << "\n";
    os << "  coordinateSystemSize = " << coordinateSystemSize << "\n";
    os << "  drawCoordinateSystem = " << drawCoordinateSystem << "\n";
    os << "  showComputationInfo = " << showComputationInfo << "\n";
    os << "  pointSize = " << pointSize << "\n";
    os << "  circleTiling = " << circleTiling << "\n";
    os << "  cylinderTiling = " << cylinderTiling << "\n";
    os << "  sphereTiling = " << sphereTiling << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VSettingsGeneral& object)
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
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  bool alwaysOnTop;                               //!< AUTO: true: OpenGL render window will be always on top of all other windows
  bool maximize;                                  //!< AUTO: true: OpenGL render window will be maximized at startup
  bool showWindow;                                //!< AUTO: true: OpenGL render window is shown on startup; false: window will be iconified at startup (e.g. if you are starting multiple computations automatically)
  float keypressRotationStep;                     //!< AUTO: rotation increment per keypress in degree (full rotation = 360 degree)
  float mouseMoveRotationFactor;                  //!< AUTO: rotation increment per 1 pixel mouse movement in degree
  float keypressTranslationStep;                  //!< AUTO: translation increment per keypress relative to window size
  float zoomStepFactor;                           //!< AUTO: change of zoom per keypress (keypad +/-) or mouse wheel increment


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsWindow()
  {
    renderWindowSize = Index2({1024,768});
    startupTimeout = 5000;
    alwaysOnTop = false;
    maximize = false;
    showWindow = true;
    keypressRotationStep = 5.f;
    mouseMoveRotationFactor = 1.f;
    keypressTranslationStep = 0.1f;
    zoomStepFactor = 1.15f;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: initial size of OpenGL render window in pixel
  void PySetRenderWindowSize(const std::array<Index,2>& renderWindowSizeInit) { renderWindowSize = renderWindowSizeInit; }
  //! AUTO: Read (Copy) access to: initial size of OpenGL render window in pixel
  std::array<Index,2> PyGetRenderWindowSize() const { return (std::array<Index,2>)(renderWindowSize); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsWindow" << ":\n";
    os << "  renderWindowSize = " << renderWindowSize << "\n";
    os << "  startupTimeout = " << startupTimeout << "\n";
    os << "  alwaysOnTop = " << alwaysOnTop << "\n";
    os << "  maximize = " << maximize << "\n";
    os << "  showWindow = " << showWindow << "\n";
    os << "  keypressRotationStep = " << keypressRotationStep << "\n";
    os << "  mouseMoveRotationFactor = " << mouseMoveRotationFactor << "\n";
    os << "  keypressTranslationStep = " << keypressTranslationStep << "\n";
    os << "  zoomStepFactor = " << zoomStepFactor << "\n";
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
* @brief        OpenGL settings for 2D and 2D rendering.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  bool showFaceEdges;                             //!< AUTO: show edges of faces; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
  bool shadeModelSmooth;                          //!< AUTO: true: turn on smoothing for shaders, which uses vertex normals to smooth surfaces
  Float4 materialSpecular;                        //!< AUTO: 4f specular color of material
  float materialShininess;                        //!< AUTO: shininess of material
  bool enableLight0;                              //!< AUTO: turn on/off light0
  Float4 light0position;                          //!< AUTO: 4f position vector of GL light0; 4th value should be 0, otherwise the vector obtains a special interpretation, see opengl manuals
  float light0ambient;                            //!< AUTO: ambient value of GL light0
  float light0diffuse;                            //!< AUTO: diffuse value of GL light0
  float light0specular;                           //!< AUTO: specular value of GL light0
  bool enableLight1;                              //!< AUTO: turn on/off light1
  Float4 light1position;                          //!< AUTO: 4f position vector of GL light1; 4th value should be 0, otherwise the vector obtains a special interpretation, see opengl manuals
  float light1ambient;                            //!< AUTO: ambient value of GL light1
  float light1diffuse;                            //!< AUTO: diffuse value of GL light1
  float light1specular;                           //!< AUTO: specular value of GL light1
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
    showFaceEdges = false;
    shadeModelSmooth = true;
    materialSpecular = Float4({1.f,1.f,1.f,1.f});
    materialShininess = 60.f;
    enableLight0 = true;
    light0position = Float4({1.f,1.f,-10.f,0.f});
    light0ambient = 0.25f;
    light0diffuse = 0.4f;
    light0specular = 0.4f;
    enableLight1 = true;
    light1position = Float4({0.f,3.f,2.f,0.f});
    light1ambient = 0.25f;
    light1diffuse = 0.4f;
    light1specular = 0.f;
    drawFaceNormals = false;
    drawVertexNormals = false;
    drawNormalsLength = 0.1f;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  void PySetInitialCenterPoint(const std::array<float,3>& initialCenterPointInit) { initialCenterPoint = initialCenterPointInit; }
  //! AUTO: Read (Copy) access to: centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
  std::array<float,3> PyGetInitialCenterPoint() const { return (std::array<float,3>)(initialCenterPoint); }

  //! AUTO: Set function (needed in pybind) for: 4f specular color of material
  void PySetMaterialSpecular(const std::array<float,4>& materialSpecularInit) { materialSpecular = materialSpecularInit; }
  //! AUTO: Read (Copy) access to: 4f specular color of material
  std::array<float,4> PyGetMaterialSpecular() const { return (std::array<float,4>)(materialSpecular); }

  //! AUTO: Set function (needed in pybind) for: 4f position vector of GL light0; 4th value should be 0, otherwise the vector obtains a special interpretation, see opengl manuals
  void PySetLight0position(const std::array<float,4>& light0positionInit) { light0position = light0positionInit; }
  //! AUTO: Read (Copy) access to: 4f position vector of GL light0; 4th value should be 0, otherwise the vector obtains a special interpretation, see opengl manuals
  std::array<float,4> PyGetLight0position() const { return (std::array<float,4>)(light0position); }

  //! AUTO: Set function (needed in pybind) for: 4f position vector of GL light1; 4th value should be 0, otherwise the vector obtains a special interpretation, see opengl manuals
  void PySetLight1position(const std::array<float,4>& light1positionInit) { light1position = light1positionInit; }
  //! AUTO: Read (Copy) access to: 4f position vector of GL light1; 4th value should be 0, otherwise the vector obtains a special interpretation, see opengl manuals
  std::array<float,4> PyGetLight1position() const { return (std::array<float,4>)(light1position); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsOpenGL" << ":\n";
    os << "  initialCenterPoint = " << initialCenterPoint << "\n";
    os << "  initialZoom = " << initialZoom << "\n";
    os << "  initialMaxSceneSize = " << initialMaxSceneSize << "\n";
    os << "  initialModelRotation = " << Matrix3DF(initialModelRotation) << "\n";
    os << "  multiSampling = " << multiSampling << "\n";
    os << "  lineWidth = " << lineWidth << "\n";
    os << "  lineSmooth = " << lineSmooth << "\n";
    os << "  textLineWidth = " << textLineWidth << "\n";
    os << "  textLineSmooth = " << textLineSmooth << "\n";
    os << "  showFaces = " << showFaces << "\n";
    os << "  showFaceEdges = " << showFaceEdges << "\n";
    os << "  shadeModelSmooth = " << shadeModelSmooth << "\n";
    os << "  materialSpecular = " << materialSpecular << "\n";
    os << "  materialShininess = " << materialShininess << "\n";
    os << "  enableLight0 = " << enableLight0 << "\n";
    os << "  light0position = " << light0position << "\n";
    os << "  light0ambient = " << light0ambient << "\n";
    os << "  light0diffuse = " << light0diffuse << "\n";
    os << "  light0specular = " << light0specular << "\n";
    os << "  enableLight1 = " << enableLight1 << "\n";
    os << "  light1position = " << light1position << "\n";
    os << "  light1ambient = " << light1ambient << "\n";
    os << "  light1diffuse = " << light1diffuse << "\n";
    os << "  light1specular = " << light1specular << "\n";
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
* @class        VSettingsContour
* @brief        Settings for contour plots; use these options to visualize field data, such as displacements, stresses, strains, etc. for bodies, nodes and finite elements.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class VSettingsContour // AUTO: 
{
public: // AUTO: 
  Index outputVariableComponent;                  //!< AUTO: select the component of the chosen output variable; e.g., for displacements, 3 components are available: 0 == x, 1 == y, 2 == z component; if this component is not available by certain objects or nodes, no value is drawn
  OutputVariableType outputVariable;              //!< AUTO: selected contour plot output variable type; select OutputVariableType.None to deactivate contour plotting.
  float minValue;                                 //!< AUTO: minimum value for contour plot; set manually, if automaticRange == False
  float maxValue;                                 //!< AUTO: maximum value for contour plot; set manually, if automaticRange == False
  bool automaticRange;                            //!< AUTO: if true, the contour plot value range is chosen automatically to the maximum range
  bool showColorBar;                              //!< AUTO: show the colour bar with minimum and maximum values for the contour plot
  Index colorBarTiling;                           //!< AUTO: number of tiles (segements) shown in the colorbar for the contour plot


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsContour()
  {
    outputVariableComponent = 0;
    outputVariable = OutputVariableType::None;
    minValue = 0;
    maxValue = 1;
    automaticRange = true;
    showColorBar = true;
    colorBarTiling = 12;
  };

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsContour" << ":\n";
    os << "  outputVariableComponent = " << outputVariableComponent << "\n";
    os << "  outputVariable = " << GetOutputVariableTypeString(outputVariable) << "\n";
    os << "  minValue = " << minValue << "\n";
    os << "  maxValue = " << maxValue << "\n";
    os << "  automaticRange = " << automaticRange << "\n";
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
* @class        VSettingsExportImages
* @brief        Functionality to export images to files (.tga format) which can be used to create animations; to activate image recording during the solution process, set SolutionSettings.recordImagesInterval accordingly.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class VSettingsExportImages // AUTO: 
{
public: // AUTO: 
  Index saveImageTimeOut;                         //!< AUTO: timeout for safing a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
  std::string saveImageFileName;                  //!< AUTO: filename (without extension!) and (relative) path for image file(s) with consecutive numbering (e.g., frame0000.tga, frame0001.tga,...); folders must already exist!
  Index saveImageFileCounter;                     //!< AUTO: current value of the counter which is used to consecutively save frames (images) with consecutive numbers
  bool saveImageSingleFile;                       //!< AUTO: true: only save single files with given filename, not adding numbering; false: add numbering to files, see saveImageFileName


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
* @class        VSettingsNodes
* @brief        Visualization settings for nodes.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  float defaultSize;                              //!< AUTO: global node size; if -1.f, node size is relative to openGL.initialMaxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for nodes; 4th value is alpha-transparency
  Index showNodalSlopes;                          //!< AUTO: draw nodal slope vectors, e.g. in ANCF beam finite elements


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsNodes()
  {
    show = true;
    showNumbers = false;
    defaultSize = -1.f;
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
    showNodalSlopes = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: default cRGB olor for nodes; 4th value is alpha-transparency
  void PySetDefaultColor(const std::array<float,4>& defaultColorInit) { defaultColor = defaultColorInit; }
  //! AUTO: Read (Copy) access to: default cRGB olor for nodes; 4th value is alpha-transparency
  std::array<float,4> PyGetDefaultColor() const { return (std::array<float,4>)(defaultColor); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VSettingsNodes" << ":\n";
    os << "  show = " << show << "\n";
    os << "  showNumbers = " << showNumbers << "\n";
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
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  VSettingsBeams beams;                           //!< AUTO: visualization settings for beams (e.g. ANCFCable or other beam elements)


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsBodies()
  {
    show = true;
    showNumbers = false;
    defaultSize = Float3({1.f,1.f,1.f});
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
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
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  bool showContact;                               //!< AUTO: flag to decide, whether contact points, lines, etc. are shown
  float defaultSize;                              //!< AUTO: global connector size; if -1.f, connector size is relative to maxSceneSize
  float contactPointsDefaultSize;                 //!< AUTO: global contact points size; if -1.f, connector size is relative to maxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for connectors; 4th value is alpha-transparency


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsConnectors()
  {
    show = true;
    showNumbers = false;
    showContact = false;
    defaultSize = 0.1f;
    contactPointsDefaultSize = 0.02f;
    defaultColor = Float4({0.2f,0.2f,1.f,1.f});
  };

  // AUTO: access functions
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
    os << "  showContact = " << showContact << "\n";
    os << "  defaultSize = " << defaultSize << "\n";
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
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  float defaultSize;                              //!< AUTO: global marker size; if -1.f, marker size is relative to maxSceneSize
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for markers; 4th value is alpha-transparency


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsMarkers()
  {
    show = true;
    showNumbers = false;
    defaultSize = 0.1f;
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
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

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
  float defaultSize;                              //!< AUTO: global load size; if -1.f, node size is relative to maxSceneSize
  bool fixedLoadSize;                             //!< AUTO: if true, the load is drawn with a fixed vector length in direction of the load vector, independently of the load size
  float loadSizeFactor;                           //!< AUTO: if fixedLoadSize=false, then this scaling factor is used to draw the load vector
  Float4 defaultColor;                            //!< AUTO: default cRGB olor for loads; 4th value is alpha-transparency


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  VSettingsLoads()
  {
    show = true;
    showNumbers = false;
    defaultSize = 0.2f;
    fixedLoadSize = true;
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
    os << "  fixedLoadSize = " << fixedLoadSize << "\n";
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
* @class        VisualizationSettings
* @brief        Settings for visualization
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-01-22 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class VisualizationSettings // AUTO: 
{
public: // AUTO: 
  VSettingsGeneral general;                       //!< AUTO: general visualization settings
  VSettingsWindow window;                         //!< AUTO: visualization window and interaction settings
  VSettingsOpenGL openGL;                         //!< AUTO: OpenGL rendering settings
  VSettingsContour contour;                       //!< AUTO: contour plot visualization settings
  VSettingsExportImages exportImages;             //!< AUTO: settings for exporting (saving) images to files in order to create animations
  VSettingsNodes nodes;                           //!< AUTO: node visualization settings
  VSettingsBodies bodies;                         //!< AUTO: body visualization settings
  VSettingsConnectors connectors;                 //!< AUTO: connector visualization settings
  VSettingsMarkers markers;                       //!< AUTO: marker visualization settings
  VSettingsLoads loads;                           //!< AUTO: load visualization settings


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "VisualizationSettings" << ":\n";
    os << "  general = " << general << "\n";
    os << "  window = " << window << "\n";
    os << "  openGL = " << openGL << "\n";
    os << "  contour = " << contour << "\n";
    os << "  exportImages = " << exportImages << "\n";
    os << "  nodes = " << nodes << "\n";
    os << "  bodies = " << bodies << "\n";
    os << "  connectors = " << connectors << "\n";
    os << "  markers = " << markers << "\n";
    os << "  loads = " << loads << "\n";
    os << "\n";
  }

  friend std::ostream& operator<<(std::ostream& os, const VisualizationSettings& object)
  {
    object.Print(os);
    return os;
  }

};


