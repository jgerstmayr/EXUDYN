


**********************
Visualization settings
**********************

This section includes hierarchical structures for visualization settings, e.g., drawing of nodes, bodies, connectors, loads and markers and furthermore openGL, window and save image options. For further information, see Section :ref:`sec-overview-basics-visualizationsettings`\ .


.. _sec-vsettingsgeneral:

VSettingsGeneral
----------------

General settings for visualization.

VSettingsGeneral has the following items:

* | **autoFitScene** [type = bool, default = True]:
  | automatically fit scene within startup after StartRenderer()
* | **axesTiling** [type = PInt, default = 12]:
  | global number of segments for drawing axes cylinders and cones (reduce this number, e.g. to 4, if many axes are drawn)
* | **backgroundColor** [type = Float4, default = [1.0,1.0,1.0,1.0], size = 4]:
  | \tabnewline red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
* | **backgroundColorBottom** [type = Float4, default = [0.8,0.8,1.0,1.0], size = 4]:
  | \tabnewline red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
* | **circleTiling** [type = PInt, default = 16]:
  | global number of segments for circles; if smaller than 2, 2 segments are used (flat)
* | **coordinateSystemSize** [type = float, default = 5.]:
  | size of coordinate system relative to font size
* | **cylinderTiling** [type = PInt, default = 16]:
  | global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
* | **drawCoordinateSystem** [type = bool, default = True]:
  | false = no coordinate system shown
* | **drawWorldBasis** [type = bool, default = False]:
  | true = draw world basis coordinate system at (0,0,0)
* | **graphicsUpdateInterval** [type = float, default = 0.1]:
  | interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
* | **minSceneSize** [type = float, default = 0.1]:
  | minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
* | **pointSize** [type = float, default = 0.01]:
  | global point size (absolute)
* | **rendererPrecision** [type = PInt, default = 4]:
  | precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
* | **renderWindowString** [type = String, default = '']:
  | string shown in render window (use this, e.g., for debugging, etc.; written below EXUDYN, similar to solutionInformation in SimulationSettings.solutionSettings)
* | **showComputationInfo** [type = bool, default = True]:
  | true = show (hide) all computation information including Exudyn and version
* | **showHelpOnStartup** [type = PInt, default = 5]:
  | seconds to show help message on startup (0=deactivate)
* | **showSolutionInformation** [type = bool, default = True]:
  | true = show solution information (from simulationSettings.solution)
* | **showSolverInformation** [type = bool, default = True]:
  | true = solver name and further information shown in render window
* | **showSolverTime** [type = bool, default = True]:
  | true = solver current time shown in render window
* | **sphereTiling** [type = PInt, default = 6]:
  | global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
* | **textAlwaysInFront** [type = bool, default = True]:
  | if true, text for item numbers and other item-related text is drawn in front; this may be unwanted in case that you only with to see numbers of objects in front; currently does not work with perspective
* | **textColor** [type = Float4, default = [0.,0.,0.,1.0], size = 4]:
  | \tabnewline general text color (default); used for system texts in render window
* | **textHasBackground** [type = bool, default = False]:
  | if true, text for item numbers and other item-related text have a background (depending on text color), allowing for better visibility if many numbers are shown; the text itself is black; therefore, dark background colors are ignored and shown as white
* | **textOffsetFactor** [type = UFloat, default = 0.005]:
  | This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes
* | **textSize** [type = float, default = 12.]:
  | general text size (font size) in pixels if not overwritten; if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
* | **threadSafeGraphicsUpdate** [type = bool, default = True]:
  | true = updating of visualization is threadsafe, but slower for complicated models; deactivate this to speed up computation, but activate for generation of animations; may be improved in future by adding a safe visualizationUpdate state
* | **useBitmapText** [type = bool, default = True]:
  | if true, texts are displayed using pre-defined bitmaps for the text; may increase the complexity of your scene, e.g., if many (>10000) node numbers shown
* | **useGradientBackground** [type = bool, default = False]:
  | true = use vertical gradient for background; 
* | **useMultiThreadedRendering** [type = bool, default = True]:
  | true = rendering is done in separate thread; false = no separate thread, which may be more stable but has lagging interaction for large models (do not interact with models during simulation); set this parameter before call to exudyn.StartRenderer(); MAC OS: uses always false, because MAC OS does not support multi threaded GLFW
* | **useWindowsDisplayScaleFactor** [type = bool, default = True]:
  | the Windows display scaling (monitor scaling; content scaling) factor is used for increased visibility of texts on high resolution displays; based on GLFW glfwGetWindowContentScale; deactivated on linux compilation as it leads to crashes (adjust textSize manually!)
* | **worldBasisSize** [type = float, default = 1.0]:
  | size of world basis coordinate system



.. _sec-vsettingscontour:

VSettingsContour
----------------

Settings for contour plots; use these options to visualize field data, such as displacements, stresses, strains, etc. for bodies, nodes and finite elements.

VSettingsContour has the following items:

* | **automaticRange** [type = bool, default = True]:
  | if true, the contour plot value range is chosen automatically to the maximum range
* | **colorBarPrecision** [type = PInt, default = 4]:
  | precision of floating point values shown in color bar; total number of digits used (max. 16)
* | **colorBarTiling** [type = PInt, default = 12, size = 1]:
  | number of tiles (segements) shown in the colorbar for the contour plot
* | **maxValue** [type = float, default = 1, size = 1]:
  | maximum value for contour plot; set manually, if automaticRange == False
* | **minValue** [type = float, default = 0, size = 1]:
  | minimum value for contour plot; set manually, if automaticRange == False
* | **nodesColored** [type = bool, default = True]:
  | if true, the contour color is also applied to nodes (except mesh nodes), otherwise node drawing is not influenced by contour settings
* | **outputVariable** [type = OutputVariableType, default = OutputVariableType::\_None]:
  | \tabnewline selected contour plot output variable type; select OutputVariableType._None to deactivate contour plotting.
* | **outputVariableComponent** [type = Int, default = 0, size = 1]:
  | select the component of the chosen output variable; e.g., for displacements, 3 components are available: 0 == x, 1 == y, 2 == z component; for stresses, 6 components are available, see OutputVariableType description; to draw the norm of a outputVariable, set component to -1; if a certain component is not available by certain objects or nodes, no value is drawn (using default color)
* | **reduceRange** [type = bool, default = True]:
  | if true, the contour plot value range is also reduced; better for static computation; in dynamic computation set this option to false, it can reduce visualization artifacts; you should also set minVal to max(float) and maxVal to min(float)
* | **rigidBodiesColored** [type = bool, default = True]:
  | if true, the contour color is also applied to triangular faces of rigid bodies and mass points, otherwise the rigid body drawing are not influenced by contour settings; for general rigid bodies (except for ObjectGround), Position, Displacement, DisplacementLocal(=0), Velocity, VelocityLocal, AngularVelocity, and AngularVelocityLocal are available; may slow down visualization!
* | **showColorBar** [type = bool, default = True]:
  | show the colour bar with minimum and maximum values for the contour plot



.. _sec-vsettingsnodes:

VSettingsNodes
--------------

Visualization settings for nodes.

VSettingsNodes has the following items:

* | **basisSize** [type = float, default = 0.2]:
  | size of basis for nodes
* | **defaultColor** [type = Float4, default = [0.2,0.2,1.,1.], size = 4]:
  | \tabnewline default cRGB color for nodes; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = -1.]:
  | global node size; if -1.f, node size is relative to openGL.initialMaxSceneSize
* | **drawNodesAsPoint** [type = bool, default = True]:
  | simplified/faster drawing of nodes; uses general->pointSize as drawing size; if drawNodesAsPoint==True, the basis of the node will be drawn with lines
* | **show** [type = bool, default = True]:
  | flag to decide, whether the nodes are shown
* | **showBasis** [type = bool, default = False]:
  | show basis (three axes) of coordinate system in 3D nodes
* | **showNodalSlopes** [type = UInt, default = False]:
  | draw nodal slope vectors, e.g. in ANCF beam finite elements
* | **showNumbers** [type = bool, default = False]:
  | flag to decide, whether the node number is shown
* | **tiling** [type = PInt, default = 4]:
  | tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4



.. _sec-vsettingsbeams:

VSettingsBeams
--------------

Visualization settings for beam finite elements.

VSettingsBeams has the following items:

* | **axialTiling** [type = PInt, default = 8]:
  | number of segments to discretise the beams axis
* | **crossSectionFilled** [type = bool, default = True]:
  | if implemented for element, cross section is drawn as solid (filled) instead of wire-frame; NOTE: some quantities may not be interpolated correctly over cross section in visualization
* | **crossSectionTiling** [type = PInt, default = 4]:
  | number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing
* | **drawVertical** [type = bool, default = False]:
  | draw contour plot outputVariables 'vertical' along beam height; contour.outputVariable must be set accordingly
* | **drawVerticalColor** [type = Float4, default = [0.2,0.2,0.2,1.], size = 4]:
  | \tabnewline color for outputVariable to be drawn along cross section (vertically)
* | **drawVerticalFactor** [type = float, default = 1.]:
  | factor for outputVariable to be drawn along cross section (vertically)
* | **drawVerticalLines** [type = bool, default = True]:
  | draw additional vertical lines for better visibility
* | **drawVerticalOffset** [type = float, default = 0.]:
  | offset for vertical drawn lines; offset is added before multiplication with drawVerticalFactor
* | **drawVerticalValues** [type = bool, default = False]:
  | show values at vertical lines; note that these numbers are interpolated values and may be different from values evaluated directly at this point!
* | **reducedAxialInterploation** [type = bool, default = True]:
  | if True, the interpolation along the beam axis may be lower than the beam element order; this may be, however, show more consistent values than a full interpolation, e.g. for strains or forces



.. _sec-vsettingskinematictree:

VSettingsKinematicTree
----------------------

Visualization settings for kinematic trees.

VSettingsKinematicTree has the following items:

* | **frameSize** [type = float, default = 0.2]:
  | size of COM and joint frames
* | **showCOMframes** [type = bool, default = False]:
  | if True, a frame is attached to every center of mass
* | **showFramesNumbers** [type = bool, default = True]:
  | if True, numbers are drawn for joint frames (O[i]J[j]) and COM frames (O[i]COM[j]) for object [i] and local joint [j]
* | **showJointFrames** [type = bool, default = True]:
  | if True, a frame is attached to the origin of every joint frame



.. _sec-vsettingsbodies:

VSettingsBodies
---------------

Visualization settings for bodies.

VSettingsBodies has the following items:

* | **beams** [type = VSettingsBeams]:
  | visualization settings for beams (e.g. ANCFCable or other beam elements)
* | **kinematicTree** [type = VSettingsKinematicTree]:
  | visualization settings for kinematic tree
* | **defaultColor** [type = Float4, default = [0.3,0.3,1.,1.], size = 4]:
  | \tabnewline default cRGB color for bodies; 4th value is 
* | **defaultSize** [type = Float3, default = [1.,1.,1.], size = 3]:
  | \tabnewline global body size of xyz-cube
* | **deformationScaleFactor** [type = float, default = 1]:
  | global deformation scale factor; also applies to nodes, if drawn; used for scaled drawing of (linear) finite elements, beams, etc.
* | **show** [type = bool, default = True]:
  | flag to decide, whether the bodies are shown
* | **showNumbers** [type = bool, default = False]:
  | flag to decide, whether the body(=object) number is shown



.. _sec-vsettingsconnectors:

VSettingsConnectors
-------------------

Visualization settings for connectors.

VSettingsConnectors has the following items:

* | **contactPointsDefaultSize** [type = float, default = 0.02]:
  | DEPRECATED: do not use! global contact points size; if -1.f, connector size is relative to maxSceneSize
* | **defaultColor** [type = Float4, default = [0.2,0.2,1.,1.], size = 4]:
  | \tabnewline default cRGB color for connectors; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = 0.1]:
  | global connector size; if -1.f, connector size is relative to maxSceneSize
* | **jointAxesLength** [type = float, default = 0.2]:
  | global joint axes length
* | **jointAxesRadius** [type = float, default = 0.02]:
  | global joint axes radius
* | **show** [type = bool, default = True]:
  | flag to decide, whether the connectors are shown
* | **showContact** [type = bool, default = False]:
  | flag to decide, whether contact points, lines, etc. are shown
* | **showJointAxes** [type = bool, default = False]:
  | flag to decide, whether contact joint axes of 3D joints are shown
* | **showNumbers** [type = bool, default = False]:
  | flag to decide, whether the connector(=object) number is shown
* | **springNumberOfWindings** [type = PInt, default = 8]:
  | number of windings for springs drawn as helical spring



.. _sec-vsettingsmarkers:

VSettingsMarkers
----------------

Visualization settings for markers.

VSettingsMarkers has the following items:

* | **defaultColor** [type = Float4, default = [0.1,0.5,0.1,1.], size = 4]:
  | \tabnewline default cRGB color for markers; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = -1.]:
  | global marker size; if -1.f, marker size is relative to maxSceneSize
* | **drawSimplified** [type = bool, default = True]:
  | draw markers with simplified symbols
* | **show** [type = bool, default = True]:
  | flag to decide, whether the markers are shown
* | **showNumbers** [type = bool, default = False]:
  | flag to decide, whether the marker numbers are shown



.. _sec-vsettingsloads:

VSettingsLoads
--------------

Visualization settings for loads.

VSettingsLoads has the following items:

* | **defaultColor** [type = Float4, default = [0.7,0.1,0.1,1.], size = 4]:
  | \tabnewline default cRGB color for loads; 4th value is alpha-transparency
* | **defaultRadius** [type = float, default = 0.005]:
  | global radius of load axis if drawn in 3D
* | **defaultSize** [type = float, default = 0.2]:
  | global load size; if -1.f, load size is relative to maxSceneSize
* | **drawSimplified** [type = bool, default = True]:
  | draw markers with simplified symbols
* | **fixedLoadSize** [type = bool, default = True]:
  | if true, the load is drawn with a fixed vector length in direction of the load vector, independently of the load size
* | **loadSizeFactor** [type = float, default = 0.1]:
  | if fixedLoadSize=false, then this scaling factor is used to draw the load vector
* | **show** [type = bool, default = True]:
  | flag to decide, whether the loads are shown
* | **showNumbers** [type = bool, default = False]:
  | flag to decide, whether the load numbers are shown



.. _sec-vsettingssensors:

VSettingsSensors
----------------

Visualization settings for sensors.

VSettingsSensors has the following items:

* | **defaultColor** [type = Float4, default = [0.6,0.6,0.1,1.], size = 4]:
  | \tabnewline default cRGB color for sensors; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = -1.]:
  | global sensor size; if -1.f, sensor size is relative to maxSceneSize
* | **drawSimplified** [type = bool, default = True]:
  | draw sensors with simplified symbols
* | **show** [type = bool, default = True]:
  | flag to decide, whether the sensors are shown
* | **showNumbers** [type = bool, default = False]:
  | flag to decide, whether the sensor numbers are shown



.. _sec-vsettingscontact:

VSettingsContact
----------------

Global visualization settings for GeneralContact. This allows to easily switch on/off during visualization. 

VSettingsContact has the following items:

* | **colorBoundingBoxes** [type = Float4, default = [0.9,0.1,0.1,1.], size = 4]:
  | \tabnewline cRGB color
* | **colorSearchTree** [type = Float4, default = [0.1,0.1,0.9,1.], size = 4]:
  | \tabnewline cRGB color
* | **contactForcesFactor** [type = float, default = 0.001]:
  | factor used for scaling of contact forces is showContactForces=True
* | **contactPointsDefaultSize** [type = float, default = 0.001]:
  | global contact points size; if -1.f, connector size is relative to maxSceneSize; used for some contacts, e.g., in ContactFrictionCircle
* | **showBoundingBoxes** [type = bool, default = False]:
  | show bounding boxes of all GeneralContacts
* | **showContactForces** [type = bool, default = False]:
  | if True, contact forces are drawn for certain contact models
* | **showContactForcesValues** [type = bool, default = False]:
  | if True and showContactForces=True, numerical values for  contact forces are shown at certain points
* | **showSearchTree** [type = bool, default = False]:
  | show search tree of all GeneralContacts
* | **showSearchTreeCells** [type = bool, default = False]:
  | show cells inside search tree



.. _sec-vsettingswindow:

VSettingsWindow
---------------

OpenGL Window and interaction settings for visualization; handle changes with care, as they might lead to unexpected results or crashes.

VSettingsWindow has the following items:

* | **alwaysOnTop** [type = bool, default = False]:
  | True: OpenGL render window will be always on top of all other windows
* | **ignoreKeys** [type = bool, default = False]:
  | True: ignore keyboard input except escape and 'F2' keys; used for interactive mode, e.g., to perform kinematic analysis; This flag can be switched with key 'F2'
* | **keyPressUserFunction** [type = KeyPressUserFunction, default = 0]:
  | add a Python function f(key, action, mods) here, which is called every time a key is pressed; function shall return true, if key has been processed; Example: \tabnewline def f(key, action, mods):\tabnewline \phantomXXX print('key=',key);\tabnewline use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
* | **limitWindowToScreenSize** [type = bool, default = True]:
  | True: render window size is limited to screen size; False: larger window sizes (e.g. for rendering) allowed according to renderWindowSize
* | **maximize** [type = bool, default = False]:
  | True: OpenGL render window will be maximized at startup
* | **renderWindowSize** [type = Index2, default = [1024,768], size = 2]:
  | initial size of OpenGL render window in pixel
* | **showMouseCoordinates** [type = bool, default = False]:
  | True: show OpenGL coordinates and distance to last left mouse button pressed position; switched on/off with key 'F3'
* | **showWindow** [type = bool, default = True]:
  | True: OpenGL render window is shown on startup; False: window will be iconified at startup (e.g. if you are starting multiple computations automatically)
* | **startupTimeout** [type = PInt, default = 2500]:
  | OpenGL render window startup timeout in ms (change might be necessary if CPU is very slow)



.. _sec-vsettingsdialogs:

VSettingsDialogs
----------------

Settings related to dialogs (e.g., visualization settings dialog).

VSettingsDialogs has the following items:

* | **alphaTransparency** [type = UFloat, default = 0.94]:
  | alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)
* | **alwaysTopmost** [type = bool, default = True]:
  | True: dialogs are always topmost (otherwise, they are sometimes hidden)
* | **fontScalingMacOS** [type = UFloat, default = 1.35]:
  | font scaling value for MacOS systems (on Windows, system display scaling is used)
* | **multiThreadedDialogs** [type = bool, default = True]:
  | True: During dialogs, the OpenGL render window will still get updates of changes in dialogs, etc., which may cause problems on some platforms or for some (complicated) models; False: changes of dialogs will take effect when dialogs are closed
* | **openTreeView** [type = bool, default = False]:
  | True: all sub-trees of the visusalization dialog are opened when opening the dialog; False: only some sub-trees are opened



.. _sec-vsettingsopengl:

VSettingsOpenGL
---------------

OpenGL settings for 2D and 2D rendering. For further details, see the OpenGL functionality. 

VSettingsOpenGL has the following items:

* | **drawFaceNormals** [type = bool, default = False, size = 1]:
  | draws triangle normals, e.g. at center of triangles; used for debugging of faces
* | **drawNormalsLength** [type = PFloat, default = 0.1, size = 1]:
  | length of normals; used for debugging
* | **drawVertexNormals** [type = bool, default = False, size = 1]:
  | draws vertex normals; used for debugging
* | **enableLight0** [type = bool, default = True, size = 1]:
  | turn on/off light0
* | **enableLight1** [type = bool, default = True, size = 1]:
  | turn on/off light1
* | **enableLighting** [type = bool, default = True, size = 1]:
  | generally enable lighting (otherwise, colors of objects are used); OpenGL: glEnable(GL_LIGHTING)
* | **faceEdgesColor** [type = Float4, default = [0.2,0.2,0.2,1.], size = 4]:
  | \tabnewline global RGBA color for face edges
* | **facesTransparent** [type = bool, default = False, size = 1]:
  | True: show faces transparent independent of transparency (A)-value in color of objects; allow to show otherwise hidden node/marker/object numbers
* | **initialCenterPoint** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \tabnewline centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True
* | **initialMaxSceneSize** [type = PFloat, default = 1.]:
  | initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
* | **initialModelRotation** [type = StdArray33F, default = [Matrix3DF[3,3,1.,0.,0., 0.,1.,0., 0.,0.,1.]], size = 3x3]:
  | \tabnewline initial model rotation matrix for OpenGl; in python use e.g.: initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]
* | **initialZoom** [type = UFloat, default = 1.]:
  | initial zoom of scene; overwritten/ignored if autoFitScene = True
* | **light0ambient** [type = float, default = 0.3, size = 1]:
  | ambient value of GL_LIGHT0
* | **light0constantAttenuation** [type = float, default = 1.0, size = 1]:
  | constant attenuation coefficient of GL_LIGHT0, this is a constant factor that attenuates the light source; attenuation factor = 1/(kx +kl*d + kq*d*d); (kc,kl,kq)=(1,0,0) means no attenuation; only used for lights, where last component of light position is 1
* | **light0diffuse** [type = float, default = 0.6, size = 1]:
  | diffuse value of GL_LIGHT0
* | **light0linearAttenuation** [type = float, default = 0.0, size = 1]:
  | linear attenuation coefficient of GL_LIGHT0, this is a linear factor for attenuation of the light source with distance
* | **light0position** [type = Float4, default = [0.2,0.2,10.,0.], size = 4]:
  | \tabnewline 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position; see opengl manuals
* | **light0quadraticAttenuation** [type = float, default = 0.0, size = 1]:
  | quadratic attenuation coefficient of GL_LIGHT0, this is a quadratic factor for attenuation of the light source with distance
* | **light0specular** [type = float, default = 0.5, size = 1]:
  | specular value of GL_LIGHT0
* | **light1ambient** [type = float, default = 0.0 , size = 1]:
  | ambient value of GL_LIGHT1
* | **light1constantAttenuation** [type = float, default = 1.0, size = 1]:
  | constant attenuation coefficient of GL_LIGHT1, this is a constant factor that attenuates the light source; attenuation factor = 1/(kx +kl*d + kq*d*d); only used for lights, where last component of light position is 1
* | **light1diffuse** [type = float, default = 0.5, size = 1]:
  | diffuse value of GL_LIGHT1
* | **light1linearAttenuation** [type = float, default = 0.0, size = 1]:
  | linear attenuation coefficient of GL_LIGHT1, this is a linear factor for attenuation of the light source with distance
* | **light1position** [type = Float4, default = [1.,1.,-10.,0.], size = 4]:
  | \tabnewline 4f position vector of GL_LIGHT0; 4th value should be 0 for lights like sun, but 1 for directional lights (and for attenuation factor being calculated); see opengl manuals
* | **light1quadraticAttenuation** [type = float, default = 0.0, size = 1]:
  | quadratic attenuation coefficient of GL_LIGHT1, this is a quadratic factor for attenuation of the light source with distance
* | **light1specular** [type = float, default = 0.6, size = 1]:
  | specular value of GL_LIGHT1
* | **lightModelAmbient** [type = Float4, default = [0.,0.,0.,1.], size = 4]:
  | \tabnewline global ambient light; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a])
* | **lightModelLocalViewer** [type = bool, default = False, size = 1]:
  | select local viewer for light; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,...)
* | **lightModelTwoSide** [type = bool, default = False, size = 1]:
  | enlighten also backside of object; may cause problems on some graphics cards and lead to slower performance; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,...)
* | **lineSmooth** [type = bool, default = True, size = 1]:
  | draw lines smooth
* | **lineWidth** [type = UFloat, default = 1., size = 1]:
  | width of lines used for representation of lines, circles, points, etc.
* | **materialAmbientAndDiffuse** [type = Float4, default = [0.6,0.6,0.6,1.], size = 4]:
  | \tabnewline 4f ambient color of material
* | **materialShininess** [type = float, default = 32., size = 1]:
  | shininess of material
* | **materialSpecular** [type = Float4, default = [0.6,0.6,0.6,1.], size = 4]:
  | \tabnewline 4f specular color of material
* | **multiSampling** [type = PInt, default = 1, size = 1]:
  | NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
* | **perspective** [type = UFloat, default = 0.]:
  | parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 0.5 (large amount of perspective); mouse coordinates will not work with perspective
* | **polygonOffset** [type = float, default = 0.01]:
  | general polygon offset for polygons, except for shadows; use this parameter to draw polygons behind lines to reduce artifacts for very large or small models
* | **shadeModelSmooth** [type = bool, default = True, size = 1]:
  | True: turn on smoothing for shaders, which uses vertex normals to smooth surfaces
* | **shadow** [type = UFloat, default = 0.]:
  | parameter \ :math:`\in [0 ... 1]`\  prescribes amount of shadow for light0 (using light0position, etc.); if this parameter is different from 1, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows
* | **shadowPolygonOffset** [type = PFloat, default = 0.1]:
  | some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
* | **showFaceEdges** [type = bool, default = False, size = 1]:
  | show edges of faces; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
* | **showFaces** [type = bool, default = True, size = 1]:
  | show faces of triangles, etc.; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
* | **showLines** [type = bool, default = True, size = 1]:
  | show lines (different from edges of faces)
* | **showMeshEdges** [type = bool, default = True, size = 1]:
  | show edges of finite elements; independent of showFaceEdges
* | **showMeshFaces** [type = bool, default = True, size = 1]:
  | show faces of finite elements; independent of showFaces
* | **textLineSmooth** [type = bool, default = False, size = 1]:
  | draw lines for representation of text smooth
* | **textLineWidth** [type = UFloat, default = 1., size = 1]:
  | width of lines used for representation of text



.. _sec-vsettingsexportimages:

VSettingsExportImages
---------------------

Functionality to export images to files (PNG or TGA format) which can be used to create animations; to activate image recording during the solution process, set SolutionSettings.recordImagesInterval accordingly.

VSettingsExportImages has the following items:

* | **heightAlignment** [type = PInt, default = 2]:
  | alignment of exported image height; using a value of 2 helps to reduce problems with video conversion (additional horizontal lines are lost)
* | **saveImageAsTextCircles** [type = bool, default = True]:
  | export circles in save image (only in TXT format)
* | **saveImageAsTextLines** [type = bool, default = True]:
  | export lines in save image (only in TXT format)
* | **saveImageAsTextTexts** [type = bool, default = False]:
  | export text in save image (only in TXT format)
* | **saveImageAsTextTriangles** [type = bool, default = False]:
  | export triangles in save image (only in TXT format)
* | **saveImageFileCounter** [type = UInt, default = 0]:
  | current value of the counter which is used to consecutively save frames (images) with consecutive numbers
* | **saveImageFileName** [type = FileName, default = 'images/frame']:
  | filename (without extension!) and (relative) path for image file(s) with consecutive numbering (e.g., frame0000.png, frame0001.png,...); ; directory will be created if it does not exist
* | **saveImageFormat** [type = String, default = 'PNG']:
  | format for exporting figures: currently only PNG, TGA and TXT available; while PNG and TGA represent the according image file formats, the TXT format results in a text file containing the 3D graphics data information as lists of lines, triangles, etc; PNG is not available for Ubuntu18.04 (check  use TGA has highest compatibility with all platforms
* | **saveImageSingleFile** [type = bool, default = False]:
  | True: only save single files with given filename, not adding numbering; False: add numbering to files, see saveImageFileName
* | **saveImageTimeOut** [type = PInt, default = 5000]:
  | timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
* | **widthAlignment** [type = PInt, default = 4]:
  | alignment of exported image width; using a value of 4 helps to reduce problems with video conversion (additional vertical lines are lost)



.. _sec-vsettingsopenvr:

VSettingsOpenVR
---------------

Functionality to interact openVR; requires special hardware or software emulator, see steam / openVR descriptions. 

VSettingsOpenVR has the following items:

* | **actionManifestFileName** [type = FileName, default = 'C:/openVRactionsManifest.json']:
  | \tabnewline This string must contain a string representing a valid absolute path to a vr_actions.json manifest, which describes all HMD, tracker, etc. devices as given by openVR
* | **enable** [type = bool, default = False]:
  | True: openVR enabled (if compiled with according flag and installed openVR)
* | **logLevel** [type = Int, default = 1]:
  | integer value setting log level of openVR: -1 (no output), 0 (error), 1 (warning), 2 (info), 3 (debug); increase log level to get more output
* | **showCompanionWindow** [type = bool, default = True]:
  | True: openVR will show companion window containing left and right eye view



.. _sec-vsettingsinteractive:

VSettingsInteractive
--------------------

Functionality to interact with render window; will include left and right mouse press actions and others in future.

VSettingsInteractive has the following items:

* | **openVR** [type = VSettingsOpenVR]:
  | openVR visualization settings
* | **highlightColor** [type = Float4, default = [0.8,0.05,0.05,0.75], size = 4]:
  | \tabnewline cRGB color for highlighted item; 4th value is alpha-transparency
* | **highlightItemIndex** [type = Int, default = -1]:
  | index of item that shall be highlighted (e.g., need to find item due to errors); if set -1, no item is highlighted
* | **highlightItemType** [type = ItemType, default = ItemType::\_None]:
  | item type (Node, Object, ...) that shall be highlighted (e.g., need to find item due to errors)
* | **highlightMbsNumber** [type = UInt, default = 0]:
  | index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
* | **highlightOtherColor** [type = Float4, default = [0.5,0.5,0.5,0.4], size = 4]:
  | \tabnewline cRGB color for other items (which are not highlighted); 4th value is alpha-transparency
* | **joystickScaleRotation** [type = float, default = 200.]:
  | rotation scaling factor for joystick input
* | **joystickScaleTranslation** [type = float, default = 6.]:
  | translation scaling factor for joystick input
* | **keypressRotationStep** [type = float, default = 5.]:
  | rotation increment per keypress in degree (full rotation = 360 degree)
* | **keypressTranslationStep** [type = float, default = 0.1]:
  | translation increment per keypress relative to window size
* | **lockModelView** [type = bool, default = False]:
  | True: all movements (with mouse/keys), rotations, zoom are disabled; initial values are considered ==> initial zoom, rotation and center point need to be adjusted, approx. 0.4*maxSceneSize is a good value
* | **mouseMoveRotationFactor** [type = float, default = 1.]:
  | rotation increment per 1 pixel mouse movement in degree
* | **pauseWithSpacebar** [type = bool, default = True]:
  | True: during simulation, space bar can be pressed to pause simulation
* | **selectionHighlights** [type = bool, default = True]:
  | True: mouse click highlights item (default: red)
* | **selectionLeftMouse** [type = bool, default = True]:
  | True: left mouse click on items and show basic information
* | **selectionRightMouse** [type = bool, default = True]:
  | True: right mouse click on items and show dictionary (read only!)
* | **selectionRightMouseGraphicsData** [type = \tabnewline bool, default = False]:
  | True: right mouse click on items also shows GraphicsData information for inspectation (may sometimes be very large and may not fit into dialog for large graphics objects!)
* | **trackMarker** [type = Int, default = -1]:
  | if valid marker index is provided and marker provides position (and orientation), the centerpoint of the scene follows the marker (and orientation); depends on trackMarkerPosition and trackMarkerOrientation; by default, only position is tracked
* | **trackMarkerMbsNumber** [type = Index, default = 0]:
  | number of main system which is used to track marker; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number
* | **trackMarkerOrientation** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \tabnewline choose which orientation axes (x,y,z) are tracked; currently can only be all zero or all one
* | **trackMarkerPosition** [type = Float3, default = [1.,1.,1.], size = 3]:
  | \tabnewline choose which coordinates or marker are tracked (x,y,z)
* | **useJoystickInput** [type = bool, default = True]:
  | True: read joystick input (use 6-axis joystick with lowest ID found when starting renderer window) and interpret as (x,y,z) position and (rotx, roty, rotz) rotation: as available from 3Dconnexion space mouse and maybe others as well; set to False, if external joystick makes problems ...
* | **zoomStepFactor** [type = float, default = 1.15]:
  | change of zoom per keypress (keypad +/-) or mouse wheel increment



.. _sec-visualizationsettings:

VisualizationSettings
---------------------

Settings for visualization. 

VisualizationSettings has the following items:

* | **bodies** [type = VSettingsBodies]:
  | body visualization settings
* | **connectors** [type = VSettingsConnectors]:
  | connector visualization settings
* | **contact** [type = VSettingsContact]:
  | contact visualization settings
* | **contour** [type = VSettingsContour]:
  | contour plot visualization settings
* | **dialogs** [type = VSettingsDialogs]:
  | dialogs settings
* | **exportImages** [type = VSettingsExportImages]:
  | settings for exporting (saving) images to files in order to create animations
* | **general** [type = VSettingsGeneral]:
  | general visualization settings
* | **interactive** [type = VSettingsInteractive]:
  | Settings for interaction with renderer
* | **loads** [type = VSettingsLoads]:
  | load visualization settings
* | **markers** [type = VSettingsMarkers]:
  | marker visualization settings
* | **nodes** [type = VSettingsNodes]:
  | node visualization settings
* | **openGL** [type = VSettingsOpenGL]:
  | OpenGL rendering settings
* | **sensors** [type = VSettingsSensors]:
  | sensor visualization settings
* | **window** [type = VSettingsWindow]:
  | visualization window and interaction settings

