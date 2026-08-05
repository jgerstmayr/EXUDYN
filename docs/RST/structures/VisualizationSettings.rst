


.. _sec-visualizationsettingsmain:


**********************
Visualization settings
**********************

This section includes hierarchical structures for visualization settings, e.g., drawing of nodes, bodies, connectors, loads and markers and furthermore openGL, window and save image options. For further information, see Section :ref:`sec-overview-basics-visualizationsettings`\ .


.. _sec-vsettingsgeneral:

VSettingsGeneral
----------------

General settings for visualization that influence all windows, default values, autofit, multithreading, etc.

VSettingsGeneral has the following items:

* | **autoFitScene** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.autoFitScene``\ 
  | automatically fit scene within startup after SC.renderer.Start()
* | **axesTiling** [type = PInt, default = 12]:
  | \ ``SC.visualizationSettings.general.axesTiling``\ 
  | global number of segments for drawing cylinders for axes and cones for arrows (reduce this number, e.g. to 4, if many axes are drawn)
* | **backgroundColor** [type = Float4, default = [1.0,1.0,1.0,1.0], size = 4]:
  | \ ``SC.visualizationSettings.general.backgroundColor``\ 
  | red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])
* | **backgroundColorBottom** [type = Float4, default = [0.8,0.8,1.0,1.0], size = 4]:
  | \ ``SC.visualizationSettings.general.backgroundColorBottom``\ 
  | red, green, blue and alpha values for bottom background color in case that useGradientBackground = True
* | **boundingBoxZoomAllFactor** [type = PFloat, default = 1.2]:
  | \ ``SC.visualizationSettings.general.boundingBoxZoomAllFactor``\ 
  | factor on boundingBox for zoom all (without minimum offset)
* | **boundingBoxZoomAllOffset** [type = UFloat, default = 0.01]:
  | \ ``SC.visualizationSettings.general.boundingBoxZoomAllOffset``\ 
  | minimum offset to bounding box of scene in window - width or height, whatever is smaller; adjust for very small or large scenes; may be negative
* | **circleTiling** [type = PInt, default = 16]:
  | \ ``SC.visualizationSettings.general.circleTiling``\ 
  | global number of segments for circles; if smaller than 2, 2 segments are used (flat)
* | **coordinateSystemSize** [type = PFloat, default = 5.]:
  | \ ``SC.visualizationSettings.general.coordinateSystemSize``\ 
  | size of coordinate system relative to font size
* | **cylinderTiling** [type = PInt, default = 16]:
  | \ ``SC.visualizationSettings.general.cylinderTiling``\ 
  | global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)
* | **graphicsUpdateInterval** [type = UFloat, default = 0.1]:
  | \ ``SC.visualizationSettings.general.graphicsUpdateInterval``\ 
  | interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed
* | **limitWindowToScreenSize** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.limitWindowToScreenSize``\ 
  | True: size for render window of respective view is limited to screen size; False: larger window sizes (e.g. for rendering) allowed according to renderWindowSize
* | **linuxDisplayScaleFactor** [type = PFloat, default = 1.]:
  | \ ``SC.visualizationSettings.general.linuxDisplayScaleFactor``\ 
  | Scaling factor for linux, which cannot determined from system by now; adjust this value to scale dialog fonts and renderer fonts
* | **minSceneSize** [type = PFloat, default = 0.1]:
  | \ ``SC.visualizationSettings.general.minSceneSize``\ 
  | minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO
* | **pointSize** [type = PFloat, default = 0.01]:
  | \ ``SC.visualizationSettings.general.pointSize``\ 
  | global point size (absolute)
* | **reallyQuitTimeLimit** [type = UReal, default = 900]:
  | \ ``SC.visualizationSettings.general.reallyQuitTimeLimit``\ 
  | number of seconds after which user is asked a security question before stopping simulation and closing renderer; set to 0 in order to always get asked; set to 1e10 to (nearly) never get asked
* | **rendererPrecision** [type = PInt, default = 4]:
  | \ ``SC.visualizationSettings.general.rendererPrecision``\ 
  | precision of general floating point numbers shown in render window: total number of digits used  (max. 16)
* | **rendererStartupTimeout** [type = PInt, default = 2500]:
  | \ ``SC.visualizationSettings.general.rendererStartupTimeout``\ 
  | OpenGL render windows startup timeout in ms (change might be necessary if CPU is very slow)
* | **renderWindowString** [type = String, default = '']:
  | \ ``SC.visualizationSettings.general.renderWindowString``\ 
  | string shown in render window (use this, e.g., for debugging, etc.; written below EXUDYN, similar to solutionInformation in SimulationSettings.solutionSettings)
* | **showHelpOnStartup** [type = UInt, default = 5]:
  | \ ``SC.visualizationSettings.general.showHelpOnStartup``\ 
  | seconds to show help message on startup (0=deactivate)
* | **showSolutionInformation** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.showSolutionInformation``\ 
  | true = show solution information (from simulationSettings.solution)
* | **showSolverInformation** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.showSolverInformation``\ 
  | true = solver name and further information shown in render window
* | **showSolverTime** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.showSolverTime``\ 
  | true = solver current time shown in render window
* | **sphereTiling** [type = PInt, default = 6]:
  | \ ``SC.visualizationSettings.general.sphereTiling``\ 
  | global number of segments for spheres; if smaller than 2, 2 segments are used (flat)
* | **textAlwaysInFront** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.textAlwaysInFront``\ 
  | if true, text for item numbers and other item-related text is drawn in front; this may be unwanted in case that you only with to see numbers of objects in front; currently does not work with perspective
* | **textColor** [type = Float4, default = [0.,0.,0.,1.0], size = 4]:
  | \ ``SC.visualizationSettings.general.textColor``\ 
  | general text color (default); used for system texts in render window
* | **textHasBackground** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.general.textHasBackground``\ 
  | if true, text for item numbers and other item-related text have a background (depending on text color), allowing for better visibility if many numbers are shown; the text itself is black; therefore, dark background colors are ignored and shown as white
* | **textOffsetFactor** [type = UFloat, default = 0.005]:
  | \ ``SC.visualizationSettings.general.textOffsetFactor``\ 
  | This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes
* | **threadSafeGraphicsUpdate** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.threadSafeGraphicsUpdate``\ 
  | true = updating of visualization is threadsafe, but slower for complicated models; deactivate this to speed up computation, but activate for generation of animations; may be improved in future by adding a safe visualizationUpdate state
* | **useBitmapText** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.useBitmapText``\ 
  | if true, texts are displayed using pre-defined bitmaps for the text; may increase the complexity of your scene, e.g., if many (>10000) node numbers shown
* | **useGradientBackground** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.general.useGradientBackground``\ 
  | true = use vertical gradient for background; 
* | **useMultiThreadedRendering** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.useMultiThreadedRendering``\ 
  | true = rendering is done in separate thread; false = no separate thread, which may be more stable but has lagging interaction for large models (do not interact with models during simulation); you MUST set this parameter BEFORE call to SC.renderer.Start(); MAC OS: uses always false, because MAC OS does not support multi threaded GLFW
* | **useWindowsDisplayScaleFactor** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.useWindowsDisplayScaleFactor``\ 
  | the Windows display scaling (monitor scaling; content scaling) factor is used for increased visibility of texts on high resolution displays; based on GLFW glfwGetWindowContentScale; deactivated on linux compilation as it leads to crashes (adjust textSize manually!)
* | **zoomAllUseBoundingBox** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.general.zoomAllUseBoundingBox``\ 
  | if true, use exact scene bounding box (but not including texts) for zoom; does not include perspective effects!



.. _sec-vsettingscontouradvanced:

VSettingsContourAdvanced
------------------------

Advanced settings for contour plots.

VSettingsContourAdvanced has the following items:

* | **colorBarPrecision** [type = PInt, default = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.colorBarPrecision``\ 
  | precision of floating point values shown in color bar; total number of digits used (max. 16)
* | **colorBarTiling** [type = PInt, default = 12, size = 1]:
  | \ ``SC.visualizationSettings.contour.advanced.colorBarTiling``\ 
  | number of tiles (segements) shown in the colorbar for the contour plot
* | **contourColor0** [type = Float4, default = [0.1,0.1,0.9,1.], size = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.contourColor0``\ 
  | RGBA color for relative value 0 used for contour plot; alpha is ignored
* | **contourColor1** [type = Float4, default = [0.1,0.9,0.9,1.], size = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.contourColor1``\ 
  | RGBA color for relative value 0.25 used for contour plot; alpha is ignored
* | **contourColor2** [type = Float4, default = [0.1,0.9,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.contourColor2``\ 
  | RGBA color for relative value 0.25 used for contour plot; alpha is ignored
* | **contourColor3** [type = Float4, default = [0.9,0.9,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.contourColor3``\ 
  | RGBA color for relative value 0.25 used for contour plot; alpha is ignored
* | **contourColor4** [type = Float4, default = [0.9,0.1,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.contourColor4``\ 
  | RGBA color for relative value 0.25 used for contour plot; alpha is ignored
* | **contourColorMax** [type = Float4, default = [0.9,0.9,0.9,1.], size = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.contourColorMax``\ 
  | RGBA color if relative value in contour plot is larger than 1 (if automaticRange=False); alpha is ignored
* | **contourColorMin** [type = Float4, default = [0.1,0.1,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.contour.advanced.contourColorMin``\ 
  | RGBA color if relative value in contour plot is smaller than 0 (if automaticRange=False); alpha is ignored
* | **showColorBar** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.contour.advanced.showColorBar``\ 
  | show the colour bar with minimum and maximum values for the contour plot



.. _sec-vsettingscontour:

VSettingsContour
----------------

Settings for contour plots; use these options to visualize field data, such as displacements, stresses, strains, etc. for bodies, nodes and finite elements.

VSettingsContour has the following items:

* | **advanced** [type = VSettingsContourAdvanced]:
  | \ ``SC.visualizationSettings.contour.advanced``\ 
  | advanced settings for contour
* | **alphaTransparency** [type = float, default = 1, size = 1]:
  | \ ``SC.visualizationSettings.contour.alphaTransparency``\ 
  | default value for contour alpha transparency (RGB color computed from contour value)
* | **automaticRange** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.contour.automaticRange``\ 
  | if true, the contour plot value range is chosen automatically to the maximum range
* | **maxValue** [type = float, default = 1, size = 1]:
  | \ ``SC.visualizationSettings.contour.maxValue``\ 
  | maximum value for contour plot; set manually, if automaticRange == False
* | **minValue** [type = float, default = 0, size = 1]:
  | \ ``SC.visualizationSettings.contour.minValue``\ 
  | minimum value for contour plot; set manually, if automaticRange == False
* | **nodesColored** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.contour.nodesColored``\ 
  | if true, the contour color is also applied to nodes (except mesh nodes), otherwise node drawing is not influenced by contour settings
* | **outputVariable** [type = OutputVariableType, default = OutputVariableType::\_None]:
  | \ ``SC.visualizationSettings.contour.outputVariable``\ 
  | selected contour plot output variable type; select OutputVariableType._None to deactivate contour plotting.
* | **outputVariableComponent** [type = Int, default = 0, size = 1]:
  | \ ``SC.visualizationSettings.contour.outputVariableComponent``\ 
  | select the component of the chosen output variable; e.g., for displacements, 3 components are available: 0 == x, 1 == y, 2 == z component; for stresses, 6 components are available, see OutputVariableType description; to draw the norm of a outputVariable, set component to -1; if a certain component is not available by certain objects or nodes, no value is drawn (using default color)
* | **reduceRange** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.contour.reduceRange``\ 
  | if true, the contour plot value range is also reduced; better for static computation; in dynamic computation set this option to false, it can reduce visualization artifacts; you should also set minVal to max(float) and maxVal to min(float)
* | **rigidBodiesColored** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.contour.rigidBodiesColored``\ 
  | if true, the contour color is also applied to triangular faces of rigid bodies and mass points, otherwise the rigid body drawing are not influenced by contour settings; for general rigid bodies (except for ObjectGround), Position, Displacement, DisplacementLocal(=0), Velocity, VelocityLocal, AngularVelocity, and AngularVelocityLocal are available; may slow down visualization!



.. _sec-vsettingsnodes:

VSettingsNodes
--------------

Visualization settings for nodes.

VSettingsNodes has the following items:

* | **basisSize** [type = float, default = 0.2]:
  | \ ``SC.visualizationSettings.nodes.basisSize``\ 
  | size of basis for nodes
* | **defaultColor** [type = Float4, default = [0.2,0.2,1.,1.], size = 4]:
  | \ ``SC.visualizationSettings.nodes.defaultColor``\ 
  | default RGBA color for nodes; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = -1.]:
  | \ ``SC.visualizationSettings.nodes.defaultSize``\ 
  | global node size; if -1.f, node size is relative to openGL.initialMaxSceneSize
* | **drawNodesAsPoint** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.nodes.drawNodesAsPoint``\ 
  | simplified/faster drawing of nodes; uses general->pointSize as drawing size; if drawNodesAsPoint==True, the basis of the node will be drawn with lines
* | **show** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.nodes.show``\ 
  | flag to decide, whether the nodes are shown
* | **showBasis** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.nodes.showBasis``\ 
  | show basis (three axes) of coordinate system in 3D nodes
* | **showNodalSlopes** [type = UInt, default = False]:
  | \ ``SC.visualizationSettings.nodes.showNodalSlopes``\ 
  | draw nodal slope vectors, e.g. in ANCF beam finite elements
* | **showNumbers** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.nodes.showNumbers``\ 
  | flag to decide, whether the node number is shown
* | **tiling** [type = PInt, default = 4]:
  | \ ``SC.visualizationSettings.nodes.tiling``\ 
  | tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4



.. _sec-vsettingsbeams:

VSettingsBeams
--------------

Visualization settings for beam finite elements.

VSettingsBeams has the following items:

* | **axialTiling** [type = PInt, default = 8]:
  | \ ``SC.visualizationSettings.bodies.beams.axialTiling``\ 
  | number of segments to discretise the beams axis
* | **crossSectionFilled** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.bodies.beams.crossSectionFilled``\ 
  | if implemented for element, cross section is drawn as solid (filled) instead of wire-frame; NOTE: some quantities may not be interpolated correctly over cross section in visualization; equivalent to drawSolid of shells
* | **crossSectionTiling** [type = PInt, default = 4]:
  | \ ``SC.visualizationSettings.bodies.beams.crossSectionTiling``\ 
  | number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing
* | **drawVertical** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.bodies.beams.drawVertical``\ 
  | draw contour plot outputVariables 'vertical' along beam height; contour.outputVariable must be set accordingly
* | **drawVerticalColor** [type = Float4, default = [0.2,0.2,0.2,1.], size = 4]:
  | \ ``SC.visualizationSettings.bodies.beams.drawVerticalColor``\ 
  | color for outputVariable to be drawn along cross section (vertically)
* | **drawVerticalFactor** [type = UFloat, default = 1.]:
  | \ ``SC.visualizationSettings.bodies.beams.drawVerticalFactor``\ 
  | factor for outputVariable to be drawn along cross section (vertically)
* | **drawVerticalLines** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.bodies.beams.drawVerticalLines``\ 
  | draw additional vertical lines for better visibility
* | **drawVerticalOffset** [type = float, default = 0.]:
  | \ ``SC.visualizationSettings.bodies.beams.drawVerticalOffset``\ 
  | offset for vertical drawn lines; offset is added before multiplication with drawVerticalFactor
* | **drawVerticalValues** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.bodies.beams.drawVerticalValues``\ 
  | show values at vertical lines; note that these numbers are interpolated values and may be different from values evaluated directly at this point!
* | **reducedAxialInterploation** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.bodies.beams.reducedAxialInterploation``\ 
  | if True, the interpolation along the beam axis may be lower than the beam element order; this may, however, show more consistent values than a full interpolation, e.g. for strains or forces



.. _sec-vsettingsshells:

VSettingsShells
---------------

Visualization settings for plate/shell finite elements.

VSettingsShells has the following items:

* | **drawSolid** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.bodies.shells.drawSolid``\ 
  | if true: to draw plates/shells as 3D objects; false: only the element surface is drawn; equivalent to crossSectionFilled in beams
* | **thicknessFactor** [type = PFloat, default = 1.]:
  | \ ``SC.visualizationSettings.bodies.shells.thicknessFactor``\ 
  | a factor multiplied with the thickness of shells/plates only for visualization (e.g. to make some effects more visible)



.. _sec-vsettingskinematictree:

VSettingsKinematicTree
----------------------

Visualization settings for kinematic trees.

VSettingsKinematicTree has the following items:

* | **frameSize** [type = float, default = 0.2]:
  | \ ``SC.visualizationSettings.bodies.kinematicTree.frameSize``\ 
  | size of COM and joint frames
* | **showCOMframes** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.bodies.kinematicTree.showCOMframes``\ 
  | if True, a frame is attached to every center of mass
* | **showFramesNumbers** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.bodies.kinematicTree.showFramesNumbers``\ 
  | if True, numbers are drawn for joint frames (O[i]J[j]) and COM frames (O[i]COM[j]) for object [i] and local joint [j]
* | **showJointFrames** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.bodies.kinematicTree.showJointFrames``\ 
  | if True, a frame is attached to the origin of every joint frame



.. _sec-vsettingsbodies:

VSettingsBodies
---------------

Visualization settings for bodies.

VSettingsBodies has the following items:

* | **beams** [type = VSettingsBeams]:
  | \ ``SC.visualizationSettings.bodies.beams``\ 
  | visualization settings for beams (e.g. ANCFCable or other beam elements)
* | **kinematicTree** [type = VSettingsKinematicTree]:
  | \ ``SC.visualizationSettings.bodies.kinematicTree``\ 
  | visualization settings for kinematic tree
* | **shells** [type = VSettingsShells]:
  | \ ``SC.visualizationSettings.bodies.shells``\ 
  | visualization settings for plates and shells
* | **defaultColor** [type = Float4, default = [0.3,0.3,1.,1.], size = 4]:
  | \ ``SC.visualizationSettings.bodies.defaultColor``\ 
  | default RGBA color for bodies; 4th value is alpha-transparency
* | **defaultSize** [type = Float3, default = [1.,1.,1.], size = 3]:
  | \ ``SC.visualizationSettings.bodies.defaultSize``\ 
  | global body size of xyz-cube
* | **deformationScaleFactor** [type = float, default = 1]:
  | \ ``SC.visualizationSettings.bodies.deformationScaleFactor``\ 
  | global deformation scale factor; also applies to nodes, if drawn; currently only used for scaled drawing of (linear) finite elements in FFRF and FFRFreducedOrder objects
* | **show** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.bodies.show``\ 
  | flag to decide, whether the bodies are shown
* | **showNumbers** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.bodies.showNumbers``\ 
  | flag to decide, whether the body(=object) number is shown



.. _sec-vsettingsconnectors:

VSettingsConnectors
-------------------

Visualization settings for connectors.

VSettingsConnectors has the following items:

* | **contactPointsDefaultSize** [type = float, default = 0.02]:
  | \ ``SC.visualizationSettings.connectors.contactPointsDefaultSize``\ 
  | DEPRECATED: do not use! global contact points size; if -1.f, connector size is relative to maxSceneSize
* | **defaultColor** [type = Float4, default = [0.2,0.2,1.,1.], size = 4]:
  | \ ``SC.visualizationSettings.connectors.defaultColor``\ 
  | default RGBA color for connectors; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = 0.1]:
  | \ ``SC.visualizationSettings.connectors.defaultSize``\ 
  | global connector size; if -1.f, connector size is relative to maxSceneSize
* | **jointAxesLength** [type = float, default = 0.2]:
  | \ ``SC.visualizationSettings.connectors.jointAxesLength``\ 
  | global joint axes length
* | **jointAxesRadius** [type = float, default = 0.02]:
  | \ ``SC.visualizationSettings.connectors.jointAxesRadius``\ 
  | global joint axes radius
* | **show** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.connectors.show``\ 
  | flag to decide, whether the connectors are shown
* | **showContact** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.connectors.showContact``\ 
  | flag to decide, whether contact points, lines, etc. are shown for special cable-circle contacts; for spheres, triangles, tori, see visualizationSettings.contact
* | **showJointAxes** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.connectors.showJointAxes``\ 
  | flag to decide, whether contact joint axes of 3D joints are shown
* | **showNumbers** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.connectors.showNumbers``\ 
  | flag to decide, whether the connector(=object) number is shown
* | **springNumberOfWindings** [type = PInt, default = 8]:
  | \ ``SC.visualizationSettings.connectors.springNumberOfWindings``\ 
  | number of windings for springs drawn as helical spring



.. _sec-vsettingsmarkers:

VSettingsMarkers
----------------

Visualization settings for markers.

VSettingsMarkers has the following items:

* | **defaultColor** [type = Float4, default = [0.1,0.5,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.markers.defaultColor``\ 
  | default RGBA color for markers; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = -1.]:
  | \ ``SC.visualizationSettings.markers.defaultSize``\ 
  | global marker size; if -1.f, marker size is relative to maxSceneSize
* | **drawSimplified** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.markers.drawSimplified``\ 
  | draw markers with simplified symbols
* | **show** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.markers.show``\ 
  | flag to decide, whether the markers are shown
* | **showNumbers** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.markers.showNumbers``\ 
  | flag to decide, whether the marker numbers are shown



.. _sec-vsettingsloads:

VSettingsLoads
--------------

Visualization settings for loads.

VSettingsLoads has the following items:

* | **defaultColor** [type = Float4, default = [0.7,0.1,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.loads.defaultColor``\ 
  | default RGBA color for loads; 4th value is alpha-transparency
* | **defaultRadius** [type = float, default = 0.005]:
  | \ ``SC.visualizationSettings.loads.defaultRadius``\ 
  | global radius of load axis if drawn in 3D
* | **defaultSize** [type = float, default = 0.2]:
  | \ ``SC.visualizationSettings.loads.defaultSize``\ 
  | global load size; if -1.f, load size is relative to maxSceneSize
* | **drawSimplified** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.loads.drawSimplified``\ 
  | draw markers with simplified symbols
* | **drawWithUserFunction** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.loads.drawWithUserFunction``\ 
  | draw loads like force vectors time dependent; make sure that fixedLoadSize=false, while otherwise only the direction will change; user functions can only be drawn, if they are either symbolic or for Python user functions if useMultiThreadedRendering=False
* | **fixedLoadSize** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.loads.fixedLoadSize``\ 
  | if true, the load is drawn with a fixed vector length in direction of the load vector, independently of the load size
* | **loadSizeFactor** [type = float, default = 0.1]:
  | \ ``SC.visualizationSettings.loads.loadSizeFactor``\ 
  | if fixedLoadSize=false, then this scaling factor is used to draw the load vector
* | **show** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.loads.show``\ 
  | flag to decide, whether the loads are shown
* | **showNumbers** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.loads.showNumbers``\ 
  | flag to decide, whether the load numbers are shown



.. _sec-vsettingstraces:

VSettingsTraces
---------------

Visualization settings for traces of sensors. Note that a large number of time points (influenced by simulationSettings.solutionSettings.sensorsWritePeriod) may lead to slow graphics.

VSettingsTraces has the following items:

* | **lineWidth** [type = UFloat, default = 2.]:
  | \ ``SC.visualizationSettings.sensors.traces.lineWidth``\ 
  | line width for traces
* | **listOfPositionSensors** [type = ArrayIndex, default = [], size = -1]:
  | \ ``SC.visualizationSettings.sensors.traces.listOfPositionSensors``\ 
  | list of position sensors which can be shown as trace inside render window if sensors have storeInternal=True; if this list is empty and showPositionTrace=True, then all available sensors are shown
* | **listOfTriadSensors** [type = ArrayIndex, default = [], size = -1]:
  | \ ``SC.visualizationSettings.sensors.traces.listOfTriadSensors``\ 
  | list of sensors of with OutputVariableType RotationMatrix; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showTriads=True; the triad is drawn at the related position
* | **listOfVectorSensors** [type = ArrayIndex, default = [], size = -1]:
  | \ ``SC.visualizationSettings.sensors.traces.listOfVectorSensors``\ 
  | list of sensors with 3D vector quantities; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showVectors=True; the vector quantity is drawn relative to the related position
* | **positionsShowEvery** [type = PInt, default = 1]:
  | \ ``SC.visualizationSettings.sensors.traces.positionsShowEvery``\ 
  | integer value i; out of available sensor data, show every i-th position
* | **sensorsMbsNumber** [type = Index, default = 0]:
  | \ ``SC.visualizationSettings.sensors.traces.sensorsMbsNumber``\ 
  | number of main system which is used to for sensor lists; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number
* | **showCurrent** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.sensors.traces.showCurrent``\ 
  | show current trace position (and especially vector quantity) related to current visualization state; this only works in solution viewer if sensor values are stored at time grid points of the solution file (up to a precision of 1e-10) and may therefore be temporarily unavailable
* | **showFuture** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.sensors.traces.showFuture``\ 
  | show trace future to current visualization state if already computed (e.g. in SolutionViewer)
* | **showPast** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.sensors.traces.showPast``\ 
  | show trace previous to current visualization state
* | **showPositionTrace** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.sensors.traces.showPositionTrace``\ 
  | show position trace of all position sensors if listOfPositionSensors=[] or of specified sensors; sensors need to activate storeInternal=True
* | **showTriads** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.sensors.traces.showTriads``\ 
  | if True, show basis vectors from rotation matrices provided by sensors
* | **showVectors** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.sensors.traces.showVectors``\ 
  | if True, show vector quantities according to description in showPositionTrace
* | **timeSpan** [type = UReal, default = 0]:
  | \ ``SC.visualizationSettings.sensors.traces.timeSpan``\ 
  | maximum trace time span of past or future trace; given in seconds of simulation time; if zero, it is unused
* | **traceColors** [type = ArrayFloat, default = [0.2,0.2,0.2,1., 0.8,0.2,0.2,1., 0.2,0.8,0.2,1., 0.2,0.2,0.8,1., 0.2,0.8,0.8,1., 0.8,0.2,0.8,1., 0.8,0.4,0.1,1.], size = -1]:
  | \ ``SC.visualizationSettings.sensors.traces.traceColors``\ 
  | RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue
* | **triadSize** [type = float, default = 0.1 ]:
  | \ ``SC.visualizationSettings.sensors.traces.triadSize``\ 
  | length of triad axes if shown
* | **triadsShowEvery** [type = PInt, default = 1]:
  | \ ``SC.visualizationSettings.sensors.traces.triadsShowEvery``\ 
  | integer value i; out of available sensor data, show every i-th triad
* | **vectorScaling** [type = float, default = 0.01]:
  | \ ``SC.visualizationSettings.sensors.traces.vectorScaling``\ 
  | scaling of vector quantities; if, e.g., loads, this factor has to be adjusted significantly
* | **vectorsShowEvery** [type = PInt, default = 1]:
  | \ ``SC.visualizationSettings.sensors.traces.vectorsShowEvery``\ 
  | integer value i; out of available sensor data, show every i-th vector



.. _sec-vsettingssensors:

VSettingsSensors
----------------

Visualization settings for sensors.

VSettingsSensors has the following items:

* | **traces** [type = VSettingsTraces]:
  | \ ``SC.visualizationSettings.sensors.traces``\ 
  | settings for showing (position/triad) sensor traces and vector plots in the render window
* | **defaultColor** [type = Float4, default = [0.6,0.6,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.sensors.defaultColor``\ 
  | default RGBA color for sensors; 4th value is alpha-transparency
* | **defaultSize** [type = float, default = -1.]:
  | \ ``SC.visualizationSettings.sensors.defaultSize``\ 
  | global sensor size; if -1.f, sensor size is relative to maxSceneSize
* | **drawSimplified** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.sensors.drawSimplified``\ 
  | draw sensors with simplified symbols
* | **show** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.sensors.show``\ 
  | flag to decide, whether the sensors are shown
* | **showNumbers** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.sensors.showNumbers``\ 
  | flag to decide, whether the sensor numbers are shown



.. _sec-vsettingscontact:

VSettingsContact
----------------

Global visualization settings for GeneralContact. This allows to easily switch on/off during visualization; also used for contact objects, such as ObjectContactSphereSphere or ObjectContactSphereTriangle. 

VSettingsContact has the following items:

* | **colorBoundingBoxes** [type = Float4, default = [0.9,0.1,0.1,1.], size = 4]:
  | \ ``SC.visualizationSettings.contact.colorBoundingBoxes``\ 
  | RGBA color for boudnding boxes, see showBoundingBoxes
* | **colorSearchTree** [type = Float4, default = [0.1,0.1,0.9,1.], size = 4]:
  | \ ``SC.visualizationSettings.contact.colorSearchTree``\ 
  | RGBA color for search tree, see showSearchTree
* | **colorSpheres** [type = Float4, default = [0.8,0.5,0.2,1.], size = 4]:
  | \ ``SC.visualizationSettings.contact.colorSpheres``\ 
  | RGBA color for contact spheres, see showSpheres
* | **colorTori** [type = Float4, default = [0.8,0.2,0.8,1.], size = 4]:
  | \ ``SC.visualizationSettings.contact.colorTori``\ 
  | RGBA color for contact tori, see showTori
* | **colorTriangles** [type = Float4, default = [0.5,0.5,0.5,1.], size = 4]:
  | \ ``SC.visualizationSettings.contact.colorTriangles``\ 
  | RGBA color for contact triangles, see showTriangles
* | **contactForcesFactor** [type = float, default = 0.001]:
  | \ ``SC.visualizationSettings.contact.contactForcesFactor``\ 
  | factor used for scaling of contact forces is showContactForces=True
* | **contactPointsDefaultSize** [type = float, default = 0.001]:
  | \ ``SC.visualizationSettings.contact.contactPointsDefaultSize``\ 
  | global contact points size; if -1.f, connector size is relative to maxSceneSize; used for some contacts, e.g., in ContactFrictionCircle
* | **showBoundingBoxes** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showBoundingBoxes``\ 
  | show computed bounding boxes of all GeneralContacts; Warning: avoid for large number of contact objects!
* | **showContactForces** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showContactForces``\ 
  | if True, contact forces are drawn for certain contact models
* | **showContactForcesValues** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showContactForcesValues``\ 
  | if True and showContactForces=True, numerical values for  contact forces are shown at certain points
* | **showSearchTree** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showSearchTree``\ 
  | show outer box of search tree for all GeneralContacts
* | **showSearchTreeCells** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showSearchTreeCells``\ 
  | show all cells of search tree; empty cells have colorSearchTree, cells with contact objects have higher red value; Warning: avoid for large number of search tree cells!
* | **showSpheres** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showSpheres``\ 
  | show contact spheres (SpheresWithMarker, ...)
* | **showTori** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showTori``\ 
  | show each contact torus
* | **showTriangles** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.contact.showTriangles``\ 
  | show contact triangles (TrianglesRigidBodyBased, ...)
* | **tilingCurves** [type = PInt, default = 8]:
  | \ ``SC.visualizationSettings.contact.tilingCurves``\ 
  | tiling for nonlinear/polynomial curves; higher values give smoother curves
* | **tilingSpheres** [type = PInt, default = 4]:
  | \ ``SC.visualizationSettings.contact.tilingSpheres``\ 
  | tiling for spheres; higher values give smoother spheres, but may lead to lower frame rates



.. _sec-vsettingscamera:

VSettingsCamera
---------------

Settings for camera like perspective, marker tracking, clipping plane, etc. Note that some options may also be found in openGL settings.

VSettingsCamera has the following items:

* | **cameraPosition** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \ ``SC.visualizationSettings.view0.camera.cameraPosition``\ , \ ``SC.visualizationSettings.view1.camera.cameraPosition``\ , \ ``SC.visualizationSettings.view2.camera.cameraPosition``\ , \ ``SC.visualizationSettings.view3.camera.cameraPosition``\ 
  | if modelCentricView=True: offset to camera position in model view (and, if used, relative to tracked marker - instead of a tracked marker position, you could also just change the camera position in camera-centric views); camera rotation follows modelRotation in renderState
* | **clippingPlaneDistance** [type = float, default = 0.]:
  | \ ``SC.visualizationSettings.view0.camera.clippingPlaneDistance``\ , \ ``SC.visualizationSettings.view1.camera.clippingPlaneDistance``\ , \ ``SC.visualizationSettings.view2.camera.clippingPlaneDistance``\ , \ ``SC.visualizationSettings.view3.camera.clippingPlaneDistance``\ 
  | distance of clipping plane on normal vector; see also clippingPlaneNormal and openGL.advanced.clippingPlaneColor
* | **clippingPlaneNormal** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \ ``SC.visualizationSettings.view0.camera.clippingPlaneNormal``\ , \ ``SC.visualizationSettings.view1.camera.clippingPlaneNormal``\ , \ ``SC.visualizationSettings.view2.camera.clippingPlaneNormal``\ , \ ``SC.visualizationSettings.view3.camera.clippingPlaneNormal``\ 
  | normal vector of clipping plane, e.g. [0,0,1] to set a xy-clipping plane; the clipped half-space is in direction of the normal; use [0,0,0] to deactivate clipping plane; Note that clipping is mainly made for triangles in order to visualize hidden objects and currently it only fully clips triangles, but does not exactly cut them; see also clippingPlaneDistance and openGL.advanced.clippingPlaneColor
* | **modelCentricView** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.view0.camera.modelCentricView``\ , \ ``SC.visualizationSettings.view1.camera.modelCentricView``\ , \ ``SC.visualizationSettings.view2.camera.modelCentricView``\ , \ ``SC.visualizationSettings.view3.camera.modelCentricView``\ 
  | True: rotations and translations are applied to model, while camera stays far enough away from the model and always captures the whole model (everything is in front of camera plane); False: camera moves and rotates while model stays in physical space; only geometry in front of camera is visible; note that the behavior of trackMarker changes with modelCentricView and some features are not available in case of modelCentricView=False.
* | **nearFarPlaneOffset** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \ ``SC.visualizationSettings.view0.camera.nearFarPlaneOffset``\ , \ ``SC.visualizationSettings.view1.camera.nearFarPlaneOffset``\ , \ ``SC.visualizationSettings.view2.camera.nearFarPlaneOffset``\ , \ ``SC.visualizationSettings.view3.camera.nearFarPlaneOffset``\ 
  | the three values are [nearPlaneOffset, farPlaneOffset, flag]; if flag=0, the offsets are ignored and computed automatically, using x = 2 * maxSceneSize * zMaxSceneFactor, setting near plane to -x and far plane to +x in case of modelCentricView=True and setting near plane to 0.01 (minimal offset to eye point) and far plane to +x if modelCentricView=False; if flag=1, the near and far plane values are just overwritten; note that positive values for near plane make objects in front of the camera invisible while negative values make objects behind the camera plane visible; in case of camera-centric view, the eyepoint can be shifted backwards using cameraPosition accordingly.
* | **perspective** [type = UFloat, default = 0.]:
  | \ ``SC.visualizationSettings.view0.camera.perspective``\ , \ ``SC.visualizationSettings.view1.camera.perspective``\ , \ ``SC.visualizationSettings.view2.camera.perspective``\ , \ ``SC.visualizationSettings.view3.camera.perspective``\ 
  | parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 1 (extreme: 5), where larger values are possible but should be used with care; NOTE that the relation to the common field of view (FOV) angle alpha, with alpha=90°, is given by perspective = tan(alpha/2) = 1; mouse coordinates (F3) can not be shown with perspective>0
* | **trackMarker** [type = Int, default = -1]:
  | \ ``SC.visualizationSettings.view0.camera.trackMarker``\ , \ ``SC.visualizationSettings.view1.camera.trackMarker``\ , \ ``SC.visualizationSettings.view2.camera.trackMarker``\ , \ ``SC.visualizationSettings.view3.camera.trackMarker``\ 
  | if valid marker index is provided and marker provides position (and orientation), the centerpoint of the scene follows the marker (and orientation); depends on trackMarkerPosition and trackMarkerOrientation; by default, only position is tracked
* | **trackMarkerMbsNumber** [type = Index, default = 0]:
  | \ ``SC.visualizationSettings.view0.camera.trackMarkerMbsNumber``\ , \ ``SC.visualizationSettings.view1.camera.trackMarkerMbsNumber``\ , \ ``SC.visualizationSettings.view2.camera.trackMarkerMbsNumber``\ , \ ``SC.visualizationSettings.view3.camera.trackMarkerMbsNumber``\ 
  | number of main system which is used to track marker; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number
* | **trackMarkerOrientation** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \ ``SC.visualizationSettings.view0.camera.trackMarkerOrientation``\ , \ ``SC.visualizationSettings.view1.camera.trackMarkerOrientation``\ , \ ``SC.visualizationSettings.view2.camera.trackMarkerOrientation``\ , \ ``SC.visualizationSettings.view3.camera.trackMarkerOrientation``\ 
  | choose which orientation axes (x,y,z) are tracked; currently can only be all zero or all one
* | **trackMarkerPosition** [type = Float3, default = [1.,1.,1.], size = 3]:
  | \ ``SC.visualizationSettings.view0.camera.trackMarkerPosition``\ , \ ``SC.visualizationSettings.view1.camera.trackMarkerPosition``\ , \ ``SC.visualizationSettings.view2.camera.trackMarkerPosition``\ , \ ``SC.visualizationSettings.view3.camera.trackMarkerPosition``\ 
  | choose which coordinates or marker are tracked (x,y,z)
* | **useRaytracer** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.view0.camera.useRaytracer``\ , \ ``SC.visualizationSettings.view1.camera.useRaytracer``\ , \ ``SC.visualizationSettings.view2.camera.useRaytracer``\ , \ ``SC.visualizationSettings.view3.camera.useRaytracer``\ 
  | True: use (software) raytracer for this view; False: use standard OpenGL renderer



.. _sec-vsettingsscene:

VSettingsScene
--------------

Settings change scene representation (show edges, show faces, global transparency), adding world basis, etc., in particular settings that are individual to each view. Note that some scene settings that are global to all views may be found in general and in openGL settings. 

VSettingsScene has the following items:

* | **drawCoordinateSystem** [type = UInt, default = 2]:
  | \ ``SC.visualizationSettings.view0.scene.drawCoordinateSystem``\ , \ ``SC.visualizationSettings.view1.scene.drawCoordinateSystem``\ , \ ``SC.visualizationSettings.view2.scene.drawCoordinateSystem``\ , \ ``SC.visualizationSettings.view3.scene.drawCoordinateSystem``\ 
  | 0 = no coordinate system shown, 1 = draw lines with text, 2 = draw arrows, 3 = draw arrows with text
* | **drawWorldBasis** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.view0.scene.drawWorldBasis``\ , \ ``SC.visualizationSettings.view1.scene.drawWorldBasis``\ , \ ``SC.visualizationSettings.view2.scene.drawWorldBasis``\ , \ ``SC.visualizationSettings.view3.scene.drawWorldBasis``\ 
  | true = draw world basis coordinate system at (0,0,0)
* | **facesTransparent** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.view0.scene.facesTransparent``\ , \ ``SC.visualizationSettings.view1.scene.facesTransparent``\ , \ ``SC.visualizationSettings.view2.scene.facesTransparent``\ , \ ``SC.visualizationSettings.view3.scene.facesTransparent``\ 
  | True: show faces transparent independent of transparency (A)-value in color of objects; allow to show otherwise hidden node/marker/object numbers
* | **showFaceEdges** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.view0.scene.showFaceEdges``\ , \ ``SC.visualizationSettings.view1.scene.showFaceEdges``\ , \ ``SC.visualizationSettings.view2.scene.showFaceEdges``\ , \ ``SC.visualizationSettings.view3.scene.showFaceEdges``\ 
  | True: show edges of triangles; using the options showFaces=false and showFaceEdges=true gives are wire frame representation
* | **showFaces** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.view0.scene.showFaces``\ , \ ``SC.visualizationSettings.view1.scene.showFaces``\ , \ ``SC.visualizationSettings.view2.scene.showFaces``\ , \ ``SC.visualizationSettings.view3.scene.showFaces``\ 
  | True: show faces of triangles, etc.; using the options showFaces=false and showFaceEdges=true gives are wireframe representation
* | **showLines** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.view0.scene.showLines``\ , \ ``SC.visualizationSettings.view1.scene.showLines``\ , \ ``SC.visualizationSettings.view2.scene.showLines``\ , \ ``SC.visualizationSettings.view3.scene.showLines``\ 
  | True: show lines (other lines than face and mesh edges)
* | **showMeshEdges** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.view0.scene.showMeshEdges``\ , \ ``SC.visualizationSettings.view1.scene.showMeshEdges``\ , \ ``SC.visualizationSettings.view2.scene.showMeshEdges``\ , \ ``SC.visualizationSettings.view3.scene.showMeshEdges``\ 
  | True: show edges of finite elements; independent of showFaceEdges
* | **showMeshFaces** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.view0.scene.showMeshFaces``\ , \ ``SC.visualizationSettings.view1.scene.showMeshFaces``\ , \ ``SC.visualizationSettings.view2.scene.showMeshFaces``\ , \ ``SC.visualizationSettings.view3.scene.showMeshFaces``\ 
  | True: show faces of finite elements; independent of showFaces
* | **worldBasisSize** [type = PFloat, default = 1.0]:
  | \ ``SC.visualizationSettings.view0.scene.worldBasisSize``\ , \ ``SC.visualizationSettings.view1.scene.worldBasisSize``\ , \ ``SC.visualizationSettings.view2.scene.worldBasisSize``\ , \ ``SC.visualizationSettings.view3.scene.worldBasisSize``\ 
  | size of world basis coordinate system



.. _sec-vsettingswindow:

VSettingsWindow
---------------

Settings for window that are individual to each view; in particular initial size, and behavior. Note that some of the settings are only used during creation of the window. 

VSettingsWindow has the following items:

* | **alwaysOnTop** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.view0.window.alwaysOnTop``\ , \ ``SC.visualizationSettings.view1.window.alwaysOnTop``\ , \ ``SC.visualizationSettings.view2.window.alwaysOnTop``\ , \ ``SC.visualizationSettings.view3.window.alwaysOnTop``\ 
  | True: render window of respective view will be always on top of all other windows
* | **globalFontSize** [type = PFloat, default = 12.]:
  | \ ``SC.visualizationSettings.view0.window.globalFontSize``\ , \ ``SC.visualizationSettings.view1.window.globalFontSize``\ , \ ``SC.visualizationSettings.view2.window.globalFontSize``\ , \ ``SC.visualizationSettings.view3.window.globalFontSize``\ 
  | general text font size (roughly measured in pixels); if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)
* | **lockModelView** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.view0.window.lockModelView``\ , \ ``SC.visualizationSettings.view1.window.lockModelView``\ , \ ``SC.visualizationSettings.view2.window.lockModelView``\ , \ ``SC.visualizationSettings.view3.window.lockModelView``\ 
  | True: all movements (with mouse/keys), rotations, zoom are disabled; the view is either based on initial values (or on the current state) ==> initial zoom, rotation and center point need to be adjusted, approx. 0.4*maxSceneSize is a good value
* | **maximize** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.view0.window.maximize``\ , \ ``SC.visualizationSettings.view1.window.maximize``\ , \ ``SC.visualizationSettings.view2.window.maximize``\ , \ ``SC.visualizationSettings.view3.window.maximize``\ 
  | True: render window of respective view will be maximized at startup
* | **renderWindowSize** [type = Index2, default = [1024,768], size = 2]:
  | \ ``SC.visualizationSettings.view0.window.renderWindowSize``\ , \ ``SC.visualizationSettings.view1.window.renderWindowSize``\ , \ ``SC.visualizationSettings.view2.window.renderWindowSize``\ , \ ``SC.visualizationSettings.view3.window.renderWindowSize``\ 
  | initial size of render window of respective view for specific view in pixels for
* | **showComputationInfo** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.view0.window.showComputationInfo``\ , \ ``SC.visualizationSettings.view1.window.showComputationInfo``\ , \ ``SC.visualizationSettings.view2.window.showComputationInfo``\ , \ ``SC.visualizationSettings.view3.window.showComputationInfo``\ 
  | true = show (hide) all computation information including Exudyn and version
* | **showMouseCoordinates** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.view0.window.showMouseCoordinates``\ , \ ``SC.visualizationSettings.view1.window.showMouseCoordinates``\ , \ ``SC.visualizationSettings.view2.window.showMouseCoordinates``\ , \ ``SC.visualizationSettings.view3.window.showMouseCoordinates``\ 
  | True: show OpenGL coordinates and distance to last left mouse button pressed position in renderer status message; switched on/off with key 'F3'; only works for axis-aligned ortho-projections
* | **showRenderStateInfo** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.view0.window.showRenderStateInfo``\ , \ ``SC.visualizationSettings.view1.window.showRenderStateInfo``\ , \ ``SC.visualizationSettings.view2.window.showRenderStateInfo``\ , \ ``SC.visualizationSettings.view3.window.showRenderStateInfo``\ 
  | True: show renderer.state infos regarding zoom, offset and rotation in renderer status message; switched on/off with 'CTRL-F3'
* | **showWindow** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.view0.window.showWindow``\ , \ ``SC.visualizationSettings.view1.window.showWindow``\ , \ ``SC.visualizationSettings.view2.window.showWindow``\ , \ ``SC.visualizationSettings.view3.window.showWindow``\ 
  | True: render window of respective view is shown when created; False: window will be iconified when created (e.g. if you are starting multiple computations automatically)



.. _sec-vsettingsview:

VSettingsView
-------------

Settings for view including camera, scene, window, and advanced options to setup a view or view window.

VSettingsView has the following items:

* | **camera** [type = VSettingsCamera]:
  | \ ``SC.visualizationSettings.view0.camera``\ , \ ``SC.visualizationSettings.view1.camera``\ , \ ``SC.visualizationSettings.view2.camera``\ , \ ``SC.visualizationSettings.view3.camera``\ 
  | settings for camera like perspective, marker tracking or clipping plane
* | **scene** [type = VSettingsScene]:
  | \ ``SC.visualizationSettings.view0.scene``\ , \ ``SC.visualizationSettings.view1.scene``\ , \ ``SC.visualizationSettings.view2.scene``\ , \ ``SC.visualizationSettings.view3.scene``\ 
  | settings which change scene representation, showing edges, faces or world basis
* | **window** [type = VSettingsWindow]:
  | \ ``SC.visualizationSettings.view0.window``\ , \ ``SC.visualizationSettings.view1.window``\ , \ ``SC.visualizationSettings.view2.window``\ , \ ``SC.visualizationSettings.view3.window``\ 
  | visualization settings for window that are individual to each view



.. _sec-vsettingswindowdeprecated:

VSettingsWindowDeprecated
-------------------------

OpenGL Window and interaction settings for visualization; handle changes with care, as they might lead to unexpected results or crashes.

VSettingsWindowDeprecated has the following items:




.. _sec-vsettingsdialogs:

VSettingsDialogs
----------------

Settings related to dialogs (e.g., visualization settings dialog).

VSettingsDialogs has the following items:

* | **alphaTransparency** [type = UFloat, default = 0.94]:
  | \ ``SC.visualizationSettings.dialogs.alphaTransparency``\ 
  | alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)
* | **alwaysTopmost** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.dialogs.alwaysTopmost``\ 
  | True: dialogs are always topmost (otherwise, they are sometimes hidden)
* | **fontScalingMacOS** [type = UFloat, default = 1.35]:
  | \ ``SC.visualizationSettings.dialogs.fontScalingMacOS``\ 
  | font scaling value for MacOS systems (on Windows, system display scaling is used)
* | **multiThreadedDialogs** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.dialogs.multiThreadedDialogs``\ 
  | True: During dialogs, the OpenGL render windows will still get updates of changes in dialogs, etc., which may cause problems on some platforms or for some (complicated) models; False: changes of dialogs will take effect when dialogs are closed
* | **openTreeView** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.dialogs.openTreeView``\ 
  | True: all sub-trees of the visusalization dialog are opened when opening the dialog; False: only some sub-trees are opened



.. _sec-vsettingsmaterial:

VSettingsMaterial
-----------------

Settings for rendering materials, in particular for the Raytracer (may be available also in the OpenGL renderer in the future). This material (widely follows Phong model) can be either accessed via SC.renderer.materials or directly in visualizationSettings.raytracer.material0, material1, etc.; note that the default values shown in the documentation only reflect material0 but not all 10 default materials.

VSettingsMaterial has the following items:

* | **alpha** [type = UFloat, default = 1.]:
  | \ ``SC.visualizationSettings.raytracer.material.alpha``\ 
  | alpha-transparency, same as in alpha channel in RGBA colors; 1=opaque, 0=fully transparent; leads to extra rendering costs per transparent pixel
* | **baseColor** [type = Float3, default = [0.5,0.5,0.5], size = 3]:
  | \ ``SC.visualizationSettings.raytracer.material.baseColor``\ 
  | RGB default material color if face color has R-color channel -1
* | **emission** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \ ``SC.visualizationSettings.raytracer.material.emission``\ 
  | RGB emissive material color (enlightened material)
* | **ior** [type = UFloat, default = 1.]:
  | \ ``SC.visualizationSettings.raytracer.material.ior``\ 
  | index of refraction for transparent materials (1=no refraction), >1 represents refraction
* | **name** [type = String, default = 'undefined']:
  | \ ``SC.visualizationSettings.raytracer.material.name``\ 
  | material name for easier handling
* | **reflectivity** [type = UFloat, default = 0.]:
  | \ ``SC.visualizationSettings.raytracer.material.reflectivity``\ 
  | controls reflectivity of material; 0=no reflections (rough, e.g. rubber), 1=fully reflective (mirror); this leads to large extra rendering costs per visible reflective pixel
* | **shininess** [type = UFloat, default = 32.]:
  | \ ``SC.visualizationSettings.raytracer.material.shininess``\ 
  | controls shininess of specular component of lights; values < 5 is not very shiny, while > 50 is very shiny
* | **specular** [type = Float3, default = [0.5,0.5,0.5], size = 3]:
  | \ ``SC.visualizationSettings.raytracer.material.specular``\ 
  | RGB specular material color



.. _sec-vsettingsraytraceradvanced:

VSettingsRaytracerAdvanced
--------------------------

Advanced settings for raytracer.

VSettingsRaytracerAdvanced has the following items:

* | **backgroundColorReflections** [type = Float4, default = [0.4,0.4,0.4,1.], size = 4]:
  | \ ``SC.visualizationSettings.raytracer.advanced.backgroundColorReflections``\ 
  | scene RGBA color for background that is hit by reflection material; while openGL.backgroundColor is used for rays that do not hit an object, this background may - if black or white - not be a suitable color for computing reflections; this is generally needed, as our scenes are usually not inside a closed geometry (like inside a room); this color is also used if maxReflectionDepth is reached
* | **searchTreeFactor** [type = PInt, default = 1]:
  | \ ``SC.visualizationSettings.raytracer.advanced.searchTreeFactor``\ 
  | This factor can be used to increase the number of search tree bins, which can improve performance in case of inequilibrated scense; range=1..128
* | **shadowScalingFactor** [type = UInt, default = 3, size = 1]:
  | \ ``SC.visualizationSettings.raytracer.advanced.shadowScalingFactor``\ 
  | if lightRadiusVariations>1, this defines the downscaling factor of the shadow map, where 2 means that the resolution is 2 times smaller than the image resolution; additionally, multisampling is not used for shadow map computation if shadowScalingFactor>0, thus reducing the computational effort for shadow computation also in case of 1; range=0..16; larger values cause significant artifacts at shadow boundaries
* | **shadowSmoothingSteps** [type = UInt, default = 3, size = 1]:
  | \ ``SC.visualizationSettings.raytracer.advanced.shadowSmoothingSteps``\ 
  | if lightRadiusVariations>1, this defines the number of smoothing steps at the low-resolution shadow map; smoothing reduces shadow artifacts caused by smaller values of lightRadiusVariations; range=0..32; smoothing  steps may cause artifacts at shadow boundaries; only works for directional lights with position (e.g. 4th component in light0Position should be 1)
* | **showText** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.raytracer.advanced.showText``\ 
  | True: show any kind of status text, node numbers, object numbers, etc. (depending on settings); False: do not show any text in raytracer, independently of settings
* | **tilesPerThread** [type = PInt, default = 12]:
  | \ ``SC.visualizationSettings.raytracer.advanced.tilesPerThread``\ 
  | Total number of sub-tiles per thread, used to evenly distribute rendering load to threads
* | **zBiasLines** [type = float, default = 1e-3]:
  | \ ``SC.visualizationSettings.raytracer.advanced.zBiasLines``\ 
  | offset for lines to draw in front of faces; relative to scene radius



.. _sec-vsettingsraytracer:

VSettingsRaytracer
------------------

Settings for raytracer (software renderer) which can be used as alternative to classic OpenGL rendering; this option may be erased in future in favor of a modern GPU rendering. To activate the raytracer, simply switch the enable flag to True. The raytracer uses CPU-based rendering and is therefore comparably slow (may take seconds to render one frame). Thus, take care with the window dimension (start with small window size like 400 x 300) and use openGL.multiSampling=1. Note that many parameters are used from openGL settings, like backgroundColor, lineWidth, multiSampling, shadow (only on/off), and lights. See the options to improve appearance and performance.

VSettingsRaytracer has the following items:

* | **advanced** [type = VSettingsRaytracerAdvanced]:
  | \ ``SC.visualizationSettings.raytracer.advanced``\ 
  | advanced settings for raytracer
* | **material0** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material0``\ 
  | settings for material0
* | **material1** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material1``\ 
  | settings for material1
* | **material2** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material2``\ 
  | settings for material2
* | **material3** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material3``\ 
  | settings for material3
* | **material4** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material4``\ 
  | settings for material4
* | **material5** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material5``\ 
  | settings for material5
* | **material6** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material6``\ 
  | settings for material6
* | **material7** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material7``\ 
  | settings for material7
* | **material8** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material8``\ 
  | settings for material8
* | **material9** [type = VSettingsMaterial]:
  | \ ``SC.visualizationSettings.raytracer.material9``\ 
  | settings for material9
* | **globalFogColor** [type = Float4, default = [0.5,0.5,0.5,1.], size = 4]:
  | \ ``SC.visualizationSettings.raytracer.globalFogColor``\ 
  | scene RGBA fog color
* | **globalFogDensity** [type = UFloat, default = 0.]:
  | \ ``SC.visualizationSettings.raytracer.globalFogDensity``\ 
  | global fog density; fog is deactivated if fogDensity=0, otherwise it is a density relative to scene max size; as it is relative, the factor has to be relatively high to be visible (usually >1)
* | **imageSizeFactor** [type = PInt, default = 1]:
  | \ ``SC.visualizationSettings.raytracer.imageSizeFactor``\ 
  | Special size factor (1-16) to allow drawing with smaller resolution (faster); use this for long rendering times for adjustments, etc.
* | **keepWindowActive** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.raytracer.keepWindowActive``\ 
  | Special flag, handle with care; True: sends some glfw functions to keep window reactive for long render times (>2 seconds); otherwise, the rendering may not finish due to timeout
* | **lightRadiusVariations** [type = PInt, default = 1, size = 1]:
  | \ ``SC.visualizationSettings.raytracer.lightRadiusVariations``\ 
  | if lightRadiusVariations>1, this defines the number of positions that are used to compute the effect of distributed lights (larger is slower but better quality); range=1..256; avoid squares of integers; good values: 1 (hard shadow boundaries), 6, 13, 20, 31, 72, 130, 240; for lower values, use shadowSmoothingSteps=2..8
* | **maxReflectionDepth** [type = UInt, default = 2]:
  | \ ``SC.visualizationSettings.raytracer.maxReflectionDepth``\ 
  | Maximum number of reflections computed for one ray (note that for each transparent face passed, the reflection depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
* | **maxTransparencyDepth** [type = UInt, default = 2]:
  | \ ``SC.visualizationSettings.raytracer.maxTransparencyDepth``\ 
  | Maximum number of transparent faces that can be passed (note that for each reflection, the transparency depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)
* | **multiSampling** [type = PInt, default = 1, size = 1]:
  | \ ``SC.visualizationSettings.raytracer.multiSampling``\ 
  | Multi-sampling used for rendering of faces, lines and text; increases image quality along edges (lines, etc.) but INCREASES rendering costs dramatically (multiSampling=3 => 3x3=9 times slower); also used for shadow if shadowScalingFactor=0; values only accepted in range [1..4]
* | **numberOfThreads** [type = PInt, default = 8]:
  | \ ``SC.visualizationSettings.raytracer.numberOfThreads``\ 
  | Number of CPU-threads (max: 256) used for software rendering (should be approx. the number of available threads)
* | **verbose** [type = Index, default = 0]:
  | \ ``SC.visualizationSettings.raytracer.verbose``\ 
  | 1: print out some debug information on rendering, in particular rendering timings and counter; 2 and higher: advanced debug information



.. _sec-vsettingsopengladvanced:

VSettingsOpenGLAdvanced
-----------------------

Advanced settings for openGL.

VSettingsOpenGLAdvanced has the following items:

* | **clippingPlaneColor** [type = Float4, default = [0.7,0.5,0.5,0.], size = 4]:
  | \ ``SC.visualizationSettings.openGL.advanced.clippingPlaneColor``\ 
  | RGBA color for clipping plane; if alpha-channel is 0, the cutting plane is not drawn; if alpha-channel is 1, the clippingPlaneColor is used; if alpha-channel is 2, the color of the object interior is used as clipping plane color (which may look strange in case of object-in-object); see also view.camera for clipping plane options
* | **depthSorting** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.depthSorting``\ 
  | True (slower): sort triangles by Z-depth to remove transparency artifacts: only works if triangles do not intersect or come close (you may like to refine triangle meshes); False: no depth-sort (faster)
* | **enableLighting** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.enableLighting``\ 
  | generally enable lighting (otherwise, colors of objects are used); OpenGL: glEnable(GL_LIGHTING)
* | **faceNormalsColor** [type = Float4, default = [0.8,0.2,0.2,1.], size = 4]:
  | \ ``SC.visualizationSettings.openGL.advanced.faceNormalsColor``\ 
  | global RGBA color for face normals
* | **initialCenterPoint** [type = Float3, default = [0.,0.,0.], size = 3]:
  | \ ``SC.visualizationSettings.openGL.advanced.initialCenterPoint``\ 
  | centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True; only used in case that modelCentricView=True
* | **initialMaxSceneSize** [type = PFloat, default = 1.]:
  | \ ``SC.visualizationSettings.openGL.advanced.initialMaxSceneSize``\ 
  | initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True
* | **initialModelRotation** [type = StdArray33F, default = [Matrix3DF[3,3,1.,0.,0., 0.,1.,0., 0.,0.,1.]], size = 3x3]:
  | \ ``SC.visualizationSettings.openGL.advanced.initialModelRotation``\ 
  | initial model rotation matrix for OpenGl; in python use e.g.: initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]; only used in case that modelCentricView=True
* | **initialZoom** [type = UFloat, default = 1.]:
  | \ ``SC.visualizationSettings.openGL.advanced.initialZoom``\ 
  | initial zoom of scene; overwritten/ignored if autoFitScene = True
* | **lightModelLocalViewer** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.lightModelLocalViewer``\ 
  | True: the camera origin is used to compute shininess effects (more realistic); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,...)
* | **lightModelTwoSide** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.lightModelTwoSide``\ 
  | enlighten also backside of object; may cause problems on some graphics cards and lead to slower performance; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,...)
* | **lineSmooth** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.lineSmooth``\ 
  | draw lines smooth
* | **polygonOffset** [type = float, default = 0.05]:
  | \ ``SC.visualizationSettings.openGL.advanced.polygonOffset``\ 
  | general polygon offset for polygons, except for shadows; use this parameter to draw polygons behind lines to reduce artifacts for very large or small models
* | **shadeModelSmooth** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.shadeModelSmooth``\ 
  | True: turn on smoothing for shaders, which uses vertex normals to smooth surfaces
* | **shadowPolygonOffset** [type = PFloat, default = 0.1]:
  | \ ``SC.visualizationSettings.openGL.advanced.shadowPolygonOffset``\ 
  | some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies
* | **showBoundingBox** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.showBoundingBox``\ 
  | show scene bounding box (red), as available in renderState.boundingBox; NOTE that the bounding box is only updated with ZoomAll or at startup; this is a debug flag and it may show reasongs for strange ZoomAll behavior, as ZoomAll should zoom to the bounding box; does only work for perspective=0
* | **textLineSmooth** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.textLineSmooth``\ 
  | draw lines for representation of text smooth
* | **textLineWidth** [type = UFloat, default = 1., size = 1]:
  | \ ``SC.visualizationSettings.openGL.advanced.textLineWidth``\ 
  | width of lines used for representation of text
* | **vertexNormalsColor** [type = Float4, default = [0.8,0.2,0.2,1.], size = 4]:
  | \ ``SC.visualizationSettings.openGL.advanced.vertexNormalsColor``\ 
  | global RGBA color for vertex normals



.. _sec-vsettingslight:

VSettingsLight
--------------

Settings for lights.

VSettingsLight has the following items:

* | **constantAttenuation** [type = float, default = 1.0, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.constantAttenuation``\ 
  | constant attenuation coefficient of GL_LIGHT[0,1,2,3], this is a constant factor that attenuates the light source; attenuation factor = 1/(kc +kl*d + kq*d*d); (kc,kl,kq)=(1,0,0) means no attenuation; only used for lights, where last component of light position is 1
* | **diffuse** [type = float, default = 0.5, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.diffuse``\ 
  | diffuse value of GL_LIGHT[0,1,2,3]
* | **enable** [type = bool, default = True, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.enable``\ 
  | turn on/off light
* | **lightRadius** [type = float, default = 0.1, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.lightRadius``\ 
  | only used by raytracers: radius of light used to compute smooth shadows (approximated by raytracer.lightRadiusVariations); if lightRadiusVariations>1, this value defines the radius of the light, converting point lights into distributed lights (slower)
* | **linearAttenuation** [type = float, default = 0.0, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.linearAttenuation``\ 
  | linear attenuation coefficient of GL_LIGHT[0,1,2,3], this is a linear factor for attenuation of the light source with distance
* | **position** [type = Float4, default = [2.,2.,10.,0.], size = 4]:
  | \ ``SC.visualizationSettings.openGL.light.position``\ 
  | 4D position vector of GL_LIGHT[0,1,2,3]; 4th value should be 0 for directional lights that are (almost) infinitely far away, like the sun, but 1 for position-based lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position to be located at a reasonable location; the openGL renderer uses shadow volumes and approximates directional lights by enlarging the direction to 200 times maxSceneSize, while the raytracer uses the correct direction; see opengl manuals
* | **quadraticAttenuation** [type = float, default = 0.0, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.quadraticAttenuation``\ 
  | quadratic attenuation coefficient of GL_LIGHT[0,1,2,3], this is a quadratic factor for attenuation of the light source with distance
* | **shadow** [type = UFloat, default = 0.]:
  | \ ``SC.visualizationSettings.openGL.light.shadow``\ 
  | in OpenGL renderer, the shadow parameter \ :math:`\in [0 ... 1]`\  prescribes amount of shadow of light [0,1,2,3] that is added to the scene, using light position (or only direction), accumulating for each light; if this parameter is different from 0, rendering of triangles becomes approx.\ 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows; for raytracer, shadow is included by a physics-based model for each light if shadow>0, accumulating effects of each light source
* | **specular** [type = float, default = 0.5, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.specular``\ 
  | specular value of GL_LIGHT[0,1,2,3]
* | **useCameraFrame** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.light.useCameraFrame``\ 
  | set False to set light positions and directions relative to model frame; True: lights are in camera frame, not following the visual transformations; this was True up to Exudyn 1.9.174



.. _sec-vsettingsopengl:

VSettingsOpenGL
---------------

OpenGL settings for 2D and 3D rendering - with many settings also used for raytracer. For further details and backgrounds also see OpenGL 1.3 functionality on the web.

VSettingsOpenGL has the following items:

* | **advanced** [type = VSettingsOpenGLAdvanced]:
  | \ ``SC.visualizationSettings.openGL.advanced``\ 
  | advanced settings for openGL
* | **light0** [type = VSettingsLight]:
  | \ ``SC.visualizationSettings.openGL.light0``\ 
  | settings for light0 and shadow
* | **light1** [type = VSettingsLight]:
  | \ ``SC.visualizationSettings.openGL.light1``\ 
  | settings for light1 and shadow
* | **light2** [type = VSettingsLight]:
  | \ ``SC.visualizationSettings.openGL.light2``\ 
  | settings for light2 and shadow
* | **light3** [type = VSettingsLight]:
  | \ ``SC.visualizationSettings.openGL.light3``\ 
  | settings for light3 and shadow
* | **drawFaceNormals** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.drawFaceNormals``\ 
  | draws triangle normals, e.g. at center of triangles; used for debugging of faces
* | **drawNormalsLength** [type = PFloat, default = 0.1, size = 1]:
  | \ ``SC.visualizationSettings.openGL.drawNormalsLength``\ 
  | length of normals; used for debugging
* | **drawVertexNormals** [type = bool, default = False, size = 1]:
  | \ ``SC.visualizationSettings.openGL.drawVertexNormals``\ 
  | draws vertex normals; used for debugging
* | **faceEdgesColor** [type = Float4, default = [0.2,0.2,0.2,1.], size = 4]:
  | \ ``SC.visualizationSettings.openGL.faceEdgesColor``\ 
  | global RGBA color for face edges
* | **faceTransparencyGlobal** [type = UFloat, default = 0.4, size = 1]:
  | \ ``SC.visualizationSettings.openGL.faceTransparencyGlobal``\ 
  | in case that facesTransparent=True this represents the max alpha-transparency
* | **lightModelAmbient** [type = Float4, default = [0.4,0.4,0.4,1.], size = 4]:
  | \ ``SC.visualizationSettings.openGL.lightModelAmbient``\ 
  | global ambient light (needed for faces that are close to orthogonal to light or faces in shadow region); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a]); also used by raytracer
* | **lineWidth** [type = UFloat, default = 1., size = 1]:
  | \ ``SC.visualizationSettings.openGL.lineWidth``\ 
  | width of lines used for representation of lines, circles, points, etc.
* | **materialShininess** [type = float, default = 32., size = 1]:
  | \ ``SC.visualizationSettings.openGL.materialShininess``\ 
  | shininess of material
* | **materialSpecular** [type = Float4, default = [0.6,0.6,0.6,1.], size = 4]:
  | \ ``SC.visualizationSettings.openGL.materialSpecular``\ 
  | RGBA specular color of material
* | **multiSampling** [type = PInt, default = 1, size = 1]:
  | \ ``SC.visualizationSettings.openGL.multiSampling``\ 
  | NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 3, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!
* | **zMaxSceneFactor** [type = PFloat, default = 2.]:
  | \ ``SC.visualizationSettings.openGL.zMaxSceneFactor``\ 
  | factor multiplied with maxSceneSize to avoid clipping of modelview; larger values reduce clipping of near or far objects, but may lead to artifacts (so-called Z-fighting)



.. _sec-vsettingsexportimages:

VSettingsExportImages
---------------------

Functionality to export images of view0 to files (PNG or TGA format) which can be used to create animations; in order to activate image recording during the solution process, set SolutionSettings.recordImagesInterval accordingly.

VSettingsExportImages has the following items:

* | **heightAlignment** [type = PInt, default = 2]:
  | \ ``SC.visualizationSettings.exportImages.heightAlignment``\ 
  | alignment of exported image height; using a value of 2 helps to reduce problems with video conversion (additional horizontal lines are lost)
* | **saveImageAsTextCircles** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.exportImages.saveImageAsTextCircles``\ 
  | export circles in save image (only in TXT format)
* | **saveImageAsTextLines** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.exportImages.saveImageAsTextLines``\ 
  | export lines in save image (only in TXT format)
* | **saveImageAsTextTexts** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.exportImages.saveImageAsTextTexts``\ 
  | export text in save image (only in TXT format)
* | **saveImageAsTextTriangles** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.exportImages.saveImageAsTextTriangles``\ 
  | export triangles in save image (only in TXT format)
* | **saveImageFileCounter** [type = UInt, default = 0]:
  | \ ``SC.visualizationSettings.exportImages.saveImageFileCounter``\ 
  | current value of the counter which is used to consecutively save frames (images) with consecutive numbers
* | **saveImageFileName** [type = FileName, default = 'images/frame']:
  | \ ``SC.visualizationSettings.exportImages.saveImageFileName``\ 
  | filename (without extension!) and (relative) path for image file(s) with consecutive numbering (e.g., frame0000.png, frame0001.png,...); ; directory will be created if it does not exist
* | **saveImageFormat** [type = String, default = 'PNG']:
  | \ ``SC.visualizationSettings.exportImages.saveImageFormat``\ 
  | format for exporting figures: currently only PNG, TGA and TXT available; while PNG and TGA represent the according image file formats, the TXT format results in a text file containing the 3D graphics data information as lists of lines, triangles, etc; PNG is not available for Ubuntu18.04 (check  use TGA has highest compatibility with all platforms
* | **saveImageSingleFile** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.exportImages.saveImageSingleFile``\ 
  | True: only save single files with given filename, not adding numbering; False: add numbering to files, see saveImageFileName
* | **saveImageTimeOut** [type = PInt, default = 5000]:
  | \ ``SC.visualizationSettings.exportImages.saveImageTimeOut``\ 
  | timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes
* | **widthAlignment** [type = PInt, default = 4]:
  | \ ``SC.visualizationSettings.exportImages.widthAlignment``\ 
  | alignment of exported image width; using a value of 4 helps to reduce problems with video conversion (additional vertical lines are lost)



.. _sec-vsettingsopenvr:

VSettingsOpenVR
---------------

Functionality to interact openVR; requires special hardware or software emulator, see steam / openVR descriptions. 

VSettingsOpenVR has the following items:

* | **actionManifestFileName** [type = FileName, default = 'C:/openVRactionsManifest.json']:
  | \ ``SC.visualizationSettings.interactive.openVR.actionManifestFileName``\ 
  | This string must contain a string representing a valid absolute path to a vr_actions.json manifest, which describes all HMD, tracker, etc. devices as given by openVR
* | **enable** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.interactive.openVR.enable``\ 
  | True: openVR enabled (if compiled with according flag and installed openVR)
* | **logLevel** [type = Int, default = 1]:
  | \ ``SC.visualizationSettings.interactive.openVR.logLevel``\ 
  | integer value setting log level of openVR: -1 (no output), 0 (error), 1 (warning), 2 (info), 3 (debug); increase log level to get more output
* | **showCompanionWindow** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.interactive.openVR.showCompanionWindow``\ 
  | True: openVR will show companion window containing left and right eye view



.. _sec-vsettingsinteractiveadvanced:

VSettingsInteractiveAdvanced
----------------------------

Advanced settings for interactive.

VSettingsInteractiveAdvanced has the following items:

* | **highlightColor** [type = Float4, default = [0.8,0.05,0.05,0.75], size = 4]:
  | \ ``SC.visualizationSettings.interactive.advanced.highlightColor``\ 
  | RGBA color for highlighted item; 4th value is alpha-transparency
* | **highlightOtherColor** [type = Float4, default = [0.5,0.5,0.5,0.4], size = 4]:
  | \ ``SC.visualizationSettings.interactive.advanced.highlightOtherColor``\ 
  | RGBA color for other items (which are not highlighted); 4th value is alpha-transparency
* | **joystickScaleRotation** [type = float, default = 200.]:
  | \ ``SC.visualizationSettings.interactive.advanced.joystickScaleRotation``\ 
  | rotation scaling factor for joystick input
* | **joystickScaleTranslation** [type = float, default = 6.]:
  | \ ``SC.visualizationSettings.interactive.advanced.joystickScaleTranslation``\ 
  | translation scaling factor for joystick input
* | **keypressRotationStep** [type = float, default = 5.]:
  | \ ``SC.visualizationSettings.interactive.advanced.keypressRotationStep``\ 
  | rotation increment per keypress in degree (full rotation = 360 degree)
* | **keypressTranslationStep** [type = float, default = 0.1]:
  | \ ``SC.visualizationSettings.interactive.advanced.keypressTranslationStep``\ 
  | translation increment per keypress relative to window size
* | **mouseMoveRotationFactor** [type = float, default = 1.]:
  | \ ``SC.visualizationSettings.interactive.advanced.mouseMoveRotationFactor``\ 
  | rotation increment per 1 pixel mouse movement in degree
* | **pauseWithSpacebar** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.interactive.advanced.pauseWithSpacebar``\ 
  | True: during simulation, space bar can be pressed to pause simulation
* | **selectionHighlights** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.interactive.advanced.selectionHighlights``\ 
  | True: enable mouse click to highlights item (default: red)
* | **selectionLeftMouse** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.interactive.advanced.selectionLeftMouse``\ 
  | True: enable left mouse click on items to show basic information
* | **selectionLeftMouseItemTypes** [type = Index, default = 31]:
  | \ ``SC.visualizationSettings.interactive.advanced.selectionLeftMouseItemTypes``\ 
  | binary flags (1,2,4,8,16) for (Node,Object,Marker,Load,Sensor) that are identified with left mouse click selection
* | **selectionRightMouse** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.interactive.advanced.selectionRightMouse``\ 
  | True: enable right mouse click on items to show dictionary (read only!)
* | **selectionRightMouseGraphicsData** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.interactive.advanced.selectionRightMouseGraphicsData``\ 
  | True: right mouse click on items also shows GraphicsData information for inspectation (may sometimes be very large and may not fit into dialog for large graphics objects!)
* | **zoomStepFactor** [type = float, default = 1.15]:
  | \ ``SC.visualizationSettings.interactive.advanced.zoomStepFactor``\ 
  | change of zoom per keypress (keypad +/-) or mouse wheel increment



.. _sec-vsettingsinteractive:

VSettingsInteractive
--------------------

Functionality to interact with render window; includes special rotation and zoom factors, item-highlighting, marker tracking, item selection and keyPressUserFunction.

VSettingsInteractive has the following items:

* | **advanced** [type = VSettingsInteractiveAdvanced]:
  | \ ``SC.visualizationSettings.interactive.advanced``\ 
  | advanced interactive visualization settings
* | **openVR** [type = VSettingsOpenVR]:
  | \ ``SC.visualizationSettings.interactive.openVR``\ 
  | openVR visualization settings
* | **autoRotateModelView** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.interactive.autoRotateModelView``\ 
  | True: rotate model view with autorotation
* | **autoRotationVelocity** [type = Float3, default = [0.,0.,1.047198], size = 3]:
  | \ ``SC.visualizationSettings.interactive.autoRotationVelocity``\ 
  | Angular velocity vector for auto-rotation of scene (only visualization view is rotated, not the model itself!)
* | **highlightItemIndex** [type = Int, default = -1]:
  | \ ``SC.visualizationSettings.interactive.highlightItemIndex``\ 
  | index of item that shall be highlighted (e.g., to find item which cauess problems); if set -1, no item is highlighted
* | **highlightItemType** [type = ItemType, default = ItemType::\_None]:
  | \ ``SC.visualizationSettings.interactive.highlightItemType``\ 
  | item type (Node, Object, ...) that shall be highlighted (e.g., to find item which cauess problems)
* | **highlightMbsNumber** [type = UInt, default = 0]:
  | \ ``SC.visualizationSettings.interactive.highlightMbsNumber``\ 
  | index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)
* | **ignoreKeys** [type = bool, default = False]:
  | \ ``SC.visualizationSettings.interactive.ignoreKeys``\ 
  | True: ignore keyboard input except escape and 'F2' keys; used for interactive mode, e.g., to perform kinematic analysis; This flag can be switched with key 'F2'; if ignoreKeys=True, then keyPressUserFunction can be used!
* | **keyPressUserFunction** [type = KeyPressUserFunction, default = 0]:
  | \ ``SC.visualizationSettings.interactive.keyPressUserFunction``\ 
  | add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example:  def f(key, action, mods): \phantom{XXX} print('key=',key); use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)
* | **logMouseCoordinates** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.interactive.logMouseCoordinates``\ 
  | True: if showMouseCoordinates=True, also log mouse coordinates (transformed to model coordinates); only works for axis-aligned ortho-projections and shows the coordinates of the current plane
* | **useJoystickInput** [type = bool, default = True]:
  | \ ``SC.visualizationSettings.interactive.useJoystickInput``\ 
  | True: read joystick input (use 6-axis joystick with lowest ID found when starting renderer window) and interpret as (x,y,z) position and (rotx, roty, rotz) rotation: as available from 3Dconnexion space mouse and maybe others as well; set to False, if external joystick makes problems ...



.. _sec-visualizationsettings:

VisualizationSettings
---------------------

Top structure for all visualization settings in Exudyn. 

VisualizationSettings has the following items:

* | **bodies** [type = VSettingsBodies]:
  | \ ``SC.visualizationSettings.bodies``\ 
  | body visualization settings
* | **connectors** [type = VSettingsConnectors]:
  | \ ``SC.visualizationSettings.connectors``\ 
  | connector visualization settings
* | **contact** [type = VSettingsContact]:
  | \ ``SC.visualizationSettings.contact``\ 
  | contact visualization settings
* | **contour** [type = VSettingsContour]:
  | \ ``SC.visualizationSettings.contour``\ 
  | contour plot visualization settings
* | **dialogs** [type = VSettingsDialogs]:
  | \ ``SC.visualizationSettings.dialogs``\ 
  | dialogs settings
* | **exportImages** [type = VSettingsExportImages]:
  | \ ``SC.visualizationSettings.exportImages``\ 
  | settings for exporting (saving) images to files in order to create animations
* | **general** [type = VSettingsGeneral]:
  | \ ``SC.visualizationSettings.general``\ 
  | general visualization settings
* | **interactive** [type = VSettingsInteractive]:
  | \ ``SC.visualizationSettings.interactive``\ 
  | Settings for interaction with renderer
* | **loads** [type = VSettingsLoads]:
  | \ ``SC.visualizationSettings.loads``\ 
  | load visualization settings
* | **markers** [type = VSettingsMarkers]:
  | \ ``SC.visualizationSettings.markers``\ 
  | marker visualization settings
* | **nodes** [type = VSettingsNodes]:
  | \ ``SC.visualizationSettings.nodes``\ 
  | node visualization settings
* | **openGL** [type = VSettingsOpenGL]:
  | \ ``SC.visualizationSettings.openGL``\ 
  | OpenGL rendering settings
* | **raytracer** [type = VSettingsRaytracer]:
  | \ ``SC.visualizationSettings.raytracer``\ 
  | Raytracer settings (builds on OpenGL rendering settings)
* | **sensors** [type = VSettingsSensors]:
  | \ ``SC.visualizationSettings.sensors``\ 
  | sensor visualization settings
* | **view0** [type = VSettingsView]:
  | \ ``SC.visualizationSettings.view0``\ 
  | Settings for main view 0
* | **view1** [type = VSettingsView]:
  | \ ``SC.visualizationSettings.view1``\ 
  | Settings for sub-view 1
* | **view2** [type = VSettingsView]:
  | \ ``SC.visualizationSettings.view2``\ 
  | Settings for sub-view 2
* | **view3** [type = VSettingsView]:
  | \ ``SC.visualizationSettings.view3``\ 
  | Settings for sub-view 3

The following parameter changes have been made:

  - visualizationSettings.general.drawCoordinateSystem → visualizationSettings.view0.scene.drawCoordinateSystem (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.general.drawWorldBasis → visualizationSettings.view0.scene.drawWorldBasis (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.general.showComputationInfo → visualizationSettings.view0.window.showComputationInfo (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.general.textSize → visualizationSettings.view0.window.globalFontSize (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.general.worldBasisSize → visualizationSettings.view0.scene.worldBasisSize (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.contour.colorBarPrecision → visualizationSettings.contour.advanced.colorBarPrecision (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.contour.colorBarTiling → visualizationSettings.contour.advanced.colorBarTiling (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.contour.showColorBar → visualizationSettings.contour.advanced.showColorBar (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.alwaysOnTop → visualizationSettings.view0.window.alwaysOnTop (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.ignoreKeys → visualizationSettings.interactive.ignoreKeys (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.keyPressUserFunction → visualizationSettings.interactive.keyPressUserFunction (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.limitWindowToScreenSize → visualizationSettings.general.limitWindowToScreenSize (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.maximize → visualizationSettings.view0.window.maximize (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.reallyQuitTimeLimit → visualizationSettings.general.reallyQuitTimeLimit (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.renderWindowSize → visualizationSettings.view0.window.renderWindowSize (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.showMouseCoordinates → visualizationSettings.view0.window.showMouseCoordinates (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.showRenderStateInfo → visualizationSettings.view0.window.showRenderStateInfo (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.showWindow → visualizationSettings.view0.window.showWindow (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.window.startupTimeout → visualizationSettings.general.rendererStartupTimeout (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.ambientLightColor → visualizationSettings.openGL.lightModelAmbient (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.backgroundColorReflections → visualizationSettings.raytracer.advanced.backgroundColorReflections (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.enable → visualizationSettings.view0.camera.useRaytracer (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.lightRadius → visualizationSettings.openGL.light0.lightRadius (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.searchTreeFactor → visualizationSettings.raytracer.advanced.searchTreeFactor (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.shadowScalingFactor → visualizationSettings.raytracer.advanced.shadowScalingFactor (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.shadowSmoothingSteps → visualizationSettings.raytracer.advanced.shadowSmoothingSteps (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.showText → visualizationSettings.raytracer.advanced.showText (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.tilesPerThread → visualizationSettings.raytracer.advanced.tilesPerThread (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.zBiasLines → visualizationSettings.raytracer.advanced.zBiasLines (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.raytracer.zOffsetCamera → visualizationSettings.openGL.dummy (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.clippingPlaneColor → visualizationSettings.openGL.advanced.clippingPlaneColor (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.clippingPlaneDistance → visualizationSettings.view0.camera.clippingPlaneDistance (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.clippingPlaneNormal → visualizationSettings.view0.camera.clippingPlaneNormal (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.depthSorting → visualizationSettings.openGL.advanced.depthSorting (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.enableLight0 → visualizationSettings.openGL.light0.enable (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.enableLight1 → visualizationSettings.openGL.light1.enable (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.enableLighting → visualizationSettings.openGL.advanced.enableLighting (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.facesTransparent → visualizationSettings.view0.scene.facesTransparent (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.initialCenterPoint → visualizationSettings.openGL.advanced.initialCenterPoint (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.initialMaxSceneSize → visualizationSettings.openGL.advanced.initialMaxSceneSize (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.initialModelRotation → visualizationSettings.openGL.advanced.initialModelRotation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.initialZoom → visualizationSettings.openGL.advanced.initialZoom (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light0ambient → visualizationSettings.openGL.dummy (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light0constantAttenuation → visualizationSettings.openGL.light0.constantAttenuation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light0diffuse → visualizationSettings.openGL.light0.diffuse (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light0linearAttenuation → visualizationSettings.openGL.light0.linearAttenuation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light0position → visualizationSettings.openGL.light0.position (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light0quadraticAttenuation → visualizationSettings.openGL.light0.quadraticAttenuation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light0specular → visualizationSettings.openGL.light0.specular (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light1ambient → visualizationSettings.openGL.dummy (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light1constantAttenuation → visualizationSettings.openGL.light1.constantAttenuation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light1diffuse → visualizationSettings.openGL.light1.diffuse (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light1linearAttenuation → visualizationSettings.openGL.light1.linearAttenuation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light1position → visualizationSettings.openGL.light1.position (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light1quadraticAttenuation → visualizationSettings.openGL.light1.quadraticAttenuation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.light1specular → visualizationSettings.openGL.light1.specular (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.lightModelLocalViewer → visualizationSettings.openGL.advanced.lightModelLocalViewer (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.lightModelTwoSide → visualizationSettings.openGL.advanced.lightModelTwoSide (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.lightPositionsInCameraFrame → visualizationSettings.openGL.light0.useCameraFrame (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.lineSmooth → visualizationSettings.openGL.advanced.lineSmooth (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.materialAmbientAndDiffuse → visualizationSettings.openGL.materialSpecular (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.perspective → visualizationSettings.view0.camera.perspective (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.polygonOffset → visualizationSettings.openGL.advanced.polygonOffset (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.shadeModelSmooth → visualizationSettings.openGL.advanced.shadeModelSmooth (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.shadow → visualizationSettings.openGL.light0.shadow (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.shadowPolygonOffset → visualizationSettings.openGL.advanced.shadowPolygonOffset (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.showFaceEdges → visualizationSettings.view0.scene.showFaceEdges (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.showFaces → visualizationSettings.view0.scene.showFaces (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.showLines → visualizationSettings.view0.scene.showLines (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.showMeshEdges → visualizationSettings.view0.scene.showMeshEdges (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.showMeshFaces → visualizationSettings.view0.scene.showMeshFaces (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.textLineSmooth → visualizationSettings.openGL.advanced.textLineSmooth (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.openGL.textLineWidth → visualizationSettings.openGL.advanced.textLineWidth (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.highlightColor → visualizationSettings.interactive.advanced.highlightColor (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.highlightOtherColor → visualizationSettings.interactive.advanced.highlightOtherColor (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.joystickScaleRotation → visualizationSettings.interactive.advanced.joystickScaleRotation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.joystickScaleTranslation → visualizationSettings.interactive.advanced.joystickScaleTranslation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.keypressRotationStep → visualizationSettings.interactive.advanced.keypressRotationStep (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.keypressTranslationStep → visualizationSettings.interactive.advanced.keypressTranslationStep (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.lockModelView → visualizationSettings.view0.window.lockModelView (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.mouseMoveRotationFactor → visualizationSettings.interactive.advanced.mouseMoveRotationFactor (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.pauseWithSpacebar → visualizationSettings.interactive.advanced.pauseWithSpacebar (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.selectionHighlights → visualizationSettings.interactive.advanced.selectionHighlights (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.selectionLeftMouse → visualizationSettings.interactive.advanced.selectionLeftMouse (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.selectionLeftMouseItemTypes → visualizationSettings.interactive.advanced.selectionLeftMouseItemTypes (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.selectionRightMouse → visualizationSettings.interactive.advanced.selectionRightMouse (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.selectionRightMouseGraphicsData → visualizationSettings.interactive.advanced.selectionRightMouseGraphicsData (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.trackMarker → visualizationSettings.view0.camera.trackMarker (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.trackMarkerMbsNumber → visualizationSettings.view0.camera.trackMarkerMbsNumber (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.trackMarkerOrientation → visualizationSettings.view0.camera.trackMarkerOrientation (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.trackMarkerPosition → visualizationSettings.view0.camera.trackMarkerPosition (changed in version 1.10.80, expires: 2030)
  - visualizationSettings.interactive.zoomStepFactor → visualizationSettings.interactive.advanced.zoomStepFactor (changed in version 1.10.80, expires: 2030)


