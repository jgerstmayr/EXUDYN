.. role:: textred
.. role:: textorange
.. role:: textblue
.. role:: textgreen
.. role:: boldred
.. role:: boldorange
.. role:: boldblue
.. role:: boldgreen

.. _sec-issuetracker:

=============
Issue tracker
=============

This section contains resolved issues per release and known bugs. Use this information to understand changes compared to previous versions. The author field is omitted if it was Johannes Gerstmayr (JG).
The extension \ ``.dev1``\  is not added in the issues list (e.g., 1.2.2.dev1==1.2.2), as it only marks versions that will not be available in pypi with standard pip install, but only with the \ ``-``\ \ ``-pre``\  option or by specifying the exact version name, see versions on `https://pypi.org/project/exudyn/ <https://pypi.org/project/exudyn/>`_.
BUG numbers refer to the according issue numbers.

General information on current version:
 
+  Exudyn version = 1.10.0, 
+  last change =  2025-07-09, 
+  Number of issues = 2139, 
+  Number of resolved issues = 1912 (0 in current version), 

************
Version 1.10
************

 * Version 1.10.0: resolved Issue 2123: Mechanisms (extension)
    - description:  Add example for gear and toothed rack mounted on body, using relative translation/rotation marker
    - **notes:** see Example involuteGearGraphics.py
    - date resolved: **2025-07-09 18:43**\ , date raised: 2025-07-02 

***********
Version 1.9
***********

 * Version 1.9.235: resolved Issue 1959: SphereSphereContact (check)
    - description:  check that position marker works without friction
    - **notes:** not needed any more as mass points and point nodes include rotation matrix and angular velocity
    - date resolved: **2025-07-09 18:12**\ , date raised: 2025-02-24 
 * Version 1.9.234: resolved Issue 2122: Mechanisms (extension)
    - description:  Add example for two gears mounted on a body with revolute joints, using CoordinateSpringDamperExt
    - **notes:** see Example involuteGearGraphics.py
    - date resolved: **2025-07-09 18:10**\ , date raised: 2025-07-02 
 * Version 1.9.233: resolved Issue 2137: general open source (docu)
    - description:  add some warnings at beginning of docu regarding open source and potential errors
    - date resolved: **2025-07-09 18:07**\ , date raised: 2025-07-05 
 * Version 1.9.232: resolved Issue 2138: RST, latex (docu)
    - description:  fill issues with references in RST and latex files
    - date resolved: **2025-07-05 18:29**\ , date raised: 2025-07-05 
 * Version 1.9.231: resolved Issue 2135: TestSuite (fix)
    - description:  put all variables into static TestSuite class, to avoid unintentionally changing variables in the testsuite scope by models
    - date resolved: **2025-07-05 15:07**\ , date raised: 2025-07-05 
 * Version 1.9.230: resolved Issue 2136: trackerlog (change)
    - description:  change to utf-8 format; this may cause problems when loading old (backup) files, which shall be manually changed to utf-8
    - date resolved: **2025-07-05 14:54**\ , date raised: 2025-07-05 
 * Version 1.9.229: resolved Issue 2134: ContactSphereTriangle (testing)
    - description:  add test example using CreateSphereQuadContact with two contacting bodies to check momentum conservation
    - date resolved: **2025-07-05 12:10**\ , date raised: 2025-07-05 
 * Version 1.9.228: resolved Issue 2133: ContactSphereTriangle (fix)
    - description:  marker transformation missing for contact computation
    - date resolved: **2025-07-05 11:10**\ , date raised: 2025-07-05 
 * Version 1.9.227: resolved Issue 2132: nodes (change)
    - description:  letter N for basis only shown in case that nodes.showNumbers=True
    - date resolved: **2025-07-05 10:46**\ , date raised: 2025-07-05 
 * Version 1.9.226: resolved Issue 2129: MarkerBodiesRelativeRotationCoordinate (fix)
    - description:  does not work without nodeNumber provided
    - date resolved: **2025-07-03 22:21**\ , date raised: 2025-07-03 
 * Version 1.9.225: resolved Issue 2121: Involute gear (extension)
    - description:  add function to generate graphics for toothed rack
    - date resolved: **2025-07-03 18:33**\ , date raised: 2025-07-02 
 * Version 1.9.224: resolved Issue 2128: SolidExtrusion (extension)
    - description:  add relative rotation and relative offset for second surface of extrusion; used e.g. for helical gears
    - date resolved: **2025-07-03 16:48**\ , date raised: 2025-07-03 
 * Version 1.9.223: resolved Issue 2126: Involute gear (extension)
    - description:  add function to create involute gear graphics
    - date resolved: **2025-07-03 15:37**\ , date raised: 2025-07-03 
 * Version 1.9.222: resolved Issue 2125: BallBearing (extension)
    - description:  add function to create ball bearing into machines; similar to issue 2031
    - date resolved: **2025-07-03 14:57**\ , date raised: 2025-07-03 
 * Version 1.9.221: resolved Issue 2118: Involute gear (extension)
    - description:  add function to create involute gear profile
    - date resolved: **2025-07-03 12:57**\ , date raised: 2025-07-02 
 * Version 1.9.220: resolved Issue 2031: BallBearing (extension)
    - description:  put function into machines module; function shalll take two markers (for outer and inner ring), which represent the interface to other bodies; add simple test model
    - date resolved: **2025-07-03 09:20**\ , date raised: 2025-05-18 
 * Version 1.9.219: resolved Issue 2124: BallBearing (extension)
    - description:  add graphics function to create rings of ball bearings
    - date resolved: **2025-07-03 09:19**\ , date raised: 2025-07-03 
 * Version 1.9.218: resolved Issue 2119: Involute gear (extension)
    - description:  add function to generate graphics for involute gear
    - date resolved: **2025-07-02 09:40**\ , date raised: 2025-07-02 
 * Version 1.9.217: resolved Issue 2117: MarkerRelative (extension)
    - description:  add offset to MarkerBodiesRelativeRotationCoordinate
    - date resolved: **2025-07-01 23:00**\ , date raised: 2025-07-01 
 * Version 1.9.216: resolved Issue 2112: MarkerRelativeRotation (extension)
    - description:  Consider marker MarkerBodiesRelativeRotationCoordinate which measures scalar relative rotation, in particular rotation relative between two bodies; action is realized as torques on two bodies, using body-fixed coordinates of body0
    - date resolved: **2025-07-01 20:09**\ , date raised: 2025-06-25 
 * Version 1.9.215: resolved Issue 2116: Raytracer (extension)
    - description:  add line color
    - date resolved: **2025-07-01 14:22**\ , date raised: 2025-07-01 
 * Version 1.9.214: resolved Issue 2115: Raytracer (extension)
    - description:  make raytracer compatible with facesTransparent, showFaces, showFaceEdges
    - date resolved: **2025-07-01 14:22**\ , date raised: 2025-07-01 
 * Version 1.9.213: resolved Issue 2114: FEM (extension)
    - description:  add function GetRigidBodyInertia to return RigidBodyInertia of nodal position-based FEM models
    - date resolved: **2025-06-30 14:30**\ , date raised: 2025-06-30 
    - resolved by: S. Weyrer
 * Version 1.9.212: resolved Issue 2111: MarkerRelativeTranslation (extension)
    - description:  Consider marker MarkerBodiesRelativeTranslationCoordinate which measures scalar relative translation, in particular relative displacement between two bodies; action is realized as forces on two bodies, using body-fixed coordinates of body0
    - date resolved: **2025-06-29 23:16**\ , date raised: 2025-06-25 
 * Version 1.9.211: resolved Issue 1961: CreateSphereQuadContact (extension)
    - description:  add create function, which uses 2 CreateSphereTriangleContact elements; should be practical for simple robots, etc.
    - date resolved: **2025-06-29 12:01**\ , date raised: 2025-02-24 
 * Version 1.9.210: resolved Issue 1958: CreateSphereSphereContact (extension)
    - description:  add Create function, including data node; similar to spring-damper; add checks that in case of friction, a rigid body marker is required
    - date resolved: **2025-06-29 11:06**\ , date raised: 2025-02-24 
 * Version 1.9.209: resolved Issue 2113: ExportSTL (fix)
    - description:  invertNormals and invertTriangles not used
    - date resolved: **2025-06-25 17:18**\ , date raised: 2025-06-25 
 * Version 1.9.208: resolved Issue 2050: parallel (fix)
    - description:  check global counters in parallelized computations which cause huge cache issues
    - **notes:** current issues seem to be related to fetch_add (use task stealing?) and cache-polution between different PARALLEL_FOR loops
    - date resolved: **2025-06-24 23:32**\ , date raised: 2025-05-26 
 * Version 1.9.207: resolved Issue 2108: ContactSphereTriangle (testing)
    - description:  test with explicit integration
    - date resolved: **2025-06-22 16:54**\ , date raised: 2025-06-21 
 * Version 1.9.206: resolved Issue 2107: ContactSphereSphere (testing)
    - description:  test with explicit integration
    - date resolved: **2025-06-22 16:54**\ , date raised: 2025-06-21 
 * Version 1.9.205: resolved Issue 2105: Examples and TestModels (fix)
    - description:  remove uncommitted test files from Examples and TestModels as they are included as examples in theDoc
    - date resolved: **2025-06-21 22:48**\ , date raised: 2025-06-19 
 * Version 1.9.204: resolved Issue 2102: automatic code (change)
    - description:  avoid writing dates to systemstrutures files and others
    - date resolved: **2025-06-19 00:22**\ , date raised: 2025-06-18 
 * Version 1.9.203: resolved Issue 2018: docu (docu)
    - description:  when searching for keywords in examples, exclude matches (like MassMatrix) where the character before or after the keyword is a letter (a-zA-Z) to avoid wrong matches!
    - date resolved: **2025-06-18 22:22**\ , date raised: 2025-05-15 
 * Version 1.9.202: resolved Issue 2104: FEMinterface (fix)
    - description:  add try except to automatic nodeSet creation in case that there are no boundaries computable
    - date resolved: **2025-06-18 18:17**\ , date raised: 2025-06-18 
 * Version 1.9.201: resolved Issue 2103: Item selection (extension)
    - description:  write currently selected item (+type, etc.) into renderer.state
    - date resolved: **2025-06-18 17:00**\ , date raised: 2025-06-18 
 * Version 1.9.200: resolved Issue 2101: parallel (extension)
    - description:  add simulationSettings.parallel.useLoadBalancing to switch two multithreading modes
    - date resolved: **2025-06-18 12:00**\ , date raised: 2025-06-18 
 * Version 1.9.199: resolved Issue 1695: taskmanager (change)
    - description:  extend microthreading for taskmanager-based load management; remove taskmanager from repo and create pure BSD license
    - **notes:** in C++ ExuThreading being now the only mode; optionally use exu.special.solver.multiThreadingLoadBalancing to switch load balancing on/off
    - date resolved: **2025-06-18 10:00**\ , date raised: 2023-11-19 
 * Version 1.9.198: resolved Issue 2100: Command window (fix)
    - description:  does not show edit dialog
    - date resolved: **2025-06-16 14:52**\ , date raised: 2025-06-16 
 * Version 1.9.197: resolved Issue 2099: contour (extension)
    - description:  add alpha channel visualizationSettings option for contour colors
    - date resolved: **2025-06-16 11:28**\ , date raised: 2025-06-16 
 * Version 1.9.196: resolved Issue 2098: graphics.CheckerBoard (extension)
    - description:  add arg materialIndex for graphics material of both colors
    - date resolved: **2025-06-16 11:03**\ , date raised: 2025-06-16 
 * Version 1.9.195: resolved Issue 2096: ContactSphereTriangle (testing)
    - description:  add test model sphereTriangleTest.py
    - date resolved: **2025-06-15 19:56**\ , date raised: 2025-06-15 
 * Version 1.9.194: resolved Issue 2095: CreateKinematicTree (testing)
    - description:  add test model
    - **notes:** add test model createKinematicTreeTest.py
    - date resolved: **2025-06-15 19:55**\ , date raised: 2025-06-15 
 * Version 1.9.193: resolved Issue 1960: ObjectContactSphereTriangle (extension)
    - description:  add contact similar to GeneralContact and to ObjectContactSphereSphere, but being able to be computed implicitly; add option to exclude certain edges to be able to also correctly handle meshes
    - date resolved: **2025-06-15 11:50**\ , date raised: 2025-02-24 
 * Version 1.9.192: :textred:`resolved BUG 2094` : ContactSphereSphere 
    - description:  ContactSphereSphere and ContactSphereTorus set frictionRegularizedRegion wrong in case of computeFromData
    - **notes:** now leads to improved convergence
    - date resolved: **2025-06-15 09:37**\ , date raised: 2025-06-15 
 * Version 1.9.191: resolved Issue 2092: FEMinterface (extension)
    - description:  ComputePostProcessingModesNGsolve: extend for multi-material coefficient function, using materials dict; 
    - date resolved: **2025-06-14 10:40**\ , date raised: 2025-06-13 
 * Version 1.9.190: resolved Issue 2090: FEMinterface (extension)
    - description:  extend ImportMeshFromNGsolve to include several materials given as dict of materials
    - date resolved: **2025-06-14 10:31**\ , date raised: 2025-06-13 
 * Version 1.9.189: resolved Issue 2089: FEMinterface (extension)
    - description:  add function to compute nodeSets from NGsolve boundary names: CreateNGsolveBoundaryNodeSets 
    - date resolved: **2025-06-14 10:31**\ , date raised: 2025-06-13 
 * Version 1.9.188: resolved Issue 2088: FEMinterface (extension)
    - description:  add internal function to get node numbers of NGsolve boundary condition: GetNodesOfNGsolveBoundary
    - date resolved: **2025-06-14 10:31**\ , date raised: 2025-06-13 
 * Version 1.9.187: resolved Issue 2093: FEMinterface (extension)
    - description:  adapt class KirchhoffMaterial for multi-domain materials
    - date resolved: **2025-06-14 10:30**\ , date raised: 2025-06-14 
 * Version 1.9.186: resolved Issue 2091: FEMinterface (change)
    - description:  remove deprecated arg computeEigenmodes from ImportMeshFromNGsolve
    - date resolved: **2025-06-13 22:16**\ , date raised: 2025-06-13 
 * Version 1.9.185: resolved Issue 2087: GetOtherMarker (change)
    - description:  return MarkerBodyRigid as body anyway must provide position and orientation
    - date resolved: **2025-06-13 22:13**\ , date raised: 2025-06-13 
 * Version 1.9.184: resolved Issue 2067: CreateKinematicTree (extension)
    - description:  add Create function for KinematicTree, using list of TreeLink; add special class TreeLink to itemInterface
    - date resolved: **2025-06-10 08:52**\ , date raised: 2025-05-29 
 * Version 1.9.183: resolved Issue 2083: SolidOfRevolution (extension)
    - description:  add check to graphics.SolidOfRevolution that order of list is correct (leads to correct inside-outside relations)
    - date resolved: **2025-06-10 01:02**\ , date raised: 2025-06-09 
 * Version 1.9.182: resolved Issue 2086: RigidBodyInertia (extension)
    - description:  Add GetGraphics for combined inertia and graphics generation
    - date resolved: **2025-06-09 12:31**\ , date raised: 2025-06-09 
 * Version 1.9.181: resolved Issue 2056: graphics (fix)
    - description:  Renderer raises warning that some objects contain inconsistencies between computed triangle normals and vertex normals
    - **notes:** mainly due to SolidOfRevolution
    - date resolved: **2025-06-09 01:14**\ , date raised: 2025-05-28 
 * Version 1.9.180: resolved Issue 2085: SolidOfRevolution (change)
    - description:  switch triangle order in SolidOfRevolution to have consistent normals and triangles
    - date resolved: **2025-06-09 01:06**\ , date raised: 2025-06-09 
 * Version 1.9.179: resolved Issue 2084: drawFaceNormals (change)
    - description:  openGL.drawFaceNormals shall draw the computed normal from the triangle points, thus showing the correct orientation of triangles
    - date resolved: **2025-06-09 00:48**\ , date raised: 2025-06-09 
 * Version 1.9.178: resolved Issue 2082: shadow (fix)
    - description:  add lightPositionsInCameraFrame flag to shadow computation to consistently draw shadow and lights in openGL
    - date resolved: **2025-06-08 18:11**\ , date raised: 2025-06-08 
 * Version 1.9.177: resolved Issue 2080: HDF5 (testing)
    - description:  Add test with all data types
    - **notes:** added example testHDF5loadSave.py
    - date resolved: **2025-06-08 18:11**\ , date raised: 2025-06-08 
 * Version 1.9.176: :textred:`resolved BUG 2079` : LoadDictFromHDF5 
    - description:  does not correctly convert np.array inside list
    - date resolved: **2025-06-08 18:11**\ , date raised: 2025-06-07 
 * Version 1.9.175: resolved Issue 2081: GL_LIGHT (extension)
    - description:  add option to switch between local and global lights, to be compatible with Raytracer
    - **notes:** added option openGL.lightPositionsInCameraFrame to switch behavior; in raytracer, this setting is always True
    - date resolved: **2025-06-08 15:44**\ , date raised: 2025-06-08 
 * Version 1.9.174: resolved Issue 2078: netgen (extension)
    - description:  add option to convert netgen / ngsolve mesh into points, triangles and normals including smooth geometries
    - date resolved: **2025-06-07 19:12**\ , date raised: 2025-06-07 
 * Version 1.9.173: resolved Issue 2077: Raytracer (extension)
    - description:  light radius receives circular variation normal to ray, giving an effient and good effect of spherical lights; use lightRadius and lightRadiusVariations
    - date resolved: **2025-06-05 21:03**\ , date raised: 2025-06-05 
 * Version 1.9.172: resolved Issue 2076: Raytracer (extension)
    - description:  add option for spherical lights with radius and random sampling
    - **notes:** added lightRadius
    - date resolved: **2025-06-04 17:52**\ , date raised: 2025-06-04 
 * Version 1.9.171: resolved Issue 2059: Raytracer (docu)
    - description:  add short docu part
    - **notes:** example available in Examples/newtonsCradle.py
    - date resolved: **2025-06-04 10:58**\ , date raised: 2025-05-28 
 * Version 1.9.170: resolved Issue 2074: invert triangles (extension)
    - description:  Add function to invert triangles and normals of graphicsData, e.g., for inverted sphere or brick
    - **notes:** added graphics.InvertTriangles and ConsistentTriangleList
    - date resolved: **2025-06-04 01:37**\ , date raised: 2025-06-02 
 * Version 1.9.169: resolved Issue 2061: Raytracer (extension)
    - description:  add materials interface via SystemContainer: renderer.SetMaterial(index, dict), dict=GetMaterial(index)
    - **notes:** can do read [] access, Set(), New(), etc.
    - date resolved: **2025-06-03 16:06**\ , date raised: 2025-05-28 
 * Version 1.9.168: resolved Issue 2063: Raytracer (extension)
    - description:  make first 10 materials in renderer accessible via visualization systems dialog
    - date resolved: **2025-06-03 16:05**\ , date raised: 2025-05-28 
 * Version 1.9.167: resolved Issue 2060: Raytracer (extension)
    - description:  adjust minZ and line offset to scene dimension
    - date resolved: **2025-06-02 17:45**\ , date raised: 2025-05-28 
 * Version 1.9.166: resolved Issue 2058: Raytracer (extension)
    - description:  add separate visualization options; keep lights
    - date resolved: **2025-06-02 17:45**\ , date raised: 2025-05-28 
 * Version 1.9.165: resolved Issue 2070: Renderer timeout (check)
    - description:  check whether calling glfwWaitEventsTimeout, glfwPollEvents or glfwSwapBuffers fixes problems with redraw timeouts with Raytracing
    - **notes:** can be done with option raytracer.keepWindowActive
    - date resolved: **2025-06-02 17:44**\ , date raised: 2025-05-31 
 * Version 1.9.164: resolved Issue 2069: Raytracer (extension)
    - description:  add global fog and material fog
    - **notes:** only global fog added
    - date resolved: **2025-06-02 17:44**\ , date raised: 2025-05-29 
 * Version 1.9.163: resolved Issue 2068: Raytracer (extension)
    - description:  activate cutting plane for raytracer; include in function IntersectRayWithTriangles, which shall first check for cutting plane and only takes then objects behind cutting plane (if hit); color is then taken from triangle behind cutting plane-without reflections and shadows
    - date resolved: **2025-06-02 17:44**\ , date raised: 2025-05-29 
 * Version 1.9.162: resolved Issue 2073: font bitmaps (extension)
    - description:  Add smoothing at pixel level for base text fonts (at grey scale) to obtain smoother fonts
    - date resolved: **2025-06-01 22:53**\ , date raised: 2025-06-01 
 * Version 1.9.161: resolved Issue 2072: renderer (change)
    - description:  exu.StartRenderer() and exu.StopRenderer() are changed into SC.renderer.Start() and SC.renderer.Stop(), having a SystemContainer SC
    - **notes:** NOTE that this affects your existing models a lot!
    - date resolved: **2025-06-01 19:13**\ , date raised: 2025-06-01 
 * Version 1.9.160: resolved Issue 2071: Raytracer (extension)
    - description:  draw system message texts as overlay of render image
    - date resolved: **2025-06-01 18:00**\ , date raised: 2025-06-01 
 * Version 1.9.159: resolved Issue 2036: renderer (change)
    - description:  put renderer-related functions into SystemContainer; preserve compatibility; consider substructure renderer in SC to collect rendering-related functionality
    - date resolved: **2025-06-01 02:36**\ , date raised: 2025-05-21 
 * Version 1.9.158: resolved Issue 2037: renderer (change)
    - description:  homogenize WaitForUserToContinue, DoRendererIdleTasks, DoIdleOperations and WaitForRenderEngineStopFlag as they are widely identical; homogenize functionality; note that DoRendererIdleTasks() with default args has to be replaced by renderer.DoIdleTasks(0)
    - **notes:** merged into SC.renderer.DoIdleTasks(); NOTE that this change affects your existing models a lot!
    - date resolved: **2025-06-01 02:35**\ , date raised: 2025-05-21 
 * Version 1.9.157: resolved Issue 2062: renderer (extension)
    - description:  add renderer substructure to SystemContainer (with backlink to MainSystemContainer)
    - **notes:** all examples, test models, teaching models and exudyn submodules adapted; old functionality preserved so far with warnings
    - date resolved: **2025-06-01 02:34**\ , date raised: 2025-05-28 
    - resolved by: CHANGE
 * Version 1.9.156: resolved Issue 2045: visualization (extension)
    - description:  use templated function to have only one single function for triangle drawing
    - date resolved: **2025-05-30 01:01**\ , date raised: 2025-05-22 
 * Version 1.9.155: resolved Issue 2066: graphics (extension)
    - description:  add option for special shape of graphics.Brick with rounded edges, offering a transition to an ellipsoid
    - date resolved: **2025-05-30 00:59**\ , date raised: 2025-05-29 
 * Version 1.9.154: resolved Issue 2049: SetRenderState (fix)
    - description:  seems to ignore autoFitScene = False and therefore does not reload zoom factor
    - **notes:** SetRenderState is extended by a flag which by default waits until the renderer is fully started or redrawn; this can impede performance, which is why this should be set to False in such cases
    - date resolved: **2025-05-29 16:22**\ , date raised: 2025-05-26 
 * Version 1.9.153: resolved Issue 2065: SimulationSettings (change)
    - description:  timeIntegration.numberOfSteps: change type to Real in order to accept steps as Python float; add warning in solver if numberOfSteps deviates significantely from integer number
    - date resolved: **2025-05-29 15:33**\ , date raised: 2025-05-29 
 * Version 1.9.152: resolved Issue 2064: exudyn.config (change)
    - description:  move further options to config: SetWriteToConsole, flush always, etc.
    - date resolved: **2025-05-29 14:47**\ , date raised: 2025-05-29 
 * Version 1.9.151: resolved Issue 2057: PrintDelayed (fix)
    - description:  printing from visualiuation thread: buffer is never emptied; use idle tasks to do so
    - date resolved: **2025-05-28 01:12**\ , date raised: 2025-05-28 
 * Version 1.9.150: resolved Issue 2055: exudyn.Print (change)
    - description:  replace print commands in exudyn modules by exudyn.Print(...) in order to have common handling of file writing, etc.
    - date resolved: **2025-05-28 00:08**\ , date raised: 2025-05-28 
 * Version 1.9.149: :textred:`resolved BUG 2054` : GeneticOptimization 
    - description:  computationIndex and parameterFunctionData do not work except for first generation
    - date resolved: **2025-05-27 23:39**\ , date raised: 2025-05-27 
 * Version 1.9.148: resolved Issue 2052: exudyn.Print (extension)
    - description:  function shall accept kwargs end, flush and sep in order to be more compatible with original Python print
    - date resolved: **2025-05-27 22:39**\ , date raised: 2025-05-27 
 * Version 1.9.147: resolved Issue 2053: FEM (fix)
    - description:  ComputePostProcessingModes raises error for larger problems due to print command
    - date resolved: **2025-05-27 22:16**\ , date raised: 2025-05-27 
 * Version 1.9.146: resolved Issue 2051: Raytracer (fix)
    - description:  resolve parallelization issue by removing global hit count
    - date resolved: **2025-05-26 18:58**\ , date raised: 2025-05-26 
 * Version 1.9.145: resolved Issue 2048: Raytracer (extension)
    - description:  add Raytracer as option; map options from openGL to Raytracer (lights, material, multiSampling, shadow, perspective, etc.)
    - date resolved: **2025-05-26 10:51**\ , date raised: 2025-05-26 
 * Version 1.9.144: resolved Issue 2047: graphics.Quad (extension)
    - description:  add normals; also affects graphics.CheckerBoard; normals are not needed, but may be modified in transformations
    - date resolved: **2025-05-25 15:40**\ , date raised: 2025-05-25 
 * Version 1.9.143: resolved Issue 2046: Raytracer (extension)
    - description:  add CPU-based software renderer / raytracer als option to render images for animations
    - **notes:** note that this is experimental; image resolution shall be small (start with 400x300) as long render times may lead to problems
    - date resolved: **2025-05-25 12:32**\ , date raised: 2025-05-25 
 * Version 1.9.142: resolved Issue 2044: visualization (extension)
    - description:  add option to sort transparent triangles to improve quality of transparent objects; use simple depth sort for triangle midpoints
    - **notes:** added option openGL.depthSorting which sorts triangles by their depth for improved transparency view (but requires triangles to be small enough)
    - date resolved: **2025-05-22 16:35**\ , date raised: 2025-05-22 
 * Version 1.9.141: resolved Issue 2043: pyi / stub files (fix)
    - description:  revise section for exudyn stubs in order to enable completion of exudyn.config and other exudyn functions
    - **notes:** seems that removing erroneous info like "special." was sufficient to work
    - date resolved: **2025-05-22 11:00**\ , date raised: 2025-05-22 
 * Version 1.9.140: resolved Issue 2033: InertiaSphere (extension)
    - description:  use density and radius to initialize as alternative option
    - date resolved: **2025-05-22 10:32**\ , date raised: 2025-05-20 
 * Version 1.9.139: resolved Issue 2042: exudyn (change)
    - description:  move exudyn.SetLinalgOutputFormatPython(), exudyn.SetOutputPrecision(), exudyn.SetPrintDelayMilliSeconds(), exudyn.SuppressWarnings(), exudyn.InfoStat(), exudyn.GetVersionString() to according variables and functions in exudyn.config, see current docu
    - date resolved: **2025-05-22 09:30**\ , date raised: 2025-05-22 
 * Version 1.9.138: resolved Issue 2041: exudyn.Solve (change)
    - description:  remove exudyn.SolveStatic exudyn.SolveDynamic and exudyn.ComputeODE2Eigenvalues from exudyn, being only available in MainSystem as mbs.SolveDynamic, etc.
    - date resolved: **2025-05-22 08:35**\ , date raised: 2025-05-22 
 * Version 1.9.137: resolved Issue 2040: exudyn.Demo1() (change)
    - description:  and exudyn.Demo2() only available as exudyn.demos.Demo1() or .Demo2()
    - date resolved: **2025-05-22 07:56**\ , date raised: 2025-05-22 
 * Version 1.9.136: resolved Issue 2039: exudyn.Go() (change)
    - description:  remove function Go() which is not intended to be used
    - date resolved: **2025-05-22 07:44**\ , date raised: 2025-05-22 
 * Version 1.9.135: resolved Issue 2038: exudyn.SetOutputPrecision (change)
    - description:  moved to exudyn.config.outputPrecision
    - date resolved: **2025-05-22 07:21**\ , date raised: 2025-05-22 
 * Version 1.9.134: resolved Issue 1630: exudyn module (extension)
    - description:  consider settings instead of putting all variables globally into module
    - date resolved: **2025-05-22 07:20**\ , date raised: 2023-06-23 
 * Version 1.9.133: resolved Issue 2035: exudyn.config (change)
    - description:  collect specific settings into config structure, see also issue 1630
    - date resolved: **2025-05-22 07:19**\ , date raised: 2025-05-21 
 * Version 1.9.132: resolved Issue 2034: Sensors.traces (extension)
    - description:  add trace time span for past and future traces, to limit length of traces
    - date resolved: **2025-05-20 08:19**\ , date raised: 2025-05-20 
 * Version 1.9.131: resolved Issue 2032: sensor traces (fix)
    - description:  wrong description using visualizationSettings.sensorTraces in docu
    - date resolved: **2025-05-20 08:19**\ , date raised: 2025-05-20 
 * Version 1.9.130: resolved Issue 2028: OutputVariables (extension)
    - description:  add outputvariables to ContactSphereTorus
    - date resolved: **2025-05-19 20:18**\ , date raised: 2025-05-18 
 * Version 1.9.129: resolved Issue 2024: ObjectContactFrictionCircleCable2DOld (change)
    - description:  remove outdated object
    - date resolved: **2025-05-18 23:35**\ , date raised: 2025-05-17 
 * Version 1.9.128: resolved Issue 2030: SolidOfRevolution (extension)
    - description:  add smoothingAngle for contour below which smoothing is applied
    - date resolved: **2025-05-18 15:35**\ , date raised: 2025-05-18 
 * Version 1.9.127: resolved Issue 2029: graphics (extension)
    - description:  add start and end angle for minor radius of torus
    - date resolved: **2025-05-18 11:24**\ , date raised: 2025-05-18 
 * Version 1.9.126: resolved Issue 2022: ContactTorusSphere (extension)
    - description:  add basic contact element; typically used for ball bearings; similar to ContactSphereSphere
    - date resolved: **2025-05-18 00:03**\ , date raised: 2025-05-16 
 * Version 1.9.125: resolved Issue 2027: CreateRigidBody (change)
    - description:  if graphicsDataList is provided, node shall still be shown, but drawsize gets 0; this allows to easily show the node basis
    - **notes:** also done for CreateMassPoint
    - date resolved: **2025-05-17 23:30**\ , date raised: 2025-05-17 
 * Version 1.9.124: resolved Issue 2026: graphics Tube (extension)
    - description:  add function graphics.Tube which generates graphicsData for a tube along a line of points with axis vectors
    - date resolved: **2025-05-17 21:48**\ , date raised: 2025-05-17 
 * Version 1.9.123: resolved Issue 2025: Torus (extension)
    - description:  add graphics function for torus
    - **notes:** using graphics.Tube function
    - date resolved: **2025-05-17 21:48**\ , date raised: 2025-05-17 
 * Version 1.9.122: resolved Issue 1922: ContactSphereSphere (testint)
    - description:  add test model with various contact models
    - date resolved: **2025-05-17 00:15**\ , date raised: 2024-11-02 
 * Version 1.9.121: resolved Issue 2023: graphics.Cylinder (extension)
    - description:  add option to draw hollow cylinder
    - date resolved: **2025-05-17 00:05**\ , date raised: 2025-05-17 
 * Version 1.9.120: resolved Issue 2021: ContactSphereSphere (extension)
    - description:  add option to include hollow sphere - sphere contact
    - date resolved: **2025-05-16 18:10**\ , date raised: 2025-05-16 
 * Version 1.9.119: resolved Issue 2020: version files (change)
    - description:  remove different version files and use only a single file for .tex and .cpp
    - date resolved: **2025-05-15 21:25**\ , date raised: 2025-05-15 
 * Version 1.9.118: resolved Issue 2019: Numpy2.0 (fix)
    - description:  Since Numpy2.0 __cpu_features__ are not available any more, therefore we need to find another way to automatically detect AVX2 features
    - date resolved: **2025-05-15 20:20**\ , date raised: 2025-05-15 
 * Version 1.9.117: resolved Issue 2012: CObjectContactCurveCircles (fix)
    - description:  complete CheckPreAssembleConsistency
    - date resolved: **2025-05-15 08:50**\ , date raised: 2025-05-11 
 * Version 1.9.116: resolved Issue 2014: ContactCurveCircles (extension)
    - description:  check damping mechanism and check negative contact forces
    - date resolved: **2025-05-13 15:12**\ , date raised: 2025-05-11 
 * Version 1.9.115: resolved Issue 1926: ContactCurveCircles (testing)
    - description:  add simple test model
    - date resolved: **2025-05-11 22:53**\ , date raised: 2024-11-04 
 * Version 1.9.114: resolved Issue 2013: ContactCurveCircles (extension)
    - description:  add visualization for contact circles
    - date resolved: **2025-05-11 22:07**\ , date raised: 2025-05-11 
 * Version 1.9.113: resolved Issue 2015: ContactSphereSphere (fix)
    - description:  radius unused in visualization
    - date resolved: **2025-05-11 21:44**\ , date raised: 2025-05-11 
 * Version 1.9.112: resolved Issue 2010: ContactCurveCircles (extension)
    - description:  change dynamic arrays to local temp arrays in CObject, similar to FFRF object
    - date resolved: **2025-05-11 15:36**\ , date raised: 2025-05-11 
 * Version 1.9.111: resolved Issue 2009: ContactCurveCircles (extension)
    - description:  draw active contact segments
    - date resolved: **2025-05-11 12:02**\ , date raised: 2025-05-11 
 * Version 1.9.110: resolved Issue 2008: ContactCurveCircles (extension)
    - description:  draw contact segments
    - date resolved: **2025-05-11 12:02**\ , date raised: 2025-05-11 
 * Version 1.9.109: :textred:`resolved BUG 2011` : ContactCurveCircles 
    - description:  force computation not correct regarding simultaneous contact with several segments
    - date resolved: **2025-05-11 12:01**\ , date raised: 2025-05-11 
 * Version 1.9.108: resolved Issue 1925: ContactCurveCircles (example)
    - description:  add example of chain drive
    - date resolved: **2025-05-11 12:01**\ , date raised: 2024-11-04 
 * Version 1.9.107: resolved Issue 1952: CreateTorsionalSpringDamper (testing)
    - description:  add test example
    - **notes:** included in createFunctionsTest
    - date resolved: **2025-05-10 23:23**\ , date raised: 2025-02-05 
 * Version 1.9.106: resolved Issue 2005: realtime (extension)
    - description:  measure realtime reserve in special timers
    - date resolved: **2025-05-10 23:18**\ , date raised: 2025-05-10 
 * Version 1.9.105: resolved Issue 2001: Create functions (extension)
    - description:  add automatism to create geometry from inertia objects by default (cylinder, sphere, brick, ...); using default colors
    - date resolved: **2025-05-10 23:07**\ , date raised: 2025-05-08 
 * Version 1.9.104: resolved Issue 2006: RigidBodyInertia (extension)
    - description:  add self.data dictionary which stores data of special inertia classes, such as radius of cylinder, etc.; this allows to obtain geometry information
    - date resolved: **2025-05-10 20:39**\ , date raised: 2025-05-10 
 * Version 1.9.103: resolved Issue 1971: Create functions (extension)
    - description:  add general test model
    - date resolved: **2025-05-10 19:56**\ , date raised: 2025-03-05 
 * Version 1.9.102: resolved Issue 1987: CreateCoordinateConstraint (extension)
    - description:  add option to constrain coordinate of object via its nodes; add option to constrain single coordinates or even a list of coordinates
    - date resolved: **2025-05-10 19:37**\ , date raised: 2025-04-08 
 * Version 1.9.101: resolved Issue 0657: Delete item (extension)
    - description:  add functionality to delete items, adding specific features to re-index nodes in objects/markers, etc. if a node, object or marker is deleted; add MaxItemNumber() function to systemData which returns unique name for items even after deletion
    - **notes:** resolved by reordering dependent items; no maxItem used
    - date resolved: **2025-05-10 01:31**\ , date raised: 2021-05-01 
 * Version 1.9.100: resolved Issue 2004: GetJointArgs (extension)
    - description:  add a utility function, which takes an existing marker and rotationMarker and another body to compute a new marker and rotationMarker for the other body; can be directly used as joint arguments like in revolute, prismatic or generic joints
    - date resolved: **2025-05-09 23:59**\ , date raised: 2025-05-09 
 * Version 1.9.99: resolved Issue 2003: GetOtherMarker (extension)
    - description:  add a utility function, which the computes a new marker from the reference position of an existing marker and another body; for easier setup of markers
    - date resolved: **2025-05-09 15:34**\ , date raised: 2025-05-09 
 * Version 1.9.98: resolved Issue 1999: delete (extension)
    - description:  add option to delete dependent markers when deleting loads
    - date resolved: **2025-05-09 08:08**\ , date raised: 2025-05-08 
 * Version 1.9.97: resolved Issue 1998: delete (extension)
    - description:  add option to delete dependent markers in connectors/joints when deleting objects
    - date resolved: **2025-05-09 08:08**\ , date raised: 2025-05-07 
 * Version 1.9.96: resolved Issue 2000: delete (extension)
    - description:  consistently delete loads and sensors
    - date resolved: **2025-05-08 20:57**\ , date raised: 2025-05-08 
 * Version 1.9.95: resolved Issue 2002: BasicTraits.h (change)
    - description:  remove and add new AdvancedStuff.h and put there sort, atomics operations, iterators, etc. which are not needed everywhere
    - date resolved: **2025-05-08 20:39**\ , date raised: 2025-05-08 
 * Version 1.9.94: resolved Issue 1997: delete (extension)
    - description:  consistently delete nodes and markers in MainSystem
    - date resolved: **2025-05-08 13:35**\ , date raised: 2025-05-06 
 * Version 1.9.93: resolved Issue 1985: CreateFunctions (extension)
    - description:  check pyi files and workflows to enable code completion and navigation in Spyder with Create functions
    - **notes:** shall be resolved with issue 1996, as it natively derives from C++ class
    - date resolved: **2025-05-06 22:20**\ , date raised: 2025-04-02 
 * Version 1.9.92: resolved Issue 1986: ObjectMassPoint (extension)
    - description:  add outputvariables rotation matrix and angular velocity (also to NodePoint) in order to allow spherical joint to be attached.
    - date resolved: **2025-05-06 22:12**\ , date raised: 2025-04-08 
 * Version 1.9.91: :textred:`resolved BUG 1995` : Sensor 
    - description:  case SensorType::KinematicTree missing in GetTypeDependentIndex
    - date resolved: **2025-05-06 17:13**\ , date raised: 2025-05-06 
 * Version 1.9.90: resolved Issue 1994: pybind11 (fix)
    - description:  local files using pybind11 2.6, not suitable for Numpy >= 2.0
    - date resolved: **2025-05-06 16:44**\ , date raised: 2025-05-06 
 * Version 1.9.89: resolved Issue 1993: MSVC (fix)
    - description:  compilation/execution of Exudyn in MSVC and python differ considerably
    - date resolved: **2025-05-06 16:00**\ , date raised: 2025-05-06 
 * Version 1.9.88: resolved Issue 1992: CSystem (fix)
    - description:  systemIsConsistent used instead of systemIsInteger in CheckSystemIntegrity
    - date resolved: **2025-05-06 12:07**\ , date raised: 2025-05-06 
 * Version 1.9.87: resolved Issue 1991: MainSystem (fix)
    - description:  Node, Object, Marker, ... functions have wrong internal range check (not including invalid index)
    - date resolved: **2025-05-05 22:43**\ , date raised: 2025-05-05 
 * Version 1.9.86: resolved Issue 1990: delete object (extension)
    - description:  first step to fulfill issue 657; reorder markers and sensors and assign invalid object number if object is used; add assemble check function for invalid indices
    - date resolved: **2025-05-05 22:18**\ , date raised: 2025-05-05 
 * Version 1.9.85: resolved Issue 1984: FEM (extension)
    - description:  Load/Save of FEM and FFRF data: default value NPY switched to NPZ due to Numpy2.x conflicts; for compatibility set mode to NPY
    - **notes:** all load/save functions should work with npy, npz, pkl and hdf5
    - date resolved: **2025-04-01 19:21**\ , date raised: 2025-04-01 
 * Version 1.9.84: resolved Issue 1983: FEM (change)
    - description:  adjust Load/Save functions to by-default support hdf5 or pkl as npy does not work with numpy 2.0
    - **notes:** changed ObjectFFRFreducedOrderInterface and FEM class to by default use numpy NPZ format
    - date resolved: **2025-04-01 16:44**\ , date raised: 2025-04-01 
 * Version 1.9.83: resolved Issue 1982: SolutionViewer (extension)
    - description:  add option Make mp4 to create videos with python ffmpeg lib
    - date resolved: **2025-04-01 08:51**\ , date raised: 2025-04-01 
 * Version 1.9.82: resolved Issue 1981: Images2Video (extension)
    - description:  add function ConvertImages2Video and dialog InteractiveImages2Video to convert images to videos directly in Python
    - date resolved: **2025-04-01 02:20**\ , date raised: 2025-04-01 
 * Version 1.9.81: resolved Issue 1980: PlotSensor (change)
    - description:  change to matplotlib.get_backend().lower() when comparing with matplotlib agg render mode; this shall avoid warnings in case of plots with agg in background
    - date resolved: **2025-03-31 23:03**\ , date raised: 2025-03-31 
 * Version 1.9.80: resolved Issue 1972: artificialIntelligence (fix)
    - description:  PreInitializeSolver always uses Generalized alpha solver, but should go in line with SetSolver
    - **notes:** this issue was erratic and is ignored; PreInitializeSolver shall be replaced by derived class calling SetSolver with the desired solverType; added remarks in artificialIntelligence.py
    - date resolved: **2025-03-31 22:42**\ , date raised: 2025-03-06 
 * Version 1.9.79: resolved Issue 1976: mbs.Assemble() (change)
    - description:  check if assembled in SolveStatic and SolveDynamic and automatically call mbs.Assemble() prior to solver; only raise warning
    - date resolved: **2025-03-31 22:15**\ , date raised: 2025-03-30 
 * Version 1.9.78: resolved Issue 1979: ComputeSystemDegreeOfFreedom (change)
    - description:  change print to exudyn.Print for workflow consistency
    - **notes:** also fixed other print commands in solver.py
    - date resolved: **2025-03-31 21:52**\ , date raised: 2025-03-31 
 * Version 1.9.77: resolved Issue 1978: SetWriteToFile (extension)
    - description:  add flushAlways (default:False) option for immediate writing to file
    - date resolved: **2025-03-31 21:48**\ , date raised: 2025-03-31 
 * Version 1.9.76: resolved Issue 1975: CreateTorsionalSpringDamper (fix)
    - description:  does not work for unlimitedRotations=True due to additional comma
    - date resolved: **2025-03-12 15:08**\ , date raised: 2025-03-12 
 * Version 1.9.75: resolved Issue 1974: CreateSphericalJoint (change)
    - description:  Make it work for bodies that do not offer a rotation matrix (mass points)
    - date resolved: **2025-03-12 14:35**\ , date raised: 2025-03-12 
 * Version 1.9.74: resolved Issue 1973: GeneralContact (change)
    - description:  remove frictionVelocityPenalty as it is not used; also remove macro ANCFuseFrictionPenalty
    - date resolved: **2025-03-07 17:00**\ , date raised: 2025-03-07 
 * Version 1.9.73: resolved Issue 1970: CreateRollingDisc (extension)
    - description:  create test model
    - date resolved: **2025-03-05 22:36**\ , date raised: 2025-03-05 
 * Version 1.9.72: resolved Issue 1969: CreateRollingDisc (extension)
    - description:  add create function to MainSystem
    - date resolved: **2025-03-05 22:17**\ , date raised: 2025-03-05 
 * Version 1.9.71: resolved Issue 1968: GeneralContact (extension)
    - description:  add option to only use dynamic search tree with no duplicated search bins
    - **notes:** in case of no static triangles, static searchtree is only created once and no additional operations are performed
    - date resolved: **2025-03-05 21:11**\ , date raised: 2025-03-03 
 * Version 1.9.70: resolved Issue 1967: Create functions (change)
    - description:  change return values of all mbs.Create functions with joints (CreateRevoluteJoint, CreateSphericalJoint, CreatePrismaticJoint, CreateGenericJoint, CreateDistanceConstraints, ...) to the object index instead of lists
    - date resolved: **2025-03-03 22:04**\ , date raised: 2025-03-03 
 * Version 1.9.69: resolved Issue 1949: RollingDiscPenalty (extension)
    - description:  add create function to MainSystem
    - **notes:** already done with issue 1957
    - date resolved: **2025-03-03 20:41**\ , date raised: 2025-02-04 
 * Version 1.9.68: resolved Issue 1966: GeneralContact (extension)
    - description:  SearchTree: add option to perform more accurate test for triangles: add simple check to see if bin is fully on one side of the triangle
    - date resolved: **2025-03-02 18:50**\ , date raised: 2025-03-02 
 * Version 1.9.67: resolved Issue 1895: GeneralContact (extension)
    - description:  add option to add static objects to GeneralContact and to SearchTree
    - date resolved: **2025-03-02 15:42**\ , date raised: 2024-10-16 
 * Version 1.9.66: resolved Issue 1965: GeneralContact (change)
    - description:  ODE2RHS timer removed and replaced with CSystem Contact:Overall timer (which includes PostNewtonStep)
    - date resolved: **2025-03-02 11:46**\ , date raised: 2025-03-02 
 * Version 1.9.65: resolved Issue 1962: CuttingPlane (extension)
    - description:  add simple option for cutting plane (point, normal vector, flag) to exclude triangles and other objects from graphics
    - **notes:** added options openGL.clippingPlaneNormal and openGL.clippingPlaneDistance to enable simple clipping
    - date resolved: **2025-02-28 21:30**\ , date raised: 2025-02-27 
    - resolved by: EXTENSION
 * Version 1.9.64: resolved Issue 1964: Cable2D (extension)
    - description:  add setting useReducedOrderIntegration=2 to interface docu
    - date resolved: **2025-02-28 20:57**\ , date raised: 2025-02-28 
 * Version 1.9.63: resolved Issue 1963: CreateRollingDiscPenalty (testing)
    - description:  create test model
    - date resolved: **2025-02-28 00:22**\ , date raised: 2025-02-27 
 * Version 1.9.62: resolved Issue 1957: RollingDiscPenalty (testing)
    - description:  add test example
    - date resolved: **2025-02-27 16:50**\ , date raised: 2025-02-09 
 * Version 1.9.61: resolved Issue 1951: RigidBody2D (testing)
    - description:  add test for physicsCenterOfMass != 0
    - date resolved: **2025-02-05 15:08**\ , date raised: 2025-02-05 
 * Version 1.9.60: resolved Issue 1955: FEM (fix)
    - description:  WarnNumpy2 contains error, as it tries to compare int with str
    - date resolved: **2025-02-05 15:04**\ , date raised: 2025-02-05 
 * Version 1.9.59: resolved Issue 1950: RigidBody2D (extension)
    - description:  add parameter physicsCenterOfMass, similar to RigidBody
    - date resolved: **2025-02-05 14:53**\ , date raised: 2025-02-05 
 * Version 1.9.58: resolved Issue 1581: mainSystemExtensions (extension)
    - description:  add LinearSpringDamper and TorsionalSpringDamper
    - **notes:** transferred to issues 1948 and 1953
    - date resolved: **2025-02-05 08:44**\ , date raised: 2023-05-21 
 * Version 1.9.57: resolved Issue 1948: CreateTorsionalSpringDamper (extension)
    - description:  add create function to MainSystem
    - date resolved: **2025-02-04 13:44**\ , date raised: 2025-02-04 
 * Version 1.9.56: :textred:`resolved BUG 1946` : SphereSphereContact 
    - description:  error in PostNewtonStep: always uses data variables
    - date resolved: **2025-02-03 15:40**\ , date raised: 2025-02-03 
 * Version 1.9.55: resolved Issue 1945: lieGroupSimplifiedKinematicRelations (change)
    - description:  lieGroupSimplifiedKinematicRelations used to test more accurate Lie group solver; for now, default value set to false in order to have test suite running
    - date resolved: **2025-01-28 11:53**\ , date raised: 2025-01-28 
 * Version 1.9.54: resolved Issue 1944: solver (fix)
    - description:  always writes 'Solver terminated unsuccessfully' independently of success.
    - date resolved: **2025-01-05 18:00**\ , date raised: 2025-01-05 
 * Version 1.9.53: resolved Issue 1479: RotationVector2RotationMatrix (fix)
    - description:  both in Python and C++, fix range to 0..2\*pi, as large angles cause low accuracy
    - date resolved: **2024-12-01 19:13**\ , date raised: 2023-03-27 
 * Version 1.9.52: resolved Issue 1651: Python 3.11 (extension)
    - description:  added Python 3.11 workflows for Windows, Linux and MacOS builds (note: problems with Rosetta x86 on MacOS)
    - **notes:** resolved earlier also for 3.12 and 3.13
    - date resolved: **2024-12-01 12:31**\ , date raised: 2023-07-20 
 * Version 1.9.51: resolved Issue 1483: PUMA560 (fix)
    - description:  COM frames of KinematicTree drawn wrong: serialRobotInverseKinematics; check COM setting
    - **notes:** resolved already with issue 1857
    - date resolved: **2024-12-01 12:29**\ , date raised: 2023-03-29 
 * Version 1.9.50: :textred:`resolved BUG 1943` : FEMinterface 
    - description:  CreateNonlinearFEMObjectGenericODE2NGsolve does not work due to change in internal FEM stiffness and mass matrices, stored as scipy sparse matrices now
    - date resolved: **2024-11-27 21:24**\ , date raised: 2024-11-27 
 * Version 1.9.49: resolved Issue 1942: System equations of motion (docu)
    - description:  add missing term partial g/partial qDot in ODE2 part, in particular for non-holonomic constriants; this part was already implemented in a generic way since the very beginning, but missed to find the way into the documentation.
    - date resolved: **2024-11-18 19:15**\ , date raised: 2024-11-18 
 * Version 1.9.48: resolved Issue 1940: NumPy2 (extension)
    - description:  Add warning in FEM for some functions that do not work properly with NumPy 2.x
    - date resolved: **2024-11-10 19:57**\ , date raised: 2024-11-10 
 * Version 1.9.47: resolved Issue 1939: HDF5 load/save (extension)
    - description:  extend to None type (but excluding numpy arrays containing None!
    - date resolved: **2024-11-10 18:43**\ , date raised: 2024-11-10 
 * Version 1.9.46: resolved Issue 1935: GraphicsData functions (change)
    - description:  adjust graphics.Sphere, graphics.Brick, etc. to create numpy arrays for improved efficiency
    - date resolved: **2024-11-10 17:24**\ , date raised: 2024-11-10 
 * Version 1.9.45: resolved Issue 1936: GraphicsData functions (change)
    - description:  adjust graphics conversion functions (graphics.Move, etc.) to handle numpy arrays as well
    - date resolved: **2024-11-10 16:37**\ , date raised: 2024-11-10 
 * Version 1.9.44: resolved Issue 1938: graphics.Sphere (fix)
    - description:  only works if addEdges <=1; add condition for number of edges
    - date resolved: **2024-11-10 16:09**\ , date raised: 2024-11-10 
 * Version 1.9.43: resolved Issue 1937: graphics.color (fix)
    - description:  type completion not working
    - date resolved: **2024-11-10 15:32**\ , date raised: 2024-11-10 
 * Version 1.9.42: resolved Issue 1934: GraphicsData write (change)
    - description:  GetObject now returns GraphicsData as numpy arrays to be consistent with issue 1934
    - date resolved: **2024-11-10 14:26**\ , date raised: 2024-11-10 
 * Version 1.9.41: resolved Issue 1933: GraphicsData read (change)
    - description:  allow that all colors, positions, triangles, points, normals, edges, ... are either lists or numpy.arrays (but flatten to 1D)
    - date resolved: **2024-11-10 14:26**\ , date raised: 2024-11-10 
 * Version 1.9.40: resolved Issue 1929: GraphicsData (extension)
    - description:  extend import/export function of GraphicsData for numpy arrays; speeds up load/safe significantly
    - date resolved: **2024-11-10 14:26**\ , date raised: 2024-11-08 
 * Version 1.9.39: resolved Issue 1908: StaticSolver (fix)
    - description:  does not write appropriate error message, but only writes: ValueError: SolveStatic terminated due to errors
    - **notes:** together with 1931
    - date resolved: **2024-11-09 17:33**\ , date raised: 2024-10-24 
 * Version 1.9.38: resolved Issue 1930: Solver (change)
    - description:  change message when solver finishes, distinguishing between solver success and failure: 'Solver terminated unsuccessfully' or 'Solver terminated unsuccessfully'
    - date resolved: **2024-11-09 17:32**\ , date raised: 2024-11-09 
 * Version 1.9.37: resolved Issue 1928: URDF (chekc)
    - description:  check import from roboticstoolbox-python and pymeshlab
    - date resolved: **2024-11-08 22:31**\ , date raised: 2024-11-08 
 * Version 1.9.36: resolved Issue 1927: SaveDictToHDF5 (extension)
    - description:  extend for saving int32, int64, float32 and float64
    - date resolved: **2024-11-08 22:30**\ , date raised: 2024-11-08 
 * Version 1.9.35: resolved Issue 1923: ContactCurveCircles (extension)
    - description:  add special contact element between curve defined by segments in contact with circles; 2D curves and circle co-move with rigid body marker at which curve is attached; enables Cam-follower mechanism, chain-sprocket contact, etc.
    - date resolved: **2024-11-04 23:36**\ , date raised: 2024-11-04 
 * Version 1.9.34: resolved Issue 1921: ContactSphereSphere (extension)
    - description:  add option to use restitution coefficient
    - date resolved: **2024-11-04 23:32**\ , date raised: 2024-11-02 
 * Version 1.9.33: resolved Issue 1919: ContactSphereSphere (extension)
    - description:  extend for nonlinear contact models; in particular restitution coefficient and adhesive elasto-plastic contact model
    - date resolved: **2024-11-04 23:32**\ , date raised: 2024-11-02 
    - resolved by: S. Weyrer
 * Version 1.9.32: resolved Issue 1918: ContactSphereSphere (extension)
    - description:  add basic contact object for sphere-sphere contact, with options for linear and nonlinear contact models as well as adhesion
    - date resolved: **2024-11-02 11:49**\ , date raised: 2024-11-02 
 * Version 1.9.31: resolved Issue 1917: useRecommendedStepSize (extension)
    - description:  add option in timeIntegration.discontinuous to turn on/off step size recommendations for contact and other discontinuous phenomena
    - date resolved: **2024-11-02 11:46**\ , date raised: 2024-11-02 
 * Version 1.9.30: resolved Issue 1916: KinematicTree (docu)
    - description:  add clarification for order to application of joint offset and joint transformation (rotation)
    - date resolved: **2024-10-30 10:56**\ , date raised: 2024-10-30 
 * Version 1.9.29: resolved Issue 1915: RigidBodyInertia (extension)
    - description:  add warning in case of unphysical inertia parameters
    - date resolved: **2024-10-29 22:26**\ , date raised: 2024-10-29 
 * Version 1.9.28: resolved Issue 1914: GeneralContact (change)
    - description:  GetSystemODE2RhsContactForces: change argument reference to copy to make it more consistent with other reference access and avoid confusion with reference configuration; default behavior unchanged
    - date resolved: **2024-10-27 10:13**\ , date raised: 2024-10-27 
 * Version 1.9.27: resolved Issue 1913: MainSystem.systemData (extension)
    - description:  add function GetODE2CoordinatesTotal which includes reference values added to coordinates
    - date resolved: **2024-10-27 10:00**\ , date raised: 2024-10-27 
 * Version 1.9.26: resolved Issue 1911: OutputVariableType (change)
    - description:  change order of types, leading to different numerical values behind OutputVariableType enum (should not affect normal codes)
    - **notes:** affected by issue 1909
    - date resolved: **2024-10-26 19:30**\ , date raised: 2024-10-26 
 * Version 1.9.25: resolved Issue 1909: OutputVariableType (extension)
    - description:  Add CoordinatesTotal which includes the reference configuration for output variables, used in sensors, or object and node outputs
    - date resolved: **2024-10-26 19:26**\ , date raised: 2024-10-26 
 * Version 1.9.24: resolved Issue 1505: reference coordinates (extension)
    - description:  add option to get total coordinates, being reference + current coordinates; gives 4 new configurations; use harmonized interface functions
    - date resolved: **2024-10-26 19:26**\ , date raised: 2023-04-08 
 * Version 1.9.23: resolved Issue 1897: particles (extension)
    - description:  Add particles module with functionality for creating densly packed particles in a box with proper regular initialization
    - date resolved: **2024-10-19 20:57**\ , date raised: 2024-10-16 
 * Version 1.9.22: resolved Issue 1903: GeneralContact (extension)
    - description:  GetSystemODE2RhsContactForces(...): add additional arg reference in order to allow linking to contact forces and thus allowing faster access (writing to this vector has no effect!)
    - date resolved: **2024-10-19 18:59**\ , date raised: 2024-10-19 
 * Version 1.9.21: resolved Issue 1902: FEMinterface (change)
    - description:  store elements as consistently as np.array instead of list of lists; check that all large arrays are np.arrays; do this also for surface list of lists
    - **notes:** now all previous list of lists in FEMinterface are defined to be numpy arrays for speed up of load/save; check your interfaces! GetSurfaceTriangles still returns list of lists
    - date resolved: **2024-10-19 18:48**\ , date raised: 2024-10-19 
 * Version 1.9.20: resolved Issue 1629: GetSystemState (extension)
    - description:  extend behavior for returning a dictionary with all data incl. accelerations and possibly alg. accelerations for generalized-alpha solver; check also option to link coords, as in issue 1504
    - **notes:** added mbs.systemData.GetSystemStateDict(...) with option to return a reference (link) to system vectors, which do not require copying and allow modifications directly
    - date resolved: **2024-10-19 18:45**\ , date raised: 2023-06-23 
 * Version 1.9.19: resolved Issue 1504: Reference/link (extension)
    - description:  extend mbs and systemData functions for reference, e.g., GetODE2Coordinates; use different function with "Link" extension, e.g., GetODE2CoordinatesLink
    - **notes:** added systemData function GetSystemStateDict which allows to obtain writeable references
    - date resolved: **2024-10-19 01:28**\ , date raised: 2023-04-08 
 * Version 1.9.18: :textred:`resolved BUG 1901` : FEM LoadFromFile 
    - description:  does not work correctly for forceVersion>0.
    - date resolved: **2024-10-18 14:58**\ , date raised: 2024-10-18 
 * Version 1.9.17: resolved Issue 1900: MainSystem UserFunctions (testing)
    - description:  Add TestModel for all kinds of PreStep, PostStep, reNewtonResidual, etc. functions
    - date resolved: **2024-10-17 23:09**\ , date raised: 2024-10-16 
 * Version 1.9.16: resolved Issue 1899: SystemJacobianUserFunction (extension)
    - description:  add a MainSystem user function with SetSystemJacobianUserFunction(...) which adds terms to the system jacobian; this is valuable as it may modify only few terms and add appropriate (experimental) terms in extension with the PreNewtonResidualUserFunction which otherwise would lead to bad convergence if important couplings are not added; as mentioned in the description, the solver's user functions are more general and should be also considered
    - date resolved: **2024-10-17 23:09**\ , date raised: 2024-10-16 
 * Version 1.9.15: resolved Issue 1882: preNewtonResidualUserFunction (extension)
    - description:  add user function called in every iteration of Newton solver in static or implicit dynamic computations
    - **notes:** function SetPreNewtonResidualUserFunction added to MainSystem, see description at MainSystem
    - date resolved: **2024-10-16 23:27**\ , date raised: 2024-10-10 
 * Version 1.9.14: resolved Issue 0838: GeneralContact jacobian (extension)
    - description:  add jacobian and PostNewton for cable-sphere (circle) contact
    - **notes:** already done earlier for ANCF beam elements
    - date resolved: **2024-10-16 16:52**\ , date raised: 2021-12-19 
 * Version 1.9.13: :textred:`resolved BUG 1893` : keyPressUserFunction 
    - description:  not working any more!
    - **notes:** works, but needs to activate window.ignoreKeys in order to call user function
    - date resolved: **2024-10-15 23:21**\ , date raised: 2024-10-15 
 * Version 1.9.12: resolved Issue 1717: Symbolic (example)
    - description:  add examples to testsuite; modify existing examples
    - **notes:** exu.symbolic used in 5 tests and examples
    - date resolved: **2024-10-15 23:08**\ , date raised: 2023-12-08 
 * Version 1.9.11: resolved Issue 1718: Mainsystem extensions (example)
    - description:  modify some examples for .Create...(...) functions
    - **notes:** new functions usually only using CreateGenericJoint, CreateRevoluteJoint, etc.; still keeping old workflows with markers for LLM trainings
    - date resolved: **2024-10-15 23:06**\ , date raised: 2023-12-08 
 * Version 1.9.10: resolved Issue 1721: Mainsystem extensions (change)
    - description:  in extension to issue 1718, change all AddRigidBody(...) functions to CreateRigidBody functionality
    - date resolved: **2024-10-15 23:05**\ , date raised: 2023-12-08 
 * Version 1.9.9: resolved Issue 1885: pickleCopyMBS (testing)
    - description:  add TestModel for copying and pickling (load/save) of MainSystem mbs
    - date resolved: **2024-10-13 22:52**\ , date raised: 2024-10-11 
 * Version 1.9.8: resolved Issue 1891: load/save mbs (extension)
    - description:  improve functionalities to load and save mbs, using GetDictionary / SetDictionary; see example pickleCopyMbs.py using pickle and HDF5 files
    - date resolved: **2024-10-13 19:16**\ , date raised: 2024-10-13 
 * Version 1.9.7: resolved Issue 1890: LoadSaveHDF5 (extension)
    - description:  enable option to load/save Python user functions
    - date resolved: **2024-10-11 22:49**\ , date raised: 2024-10-11 
 * Version 1.9.6: resolved Issue 1886: NGsolve CMS test (testing)
    - description:  add test model for whole CMS functionality but loading from stored data; test all 3 new file formats for storing FEM data
    - date resolved: **2024-10-11 09:00**\ , date raised: 2024-10-11 
 * Version 1.9.5: resolved Issue 1887: HDF5 (extension)
    - description:  add functions to advancedUtilities to load/save hierarchical dictionary data
    - date resolved: **2024-10-11 01:00**\ , date raised: 2024-10-11 
 * Version 1.9.4: resolved Issue 1884: FEMinterface (extension)
    - description:  extend LoadFromFile and SaveToFile for HDF5 and PKL (pickle) file formats
    - date resolved: **2024-10-11 01:00**\ , date raised: 2024-10-11 
 * Version 1.9.3: resolved Issue 1883: RigidBodySpringDamper (change)
    - description:  C++: change computation of outputvariables such that springForceTorqueUserFunction is not called for computation of displacement, rotation, etc., but only for forces and torques
    - date resolved: **2024-10-11 00:03**\ , date raised: 2024-10-11 
 * Version 1.9.2: resolved Issue 1881: Loads visualization (check)
    - description:  consider an option to show time-dependent loads, in particular in case of symbolic user functions
    - **notes:** added flag  visualizationSettings.loads.drawWithUserFunction and test model loadUserFunctionTest.py
    - date resolved: **2024-10-10 10:48**\ , date raised: 2024-10-09 
 * Version 1.9.1: resolved Issue 1723: Symbolic (extension)
    - description:  SymbolicRealVector: add EvaluateItem(i) in operators wherever possible to efficiently evaluate single components instead of all vector components
    - date resolved: **2024-10-09 22:23**\ , date raised: 2023-12-09 
 * Version 1.9.0: resolved Issue 1880: MatrixContainer (fix)
    - description:  when initialized with lists of lists, it prints the lists
    - date resolved: **2024-10-09 11:06**\ , date raised: 2024-10-09 

***********
Version 1.8
***********

 * Version 1.8.81: resolved Issue 1879: MatrixContainer (extension)
    - description:  add feature to directly initialize with scipy sparse csr matrix; add new function Initialize to initialize MatrixContainer, and AddSparseMatrix for adding sparse matrices with factor
    - date resolved: **2024-10-09 09:39**\ , date raised: 2024-10-09 
 * Version 1.8.80: resolved Issue 1878: robotics.mobile (change)
    - description:  correct camelcase writing of mobileRobot2MBS to MobileRobot2MBS, getWheelVelocities to GetWheelVelocities, and getCartesianVelocities to GetCartesianVelocities; adjust examples
    - date resolved: **2024-10-09 08:04**\ , date raised: 2024-10-09 
 * Version 1.8.79: resolved Issue 1844: AddRigidBody (change)
    - description:  add warning to this and related functions for deprecation
    - date resolved: **2024-10-09 07:40**\ , date raised: 2024-05-28 
 * Version 1.8.78: resolved Issue 1455: MarkerSuperElementRigidBody (fix)
    - description:  fix derivative of exponential map for velocity level
    - **notes:** already resolved earlier
    - date resolved: **2024-10-09 07:37**\ , date raised: 2023-03-05 
 * Version 1.8.77: resolved Issue 1341: MacOS multithreading (fix)
    - description:  resolve compilation problems with NGsolve taskmanager on Apple MacOS
    - **notes:** already resolved earlier
    - date resolved: **2024-10-09 07:36**\ , date raised: 2022-12-26 
 * Version 1.8.76: resolved Issue 0496: controller (extension)
    - description:  add possibility to differentiate loads w.r.t. sensor?/object/node (use additional sensor numbers which provide dependencies); add optional dependence on sensors; integrateors using ODE1 or discrete implementation
    - **notes:** already resolved in version 1.6.84; available via systemData.AddODE2LoadDependencies
    - date resolved: **2024-10-09 07:33**\ , date raised: 2020-12-09 
 * Version 1.8.75: resolved Issue 1877: Create functions (change)
    - description:  CreateDistanceConstraint, CreateSpringDamper and similar create functions have bodyList instead of bodyNumbers, which is used for joints; use bodyNumbers and allow bodyList as deprecated option
    - **notes:** kept compatibility with existing bodyList args, but will be removed in future versions
    - date resolved: **2024-10-08 23:04**\ , date raised: 2024-10-08 
 * Version 1.8.74: resolved Issue 1862: CSR functionality in FEM (extension)
    - description:  change internal CSR format to scipy CSR format; add simple check for Scipy to be installed at start of FEM, set scipyInstalled=True, use CheckSciPyInstalled() function for unified errors
    - date resolved: **2024-10-08 17:29**\ , date raised: 2024-10-02 
 * Version 1.8.73: resolved Issue 1870: FEMinterface (check)
    - description:  check all occurances of GetStiffnessMatrix and GetMassMatrix for sparse mode now using scipy sparse matrix
    - **notes:** Make sure that using FEMinterface fem, fem.GetMassMatrix of fem.GetStiffnessMatrix now returns a SciPy sparse matrix, which can be converted into the previous form by using ScipySparseCSRtoCSR(fem.GetMassMatrix())
    - date resolved: **2024-10-08 17:28**\ , date raised: 2024-10-07 
 * Version 1.8.72: resolved Issue 1876: FEMinterface (change)
    - description:  SaveToFile: used wrong default fileVersion=13, which should have been fileVersion=1; as we now shift to fileVersion=2, old files may be loaded with forceVersion=1
    - date resolved: **2024-10-08 17:26**\ , date raised: 2024-10-08 
 * Version 1.8.71: resolved Issue 1871: MatrixContainer (fix)
    - description:  fix compilation issues with pybind11 and scipy coo matrix
    - date resolved: **2024-10-08 17:26**\ , date raised: 2024-10-07 
 * Version 1.8.70: resolved Issue 1875: FEMinterface (change)
    - description:  the mass and stiffness matrices in FEMinterface are either None or given in SciPy-sparse csr format; this gives also a a new load/save fileVersion of FEMinterface
    - date resolved: **2024-10-08 13:09**\ , date raised: 2024-10-08 
 * Version 1.8.69: resolved Issue 1874: FEMinterface (change)
    - description:  SaveToFile: add version 2 which also pickles mass and stiffnessMatrix
    - date resolved: **2024-10-08 11:11**\ , date raised: 2024-10-08 
 * Version 1.8.68: resolved Issue 1873: FEM module (change)
    - description:  replace print(...) with exu.Print(...) commands to better control output flow
    - date resolved: **2024-10-08 10:33**\ , date raised: 2024-10-08 
 * Version 1.8.67: resolved Issue 1872: MatrixContainer (extension)
    - description:  SetWithSparseMatrixCSR: mark as deprecated; instead add new function SetWithSparseMatrix with additional arg factor (default=1) to multply matrix values with factor before adding
    - date resolved: **2024-10-07 18:34**\ , date raised: 2024-10-07 
 * Version 1.8.66: resolved Issue 1869: FEMinterface (change)
    - description:  change default values of massMatrix and stiffnessMatrix to None instead of np.zeros((0,0))
    - date resolved: **2024-10-07 01:54**\ , date raised: 2024-10-07 
 * Version 1.8.65: resolved Issue 1865: MatrixContainer (extension)
    - description:  add functionality to add Scipy csr matrix AddSparseMatrix(..., factor=1) with factor
    - date resolved: **2024-10-07 01:35**\ , date raised: 2024-10-02 
 * Version 1.8.64: resolved Issue 0840: explicit solvers velocity verlet (extension)
    - description:  add velocity verlet integration scheme in particular for particle and contact simulation
    - **notes:** added, but not yet tested for Lie group case!
    - date resolved: **2024-10-07 00:37**\ , date raised: 2021-12-19 
 * Version 1.8.63: resolved Issue 1868: SetWithSparseMatrixCSR (change)
    - description:  change default behavior to useDenseMatrix=False, using sparse mode by default
    - date resolved: **2024-10-06 22:03**\ , date raised: 2024-10-06 
 * Version 1.8.62: resolved Issue 1867: graphicsDataUtilities (docu)
    - description:  adapt documentation to replace graphicsDataUtilities with graphics
    - date resolved: **2024-10-06 17:24**\ , date raised: 2024-10-06 
 * Version 1.8.61: resolved Issue 1866: DOCU GraphicsData: Line (fix)
    - description:  Mistake in Line example; add hint to use graphics.Lines function
    - date resolved: **2024-10-06 17:06**\ , date raised: 2024-10-06 
 * Version 1.8.60: resolved Issue 1854: figure (docu)
    - description:  replace figure theoryRotationsTaitBryanAngles, which has been accidentially copied
    - date resolved: **2024-10-06 16:51**\ , date raised: 2024-06-05 
 * Version 1.8.59: :textred:`resolved BUG 1860` : ComputeODE2Eigenvalues 
    - description:  in case of computeComplexEigenvalues=True, the option convert2Frequencies=True computes wrong frequencies; this combination thus will be deactivated, as it makes little sense anyhow
    - date resolved: **2024-09-19 16:15**\ , date raised: 2024-09-19 
 * Version 1.8.58: resolved Issue 1859: Frames (fix)
    - description:  frame numbers not shown (e.g. for nodes)
    - **notes:** fixed together with issue 1858
    - date resolved: **2024-08-07 17:26**\ , date raised: 2024-08-07 
 * Version 1.8.57: resolved Issue 1858: KinematicTree (fix)
    - description:  frame numbers not shown
    - **notes:** changing default showFramesNumbers=False
    - date resolved: **2024-08-07 17:26**\ , date raised: 2024-08-07 
 * Version 1.8.56: :textred:`resolved BUG 1857` : KinematicTree 
    - description:  COM not correctly drawn (drawn at joint location)
    - date resolved: **2024-08-07 17:05**\ , date raised: 2024-08-07 
 * Version 1.8.55: resolved Issue 1856: FromSTLfile (fix)
    - description:  function does not work for A or p being different from []
    - date resolved: **2024-07-03 11:33**\ , date raised: 2024-07-03 
 * Version 1.8.54: resolved Issue 1849: tutorial (docu)
    - description:  add FFRFreducedOrder with NGsolve tutorial
    - date resolved: **2024-06-23 16:59**\ , date raised: 2024-06-02 
 * Version 1.8.53: resolved Issue 1855: lie group integrator (change)
    - description:  add improved lie group integrator for generalized alpha, not using the simplified update and thus having higher accuracy
    - date resolved: **2024-06-15 13:42**\ , date raised: 2024-06-15 
 * Version 1.8.52: resolved Issue 1852: tutorials (docu)
    - description:  adapt tutorials to new exudyn.graphics structure
    - date resolved: **2024-06-04 21:48**\ , date raised: 2024-06-04 
 * Version 1.8.51: resolved Issue 1853: exudyn.graphics (docu)
    - description:  add documentation in python utilities for these functions
    - date resolved: **2024-06-04 21:06**\ , date raised: 2024-06-04 
 * Version 1.8.50: resolved Issue 1851: artificialIntelligence (change)
    - description:  adapt examples to new stable-baselines version
    - **notes:** tested with stable-baselines3 V1.7.0 and V2.3.2
    - date resolved: **2024-06-04 17:24**\ , date raised: 2024-06-04 
 * Version 1.8.49: resolved Issue 1850: artificialIntelligence (change)
    - description:  adapt to stable-baselines3 2.3; add old mode with gym and new mode with gymnasium
    - **notes:** kept old mode available if gym is installed with stable-baselines without gymnasium
    - date resolved: **2024-06-04 14:59**\ , date raised: 2024-06-04 
    - resolved by: P. Manzl
 * Version 1.8.48: resolved Issue 0406: add NGsolve test (extension)
    - description:  with FEMinterface, only for Python37 version
    - **notes:** outdated and removed (see new issue 1849)
    - date resolved: **2024-06-02 13:30**\ , date raised: 2020-05-22 
 * Version 1.8.47: resolved Issue 0837: GeneralContact jacobian (extension)
    - description:  add jacobian and PostNewton for Triangle-Sphere contact
    - date resolved: **2024-06-02 10:58**\ , date raised: 2021-12-19 
 * Version 1.8.46: resolved Issue 1847: GeneralContact (fix)
    - description:  change several for-break commands to continue to improve contact behavior
    - date resolved: **2024-06-01 20:13**\ , date raised: 2024-06-01 
 * Version 1.8.45: resolved Issue 1475: tutorial videos (docu)
    - description:  add new tutorial, replace old ones
    - **notes:** already completed on Feb 15 2024
    - date resolved: **2024-05-20 19:13**\ , date raised: 2023-03-27 
 * Version 1.8.44: resolved Issue 1474: tutorial videos (fix)
    - description:  fix gettings started video
    - **notes:** already completed on Feb 15 2024
    - date resolved: **2024-05-20 19:13**\ , date raised: 2023-03-27 
 * Version 1.8.43: resolved Issue 1842: add FurtherExampels folder for further examples, especially useful for training of LLMs (extension)
    - description:  EXTENSION
    - date resolved: **2024-05-20 19:08**\ , date raised: 2024-05-17 
 * Version 1.8.42: resolved Issue 1843: simulationSettings (docu)
    - description:  correct path of simulationSettings.staticSolverSettings to simulationSettings.staticSolver in RTD description
    - date resolved: **2024-05-17 18:36**\ , date raised: 2024-05-17 
 * Version 1.8.41: resolved Issue 1841: AddODE2LoadDependencies (fix)
    - description:  description is wrong
    - date resolved: **2024-05-17 18:29**\ , date raised: 2024-05-17 
 * Version 1.8.40: resolved Issue 1840: std::isalpha (fix)
    - description:  remove std::isalpha and std::isalnum, as it does not compile on certain older compilers
    - date resolved: **2024-05-13 17:08**\ , date raised: 2024-05-13 
 * Version 1.8.39: resolved Issue 1838: Examples check (extension)
    - description:  add test to check whether examples basically run; use timeout and special setting to turn off all key-press waiting; exclude large examples
    - date resolved: **2024-05-12 10:19**\ , date raised: 2024-05-11 
 * Version 1.8.38: :textred:`resolved BUG 1839` : ALEANCFCable2D 
    - description:  raises error 'ANCFCable2d:ComputeAxialStrain_t not implemented' if ForceLocal is evaluated
    - **notes:** results have to be verified
    - date resolved: **2024-05-11 19:18**\ , date raised: 2024-05-11 
 * Version 1.8.37: resolved Issue 1187: ALEANCFCable2D (extension)
    - description:  add missing terms related to damping terms coupled with delta qALE
    - **notes:** already implemented and checked with paper up to come
    - date resolved: **2024-05-11 19:00**\ , date raised: 2022-07-06 
 * Version 1.8.36: resolved Issue 1692: exudyn.graphics (fix)
    - description:  change GraphicsData functions in examples to exudyn.graphics
    - **notes:** CHECK your files, if there are any issues due to this change; in general, the previous functionality should be maintained
    - date resolved: **2024-05-11 01:18**\ , date raised: 2023-11-19 
 * Version 1.8.35: resolved Issue 1825: GraphicsDataCube (change)
    - description:  rename to GraphicsDataBrick... functions; use assignment to keep previous functionality
    - **notes:** not changed, but introduced exudyn.graphics submodule for graphics functions. OrthoCube now available as exudyn.graphics.Brick
    - date resolved: **2024-05-11 01:16**\ , date raised: 2024-04-25 
 * Version 1.8.34: resolved Issue 1691: exudyn.graphics (extension)
    - description:  map graphicsDataUtilities functions to exudyn.graphics for better readability
    - **notes:** NOTE that previous GraphicsData functions still work; however, it may be necessary to import exudyn.utilities if you imported exudyn.graphicsDataUtilities directly; examples and test models are adjusted to new functions; it is recommended that users switch to new exudyn.graphics functionality!
    - date resolved: **2024-05-10 19:45**\ , date raised: 2023-11-19 
 * Version 1.8.33: resolved Issue 1837: graphics (check)
    - description:  test graphics submodule for future replacement of graphicsDataUtilities
    - date resolved: **2024-05-10 10:09**\ , date raised: 2024-05-10 
 * Version 1.8.32: resolved Issue 1832: theory docu (docu)
    - description:  add description of computation of stresses for FFRFreducedOrder
    - date resolved: **2024-05-09 17:12**\ , date raised: 2024-05-03 
 * Version 1.8.31: resolved Issue 1836: ComputeLinearizedSystem (extension)
    - description:  output constraint and nullspace matrices for constrained systems
    - date resolved: **2024-05-05 16:18**\ , date raised: 2024-05-05 
 * Version 1.8.30: resolved Issue 1835: ComputeLinearizedSystem (extension)
    - description:  add option to compute linearized system of constrained system
    - date resolved: **2024-05-05 16:18**\ , date raised: 2024-05-05 
 * Version 1.8.29: resolved Issue 1834: ComputeLinearizedSystem (change)
    - description:  remove sparse solver option useSparseSolver, as it is not implemented
    - date resolved: **2024-05-05 16:17**\ , date raised: 2024-05-05 
 * Version 1.8.28: resolved Issue 1833: ComputeODE2EigenValues (extension)
    - description:  extend to complex eigenvalues
    - **notes:** added TestModel complexEigenvaluesTest.py
    - date resolved: **2024-05-04 18:12**\ , date raised: 2024-05-03 
 * Version 1.8.27: resolved Issue 1831: ObjectJointRollingDisc (change)
    - description:  change computation of constraints into local joint frame, enabling lateral and forward constraint independently; see old comment on constrainedAxes
    - date resolved: **2024-04-29 08:03**\ , date raised: 2024-04-29 
 * Version 1.8.26: resolved Issue 1830: AddLidar (fix)
    - description:  angles of sensors should be equally arranged between angleStart and angleEnd using numberOfSensors and having a sensor both at start and end angle
    - date resolved: **2024-04-27 17:06**\ , date raised: 2024-04-27 
 * Version 1.8.25: resolved Issue 1829: AddLidar (fix)
    - description:  several arguments are not passed to CreateDistanceSensor; add all arguments to interface, which may cause some change in behavior!
    - date resolved: **2024-04-27 16:48**\ , date raised: 2024-04-27 
 * Version 1.8.24: resolved Issue 1828: Examples (example)
    - description:  add example mobileMecanumWheelRobotWithLidar.py for a mecanum wheeled robot with lidar and mapping
    - date resolved: **2024-04-27 11:11**\ , date raised: 2024-04-27 
    - resolved by: P. Manzl
 * Version 1.8.23: resolved Issue 1827: AddLidar (fix)
    - description:  argument rotation is not used
    - date resolved: **2024-04-27 11:08**\ , date raised: 2024-04-27 
 * Version 1.8.22: resolved Issue 1826: AddLidar (change)
    - description:  angles of sensor directions are not defined and erronously with respect to Y-axis; angleStart and angleEnde shall be measured w.r.t. the X-axis (angle=0) and in positive rotation sense about local Z-axis
    - date resolved: **2024-04-27 11:08**\ , date raised: 2024-04-27 
 * Version 1.8.21: resolved Issue 1824: ObjectConnectorCoordinateVector (docu)
    - description:  fix docu and remove inexisting parameters
    - date resolved: **2024-04-21 19:34**\ , date raised: 2024-04-21 
 * Version 1.8.20: resolved Issue 1820: item type info (change)
    - description:  change py::dict to py:object in functions such as AddMarker according to github issue #64, otherwise giving typing errors in PyLance and similar
    - date resolved: **2024-04-21 19:17**\ , date raised: 2024-04-19 
 * Version 1.8.19: resolved Issue 1823: stub files (change)
    - description:  change type in AddObject, AddMarker, AddSensor, AddLoad, AddNode from pyObject: dict to pyObject: Any in order to resolve typing errors
    - date resolved: **2024-04-21 19:16**\ , date raised: 2024-04-19 
 * Version 1.8.18: resolved Issue 1819: KinematicTreePendulum.py (example)
    - description:  Version 1.7.71 broke example kinematicTreePendulum with transition from SensorObject to SensorBody
    - **notes:** changed SensorObject(objectNumber=oKT into SensorBody(bodyNumber=oKT
    - date resolved: **2024-04-17 16:13**\ , date raised: 2024-04-17 
    - resolved by: P. Manzl
 * Version 1.8.17: resolved Issue 1818: stub files (fix)
    - description:  add types for mainsystem extensions (e.g. SolutionViewer)
    - **notes:** not resolved, because mainSystemExtension functions do not contain argument types
    - date resolved: **2024-04-14 17:54**\ , date raised: 2024-04-14 
 * Version 1.8.16: resolved Issue 1817: stub files (fix)
    - description:  add types for system structures (mainly solver functions)
    - **notes:** not resolved, because this information is not available in the structures for solver methods
    - date resolved: **2024-04-14 17:53**\ , date raised: 2024-04-14 
 * Version 1.8.15: resolved Issue 1816: stub files (fix)
    - description:  fix a couple of wrong return types and similar wrong typings (in particular return types in SetODE2Coordinates, etc.
    - date resolved: **2024-04-14 16:15**\ , date raised: 2024-04-14 
 * Version 1.8.14: resolved Issue 1814: stub files (fix)
    - description:  .pyi files do not contain correct default args for C++ interfaces
    - date resolved: **2024-04-14 16:15**\ , date raised: 2024-04-14 
 * Version 1.8.13: resolved Issue 1815: stub files (fix)
    - description:  .pyi are missing backslash before underscore (for latex) documentation
    - date resolved: **2024-04-14 15:15**\ , date raised: 2024-04-14 
 * Version 1.8.12: resolved Issue 1812: GetMarkerOutput (fix)
    - description:  not working for MarkerKinematicTreeRigid in case of Reference configuration; adjustments done in CObjectKinematicTree::ComputeTreeTransformations to avoid the need for velocities in the retrieval of positions
    - date resolved: **2024-04-03 19:38**\ , date raised: 2024-04-03 
 * Version 1.8.11: resolved Issue 1811: Sensitivity Analysis for functions with single outputs (fix)
    - issue author: PM
    - description:  ComputeSensitivities and PlotSensitivityResults does not work for functions with a single output
    - date resolved: **2024-03-26 19:24**\ , date raised: 2024-03-26 
    - resolved by: P. Manzl
 * Version 1.8.10: resolved Issue 1810: GeneralContact (change)
    - description:  unify computation of contact forces and jacobians for sphere-sphere and trig-sphere contact
    - date resolved: **2024-03-25 08:21**\ , date raised: 2024-03-25 
 * Version 1.8.9: resolved Issue 1809: GeneralContact (fix)
    - description:  apply changes in sphere-sphere contact to Jacobian and to docu
    - date resolved: **2024-03-17 19:05**\ , date raised: 2024-03-17 
 * Version 1.8.8: :textred:`resolved BUG 1807` : GeneralContact 
    - description:  sphere-shpere contact causes spurious internal torque
    - **notes:** fixed by adding 0.5*penetration in lever arm; adjusted also documentation; gives new test suite results
    - date resolved: **2024-03-17 10:56**\ , date raised: 2024-03-17 
 * Version 1.8.7: :textred:`resolved BUG 1805` : CreateRigidBody 
    - description:  raises exception in case that initialRotationMatrix is not None; SOLUTION: replace == None for ALL cases (check other functions) with is None or is not Note!
    - **notes:** fixed with 1808
    - date resolved: **2024-03-17 10:55**\ , date raised: 2024-03-16 
 * Version 1.8.6: :textred:`resolved BUG 1806` : CreateRigidBody 
    - description:  initialRotationMatrix has no effect at least in explicit integration
    - **notes:** wrong operator* used for multiplication of reference rotation matrix and initial rotation matrix; FIXED
    - date resolved: **2024-03-17 10:54**\ , date raised: 2024-03-16 
 * Version 1.8.5: resolved Issue 1808: advanced utilities (extension)
    - description:  added function to check for None and not None: IsNone(x), IsNotNone(x)
    - date resolved: **2024-03-17 10:29**\ , date raised: 2024-03-17 
 * Version 1.8.4: resolved Issue 1804: GeneralContact (check)
    - description:  check triangle-sphere contact: torque for triangle computed with "((-sphereI.radius)\*deltaP0).CrossProduct(fVec)" lever arm should be -(trigPP - rigid.position) ?
    - **notes:** changed term for torque on rigid body according to added documentation in theory section
    - date resolved: **2024-03-17 10:15**\ , date raised: 2024-03-16 
 * Version 1.8.3: resolved Issue 1802: RigidBodySpringDamper (check)
    - description:  check intrinsic joint formulation
    - **notes:** no inconsistencies found or detected in examples
    - date resolved: **2024-03-13 13:10**\ , date raised: 2024-03-07 
 * Version 1.8.2: resolved Issue 1803: MarkerSuperElementRigid (extension)
    - description:  add option for tangent operator in alternativeFormulation
    - date resolved: **2024-03-13 13:09**\ , date raised: 2024-03-13 
 * Version 1.8.1: resolved Issue 0734: continuous integration (coding)
    - description:  test CI capabilities with GitHub and MacOS compilation
    - **notes:** resolved with issue 1792
    - date resolved: **2024-03-09 16:04**\ , date raised: 2021-08-12 
 * Version 1.8.0: resolved Issue 1789: AvailableItems (extension)
    - description:  add exudyn.special function to retrieve available items as dictionary with lists
    - date resolved: **2024-03-06 09:02**\ , date raised: 2024-02-21 

***********
Version 1.7
***********

 * Version 1.7.123: resolved Issue 1795: joint constraints (docu)
    - description:  theory: add description for formulation of joint constraints
    - **notes:** added equations to position markers and JointSpherical
    - date resolved: **2024-03-03 22:04**\ , date raised: 2024-02-25 
 * Version 1.7.122: resolved Issue 1800: GeneralContact (extension)
    - description:  add option for GetActiveContacts to return number of contacts per contact type in case that itemIndex=-1
    - date resolved: **2024-02-29 15:50**\ , date raised: 2024-02-29 
 * Version 1.7.121: resolved Issue 1799: exudyn __init__.py (change)
    - description:  remove NoAVX option for linux, as linux does not (yet) have a AVX2 option; crashed on linux arm/aarch architecture
    - date resolved: **2024-02-29 14:34**\ , date raised: 2024-02-29 
 * Version 1.7.120: resolved Issue 1797: Github actions (extension)
    - description:  create single line output for testsuite with specific mode; add test suite for github actions and merge outputs into single file
    - **notes:** put information into filename of text tilde with output of testsuite
    - date resolved: **2024-02-29 14:32**\ , date raised: 2024-02-27 
 * Version 1.7.119: resolved Issue 1798: linux arm (extension)
    - description:  add multilinux aarch64 wheels to GH build actions
    - date resolved: **2024-02-29 14:31**\ , date raised: 2024-02-29 
 * Version 1.7.118: resolved Issue 1796: MacOSX universal2 (extension)
    - description:  add build option for macos universal files on GH actions to have both arm and x86 on board
    - **notes:** NOTE that pip 20.3 is required to install these wheels!
    - date resolved: **2024-02-27 15:06**\ , date raised: 2024-02-27 
 * Version 1.7.117: resolved Issue 1794: fix curly brackets (docu)
    - description:  fix curly brackets {} in RST files
    - date resolved: **2024-02-25 20:43**\ , date raised: 2024-02-25 
 * Version 1.7.116: resolved Issue 1793: manylinux2014 (extension)
    - description:  build highly compatible manylinux2014 and manylinux2_17 wheels with github actions docker, to run on CentOS and Rocky Linux as well as ubuntu
    - date resolved: **2024-02-24 23:28**\ , date raised: 2024-02-24 
 * Version 1.7.115: resolved Issue 1792: Github actions CI (extension)
    - description:  add github actions to create automatically Windows, Ubunut and MacOS wheels
    - date resolved: **2024-02-24 23:28**\ , date raised: 2024-02-24 
 * Version 1.7.114: resolved Issue 1787: Python 3.12 (extension)
    - description:  include Python 3.12 wheels into build process
    - date resolved: **2024-02-24 23:25**\ , date raised: 2024-02-21 
 * Version 1.7.113: :textred:`resolved BUG 1791` : Autoregistration items 
    - description:  node does not initialize CData
    - date resolved: **2024-02-24 17:45**\ , date raised: 2024-02-24 
 * Version 1.7.112: resolved Issue 1788: items auto-registration (change)
    - description:  add a simple way to automatically register items; use C++ map to create item in object-factory
    - date resolved: **2024-02-21 19:17**\ , date raised: 2024-02-21 
 * Version 1.7.111: resolved Issue 1790: exudyn minimal (extension)
    - description:  add flag EXUDYN_MINIMAL_ITEMS to achieve fast compilation for testing
    - date resolved: **2024-02-21 18:53**\ , date raised: 2024-02-21 
 * Version 1.7.110: :textred:`resolved BUG 1786` : ComputeODE2singleLoad 
    - description:  raises error in static computation "inconsistent jacobian"; workaround settings computeLoadsJacobian=False or using sparse solver
    - **notes:** exception due to inconsistent computation of mass proportional load jacobian with rigid body
    - date resolved: **2024-02-16 11:42**\ , date raised: 2024-02-16 
 * Version 1.7.109: resolved Issue 1785: beam tutorial (docu)
    - description:  add beam tutorial example and tutorial in theDoc
    - date resolved: **2024-02-13 22:08**\ , date raised: 2024-02-13 
 * Version 1.7.108: resolved Issue 1784: theory theDoc (docu)
    - description:  add introduction to multibody dynamics, kinematics, dynamics, rotations
    - date resolved: **2024-02-13 16:10**\ , date raised: 2024-02-13 
 * Version 1.7.107: resolved Issue 1783: RST graphics (docu)
    - description:  add graphics for readthedocs representation, in solvers; fix references
    - date resolved: **2024-02-13 16:10**\ , date raised: 2024-02-13 
 * Version 1.7.106: resolved Issue 1782: GeneralContact (extension)
    - description:  add flag computeContactForces to settings, which computes contribution of contact forces to system vector (may slow down computations!); similar to issue 936
    - date resolved: **2024-02-12 09:07**\ , date raised: 2024-02-12 
 * Version 1.7.105: resolved Issue 0936: GeneralContact (extension)
    - description:  add interface function to get contact forces
    - date resolved: **2024-02-12 09:07**\ , date raised: 2022-02-10 
 * Version 1.7.104: resolved Issue 1781: GeneralContact (extension)
    - description:  add function Get/SetTriangleRigidBodyBased, to get data or modify data of current contact triangle
    - date resolved: **2024-02-08 20:38**\ , date raised: 2024-02-08 
 * Version 1.7.103: resolved Issue 1780: GeneralContact (extension)
    - description:  add function SetSphereMarkerBased to set data for spheres during simulation
    - date resolved: **2024-02-08 20:38**\ , date raised: 2024-02-08 
 * Version 1.7.102: resolved Issue 1779: GeneralContact (change)
    - description:  GetMarkerBasedSphere: change to GetSphereMarkerBased; add flag to decide whether to add basic data or not
    - date resolved: **2024-02-08 19:08**\ , date raised: 2024-02-08 
 * Version 1.7.101: resolved Issue 1778: cRGB settings (change)
    - description:  change cRGB consistently to RGBA in visualization settings
    - date resolved: **2024-02-08 18:57**\ , date raised: 2024-02-08 
 * Version 1.7.100: resolved Issue 1775: BodyGraphicsData (fix)
    - description:  access with Get/SetObjectParameter is missing for graphicsData
    - date resolved: **2024-02-04 22:01**\ , date raised: 2024-02-04 
 * Version 1.7.99: resolved Issue 1774: CreateSymbolicUserFunction (change)
    - description:  change order of args userFunctionName, itemIndex, and verbose; add additional itemTypeName
    - date resolved: **2024-02-04 00:36**\ , date raised: 2024-02-04 
 * Version 1.7.98: resolved Issue 1773: CreateSymbolicUserFunction (extension)
    - description:  add option to directly pass itemTypeName instead of itemIndex in order to pre-compute user function
    - date resolved: **2024-02-04 00:36**\ , date raised: 2024-02-04 
 * Version 1.7.97: resolved Issue 1771: TransferUserFunction2Item (change)
    - description:  add functionality to allow direct assignment of symbolic user functions to userFunction parameters in objects, loads, etc.; remove TransferUserFunction2Item as this function is then no longer needed
    - date resolved: **2024-02-03 23:55**\ , date raised: 2024-02-03 
 * Version 1.7.96: resolved Issue 1766: Python user functions (extension)
    - description:  use PythonUserFunctionBase class for all Item user functions; add consistent Get/Set function for item access; should then automatically work with pickle
    - date resolved: **2024-02-03 23:54**\ , date raised: 2024-02-02 
 * Version 1.7.95: resolved Issue 1750: python user functions (extension)
    - description:  add additional py::function to user functions in order to store original python function for pickling
    - date resolved: **2024-02-03 23:54**\ , date raised: 2024-01-29 
 * Version 1.7.94: resolved Issue 1759: Renderer (fix)
    - description:  there is an issue when restarting the renderer, which displays previous (old) data; requires to add some function which erases stored graphics data on call of StartRenderer()
    - date resolved: **2024-02-03 23:53**\ , date raised: 2024-01-31 
 * Version 1.7.93: resolved Issue 1770: mainsystem extensions (extensions)
    - description:  add user function to mbs.Create...() functions
    - date resolved: **2024-02-03 22:50**\ , date raised: 2024-02-03 
 * Version 1.7.92: resolved Issue 1763: Python user functions (extension)
    - description:  add pickle functionality (requires issue 1752)
    - date resolved: **2024-02-02 10:36**\ , date raised: 2024-01-31 
 * Version 1.7.91: resolved Issue 1752: user functions types (extension)
    - description:  create UserFunctionBase class and derived classes, containing std::function, and a py::object with metadata (Python function, Symbolic function, etc.); When settings user functions, they can be either initialized with 0 / Python function or with a UserFunction dict, which contains additional decorators; In particular, user functions will be rebuilt as symbolic; return values of user functions are then dictionaries
    - date resolved: **2024-02-02 10:36**\ , date raised: 2024-01-29 
 * Version 1.7.90: resolved Issue 1765: MainSystem user functions (extension)
    - description:  test special class for MainSystem user functions such as preStepUserFunction; if requeste, convert to Dict, in particular for symbolic or other special user functions
    - date resolved: **2024-02-02 10:33**\ , date raised: 2024-02-02 
 * Version 1.7.89: resolved Issue 1762: SystemContainer (extension)
    - description:  add pickle functionality
    - date resolved: **2024-01-31 20:22**\ , date raised: 2024-01-31 
 * Version 1.7.88: resolved Issue 1628: pickle MainSystem (extension)
    - description:  consider a pickle method for certain objects; add consistent info in description; MainSystem, SimulationSettings, VisualizationSettings
    - **notes:** consider with care, as not all things are copied (user functions, contact, ...)
    - date resolved: **2024-01-31 20:22**\ , date raised: 2023-06-23 
 * Version 1.7.87: resolved Issue 1761: pickle (extension)
    - description:  add pickle to settings and structures
    - date resolved: **2024-01-31 20:21**\ , date raised: 2024-01-31 
 * Version 1.7.86: resolved Issue 1760: pickle (extension)
    - description:  add pickle to ItemIndices
    - date resolved: **2024-01-31 20:21**\ , date raised: 2024-01-31 
 * Version 1.7.85: resolved Issue 1757: DictionariesGetSet (extension)
    - description:  add C++ GetDictionary(...) function for read/write of system structures; add Get/SetDictionary to pybind interface
    - date resolved: **2024-01-31 17:27**\ , date raised: 2024-01-30 
 * Version 1.7.84: resolved Issue 1758: StartRenderer (fix)
    - description:  add UpdateGraphicsDataNow() after start of renderer in order to avoid showing stored data for second run or renderer
    - date resolved: **2024-01-31 12:52**\ , date raised: 2024-01-31 
 * Version 1.7.83: resolved Issue 1755: CSystem in MainSystem (change)
    - description:  change CSystem\* to CSystem in MainSystem, to make copying easier
    - date resolved: **2024-01-30 16:43**\ , date raised: 2024-01-30 
 * Version 1.7.82: resolved Issue 1756: SystemContainer (change)
    - description:  remove SystemContainer and just keep MainSystemContainer, as it is not needed; simplifies copying
    - date resolved: **2024-01-30 16:42**\ , date raised: 2024-01-30 
 * Version 1.7.81: resolved Issue 1754: MainSystem (extension)
    - description:  change creation of MainSystem; allow construction like exudyn.MainSystem(), add new function Append(MainSystem) to MainSystemContainer; this will allow pickling both of MainSystem and MainSystemContainer
    - date resolved: **2024-01-30 14:34**\ , date raised: 2024-01-30 
 * Version 1.7.80: resolved Issue 1753: postStepUserFunction (extension)
    - description:  add user function to be called at end of time step, just before storing results to file; this allows to override results, etc.
    - date resolved: **2024-01-30 08:01**\ , date raised: 2024-01-30 
 * Version 1.7.79: resolved Issue 1748: user functions (fix)
    - description:  check option to set them to 0; PreStepUserFunction as well as item user functions
    - **notes:** Note that when reading user functions, mbs.GetObjectParameter(...) also consistently gives now 0 instead of previously None
    - date resolved: **2024-01-29 14:41**\ , date raised: 2024-01-29 
 * Version 1.7.78: resolved Issue 1749: User function (extension)
    - description:  allow assignment to 0 for MainSystem user functions; SetPreStepUserFunction and SetPostNewtonUserFunction
    - date resolved: **2024-01-29 14:34**\ , date raised: 2024-01-29 
 * Version 1.7.77: resolved Issue 1746: GenerateStraightBeam (extension)
    - description:  build generic function to create beams along straight line; add interface both for ANCF (old GenerateStraightLineANCFCable function) as well as for geometrically exact beam
    - date resolved: **2024-01-28 19:46**\ , date raised: 2024-01-28 
 * Version 1.7.76: resolved Issue 1747: GeometricallyExactBeam2D (extension)
    - description:  finish interface for load mass proportional
    - **notes:** also done for 3D version
    - date resolved: **2024-01-28 18:17**\ , date raised: 2024-01-28 
 * Version 1.7.75: resolved Issue 1745: geometrically exact beam 2D (change)
    - description:  adjust implementation of reference strains to allow connection of several beams at one node in case that includeReferenceRotations=0
    - date resolved: **2024-01-27 22:16**\ , date raised: 2024-01-27 
 * Version 1.7.74: resolved Issue 1744: geometrically exact beam 2D (docu)
    - description:  fix inconsistent documentation of reference strains
    - date resolved: **2024-01-27 22:16**\ , date raised: 2024-01-27 
 * Version 1.7.73: resolved Issue 1743: CreateRevoluteJoint (fix)
    - description:  Description for local/global axis is wrong; behavior is switched by useGlobalFrame flag
    - date resolved: **2024-01-06 11:20**\ , date raised: 2024-01-06 
 * Version 1.7.72: resolved Issue 1742: CreatePrismaticJoint (fix)
    - description:  Description for local/global axis is wrong; behavior is switched by useGlobalFrame flag
    - date resolved: **2024-01-06 11:20**\ , date raised: 2024-01-06 
 * Version 1.7.71: :textred:`resolved BUG 1741` : ObjectKinematicTree 
    - description:  SensorObject does not work with OutputVariableType Coordinates; example does not work;
    - **notes:** Coordinates, Force, etc. now available with GetObjectOutputBody and SensorBody
    - date resolved: **2024-01-06 11:02**\ , date raised: 2024-01-06 
 * Version 1.7.70: resolved Issue 1739: symbolic (fix)
    - description:  SetValue should raise exception if called with symbolic expression
    - date resolved: **2023-12-19 08:30**\ , date raised: 2023-12-19 
 * Version 1.7.69: resolved Issue 1737: symbolic (extension)
    - description:  add symbolic.pyi stub file for autocompletion of symbolic features
    - date resolved: **2023-12-18 19:48**\ , date raised: 2023-12-18 
 * Version 1.7.68: resolved Issue 1738: symbolic (docu)
    - description:  fix documentation for operators
    - date resolved: **2023-12-18 19:47**\ , date raised: 2023-12-18 
 * Version 1.7.67: resolved Issue 1727: SymbolicRealMatrix (docu)
    - description:  add documentation and example for symbolic matrix for user functions
    - date resolved: **2023-12-18 08:24**\ , date raised: 2023-12-12 
 * Version 1.7.66: :textred:`resolved BUG 1736` : symbolic 
    - description:  symbolic user function: crashes when user function object is deleted
    - date resolved: **2023-12-15 18:01**\ , date raised: 2023-12-15 
 * Version 1.7.65: resolved Issue 1735: symbolic (fix)
    - description:  check delete counts and reference counts for +=, etc.
    - date resolved: **2023-12-15 18:01**\ , date raised: 2023-12-15 
 * Version 1.7.64: resolved Issue 1729: Symbolic (testing)
    - description:  add vector/matrix tests in comparison with Python numpy and check delete counts
    - date resolved: **2023-12-15 13:52**\ , date raised: 2023-12-12 
 * Version 1.7.63: resolved Issue 1728: Symbolic (testing)
    - description:  add scalar tests in comparison with Python math and check delete counts
    - date resolved: **2023-12-15 13:52**\ , date raised: 2023-12-12 
 * Version 1.7.62: resolved Issue 1733: symbolic (check)
    - description:  check overloading __len__ operator for vector
    - date resolved: **2023-12-15 13:06**\ , date raised: 2023-12-15 
 * Version 1.7.61: resolved Issue 1734: symbolic (fix)
    - description:  write operator[] for Matrix and Vector fails
    - date resolved: **2023-12-15 11:51**\ , date raised: 2023-12-15 
 * Version 1.7.60: :textred:`resolved BUG 1732` : symbolic 
    - description:  Vector.SetVector(...), Matrix.SetMatrix(...) not working; fix Pybind interface
    - date resolved: **2023-12-15 11:12**\ , date raised: 2023-12-15 
 * Version 1.7.59: resolved Issue 1680: chatGPTupdate (example)
    - description:  add simple example for load userFunction
    - date resolved: **2023-12-14 00:01**\ , date raised: 2023-10-29 
 * Version 1.7.58: resolved Issue 1693: VObjectGround (fix)
    - description:  remove parameter color, as it is not used (check)
    - date resolved: **2023-12-13 23:40**\ , date raised: 2023-11-19 
 * Version 1.7.57: resolved Issue 1726: SymbolicRealMatrix (extension)
    - description:  add symbolic matrix for user functions
    - **notes:** note: currently implemented less efficient with memory allocations
    - date resolved: **2023-12-13 14:02**\ , date raised: 2023-12-12 
 * Version 1.7.56: resolved Issue 1731: ANCFCable2D (extension)
    - description:  add user functions for bending moment and axial force, allowing to implement arbitrary material models
    - date resolved: **2023-12-13 13:47**\ , date raised: 2023-12-13 
 * Version 1.7.55: resolved Issue 1730: GenerateStraightLineANCFCable (fix)
    - description:  raises Warning for default values [0,0,0,0,0,0] in 2D case
    - date resolved: **2023-12-13 13:27**\ , date raised: 2023-12-13 
 * Version 1.7.54: resolved Issue 1725: Symbolic (extension)
    - description:  add ResizableConstMatrix and create symbolic matrix-vector functions
    - date resolved: **2023-12-12 14:16**\ , date raised: 2023-12-10 
 * Version 1.7.53: resolved Issue 1716: Symbolic (docu)
    - description:  add symbolic user function description to documentation; also mention in performance section
    - date resolved: **2023-12-10 21:57**\ , date raised: 2023-12-08 
 * Version 1.7.52: resolved Issue 1724: C++ ToString (change)
    - description:  changing the behavior for standard conversion of double and int values to strings, in particular during errors. Now using the same precision as defined with exudyn.SetOutputPrecision()
    - date resolved: **2023-12-09 19:26**\ , date raised: 2023-12-09 
 * Version 1.7.51: resolved Issue 1715: Symbolic (docu)
    - description:  add symbolic section to documentation
    - date resolved: **2023-12-09 16:29**\ , date raised: 2023-12-08 
 * Version 1.7.50: resolved Issue 1708: Symbolic (extension)
    - description:  make symbolic variable space, available globally in exudyn.symbolic as well as in mbs.symbolic; this allows to store/transfer data into user functions without the need for Python; use integer handles which are returned by creation function: a=NamedReal(value, name); handle=AddVariableReal(a)
    - **notes:** not yet put into mbs and using no integer handles, but std::unordered_map, which has highly efficient hash table included
    - date resolved: **2023-12-09 16:29**\ , date raised: 2023-12-03 
 * Version 1.7.49: resolved Issue 1699: exudyn. ... (docu)
    - description:  add undocumented features of exudyn module, such as Demo1(), Demo2(), __version__ or C++
    - date resolved: **2023-12-08 18:41**\ , date raised: 2023-11-22 
 * Version 1.7.48: resolved Issue 1698: experimental, special (docu)
    - description:  add experimental and special features to documentation / pybindings
    - date resolved: **2023-12-08 18:41**\ , date raised: 2023-11-21 
 * Version 1.7.47: resolved Issue 1710: Symbolic (extension)
    - description:  add basic (automatic) differentiation feature for expressions: EvaluateDiff()
    - date resolved: **2023-12-08 07:22**\ , date raised: 2023-12-03 
 * Version 1.7.46: resolved Issue 1700: ResizableConstSizeVector (extension)
    - description:  consider a Vector with fixed size, which can be extended if necessary - e.g. for local variables in sensors or GetOutputVariable; is efficient for small vectors and still works for larger one
    - date resolved: **2023-12-07 23:03**\ , date raised: 2023-11-22 
 * Version 1.7.45: resolved Issue 1714: Symbolic (extension)
    - description:  add vector as symbolic expression, allowing vectors in user functions
    - date resolved: **2023-12-07 19:55**\ , date raised: 2023-12-07 
 * Version 1.7.44: resolved Issue 1713: user functions (change)
    - description:  change StdVector to StdVector3D and StdVector6D in relevant cases in order to achieve light-weight interface for symbolic interfaces
    - date resolved: **2023-12-07 19:53**\ , date raised: 2023-12-07 
 * Version 1.7.43: resolved Issue 1712: Symbolic (extension)
    - description:  add automatic creation of user functions to AutoGenerateObjects
    - date resolved: **2023-12-05 00:06**\ , date raised: 2023-12-03 
 * Version 1.7.42: resolved Issue 1711: Symbolic (extension)
    - description:  put most parts of specific user functioninto base SymbolicFunction; EvaluateUF: use variadic args to generalize
    - date resolved: **2023-12-05 00:06**\ , date raised: 2023-12-03 
 * Version 1.7.41: resolved Issue 1709: Symbolic (extension)
    - description:  add most of Pythons math module functions to symbolic functions list
    - date resolved: **2023-12-03 15:01**\ , date raised: 2023-12-03 
 * Version 1.7.40: resolved Issue 1705: Symbolic user function (extension)
    - description:  allow parallel computation of non-Python user functions
    - **notes:** this is enabled by adding user function after Assemble, thus objects are not registered to have Python user functions
    - date resolved: **2023-12-03 14:58**\ , date raised: 2023-11-28 
 * Version 1.7.39: resolved Issue 1706: RigidBodySpringDamper (docu)
    - description:  intrinsicFormulation: add test to test suite and document new functionality
    - date resolved: **2023-11-30 23:56**\ , date raised: 2023-11-30 
 * Version 1.7.38: resolved Issue 1707: RigidBodySpringDamper (check)
    - description:  intrinsicFormulation: check conserving properties of joint forces for two freely rotating bodies: additional torque of forces may be required in implementation
    - date resolved: **2023-11-30 23:39**\ , date raised: 2023-11-30 
 * Version 1.7.37: resolved Issue 1486: RigidBodySpringDamper (extension)
    - description:  extend for Lie group formulation, evaluating connectors at mid-configuration according to Masarati and Morandini
    - date resolved: **2023-11-30 20:51**\ , date raised: 2023-04-01 
 * Version 1.7.36: resolved Issue 1696: exudyn.symbolic (extension)
    - description:  add experimental expression trees for building symbolic expressions to be used for user functions
    - **notes:** basic symbolic functionality added; tested with SpringDamper user function, leading to speedup of 10 against regular Python function
    - date resolved: **2023-11-28 09:08**\ , date raised: 2023-11-21 
    - resolved by: EXTENSION
 * Version 1.7.35: resolved Issue 1704: solver timeout (extension)
    - description:  added module-wide flag for timeout: exudyn.special.solver.timeout in order to stop simulations after certain time; use with care
    - date resolved: **2023-11-22 09:43**\ , date raised: 2023-11-22 
 * Version 1.7.34: resolved Issue 1703: Pybind module (change)
    - description:  use suggestions of from search: pybind11, how to split my code into multiple modules/files - stackoverflow; should improve compilation time
    - **notes:** created 2 new pybind module files; no major compilation speedup visible on laptop
    - date resolved: **2023-11-22 08:24**\ , date raised: 2023-11-22 
 * Version 1.7.33: resolved Issue 1702: PyErr_CheckSignals (check)
    - description:  check if this method available in pybind helps to allow stopping long-lasting computations in exudyn
    - **notes:** still does not work in Spyder
    - date resolved: **2023-11-22 02:03**\ , date raised: 2023-11-22 
 * Version 1.7.32: resolved Issue 1701: RunCppUnitTests (change)
    - description:  move to exudyn.special.RunCppUnitTests
    - date resolved: **2023-11-22 00:29**\ , date raised: 2023-11-22 
 * Version 1.7.31: resolved Issue 1697: exudyn.experimental (change)
    - description:  change exudyn.Experimental() into exudyn.experimental structure for clearer view on it; not intended to be used widely
    - **notes:** see also issue 1613
    - date resolved: **2023-11-21 21:18**\ , date raised: 2023-11-21 
 * Version 1.7.30: resolved Issue 1694: load user functions (change)
    - description:  add stl and numpy bindings in order to have Vector3D converted into numpy arrays in loadVectorUserFunction
    - date resolved: **2023-11-19 23:05**\ , date raised: 2023-11-19 
 * Version 1.7.29: resolved Issue 1690: mainSystemExtensions (extension)
    - description:  add CreateGround() with referencePosition, referenceRotationMatrix and visualization
    - date resolved: **2023-11-19 23:05**\ , date raised: 2023-11-19 
 * Version 1.7.28: resolved Issue 1679: CreateTorque (extension)
    - description:  add create function for nodes and bodies (using node=None, body=None default) to be used either for nodes or bodies; automatically adds markers; bodyFixed=False; add option to add userFunction
    - date resolved: **2023-11-19 22:22**\ , date raised: 2023-10-29 
 * Version 1.7.27: resolved Issue 1678: CreateForce (extension)
    - description:  add create function for nodes and bodies (using node=None, body=None default) to be used either for nodes or bodies; automatically adds markers; bodyFixed=False; add option to add userFunction
    - date resolved: **2023-11-19 22:22**\ , date raised: 2023-10-29 
 * Version 1.7.26: resolved Issue 1689: mainSystemExtensions (extension)
    - description:  change bodyOrNodeList into bodyList; allow bodyOrNodeList as alternative arg, but avoid using in examples
    - date resolved: **2023-11-19 21:04**\ , date raised: 2023-11-19 
 * Version 1.7.25: resolved Issue 1688: mainSystemExtensions (extension)
    - description:  allow special case distance=0 in CreateDistanceConstraint; this will then create a SphericalJoint
    - date resolved: **2023-11-19 21:04**\ , date raised: 2023-11-19 
 * Version 1.7.24: resolved Issue 1687: mainSystemExtensions (extension)
    - description:  allow referenceLength=0 in ConnectorSpringDamper
    - date resolved: **2023-11-19 21:04**\ , date raised: 2023-11-19 
 * Version 1.7.23: resolved Issue 1667: ANCFCable (extension)
    - description:  add visulization with cylinders
    - date resolved: **2023-11-19 19:39**\ , date raised: 2023-10-16 
 * Version 1.7.22: resolved Issue 1686: SpringDamper (extension)
    - description:  allow springLength=0, as it does not cause problems in computations
    - **notes:** added special behavior for L=0, but velocity not equal 0; may cause convergence issues in particular for static problems; added special behavior for L=0, but velocity not equal 0; may cause convergence issues in particular for static problems; also allow referenceLength=0
    - date resolved: **2023-11-19 19:11**\ , date raised: 2023-11-19 
 * Version 1.7.21: resolved Issue 1685: ANCF cable with rigid marker (example)
    - description:  add example with ANCFCable2D and rigid body marker, prescribing rotation of one end
    - **notes:** ANCFrotatingCable2D.py
    - date resolved: **2023-11-07 14:04**\ , date raised: 2023-11-07 
 * Version 1.7.20: resolved Issue 1673: ReadTheDocs (fix)
    - description:  add the required .readthedocs.yaml file and move requirements.txt into docs folder, as it is related to sphinx only
    - date resolved: **2023-10-25 09:16**\ , date raised: 2023-10-25 
 * Version 1.7.19: resolved Issue 1672: LieGroup explicit integration (change)
    - description:  fixed lieGroupDataNodes; lieGroupDataNodes renamed into lieGroupNodes in explict integration
    - date resolved: **2023-10-16 10:00**\ , date raised: 2023-10-16 
 * Version 1.7.18: resolved Issue 1671: PlotSensor (change)
    - description:  use IsListOrArray function to check for non-empty offsets and factors, to comply with numpy arrays
    - date resolved: **2023-10-16 08:32**\ , date raised: 2023-10-16 
 * Version 1.7.17: resolved Issue 1670: GenerateStraightLineANCFCable (extension)
    - description:  add GenerateStraightLineANCFCable for 3D cables and adjust 2D version
    - date resolved: **2023-10-16 08:31**\ , date raised: 2023-10-16 
 * Version 1.7.16: resolved Issue 1669: NodePoint3DSlope (fix)
    - description:  adjust all test models and examples for new NodePointSlope... names
    - date resolved: **2023-10-16 08:02**\ , date raised: 2023-10-16 
 * Version 1.7.15: resolved Issue 1663: NodePoint3DSlope23 (check)
    - description:  check d/dt(A) ... computation of time derivative of rotation matrix, also check jacobians and angular velocities; add tests also for Slope12 node
    - date resolved: **2023-10-16 07:45**\ , date raised: 2023-10-15 
 * Version 1.7.14: resolved Issue 1666: NodePoint3DSlope1 (change)
    - description:  change to NodePointSlope1
    - date resolved: **2023-10-15 22:29**\ , date raised: 2023-10-15 
 * Version 1.7.13: resolved Issue 1665: Point3DS23 (change)
    - description:  remove Point3DS23 as its name is inconsistent with 3D convention and not really needed
    - date resolved: **2023-10-15 22:24**\ , date raised: 2023-10-15 
 * Version 1.7.12: resolved Issue 1664: NodePoint3DSlope23 (change)
    - description:  change to NodePointSlope23
    - date resolved: **2023-10-15 22:23**\ , date raised: 2023-10-15 
 * Version 1.7.11: resolved Issue 1075: ANCFCable (extension)
    - description:  add 3D version of cable element, not using BeamSection interface for compatibility with Cable2D
    - date resolved: **2023-10-15 14:45**\ , date raised: 2022-05-06 
 * Version 1.7.10: resolved Issue 1661: NodePoint3DSlope12 (extension)
    - description:  add ANCF node with slopes x/y for thin plate element
    - date resolved: **2023-10-15 14:21**\ , date raised: 2023-10-15 
 * Version 1.7.9: resolved Issue 1403: Lie group nodes (change)
    - description:  remove LieGroup node RigidBodyRotVecDataLG as it is not needed any more as it can be substituted with more efficient RigidBodyRotVecLG
    - **notes:** right now added comments in C++ files, not completely removed
    - date resolved: **2023-10-15 11:57**\ , date raised: 2023-01-18 
 * Version 1.7.8: resolved Issue 1659: remove math.h (change)
    - description:  change with <cmath> for C++ conformity
    - date resolved: **2023-10-12 17:04**\ , date raised: 2023-10-12 
 * Version 1.7.7: resolved Issue 1658: switch to MSVC2022 (change)
    - description:  change main_sln_Template.sln for 2022 update; use cl.exe from MSVC2022 for compilation of wheels; slightly increases performance
    - **notes:** compilation successful and TestSuite runs through
    - date resolved: **2023-10-12 17:04**\ , date raised: 2023-10-12 
 * Version 1.7.6: resolved Issue 1660: plot (change)
    - description:  change plt.tight_layout and plt.legend to fig. if possible to avoid warnings
    - date resolved: **2023-10-12 13:39**\ , date raised: 2023-10-12 
 * Version 1.7.5: resolved Issue 1657: Docu (fix)
    - description:  latex errors in robotics mobile and ROS
    - date resolved: **2023-10-09 20:27**\ , date raised: 2023-10-09 
 * Version 1.7.4: resolved Issue 1656: ROS rosInterface (extension)
    - description:  Created robotics.rosInterface and Python models in Examples: ROSMassPoint.py, ROSMobileManipulator.py, ROSTurtle.py with supplementary, see Examples/supplementary: ROSControlMobileManipulation.py, ROSControlTurtleVelocity.py, etc.
    - date resolved: **2023-09-15 15:50**\ , date raised: 2023-09-15 
    - resolved by: Martin Sereinig
 * Version 1.7.3: resolved Issue 1655: SolveStatic (extension)
    - description:  add option for static solver to handle ODE1 quantities; currently, the option is to set ODE1coordinates to initial values during static computation
    - date resolved: **2023-09-07 21:13**\ , date raised: 2023-09-07 
 * Version 1.7.2: resolved Issue 1654: Python3.6 support (change)
    - description:  discontinuing testing and creation of pip installers for Python3.6 in Windows, Linux and MacOS as Python3.6 had end-of-life 2021-12-23; Python 3.7 also had end-of-life recently, so please expect discontinued support soon
    - date resolved: **2023-09-03 16:03**\ , date raised: 2023-09-03 
 * Version 1.7.1: resolved Issue 1652: rosInterface.py (fix)
    - description:  add (missing) file to DOCU
    - date resolved: **2023-08-08 17:53**\ , date raised: 2023-08-08 
 * Version 1.7.0: resolved Issue 1649: release (release)
    - description:  switch to new release 1.7
    - date resolved: **2023-07-19 16:07**\ , date raised: 2023-07-19 

***********
Version 1.6
***********

 * Version 1.6.189: resolved Issue 1648: ReevingSystemSprings (fix)
    - description:  adjust test example for treating compression forces
    - date resolved: **2023-07-17 12:10**\ , date raised: 2023-07-17 
 * Version 1.6.188: resolved Issue 1647: sensorTraces (extension)
    - description:  add vectors and triads to position traces; show current vector or triad and add some further options for visualization, see visualizationSettings sensors.traces
    - **notes:** added triads and vectors, showing traces of motion at sensor points
    - date resolved: **2023-07-14 18:34**\ , date raised: 2023-07-14 
 * Version 1.6.187: resolved Issue 1640: visualization (extension)
    - description:  show trace of sensor positions (incl. frames) in render window; settings and list of sensors provided in visualizationSettings.sensors.traces with list of sensors, positionTrace, listOfPositionSensors=[] (empty means all position sensors, listOfVectorSensors=[] which can provide according vector quantities for positions; showVectors, vectorScaling=0.001, showPast=True, showFuture=False, showCurrent=True, lineWidth=2
    - date resolved: **2023-07-14 11:20**\ , date raised: 2023-07-11 
 * Version 1.6.186: resolved Issue 1646: ArrayFloat (extension)
    - description:  C++ add type
    - date resolved: **2023-07-13 16:27**\ , date raised: 2023-07-13 
 * Version 1.6.185: resolved Issue 1645: ReevingSystemSprings (extension)
    - description:  add way to remove compression forces in rope
    - **notes:** added parameter regularizationForce with tanh regularization for avoidance of compressive spring force
    - date resolved: **2023-07-12 16:05**\ , date raised: 2023-07-12 
 * Version 1.6.184: resolved Issue 1644: Minimize (extension)
    - description:  processing.Minimize: improve printout of current error of objective function=loss; only print every 1 second
    - date resolved: **2023-07-12 09:29**\ , date raised: 2023-07-12 
 * Version 1.6.183: :textred:`resolved BUG 1643` : Minimize 
    - description:  processing.Minimize function has an internal bug, such that it does not work with initialGuess=[]
    - date resolved: **2023-07-12 09:29**\ , date raised: 2023-07-12 
 * Version 1.6.182: :textred:`resolved BUG 1642` : SolutionViewer 
    - description:  record image not working with visualizationSettings useMultiThreadedRendering=True
    - date resolved: **2023-07-11 17:58**\ , date raised: 2023-07-11 
 * Version 1.6.181: resolved Issue 1641: SolutionViewer (fix)
    - description:  github issue#51: graphicsDataUserFunction in SolutionViewer not called; add call to graphicsData user functions in redraw image loop
    - date resolved: **2023-07-11 17:16**\ , date raised: 2023-07-11 
 * Version 1.6.180: resolved Issue 1638: GeneticOptimization (extension)
    - description:  add argument parameterFunctionData to GeneticOptimization; same as in ParameterVariation, paramterFunctionData allows to pass additional data to the objective function
    - date resolved: **2023-07-09 09:15**\ , date raised: 2023-07-09 
 * Version 1.6.179: resolved Issue 1637: add ChatGPT update information (extension)
    - description:  create Python model Examples/chatGPTupdate.py which includes information that is used by ChatGPT4 to improve abilities to create simple models fully automatic
    - date resolved: **2023-06-30 15:01**\ , date raised: 2023-06-30 
 * Version 1.6.178: resolved Issue 1636: CreateMassPoint (change)
    - description:  change args referenceCoordinates to referencePosition, initialCoordinates to initialDisplacement, and initialVelocities to initialVelocity to be consistent with CreateRigidBody (but different from MassPoint itself)
    - date resolved: **2023-06-30 14:09**\ , date raised: 2023-06-30 
 * Version 1.6.177: resolved Issue 1635: SmartRound2String (extension)
    - description:  add function to basic utilities to enable simple printing of numbers with few digits, including comma dot and not eliminating small numbers, e.g., 1e-5 stays 1e-5
    - date resolved: **2023-06-30 09:22**\ , date raised: 2023-06-30 
 * Version 1.6.176: resolved Issue 1634: create directories (extension)
    - description:  add automatic creation of directories to FEM SaveToFile, plotting and ParameterVariation
    - date resolved: **2023-06-29 10:29**\ , date raised: 2023-06-29 
 * Version 1.6.175: :textred:`resolved BUG 1633` : GetInterpolatedSignalValue 
    - description:  timeArray needs to be replaced with timeArrayNew in case of 2D input array
    - date resolved: **2023-06-28 21:15**\ , date raised: 2023-06-28 
 * Version 1.6.174: resolved Issue 1632: PlotFFT (fix)
    - description:  matplotlib >= 1.7 complains about ax.grid(b=...) as parameter b has been replaced by visible
    - date resolved: **2023-06-27 14:15**\ , date raised: 2023-06-27 
 * Version 1.6.173: resolved Issue 1626: mutable args itemInterface (fix)
    - description:  also copy dictionaries, mainly for visualization (flat level, but this should be sufficient)
    - date resolved: **2023-06-21 10:40**\ , date raised: 2023-06-21 
 * Version 1.6.172: resolved Issue 1627: mutable default args (change)
    - description:  complete changes and adaptations for default args in Python functions and item interface; note individual adaptations for lists, vectors, matrices and special lists of lists or matrix containers; for itemInterface, anyway all data is copied into C++; for more information see issues 1536, 1540, 1612, 1624, 1625, 1626
    - date resolved: **2023-06-21 10:15**\ , date raised: 2023-06-21 
 * Version 1.6.171: resolved Issue 1625: change to default arg None (change)
    - description:  change default args for Vector2DList, Vector3DList, Vector6DList, Matrix3DList, to None; ArrayNodeIndex, ArrayMarkerIndex, ArraySensorIndex obtain copy method and are copied now; avoid problem of mutable default args
    - date resolved: **2023-06-21 00:26**\ , date raised: 2023-06-20 
 * Version 1.6.170: resolved Issue 1624: MatrixContainer (change)
    - description:  change default values for matrix container to None; avoid problem of mutable default args
    - date resolved: **2023-06-20 23:39**\ , date raised: 2023-06-20 
 * Version 1.6.169: resolved Issue 1540: mutable args itemInterface (check)
    - description:  copy lists in itemInterface in order to avoid change of default args by user n=NodePoint();n.referenceCoordinates[0]=42;n1=NodePoint()
    - **notes:** simple vectors, matrices and lists are copied with np.array(...) while complex matrix and list of array types are now initialized with None
    - date resolved: **2023-06-20 22:13**\ , date raised: 2023-04-28 
 * Version 1.6.168: resolved Issue 1620: docu MainSystemExtensions (docu)
    - description:  reorder MainSystemExtensions with separate section for Create functions and one section for remaining functions
    - date resolved: **2023-06-19 22:14**\ , date raised: 2023-06-13 
 * Version 1.6.167: :textred:`resolved BUG 1622` : mouse click 
    - description:  fix crash on linux if left / right mouse click on render window (related to OpenGL select window)
    - **notes:** occurs on WSL with WSLg; using 'export LIBGL_ALWAYS_SOFTWARE=1' will resolve the problem; put this line into your .bashrc
    - date resolved: **2023-06-19 22:11**\ , date raised: 2023-06-19 
 * Version 1.6.166: :textred:`resolved BUG 1621` : LinearSolverType 
    - description:  fix crash on linux in function SetLinearSolverType
    - date resolved: **2023-06-19 20:27**\ , date raised: 2023-06-19 
 * Version 1.6.165: resolved Issue 1623: MouseSelectOpenGL (extension)
    - description:  add optional debbuging output
    - date resolved: **2023-06-19 19:56**\ , date raised: 2023-06-19 
 * Version 1.6.164: resolved Issue 1267: matrix inverse (extension)
    - description:  add pivot threshold to options, may improve redundant constraints problems
    - **notes:** only available for EigenDense with ignoreSingularJacobian and EXUdense linear solvers
    - date resolved: **2023-06-12 13:24**\ , date raised: 2022-09-21 
 * Version 1.6.163: resolved Issue 1616: eigen LU (check)
    - description:  check fastest solver for regular and overdetermined systems
    - **notes:** Eigen::PartialPivLU 2.5 times faster for 65 DOF test in factorization, factor 3 faster for backsubst
    - date resolved: **2023-06-12 13:22**\ , date raised: 2023-06-11 
 * Version 1.6.162: resolved Issue 1607: Bricard mechanism (testing)
    - description:  add example and test ComputeSystemDegreesOfFreedom
    - date resolved: **2023-06-12 11:02**\ , date raised: 2023-06-09 
 * Version 1.6.161: resolved Issue 1617: solver error message (fix)
    - description:  revise hint for ignoreSingularJacobian
    - date resolved: **2023-06-12 01:39**\ , date raised: 2023-06-11 
 * Version 1.6.160: resolved Issue 1615: pivotThreshold (fix)
    - description:  add already existing parameter [which is currently not used in solver!] to solver interface add FactorizeNew arg
    - date resolved: **2023-06-12 01:39**\ , date raised: 2023-06-11 
 * Version 1.6.159: resolved Issue 1266: matrix inverse (extension)
    - description:  add full pivoting mode for matrix inverse, to resolve redundant constraints; consider Eigen FullPivotLU for dense matrices - see classEigen_1_1FullPivLU.html
    - **notes:** also added new LinearSolverType.EigenDense which allows to chose FullPivLU by settings ignoreSingularJacobian=True
    - date resolved: **2023-06-12 00:17**\ , date raised: 2022-09-21 
 * Version 1.6.158: resolved Issue 1619: LinearSolverType (change)
    - description:  switch to bit-wise numbering of solver types, in order to alleviate checks
    - date resolved: **2023-06-11 23:52**\ , date raised: 2023-06-11 
 * Version 1.6.157: resolved Issue 1618: ignoreRedundantConstraints (change)
    - description:  remove option ignoreRedundantConstraints as it cannot be applied with Eigen::FullPivLU; use ignoreSingularJacobian instead
    - date resolved: **2023-06-11 23:46**\ , date raised: 2023-06-11 
 * Version 1.6.156: resolved Issue 1613: Experimental (extension)
    - description:  add experimental class, which can be accessed in Python by exudyn.Experimental(); inside C++, just needs to be imported; allows simple testing without interference with main features
    - date resolved: **2023-06-11 19:56**\ , date raised: 2023-06-11 
 * Version 1.6.155: resolved Issue 1536: mutable arguments (fix)
    - description:  check and fix Python functions with mutable arguments such as [] or {}, with potential risk of changing internally in function, leading to unexpected behavior in second call
    - **notes:** checked all default list and dict args
    - date resolved: **2023-06-11 00:24**\ , date raised: 2023-04-27 
 * Version 1.6.154: resolved Issue 1612: mutable arguments (fix)
    - description:  check and fix problems in beams.py, FEM.py and graphicsDataUtilities.py
    - **notes:** see also issue 1536
    - date resolved: **2023-06-10 21:34**\ , date raised: 2023-06-10 
 * Version 1.6.153: resolved Issue 1609: AnimateModes (extension)
    - description:  extend for using a set of system eigenmodes
    - date resolved: **2023-06-10 20:25**\ , date raised: 2023-06-10 
 * Version 1.6.152: resolved Issue 1580: mainSystemExtensions (extension)
    - description:  add RigidBodySpringDamper
    - date resolved: **2023-06-10 20:25**\ , date raised: 2023-05-21 
 * Version 1.6.151: resolved Issue 1606: ComputeODE2Eigenvalues (extension)
    - description:  add eigenvector computation to constrained case
    - **notes:** needs further testing!
    - date resolved: **2023-06-10 19:11**\ , date raised: 2023-06-08 
 * Version 1.6.150: resolved Issue 1611: SolutionViewer (change)
    - description:  change internal variables from mbs.variables to mbs.sys
    - date resolved: **2023-06-10 17:48**\ , date raised: 2023-06-10 
 * Version 1.6.149: resolved Issue 1610: AnimateModes (change)
    - description:  change internal variables from mbs.variables to mbs.sys
    - date resolved: **2023-06-10 17:47**\ , date raised: 2023-06-10 
 * Version 1.6.148: :textred:`resolved BUG 1608` : visualization zoom all 
    - description:  when calling ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues and similar functions, and StartRenderer is called right afterwards, zoom all does not work
    - **notes:** shall be resolved just by calling StartRenderer before first call to any solver functionality
    - date resolved: **2023-06-10 11:44**\ , date raised: 2023-06-10 
 * Version 1.6.147: resolved Issue 1435: solver (extension)
    - description:  add deriviative of loads to regular jacobian computation with flag (default=False); use numerical diff sim. to JacobianODE2
    - **notes:** already done earlier in #1546
    - date resolved: **2023-06-08 23:35**\ , date raised: 2023-02-16 
 * Version 1.6.146: resolved Issue 1597: Command execute (fix)
    - description:  switch to grid method for placing widgets, tkinter does not allow pack and grid in different windows
    - date resolved: **2023-06-08 21:56**\ , date raised: 2023-06-05 
 * Version 1.6.145: :textred:`resolved BUG 1593` : TemporaryComputationDataArray bug 
    - description:  ERROR: "TemporaryComputationDataArray::operator[]: index out of range" is raised if single-threaded computation is run after multi-threaded simulation; requires restart of Python instance
    - date resolved: **2023-06-08 18:50**\ , date raised: 2023-06-03 
 * Version 1.6.144: resolved Issue 1599: ComputeODE2Eigenvalues (extension)
    - description:  compute eigenmodes in case of algebraic equations
    - date resolved: **2023-06-08 18:46**\ , date raised: 2023-06-08 
 * Version 1.6.143: resolved Issue 1603: AddSensor (extension)
    - description:  add check if no outputVariable is provided-> immediately raise error
    - **notes:** was already in system checks, which however were not called, see issue 1604
    - date resolved: **2023-06-08 18:45**\ , date raised: 2023-06-08 
 * Version 1.6.142: resolved Issue 1598: eigenvalues constrained system (example)
    - description:  add example for eigenvalue computation of constrained system
    - **notes:** added computeODE2AEeigenvaluesTest.py
    - date resolved: **2023-06-08 18:45**\ , date raised: 2023-06-08 
 * Version 1.6.141: resolved Issue 1605: ComputeSystemDegreeOfFreedom (change)
    - description:  change output to a dictionary in order to have readable results
    - date resolved: **2023-06-08 18:35**\ , date raised: 2023-06-08 
 * Version 1.6.140: resolved Issue 1604: Sensors (fix)
    - description:  PreAssembleConsistencies not called in Assemble; thus, no checks are performed on sensor inputs
    - date resolved: **2023-06-08 18:24**\ , date raised: 2023-06-08 
 * Version 1.6.139: :textred:`resolved BUG 1602` : MainSystem CreateRigidBody 
    - description:  referenceRotationMatrix multiplied in wrong way with initialRotationMatrix
    - **notes:** also rotation parameters in initialVelocities were wrong for initialRotationMatrix!=np.eye(3); fixed, but more testing needed
    - date resolved: **2023-06-08 17:42**\ , date raised: 2023-06-08 
 * Version 1.6.138: resolved Issue 1600: stub files (extension)
    - description:  extend .pyi files for system structures functions, e.g., ComputeJacobianODE2RHS
    - date resolved: **2023-06-08 17:17**\ , date raised: 2023-06-08 
 * Version 1.6.137: resolved Issue 1601: stub files (change)
    - description:  merge .pyi files to have classes such as MainSystem only appearing once
    - date resolved: **2023-06-08 16:37**\ , date raised: 2023-06-08 
 * Version 1.6.136: resolved Issue 0746: ComputeODEEigenvalues (extension)
    - description:  add possibility to eliminate coordinate constraints, possibly to use SVD/ null space matrix for projection
    - **notes:** algebraic constraints now considered automatically; algebraic constraints now considered automatically
    - date resolved: **2023-06-07 23:52**\ , date raised: 2021-09-03 
    - resolved by: M. Pieber, JG
 * Version 1.6.135: resolved Issue 0743: ComputeODE2Eigenvalues (extension)
    - description:  add vector of constrained coordinates which are eliminated; also add functionality for complex eigenvalues
    - **notes:** already done earlier
    - date resolved: **2023-06-07 23:09**\ , date raised: 2021-08-20 
 * Version 1.6.134: resolved Issue 1595: Command dialog (extension)
    - description:  extend to multi-line commands; execute code using CTRL-Return
    - **notes:** Behavior is now DIFFERENT, as variables are not printed automatically; write e.g. print(mbs) to see mbs representation; see section Execute Command and Help
    - date resolved: **2023-06-04 19:40**\ , date raised: 2023-06-03 
 * Version 1.6.133: resolved Issue 1596: linuxDisplayScaleFactor (extension)
    - description:  add scaling for fonts on linux, specifically for high resolution screens
    - date resolved: **2023-06-04 00:13**\ , date raised: 2023-06-04 
 * Version 1.6.132: resolved Issue 1588: ParameterVariation, useMPI (fix)
    - description:  only accept useMPI if set True
    - date resolved: **2023-06-03 19:26**\ , date raised: 2023-05-31 
 * Version 1.6.131: resolved Issue 1579: mainSystemExtensions (extension)
    - description:  Add distance constraint and CartesianSpringDamper
    - date resolved: **2023-06-03 19:26**\ , date raised: 2023-05-21 
 * Version 1.6.130: resolved Issue 1594: window closing, key Q (change)
    - description:  slightly adapt behavior; fix some smaller issues with expected window behavior
    - date resolved: **2023-06-03 17:01**\ , date raised: 2023-06-03 
 * Version 1.6.129: resolved Issue 1356: CHECK (extension)
    - description:  Add security question on quit/escape if computation Renderer runs longer than 15 minutes
    - **notes:** added message in render window to click twice on exit window icon (X) after 15 minutes; key Q and escape get tkinter message box
    - date resolved: **2023-06-03 16:00**\ , date raised: 2023-01-01 
 * Version 1.6.128: resolved Issue 1592: SolverBase it.endTime (fix)
    - description:  in dynamic solver it.endTime is overwritten with simulationSettings.timeIntegration.endTime; this is against the description and does not allow to change it.endTime in command window; remove overwritting to be consistent with description of Execute Command and Help section in EXUDYN Basics
    - date resolved: **2023-06-03 14:25**\ , date raised: 2023-06-03 
 * Version 1.6.127: resolved Issue 1591: SphericalJoint (fix)
    - description:  causes memory allocation; check LinkedDataVector cast
    - date resolved: **2023-06-01 11:12**\ , date raised: 2023-06-01 
 * Version 1.6.126: resolved Issue 1590: serialRobotKinematicTree.py (fix)
    - description:  fixed static torque compensation for kinematic tree and fixed sensor outputs
    - date resolved: **2023-05-31 13:07**\ , date raised: 2023-05-31 
 * Version 1.6.125: resolved Issue 1589: ObjectKinematicTree (docu)
    - description:  add description of SensorKinematicTree output variables per link
    - date resolved: **2023-05-31 12:42**\ , date raised: 2023-05-31 
 * Version 1.6.124: resolved Issue 1587: solution file footer (change)
    - description:  add comma after cpuTime=...
    - date resolved: **2023-05-30 23:52**\ , date raised: 2023-05-30 
 * Version 1.6.123: resolved Issue 1586: SetPreStepUserFunction, SetPostNewtonUserFunction (extension)
    - description:  add exception handling in set function
    - date resolved: **2023-05-26 19:52**\ , date raised: 2023-05-26 
 * Version 1.6.122: resolved Issue 1585: class name highlighting RTD (docu)
    - description:  fixed exporting class names from pythonUtilities
    - date resolved: **2023-05-25 15:19**\ , date raised: 2023-05-25 
 * Version 1.6.121: resolved Issue 1584: MiniExamples (change)
    - description:  remove import of itemInterface
    - date resolved: **2023-05-25 14:32**\ , date raised: 2023-05-25 
 * Version 1.6.120: :textred:`resolved BUG 1583` : GeneticOptimization 
    - description:  results are erroneous in case of crossoverProbability > 0
    - **notes:** fixed writing of output files which had mixed order due to parameter cross-over
    - date resolved: **2023-05-23 18:31**\ , date raised: 2023-05-23 
 * Version 1.6.119: resolved Issue 1582: mainSystemExtensions (fix)
    - description:  remove import of tkinter and matplotlib to resolve errors when loading exudyn and these libs are not installed
    - date resolved: **2023-05-22 10:54**\ , date raised: 2023-05-22 
 * Version 1.6.118: resolved Issue 1577: examples (fix)
    - description:  test if all Examples are still running
    - date resolved: **2023-05-20 22:23**\ , date raised: 2023-05-20 
 * Version 1.6.117: resolved Issue 1578: ObjectConnectorCoordinateSpringDamper (fix)
    - description:  correct docu on object and user function description (still includes friction)
    - date resolved: **2023-05-20 21:13**\ , date raised: 2023-05-20 
 * Version 1.6.116: resolved Issue 1569: mainSystemExtensions (example)
    - description:  add mini-examples for extensions
    - **notes:** collected miniexamples in mainSystemExtensionsTests.py; 
    - date resolved: **2023-05-20 21:05**\ , date raised: 2023-05-15 
 * Version 1.6.115: resolved Issue 1571: mainSystemExtensions (change)
    - description:  adapt Examples to Python extensions (SolveDynamic, CreateRigidBody, CreateGenericJoint, ...)
    - date resolved: **2023-05-18 23:43**\ , date raised: 2023-05-15 
 * Version 1.6.114: resolved Issue 1570: mainSystemExtensions (change)
    - description:  adapt TestModels to Python extensions
    - date resolved: **2023-05-18 23:43**\ , date raised: 2023-05-15 
 * Version 1.6.113: :textred:`resolved BUG 1576` : multithreading 
    - description:  running laserScannerTest.py after a multithreaded contact computation raises the EXCEPTION: TemporaryComputationDataArray::operator[]: index out of range
    - date resolved: **2023-05-18 23:13**\ , date raised: 2023-05-18 
 * Version 1.6.112: resolved Issue 1575: ConnectorDistance (change)
    - description:  changed parameter distance to PReal, not allowing zero distance to be prescribed
    - date resolved: **2023-05-18 13:03**\ , date raised: 2023-05-18 
 * Version 1.6.111: resolved Issue 1573: mainSystemExtensions (docu)
    - description:  Adapt tutorials to new functionality
    - date resolved: **2023-05-18 12:19**\ , date raised: 2023-05-16 
 * Version 1.6.110: resolved Issue 1574: Python utilities (fix)
    - description:  classes not appearing in table of contents on RTD
    - date resolved: **2023-05-17 20:21**\ , date raised: 2023-05-16 
 * Version 1.6.109: resolved Issue 1572: CreateRigidBody (extension)
    - description:  added to MainSystem to enable mbs.CreateRigidBody(...)
    - date resolved: **2023-05-16 11:54**\ , date raised: 2023-05-16 
 * Version 1.6.108: resolved Issue 1568: mainSystemExtensions (extension)
    - description:  create first sample of Python extensions for basic joints
    - date resolved: **2023-05-15 18:13**\ , date raised: 2023-05-15 
 * Version 1.6.107: resolved Issue 1567: DrawSystemGraph (change)
    - description:  in case of showItemNames, no item numbers are shown as they confuse with numbers used in names
    - date resolved: **2023-05-15 17:18**\ , date raised: 2023-05-15 
 * Version 1.6.106: resolved Issue 1566: AddDistanceSensor(...) (change)
    - description:  function RENAMED into CreateDistanceSensor(...) to be consistent with future naming; also renamed DistanceSensorSetupGeometry(...) into CreateDistanceSensorGeometry(...)
    - date resolved: **2023-05-15 11:34**\ , date raised: 2023-05-15 
 * Version 1.6.105: resolved Issue 1563: MainSystem Python extensions (extension)
    - description:  add Python utility functions for mbs, such as PlotSensor, SolveDynamic, ...; use identical interfaces to alleviate creation of .pyi files and documentation; add new flag mbsFunction as hint to put docu to MainSystem and make .pyi extension
    - **notes:** see Section :ref:`sec-mainsystem-pythonextensions`\  for extended functionality
    - date resolved: **2023-05-15 01:17**\ , date raised: 2023-05-09 
 * Version 1.6.104: resolved Issue 1564: Type definitions (docu)
    - description:  fix header structure in latex and RST for Type Definitions
    - date resolved: **2023-05-11 11:30**\ , date raised: 2023-05-11 
 * Version 1.6.103: resolved Issue 1561: stub files .pyi (extension)
    - description:  add .pyi files to setup_tools, copying them from autogenerate folder; use try catch to avoid problems at other platforms
    - date resolved: **2023-05-10 23:30**\ , date raised: 2023-05-09 
 * Version 1.6.102: resolved Issue 1560: stub files .pyi (extension)
    - description:  automatically create stub file for C++ classes such as MainSystem, SystemContainer, GeneralContact, ... by adding return type information and creating .pyi file; use temporary .pyi files in autogenerate folder
    - date resolved: **2023-05-10 23:19**\ , date raised: 2023-05-09 
 * Version 1.6.101: resolved Issue 1562: stub files .pyi (extension)
    - description:  add .pyi files for enums from autoGeneratePyBindings
    - date resolved: **2023-05-10 20:43**\ , date raised: 2023-05-09 
 * Version 1.6.100: resolved Issue 1559: stub files .pyi (extension)
    - description:  automatically create stub file for settings to alleviate auto-completion; type completion now also works for functions, types and structures: tested in Spyder and Visual Studio Code
    - date resolved: **2023-05-10 08:39**\ , date raised: 2023-05-09 
 * Version 1.6.99: resolved Issue 1557: stub files (check)
    - description:  test creating stub files .pyi which are needed for MainSystem Python extensions
    - **notes:** tested with Spyder 5.1.5 and Visual Studio Code 1.78
    - date resolved: **2023-05-10 08:39**\ , date raised: 2023-05-07 
 * Version 1.6.98: resolved Issue 1558: enum in global scope (fix)
    - description:  pybind11 translates enums to global module scope, e.g. exudyn.ItemType.Marker is also available as exudyn.Marker; remove export_values() in pybind interface
    - **notes:** if you by occasion used e.g. exu.DisplacementLocal instead of exu.OutputVariableType.DisplacementLocal you need to adapt your code!
    - date resolved: **2023-05-09 18:30**\ , date raised: 2023-05-09 
 * Version 1.6.97: resolved Issue 1556: IsValidPRealPInt, IsValidURealPInt (extension)
    - description:  add functions to advancedUtilities for additional checks in Python functions
    - date resolved: **2023-05-07 20:33**\ , date raised: 2023-05-07 
 * Version 1.6.96: resolved Issue 1555: color4default (extension)
    - description:  add default color to graphicsDataUtilities as a default value for items
    - date resolved: **2023-05-07 18:12**\ , date raised: 2023-05-07 
 * Version 1.6.95: resolved Issue 1554: NodePoint2D (change)
    - description:  draw as sphere by default to improve visibility
    - date resolved: **2023-05-07 16:31**\ , date raised: 2023-05-07 
 * Version 1.6.94: resolved Issue 1539: item names (change)
    - description:  remove stored string and replace by empty string in case of default item name
    - **notes:** not changed: std::string has practically no effect on memory footprint of items; check memory footprint separately (LTG lists, etc.)!
    - date resolved: **2023-05-05 23:32**\ , date raised: 2023-04-28 
 * Version 1.6.93: resolved Issue 1553: AccelerationLocal, AngularAccelerationLocal (fix)
    - description:  add missing values to Pybind interface
    - date resolved: **2023-05-02 17:41**\ , date raised: 2023-05-02 
 * Version 1.6.92: resolved Issue 1552: HydraulicActuator (extension)
    - description:  add VelocityLocal as output variable, which provides the time derivative of the distance, being the actuator velocity; updated docu and fixed description for outputvariable Velocity
    - date resolved: **2023-05-02 16:51**\ , date raised: 2023-05-02 
 * Version 1.6.91: resolved Issue 1551: GeometricallyExactBeam (change)
    - description:  evaluate rotation at midspan of beam
    - date resolved: **2023-05-02 15:02**\ , date raised: 2023-05-02 
 * Version 1.6.90: resolved Issue 1502: ANCFBeam (testing)
    - description:  check for very large deformations
    - **notes:** poor performance for large deformations caused by missing load jacobian
    - date resolved: **2023-05-01 19:26**\ , date raised: 2023-04-08 
 * Version 1.6.89: resolved Issue 1501: GeometricallyExactBeam (testing)
    - description:  check for very large deformations
    - **notes:** poor performance for large deformations caused by missing load jacobian
    - date resolved: **2023-05-01 19:26**\ , date raised: 2023-04-08 
 * Version 1.6.88: resolved Issue 1546: computeLoadsJacobian (change)
    - description:  add flag in timeIntegration and staticSolver settings to turn on/off jacobian of loads; will be by default turned on in static solver, turned off in time integration; adapt your models
    - date resolved: **2023-05-01 19:22**\ , date raised: 2023-05-01 
 * Version 1.6.87: resolved Issue 1545: ComputeSingleLoads (extension)
    - description:  adapt for Jacobian computation of loads
    - date resolved: **2023-05-01 19:22**\ , date raised: 2023-05-01 
 * Version 1.6.86: resolved Issue 1547: GetMarkerOutput (fix)
    - description:  in case of OuputVariableType.Coordinates, add check if Marker is of type Coordinate or Coordinates
    - date resolved: **2023-05-01 16:17**\ , date raised: 2023-05-01 
 * Version 1.6.85: resolved Issue 1544: systemData.GetNodeLocalToGlobal (extension)
    - description:  add systemData access functions for LTG mappings of nodes, useful to check node coordinates; works for ODE2, ODE1, AE and Data coordinates; useful to create load dependencies
    - date resolved: **2023-04-29 22:03**\ , date raised: 2023-04-29 
 * Version 1.6.84: resolved Issue 1543: systemData (extension)
    - description:  add InfoLTG function which outputs LTG lists and load dependencies
    - date resolved: **2023-04-29 22:03**\ , date raised: 2023-04-29 
 * Version 1.6.83: resolved Issue 0483: URDF file (extension)
    - description:  check importing an URDF file for robots, urdf_parser_py
    - **notes:** not done: should be done instead by Corke robotics-toolbox
    - date resolved: **2023-04-29 20:20**\ , date raised: 2020-12-04 
 * Version 1.6.82: resolved Issue 1542: HydraulicsActuator (change)
    - description:  changing referenceVolume0 and referenceVolume1 to hoseVolume0 and hoseVolume1 with different meaning according to referenced paper; adjust your models!
    - **notes:** thanks to Qasim Khadim for provigind the model
    - date resolved: **2023-04-29 01:11**\ , date raised: 2023-04-29 
 * Version 1.6.81: resolved Issue 1541: HydraulicsActuator (extension)
    - description:  extend model of effective bulk modulus acc. to paper https://doi.org/10.1007/s11044-019-09696-y
    - date resolved: **2023-04-29 01:10**\ , date raised: 2023-04-28 
 * Version 1.6.80: resolved Issue 1538: taskmanager (fix)
    - description:  fix problem with taskmanager shutdown due to issue 1532
    - date resolved: **2023-04-27 15:35**\ , date raised: 2023-04-27 
 * Version 1.6.79: resolved Issue 1537: output.multiThreadingMode (fix)
    - description:  has not been set correctly in solver so far; switch output.numberOfThreadsUsed and output.multiThreadingMode
    - date resolved: **2023-04-27 15:35**\ , date raised: 2023-04-27 
 * Version 1.6.78: resolved Issue 1535: writeSensors (extension)
    - description:  add global flag to deactivate sensor file creating/writing and sensor storing; set flag in solver functions such as ComputeLinearizedSystem, etc. to avoid erasing sensor files or sensor data
    - date resolved: **2023-04-27 11:34**\ , date raised: 2023-04-26 
 * Version 1.6.77: resolved Issue 1533: FinalizeSolver (fix)
    - description:  call FinalizeSolver in ComputeLinearizedSystem, ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues for consistency reason; also deactivate file writing and sensor writing and solverInformation writing
    - date resolved: **2023-04-26 19:25**\ , date raised: 2023-04-26 
 * Version 1.6.76: resolved Issue 1532: CSolverBase.cpp (fix)
    - description:  close output and sensor files in destructor of CSolverBase
    - date resolved: **2023-04-26 19:24**\ , date raised: 2023-04-26 
 * Version 1.6.75: resolved Issue 1534: EXUlie (change)
    - description:  improve TExpSE3 and TExpSE3Inv regarding small values according to PhD thesis of Stefan Hante
    - **notes:** improves numerical behavior and convergence of GeometricallyExactBeam
    - date resolved: **2023-04-26 18:33**\ , date raised: 2023-04-26 
 * Version 1.6.74: resolved Issue 1531: OpenVR (fix)
    - description:  change order of eye transformation and projection in OpenVRinterface GetCurrentViewProjectionMatrix to be consistent with master thesis
    - date resolved: **2023-04-26 16:49**\ , date raised: 2023-04-26 
 * Version 1.6.73: resolved Issue 1530: ComputeSystemDegreeOfFreedom (extension)
    - description:  add exudyn.solver function to numerically compute DOF of constrained mechanisms
    - date resolved: **2023-04-26 11:16**\ , date raised: 2023-04-26 
 * Version 1.6.72: resolved Issue 1528: structures and settings (docu)
    - description:  function arguments in RST / html are inappropriately noted; fix; replace true/false with True/False
    - date resolved: **2023-04-26 09:16**\ , date raised: 2023-04-26 
 * Version 1.6.71: resolved Issue 1527: GeneralContact (extension)
    - description:  add function UpdateContacts, which computes current bounding boxes and active contacts, to be used in access functions to GeneralContact if isActive=False (otherwise this is anyway done in contact computations of every computation step)
    - date resolved: **2023-04-23 20:15**\ , date raised: 2023-04-23 
 * Version 1.6.70: resolved Issue 0935: GeneralContact (extension)
    - description:  add interface function to get contact pairs
    - **notes:** function GetActiveContacts added to GeneralContact, which returns all active global contact indices for selected contact type
    - date resolved: **2023-04-23 20:13**\ , date raised: 2022-02-10 
 * Version 1.6.69: resolved Issue 1516: solver failed function (extension)
    - description:  add function to check if solver failed, using stored solver structure as input; return True/False and string (optionally error code) describing failure
    - **notes:** added function SolverSuccess() to exudyn.solver; returns success and error string as created by solver internal function GetErrorString(...)
    - date resolved: **2023-04-23 18:43**\ , date raised: 2023-04-19 
 * Version 1.6.68: resolved Issue 1526: solver GetErrorString (extension)
    - description:  MainSolverStatic, MainSolverExplicit, MainSolverImplicit now get function GetErrorString to obtain error string set if SolveSteps or SolveSystem failed (returned false)
    - date resolved: **2023-04-23 11:46**\ , date raised: 2023-04-23 
 * Version 1.6.67: resolved Issue 1525: output.finishedSuccessfully (extension)
    - description:  flag is now set both in SolveSteps(...) as well in SolveSystem(...) to indicate if solver has been successful or failed; practical flag for lateron determination of solver errors
    - date resolved: **2023-04-23 11:46**\ , date raised: 2023-04-23 
 * Version 1.6.66: resolved Issue 1524: netgen STL file (examples)
    - description:  add example with netgen and STL files with meshing
    - date resolved: **2023-04-21 17:51**\ , date raised: 2023-04-21 
 * Version 1.6.65: resolved Issue 1521: ObjectFFRFreducedOrderInterface (extension)
    - description:  add LoadFromFile/SaveToFile similar to FEMinterface
    - **notes:** this function should be used to store CMS data if FEM is too large to load/store; CMS data still stores all node positions, triangle list (for visualization) and modeBasis for computation tasks; may be still large e.g. for many nodes and large number of modes
    - date resolved: **2023-04-20 21:06**\ , date raised: 2023-04-20 
 * Version 1.6.64: resolved Issue 1519: FEMinterface (change)
    - description:  add version to LoadFromFile/SaveToFile function; store file version as first field to load/store in order to be able to load older data as well; use forceVersion=0 to load files in old format
    - **notes:** warning is printed if old file is loaded
    - date resolved: **2023-04-20 20:59**\ , date raised: 2023-04-19 
 * Version 1.6.63: resolved Issue 1523: StrNodeType2NodeType (extension)
    - description:  add function to rigidBodyUtilities in order to make str to type conversion
    - date resolved: **2023-04-20 18:34**\ , date raised: 2023-04-20 
 * Version 1.6.62: resolved Issue 1522: ObjectFFRFreducedOrderInterface (change)
    - description:  remove femInterface from internal variables, as it is only used for postProcessingModes; store postProcessingModes instead
    - date resolved: **2023-04-20 16:34**\ , date raised: 2023-04-20 
 * Version 1.6.61: resolved Issue 1520: ImportFromAbaqusInputFile (change)
    - description:  use VolumeToSurfaceElements for creating of surface elements; add option to automatically create surface triangles
    - **notes:** by default, only surface triangles are created!
    - date resolved: **2023-04-20 15:44**\ , date raised: 2023-04-20 
 * Version 1.6.60: resolved Issue 1518: ObjectFFRFreducedOrderInterface (fix)
    - description:  roundMassMatrix and roundStiffnessMatrix are not used
    - **notes:** added as arguments in RoundMatrix
    - date resolved: **2023-04-19 19:08**\ , date raised: 2023-04-19 
 * Version 1.6.59: resolved Issue 1517: ImportFromAbaqusInputFile (extension)
    - description:  extended for Tet4 and Tet10 as well as C3D20R elements and added function ConvertTetToTrigs(...)
    - date resolved: **2023-04-19 18:14**\ , date raised: 2023-04-19 
 * Version 1.6.58: resolved Issue 1515: unused header files (cleanup)
    - description:  remove unused header files for C, Main and Visu: JointPrismatic.h, JointRevolute.h
    - date resolved: **2023-04-16 13:03**\ , date raised: 2023-04-16 
 * Version 1.6.57: resolved Issue 1269: LaserSensor (extension)
    - description:  add advanced distance sensors replicating laser scanner (with axis, revolution speed and initial direction)
    - **notes:** added function AddLidar(...) into exudyn.robotics.utilities; see laserScannerTest.py
    - date resolved: **2023-04-15 15:33**\ , date raised: 2022-09-21 
 * Version 1.6.56: resolved Issue 1270: DistanceSensor (extension)
    - description:  add advanced distance sensor with possibility to use a set of beams and averaging or multiple output
    - **notes:** added DistanceSensorSetupGeometry in exudyn.utilities to set up contact geometry easily; see laserScannerTest.py
    - date resolved: **2023-04-15 15:32**\ , date raised: 2022-09-21 
 * Version 1.6.55: :textred:`resolved BUG 1514` : GetInterpolatedSignalValue 
    - description:  check for simple test, seems not to work
    - **notes:** changed order of args: dataArrayIndex and timeArrayIndex
    - date resolved: **2023-04-15 14:34**\ , date raised: 2023-04-14 
 * Version 1.6.54: resolved Issue 1513: RevoluteJoint, PrismaticJoint (fix)
    - description:  both ObjectJointPrismaticX and ObjectJointRevoluteZ have wrong internal typename JointRevolute; change to correct names; when analyzing such objects, they will return wrong typenames
    - date resolved: **2023-04-14 14:36**\ , date raised: 2023-04-14 
 * Version 1.6.53: resolved Issue 1511: AddDistanceSensor (extension)
    - description:  add optional color for laser beam
    - date resolved: **2023-04-13 17:45**\ , date raised: 2023-04-13 
 * Version 1.6.52: :textred:`resolved BUG 1510` : AddDistanceSensor 
    - description:  checks performed with invalid marker number
    - date resolved: **2023-04-13 10:28**\ , date raised: 2023-04-13 
 * Version 1.6.51: resolved Issue 1141: object factory (extension)
    - description:  consider hash tables for object factory; check current performance?
    - **notes:** no changes; string comparison has no effect on performance as compared to new and pybind11 overheads for call to  AddObject, etc.
    - date resolved: **2023-04-10 21:48**\ , date raised: 2022-06-12 
 * Version 1.6.50: resolved Issue 1506: ODE2Size (fix)
    - description:  and other functions have additional ConfigurationType: remove
    - **notes:** kept the argument ConfigurationType; usually, all configuration types have same sizes but the call must be related to a speficic configuration
    - date resolved: **2023-04-10 21:42**\ , date raised: 2023-04-08 
 * Version 1.6.49: resolved Issue 1509: SystemData (fix)
    - description:  prevent from creation of a pure SystemData in exudyn; check constructor used in SC.AddSystem and SystemData()
    - date resolved: **2023-04-10 21:37**\ , date raised: 2023-04-10 
 * Version 1.6.48: resolved Issue 1508: MainSystem (fix)
    - description:  prevent from creation of a pure MainSystem in exudyn; check constructor used in SC.AddSystem and MainSystem()
    - date resolved: **2023-04-10 21:37**\ , date raised: 2023-04-10 
 * Version 1.6.47: resolved Issue 1507: SystemContainer (docu)
    - description:  add description for SystemContainer itself; sying that it behaves like a variable in Python
    - date resolved: **2023-04-10 21:36**\ , date raised: 2023-04-10 
 * Version 1.6.46: resolved Issue 1503: item description (docu)
    - description:  add general description to RST / sphinx
    - date resolved: **2023-04-08 19:04**\ , date raised: 2023-04-08 
 * Version 1.6.45: resolved Issue 1496: AddMainObjectPyClass (check)
    - description:  C++: for adding objects, nodes, etc. currently py::object and py::dict is copied: check performance increase, if it is passed by reference
    - **notes:** tests show small performance improvements (<10 percent) for creation of items
    - date resolved: **2023-04-08 17:36**\ , date raised: 2023-04-07 
 * Version 1.6.44: resolved Issue 1498: MarkerNodeRotationCoordinate (check)
    - description:  check if Orientation type is correct, see docu
    - **notes:** removed Marker::Orientation from MarkerNodeRotationCoordinate as it is not provided
    - date resolved: **2023-04-08 17:30**\ , date raised: 2023-04-08 
 * Version 1.6.43: :textred:`resolved BUG 1497` : PyBeamSection 
    - description:  C++: segmentation fault in linux, caused by conversion from numpy array into std::array; use SetConstMatrixTemplateSafely for writing matrix or list
    - date resolved: **2023-04-07 19:05**\ , date raised: 2023-04-07 
 * Version 1.6.42: :textred:`resolved BUG 1495` : BeamSection 
    - description:  PyBeamSection does not call parent constructor (BeamSection); leads to seg fault in linux version
    - date resolved: **2023-04-07 13:32**\ , date raised: 2023-04-07 
 * Version 1.6.41: resolved Issue 1272: GeometricallyExactBeam (check)
    - description:  check jacobian computation as numerical differentiation works better; similar to #1100
    - **notes:** fixed bug of GeometricallyExactBeam having wrong jacobian
    - date resolved: **2023-04-06 18:28**\ , date raised: 2022-09-24 
 * Version 1.6.40: resolved Issue 1492: Python utilities (docu)
    - description:  fix indentation of examples
    - date resolved: **2023-04-06 14:55**\ , date raised: 2023-04-06 
 * Version 1.6.39: resolved Issue 1491: ProcessParameterList (fix)
    - description:  remove addComputationIndex, as it is unused
    - date resolved: **2023-04-06 14:35**\ , date raised: 2023-04-06 
 * Version 1.6.38: resolved Issue 1156: renderState (docu)
    - description:  add description of render state from C++ into theDoc
    - **notes:** added section Render State in section Graphics and Visualization (GUI)
    - date resolved: **2023-04-05 14:14**\ , date raised: 2022-06-21 
 * Version 1.6.37: resolved Issue 1468: sphinx (docu)
    - description:  add issue and bugs section with resolved issues and open issues table; resolved issues with version
    - **notes:** already done earlier
    - date resolved: **2023-04-05 14:08**\ , date raised: 2023-03-22 
 * Version 1.6.36: resolved Issue 1490: ANCFBeam (change)
    - description:  remove testBeamRectangularSize
    - date resolved: **2023-04-05 13:59**\ , date raised: 2023-04-05 
 * Version 1.6.35: resolved Issue 1088: ANCFBeam3D (extension)
    - description:  add test model, compare to 2013 paper
    - date resolved: **2023-04-04 20:59**\ , date raised: 2022-05-16 
 * Version 1.6.34: :textred:`resolved BUG 1489` : CObjectANCFBeam 
    - description:  Computation of elastic forces uses global instead of local twist-and-curvature vector
    - date resolved: **2023-04-04 20:04**\ , date raised: 2023-04-04 
 * Version 1.6.33: resolved Issue 1488: NodePoint3DSlope23 (extension)
    - description:  add full rigid body output values to node (e.g., Rotation missing)
    - date resolved: **2023-04-04 13:31**\ , date raised: 2023-04-04 
 * Version 1.6.32: resolved Issue 1487: Acceleration (extension)
    - description:  add GetAcceleration function to all nodes; add acceleration to all ODE2-based node output variables
    - date resolved: **2023-04-04 13:27**\ , date raised: 2023-04-04 
 * Version 1.6.31: resolved Issue 1482: EnterTaskManager (change)
    - description:  removed call to EnterTaskManager in serial mode as it causes large overhead; also removed large array for tracer
    - date resolved: **2023-03-28 18:47**\ , date raised: 2023-03-28 
 * Version 1.6.30: resolved Issue 1481: InverseKinematicsNumerical (change)
    - description:  changed SolveIKine to Solve and changed OutputVariable Rotation to RotationMatrix at tool
    - date resolved: **2023-03-28 16:30**\ , date raised: 2023-03-28 
 * Version 1.6.29: resolved Issue 1480: LogSE3 (extension)
    - description:  fixed according to LogSO3 and added efficient version with vectors in C++
    - date resolved: **2023-03-28 15:10**\ , date raised: 2023-03-28 
 * Version 1.6.28: resolved Issue 1478: LogSO3 (fix)
    - description:  Add new LogSO3 C++ function, also to be used in LogSE3 which fully works for 0..pi rotation range; improved accuracy for very small rotations as well as rotations close to pi, e.g. 0.99999999\*pi, where the standard approach fails
    - date resolved: **2023-03-28 15:09**\ , date raised: 2023-03-27 
 * Version 1.6.27: resolved Issue 1469: RotationMatrix2Rxyz (fix)
    - description:  extend implementation for rot[1]=pi/2
    - **notes:** resolved both in Python and C++; some simulation results may change, especially in output of Rotations in sensors
    - date resolved: **2023-03-28 14:46**\ , date raised: 2023-03-22 
 * Version 1.6.26: resolved Issue 1477: LogSO3 (fix)
    - description:  Add new LogSO3 Python function, also to be used in LogSE3 which fully works for 0..pi rotation range; improved accuracy for very small rotations as well as rotations close to pi, e.g. 0.99999999\*pi, where the standard approach fails
    - date resolved: **2023-03-27 20:16**\ , date raised: 2023-03-27 
 * Version 1.6.25: resolved Issue 1476: RotationMatrix2... (fix)
    - description:  RotationMatrix2EulerParameters, RotationMatrix2RotXYZ, RotationMatrix2RotZYZ not working both with list of lists and np.arrays
    - date resolved: **2023-03-27 19:58**\ , date raised: 2023-03-27 
 * Version 1.6.23: resolved Issue 0312: Add all types to pybind (extension)
    - description:  add remaining types to pybind - for user elements
    - **notes:** important types already added; further types currently not planned
    - date resolved: **2023-03-27 00:45**\ , date raised: 2020-01-10 
 * Version 1.6.22: resolved Issue 0848: add demos to github (docu)
    - description:  add demo videos to github or to youtube
    - **notes:** already resolved earlier: added videos to youtube
    - date resolved: **2023-03-27 00:42**\ , date raised: 2021-12-26 
 * Version 1.6.21: resolved Issue 0843: connector jacobian springDamper (extension)
    - description:  add analytic jacobian for SpringDamper connector
    - **notes:** already resolved earlier
    - date resolved: **2023-03-27 00:42**\ , date raised: 2021-12-23 
 * Version 1.6.20: resolved Issue 0851: ComputeObjectODE2LHS (extension)
    - description:  add computation functions for bodies and connectors; for connectors, markerData is computed automatically
    - **notes:** already resolved earlier with precomputed lists
    - date resolved: **2023-03-27 00:40**\ , date raised: 2022-01-08 
 * Version 1.6.19: resolved Issue 1325: MergeGraphicsDataTriangleList (fix)
    - description:  merging of edges erroneous or problems with GraphicsDataCylinder
    - date resolved: **2023-03-27 00:21**\ , date raised: 2022-12-19 
 * Version 1.6.18: resolved Issue 1472: GraphicsData (extension)
    - description:  add addEdges option to all GraphicsData Python functions
    - **notes:** added to sphere, cylinder, SolidOfRevolution, SolidOfExtrusion
    - date resolved: **2023-03-27 00:10**\ , date raised: 2023-03-26 
 * Version 1.6.17: resolved Issue 1473: GraphicsData (change)
    - description:  improve edges of cylinder and sphere by adding 6 lines along cylinder, some circles for sphere
    - date resolved: **2023-03-27 00:09**\ , date raised: 2023-03-26 
 * Version 1.6.16: resolved Issue 1459: mention papers (docu)
    - description:  add list of papers where exudyn has been used in theDoc and RTD
    - date resolved: **2023-03-26 15:04**\ , date raised: 2023-03-10 
 * Version 1.6.15: resolved Issue 1471: issues tracker (docu)
    - description:  remove html file from docs, use RSTfiles instead / move to readthedocs
    - date resolved: **2023-03-24 20:40**\ , date raised: 2023-03-24 
 * Version 1.6.14: resolved Issue 1470: GraphicsDataBasis (extension)
    - description:  add orientation and duplicate function with homogeneous transformation (HT) argument
    - **notes:** function called GraphicsDataFrame for Homogeneous transformation
    - date resolved: **2023-03-24 10:04**\ , date raised: 2023-03-24 
 * Version 1.6.13: resolved Issue 1467: InverseKinematicsNumerical (fix)
    - description:  fix success
    - date resolved: **2023-03-22 12:06**\ , date raised: 2023-03-22 
    - resolved by: P. Manzl
 * Version 1.6.12: resolved Issue 1466: modKKDH (change)
    - description:  consistently renamed into modDHKK in robotics module
    - date resolved: **2023-03-22 11:04**\ , date raised: 2023-03-22 
 * Version 1.6.11: resolved Issue 1465: robotics.models (change)
    - description:  adjust robot definitions, add dhMode to dictionary; fix LinkList2Robot
    - date resolved: **2023-03-22 10:39**\ , date raised: 2023-03-22 
 * Version 1.6.10: resolved Issue 1464: artificialIntelligence (fix)
    - description:  adapt to stable-baselines3 > 1.5.0; Class OpenAIGymInterfaceEnv does not support stable-baselines3 > 1.5.0 because OpenAIGymInterfaceEnv is not inherited from gym.Env
    - date resolved: **2023-03-20 12:54**\ , date raised: 2023-03-20 
    - resolved by: P. Manzl
 * Version 1.6.9: resolved Issue 1463: mpi4py (extension)
    - description:  add option to use MPI (message passing interface) for ProcessParameterList
    - **notes:** tests on supercomputer successful
    - date resolved: **2023-03-18 22:59**\ , date raised: 2023-03-18 
 * Version 1.6.8: resolved Issue 1462: minimum coordinates (docu)
    - description:  change consistently to minimal coordinates
    - date resolved: **2023-03-14 17:39**\ , date raised: 2023-03-14 
 * Version 1.6.7: :textred:`resolved BUG 1461` : isIntType check ParameterVariation 
    - description:  In the ParameterVariation (part of the processing module)  the integer type check failed when Variables of different types were used, now checking each variable independently
    - **notes:** added isIntType arrays
    - date resolved: **2023-03-14 12:50**\ , date raised: 2023-03-14 
    - resolved by: P. Manzl
 * Version 1.6.6: resolved Issue 1457: Newton (docu)
    - description:  fix steps and Newton iterations in description of Newton settings (thanks to Martin Arnold!)
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-03-09 
 * Version 1.6.5: resolved Issue 1456: sphinx/github pages (docu)
    - description:  add examples and test models for better search results
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-03-09 
 * Version 1.6.4: resolved Issue 1445: sphinx/github pages (docu)
    - description:  add solver description
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-02-22 
 * Version 1.6.3: resolved Issue 1443: sphinx/github pages (docu)
    - description:  add theory parts as far as possible
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-02-22 
 * Version 1.6.2: resolved Issue 1460: theDoc (docu)
    - description:  change structure: theory, notations earlier; remove duplicated MatrixContainer description
    - date resolved: **2023-03-12 16:20**\ , date raised: 2023-03-12 
 * Version 1.6.1: resolved Issue 1458: artificialIntelligence nan test (extension)
    - description:  check for nan values in TestModel evaluation
    - **notes:** failed steps may lead to nan values when evaluating the TestModel in the artificialIntelligence module. When this is detected the flagNan is set and evaluation is stopped. Check for flagNan in the Evaluation should be implemented.
    - date resolved: **2023-03-10 15:59**\ , date raised: 2023-03-10 
    - resolved by: P. Manzl
 * Version 1.6.0: resolved Issue 1426: cleanup howto files (fix)
    - description:  check if info is still valid (NGsolve, WSL, anaconda, ...)
    - **notes:** removed outdated files
    - date resolved: **2023-03-08 16:58**\ , date raised: 2023-02-11 

***********
Version 1.5
***********

 * Version 1.5.118: resolved Issue 1454: julia (docu)
    - description:  add sub section on interoperability with julia in Overview on Exudyn / Advanced topics; add some relevant examples for usage
    - date resolved: **2023-03-05 16:40**\ , date raised: 2023-03-05 
 * Version 1.5.117: resolved Issue 1453: __NOGLFW option not working (fix)
    - description:  exclude according functions in rendererPythonInterface
    - **notes:** added NOGLFW version to automatic build and testsuite
    - date resolved: **2023-02-28 15:18**\ , date raised: 2023-02-28 
 * Version 1.5.116: resolved Issue 1452: Single command (docu)
    - description:  add some more detailed description in theDoc after Visualization settings dialog
    - date resolved: **2023-02-26 00:40**\ , date raised: 2023-02-26 
 * Version 1.5.115: resolved Issue 1451: solver interface (check)
    - description:  check modification of solver structures like it to be writable
    - **notes:** added now write functionality for solvers; use with care and only if you know what you do...
    - date resolved: **2023-02-26 00:12**\ , date raised: 2023-02-25 
 * Version 1.5.114: resolved Issue 1447: CSensorMarker (extension)
    - description:  add RotationMatrix as additional outputvariable type
    - **notes:** Coordinates also added as output variable
    - date resolved: **2023-02-24 15:46**\ , date raised: 2023-02-23 
 * Version 1.5.113: resolved Issue 1446: CSensorMarker (fix)
    - description:  does not check types; Make GetOutputVariableTypes as in objects; orientation only for selected markers; check types in Assemble prechecks
    - date resolved: **2023-02-24 15:46**\ , date raised: 2023-02-23 
 * Version 1.5.112: :textred:`resolved BUG 1449` : DOPRI5 
    - description:  small bug introduced in previous update: currentStepSize not updated any more
    - date resolved: **2023-02-24 15:45**\ , date raised: 2023-02-24 
 * Version 1.5.111: resolved Issue 1448: GenericJoint (extension)
    - description:  add experimental flag alternativeConstraints to switch to different constraint equations, e.g. for 3 constrained rotations; this resolves unphysical 180 degree flips especially in static cases; flag may be removed in future
    - **notes:** thanks for P. Manzl for raising this problem
    - date resolved: **2023-02-24 13:52**\ , date raised: 2023-02-24 
 * Version 1.5.110: resolved Issue 1444: sphinx/github pages (docu)
    - description:  fix equation references and figures
    - date resolved: **2023-02-24 13:50**\ , date raised: 2023-02-22 
 * Version 1.5.109: resolved Issue 1442: add readthedocs.io (docu)
    - description:  link github project to readthedocs.io site
    - date resolved: **2023-02-22 23:59**\ , date raised: 2023-02-22 
 * Version 1.5.108: resolved Issue 1441: sphinx/github pages (docu)
    - description:  add more parts on items (equations)
    - date resolved: **2023-02-22 23:57**\ , date raised: 2023-02-22 
 * Version 1.5.107: resolved Issue 1440: ParameterVariation (change)
    - description:  integer should be kept as integers, if they are provided as a list and if variation is only on integers
    - **notes:** integers are kept if either tuple(start, end, numberOfValues) with start/end and numberOfValues are all integer or list contains only integers
    - date resolved: **2023-02-22 13:46**\ , date raised: 2023-02-22 
    - resolved by: S. Holzinger
 * Version 1.5.106: resolved Issue 1439: numpy dependency (extension)
    - description:  add install_requires to setup.py to force numpy to be installed; exudynCPP always requires now numpy
    - date resolved: **2023-02-21 18:11**\ , date raised: 2023-02-21 
 * Version 1.5.105: resolved Issue 1438: sphinx / github pages (docu)
    - description:  add input/output tables for items in RST files
    - date resolved: **2023-02-21 10:14**\ , date raised: 2023-02-19 
 * Version 1.5.104: resolved Issue 1437: sphinx / github pages (docu)
    - description:  add items to RST files/Sphinx to show up on github pages
    - date resolved: **2023-02-19 21:39**\ , date raised: 2023-02-19 
 * Version 1.5.103: resolved Issue 1436: sphinx / github pages (docu)
    - description:  add settings and structures; fix latex parts; fix links
    - date resolved: **2023-02-18 00:56**\ , date raised: 2023-02-18 
 * Version 1.5.102: :textred:`resolved BUG 1433` : Lie group integration 
    - description:  system with > 1 node raises LinkedDataVectorBases exception
    - date resolved: **2023-02-16 16:00**\ , date raised: 2023-02-16 
 * Version 1.5.101: resolved Issue 1432: artificialIntelligence module (fix)
    - description:  fix np.float64 not beeing detected as a scalar (float) for initializationValues of RL environment and possible waiting without activated renderer
    - date resolved: **2023-02-16 15:58**\ , date raised: 2023-02-16 
    - resolved by: P. Manzl
 * Version 1.5.100: resolved Issue 1431: make github pages (extension)
    - description:  install workflow for github pages to host pages for exudyn
    - **notes:** due to automatic conversion, there are many small errors (typos) remaining; for safety, check theDoc.pdf in any case
    - date resolved: **2023-02-16 00:01**\ , date raised: 2023-02-16 
 * Version 1.5.99: resolved Issue 1430: create rst docu files (extension)
    - description:  Create rst (markup language) files for C++ command interface and Python utilities
    - date resolved: **2023-02-16 00:00**\ , date raised: 2023-02-15 
 * Version 1.5.98: resolved Issue 1429: revise docu creation (fix)
    - description:  revise several helper functions for docu creation to homogenize latex and rst output
    - date resolved: **2023-02-16 00:00**\ , date raised: 2023-02-15 
 * Version 1.5.97: resolved Issue 1428: ParameterVariation (change)
    - description:  add conversion of parameters which are numpy.float64 to float which simplifies checking agains type(float); computationIndex becomes int
    - date resolved: **2023-02-13 17:18**\ , date raised: 2023-02-13 
 * Version 1.5.96: resolved Issue 1427: IsReal(x) and IsInteger(x) (extension)
    - description:  add checks which allow to check versus any float / numpy.float resp. int / numpy.int values; added to advancedUtilities
    - date resolved: **2023-02-13 17:17**\ , date raised: 2023-02-13 
 * Version 1.5.95: resolved Issue 1425: sphinx (extension)
    - description:  add github pages at https://jgerstmayr.github.io/EXUDYN created with sphinx
    - date resolved: **2023-02-11 16:41**\ , date raised: 2023-02-11 
 * Version 1.5.94: resolved Issue 1423: numerical Jacobians (change)
    - description:  add variadic template to realize numerical differentiation for objects in a consistent way
    - date resolved: **2023-02-08 12:17**\ , date raised: 2023-02-08 
 * Version 1.5.93: resolved Issue 1404: Lie group method (fix)
    - description:  add consistent numerical derivatives for Lie group nodes, needing the composition rule for incremental changes
    - date resolved: **2023-02-08 12:17**\ , date raised: 2023-01-18 
 * Version 1.5.92: resolved Issue 1422: numerical Jacobians (change)
    - description:  add variadic template to realize numerical differentiation in a consistent way
    - date resolved: **2023-02-08 00:16**\ , date raised: 2023-02-07 
 * Version 1.5.91: resolved Issue 1421: LIE_GROUP_IMPLICIT_SOLVER (change)
    - description:  remove preprocessor flag, as it is always set
    - date resolved: **2023-02-07 12:07**\ , date raised: 2023-02-07 
 * Version 1.5.90: resolved Issue 1420: setup.py (extension)
    - description:  reduce output of all compiler options with --quiet mode
    - date resolved: **2023-02-02 18:27**\ , date raised: 2023-02-02 
 * Version 1.5.89: resolved Issue 1419: window size (extension)
    - description:  window size currently limited to screen size and larger windows not accepted; override size limitations by using glfwSetWindowSize
    - **notes:** added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated
    - date resolved: **2023-02-02 10:31**\ , date raised: 2023-02-02 
 * Version 1.5.88: resolved Issue 1418: OpenVR (change)
    - description:  adapt projection for OpenVR compatibility; exchange multiplication of pose and eye to fulfill classic OpenGL needs
    - date resolved: **2023-02-01 10:30**\ , date raised: 2023-02-01 
 * Version 1.5.87: resolved Issue 1417: ConnectorSpringDamperExt (fix)
    - description:  remove unintended output during Assemble()
    - date resolved: **2023-01-25 20:09**\ , date raised: 2023-01-25 
 * Version 1.5.86: resolved Issue 1405: AddDistanceSensor (extension)
    - description:  add rotation of Marker to ObjectGround in SensorUserFunction
    - **notes:** added option to draw displaced laser beam; rotation added, if rigid marker used
    - date resolved: **2023-01-23 10:05**\ , date raised: 2023-01-19 
 * Version 1.5.85: resolved Issue 1416: AddDistanceSensor (fix)
    - description:  UFsensorDistance misses the rotation part in visualization of laser beam with ground object
    - date resolved: **2023-01-23 09:39**\ , date raised: 2023-01-23 
 * Version 1.5.84: resolved Issue 1413: CoordinateSpringDamperExt (change)
    - description:  change friction parameter dimensions to forces (because there is no normal force) and change parameter names
    - date resolved: **2023-01-22 23:43**\ , date raised: 2023-01-21 
 * Version 1.5.83: resolved Issue 1412: CoordinateSpringDamperExt (extension)
    - description:  finalize implementation for bristle friction model
    - date resolved: **2023-01-22 23:43**\ , date raised: 2023-01-21 
 * Version 1.5.82: resolved Issue 1411: CoordinateSpringDamperExt (extension)
    - description:  finalize implementation for limit stops
    - date resolved: **2023-01-22 23:43**\ , date raised: 2023-01-21 
 * Version 1.5.81: resolved Issue 1410: examples (change)
    - description:  adapt Examples/massSpringFrictionInteractive.py and Examples/lugreFrictionTest.py to new CoordinateSpringDamperExt
    - date resolved: **2023-01-21 22:04**\ , date raised: 2023-01-21 
 * Version 1.5.80: resolved Issue 1136: ConnectorsExt (extension)
    - description:  add extended Ext versions of connectors: CoordinateSpringDamperExt, TorsionalSpringDamperExt, LinearSpringDamperExt, which allow for friction, coordinate limitation (with other SD-values) and possibly future extensions
    - date resolved: **2023-01-21 21:29**\ , date raised: 2022-06-10 
 * Version 1.5.79: resolved Issue 1407: pause (extension)
    - description:  add option for pause by pressing space bar
    - date resolved: **2023-01-21 21:28**\ , date raised: 2023-01-21 
 * Version 1.5.78: resolved Issue 1409: CoordinateSpringDamper (change)
    - description:  remove dryFriction and dryFrictionProportionalZone; this functionality will be made available in CoordinateSpringDamperExt
    - **notes:** see CoordinateSpringDamper in theDoc.pdf how to convert old models using friction parameters to new ones; note user function interfaces have been changed!
    - date resolved: **2023-01-21 17:53**\ , date raised: 2023-01-21 
 * Version 1.5.77: resolved Issue 1408: WaitForUserToContinue (extension)
    - description:  add argument printMessage, which can be set false to avoid text output in console; default behavior preserved
    - date resolved: **2023-01-21 13:51**\ , date raised: 2023-01-21 
 * Version 1.5.76: resolved Issue 1406: git tags (extension)
    - description:  add git tags for every new version automatically
    - **notes:** now versions can be found easier in github and they match the version numbers in pypi with pip installer
    - date resolved: **2023-01-19 18:47**\ , date raised: 2023-01-19 
 * Version 1.5.75: resolved Issue 1314: Pybind11 (check)
    - description:  test compilation with higher version of pybind11 (currently pybind11=2.6.0 in setup.py
    - **notes:** already done earlier; works well and allows compilation with Python3.11; added some switches in setup.py as not all Pybind11 versions work everywhere
    - date resolved: **2023-01-19 01:04**\ , date raised: 2022-12-14 
 * Version 1.5.74: resolved Issue 1365: OpenVR (example)
    - description:  add Python example with openVR
    - date resolved: **2023-01-19 01:02**\ , date raised: 2023-01-03 
 * Version 1.5.73: resolved Issue 1326: OpenVR (change)
    - description:  add OpenVR license and mention in getting started
    - date resolved: **2023-01-19 01:02**\ , date raised: 2022-12-20 
 * Version 1.5.72: resolved Issue 1402: openVR (extension)
    - description:  add flag to set compilation with openVR in setup.py; copy .dll in Windows case
    - date resolved: **2023-01-18 22:57**\ , date raised: 2023-01-17 
 * Version 1.5.71: resolved Issue 1364: OpenVR (extension)
    - description:  test simple use case
    - **notes:** basic functionality added; enable compilation with openVR by setting '-D__EXUDYN_USE_OPENVR' in cpp compiler flags of setup.py
    - date resolved: **2023-01-17 09:32**\ , date raised: 2023-01-03 
    - resolved by: EXTENSION
 * Version 1.5.70: resolved Issue 1401: lockModelView (extension)
    - description:  add flag which allows to fully lock rotation, zoom, etc.; in this case, only initial values in openGL settings are accepted for setup of view, but mouse or key input is ignored
    - date resolved: **2023-01-17 09:20**\ , date raised: 2023-01-17 
 * Version 1.5.69: resolved Issue 1363: test OpenVR with basic OpenGL (check)
    - description:  check if current openGL is sufficient with textures
    - date resolved: **2023-01-16 23:11**\ , date raised: 2023-01-03 
 * Version 1.5.68: resolved Issue 1400: renderState (change)
    - description:  vectors and matrices are returned in numpy format; this allows simpler computation, but does not anymore allow to treat output as list (+ operator!)
    - date resolved: **2023-01-16 21:11**\ , date raised: 2023-01-16 
 * Version 1.5.67: resolved Issue 1399: renderState (change)
    - description:  initialization now done consistently for SC.AttachToRenderEngine() and exudyn.StartRenderer(); behavior should be as before
    - date resolved: **2023-01-16 21:10**\ , date raised: 2023-01-16 
 * Version 1.5.66: resolved Issue 1398: renderState (extension)
    - description:  add projectionMatrix containing the current projection usd (usually identity matrix)
    - date resolved: **2023-01-16 20:09**\ , date raised: 2023-01-16 
 * Version 1.5.65: resolved Issue 1397: showFaceEdges, showMeshEdges (fix)
    - description:  wrong switching causing to show edges only if also some faces are activated
    - date resolved: **2023-01-12 22:32**\ , date raised: 2023-01-12 
 * Version 1.5.64: resolved Issue 1387: Get...Output (extension)
    - description:  add way to work for Reference configuration
    - **notes:** nodes, bodies and markers allow reference configuration; sensors also allow it for GetSensorValues
    - date resolved: **2023-01-12 22:14**\ , date raised: 2023-01-12 
 * Version 1.5.63: resolved Issue 1389: configuration checks (extension)
    - description:  check all systemData C++ interface functions regarding illegal configuration
    - date resolved: **2023-01-12 22:03**\ , date raised: 2023-01-12 
 * Version 1.5.62: resolved Issue 1388: configuration checks (extension)
    - description:  IsConfigurationInitialCurrentReferenceVisualization and IsConfigurationInitialCurrentVisualization shall be extended for StartOfStep; add hint that a calling function may have used an illegal configuration or None
    - date resolved: **2023-01-12 22:03**\ , date raised: 2023-01-12 
 * Version 1.5.61: resolved Issue 1396: Demo (extnesion)
    - description:  add a two demos included into the python module; put into exudyn.demos; add hint for larger examples; Demo1() = without graphics, just creating output file; Demo2() is rigid3Dexample with SolutionViewer
    - date resolved: **2023-01-12 18:51**\ , date raised: 2023-01-12 
 * Version 1.5.60: resolved Issue 1385: help (extension)
    - description:  add exudyn.help() function for short notes
    - date resolved: **2023-01-12 18:04**\ , date raised: 2023-01-12 
 * Version 1.5.59: :textred:`resolved BUG 1390` : ComputeLinearizedSystem 
    - description:  not working because of wrong interface to ComputeJacobianODE2RHS
    - date resolved: **2023-01-12 17:53**\ , date raised: 2023-01-12 
 * Version 1.5.58: resolved Issue 1394: eigenvalues test (testing)
    - description:  refine test model for ComputeODE2Eigenvalues
    - date resolved: **2023-01-12 17:44**\ , date raised: 2023-01-12 
 * Version 1.5.57: resolved Issue 1393: ComputeJacobianAE (change)
    - description:  change default values to fit to conventional tangential stiffness matrix computation
    - date resolved: **2023-01-12 17:00**\ , date raised: 2023-01-12 
 * Version 1.5.56: resolved Issue 1392: ComputeJacobianODE1RHS (change)
    - description:  change default values to fit to conventional tangential stiffness matrix computation
    - date resolved: **2023-01-12 17:00**\ , date raised: 2023-01-12 
 * Version 1.5.55: resolved Issue 1391: ComputeJacobianODE2RHS (change)
    - description:  change default values to fit to conventional tangential stiffness matrix computation
    - date resolved: **2023-01-12 17:00**\ , date raised: 2023-01-12 
 * Version 1.5.54: :textred:`resolved BUG 1386` : GetMarkerOutput 
    - description:  crashes if used with any configuration before Assemble(); add check that it may be only called after Assemble()
    - **notes:** add check and raise error
    - date resolved: **2023-01-12 14:15**\ , date raised: 2023-01-12 
 * Version 1.5.53: resolved Issue 1384: visualizationSettings (extension)
    - description:  add textOffsetFactor in general options to adjust text offset if not drawn always in front
    - **notes:** also adjusted some text appearance settings
    - date resolved: **2023-01-11 20:42**\ , date raised: 2023-01-11 
 * Version 1.5.52: resolved Issue 1383: visualizationSettings (docu)
    - description:  update docu of Visualization settings dialog in introduction
    - date resolved: **2023-01-11 17:41**\ , date raised: 2023-01-11 
 * Version 1.5.51: resolved Issue 1382: visualizationSettings (change)
    - description:  increase initial size of visualizations dialog for larger screens; see  Section :ref:`sec-overview-basics-visualizationsettings`\  how to change to a smaller window size
    - date resolved: **2023-01-11 16:57**\ , date raised: 2023-01-11 
 * Version 1.5.50: resolved Issue 1381: visualizationSettings (change)
    - description:  change order of items to have easier access to each tree node
    - **notes:** dialogs.openTreeView=True can be used to switch to original behavior
    - date resolved: **2023-01-11 16:34**\ , date raised: 2023-01-11 
 * Version 1.5.49: resolved Issue 1379: item text color (change)
    - description:  draw colors in item texts (node numbers, etc.) different from item color to improve visibility
    - date resolved: **2023-01-11 16:28**\ , date raised: 2023-01-11 
 * Version 1.5.48: resolved Issue 1308: OpenGL texts (extension)
    - description:  add sub function for drawing text bitmaps; use different order of drawing for transparent and non-transparent triangles to get improved text drawing
    - **notes:** now having possibility to draw item texts in front or transparent
    - date resolved: **2023-01-11 16:28**\ , date raised: 2022-12-08 
 * Version 1.5.47: resolved Issue 1380: loads (change)
    - description:  draw load numbers at end of load vectors
    - **notes:** made Force consistent with Torque and LoadMassProportional
    - date resolved: **2023-01-11 14:43**\ , date raised: 2023-01-11 
 * Version 1.5.46: resolved Issue 1378: font drawing (extension)
    - description:  add visualizationSettings.general for textDrawInFront and textHasBackGround for having a (currently) white background color
    - date resolved: **2023-01-11 09:46**\ , date raised: 2023-01-11 
 * Version 1.5.45: resolved Issue 1377: openvr (change)
    - description:  not compatible right now with Exudyn under Windows while linux seems to work
    - date resolved: **2023-01-09 21:11**\ , date raised: 2023-01-09 
 * Version 1.5.44: resolved Issue 1370: advancedUtilities (extension)
    - description:  add advanced utilities depnding on numpy and math; functions not suitable for exudyn.basicUtilities or exudyn.utilities
    - date resolved: **2023-01-07 01:57**\ , date raised: 2023-01-06 
 * Version 1.5.43: resolved Issue 1375: initial accelerations (extension)
    - description:  add flag to decide whether initial accelerations are erased at beginning of simulation; the background is that they should not be erased, if they are prolonged for subsequent simulation; check if this changes behavior, as ODE2_tt coordinates are anyway resetted at Assemble()
    - date resolved: **2023-01-07 01:56**\ , date raised: 2023-01-06 
 * Version 1.5.42: resolved Issue 1376: advancedUtilities (change)
    - description:  move functions exudyn.utilities from to exudyn.advancedUtilities; still included in exudyn.utilities
    - date resolved: **2023-01-06 22:54**\ , date raised: 2023-01-06 
 * Version 1.5.41: :textred:`resolved BUG 1374` : InteractiveDialog 
    - description:  accelerations are not reused from last period; need to copy current accelerations into initial accelerations
    - date resolved: **2023-01-06 21:21**\ , date raised: 2023-01-06 
 * Version 1.5.40: resolved Issue 1373: GraphicsData (change)
    - description:  add option to add edges in GraphicsData...Cube... functions; switch some order of options 
    - date resolved: **2023-01-06 19:25**\ , date raised: 2023-01-06 
 * Version 1.5.39: resolved Issue 1372: utilities (fix)
    - description:  ComputeSkewMatrix: remove duplicate (with less functionality) from utilities and put into rigidBodyUtilities; correct import in FEM
    - date resolved: **2023-01-06 19:19**\ , date raised: 2023-01-06 
 * Version 1.5.38: resolved Issue 1371: exudyn.utilities (change)
    - description:  remove functions CheckInputVector and CheckInputIndexArray as they are unused and replaced with improved functions in advancedUtilities
    - date resolved: **2023-01-06 17:46**\ , date raised: 2023-01-06 
 * Version 1.5.37: resolved Issue 1369: add sub-module machines (extension)
    - description:  will include mechanical engineering and machine element relevant topics, such as bearings, gears, mechanisms, etc.
    - date resolved: **2023-01-06 11:47**\ , date raised: 2023-01-06 
 * Version 1.5.36: resolved Issue 1368: GraphicsDataCylinder (extension)
    - description:  new option to only add edges without faces
    - date resolved: **2023-01-05 23:03**\ , date raised: 2023-01-05 
 * Version 1.5.35: resolved Issue 1367: GraphicsDataSolidExtrusion (extension)
    - description:  add edges and normals for smoothening
    - date resolved: **2023-01-05 23:03**\ , date raised: 2023-01-05 
 * Version 1.5.34: resolved Issue 1366: GraphicsData (extension)
    - description:  add functionality for GraphicsDataSolidExtrusion to include circles: CirclePointsAndSegments() and to manipulate point lists defining geometries: SegmentsFromPoints()
    - date resolved: **2023-01-05 18:15**\ , date raised: 2023-01-05 
 * Version 1.5.33: resolved Issue 1362: add OpenVR interface (extension)
    - description:  add interface, but by default not compiled
    - date resolved: **2023-01-03 23:00**\ , date raised: 2023-01-03 
 * Version 1.5.32: resolved Issue 1361: Python 3.11 (extension)
    - description:  create first development wheels for Python 3.11; Windows+Linux
    - **notes:** problems to get conda with Python3.11 running, especially on ubuntu
    - date resolved: **2023-01-03 18:48**\ , date raised: 2023-01-03 
 * Version 1.5.31: resolved Issue 1360: Python 3.11 (extension)
    - description:  adjust setup.py to support Python 3.11; requires Pybind11 2.10
    - date resolved: **2023-01-03 14:37**\ , date raised: 2023-01-03 
 * Version 1.5.30: resolved Issue 1335: RollingDiscPenalty (extension)
    - description:  add test example for ground moving (rotating table
    - date resolved: **2023-01-02 19:07**\ , date raised: 2022-12-25 
 * Version 1.5.29: resolved Issue 1359: ObjectGround (extension)
    - description:  extend for referenceRotation to allow rotation of ground objects (especially for visualization and contact)
    - date resolved: **2023-01-02 11:49**\ , date raised: 2023-01-02 
 * Version 1.5.28: resolved Issue 0839: multithreaded jacobian (extension)
    - description:  add functionality for multithreaded jacobian and mass matrix
    - **notes:** mass matrix resolved; other is redundant with #1203
    - date resolved: **2023-01-02 01:04**\ , date raised: 2021-12-19 
 * Version 1.5.27: resolved Issue 0828: MT integration2 (extension)
    - description:  add multithreading to mass matrix and jacobian; add multithreaded adding of vector with special templated functions to allow special (templated) solver functions
    - **notes:** redundant with #839
    - date resolved: **2023-01-02 01:02**\ , date raised: 2021-12-09 
 * Version 1.5.26: resolved Issue 0791: test Eigen SimplicialLDLT (extension)
    - description:  use systemMatricesArePD to enable LDLT solver
    - **notes:** tested earlier; does not work in most cases
    - date resolved: **2023-01-02 01:01**\ , date raised: 2021-11-02 
 * Version 1.5.25: resolved Issue 0437: solvercontainer+ systemcontainer (change)
    - description:  remove SolverContainer and SystemContainer from systemStructuresDefinition and work manually
    - **notes:** already done much earlier
    - date resolved: **2023-01-02 00:59**\ , date raised: 2020-07-21 
 * Version 1.5.24: resolved Issue 0315: Add user object (extension)
    - description:  add user object
    - **notes:** done with GenericODE2 and CoordinateVectorConstraint
    - date resolved: **2023-01-02 00:57**\ , date raised: 2020-01-10 
 * Version 1.5.23: resolved Issue 0261: MarkerDataJacobians (extension)
    - description:  Add access functions for jacobians and other marker data: SetPositionJacobian, GetPositionJacobian, etc.; add jacobian types = markertypes, which check if wrong jacobian is accessed
    - **notes:** not suitable anymore
    - date resolved: **2023-01-02 00:56**\ , date raised: 2019-09-11 
 * Version 1.5.22: resolved Issue 1331: exudyn cpp (extension)
    - description:  add __repr__ and help which writes some information on workflow (github, theDoc, Examples, ...)
    - **notes:** not possible
    - date resolved: **2023-01-02 00:51**\ , date raised: 2022-12-21 
 * Version 1.5.21: resolved Issue 1323: AVX2 (extension)
    - description:  update documentation for improved functionality with AVX2 code
    - **notes:** already done when resolving #1330
    - date resolved: **2023-01-02 00:50**\ , date raised: 2022-12-17 
 * Version 1.5.20: resolved Issue 1355: visualizationSettings (check)
    - description:  check possibility for update loop to continue simulation
    - **notes:** not feasible now, as it requires to modify time integration loop
    - date resolved: **2023-01-02 00:48**\ , date raised: 2022-12-31 
 * Version 1.5.19: resolved Issue 1353: visualizationSettings (extension)
    - description:  add KEY Q shortcut to close dialog
    - date resolved: **2023-01-02 00:48**\ , date raised: 2022-12-31 
 * Version 1.5.18: resolved Issue 1358: DictionariesGetSet (change)
    - description:  add new types VectorFloat and MatrixFloat in order to distinguish from double values
    - **notes:** also fixed matrix conversion in visualizationSettings
    - date resolved: **2023-01-02 00:47**\ , date raised: 2023-01-02 
 * Version 1.5.17: resolved Issue 1357: visualizationSettings (extension)
    - description:  add single click edit event
    - date resolved: **2023-01-01 23:27**\ , date raised: 2023-01-01 
 * Version 1.5.16: resolved Issue 1354: visualizationSettings (extension)
    - description:  double click on bool variables changes state
    - date resolved: **2023-01-01 23:22**\ , date raised: 2022-12-31 
 * Version 1.5.15: resolved Issue 1352: tkinter dialogs (fix)
    - description:  add option for transparency to dialogs
    - **notes:** visualizationSettings.dialogs.transparency
    - date resolved: **2022-12-31 11:01**\ , date raised: 2022-12-31 
 * Version 1.5.14: resolved Issue 1351: tkinter MacOS (fix)
    - description:  adjust font size and row size in ttk right mouse dialog
    - **notes:** visualizationSettings.dialogs.fontScalingMacOS
    - date resolved: **2022-12-31 11:00**\ , date raised: 2022-12-31 
 * Version 1.5.13: resolved Issue 1350: tkinter MacOS (fix)
    - description:  adjust font size and row size in ttk visualizationSettings dialog
    - **notes:** visualizationSettings.dialogs.fontScalingMacOS
    - date resolved: **2022-12-31 11:00**\ , date raised: 2022-12-31 
 * Version 1.5.12: :textred:`resolved BUG 0752` : tkinter MacOS 
    - description:  tkinter fails when loaded inside interactive.py; early call to Tk() inside an example works and seems to help; check options to correctly load tkinter in MacOS (Rosetta 2, on M1)
    - **notes:** problem fixed with single threaded renderer, doing illegal operations in glfw callbacks
    - date resolved: **2022-12-31 01:01**\ , date raised: 2021-09-20 
 * Version 1.5.11: resolved Issue 1349: RequireVersion (fix)
    - description:  some bugs including exu.GetVersionString and not raising exception
    - date resolved: **2022-12-30 15:03**\ , date raised: 2022-12-30 
 * Version 1.5.10: resolved Issue 1343: tkinter in exudyn (docu)
    - description:  add comment in FAQ on potential problems with tkinter; add option to let exudyn know if tkinter is already running
    - date resolved: **2022-12-28 23:39**\ , date raised: 2022-12-27 
 * Version 1.5.9: resolved Issue 1347: visualizationSettings (extension)
    - description:  add flag visualizationSettings.dialog.multiThreadedDialogs to turn on/off immediate apply of visualizationSettings changes; extend exudyn.GUI and rendererPythonInterface.h accordingly
    - **notes:** NOTE that this flag should be turned off in case of crashes during/after dialogs; could make problems on special platforms such as MacOS
    - date resolved: **2022-12-28 23:05**\ , date raised: 2022-12-28 
 * Version 1.5.8: resolved Issue 1348: EditDictionaryWithTypeInfo (change)
    - description:  change interface to pass directly visualizationSettings, allowing to update data
    - date resolved: **2022-12-28 21:50**\ , date raised: 2022-12-28 
 * Version 1.5.7: resolved Issue 1346: visualizationSettings (check)
    - description:  check if visualiuation settings dialog can be installed such that updating of data immediately affects renderer window
    - date resolved: **2022-12-28 16:53**\ , date raised: 2022-12-28 
 * Version 1.5.6: resolved Issue 1339: tkinter MacOS (fix)
    - description:  add tkinter.Tk() into exudyn.sys["tkinterRoot"] before call to StartRenderer(); check for tkinterRoot in exudyn.sys on startup of interactive dialog; resolves crash on MacOS for SolutionViewer
    - date resolved: **2022-12-27 23:50**\ , date raised: 2022-12-26 
 * Version 1.5.5: resolved Issue 1345: interface default args (fix)
    - description:  change true/false to True/False in theDoc for PYthon command interface
    - date resolved: **2022-12-27 23:24**\ , date raised: 2022-12-27 
 * Version 1.5.4: resolved Issue 1344: exudyn (extension)
    - description:  add function IsRendererRunning(), to avoid Warnings when renderer is restarted
    - date resolved: **2022-12-27 23:23**\ , date raised: 2022-12-27 
 * Version 1.5.3: resolved Issue 1338: MacOS (fix)
    - description:  DoRendererIdleTasks() crashes if no Renderer is active; add check in python call to avoid crash
    - **notes:** crash resolved on Windows; MacOS test still open
    - date resolved: **2022-12-27 17:36**\ , date raised: 2022-12-26 
 * Version 1.5.2: resolved Issue 1340: MacOS multithreading (change)
    - description:  activate simplified multithreading by switching to TinyThreading in case of MacOS
    - date resolved: **2022-12-27 17:19**\ , date raised: 2022-12-26 
 * Version 1.5.1: resolved Issue 1342: MacOS M1 (extension)
    - description:  add string ARM to Platform string, e.g., used in solution and sensor files
    - date resolved: **2022-12-27 17:03**\ , date raised: 2022-12-27 
 * Version 1.5.0: resolved Issue 1336: test suite (fix)
    - description:  resolve linux problem with RigidBodySpringDamper.py MiniExample
    - **notes:** problem in testsuite due to AVX differences, causing all non-AVX cases to fail (also linux); fixed with special case in reference solutions
    - date resolved: **2022-12-25 20:10**\ , date raised: 2022-12-25 

***********
Version 1.4
***********

 * Version 1.4.65: resolved Issue 1264: RollingDiscPenalty (extension)
    - description:  correct torque on ground, which currently has no effect, but will be used for moving ground
    - **notes:** resolved earlier
    - date resolved: **2022-12-25 12:15**\ , date raised: 2022-09-17 
 * Version 1.4.64: resolved Issue 1298: ReevingSystemLinear (testing)
    - description:  add test model to test suite
    - date resolved: **2022-12-25 12:11**\ , date raised: 2022-12-01 
 * Version 1.4.63: resolved Issue 1333: gcc linux (change)
    - description:  adjust -march compilation options for improved performance on linux; try -march=skylake for AVX2 optimization
    - **notes:** did not work: either gives compilation errors or segmentation faults
    - date resolved: **2022-12-25 00:45**\ , date raised: 2022-12-24 
 * Version 1.4.62: resolved Issue 1334: NOGLFW (change)
    - description:  remove SystemContainer constructor warning for AttachToRenderEngine in case of NOGLFW
    - **notes:** also done for DetachFromRenderEngine
    - date resolved: **2022-12-25 00:06**\ , date raised: 2022-12-25 
 * Version 1.4.61: resolved Issue 1332: MacOS (fix)
    - description:  compilation does not finish due to -framework Cocoa, etc. errors
    - **notes:** changed -framework library lists by separating commands into two separate strings; compilation now runs through for Python 3.8-3.10 on MacOS with M1 and 3.7.-3.10 on _x86 emulation
    - date resolved: **2022-12-24 22:20**\ , date raised: 2022-12-24 
 * Version 1.4.60: resolved Issue 1330: AVX2 (fix)
    - description:  fix check for AVX and AVX2 on module import
    - date resolved: **2022-12-21 19:33**\ , date raised: 2022-12-21 
 * Version 1.4.59: resolved Issue 1329: SolutionViewer (example)
    - description:  add example for SolutionViewer with multiple static simulations performed, writing results into single file; see Examples/solutionViewerMultipleSimulations.py
    - date resolved: **2022-12-21 18:05**\ , date raised: 2022-12-21 
 * Version 1.4.58: resolved Issue 1328: simulationSettings.solutionSettings (extension)
    - description:  add option writeInitialValues in order to turn on/off writing of initial values for coordinatesSolution and sensors; by default turned on as was done so far
    - date resolved: **2022-12-21 14:07**\ , date raised: 2022-12-21 
 * Version 1.4.57: resolved Issue 1037: sensor files (extension)
    - description:  add file footer information same as in coordinatesSolutionFile including CPUtimeElapsed with 3 digits
    - **notes:** turned on by default, but should be switched off if it causes compatibility problems for your postprocessing tool
    - date resolved: **2022-12-21 12:09**\ , date raised: 2022-04-07 
 * Version 1.4.56: resolved Issue 1327: sensorsWriteFileHeader (fix)
    - description:  not working
    - **notes:** added option into solver, now turning on/off as desired; added option into solver, now turning on/off as desired; added option into solver, now turning on/off as desired
    - date resolved: **2022-12-21 12:06**\ , date raised: 2022-12-21 
 * Version 1.4.55: resolved Issue 1317: DistanceSensor (example)
    - description:  Add example with distance sensor
    - date resolved: **2022-12-20 13:49**\ , date raised: 2022-12-16 
 * Version 1.4.54: resolved Issue 1318: DistanceSensor (extension)
    - description:  Add measurement for GeneralContact trigsRigidBody
    - date resolved: **2022-12-19 21:49**\ , date raised: 2022-12-16 
 * Version 1.4.53: resolved Issue 1316: DistanceSensor (extension)
    - description:  add utilities function to create DistanceSensor based on general contact
    - **notes:** added AddDistanceSensor to exudyn utilities.py
    - date resolved: **2022-12-19 01:09**\ , date raised: 2022-12-16 
 * Version 1.4.52: resolved Issue 1324: DistanceSensor (extension)
    - description:  utilities function AddDistanceSensor extended for measureVelocity to measure velocity similar as in a laser Doppler vibrometer (LDV)
    - date resolved: **2022-12-18 20:59**\ , date raised: 2022-12-18 
 * Version 1.4.51: resolved Issue 1320: GeneralContact (extension)
    - description:  add binary contact types to pybind interface; add conversion of binary types to type indices
    - **notes:** only TypeIndex added which is sufficient for DistanceSensor
    - date resolved: **2022-12-17 00:32**\ , date raised: 2022-12-16 
 * Version 1.4.50: resolved Issue 1319: DistanceSensor (extension)
    - description:  Add radius for option to measure with cylinder with given radius instead of line; useful for particles
    - date resolved: **2022-12-17 00:32**\ , date raised: 2022-12-16 
 * Version 1.4.49: resolved Issue 1321: DistanceSensor (extension)
    - description:  Add option to select which contact types are considered
    - date resolved: **2022-12-17 00:31**\ , date raised: 2022-12-16 
 * Version 1.4.48: resolved Issue 1322: AVX2 import (extension)
    - description:  add checks based on numpy.core._multiarray_umath to find if CPU has AVX2 support; in release, user may directly import noAVX version by setting sys.exudynCPUhasAVX2=False
    - date resolved: **2022-12-16 23:42**\ , date raised: 2022-12-16 
 * Version 1.4.47: :textred:`resolved BUG 1315` : SetSearchTreeBox 
    - description:  has no effect, and searchTree is always computed automatically
    - **notes:** now search tree can be initialized smaller or larger than initial geometry in order to optimize for specific problem
    - date resolved: **2022-12-15 21:20**\ , date raised: 2022-12-15 
 * Version 1.4.46: resolved Issue 1268: DistanceSensor (extension)
    - description:  add simple distance sensor
    - **notes:** can be realized with GeneralContact and SensorUserFunction
    - date resolved: **2022-12-15 20:23**\ , date raised: 2022-09-21 
 * Version 1.4.45: resolved Issue 1271: GeneralContact (extension)
    - description:  Add interface functions for GeneralContact: allowing to measure distance along a line; get markers/spheres in box; get beams in box; etc.
    - date resolved: **2022-12-15 20:20**\ , date raised: 2022-09-21 
 * Version 1.4.44: resolved Issue 1313: AVX2 (extension)
    - description:  add additional library without AVX and use try/except to import exudynCPPnoAVX in case that AVX2 is not available
    - date resolved: **2022-12-14 20:27**\ , date raised: 2022-12-14 
 * Version 1.4.43: resolved Issue 1312: SolveDynamic (extension)
    - description:  add flag computeMassMatrixInversePerBody for explicit time integration to compute mass matrix inverse per body; read theDoc and use with care!
    - **notes:** check your models! ComputeMassMatrix has been adapted for every body!!!
    - date resolved: **2022-12-13 21:00**\ , date raised: 2022-12-13 
 * Version 1.4.42: resolved Issue 1311: DynamicSolver (fix)
    - description:  include flag timeIntegration.reuseConstantMassMatrix into explicit solver; up to now, this option was always turned on
    - date resolved: **2022-12-13 18:51**\ , date raised: 2022-12-13 
 * Version 1.4.41: resolved Issue 1310: searchTreeUpdateCounter (fix)
    - description:  needs reset to 0 after reaching limit
    - date resolved: **2022-12-13 17:58**\ , date raised: 2022-12-13 
 * Version 1.4.40: resolved Issue 0847: GeneralContact searchtree (extension)
    - description:  add options to recompute searchtree size if particles are moving out of region; add option to flush all dynamical arrays (searchtree, allActiveContacts, etc.) after certain time
    - **notes:** added option resetSearchTreeInterval into GeneralContact settings
    - date resolved: **2022-12-12 10:40**\ , date raised: 2021-12-23 
 * Version 1.4.39: resolved Issue 1309: rigidBodyUtilities (extension)
    - description:  AddRigidBody: add default argument for nodeType as RotationEulerParameters; simplifies creation of rigid bodies
    - date resolved: **2022-12-08 18:16**\ , date raised: 2022-12-08 
 * Version 1.4.38: resolved Issue 1302: GL_POLYGON_OFFSET_FILL (fix)
    - description:  correct polygon offset setting for regular faces/lines and add adjustable parameter in visualizationSettings
    - date resolved: **2022-12-08 01:50**\ , date raised: 2022-12-06 
 * Version 1.4.37: resolved Issue 1307: OpenGL (change)
    - description:  change line and polygon drawing in order to avoid line artifacts; newly introduced polygon offset may cause problems: check your visualization, send reports if problems and set polygonOffset=0 in severe cases
    - date resolved: **2022-12-08 01:06**\ , date raised: 2022-12-08 
 * Version 1.4.36: resolved Issue 1304: ConnectorRollingDiscPenalty (docu)
    - description:  extend docu for arbitrary planeNormal and moving ground
    - date resolved: **2022-12-07 21:57**\ , date raised: 2022-12-07 
 * Version 1.4.35: resolved Issue 1306: ObjectJointRollingDisc (extension)
    - description:  add discAxis to object parameters, being able to change from default x-axis
    - date resolved: **2022-12-07 20:18**\ , date raised: 2022-12-07 
 * Version 1.4.34: resolved Issue 1305: ObjectJointRollingDisc (change)
    - description:  change computation of trail, generalized for two bodies moving relative to each other (needs further testing)
    - date resolved: **2022-12-07 20:18**\ , date raised: 2022-12-07 
 * Version 1.4.33: resolved Issue 1303: ConnectorRollingDiscPenalty (extension)
    - description:  extended formulation for arbitrary planeNormal; ground body can now also move in space (testing needed)
    - date resolved: **2022-12-07 17:49**\ , date raised: 2022-12-07 
 * Version 1.4.32: resolved Issue 1301: graphicsDataUtilities (fix)
    - description:  AddEdgesAndSmoothenNormals fixed to process colors
    - date resolved: **2022-12-06 12:37**\ , date raised: 2022-12-06 
 * Version 1.4.31: resolved Issue 1300: graphicsDataUtilities (extension)
    - description:  GraphicsDataFromPointsAndTrigs extended to accept color per point or 4 RGBA values
    - date resolved: **2022-12-06 12:37**\ , date raised: 2022-12-06 
 * Version 1.4.30: :textred:`resolved BUG 1299` : GeneralContact 
    - description:  searchTreeBox not computed automatically (must be set manually)
    - **notes:** computation of searchtree performed automatically now if no search tree box is specified; output message written in case of verboseMode=1
    - date resolved: **2022-12-02 12:51**\ , date raised: 2022-12-02 
 * Version 1.4.29: resolved Issue 1297: ConnectorSpringDamper (extension)
    - description:  return scalar spring-damper force with OutputVariableType ForceLocal
    - date resolved: **2022-12-01 16:31**\ , date raised: 2022-12-01 
 * Version 1.4.28: resolved Issue 1296: dynamic solver (check)
    - description:  add check that useIndex2=True is consistent with useNewmark=True
    - date resolved: **2022-11-30 14:12**\ , date raised: 2022-11-29 
 * Version 1.4.27: resolved Issue 0925: ReevingSystemSprings (extension)
    - description:  create reeving system along points defined by markers, using massless springs and one total length; add rigid body markers for position of sheaves; use coordinate markers for prescribed change of length at end of reeving system
    - **notes:** see new ObjectConnectorReevingSystemSprings
    - date resolved: **2022-11-19 01:11**\ , date raised: 2022-02-03 
 * Version 1.4.26: resolved Issue 1295: BeamGeometricallyExact2D (extension)
    - description:  add damping terms for bending, axial and shear deformation
    - date resolved: **2022-11-17 15:59**\ , date raised: 2022-11-17 
 * Version 1.4.25: resolved Issue 1294: BeamGeometricallyExact2D (extension)
    - description:  add reference strain/curvature
    - date resolved: **2022-11-17 15:59**\ , date raised: 2022-11-17 
 * Version 1.4.24: resolved Issue 1293: MarkerDataStructure (extension)
    - description:  extend for more than 2 MarkerData; adjust caller functions
    - date resolved: **2022-11-13 16:53**\ , date raised: 2022-11-13 
 * Version 1.4.23: resolved Issue 1291: Lie Group integration (fix)
    - description:  CompositionRule in LieGroup nodes does not consider reference position; add reference configuration in composition rule (and subtract afterwards)
    - date resolved: **2022-11-09 15:00**\ , date raised: 2022-11-09 
 * Version 1.4.22: :textred:`resolved BUG 1289` : visualizationSettings 
    - description:  interactive.trackMarker has wrong type leading to crash when closing visualizationSettings
    - date resolved: **2022-11-05 14:15**\ , date raised: 2022-11-05 
 * Version 1.4.21: resolved Issue 1286: trackMarker (fix)
    - description:  selection of objects wrong / check mouse selection procedure
    - date resolved: **2022-11-04 20:46**\ , date raised: 2022-11-02 
 * Version 1.4.20: resolved Issue 1288: ConnectorRollingDiscPenalty (extension)
    - description:  add useLinearProportionalZone which performs better in implicit time integration
    - date resolved: **2022-11-04 17:16**\ , date raised: 2022-11-04 
 * Version 1.4.19: resolved Issue 1287: ConnectorRollingDiscPenalty (extension)
    - description:  add viscousFriction, using separate values for local X/Y coordinates
    - date resolved: **2022-11-04 17:16**\ , date raised: 2022-11-04 
 * Version 1.4.18: resolved Issue 1276: Renderer: marker tracking (extension)
    - description:  add option to track markers in renderer
    - **notes:** see visualizationSettings.interactive for several trackMarker... options
    - date resolved: **2022-11-02 17:13**\ , date raised: 2022-10-12 
 * Version 1.4.17: resolved Issue 1279: release_assert (change)
    - description:  remove all asserts (in automatic code generation)
    - date resolved: **2022-11-02 17:10**\ , date raised: 2022-10-13 
 * Version 1.4.16: resolved Issue 1285: Lobatto, LobattoIntegrate (fix)
    - description:  order nomenclature is wrong. Lobatto2 needs to be renamed into Lobatto1 and so on
    - date resolved: **2022-11-02 16:43**\ , date raised: 2022-11-02 
 * Version 1.4.15: resolved Issue 1284: extend ALEANCF beam (extension)
    - description:  add effects due to axial and bending viscous damping in case of movingMassFactor==1
    - **notes:** new damping terms for axially moving beams implemented according to 2022 Paper of Pieber, Ntarladima, Gerstmayr only for case movingMassFactor=1
    - date resolved: **2022-10-31 15:43**\ , date raised: 2022-10-25 
 * Version 1.4.14: :textred:`resolved BUG 1283` : ProcessParameterList 
    - description:  in case useMultiProcessing=False the output file does not contain correct ranges
    - date resolved: **2022-10-20 21:59**\ , date raised: 2022-10-20 
 * Version 1.4.13: resolved Issue 1282: PlotSensor (extension)
    - description:  added return value including [plt, fig, ax, line] to be used for subsequent operations
    - date resolved: **2022-10-19 20:30**\ , date raised: 2022-10-19 
 * Version 1.4.12: resolved Issue 1281: results monitor (extension)
    - description:  extend results monitor for viewing more detailed results of ParameterVariation with colorVariations option; add function SingleIndex2SubIndices
    - date resolved: **2022-10-17 07:30**\ , date raised: 2022-10-17 
 * Version 1.4.11: :textred:`resolved BUG 1280` : ParameterVariation 
    - description:  resultsFile not working (problem with ProcessParameterList
    - date resolved: **2022-10-14 08:22**\ , date raised: 2022-10-14 
 * Version 1.4.10: resolved Issue 1278: center point (fix)
    - description:  add key O to change center of rotation; resolve finally original issue 1155
    - date resolved: **2022-10-12 22:37**\ , date raised: 2022-10-12 
 * Version 1.4.9: resolved Issue 1277: shadowPolygonOffset (change)
    - description:  decrease default value from 10 to 0.1; adjust in your models
    - date resolved: **2022-10-12 20:27**\ , date raised: 2022-10-12 
 * Version 1.4.8: resolved Issue 1275: GetMarkerOutput (extension)
    - description:  add function mbs.GetMarkerOutput() to return position, velocitiy, rotation matrix and angular velocity for markers if available
    - date resolved: **2022-10-12 09:18**\ , date raised: 2022-10-12 
 * Version 1.4.7: :textred:`resolved BUG 1274` : ALECable2D 
    - description:  missing term L in preComputedB terms in C++ implementation
    - date resolved: **2022-09-25 14:17**\ , date raised: 2022-09-25 
 * Version 1.4.6: resolved Issue 1242: Beam3D (change)
    - description:  rename ANCFBeam3D to ANCFBeam and GeometricallyExactBeam3D in same way for consistency reasons
    - date resolved: **2022-09-24 19:35**\ , date raised: 2022-08-24 
 * Version 1.4.5: resolved Issue 1265: RollingDisc (fix)
    - description:  correct description of coordinate systems in RollingDisc and RollingDiscPenalty; remove wrong transposed sign
    - date resolved: **2022-09-19 17:46**\ , date raised: 2022-09-19 
 * Version 1.4.4: resolved Issue 1263: RollingDiscPenalty (extension)
    - description:  add arbitrary local wheel axis (discAxis) instead of x-axis only
    - date resolved: **2022-09-17 10:42**\ , date raised: 2022-09-17 
 * Version 1.4.3: resolved Issue 1262: RigidBodyInertia (extension)
    - description:  add += operator
    - date resolved: **2022-09-17 08:23**\ , date raised: 2022-09-17 
 * Version 1.4.2: resolved Issue 1261: Lie group integration (extension)
    - description:  improve implicit Lie group integration, adapt old rotation vector approach, add tangent operator
    - date resolved: **2022-09-16 18:14**\ , date raised: 2022-09-16 
 * Version 1.4.1: :textred:`resolved BUG 1260` : MacOS 
    - description:  compilation not working on MacOS with taskmanager adaptions
    - date resolved: **2022-09-15 11:14**\ , date raised: 2022-09-15 
 * Version 1.4.0: resolved Issue 0860: SparseVectorDomain (extension)
    - description:  ?NEEDED (see 0862): create templated SparseVector with IndexValue+domain, having C-array of ResizableArray<IndexValue>, used for multithreaded creation of sparse vectors, associated indices for lateron fill into system vectors
    - **notes:** done in TemporaryComputationData now
    - date resolved: **2022-09-15 08:50**\ , date raised: 2022-01-13 

***********
Version 1.3
***********

 * Version 1.3.105: resolved Issue 0861: SparseVectorParallel (extension)
    - description:  create SparseVector with: mainSparseVector + ArrayIndex2 with per-thread max index and current index; SparseTriplets exceeding the index go into ResizableArray<SparseVector\*> threadSparseVector; Function to finally fill all triplets into main vector
    - **notes:** done in TemporaryComputationData now
    - date resolved: **2022-09-15 08:49**\ , date raised: 2022-01-13 
 * Version 1.3.104: resolved Issue 1259: MarkerSuperElementRigid (extension)
    - description:  extend offset for case that it is large
    - date resolved: **2022-09-14 15:18**\ , date raised: 2022-09-14 
 * Version 1.3.103: resolved Issue 1258: AnimateModes (extension)
    - description:  add option to change sign of mode
    - date resolved: **2022-09-14 08:51**\ , date raised: 2022-09-14 
 * Version 1.3.102: :textred:`resolved BUG 1257` : AnimateModes 
    - description:  button for "Faces only" not working due to new way edges are turned on / off; workaround by pressing T in render window
    - **notes:** buttons now affect the mesh faces / edges instead of regular edges / faces
    - date resolved: **2022-09-14 08:28**\ , date raised: 2022-09-13 
 * Version 1.3.101: resolved Issue 1256: GUI (extension)
    - description:  added some variables in exudyn.GUI which may be used to adjust appearance of dialogs, specifically for extreme display scaling
    - date resolved: **2022-09-13 16:42**\ , date raised: 2022-09-13 
 * Version 1.3.100: resolved Issue 1252: use display scaling in GUI (extension)
    - description:  use display scaling in visualizationSettings, etc.
    - date resolved: **2022-09-13 16:42**\ , date raised: 2022-09-13 
 * Version 1.3.99: resolved Issue 1255: tkinter (change)
    - description:  put root calls into try-except clause in order to preserve operation on special systems like MacOS or Linux
    - date resolved: **2022-09-13 14:49**\ , date raised: 2022-09-13 
 * Version 1.3.98: resolved Issue 1254: SolutionViewer, AnimateModes (extension)
    - description:  add font size and title; this allows to scale fonts as monitor scaling is not active here
    - date resolved: **2022-09-13 14:13**\ , date raised: 2022-09-13 
 * Version 1.3.97: resolved Issue 1253: renderState (docu)
    - description:  add description into theDoc, in section 3D graphics visualization
    - date resolved: **2022-09-13 11:12**\ , date raised: 2022-09-13 
 * Version 1.3.96: resolved Issue 1251: useWindowsDisplayScaleFactor (change)
    - description:  changed visualizationsSettings.general option name from useWindowsMonitorScaleFactor to useWindowsDisplayScaleFactor
    - date resolved: **2022-09-13 10:33**\ , date raised: 2022-09-13 
 * Version 1.3.95: resolved Issue 0538: displayScaling (extension)
    - description:  add displayScaling to renderState and use this value for GUI.py in visualizationSettings
    - **notes:** added displayScaling and automatic updating when Windows display scaling is changed or window is moved to other display
    - date resolved: **2022-09-13 10:19**\ , date raised: 2021-01-06 
 * Version 1.3.94: resolved Issue 1250: FEM HurtyCraigBampton (extension)
    - description:  ComputeHurtyCraigBamptonModes now includes option to compute RBE3 case; adds optional boundary node weights
    - date resolved: **2022-09-07 12:33**\ , date raised: 2022-09-06 
 * Version 1.3.93: resolved Issue 1249: FEM interface (extension)
    - description:  add function GetNodeWeightsFromSurfaceAreas which computes correct weights for linear finite elements (tested for tetrahedrals); this weighting can now reduce erroneous offset in MarkerSuperElementRigid significantly; also used for RBE3 mode computation
    - date resolved: **2022-09-07 12:33**\ , date raised: 2022-09-06 
 * Version 1.3.92: resolved Issue 1248: AddObjectFFRFreducedOrderWithUserFunctions (fix)
    - description:  wrong description of user functions; add missing itemIndex in description
    - date resolved: **2022-09-05 18:39**\ , date raised: 2022-09-05 
 * Version 1.3.91: resolved Issue 1246: DrawSystemGraph (extension)
    - description:  add option to create multi-line graphs; improving appearance and readability
    - date resolved: **2022-09-02 09:05**\ , date raised: 2022-09-02 
 * Version 1.3.90: resolved Issue 1244: pre-compiled linux (extension)
    - description:  add all Python 3.6 - 3.10 linux 64 bit versions to pypi with pip installer
    - date resolved: **2022-09-01 10:02**\ , date raised: 2022-08-25 
 * Version 1.3.89: resolved Issue 0562: Gen alpha Lie (extension)
    - description:  add Lie groups to new generalized alpha integrator
    - date resolved: **2022-08-26 14:09**\ , date raised: 2021-01-26 
 * Version 1.3.88: resolved Issue 1245: Rotation output variable (change)
    - description:  changed/fixed OutputVariableType.Rotation for NodeRigidBodyRotVecLG to output Tait-Bryan rotations instead of rotation parameters; corrected description for NodeRigidBodyRxyz: returns rotation parameters directly, NOT recomputed from RotationMatrix
    - date resolved: **2022-08-26 10:25**\ , date raised: 2022-08-26 
 * Version 1.3.87: resolved Issue 1243: Lie group integration (extension)
    - issue author: S. Holzinger
    - description:  add new functions for implicit Lie group integration (FIRST TESTS)
    - date resolved: **2022-08-24 17:13**\ , date raised: 2022-08-24 
    - resolved by: S. Holzinger
 * Version 1.3.86: :textred:`resolved BUG 1239` : Timer registration 
    - description:  self-registration of Timers fails on Linux and may lead to crashes; depends on order of initialization of global variables
    - **notes:** changed timer registration to suggested way with scalar variables, guaranteeing initialization
    - date resolved: **2022-08-24 15:10**\ , date raised: 2022-08-24 
 * Version 1.3.85: :textred:`resolved BUG 1225` : Linux TestSuite 
    - description:  segmentation fault when running ANCFgeneralContactCircle.py
    - date resolved: **2022-08-24 09:13**\ , date raised: 2022-08-11 
 * Version 1.3.84: resolved Issue 1238: visualization dialog (change)
    - description:  resort options, such that contour options are on top
    - date resolved: **2022-08-23 15:19**\ , date raised: 2022-08-23 
 * Version 1.3.83: resolved Issue 1237: Beams (extension)
    - description:  add option for drawing filled cross-sections (or alternatively wire frames)
    - date resolved: **2022-08-23 15:19**\ , date raised: 2022-08-23 
 * Version 1.3.82: resolved Issue 1236: GeometricallyExactBeam (fix)
    - description:  GeometricallyExactBeam2D and GeometricallyExactBeam3D do not show values in contour plot
    - date resolved: **2022-08-23 14:15**\ , date raised: 2022-08-23 
 * Version 1.3.81: resolved Issue 1235: GeometricallyExactBeam2D (fix)
    - description:  add missing output variables (strain, curvatureLocal, forces,torques)
    - date resolved: **2022-08-23 12:02**\ , date raised: 2022-08-23 
 * Version 1.3.80: :textred:`resolved BUG 1233` : KinematicTree 
    - description:  Jacobian in KinematicTree has too many approximations: either missing velocity terms have large influence or double entries for connectors on single KinematicTree; check stiffFlyballGovernor w/o systemWideDifferentiation
    - **notes:** fixed JacobianODE2 for duplicate global indices, e.g., in case that Connector is attached with two markers to same object (kinematic tree)
    - date resolved: **2022-08-23 11:41**\ , date raised: 2022-08-22 
 * Version 1.3.79: resolved Issue 1234: Newton / Jacobian (extension)
    - description:  add new Newton / numericalDifferentiation setting jacobianConnectorDerivative for faster Jacobian computations
    - date resolved: **2022-08-23 10:15**\ , date raised: 2022-08-23 
 * Version 1.3.78: resolved Issue 1224: KinematicTree (check)
    - description:  visulization problems with kinematicTreeConstraintTest.py with 10 links and more; may be caused by wrong states in visualization or possible bug in KinematicTreeMarker
    - **notes:** resolved with issue 1232 by adding additional temporary variables for visualization
    - date resolved: **2022-08-22 23:26**\ , date raised: 2022-08-07 
 * Version 1.3.77: :textred:`resolved BUG 1232` : KinematicTree 
    - description:  visualization of joints uses illegal temporary data; leads to data race and erroneous results
    - date resolved: **2022-08-22 23:23**\ , date raised: 2022-08-22 
 * Version 1.3.76: :textred:`resolved BUG 1231` : BasicDefinitions 
    - description:  C++: definition of MAXREAL wrong; affects searchtree and contact
    - date resolved: **2022-08-22 21:33**\ , date raised: 2022-08-22 
 * Version 1.3.75: resolved Issue 1230: GetInitialVector (change)
    - description:  change GetInitialVector into GetInitialCoordinateVector for consistency reasons; samge for GetInitialVector_t, SetInitialVector, SetInitialVector_t
    - date resolved: **2022-08-17 17:53**\ , date raised: 2022-08-17 
 * Version 1.3.74: :textred:`resolved BUG 1229` : CMarkerBodyCable2DShape 
    - description:  system error due to incorrect initialization of matrix
    - date resolved: **2022-08-15 15:31**\ , date raised: 2022-08-15 
 * Version 1.3.73: resolved Issue 1228: add LieGroup node with data coordinates (extension)
    - description:  add special Lie group node for implicit integration, containing the start-of-step configuration in data coordinates and additionally use regular ODE2 coordinates for the incremental motion
    - date resolved: **2022-08-12 19:35**\ , date raised: 2022-08-12 
 * Version 1.3.72: resolved Issue 1227: CNode.cpp (change)
    - description:  C++: remove exceptions for illegal index access in GetCurrentCoordinate(...)
    - date resolved: **2022-08-12 19:18**\ , date raised: 2022-08-12 
 * Version 1.3.71: resolved Issue 1226: initial coordinates (extension)
    - description:  C++: nodes get a separate SetInitialCoordinateVector() function; used for special nodes with mixed coordinates
    - date resolved: **2022-08-12 19:17**\ , date raised: 2022-08-12 
 * Version 1.3.70: resolved Issue 1115: KinematicTree (extension)
    - description:  C++ implement efficient T66 transformations
    - **notes:** stable implementation, with formulas different to Featherstone formulas, but in line with 6D matrix manipulations; further tests and documentation needed
    - date resolved: **2022-08-07 22:55**\ , date raised: 2022-05-29 
 * Version 1.3.69: :textred:`resolved BUG 1223` : T66MotionInverse 
    - description:  C++: RigidBodyMath implementation of T66 inverse is wrong, could affect special KinematicTree force reaction
    - date resolved: **2022-08-05 21:45**\ , date raised: 2022-08-05 
 * Version 1.3.68: resolved Issue 1222: KinematicTree (change)
    - description:  remove some temporary variables from interface as new efficient transformations cannot be converted to Python easily
    - date resolved: **2022-08-04 16:28**\ , date raised: 2022-08-04 
 * Version 1.3.67: resolved Issue 1221: Transformations66List (change)
    - description:  C++: rename into Transformation66List
    - date resolved: **2022-08-04 16:27**\ , date raised: 2022-08-04 
 * Version 1.3.66: resolved Issue 1220: PlotImage (extension)
    - description:  add options for orthogonal projection and removing axes and background
    - date resolved: **2022-07-26 09:34**\ , date raised: 2022-07-26 
 * Version 1.3.65: resolved Issue 1205: LinkedDataVectorParallel (check)
    - description:  C++: check if LinkedDataVector can obtain performance mode from ResizableVectorParallel
    - date resolved: **2022-07-22 20:45**\ , date raised: 2022-07-12 
 * Version 1.3.64: resolved Issue 1206: parallel (extension)
    - description:  C++: parallelize important vector-vector and matrix-vector (MultMatrix, MultAdd, ...) operations with optional commands
    - **notes:** already done in ResizableVectorParallel, but extensions in LinkedDataVector needed
    - date resolved: **2022-07-22 19:49**\ , date raised: 2022-07-12 
 * Version 1.3.63: resolved Issue 1218: SolverExplicit (extension)
    - description:  parallelize Lie group updates in explicit solver
    - date resolved: **2022-07-22 19:26**\ , date raised: 2022-07-22 
 * Version 1.3.62: resolved Issue 1219: SolverExplicit (change)
    - description:  turn off Lie group integration if no Lie group nodes available
    - date resolved: **2022-07-22 18:15**\ , date raised: 2022-07-22 
 * Version 1.3.61: resolved Issue 1160: numpy arrays (change)
    - description:  change conversion behavior for Vector3D and Matrix3D, automatically transformed into numpy arrays instead of std::vector which gives a list right now
    - **notes:** for items containing VectorXD or MatrixXD, the parameter returned in GetObject() and similar functions is now giving numpy arrays; for system structures, this is anyway already implemented; specifically, the changed behaviour can be used for referenceCoordinates in user functions; BEHAVIOUR CHANGED: check your models!
    - date resolved: **2022-07-21 19:33**\ , date raised: 2022-06-26 
 * Version 1.3.60: resolved Issue 1209: parallel (extension)
    - description:  C++: add multithreaded parallelization for PostNewton
    - date resolved: **2022-07-21 10:02**\ , date raised: 2022-07-14 
 * Version 1.3.59: resolved Issue 1035: TestModels (change)
    - description:  change modelUnitTests imports in TestModels such that they also work in case that example is run outside TestModels directory
    - **notes:** done earlier
    - date resolved: **2022-07-20 14:33**\ , date raised: 2022-04-07 
 * Version 1.3.58: resolved Issue 1016: TestSuite (testing)
    - description:  add example of reeving system
    - **notes:** done earlier
    - date resolved: **2022-07-20 14:33**\ , date raised: 2022-03-28 
 * Version 1.3.57: resolved Issue 1217: GraphicsData (extension)
    - description:  add functions for drawing text, line and circle: GraphicsDataLine, GraphicsDataText, GraphicsDataCircle
    - date resolved: **2022-07-20 09:45**\ , date raised: 2022-07-20 
 * Version 1.3.56: :textred:`resolved BUG 1216` : renderer: Circle 
    - description:  circles drawn wrongly, not closed for circleTiling <= 6 and producing overly many lines
    - date resolved: **2022-07-19 20:11**\ , date raised: 2022-07-19 
 * Version 1.3.55: resolved Issue 1214: Export lines (extension)
    - description:  add option to export all lines from renderer similar to RenderImage, however, just exporting the raw line information (2 points, RGBA color)
    - date resolved: **2022-07-19 18:39**\ , date raised: 2022-07-19 
 * Version 1.3.54: resolved Issue 1215: GraphicsData addEdges (fix)
    - description:  some examples still in a previous state expecting wrong GraphicsData format
    - date resolved: **2022-07-19 12:14**\ , date raised: 2022-07-19 
 * Version 1.3.53: :textred:`resolved BUG 1213` : mbs.systemData.Info() 
    - description:  mbs.systemData.Info() gives error for KinematicTree
    - date resolved: **2022-07-15 15:34**\ , date raised: 2022-07-15 
 * Version 1.3.52: resolved Issue 1212: DynamicSolverType::RK67 (fix)
    - description:  shows DynamicSolverType::invalid
    - date resolved: **2022-07-15 15:00**\ , date raised: 2022-07-15 
 * Version 1.3.51: resolved Issue 1211: PlotSensor (extension)
    - description:  add listMarkerStyles and listMarkerStylesFilled to exudyn.plot for use in loops
    - date resolved: **2022-07-15 09:28**\ , date raised: 2022-07-15 
 * Version 1.3.50: resolved Issue 1210: microThread (check)
    - description:  check if threading can be optimized by only using sync atomic variables
    - **notes:** no big improvement found, also using bit-wise thread communication and saving some atomic operations
    - date resolved: **2022-07-14 20:40**\ , date raised: 2022-07-14 
 * Version 1.3.49: resolved Issue 1208: timer PostNewton (extension)
    - description:  add timer for PostNewtonStep
    - date resolved: **2022-07-14 00:15**\ , date raised: 2022-07-14 
 * Version 1.3.48: resolved Issue 1207: microThreading (extension)
    - description:  add micro threading library which already takes effect for small systems (10-20 3D rigid bodies); currently included with separate compile option in BasicDefinitions.h
    - date resolved: **2022-07-13 13:43**\ , date raised: 2022-07-13 
 * Version 1.3.47: resolved Issue 1199: adjust testSuite (testing)
    - description:  due to change of state vector to ResizableVectorParallel with AVX arithmetic, minor changes in reference solutions happened
    - date resolved: **2022-07-12 17:06**\ , date raised: 2022-07-11 
 * Version 1.3.46: resolved Issue 1193: SparseSolver analyzePattern (extension)
    - description:  add option to reuse result of analyzePattern() for successive computations; especially in time integration, using the number of non-zeros as indicator is something has changed; add some initializationFunctions to reset, especially after non-convergence
    - date resolved: **2022-07-12 17:06**\ , date raised: 2022-07-10 
 * Version 1.3.45: resolved Issue 1204: parallel / multithreaded (extension)
    - description:  C++: add multithreading for AlgebraicEquations
    - date resolved: **2022-07-12 15:03**\ , date raised: 2022-07-12 
 * Version 1.3.44: resolved Issue 1201: parallel / multithreaded (extension)
    - description:  C++: add multithreading for ProjectedReactionForces
    - date resolved: **2022-07-12 13:04**\ , date raised: 2022-07-12 
 * Version 1.3.43: resolved Issue 1200: CSystem::Jacobians (change)
    - description:  C++: removed single flags for jacobians and replaced by JacobianType; check your results (bugs may happen)
    - date resolved: **2022-07-12 10:55**\ , date raised: 2022-07-12 
 * Version 1.3.42: resolved Issue 1198: parallel MassMatrix (extension)
    - description:  use multithreading to compute mass matrix; treat objects with user functions separately
    - date resolved: **2022-07-11 23:18**\ , date raised: 2022-07-11 
 * Version 1.3.41: :textred:`resolved BUG 1197` : ResizableArray 
    - description:  C++: copy of illegal parts of memory when enlarging array
    - date resolved: **2022-07-11 22:58**\ , date raised: 2022-07-11 
 * Version 1.3.40: resolved Issue 1195: remove openmp (change)
    - description:  remove openmp compile options from setup.py as it is not used for now
    - date resolved: **2022-07-11 19:02**\ , date raised: 2022-07-11 
 * Version 1.3.39: resolved Issue 1191: SystemState (change)
    - description:  C++: replace Vector state with ResizableVector(Parallel)
    - date resolved: **2022-07-10 14:50**\ , date raised: 2022-07-10 
 * Version 1.3.38: resolved Issue 1186: ALEANCFCable2D (extension)
    - description:  add missing terms related to curvature and strain coupling with delta qALE
    - date resolved: **2022-07-09 21:25**\ , date raised: 2022-07-06 
 * Version 1.3.37: resolved Issue 1190: ContactFrictionCircleCable2D (extension)
    - description:  now adapted to work in general with ALECable2D, however, only for tangential frictionStiffness=0
    - date resolved: **2022-07-09 14:33**\ , date raised: 2022-07-09 
 * Version 1.3.36: :textred:`resolved BUG 1188` : beam.py 
    - description:  missing eii structure before Point2DS1
    - date resolved: **2022-07-08 16:22**\ , date raised: 2022-07-08 
 * Version 1.3.35: resolved Issue 1185: ContactFrictionCircleCable2D (extension)
    - description:  add ALE term to marker and contact element
    - date resolved: **2022-07-06 15:30**\ , date raised: 2022-07-06 
 * Version 1.3.34: resolved Issue 1183: star imports (change)
    - description:  remove \* imports from all .py modules except utilities.py
    - **notes:** also fixed some undetected bugs in unused functions in exudyn.* utilities
    - date resolved: **2022-07-06 11:56**\ , date raised: 2022-07-06 
 * Version 1.3.33: resolved Issue 1184: exudyn.utilities (change)
    - description:  remove import of time and copy; needs to be included separately into models
    - **notes:** check your models!
    - date resolved: **2022-07-06 11:55**\ , date raised: 2022-07-06 
 * Version 1.3.32: resolved Issue 1182: roboticsCore.py (change)
    - description:  remove \* import of many exudyn packages
    - date resolved: **2022-07-06 10:58**\ , date raised: 2022-07-06 
 * Version 1.3.31: resolved Issue 1181: setup.py (change)
    - description:  due to deprecation warning, use namespace_packages instead of packages in setup(...); remove include_package_data=True
    - date resolved: **2022-07-06 09:43**\ , date raised: 2022-07-06 
 * Version 1.3.30: resolved Issue 0954: ObjectContactFrictionCircleCable2D (extension)
    - description:  Add exception in PreAssembleChecks if marker is not a MarkerBody, as implementation only works for MarkerBody but not for MarkerNode
    - **notes:** resolved, as now a MarkerNodeRigid may also be used
    - date resolved: **2022-07-05 21:18**\ , date raised: 2022-02-27 
 * Version 1.3.29: resolved Issue 1180: AddEdgesAndSmoothenNormals (extension)
    - description:  specialized function for STL file enhancement
    - date resolved: **2022-07-05 20:51**\ , date raised: 2022-07-05 
 * Version 1.3.28: resolved Issue 1179: show lines (extension)
    - description:  add separate visualization.openGL flag for showing/hiding lines
    - date resolved: **2022-07-05 11:20**\ , date raised: 2022-07-05 
 * Version 1.3.27: resolved Issue 1178: STL import/export (extension)
    - description:  add option to invert triangles and/or normls on import or export of STL meshes
    - date resolved: **2022-07-05 08:38**\ , date raised: 2022-07-05 
 * Version 1.3.26: resolved Issue 1177: selection right mouse (extension)
    - description:  now also showing GraphicsData optionally with visualizationSettings.interactive.selectionRightMouseGraphicsData
    - date resolved: **2022-07-05 08:38**\ , date raised: 2022-07-05 
 * Version 1.3.25: resolved Issue 1099: TriangleList (extension)
    - description:  extend GraphicsData TriangleList with two optional lists (which may be empty) containing edges (tuples of point numbers) as well as edgeColors; allows to easily add edges to graphics representation
    - date resolved: **2022-07-05 00:04**\ , date raised: 2022-05-22 
 * Version 1.3.24: resolved Issue 1174: mbs.GetObject (extension)
    - description:  add option to receive graphicsData (or not)
    - **notes:** default behavior kept same, not returning graphicsData in dict
    - date resolved: **2022-07-04 23:59**\ , date raised: 2022-07-04 
 * Version 1.3.23: resolved Issue 1159: BodyGraphicsData (extension)
    - description:  add method to convert bodyGraphicsData into dictionary; add flag to GetObject to by default not show graphicsData
    - date resolved: **2022-07-04 23:59**\ , date raised: 2022-06-26 
 * Version 1.3.22: resolved Issue 1176: GraphicsDataCylinder...() (change)
    - description:  returns always a GraphicsData dictionary, independently of addEdges is True or False
    - date resolved: **2022-07-04 23:33**\ , date raised: 2022-07-04 
 * Version 1.3.21: resolved Issue 1175: GraphicsDataOrthoCube...() (change)
    - description:  returns always a GraphicsData dictionary, independently of adding edges or not
    - date resolved: **2022-07-04 23:33**\ , date raised: 2022-07-04 
 * Version 1.3.20: resolved Issue 1169: conversion to STL (extension)
    - description:  add function to convert graphicsData triangle meshes into STL; allows import into other tools
    - date resolved: **2022-07-04 20:55**\ , date raised: 2022-07-03 
 * Version 1.3.19: resolved Issue 0821: include numpy-stl (extension)
    - description:  include library to import stl files; add Warning to GraphicsDataFromSTLfileTxt for large file sizes and check if is ascii; in binary case, try switching to numpy-stl; add example to load binary files; add example to convert stl ascii to binary files
    - **notes:** example for stl import added: Examples/stlFileImport.py
    - date resolved: **2022-07-04 20:55**\ , date raised: 2021-12-06 
 * Version 1.3.18: resolved Issue 1173: GetObjectOutputBody (change)
    - description:  add default argument localPosition=[0,0,0]
    - date resolved: **2022-07-04 11:45**\ , date raised: 2022-07-04 
 * Version 1.3.17: resolved Issue 1172: GetObjectOutput (change)
    - description:  include configurationType in interface (default: Current); not available in connectors; add objectNumber in C++ interface
    - date resolved: **2022-07-04 11:43**\ , date raised: 2022-07-04 
 * Version 1.3.16: :textred:`resolved BUG 1171` : MarkerKinematicTreeRigid 
    - description:  incorrect GetPosition, GetVelocity, GetAngularVelocity, ...
    - date resolved: **2022-07-04 11:43**\ , date raised: 2022-07-03 
 * Version 1.3.15: resolved Issue 1161: KinematicTree (change)
    - description:  change GetObjectOutputBody to GetObjectOutput as localPosition does not make sense here
    - date resolved: **2022-07-04 11:39**\ , date raised: 2022-06-27 
 * Version 1.3.14: resolved Issue 1170: MarkerKinematicTreeRigid (extension)
    - description:  add visualization
    - date resolved: **2022-07-03 20:23**\ , date raised: 2022-07-03 
 * Version 1.3.13: resolved Issue 1168: left mouse (change)
    - description:  deactivate mouse select if renderer is showing mouse coordinates (and doing measuring)
    - date resolved: **2022-07-02 11:29**\ , date raised: 2022-07-02 
 * Version 1.3.12: resolved Issue 1166: InteractiveDialog (extension)
    - description:  In all interactive dialogs, especially the SolutionViewer now stopping the render window (with Q or Escape) also stops the interactive dialog; behavior changed with checkRenderEngineStopFlag
    - date resolved: **2022-06-29 10:15**\ , date raised: 2022-06-29 
 * Version 1.3.11: resolved Issue 1165: class Robot (change)
    - description:  drawing of cylinder for first body in robot removed; this part must be drawn manually at base (or not drawn)
    - date resolved: **2022-06-29 10:02**\ , date raised: 2022-06-29 
 * Version 1.3.10: resolved Issue 0720: AddObjectFFRFreducedOrder (extension)
    - description:  add gravity as with user functions version, similar to AddRigidBody
    - date resolved: **2022-06-29 09:27**\ , date raised: 2021-07-12 
 * Version 1.3.9: resolved Issue 1018: MacOS (change)
    - description:  adjust mouse scroll factor in case of Apple/Mac OS compilation (e.g. multiply with 0.05 by default)
    - **notes:** changed way to compute zoomFactor for larger yOffsets in scroll callback
    - date resolved: **2022-06-28 15:30**\ , date raised: 2022-03-29 
 * Version 1.3.8: resolved Issue 1164: OpenGL shadow (fix)
    - description:  resolve issues with shadows drawing of many objects; added _WRAP to stencil INCR and DECR operations to resolve overflows
    - date resolved: **2022-06-27 19:20**\ , date raised: 2022-06-27 
 * Version 1.3.7: resolved Issue 1163: OpenGL normals (fix)
    - description:  correct orientation of triangles and normals in GraphicsData functions
    - date resolved: **2022-06-27 16:22**\ , date raised: 2022-06-27 
 * Version 1.3.6: resolved Issue 1162: OpenGL normals (change)
    - description:  correct normals in GraphicsDataSphere, GraphicsDataCylinder and GraphicsDataSolidOfRevolution to point outwards; turn off GL_LIGHT_MODEL_TWO_SIDE by default
    - date resolved: **2022-06-27 14:12**\ , date raised: 2022-06-27 
 * Version 1.3.5: resolved Issue 1158: RigidBodyInertia (extension)
    - description:  add Transformed() function to return rigid body inertia transformed by homogeneous transformation; used in class Robot for certain transformations of link frame
    - date resolved: **2022-06-27 01:20**\ , date raised: 2022-06-26 
 * Version 1.3.4: resolved Issue 1072: class Robot (extension)
    - description:  extend KinematicTree export in order to be able to create robots with localHT!=HT0()
    - date resolved: **2022-06-27 00:41**\ , date raised: 2022-05-06 
 * Version 1.3.3: resolved Issue 1131: class Robot (extension)
    - description:  add list of graphicsDataLists to each body for individual graphics of links allowing also graphics for tools
    - date resolved: **2022-06-26 23:46**\ , date raised: 2022-06-03 
 * Version 1.3.2: resolved Issue 1094: add links to other github repos (docu)
    - description:  link more directly to ngsolve and openAI / stable_baslines3 / etc.; refer to openAI in github links and add ref to theDoc.pdf ; add example/teaser?
    - date resolved: **2022-06-24 09:20**\ , date raised: 2022-05-21 
 * Version 1.3.1: resolved Issue 1157: C++ user functions (testing)
    - description:  check if linking C++ user functions directly boosts performance
    - **notes:** C++ functions are translated by including pybind11/functional.h; workaround would be two user functions, one without MainSystem and without including functional.h; C++ functions could then be compiled in a very simple manner
    - date resolved: **2022-06-23 16:57**\ , date raised: 2022-06-23 
 * Version 1.3.0: :textred:`resolved BUG 0677` : single threaded renderer 
    - description:  correct crash with visualization dialog (MacOS)
    - **notes:** not resolved, but it is an issue of missing tkinter capabilities in MacOS
    - date resolved: **2022-06-22 07:59**\ , date raised: 2021-05-12 

***********
Version 1.2
***********

 * Version 1.2.146: resolved Issue 0735: parallel build (check)
    - description:  check parallel build with MSbuild to reduce compilation times
    - **notes:** already done earlier
    - date resolved: **2022-06-22 07:58**\ , date raised: 2021-08-12 
 * Version 1.2.145: resolved Issue 0863: RaspberryPi (extension)
    - description:  check compilation on Raspi, make adaptation of ngsolve includes to run
    - **notes:** already done earlier
    - date resolved: **2022-06-22 07:48**\ , date raised: 2022-01-14 
 * Version 1.2.144: resolved Issue 0968: MarkerBodyCable2DShape (extension)
    - description:  add offset from beam axis, to compute position, velocity and jacobians at contact surface
    - **notes:** already done earlier
    - date resolved: **2022-06-22 07:46**\ , date raised: 2022-03-03 
 * Version 1.2.143: resolved Issue 0972: ContactFrictionCircleCable2D (docu)
    - description:  add / extend description
    - date resolved: **2022-06-22 07:44**\ , date raised: 2022-03-09 
 * Version 1.2.142: resolved Issue 1155: center point (extension)
    - description:  add key "o" option to set center point to current center point of view; this allows to rotate around the current center point of the view
    - **notes:** currently deactivated due to transformation of coordinates issue
    - date resolved: **2022-06-21 18:16**\ , date raised: 2022-06-21 
 * Version 1.2.141: resolved Issue 1152: Renderer (extension)
    - description:  add separate function to compute accurate scene size, store as 3D vector and as norm
    - date resolved: **2022-06-21 17:56**\ , date raised: 2022-06-20 
 * Version 1.2.140: resolved Issue 1154: Zoom all (change)
    - description:  change procedures for zoom all and for computation of maximum scene coordinates; may affect appearance of your models; necessary for consistently computing perspective and shadow parameters
    - date resolved: **2022-06-21 08:38**\ , date raised: 2022-06-21 
 * Version 1.2.139: resolved Issue 1153: noglfw (check)
    - description:  check compilation without GLFW
    - date resolved: **2022-06-20 10:58**\ , date raised: 2022-06-20 
 * Version 1.2.138: resolved Issue 1150: OpenGL (extension)
    - description:  add shadows (simple)
    - **notes:** activate with SC.VisualizationSettings().openGL.shadow, chosing a value between 0. and 1; good results obtained with 0.5
    - date resolved: **2022-06-20 01:49**\ , date raised: 2022-06-18 
 * Version 1.2.137: resolved Issue 1151: linux builds (change)
    - description:  remove -g flag from linux builds, leading to 2.6MB instead of 38MB binaries; to enable debug information (e.g. to detect origin of some crashes, remove the -g0 flag in setup.py)
    - date resolved: **2022-06-19 18:34**\ , date raised: 2022-06-19 
 * Version 1.2.136: resolved Issue 1149: OpenGL (extension)
    - description:  add perspective
    - **notes:** added openGL option perspective; EXPERIMENTAL!
    - date resolved: **2022-06-19 01:59**\ , date raised: 2022-06-18 
 * Version 1.2.135: resolved Issue 1148: jacobian ODE1 ODE2 (fix)
    - description:  fix jacobian computations for mixed ODE1-ODE2 components; check with HydraulicActuatorSimple (lower number of jacobians and iterations)
    - date resolved: **2022-06-18 23:38**\ , date raised: 2022-06-18 
 * Version 1.2.134: resolved Issue 1147: ODE1Coordinates_t (extension)
    - description:  add missing function SetODE1Coordinates_t and GetODE1Coordinates_t into Python interface
    - date resolved: **2022-06-17 01:14**\ , date raised: 2022-06-17 
 * Version 1.2.133: resolved Issue 1096: hydraulics (example)
    - description:  add hydraulic actuator with new element HydraulicActuatorSimple
    - date resolved: **2022-06-17 00:06**\ , date raised: 2022-05-22 
 * Version 1.2.132: resolved Issue 1146: solver jacobians (extension)
    - description:  extended jacobians for ODE1 and ODE2 residuals for ODE1-ODE2 coupling in ComputeJacobianODE2RHS, ComputeJacobianODE1RHS, and ComputeJacobianAE; changed default values as well; check your code if you used these functions in Python user functions
    - date resolved: **2022-06-16 19:35**\ , date raised: 2022-06-16 
 * Version 1.2.131: resolved Issue 1145: static solver (extension)
    - description:  add check if system contains ODE1 variables
    - date resolved: **2022-06-16 18:52**\ , date raised: 2022-06-16 
 * Version 1.2.130: resolved Issue 1144: PlotSensor (extension)
    - description:  added option to allow components=[plot.componentNorm] for displaying norm of sensors
    - date resolved: **2022-06-16 17:32**\ , date raised: 2022-06-16 
 * Version 1.2.129: resolved Issue 1143: numba jit (example)
    - description:  add example for numba jit speedup of Python user functions
    - **notes:** added Example springDamperUserFunctionNumbaJIT.py showing speedup of 4 for simple Python function; note that mbs functions cannot be processed by numba
    - date resolved: **2022-06-16 17:01**\ , date raised: 2022-06-16 
 * Version 1.2.128: resolved Issue 1106: HydraulicsActuator (extension)
    - description:  add HydraulicsActuator as object, containing pressure equations
    - **notes:** added new double acting HydraulicActuatorSimple with internal pressure equations and possibility to modify valves by user functions
    - date resolved: **2022-06-16 12:00**\ , date raised: 2022-05-23 
 * Version 1.2.127: resolved Issue 1138: artificialIntelligence (extension)
    - description:  OpenAIGymInterfaceEnv obtained additional member variable randomInitializationValue which may be adapted; in future, this may be done in a separate function
    - date resolved: **2022-06-10 12:26**\ , date raised: 2022-06-10 
 * Version 1.2.126: resolved Issue 1137: exudynFast (change)
    - description:  for linux builds, do not activate exudynFast, only for noglfw
    - date resolved: **2022-06-10 12:16**\ , date raised: 2022-06-10 
 * Version 1.2.125: :textred:`resolved BUG 1135` : KinematicTree 
    - description:  linux version of KinematicTree gives significantly (6e-7) different results, check initialization
    - **notes:** resulted due to high sensitivity to disturbances (1e-15), especially in acceleration sensors, and ONLY for mbs results!
    - date resolved: **2022-06-09 20:32**\ , date raised: 2022-06-09 
 * Version 1.2.124: resolved Issue 1134: KinematicTree (extension)
    - description:  add Jacobian computation according to class Robot in order to realize AccessFunction needed from MarkerKinematicTreeRigid; uses special access function because the SuperElement interface does not provide link number
    - date resolved: **2022-06-08 18:30**\ , date raised: 2022-06-06 
 * Version 1.2.123: resolved Issue 1071: KinematicTree (extension)
    - description:  add special MarkerKinematicTreeRigidBody with link number and local position
    - date resolved: **2022-06-08 18:30**\ , date raised: 2022-05-05 
 * Version 1.2.122: resolved Issue 1130: class Robot (change)
    - description:  tool only working for serial robots; in case of tree structure, tool should not be used as it is attached only to last link
    - **notes:** added warning to class Robot.AddLink(...) which raises Warning if tool is defined and tree structure is generated
    - date resolved: **2022-06-06 00:32**\ , date raised: 2022-06-03 
 * Version 1.2.121: resolved Issue 1132: class Robot (extension)
    - description:  extend Jacobian, LinkHT and JointHT functions for tree structure; check StaticTorques
    - **notes:** added comparison for class Robot functions in kinematicTreeAndMBStest.py
    - date resolved: **2022-06-06 00:31**\ , date raised: 2022-06-03 
 * Version 1.2.120: resolved Issue 1133: SolutionViewer (extension)
    - description:  bind additional Button "Q" in interactive dialogs for quit (in addition to Escape)
    - date resolved: **2022-06-05 18:25**\ , date raised: 2022-06-05 
 * Version 1.2.119: resolved Issue 1114: class Robot (testing)
    - description:  create new example(s) comparing CreateKinematicTree and CreateRedundantCoordinateMBS with control and prismatic joints
    - date resolved: **2022-06-05 14:33**\ , date raised: 2022-05-29 
 * Version 1.2.118: :textred:`resolved BUG 1127` : class Robot 
    - description:  CreateKinematicTree not working for tree structure
    - date resolved: **2022-06-03 00:28**\ , date raised: 2022-06-03 
 * Version 1.2.117: :textred:`resolved BUG 1129` : class Robot 
    - description:  GetParentIndex(...), HasParent(...) not working for tree structure
    - date resolved: **2022-06-03 00:19**\ , date raised: 2022-06-03 
 * Version 1.2.116: :textred:`resolved BUG 1128` : class Robot 
    - description:  CreateRedundantCoordinateMBS not working for tree structure
    - date resolved: **2022-06-03 00:19**\ , date raised: 2022-06-03 
 * Version 1.2.115: :textred:`resolved BUG 1126` : KinematicTree 
    - description:  wrong formula in computation of acceleration
    - date resolved: **2022-06-02 22:41**\ , date raised: 2022-06-02 
 * Version 1.2.114: resolved Issue 1123: class Robot (extension)
    - description:  extend CreateRedundantCoordinateMBS for jointSpringDamperUserFunctionList with prismatic joints
    - date resolved: **2022-06-01 23:44**\ , date raised: 2022-06-01 
 * Version 1.2.113: resolved Issue 1125: class Robot (extension)
    - description:  extend CreateRedundantCoordinateMBS for control of PrsimaticJoints with new LinearSpringDamper
    - date resolved: **2022-06-01 23:43**\ , date raised: 2022-06-01 
 * Version 1.2.112: resolved Issue 1124: LinearSpringDamper (extension)
    - description:  add LinearSpringDamper which is the corresponding spring damper/actuator for prismatic joints (like TorsionalSpringDamper for revolute joints), being aligned with a rigid marker, having no limits compared to regular (distance based) SpringDamper
    - date resolved: **2022-06-01 23:43**\ , date raised: 2022-06-01 
 * Version 1.2.111: resolved Issue 0503: serialrobot (extension)
    - description:  adapt serial robot for prismatic joints
    - **notes:** functionality with Prismatic joints  now included into class Robot, converting to mbs with CreateRedundantCoordinateMBS or CreateKinematicTree, see issue
    - date resolved: **2022-06-01 20:38**\ , date raised: 2020-12-16 
 * Version 1.2.110: resolved Issue 1121: AnimateSolution (change)
    - description:  mark as deprecated, use SolutionViewer instead
    - date resolved: **2022-06-01 20:25**\ , date raised: 2022-05-31 
 * Version 1.2.109: :textred:`resolved BUG 1122` : GraphicsDataBasis 
    - description:  kwargs argument radius not working
    - date resolved: **2022-05-31 15:32**\ , date raised: 2022-05-31 
 * Version 1.2.108: resolved Issue 1108: KinematicTree (check)
    - description:  check OutputVariable functions and compare with redundant-MBS based bodies
    - date resolved: **2022-05-30 21:06**\ , date raised: 2022-05-27 
 * Version 1.2.107: resolved Issue 1095: hydraulics (example)
    - description:  add hydraulic actuator with user function
    - **notes:** example HydraulicsUserFunction.py added already 5 days earlier
    - date resolved: **2022-05-30 20:06**\ , date raised: 2022-05-22 
 * Version 1.2.106: resolved Issue 1109: class Robot (extension)
    - description:  CreateRedundantCoordinateMBS: jointLoadUserFunctionList and createJointTorqueLoads marked as deprecated; use NEW jointSpringDamperUserFunctionList which allows to directly actuate at revolute/prismatic joints
    - date resolved: **2022-05-30 20:05**\ , date raised: 2022-05-29 
 * Version 1.2.105: :textred:`resolved BUG 1120` : KinematicTree 
    - description:  composite inertia contains wrong index in calculation
    - date resolved: **2022-05-30 20:04**\ , date raised: 2022-05-30 
 * Version 1.2.104: resolved Issue 1113: KinematicTree (testing)
    - description:  test for prismatic joint
    - date resolved: **2022-05-30 20:04**\ , date raised: 2022-05-29 
 * Version 1.2.103: :textred:`resolved BUG 1119` : KinematicTree 
    - description:  wrong signs (missing inverse) of prismatic joints in C++ and Python implementation of KinematicTree
    - date resolved: **2022-05-30 17:55**\ , date raised: 2022-05-30 
 * Version 1.2.102: :textred:`resolved BUG 1118` : ObjectKinematicTree 
    - description:  conversion to dict in mbs.GetObject(...) does not work for Matrix3D
    - date resolved: **2022-05-30 16:02**\ , date raised: 2022-05-30 
 * Version 1.2.101: resolved Issue 1117: Python3.8 (change)
    - description:  the exudyn version for Python3.8 now includes range checks (and is slower than before); for fast version of exudyn without range checks, check issues 1116 and section 'Performance and ways to speed up computations' in theDoc
    - date resolved: **2022-05-30 00:34**\ , date raised: 2022-05-30 
 * Version 1.2.100: resolved Issue 1116: exudynFast (extension)
    - description:  add separate track for fast exudyn versions; the compiler flag _FAST_EXUDYN_LINALG, previously only activated in Python 3.8, is now used for Python 3.7 and Python 3.8 versions ONLY if according import flags are set by doing the following 3 steps: import sys; sys.exudynFast=True; import exudyn
    - date resolved: **2022-05-30 00:34**\ , date raised: 2022-05-29 
 * Version 1.2.99: resolved Issue 1112: class Robot (change)
    - description:  CreateKinematicTree: jointForceVector, jointPositionOffsetVector, jointVelocityOffsetVector, jointPControlVector, jointDControlVector removed and replaced by PDcontrol structure in  RobotLink; forceUserFunction kept as is; return value slightly changed
    - date resolved: **2022-05-29 19:10**\ , date raised: 2022-05-29 
 * Version 1.2.98: resolved Issue 1111: class Robot (extension)
    - description:  RobotLink adds feature to define PDcontrol, used in robots for joint control of link
    - date resolved: **2022-05-29 19:10**\ , date raised: 2022-05-29 
 * Version 1.2.97: resolved Issue 1110: class Robot (extension)
    - description:  CreateRedundantCoordinateMBS: returns additional list springDamperList, which contains more efficient spring dampers for joint control
    - date resolved: **2022-05-29 17:05**\ , date raised: 2022-05-29 
 * Version 1.2.96: resolved Issue 1070: KinematicTree (extension)
    - description:  add SensorSuperElement (BUT call it SensorKinematicTree!) functionality for Position, Velocity, Acceleration, RotationMatrix, AngularVelocity, ...
    - **notes:** created new SensorKinematicTree; tests not done yet
    - date resolved: **2022-05-27 23:43**\ , date raised: 2022-05-05 
 * Version 1.2.95: resolved Issue 1107: Sensors (change)
    - description:  C++: move sensor-specific consistency tests from CSystem to CheckPreAssembleConsistency
    - date resolved: **2022-05-27 23:41**\ , date raised: 2022-05-26 
 * Version 1.2.94: resolved Issue 1103: Object (extension)
    - description:  C++: enable objects to be mixed of ODE2, ODE1 and AE variables (most general case); add jacobian_ODE1ODE2 flags and functions in case of ODE1
    - **notes:** NumericalJacobianODE1RHS modified accordingly
    - date resolved: **2022-05-23 23:06**\ , date raised: 2022-05-23 
 * Version 1.2.93: resolved Issue 1101: HydraulicActuator (example)
    - description:  add HydraulicActuator with user function example
    - **notes:** Duplicate of issue 1102
    - date resolved: **2022-05-23 22:10**\ , date raised: 2022-05-23 
 * Version 1.2.92: resolved Issue 1105: NodeGenericAE (extension)
    - description:  add node with algebraic variables
    - date resolved: **2022-05-23 22:02**\ , date raised: 2022-05-23 
 * Version 1.2.91: resolved Issue 0850: GetNodeODE1Index, GetNodeAEIndex (extension)
    - description:  add missing access functions
    - date resolved: **2022-05-23 21:48**\ , date raised: 2022-01-08 
 * Version 1.2.90: resolved Issue 1102: HydraulicActuator (example)
    - description:  add HydraulicActuator with user function example
    - **notes:** Examples/HydraulicsUserFunction.py
    - date resolved: **2022-05-23 19:53**\ , date raised: 2022-05-23 
 * Version 1.2.89: resolved Issue 1098: GraphicsDataCylinder (extension)
    - description:  add option addEdges which in addition returns GraphicsData for edges; returns then list of dictionaries
    - date resolved: **2022-05-22 20:54**\ , date raised: 2022-05-22 
 * Version 1.2.88: resolved Issue 1097: GraphicsDataOrthoCubePoint (extension)
    - description:  add option addEdges which in addition returns GraphicsData for edges; returns then list of dictionaries
    - date resolved: **2022-05-22 20:54**\ , date raised: 2022-05-22 
 * Version 1.2.87: :textred:`resolved BUG 1093` : ANCFBeam3D 
    - description:  giving wrong results because of error in template<class TMatrix> ConstSizeMatrixBase& operator+= (const TMatrix& matrix); leading to twice the mass matrix
    - date resolved: **2022-05-20 16:07**\ , date raised: 2022-05-20 
 * Version 1.2.86: resolved Issue 1092: openAI gym (extension)
    - description:  add example for interface with openAI gym using cart-pole model
    - **notes:** see Examples/testGymCartpole.py
    - date resolved: **2022-05-18 09:47**\ , date raised: 2022-05-18 
 * Version 1.2.85: :textred:`resolved BUG 1091` : ComputeODE2Eigenvalues 
    - description:  eigenvectors sorting is not according to eigenvalues sorting
    - date resolved: **2022-05-17 10:00**\ , date raised: 2022-05-17 
 * Version 1.2.84: resolved Issue 1090: ComputeODE2Eigenvalues (extension)
    - description:  solver.ComputeODE2Eigenvalues gets additional flag constrainedCoordinates to specify list of constrained coordinates in system for eigenvalue computation
    - date resolved: **2022-05-16 23:01**\ , date raised: 2022-05-16 
 * Version 1.2.83: :textred:`resolved BUG 1089` : ComputeODE2Eigenvalues 
    - description:  setInitialValues has no effect
    - **notes:** flag erased
    - date resolved: **2022-05-16 22:03**\ , date raised: 2022-05-16 
 * Version 1.2.82: :textred:`resolved BUG 1086` : ParameterVariation 
    - description:  numberOfThreads cannot be passed to parameter variation as it is duplicated by \*\*args
    - **notes:** numberOfThreads is now a named argument
    - date resolved: **2022-05-16 16:34**\ , date raised: 2022-05-16 
 * Version 1.2.81: resolved Issue 1079: Beam drawing (extension)
    - description:  add generic drawing function for beams, especially for contour plot
    - date resolved: **2022-05-15 23:24**\ , date raised: 2022-05-09 
 * Version 1.2.80: resolved Issue 1076: ANCFBeam3D (extension)
    - description:  add shear deformable 3D ANCF beam according to Nachbagauer Gerstmayr Pechstein using structural mechanics formulation
    - date resolved: **2022-05-15 23:24**\ , date raised: 2022-05-06 
 * Version 1.2.79: resolved Issue 1081: linux testSuite (check)
    - description:  check failed tests under linux; see tests for 1.2.75 linux
    - **notes:** resolved issues related to initial accelerations; still deviation in test generalContactFrictionTests.py
    - date resolved: **2022-05-11 18:54**\ , date raised: 2022-05-11 
 * Version 1.2.78: :textred:`resolved BUG 1084` : node frame drawing 
    - description:  node frames shows letter N unintended
    - date resolved: **2022-05-11 18:23**\ , date raised: 2022-05-11 
 * Version 1.2.77: :textred:`resolved BUG 1083` : initial accelerations 
    - description:  not correctly handled in linux build (and probably also on MacOS
    - date resolved: **2022-05-11 16:18**\ , date raised: 2022-05-11 
 * Version 1.2.76: resolved Issue 1082: testsuite linux (extension)
    - description:  extend test suite to run always for linux and add file ending for linux and MacOS
    - date resolved: **2022-05-11 12:51**\ , date raised: 2022-05-11 
 * Version 1.2.75: :textred:`resolved BUG 1080` : gcc compile 
    - description:  errors when compiling PyVectorLists and PyMatrixLists on gcc
    - date resolved: **2022-05-11 09:27**\ , date raised: 2022-05-11 
 * Version 1.2.74: resolved Issue 0471: geometrically exact beam3D (extension)
    - description:  add to CPP
    - **notes:** added first version based on conventional parameterization of nodes, but using Lie groups for beam section forces in ode2LHS
    - date resolved: **2022-05-09 21:24**\ , date raised: 2020-11-21 
 * Version 1.2.73: resolved Issue 1077: TExpSO3(Omega) (change)
    - description:  reduce number of terms in termExpanded; only up to x\*\*4 needed for 16 digits
    - date resolved: **2022-05-09 21:22**\ , date raised: 2022-05-07 
 * Version 1.2.72: resolved Issue 1074: BeamSection (extension)
    - description:  add new structure (cpp+Python) for beam cross section as well as BeamSectionGeometry for geometrical representation
    - date resolved: **2022-05-09 21:22**\ , date raised: 2022-05-06 
 * Version 1.2.71: resolved Issue 1073: structures description (extension)
    - description:  add description for simulationSettings, visualizationSettings, solver structures, etc. in Python bindings
    - date resolved: **2022-05-06 13:54**\ , date raised: 2022-05-06 
 * Version 1.2.70: resolved Issue 1069: KinematicTree (extension)
    - description:  add interface to class Robot, to create either redundant or minimal coordinates kinematic tree
    - **notes:** however, is not capable of creating robots with localHT!=HT0()
    - date resolved: **2022-05-06 00:07**\ , date raised: 2022-05-05 
 * Version 1.2.69: resolved Issue 1064: KinematicTree (docu)
    - description:  add basic description (equations)
    - date resolved: **2022-05-05 22:14**\ , date raised: 2022-05-05 
 * Version 1.2.68: resolved Issue 1067: KinematicTree (extension)
    - description:  add standalone example
    - **notes:** see TestModels/kinematicTreeTest.py
    - date resolved: **2022-05-05 20:00**\ , date raised: 2022-05-05 
 * Version 1.2.67: resolved Issue 1065: KinematicTree (docu)
    - description:  add user function description
    - date resolved: **2022-05-05 20:00**\ , date raised: 2022-05-05 
 * Version 1.2.66: resolved Issue 1068: KinematicTree (extension)
    - description:  add test
    - **notes:** added test and MiniExample
    - date resolved: **2022-05-05 19:59**\ , date raised: 2022-05-05 
 * Version 1.2.65: :textred:`resolved BUG 1063` : KinematicTree 
    - description:  cpp version gives wrong results as compared to MBS
    - **notes:** corrected inertia - must be w.r.t. COM
    - date resolved: **2022-05-05 16:14**\ , date raised: 2022-05-05 
 * Version 1.2.64: resolved Issue 1066: BodyGraphicsDataList (extension)
    - description:  new interface used in Kinematic tree which holds a list of GraphicsData lists
    - date resolved: **2022-05-05 11:32**\ , date raised: 2022-05-05 
 * Version 1.2.63: resolved Issue 0655: add KinematicTree (extension)
    - description:  ObjectKinematicTree (minimal coordinates)
    - **notes:** this is a first version, but tests and validation are necessary!
    - date resolved: **2022-05-05 10:27**\ , date raised: 2021-05-01 
 * Version 1.2.62: resolved Issue 0913: KinematicTree (extension)
    - description:  add efficient C++ functionality for operating on 6D vectors and matrices for kinematics/dynamics
    - date resolved: **2022-05-05 10:23**\ , date raised: 2022-02-02 
 * Version 1.2.61: resolved Issue 1062: RigidBodyMath.h (change)
    - description:  remove duplicate from src/Utilities
    - date resolved: **2022-05-03 11:33**\ , date raised: 2022-05-03 
 * Version 1.2.60: resolved Issue 1047: Matrix3DList (extension)
    - description:  add new structure Matrix3DList; used to create list of 3D matrices and transfer into MainSystem mbs, e.g. in KinematicTree
    - date resolved: **2022-05-01 00:47**\ , date raised: 2022-04-25 
 * Version 1.2.59: resolved Issue 1038: simulateInRealtime (extension)
    - description:  put waitMicroSeconds into settings
    - date resolved: **2022-04-30 20:41**\ , date raised: 2022-04-07 
 * Version 1.2.58: resolved Issue 1049: MacOS (check)
    - description:  check if flag -mmacosx-version-min=11.0 is needed in setup.py and if this causes problems on older pre-AppleM1 which needs wheel version 10.9
    - **notes:** x86 compilation requires 11.0, otherwise it fails; need to find another way for compilation
    - date resolved: **2022-04-29 09:59**\ , date raised: 2022-04-25 
 * Version 1.2.57: resolved Issue 1060: buildDate.tex (change)
    - description:  remove by default change of buildDate.tex in setup.py
    - date resolved: **2022-04-29 08:20**\ , date raised: 2022-04-28 
 * Version 1.2.56: :textred:`resolved BUG 1059` : MacOS compile 
    - description:  sse2neon.h not found
    - **notes:** exclude parallel threads from MacOS version until resolving compilation problem
    - date resolved: **2022-04-28 11:58**\ , date raised: 2022-04-28 
 * Version 1.2.55: resolved Issue 0905: Add description of Pybind11 interaction (docu)
    - description:  add some description of C++-Python interaction and entry points into C++
    - date resolved: **2022-04-27 20:37**\ , date raised: 2022-02-01 
 * Version 1.2.54: resolved Issue 1015: TestSuite (docu)
    - description:  add notes in Exudyn on testsuite (add remark: in many cases for tracking of changes, not for validation)
    - date resolved: **2022-04-27 20:21**\ , date raised: 2022-03-28 
 * Version 1.2.53: resolved Issue 1058: ClearWorkspace (change)
    - description:  also close open matplotlib figures before they are lost
    - date resolved: **2022-04-27 19:47**\ , date raised: 2022-04-27 
 * Version 1.2.52: :textred:`resolved BUG 1052` : ClearWorkspace 
    - description:  has no effect on global variables
    - **notes:** NOTE: matplotlib looses figures which cannot be closed with closeAll in PlotSensor
    - date resolved: **2022-04-27 19:03**\ , date raised: 2022-04-25 
 * Version 1.2.51: :textred:`resolved BUG 1053` : coordinatesSolution 
    - description:  coordinatesSolutionFile is corrupted for ACF test in text mode
    - **notes:** not repeatable; may have been caused by iPython crash
    - date resolved: **2022-04-27 10:29**\ , date raised: 2022-04-26 
 * Version 1.2.50: resolved Issue 1051: CreateNonlinearFEMObjectGenericODE2NGsolve (testing)
    - description:  test FEM.CreateNonlinearFEMObjectGenericODE2NGsolve with simple example
    - **notes:** included in ACFtest.py
    - date resolved: **2022-04-27 10:23**\ , date raised: 2022-04-25 
 * Version 1.2.49: resolved Issue 1057: solutionInformation (extension)
    - description:  replace new lines in solutionInformation when writing into coordinatesSolution file in order to preserve readable file structure
    - date resolved: **2022-04-27 10:22**\ , date raised: 2022-04-27 
 * Version 1.2.48: resolved Issue 1056: MarkerSuperElementRigid (extension)
    - description:  add check for node sizes used in marker
    - date resolved: **2022-04-26 22:29**\ , date raised: 2022-04-26 
 * Version 1.2.47: :textred:`resolved BUG 1055` : NodeGenericODE2 error 
    - description:  NodeGenericODE2 raises system error CNodeODE2::GetVelocity: call illegal during solver initialize
    - **notes:** added GetVelocity and GetAcceleration to NodeGenericODE2, with risk that these functions are used unintended
    - date resolved: **2022-04-26 22:29**\ , date raised: 2022-04-26 
 * Version 1.2.46: resolved Issue 1054: NodeIndex, ... (extension)
    - description:  add possibility to allow simple arithmetic operations +,-,\*,-() for NodeIndex, MarkerIndex, etc.
    - date resolved: **2022-04-26 22:29**\ , date raised: 2022-04-26 
 * Version 1.2.45: :textred:`resolved BUG 1050` : CreateLinearFEMObjectGenericODE2 
    - description:  FEM.CreateLinearFEMObjectGenericODE2 not working
    - date resolved: **2022-04-25 17:34**\ , date raised: 2022-04-25 
 * Version 1.2.44: resolved Issue 1046: Vector3DList (extension)
    - description:  add new structure Vector3DList; used to create list of 3D vectors and transfer into MainSystem mbs, e.g. in KinematicTree
    - date resolved: **2022-04-25 08:40**\ , date raised: 2022-04-25 
 * Version 1.2.43: resolved Issue 1045: dispy cluster (extension)
    - description:  fixed some bugs in ProcessParameterList (http_server) ; added some checks (if dispy is installed); cluster is used now iff clusterHostNames != []) and useMultiProcessing==True
    - date resolved: **2022-04-23 17:09**\ , date raised: 2022-04-23 
 * Version 1.2.42: :textred:`resolved BUG 1044` : GetNodesOnLine 
    - description:  FEM.GetNodesOnLine raises exception
    - date resolved: **2022-04-22 18:28**\ , date raised: 2022-04-22 
 * Version 1.2.41: :textred:`resolved BUG 1043` : serialRobotTest.py 
    - description:  class robotics.Robot computes wrong static torque compensation in function StaticTorques(HT)
    - **notes:** due to #0744 this bug has been magically resolved!
    - date resolved: **2022-04-21 15:15**\ , date raised: 2022-04-21 
 * Version 1.2.40: resolved Issue 1042: multithreaded compilation (extension)
    - description:  enable setup.py with multithreaded compilation, using Monkey patch for compiler with multithreading library; enable parallel build with: python setup.py install --parallel
    - date resolved: **2022-04-21 02:00**\ , date raised: 2022-04-21 
 * Version 1.2.39: resolved Issue 0744: CreateRedundantCoordinateMBS (change)
    - description:  interchange joint markers, affecting the sign of the measured joint rotation angle, being now consistent with minimal coordinate formulations / kinematicTree; also interchange jointTorque0List/1List to be consistent with marker list 
    - **notes:** adapted SerialRobotTestDH2.py and SerialRobotTSD.py in Example folders for corrected signs
    - date resolved: **2022-04-20 00:37**\ , date raised: 2021-09-01 
 * Version 1.2.38: resolved Issue 1041: Pluecker transforms (change)
    - description:  correct Pluecker transform T66 functions, such as RotationTranslation2T66, etc. and introduce inverse functions for usage in Featherstone algorithm
    - **notes:** note that rotations in RotationX2T66, RotationY2T66, RotationZ2T66 are transposed/CHANGED as compared to previous version; T66toRotationTranslation is corrected/CHANGED and now is consistent with the backtransformation; InverseT66toRotationTranslation does the inverse operation (but also for rotation); RotationTranslation2T66 corrected/CHANGED; HT2T66 is corrected/CHANGED with inverse version HT2T66Inverse; rotations in Exudyn now consistent between T66, HT and other rotation matrices
    - date resolved: **2022-04-19 16:16**\ , date raised: 2022-04-19 
 * Version 1.2.37: resolved Issue 1021: twitter (extension)
    - description:  add twitter account for exudyn: https://twitter.com/RExudyn
    - **notes:** Follow me!
    - date resolved: **2022-04-12 16:28**\ , date raised: 2022-03-31 
 * Version 1.2.36: resolved Issue 1040: contour bodies (extension)
    - description:  show contour colors for GraphicsData added to bodies
    - **notes:** option turned on by default, but may slow down visualization for larger models; turn off in case of problems...
    - date resolved: **2022-04-10 17:21**\ , date raised: 2022-04-10 
 * Version 1.2.35: resolved Issue 1039: mesh edges (extension)
    - description:  show mesh edges independently of visualization mode (edges on/off) - allowing to show mesh edges but not other graphics edges
    - date resolved: **2022-04-10 11:42**\ , date raised: 2022-04-10 
 * Version 1.2.34: resolved Issue 1036: solution file (change)
    - description:  fixed inconsistent information in line 3 of coordinatesSolutionFile: ODE2 coordinates
    - date resolved: **2022-04-07 20:30**\ , date raised: 2022-04-07 
 * Version 1.2.33: resolved Issue 1034: solution and sensor files (extension)
    - description:  add Python version to e.g. Exudyn version = 1.x.y Python3.9 / Python3.6(32bits) in coordinatesSolutionFile and sensor output files to identify exactly the versions and platform used for computation; check also Parameter and optimization output files for updated information
    - date resolved: **2022-04-07 12:02**\ , date raised: 2022-04-07 
 * Version 1.2.32: resolved Issue 1033: InteractiveDialog (extenison)
    - description:  also add tkInter DoubleVar for sliders to make bi-directional interaction simpler (but not needed)
    - date resolved: **2022-04-04 21:26**\ , date raised: 2022-04-04 
 * Version 1.2.31: resolved Issue 1031: InteractiveDialog (extension)
    - description:  extended with option addLabelStringVariables to be able to modify strings in text labels
    - date resolved: **2022-04-04 17:20**\ , date raised: 2022-04-04 
 * Version 1.2.30: resolved Issue 1030: AVX2 (change)
    - description:  change both Python 3.6 32bit/46bit versions to compilation without AVX
    - date resolved: **2022-04-04 15:39**\ , date raised: 2022-04-04 
 * Version 1.2.29: resolved Issue 1024: PERFORM_UNIT_TESTS (change)
    - description:  set PERFORM_UNIT_TESTS for P3.7 instead of P3.6
    - date resolved: **2022-04-04 15:38**\ , date raised: 2022-03-31 
 * Version 1.2.28: :textred:`resolved BUG 1029` : ContactFrictionCircleCable2D 
    - description:  computes wrong segment length internally, leading to wrong sticking position
    - date resolved: **2022-04-04 12:14**\ , date raised: 2022-04-04 
 * Version 1.2.27: :textred:`resolved BUG 1028` : cnt in Render window 
    - description:  debug information cnt=.. shown in Render window
    - date resolved: **2022-04-04 11:50**\ , date raised: 2022-04-04 
 * Version 1.2.26: resolved Issue 1027: RotationMatrix2EulerParameters (change)
    - description:  RotationMatrix2EulerParameters computes Euler parameters with large deviations from unit norm in case of inaccurate rotation matrices; this may lead to failure of CheckPreAssembleConsistencies; resolved by adding normalization before returning Euler parameters
    - date resolved: **2022-04-03 00:24**\ , date raised: 2022-04-03 
 * Version 1.2.25: resolved Issue 1025: build venv (extension)
    - description:  switch to building windows purely on virtual conda environments, allowing to build all windows and linux versions in parallel
    - date resolved: **2022-04-02 00:28**\ , date raised: 2022-04-02 
 * Version 1.2.24: resolved Issue 1022: virtual environments (change)
    - description:  switch to virtual environments in anaconda for compilation on different platforms
    - date resolved: **2022-04-02 00:28**\ , date raised: 2022-03-31 
 * Version 1.2.23: resolved Issue 1023: remove /Zi in MSVC (change)
    - description:  remove compilation flag /Zi in setup.py which prevents from parallel runs of MSVC cl.exe
    - date resolved: **2022-03-31 11:28**\ , date raised: 2022-03-31 
 * Version 1.2.22: resolved Issue 1020: robotics links (docu)
    - description:  links to github are not resolved correctly for robotics module
    - date resolved: **2022-03-30 10:47**\ , date raised: 2022-03-30 
 * Version 1.2.21: resolved Issue 1019: SpaceMouse (extension)
    - description:  add functionality for 3D mouse / spacemouse by reading joystick inputs and interpret as 3D position and rotation data; add visualization flag interactive.useJoystickInput (default=True); deactivate this flag if your external device makes problems
    - date resolved: **2022-03-29 14:37**\ , date raised: 2022-03-29 
 * Version 1.2.20: resolved Issue 1017: ContactFrictionCircleCable2D (testing)
    - description:  check and test computation of sticking position segment length: undeformed versus deformed
    - **notes:** switched to reference length in computation of relative sticking position as given in theDoc; leads to improved results
    - date resolved: **2022-03-28 20:33**\ , date raised: 2022-03-28 
 * Version 1.2.19: resolved Issue 1014: SaveImage (change)
    - description:  make window height divisible by 2 (skip one line if necessary)
    - **notes:** added alignment option for width and height; default options work well for ffmpeg conversion
    - date resolved: **2022-03-28 14:25**\ , date raised: 2022-03-28 
 * Version 1.2.18: resolved Issue 1013: TGA output (change)
    - description:  switch from TGA to .png output using stb_image_write.h in GLFW
    - **notes:** added mode exportImages.saveImageFormat to chose between PNG and TGA - PNG with much smaller files!
    - date resolved: **2022-03-28 14:24**\ , date raised: 2022-03-28 
 * Version 1.2.17: resolved Issue 1012: GLFW (change)
    - description:  switch to GLFW3.3.6 includes in order to be in line with AppleM1 version
    - date resolved: **2022-03-28 12:03**\ , date raised: 2022-03-28 
 * Version 1.2.16: resolved Issue 1011: Apple M1 (change)
    - description:  use universal glfw libs for both Apple x86 and Apple arm M1
    - date resolved: **2022-03-28 12:03**\ , date raised: 2022-03-28 
 * Version 1.2.15: resolved Issue 1010: autodiff (change)
    - description:  extract autodiff as separate module
    - **notes:** now using AutomaticDifferentiation.h in Utilities
    - date resolved: **2022-03-28 11:56**\ , date raised: 2022-03-28 
 * Version 1.2.14: :textred:`resolved BUG 1009` : solutionViewer 
    - description:  does not load automatically due to change of coordinatesSolution filename ending
    - **notes:** changed loading of default file in SolutionViewer
    - date resolved: **2022-03-28 09:11**\ , date raised: 2022-03-28 
 * Version 1.2.13: :textred:`resolved BUG 1007` : sensor double values 
    - description:  sensor outputs two times for a single time step
    - **notes:** error occured due to automaticStepSize activated and call to ReduceStepSize even in case adaptiveStep=0; automaticStepSize now deactivated for solvers without step size control
    - date resolved: **2022-03-27 19:31**\ , date raised: 2022-03-24 
 * Version 1.2.12: resolved Issue 1006: ContactFrictionCircleCable2D (change)
    - description:  LHS computation: exclude undefined state from sticking position computation
    - date resolved: **2022-03-22 12:43**\ , date raised: 2022-03-22 
 * Version 1.2.11: resolved Issue 1005: PostNewton timer (change)
    - description:  remove timer from timer structures and activate as special timer
    - date resolved: **2022-03-22 12:30**\ , date raised: 2022-03-22 
 * Version 1.2.10: resolved Issue 1004: ContactFrictionCircleCable2D (extension)
    - description:  add option usePointWiseNormals flag as an additional option to control the way forces are applied to cable
    - date resolved: **2022-03-21 10:12**\ , date raised: 2022-03-21 
 * Version 1.2.9: resolved Issue 1003: setup.py (extension)
    - description:  include manifest.in and readme.md in main
    - date resolved: **2022-03-20 11:47**\ , date raised: 2022-03-20 
 * Version 1.2.8: resolved Issue 1002: pypi problems (change)
    - description:  previous version marked beta
    - date resolved: **2022-03-19 10:54**\ , date raised: 2022-03-19 
 * Version 1.2.7: resolved Issue 0986: ANCFCable2D (docu)
    - description:  extend/finalize description - specifically for integration points and OutputVariable functions
    - date resolved: **2022-03-18 23:09**\ , date raised: 2022-03-15 
 * Version 1.2.6: resolved Issue 1001: pypi (change)
    - description:  finalized conversion of markup file for description at pypi.org: use only links to github, as pypi does not recognize .rst file in github format
    - date resolved: **2022-03-18 20:54**\ , date raised: 2022-03-18 
 * Version 1.2.5: resolved Issue 1000: adjust version in docu (docu)
    - description:  recompile
    - date resolved: **2022-03-18 19:07**\ , date raised: 2022-03-18 
 * Version 1.2.4: resolved Issue 0999: pypi (extension)
    - description:  improved versioning
    - date resolved: **2022-03-18 18:28**\ , date raised: 2022-03-18 
 * Version 1.2.3: resolved Issue 0998: pypi (extension)
    - description:  add description and tags
    - date resolved: **2022-03-18 18:09**\ , date raised: 2022-03-18 
 * Version 1.2.2: resolved Issue 0997: pre-release version (extension)
    - description:  add ".dev" tag in version and wheel name for pre-releases, allowing to distinguish on pypi for different versions
    - date resolved: **2022-03-18 17:02**\ , date raised: 2022-03-18 
 * Version 1.2.1: resolved Issue 0996: pre-release (extension)
    - description:  allow pre-releases to be uploaded on pypi, fetched with "pip install exudyn --pre"
    - date resolved: **2022-03-18 15:52**\ , date raised: 2022-03-18 
 * Version 1.2.0: resolved Issue 0995: add to pypi index (extension)
    - description:  add exudyn to pypi index; allows to use "pip install exudyn"
    - date resolved: **2022-03-18 10:44**\ , date raised: 2022-03-18 

***********
Version 1.1
***********

 * Version 1.1.177: :textred:`resolved BUG 0994` : ObjectContactFrictionCircleCable2D 
    - description:  forces on circle added up twice, because weighting factor not considered
    - **notes:** tested force on circle with static computation in belt drive
    - date resolved: **2022-03-18 08:30**\ , date raised: 2022-03-18 
 * Version 1.1.176: resolved Issue 0880: User function for ConnectorCoordinateVector (extension)
    - description:  add user function for constraint and for jacobian; test with double pendulum made of masses
    - date resolved: **2022-03-17 18:14**\ , date raised: 2022-01-24 
 * Version 1.1.175: :textred:`resolved BUG 0993` : ObjectConnectorDistance 
    - description:  activeConnector=False produces wrong jacobian
    - date resolved: **2022-03-17 17:52**\ , date raised: 2022-03-17 
 * Version 1.1.174: resolved Issue 0992: PlotSensor (extension)
    - description:  add PlotSensorDefaults function which allows to set default values for all subsequent PlotSensor calls
    - **notes:** use e.g. PlotSensorDefaults().fontSize=16 to change fontSize for all subsequent calls
    - date resolved: **2022-03-17 17:28**\ , date raised: 2022-03-17 
 * Version 1.1.173: resolved Issue 0991: velocityOffset (extension)
    - description:  add velocity offset for SpringDamper and TorsionalSpringDamper, allowing simple controllers using offset and velocityOffset in preStepUser functions
    - date resolved: **2022-03-16 23:51**\ , date raised: 2022-03-16 
 * Version 1.1.172: resolved Issue 0989: convergence problems (docu)
    - description:  add specific section in theDoc - Exudyn Basics related to ways for resolving convergence problems
    - date resolved: **2022-03-16 18:59**\ , date raised: 2022-03-16 
 * Version 1.1.171: resolved Issue 0938: ANCFCable2D (extension)
    - description:  add drawing function for forces in normal direction with factor
    - date resolved: **2022-03-15 20:33**\ , date raised: 2022-02-10 
 * Version 1.1.170: resolved Issue 0933: RigidBody (extension)
    - description:  add accelerationLocal and angularAccelerationLocal output
    - **notes:** added to ObjectRigidBody and ObjectRigidBody2D
    - date resolved: **2022-03-15 20:14**\ , date raised: 2022-02-07 
 * Version 1.1.169: resolved Issue 0967: ANCFCable2D (extension)
    - description:  add OutputVariables RotationMatrix, Rotation, AngularVelocity(Local) and AngularAcceleration
    - **notes:** Acceleration, AngularVelocity and AngularAcceleration currently only implemented for ANCFCable2D but not for ALEANCFCable2D
    - date resolved: **2022-03-15 20:01**\ , date raised: 2022-03-03 
 * Version 1.1.168: resolved Issue 0966: ANCFCable2D (extension)
    - description:  add missing terms for OutputVariables and AccessFunctions to allow constraints that are at local position y!=0
    - date resolved: **2022-03-15 19:54**\ , date raised: 2022-03-03 
 * Version 1.1.167: resolved Issue 0983: plot (extension)
    - description:  add function to create plot-ready data from mbs.GetObjectOutputBody for a list of consecutive beams, e.g., axial force, displacement or curvature along axial reference coordinate
    - **notes:** added function DataArrayFromSensorList which allows to create data from a list of sensors
    - date resolved: **2022-03-14 20:41**\ , date raised: 2022-03-11 
 * Version 1.1.166: resolved Issue 0982: PlotSensor (extentsion)
    - description:  add option to plot 2D arrays
    - **notes:** numpy arrays are used instead of sensorNumbers; these arrays must have the same format as data stored in sensor files; this data format does not create any labels
    - date resolved: **2022-03-14 16:24**\ , date raised: 2022-03-11 
 * Version 1.1.165: resolved Issue 0985: startOfStepState (extension)
    - description:  initialize startOfStepState together with currentState in InitializeSolverInitialConditions(...) in order to be valid when sensors are written in initialization
    - date resolved: **2022-03-14 13:07**\ , date raised: 2022-03-14 
 * Version 1.1.164: resolved Issue 0981: ObjectContactFrictionCircleCable2D (extension)
    - description:  add OutputVariable functions for Coordinates (gap, slip), Coordinates_t (gap_t, slip_t), and contact and friction forces per segment (ForceLocal)
    - date resolved: **2022-03-14 12:58**\ , date raised: 2022-03-11 
 * Version 1.1.163: resolved Issue 0980: renderer precision (extension)
    - description:  add options for general precision in renderer: general.rendererPrecision as well as precision for colorbars in contour: contour.colorBarPrecision
    - date resolved: **2022-03-11 14:05**\ , date raised: 2022-03-11 
 * Version 1.1.162: resolved Issue 0979: VisualizationSettings (extension)
    - description:  VisualizationSettings.showContactForcesValues is added to show numerical values for contact forces
    - date resolved: **2022-03-11 10:19**\ , date raised: 2022-03-11 
 * Version 1.1.161: resolved Issue 0978: ObjectANCFCable2D (extension)
    - description:  add improved axial strain computation for reduced order integration
    - **notes:** works for axial strain and axial force if reducedAxialInterploation=True
    - date resolved: **2022-03-10 20:36**\ , date raised: 2022-03-10 
 * Version 1.1.160: resolved Issue 0977: ObjectANCFCable2D (extension)
    - description:  add new mode useReducedOrderIntegration=2 with good performance/accuracy and exceptional representation of axial strains
    - date resolved: **2022-03-10 20:08**\ , date raised: 2022-03-10 
 * Version 1.1.159: resolved Issue 0976: ContactCircleCable2D (extension)
    - description:  add visualization flag showContactCircle; uses circleTiling\*4 for tiling (from VisualizationSettings.general)!
    - date resolved: **2022-03-10 14:15**\ , date raised: 2022-03-10 
 * Version 1.1.158: resolved Issue 0975: VisualizationSettings (extension)
    - description:  added showContactForces and contactForcesFactor in contact; this flag is currently only available in ContactCircleCable2D
    - date resolved: **2022-03-10 13:52**\ , date raised: 2022-03-10 
 * Version 1.1.157: resolved Issue 0974: VisualizationSettings (change)
    - description:  moved contactPointsDefaultSize from connectors to contact; connectors.contactPointsDefaultSize is inactive from now!
    - date resolved: **2022-03-10 13:50**\ , date raised: 2022-03-10 
 * Version 1.1.156: resolved Issue 0953: ContactFrictionCircleCable2D (extension)
    - description:  add improved (static) friction model
    - **notes:** also added improved PostNewton switching strategies and changed initial values for NodeGenericData
    - date resolved: **2022-03-09 21:42**\ , date raised: 2022-02-25 
 * Version 1.1.155: resolved Issue 0932: ANCFCable2D (extension)
    - description:  add velocityLocal + accelerationLocal output in axial/normal direction
    - **notes:** only velocityLocal added
    - date resolved: **2022-03-06 18:24**\ , date raised: 2022-02-07 
 * Version 1.1.154: resolved Issue 0931: ANCFCable2D (extension)
    - description:  add acceleration as output variable
    - **notes:** only added for ANCF, but not for ALEANCF due to coupling terms with vALE
    - date resolved: **2022-03-06 18:24**\ , date raised: 2022-02-07 
 * Version 1.1.153: resolved Issue 0970: PlotSensor (extension)
    - description:  PlotSensor allows to set fileCommentChar and fileDelimiterChar
    - date resolved: **2022-03-04 13:27**\ , date raised: 2022-03-04 
 * Version 1.1.152: resolved Issue 0969: exudyn.plot (extension)
    - description:  added method to convert output files from other codes; in particular plot.FileStripSpaces(...) can be used to strip leading / trailing spaces and remove double spaces
    - date resolved: **2022-03-04 13:27**\ , date raised: 2022-03-04 
 * Version 1.1.151: resolved Issue 0965: ANCFCable2D (extension)
    - description:  add option for strainIsRelativeToReference, which if set to 1. accounts for the reference geometry as the stress-free configuration; also works for ALE Cable2D
    - date resolved: **2022-03-03 08:13**\ , date raised: 2022-03-03 
 * Version 1.1.150: resolved Issue 0964: CreateReevingCurve (change)
    - description:  corrected sign of returned curvatures, now can be directly used for beam elements
    - date resolved: **2022-03-02 17:20**\ , date raised: 2022-03-02 
 * Version 1.1.149: :textred:`resolved BUG 0963` : CreateReevingCurve 
    - description:  removeFirstLine=True not working: wrong case i==0 needs to be changed to i==1
    - date resolved: **2022-03-02 15:25**\ , date raised: 2022-03-02 
 * Version 1.1.148: resolved Issue 0962: CreateReevingCurve (change)
    - description:  adjust number of nodes in case of closed Curve (numberOfANCFnodes=20 gives 20 elements in this case)
    - date resolved: **2022-03-02 15:25**\ , date raised: 2022-03-02 
 * Version 1.1.147: resolved Issue 0961: stepInformation (change)
    - description:  changed flags in timeIntegration and staticSolver stepInformation: value of 1024 now causes output at every step; all values > 16 have been divided by two; 255=detailed overall output, 2047=detailed output every step; see SimulationSettings in theDoc
    - date resolved: **2022-03-02 10:33**\ , date raised: 2022-03-02 
 * Version 1.1.146: resolved Issue 0959: coordinatesSolution (change)
    - description:  change ending of coordinates solution files from .txt to .sol in case of binary files
    - date resolved: **2022-03-02 10:14**\ , date raised: 2022-03-02 
 * Version 1.1.145: resolved Issue 0960: ObjectContactFrictionCircleCable2D (change)
    - description:  move to ObjectContactFrictionCircleCable2DOld and improve new version
    - date resolved: **2022-03-02 09:50**\ , date raised: 2022-03-02 
 * Version 1.1.144: resolved Issue 0958: LoadForceVector (docu)
    - description:  add note to description in forces that user function values are available in sensors, but they are not updated during drawing
    - date resolved: **2022-03-01 19:40**\ , date raised: 2022-02-28 
 * Version 1.1.143: resolved Issue 0952: rolling joints (extension)
    - description:  ConnectorRollingDiscPenalty and JointRollingDisc now add a OutputVariable RotationMatrix, containing the J1 to global transformation
    - date resolved: **2022-03-01 19:35**\ , date raised: 2022-02-25 
 * Version 1.1.142: resolved Issue 0951: Friction (test)
    - description:  test LuGre friction model as ODE1 model versus position/history based model
    - **notes:** added lugreFrictionODE1.py as a demo showing the LuGre model based on a ODE1 user function
    - date resolved: **2022-03-01 19:34**\ , date raised: 2022-02-24 
 * Version 1.1.141: resolved Issue 0956: ProfileLinearAccelerationsList (extension)
    - description:  add linear acceleration profile to robotics.motion, currently only allowing to create profile directly from accelerations
    - date resolved: **2022-02-28 19:05**\ , date raised: 2022-02-28 
 * Version 1.1.140: resolved Issue 0955: robotics.motion (change)
    - description:  change PTPprofile to BasicProfile
    - date resolved: **2022-02-28 09:59**\ , date raised: 2022-02-28 
 * Version 1.1.139: :textred:`resolved BUG 0950` : JointRollingDisc 
    - description:  OutputVariable VelocityLocal is returned in global coordinates; description in theDoc is wrong; will be changed to outputVariable Velocity; local velocity will represent the slippage in special local joint J1 coordinates; see theDoc for updated functionality and outputs
    - date resolved: **2022-02-25 14:32**\ , date raised: 2022-02-22 
 * Version 1.1.138: :textred:`resolved BUG 0949` : ConnectorRollingDiscPenalty 
    - description:  OutputVariable VelocityLocal local is returned in global coordinates; description in theDoc is wrong; will be changed to outputVariable Velocity; see theDoc for updated functionality and outputs
    - date resolved: **2022-02-25 14:32**\ , date raised: 2022-02-22 
 * Version 1.1.137: resolved Issue 0946: sensors storeInternal (extension)
    - description:  adapt most TestModels to store sensordata internally
    - date resolved: **2022-02-21 00:43**\ , date raised: 2022-02-20 
 * Version 1.1.136: resolved Issue 0947: TestModels (change)
    - description:  change most test models to use Sensors storeInternal mode; this avoids creating many files during TestSuite runs
    - date resolved: **2022-02-20 20:40**\ , date raised: 2022-02-20 
 * Version 1.1.135: resolved Issue 0921: PlotSensor (extension)
    - description:  add option to add subplots
    - **notes:** allows to create subplots, adjusting also the plot size using sizeInches; for examples see Examples/plotSensorExamples.py
    - date resolved: **2022-02-19 22:45**\ , date raised: 2022-02-02 
 * Version 1.1.134: resolved Issue 0922: PlotSensor (extension)
    - description:  add option to add linewidth, markersize, markerStyles=["o",...], markerSizes=[], lineStyles=["-",...], colors=[..], markerDensity=...; lineWidths=[] allowing to plot a reduced number of markers on top of a line
    - **notes:** added many options for line and marker styles, check: colors, lineStyles, lineWidths, markerStyles, markerSizes, markerDensity
    - date resolved: **2022-02-19 22:41**\ , date raised: 2022-02-02 
 * Version 1.1.133: resolved Issue 0920: PlotSensor (extension)
    - description:  add x-range and y-range for zoom
    - **notes:** added rangeX and rangeY options to specify range
    - date resolved: **2022-02-19 16:59**\ , date raised: 2022-02-02 
 * Version 1.1.132: resolved Issue 0945: PlotSensor (extension)
    - description:  extend option for offsets, allowing sensor data to use as offset (e.g., loaded from file or from internal sensor data)
    - **notes:** see plotSensorExamples for some particular usage
    - date resolved: **2022-02-19 16:45**\ , date raised: 2022-02-19 
 * Version 1.1.131: resolved Issue 0944: PlotSensor (extension)
    - description:  add argument labels, which can be string (for one sensor) or list of strings (according to number of sensors resp. components) representing the labels used in legend; if not provided, automatically generated legend is used 
    - date resolved: **2022-02-19 15:47**\ , date raised: 2022-02-19 
 * Version 1.1.130: resolved Issue 0918: Sensors store values internally (extension)
    - description:  add option to store sensor values internally; using ResizableMatrix internally; rows added and matrix is automatically resized
    - **notes:** storeInternal boost the speed of writing sensor values, however, most of time is spent on computing sensor values, which typically about 2 seconds for 1e6 time steps per sensor; file write adds usually 3 seconds extra on that bill
    - date resolved: **2022-02-19 00:13**\ , date raised: 2022-02-02 
 * Version 1.1.129: resolved Issue 0943: mbs.GetSensorStoredData() (extension)
    - description:  add MainSystem functionality to retrieve internally stored data in sensor; used, e.g., for PlotSensor to plot data without storing in files
    - date resolved: **2022-02-19 00:10**\ , date raised: 2022-02-18 
 * Version 1.1.128: :textred:`resolved BUG 0942` : sensorsAppendToFile 
    - description:  appendToFile is used instead of sensorsAppendToFile for switching between append and replace operations for files
    - date resolved: **2022-02-18 20:54**\ , date raised: 2022-02-18 
 * Version 1.1.127: resolved Issue 0901: SimulationSettings (extension)
    - description:  improve type completion by adding py::init<...> functions for all subclasses
    - **notes:** not solvable in this simple way as type completion is not improved when adding this information
    - date resolved: **2022-02-18 20:30**\ , date raised: 2022-01-31 
 * Version 1.1.126: resolved Issue 0941: MotionInterpolator (extension)
    - description:  mark as deprecated; instead, created motion submodule in robotics with class Trajectory; uses classes ProfileConstantAcceleration and ProfilePTP to construct piecewise profiles for trajectory; precomputes acceleration profiles in first step and thus is several factors faster in Python implementation; Trajectory converts to dict and can be printed in order to obtain key values of computed trajectories
    - date resolved: **2022-02-18 16:31**\ , date raised: 2022-02-15 
 * Version 1.1.125: resolved Issue 0940: MotionInterpolator (extension)
    - description:  add option to add trajectories defined by duration as well as by maxVelocity and maxAcceleration using synchronous PTP trajectory generation
    - date resolved: **2022-02-15 12:38**\ , date raised: 2022-02-15 
 * Version 1.1.124: :textred:`resolved BUG 0939` : NodeRigidBody2D 
    - description:  does not correctly measure rotations (returns x-coordinate instead of angle)
    - date resolved: **2022-02-15 09:58**\ , date raised: 2022-02-15 
 * Version 1.1.123: resolved Issue 0745: SensitivityAnalysis() (extension)
    - description:  add functionality to processing, evaluating the sensitivities of certain sensor values w.r.t. parameters; same interface as ParameterVariation
    - **notes:** see processing.ComputeSensitivities(...)
    - date resolved: **2022-02-14 09:50**\ , date raised: 2021-09-03 
    - resolved by: P. Manzl
 * Version 1.1.122: :textred:`resolved BUG 0934` : exudyn.signal 
    - description:  conflicts with Python 3.8.8 and Python 3.9.7 (and possibly other) with internal Python signal package; change exudyn.signal to exudyn.signalProcessing
    - date resolved: **2022-02-10 09:23**\ , date raised: 2022-02-10 
 * Version 1.1.121: resolved Issue 0870: binary output (extension)
    - description:  add option to create binary solution files
    - **notes:** notes: use outputPrecision to switch between float and double - see there; speeds up file writing considerably and reduces file sizes; Integrated into LoadSolutionFile
    - date resolved: **2022-02-09 08:45**\ , date raised: 2022-01-18 
 * Version 1.1.120: resolved Issue 0930: LoadSolutionFile (extension)
    - description:  extend function to check if binary file; if yes, switches to binary mode
    - date resolved: **2022-02-09 08:44**\ , date raised: 2022-02-07 
 * Version 1.1.119: resolved Issue 0929: coordinatesSolution, sensors (change)
    - description:  add version to coordinatesSolutionFile and sensor output files; should not affect current parsing of output files
    - date resolved: **2022-02-07 00:02**\ , date raised: 2022-02-07 
 * Version 1.1.118: resolved Issue 0878: sort settings options (docu)
    - description:  sort settings representation in Python by sorting dictionaries prior to writing inteface files; keep current sorting (grouping) in latex documentation
    - **notes:** type completion may change sorting afterwards
    - date resolved: **2022-02-06 21:57**\ , date raised: 2022-01-24 
 * Version 1.1.117: resolved Issue 0928: CPP UNIT TESTS (testing)
    - description:  fail because of change of ConstSizeVector to use move assignment and move constructor
    - **notes:** adapted test to avoid move assignment
    - date resolved: **2022-02-06 00:04**\ , date raised: 2022-02-06 
 * Version 1.1.116: resolved Issue 0919: Minimize (extension)
    - description:  add optimization with same interface as GenticOptimization but based on scipy.minimize
    - **notes:** added Minimize to processing; usage is nearly same as GeneticOptimization(...); example under Examples/minimizeExample.py
    - date resolved: **2022-02-04 15:45**\ , date raised: 2022-02-02 
    - resolved by: S. Holzinger
 * Version 1.1.115: :textred:`resolved BUG 0924` : GeneralContact 
    - description:  WARNING message raised in ANCF contact
    - date resolved: **2022-02-03 12:01**\ , date raised: 2022-02-03 
 * Version 1.1.114: resolved Issue 0923: CreateReevingCurve (extension)
    - description:  add function CreateReevingCurve(...) in exudyn.beams to create reeving system along circles, allows to create curve and nodes for ANCFCable2D elements created with PointsAndSlopes2ANCFCable2D(...); see Examples/reevingSystem.py
    - date resolved: **2022-02-03 12:00**\ , date raised: 2022-02-02 
 * Version 1.1.113: resolved Issue 0803: PostNewton (check)
    - description:  Check discontinuous iterations in combination with adaptive step (immediately reduces step size even if ignoreMaxSteps=True)
    - **notes:** did not further show up; possibly due to non-convergence
    - date resolved: **2022-02-02 08:20**\ , date raised: 2021-11-25 
 * Version 1.1.112: resolved Issue 0906: PlotSensor (extension)
    - description:  add option componentsX to add x-components for figures, e.g., to plot y over x position, instead over time
    - date resolved: **2022-02-01 18:34**\ , date raised: 2022-02-01 
 * Version 1.1.111: resolved Issue 0864: automatic example referencing (fix)
    - description:  fix searching for examples, e.g., NodePoint as NodePoint( in order no to find NodePoint2D examples
    - date resolved: **2022-01-31 23:01**\ , date raised: 2022-01-14 
 * Version 1.1.110: :textred:`resolved BUG 0903` : GeneralContact 
    - description:  autocomputed searchTree gives very large values and visualization not working
    - **notes:** ANCFCable2D bounding box computation had 1 wrong else case
    - date resolved: **2022-01-31 21:34**\ , date raised: 2022-01-31 
 * Version 1.1.109: resolved Issue 0899: GenerateCircularArcANCFCable2D (extension)
    - description:  add function to create beams along circular arc
    - date resolved: **2022-01-31 14:20**\ , date raised: 2022-01-30 
 * Version 1.1.108: :textred:`resolved BUG 0902` : GenerateStraightLineANCFCable2D 
    - description:  cableNodePositionList does not returns 3D vectors for nodes except first node
    - date resolved: **2022-01-31 14:19**\ , date raised: 2022-01-31 
 * Version 1.1.107: resolved Issue 0898: GenerateStraightLineANCFCable2D (extension)
    - description:  add option to use existing nodes in generation of beams
    - date resolved: **2022-01-31 10:11**\ , date raised: 2022-01-30 
 * Version 1.1.106: resolved Issue 0897: add Python utility beams (extension)
    - description:  add exudyn.beams utility module, containing helper functions for creating beams, etc.; move existing functions to this module
    - date resolved: **2022-01-31 10:11**\ , date raised: 2022-01-30 
 * Version 1.1.105: :textred:`resolved BUG 0900` : GenerateStraightLineANCFCable2D 
    - description:  arguments vALE and ConstrainAleCoordinate are not implemented and need to be removed
    - date resolved: **2022-01-30 23:32**\ , date raised: 2022-01-30 
 * Version 1.1.104: resolved Issue 0896: theDoc objects (docu)
    - description:  sort objects into bodies, basic connectors, constraints and joints
    - date resolved: **2022-01-30 23:12**\ , date raised: 2022-01-30 
 * Version 1.1.103: resolved Issue 0895: ConnectorGravity (extension)
    - description:  add connector representing gravitational forces between heavy masses (planet, satellite, etc.)
    - date resolved: **2022-01-30 19:05**\ , date raised: 2022-01-30 
 * Version 1.1.102: resolved Issue 0894: colors (extension)
    - description:  added several colors in graphicsDataUtilities, added color4black, extended color4list to 16 colors
    - date resolved: **2022-01-30 18:10**\ , date raised: 2022-01-30 
 * Version 1.1.101: resolved Issue 0893: GeneralContact (change)
    - description:  changed flag introSpheresContact to sphereSphereContact; added flag for special mode to recycle last Post Newton step friction force
    - date resolved: **2022-01-28 18:55**\ , date raised: 2022-01-28 
 * Version 1.1.100: resolved Issue 0891: adapt to Python3.9 (extension)
    - description:  make wheels and installers for Python3.9, running on Anaconda3-2021-11 Windows-x86_64
    - date resolved: **2022-01-25 23:31**\ , date raised: 2022-01-25 
 * Version 1.1.99: resolved Issue 0889: PlotSensor (testing)
    - description:  add test for PlotSensor, returning 1 if all plotting runs without crashing
    - date resolved: **2022-01-25 19:03**\ , date raised: 2022-01-25 
 * Version 1.1.98: resolved Issue 0890: PlotSensor (extension)
    - description:  add possibility to use filenames instead of sensor numbers, automatically loading these files
    - date resolved: **2022-01-25 18:00**\ , date raised: 2022-01-25 
 * Version 1.1.97: :textred:`resolved BUG 0887` : iPython output stops 
    - description:  after a solver error, output of iPython stops
    - **notes:** changed to correct catching/throwing of Python and C++ exception types; outpur continues after solver error
    - date resolved: **2022-01-25 14:25**\ , date raised: 2022-01-25 
 * Version 1.1.96: resolved Issue 0885: GetInterpolatedSignalValue (extension)
    - description:  add check in case that time values are distributed non-uniform; add tolerance as option
    - date resolved: **2022-01-25 12:10**\ , date raised: 2022-01-25 
 * Version 1.1.95: resolved Issue 0884: PlotSensor (extension)
    - description:  add optional title to plot
    - date resolved: **2022-01-25 11:52**\ , date raised: 2022-01-25 
 * Version 1.1.94: resolved Issue 0883: PlotSensor (extension)
    - description:  if sensorNumbers is scalar, components is a list, sensorNumbers is automatically adjusted; accepts now e.g. PlotSensor(mbs, 0, components=[0,1,2])
    - date resolved: **2022-01-25 11:36**\ , date raised: 2022-01-25 
 * Version 1.1.93: resolved Issue 0881: PlotSensor (extension)
    - description:  add option to apply factors and offsets to plotted signals; add option to add labels which appears at legend
    - date resolved: **2022-01-25 11:36**\ , date raised: 2022-01-25 
 * Version 1.1.92: resolved Issue 0882: PlotSensor (change)
    - description:  add option to use X, Y and Z components instead of 0, 1, 2 for Position, Displacement etc.; this option is enabled by default and changes the appearance -> set False, to preserve the old mode; component now also shown if only one curve plotted
    - date resolved: **2022-01-25 11:19**\ , date raised: 2022-01-25 
 * Version 1.1.91: resolved Issue 0879: LoadSolutionFile (extension)
    - description:  changed safeMode to loading sinle lines, which saves memory enormously, and added new options for loading huge files
    - date resolved: **2022-01-24 14:53**\ , date raised: 2022-01-24 
 * Version 1.1.90: :textred:`resolved BUG 0877` : AddSensorRecorder 
    - description:  fails for scalar Sensors OutputVariableTypes (e.g. when measuring Rotation of TorsionalSpringDamper)
    - **notes:** added special scalar case
    - date resolved: **2022-01-21 09:17**\ , date raised: 2022-01-21 
 * Version 1.1.89: resolved Issue 0876: flush files (extension)
    - description:  add option to flush solution and sensor files immediately after writing, simplifying the readout process; add option for large scale simulations, which are always flushed - helping for continuation of computations on supercomputers
    - date resolved: **2022-01-20 14:01**\ , date raised: 2022-01-20 
 * Version 1.1.88: resolved Issue 0875: PlotSensor (extension)
    - description:  fixed fontSize option and add options for minor/major ticks and SAVE figure to PlotSensor(...) using fileName=...
    - date resolved: **2022-01-19 11:08**\ , date raised: 2022-01-19 
 * Version 1.1.87: resolved Issue 0800: GeneralContact ANCFCable (extension)
    - description:  add ANCFCable2D to GeneralContact, enabling contact with planar spheres (cylinders)
    - date resolved: **2022-01-18 18:54**\ , date raised: 2021-11-19 
 * Version 1.1.86: resolved Issue 0866: numberOfThreads (extension)
    - description:  move simulationSettings.numberOfThreads into new section parallel in simulationSettings; remove comment [not implemented]
    - date resolved: **2022-01-18 17:43**\ , date raised: 2022-01-15 
 * Version 1.1.85: resolved Issue 0872: preStepPyExecute (change)
    - description:  remove preStepPyExecute from docu, time integration / static solver interface and from CSolverBase
    - date resolved: **2022-01-18 16:57**\ , date raised: 2022-01-18 
 * Version 1.1.84: resolved Issue 0874: improved Newton restart (change)
    - description:  added an additional Newton iteration after restarting modified Newton or when switching to full Newton; this reduces effects in generalized alpha and may improve behaviour with severe nonlinearities
    - date resolved: **2022-01-18 13:44**\ , date raised: 2022-01-18 
 * Version 1.1.83: resolved Issue 0869: adaptiveStepRecoveryIterations (change)
    - description:  add option to static and dynamic solvers to adjust max. Newton+disc. iterations prior to increase of step size; changed (previous internal) default value from 5 to 7
    - date resolved: **2022-01-17 19:54**\ , date raised: 2022-01-17 
 * Version 1.1.82: resolved Issue 0868: solverSettings.stepInformation (extension)
    - description:  change modes to add up binary flags; ADD several new options to show Newton iterations, jacobians, discontinuous iterations, per step or period; also add option to show output at every step
    - date resolved: **2022-01-17 12:11**\ , date raised: 2022-01-17 
 * Version 1.1.81: resolved Issue 0857: GetInterpolatedSignalValue (extension)
    - description:  new function to interpolate a numeric signal with time/data vectors at a certain time point
    - date resolved: **2022-01-11 15:41**\ , date raised: 2022-01-11 
 * Version 1.1.80: resolved Issue 0856: IndexFromValue (extension)
    - description:  function got faster mode in case of constant sampling rate
    - date resolved: **2022-01-11 14:39**\ , date raised: 2022-01-11 
 * Version 1.1.79: resolved Issue 0854: Add LTG description (docu)
    - description:  add section on local-to-global mapping of coordinates Section :ref:`sec-overview-ltgmapping`\ 
    - date resolved: **2022-01-08 12:09**\ , date raised: 2022-01-08 
 * Version 1.1.78: resolved Issue 0849: publications directory (docu)
    - description:  create separate directory Examples/publications/ for publication data, Python files of numerical examples, etc.
    - date resolved: **2022-01-06 16:32**\ , date raised: 2022-01-06 
 * Version 1.1.77: resolved Issue 0846: analytic jacobians (extension)
    - description:  add analytic jacobians general functionality, realized for CartesianSpringDamper and CoordinateSpringDamper; deactivate with newton.numericalDifferentiation.forODE2connectors = False
    - date resolved: **2021-12-23 11:07**\ , date raised: 2021-12-23 
 * Version 1.1.76: resolved Issue 0770: Marker jacobian derivative2 (extension)
    - description:  add jacobian derivative to most important connector markers
    - **notes:** implemented for markers except MarkerSuperElement; only MarkerPosition and MarkerRigidBody affected mostly; first tests show that jacobianDerivative agrees with numerical differentiation, BUT even increases iteration numbers
    - date resolved: **2021-12-22 21:21**\ , date raised: 2021-09-28 
 * Version 1.1.75: resolved Issue 0842: implement AccessFunctionType::JacobianTtimesVector_q (extension)
    - description:  implement function needed for MarkerBody for objects ANCFCable, ObjectFFRF and ObjectFFRFreducedOrder; currently raising exception if used in this setup!
    - date resolved: **2021-12-22 21:18**\ , date raised: 2021-12-22 
 * Version 1.1.74: resolved Issue 0831: Explicit solvers (change)
    - description:  remove second ComputeODE2Acceleration() call and copy solutionODE2_tt from beginning of time step rk.stageDerivODE2_t[0], same for ODE1 variables; add flag but change that by default as it speeds up 2x; could also use information from previous step ...?
    - date resolved: **2021-12-20 13:09**\ , date raised: 2021-12-15 
 * Version 1.1.73: resolved Issue 0832: explicit integrator (extension)
    - description:  add flag timeintegration.explicitIntegration.computeEndOfStepAccelerations to compute end-of-step accelerations; this computation doubles the effort of explicit one-step-methods, particularly relevant in particles or contact simulations
    - date resolved: **2021-12-17 12:38**\ , date raised: 2021-12-17 
 * Version 1.1.72: resolved Issue 0450: MT integration (extension)
    - description:  fully integrate multithreading into system.cpp, vector.cpp and dense solver
    - **notes:** integrated into system, missing mass matrix and jacobian
    - date resolved: **2021-12-09 18:17**\ , date raised: 2020-09-16 
 * Version 1.1.71: resolved Issue 0799: GeneralContact description (docu)
    - description:  add general section in theDoc for description of GeneralContact
    - date resolved: **2021-12-09 12:36**\ , date raised: 2021-11-19 
 * Version 1.1.70: resolved Issue 0793: performance section (docu)
    - description:  add performance and speedup section to theDoc, explaining most useful settings like modifiedNewton, EigenSparse, writeToFile, step size, numberOfThreads, constant mass matrix, etc.
    - date resolved: **2021-12-09 12:36**\ , date raised: 2021-11-02 
 * Version 1.1.69: resolved Issue 0823: add trig-sphere friction tests (extension)
    - description:  add simple test cases to check friction implementation
    - date resolved: **2021-12-09 08:28**\ , date raised: 2021-12-06 
 * Version 1.1.68: resolved Issue 0822: shrink mesh (extension)
    - description:  add method to shrink meshes using max distance to surface with normals; used for contact trig-sphere implementation
    - **notes:** currently slows down for larger meshes due to elimination of duplicate points!
    - date resolved: **2021-12-09 08:28**\ , date raised: 2021-12-06 
 * Version 1.1.67: :textred:`resolved BUG 0827` : GraphicsData cube 
    - description:  cubes has wrong numbering of nodes
    - **notes:** FIXED numbering in order to allow for correct normals needed in contact computation
    - date resolved: **2021-12-07 17:14**\ , date raised: 2021-12-07 
 * Version 1.1.66: resolved Issue 0826: GraphicsData normals (extension)
    - description:  add normals to some GraphicsData  cube objects
    - date resolved: **2021-12-07 16:27**\ , date raised: 2021-12-07 
 * Version 1.1.65: resolved Issue 0825: GraphicsData add defaults (extension)
    - description:  add some default values, especially to (center)point of GraphicsDataSphere, GraphicsDataCylinder, GraphicsDataOrthoCube, etc. in order to reduce interface sizes for objects added in the centerpoint [0,0,0]; thus some defaults were also necessary for radius or sizes!
    - date resolved: **2021-12-07 14:41**\ , date raised: 2021-12-07 
 * Version 1.1.64: resolved Issue 0824: PlotSensor (change)
    - description:  added serval NEW options, including: newFigure, colorCodeOffset, figureName and closeAll; NOTE that now PlotSensor by default opens a new figure!
    - date resolved: **2021-12-07 12:11**\ , date raised: 2021-12-07 
 * Version 1.1.63: :textred:`resolved BUG 0820` : RigidBody visualization 
    - description:  normals in rigid body visualization transformed wrongly
    - **notes:** before, rotating bodies may have shown shading, now resolved
    - date resolved: **2021-12-05 17:32**\ , date raised: 2021-12-05 
 * Version 1.1.62: resolved Issue 0816: GraphicsData convert (extension)
    - description:  add function GraphicsData2TrigsAndPoints(...) to convert graphicsData into triangles and points
    - date resolved: **2021-12-05 15:43**\ , date raised: 2021-12-02 
 * Version 1.1.61: resolved Issue 0814: robotics.future, robotics.utilities, robotics.mobile (extension)
    - description:  add special submodules for robotics functions; future contains currently developed submodules that will be available in future at a different location in robotics
    - date resolved: **2021-12-05 15:43**\ , date raised: 2021-12-02 
 * Version 1.1.60: resolved Issue 0819: ObjectRigidBody (change)
    - description:  correct HasConstantMassMatrix for Lie group nodes and COM=0
    - date resolved: **2021-12-05 11:19**\ , date raised: 2021-12-05 
 * Version 1.1.59: resolved Issue 0804: GeneralContact TriangleMesh (extension)
    - description:  Add rigid-body-marker based triangle mesh to GeneralContact
    - date resolved: **2021-12-04 10:08**\ , date raised: 2021-11-25 
 * Version 1.1.58: resolved Issue 0818: MergeGraphicsDataTriangleList (extension)
    - description:  now works if either both lists contain normals or both do not
    - date resolved: **2021-12-03 20:15**\ , date raised: 2021-12-03 
 * Version 1.1.57: resolved Issue 0817: NodePointGround (extension)
    - description:  add Node::Orientation to NodeType, such that it can also be used as a rigidBody node
    - date resolved: **2021-12-03 14:34**\ , date raised: 2021-12-03 
 * Version 1.1.56: resolved Issue 0815: tCPU showing wrong time (change)
    - description:  minor bug; time shown is since starting of iPython
    - date resolved: **2021-12-02 17:48**\ , date raised: 2021-12-02 
 * Version 1.1.55: resolved Issue 0813: exudyn.robotics.special (change)
    - description:  extend and move roboticsSpecial to robotics.special; add special robotics functionality likde manipulability
    - date resolved: **2021-12-02 11:03**\ , date raised: 2021-12-02 
    - resolved by: M. Sereinig
 * Version 1.1.54: resolved Issue 0618: robotics submodule (extension)
    - description:  Create robotics.special and robotics.mecanum or robotics.ros submodules with subdirectories
    - date resolved: **2021-12-02 11:02**\ , date raised: 2021-03-23 
 * Version 1.1.53: resolved Issue 0812: Suppress warnings (extension)
    - description:  add flag to globally suppress warnings
    - **notes:** use exudyn.SuppressWarnings(True) to turn off warnings
    - date resolved: **2021-12-01 08:50**\ , date raised: 2021-12-01 
 * Version 1.1.52: resolved Issue 0811: add default constructors (change)
    - description:  add rule of five default constructors to SlimVector, SlimArray, ConstSizeVector and ConstSizeMatrix; remove mutable from data
    - date resolved: **2021-11-28 21:56**\ , date raised: 2021-11-28 
 * Version 1.1.51: resolved Issue 0810: GeneralContact visualization (extension)
    - description:  add visualization for searchtree (box) and bounding boxes
    - **notes:** use SC.visualizationSettings.contact to adjust the various options to visualize the contact search tree
    - date resolved: **2021-11-26 23:20**\ , date raised: 2021-11-26 
 * Version 1.1.50: resolved Issue 0809: ParameterVariation (extension)
    - description:  Added a additional argument parameterFunctionData={} to function ParameterVariation(). The argument parameterFunctionData can be used to make global data available inside the parameterFunction.
    - date resolved: **2021-11-26 12:57**\ , date raised: 2021-11-26 
    - resolved by: S. Holzinger
 * Version 1.1.49: resolved Issue 0808: Performance test for GeneralContact (testint)
    - description:  added test with sphere contact
    - date resolved: **2021-11-26 10:49**\ , date raised: 2021-11-26 
 * Version 1.1.48: resolved Issue 0802: GeneralContact (testing)
    - description:  add TestModel for Sphere-Sphere GeneralContact
    - date resolved: **2021-11-25 22:58**\ , date raised: 2021-11-25 
 * Version 1.1.47: resolved Issue 0807: GeneralContact (change)
    - description:  added new functions to initialize searchTree, searchTreeBox and frictionPairings which were previously in FinalizeContact(...)
    - date resolved: **2021-11-25 22:16**\ , date raised: 2021-11-25 
 * Version 1.1.46: resolved Issue 0806: removed FinalizeContact (change)
    - description:  removed this function, which is now automatically called in mbs.Assemble()
    - date resolved: **2021-11-25 22:15**\ , date raised: 2021-11-25 
 * Version 1.1.45: resolved Issue 0805: AssembleSystemInitialize (extension)
    - description:  add additional function inside mbs.Assemble() to initialize GeneralContact
    - date resolved: **2021-11-25 22:14**\ , date raised: 2021-11-25 
 * Version 1.1.44: resolved Issue 0779: sensor recorder (extension)
    - description:  add utilities function for recording signals internally in mbs; avoids writing to sensor files, which helps reducing overhead in ParameterVariation and GeneticOptimization
    - **notes:** available in Python utilities function AddSensorRecorder(...)
    - date resolved: **2021-11-25 11:10**\ , date raised: 2021-10-17 
 * Version 1.1.43: resolved Issue 0801: Box3D (change)
    - description:  check Ubuntu20 warnings; replace Vector3D pmin with Real pmin[3] to avoid warnings and gain speedup
    - date resolved: **2021-11-25 11:08**\ , date raised: 2021-11-23 
 * Version 1.1.42: resolved Issue 0798: Implicit GeneralContact (extension)
    - description:  add ODE2RHS jacobian and PostNewton to GeneralContact
    - date resolved: **2021-11-19 16:07**\ , date raised: 2021-11-19 
 * Version 1.1.41: resolved Issue 0787: GeneralContact (extension)
    - description:  add contact object, directly in mbs, which allows different types of contact with efficient computation and search trees; start with simple spherical contact
    - date resolved: **2021-11-19 16:06**\ , date raised: 2021-11-01 
 * Version 1.1.40: :textred:`resolved BUG 0784` : verboseMode 
    - description:  output of every step in verboseMode=1 after long time or if there are some very long lasting steps
    - **notes:** not fully clarified, but modified time when data is output; may occur in case of very long running iPython?
    - date resolved: **2021-11-14 23:23**\ , date raised: 2021-10-31 
 * Version 1.1.39: :textred:`resolved BUG 0790` : multithreading fails for user functions 
    - description:  build separate lists for objects and loads with user functions, excluded in MT evaluation
    - date resolved: **2021-11-14 23:21**\ , date raised: 2021-11-02 
 * Version 1.1.38: :textred:`resolved BUG 0794` : error when switching from n to 1 threads 
    - description:  RuntimeError: TemporaryComputationDataArray::operator[]: index out of range caused when switching from numberOfThreads>1 to 1 thread
    - date resolved: **2021-11-13 23:59**\ , date raised: 2021-11-04 
 * Version 1.1.37: resolved Issue 0788: multithreaded ODE2RHS and compute loads (extension)
    - description:  use multithreaded computation for ODE2 RHS and for loads computation; use simulationSettings.numberOfThreads > 1 for multithreaded computation
    - date resolved: **2021-11-13 23:59**\ , date raised: 2021-11-01 
 * Version 1.1.36: resolved Issue 0796: GL list GeneralContact (extension)
    - description:  speed up visualization with GL list for spheres in GeneralContact
    - date resolved: **2021-11-13 23:03**\ , date raised: 2021-11-10 
 * Version 1.1.35: resolved Issue 0797: itemInterface (extension)
    - description:  add representation for item interface classes, using __repr__() = dict(self)
    - date resolved: **2021-11-10 08:40**\ , date raised: 2021-11-10 
 * Version 1.1.34: resolved Issue 0789: constant mass matrix (extension)
    - description:  do not recompute mass matrix if it is totally constant in implicit solver; add flag to solver options to force recompute
    - date resolved: **2021-11-08 10:30**\ , date raised: 2021-11-02 
 * Version 1.1.33: resolved Issue 0795: add TCP/IP interface (extension)
    - description:  add CreateTCPIPconnection and other functions to utilities for interconnection with other programs via TCP/IP
    - date resolved: **2021-11-08 10:29**\ , date raised: 2021-11-08 
 * Version 1.1.32: resolved Issue 0786: TemporaryComputationDataArray (extension)
    - description:  add array of TemporaryCompData for multithreaded computation
    - date resolved: **2021-11-01 23:01**\ , date raised: 2021-11-01 
 * Version 1.1.31: resolved Issue 0785: ComputeSystemODE1RHS (extension)
    - description:  add list of loads with ODE1 relevancy, otherwise all loads are computed even if there are no ODE1 coordinates
    - **notes:** added simple flag to avoid computation if no ODE1 coordinates available
    - date resolved: **2021-11-01 21:28**\ , date raised: 2021-11-01 
 * Version 1.1.30: resolved Issue 0781: add n-mass-oscillator (example)
    - description:  add interactive example based on simulateInteractively for n-mass-oscillator with step and frequency excitation
    - date resolved: **2021-10-28 16:44**\ , date raised: 2021-10-25 
 * Version 1.1.29: resolved Issue 0747: ComputeLinearizedSystem (extension)
    - description:  add exudyn.ComputeLinearizedSystem similar to what is done in ComputeODEEigenvalues, returning M, K, D, ...
    - date resolved: **2021-10-27 18:40**\ , date raised: 2021-09-03 
 * Version 1.1.28: resolved Issue 0780: enable close window button (extension)
    - description:  close window button is now enabled, which stops current and following simulations until render window is restarted or SetRenderEngineStopFlag(False)
    - **notes:** if a simulation with renderer is quit (also ESCAPE button), then a further call to solver will be ignored until the simulation is reset, or SetRenderEngineStopFlag(False)
    - date resolved: **2021-10-21 09:31**\ , date raised: 2021-10-21 
 * Version 1.1.27: resolved Issue 0778: GenericODE2 FEM (extension)
    - description:  add FEMinterface function for creation of ObjectGenericODE2 with linear FEM model and nonlinear FEM model (using NGsolve)
    - date resolved: **2021-10-16 23:23**\ , date raised: 2021-10-16 
 * Version 1.1.26: :textred:`resolved BUG 0776` : MatrixContainer::SetWithSparseMatrixCSR 
    - description:  does not set number of columns and rows due to error in MatrixContainer::SetAllZero()
    - date resolved: **2021-10-09 16:53**\ , date raised: 2021-10-08 
 * Version 1.1.25: resolved Issue 0767: sparse ObjectJacobianODE2 (extension)
    - description:  add dense and sparse interface to ObjectJacobianODE2
    - date resolved: **2021-10-09 16:53**\ , date raised: 2021-09-27 
 * Version 1.1.24: :textred:`resolved BUG 0775` : ObjectGenericODE2, ObjectFFRFreducedOrder 
    - description:  visualization fails if outputVariable = None
    - date resolved: **2021-10-08 17:26**\ , date raised: 2021-10-08 
 * Version 1.1.23: resolved Issue 0773: ImportMeshFromNGsolve (extension)
    - description:  added option meshOrder which allows to use second order elements with meshOrder=2, leading to much higher accuracy of displacements and stresses
    - date resolved: **2021-10-01 14:04**\ , date raised: 2021-10-01 
 * Version 1.1.22: resolved Issue 0774: ComputePostProcessingModesNGsolve (extension)
    - description:  added improved functionality for ComputePostProcessingModes using NGsolve, speeding up computations by factor of 10
    - date resolved: **2021-10-01 14:03**\ , date raised: 2021-10-01 
 * Version 1.1.21: resolved Issue 0772: compute HCB modes with NGsolve (extension)
    - description:  add much faster computation function ComputeHurtyCraigBamptonModesNGsolve for computation of eigenmodes
    - date resolved: **2021-10-01 14:03**\ , date raised: 2021-10-01 
 * Version 1.1.20: resolved Issue 0771: GeneticOptimization (extension)
    - description:  add normal distribution and distanceFactorGenerations
    - date resolved: **2021-09-29 17:38**\ , date raised: 2021-09-29 
 * Version 1.1.19: resolved Issue 0769: Marker jacobian derivative (extension)
    - description:  add jacobian derivative to markers to allow analytical differentiation of connectors
    - date resolved: **2021-09-28 18:57**\ , date raised: 2021-09-28 
 * Version 1.1.18: resolved Issue 0768: Newton.useNumericalDifferentiation (change)
    - description:  change to Newton.numericalDifferentiation.forAE and Newton.numericalDifferentiation.forODE2; previous Newton.useNumericalDifferentiation only affected AE (algebraic equations) and should be changed now to  Newton.numericalDifferentiation.forAE; default is False
    - date resolved: **2021-09-27 15:11**\ , date raised: 2021-09-27 
 * Version 1.1.17: resolved Issue 0612: sparse object matrices (extension)
    - description:  add sparse matrix computation mode for ComputeMassMatrix and for ObjectJacobianODE2; consider Lie algebra derivatives
    - **notes:** sparse ObjectJacobianODE2 computation not yet implemented and moved to issue767
    - date resolved: **2021-09-27 14:35**\ , date raised: 2021-03-21 
 * Version 1.1.16: resolved Issue 0766: ObjectGenericODE2 (change)
    - description:  change mass matrix, stiffnessmatrix, etc. types to PyMatrixContainer in order to accept dense and sparse matrices
    - date resolved: **2021-09-27 14:32**\ , date raised: 2021-09-26 
 * Version 1.1.15: resolved Issue 0765: describe Python types (docu)
    - description:  describe Python types such as NumpyMatrix or PyMatrixContainer in intro to objects, nodes, ...
    - date resolved: **2021-09-27 14:32**\ , date raised: 2021-09-26 
 * Version 1.1.14: resolved Issue 0764: PyMatrixContainer (extension)
    - description:  extend PyMatrixContainer to accept numpy.array or list of lists as input
    - date resolved: **2021-09-26 23:35**\ , date raised: 2021-09-26 
 * Version 1.1.13: resolved Issue 0763: ComputeMassMatrix (extension)
    - description:  add sparse mode with MatrixContainer
    - date resolved: **2021-09-26 18:01**\ , date raised: 2021-09-26 
 * Version 1.1.12: resolved Issue 0762: MatrixBase (performance)
    - description:  remove virtual from begin/end operators
    - date resolved: **2021-09-26 17:36**\ , date raised: 2021-09-26 
 * Version 1.1.11: :textred:`resolved BUG 0750` : ANCF/ALE contour plot 
    - description:  contour plot not showing displacements or forces
    - **notes:** bug due to issue 760, which has been resolved now!
    - date resolved: **2021-09-24 08:48**\ , date raised: 2021-09-03 
 * Version 1.1.10: resolved Issue 0761: LinkedDataVectorBase (extension)
    - description:  allowing SetNumberOfItems to make LinkedDataVectors smaller after linking
    - date resolved: **2021-09-24 08:47**\ , date raised: 2021-09-24 
 * Version 1.1.9: :textred:`resolved BUG 0760` : contour plot 
    - description:  nodes in contour plot leading to exudyn crash due to ConstSizeVector decoupled from Vector
    - date resolved: **2021-09-24 08:47**\ , date raised: 2021-09-24 
 * Version 1.1.8: resolved Issue 0758: FEM CMSObjectComputeNorm (extension)
    - description:  add function into FEM to compute maximum stress / strain / etc for CMSObject (ObjectFFRFreducedOrder), using only the objectNumber as an input; options are outputVariableType=StressLocal, norm="" (Mises, L2norm, none), nodeNumbers=[] ... providing optional list of nodes to restrict the computation
    - date resolved: **2021-09-23 19:25**\ , date raised: 2021-09-22 
 * Version 1.1.7: resolved Issue 0757: FEM GetNodePositionsMean (extension)
    - description:  add function into FEMinterface to compute mean (average) position based on nodeNumbers (as list)
    - date resolved: **2021-09-23 17:38**\ , date raised: 2021-09-22 
 * Version 1.1.6: :textred:`resolved BUG 0759` : contour plot 
    - description:  equivalent stress showing negative color bar values in contour plot
    - **notes:** contour plot with norm (component -1) showing only positive min and max values when tested
    - date resolved: **2021-09-23 17:19**\ , date raised: 2021-09-23 
 * Version 1.1.5: resolved Issue 0756: FEM MisesStress (extension)
    - description:  add function that computes Mises stress from 6 stress components as obtained in stress sensor
    - **notes:** put into exudyn.physics module
    - date resolved: **2021-09-23 16:02**\ , date raised: 2021-09-22 
 * Version 1.1.4: resolved Issue 0755: GetKinematicTree66 in robotics (extension)
    - description:  add function to export KinematicTree66 from Robotic class
    - date resolved: **2021-09-22 18:06**\ , date raised: 2021-09-22 
 * Version 1.1.3: resolved Issue 0754: performance tests (test)
    - description:  add automated performance tests for solver speed to determine significant drop of performance
    - date resolved: **2021-09-22 10:10**\ , date raised: 2021-09-22 
 * Version 1.1.2: resolved Issue 0620: TorsionalSpringDamper (extension)
    - description:  add torsional spring damper similar to SpringDamper, fixed on a single local axis of marker0, allowing to realize controllers and torques
    - date resolved: **2021-09-16 10:45**\ , date raised: 2021-04-06 
 * Version 1.1.1: resolved Issue 0679: Renderer tkinter (extension)
    - description:  add flag to disable calls to tkinter from Renderer, which is not possible if tkinter is already used for interactive dialogs. This allows to open visualizationSettings in AnimateModes and SolutionViewer
    - date resolved: **2021-09-14 10:21**\ , date raised: 2021-05-14 
 * Version 1.1.0: :textred:`resolved BUG 0751` : SetMarkerParameter 
    - description:  causes internal error, because of wrong index check; workaround: use int(..) to cast marker index
    - date resolved: **2021-09-10 16:22**\ , date raised: 2021-09-10 

***********
Version 1.0
***********

 * Version 1.0.295: :textred:`resolved BUG 0749` : ObjectALEANCFCable2D 
    - description:  precomputed mass terms not computed accordingly; leads to crash if not static solution computed in advance
    - date resolved: **2021-09-03 15:12**\ , date raised: 2021-09-03 
 * Version 1.0.294: resolved Issue 0748: Extend solver description (docu)
    - description:  add some description for SolveStatic / SolveDynamic in solver chapter
    - date resolved: **2021-09-03 13:54**\ , date raised: 2021-09-03 
 * Version 1.0.293: resolved Issue 0504: serialrobot (extension)
    - description:  build completely from homogenouos transformations, COMs, inertia tensors, masses, axes, axesTypes
    - **notes:** done earlier
    - date resolved: **2021-08-22 11:12**\ , date raised: 2020-12-16 
 * Version 1.0.292: resolved Issue 0540: github (extension)
    - description:  update README.rst file and make it similar to other packages (e.g. pydy)
    - **notes:** done earlier
    - date resolved: **2021-08-22 11:11**\ , date raised: 2021-01-09 
 * Version 1.0.291: resolved Issue 0729: README.rst (docu)
    - description:  add .rst readme file containing gettingStarted, introduction, tutorial and other information
    - date resolved: **2021-08-22 10:58**\ , date raised: 2021-08-05 
 * Version 1.0.290: resolved Issue 0740: robotics (extension)
    - description:  extend Robot class for Modified DH parameters and general transformations; add transformations before and after joint axis
    - date resolved: **2021-08-19 00:22**\ , date raised: 2021-08-18 
 * Version 1.0.289: :textred:`resolved BUG 0739` : robotics 
    - description:  CreateRedundantCoordinateMBS draws wrong axes
    - date resolved: **2021-08-19 00:22**\ , date raised: 2021-08-18 
 * Version 1.0.288: :textred:`resolved BUG 0742` : robotics 
    - description:  Robot.JointHT computes LinkHT
    - date resolved: **2021-08-18 23:31**\ , date raised: 2021-08-18 
 * Version 1.0.287: resolved Issue 0741: robotics (change)
    - description:  add base and tool class to robotics to have more flexibility for future developments; replace toolHT to tool.HT, baseHT to base.HT; add tool.visualization and base.visualization
    - date resolved: **2021-08-18 15:07**\ , date raised: 2021-08-18 
 * Version 1.0.286: resolved Issue 0733: ContactCoordinate (extension)
    - description:  add recommendedStepSize to ContactCoordinate and find optimal solution with data variable from StartOfStep configuration; check if step size is permanently reduced with recommendedStepSize; check a way of an overall recommendedStepSize (with filter) or allow a single event not to change global step size
    - date resolved: **2021-08-13 13:23**\ , date raised: 2021-08-12 
 * Version 1.0.285: resolved Issue 0313: Add user node ODE2 (extension)
    - description:  add user node with getposition, rotation, access functions
    - **notes:** not needed: GenericNodes can be used for that
    - date resolved: **2021-08-10 12:57**\ , date raised: 2020-01-10 
 * Version 1.0.284: resolved Issue 0730: RigidBody tutorial (docu)
    - description:  add tutorial for rigid body with AddRigidBody(...), AddRevoluteJoint(...) functionalities
    - date resolved: **2021-08-06 20:05**\ , date raised: 2021-08-05 
 * Version 1.0.283: resolved Issue 0732: DrawSystemGraph (extension)
    - description:  improve visualization and return graph and other information
    - date resolved: **2021-08-06 17:58**\ , date raised: 2021-08-06 
 * Version 1.0.282: :textred:`resolved BUG 0731` : AddRevoluteJoint 
    - description:  AddRevoluteJoint shows error in axis definition
    - date resolved: **2021-08-05 13:40**\ , date raised: 2021-08-05 
 * Version 1.0.281: resolved Issue 0704: Optimization2 (optimize)
    - description:  add direct function to NodeRigidBody to retrieve essential data for rigid body EOM and MarkerRigidBody; add flag, if rotation matrix and other quantities needed
    - **notes:** still no optimization for Lie group nodes, which however have simpler matrices
    - date resolved: **2021-07-31 22:33**\ , date raised: 2021-07-04 
 * Version 1.0.280: resolved Issue 0514: general wheel (extension)
    - description:  add general wheel model (with general rotation body); for mecanum wheel rolls
    - **notes:** implemented ObjectContactConvexRoll for general usage in Mecanum wheels and other applications
    - date resolved: **2021-07-31 21:20**\ , date raised: 2020-12-19 
    - resolved by: P. Manzl
 * Version 1.0.279: resolved Issue 0727: NodeRigidBody2D (docu)
    - description:  Outputvariable Rotation gives 3D vector, but wrong description in DOCU
    - **notes:** additionally: rotation is now directly copied from rotation coordinate and is not recomputed from Tait-Bryan angles of rotation matrix
    - date resolved: **2021-07-31 21:18**\ , date raised: 2021-07-14 
 * Version 1.0.278: resolved Issue 0726: description of nodes (docu)
    - description:  finish detailed description of 3D nodes and add rotation parameter description for Tait-Bryan in theory part
    - date resolved: **2021-07-13 21:17**\ , date raised: 2021-07-13 
 * Version 1.0.277: resolved Issue 0725: unify description (docu)
    - description:  unify notation for special vectors, e.g., reference point or local position; add unified abbreviations for ODE2, etc.
    - date resolved: **2021-07-13 21:17**\ , date raised: 2021-07-13 
 * Version 1.0.276: :textred:`resolved BUG 0724` : Linux version 
    - description:  exudyn fails after import exudyn on Ubuntu18.04 and 20.04, showing error with RenderStateMachine selectionString
    - **notes:** version 276 tested with Ubuntu20.04, working again
    - date resolved: **2021-07-12 22:47**\ , date raised: 2021-07-12 
 * Version 1.0.275: resolved Issue 0723: SolutionViewer (change)
    - description:  remove SolutionViewer from exudyn init file, as it causes problems if no tkinter or matplotlib installed
    - **notes:** use exudyn.interactive.SolutionViewer(...) instead
    - date resolved: **2021-07-12 20:23**\ , date raised: 2021-07-12 
 * Version 1.0.274: :textred:`resolved BUG 0722` : glfwGetWindowContentScale 
    - description:  function causes immediate crash on linux (UBUNTU) when importing exudyn
    - **notes:** added flag for linux compilation, excluding font scaling
    - date resolved: **2021-07-12 19:31**\ , date raised: 2021-07-12 
 * Version 1.0.273: resolved Issue 0719: Pybind11 2.6 (change)
    - description:  switch to Pybind11 2.6 in included C++ files
    - date resolved: **2021-07-12 16:57**\ , date raised: 2021-07-12 
 * Version 1.0.272: resolved Issue 0718: Python3.8 FASTLINALG (change)
    - description:  using now __FAST_EXUDYN_LINALG option, which excludes all range checks and other checks in arrays, matrices, etc.; leads usually to 30percent higher performance
    - date resolved: **2021-07-12 16:57**\ , date raised: 2021-07-12 
 * Version 1.0.271: :textred:`resolved BUG 0721` : FEM.GetNodesOnLine(..) 
    - description:  fails because self. missing in call to GetNodesOnCylinder
    - date resolved: **2021-07-12 16:35**\ , date raised: 2021-07-12 
 * Version 1.0.270: resolved Issue 0717: SC.StaticSolve, SC.TimeIntegrationSolve (change)
    - description:  remove these deprecated functions from interface
    - date resolved: **2021-07-12 15:33**\ , date raised: 2021-07-11 
 * Version 1.0.269: resolved Issue 0669: remove old solvers (change)
    - description:  remove old static and dynamic solvers as they are not any more up to date with graphics interface
    - date resolved: **2021-07-12 15:32**\ , date raised: 2021-05-10 
 * Version 1.0.268: resolved Issue 0716: SystemIsConsistent (change)
    - description:  add checks for functions that may not be called if not SystemIsConsistent
    - date resolved: **2021-07-11 17:30**\ , date raised: 2021-07-11 
 * Version 1.0.267: :textred:`resolved BUG 0714` : mbs.GetSensorValues 
    - description:  raises error for ObjectFFRFreducedOrder: ERROR: LinkedDataVectorBase(const VectorBase<T>&, Index), startPosition < 0
    - **notes:** caused when called before Assemble(); checks added in future
    - date resolved: **2021-07-11 17:08**\ , date raised: 2021-07-10 
 * Version 1.0.266: :textred:`resolved BUG 0715` : ObjectFFRFreducedOrder 
    - description:  OutputVariable Displacement includes localPosition, but should not
    - **notes:** GetMeshNodeLocalPosition included reference position twice
    - date resolved: **2021-07-11 16:33**\ , date raised: 2021-07-10 
 * Version 1.0.265: resolved Issue 0706: ConstSizeVector (optimize)
    - description:  decouple ConstSizeVector and ConstSizeMatrix from Vector / Matrix and avoid virtual calls, erase all rule of 5 member functions, optimize algebra
    - **notes:** improved speed up to factor 2 for some items!
    - date resolved: **2021-07-11 15:06**\ , date raised: 2021-07-06 
 * Version 1.0.264: :textred:`resolved BUG 0713` : ObjectFFRFreducedOrder 
    - description:  GetOutputVariableSuperElement does not agree with types described in theDoc; object does not provide Displacement or Position, sensors return wrong values
    - **notes:** corrected C++ implementation and theDoc.pdf for OutputVariableTypesSuperElement
    - date resolved: **2021-07-09 20:53**\ , date raised: 2021-07-09 
 * Version 1.0.263: resolved Issue 0712: serialRobot (change)
    - description:  improve speed of serial robot by transferring controllers from load userfunctions to mbs.SetPreStepUserFunction
    - date resolved: **2021-07-09 13:28**\ , date raised: 2021-07-09 
 * Version 1.0.262: resolved Issue 0711: generator files (change)
    - description:  changed backslash to slash in generator files such that they can also be executed on Linux and MacOS
    - date resolved: **2021-07-09 12:18**\ , date raised: 2021-07-09 
 * Version 1.0.261: resolved Issue 0699: CMarkerBodyRigid::ComputeMarkerData (optimize)
    - description:  implement optimized version for Rigid node and ObjectRigidBody and avoid repeated computation of rotation matrix, etc.
    - date resolved: **2021-07-08 00:46**\ , date raised: 2021-07-01 
 * Version 1.0.260: :textred:`resolved BUG 0709` : Linux/MacOS compile error 
    - description:  compiler error caused by EXU::Square
    - date resolved: **2021-07-07 19:11**\ , date raised: 2021-07-07 
 * Version 1.0.259: resolved Issue 0708: preprocessor flags (change)
    - description:  move EXUDYN_RELEASE to preprocessor flags in setup.py
    - date resolved: **2021-07-07 08:53**\ , date raised: 2021-07-07 
 * Version 1.0.258: resolved Issue 0705: Optimization3 (optimize)
    - description:  optimize ObjectRigidBody EOM, take Glocal columns instead numberOfRotationCoordinates, move rot_t into loop, etc.
    - date resolved: **2021-07-06 23:04**\ , date raised: 2021-07-04 
 * Version 1.0.257: resolved Issue 0703: ComputeOrthonormalBasis (change)
    - description:  changed rigidBodyUtilities function, which returns a list of basis vectors, into ComputeOrthonormalBasisVectors, while ComputeOrthonormalBasis now returns a rotation matrix
    - date resolved: **2021-07-02 08:49**\ , date raised: 2021-07-02 
 * Version 1.0.256: resolved Issue 0702: AddPrismaticJoint (extension)
    - description:  add convenient utility function to add prismatic joint based on 2 bodies, point and axis, doing all necessary work in background
    - date resolved: **2021-07-02 08:49**\ , date raised: 2021-07-02 
 * Version 1.0.255: resolved Issue 0701: AddRevoluteJoint (extension)
    - description:  add convenient utility function to add revolute joint based on 2 bodies, point and axis, doing all necessary work in background
    - date resolved: **2021-07-02 08:49**\ , date raised: 2021-07-02 
 * Version 1.0.254: resolved Issue 0700: add links for utility functions (docu)
    - description:  ADDED LINKS to Examples/ and TestModels/ example files at end of each python utility function and class, see Section :ref:`sec-pythonutilityfunctions`\ 
    - date resolved: **2021-07-01 21:46**\ , date raised: 2021-07-01 
 * Version 1.0.253: :textred:`resolved BUG 0697` : GenericJoint 
    - description:  index2 equations not properly implemented for prismatic joints
    - **notes:** added second term for index2 case if joint position not constrained; TrapezoidalIndex2 solver now works if translational joint axes not constrained
    - date resolved: **2021-07-01 15:50**\ , date raised: 2021-07-01 
 * Version 1.0.252: resolved Issue 0696: add PrismaticJoint (extension)
    - description:  add 3D prismatic joint with rotationMarker0/1 to adjust local coordinate systems and joint local x axis as the free axis of the joint
    - date resolved: **2021-07-01 13:43**\ , date raised: 2021-07-01 
 * Version 1.0.251: resolved Issue 0369: add RevoluteJoint (extension)
    - description:  add 3D revolute joint with rotationMarker0/1 to adjust joint coordinates and joint local z axis as rotation axis
    - date resolved: **2021-07-01 13:43**\ , date raised: 2020-04-10 
 * Version 1.0.250: resolved Issue 0695: Solution functions (change)
    - description:  remove exu and SC arguments from exudyn.utilities functions SetSolutionState(...), AnimateSolution(...); remove functoin SetVisualizationState(...): use SetSolutionState instead!
    - date resolved: **2021-06-29 17:47**\ , date raised: 2021-06-29 
 * Version 1.0.249: resolved Issue 0694: SolutionViewer (extension)
    - description:  add interactive dialog to view solution based on coordinateSolution.txt
    - date resolved: **2021-06-29 17:31**\ , date raised: 2021-06-29 
 * Version 1.0.248: resolved Issue 0693: RigidBody user function (extension)
    - description:  add test model for GenericODE2 user function based rigid body with Euler parameter and constraint
    - date resolved: **2021-06-28 19:34**\ , date raised: 2021-06-28 
 * Version 1.0.247: resolved Issue 0564: NodeRigidBodyEP (change)
    - description:  transfer EP constraint from object to node, for future application to 3D beams
    - date resolved: **2021-06-28 19:34**\ , date raised: 2021-01-28 
 * Version 1.0.246: resolved Issue 0413: ConnectorCoordinateVectorUF (extension)
    - description:  implement coordinate vector constraint user function; can be used as generic joint
    - date resolved: **2021-06-28 16:19**\ , date raised: 2020-05-25 
 * Version 1.0.245: resolved Issue 0691: extend ConnectorCoordinateVector (extension)
    - description:  extend ConnectorCoordinateVector for quadratic terms to be used as Euler Parameters constraint
    - date resolved: **2021-06-27 23:29**\ , date raised: 2021-06-27 
 * Version 1.0.244: resolved Issue 0690: add MarkerNodeCoordinates (extension)
    - description:  used for CoordinateVector constraint
    - date resolved: **2021-06-27 23:29**\ , date raised: 2021-06-27 
 * Version 1.0.243: resolved Issue 0689: add itemIndex to user functions (change)
    - description:  CHANGE OF userFunctions interface with additional itemIndex for ConnectorSpringDamper, ConnectorCartesianSpringDamper, ConnectorRigidBodySpringDamper, ConnectorCoordinate, ConnectorCoordinateVector, ConnectorJointGeneric; see theDoc for changes in the interface of these user functions and adapt your models!
    - **notes:** WARNING: Interface of user functions for ConnectorSpringDamper, ConnectorCartesianSpringDamper, ConnectorRigidBodySpringDamper, ConnectorCoordinate, ConnectorCoordinateVector, ConnectorJointGeneric changed!!!
    - date resolved: **2021-06-27 20:45**\ , date raised: 2021-06-27 
 * Version 1.0.242: resolved Issue 0688: ObjectGenericODE2 (change)
    - description:  extend ObjectGenericODE2 and ObjectGenericODE1 user functions for the item index to have access to nodes and other informaiton: WARNING: you need to adapt your existing user functions!
    - **notes:** WARNING: Interface of user functions for ObjectGenericODE2, ObjectFFRF... changed!!!
    - date resolved: **2021-06-27 20:44**\ , date raised: 2021-06-24 
 * Version 1.0.241: resolved Issue 0687: ObjectRigidBody (extension)
    - description:  add output variable VelocityLocal to 2D and 3D rigid body objects
    - date resolved: **2021-06-22 16:55**\ , date raised: 2021-06-22 
 * Version 1.0.240: resolved Issue 0686: eigenvalue solver (extension)
    - description:  use ngsolve solver for eigenvalue computation speedup
    - date resolved: **2021-06-14 15:21**\ , date raised: 2021-06-14 
 * Version 1.0.239: :textred:`resolved BUG 0684` : openGL.multisampling 
    - description:  Mac OS multisampling option in visualizationSettings crashes; ==> do not change this option under Mac OS
    - **notes:** excluded multisampling option for MacOS compilation
    - date resolved: **2021-05-30 12:02**\ , date raised: 2021-05-25 
 * Version 1.0.238: :textred:`resolved BUG 0681` : ObjectALEANCFCable2D 
    - description:  ObjectALEANCFCable2D position jacobian does not provide axially moving part for ObjectContactFrictionCircleCable2D, NEED TO BE ADDED
    - **notes:** had been already included in MarkerBodyCable2Dshape and works for roll contact
    - date resolved: **2021-05-30 12:01**\ , date raised: 2021-05-17 
 * Version 1.0.237: resolved Issue 0685: NodeRigidBody2D (change)
    - description:  add same drawing as 3D nodes (with reference frame)
    - date resolved: **2021-05-30 11:37**\ , date raised: 2021-05-30 
 * Version 1.0.236: resolved Issue 0683: SlidingJointRigid (extension)
    - description:  Add functionality for rigid sliding joint or add a flag for sliding joint to do both options
    - date resolved: **2021-05-20 09:30**\ , date raised: 2021-05-19 
 * Version 1.0.235: resolved Issue 0682: copy paste code from theDoc.pdf (extension)
    - description:  changed ' characters to enable copy/paste of code with quotes
    - date resolved: **2021-05-19 08:54**\ , date raised: 2021-05-19 
 * Version 1.0.234: :textred:`resolved BUG 0680` : SetSystemState 
    - description:  mbs.systemData.SetSystemState does not set data coordinates
    - date resolved: **2021-05-15 11:07**\ , date raised: 2021-05-15 
 * Version 1.0.233: resolved Issue 0676: single threaded renderer (change)
    - description:  improve exu.DoRendererIdleTasks() and use it in all python function - AnimateModes, Interactive, etc.
    - **notes:** can now be also called in multithreaded renderer
    - date resolved: **2021-05-14 21:42**\ , date raised: 2021-05-12 
 * Version 1.0.232: :textred:`resolved BUG 0678` : mouseInteractiveExample 
    - description:  not running any more, check new Renderer functions
    - date resolved: **2021-05-14 21:41**\ , date raised: 2021-05-14 
 * Version 1.0.231: resolved Issue 0630: HurtyCraigBampton (extension)
    - description:  extend computation to work with 0 eigenmodes
    - date resolved: **2021-05-12 23:51**\ , date raised: 2021-04-23 
 * Version 1.0.230: resolved Issue 0642: ComputeHurtyCraigBamptonModes (extension)
    - description:  add possibility to add position only interfaces
    - **notes:** abandoned, because makes no sense with RBE2 modes
    - date resolved: **2021-05-12 23:35**\ , date raised: 2021-04-30 
 * Version 1.0.229: :textred:`resolved BUG 0674` : OutputVariable.StressLocal 
    - description:  norm of OutputVariable stresses does not work
    - date resolved: **2021-05-12 23:21**\ , date raised: 2021-05-12 
 * Version 1.0.228: :textred:`resolved BUG 0456` : ObjectFFRF bug with GenericJoint 
    - description:  raises error: CSolverBase::SolveSteps CObjectSuperElement:GetAccessFunctionSuperElement: AngularVelocity_qt not implemented; cannot compute jacobian for orientation
    - date resolved: **2021-05-12 22:27**\ , date raised: 2020-10-13 
 * Version 1.0.226: resolved Issue 0100: UPDATE Lest tests (new feature)
    - description:  update lest tests (select C++ vs. python tests)    
    - date resolved: **2021-05-12 22:21**\ , date raised: 2019-04-01 
 * Version 1.0.225: resolved Issue 0372: add manual solver example (extension)
    - description:  add manual for solver and example; also add new prestep user function
    - **notes:** resolved earlier
    - date resolved: **2021-05-12 22:20**\ , date raised: 2020-04-10 
 * Version 1.0.224: resolved Issue 0384: Solver interface (extension)
    - description:  change solver interface such that it stores MainSystem/mbs for user functions; MainSolverXYZ and CSolverXYZ take MainSystem as argument in constructor
    - date resolved: **2021-05-12 22:18**\ , date raised: 2020-05-06 
 * Version 1.0.223: resolved Issue 0675: PostProcessingModes (extension)
    - description:  compute PostProcessingModes with multiprocessing
    - date resolved: **2021-05-12 22:10**\ , date raised: 2021-05-12 
 * Version 1.0.222: resolved Issue 0672: right-mouse-dialog (extension)
    - description:  add item indices to right mouse dialog
    - date resolved: **2021-05-12 22:05**\ , date raised: 2021-05-11 
 * Version 1.0.221: resolved Issue 0673: FEM.PostProcessingModes (extension)
    - description:  compute PostProcessingModes in FEMinterface with multiprocessing option
    - date resolved: **2021-05-12 15:37**\ , date raised: 2021-05-12 
 * Version 1.0.220: resolved Issue 0430: stress modes FEMinterface (extension)
    - description:  add into FEMinterface and allow storing that data
    - **notes:** resolved already earlier, see issue 623
    - date resolved: **2021-05-12 15:36**\ , date raised: 2020-07-01 
 * Version 1.0.219: :textred:`resolved BUG 0631` : ObjectFFRFreducedOrder 
    - description:  freefree eigenmodes and Hurty-Craig-Bampton modes do not converge to same results
    - **notes:** convergence for eigenmodes and HCB modes given, but differences due to HCB boundary sets, inconsistent initial conditions for MarkerSuperElementRigid; pure beam bending converges well
    - date resolved: **2021-05-12 14:05**\ , date raised: 2021-04-23 
 * Version 1.0.218: :textred:`resolved BUG 0671` : mbs.Reset() and SC.Reset() 
    - description:  reset MainSystem mbs and SystemContainer SC hangs; current SC is erronously stolen from renderer when another SC is deleted
    - date resolved: **2021-05-11 10:55**\ , date raised: 2021-05-11 
 * Version 1.0.217: resolved Issue 0670: MacOS graphics support (extension)
    - description:  add compatibility to MacOS in single-threaded graphics mode (tested with OS X 10.7)
    - date resolved: **2021-05-10 22:34**\ , date raised: 2021-05-10 
 * Version 1.0.216: resolved Issue 0647: Single Thread Renderer (extension)
    - description:  Implement single thread renderer version for MAC OS compatibility test
    - date resolved: **2021-05-10 22:32**\ , date raised: 2021-04-30 
 * Version 1.0.215: resolved Issue 0634: Set visualization state (extension)
    - description:  add thread-safe variant for updating the visualization state
    - date resolved: **2021-05-10 18:32**\ , date raised: 2021-04-25 
 * Version 1.0.214: resolved Issue 0668: multiple mbs and SystemContainer support (extension)
    - description:  adapt renderer and MainSystemContainer to work with multiple MainSystems (mbs) and SC instances at same time; add SC.AttachToRenderEnginer, SC.DetachFromRenderEngine
    - date resolved: **2021-05-10 18:20**\ , date raised: 2021-05-10 
 * Version 1.0.213: :textred:`resolved BUG 0665` : StartRenderer() 
    - description:  without being in a render loop (e.g., SC.WaitForRenderEngineStopFlag()), the pure StartRenderer() crashes upon left mouse click
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-05-05 
 * Version 1.0.212: resolved Issue 0664: right mouse (change)
    - description:  add function to retrieve py::dict from items safely in python thread into temporary storage; check also other python calls to operate fully in main thread
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-05-05 
 * Version 1.0.211: :textred:`resolved BUG 0662` : Render window 
    - description:  during open tkinter dialogs, the render window responds on keyboard or mouse input, which calls again python functions that hang up the system
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-05-04 
 * Version 1.0.210: resolved Issue 0633: WaitAndLockSemaphoreIgnore (check)
    - description:  check which atomic_flags are needed in C++ to make code threadsafe
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-04-25 
 * Version 1.0.209: resolved Issue 0667: tkinter dialogs focus and on top (extension)
    - description:  when opening tkinter dialogs - visualizationSettings, edit dialogs, help, ... - they immediately get focus and are on top
    - date resolved: **2021-05-10 14:49**\ , date raised: 2021-05-10 
 * Version 1.0.208: resolved Issue 0666: make renderer Python and thread safe (change)
    - description:  add strict separation between renderer (thread) and Python (thread); add rendererPythonInterface between both threads; left and right mouse clicks now safe to press; render window does not accept any input as long as tkinter window is open, but does not produce crashes any more
    - date resolved: **2021-05-10 14:49**\ , date raised: 2021-05-10 
 * Version 1.0.207: resolved Issue 0663: help button (docu)
    - description:  show "press h for help" as startup message for 10 seconds and sync help message with theDoc.pdf
    - date resolved: **2021-05-05 10:02**\ , date raised: 2021-05-05 
 * Version 1.0.206: resolved Issue 0653: add right mouse edit dialog (extension)
    - description:  open Edit dialog for item on right-mouse-press
    - date resolved: **2021-05-04 20:58**\ , date raised: 2021-05-01 
 * Version 1.0.205: resolved Issue 0652: identify itemID under mouse coursor (extension)
    - description:  identify object/node/... under mouse using unique color for itemID (left mouse button press)
    - date resolved: **2021-05-04 12:49**\ , date raised: 2021-05-01 
 * Version 1.0.204: resolved Issue 0637: Python3.8 windows wheels (extension)
    - description:  create Python3.8 windows wheels automatically
    - date resolved: **2021-05-03 18:51**\ , date raised: 2021-04-26 
 * Version 1.0.203: resolved Issue 0661: add C++ unit tests (extension)
    - description:  add C++ unit tests to Python3.6 64bits version and to testSuite. Changed initialization of all vector types to avoid errors of Vector({5}), now allowing only Vector({4.}) in constructors
    - date resolved: **2021-05-03 18:50**\ , date raised: 2021-05-03 
 * Version 1.0.202: resolved Issue 0660: initializerList (check)
    - description:  check if Vector is used with initializer list with one item - Vector({10}), converting to std::vector or Vector(10)
    - date resolved: **2021-05-03 18:50**\ , date raised: 2021-05-03 
 * Version 1.0.201: resolved Issue 0659: Troubleshooting (docu)
    - description:  Add Trouble shooting section, treating common Python and solver errors to theDoc.pdf
    - date resolved: **2021-05-03 10:18**\ , date raised: 2021-05-03 
 * Version 1.0.200: resolved Issue 0650: Highlight item# (extension)
    - description:  Highlight item# for object/node/etc.; add to visualizationSettings (itemType, item#, colorHighlightItem, colorOtherItems), draw all other items in gray
    - date resolved: **2021-05-03 01:18**\ , date raised: 2021-05-01 
 * Version 1.0.199: resolved Issue 0649: add ItemType (extension)
    - description:  Add enum ItemType: Node, Object, ...
    - date resolved: **2021-05-03 01:18**\ , date raised: 2021-05-01 
 * Version 1.0.198: resolved Issue 0651: add itemID to graphics objects (extension)
    - description:  Add itemID (nodes, objects, markers, loads, sensors, in that order) to graphics objects (for right-mouse-press)
    - date resolved: **2021-05-02 21:26**\ , date raised: 2021-05-01 
 * Version 1.0.197: resolved Issue 0658: add VisualizationSettings() interactive (change)
    - description:  move visualizationSettings window functions keypressRotationStep, mouseMoveRotationFactor, keypressTranslationStep, zoomStepFactor to new substructure "interactive"
    - **notes:** \ **adapt your models if you used these options!**\ 
    - date resolved: **2021-05-01 23:36**\ , date raised: 2021-05-01 
 * Version 1.0.196: resolved Issue 0654: coordinates sizes (extension)
    - description:  add function ODE2Size(...), ODE1Size(...), SystemSize(...) to mbs.systemData to retrieve number of ODE2,ODE1,AE and Data coordinates for certain configurationType; only works after mbs.Assemble()
    - date resolved: **2021-05-01 23:23**\ , date raised: 2021-05-01 
 * Version 1.0.195: resolved Issue 0656: mbs.systemData (change)
    - description:  removed GetCurrentTime() and SetVisualizationTime(...) which have been marked as deprecated already
    - **notes:** Use GetTime(...) and SetTime(...) in mbs.systemData instead
    - date resolved: **2021-05-01 23:08**\ , date raised: 2021-05-01 
 * Version 1.0.194: resolved Issue 0644: solver messages (extension)
    - description:  add solver message if not converged with helpful hints (especially if invert fails or newton fails)
    - date resolved: **2021-05-01 02:31**\ , date raised: 2021-04-30 
 * Version 1.0.193: resolved Issue 0349: add causing row (extension)
    - description:  output causing row/column (=coordinate) which leads to singular matrix; do this for Matrix.Invert as well as for SparseLU .info code; matrix class creates string with error message!
    - date resolved: **2021-05-01 02:30**\ , date raised: 2020-03-02 
 * Version 1.0.192: resolved Issue 0646: jacobian singular (extension)
    - description:  resolve singularities in general jacobian: resolves coordinates which are still free for static problems, but is marked as unsafe
    - date resolved: **2021-05-01 02:29**\ , date raised: 2021-04-30 
 * Version 1.0.191: resolved Issue 0645: redundant constraints (extension)
    - description:  resolve redundant constraints: add flag linearSolverSettings.ignoreSingularJacobian in SC.SimulationSettings() to ignore singular constraint jacobians
    - date resolved: **2021-05-01 02:29**\ , date raised: 2021-04-30 
 * Version 1.0.190: resolved Issue 0333: node numbers with type (extension)
    - description:  extend node/object/... numbers as python class with type information to check if item numbers are mixed illegally
    - **notes:** done already earlier, but still marked as unresolved
    - date resolved: **2021-04-30 17:04**\ , date raised: 2020-02-06 
 * Version 1.0.189: resolved Issue 0641: ObjectContactFrictionCircleCable2D (docu)
    - description:  add description and figure for theory and computation
    - date resolved: **2021-04-29 08:40**\ , date raised: 2021-04-29 
 * Version 1.0.188: :textred:`resolved BUG 0423` : fix MarkerSuperElementRigidBody 
    - description:  fix velocity level for MarkerSuperElementRigidBody (check constraint equations)
    - **notes:** fixed several errors and test examples work now on velocity level, but further checks are necessary
    - date resolved: **2021-04-27 18:24**\ , date raised: 2020-06-09 
 * Version 1.0.187: resolved Issue 0635: AnimateModes (check)
    - description:  check, why animate modes has threading-conflicts; use std::cout to find issues
    - **notes:** resolved threading conflicts, but visualization state set inbetween graphics update, which needs to resolve #634
    - date resolved: **2021-04-27 18:20**\ , date raised: 2021-04-25 
 * Version 1.0.186: resolved Issue 0640: MarkerSuperElementRigid (extension)
    - description:  remove referencePosition and add offset instead (to correct errors of midpoint due to small mesh-unsymmetries)
    - **notes:** \ **CHANGED interface**\ : MarkerSuperElementRigid does not have a referencePosition anymore, but adds a parameter offset
    - date resolved: **2021-04-27 18:18**\ , date raised: 2021-04-26 
 * Version 1.0.185: :textred:`resolved BUG 0639` : ObjectFFRFreducedOrder 
    - description:  incorrect AccessFunction AccessFunctionType::AngularVelocity_qt, missing correct reference point = midpoint for computation of rotation
    - date resolved: **2021-04-26 21:47**\ , date raised: 2021-04-26 
 * Version 1.0.184: resolved Issue 0638: MarkerSuperElementRigid (extension)
    - description:  use consistent reference point = midpoint for computation of rotation and use exponential Map for rotation matrix
    - date resolved: **2021-04-26 21:47**\ , date raised: 2021-04-26 
 * Version 1.0.183: resolved Issue 0636: Python3.8 (extension)
    - description:  add python 3.8 compilation tests; resolve issues with __index__ method needed fore NodeIndex, MarkerIndex, etc.
    - date resolved: **2021-04-26 16:25**\ , date raised: 2021-04-26 
 * Version 1.0.182: :textred:`resolved BUG 0629` : mesh visualization 
    - description:  visualization artifacts in larger FE meshes due to multithreading
    - **notes:** added flag threadSafeGraphicsUpdate to avoid thread conflicts between graphics and computation, which is by default set True and MAY SLOW DOWN your computation speed if True
    - date resolved: **2021-04-25 22:28**\ , date raised: 2021-04-22 
 * Version 1.0.181: resolved Issue 0632: CMS theory (docu)
    - description:  add theory section for Hurty-Craig-Bampton modes and eigenmode computation
    - date resolved: **2021-04-23 19:03**\ , date raised: 2021-04-23 
 * Version 1.0.180: :textred:`resolved BUG 0628` : FEMinterface.GetNodesOnCylinder 
    - description:  returns erroneous indices
    - **notes:** corrected indexing and add warnings for illegal node types
    - date resolved: **2021-04-22 22:59**\ , date raised: 2021-04-22 
 * Version 1.0.179: resolved Issue 0616: Craig-Bampton (extension)
    - description:  add static modes (Hurty-Craig-Bampton) to computation of modes in CMSinterface
    - **notes:** implemented in FEMinterface.ComputeHurtyCraigBamptonModes(...)
    - date resolved: **2021-04-21 11:09**\ , date raised: 2021-03-21 
 * Version 1.0.178: resolved Issue 0624: norm in contour plots (extension)
    - description:  show norm (of vectors or stresses) in contour plot, using special outputVariable component=-1
    - date resolved: **2021-04-09 18:18**\ , date raised: 2021-04-09 
 * Version 1.0.177: resolved Issue 0623: postProcessingModes (extension)
    - description:  add function to compute stress or strain modes for postprocessing, working for linear tetraherons (Tet4); see Examples/NGsolvePostProcessingStresses.py
    - date resolved: **2021-04-09 15:46**\ , date raised: 2021-04-09 
 * Version 1.0.176: resolved Issue 0424: show modes (extension)
    - description:  add feature to visualize eigenmodes, e.g. using ObjectFFRFreducedOrder and set one initialCoordinate nonzero
    - date resolved: **2021-04-07 13:48**\ , date raised: 2020-06-12 
 * Version 1.0.175: resolved Issue 0619: Eigenmode visualizer (extension)
    - description:  visualize eigenmodes with interactive tools with new function AnimateModes(...) to show eigenmodes of system or ObjectFFRFreducedOrder (see Section :ref:`sec-interactive-animatemodes`\ )
    - date resolved: **2021-04-07 13:47**\ , date raised: 2021-03-30 
 * Version 1.0.174: resolved Issue 0622: InteractiveDialog (extension)
    - description:  improved functionality of InteractiveDialog in interactive.py, specially for animating modes
    - date resolved: **2021-04-07 12:20**\ , date raised: 2021-04-07 
 * Version 1.0.173: :textred:`resolved BUG 0621` : mbs.GetNodeODE2Index 
    - description:  Fails for NodeRigidBodyEP, because mix of AE and ODE2 variables
    - **notes:** added correct type check in MainSystem::PyGetNodeODE2Index
    - date resolved: **2021-04-07 09:12**\ , date raised: 2021-04-07 
 * Version 1.0.172: resolved Issue 0403: CMS C++ (extension)
    - description:  add ObjectFFRFreducedOrder (CMS) equations in C++ and clean up code
    - date resolved: **2021-03-30 16:57**\ , date raised: 2020-05-21 
 * Version 1.0.171: resolved Issue 0470: geometrically exact beam2D (extension)
    - description:  add to CPP
    - date resolved: **2021-03-25 18:07**\ , date raised: 2020-11-21 
 * Version 1.0.170: resolved Issue 0563: ODE1Coordinate (extension)
    - description:  add MarkerODE1Coordinate and extend LoadCoordinate for ODE1
    - date resolved: **2021-03-25 07:43**\ , date raised: 2021-01-27 
 * Version 1.0.169: resolved Issue 0615: MarkerNodeCoordinate (extension)
    - description:  add check in CSystem for valid coordinate numbers in MarkerNodeCoordinate
    - date resolved: **2021-03-22 14:18**\ , date raised: 2021-03-21 
 * Version 1.0.168: resolved Issue 0611: adaptiveStep (extension)
    - description:  add adaptiveStepIncrease, Decrease and RecoverySteps options to control behavior in case of discontinuous problems
    - date resolved: **2021-03-21 00:21**\ , date raised: 2021-03-21 
 * Version 1.0.167: resolved Issue 0603: loadFactor (change)
    - description:  exclude load factor for loads with user functions in static computations
    - date resolved: **2021-03-20 23:24**\ , date raised: 2021-03-18 
 * Version 1.0.166: resolved Issue 0610: startOfStep (extension)
    - description:  add access function for nodal coordinates at startOfStep configuration, used in mbs.GetNodeOutput(configuration = exu.ConfigurationType.startOfStep)
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-20 
 * Version 1.0.165: resolved Issue 0609: SolveDynamic, SolveStatic (change)
    - description:  store dynamicSolver and staticSolver in mbs.sys dictionary immediately after creation, which allows to use these structures in user functions during static or dynamic solution
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-20 
 * Version 1.0.164: resolved Issue 0607: test recommendedStepSize (test)
    - description:  test recommendedStepSize and PostNewtonUserFunction with simple elastic contact example
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-20 
 * Version 1.0.163: resolved Issue 0605: UIndex, UReal (extension)
    - description:  change all relevant unsigned quantities to UIndex and UReal, as well as Vectors of UReal and Arrays of UIndex
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-19 
 * Version 1.0.162: resolved Issue 0604: UIndex check (extension)
    - description:  add automatic check in item interface to check for correctness of UIndex and UReal quantities
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-19 
 * Version 1.0.161: resolved Issue 0337: local quantities beam (change)
    - description:  change beam output of Force, Torque, Curvature, Stress and Strain to ForceLocal, TorqueLocal, CurvatureLocal, StressLocal, and StrainLocal
    - **notes:** \ **WARNING**\ : you need to adapt force, torque and stress, strain and curvature output variables accordingly as they may have changed specifically for ANCF beams; adapt all your model files regarding Force, Torque, etc.
    - date resolved: **2021-03-20 23:22**\ , date raised: 2020-02-18 
 * Version 1.0.160: resolved Issue 0139: Index (change)
    - description:  change Index to (signed) int and use UIndex in python interface for unsigned parameters
    - **notes:** \ **ATTENTION**\ : this change affects many routines. All TestSuite examples passed the change but there may still be open problems due to this major change.
    - date resolved: **2021-03-20 23:21**\ , date raised: 2019-05-20 
 * Version 1.0.159: resolved Issue 0606: resolve errors 32bit testsuite (test)
    - description:  add extra tolerances for 32bit
    - date resolved: **2021-03-20 23:19**\ , date raised: 2021-03-20 
 * Version 1.0.158: :textred:`resolved BUG 0575` : new genAlpha solver 
    - description:  new generalized alpha/implicit trapezoidal solver does not call solver user functions; Solution: derive CSolverImplicitSecondOrderTimeIntNew from CSolverBase and add user functions on top
    - **notes:** solved by removing old solver structure; new solver fully supports user functions now
    - date resolved: **2021-03-18 21:34**\ , date raised: 2021-02-08 
 * Version 1.0.157: resolved Issue 0602: PostNewton step size recommendation (extension)
    - description:  add step recommendation as outcome of PostNewton function to improve contact and friction accuracy
    - date resolved: **2021-03-18 21:33**\ , date raised: 2021-03-18 
 * Version 1.0.156: resolved Issue 0601: mbs.postNewtonUserFunction (extension)
    - description:  add function PostNewton(...) to be called after step update (Newton or explicit step)
    - date resolved: **2021-03-18 21:33**\ , date raised: 2021-03-18 
 * Version 1.0.155: resolved Issue 0600: ImplicitSecondOrderSolver (cleanup)
    - description:  remove old solver
    - date resolved: **2021-03-18 21:33**\ , date raised: 2021-03-18 
 * Version 1.0.154: resolved Issue 0598: rigidBodyUtilities (extension)
    - description:  add G matrices for Rxyz (Tait-Bryan angles) and also time derivatives of G
    - date resolved: **2021-03-18 17:05**\ , date raised: 2021-03-18 
 * Version 1.0.153: resolved Issue 0594: CMS rotations (extension)
    - description:  test and extend CMS / ObjectFFRFreducedOrder object for other rotation parameterizations (Tait-Bryan and rotation vector/Lie group) such that they work with explicit codes
    - date resolved: **2021-03-18 17:04**\ , date raised: 2021-02-24 
 * Version 1.0.152: resolved Issue 0597: ObjectRigidBody (description)
    - description:  fix description of equations of motion (missing m) and add steps in derivation
    - date resolved: **2021-03-18 08:22**\ , date raised: 2021-03-18 
 * Version 1.0.151: resolved Issue 0283: cylinder with hole (new feature)
    - description:  add TriangleList for cylinder with hole
    - **notes:** not implemented, because it can be easily created with GraphicsDataSolidOfRevolution
    - date resolved: **2021-03-16 16:59**\ , date raised: 2019-12-07 
 * Version 1.0.150: resolved Issue 0396: description (description)
    - description:  add latex description for ObjectFFRF
    - date resolved: **2021-03-16 16:57**\ , date raised: 2020-05-16 
 * Version 1.0.149: resolved Issue 0394: description (description)
    - description:  add latex description for ObjectSuperElement
    - date resolved: **2021-03-16 16:57**\ , date raised: 2020-05-16 
 * Version 1.0.148: resolved Issue 0595: ObjectFFRFreducedOrder (extension)
    - description:  add general nodeType to AddObjectFFRFreducedOrderWithUserFunctions
    - date resolved: **2021-03-16 16:56**\ , date raised: 2021-03-14 
 * Version 1.0.147: resolved Issue 0461: ObjectRigidBody (check)
    - description:  check discription of output variables
    - date resolved: **2021-03-16 16:55**\ , date raised: 2020-11-12 
 * Version 1.0.146: resolved Issue 0596: GetRigidBodyNode (extension)
    - description:  add function GetRigidBodyNode into rigidBodyUtilities, which returns a node item for an according node type, e.g., Euler parameters or rotation vector
    - date resolved: **2021-03-14 14:43**\ , date raised: 2021-03-14 
 * Version 1.0.145: resolved Issue 0583: FFRF docu (docu)
    - description:  add documentation for FFRF and FFRFreducedOrder (CMS) to documentation
    - date resolved: **2021-03-01 12:15**\ , date raised: 2021-02-16 
 * Version 1.0.144: resolved Issue 0455: FEM help (docu)
    - description:  add comments to ObjectFFRF (Tisserand frame!) and reduced that there are convenient helper functions in FEM, etc. for creating objects
    - date resolved: **2021-02-24 21:21**\ , date raised: 2020-10-13 
 * Version 1.0.143: resolved Issue 0593: Add MacOS support (change)
    - description:  make minor adjustments for MacOS to run setup.py
    - date resolved: **2021-02-22 13:10**\ , date raised: 2021-02-22 
 * Version 1.0.142: :textred:`resolved BUG 0592` : StartRenderer 
    - description:  flag verbose=True not working
    - date resolved: **2021-02-22 10:08**\ , date raised: 2021-02-22 
 * Version 1.0.141: resolved Issue 0590: SensorMarker visualization (extension)
    - description:  add visualization for SensorMarker according to marker position
    - date resolved: **2021-02-19 10:21**\ , date raised: 2021-02-19 
 * Version 1.0.140: resolved Issue 0400: add SensorMarker (extension)
    - description:  add sensor for markers, restricting to position/velocity and rotation/angular velocity
    - date resolved: **2021-02-18 19:15**\ , date raised: 2020-05-20 
 * Version 1.0.139: resolved Issue 0585: UserSensor (extension)
    - description:  add user sensor, which enables the user to add any kind of sensor, specifically to combine several sensor outputs into one single sensor
    - date resolved: **2021-02-18 19:14**\ , date raised: 2021-02-17 
 * Version 1.0.138: resolved Issue 0589: solver updateInitialValues (change)
    - description:  update also initial coordinates in order to avoid jumps in accelerations when continuing simulation
    - date resolved: **2021-02-18 18:15**\ , date raised: 2021-02-18 
 * Version 1.0.137: resolved Issue 0588: Solver file header (change)
    - description:  move writing of solution file and sensor files headers from InitializeSolverPreChecks(...) to InitializeSolverInitialConditions(...) to avoid sensor evaluation for initial configuration
    - date resolved: **2021-02-18 16:23**\ , date raised: 2021-02-18 
 * Version 1.0.136: resolved Issue 0587: ALEANCFCable2D (fix)
    - description:  change mass proportional load to include force in direction of ALE coordinate
    - date resolved: **2021-02-17 18:19**\ , date raised: 2021-02-17 
 * Version 1.0.135: resolved Issue 0586: GeneticOptimization (fix)
    - description:  add special warnings and adaptations in order to catch cases where elitistRatio\*populationSize < 1 and if distanceFactor >= 1
    - date resolved: **2021-02-17 18:17**\ , date raised: 2021-02-17 
 * Version 1.0.134: resolved Issue 0584: parameter variation (extension)
    - description:  add possibility to prescribe set of parameters using list, e.g.,  'mass':[1,2,4,8] instead of of tuple which describes the range: 'mass':(0,6,4)
    - date resolved: **2021-02-16 09:43**\ , date raised: 2021-02-16 
 * Version 1.0.133: resolved Issue 0582: optimization (docu)
    - description:  add parameter variation and genetic optiization to documentation of solvers
    - date resolved: **2021-02-16 08:21**\ , date raised: 2021-02-16 
 * Version 1.0.132: :textred:`resolved BUG 0581` : SetODE2Coordinates_tt 
    - description:  SetODE2Coordinates_tt writes to velocities instead of accelerations
    - date resolved: **2021-02-14 11:12**\ , date raised: 2021-02-14 
 * Version 1.0.131: resolved Issue 0580: ParameterVariation (extension)
    - description:  processing.ParameterVariation(...): write to resultsFile to show progress in resultsMonitor.py
    - date resolved: **2021-02-11 17:49**\ , date raised: 2021-02-11 
 * Version 1.0.130: resolved Issue 0576: add ClearWorkspace (extension)
    - description:  add ClearWorkspace() to basicUtilities which allows simple and save cleanup of globals() in python environment; recommended to be called at beginning of complex models
    - date resolved: **2021-02-10 12:35**\ , date raised: 2021-02-08 
 * Version 1.0.129: resolved Issue 0569: contour text (fix)
    - description:  add space to computation info text before contour plot text
    - date resolved: **2021-02-10 12:35**\ , date raised: 2021-02-05 
 * Version 1.0.128: resolved Issue 0568: Renderer axes (fix)
    - description:  use X(0), Y(1) and Z(2) for axes description to be compliant with Python indexing starting with 0 as well as contour components
    - date resolved: **2021-02-10 12:35**\ , date raised: 2021-02-05 
 * Version 1.0.127: resolved Issue 0579: ClearWorkspace (extension)
    - description:  add ClearWorkspacefunction to exudyn.basicUtilities, which allows to reset global variables in ipython; see example in function description
    - date resolved: **2021-02-10 12:13**\ , date raised: 2021-02-10 
 * Version 1.0.126: resolved Issue 0578: SmoothStep (extension)
    - description:  add SmoothStep function to exudyn.utilities, which produces a smooth step function using cosine
    - date resolved: **2021-02-10 12:13**\ , date raised: 2021-02-10 
 * Version 1.0.125: resolved Issue 0572: void (fix)
    - description:  redundant with issue 568
    - date resolved: **2021-02-10 12:05**\ , date raised: 2021-02-05 
 * Version 1.0.124: resolved Issue 0571: void (fix)
    - description:  redundant with issue 568
    - date resolved: **2021-02-10 12:05**\ , date raised: 2021-02-05 
 * Version 1.0.123: resolved Issue 0570: void (fix)
    - description:  redundant with issue 568
    - date resolved: **2021-02-10 12:05**\ , date raised: 2021-02-05 
 * Version 1.0.122: :textred:`resolved BUG 0577` : PostNewtonStep 
    - description:  perform PostNewtonStep and PostDiscontinuousIterationStep only for active objects
    - date resolved: **2021-02-09 14:00**\ , date raised: 2021-02-09 
 * Version 1.0.121: resolved Issue 0355: generalized alpha (extension)
    - description:  implement version of Brüls and Arnold for generalized alpha solver
    - **notes:**  WARNING: switched to new solver based on displacement increments (Arnold/Bruls,2007), which leads to DIFFERENT (but improved) RESULTS than previous dynamic implicit integrator; new implicit solver now works with ODE1 variables
    - date resolved: **2021-02-08 01:56**\ , date raised: 2020-03-08 
 * Version 1.0.120: resolved Issue 0573: merge solver documentation (docu)
    - description:  merge docu on solver in EXUDYN overview and in solver section
    - date resolved: **2021-02-07 17:33**\ , date raised: 2021-02-07 
 * Version 1.0.119: resolved Issue 0567: solvers description (docu)
    - description:  extend description for equations of motion, explicit and implicit solvers
    - date resolved: **2021-02-04 01:14**\ , date raised: 2021-02-03 
 * Version 1.0.118: resolved Issue 0560: Impl integrator ODE1 (extension)
    - description:  add ODE1 coordinates to implicit integrator
    - date resolved: **2021-02-02 15:16**\ , date raised: 2021-01-26 
 * Version 1.0.117: resolved Issue 0508: implicit Lie group integrator (extension)
    - description:  implement implicit index2/index3 Lie group integrator as python function
    - **notes:** cancelled, because will be directly done in C++
    - date resolved: **2021-02-02 15:15**\ , date raised: 2020-12-17 
 * Version 1.0.116: resolved Issue 0566: memory alloc cnt (check)
    - description:  add control to check whether large amount of memory allocations happen during time integration+test suite
    - date resolved: **2021-02-01 01:38**\ , date raised: 2021-01-29 
 * Version 1.0.115: resolved Issue 0541: objectODE1/2, constraint lists (extension)
    - description:  add lists of ODE1 and ODE2 objects, constraints, etc. in cSystemData in order to speed up processing
    - date resolved: **2021-02-01 01:38**\ , date raised: 2021-01-13 
 * Version 1.0.114: resolved Issue 0557: RK with constraints (extension)
    - description:  add CoordinateConstraints to explict Runge-Kutta solvers
    - **notes:** only ground constraints included for now
    - date resolved: **2021-01-27 17:38**\ , date raised: 2021-01-26 
 * Version 1.0.113: resolved Issue 0558: Lie group tests (test)
    - description:  add Lie group integrator simple tests
    - date resolved: **2021-01-27 12:00**\ , date raised: 2021-01-26 
 * Version 1.0.112: resolved Issue 0550: GraphicsDataArrow (extension)
    - description:  add arrow to graphicsDataUtilities
    - **notes:** also added GraphicsDataBasis(...) for drawing 3 orthogonal basis vectors, GraphicsDataCheckerBoard(...) for simple drawing of checker board background and MergeGraphicsDataTriangleList(...) for merging graphicsData triangle lists
    - date resolved: **2021-01-27 00:10**\ , date raised: 2021-01-17 
 * Version 1.0.111: resolved Issue 0495: add ODE1 coordinates (extension)
    - description:  extend system (Jacobian, etc.) for ODE1 coordinates
    - date resolved: **2021-01-26 13:21**\ , date raised: 2020-12-09 
 * Version 1.0.110: resolved Issue 0556: explicit RK tests (test)
    - description:  add tests for explicit Runge Kutta integrators to TestModels
    - date resolved: **2021-01-26 13:17**\ , date raised: 2021-01-25 
 * Version 1.0.109: resolved Issue 0555: explicit Lie group integrator (extension)
    - description:  add existing Lie group integrator in C++
    - date resolved: **2021-01-26 13:17**\ , date raised: 2021-01-25 
 * Version 1.0.108: resolved Issue 0554: explicit integrator (extension)
    - description:  add explicit integrator with automatic step size control (DOPRI5, ODE23); checkout Section :ref:`sec-explicitsolver`\  for description of explicit solvers and Section :ref:`sec-dynamicsolvertype`\  for available solver types
    - date resolved: **2021-01-25 00:54**\ , date raised: 2021-01-24 
 * Version 1.0.107: resolved Issue 0513: add RK4 integrator (extension)
    - description:  put existing python RK4 integrator into CPP
    - date resolved: **2021-01-25 00:54**\ , date raised: 2020-12-19 
 * Version 1.0.106: resolved Issue 0533: ObjectGenericODE1 (extension)
    - description:  add object ObjectGenericODE1 for generic first order ODEs
    - date resolved: **2021-01-21 17:27**\ , date raised: 2021-01-04 
 * Version 1.0.105: resolved Issue 0553: create physics submodule (extension)
    - description:  create exudyn.physics and add friction functions
    - date resolved: **2021-01-20 10:25**\ , date raised: 2021-01-20 
 * Version 1.0.104: :textred:`resolved BUG 0552` : DrawSystemGraph 
    - description:  does not work with RigidBodySpringDamper due to invalid GenericNodeData number
    - date resolved: **2021-01-19 14:43**\ , date raised: 2021-01-19 
 * Version 1.0.103: resolved Issue 0551: InteractiveDialog (extension)
    - description:  add interactive tkinter dialog and new submodule exudyn.interactive to interact with models
    - date resolved: **2021-01-19 00:26**\ , date raised: 2021-01-19 
 * Version 1.0.102: resolved Issue 0549: show solver name and time (extension)
    - description:  add options to show/hide solver name and current time in render window
    - date resolved: **2021-01-17 17:42**\ , date raised: 2021-01-17 
 * Version 1.0.101: :textred:`resolved BUG 0545` : mbs.WaitForUserToContinue() 
    - description:  call to WaitForUserToContinue() does not always wait for keypress. Check StartRender() function and flag settings
    - date resolved: **2021-01-17 16:55**\ , date raised: 2021-01-15 
 * Version 1.0.100: :textred:`resolved BUG 0548` : SolveDynamic/SolveStatic 
    - description:  option updateInitialValues not working
    - **notes:** corrected SetSystemState call
    - date resolved: **2021-01-17 00:08**\ , date raised: 2021-01-17 
 * Version 1.0.99: resolved Issue 0542: GeneticOptimization (extension)
    - description:  store values continuously to file, add automatic loader and animate optimized values
    - **notes:** added resultsMonitor.py to exudyn module
    - date resolved: **2021-01-15 15:18**\ , date raised: 2021-01-13 
 * Version 1.0.98: resolved Issue 0547: realtimeSimulation (extension)
    - description:  add factor for timeIntegration.simulateInRealtime
    - date resolved: **2021-01-15 15:16**\ , date raised: 2021-01-15 
 * Version 1.0.97: resolved Issue 0546: add __version__ version to module (extension)
    - description:  enable exudyn.__version__ as commonly used in other modules
    - date resolved: **2021-01-15 15:05**\ , date raised: 2021-01-15 
 * Version 1.0.96: resolved Issue 0544: geneticOptimization (extension)
    - description:  add optional argument resultsFile to specify a file for output of results data
    - date resolved: **2021-01-14 23:17**\ , date raised: 2021-01-14 
 * Version 1.0.95: resolved Issue 0543: add results monitor (extension)
    - description:  add resultsMonitor.py to be called from command line for doing continuous visualization of sensors and geneticOptimization output
    - date resolved: **2021-01-14 23:08**\ , date raised: 2021-01-14 
 * Version 1.0.94: resolved Issue 0532: NodeGenericODE1 (extension)
    - description:  add node NodeGenericODE1 for arbitrary number of ODE1 coordinates
    - date resolved: **2021-01-13 20:12**\ , date raised: 2021-01-04 
 * Version 1.0.93: resolved Issue 0531: solidExtrusion (extension)
    - description:  add graphicsData for solid extrusion (prismatic) body; based on 2D point and segment list for flat boundaries
    - date resolved: **2021-01-10 20:43**\ , date raised: 2021-01-04 
 * Version 1.0.92: resolved Issue 0539: RigidBodySpringDamper (extension)
    - description:  add postNewtonStepUserFunction and dataCoordinates
    - date resolved: **2021-01-08 14:34**\ , date raised: 2021-01-08 
 * Version 1.0.91: :textred:`resolved BUG 0537` : Render window 
    - description:  double calling of Render(...) function could happen from RunLoop/Render and glfwSetWindowRefreshCallback (set in InitCreateWindow(...)); check if semaphore would remove visualization problems
    - **notes:** added semaphore but FEM visualization anomalies are still there
    - date resolved: **2021-01-07 11:23**\ , date raised: 2021-01-06 
 * Version 1.0.90: resolved Issue 0385: add solver eigenvalues example (extension)
    - description:  add Examples/solverFunctionsTestEigenvalues  to test suite
    - **notes:** added ComputeODE2EigenvaluesTest.py using new functionality exudyn.solver.ComputeODE2Eigenvalues(...)
    - date resolved: **2021-01-07 11:08**\ , date raised: 2020-05-06 
 * Version 1.0.89: resolved Issue 0494: add all tests (extension)
    - description:  add all TestModel/\*.py to testsuite and also examples before making changes to solver
    - date resolved: **2021-01-07 11:04**\ , date raised: 2020-12-09 
 * Version 1.0.88: resolved Issue 0515: user function connector (extension)
    - description:  add forceUserFunction for ObjectConnectorRigidBodySpringDamper to enable User connector
    - date resolved: **2021-01-07 11:03**\ , date raised: 2020-12-19 
 * Version 1.0.87: resolved Issue 0506: utilities docu (extension)
    - description:  complete documentation for all exudyn python utilities and add unique headers for documentation
    - date resolved: **2021-01-06 22:57**\ , date raised: 2020-12-16 
 * Version 1.0.86: resolved Issue 0530: solidOfRevolution (extension)
    - description:  add graphicsData for solid of revoluation
    - date resolved: **2021-01-06 00:31**\ , date raised: 2021-01-04 
 * Version 1.0.85: resolved Issue 0536: GraphicsDataPlane (extension)
    - description:  add graphicsData for simple rectangular plane with option for checkerboard pattern
    - date resolved: **2021-01-05 22:57**\ , date raised: 2021-01-05 
 * Version 1.0.84: resolved Issue 0535: alternating color for cylinder (extension)
    - description:  add alternatingColor argument in GraphicsDataCylinder for visualization of rotation of cylindric bodies
    - date resolved: **2021-01-05 21:46**\ , date raised: 2021-01-05 
 * Version 1.0.83: resolved Issue 0529: add MainSystem to userFunctions (change)
    - description:  add MainSystem "mbs" to all user functions as first argument (WARNING: this changes ALL user functions!!!
    - date resolved: **2021-01-05 14:31**\ , date raised: 2021-01-04 
 * Version 1.0.82: resolved Issue 0447: test examples (check)
    - description:  test all examples with new index types
    - date resolved: **2021-01-05 14:31**\ , date raised: 2020-09-09 
 * Version 1.0.81: :textred:`resolved BUG 0534` : PlotSensor 
    - description:  PlotSensor crashes for Load sensors because no outputVariableType exists
    - **notes:** added special treatment for load sensors
    - date resolved: **2021-01-04 20:11**\ , date raised: 2021-01-04 
 * Version 1.0.80: resolved Issue 0527: faces transparent (extension)
    - description:  add general transparency flag for faces in visualizationSettings.openGL, switchable with button "T"; allows to make node/marker/object numbers visible
    - date resolved: **2021-01-03 21:53**\ , date raised: 2021-01-03 
 * Version 1.0.79: resolved Issue 0509: ComputeODE2Eigenvalues (test)
    - description:  add example in TestModels
    - date resolved: **2021-01-03 10:44**\ , date raised: 2020-12-18 
 * Version 1.0.78: resolved Issue 0528: textured fonts (extension)
    - description:  use TEXTURED based bitmap fonts based stored in glLists, allowing better interpolation, scalability (currently up to font size 64 without quality drop) and much higher performance
    - date resolved: **2021-01-03 10:29**\ , date raised: 2021-01-03 
 * Version 1.0.77: :textred:`resolved BUG 0526` : solver.ComputeODE2Eigenvalues 
    - description:  dense mode returned unsorted eigenvalues==>add sorting
    - date resolved: **2021-01-03 10:21**\ , date raised: 2021-01-03 
 * Version 1.0.76: resolved Issue 0524: interpret UTF8 (change)
    - description:  add conversion from UTF8 to unicode to interpret most central European characters + some important characters correctly (see Section :ref:`sec-graphicsdata`\ )
    - date resolved: **2021-01-02 20:13**\ , date raised: 2020-12-29 
 * Version 1.0.75: resolved Issue 0525: opengl write UTF8 (extension)
    - description:  use UTF8 encoding in opengl text output
    - date resolved: **2020-12-29 21:00**\ , date raised: 2020-12-29 
 * Version 1.0.74: resolved Issue 0523: show version (extension)
    - description:  show current version info in openGL window; can be switched off with showComputationInfo=False
    - date resolved: **2020-12-27 01:33**\ , date raised: 2020-12-27 
 * Version 1.0.73: resolved Issue 0522: openGl issues (fix)
    - description:  fix positioning problems of coordinate system and contour colorbar
    - **notes:** now using pixel coordinates for info texts and different font sizes
    - date resolved: **2020-12-27 01:31**\ , date raised: 2020-12-27 
 * Version 1.0.72: resolved Issue 0521: useWindowsDisplayScaleFactor (extension)
    - description:  add new option useWindowsDisplayScaleFactor in visualizationSettings.general to include display scaling factor for font sizes
    - date resolved: **2020-12-27 01:25**\ , date raised: 2020-12-27 
 * Version 1.0.71: resolved Issue 0520: useBitmapText (extension)
    - description:  add new option useBitmapText in visualizationSettings.general to activate bitmap fonts (deprecated; now using textured fonts)
    - date resolved: **2020-12-27 01:25**\ , date raised: 2020-12-27 
 * Version 1.0.70: resolved Issue 0516: add bitmap font (extension)
    - description:  add font using OpenGL bitmaps to improve visibility of texts (deprecated, now using textured fonts)
    - date resolved: **2020-12-27 01:23**\ , date raised: 2020-12-21 
 * Version 1.0.69: resolved Issue 0519: correct coordinateSystemSize (change)
    - description:  set visualizationSettings.general.coordinateSystemSize relative to fontSize which scales better with larger screens
    - date resolved: **2020-12-24 01:25**\ , date raised: 2020-12-24 
 * Version 1.0.68: resolved Issue 0518: windows display scaling (extension)
    - description:  include windows display (screen) scaling into drawing of texts to increase visibility on high dpi screens
    - **notes:** added flag in visualizationSettings: general.useWindowsDisplayScaleFactor
    - date resolved: **2020-12-24 00:22**\ , date raised: 2020-12-24 
 * Version 1.0.67: resolved Issue 0511: GeneticOptimization (test)
    - description:  add example in TestModels
    - date resolved: **2020-12-19 23:31**\ , date raised: 2020-12-19 
 * Version 1.0.66: resolved Issue 0510: ParameterVariation (test)
    - description:  add example in TestModels
    - date resolved: **2020-12-19 23:31**\ , date raised: 2020-12-19 
 * Version 1.0.65: resolved Issue 0502: rigidbodyinertia (docu)
    - description:  add description for rigidBodyUtilities class RigidBodyInertia
    - date resolved: **2020-12-19 23:28**\ , date raised: 2020-12-14 
 * Version 1.0.64: :textred:`resolved BUG 0512` : testsuite 
    - description:  EXUDYN build date referred shows wrong path
    - **notes:** refer now to installed module
    - date resolved: **2020-12-19 00:44**\ , date raised: 2020-12-19 
 * Version 1.0.63: resolved Issue 0507: changes (extension)
    - description:  incorporate resolved issues and bugs into theDoc.pdf
    - date resolved: **2020-12-17**\ , date raised: 2020-12-16 
 * Version 1.0.62: resolved Issue 0505: rigidBodyUtilities (extension)
    - description:  add description for RigidBodyInertia class
    - date resolved: **2020-12-17**\ , date raised: 2020-12-16 
 * Version 1.0.61: resolved Issue 0501: geneticOptimization add crossover (extension)
    - description:  added crossover and improved parameters for GeneticOptimization
    - date resolved: **2020-12-14**\ , date raised: 2020-12-14 
 * Version 1.0.60: resolved Issue 0497: genetic algorithm (check)
    - description:  check if stochsearch or genetic algorithm has simpler interface
    - date resolved: **2020-12-14**\ , date raised: 2020-12-10 
 * Version 1.0.59: resolved Issue 0500: FilterSignal (extension)
    - description:  put in signal module, make it working for 1D signals as well
    - date resolved: **2020-12-11**\ , date raised: 2020-12-10 
 * Version 1.0.58: resolved Issue 0492: FEMinterface GetNodesInOrthoCube (extension)
    - description:  add function which returns all nodes lying in cube aligned with global coordinate system, using [pMin, pMax], with tolerance
    - date resolved: **2020-12-11**\ , date raised: 2020-12-08 
 * Version 1.0.57: resolved Issue 0491: FEMinterface GetNodesOnCylinder (extension)
    - description:  add function which returns all nodes lying on specific cylinder, with tolerance
    - date resolved: **2020-12-11**\ , date raised: 2020-12-08 
 * Version 1.0.56: :textred:`resolved BUG 0499` : key V gives error 
    - description:  keypress V for visualizationSettings dialog gives error
    - date resolved: **2020-12-10**\ , date raised: 2020-12-10 
 * Version 1.0.55: :textred:`resolved BUG 0498` : SensorObject position 
    - description:  wrong position shown in sensor
    - date resolved: **2020-12-10**\ , date raised: 2020-12-10 
 * Version 1.0.54: resolved Issue 0484: test DEAP (test)
    - description:  test genetic optimization with DEAP
    - **notes:** too many parameters and too involved to simply include
    - date resolved: **2020-12-10**\ , date raised: 2020-12-04 
 * Version 1.0.53: :textred:`resolved BUG 0493` : CheckForValidFunction 
    - description:  modify / add this check to setParameters; additional if for setting this to 0
    - date resolved: **2020-12-09**\ , date raised: 2020-12-09 
 * Version 1.0.52: :textred:`resolved BUG 0490` : keypress crash 
    - description:  find out causes for crash in keyPress user function; find way to deactivate the user function (set it to 0)
    - date resolved: **2020-12-09**\ , date raised: 2020-12-07 
 * Version 1.0.51: resolved Issue 0389: MainSystem includes (cleanup)
    - description:  put SystemIntegrity item checks into separate file, to reduce includig MainSystem into every .cpp item file
    - date resolved: **2020-12-09**\ , date raised: 2020-05-13 
 * Version 1.0.50: resolved Issue 0357: solver flag prolong solution (extension)
    - description:  add flag for solvers that current state at end of computation is set as initial state for next solving
    - **notes:** added into new python interface of solver
    - date resolved: **2020-12-09**\ , date raised: 2020-03-11 
 * Version 1.0.49: resolved Issue 0489: add gradient background (extension)
    - description:  add according visualization.general option
    - date resolved: **2020-12-06**\ , date raised: 2020-12-06 
 * Version 1.0.48: :textred:`resolved BUG 0488` : problem with coordinate sys 
    - description:  fix problems with drawing of coordinate system: text moves strangely and axes dissappear after rotation
    - date resolved: **2020-12-06**\ , date raised: 2020-12-06 
 * Version 1.0.47: resolved Issue 0487: draw world basis (extension)
    - description:  add option to draw coordinate system at origin (world basis)
    - date resolved: **2020-12-06**\ , date raised: 2020-12-06 
 * Version 1.0.46: resolved Issue 0486: realtime (extension)
    - description:  add flag to time integration to simulate in realtime
    - date resolved: **2020-12-05**\ , date raised: 2020-12-05 
 * Version 1.0.45: resolved Issue 0485: mouse coordinates (extension)
    - description:  store mouse coordinates in renderState
    - date resolved: **2020-12-05**\ , date raised: 2020-12-05 
 * Version 1.0.44: resolved Issue 0467: mouse coordinates (extension)
    - description:  show mouse coordinates in render window (without transformation)
    - date resolved: **2020-12-05**\ , date raised: 2020-11-19 
 * Version 1.0.43: resolved Issue 0325: key callback (extension)
    - description:  add key callback function into graphics module to enable interactive settings, etc.; transfer latin letters, SHIFT, CTRL, ALT, 0-9,A-Z,.,SPACE as ASCII code
    - date resolved: **2020-12-05**\ , date raised: 2020-01-26 
 * Version 1.0.42: resolved Issue 0460: test accelerations (test)
    - description:  test GetODE2Coordinates_tt, nodal accelerations and rigidbody2D/3D accelerations
    - date resolved: **2020-12-04**\ , date raised: 2020-11-12 
 * Version 1.0.41: resolved Issue 0482: store model view (extension)
    - description:  store renderState in exudyn.sys dictionary after exu.StopRenderer() for subsequent simulations
    - date resolved: **2020-12-03**\ , date raised: 2020-12-03 
 * Version 1.0.40: resolved Issue 0478: link examples (docu)
    - description:  automatically add links to examples in thedoc
    - date resolved: **2020-12-03**\ , date raised: 2020-12-02 
 * Version 1.0.39: resolved Issue 0477: links in theDoc (extension)
    - description:  add links between user functions, add labels to item sections
    - date resolved: **2020-12-03**\ , date raised: 2020-12-02 
 * Version 1.0.38: resolved Issue 0463: accelerations (extension)
    - description:  add accelerations Outputvariable to Super elements
    - date resolved: **2020-12-03**\ , date raised: 2020-11-18 
 * Version 1.0.37: resolved Issue 0481: eigenvalue solver (extension)
    - description:  add eigenvalue computation interface for mbs in python
    - date resolved: **2020-12-02**\ , date raised: 2020-12-02 
 * Version 1.0.36: resolved Issue 0480: python solver (extension)
    - description:  add solver interfaces in python for MainSolverStatic and MainSolverImplicitSecondOrder, helping to retrieve solver data and to make solvers accessible for users
    - date resolved: **2020-12-02**\ , date raised: 2020-12-02 
 * Version 1.0.35: resolved Issue 0479: solver return (extension)
    - description:  add return value to solvers and copy solver structures to mbs.sys variables after finishing
    - **notes:** added python interfaces and kept old cpp solvers
    - date resolved: **2020-12-02**\ , date raised: 2020-12-02 
 * Version 1.0.34: resolved Issue 0469: userfunctions (extension)
    - description:  put user function generation in objectdefinition, with seperate U userfunction flag - this will automatically document the user function parameters (AND return values); this improves documentation and adds a unique interface in C++ using exception handling as well as GIL handling
    - date resolved: **2020-12-02**\ , date raised: 2020-11-20 
 * Version 1.0.33: resolved Issue 0458: graphicsDataUserFunction (docu)
    - description:  add example to docu in ObjectGround and GenericODE2 and add more accurate docu to ALL python user functions
    - date resolved: **2020-12-02**\ , date raised: 2020-11-10 
 * Version 1.0.32: resolved Issue 0428: queue user functions (extension)
    - description:  implement drawing user functions as global function similar to PyProcessQueue, in order to avoid messing up the CSystem and visualization modules
    - date resolved: **2020-12-02**\ , date raised: 2020-06-26 
 * Version 1.0.31: resolved Issue 0476: add RequireVersion (extension)
    - description:  functionality to allow to add a simple check to see if the installed version meets the requirements
    - date resolved: **2020-11-30**\ , date raised: 2020-11-30 
 * Version 1.0.30: resolved Issue 0475: rolling disc ext (extension)
    - description:  add force on ground and moving ground for ObjectJointRollingDisc
    - **notes:** needs to be tested further
    - date resolved: **2020-11-29**\ , date raised: 2020-11-26 
 * Version 1.0.29: resolved Issue 0474: auto compilation (check)
    - description:  check automatic compilation; check version in wheels; check linux wheels
    - **notes:** linux wheels can not be built with admin rights
    - date resolved: **2020-11-29**\ , date raised: 2020-11-25 
 * Version 1.0.28: resolved Issue 0473: no glfw option (extension)
    - description:  add simple option in setup.py to deactivate glfw both in setup.py as well as in C++ part
    - date resolved: **2020-11-29**\ , date raised: 2020-11-25 
 * Version 1.0.27: resolved Issue 0457: GetVersionString (extension)
    - description:  put into docu with pybindings
    - date resolved: **2020-11-29**\ , date raised: 2020-11-07 
 * Version 1.0.26: resolved Issue 0472: examples in utilities (extension)
    - description:  activate lstlisting for examples
    - date resolved: **2020-11-25**\ , date raised: 2020-11-25 
 * Version 1.0.25: resolved Issue 0468: test WSL2 (test)
    - description:  test compilation on WSL2 - Windows subsystem for Linux
    - **notes:** WSL2 now used to automatically create linux wheels
    - date resolved: **2020-11-21**\ , date raised: 2020-11-19 
 * Version 1.0.24: :textred:`resolved BUG 0465` : SC.GetSystem(..) 
    - description:  raises RuntimeError: should return reference instead of copy
    - date resolved: **2020-11-21**\ , date raised: 2020-11-18 
 * Version 1.0.23: resolved Issue 0446: NodeIndex in arrays (check)
    - description:  use additional functionality to enable index type checks also in arrays, e.g., ArrayIndex of node numbers
    - **notes:** not needed for now
    - date resolved: **2020-11-21**\ , date raised: 2020-09-09 
 * Version 1.0.22: resolved Issue 0383: pybind11 submodule (extension)
    - description:  used for advanced functions, not necessarily included in exudyn or make other module
    - **notes:** not needed for now
    - date resolved: **2020-11-21**\ , date raised: 2020-05-06 
 * Version 1.0.21: resolved Issue 0191: Newton lambda (check)
    - description:  Check whether Newton can be implemented as lambda-function
    - **notes:** not needed for now
    - date resolved: **2020-11-21**\ , date raised: 2019-06-17 
 * Version 1.0.20: resolved Issue 0466: main/bin (change)
    - description:  remove main/bin from github and from Tools folder
    - date resolved: **2020-11-19**\ , date raised: 2020-11-19 
 * Version 1.0.19: resolved Issue 0464: processing module (extension)
    - description:  create processing module for parameter variation and optimization using multiprocessing library
    - date resolved: **2020-11-18**\ , date raised: 2020-11-18 
 * Version 1.0.18: resolved Issue 0462: AVX Celeron problems (docu)
    - description:  add info to documentation - FAQ AND common problems and installation instructions that CPUs without AVX support only work with 32bit version
    - date resolved: **2020-11-18**\ , date raised: 2020-11-16 
 * Version 1.0.17: resolved Issue 0459: lie group utilities (extension)
    - description:  add documented lie group utilities to exudyn (python) lib
    - date resolved: **2020-11-11**\ , date raised: 2020-11-11 
    - resolved by: S. Holzinger
 * Version 1.0.16: resolved Issue 0454: add item graph (extension)
    - description:  add graph containing nodes, objects, etc.
    - date resolved: **2020-10-08**\ , date raised: 2020-10-08 
 * Version 1.0.15: resolved Issue 0453: systemdata.NumberOfSensors (extension)
    - description:  add access function for systemdata.NumberOfSensors()
    - date resolved: **2020-10-08**\ , date raised: 2020-10-08 
 * Version 1.0.14: resolved Issue 0449: MT ngsolve (extension)
    - description:  add NGsolve multithreading library (task manager)
    - **notes:** first tests made
    - date resolved: **2020-09-16**\ , date raised: 2020-09-15 
 * Version 1.0.13: resolved Issue 0330: correct ODE2RHS (change)
    - description:  correct ODE2RHS to ODE2Terms in objects because it is left-hand-side
    - **notes:** changed object computation function from RHS to LHS, as it always computed the LHS (the system.cpp function ComputeODE2RHS then puts it to RHS)
    - date resolved: **2020-09-10**\ , date raised: 2020-02-03 
 * Version 1.0.12: resolved Issue 0435: check runtimeError (check)
    - description:  check if exception runtimeerror works for all catch cases (test in windows?)
    - date resolved: **2020-09-09**\ , date raised: 2020-07-21 
 * Version 1.0.11: resolved Issue 0445: remove GetItemByName() (change)
    - description:  remove GetNodeByName, GetObjectByName, etc. from C++ interface; already disabled in python interface before
    - date resolved: **2020-09-08**\ , date raised: 2020-09-08 
 * Version 1.0.10: resolved Issue 0288: Item::CallFunction (change)
    - description:  Disable Item::CallFunction functionality from EXUDYN; either outputvariables can be used, or some functions are automatically created including the documentation
    - **notes:** already removed from python interface earlier
    - date resolved: **2020-09-08**\ , date raised: 2019-12-10 
 * Version 1.0.9: resolved Issue 0443: SensorObject (warning)
    - description:  add error message, if sensorobject is used for a body (and check if SensorBody excepts object other than body
    - **notes:** added test for SensorObject if attached to body
    - date resolved: **2020-09-04**\ , date raised: 2020-09-03 
 * Version 1.0.8: :textred:`resolved BUG 0442` : difference MSC and setuptools 
    - description:  compilation with MSC and setuptools gives different results
    - **notes:** problem with VS2019 compilation of Eigen; resolved by removing VS2019 installation
    - date resolved: **2020-08-25**\ , date raised: 2020-08-24 
 * Version 1.0.7: resolved Issue 0431: auto create dirs (extension)
    - description:  automatically create dictionaries if they do not exist
    - date resolved: **2020-08-25**\ , date raised: 2020-07-01 
 * Version 1.0.6: resolved Issue 0439: setuptools (extension)
    - description:  use setuptools for installation
    - date resolved: **2020-08-17**\ , date raised: 2020-08-13 
 * Version 1.0.5: resolved Issue 0381: test pybind11_2020 (test)
    - description:  downloaded in Download folder
    - date resolved: **2020-08-17**\ , date raised: 2020-05-06 
 * Version 1.0.4: resolved Issue 0378: setup tools (extension)
    - description:  use setup tools to install EXUDYN on local user accounts; use installed python version to decide which version to install
    - date resolved: **2020-08-17**\ , date raised: 2020-05-04 
 * Version 1.0.3: resolved Issue 0441: remove WorkingRelease path (change)
    - description:  do not include WorkingRelease to sys.path any more, but require installation of modules
    - date resolved: **2020-08-14**\ , date raised: 2020-08-14 
 * Version 1.0.2: resolved Issue 0440: exudyn package (extension)
    - description:  make a package with sub .py files in exudyn package - requires renaming of C++ module
    - date resolved: **2020-08-14**\ , date raised: 2020-08-13 
 * Version 1.0.1: resolved Issue 0438: UBUNTU (extension)
    - description:  adapt setup.py and implementation for gcc and UBUNTU
    - date resolved: **2020-08-13**\ , date raised: 2020-08-13 
 * Version 1.0.0: :textred:`resolved BUG 0434` : CheckSystemIntegrity 
    - description:  gives wrong node, marker, etc. numbers for some checks
    - date resolved: **2020-07-20**\ , date raised: 2020-07-20 

***********
Version 0.1
***********

 * Version 0.1.367: resolved Issue 0433: #pragma once (change)
    - description:  remove #pragma once directives, not compatible with gcc
    - date resolved: **2020-07-20**\ , date raised: 2020-07-20 
 * Version 0.1.366: :textred:`resolved BUG 0432` : FFRF object bug 
    - description:  wrong results in ObjectFFRF in case of refpos!=0
    - date resolved: **2020-07-02**\ , date raised: 2020-07-02 
 * Version 0.1.365: resolved Issue 0429: add stress modes (extension)
    - description:  add additional modes in ObjectFFRFreducedOrder to visualize stresses and strains
    - date resolved: **2020-07-01**\ , date raised: 2020-07-01 
 * Version 0.1.364: resolved Issue 0419: visualization user function (extension)
    - description:  add possibility of user function for visualization: is called on cSystem side to generate graphicsData lists or via a thread-safe callback
    - date resolved: **2020-06-26**\ , date raised: 2020-05-29 
 * Version 0.1.363: resolved Issue 0371: prestep py function (extension)
    - description:  Add prestep function as function using mbs (MainSystem)
    - date resolved: **2020-06-24**\ , date raised: 2020-04-10 
 * Version 0.1.362: resolved Issue 0284: utilities docu (docu)
    - description:  Add documentation for exudynUtilities.py
    - date resolved: **2020-06-24**\ , date raised: 2019-12-07 
 * Version 0.1.361: resolved Issue 0259: output variable connector (extension)
    - description:  add consistent output variables for connectors
    - date resolved: **2020-06-24**\ , date raised: 2019-08-30 
 * Version 0.1.360: resolved Issue 0427: RollingDiscPenalty (extension)
    - description:  implement RollingDisc model with penalty formulation and friction
    - date resolved: **2020-06-22**\ , date raised: 2020-06-19 
 * Version 0.1.359: resolved Issue 0426: RollingDisc (extension)
    - description:  implement RollingDisc model
    - date resolved: **2020-06-22**\ , date raised: 2020-06-19 
 * Version 0.1.358: resolved Issue 0425: HasVelocityEquations() (check)
    - description:  check if HasVelocityEquations() is really needed
    - date resolved: **2020-06-19**\ , date raised: 2020-06-19 
 * Version 0.1.357: resolved Issue 0420: SuperElement gravity (extension)
    - description:  add gravity to superelements
    - date resolved: **2020-06-09**\ , date raised: 2020-06-03 
 * Version 0.1.356: resolved Issue 0405: MarkerSuperElementReducedOrderRigidBody (extension)
    - description:  Implement averaging multinode marker for position and orientation for reduced order elements
    - date resolved: **2020-06-09**\ , date raised: 2020-05-21 
 * Version 0.1.355: resolved Issue 0281: add STL import (new feature)
    - description:  add exudynGraphics function for STL faces import
    - date resolved: **2020-06-09**\ , date raised: 2019-12-05 
 * Version 0.1.354: resolved Issue 0422: contour maxValue (extension)
    - description:  add second mode to contour auto range, which does not reduce the range
    - date resolved: **2020-06-07**\ , date raised: 2020-06-07 
 * Version 0.1.353: resolved Issue 0421: standard views (extension)
    - description:  add key shortcuts for standard views
    - date resolved: **2020-06-07**\ , date raised: 2020-06-07 
 * Version 0.1.352: resolved Issue 0418: read only parameters (check)
    - description:  raise exception if read only values are attempted to be overwritten==> currently, read only parameters are ignored!
    - date resolved: **2020-06-01**\ , date raised: 2020-05-28 
 * Version 0.1.351: resolved Issue 0417: visualization shortnames (extension)
    - description:  add shortnames, e.g., VMass1D for visualization objects
    - date resolved: **2020-06-01**\ , date raised: 2020-05-27 
 * Version 0.1.350: resolved Issue 0411: MarkerSuperElementPosition (extension)
    - description:  cleanup MarkerSuperElementPosition Matrix objects and test
    - date resolved: **2020-06-01**\ , date raised: 2020-05-25 
 * Version 0.1.349: resolved Issue 0408: ObjectConnectorRelativeRotation (extension)
    - description:  add connector with drive functionality, which constrains relative rotation (or translation) in the local frame of one Marker; use gear ratio + offset; enables gears, drives, etc.
    - date resolved: **2020-06-01**\ , date raised: 2020-05-22 
 * Version 0.1.348: resolved Issue 0404: MarkerSuperElementRigidBody (extension)
    - description:  Implement averaging multinode maker for position and orientation
    - date resolved: **2020-06-01**\ , date raised: 2020-05-21 
 * Version 0.1.347: resolved Issue 0395: description (description)
    - description:  add latex description for MarkerSuperElementPosition
    - date resolved: **2020-06-01**\ , date raised: 2020-05-16 
 * Version 0.1.346: resolved Issue 0414: ObjectFFRFreducedOrder EP (extension)
    - description:  add Euler Parameter constraint to ObjectFFRFreducedOrder
    - date resolved: **2020-05-29**\ , date raised: 2020-05-26 
 * Version 0.1.345: resolved Issue 0338: check accessFunctionTypes (extension)
    - description:  add automatized marker/force/accessFunctionType checks using the fact that same bits are used in types
    - date resolved: **2020-05-28**\ , date raised: 2020-02-18 
 * Version 0.1.344: resolved Issue 0329: check markers (extension)
    - description:  add integrity check if node/body implements necessary access functions for marker
    - date resolved: **2020-05-28**\ , date raised: 2020-02-02 
 * Version 0.1.343: resolved Issue 0416: check Markers (extension)
    - description:  add check (WARNING) if joint is applied to two markers directing to identical nodes or bodies
    - **notes:** not possible for FFRF and generic objects
    - date resolved: **2020-05-27**\ , date raised: 2020-05-27 
 * Version 0.1.342: :textred:`resolved BUG 0415` : CNodeRigidBody2D bug 
    - description:  CNodeRigidBody2D misses OutputVariables Rotation, RotationMatrix, AngularVelocity(Local)
    - date resolved: **2020-05-26**\ , date raised: 2020-05-26 
 * Version 0.1.341: resolved Issue 0407: implement FFRF tisserand frame (extension)
    - description:  implement Phit.T\*M\*c_F=0 and xRefTilde.T\*M\*c_F=0 as FFRF constraint for rigid body motion
    - **notes:** only possible as ConnectorCoordinateVector constraint externally
    - date resolved: **2020-05-26**\ , date raised: 2020-05-22 
 * Version 0.1.340: resolved Issue 0388: add 1D nodes and objects (extension)
    - description:  add 1D nodes/objects for drive-train applications; add optional visualization offset position p0, rotation A0 and transformation q->(u3D, theta3d), which transforms the 1D coordinate to 3D translation and rotation
    - date resolved: **2020-05-26**\ , date raised: 2020-05-08 
 * Version 0.1.339: resolved Issue 0387: add velocity markers (extension)
    - description:  add MarkerNodeRotationCoordinate (velocity level=angular velocity), with option for local frame and for MarkerRotation matrix, this enables to couple single (angular) velocities and to couple drives, etc. between 1D objects and 3D objects
    - date resolved: **2020-05-26**\ , date raised: 2020-05-08 
 * Version 0.1.338: resolved Issue 0412: ConnectorCoordinateVector (extension)
    - description:  implement coordinate vector constraint applied to a body; can be used as generic joint (with user function)
    - date resolved: **2020-05-25**\ , date raised: 2020-05-25 
 * Version 0.1.337: resolved Issue 0409: MarkerObjectCoordinates (extension)
    - description:  add new Marker MarkerObjectCoordinates, which applies to all coordinates of an object; this enables generic constraints on object coordinates (nodal coordinates may be added in future)
    - date resolved: **2020-05-25**\ , date raised: 2020-05-24 
 * Version 0.1.336: resolved Issue 0402: add NGsolve interface (extension)
    - description:  add NGsolve to FEMinterface to create mechanical body from geo, some options; create M, K, nodeList, elements and surface; add surfaces for specific boundaries
    - date resolved: **2020-05-22**\ , date raised: 2020-05-21 
 * Version 0.1.335: resolved Issue 0399: ObjectContactFrictionCircleCable2D (correct)
    - description:  ObjectContactFrictionCircleCable2D: contact stiffness wrong comment
    - date resolved: **2020-05-22**\ , date raised: 2020-05-20 
 * Version 0.1.334: resolved Issue 0393: clean up ObjectGenericODE2 (cleanup)
    - description:  remove ffrf from ObjectGenericODE2
    - date resolved: **2020-05-21**\ , date raised: 2020-05-16 
 * Version 0.1.333: resolved Issue 0398: FEM interface (description)
    - description:  add FEMinterface python class for mesh and system matrix import, surface mesh extraction, mode computation, export to ObjectFFRF etc.
    - date resolved: **2020-05-17**\ , date raised: 2020-05-16 
 * Version 0.1.332: resolved Issue 0397: tests FFRF (description)
    - description:  add TestModels for FFRF and FFRFreducedOrder
    - date resolved: **2020-05-17**\ , date raised: 2020-05-16 
 * Version 0.1.331: resolved Issue 0392: MarkerSuperElementRigidReducedOrder (extension)
    - description:  add MarkerSuperElementRigidReducedOrder for reducedOrder objects
    - date resolved: **2020-05-17**\ , date raised: 2020-05-16 
 * Version 0.1.330: resolved Issue 0391: add MarkerSuperElementPosition (extension)
    - description:  add MarkerSuperElementPosition
    - date resolved: **2020-05-16**\ , date raised: 2020-05-16 
 * Version 0.1.329: resolved Issue 0125: LinkedDataMatrix (new feature)
    - description:  Implement LinkedDataMatrix
    - date resolved: **2020-05-16**\ , date raised: 2019-05-13 
 * Version 0.1.328: resolved Issue 0386: add superelement (extension)
    - description:  add intermediate object, which offers access functions for node-position/velocity,/jacobian; object redirects either to nodes or uses the mode basis (FFRF) to compute position of virtual node
    - date resolved: **2020-05-15**\ , date raised: 2020-05-07 
 * Version 0.1.327: resolved Issue 0382: add FFRF object (extension)
    - description:  enabling full and reduced set of coordinates
    - date resolved: **2020-05-15**\ , date raised: 2020-05-06 
 * Version 0.1.326: resolved Issue 0368: Implement reduced FFRF (extension)
    - description:  Implement reduced FFRF in GenericODE2 by adding a transformation matrix
    - date resolved: **2020-05-15**\ , date raised: 2020-04-10 
 * Version 0.1.325: resolved Issue 0379: auto testSuite (extension)
    - description:  integrate test suite into workflow for generating WorkingReleaseXYZ; copy output.log to WorkingRelease folders
    - date resolved: **2020-05-06**\ , date raised: 2020-05-05 
 * Version 0.1.324: resolved Issue 0367: Implement FFRF (extension)
    - description:  Implement FFRF in GenericODE2
    - **notes:** internal folder only
    - date resolved: **2020-05-06**\ , date raised: 2020-04-10 
 * Version 0.1.323: resolved Issue 0377: testSuite (extension)
    - description:  integrate test suite into workflow for generating WorkingReleaseXYZ; copy output.log to WorkingRelease folders; make directory with named (versionNr) logs to quickly find problems
    - date resolved: **2020-05-05**\ , date raised: 2020-05-04 
 * Version 0.1.322: resolved Issue 0376: user functions catch (extension)
    - description:  catch exceptions of user functions, such that uunidentified errors in time integration can be located
    - date resolved: **2020-04-25**\ , date raised: 2020-04-24 
 * Version 0.1.321: resolved Issue 0373: loadVectorUserFunction (check)
    - description:  loadVectorUserFunction does not work with np.array as return type
    - **notes:** changed std::function return type from StdVector3D(=std::array) to StdVector(=std::vector), which works automatically with numpy arrays
    - date resolved: **2020-04-24**\ , date raised: 2020-04-14 
 * Version 0.1.320: resolved Issue 0336: euler parameters (check)
    - description:  check input of euler paramters: is norm of EP given for 1-e8
    - date resolved: **2020-04-24**\ , date raised: 2020-02-17 
 * Version 0.1.319: resolved Issue 0375: rigid body COM (extension)
    - description:  add center of mass COM to rigid body; extend equations of motion
    - date resolved: **2020-04-22**\ , date raised: 2020-04-22 
 * Version 0.1.318: resolved Issue 0361: check GenericJoint (check)
    - description:  check generic joint jacobian: in case that not all translational components are fixed, the jacobian entries should be excluded
    - date resolved: **2020-04-22**\ , date raised: 2020-04-09 
 * Version 0.1.317: resolved Issue 0339: add LoadController (extension)
    - description:  add possibility to add controllers with sensors as input and a user python function
    - date resolved: **2020-04-22**\ , date raised: 2020-02-18 
 * Version 0.1.316: resolved Issue 0370: Implement DH parameters (extension)
    - description:  use DH-parameters to set up link relative to previous body
    - date resolved: **2020-04-20**\ , date raised: 2020-04-10 
 * Version 0.1.315: resolved Issue 0374: SetRenderState (extension)
    - description:  add function SetRenderState in analogy to GetRenderState in order to set previously saved openGL state
    - date resolved: **2020-04-14**\ , date raised: 2020-04-14 
 * Version 0.1.314: resolved Issue 0360: 3D rotation (extension)
    - description:  make incremental rotation with right-mouse-button relative to current configuration
    - date resolved: **2020-04-14**\ , date raised: 2020-03-20 
 * Version 0.1.313: resolved Issue 0366: Add FFRF GenericODE2 (extension)
    - description:  Add FFRF mode to GenericODE2, using the first node as RigidBodyNode to represent the floating reference frame
    - date resolved: **2020-04-11**\ , date raised: 2020-04-10 
 * Version 0.1.312: resolved Issue 0365: add SphericalJoint (extension)
    - description:  add separate spherical joint with option to constrain one of the 3 axes
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.311: resolved Issue 0364: add multinodal marker (extension)
    - description:  add multinodal marker with weights for GenericODE2 objects
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.310: resolved Issue 0363: draw springs 3d (extension)
    - description:  add 3d helical spring drawing
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.309: resolved Issue 0362: unique drawing (extension)
    - description:  make unique line and 3d drawing for markers, sensors and loads
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.308: resolved Issue 0359: vis nodes (extension)
    - description:  add option to visualize nodes as spheres
    - date resolved: **2020-04-10**\ , date raised: 2020-03-20 
 * Version 0.1.307: resolved Issue 0358: vis nodes (extension)
    - description:  visualize 3D nodes as 3 circles
    - date resolved: **2020-04-10**\ , date raised: 2020-03-20 
 * Version 0.1.306: resolved Issue 0297: ALEANCFCable2D mass terms (check)
    - description:  check if mass terms in ALEANCFCable2D (9th column/row)
    - **notes:** terms are correct, but lead to instability at high velocities
    - date resolved: **2020-04-10**\ , date raised: 2019-12-16 
 * Version 0.1.305: resolved Issue 0225: SlidingJointRigid (extension)
    - description:  Add functionality for rigid sliding joint or add a flag for sliding joint to do both options
    - **notes:** not needed for now
    - date resolved: **2020-04-10**\ , date raised: 2019-07-10 
 * Version 0.1.304: resolved Issue 0356: sensor tutorial (extension)
    - description:  add sensor to tutorial
    - date resolved: **2020-03-13**\ , date raised: 2020-03-08 
 * Version 0.1.303: resolved Issue 0302: benchmark problems (verification)
    - description:  implement benchmark problems of iftomm web page and from papers Bruls/Arnold, Terze, etc.
    - date resolved: **2020-03-08**\ , date raised: 2019-12-26 
 * Version 0.1.302: resolved Issue 0352: exceptions pybind (extension)
    - description:  check whether lambda functions do not catch appropriately exceptions in pybind ==> use manual try catch mechanisms to catch exceptions
    - date resolved: **2020-03-06**\ , date raised: 2020-03-03 
 * Version 0.1.301: resolved Issue 0328: check ANCFALE (check)
    - description:  check if access functions implemented correctly: local jacobians include only 3x8 instead of 3x9 components?
    - date resolved: **2020-03-06**\ , date raised: 2020-02-02 
 * Version 0.1.300: resolved Issue 0353: AddItem (extension)
    - description:  unify AddItem(dict/py::object) into one function
    - date resolved: **2020-03-05**\ , date raised: 2020-03-04 
 * Version 0.1.299: resolved Issue 0351: exudynFast (extension)
    - description:  name all versions exudyn, because of unresolvable conflicts in utilities includes
    - date resolved: **2020-03-05**\ , date raised: 2020-03-03 
 * Version 0.1.298: resolved Issue 0350: Add ODE2equations (extension)
    - description:  add ObjectODE2equations with GenericODE2/NodePoint/NodePoint2D nodes
    - date resolved: **2020-03-05**\ , date raised: 2020-03-02 
 * Version 0.1.297: resolved Issue 0348: GetObjectOutputBody (extension)
    - description:  GetObjectOutputBody and similar functions shall not be callable if system is inconsistent
    - date resolved: **2020-03-05**\ , date raised: 2020-02-24 
 * Version 0.1.296: resolved Issue 0346: evaluate autodiff (check)
    - description:  evaluate netgen autodiff and autodiff.github.io for straightforward auto-differentiation of ODE2RHS functions
    - date resolved: **2020-03-05**\ , date raised: 2020-02-23 
 * Version 0.1.293: resolved Issue 0345: time in connector UF (extension)
    - description:  add time to connector user functions, to allow time dependent trajectories in spring dampers
    - date resolved: **2020-02-23**\ , date raised: 2020-02-22 
 * Version 0.1.292: resolved Issue 0335: docu examples+equations (extension)
    - description:  add multiline equations section and example section in the object description
    - date resolved: **2020-02-23**\ , date raised: 2020-02-16 
 * Version 0.1.291: resolved Issue 0309: GenericRigidBodyJoint2D (extension)
    - description:  Add generic joint with local transformation matrix of marker1; enable fixed or free ux,uy and phi in the joint, relative to marker1 coordinate system
    - date resolved: **2020-02-23**\ , date raised: 2020-01-10 
 * Version 0.1.290: resolved Issue 0287: Std::array (extension)
    - description:  check if performance of python interfaces can be improved using std::array; add typdefs for std::array<Real,3>, etc.
    - date resolved: **2020-02-23**\ , date raised: 2019-12-09 
 * Version 0.1.289: resolved Issue 0343: extend CoordinateConstraint (extension)
    - description:  extend CoordinateConstraint user function: value0/1, value_t0/1, factorValue1
    - **notes:** NOT COMPLETED: do this extension instead in a separate user coordinate constraint as it requires also the jacobians to be computed
    - date resolved: **2020-02-22**\ , date raised: 2020-02-20 
 * Version 0.1.288: resolved Issue 0344: initialDisplacements (change)
    - description:  change the misleading name initialDisplacements to initialCoordinates in all nodes for consitency reasons
    - date resolved: **2020-02-21**\ , date raised: 2020-02-21 
 * Version 0.1.287: resolved Issue 0342: add load sensor (new feature)
    - description:  add load sensor which measures loads especially if modified in user defined loads
    - date resolved: **2020-02-19**\ , date raised: 2020-02-19 
 * Version 0.1.286: resolved Issue 0341: add bodyFixed loads (extension)
    - description:  add bodyFixed (local / follower) forces and torques
    - date resolved: **2020-02-19**\ , date raised: 2020-02-19 
 * Version 0.1.285: resolved Issue 0340: bodyFixed force (extension)
    - description:  use markers bodyFixed property to realize local forces - make bodyFixed globally available in markers and add to computation of loads in csystem.cpp
    - **notes:** moved bodyFixed property to loads; see issue #341
    - date resolved: **2020-02-19**\ , date raised: 2020-02-19 
 * Version 0.1.284: resolved Issue 0324: visualize sensors (extension)
    - description:  add visualization to sensors (draw as circle with cross)
    - date resolved: **2020-02-18**\ , date raised: 2020-01-25 
 * Version 0.1.283: resolved Issue 0301: exudynUtilities (extension)
    - description:  split up exudynUtilities into separate files
    - date resolved: **2020-02-14**\ , date raised: 2019-12-26 
 * Version 0.1.282: resolved Issue 0334: add RigidBodySpringDamper (new feature)
    - description:  add a generalization for CartesianSpringDamper, using local coordinate systems and coupling all local translations and rotations
    - date resolved: **2020-02-12**\ , date raised: 2020-02-12 
 * Version 0.1.281: resolved Issue 0319: PyError in C++ (check)
    - description:  check why PyError is not working any more properly
    - **notes:** did not show up again
    - date resolved: **2020-02-12**\ , date raised: 2020-01-16 
 * Version 0.1.280: resolved Issue 0311: Add generic node ODE2 (extension)
    - description:  generic node with n ODE2 coordinates
    - date resolved: **2020-02-12**\ , date raised: 2020-01-10 
 * Version 0.1.279: resolved Issue 0310: GenericRigidBodyJoint3D (extension)
    - description:  Add generic joint with local transformation matrix of marker1; enable fixed or free tranlatory motion (ux,uy,uz) and rotations (phix,phiy,phiz) in the joint, relative to marker1 coordinate system
    - date resolved: **2020-02-12**\ , date raised: 2020-01-10 
 * Version 0.1.278: resolved Issue 0304: time in constraints (extensions)
    - description:  consistently add time to constraint evaluation; add flag to mark time dependency of constraints (influence on velocity level and on initial conditions)
    - date resolved: **2020-02-12**\ , date raised: 2019-12-28 
 * Version 0.1.277: resolved Issue 0331: correct Rigid3DEP (check)
    - description:  correct gyroscopic terms and add test suite example
    - date resolved: **2020-02-03**\ , date raised: 2020-02-03 
 * Version 0.1.276: resolved Issue 0317: ANCF beam (extension)
    - description:  add rigid body access functions for ANCF elements
    - date resolved: **2020-02-02**\ , date raised: 2020-01-15 
 * Version 0.1.275: resolved Issue 0327: visualizationSettings (extension)
    - description:  convert system structures to dictionary and back; include type information
    - date resolved: **2020-01-31**\ , date raised: 2020-01-27 
 * Version 0.1.274: :textred:`resolved BUG 0326` : python single line 
    - description:  fix possible threading issues of python single command execute
    - date resolved: **2020-01-31**\ , date raised: 2020-01-27 
 * Version 0.1.273: resolved Issue 0146: pout, threads (extension)
    - description:  Check if pout is threadsafe  ==> introduce a mutex
    - date resolved: **2020-01-31**\ , date raised: 2019-05-25 
 * Version 0.1.272: resolved Issue 0322: node ltg (extension)
    - description:  add python access function to retrieve node local to global ODE2 and AE coordinates
    - **notes:** added reference to starting index in global coordinate vector
    - date resolved: **2020-01-25**\ , date raised: 2020-01-22 
 * Version 0.1.271: resolved Issue 0305: consistent shut down (extension)
    - description:  Perform consistent shut down of exudyn in case of PyError(): stop renderer before raising Py or SysError()
    - **notes:** but errors which occur directly in python cannot be catched
    - date resolved: **2020-01-25**\ , date raised: 2019-12-28 
 * Version 0.1.270: resolved Issue 0255: add docu for solvers (docu)
    - description:  add documentation about flags, formulas, error control, jacobians and about the steps in the solvers
    - date resolved: **2020-01-25**\ , date raised: 2019-08-28 
 * Version 0.1.269: resolved Issue 0227: IntegrityNodeCheck (extension)
    - description:  add integrity check that correct node type is supplied to object; use an additional function for RequiredNodeType(); use none for special elements or mixed node types --> check needs to be put into element specific checks
    - date resolved: **2020-01-25**\ , date raised: 2019-07-13 
 * Version 0.1.268: resolved Issue 0216: Sensors (new feature)
    - description:  Add sensor concept and add simple sensors for postprocessing; e.g. add simple sensors to cSystemData, which are written to prescribed sensor file or global sensor file; sensors have [marker, OutputVariableType, coordinate1, coordinate2=0]
    - date resolved: **2020-01-25**\ , date raised: 2019-06-29 
 * Version 0.1.267: :textred:`resolved BUG 0321` : RenderWindow focus 
    - description:  add options to resolve problem of focus/showing of render window in spyer/ipython
    - **notes:** solved: change sypder preferences, see FAQ
    - date resolved: **2020-01-22**\ , date raised: 2020-01-22 
 * Version 0.1.266: resolved Issue 0320: GetParameter (extension)
    - description:  in autogenerated object/node GetNodeParameter(..): change return value to py::in_(invalid index...) in order to reduce error message in spyder
    - date resolved: **2020-01-22**\ , date raised: 2020-01-16 
 * Version 0.1.265: resolved Issue 0316: Add try/catch (extension)
    - description:  add try/catch to main python interface functions; add in all autogenerated functions
    - date resolved: **2020-01-22**\ , date raised: 2020-01-10 
 * Version 0.1.264: resolved Issue 0318: sparse init acc (extension)
    - description:  compute initial accelerations with sparse solver
    - date resolved: **2020-01-21**\ , date raised: 2020-01-15 
 * Version 0.1.263: resolved Issue 0268: Export M, D, K, Cq (new feature)
    - description:  Make (linearized/constant) mass, damping, stiffness and constraint matrices available in python
    - date resolved: **2020-01-08**\ , date raised: 2019-10-10 
 * Version 0.1.262: resolved Issue 0269: Export residuals (new feature)
    - description:  Make residuals available in python
    - date resolved: **2020-01-06**\ , date raised: 2019-10-10 
 * Version 0.1.261: resolved Issue 0190: PybindMatrix (new feature)
    - description:  Use Pybind to bind matrices and vectors to numpy (simplify interface); copy all matrix/vector contents for now; only lateron, an option without copying would be nice; link to pybind example see https://github.com/pybind/pybind11/blob/master/tests/test_buffers.cpp as well as the pybind reference section about numpy
    - date resolved: **2020-01-06**\ , date raised: 2019-06-16 
 * Version 0.1.259: resolved Issue 0276: PySolver (new feature)
    - description:  Link data structures and functions of solver to python; use new MainSolver object for that reason
    - date resolved: **2020-01-05**\ , date raised: 2019-12-01 
 * Version 0.1.258: resolved Issue 0290: SetOutputToConsole (new feature)
    - description:  SetOutputToConsole(flag): add functionality to write activate/deactivate console output
    - date resolved: **2020-01-04**\ , date raised: 2019-12-13 
 * Version 0.1.257: resolved Issue 0289: SetOutputToFile (new feature)
    - description:  SetOutputToFile(flage, fileName): add functionality to write all console output to file
    - date resolved: **2020-01-04**\ , date raised: 2019-12-13 
 * Version 0.1.256: resolved Issue 0307: Add flowcharts (docu)
    - description:  add flowcharts for items, exu,SC,mbs,systemData, ... and solver
    - date resolved: **2019-12-31**\ , date raised: 2019-12-30 
 * Version 0.1.255: resolved Issue 0306: WaitForUserToContinue (extension)
    - description:  check if renderer is running; if not, the functions WaitForUserToContinue and WaitForRenderEngineStopFlag will do nothing or wait for console input
    - date resolved: **2019-12-31**\ , date raised: 2019-12-28 
 * Version 0.1.254: resolved Issue 0308: add show constraints mode (extension)
    - description:  add mode to show constraints with key "c" in openGL renderer
    - date resolved: **2019-12-30**\ , date raised: 2019-12-30 
 * Version 0.1.253: resolved Issue 0299: initial accelerations (extension)
    - description:  consistently compute initial accelerations for Newmark (and approx. for generalizedalpha
    - date resolved: **2019-12-27**\ , date raised: 2019-12-17 
 * Version 0.1.252: resolved Issue 0300: exudyn rules (extension)
    - description:  add name conventions, code style and rules, etc. to docu
    - date resolved: **2019-12-26**\ , date raised: 2019-12-25 
 * Version 0.1.251: resolved Issue 0256: cleanup solvers (clean)
    - description:  cleanup and unify initialization and computation iterations for solvers; add solver data structure accessible via pybind
    - date resolved: **2019-12-25**\ , date raised: 2019-08-28 
 * Version 0.1.250: resolved Issue 0254: CircleContact (extension)
    - description:  pre-check possible region of contact
    - date resolved: **2019-12-25**\ , date raised: 2019-08-28 
 * Version 0.1.249: resolved Issue 0244: RecordFrames (new feature)
    - description:  grap openGL snapshot with glReadPixels and store frame to image file
    - date resolved: **2019-12-25**\ , date raised: 2019-08-22 
 * Version 0.1.248: resolved Issue 0208: Check diff rel eps (check)
    - description:  Check why the differentiation parameter needs to be very small 1e-11 in case of larger system sizes
    - **notes:** might be solved with new jacobianAE and new solvers
    - date resolved: **2019-12-25**\ , date raised: 2019-06-28 
 * Version 0.1.247: resolved Issue 0298: solver step reduction (extension)
    - description:  add step reduction if Newton / discIt fails to solver
    - date resolved: **2019-12-18**\ , date raised: 2019-12-16 
 * Version 0.1.246: resolved Issue 0295: new static solver (extension)
    - description:  add new static solver and test it
    - date resolved: **2019-12-17**\ , date raised: 2019-12-16 
 * Version 0.1.245: resolved Issue 0245: generalized alpha Newton (check)
    - description:  revise newton method in time integration solver: modified and full newton are not consistent; time integration continues even in case of errors, etc.
    - date resolved: **2019-12-17**\ , date raised: 2019-08-23 
 * Version 0.1.244: resolved Issue 0296: ANCFCable2DALE (extension)
    - description:  additional flag for moving mass terms
    - date resolved: **2019-12-16**\ , date raised: 2019-12-16 
 * Version 0.1.243: resolved Issue 0294: improve impl. solver (extension)
    - description:  compute correct initial conditions, add new convergence criteria (residuum/nCoords and newtonDecrement.norm2/nCoords)
    - date resolved: **2019-12-15**\ , date raised: 2019-12-15 
 * Version 0.1.242: resolved Issue 0275: solver_data (new feature)
    - description:  restructure solver data structure: computation data, temporary data, system matrices, functions
    - date resolved: **2019-12-15**\ , date raised: 2019-12-01 
 * Version 0.1.241: resolved Issue 0293: disc.iteration (change)
    - description:  discontinuous iteration in generalized alpha: verbose wrongly used; should not have affected the solution
    - date resolved: **2019-12-14**\ , date raised: 2019-12-14 
 * Version 0.1.240: :textred:`resolved BUG 0292` : time steps 
    - description:  Wrong time is used for time integration when evaluating loads, etc.; need to change time from beginning of step to end of step for evaluation of RHS
    - date resolved: **2019-12-13**\ , date raised: 2019-12-13 
 * Version 0.1.239: :textred:`resolved BUG 0291` : generalized alpha 
    - description:  discontinuous iteration is is not initialized for algorithmic acceleration
    - date resolved: **2019-12-13**\ , date raised: 2019-12-13 
 * Version 0.1.238: resolved Issue 0187: GetAccessFunctionBody (change)
    - description:  Use ResizableMatrix in GetAccessFunctionBody and Resizable Vector in GetOutputVariableBody; add templated fill-in function to ResizableVector/Matrix to be able to copy data more easy from other types
    - date resolved: **2019-12-11 13:27**\ , date raised: 2019-06-13 
 * Version 0.1.237: resolved Issue 0099: split marker/load (new feature)
    - description:  split Marker and Load into .h and .cpp AND reduce dependencies on Nodes, body, etc.    
    - date resolved: **2019-12-11**\ , date raised: 2019-04-01 
 * Version 0.1.236: resolved Issue 0006: link (new feature)
    - description:  link to matrix/vector classes to Eigen OR MKL solvers    
    - date resolved: **2019-12-11**\ , date raised: 2019-04-01 
 * Version 0.1.235: resolved Issue 0286: check visualization (check)
    - description:  Check visualization of items, specifically of nodes and objects: graphicsdata should use the AddBodyGraphicsData(...) function for rigid body transformation
    - date resolved: **2019-12-10**\ , date raised: 2019-12-07 
 * Version 0.1.234: resolved Issue 0235: CqTLambda (extension)
    - description:  ComputeODE2RHS (=CqT\*lambda) based on single constraint object jacobians instead of global matrix multiply
    - date resolved: **2019-12-09**\ , date raised: 2019-08-19 
 * Version 0.1.233: resolved Issue 0158: SuperLU (new feature)
    - description:  Link SuperLU to linalg
    - date resolved: **2019-12-09**\ , date raised: 2019-05-28 
 * Version 0.1.232: resolved Issue 0157: EigenTriple (new feature)
    - description:  Add Eigentriple to mass matrix and jacobian computation
    - date resolved: **2019-12-09**\ , date raised: 2019-05-28 
 * Version 0.1.231: resolved Issue 0155: Joints (new feature)
    - description:  Add 2D spherical and prismatic joint
    - date resolved: **2019-12-09**\ , date raised: 2019-05-28 
 * Version 0.1.230: :textred:`resolved BUG 0285` : rigidbody2D 
    - description:  TriangleList does not work
    - date resolved: **2019-12-07**\ , date raised: 2019-12-07 
 * Version 0.1.229: resolved Issue 0282: relativ paths import (extension)
    - description:  use relative paths for import of WorkingRelease and TestModels
    - date resolved: **2019-12-07**\ , date raised: 2019-12-07 
 * Version 0.1.228: resolved Issue 0271: referenceCoordsRigid2D (check)
    - description:  check whether all reference coordinates in NodeRigid2D are correctly considered
    - date resolved: **2019-12-07**\ , date raised: 2019-10-19 
 * Version 0.1.227: resolved Issue 0280: add OpenGL settings (new feature)
    - description:  add settings for lights, material, normals and faces
    - date resolved: **2019-12-05**\ , date raised: 2019-12-05 
 * Version 0.1.226: resolved Issue 0279: add .py cube and cylinder (new feature)
    - description:  add functions in EXUDYN utilities for creation of 3D cube and cylinder with GLTriangles
    - date resolved: **2019-12-05**\ , date raised: 2019-12-05 
 * Version 0.1.225: resolved Issue 0265: Graphics Faces (new feature)
    - description:  Add triangular faces for 3D graphics
    - date resolved: **2019-12-05**\ , date raised: 2019-10-10 
 * Version 0.1.224: resolved Issue 0278: PyFunctions test (new feature)
    - description:  Test PyFunctions for SpringDamper, CoordinateSpringDamper, CoordinateConstraint and CoordinateLoad
    - date resolved: **2019-12-02**\ , date raised: 2019-12-01 
 * Version 0.1.223: resolved Issue 0277: PyFunctions (new feature)
    - description:  finalize PyFunctions for SpringDamper, CoordinateSpringDamper, CoordinateConstraint and CoordinateLoad
    - date resolved: **2019-12-02**\ , date raised: 2019-12-01 
 * Version 0.1.222: resolved Issue 0274: SparseLU (new feature)
    - description:  Add sparse matrices and sparse solver to static and dynamic solvers
    - date resolved: **2019-12-01**\ , date raised: 2019-11-14 
 * Version 0.1.221: resolved Issue 0273: constraint action (extension)
    - description:  Add constraint action forces \ :math:`Cq^T \cdot lambda`\  via a function in csystem; eliminates need for computation of separate matrix during computation
    - date resolved: **2019-12-01**\ , date raised: 2019-11-14 
 * Version 0.1.220: resolved Issue 0270: Intro to theDoc (new feature)
    - description:  write introductory sections and tutorials for theDoc
    - date resolved: **2019-12-01**\ , date raised: 2019-10-19 
 * Version 0.1.219: resolved Issue 0267: GitLab (new feature)
    - description:  Put EXUDYN on UIBK/GitLab
    - date resolved: **2019-12-01**\ , date raised: 2019-10-10 
 * Version 0.1.218: resolved Issue 0086: use (new feature)
    - description:  use pybind/functional.h (see pybind11.readthedocs.io) to: use python functions / classes in C++; also use classes to define functions (+parameters); user defined python objects!!!; see refToFunctionsClasses.py    
    - date resolved: **2019-12-01**\ , date raised: 2019-04-01 
 * Version 0.1.217: resolved Issue 0264: Rigid 3D (new feature)
    - description:  include rigid body, add graphics
    - date resolved: **2019-11-26**\ , date raised: 2019-10-10 
 * Version 0.1.216: :textred:`resolved BUG 0272` : maximumSolutionNorm 
    - description:  changed default value for maximum solution norm from 1e10 to 1e38, because it is the square norm (no square root!) and was easily exceeded with values u (or v) larger than 100000; now a scalar value can be up to 1e19, before the Newton method is stopped
    - **notes:** new limit is 1e38 for compatibility with float
    - date resolved: **2019-11-08**\ , date raised: 2019-11-08 
 * Version 0.1.215: :textred:`resolved BUG 0266` : correct unit tests 
    - description:  check 2 failures in unit tests
    - **notes:** small errors in tests 2 and 7 fixed by updating the reference values in the 1e-12 - 1e-9 range
    - date resolved: **2019-10-17**\ , date raised: 2019-10-10 
 * Version 0.1.214: resolved Issue 0263: ANCF damping (extension)
    - description:  finish bending damping
    - date resolved: **2019-10-17**\ , date raised: 2019-10-10 
 * Version 0.1.213: resolved Issue 0262: python generator (extension)
    - description:  add specific flag to autogenerated files, which detects if files have been changed or not
    - date resolved: **2019-10-10**\ , date raised: 2019-09-12 
 * Version 0.1.212: resolved Issue 0230: CircleContact2D (extension)
    - description:  add friction
    - date resolved: **2019-09-12**\ , date raised: 2019-07-23 
 * Version 0.1.211: resolved Issue 0257: color bar (new feature)
    - description:  add color bar for contour plots
    - date resolved: **2019-08-30**\ , date raised: 2019-08-30 
 * Version 0.1.210: resolved Issue 0253: outputvariables (extension)
    - description:  add new autogenerated output variables to all objects
    - date resolved: **2019-08-30**\ , date raised: 2019-08-28 
 * Version 0.1.209: resolved Issue 0231: PostProcessing (extension)
    - description:  Add contour plot and settings - must be same as in sensors
    - date resolved: **2019-08-30**\ , date raised: 2019-07-23 
 * Version 0.1.208: resolved Issue 0251: switching sliding joint (extension)
    - description:  add sliding joint example with switching activeConnector flag and relocation of sliding body
    - date resolved: **2019-08-28**\ , date raised: 2019-08-25 
 * Version 0.1.207: resolved Issue 0215: DiscontinuousIteration (extension)
    - description:  Add discontinuous iteration to dynamic solvers; also incorporate restarting of Newton by adding data variables
    - date resolved: **2019-08-28**\ , date raised: 2019-06-28 
 * Version 0.1.206: resolved Issue 0180: Jacobian (extension)
    - description:  Add jacobian computation for constraints based on markers and objects/nodes jacobians
    - date resolved: **2019-08-26**\ , date raised: 2019-06-11 
 * Version 0.1.205: :textred:`resolved BUG 0250` : test suite graphics fails 
    - description:  multiple use of graphics in testsuite leads to crashes; inconsistent cSystem and vSystem containers; check mbs.Reset() function
    - **notes:** added missing call to visualizationSystemData.Reset() in mbs.Reset()
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.204: resolved Issue 0249: user stopflag python (new feature)
    - description:  add read and write access to stopSimulation flag in python; this allows to interrupt python loops - e.g. for static loading or for animation
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.203: resolved Issue 0248: visualize solution (new feature)
    - description:  set visualization state and time with pybind interface and send renderer update flag
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.202: resolved Issue 0247: animateSolution (new feature)
    - description:  load solution file and consecutively set visualization state to loaded states; implement in python
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.201: resolved Issue 0240: TimeIntegrationCPU (new feature)
    - description:  add CPU statistics accoding to static solver (with common data structure) to time integration; this will be needed to check performance of sparse matrix versions
    - date resolved: **2019-08-25**\ , date raised: 2019-08-21 
 * Version 0.1.200: resolved Issue 0232: LoadSolution (new feature)
    - description:  Add functionality to load solution from coordinates solution file
    - date resolved: **2019-08-25**\ , date raised: 2019-07-23 
 * Version 0.1.199: :textred:`resolved BUG 0246` : ActivateConnector 
    - description:  Error in Newton / time integration with pure algebraic equations (CoordinateConstraint with activateConnector=False)
    - date resolved: **2019-08-23**\ , date raised: 2019-08-23 
 * Version 0.1.198: resolved Issue 0243: Vactive (extension)
    - description:  consistently integrate visualization active flag in UpdateGraphics for all items
    - date resolved: **2019-08-23**\ , date raised: 2019-08-22 
 * Version 0.1.197: resolved Issue 0242: MarkerDataComp (extension)
    - description:  Unify markerData computation in CSystem and in GetOutputVariableConnector at different places using a CSystem function
    - **notes:** not fully checked
    - date resolved: **2019-08-22**\ , date raised: 2019-08-22 
 * Version 0.1.196: resolved Issue 0233: Get/SetParameters (new feature)
    - description:  add functionality to set single parameters of items via pybind interface
    - date resolved: **2019-08-22**\ , date raised: 2019-07-23 
 * Version 0.1.195: resolved Issue 0239: PybindInterface (extension)
    - description:  Complete pybind interfaces for systemData, version, ...
    - date resolved: **2019-08-21**\ , date raised: 2019-08-21 
 * Version 0.1.194: resolved Issue 0238: JacobianODE2RHS_t (extension)
    - description:  add object-wise computation to NumericalJacobianODE2RHS_t in System.cpp
    - date resolved: **2019-08-21**\ , date raised: 2019-08-21 
 * Version 0.1.193: resolved Issue 0237: CoordinateSpringDamper (new feature)
    - description:  add new scalar (coordinate) spring damper for action on arbitrary objects; include dry friction as option
    - date resolved: **2019-08-21**\ , date raised: 2019-08-20 
 * Version 0.1.192: resolved Issue 0221: LoadCoordinate (new feature)
    - description:  Add a load which is attached to a single MarkerCoordinate
    - date resolved: **2019-08-21**\ , date raised: 2019-07-04 
 * Version 0.1.191: resolved Issue 0131: ResizableMatrix (check)
    - description:  Investigate casting between Matrix and ResizableMatrix ... check assignement operator (should it work?); implicit casting should be avoided because of memory allocation ==> use only explicit CopyFrom(Matrix)
    - date resolved: **2019-08-21**\ , date raised: 2019-05-17 
 * Version 0.1.190: resolved Issue 0236: IssueTracker (extension)
    - description:  add date, release and version (=number of resolved issues) into issues tracker
    - date resolved: **2019-08-20**\ , date raised: 2019-08-20 
 * Version 0.1.189: resolved Issue 0234: IntegrityMarkerCheck (check)
    - description:  Add check that correct markertype is used: e.g. BodyRigid for torque or CoordinateMarker, etc.
    - date resolved: **2019-08-19**\ , date raised: 2019-08-19 
 * Version 0.1.188: resolved Issue 0224: MarkerNodeCoordinate (extension)
    - description:  Add Integrity check for valid coordinate number
    - date resolved: **2019-08-19**\ , date raised: 2019-07-08 
 * Version 0.1.187: resolved Issue 0222: RequestedMarkerType (extension)
    - description:  Add RequestedMarkerType check to SystemIntegrity checks; specific marker type checks (e.g. SlidingJoint) are done in the element-specific checks ==> requestedMarkertype=None
    - date resolved: **2019-08-19**\ , date raised: 2019-07-06 
 * Version 0.1.186: resolved Issue 0092: add args (new feature)
    - description:  add args to pybind interface: m.def("add", &add, "A function which adds two numbers", py::arg("i") = 1, py::arg("j") = 2);    
    - date resolved: **2019-08-19**\ , date raised: 2019-04-01 
 * Version 0.1.185: resolved Issue 0091: Objects: (new feature)
    - description:  Objects: add to GetOutputVariableTypes(): GetAccessibleMarkerTypes() ==> returns all MarkerFlags, which can be used with body ...   
    - **notes:** already available via GetAccessFunctionTypes and GetOutputVariableTypes
    - date resolved: **2019-08-19**\ , date raised: 2019-04-01 
 * Version 0.1.184: resolved Issue 0085: Add (new feature)
    - description:  Add Test suite for Python side (test all interface functions)    
    - date resolved: **2019-08-19**\ , date raised: 2019-04-01 
 * Version 0.1.183: resolved Issue 0228: TimeIntNewton (check)
    - description:  Check why modified Newton does not converge as full Newton
    - date resolved: **2019-08-18**\ , date raised: 2019-07-15 
 * Version 0.1.182: resolved Issue 0223: Pendulum (check)
    - description:  Pendulum example with constraint does not work any more. check jacobian and algebraic equations
    - date resolved: **2019-08-18**\ , date raised: 2019-07-08 
 * Version 0.1.181: resolved Issue 0229: AxiallyMovingJoint (new feature)
    - description:  add prescribed sliding of point along ALECable2D
    - date resolved: **2019-07-24**\ , date raised: 2019-07-23 
 * Version 0.1.180: resolved Issue 0219: AxiallyMovingCable2D (new feature)
    - description:  Add ALE cable element with axially moving component; derive this class from ANCFCable2D; in this way parameters of ANCFCable2D are hidden, but all functions can be reused; direct access to parameters. must be removed in all ANCFCable2D implementation
    - date resolved: **2019-07-24**\ , date raised: 2019-07-04 
 * Version 0.1.179: resolved Issue 0226: SlidingJoint2D (extension)
    - description:  Add jacobian function
    - date resolved: **2019-07-23**\ , date raised: 2019-07-10 
 * Version 0.1.178: resolved Issue 0220: NodeGenericODE2 (new feature)
    - description:  Add generic node for ODE2 coordinates, used for AxiallyMovingCable2D
    - date resolved: **2019-07-23**\ , date raised: 2019-07-04 
 * Version 0.1.177: resolved Issue 0217: ContactFrictionCircle2D (new feature)
    - description:  Add a circular contact+friction; same as ObjectContactCircleCable2D
    - date resolved: **2019-07-23**\ , date raised: 2019-06-28 
 * Version 0.1.176: resolved Issue 0218: numDiff (check)
    - description:  Check whether the reference coordinates should be added to current coordinates for the size of the differentiation parameter
    - date resolved: **2019-07-12**\ , date raised: 2019-07-04 
 * Version 0.1.175: resolved Issue 0213: ObjectSlidingJoint2D (new feature)
    - description:  add sliding joint and according marker(s): one marker for set  of cable elements or a list of markers (needs to update ltg list)
    - date resolved: **2019-07-12**\ , date raised: 2019-06-28 
 * Version 0.1.174: resolved Issue 0210: LoadMassProportional (new feature)
    - description:  Add mass proportional vector loading: body marker + according load
    - date resolved: **2019-07-12**\ , date raised: 2019-06-28 
 * Version 0.1.173: resolved Issue 0185: User system function (new feature)
    - description:  Add user-defined system function to every end of step in time integration; use separate User-structure in settings
    - date resolved: **2019-07-10**\ , date raised: 2019-06-13 
 * Version 0.1.172: resolved Issue 0153: StaticSolver (new feature)
    - description:  Add nonlinear iteration for contact
    - date resolved: **2019-07-09**\ , date raised: 2019-05-28 
 * Version 0.1.171: resolved Issue 0214: Numerical Jacobian (extension)
    - description:  Add numerical jacobian for every object / constraint instead of global jacobian
    - date resolved: **2019-07-04**\ , date raised: 2019-06-28 
 * Version 0.1.170: resolved Issue 0212: ObjectContactCircleANCF2D (new feature)
    - description:  add a circular contact with centerpoint, radius and range of according angle of a circle segment
    - date resolved: **2019-07-04**\ , date raised: 2019-06-28 
 * Version 0.1.169: resolved Issue 0211: MarkerANCFCable2DShape (new feature)
    - description:  Add a marker to measure 2D/3D? ANCF shapes
    - date resolved: **2019-07-04**\ , date raised: 2019-06-28 
 * Version 0.1.168: resolved Issue 0193: Item names (extension)
    - description:  Add name tag to class interfaces for objects, nodes, ... to add item names; use empty string ("") to identify that default names shall be generated
    - date resolved: **2019-07-04**\ , date raised: 2019-06-19 
 * Version 0.1.167: resolved Issue 0166: ODE2CoordinatesNode (new feature)
    - description:  Replaced by new issue 220
    - date resolved: **2019-07-04**\ , date raised: 2019-06-03 
 * Version 0.1.166: resolved Issue 0207: ObjectContactCoordinate (new feature)
    - description:  add functionality for nonlinear iterations in objects and in static solver
    - date resolved: **2019-06-28**\ , date raised: 2019-06-28 
 * Version 0.1.165: resolved Issue 0206: ObjectContactCoordinate (new feature)
    - description:  Add a contact connector for single coordinates
    - date resolved: **2019-06-28**\ , date raised: 2019-06-28 
 * Version 0.1.164: resolved Issue 0183: MarkerNodeCoordinate (new feature)
    - description:  A marker which addresses a certain nodal coordinate for Issue #165 NodalConstraint
    - date resolved: **2019-06-28**\ , date raised: 2019-06-12 
 * Version 0.1.163: resolved Issue 0145: Jacobian (extension)
    - description:  Compute jacobian in timeintegration also with respect to velocities
    - date resolved: **2019-06-28**\ , date raised: 2019-05-23 
 * Version 0.1.162: resolved Issue 0093: SystemChecks (new feature)
    - description:  check system consistency/integrity before Assemble: objects->nodes, marker<->objects/nodes, marker<->constraints, marker<->loads
    - date resolved: **2019-06-28**\ , date raised: 2019-04-01 
 * Version 0.1.161: resolved Issue 0205: ObjectConnectorCartesianSpringDamper (new feature)
    - description:  Add a cartesian spring damper, which acts with certain parameters in x,y, and z-direction; can be used for 2D and 3D elements
    - date resolved: **2019-06-27**\ , date raised: 2019-06-27 
 * Version 0.1.160: resolved Issue 0204: NodeGenericData (new feature)
    - description:  Add new node with data coordinates; generic size
    - date resolved: **2019-06-26**\ , date raised: 2019-06-26 
 * Version 0.1.159: resolved Issue 0189: PythonClassNames (change)
    - description:  Add field pythonShortName to objectDefinition, to give the python object - e.g. ObjectConnectorDistance a better name - e.g. simply Distance, SpringDamper; use typedef, i.e. ConstrainDistance = ObjectConnectorDistance
    - date resolved: **2019-06-26**\ , date raised: 2019-06-14 
 * Version 0.1.158: resolved Issue 0188: useIndex2 (change)
    - description:  Rename useIndex2 in connectors / algebraic equations to velocityLevel
    - date resolved: **2019-06-26**\ , date raised: 2019-06-14 
 * Version 0.1.157: resolved Issue 0169: graphics (extension)
    - description:  Add graphics representation for ForceVector
    - date resolved: **2019-06-26**\ , date raised: 2019-06-05 
 * Version 0.1.156: resolved Issue 0165: ConstraintNodeCoord (new feature)
    - description:  ObjectConstraintNodeCoordinate: used to directly constrain two nodal coordinates
    - date resolved: **2019-06-26**\ , date raised: 2019-06-03 
 * Version 0.1.155: resolved Issue 0154: RigidBody (new feature)
    - description:  Add 2D rigid body
    - date resolved: **2019-06-26**\ , date raised: 2019-05-28 
 * Version 0.1.154: resolved Issue 0152: CData (new feature)
    - description:  Couple CData, initialState, etc. to pybind DIRECTLY as state structure --> enable multiple computations
    - date resolved: **2019-06-26**\ , date raised: 2019-05-28 
 * Version 0.1.153: resolved Issue 0140: enum Index (change)
    - description:  change from enum class to enum which is convertible to Index
    - date resolved: **2019-06-26**\ , date raised: 2019-05-20 
 * Version 0.1.152: resolved Issue 0133: AddMatrix (check)
    - description:  Test function add matrix and use it in mass matrix assembly
    - date resolved: **2019-06-26**\ , date raised: 2019-05-17 
 * Version 0.1.151: resolved Issue 0129: Solve:MarkerData (extension)
    - description:  Implement a GetMarkerData() function for markers
    - date resolved: **2019-06-26**\ , date raised: 2019-05-14 
 * Version 0.1.150: resolved Issue 0090: integrate (new feature)
    - description:  integrate rigid body (3D/2D) and according constraints    
    - date resolved: **2019-06-26**\ , date raised: 2019-04-01 
 * Version 0.1.149: resolved Issue 0203: LoadTorqueVector (new feature)
    - description:  Add new load TorqueVector
    - date resolved: **2019-06-25**\ , date raised: 2019-06-25 
 * Version 0.1.148: resolved Issue 0202: MarkerBodyRigid (new feature)
    - description:  Create a rigid body marker for application of torques
    - date resolved: **2019-06-24**\ , date raised: 2019-06-24 
 * Version 0.1.147: resolved Issue 0201: ObjectANCFCable2D (new feature)
    - description:  Add kappa0 and eps0
    - date resolved: **2019-06-23**\ , date raised: 2019-06-23 
 * Version 0.1.146: resolved Issue 0200: ObjectANCFCable2D (new feature)
    - description:  Create 2D ancf Bernoulli-Euler beam elements
    - date resolved: **2019-06-22**\ , date raised: 2019-06-28 
 * Version 0.1.145: resolved Issue 0199: NodePoint2DSlope1 (new feature)
    - description:  Create node for 2D ancf Bernoulli-Euler beam elements
    - date resolved: **2019-06-18**\ , date raised: 2019-06-18 
 * Version 0.1.144: resolved Issue 0198: 2D Nodes/Objects (new feature)
    - description:  Add NodePoint2D, NodeRigidBody2D, ObjectMassPoint2D, ObjectRigidBody2D, ObjectJointRevolute2D
    - date resolved: **2019-06-17**\ , date raised: 2019-06-17 
 * Version 0.1.143: resolved Issue 0197: ObjectConstraintCoordinate (new feature)
    - description:  constrain two coordinates; possibly add an offset (modifyable?)
    - date resolved: **2019-06-16**\ , date raised: 2019-06-16 
 * Version 0.1.142: resolved Issue 0196: MarkerNodeCoordinate (new feature)
    - description:  DE2/ODE1 coordinate at displacement or velocity level; extend MarkerType/OutputVariable interface
    - date resolved: **2019-06-15**\ , date raised: 2019-06-15 
 * Version 0.1.141: resolved Issue 0195: NodePointGround (new feature)
    - description:  Add node similar to nodepoint, but no action and zero coordinates
    - date resolved: **2019-06-14**\ , date raised: 2019-06-14 
 * Version 0.1.140: resolved Issue 0186: MarkerNodePoint (new feature)
    - description:  Add a Marker to Node point; extend according assemble and computation functions
    - date resolved: **2019-06-13**\ , date raised: 2019-06-13 
 * Version 0.1.139: resolved Issue 0184: GeneralizedAlpha (extension)
    - description:  Extend Newmark to generalized alpha
    - date resolved: **2019-06-13**\ , date raised: 2019-06-13 
 * Version 0.1.136: resolved Issue 0089: create (new feature)
    - description:  create .tex reference pages for objects    
    - date resolved: **2019-06-13**\ , date raised: 2019-04-01 
 * Version 0.1.135: resolved Issue 0182: Item Dicts (extension)
    - description:  Use letter V ahead all visualization parameters in interface; add VNodePoint python class for interface with visualization
    - date resolved: **2019-06-12**\ , date raised: 2019-06-12 
 * Version 0.1.134: resolved Issue 0181: SolutionFile (extension)
    - description:  Add user_defined comments to user file
    - date resolved: **2019-06-12**\ , date raised: 2019-06-12 
 * Version 0.1.133: resolved Issue 0177: safe copy (extension)
    - description:  Make consistent copy of current version
    - date resolved: **2019-06-12**\ , date raised: 2019-06-11 
 * Version 0.1.132: resolved Issue 0176: Python Interface (check)
    - description:  Check if it is better to add python classes for objects with according init function (can they be converted implicitly to dict and used in current AddNode(py::dict) function - or to use additional AddNode(...) function interfaces in MainSystem
    - date resolved: **2019-06-12**\ , date raised: 2019-06-09 
 * Version 0.1.131: resolved Issue 0175: Interface class (new feature)
    - description:  Add interface Python classes which return a dictionary for items; this enables autocompletion in editor ...
    - date resolved: **2019-06-12**\ , date raised: 2019-06-07 
 * Version 0.1.130: resolved Issue 0179: GLFW client (extension)
    - description:  Link client to MSC instead to MainSystem and link visualization to all cSystems; add "visualization.show" flag to visualize system
    - date resolved: **2019-06-11**\ , date raised: 2019-06-11 
 * Version 0.1.129: resolved Issue 0178: Start/stop renderer (change)
    - description:  Put function into MSC
    - date resolved: **2019-06-11**\ , date raised: 2019-06-11 
 * Version 0.1.128: resolved Issue 0170: RendererWait (extension)
    - description:  Add Wait function for renderer in pybind interface
    - date resolved: **2019-06-11**\ , date raised: 2019-06-06 
 * Version 0.1.127: resolved Issue 0168: Algebraic equations (extension)
    - description:  Extend implicit time integration for algebraic equations; add warning to explicit time integration
    - date resolved: **2019-06-11**\ , date raised: 2019-06-04 
 * Version 0.1.126: resolved Issue 0115: ComputationSystem (extension)
    - description:  Extend static computation for AE equations (DistanceConstraint)
    - date resolved: **2019-06-11**\ , date raised: 2019-05-13 
 * Version 0.1.125: resolved Issue 0156: Graphics (new feature)
    - description:  Add basic graphics elements (Line, Polygon, Circle) to bodies and ground visualization objects ==> for moving and static objects
    - date resolved: **2019-06-05**\ , date raised: 2019-05-28 
 * Version 0.1.124: resolved Issue 0167: Newton AE (new feature)
    - description:  Extend Newton for algebraic equations
    - date resolved: **2019-06-04**\ , date raised: 2019-06-03 
 * Version 0.1.123: resolved Issue 0163: FinishDistance (new feature)
    - description:  Finish algebraic equations for static and dynamic solver with distance
    - date resolved: **2019-06-04**\ , date raised: 2019-06-03 
 * Version 0.1.122: resolved Issue 0162: LagrangeMult (check)
    - description:  Check sign of Lagrange multipliers; related to CSystem::ComputeAERHS, last line
    - date resolved: **2019-06-04**\ , date raised: 2019-06-03 
 * Version 0.1.121: resolved Issue 0160: GraphicsUpdate (extension)
    - description:  Write GraphicsUpdate for nodes and SpringDamper
    - date resolved: **2019-06-03**\ , date raised: 2019-05-29 
 * Version 0.1.120: resolved Issue 0151: OpenGL options (new feature)
    - description:  Add opengl options to pybind: Visualization:General,Window(mouse move, zoom),OpenGL,System(Objects,Nodes,...),Text
    - date resolved: **2019-05-29**\ , date raised: 2019-05-28 
 * Version 0.1.119: resolved Issue 0149: StaticSolver (extension)
    - description:  Static solver: add time to state structure and file output
    - date resolved: **2019-05-29**\ , date raised: 2019-05-27 
 * Version 0.1.118: resolved Issue 0159: StopComputation (new feature)
    - description:  Add shortcut (CTRL Q) to OpenGL to quit simulation
    - date resolved: **2019-05-28**\ , date raised: 2019-05-28 
 * Version 0.1.117: resolved Issue 0150: OpenGLText (extension)
    - description:  Add simple text structure to OpenGL
    - date resolved: **2019-05-28**\ , date raised: 2019-05-28 
 * Version 0.1.116: resolved Issue 0148: CData (extension)
    - description:  extend static/dynamic solvers to work with CData State structures, including start of step, initial step and according time values
    - date resolved: **2019-05-28**\ , date raised: 2019-05-25 
 * Version 0.1.115: :textred:`resolved BUG 0147` : glfwRenderer 
    - description:  StopRenderer() leads to python session termination ==> try to debug
    - date resolved: **2019-05-28**\ , date raised: 2019-05-25 
 * Version 0.1.114: resolved Issue 0144: accelerations (extension)
    - description:  add accelerations to cData and option for export (in time integration
    - date resolved: **2019-05-28**\ , date raised: 2019-05-23 
 * Version 0.1.113: resolved Issue 0117: Visualization (extension)
    - description:  Link visualization Items to main Items
    - date resolved: **2019-05-28**\ , date raised: 2019-05-13 
 * Version 0.1.112: resolved Issue 0116: Visualization (extension)
    - description:  Add Visualization objects/nodes/...
    - date resolved: **2019-05-28**\ , date raised: 2019-05-13 
 * Version 0.1.111: resolved Issue 0143: Newmark (extension)
    - description:  Add coefficients to Implicit Trapezoidal rule interface
    - date resolved: **2019-05-27**\ , date raised: 2019-05-23 
 * Version 0.1.110: resolved Issue 0083: graphics (new feature)
    - description:  Link GLFW: Add library; add VisualizationSystem; link to MainSystem; add data structure (linked to pybind); Initialization; add FLAG to deactivate    
    - date resolved: **2019-05-27**\ , date raised: 2019-04-01 
 * Version 0.1.109: resolved Issue 0138: Discussion (new feature)
    - description:  Add new label DISCUSSION with blue color to issue tracker
    - date resolved: **2019-05-23**\ , date raised: 2019-05-19 
 * Version 0.1.108: :textred:`resolved BUG 0137` : StaticSolver 
    - description:  Debug static solver - crashes
    - date resolved: **2019-05-23**\ , date raised: 2019-05-19 
 * Version 0.1.107: resolved Issue 0134: SpringDamper (extension)
    - description:  Add velocities to Bodies / Markers and to SpringDamperActuator
    - date resolved: **2019-05-23**\ , date raised: 2019-05-19 
 * Version 0.1.106: resolved Issue 0113: ComputationSystem (new test)
    - description:  Test Loads
    - date resolved: **2019-05-23**\ , date raised: 2019-05-13 
 * Version 0.1.105: resolved Issue 0096: check Timeint (new feature)
    - description:  check TimeIntegration to work with new objects    
    - date resolved: **2019-05-23**\ , date raised: 2019-04-01 
 * Version 0.1.104: resolved Issue 0128: CMarker::GetPosJac (change)
    - description:  Remove return value Matrix and use Matrix& in arguments to avoid memory allocation
    - date resolved: **2019-05-19**\ , date raised: 2019-05-14 
 * Version 0.1.103: resolved Issue 0114: StaticSolver (new feature)
    - description:  Add Static (Nonlinear) Solver
    - date resolved: **2019-05-19**\ , date raised: 2019-05-13 
 * Version 0.1.102: resolved Issue 0112: ComputationSystem (extension)
    - description:  Add Jacobian evaluation for ODE2RHS
    - date resolved: **2019-05-19**\ , date raised: 2019-05-13 
 * Version 0.1.101: resolved Issue 0110: ComputationSystem (extension)
    - description:  Add/merge system level computation functions: ComputeMass, ComputeODE2RHS, ...
    - date resolved: **2019-05-19**\ , date raised: 2019-05-13 
 * Version 0.1.100: resolved Issue 0084: Setup (new feature)
    - description:  Setup static solver (use Matrix-solver OR eigen-SuperLU)    
    - date resolved: **2019-05-19**\ , date raised: 2019-04-01 
 * Version 0.1.99: resolved Issue 0082: Add (new feature)
    - description:  Add CObjectGround to objects    
    - date resolved: **2019-05-19**\ , date raised: 2019-04-01 
 * Version 0.1.98: resolved Issue 0132: MatrixInvert (new feature)
    - description:  Implement simple matrix inversion for simpler tetss without making use of Eigen library
    - date resolved: **2019-05-17**\ , date raised: 2019-05-17 
 * Version 0.1.97: resolved Issue 0127: constraint ODE2RHS (change)
    - description:  add new interface for ComputeODE2RHS and ComputeAlgebraicEquations; add Array<MarkerData>, which is prefilled during computation; MarkerData=Position,Velocity,PosJacobian,RotJacobian; 
    - date resolved: **2019-05-17**\ , date raised: 2019-05-14 
 * Version 0.1.96: resolved Issue 0122: ResizableMatrix (new feature)
    - description:  Implement resizable matrix for temporary data structures in solver/time integration
    - date resolved: **2019-05-17**\ , date raised: 2019-05-13 
 * Version 0.1.95: resolved Issue 0111: ComputationSystem (extension)
    - description:  Add temporary evaluation structures (Matrices/Vectors) for system evaluation functions (Contraints/Markers)
    - date resolved: **2019-05-17**\ , date raised: 2019-05-13 
 * Version 0.1.94: resolved Issue 0004: Finish (new feature)
    - description:  Finish functions for all matrix classes    
    - date resolved: **2019-05-17**\ , date raised: 2019-04-01 
 * Version 0.1.93: resolved Issue 0003: set (new feature)
    - description:  set up all matrix classes    
    - date resolved: **2019-05-17**\ , date raised: 2019-04-01 
 * Version 0.1.92: resolved Issue 0001: Finish (new feature)
    - description:  Finish functions for all vector classes    
    - date resolved: **2019-05-17**\ , date raised: 2019-04-01 
 * Version 0.1.91: resolved Issue 0130: Eigen (check)
    - description:  perform eigen tests in separate console project
    - date resolved: **2019-05-16**\ , date raised: 2019-05-16 
 * Version 0.1.90: resolved Issue 0109: Solver (new feature)
    - description:  count memory allocations (=new) during solving; eliminate memory allocation
    - date resolved: **2019-05-15**\ , date raised: 2019-05-12 
 * Version 0.1.89: resolved Issue 0107: SpringDamper (change)
    - description:  remove Matrix from CObjectConstraintSpringDamperActuator::ComputeODE2RHS(...)
    - date resolved: **2019-05-15**\ , date raised: 2019-05-12 
 * Version 0.1.88: resolved Issue 0120: release assert (check)
    - description:  check if release assert works correctly in release and debug mode (check with invalid vector access)
    - date resolved: **2019-05-13**\ , date raised: 2019-05-13 
 * Version 0.1.87: resolved Issue 0119: available types (new feature)
    - description:  Show available types in GetObject/Node/Marker/...Defaults() function, if args are used: GetNodeDefault()
    - date resolved: **2019-05-13**\ , date raised: 2019-05-13 
 * Version 0.1.86: resolved Issue 0108: Matrix/Vector (new feature)
    - description:  add global counter for memory allocations (=new)
    - date resolved: **2019-05-13**\ , date raised: 2019-05-12 
 * Version 0.1.85: resolved Issue 0106: VS2017_PYPLOT (compatibility)
    - description:  matplotlib.pyplot does not work in VS2017 - installation fails, while matplotlib is installed; upgrade of pip installer does not help
    - **notes:** restart of VS2017 solved problem
    - date resolved: **2019-05-12**\ , date raised: 2019-05-12 
 * Version 0.1.84: resolved Issue 0103: dict access (new feature)
    - description:  Lateron: Add dict access to functions (NodeAccessFunction({'name':node_name, 'function':'Stress','position':[0,1,2,0.5],'option':'Cauchy'})    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.83: resolved Issue 0102: dict access (new feature)
    - description:  Lateron: Add dict access to functions (NodeAccessFunction({'index':ind, 'function':'CurrentPosition'})    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.82: resolved Issue 0101: Add (new feature)
    - description:  Add representation and mainobject.help()") # add latex description ... for Reference manual    
    - **notes:** NOT NEEDED: help already included in pybind/Python
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.81: resolved Issue 0095: MainSystemContainer (new feature)
    - description:  move MainSystemContainer to own class    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.80: resolved Issue 0081: Debug: (new feature)
    - description:  Debug: CMarkerBodyPosition::GetPositionJacobian    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.79: resolved Issue 0105: issue tracker (new feature)
    - description:  add filename and line number to issue tracker
    - date resolved: **2019-05-11**\ , date raised: 2019-05-11 
 * Version 0.1.78: resolved Issue 0104: issue tracker (new feature)
    - description:  set up issue tracking system
    - date resolved: **2019-05-10**\ , date raised: 2019-05-10 
 * Version 0.1.77: resolved Issue 0136: Differentiate (new feature)
    - description:  Add class function for numerical differentiation of a member function of CSystem
    - **notes:** first tests did not work ==> added manually
    - date raised: 2019-05-19 
 * Version 0.1.76: resolved Issue 0087: Data dependency (new feature)
    - description:  For the moment: use CSystemData\* in all objects, ...    
    - date raised: 2019-02-01 
 * Version 0.1.75: resolved Issue 0080: Setup (new feature)
    - description:  Setup time integration    
    - date raised: 2019-02-01 
 * Version 0.1.74: resolved Issue 0079: Assemble (new feature)
    - description:  Assemble function renewed(split into nodes-section, etc.): assign node coordinates, initialize global coordinate vectors    
    - date raised: 2019-02-01 
 * Version 0.1.73: resolved Issue 0078: def_readwrite (new feature)
    - description:  def_readwrite used to access SystemStates initial, current, ...    
    - date raised: 2019-02-01 
 * Version 0.1.72: resolved Issue 0077: Add (new feature)
    - description:  Add pybinding to CData as well ==> but only for read access    
    - date raised: 2019-02-01 
 * Version 0.1.71: resolved Issue 0076: Python: (new feature)
    - description:  Python: add pybinding to SystemState .def_property and get/set functions; then access initial, current, etc. via separate functions, e.g.    
    - date raised: 2019-02-01 
 * Version 0.1.70: resolved Issue 0075: CData (new feature)
    - description:  CData --> class SystemState; change to individual SystemState for current, initial, reference, etc.    
    - date raised: 2019-02-01 
 * Version 0.1.69: resolved Issue 0074: change (new feature)
    - description:  change ...ObjectType(), NodeType() in object/node to Type(); unified with CMarker!    
    - date raised: 2019-02-01 
 * Version 0.1.68: resolved Issue 0073: change (new feature)
    - description:  change ...CObjectType to ObjectType (same as nodes, markers, outputvariabletype...); NO "C"    
    - date raised: 2019-02-01 
 * Version 0.1.67: resolved Issue 0072: Use (new feature)
    - description:  Use consistently "Get..." in function (C++: always; py interface: discuss)    
    - date raised: 2019-02-01 
 * Version 0.1.66: resolved Issue 0071: add (new feature)
    - description:  add error handling for AddMainNode/Object/... according to AddMainMarker    
    - date raised: 2019-02-01 
 * Version 0.1.65: resolved Issue 0070: add (new feature)
    - description:  add GetLoadVector() for LoadForceVector    
    - date raised: 2019-02-01 
 * Version 0.1.64: resolved Issue 0069: DONE: (new feature)
    - description:  DONE: size=1 or 3; add dimensionality of Load==>corresponds to dim of Marker; remove loadVector default from CLoad    
    - date raised: 2019-02-01 
 * Version 0.1.63: resolved Issue 0068: no: (new feature)
    - description:  no: marker can have variable dimension; add dimensionality of Marker (MarkerBodyPosition=3, MarkerBodyCoordinate=1, MarkerBodyRigid=6, MarkerNodePoint=3, etc.)    
    - date raised: 2019-02-01 
 * Version 0.1.62: resolved Issue 0067: finish (new feature)
    - description:  finish constraints, markers and loads    
    - date raised: 2019-02-01 
 * Version 0.1.61: resolved Issue 0066: integrate (new feature)
    - description:  integrate markers and loads    
    - date raised: 2019-02-01 
 * Version 0.1.60: resolved Issue 0065: constraint (new feature)
    - description:  constraint integration: .cpp file, object factory    
    - date raised: 2019-02-01 
 * Version 0.1.59: resolved Issue 0064: sys.InfoDetailed() (new feature)
    - description:  sys.InfoDetailed() ==> Dicts of all objects, nodes, markers, ...    
    - date raised: 2019-02-01 
 * Version 0.1.58: resolved Issue 0063: sys.InfoSummary(): (new feature)
    - description:  sys.InfoSummary(): __repr__ of sys ==> shows lists and CData;    
    - date raised: 2019-02-01 
 * Version 0.1.57: resolved Issue 0062: GetNumberOfNodes() (new feature)
    - description:  sys.GetNumberOfNodes(), etc.    
    - date raised: 2019-02-01 
 * Version 0.1.56: resolved Issue 0061: Test (new feature)
    - description:  Test CallObjectFunction(...) ==> error in sys.PyGetOutputVariable(0,ht.OutputVariableType.Position)     
    - date raised: 2019-02-01 
 * Version 0.1.55: resolved Issue 0060: change (new feature)
    - description:  change marker->body/load->body/... references from pointers to numbers    
    - date raised: 2019-02-01 
 * Version 0.1.54: resolved Issue 0059: GetOutputVariableBod (new feature)
    - description:  GetOutputVariableBody(variableType, localPosition, configuration, value)    
    - date raised: 2019-02-01 
 * Version 0.1.53: resolved Issue 0058: GetOutputVariable (new feature)
    - description:  GetOutputVariable(variableType, value)    
    - date raised: 2019-02-01 
 * Version 0.1.52: resolved Issue 0057: Vector: (new feature)
    - description:  Vector: add .cpp file and resolve SlimVector compiler conflict    
    - date raised: 2019-02-01 
 * Version 0.1.51: resolved Issue 0056: CallFunction(...) (new feature)
    - description:  MainObject::CallFunction(...)    
    - date raised: 2019-02-01 
 * Version 0.1.50: resolved Issue 0055: PyCallObjectFunction (new feature)
    - description:  MainSystem::PyCallObjectFunction(...) --> py::object    
    - date raised: 2019-02-01 
 * Version 0.1.49: resolved Issue 0054: add (new feature)
    - description:  add CObjectMassPoint.cpp and copy functions from COMassPoint.h    
    - date raised: 2019-02-01 
 * Version 0.1.48: resolved Issue 0053: resolve (new feature)
    - description:  resolve includes for Node, Body, MainSystem, ObjectFactory, TimeIntegrationSolver    
    - date raised: 2019-02-01 
 * Version 0.1.47: resolved Issue 0052: add (new feature)
    - description:  add addProtected/Public to python autoGenerator    
    - date raised: 2019-02-01 
 * Version 0.1.46: resolved Issue 0051: migrate (new feature)
    - description:  migrate from COBody to CObjectBody and MainObjectBody (COMMENT OUT main.cpp and similar implementations...)    
    - date raised: 2019-02-01 
 * Version 0.1.45: resolved Issue 0050: put (new feature)
    - description:  put CSystemData\* into CObject    
    - date raised: 2019-02-01 
 * Version 0.1.44: resolved Issue 0049: add (new feature)
    - description:  add pybindings to Marker-/Load-/OuputVariable-/...types;     
    - date raised: 2019-02-01 
 * Version 0.1.43: resolved Issue 0048: add (new feature)
    - description:  add MainMassPoint, ...    
    - date raised: 2019-02-01 
 * Version 0.1.42: resolved Issue 0047: add (new feature)
    - description:  add MainMarker, MainObject, ...    
    - date raised: 2019-02-01 
 * Version 0.1.41: resolved Issue 0046: finish (new feature)
    - description:  finish MainNodePoint and access classes of Node     
    - date raised: 2019-02-01 
 * Version 0.1.40: resolved Issue 0045: ModifyNode(node (new feature)
    - description:  ModifyNode(node, dict)    
    - date raised: 2019-02-01 
 * Version 0.1.39: resolved Issue 0044: AddNode('nodeType' (new feature)
    - description:  AddNode('nodeType':'...', ...), GetNode(number or nodeName), GetDefaultNode('nodeType')    
    - date raised: 2019-02-01 
 * Version 0.1.38: resolved Issue 0043: Objects (new feature)
    - description:  Objects available in Python vs. Dict-Interface    
    - date raised: 2019-02-01 
 * Version 0.1.37: resolved Issue 0042: create (new feature)
    - description:  create concept for python integration:system    
    - date raised: 2019-02-01 
 * Version 0.1.36: resolved Issue 0041: create (new feature)
    - description:  create concept for python integration:specific objects: RigidBody, ...    
    - date raised: 2019-02-01 
 * Version 0.1.35: resolved Issue 0040: create (new feature)
    - description:  create concept for python integration:core objects: Node, Marker, ...    
    - date raised: 2019-02-01 
 * Version 0.1.34: resolved Issue 0039: move (new feature)
    - description:  move masspoint and other objects to objects directory    
    - date raised: 2019-02-01 
 * Version 0.1.33: resolved Issue 0038: create (new feature)
    - description:  create Object factory: objects added to specified CSystem    
    - date raised: 2019-02-01 
 * Version 0.1.32: resolved Issue 0037: access (new feature)
    - description:  access item in CSystem (test)    
    - date raised: 2019-02-01 
 * Version 0.1.31: resolved Issue 0036: add (new feature)
    - description:  add python access to CSystem members (obtain a certain CSystem as a copy?)    
    - date raised: 2019-02-01 
 * Version 0.1.30: resolved Issue 0035: AddPyfunction (new feature)
    - description:  AddPyfunction add pyFunction (which is global in module.cpp) to create a CSystem in SC    
    - date raised: 2019-02-01 
 * Version 0.1.29: resolved Issue 0034: define (new feature)
    - description:  define python dict structure (folders) for object definition:     
    - date raised: 2019-02-01 
 * Version 0.1.28: resolved Issue 0033: special (new feature)
    - description:  special function headers in MainObject (compute specific things, postprocessing of stresses?)    
    - date raised: 2019-02-01 
 * Version 0.1.27: resolved Issue 0032: MainObject (new feature)
    - description:  MainObject links to functions of CObject, as far as needed    
    - date raised: 2019-02-01 
 * Version 0.1.26: resolved Issue 0031: ObjectFactory (new feature)
    - description:  ObjectFactory function    
    - date raised: 2019-02-01 
 * Version 0.1.25: resolved Issue 0030: GetDict (new feature)
    - description:  GetDict / SetDict for MainObject types; all MAINObjects have this functionality?; this means a very deep integration of pybind11!; but not needed in base class, because all pybindings in derived class; all communication at 'dict' level, ; MainSystem.SetObjectDict(objectID[Name], dict); MainSystem.SetNodeDict(nodeID[Name], dict), etc.    
    - date raised: 2019-02-01 
 * Version 0.1.24: resolved Issue 0029: dict (new feature)
    - description:  dict access to MainObject for every chosen parameter or function (put into pymodule-part)    
    - date raised: 2019-02-01 
 * Version 0.1.23: resolved Issue 0028: (B) (new feature)
    - description:  (B) bind-only functions of mother class (without definition in Main/CObject)    
    - date raised: 2019-02-01 
 * Version 0.1.22: resolved Issue 0027: (D) (new feature)
    - description:  (D) function can have declaration only    
    - date raised: 2019-02-01 
 * Version 0.1.21: resolved Issue 0026: chose (new feature)
    - description:  chose, which parameter goes into CObject or MainObject (no doubling!)    
    - date raised: 2019-02-01 
 * Version 0.1.20: resolved Issue 0025: creates (new feature)
    - description:  creates CObject, MainObject, etc. classes    
    - date raised: 2019-02-01 
 * Version 0.1.19: resolved Issue 0024: write (new feature)
    - description:  write new pythonAutoGenerateObjects.py file for Objects, Nodes, Markers, etc.    
    - date raised: 2019-02-01 
 * Version 0.1.18: resolved Issue 0023: Create (new feature)
    - description:  Create -py multibody system according to CreateMultibodySystem() ==> what is done there to add objects?    
    - date raised: 2019-02-01 
 * Version 0.1.17: resolved Issue 0022: MainObject (new feature)
    - description:  MainObject setter function, using Python dictionary    
    - date raised: 2019-02-01 
 * Version 0.1.16: resolved Issue 0021: MainObject (new feature)
    - description:  MainObject getter function, using Python dictionary    
    - date raised: 2019-02-01 
 * Version 0.1.15: resolved Issue 0020: MainObjectFactory (new feature)
    - description:  MainObjectFactory function, using Python dictionary    
    - date raised: 2019-02-01 
 * Version 0.1.14: resolved Issue 0019: MainObject: (new feature)
    - description:  MainObject: Pointer to CObject, Pointer to VObject, MainObjectParameters, special member variables, access functions, initialization    
    - date raised: 2019-02-01 
 * Version 0.1.13: resolved Issue 0018: CObject: (new feature)
    - description:  CObject: CObjectParameters (currently all public), special member variables, compute functions    
    - date raised: 2019-02-01 
 * Version 0.1.12: resolved Issue 0017: Access (new feature)
    - description:  Access to all objects (nodes, markers, etc) independent of class hierarchy: with a Dictionary() access    
    - date raised: 2019-02-01 
 * Version 0.1.11: resolved Issue 0016: change (new feature)
    - description:  change pythonAutoGenerator Setter/Getter functions to Pybind default: Set...(const Value& value) {...}    
    - date raised: 2019-02-01 
 * Version 0.1.10: resolved Issue 0015: extend (new feature)
    - description:  extend pythonAutoGenerationInterfaces for python+Main-interface classes (flag p?)    
    - date raised: 2019-02-01 
 * Version 0.1.9: resolved Issue 0014: rename (new feature)
    - description:  rename PyCNode, PyCSystem, etc. to MainNode, etc.    
    - date raised: 2019-02-01 
 * Version 0.1.8: resolved Issue 0013: add (new feature)
    - description:  add MainSystem to CSystem and add functionality for object factory    
    - date raised: 2019-02-01 
 * Version 0.1.7: resolved Issue 0012: check (new feature)
    - description:  check how derived classes work in pybind (need for trampoline?) ==> use Test class    
    - date raised: 2019-02-01 
 * Version 0.1.6: resolved Issue 0011: Add (new feature)
    - description:  Add global stream which always goes to Python    
    - date raised: 2019-02-01 
 * Version 0.1.5: resolved Issue 0010: create (new feature)
    - description:  create PySystemContainer:SystemContainer which adds the Python components to SystemContainer    
    - date raised: 2019-02-01 
 * Version 0.1.4: resolved Issue 0009: create (new feature)
    - description:  create SystemContainer as basis for all Comptational (MBS) System objects    
    - date raised: 2019-02-01 
 * Version 0.1.3: resolved Issue 0008: access (new feature)
    - description:  access to system and objects, but objects still live in C++ world    
    - date raised: 2019-02-01 
 * Version 0.1.2: resolved Issue 0007: decide: (new feature)
    - description:  decide: where do objects live?: objects + system live in C++ (otherwise parallelization inefficient)    
    - date raised: 2019-02-01 
 * Version 0.1.1: resolved Issue 0000: Test (new feature)
    - description:  Test efficiency of virtual function calls in current Vector/Matrix library ==> already done earlier ==> will be efficient for AVX implementation (small overhead accepted)    
    - date raised: 2019-02-01 

***********
Open issues
***********

 * **open issue 2131:** ObjectContact       
    - description:  add rolling resistance
    - date raised: 2025-07-05 

 * **open issue 2130:** ObjectContact       
    - description:  consider changing contact objects like SphereSphere, SphereTriangle, etc. to switch in PostNewtonStep based on force sign rather than gap sign; possibly use global switch
    - date raised: 2025-07-05 

 * **open issue 2120:** Screw graphics      
    - description:  add function to generate graphics for screw
    - date raised: 2025-07-02 

 * **open issue 2110:** solver              
    - description:  add flag for local frame implicit solver, computing jacobians in local frame and using specific step updates
    - date raised: 2025-06-24 

 * **open issue 2109:** DOPRI5              
    - description:  DOPRI5 automatic step size not working well with discontinuities (ContactSphereSphere, etc.)
    - date raised: 2025-06-22 

 * **open issue 2106:** Chain drive         
    - description:  add function to create chain gears as well as geometry from chain drive; calculate length similar to reeving system, but with two chains (and kinck) to compensate length
    - date raised: 2025-06-21 

 * **open issue 2097:** ContactSphereTriangle
    - description:  add frictionStiffness similar to bristle model in ANCF contact
    - date raised: 2025-06-15 

 * **open issue 2075:** GetDictionary       
    - description:  add read/write dict access for new structures SC.renderer and exudyn.config
    - date raised: 2025-06-02 

 * **open issue 2017:** ContactCurveCircles 
    - description:  implement polynomial enhancements
    - date raised: 2025-05-13 

 * **open issue 2016:** ContactCurveCircles 
    - description:  add output variables
    - date raised: 2025-05-11 

 * **open issue 2007:** solver timers       
    - description:  add special timer for Python user functions, as solver timer for python does not include user functions
    - date raised: 2025-05-10 

 * **open issue 1996:** MainSystemExtensions
    - description:  expose MainSystem as MainSystemBase into python; derive Python class MainSystem from MainSystemBase in MainSystemExtensions; this should increase visibility of Python code!
    - date raised: 2025-05-06 

 * **open issue 1989:** Body force sensor   
    - description:  add force sensor option for single-noded bodies and bodies which do not share nodes (mass points, rigid bodies, ffrf); for rigid bodies, obtains force and torque; for FFRF, it is generalized force; implement in a way that the contributions of loads are computed like in GeneralContact - based on a flag -, as soon as a BodySensor measures a force or torque
    - date raised: 2025-04-15 

 * **open issue 1988:** exceptions          
    - description:  change 'except:' to 'except Exception as e:' as this passes through the keyboard interrupts, which is better for parameter variation and other long-running codes
    - date raised: 2025-04-10 

 * **open issue 1977:** solver              
    - description:  check if solver can raise full solver error message in exception, in order to alleviate tracing during automated code evaluation in SolveStatic and SolveDynamic
    - date raised: 2025-03-30 

 * **open issue 1956:** items docu          
    - description:  add representative figure to each item
    - date raised: 2025-02-09 

 * **open issue 1954:** CreateLinearSpringDamper
    - description:  add test example
    - date raised: 2025-02-05 

 * **open issue 1953:** CreateLinearSpringDamper
    - description:  add create function to MainSystem
    - date raised: 2025-02-05 

 * **open issue 1947:** GeneralContact      
    - description:  check difference of friction force computation of SphereSphereContact (see notes in .cpp file) and GeneralContact
    - date raised: 2025-02-03 

 * **open issue 1941:** URDF                
    - description:  include parent names and create correct parent indices in URDF files by using link name lists; store list of base link names as well and do reordering
    - date raised: 2024-11-12 

 * **open issue 1932:** URDF import         
    - description:  GetURDFrobotData: add tool transformations accordingly (currently shown with no consecutive transformations)
    - date raised: 2024-11-10 

 * **open issue 1931:** URDF import         
    - description:  GetURDFrobotData: check for import of other scene information than mesh; check for import of collision
    - date raised: 2024-11-10 

 * **open issue 1924:** ContactCurveCircles 
    - description:  extend to frictional contact
    - date raised: 2024-11-04 

 * **open issue 1920:** ContactSphereSphere 
    - description:  add autodiff Jacobian
    - date raised: 2024-11-02 

 * **open issue 1912:** co-simulation       
    - description:  add simple model of two mass-spring-dampers to show simulator coupling of two implicit-explicit mbs, similar to issue 1905
    - date raised: 2024-10-26 

 * **open issue 1910:** total coordinates   
    - description:  consider a solver option which continuously provides total coordinates during iterations which could be used in user functions or globally and also be linked instead of copied; add an exception if the respective option is not switched on; by default, only add an empty Vector currentState.ODE2CoordsTotal
    - date raised: 2024-10-26 

 * **open issue 1907:** particles           
    - description:  add improved functions to create densly packed particles in box using simulation
    - date raised: 2024-10-19 

 * **open issue 1906:** particles           
    - description:  add improved functions to create more densly packed particles using advanced geometrical considerations for randomized radius spherical particles
    - date raised: 2024-10-19 

 * **open issue 1905:** simulator coupling  
    - description:  add test model for simulator coupling using mbs0 with joints and implicit integrator coupled to mbs1 with explicit integration
    - date raised: 2024-10-19 

 * **open issue 1904:** Reference/link data 
    - description:  evaluate further options to link to internal data, such as objects, nodes, etc.; possibly with GetObjectParameter(...) and similar functions possibly automated; this would highly speed up user functions as it reduces the number of exudyn function calls
    - date raised: 2024-10-19 

 * **open issue 1898:** particles           
    - description:  Add Python functionality for periodic walls, using pairs of walls at which particles are duplicated to the (-) side of a wall as soon as they transverse a periodic wall at the (+) side
    - date raised: 2024-10-16 

 * **open issue 1896:** GeneralContact      
    - description:  add option to completely freeze searchTree bins if all velocities are below a certain threshold; mark these contact objects as inactive by checking activity from time to time; reactivate bins only if item at boundary exceeds velocity limit at active boundary
    - date raised: 2024-10-16 

 * **open issue 1894:** GeneralContact      
    - description:  add n\*max(velocity)\*stepSize safety for bounding box, in order to update some objects less frequently (when max dist is reached)
    - date raised: 2024-10-16 

 * **open issue 1892:** Load/Save HDF5      
    - description:  Add Data structures like MatrixContainer, Vector3DList, etc.
    - date raised: 2024-10-13 

 * **open issue 1888:** mbs.GetDictionary   
    - description:  does not work for symbolic userfunctions
    - date raised: 2024-10-11 

 * **open issue 1864:** MatrixContainer     
    - description:  consider functionality to link to dense numpy matrix; possibly by using the allocatedSize in ResizableMatrix to indicate linking rather than allocation
    - date raised: 2024-10-02 

 * **open issue 1863:** MatrixContainer     
    - description:  consider functionality to link to sparse CSR scipy matrix rather than using the current CSR format - as a minimal solution do copying on C++ level; Problem: scipy CSR uses other format than Exudyns Triplets
    - date raised: 2024-10-02 

 * **open issue 1861:** exceptions and ValueError
    - description:  check all ValueError exceptions and change to appropriate error handling, like importerror, runtime error or value error!
    - date raised: 2024-09-19 

 * **open issue 1848:** GeneralContact      
    - description:  Test and improve implicit SPHERE-TRIG contact
    - date raised: 2024-06-02 

 * **open issue 1846:** ComputePostProcessingModes
    - description:  numberOfThreads> 1 not working: no modes computed
    - date raised: 2024-05-29 

 * **open issue 1845:** ComputePostProcessingModes
    - description:  numberOfThreads> 1 not working: conversion of vectorInput to np.array makes problems
    - date raised: 2024-05-29 

 * **open issue 1822:** inverse dynamics    
    - description:  consider an inverse dynamics solver for constrained systems; this requires decouple constraints from Lagrange multipliers; add separate LTG list for Lagrange multipliers and objectLTGAE; separate CObject::GetAlgebraicEquationsSize from LagrangeMultiplier size; check markerDataStructure.GetLagrangeMultipliers; see CSystemData::ComputeMarkerDataStructure
    - date raised: 2024-04-19 

 * **open issue 1821:** kinematics solver   
    - description:  consider functionality of a kinematic solver; this could be based on a quasi-static solver which utilizes the velocity constraint level and computes unknown velocities for a given configuration; first attempt could be based on finite differences for prescribed incremental motion and resulting incremental coordinates; only possible for systems with DOF=0; alternatively, we could compute velocity coordinates only from the constrained system, which however would only work if all coordinates are constrained
    - date raised: 2024-04-19 

 * **open issue 1813:** Marker positions    
    - description:  wrong representation of marker positions in AnimateModes for deformation scaling=0
    - date raised: 2024-04-08 

 * **open issue 1801:** joint constraints   
    - description:  add description of position jacobian for rigid bodies (in particular 3D rigid); add reference in description for MarkerBodyPosition
    - date raised: 2024-03-03 

 * **open issue 1777:** GraphicsData Sphere 
    - description:  add spheres to graphicsData interface; user AddSphere method; only use in case that full sphere is shown; add option to fall back to regular triangular representation
    - date raised: 2024-02-07 

 * **open issue 1776:** ComputeLinearizedSystem
    - description:  consider paper of Agundez, Vallejo, Freire, Mikkola in International Journal of Mechanical Sciences, Vol 268, 2024 for computation of linearized system and eigenmodes. Test case with bicycle
    - date raised: 2024-02-07 

 * **open issue 1769:** C++ user functions  
    - description:  Add cpp user functions fully to PythonUserFunctionBase capabilities
    - date raised: 2024-02-02 

 * **open issue 1768:** C++ user functions  
    - description:  Add pybind object as container for Cpp user functions, similar to autogenerated SetUserFunction
    - date raised: 2024-02-02 

 * **open issue 1767:** C++ user functions  
    - description:  add file for cpp user functions, registration mechanism like timers
    - date raised: 2024-02-02 

 * **open issue 1764:** GeneralContact      
    - description:  add pickle functionality
    - date raised: 2024-01-31 

 * **open issue 1751:** C++ user functions  
    - description:  check injection of user functions with special item-function method, and separate cpp file holding prototypes of those functions, which may be injected accordingly; use mechanism to record user functions in exudyn.functions.userCpp.SpringDamper
    - date raised: 2024-01-29 

 * **open issue 1740:** symbolic            
    - description:  check examples in Docu for consistency
    - date raised: 2023-12-19 

 * **open issue 1722:** Parameter type      
    - description:  add exudyn.Parameter for all parameters occuring in items, such as referencePosition, physicsMass, etc.; this helps to avoid strings in user functions to access parameter and may allow to use  more efficient case/switch in GetObjectParameter(...)
    - date raised: 2023-12-09 

 * **open issue 1720:** Mainsystem extensions
    - description:  change create2D functions into separate Create functions
    - date raised: 2023-12-08 

 * **open issue 1719:** generated examples  
    - description:  add set of generically generated examples, generated examples
    - date raised: 2023-12-08 

 * **open issue 1684:** ObjectIndex         
    - description:  consider functionality such as ComputeMassMatrix; ComputeODE2RHS, etc.; would require some default simulation settings (store in mainsystem?)
    - date raised: 2023-10-29 

 * **open issue 1683:** ItemIndices         
    - description:  consider direct access to outputvariables in node: nodeIndex.current.position; at least nodeIndex.GetOutput(variableType, configuration) would be valuable
    - date raised: 2023-10-29 

 * **open issue 1682:** ItemIndices         
    - description:  add previous CallFunction functionalities to NodeIndex, etc.; IsNodeGroup(group), IsNodeType(type), SizeODE2(), ..., 
    - date raised: 2023-10-29 

 * **open issue 1681:** ItemIndices         
    - description:  add option to add force/torque directly; add gravity to bodies
    - date raised: 2023-10-29 

 * **open issue 1677:** systemData          
    - description:  add GetDict(), Set(systemDict=[Dict]) functions which returns the whole dictionary for the system; containing list of nodes, objects, ...; each item is represented by its dictionary; could be used for set/get in future
    - date raised: 2023-10-29 

 * **open issue 1676:** ItemIndices         
    - description:  consider overriding __getattr__ and __setattr__ methods through pybind (or in Python with patching); this should allow to access data directly mapped via the dictionary
    - date raised: 2023-10-29 

 * **open issue 1675:** ItemIndices         
    - description:  consider adding MainSystem\* to indices; this would allow to directly operate on Nodes
    - date raised: 2023-10-29 

 * **open issue 1674:** license.ext         
    - description:  split into internal and external licenses
    - date raised: 2023-10-29 

 * **open issue 1668:** ANCFCable           
    - description:  add test example
    - date raised: 2023-10-16 

 * **open issue 1662:** ANCFThinPlate       
    - description:  add ANCF plate element based on 2 inplane slope vectors Slope12, based on Dufva/Shabana
    - date raised: 2023-10-15 

 * **open issue 1653:** ANCFBeam            
    - description:  reconsider name: ANCFBeamStructural, not to have too many cases; use this for 2/3 node, different number of slopes except for 1 slope, which is ANCFCable, the 3D version of ANCFCable2D
    - date raised: 2023-08-16 

 * **open issue 1650:** MacOS Rosetta       
    - description:  importing numpy gives Intel MKL Warning: Support of Intel Streaming SIMD Extensions 4.2 ... has been deprecated. Intel oneAPI Math Kernel Library 2025.0 will require AVX instructions
    - date raised: 2023-07-20 

 * **open issue 1631:** velocityOffset      
    - description:  add to CartesianSpringDamper, RigidBodySpringDamper
    - date raised: 2023-06-26 

 * **open issue 1614:** static members      
    - description:  LinearSolver GeneralMatrixEXUdense::FactorizeNew has static ResizableMatrix m, which should be turned into class members; add reset method to free memory at solver finalization
    - date raised: 2023-06-11 

 * **open issue 1565:** utilities InitializeFromRestartFile
    - description:  finalize C++ functionality and Python function
    - date raised: 2023-05-14 

 * **open issue 1550:** GeometricallyExactBeam
    - description:  add F_Lie\*Glocal_q term for Jacobian to improve convergence
    - date raised: 2023-05-02 

 * **open issue 1549:** KinematicTree       
    - description:  consider extension w.r.t. rigid body node at basis (Lie group node in explicit integration...); add baseNode (default=invalid), inertia could be added via a separate rigid body?
    - date raised: 2023-05-02 

 * **open issue 1548:** ODE1 loads          
    - description:  fully add Jacobian functionality for ODE1 loads and add test for ODE1 loads or recycle one
    - date raised: 2023-05-01 

 * **open issue 1529:** solver              
    - description:  solver functions GetSystemJacobian() and GetSystemMassMatrix() need to be extended with arg sparseTriplets=False; if True, it will return CSR sparse triplets, useful for large matrices, e.g. in eigenvalue computation in linearized system
    - date raised: 2023-04-26 

 * :textred:`open issue 1512:` return value policy 
    - description:  check return value policy of GeneralContact (as example for further decisions); see if reference in ALL access functions makes no problems if object is deleted on Python side
    - date raised: 2023-04-13 

 * :textorange:`open issue 1500:` ANCFBeam            
    - description:  check for advanced right-angle frame
    - date raised: 2023-04-08 

 * :textorange:`open issue 1499:` GeometricallyExactBeam
    - description:  check for advanced right-angle frame
    - date raised: 2023-04-08 

 * :textred:`open issue 1494:` GeometricallyExactBeam
    - description:  add reference configuration to residual and jacobian
    - date raised: 2023-04-06 

 * **open issue 1493:** StaticSolver        
    - description:  add exception in case that Lie group nodes are used with static solver, which cannot work
    - date raised: 2023-04-06 

 * **open issue 1485:** Spring-Damper connector description
    - description:  add general description for connectors based on spring-dampers (penalty)
    - date raised: 2023-04-01 

 * **open issue 1484:** Joint description   
    - description:  add general description for joint constraints
    - date raised: 2023-04-01 

 * :textred:`open issue 1450:` coordinatesSolution 
    - description:  add number of threads to solution files and more details on computer; check parameter variation and other files (e.g. numberOfThreads and final computation time)
    - date raised: 2023-02-25 

 * :textred:`open issue 1434:` solver              
    - description:  add CqT\*lambda terms to systemwide jacobian computation with flag
    - date raised: 2023-02-16 

 * **open issue 1424:** NumericalJacobianODE1RHS
    - description:  add case for duplicated ODE1 coordinates if connector has two markers for the same object, same as ltgODE2numDiff
    - date raised: 2023-02-08 

 * **open issue 1415:** CoordinateSpringDamperExt
    - description:  add flag for stepSizeRecommendation, where 0 is no recommendation, -1 is automatic and >0 is a directly recommended step size
    - date raised: 2023-01-22 

 * **open issue 1414:** InteractiveDialog   
    - description:  extend for explicit solver; needs internally different setup of solvers; use dynamicSolverType with default generalizedAlpha changable to Newmark/Index2 as well as explicit solvers
    - date raised: 2023-01-22 

 * **open issue 1395:** ComputeLinearizedSystem
    - description:  add test model
    - date raised: 2023-01-12 

 * **open issue 1337:** Newton              
    - description:  C++: check if SysError(s) in CSolverBase::Newton() can be changed into regular failure and step reduction for adaptiveStep
    - date raised: 2022-12-26 

 * **open issue 1292:** CSensorObject       
    - description:  store MarkerDataStructure locally in order to avoid memory allocations for evaluation of sensor data; also do this for MainSystem::PyGetObjectOutputVariable
    - date raised: 2022-11-13 

 * **open issue 1290:** ContactFrictionCircleCable2D
    - description:  shows tangential forces in case of all friction stiffness and damping values are zero; may be caused by specific projection
    - date raised: 2022-11-05 

 * **open issue 1273:** GeometricallyExactBeam
    - description:  check quadratic velocity terms
    - date raised: 2022-09-24 

 * **open issue 1247:** UserFunctions       
    - description:  check whether optimization of user functions with numba/JIT removes C->Python->C roundtrip overhead using pybind11 f.target approach from tests/test_callbacks.cpp; this would enable to retrieve the original c-function pointer
    - date raised: 2022-09-02 

 * **open issue 1241:** Register Items      
    - description:  consider mechanism to self-register items: objects, nodes, ...; same a swith TimerStructure registration; this allows to add user-defined objects without touching the overall code
    - date raised: 2022-08-24 

 * **open issue 1240:** Register unit tests 
    - description:  consider mechanism to self-register unit tests; same a swith TimerStructure registration
    - date raised: 2022-08-24 

 * :textred:`open issue 1203:` parallel / multithreaded
    - description:  C++: add multithreading for JacobianODE2 (analytic jacobians)
    - date raised: 2022-07-12 

 * :textred:`open issue 1202:` parallel / multithreaded
    - description:  C++: add multithreading for JacobianAE
    - date raised: 2022-07-12 

 * **open issue 1196:** ObjectFFRFreducedOrder sparse
    - description:  add sparse option for ObjectFFRFreducedOrder, filling in flexible-flexible mass terms into sparse mass matrix
    - date raised: 2022-07-11 

 * **open issue 1194:** generalizedAlpha scaling
    - description:  turn on/off scaling in interface to test symmetric solver speedup
    - date raised: 2022-07-11 

 * **open issue 1192:** ExplicitSolver      
    - description:  Newton / startOfStep: check if dataCoords should also be copied
    - date raised: 2022-07-10 

 * **open issue 1189:** ContactFrictionCircleCable2D
    - description:  add velocity offset to MarkerCable2DShape
    - date raised: 2022-07-08 

 * **open issue 1167:** user functions      
    - description:  check https://pybind11.readthedocs.io/en/stable/advanced/cast/functional.html regarding stateless functions and test performance with C++ functions for simple spring-damper
    - date raised: 2022-06-29 

 * **open issue 1142:** item functions checker
    - description:  add automatic tests for all necessary functions in items, such as nodes, objects, etc.; run tests similar to LEST test suite
    - date raised: 2022-06-12 

 * :textorange:`open issue 1140:` c++ user elements   
    - description:  add auto registration for C++ user items
    - date raised: 2022-06-12 

 * :textorange:`open issue 1139:` c++ user elements   
    - description:  add description regarding which functions are needed to add C++ user elements
    - date raised: 2022-06-12 

 * **open issue 1104:** GenericObject       
    - description:  add most general generic object containing ODE1, ODE2 and AE equations + unknowns; jacobianAE as user functions
    - date raised: 2022-05-23 

 * **open issue 1100:** GeometricallyExactBeam3D
    - description:  finalize implementation and fix jacobian computation
    - date raised: 2022-05-23 

 * **open issue 1087:** ANCFBeam3D          
    - description:  complete implementation of all functions (rigid marker, etc.)
    - date raised: 2022-05-16 

 * **open issue 1078:** BeamSectionGeometry 
    - description:  add BeamSectionGeometry to 2D beam elements
    - date raised: 2022-05-09 

 * :textred:`open issue 1061:` Reference and copy  
    - description:  add information to theDoc regarding copying and referencing objects, such as mbs, GetObject(...), etc.; add info into description C/R into generatePyBindings?
    - date raised: 2022-04-30 

 * **open issue 1032:** ObjectConnectorCoordinateVector
    - description:  cleanup, consider better UF and check implementation with theory (jac?)
    - date raised: 2022-04-04 

 * **open issue 1026:** Save as PNG         
    - description:  check MacOS implementation if glfw works with saving PNG files
    - date raised: 2022-04-02 

 * **open issue 1008:** Contact switching   
    - description:  Test improved contact integration method with resolution of switching and correction of integration of discontinuous forces; compare to switching point resolution
    - date raised: 2022-03-26 

 * :textred:`open issue 0990:` MarkerNodeCoordinate
    - description:  add option addReferenceCoordinates=False to include reference value in coordinate
    - date raised: 2022-03-16 

 * **open issue 0988:** ComputeConstraintJacobianDerivative
    - description:  make sparse version similar to numerically differentiated single objects in JacobianODE2RHS
    - date raised: 2022-03-15 

 * :textorange:`open issue 0987:` ALEANCFCable2D      
    - description:  add description - specifically regarding OutputVariables, special terms not available in ANCFCable2D (and add reference to ASME CND paper)
    - date raised: 2022-03-15 

 * **open issue 0984:** OutputVariableConnector
    - description:  check all penalty-based connectors if OutputVariable for forces is only computed if activeConnector=True
    - date raised: 2022-03-14 

 * **open issue 0973:** ContactFrictionCircleCable2D
    - description:  adapt old tests and create new beltdrive test
    - date raised: 2022-03-09 

 * **open issue 0971:** Renderer            
    - description:  add mechanisms to catch exceptions inside renderer thread; try detaching renderer thread
    - date raised: 2022-03-06 

 * **open issue 0957:** JointRevolute2D     
    - description:  add OutputVariables in C++ and in DOCU; check other objects with missing OutputVariables
    - date raised: 2022-02-28 

 * **open issue 0948:** update mecanumWheelRollingDiscTest
    - description:  update w.r.t. Trajectory class and TorsionalSpringDamper
    - date raised: 2022-02-21 

 * **open issue 0937:** GeneralContact      
    - description:  add option to draw contact forces
    - date raised: 2022-02-10 

 * **open issue 0927:** GeneralContact ANCF 
    - description:  test if 3 maxTangentialVelocities in 3-point Lobatto integration lead to better Newton performance
    - date raised: 2022-02-04 

 * **open issue 0926:** RollingDiscPenalty  
    - description:  rollingFrictionViscous only works for rolls with axis parallel to z-Plane; add MISSING formulas to Docu and adapt formulation
    - date raised: 2022-02-03 

 * **open issue 0917:** mbs.ComputeObjectLHSJacobian
    - description:  add computation functions for object; using system function; needing option for analytic/numeric computation
    - date raised: 2022-02-02 

 * **open issue 0916:** mbs.ComputeObjectAccessFunction
    - description:  add computation functions for access functions, e.g., TranlationalVelocity_qt, AngVel_qt, ...
    - date raised: 2022-02-02 

 * **open issue 0915:** mbs.ComputeObject...
    - description:  add mbs.ComputeObjectMassMatrix(...)
    - date raised: 2022-02-02 

 * **open issue 0914:** mbs.ComputeNode...  
    - description:  add computation functions for nodes, e.g., position or rotation jacobian; coordinates are already available in GetNodeOutput(...)
    - date raised: 2022-02-02 

 * **open issue 0912:** GeneralContact      
    - description:  add second CCactiveSetError mode, which computes error for given active set ==> error for PostNewton computed (error in assumed conditions forces)
    - date raised: 2022-02-02 

 * **open issue 0911:** GeneralContact      
    - description:  implement contact laws
    - date raised: 2022-02-02 

 * **open issue 0910:** GeneralContact      
    - description:  implement recommended step size (with error bound and separate min stepsize to avoid too small steps
    - date raised: 2022-02-02 

 * **open issue 0909:** GeneralContact      
    - description:  add functionality to store/restore contact state for start of time step
    - date raised: 2022-02-02 

 * **open issue 0908:** MarkerData          
    - description:  add configuration to markerdata computation; allows configuration in sensors and startOfStep configuration in Contact
    - date raised: 2022-02-02 

 * **open issue 0907:** GeneralContact      
    - description:  add implicit Trig-Sphere contact
    - date raised: 2022-02-02 

 * **open issue 0904:** GeneralContact      
    - description:  PostNewton (sphere-sphere, ancf-circle): add if clause to switch off contact in case of negative contact force
    - date raised: 2022-01-31 

 * **open issue 0892:** AccessFunctionType  
    - description:  add AccessFunctionType::TranslationalVelocity_q and AccessFunctionType::AngularVelocity_q needed for analytical jacobians
    - date raised: 2022-01-26 

 * **open issue 0888:** add information on error handling
    - description:  explain System errors, Python errors and Warnings; explain exception handling and add example
    - date raised: 2022-01-25 

 * **open issue 0886:** Exceptions          
    - description:  test py::raise_from for Python-induced exceptions (in renderPythonInterface) or for SystemErrors; check whether execeptions are originating from C or Python, see pybind11 Exceptions
    - date raised: 2022-01-25 

 * **open issue 0873:** GenericJoint        
    - description:  improve computation of jacobian, using crossproduct
    - date raised: 2022-01-18 

 * **open issue 0871:** restart method      
    - description:  add option solutionSettings.writeRestartFile to restart from separate restart file; solutionSettings.restartFileName defines folder and fileName; solutionSettings.restartWritePeriod defines time in seconds, how often it is written; also writes backup file
    - date raised: 2022-01-18 

 * **open issue 0867:** GeneralContact      
    - description:  change deltaV terms in ANCFCable and TrigSphere contact to fit signs used in docu
    - date raised: 2022-01-17 

 * **open issue 0865:** multithreaded solver
    - description:  test TaskManager::SuspendWorkers() for non-parallel parts (e.g. linear solver); possibly measure time spent for these parts, which should be larger than 2 ms to make sense; add option parallel.stopThreadsInSerialSections=False
    - date raised: 2022-01-15 

 * **open issue 0862:** Vector alignment    
    - description:  add 32 or 64 byte memory + length alignment to allocation of vectors in order to be able to always use AVX and/or loop unrolling for copying or manipulating vectors; use _mm_malloc / _mm_free and separate flags to turn on/off memory and length alignment, memory alignment turned off in case that AVX is not available
    - date raised: 2022-01-13 

 * **open issue 0859:** GeneralContact      
    - description:  split searchtree into regions, proportional to number of threads (FinalizeContact); use 2 splits in x, 2 splits in y, etc. until uneven number left; add class Box3Dindexed:Box3D, which adds index for region in searchtree; add access in GeneralContact for adding bounding box, creating the index; index is -1, if overlapping, filled in serially
    - date raised: 2022-01-13 

 * **open issue 0858:** ParallelFor         
    - description:  use ParallelFor with costs argument in GeneralContact and CSystem, to optimize usage
    - date raised: 2022-01-13 

 * **open issue 0855:** Assemble() docu     
    - description:  add information on general approach of adding objects and mbs.Assemble() procedure in Overview on Exudyn; add figure Add Nodes/Objects->Assemble->Solve
    - date raised: 2022-01-09 

 * **open issue 0853:** ComputeObjectJacobian...
    - description:  add mbs computation functions for jacobians, for ODE1, ODE2 and AE
    - date raised: 2022-01-08 

 * **open issue 0852:** ComputeObjectAlgebraicEquations
    - description:  add mbs computation functions for constraints
    - date raised: 2022-01-08 

 * **open issue 0845:** Jacobian documentation
    - description:  add documentation to object and connector jacobians
    - date raised: 2021-12-23 

 * **open issue 0844:** connector jacobian RigidBodySpringDamper
    - description:  add analytic jacobian for RigidBodySpringDamper connector
    - date raised: 2021-12-23 

 * **open issue 0841:** GeneralContact regularized friction
    - description:  extend regularized friction (Haff-Werner) to integrated form using either Cundall-Stack friction or breaking tangential springs
    - date raised: 2021-12-19 

 * **open issue 0836:** GeneralContact add cone
    - description:  for 3D cables add cone (including cylinder) to contact with 3D cables
    - date raised: 2021-12-19 

 * **open issue 0835:** GeneralContact add dissipative laws
    - description:  add common dissipative (coeff of restitution, etc. laws to contact
    - date raised: 2021-12-19 

 * **open issue 0834:** GeneralContact add contact laws
    - description:  add Hertzian contact laws to contacts, using separate enum for contact laws and additional parameter
    - date raised: 2021-12-19 

 * **open issue 0833:** GeneralContact rolling pivoting
    - description:  add rolling and pivoting (drilling, boring) friction to spheres and triangles
    - date raised: 2021-12-17 

 * **open issue 0829:** velocityOffset      
    - description:  add velocity offset to all spring dampers in order to replace many user functions with preStepUserFunctions
    - date raised: 2021-12-15 

 * **open issue 0792:** test Pardiso integration
    - description:  VS2017 settings with Intel Performance Libraries and test interface via Eigen
    - date raised: 2021-11-02 

 * **open issue 0783:** SetObjectParameter  
    - description:  extend functionality of Set[Item]Parameter functions to accept lists AND numpy arrays for vectors
    - date raised: 2021-10-29 

 * **open issue 0782:** MatrixContainer     
    - description:  add SetWithNGsolveSparseMatrix; add an interface to directly convert from NGsolve matrix, also converting coordinate storage xxyyzz
    - date raised: 2021-10-27 

 * **open issue 0777:** GenericODE2         
    - description:  add tests for dense and sparse mass and jacobian matrices, together with sparse/dense solvers
    - date raised: 2021-10-09 

 * **open issue 0753:** autodiff for Connectors
    - description:  add autodiff for connectors using spezial sizes like 6 for 2 position nodes, 14 for 2 rigid bodies and 40 for most objects (ObjectFFRFreducedOrder) + 100? as extreme case, falling back to numerical diff for any larger case
    - date raised: 2021-09-21 

 * **open issue 0737:** ContactCoordinate   
    - description:  check if is very close to switching, perform switching for end of step and set error very small; if immediate swichting after beginning of step, do not set stepRecommendation to avoid step reduction; repeat step; time integration: if recommended step is set, reduction is performed in first iteration, otherwise iterate
    - date raised: 2021-08-13 

 * **open issue 0736:** include GeomExactBeam3D
    - description:  as provided by Jan Tomec
    - date raised: 2021-08-12 

 * **open issue 0728:** optimize CollectCurrentNodeMarkerData
    - description:  optimize function for CNodeRigidBodyRotVecLG
    - date raised: 2021-07-31 

 * **open issue 0710:** CollectCurrentNodeData
    - description:  implement CollectCurrentNodeData for NodeRigidBody2D, optimize CollectCurrentNodeData for all rigid body nodes
    - date raised: 2021-07-08 

 * **open issue 0707:** LinkedDataVector,ResizableVector
    - description:  consider removing ResizableVector(integrate into Vector), implement LinkedDataVector as templated spezialization of Vector, no virtual calls in Vector
    - date raised: 2021-07-06 

 * **open issue 0698:** GetAvailableJacobians
    - description:  unify constraint.GetAvailableJacobians() with jacobian computations in joints, in order to avoid large overheads for jacobian assembly
    - date raised: 2021-07-01 

 * **open issue 0692:** CSystem             
    - description:  check JacobianAE: jacobianGM.AddSubmatrixTransposed(temp.localJacobianAE_ODE2_t ... if _t is correctly used
    - date raised: 2021-06-28 

 * **open issue 0648:** solver tutorial     
    - description:  create video with frequent solver errors and FAQ
    - date raised: 2021-05-01 

 * **open issue 0643:** ObjectFFRFreducedOrder / CMS
    - description:  create tutorial with two bodies (crank, connecting rod, rigid piston)
    - date raised: 2021-04-30 

 * **open issue 0627:** ObjectContactFrictionCircleCable2D
    - description:  add description, connector equations and figure
    - date raised: 2021-04-21 

 * **open issue 0626:** GenericJoint        
    - description:  add more description on constraint configurations, coordinate transformations and figures for GenericJoint
    - date raised: 2021-04-21 

 * **open issue 0625:** ObjectRigidBody     
    - description:  revise equations of motion and add figure for COM and local coordinates
    - date raised: 2021-04-15 

 * **open issue 0617:** python userFunctions
    - description:  consider adding an additional userFunctionVariable [List? or Dict?], which contains indices or further parameters needed in the userFunction
    - date raised: 2021-03-23 

 * **open issue 0614:** sensor dependencies 
    - description:  add functionality to compute sensor-dependencies (for LTG computation), used in controller connectors? alternatively add dependentNodes to existing connectors
    - date raised: 2021-03-21 

 * **open issue 0613:** MarkerObjectODE2Coordinates
    - description:  add simple test into TestModels
    - date raised: 2021-03-21 

 * **open issue 0608:** recommendedStepSize 
    - description:  add recommendedStepSize to ContactCoordinate element
    - date raised: 2021-03-20 

 * **open issue 0599:** ObjectFFRFreducedOrder
    - description:  compare Tait-Bryan and RotationVector cases with Euler parameters
    - date raised: 2021-03-18 

 * **open issue 0591:** ObjectFFRF          
    - description:  Check forceVector and gravity forces in comparison to paper
    - date raised: 2021-02-21 

 * **open issue 0574:** initialAccelerations
    - description:  add dg/dq\*(dot q) term for initial accelerations in velocity level constraints; check with rolling coin
    - date raised: 2021-02-07 

 * **open issue 0565:** Drift inspector     
    - description:  add functionality to check whether drift gets too large
    - date raised: 2021-01-29 

 * **open issue 0561:** Gen alpha2          
    - description:  setup new generalized alpha integrator with GGL
    - date raised: 2021-01-26 

 * **open issue 0559:** Lie group tests2    
    - description:  add Lie group integrator flybar governor
    - date raised: 2021-01-26 

 * **open issue 0517:** item number textures
    - description:  add special textures for item numbers
    - date raised: 2020-12-22 

 * **open issue 0452:** AVX objects         
    - description:  test AVX objects
    - date raised: 2020-09-16 

 * **open issue 0451:** AVX integration     
    - description:  test AVX in vector.cpp and dense solver
    - date raised: 2020-09-16 

 * **open issue 0444:** docstrings          
    - description:  change function comments in .py files to PEP standardized docstrings
    - date raised: 2020-09-06 

 * **open issue 0436:** virtual functions   
    - description:  make virtual functions consistent for some system classes like MainSystem, etc. which have no derived classes
    - date raised: 2020-07-21 

 * **open issue 0410:** drawing information 
    - description:  add consistent drawing information in show field of every item
    - date raised: 2020-05-24 

 * **open issue 0401:** add sensor miniexamples
    - description:  .
    - date raised: 2020-05-21 

 * **open issue 0390:** SlimVector          
    - description:  check if erasing all <rule of 5> methods in SlimVector work and speed up code performance
    - date raised: 2020-05-16 

 * **open issue 0380:** mass matrix update  
    - description:  mass matrix is not updated in Generalized Alpha solver in CSolverImplicitSecondOrderTimeInt::ComputeNewtonJacobian - may be critical for 3d rigid bodies
    - date raised: 2020-05-06 

 * **open issue 0354:** autodiff            
    - description:  add consistent object (not connector) differentiation either manually or with autodiff
    - date raised: 2020-03-05 

 * **open issue 0347:** initialCoordinates_t
    - description:  instead of initialVelocities
    - date raised: 2020-02-24 

 * **open issue 0332:** getobject/nodeparameter
    - description:  extend getobjectparameter/node/.. with default function from MainObject / MainNode/ ... which returns basic information, e.g., NodeType 
    - date raised: 2020-02-04 

 * **open issue 0323:** invalid index test  
    - description:  add test which checks that invalidIndex in python (-1) converts to invalidIndex in exudyn (use simple object with node number and check node number after setting object
    - date raised: 2020-01-24 

 * **open issue 0314:** Add user marker     
    - description:  add user marker
    - date raised: 2020-01-10 

 * **open issue 0303:** constraints derivatives
    - description:  add consistent flag, if constraints have velocity coordinate dependence or if they explicitly depend on time (needs additional derivatives for consistent initial accelerations)!
    - date raised: 2019-12-26 

 * **open issue 0260:** static computation  
    - description:  add consistent flag to markerdata computation, ODE2RHS computation, etc. for static computation, which does not compute information on velocities then.
    - date raised: 2019-09-10 

 * **open issue 0258:** contour plot        
    - description:  extend mass points and rigid bodies for contour plotting
    - date raised: 2019-08-30 

 * **open issue 0252:** PostNewtonStep      
    - description:  post newton step object functions shall be called from solver including a ResizableVector& dataVariables to be changed; post newton function shall not use direct write access to nodal data coordinates
    - date raised: 2019-08-27 

 * **open issue 0241:** PostNewtonStep      
    - description:  Check why markerData is computed with computeJacobian=true in CSystem::PostNewtonStep; is jacobian information really needed?
    - date raised: 2019-08-22 

 * **open issue 0209:** contact iteration   
    - description:  make simple example for contact to check changing jacobian matrices from ContactCoordinate
    - date raised: 2019-06-28 

 * **open issue 0194:** Jacobians           
    - description:  Make unique member function names for rotation/orientation jacobians in nodes and bodies
    - date raised: 2019-06-25 

 * **open issue 0174:** Iterator begin      
    - description:  Check if const consistency is realizable
    - date raised: 2019-06-07 

 * **open issue 0173:** LinkedDataVector    
    - description:  Check that the Vector is declared as const LinkedDataVectors if it is returned by a function; otherwise unforeseeable problems might occur
    - date raised: 2019-06-07 

 * **open issue 0172:** Matrix Init         
    - description:  Consider to put this funciton into constructor and call constructor instead of init
    - date raised: 2019-06-07 

 * **open issue 0171:** Matrix delete       
    - description:  Consider not to set data=NULL in order to detect memory which is deleted twice
    - date raised: 2019-06-07 

 * **open issue 0161:** SlimArray           
    - description:  Add template specializations to SlimArray<T,2..4> similar to SlimVector to speed up initialization of short vectors
    - date raised: 2019-06-02 

 * **open issue 0142:** Vector performance  
    - description:  check Vector operator[], and ConstVector performance regarding inlining
    - date raised: 2019-05-21 

 * **open issue 0126:** Linalg tests        
    - description:  Add unit tests for new ConstSizeMatrix and ResizableMatrix
    - date raised: 2019-05-13 

 * **open issue 0124:** ConstSizeVector     
    - description:  check if begin/end() overriding of Vector:: function is needed?
    - date raised: 2019-05-13 

 * **open issue 0123:** Linalg Override     
    - description:  Add override statement to all derived classes in linalg for safety
    - date raised: 2019-05-13 

 * **open issue 0121:** allocation failure  
    - description:  assert that every allocation in Matrix, Vector, ResizableArray, ... is performed with try/catch - compare Matrix::AllocateMemory(...)
    - date raised: 2019-05-13 

 * :textblue:`open issue 0098:` Destructors/Cleanup 
    - description:  add destructors/cleanup to MainSystemData and all other system functions (check new commands)    
    - date raised: 2019-04-01 

 * :textblue:`open issue 0088:` Data dependency     
    - description:  use dependencies for every computational member function; node: 		NodeData: LinkedDataVector displacement, velocity, acceleration;; object(singlenoded): 	ObjectData: LinkedDataVector displacement, velocity, acceleration;; object(multinoded): 	ObjectData: ResizableArray<LinkedDataVector> displacements, velocities, accelerations;; constraint(Lagr.):	FunctionResults1, FunctionResults2 (e.g. RotMatrix1, Position1, ...); marker:		only transforms data (load/constraint); load:			only provides load information    
    - date raised: 2019-04-01 

 * :textblue:`open issue 0005:` finish              
    - description:  finish tests for all matrix classes    
    - date raised: 2019-04-01 

 * :textblue:`open issue 0002:` finish              
    - description:  finish tests for all vector classes    
    - date raised: 2019-04-01 

**********
Known bugs
**********

 * :textred:`open BUG 2127:` ContactSphereTorus  
    - description:  check torques on both bodies, as there seems to be momentum conservation issues in ball bearings
    - date raised: 2025-07-03 

 * :textred:`open BUG 1889:` symbolic            
    - description:  GetLoad and similar functions do not work with symbolic user functions and raise TypeError: Object of type 'exudyn.exudynCPP.symbolic.UserFunction' is not an instance of 'function'; see also issue with mbs.GetDictionary()
    - date raised: 2024-10-11 

 * :textred:`open BUG 1772:` item.GetDictionary  
    - description:  item.GetDictionary not working for new user function interface with symbolic user function
    - date raised: 2024-02-04 

 * :textred:`open BUG 1639:` SolveDynamic FFRF   
    - description:  repeated call to mbs.SolveDynamic gives divergence; attributed to FFRFreducedOrder model; workaround uses repeated build of model before calling solver again; may be related to FFRF or MarkerSuperElement-internal variables
    - date raised: 2023-07-10 

 * :textred:`open BUG 1085:` GeneralContact      
    - description:  generalContactFrictionTests.py gives considerably different results after t=0.05 seconds between Windows and linux compiled version; may be caused by some initialization problems (bugs...); needs further tests
    - date raised: 2022-05-11 

 * :textred:`open BUG 1048:` sse2neon.h          
    - description:  on Apple, sse2neon.h is missing (include from github) and compilation fails; check if this only happens on M1 and change include modes of sse2neon.h; add this file to python setup.py for other cases
    - date raised: 2022-04-25 

 * :textred:`open BUG 0830:` PostNewton          
    - description:  PostNewton missing in explicit solvers; add warning or add after single steps (but exclude in contact computation!)
    - date raised: 2021-12-15 

 * :textred:`open BUG 0738:` ObjectContactCoordinate
    - description:  modified Newton does not work, no Jacobian update computed when switching
    - date raised: 2021-08-13 

 * :textred:`open BUG 0448:` ObjectGenericODE2 bug
    - description:  ObjectGenericODE2 crashes without message when initialized with invalid node numbers
    - date raised: 2020-09-09 


