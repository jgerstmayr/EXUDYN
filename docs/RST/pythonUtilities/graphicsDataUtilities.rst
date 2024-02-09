
.. _sec-module-graphicsdatautilities:

Module: graphicsDataUtilities
=============================

Utility functions for visualization, which provides functions for basic shapes
like cube, cylinder, sphere, solid of revolution. Functions generate dictionaries
which contain line, text or triangle primitives for drawing in Exudyn using OpenGL.

- Author:    Johannes Gerstmayr 
- Date:      2020-07-26 (created) 
- | Notes:
  | Some useful colors are defined, using RGBA (Red, Green, Blue and Alpha = opacity) channels            in the range [0,1], e.g., red = [1,0,0,1].
  | Available colors are: color4red, color4green, color4blue, color4cyan, color4magenta, color4yellow, color4orange, color4pink, color4lawngreen, color4violet, color4springgreen, color4dodgerblue, color4grey, color4darkgrey, color4lightgrey, color4lightred, color4lightgreen, color4steelblue, color4brown, color4black, color4darkgrey2, color4lightgrey2, color4white
  | Additionally, a list of 16 colors 'color4list' is available, which is intended to be used, e.g., for creating n bodies with different colors


.. _sec-graphicsdatautilities-switchtripletorder:

Function: SwitchTripletOrder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SwitchTripletOrder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L79>`__\ (\ ``vector``\ )

- | \ *function description*\ :
  | helper function to switch order of three items in a list; mostly used for reverting normals in triangles
- | \ *input*\ :
  | 3D vector as list or as np.array
- | \ *output*\ :
  | interchanged 2nd and 3rd component of list



----


.. _sec-graphicsdatautilities-computetrianglenormal:

Function: ComputeTriangleNormal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeTriangleNormal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L89>`__\ (\ ``p0``\ , \ ``p1``\ , \ ``p2``\ )

- | \ *function description*\ :
  | compute normalized normal for 3 triangle points
- | \ *input*\ :
  | 3D vector as list or as np.array
- | \ *output*\ :
  | normal as np.array



----


.. _sec-graphicsdatautilities-computetrianglearea:

Function: ComputeTriangleArea
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeTriangleArea <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L102>`__\ (\ ``p0``\ , \ ``p1``\ , \ ``p2``\ )

- | \ *function description*\ :
  | compute area of triangle given by 3 points
- | \ *input*\ :
  | 3D vector as list or as np.array
- | \ *output*\ :
  | area as float



----


.. _sec-graphicsdatautilities-graphicsdata2pointsandtrigs:

Function: GraphicsData2PointsAndTrigs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsData2PointsAndTrigs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L109>`__\ (\ ``g``\ )

- | \ *function description*\ :
  | convert graphics data into list of points and list of triangle indices (triplets)
- | \ *input*\ :
  | g contains a GraphicsData with type TriangleList
- | \ *output*\ :
  | returns [points, triangles], with points as list of np.array with 3 floats per point and triangles as a list of np.array with 3 int per triangle (0-based indices to points)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex), \ `tippeTop.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/tippeTop.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdatafrompointsandtrigs:

Function: GraphicsDataFromPointsAndTrigs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataFromPointsAndTrigs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L132>`__\ (\ ``points``\ , \ ``triangles``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | convert triangles and points as returned from GraphicsData2TrigsAndPoints(...)
- | \ *input*\ :
  | \ ``points``\ : list of np.array with 3 floats per point
  | \ ``triangles``\ : list of np.array with 3 int per triangle (0-based indices to triangles)
  | \ ``color``\ : provided as list of 4 RGBA values or single list of (number of points)\*[4 RGBA values]
- | \ *output*\ :
  | returns GraphicsData with type TriangleList

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-refinemesh:

Function: RefineMesh
^^^^^^^^^^^^^^^^^^^^
`RefineMesh <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L158>`__\ (\ ``points``\ , \ ``triangles``\ )

- | \ *function description*\ :
  | refine triangle mesh; every triangle is subdivided into 4 triangles
- | \ *input*\ :
  | \ ``points``\ : list of np.array with 3 floats per point
  | \ ``triangles``\ : list of np.array with 3 int per triangle (0-based indices to triangles)
- | \ *output*\ :
  | returns [points2, triangles2] containing the refined mesh; if the original mesh is consistent, no points are duplicated; if the mesh is not consistent, some mesh points are duplicated!
- | \ *notes*\ :
  | becomes slow for meshes with more than 5000 points

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `tippeTop.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/tippeTop.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-shrinkmeshnormaltosurface:

Function: ShrinkMeshNormalToSurface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ShrinkMeshNormalToSurface <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L217>`__\ (\ ``points``\ , \ ``triangles``\ , \ ``distance``\ )

- | \ *function description*\ :
  | shrink mesh using triangle normals; every point is at least moved a distance 'distance' normal from boundary
- | \ *input*\ :
  | \ ``points``\ : list of np.array with 3 floats per point
  | \ ``triangles``\ : list of np.array with 3 int per triangle (0-based indices to triangles)
  | \ ``distance``\ : float value of minimum distance
- | \ *output*\ :
  | returns [points2, triangles2] containing the refined mesh; currently the points of the subdivided triangles are duplicated!
- | \ *notes*\ :
  | ONLY works for consistent meshes (no duplicated points!)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-movegraphicsdata:

Function: MoveGraphicsData
^^^^^^^^^^^^^^^^^^^^^^^^^^
`MoveGraphicsData <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L252>`__\ (\ ``g``\ , \ ``pOff``\ , \ ``Aoff``\ )

- | \ *function description*\ :
  | add rigid body transformation to GraphicsData, using position offset (global) pOff (list or np.array) and rotation Aoff (transforms local to global coordinates; list of lists or np.array); see Aoff how to scale coordinates!
- | \ *input*\ :
  | \ ``g``\ : graphicsData to be transformed
  | \ ``pOff``\ : 3D offset as list or numpy.array added to rotated points
  | \ ``Aoff``\ : 3D rotation matrix as list of lists or numpy.array with shape (3,3); if A is scaled by factor, e.g. using 0.001\*np.eye(3), you can also scale the coordinates!!!
- | \ *output*\ :
  | returns new graphcsData object to be used for drawing in objects
- | \ *notes*\ :
  | transformation corresponds to HomogeneousTransformation(Aoff, pOff), transforming original coordinates v into vNew = pOff + Aoff @ v

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Ex), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-mergegraphicsdatatrianglelist:

Function: MergeGraphicsDataTriangleList
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`MergeGraphicsDataTriangleList <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L311>`__\ (\ ``g1``\ , \ ``g2``\ )

- | \ *function description*\ :
  | merge 2 different graphics data with triangle lists
- | \ *input*\ :
  | graphicsData dictionaries g1 and g2 obtained from GraphicsData functions
- | \ *output*\ :
  | one graphicsData dictionary with single triangle lists and compatible points and normals, to be used in visualization of EXUDYN objects; edges are merged; edgeColor is taken from graphicsData g1

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdataline:

Function: GraphicsDataLine
^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataLine <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L366>`__\ (\ ``pList``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | generate graphics data for lines, given by list of points and color; transforms to GraphicsData dictionary
- | \ *input*\ :
  | \ ``pList``\ : list of 3D numpy arrays or lists (to achieve closed curve, set last point equal to first point)
  | \ ``color``\ : provided as list of 4 RGBA values
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects
- | \ *example*\ :

.. code-block:: python

  #create simple 3-point lines
  gLine=GraphicsDataLine([[0,0,0],[1,0,0],[2,0.5,0]], color=color4red)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex), \ `doublePendulum2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/doublePendulum2D.py>`_\  (Ex), \ `simple4linkPendulumBing.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/simple4linkPendulumBing.py>`_\  (Ex), \ `doublePendulum2DControl.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/doublePendulum2DControl.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdatacircle:

Function: GraphicsDataCircle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataCircle <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L382>`__\ (\ ``point = [0,0,0]``\ , \ ``radius = 1``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | generate graphics data for a single circle; currently the plane normal = [0,0,1], just allowing to draw planar circles -- this may be extended in future!
- | \ *input*\ :
  | \ ``point``\ : center point of circle
  | \ ``radius``\ : radius of circle
  | \ ``color``\ : provided as list of 4 RGBA values
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects
- | \ *notes*\ :
  | the tiling (number of segments to draw circle) can be adjusted by visualizationSettings.general.circleTiling

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-graphicsdatatext:

Function: GraphicsDataText
^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataText <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L393>`__\ (\ ``point = [0,0,0]``\ , \ ``text = ''``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | generate graphics data for a text drawn at a 3D position
- | \ *input*\ :
  | \ ``point``\ : position of text
  | \ ``text``\ : string representing text
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``**nodes``\ : text size can be adjusted with visualizationSettings.general.textSize, which affects the text size (=font size) globally
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-graphicsdatarectangle:

Function: GraphicsDataRectangle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataRectangle <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L400>`__\ (\ ``xMin``\ , \ ``yMin``\ , \ ``xMax``\ , \ ``yMax``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | generate graphics data for 2D rectangle
- | \ *input*\ :
  | minimal and maximal cartesian coordinates in (x/y) plane; color provided as list of 4 RGBA values
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Ex), \ `lavalRotor2Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lavalRotor2Dtest.py>`_\  (Ex), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesTest.py>`_\  (Ex), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TM), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TM), \ `ANCFslidingAndALEjointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFslidingAndALEjointTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdataorthocubelines:

Function: GraphicsDataOrthoCubeLines
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataOrthoCubeLines <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L411>`__\ (\ ``xMin``\ , \ ``yMin``\ , \ ``zMin``\ , \ ``xMax``\ , \ ``yMax``\ , \ ``zMax``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | generate graphics data for orthogonal block drawn with lines
- | \ *input*\ :
  | minimal and maximal cartesian coordinates for orthogonal cube; color provided as list of 4 RGBA values
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `rigid3Dexample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigid3Dexample.py>`_\  (Ex), \ `genericJointUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/genericJointUserFunctionTest.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `sphericalJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sphericalJointTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdataorthocube:

Function: GraphicsDataOrthoCube
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataOrthoCube <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L428>`__\ (\ ``xMin``\ , \ ``yMin``\ , \ ``zMin``\ , \ ``xMax``\ , \ ``yMax``\ , \ ``zMax``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``addNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color4black``\ , \ ``addFaces = True``\ )

- | \ *function description*\ :
  | generate graphics data for orthogonal 3D block with min and max dimensions
- | \ *input*\ :
  | \ ``x/y/z/Min/Max``\ : minimal and maximal cartesian coordinates for orthogonal cube
  | \ ``color``\ : list of 4 RGBA values
  | \ ``addNormals``\ : add face normals to triangle information
  | \ ``addEdges``\ : if True, edges are added in TriangleList of GraphicsData
  | \ ``edgeColor``\ : optional color for edges
  | \ ``addFaces``\ : if False, no faces are added (only edges)
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Ex), \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Ex), \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Ex), \ `performanceMultiThreadingNG.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/performanceMultiThreadingNG.py>`_\  (Ex), \ `rigidBodyIMUtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyIMUtest.py>`_\  (Ex), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `explicitLieGroupIntegratorPythonTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorPythonTest.py>`_\  (TM), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdataorthocubepoint:

Function: GraphicsDataOrthoCubePoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataOrthoCubePoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L444>`__\ (\ ``centerPoint = [0,0,0]``\ , \ ``size = [0.1,0.1,0.1]``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``addNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color4black``\ , \ ``addFaces = True``\ )

- | \ *function description*\ :
  | generate graphics data forfor orthogonal 3D block with center point and size
- | \ *input*\ :
  | \ ``centerPoint``\ : center of cube as 3D list or np.array
  | \ ``size``\ : size as 3D list or np.array
  | \ ``color``\ : list of 4 RGBA values
  | \ ``addNormals``\ : add face normals to triangle information
  | \ ``addEdges``\ : if True, edges are added in TriangleList of GraphicsData
  | \ ``edgeColor``\ : optional color for edges
  | \ ``addFaces``\ : if False, no faces are added (only edges)
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects; if addEdges=True, it returns a list of two dictionaries

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdatacube:

Function: GraphicsDataCube
^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataCube <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L471>`__\ (\ ``pList``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``faces = [1,1,1,1,1,1]``\ , \ ``addNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color4black``\ , \ ``addFaces = True``\ )

- | \ *function description*\ :
  | generate graphics data for general block with endpoints, according to given vertex definition
- | \ *input*\ :
  | \ ``pList``\ : is a list of points [[x0,y0,z0],[x1,y1,z1],...]
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``faces``\ : includes the list of six binary values (0/1), denoting active faces (value=1); set index to zero to hide face
  | \ ``addNormals``\ : if True, normals are added and there are separate points for every triangle
  | \ ``addEdges``\ : if True, edges are added in TriangleList of GraphicsData
  | \ ``edgeColor``\ : optional color for edges
  | \ ``addFaces``\ : if False, no faces are added (only edges)
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects



----


.. _sec-graphicsdatautilities-graphicsdatasphere:

Function: GraphicsDataSphere
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataSphere <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L557>`__\ (\ ``point = [0,0,0]``\ , \ ``radius = 0.1``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 8``\ , \ ``addEdges = False``\ , \ ``edgeColor = color4black``\ , \ ``addFaces = True``\ )

- | \ *function description*\ :
  | generate graphics data for a sphere with point p and radius
- | \ *input*\ :
  | \ ``point``\ : center of sphere (3D list or np.array)
  | \ ``radius``\ : positive value
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``nTiles``\ : used to determine resolution of sphere >=3; use larger values for finer resolution
  | \ ``addEdges``\ : True or number of edges along sphere shell (under development); for optimal drawing, nTiles shall be multiple of 4 or 8
  | \ ``edgeColor``\ : optional color for edges
  | \ ``addFaces``\ : if False, no faces are added (only edges)
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionTest.py>`_\  (Ex), \ `connectorGravityTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/connectorGravityTest.py>`_\  (TM), \ `contactCoordinateTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactCoordinateTest.py>`_\  (TM), \ `coordinateVectorConstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraint.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdatacylinder:

Function: GraphicsDataCylinder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataCylinder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L684>`__\ (\ ``pAxis = [0,0,0]``\ , \ ``vAxis = [0,0,1]``\ , \ ``radius = 0.1``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 16``\ , \ ``angleRange = [0,2*pi]``\ , \ ``lastFace = True``\ , \ ``cutPlain = True``\ , \ ``addEdges = False``\ , \ ``edgeColor = color4black``\ , \ ``addFaces = True``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | generate graphics data for a cylinder with given axis, radius and color; nTiles gives the number of tiles (minimum=3)
- | \ *input*\ :
  | \ ``pAxis``\ : axis point of one face of cylinder (3D list or np.array)
  | \ ``vAxis``\ : vector representing the cylinder's axis (3D list or np.array)
  | \ ``radius``\ : positive value representing radius of cylinder
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``nTiles``\ : used to determine resolution of cylinder >=3; use larger values for finer resolution
  | \ ``angleRange``\ : given in rad, to draw only part of cylinder (halfcylinder, etc.); for full range use [0..2 \* pi]
  | \ ``lastFace``\ : if angleRange != [0,2\*pi], then the faces of the open cylinder are shown with lastFace = True
  | \ ``cutPlain``\ : only used for angleRange != [0,2\*pi]; if True, a plane is cut through the part of the cylinder; if False, the cylinder becomes a cake shape ...
  | \ ``addEdges``\ : if True, edges are added in TriangleList of GraphicsData; if addEdges is integer, additional int(addEdges) lines are added on the cylinder mantle
  | \ ``edgeColor``\ : optional color for edges
  | \ ``addFaces``\ : if False, no faces are added (only edges)
  | \ ``alternatingColor``\ : if given, optionally another color in order to see rotation of solid; only works, if angleRange=[0,2\*pi]
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Ex), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM), \ `coordinateSpringDamperExt.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateSpringDamperExt.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdatarigidlink:

Function: GraphicsDataRigidLink
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataRigidLink <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L883>`__\ (\ ``p0``\ , \ ``p1``\ , \ ``axis0 = [0,0,0]``\ , \ ``axis1 = [0,0,0]``\ , \ ``radius = [0.1,0.1]``\ , \ ``thickness = 0.05``\ , \ ``width = [0.05,0.05]``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 16``\ )

- | \ *function description*\ :
  | generate graphics data for a planar Link between the two joint positions, having two axes
- | \ *input*\ :
  | \ ``p0``\ : joint0 center position
  | \ ``p1``\ : joint1 center position
  | \ ``axis0``\ : direction of rotation axis at p0, if drawn as a cylinder; [0,0,0] otherwise
  | \ ``axis1``\ : direction of rotation axis of p1, if drawn as a cylinder; [0,0,0] otherwise
  | \ ``radius``\ : list of two radii [radius0, radius1], being the two radii of the joints drawn by a cylinder or sphere
  | \ ``width``\ : list of two widths [width0, width1], being the two widths of the joints drawn by a cylinder; ignored for sphere
  | \ ``thickness``\ : the thickness of the link (shaft) between the two joint positions; thickness in z-direction or diameter (cylinder)
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``nTiles``\ : used to determine resolution of cylinder >=3; use larger values for finer resolution
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Ex), \ `multiMbsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/multiMbsTest.py>`_\  (Ex), \ `rigidBodyTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial.py>`_\  (Ex), \ `rigidBodyTutorial2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial2.py>`_\  (Ex), \ `fourBarMechanismIftomm.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismIftomm.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM), \ `sliderCrank3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dtest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdatafromstlfiletxt:

Function: GraphicsDataFromSTLfileTxt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataFromSTLfileTxt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L940>`__\ (\ ``fileName``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``verbose = False``\ , \ ``invertNormals = True``\ , \ ``invertTriangles = True``\ )

- | \ *function description*\ :
  | generate graphics data from STL file (text format!) and use color for visualization; this function is slow, use stl binary files with GraphicsDataFromSTLfile(...)
- | \ *input*\ :
  | \ ``fileName``\ : string containing directory and filename of STL-file (in text / SCII format) to load
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``verbose``\ : if True, useful information is provided during reading
  | \ ``invertNormals``\ : if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
  | \ ``invertTriangles``\ : if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn
- | \ *output*\ :
  | creates graphicsData, inverting the STL graphics regarding normals and triangle orientations (interchanged 2nd and 3rd component of triangle index)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `stlFileImport.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stlFileImport.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-graphicsdatafromstlfile:

Function: GraphicsDataFromSTLfile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataFromSTLfile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1037>`__\ (\ ``fileName``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``verbose = False``\ , \ ``density = 0.``\ , \ ``scale = 1.``\ , \ ``Aoff = []``\ , \ ``pOff = []``\ , \ ``invertNormals = True``\ , \ ``invertTriangles = True``\ )

- | \ *function description*\ :
  | generate graphics data from STL file, allowing text or binary format; requires numpy-stl to be installed; additionally can scale, rotate and translate
- | \ *input*\ :
  | \ ``fileName``\ : string containing directory and filename of STL-file (in text / SCII format) to load
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``verbose``\ : if True, useful information is provided during reading
  | \ ``density``\ : if given and if verbose, mass, volume, inertia, etc. are computed
  | \ ``scale``\ : point coordinates are transformed by scaling factor
  | \ ``invertNormals``\ : if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
  | \ ``invertTriangles``\ : if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn
- | \ *output*\ :
  | creates graphicsData, inverting the STL graphics regarding normals and triangle orientations (interchanged 2nd and 3rd component of triangle index)
- | \ *notes*\ :
  | the model is first scaled, then rotated, then the offset pOff is added; finally min, max, mass, volume, inertia, com are computed!

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `ROSTurtle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSTurtle.py>`_\  (Ex), \ `stlFileImport.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stlFileImport.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-addedgesandsmoothennormals:

Function: AddEdgesAndSmoothenNormals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddEdgesAndSmoothenNormals <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1110>`__\ (\ ``graphicsData``\ , \ ``edgeColor = color4black``\ , \ ``edgeAngle = 0.25*pi``\ , \ ``pointTolerance = 5``\ , \ ``addEdges = True``\ , \ ``smoothNormals = True``\ , \ ``roundDigits = 5``\ , \ ``triangleColor = []``\ )

- | \ *function description*\ :
  | compute and return GraphicsData with edges and smoothend normals for mesh consisting of points and triangles (e.g., as returned from GraphicsData2PointsAndTrigs)
  | \ ``graphicsData``\ : single GraphicsData object of type TriangleList; existing edges are ignored
  | \ ``edgeColor``\ : optional color for edges
  | \ ``edgeAngle``\ : angle above which edges are added to geometry
  | \ ``roundDigits``\ : number of digits, relative to max dimensions of object, at which points are assumed to be equal
  | \ ``smoothNormals``\ : if True, algorithm tries to smoothen normals at vertices; otherwise, uses triangle normals
  | \ ``addEdges``\ : if True, edges are added in TriangleList of GraphicsData
  | \ ``triangleColor``\ : if triangleColor is set to a RGBA color, this color is used for the new triangle mesh throughout
- | \ *output*\ :
  | returns GraphicsData with added edges and smoothed normals
- | \ *notes*\ :
  | this function is suitable for STL import; it assumes that all colors in graphicsData are the same and only takes the first color!

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `stlFileImport.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stlFileImport.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-exportgraphicsdata2stl:

Function: ExportGraphicsData2STL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ExportGraphicsData2STL <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1274>`__\ (\ ``graphicsData``\ , \ ``fileName``\ , \ ``solidName = 'ExudynSolid'``\ , \ ``invertNormals = True``\ , \ ``invertTriangles = True``\ )

- | \ *function description*\ :
  | export given graphics data (only type TriangleList allowed!) to STL ascii file using fileName
- | \ *input*\ :
  | \ ``graphicsData``\ : a single GraphicsData dictionary with type='TriangleList', no list of GraphicsData
  | \ ``fileName``\ : file name including (local) path to export STL file
  | \ ``solidName``\ : optional name used in STL file
  | \ ``invertNormals``\ : if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
  | \ ``invertTriangles``\ : if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `stlFileImport.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stlFileImport.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-graphicsdatasolidofrevolution:

Function: GraphicsDataSolidOfRevolution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataSolidOfRevolution <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1343>`__\ (\ ``pAxis``\ , \ ``vAxis``\ , \ ``contour``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 16``\ , \ ``smoothContour = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color4black``\ , \ ``addFaces = True``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | generate graphics data for a solid of revolution with given 3D point and axis, 2D point list for contour, (optional)2D normals and color;
- | \ *input*\ :
  | \ ``pAxis``\ : axis point of one face of solid of revolution (3D list or np.array)
  | \ ``vAxis``\ : vector representing the solid of revolution's axis (3D list or np.array)
  | \ ``contour``\ : a list of 2D-points, specifying the contour (x=axis, y=radius), e.g.: [[0,0],[0,0.1],[1,0.1]]
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``nTiles``\ : used to determine resolution of solid; use larger values for finer resolution
  | \ ``smoothContour``\ : if True, the contour is made smooth by auto-computing normals to the contour
  | \ ``addEdges``\ : True or number of edges along revolution mantle; for optimal drawing, nTiles shall be multiple addEdges
  | \ ``edgeColor``\ : optional color for edges
  | \ ``addFaces``\ : if False, no faces are added (only edges)
  | \ ``alternatingColor``\ : add a second color, which enables to see the rotation of the solid
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects
- | \ *example*\ :

.. code-block:: python

  #simple contour, using list of 2D points:
  contour=[[0,0.2],[0.3,0.2],[0.5,0.3],[0.7,0.4],[1,0.4],[1,0.]]
  rev1 = GraphicsDataSolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0],
                                       contour=contour, color=color4red,
                                       alternatingColor=color4grey)
  #draw torus:
  contour=[]
  r = 0.2 #small radius of torus
  R = 0.5 #big radius of torus
  nc = 16 #discretization of torus
  for i in range(nc+3): #+3 in order to remove boundary effects
      contour+=[[r*cos(i/nc*pi*2),R+r*sin(i/nc*pi*2)]]
  #use smoothContour to make torus looking smooth
  rev2 = GraphicsDataSolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0],
                                       contour=contour, color=color4red,
                                       nTiles = 64, smoothContour=True)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex), \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdataarrow:

Function: GraphicsDataArrow
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataArrow <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1482>`__\ (\ ``pAxis``\ , \ ``vAxis``\ , \ ``radius``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``headFactor = 2``\ , \ ``headStretch = 4``\ , \ ``nTiles = 12``\ )

- | \ *function description*\ :
  | generate graphics data for an arrow with given origin, axis, shaft radius, optional size factors for head and color; nTiles gives the number of tiles (minimum=3)
- | \ *input*\ :
  | \ ``pAxis``\ : axis point of the origin (base) of the arrow (3D list or np.array)
  | \ ``vAxis``\ : vector representing the vector pointing from the origin to the tip (head) of the error (3D list or np.array)
  | \ ``radius``\ : positive value representing radius of shaft cylinder
  | \ ``headFactor``\ : positive value representing the ratio between head's radius and the shaft radius
  | \ ``headStretch``\ : positive value representing the ratio between the head's radius and the head's length
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``nTiles``\ : used to determine resolution of arrow (of revolution object) >=3; use larger values for finer resolution
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `reevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystem.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdatabasis:

Function: GraphicsDataBasis
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataBasis <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1500>`__\ (\ ``origin = [0,0,0]``\ , \ ``rotationMatrix = np.eye(3)``\ , \ ``length = 1``\ , \ ``colors = [color4red, color4green, color4blue]``\ , \ ``headFactor = 2``\ , \ ``headStretch = 4``\ , \ ``nTiles = 12``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | generate graphics data for three arrows representing an orthogonal basis with point of origin, shaft radius, optional size factors for head and colors; nTiles gives the number of tiles (minimum=3)
- | \ *input*\ :
  | \ ``origin``\ : point of the origin of the base (3D list or np.array)
  | \ ``rotationMatrix``\ : optional transformation, which rotates the basis vectors
  | \ ``length``\ : positive value representing lengths of arrows for basis
  | \ ``colors``\ : provided as list of 3 colors (list of 4 RGBA values)
  | \ ``headFactor``\ : positive value representing the ratio between head's radius and the shaft radius
  | \ ``headStretch``\ : positive value representing the ratio between the head's radius and the head's length
  | \ ``nTiles``\ : used to determine resolution of arrows of basis (of revolution object) >=3; use larger values for finer resolution
  | \ ``radius``\ : positive value representing radius of arrows; default: radius = 0.01\*length
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `gyroStability.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/gyroStability.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdataframe:

Function: GraphicsDataFrame
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataFrame <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1523>`__\ (\ ``HT = np.eye(4)``\ , \ ``length = 1``\ , \ ``colors = [color4red, color4green, color4blue]``\ , \ ``headFactor = 2``\ , \ ``headStretch = 4``\ , \ ``nTiles = 12``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | generate graphics data for frame (similar to GraphicsDataBasis), showing three arrows representing an orthogonal basis for the homogeneous transformation HT; optional shaft radius, optional size factors for head and colors; nTiles gives the number of tiles (minimum=3)
- | \ *input*\ :
  | \ ``HT``\ : homogeneous transformation representing frame
  | \ ``length``\ : positive value representing lengths of arrows for basis
  | \ ``colors``\ : provided as list of 3 colors (list of 4 RGBA values)
  | \ ``headFactor``\ : positive value representing the ratio between head's radius and the shaft radius
  | \ ``headStretch``\ : positive value representing the ratio between the head's radius and the head's length
  | \ ``nTiles``\ : used to determine resolution of arrows of basis (of revolution object) >=3; use larger values for finer resolution
  | \ ``radius``\ : positive value representing radius of arrows; default: radius = 0.01\*length
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-graphicsdataquad:

Function: GraphicsDataQuad
^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataQuad <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1555>`__\ (\ ``pList``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | generate graphics data for simple quad with option for checkerboard pattern;
  | points are arranged counter-clock-wise, e.g.: p0=[0,0,0], p1=[1,0,0], p2=[1,1,0], p3=[0,1,0]
- | \ *input*\ :
  | \ ``pList``\ : list of 4 quad points [[x0,y0,z0],[x1,y1,z1],...]
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``alternatingColor``\ : second color; if defined, a checkerboard pattern (default: 10x10) is drawn with color and alternatingColor
  | \ ``nTiles``\ : number of tiles for checkerboard pattern (default: 10)
  | \ ``nTilesY``\ : if defined, use number of tiles in y-direction different from x-direction (=nTiles)
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects
- | \ *example*\ :

.. code-block:: python

  plane = GraphicsDataQuad([[-8, 0, -8],[ 8, 0, -8,],[ 8, 0, 8],[-8, 0, 8]],
                           color4darkgrey, nTiles=8,
                           alternatingColor=color4lightgrey)
  oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                        visualization=VObjectGround(graphicsData=[plane])))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Ex), \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Ex), \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Ex), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Ex), \ `simulateInteractively.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/simulateInteractively.py>`_\  (Ex)



----


.. _sec-graphicsdatautilities-graphicsdatacheckerboard:

Function: GraphicsDataCheckerBoard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataCheckerBoard <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1626>`__\ (\ ``point = [0,0,0]``\ , \ ``normal = [0,0,1]``\ , \ ``size = 1``\ , \ ``color = color4lightgrey``\ , \ ``alternatingColor = color4lightgrey2``\ , \ ``nTiles = 10``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | function to generate checkerboard background;
  | points are arranged counter-clock-wise, e.g.:
- | \ *input*\ :
  | \ ``point``\ : midpoint of pattern provided as list or np.array
  | \ ``normal``\ : normal to plane provided as list or np.array
  | \ ``size``\ : dimension of first side length of quad
  | \ ``size2``\ : dimension of second side length of quad
  | \ ``color``\ : provided as list of 4 RGBA values
  | \ ``alternatingColor``\ : second color; if defined, a checkerboard pattern (default: 10x10) is drawn with color and alternatingColor
  | \ ``nTiles``\ : number of tiles for checkerboard pattern in first direction
  | \ ``nTiles2``\ : number of tiles for checkerboard pattern in second direction; default: nTiles
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects
- | \ *example*\ :

.. code-block:: python

  plane = GraphicsDataCheckerBoard(normal=[0,0,1], size=5)
  oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                        visualization=VObjectGround(graphicsData=[plane])))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Ex), \ `ANCFoutputTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFoutputTest.py>`_\  (TM), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `connectorGravityTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/connectorGravityTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-computetriangularmesh:

Function: ComputeTriangularMesh
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeTriangularMesh <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1670>`__\ (\ ``vertices``\ , \ ``segments``\ )

- | \ *function description*\ :
  | helper function to compute triangular mesh from list of vertices (=points) and segments;
  | computes triangular meshes for non-convex case. In order to make it efficient, it first computes
  | neighbors and then defines triangles at segments to be inside/outside. Finally neighboring
  | relations are used to define all triangles inside/outside
  | finally only returns triangles that are inside the segments
- | \ *input*\ :
  | \ ``vertices``\ : list of pairs of coordinates of vertices in mesh [x,y]
  | \ ``segments``\ : list of segments, which are pairs of node numbers [i,j], defining the boundary of the mesh;
  | the ordering of the nodes is such that left triangle = inside, right triangle = outside, compare example with segment [V1,V2]:

  | inside
  | V1         V2
  | O----------O
  | outside
- | \ *output*\ :
  | triangulation structure of Delaunay(...), see scipy.spatial.Delaunaystructure, containing all simplices (=triangles)
- | \ *notes*\ :
  | Delauney will not work if points are duplicated; you must first create point lists without duplicated points!
- | \ *example*\ :

.. code-block:: python

  points = np.array([[0, 0], [0, 2], [2, 2], [2, 1], [1, 1], [0, 1], [1, 0]])
  segments = [len(points)-1,0]
  for i in range(len(points)-1):
      segments += [i,i+1]
  tri = ComputeTriangularMesh(points, segments)
  print(tri.simplices)




----


.. _sec-graphicsdatautilities-segmentsfrompoints:

Function: SegmentsFromPoints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SegmentsFromPoints <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1762>`__\ (\ ``points``\ , \ ``pointIndexOffset = 0``\ , \ ``invert = False``\ , \ ``closeCurve = True``\ )

- | \ *function description*\ :
  | convert point list into segments (indices to points); point indices start with pointIndexOffset
- | \ *input*\ :
  | \ ``invert``\ : True: circle defines outter boundary; False: circle cuts out geometry inside a geometry
  | \ ``pointIndexOffset``\ : point indices start with pointIndexOffset
- | \ *output*\ :
  | return segments, containing list of lists of point indices for segments



----


.. _sec-graphicsdatautilities-circlepointsandsegments:

Function: CirclePointsAndSegments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CirclePointsAndSegments <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1789>`__\ (\ ``center = [0,0]``\ , \ ``radius = 0.1``\ , \ ``invert = False``\ , \ ``pointIndexOffset = 0``\ , \ ``nTiles = 16``\ )

- | \ *function description*\ :
  | create points and segments, used in GraphicsDataSolidExtrusion(...) for circle with given parameters
- | \ *input*\ :
  | \ ``center``\ : 2D center point (list/numpy array) for circle center
  | \ ``radius``\ : radius of circle
  | \ ``invert``\ : True: circle defines outter boundary; False: circle cuts out geometry inside a geometry
  | \ ``pointIndexOffset``\ : point indices start with pointIndexOffset
  | \ ``nTiles``\ : number of tiles/segments for circle creation (higher is finer)
- | \ *output*\ :
  | return [points, segments], both containing lists of lists
- | \ *notes*\ :
  | geometries may not intersect!



----


.. _sec-graphicsdatautilities-graphicsdatasolidextrusion:

Function: GraphicsDataSolidExtrusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataSolidExtrusion <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L1824>`__\ (\ ``vertices``\ , \ ``segments``\ , \ ``height``\ , \ ``rot = np.diag([1,1,1])``\ , \ ``pOff = [0,0,0]``\ , \ ``color = [0,0,0,1]``\ , \ ``smoothNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color4black``\ , \ ``addFaces = True``\ )

- | \ *function description*\ :
  | create graphicsData for solid extrusion based on 2D points and segments; by default, the extrusion is performed in z-direction;
  | additional transformations are possible to translate and rotate the extruded body;
- | \ *input*\ :
  | \ ``vertices``\ : list of pairs of coordinates of vertices in mesh [x,y], see ComputeTriangularMesh(...)
  | \ ``segments``\ : list of segments, which are pairs of node numbers [i,j], defining the boundary of the mesh;
  | the ordering of the nodes is such that left triangle = inside, right triangle = outside; see ComputeTriangularMesh(...)
  | \ ``height``\ :   height of extruded object
  | \ ``rot``\ :      rotation matrix, which the extruded object point coordinates are multiplied with before adding offset
  | \ ``pOff``\ :     3D offset vector added to extruded coordinates; the z-coordinate of the extrusion object obtains 0 for the base plane, z=height for the top plane
  | \ ``smoothNormals``\ : if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally
  | \ ``addEdges``\ : if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
  | \ ``edgeColor``\ : optional color for edges
  | \ ``addFaces``\ : if False, no faces are added (only edges)
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex)

