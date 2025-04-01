
.. _sec-module-graphics:

Module: graphics
================

This module newly introduces revised graphics functions, coherent with Exudyn terminology;
it provides basic graphics elements like cuboid, cylinder, sphere, solid of revolution, etc.;
offers also some advanced functions for STL import and mesh manipulation; 
for some advanced functions see graphicsDataUtilties;
GraphicsData helper functions generate dictionaries which contain line, text or triangle primitives for drawing in Exudyn using OpenGL.

- Author:    Johannes Gerstmayr 
- Date:      2024-05-10 (created) 


.. _sec-graphics-sphere:

Function: Sphere
^^^^^^^^^^^^^^^^
`Sphere <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L120>`__\ (\ ``point = [0,0,0]``\ , \ ``radius = 0.1``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 8``\ , \ ``addEdges = False``\ , \ ``edgeColor = color.black``\ , \ ``addFaces = True``\ )

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

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `connectorGravityTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/connectorGravityTest.py>`_\  (TM), \ `contactCoordinateTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactCoordinateTest.py>`_\  (TM), \ `contactSphereSphereTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactSphereSphereTest.py>`_\  (TM)



----


.. _sec-graphics-lines:

Function: Lines
^^^^^^^^^^^^^^^
`Lines <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L243>`__\ (\ ``pList``\ , \ ``color = [0.,0.,0.,1.]``\ )

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
  gLine=graphics.Lines([[0,0,0],[1,0,0],[2,0.5,0]], color=color.red)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex), \ `doublePendulum2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/doublePendulum2D.py>`_\  (Ex), \ `rigid3Dexample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigid3Dexample.py>`_\  (Ex), \ `simple4linkPendulumBing.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/simple4linkPendulumBing.py>`_\  (Ex), \ `doublePendulum2DControl.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/doublePendulum2DControl.py>`_\  (TM), \ `genericJointUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/genericJointUserFunctionTest.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `sphericalJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sphericalJointTest.py>`_\  (TM)



----


.. _sec-graphics-circle:

Function: Circle
^^^^^^^^^^^^^^^^
`Circle <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L260>`__\ (\ ``point = [0,0,0]``\ , \ ``radius = 1``\ , \ ``color = [0.,0.,0.,1.]``\ )

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

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex)



----


.. _sec-graphics-text:

Function: Text
^^^^^^^^^^^^^^
`Text <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L272>`__\ (\ ``point = [0,0,0]``\ , \ ``text = ''``\ , \ ``color = [0.,0.,0.,1.]``\ )

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

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex), \ `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveGeometry.py>`_\  (Ex)



----


.. _sec-graphics-cuboid:

Function: Cuboid
^^^^^^^^^^^^^^^^
`Cuboid <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L286>`__\ (\ ``pList``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``faces = [1,1,1,1,1,1]``\ , \ ``addNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color.black``\ , \ ``addFaces = True``\ )

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

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Ex), \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `complexEigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/complexEigenvaluesTest.py>`_\  (TM)



----


.. _sec-graphics-brickxyz:

Function: BrickXYZ
^^^^^^^^^^^^^^^^^^
`BrickXYZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L374>`__\ (\ ``xMin``\ , \ ``yMin``\ , \ ``zMin``\ , \ ``xMax``\ , \ ``yMax``\ , \ ``zMax``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``addNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color.black``\ , \ ``addFaces = True``\ )

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
- | \ *notes*\ :
  | DEPRECATED

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Ex), \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Ex), \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Ex), \ `performanceMultiThreadingNG.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/performanceMultiThreadingNG.py>`_\  (Ex), \ `rigidBodyIMUtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyIMUtest.py>`_\  (Ex), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `explicitLieGroupIntegratorPythonTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorPythonTest.py>`_\  (TM), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TM)



----


.. _sec-graphics-brick:

Function: Brick
^^^^^^^^^^^^^^^
`Brick <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L392>`__\ (\ ``centerPoint = [0,0,0]``\ , \ ``size = [0.1,0.1,0.1]``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``addNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color.black``\ , \ ``addFaces = True``\ )

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

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `complexEigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/complexEigenvaluesTest.py>`_\  (TM)



----


.. _sec-graphics-cylinder:

Function: Cylinder
^^^^^^^^^^^^^^^^^^
`Cylinder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L424>`__\ (\ ``pAxis = [0,0,0]``\ , \ ``vAxis = [0,0,1]``\ , \ ``radius = 0.1``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 16``\ , \ ``angleRange = [0,2*pi]``\ , \ ``lastFace = True``\ , \ ``cutPlain = True``\ , \ ``addEdges = False``\ , \ ``edgeColor = color.black``\ , \ ``addFaces = True``\ , \ ``**kwargs``\ )

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

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Ex), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM)



----


.. _sec-graphics-rigidlink:

Function: RigidLink
^^^^^^^^^^^^^^^^^^^
`RigidLink <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L623>`__\ (\ ``p0``\ , \ ``p1``\ , \ ``axis0 = [0,0,0]``\ , \ ``axis1 = [0,0,0]``\ , \ ``radius = [0.1,0.1]``\ , \ ``thickness = 0.05``\ , \ ``width = [0.05,0.05]``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 16``\ )

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

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Ex), \ `multiMbsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/multiMbsTest.py>`_\  (Ex), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Ex), \ `fourBarMechanismIftomm.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismIftomm.py>`_\  (TM), \ `rollingDiscTangentialForces.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingDiscTangentialForces.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM)



----


.. _sec-graphics-solidofrevolution:

Function: SolidOfRevolution
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SolidOfRevolution <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L705>`__\ (\ ``pAxis``\ , \ ``vAxis``\ , \ ``contour``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``nTiles = 16``\ , \ ``smoothContour = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color.black``\ , \ ``addFaces = True``\ , \ ``**kwargs``\ )

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
  rev1 = graphics.SolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0],
                                       contour=contour, color=color.red,
                                       alternatingColor=color.grey)
  #draw torus:
  contour=[]
  r = 0.2 #small radius of torus
  R = 0.5 #big radius of torus
  nc = 16 #discretization of torus
  for i in range(nc+3): #+3 in order to remove boundary effects
      contour+=[[r*cos(i/nc*pi*2),R+r*sin(i/nc*pi*2)]]
  #use smoothContour to make torus looking smooth
  rev2 = graphics.SolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0],
                                       contour=contour, color=color.red,
                                       nTiles = 64, smoothContour=True)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `gyroStability.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/gyroStability.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex), \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TM)



----


.. _sec-graphics-arrow:

Function: Arrow
^^^^^^^^^^^^^^^
`Arrow <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L844>`__\ (\ ``pAxis``\ , \ ``vAxis``\ , \ ``radius``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``headFactor = 2``\ , \ ``headStretch = 4``\ , \ ``nTiles = 12``\ )

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


.. _sec-graphics-basis:

Function: Basis
^^^^^^^^^^^^^^^
`Basis <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L862>`__\ (\ ``origin = [0,0,0]``\ , \ ``rotationMatrix = np.eye(3)``\ , \ ``length = 1``\ , \ ``colors = [color.red, color.green, color.blue]``\ , \ ``headFactor = 2``\ , \ ``headStretch = 4``\ , \ ``nTiles = 12``\ , \ ``**kwargs``\ )

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

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `gyroStability.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/gyroStability.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `createRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createRollingDiscTest.py>`_\  (TM), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM)



----


.. _sec-graphics-frame:

Function: Frame
^^^^^^^^^^^^^^^
`Frame <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L885>`__\ (\ ``HT = np.eye(4)``\ , \ ``length = 1``\ , \ ``colors = [color.red, color.green, color.blue]``\ , \ ``headFactor = 2``\ , \ ``headStretch = 4``\ , \ ``nTiles = 12``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | generate graphics data for frame (similar to Basis), showing three arrows representing an orthogonal basis for the homogeneous transformation HT; optional shaft radius, optional size factors for head and colors; nTiles gives the number of tiles (minimum=3)
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

    \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM)



----


.. _sec-graphics-quad:

Function: Quad
^^^^^^^^^^^^^^
`Quad <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L917>`__\ (\ ``pList``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``**kwargs``\ )

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

  plane = graphics.Quad([[-8, 0, -8],[ 8, 0, -8,],[ 8, 0, 8],[-8, 0, 8]],
                           color.darkgrey, nTiles=8,
                           alternatingColor=color.lightgrey)
  oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                        visualization=VObjectGround(graphicsData=[plane])))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Ex), \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Ex), \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Ex), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Ex), \ `simulateInteractively.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/simulateInteractively.py>`_\  (Ex)



----


.. _sec-graphics-checkerboard:

Function: CheckerBoard
^^^^^^^^^^^^^^^^^^^^^^
`CheckerBoard <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L989>`__\ (\ ``point = [0,0,0]``\ , \ ``normal = [0,0,1]``\ , \ ``size = 1``\ , \ ``color = color.lightgrey``\ , \ ``alternatingColor = color.lightgrey2``\ , \ ``nTiles = 10``\ , \ ``**kwargs``\ )

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

  plane = graphics.CheckerBoard(normal=[0,0,1], size=5)
  oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                        visualization=VObjectGround(graphicsData=[plane])))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `ANCFoutputTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFoutputTest.py>`_\  (TM), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `connectorGravityTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/connectorGravityTest.py>`_\  (TM)



----


.. _sec-graphics-solidextrusion:

Function: SolidExtrusion
^^^^^^^^^^^^^^^^^^^^^^^^
`SolidExtrusion <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1023>`__\ (\ ``vertices``\ , \ ``segments``\ , \ ``height``\ , \ ``rot = np.diag([1,1,1])``\ , \ ``pOff = [0,0,0]``\ , \ ``color = [0,0,0,1]``\ , \ ``smoothNormals = False``\ , \ ``addEdges = False``\ , \ ``edgeColor = color.black``\ , \ ``addFaces = True``\ )

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

    \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `simulatorCouplingTwoMbs.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/simulatorCouplingTwoMbs.py>`_\  (TM)



----


.. _sec-graphics-frompointsandtrigs:

Function: FromPointsAndTrigs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`FromPointsAndTrigs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1155>`__\ (\ ``points``\ , \ ``triangles``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``normals = None``\ )

- | \ *function description*\ :
  | convert triangles and points as returned from graphics.ToPointsAndTrigs(...) to GraphicsData; additionally, normals and color(s) can be provided
- | \ *input*\ :
  | \ ``points``\ : list or np.array with np rows of 3 columns (floats) per point (with np points)
  | \ ``triangles``\ : list or np.array with 3 int per triangle (0-based indices to triangles), giving a matrix with nt rows and 3 columns (with nt triangles)
  | \ ``color``\ : provided as list of 4 RGBA values or single list of (np)\*[4 RGBA values]
  | \ ``normals``\ : if not None, they have to be provided per point (as matrix with nPand will be added to returned GraphicsData
- | \ *output*\ :
  | returns GraphicsData with type TriangleList

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveGeometry.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `generalContactImplicit1.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactImplicit1.py>`_\  (TM), \ `generalContactImplicit2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactImplicit2.py>`_\  (TM)



----


.. _sec-graphics-topointsandtrigs:

Function: ToPointsAndTrigs
^^^^^^^^^^^^^^^^^^^^^^^^^^
`ToPointsAndTrigs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1187>`__\ (\ ``g``\ )

- | \ *function description*\ :
  | convert graphics data into list of points and list of triangle indices (triplets)
- | \ *input*\ :
  | g contains a GraphicsData with type TriangleList
- | \ *output*\ :
  | returns [points, triangles], with points as list of np.array with 3 floats per point and triangles as a list of np.array with 3 int per triangle (0-based indices to points)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `reinforcementLearningRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reinforcementLearningRobot.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactCylinderTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactCylinderTest.py>`_\  (TM), \ `generalContactCylinderTrigsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactCylinderTrigsTest.py>`_\  (TM)



----


.. _sec-graphics-move:

Function: Move
^^^^^^^^^^^^^^
`Move <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1212>`__\ (\ ``g``\ , \ ``pOff``\ , \ ``Aoff = None``\ )

- | \ *function description*\ :
  | add rigid body transformation to GraphicsData, using position offset (global) pOff (list or np.array) and rotation Aoff (transforms local to global coordinates; list of lists or np.array); see Aoff how to scale coordinates!
- | \ *input*\ :
  | \ ``g``\ : graphicsData to be transformed
  | \ ``pOff``\ : 3D offset as list or numpy.array added to rotated points
  | \ ``Aoff``\ : 3D rotation matrix as list of lists or numpy.array with shape (3,3); if A is scaled by factor, e.g. using 0.001\*np.eye(3), you can also scale the coordinates; if Aoff=None, no rotation is performed
- | \ *output*\ :
  | returns new graphcsData object to be used for drawing in objects
- | \ *notes*\ :
  | transformation corresponds to HomogeneousTransformation(Aoff, pOff), transforming original coordinates v into vNew = pOff + Aoff @ v

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Ex), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TM)



----


.. _sec-graphics-mergetrianglelists:

Function: MergeTriangleLists
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`MergeTriangleLists <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1263>`__\ (\ ``g1``\ , \ ``g2``\ )

- | \ *function description*\ :
  | merge 2 different graphics data with triangle lists
- | \ *input*\ :
  | graphicsData dictionaries g1 and g2 obtained from GraphicsData functions
- | \ *output*\ :
  | one graphicsData dictionary with single triangle lists and compatible points and normals, to be used in visualization of EXUDYN objects; edges are merged; edgeColor is taken from graphicsData g1

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



----


.. _sec-graphics-fromstlfileascii:

Function: FromSTLfileASCII
^^^^^^^^^^^^^^^^^^^^^^^^^^
`FromSTLfileASCII <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1318>`__\ (\ ``fileName``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``verbose = False``\ , \ ``invertNormals = True``\ , \ ``invertTriangles = True``\ )

- | \ *function description*\ :
  | generate graphics data from STL file (text format!) and use color for visualization; this function is slow, use stl binary files with FromSTLfile(...)
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


.. _sec-graphics-fromstlfile:

Function: FromSTLfile
^^^^^^^^^^^^^^^^^^^^^
`FromSTLfile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1415>`__\ (\ ``fileName``\ , \ ``color = [0.,0.,0.,1.]``\ , \ ``verbose = False``\ , \ ``density = 0.``\ , \ ``scale = 1.``\ , \ ``Aoff = []``\ , \ ``pOff = []``\ , \ ``invertNormals = True``\ , \ ``invertTriangles = True``\ )

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


.. _sec-graphics-addedgesandsmoothennormals:

Function: AddEdgesAndSmoothenNormals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddEdgesAndSmoothenNormals <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1486>`__\ (\ ``graphicsData``\ , \ ``edgeColor = color.black``\ , \ ``edgeAngle = 0.25*pi``\ , \ ``pointTolerance = 5``\ , \ ``addEdges = True``\ , \ ``smoothNormals = True``\ , \ ``roundDigits = 5``\ , \ ``triangleColor = []``\ )

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

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveGeometry.py>`_\  (Ex), \ `stlFileImport.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stlFileImport.py>`_\  (Ex)



----


.. _sec-graphics-exportstl:

Function: ExportSTL
^^^^^^^^^^^^^^^^^^^
`ExportSTL <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphics.py\#L1650>`__\ (\ ``graphicsData``\ , \ ``fileName``\ , \ ``solidName = 'ExudynSolid'``\ , \ ``invertNormals = True``\ , \ ``invertTriangles = True``\ )

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

