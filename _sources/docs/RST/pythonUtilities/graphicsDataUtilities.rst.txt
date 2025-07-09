
.. _sec-module-graphicsdatautilities:

Module: graphicsDataUtilities
=============================

Utility functions for visualization, which provides functions for special graphics manipulation, colors, mesh manipulation, etc.;
note that specific function for GraphicsData creation now moved into the graphics submodule;
includes functionality like mesh manipulation and some helper functions

- Author:    Johannes Gerstmayr 
- Date:      2020-07-26 (created)  Modified: 2024-05-10 (moved primitive functions to graphics) 
- | Notes:
  | Some useful colors are defined, using RGBA (Red, Green, Blue and Alpha = opacity) channels            in the range [0,1], e.g., red = [1,0,0,1].
  | Available colors are: color4red, color4green, color4blue, color4cyan, color4magenta, color4yellow, color4orange, color4pink, color4lawngreen, color4violet, color4springgreen, color4dodgerblue, color4grey, color4darkgrey, color4lightgrey, color4lightred, color4lightgreen, color4steelblue, color4brown, color4black, color4darkgrey2, color4lightgrey2, color4white
  | Additionally, a list of 16 colors 'color4list' is available, which is intended to be used, e.g., for creating n bodies with different colors


.. _sec-graphicsdatautilities-switchtripletorder:

Function: SwitchTripletOrder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SwitchTripletOrder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L85>`__\ (\ ``vector``\ )

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
`ComputeTriangleNormal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L95>`__\ (\ ``p0``\ , \ ``p1``\ , \ ``p2``\ )

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
`ComputeTriangleArea <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L108>`__\ (\ ``p0``\ , \ ``p1``\ , \ ``p2``\ )

- | \ *function description*\ :
  | compute area of triangle given by 3 points
- | \ *input*\ :
  | 3D vector as list or as np.array
- | \ *output*\ :
  | area as float



----


.. _sec-graphicsdatautilities-compute6nodetrigsnormals:

Function: Compute6NodeTrigsNormals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Compute6NodeTrigsNormals <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L115>`__\ (\ ``elementNodes``\ )

- | \ *function description*\ :
  | Internal function: compute normals to 6-node triangular surface given by elementNodes
  | \ ``input``\ : elementNodes given as np.array with 6 node vectors in rows; node ordering must follow Netgen order, see the local coordinates in the function
- | \ *output*\ :
  | returns np.array with 6 normals in rows



----


.. _sec-graphicsdatautilities-refinemesh:

Function: RefineMesh
^^^^^^^^^^^^^^^^^^^^
`RefineMesh <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L186>`__\ (\ ``points``\ , \ ``triangles``\ )

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

    \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `tippeTop.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/tippeTop.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactCylinderTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactCylinderTest.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `generalContactImplicit1.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactImplicit1.py>`_\  (TM), \ `generalContactImplicit2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactImplicit2.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-shrinkmeshnormaltosurface:

Function: ShrinkMeshNormalToSurface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ShrinkMeshNormalToSurface <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L238>`__\ (\ ``points``\ , \ ``triangles``\ , \ ``distance``\ )

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


.. _sec-graphicsdatautilities-computetriangularmesh:

Function: ComputeTriangularMesh
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeTriangularMesh <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L286>`__\ (\ ``vertices``\ , \ ``segments``\ )

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
  exudyn.Print(tri.simplices)




----


.. _sec-graphicsdatautilities-segmentsfrompoints:

Function: SegmentsFromPoints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SegmentsFromPoints <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L370>`__\ (\ ``points``\ , \ ``pointIndexOffset = 0``\ , \ ``invert = False``\ , \ ``closeCurve = True``\ )

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
`CirclePointsAndSegments <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L397>`__\ (\ ``center = [0,0]``\ , \ ``radius = 0.1``\ , \ ``invert = False``\ , \ ``pointIndexOffset = 0``\ , \ ``nTiles = 16``\ )

- | \ *function description*\ :
  | create points and segments, used in SolidExtrusion(...) for circle with given parameters
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


.. _sec-graphicsdatautilities-graphicsdatarectangle:

Function: GraphicsDataRectangle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataRectangle <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L421>`__\ (\ ``xMin``\ , \ ``yMin``\ , \ ``xMax``\ , \ ``yMax``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | generate graphics data for 2D rectangle
- | \ *input*\ :
  | minimal and maximal cartesian coordinates in (x/y) plane; color provided as list of 4 RGBA values
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects
- | \ *notes*\ :
  | DEPRECATED

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Ex), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Ex), \ `lavalRotor2Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lavalRotor2Dtest.py>`_\  (Ex), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesTest.py>`_\  (Ex), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TM), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TM), \ `ANCFslidingAndALEjointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFslidingAndALEjointTest.py>`_\  (TM)



----


.. _sec-graphicsdatautilities-graphicsdataorthocubelines:

Function: GraphicsDataOrthoCubeLines
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GraphicsDataOrthoCubeLines <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/graphicsDataUtilities.py\#L433>`__\ (\ ``xMin``\ , \ ``yMin``\ , \ ``zMin``\ , \ ``xMax``\ , \ ``yMax``\ , \ ``zMax``\ , \ ``color = [0.,0.,0.,1.]``\ )

- | \ *function description*\ :
  | generate graphics data for orthogonal block drawn with lines
- | \ *input*\ :
  | minimal and maximal cartesian coordinates for orthogonal cube; color provided as list of 4 RGBA values
- | \ *output*\ :
  | graphicsData dictionary, to be used in visualization of EXUDYN objects
- | \ *notes*\ :
  | DEPRECATED

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `rigid3Dexample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigid3Dexample.py>`_\  (Ex), \ `genericJointUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/genericJointUserFunctionTest.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `sphericalJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sphericalJointTest.py>`_\  (TM)

