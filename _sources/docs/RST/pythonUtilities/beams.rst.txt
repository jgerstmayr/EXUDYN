
.. _sec-module-beams:

Module: beams
=============

Beam utility functions, e.g. for creation of sequences of straight or curved beams.

- Author:    Johannes Gerstmayr 
- Date:      2022-01-30 (created) 
- Notes:     For a list of plot colors useful for matplotlib, see also utilities.PlotLineCode(...) 


.. _sec-beams-generatestraightlineancfcable2d:

Function: GenerateStraightLineANCFCable2D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GenerateStraightLineANCFCable2D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L38>`__\ (\ ``mbs``\ , \ ``positionOfNode0``\ , \ ``positionOfNode1``\ , \ ``numberOfElements``\ , \ ``cableTemplate``\ , \ ``massProportionalLoad = [0,0,0]``\ , \ ``fixedConstraintsNode0 = [0,0,0,0]``\ , \ ``fixedConstraintsNode1 = [0,0,0,0]``\ , \ ``nodeNumber0 = -1``\ , \ ``nodeNumber1 = -1``\ )

- | \ *function description*\ :
  | generate 2D ANCF cable elements along straight line given by two points; applies discretization (numberOfElements) and may apply gravity as well as nodal constraints
- | \ *input*\ :
  | \ ``mbs``\ : the system where ANCF cables are added
  | \ ``positionOfNode0``\ : 3D position (list or np.array) for starting point of line
  | \ ``positionOfNode1``\ : 3D position (list or np.array) for end point of line
  | \ ``numberOfElements``\ : for discretization of line
  | \ ``cableTemplate``\ : a ObjectANCFCable2D object, containing the desired cable properties; cable length and node numbers are set automatically
  | \ ``massProportionalLoad``\ : a 3D list or np.array, containing the gravity vector or zero
  | \ ``fixedConstraintsNode0``\ : a list of 4 binary values, indicating the coordinate contraints on the first node (x,y-position and x,y-slope); use None in order to apply no constraints
  | \ ``fixedConstraintsNode1``\ : a list of 4 binary values, indicating the coordinate contraints on the last node (x,y-position and x,y-slope); use None in order to apply no constraints
  | \ ``nodeNumber0``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode0
  | \ ``nodeNumber1``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode1
- | \ *output*\ :
  | returns a list containing created items [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
- | \ *notes*\ :
  | use GenerateStraightBeam instead
- | \ *example*\ :

.. code-block:: python

  see Examples/ANCF_cantilever_test.py


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `ANCFcantileverTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTest.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TM)



----


.. _sec-beams-generatestraightlineancfcable:

Function: GenerateStraightLineANCFCable
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GenerateStraightLineANCFCable <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L66>`__\ (\ ``mbs``\ , \ ``positionOfNode0``\ , \ ``positionOfNode1``\ , \ ``numberOfElements``\ , \ ``cableTemplate``\ , \ ``massProportionalLoad = [0,0,0]``\ , \ ``fixedConstraintsNode0 = [0,0,0, 0,0,0]``\ , \ ``fixedConstraintsNode1 = [0,0,0, 0,0,0]``\ , \ ``nodeNumber0 = -1``\ , \ ``nodeNumber1 = -1``\ )

- | \ *function description*\ :
  | generate 3D ANCF cable elements along straight line given by two points; applies discretization (numberOfElements) and may apply gravity as well as nodal constraints
- | \ *input*\ :
  | \ ``mbs``\ : the system where ANCF cables are added
  | \ ``positionOfNode0``\ : 3D position (list or np.array) for starting point of line
  | \ ``positionOfNode1``\ : 3D position (list or np.array) for end point of line
  | \ ``numberOfElements``\ : for discretization of line
  | \ ``cableTemplate``\ : a ObjectANCFCable object, containing the desired cable properties; cable length and node numbers are set automatically
  | \ ``massProportionalLoad``\ : a 3D list or np.array, containing the gravity vector or zero
  | \ ``fixedConstraintsNode0``\ : a list of binary values, indicating the coordinate contraints on the first node (position and slope); 4 coordinates for 2D and 6 coordinates for 3D node; use None in order to apply no constraints
  | \ ``fixedConstraintsNode1``\ : a list of binary values, indicating the coordinate contraints on the last node (position and slope); 4 coordinates for 2D and 6 coordinates for 3D node; use None in order to apply no constraints
  | \ ``nodeNumber0``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode0
  | \ ``nodeNumber1``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode1
- | \ *output*\ :
  | returns a list containing created items [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
- | \ *example*\ :

.. code-block:: python

  see Examples/ANCF_cantilever_test.py


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFcableCantilevered.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcableCantilevered.py>`_\  (Ex), \ `ANCFcable2DuserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcable2DuserFunction.py>`_\  (TM)



----


.. _sec-beams-generatestraightbeam:

Function: GenerateStraightBeam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GenerateStraightBeam <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L109>`__\ (\ ``mbs``\ , \ ``positionOfNode0``\ , \ ``positionOfNode1``\ , \ ``numberOfElements``\ , \ ``beamTemplate``\ , \ ``gravity = [0,0,0]``\ , \ ``fixedConstraintsNode0 = None``\ , \ ``fixedConstraintsNode1 = None``\ , \ ``nodeNumber0 = -1``\ , \ ``nodeNumber1 = -1``\ )

- | \ *function description*\ :
  | generic function to create beam elements along straight line given by two points; applies discretization (numberOfElements) and may apply gravity as well as nodal constraints
- | \ *input*\ :
  | \ ``mbs``\ : the system where beam elements are added
  | \ ``positionOfNode0``\ : 3D position (list or np.array) for starting point of line
  | \ ``positionOfNode1``\ : 3D position (list or np.array) for end point of line
  | \ ``numberOfElements``\ : for discretization of line
  | \ ``beamTemplate``\ : a Beam object (ObjectANCFCable2D, ObjectBeamGeometricallyExact2D, ObjectALEANCFCable2D, etc.), containing the desired beam type and properties; finite (beam) element length and node numbers are set automatically; for ALE element, the beamTemplate.nodeNumbers[2] must be set in the template and will not be overwritten
  | \ ``gravity``\ : a 3D list or np.array, containing the gravity vector or zero
  | \ ``fixedConstraintsNode0``\ : a list of binary values, indicating the coordinate contraints on the first node (position and slope); must agree with the number of coordinates in the node; use None to add no constraints
  | \ ``fixedConstraintsNode1``\ : a list of binary values, indicating the coordinate contraints on the last node (position and slope); must agree with the number of coordinates in the node; use None to add no constraints
  | \ ``nodeNumber0``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode0
  | \ ``nodeNumber1``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode1
- | \ *output*\ :
  | returns a list containing created items [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes exudyn.beams
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #example of flexible pendulum
  beamTemplate = ObjectBeamGeometricallyExact2D(physicsMassPerLength=0.02,
                      physicsCrossSectionInertia=8e-9,
                      physicsBendingStiffness=8e-4,
                      physicsAxialStiffness=2000,
                      physicsShearStiffness=650,
                      visualization=VObjectBeamGeometricallyExact2D(drawHeight = 0.002))
  #create straight beam with 10 elements, apply gravity and fix (x,y) position of node 0 (rotation left free)
  beamInfo = GenerateStraightBeam(mbs, positionOfNode0=[0,0,0], positionOfNode1=[0.5,0,0],
                                  numberOfElements=10, beamTemplate=beamTemplate,
                                  gravity=[0,-9.81,0], fixedConstraintsNode0=[1,1,0],)
  #beamInfo contains nodes, beamObjects, loads, etc.
  #Assemble and solve


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beamTutorial.py>`_\  (Ex), \ `pendulumGeomExactBeam2Dsimple.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumGeomExactBeam2Dsimple.py>`_\  (Ex)



----


.. _sec-beams-generatecirculararcancfcable2d:

Function: GenerateCircularArcANCFCable2D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GenerateCircularArcANCFCable2D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L272>`__\ (\ ``mbs``\ , \ ``positionOfNode0``\ , \ ``radius``\ , \ ``startAngle``\ , \ ``arcAngle``\ , \ ``numberOfElements``\ , \ ``cableTemplate``\ , \ ``massProportionalLoad = [0,0,0]``\ , \ ``fixedConstraintsNode0 = [0,0,0,0]``\ , \ ``fixedConstraintsNode1 = [0,0,0,0]``\ , \ ``nodeNumber0 = -1``\ , \ ``nodeNumber1 = -1``\ , \ ``setCurvedReferenceConfiguration = True``\ , \ ``verboseMode = False``\ )

- | \ *function description*\ :
  | generate cable elements along circular arc with given start point, radius, start angle (measured relative to \ :math:`x`\ -axis, in positive rotation sense) and angle of arc
- | \ *input*\ :
  | \ ``mbs``\ : the system where ANCF cables are added
  | \ ``positionOfNode0``\ : 3D position (list or np.array) for starting point of line
  | \ ``radius``\ : radius of arc
  | \ ``startAngle``\ : start angle of arc in radians  (\ :math:`0 \ldots 2 \pi`\ ), defines the direction of the slope vector, measured relative to \ :math:`x`\ -axis, in positive rotation sense
  | \ ``arcAngle``\ : total angle of arc in radians (\ :math:`0 \ldots 2 \pi`\ ), measured in positive rotation sense (negative angle reverts curvature and center point of circle)
  | \ ``numberOfElements``\ : for discretization of arc
  | \ ``cableTemplate``\ : a ObjectANCFCable2D object, containing the desired cable properties; cable length and node numbers are set automatically
  | \ ``massProportionalLoad``\ : a 3D list or np.array, containing the gravity vector or zero
  | \ ``fixedConstraintsNode0``\ : a list of 4 binary values, indicating the coordinate contraints on the first node (x,y-position and x,y-slope)
  | \ ``fixedConstraintsNode1``\ : a list of 4 binary values, indicating the coordinate contraints on the last node (x,y-position and x,y-slope)
  | \ ``nodeNumber0``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode0
  | \ ``nodeNumber1``\ : if set other than -1, this node number defines the node that shall be used at positionOfNode1
  | \ ``setCurvedReferenceConfiguration``\ : if True, the curvature +/-(1/radius) is set as a reference configuration (sign depends on arcAngle); if False, the reference configuration is straight
  | \ ``verboseMode``\ : if True, prints out information on created nodes
- | \ *output*\ :
  | returns a list [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)



----


.. _sec-beams-createreevingcurve:

Function: CreateReevingCurve
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateReevingCurve <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L426>`__\ (\ ``circleList``\ , \ ``drawingLinesPerCircle = 64``\ , \ ``numberOfANCFnodes = -1``\ , \ ``removeLastLine = False``\ , \ ``removeFirstLine = False``\ , \ ``radialOffset = 0.``\ , \ ``closedCurve = False``\ , \ ``graphicsElementsPerCircle = 64``\ , \ ``graphicsNodeSize = 0``\ , \ ``colorCircles = [0.,0.5,1.,1.]``\ , \ ``colorLines = [1.,0.5,0.,1.]``\ )

- | \ *function description*\ :
  | CreateReevingCurve for creating the geometry of a reeving system based on circles with radius and left/right side of passing the circles; left/right is seen in the direction passing from one to the next circle
- | \ *input*\ :
  | \ ``circleList``\ : list containing center position, radius and 'L' (left) or 'R' (right) passing of circle
  | \ ``radialOffset``\ : additional offset added to circles to account for half height of rope or beam
  | \ ``closedCurve``\ : if True, the system adds circleList[0] and  circleList[1] at end of list and sets removeLastLine=True and removeFirstLine=False, in order to generate a closed curve according to given circles; furthermore, the number of nodes becomes equal to the number of elements in this case
  | \ ``drawingLinesPerCircle``\ : number of lines in lineData per one revolution
  | \ ``numberOfANCFnodes``\ : if not -1, function also generates nodes with equidistant distribution along curve!
  | \ ``graphicsElementsPerCircle``\ : number of drawing lines generated in graphicsDataLines per circle revolution (larger generates better approximation of circles)
  | \ ``graphicsNodeSize``\ : if not 0, addes graphics representation of nodes generated; for check if mesh is correct
  | \ ``removeFirstLine``\ : removes first line generated, which may be unwanted
  | \ ``removeLastLine``\ : removes last line generated, which may be unwanted
  | \ ``colorCircles``\ : RGBA color for circles
  | \ ``colorLines``\ : RGBA color for lines
- | \ *output*\ :
  | return a dictionary with {'ancfPointsSlopes':ancfPointsSlopes, 'elementLengths':elementLengths, 'elementCurvatures':elementCurvatures, 'totalLength':totalLength, 'circleData':circle2D, 'graphicsDataLines':graphicsDataLines, 'graphicsDataCircles':graphicsDataCircles }; 'ancfPointsSlopes' denotes 4-dimensional vector with (x/y) position and (x/y) slope coordinates in a row; 'elementLengths' is the list of curved lengths for elements between nodes (size is 1 smaller than number of nodes), 'elementCurvatures' is the list of scalar curvatures between nodes (according to list of elementLengths), 'totalLength' is the total length of the reeving line, 'circleData' represents the lines and arcs calculated for the reeving system, 'graphicsDataLines' is the graphicsData for the lines and 'graphicsDataCircles' represents the graphicsData for the circles
- | \ *example*\ :

.. code-block:: python

  #list with circle center, radius and side at which rope runs
  circleList = [[[0,0],0.2,'L'],
                [[0,1],0.2,'L'],
                [[0.8,0.8],0.4,'L'],
                [[1,0],0.2,'L'],
                [[0,0],0.2,'L'],
                [[0,1],0.2,'L'],
                ]
  [] = CreateReevingCurve(circleList,
                          removeLastLine=True, #allows closed curve
                          numberOfANCFnodes=50)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Ex), \ `contactCurveWithLongCurve.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/contactCurveWithLongCurve.py>`_\  (Ex)



----


.. _sec-beams-pointsandslopes2ancfcable2d:

Function: PointsAndSlopes2ANCFCable2D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`PointsAndSlopes2ANCFCable2D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L655>`__\ (\ ``mbs``\ , \ ``ancfPointsSlopes``\ , \ ``elementLengths``\ , \ ``cableTemplate``\ , \ ``massProportionalLoad = [0,0,0]``\ , \ ``fixedConstraintsNode0 = [0,0,0,0]``\ , \ ``fixedConstraintsNode1 = [0,0,0,0]``\ , \ ``firstNodeIsLastNode = True``\ , \ ``elementCurvatures = []``\ , \ ``graphicsSizeConstraints = -1``\ )

- | \ *function description*\ :
  | Create nodes and ANCFCable2D elements in MainSystem mbs from a given set of nodes, elements lengths and a template for the cable, based on output of function CreateReevingCurve(...); function works similar to GenerateStraightLineANCFCable2D, but for arbitrary geometry (curved elements); optionally add loads and constraints
- | \ *input*\ :
  | \ ``mbs``\ : the system where ANCF elements and nodes are added
  | \ ``ancfPointsSlopes``\ : list of position and slopes for nodes, provided as 4D numpy arrays, as returned by CreateReevingCurve(...)
  | \ ``elementLengths``\ : list of element lengths per element, as returned by CreateReevingCurve(...)
  | \ ``cableTemplate``\ : a ObjectANCFCable2D object, containing the desired cable properties; cable length and node numbers are set automatically
  | \ ``massProportionalLoad``\ : a 3D list or np.array, containing the gravity vector to be applied to all elements or zero
  | \ ``fixedConstraintsNode0``\ : a list of 4 binary values, indicating the coordinate contraints on the first node (x,y-position and x,y-slope)
  | \ ``fixedConstraintsNode1``\ : a list of 4 binary values, indicating the coordinate contraints on the last node (x,y-position and x,y-slope)
  | \ ``firstNodeIsLastNode``\ : if True, then the last node is using the node number of the first node and the curve is closed; otherwise, the first and last nodes are different, and the curve is open
  | \ ``elementCurvatures``\ : optional list of pre-curvatures of elements, used to override the cableTemplate entry 'physicsReferenceCurvature'; use 0. for straight lines!
  | \ ``graphicsSizeConstraints``\ : if set other than -1, it will be used as the size for drawing applied coordinate constraints
- | \ *output*\ :
  | returns a list [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Ex), \ `reevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystem.py>`_\  (Ex)



----


.. _sec-beams-generateslidingjoint:

Function: GenerateSlidingJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GenerateSlidingJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L740>`__\ (\ ``mbs``\ , \ ``cableObjectList``\ , \ ``markerBodyPositionOfSlidingBody``\ , \ ``localMarkerIndexOfStartCable = 0``\ , \ ``slidingCoordinateStartPosition = 0``\ )

- | \ *function description*\ :
  | generate a sliding joint from a list of cables, marker to a sliding body, etc.
- | \ *output*\ :
  | returns the sliding joint object

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFslidingAndALEjointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFslidingAndALEjointTest.py>`_\  (TM)



----


.. _sec-beams-generatealeslidingjoint:

Function: GenerateAleSlidingJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GenerateAleSlidingJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/beams.py\#L767>`__\ (\ ``mbs``\ , \ ``cableObjectList``\ , \ ``markerBodyPositionOfSlidingBody``\ , \ ``AleNode``\ , \ ``localMarkerIndexOfStartCable = 0``\ , \ ``AleSlidingOffset = 0``\ , \ ``activeConnector = True``\ , \ ``penaltyStiffness = 0``\ )

- | \ *function description*\ :
  | generate an ALE sliding joint from a list of cables, marker to a sliding body, etc.
- | \ *output*\ :
  | returns the sliding joint object

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFslidingAndALEjointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFslidingAndALEjointTest.py>`_\  (TM)

