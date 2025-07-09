

.. _sec-item-objectjointrevolutez:

ObjectJointRevoluteZ
====================

A revolute joint in 3D; constrains the position of two rigid body markers and the rotation about two axes, while the joint \ :math:`z`\ -rotation axis (defined in local coordinates of marker 0 / joint J0 coordinates) can freely rotate. An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddRevoluteJoint(...), Section :ref:`sec-rigidbodyutilities-addrevolutejoint`\ , for two rigid bodies (or ground). \addExampleImage{RevoluteJointZ}

\ **Additional information for ObjectJointRevoluteZ**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``RevoluteJointZ``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRevoluteJointZ``\ 


The item \ **ObjectJointRevoluteZ**\  with type = 'JointRevoluteZ' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **rotationMarker0** [\ :math:`\LU{m0,J0}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m0`\ ; translation and rotation axes for marker \ :math:`m0`\  are defined in the local body coordinate system and additionally transformed by rotationMarker0
* | **rotationMarker1** [\ :math:`\LU{m1,J1}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m1`\ ; translation and rotation axes for marker \ :math:`m1`\  are defined in the local body coordinate system and additionally transformed by rotationMarker1
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointRevoluteZ]:
  | parameters for visualization of item



The item VObjectJointRevoluteZ has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **axisRadius** [type = float, default = 0.1]:
  | radius of joint axis to draw
* | **axisLength** [type = float, default = 0.4]:
  | length of joint axis to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectjointrevolutez:

DESCRIPTION of ObjectJointRevoluteZ
-----------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
  | current global position of position marker \ :math:`m0`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
  | current global velocity of position marker \ :math:`m0`\ 
* | ``DisplacementLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
  | relative displacement in local joint0 coordinates; uses local J0 coordinates even for spherical joint configuration
* | ``VelocityLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
  | relative translational velocity in local joint0 coordinates
* | ``Rotation``\ : \ :math:`\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp`\ 
  | relative rotation parameters (Tait Bryan Rxyz); Z component represents rotation of joint, other components represent constraint drift
* | ``AngularVelocityLocal``\ : \ :math:`\LU{J0}{\Delta\tomega}`\ 
  | relative angular velocity in joint J0 coordinates, giving a vector with Z-component only
* | ``ForceLocal``\ : \ :math:`\LU{J0}{{\mathbf{f}}}`\ 
  | joint force in local \ :math:`J0`\  coordinates
* | ``TorqueLocal``\ : \ :math:`\LU{J0}{{\mathbf{m}}}`\ 
  | joint torques in local \ :math:`J0`\  coordinates; torque around Z is zero



.. _sec-objectjointrevolutez-definitionofquantities:


Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | current global position which is provided by marker m0
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0
   * - | joint J0 orientation
     - | \ :math:`\LU{0,J0}{\Rot} = \LU{0,m0}{\Rot} \LU{m0,J0}{\Rot}`\ 
     - | joint \ :math:`J0`\  rotation matrix
   * - | joint J0 orientation vectors
     - | \ :math:`\LU{0,J0}{\Rot} = [\LU{0}{{\mathbf{t}}_{x0}},\,\LU{0}{{\mathbf{t}}_{y0}},\,\LU{0}{{\mathbf{t}}_{z0}}]\tp`\ 
     - | orientation vectors (represent local \ :math:`x`\ , \ :math:`y`\ , and \ :math:`z`\  axes) in global coordinates, used for definition of constraint equations
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | accordingly
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | joint J1 orientation
     - | \ :math:`\LU{0,J1}{\Rot} = \LU{0,m1}{\Rot} \LU{m1,J1}{\Rot}`\ 
     - | joint \ :math:`J1`\  rotation matrix
   * - | joint J1 orientation vectors
     - | \ :math:`\LU{0,J1}{\Rot} = [\LU{0}{{\mathbf{t}}_{x1}},\,\LU{0}{{\mathbf{t}}_{y1}},\,\LU{0}{{\mathbf{t}}_{z1}}]\tp`\ 
     - | orientation vectors (represent local \ :math:`x`\ , \ :math:`y`\ , and \ :math:`z`\  axes) in global coordinates, used for definition of constraint equations
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | accordingly
   * - | marker m0 velocity
     - | \ :math:`\LU{b}{\tomega}_{m0}`\ 
     - | current local angular velocity vector provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{b}{\tomega}_{m1}`\ 
     - | current local angular velocity vector provided by marker m1
   * - | Displacement
     - | \ :math:`\LU{0}{\Delta{\mathbf{p}}}=\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | used, if all translational axes are constrained
   * - | Velocity
     - | \ :math:`\LU{0}{\Delta{\mathbf{v}}} = \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | used, if all translational axes are constrained (velocity level)
   * - | DisplacementLocal
     - | \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta{\mathbf{p}}}`\ 
   * - | VelocityLocal
     - | \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta{\mathbf{v}}}`\  \ :math:`\ldots`\  note that this is the global relative velocity projected into the local \ :math:`J0`\  coordinate system
   * - | AngularVelocityLocal
     - | \ :math:`\LU{J0}{\Delta\omega}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \left( \LU{0,m1}{\Rot} \LU{m1}{\omega} - \LU{0,m0}{\Rot} \LU{m0}{\omega} \right)`\ 
   * - | algebraic variables
     - | \ :math:`{\mathbf{z}}=[\lambda_0,\,\ldots,\,\lambda_5]\tp`\ 
     - | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations


Connector constraint equations
------------------------------


\ **Equations for translational part (\ ``activeConnector = True``\ )** :

The translational index 3 constraints read,

.. math::

   \LU{0}{\Delta{\mathbf{p}}} = \Null


and the translational index 2 constraints read

.. math::

   \LU{0}{\Delta {\mathbf{v}}} = \Null



\ **Equations for rotational part (\ ``activeConnector = True``\ )** :

Note that the axes are always given in global coordinates, compare the table in Section :ref:`sec-objectjointrevolutez-definitionofquantities`\ ,
and they include the transformations by \ :math:`\LU{m0,J0}{\Rot}`\  and \ :math:`\LU{m1,J1}{\Rot}`\ .
The index 3 constraint equations read

.. math::
   :label: eq-objectjointrevolutez-index3

   \LU{0}{{\mathbf{t}}}_{z0}\tp \LU{0}{{\mathbf{t}}}_{x1} &=& 0 \\
   \LU{0}{{\mathbf{t}}}_{z0}\tp \LU{0}{{\mathbf{t}}}_{y1} &=& 0


The index 2 constraints follow from the derivative of Eq. :eq:`eq-objectjointrevolutez-index3`\  w.r.t.\ time, and are given in the C++ code.
if \ ``activeConnector = False``\ , 

.. math::

   {\mathbf{z}} = \Null





.. _miniexample-objectjointrevolutez:

MINI EXAMPLE for ObjectJointRevoluteZ
-------------------------------------


.. code-block:: python
   :linenos:

   #example with rigid body at [0,0,0], with torsional load
   nBody = mbs.AddNode(RigidRxyz())
   oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                   nodeNumber=nBody))
   
   mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                           localPosition = [0,0,0]))
   mbs.AddObject(RevoluteJointZ(markerNumbers = [mGround, mBody])) #rotation around ground Z-axis
   
   #torque around z-axis; 
   mbs.AddLoad(Torque(markerNumber = mBody, loadVector=[0,0,1])) 
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic(exu.SimulationSettings())
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Rotation)[2]

Relevant Examples and TestModels with weblink:

    \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Examples/), \ `rigidBodyTutorial3withMarkers.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3withMarkers.py>`_\  (Examples/), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Examples/), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Examples/), \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Examples/), \ `multiMbsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/multiMbsTest.py>`_\  (Examples/), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Examples/), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Examples/), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Examples/), \ `rollerBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rollerBearningModel.py>`_\  (Examples/), \ `solutionViewerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/solutionViewerTest.py>`_\  (Examples/), \ `plotSensorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/plotSensorTest.py>`_\  (TestModels/), \ `revoluteJointPrismaticJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/revoluteJointPrismaticJointTest.py>`_\  (TestModels/), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


