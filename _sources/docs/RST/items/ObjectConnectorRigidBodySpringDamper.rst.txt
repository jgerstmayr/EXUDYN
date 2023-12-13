

.. _sec-item-objectconnectorrigidbodyspringdamper:

ObjectConnectorRigidBodySpringDamper
====================================

An 3D spring-damper element acting on relative displacements and relative rotations of two rigid body (position+orientation) markers; connects to (position+orientation)-based markers and represents a penalty-based rigid joint (or prismatic, revolute, etc.)

\ **Additional information for ObjectConnectorRigidBodySpringDamper**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``RigidBodySpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidBodySpringDamper``\ 


The item \ **ObjectConnectorRigidBodySpringDamper**\  with type = 'ConnectorRigidBodySpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData (size depends on application) for dataCoordinates for user functions (e.g., implementing contact/friction user function)
* | **stiffness** [type = Matrix6D, default = np.zeros((6,6))]:
  | stiffness [SI:N/m or Nm/rad] of translational, torsional and coupled springs; act against relative displacements in x, y, and z-direction as well as the relative angles (calculated as Euler angles); in the simplest case, the first 3 diagonal values correspond to the local stiffness in x,y,z direction and the last 3 diagonal values correspond to the rotational stiffness around x,y and z axis
* | **damping** [type = Matrix6D, default = np.zeros((6,6))]:
  | damping [SI:N/(m/s) or Nm/(rad/s)] of translational, torsional and coupled dampers; very similar to stiffness, however, the rotational velocity is computed from the angular velocity vector
* | **rotationMarker0** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 0; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker0
* | **rotationMarker1** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 1; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker1
* | **offset** [type = Vector6D, default = [0.,0.,0.,0.,0.,0.]]:
  | translational and rotational offset considered in the spring force calculation
* | **intrinsicFormulation** [type = Bool, default = False]:
  | if True, the joint uses the intrinsic formulation, which is independent on order of markers, using a mid-point and mid-rotation for evaluation and application of connector forces and torques; this uses a Lie group formulation; in this case, the force/torque vector is computed from the stiffness matrix times the 6-vector of the SE3 matrix logarithm between the two marker positions/rotations, see the equations
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceTorqueUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^6`\ , type = PyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D, default =  0]:
  | A Python function which computes the 6D force-torque vector (3D force + 3D torque) between the two rigid body markers, if activeConnector=True; see description below
* | **postNewtonStepUserFunction** [\ :math:`\mathrm{UF}_{PN} \in \Rcal`\ , type = PyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D, default =  0]:
  | A Python function which computes the error of the PostNewtonStep; see description below
* | **visualization** [type = VObjectConnectorRigidBodySpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorRigidBodySpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorrigidbodyspringdamper:

DESCRIPTION of ObjectConnectorRigidBodySpringDamper
---------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``DisplacementLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
  | relative displacement in local joint0 coordinates
* | ``VelocityLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
  | relative translational velocity in local joint0 coordinates
* | ``Rotation``\ : \ :math:`\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp`\ 
  | relative rotation parameters (Tait Bryan Rxyz); these are the angles used for calculation of joint torques (e.g. if cX is the diagonal rotational stiffness, the moment for axis X reads mX=cX*phiX, etc.)
* | ``AngularVelocityLocal``\ : \ :math:`\LU{J0}{\Delta\tomega}`\ 
  | relative angular velocity in local joint0 coordinates
* | ``ForceLocal``\ : \ :math:`\LU{J0}{{\mathbf{f}}}`\ 
  | joint force in local joint0 coordinates
* | ``TorqueLocal``\ : \ :math:`\LU{J0}{{\mathbf{m}}}`\ 
  | joint torque in in local joint0 coordinates



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | input parameter
     - | symbol
     - | description
   * - | stiffness
     - | \ :math:`{\mathbf{k}} \in \mathbb{R}^{6\times 6}`\ 
     - | stiffness in \ :math:`J0`\  coordinates
   * - | damping
     - | \ :math:`{\mathbf{d}} \in \mathbb{R}^{6\times 6}`\ 
     - | damping in \ :math:`J0`\  coordinates
   * - | offset
     - | \ :math:`\LUR{J0}{{\mathbf{v}}}{\mathrm{off}} \in \mathbb{R}^{6}`\ 
     - | offset in \ :math:`J0`\  coordinates
   * - | rotationMarker0
     - | \ :math:`\LU{m0,J0}{\Rot}`\ 
     - | rotation matrix which transforms from joint 0 into marker 0 coordinates
   * - | rotationMarker1
     - | \ :math:`\LU{m1,J1}{\Rot}`\ 
     - | rotation matrix which transforms from joint 1 into marker 1 coordinates
   * - | markerNumbers[0]
     - | \ :math:`m0`\ 
     - | global marker number m0
   * - | markerNumbers[1]
     - | \ :math:`m1`\ 
     - | global marker number m1


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
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | accordingly
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | accordingly
   * - | marker m0 velocity
     - | \ :math:`\LU{m0}{\tomega}_{m0}`\ 
     - | current local angular velocity vector provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{m1}{\tomega}_{m1}`\ 
     - | current local angular velocity vector provided by marker m1
   * - | Displacement
     - | \ :math:`\LU{0}{\Delta{\mathbf{p}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
   * - | Velocity
     - | \ :math:`\LU{0}{\Delta{\mathbf{v}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
   * - | DisplacementLocal
     - | \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta{\mathbf{p}}}`\ 
   * - | VelocityLocal
     - | \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta{\mathbf{v}}}`\ 
   * - | AngularVelocityLocal
     - | \ :math:`\LU{J0}{\Delta\tomega}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \left( \LU{0,m1}{\Rot} \LU{m1}{\tomega} - \LU{0,m0}{\Rot} \LU{m0}{\tomega} \right)`\ 



Connector forces
----------------

If \ ``activeConnector = True``\ , the vector spring force is computed as

.. math::

   \vp{\LU{J0}{{\mathbf{f}}_{SD}}}{\LU{J0}{{\mathbf{m}}_{SD}}} = {\mathbf{k}} \left( \vp{\LU{J0}{\Delta{\mathbf{p}}}}{\LU{J0}{\ttheta}} - \LUR{J0}{{\mathbf{v}}}{\mathrm{off}}\right) + {\mathbf{d}} \vp{\LU{J0}{\Delta{\mathbf{v}}}}{\LU{J0}{\Delta\omega}}


For the application of joint forces to markers, \ :math:`[\LU{J0}{{\mathbf{f}}_{SD}},\,\LU{J0}{{\mathbf{m}}_{SD}}]\tp`\  is transformed into global coordinates.
if \ ``activeConnector = False``\ , \ :math:`\LU{J0}{{\mathbf{f}}_{SD}}`\  and  \ :math:`\LU{J0}{{\mathbf{m}}_{SD}}`\  are set to zero.

If the springForceTorqueUserFunction \ :math:`\mathrm{UF}`\  is defined and \ ``activeConnector = True``\ , 
\ :math:`{\mathbf{f}}_{SD}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   {\mathbf{f}}_{SD} = \mathrm{UF}(mbs, t, i_N, \LU{J0}{\Delta{\mathbf{p}}}, \LU{J0}{\ttheta}, \LU{J0}{\Delta{\mathbf{v}}}, \LU{J0}{\Delta\tomega}, \mathrm{stiffness}, \mathrm{damping}, \mathrm{rotationMarker0}, \mathrm{rotationMarker1}, \mathrm{offset})


and \ ``iN``\  represents the itemNumber (=objectNumber).

--------

\ **Userfunction**\ : ``springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)`` 


A user function, which computes the 6D spring-damper force-torque vector depending on mbs, time, local quantities 
(displacement, rotation, velocity, angularVelocity, stiffness), which are evaluated at current time, which are relative quantities between 
both markers and which are defined in joint J0 coordinates. 
As relative rotations are defined by Tait-Bryan rotation parameters, it is recommended to use this connector for small relative rotations only 
(except for rotations about one axis).
Furthermore, the user function contains object parameters (stiffness, damping, rotationMarker0/1, offset).
Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

Detailed description of the arguments and local quantities:

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs in which underlying item is defined
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``displacement``\ 
     - | Vector3D
     - | \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
   * - | \ ``rotation``\ 
     - | Vector3D
     - | \ :math:`\LU{J0}{\ttheta}`\ 
   * - | \ ``velocity``\ 
     - | Vector3D
     - | \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
   * - | \ ``angularVelocity``\ 
     - | Vector3D
     - | \ :math:`\LU{J0}{\Delta\tomega}`\ 
   * - | \ ``stiffness``\ 
     - | Vector6D
     - | copied from object
   * - | \ ``damping``\ 
     - | Vector6D
     - | copied from object
   * - | \ ``rotJ0``\ 
     - | Matrix3D
     - | rotationMarker0 copied from object
   * - | \ ``rotJ1``\ 
     - | Matrix3D
     - | rotationMarker1 copied from object
   * - | \ ``offset``\ 
     - | Vector6D
     - | copied from object
   * - | \returnValue
     - | Vector6D
     - | list or numpy array of computed spring force-torque


--------

\ **Userfunction**\ : ``postNewtonStepUserFunction(mbs, t, Index itemIndex, dataCoordinates, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)`` 


A user function which computes the error of the PostNewtonStep \ :math:`\varepsilon_{PN}`\ , a recommended for stepsize reduction \ :math:`t_{recom}`\  (use values > 0 to recommend step size or values < 0 else; 0 gives minimum step size) 
and the updated dataCoordinates \ :math:`{\mathbf{d}}^k`\  of \ ``NodeGenericData``\  \ :math:`n_d`\ .
Except from \ ``dataCoordinates``\ , the arguments are the same as in \ ``springForceTorqueUserFunction``\ .
The \ ``postNewtonStepUserFunction``\  should be used together with the dataCoordinates in order to implement a active set or switching strategy
for discontinuous events, such as in contact, friction, plasticity, fracture or similar.

Detailed description of the arguments and local quantities:

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs in which underlying item is defined
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``dataCoordinates``\ 
     - | Vector
     - | \ :math:`{\mathbf{d}}^{k-1} = [d_0^{k-1},\; d_1^{k-1},\; \ldots]`\  for previous post Newton step \ :math:`k-1`\ 
   * - | ...
     - | ...
     - | other arguements see \ ``springForceTorqueUserFunction``\ 
   * - | \returnValue
     - | Vector
     - | \ :math:`\left[\varepsilon_{PN},\; t_{recom},\; d_0^{k},\; d_1^{k}, ...\right]`\  where \ :math:`k`\  indicates the current step


--------

\ **User function example**\ :



.. code-block:: python

    #define simple force for spring-damper:
    def UFforce(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, 
                stiffness, damping, rotJ0, rotJ1, offset): 
        k = stiffness #passed as list
        u = displacement
        return [u[0]*k[0][0],u[1]*k[1][1],u[2]*k[2][2], 0,0,0]
    
    #markerNumbers and parameters taken from mini example
    mbs.AddObject(RigidBodySpringDamper(markerNumbers = [mGround, mBody], 
                                        stiffness = np.diag([k,k,k, 0,0,0]), 
                                        damping = np.diag([0,k*0.01,0, 0,0,0]), 
                                        offset = [0,0,0, 0,0,0],
                                        springForceTorqueUserFunction = UFforce))





.. _miniexample-objectconnectorrigidbodyspringdamper:

MINI EXAMPLE for ObjectConnectorRigidBodySpringDamper
-----------------------------------------------------


.. code-block:: python
   :linenos:

   #example with rigid body at [0,0,0], 1kg under initial velocity
   k=500
   nBody = mbs.AddNode(RigidRxyz(initialVelocities=[0,1e3,0, 0,0,0]))
   oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                   nodeNumber=nBody))
   
   mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                           localPosition = [0,0,0]))
   mbs.AddObject(RigidBodySpringDamper(markerNumbers = [mGround, mBody], 
                                       stiffness = np.diag([k,k,k, 0,0,0]), 
                                       damping = np.diag([0,k*0.01,0, 0,0,0]), 
                                       offset = [0,0,0, 0,0,0]))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic(exu.SimulationSettings())
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Displacement)[1] 

Relevant Examples and TestModels with weblink:

    \ `ROSMassPoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMassPoint.py>`_\  (Examples/), \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Examples/), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Examples/), \ `connectorRigidBodySpringDamperTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/connectorRigidBodySpringDamperTest.py>`_\  (TestModels/), \ `rotatingTableTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rotatingTableTest.py>`_\  (TestModels/), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TestModels/), \ `superElementRigidJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/superElementRigidJointTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


