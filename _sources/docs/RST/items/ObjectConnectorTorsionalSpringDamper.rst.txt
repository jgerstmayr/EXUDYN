

.. _sec-item-objectconnectortorsionalspringdamper:

ObjectConnectorTorsionalSpringDamper
====================================

An torsional spring-damper element acting on relative rotations around Z-axis of local joint0 coordinate system; connects to orientation-based markers; if other rotation axis than the local joint0 Z axis shall be used, the joint rotationMarker0 / rotationMarker1 may be used. The joint perfectly extends a RevoluteJoint with a spring-damper, which can also be used to represent feedback control in an elegant and efficient way, by chosing appropriate user functions. It also allows to measure continuous / infinite rotations by making use of a NodeGeneric which compensates \ :math:`\pm \pi`\  jumps in the measured rotation (OutputVariableType.Rotation).

\ **Additional information for ObjectConnectorTorsionalSpringDamper**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Orientation``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``TorsionalSpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VTorsionalSpringDamper``\ 


The item \ **ObjectConnectorTorsionalSpringDamper**\  with type = 'ConnectorTorsionalSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData with 1 dataCoordinate for continuous rotation reconstruction; if this node is left to invalid index, it will not be used
* | **stiffness** [\ :math:`k`\ , type = Real, default = 0.]:
  | torsional stiffness [SI:Nm/rad] against relative rotation
* | **damping** [\ :math:`d`\ , type = Real, default = 0.]:
  | torsional damping [SI:Nm/(rad/s)]
* | **rotationMarker0** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 0; transforms joint into marker coordinates
* | **rotationMarker1** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 1; transforms joint into marker coordinates
* | **offset** [\ :math:`\theta_\mathrm{off}`\ , type = Real, default = 0.]:
  | rotational offset considered in the spring torque calculation (this can be used as rotation control input!)
* | **velocityOffset** [\ :math:`\omega_\mathrm{off}`\ , type = Real, default = 0.]:
  | angular velocity offset considered in the damper torque calculation (this can be used as angular velocity control input!)
* | **torque** [\ :math:`\tau_c`\ , type = Real, default = 0.]:
  | additional constant torque [SI:Nm] added to spring-damper; this can be used to prescribe a torque between the two attached bodies (e.g., for actuation and control)
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springTorqueUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which computes the scalar torque between the two rigid body markers in local joint0 coordinates, if activeConnector=True; see description below
* | **visualization** [type = VObjectConnectorTorsionalSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorTorsionalSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectortorsionalspringdamper:

DESCRIPTION of ObjectConnectorTorsionalSpringDamper
---------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Rotation``\ : \ :math:`\Delta\theta`\ 
  | relative rotation around the spring-damper Z-coordinate, enhanced to a continuous rotation (infinite rotations \ :math:`>+\pi`\  and \ :math:`<-\pi`\ ) if a NodeGeneric with 1 coordinate as added
* | ``AngularVelocityLocal``\ : \ :math:`\Delta\omega`\ 
  | scalar relative angular velocity around joint0 Z-axis
* | ``TorqueLocal``\ : \ :math:`\tau_{SD}`\ 
  | scalar spring-damper torque around the local joint0 Z-axis



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | input parameter
     - | symbol
     - | description
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
   * - | nodeNumber
     - | \ :math:`n0`\ 
     - | optional node number of a generic node (otherwise exu.InvalidIndex())


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | marker m0 ang.\ velocity
     - | \ :math:`\LU{m0}{\tomega}_{m0}`\ 
     - | current local angular velocity vector provided by marker m0
   * - | marker m1 ang.\ velocity
     - | \ :math:`\LU{m1}{\tomega}_{m1}`\ 
     - | current local angular velocity vector provided by marker m1
   * - | AngularVelocityLocal
     - | \ :math:`\Delta\omega = \left( \LU{J0,m1}{\Rot} \LU{m1}{\tomega} - \LU{J0,m0}{\Rot} \LU{m0}{\tomega} \right)_Z`\ 
     - | angular velocity around joint0 Z-axis



Connector forces
----------------

If \ ``activeConnector = True``\ , the vector spring force is computed as

.. math::

   \tau_{SD} = k \left(\Delta\theta - \theta_\mathrm{off} \right) + d \left(\Delta\omega - \omega_\mathrm{off} \right) + \tau_c


if \ ``activeConnector = False``\ , \ :math:`\tau_{SD}`\  is set zero.

If the springTorqueUserFunction \ :math:`\mathrm{UF}`\  is defined and \ ``activeConnector = True``\ , 
\ :math:`\tau_{SD}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   \tau_{SD} = \mathrm{UF}(mbs, t, i_N, \Delta\theta, \Delta\omega, \mathrm{stiffness}, \mathrm{damping}, \mathrm{offset})


and \ ``iN``\  represents the itemNumber (=objectNumber).

--------

\ **Userfunction**\ : ``springTorqueUserFunction(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset)`` 


A user function, which computes the scalar torque depending on mbs, time, local quantities 
(relative rotation, relative angularVelocity), which are evaluated at current time. 
Furthermore, the user function contains object parameters (stiffness, damping, offset).
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
   * - | \ ``rotation``\ 
     - | Real
     - | \ :math:`\Delta \theta`\ 
   * - | \ ``angularVelocity``\ 
     - | Real
     - | \ :math:`\Delta \omega`\ 
   * - | \ ``stiffness``\ 
     - | Real
     - | copied from object
   * - | \ ``damping``\ 
     - | Real
     - | copied from object
   * - | \ ``offset``\ 
     - | Real
     - | copied from object
   * - | \returnValue
     - | Real
     - | computed torque


--------

\ **User function example**\ :



.. code-block:: python

    #define simple cubic force for spring-damper:
    def UFforce(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset): 
        k = stiffness #passed as list
        u = rotation
        return k*u + 0.1*k*u**3
    
    #markerNumbers and parameters taken from mini example
    mbs.AddObject(TorsionalSpringDamper(markerNumbers = [mGround, mBody], 
                                        stiffness = k, 
                                        damping = k*0.01, 
                                        offset = 0,
                                        springTorqueUserFunction = UFforce))





.. _miniexample-objectconnectortorsionalspringdamper:

MINI EXAMPLE for ObjectConnectorTorsionalSpringDamper
-----------------------------------------------------


.. code-block:: python
   :linenos:

   #example with rigid body at [0,0,0], with torsional load
   k=2e3
   nBody = mbs.AddNode(RigidRxyz())
   oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                   nodeNumber=nBody))
   
   mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                           localPosition = [0,0,0]))
   mbs.AddObject(RevoluteJointZ(markerNumbers = [mGround, mBody])) #rotation around ground Z-axis
   mbs.AddObject(TorsionalSpringDamper(markerNumbers = [mGround, mBody], 
                                       stiffness = k, damping = k*0.01, offset = 0))
   
   #torque around z-axis; expect approx. phiZ = 1/k=0.0005
   mbs.AddLoad(Torque(markerNumber = mBody, loadVector=[0,0,1])) 
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic(exu.SimulationSettings())
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Rotation)[2]

Relevant Examples and TestModels with weblink:

    \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Examples/), \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Examples/), \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Examples/), \ `ROSTurtle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSTurtle.py>`_\  (Examples/), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Examples/), \ `serialRobotTSD.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTSD.py>`_\  (Examples/), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TestModels/), \ `rotatingTableTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rotatingTableTest.py>`_\  (TestModels/), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


