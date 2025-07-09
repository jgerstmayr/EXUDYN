

.. _sec-item-objectconnectorlinearspringdamper:

ObjectConnectorLinearSpringDamper
=================================

An linear spring-damper element acting on relative translations along given axis of local joint0 coordinate system; connects to position and orientation-based markers; the linear spring-damper is intended to act within prismatic joints or in situations where only one translational axis is free; if the two markers rotate relative to each other, the spring-damper will always act in the local joint0 coordinate system.

\ **Additional information for ObjectConnectorLinearSpringDamper**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``LinearSpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VLinearSpringDamper``\ 


The item \ **ObjectConnectorLinearSpringDamper**\  with type = 'ConnectorLinearSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,\, m1]`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **stiffness** [\ :math:`k`\ , type = Real, default = 0.]:
  | torsional stiffness [SI:Nm/rad] against relative rotation
* | **damping** [\ :math:`d`\ , type = Real, default = 0.]:
  | torsional damping [SI:Nm/(rad/s)]
* | **axisMarker0** [\ :math:`\LU{m0}{{\mathbf{d}}}`\ , type = Vector3D, default = [1,0,0]]:
  | local axis of spring-damper in marker 0 coordinates; this axis will co-move with marker \ :math:`m0`\ ; if marker m0 is attached to ground, the spring-damper represents linear equations
* | **offset** [\ :math:`x_\mathrm{off}`\ , type = Real, default = 0.]:
  | translational offset considered in the spring force calculation (this can be used as position control input!)
* | **velocityOffset** [\ :math:`v_\mathrm{off}`\ , type = Real, default = 0.]:
  | velocity offset considered in the damper force calculation (this can be used as velocity control input!)
* | **force** [\ :math:`f_c`\ , type = Real, default = 0.]:
  | additional constant force [SI:Nm] added to spring-damper; this can be used to prescribe a force between the two attached bodies (e.g., for actuation and control)
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which computes the scalar force between the two rigid body markers along axisMarker0 in \ :math:`m0`\  coordinates, if activeConnector=True; see description below
* | **visualization** [type = VObjectConnectorLinearSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorLinearSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **drawAsCylinder** [type = Bool, default = False]:
  | if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorlinearspringdamper:

DESCRIPTION of ObjectConnectorLinearSpringDamper
------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``DisplacementLocal``\ : \ :math:`\Delta x`\ 
  | (scalar) relative displacement of the spring-damper
* | ``VelocityLocal``\ : \ :math:`\Delta v`\ 
  | (scalar) relative velocity of spring-damper
* | ``ForceLocal``\ : \ :math:`f_{SD}`\ 
  | (scalar) spring-damper force



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | input parameter
     - | symbol
     - | description
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
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}_{m0}}`\ 
     - | current position matrix provided by marker m0
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}_{m1}}`\ 
     - | current position matrix provided by marker m1
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity vector provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | current global velocity vector provided by marker m1
   * - | relative displacement
     - | \ :math:`\Delta x = (\LU{0,m0}{\Rot} \LU{m0}{{\mathbf{d}}})\tp (\LU{0}{{\mathbf{p}}_{m1}} - \LU{0}{{\mathbf{p}}_{m0}})`\ 
     - | scalar relative displacement
   * - | relative velocity
     - | \ :math:`\Delta v = (\LU{0,m0}{\Rot} \LU{m0}{{\mathbf{d}}})\tp (\LU{0}{{\mathbf{v}}_{m1}} - \LU{0}{{\mathbf{v}}_{m0}})`\ 
     - | scalar relative velocity; note that this only corresponds to the time derivative of \ :math:`\Delta x`\  if the markers only move along the axis (in a prismatic joint)



Connector forces
----------------

If \ ``activeConnector = True``\ , the vector spring force is computed as

.. math::

   f_{SD} = k \left(\Delta x - x_\mathrm{off} \right) + d \left(\Delta v - v_\mathrm{off} \right) + f_c


if \ ``activeConnector = False``\ , \ :math:`f_{SD}`\  is set zero.

If the springForceUserFunction \ :math:`\mathrm{UF}`\  is defined and \ ``activeConnector = True``\ , 
\ :math:`f_{SD}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   f_{SD} = \mathrm{UF}(mbs, t, i_N, \Delta x, \Delta v, \mathrm{stiffness}, \mathrm{damping}, \mathrm{offset})


and \ ``iN``\  represents the itemNumber (=objectNumber).

--------

\ **Userfunction**\ : ``springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)`` 


A user function, which computes the scalar torque depending on mbs, time, local quantities 
(relative displacement, relative velocity), which are evaluated at current time. 
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
   * - | \ ``displacement``\ 
     - | Real
     - | \ :math:`\Delta x`\ 
   * - | \ ``velocity``\ 
     - | Real
     - | \ :math:`\Delta v`\ 
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
     - | computed force


--------

\ **User function example**\ :



.. code-block:: python

    #define simple cubic force for spring-damper:
    def UFforce(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset): 
        k = stiffness #passed as list
        return k*displacement + 0.1*k* displacement**3
    
    #markerNumbers and parameters taken from mini example
    mbs.AddObject(LinearSpringDamper(markerNumbers = [mGround, mBody], 
                                     stiffness = k, 
                                     damping = k*0.01, 
                                     offset = 0,
                                     springForceUserFunction = UFforce))





.. _miniexample-objectconnectorlinearspringdamper:

MINI EXAMPLE for ObjectConnectorLinearSpringDamper
--------------------------------------------------


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
   mbs.AddObject(PrismaticJointX(markerNumbers = [mGround, mBody])) #motion along ground X-axis
   mbs.AddObject(LinearSpringDamper(markerNumbers = [mGround, mBody], axisMarker0=[1,0,0],
                                    stiffness = k, damping = k*0.01, offset = 0))
   
   #force along x-axis; expect approx. Delta x = 1/k=0.0005
   mbs.AddLoad(Force(markerNumber = mBody, loadVector=[1,0,0])) 
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic(exu.SimulationSettings())
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Displacement)[0]

Relevant Examples and TestModels with weblink:

    \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Examples/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


