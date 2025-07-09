

.. _sec-item-objectrigidbody2d:

ObjectRigidBody2D
=================

A 2D rigid body which is attached to a rigid body 2D node. The body obtains coordinates, position, velocity, etc. from the underlying 2D node.

\ **Additional information for ObjectRigidBody2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position2D``\  + \ ``Orientation2D``\  + \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``RigidBody2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidBody2D``\ 


The item \ **ObjectRigidBody2D**\  with type = 'RigidBody2D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of rigid body
* | **physicsInertia** [\ :math:`J`\ , type = UReal, default = 0.]:
  | inertia [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. reference point; this is equal to the center of mass, if physicsCenterOfMass = 0
* | **physicsCenterOfMass** [\ :math:`\LU{b}{{\mathbf{b}}_{COM}}`\ , type = Vector2D, size = 2, default = [0.,0.]]:
  | local position of \ :ref:`COM <COM>`\  relative to the body's reference point; if the vector of the \ :ref:`COM <COM>`\  is [0,0], the computation will not consider additional terms for the \ :ref:`COM <COM>`\  and it is faster
* | **nodeNumber** [\ :math:`n_0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for 2D rigid body node
* | **visualization** [type = VObjectRigidBody2D]:
  | parameters for visualization of item



The item VObjectRigidBody2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure


----------

.. _description-objectrigidbody2d:

DESCRIPTION of ObjectRigidBody2D
--------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\Rot}\pLocB`\ 
  | global position vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig + \LU{0b}{\Rot}\pLocB`\ 
  | global displacement vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig(\pLocB) = \LU{0}{\dot{\mathbf{u}}}\cConfig + \LU{0b}{\Rot}(\LU{b}{\tomega} \times \pLocB\cConfig)`\ 
  | global velocity vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``VelocityLocal``\ : \ :math:`\LU{b}{{\mathbf{v}}}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{{\mathbf{v}}}\cConfig(\pLocB)`\ 
  | local (body-fixed) velocity vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``RotationMatrix``\ : \ :math:`\mathrm{vec}(\LU{0b}{\Rot})=[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp`\ 
  | vector with 9 components of the rotation matrix (row-major format)
* | ``Rotation``\ : \ :math:`\theta_{0\mathrm{config}}`\ 
  | scalar rotation angle of body
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig`\ 
  | angular velocity of body
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig`\ 
  | local (body-fixed) 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig(\pLocB) = \LU{0}{\ddot{\mathbf{u}}} + \LU{0}{\talpha} \times (\LU{0b}{\Rot} \pLocB) +  \LU{0}{\tomega} \times ( \LU{0}{\tomega} \times(\LU{0b}{\Rot} \pLocB))`\ 
  | global acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AccelerationLocal``\ : \ :math:`\LU{b}{{\mathbf{a}}}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{{\mathbf{a}}}\cConfig(\pLocB)`\ 
  | local (body-fixed) acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AngularAcceleration``\ : \ :math:`\LU{0}{\talpha}\cConfig`\ 
  | angular acceleration vector of body
* | ``AngularAccelerationLocal``\ : \ :math:`\LU{b}{\talpha}\cConfig = \LU{b0}{\Rot} \LU{0}{\talpha}\cConfig`\ 
  | local angular acceleration vector of body



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | reference position
     - | \ :math:`\pRefG\cConfig + \pRefG\cRef = \LU{0}{{\mathbf{p}}}(n_0)\cConfig`\ 
     - | reference point, only equal to the position of \ :ref:`COM <COM>`\  if \ :math:`\LU{b}{{\mathbf{b}}_{COM}}=\Null`\ ; provided by node \ :math:`n_0`\  in any configuration (except reference)
   * - | reference point displacement
     - | \ :math:`\LU{0}{{\mathbf{u}}}\cConfig =\pRefG\cConfig = [q_0,\;q_1,\;0]\cConfig\tp = \LU{0}{{\mathbf{u}}}(n_0)\cConfig`\ 
     - | displacement of reference point which is provided by node \ :math:`n_0`\  in any configuration; NOTE that for configurations other than reference, it is follows that \ :math:`\pRefG\cRef - \pRefG\cConfig`\ 
   * - | reference point velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = [\dot q_0,\;\dot q_1,\;0]\cConfig\tp = \LU{0}{{\mathbf{v}}}(n_0)\cConfig`\ 
     - | velocity of reference point which is provided by node \ :math:`n_0`\  in any configuration
   * - | body rotation
     - | \ :math:`\LU{0}{\theta}_{0\mathrm{config}} = \theta_0(n_0)\cConfig = \psi_0(n_0)\cRef + \psi_0(n_0)\cConfig`\ 
     - | rotation of body as provided by node \ :math:`n_0`\  in any configuration
   * - | body rotation matrix
     - | \ :math:`\LU{0b}{\Rot}\cConfig = \LU{0b}{\Rot}(n_0)\cConfig`\ 
     - | rotation matrix which transforms local to global coordinates as given by node
   * - | local position
     - | \ :math:`\pLocB = [\LU{b}{b_0},\,\LU{b}{b_1},\,0]\tp`\ 
     - | local position as used by markers or sensors
   * - | body angular velocity
     - | \ :math:`\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0(n_0),\,0,\,0]}\cConfig\tp`\ 
     - | rotation of body as provided by node \ :math:`n_0`\  in any configuration
   * - | (generalized) coordinates
     - | \ :math:`{\mathbf{c}}\cConfig = [q_0,q_1,\;\psi_0]\tp`\ 
     - | generalized coordinates of body (= coordinates of node)
   * - | generalized forces
     - | \ :math:`\LU{0}{{\mathbf{f}}} = [f_0,\;f_1,\;\tau_2]\tp`\ 
     - | generalized forces applied to body
   * - | applied forces
     - | \ :math:`\LU{0}{{\mathbf{f}}}_a = [f_0,\;f_1,\;0]\tp`\ 
     - | applied forces (loads, connectors, joint reaction forces, ...)
   * - | applied torques
     - | \ :math:`\LU{0}{\ttau}_a = [0,\;0,\;\tau_2]\tp`\ 
     - | applied torques (loads, connectors, joint reaction forces, ...)


Equations of motion
-------------------

The equations of motion in case that \ ``physicsCenterOfMass``\ =\ :math:`\Null`\  read:

.. math::

   \mr{m}{0}{0} {0}{m}{0} {0}{0}{J} \vr{\ddot q_0}{\ddot q_1}{\ddot \psi_0} = \vr{f_0}{f_1}{\tau_2} = {\mathbf{f}}.


if \ ``physicsCenterOfMass``\  is nonzero, we resort to (not that \ :math:`J`\  represents the moment of inertia related to the reference point!):

.. math::

   \mr{m}{0}{G_x} {0}{m}{G_y} {G_x}{G_y}{J} \vr{\ddot q_0}{\ddot q_1}{\ddot \psi_0} = \vr{m \dot \psi_0^2 b_x }{m \dot \psi_0^2 b_y}{0} + \vr{f_0}{f_1}{\tau_2} = {\mathbf{f}}.


where we use the relations caused by the non-zero center of mass

.. math::

   \vp{G_x}{G_y} = m \vp{b_y}{-b_x} \quad \mathrm{and} \quad \vp{b_x}{b_y} = \LU{0}{{\mathbf{b}}_{COM}}



Position-based markers can measure position \ :math:`{\mathbf{p}}\cConfig(\pLocB)`\  depending on the local position \ :math:`\pLocB`\ . 
The \ **position jacobian**\  depends on the local position \ :math:`\pLocB`\  and is defined as,

.. math::

   \LU{0}{{\mathbf{J}}_{pos}} = \partial \LU{0}{{\mathbf{p}}}\cConfig(\pLocB)\cCur / \partial {\mathbf{c}}\cCur = \mr{1}{0}{-\sin(\theta)\LU{b}{b_0} - \cos(\theta)\LU{b}{b_1}} {0}{1}{\cos(\theta)\LU{b}{b_0}-\sin(\theta)\LU{b}{b_1}} {0}{0}{0}


which transforms the action of global forces \ :math:`\LU{0}{{\mathbf{f}}}`\  of position-based markers on the coordinates \ :math:`{\mathbf{c}}`\ ,

.. math::

   {\mathbf{Q}} = \LU{0}{{\mathbf{J}}_{pos}\tp} \LU{0}{{\mathbf{f}}}_a


Note that a LoadCoordinate on coordinate 2 of the node would add a torque \ :math:`\tau_2`\  on the RHS.
The \ **rotation jacobian**\ , which is computed from angular velocity, reads

.. math::

   \LU{0}{{\mathbf{J}}_{rot}} = \partial \LU{0}{\tomega}\cCur / \partial \dot {\mathbf{c}}\cCur = \mr{0}{0}{0} {0}{0}{0} {0}{0}{1}


and transforms the action of global torques \ :math:`\LU{0}{\ttau}`\  of orientation-based markers on the coordinates \ :math:`{\mathbf{c}}`\ ,

.. math::

   {\mathbf{Q}} = \LU{0}{{\mathbf{J}}_{rot}\tp} \, \LU{0}{\ttau}_a



--------

\ **Userfunction**\ : ``graphicsDataUserFunction(mbs, itemNumber)`` 


A user function, which is called by the visualization thread in order to draw user-defined objects.
The function can be used to generate any \ ``BodyGraphicsData``\ , see Section  :ref:`sec-graphicsdata`\ .
Use \ ``exudyn.graphics``\  functions, see Section  :ref:`sec-module-graphics`\ , to create more complicated objects. 
Note that \ ``graphicsDataUserFunction``\  needs to copy lots of data and is therefore
inefficient and only designed to enable simpler tests, but not large scale problems.

For an example for \ ``graphicsDataUserFunction``\  see ObjectGround, Section :ref:`sec-item-objectground`\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides reference to mbs, which can be used in user function to access all data of the object
   * - | \ ``itemNumber``\ 
     - | int
     - | integer number of the object in mbs, allowing easy access
   * - | \returnValue
     - | BodyGraphicsData
     - | list of \ ``GraphicsData``\  dictionaries, see Section  :ref:`sec-graphicsdata`\ 




.. _miniexample-objectrigidbody2d:

MINI EXAMPLE for ObjectRigidBody2D
----------------------------------


.. code-block:: python
   :linenos:

   node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates = [1,1,0.25*np.pi], 
                                      initialCoordinates=[0.5,0,0],
                                      initialVelocities=[0.5,0,0.75*np.pi]))
   mbs.AddObject(RigidBody2D(nodeNumber = node, physicsMass=1, physicsInertia=2))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result
   exudynTestGlobals.testResult = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
   exudynTestGlobals.testResult+= mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[2]
   #final x-coordinate of position shall be 2, angle theta shall be np.pi

Relevant Examples and TestModels with weblink:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `reevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystem.py>`_\  (Examples/), \ `reevingSystemOpen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystemOpen.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `doublePendulum2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/doublePendulum2D.py>`_\  (Examples/), \ `finiteSegmentMethod.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/finiteSegmentMethod.py>`_\  (Examples/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TestModels/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


