

.. _sec-item-objectrotationalmass1d:

ObjectRotationalMass1D
======================

A 1D rotational inertia (mass) which is attached to Node1D.

\ **Additional information for ObjectRotationalMass1D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested \ ``Node``\  type = \ ``GenericODE2``\ 
* | \ **Short name**\  for Python = \ ``Rotor1D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRotor1D``\ 


The item \ **ObjectRotationalMass1D**\  with type = 'RotationalMass1D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsInertia** [\ :math:`J`\ , type = UReal, default = 0.]:
  | inertia components [SI:kgm\ :math:`^2`\ ] of rotor / rotational mass
* | **nodeNumber** [\ :math:`n0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) of Node1D, providing rotation coordinate \ :math:`\psi_0 = c_0`\ 
* | **referencePosition** [\ :math:`\LU{0}{\pRef_0}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | a constant reference position = reference point, used to assign joint constraints accordingly and for drawing
* | **referenceRotation** [\ :math:`\LU{0i}{\Rot_{0}} \in \Rcal^{3 \times 3}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | an intermediate rotation matrix, which transforms the 1D coordinate into 3D, see description
* | **visualization** [type = VObjectRotationalMass1D]:
  | parameters for visualization of item



The item VObjectRotationalMass1D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure


----------

.. _description-objectrotationalmass1d:

DESCRIPTION of ObjectRotationalMass1D
-------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig= \pRefG`\ 
  | global position vector; for interpretation see intermediate variables
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig`\ 
  | global displacement vector; for interpretation see intermediate variables
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig`\ 
  | global velocity vector; for interpretation see intermediate variables
* | ``RotationMatrix``\ : \ :math:`\LU{0b}{\Rot}`\ 
  | vector with 9 components of the rotation matrix (row-major format)
* | ``Rotation``\ : \ :math:`\theta`\ 
  | scalar rotation angle obtained from underlying node
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig`\ 
  | angular velocity of body
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig`\ 
  | local (body-fixed) 3D velocity vector of node



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | position coordinate
     - | \ :math:`{\theta_0}\cConfig = {c_0}\cConfig + {c_0}\cRef`\ 
     - | total rotation coordinate of node (e.g., Node1D) in any configuration (nodal coordinate \ :math:`c_0`\ )
   * - | displacement coordinate
     - | \ :math:`{\psi_0}\cConfig = {c_0}\cConfig`\ 
     - | change of rotation coordinate of mass node (e.g., Node1D) in any configuration (nodal coordinate \ :math:`c_0`\ )
   * - | velocity coordinate
     - | \ :math:`{\dot \psi_{0\cConfig}}`\ 
     - | rotation velocity coordinate of mass node (e.g., Node1D) in any configuration
   * - | Position
     - | \ :math:`\LU{0}{{\mathbf{p}}}\cConfig =\LU{0}{\pRef_0}`\ 
     - | constant (translational) position of mass object in any configuration
   * - | Displacement
     - | \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [0,0,0]\tp`\ 
     - | (translational) displacement of mass object in any configuration
   * - | Velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = [0,0,0]\tp`\ 
     - | (translational) velocity of mass object in any configuration
   * - | AngularVelocity
     - | \ :math:`\LU{0}{\tomega}\cConfig = \LU{0i}{\Rot_{0}} \LU{i}{\vr{0}{0}{\dot \psi_0}}\tp`\ 
     - | 
   * - | AngularVelocityLocal
     - | \ :math:`\LU{b}{\tomega}\cConfig = \LU{i}{\vr{0}{0}{\dot \psi_0}}\tp`\ 
     - | 
   * - | RotationMatrix
     - | \ :math:`\LU{0b}{\Rot} = \LU{0i}{\Rot_{0}} \LU{ib}{\mr{\cos(\theta_0)}{-\sin(\theta_0)}{0} {\sin(\theta_0)}{\cos(\theta_0)}{0} {0}{0}{1}}`\ 
     - | transformation of local body (\ :math:`b`\ ) coordinates to global (0) coordinates
   * - | residual force
     - | \ :math:`\tau`\ 
     - | residual of all forces on mass object
   * - | applied force
     - | \ :math:`\LU{0}{{\mathbf{f}}}_a = [f_0,\;f_1,\;f_2]\tp`\ 
     - | 3D applied force (loads, connectors, joint reaction forces, ...)
   * - | applied torque
     - | \ :math:`\LU{0}{\ttau}_a = [\tau_0,\;\tau_1,\;\tau_2]\tp`\ 
     - | 3D applied torque (loads, connectors, joint reaction forces, ...)

A rigid body marker (e.g., MarkerBodyRigid) may be attached to this object and forces/torques can be applied. 
However, forces will have no effect and torques will only have effect in 'direction' of the coordinate.


Equations of motion
-------------------


.. math::

   J \cdot \ddot \psi_0 = \tau.


Note that \ :math:`\tau`\  is computed from all connectors and loads upon the object. E.g., a 3D torque vector \ :math:`\LU{0}{\ttau}_a`\  is 
transformed to \ :math:`\tau`\  as

.. math::

   \tau = \LU{b}{[0,\,0,\,1]}\LU{b0}{\Rot_{0}} \LU{0}{\ttau}_a


Thus, the \ **rotation jacobian**\  reads 

.. math::

   {\mathbf{J}}_{rot} = \partial \tomega\cCur / \partial \dot q_{0,cur} = \LU{b}{[0,\,0,\,1]} \LU{b0}{\Rot_{0}}





.. _miniexample-objectrotationalmass1d:

MINI EXAMPLE for ObjectRotationalMass1D
---------------------------------------


.. code-block:: python
   :linenos:

   node = mbs.AddNode(Node1D(referenceCoordinates = [1], #\psi_0ref
                             initialCoordinates=[0.5],   #\psi_0ini
                             initialVelocities=[0.5]))   #\psi_t0ini
   rotor = mbs.AddObject(Rotor1D(nodeNumber = node, physicsInertia=1))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result, get current rotor z-rotation at local position [0,0,0]
   exudynTestGlobals.testResult = mbs.GetObjectOutputBody(rotor, exu.OutputVariableType.Rotation, [0,0,0])
   #final z-angle of rotor shall be 2

Relevant Examples and TestModels with weblink:

    \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TestModels/), \ `coordinateSpringDamperExt.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateSpringDamperExt.py>`_\  (TestModels/), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


