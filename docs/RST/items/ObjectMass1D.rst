

.. _sec-item-objectmass1d:

ObjectMass1D
============

A 1D (translational) mass which is attached to Node1D. Note, that the mass does not need to have the interpretation as a translational mass.

\ **Additional information for ObjectMass1D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested \ ``Node``\  type = \ ``GenericODE2``\ 
* | \ **Short name**\  for Python = \ ``Mass1D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VMass1D``\ 


The item \ **ObjectMass1D**\  with type = 'Mass1D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of mass
* | **nodeNumber** [\ :math:`n0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for Node1D
* | **referencePosition** [\ :math:`\LU{0}{\pRef_0}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | a reference position, used to transform the 1D coordinate to a position
* | **referenceRotation** [\ :math:`\LU{0b}{\Rot_{0}} \in \Rcal^{3 \times 3}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | the constant body rotation matrix, which transforms body-fixed (b) to global (0) coordinates
* | **visualization** [type = VObjectMass1D]:
  | parameters for visualization of item



The item VObjectMass1D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure


----------

.. _description-objectmass1d:

DESCRIPTION of ObjectMass1D
---------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig`\ 
  | global position vector; for interpretation see intermediate variables
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig`\ 
  | global displacement vector; for interpretation see intermediate variables
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig`\ 
  | global velocity vector; for interpretation see intermediate variables
* | ``RotationMatrix``\ : \ :math:`\LU{0b}{\Rot}`\ 
  | vector with 9 components of the rotation matrix (row-major format)
* | ``Rotation``\ : 
  | vector with 3 components of the Euler/Tait-Bryan angles in xyz-sequence (\ :math:`\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)`\ ), recomputed from rotation matrix \ :math:`\LU{0b}{\Rot}`\ 
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
     - | \ :math:`{p_0}\cConfig = {c_0}\cConfig + {c_0}\cRef`\ 
     - | position coordinate of node (nodal coordinate \ :math:`c_0`\ ) in any configuration
   * - | displacement coordinate
     - | \ :math:`{u_0}\cConfig = {c_0}\cConfig`\ 
     - | displacement coordinate of mass node in any configuration
   * - | velocity coordinate
     - | \ :math:`{u_0}\cConfig`\ 
     - | velocity coordinate of mass node in any configuration
   * - | Position
     - | \ :math:`\LU{0}{{\mathbf{p}}}\cConfig =\LU{0}{\pRef_0} + \LU{0b}{\Rot_{0}} \LU{b}{\vr{p_0}{0}{0}}\cConfig`\ 
     - | (translational) position of mass object in any configuration
   * - | Displacement
     - | \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = \LU{0b}{\Rot_{0}} \LU{b}{\vr{q_0}{0}{0}}\cConfig`\ 
     - | (translational) displacement of mass object in any configuration
   * - | Velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = \LU{0b}{\Rot_{0}} \LU{b}{\vr{\dot q_0}{0}{0}}\cConfig`\ 
     - | (translational) velocity of mass object in any configuration
   * - | residual force
     - | \ :math:`f`\ 
     - | residual of all forces on mass object
   * - | applied force
     - | \ :math:`\LU{0}{{\mathbf{f}}}_a = [f_0,\;f_1,\;f_2]\tp`\ 
     - | 3D applied force (loads, connectors, joint reaction forces, ...)
   * - | applied torque
     - | \ :math:`\LU{0}{\ttau}_a = [\tau_0,\;\tau_1,\;\tau_2]\tp`\ 
     - | 3D applied torque (loads, connectors, joint reaction forces, ...)

A rigid body marker (e.g., MarkerBodyRigid) may be attached to this object and forces/torques can be applied. 
However, torques will have no effect and forces will only have effect in 'direction' of the coordinate.


Equations of motion
-------------------


.. math::

   m \cdot \ddot q_0 = f.


Note that \ :math:`f`\  is computed from all connectors and loads upon the object. E.g., a 3D force vector \ :math:`\LU{0}{{\mathbf{f}}}_a`\  is 
transformed to \ :math:`f`\  as

.. math::

   f = \LU{b}{[1,\,0,\,0]} \LU{b0}{\Rot_{0}} \LU{0}{{\mathbf{f}}}_a


Thus, the \ **position jacobian**\  reads 

.. math::

   {\mathbf{J}}_{pos} = \partial {\mathbf{p}}\cCur / \partial {q_0}\cCur = \LU{b}{[1,\,0,\,0]} \LU{b0}{\Rot_{0}}





.. _miniexample-objectmass1d:

MINI EXAMPLE for ObjectMass1D
-----------------------------


.. code-block:: python
   :linenos:

   node = mbs.AddNode(Node1D(referenceCoordinates = [1], 
                             initialCoordinates=[0.5],
                             initialVelocities=[0.5]))
   mass = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=1))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result, get current mass position at local position [0,0,0]
   exudynTestGlobals.testResult = mbs.GetObjectOutputBody(mass, exu.OutputVariableType.Position, [0,0,0])[0]
   #final x-coordinate of position shall be 2

Relevant Examples and TestModels with weblink:

    \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Examples/), \ `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionTest.py>`_\  (Examples/), \ `mpi4pyExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mpi4pyExample.py>`_\  (Examples/), \ `multiprocessingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/multiprocessingTest.py>`_\  (Examples/), \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Examples/), \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Examples/), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Examples/), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


