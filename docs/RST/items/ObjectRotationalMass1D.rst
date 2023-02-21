

.. _sec-item-objectrotationalmass1d:

ObjectRotationalMass1D
======================

A 1D rotational inertia (mass) which is attached to Node1D.

\ **Additional information for ObjectRotationalMass1D**\ :

* | The Object has the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested node type = \ ``GenericODE2``\ 
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



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig= pRefG`\ 
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




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


