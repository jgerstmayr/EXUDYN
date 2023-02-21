

.. _sec-item-objectmass1d:

ObjectMass1D
============

A 1D (translational) mass which is attached to Node1D. Note, that the mass does not need to have the interpretation as a translational mass.

\ **Additional information for ObjectMass1D**\ :

* | The Object has the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested node type = \ ``GenericODE2``\ 
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




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


