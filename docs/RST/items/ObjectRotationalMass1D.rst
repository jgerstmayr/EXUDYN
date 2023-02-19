

.. _sec-item-objectrotationalmass1d:

ObjectRotationalMass1D
======================

A 1D rotational inertia (mass) which is attached to Node1D.
 



The item \ **ObjectRotationalMass1D**\  with type = 'RotationalMass1D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsInertia** [type = UReal, default = 0.]:
  | inertia components [SI:kgm\ :math:`^2`\ ] of rotor / rotational mass
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) of Node1D, providing rotation coordinate \ :math:`\psi_0 = c_0`\ 
* | **referencePosition** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | a constant reference position = reference point, used to assign joint constraints accordingly and for drawing
* | **referenceRotation** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | an intermediate rotation matrix, which transforms the 1D coordinate into 3D, see description



The item VObjectRotationalMass1D has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


