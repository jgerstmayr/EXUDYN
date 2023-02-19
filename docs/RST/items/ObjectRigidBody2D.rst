

.. _sec-item-objectrigidbody2d:

ObjectRigidBody2D
=================

A 2D rigid body which is attached to a rigid body 2D node. The body obtains coordinates, position, velocity, etc. from the underlying 2D node
 



The item \ **ObjectRigidBody2D**\  with type = 'RigidBody2D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [type = UReal, default = 0.]:
  | mass [SI:kg] of rigid body
* | **physicsInertia** [type = UReal, default = 0.]:
  | inertia [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. center of mass
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for 2D rigid body node



The item VObjectRigidBody2D has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


