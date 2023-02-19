

.. _sec-item-noderigidbody2d:

NodeRigidBody2D
===============

A 2D rigid body node for rigid bodies or beams; the node has 2 displacement degrees of freedom and one rotation coordinate (rotation around z-axis: uphi). All coordinates are ODE2, used for second order differetial equations.
 



The item \ **NodeRigidBody2D**\  with type = 'RigidBody2D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | reference coordinates (x-pos,y-pos and rotation) of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
* | **initialCoordinates** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | initial displacement coordinates and angle (relative to reference coordinates)
* | **initialVelocities** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | initial velocity coordinates



The item VNodeRigidBody2D has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


