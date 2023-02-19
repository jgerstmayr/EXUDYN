

.. _sec-item-noderigidbodyrotveclg:

NodeRigidBodyRotVecLG
=====================

A 3D rigid body node based on rotation vector and Lie group methods for rigid bodies; the node has 3 displacement coordinates and three rotation coordinates and can be used in combination with explicit Lie Group time integration methods.
 



The item \ **NodeRigidBodyRotVecLG**\  with type = 'RigidBodyRotVecLG' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 3]:
  | reference coordinates (position and rotation vector \ :math:`\tnu`\ ) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)
* | **initialCoordinates** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 3]:
  | initial displacement coordinates \ :math:`{\mathbf{u}}`\  and rotation vector \ :math:`\tnu`\  relative to reference coordinates
* | **initialVelocities** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 3]:
  | initial velocity coordinate: time derivatives of displacement and angular velocity vector



The item VNodeRigidBodyRotVecLG has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


